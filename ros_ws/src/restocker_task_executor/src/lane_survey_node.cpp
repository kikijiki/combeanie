// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Look down one lane: aim, then measure.
//
// The behaviour owns nothing new. Aiming is a ViewpointSurvey to the lane's own station, which is
// the viewpoint survey primitive unchanged and under the same planning-scene authority gate;
// measuring is one AcquireLaneObservation call against the node that owns the depth stream. What
// this file contributes is the join between them, and the join is where the only interesting
// decision is: which frame is allowed to be the measurement.
//
// The rule is that the acquisition may only use a frame stamped after the arm stopped, and the
// instant it stopped is read from this node's clock the moment the motion completed rather than
// inferred from a duration. Measured cost: after the controllers report
// success, the first frame stamped arrives 0.040 s of simulated time later and the third 0.640 s
// later. The dwell below is a budget over that, not a rate.

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/survey_lane.hpp>
#include <restocker_interfaces/srv/acquire_lane_observation.hpp>
#include <restocker_perception/survey_stations.hpp>
#include <restocker_perception/tf_viewpoint_port.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_task_executor/moveit_motion_port.hpp"
#include "restocker_task_executor/survey_submit_refusal.hpp"
#include "restocker_task_executor/viewpoint_survey.hpp"

namespace restocker_task_executor
{
namespace
{

using SurveyLane = restocker_interfaces::action::SurveyLane;
using AcquireLaneObservation = restocker_interfaces::srv::AcquireLaneObservation;
using GoalHandle = rclcpp_action::ServerGoalHandle<SurveyLane>;

[[nodiscard]] geometry_msgs::msg::PoseStamped stamped(
  const std::string & frame_id, const rclcpp::Time & stamp, const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.pose = tf2::toMsg(pose);
  return message;
}

[[nodiscard]] double rotation_error(const Eigen::Isometry3d & left, const Eigen::Isometry3d & right)
{
  const Eigen::AngleAxisd difference(left.linear().transpose() * right.linear());
  return std::abs(difference.angle());
}

class LaneSurveyNode
{
public:
  explicit LaneSurveyNode(const rclcpp::NodeOptions & options)
  : node_(std::make_shared<rclcpp::Node>("lane_survey", options))
  {
    planning_frame_ = declare("planning_frame", std::string("world"));
    shelf_frame_ = declare("shelf_frame", std::string(restocker_perception::kShelfFrame));
    const std::string geometry_path = declare("workcell_geometry_path", std::string());
    const std::string acquire_service =
      declare("acquire_service", std::string("/perception/acquire_lane_observation"));
    const bool require_scene_authority = declare("require_planning_scene_authority", true);
    if (geometry_path.empty()) {
      throw std::invalid_argument(
              "workcell_geometry_path must name the surveyed workcell geometry: a lane survey "
              "aims at a station derived from it");
    }
    stations_ = restocker_perception::nominal_survey_stations(
      restocker_perception::load_workcell_survey_geometry(geometry_path));

    // How long after the controllers report success before a frame may be measured. Not a
    // settling time for the arm: the acquisition takes the first frame stamped after this
    // instant, so the frame is rendered with the arm already stopped.
    // The renderer measured 0.040 s to the first frame; 0.150 s is that with room.
    dwell_ = std::chrono::milliseconds(declare("acquisition_dwell_ms", static_cast<int>(150)));
    acquisition_timeout_ =
      std::chrono::milliseconds(declare("acquisition_timeout_ms", static_cast<int>(3000)));
    config_.position_tolerance_m = declare("position_tolerance_m", 0.006);
    config_.orientation_tolerance_rad = declare("orientation_tolerance_rad", 0.010);
    config_.velocity_scaling = declare("velocity_scaling", 0.2);
    config_.acceleration_scaling = declare("acceleration_scaling", 0.2);
    config_.planning_time =
      std::chrono::milliseconds(declare("planning_time_ms", static_cast<int>(5000)));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

    rclcpp::NodeOptions port_options;
    port_options.automatically_declare_parameters_from_overrides(true);
    port_options.parameter_overrides(node_->get_node_options().parameter_overrides());
    viewpoints_ = std::make_unique<restocker_perception::TfViewpointPort>(
      port_options, restocker_perception::TfViewpointPortConfig{}, "lane_survey_tf_client");
    MoveItMotionPortConfig motion_config;
    motion_config.planning_frame = planning_frame_;
    motion_config.require_planning_scene_authority = require_scene_authority;
    motion_ = std::make_unique<MoveItMotionPort>(
      port_options, motion_config, "lane_survey_motion_client");
    survey_ = std::make_unique<ViewpointSurvey>(*viewpoints_, *motion_, config_);

    // Its own group, because the action's execute callback calls this service and waits.
    acquisition_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    acquire_ = node_->create_client<AcquireLaneObservation>(
      acquire_service, rclcpp::ServicesQoS(), acquisition_group_);

    server_ = rclcpp_action::create_server<SurveyLane>(
      node_, "survey_lane",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SurveyLane::Goal> goal) {
        return handle_goal(*goal);
      },
      [this](const std::shared_ptr<GoalHandle> &) {
        survey_->cancel();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {start(handle);});
    RCLCPP_INFO(
      node_->get_logger(), "lane survey server ready on /survey_lane, acquiring through %s",
      acquire_service.c_str());
  }

  ~LaneSurveyNode()
  {
    // Before anything that could hold the goal handle is destroyed. `~ServerGoalHandle` publishes
    // a cancellation for a goal that never reached a terminal state, and by teardown the context
    // is already shut down, so that publish throws from a destructor and terminates the process
    // (seen as SIGABRT at the end of a run with a survey in flight). `abort` marks the goal
    // terminal before it publishes, so the destructor has nothing left to do and any remaining
    // throw is ours to catch.
    terminate_outstanding_goal();
    survey_.reset();
    if (motion_) {
      motion_->shutdown();
    }
    if (viewpoints_) {
      viewpoints_->shutdown();
    }
  }

  LaneSurveyNode(const LaneSurveyNode &) = delete;
  LaneSurveyNode & operator=(const LaneSurveyNode &) = delete;
  LaneSurveyNode(LaneSurveyNode &&) = delete;
  LaneSurveyNode & operator=(LaneSurveyNode &&) = delete;

  [[nodiscard]] rclcpp::Node::SharedPtr node() const {return node_;}

private:
  template<typename Value>
  [[nodiscard]] Value declare(const std::string & name, const Value & fallback)
  {
    return node_->has_parameter(name) ? node_->get_parameter(name).get_value<Value>() :
           node_->declare_parameter<Value>(name, fallback);
  }

  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(const SurveyLane::Goal & goal) const
  {
    if (goal.lane_id.empty()) {
      RCLCPP_WARN(node_->get_logger(), "rejecting a lane survey goal that names no lane");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::scoped_lock lock(mutex_);
    if (busy_) {
      RCLCPP_WARN(node_->get_logger(), "rejecting a lane survey while another is outstanding");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void terminate_outstanding_goal() noexcept
  {
    std::shared_ptr<GoalHandle> handle;
    {
      std::scoped_lock lock(mutex_);
      handle = outstanding_.lock();
      outstanding_.reset();
      busy_ = false;
    }
    if (!handle) {
      return;
    }
    try {
      auto result = std::make_shared<SurveyLane::Result>();
      result->lane_id = handle->get_goal()->lane_id;
      result->outcome = SurveyLane::Result::OUTCOME_UNAVAILABLE;
      result->detail = "the lane survey node shut down before this survey completed";
      // The goal was in flight: neither no-command nor a terminal stop can be claimed
      // (Milestone 10 §6, Card 052 — both flags false is the fail-closed evidence state).
      result->motion_definitely_not_started = false;
      result->execution_reached_terminal_stop = false;
      handle->abort(result);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "outstanding lane survey could not be aborted at shutdown: %s",
        error.what());
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "outstanding lane survey could not be aborted at shutdown");
    }
  }

  void start(const std::shared_ptr<GoalHandle> & handle)
  {
    {
      std::scoped_lock lock(mutex_);
      busy_ = true;
      outstanding_ = handle;
    }
    const SurveyLane::Goal goal = *handle->get_goal();
    auto result = std::make_shared<SurveyLane::Result>();
    result->lane_id = goal.lane_id;
    // Pre-motion default: every exit before the aim completion proves no joint command was
    // issued for this attempt (Milestone 10 §6, Card 052). on_aimed replaces both flags from
    // the aim's own completion.
    result->motion_definitely_not_started = true;
    result->execution_reached_terminal_stop = false;

    const restocker_perception::SurveyStation * station =
      restocker_perception::find_survey_station(stations_, goal.lane_id);
    if (station == nullptr) {
      result->outcome = SurveyLane::Result::OUTCOME_UNKNOWN_LANE;
      result->detail = "no nominal survey station named " + goal.lane_id;
      finish(handle, result, false);
      return;
    }

    if (goal.measure_only) {
      // Measure from where the arm already is. The viewpoint half is skipped and no achieved
      // pose is reported, since nothing was commanded.
      acquire_and_finish(handle, result, goal.lane_id);
      return;
    }

    auto feedback = std::make_shared<SurveyLane::Feedback>();
    feedback->phase = SurveyLane::Feedback::PHASE_AIMING;
    handle->publish_feedback(feedback);

    geometry_msgs::msg::TransformStamped message;
    try {
      message = tf_buffer_->lookupTransform(
        planning_frame_, shelf_frame_, tf2::TimePointZero, tf2::durationFromSec(5.0));
    } catch (const tf2::TransformException & error) {
      result->outcome = SurveyLane::Result::OUTCOME_INVALID_REQUEST;
      result->detail = "could not resolve " + planning_frame_ + " <- " + shelf_frame_ + ": " +
        error.what();
      finish(handle, result, false);
      return;
    }
    const restocker_perception::Result<restocker_perception::FramedTransform> transform =
      restocker_perception::FramedTransform::create(
      shelf_frame_, planning_frame_, tf2::transformToEigen(message));
    if (!transform) {
      result->outcome = SurveyLane::Result::OUTCOME_INVALID_REQUEST;
      result->detail = transform.error().detail;
      finish(handle, result, false);
      return;
    }
    restocker_perception::Result<restocker_perception::CameraViewpoint> viewpoint =
      restocker_perception::station_viewpoint(*station, transform.value());
    if (!viewpoint) {
      result->outcome = SurveyLane::Result::OUTCOME_INVALID_REQUEST;
      result->detail = viewpoint.error().detail;
      finish(handle, result, false);
      return;
    }

    ViewpointSurveyConfig overrides;
    overrides.position_tolerance_m = goal.position_tolerance_m;
    overrides.orientation_tolerance_rad = goal.orientation_tolerance_rad;
    overrides.velocity_scaling = 0.0;
    overrides.acceleration_scaling = 0.0;
    overrides.planning_time = std::chrono::milliseconds(0);

    const restocker_perception::CameraViewpoint commanded = viewpoint.value();
    const SurveySubmitResult submitted = survey_->submit(
      OperationCorrelation{0U, ++operation_generation_}, viewpoint.value(), overrides,
      [this, handle, result, commanded](SurveyCompletion completion) {
        on_aimed(handle, result, commanded, std::move(completion));
      });
    if (submitted) {
      return;
    }
    apply_lane_submit_refusal(submitted.status, *result);
    result->detail = submitted.detail;
    finish(handle, result, false);
  }

  void on_aimed(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<SurveyLane::Result> & result,
    const restocker_perception::CameraViewpoint & commanded, SurveyCompletion completion)
  {
    // The instant the motion completed. Everything the acquisition is allowed to measure is
    // stamped after this, which is what makes the frame one the stopped arm produced.
    const rclcpp::Time stopped = node_->now();

    // First-hand motion evidence for the recovery classification (Milestone 10 §6, Card 052):
    // the aim's own outcome decides whether a joint command could have been issued, and a
    // stop is established by the backend's terminal-stop flag or by arrival itself — the
    // controllers reporting success is the verified stop for this attempt, so a later
    // acquisition failure still carries it.
    result->motion_definitely_not_started = survey_definitely_not_started(completion.outcome);
    result->execution_reached_terminal_stop = completion.execution_reached_terminal_stop ||
      completion.outcome == SurveyOutcome::kArrived;

    if (completion.outcome != SurveyOutcome::kArrived) {
      result->outcome = completion.outcome == SurveyOutcome::kCanceled ?
        SurveyLane::Result::OUTCOME_CANCELED :
        SurveyLane::Result::OUTCOME_VIEWPOINT_UNREACHED;
      result->detail = std::string(survey_outcome_name(completion.outcome)) + ": " +
        completion.detail;
      RCLCPP_WARN(
        node_->get_logger(), "lane survey of %s did not reach its viewpoint: %s",
        result->lane_id.c_str(), result->detail.c_str());
      finish(handle, result, completion.outcome == SurveyOutcome::kCanceled);
      return;
    }

    // Where the camera actually ended up, straight out of TF. The measurement uses the transform
    // at the frame's own stamp rather than this one; this is reported so a bad measurement can be
    // told apart from a bad aim.
    try {
      const geometry_msgs::msg::TransformStamped achieved = tf_buffer_->lookupTransform(
        planning_frame_, restocker_perception::kWristCameraOpticalFrame, tf2::TimePointZero,
        tf2::durationFromSec(2.0));
      const Eigen::Isometry3d pose = tf2::transformToEigen(achieved);
      result->viewpoint_measured = true;
      result->achieved_camera_optical_pose =
        stamped(planning_frame_, achieved.header.stamp, pose);
      result->achieved_translation_error_m =
        (pose.translation() - commanded.pose.pose.translation()).norm();
      result->achieved_rotation_error_rad = rotation_error(pose, commanded.pose.pose);
    } catch (const tf2::TransformException & error) {
      result->viewpoint_measured = false;
    }

    auto feedback = std::make_shared<SurveyLane::Feedback>();
    feedback->phase = SurveyLane::Feedback::PHASE_ACQUIRING;
    handle->publish_feedback(feedback);
    acquire_and_finish(handle, result, result->lane_id, stopped + rclcpp::Duration(dwell_));
  }

  void acquire_and_finish(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<SurveyLane::Result> & result, const std::string & lane_id,
    std::optional<rclcpp::Time> not_before = std::nullopt)
  {
    if (!acquire_->wait_for_service(std::chrono::seconds(2))) {
      result->outcome = SurveyLane::Result::OUTCOME_UNAVAILABLE;
      result->detail = "the lane acquisition service is not available";
      RCLCPP_ERROR(node_->get_logger(), "%s", result->detail.c_str());
      finish(handle, result, false);
      return;
    }
    auto request = std::make_shared<AcquireLaneObservation::Request>();
    request->lane_id = lane_id;
    if (not_before) {
      request->not_before = *not_before;
    }
    request->timeout = rclcpp::Duration(acquisition_timeout_);

    auto future = acquire_->async_send_request(request);
    if (future.wait_for(acquisition_timeout_ + std::chrono::seconds(3)) !=
      std::future_status::ready)
    {
      result->outcome = SurveyLane::Result::OUTCOME_ACQUISITION_FAILED;
      result->detail = "the lane acquisition service did not answer";
      finish(handle, result, false);
      return;
    }
    const AcquireLaneObservation::Response response = *future.get();
    if (response.outcome != AcquireLaneObservation::Response::OUTCOME_PUBLISHED) {
      result->outcome = SurveyLane::Result::OUTCOME_ACQUISITION_FAILED;
      result->detail = "acquisition outcome " + std::to_string(response.outcome) + ": " +
        response.detail;
      RCLCPP_WARN(
        node_->get_logger(), "lane survey of %s published nothing: %s", lane_id.c_str(),
        result->detail.c_str());
      finish(handle, result, false);
      return;
    }
    result->outcome = SurveyLane::Result::OUTCOME_OBSERVED;
    result->detail = response.detail;
    result->observation = response.observation;
    RCLCPP_INFO(
      node_->get_logger(),
      "lane survey of %s: aim %.4f m / %.4f rad, available depth %.4f m, coverage %.3f, "
      "observation status %u",
      lane_id.c_str(), result->achieved_translation_error_m, result->achieved_rotation_error_rad,
      response.observation.available_depth_m,
      static_cast<double>(response.observation.confidence), response.observation.status);
    finish(handle, result, false);
  }

  void finish(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<SurveyLane::Result> & result, bool canceled)
  {
    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
      outstanding_.reset();
    }
    // A terminal transition after the context is down throws from inside rclcpp: the motion
    // completion arrives on a port worker thread, which races the signal handler rather than
    // the executor, so a survey that finishes during teardown lands here with an invalid
    // action server. That is a shutdown race, not a reason to terminate the process — the
    // same guard tray_survey_node and survey_viewpoint_node carry. An unguarded throw from
    // this thread reaches std::terminate and shows up as SIGABRT in a post-shutdown check.
    try {
      if (canceled && handle->is_canceling()) {
        handle->canceled(result);
        return;
      }
      // Every outcome is a completed survey, exactly as for SurveyViewpoint: a refused viewpoint or
      // a failed acquisition is a result the caller has to read, not an aborted goal.
      handle->succeed(result);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "lane survey result could not be delivered: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "lane survey result could not be delivered");
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string shelf_frame_;
  std::chrono::milliseconds dwell_{0};
  std::chrono::milliseconds acquisition_timeout_{0};
  ViewpointSurveyConfig config_;
  std::vector<restocker_perception::SurveyStation> stations_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<restocker_perception::TfViewpointPort> viewpoints_;
  std::unique_ptr<MoveItMotionPort> motion_;
  std::unique_ptr<ViewpointSurvey> survey_;
  rclcpp::CallbackGroup::SharedPtr acquisition_group_;
  rclcpp::Client<AcquireLaneObservation>::SharedPtr acquire_;
  rclcpp_action::Server<SurveyLane>::SharedPtr server_;
  mutable std::mutex mutex_;
  bool busy_{false};
  // Weak: the executing goal is owned by the completion callback that carries it, and this must
  // not keep it alive past the survey it belongs to.
  std::weak_ptr<GoalHandle> outstanding_;
  std::uint64_t operation_generation_{0U};
};

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  int status = 0;
  try {
    auto server = std::make_shared<restocker_task_executor::LaneSurveyNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server->node());
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("lane_survey"), "lane survey node failed: %s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
