// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Consumer of the viewpoint boundary: an action server that aims the wrist camera.
//
// It owns nothing new. A survey is a ViewpointPort resolution followed by an ordinary kFreeSpace
// MotionPort segment, and both of those already exist with their own private nodes and executor
// threads. What this file adds is the one thing that had no home: a place where "put the camera
// here" is a request a client can make, and where the answer says both what was commanded and
// what was achieved, measured independently of the arithmetic that commanded it.
//
// That independence is the point. The commanded tool pose comes from composing the mount
// transform; the achieved camera pose is read straight out of TF, which composes the whole
// kinematic chain. If the mount were applied on the wrong side, the two would disagree by twice
// the mount offset and the disagreement is reported on every single request.

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>
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

using SurveyViewpoint = restocker_interfaces::action::SurveyViewpoint;
using GoalHandle = rclcpp_action::ServerGoalHandle<SurveyViewpoint>;

[[nodiscard]] std::uint8_t result_outcome(SurveyOutcome outcome) noexcept
{
  switch (outcome) {
    case SurveyOutcome::kArrived:
      return SurveyViewpoint::Result::OUTCOME_ARRIVED;
    case SurveyOutcome::kViewpointUnresolved:
      return SurveyViewpoint::Result::OUTCOME_VIEWPOINT_UNRESOLVED;
    case SurveyOutcome::kPlanningFailed:
      return SurveyViewpoint::Result::OUTCOME_PLANNING_FAILED;
    case SurveyOutcome::kExecutionFailed:
      return SurveyViewpoint::Result::OUTCOME_EXECUTION_FAILED;
    case SurveyOutcome::kCanceled:
      return SurveyViewpoint::Result::OUTCOME_CANCELED;
    case SurveyOutcome::kTimedOut:
      return SurveyViewpoint::Result::OUTCOME_TIMED_OUT;
    case SurveyOutcome::kUnavailable:
      return SurveyViewpoint::Result::OUTCOME_UNAVAILABLE;
    case SurveyOutcome::kInvalidRequest:
      return SurveyViewpoint::Result::OUTCOME_INVALID_REQUEST;
  }
  return SurveyViewpoint::Result::OUTCOME_UNSET;
}

[[nodiscard]] geometry_msgs::msg::PoseStamped stamped(
  const std::string & frame_id, const rclcpp::Time & stamp, const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.pose = tf2::toMsg(pose);
  return message;
}

// The geodesic angle between two orientations, in [0, pi]. The same quantity
// compare_framed_poses reports, restated here because this file compares an achieved pose against
// a commanded one rather than an estimate against a reference.
[[nodiscard]] double rotation_error(const Eigen::Isometry3d & left, const Eigen::Isometry3d & right)
{
  const Eigen::AngleAxisd difference(left.linear().transpose() * right.linear());
  return std::abs(difference.angle());
}

class SurveyViewpointNode
{
public:
  explicit SurveyViewpointNode(const rclcpp::NodeOptions & options)
  : node_(std::make_shared<rclcpp::Node>("survey_viewpoint", options))
  {
    planning_frame_ = declare("planning_frame", std::string("world"));
    shelf_frame_ = declare("shelf_frame", std::string(restocker_perception::kShelfFrame));
    const std::string geometry_path = declare("workcell_geometry_path", std::string());
    const bool require_scene_authority = declare("require_planning_scene_authority", true);
    // How long to wait after the controllers report success before reading TF for the achieved
    // pose. The controllers stop the trajectory at its last point, but the simulated arm settles
    // over a few physics steps after that, and a reading taken before it does measures the
    // settling rather than the aiming.
    settle_ = std::chrono::milliseconds(
      declare("achieved_pose_settle_ms", static_cast<int>(400)));
    config_.position_tolerance_m = declare("position_tolerance_m", 0.006);
    config_.orientation_tolerance_rad = declare("orientation_tolerance_rad", 0.010);
    config_.velocity_scaling = declare("velocity_scaling", 0.2);
    config_.acceleration_scaling = declare("acceleration_scaling", 0.2);
    config_.planning_time =
      std::chrono::milliseconds(declare("planning_time_ms", static_cast<int>(5000)));

    if (geometry_path.empty()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "no workcell_geometry_path: nominal survey stations are unavailable and only explicit "
        "camera poses will be accepted");
    } else {
      try {
        stations_ = restocker_perception::nominal_survey_stations(
          restocker_perception::load_workcell_survey_geometry(geometry_path));
        RCLCPP_INFO(
          node_->get_logger(), "%zu nominal survey stations derived from %s", stations_.size(),
          geometry_path.c_str());
        for (const restocker_perception::SurveyStation & station : stations_) {
          const Eigen::Vector3d origin = station.shelf_from_optical.translation();
          RCLCPP_INFO(
            node_->get_logger(),
            "  station %s: camera at %s (%.3f, %.3f, %.3f), nominal range %.3f m",
            station.name.c_str(), shelf_frame_.c_str(), origin.x(), origin.y(), origin.z(),
            station.nominal_range_m);
        }
      } catch (const std::exception & error) {
        // Loud, and fatal to the station table only: explicit camera poses still work, and a
        // survey asked for a station it cannot derive is refused by name rather than aimed
        // somewhere plausible.
        RCLCPP_ERROR(
          node_->get_logger(), "could not derive survey stations from %s: %s",
          geometry_path.c_str(), error.what());
      }
    }

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // The listener spins on its own thread. Fed from this node's default, mutually exclusive
    // callback group it would share that group with the action's execute callback, and a lookup
    // made while a goal is being set up would then wait for transforms the executor cannot
    // deliver until that lookup returns.
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

    // Both ports read the robot model and TF from their own nodes, so they are handed this node's
    // parameter overrides exactly as the coordinator hands them to its motion port.
    rclcpp::NodeOptions port_options;
    port_options.automatically_declare_parameters_from_overrides(true);
    port_options.parameter_overrides(node_->get_node_options().parameter_overrides());
    viewpoints_ = std::make_unique<restocker_perception::TfViewpointPort>(
      port_options, restocker_perception::TfViewpointPortConfig{},
      "survey_viewpoint_tf_client");
    MoveItMotionPortConfig motion_config;
    motion_config.planning_frame = planning_frame_;
    motion_config.require_planning_scene_authority = require_scene_authority;
    motion_ = std::make_unique<MoveItMotionPort>(
      port_options, motion_config, "survey_viewpoint_motion_client");
    survey_ = std::make_unique<ViewpointSurvey>(*viewpoints_, *motion_, config_);

    server_ = rclcpp_action::create_server<SurveyViewpoint>(
      node_, "survey_viewpoint",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SurveyViewpoint::Goal> goal) {
        return handle_goal(*goal);
      },
      [this](const std::shared_ptr<GoalHandle> &) {
        survey_->cancel();
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<GoalHandle> handle) {start(handle);});
    RCLCPP_INFO(node_->get_logger(), "survey viewpoint server ready on /survey_viewpoint");
  }

  ~SurveyViewpointNode()
  {
    // Before anything that could hold the goal handle is destroyed. `~ServerGoalHandle` publishes
    // a cancellation for a goal that never reached a terminal state, and by teardown the context
    // is already shut down, so that publish throws, from a destructor, which terminates the
    // process. `abort` marks the goal terminal before it publishes, so the destructor has nothing
    // left to do and the throw, if it still comes, is ours to catch. The same fault was observed
    // in lane_survey_node, which has this action-server shape exactly.
    terminate_outstanding_goal();
    // Order matters: the survey holds references to both ports, so it goes first.
    survey_.reset();
    if (motion_) {
      motion_->shutdown();
    }
    if (viewpoints_) {
      viewpoints_->shutdown();
    }
  }

  SurveyViewpointNode(const SurveyViewpointNode &) = delete;
  SurveyViewpointNode & operator=(const SurveyViewpointNode &) = delete;
  SurveyViewpointNode(SurveyViewpointNode &&) = delete;
  SurveyViewpointNode & operator=(SurveyViewpointNode &&) = delete;

  [[nodiscard]] rclcpp::Node::SharedPtr node() const {return node_;}

private:
  template<typename Value>
  [[nodiscard]] Value declare(const std::string & name, const Value & fallback)
  {
    return node_->has_parameter(name) ? node_->get_parameter(name).get_value<Value>() :
           node_->declare_parameter<Value>(name, fallback);
  }

  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(const SurveyViewpoint::Goal & goal) const
  {
    if (goal.station.empty() && goal.camera_optical_pose.header.frame_id.empty()) {
      RCLCPP_WARN(
        node_->get_logger(), "rejecting a survey goal that names neither a station nor a pose");
      return rclcpp_action::GoalResponse::REJECT;
    }
    std::scoped_lock lock(mutex_);
    if (busy_) {
      RCLCPP_WARN(node_->get_logger(), "rejecting a survey goal while another is outstanding");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Turn the goal into a camera pose in the planning frame. Returns nullopt with `detail` set.
  [[nodiscard]] std::optional<restocker_perception::CameraViewpoint> requested_viewpoint(
    const SurveyViewpoint::Goal & goal, std::string & detail)
  {
    if (!goal.station.empty()) {
      const restocker_perception::SurveyStation * station =
        restocker_perception::find_survey_station(stations_, goal.station);
      if (station == nullptr) {
        detail = "no nominal survey station named " + goal.station;
        return std::nullopt;
      }
      geometry_msgs::msg::TransformStamped message;
      try {
        message = tf_buffer_->lookupTransform(
          planning_frame_, shelf_frame_, tf2::TimePointZero, tf2::durationFromSec(5.0));
      } catch (const tf2::TransformException & error) {
        detail = "could not resolve " + planning_frame_ + " <- " + shelf_frame_ + ": " +
          error.what();
        return std::nullopt;
      }
      const restocker_perception::Result<restocker_perception::FramedTransform> transform =
        restocker_perception::FramedTransform::create(
        shelf_frame_, planning_frame_, tf2::transformToEigen(message));
      if (!transform) {
        detail = transform.error().detail;
        return std::nullopt;
      }
      restocker_perception::Result<restocker_perception::CameraViewpoint> viewpoint =
        restocker_perception::station_viewpoint(*station, transform.value());
      if (!viewpoint) {
        detail = viewpoint.error().detail;
        return std::nullopt;
      }
      if (!goal.label.empty()) {
        viewpoint.value().label = goal.label;
      }
      return viewpoint.value();
    }

    if (goal.camera_optical_pose.header.frame_id != planning_frame_) {
      detail = "an explicit viewpoint must be expressed in the planning frame " + planning_frame_ +
        ", not in " + goal.camera_optical_pose.header.frame_id;
      return std::nullopt;
    }
    restocker_perception::CameraViewpoint viewpoint;
    viewpoint.pose.frame_id = planning_frame_;
    tf2::fromMsg(goal.camera_optical_pose.pose, viewpoint.pose.pose);
    viewpoint.label = goal.label.empty() ? std::string("explicit viewpoint") : goal.label;
    if (!viewpoint.pose.pose.matrix().allFinite()) {
      detail = "the requested camera pose is not finite";
      return std::nullopt;
    }
    return viewpoint;
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
      auto result = std::make_shared<SurveyViewpoint::Result>();
      result->station = handle->get_goal()->station;
      // Review F2 (Card 052): a shutdown abort must not claim OUTCOME_UNAVAILABLE — that
      // constant means "the backend never engaged, no motion was attempted", and the
      // campaign's leg-evidence helpers read it as exactly that. OUTCOME_SHUTDOWN claims
      // neither no-motion nor a stop, so every consumer fails closed on it, the way
      // lane_survey_node's shutdown abort forces both of its flags false.
      result->outcome = SurveyViewpoint::Result::OUTCOME_SHUTDOWN;
      result->detail = "the survey viewpoint node shut down before this survey completed";
      handle->abort(result);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "outstanding viewpoint survey could not be aborted at shutdown: %s",
        error.what());
    } catch (...) {
      RCLCPP_WARN(
        node_->get_logger(), "outstanding viewpoint survey could not be aborted at shutdown");
    }
  }

  void start(const std::shared_ptr<GoalHandle> & handle)
  {
    {
      std::scoped_lock lock(mutex_);
      busy_ = true;
      outstanding_ = handle;
    }
    const SurveyViewpoint::Goal goal = *handle->get_goal();
    auto result = std::make_shared<SurveyViewpoint::Result>();
    result->station = goal.station;

    auto feedback = std::make_shared<SurveyViewpoint::Feedback>();
    feedback->phase = SurveyViewpoint::Feedback::PHASE_RESOLVING_VIEWPOINT;
    handle->publish_feedback(feedback);

    std::string detail;
    const std::optional<restocker_perception::CameraViewpoint> viewpoint =
      requested_viewpoint(goal, detail);
    if (!viewpoint) {
      result->outcome = SurveyViewpoint::Result::OUTCOME_INVALID_REQUEST;
      result->detail = detail;
      RCLCPP_WARN(node_->get_logger(), "survey refused: %s", detail.c_str());
      finish(handle, result, false);
      return;
    }
    result->commanded_camera_optical_pose =
      stamped(planning_frame_, node_->now(), viewpoint->pose.pose);

    ViewpointSurveyConfig overrides;
    overrides.position_tolerance_m = goal.position_tolerance_m;
    overrides.orientation_tolerance_rad = goal.orientation_tolerance_rad;
    overrides.velocity_scaling = 0.0;
    overrides.acceleration_scaling = 0.0;
    overrides.planning_time = std::chrono::milliseconds(0);

    feedback->phase = SurveyViewpoint::Feedback::PHASE_MOVING;
    handle->publish_feedback(feedback);
    const restocker_perception::CameraViewpoint commanded = *viewpoint;
    const SurveySubmitResult submitted = survey_->submit(
      OperationCorrelation{0U, ++operation_generation_}, *viewpoint, overrides,
      [this, handle, result, commanded](SurveyCompletion completion) {
        complete(handle, result, commanded, std::move(completion));
      });
    if (submitted) {
      return;
    }
    apply_viewpoint_submit_refusal(submitted.status, *result);
    result->detail = submitted.detail;
    RCLCPP_WARN(
      node_->get_logger(), "survey submission refused: %s", submitted.detail.c_str());
    finish(handle, result, false);
  }

  void complete(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<SurveyViewpoint::Result> & result,
    const restocker_perception::CameraViewpoint & commanded, SurveyCompletion completion)
  {
    result->outcome = result_outcome(completion.outcome);
    result->detail = completion.detail;
    result->execution_reached_terminal_stop = completion.execution_reached_terminal_stop;
    if (completion.viewpoint_resolved) {
      result->commanded_tool0_pose =
        stamped(planning_frame_, node_->now(), completion.commanded_goal.pose.pose);
    }

    // Let the arm settle before reading where it ended up, then read the achieved camera pose
    // straight out of TF. This is the independent half of the check: nothing in this measurement
    // passes through the mount arithmetic that produced the tool goal.
    if (settle_.count() > 0) {
      std::this_thread::sleep_for(settle_);
    }
    geometry_msgs::msg::TransformStamped achieved;
    try {
      achieved = tf_buffer_->lookupTransform(
        planning_frame_, restocker_perception::kWristCameraOpticalFrame, tf2::TimePointZero,
        tf2::durationFromSec(2.0));
      const Eigen::Isometry3d pose = tf2::transformToEigen(achieved);
      result->achieved_pose_measured = true;
      result->achieved_camera_optical_pose =
        stamped(planning_frame_, achieved.header.stamp, pose);
      result->achieved_translation_error_m =
        (pose.translation() - commanded.pose.pose.translation()).norm();
      result->achieved_rotation_error_rad = rotation_error(pose, commanded.pose.pose);
    } catch (const tf2::TransformException & error) {
      result->achieved_pose_measured = false;
      result->detail += " (achieved camera pose not measured: " + std::string(error.what()) + ")";
    }

    // First-hand, because the alternative is a report several transitions removed from its cause.
    RCLCPP_INFO(
      node_->get_logger(),
      "survey %s -> %s: commanded camera (%.4f, %.4f, %.4f), tool (%.4f, %.4f, %.4f), "
      "achieved %s, error %.4f m / %.4f rad. %s",
      commanded.label.c_str(), survey_outcome_name(completion.outcome),
      commanded.pose.pose.translation().x(), commanded.pose.pose.translation().y(),
      commanded.pose.pose.translation().z(),
      result->commanded_tool0_pose.pose.position.x,
      result->commanded_tool0_pose.pose.position.y,
      result->commanded_tool0_pose.pose.position.z,
      result->achieved_pose_measured ? "measured" : "not measured",
      result->achieved_translation_error_m, result->achieved_rotation_error_rad,
      result->detail.c_str());

    finish(handle, result, completion.outcome == SurveyOutcome::kCanceled);
  }

  void finish(
    const std::shared_ptr<GoalHandle> & handle,
    const std::shared_ptr<SurveyViewpoint::Result> & result, bool canceled)
  {
    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
      outstanding_.reset();
    }
    // A terminal transition after the context is down throws from inside rclcpp: the
    // completion arrives on a port worker thread, which races the signal handler rather than
    // the executor, so a motion that finishes during teardown lands here with an invalid
    // action server. That is a shutdown race, not a reason to terminate the process — the
    // same guard tray_survey_node carries. An unguarded throw from this thread reaches
    // std::terminate and shows up as SIGABRT in a launch test's post-shutdown check.
    try {
      if (canceled && handle->is_canceling()) {
        handle->canceled(result);
        return;
      }
      // Every outcome is a completed survey: a refused viewpoint or a failed plan is a result the
      // caller has to read, not an aborted goal whose result it must guess at. The outcome field
      // carries the verdict.
      handle->succeed(result);
    } catch (const std::exception & error) {
      RCLCPP_WARN(
        node_->get_logger(), "survey viewpoint result could not be delivered: %s", error.what());
    } catch (...) {
      RCLCPP_WARN(node_->get_logger(), "survey viewpoint result could not be delivered");
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string shelf_frame_;
  std::chrono::milliseconds settle_{0};
  ViewpointSurveyConfig config_;
  std::vector<restocker_perception::SurveyStation> stations_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<restocker_perception::TfViewpointPort> viewpoints_;
  std::unique_ptr<MoveItMotionPort> motion_;
  std::unique_ptr<ViewpointSurvey> survey_;
  rclcpp_action::Server<SurveyViewpoint>::SharedPtr server_;
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
    // The action server's execute callback runs the whole survey, and the survey's completion
    // arrives on a port worker thread, so this node needs a multi-threaded executor: a
    // single-threaded one would hold the goal callback while the ports it is waiting on cannot
    // publish feedback.
    auto server = std::make_shared<restocker_task_executor::SurveyViewpointNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server->node());
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("survey_viewpoint"), "survey viewpoint node failed: %s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
