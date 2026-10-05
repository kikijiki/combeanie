// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/moveit_motion_port.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <future>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "restocker_task_executor/controller_trajectory_slicing.hpp"
#include "restocker_task_executor/free_space_race.hpp"
#include "restocker_task_executor/grasp_escape_planner.hpp"
#include "restocker_task_executor/linear_path_refinement.hpp"
#include "restocker_task_executor/motion_execute_verdict.hpp"
#include "restocker_task_executor/race_planning_scene.hpp"
#include "restocker_task_executor/moveit_path_constraint_conversion.hpp"
#include "restocker_task_executor/obstacle_projection.hpp"
#include "restocker_task_executor/planning_contract.hpp"
#include "restocker_task_executor/trajectory_scene_validation.hpp"

namespace restocker_task_executor
{
namespace
{

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

// Most a failure detail names before it stops listing.
constexpr std::size_t kMaxReportedContacts = 3U;

// The post-plan corridor gate interpolates each joint-space leg at no more than this change per
// sample: 2.5 mm for the rail, 0.0025 rad for revolute joints. Both leg endpoints are checked.
constexpr double kPathConstraintInterpolationStep = 0.0025;
// OMPL's constrained state space projects samples onto the requested face, and MoveIt's
// interpolation has landed up to 1.7 mm outside afterward. The planner works 5 mm inside the
// geometry-owned box; dense FK validation still uses the unmodified outer box.
constexpr double kPathConstraintPlanningInsetM = 0.005;
// A pose goal has several IK branches, only some of which continue straight into the tray without
// putting the wrist through it. Re-sample a bounded number of times.
constexpr std::size_t kMaximumContinuationIkAttempts = 128U;
constexpr double kContinuationIkTimeoutS = 0.05;

// Clears path constraints even when planning returns early, so a prior segment's corridor never
// carries over.
class PathConstraintGuard
{
public:
  explicit PathConstraintGuard(MoveGroupInterface & interface) noexcept
  : interface_(&interface) {}

  PathConstraintGuard(const PathConstraintGuard &) = delete;
  PathConstraintGuard & operator=(const PathConstraintGuard &) = delete;

  ~PathConstraintGuard()
  {
    if (interface_ != nullptr) {
      try {
        interface_->clearPathConstraints();
      } catch (...) {
        // Best-effort; the next plan sets or clears explicitly.
      }
    }
  }

private:
  MoveGroupInterface * interface_;
};

struct TransferTrajectoryValidation
{
  std::string violation;
  double maximum_tool0_x_tilt_rad{0.0};
  std::size_t maximum_tilt_sample{0U};
  std::size_t sample_count{0U};
};

[[nodiscard]] TransferTrajectoryValidation validate_transfer_trajectory(
  const moveit::core::RobotState & planning_start,
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const MotionPathConstraints * constraints, std::optional<double> maximum_upright_tilt_rad,
  const std::string & planning_frame, const std::string & end_effector_link)
{
  TransferTrajectoryValidation validation;
  if ((constraints == nullptr || !constraints->position_box) && !maximum_upright_tilt_rad) {
    return validation;
  }
  if (constraints != nullptr &&
    ((!constraints->planning_frame.empty() && constraints->planning_frame != planning_frame) ||
    (!constraints->link_name.empty() && constraints->link_name != end_effector_link)))
  {
    validation.violation =
      "post-plan transfer validation cannot transform a non-default frame or link";
    return validation;
  }
  const auto & names = trajectory.joint_trajectory.joint_names;
  const auto & points = trajectory.joint_trajectory.points;
  const auto & model = planning_start.getRobotModel();
  if (!model || model->getLinkModel(end_effector_link) == nullptr || names.empty() ||
    points.empty())
  {
    validation.violation =
      "post-plan transfer validation received no usable robot trajectory or link";
    return validation;
  }
  for (const auto & point : points) {
    if (point.positions.size() != names.size()) {
      validation.violation =
        "post-plan transfer validation found a trajectory point with missing joints";
      return validation;
    }
  }

  moveit::core::RobotState state(planning_start);
  std::vector<double> positions(names.size(), 0.0);
  std::size_t sample = 0U;
  for (std::size_t point_index = 0; point_index < points.size(); ++point_index) {
    const auto & finish = points[point_index].positions;
    const auto & begin = point_index == 0U ? finish : points[point_index - 1U].positions;
    double maximum_delta = 0.0;
    for (std::size_t joint = 0; joint < names.size(); ++joint) {
      maximum_delta = std::max(maximum_delta, std::abs(finish[joint] - begin[joint]));
    }
    const auto subdivisions = std::max<std::size_t>(
      1U, static_cast<std::size_t>(std::ceil(
        maximum_delta /
        kPathConstraintInterpolationStep)));
    for (std::size_t subdivision = point_index == 0U ? 0U : 1U;
      subdivision <= subdivisions; ++subdivision)
    {
      const double alpha = static_cast<double>(subdivision) /
        static_cast<double>(subdivisions);
      for (std::size_t joint = 0; joint < names.size(); ++joint) {
        positions[joint] = begin[joint] + alpha * (finish[joint] - begin[joint]);
        if (!std::isfinite(positions[joint])) {
          validation.violation =
            "post-plan transfer validation found a non-finite joint position";
          return validation;
        }
        state.setVariablePosition(names[joint], positions[joint]);
      }
      state.updateLinkTransforms();
      const Eigen::Isometry3d & world_from_tool0 =
        state.getGlobalLinkTransform(end_effector_link);
      const Eigen::Vector3d position = world_from_tool0.translation();
      const double upright_alignment = std::clamp(
        world_from_tool0.linear().col(0).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
      const double tilt = std::acos(upright_alignment);
      if (tilt > validation.maximum_tool0_x_tilt_rad) {
        validation.maximum_tool0_x_tilt_rad = tilt;
        validation.maximum_tilt_sample = sample;
      }
      if (constraints != nullptr && constraints->position_box &&
        !motion_path_position_satisfies_box(position, *constraints->position_box))
      {
        std::ostringstream detail;
        detail << std::setprecision(12) << "planned trajectory leaves enforced transfer box at "
               << "sample " << sample << " (trajectory point " << point_index << ") with "
               << end_effector_link << "=(" << position.x() << ", " << position.y() << ", "
               << position.z() << "); " << constraints->description;
        validation.violation = detail.str();
        validation.sample_count = sample + 1U;
        return validation;
      }
      if (maximum_upright_tilt_rad && tilt > *maximum_upright_tilt_rad) {
        std::ostringstream detail;
        detail << std::setprecision(12)
               << "planned trajectory tips held-product/tool0 +X axis " << tilt
               << " rad from planning +Z at sample " << sample << " (trajectory point "
               << point_index << "), exceeding hard upright budget "
               << *maximum_upright_tilt_rad << " rad; "
               << (constraints == nullptr ? std::string{} : constraints->description);
        validation.violation = detail.str();
        validation.sample_count = sample + 1U;
        return validation;
      }
      ++sample;
    }
  }
  validation.sample_count = sample;
  return validation;
}

[[nodiscard]] std::string with_moveit_reason(
  const std::string & detail, int moveit_error_code)
{
  std::ostringstream stream;
  stream << detail << " (MoveIt reported " << moveit_error_name(moveit_error_code) << ", code "
         << moveit_error_code << ")";
  return stream.str();
}

// Fastest velocity the trajectory asks of one joint. Negative when it has no velocities.
[[nodiscard]] double planned_joint_speed_maximum(
  const moveit_msgs::msg::RobotTrajectory & trajectory, std::size_t joint_index)
{
  double maximum = -1.0;
  for (const auto & point : trajectory.joint_trajectory.points) {
    if (joint_index >= point.velocities.size()) {
      continue;
    }
    maximum = std::max(maximum, std::abs(point.velocities[joint_index]));
  }
  return maximum;
}

struct ContinuationEndpointSelection
{
  std::vector<moveit::core::RobotState> endpoints;
  MotionOutcome failure_outcome{MotionOutcome::kPlanningFailed};
  std::string failure_detail;
  bool planner_refused{false};
};

[[nodiscard]] ContinuationEndpointSelection select_continuation_endpoints(
  MoveGroupInterface & interface,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const moveit::core::RobotState & current_state,
  const moveit::core::JointModelGroup * planning_group,
  const moveit::core::RobotModelConstPtr & robot_model,
  const MoveItMotionPortConfig & config, const MotionGoal & goal,
  std::chrono::steady_clock::time_point planning_deadline)
{
  ContinuationEndpointSelection selection;
  double last_fraction = 0.0;
  int last_error = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
  std::size_t sampled_endpoints = 0U;
  // Why each sampled endpoint was not retained. Without these the empty-selection message can
  // only quote a fraction that, when no continuation was ever checked, is still the 0.0 it was
  // initialized with — which reads like a blocked path when nothing was measured at all.
  std::size_t ik_unsolved = 0U;
  std::size_t endpoint_rejected_by_scene = 0U;
  std::size_t continuation_rejected = 0U;
  std::size_t duplicates = 0U;
  bool any_continuation_checked = false;
  for (std::size_t attempt = 1U;
    attempt <= kMaximumContinuationIkAttempts &&
    selection.endpoints.size() < goal.free_space_plan_candidates;
    ++attempt)
  {
    if (std::chrono::steady_clock::now() >= planning_deadline) {
      break;
    }
    sampled_endpoints = attempt;
    moveit::core::RobotState endpoint(current_state);
    endpoint.setToRandomPositions(planning_group);
    if (!endpoint.setFromIK(
        planning_group, goal.planning_frame_from_tool0,
        config.end_effector_link, kContinuationIkTimeoutS))
    {
      ++ik_unsolved;
      continue;
    }
    // computeCartesianPath() can return 100% when its supplied start is in collision, so validate
    // the endpoint with the current jaw state first.
    if (!validity_client || !validity_client->service_is_ready()) {
      selection.failure_outcome = MotionOutcome::kUnavailable;
      selection.failure_detail =
        "/check_state_validity is unavailable while selecting a continuation-safe endpoint";
      return selection;
    }
    auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    moveit::core::robotStateToRobotStateMsg(endpoint, request->robot_state);
    request->group_name = config.planning_group;
    auto future = validity_client->async_send_request(request);
    const auto validity_wait = std::min(
      config.state_validity_timeout,
      std::chrono::duration_cast<std::chrono::milliseconds>(
        planning_deadline - std::chrono::steady_clock::now()));
    if (validity_wait <= std::chrono::milliseconds::zero() ||
      future.wait_for(validity_wait) != std::future_status::ready)
    {
      validity_client->remove_pending_request(future);
      selection.failure_outcome = MotionOutcome::kTimedOut;
      selection.failure_detail = goal.label +
        " exhausted its planning budget checking an IK endpoint";
      return selection;
    }
    const auto validity = future.get();
    if (!validity) {
      selection.failure_outcome = MotionOutcome::kUnavailable;
      selection.failure_detail =
        "/check_state_validity returned no continuation-safe endpoint verdict";
      return selection;
    }
    if (!validity->valid) {
      ++endpoint_rejected_by_scene;
      continue;
    }
    for (const auto & joint : config.gripper_joint_names) {
      if (!robot_model->hasJointModel(joint)) {
        selection.failure_outcome = MotionOutcome::kUnavailable;
        selection.failure_detail =
          "required continuation gripper joints are absent from the MoveIt robot model";
        return selection;
      }
      endpoint.setVariablePosition(
        joint, *goal.required_linear_continuation_gripper_joint_position_m);
    }
    endpoint.update();
    moveit::core::RobotState pregrasp_endpoint(endpoint);
    interface.setStartState(endpoint);
    moveit_msgs::msg::RobotTrajectory continuation;
    moveit_msgs::msg::MoveItErrorCodes continuation_error;
    continuation_error.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    const geometry_msgs::msg::Pose continuation_target =
      tf2::toMsg(*goal.required_linear_continuation_pose);
    last_fraction = interface.computeCartesianPath(
      {continuation_target}, goal.cartesian_step_m, continuation, true,
      &continuation_error);
    last_error = continuation_error.val;
    any_continuation_checked = true;
    if (last_fraction < goal.minimum_cartesian_fraction) {
      ++continuation_rejected;
      continue;
    }
    if (goal.required_linear_continuation_vertical_margin_m > 0.0) {
      geometry_msgs::msg::Pose clearance_target = continuation_target;
      clearance_target.position.z -= goal.required_linear_continuation_vertical_margin_m;
      interface.setStartState(endpoint);
      moveit_msgs::msg::RobotTrajectory clearance_continuation;
      moveit_msgs::msg::MoveItErrorCodes clearance_error;
      clearance_error.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
      last_fraction = interface.computeCartesianPath(
        {clearance_target}, goal.cartesian_step_m, clearance_continuation, true,
        &clearance_error);
      last_error = clearance_error.val;
      any_continuation_checked = true;
      if (last_fraction < goal.minimum_cartesian_fraction) {
        ++continuation_rejected;
        continue;
      }
    }
    if (goal.required_postcontinuation_linear_retract_pose) {
      const auto advance_endpoint = [&endpoint](
        const moveit_msgs::msg::RobotTrajectory & path) {
        const auto & names = path.joint_trajectory.joint_names;
        const auto & points = path.joint_trajectory.points;
        if (points.empty() || points.back().positions.size() != names.size()) {
          return false;
        }
        for (std::size_t index = 0U; index < names.size(); ++index) {
          endpoint.setVariablePosition(names[index], points.back().positions[index]);
        }
        endpoint.update();
        return true;
      };
      if (!advance_endpoint(continuation)) {
        selection.failure_outcome = MotionOutcome::kUnavailable;
        selection.failure_detail = goal.label +
          " cannot reconstruct the endpoint of its required approach";
        return selection;
      }
      bool chain_supported = true;
      for (const auto * continuation_pose : {
            &*goal.required_postcontinuation_linear_retract_pose,
            &*goal.required_postcontinuation_linear_egress_pose})
      {
        for (const auto & joint : config.gripper_joint_names) {
          endpoint.setVariablePosition(
            joint, *goal.required_postcontinuation_gripper_joint_position_m);
        }
        endpoint.update();
        interface.setStartState(endpoint);
        moveit_msgs::msg::RobotTrajectory path;
        moveit_msgs::msg::MoveItErrorCodes path_error;
        path_error.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
        const geometry_msgs::msg::Pose path_target = tf2::toMsg(*continuation_pose);
        last_fraction = interface.computeCartesianPath(
          {path_target}, goal.cartesian_step_m, path, true, &path_error);
        last_error = path_error.val;
        any_continuation_checked = true;
        if (last_fraction < goal.minimum_cartesian_fraction || !advance_endpoint(path)) {
          chain_supported = false;
          break;
        }
      }
      if (!chain_supported) {
        ++continuation_rejected;
        continue;
      }
    }
    const bool duplicate = std::any_of(
      selection.endpoints.cbegin(), selection.endpoints.cend(),
      [&pregrasp_endpoint, planning_group](const moveit::core::RobotState & retained) {
        return std::all_of(
          planning_group->getVariableNames().cbegin(),
          planning_group->getVariableNames().cend(),
          [&pregrasp_endpoint, &retained](const std::string & variable) {
            return std::abs(
              pregrasp_endpoint.getVariablePosition(variable) -
              retained.getVariablePosition(variable)) < 1.0e-4;
          });
      });
    if (!duplicate) {
      selection.endpoints.push_back(std::move(pregrasp_endpoint));
    } else {
      ++duplicates;
    }
  }
  interface.setStartState(current_state);
  if (selection.endpoints.empty()) {
    std::ostringstream detail;
    detail << goal.label << " sampled " << sampled_endpoints
           << " IK endpoints, but none was retained: " << ik_unsolved << " never solved IK, "
           << endpoint_rejected_by_scene
           << " were rejected by the planning scene before any continuation was checked, "
           << continuation_rejected << " failed the required linear continuation";
    if (any_continuation_checked) {
      detail << " (last fraction=" << std::setprecision(6) << 100.0 * last_fraction
             << "% and MoveIt reported " << moveit_error_name(last_error) << " (code "
             << last_error << ")";
      if (duplicates > 0U) {
        detail << "; " << duplicates << " duplicated an endpoint already retained";
      }
      detail << ")";
    } else {
      detail << "; no linear continuation was checked, so there is no fraction to report";
    }
    selection.failure_detail = detail.str();
    selection.planner_refused = true;
    return selection;
  }
  const auto squared_joint_travel =
    [&current_state, planning_group](const moveit::core::RobotState & endpoint) {
      double travel = 0.0;
      for (const auto & variable : planning_group->getVariableNames()) {
        const double delta = endpoint.getVariablePosition(variable) -
          current_state.getVariablePosition(variable);
        travel += delta * delta;
      }
      return travel;
    };
  std::stable_sort(
    selection.endpoints.begin(), selection.endpoints.end(),
    [&squared_joint_travel](const auto & left, const auto & right) {
      return squared_joint_travel(left) < squared_joint_travel(right);
    });
  return selection;
}

}  // namespace

// Owns the MoveGroupInterface so the header does not have to include MoveIt.
class MoveItMotionPort::MoveGroupHolder
{
public:
  MoveGroupHolder(
    const rclcpp::Node::SharedPtr & node, const MoveItMotionPortConfig & config)
  : interface_(
      node, config.planning_group, std::shared_ptr<tf2_ros::Buffer>(),
      rclcpp::Duration(config.startup_timeout))
  {
    interface_.setEndEffectorLink(config.end_effector_link);
    interface_.setPoseReferenceFrame(config.planning_frame);
    interface_.setPlanningPipelineId(config.planning_pipeline);
    interface_.setPlannerId(config.planner_id);
  }

  [[nodiscard]] MoveGroupInterface & interface() noexcept {return interface_;}

private:
  MoveGroupInterface interface_;
};

// Card 053: the two pipelines the free-space race runs inside one unchanged slice, built once
// per port from the node's ompl and ompl_fallback parameter namespaces (the launch passes both
// to every port-hosting node). Each pipeline owns its OWN RobotModel — and therefore its own
// KDL solver instance: IKConstraintSampler takes jmg->getSolverInstance(), so two pipelines on
// one model share a non-thread-safe solver across OMPL's goal-sampling threads (measured: two
// campaigns' survey_viewpoint SIGSEGVs, core pid 1420460, both threads inside
// KDLKinematicsPlugin). Each race pairs its pipeline with the scene built from its own model,
// so collision checking stays identical in content (one scene message) and consistent per
// thread.
class MoveItMotionPort::RacePipelines
{
public:
  RacePipelines(
    moveit::core::RobotModelConstPtr primary_model,
    robot_model_loader::RobotModelLoaderPtr fallback_loader,
    planning_pipeline::PlanningPipelinePtr primary_pipeline,
    planning_pipeline::PlanningPipelinePtr fallback_pipeline)
  : primary_model(std::move(primary_model)),
    fallback_loader(std::move(fallback_loader)),
    primary_pipeline(std::move(primary_pipeline)),
    fallback_pipeline(std::move(fallback_pipeline))
  {
  }

  moveit::core::RobotModelConstPtr primary_model;
  robot_model_loader::RobotModelLoaderPtr fallback_loader;
  planning_pipeline::PlanningPipelinePtr primary_pipeline;
  planning_pipeline::PlanningPipelinePtr fallback_pipeline;
};

MoveItMotionPort::MoveItMotionPort(
  const rclcpp::NodeOptions & options, MoveItMotionPortConfig config, std::string node_name)
: config_(std::move(config)),
  node_(std::make_shared<rclcpp::Node>(std::move(node_name), options)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  execution_clients_.reserve(config_.execution_controllers.size());
  for (const auto & controller : config_.execution_controllers) {
    execution_clients_.push_back(
      rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
        node_, "/" + controller + "/follow_joint_trajectory"));
  }
  state_validity_client_ = node_->create_client<moveit_msgs::srv::GetStateValidity>(
    "/check_state_validity");
  planning_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
    "/get_planning_scene");
  // MoveIt reports a bare CONTROL_FAILED for both a path-tolerance abort and a rejected goal; the
  // controller's own log says which. Volatile QoS avoids replaying rosout's transient-local
  // backlog.
  controller_log_subscription_ = node_->create_subscription<rcl_interfaces::msg::Log>(
    config_.controller_log_topic, rclcpp::QoS(rclcpp::KeepLast(100)).reliable(),
    [this](rcl_interfaces::msg::Log::ConstSharedPtr log) {
      if (
        std::ranges::find(config_.execution_controllers, log->name) ==
        config_.execution_controllers.end())
      {
        return;
      }
      auto report = parse_controller_abort(log->name, log->msg);
      if (!report) {
        return;
      }
      std::scoped_lock lock(controller_abort_mutex_);
      controller_abort_ = std::move(report);
    });
  // Peak feedback speeds, to tell the recorded impulse from tracking lag. QoS matches
  // RosGripperPort.
  joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_state_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::JointState::ConstSharedPtr state) {
      if (state->velocity.size() != state->name.size()) {
        return;
      }
      std::scoped_lock lock(joint_speed_mutex_);
      ++joint_state_samples_;
      for (std::size_t index = 0; index < state->name.size(); ++index) {
        auto & peak = joint_peak_speed_[state->name[index]];
        peak = std::max(peak, std::abs(state->velocity[index]));
      }
    });
  if (config_.require_planning_scene_authority) {
    // The projector publishes status latched, so this answers on connection.
    scene_status_subscription_ =
      node_->create_subscription<restocker_interfaces::msg::PlanningSceneProjectionStatus>(
      config_.planning_scene_status_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      [this](restocker_interfaces::msg::PlanningSceneProjectionStatus::ConstSharedPtr status) {
        std::scoped_lock lock(scene_status_mutex_);
        scene_status_ = std::move(status);
      });
    scene_status_client_ =
      node_->create_client<restocker_interfaces::srv::GetPlanningSceneProjectionStatus>(
      config_.planning_scene_status_service);
    scene_lease_validation_client_ =
      node_->create_client<restocker_interfaces::srv::ValidatePlanningSceneLease>(
      config_.planning_scene_lease_validation_service);
  }
  executor_->add_node(node_);
  executor_thread_ = std::thread(
    [this]() noexcept {
      try {
        executor_->spin();
      } catch (...) {
        // Losing the executor makes later submissions report kUnavailable (fail closed).
      }
    });
  worker_ = std::thread([this]() noexcept {run_worker();});
}

MoveItMotionPort::~MoveItMotionPort()
{
  shutdown();
}

void MoveItMotionPort::shutdown() noexcept
{
  if (stopping_.exchange(true)) {
    return;
  }
  cancel();
  work_available_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  try {
    executor_->cancel();
  } catch (...) {
    // Torn down anyway.
    // The executor is being torn down; joining below is what matters.
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  controller_log_subscription_.reset();
  joint_state_subscription_.reset();
  {
    std::scoped_lock lock(move_group_mutex_);
    move_group_.reset();
  }
  try {
    executor_->remove_node(node_);
  } catch (...) {
    // Best-effort during teardown.
    // Removal is best-effort during teardown.
  }
}

bool MoveItMotionPort::ready() const
{
  if (stopping_.load(std::memory_order_acquire)) {
    return false;
  }
  if (node_->count_publishers("/joint_states") == 0U) {
    return false;
  }
  // An empty controller list fails closed.
  if (execution_clients_.empty()) {
    return false;
  }
  return std::ranges::all_of(
    execution_clients_, [](const auto & client) {
      return client && client->action_server_is_ready();
    });
}

MotionSubmitResult MoveItMotionPort::submit(
  OperationCorrelation correlation, MotionGoal goal, CompletionCallback callback)
{
  if (!callback) {
    return {MotionSubmitStatus::kInvalidRequest,
      "motion submission requires a completion callback"};
  }
  if (!valid_motion_goal(goal)) {
    return {MotionSubmitStatus::kInvalidRequest,
      "motion goal is not a finite, orthonormal pose with usable limits"};
  }
  if (stopping_.load(std::memory_order_acquire)) {
    return {MotionSubmitStatus::kUnavailable, "motion port is shutting down"};
  }

  // Logged at the boundary to tell a goal never submitted from one the worker never started.
  const std::string label = goal.label;
  {
    std::scoped_lock lock(mutex_);
    if (pending_ || busy_) {
      RCLCPP_WARN(
        node_->get_logger(), "motion %s refused: the port already owns an outstanding goal",
        label.c_str());
      return {MotionSubmitStatus::kBusy, "motion port already owns an outstanding goal"};
    }
    cancel_requested_.store(false, std::memory_order_release);
    pending_.emplace(PendingGoal{correlation, std::move(goal), std::move(callback)});
  }
  RCLCPP_INFO(
    node_->get_logger(), "motion %s accepted as operation %s", label.c_str(),
    std::to_string(correlation.operation_generation).c_str());
  work_available_.notify_one();
  return {MotionSubmitStatus::kAccepted, {}};
}

void MoveItMotionPort::cancel() noexcept
{
  cancel_requested_.store(true, std::memory_order_release);
  std::scoped_lock lock(move_group_mutex_);
  if (move_group_) {
    try {
      move_group_->interface().stop();
    } catch (...) {
      // An undeliverable stop leaves the segment to time out, which is still terminal.
    }
  }
}

void MoveItMotionPort::run_worker() noexcept
{
  for (;; ) {
    PendingGoal pending;
    {
      std::unique_lock lock(mutex_);
      work_available_.wait(
        lock, [this]() {return pending_.has_value() || stopping_.load(std::memory_order_acquire);});
      if (!pending_) {
        return;                     // shutting down with nothing outstanding
      }
      pending = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }
    RCLCPP_INFO(
      node_->get_logger(), "motion %s dequeued by the port worker", pending.goal.label.c_str());

    execute_called_.store(false, std::memory_order_release);
    auto completion = execute_goal(pending);
    completion.submitted_to_backend = execute_called_.load(std::memory_order_acquire);

    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
    }
    // Logged before the completion leaves the port: downstream stages can drop it, leaving only the
    // coordinator's deadline as a symptom.
    RCLCPP_INFO(
      node_->get_logger(), "motion %s completed: %s%s%s",
      pending.goal.label.c_str(), motion_outcome_name(completion.outcome),
      completion.detail.empty() ? "" : ": ", completion.detail.c_str());
    try {
      pending.callback(std::move(completion));
    } catch (const std::exception & error) {
      // A throwing sink must not kill the worker, but the goal can now only end at its deadline.
      RCLCPP_ERROR(
        node_->get_logger(),
        "motion %s completion sink threw, so the coordinator will never see it: %s",
        pending.goal.label.c_str(), error.what());
    } catch (...) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "motion %s completion sink threw, so the coordinator will never see it",
        pending.goal.label.c_str());
    }
  }
}

// One line naming the exact inputs of a refused straight line — the planning group's start
// positions, the target pose and the interpolation step — appended to the refusal receipt so a
// single console line replays offline. Card 046: whether the stop is a kinematic limit or an
// interpolation artefact can only be decided by re-running this line at a finer step, and the
// receipts so far carry no joint state to replay from.
std::string describe_linear_refusal_fixture(
  const moveit::core::RobotState & start_state,
  const moveit::core::JointModelGroup * planning_group,
  const std::vector<std::string> & jaw_joints,
  const geometry_msgs::msg::Pose & target, double step_m)
{
  std::ostringstream fixture;
  fixture << std::fixed << std::setprecision(9);
  fixture << "fixture: start=[";
  if (planning_group == nullptr) {
    fixture << "planning group unavailable";
  } else {
    bool first = true;
    const auto append_variable = [&start_state, &fixture, &first](const std::string & variable) {
      if (!first) {
        fixture << ' ';
      }
      first = false;
      fixture << variable << '=' << start_state.getVariablePosition(variable);
    };
    for (const auto & variable : planning_group->getVariableNames()) {
      append_variable(variable);
    }
    // The jaws are outside the planning group but the collision-checked run carries them, so a
    // replay that only restores the group's variables cannot rebuild that run's scene state.
    for (const auto & joint : jaw_joints) {
      if (start_state.getRobotModel() && start_state.getRobotModel()->hasJointModel(joint)) {
        append_variable(joint);
      }
    }
  }
  fixture <<
    "] target xyz=(" << target.position.x << ", " << target.position.y << ", " <<
    target.position.z << ") quat=(" << target.orientation.x << ", " << target.orientation.y <<
    ", " << target.orientation.z << ", " << target.orientation.w << ") step=" <<
    std::setprecision(6) << step_m << " m";
  return fixture.str();
}

MotionCompletion MoveItMotionPort::execute_goal(const PendingGoal & pending) noexcept
{
  const auto fail = [&pending](
    MotionOutcome outcome, std::string detail, bool reached_terminal_stop = false) {
    return MotionCompletion{
    pending.correlation, outcome, std::move(detail), reached_terminal_stop, false};
  };
  // The planner was given the pose and refused it: the only planning failure that says anything
  // about the goal itself.
  const auto planner_refusal = [&pending](std::string detail) {
    return MotionCompletion{
    pending.correlation, MotionOutcome::kPlanningFailed, std::move(detail), false, true};
  };

  // Milestone 10 §6 (Card 062): a grasp that closed the jaws and failed is left along the reversed
  // approach first, then the goal's own plan starts from the standoff. The coordinator still sees
  // one motion; the completion records that the escape ran so it is never commanded twice.
  if (pending.goal.grasp_escape && !pending.goal.grasp_escape_leg) {
    PendingGoal escape = pending;
    escape.goal.planning_frame_from_tool0 =
      pending.goal.grasp_escape->planning_frame_from_standoff_tool0;
    escape.goal.path = MotionPathKind::kLinear;
    escape.goal.grasp_escape_leg = true;
    escape.goal.label = pending.goal.label + " grasp escape";
    escape.goal.free_space_plan_candidates = 1U;
    escape.goal.linear_egress_pose.reset();
    escape.goal.required_linear_continuation_pose.reset();
    escape.goal.required_linear_continuation_gripper_joint_position_m.reset();
    escape.goal.required_linear_continuation_vertical_margin_m = 0.0;
    escape.goal.required_postcontinuation_linear_retract_pose.reset();
    escape.goal.required_postcontinuation_linear_egress_pose.reset();
    escape.goal.required_postcontinuation_gripper_joint_position_m.reset();
    escape.goal.required_postmotion_linear_egress_pose.reset();
    escape.goal.required_postmotion_linear_egress_gripper_joint_position_m.reset();
    escape.goal.maximum_tool0_x_axis_tilt_rad.reset();
    escape.goal.path_constraints.reset();
    auto escape_completion = execute_goal(escape);
    if (escape_completion.outcome != MotionOutcome::kSucceeded) {
      escape_completion.detail = pending.goal.label + " failed during the grasp escape: " +
        escape_completion.detail;
      // Leaving the grasp says nothing about the goal pose, so it never spends a candidate.
      escape_completion.planner_refused_the_goal = false;
      return escape_completion;
    }
    PendingGoal remainder = pending;
    remainder.goal.grasp_escape.reset();
    remainder.goal.reset_planning_interface_before_plan = false;
    auto completion = execute_goal(remainder);
    completion.grasp_escape_executed = true;
    if (completion.outcome == MotionOutcome::kSucceeded) {
      completion.detail = pending.goal.label +
        " left the grasp along the reversed approach, then " + completion.detail;
      return completion;
    }
    completion.detail = pending.goal.label +
      " left the grasp along the reversed approach but then failed: " + completion.detail;
    // The escape executed to its verified end, so a planning refusal of the rest still means
    // "nothing commanded since the last verified segment" — and a planner verdict on the goal
    // keeps spending that candidate (rung 4), as it would have without the escape (Card 062
    // dev run 1). Any other not-started outcome cannot claim the arm never moved.
    if (completion.outcome != MotionOutcome::kPlanningFailed &&
      motion_definitely_not_started(completion.outcome))
    {
      completion.outcome = MotionOutcome::kExecutionFailed;
      completion.execution_reached_terminal_stop = true;
      completion.planner_refused_the_goal = false;
    }
    return completion;
  }

  // A placed product leaves the lane along the surveyed withdrawal, then repositions to the camera
  // viewpoint (a free-space goal). One Cartesian segment could be kinematically impossible, so the
  // legs are composed here and the coordinator still sees one retreat.
  if (pending.goal.linear_egress_pose) {
    PendingGoal egress = pending;
    egress.goal.planning_frame_from_tool0 = *pending.goal.linear_egress_pose;
    egress.goal.linear_egress_pose.reset();
    egress.goal.path = MotionPathKind::kLinear;
    egress.goal.path_constraints.reset();
    egress.goal.label = pending.goal.label + " egress";
    auto egress_completion = execute_goal(egress);
    if (egress_completion.outcome != MotionOutcome::kSucceeded) {
      egress_completion.detail = pending.goal.label + " failed during lane egress: " +
        egress_completion.detail;
      return egress_completion;
    }

    PendingGoal reposition = pending;
    reposition.goal.linear_egress_pose.reset();
    reposition.goal.label = pending.goal.label + " survey reposition";
    auto reposition_completion = execute_goal(reposition);
    if (reposition_completion.outcome == MotionOutcome::kSucceeded) {
      reposition_completion.detail = pending.goal.label +
        " lane egress and survey reposition reached";
      return reposition_completion;
    }
    reposition_completion.detail = pending.goal.label +
      " reached lane egress but failed during survey reposition: " +
      reposition_completion.detail;
    // The first leg executed, so a refusal to plan the second cannot claim the arm never moved.
    if (motion_definitely_not_started(reposition_completion.outcome)) {
      reposition_completion.outcome = MotionOutcome::kExecutionFailed;
      reposition_completion.execution_reached_terminal_stop = true;
      reposition_completion.planner_refused_the_goal = false;
    }
    return reposition_completion;
  }

  if (cancel_requested_.load(std::memory_order_acquire)) {
    return fail(MotionOutcome::kCanceled, "cancellation arrived before planning started");
  }

  if (pending.goal.reset_planning_interface_before_plan) {
    // The previous transfer is at a terminal, pre-motion boundary; rebuilding bounds MoveGroup's
    // action-goal bookkeeping.
    std::scoped_lock lock(move_group_mutex_);
    move_group_.reset();
  }

  // Record the geometry this segment is planned against. Nothing is commanded yet, so a refusal
  // here is a planning failure.
  std::optional<PreGraspSceneAuthority> scene_authority;
  if (config_.require_planning_scene_authority) {
    std::string detail;
    scene_authority = establish_scene_authority(
      pending.goal.planning_scene_lease_token, detail);
    if (!scene_authority) {
      RCLCPP_WARN(node_->get_logger(), "%s", detail.c_str());
      return fail(MotionOutcome::kPlanningFailed, detail);
    }
  }

  MoveGroupInterface * interface = nullptr;
  MoveGroupHolder * move_group_holder = nullptr;
  try {
    std::scoped_lock lock(move_group_mutex_);
    if (!move_group_) {
      // Constructed lazily so the coordinator can start before move_group is up.
      move_group_ = std::make_unique<MoveGroupHolder>(node_, config_);
    }
    move_group_holder = move_group_.get();
    interface = &move_group_->interface();
  } catch (const std::exception & error) {
    return fail(
      MotionOutcome::kUnavailable,
      std::string("MoveGroup is unavailable: ") + error.what());
  } catch (...) {
    return fail(MotionOutcome::kUnavailable, "MoveGroup is unavailable");
  }

  try {
    if (!interface->startStateMonitor(
        std::chrono::duration<double>(config_.startup_timeout).count()))
    {
      return fail(
        MotionOutcome::kUnavailable, "MoveIt never published a current robot state");
    }
    auto current_state = interface->getCurrentState(
      std::chrono::duration<double>(config_.current_state_timeout).count());
    if (!current_state) {
      return fail(
        MotionOutcome::kUnavailable, "MoveIt could not read the current planning start state");
    }
    const auto & robot_model = current_state->getRobotModel();
    const auto * planning_group =
      robot_model ? robot_model->getJointModelGroup(config_.planning_group) : nullptr;
    if (planning_group == nullptr ||
      !std::isfinite(config_.maximum_start_state_bounds_correction) ||
      config_.maximum_start_state_bounds_correction < 0.0)
    {
      return fail(
        MotionOutcome::kUnavailable,
        "MoveIt planning group or start-state bounds correction is unavailable");
    }
    for (const auto & variable : planning_group->getVariableNames()) {
      const auto & bounds = robot_model->getVariableBounds(variable);
      if (!bounds.position_bounded_) {
        continue;
      }
      const double measured = current_state->getVariablePosition(variable);
      const auto normalized = normalize_bounded_planning_position(
        measured, bounds.min_position_, bounds.max_position_,
        config_.maximum_start_state_bounds_correction);
      if (!normalized) {
        std::ostringstream detail;
        detail << std::setprecision(12) << "current joint '" << variable << "' position " <<
          measured << " is outside planning bounds [" << bounds.min_position_ << ", " <<
          bounds.max_position_ << "] by more than the allowed start-state correction " <<
          config_.maximum_start_state_bounds_correction;
        return fail(MotionOutcome::kPlanningFailed, detail.str());
      }
      if (normalized->corrected) {
        RCLCPP_WARN(
          node_->get_logger(),
          "clamping current joint '%s' from %.12f to planning bound %.12f "
          "(maximum correction %.6f)",
          variable.c_str(), measured, normalized->value,
          config_.maximum_start_state_bounds_correction);
        current_state->setVariablePosition(variable, normalized->value);
      }
    }
    current_state->update();
    interface->setStartState(*current_state);
    interface->setPlanningTime(
      std::chrono::duration<double>(pending.goal.planning_time).count());
    interface->setMaxVelocityScalingFactor(pending.goal.velocity_scaling);
    interface->setMaxAccelerationScalingFactor(pending.goal.acceleration_scaling);
    interface->setGoalPositionTolerance(pending.goal.position_tolerance_m);
    interface->setGoalOrientationTolerance(pending.goal.orientation_tolerance_rad);
    interface->clearPoseTargets();

    geometry_msgs::msg::Pose target = tf2::toMsg(pending.goal.planning_frame_from_tool0);
    const bool linear = pending.goal.path == MotionPathKind::kLinear;
    // A planner that cannot sample a goal state does not say which pose it rejected.
    RCLCPP_INFO(
      node_->get_logger(),
      "planning %s %s to %s pose xyz=(%.4f, %.4f, %.4f) quat=(%.4f, %.4f, %.4f, %.4f)",
      linear ? "linear" : "free-space", pending.goal.label.c_str(),
      config_.end_effector_link.c_str(), target.position.x, target.position.y, target.position.z,
      target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w);

    moveit_msgs::msg::RobotTrajectory trajectory;
    if (linear && pending.goal.grasp_escape_leg) {
      const auto escape = plan_grasp_escape(pending, *current_state, planning_group, trajectory);
      if (!escape.planned) {
        RCLCPP_WARN(
          node_->get_logger(), "%s refused: %s", pending.goal.label.c_str(),
          escape.detail.c_str());
        // A property of the scene around the arm, not of any grasp candidate (Card 043's rule).
        return fail(MotionOutcome::kPlanningFailed, pending.goal.label + ": " + escape.detail);
      }
      RCLCPP_INFO(node_->get_logger(), "%s: %s", pending.goal.label.c_str(), escape.detail.c_str());
      if (const auto timing = validate_trajectory_timing(
          observe_trajectory(trajectory),
          pending.goal.label);
        !timing.ok())
      {
        RCLCPP_WARN(node_->get_logger(), "%s", timing.detail.c_str());
        return fail(MotionOutcome::kPlanningFailed, timing.detail);
      }
    } else if (linear) {
      // Straight line, collision-checked at cartesian_step_m. Approach, retract, insert and retreat
      // have millimetres of side clearance, so a sampled detour would fail the solution validator.
      // Milestone 10 §6 (Card 046): a line that comes back partial is retried at a finer step
      // inside this segment's own planning budget before it becomes a verdict — the interpolator
      // makes one zero-timeout IK attempt per waypoint, so a stop can be an attempt that did not
      // converge where a finer interpolation follows the line.
      const auto linear_budget_deadline =
        std::chrono::steady_clock::now() + pending.goal.planning_time;
      const auto budget_remaining = [&linear_budget_deadline]() {
        return std::chrono::steady_clock::now() < linear_budget_deadline;
      };
      const auto cancel_requested = [this]() {
        return cancel_requested_.load(std::memory_order_acquire);
      };
      const auto interrupted = [&](const LinearPathRefinement & result)
        -> std::optional<MotionCompletion> {
        if (result.canceled || cancel_requested()) {
          return fail(
            MotionOutcome::kCanceled,
            pending.goal.label + ": cancellation arrived during Cartesian planning; " +
            result.receipt());
        }
        if (result.budget_spent || !budget_remaining()) {
          return fail(
            MotionOutcome::kPlanningFailed,
            pending.goal.label + ": Cartesian planning budget spent; " + result.receipt());
        }
        return std::nullopt;
      };
      const auto compute_line = [&interface, &target](
        double step_m, moveit_msgs::msg::RobotTrajectory & path,
        moveit_msgs::msg::MoveItErrorCodes & code) {
        code.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
        return interface->computeCartesianPath({target}, step_m, path, true, &code);
      };
      const LinearPathRefinement refinement = refine_linear_path(
        pending.goal.cartesian_step_m, pending.goal.minimum_cartesian_fraction, compute_line,
        budget_remaining, {kLinearRefinementStepFineM, kLinearRefinementStepFinerM},
        cancel_requested);
      if (const auto failure = interrupted(refinement)) {
        return *failure;
      }
      if (refinement.complete()) {
        trajectory = refinement.executed().path;
      } else {
        // Nothing was commanded, so this is a planning failure; the cause is established below.
        // The receipt quotes the goal's own step, exactly as it did before refinement existed.
        const LinearPathAttempt & primary = refinement.primary();
        const auto detail = describe_truncated_linear_path(
          *move_group_holder, pending.goal, target, primary.path, primary.fraction,
          primary.error_code);
        if (const auto failure = interrupted(refinement)) {
          return *failure;
        }
        const auto condition = establish_start_state_condition(*current_state);
        if (const auto failure = interrupted(refinement)) {
          return *failure;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "%s linear path refused at fraction %.4f: %s; %s; %s; %s; %s",
          pending.goal.label.c_str(), primary.fraction, detail.c_str(),
          describe_start_state_condition(condition).c_str(), describe_scene_world_objects().c_str(),
          describe_linear_refusal_fixture(
            *current_state, planning_group, config_.gripper_joint_names, target,
            pending.goal.cartesian_step_m).c_str(),
          refinement.receipt().c_str());
        if (condition.established && condition.in_collision) {
          // The scene has the arm already in contact: a property of the scene, not of this
          // grasp candidate (the other candidates plan from the same state), so it must not
          // consume one. Milestone 10 §6, Card 043.
          return fail(
            MotionOutcome::kPlanningFailed,
            pending.goal.label + ": " + describe_start_state_condition(condition) + "; " + detail);
        }
        return planner_refusal(detail);
      }
      // MoveIt's Cartesian path service ignores the return value of its time parameterisation and
      // answers SUCCESS, so an untimed path would command every waypoint at once.
      if (const auto timing = validate_trajectory_timing(
          observe_trajectory(trajectory),
          pending.goal.label);
        !timing.ok())
      {
        RCLCPP_WARN(node_->get_logger(), "%s", timing.detail.c_str());
        return planner_refusal(timing.detail);
      }
      if (pending.goal.required_postmotion_linear_egress_pose) {
        const auto & names = trajectory.joint_trajectory.joint_names;
        const auto & points = trajectory.joint_trajectory.points;
        if (points.empty() || points.back().positions.size() != names.size()) {
          return planner_refusal(
            pending.goal.label + " cannot reconstruct its endpoint for the required egress");
        }
        moveit::core::RobotState endpoint(*current_state);
        for (std::size_t index = 0U; index < names.size(); ++index) {
          endpoint.setVariablePosition(names[index], points.back().positions[index]);
        }
        for (const auto & joint : config_.gripper_joint_names) {
          if (!robot_model->hasJointModel(joint)) {
            return fail(
              MotionOutcome::kUnavailable,
              "required post-motion egress gripper joints are absent from the robot model");
          }
          endpoint.setVariablePosition(
            joint, *pending.goal.required_postmotion_linear_egress_gripper_joint_position_m);
        }
        endpoint.update();
        interface->setStartState(endpoint);
        const geometry_msgs::msg::Pose egress_target =
          tf2::toMsg(*pending.goal.required_postmotion_linear_egress_pose);
        // The proof shares the commanded line's deadline; it cannot replenish the segment's
        // planning budget. Cancellation/expiry also prevents its first service request.
        const auto compute_egress = [&interface, &egress_target](
          double step_m, moveit_msgs::msg::RobotTrajectory & path,
          moveit_msgs::msg::MoveItErrorCodes & code) {
          code.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
          return interface->computeCartesianPath({egress_target}, step_m, path, true, &code);
        };
        const LinearPathRefinement egress_refinement = refine_linear_path(
          pending.goal.cartesian_step_m, pending.goal.minimum_cartesian_fraction, compute_egress,
          budget_remaining, {kLinearRefinementStepFineM, kLinearRefinementStepFinerM},
          cancel_requested);
        if (const auto failure = interrupted(egress_refinement)) {
          interface->setStartState(*current_state);
          return *failure;
        }
        if (!egress_refinement.complete()) {
          const LinearPathAttempt & primary = egress_refinement.primary();
          MotionGoal egress_goal = pending.goal;
          egress_goal.label = pending.goal.label + " required post-release egress";
          const auto detail = describe_truncated_linear_path(
            *move_group_holder, egress_goal, egress_target, primary.path, primary.fraction,
            primary.error_code);
          interface->setStartState(*current_state);
          if (const auto failure = interrupted(egress_refinement)) {
            return *failure;
          }
          RCLCPP_WARN(
            node_->get_logger(), "%s; %s; %s", detail.c_str(),
            describe_linear_refusal_fixture(
              endpoint, planning_group, config_.gripper_joint_names, egress_target,
              pending.goal.cartesian_step_m).c_str(),
            egress_refinement.receipt().c_str());
          return planner_refusal(detail);
        }
        interface->setStartState(*current_state);
      }
    } else {
      // OMPL plans inside a box inset from the coordinator-owned outer AABB so projected
      // face states are not boundary states; the dense FK interpolation below checks the
      // unmodified outer box. The whole segment — constraints, continuation-endpoint
      // selection, budget split and the refusal receipt — lives in plan_free_space.
      const auto free_space = plan_free_space(
        pending, interface, *current_state, planning_group, robot_model, target, trajectory);
      if (!free_space.planned) {
        return free_space.failure;
      }
    }
    if (pending.goal.path_constraints || pending.goal.maximum_tool0_x_axis_tilt_rad) {
      const auto validation = validate_transfer_trajectory(
        *current_state, trajectory,
        pending.goal.path_constraints ? &*pending.goal.path_constraints : nullptr,
        pending.goal.maximum_tool0_x_axis_tilt_rad,
        config_.planning_frame, config_.end_effector_link);
      RCLCPP_INFO(
        node_->get_logger(),
        "planned %s upright metric: tool0 +X maximum tilt from planning +Z %.6f rad "
        "(%.3f deg) at dense sample %zu of %zu",
        pending.goal.label.c_str(), validation.maximum_tool0_x_tilt_rad,
        validation.maximum_tool0_x_tilt_rad * 180.0 / std::acos(-1.0),
        validation.maximum_tilt_sample, validation.sample_count);
      if (!validation.violation.empty()) {
        RCLCPP_WARN(node_->get_logger(), "%s", validation.violation.c_str());
        return planner_refusal(validation.violation);
      }
    }
    if (cancel_requested_.load(std::memory_order_acquire)) {
      return fail(MotionOutcome::kCanceled, "cancellation arrived before execution started");
    }

    const auto & trajectory_points = trajectory.joint_trajectory.points;
    const double planned_duration_s = trajectory_points.empty() ? 0.0 :
      static_cast<double>(trajectory_points.back().time_from_start.sec) +
      static_cast<double>(trajectory_points.back().time_from_start.nanosec) * 1e-9;
    RCLCPP_INFO(
      node_->get_logger(), "planned %s trajectory has %zu points and %.3f s duration",
      pending.goal.label.c_str(), trajectory_points.size(), planned_duration_s);

    // Milestone 10 §6, Card 061: a trajectory that fits one goal and already stops at both ends
    // keeps the planner's timing; only real slices are re-timed stop-to-stop.
    const auto division = divide_into_controller_goals(
      trajectory, *current_state, config_.planning_group, pending.goal.velocity_scaling,
      pending.goal.acceleration_scaling, pending.goal.maximum_controller_goal_duration);
    if (!division.ok()) {
      return planner_refusal(
        pending.goal.label + " trajectory cannot be divided into bounded controller goals: " +
        division.refusal);
    }
    if (division.planner_timing_kept) {
      RCLCPP_INFO(
        node_->get_logger(),
        "%s fits one controller goal and is at rest at both ends: executing with the planner's "
        "own timing", pending.goal.label.c_str());
    }
    if (division.fallback_retimes > 0U) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s: %zu controller slice(s) re-timed at the fallback path tolerance %g after the strict "
        "tolerance %g failed", pending.goal.label.c_str(), division.fallback_retimes,
        kControllerSliceFallbackPathTolerance, kControllerSlicePathTolerance);
    }
    const auto & execution_trajectories = division.goals;
    for (const auto & execution_trajectory : execution_trajectories) {
      if (pending.goal.path_constraints || pending.goal.maximum_tool0_x_axis_tilt_rad) {
        const auto validation = validate_transfer_trajectory(
          *current_state, execution_trajectory,
          pending.goal.path_constraints ? &*pending.goal.path_constraints : nullptr,
          pending.goal.maximum_tool0_x_axis_tilt_rad,
          config_.planning_frame, config_.end_effector_link);
        if (!validation.violation.empty()) {
          return planner_refusal(
            pending.goal.label + " bounded controller goal failed post-retiming validation: " +
            validation.violation);
        }
      }
    }
    if (execution_trajectories.size() > 1U) {
      RCLCPP_INFO(
        node_->get_logger(),
        "%s will execute as %zu contiguous controller goals, each at most %.3f s",
        pending.goal.label.c_str(), execution_trajectories.size(),
        std::chrono::duration<double>(pending.goal.maximum_controller_goal_duration).count());
    }

    begin_segment_observation();
    for (std::size_t slice = 0U; slice < execution_trajectories.size(); ++slice) {
      if (cancel_requested_.load(std::memory_order_acquire)) {
        return fail(
          MotionOutcome::kCanceled, "cancellation arrived between controller goals", slice > 0U);
      }
      // Each gate authorizes only this next submission. Keep the original plan provenance:
      // a changed scene needs a new collision proof for every remaining controller goal.
      if (scene_authority) {
        if (const auto lost = scene_authority_lost(
            *scene_authority, pending.goal.planning_scene_lease_token,
            execution_trajectories[slice], robot_model, !pending.goal.grasp_escape_leg);
          !lost.empty())
        {
          RCLCPP_WARN(node_->get_logger(), "%s", lost.c_str());
          const auto refusal = cancel_requested_.load(std::memory_order_acquire) ||
            stopping_.load(std::memory_order_acquire) ? MotionOutcome::kCanceled :
            (slice == 0U ? MotionOutcome::kPlanningFailed : MotionOutcome::kExecutionFailed);
          return fail(
            refusal,
            pending.goal.label + " refused controller goal " + std::to_string(slice + 1U) +
            ": " + lost, slice > 0U);
        }
      }
      const std::string execution_label = execution_trajectories.size() == 1U ?
        pending.goal.label : pending.goal.label + " controller goal " +
        std::to_string(slice + 1U) + "/" + std::to_string(execution_trajectories.size());
      const auto & execution_trajectory = execution_trajectories[slice];
      if (cancel_requested_.load(std::memory_order_acquire) ||
        stopping_.load(std::memory_order_acquire))
      {
        return fail(
          MotionOutcome::kCanceled, "cancellation arrived before controller submission",
          slice > 0U);
      }
      execute_called_.store(true, std::memory_order_release);
      const int execute_code = interface->execute(
        execution_trajectory, config_.execution_controllers).val;
      // Only CONTROL_FAILED (controller reported a terminal status) proves the segment is over
      // at the backend. TIMED_OUT does not (the duration monitor cancels the controllers
      // without waiting for their terminal results), and neither do the other execute failures
      // (preemption, rejected goal, unresponsive backend): those leave the stop unestablished.
      const auto verdict = classify_execute_result(execute_code, execution_label);
      const auto & executed = verdict.planning;
      if (!executed.ok()) {
        auto detail = with_moveit_reason(executed.detail, execute_code);
        // CONTROL_FAILED discards the controller's reason; restore it with the velocity evidence.
        if (execute_code == moveit_msgs::msg::MoveItErrorCodes::CONTROL_FAILED) {
          if (const auto abort = observed_controller_abort()) {
            const auto evidence = segment_velocity_evidence(
              *move_group_holder, execution_trajectory);
            detail += "; " + describe_controller_abort(*abort, evidence);
          } else {
            detail += "; no controller abort was reported on " + config_.controller_log_topic +
              " for this segment";
          }
          RCLCPP_WARN(node_->get_logger(), "%s", detail.c_str());
        }
        if (cancel_requested_.load(std::memory_order_acquire)) {
          return fail(MotionOutcome::kCanceled, "motion stopped after cancellation: " + detail);
        }
        return fail(verdict.outcome, detail, verdict.reached_terminal_stop);
      }
    }
    return MotionCompletion{
      pending.correlation, MotionOutcome::kSucceeded,
      pending.goal.label + " pose reached through " +
      std::to_string(execution_trajectories.size()) + " bounded controller goal(s)"};
  } catch (const std::exception & error) {
    // The arm may already be moving, so this cannot claim a not-started outcome.
    return fail(
      MotionOutcome::kExecutionFailed,
      std::string("MoveIt raised during plan-and-execute: ") + error.what());
  } catch (...) {
    return fail(MotionOutcome::kExecutionFailed, "MoveIt raised during plan-and-execute");
  }
}

// Plans one free-space segment: optional path constraints, continuation-endpoint selection, and
// the goal's planning budget split across its free-space candidates. Fills `trajectory` with the
// shortest accepted plan, or returns the refusal — carrying whether it was the planner's verdict
// on the goal, which is what the coordinator spends a grasp candidate on.
bool MoveItMotionPort::ensure_race_pipelines(const moveit::core::RobotModelConstPtr & robot_model)
{
  if (race_pipelines_) {
    return true;
  }
  if (race_unavailable_) {
    return false;
  }
  try {
    // The fallback pipeline gets its OWN model load so its JMG allocates its own KDL solver
    // instance (IKConstraintSampler shares jmg->getSolverInstance() across every context built
    // on that model, and KDL is not thread-safe — the survey_viewpoint SIGSEGVs).
    auto fallback_loader = std::make_shared<robot_model_loader::RobotModelLoader>(
      node_, robot_model_loader::RobotModelLoader::Options());
    auto fallback_model = fallback_loader->getModel();
    if (!fallback_model) {
      throw std::runtime_error("the fallback robot model failed to load");
    }
    auto primary_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
      robot_model, node_, kRacePrimaryPipeline);
    auto fallback_pipeline = std::make_shared<planning_pipeline::PlanningPipeline>(
      fallback_model, node_, kRaceFallbackPipeline);
    race_pipelines_ = std::make_unique<RacePipelines>(
      robot_model, fallback_loader, primary_pipeline, fallback_pipeline);
    RCLCPP_INFO(
      node_->get_logger(),
      "free-space race ready: %s (RRTConnect) + %s (PRM) inside each unchanged slice "
      "(separate IK solver instances per pipeline)",
      kRacePrimaryPipeline, kRaceFallbackPipeline);
    return true;
  } catch (const std::exception & error) {
    race_unavailable_ = true;
    RCLCPP_WARN(
      node_->get_logger(),
      "free-space race unavailable (%s); planning falls back to the move_group service",
      error.what());
    return false;
  }
}

std::optional<MoveItMotionPort::RaceRun> MoveItMotionPort::race_plan(
  const moveit_msgs::msg::MotionPlanRequest & base_request, double slice_seconds,
  const moveit::core::RobotModelConstPtr & robot_model)
{
  // One snapshot of move_group's own scene — the service the refusal receipts already use —
  // so both pipelines plan against identical collision content, and against move_group's
  // collision model: padding, scale and allowed-collision matrix included (Milestone 10 §6,
  // Cards 053 and 068). A snapshot that does not arrive, or cannot supply that model, means
  // this slice runs the today-path service plan instead.
  if (!planning_scene_client_ || !planning_scene_client_->service_is_ready()) {
    return std::nullopt;
  }
  auto scene_request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  scene_request->components.components = kRaceSceneComponents;
  auto scene_future = planning_scene_client_->async_send_request(scene_request);
  if (scene_future.wait_for(config_.state_validity_timeout) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(scene_future);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000,
      "free-space race skipped: /get_planning_scene did not answer within %ld ms",
      config_.state_validity_timeout.count());
    return std::nullopt;
  }
  auto scene_response = scene_future.get();
  if (!scene_response) {
    return std::nullopt;
  }
  // One scene MESSAGE, applied onto each pipeline's own model: identical collision content for
  // both winners, each thread internally consistent (scene model == pipeline model).
  auto primary_build = make_race_scene(robot_model, scene_response->scene);
  auto fallback_build =
    make_race_scene(race_pipelines_->fallback_loader->getModel(), scene_response->scene);
  if (!primary_build.scene || !fallback_build.scene) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 10000,
      "free-space race skipped: %s; planning through the move_group service",
      (!primary_build.scene ? primary_build.refusal : fallback_build.refusal).c_str());
    return std::nullopt;
  }
  const planning_scene::PlanningSceneConstPtr primary_scene = primary_build.scene;
  const planning_scene::PlanningSceneConstPtr fallback_scene = fallback_build.scene;

  auto primary = base_request;
  primary.pipeline_id = kRacePrimaryPipeline;
  primary.planner_id = config_.planner_id;
  auto fallback = base_request;
  fallback.pipeline_id = kRaceFallbackPipeline;
  fallback.planner_id = kRaceFallbackPlannerId;

  // The race itself: two threads, one pipeline + one scene each, first success terminates the
  // loser (the stopping-criterion behaviour planWithParallelPipelines provides upstream — we
  // run our own so each pipeline stays paired with its own model's IK solver).
  planning_interface::MotionPlanResponse primary_response;
  planning_interface::MotionPlanResponse fallback_response;
  double primary_seconds = 0.0;
  double fallback_seconds = 0.0;
  std::atomic<bool> primary_done{false};
  std::atomic<bool> fallback_done{false};
  std::atomic<bool> primary_ok{false};
  std::atomic<bool> fallback_ok{false};
  const auto wall_start = std::chrono::steady_clock::now();
  const auto run_one = [&](
    planning_interface::MotionPlanResponse & response, double & seconds,
    std::atomic<bool> & done, std::atomic<bool> & self_ok,
    const std::atomic<bool> & other_done,
    const planning_pipeline::PlanningPipelinePtr & own_pipeline,
    const planning_pipeline::PlanningPipelinePtr & other_pipeline,
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit_msgs::msg::MotionPlanRequest & request)
  {
    const auto started = std::chrono::steady_clock::now();
    try {
      response = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
        request, scene, {{request.pipeline_id, own_pipeline}});
      const bool ok =
        response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
        response.trajectory;
      self_ok.store(ok, std::memory_order_release);
      if (ok && !other_done.load(std::memory_order_acquire)) {
        other_pipeline->terminate();
      }
    } catch (const std::exception & error) {
      response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
      RCLCPP_WARN(
        node_->get_logger(), "free-space race thread raised (%s)", error.what());
    }
    seconds = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    done.store(true, std::memory_order_release);
  };
  std::thread primary_thread(
    run_one, std::ref(primary_response), std::ref(primary_seconds),
    std::ref(primary_done), std::ref(primary_ok), std::cref(fallback_done),
    race_pipelines_->primary_pipeline, race_pipelines_->fallback_pipeline,
    primary_scene, primary);
  std::thread fallback_thread(
    run_one, std::ref(fallback_response), std::ref(fallback_seconds),
    std::ref(fallback_done), std::ref(fallback_ok), std::cref(primary_done),
    race_pipelines_->fallback_pipeline, race_pipelines_->primary_pipeline,
    fallback_scene, fallback);
  primary_thread.join();
  fallback_thread.join();
  const double wall_seconds = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - wall_start).count();

  const auto outcome_of = [](
    const std::string & name, const planning_interface::MotionPlanResponse & response,
    double seconds) {
    RacePipelineOutcome outcome;
    outcome.name = name;
    outcome.error_code = response.error_code.val;
    outcome.seconds = seconds;
    outcome.has_trajectory =
      response.trajectory != nullptr &&
      response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return outcome;
  };
  const std::vector<RacePipelineOutcome> outcomes = {
    outcome_of(kRacePrimaryPipeline, primary_response, primary_seconds),
    outcome_of(kRaceFallbackPipeline, fallback_response, fallback_seconds)};
  RaceRun run;
  run.decision = decide_race(outcomes, wall_seconds);
  if (run.decision.success) {
    const auto duration_of = [](
      const planning_interface::MotionPlanResponse & response) {
      moveit_msgs::msg::RobotTrajectory message;
      response.trajectory->getRobotTrajectoryMsg(message);
      if (message.joint_trajectory.points.empty()) {
        return std::numeric_limits<double>::infinity();
      }
      const auto & finish = message.joint_trajectory.points.back().time_from_start;
      return static_cast<double>(finish.sec) +
             static_cast<double>(finish.nanosec) * 1e-9;
    };
    const planning_interface::MotionPlanResponse * winner = nullptr;
    if (primary_ok.load(std::memory_order_acquire) && primary_response.trajectory) {
      winner = &primary_response;
    }
    if (fallback_ok.load(std::memory_order_acquire) && fallback_response.trajectory) {
      if (!winner || duration_of(fallback_response) < duration_of(*winner)) {
        winner = &fallback_response;
      }
    }
    if (!winner) {
      return std::nullopt;
    }
    winner->trajectory->getRobotTrajectoryMsg(run.trajectory);
  }
  return run;
}

MoveItMotionPort::FreeSpacePlanResult MoveItMotionPort::plan_free_space(
  const PendingGoal & pending, MoveGroupInterface * interface,
  const moveit::core::RobotState & current_state,
  const moveit::core::JointModelGroup * planning_group,
  const moveit::core::RobotModelConstPtr & robot_model,
  const geometry_msgs::msg::Pose & target, moveit_msgs::msg::RobotTrajectory & trajectory)
{
  const auto fail =
    [&pending](MotionOutcome outcome, std::string detail, bool reached_terminal_stop = false) {
      return FreeSpacePlanResult{
      false, MotionCompletion{
        pending.correlation, outcome, std::move(detail), reached_terminal_stop, false}};
    };
  const auto planner_refusal = [&pending](std::string detail) {
    return FreeSpacePlanResult{
    false, MotionCompletion{
      pending.correlation, MotionOutcome::kPlanningFailed, std::move(detail), false, true}};
  };
  // OMPL plans inside a box inset from the coordinator-owned outer AABB so projected face
  // states are not boundary states. Dense FK interpolation below checks every tool0 sample
  // against the unmodified outer box.
  PathConstraintGuard path_constraint_guard(*interface);
  interface->clearPathConstraints();
  if (pending.goal.path_constraints) {
    const std::string & description = pending.goal.path_constraints->description;
    RCLCPP_INFO(
      node_->get_logger(),
      "planning free-space %s with enforced post-plan constraints: %s",
      pending.goal.label.c_str(),
      description.empty() ? "present" : description.c_str());
    MotionPathConstraints planner_constraints = *pending.goal.path_constraints;
    if (planner_constraints.position_box) {
      const auto inset = inset_motion_path_position_box(
        *planner_constraints.position_box, kPathConstraintPlanningInsetM);
      if (!inset) {
        return planner_refusal(
          "transfer box is too small for the fail-closed planning inset");
      }
      planner_constraints.position_box = *inset;
    }
    interface->setPathConstraints(
      to_moveit_path_constraints(
        planner_constraints, config_.planning_frame, config_.end_effector_link));
  }
  const auto planning_deadline = std::chrono::steady_clock::now() + pending.goal.planning_time;
  std::vector<moveit::core::RobotState> continuation_endpoints;
  if (pending.goal.required_linear_continuation_pose) {
    auto selection = select_continuation_endpoints(
      *interface, state_validity_client_, current_state, planning_group, robot_model,
      config_, pending.goal, planning_deadline);
    if (selection.endpoints.empty()) {
      // The selection runs before the planner and reports its own failure as a refusal, so
      // the start-state condition has to be established here, or a scene that would have
      // refused every candidate anyway spends them one selection at a time first.
      const auto condition = establish_start_state_condition(current_state);
      RCLCPP_WARN(
        node_->get_logger(),
        "%s continuation endpoint selection refused: %s; %s; %s",
        pending.goal.label.c_str(), selection.failure_detail.c_str(),
        describe_start_state_condition(condition).c_str(),
        describe_scene_world_objects().c_str());
      if (selection.planner_refused) {
        if (condition.established && condition.in_collision) {
          return fail(
            MotionOutcome::kPlanningFailed, pending.goal.label + ": " +
            describe_start_state_condition(condition) + "; " + selection.failure_detail);
        }
        return planner_refusal(std::move(selection.failure_detail));
      }
      return fail(selection.failure_outcome, std::move(selection.failure_detail));
    }
    continuation_endpoints = std::move(selection.endpoints);
    RCLCPP_INFO(
      node_->get_logger(),
      "%s retained %zu distinct IK endpoints after proving each required linear continuation",
      pending.goal.label.c_str(), continuation_endpoints.size());
  } else if (!interface->setPoseTarget(target, config_.end_effector_link)) {
    return planner_refusal("pose target was rejected by the robot model");
  }

  double selected_duration_s = std::numeric_limits<double>::infinity();
  int last_plan_code = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
  double last_attempt_seconds = -1.0;
  double last_slice_seconds = -1.0;
  std::string last_post_plan_rejection;
  // Card 053: per-pipeline receipt of the last raced slice (empty on the service path).
  std::string last_race_receipt;
  const bool race_ready = ensure_race_pipelines(robot_model);
  // Card 074 (Milestone 10 §6): the split of this floor is not a budget — every candidate keeps
  // a first-pass slice, and the reserved remainder is one longer attempt for the first
  // candidate that TIMED_OUT (the preferred endpoint first: the loop stops at the first
  // endpoint that plans, and its aggregate share inside the round is half the floor). An even
  // split handed that endpoint only floor/N seconds; Card 066's dense dev run 3 refused five
  // rounds that way — 21 × ~15 s timeouts against the same scene, then the same endpoint
  // planned in 11.1 s — spending 300 s of the whole-task bound on refusals.
  const auto slice_plan = plan_free_space_slices(
    std::chrono::duration<double>(pending.goal.planning_time).count(),
    pending.goal.free_space_plan_candidates);
  if (!(slice_plan.first_pass_slice_seconds > 0.0)) {
    return fail(
      MotionOutcome::kPlanningFailed,
      pending.goal.label + " has no free-space planning budget to divide");
  }
  const auto attempt_candidate =
    [&](std::size_t candidate, double slice_seconds) -> FreeSpaceSliceAttempt {
      if (cancel_requested_.load(std::memory_order_acquire)) {
        return FreeSpaceSliceAttempt{false, false, true, false};
      }
      if (!continuation_endpoints.empty() &&
        !interface->setJointValueTarget(
          continuation_endpoints[candidate % continuation_endpoints.size()]))
      {
        return FreeSpaceSliceAttempt{false, false, false, true};
      }
      interface->setPlanningTime(slice_seconds);
      interface->setNumPlanningAttempts(1);
      const auto attempt_started = std::chrono::steady_clock::now();
      MoveGroupInterface::Plan plan;
      last_race_receipt.clear();
      bool raced = false;
      if (race_ready) {
        // The exact request plan() would send (start state, goal, path constraints, scaling,
        // tolerances), raced on both pipelines inside this slice — Milestone 10 §6, Card 053.
        moveit_msgs::msg::MotionPlanRequest base_request;
        interface->constructMotionPlanRequest(base_request);
        base_request.allowed_planning_time = slice_seconds;
        base_request.num_planning_attempts = 1;
        if (auto run = race_plan(base_request, slice_seconds, robot_model); run) {
          raced = true;
          last_plan_code = run->decision.classification_code;
          last_attempt_seconds = run->decision.wall_seconds;
          last_slice_seconds = slice_seconds;
          last_race_receipt = run->decision.receipt;
          if (run->decision.success) {
            plan.trajectory = std::move(run->trajectory);
          }
        }
      }
      if (!raced) {
        last_plan_code = interface->plan(plan).val;
        last_attempt_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - attempt_started).count();
        last_slice_seconds = slice_seconds;
      }
      const auto planned = classify_moveit_error(
        PlanningPhase::kPlan, last_plan_code, pending.goal.label);
      if (!planned.ok()) {
        // One line per refused attempt: how long the planner actually had, against the slice
        // it was given and the budget left, with MoveIt's own code and name — and, on a raced
        // slice, what each pipeline answered (Milestone 10 §6, Cards 043 and 053).
        const auto left = planning_deadline - std::chrono::steady_clock::now();
        const double left_seconds =
          std::max(0.0, std::chrono::duration<double>(left).count());
        const std::string race_note =
          last_race_receipt.empty() ? std::string() : "; race: " + last_race_receipt;
        RCLCPP_WARN(
          node_->get_logger(),
          "%s free-space candidate %zu/%zu plan failed after %.3f s of a %.3f s slice "
          "(%.3f s of budget remaining): MoveIt %s (code %d)%s",
          pending.goal.label.c_str(), candidate + 1U,
          pending.goal.free_space_plan_candidates, last_attempt_seconds, last_slice_seconds,
          left_seconds, moveit_error_name(last_plan_code), last_plan_code, race_note.c_str());
        return FreeSpaceSliceAttempt{
        false, planned.status == PlanningStatus::kPlanningTimedOut, false, false};
      }
      if (pending.goal.path_constraints || pending.goal.maximum_tool0_x_axis_tilt_rad) {
        const auto validation = validate_transfer_trajectory(
          current_state, plan.trajectory,
          pending.goal.path_constraints ? &*pending.goal.path_constraints : nullptr,
          pending.goal.maximum_tool0_x_axis_tilt_rad,
          config_.planning_frame, config_.end_effector_link);
        last_post_plan_rejection = validation.violation;
        if (!validation.violation.empty()) {
          RCLCPP_WARN(node_->get_logger(), "%s", validation.violation.c_str());
          return FreeSpaceSliceAttempt{false, false, false, false};
        }
      }
      const auto & points = plan.trajectory.joint_trajectory.points;
      if (points.empty()) {
        last_post_plan_rejection =
          pending.goal.label + " planner returned an empty successful trajectory";
        return FreeSpaceSliceAttempt{false, false, false, false};
      }
      const auto & finish = points.back().time_from_start;
      const double duration_s = static_cast<double>(finish.sec) +
        static_cast<double>(finish.nanosec) * 1e-9;
      if (!std::isfinite(duration_s) || duration_s <= 0.0) {
        last_post_plan_rejection =
          pending.goal.label + " planner returned a trajectory with invalid duration";
        return FreeSpaceSliceAttempt{false, false, false, false};
      }
      RCLCPP_INFO(
        node_->get_logger(), "%s free-space candidate %zu/%zu has %.3f s duration",
        pending.goal.label.c_str(), candidate + 1U,
        pending.goal.free_space_plan_candidates, duration_s);
      if (duration_s < selected_duration_s) {
        selected_duration_s = duration_s;
        trajectory = std::move(plan.trajectory);
      }
      return FreeSpaceSliceAttempt{true, false, false, false};
    };
  const auto round = run_free_space_slice_round(
    slice_plan, pending.goal.free_space_plan_candidates,
    // Pre-grasp endpoints are ranked by joint travel; stop at the first planned one. The rest
    // are fallbacks for a disconnected IK branch.
    !continuation_endpoints.empty(),
    [&planning_deadline]() {
      return planning_deadline - std::chrono::steady_clock::now();
    },
    attempt_candidate);
  if (round.status == FreeSpaceSliceRoundStatus::kCanceled) {
    return fail(MotionOutcome::kCanceled, "cancellation arrived during planning");
  }
  if (round.status == FreeSpaceSliceRoundStatus::kEndpointRejected) {
    return planner_refusal(
      pending.goal.label + " collision-free continuation endpoint was rejected as a "
      "joint goal");
  }
  const std::size_t accepted_candidates = round.accepted_candidates;
  if (accepted_candidates == 0U) {
    // The refusal receipt. One line carrying MoveIt's own code and name, the time the last
    // attempt actually had against its slice, what of the budget was spent, the scene the
    // plan was made against, and what that scene says about the state the plan would have
    // started from — which is what separates an expired budget from a scene that refused
    // (Milestone 10 §6, Card 043). Emitted only here: the green path never pays for it.
    const auto refusal_receipt =
      [&](const std::string & reason, const std::string & start_state) {
        const auto now = std::chrono::steady_clock::now();
        const auto start = planning_deadline - pending.goal.planning_time;
        const double budget_seconds =
          std::chrono::duration<double>(pending.goal.planning_time).count();
        const double spent_seconds = std::chrono::duration<double>(now - start).count();
        const double left_seconds =
          std::max(0.0, std::chrono::duration<double>(planning_deadline - now).count());
        const std::string scene = describe_scene_world_objects();
        const std::string race_note =
          last_race_receipt.empty() ? std::string() : "; race: " + last_race_receipt;
        if (last_plan_code == moveit_msgs::msg::MoveItErrorCodes::UNDEFINED) {
          RCLCPP_WARN(
            node_->get_logger(),
            "%s free-space plan refused: %s; no planning attempt ran; spent %.3f s of the "
            "%.3f s planning budget (%.3f s remaining); %s; %s",
            pending.goal.label.c_str(), reason.c_str(), spent_seconds, budget_seconds,
            left_seconds, start_state.c_str(), scene.c_str());
          return;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "%s free-space plan refused: %s; MoveIt %s (code %d); last attempt %.3f s of a "
          "%.3f s slice; spent %.3f s of the %.3f s planning budget (%.3f s remaining); "
          "%s; %s%s",
          pending.goal.label.c_str(), reason.c_str(), moveit_error_name(last_plan_code),
          last_plan_code, last_attempt_seconds, last_slice_seconds, spent_seconds,
          budget_seconds, left_seconds, start_state.c_str(), scene.c_str(),
          race_note.c_str());
      };
    if (!last_post_plan_rejection.empty()) {
      // A plan was produced, so MoveIt's own start-state adapter passed: the pose is what
      // refused, and the receipt says the start state was ruled out.
      refusal_receipt(
        last_post_plan_rejection,
        "start state passed MoveIt's start-state check (a plan was produced from it)");
      return planner_refusal(last_post_plan_rejection);
    }
    if (last_plan_code == moveit_msgs::msg::MoveItErrorCodes::UNDEFINED) {
      refusal_receipt(
        "the planning budget expired before an attempt ran", "no planning attempt ran");
      return fail(
        MotionOutcome::kTimedOut, pending.goal.label + " exhausted its planning budget");
    }
    const auto condition = establish_start_state_condition(current_state);
    const auto planned = classify_plan_failure(
      last_plan_code, condition,
      PlanningSlice{last_attempt_seconds, last_slice_seconds}, pending.goal.label);
    const auto outcome = motion_outcome_for(PlanningPhase::kPlan, planned.status);
    auto detail = with_moveit_reason(planned.detail, last_plan_code);
    refusal_receipt(
      "no free-space candidate produced a plan",
      describe_start_state_condition(condition));
    // Only a planning rejection is a verdict on this pose. A start state the scene refuses
    // is a property of the scene — every candidate plans from the same state — so it is
    // classified kStartStateInvalid and never reaches this branch; timeouts likewise.
    const bool pose_verdict = planned.status == PlanningStatus::kPlanningRejected;
    return (outcome == MotionOutcome::kPlanningFailed && pose_verdict) ?
           planner_refusal(std::move(detail)) : fail(outcome, std::move(detail));
  }
  if (pending.goal.free_space_plan_candidates > 1U) {
    RCLCPP_INFO(
      node_->get_logger(), "%s selected the shortest of %zu valid free-space candidates",
      pending.goal.label.c_str(), accepted_candidates);
  }
  return FreeSpacePlanResult{true, MotionCompletion{}};
}

void MoveItMotionPort::begin_segment_observation()
{
  {
    std::scoped_lock lock(controller_abort_mutex_);
    controller_abort_.reset();
  }
  std::scoped_lock lock(joint_speed_mutex_);
  joint_peak_speed_.clear();
  joint_state_samples_ = 0U;
}

std::optional<ControllerAbortReport> MoveItMotionPort::observed_controller_abort()
{
  // The line is logged before MoveIt answers, so this waits on transport only. Polled because the
  // wait is short, once per failed segment, and must not hold a completion open.
  const auto deadline = std::chrono::steady_clock::now() + config_.controller_abort_report_timeout;
  for (;; ) {
    {
      std::scoped_lock lock(controller_abort_mutex_);
      if (controller_abort_) {
        return controller_abort_;
      }
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return std::nullopt;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

SegmentVelocityEvidence MoveItMotionPort::segment_velocity_evidence(
  MoveGroupHolder & holder, const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  SegmentVelocityEvidence evidence;
  std::unordered_map<std::string, double> peaks;
  {
    std::scoped_lock lock(joint_speed_mutex_);
    evidence.samples = joint_state_samples_;
    peaks = joint_peak_speed_;
  }
  if (evidence.samples == 0U) {
    return evidence;
  }
  // URDF bounds, not the model's: joint_limits.yaml margins the RobotModel's bounds, so reaching
  // those is just an aggressive plan. The simulator clamps to the URDF limit, so feedback velocity
  // on it is a motion the controller did not command.
  const auto & names = trajectory.joint_trajectory.joint_names;
  urdf::ModelInterfaceSharedPtr description;
  try {
    if (const auto & model = holder.interface().getRobotModel()) {
      description = model->getURDF();
    }
  } catch (...) {
    description.reset();
  }
  if (!description) {
    return evidence;
  }
  // Tracked over the whole loop: the unsaturated report needs the closest approach to the bound.
  double nearest_fraction = -1.0;
  std::size_t nearest_index = names.size();
  double nearest_limit = 0.0;
  for (std::size_t index = 0; index < names.size(); ++index) {
    const auto joint = description->getJoint(names[index]);
    if (!joint || !joint->limits || joint->limits->velocity <= 0.0) {
      continue;
    }
    const auto peak = peaks.find(names[index]);
    if (peak == peaks.end()) {
      continue;
    }
    const double limit = joint->limits->velocity;
    if (const double fraction = peak->second / limit; fraction > nearest_fraction) {
      nearest_fraction = fraction;
      nearest_index = index;
      nearest_limit = limit;
    }
    // The simulator writes the clamp back exactly; the 0.1% tolerance is for message precision.
    if (peak->second < limit * 0.999) {
      continue;
    }
    // Report only the first joint over: the impulse moves one joint, and listing all would
    // fragment the taxonomy signature.
    if (evidence.joint.empty()) {
      evidence.joint = names[index];
      evidence.urdf_limit = limit;
      evidence.planned_maximum = planned_joint_speed_maximum(trajectory, index);
    }
  }
  if (nearest_index < names.size()) {
    evidence.nearest_joint = names[nearest_index];
    evidence.nearest_peak = peaks.at(names[nearest_index]);
    evidence.nearest_limit = nearest_limit;
    evidence.nearest_planned = planned_joint_speed_maximum(trajectory, nearest_index);
  }
  return evidence;
}

PreGraspPlanningAuthorityConfig MoveItMotionPort::scene_authority_config() const
{
  PreGraspPlanningAuthorityConfig authority_config;
  authority_config.planning_frame = config_.planning_frame;
  authority_config.maximum_scene_age = config_.maximum_scene_status_age;
  // Attachment age is unused here but must be valid for the shared validator.
  authority_config.maximum_attachment_age = config_.maximum_scene_status_age;
  authority_config.maximum_future_skew = config_.maximum_scene_status_future_skew;
  return authority_config;
}

namespace
{

// True for outcomes that describe a scene in motion, not one that cannot be trusted. The gate
// reports these as waits: STATE_SYNCHRONIZING at each reconciliation start, a status not yet
// received after connecting, a dependency being re-tested next cycle, aged-out evidence that
// clears on the next observation. The mutation lease is still answered as a rejection, and the
// status carrying its release lags the release by a few ms; refusing during a held product
// latches an operator-required fault, so wait. Bounded by scene_authority_settle_timeout.
[[nodiscard]] bool transient_authority(const PlanningSceneAuthorityResult & result) noexcept
{
  return result.decision == PreGraspAuthorityDecision::kWait ||
         result.error == PreGraspAuthorityErrorCode::kSceneLeaseActive;
}

}  // namespace

// Polls the projection status until the authority is settled, one way or the other. A rejection
// that is not transient is returned at once, because a rejection is an answer.
PlanningSceneAuthorityResult MoveItMotionPort::settled_scene_authority(
  const std::optional<PreGraspSceneAuthority> & baseline,
  const std::optional<PlanningSceneLeaseAuthority> & lease, std::string & projector_detail,
  std::chrono::steady_clock::time_point outer_deadline)
{
  const auto deadline = std::min(
    outer_deadline, std::chrono::steady_clock::now() + config_.scene_authority_settle_timeout);
  PlanningSceneAuthorityResult result;
  result.detail = "planning-scene projection status has never been received";
  projector_detail.clear();
  for (;; ) {
    restocker_interfaces::msg::PlanningSceneProjectionStatus::ConstSharedPtr status;
    {
      std::scoped_lock lock(scene_status_mutex_);
      status = scene_status_;
    }
    if (status) {
      projector_detail = status->detail;
      // The plan must cover the revision the projector had applied when it was made.
      const auto required = baseline ? baseline->applied_revision : status->applied_revision;
      result = evaluate_planning_scene_authority(
        *status, required, node_->now(), scene_authority_config(), baseline, lease);
      if (!transient_authority(result)) {
        return result;
      }
    }
    if (cancel_requested_.load(std::memory_order_acquire) ||
      stopping_.load(std::memory_order_acquire) ||
      std::chrono::steady_clock::now() >= deadline)
    {
      return result;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
}

std::optional<PlanningSceneLeaseAuthority> MoveItMotionPort::validate_scene_lease(
  const std::string & token, std::string & detail,
  std::chrono::steady_clock::time_point outer_deadline)
{
  detail.clear();
  if (token.empty()) {
    return std::nullopt;
  }
  if (!scene_lease_validation_client_ || !scene_lease_validation_client_->service_is_ready()) {
    detail = "planning-scene lease validation service is unavailable";
    return std::nullopt;
  }
  auto request =
    std::make_shared<restocker_interfaces::srv::ValidatePlanningSceneLease::Request>();
  request->token = token;
  auto future = scene_lease_validation_client_->async_send_request(request);
  const auto deadline = std::min(
    outer_deadline,
    std::chrono::steady_clock::now() + config_.planning_scene_lease_validation_timeout);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (cancel_requested_.load(std::memory_order_acquire) ||
      stopping_.load(std::memory_order_acquire) ||
      std::chrono::steady_clock::now() >= deadline)
    {
      scene_lease_validation_client_->remove_pending_request(future);
      detail = "planning-scene lease validation timed out or was canceled";
      return std::nullopt;
    }
  }
  try {
    const auto response = future.get();
    using LeaseStatus = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;
    using Lease = restocker_interfaces::msg::PlanningSceneLease;
    if (!response || response->status.code != LeaseStatus::VALID || !response->has_lease ||
      response->lease.lease_id == 0U ||
      response->lease.granted_applied_revision == 0U ||
      response->lease.verification_epoch == 0U || response->lease.phase != Lease::PHASE_HELD)
    {
      detail = "planning-scene lease capability was not validated";
      if (response && !response->status.detail.empty()) {
        detail += ": " + response->status.detail;
      }
      return std::nullopt;
    }
    return PlanningSceneLeaseAuthority{
      response->lease.lease_id, response->lease.granted_applied_revision,
      response->lease.verification_epoch};
  } catch (const std::exception & error) {
    detail = std::string("planning-scene lease validation failed: ") + error.what();
  } catch (...) {
    detail = "planning-scene lease validation failed";
  }
  return std::nullopt;
}

std::optional<PreGraspSceneAuthority> MoveItMotionPort::establish_scene_authority(
  const std::string & lease_token, std::string & detail)
{
  std::string validation_detail;
  const auto lease = validate_scene_lease(lease_token, validation_detail);
  if (!lease_token.empty() && !lease) {
    detail = "planning-scene authority could not be established before planning: " +
      validation_detail;
    return std::nullopt;
  }
  std::string projector_detail;
  const auto result = settled_scene_authority(std::nullopt, lease, projector_detail);
  if (result) {
    return result.authority;
  }
  detail = "planning-scene authority could not be established before planning (" +
    std::string(to_string(result.decision)) + "/" + to_string(result.error) + "): " +
    result.detail + " -- projector detail: " + projector_detail;
  return std::nullopt;
}

std::string MoveItMotionPort::scene_authority_lost(
  const PreGraspSceneAuthority & baseline, const std::string & lease_token,
  const moveit_msgs::msg::RobotTrajectory & next_goal,
  const moveit::core::RobotModelConstPtr & robot_model, bool may_revalidate)
{
  const auto deadline = std::chrono::steady_clock::now() + config_.scene_authority_settle_timeout;
  TrajectorySceneGateHooks hooks;
  hooks.interrupted = [this, deadline]() {
    return cancel_requested_.load(std::memory_order_acquire) ||
           stopping_.load(std::memory_order_acquire) ||
           std::chrono::steady_clock::now() >= deadline;
  };
  hooks.read_authority = [this, &lease_token, &hooks, deadline]() {
    PlanningSceneAuthorityResult authority;
    authority.detail = "causal planning-scene status service is unavailable";
    if (!scene_status_client_ || !scene_status_client_->service_is_ready()) {
      return authority;
    }
    while (!hooks.interrupted()) {
      std::string detail;
      const auto lease = validate_scene_lease(lease_token, detail, deadline);
      if (!lease_token.empty() && !lease) {
        return PlanningSceneAuthorityResult{
        PreGraspAuthorityDecision::kRejected,
        PreGraspAuthorityErrorCode::kSceneLeaseCapabilityInvalid, std::nullopt, detail};
      }
      auto request =
        std::make_shared<restocker_interfaces::srv::GetPlanningSceneProjectionStatus::Request>();
      auto future = scene_status_client_->async_send_request(request);
      const auto read_deadline = std::min(
        deadline, std::chrono::steady_clock::now() + config_.state_validity_timeout);
      while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
        if (hooks.interrupted() || std::chrono::steady_clock::now() >= read_deadline) {
          scene_status_client_->remove_pending_request(future);
          authority.detail = "causal planning-scene status read timed out or was canceled";
          return authority;
        }
      }
      const auto response = future.get();
      if (!response) {
        authority.detail = "causal planning-scene status read returned no response";
        return authority;
      }
      const auto & status = response->status;
      authority = evaluate_planning_scene_authority(
        status, status.applied_revision, node_->now(), scene_authority_config(), std::nullopt,
        lease);
      // An observed unauthorized lease is a rejection at submission, even if it might later
      // release. Only ordinary projector waits may settle inside this operation's budget.
      if (authority.decision != PreGraspAuthorityDecision::kWait) {
        if (!authority) {
          authority.detail += " -- projector detail: " + status.detail;
        }
        return authority;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    authority.decision = PreGraspAuthorityDecision::kRejected;
    authority.detail = "causal planning-scene status canceled or its steady deadline expired";
    return authority;
  };
  hooks.validate_next_goal = [this, &next_goal, &robot_model, &hooks, deadline, may_revalidate]() {
    // Escape uses a private, narrowly scoped contact matrix. Never substitute move_group's
    // matrix for that contract or exempt those contacts in an ordinary revalidation.
    if (!may_revalidate) {
      return std::string(
        "scene content changed for a scoped-contact escape; a new plan is required");
    }
    if (!planning_scene_client_ || !planning_scene_client_->service_is_ready()) {
      return std::string("scene revalidation cannot read /get_planning_scene");
    }
    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components = kTrajectorySceneComponents;
    auto future = planning_scene_client_->async_send_request(request);
    const auto read_deadline = std::min(
      deadline, std::chrono::steady_clock::now() + config_.state_validity_timeout);
    while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
      if (hooks.interrupted() || std::chrono::steady_clock::now() >= read_deadline) {
        planning_scene_client_->remove_pending_request(future);
        return std::string("scene revalidation read timed out or was canceled");
      }
    }
    const auto response = future.get();
    if (!response) {
      return std::string("scene revalidation returned no snapshot");
    }
    return validate_controller_goal_collision_scene(
      robot_model, response->scene, next_goal, config_.planning_group, hooks.interrupted,
      config_.maximum_start_state_bounds_correction);
  };
  const auto result = validate_trajectory_scene_submission(baseline, hooks);
  if (!result.accepted) {
    return "refusing to execute: " + result.detail;
  }
  if (result.revalidated) {
    RCLCPP_INFO(
      node_->get_logger(),
      "scene changed since plan generation %s; next controller goal collision-validated "
      "against a fresh stable scene (later goals retain original provenance)",
      std::to_string(baseline.scene_content_generation).c_str());
  }
  return {};
}

std::string MoveItMotionPort::describe_truncated_linear_path(
  MoveGroupHolder & holder, const MotionGoal & goal,
  const geometry_msgs::msg::Pose & target,
  const moveit_msgs::msg::RobotTrajectory & achieved, double fraction,
  const moveit_msgs::msg::MoveItErrorCodes & error_code)
{
  std::ostringstream detail;
  detail << goal.label << ": ";
  if (fraction < 0.0) {
    // -1.0 is MoveIt's failed-request sentinel, not a distance.
    detail << "MoveIt refused the linear path outright and returned its error sentinel, so no "
      "part of the straight line was computed and nothing establishes what stopped it. MoveIt "
      "reported " << moveit_error_name(error_code.val) << " (code " << error_code.val << ")";
    return detail.str();
  }

  // Re-run the interpolation with collision checking off. The difference is evidence about the
  // planning scene, not a cause, and never drives the arm.
  moveit_msgs::msg::RobotTrajectory unobstructed_path;
  moveit_msgs::msg::MoveItErrorCodes unobstructed_code;
  unobstructed_code.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
  const double unobstructed = holder.interface().computeCartesianPath(
    {target}, goal.cartesian_step_m, unobstructed_path, false, &unobstructed_code);

  const std::size_t achieved_points = achieved.joint_trajectory.points.size();
  const std::size_t unobstructed_points = unobstructed_path.joint_trajectory.points.size();
  detail << "linear path stopped " << fraction * 100.0 << "% of the way to the goal after "
         << achieved_points << " interpolated waypoints; MoveIt reported "
         << moveit_error_name(error_code.val) << " (code " << error_code.val << ")";
  if (unobstructed < 0.0) {
    detail << ". The same line with collision checking disabled was refused outright ("
           << moveit_error_name(unobstructed_code.val)
           << "), so a planning-scene collision is not established";
    return detail.str();
  }
  detail << ". The same line with collision checking disabled reaches " << unobstructed * 100.0
         << "%";
  if (unobstructed < goal.minimum_cartesian_fraction) {
    detail << ", so the straight line is not kinematically reachable and the planning scene is "
      "not what stopped it";
    return detail.str();
  }
  detail << ", so the straight line is kinematically reachable. ";
  if (unobstructed_points < 2U) {
    detail << "The collision-free interpolation carries no waypoint past the start, so the "
      "configuration the collision-checked run refused could not be checked";
    return detail.str();
  }
  // Waypoint counts are not comparable between the runs (MoveIt subdivides intervals it cannot
  // validate), so the refused configuration is the collision-free run's first waypoint past the
  // fraction reached. MoveIt is asked about it directly.
  const auto last_index = unobstructed_points - 1U;
  const auto refused_index = std::min(
    last_index,
    static_cast<std::size_t>(
      std::floor(fraction * static_cast<double>(last_index))) + 1U);
  detail << describe_state_validity(unobstructed_path.joint_trajectory, refused_index);
  return detail.str();
}

std::string MoveItMotionPort::describe_state_validity(
  const trajectory_msgs::msg::JointTrajectory & path, std::size_t point_index)
{
  if (point_index >= path.points.size()) {
    return "The refused configuration is not present in the collision-free interpolation, so it "
           "could not be checked";
  }
  if (!state_validity_client_ || !state_validity_client_->service_is_ready()) {
    return "/check_state_validity is not available, so the reason MoveIt refused that "
           "configuration is not established";
  }
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  request->group_name = config_.planning_group;
  // A diff against the scene's current state keeps the attached product and other joints as is.
  request->robot_state.is_diff = true;
  request->robot_state.joint_state.name = path.joint_names;
  request->robot_state.joint_state.position = path.points[point_index].positions;

  auto future = state_validity_client_->async_send_request(request);
  if (future.wait_for(config_.state_validity_timeout) != std::future_status::ready) {
    state_validity_client_->remove_pending_request(future);
    return "/check_state_validity did not answer in time, so the reason MoveIt refused that "
           "configuration is not established";
  }
  const auto response = future.get();
  if (!response) {
    return "/check_state_validity returned nothing, so the reason MoveIt refused that "
           "configuration is not established";
  }
  // This is the collision-free run's configuration at the stop distance, not necessarily what the
  // checked run would have used.
  if (response->valid) {
    return "MoveIt reports the collision-free interpolation's configuration at that distance "
           "valid, so a static collision there is not what stopped the checked run";
  }
  if (response->contacts.empty()) {
    return "MoveIt reports the collision-free interpolation's configuration at that distance "
           "invalid but named no contact";
  }
  std::ostringstream contacts;
  contacts << "MoveIt reports the collision-free interpolation's configuration at that distance "
    "in collision:";
  const std::size_t reported = std::min(response->contacts.size(), kMaxReportedContacts);
  for (std::size_t index = 0U; index < reported; ++index) {
    const auto & contact = response->contacts[index];
    contacts << (index == 0U ? " " : ", ") << contact.contact_body_1 << " against "
             << contact.contact_body_2;
  }
  if (response->contacts.size() > reported) {
    contacts << " and " << response->contacts.size() - reported << " more";
  }
  return contacts.str();
}

StartStateCondition MoveItMotionPort::establish_start_state_condition(
  const moveit::core::RobotState & start_state)
{
  StartStateCondition condition;
  if (!state_validity_client_ || !state_validity_client_->service_is_ready()) {
    condition.unestablished = "/check_state_validity is unavailable";
    return condition;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  // The full state, grippers included: the jaws are part of what the scene is asked about.
  moveit::core::robotStateToRobotStateMsg(start_state, request->robot_state);
  request->group_name = config_.planning_group;
  auto future = state_validity_client_->async_send_request(request);
  if (future.wait_for(config_.state_validity_timeout) != std::future_status::ready) {
    state_validity_client_->remove_pending_request(future);
    condition.unestablished = "/check_state_validity did not answer within " +
      std::to_string(config_.state_validity_timeout.count()) + " ms";
    return condition;
  }
  const auto response = future.get();
  if (!response) {
    condition.unestablished = "/check_state_validity returned no answer";
    return condition;
  }
  condition.established = true;
  condition.in_collision = !response->valid;
  const std::size_t reported = std::min(response->contacts.size(), kMaxReportedContacts);
  condition.contacts.reserve(reported);
  for (std::size_t index = 0U; index < reported; ++index) {
    const auto & contact = response->contacts[index];
    condition.contacts.push_back(
      contact.contact_body_1 + " against " + contact.contact_body_2);
  }
  condition.suppressed_contacts = response->contacts.size() - reported;
  return condition;
}

MoveItMotionPort::GraspEscapePlan MoveItMotionPort::plan_grasp_escape(
  const PendingGoal & pending, const moveit::core::RobotState & current_state,
  const moveit::core::JointModelGroup * planning_group,
  moveit_msgs::msg::RobotTrajectory & trajectory)
{
  GraspEscapePlan plan;
  const auto & robot_model = current_state.getRobotModel();
  if (robot_model->getModelFrame() != config_.planning_frame) {
    plan.detail = "the grasp escape needs the planning frame '" + config_.planning_frame +
      "' to be the model frame '" + robot_model->getModelFrame() + "'";
    return plan;
  }
  // The same snapshot the free-space race plans against, plus move_group's own allowed-collision
  // matrix and link padding, so the local copy refuses exactly what move_group would before the
  // scoped entries are added.
  if (!planning_scene_client_ || !planning_scene_client_->service_is_ready()) {
    plan.detail = "the grasp escape cannot read the scene (/get_planning_scene is unavailable)";
    return plan;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  // Card 068's components and builder: the same collision model the race plans on.
  request->components.components = kRaceSceneComponents;
  auto future = planning_scene_client_->async_send_request(request);
  if (future.wait_for(config_.state_validity_timeout) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(future);
    plan.detail = "the grasp escape cannot read the scene (/get_planning_scene did not answer "
      "within " + std::to_string(config_.state_validity_timeout.count()) + " ms)";
    return plan;
  }
  const auto response = future.get();
  if (!response) {
    plan.detail = "the grasp escape cannot read the scene (/get_planning_scene returned nothing)";
    return plan;
  }
  auto line = plan_grasp_escape_from_snapshot(
    response->scene, current_state, planning_group,
    robot_model->getLinkModel(config_.end_effector_link), *pending.goal.grasp_escape,
    pending.goal.cartesian_step_m, pending.goal.minimum_cartesian_fraction);
  if (!line.planned) {
    plan.detail = std::move(line.detail);
    return plan;
  }
  robot_trajectory::RobotTrajectory path(robot_model, planning_group);
  for (const auto & waypoint : line.waypoints) {
    path.addSuffixWayPoint(waypoint, 0.0);
  }
  const trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
  if (!time_parameterization.computeTimeStamps(
      path, pending.goal.velocity_scaling, pending.goal.acceleration_scaling))
  {
    plan.detail = "the grasp escape line could not be time-parameterized";
    return plan;
  }
  path.getRobotTrajectoryMsg(trajectory);
  plan.detail = std::move(line.detail);
  plan.planned = true;
  return plan;
}

std::string MoveItMotionPort::describe_scene_world_objects()
{
  if (!planning_scene_client_ || !planning_scene_client_->service_is_ready()) {
    return "scene world objects not established (/get_planning_scene is unavailable)";
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  auto future = planning_scene_client_->async_send_request(request);
  if (future.wait_for(config_.state_validity_timeout) != std::future_status::ready) {
    planning_scene_client_->remove_pending_request(future);
    return "scene world objects not established (/get_planning_scene did not answer within " +
           std::to_string(config_.state_validity_timeout.count()) + " ms)";
  }
  const auto response = future.get();
  if (!response) {
    return "scene world objects not established (/get_planning_scene returned no answer)";
  }
  const auto & objects = response->scene.world.collision_objects;
  std::ostringstream stream;
  stream << "scene carries " << objects.size() << " world object(s)";
  bool any_obstacle = false;
  for (const auto & object : objects) {
    if (!object.id.starts_with(kObstacleIdPrefix)) {
      continue;
    }
    stream << (any_obstacle ? "," : "; projected obstacles:") << " " << object.id;
    any_obstacle = true;
  }
  return stream.str();
}

}  // namespace restocker_task_executor
