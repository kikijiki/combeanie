// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

const std::vector<std::string> kManipulatorJoints{
  "rail_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
  "wrist_2_joint", "wrist_3_joint"};
const std::vector<std::string> kExecutionControllers{"arm_controller", "rail_controller"};
constexpr char kShelfPanelId[] = "shelf_back_panel";

struct JointStateSnapshot
{
  std::vector<std::string> names;
  std::vector<double> positions;
  builtin_interfaces::msg::Time stamp;
};

class JointStateObserver
{
public:
  explicit JointStateObserver(const rclcpp::Node::SharedPtr & node)
  {
    subscription_ = node->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::ConstSharedPtr message) {
        {
          std::lock_guard lock(mutex_);
          latest_ = JointStateSnapshot{message->name, message->position, message->header.stamp};
        }
        received_.notify_all();
      });
  }

  [[nodiscard]] std::optional<JointStateSnapshot> wait_for_first(
    std::chrono::duration<double> timeout)
  {
    std::unique_lock lock(mutex_);
    received_.wait_for(lock, timeout, [this]() {return latest_.has_value();});
    return latest_;
  }

  [[nodiscard]] std::optional<JointStateSnapshot> latest() const
  {
    std::lock_guard lock(mutex_);
    return latest_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable received_;
  std::optional<JointStateSnapshot> latest_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

struct RuntimeConfiguration
{
  double startup_timeout_sec;
  double planning_time_sec;
  double state_max_age_sec;
  double verification_timeout_sec;
  double goal_joint_tolerance;
  double goal_position_tolerance_m;
  double goal_orientation_tolerance_rad;
  double verification_joint_tolerance;
  double verification_position_tolerance_m;
  double verification_orientation_tolerance_rad;
};

PlanningResult validate_snapshot(
  const rclcpp::Node::SharedPtr & node, const JointStateSnapshot & snapshot,
  double state_max_age_sec)
{
  const rclcpp::Time observation_time(snapshot.stamp, node->get_clock()->get_clock_type());
  const double age_sec = (node->now() - observation_time).seconds();
  return validate_joint_state(snapshot.names, snapshot.positions, age_sec, state_max_age_sec);
}

TrajectoryObservation observe_trajectory(const MoveGroupInterface::Plan & plan)
{
  TrajectoryObservation observation;
  const auto & trajectory = plan.trajectory.joint_trajectory;
  observation.joint_names = trajectory.joint_names;
  observation.points.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    const double time_sec = static_cast<double>(point.time_from_start.sec) +
      static_cast<double>(point.time_from_start.nanosec) * 1e-9;
    observation.points.push_back({point.positions, time_sec});
  }
  return observation;
}

PlanningResult wait_for_joint_target(
  const rclcpp::Node::SharedPtr & node, const JointStateObserver & observer,
  const std::vector<double> & target, const RuntimeConfiguration & configuration)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(configuration.verification_timeout_sec);
  PlanningResult latest_result{
    PlanningStatus::kPostExecutionMismatch, "no post-execution joint state received"};
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    const auto snapshot = observer.latest();
    if (snapshot.has_value()) {
      const auto state_result =
        validate_snapshot(node, snapshot.value(), configuration.state_max_age_sec);
      if (!state_result.ok()) {
        latest_result = state_result;
      } else {
        latest_result = validate_joint_target(
          snapshot->names, snapshot->positions, target,
          configuration.verification_joint_tolerance);
        if (latest_result.ok()) {
          return latest_result;
        }
      }
    }
    std::this_thread::sleep_for(20ms);
  }
  return latest_result;
}

PlanningResult validate_before_execution(
  const rclcpp::Node::SharedPtr & node, const JointStateObserver & observer,
  const RuntimeConfiguration & configuration)
{
  const auto snapshot = observer.latest();
  if (!snapshot.has_value()) {
    return {
      PlanningStatus::kRobotStateIncomplete,
      "joint state disappeared before trajectory execution"};
  }
  return validate_snapshot(node, snapshot.value(), configuration.state_max_age_sec);
}

PlanningResult plan_and_execute_joint_goal(
  const rclcpp::Node::SharedPtr & node, JointStateObserver & observer,
  MoveGroupInterface & move_group, const RuntimeConfiguration & configuration,
  const std::vector<double> & target)
{
  move_group.setStartStateToCurrentState();
  if (!move_group.setJointValueTarget(target)) {
    return {
      PlanningStatus::kPlanningRejected, "joint-space target violates the robot model bounds"};
  }

  MoveGroupInterface::Plan plan;
  auto result = classify_moveit_error(
    PlanningPhase::kPlan, move_group.plan(plan).val, "joint-space goal");
  if (!result.ok()) {
    return result;
  }
  result = validate_trajectory(observe_trajectory(plan));
  if (!result.ok()) {
    return result;
  }
  result = validate_before_execution(node, observer, configuration);
  if (!result.ok()) {
    return result;
  }

  result = classify_moveit_error(
    PlanningPhase::kExecute, move_group.execute(plan, kExecutionControllers).val,
    "joint-space goal");
  if (!result.ok()) {
    return result;
  }
  return wait_for_joint_target(node, observer, target, configuration);
}

Eigen::Isometry3d forward_kinematics_target(
  const moveit::core::RobotModelConstPtr & model, const std::vector<double> & target)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (std::size_t index = 0; index < kManipulatorJoints.size(); ++index) {
    state.setVariablePosition(kManipulatorJoints[index], target[index]);
  }
  state.update();
  return state.getGlobalLinkTransform("tool0");
}

moveit::core::RobotState robot_state_for_target(
  const moveit::core::RobotModelConstPtr & model, const std::vector<double> & target)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (std::size_t index = 0; index < kManipulatorJoints.size(); ++index) {
    state.setVariablePosition(kManipulatorJoints[index], target[index]);
  }
  state.update();
  return state;
}

PlanningResult verify_reviewed_self_collision(
  const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModelConstPtr & model)
{
  // Reviewed against the official UR10e collision meshes. This fold drives the forearm through
  // the shoulder while remaining inside every joint bound.
  const std::vector<double> fixture{0.0, 2.52, 1.06, 2.98, 2.15, 1.21, 1.02};
  const std::pair<std::string, std::string> expected_pair{
    "forearm_link", "shoulder_link"};

  planning_scene::PlanningScene scene(model);
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 20;
  request.max_contacts_per_pair = 5;
  request.group_name = "manipulator";

  auto state = robot_state_for_target(model, fixture);
  if (!state.satisfiesBounds(model->getJointModelGroup("manipulator"))) {
    return {
      PlanningStatus::kStartStateInvalid, "reviewed self-collision fixture violates joint bounds"};
  }
  collision_detection::CollisionResult collision;
  scene.checkSelfCollision(request, collision, state, scene.getAllowedCollisionMatrix());
  const auto expected_contact = std::find_if(
    collision.contacts.cbegin(), collision.contacts.cend(),
    [&expected_pair](const auto & entry) {
      const auto & pair = entry.first;
      return pair == expected_pair ||
             (pair.first == expected_pair.second && pair.second == expected_pair.first);
    });
  if (expected_contact == collision.contacts.cend()) {
    std::string detail = "reviewed " + expected_pair.first + "/" + expected_pair.second +
      " self-collision contact was not reported; reported_pairs=" +
      std::to_string(collision.contacts.size());
    for (const auto & [pair, contacts] : collision.contacts) {
      detail += " " + pair.first + "/" + pair.second + "=" + std::to_string(contacts.size());
    }
    return {
      PlanningStatus::kExpectedCollisionMissing,
      std::move(detail)};
  }
  RCLCPP_INFO(
    node->get_logger(), "event=self_collision_verified link_1=%s link_2=%s",
    expected_pair.first.c_str(), expected_pair.second.c_str());
  return {PlanningStatus::kSuccess, "reviewed self-collision state was rejected"};
}

moveit_msgs::msg::CollisionObject make_shelf_panel(const Eigen::Isometry3d & target)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = kShelfPanelId;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  shape_msgs::msg::SolidPrimitive panel;
  panel.type = shape_msgs::msg::SolidPrimitive::BOX;
  panel.dimensions = {0.08, 0.45, 0.30};
  object.primitives.push_back(panel);

  geometry_msgs::msg::Pose pose;
  pose.position.x = target.translation().x();
  pose.position.y = target.translation().y();
  pose.position.z = target.translation().z();
  pose.orientation.w = 1.0;
  object.primitive_poses.push_back(pose);
  return object;
}

PlanningResult query_state_validity(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & client,
  const moveit::core::RobotState & state, double timeout_sec, bool expected_valid,
  bool require_shelf_contact)
{
  if (!client->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
    return {
      PlanningStatus::kServerUnavailable, "timed out waiting for /check_state_validity"};
  }
  auto request = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  moveit::core::robotStateToRobotStateMsg(state, request->robot_state);
  request->group_name = "manipulator";
  auto future = client->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(timeout_sec)) != std::future_status::ready) {
    return {
      PlanningStatus::kSceneSynchronizationTimedOut,
      "timed out querying state validity from the monitored planning scene"};
  }
  const auto response = future.get();
  if (response->valid != expected_valid) {
    return {
      PlanningStatus::kExpectedCollisionMissing,
      expected_valid ? "known collision-free fixture was rejected" :
      "shelf collision fixture remained valid"};
  }
  if (!require_shelf_contact) {
    return {PlanningStatus::kSuccess, "state validity matched expectation"};
  }
  for (const auto & contact : response->contacts) {
    if (contact.contact_body_1 == kShelfPanelId || contact.contact_body_2 == kShelfPanelId) {
      RCLCPP_INFO(
        node->get_logger(), "event=scene_collision_verified object=%s body_1=%s body_2=%s",
        kShelfPanelId, contact.contact_body_1.c_str(), contact.contact_body_2.c_str());
      return {PlanningStatus::kSuccess, "named shelf panel contact was reported"};
    }
  }
  return {
    PlanningStatus::kExpectedCollisionMissing,
    "invalid fixture state did not report contact with shelf_back_panel"};
}

PlanningResult verify_shelf_collision_rejection(
  const rclcpp::Node::SharedPtr & node, MoveGroupInterface & move_group,
  const RuntimeConfiguration & configuration, const std::vector<double> & target)
{
  const auto model = move_group.getRobotModel();
  auto fixture_state = robot_state_for_target(model, target);
  auto validity_client =
    node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  auto result = query_state_validity(
    node, validity_client, fixture_state, configuration.startup_timeout_sec, true, false);
  if (!result.ok()) {
    return result;
  }

  moveit::planning_interface::PlanningSceneInterface scene_interface;
  const auto panel = make_shelf_panel(fixture_state.getGlobalLinkTransform("tool0"));
  if (!scene_interface.applyCollisionObject(panel)) {
    return {
      PlanningStatus::kSceneSynchronizationTimedOut,
      "MoveIt rejected the synchronous shelf_back_panel scene update"};
  }
  const auto known_objects = scene_interface.getObjects({kShelfPanelId});
  PlanningResult scene_result{PlanningStatus::kSuccess, "shelf panel update acknowledged"};
  if (!known_objects.contains(kShelfPanelId)) {
    scene_result = {
      PlanningStatus::kSceneSynchronizationTimedOut,
      "shelf_back_panel was not acknowledged by the monitored planning scene"};
  }

  if (scene_result.ok()) {
    const auto current_state = move_group.getCurrentState(configuration.startup_timeout_sec);
    if (!current_state) {
      scene_result = {
        PlanningStatus::kRobotStateIncomplete,
        "could not read current state after applying shelf_back_panel"};
    } else {
      scene_result = query_state_validity(
        node, validity_client, *current_state, configuration.startup_timeout_sec, true, false);
      if (!scene_result.ok()) {
        scene_result.detail = "shelf panel invalidated the planning start state: " +
          scene_result.detail;
      } else {
        RCLCPP_INFO(
          node->get_logger(), "event=collision_fixture_start_state status=valid object=%s",
          kShelfPanelId);
      }
    }
  }
  if (scene_result.ok()) {
    scene_result = query_state_validity(
      node, validity_client, fixture_state, configuration.startup_timeout_sec, false, true);
  }
  if (scene_result.ok()) {
    move_group.setStartStateToCurrentState();
    move_group.clearPoseTargets();
    move_group.setJointValueTarget(target);
    MoveGroupInterface::Plan rejected_plan;
    const auto planning_error = move_group.plan(rejected_plan);
    if (planning_error == moveit::core::MoveItErrorCode::SUCCESS) {
      scene_result = {
        PlanningStatus::kExpectedCollisionMissing,
        "planner accepted a goal in acknowledged shelf_back_panel collision"};
    } else {
      RCLCPP_INFO(
        node->get_logger(), "event=collision_plan_rejected object=%s moveit_code=%d",
        kShelfPanelId, planning_error.val);
    }
  }

  auto removal = panel;
  removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!scene_interface.applyCollisionObject(removal) ||
    scene_interface.getObjects({kShelfPanelId}).contains(kShelfPanelId))
  {
    return {
      PlanningStatus::kSceneSynchronizationTimedOut,
      "failed to remove shelf_back_panel from the monitored planning scene"};
  }
  return scene_result;
}

PlanningResult validate_current_pose(
  MoveGroupInterface & move_group, const Eigen::Isometry3d & target,
  const RuntimeConfiguration & configuration)
{
  const geometry_msgs::msg::PoseStamped current = move_group.getCurrentPose("tool0");
  const Eigen::Vector3d current_position(
    current.pose.position.x, current.pose.position.y, current.pose.position.z);
  const Eigen::Quaterniond current_orientation(
    current.pose.orientation.w, current.pose.orientation.x, current.pose.orientation.y,
    current.pose.orientation.z);
  const Eigen::Quaterniond target_orientation(target.rotation());
  return validate_pose_error(
    (current_position - target.translation()).norm(),
    current_orientation.normalized().angularDistance(target_orientation.normalized()),
    configuration.verification_position_tolerance_m,
    configuration.verification_orientation_tolerance_rad);
}

PlanningResult plan_and_execute_pose_goal(
  const rclcpp::Node::SharedPtr & node, JointStateObserver & observer,
  MoveGroupInterface & move_group, const RuntimeConfiguration & configuration,
  const Eigen::Isometry3d & target)
{
  move_group.setStartStateToCurrentState();
  move_group.clearPoseTargets();
  if (!move_group.setPoseTarget(target, "tool0")) {
    return {PlanningStatus::kPlanningRejected, "tool pose target could not be represented"};
  }

  MoveGroupInterface::Plan plan;
  auto result =
    classify_moveit_error(PlanningPhase::kPlan, move_group.plan(plan).val, "tool pose goal");
  if (!result.ok()) {
    return result;
  }
  result = validate_trajectory(observe_trajectory(plan));
  if (!result.ok()) {
    return result;
  }
  result = validate_before_execution(node, observer, configuration);
  if (!result.ok()) {
    return result;
  }

  result = classify_moveit_error(
    PlanningPhase::kExecute, move_group.execute(plan, kExecutionControllers).val,
    "tool pose goal");
  if (!result.ok()) {
    return result;
  }
  result = validate_before_execution(node, observer, configuration);
  if (!result.ok()) {
    return result;
  }
  return validate_current_pose(move_group, target, configuration);
}

PlanningResult run_smoke_test(
  const rclcpp::Node::SharedPtr & node, JointStateObserver & observer,
  const RuntimeConfiguration & configuration)
{
  const auto initial_snapshot = observer.wait_for_first(
    std::chrono::duration<double>(configuration.startup_timeout_sec));
  if (!initial_snapshot.has_value()) {
    return {PlanningStatus::kRobotStateIncomplete, "timed out waiting for /joint_states"};
  }
  auto result = validate_snapshot(node, initial_snapshot.value(), configuration.state_max_age_sec);
  if (!result.ok()) {
    return result;
  }

  std::optional<MoveGroupInterface> move_group;
  try {
    move_group.emplace(
      node, "manipulator", nullptr,
      rclcpp::Duration::from_seconds(configuration.startup_timeout_sec));
  } catch (const std::exception & error) {
    return {
      PlanningStatus::kServerUnavailable,
      std::string("failed to connect to MoveGroup: ") + error.what()};
  }
  move_group->setEndEffectorLink("tool0");
  result = validate_model_contract(
    move_group->getPlanningFrame(), move_group->getEndEffectorLink(),
    move_group->getActiveJoints());
  if (!result.ok()) {
    return result;
  }
  if (!move_group->startStateMonitor(configuration.startup_timeout_sec)) {
    return {
      PlanningStatus::kRobotStateIncomplete,
      "MoveGroup current-state monitor did not receive a complete robot state"};
  }

  move_group->setPlanningPipelineId("ompl");
  move_group->setPlannerId("RRTConnectkConfigDefault");
  move_group->setPlanningTime(configuration.planning_time_sec);
  move_group->setNumPlanningAttempts(1);
  move_group->setMaxVelocityScalingFactor(0.2);
  move_group->setMaxAccelerationScalingFactor(0.2);
  move_group->setGoalJointTolerance(configuration.goal_joint_tolerance);
  move_group->setGoalPositionTolerance(configuration.goal_position_tolerance_m);
  move_group->setGoalOrientationTolerance(configuration.goal_orientation_tolerance_rad);
  move_group->setPoseReferenceFrame("world");
  move_group->setWorkspace(-2.5, -2.5, -0.5, 2.5, 2.5, 3.0);

  const std::vector<double> joint_target{0.35, 0.2, -0.4, 0.6, 0.1, 0.2, -0.1};
  RCLCPP_INFO(node->get_logger(), "event=plan_start goal=joint_space");
  result = plan_and_execute_joint_goal(
    node, observer, move_group.value(), configuration, joint_target);
  if (!result.ok()) {
    return result;
  }
  RCLCPP_INFO(node->get_logger(), "event=execution_verified goal=joint_space");

  const std::vector<double> pose_seed{-0.25, -0.3, -0.55, 0.75, -0.2, 0.3, 0.25};
  const Eigen::Isometry3d pose_target =
    forward_kinematics_target(move_group->getRobotModel(), pose_seed);
  RCLCPP_INFO(node->get_logger(), "event=plan_start goal=tool_pose");
  result = plan_and_execute_pose_goal(
    node, observer, move_group.value(), configuration, pose_target);
  if (!result.ok()) {
    return result;
  }
  RCLCPP_INFO(node->get_logger(), "event=execution_verified goal=tool_pose");
  move_group->clearPoseTargets();

  result = verify_reviewed_self_collision(node, move_group->getRobotModel());
  if (!result.ok()) {
    return result;
  }

  const std::vector<double> shelf_fixture_target{1.15, 0.45, -0.65, 0.85, 0.2, -0.3, 0.15};
  result = verify_shelf_collision_rejection(
    node, move_group.value(), configuration, shelf_fixture_target);
  if (!result.ok()) {
    return result;
  }
  return {
    PlanningStatus::kSuccess,
    "joint and pose execution plus self and scene collision checks passed"};
}

RuntimeConfiguration read_configuration(const rclcpp::Node::SharedPtr & node)
{
  const auto read_double = [&node](const std::string & name, double default_value) {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).as_double();
    }
    return node->declare_parameter(name, default_value);
  };
  return {
    read_double("startup_timeout_sec", 30.0),
    read_double("planning_time_sec", 5.0),
    read_double("state_max_age_sec", 0.5),
    read_double("verification_timeout_sec", 5.0),
    read_double("goal_joint_tolerance", 0.01),
    read_double("goal_position_tolerance_m", 0.005),
    read_double("goal_orientation_tolerance_rad", 0.02),
    read_double("verification_joint_tolerance", 0.03),
    read_double("verification_position_tolerance_m", 0.015),
    read_double("verification_orientation_tolerance_rad", 0.05),
  };
}

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<rclcpp::Node>(
    "planning_smoke_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  restocker_task_executor::JointStateObserver observer(node);
  const auto configuration = restocker_task_executor::read_configuration(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  const auto result = restocker_task_executor::run_smoke_test(node, observer, configuration);
  if (result.ok()) {
    RCLCPP_INFO(
      node->get_logger(), "event=planning_smoke_result status=%s detail=\"%s\"",
      restocker_task_executor::to_string(result.status), result.detail.c_str());
  } else {
    RCLCPP_ERROR(
      node->get_logger(), "event=planning_smoke_result status=%s detail=\"%s\"",
      restocker_task_executor::to_string(result.status), result.detail.c_str());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return result.exit_code();
}
