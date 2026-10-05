// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "restocker_control/execution_controller_authority.hpp"

namespace restocker_control
{
namespace
{

[[nodiscard]] std::vector<std::string> arm_joints()
{
  return {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
    "wrist_2_joint", "wrist_3_joint"};
}

[[nodiscard]] std::vector<std::string> commands(const std::vector<std::string> & joints)
{
  std::vector<std::string> values;
  for (const auto & joint : joints) {
    values.push_back(joint + "/velocity");
  }
  return values;
}

[[nodiscard]] std::vector<std::string> states(const std::vector<std::string> & joints)
{
  std::vector<std::string> values;
  for (const auto & joint : joints) {
    values.push_back(joint + "/position");
    values.push_back(joint + "/velocity");
  }
  return values;
}

[[nodiscard]] ExecutionControllerEvidence healthy_evidence()
{
  ExecutionControllerEvidence evidence;
  evidence.controller_manager_epoch_before = "controller-manager:test";
  evidence.controller_manager_epoch_after = "controller-manager:test";
  evidence.generation_before = 7U;
  evidence.generation_after = 7U;
  evidence.observed_at = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  const auto arm = arm_joints();
  const std::vector<std::string> rail{"rail_joint"};
  evidence.controllers = {
    {"arm_controller", "joint_trajectory_controller/JointTrajectoryController", "active",
      "arm_controller/follow_joint_trajectory", true, arm, commands(arm), states(arm)},
    {"rail_controller", "joint_trajectory_controller/JointTrajectoryController", "active",
      "rail_controller/follow_joint_trajectory", true, rail, commands(rail), states(rail)}};
  for (const auto & [controller,
    joints] : std::vector<std::pair<std::string, std::vector<std::string>>>{
        {"arm_controller", arm}, {"rail_controller", rail}})
  {
    for (const auto & joint : joints) {
      evidence.hardware_interfaces.push_back(
        {joint + "/velocity", HardwareInterfaceKind::Command, true, controller});
      evidence.hardware_interfaces.push_back(
        {joint + "/position", HardwareInterfaceKind::State, true, std::nullopt});
      evidence.hardware_interfaces.push_back(
        {joint + "/velocity", HardwareInterfaceKind::State, true, std::nullopt});
    }
  }
  evidence.moveit_mappings = {
    {"arm_controller", "FollowJointTrajectory", "follow_joint_trajectory", arm},
    {"rail_controller", "FollowJointTrajectory", "follow_joint_trajectory", rail},
    {"gripper_controller", "FollowJointTrajectory", "follow_joint_trajectory",
      {"left_finger_joint", "right_finger_joint"}}};
  return evidence;
}

[[nodiscard]] rclcpp::Time healthy_now()
{
  return rclcpp::Time(10'100'000'000LL, RCL_ROS_TIME);
}

void expect_error(
  const ExecutionControllerEvidence & evidence,
  ExecutionControllerAuthorityErrorCode expected,
  const ExecutionControllerContract & contract = {})
{
  const auto result = evaluate_execution_controller_authority(evidence, healthy_now(), contract);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.error, expected) << result.detail;
  EXPECT_FALSE(result.detail.empty());
}

TEST(ExecutionControllerAuthority, AcceptsCoherentArmAndRailAuthority)
{
  const auto result = evaluate_execution_controller_authority(healthy_evidence(), healthy_now());
  ASSERT_TRUE(result) << result.detail;
  EXPECT_EQ(result.error, ExecutionControllerAuthorityErrorCode::None);
  EXPECT_EQ(result.authority->controller_manager_epoch, "controller-manager:test");
  EXPECT_EQ(result.authority->source_generation, 7U);
  EXPECT_NE(result.authority->contract_fingerprint, 0U);
  EXPECT_EQ(
    result.authority->controller_names,
    (std::array<std::string, 2>{"arm_controller", "rail_controller"}));
  EXPECT_EQ(result.authority->trajectory_joint_names.front(), "rail_joint");
  EXPECT_EQ(result.authority->trajectory_joint_names.back(), "wrist_3_joint");
}

TEST(ExecutionControllerAuthority, RejectsInvalidUnstableAndErroredSources)
{
  auto evidence = healthy_evidence();
  evidence.controller_manager_epoch_before.clear();
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::InvalidSource);
  evidence = healthy_evidence();
  evidence.controller_manager_epoch_after = "controller-manager:restarted";
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::UnstableSnapshot);
  evidence = healthy_evidence();
  evidence.generation_after++;
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::UnstableSnapshot);
  evidence = healthy_evidence();
  evidence.controller_manager_error = true;
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::SourceError);
  evidence = healthy_evidence();
  evidence.hardware_error = true;
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::SourceError);
}

TEST(ExecutionControllerAuthority, RejectsStaleAndExcessivelyFutureEvidence)
{
  auto evidence = healthy_evidence();
  const auto contract = ExecutionControllerContract{};
  const auto at_age_limit = evidence.observed_at + rclcpp::Duration(contract.maximum_age);
  EXPECT_TRUE(evaluate_execution_controller_authority(evidence, at_age_limit, contract));

  const auto stale_now = evidence.observed_at +
    rclcpp::Duration(contract.maximum_age + std::chrono::nanoseconds(1));
  const auto stale = evaluate_execution_controller_authority(evidence, stale_now, contract);
  EXPECT_FALSE(stale);
  EXPECT_EQ(stale.error, ExecutionControllerAuthorityErrorCode::StaleEvidence);

  const auto at_future_limit = evidence.observed_at -
    rclcpp::Duration(contract.maximum_future_skew);
  EXPECT_TRUE(evaluate_execution_controller_authority(evidence, at_future_limit, contract));
  const auto future_now = evidence.observed_at -
    rclcpp::Duration(contract.maximum_future_skew + std::chrono::nanoseconds(1));
  const auto future = evaluate_execution_controller_authority(evidence, future_now, contract);
  EXPECT_FALSE(future);
  EXPECT_EQ(future.error, ExecutionControllerAuthorityErrorCode::FutureEvidence);
}

TEST(ExecutionControllerAuthority, RejectsMissingDuplicateOrMismatchedControllers)
{
  auto evidence = healthy_evidence();
  evidence.controllers.pop_back();
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::MissingController);
  evidence = healthy_evidence();
  evidence.controllers.push_back(evidence.controllers.front());
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::DuplicateController);

  for (int field = 0; field < 7; ++field) {
    evidence = healthy_evidence();
    auto & arm = evidence.controllers.front();
    if (field == 0) {arm.type = "wrong/Type";}
    if (field == 1) {arm.state = "inactive";}
    if (field == 2) {arm.action_name = "wrong/action";}
    if (field == 3) {arm.action_server_ready = false;}
    if (field == 4) {arm.joints.pop_back();}
    if (field == 5) {arm.claimed_command_interfaces.pop_back();}
    if (field == 6) {arm.state_interfaces.pop_back();}
    expect_error(evidence, ExecutionControllerAuthorityErrorCode::ControllerMismatch);
  }

  evidence = healthy_evidence();
  evidence.controllers.front().claimed_command_interfaces.push_back(
    evidence.controllers.front().claimed_command_interfaces.front());
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::ControllerMismatch);
  evidence = healthy_evidence();
  evidence.controllers.front().state_interfaces.push_back(
    evidence.controllers.front().state_interfaces.front());
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::ControllerMismatch);
  evidence = healthy_evidence();
  evidence.controllers.push_back(
    {"conflicting_controller", "test/Controller", "active", "unused", false, {},
      {"shoulder_pan_joint/velocity"}, {}});
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::ControllerMismatch);
}

TEST(ExecutionControllerAuthority, TreatsClaimedInterfaceOrderAsNonsemantic)
{
  auto evidence = healthy_evidence();
  std::reverse(
    evidence.controllers.front().claimed_command_interfaces.begin(),
    evidence.controllers.front().claimed_command_interfaces.end());
  EXPECT_TRUE(evaluate_execution_controller_authority(evidence, healthy_now()));
}

TEST(ExecutionControllerAuthority, RejectsHardwareInterfaceDefects)
{
  auto evidence = healthy_evidence();
  evidence.hardware_interfaces.pop_back();
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::MissingHardwareInterface);
  evidence = healthy_evidence();
  evidence.hardware_interfaces.push_back(evidence.hardware_interfaces.front());
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::DuplicateHardwareInterface);
  for (int field = 0; field < 3; ++field) {
    evidence = healthy_evidence();
    auto & command = evidence.hardware_interfaces.front();
    if (field == 0) {command.available = false;}
    if (field == 1) {command.claimed_by = "gripper_controller";}
    if (field == 2) {command.claimed_by.reset();}
    expect_error(evidence, ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch);
  }
  evidence = healthy_evidence();
  evidence.hardware_interfaces[1].claimed_by = "arm_controller";
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch);
}

TEST(ExecutionControllerAuthority, RejectsMoveItMappingDefects)
{
  auto evidence = healthy_evidence();
  evidence.moveit_mappings.erase(evidence.moveit_mappings.begin() + 1);
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::MissingMoveItMapping);
  evidence = healthy_evidence();
  evidence.moveit_mappings.push_back(evidence.moveit_mappings.front());
  expect_error(evidence, ExecutionControllerAuthorityErrorCode::DuplicateMoveItMapping);
  for (int field = 0; field < 3; ++field) {
    evidence = healthy_evidence();
    auto & mapping = evidence.moveit_mappings.front();
    if (field == 0) {mapping.type = "wrong";}
    if (field == 1) {mapping.action_namespace = "wrong";}
    if (field == 2) {mapping.joints.pop_back();}
    expect_error(evidence, ExecutionControllerAuthorityErrorCode::MoveItMappingMismatch);
  }

  auto endpoint_contract = ExecutionControllerContract{};
  endpoint_contract.moveit_action_namespace = "execute_path";
  evidence = healthy_evidence();
  evidence.moveit_mappings[0].action_namespace = "execute_path";
  evidence.moveit_mappings[1].action_namespace = "execute_path";
  expect_error(
    evidence, ExecutionControllerAuthorityErrorCode::ControllerMismatch, endpoint_contract);
}

TEST(ExecutionControllerAuthority, RejectsInvalidContractAndFingerprintsSemanticChanges)
{
  auto contract = ExecutionControllerContract{};
  contract.rail_joint = contract.arm_joints.front();
  expect_error(
    healthy_evidence(), ExecutionControllerAuthorityErrorCode::InvalidContract, contract);

  const auto baseline = evaluate_execution_controller_authority(healthy_evidence(), healthy_now());
  contract = ExecutionControllerContract{};
  contract.moveit_action_namespace = "execute_path";
  auto changed_evidence = healthy_evidence();
  changed_evidence.moveit_mappings[0].action_namespace = "execute_path";
  changed_evidence.moveit_mappings[1].action_namespace = "execute_path";
  changed_evidence.controllers[0].action_name = "arm_controller/execute_path";
  changed_evidence.controllers[1].action_name = "rail_controller/execute_path";
  const auto changed = evaluate_execution_controller_authority(
    changed_evidence, healthy_now(), contract);
  ASSERT_TRUE(baseline);
  ASSERT_TRUE(changed) << changed.detail;
  EXPECT_NE(
    baseline.authority->contract_fingerprint,
    changed.authority->contract_fingerprint);
  ASSERT_TRUE(fingerprint_execution_controller_contract());
  EXPECT_EQ(
    *fingerprint_execution_controller_contract(),
    baseline.authority->contract_fingerprint);
  EXPECT_EQ(
    *fingerprint_execution_controller_contract(contract),
    changed.authority->contract_fingerprint);

  for (const auto invalid_name : {"/arm_controller", "arm_controller/"}) {
    contract = ExecutionControllerContract{};
    contract.arm_controller_name = invalid_name;
    expect_error(
      healthy_evidence(), ExecutionControllerAuthorityErrorCode::InvalidContract, contract);
  }
  contract = ExecutionControllerContract{};
  contract.maximum_age = std::chrono::nanoseconds::zero();
  EXPECT_FALSE(fingerprint_execution_controller_contract(contract));
  expect_error(
    healthy_evidence(), ExecutionControllerAuthorityErrorCode::InvalidContract, contract);
  contract = ExecutionControllerContract{};
  contract.maximum_future_skew = std::chrono::nanoseconds(-1);
  expect_error(
    healthy_evidence(), ExecutionControllerAuthorityErrorCode::InvalidContract, contract);
}

TEST(ExecutionControllerAuthority, ProvidesStableDiagnosticNames)
{
  EXPECT_STREQ(
    to_string(ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch),
    "hardware_interface_mismatch");
}

}  // namespace
}  // namespace restocker_control
