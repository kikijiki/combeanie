// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>

namespace restocker_control
{

inline constexpr std::size_t kExecutionArmJointCount = 6U;
inline constexpr std::size_t kExecutionJointCount = 7U;
inline constexpr std::size_t kExecutionControllerCount = 2U;

enum class HardwareInterfaceKind : std::uint8_t {Command, State};

struct TrajectoryControllerObservation
{
  std::string name;
  std::string type;
  std::string state;
  std::string action_name;
  bool action_server_ready{false};
  std::vector<std::string> joints;
  std::vector<std::string> claimed_command_interfaces;
  std::vector<std::string> state_interfaces;
};

struct HardwareInterfaceObservation
{
  std::string name;
  HardwareInterfaceKind kind{HardwareInterfaceKind::State};
  bool available{false};
  std::optional<std::string> claimed_by;
};

struct MoveItControllerMappingObservation
{
  std::string name;
  std::string type;
  std::string action_namespace;
  std::vector<std::string> joints;
};

struct ExecutionControllerEvidence
{
  std::string controller_manager_epoch_before;
  std::string controller_manager_epoch_after;
  std::uint64_t generation_before{0U};
  std::uint64_t generation_after{0U};
  rclcpp::Time observed_at{std::int64_t{0}, RCL_ROS_TIME};
  bool controller_manager_error{false};
  bool hardware_error{false};
  std::vector<TrajectoryControllerObservation> controllers;
  std::vector<HardwareInterfaceObservation> hardware_interfaces;
  std::vector<MoveItControllerMappingObservation> moveit_mappings;
};

struct ExecutionControllerContract
{
  std::string arm_controller_name{"arm_controller"};
  std::string rail_controller_name{"rail_controller"};
  std::string trajectory_controller_type{
    "joint_trajectory_controller/JointTrajectoryController"};
  std::string moveit_controller_type{"FollowJointTrajectory"};
  std::string moveit_action_namespace{"follow_joint_trajectory"};
  // Command interface the arm and rail controllers must exclusively claim (velocity, so planned
  // joint velocity is fed forward). The gripper keeps its position interface.
  std::string command_interface{"velocity"};
  std::chrono::nanoseconds maximum_age{std::chrono::milliseconds(250)};
  std::chrono::nanoseconds maximum_future_skew{std::chrono::milliseconds(25)};
  std::array<std::string, kExecutionArmJointCount> arm_joints{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
    "wrist_3_joint"};
  std::string rail_joint{"rail_joint"};
};

struct ExecutionControllerAuthority
{
  std::string controller_manager_epoch;
  std::uint64_t source_generation{0U};
  rclcpp::Time observed_at{std::int64_t{0}, RCL_ROS_TIME};
  std::uint64_t contract_fingerprint{0U};
  std::array<std::string, kExecutionControllerCount> controller_names{};
  std::array<std::string, kExecutionJointCount> trajectory_joint_names{};
};

enum class ExecutionControllerAuthorityErrorCode : std::uint8_t
{
  None,
  InvalidContract,
  InvalidSource,
  StaleEvidence,
  FutureEvidence,
  UnstableSnapshot,
  SourceError,
  MissingController,
  DuplicateController,
  ControllerMismatch,
  MissingHardwareInterface,
  DuplicateHardwareInterface,
  HardwareInterfaceMismatch,
  MissingMoveItMapping,
  DuplicateMoveItMapping,
  MoveItMappingMismatch,
};

struct ExecutionControllerAuthorityResult
{
  ExecutionControllerAuthorityErrorCode error{
    ExecutionControllerAuthorityErrorCode::InvalidSource};
  std::optional<ExecutionControllerAuthority> authority;
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept {return authority.has_value();}
};

[[nodiscard]] ExecutionControllerAuthorityResult evaluate_execution_controller_authority(
  const ExecutionControllerEvidence & evidence,
  const rclcpp::Time & ros_now,
  const ExecutionControllerContract & contract = {});

// Returns the contract identity stored in an accepted authority, or nullopt for an invalid
// contract. Downstream consumers use it to reject an authority produced for a different contract.
[[nodiscard]] std::optional<std::uint64_t> fingerprint_execution_controller_contract(
  const ExecutionControllerContract & contract = {});

[[nodiscard]] const char * to_string(ExecutionControllerAuthorityErrorCode value) noexcept;

}  // namespace restocker_control
