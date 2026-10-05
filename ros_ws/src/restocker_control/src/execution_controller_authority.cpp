// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/execution_controller_authority.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace restocker_control
{
namespace
{

[[nodiscard]] ExecutionControllerAuthorityResult failure(
  ExecutionControllerAuthorityErrorCode code, std::string detail)
{
  return {code, std::nullopt, std::move(detail)};
}

template<typename Range>
[[nodiscard]] bool unique_nonempty(const Range & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const auto & value) {
      return !value.empty();
    }) &&
         std::set<std::string>(values.begin(), values.end()).size() == values.size();
}

class Fingerprint
{
public:
  void add(const std::string & value)
  {
    add(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char byte : value) {
      value_ ^= byte;
      value_ *= 1099511628211ULL;
    }
  }
  void add(std::uint64_t value)
  {
    for (std::size_t index = 0; index < sizeof(value); ++index) {
      value_ ^= static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
      value_ *= 1099511628211ULL;
    }
  }
  [[nodiscard]] std::uint64_t value() const noexcept {return value_;}

private:
  std::uint64_t value_{14695981039346656037ULL};
};

[[nodiscard]] std::uint64_t compute_contract_fingerprint(
  const ExecutionControllerContract & contract)
{
  Fingerprint fingerprint;
  fingerprint.add("execution-controller-contract:v2");
  fingerprint.add(contract.arm_controller_name);
  fingerprint.add(contract.rail_controller_name);
  fingerprint.add(contract.trajectory_controller_type);
  fingerprint.add(contract.moveit_controller_type);
  fingerprint.add(contract.moveit_action_namespace);
  fingerprint.add(static_cast<std::uint64_t>(contract.maximum_age.count()));
  fingerprint.add(static_cast<std::uint64_t>(contract.maximum_future_skew.count()));
  for (const auto & joint : contract.arm_joints) {
    fingerprint.add(joint);
  }
  fingerprint.add(contract.rail_joint);
  return fingerprint.value();
}

[[nodiscard]] bool is_relative_graph_name_component(const std::string & value)
{
  return !value.empty() && value.front() != '/' && value.back() != '/';
}

[[nodiscard]] std::string resolved_action_name(
  const std::string & controller_name, const std::string & action_namespace)
{
  return controller_name + "/" + action_namespace;
}

[[nodiscard]] std::optional<std::string> invalid_contract(
  const ExecutionControllerContract & contract)
{
  if (contract.arm_controller_name.empty() || contract.rail_controller_name.empty() ||
    contract.arm_controller_name == contract.rail_controller_name ||
    contract.trajectory_controller_type.empty() || contract.moveit_controller_type.empty() ||
    !is_relative_graph_name_component(contract.arm_controller_name) ||
    !is_relative_graph_name_component(contract.rail_controller_name) ||
    !is_relative_graph_name_component(contract.moveit_action_namespace) ||
    contract.maximum_age.count() <= 0 || contract.maximum_future_skew.count() < 0 ||
    contract.rail_joint.empty() || contract.command_interface.empty() ||
    !unique_nonempty(contract.arm_joints) ||
    std::find(contract.arm_joints.begin(), contract.arm_joints.end(), contract.rail_joint) !=
    contract.arm_joints.end())
  {
    return "execution controller contract contains empty or overlapping identities";
  }
  return std::nullopt;
}

[[nodiscard]] std::vector<std::string> command_interfaces(
  const std::vector<std::string> & joints, const std::string & interface)
{
  std::vector<std::string> result;
  result.reserve(joints.size());
  for (const auto & joint : joints) {
    result.push_back(joint + "/" + interface);
  }
  return result;
}

[[nodiscard]] std::set<std::string> state_interfaces(
  const std::vector<std::string> & joints)
{
  std::set<std::string> result;
  for (const auto & joint : joints) {
    result.insert(joint + "/position");
    result.insert(joint + "/velocity");
  }
  return result;
}

template<typename T>
[[nodiscard]] std::map<std::string, const T *> index_unique(
  const std::vector<T> & values, bool & duplicate)
{
  std::map<std::string, const T *> result;
  for (const auto & value : values) {
    if (value.name.empty() || !result.emplace(value.name, &value).second) {
      duplicate = true;
    }
  }
  return result;
}

[[nodiscard]] std::optional<std::string> invalid_controller(
  const TrajectoryControllerObservation & actual,
  const std::string & expected_name,
  const std::string & expected_action,
  const std::vector<std::string> & expected_joints,
  const ExecutionControllerContract & contract)
{
  if (actual.name != expected_name || actual.type != contract.trajectory_controller_type ||
    actual.state != "active" || actual.action_name != expected_action ||
    !actual.action_server_ready || actual.joints != expected_joints)
  {
    return expected_name + " runtime identity, state, action, or joint order is invalid";
  }
  const auto expected_commands =
    command_interfaces(expected_joints, contract.command_interface);
  const std::set<std::string> actual_commands(
    actual.claimed_command_interfaces.begin(), actual.claimed_command_interfaces.end());
  const std::set<std::string> normalized_expected_commands(
    expected_commands.begin(), expected_commands.end());
  if (actual_commands.size() != actual.claimed_command_interfaces.size() ||
    actual_commands != normalized_expected_commands)
  {
    return expected_name + " does not exclusively claim the configured " +
           contract.command_interface + " interfaces";
  }
  const std::set<std::string> actual_states(
    actual.state_interfaces.begin(), actual.state_interfaces.end());
  const auto expected_states = state_interfaces(expected_joints);
  if (actual_states.size() != actual.state_interfaces.size() || !std::includes(
      actual_states.begin(), actual_states.end(), expected_states.begin(), expected_states.end()))
  {
    return expected_name + " lacks required position or velocity state interfaces";
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::string> invalid_mapping(
  const MoveItControllerMappingObservation & actual,
  const std::string & expected_name,
  const std::vector<std::string> & expected_joints,
  const ExecutionControllerContract & contract)
{
  if (actual.name != expected_name || actual.type != contract.moveit_controller_type ||
    actual.action_namespace != contract.moveit_action_namespace ||
    actual.joints != expected_joints)
  {
    return expected_name + " MoveIt mapping is inconsistent with the execution contract";
  }
  return std::nullopt;
}

}  // namespace

ExecutionControllerAuthorityResult evaluate_execution_controller_authority(
  const ExecutionControllerEvidence & evidence,
  const rclcpp::Time & ros_now,
  const ExecutionControllerContract & contract)
{
  if (const auto invalid = invalid_contract(contract)) {
    return failure(ExecutionControllerAuthorityErrorCode::InvalidContract, *invalid);
  }
  if (evidence.controller_manager_epoch_before.empty() ||
    evidence.controller_manager_epoch_after.empty() || evidence.generation_before == 0U ||
    evidence.generation_after == 0U ||
    evidence.observed_at.get_clock_type() != RCL_ROS_TIME ||
    ros_now.get_clock_type() != RCL_ROS_TIME || evidence.observed_at.nanoseconds() <= 0 ||
    ros_now.nanoseconds() <= 0)
  {
    return failure(
      ExecutionControllerAuthorityErrorCode::InvalidSource,
      "invalid authority source");
  }
  if (evidence.controller_manager_epoch_before != evidence.controller_manager_epoch_after ||
    evidence.generation_before != evidence.generation_after)
  {
    return failure(
      ExecutionControllerAuthorityErrorCode::UnstableSnapshot,
      "controller evidence changed across the double-read fence");
  }
  if (evidence.observed_at <= ros_now &&
    (ros_now - evidence.observed_at).nanoseconds() > contract.maximum_age.count())
  {
    return failure(
      ExecutionControllerAuthorityErrorCode::StaleEvidence,
      "controller evidence is older than the configured freshness limit");
  }
  if (evidence.observed_at > ros_now &&
    (evidence.observed_at - ros_now).nanoseconds() > contract.maximum_future_skew.count())
  {
    return failure(
      ExecutionControllerAuthorityErrorCode::FutureEvidence,
      "controller evidence is beyond the configured future-skew limit");
  }
  if (evidence.controller_manager_error || evidence.hardware_error) {
    return failure(
      ExecutionControllerAuthorityErrorCode::SourceError,
      "controller source reported an error");
  }

  bool duplicate = false;
  const auto controllers = index_unique(evidence.controllers, duplicate);
  if (duplicate) {
    return failure(
      ExecutionControllerAuthorityErrorCode::DuplicateController,
      "duplicate controller identity");
  }
  const auto arm_joints = std::vector<std::string>(
    contract.arm_joints.begin(),
    contract.arm_joints.end());
  const std::vector<std::string> rail_joints{contract.rail_joint};
  for (const auto & expected : std::array{
      std::pair{contract.arm_controller_name, arm_joints},
      std::pair{contract.rail_controller_name, rail_joints}})
  {
    const auto found = controllers.find(expected.first);
    if (found == controllers.end()) {
      return failure(
        ExecutionControllerAuthorityErrorCode::MissingController,
        "missing controller: " + expected.first);
    }
    const auto action = resolved_action_name(expected.first, contract.moveit_action_namespace);
    if (const auto invalid =
      invalid_controller(*found->second, expected.first, action, expected.second, contract))
    {
      return failure(ExecutionControllerAuthorityErrorCode::ControllerMismatch, *invalid);
    }
  }
  const std::set<std::string> required_commands = [&]() {
    auto values = command_interfaces(arm_joints, contract.command_interface);
    const auto rail_commands = command_interfaces(rail_joints, contract.command_interface);
    values.insert(values.end(), rail_commands.begin(), rail_commands.end());
    return std::set<std::string>(values.begin(), values.end());
  }();
  for (const auto & [name, controller] : controllers) {
    if (name == contract.arm_controller_name || name == contract.rail_controller_name) {
      continue;
    }
    for (const auto & claimed : controller->claimed_command_interfaces) {
      if (required_commands.contains(claimed)) {
        return failure(
          ExecutionControllerAuthorityErrorCode::ControllerMismatch,
          name + " also claims required execution interface " + claimed);
      }
    }
  }

  std::map<std::pair<HardwareInterfaceKind, std::string>,
    const HardwareInterfaceObservation *> hardware;
  for (const auto & interface : evidence.hardware_interfaces) {
    const auto key = std::pair{interface.kind, interface.name};
    if (interface.name.empty() || !hardware.emplace(key, &interface).second) {
      return failure(
        ExecutionControllerAuthorityErrorCode::DuplicateHardwareInterface,
        "duplicate hardware interface");
    }
  }
  for (const auto & expected : std::array{
      std::pair{contract.arm_controller_name, arm_joints},
      std::pair{contract.rail_controller_name, rail_joints}})
  {
    for (const auto & joint : expected.second) {
      const auto command_name = joint + "/" + contract.command_interface;
      const auto command = hardware.find({HardwareInterfaceKind::Command, command_name});
      if (command == hardware.end()) {
        return failure(
          ExecutionControllerAuthorityErrorCode::MissingHardwareInterface,
          "missing command interface: " + command_name);
      }
      if (!command->second->available || command->second->claimed_by != expected.first) {
        return failure(
          ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch,
          "invalid command claim: " + command_name);
      }
      for (const auto suffix : {"/position", "/velocity"}) {
        const auto state_name = joint + suffix;
        const auto state = hardware.find({HardwareInterfaceKind::State, state_name});
        if (state == hardware.end()) {
          return failure(
            ExecutionControllerAuthorityErrorCode::MissingHardwareInterface,
            "missing state interface: " + state_name);
        }
        if (!state->second->available || state->second->claimed_by) {
          return failure(
            ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch,
            "invalid state interface: " + state_name);
        }
      }
    }
  }

  duplicate = false;
  const auto mappings = index_unique(evidence.moveit_mappings, duplicate);
  if (duplicate) {
    return failure(
      ExecutionControllerAuthorityErrorCode::DuplicateMoveItMapping,
      "duplicate MoveIt mapping");
  }
  for (const auto & expected : std::array{
      std::pair{contract.arm_controller_name, arm_joints},
      std::pair{contract.rail_controller_name, rail_joints}})
  {
    const auto found = mappings.find(expected.first);
    if (found == mappings.end()) {
      return failure(
        ExecutionControllerAuthorityErrorCode::MissingMoveItMapping,
        "missing MoveIt mapping: " + expected.first);
    }
    if (const auto invalid =
      invalid_mapping(*found->second, expected.first, expected.second, contract))
    {
      return failure(ExecutionControllerAuthorityErrorCode::MoveItMappingMismatch, *invalid);
    }
  }

  ExecutionControllerAuthority authority;
  authority.controller_manager_epoch = evidence.controller_manager_epoch_before;
  authority.source_generation = evidence.generation_before;
  authority.observed_at = evidence.observed_at;
  authority.contract_fingerprint = compute_contract_fingerprint(contract);
  authority.controller_names = {contract.arm_controller_name, contract.rail_controller_name};
  authority.trajectory_joint_names[0] = contract.rail_joint;
  std::copy(
    contract.arm_joints.begin(), contract.arm_joints.end(),
    authority.trajectory_joint_names.begin() + 1);
  return {ExecutionControllerAuthorityErrorCode::None, std::move(authority), {}};
}

std::optional<std::uint64_t> fingerprint_execution_controller_contract(
  const ExecutionControllerContract & contract)
{
  if (invalid_contract(contract)) {
    return std::nullopt;
  }
  return compute_contract_fingerprint(contract);
}

const char * to_string(ExecutionControllerAuthorityErrorCode value) noexcept
{
  switch (value) {
    case ExecutionControllerAuthorityErrorCode::None: return "none";
    case ExecutionControllerAuthorityErrorCode::InvalidContract: return "invalid_contract";
    case ExecutionControllerAuthorityErrorCode::InvalidSource: return "invalid_source";
    case ExecutionControllerAuthorityErrorCode::StaleEvidence: return "stale_evidence";
    case ExecutionControllerAuthorityErrorCode::FutureEvidence: return "future_evidence";
    case ExecutionControllerAuthorityErrorCode::UnstableSnapshot: return "unstable_snapshot";
    case ExecutionControllerAuthorityErrorCode::SourceError: return "source_error";
    case ExecutionControllerAuthorityErrorCode::MissingController: return "missing_controller";
    case ExecutionControllerAuthorityErrorCode::DuplicateController: return "duplicate_controller";
    case ExecutionControllerAuthorityErrorCode::ControllerMismatch: return "controller_mismatch";
    case ExecutionControllerAuthorityErrorCode::MissingHardwareInterface: return
        "missing_hardware_interface";
    case ExecutionControllerAuthorityErrorCode::DuplicateHardwareInterface: return
        "duplicate_hardware_interface";
    case ExecutionControllerAuthorityErrorCode::HardwareInterfaceMismatch: return
        "hardware_interface_mismatch";
    case ExecutionControllerAuthorityErrorCode::MissingMoveItMapping: return
        "missing_moveit_mapping";
    case ExecutionControllerAuthorityErrorCode::DuplicateMoveItMapping: return
        "duplicate_moveit_mapping";
    case ExecutionControllerAuthorityErrorCode::MoveItMappingMismatch: return
        "moveit_mapping_mismatch";
  }
  return "invalid_source";
}

}  // namespace restocker_control
