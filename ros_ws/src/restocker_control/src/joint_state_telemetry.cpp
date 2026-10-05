// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/joint_state_telemetry.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace restocker_control
{
namespace
{

using Result = JointStateTelemetryResult<restocker_interfaces::msg::RobotTelemetry>;

constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;

const std::array<std::string_view, 6> kArmJointNames{
  restocker_interfaces::msg::RobotTelemetry::JOINT_1_NAME,
  restocker_interfaces::msg::RobotTelemetry::JOINT_2_NAME,
  restocker_interfaces::msg::RobotTelemetry::JOINT_3_NAME,
  restocker_interfaces::msg::RobotTelemetry::JOINT_4_NAME,
  restocker_interfaces::msg::RobotTelemetry::JOINT_5_NAME,
  restocker_interfaces::msg::RobotTelemetry::JOINT_6_NAME,
};
constexpr std::string_view kRailJointName{"rail_joint"};
const std::array<std::string_view, 2> kGripperJointNames{
  restocker_interfaces::msg::RobotTelemetry::LEFT_FINGER_JOINT_NAME,
  restocker_interfaces::msg::RobotTelemetry::RIGHT_FINGER_JOINT_NAME,
};

[[nodiscard]] bool is_ascii_alphanumeric(const char value) noexcept
{
  return (value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9');
}

[[nodiscard]] bool is_source_id_suffix_character(const char value) noexcept
{
  return is_ascii_alphanumeric(value) || value == '_' || value == '-' || value == '.' ||
         value == ':' || value == '/';
}

[[nodiscard]] std::optional<std::int64_t> timestamp_nanoseconds(
  const builtin_interfaces::msg::Time & stamp) noexcept
{
  if (stamp.sec < 0 || stamp.nanosec >= static_cast<std::uint32_t>(kNanosecondsPerSecond)) {
    return std::nullopt;
  }
  const auto value = static_cast<std::int64_t>(stamp.sec) * kNanosecondsPerSecond +
    static_cast<std::int64_t>(stamp.nanosec);
  if (value <= 0) {
    return std::nullopt;
  }
  return value;
}

[[nodiscard]] Result failure(JointStateTelemetryErrorCode code, std::string detail)
{
  return Result::failure({code, std::move(detail)});
}

}  // namespace

bool is_valid_telemetry_source_id(const std::string_view source_id) noexcept
{
  if (source_id.empty() || source_id.size() > 128 || !is_ascii_alphanumeric(source_id.front())) {
    return false;
  }
  for (const char character : source_id.substr(1)) {
    if (!is_source_id_suffix_character(character)) {
      return false;
    }
  }
  return true;
}

JointStateTelemetryResult<restocker_interfaces::msg::RobotTelemetry>
project_joint_state_telemetry(
  const sensor_msgs::msg::JointState & sample, const std::string_view source_id,
  const std::optional<builtin_interfaces::msg::Time> & last_published_stamp)
{
  if (!is_valid_telemetry_source_id(source_id)) {
    return failure(
      JointStateTelemetryErrorCode::InvalidSourceId,
      "source_id must satisfy the normalized 1-128 character identity grammar");
  }

  const auto sample_time = timestamp_nanoseconds(sample.header.stamp);
  if (!sample_time) {
    return failure(
      JointStateTelemetryErrorCode::InvalidTimestamp,
      "joint-state timestamp must be positive and normalized");
  }
  if (last_published_stamp) {
    const auto previous_time = timestamp_nanoseconds(*last_published_stamp);
    if (!previous_time) {
      return failure(
        JointStateTelemetryErrorCode::InvalidTimestamp,
        "last-published timestamp fence is malformed");
    }
    if (*sample_time <= *previous_time) {
      return failure(
        JointStateTelemetryErrorCode::NonMonotonicTimestamp,
        "joint-state timestamp is not newer than the last published sample");
    }
  }

  if (sample.position.size() != sample.name.size()) {
    return failure(
      JointStateTelemetryErrorCode::MissingRequiredPosition,
      "joint-state position array must exactly match the name array");
  }
  if (sample.velocity.size() != sample.name.size()) {
    return failure(
      JointStateTelemetryErrorCode::MissingRequiredVelocity,
      "joint-state velocity array must exactly match the name array");
  }

  std::unordered_map<std::string_view, std::size_t> joint_indices;
  joint_indices.reserve(sample.name.size());
  std::unordered_set<std::string_view> observed_names;
  observed_names.reserve(sample.name.size());
  for (std::size_t index = 0; index < sample.name.size(); ++index) {
    const auto name = std::string_view(sample.name[index]);
    if (!observed_names.insert(name).second) {
      return failure(
        JointStateTelemetryErrorCode::DuplicateJointName,
        "joint-state sample repeats joint name '" + std::string(name) + "'");
    }
    joint_indices.emplace(name, index);
  }

  restocker_interfaces::msg::RobotTelemetry output;
  output.stamp = sample.header.stamp;
  output.source_id = source_id;

  for (std::size_t output_index = 0; output_index < kArmJointNames.size(); ++output_index) {
    const auto required_name = kArmJointNames[output_index];
    const auto found = joint_indices.find(required_name);
    if (found == joint_indices.end()) {
      return failure(
        JointStateTelemetryErrorCode::MissingRequiredJoint,
        "joint-state sample is missing required joint '" + std::string(required_name) + "'");
    }
    if (found->second >= sample.position.size()) {
      return failure(
        JointStateTelemetryErrorCode::MissingRequiredPosition,
        "joint-state sample has no position for required joint '" + std::string(required_name) +
        "'");
    }
    const double position = sample.position[found->second];
    if (!std::isfinite(position)) {
      return failure(
        JointStateTelemetryErrorCode::NonFinitePosition,
        "joint-state sample has a non-finite position for required joint '" +
        std::string(required_name) + "'");
    }
    output.joint_names[output_index] = required_name;
    output.joint_positions[output_index] = position;
    const double velocity = sample.velocity[found->second];
    if (!std::isfinite(velocity)) {
      return failure(
        JointStateTelemetryErrorCode::NonFiniteVelocity,
        "joint-state sample has a non-finite velocity for required joint '" +
        std::string(required_name) + "'");
    }
    output.joint_velocities[output_index] = velocity;
  }

  const auto rail = joint_indices.find(kRailJointName);
  if (rail == joint_indices.end()) {
    return failure(
      JointStateTelemetryErrorCode::MissingRequiredJoint,
      "joint-state sample is missing required joint 'rail_joint'");
  }
  if (rail->second >= sample.position.size()) {
    return failure(
      JointStateTelemetryErrorCode::MissingRequiredPosition,
      "joint-state sample has no position for required joint 'rail_joint'");
  }
  output.rail_position = sample.position[rail->second];
  if (!std::isfinite(output.rail_position)) {
    return failure(
      JointStateTelemetryErrorCode::NonFinitePosition,
      "joint-state sample has a non-finite position for required joint 'rail_joint'");
  }
  output.rail_velocity = sample.velocity[rail->second];
  if (!std::isfinite(output.rail_velocity)) {
    return failure(
      JointStateTelemetryErrorCode::NonFiniteVelocity,
      "joint-state sample has a non-finite velocity for required joint 'rail_joint'");
  }

  for (std::size_t output_index = 0; output_index < kGripperJointNames.size(); ++output_index) {
    const auto required_name = kGripperJointNames[output_index];
    const auto found = joint_indices.find(required_name);
    if (found == joint_indices.end()) {
      return failure(
        JointStateTelemetryErrorCode::MissingRequiredJoint,
        "joint-state sample is missing required joint '" + std::string(required_name) + "'");
    }
    if (found->second >= sample.position.size()) {
      return failure(
        JointStateTelemetryErrorCode::MissingRequiredPosition,
        "joint-state sample has no position for required joint '" + std::string(required_name) +
        "'");
    }
    const double position = sample.position[found->second];
    if (!std::isfinite(position)) {
      return failure(
        JointStateTelemetryErrorCode::NonFinitePosition,
        "joint-state sample has a non-finite position for required joint '" +
        std::string(required_name) + "'");
    }
    output.gripper_joint_names[output_index] = required_name;
    output.gripper_joint_positions[output_index] = position;
    const double velocity = sample.velocity[found->second];
    if (!std::isfinite(velocity)) {
      return failure(
        JointStateTelemetryErrorCode::NonFiniteVelocity,
        "joint-state sample has a non-finite velocity for required joint '" +
        std::string(required_name) + "'");
    }
    output.gripper_joint_velocities[output_index] = velocity;
  }

  return Result::success(std::move(output));
}

std::string_view to_string(const JointStateTelemetryErrorCode code) noexcept
{
  switch (code) {
    case JointStateTelemetryErrorCode::InvalidSourceId:
      return "invalid_source_id";
    case JointStateTelemetryErrorCode::InvalidTimestamp:
      return "invalid_timestamp";
    case JointStateTelemetryErrorCode::DuplicateJointName:
      return "duplicate_joint_name";
    case JointStateTelemetryErrorCode::MissingRequiredJoint:
      return "missing_required_joint";
    case JointStateTelemetryErrorCode::MissingRequiredPosition:
      return "missing_required_position";
    case JointStateTelemetryErrorCode::MissingRequiredVelocity:
      return "missing_required_velocity";
    case JointStateTelemetryErrorCode::NonFinitePosition:
      return "non_finite_position";
    case JointStateTelemetryErrorCode::NonFiniteVelocity:
      return "non_finite_velocity";
    case JointStateTelemetryErrorCode::NonMonotonicTimestamp:
      return "non_monotonic_timestamp";
  }
  return "unknown";
}

}  // namespace restocker_control
