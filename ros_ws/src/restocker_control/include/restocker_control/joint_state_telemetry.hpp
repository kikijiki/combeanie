// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include <builtin_interfaces/msg/time.hpp>
#include <restocker_interfaces/msg/robot_telemetry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace restocker_control
{

enum class JointStateTelemetryErrorCode
{
  InvalidSourceId,
  InvalidTimestamp,
  DuplicateJointName,
  MissingRequiredJoint,
  MissingRequiredPosition,
  MissingRequiredVelocity,
  NonFinitePosition,
  NonFiniteVelocity,
  NonMonotonicTimestamp,
};

struct JointStateTelemetryError
{
  JointStateTelemetryErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] JointStateTelemetryResult
{
public:
  [[nodiscard]] static JointStateTelemetryResult success(T value)
  {
    return JointStateTelemetryResult(std::move(value));
  }

  [[nodiscard]] static JointStateTelemetryResult failure(JointStateTelemetryError error)
  {
    return JointStateTelemetryResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const JointStateTelemetryError & error() const
  {
    return std::get<JointStateTelemetryError>(storage_);
  }

private:
  explicit JointStateTelemetryResult(T value)
  : storage_(std::move(value)) {}

  explicit JointStateTelemetryResult(JointStateTelemetryError error)
  : storage_(std::move(error)) {}

  std::variant<T, JointStateTelemetryError> storage_;
};

[[nodiscard]] bool is_valid_telemetry_source_id(std::string_view source_id) noexcept;

[[nodiscard]] JointStateTelemetryResult<restocker_interfaces::msg::RobotTelemetry>
project_joint_state_telemetry(
  const sensor_msgs::msg::JointState & sample, std::string_view source_id,
  const std::optional<builtin_interfaces::msg::Time> & last_published_stamp = std::nullopt);

[[nodiscard]] std::string_view to_string(JointStateTelemetryErrorCode code) noexcept;

}  // namespace restocker_control
