// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <sensor_msgs/msg/joint_state.hpp>

#include "restocker_control/joint_state_telemetry.hpp"

namespace restocker_control
{
namespace
{

sensor_msgs::msg::JointState valid_sample()
{
  sensor_msgs::msg::JointState sample;
  sample.header.stamp.sec = 42;
  sample.header.stamp.nanosec = 123;
  sample.name = {
    "left_finger_joint", "wrist_1_joint", "rail_joint", "shoulder_pan_joint", "wrist_3_joint",
    "shoulder_lift_joint",
    "right_finger_joint", "wrist_2_joint", "elbow_joint"};
  sample.position = {0.01, 4.0, 0.75, 1.0, 6.0, 2.0, 0.02, 5.0, 3.0};
  sample.velocity = {0.001, 0.4, 0.075, 0.1, 0.6, 0.2, 0.002, 0.5, 0.3};
  return sample;
}

TEST(JointStateTelemetry, ProjectsCanonicalOrderAndIgnoresAdditionalJoints)
{
  const auto result = project_joint_state_telemetry(valid_sample(), "control/adapter-v1");
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().stamp.sec, 42);
  EXPECT_EQ(result.value().stamp.nanosec, 123U);
  EXPECT_EQ(result.value().source_id, "control/adapter-v1");
  EXPECT_EQ(
    result.value().joint_names,
    (std::array<std::string, 6>{
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
        "wrist_2_joint", "wrist_3_joint"}));
  EXPECT_EQ(
    result.value().joint_positions, (std::array<double, 6>{1.0, 2.0, 3.0, 4.0, 5.0, 6.0}));
  EXPECT_EQ(
    result.value().joint_velocities,
    (std::array<double, 6>{0.1, 0.2, 0.3, 0.4, 0.5, 0.6}));
  EXPECT_DOUBLE_EQ(result.value().rail_position, 0.75);
  EXPECT_DOUBLE_EQ(result.value().rail_velocity, 0.075);
  EXPECT_EQ(
    result.value().gripper_joint_names,
    (std::array<std::string, 2>{"left_finger_joint", "right_finger_joint"}));
  EXPECT_EQ(result.value().gripper_joint_positions, (std::array<double, 2>{0.01, 0.02}));
  EXPECT_EQ(result.value().gripper_joint_velocities, (std::array<double, 2>{0.001, 0.002}));
}

TEST(JointStateTelemetry, RejectsMissingRequiredJoint)
{
  for (const std::size_t index : {std::size_t{0}, std::size_t{2}, std::size_t{3}}) {
    auto sample = valid_sample();
    sample.name[index] = "unrelated_joint";
    const auto result = project_joint_state_telemetry(sample, "control/adapter");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredJoint);
  }
}

TEST(JointStateTelemetry, RejectsDuplicateRequiredOrAdditionalJoint)
{
  for (const auto & duplicate : {
        std::pair<std::size_t, std::string>{5, "shoulder_pan_joint"},
        std::pair<std::size_t, std::string>{6, "left_finger_joint"}})
  {
    auto sample = valid_sample();
    sample.name[duplicate.first] = duplicate.second;
    const auto result = project_joint_state_telemetry(sample, "control/adapter");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::DuplicateJointName);
  }
}

TEST(JointStateTelemetry, RejectsMissingRequiredPosition)
{
  auto sample = valid_sample();
  sample.position.resize(8);
  const auto result = project_joint_state_telemetry(sample, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredPosition);
}

TEST(JointStateTelemetry, RejectsMismatchedPositionAndVelocityArrays)
{
  auto missing_position = valid_sample();
  missing_position.position.resize(8);
  auto result = project_joint_state_telemetry(missing_position, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredPosition);

  auto missing_velocity = valid_sample();
  missing_velocity.velocity.resize(8);
  result = project_joint_state_telemetry(missing_velocity, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredVelocity);

  auto additional_position = valid_sample();
  additional_position.position.push_back(0.0);
  result = project_joint_state_telemetry(additional_position, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredPosition);

  auto additional_velocity = valid_sample();
  additional_velocity.velocity.push_back(0.0);
  result = project_joint_state_telemetry(additional_velocity, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::MissingRequiredVelocity);
}

TEST(JointStateTelemetry, RejectsNonFiniteRequiredPositions)
{
  for (const std::pair<std::size_t, double> bad_position : {
        std::pair<std::size_t, double>{2, std::numeric_limits<double>::infinity()},
        std::pair<std::size_t, double>{3, std::numeric_limits<double>::quiet_NaN()},
        std::pair<std::size_t, double>{6, std::numeric_limits<double>::infinity()}})
  {
    auto sample = valid_sample();
    sample.position[bad_position.first] = bad_position.second;
    const auto result = project_joint_state_telemetry(sample, "control/adapter");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::NonFinitePosition);
  }
}

TEST(JointStateTelemetry, RejectsNonFiniteRequiredVelocities)
{
  for (const std::pair<std::size_t, double> bad_velocity : {
        std::pair<std::size_t, double>{2, std::numeric_limits<double>::infinity()},
        std::pair<std::size_t, double>{3, std::numeric_limits<double>::quiet_NaN()},
        std::pair<std::size_t, double>{6, std::numeric_limits<double>::infinity()}})
  {
    auto sample = valid_sample();
    sample.velocity[bad_velocity.first] = bad_velocity.second;
    const auto result = project_joint_state_telemetry(sample, "control/adapter");
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::NonFiniteVelocity);
  }
}

TEST(JointStateTelemetry, RejectsZeroMalformedAndNonMonotonicTimestamps)
{
  auto zero = valid_sample();
  zero.header.stamp.sec = 0;
  zero.header.stamp.nanosec = 0;
  auto result = project_joint_state_telemetry(zero, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::InvalidTimestamp);

  auto malformed = valid_sample();
  malformed.header.stamp.nanosec = 1'000'000'000U;
  result = project_joint_state_telemetry(malformed, "control/adapter");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::InvalidTimestamp);

  const auto sample = valid_sample();
  result = project_joint_state_telemetry(sample, "control/adapter", sample.header.stamp);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::NonMonotonicTimestamp);

  auto older_stamp = sample.header.stamp;
  older_stamp.nanosec += 1U;
  result = project_joint_state_telemetry(sample, "control/adapter", older_stamp);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::NonMonotonicTimestamp);
}

TEST(JointStateTelemetry, ValidatesStableSourceIdentityGrammar)
{
  EXPECT_TRUE(is_valid_telemetry_source_id("restocker_control/joint_state_adapter"));
  EXPECT_TRUE(is_valid_telemetry_source_id("A:b-c_d.e/f"));
  EXPECT_FALSE(is_valid_telemetry_source_id(""));
  EXPECT_FALSE(is_valid_telemetry_source_id("/leading-slash"));
  EXPECT_FALSE(is_valid_telemetry_source_id("contains space"));
  EXPECT_FALSE(is_valid_telemetry_source_id(std::string(129, 'a')));

  const auto result = project_joint_state_telemetry(valid_sample(), "/invalid");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, JointStateTelemetryErrorCode::InvalidSourceId);
}

}  // namespace
}  // namespace restocker_control
