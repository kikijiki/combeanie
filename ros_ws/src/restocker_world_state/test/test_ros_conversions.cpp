// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/robot_execution_state.hpp>
#include <restocker_interfaces/msg/robot_telemetry.hpp>
#include <restocker_interfaces/msg/world_state_snapshot.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_world_state
{
namespace
{

using Message = restocker_interfaces::msg::ObjectObservation;
using LaneMessage = restocker_interfaces::msg::LaneObservation;
using RobotTelemetryMessage = restocker_interfaces::msg::RobotTelemetry;

Message valid_message()
{
  Message message;
  message.header.frame_id = "world";
  message.header.stamp.sec = 10;
  message.source_object_id = "sim:can_01";
  message.product_class = Message::PRODUCT_CLASS_CAN;
  message.has_sku = true;
  message.sku = "SIM-CAN";
  message.pose.pose.position.x = 0.25;
  message.pose.pose.position.y = -0.5;
  message.pose.pose.position.z = 0.6;
  message.pose.pose.orientation.w = 1.0;
  for (std::size_t index = 0; index < 6; ++index) {
    message.pose.covariance[index * 6 + index] = 1.0e-8;
  }
  message.orientation = Message::ORIENTATION_UPRIGHT;
  message.confidence = 1.0F;
  message.backend_name = "gazebo_ground_truth";
  message.backend_version = "test";
  message.status = Message::STATUS_OK;
  return message;
}

TEST(RosObservationConversion, ConvertsCompleteValidatedMessage)
{
  const auto result = object_observation_from_message(valid_message(), 0.99);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().source_object_id, "sim:can_01");
  EXPECT_EQ(result.value().frame_id, "world");
  EXPECT_EQ(result.value().product_class, ProductClass::Can);
  EXPECT_EQ(result.value().sku, "SIM-CAN");
  EXPECT_EQ(result.value().orientation, ObjectOrientation::Upright);
  EXPECT_DOUBLE_EQ(result.value().pose_in_world.translation().x(), 0.25);
  EXPECT_DOUBLE_EQ(result.value().pose_covariance(5, 5), 1.0e-8);
  EXPECT_EQ(result.value().observation_time.nanoseconds(), 10'000'000'000LL);
}

TEST(RosObservationConversion, RejectsStatusProvenanceConfidenceEnumsAndQuaternion)
{
  auto status = valid_message();
  status.status = Message::STATUS_INVALID_POSE;
  EXPECT_FALSE(object_observation_from_message(status, 0.5));

  auto provenance = valid_message();
  provenance.backend_version.clear();
  EXPECT_FALSE(object_observation_from_message(provenance, 0.5));

  auto confidence = valid_message();
  confidence.confidence = 0.49F;
  EXPECT_FALSE(object_observation_from_message(confidence, 0.5));

  auto unknown_enum = valid_message();
  unknown_enum.product_class = 200;
  EXPECT_FALSE(object_observation_from_message(unknown_enum, 0.5));

  auto quaternion = valid_message();
  quaternion.pose.pose.orientation.w = 2.0;
  EXPECT_FALSE(object_observation_from_message(quaternion, 0.5));

  auto non_finite = valid_message();
  non_finite.pose.pose.position.x = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(object_observation_from_message(non_finite, 0.5));
}

LaneMessage valid_lane_message()
{
  LaneMessage message;
  message.header.frame_id = "lane_01";
  message.header.stamp.sec = 10;
  message.lane_id = "lane_01";
  message.available_depth_m = 0.7;
  message.observed_source_object_ids = {"sim:can_01", "sim:can_02"};
  message.confidence = 1.0F;
  message.backend_name = "gazebo_ground_truth";
  message.backend_version = "test";
  message.status = LaneMessage::STATUS_OK;
  return message;
}

TEST(RosLaneObservationConversion, ConvertsCompleteValidatedMessage)
{
  const auto result = lane_observation_from_message(valid_lane_message(), 0.99);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().id.value, "lane_01");
  EXPECT_EQ(
    result.value().observed_source_object_ids,
    std::vector<std::string>({"sim:can_01", "sim:can_02"}));
  EXPECT_DOUBLE_EQ(result.value().available_depth_m, 0.7);
  EXPECT_FALSE(result.value().obstructed);
  EXPECT_EQ(result.value().observation_time.nanoseconds(), 10'000'000'000LL);
}

TEST(RosLaneObservationConversion, RejectsInvalidContractFields)
{
  auto status = valid_lane_message();
  status.status = LaneMessage::STATUS_INVALID_GEOMETRY;
  EXPECT_FALSE(lane_observation_from_message(status, 0.5));

  auto frame = valid_lane_message();
  frame.header.frame_id = "shelf";
  EXPECT_FALSE(lane_observation_from_message(frame, 0.5));

  auto provenance = valid_lane_message();
  provenance.backend_name.clear();
  EXPECT_FALSE(lane_observation_from_message(provenance, 0.5));

  auto confidence = valid_lane_message();
  confidence.confidence = 0.49F;
  EXPECT_FALSE(lane_observation_from_message(confidence, 0.5));

  auto depth = valid_lane_message();
  depth.available_depth_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(lane_observation_from_message(depth, 0.5));

  auto unsorted = valid_lane_message();
  unsorted.observed_source_object_ids = {"sim:can_02", "sim:can_01"};
  EXPECT_FALSE(lane_observation_from_message(unsorted, 0.5));

  auto duplicate = valid_lane_message();
  duplicate.observed_source_object_ids = {"sim:can_01", "sim:can_01"};
  EXPECT_FALSE(lane_observation_from_message(duplicate, 0.5));
}

// Measured coverage floor: unobstructed acquisitions score 1.000, half-blind empty lanes 0.463,
// and the producer itself refuses below 0.90.
constexpr double kNodeDefaultMinimumLaneConfidence = 0.90;
constexpr std::array<float, 8> kCalibratedSensorBand{
  0.60F, 0.65F, 0.70F, 0.75F, 0.80F, 0.85F, 0.90F, 0.95F};

// Coverage at or above the floor is admitted; the camera mid-band below it is refused.
TEST(RosLaneObservationConversion, AdmitsHonestCoverageAtTheMeasuredFloor)
{
  ASSERT_DOUBLE_EQ(kNodeDefaultMinimumLaneConfidence, 0.90) <<
    "the node default changed; update this assertion and the refusal cases below together";
  for (const float confidence : {0.90F, 0.95F, 1.0F}) {
    auto message = valid_lane_message();
    message.confidence = confidence;
    const auto result = lane_observation_from_message(message, kNodeDefaultMinimumLaneConfidence);
    EXPECT_TRUE(result) <<
      "lane observation carrying coverage " << confidence <<
      " was refused at the measured floor of " << kNodeDefaultMinimumLaneConfidence <<
      (result ? std::string{} : (": " + result.error().detail));
  }
  for (const float confidence : {0.60F, 0.65F, 0.70F, 0.75F, 0.80F, 0.85F}) {
    auto message = valid_lane_message();
    message.confidence = confidence;
    const auto result = lane_observation_from_message(message, kNodeDefaultMinimumLaneConfidence);
    EXPECT_FALSE(result) <<
      "lane observation carrying coverage " << confidence <<
      " was admitted below the measured floor of " << kNodeDefaultMinimumLaneConfidence;
  }
}

// A producer refusal status must be rejected at any floor, including zero, so an empty lane the
// producer never saw is not reported as empty.
TEST(RosLaneObservationConversion, RefusesAnObservationTheProducerCouldNotEarnAtEveryFloor)
{
  for (const double configured_floor : {0.0, 0.5, 0.6, 0.9, 0.99}) {
    auto refused = valid_lane_message();
    refused.status = LaneMessage::STATUS_INSUFFICIENT_COVERAGE;
    refused.status_detail = "coverage 0.000 is below the floor 0.900";
    // Emptiest possible lane, as a blinded depth window reports.
    refused.available_depth_m = 0.85;
    refused.observed_source_object_ids.clear();
    refused.confidence = 0.0F;
    const auto result = lane_observation_from_message(refused, configured_floor);
    EXPECT_FALSE(result) <<
      "a lane observation whose producer refused to answer was admitted at a floor of " <<
      configured_floor;

    // Same message with an OK status is admitted, so only the status caused the refusal.
    auto honest = refused;
    honest.status = LaneMessage::STATUS_OK;
    honest.status_detail.clear();
    honest.confidence = 1.0F;
    EXPECT_TRUE(lane_observation_from_message(honest, configured_floor));
  }
}

// Every confidence in the sensor band converts cleanly once the floor admits it.
TEST(RosLaneObservationConversion, AdmitsTheCalibratedSensorBandOnceTheFloorAllowsIt)
{
  for (const float confidence : kCalibratedSensorBand) {
    auto message = valid_lane_message();
    message.confidence = confidence;
    const auto result = lane_observation_from_message(message, 0.60);
    EXPECT_TRUE(result) <<
      "lane observation for " << message.lane_id << " carrying confidence " << confidence <<
      " was discarded against a floor of 0.60 for a reason other than the floor: " <<
      (result ? std::string{} : result.error().detail);
  }
}

// The gate is `>=`. `confidence` is a float32 and the floor a double, so promoting the message
// value decides the at-floor case by literal rounding (0.70, 0.90 and 0.95 round below their
// double; 0.99 rounds the other way). Pin the boundary at several floors.
TEST(RosLaneObservationConversion, AdmitsExactlyTheFloorAtEveryPlausibleFloor)
{
  for (const double configured_floor : {0.5, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 0.99}) {
    auto at_floor = valid_lane_message();
    at_floor.confidence = static_cast<float>(configured_floor);
    const auto admitted = lane_observation_from_message(at_floor, configured_floor);
    EXPECT_TRUE(admitted) <<
      "lane observation for " << at_floor.lane_id << " carrying confidence " <<
      at_floor.confidence << ", exactly the configured floor of " << configured_floor <<
      ", was discarded: " << (admitted ? std::string{} : admitted.error().detail);
  }
}

// One float step below the floor is still refused.
TEST(RosLaneObservationConversion, RejectsTheFloatStepBelowTheFloor)
{
  for (const double configured_floor : {0.5, 0.6, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 0.99}) {
    auto below_floor = valid_lane_message();
    below_floor.confidence = std::nextafter(static_cast<float>(configured_floor), 0.0F);
    const auto refused = lane_observation_from_message(below_floor, configured_floor);
    EXPECT_FALSE(refused) <<
      "lane observation for " << below_floor.lane_id << " carrying confidence " <<
      below_floor.confidence << ", one float step below the configured floor of " <<
      configured_floor << ", was admitted";
  }
}

// The refusal detail must name the lane, its confidence and the floor.
TEST(RosLaneObservationConversion, NamesTheDiscardedLaneItsConfidenceAndTheFloor)
{
  auto message = valid_lane_message();
  message.confidence = 0.82F;
  const auto result = lane_observation_from_message(message, 0.99);
  ASSERT_FALSE(result) << "an observation below the floor must not be admitted";
  const std::string & detail = result.error().detail;
  EXPECT_NE(detail.find("lane_01"), std::string::npos) << detail;
  EXPECT_NE(detail.find("0.82"), std::string::npos) << detail;
  EXPECT_NE(detail.find("0.99"), std::string::npos) << detail;
}

RobotTelemetryMessage valid_robot_telemetry_message()
{
  RobotTelemetryMessage message;
  message.stamp.sec = 10;
  message.source_id = "joint_state_adapter";
  message.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
  message.joint_positions = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  message.joint_velocities = {0.01, -0.02, 0.03, -0.04, 0.05, -0.06};
  message.rail_position = 0.25;
  message.rail_velocity = 0.025;
  message.gripper_joint_names = {"left_finger_joint", "right_finger_joint"};
  message.gripper_joint_positions = {0.01, 0.02};
  message.gripper_joint_velocities = {0.001, 0.002};
  return message;
}

TEST(RosRobotTelemetryConversion, ConvertsCanonicalTelemetry)
{
  const auto result = robot_telemetry_from_message(valid_robot_telemetry_message());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(
    result.value().joint_positions,
    (std::array<double, kArmJointCount>{0.1, -0.2, 0.3, -0.4, 0.5, -0.6}));
  EXPECT_EQ(
    result.value().joint_velocities,
    (std::array<double, kArmJointCount>{0.01, -0.02, 0.03, -0.04, 0.05, -0.06}));
  EXPECT_DOUBLE_EQ(result.value().rail_position, 0.25);
  EXPECT_DOUBLE_EQ(result.value().rail_velocity, 0.025);
  EXPECT_EQ(
    result.value().gripper_joint_positions,
    (std::array<double, kGripperJointCount>{0.01, 0.02}));
  EXPECT_EQ(
    result.value().gripper_joint_velocities,
    (std::array<double, kGripperJointCount>{0.001, 0.002}));
  EXPECT_EQ(result.value().observation_time.nanoseconds(), 10'000'000'000LL);
  EXPECT_EQ(result.value().source_id, "joint_state_adapter");
}

TEST(RosRobotTelemetryConversion, RejectsInvalidIdentityOrderTimeAndNumbers)
{
  auto empty_source = valid_robot_telemetry_message();
  empty_source.source_id.clear();
  EXPECT_FALSE(robot_telemetry_from_message(empty_source));

  auto malformed_source = valid_robot_telemetry_message();
  malformed_source.source_id = " primary adapter ";
  EXPECT_FALSE(robot_telemetry_from_message(malformed_source));

  auto wrong_order = valid_robot_telemetry_message();
  std::swap(wrong_order.joint_names[0], wrong_order.joint_names[1]);
  EXPECT_FALSE(robot_telemetry_from_message(wrong_order));

  auto zero_time = valid_robot_telemetry_message();
  zero_time.stamp.sec = 0;
  EXPECT_FALSE(robot_telemetry_from_message(zero_time));

  auto invalid_joint = valid_robot_telemetry_message();
  invalid_joint.joint_positions[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_joint));

  auto invalid_joint_velocity = valid_robot_telemetry_message();
  invalid_joint_velocity.joint_velocities[3] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_joint_velocity));

  auto invalid_rail = valid_robot_telemetry_message();
  invalid_rail.rail_position = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_rail));

  auto invalid_rail_velocity = valid_robot_telemetry_message();
  invalid_rail_velocity.rail_velocity = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_rail_velocity));

  auto wrong_gripper_order = valid_robot_telemetry_message();
  std::swap(wrong_gripper_order.gripper_joint_names[0], wrong_gripper_order.gripper_joint_names[1]);
  EXPECT_FALSE(robot_telemetry_from_message(wrong_gripper_order));

  auto duplicate_gripper_name = valid_robot_telemetry_message();
  duplicate_gripper_name.gripper_joint_names[1] = "left_finger_joint";
  EXPECT_FALSE(robot_telemetry_from_message(duplicate_gripper_name));

  auto invalid_gripper = valid_robot_telemetry_message();
  invalid_gripper.gripper_joint_positions[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_gripper));

  auto invalid_gripper_velocity = valid_robot_telemetry_message();
  invalid_gripper_velocity.gripper_joint_velocities[1] =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(robot_telemetry_from_message(invalid_gripper_velocity));
}

TEST(RosReservationConversion, PreservesOptionalSourceAndRejectsAmbiguousEncoding)
{
  restocker_interfaces::srv::ReserveTask::Request message;
  message.request_id = "task-0001";
  message.selected_snapshot_revision = 8;
  message.object_id = 1;
  message.object_revision = 5;
  message.has_source_lane = true;
  message.source_lane_id = "stock_lane";
  message.source_lane_revision = 6;
  message.destination_lane_id = "lane_01";
  message.destination_lane_revision = 7;
  const auto converted = reserve_task_request_from_message(message);
  ASSERT_TRUE(converted) << converted.error().detail;
  EXPECT_EQ(converted.value().source_lane, LaneId{"stock_lane"});
  EXPECT_EQ(converted.value().source_lane_revision, 6U);

  message.has_source_lane = false;
  EXPECT_FALSE(reserve_task_request_from_message(message));
  message.source_lane_id.clear();
  message.source_lane_revision = 0;
  const auto without_source = reserve_task_request_from_message(message);
  ASSERT_TRUE(without_source);
  EXPECT_FALSE(without_source.value().source_lane);
  EXPECT_FALSE(without_source.value().source_lane_revision);
}

TEST(RosReservationConversion, ValidatesAllClosedEnums)
{
  using RobotMessage = restocker_interfaces::msg::RobotExecutionState;
  restocker_interfaces::srv::CheckpointTaskState::Request checkpoint;
  checkpoint.token = "token";
  checkpoint.operation_id = "checkpoint-1";
  checkpoint.expected_reservation_id = 42;
  checkpoint.expected_reservation_stage = checkpoint.STAGE_RESERVED;
  checkpoint.expected_reservation_revision = 8;
  checkpoint.task_phase = RobotMessage::TASK_RECOVERING;
  checkpoint.fault_state = RobotMessage::FAULT_RECOVERABLE;
  const auto converted_checkpoint =
    checkpoint_request_from_message(checkpoint, rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME));
  ASSERT_TRUE(converted_checkpoint);
  EXPECT_EQ(converted_checkpoint.value().task_phase, TaskPhase::Recovering);
  EXPECT_EQ(converted_checkpoint.value().expected_reservation_id, 42U);
  checkpoint.task_phase = 200;
  EXPECT_FALSE(
    checkpoint_request_from_message(checkpoint, rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME)));

  restocker_interfaces::srv::CommitReservedDetachment::Request detachment;
  detachment.disposition = detachment.RELEASE_WITHOUT_MEMBERSHIP;
  detachment.released_at.sec = 9;
  detachment.released_at.nanosec = 250'000'000U;
  const auto converted_detachment = detachment_request_from_message(detachment);
  ASSERT_TRUE(converted_detachment);
  EXPECT_EQ(
    converted_detachment.value().disposition,
    DetachmentDisposition::ReleaseWithoutMembership);
  // The placement predicate compares this instant against destination evidence, so it must
  // survive conversion unchanged instead of being re-derived from the service call's arrival.
  EXPECT_EQ(converted_detachment.value().released_at.nanoseconds(), 9'250'000'000LL);
  detachment.released_at.nanosec = 1'000'000'000U;
  EXPECT_FALSE(detachment_request_from_message(detachment));
  detachment.released_at.nanosec = 250'000'000U;
  detachment.disposition = 200;
  EXPECT_FALSE(detachment_request_from_message(detachment));

  restocker_interfaces::srv::ReleaseTaskReservation::Request release;
  release.expected_reservation_id = 7;
  release.expected_reservation_stage = release.EXPECTED_STAGE_RESERVED;
  release.expected_reservation_revision = 11;
  release.outcome = release.OUTCOME_FAILED_SAFE;
  release.terminal_task_phase = RobotMessage::TASK_FAULT;
  release.terminal_fault_state = RobotMessage::FAULT_EXTERNAL_INCONSISTENCY;
  const auto converted_release = release_request_from_message(release);
  ASSERT_TRUE(converted_release);
  EXPECT_EQ(converted_release.value().outcome, ReservationOutcome::FailedSafe);
  EXPECT_EQ(converted_release.value().expected_reservation_id, 7U);
  EXPECT_EQ(converted_release.value().expected_reservation_stage, ReservationStage::Reserved);
  EXPECT_EQ(converted_release.value().expected_reservation_revision, 11U);
  release.terminal_fault_state = 200;
  EXPECT_FALSE(release_request_from_message(release));
  release.terminal_fault_state = RobotMessage::FAULT_EXTERNAL_INCONSISTENCY;
  release.expected_reservation_stage = 200;
  EXPECT_FALSE(release_request_from_message(release));
}

TEST(RosReservationConversion, MapsEveryDomainErrorAndNeverDefaultsToSuccess)
{
  using Status = restocker_interfaces::msg::WorldStateOperationStatus;
  const std::vector<std::pair<WorldStateErrorCode, std::uint16_t>> expectations{
    {WorldStateErrorCode::InvalidArgument, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::ClockMismatch, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::StaleObservation, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::FutureObservation, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::OutOfOrder, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::FrameMismatch, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::IdentityConflict, Status::INVALID_ARGUMENT},
    {WorldStateErrorCode::NotFound, Status::NOT_FOUND},
    {WorldStateErrorCode::Removed, Status::NOT_FOUND},
    {WorldStateErrorCode::RevisionConflict, Status::REVISION_CONFLICT},
    {WorldStateErrorCode::ReservationConflict, Status::RESERVATION_CONFLICT},
    {WorldStateErrorCode::TokenMismatch, Status::TOKEN_MISMATCH},
    {WorldStateErrorCode::PredicateFailed, Status::PREDICATE_FAILED},
    {WorldStateErrorCode::InvalidTransition, Status::INVALID_TRANSITION},
    {WorldStateErrorCode::IdempotencyConflict, Status::IDEMPOTENCY_CONFLICT},
    {WorldStateErrorCode::ResourceExhausted, Status::RESOURCE_EXHAUSTED},
    {WorldStateErrorCode::InvariantViolation, Status::INTERNAL_ERROR},
    {WorldStateErrorCode::AttachmentClockNotReady, Status::ATTACHMENT_CLOCK_NOT_READY},
    {WorldStateErrorCode::ClockAuthorityInhibited, Status::CLOCK_AUTHORITY_INHIBITED},
  };
  for (const auto &[code, expected] : expectations) {
    EXPECT_NO_THROW(static_cast<void>(to_string(code)));
    const auto status = operation_status_from_error(WorldStateError{code, "detail"});
    EXPECT_EQ(status.code, expected);
    EXPECT_NE(status.code, Status::UNSET);
    EXPECT_NE(status.code, Status::OK);
    EXPECT_EQ(status.detail, "detail");
  }
  EXPECT_EQ(operation_status_ok().code, Status::OK);
}

TEST(RosReservationConversion, RoundTripsAndValidatesDiagnosticReservation)
{
  const TaskReservation reservation{42,
    "task-0001",
    ObjectId{17},
    "sim:can_17",
    ProductClass::Can,
    "SIM-CAN",
    LaneId{"stock_lane"},
    LaneId{"lane_01"},
    ReservationStage::Reserved,
    false,
    rclcpp::Time(10'500'000'000LL, RCL_ROS_TIME),
    7,
    6,
    0.85,
    {},
    8,
    ProductClass::Can,
    "SIM-CAN"};
  const auto message = task_reservation_to_message(reservation);
  const auto converted = task_reservation_from_message(message, 8);
  ASSERT_TRUE(converted) << converted.error().detail;
  EXPECT_EQ(converted.value().reservation_id, 42U);
  EXPECT_EQ(converted.value().object_id, ObjectId{17});
  EXPECT_EQ(converted.value().source_lane, LaneId{"stock_lane"});
  EXPECT_EQ(converted.value().destination_lane, LaneId{"lane_01"});
  EXPECT_EQ(converted.value().sku, "SIM-CAN");
  EXPECT_EQ(converted.value().destination_expected_product_class, ProductClass::Can);
  EXPECT_EQ(converted.value().destination_expected_sku, "SIM-CAN");

  auto future_revision = message;
  future_revision.revision = 9;
  EXPECT_FALSE(task_reservation_from_message(future_revision, 8));

  auto same_lane = message;
  same_lane.destination_lane_id = "stock_lane";
  EXPECT_FALSE(task_reservation_from_message(same_lane, 8));

  auto premature_placement = message;
  premature_placement.placed_in_destination = true;
  EXPECT_FALSE(task_reservation_from_message(premature_placement, 8));

  auto ambiguous_sku = message;
  ambiguous_sku.has_sku = false;
  EXPECT_FALSE(task_reservation_from_message(ambiguous_sku, 8));

  // The captured destination policy is optional-SKU the same way the object's own SKU is: the
  // presence flag and the string must agree, or the policy is unreadable.
  auto ambiguous_captured_sku = message;
  ambiguous_captured_sku.has_destination_expected_sku = false;
  EXPECT_FALSE(task_reservation_from_message(ambiguous_captured_sku, 8));
}

TrackedObject tracked_object(ObjectId id, TrackingState state)
{
  TrackedObject object;
  object.id = id;
  object.source_object_id = "sim:" + std::to_string(id.value);
  object.product_class = ProductClass::Can;
  object.pose_in_world = Eigen::Isometry3d::Identity();
  object.pose_covariance = PoseCovariance::Identity() * 1.0e-8;
  object.orientation = ObjectOrientation::Upright;
  object.tracking_state = state;
  object.observation_time = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  object.transition_time = object.observation_time;
  object.revision = id.value;
  return object;
}

restocker_interfaces::msg::WorldStateSnapshot valid_snapshot_message()
{
  WorldStateSnapshot snapshot;
  snapshot.revision = 7;
  auto object = tracked_object(ObjectId{1}, TrackingState::Tracked);
  object.sku = "SIM-CAN";
  object.revision = 5;
  snapshot.objects.emplace(object.id, object);
  snapshot.lanes.emplace(
    LaneId{"stock_lane"},
    ShelfLane{LaneId{"stock_lane"},
      ProductClass::Can,
      "SIM-CAN",
      4,
      {ObjectId{1}},
      {"sim:1"},
      0.85,
      0.70,
      false,
      rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
      false,
      rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME),
      6,
      false,
      6});
  snapshot.lanes.emplace(
    LaneId{"lane_01"}, ShelfLane{LaneId{"lane_01"},
      ProductClass::Can,
      "SIM-CAN",
      4,
      {},
      {},
      0.85,
      0.85,
      false,
      rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
      false,
      rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME),
      7,
      false,
      7});
  snapshot.robot.telemetry_time = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  snapshot.robot.joint_velocities = {0.01, -0.02, 0.03, -0.04, 0.05, -0.06};
  snapshot.robot.rail_velocity = 0.025;
  snapshot.robot.gripper_joint_positions = {0.01, 0.02};
  snapshot.robot.gripper_joint_velocities = {0.001, 0.002};
  snapshot.robot.telemetry_source_id = "test/snapshot-source";
  snapshot.robot.telemetry_revision = 4;
  snapshot.robot.revision = 4;
  snapshot.active_reservation = TaskReservation{42,
    "task-0001",
    ObjectId{1},
    "sim:1",
    ProductClass::Can,
    "SIM-CAN",
    LaneId{"stock_lane"},
    LaneId{"lane_01"},
    ReservationStage::Reserved,
    false,
    rclcpp::Time(10'500'000'000LL, RCL_ROS_TIME),
    7,
    4,
    0.85,
    {},
    7,
    ProductClass::Can,
    "SIM-CAN"};
  snapshot.events.push_back(
    WorldStateEvent{7, rclcpp::Time(10'500'000'000LL, RCL_ROS_TIME),
      EventKind::TaskReserved, ObjectId{1}, LaneId{"lane_01"},
      "reserved"});
  return snapshot_to_message(
    snapshot,
    SnapshotMessageOptions{"world", rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME), true, true});
}

TEST(RosSnapshotDecoding, RoundTripsCompleteAuthoritativeSnapshot)
{
  const auto result = snapshot_from_message(valid_snapshot_message(), "world");
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().revision, 7U);
  ASSERT_EQ(result.value().objects.size(), 1U);
  EXPECT_EQ(result.value().objects.at(ObjectId{1}).source_object_id, "sim:1");
  EXPECT_EQ(result.value().objects.at(ObjectId{1}).sku, "SIM-CAN");
  ASSERT_EQ(result.value().lanes.size(), 2U);
  EXPECT_EQ(
    result.value().lanes.at(LaneId{"stock_lane"}).contents,
    std::vector<ObjectId>{ObjectId{1}});
  ASSERT_TRUE(result.value().active_reservation);
  EXPECT_EQ(result.value().active_reservation->stage, ReservationStage::Reserved);
  EXPECT_EQ(result.value().active_reservation->source_lane, LaneId{"stock_lane"});
  EXPECT_EQ(
    result.value().robot.gripper_joint_positions,
    (std::array<double, kGripperJointCount>{0.01, 0.02}));
  EXPECT_EQ(
    result.value().robot.joint_velocities,
    (std::array<double, kArmJointCount>{0.01, -0.02, 0.03, -0.04, 0.05, -0.06}));
  EXPECT_DOUBLE_EQ(result.value().robot.rail_velocity, 0.025);
  EXPECT_EQ(
    result.value().robot.gripper_joint_velocities,
    (std::array<double, kGripperJointCount>{0.001, 0.002}));
  EXPECT_EQ(result.value().robot.telemetry_source_id, "test/snapshot-source");
  ASSERT_EQ(result.value().events.size(), 1U);
  EXPECT_EQ(result.value().events.front().kind, EventKind::TaskReserved);
}

// Ledger identity trust crosses the boundary as a plain flag plus its appended event kind: the
// withdrawal must survive snapshot decode (that is how the executor and operators see it) and
// no unknown kind may be admitted beside it.
TEST(RosSnapshotConversion, RoundTripsLedgerIdentityTrustAndItsEventKind)
{
  auto wire = valid_snapshot_message();
  for (auto & lane : wire.lanes) {
    if (lane.id == "lane_01") {
      lane.ledger_unreliable = true;
    }
  }
  restocker_interfaces::msg::WorldStateEvent withdrawal;
  withdrawal.revision = 7;
  withdrawal.event_time.sec = 10;
  withdrawal.event_time.nanosec = 500'000'000U;
  withdrawal.kind = restocker_interfaces::msg::WorldStateEvent::LANE_LEDGER_UNRELIABLE;
  withdrawal.has_lane_id = true;
  withdrawal.lane_id = "lane_01";
  withdrawal.detail = "ledger unreliable: measured column grew beyond ledger length";
  wire.events.push_back(withdrawal);

  const auto result = snapshot_from_message(wire, "world");
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(result.value().lanes.at(LaneId{"lane_01"}).ledger_unreliable);
  EXPECT_FALSE(result.value().lanes.at(LaneId{"stock_lane"}).ledger_unreliable);
  ASSERT_EQ(result.value().events.size(), 2U);
  EXPECT_EQ(result.value().events.back().kind, EventKind::LaneLedgerUnreliable);
  EXPECT_EQ(result.value().events.back().lane_id, LaneId{"lane_01"});

  auto unknown_kind = wire;
  unknown_kind.events.back().kind =
    restocker_interfaces::msg::WorldStateEvent::LANE_LEDGER_UNRELIABLE + 1U;
  EXPECT_FALSE(snapshot_from_message(unknown_kind, "world"));

  const auto reencoded = snapshot_to_message(
    result.value(),
    SnapshotMessageOptions{"world", rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME), true, true});
  for (const auto & lane : reencoded.lanes) {
    EXPECT_EQ(lane.ledger_unreliable, lane.id == "lane_01");
  }
}

TEST(RosSnapshotDecoding, RejectsMalformedFrameTimeIdentityPoseAndCovariance)
{
  const auto valid = valid_snapshot_message();
  auto wrong_frame = valid;
  wrong_frame.header.frame_id = "map";
  const auto frame_result = snapshot_from_message(wrong_frame, "world");
  ASSERT_FALSE(frame_result);
  EXPECT_EQ(frame_result.error().code, WorldStateErrorCode::FrameMismatch);

  auto bad_time = valid;
  bad_time.header.stamp.nanosec = 1'000'000'000U;
  EXPECT_FALSE(snapshot_from_message(bad_time, "world"));

  auto duplicate_source = valid;
  duplicate_source.objects.push_back(duplicate_source.objects.front());
  duplicate_source.objects.back().id = 2;
  EXPECT_FALSE(snapshot_from_message(duplicate_source, "world"));

  auto bad_quaternion = valid;
  bad_quaternion.objects.front().pose.pose.orientation.w = 0.5;
  EXPECT_FALSE(snapshot_from_message(bad_quaternion, "world"));

  auto asymmetric_covariance = valid;
  asymmetric_covariance.objects.front().pose.covariance[1] = 1.0;
  EXPECT_FALSE(snapshot_from_message(asymmetric_covariance, "world"));

  auto negative_covariance = valid;
  negative_covariance.objects.front().pose.covariance[0] = -1.0;
  EXPECT_FALSE(snapshot_from_message(negative_covariance, "world"));

  auto malformed_robot_source = valid;
  malformed_robot_source.robot.telemetry_source_id = " invalid source ";
  EXPECT_FALSE(snapshot_from_message(malformed_robot_source, "world"));

  auto missing_robot_source = valid;
  missing_robot_source.robot.telemetry_source_id.clear();
  EXPECT_FALSE(snapshot_from_message(missing_robot_source, "world"));

  auto nonfinite_gripper = valid;
  nonfinite_gripper.robot.gripper_joint_positions[0] =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(snapshot_from_message(nonfinite_gripper, "world"));

  auto nonfinite_velocity = valid;
  nonfinite_velocity.robot.joint_velocities[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(snapshot_from_message(nonfinite_velocity, "world"));
}

TEST(RosSnapshotDecoding, RejectsBrokenMembershipAndAttachmentCoupling)
{
  const auto valid = valid_snapshot_message();
  auto missing_object = valid;
  missing_object.lanes.front().contents = {99};
  EXPECT_FALSE(snapshot_from_message(missing_object, "world"));

  auto duplicate_membership = valid;
  duplicate_membership.lanes.front().contents = {1};
  EXPECT_FALSE(snapshot_from_message(duplicate_membership, "world"));

  auto held_but_free = valid;
  held_but_free.robot.has_held_object = true;
  held_but_free.robot.held_object = 1;
  EXPECT_FALSE(snapshot_from_message(held_but_free, "world"));

  auto attached_but_not_held = valid;
  attached_but_not_held.objects.front().grasp_state =
    restocker_interfaces::msg::TrackedObject::GRASP_ATTACHED;
  EXPECT_FALSE(snapshot_from_message(attached_but_not_held, "world"));
}

TEST(RosSnapshotDecoding, RejectsBrokenReservationAndEventInvariants)
{
  const auto valid = valid_snapshot_message();
  auto reservation_identity = valid;
  reservation_identity.active_reservation.object_source_id = "sim:other";
  EXPECT_FALSE(snapshot_from_message(reservation_identity, "world"));

  auto reservation_stage = valid;
  reservation_stage.active_reservation.stage =
    restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED;
  EXPECT_FALSE(snapshot_from_message(reservation_stage, "world"));

  auto inactive_payload = valid;
  inactive_payload.has_active_reservation = false;
  EXPECT_FALSE(snapshot_from_message(inactive_payload, "world"));

  auto unordered_events = valid;
  unordered_events.events.push_back(unordered_events.events.front());
  unordered_events.events.back().revision = 6;
  EXPECT_FALSE(snapshot_from_message(unordered_events, "world"));

  auto invalid_event_identity = valid;
  invalid_event_identity.events.front().has_object_id = false;
  EXPECT_FALSE(snapshot_from_message(invalid_event_identity, "world"));

  // A reservation whose captured destination policy refuses its own object could never have been
  // granted; the decoder refuses it rather than carrying an impossible grant across the boundary.
  auto impossible_capture = valid;
  impossible_capture.active_reservation.destination_expected_product_class =
    restocker_interfaces::msg::TaskReservation::PRODUCT_CLASS_LARGE_BOTTLE;
  impossible_capture.active_reservation.has_destination_expected_sku = true;
  impossible_capture.active_reservation.destination_expected_sku = "SIM-BOTTLE-LARGE";
  EXPECT_FALSE(snapshot_from_message(impossible_capture, "world"));
}

TEST(RosSnapshotConversion, PreservesOrderingRevisionAndRequestFiltering)
{
  WorldStateSnapshot snapshot;
  snapshot.revision = 7;
  snapshot.objects.emplace(ObjectId{2}, tracked_object(ObjectId{2}, TrackingState::Removed));
  snapshot.objects.emplace(ObjectId{1}, tracked_object(ObjectId{1}, TrackingState::Tracked));
  snapshot.lanes.emplace(
    LaneId{"lane_01"}, ShelfLane{LaneId{"lane_01"},
      ProductClass::Can,
      "SIM-CAN",
      4,
      {ObjectId{1}},
      {"sim:can_01"},
      0.85,
      0.70,
      false,
      rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
      false,
      rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME),
      5,
      false,
      6});
  snapshot.events.push_back(
    WorldStateEvent{1, rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
      EventKind::ObjectObserved, ObjectId{1}, std::nullopt,
      "created"});
  snapshot.active_reservation = TaskReservation{42,
    "task-0001",
    ObjectId{1},
    "sim:1",
    ProductClass::Can,
    "SIM-CAN",
    std::nullopt,
    LaneId{"lane_01"},
    ReservationStage::Reserved,
    false,
    rclcpp::Time(10'500'000'000LL, RCL_ROS_TIME),
    7,
    4,
    0.85,
    {},
    7,
    ProductClass::Can,
    "SIM-CAN"};

  const auto object_message = tracked_object_to_message(snapshot.objects.at(ObjectId{1}));
  EXPECT_EQ(object_message.id, 1U);
  EXPECT_EQ(object_message.source_object_id, "sim:1");
  EXPECT_EQ(object_message.revision, 1U);
  const auto lane_message = shelf_lane_to_message(snapshot.lanes.at(LaneId{"lane_01"}));
  EXPECT_EQ(lane_message.id, "lane_01");
  EXPECT_EQ(lane_message.contents, std::vector<std::uint64_t>{1U});
  EXPECT_EQ(lane_message.evidence_revision, 5U);
  const auto robot_message = robot_execution_state_to_message(snapshot.robot);
  EXPECT_EQ(robot_message.task_phase, robot_message.TASK_IDLE);
  EXPECT_FALSE(robot_message.has_held_object);
  // Nothing held: grasp is identity. The message default is a zero quaternion, which would fail
  // validation on the way back in.
  EXPECT_DOUBLE_EQ(robot_message.grasp_center_from_held_object.orientation.w, 1.0);
  const auto robot_round_trip = robot_execution_state_from_message(robot_message, 7);
  ASSERT_TRUE(robot_round_trip) << robot_round_trip.error().detail;
  EXPECT_TRUE(
    robot_round_trip.value().grasp_center_from_held_object.isApprox(
      Eigen::Isometry3d::Identity()));

  auto carried = robot_message;
  carried.grasp_center_from_held_object.position.z = 0.031;
  carried.grasp_center_from_held_object.orientation.w = 0.984807753012208;
  carried.grasp_center_from_held_object.orientation.z = 0.17364817766693033;
  const auto carried_round_trip = robot_execution_state_from_message(carried, 7);
  ASSERT_TRUE(carried_round_trip) << carried_round_trip.error().detail;
  EXPECT_DOUBLE_EQ(
    carried_round_trip.value().grasp_center_from_held_object.translation().z(), 0.031);

  auto malformed = robot_message;
  malformed.grasp_center_from_held_object.orientation.w = 0.0;
  EXPECT_FALSE(robot_execution_state_from_message(malformed, 7));

  const auto bounded = snapshot_to_message(
    snapshot,
    SnapshotMessageOptions{"world", rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME), false, false});
  ASSERT_EQ(bounded.objects.size(), 1U);
  EXPECT_EQ(bounded.objects.front().id, 1U);
  EXPECT_TRUE(bounded.events.empty());
  EXPECT_EQ(bounded.revision, 7U);
  EXPECT_EQ(bounded.header.frame_id, "world");
  EXPECT_EQ(bounded.header.stamp.sec, 11);
  EXPECT_TRUE(bounded.has_active_reservation);
  EXPECT_EQ(bounded.active_reservation.reservation_id, 42U);
  EXPECT_EQ(bounded.active_reservation.request_id, "task-0001");
  EXPECT_EQ(bounded.active_reservation.object_source_id, "sim:1");
  EXPECT_EQ(bounded.active_reservation.destination_lane_id, "lane_01");
  EXPECT_FALSE(bounded.active_reservation.placed_in_destination);
  ASSERT_EQ(bounded.lanes.size(), 1U);
  EXPECT_EQ(bounded.lanes.front().contents, std::vector<std::uint64_t>{1});
  EXPECT_EQ(
    bounded.lanes.front().observed_source_object_ids,
    std::vector<std::string>{"sim:can_01"});
  EXPECT_EQ(bounded.lanes.front().evidence_revision, 5U);

  const auto complete = snapshot_to_message(
    snapshot,
    SnapshotMessageOptions{"world", rclcpp::Time(11'000'000'000LL, RCL_ROS_TIME), true, true});
  ASSERT_EQ(complete.objects.size(), 2U);
  EXPECT_EQ(complete.objects[0].id, 1U);
  EXPECT_EQ(complete.objects[1].id, 2U);
  ASSERT_EQ(complete.events.size(), 1U);
  EXPECT_TRUE(complete.events.front().has_object_id);
  EXPECT_EQ(complete.events.front().object_id, 1U);
}

}  // namespace
}  // namespace restocker_world_state
