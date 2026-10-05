// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "restocker_world_state/ros_conversions.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{
namespace
{
using namespace std::chrono_literals;
using Status = restocker_interfaces::msg::WorldStateOperationStatus;

rclcpp::Time at(std::int64_t ns)
{
  return rclcpp::Time(ns, RCL_ROS_TIME);
}

WorldStateConfig temporal_config()
{
  WorldStateConfig value;
  value.maximum_observation_age = 2s;
  value.lane_evidence_validity = 60s;
  value.maximum_future_skew = 100ms;
  // Only the reserved lifecycle fits. Readiness and conflicts cannot consume its cleanup room.
  value.operation_journal_capacity = 4;
  value.product_lane_profiles = {{ProductClass::Can, std::nullopt, 0.111, 0.066}};
  return value;
}

ObjectObservation observation(std::int64_t ns)
{
  ObjectObservation value;
  value.source_object_id = "temporal-object";
  value.frame_id = "world";
  value.product_class = ProductClass::Can;
  value.pose_in_world = Eigen::Isometry3d::Identity();
  value.pose_covariance = PoseCovariance::Identity() * 1.0e-6;
  value.orientation = ObjectOrientation::Upright;
  value.observation_time = at(ns);
  return value;
}

class AttachmentTemporalAdmission : public ::testing::Test
{
protected:
  WorldStateStore store{temporal_config()};
  ReservationReceipt reserved;
  ReservedAttachmentRequest request;
  ObjectId object_id;

  AuthorityClockSample sample(std::int64_t ns, bool managed = true)
  {
    return store.capture_authority_clock(at(ns), managed);
  }

  Result<ObservationReceipt> observe(
    std::int64_t capture, std::int64_t receipt,
    bool managed = true)
  {
    const auto clock = sample(receipt, managed);
    return store.observe_object(observation(capture), clock.time, clock);
  }

  ReservedAttachmentResult commit(std::int64_t ns)
  {
    const auto clock = sample(ns);
    return store.commit_reserved_attachment(request, clock.time, clock);
  }

  void SetUp() override
  {
    const auto lane = store.configure_lane(
      LaneDefinition{LaneId{"lane"}, ProductClass::Can, std::nullopt, 0.85}, at(9'000'000'000));
    ASSERT_TRUE(lane);
    const auto lane_clock = sample(9'400'000'000);
    ASSERT_TRUE(
      store.update_lane(
        LaneObservation{LaneId{"lane"}, {}, 0.85, false, lane_clock.time},
        lane_clock.time, lane.value().revision, lane_clock));
    const auto object = observe(9'500'000'000, 9'500'000'000);
    ASSERT_TRUE(object);
    object_id = object.value().object_id;
    const auto robot_clock = sample(9'550'000'000);
    ASSERT_TRUE(
      store.observe_robot_telemetry(
        RobotTelemetryObservation{{}, {}, 0.0, 0.0, robot_clock.time, {}, {}, "temporal-robot"},
        robot_clock.time, robot_clock));
    const auto before = store.snapshot();
    const auto reserve_clock = sample(9'600'000'000);
    const auto result = store.reserve_task(
      ReserveTaskRequest{"temporal-reserve", before.revision, object_id,
        before.objects.at(object_id).revision, std::nullopt, std::nullopt, LaneId{"lane"},
        before.lanes.at(LaneId{"lane"}).revision}, reserve_clock.time, reserve_clock);
    ASSERT_TRUE(result) << result.error().detail;
    reserved = result.value();
    request = {reserved.token, "temporal-attach", Eigen::Isometry3d::Identity()};
  }
};

TEST_F(AttachmentTemporalAdmission, PausedReadinessDoesNotSpendReservedJournalCapacity)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  const auto before = store.snapshot();
  for (int attempt = 0; attempt < 100; ++attempt) {
    const auto pending = commit(9'660'000'000);
    ASSERT_FALSE(pending);
    EXPECT_EQ(pending.error().code, WorldStateErrorCode::AttachmentClockNotReady);
    ASSERT_TRUE(pending.error().attachment_temporal_refusal);
    const auto & evidence = *pending.error().attachment_temporal_refusal;
    EXPECT_EQ(evidence.object_received_at_ns, 9'650'000'000);
    EXPECT_EQ(evidence.object_clock_lineage, evidence.commit_clock_lineage);
    EXPECT_TRUE(evidence.object_managed_ros_started);
    EXPECT_TRUE(evidence.commit_managed_ros_started);
    EXPECT_FALSE(pending.historical_receipt());
    EXPECT_EQ(store.snapshot().revision, before.revision);
  }
  const auto applied = commit(9'700'000'000);
  ASSERT_TRUE(applied) << applied.error().detail;
  const auto detached = store.commit_reserved_detachment(
    {reserved.token, "detach", DetachmentDisposition::ReleaseWithoutMembership, at(9'710'000'000)},
    at(9'720'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  const auto & receipt = detached.value();
  ASSERT_TRUE(
    store.release_task_reservation(
      {receipt.token, "release", receipt.reservation.reservation_id,
        receipt.reservation.stage, receipt.reservation.revision, ReservationOutcome::FailedSafe,
        TaskPhase::Fault, FaultState::Recoverable}, at(9'730'000'000)));
}

TEST_F(AttachmentTemporalAdmission, NewFutureObservationMovesReadinessWithoutEarlyMutation)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  EXPECT_EQ(commit(9'660'000'000).error().code, WorldStateErrorCode::AttachmentClockNotReady);
  ASSERT_TRUE(observe(9'750'000'000, 9'690'000'000));
  const auto before = store.snapshot().revision;
  EXPECT_EQ(commit(9'700'000'000).error().code, WorldStateErrorCode::AttachmentClockNotReady);
  EXPECT_EQ(store.snapshot().revision, before);
  ASSERT_TRUE(commit(9'750'000'000));
}

TEST_F(AttachmentTemporalAdmission, AppliedPoseObservationKeepsItsCaptureAndReadiness)
{
  auto moved = observation(9'700'000'000);
  moved.pose_in_world.translation().x() = 0.2;
  const auto clock = sample(9'650'000'000);
  ASSERT_TRUE(store.observe_object(moved, clock.time, clock));
  const auto before = store.snapshot();
  EXPECT_DOUBLE_EQ(before.objects.at(object_id).pose_in_world.translation().x(), 0.2);
  EXPECT_EQ(commit(9'660'000'000).error().code, WorldStateErrorCode::AttachmentClockNotReady);
  EXPECT_EQ(store.snapshot().revision, before.revision);
  ASSERT_TRUE(commit(9'700'000'000));
}

TEST_F(AttachmentTemporalAdmission, DelayedCapturesAdvanceByCaptureTimeNotPriorReceipt)
{
  ASSERT_TRUE(observe(9'610'000'000, 9'640'000'000));
  ASSERT_TRUE(observe(9'620'000'000, 9'650'000'000));
  EXPECT_EQ(observe(9'615'000'000, 9'660'000'000).error().code, WorldStateErrorCode::OutOfOrder);
  EXPECT_EQ(store.snapshot().objects.at(object_id).observation_time.nanoseconds(), 9'620'000'000);
  ASSERT_TRUE(commit(9'670'000'000));
}

TEST_F(AttachmentTemporalAdmission, MissingManagedAdmissionProvenanceRemainsHardRefusal)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000, false));
  EXPECT_EQ(commit(9'660'000'000).error().code, WorldStateErrorCode::OutOfOrder);
  ASSERT_TRUE(commit(9'700'000'000));
}

TEST_F(AttachmentTemporalAdmission, UnmanagedCommitCannotBorrowManagedObservationEligibility)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  const auto clock = sample(9'660'000'000, false);
  const auto result = store.commit_reserved_attachment(request, clock.time, clock);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStateErrorCode::OutOfOrder);
}

TEST_F(AttachmentTemporalAdmission, NewerRobotDeliveryIsNotMisclassifiedAsClockRollback)
{
  const auto earlier_commit = sample(9'640'000'000);
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  const auto robot_clock = sample(9'670'000'000);
  ASSERT_TRUE(
    store.observe_robot_telemetry(
      RobotTelemetryObservation{{}, {}, 0.0, 0.0, at(9'660'000'000), {}, {}, "temporal-robot"},
      robot_clock.time, robot_clock));
  const auto result = store.commit_reserved_attachment(
    request, earlier_commit.time,
    earlier_commit);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStateErrorCode::OutOfOrder);
  EXPECT_TRUE(result.error().attachment_temporal_refusal->robot_postdates_commit);
  // Delivering an earlier retained sample did not poison current authority.
  ASSERT_TRUE(commit(9'700'000'000));
}

TEST_F(AttachmentTemporalAdmission, AttachAndDetachKeepTheirDelayedObservationFences)
{
  ASSERT_TRUE(commit(9'700'000'000));
  EXPECT_EQ(observe(9'690'000'000, 9'710'000'000).error().code, WorldStateErrorCode::OutOfOrder);
  auto different = request;
  different.operation_id = "second-attach";
  const auto clock = sample(9'720'000'000);
  EXPECT_EQ(
    store.commit_reserved_attachment(different, clock.time, clock).error().code,
    WorldStateErrorCode::InvalidTransition);
  ASSERT_TRUE(
    store.commit_reserved_detachment(
      {reserved.token, "detach", DetachmentDisposition::ReleaseWithoutMembership,
        at(9'720'000'000)}, at(9'730'000'000)));
  EXPECT_EQ(observe(9'720'000'000, 9'740'000'000).error().code, WorldStateErrorCode::OutOfOrder);
  ASSERT_TRUE(observe(9'750'000'000, 9'760'000'000));
}

TEST_F(AttachmentTemporalAdmission, InvalidCapabilityAndExcessSkewDoNotBecomeReadiness)
{
  const auto rejected = observe(9'800'000'001, 9'700'000'000);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::FutureObservation);
  ASSERT_TRUE(observe(9'800'000'000, 9'700'000'000));
  const auto clock = sample(9'710'000'000);
  auto wrong = request;
  wrong.token = "another-owner";
  const auto result = store.commit_reserved_attachment(wrong, clock.time, clock);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStateErrorCode::TokenMismatch);
  ASSERT_TRUE(commit(9'800'000'000));
}

TEST_F(AttachmentTemporalAdmission, ClockResetInhibitsWithoutAuthoritativeMutation)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  const auto before = store.snapshot();
  store.notify_clock_discontinuity();
  EXPECT_EQ(commit(9'800'000'000).error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  EXPECT_EQ(
    observe(9'900'000'000, 9'900'000'000).error().code,
    WorldStateErrorCode::ClockAuthorityInhibited);
  EXPECT_EQ(store.snapshot().revision, before.revision);
  const auto proof = store.validate_execution_world_authority(
    reserved.token,
    {reserved.reservation.reservation_id, reserved.reservation.revision, object_id,
      LaneId{"lane"}});
  ASSERT_FALSE(proof);
  EXPECT_EQ(proof.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST_F(AttachmentTemporalAdmission, DecreasingOrderedSampleLatchesEvenWithoutJumpDelivery)
{
  sample(9'700'000'000);
  const auto reset = sample(9'699'999'999);
  EXPECT_EQ(
    store.commit_reserved_attachment(request, reset.time, reset).error().code,
    WorldStateErrorCode::ClockAuthorityInhibited);
  EXPECT_EQ(commit(10'000'000'000).error().code, WorldStateErrorCode::ClockAuthorityInhibited);
}

TEST_F(AttachmentTemporalAdmission, HistoricalReplayAfterResetIsEvidenceAndNotOrdinarySuccess)
{
  const auto applied = commit(9'700'000'000);
  ASSERT_TRUE(applied);
  const auto detached = store.commit_reserved_detachment(
    {reserved.token, "detach", DetachmentDisposition::ReleaseWithoutMembership, at(9'710'000'000)},
    at(9'720'000'000));
  ASSERT_TRUE(detached);
  const auto & receipt = detached.value();
  ASSERT_TRUE(
    store.release_task_reservation(
      {receipt.token, "release", receipt.reservation.reservation_id,
        receipt.reservation.stage, receipt.reservation.revision, ReservationOutcome::FailedSafe,
        TaskPhase::Fault, FaultState::Recoverable}, at(9'730'000'000)));
  const auto before = store.snapshot();
  const auto replay = commit(9'740'000'000);
  ASSERT_TRUE(replay);
  EXPECT_EQ(replay.value().revision, applied.value().revision);
  store.notify_clock_discontinuity();
  const auto inhibited = commit(9'750'000'000);
  ASSERT_FALSE(inhibited);
  EXPECT_EQ(inhibited.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  ASSERT_TRUE(inhibited.historical_receipt());
  EXPECT_EQ(inhibited.historical_receipt()->revision, applied.value().revision);
  EXPECT_EQ(inhibited.historical_receipt()->reservation.stage, ReservationStage::Attached);
  EXPECT_EQ(store.snapshot().revision, before.revision);
  const auto wire = attachment_result_to_message(inhibited, before.revision);
  EXPECT_EQ(wire.status.code, Status::CLOCK_AUTHORITY_INHIBITED);
  EXPECT_EQ(wire.world_revision, applied.value().revision);
  EXPECT_TRUE(wire.has_reservation);
  EXPECT_EQ(wire.reservation.object_id, object_id.value);
  EXPECT_EQ(wire.reservation.stage, restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED);
  request.grasp_center_from_held_object.translation().x() = 0.01;
  EXPECT_EQ(commit(9'760'000'000).error().code, WorldStateErrorCode::IdempotencyConflict);
}

TEST_F(AttachmentTemporalAdmission, TransformFingerprintUsesExactPostConversionValues)
{
  restocker_interfaces::srv::CommitReservedAttachment::Request wire;
  wire.token = request.token;
  wire.operation_id = request.operation_id;
  wire.grasp_center_from_held_object.orientation.z = std::sin(0.185);
  wire.grasp_center_from_held_object.orientation.w = std::cos(0.185);
  const auto converted = attachment_request_from_message(wire);
  ASSERT_TRUE(converted);
  request = converted.value();
  const auto applied = commit(9'700'000'000);
  ASSERT_TRUE(applied);
  wire.grasp_center_from_held_object.orientation.z *= -1;
  wire.grasp_center_from_held_object.orientation.w *= -1;
  wire.grasp_center_from_held_object.position.x = -0.0;
  const auto equivalent = attachment_request_from_message(wire);
  ASSERT_TRUE(equivalent);
  request = equivalent.value();
  const auto replay = commit(9'710'000'000);
  ASSERT_TRUE(replay);
  EXPECT_EQ(replay.value().revision, applied.value().revision);
  // Far below approximate pose tolerances, but still a different finite payload.
  request.grasp_center_from_held_object.translation().x() = std::numeric_limits<double>::min();
  EXPECT_EQ(commit(9'720'000'000).error().code, WorldStateErrorCode::IdempotencyConflict);
  request = equivalent.value();
  request.grasp_center_from_held_object.translation().x() = 0.001;
  EXPECT_EQ(commit(9'730'000'000).error().code, WorldStateErrorCode::IdempotencyConflict);
  // Exercise every encoded coefficient at adjacent representable precision. These values pass
  // the existing finite/rotation validation; fingerprinting must not round any of them away.
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      request = equivalent.value();
      double & coefficient = request.grasp_center_from_held_object.matrix()(row, column);
      coefficient = std::nextafter(coefficient, coefficient + 1.0);
      EXPECT_EQ(commit(9'740'000'000).error().code, WorldStateErrorCode::IdempotencyConflict)
        << row << "," << column;
    }
  }
  EXPECT_EQ(store.snapshot().revision, applied.value().revision);
}

// Card CMB-096 SC-002, quaternion identity semantics. The wire quaternion is not part of the
// request identity: `attachment_request_from_message` admits the orientation, normalizes it and
// keeps only the 4x4 grasp matrix, and `attachment_fingerprint` hashes those 16 post-conversion
// coefficients. `q` and `-q` are the same rotation, their matrices are bit-identical (every
// entry of Eigen's `toRotationMatrix` is a product of exactly two quaternion factors or a
// square, invariant under a global sign flip), so the negated retransmission is the same
// request: it must replay the original receipt, not conflict and not transition state.
TEST_F(
  AttachmentTemporalAdmission,
  QuaternionNegationOfSameRotationReplaysIdenticallyThroughRealConversion)
{
  restocker_interfaces::srv::CommitReservedAttachment::Request wire;
  wire.token = request.token;
  wire.operation_id = "attach-q-negation";
  const Eigen::Quaterniond unit(
    Eigen::AngleAxisd(0.9, Eigen::Vector3d(1.0, 2.0, -0.5).normalized()));
  wire.grasp_center_from_held_object.orientation.w = unit.w();
  wire.grasp_center_from_held_object.orientation.x = unit.x();
  wire.grasp_center_from_held_object.orientation.y = unit.y();
  wire.grasp_center_from_held_object.orientation.z = unit.z();
  wire.grasp_center_from_held_object.position.x = 0.012;
  wire.grasp_center_from_held_object.position.y = -0.034;
  wire.grasp_center_from_held_object.position.z = 0.045;
  const auto original_conversion = attachment_request_from_message(wire);
  ASSERT_TRUE(original_conversion) << original_conversion.error().detail;
  request = original_conversion.value();
  const auto applied = commit(9'700'000'000);
  ASSERT_TRUE(applied) << applied.error().detail;

  wire.grasp_center_from_held_object.orientation.w = -unit.w();
  wire.grasp_center_from_held_object.orientation.x = -unit.x();
  wire.grasp_center_from_held_object.orientation.y = -unit.y();
  wire.grasp_center_from_held_object.orientation.z = -unit.z();
  const auto negated_conversion = attachment_request_from_message(wire);
  ASSERT_TRUE(negated_conversion) << negated_conversion.error().detail;
  const auto & original_matrix = original_conversion.value().grasp_center_from_held_object.matrix();
  const auto & negated_matrix = negated_conversion.value().grasp_center_from_held_object.matrix();
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      EXPECT_EQ(
        std::bit_cast<std::uint64_t>(original_matrix(row, column)),
        std::bit_cast<std::uint64_t>(negated_matrix(row, column)))
        << "q and -q must convert to bit-identical coefficients at " << row << "," << column;
    }
  }
  request = negated_conversion.value();
  const auto replay = commit(9'710'000'000);
  ASSERT_TRUE(replay) << replay.error().detail;
  EXPECT_EQ(replay.value().revision, applied.value().revision);
  EXPECT_EQ(replay.value().token, applied.value().token);
  EXPECT_EQ(store.snapshot().revision, applied.value().revision);
}

// Card CMB-096 SC-002, the other half of quaternion identity: a non-unit wire quaternion is
// admitted only inside the 1e-6 unit band of `pose_from_message`, which then normalizes before
// the fingerprint sees the matrix. For this dyadic component c = 0.5 + 2^-26 the wire norm is
// 2c = 1 + 2^-25 (non-unit, inside the band) and the normalization c/(2c) = 0.5 is exact, so
// the non-unit wire and its normalized equivalent convert to bit-identical matrices. Request
// identity is exact post-conversion coefficient equality with no approximate tolerance: these
// two are the same request because conversion makes them equal, not because anything rounds
// them together afterwards.
TEST_F(
  AttachmentTemporalAdmission,
  NonUnitInBandQuaternionReplaysAgainstItsNormalizedEquivalent)
{
  const double non_unit_component = std::ldexp(1.0, -1) + std::ldexp(1.0, -26);
  const Eigen::Quaterniond raw(
    non_unit_component, non_unit_component, non_unit_component, non_unit_component);
  ASSERT_NE(raw.norm(), 1.0) << "the wire quaternion must actually be non-unit";
  ASSERT_LE(std::abs(raw.norm() - 1.0), 1.0e-6) << "but inside the admission band";

  restocker_interfaces::srv::CommitReservedAttachment::Request wire;
  wire.token = request.token;
  wire.operation_id = "attach-non-unit";
  wire.grasp_center_from_held_object.orientation.w = non_unit_component;
  wire.grasp_center_from_held_object.orientation.x = non_unit_component;
  wire.grasp_center_from_held_object.orientation.y = non_unit_component;
  wire.grasp_center_from_held_object.orientation.z = non_unit_component;
  wire.grasp_center_from_held_object.position.x = 0.012;
  wire.grasp_center_from_held_object.position.y = -0.034;
  wire.grasp_center_from_held_object.position.z = 0.045;
  const auto non_unit = attachment_request_from_message(wire);
  ASSERT_TRUE(non_unit) << non_unit.error().detail;

  wire.grasp_center_from_held_object.orientation.w = 0.5;
  wire.grasp_center_from_held_object.orientation.x = 0.5;
  wire.grasp_center_from_held_object.orientation.y = 0.5;
  wire.grasp_center_from_held_object.orientation.z = 0.5;
  const auto normalized_equivalent = attachment_request_from_message(wire);
  ASSERT_TRUE(normalized_equivalent) << normalized_equivalent.error().detail;
  const auto & non_unit_matrix = non_unit.value().grasp_center_from_held_object.matrix();
  const auto & normalized_matrix =
    normalized_equivalent.value().grasp_center_from_held_object.matrix();
  for (Eigen::Index row = 0; row < 4; ++row) {
    for (Eigen::Index column = 0; column < 4; ++column) {
      EXPECT_EQ(
        std::bit_cast<std::uint64_t>(non_unit_matrix(row, column)),
        std::bit_cast<std::uint64_t>(normalized_matrix(row, column)))
        << "admission normalization must be exact for this value at " << row << "," << column;
    }
  }

  request = non_unit.value();
  const auto applied = commit(9'700'000'000);
  ASSERT_TRUE(applied) << applied.error().detail;
  request = normalized_equivalent.value();
  const auto replay = commit(9'710'000'000);
  ASSERT_TRUE(replay) << replay.error().detail;
  EXPECT_EQ(replay.value().revision, applied.value().revision);
  EXPECT_EQ(store.snapshot().revision, applied.value().revision);
}

// Card CMB-096 SC-002: conversion does not renormalize arbitrary normalizable quaternions.
// Admission requires finite coefficients and a norm within 1e-6 of 1; a quaternion that could
// be normalized (non-zero norm) but sits outside that band is rejected with the documented
// error before any store identity is considered.
TEST(AttachmentConversionIdentity, NormalizableOutOfBandQuaternionsAreRejectedAtConversion)
{
  constexpr const char * kDocumented =
    "attachment grasp transform is not a finite rigid transform";
  restocker_interfaces::srv::CommitReservedAttachment::Request wire;
  wire.grasp_center_from_held_object.orientation.w = 1.0;
  const auto unit_baseline = attachment_request_from_message(wire);
  ASSERT_TRUE(unit_baseline) << unit_baseline.error().detail;

  // Norm 0.8: obviously normalizable, far outside the band.
  wire.grasp_center_from_held_object.orientation.w = 0.4;
  wire.grasp_center_from_held_object.orientation.x = 0.4;
  wire.grasp_center_from_held_object.orientation.y = 0.4;
  wire.grasp_center_from_held_object.orientation.z = 0.4;
  const auto sub_unit = attachment_request_from_message(wire);
  ASSERT_FALSE(sub_unit);
  EXPECT_EQ(sub_unit.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(sub_unit.error().detail, kDocumented);

  // Barely outside the band: norm 1.001, well normalizable, still rejected.
  wire.grasp_center_from_held_object.orientation.w = 1.001;
  wire.grasp_center_from_held_object.orientation.x = 0.0;
  wire.grasp_center_from_held_object.orientation.y = 0.0;
  wire.grasp_center_from_held_object.orientation.z = 0.0;
  const auto slightly_over_unit = attachment_request_from_message(wire);
  ASSERT_FALSE(slightly_over_unit);
  EXPECT_EQ(slightly_over_unit.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(slightly_over_unit.error().detail, kDocumented);

  // Norm 2: (1,1,1,1) normalizes cleanly in floating point, but admission is the unit band,
  // not normalizability.
  wire.grasp_center_from_held_object.orientation.w = 1.0;
  wire.grasp_center_from_held_object.orientation.x = 1.0;
  wire.grasp_center_from_held_object.orientation.y = 1.0;
  wire.grasp_center_from_held_object.orientation.z = 1.0;
  const auto double_norm = attachment_request_from_message(wire);
  ASSERT_FALSE(double_norm);
  EXPECT_EQ(double_norm.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(double_norm.error().detail, kDocumented);
}

// Card CMB-096 SC-002: invalid wire quaternions stay rejected with the documented conversion
// error. The zero quaternion is rejected by the unit band (|0-1| > 1e-6) before any
// normalization could divide by zero; NaN and infinity are rejected by the finiteness check,
// which must run before the norm comparison because every NaN comparison is false.
TEST(AttachmentConversionIdentity, ZeroNormAndNonFiniteQuaternionsAreRejectedWithDocumentedError)
{
  constexpr const char * kDocumented =
    "attachment grasp transform is not a finite rigid transform";
  restocker_interfaces::srv::CommitReservedAttachment::Request wire;
  wire.grasp_center_from_held_object.orientation.w = 1.0;
  const auto unit_baseline = attachment_request_from_message(wire);
  ASSERT_TRUE(unit_baseline) << unit_baseline.error().detail;

  wire.grasp_center_from_held_object.orientation.w = 0.0;
  wire.grasp_center_from_held_object.orientation.x = 0.0;
  wire.grasp_center_from_held_object.orientation.y = 0.0;
  wire.grasp_center_from_held_object.orientation.z = 0.0;
  const auto zero_norm = attachment_request_from_message(wire);
  ASSERT_FALSE(zero_norm);
  EXPECT_EQ(zero_norm.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(zero_norm.error().detail, kDocumented);

  wire.grasp_center_from_held_object.orientation.x =
    std::numeric_limits<double>::quiet_NaN();
  wire.grasp_center_from_held_object.orientation.w = 1.0;
  const auto not_a_number = attachment_request_from_message(wire);
  ASSERT_FALSE(not_a_number);
  EXPECT_EQ(not_a_number.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(not_a_number.error().detail, kDocumented);

  wire.grasp_center_from_held_object.orientation.x = 0.0;
  wire.grasp_center_from_held_object.orientation.w =
    std::numeric_limits<double>::infinity();
  const auto infinite = attachment_request_from_message(wire);
  ASSERT_FALSE(infinite);
  EXPECT_EQ(infinite.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(infinite.error().detail, kDocumented);
}

TEST_F(AttachmentTemporalAdmission, ExactAndConflictingPayloadsStaySeparateAfterDetachAndRelease)
{
  const auto original_request = request;
  const auto attached = commit(9'700'000'000);
  ASSERT_TRUE(attached);
  const auto detached = store.commit_reserved_detachment(
    {reserved.token, "detach", DetachmentDisposition::ReleaseWithoutMembership,
      at(9'710'000'000)}, at(9'720'000'000));
  ASSERT_TRUE(detached);
  const auto verify_replays = [&](std::int64_t now_ns) {
    const SnapshotMessageOptions options{"world", at(now_ns), true, true};
    const auto current = snapshot_to_message(store.snapshot(), options);
    // The current world can be detached or released; the retained attach receipt stays attached.
    const auto replay = commit(now_ns);
    ASSERT_TRUE(replay);
    EXPECT_EQ(replay.value().revision, attached.value().revision);
    EXPECT_EQ(replay.value().reservation.stage, ReservationStage::Attached);
    EXPECT_FALSE(replay.historical_receipt());
    EXPECT_EQ(snapshot_to_message(store.snapshot(), options), current);
    request.grasp_center_from_held_object.translation().x() = 0.001;
    const auto conflict = commit(now_ns);
    ASSERT_FALSE(conflict);
    EXPECT_EQ(conflict.error().code, WorldStateErrorCode::IdempotencyConflict);
    EXPECT_FALSE(conflict.historical_receipt());
    // Includes revision, active stage, held identity, every object/lane and the complete events.
    EXPECT_EQ(snapshot_to_message(store.snapshot(), options), current);
    request = original_request;
    const auto replay_after_conflict = commit(now_ns);
    ASSERT_TRUE(replay_after_conflict);
    EXPECT_EQ(replay_after_conflict.value().revision, attached.value().revision);
    EXPECT_EQ(replay_after_conflict.value().reservation.stage, ReservationStage::Attached);
    EXPECT_EQ(snapshot_to_message(store.snapshot(), options), current);
  };
  ASSERT_TRUE(store.snapshot().active_reservation);
  EXPECT_EQ(store.snapshot().active_reservation->stage, ReservationStage::Detached);
  EXPECT_FALSE(store.snapshot().robot.held_object);
  verify_replays(9'730'000'000);
  const auto & receipt = detached.value();
  ASSERT_TRUE(
    store.release_task_reservation(
      {receipt.token, "release", receipt.reservation.reservation_id,
        receipt.reservation.stage, receipt.reservation.revision, ReservationOutcome::FailedSafe,
        TaskPhase::Fault, FaultState::Recoverable}, at(9'740'000'000)));
  EXPECT_FALSE(store.snapshot().active_reservation);
  EXPECT_FALSE(store.snapshot().robot.held_object);
  verify_replays(9'750'000'000);
}

TEST_F(AttachmentTemporalAdmission, InvalidTransformAndMismatchedSampleCannotReplayAsSuccess)
{
  ASSERT_TRUE(commit(9'700'000'000));
  const auto clock = sample(9'710'000'000);
  const auto mismatch = store.commit_reserved_attachment(request, at(9'720'000'000), clock);
  ASSERT_FALSE(mismatch);
  EXPECT_EQ(mismatch.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_FALSE(mismatch.historical_receipt());
  request.grasp_center_from_held_object.translation().x() = std::numeric_limits<double>::infinity();
  EXPECT_EQ(commit(9'730'000'000).error().code, WorldStateErrorCode::InvalidArgument);
}

TEST_F(AttachmentTemporalAdmission, DiagnosticCorrelationIsExactBoundedAndImmutable)
{
  ASSERT_TRUE(observe(9'700'000'000, 9'650'000'000));
  request.operation_id = std::string(128, 'x');
  request.operation_id[0] = '\n';
  request.operation_id[1] = '\0';
  const auto pending = commit(9'660'000'000);
  ASSERT_FALSE(pending);
  ASSERT_TRUE(pending.error().attachment_temporal_refusal);
  const auto & captured = *pending.error().attachment_temporal_refusal;
  EXPECT_EQ(captured.decision_kind, WorldStateErrorCode::AttachmentClockNotReady);
  EXPECT_FALSE(captured.clock_inhibited);
  EXPECT_EQ(captured.authority_clock_lineage, captured.commit_clock_lineage);
  EXPECT_EQ(captured.operation_id.original_size, 128U);
  EXPECT_FALSE(captured.operation_id.truncated());
  const auto operation_hex = captured.operation_id.hex_prefix();
  EXPECT_EQ(std::string(operation_hex.data()).size(), 256U);
  EXPECT_EQ(std::string(operation_hex.data()).substr(0, 4), "0a00");
  EXPECT_EQ(captured.source_id.original_size, std::string("temporal-object").size());
  EXPECT_EQ(
    std::string(captured.source_id.hex_prefix().data()),
    "74656d706f72616c2d6f626a656374");
  const auto old_lineage = captured.authority_clock_lineage;
  ASSERT_TRUE(observe(9'750'000'000, 9'690'000'000));
  store.notify_clock_discontinuity();
  EXPECT_FALSE(captured.clock_inhibited);
  EXPECT_EQ(captured.authority_clock_lineage, old_lineage);
  EXPECT_EQ(captured.object_observation_ns, 9'700'000'000);
  EXPECT_EQ(captured.operation_id.hex_prefix(), operation_hex);
  const auto inhibited = commit(9'800'000'000);
  ASSERT_FALSE(inhibited);
  // Early inhibition does not pretend to have reached the temporal-comparison boundary.
  EXPECT_FALSE(inhibited.error().attachment_temporal_refusal);
}

TEST(AttachmentTemporalDiagnostics, SourcePrefixesDeclareTruncationAndAreNotUniqueIdentities)
{
  const std::string prefix(128, 's');
  const auto first = AttachmentDiagnosticIdentity::capture(prefix + "first");
  const auto second = AttachmentDiagnosticIdentity::capture(prefix + "other");
  EXPECT_TRUE(first.truncated());
  EXPECT_EQ(first.original_size, 133U);
  EXPECT_EQ(first.hex_prefix(), second.hex_prefix());
  EXPECT_EQ(std::string(first.hex_prefix().data()).size(), 256U);
  EXPECT_FALSE(AttachmentDiagnosticIdentity::capture(prefix).truncated());
  EXPECT_EQ(std::string(AttachmentDiagnosticIdentity::capture("").hex_prefix().data()), "");
}

TEST(AttachmentTemporalBootstrap, OldSampleCannotCrossActivationIntoEmptyStore)
{
  WorldStateStore store(temporal_config());
  ASSERT_TRUE(
    store.configure_lane(
      LaneDefinition{LaneId{"lane"}, ProductClass::Can, std::nullopt, 0.85}, at(1)));
  const auto previous = store.capture_authority_clock(at(9'000'000'000), false);
  store.notify_clock_discontinuity();
  const auto rejected = store.observe_object(observation(9'000'000'000), previous.time, previous);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::ClockAuthorityInhibited);
  const auto current = store.capture_authority_clock(at(1'000'000'000), true);
  // Static configured lanes did not arm global inhibition; the new lineage may bootstrap.
  ASSERT_TRUE(store.observe_object(observation(1'000'000'000), current.time, current));
}

TEST(AttachmentTemporalBootstrap, PlainSystemTimeAdmissionKeepsExistingContract)
{
  auto config = temporal_config();
  config.clock_type = RCL_SYSTEM_TIME;
  WorldStateStore store(config);
  auto value = observation(9'000'000'000);
  value.observation_time = rclcpp::Time(9'000'000'000, RCL_SYSTEM_TIME);
  const auto clock = store.capture_authority_clock(value.observation_time, true);
  EXPECT_FALSE(clock.managed_ros_started);
  ASSERT_TRUE(store.observe_object(value, clock.time, clock));
}

TEST(AttachmentTemporalWire, InhibitedWithoutHistoryAndReadinessHaveNoSuccessReceipt)
{
  for (const auto code : {WorldStateErrorCode::AttachmentClockNotReady,
      WorldStateErrorCode::ClockAuthorityInhibited})
  {
    const auto result = ReservedAttachmentResult::failure({code, "refused"});
    const auto wire = attachment_result_to_message(result, 42);
    EXPECT_FALSE(wire.has_reservation);
    EXPECT_EQ(wire.world_revision, 42U);
    EXPECT_NE(wire.status.code, Status::OK);
  }
}

}  // namespace
}  // namespace restocker_world_state
