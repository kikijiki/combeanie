// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <string>
#include <vector>
#include <thread>
#include <utility>

#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{
namespace
{
// A representative grasp: product slightly forward of the grasp datum and yawed. The tests
// check that it is recorded and returned unchanged.
[[nodiscard]] Eigen::Isometry3d test_grasp()
{
  Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
  grasp.translation() = Eigen::Vector3d(0.004, -0.002, 0.031);
  grasp.linear() = Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return grasp;
}


TEST(TaskReservationSemantics, ActiveTaskMatrixIsExhaustive)
{
  constexpr std::array phases{
    TaskPhase::Idle, TaskPhase::ValidatingScene, TaskPhase::Executing, TaskPhase::Recovering,
    TaskPhase::Fault, TaskPhase::RequestingOperator};
  constexpr std::array faults{
    FaultState::None, FaultState::Recoverable, FaultState::NonRecoverable,
    FaultState::ExternalInconsistency};

  for (const auto phase : phases) {
    for (const auto fault : faults) {
      const bool expected =
        (phase == TaskPhase::Executing && fault == FaultState::None) ||
        (phase == TaskPhase::Recovering && fault == FaultState::Recoverable) ||
        ((phase == TaskPhase::Fault || phase == TaskPhase::RequestingOperator) &&
        fault != FaultState::None);
      EXPECT_EQ(valid_active_task_semantics(phase, fault), expected)
        << "phase=" << static_cast<unsigned int>(phase)
        << " fault=" << static_cast<unsigned int>(fault);
    }
  }
}

using namespace std::chrono_literals;

[[nodiscard]] rclcpp::Time ros_time(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

[[nodiscard]] ObjectObservation can_observation(std::int64_t timestamp_ns)
{
  ObjectObservation value;
  value.source_object_id = "sim:can_01";
  value.frame_id = "world";
  value.product_class = ProductClass::Can;
  value.sku = "SIM-CAN-STD";
  value.pose_in_world = Eigen::Isometry3d::Identity();
  value.pose_covariance = PoseCovariance::Identity() * 1.0e-6;
  value.orientation = ObjectOrientation::Upright;
  value.observation_time = ros_time(timestamp_ns);
  return value;
}

struct FixtureState
{
  ObjectId object_id;
  ReserveTaskRequest request;
};

// The catalogued can: 0.033 m radius, so one more needs its diameter plus the lane's 0.045 m entry
// clearance of free rear depth, and adds one diameter foreshortened by the 4-degree bed to the
// settled column. Values derive from `load_product_lane_profiles` on the shipped catalog and
// survey.
inline constexpr double kCanPitchM = 2.0 * 0.033 * 0.99756405025982420;
inline constexpr double kCanRearDepthM = 2.0 * 0.033 + 0.045;

[[nodiscard]] WorldStateConfig config(std::size_t journal_capacity = 64)
{
  WorldStateConfig value;
  value.maximum_observation_age = 2s;
  value.lane_evidence_validity = 60s;
  value.maximum_future_skew = 100ms;
  value.event_capacity = 64;
  value.operation_journal_capacity = journal_capacity;
  value.product_lane_profiles = {
    ProductLaneProfile{ProductClass::Can, std::nullopt, kCanRearDepthM, kCanPitchM}};
  return value;
}

[[nodiscard]] FixtureState populate(WorldStateStore & store)
{
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  EXPECT_TRUE(configured);
  if (!configured) {
    return {};
  }
  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  EXPECT_TRUE(evidence);
  if (!evidence) {
    return {};
  }
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
  EXPECT_TRUE(observed);
  if (!observed) {
    return {};
  }
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'550'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  EXPECT_TRUE(telemetry);
  if (!telemetry) {
    return {};
  }
  const auto snapshot = store.snapshot();
  return FixtureState{
    observed.value().object_id,
    ReserveTaskRequest{"task-0001", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      evidence.value().revision}};
}

[[nodiscard]] ReservationReceipt reserve(WorldStateStore & store, const FixtureState & state)
{
  const auto result = store.reserve_task(state.request, ros_time(9'600'000'000));
  EXPECT_TRUE(result) << result.error().detail;
  if (!result) {
    return {};
  }
  return result.value();
}

[[nodiscard]] ExecutionWorldAuthorityExpectation authority_expectation(
  const ReservationReceipt & receipt)
{
  return {
    receipt.reservation.reservation_id, receipt.reservation.revision,
    receipt.reservation.object_id, receipt.reservation.destination_lane};
}

[[nodiscard]] ReleaseReservationRequest release_request(
  const ReservationReceipt & receipt,
  std::string operation_id,
  ReservationOutcome outcome,
  TaskPhase task_phase,
  FaultState fault_state)
{
  return {receipt.token,
    std::move(operation_id),
    receipt.reservation.reservation_id,
    receipt.reservation.stage,
    receipt.reservation.revision,
    outcome,
    task_phase,
    fault_state};
}

TEST(WorldStateReservations, RejectsZeroJournalCapacity)
{
  auto invalid_config = config();
  invalid_config.operation_journal_capacity = 0;
  EXPECT_THROW(WorldStateStore store(invalid_config), std::invalid_argument);
}

TEST(WorldStateReservations, RejectsInvalidReservedPoseDivergenceBound)
{
  auto negative = config();
  negative.reserved_pose_divergence_bound_m = -0.001;
  EXPECT_THROW(WorldStateStore store(negative), std::invalid_argument);
  auto not_a_number = config();
  not_a_number.reserved_pose_divergence_bound_m =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(WorldStateStore store(not_a_number), std::invalid_argument);
}

// The archived old-core witness failed with OutOfOrder. This repaired version supplies managed
// clock provenance and requires the exact typed refusal, without accepting the early commit.
TEST(WorldStateReservations, FutureObservationNeedsDistinctAttachmentReadiness)
{
  WorldStateStore store(config());
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  ASSERT_TRUE(configured) << configured.error().detail;
  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
    ros_time(9'400'000'000), configured.value().revision);
  ASSERT_TRUE(evidence) << evidence.error().detail;
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(9'500'000'000));
  ASSERT_TRUE(observed) << observed.error().detail;
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'550'000'000), {}, {}, "test/temporal-admission"},
    ros_time(9'550'000'000));
  ASSERT_TRUE(telemetry) << telemetry.error().detail;

  const auto selected = store.snapshot();
  const ObjectId object_id = observed.value().object_id;
  const auto reserved = store.reserve_task(
    ReserveTaskRequest{"temporal-witness", selected.revision, object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      evidence.value().revision},
    ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;

  const auto future_observation =
    store.observe_object(
    can_observation(9'700'000'000), ros_time(9'650'000'000),
    store.capture_authority_clock(ros_time(9'650'000'000), true));
  ASSERT_TRUE(future_observation) << future_observation.error().detail;
  const auto before_commit = store.snapshot();
  const auto & object = before_commit.objects.at(object_id);
  EXPECT_EQ(object.observation_time.nanoseconds(), 9'700'000'000);
  EXPECT_EQ(object.transition_time.nanoseconds(), 9'700'000'000);
  EXPECT_TRUE(object.pose_in_world.isApprox(selected.objects.at(object_id).pose_in_world));
  EXPECT_GT(object.revision, observed.value().revision);
  EXPECT_LT(before_commit.robot.telemetry_time.nanoseconds(), 9'660'000'000);

  const ReservedAttachmentRequest request{
    reserved.value().token, "temporal-witness-attach", test_grasp()};
  const auto premature = store.commit_reserved_attachment(
    request, ros_time(9'660'000'000),
    store.capture_authority_clock(ros_time(9'660'000'000), true));
  ASSERT_FALSE(premature);
  EXPECT_EQ(premature.error().code, WorldStateErrorCode::AttachmentClockNotReady);
  const auto refused = store.snapshot();
  EXPECT_EQ(refused.revision, before_commit.revision);
  ASSERT_TRUE(refused.active_reservation);
  EXPECT_EQ(refused.active_reservation->stage, ReservationStage::Reserved);
  EXPECT_EQ(refused.objects.at(object_id).grasp_state, GraspState::Free);
  EXPECT_FALSE(refused.robot.held_object);

  const auto matured = store.commit_reserved_attachment(
    request, ros_time(9'700'000'000),
    store.capture_authority_clock(ros_time(9'700'000'000), true));
  ASSERT_TRUE(matured) << matured.error().detail;
  EXPECT_EQ(matured.value().reservation.stage, ReservationStage::Attached);
  const auto replay = store.commit_reserved_attachment(
    request, ros_time(9'710'000'000),
    store.capture_authority_clock(ros_time(9'710'000'000), true));
  ASSERT_TRUE(replay) << replay.error().detail;
  EXPECT_EQ(replay.value().revision, matured.value().revision);
  EXPECT_EQ(store.snapshot().robot.held_object, object_id);
}

TEST(WorldStateReservations, CapturesAttachmentTemporalRefusalWithoutChangingItsDecision)
{
  struct Case
  {
    std::int64_t observation_ns;
    std::int64_t commit_ns;
    bool object_postdates;
    bool robot_postdates;
  };
  // Deliberately old mutation arguments exercise both sides of the existing strict comparison.
  // This is a diagnostic-copy test, not evidence that a serialized runtime clock regressed.
  const std::array cases{
    Case{9'700'000'000, 9'660'000'000, true, false},
    Case{9'500'000'000, 9'540'000'000, false, true},
    Case{9'700'000'000, 9'540'000'000, true, true}};
  for (const auto & value : cases) {
    WorldStateStore store(config());
    const FixtureState state = populate(store);
    const ReservationReceipt reserved = reserve(store, state);
    ASSERT_FALSE(reserved.token.empty());
    if (value.observation_ns > 9'500'000'000) {
      ASSERT_TRUE(
        store.observe_object(can_observation(value.observation_ns), ros_time(9'650'000'000)));
    }
    const auto before = store.snapshot();
    const auto refused = store.commit_reserved_attachment(
      {reserved.token, "diagnostic-attach", test_grasp()}, ros_time(value.commit_ns));
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().code, WorldStateErrorCode::OutOfOrder);
    EXPECT_EQ(refused.error().detail, "attachment predates object or robot state");
    ASSERT_TRUE(refused.error().attachment_temporal_refusal);
    const auto & captured = *refused.error().attachment_temporal_refusal;
    EXPECT_EQ(captured.committed_at_ns, value.commit_ns);
    EXPECT_EQ(captured.object_observation_ns, value.observation_ns);
    EXPECT_EQ(captured.object_transition_ns, value.observation_ns);
    EXPECT_EQ(captured.robot_telemetry_ns, before.robot.telemetry_time.nanoseconds());
    EXPECT_EQ(captured.world_revision, before.revision);
    EXPECT_EQ(captured.object_id, state.object_id);
    EXPECT_EQ(captured.object_revision, before.objects.at(state.object_id).revision);
    EXPECT_EQ(captured.robot_revision, before.robot.revision);
    EXPECT_EQ(captured.robot_telemetry_revision, before.robot.telemetry_revision);
    EXPECT_EQ(captured.reservation_id, reserved.reservation.reservation_id);
    EXPECT_EQ(captured.reservation_revision, reserved.reservation.revision);
    EXPECT_EQ(captured.reservation_stage, static_cast<std::uint8_t>(ReservationStage::Reserved));
    EXPECT_EQ(captured.object_postdates_commit, value.object_postdates);
    EXPECT_EQ(captured.robot_postdates_commit, value.robot_postdates);
    EXPECT_EQ(captured.decision_kind, WorldStateErrorCode::OutOfOrder);
    EXPECT_FALSE(captured.clock_inhibited);
    EXPECT_EQ(
      std::string(captured.operation_id.hex_prefix().data()),
      "646961676e6f737469632d617474616368");
    EXPECT_EQ(std::string(captured.source_id.hex_prefix().data()), "73696d3a63616e5f3031");
    EXPECT_EQ(store.snapshot().revision, before.revision);
    EXPECT_FALSE(store.snapshot().robot.held_object);

    // The retained error describes the original refusal even after fresh evidence changes state.
    ASSERT_TRUE(store.observe_object(can_observation(9'800'000'000), ros_time(9'800'000'000)));
    EXPECT_GT(store.snapshot().revision, captured.world_revision);
    EXPECT_EQ(captured.object_observation_ns, value.observation_ns);
    EXPECT_EQ(captured.object_revision, before.objects.at(state.object_id).revision);
    const auto wrong_token = store.commit_reserved_attachment(
      {"wrong-token", "diagnostic-wrong-token", test_grasp()}, ros_time(9'900'000'000));
    ASSERT_FALSE(wrong_token);
    EXPECT_FALSE(wrong_token.error().attachment_temporal_refusal);
  }
}

// Card 029 SC-001(a): while a reservation is at stage Reserved, confirm-frame jitter inside the
// divergence bound must not move the reserved product's pose — that is the pose the grasp is
// staged from and the planning scene projects — while freshness, revision and events keep
// advancing. Pre-fix every one of these observations replaced the pose, which is the reproduced
// defect behind SC-002's 43-47 % approach refusals. Unrelated objects are unaffected.
TEST(WorldStateReservations, HoldsReservedPoseAgainstConfirmJitterWhileFreshnessAdvances)
{
  WorldStateStore store(config());
  auto neighbour = can_observation(9'450'000'000);
  neighbour.source_object_id = "sim:can_02";
  neighbour.pose_in_world.translation().y() = 0.2;
  const auto neighbour_receipt = store.observe_object(neighbour, ros_time(10'000'000'000));
  ASSERT_TRUE(neighbour_receipt) << neighbour_receipt.error().detail;

  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_FALSE(reserved.token.empty());
  const Eigen::Vector3d pinned =
    store.snapshot().objects.at(state.object_id).pose_in_world.translation();

  std::int64_t previous_stamp = 9'500'000'000;
  Revision previous_object_revision =
    store.snapshot().objects.at(state.object_id).revision;
  // Offsets all within the 0.020 m bound, each well past the projector's 0.5 mm deadband — the
  // exact shape that used to rewrite restocker/object/1 every cycle under the frozen grasp.
  const std::vector<Eigen::Vector3d> offsets{
    {0.004, -0.003, 0.0}, {0.012, 0.005, 0.0}, {-0.009, -0.011, 0.0}, {0.006, 0.010, 0.0}};
  std::size_t index = 0;
  for (const auto & offset : offsets) {
    ASSERT_LE(offset.norm(), 0.020) << index;
    const std::int64_t stamp = previous_stamp + 40'000'000;
    auto jittered = can_observation(stamp);
    jittered.pose_in_world.translation() = pinned + offset;
    const auto admitted = store.observe_object(jittered, ros_time(10'000'000'000));
    ASSERT_TRUE(admitted) << admitted.error().detail;
    const auto snapshot = store.snapshot();
    const auto & object = snapshot.objects.at(state.object_id);
    EXPECT_TRUE(object.pose_in_world.translation().isApprox(pinned, 1.0e-12))
      << "frame " << index << " moved the held pose";
    EXPECT_EQ(object.observation_time.nanoseconds(), stamp);
    EXPECT_GT(object.revision, previous_object_revision);
    // Freshness the projector and the reservation boundaries read still advances.
    const auto validation = store.validate_task_reservation(reserved.token);
    ASSERT_TRUE(validation) << validation.error().detail;
    previous_stamp = stamp;
    previous_object_revision = object.revision;
    ++index;
  }
  const auto final_snapshot = store.snapshot();
  ASSERT_FALSE(final_snapshot.events.empty());
  EXPECT_EQ(final_snapshot.events.back().kind, EventKind::ObjectObserved);
  EXPECT_EQ(
    final_snapshot.events.back().detail,
    "object re-observed; pose held under active reservation");

  // The hold is scoped to the reserved object: an unrelated product keeps moving freely.
  auto neighbour_move = can_observation(previous_stamp + 40'000'000);
  neighbour_move.source_object_id = "sim:can_02";
  neighbour_move.pose_in_world.translation().y() = 0.204;
  const auto moved_neighbour =
    store.observe_object(neighbour_move, ros_time(10'000'000'000));
  ASSERT_TRUE(moved_neighbour) << moved_neighbour.error().detail;
  EXPECT_NEAR(
    store.snapshot().objects.at(
      neighbour_receipt.value().object_id).pose_in_world.translation().y(),
    0.204, 1.0e-12);
}

// Card 088 stage 1 (CMB-SPEC-11 I-2): the newest admitted measurement is kept as one unit —
// pose, covariance, orientation and the stamp that describes them — even when the reservation hold
// retains the act-on fields. The act-on view must stay exactly the held reference, and the
// measured unit must never be mistaken for it (nor the reverse).
TEST(WorldStateReservations, KeepsLatestMeasuredUnitDistinctFromHeldActOnPoseUnderReservation)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);

  // Free object: measured and act-on views coincide on the first admission.
  {
    const auto & object = store.snapshot().objects.at(state.object_id);
    EXPECT_TRUE(object.latest_measured.pose.isApprox(object.pose_in_world, 1.0e-12));
    EXPECT_TRUE(object.latest_measured.covariance.isApprox(object.pose_covariance, 1.0e-12));
    EXPECT_EQ(object.latest_measured.orientation, object.orientation);
    EXPECT_EQ(object.latest_measured.observation_time.nanoseconds(), 9'500'000'000);
    EXPECT_EQ(
      object.latest_measured.observation_time.nanoseconds(),
      object.observation_time.nanoseconds());
  }

  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_FALSE(reserved.token.empty());
  const auto held = store.snapshot().objects.at(state.object_id);
  // Reserving is not an observation: the measured unit is untouched.
  EXPECT_EQ(held.latest_measured.observation_time.nanoseconds(), 9'500'000'000);
  EXPECT_TRUE(held.latest_measured.pose.isApprox(held.pose_in_world, 1.0e-12));

  // A within-bound observation with a different pose, covariance and orientation.
  auto jittered = can_observation(9'540'000'000);
  jittered.pose_in_world.translation() = held.pose_in_world.translation() +
    Eigen::Vector3d(0.007, -0.004, 0.001);
  jittered.pose_covariance = PoseCovariance::Identity() * 4.0e-6;
  jittered.orientation = ObjectOrientation::Tilted;
  ASSERT_LE(
    (jittered.pose_in_world.translation() - held.pose_in_world.translation()).norm(),
    0.020);
  const auto admitted = store.observe_object(jittered, ros_time(10'000'000'000));
  ASSERT_TRUE(admitted) << admitted.error().detail;

  const auto after = store.snapshot().objects.at(state.object_id);
  // Zero behavior delta: act-on fields held, stamps and revision advanced as before.
  EXPECT_TRUE(after.pose_in_world.isApprox(held.pose_in_world, 1.0e-12));
  EXPECT_TRUE(after.pose_covariance.isApprox(held.pose_covariance, 1.0e-12));
  EXPECT_EQ(after.orientation, held.orientation);
  EXPECT_EQ(after.observation_time.nanoseconds(), 9'540'000'000);
  EXPECT_EQ(after.transition_time.nanoseconds(), 9'540'000'000);
  EXPECT_GT(after.revision, held.revision);
  // The measured unit is exactly the admitted observation, as one paired unit.
  EXPECT_TRUE(after.latest_measured.pose.isApprox(jittered.pose_in_world, 1.0e-12));
  EXPECT_TRUE(after.latest_measured.covariance.isApprox(jittered.pose_covariance, 1.0e-12));
  EXPECT_EQ(after.latest_measured.orientation, ObjectOrientation::Tilted);
  EXPECT_EQ(after.latest_measured.observation_time.nanoseconds(), 9'540'000'000);
  // And it is not the act-on view: conflating the two fails these.
  EXPECT_FALSE(after.latest_measured.pose.isApprox(after.pose_in_world, 1.0e-9));
  EXPECT_FALSE(after.latest_measured.covariance.isApprox(after.pose_covariance, 1.0e-9));
  EXPECT_NE(after.latest_measured.orientation, after.orientation);

  // A later held observation supersedes the previous measured unit, never the act-on view.
  auto later = can_observation(9'580'000'000);
  later.pose_in_world.translation() = held.pose_in_world.translation() +
    Eigen::Vector3d(-0.002, 0.009, 0.0);
  ASSERT_TRUE(store.observe_object(later, ros_time(10'000'000'000)));
  const auto latest = store.snapshot().objects.at(state.object_id);
  EXPECT_TRUE(latest.pose_in_world.isApprox(held.pose_in_world, 1.0e-12));
  EXPECT_TRUE(latest.latest_measured.pose.isApprox(later.pose_in_world, 1.0e-12));
  EXPECT_EQ(latest.latest_measured.orientation, ObjectOrientation::Upright);
  EXPECT_EQ(latest.latest_measured.observation_time.nanoseconds(), 9'580'000'000);
}

// Card 088 stage 1: past the divergence bound the act-on view is re-based onto the observation
// and the measured unit holds the same observation; the two agree again.
TEST(WorldStateReservations, LatestMeasuredMatchesActOnPoseWhenReservedObservationDiverges)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  ASSERT_FALSE(reserve(store, state).token.empty());
  const Eigen::Vector3d pinned =
    store.snapshot().objects.at(state.object_id).pose_in_world.translation();

  auto knocked = can_observation(9'620'000'000);
  knocked.pose_in_world.translation() = pinned + Eigen::Vector3d(0.05, 0.0, 0.0);
  knocked.pose_covariance = PoseCovariance::Identity() * 9.0e-6;
  knocked.orientation = ObjectOrientation::Horizontal;
  ASSERT_TRUE(store.observe_object(knocked, ros_time(10'000'000'000)));
  const auto object = store.snapshot().objects.at(state.object_id);
  EXPECT_TRUE(object.pose_in_world.isApprox(knocked.pose_in_world, 1.0e-12));
  EXPECT_TRUE(object.latest_measured.pose.isApprox(knocked.pose_in_world, 1.0e-12));
  EXPECT_TRUE(object.latest_measured.covariance.isApprox(knocked.pose_covariance, 1.0e-12));
  EXPECT_EQ(object.latest_measured.orientation, ObjectOrientation::Horizontal);
  EXPECT_EQ(object.orientation, ObjectOrientation::Horizontal);
  EXPECT_EQ(object.latest_measured.observation_time.nanoseconds(), 9'620'000'000);
}

// Card 029 SC-001(b): past the divergence bound the observation's pose is applied and becomes
// the new held pose — a real movement is surfaced, never hidden — and the hold re-latches there.
TEST(WorldStateReservations, AppliesReservedPoseBeyondDivergenceBoundAndReLatches)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_FALSE(reserved.token.empty());
  const Eigen::Vector3d pinned =
    store.snapshot().objects.at(state.object_id).pose_in_world.translation();

  auto knocked = can_observation(9'620'000'000);
  knocked.pose_in_world.translation() = pinned + Eigen::Vector3d(0.05, 0.0, 0.0);
  const auto applied = store.observe_object(knocked, ros_time(10'000'000'000));
  ASSERT_TRUE(applied) << applied.error().detail;
  auto snapshot = store.snapshot();
  const auto & displaced = snapshot.objects.at(state.object_id);
  EXPECT_TRUE(
    displaced.pose_in_world.translation().isApprox(pinned + Eigen::Vector3d(0.05, 0.0, 0.0)));
  ASSERT_FALSE(snapshot.events.empty());
  EXPECT_NE(
    snapshot.events.back().detail.find(
      "object re-observed; pose applied beyond reserved divergence bound (distance"),
    std::string::npos) << snapshot.events.back().detail;
  EXPECT_NE(
    snapshot.events.back().detail.find("m)"), std::string::npos) << snapshot.events.back().detail;
  // Identity predicates are untouched, so the reservation itself still validates: the scene now
  // shows the truth and the existing plan fence / collision / attach gates do the failing closed.
  EXPECT_TRUE(store.validate_task_reservation(reserved.token));

  // Re-latch: the next confirm frame near the NEW pose holds there instead of dancing on.
  const Eigen::Vector3d relatched =
    store.snapshot().objects.at(state.object_id).pose_in_world.translation();
  auto nearby = can_observation(9'660'000'000);
  nearby.pose_in_world.translation() = relatched + Eigen::Vector3d(0.004, -0.003, 0.0);
  const auto held_again = store.observe_object(nearby, ros_time(10'000'000'000));
  ASSERT_TRUE(held_again) << held_again.error().detail;
  EXPECT_TRUE(
    store.snapshot().objects.at(state.object_id).pose_in_world.translation().isApprox(
      relatched, 1.0e-12));
}

// Card 029 SC-001(c): the hold is scoped to the reservation's pre-attach window. From the
// Attached stage on, and again after any release, ordinary pose admission resumes.
TEST(WorldStateReservations, ReservedPoseHoldEndsAtAttachAndAtRelease)
{
  {
    WorldStateStore store(config());
    const FixtureState state = populate(store);
    const ReservationReceipt reserved = reserve(store, state);
    ASSERT_FALSE(reserved.token.empty());
    ASSERT_TRUE(
      store.commit_reserved_attachment(
        ReservedAttachmentRequest{reserved.token, "attach-hold-exit"}, ros_time(9'650'000'000)))
      << "attach must succeed to reach the Attached stage";
    auto carried = can_observation(9'700'000'000);
    carried.pose_in_world.translation().x() = 0.004;
    const auto admitted = store.observe_object(carried, ros_time(10'000'000'000));
    ASSERT_TRUE(admitted) << admitted.error().detail;
    EXPECT_NEAR(
      store.snapshot().objects.at(state.object_id).pose_in_world.translation().x(),
      0.004, 1.0e-12);
  }
  {
    WorldStateStore store(config());
    const FixtureState state = populate(store);
    const ReservationReceipt reserved = reserve(store, state);
    ASSERT_FALSE(reserved.token.empty());
    const auto release = release_request(
      reserved, "release-hold-exit", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::Recoverable);
    ASSERT_TRUE(store.release_task_reservation(release, ros_time(9'650'000'000)));
    auto after_release = can_observation(9'700'000'000);
    after_release.pose_in_world.translation().x() = 0.004;
    const auto admitted = store.observe_object(after_release, ros_time(10'000'000'000));
    ASSERT_TRUE(admitted) << admitted.error().detail;
    EXPECT_NEAR(
      store.snapshot().objects.at(state.object_id).pose_in_world.translation().x(),
      0.004, 1.0e-12);
  }
}

TEST(WorldStateReservations, RequiresRealLaneEvidenceAndExactEntityRevisions)
{
  WorldStateStore store(config());
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  ASSERT_TRUE(configured);
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'525'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(telemetry);
  const auto snapshot = store.snapshot();
  ReserveTaskRequest request{"task-no-evidence",
    snapshot.revision,
    observed.value().object_id,
    observed.value().revision,
    std::nullopt,
    std::nullopt,
    LaneId{"lane_01"},
    configured.value().revision};
  const auto no_evidence = store.reserve_task(request, ros_time(9'600'000'000));
  ASSERT_FALSE(no_evidence);
  EXPECT_EQ(no_evidence.error().code, WorldStateErrorCode::PredicateFailed);

  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'550'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  ASSERT_TRUE(evidence);
  request.request_id = "task-stale";
  request.selected_snapshot_revision = evidence.value().revision;
  request.destination_lane_revision = configured.value().revision;
  const auto stale = store.reserve_task(request, ros_time(9'600'000'000));
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::RevisionConflict);
}

TEST(WorldStateReservations, RejectsUnobservedRobotTelemetryWithoutMutation)
{
  WorldStateStore store(config());
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  ASSERT_TRUE(configured);
  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  ASSERT_TRUE(evidence);
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  const auto snapshot = store.snapshot();
  ASSERT_EQ(snapshot.robot.revision, 0U);

  const auto result = store.reserve_task(
    ReserveTaskRequest{"task-unobserved-robot", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      evidence.value().revision},
    ros_time(9'600'000'000));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_EQ(store.snapshot().revision, snapshot.revision);
  EXPECT_FALSE(store.snapshot().active_reservation);
}

TEST(WorldStateReservations, BindsAdmissionToLatestContinuousTelemetry)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const Revision selected_robot_revision = store.snapshot().robot.revision;

  const auto newer = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'575'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(newer) << newer.error().detail;
  ASSERT_GT(newer.value().revision, selected_robot_revision);

  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  EXPECT_EQ(
    reserved.value().reservation.admitted_robot_telemetry_revision,
    newer.value().revision);
}

TEST(WorldStateReservations, RejectsStaleRobotTelemetryAtTransactionalBoundary)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto before = store.snapshot();

  const auto stale = store.reserve_task(state.request, ros_time(11'550'000'001));
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_EQ(store.snapshot().revision, before.revision);
  EXPECT_FALSE(store.snapshot().active_reservation);
}

TEST(WorldStateReservations, RevalidatesTaskEvidenceFreshnessAtTransactionalBoundary)
{
  WorldStateStore stale_object_store(config());
  FixtureState stale_object_state = populate(stale_object_store);
  const auto destination = stale_object_store.snapshot().lanes.at(LaneId{"lane_01"});
  const auto fresh_lane = stale_object_store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(11'550'000'000)},
    ros_time(11'600'000'000), destination.revision);
  ASSERT_TRUE(fresh_lane) << fresh_lane.error().detail;
  const auto fresh_robot = stale_object_store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(11'575'000'000), {}, {}, "test/reservations"},
    ros_time(11'600'000'000));
  ASSERT_TRUE(fresh_robot) << fresh_robot.error().detail;
  stale_object_state.request.request_id = "task-stale-object";
  stale_object_state.request.selected_snapshot_revision = fresh_robot.value().revision;
  stale_object_state.request.destination_lane_revision = fresh_lane.value().revision;
  const auto stale_object = stale_object_store.reserve_task(
    stale_object_state.request, ros_time(11'600'000'001));
  ASSERT_FALSE(stale_object);
  EXPECT_EQ(stale_object.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_FALSE(stale_object_store.snapshot().active_reservation);

  WorldStateStore stale_lane_store(config());
  FixtureState stale_lane_state = populate(stale_lane_store);
  // Lane evidence from populate is at 9.4 s. Refresh object and robot near the reservation
  // instant 61 s later so only the lane's validity horizon refuses.
  constexpr std::int64_t kReserveNs = 9'400'000'000LL + 61'000'000'000LL;
  const auto fresh_object = stale_lane_store.observe_object(
    can_observation(kReserveNs - 50'000'000), ros_time(kReserveNs));
  ASSERT_TRUE(fresh_object) << fresh_object.error().detail;
  const auto newer_robot = stale_lane_store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(kReserveNs - 25'000'000), {}, {}, "test/reservations"},
    ros_time(kReserveNs));
  ASSERT_TRUE(newer_robot) << newer_robot.error().detail;
  stale_lane_state.request.request_id = "task-stale-lane";
  stale_lane_state.request.selected_snapshot_revision = newer_robot.value().revision;
  stale_lane_state.request.object_revision = fresh_object.value().revision;
  const auto stale_lane = stale_lane_store.reserve_task(
    stale_lane_state.request, ros_time(kReserveNs));
  ASSERT_FALSE(stale_lane);
  EXPECT_EQ(stale_lane.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_NE(stale_lane.error().detail.find("validity horizon"), std::string::npos)
    << stale_lane.error().detail;
  EXPECT_FALSE(stale_lane_store.snapshot().active_reservation);

  WorldStateStore boundary_store(config());
  const FixtureState boundary_state = populate(boundary_store);
  const auto boundary = boundary_store.reserve_task(
    boundary_state.request, ros_time(11'400'000'000));
  ASSERT_TRUE(boundary) << boundary.error().detail;
}

TEST(WorldStateReservations, LinearizesTelemetryAcrossReservationAndAttachment)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_FALSE(reserved.token.empty());

  const auto delayed_pre_reservation = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.1, 0.0,
      ros_time(9'575'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(delayed_pre_reservation) << delayed_pre_reservation.error().detail;
  EXPECT_GT(store.snapshot().revision, reserved.revision);

  const auto active_telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.1, 0.0,
      ros_time(9'650'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(active_telemetry) << active_telemetry.error().detail;
  auto snapshot = store.snapshot();
  ASSERT_TRUE(snapshot.active_reservation);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Executing);
  EXPECT_EQ(snapshot.robot.fault_state, FaultState::None);
  EXPECT_FALSE(snapshot.robot.held_object);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-linearization"}, ros_time(9'700'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  const auto attached_telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.2, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.2, 0.0,
      ros_time(9'750'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(attached_telemetry) << attached_telemetry.error().detail;
  snapshot = store.snapshot();
  ASSERT_TRUE(snapshot.active_reservation);
  EXPECT_EQ(snapshot.active_reservation->stage, ReservationStage::Attached);
  EXPECT_EQ(snapshot.robot.held_object, state.object_id);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Executing);
  EXPECT_EQ(snapshot.objects.at(state.object_id).grasp_state, GraspState::Attached);
}

// Seeds `lane_01` with a product packed against the front rail and returns the reservation request
// for a second one. Every case below asks whether the depth left behind the column fits another.
[[nodiscard]] FixtureState populate_with_occupant(
  WorldStateStore & store, double available_depth_m)
{
  const auto configured = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  EXPECT_TRUE(configured);
  if (!configured) {
    return {};
  }
  const auto evidence = store.update_lane(
    LaneObservation{
        LaneId{"lane_01"}, {"sim:already_here"}, available_depth_m, false,
        ros_time(9'400'000'000)},
    ros_time(10'000'000'000), configured.value().revision);
  EXPECT_TRUE(evidence);
  if (!evidence) {
    return {};
  }
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
  EXPECT_TRUE(observed);
  if (!observed) {
    return {};
  }
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'550'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  EXPECT_TRUE(telemetry);
  if (!telemetry) {
    return {};
  }
  const auto snapshot = store.snapshot();
  return FixtureState{
    observed.value().object_id,
    ReserveTaskRequest{"task-0001", snapshot.revision, observed.value().object_id,
      observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
      evidence.value().revision}};
}

// The destination must have room at its rear for one more of this product (not merely be empty).
TEST(WorldStateReservations, AdmitsASecondProductBehindTheFirstAndRefusesOneThatWillNotFit)
{
  {
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
    ASSERT_TRUE(reserved) << reserved.error().detail;
    // The reservation records what it was granted against, as the placement proof's baseline.
    EXPECT_DOUBLE_EQ(reserved.value().reservation.destination_available_depth_m, 0.734);
    EXPECT_EQ(
      reserved.value().reservation.destination_source_ids,
      std::vector<std::string>{"sim:already_here"});
  }
  {
    // A shortfall beyond the bounded adaptive-release tolerance still refuses.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, kCanRearDepthM - 0.011);
    const auto refused = store.reserve_task(state.request, ros_time(9'600'000'000));
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
    EXPECT_NE(refused.error().detail.find("no room at its rear"), std::string::npos);
  }
  {
    // The final semantic slot may reduce the preferred entry clearance by the measured 10 mm
    // tolerance; selection and placement keep the full product envelope and margins.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, kCanRearDepthM - 0.009);
    EXPECT_TRUE(store.reserve_task(state.request, ros_time(9'600'000'000)));
  }
  {
    // Exactly enough depth is accepted.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, kCanRearDepthM);
    EXPECT_TRUE(store.reserve_task(state.request, ros_time(9'600'000'000)));
  }
  {
    // A lane two cans deep takes a third on the same predicate; only remaining depth decides.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734 - kCanPitchM);
    EXPECT_TRUE(store.reserve_task(state.request, ros_time(9'600'000'000)));
  }
}

// Carries a reservation from grant through attach, then commits the placement against the
// destination evidence the caller supplies. Returns the commit result.
[[nodiscard]] Result<ReservationReceipt> place_against_evidence(
  WorldStateStore & store, const FixtureState & state,
  const std::vector<std::string> & observed, double available_depth_m,
  std::int64_t evidence_ns, std::int64_t released_ns)
{
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  EXPECT_TRUE(reserved) << reserved.error().detail;
  if (!reserved) {
    return Result<ReservationReceipt>::failure(reserved.error());
  }
  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.value().token, "attach-0001"}, ros_time(9'800'000'000));
  EXPECT_TRUE(attached) << attached.error().detail;
  if (!attached) {
    return Result<ReservationReceipt>::failure(attached.error());
  }
  const auto snapshot = store.snapshot();
  const auto evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, observed, available_depth_m, false, ros_time(evidence_ns)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  EXPECT_TRUE(evidence) << evidence.error().detail;
  if (!evidence) {
    return Result<ReservationReceipt>::failure(evidence.error());
  }
  return store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "detach-0001",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(released_ns)},
    ros_time(9'900'000'000));
}

// The lane takes a third product on the same terms as the second, each proven by the same pitch.
// A loop, since nothing counts: the predicate reads depth only.
TEST(WorldStateReservations, TakesAThirdProductAndProvesEachOneByThePitch)
{
  double available = 0.85;
  std::vector<std::string> occupants;
  for (int placement = 0; placement < 3; ++placement) {
    WorldStateStore store(config());
    const auto configured = store.configure_lane(
      LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
      ros_time(9'000'000'000));
    ASSERT_TRUE(configured);
    std::vector<std::string> sorted = occupants;
    std::sort(sorted.begin(), sorted.end());
    const auto evidence = store.update_lane(
      LaneObservation{LaneId{"lane_01"}, sorted, available, false, ros_time(9'400'000'000)},
      ros_time(10'000'000'000), configured.value().revision);
    ASSERT_TRUE(evidence) << placement;
    const auto observed =
      store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
    ASSERT_TRUE(observed) << placement;
    ASSERT_TRUE(
      store.observe_robot_telemetry(
        RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
          ros_time(9'550'000'000), {}, {}, "test/reservations"},
        ros_time(10'000'000'000))) << placement;
    const auto snapshot = store.snapshot();
    const FixtureState state{
      observed.value().object_id,
      ReserveTaskRequest{"task-0001", snapshot.revision, observed.value().object_id,
        observed.value().revision, std::nullopt, std::nullopt, LaneId{"lane_01"},
        evidence.value().revision}};
    sorted.push_back("sim:can_01");
    std::sort(sorted.begin(), sorted.end());
    const auto placed = place_against_evidence(
      store, state, sorted, available - kCanPitchM, 9'850'000'000, 9'840'000'000);
    ASSERT_TRUE(placed) << placement << ": " << placed.error().detail;
    occupants.push_back("sim:already_here_" + std::to_string(placement));
    available -= kCanPitchM;
  }
  // Three pitches are gone and the lane is far from full: the eleventh can is the first refused.
  EXPECT_GT(available, kCanRearDepthM);
}

// Placement proof for multi-product lanes: the column grew by this product's own pitch since the
// grant, and the evidence carrying that growth was recorded strictly after the release.
TEST(WorldStateReservations, ProvesAPlacementByTheColumnGrowingByOneProductPitch)
{
  {
    // The lane names the target, the evidence post-dates the release, but the depth has not
    // moved: nothing was added, so nothing is proven.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto refused = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01"}, 0.734, 9'850'000'000, 9'840'000'000);
    ASSERT_FALSE(refused);
    EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
    EXPECT_NE(refused.error().detail.find("grown by one product pitch"), std::string::npos);
  }
  {
    // The simulator's cylinder narrowphase cannot measure column growth. Its explicit opt-in
    // still requires the exact target identity in fresh evidence recorded after release.
    WorldStateConfig identity_only = config();
    identity_only.placement_require_column_growth = false;
    WorldStateStore store(identity_only);
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto placed = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01"}, 0.734, 9'850'000'000,
      9'840'000'000);
    ASSERT_TRUE(placed) << placed.error().detail;
  }
  {
    // A settled second can: the column is one pitch longer than it was at the reservation.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto placed = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01"}, 0.734 - kCanPitchM,
      9'850'000'000, 9'840'000'000);
    ASSERT_TRUE(placed) << placed.error().detail;
    EXPECT_EQ(
      store.snapshot().lanes.at(LaneId{"lane_01"}).contents,
      std::vector<ObjectId>{state.object_id});
  }
  {
    // Mid-roll: the product left the jaws 2 s ago and is still rolling, so the lane reports less
    // free depth than it will settle at. That overstates the column (the safe direction), so the
    // placement is proven.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto rolling = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01"}, 0.045, 9'850'000'000, 9'840'000'000);
    ASSERT_TRUE(rolling) << rolling.error().detail;
  }
  {
    // The hover: mid-roll evidence with the held product inside the lane volume, but the jaws have
    // not opened, so the evidence predates the release and proves nothing.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto hovered = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01"}, 0.045, 9'840'000'000, 9'850'000'000);
    ASSERT_FALSE(hovered);
    EXPECT_EQ(hovered.error().code, WorldStateErrorCode::PredicateFailed);
    EXPECT_NE(hovered.error().detail.find("after the release"), std::string::npos);
  }
  {
    // §3's camera proof: with growth required (the default) the growth IS the proof, so
    // unnamed post-release evidence that shows the depth shrink commits. The wrist lane
    // producer never names identities in a lane; requiring the name made sensor-driven
    // placements structurally uncommittable (Card 029 attempt 5).
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto placed = place_against_evidence(
      store, state, {"sim:already_here"}, 0.734 - kCanPitchM, 9'850'000'000, 9'840'000'000);
    ASSERT_TRUE(placed) << placed.error().detail;
    EXPECT_EQ(
      store.snapshot().lanes.at(LaneId{"lane_01"}).contents,
      std::vector<ObjectId>{state.object_id});
  }
  {
    // The identity opt-out keeps the old rule: with growth disabled the evidence must name the
    // target, and depth alone never proves a placement in that mode.
    WorldStateConfig identity_only = config();
    identity_only.placement_require_column_growth = false;
    WorldStateStore store(identity_only);
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto unnamed = place_against_evidence(
      store, state, {"sim:already_here"}, 0.734 - kCanPitchM, 9'850'000'000, 9'840'000'000);
    ASSERT_FALSE(unnamed);
    EXPECT_EQ(unnamed.error().code, WorldStateErrorCode::PredicateFailed);
    EXPECT_NE(unnamed.error().detail.find("naming the target"), std::string::npos);
  }
  {
    // Nor does a product the reservation never accounted for appearing beside the target.
    WorldStateStore store(config());
    const FixtureState state = populate_with_occupant(store, 0.734);
    const auto foreign = place_against_evidence(
      store, state, {"sim:already_here", "sim:can_01", "sim:foreign"}, 0.734 - kCanPitchM,
      9'850'000'000, 9'840'000'000);
    ASSERT_FALSE(foreign);
    EXPECT_EQ(foreign.error().code, WorldStateErrorCode::PredicateFailed);
  }
}

// The sensor-driven shape end to end: every destination observation is what the wrist depth
// producer publishes — geometry only, `observed_source_object_ids` empty. Default config
// (growth proof). Commit, Detached revalidation, and release must all accept it; this is the
// lifecycle Card 029 attempt 5 could never finish because naming ran first.
TEST(WorldStateReservations, WristShapedEvidenceProvesThePlacementAndReleasesTheReservation)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  EXPECT_DOUBLE_EQ(reserved.value().reservation.destination_available_depth_m, 0.85);
  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.value().token, "wrist-attach"}, ros_time(9'800'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;

  auto snapshot = store.snapshot();
  const auto evidence = store.update_lane(
    LaneObservation{
        LaneId{"lane_01"}, {}, 0.85 - kCanPitchM, false, ros_time(9'850'000'000)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(evidence) << evidence.error().detail;

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "wrist-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  EXPECT_TRUE(detached.value().reservation.placed_in_destination);
  EXPECT_EQ(
    store.snapshot().lanes.at(LaneId{"lane_01"}).contents,
    std::vector<ObjectId>{state.object_id});

  // Detached revalidation re-judges the same flag-selected proof against current evidence.
  const auto validated = store.validate_task_reservation(reserved.value().token);
  ASSERT_TRUE(validated) << validated.error().detail;

  const auto released = store.release_task_reservation(
    release_request(
      detached.value(), "wrist-release", ReservationOutcome::Succeeded,
      TaskPhase::Idle, FaultState::None),
    ros_time(9'950'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  EXPECT_FALSE(store.snapshot().active_reservation);
}

// Did not land: post-release, fresh, unobstructed identity-free evidence whose depth never moved
// against the grant-time baseline proves nothing, and the refusal names the growth it wanted.
TEST(WorldStateReservations, RefusesWristShapedEvidenceThatShowsNoGrowth)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "no-growth-attach"},
      ros_time(9'800'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto refused = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "no-growth-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("grown by one product pitch"), std::string::npos)
    << refused.error().detail;
  EXPECT_TRUE(store.snapshot().robot.held_object);
}

// Ambiguous: growth short of one pitch less tolerance — the dense-demo under-count shape (Card
// 019 measured 9.7 mm against a 67.8 mm pitch at capacity) — does not prove a placement. Fail
// closed rather than weaken the threshold.
TEST(WorldStateReservations, RefusesWristShapedEvidenceWithSubPitchGrowth)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "undercount-attach"},
      ros_time(9'800'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{
        LaneId{"lane_01"}, {}, 0.85 - 0.0097, false, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto refused = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "undercount-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("grown by one product pitch"), std::string::npos)
    << refused.error().detail;
}

// Hover: the depth shrink is real in the reading, but the observation predates the release, so
// the carry (or the arm in the lane mouth) would read as a placement. Ordering refuses first.
TEST(WorldStateReservations, RefusesWristShapedEvidenceOlderThanTheRelease)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "hover-attach"},
      ros_time(9'800'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{
        LaneId{"lane_01"}, {}, 0.85 - kCanPitchM, false, ros_time(9'840'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto refused = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "hover-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'850'000'000)},
    ros_time(9'900'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("recorded after the release"), std::string::npos)
    << refused.error().detail;
}

// Obstruction refuses regardless of producer or proof mode: a lane the camera could not see
// cleanly is not evidence of a landing, however the depth reads.
TEST(WorldStateReservations, RefusesWristShapedEvidenceWhileObstructed)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "obstructed-attach"},
      ros_time(9'800'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{
        LaneId{"lane_01"}, {}, 0.85 - kCanPitchM, true, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto refused = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "obstructed-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  // The Attached-stage destination predicate refuses before the placement proof runs; the
  // judged evidence still carries the obstruction the refusal is about.
  EXPECT_NE(refused.error().detail.find("obstructed=true"), std::string::npos)
    << refused.error().detail;
}

// After a wrist-shaped commit, a later survey that shows the growth gone (depth back to the
// grant-time baseline) fails Detached revalidation, so release refuses: the placement no longer
// stands per current evidence, fail-closed in both directions.
TEST(WorldStateReservations, DetachedRevalidationRefusesWhenTheGrowthCollapses)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.value().token, "collapse-attach"},
      ros_time(9'800'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{
        LaneId{"lane_01"}, {}, 0.85 - kCanPitchM, false, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "collapse-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;

  snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'910'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto refused = store.release_task_reservation(
    release_request(
      detached.value(), "collapse-release", ReservationOutcome::Succeeded,
      TaskPhase::Idle, FaultState::None),
    ros_time(9'950'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
}

// A product whose depth the world state cannot look up is not placed on the strength of the lane
// looking roomy; only the catalogue says how much of a lane a product costs.
TEST(WorldStateReservations, RefusesAProductWithNoCataloguedLaneDepth)
{
  WorldStateConfig without_profiles = config();
  without_profiles.product_lane_profiles.clear();
  WorldStateStore store(without_profiles);
  const FixtureState state = populate(store);
  const auto refused = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("no catalogued lane depth"), std::string::npos);
}

TEST(WorldStateReservations, AllowsEvidenceRevisionsAndValidatesStageSpecificOccupancy)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  EXPECT_EQ(reserved.token.size(), 32U);
  EXPECT_EQ(reserved.reservation.stage, ReservationStage::Reserved);
  EXPECT_EQ(store.snapshot().robot.task_phase, TaskPhase::Executing);

  auto moved = can_observation(9'700'000'000);
  moved.pose_in_world.translation().x() = 0.1;
  const auto observed = store.observe_object(moved, ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  auto snapshot = store.snapshot();
  const auto lane_revision = snapshot.lanes.at(LaneId{"lane_01"}).revision;
  const auto refreshed = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'700'000'000)},
    ros_time(10'000'000'000), lane_revision);
  ASSERT_TRUE(refreshed);
  const auto validation = store.validate_task_reservation(reserved.token);
  ASSERT_TRUE(validation) << validation.error().detail;
  EXPECT_GT(validation.value().revision, reserved.revision);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-0001"}, ros_time(9'800'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  EXPECT_EQ(attached.value().reservation.stage, ReservationStage::Attached);
  const auto attach_replay = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-0001"}, ros_time(9'900'000'000));
  ASSERT_TRUE(attach_replay);
  EXPECT_EQ(attach_replay.value().revision, attached.value().revision);

  snapshot = store.snapshot();
  const auto target_evidence = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.78, false, ros_time(9'850'000'000)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(target_evidence);
  EXPECT_TRUE(store.validate_task_reservation(reserved.token));

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-0001",
      DetachmentDisposition::PlaceInReservedDestination, ros_time(9'840'000'000)},
    ros_time(9'900'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  EXPECT_EQ(detached.value().reservation.stage, ReservationStage::Detached);
  EXPECT_TRUE(detached.value().reservation.placed_in_destination);
  snapshot = store.snapshot();
  EXPECT_FALSE(snapshot.robot.held_object);
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).contents, std::vector<ObjectId>{state.object_id});

  const auto conflicting_replay = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-0001",
      DetachmentDisposition::ReleaseWithoutMembership, ros_time(9'840'000'000)},
    ros_time(9'950'000'000));
  ASSERT_FALSE(conflicting_replay);
  EXPECT_EQ(conflicting_replay.error().code, WorldStateErrorCode::IdempotencyConflict);

  const auto wrong_placed_outcome = store.release_task_reservation(
    release_request(
      detached.value(), "release-placed-as-failed", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::Recoverable),
    ros_time(10'000'000'000));
  ASSERT_FALSE(wrong_placed_outcome);
  EXPECT_EQ(wrong_placed_outcome.error().code, WorldStateErrorCode::InvalidTransition);

  const auto release =
    release_request(
    detached.value(), "release-0001", ReservationOutcome::Succeeded,
    TaskPhase::Idle, FaultState::None);
  const auto released = store.release_task_reservation(release, ros_time(10'000'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  EXPECT_FALSE(store.snapshot().active_reservation);
  const auto release_replay = store.release_task_reservation(release, ros_time(10'100'000'000));
  ASSERT_TRUE(release_replay);
  EXPECT_EQ(release_replay.value().revision, released.value().revision);
}

// The gripper carries the product into the destination lane volume during pre-insert, so the lane
// names it before release. That satisfies every occupancy clause but proves only presence; evidence
// recorded after the verified release is required, and the same commit is then accepted.
TEST(WorldStateReservations, RefusesPlacementOnEvidenceOlderThanTheRelease)
{
  WorldStateStore store(config());
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.token, "pre-insert-attach", test_grasp()},
      ros_time(9'650'000'000)));

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.78, false, ros_time(9'700'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));

  const ReservedDetachmentRequest request{
    reserved.token, "pre-insert-detach", DetachmentDisposition::PlaceInReservedDestination,
    ros_time(9'800'000'000)};
  const auto refused = store.commit_reserved_detachment(request, ros_time(9'820'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("recorded after the release"), std::string::npos)
    << refused.error().detail;
  EXPECT_EQ(store.snapshot().robot.held_object, state.object_id);

  snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.78, false, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  const auto accepted = store.commit_reserved_detachment(request, ros_time(9'900'000'000));
  ASSERT_TRUE(accepted) << accepted.error().detail;
  EXPECT_TRUE(accepted.value().reservation.placed_in_destination);
  EXPECT_EQ(
    store.snapshot().lanes.at(LaneId{"lane_01"}).contents,
    std::vector<ObjectId>{state.object_id});
}

// Under a moving eye the destination may be tens of seconds old at commit; only evidence outside
// its validity horizon or an explicit invalidation refuses a placement, not ingest age.
TEST(WorldStateReservations, RefusesPlacementOnDestinationEvidenceOutsideValidityHorizon)
{
  WorldStateStore store(config());
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.token, "stale-evidence-attach", test_grasp()},
      ros_time(9'620'000'000)));

  const auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.78, false, ros_time(9'700'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));

  const ReservedDetachmentRequest request{
    reserved.token, "stale-evidence-detach", DetachmentDisposition::PlaceInReservedDestination,
    ros_time(9'650'000'000)};
  // 61 s after the survey: outside the validity horizon.
  const auto refused = store.commit_reserved_detachment(
    request, ros_time(9'700'000'000 + 61'000'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::PredicateFailed);
  EXPECT_NE(refused.error().detail.find("validity horizon"), std::string::npos)
    << refused.error().detail;
  EXPECT_TRUE(store.snapshot().lanes.at(LaneId{"lane_01"}).contents.empty());
}

TEST(WorldStateReservations, RefusesSelectionAndPlacementOnInvalidatedLaneEvidenceUntilResurveyed)
{
  WorldStateStore store(config());
  FixtureState state = populate(store);
  auto snapshot = store.snapshot();
  const auto invalidated = store.invalidate_lane_evidence(
    LaneId{"lane_01"}, ros_time(9'450'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(invalidated) << invalidated.error().detail;
  EXPECT_TRUE(store.snapshot().lanes.at(LaneId{"lane_01"}).evidence_invalidated);

  state.request.request_id = "task-invalidated-lane";
  state.request.selected_snapshot_revision = store.snapshot().revision;
  state.request.destination_lane_revision =
    store.snapshot().lanes.at(LaneId{"lane_01"}).revision;
  const auto refused_reserve = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_FALSE(refused_reserve);
  EXPECT_EQ(refused_reserve.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_NE(refused_reserve.error().detail.find("invalidated"), std::string::npos)
    << refused_reserve.error().detail;

  snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'500'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));
  EXPECT_FALSE(store.snapshot().lanes.at(LaneId{"lane_01"}).evidence_invalidated);

  state.request.request_id = "task-after-resurvey";
  state.request.selected_snapshot_revision = store.snapshot().revision;
  state.request.object_revision = store.snapshot().objects.at(state.object_id).revision;
  state.request.destination_lane_revision =
    store.snapshot().lanes.at(LaneId{"lane_01"}).revision;
  const auto reserved = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
}

// A release the caller cannot place inside this reservation's window is a caller defect: refused
// as an argument, never retried into an acceptance.
TEST(WorldStateReservations, RejectsPlacementReleaseTimesOutsideTheReservation)
{
  WorldStateStore store(config());
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.token, "window-attach", test_grasp()},
      ros_time(9'650'000'000)));
  const auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.78, false, ros_time(9'850'000'000)},
      ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision));

  for (const auto & released_at :
    {ros_time(0), ros_time(9'500'000'000), ros_time(9'950'000'000)})
  {
    const auto refused = store.commit_reserved_detachment(
      ReservedDetachmentRequest{
          reserved.token, "window-detach", DetachmentDisposition::PlaceInReservedDestination,
          released_at},
      ros_time(9'900'000'000));
    ASSERT_FALSE(refused) << released_at.nanoseconds();
    EXPECT_EQ(refused.error().code, WorldStateErrorCode::InvalidArgument);
  }
  EXPECT_EQ(store.snapshot().robot.held_object, state.object_id);
}

TEST(WorldStateReservations, RejectsForeignEvidenceWrongTokensAndUnsafeRelease)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto wrong_token = store.validate_task_reservation("not-the-token");
  ASSERT_FALSE(wrong_token);
  EXPECT_EQ(wrong_token.error().code, WorldStateErrorCode::TokenMismatch);

  const auto unsafe_release = store.release_task_reservation(
    release_request(
      reserved, "release-too-early", ReservationOutcome::Succeeded, TaskPhase::Idle,
      FaultState::None),
    ros_time(9'700'000'000));
  ASSERT_FALSE(unsafe_release);
  EXPECT_EQ(unsafe_release.error().code, WorldStateErrorCode::InvalidTransition);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-foreign"}, ros_time(9'700'000'000));
  ASSERT_TRUE(attached);
  auto snapshot = store.snapshot();
  const auto foreign = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {"sim:foreign"}, 0.70, false, ros_time(9'800'000'000)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(foreign);
  const auto invalid = store.validate_task_reservation(reserved.token);
  ASSERT_FALSE(invalid);
  EXPECT_EQ(invalid.error().code, WorldStateErrorCode::PredicateFailed);

  const auto release_attached = store.release_task_reservation(
    release_request(
      attached.value(), "release-attached", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::ExternalInconsistency),
    ros_time(9'900'000'000));
  ASSERT_FALSE(release_attached);
  EXPECT_EQ(release_attached.error().code, WorldStateErrorCode::InvalidTransition);
  EXPECT_TRUE(store.snapshot().active_reservation);
}

TEST(WorldStateReservations, ReturnsOneCapabilityAuthorizedExecutionWorldProof)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto expected = authority_expectation(reserved);
  const auto before = store.snapshot();

  auto proof = store.validate_execution_world_authority(
    reserved.token, expected);
  ASSERT_TRUE(proof) << proof.error().detail;
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.revision, before.revision);
  EXPECT_EQ(snapshot.events.size(), before.events.size());
  EXPECT_EQ(proof.value().revision, snapshot.revision);
  EXPECT_EQ(proof.value().planning_frame, "world");
  EXPECT_EQ(proof.value().reservation.reservation_id, reserved.reservation.reservation_id);
  EXPECT_EQ(proof.value().reservation.object_id, state.object_id);
  EXPECT_EQ(proof.value().object.id, state.object_id);
  EXPECT_EQ(proof.value().object.revision, snapshot.objects.at(state.object_id).revision);
  EXPECT_EQ(proof.value().destination_lane.id, state.request.destination_lane);
  EXPECT_EQ(
    proof.value().destination_lane.revision,
    snapshot.lanes.at(state.request.destination_lane).revision);
  EXPECT_FALSE(proof.value().source_lane);
  EXPECT_EQ(proof.value().robot.revision, snapshot.robot.revision);
  EXPECT_EQ(proof.value().robot.telemetry_revision, snapshot.robot.telemetry_revision);

  proof.value().object.source_object_id = "mutated-copy";
  proof.value().destination_lane.obstructed = true;
  proof.value().robot.rail_position = 1.0;
  const auto repeated = store.validate_execution_world_authority(reserved.token, expected);
  ASSERT_TRUE(repeated) << repeated.error().detail;
  EXPECT_EQ(repeated.value().object.source_object_id, "sim:can_01");
  EXPECT_FALSE(repeated.value().destination_lane.obstructed);
  EXPECT_DOUBLE_EQ(repeated.value().robot.rail_position, 0.0);
  EXPECT_EQ(repeated.value().revision, before.revision);

  const auto wrong_token = store.validate_execution_world_authority(
    "not-the-token", {});
  ASSERT_FALSE(wrong_token);
  EXPECT_EQ(wrong_token.error().code, WorldStateErrorCode::TokenMismatch);
  auto wrong_object_expectation = expected;
  wrong_object_expectation.object_id = ObjectId{state.object_id.value + 1U};
  const auto wrong_object = store.validate_execution_world_authority(
    reserved.token, wrong_object_expectation);
  ASSERT_FALSE(wrong_object);
  EXPECT_EQ(wrong_object.error().code, WorldStateErrorCode::IdentityConflict);
  auto wrong_lane_expectation = expected;
  wrong_lane_expectation.destination_lane = LaneId{"other_lane"};
  const auto wrong_lane = store.validate_execution_world_authority(
    reserved.token, wrong_lane_expectation);
  ASSERT_FALSE(wrong_lane);
  EXPECT_EQ(wrong_lane.error().code, WorldStateErrorCode::IdentityConflict);
  auto malformed_object_expectation = expected;
  malformed_object_expectation.object_id = ObjectId{};
  const auto malformed_object = store.validate_execution_world_authority(
    reserved.token, malformed_object_expectation);
  ASSERT_FALSE(malformed_object);
  EXPECT_EQ(malformed_object.error().code, WorldStateErrorCode::InvalidArgument);
  auto malformed_lane_expectation = expected;
  malformed_lane_expectation.destination_lane = LaneId{};
  const auto malformed_lane = store.validate_execution_world_authority(
    reserved.token, malformed_lane_expectation);
  ASSERT_FALSE(malformed_lane);
  EXPECT_EQ(malformed_lane.error().code, WorldStateErrorCode::InvalidArgument);
  auto stale_reservation = expected;
  ++stale_reservation.reservation_revision;
  const auto stale = store.validate_execution_world_authority(
    reserved.token, stale_reservation);
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::RevisionConflict);
  auto zero_reservation_id = expected;
  zero_reservation_id.reservation_id = 0U;
  const auto malformed_reservation_id = store.validate_execution_world_authority(
    reserved.token, zero_reservation_id);
  ASSERT_FALSE(malformed_reservation_id);
  EXPECT_EQ(malformed_reservation_id.error().code, WorldStateErrorCode::InvalidArgument);
  auto zero_reservation_revision = expected;
  zero_reservation_revision.reservation_revision = 0U;
  const auto malformed_reservation_revision = store.validate_execution_world_authority(
    reserved.token, zero_reservation_revision);
  ASSERT_FALSE(malformed_reservation_revision);
  EXPECT_EQ(malformed_reservation_revision.error().code, WorldStateErrorCode::InvalidArgument);
  auto wrong_reservation_id = expected;
  ++wrong_reservation_id.reservation_id;
  const auto wrong_reservation = store.validate_execution_world_authority(
    reserved.token, wrong_reservation_id);
  ASSERT_FALSE(wrong_reservation);
  EXPECT_EQ(wrong_reservation.error().code, WorldStateErrorCode::RevisionConflict);

  const auto obstructed = store.update_lane(
    LaneObservation{
        state.request.destination_lane, {}, 0.85, true, ros_time(9'700'000'000)},
    ros_time(10'000'000'000),
    snapshot.lanes.at(state.request.destination_lane).revision);
  ASSERT_TRUE(obstructed);
  const auto invalidated = store.validate_execution_world_authority(
    reserved.token, expected);
  ASSERT_FALSE(invalidated);
  EXPECT_EQ(invalidated.error().code, WorldStateErrorCode::PredicateFailed);
}

TEST(WorldStateReservations, IncludesReservedSourceLaneInExecutionWorldProof)
{
  WorldStateStore store(config());
  const FixtureState initial = populate(store);
  const auto source = store.configure_lane(
    LaneDefinition{LaneId{"stock_lane"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'575'000'000));
  ASSERT_TRUE(source);

  auto snapshot = store.snapshot();
  ASSERT_TRUE(
    store.commit_attachment(
      initial.object_id, test_grasp(), ros_time(9'600'000'000),
      AttachPreconditions{
        snapshot.objects.at(initial.object_id).revision, snapshot.robot.revision, std::nullopt}));
  snapshot = store.snapshot();
  ASSERT_TRUE(
    store.commit_detachment(
      initial.object_id, LaneId{"stock_lane"}, ros_time(9'625'000'000),
      DetachPreconditions{
        snapshot.objects.at(initial.object_id).revision, snapshot.robot.revision,
        snapshot.lanes.at(LaneId{"stock_lane"}).revision}));

  snapshot = store.snapshot();
  const ReserveTaskRequest request{
    "source-proof-reserve", snapshot.revision, initial.object_id,
    snapshot.objects.at(initial.object_id).revision,
    LaneId{"stock_lane"}, snapshot.lanes.at(LaneId{"stock_lane"}).revision,
    initial.request.destination_lane,
    snapshot.lanes.at(initial.request.destination_lane).revision};
  const auto reserved = store.reserve_task(request, ros_time(9'650'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;

  const auto proof = store.validate_execution_world_authority(
    reserved.value().token, authority_expectation(reserved.value()));
  ASSERT_TRUE(proof) << proof.error().detail;
  ASSERT_TRUE(proof.value().source_lane);
  EXPECT_EQ(proof.value().source_lane->id, LaneId{"stock_lane"});
  EXPECT_EQ(proof.value().source_lane->contents, std::vector<ObjectId>{initial.object_id});
  EXPECT_EQ(proof.value().revision, store.snapshot().revision);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.value().token, "source-proof-attach"},
    ros_time(9'675'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  const auto attached_proof = store.validate_execution_world_authority(
    reserved.value().token, authority_expectation(attached.value()));
  ASSERT_TRUE(attached_proof) << attached_proof.error().detail;
  ASSERT_TRUE(attached_proof.value().source_lane);
  EXPECT_TRUE(attached_proof.value().source_lane->contents.empty());
  EXPECT_EQ(attached_proof.value().object.grasp_state, GraspState::Attached);
  EXPECT_EQ(attached_proof.value().robot.held_object, initial.object_id);

  snapshot = store.snapshot();
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{
        initial.request.destination_lane, {"sim:can_01"}, 0.78, false,
        ros_time(9'700'000'000)},
      ros_time(10'000'000'000),
      snapshot.lanes.at(initial.request.destination_lane).revision));
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.value().token, "source-proof-detach",
        DetachmentDisposition::PlaceInReservedDestination, ros_time(9'690'000'000)},
    ros_time(9'725'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  const auto detached_proof = store.validate_execution_world_authority(
    reserved.value().token, authority_expectation(detached.value()));
  ASSERT_TRUE(detached_proof) << detached_proof.error().detail;
  ASSERT_TRUE(detached_proof.value().source_lane);
  EXPECT_TRUE(detached_proof.value().source_lane->contents.empty());
  EXPECT_EQ(
    detached_proof.value().destination_lane.contents,
    std::vector<ObjectId>{initial.object_id});
  EXPECT_EQ(detached_proof.value().object.grasp_state, GraspState::Free);
  EXPECT_FALSE(detached_proof.value().robot.held_object);

  const auto released = store.release_task_reservation(
    release_request(
      detached.value(), "source-proof-release", ReservationOutcome::Succeeded,
      TaskPhase::Idle, FaultState::None),
    ros_time(9'750'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  const auto old_token = store.validate_execution_world_authority(
    reserved.value().token, authority_expectation(detached.value()));
  ASSERT_FALSE(old_token);
  EXPECT_EQ(old_token.error().code, WorldStateErrorCode::TokenMismatch);
}

TEST(WorldStateReservations, ExecutionWorldProofNeverTearsAcrossLifecycleMutations)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ExecutionWorldAuthorityExpectation current = authority_expectation(reserved);
  std::mutex expectation_mutex;
  std::condition_variable attached_condition;
  std::atomic<bool> saw_attached{false};
  std::atomic<bool> writer_done{false};
  std::atomic<bool> writer_failed{false};
  std::atomic<bool> invalid_proof{false};

  const auto tuple_is_valid = [&state](const ExecutionWorldAuthorityProof & proof) {
    if (proof.revision == 0U || proof.reservation.object_id != state.object_id ||
      proof.object.id != state.object_id ||
      proof.destination_lane.id != state.request.destination_lane || proof.source_lane)
    {
      return false;
    }
    switch (proof.reservation.stage) {
      case ReservationStage::Reserved:
        return proof.object.grasp_state == GraspState::Free && !proof.robot.held_object &&
               proof.destination_lane.contents.empty();
      case ReservationStage::Attached:
        return proof.object.grasp_state == GraspState::Attached &&
               proof.robot.held_object == state.object_id &&
               proof.destination_lane.contents.empty();
      case ReservationStage::Detached:
        return proof.object.grasp_state == GraspState::Free && !proof.robot.held_object &&
               proof.destination_lane.contents == std::vector<ObjectId>{state.object_id};
    }
    return false;
  };

  const auto initial = store.validate_execution_world_authority(reserved.token, current);
  ASSERT_TRUE(initial) << initial.error().detail;
  ASSERT_TRUE(tuple_is_valid(initial.value()));

  std::thread writer([&]() {
      const auto attached = store.commit_reserved_attachment(
        ReservedAttachmentRequest{reserved.token, "proof-race-attach"},
        ros_time(9'700'000'000));
      if (!attached) {
        writer_failed.store(true);
        writer_done.store(true);
        return;
      }
      {
        std::lock_guard lock(expectation_mutex);
        current = authority_expectation(attached.value());
      }
      {
        std::unique_lock lock(expectation_mutex);
        if (!attached_condition.wait_for(lock, 2s, [&]() {return saw_attached.load();})) {
          writer_failed.store(true);
          writer_done.store(true);
          return;
        }
      }
      auto snapshot = store.snapshot();
      const auto evidence = store.update_lane(
        LaneObservation{
          state.request.destination_lane, {"sim:can_01"}, 0.78, false,
          ros_time(9'725'000'000)},
        ros_time(10'000'000'000),
        snapshot.lanes.at(state.request.destination_lane).revision);
      if (!evidence) {
        writer_failed.store(true);
        writer_done.store(true);
        return;
      }
      const auto detached = store.commit_reserved_detachment(
        ReservedDetachmentRequest{
          reserved.token, "proof-race-detach",
          DetachmentDisposition::PlaceInReservedDestination, ros_time(9'710'000'000)},
        ros_time(9'750'000'000));
      if (!detached) {
        writer_failed.store(true);
        writer_done.store(true);
        return;
      }
      {
        std::lock_guard lock(expectation_mutex);
        current = authority_expectation(detached.value());
      }
      writer_done.store(true);
    });

  while (!writer_done.load()) {
    ExecutionWorldAuthorityExpectation expected;
    {
      std::lock_guard lock(expectation_mutex);
      expected = current;
    }
    const auto proof = store.validate_execution_world_authority(reserved.token, expected);
    if (!proof) {
      if (proof.error().code != WorldStateErrorCode::RevisionConflict) {
        invalid_proof.store(true);
      }
      continue;
    }
    if (proof.value().reservation.reservation_id != expected.reservation_id ||
      proof.value().reservation.revision != expected.reservation_revision ||
      !tuple_is_valid(proof.value()))
    {
      invalid_proof.store(true);
    }
    if (proof.value().reservation.stage == ReservationStage::Attached) {
      saw_attached.store(true);
      attached_condition.notify_one();
    }
  }
  writer.join();

  EXPECT_FALSE(writer_failed.load());
  EXPECT_TRUE(saw_attached.load());
  EXPECT_FALSE(invalid_proof.load());
  ExecutionWorldAuthorityExpectation final_expected;
  {
    std::lock_guard lock(expectation_mutex);
    final_expected = current;
  }
  const auto final_proof = store.validate_execution_world_authority(
    reserved.token, final_expected);
  ASSERT_TRUE(final_proof) << final_proof.error().detail;
  EXPECT_EQ(final_proof.value().reservation.stage, ReservationStage::Detached);
  EXPECT_TRUE(tuple_is_valid(final_proof.value()));
}

TEST(WorldStateReservations, CancelsOnlyAConsistentPreAttachTask)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto cancel = release_request(
    reserved, "cancel-0001", ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  const auto result = store.release_task_reservation(cancel, ros_time(9'700'000'000));
  ASSERT_TRUE(result) << result.error().detail;
  const auto snapshot = store.snapshot();
  EXPECT_FALSE(snapshot.active_reservation);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Idle);
  EXPECT_EQ(snapshot.objects.at(state.object_id).grasp_state, GraspState::Free);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
}

TEST(WorldStateReservations, FailsSafelyBeforeAttachment)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto failed_safe =
    release_request(
    reserved, "failed-safe-0001", ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::NonRecoverable);
  auto invalid_terminal = failed_safe;
  invalid_terminal.operation_id = "failed-safe-invalid-terminal";
  invalid_terminal.terminal_task_phase = TaskPhase::Idle;
  invalid_terminal.terminal_fault_state = FaultState::None;
  const auto invalid = store.release_task_reservation(invalid_terminal, ros_time(9'700'000'000));
  ASSERT_FALSE(invalid);
  EXPECT_EQ(invalid.error().code, WorldStateErrorCode::InvalidTransition);
  EXPECT_TRUE(store.snapshot().active_reservation);

  const auto result = store.release_task_reservation(failed_safe, ros_time(9'700'000'000));
  ASSERT_TRUE(result) << result.error().detail;

  const auto snapshot = store.snapshot();
  EXPECT_FALSE(snapshot.active_reservation);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Fault);
  EXPECT_EQ(snapshot.robot.fault_state, FaultState::NonRecoverable);
  EXPECT_FALSE(snapshot.robot.held_object);
  EXPECT_EQ(snapshot.objects.at(state.object_id).grasp_state, GraspState::Free);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());

  const auto replay = store.release_task_reservation(failed_safe, ros_time(9'800'000'000));
  ASSERT_TRUE(replay);
  EXPECT_EQ(replay.value().revision, result.value().revision);

  auto conflicting_replay = failed_safe;
  conflicting_replay.outcome = ReservationOutcome::Canceled;
  conflicting_replay.terminal_task_phase = TaskPhase::Idle;
  conflicting_replay.terminal_fault_state = FaultState::None;
  const auto conflict = store.release_task_reservation(conflicting_replay, ros_time(9'900'000'000));
  ASSERT_FALSE(conflict);
  EXPECT_EQ(conflict.error().code, WorldStateErrorCode::IdempotencyConflict);
}

TEST(WorldStateReservations, TelemetryCannotClearTerminalTaskSemantics)
{
  const auto verify_terminal_phase = [](TaskPhase phase, FaultState fault) {
    WorldStateStore store(config());
    const FixtureState state = populate(store);
    const ReservationReceipt reserved = reserve(store, state);
    const auto released = store.release_task_reservation(
      release_request(
        reserved, "release-terminal-semantics", ReservationOutcome::FailedSafe,
        phase, fault),
      ros_time(9'700'000'000));
    ASSERT_TRUE(released) << released.error().detail;
    const auto before_telemetry = store.snapshot();

    const auto telemetry = store.observe_robot_telemetry(
      RobotTelemetryObservation{{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.1, 0.0,
        ros_time(9'750'000'000), {}, {}, "test/reservations"},
      ros_time(10'000'000'000));
    ASSERT_TRUE(telemetry) << telemetry.error().detail;
    const auto after_telemetry = store.snapshot();
    EXPECT_EQ(after_telemetry.robot.task_phase, phase);
    EXPECT_EQ(after_telemetry.robot.fault_state, fault);
    EXPECT_FALSE(after_telemetry.robot.held_object);
    EXPECT_GT(
      after_telemetry.robot.telemetry_revision,
      before_telemetry.robot.telemetry_revision);
  };

  verify_terminal_phase(TaskPhase::Fault, FaultState::ExternalInconsistency);
  verify_terminal_phase(TaskPhase::RequestingOperator, FaultState::NonRecoverable);
}

TEST(WorldStateReservations, RejectsStalePreMotionReleaseAfterDetachment)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto stale_release =
    release_request(
    reserved, "stale-pre-motion-release", ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::NonRecoverable);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-before-stale-release"},
    ros_time(9'700'000'000));
  ASSERT_TRUE(attached);
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-before-stale-release",
      DetachmentDisposition::ReleaseWithoutMembership},
    ros_time(9'800'000'000));
  ASSERT_TRUE(detached);

  const auto rejected = store.release_task_reservation(stale_release, ros_time(9'900'000'000));
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::RevisionConflict);
  ASSERT_TRUE(store.snapshot().active_reservation);
  EXPECT_EQ(store.snapshot().active_reservation->stage, ReservationStage::Detached);

  const auto canceled_detached = store.release_task_reservation(
    release_request(
      detached.value(), "canceled-detached-release", ReservationOutcome::Canceled,
      TaskPhase::Idle, FaultState::None),
    ros_time(9'900'000'000));
  ASSERT_FALSE(canceled_detached);
  EXPECT_EQ(canceled_detached.error().code, WorldStateErrorCode::InvalidTransition);

  const auto valid_release =
    release_request(
    detached.value(), "valid-detached-release", ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::NonRecoverable);
  EXPECT_TRUE(store.release_task_reservation(valid_release, ros_time(9'900'000'000)));
}

TEST(WorldStateReservations, PreservesSourceMembershipOnPreMotionFailure)
{
  WorldStateStore store(config());
  const auto destination = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  const auto source = store.configure_lane(
    LaneDefinition{LaneId{"stock_lane"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'000'000'000));
  ASSERT_TRUE(destination);
  ASSERT_TRUE(source);
  ASSERT_TRUE(
    store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'400'000'000)},
      ros_time(10'000'000'000), destination.value().revision));
  const auto observed =
    store.observe_object(can_observation(9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
      ros_time(9'550'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(telemetry);
  auto snapshot = store.snapshot();
  const auto attached = store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'600'000'000),
    AttachPreconditions{snapshot.objects.at(observed.value().object_id).revision,
      snapshot.robot.revision, std::nullopt});
  ASSERT_TRUE(attached);
  snapshot = store.snapshot();
  const auto detached = store.commit_detachment(
    observed.value().object_id, LaneId{"stock_lane"}, ros_time(9'700'000'000),
    DetachPreconditions{snapshot.objects.at(observed.value().object_id).revision,
      snapshot.robot.revision,
      snapshot.lanes.at(LaneId{"stock_lane"}).revision});
  ASSERT_TRUE(detached);

  snapshot = store.snapshot();
  const auto reserved = store.reserve_task(
    ReserveTaskRequest{"source-membership-reserve", snapshot.revision, observed.value().object_id,
      snapshot.objects.at(observed.value().object_id).revision,
      LaneId{"stock_lane"}, snapshot.lanes.at(LaneId{"stock_lane"}).revision,
      LaneId{"lane_01"}, snapshot.lanes.at(LaneId{"lane_01"}).revision},
    ros_time(9'800'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;

  const auto failed =
    release_request(
    reserved.value(), "source-membership-failed-safe",
    ReservationOutcome::FailedSafe, TaskPhase::Fault, FaultState::Recoverable);
  ASSERT_TRUE(store.release_task_reservation(failed, ros_time(9'900'000'000)));
  snapshot = store.snapshot();
  EXPECT_EQ(
    snapshot.lanes.at(LaneId{"stock_lane"}).contents,
    std::vector<ObjectId>{observed.value().object_id});
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
}

TEST(WorldStateReservations, SupportsVerifiedFreeRecoveryAndFaultRelease)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_TRUE(
    store.commit_reserved_attachment(
      ReservedAttachmentRequest{reserved.token, "attach-recovery"}, ros_time(9'700'000'000)));
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-recovery",
      DetachmentDisposition::ReleaseWithoutMembership},
    ros_time(9'800'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  EXPECT_FALSE(detached.value().reservation.placed_in_destination);
  const auto released = store.release_task_reservation(
    release_request(
      detached.value(), "release-recovery", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::Recoverable),
    ros_time(9'900'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  const auto snapshot = store.snapshot();
  EXPECT_FALSE(snapshot.active_reservation);
  EXPECT_FALSE(snapshot.robot.held_object);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Fault);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
}

TEST(WorldStateReservations, FencesLegacyObjectMutationsButPermitsUnrelatedEvidence)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  auto second_observation = can_observation(9'550'000'000);
  second_observation.source_object_id = "sim:can_02";
  second_observation.pose_in_world.translation().y() = 0.2;
  const auto second = store.observe_object(second_observation, ros_time(10'000'000'000));
  ASSERT_TRUE(second);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_FALSE(reserved.token.empty());

  auto snapshot = store.snapshot();
  const auto target_lifecycle = store.set_tracking_state(
    state.object_id, TrackingState::Lost, ros_time(9'700'000'000),
    ObjectContextPreconditions{snapshot.objects.at(state.object_id).revision, std::nullopt});
  ASSERT_FALSE(target_lifecycle);
  EXPECT_EQ(target_lifecycle.error().code, WorldStateErrorCode::ReservationConflict);

  const auto legacy_attachment =
    store.commit_attachment(
    state.object_id, test_grasp(), ros_time(9'700'000'000),
    AttachPreconditions{snapshot.objects.at(state.object_id).revision,
      snapshot.robot.revision, std::nullopt});
  ASSERT_FALSE(legacy_attachment);
  EXPECT_EQ(legacy_attachment.error().code, WorldStateErrorCode::ReservationConflict);

  const auto legacy_detachment =
    store.commit_detachment(
    state.object_id, std::nullopt, ros_time(9'700'000'000),
    DetachPreconditions{snapshot.objects.at(state.object_id).revision,
      snapshot.robot.revision, std::nullopt});
  ASSERT_FALSE(legacy_detachment);
  EXPECT_EQ(legacy_detachment.error().code, WorldStateErrorCode::ReservationConflict);

  const auto telemetry = store.observe_robot_telemetry(
    RobotTelemetryObservation{{0.1, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.2, 0.0,
      ros_time(9'700'000'000), {}, {}, "test/reservations"},
    ros_time(10'000'000'000));
  ASSERT_TRUE(telemetry) << telemetry.error().detail;
  snapshot = store.snapshot();
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Executing);
  EXPECT_EQ(snapshot.robot.fault_state, FaultState::None);
  EXPECT_FALSE(snapshot.robot.held_object);
  const auto unrelated_lifecycle = store.set_tracking_state(
    second.value().object_id, TrackingState::Lost, ros_time(9'750'000'000),
    ObjectContextPreconditions{second.value().revision, std::nullopt});
  ASSERT_TRUE(unrelated_lifecycle) << unrelated_lifecycle.error().detail;
  const auto unrelated_lane = store.configure_lane(
    LaneDefinition{LaneId{"lane_02"}, ProductClass::Can, "SIM-CAN-STD", 0.85},
    ros_time(9'800'000'000));
  ASSERT_TRUE(unrelated_lane) << unrelated_lane.error().detail;
  EXPECT_TRUE(store.validate_task_reservation(reserved.token));
}

TEST(WorldStateReservations, CheckpointsPreserveCancellationCapacityAndReplay)
{
  WorldStateStore store(config(5));
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto telemetry_before = store.snapshot().robot;
  ReservedTaskCheckpoint checkpoint{
    reserved.token, "checkpoint-0001", reserved.reservation.reservation_id,
    reserved.reservation.stage, reserved.reservation.revision,
    TaskPhase::Recovering, FaultState::Recoverable, ros_time(9'700'000'000)};
  const auto first = store.checkpoint_reserved_task(checkpoint);
  ASSERT_TRUE(first) << first.error().detail;
  const auto replay = store.checkpoint_reserved_task(checkpoint);
  ASSERT_TRUE(replay);
  EXPECT_EQ(replay.value().revision, first.value().revision);

  const auto after_checkpoint = store.snapshot().robot;
  EXPECT_EQ(after_checkpoint.joint_positions, telemetry_before.joint_positions);
  EXPECT_DOUBLE_EQ(after_checkpoint.rail_position, telemetry_before.rail_position);
  EXPECT_EQ(after_checkpoint.telemetry_time, telemetry_before.telemetry_time);
  EXPECT_EQ(after_checkpoint.telemetry_revision, telemetry_before.telemetry_revision);

  checkpoint.update_time = ros_time(9'750'000'000);
  const auto retry_with_new_server_time = store.checkpoint_reserved_task(checkpoint);
  ASSERT_TRUE(retry_with_new_server_time);
  EXPECT_EQ(retry_with_new_server_time.value().revision, first.value().revision);

  checkpoint.task_phase = TaskPhase::Fault;
  checkpoint.fault_state = FaultState::ExternalInconsistency;
  const auto conflict = store.checkpoint_reserved_task(checkpoint);
  ASSERT_FALSE(conflict);
  EXPECT_EQ(conflict.error().code, WorldStateErrorCode::IdempotencyConflict);

  checkpoint.operation_id = "checkpoint-over-capacity";
  checkpoint.expected_reservation_revision = first.value().revision;
  const auto exhausted = store.checkpoint_reserved_task(checkpoint);
  ASSERT_FALSE(exhausted);
  EXPECT_EQ(exhausted.error().code, WorldStateErrorCode::ResourceExhausted);
  EXPECT_EQ(store.snapshot().active_reservation->stage, ReservationStage::Reserved);

  const auto cancel = release_request(
    first.value(), "cancel-at-capacity", ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  const auto released = store.release_task_reservation(cancel, ros_time(9'800'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  EXPECT_FALSE(store.snapshot().active_reservation);
  EXPECT_EQ(
    store.release_task_reservation(cancel, ros_time(9'900'000'000)).value().revision,
    released.value().revision);
  checkpoint = ReservedTaskCheckpoint{
    reserved.token, "checkpoint-0001", reserved.reservation.reservation_id,
    reserved.reservation.stage, reserved.reservation.revision,
    TaskPhase::Recovering, FaultState::Recoverable, ros_time(9'700'000'000)};
  EXPECT_EQ(store.checkpoint_reserved_task(checkpoint).value().revision, first.value().revision);
}

TEST(WorldStateReservations, RefusesAdmissionWithoutCompleteLifecycleCapacity)
{
  for (const std::size_t capacity : {1U, 2U, 3U}) {
    SCOPED_TRACE(capacity);
    WorldStateStore store(config(capacity));
    const auto state = populate(store);
    const auto before = store.snapshot();
    for (int attempt = 0; attempt < 2; ++attempt) {
      const auto refused = store.reserve_task(state.request, ros_time(9'600'000'000));
      ASSERT_FALSE(refused);
      EXPECT_EQ(refused.error().code, WorldStateErrorCode::ResourceExhausted);
      EXPECT_EQ(store.snapshot().revision, before.revision);
      EXPECT_FALSE(store.snapshot().active_reservation);
    }
  }
}

TEST(WorldStateReservations, MinimalJournalProtectsEveryStageUntilRelease)
{
  WorldStateStore store(config(4));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  auto refuse_checkpoint = [&store](const ReservationReceipt & receipt) {
    const auto checkpoint = store.checkpoint_reserved_task(
      ReservedTaskCheckpoint{
          receipt.token, "extra-checkpoint", receipt.reservation.reservation_id,
          receipt.reservation.stage, receipt.reservation.revision,
          TaskPhase::Executing, FaultState::None, ros_time(9'800'000'000)});
    ASSERT_FALSE(checkpoint);
    EXPECT_EQ(checkpoint.error().code, WorldStateErrorCode::ResourceExhausted);
    EXPECT_EQ(store.snapshot().revision, receipt.revision);
  };
  refuse_checkpoint(reserved);
  const ReservedAttachmentRequest attach{reserved.token, "attach-minimal"};
  const auto attached = store.commit_reserved_attachment(attach, ros_time(9'700'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  refuse_checkpoint(attached.value());
  const ReservedDetachmentRequest detach{
    reserved.token, "detach-minimal", DetachmentDisposition::ReleaseWithoutMembership};
  const auto detached = store.commit_reserved_detachment(detach, ros_time(9'800'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  refuse_checkpoint(detached.value());
  const auto release = release_request(
    detached.value(), "release-minimal", ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::Recoverable);
  const auto released = store.release_task_reservation(release, ros_time(9'900'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  EXPECT_FALSE(store.snapshot().active_reservation);
  EXPECT_FALSE(store.snapshot().robot.held_object);
  auto next_request = state.request;
  next_request.request_id = "next-reservation";
  const auto exhausted = store.reserve_task(next_request, ros_time(9'950'000'000));
  ASSERT_FALSE(exhausted);
  EXPECT_EQ(exhausted.error().code, WorldStateErrorCode::ResourceExhausted);
  EXPECT_EQ(
    store.reserve_task(state.request, ros_time(9'600'000'000)).value().revision,
    reserved.revision);
  EXPECT_EQ(
    store.commit_reserved_attachment(attach, ros_time(9'700'000'000)).value().revision,
    attached.value().revision);
  EXPECT_EQ(
    store.commit_reserved_detachment(detach, ros_time(9'800'000'000)).value().revision,
    detached.value().revision);
  EXPECT_EQ(
    store.release_task_reservation(release, ros_time(9'900'000'000)).value().revision,
    released.value().revision);
}

TEST(WorldStateReservations, CheckpointsRequireClosedSemanticsAndExactReservationVersion)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  const auto before = store.snapshot();

  ReservedTaskCheckpoint stale_time{
    reserved.token, "checkpoint-stale-time", reserved.reservation.reservation_id,
    reserved.reservation.stage, reserved.reservation.revision,
    TaskPhase::Executing, FaultState::None, ros_time(9'500'000'000)};
  const auto out_of_order = store.checkpoint_reserved_task(stale_time);
  ASSERT_FALSE(out_of_order);
  EXPECT_EQ(out_of_order.error().code, WorldStateErrorCode::OutOfOrder);
  EXPECT_EQ(store.snapshot().revision, before.revision);

  ReservedTaskCheckpoint checkpoint{
    reserved.token, "checkpoint-invalid-semantics", reserved.reservation.reservation_id,
    reserved.reservation.stage, reserved.reservation.revision,
    TaskPhase::Executing, FaultState::Recoverable, ros_time(9'700'000'000)};
  const auto invalid_semantics = store.checkpoint_reserved_task(checkpoint);
  ASSERT_FALSE(invalid_semantics);
  EXPECT_EQ(invalid_semantics.error().code, WorldStateErrorCode::InvalidTransition);
  EXPECT_EQ(store.snapshot().revision, before.revision);

  checkpoint.operation_id = "checkpoint-stale-revision";
  checkpoint.task_phase = TaskPhase::Executing;
  checkpoint.fault_state = FaultState::None;
  --checkpoint.expected_reservation_revision;
  const auto stale_revision = store.checkpoint_reserved_task(checkpoint);
  ASSERT_FALSE(stale_revision);
  EXPECT_EQ(stale_revision.error().code, WorldStateErrorCode::RevisionConflict);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-before-checkpoint"},
    ros_time(9'750'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  checkpoint.operation_id = "checkpoint-delayed-stage";
  checkpoint.expected_reservation_revision = reserved.reservation.revision;
  const auto delayed = store.checkpoint_reserved_task(checkpoint);
  ASSERT_FALSE(delayed);
  EXPECT_EQ(delayed.error().code, WorldStateErrorCode::RevisionConflict);
}

TEST(WorldStateReservations, SnapshotReadersObserveOnlyAtomicStages)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  std::atomic<bool> stop{false};
  std::atomic<bool> invalid{false};
  std::thread reader([&]() {
      while (!stop.load(std::memory_order_acquire)) {
        const auto snapshot = store.snapshot();
        if (!snapshot.active_reservation) {
          continue;
        }
        const auto & object = snapshot.objects.at(state.object_id);
        const auto stage = snapshot.active_reservation->stage;
        const bool reserved_consistent =
        stage != ReservationStage::Reserved ||
        (object.grasp_state == GraspState::Free && !snapshot.robot.held_object);
        const bool attached_consistent =
        stage != ReservationStage::Attached || (object.grasp_state == GraspState::Attached &&
        snapshot.robot.held_object == state.object_id);
        const bool detached_consistent =
        stage != ReservationStage::Detached ||
        (object.grasp_state == GraspState::Free && !snapshot.robot.held_object);
        if (!reserved_consistent || !attached_consistent || !detached_consistent) {
          invalid.store(true, std::memory_order_release);
        }
      }
    });

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-concurrent"}, ros_time(9'700'000'000));
  if (!attached) {
    stop.store(true, std::memory_order_release);
    reader.join();
    FAIL() << attached.error().detail;
  }
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-concurrent",
      DetachmentDisposition::ReleaseWithoutMembership},
    ros_time(9'800'000'000));
  stop.store(true, std::memory_order_release);
  reader.join();
  ASSERT_TRUE(detached) << detached.error().detail;
  EXPECT_FALSE(invalid.load(std::memory_order_acquire));
}

// SC-003, revision branch: a policy change bumps the lane revision, so a reservation request
// built from the pre-change snapshot fails on the existing revision-conflict path and a fresh
// request is admitted against the new revision once the invalidated evidence is re-surveyed.
TEST(WorldStateReservations, RefusesReservationSelectedBeforeALanePolicyChange)
{
  WorldStateStore store(config());
  FixtureState state = populate(store);
  ASSERT_NE(state.object_id.value, 0U);

  const auto policy = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 8},
    state.request.destination_lane_revision, ros_time(9'450'000'000),
    [](const std::vector<LanePolicy> &) {return true;});
  ASSERT_TRUE(policy) << (policy ? "" : policy.error().detail);

  // The selection captured before the change conflicts; nothing is reserved.
  const auto stale = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::RevisionConflict);
  EXPECT_FALSE(store.snapshot().active_reservation);

  // The policy change invalidated the lane's evidence, so the lane is re-surveyed before it is
  // selected again — under the post-change revision.
  const auto after_policy = store.snapshot();
  const auto resurveyed = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'450'000'000)},
    ros_time(10'000'000'000), after_policy.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(resurveyed) << (resurveyed ? "" : resurveyed.error().detail);

  const auto fresh = store.snapshot();
  state.request.selected_snapshot_revision = fresh.revision;
  state.request.object_revision = fresh.objects.at(state.object_id).revision;
  state.request.destination_lane_revision = fresh.lanes.at(LaneId{"lane_01"}).revision;
  const auto granted = store.reserve_task(state.request, ros_time(9'600'000'000));
  ASSERT_TRUE(granted) << (granted ? "" : granted.error().detail);
  EXPECT_EQ(granted.value().reservation.destination_expected_product_class, ProductClass::Can);
}

// SC-003, completion branch: a reservation already granted completes under the policy it was
// granted under, even when the lane's live intent has since changed to a different product.
TEST(WorldStateReservations, InFlightReservationCompletesUnderItsCapturedPolicy)
{
  WorldStateStore store(config());
  const FixtureState state = populate(store);
  const ReservationReceipt reserved = reserve(store, state);
  ASSERT_EQ(reserved.reservation.destination_expected_product_class, ProductClass::Can);
  ASSERT_EQ(reserved.reservation.destination_expected_sku, "SIM-CAN-STD");

  // The owner re-parks the lane on a different product while the transfer is mid-air.
  const auto policy = store.set_lane_policy(
    LanePolicy{LaneId{"lane_01"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 3},
    store.snapshot().lanes.at(LaneId{"lane_01"}).revision,
    ros_time(9'650'000'000),
    [](const std::vector<LanePolicy> &) {return true;});
  ASSERT_TRUE(policy) << (policy ? "" : policy.error().detail);
  {
    const auto after_change = store.snapshot();
    const auto & live_lane = after_change.lanes.at(LaneId{"lane_01"});
    EXPECT_EQ(live_lane.expected_product_class, ProductClass::SmallBottle);
    EXPECT_EQ(live_lane.target_count, 3U);
    EXPECT_TRUE(live_lane.evidence_invalidated);
  }

  // Revalidation runs against the captured policy, so the grant still validates, still holds
  // execution authority, and still attaches — no new conflict path, nothing stranded.
  const auto validation = store.validate_task_reservation(reserved.token);
  ASSERT_TRUE(validation) << (validation ? "" : validation.error().detail);
  EXPECT_EQ(
    validation.value().reservation.destination_expected_product_class, ProductClass::Can);
  const auto authority = store.validate_execution_world_authority(
    reserved.token, authority_expectation(reserved));
  ASSERT_TRUE(authority) << (authority ? "" : authority.error().detail);
  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-policy"}, ros_time(9'700'000'000));
  ASSERT_TRUE(attached) << (attached ? "" : attached.error().detail);

  // Placement is proven by a post-release survey, which every transfer needs anyway and which
  // clears the policy change's evidence invalidation on the way.
  const auto snapshot = store.snapshot();
  const auto surveyed = store.update_lane(
    LaneObservation{
        LaneId{"lane_01"}, {"sim:can_01"}, 0.85 - kCanPitchM, false,
        ros_time(9'750'000'000)},
    ros_time(10'000'000'000), snapshot.lanes.at(LaneId{"lane_01"}).revision);
  ASSERT_TRUE(surveyed) << (surveyed ? "" : surveyed.error().detail);

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{reserved.token, "detach-policy",
      DetachmentDisposition::PlaceInReservedDestination, ros_time(9'740'000'000)},
    ros_time(9'800'000'000));
  ASSERT_TRUE(detached) << (detached ? "" : detached.error().detail);
  EXPECT_TRUE(detached.value().reservation.placed_in_destination);

  const auto release = release_request(
    detached.value(), "release-policy", ReservationOutcome::Succeeded,
    TaskPhase::Idle, FaultState::None);
  const auto done = store.release_task_reservation(release, ros_time(9'850'000'000));
  ASSERT_TRUE(done) << (done ? "" : done.error().detail);
  EXPECT_EQ(
    store.snapshot().lanes.at(LaneId{"lane_01"}).expected_product_class,
    ProductClass::SmallBottle);
}

// Test-only witness against the original token-only attachment fingerprint.
TEST(WorldStateReservations, ChangedAttachmentTransformMustConflictOnOriginalCore)
{
  WorldStateStore store(config(4));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  ReservedAttachmentRequest request{reserved.token, "payload-witness", test_grasp()};
  const auto original = store.commit_reserved_attachment(request, ros_time(9'700'000'000));
  ASSERT_TRUE(original);
  request.grasp_center_from_held_object.translation().x() += 0.001;
  const auto changed = store.commit_reserved_attachment(request, ros_time(9'710'000'000));
  ASSERT_FALSE(changed) << "a changed valid grasp transform must not replay the original success";
  EXPECT_EQ(changed.error().code, WorldStateErrorCode::IdempotencyConflict);
  EXPECT_EQ(store.snapshot().revision, original.value().revision);
}

// Card CMB-096 SC-002: a genuinely different rotation, and then a genuinely different
// translation, are different requests under the exact post-conversion coefficient identity.
// Each must fail with IdempotencyConflict without a state transition, and the original
// receipt must remain replayable afterwards, unchanged down to the stored grasp.
TEST(
  WorldStateReservations,
  ChangedRotationOrTranslationConflictsAndPreservesOriginalReceipt)
{
  WorldStateStore store(config(4));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  ReservedAttachmentRequest request{reserved.token, "payload-geometry-identity", test_grasp()};
  const auto original = store.commit_reserved_attachment(request, ros_time(9'700'000'000));
  ASSERT_TRUE(original) << original.error().detail;
  const Revision attached_revision = original.value().revision;
  ASSERT_EQ(store.snapshot().revision, attached_revision);

  // Same translation, different rotation: yaw 0.37 -> 0.5.
  request.grasp_center_from_held_object.linear() =
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto changed_rotation =
    store.commit_reserved_attachment(request, ros_time(9'710'000'000));
  ASSERT_FALSE(changed_rotation);
  EXPECT_EQ(changed_rotation.error().code, WorldStateErrorCode::IdempotencyConflict);
  EXPECT_FALSE(changed_rotation.historical_receipt());
  EXPECT_EQ(store.snapshot().revision, attached_revision);

  // Original rotation, different translation.
  request.grasp_center_from_held_object = test_grasp();
  request.grasp_center_from_held_object.translation().y() += 0.001;
  const auto changed_translation =
    store.commit_reserved_attachment(request, ros_time(9'720'000'000));
  ASSERT_FALSE(changed_translation);
  EXPECT_EQ(changed_translation.error().code, WorldStateErrorCode::IdempotencyConflict);
  EXPECT_FALSE(changed_translation.historical_receipt());
  EXPECT_EQ(store.snapshot().revision, attached_revision);

  // The original receipt is still intact: the exact request replays it, and the conflict
  // attempts overwrote neither the revision nor the stored grasp.
  request.grasp_center_from_held_object = test_grasp();
  const auto replay = store.commit_reserved_attachment(request, ros_time(9'730'000'000));
  ASSERT_TRUE(replay) << replay.error().detail;
  EXPECT_EQ(replay.value().revision, original.value().revision);
  EXPECT_EQ(replay.value().token, original.value().token);
  EXPECT_EQ(
    replay.value().reservation.reservation_id, original.value().reservation.reservation_id);
  EXPECT_EQ(replay.value().reservation.stage, original.value().reservation.stage);
  EXPECT_EQ(replay.value().reservation.revision, original.value().reservation.revision);
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.revision, attached_revision);
  EXPECT_EQ(
    snapshot.robot.grasp_center_from_held_object.translation().x(),
    test_grasp().translation().x());
  EXPECT_EQ(
    snapshot.robot.grasp_center_from_held_object.translation().y(),
    test_grasp().translation().y());
  EXPECT_EQ(
    snapshot.robot.grasp_center_from_held_object.translation().z(),
    test_grasp().translation().z());
}

// Card CMB-096 SC-002: attachment_fingerprint canonicalizes every zero coefficient
// (`value == 0.0 ? 0.0 : value`) before hashing the bit pattern, so -0.0 and +0.0 — equal as
// doubles, different in the sign bit — are the same request. Without that canonicalization
// these two payloads would hash differently and the second commit would conflict.
TEST(WorldStateReservations, SignedZeroCoefficientsReplayIdenticallyAfterCanonicalization)
{
  WorldStateStore store(config(4));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);

  Eigen::Isometry3d positive_zeros = test_grasp();
  Eigen::Isometry3d negative_zeros = test_grasp();
  // The yawed grasp has exact zeros in the third column/row of the rotation and we give the
  // held pose a zero z translation, so both a rotation-block and a translation coefficient
  // carry opposite sign bits below.
  positive_zeros.translation().z() = 0.0;
  negative_zeros.translation().z() = -0.0;
  positive_zeros.matrix()(0, 2) = 0.0;
  negative_zeros.matrix()(0, 2) = -0.0;
  positive_zeros.matrix()(2, 0) = 0.0;
  negative_zeros.matrix()(2, 0) = -0.0;

  // The payloads really differ in the sign bit of those zeros; only fingerprint
  // canonicalization equates them. (Coefficients are bound to locals first: passing
  // `matrix()(row, column)` straight into a call confuses the uncrustify formatter.)
  const double positive_z = positive_zeros.matrix()(2, 3);
  const double negative_z = negative_zeros.matrix()(2, 3);
  const double positive_r02 = positive_zeros.matrix()(0, 2);
  const double negative_r02 = negative_zeros.matrix()(0, 2);
  EXPECT_EQ(positive_z, negative_z);
  EXPECT_FALSE(std::signbit(positive_z));
  EXPECT_TRUE(std::signbit(negative_z));
  EXPECT_EQ(positive_r02, negative_r02);
  EXPECT_FALSE(std::signbit(positive_r02));
  EXPECT_TRUE(std::signbit(negative_r02));

  ReservedAttachmentRequest request{reserved.token, "attach-signed-zero", positive_zeros};
  const auto original = store.commit_reserved_attachment(request, ros_time(9'700'000'000));
  ASSERT_TRUE(original) << original.error().detail;
  request.grasp_center_from_held_object = negative_zeros;
  const auto replay = store.commit_reserved_attachment(request, ros_time(9'710'000'000));
  ASSERT_TRUE(replay) << replay.error().detail;
  EXPECT_EQ(replay.value().revision, original.value().revision);
  EXPECT_EQ(store.snapshot().revision, original.value().revision);
}

// CMB-SPEC-10 Stage 1: retention bounds are named and observable, never enforced differently.
const JournalRetentionSnapshot & journal_named(
  const std::vector<JournalRetentionSnapshot> & snapshots, const std::string & name)
{
  const auto found = std::find_if(
    snapshots.begin(), snapshots.end(),
    [&name](const JournalRetentionSnapshot & entry) {return entry.journal == name;});
  EXPECT_NE(found, snapshots.end()) << name;
  static const JournalRetentionSnapshot missing{};
  return found == snapshots.end() ? missing : *found;
}

TEST(WorldStateRetention, FreshStoreReportsEmptyBoundedJournalsAndInstanceId)
{
  WorldStateStore store(config(8));
  WorldStateStore other(config(8));
  const auto snapshots = store.retention_snapshot();
  const auto & operations = journal_named(snapshots, "world_state.operations");
  EXPECT_EQ(operations.capacity, 8U);
  EXPECT_EQ(operations.size, 0U);
  EXPECT_EQ(operations.open_obligations, 0U);
  EXPECT_EQ(operations.terminal_receipts, 0U);
  EXPECT_EQ(operations.reserved_credits, 0U);
  EXPECT_FALSE(operations.inhibited);
  EXPECT_FALSE(operations.evicting);
  EXPECT_FALSE(operations.epoch_id.empty());
  EXPECT_EQ(operations.epoch_id, store.retention_snapshot().front().epoch_id);
  EXPECT_NE(operations.epoch_id, other.retention_snapshot().front().epoch_id);
  const auto & events = journal_named(snapshots, "world_state.events");
  EXPECT_EQ(events.capacity, 64U);
  EXPECT_TRUE(events.evicting);
  EXPECT_EQ(events.open_obligations, 0U);
}

TEST(WorldStateRetention, ClassifiesOpenObligationsAndTerminalReceiptsAcrossLifecycle)
{
  WorldStateStore store(config(8));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  auto operations = [&store]() {
    return journal_named(store.retention_snapshot(), "world_state.operations");
  };
  auto expect_shape = [&](std::size_t size, std::size_t open, std::size_t terminal,
    std::size_t credits) {
    const auto journal = operations();
    EXPECT_EQ(journal.size, size);
    EXPECT_EQ(journal.open_obligations, open);
    EXPECT_EQ(journal.terminal_receipts, terminal);
    EXPECT_EQ(journal.open_obligations + journal.terminal_receipts, journal.size);
    EXPECT_EQ(journal.reserved_credits, credits);
    EXPECT_LE(journal.size + journal.reserved_credits, journal.capacity);
  };
  expect_shape(1, 1, 0, 3);

  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-retention"}, ros_time(9'700'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  expect_shape(2, 2, 0, 2);

  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.token, "detach-retention", DetachmentDisposition::ReleaseWithoutMembership},
    ros_time(9'800'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  expect_shape(3, 3, 0, 1);

  const auto released = store.release_task_reservation(
    release_request(
      detached.value(), "release-retention", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::Recoverable),
    ros_time(9'900'000'000));
  ASSERT_TRUE(released) << released.error().detail;
  expect_shape(4, 0, 4, 0);
}

TEST(WorldStateRetention, FullJournalKeepsEveryRecordAndRefusesWithoutEviction)
{
  WorldStateStore store(config(4));
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  const auto attached = store.commit_reserved_attachment(
    ReservedAttachmentRequest{reserved.token, "attach-full"}, ros_time(9'700'000'000));
  ASSERT_TRUE(attached) << attached.error().detail;
  const auto detached = store.commit_reserved_detachment(
    ReservedDetachmentRequest{
        reserved.token, "detach-full", DetachmentDisposition::ReleaseWithoutMembership},
    ros_time(9'800'000'000));
  ASSERT_TRUE(detached) << detached.error().detail;
  const auto released = store.release_task_reservation(
    release_request(
      detached.value(), "release-full", ReservationOutcome::FailedSafe,
      TaskPhase::Fault, FaultState::Recoverable),
    ros_time(9'900'000'000));
  ASSERT_TRUE(released) << released.error().detail;

  const auto before = journal_named(store.retention_snapshot(), "world_state.operations");
  EXPECT_EQ(before.size, before.capacity);
  auto next = state.request;
  next.request_id = "next-after-full";
  const auto refused = store.reserve_task(next, ros_time(9'950'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::ResourceExhausted);
  const auto after = journal_named(store.retention_snapshot(), "world_state.operations");
  EXPECT_EQ(after.size, before.size);
  EXPECT_EQ(after.terminal_receipts, before.terminal_receipts);
  EXPECT_EQ(
    store.reserve_task(state.request, ros_time(9'600'000'000)).value().revision,
    reserved.revision);
}

TEST(WorldStateRetention, CheckpointIsOpenAndEventJournalReportsFifoBound)
{
  auto small = config(8);
  small.event_capacity = 2;
  WorldStateStore store(small);
  const auto state = populate(store);
  const auto reserved = reserve(store, state);
  const auto checkpoint = store.checkpoint_reserved_task(
    ReservedTaskCheckpoint{
        reserved.token, "checkpoint-retention", reserved.reservation.reservation_id,
        reserved.reservation.stage, reserved.reservation.revision,
        TaskPhase::Recovering, FaultState::Recoverable, ros_time(9'700'000'000)});
  ASSERT_TRUE(checkpoint) << checkpoint.error().detail;
  const auto snapshots = store.retention_snapshot();
  const auto & operations = journal_named(snapshots, "world_state.operations");
  EXPECT_EQ(operations.size, 2U);
  EXPECT_EQ(operations.open_obligations, 2U);
  const auto & events = journal_named(snapshots, "world_state.events");
  EXPECT_EQ(events.capacity, 2U);
  EXPECT_EQ(events.size, 2U);
  EXPECT_TRUE(events.evicting);
}

TEST(WorldStateRetention, ClockInhibitionIsReportedWithoutChangingJournalCounts)
{
  WorldStateStore store(config(8));
  static_cast<void>(populate(store));  // arms the clock authority with first evidence
  const auto before = journal_named(store.retention_snapshot(), "world_state.operations");
  EXPECT_FALSE(before.inhibited);
  store.notify_clock_discontinuity();
  const auto after = journal_named(store.retention_snapshot(), "world_state.operations");
  EXPECT_TRUE(after.inhibited);
  EXPECT_EQ(after.size, before.size);
  EXPECT_EQ(after.capacity, before.capacity);
  EXPECT_EQ(after.epoch_id, before.epoch_id) << "a clock discontinuity is not an epoch";
}

}  // namespace
}  // namespace restocker_world_state
