// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <stdexcept>

#include "restocker_task_executor/coordinator_startup.hpp"
#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using restocker_world_state::FaultState;
using restocker_world_state::GraspState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationStage;
using restocker_world_state::ShelfLane;
using restocker_world_state::SnapshotMessageOptions;
using restocker_world_state::TaskPhase;
using restocker_world_state::TaskReservation;
using restocker_world_state::TrackedObject;
using restocker_world_state::TrackingState;
using restocker_world_state::WorldStateSnapshot;

constexpr SteadyTime kNow{10s};

[[nodiscard]] StartupAuthorityClassification ready_classification()
{
  return {CoordinatorStartupStatus::kReady, std::nullopt, "clean"};
}

[[nodiscard]] restocker_interfaces::msg::WorldStateSnapshot orphan_message(
  ReservationStage stage)
{
  const rclcpp::Time observed_at{10'000'000'000LL, RCL_ROS_TIME};
  WorldStateSnapshot snapshot;
  snapshot.revision = 12U;

  TrackedObject object;
  object.id = ObjectId{17U};
  object.source_object_id = "sim:can_17";
  object.product_class = ProductClass::Can;
  object.sku = "SIM-CAN-STD";
  object.pose_covariance.setIdentity();
  object.orientation = ObjectOrientation::Upright;
  object.tracking_state = TrackingState::Tracked;
  object.grasp_state = stage == ReservationStage::Attached ?
    GraspState::Attached : GraspState::Free;
  object.observation_time = observed_at;
  object.transition_time = observed_at;
  object.revision = 8U;
  snapshot.objects.emplace(object.id, object);

  ShelfLane source;
  source.id = LaneId{"stock_lane"};
  source.expected_product_class = ProductClass::Can;
  source.expected_sku = object.sku;
  if (stage == ReservationStage::Reserved) {
    source.contents.push_back(object.id);
  }
  source.depth_m = 0.85;
  source.available_depth_m = 0.70;
  source.last_verified = observed_at;
  source.evidence_revision = 9U;
  source.revision = 9U;
  snapshot.lanes.emplace(source.id, source);

  ShelfLane destination = source;
  destination.id = LaneId{"lane_01"};
  destination.contents.clear();
  destination.available_depth_m = 0.85;
  destination.evidence_revision = 10U;
  destination.revision = 10U;
  if (stage == ReservationStage::Detached) {
    destination.contents.push_back(object.id);
  }
  snapshot.lanes.emplace(destination.id, destination);

  snapshot.robot.telemetry_time = observed_at;
  snapshot.robot.telemetry_source_id = "test/coordinator-startup";
  snapshot.robot.telemetry_revision = 7U;
  snapshot.robot.revision = 11U;
  snapshot.robot.task_phase = TaskPhase::Executing;
  if (stage == ReservationStage::Attached) {
    snapshot.robot.held_object = object.id;
  }
  snapshot.active_reservation = TaskReservation{
    81U, "orphaned-request-81", object.id, object.source_object_id,
    object.product_class, object.sku, source.id, destination.id, stage,
    stage == ReservationStage::Detached, observed_at, 11U, 7U, 0.85, {}, 12U,
    ProductClass::Unknown, std::nullopt};

  return restocker_world_state::snapshot_to_message(
    snapshot, SnapshotMessageOptions{"world", observed_at, false, false});
}

TEST(CoordinatorStartupGate, RejectsInvalidDurations)
{
  EXPECT_THROW(
    CoordinatorStartupGate({std::chrono::milliseconds::zero(), 1ms}), std::invalid_argument);
  EXPECT_THROW(CoordinatorStartupGate({1ms, -1ms}), std::invalid_argument);
  EXPECT_THROW(
    CoordinatorStartupGate({std::chrono::milliseconds::max(), 1ms}), std::invalid_argument);
}

TEST(CoordinatorStartupGate, WaitsForServiceThenSubmitsOneGeneration)
{
  CoordinatorStartupGate gate({100ms, 25ms});

  EXPECT_EQ(gate.poll(kNow, false).kind, StartupProbeActionKind::kNone);
  const auto submit = gate.poll(kNow, true);
  EXPECT_EQ(submit.kind, StartupProbeActionKind::kSubmit);
  EXPECT_EQ(submit.attempt_generation, 1U);
  EXPECT_EQ(submit.deadline, kNow + 100ms);
  EXPECT_EQ(gate.poll(kNow + 50ms, true).kind, StartupProbeActionKind::kNone);
}

TEST(CoordinatorStartupGate, AppliesCleanCompletionBeforeDeadline)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto submit = gate.poll(kNow, true);

  EXPECT_EQ(
    gate.complete(
      submit.attempt_generation, kNow + 99ms, kNow + 150ms,
      ready_classification()),
    StartupCompletionDisposition::kApplied);
  EXPECT_EQ(gate.snapshot().status, CoordinatorStartupStatus::kReady);
  EXPECT_FALSE(gate.active_deadline());
}

TEST(CoordinatorStartupGate, DeadlineEqualityRetriesAndFencesOldAttempt)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto first = gate.poll(kNow, true);

  EXPECT_EQ(
    gate.complete(
      first.attempt_generation, first.deadline, kNow + 110ms,
      ready_classification()),
    StartupCompletionDisposition::kTimedOut);
  EXPECT_EQ(gate.poll(kNow + 130ms, true).kind, StartupProbeActionKind::kNone);
  const auto second = gate.poll(kNow + 135ms, true);
  ASSERT_EQ(second.kind, StartupProbeActionKind::kSubmit);
  EXPECT_EQ(second.attempt_generation, 2U);
  EXPECT_EQ(
    gate.complete(
      first.attempt_generation, kNow + 90ms, kNow + 136ms,
      ready_classification()),
    StartupCompletionDisposition::kDiscarded);
}

TEST(CoordinatorStartupGate, PollRetiresTimedOutAttemptAndRetriesLater)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto first = gate.poll(kNow, true);

  const auto timeout = gate.poll(first.deadline, true);
  EXPECT_EQ(timeout.kind, StartupProbeActionKind::kRetireTimedOut);
  EXPECT_EQ(timeout.attempt_generation, first.attempt_generation);
  EXPECT_EQ(gate.poll(first.deadline + 24ms, true).kind, StartupProbeActionKind::kNone);
  EXPECT_EQ(gate.poll(first.deadline + 25ms, true).attempt_generation, 2U);
}

TEST(CoordinatorStartupGate, TransportFailureUsesArrivalDeadlinePrecedence)
{
  CoordinatorStartupGate early({100ms, 25ms});
  const auto early_probe = early.poll(kNow, true);
  EXPECT_EQ(
    early.transport_failed(
      early_probe.attempt_generation, kNow + 99ms, kNow + 150ms, "transport exception"),
    StartupCompletionDisposition::kRetryScheduled);
  EXPECT_EQ(early.snapshot().detail, "transport exception");

  CoordinatorStartupGate late({100ms, 25ms});
  const auto late_probe = late.poll(kNow, true);
  EXPECT_EQ(
    late.transport_failed(
      late_probe.attempt_generation, late_probe.deadline, kNow + 150ms,
      "transport exception"),
    StartupCompletionDisposition::kTimedOut);
  EXPECT_NE(late.snapshot().detail, "transport exception");
}

TEST(CoordinatorStartupGate, SubmissionFailureRetiresOnlyCurrentAttempt)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto first = gate.poll(kNow, true);

  EXPECT_FALSE(gate.submission_failed(first.attempt_generation + 1U, kNow, "stale"));
  EXPECT_TRUE(gate.submission_failed(first.attempt_generation, kNow, "transport rejected"));
  EXPECT_EQ(gate.snapshot().status, CoordinatorStartupStatus::kWaitingForAuthority);
  EXPECT_EQ(gate.snapshot().detail, "transport rejected");
}

TEST(CoordinatorStartupGate, RetiresPendingProbeForShutdown)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto first = gate.poll(kNow, true);

  EXPECT_EQ(gate.retire_for_shutdown(), first.attempt_generation);
  EXPECT_FALSE(gate.retire_for_shutdown());
  EXPECT_FALSE(gate.active_deadline());
}

TEST(CoordinatorStartupGate, TerminalAuthorityDecisionCannotBeOverwrittenByTransportFault)
{
  CoordinatorStartupGate gate({100ms, 25ms});
  const auto probe = gate.poll(kNow, true);
  ASSERT_EQ(
    gate.complete(probe.attempt_generation, kNow + 1ms, kNow + 2ms, ready_classification()),
    StartupCompletionDisposition::kApplied);

  EXPECT_FALSE(gate.latch_fault("late transport fault"));
  EXPECT_EQ(gate.snapshot().status, CoordinatorStartupStatus::kReady);
  EXPECT_EQ(gate.snapshot().detail, "clean");
}

TEST(CoordinatorStartupStatus, ReportsOnlyEffectiveAdmissionReadiness)
{
  GoalAdmissionSnapshot admission;
  admission.ready = true;
  EXPECT_TRUE(effective_admission_ready(admission, false));

  admission.inhibited = true;
  EXPECT_FALSE(effective_admission_ready(admission, false));
  admission.inhibited = false;
  EXPECT_FALSE(effective_admission_ready(admission, true));
  admission.ready = false;
  EXPECT_FALSE(effective_admission_ready(admission, false));
}

TEST(CoordinatorStartupClassification, RequiresCleanUnreservedRobotSemantics)
{
  WorldStateSnapshot snapshot;
  EXPECT_EQ(classify_startup_authority(snapshot).status, CoordinatorStartupStatus::kReady);

  snapshot.robot.task_phase = TaskPhase::ValidatingScene;
  EXPECT_EQ(classify_startup_authority(snapshot).status, CoordinatorStartupStatus::kFaulted);
  snapshot.robot.task_phase = TaskPhase::Idle;
  snapshot.robot.fault_state = FaultState::Recoverable;
  EXPECT_EQ(classify_startup_authority(snapshot).status, CoordinatorStartupStatus::kFaulted);
  snapshot.robot.fault_state = FaultState::None;
  snapshot.robot.held_object = restocker_world_state::ObjectId{7U};
  EXPECT_EQ(classify_startup_authority(snapshot).status, CoordinatorStartupStatus::kFaulted);
}

TEST(CoordinatorStartupClassification, RetainsCompleteValidOrphanSummary)
{
  WorldStateSnapshot snapshot;
  TaskReservation reservation;
  reservation.reservation_id = 42U;
  reservation.request_id = "request-42";
  reservation.stage = ReservationStage::Attached;
  reservation.revision = 19U;
  snapshot.active_reservation = reservation;
  snapshot.robot.task_phase = TaskPhase::Recovering;
  snapshot.robot.fault_state = FaultState::Recoverable;

  const auto classified = classify_startup_authority(snapshot);
  ASSERT_EQ(classified.status, CoordinatorStartupStatus::kOrphanedReservation);
  ASSERT_TRUE(classified.orphan);
  EXPECT_EQ(classified.orphan->reservation_id, 42U);
  EXPECT_EQ(classified.orphan->request_id, "request-42");
  EXPECT_EQ(classified.orphan->stage, ReservationStage::Attached);
  EXPECT_EQ(classified.orphan->revision, 19U);
}

TEST(CoordinatorStartupClassification, DecodedAuthorityPreservesEveryValidReservationStage)
{
  for (const auto stage : {
        ReservationStage::Reserved, ReservationStage::Attached,
        ReservationStage::Detached})
  {
    const auto decoded = restocker_world_state::snapshot_from_message(
      orphan_message(stage), "world");
    ASSERT_TRUE(decoded) << decoded.error().detail;
    const auto classified = classify_startup_authority(decoded.value());
    ASSERT_EQ(classified.status, CoordinatorStartupStatus::kOrphanedReservation);
    ASSERT_TRUE(classified.orphan);
    EXPECT_EQ(classified.orphan->stage, stage);
    EXPECT_EQ(classified.orphan->reservation_id, 81U);
    EXPECT_EQ(classified.orphan->request_id, "orphaned-request-81");
  }
}

TEST(CoordinatorStartupClassification, MalformedAuthorityNeverReachesClassification)
{
  auto malformed = orphan_message(ReservationStage::Reserved);
  malformed.header.frame_id = "map";
  EXPECT_FALSE(restocker_world_state::snapshot_from_message(malformed, "world"));

  malformed = orphan_message(ReservationStage::Attached);
  malformed.robot.has_held_object = false;
  EXPECT_FALSE(restocker_world_state::snapshot_from_message(malformed, "world"));
}

TEST(CoordinatorStartupClassification, FaultsInvalidActiveTaskMatrix)
{
  WorldStateSnapshot snapshot;
  snapshot.active_reservation = TaskReservation{};
  snapshot.robot.task_phase = TaskPhase::Executing;
  snapshot.robot.fault_state = FaultState::Recoverable;

  EXPECT_EQ(classify_startup_authority(snapshot).status, CoordinatorStartupStatus::kFaulted);
}

TEST(CoordinatorStartupGeneration, NeverWraps)
{
  EXPECT_EQ(next_startup_attempt(0U), 1U);
  EXPECT_EQ(
    next_startup_attempt(std::numeric_limits<OperationGeneration>::max() - 1U),
    std::numeric_limits<OperationGeneration>::max());
  EXPECT_FALSE(next_startup_attempt(std::numeric_limits<OperationGeneration>::max()));
  EXPECT_EQ(kStartupGoalGeneration, std::numeric_limits<GoalGeneration>::max());
}

}  // namespace
}  // namespace restocker_task_executor
