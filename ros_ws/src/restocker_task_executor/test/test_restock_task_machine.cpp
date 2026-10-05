// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>

#include "restocker_task_executor/restock_task_machine.hpp"

namespace restocker_task_executor
{
namespace
{

using State = RestockTaskState;
using Command = RestockTaskCommand;
using Event = RestockTaskEvent;
using Fault = RestockTaskFault;

RestockTaskTransition succeed(RestockTaskMachine & machine)
{
  const auto state = machine.status().state;
  const bool execution_state =
    state == State::kExecutePreGrasp || state == State::kExecuteApproach ||
    state == State::kExecuteRetract || state == State::kExecuteCarryStart ||
    state == State::kExecutePreInsert ||
    state == State::kExecuteInsert || state == State::kExecuteRetreat;
  if (execution_state && !machine.status().current_execution_may_have_started) {
    const auto generation = static_cast<std::uint64_t>(state) + 1U;
    EXPECT_TRUE(
      machine.dispatch(Event::kExecutionOperationStarted, {}, generation).accepted);
    EXPECT_TRUE(
      machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, generation).accepted);
  }
  return machine.dispatch(Event::kOperationSucceeded);
}

void accept_execution(RestockTaskMachine & machine, std::uint64_t generation)
{
  ASSERT_TRUE(
    machine.dispatch(Event::kExecutionOperationStarted, {}, generation).accepted);
  ASSERT_TRUE(
    machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, generation).accepted);
}

void drive_to(RestockTaskMachine & machine, State target)
{
  ASSERT_TRUE(machine.begin().accepted);
  while (machine.status().state != target && !machine.terminal()) {
    ASSERT_TRUE(succeed(machine).accepted);
  }
  ASSERT_EQ(machine.status().state, target);
}

TEST(RestockTaskMachine, CompletesTheExplicitDeterministicStateSequence)
{
  RestockTaskMachine machine;
  auto transition = machine.begin();
  EXPECT_EQ(transition.state, State::kValidateScene);
  const std::array<State, 31> states{
    State::kSelectPair,
    State::kReserveTask,
    State::kGenerateGrasps,
    State::kPlanPreGrasp,
    State::kExecutePreGrasp,
    State::kOpenGripperForApproach,
    State::kPlanApproach,
    State::kExecuteApproach,
    State::kCloseGripper,
    State::kVerifyGrasp,
    State::kAttachTransaction,
    State::kPlanRetract,
    State::kExecuteRetract,
    State::kObserveDestination,
    State::kGeneratePlacement,
    State::kPlanCarryStart,
    State::kExecuteCarryStart,
    State::kPlanPreInsert,
    State::kExecutePreInsert,
    State::kPlanInsert,
    State::kExecuteInsert,
    State::kOpenGripper,
    State::kDetachTransaction,
    State::kPlanRetreat,
    State::kExecuteRetreat,
    State::kSurveyDestination,
    State::kCommitDetachment,
    State::kVerifyPlacement,
    State::kUpdateInventory,
    // The world state keeps the reservation active until a terminal outcome is recorded.
    State::kReleaseTask,
    State::kComplete};
  for (const auto expected : states) {
    transition = succeed(machine);
    ASSERT_TRUE(transition.accepted);
    EXPECT_EQ(transition.state, expected);
  }
  EXPECT_TRUE(machine.terminal());
  EXPECT_FALSE(machine.status().object_held);
  EXPECT_FALSE(machine.status().reservation_active);
}

// A completed task releases its reservation before completing.
TEST(RestockTaskMachine, ReleasesTheReservationBeforeCompletingRatherThanCanceling)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kUpdateInventory);
  ASSERT_TRUE(machine.status().reservation_active);

  auto transition = succeed(machine);
  ASSERT_TRUE(transition.accepted);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_TRUE(transition.reservation_active);

  transition = succeed(machine);
  ASSERT_TRUE(transition.accepted);
  EXPECT_EQ(transition.state, State::kComplete);
  EXPECT_FALSE(transition.reservation_active);
  // No fault on the release tells the driver to release as succeeded.
  EXPECT_EQ(transition.fault, RestockTaskFault::kNone);
  EXPECT_TRUE(machine.terminal());
}

TEST(RestockTaskMachine, PublishesCommandSpecificSteadyClockTimeouts)
{
  RestockTaskMachine machine;
  auto transition = machine.begin();
  EXPECT_EQ(transition.timeout, std::chrono::milliseconds(3000));
  EXPECT_EQ(transition.total_timeout, std::chrono::milliseconds(120000));
  RestockTaskMachine planning_machine;
  drive_to(planning_machine, State::kPlanPreGrasp);
  EXPECT_EQ(planning_machine.status().timeout, std::chrono::milliseconds(5000));
}

// Both jaw-open states drive the same port command and the same controller goal; the approach
// variant must not fall through to the 3 s validation budget. At 3 s a loaded full suite times
// the open out while its settle verification is still running, the ledger retries into the port's
// still-outstanding goal, and the coordinator inhibits the whole campaign (demonstrated by
// test_autonomous_restock_demo_runtime under full-suite load).
TEST(RestockTaskMachine, BothJawOpenStatesCarryTheExecutionTimeout)
{
  RestockTaskMachine approach_machine;
  drive_to(approach_machine, State::kOpenGripperForApproach);
  EXPECT_EQ(approach_machine.status().command, Command::kOpenGripper);
  EXPECT_EQ(approach_machine.status().timeout, std::chrono::milliseconds(15000));

  RestockTaskMachine release_machine;
  drive_to(release_machine, State::kOpenGripper);
  EXPECT_EQ(release_machine.status().command, Command::kOpenGripper);
  EXPECT_EQ(release_machine.status().timeout, std::chrono::milliseconds(15000));

  RestockTaskMachine close_machine;
  drive_to(close_machine, State::kCloseGripper);
  EXPECT_EQ(close_machine.status().timeout, std::chrono::milliseconds(15000));
}

TEST(RestockTaskMachine, RetriesThenRunsBoundedRecovery)
{
  RestockTaskMachine machine({1U, 2U});
  drive_to(machine, State::kPlanPreGrasp);
  auto transition = machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kPlanPreGrasp);
  EXPECT_EQ(transition.attempt, 2U);
  transition = machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kRecover);
  EXPECT_EQ(transition.command, Command::kExecuteRecovery);
  EXPECT_EQ(transition.recovery_attempt, 1U);
  EXPECT_EQ(succeed(machine).state, State::kPlanPreGrasp);
}

// A straight-line segment depends on the configuration the preceding free-space traverse left the
// arm in, so recovery returns to the traverse, not to the line. Recovery clears the fault; a task
// that recovers and completes must release its reservation as a success (the world state refuses
// a detached, placed reservation released as anything else).
TEST(RestockTaskMachine, SuccessfulRecoveryClearsTheFaultItRecoveredFrom)
{
  RestockTaskMachine machine({0U, 2U});
  drive_to(machine, State::kPlanPreGrasp);
  auto transition = machine.dispatch(Event::kTerminalFailure);
  ASSERT_EQ(transition.state, State::kRecover);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanPreGrasp);
  EXPECT_EQ(transition.fault, Fault::kNone);
}

TEST(RestockTaskMachine, LinearSegmentRecoveryResumesAtTheTraverseBeforeIt)
{
  RestockTaskMachine insert_machine({0U, 1U});
  drive_to(insert_machine, State::kPlanInsert);
  ASSERT_EQ(insert_machine.dispatch(Event::kTerminalFailure).state, State::kRecover);
  EXPECT_EQ(succeed(insert_machine).state, State::kPlanPreInsert);

  RestockTaskMachine approach_machine({0U, 1U});
  drive_to(approach_machine, State::kPlanApproach);
  ASSERT_EQ(approach_machine.dispatch(Event::kTerminalFailure).state, State::kRecover);
  EXPECT_EQ(succeed(approach_machine).state, State::kPlanPreGrasp);
}

TEST(RestockTaskMachine, ExhaustedRecoveryInhibitsAndRequestsOperator)
{
  RestockTaskMachine machine({0U, 1U});
  drive_to(machine, State::kGenerateGrasps);
  ASSERT_EQ(machine.dispatch(Event::kTerminalFailure).state, State::kRecover);
  ASSERT_EQ(succeed(machine).state, State::kGenerateGrasps);
  auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kFault);
  EXPECT_TRUE(machine.motion_inhibited());
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kRequestOperator);
  EXPECT_TRUE(machine.terminal());
  EXPECT_TRUE(machine.motion_inhibited());
}

TEST(RestockTaskMachine, CancelsBeforeMotionOnlyAfterReservationOutcome)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  auto transition = machine.dispatch(Event::kCancelRequested);
  EXPECT_EQ(transition.state, State::kReserveTask);
  EXPECT_TRUE(transition.cancel_requested);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_TRUE(transition.reservation_active);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_FALSE(transition.reservation_active);
}

TEST(RestockTaskMachine, TracksCapabilityBeforeReservationReadback)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);

  auto transition = machine.dispatch(Event::kReservationAcquired);
  EXPECT_EQ(transition.state, State::kReserveTask);
  EXPECT_TRUE(transition.reservation_active);
  EXPECT_FALSE(machine.dispatch(Event::kReservationAcquired).accepted);

  transition = machine.dispatch(Event::kTerminalFailure, "reservation readback failed");
  EXPECT_EQ(transition.state, State::kRecover);
  EXPECT_TRUE(transition.reservation_active);
}

TEST(RestockTaskMachine, SafeAbortReleasesAcquiredReservationWithoutUserCancellation)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  ASSERT_TRUE(machine.dispatch(Event::kReservationAcquired).accepted);

  auto transition = machine.dispatch(Event::kSafeAbortRequested, "validation unavailable");
  EXPECT_EQ(transition.state, State::kReserveTask);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  EXPECT_FALSE(transition.cancel_requested);
  EXPECT_TRUE(transition.safe_abort_requested);
  EXPECT_TRUE(transition.reservation_active);

  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  EXPECT_FALSE(transition.cancel_requested);
  EXPECT_FALSE(transition.reservation_active);
}

TEST(RestockTaskMachine, DrainTerminatesBeforeReservationWithoutUserCancellation)
{
  RestockTaskMachine machine;
  ASSERT_EQ(machine.begin().state, State::kValidateScene);

  auto transition = machine.dispatch(Event::kDrainRequested, "process shutdown");
  ASSERT_TRUE(transition.accepted);
  EXPECT_TRUE(transition.safe_abort_requested);
  EXPECT_FALSE(transition.cancel_requested);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  transition = machine.dispatch(Event::kTerminalFailure, "read-only request retired");
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  EXPECT_FALSE(transition.reservation_active);
}

TEST(RestockTaskMachine, DrainReleasesAReservationAtPreparedExecutionBoundary)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  ASSERT_TRUE(machine.dispatch(Event::kReservationAcquired).accepted);
  auto transition = machine.dispatch(Event::kDrainRequested);
  ASSERT_TRUE(transition.accepted);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);

  RestockTaskMachine after_motion;
  drive_to(after_motion, State::kExecutePreGrasp);
  auto prepared = after_motion.dispatch(Event::kDrainRequested);
  EXPECT_TRUE(prepared.accepted);
  EXPECT_FALSE(prepared.first_trajectory_may_have_started);
  prepared = after_motion.dispatch(Event::kTerminalFailure, "no execution operation to drain");
  EXPECT_EQ(prepared.state, State::kReleaseTask);
}

TEST(RestockTaskMachine, SafeAbortLeavesGenerateGraspsAtItsProvenBoundary)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kGenerateGrasps);

  auto transition = machine.dispatch(Event::kSafeAbortRequested);
  EXPECT_EQ(transition.state, State::kGenerateGrasps);
  EXPECT_TRUE(transition.safe_abort_requested);
  EXPECT_FALSE(transition.first_trajectory_may_have_started);
  EXPECT_FALSE(transition.object_held);
  EXPECT_EQ(succeed(machine).state, State::kReleaseTask);
}

TEST(RestockTaskMachine, SafeAbortRejectsUnsafeOrAuthorityFreePhases)
{
  RestockTaskMachine before_reservation;
  ASSERT_EQ(before_reservation.begin().state, State::kValidateScene);
  EXPECT_FALSE(before_reservation.dispatch(Event::kSafeAbortRequested).accepted);

  RestockTaskMachine prepared_execution;
  drive_to(prepared_execution, State::kExecutePreGrasp);
  EXPECT_TRUE(prepared_execution.dispatch(Event::kSafeAbortRequested).accepted);
  EXPECT_EQ(
    prepared_execution.dispatch(Event::kTerminalFailure).state,
    State::kReleaseTask);

  RestockTaskMachine releasing;
  drive_to(releasing, State::kGenerateGrasps);
  ASSERT_TRUE(releasing.dispatch(Event::kSafeAbortRequested).accepted);
  ASSERT_EQ(succeed(releasing).state, State::kReleaseTask);
  EXPECT_FALSE(releasing.dispatch(Event::kSafeAbortRequested).accepted);
  ASSERT_EQ(succeed(releasing).state, State::kCanceled);
  EXPECT_FALSE(releasing.dispatch(Event::kSafeAbortRequested).accepted);
}

TEST(RestockTaskMachine, WholeTaskDeadlineTakesPrecedenceOverSafeAbort)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kGenerateGrasps);
  ASSERT_TRUE(machine.dispatch(Event::kSafeAbortRequested).accepted);

  auto transition = machine.dispatch(Event::kTaskDeadlineExceeded);
  EXPECT_EQ(transition.fault, Fault::kTimedOut);
  EXPECT_TRUE(transition.safe_abort_requested);
  EXPECT_TRUE(transition.cancel_requested);
  ASSERT_EQ(succeed(machine).state, State::kReleaseTask);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.fault, Fault::kTimedOut);
}

TEST(RestockTaskMachine, SafeAbortPreventsRetryAfterReservationBoundaryFailure)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  ASSERT_TRUE(machine.dispatch(Event::kReservationAcquired).accepted);
  ASSERT_TRUE(machine.dispatch(Event::kSafeAbortRequested).accepted);

  const auto transition = machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_EQ(transition.attempt, 1U);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  EXPECT_TRUE(transition.safe_abort_requested);
}

TEST(RestockTaskMachine, SafeAbortPreventsRetryAfterPreGraspPlanningFailure)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kPlanPreGrasp);
  ASSERT_TRUE(machine.dispatch(Event::kSafeAbortRequested).accepted);

  const auto transition = machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_EQ(transition.attempt, 1U);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
  EXPECT_TRUE(transition.safe_abort_requested);
}

TEST(RestockTaskMachine, PreparedPreGraspExecutionHasNoMotionEvidence)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);

  const auto prepared = machine.status();
  EXPECT_FALSE(prepared.first_trajectory_may_have_started);
  EXPECT_FALSE(prepared.current_execution_may_have_started);
  EXPECT_EQ(prepared.current_execution_operation_generation, 0U);
  EXPECT_FALSE(machine.dispatch(Event::kOperationSucceeded).accepted);
}

TEST(RestockTaskMachine, PreparedPreGraspCancellationReleasesWithoutMotionCancellation)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);

  auto transition = machine.dispatch(Event::kCancelRequested);
  EXPECT_EQ(transition.state, State::kExecutePreGrasp);
  EXPECT_EQ(transition.command, Command::kExecutePreGrasp);
  transition = machine.dispatch(Event::kTerminalFailure, "no execution operation was submitted");
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_FALSE(transition.first_trajectory_may_have_started);
}

TEST(RestockTaskMachine, CancellationPreventsAFirstExecutionSubmission)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);
  ASSERT_TRUE(machine.dispatch(Event::kCancelRequested).accepted);

  EXPECT_FALSE(
    machine.dispatch(Event::kExecutionOperationStarted, {}, 53U).accepted);
  EXPECT_FALSE(machine.status().first_trajectory_may_have_started);
}

TEST(RestockTaskMachine, LateAcceptanceAfterCancellationRetainsConservativeMotionEvidence)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);
  ASSERT_TRUE(
    machine.dispatch(Event::kExecutionOperationStarted, {}, 54U).accepted);
  ASSERT_TRUE(machine.dispatch(Event::kCancelRequested).accepted);

  const auto accepted =
    machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, 54U);
  EXPECT_TRUE(accepted.accepted);
  EXPECT_TRUE(accepted.first_trajectory_may_have_started);
  EXPECT_TRUE(accepted.current_execution_may_have_started);
}

TEST(RestockTaskMachine, PreparedPreGraspDeadlineReleasesWithoutMotionCancellation)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);

  auto transition = machine.dispatch(Event::kTaskDeadlineExceeded);
  EXPECT_EQ(transition.state, State::kExecutePreGrasp);
  EXPECT_EQ(transition.fault, Fault::kTimedOut);
  transition = machine.dispatch(Event::kTimeout, "prepared boundary deadline observed");
  EXPECT_EQ(transition.state, State::kReleaseTask);
  EXPECT_FALSE(transition.current_execution_may_have_started);
}

TEST(RestockTaskMachine, ExecutionAcceptanceIsExactAndPerOperation)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);

  EXPECT_FALSE(
    machine.dispatch(Event::kExecutionOperationStarted, {}, 0U).accepted);
  ASSERT_TRUE(
    machine.dispatch(Event::kExecutionOperationStarted, {}, 51U).accepted);
  EXPECT_FALSE(
    machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, 52U).accepted);
  auto accepted = machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, 51U);
  ASSERT_TRUE(accepted.accepted);
  EXPECT_TRUE(accepted.first_trajectory_may_have_started);
  EXPECT_TRUE(accepted.current_execution_may_have_started);
  EXPECT_EQ(accepted.current_execution_operation_generation, 51U);
  EXPECT_FALSE(
    machine.dispatch(Event::kTrajectoryExecutionAccepted, {}, 51U).accepted);

  ASSERT_EQ(
    machine.dispatch(Event::kOperationSucceeded).state, State::kOpenGripperForApproach);
  ASSERT_EQ(succeed(machine).state, State::kPlanApproach);
  ASSERT_EQ(succeed(machine).state, State::kExecuteApproach);
  EXPECT_FALSE(machine.status().current_execution_may_have_started);
  EXPECT_EQ(machine.status().current_execution_operation_generation, 0U);
  EXPECT_FALSE(machine.dispatch(Event::kOperationSucceeded).accepted);
}

TEST(RestockTaskMachine, ActiveFreeSpaceCancellationVerifiesStopAndRetreat)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);
  accept_execution(machine, 41U);
  auto transition = machine.dispatch(Event::kCancelRequested);
  EXPECT_EQ(transition.state, State::kCancelActiveMotion);
  EXPECT_EQ(transition.command, Command::kCancelMotionAndVerifyStop);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
  ASSERT_EQ(succeed(machine).state, State::kExecuteRetreat);
  ASSERT_EQ(succeed(machine).state, State::kReleaseTask);
  EXPECT_EQ(succeed(machine).state, State::kCanceled);
}

TEST(RestockTaskMachine, ExecutionFailureVerifiesStopThenReplans)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecuteApproach);
  accept_execution(machine, 42U);
  auto transition = machine.dispatch(Event::kTimeout);
  EXPECT_EQ(transition.state, State::kCancelActiveMotion);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kRecover);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanApproach);
}

TEST(RestockTaskMachine, WholeTaskDeadlineStopsMotionThenRetreats)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecutePreGrasp);
  accept_execution(machine, 43U);
  auto transition = machine.dispatch(Event::kTaskDeadlineExceeded);
  EXPECT_EQ(transition.state, State::kCancelActiveMotion);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
  ASSERT_EQ(succeed(machine).state, State::kExecuteRetreat);
  ASSERT_EQ(succeed(machine).state, State::kReleaseTask);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.fault, Fault::kTimedOut);
}

TEST(RestockTaskMachine, WholeTaskDeadlineLetsChildTransactionReconcile)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kAttachTransaction);
  auto transition = machine.dispatch(Event::kTaskDeadlineExceeded);
  EXPECT_EQ(transition.state, State::kAttachTransaction);
  EXPECT_TRUE(transition.cancel_requested);
  transition = machine.dispatch(Event::kTransactionRolledBack);
  // Card 062: the rolled-back attach left the jaws closed on a product that is not held, so the
  // deadline's cleanup retreat opens them first.
  EXPECT_EQ(transition.state, State::kOpenGripperForEscape);
  EXPECT_EQ(transition.fault, Fault::kTimedOut);
  EXPECT_EQ(succeed(machine).state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, ConstrainedMotionCancellationWaitsForEndpoint)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kExecuteApproach);
  accept_execution(machine, 44U);
  auto transition = machine.dispatch(Event::kCancelRequested);
  EXPECT_EQ(transition.state, State::kExecuteApproach);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, AttachmentCancellationRetractsWithHeldObject)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kAttachTransaction);
  ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kAttachTransaction);
  auto transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetract);
  EXPECT_TRUE(transition.object_held);
  ASSERT_EQ(succeed(machine).state, State::kExecuteRetract);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, DetachmentCancellationStillRetreatsToSurvey)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kDetachTransaction);
  ASSERT_TRUE(machine.status().object_held);
  ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kDetachTransaction);
  auto transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
  EXPECT_FALSE(transition.object_held);
  ASSERT_EQ(succeed(machine).state, State::kExecuteRetreat);
  EXPECT_EQ(succeed(machine).state, State::kSurveyDestination);
}

TEST(RestockTaskMachine, OpenGripperCancellationCannotSkipDetachment)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kOpenGripper);
  ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kOpenGripper);
  EXPECT_EQ(succeed(machine).state, State::kDetachTransaction);
}

TEST(RestockTaskMachine, CancellationDuringRecoveryWaitsForRecoveryResult)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kVerifyGrasp);
  ASSERT_EQ(machine.dispatch(Event::kTerminalFailure).state, State::kRecover);
  ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kRecover);
  const auto transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kPlanRetreat);
  EXPECT_EQ(transition.fault, Fault::kCanceled);
}

TEST(RestockTaskMachine, CancellationDuringPreReservationRecoveryNeedsNoRelease)
{
  RestockTaskMachine machine;
  ASSERT_EQ(machine.begin().state, State::kValidateScene);
  ASSERT_EQ(
    machine.dispatch(Event::kTerminalFailure, "snapshot unavailable").state,
    State::kRecover);
  ASSERT_FALSE(machine.status().reservation_active);
  ASSERT_FALSE(machine.status().first_trajectory_may_have_started);
  ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kRecover);
  const auto transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCanceled);
  EXPECT_EQ(transition.command, Command::kNone);
  EXPECT_EQ(transition.fault, Fault::kCanceled);
  EXPECT_FALSE(transition.reservation_active);
}

TEST(RestockTaskMachine, RolledBackTransactionsPreserveHeldIdentity)
{
  RestockTaskMachine attach_machine;
  drive_to(attach_machine, State::kAttachTransaction);
  auto transition = attach_machine.dispatch(Event::kTransactionRolledBack);
  EXPECT_EQ(transition.state, State::kRecover);
  EXPECT_FALSE(transition.object_held);
  EXPECT_EQ(succeed(attach_machine).state, State::kOpenGripperForEscape);
  EXPECT_EQ(succeed(attach_machine).state, State::kPlanPreGrasp);

  RestockTaskMachine detach_machine;
  drive_to(detach_machine, State::kDetachTransaction);
  transition = detach_machine.dispatch(Event::kTransactionRolledBack);
  EXPECT_EQ(transition.state, State::kRecover);
  EXPECT_TRUE(transition.object_held);
}

TEST(RestockTaskMachine, InhibitedChildTransactionFaultsWithoutRetry)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kAttachTransaction);
  auto transition = machine.dispatch(Event::kTransactionInhibited);
  EXPECT_EQ(transition.state, State::kFault);
  EXPECT_EQ(transition.fault, Fault::kExternalInconsistency);
  EXPECT_TRUE(machine.motion_inhibited());
}

TEST(RestockTaskMachine, RejectsInvalidConfigAndTransactionEventsOutsideTransactions)
{
  RestockTaskConfig config;
  config.recovery_timeout = std::chrono::milliseconds(0);
  RestockTaskMachine invalid(config);
  auto transition = invalid.begin();
  EXPECT_EQ(transition.state, State::kFault);
  EXPECT_EQ(transition.fault, Fault::kInvalidConfiguration);

  RestockTaskMachine machine;
  ASSERT_TRUE(machine.begin().accepted);
  transition = machine.dispatch(Event::kTransactionRolledBack);
  EXPECT_FALSE(transition.accepted);
  EXPECT_EQ(transition.fault, Fault::kInvalidEvent);
}

TEST(RestockTaskMachine, ReSelectsBoundedlyWhenReservationPreconditionsAreSuperseded)
{
  RestockTaskConfig config;
  config.max_selection_restarts = 2U;
  RestockTaskMachine machine(config);
  drive_to(machine, State::kReserveTask);
  EXPECT_EQ(machine.selection_restarts(), 0U);

  for (std::size_t restart = 1U; restart <= config.max_selection_restarts; ++restart) {
    const auto superseded = machine.dispatch(Event::kSelectionSuperseded);
    EXPECT_TRUE(superseded.accepted);
    EXPECT_EQ(superseded.state, State::kValidateScene);
    EXPECT_EQ(superseded.command, Command::kValidateScene);
    EXPECT_EQ(superseded.fault, Fault::kNone);
    EXPECT_FALSE(superseded.reservation_active);
    EXPECT_EQ(machine.selection_restarts(), restart);
    while (machine.status().state != State::kReserveTask) {
      ASSERT_TRUE(succeed(machine).accepted);
    }
  }

  // An endlessly superseded selection is an unstable world, so the retry bound must hold.
  const auto exhausted = machine.dispatch(Event::kSelectionSuperseded);
  EXPECT_TRUE(exhausted.accepted);
  EXPECT_EQ(exhausted.state, State::kFault);
  EXPECT_EQ(exhausted.fault, Fault::kRetryExhausted);
  EXPECT_TRUE(machine.motion_inhibited());
}

TEST(RestockTaskMachine, RejectsSupersededSelectionOnceAnythingIsCommitted)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  ASSERT_TRUE(machine.dispatch(Event::kReservationAcquired).accepted);

  const auto rejected = machine.dispatch(Event::kSelectionSuperseded);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(rejected.fault, Fault::kInvalidEvent);
  EXPECT_EQ(machine.status().state, State::kReserveTask);
  EXPECT_EQ(machine.selection_restarts(), 0U);

  RestockTaskMachine idle;
  ASSERT_TRUE(idle.begin().accepted);
  EXPECT_FALSE(idle.dispatch(Event::kSelectionSuperseded).accepted);
}

TEST(RestockTaskMachine, EndsSupersededSelectionCleanlyAfterTermination)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kReserveTask);
  ASSERT_TRUE(machine.dispatch(Event::kCancelRequested).accepted);

  const auto canceled = machine.dispatch(Event::kSelectionSuperseded);
  EXPECT_TRUE(canceled.accepted);
  EXPECT_EQ(canceled.state, State::kCanceled);
  EXPECT_EQ(canceled.fault, Fault::kCanceled);
  EXPECT_TRUE(machine.terminal());
}

TEST(RestockTaskMachine, SupersedesSelectionAtSelectPairBeforeCommitment)
{
  // Selection-time evidence aging: kSelectPair is pre-commitment, so a superseded selection
  // there re-observes on the same bounded restart counter as reserve-time supersession.
  RestockTaskMachine machine;
  drive_to(machine, State::kSelectPair);
  EXPECT_EQ(machine.selection_restarts(), 0U);

  const auto superseded = machine.dispatch(Event::kSelectionSuperseded);
  EXPECT_TRUE(superseded.accepted);
  EXPECT_EQ(superseded.state, State::kValidateScene);
  EXPECT_EQ(superseded.command, Command::kValidateScene);
  EXPECT_EQ(superseded.fault, Fault::kNone);
  EXPECT_FALSE(superseded.reservation_active);
  EXPECT_EQ(machine.selection_restarts(), 1U);
}

// Milestone 10 §6 (Card 051): under the whole-task deadline an arm that already moved takes the
// bounded cleanup retreat before the reservation is released; before this contract the machine
// released in place. A user cancel keeps stopping in place — the retreat is commandable only
// under the kTaskDeadline termination.
TEST(RestockTaskMachine, WholeTaskDeadlineRetreatsAnArmThatMovedBeforeReleasing)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kPlanApproach);
  ASSERT_TRUE(machine.status().first_trajectory_may_have_started);

  // A plan refusal at the approach enters bounded recovery, which resumes at plan_pre_grasp.
  ASSERT_TRUE(
    machine.dispatch(Event::kTerminalFailure, "approach plan refused").accepted);
  ASSERT_EQ(machine.status().state, State::kRecover);
  ASSERT_TRUE(succeed(machine).accepted);
  ASSERT_EQ(machine.status().state, State::kPlanPreGrasp);

  const auto expired = machine.dispatch(Event::kTaskDeadlineExceeded, "deadline");
  ASSERT_TRUE(expired.accepted);
  ASSERT_TRUE(expired.cancel_requested);
  ASSERT_TRUE(expired.task_deadline_exceeded);
  EXPECT_EQ(expired.state, State::kPlanPreGrasp);

  // The plan-stage short-circuit reports success without motion; the termination routes the
  // moved arm through the retreat (this is the machine half of the guard recognition).
  const auto after = succeed(machine);
  EXPECT_EQ(after.state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, UserCancelReleasesWithoutRetreating)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kPlanApproach);
  ASSERT_TRUE(machine.status().first_trajectory_may_have_started);
  ASSERT_TRUE(
    machine.dispatch(Event::kTerminalFailure, "approach plan refused").accepted);
  ASSERT_TRUE(succeed(machine).accepted);
  ASSERT_EQ(machine.status().state, State::kPlanPreGrasp);

  const auto canceled = machine.dispatch(Event::kCancelRequested, "user cancel");
  ASSERT_TRUE(canceled.accepted);
  ASSERT_TRUE(canceled.cancel_requested);
  ASSERT_FALSE(canceled.task_deadline_exceeded);

  const auto after = succeed(machine);
  EXPECT_EQ(after.state, State::kReleaseTask)
    << "a user cancel keeps stopping in place: release, no retreat";
}

// Milestone 10 §6 rung 5 (Card 051): the recoverable skip is requested from the fault boundary,
// routes through the cleanup, is single-shot, and keeps the recorded fault so the terminal can
// deliver the typed outcome instead of a kCanceled verdict.
TEST(RestockTaskMachine, RecoverableSkipReleasesAReservationHeldPreMotion)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kGenerateGrasps);
  ASSERT_TRUE(machine.status().reservation_active);
  ASSERT_TRUE(machine.dispatch(Event::kTerminalFailure, "generation refused").accepted);
  ASSERT_EQ(machine.status().state, State::kRecover);
  ASSERT_TRUE(machine.dispatch(Event::kTimeout, "recovery stalled").accepted);
  ASSERT_EQ(machine.status().state, State::kFault);
  const auto fault_before = machine.status().fault;

  const auto skip = machine.dispatch(Event::kRecoverableSkipRequested, "rung 5");
  ASSERT_TRUE(skip.accepted);
  EXPECT_TRUE(skip.recoverable_skip);
  EXPECT_EQ(skip.state, State::kReleaseTask);
  EXPECT_NE(skip.fault, Fault::kCanceled) << "the recorded fault must survive for the terminal";

  // Single-shot: a second request is refused, so the ladder always terminates.
  const auto again = machine.dispatch(Event::kRecoverableSkipRequested, "rung 5 again");
  EXPECT_FALSE(again.accepted);
  EXPECT_NE(
    again.detail.find("already attempted"), std::string::npos) << again.detail;

  ASSERT_EQ(
    machine.dispatch(Event::kOperationSucceeded, "reservation released").state,
    State::kCanceled);
  EXPECT_EQ(machine.status().fault, fault_before)
    << "kCanceled must not overwrite the fault the skip was requested from";
}

TEST(RestockTaskMachine, RecoverableSkipRetreatsAnArmThatMoved)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kPlanApproach);
  ASSERT_TRUE(machine.status().first_trajectory_may_have_started);
  ASSERT_TRUE(machine.dispatch(Event::kTerminalFailure, "approach refused").accepted);
  ASSERT_TRUE(machine.dispatch(Event::kTimeout, "recovery stalled").accepted);
  ASSERT_EQ(machine.status().state, State::kFault);

  const auto skip = machine.dispatch(Event::kRecoverableSkipRequested, "rung 5");
  ASSERT_TRUE(skip.accepted);
  EXPECT_EQ(skip.state, State::kPlanRetreat);
  EXPECT_TRUE(skip.recoverable_skip);

  // The termination routing engages for the skip: retreat executes, then the cleanup walks on.
  const auto at_execute = succeed(machine);
  EXPECT_EQ(at_execute.state, State::kExecuteRetreat);
}

TEST(RestockTaskMachine, RecoverableSkipEndsDirectlyWhenNothingWasReservedOrMoved)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kSelectPair);
  ASSERT_TRUE(machine.dispatch(Event::kTerminalFailure, "selection refused").accepted);
  ASSERT_TRUE(machine.dispatch(Event::kTimeout, "recovery stalled").accepted);
  ASSERT_EQ(machine.status().state, State::kFault);

  const auto skip = machine.dispatch(Event::kRecoverableSkipRequested, "rung 5");
  ASSERT_TRUE(skip.accepted);
  EXPECT_EQ(skip.state, State::kCanceled);
  EXPECT_NE(skip.fault, Fault::kCanceled);
}

TEST(RestockTaskMachine, RecoverableSkipIsRefusedOutsideTheFaultBoundary)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kGenerateGrasps);
  const auto refused = machine.dispatch(Event::kRecoverableSkipRequested, "too early");
  EXPECT_FALSE(refused.accepted);
  EXPECT_EQ(machine.status().state, State::kGenerateGrasps);
}

TEST(RestockTaskMachine, ExposesStableDiagnosticNames)
{
  EXPECT_STREQ(to_string(State::kObserveDestination), "observe_destination");
  EXPECT_STREQ(to_string(Command::kRunDetachTransaction), "run_detach_transaction");
  EXPECT_STREQ(to_string(Fault::kRecoveryFailed), "recovery_failed");
  EXPECT_STREQ(to_string(State::kOpenGripperForEscape), "open_gripper_for_escape");
}

// Milestone 10 §6 (Card 062): a grasp that closed the jaws and then failed must open them before
// any plan leaves the grasp — Card 010 SC-004 slots 16/21 re-planned from closed jaws around an
// unheld product until every start state was in collision.
TEST(RestockTaskMachine, RolledBackAttachOpensTheJawsBeforeReplanningThePreGrasp)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kAttachTransaction);
  ASSERT_EQ(machine.dispatch(Event::kTransactionRolledBack).state, State::kRecover);
  const auto open = succeed(machine);
  EXPECT_EQ(open.state, State::kOpenGripperForEscape);
  EXPECT_EQ(open.command, Command::kOpenGripper);
  EXPECT_EQ(open.timeout, std::chrono::milliseconds(15000));
  EXPECT_FALSE(open.object_held);
  EXPECT_NE(open.detail.find("grasp escape"), std::string::npos);
  EXPECT_EQ(succeed(machine).state, State::kPlanPreGrasp);
  // Once open, the retry walks the ordinary sequence: no second escape before the next close.
  EXPECT_EQ(succeed(machine).state, State::kExecutePreGrasp);
  EXPECT_EQ(succeed(machine).state, State::kOpenGripperForApproach);
}

TEST(RestockTaskMachine, FailedCloseOrVerifyOpensTheJawsBeforeRegenerating)
{
  for (const auto failing : {State::kCloseGripper, State::kVerifyGrasp}) {
    RestockTaskMachine machine;
    drive_to(machine, failing);
    ASSERT_EQ(
      machine.dispatch(Event::kTerminalFailure, "grasp refused").state, State::kRecover);
    EXPECT_EQ(succeed(machine).state, State::kOpenGripperForEscape);
    EXPECT_EQ(succeed(machine).state, State::kGenerateGrasps);
  }
}

TEST(RestockTaskMachine, RecoverableSkipFromAFailedGraspOpensTheJawsBeforeTheCleanupRetreat)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kVerifyGrasp);
  ASSERT_TRUE(machine.dispatch(Event::kTerminalFailure, "grasp refused").accepted);
  ASSERT_TRUE(machine.dispatch(Event::kTimeout, "recovery stalled").accepted);
  ASSERT_EQ(machine.status().state, State::kFault);
  const auto skip = machine.dispatch(Event::kRecoverableSkipRequested, "rung 5");
  ASSERT_TRUE(skip.accepted);
  EXPECT_EQ(skip.state, State::kOpenGripperForEscape);
  EXPECT_TRUE(skip.recoverable_skip);
  EXPECT_EQ(succeed(machine).state, State::kPlanRetreat);
  EXPECT_EQ(succeed(machine).state, State::kExecuteRetreat);
}

TEST(RestockTaskMachine, DeadlineDuringTheEscapeOpenStillOwesTheCleanupRetreat)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kAttachTransaction);
  ASSERT_EQ(machine.dispatch(Event::kTransactionRolledBack).state, State::kRecover);
  ASSERT_EQ(succeed(machine).state, State::kOpenGripperForEscape);
  ASSERT_EQ(
    machine.dispatch(Event::kTaskDeadlineExceeded).state, State::kOpenGripperForEscape);
  EXPECT_EQ(succeed(machine).state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, UserCancelAtAGraspAddsNoJawMotion)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kVerifyGrasp);
  ASSERT_TRUE(machine.dispatch(Event::kCancelRequested).accepted);
  ASSERT_EQ(succeed(machine).state, State::kRecover);
  // An operator's stop keeps stopping in place: the retreat the driver short-circuits, no open.
  EXPECT_EQ(succeed(machine).state, State::kPlanRetreat);
}

TEST(RestockTaskMachine, AHeldProductNeverTakesTheGraspEscape)
{
  RestockTaskMachine machine;
  drive_to(machine, State::kPlanRetract);
  ASSERT_TRUE(machine.status().object_held);
  ASSERT_TRUE(machine.dispatch(Event::kCancelRequested).accepted);
  ASSERT_EQ(succeed(machine).state, State::kExecuteRetract);
  EXPECT_EQ(succeed(machine).state, State::kPlanRetreat);
  // Nor does the ordinary post-detach retreat.
  RestockTaskMachine full;
  drive_to(full, State::kDetachTransaction);
  EXPECT_EQ(succeed(full).state, State::kPlanRetreat);
  EXPECT_FALSE(full.status().object_held);
}

}  // namespace
}  // namespace restocker_task_executor
