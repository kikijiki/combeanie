// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <vector>

#include "restocker_task_executor/attachment_transaction_machine.hpp"

namespace restocker_task_executor
{
namespace
{

using Event = AttachmentTransactionEvent;
using State = AttachmentTransactionState;
using Command = AttachmentTransactionCommand;
using Fault = AttachmentTransactionFault;

AttachmentTransactionTransition succeed(AttachmentTransactionMachine & machine)
{
  return machine.dispatch(Event::kOperationSucceeded);
}

void drive_to(
  AttachmentTransactionMachine & machine, State target,
  AttachmentTransactionKind kind = AttachmentTransactionKind::kAttach)
{
  ASSERT_TRUE(machine.begin(kind).accepted);
  while (machine.status().state != target && !machine.terminal()) {
    ASSERT_TRUE(succeed(machine).accepted);
  }
  ASSERT_EQ(machine.status().state, target);
}

TEST(AttachmentTransactionMachine, DetachProvesPlacementBeforeMoveItAndSemanticCommit)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kProvePlacement, AttachmentTransactionKind::kDetach);
  EXPECT_EQ(machine.status().command, Command::kVerifySettledPlacement);
  EXPECT_EQ(succeed(machine).state, State::kApplyMoveItDiff);

  AttachmentTransactionMachine failed_proof;
  drive_to(failed_proof, State::kProvePlacement, AttachmentTransactionKind::kDetach);
  const auto transition = failed_proof.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kCompensateGazebo);
  EXPECT_EQ(transition.command, Command::kSendInverseGazeboCommand);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
}

TEST(AttachmentTransactionMachine, DetachPlacementProofCancellationRestoresAttachment)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kProvePlacement, AttachmentTransactionKind::kDetach);
  EXPECT_EQ(machine.dispatch(Event::kCancelRequested).state, State::kProvePlacement);
  const auto transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCompensateGazebo);
  EXPECT_EQ(transition.fault, Fault::kCanceled);
}

TEST(AttachmentTransactionMachine, CompletesAttachAndDetachThroughExplicitCommands)
{
  for (const auto kind :
    {AttachmentTransactionKind::kAttach, AttachmentTransactionKind::kDetach})
  {
    AttachmentTransactionMachine machine;
    auto transition = machine.begin(kind);
    EXPECT_EQ(transition.command, Command::kAcquireSceneLease);
    EXPECT_EQ(transition.timeout, std::chrono::milliseconds(5000));
    const std::vector<State> attach_states{
      State::kCommandGazebo,
      State::kReconcileGazeboOutcome,
      State::kApplyMoveItDiff,
      State::kVerifyMoveItDiff,
      State::kCommitWorldState,
      State::kReleaseSceneLease,
      State::kVerifyProjection,
      State::kSucceeded};
    const std::vector<State> detach_states{
      State::kCommandGazebo,
      State::kReconcileGazeboOutcome,
      State::kProvePlacement,
      State::kApplyMoveItDiff,
      State::kVerifyMoveItDiff,
      State::kCommitWorldState,
      State::kReleaseSceneLease,
      State::kVerifyProjection,
      State::kSucceeded};
    const auto & expected_states = kind == AttachmentTransactionKind::kAttach ?
      attach_states : detach_states;
    for (const auto expected : expected_states) {
      transition = succeed(machine);
      ASSERT_TRUE(transition.accepted);
      EXPECT_EQ(transition.state, expected);
    }
    EXPECT_TRUE(machine.terminal());
    EXPECT_FALSE(machine.motion_inhibited());
    EXPECT_EQ(machine.kind(), kind);
  }
}

TEST(AttachmentTransactionMachine, ReconcilesUnknownExternalOutcomesWithoutBlindReplay)
{
  AttachmentTransactionMachine machine;
  ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(succeed(machine).state, State::kCommandGazebo);
  auto transition = machine.dispatch(Event::kTimeout);
  EXPECT_EQ(transition.state, State::kReconcileGazeboOutcome);
  EXPECT_EQ(transition.command, Command::kQueryGazeboOutcome);
  EXPECT_EQ(transition.timeout, std::chrono::milliseconds(10000));
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kApplyMoveItDiff);

  transition = machine.dispatch(Event::kOutcomeUnknown);
  EXPECT_EQ(transition.state, State::kVerifyMoveItDiff);
  EXPECT_EQ(transition.command, Command::kReadBackMoveItScene);
  ASSERT_EQ(succeed(machine).state, State::kCommitWorldState);

  transition = machine.dispatch(Event::kTimeout);
  EXPECT_EQ(transition.state, State::kReconcileCommittedState);
  EXPECT_EQ(transition.command, Command::kReconcileCommittedState);
  EXPECT_EQ(succeed(machine).state, State::kReleaseSceneLease);
}

TEST(AttachmentTransactionMachine, ReconcilesLateSceneLeaseAcquisitionBeforeMutation)
{
  AttachmentTransactionMachine machine;
  ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
  auto transition = machine.dispatch(Event::kTimeout);
  EXPECT_EQ(transition.state, State::kReconcileSceneLeaseOutcome);
  EXPECT_EQ(transition.command, Command::kQuerySceneLeaseOutcome);
  EXPECT_EQ(transition.timeout, std::chrono::milliseconds(10000));
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCommandGazebo);

  AttachmentTransactionMachine not_acquired;
  ASSERT_TRUE(not_acquired.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(
    not_acquired.dispatch(Event::kOutcomeUnknown).state,
    State::kReconcileSceneLeaseOutcome);
  transition = not_acquired.dispatch(Event::kConfirmedNotApplied);
  EXPECT_EQ(transition.state, State::kRolledBack);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);

  AttachmentTransactionMachine canceled_late_grant;
  ASSERT_TRUE(canceled_late_grant.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(
    canceled_late_grant.dispatch(Event::kTimeout).state,
    State::kReconcileSceneLeaseOutcome);
  ASSERT_EQ(
    canceled_late_grant.dispatch(Event::kCancelRequested).state,
    State::kReconcileSceneLeaseOutcome);
  transition = succeed(canceled_late_grant);
  EXPECT_EQ(transition.state, State::kReleaseCompensationLease);
  EXPECT_EQ(transition.fault, Fault::kCanceled);

  AttachmentTransactionMachine lost_lease_outcome({0U, 1U});
  ASSERT_TRUE(lost_lease_outcome.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(
    lost_lease_outcome.dispatch(Event::kTimeout).state,
    State::kReconcileSceneLeaseOutcome);
  transition = lost_lease_outcome.dispatch(Event::kOutcomeUnknown);
  EXPECT_EQ(transition.state, State::kInhibitMotion);
  EXPECT_EQ(transition.fault, Fault::kOutcomeUnknown);
}

TEST(AttachmentTransactionMachine, UnknownMoveItApplyIsRemovedDuringRollback)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kApplyMoveItDiff);
  ASSERT_EQ(machine.dispatch(Event::kTimeout).state, State::kVerifyMoveItDiff);
  ASSERT_EQ(
    machine.dispatch(Event::kTerminalFailure).state,
    State::kCompensateGazebo);
  EXPECT_EQ(succeed(machine).state, State::kCompensateMoveIt);
}

TEST(AttachmentTransactionMachine, RollsBackBeforePhysicalMutation)
{
  AttachmentTransactionMachine machine;
  ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(succeed(machine).state, State::kCommandGazebo);
  auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kReleaseCompensationLease);
  EXPECT_EQ(transition.command, Command::kReleaseCompensationLease);
  EXPECT_EQ(transition.timeout, std::chrono::milliseconds(10000));
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kVerifyCompensationProjection);
  EXPECT_EQ(transition.command, Command::kWaitForCompensationProjection);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kRolledBack);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
}

TEST(AttachmentTransactionMachine, RejectedLeaseAcquisitionNeedsNoCompensationRelease)
{
  AttachmentTransactionMachine machine;
  ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
  const auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kRolledBack);
  EXPECT_EQ(transition.command, Command::kNone);
  EXPECT_EQ(transition.fault, Fault::kOperationFailed);
}

TEST(AttachmentTransactionMachine, CompensatesPhysicalThenGeometricMutation)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kCommitWorldState);
  auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kCompensateGazebo);
  EXPECT_EQ(transition.command, Command::kSendInverseGazeboCommand);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kCompensateMoveIt);
  EXPECT_EQ(transition.command, Command::kApplyInverseMoveItDiff);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kReleaseCompensationLease);
  EXPECT_EQ(succeed(machine).state, State::kVerifyCompensationProjection);
  EXPECT_EQ(succeed(machine).state, State::kRolledBack);
}

TEST(AttachmentTransactionMachine, CompensationUncertaintyAlwaysInhibitsMotion)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kCommitWorldState);
  ASSERT_EQ(
    machine.dispatch(Event::kTerminalFailure).state,
    State::kCompensateGazebo);
  const auto transition = machine.dispatch(Event::kOutcomeUnknown);
  EXPECT_EQ(transition.state, State::kInhibitMotion);
  EXPECT_EQ(transition.command, Command::kInhibitMotion);
  EXPECT_EQ(transition.fault, Fault::kCompensationFailed);
  EXPECT_TRUE(machine.motion_inhibited());
}

TEST(AttachmentTransactionMachine, ReconcilesFailuresAfterSemanticCommit)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kReleaseSceneLease);
  auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kReconcileCommittedState);
  EXPECT_EQ(transition.command, Command::kReconcileCommittedState);
  transition = succeed(machine);
  EXPECT_EQ(transition.state, State::kSucceeded);
}

TEST(AttachmentTransactionMachine, FailedAuthoritativeReconciliationInhibitsMotion)
{
  AttachmentTransactionMachine machine;
  drive_to(machine, State::kReleaseSceneLease);
  ASSERT_EQ(
    machine.dispatch(Event::kTerminalFailure).state,
    State::kReconcileCommittedState);
  const auto transition = machine.dispatch(Event::kTerminalFailure);
  EXPECT_EQ(transition.state, State::kInhibitMotion);
  EXPECT_EQ(transition.fault, Fault::kExternalInconsistency);
}

TEST(AttachmentTransactionMachine, BoundsNormalAndReconciliationRetries)
{
  AttachmentTransactionMachine operation_machine({1U, 2U});
  ASSERT_TRUE(operation_machine.begin(AttachmentTransactionKind::kAttach).accepted);
  auto transition = operation_machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kAcquireSceneLease);
  EXPECT_EQ(transition.attempt, 2U);
  transition = operation_machine.dispatch(Event::kRetryableFailure);
  EXPECT_EQ(transition.state, State::kRolledBack);
  EXPECT_EQ(transition.fault, Fault::kRetryExhausted);

  AttachmentTransactionMachine reconcile_machine({0U, 2U});
  ASSERT_TRUE(reconcile_machine.begin(AttachmentTransactionKind::kAttach).accepted);
  ASSERT_EQ(succeed(reconcile_machine).state, State::kCommandGazebo);
  ASSERT_EQ(
    reconcile_machine.dispatch(Event::kTimeout).state,
    State::kReconcileGazeboOutcome);
  transition = reconcile_machine.dispatch(Event::kOutcomeUnknown);
  EXPECT_EQ(transition.attempt, 2U);
  transition = reconcile_machine.dispatch(Event::kOutcomeUnknown);
  EXPECT_EQ(transition.state, State::kInhibitMotion);
  EXPECT_EQ(transition.fault, Fault::kOutcomeUnknown);
}

TEST(AttachmentTransactionMachine, CancellationWaitsForSafeCompensationBoundaries)
{
  const std::array<State, 6> precommit_states{
    State::kAcquireSceneLease,
    State::kCommandGazebo,
    State::kReconcileGazeboOutcome,
    State::kApplyMoveItDiff,
    State::kVerifyMoveItDiff,
    State::kCommitWorldState};
  for (const auto state : precommit_states) {
    AttachmentTransactionMachine machine;
    drive_to(machine, state);
    const auto canceled = machine.dispatch(Event::kCancelRequested);
    ASSERT_TRUE(canceled.accepted);
    EXPECT_TRUE(canceled.cancel_requested);
    EXPECT_EQ(canceled.state, state);
    auto transition = succeed(machine);
    while (!machine.terminal()) {
      transition = succeed(machine);
      ASSERT_TRUE(transition.accepted);
    }
    if (state == State::kCommitWorldState) {
      EXPECT_EQ(machine.status().state, State::kSucceeded);
    } else {
      EXPECT_EQ(machine.status().state, State::kRolledBack);
      EXPECT_EQ(machine.status().fault, Fault::kCanceled);
    }
  }
}

TEST(AttachmentTransactionMachine, CancellationDuringCommitRecoveryStillReconciles)
{
  for (const auto state :
    {State::kReleaseSceneLease, State::kVerifyProjection, State::kReconcileCommittedState})
  {
    AttachmentTransactionMachine machine;
    if (state == State::kReconcileCommittedState) {
      drive_to(machine, State::kReleaseSceneLease);
      ASSERT_EQ(
        machine.dispatch(Event::kTerminalFailure).state,
        State::kReconcileCommittedState);
    } else {
      drive_to(machine, state);
    }
    ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, state);
    auto transition = succeed(machine);
    while (!machine.terminal()) {
      transition = succeed(machine);
      ASSERT_TRUE(transition.accepted);
    }
    EXPECT_EQ(machine.status().state, State::kSucceeded);
    EXPECT_TRUE(machine.status().cancel_requested);
  }
}

TEST(AttachmentTransactionMachine, CancellationDuringRollbackCannotBypassCompensation)
{
  for (const auto state :
    {State::kCompensateGazebo,
      State::kCompensateMoveIt,
      State::kReleaseCompensationLease,
      State::kVerifyCompensationProjection})
  {
    AttachmentTransactionMachine machine;
    if (state == State::kReleaseCompensationLease ||
      state == State::kVerifyCompensationProjection)
    {
      ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
      ASSERT_EQ(succeed(machine).state, State::kCommandGazebo);
      ASSERT_EQ(
        machine.dispatch(Event::kTerminalFailure).state,
        State::kReleaseCompensationLease);
      if (state == State::kVerifyCompensationProjection) {
        ASSERT_EQ(succeed(machine).state, State::kVerifyCompensationProjection);
      }
    } else {
      drive_to(machine, State::kCommitWorldState);
      ASSERT_EQ(
        machine.dispatch(Event::kTerminalFailure).state,
        State::kCompensateGazebo);
      if (state == State::kCompensateMoveIt) {
        ASSERT_EQ(succeed(machine).state, State::kCompensateMoveIt);
      }
    }
    ASSERT_EQ(machine.dispatch(Event::kCancelRequested).state, state);
    while (!machine.terminal()) {
      ASSERT_TRUE(succeed(machine).accepted);
    }
    EXPECT_EQ(machine.status().state, State::kRolledBack);
    EXPECT_EQ(machine.status().fault, Fault::kCanceled);
  }
}

TEST(AttachmentTransactionMachine, RejectsInvalidConfigurationAndIllegalEvents)
{
  AttachmentTransactionMachine invalid({1U, 0U});
  auto transition = invalid.begin(AttachmentTransactionKind::kAttach);
  EXPECT_EQ(transition.state, State::kInhibitMotion);
  EXPECT_EQ(transition.fault, Fault::kInvalidConfiguration);

  AttachmentTransactionMachine machine;
  transition = machine.dispatch(Event::kOperationSucceeded);
  EXPECT_FALSE(transition.accepted);
  EXPECT_EQ(transition.fault, Fault::kInvalidEvent);
  ASSERT_TRUE(machine.begin(AttachmentTransactionKind::kAttach).accepted);
  transition = machine.dispatch(Event::kConfirmedNotApplied);
  EXPECT_FALSE(transition.accepted);
  EXPECT_EQ(transition.state, State::kAcquireSceneLease);
}

TEST(AttachmentTransactionMachine, ExposesStableDiagnosticNames)
{
  EXPECT_STREQ(to_string(AttachmentTransactionKind::kDetach), "detach");
  EXPECT_STREQ(to_string(State::kReconcileGazeboOutcome), "reconcile_gazebo_outcome");
  EXPECT_STREQ(to_string(Command::kSendInverseGazeboCommand), "send_inverse_gazebo_command");
  EXPECT_STREQ(to_string(Fault::kExternalInconsistency), "external_inconsistency");
}

}  // namespace
}  // namespace restocker_task_executor
