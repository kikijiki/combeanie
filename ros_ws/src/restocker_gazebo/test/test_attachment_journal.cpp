// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{
namespace
{

CanonicalPose identity_pose(double x = 0.0, double y = 0.0, double z = 0.0)
{
  return CanonicalPose{{x, y, z}, {0.0, 0.0, 0.0, 1.0}};
}

AttachmentIdentity can_identity(std::uint64_t object_id = 17)
{
  return AttachmentIdentity{
    object_id, "sim:stock_can_01", "restocker", "gripper", "stock_can_01", "product_body",
  };
}

AttachmentRequest attach_request(std::uint64_t object_id = 17)
{
  return AttachmentRequest{
    AttachmentCommand::kAttach, 41, can_identity(object_id), true, identity_pose(0.0, 0.0, 0.14),
  };
}

AttachmentRequest detach_request(std::uint64_t object_id = 17)
{
  return AttachmentRequest{
    AttachmentCommand::kDetach, 41, can_identity(object_id), false, {},
  };
}

AttachmentEvidence evidence(bool joint_observed)
{
  AttachmentEvidence result;
  result.joint_observed = joint_observed;
  result.parent_to_child = identity_pose(0.0, 0.0, 0.14);
  result.child_pose_in_world = identity_pose(0.3, -0.7, 0.8);
  result.relative_twist_in_parent = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  result.simulator_iteration = 1234;
  result.simulation_time_ns = 9'500'000'000;
  return result;
}

AttachmentJournalReply complete_attach(AttachmentJournal & journal, std::string operation_id)
{
  EXPECT_EQ(
    journal.mark_validation_succeeded(operation_id).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    journal.mark_mutation_started(operation_id).status.code,
    AttachmentStatusCode::kPending);
  return journal.mark_verification_succeeded(operation_id, evidence(true));
}

AttachmentJournalReply complete_detach(AttachmentJournal & journal, std::string operation_id)
{
  EXPECT_EQ(
    journal.mark_validation_succeeded(operation_id).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    journal.mark_mutation_started(operation_id).status.code,
    AttachmentStatusCode::kPending);
  return journal.mark_verification_succeeded(operation_id, evidence(false));
}

TEST(AttachmentPose, CanonicalizesQuaternionSignAndRejectsInvalidInput)
{
  const auto positive = canonicalize_pose({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, 1.0});
  const auto negative = canonicalize_pose({1.0, 2.0, 3.0}, {0.0, 0.0, 0.0, -1.0});
  ASSERT_TRUE(positive);
  ASSERT_TRUE(negative);
  EXPECT_EQ(*positive, *negative);
  EXPECT_DOUBLE_EQ(negative->rotation_xyzw[3], 1.0);

  EXPECT_FALSE(canonicalize_pose({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}));
  EXPECT_FALSE(canonicalize_pose({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 2.0}));
  EXPECT_FALSE(
    canonicalize_pose(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
      {0.0, 0.0, 0.0, 1.0}));
  EXPECT_FALSE(canonicalize_pose({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, -1.0));
}

TEST(AttachmentJournal, ConstructorRequiresEpochAndBoundedDetachCapacity)
{
  EXPECT_THROW(AttachmentJournal("", 8), std::invalid_argument);
  EXPECT_THROW(AttachmentJournal("epoch-a", 2), std::invalid_argument);
  EXPECT_THROW(AttachmentJournal("epoch-a", 4, std::nullopt, 0), std::invalid_argument);

  auto invalid_identity = can_identity();
  invalid_identity.child_link.clear();
  EXPECT_THROW(AttachmentJournal("epoch-a", 8, invalid_identity), std::invalid_argument);
}

TEST(AttachmentJournal, StartsDetachedWithAValidDefaultDenyingNoEvidence)
{
  const AttachmentJournal journal("epoch-a", 8);
  EXPECT_EQ(journal.capacity(), 8U);
  EXPECT_EQ(journal.size(), 0U);
  EXPECT_EQ(journal.reserved_detach_slots(), 0U);
  EXPECT_EQ(journal.state().simulator_epoch, "epoch-a");
  EXPECT_EQ(journal.state().phase, AttachmentPhase::kDetached);
  EXPECT_EQ(journal.state().motion_gate, AttachmentMotionGate::kValid);
  EXPECT_EQ(journal.state().status.code, AttachmentStatusCode::kDetached);
  EXPECT_FALSE(journal.state().attached_identity);
}

TEST(AttachmentJournal, RejectsMalformedRequestsWithoutConsumingCapacity)
{
  AttachmentJournal journal("epoch-a", 8);
  auto request = attach_request();
  EXPECT_EQ(journal.submit("", request).status.code, AttachmentStatusCode::kInvalidArgument);
  request.reservation_id = 0;
  EXPECT_EQ(journal.submit("attach", request).status.code, AttachmentStatusCode::kInvalidArgument);
  request = attach_request();
  request.expected_grasp_center_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 2.0};
  EXPECT_EQ(journal.submit("attach", request).status.code, AttachmentStatusCode::kInvalidArgument);
  request = detach_request();
  request.has_expected_grasp = true;
  EXPECT_EQ(journal.submit("detach", request).status.code, AttachmentStatusCode::kInvalidArgument);
  EXPECT_EQ(journal.size(), 0U);
}

TEST(AttachmentJournal, CanonicalReplayIsIdempotentAndChangedPayloadConflicts)
{
  AttachmentJournal journal("epoch-a", 8);
  auto request = attach_request();
  const auto accepted = journal.submit("attach-1", request);
  ASSERT_TRUE(accepted.operation);
  EXPECT_FALSE(accepted.replayed);
  EXPECT_EQ(journal.reserved_detach_slots(), 2U);

  request.expected_grasp_center_to_child.rotation_xyzw = {0.0, 0.0, 0.0, -1.0};
  const auto replay = journal.submit("attach-1", request);
  ASSERT_TRUE(replay.operation);
  EXPECT_TRUE(replay.replayed);
  EXPECT_EQ(replay.operation->sequence, accepted.operation->sequence);

  request.identity.object_id = 18;
  EXPECT_EQ(
    journal.submit("attach-1", request).status.code,
    AttachmentStatusCode::kIdempotencyConflict);
  EXPECT_EQ(journal.size(), 1U);
}

TEST(AttachmentJournal, SerializesMutationsAndRetainsTerminalAttachmentEvidence)
{
  AttachmentJournal journal("epoch-a", 8);
  EXPECT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(journal.state().phase, AttachmentPhase::kValidatingAttach);
  EXPECT_EQ(journal.state().motion_gate, AttachmentMotionGate::kInhibited);
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kConflict);

  const auto completed = complete_attach(journal, "attach-1");
  ASSERT_TRUE(completed.operation);
  EXPECT_EQ(completed.status.code, AttachmentStatusCode::kAttached);
  EXPECT_EQ(completed.state.phase, AttachmentPhase::kAttached);
  EXPECT_EQ(completed.state.motion_gate, AttachmentMotionGate::kValid);
  ASSERT_TRUE(completed.state.attached_identity);
  EXPECT_EQ(*completed.state.attached_identity, can_identity());
  ASSERT_TRUE(completed.state.evidence);
  EXPECT_TRUE(completed.state.evidence->joint_observed);

  const auto replay = journal.submit("attach-1", attach_request());
  EXPECT_TRUE(replay.replayed);
  EXPECT_EQ(replay.status.code, AttachmentStatusCode::kAttached);
  EXPECT_EQ(replay.operation, completed.operation);
  EXPECT_EQ(
    journal.submit("attach-3", attach_request()).status.code,
    AttachmentStatusCode::kStateMismatch);
}

TEST(AttachmentJournal, SafeAttachRejectionReleasesEveryReservedDetachAttempt)
{
  AttachmentJournal journal("epoch-a", 3);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(journal.reserved_detach_slots(), 2U);
  const auto rejected = journal.reject_before_mutation(
    "attach-1", {AttachmentStatusCode::kGripperNotReady, "finger target not reached"});
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kGripperNotReady);
  ASSERT_TRUE(rejected.operation);
  EXPECT_EQ(rejected.operation->phase, AttachmentPhase::kDetached);
  EXPECT_EQ(rejected.state.phase, AttachmentPhase::kDetached);
  EXPECT_EQ(rejected.state.motion_gate, AttachmentMotionGate::kValid);
  EXPECT_EQ(journal.reserved_detach_slots(), 0U);
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kResourceExhausted);
}

TEST(AttachmentJournal, DetachConsumesReservedAttemptsAndVerifiesAbsence)
{
  AttachmentJournal journal("epoch-a", 4);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);
  EXPECT_EQ(journal.reserved_detach_slots(), 2U);

  EXPECT_EQ(
    journal.submit("wrong-detach", detach_request(18)).status.code,
    AttachmentStatusCode::kStateMismatch);
  EXPECT_EQ(journal.reserved_detach_slots(), 2U);
  EXPECT_EQ(
    journal.submit("detach-1", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(journal.reserved_detach_slots(), 1U);
  const auto completed = complete_detach(journal, "detach-1");
  EXPECT_EQ(completed.status.code, AttachmentStatusCode::kDetached);
  EXPECT_FALSE(completed.state.attached_identity);
  EXPECT_FALSE(completed.state.evidence->joint_observed);
  EXPECT_EQ(journal.reserved_detach_slots(), 0U);
}

TEST(AttachmentJournal, SafeDetachRejectionLeavesRemainingRecoveryCapacity)
{
  AttachmentJournal journal("epoch-a", 4);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);
  ASSERT_EQ(
    journal.submit("detach-1", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(journal.reserved_detach_slots(), 1U);
  const auto rejected = journal.reject_before_mutation(
    "detach-1", {AttachmentStatusCode::kGripperNotReady, "gripper is not open"});
  ASSERT_TRUE(rejected.operation);
  EXPECT_EQ(rejected.operation->phase, AttachmentPhase::kAttached);
  EXPECT_EQ(rejected.state.phase, AttachmentPhase::kAttached);
  EXPECT_EQ(rejected.state.motion_gate, AttachmentMotionGate::kValid);
  EXPECT_EQ(journal.reserved_detach_slots(), 1U);
  EXPECT_EQ(
    journal.submit("detach-2", detach_request()).status.code,
    AttachmentStatusCode::kPending);
}

TEST(AttachmentJournal, DetachReplayIgnoresTheUnusedExpectedGraspStorage)
{
  AttachmentJournal journal("epoch-a", 8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);

  const auto accepted = journal.submit("detach-1", detach_request());
  ASSERT_TRUE(accepted.operation);
  auto retry = detach_request();
  retry.expected_grasp_center_to_child = identity_pose(99.0, -42.0, 7.0);
  const auto replay = journal.submit("detach-1", retry);
  EXPECT_TRUE(replay.replayed);
  EXPECT_EQ(replay.status.code, AttachmentStatusCode::kPending);
  EXPECT_EQ(replay.operation, accepted.operation);
}

TEST(AttachmentJournal, PostMutationContradictionFailsClosedAndBlocksNewCommands)
{
  AttachmentJournal journal("epoch-a", 8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_validation_succeeded("attach-1").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(journal.mark_mutation_started("attach-1").status.code, AttachmentStatusCode::kPending);

  auto contradictory = evidence(false);
  const auto unknown = journal.mark_verification_succeeded("attach-1", contradictory);
  EXPECT_EQ(unknown.status.code, AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_EQ(unknown.state.phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(unknown.state.motion_gate, AttachmentMotionGate::kInhibited);
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_EQ(
    journal.reject_before_mutation("attach-1", {AttachmentStatusCode::kInternalError, "too late"})
    .status.code,
    AttachmentStatusCode::kStateMismatch);
}

TEST(AttachmentJournal, NegativeSimulationTimeMakesPostMutationOutcomeUnknown)
{
  AttachmentJournal journal("epoch-a", 8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_validation_succeeded("attach-1").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(journal.mark_mutation_started("attach-1").status.code, AttachmentStatusCode::kPending);

  auto invalid = evidence(true);
  invalid.simulation_time_ns = -1;
  const auto unknown = journal.mark_verification_succeeded("attach-1", invalid);
  EXPECT_EQ(unknown.status.code, AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_EQ(unknown.state.phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(unknown.state.motion_gate, AttachmentMotionGate::kInhibited);
}

TEST(AttachmentJournal, QueryDistinguishesCurrentStateMissingAndJournaledOperation)
{
  AttachmentJournal journal("epoch-a", 8);
  EXPECT_EQ(journal.query().status.code, AttachmentStatusCode::kDetached);
  EXPECT_FALSE(journal.query().operation);
  EXPECT_EQ(journal.query("missing").status.code, AttachmentStatusCode::kOperationNotFound);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto query = journal.query("attach-1");
  EXPECT_TRUE(query.replayed);
  EXPECT_TRUE(query.operation);
  EXPECT_EQ(query.operation->operation_id, "attach-1");
}

TEST(AttachmentJournal, EpochRotationIsCleanOnlyFromAnIdleDetachedState)
{
  AttachmentJournal clean("epoch-a", 8);
  clean.rotate_epoch("epoch-b");
  EXPECT_EQ(clean.state().simulator_epoch, "epoch-b");
  EXPECT_EQ(clean.state().phase, AttachmentPhase::kDetached);
  EXPECT_EQ(clean.state().motion_gate, AttachmentMotionGate::kValid);
  EXPECT_THROW(clean.rotate_epoch("epoch-b"), std::invalid_argument);

  AttachmentJournal pending("epoch-a", 8);
  ASSERT_EQ(
    pending.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  pending.rotate_epoch("epoch-b");
  EXPECT_EQ(pending.state().phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(pending.state().status.code, AttachmentStatusCode::kSimulatorEpochChanged);
  EXPECT_EQ(pending.state().motion_gate, AttachmentMotionGate::kInhibited);
  EXPECT_EQ(pending.size(), 0U);
}

TEST(AttachmentJournal, RecoveredAttachmentStartsInhibitedWithDetachCapacity)
{
  AttachmentJournal recovered("epoch-b", 8, can_identity());
  EXPECT_EQ(recovered.state().phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(recovered.state().motion_gate, AttachmentMotionGate::kInhibited);
  EXPECT_EQ(recovered.state().status.code, AttachmentStatusCode::kExternalInconsistency);
  ASSERT_TRUE(recovered.state().attached_identity);
  EXPECT_EQ(recovered.reserved_detach_slots(), 2U);
  EXPECT_EQ(
    recovered.submit("detach-recovery", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto rejected = recovered.reject_before_mutation(
    "detach-recovery", {AttachmentStatusCode::kGripperNotReady, "gripper remains closed"});
  ASSERT_TRUE(rejected.operation);
  EXPECT_EQ(rejected.operation->phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kGripperNotReady);
  EXPECT_EQ(rejected.state.phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(rejected.state.motion_gate, AttachmentMotionGate::kInhibited);
}

TEST(AttachmentJournal, HeldObjectWatchdogFaultInhibitsMotionAndRetainsIdentity)
{
  AttachmentJournal journal("epoch-a", 8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);

  const auto fault = journal.mark_external_inconsistency("owned joint disappeared");
  EXPECT_EQ(fault.status.code, AttachmentStatusCode::kExternalInconsistency);
  EXPECT_EQ(fault.state.phase, AttachmentPhase::kInconsistent);
  EXPECT_EQ(fault.state.motion_gate, AttachmentMotionGate::kInhibited);
  ASSERT_TRUE(fault.state.attached_identity);
  EXPECT_EQ(fault.state.attached_identity->object_id, 17U);
  EXPECT_EQ(
    journal.submit("attach-other", attach_request()).status.code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentJournal, WatchdogFaultTransitionRejectsInvalidLifecycleAndEvidence)
{
  AttachmentJournal journal("epoch-a", 8);
  EXPECT_EQ(
    journal.mark_external_inconsistency("not attached").status.code,
    AttachmentStatusCode::kStateMismatch);

  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    journal.mark_external_inconsistency("mutation active").status.code,
    AttachmentStatusCode::kStateMismatch);
  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);

  auto contradictory = evidence(false);
  EXPECT_EQ(
    journal.mark_external_inconsistency("bad evidence", contradictory).status.code,
    AttachmentStatusCode::kInvalidArgument);
  EXPECT_EQ(journal.state().phase, AttachmentPhase::kAttached);
  EXPECT_EQ(
    journal.mark_external_inconsistency("").status.code,
    AttachmentStatusCode::kStateMismatch);
}

// CMB-SPEC-10 Stage 1: retention bounds are named and observable.
TEST(AttachmentJournalRetention, ReportsEpochCapacityAndClassifiesLifecycle)
{
  AttachmentJournal journal("epoch-a", 8);
  auto snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.journal, "simulator.attachment");
  EXPECT_EQ(snapshot.epoch_id, "epoch-a");
  EXPECT_EQ(snapshot.capacity, 8U);
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.terminal_receipts, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);
  EXPECT_FALSE(snapshot.inhibited);
  EXPECT_FALSE(snapshot.evicting);

  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code, AttachmentStatusCode::kPending);
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.size, 1U);
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.reserved_credits, 2U);
  EXPECT_TRUE(snapshot.inhibited);

  ASSERT_EQ(complete_attach(journal, "attach-1").status.code, AttachmentStatusCode::kAttached);
  snapshot = journal.retention_snapshot();
  // The attach record still backs the held object: it is an obligation, not a receipt.
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.terminal_receipts, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 2U);
  EXPECT_TRUE(snapshot.held_attachment);
  EXPECT_FALSE(snapshot.inhibited);

  ASSERT_EQ(
    journal.submit("detach-1", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(complete_detach(journal, "detach-1").status.code, AttachmentStatusCode::kDetached);
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.size, 2U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.terminal_receipts, 2U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);
  EXPECT_EQ(snapshot.open_obligations + snapshot.terminal_receipts, snapshot.size);
  EXPECT_FALSE(snapshot.held_attachment);
}

TEST(AttachmentJournalRetention, UnknownOutcomeIsAnOpenObligationAndRotationClearsRecords)
{
  AttachmentJournal journal("epoch-a", 8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code, AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_validation_succeeded("attach-1").status.code, AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_mutation_started("attach-1").status.code, AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.mark_outcome_unknown("attach-1", "no proof").status.code,
    AttachmentStatusCode::kOutcomeUnknown);
  auto snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_TRUE(snapshot.inhibited);

  journal.rotate_epoch("epoch-b");
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.epoch_id, "epoch-b");
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_TRUE(snapshot.inhibited) << "the dropped obligation is re-expressed as an inhibited gate";
}

TEST(AttachmentJournalRetention, RecoveredAttachmentReportsReservedCreditsWithoutRecords)
{
  AttachmentJournal recovered("epoch-b", 8, can_identity());
  const auto snapshot = recovered.retention_snapshot();
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 2U);
  EXPECT_TRUE(snapshot.held_attachment) << "a held joint is state, not a record";
  EXPECT_TRUE(snapshot.inhibited);

  AttachmentJournal clean("epoch-a", 8);
  EXPECT_FALSE(clean.retention_snapshot().held_attachment);
}

}  // namespace
}  // namespace restocker_gazebo
