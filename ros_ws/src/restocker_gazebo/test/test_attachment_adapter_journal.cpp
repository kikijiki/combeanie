// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <optional>
#include <string>

#include "restocker_gazebo/attachment_adapter_journal.hpp"

namespace restocker_gazebo
{
namespace
{

AttachmentIdentity identity()
{
  return AttachmentIdentity{
    7, "sim:stock_can_01", "restocker", "gripper", "stock_can_01", "product_body"};
}

AttachmentPhysicalState detached_state()
{
  AttachmentPhysicalState result;
  result.simulator_epoch = "sim-epoch";
  result.sequence = 1;
  result.phase = AttachmentPhase::kDetached;
  result.motion_gate = AttachmentMotionGate::kValid;
  result.status = AttachmentStatus{AttachmentStatusCode::kDetached, "detached"};
  return result;
}

AdapterMutationRequest attach_request()
{
  AdapterMutationRequest result;
  result.command = AttachmentCommand::kAttach;
  result.object_id = 7;
  result.reservation_token = "reservation-secret";
  result.planning_scene_lease_token = "lease-secret";
  result.has_expected_grasp = true;
  result.expected_grasp_center_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  return result;
}

AdapterMutationRequest detach_request()
{
  AdapterMutationRequest result;
  result.command = AttachmentCommand::kDetach;
  result.object_id = 7;
  result.reservation_token = "reservation-secret";
  result.planning_scene_lease_token = "lease-secret";
  return result;
}

AdapterAuthorizationResult authorization(
  AdapterReservationStage stage = AdapterReservationStage::kReserved)
{
  AdapterAuthorizationEvidence evidence;
  evidence.reservation_id = 41;
  evidence.object_id = 7;
  evidence.object_source_id = "sim:stock_can_01";
  evidence.reservation_stage = stage;
  evidence.lease_id = 17;
  evidence.lease_phase = AdapterLeasePhase::kHeld;
  evidence.resolved_identity = identity();
  return AdapterAuthorizationResult{
    AttachmentStatus{AttachmentStatusCode::kPending, "authorized"}, evidence};
}

AttachmentEvidence evidence(bool joint_observed)
{
  AttachmentEvidence result;
  result.joint_observed = joint_observed;
  result.parent_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  result.child_pose_in_world.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  result.simulator_iteration = 100;
  result.simulation_time_ns = 1000;
  return result;
}

AttachmentJournalReply transport_terminal(
  const std::string & operation_id, const AttachmentRequest & request,
  AttachmentStatusCode terminal)
{
  const bool attached = terminal == AttachmentStatusCode::kAttached;
  AttachmentOperationRecord operation;
  operation.simulator_epoch = "sim-epoch";
  operation.sequence = 3;
  operation.operation_id = operation_id;
  operation.request = request;
  operation.phase = attached ? AttachmentPhase::kAttached : AttachmentPhase::kDetached;
  operation.status = AttachmentStatus{terminal, "terminal"};
  operation.mutation_started = true;
  operation.evidence = evidence(attached);

  AttachmentPhysicalState state;
  state.simulator_epoch = "sim-epoch";
  state.sequence = 3;
  state.phase = operation.phase;
  state.motion_gate = AttachmentMotionGate::kValid;
  state.status = operation.status;
  state.evidence = operation.evidence;
  if (attached) {
    state.attached_identity = request.identity;
  }
  return AttachmentJournalReply{operation.status, false, operation, state};
}

AttachmentJournalReply safe_rejection(
  const std::string & operation_id, const AttachmentRequest & request)
{
  AttachmentOperationRecord operation;
  operation.simulator_epoch = "sim-epoch";
  operation.sequence = 2;
  operation.operation_id = operation_id;
  operation.request = request;
  operation.phase = AttachmentPhase::kDetached;
  operation.status = AttachmentStatus{
    AttachmentStatusCode::kGripperNotReady, "finger target mismatch"};
  operation.mutation_started = false;
  return AttachmentJournalReply{operation.status, false, operation, detached_state()};
}

AttachmentAdapterJournal ready_journal(std::size_t capacity = 8)
{
  AttachmentAdapterJournal result(capacity);
  const auto initialized = result.initialize(detached_state(), true);
  EXPECT_EQ(initialized.status.code, AttachmentStatusCode::kDetached);
  EXPECT_FALSE(result.motion_inhibited());
  return result;
}

TEST(AttachmentAdapterJournalTest, RejectsZeroCapacityAndCommandsBeforeReconciliation)
{
  EXPECT_THROW(AttachmentAdapterJournal(0), std::invalid_argument);
  AttachmentAdapterJournal journal(4);
  EXPECT_EQ(
    journal.submit("attach", attach_request()).status.code,
    AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, StartupRequiresCleanCrossSystemDetachedAgreement)
{
  AttachmentAdapterJournal journal(4);
  auto attached = detached_state();
  attached.phase = AttachmentPhase::kAttached;
  attached.status.code = AttachmentStatusCode::kAttached;
  attached.attached_identity = identity();
  EXPECT_EQ(
    journal.initialize(attached, true).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.initialize(detached_state(), true).status.code,
    AttachmentStatusCode::kStateMismatch);
}

TEST(AttachmentAdapterJournalTest, AuthorizesAndRevalidatesACompleteAttach)
{
  auto journal = ready_journal();
  const auto submitted = journal.submit("attach-1", attach_request());
  EXPECT_EQ(submitted.status.code, AttachmentStatusCode::kPending);
  EXPECT_EQ(submitted.action, AdapterAction::kValidateInitialCapabilities);
  EXPECT_FALSE(submitted.transport_request);
  EXPECT_TRUE(journal.motion_inhibited());

  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_TRUE(authorized.transport_request);
  EXPECT_EQ(authorized.action, AdapterAction::kSubmitTransportCommand);
  EXPECT_EQ(authorized.transport_request->reservation_id, 41U);
  EXPECT_EQ(authorized.transport_request->identity, identity());
  const auto transport_request = *authorized.transport_request;

  EXPECT_EQ(
    journal.mark_transport_submitted("attach-1").action,
    AdapterAction::kQueryTransportOperation);
  auto terminal = transport_terminal(
    "attach-1", transport_request, AttachmentStatusCode::kAttached);
  EXPECT_EQ(
    journal.observe_transport_reply("attach-1", terminal).action,
    AdapterAction::kValidateTerminalCapabilities);
  const auto completed = journal.complete_terminal_authorization("attach-1", authorization());
  EXPECT_EQ(completed.status.code, AttachmentStatusCode::kAttached);
  ASSERT_TRUE(completed.state);
  EXPECT_EQ(completed.state->attached_identity, identity());
  EXPECT_FALSE(journal.motion_inhibited());

  const auto replayed = journal.submit("attach-1", attach_request());
  EXPECT_EQ(replayed.status.code, AttachmentStatusCode::kAttached);
  EXPECT_TRUE(replayed.replayed);
  EXPECT_EQ(replayed.action, AdapterAction::kNone);
}

TEST(AttachmentAdapterJournalTest, AcceptsOneUlpQuaternionRenormalizationFromGazebo)
{
  auto journal = ready_journal();
  auto request = attach_request();
  request.expected_grasp_center_to_child.rotation_xyzw = {
    -0.5370096126743666, 0.5182685781648229,
    -0.5849953189410149, 0.31748832041421593};
  ASSERT_EQ(
    journal.submit("attach-renormalized", request).action,
    AdapterAction::kValidateInitialCapabilities);
  const auto authorized = journal.complete_initial_authorization(
    "attach-renormalized", authorization());
  ASSERT_TRUE(authorized.transport_request);
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-renormalized").action,
    AdapterAction::kQueryTransportOperation);

  auto echoed_request = *authorized.transport_request;
  echoed_request.expected_grasp_center_to_child.rotation_xyzw[0] = std::nextafter(
    echoed_request.expected_grasp_center_to_child.rotation_xyzw[0], 0.0);
  const auto terminal = transport_terminal(
    "attach-renormalized", echoed_request, AttachmentStatusCode::kAttached);
  EXPECT_EQ(
    journal.observe_transport_reply("attach-renormalized", terminal).action,
    AdapterAction::kValidateTerminalCapabilities);
}

TEST(AttachmentAdapterJournalTest, RejectsQuaternionChangeOutsideWireNormalizationShell)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-changed-pose", attach_request()).action,
    AdapterAction::kValidateInitialCapabilities);
  const auto authorized = journal.complete_initial_authorization(
    "attach-changed-pose", authorization());
  ASSERT_TRUE(authorized.transport_request);
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-changed-pose").action,
    AdapterAction::kQueryTransportOperation);

  auto changed_request = *authorized.transport_request;
  changed_request.expected_grasp_center_to_child.rotation_xyzw[0] = 1.0e-12;
  const auto terminal = transport_terminal(
    "attach-changed-pose", changed_request, AttachmentStatusCode::kAttached);
  const auto rejected = journal.observe_transport_reply("attach-changed-pose", terminal);
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, ChangedTokenConflictsWithoutExposingCapabilities)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  auto changed = attach_request();
  changed.reservation_token = "different-secret";
  const auto conflict = journal.submit("attach-1", changed);
  EXPECT_EQ(conflict.status.code, AttachmentStatusCode::kIdempotencyConflict);
  EXPECT_FALSE(conflict.transport_request);
  EXPECT_EQ(conflict.status.detail.find("secret"), std::string::npos);
}

TEST(AttachmentAdapterJournalTest, InitialAuthorizationFailureHasNoTransportSideEffect)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).action,
    AdapterAction::kValidateInitialCapabilities);
  auto wrong_stage = authorization(AdapterReservationStage::kAttached);
  const auto rejected = journal.complete_initial_authorization("attach-1", wrong_stage);
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kAuthorizationFailed);
  EXPECT_EQ(rejected.action, AdapterAction::kNone);
  EXPECT_FALSE(rejected.transport_request);
  EXPECT_FALSE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).action,
    AdapterAction::kValidateInitialCapabilities);
}

TEST(AttachmentAdapterJournalTest, SafePhysicalRejectionAllowsAnotherOperation)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").status.code,
    AttachmentStatusCode::kPending);
  const auto rejected = journal.observe_transport_reply(
    "attach-1", safe_rejection("attach-1", *authorized.transport_request));
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kGripperNotReady);
  EXPECT_FALSE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).action,
    AdapterAction::kValidateInitialCapabilities);
}

TEST(AttachmentAdapterJournalTest, MissingTerminalProofLatchesInconsistency)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").status.code,
    AttachmentStatusCode::kPending);
  auto malformed = transport_terminal(
    "attach-1", *authorized.transport_request, AttachmentStatusCode::kAttached);
  malformed.operation->evidence.reset();
  const auto rejected = journal.observe_transport_reply("attach-1", malformed);
  EXPECT_EQ(rejected.status.code, AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentAdapterJournalTest, CapabilityLossAfterMutationLatchesInconsistency)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.observe_transport_reply(
      "attach-1",
      transport_terminal(
        "attach-1", *authorized.transport_request,
        AttachmentStatusCode::kAttached))
    .status.code,
    AttachmentStatusCode::kPending);
  AdapterAuthorizationResult lost{
    AttachmentStatus{AttachmentStatusCode::kTokenMismatch, "token mismatch"}, std::nullopt};
  EXPECT_EQ(
    journal.complete_terminal_authorization("attach-1", lost).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, ChangedReservationIdentityAfterMutationLatchesInconsistency)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.observe_transport_reply(
      "attach-1",
      transport_terminal(
        "attach-1", *authorized.transport_request, AttachmentStatusCode::kAttached)).action,
    AdapterAction::kValidateTerminalCapabilities);
  auto changed = authorization();
  changed.evidence->reservation_id = 42;
  EXPECT_EQ(
    journal.complete_terminal_authorization("attach-1", changed).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, UnknownDeadlineKeepsExactOperationReconcilable)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").status.code,
    AttachmentStatusCode::kPending);
  const auto timed_out = journal.mark_reconciliation_deadline_exceeded(
    "attach-1", "steady-clock deadline exceeded");
  EXPECT_EQ(timed_out.status.code, AttachmentStatusCode::kOutcomeUnknown);
  EXPECT_EQ(timed_out.action, AdapterAction::kQueryTransportOperation);
  EXPECT_TRUE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.observe_transport_reply(
      "attach-1",
      transport_terminal(
        "attach-1", *authorized.transport_request, AttachmentStatusCode::kAttached)).action,
    AdapterAction::kValidateTerminalCapabilities);
}

TEST(AttachmentAdapterJournalTest, DetachRequiresAttachedReservationStage)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("detach-1", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    journal.complete_initial_authorization(
      "detach-1", authorization(AdapterReservationStage::kReserved)).status.code,
    AttachmentStatusCode::kAuthorizationFailed);

  auto second = ready_journal();
  ASSERT_EQ(
    second.submit("detach-1", detach_request()).status.code,
    AttachmentStatusCode::kPending);
  const auto authorized = second.complete_initial_authorization(
    "detach-1", authorization(AdapterReservationStage::kAttached));
  ASSERT_TRUE(authorized.transport_request);
  EXPECT_EQ(authorized.transport_request->command, AttachmentCommand::kDetach);
  EXPECT_FALSE(authorized.transport_request->has_expected_grasp);
}

// The simulator restores an idle phase and a valid motion gate when it refuses a mutation before
// touching anything, but the reported state still carries the refusal code. That code must not be
// read as the current state: doing so latched a permanent motion inhibition on the next idle poll
// and the coordinator's retry was refused as motion-inhibited.
TEST(AttachmentAdapterJournalTest, IdleWatchdogAcceptsAnIdleStateLeftByARefusedMutation)
{
  auto journal = ready_journal();
  AttachmentPhysicalState restored = detached_state();
  restored.sequence = 2;
  restored.status = AttachmentStatus{
    AttachmentStatusCode::kOutOfTolerance, "observed grasp transform differs from the candidate"};
  EXPECT_EQ(
    journal.observe_idle_state(restored).status.code, AttachmentStatusCode::kOutOfTolerance);
  EXPECT_FALSE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.submit("attach-1", attach_request()).action,
    AdapterAction::kValidateInitialCapabilities);
}

// The physical state itself must still be consistent: an inhibited gate or a phase that
// disagrees with what is held is a cross-system disagreement.
TEST(AttachmentAdapterJournalTest, IdleWatchdogStillLatchesAnIncoherentPhysicalState)
{
  auto inhibited_gate = ready_journal();
  AttachmentPhysicalState gated = detached_state();
  gated.sequence = 2;
  gated.motion_gate = AttachmentMotionGate::kInhibited;
  EXPECT_EQ(
    inhibited_gate.observe_idle_state(gated).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(inhibited_gate.motion_inhibited());

  auto held_while_detached = ready_journal();
  AttachmentPhysicalState mismatched = detached_state();
  mismatched.sequence = 2;
  mismatched.attached_identity = identity();
  EXPECT_EQ(
    held_while_detached.observe_idle_state(mismatched).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(held_while_detached.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, IdleWatchdogAcceptsOnlyStableTerminalPhysicalState)
{
  auto journal = ready_journal();
  auto observation = detached_state();
  observation.sequence = 2;
  EXPECT_EQ(
    journal.observe_idle_state(observation).status.code,
    AttachmentStatusCode::kDetached);
  EXPECT_FALSE(journal.motion_inhibited());

  observation.sequence = 3;
  observation.simulator_epoch = "restarted-simulator";
  EXPECT_EQ(
    journal.observe_idle_state(observation).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
  EXPECT_EQ(
    journal.observe_idle_state(observation).status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, MalformedTransportReplyCanLatchActiveInconsistency)
{
  auto journal = ready_journal();
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.complete_initial_authorization("attach-1", authorization()).action,
    AdapterAction::kSubmitTransportCommand);
  ASSERT_EQ(
    journal.mark_transport_submitted("attach-1").action,
    AdapterAction::kQueryTransportOperation);
  EXPECT_EQ(
    journal.mark_transport_inconsistency("attach-1", "malformed reply").status.code,
    AttachmentStatusCode::kExternalInconsistency);
  EXPECT_TRUE(journal.motion_inhibited());
}

TEST(AttachmentAdapterJournalTest, IdleTransportProtocolFailureLatchesInconsistency)
{
  auto journal = ready_journal();
  const auto failure = journal.mark_idle_inconsistency("malformed watchdog reply");
  EXPECT_EQ(failure.status.code, AttachmentStatusCode::kExternalInconsistency);
  ASSERT_TRUE(failure.state);
  EXPECT_EQ(failure.state->simulator_epoch, "sim-epoch");
  EXPECT_TRUE(journal.motion_inhibited());
}

// Drive the actual simulator journal so mismatched adapter/plugin record counts cannot be hidden
// by a fabricated terminal reply. These are pure state machines; no simulator is launched.
AdapterReply complete_operation(
  AttachmentAdapterJournal & adapter, AttachmentJournal & simulator,
  const std::string & operation_id, const AdapterMutationRequest & request,
  bool reject = false)
{
  const bool attach = request.command == AttachmentCommand::kAttach;
  auto authorized = authorization(
    attach ? AdapterReservationStage::kReserved : AdapterReservationStage::kAttached);
  authorized.evidence->lease_id = attach ? 17 : 18;
  EXPECT_EQ(adapter.submit(operation_id, request).status.code, AttachmentStatusCode::kPending);
  const auto admitted = adapter.complete_initial_authorization(operation_id, authorized);
  EXPECT_TRUE(admitted.transport_request);
  if (!admitted.transport_request) {
    return admitted;
  }
  EXPECT_EQ(
    adapter.mark_transport_submitted(operation_id).action,
    AdapterAction::kQueryTransportOperation);
  EXPECT_EQ(
    simulator.submit(operation_id, *admitted.transport_request).status.code,
    AttachmentStatusCode::kPending);
  if (reject) {
    return adapter.observe_transport_reply(
      operation_id, simulator.reject_before_mutation(
        operation_id, {AttachmentStatusCode::kGripperNotReady, "gripper is not ready"}));
  }
  EXPECT_EQ(
    simulator.mark_validation_succeeded(operation_id).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    simulator.mark_mutation_started(operation_id).status.code,
    AttachmentStatusCode::kPending);
  EXPECT_EQ(
    adapter.observe_transport_reply(
      operation_id,
      simulator.mark_verification_succeeded(operation_id, evidence(attach))).action,
    AdapterAction::kValidateTerminalCapabilities);
  return adapter.complete_terminal_authorization(operation_id, authorized);
}

TEST(AttachmentAdapterJournalTest, ProtectsDetachAcrossIndependentJournalExhaustion)
{
  auto adapter = ready_journal(4);
  AttachmentJournal simulator("sim-epoch", 3);
  ASSERT_EQ(
    complete_operation(adapter, simulator, "attach", attach_request()).status.code,
    AttachmentStatusCode::kAttached);

  auto unauthorized = detach_request();
  unauthorized.reservation_token = "wrong-owner";
  ASSERT_EQ(adapter.submit("denied", unauthorized).status.code, AttachmentStatusCode::kPending);
  const AdapterAuthorizationResult denied{
    {AttachmentStatusCode::kTokenMismatch, "wrong reservation"}, std::nullopt};
  ASSERT_EQ(
    adapter.complete_initial_authorization("denied", denied).status.code,
    AttachmentStatusCode::kTokenMismatch);
  EXPECT_EQ(adapter.size(), 2U);
  EXPECT_EQ(simulator.size(), 1U);
  for (int attempt = 0; attempt < 4; ++attempt) {
    EXPECT_EQ(
      adapter.submit("unrelated-" + std::to_string(attempt), unauthorized).status.code,
      AttachmentStatusCode::kResourceExhausted);
    EXPECT_EQ(adapter.size(), 2U);
    EXPECT_EQ(
      adapter.submit("denied", unauthorized).status.code,
      AttachmentStatusCode::kTokenMismatch);
  }

  auto wrong_object = detach_request();
  ++wrong_object.object_id;
  EXPECT_EQ(
    adapter.submit("wrong-object", wrong_object).status.code,
    AttachmentStatusCode::kResourceExhausted);
  EXPECT_EQ(
    adapter.submit("duplicate-attach", attach_request()).status.code,
    AttachmentStatusCode::kStateMismatch);

  // Attachment transactions release their scene lease: detach legitimately owns a fresh lease.
  auto detach = detach_request();
  detach.planning_scene_lease_token = "fresh-detach-lease";
  ASSERT_EQ(
    complete_operation(adapter, simulator, "detach-rejected", detach, true).status.code,
    AttachmentStatusCode::kGripperNotReady);
  ASSERT_EQ(
    complete_operation(adapter, simulator, "detach", detach).status.code,
    AttachmentStatusCode::kDetached);
  EXPECT_EQ(adapter.size(), 4U);
  EXPECT_EQ(simulator.size(), 3U);
  EXPECT_FALSE(adapter.motion_inhibited());
  EXPECT_FALSE(simulator.state().attached_identity);
  EXPECT_EQ(
    adapter.submit("next-attach", attach_request()).status.code,
    AttachmentStatusCode::kResourceExhausted);
  for (int replay = 0; replay < 3; ++replay) {
    EXPECT_EQ(adapter.submit("detach", detach).status.code, AttachmentStatusCode::kDetached);
    EXPECT_EQ(
      adapter.submit("detach-rejected", detach).status.code,
      AttachmentStatusCode::kGripperNotReady);
    EXPECT_EQ(
      adapter.submit("attach", attach_request()).status.code,
      AttachmentStatusCode::kAttached);
    EXPECT_EQ(simulator.query("detach").status.code, AttachmentStatusCode::kDetached);
  }
  detach.reservation_token = "different-owner";
  EXPECT_EQ(
    adapter.submit("detach", detach).status.code,
    AttachmentStatusCode::kIdempotencyConflict);
}

TEST(AttachmentAdapterJournalTest, RejectedAttachReleasesUnusedCleanupCredits)
{
  for (const bool physical_rejection : {false, true}) {
    SCOPED_TRACE(physical_rejection);
    auto adapter = ready_journal(4);
    AttachmentJournal simulator("sim-epoch", 4);
    if (physical_rejection) {
      ASSERT_EQ(
        complete_operation(adapter, simulator, "rejected", attach_request(), true).status.code,
        AttachmentStatusCode::kGripperNotReady);
    } else {
      ASSERT_EQ(
        adapter.submit("rejected", attach_request()).status.code,
        AttachmentStatusCode::kPending);
      ASSERT_EQ(
        adapter.complete_initial_authorization(
          "rejected",
          {{AttachmentStatusCode::kTokenMismatch, "denied"}, std::nullopt}).status.code,
        AttachmentStatusCode::kTokenMismatch);
    }
    EXPECT_EQ(adapter.size(), 1U);
    ASSERT_EQ(
      complete_operation(adapter, simulator, "attach", attach_request()).status.code,
      AttachmentStatusCode::kAttached);
    ASSERT_EQ(
      complete_operation(adapter, simulator, "detach", detach_request()).status.code,
      AttachmentStatusCode::kDetached);
    EXPECT_EQ(
      adapter.query("rejected").status.code, physical_rejection ?
      AttachmentStatusCode::kGripperNotReady : AttachmentStatusCode::kTokenMismatch);
  }
}

TEST(AttachmentAdapterJournalTest, MatchingAuthorizationFailuresUseFiniteDetachAttemptBudget)
{
  auto adapter = ready_journal(3);
  AttachmentJournal simulator("sim-epoch", 3);
  ASSERT_EQ(
    complete_operation(adapter, simulator, "attach", attach_request()).status.code,
    AttachmentStatusCode::kAttached);
  const AdapterAuthorizationResult denied{
    {AttachmentStatusCode::kAuthorizationFailed, "current lease unavailable"}, std::nullopt};
  for (int attempt = 0; attempt < 2; ++attempt) {
    const auto id = "detach-denied-" + std::to_string(attempt);
    EXPECT_EQ(adapter.submit(id, detach_request()).status.code, AttachmentStatusCode::kPending);
    EXPECT_EQ(
      adapter.complete_initial_authorization(id, denied).status.code,
      AttachmentStatusCode::kAuthorizationFailed);
    EXPECT_EQ(
      adapter.submit(id, detach_request()).status.code,
      AttachmentStatusCode::kAuthorizationFailed);
  }
  EXPECT_EQ(adapter.size(), adapter.capacity());
  EXPECT_EQ(
    adapter.submit("detach-exhausted", detach_request()).status.code,
    AttachmentStatusCode::kResourceExhausted);
  EXPECT_EQ(simulator.size(), 1U);
  EXPECT_TRUE(simulator.state().attached_identity);
}

TEST(AttachmentAdapterJournalTest, RefusesAttachWithoutItsConfiguredCleanupReserve)
{
  for (const std::size_t capacity : {1U, 2U}) {
    auto adapter = ready_journal(capacity);
    EXPECT_EQ(
      adapter.submit("attach", attach_request()).status.code,
      AttachmentStatusCode::kResourceExhausted);
    EXPECT_EQ(adapter.size(), 0U);
    EXPECT_FALSE(adapter.motion_inhibited());
  }
  AttachmentAdapterJournal configured(4, 3);
  ASSERT_EQ(
    configured.initialize(detached_state(), true).status.code,
    AttachmentStatusCode::kDetached);
  AttachmentJournal simulator("sim-epoch", 4, std::nullopt, 3);
  ASSERT_EQ(
    complete_operation(configured, simulator, "attach", attach_request()).status.code,
    AttachmentStatusCode::kAttached);
  for (int attempt = 0; attempt < 2; ++attempt) {
    EXPECT_EQ(
      complete_operation(
        configured, simulator,
        "detach-retry-" + std::to_string(attempt), detach_request(), true).status.code,
      AttachmentStatusCode::kGripperNotReady);
  }
  EXPECT_EQ(
    complete_operation(configured, simulator, "detach", detach_request()).status.code,
    AttachmentStatusCode::kDetached);
  EXPECT_EQ(configured.size(), configured.capacity());
  EXPECT_EQ(simulator.size(), simulator.capacity());
}

TEST(AttachmentAdapterJournalTest, CapacityIsFixedAndNonEvicting)
{
  auto journal = ready_journal(3);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  AdapterAuthorizationResult denied{
    AttachmentStatus{AttachmentStatusCode::kTokenMismatch, "denied"}, std::nullopt};
  ASSERT_EQ(
    journal.complete_initial_authorization("attach-1", denied).status.code,
    AttachmentStatusCode::kTokenMismatch);
  EXPECT_EQ(journal.size(), 1U);
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kResourceExhausted);
  EXPECT_EQ(
    journal.query("attach-1").status.code,
    AttachmentStatusCode::kTokenMismatch);
}

// CMB-SPEC-10 Stage 1: retention bounds are named and observable.
TEST(AttachmentAdapterJournalTest, RetentionSnapshotClassifiesOpenAndTerminalRecords)
{
  AttachmentAdapterJournal fresh(8);
  auto snapshot = fresh.retention_snapshot();
  EXPECT_EQ(snapshot.journal, "adapter.operations");
  EXPECT_FALSE(snapshot.epoch_id.empty());
  EXPECT_NE(snapshot.epoch_id, AttachmentAdapterJournal(8).retention_snapshot().epoch_id);
  EXPECT_TRUE(snapshot.inhibited) << "uninitialized adapter admits nothing";
  EXPECT_FALSE(snapshot.evicting);

  auto journal = ready_journal(8);
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.capacity, 8U);
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_FALSE(snapshot.inhibited);
  EXPECT_FALSE(snapshot.held_attachment);

  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.size, 1U);
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.reserved_credits, 2U);
  EXPECT_TRUE(snapshot.inhibited);

  const auto authorized = journal.complete_initial_authorization("attach-1", authorization());
  ASSERT_TRUE(authorized.transport_request);
  const auto request = *authorized.transport_request;
  static_cast<void>(journal.mark_transport_submitted("attach-1"));
  static_cast<void>(
    journal.observe_transport_reply(
      "attach-1", transport_terminal("attach-1", request, AttachmentStatusCode::kAttached)));
  ASSERT_EQ(
    journal.complete_terminal_authorization("attach-1", authorization()).status.code,
    AttachmentStatusCode::kAttached);
  snapshot = journal.retention_snapshot();
  // The settled attach still owns the cleanup credit, so it is an obligation, not a receipt.
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.terminal_receipts, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 2U);
  EXPECT_TRUE(snapshot.held_attachment);
  EXPECT_FALSE(snapshot.inhibited);
}

TEST(AttachmentAdapterJournalTest, RetentionSnapshotCountsRejectedAndInconsistentRecords)
{
  auto journal = ready_journal(8);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  AdapterAuthorizationResult denied{
    AttachmentStatus{AttachmentStatusCode::kTokenMismatch, "denied"}, std::nullopt};
  ASSERT_EQ(
    journal.complete_initial_authorization("attach-1", denied).status.code,
    AttachmentStatusCode::kTokenMismatch);
  auto snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.size, 1U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.terminal_receipts, 1U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);

  ASSERT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  ASSERT_EQ(
    journal.complete_initial_authorization("attach-2", authorization()).action,
    AdapterAction::kSubmitTransportCommand);
  static_cast<void>(journal.mark_transport_submitted("attach-2"));
  static_cast<void>(journal.mark_transport_inconsistency("attach-2", "mismatch"));
  snapshot = journal.retention_snapshot();
  EXPECT_EQ(snapshot.size, 2U);
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.terminal_receipts, 1U);
  EXPECT_TRUE(snapshot.inhibited);
}

TEST(AttachmentAdapterJournalTest, RetentionSnapshotIsReadOnlyAndNeverEvicts)
{
  auto journal = ready_journal(3);
  ASSERT_EQ(
    journal.submit("attach-1", attach_request()).status.code,
    AttachmentStatusCode::kPending);
  AdapterAuthorizationResult denied{
    AttachmentStatus{AttachmentStatusCode::kTokenMismatch, "denied"}, std::nullopt};
  static_cast<void>(journal.complete_initial_authorization("attach-1", denied));
  const auto before = journal.retention_snapshot();
  for (int index = 0; index < 3; ++index) {
    EXPECT_EQ(journal.retention_snapshot().size, before.size);
  }
  EXPECT_EQ(
    journal.submit("attach-2", attach_request()).status.code,
    AttachmentStatusCode::kResourceExhausted);
  const auto after = journal.retention_snapshot();
  EXPECT_EQ(after.size, before.size);
  EXPECT_EQ(after.terminal_receipts, before.terminal_receipts);
  EXPECT_EQ(journal.query("attach-1").status.code, AttachmentStatusCode::kTokenMismatch);
}

}  // namespace
}  // namespace restocker_gazebo
