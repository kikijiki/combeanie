// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/attachment_transaction_machine.hpp"

#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] AttachmentTransactionCommand command_for(AttachmentTransactionState state)
{
  switch (state) {
    case AttachmentTransactionState::kIdle:
    case AttachmentTransactionState::kSucceeded:
    case AttachmentTransactionState::kRolledBack:
      return AttachmentTransactionCommand::kNone;
    case AttachmentTransactionState::kAcquireSceneLease:
      return AttachmentTransactionCommand::kAcquireSceneLease;
    case AttachmentTransactionState::kReconcileSceneLeaseOutcome:
      return AttachmentTransactionCommand::kQuerySceneLeaseOutcome;
    case AttachmentTransactionState::kCommandGazebo:
      return AttachmentTransactionCommand::kSendGazeboCommand;
    case AttachmentTransactionState::kReconcileGazeboOutcome:
      return AttachmentTransactionCommand::kQueryGazeboOutcome;
    case AttachmentTransactionState::kProvePlacement:
      return AttachmentTransactionCommand::kVerifySettledPlacement;
    case AttachmentTransactionState::kApplyMoveItDiff:
      return AttachmentTransactionCommand::kApplyMoveItDiff;
    case AttachmentTransactionState::kVerifyMoveItDiff:
      return AttachmentTransactionCommand::kReadBackMoveItScene;
    case AttachmentTransactionState::kCommitWorldState:
      return AttachmentTransactionCommand::kCommitWorldState;
    case AttachmentTransactionState::kReleaseSceneLease:
      return AttachmentTransactionCommand::kReleaseSceneLease;
    case AttachmentTransactionState::kVerifyProjection:
      return AttachmentTransactionCommand::kWaitForProjection;
    case AttachmentTransactionState::kCompensateGazebo:
      return AttachmentTransactionCommand::kSendInverseGazeboCommand;
    case AttachmentTransactionState::kCompensateMoveIt:
      return AttachmentTransactionCommand::kApplyInverseMoveItDiff;
    case AttachmentTransactionState::kReleaseCompensationLease:
      return AttachmentTransactionCommand::kReleaseCompensationLease;
    case AttachmentTransactionState::kVerifyCompensationProjection:
      return AttachmentTransactionCommand::kWaitForCompensationProjection;
    case AttachmentTransactionState::kReconcileCommittedState:
      return AttachmentTransactionCommand::kReconcileCommittedState;
    case AttachmentTransactionState::kInhibitMotion:
      return AttachmentTransactionCommand::kInhibitMotion;
  }
  return AttachmentTransactionCommand::kInhibitMotion;
}

[[nodiscard]] bool is_terminal(AttachmentTransactionState state)
{
  return state == AttachmentTransactionState::kSucceeded ||
         state == AttachmentTransactionState::kRolledBack ||
         state == AttachmentTransactionState::kInhibitMotion;
}

[[nodiscard]] std::chrono::milliseconds timeout_for(
  AttachmentTransactionState state, const AttachmentTransactionConfig & config)
{
  if (state == AttachmentTransactionState::kReconcileSceneLeaseOutcome ||
    state == AttachmentTransactionState::kReconcileGazeboOutcome ||
    state == AttachmentTransactionState::kReconcileCommittedState)
  {
    return config.reconciliation_timeout;
  }
  if (state == AttachmentTransactionState::kCompensateGazebo ||
    state == AttachmentTransactionState::kCompensateMoveIt ||
    state == AttachmentTransactionState::kReleaseCompensationLease ||
    state == AttachmentTransactionState::kVerifyCompensationProjection)
  {
    return config.compensation_timeout;
  }
  return is_terminal(state) || state == AttachmentTransactionState::kIdle ?
         std::chrono::milliseconds{0} : config.operation_timeout;
}

}  // namespace

AttachmentTransactionMachine::AttachmentTransactionMachine(AttachmentTransactionConfig config)
: config_(config)
{
  configuration_valid_ = config_.max_reconciliation_retries > 0U &&
    config_.operation_timeout.count() > 0 && config_.reconciliation_timeout.count() > 0 &&
    config_.compensation_timeout.count() > 0;
}

AttachmentTransactionTransition AttachmentTransactionMachine::begin(
  AttachmentTransactionKind kind)
{
  if (state_ != AttachmentTransactionState::kIdle) {
    return reject(
      AttachmentTransactionFault::kInvalidEvent,
      "an attachment transaction is already active or terminal");
  }
  if (!configuration_valid_) {
    return inhibit(
      AttachmentTransactionFault::kInvalidConfiguration,
      "reconciliation attempts and all steady-clock timeouts must be positive");
  }
  kind_ = kind;
  return transition_to(
    AttachmentTransactionState::kAcquireSceneLease,
    std::string("begin ") + to_string(kind) + " transaction");
}

AttachmentTransactionTransition AttachmentTransactionMachine::dispatch(
  AttachmentTransactionEvent event, std::string detail)
{
  if (state_ == AttachmentTransactionState::kIdle || terminal()) {
    return reject(
      AttachmentTransactionFault::kInvalidEvent,
      "events are not accepted while the transaction is idle or terminal");
  }
  if (event == AttachmentTransactionEvent::kCancelRequested) {
    cancel_requested_ = true;
    fault_ = AttachmentTransactionFault::kCanceled;
    detail_ = detail.empty() ? "cancellation requested; waiting for a safe boundary" :
      std::move(detail);
    return status();
  }
  if (event == AttachmentTransactionEvent::kRetryableFailure) {
    return retry_or_fail(AttachmentTransactionFault::kRetryExhausted, std::move(detail));
  }
  if (event == AttachmentTransactionEvent::kOutcomeUnknown ||
    event == AttachmentTransactionEvent::kTimeout)
  {
    if (state_ == AttachmentTransactionState::kAcquireSceneLease) {
      return transition_to(
        AttachmentTransactionState::kReconcileSceneLeaseOutcome,
        "lease acquisition outcome is unknown; poll the idempotent operation");
    }
    if (state_ == AttachmentTransactionState::kCommandGazebo) {
      return transition_to(
        AttachmentTransactionState::kReconcileGazeboOutcome,
        "Gazebo command outcome is unknown; query the operation journal");
    }
    if (state_ == AttachmentTransactionState::kApplyMoveItDiff) {
      moveit_transition_applied_ = true;
      return transition_to(
        AttachmentTransactionState::kVerifyMoveItDiff,
        "MoveIt apply outcome is unknown; determine state by readback");
    }
    if (state_ == AttachmentTransactionState::kCommitWorldState) {
      world_commit_outcome_unknown_ = true;
      return transition_to(
        AttachmentTransactionState::kReconcileCommittedState,
        "world-state commit outcome is unknown; reconcile by reservation identity");
    }
    if (state_ == AttachmentTransactionState::kReconcileSceneLeaseOutcome ||
      state_ == AttachmentTransactionState::kReconcileGazeboOutcome ||
      state_ == AttachmentTransactionState::kReconcileCommittedState)
    {
      return retry_or_fail(AttachmentTransactionFault::kOutcomeUnknown, std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kCompensateGazebo ||
      state_ == AttachmentTransactionState::kCompensateMoveIt ||
      state_ == AttachmentTransactionState::kReleaseCompensationLease ||
      state_ == AttachmentTransactionState::kVerifyCompensationProjection)
    {
      return inhibit(
        AttachmentTransactionFault::kCompensationFailed,
        detail.empty() ? "compensation outcome cannot be proven" : std::move(detail));
    }
    if (world_state_committed_) {
      return transition_to(
        AttachmentTransactionState::kReconcileCommittedState,
        detail.empty() ? "post-commit operation requires authoritative reconciliation" :
        std::move(detail));
    }
    return begin_rollback(
      AttachmentTransactionFault::kOperationFailed,
      detail.empty() ? "operation timed out before semantic commit" : std::move(detail));
  }

  if (event == AttachmentTransactionEvent::kTerminalFailure) {
    if (state_ == AttachmentTransactionState::kCompensateGazebo ||
      state_ == AttachmentTransactionState::kCompensateMoveIt ||
      state_ == AttachmentTransactionState::kReleaseCompensationLease ||
      state_ == AttachmentTransactionState::kVerifyCompensationProjection)
    {
      return inhibit(
        AttachmentTransactionFault::kCompensationFailed,
        detail.empty() ? "compensation failed" : std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kReconcileSceneLeaseOutcome) {
      return inhibit(
        AttachmentTransactionFault::kExternalInconsistency,
        detail.empty() ? "lease acquisition outcome reconciliation failed" :
        std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kReconcileGazeboOutcome) {
      return inhibit(
        AttachmentTransactionFault::kExternalInconsistency,
        detail.empty() ? "Gazebo outcome reconciliation failed" : std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kReconcileCommittedState) {
      return inhibit(
        AttachmentTransactionFault::kExternalInconsistency,
        detail.empty() ? "authoritative committed-state reconciliation failed" :
        std::move(detail));
    }
    if (world_state_committed_) {
      return transition_to(
        AttachmentTransactionState::kReconcileCommittedState,
        detail.empty() ? "post-commit failure requires authoritative reconciliation" :
        std::move(detail));
    }
    return begin_rollback(
      AttachmentTransactionFault::kOperationFailed,
      detail.empty() ? "operation failed before semantic commit" : std::move(detail));
  }

  if (event == AttachmentTransactionEvent::kConfirmedNotApplied) {
    if (state_ == AttachmentTransactionState::kReconcileSceneLeaseOutcome) {
      scene_lease_acquired_ = false;
      fault_ = cancel_requested_ ? AttachmentTransactionFault::kCanceled :
        AttachmentTransactionFault::kOperationFailed;
      return transition_to(
        AttachmentTransactionState::kRolledBack,
        detail.empty() ? "projector proved that no transaction lease was acquired" :
        std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kReconcileGazeboOutcome) {
      physical_transition_applied_ = false;
      return begin_rollback(
        AttachmentTransactionFault::kOperationFailed,
        detail.empty() ? "Gazebo proved that the transition was not applied" :
        std::move(detail));
    }
    if (state_ == AttachmentTransactionState::kReconcileCommittedState &&
      world_commit_outcome_unknown_)
    {
      world_commit_outcome_unknown_ = false;
      return begin_rollback(
        AttachmentTransactionFault::kOperationFailed,
        detail.empty() ? "world state proved that the semantic commit was not applied" :
        std::move(detail));
    }
    return reject(
      AttachmentTransactionFault::kInvalidEvent,
      "confirmed-not-applied is invalid for the current transaction state");
  }

  if (event != AttachmentTransactionEvent::kOperationSucceeded) {
    return reject(
      AttachmentTransactionFault::kInvalidEvent, "unsupported transaction event");
  }
  return handle_success();
}

AttachmentTransactionTransition AttachmentTransactionMachine::handle_success()
{
  switch (state_) {
    case AttachmentTransactionState::kAcquireSceneLease:
      scene_lease_acquired_ = true;
      if (cancel_requested_) {
        return transition_to(
          AttachmentTransactionState::kReleaseCompensationLease,
          "lease acquisition completed after cancellation");
      }
      return transition_to(
        AttachmentTransactionState::kCommandGazebo, "planning-scene lease acquired");
    case AttachmentTransactionState::kReconcileSceneLeaseOutcome:
      scene_lease_acquired_ = true;
      if (cancel_requested_) {
        return transition_to(
          AttachmentTransactionState::kReleaseCompensationLease,
          "late lease acquisition confirmed after cancellation");
      }
      return transition_to(
        AttachmentTransactionState::kCommandGazebo,
        "late lease acquisition confirmed by idempotent polling");
    case AttachmentTransactionState::kCommandGazebo:
      return transition_to(
        AttachmentTransactionState::kReconcileGazeboOutcome,
        "Gazebo command accepted; terminal physical state must be reconciled");
    case AttachmentTransactionState::kReconcileGazeboOutcome:
      physical_transition_applied_ = true;
      if (cancel_requested_) {
        return transition_to(
          AttachmentTransactionState::kCompensateGazebo,
          "physical transition completed after cancellation");
      }
      if (kind_ == AttachmentTransactionKind::kDetach) {
        return transition_to(
          AttachmentTransactionState::kProvePlacement,
          "physical detachment verified; settled placement proof is required");
      }
      return transition_to(
        AttachmentTransactionState::kApplyMoveItDiff,
        "physical transition verified from the Gazebo journal");
    case AttachmentTransactionState::kProvePlacement:
      if (cancel_requested_) {
        return transition_to(
          AttachmentTransactionState::kCompensateGazebo,
          "placement proof completed after cancellation; restore attachment");
      }
      return transition_to(
        AttachmentTransactionState::kApplyMoveItDiff,
        "settled placement and destination evidence verified before semantic commit");
    case AttachmentTransactionState::kApplyMoveItDiff:
      moveit_transition_applied_ = true;
      return transition_to(
        AttachmentTransactionState::kVerifyMoveItDiff,
        "MoveIt diff accepted; exact readback is required");
    case AttachmentTransactionState::kVerifyMoveItDiff:
      moveit_transition_applied_ = true;
      if (cancel_requested_) {
        return transition_to(
          AttachmentTransactionState::kCompensateGazebo,
          "MoveIt transition verified after cancellation");
      }
      return transition_to(
        AttachmentTransactionState::kCommitWorldState,
        "MoveIt transition verified exactly");
    case AttachmentTransactionState::kCommitWorldState:
      world_state_committed_ = true;
      world_commit_outcome_unknown_ = false;
      return transition_to(
        AttachmentTransactionState::kReleaseSceneLease,
        "authoritative semantic state committed");
    case AttachmentTransactionState::kReleaseSceneLease:
      scene_lease_acquired_ = false;
      return transition_to(
        AttachmentTransactionState::kVerifyProjection,
        "planning-scene lease released at the committed revision");
    case AttachmentTransactionState::kVerifyProjection:
      return transition_to(
        AttachmentTransactionState::kSucceeded,
        cancel_requested_ ? "transaction committed safely after cancellation request" :
        "transaction completed");
    case AttachmentTransactionState::kCompensateGazebo:
      physical_transition_applied_ = false;
      if (moveit_transition_applied_) {
        return transition_to(
          AttachmentTransactionState::kCompensateMoveIt,
          "inverse physical transition verified; restore MoveIt representation");
      }
      return transition_to(
        AttachmentTransactionState::kReleaseCompensationLease,
        "inverse physical transition verified");
    case AttachmentTransactionState::kCompensateMoveIt:
      moveit_transition_applied_ = false;
      return transition_to(
        AttachmentTransactionState::kReleaseCompensationLease,
        "inverse MoveIt representation verified");
    case AttachmentTransactionState::kReleaseCompensationLease:
      scene_lease_acquired_ = false;
      return transition_to(
        AttachmentTransactionState::kVerifyCompensationProjection,
        "compensation lease released; restored projection must be verified");
    case AttachmentTransactionState::kVerifyCompensationProjection:
      return transition_to(
        AttachmentTransactionState::kRolledBack,
        cancel_requested_ ? "transaction canceled and rolled back" :
        "transaction failure rolled back");
    case AttachmentTransactionState::kReconcileCommittedState:
      scene_lease_acquired_ = false;
      if (world_commit_outcome_unknown_) {
        world_state_committed_ = true;
        world_commit_outcome_unknown_ = false;
        return transition_to(
          AttachmentTransactionState::kReleaseSceneLease,
          "world-state commit was confirmed by authoritative reconciliation");
      }
      return transition_to(
        AttachmentTransactionState::kSucceeded,
        "committed state and planning projection reconciled");
    case AttachmentTransactionState::kIdle:
    case AttachmentTransactionState::kSucceeded:
    case AttachmentTransactionState::kRolledBack:
    case AttachmentTransactionState::kInhibitMotion:
      break;
  }
  return reject(
    AttachmentTransactionFault::kInvalidEvent,
    "operation success is invalid for the current transaction state");
}

AttachmentTransactionTransition AttachmentTransactionMachine::status() const
{
  return {
    true, state_, command_for(state_), fault_, attempt_, timeout_for(state_, config_),
    cancel_requested_, detail_};
}

AttachmentTransactionKind AttachmentTransactionMachine::kind() const noexcept
{
  return kind_;
}

bool AttachmentTransactionMachine::terminal() const noexcept
{
  return is_terminal(state_);
}

bool AttachmentTransactionMachine::motion_inhibited() const noexcept
{
  return state_ == AttachmentTransactionState::kInhibitMotion;
}

AttachmentTransactionTransition AttachmentTransactionMachine::transition_to(
  AttachmentTransactionState next, std::string detail)
{
  state_ = next;
  attempt_ = is_terminal(next) ? attempt_ : 1U;
  if (!detail.empty()) {
    detail_ = std::move(detail);
  }
  return status();
}

AttachmentTransactionTransition AttachmentTransactionMachine::reject(
  AttachmentTransactionFault fault, std::string detail) const
{
  return {
    false, state_, command_for(state_), fault, attempt_, timeout_for(state_, config_),
    cancel_requested_, std::move(detail)};
}

AttachmentTransactionTransition AttachmentTransactionMachine::begin_rollback(
  AttachmentTransactionFault fault, std::string detail)
{
  fault_ = cancel_requested_ ? AttachmentTransactionFault::kCanceled : fault;
  if (world_state_committed_) {
    return transition_to(
      AttachmentTransactionState::kReconcileCommittedState, std::move(detail));
  }
  if (physical_transition_applied_) {
    return transition_to(AttachmentTransactionState::kCompensateGazebo, std::move(detail));
  }
  if (scene_lease_acquired_) {
    return transition_to(
      AttachmentTransactionState::kReleaseCompensationLease, std::move(detail));
  }
  return transition_to(AttachmentTransactionState::kRolledBack, std::move(detail));
}

AttachmentTransactionTransition AttachmentTransactionMachine::inhibit(
  AttachmentTransactionFault fault, std::string detail)
{
  fault_ = fault;
  return transition_to(AttachmentTransactionState::kInhibitMotion, std::move(detail));
}

AttachmentTransactionTransition AttachmentTransactionMachine::retry_or_fail(
  AttachmentTransactionFault exhausted_fault, std::string detail)
{
  if (attempt_ < retry_limit()) {
    ++attempt_;
    detail_ = detail.empty() ? "retry the bounded transaction operation" : std::move(detail);
    return status();
  }
  if (state_ == AttachmentTransactionState::kReconcileSceneLeaseOutcome ||
    state_ == AttachmentTransactionState::kReconcileGazeboOutcome ||
    state_ == AttachmentTransactionState::kReconcileCommittedState)
  {
    return inhibit(
      exhausted_fault,
      detail.empty() ? "bounded reconciliation attempts exhausted" : std::move(detail));
  }
  if (state_ == AttachmentTransactionState::kCompensateGazebo ||
    state_ == AttachmentTransactionState::kCompensateMoveIt ||
    state_ == AttachmentTransactionState::kReleaseCompensationLease ||
    state_ == AttachmentTransactionState::kVerifyCompensationProjection)
  {
    return inhibit(
      AttachmentTransactionFault::kCompensationFailed,
      detail.empty() ? "bounded compensation attempts exhausted" : std::move(detail));
  }
  return begin_rollback(
    exhausted_fault,
    detail.empty() ? "bounded operation attempts exhausted" : std::move(detail));
}

std::size_t AttachmentTransactionMachine::retry_limit() const noexcept
{
  if (state_ == AttachmentTransactionState::kReconcileSceneLeaseOutcome ||
    state_ == AttachmentTransactionState::kReconcileGazeboOutcome ||
    state_ == AttachmentTransactionState::kReconcileCommittedState)
  {
    return config_.max_reconciliation_retries;
  }
  return config_.max_operation_retries + 1U;
}

const char * to_string(AttachmentTransactionKind kind) noexcept
{
  switch (kind) {
    case AttachmentTransactionKind::kAttach: return "attach";
    case AttachmentTransactionKind::kDetach: return "detach";
  }
  return "unknown";
}

const char * to_string(AttachmentTransactionState state) noexcept
{
  switch (state) {
    case AttachmentTransactionState::kIdle: return "idle";
    case AttachmentTransactionState::kAcquireSceneLease: return "acquire_scene_lease";
    case AttachmentTransactionState::kReconcileSceneLeaseOutcome:
      return "reconcile_scene_lease_outcome";
    case AttachmentTransactionState::kCommandGazebo: return "command_gazebo";
    case AttachmentTransactionState::kReconcileGazeboOutcome:
      return "reconcile_gazebo_outcome";
    case AttachmentTransactionState::kProvePlacement: return "prove_placement";
    case AttachmentTransactionState::kApplyMoveItDiff: return "apply_moveit_diff";
    case AttachmentTransactionState::kVerifyMoveItDiff: return "verify_moveit_diff";
    case AttachmentTransactionState::kCommitWorldState: return "commit_world_state";
    case AttachmentTransactionState::kReleaseSceneLease: return "release_scene_lease";
    case AttachmentTransactionState::kVerifyProjection: return "verify_projection";
    case AttachmentTransactionState::kCompensateGazebo: return "compensate_gazebo";
    case AttachmentTransactionState::kCompensateMoveIt: return "compensate_moveit";
    case AttachmentTransactionState::kReleaseCompensationLease:
      return "release_compensation_lease";
    case AttachmentTransactionState::kVerifyCompensationProjection:
      return "verify_compensation_projection";
    case AttachmentTransactionState::kReconcileCommittedState:
      return "reconcile_committed_state";
    case AttachmentTransactionState::kSucceeded: return "succeeded";
    case AttachmentTransactionState::kRolledBack: return "rolled_back";
    case AttachmentTransactionState::kInhibitMotion: return "inhibit_motion";
  }
  return "unknown";
}

const char * to_string(AttachmentTransactionCommand command) noexcept
{
  switch (command) {
    case AttachmentTransactionCommand::kNone: return "none";
    case AttachmentTransactionCommand::kAcquireSceneLease: return "acquire_scene_lease";
    case AttachmentTransactionCommand::kQuerySceneLeaseOutcome:
      return "query_scene_lease_outcome";
    case AttachmentTransactionCommand::kSendGazeboCommand: return "send_gazebo_command";
    case AttachmentTransactionCommand::kQueryGazeboOutcome: return "query_gazebo_outcome";
    case AttachmentTransactionCommand::kVerifySettledPlacement:
      return "verify_settled_placement";
    case AttachmentTransactionCommand::kApplyMoveItDiff: return "apply_moveit_diff";
    case AttachmentTransactionCommand::kReadBackMoveItScene: return "read_back_moveit_scene";
    case AttachmentTransactionCommand::kCommitWorldState: return "commit_world_state";
    case AttachmentTransactionCommand::kReleaseSceneLease: return "release_scene_lease";
    case AttachmentTransactionCommand::kWaitForProjection: return "wait_for_projection";
    case AttachmentTransactionCommand::kSendInverseGazeboCommand:
      return "send_inverse_gazebo_command";
    case AttachmentTransactionCommand::kApplyInverseMoveItDiff:
      return "apply_inverse_moveit_diff";
    case AttachmentTransactionCommand::kReleaseCompensationLease:
      return "release_compensation_lease";
    case AttachmentTransactionCommand::kWaitForCompensationProjection:
      return "wait_for_compensation_projection";
    case AttachmentTransactionCommand::kReconcileCommittedState:
      return "reconcile_committed_state";
    case AttachmentTransactionCommand::kInhibitMotion: return "inhibit_motion";
  }
  return "unknown";
}

const char * to_string(AttachmentTransactionFault fault) noexcept
{
  switch (fault) {
    case AttachmentTransactionFault::kNone: return "none";
    case AttachmentTransactionFault::kCanceled: return "canceled";
    case AttachmentTransactionFault::kOperationFailed: return "operation_failed";
    case AttachmentTransactionFault::kRetryExhausted: return "retry_exhausted";
    case AttachmentTransactionFault::kOutcomeUnknown: return "outcome_unknown";
    case AttachmentTransactionFault::kInvalidConfiguration: return "invalid_configuration";
    case AttachmentTransactionFault::kInvalidEvent: return "invalid_event";
    case AttachmentTransactionFault::kCompensationFailed: return "compensation_failed";
    case AttachmentTransactionFault::kExternalInconsistency: return "external_inconsistency";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
