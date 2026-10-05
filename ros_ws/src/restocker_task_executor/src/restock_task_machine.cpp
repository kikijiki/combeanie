// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_task_machine.hpp"

#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] RestockTaskCommand command_for(RestockTaskState state)
{
  switch (state) {
    case RestockTaskState::kIdle:
    case RestockTaskState::kComplete:
    case RestockTaskState::kCanceled:
      return RestockTaskCommand::kNone;
    case RestockTaskState::kValidateScene: return RestockTaskCommand::kValidateScene;
    case RestockTaskState::kSelectPair: return RestockTaskCommand::kSelectPair;
    case RestockTaskState::kReserveTask: return RestockTaskCommand::kReserveTask;
    case RestockTaskState::kGenerateGrasps: return RestockTaskCommand::kGenerateGrasps;
    case RestockTaskState::kPlanPreGrasp: return RestockTaskCommand::kPlanPreGrasp;
    case RestockTaskState::kExecutePreGrasp: return RestockTaskCommand::kExecutePreGrasp;
    case RestockTaskState::kOpenGripperForApproach: return RestockTaskCommand::kOpenGripper;
    case RestockTaskState::kOpenGripperForEscape: return RestockTaskCommand::kOpenGripper;
    case RestockTaskState::kPlanApproach: return RestockTaskCommand::kPlanApproach;
    case RestockTaskState::kExecuteApproach: return RestockTaskCommand::kExecuteApproach;
    case RestockTaskState::kCloseGripper: return RestockTaskCommand::kCloseGripper;
    case RestockTaskState::kVerifyGrasp: return RestockTaskCommand::kVerifyGrasp;
    case RestockTaskState::kAttachTransaction:
      return RestockTaskCommand::kRunAttachTransaction;
    case RestockTaskState::kPlanRetract: return RestockTaskCommand::kPlanRetract;
    case RestockTaskState::kExecuteRetract: return RestockTaskCommand::kExecuteRetract;
    case RestockTaskState::kObserveDestination:
      return RestockTaskCommand::kObserveDestination;
    case RestockTaskState::kGeneratePlacement:
      return RestockTaskCommand::kGeneratePlacement;
    case RestockTaskState::kPlanCarryStart: return RestockTaskCommand::kPlanCarryStart;
    case RestockTaskState::kExecuteCarryStart: return RestockTaskCommand::kExecuteCarryStart;
    case RestockTaskState::kPlanPreInsert: return RestockTaskCommand::kPlanPreInsert;
    case RestockTaskState::kExecutePreInsert: return RestockTaskCommand::kExecutePreInsert;
    case RestockTaskState::kPlanInsert: return RestockTaskCommand::kPlanInsert;
    case RestockTaskState::kExecuteInsert: return RestockTaskCommand::kExecuteInsert;
    case RestockTaskState::kOpenGripper: return RestockTaskCommand::kOpenGripper;
    case RestockTaskState::kDetachTransaction:
      return RestockTaskCommand::kRunDetachTransaction;
    case RestockTaskState::kPlanRetreat: return RestockTaskCommand::kPlanRetreat;
    case RestockTaskState::kExecuteRetreat: return RestockTaskCommand::kExecuteRetreat;
    case RestockTaskState::kSurveyDestination:
      return RestockTaskCommand::kSurveyDestination;
    case RestockTaskState::kCommitDetachment:
      return RestockTaskCommand::kRunCommitDetachment;
    case RestockTaskState::kVerifyPlacement: return RestockTaskCommand::kVerifyPlacement;
    case RestockTaskState::kUpdateInventory: return RestockTaskCommand::kUpdateInventory;
    case RestockTaskState::kCancelActiveMotion:
      return RestockTaskCommand::kCancelMotionAndVerifyStop;
    case RestockTaskState::kReleaseTask:
      return RestockTaskCommand::kReleaseTaskReservation;
    case RestockTaskState::kRecover: return RestockTaskCommand::kExecuteRecovery;
    case RestockTaskState::kFault: return RestockTaskCommand::kInhibitMotion;
    case RestockTaskState::kRequestOperator: return RestockTaskCommand::kRequestOperator;
  }
  return RestockTaskCommand::kInhibitMotion;
}

[[nodiscard]] bool is_terminal(RestockTaskState state)
{
  return state == RestockTaskState::kComplete || state == RestockTaskState::kCanceled ||
         state == RestockTaskState::kRequestOperator;
}

[[nodiscard]] bool is_planning_state(RestockTaskState state)
{
  return state == RestockTaskState::kPlanPreGrasp ||
         state == RestockTaskState::kPlanApproach ||
         state == RestockTaskState::kPlanRetract ||
         state == RestockTaskState::kPlanCarryStart ||
         state == RestockTaskState::kPlanPreInsert ||
         state == RestockTaskState::kPlanInsert ||
         state == RestockTaskState::kPlanRetreat;
}

[[nodiscard]] bool is_execution_state(RestockTaskState state)
{
  return state == RestockTaskState::kExecutePreGrasp ||
         state == RestockTaskState::kExecuteApproach ||
         state == RestockTaskState::kExecuteRetract ||
         state == RestockTaskState::kExecuteCarryStart ||
         state == RestockTaskState::kExecutePreInsert ||
         state == RestockTaskState::kExecuteInsert ||
         state == RestockTaskState::kExecuteRetreat;
}

[[nodiscard]] bool is_free_space_execution(RestockTaskState state)
{
  return state == RestockTaskState::kExecutePreGrasp ||
         state == RestockTaskState::kExecuteRetract ||
         state == RestockTaskState::kExecuteCarryStart ||
         state == RestockTaskState::kExecutePreInsert ||
         state == RestockTaskState::kExecuteRetreat;
}

[[nodiscard]] bool is_transaction_state(RestockTaskState state)
{
  return state == RestockTaskState::kAttachTransaction ||
         state == RestockTaskState::kDetachTransaction ||
         state == RestockTaskState::kCommitDetachment;
}

[[nodiscard]] std::chrono::milliseconds timeout_for(
  RestockTaskState state, const RestockTaskConfig & config)
{
  if (is_terminal(state) || state == RestockTaskState::kIdle) {
    return std::chrono::milliseconds{0};
  }
  if (is_planning_state(state)) {
    return config.planning_timeout;
  }
  if (is_execution_state(state) || state == RestockTaskState::kCloseGripper ||
    state == RestockTaskState::kOpenGripper ||
    state == RestockTaskState::kOpenGripperForApproach ||
    state == RestockTaskState::kOpenGripperForEscape ||
    state == RestockTaskState::kCancelActiveMotion)
  {
    return config.execution_timeout;
  }
  if (is_transaction_state(state)) {
    return config.transaction_timeout;
  }
  if (state == RestockTaskState::kRecover || state == RestockTaskState::kFault) {
    return config.recovery_timeout;
  }
  return config.validation_timeout;
}

[[nodiscard]] RestockTaskState next_success_state(RestockTaskState state)
{
  switch (state) {
    case RestockTaskState::kValidateScene: return RestockTaskState::kSelectPair;
    case RestockTaskState::kSelectPair: return RestockTaskState::kReserveTask;
    case RestockTaskState::kReserveTask: return RestockTaskState::kGenerateGrasps;
    case RestockTaskState::kGenerateGrasps: return RestockTaskState::kPlanPreGrasp;
    case RestockTaskState::kPlanPreGrasp: return RestockTaskState::kExecutePreGrasp;
    case RestockTaskState::kExecutePreGrasp:
      return RestockTaskState::kOpenGripperForApproach;
    case RestockTaskState::kOpenGripperForApproach: return RestockTaskState::kPlanApproach;
    case RestockTaskState::kPlanApproach: return RestockTaskState::kExecuteApproach;
    case RestockTaskState::kExecuteApproach: return RestockTaskState::kCloseGripper;
    case RestockTaskState::kCloseGripper: return RestockTaskState::kVerifyGrasp;
    case RestockTaskState::kVerifyGrasp: return RestockTaskState::kAttachTransaction;
    case RestockTaskState::kAttachTransaction: return RestockTaskState::kPlanRetract;
    case RestockTaskState::kPlanRetract: return RestockTaskState::kExecuteRetract;
    case RestockTaskState::kExecuteRetract: return RestockTaskState::kObserveDestination;
    case RestockTaskState::kObserveDestination: return RestockTaskState::kGeneratePlacement;
    case RestockTaskState::kGeneratePlacement: return RestockTaskState::kPlanCarryStart;
    case RestockTaskState::kPlanCarryStart: return RestockTaskState::kExecuteCarryStart;
    case RestockTaskState::kExecuteCarryStart: return RestockTaskState::kPlanPreInsert;
    case RestockTaskState::kPlanPreInsert: return RestockTaskState::kExecutePreInsert;
    case RestockTaskState::kExecutePreInsert: return RestockTaskState::kPlanInsert;
    case RestockTaskState::kPlanInsert: return RestockTaskState::kExecuteInsert;
    case RestockTaskState::kExecuteInsert: return RestockTaskState::kOpenGripper;
    case RestockTaskState::kOpenGripper: return RestockTaskState::kDetachTransaction;
    case RestockTaskState::kDetachTransaction: return RestockTaskState::kPlanRetreat;
    case RestockTaskState::kPlanRetreat: return RestockTaskState::kExecuteRetreat;
    case RestockTaskState::kExecuteRetreat: return RestockTaskState::kSurveyDestination;
    case RestockTaskState::kSurveyDestination: return RestockTaskState::kCommitDetachment;
    case RestockTaskState::kCommitDetachment: return RestockTaskState::kVerifyPlacement;
    case RestockTaskState::kVerifyPlacement: return RestockTaskState::kUpdateInventory;
    case RestockTaskState::kUpdateInventory: return RestockTaskState::kComplete;
    default: return state;
  }
}

[[nodiscard]] RestockTaskState recovery_resume_for(RestockTaskState state)
{
  switch (state) {
    // A straight-line segment starts from the configuration the preceding free-space traverse
    // left, and that configuration decides how much straight travel is possible. Retrying from
    // the same configuration repeats the failure, so resume at the free-space plan before it.
    case RestockTaskState::kPlanApproach: return RestockTaskState::kPlanPreGrasp;
    case RestockTaskState::kPlanInsert: return RestockTaskState::kPlanPreInsert;
    case RestockTaskState::kExecutePreGrasp: return RestockTaskState::kPlanPreGrasp;
    case RestockTaskState::kExecuteApproach: return RestockTaskState::kPlanApproach;
    case RestockTaskState::kCloseGripper:
    case RestockTaskState::kVerifyGrasp:
      // Jaw close / verify failures re-enter generate so the driver rebinds selection to the
      // snapshot verify may have adopted and regenerates the batch for the next free-space leg
      // (recovery_attempt > 0 relaxes the generate-success first-trajectory gate).
      return RestockTaskState::kGenerateGrasps;
    // A fully rolled-back attach leaves the arm at the attempted grasp with the candidate batch
    // retained; the driver rejects that candidate, so plan the next one in free space.
    case RestockTaskState::kAttachTransaction: return RestockTaskState::kPlanPreGrasp;
    case RestockTaskState::kExecuteRetract: return RestockTaskState::kPlanRetract;
    case RestockTaskState::kExecuteCarryStart: return RestockTaskState::kPlanCarryStart;
    case RestockTaskState::kExecutePreInsert: return RestockTaskState::kPlanPreInsert;
    case RestockTaskState::kExecuteInsert: return RestockTaskState::kPlanInsert;
    case RestockTaskState::kOpenGripper:
    case RestockTaskState::kDetachTransaction:
      return RestockTaskState::kObserveDestination;
    case RestockTaskState::kExecuteRetreat: return RestockTaskState::kPlanRetreat;
    case RestockTaskState::kSurveyDestination:
    case RestockTaskState::kCommitDetachment:
    case RestockTaskState::kVerifyPlacement:
      // Physical detach already happened; re-aim and re-measure so the semantic commit can
      // still prove column growth strictly after release.
      return RestockTaskState::kPlanRetreat;
    default: return state;
  }
}

}  // namespace

RestockTaskMachine::RestockTaskMachine(RestockTaskConfig config)
: config_(config)
{
  configuration_valid_ = config_.max_recovery_attempts > 0U &&
    config_.validation_timeout.count() > 0 && config_.planning_timeout.count() > 0 &&
    config_.execution_timeout.count() > 0 && config_.transaction_timeout.count() > 0 &&
    config_.recovery_timeout.count() > 0 && config_.total_timeout.count() > 0;
}

RestockTaskTransition RestockTaskMachine::begin()
{
  if (state_ != RestockTaskState::kIdle) {
    return reject(RestockTaskFault::kInvalidEvent, "a restock task is already active or terminal");
  }
  if (!configuration_valid_) {
    return begin_fault(
      RestockTaskFault::kInvalidConfiguration,
      "recovery attempts and all steady-clock timeouts must be positive");
  }
  return transition_to(RestockTaskState::kValidateScene, "begin deterministic restock task");
}

RestockTaskTransition RestockTaskMachine::dispatch(
  RestockTaskEvent event, std::string detail,
  std::optional<std::uint64_t> operation_generation)
{
  if (state_ == RestockTaskState::kIdle || terminal()) {
    return reject(
      RestockTaskFault::kInvalidEvent,
      "events are not accepted while the task is idle or terminal");
  }
  if (event == RestockTaskEvent::kReservationAcquired) {
    if (state_ != RestockTaskState::kReserveTask || reservation_active_) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "reservation acquisition is accepted exactly once in reserve-task state");
    }
    reservation_active_ = true;
    detail_ = detail.empty() ?
      "reservation capability acquired; authoritative validation is pending" :
      std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kSelectionSuperseded) {
    // The world state rejects a reservation whose entity revisions moved or whose predicates no
    // longer hold before mutating anything, so the task owns no reservation. This is the normal
    // result of racing the observation stream: re-run read/select/reserve on fresh evidence.
    // kSelectPair is also pre-commitment: selection-time evidence can age out while the
    // perception reacquire holds the state, and the retained snapshot must be refreshed the
    // same bounded way rather than selected against.
    const bool before_any_commitment =
      (state_ == RestockTaskState::kReserveTask ||
      state_ == RestockTaskState::kSelectPair) &&
      !reservation_active_ && !first_trajectory_may_have_started_ && !object_held_;
    if (!before_any_commitment) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "a superseded selection is accepted only before a reservation or motion is committed");
    }
    if (termination_requested()) {
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kCanceled,
        "selection was superseded after termination; nothing was reserved");
    }
    if (selection_restart_ >= config_.max_selection_restarts) {
      std::string exhausted = "bounded reservation re-selection attempts exhausted";
      if (!detail.empty()) {
        exhausted += "; last cause: " + detail;
      }
      return begin_fault(RestockTaskFault::kRetryExhausted, std::move(exhausted));
    }
    ++selection_restart_;
    return transition_to(
      RestockTaskState::kValidateScene,
      detail.empty() ? "reservation preconditions were superseded; re-observe and re-select" :
      std::move(detail));
  }
  if (event == RestockTaskEvent::kRecoverableSkipRequested) {
    // Milestone 10 §6 rung 5 (Card 051): the typed exit from the fault boundary. Explicit
    // (requested, never inferred), single-shot (a second request is refused, so the ladder
    // terminates), and routed through the cleanup the termination branches already walk:
    // retreat when the arm moved, release when a reservation exists, done otherwise.
    if (state_ == RestockTaskState::kIdle || terminal()) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "events are not accepted while the task is idle or terminal");
    }
    if (recoverable_skip_) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "the recoverable skip was already attempted for this task; it cannot loop");
    }
    if (state_ != RestockTaskState::kFault) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "a recoverable skip is requested only from the fault boundary");
    }
    if (current_execution_may_have_started_) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "a recoverable skip requires the verified stop that fault handling established");
    }
    recoverable_skip_ = true;
    detail_ = detail.empty() ? "recoverable skip requested (rung 5)" : std::move(detail);
    if (first_trajectory_may_have_started_) {
      return transition_to(
        RestockTaskState::kPlanRetreat,
        object_held_ ?
        "recoverable skip: the arm is stopped holding the product; the cleanup retreat carries "
        "it to the safe pose" :
        "recoverable skip: the arm is at its verified stop; the cleanup retreat returns to the "
        "safe pose");
    }
    if (reservation_active_) {
      return transition_to(
        RestockTaskState::kReleaseTask,
        "recoverable skip: nothing moved; release the reservation");
    }
    return transition_to(
      RestockTaskState::kCanceled,
      "recoverable skip: nothing was reserved and nothing moved");
  }
  if (event == RestockTaskEvent::kExecutionOperationStarted) {
    if (!is_execution_state(state_) || !operation_generation || *operation_generation == 0 ||
      current_execution_operation_generation_ != 0 ||
      current_execution_may_have_started_ ||
      (termination_requested() && !first_trajectory_may_have_started_ && !object_held_))
    {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "execution operation start requires an unterminated, unbound execution state and "
        "nonzero generation");
    }
    current_execution_operation_generation_ = *operation_generation;
    detail_ = detail.empty() ? "execution operation generation bound" : std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kTrajectoryExecutionAccepted) {
    if (!is_execution_state(state_) || !operation_generation ||
      *operation_generation == 0 ||
      *operation_generation != current_execution_operation_generation_ ||
      current_execution_may_have_started_)
    {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "trajectory acceptance requires the exact current execution operation generation");
    }
    current_execution_may_have_started_ = true;
    first_trajectory_may_have_started_ = true;
    detail_ = detail.empty() ?
      "trajectory execution accepted; motion may have started" : std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kCancelRequested) {
    return handle_cancel(std::move(detail));
  }
  if (event == RestockTaskEvent::kTaskDeadlineExceeded) {
    task_deadline_exceeded_ = true;
    cancel_requested_ = true;
    fault_ = RestockTaskFault::kTimedOut;
    if (is_free_space_execution(state_) && current_execution_may_have_started_) {
      canceled_motion_state_ = state_;
      return transition_to(
        RestockTaskState::kCancelActiveMotion,
        "task deadline exceeded; cancel active motion and verify stop");
    }
    detail_ = detail.empty() ? "whole-task deadline exceeded; waiting for a safe boundary" :
      std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kDrainRequested) {
    if (first_trajectory_may_have_started_ || object_held_ || is_transaction_state(state_) ||
      state_ == RestockTaskState::kCancelActiveMotion || state_ == RestockTaskState::kFault ||
      state_ == RestockTaskState::kRequestOperator || is_terminal(state_))
    {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "coordinator drain is implemented only at a pre-motion cleanup boundary");
    }
    safe_abort_requested_ = true;
    deadline_retreat_revoked_ = true;  // stop wins: no retreat authority survives a drain
    if (!task_deadline_exceeded_) {
      fault_ = RestockTaskFault::kOperationFailed;
    }
    detail_ = detail.empty() ?
      "coordinator drain requested; waiting for the current operation boundary" :
      std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kSafeAbortRequested) {
    if (!reservation_active_ || first_trajectory_may_have_started_ || object_held_ ||
      state_ == RestockTaskState::kReleaseTask || is_transaction_state(state_) ||
      state_ == RestockTaskState::kCancelActiveMotion || state_ == RestockTaskState::kFault)
    {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "safe abort requires an active pre-motion reservation at a cleanup-safe boundary");
    }
    safe_abort_requested_ = true;
    deadline_retreat_revoked_ = true;  // stop wins: no retreat authority survives a safe abort
    if (!task_deadline_exceeded_) {
      fault_ = RestockTaskFault::kOperationFailed;
    }
    detail_ = detail.empty() ?
      "safe abort requested; waiting for the current operation boundary" : std::move(detail);
    return status();
  }
  if (event == RestockTaskEvent::kTransactionInhibited) {
    if (!is_transaction_state(state_)) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "transaction-inhibited event is invalid outside a transaction state");
    }
    return begin_fault(
      RestockTaskFault::kExternalInconsistency,
      detail.empty() ? "child transaction inhibited motion" : std::move(detail));
  }
  if (event == RestockTaskEvent::kTransactionRolledBack) {
    if (!is_transaction_state(state_)) {
      return reject(
        RestockTaskFault::kInvalidEvent,
        "transaction-rolled-back event is invalid outside a transaction state");
    }
    if (state_ == RestockTaskState::kAttachTransaction) {
      object_held_ = false;
    } else if (state_ == RestockTaskState::kDetachTransaction) {
      // Physical detach was rolled back / compensated, so the product is held again.
      object_held_ = true;
      physical_detach_applied_ = false;
    }
    // CommitDetachment rollback leaves the physical release in place; object_held_ stays false.
    if (cancel_requested_) {
      return transition_to(
        RestockTaskState::kPlanRetreat,
        "canceled transaction rolled back; retreat from the work area");
    }
    return begin_recovery(
      RestockTaskFault::kTransactionRolledBack,
      detail.empty() ? "child transaction rolled back" : std::move(detail));
  }
  if (event == RestockTaskEvent::kRetryableFailure) {
    if (state_ == RestockTaskState::kFault) {
      return transition_to(
        RestockTaskState::kRequestOperator,
        detail.empty() ? "fault-state operation failed; operator required" :
        std::move(detail));
    }
    if (is_execution_state(state_) && current_execution_may_have_started_) {
      fault_ = RestockTaskFault::kOperationFailed;
      canceled_motion_state_ = state_;
      return transition_to(
        RestockTaskState::kCancelActiveMotion,
        detail.empty() ? "execution failure requires verified stop before recovery" :
        std::move(detail));
    }
    const bool cleanup_boundary = state_ != RestockTaskState::kCancelActiveMotion &&
      state_ != RestockTaskState::kReleaseTask && !is_transaction_state(state_);
    if (cleanup_boundary && termination_requested() && !reservation_active_ &&
      !first_trajectory_may_have_started_)
    {
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kCanceled,
        "pre-motion operation ended after termination; no cleanup authority is active");
    }
    if (cleanup_boundary && termination_requested() && reservation_active_ &&
      !first_trajectory_may_have_started_ && !object_held_)
    {
      return transition_to(
        RestockTaskState::kReleaseTask,
        "pre-motion operation ended after termination; release the reservation");
    }
    return retry_or_recover(std::move(detail));
  }
  if (event == RestockTaskEvent::kTimeout ||
    event == RestockTaskEvent::kTerminalFailure)
  {
    const RestockTaskFault failure = event == RestockTaskEvent::kTimeout ?
      RestockTaskFault::kTimedOut : RestockTaskFault::kOperationFailed;
    if (is_execution_state(state_) && current_execution_may_have_started_) {
      fault_ = failure;
      canceled_motion_state_ = state_;
      return transition_to(
        RestockTaskState::kCancelActiveMotion,
        detail.empty() ? "execution ended without a verified stop" : std::move(detail));
    }
    if (state_ == RestockTaskState::kFault) {
      return transition_to(
        RestockTaskState::kRequestOperator,
        detail.empty() ? "fault handling could not establish a better safe state" :
        std::move(detail));
    }
    if (state_ == RestockTaskState::kRecover) {
      return begin_fault(
        RestockTaskFault::kRecoveryFailed,
        detail.empty() ? "deterministic recovery failed" : std::move(detail));
    }
    if (state_ == RestockTaskState::kCancelActiveMotion ||
      state_ == RestockTaskState::kReleaseTask || is_transaction_state(state_))
    {
      return begin_fault(
        RestockTaskFault::kExternalInconsistency,
        detail.empty() ? "safe task boundary could not be proven" : std::move(detail));
    }
    if (termination_requested() && !reservation_active_ &&
      !first_trajectory_may_have_started_)
    {
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(RestockTaskState::kCanceled, "canceled before any mutation or motion");
    }
    if (termination_requested() && reservation_active_ &&
      !first_trajectory_may_have_started_)
    {
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kReleaseTask,
        "canceled pre-motion operation ended; release the reservation");
    }
    return begin_recovery(
      failure, detail.empty() ? "task operation failed" : std::move(detail));
  }
  if (event != RestockTaskEvent::kOperationSucceeded) {
    return reject(RestockTaskFault::kInvalidEvent, "unsupported task event");
  }
  return handle_success();
}

RestockTaskTransition RestockTaskMachine::handle_success()
{
  const RestockTaskState completed = state_;
  if (is_execution_state(completed) && !current_execution_may_have_started_) {
    return reject(
      RestockTaskFault::kInvalidEvent,
      "execution success requires acceptance evidence for the current operation");
  }
  if (completed == RestockTaskState::kReserveTask) {
    reservation_active_ = true;
  } else if (completed == RestockTaskState::kAttachTransaction) {
    object_held_ = true;
    jaws_closed_on_unheld_product_ = false;
  } else if (completed == RestockTaskState::kDetachTransaction) {
    object_held_ = false;
    physical_detach_applied_ = true;
  } else if (completed == RestockTaskState::kCommitDetachment) {
    placement_committed_ = true;
  } else if (completed == RestockTaskState::kUpdateInventory) {
    if (reservation_active_) {
      // A reservation stays active until released with a terminal outcome; success releases
      // it like cancellation does, with a different outcome and end state.
      releasing_after_completion_ = true;
      return transition_to(
        RestockTaskState::kReleaseTask, "inventory updated; release the task reservation");
    }
  } else if (completed == RestockTaskState::kReleaseTask) {
    reservation_active_ = false;
    if (releasing_after_completion_) {
      return transition_to(
        RestockTaskState::kComplete, "reservation released after a verified placement");
    }
    if (!task_deadline_exceeded_ && !safe_abort_requested_) {
      fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
    }
    return transition_to(
      RestockTaskState::kCanceled,
      safe_abort_requested_ ? "reservation released after safe abort" :
      "reservation released after cancellation");
  } else if (completed == RestockTaskState::kOpenGripperForEscape) {
    // Milestone 10 §6 (Card 062): the jaws are off the unheld product; the route that was
    // diverted continues, and the motion that leaves the grasp carries the straight-line escape.
    // A termination that arrived during the open still owes the cleanup retreat.
    jaws_closed_on_unheld_product_ = false;
    const RestockTaskState resume =
      termination_requested() ? RestockTaskState::kPlanRetreat : grasp_escape_resume_state_;
    return transition_to(
      resume, "grasp escape: jaws opened to the approach clearance; leave the grasp");
  } else if (completed == RestockTaskState::kFault) {
    return transition_to(RestockTaskState::kRequestOperator, "motion inhibited; operator required");
  } else if (completed == RestockTaskState::kRecover) {
    if (termination_requested()) {
      if (first_trajectory_may_have_started_) {
        return transition_to(
          RestockTaskState::kPlanRetreat,
          "recovery reached a post-motion cancellation boundary");
      }
      if (reservation_active_) {
        return transition_to(
          RestockTaskState::kReleaseTask,
          "recovery reached a reserved pre-motion cancellation boundary");
      }
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kCanceled,
        "recovery reached a pre-reservation cancellation boundary");
    }
    // Clear the fault that triggered recovery, or later decisions (including the terminal
    // reservation release, which the world state refuses as a failure after placement) see a
    // failed task.
    fault_ = RestockTaskFault::kNone;
    return transition_to(recovery_resume_state_, "deterministic recovery completed");
  } else if (completed == RestockTaskState::kCancelActiveMotion) {
    if (!cancel_requested_) {
      const RestockTaskState resume = recovery_resume_for(canceled_motion_state_);
      auto transition = begin_recovery(
        fault_, "active motion stopped and verified before deterministic recovery");
      recovery_resume_state_ = resume;
      return transition;
    }
    if (canceled_motion_state_ == RestockTaskState::kExecuteRetreat) {
      if (object_held_) {
        if (!task_deadline_exceeded_) {
          fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
        }
        return transition_to(
          RestockTaskState::kCanceled,
          "retreat motion stopped with the held object in a verified safe state");
      }
      if (!placement_committed_ && physical_detach_applied_) {
        // Physical detach already happened; cancellation still owes the destination survey and
        // semantic commit before the reservation can be released.
        return transition_to(
          RestockTaskState::kSurveyDestination,
          "retreat motion stopped after physical detach; survey and commit the placement");
      }
      return transition_to(
        RestockTaskState::kReleaseTask,
        "retreat motion stopped; release the task reservation");
    }
    return transition_to(
      RestockTaskState::kPlanRetreat,
      "active motion stopped; plan a verified retreat");
  }

  if (termination_requested()) {
    if (completed == RestockTaskState::kValidateScene ||
      completed == RestockTaskState::kSelectPair)
    {
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(RestockTaskState::kCanceled, "canceled before reservation or motion");
    }
    if (completed == RestockTaskState::kReserveTask ||
      completed == RestockTaskState::kGenerateGrasps ||
      completed == RestockTaskState::kPlanPreGrasp)
    {
      // Milestone 10 §6 (Card 051): under the whole-task deadline an arm that already moved
      // gets the bounded cleanup retreat before the reservation is released; a user cancel
      // (no deadline) keeps stopping in place exactly as before.
      if (task_deadline_exceeded_ && first_trajectory_may_have_started_ &&
        reservation_active_)
      {
        return transition_to(
          RestockTaskState::kPlanRetreat,
          "whole-task deadline: the arm moved before the expiry; retreat to the safe pose "
          "before release");
      }
      return transition_to(
        RestockTaskState::kReleaseTask,
        "pre-motion cancellation reached a reservation-release boundary");
    }
    if (completed == RestockTaskState::kExecuteApproach ||
      completed == RestockTaskState::kExecuteInsert ||
      completed == RestockTaskState::kPlanApproach ||
      completed == RestockTaskState::kObserveDestination ||
      completed == RestockTaskState::kGeneratePlacement ||
      completed == RestockTaskState::kPlanCarryStart ||
      completed == RestockTaskState::kPlanPreInsert ||
      completed == RestockTaskState::kPlanInsert)
    {
      return transition_to(
        RestockTaskState::kPlanRetreat,
        "canceled operation reached an endpoint; retreat without further manipulation");
    }
    if (completed == RestockTaskState::kCloseGripper ||
      completed == RestockTaskState::kVerifyGrasp)
    {
      return begin_recovery(
        RestockTaskFault::kCanceled,
        "grasp-side cancellation requires deterministic gripper recovery");
    }
    if (completed == RestockTaskState::kAttachTransaction) {
      return transition_to(
        RestockTaskState::kPlanRetract,
        "attachment committed during cancellation; plan a verified retract");
    }
    if (completed == RestockTaskState::kPlanRetract) {
      return transition_to(
        RestockTaskState::kExecuteRetract,
        "attachment committed during cancellation; execute the verified retract");
    }
    if (completed == RestockTaskState::kExecuteRetract ||
      completed == RestockTaskState::kExecuteCarryStart ||
      completed == RestockTaskState::kExecutePreInsert)
    {
      return transition_to(
        RestockTaskState::kPlanRetreat, "cancellation requires a verified retreat");
    }
    if (completed == RestockTaskState::kOpenGripper) {
      return transition_to(
        RestockTaskState::kDetachTransaction,
        "open gripper must be followed by a reconciled detachment transaction");
    }
    if (completed == RestockTaskState::kDetachTransaction) {
      return transition_to(
        RestockTaskState::kPlanRetreat,
        "physical detachment completed during cancellation; retreat to survey the destination");
    }
    if (completed == RestockTaskState::kPlanRetreat) {
      return transition_to(
        RestockTaskState::kExecuteRetreat, "execute the cancellation retreat");
    }
    if (completed == RestockTaskState::kExecuteRetreat) {
      if (!object_held_ && physical_detach_applied_ && !placement_committed_) {
        return transition_to(
          RestockTaskState::kSurveyDestination,
          "retreat complete after physical detach; survey and commit before release");
      }
      if (reservation_active_ && !object_held_) {
        return transition_to(
          RestockTaskState::kReleaseTask, "safe retreat complete; release reservation");
      }
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kCanceled,
        "cancellation reached a safe terminal state");
    }
    if (completed == RestockTaskState::kSurveyDestination) {
      return transition_to(
        RestockTaskState::kCommitDetachment,
        "destination surveyed during cancellation; commit the placement");
    }
    if (completed == RestockTaskState::kCommitDetachment) {
      return transition_to(
        RestockTaskState::kVerifyPlacement,
        "detachment committed during cancellation; verify the resulting placement");
    }
    if (completed == RestockTaskState::kVerifyPlacement ||
      completed == RestockTaskState::kUpdateInventory)
    {
      if (reservation_active_ && !object_held_) {
        return transition_to(
          RestockTaskState::kReleaseTask, "safe retreat complete; release reservation");
      }
      if (!task_deadline_exceeded_ && !safe_abort_requested_) {
        fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
      }
      return transition_to(
        RestockTaskState::kCanceled,
        "cancellation reached a safe terminal state");
    }
  }

  const RestockTaskState next = next_success_state(completed);
  if (next == completed) {
    return reject(
      RestockTaskFault::kInvalidEvent, "operation success is invalid for the current task state");
  }
  return transition_to(next, std::string(to_string(completed)) + " completed");
}

RestockTaskTransition RestockTaskMachine::handle_cancel(std::string detail)
{
  cancel_requested_ = true;
  // Card 051 review blocker: an operator's stop-in-place revokes any cleanup-retreat authority
  // the whole-task deadline armed — even when the deadline fired first, a later user cancel
  // means no new motion.
  deadline_retreat_revoked_ = true;
  if (!task_deadline_exceeded_ && !safe_abort_requested_ &&
    state_ != RestockTaskState::kFault &&
    fault_ != RestockTaskFault::kExternalInconsistency)
  {
    fault_ = recoverable_skip_ ? fault_ : RestockTaskFault::kCanceled;
  }
  detail_ = detail.empty() ? "cancellation requested; waiting for a safe boundary" :
    std::move(detail);
  if (is_free_space_execution(state_) && current_execution_may_have_started_) {
    canceled_motion_state_ = state_;
    return transition_to(
      RestockTaskState::kCancelActiveMotion,
      "cancel active trajectory and verify the stopped robot state");
  }
  return status();
}

RestockTaskTransition RestockTaskMachine::transition_to(
  RestockTaskState next, std::string detail)
{
  if (next == RestockTaskState::kCloseGripper) {
    jaws_closed_on_unheld_product_ = true;
  }
  // Milestone 10 §6 (Card 062): every route that leaves a failed grasp — the recovery resume,
  // the recoverable skip, the deadline's cleanup retreat — opens the jaws first. An operator's
  // stop (cancel, drain, safe abort) keeps stopping in place: no jaw motion is added under it.
  const bool leaves_grasp = next == RestockTaskState::kGenerateGrasps ||
    next == RestockTaskState::kPlanPreGrasp || next == RestockTaskState::kPlanRetreat;
  const bool operator_stop = deadline_retreat_revoked_ ||
    (cancel_requested_ && !task_deadline_exceeded_);
  if (leaves_grasp && jaws_closed_on_unheld_product_ && !object_held_ && !operator_stop) {
    grasp_escape_resume_state_ = next;
    detail = "grasp escape: the jaws closed on a product that is not held; open them to the "
      "approach clearance before " + std::string(to_string(next)) +
      (detail.empty() ? std::string{} : " (" + detail + ")");
    next = RestockTaskState::kOpenGripperForEscape;
  }
  const RestockTaskState previous = state_;
  state_ = next;
  attempt_ = is_terminal(next) ? attempt_ : 1U;
  if (is_execution_state(next) || is_execution_state(previous)) {
    current_execution_may_have_started_ = false;
    current_execution_operation_generation_ = 0;
  }
  if (!detail.empty()) {
    detail_ = std::move(detail);
  }
  return status();
}

RestockTaskTransition RestockTaskMachine::begin_recovery(
  RestockTaskFault fault, std::string detail)
{
  if (recovery_attempt_ >= config_.max_recovery_attempts) {
    return begin_fault(
      RestockTaskFault::kRetryExhausted,
      detail.empty() ? "bounded recovery attempts exhausted" : std::move(detail));
  }
  recovery_resume_state_ = recovery_resume_for(state_);
  ++recovery_attempt_;
  fault_ = task_deadline_exceeded_ ? RestockTaskFault::kTimedOut :
    safe_abort_requested_ ? RestockTaskFault::kOperationFailed :
    cancel_requested_ ? RestockTaskFault::kCanceled : fault;
  return transition_to(RestockTaskState::kRecover, std::move(detail));
}

RestockTaskTransition RestockTaskMachine::begin_fault(RestockTaskFault fault, std::string detail)
{
  fault_ = fault;
  return transition_to(RestockTaskState::kFault, std::move(detail));
}

RestockTaskTransition RestockTaskMachine::reject(
  RestockTaskFault fault, std::string detail) const
{
  return {
    false, state_, command_for(state_), fault, attempt_, recovery_attempt_,
    timeout_for(state_, config_), config_.total_timeout, cancel_requested_,
    safe_abort_requested_, task_deadline_exceeded_, recoverable_skip_,
    deadline_retreat_revoked_,
    first_trajectory_may_have_started_,
    current_execution_may_have_started_, current_execution_operation_generation_,
    object_held_, reservation_active_,
    std::move(detail)};
}

RestockTaskTransition RestockTaskMachine::retry_or_recover(std::string detail)
{
  if (state_ == RestockTaskState::kRecover) {
    return begin_fault(
      RestockTaskFault::kRecoveryFailed,
      detail.empty() ? "recovery operation failed" : std::move(detail));
  }
  if (state_ == RestockTaskState::kCancelActiveMotion ||
    state_ == RestockTaskState::kReleaseTask || is_transaction_state(state_))
  {
    return begin_fault(
      RestockTaskFault::kExternalInconsistency,
      detail.empty() ? "safe-boundary operation cannot be retried by the task machine" :
      std::move(detail));
  }
  const std::size_t limit = config_.max_operation_retries + 1U;
  if (is_execution_state(state_)) {
    current_execution_may_have_started_ = false;
    current_execution_operation_generation_ = 0;
  }
  if (attempt_ < limit) {
    ++attempt_;
    detail_ = detail.empty() ? "retry bounded task operation" : std::move(detail);
    return status();
  }
  return begin_recovery(
    RestockTaskFault::kRetryExhausted,
    detail.empty() ? "bounded task operation retries exhausted" : std::move(detail));
}

RestockTaskTransition RestockTaskMachine::status() const
{
  return {
    true, state_, command_for(state_), fault_, attempt_, recovery_attempt_,
    timeout_for(state_, config_), config_.total_timeout, cancel_requested_,
    safe_abort_requested_, task_deadline_exceeded_, recoverable_skip_,
    deadline_retreat_revoked_,
    first_trajectory_may_have_started_,
    current_execution_may_have_started_, current_execution_operation_generation_,
    object_held_, reservation_active_, detail_};
}

std::size_t RestockTaskMachine::selection_restarts() const noexcept
{
  return selection_restart_;
}

bool RestockTaskMachine::termination_requested() const noexcept
{
  // The recoverable skip is a termination-shaped cleanup (Milestone 10 §6 rung 5, Card 051):
  // the success routing that walks retreat → survey/commit → release must engage for it, while
  // the driver's short-circuit and identity guard read the transition flag themselves.
  return cancel_requested_ || safe_abort_requested_ || recoverable_skip_;
}

bool RestockTaskMachine::terminal() const noexcept
{
  return is_terminal(state_);
}

bool RestockTaskMachine::motion_inhibited() const noexcept
{
  return state_ == RestockTaskState::kFault || state_ == RestockTaskState::kRequestOperator;
}

const char * to_string(RestockTaskState state) noexcept
{
  switch (state) {
    case RestockTaskState::kIdle: return "idle";
    case RestockTaskState::kValidateScene: return "validate_scene";
    case RestockTaskState::kSelectPair: return "select_pair";
    case RestockTaskState::kReserveTask: return "reserve_task";
    case RestockTaskState::kGenerateGrasps: return "generate_grasps";
    case RestockTaskState::kPlanPreGrasp: return "plan_pre_grasp";
    case RestockTaskState::kExecutePreGrasp: return "execute_pre_grasp";
    case RestockTaskState::kOpenGripperForApproach: return "open_gripper_for_approach";
    case RestockTaskState::kPlanApproach: return "plan_approach";
    case RestockTaskState::kExecuteApproach: return "execute_approach";
    case RestockTaskState::kCloseGripper: return "close_gripper";
    case RestockTaskState::kVerifyGrasp: return "verify_grasp";
    case RestockTaskState::kAttachTransaction: return "attach_transaction";
    case RestockTaskState::kPlanRetract: return "plan_retract";
    case RestockTaskState::kExecuteRetract: return "execute_retract";
    case RestockTaskState::kObserveDestination: return "observe_destination";
    case RestockTaskState::kGeneratePlacement: return "generate_placement";
    case RestockTaskState::kPlanCarryStart: return "plan_carry_start";
    case RestockTaskState::kExecuteCarryStart: return "execute_carry_start";
    case RestockTaskState::kPlanPreInsert: return "plan_pre_insert";
    case RestockTaskState::kExecutePreInsert: return "execute_pre_insert";
    case RestockTaskState::kPlanInsert: return "plan_insert";
    case RestockTaskState::kExecuteInsert: return "execute_insert";
    case RestockTaskState::kOpenGripper: return "open_gripper";
    case RestockTaskState::kDetachTransaction: return "detach_transaction";
    case RestockTaskState::kPlanRetreat: return "plan_retreat";
    case RestockTaskState::kExecuteRetreat: return "execute_retreat";
    case RestockTaskState::kSurveyDestination: return "survey_destination";
    case RestockTaskState::kCommitDetachment: return "commit_detachment";
    case RestockTaskState::kVerifyPlacement: return "verify_placement";
    case RestockTaskState::kUpdateInventory: return "update_inventory";
    case RestockTaskState::kCancelActiveMotion: return "cancel_active_motion";
    case RestockTaskState::kReleaseTask: return "release_task";
    case RestockTaskState::kRecover: return "recover";
    case RestockTaskState::kFault: return "fault";
    case RestockTaskState::kRequestOperator: return "request_operator";
    case RestockTaskState::kComplete: return "complete";
    case RestockTaskState::kCanceled: return "canceled";
    case RestockTaskState::kOpenGripperForEscape: return "open_gripper_for_escape";
  }
  return "unknown";
}

const char * to_string(RestockTaskCommand command) noexcept
{
  switch (command) {
    case RestockTaskCommand::kNone: return "none";
    case RestockTaskCommand::kValidateScene: return "validate_scene";
    case RestockTaskCommand::kSelectPair: return "select_pair";
    case RestockTaskCommand::kReserveTask: return "reserve_task";
    case RestockTaskCommand::kGenerateGrasps: return "generate_grasps";
    case RestockTaskCommand::kPlanPreGrasp: return "plan_pre_grasp";
    case RestockTaskCommand::kExecutePreGrasp: return "execute_pre_grasp";
    case RestockTaskCommand::kPlanApproach: return "plan_approach";
    case RestockTaskCommand::kExecuteApproach: return "execute_approach";
    case RestockTaskCommand::kCloseGripper: return "close_gripper";
    case RestockTaskCommand::kVerifyGrasp: return "verify_grasp";
    case RestockTaskCommand::kRunAttachTransaction: return "run_attach_transaction";
    case RestockTaskCommand::kPlanRetract: return "plan_retract";
    case RestockTaskCommand::kExecuteRetract: return "execute_retract";
    case RestockTaskCommand::kObserveDestination: return "observe_destination";
    case RestockTaskCommand::kGeneratePlacement: return "generate_placement";
    case RestockTaskCommand::kPlanCarryStart: return "plan_carry_start";
    case RestockTaskCommand::kExecuteCarryStart: return "execute_carry_start";
    case RestockTaskCommand::kPlanPreInsert: return "plan_pre_insert";
    case RestockTaskCommand::kExecutePreInsert: return "execute_pre_insert";
    case RestockTaskCommand::kPlanInsert: return "plan_insert";
    case RestockTaskCommand::kExecuteInsert: return "execute_insert";
    case RestockTaskCommand::kOpenGripper: return "open_gripper";
    case RestockTaskCommand::kRunDetachTransaction: return "run_detach_transaction";
    case RestockTaskCommand::kPlanRetreat: return "plan_retreat";
    case RestockTaskCommand::kExecuteRetreat: return "execute_retreat";
    case RestockTaskCommand::kSurveyDestination: return "survey_destination";
    case RestockTaskCommand::kRunCommitDetachment: return "run_commit_detachment";
    case RestockTaskCommand::kVerifyPlacement: return "verify_placement";
    case RestockTaskCommand::kUpdateInventory: return "update_inventory";
    case RestockTaskCommand::kCancelMotionAndVerifyStop:
      return "cancel_motion_and_verify_stop";
    case RestockTaskCommand::kReleaseTaskReservation: return "release_task_reservation";
    case RestockTaskCommand::kExecuteRecovery: return "execute_recovery";
    case RestockTaskCommand::kInhibitMotion: return "inhibit_motion";
    case RestockTaskCommand::kRequestOperator: return "request_operator";
  }
  return "unknown";
}

const char * to_string(RestockTaskFault fault) noexcept
{
  switch (fault) {
    case RestockTaskFault::kNone: return "none";
    case RestockTaskFault::kCanceled: return "canceled";
    case RestockTaskFault::kOperationFailed: return "operation_failed";
    case RestockTaskFault::kRetryExhausted: return "retry_exhausted";
    case RestockTaskFault::kTimedOut: return "timed_out";
    case RestockTaskFault::kTransactionRolledBack: return "transaction_rolled_back";
    case RestockTaskFault::kExternalInconsistency: return "external_inconsistency";
    case RestockTaskFault::kRecoveryFailed: return "recovery_failed";
    case RestockTaskFault::kInvalidConfiguration: return "invalid_configuration";
    case RestockTaskFault::kInvalidEvent: return "invalid_event";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
