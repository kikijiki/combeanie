// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_coordinator_driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp_action/rclcpp_action.hpp>

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/generation_scoped_inbox_deposit.hpp"
#include "restocker_task_executor/transfer_path_constraints.hpp"
#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr std::size_t kMaximumAdvanceSteps = 64U;

[[nodiscard]] std::size_t checked_ingress_budget(std::size_t ordinary_capacity)
{
  constexpr std::array<std::size_t, 4U> additions{
    CoordinatorInbox::kOverflowMarkerCapacity,
    CoordinatorInbox::kAcceptedHandoffEmergencyCapacity,
    CoordinatorInbox::kCleanupEvidenceLossMarkerCapacity,
    CoordinatorInbox::kCleanupEmergencyCapacity};
  auto result = ordinary_capacity;
  for (const auto addition : additions) {
    if (addition > std::numeric_limits<std::size_t>::max() - result) {
      throw std::invalid_argument("coordinator inbox pump budget overflows size_t");
    }
    result += addition;
  }
  return result;
}

[[nodiscard]] RestockActionOutcome selection_outcome(SelectionErrorCode code) noexcept
{
  switch (code) {
    case SelectionErrorCode::InvalidConfiguration:
    case SelectionErrorCode::InvalidSnapshot:
    case SelectionErrorCode::MissingGeometry:
      return RestockActionOutcome::kValidationFailed;
    // Aged-out evidence gets a distinct status but the same reservation semantics as
    // kNoCompatiblePair (safe terminal, no inhibition).
    case SelectionErrorCode::ObjectStale:
    case SelectionErrorCode::LaneStale:
    case SelectionErrorCode::RobotStale:
    case SelectionErrorCode::PerceptionUnlive:
      return RestockActionOutcome::kObservationEvidenceStale;
    case SelectionErrorCode::RequestedIdentityIncomplete:
    case SelectionErrorCode::ObjectNotFound:
    case SelectionErrorCode::LaneNotFound:
    case SelectionErrorCode::ObjectUnavailable:
    case SelectionErrorCode::RobotUnavailable:
    case SelectionErrorCode::UnsupportedProduct:
    case SelectionErrorCode::ObjectOutsideStock:
    case SelectionErrorCode::LaneUnavailable:
    case SelectionErrorCode::LaneEvidenceInvalidated:
    case SelectionErrorCode::IncompatiblePair:
    // The ledger names products this lane's policy no longer accepts: the lane is skipped and
    // the typed detail carries the conflict. A safe terminal on the existing no-compatible-pair
    // path — nothing is mutated, and the operator already has the policy and ledger events.
    case SelectionErrorCode::LaneWrongProduct:
    case SelectionErrorCode::InsufficientLaneDepth:
    case SelectionErrorCode::NoEligiblePair:
      return RestockActionOutcome::kNoCompatiblePair;
  }
  return RestockActionOutcome::kInternalError;
}

[[nodiscard]] bool internal_grasp_error(GraspCandidateErrorCode code) noexcept
{
  switch (code) {
    case GraspCandidateErrorCode::InvalidConfiguration:
    case GraspCandidateErrorCode::InvalidToolTransform:
    case GraspCandidateErrorCode::InvalidCandidateBatch:
    case GraspCandidateErrorCode::CandidateIntentChanged:
      return true;
    case GraspCandidateErrorCode::InvalidObject:
    case GraspCandidateErrorCode::NoValidJawTarget:
    case GraspCandidateErrorCode::StaleEvidence:
    case GraspCandidateErrorCode::ReservationMismatch:
    case GraspCandidateErrorCode::SelectionChanged:
      return false;
  }
  return true;
}

[[nodiscard]] bool valid_configuration(const RestockCoordinatorDriverConfig & config)
{
  return config.inbox_capacity > 0U && !config.planning_frame.empty() &&
         config.maximum_object_age.count() >= 0 &&
         config.maximum_robot_age.count() >= 0 &&
         config.maximum_future_skew.count() >= 0 &&
         std::isfinite(config.pregrasp_continuation_vertical_margin_m) &&
         config.pregrasp_continuation_vertical_margin_m >= 0.0 &&
         config.task.max_recovery_attempts > 0U &&
         config.task.validation_timeout.count() > 0 &&
         config.task.planning_timeout.count() > 0 &&
         config.task.execution_timeout.count() > 0 &&
         config.task.transaction_timeout.count() > 0 &&
         config.task.recovery_timeout.count() > 0 &&
         config.task.total_timeout.count() > 0 &&
         config.reconciliation.window.count() > 0 &&
         config.reconciliation.attempt_timeout.count() > 0 &&
         config.reconciliation.max_attempts > 0U &&
         config.perception_reacquire_timeout.count() >= 0 &&
         config.perception_reacquire_timeout <
         config.task.planning_timeout + config.task.execution_timeout &&
         config.recovery_stop_settle_timeout.count() >= 0 &&
         config.recovery_stop_settle_timeout < config.task.recovery_timeout &&
         config.recovery_stop_settle_poll.count() > 0 &&
         std::isfinite(config.reasoner_minimum_confidence) &&
         config.reasoner_minimum_confidence >= 0.0 &&
         config.reasoner_minimum_confidence <= 1.0;
}

[[nodiscard]] bool termination_latch_succeeded(
  const GoalTerminationLatchDecision & decision) noexcept
{
  return decision.status() == GoalTerminationLatchStatus::kLatched ||
         decision.status() == GoalTerminationLatchStatus::kAlreadyLatched;
}

[[nodiscard]] bool valid_accepted_termination_state(
  const GoalAdmissionSnapshot & admission,
  const CoordinatorAcceptedGoal & accepted,
  bool allow_missing_drain_notice_for_cleanup = false) noexcept
{
  if (accepted.drain_requested_at && *accepted.drain_requested_at == SteadyTime::max()) {
    return false;
  }
  const bool coherent_policy = [&admission]() {
    switch (admission.termination_intent) {
      case GoalTerminationIntent::kNone:
        return !admission.cancel_requested && !admission.safe_abort_requested &&
               !admission.task_deadline_exceeded;
      case GoalTerminationIntent::kUserCancel:
        return admission.cancel_requested && !admission.safe_abort_requested &&
               !admission.task_deadline_exceeded;
      case GoalTerminationIntent::kShutdownDrain:
        return admission.safe_abort_requested && !admission.task_deadline_exceeded;
      case GoalTerminationIntent::kSafeAbort:
        return admission.safe_abort_requested && !admission.task_deadline_exceeded;
      case GoalTerminationIntent::kTaskDeadline:
        return admission.task_deadline_exceeded;
    }
    return false;
  }();
  if (!coherent_policy) {
    return false;
  }
  if (!admission.first_termination) {
    return !accepted.drain_requested_at &&
           admission.termination_intent == GoalTerminationIntent::kNone;
  }
  const auto & first = *admission.first_termination;
  if (validate_goal_termination_record(first) != GoalTerminationValidationError::kNone ||
    first.goal_id != accepted.goal_id ||
    first.goal_generation != accepted.goal_generation)
  {
    return false;
  }
  switch (first.kind) {
    case GoalTerminationKind::kUserCancel:
      if (!admission.cancel_requested) {
        return false;
      }
      break;
    case GoalTerminationKind::kCoordinatorDrain:
      if (!admission.safe_abort_requested ||
        (accepted.drain_requested_at && *accepted.drain_requested_at != first.arrived_at) ||
        (!accepted.drain_requested_at && !accepted.sealed_pending_control_handoff &&
        !allow_missing_drain_notice_for_cleanup))
      {
        return false;
      }
      break;
    case GoalTerminationKind::kShutdown:
    case GoalTerminationKind::kSafeAbort:
    case GoalTerminationKind::kAuthorityLoss:
    case GoalTerminationKind::kInboxOverflow:
    case GoalTerminationKind::kProtocolFailure:
      if (!admission.safe_abort_requested) {
        return false;
      }
      break;
    case GoalTerminationKind::kTaskDeadline:
      if (!admission.task_deadline_exceeded) {
        return false;
      }
      break;
    case GoalTerminationKind::kMotionDeadline:
      return false;
  }
  return !accepted.drain_requested_at || admission.safe_abort_requested ||
         (first.kind == GoalTerminationKind::kUserCancel && admission.cancel_requested &&
         admission.termination_intent == GoalTerminationIntent::kUserCancel);
}

[[nodiscard]] bool valid_pristine_generation_gate(
  const CoordinatorGenerationQuiescenceSnapshot & gate,
  const CoordinatorAcceptedGoal & accepted,
  const GoalAdmissionSnapshot & admission) noexcept
{
  return admission.goal_id && gate.goal_id == accepted.goal_id &&
         gate.goal_id == *admission.goal_id && gate.generation == accepted.goal_generation &&
         gate.generation == admission.generation && !gate.sealed &&
         gate.active_deposit_count == 0U && !gate.deposit_unresolved && !gate.adapter_fault &&
         gate.retirement_fence_state == GenerationRetirementFenceState::kHealthy &&
         !gate.probe_outstanding && !gate.receipt_outstanding && gate.accepted_ack_cookie == 0U;
}

[[nodiscard]] bool generation_gate_blocks_forward_work(
  const std::unique_ptr<RestockGoalContext> & context)
{
  if (!context) {
    return false;
  }
  try {
    const auto gate = context->generation_quiescence()->snapshot();
    return gate.adapter_fault || gate.deposit_unresolved ||
           gate.retirement_fence_state == GenerationRetirementFenceState::kFailed;
  } catch (...) {
    (void)context->generation_quiescence()->mark_synchronization_failure();
    return true;
  }
}

// Segments travelled while the product is coupled to the gripper (attach commits before the
// retract, detach commits before the retreat).
[[nodiscard]] bool carries_product(MotionSegment segment) noexcept
{
  switch (segment) {
    case MotionSegment::kRetract:
    case MotionSegment::kCarryStart:
    case MotionSegment::kPreInsert:
    case MotionSegment::kInsert:
      return true;
    case MotionSegment::kPreGrasp:
    case MotionSegment::kApproach:
    case MotionSegment::kRetreat:
      return false;
  }
  return false;
}

}  // namespace

const char * motion_segment_name(MotionSegment segment) noexcept
{
  switch (segment) {
    case MotionSegment::kPreGrasp:
      return "pre-grasp";
    case MotionSegment::kApproach:
      return "approach";
    case MotionSegment::kRetract:
      return "retract";
    case MotionSegment::kCarryStart:
      return "carry-start";
    case MotionSegment::kPreInsert:
      return "pre-insert";
    case MotionSegment::kInsert:
      return "insert";
    case MotionSegment::kRetreat:
      return "retreat";
  }
  return "unknown";
}

bool linear_motion_segment(MotionSegment segment) noexcept
{
  switch (segment) {
    case MotionSegment::kApproach:
    case MotionSegment::kRetract:
    case MotionSegment::kCarryStart:
    case MotionSegment::kInsert:
    case MotionSegment::kRetreat:
      // Straight-line slides with millimetres of side clearance; a sampling planner could swing
      // a finger through the product.
      return true;
    case MotionSegment::kPreGrasp:
    case MotionSegment::kPreInsert:
      // Free-space repositioning; a straight line is not required or usually reachable.
      return false;
  }
  return false;
}

std::optional<MotionSegment> planned_motion_segment(RestockTaskCommand command) noexcept
{
  switch (command) {
    case RestockTaskCommand::kPlanPreGrasp:
      return MotionSegment::kPreGrasp;
    case RestockTaskCommand::kPlanApproach:
      return MotionSegment::kApproach;
    case RestockTaskCommand::kPlanRetract:
      return MotionSegment::kRetract;
    case RestockTaskCommand::kPlanCarryStart:
      return MotionSegment::kCarryStart;
    case RestockTaskCommand::kPlanPreInsert:
      return MotionSegment::kPreInsert;
    case RestockTaskCommand::kPlanInsert:
      return MotionSegment::kInsert;
    case RestockTaskCommand::kPlanRetreat:
      return MotionSegment::kRetreat;
    default:
      return std::nullopt;
  }
}

std::optional<MotionSegment> executed_motion_segment(RestockTaskCommand command) noexcept
{
  switch (command) {
    case RestockTaskCommand::kExecutePreGrasp:
      return MotionSegment::kPreGrasp;
    case RestockTaskCommand::kExecuteApproach:
      return MotionSegment::kApproach;
    case RestockTaskCommand::kExecuteRetract:
      return MotionSegment::kRetract;
    case RestockTaskCommand::kExecuteCarryStart:
      return MotionSegment::kCarryStart;
    case RestockTaskCommand::kExecutePreInsert:
      return MotionSegment::kPreInsert;
    case RestockTaskCommand::kExecuteInsert:
      return MotionSegment::kInsert;
    case RestockTaskCommand::kExecuteRetreat:
      return MotionSegment::kRetreat;
    default:
      return std::nullopt;
  }
}

namespace
{

[[nodiscard]] const char * routing_guard_state_name(
  MotionWorkflowRoutingGuardState state) noexcept
{
  switch (state) {
    case MotionWorkflowRoutingGuardState::kGeneric:
      return "generic";
    case MotionWorkflowRoutingGuardState::kMotionOwned:
      return "motion-owned";
    case MotionWorkflowRoutingGuardState::kOwnershipUncertain:
      return "ownership-uncertain";
  }
  return "unknown";
}

[[nodiscard]] const char * deposit_outcome_name(
  GenerationScopedInboxDepositOutcome outcome) noexcept
{
  switch (outcome) {
    case GenerationScopedInboxDepositOutcome::kDeposited:
      return "deposited";
    case GenerationScopedInboxDepositOutcome::kUnresolved:
      return "unresolved";
    case GenerationScopedInboxDepositOutcome::kSealed:
      return "sealed";
    case GenerationScopedInboxDepositOutcome::kFailed:
      return "failed";
  }
  return "unknown";
}

// Deposits an asynchronous completion and reports when it does not land (the goal would then end
// at its command deadline). Best-effort, never throws: runs on a port's thread inside a callback.
template<typename Deposit>
void deposit_and_report(
  const CoordinatorAsyncDepositDiagnostic & diagnostic, const char * kind,
  const std::shared_ptr<CoordinatorGenerationQuiescence> & gate,
  const CoordinatorGoalId & goal_id, GoalGeneration generation, Deposit && deposit) noexcept
{
  const auto result =
    deposit_for_generation(gate, goal_id, generation, std::forward<Deposit>(deposit));
  if (result.outcome == GenerationScopedInboxDepositOutcome::kDeposited || !diagnostic) {
    return;
  }
  try {
    diagnostic(
      std::string(kind) + " completion was not deposited (" +
      deposit_outcome_name(result.outcome) + "), so the pump will never see it and goal "
      "generation " + std::to_string(generation) +
      " can only end at its command deadline");
  } catch (...) {
    // An undeliverable diagnostic must not take the port's worker thread down.
  }
}

}  // namespace

RestockCoordinatorDriver::RestockCoordinatorDriver(
  GoalAdmissionSlot & admission, WorldStateCoordinatorPort & world_state,
  CoordinatorTaskSelector selector, CoordinatorGraspAuthorityProvider grasp_authority,
  CoordinatorGraspGenerator grasp_generator,
  CoordinatorSteadyNow steady_now, CoordinatorRosNow ros_now,
  RestockCoordinatorDriverConfig config,
  CoordinatorSteadyNow async_evidence_now,
  CoordinatorPlacementGenerator placement_generator,
  MotionPort * motion, GripperPort * gripper, AttachmentPort * attachment,
  restocker_reasoner::RecoveryAdvisorPort * recovery_advisor,
  restocker_reasoner::RecoveryAuditLog * recovery_audit,
  CoordinatorRetreatTargetProvider retreat_target,
  CoordinatorLaneEvidenceInvalidator invalidate_lane_evidence,
  CoordinatorDestinationObservationAcquirer acquire_destination_observation,
  CoordinatorPerceptionLivenessCheck perception_liveness,
  CoordinatorAsyncDepositDiagnostic async_deposit_diagnostic)
: admission_(admission),
  termination_router_(admission_),
  world_state_(world_state),
  placement_generator_(std::move(placement_generator)),
  retreat_target_(std::move(retreat_target)),
  invalidate_lane_evidence_(std::move(invalidate_lane_evidence)),
  acquire_destination_observation_(std::move(acquire_destination_observation)),
  perception_liveness_(std::move(perception_liveness)),
  motion_(motion),
  gripper_(gripper),
  attachment_(attachment),
  recovery_advisor_(recovery_advisor),
  recovery_audit_(recovery_audit),
  recovery_advice_mailbox_(
    recovery_advisor == nullptr ?
    nullptr : std::make_shared<restocker_reasoner::RecoveryAdviceMailbox>()),
  selector_(std::move(selector)),
  grasp_authority_(std::move(grasp_authority)),
  grasp_generator_(std::move(grasp_generator)),
  steady_now_(std::move(steady_now)),
  async_evidence_now_(
    async_evidence_now ? std::move(async_evidence_now) : steady_now_),
  async_deposit_diagnostic_(std::move(async_deposit_diagnostic)),
  ros_now_(std::move(ros_now)),
  config_(std::move(config)),
  ingress_budget_(checked_ingress_budget(config_.inbox_capacity)),
  inbox_(std::make_shared<CoordinatorInbox>(config_.inbox_capacity)),
  ledger_(config_.reconciliation)
{
  if (!selector_ || !grasp_authority_ || !grasp_generator_ || !steady_now_ ||
    !async_evidence_now_ || !ros_now_ ||
    !valid_configuration(config_))
  {
    throw std::invalid_argument("coordinator driver configuration and ports must be valid");
  }
}

std::shared_ptr<CoordinatorInbox> RestockCoordinatorDriver::inbox() const noexcept
{
  return inbox_;
}

void RestockCoordinatorDriver::pump(SteadyTime now)
{
  if (guard_all_generic_routes()) {
    return;
  }
  const auto observe_generation_gate = [this, now]() {
    if (!generation_gate_blocks_forward_work(context_)) {
      return false;
    }
    inhibit("generation quiescence gate is failed or unresolved", now);
    return true;
  };
  const bool entry_gate_fault = observe_generation_gate();

  std::optional<CoordinatorInboxDelivery> fenced_drain_delivery;
  for (std::size_t processed = 0U; processed < ingress_budget_; ++processed) {
    auto delivery = inbox_->try_pop();
    if (!delivery) {
      break;
    }
    if (pending_accepted_drain_fence_) {
      if (!validate_pending_accepted_drain_fence(*delivery)) {
        reject_pending_accepted_drain_fence(
          "sealed accepted-goal handoff was not followed by its exact drain control", now);
        break;
      }
      fenced_drain_delivery.emplace(std::move(*delivery));
      break;
    }
    process(std::move(*delivery), now);
    // Re-check after each delivery: an adapter may have changed motion authority synchronously.
    if (guard_all_generic_routes()) {
      return;
    }
  }
  if (fenced_drain_delivery) {
    pending_accepted_drain_fence_.reset();
    task_deadline_ = bounded_deadline(context_->steady_started(), config_.task.total_timeout);
    observe_transition(context_->task_status(), now);
    process(std::move(*fenced_drain_delivery), now);
    if (guard_all_generic_routes()) {
      return;
    }
  } else if (pending_accepted_drain_fence_) {
    reject_pending_accepted_drain_fence(
      "sealed accepted-goal handoff exhausted its pump without the exact drain control", now);
  }
  if (inbox_->snapshot().generation_accounting_conflict) {
    if (context_) {
      inhibit("coordinator inbox generation accounting conflict", now);
    } else {
      admission_.inhibit("coordinator inbox generation accounting conflict");
    }
    return;
  }
  const bool post_drain_gate_fault =
    !entry_gate_fault && observe_generation_gate();
  if (entry_gate_fault || post_drain_gate_fault) {
    // Forward phases (planner polling, deadlines) are skipped; advance() still does the
    // bounded cancellation and reservation cleanup that inhibit() requires.
    advance(now);
    finish_if_terminal(now);
    release_acknowledged_terminal();
    return;
  }
  check_deadlines(now);
  advance(now);
  finish_if_terminal(now);
  release_acknowledged_terminal();
}

bool RestockCoordinatorDriver::validate_pending_accepted_drain_fence(
  const CoordinatorInboxDelivery & delivery) const
{
  const auto * control = std::get_if<CoordinatorControlEvent>(&delivery);
  const auto & fence = *pending_accepted_drain_fence_;
  if (!control || control->kind != CoordinatorControlKind::kDrainRequested ||
    control->goal_generation != fence.goal_generation || control->operation_generation != 0U ||
    control->arrived_at != fence.arrived_at || !context_ ||
    context_->generation() != fence.goal_generation || !fence.first_termination)
  {
    return false;
  }
  const auto admission = admission_.snapshot();
  if (admission.phase != GoalSlotPhase::kActive || admission.inhibited ||
    admission.mutation_submission || !admission.goal_id ||
    *admission.goal_id != context_->goal_id() || admission.generation != fence.goal_generation ||
    admission.first_termination.get() != fence.first_termination.get() ||
    validate_goal_termination_record(*fence.first_termination) !=
    GoalTerminationValidationError::kNone ||
    fence.first_termination->kind != GoalTerminationKind::kCoordinatorDrain ||
    fence.first_termination->goal_id != context_->goal_id() ||
    fence.first_termination->goal_generation != fence.goal_generation ||
    fence.first_termination->arrived_at != fence.arrived_at)
  {
    return false;
  }
  return true;
}

void RestockCoordinatorDriver::reject_pending_accepted_drain_fence(
  std::string detail, SteadyTime now)
{
  pending_accepted_drain_fence_.reset();
  primary_outcome_ = RestockActionOutcome::kExternalInconsistency;
  primary_detail_ = detail;
  inhibit(std::move(detail), now);
}

std::vector<CoordinatorDriverOutput> RestockCoordinatorDriver::take_outputs()
{
  std::vector<CoordinatorDriverOutput> result;
  result.swap(outputs_);
  return result;
}

bool RestockCoordinatorDriver::acknowledge_terminal_delivery(GoalGeneration generation)
{
  if (guard_generic_cleanup()) {
    return false;
  }
  if (!context_ || !terminal_output_emitted_ ||
    context_->generation() != generation || terminal_delivery_acknowledged_)
  {
    return false;
  }
  terminal_delivery_acknowledged_ = true;
  release_acknowledged_terminal();
  return true;
}

RestockCoordinatorDriverSnapshot RestockCoordinatorDriver::snapshot() const
{
  const auto admission = admission_.snapshot();
  const auto pending_operation = ledger_.pending();
  const bool reservation_capability_may_remain =
    (context_ && context_->reservation().has_value() && !context_->release_proof()) ||
    (pending_operation && pending_operation->effect == OperationEffect::kIdempotentMutation);
  return {
    context_ != nullptr, terminal_output_emitted_, inhibited_ || admission.inhibited,
    context_ ? context_->generation() : 0U,
    context_ ? std::optional<RestockTaskTransition>(context_->task_status()) : std::nullopt,
    pending_operation, request_handles_.size(), reservation_capability_may_remain,
    context_ && context_->grasp_candidate_batch() ?
    context_->grasp_candidate_batch()->candidates.size() : 0U,
    command_deadline_, task_deadline_, inactive_after_terminal_ack_generation_,
    motion_workflow_routing_state(), guarded_generic_timeout_count_,
    guarded_generic_advance_count_, guarded_generic_cleanup_count_,
    perception_reacquire_.has_value(), selection_reacquire_.has_value()};
}

MotionWorkflowRoutingGuardState
RestockCoordinatorDriver::motion_workflow_routing_state() const noexcept
{
  try {
    const auto pending = ledger_.pending();
    bool owned = false;
    bool uncertain = false;

    MotionAdmissionRoutingOwnershipStatus admission_ownership =
      MotionAdmissionRoutingOwnershipStatus::kNotOwned;
    if (context_) {
      admission_ownership = admission_.observe_motion_routing_ownership(
        context_->goal_id(), context_->generation(), pending ? &*pending : nullptr,
        ledger_.issuer_binding());
    } else {
      const auto admission = admission_.snapshot();
      if (admission.motion_workflow_active || admission.motion_registration ||
        admission.motion_request_bound || admission.motion_submission_state_present)
      {
        admission_ownership = MotionAdmissionRoutingOwnershipStatus::kInactive;
      }
    }
    const bool admission_owned =
      admission_ownership == MotionAdmissionRoutingOwnershipStatus::kOwned;
    owned = owned || admission_owned;
    uncertain = uncertain ||
      (admission_ownership != MotionAdmissionRoutingOwnershipStatus::kNotOwned &&
      admission_ownership != MotionAdmissionRoutingOwnershipStatus::kOwned);

    // A pending cancelable-motion operation does not imply a second consumer; only
    // admission-registered motion can fence the generic routes.
    (void)pending;

    if (uncertain) {
      return MotionWorkflowRoutingGuardState::kOwnershipUncertain;
    }
    return owned ? MotionWorkflowRoutingGuardState::kMotionOwned :
           MotionWorkflowRoutingGuardState::kGeneric;
  } catch (...) {
    // An uninspectable authority is not evidence of no ownership.
    return MotionWorkflowRoutingGuardState::kOwnershipUncertain;
  }
}

// A guarded route makes no progress and looks idle, so each guard reports the first refusal and
// repeats it periodically.
void RestockCoordinatorDriver::report_guarded_route(
  const char * route, MotionWorkflowRoutingGuardState state) noexcept
{
  constexpr std::size_t kRepeatEvery = 200U;
  const bool changed = state != last_reported_routing_state_;
  if (!changed && guarded_route_reports_ % kRepeatEvery != 0U) {
    ++guarded_route_reports_;
    return;
  }
  if (changed) {
    guarded_route_reports_ = 0U;
    last_reported_routing_state_ = state;
  }
  ++guarded_route_reports_;
  if (!async_deposit_diagnostic_) {
    return;
  }
  try {
    async_deposit_diagnostic_(
      std::string("coordinator refused its ") + route + " route: motion workflow routing is " +
      routing_guard_state_name(state) + " rather than generic, so the pump makes no progress "
      "while this holds");
  } catch (...) {
    // An undeliverable diagnostic must not stop the pump.
  }
}

bool RestockCoordinatorDriver::guard_all_generic_routes() noexcept
{
  const auto state = motion_workflow_routing_state();
  if (state == MotionWorkflowRoutingGuardState::kGeneric) {
    last_reported_routing_state_ = state;
    return false;
  }
  report_guarded_route("all-generic", state);
  if (guarded_generic_timeout_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_timeout_count_;
  }
  if (guarded_generic_advance_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_advance_count_;
  }
  if (guarded_generic_cleanup_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_cleanup_count_;
  }
  return true;
}

bool RestockCoordinatorDriver::guard_generic_timeout() noexcept
{
  const auto state = motion_workflow_routing_state();
  if (state == MotionWorkflowRoutingGuardState::kGeneric) {
    last_reported_routing_state_ = state;
    return false;
  }
  report_guarded_route("timeout", state);
  if (guarded_generic_timeout_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_timeout_count_;
  }
  return true;
}

bool RestockCoordinatorDriver::guard_generic_advance() noexcept
{
  const auto state = motion_workflow_routing_state();
  if (state == MotionWorkflowRoutingGuardState::kGeneric) {
    last_reported_routing_state_ = state;
    return false;
  }
  report_guarded_route("advance", state);
  if (guarded_generic_advance_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_advance_count_;
  }
  return true;
}

bool RestockCoordinatorDriver::guard_generic_cleanup() noexcept
{
  const auto state = motion_workflow_routing_state();
  if (state == MotionWorkflowRoutingGuardState::kGeneric) {
    last_reported_routing_state_ = state;
    return false;
  }
  report_guarded_route("cleanup", state);
  if (guarded_generic_cleanup_count_ != std::numeric_limits<std::size_t>::max()) {
    ++guarded_generic_cleanup_count_;
  }
  return true;
}

void RestockCoordinatorDriver::process(CoordinatorInboxDelivery delivery, SteadyTime now)
{
  std::visit(
    [this, now](auto && item) {
      using T = std::decay_t<decltype(item)>;
      if (pre_context_overflow_ && !std::is_same_v<T, CoordinatorAcceptedGoal> &&
      !std::is_same_v<T, CoordinatorOverflowMarker> &&
      !std::is_same_v<T, CoordinatorCleanupEvidenceLossMarker>)
      {
        // Overflow arrives ahead of the accepted-goal item. Other deliveries cannot be
        // interpreted without the goal context and must not emit generation-zero faults.
        return;
      }
      if constexpr (std::is_same_v<T, CoordinatorOverflowMarker>) {
        const auto admission = admission_.snapshot();
        const auto overflow_generation = context_ ? context_->generation() : admission.generation;
        if (!admissible_goal_generation(item.goal_generation) || item.detail.empty() ||
        item.goal_generation != overflow_generation)
        {
          if (context_) {
            inhibit("coordinator inbox overflow marker is malformed or stale", now);
          } else {
            admission_.inhibit("coordinator inbox overflow marker is malformed or stale");
          }
          return;
        }
        const auto overflow_goal_id = context_ ?
        std::optional<CoordinatorGoalId>(context_->goal_id()) : admission.goal_id;
        if (!overflow_goal_id) {
          if (context_) {
            inhibit("coordinator inbox overflow marker has no bound goal", now);
          } else {
            admission_.inhibit("coordinator inbox overflow marker has no bound goal");
          }
          return;
        }
        auto loss = termination_router_.record_inbox_loss(
          *overflow_goal_id, overflow_generation, item.arrived_at);
        auto first_termination = loss.record();
        if (!termination_latch_succeeded(loss) || !first_termination ||
        validate_goal_termination_record(*first_termination) !=
        GoalTerminationValidationError::kNone ||
        first_termination->goal_id != *overflow_goal_id ||
        first_termination->goal_generation != overflow_generation)
        {
          if (context_) {
            inhibit("coordinator inbox loss could not be recorded authoritatively", now);
          } else {
            admission_.inhibit("coordinator inbox loss could not be recorded authoritatively");
          }
          return;
        }
        if (!context_) {
          if (pre_context_overflow_) {
            const auto & retained = *pre_context_overflow_;
            if (!retained.first_termination ||
            *retained.first_termination != *first_termination)
            {
              admission_.inhibit(
                "repeated coordinator overflow disagrees with retained termination evidence");
            }
            return;
          }
          pre_context_overflow_.emplace(
            RetainedPreContextOverflow{
            std::move(first_termination), item.arrived_at, std::string(item.detail)});
          admission_.inhibit(
            pre_context_overflow_->detail.empty() ?
            "coordinator inbox overflowed before accepted-goal handoff" :
            pre_context_overflow_->detail);
          return;
        }
        begin_deferred_inhibition_cleanup(
          std::string(item.detail), item.arrived_at, now);
      } else if constexpr (std::is_same_v<T, CoordinatorCleanupEvidenceLossMarker>) {
        const auto admission = admission_.snapshot();
        const auto expected_generation = context_ ? context_->generation() : admission.generation;
        const bool valid_kind =
        item.kind == CoordinatorCleanupEvidenceKind::kSnapshot ||
        item.kind == CoordinatorCleanupEvidenceKind::kReleaseReservation;
        const bool valid_reconciliation = !item.reconciliation ||
        ((item.reconciliation->kind == ReconciliationKind::kReplayMutation ||
        item.reconciliation->kind == ReconciliationKind::kReadback) &&
        item.reconciliation->attempt_number != 0U);
        if (!valid_kind || !admissible_goal_generation(item.correlation.goal_generation) ||
        item.correlation.operation_generation == 0U ||
        item.correlation.operation_generation ==
        std::numeric_limits<OperationGeneration>::max() ||
        !valid_reconciliation || item.correlation.goal_generation != expected_generation)
        {
          if (context_) {
            inhibit("cleanup evidence-loss marker is malformed or stale", now);
          } else {
            admission_.inhibit("cleanup evidence-loss marker is malformed or stale");
          }
          return;
        }
        if (context_) {
          begin_deferred_inhibition_cleanup(
            "coordinator cleanup callback evidence was lost", item.arrived_at, now);
        } else {
          admission_.inhibit("coordinator cleanup callback evidence was lost");
        }
      } else if constexpr (std::is_same_v<T, CoordinatorAcceptedGoal>) {
        accept_goal(std::move(item), now);
      } else if constexpr (std::is_same_v<T, CoordinatorControlEvent>) {
        process_control(item, now);
      } else {
        process_completion(item, now);
      }
    },
    std::move(delivery));
}

void RestockCoordinatorDriver::accept_goal(
  CoordinatorAcceptedGoal accepted, SteadyTime now)
{
  if (context_) {
    inhibit("accepted-goal ingress overlapped an active context", now);
    return;
  }
  const auto admission = admission_.snapshot();
  const bool retained_overflow_handoff = [&]() noexcept {
    if (!admission.inhibited || !pre_context_overflow_ || !admission.first_termination) {
      return false;
    }
    const auto & retained = *pre_context_overflow_;
    const auto & first = *admission.first_termination;
    return validate_goal_termination_record(first) == GoalTerminationValidationError::kNone &&
           first.goal_id == accepted.goal_id &&
           first.goal_generation == accepted.goal_generation &&
           retained.first_termination &&
           *retained.first_termination == first &&
           retained.first_termination.get() == admission.first_termination.get() &&
           retained.arrived_at != SteadyTime::max() &&
           valid_accepted_termination_state(admission, accepted, true);
  }();
  const bool valid_normal_handoff = valid_accepted_termination_state(admission, accepted);
  if (admission.phase != GoalSlotPhase::kActive || !admission.goal_id ||
    *admission.goal_id != accepted.goal_id ||
    admission.generation != accepted.goal_generation ||
    (admission.inhibited && !retained_overflow_handoff) ||
    (!valid_normal_handoff && !retained_overflow_handoff))
  {
    admission_.inhibit("accepted-goal ingress does not match the active admission capability");
    return;
  }
  if (!accepted.generation_quiescence) {
    admission_.inhibit("accepted-goal ingress is missing generation quiescence authority");
    return;
  }
  try {
    const auto gate = accepted.generation_quiescence->snapshot();
    if (!valid_pristine_generation_gate(gate, accepted, admission)) {
      admission_.inhibit(
        "accepted-goal ingress generation quiescence authority is mismatched or non-pristine");
      return;
    }
  } catch (...) {
    (void)accepted.generation_quiescence->mark_synchronization_failure();
    admission_.inhibit("accepted-goal ingress generation quiescence snapshot failed");
    return;
  }
  const bool sealed_drain_handoff = accepted.sealed_pending_control_handoff &&
    !accepted.drain_requested_at && admission.first_termination &&
    admission.first_termination->kind == GoalTerminationKind::kCoordinatorDrain &&
    !retained_overflow_handoff;
  commanded_ = {};
  try {
    context_ = std::make_unique<RestockGoalContext>(
      RestockGoalContextInit{
        accepted.goal_id, accepted.goal_generation, std::move(accepted.selection_request),
        accepted.steady_started, accepted.simulation_started, config_.task,
        std::move(accepted.generation_quiescence)});
  } catch (const std::invalid_argument & error) {
    const std::string detail = std::string("accepted goal context is invalid: ") + error.what();
    if (retained_overflow_handoff) {
      admission_.inhibit(detail);
    } else {
      inhibit(detail, now);
    }
    return;
  }
  inactive_after_terminal_ack_generation_.reset();
  task_deadline_.reset();
  deadline_observed_after_first_termination_ = false;
  deadline_retreat_until_.reset();
  if (!sealed_drain_handoff) {
    task_deadline_ = bounded_deadline(accepted.steady_started, config_.task.total_timeout);
  }
  terminal_output_emitted_ = false;
  terminal_delivery_acknowledged_ = false;
  release_submission_rejections_ = 0U;
  release_submission_exhausted_ = false;
  primary_outcome_.reset();
  primary_detail_.reset();
  inhibited_ = retained_overflow_handoff;
  executed_segment_.reset();
  perception_reacquire_.reset();
  selection_reacquire_.reset();
  recovery_stop_settle_.reset();
  transaction_rollback_recovery_evidence_.reset();
  next_execution_generation_ = 0U;
  const auto initial_transition = context_->begin_task();
  if (retained_overflow_handoff) {
    observe_transition(initial_transition, now);
    auto retained = std::move(*pre_context_overflow_);
    pre_context_overflow_.reset();
    primary_outcome_ = RestockActionOutcome::kExternalInconsistency;
    primary_detail_ = retained.detail.empty() ?
      "coordinator inbox overflowed before accepted-goal handoff" : retained.detail;
    outputs_.push_back(
      CoordinatorDriverOutput{
        CoordinatorDriverOutputKind::kInhibited, context_->generation(),
        context_->task_status(), context_->selection(),
        RestockActionOutcome::kExternalInconsistency, metrics(now), *primary_detail_});
    dispatch(RestockTaskEvent::kTerminalFailure, *primary_detail_, retained.arrived_at);
    return;
  }
  if (sealed_drain_handoff) {
    pending_accepted_drain_fence_.emplace(
      PendingAcceptedDrainFence{
        accepted.goal_generation, admission.first_termination->arrived_at,
        admission.first_termination});
    return;
  }
  observe_transition(initial_transition, now);
  if (accepted.drain_requested_at) {
    primary_outcome_ = RestockActionOutcome::kShutdown;
    dispatch(
      RestockTaskEvent::kDrainRequested,
      "coordinator drain was latched before accepted-goal delivery",
      *accepted.drain_requested_at);
    abort_read_only_for_termination(*accepted.drain_requested_at, now);
  }
}

void RestockCoordinatorDriver::process_control(
  const CoordinatorControlEvent & event, SteadyTime now)
{
  if (!context_ || event.goal_generation != context_->generation()) {
    return;
  }
  switch (event.kind) {
    case CoordinatorControlKind::kCancelRequested:
      if (!primary_outcome_) {
        primary_outcome_ = RestockActionOutcome::kCanceled;
        primary_detail_ = event.detail.empty() ? "client cancellation requested" : event.detail;
      }
      dispatch(RestockTaskEvent::kCancelRequested, event.detail, now);
      abort_read_only_for_termination(event.arrived_at, now);
      break;
    case CoordinatorControlKind::kDrainRequested:
      if (primary_outcome_ != RestockActionOutcome::kExternalInconsistency) {
        primary_outcome_ = RestockActionOutcome::kShutdown;
        primary_detail_ = event.detail.empty() ? "coordinator drain requested" : event.detail;
      }
      dispatch(RestockTaskEvent::kDrainRequested, event.detail, now);
      abort_read_only_for_termination(event.arrived_at, now);
      break;
    case CoordinatorControlKind::kSafeAbortRequested:
      dispatch(RestockTaskEvent::kSafeAbortRequested, event.detail, now);
      abort_read_only_for_termination(event.arrived_at, now);
      break;
    case CoordinatorControlKind::kAuthorityFaultSafeAbortRequested:
      primary_outcome_ = RestockActionOutcome::kExternalInconsistency;
      primary_detail_ = event.detail.empty() ?
        "runtime authority transform became invalid" : event.detail;
      deferred_inhibition_detail_ = *primary_detail_;
      dispatch(RestockTaskEvent::kSafeAbortRequested, *primary_detail_, now);
      abort_read_only_for_termination(event.arrived_at, now);
      break;
    case CoordinatorControlKind::kStateDeadline:
      if (const auto ticket = ledger_.pending();
        ticket && event.operation_generation == ticket->operation_generation)
      {
        check_deadlines(event.arrived_at);
      }
      break;
    case CoordinatorControlKind::kTaskDeadline:
      check_deadlines(event.arrived_at);
      break;
    case CoordinatorControlKind::kHeartbeat:
      break;
    case CoordinatorControlKind::kShutdown:
      begin_deferred_inhibition_cleanup(
        event.detail.empty() ? "coordinator shutdown interrupted an active goal" : event.detail,
        event.arrived_at, now);
      break;
  }
}

template<typename Completion>
OperationCompletionDecision RestockCoordinatorDriver::classify_completion(
  const Completion & completion)
{
  if (completion.has_response()) {
    return ledger_.complete(
      completion.correlation.goal_generation,
      completion.correlation.operation_generation, completion.arrived_at);
  }
  return ledger_.report_unknown_outcome(
    completion.correlation.goal_generation,
    completion.correlation.operation_generation, completion.arrived_at);
}

std::optional<SteadyTime>
RestockCoordinatorDriver::authorize_termination_cleanup_evidence_time(
  const OperationCorrelation & correlation, SteadyTime arrived_at) const
{
  if (arrived_at != SteadyTime::max()) {
    return arrived_at;
  }
  const auto ticket = ledger_.pending();
  if (!context_ || !ticket || context_->generation() != correlation.goal_generation ||
    ticket->goal_generation != correlation.goal_generation ||
    ticket->operation_generation != correlation.operation_generation)
  {
    return std::nullopt;
  }
  const auto admission = admission_.snapshot();
  if (admission.phase != GoalSlotPhase::kActive || !admission.goal_id ||
    *admission.goal_id != context_->goal_id() ||
    admission.generation != correlation.goal_generation ||
    !admission.first_termination ||
    validate_goal_termination_record(*admission.first_termination) !=
    GoalTerminationValidationError::kNone)
  {
    return std::nullopt;
  }
  // The callback withheld its timestamp behind the invalid-time sentinel; the termination
  // record carries the same clock sample.
  return admission.first_termination->arrived_at;
}

void RestockCoordinatorDriver::process_completion(
  const SnapshotCompletion & completion, SteadyTime now)
{
  const auto ticket = ledger_.pending();
  if (completion.arrived_at == SteadyTime::max() &&
    (!ticket || ticket->command != RestockTaskCommand::kReleaseTaskReservation))
  {
    return;
  }
  const auto arrived_at = authorize_termination_cleanup_evidence_time(
    completion.correlation, completion.arrived_at);
  if (!arrived_at) {
    return;
  }
  if (*arrived_at != completion.arrived_at) {
    auto authorized = completion;
    authorized.arrived_at = *arrived_at;
    process_completion(authorized, now);
    return;
  }
  if (completion.reconciliation &&
    completion.reconciliation->kind != ReconciliationKind::kReadback)
  {
    inhibit("snapshot completion carries a non-readback reconciliation kind", now);
    return;
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->goal_generation == completion.correlation.goal_generation &&
    ticket->operation_generation == completion.correlation.operation_generation &&
    ticket->command != RestockTaskCommand::kValidateScene &&
    ticket->command != RestockTaskCommand::kObserveDestination &&
    ticket->command != RestockTaskCommand::kSurveyDestination &&
    ticket->command != RestockTaskCommand::kVerifyGrasp &&
    ticket->command != RestockTaskCommand::kVerifyPlacement &&
    ticket->command != RestockTaskCommand::kUpdateInventory &&
    ticket->command != RestockTaskCommand::kExecuteRecovery &&
    ticket->command != RestockTaskCommand::kReleaseTaskReservation)
  {
    inhibit("snapshot completion service does not match the pending command", now);
    return;
  }
  complete_matching_request_handle(completion.correlation, completion.reconciliation);
  if (completion.reconciliation) {
    const auto active = ledger_.active_reconciliation();
    if (!active || active->ticket.goal_generation != completion.correlation.goal_generation ||
      active->ticket.operation_generation != completion.correlation.operation_generation ||
      active->kind != completion.reconciliation->kind ||
      active->attempt_number != completion.reconciliation->attempt_number)
    {
      return;
    }
    if (completion.arrived_at >= active->deadline) {
      const auto disposition = ledger_.complete_reconciliation(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation, completion.reconciliation->kind,
        completion.reconciliation->attempt_number, completion.arrived_at);
      if (disposition == ReconciliationCompletionDisposition::kExhausted) {
        inhibit("reservation-release readback reconciliation expired", now);
      }
      return;
    }
    if (!completion.has_response() || !context_) {
      (void)ledger_.complete_reconciliation(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation, completion.reconciliation->kind,
        completion.reconciliation->attempt_number, completion.arrived_at);
      return;
    }
    auto decoded = restocker_world_state::snapshot_from_message(
      completion.response->snapshot, config_.planning_frame);
    if (!decoded) {
      (void)ledger_.complete_reconciliation(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation, completion.reconciliation->kind,
        completion.reconciliation->attempt_number, completion.arrived_at);
      inhibit("reconciliation snapshot is malformed: " + decoded.error().detail, now);
      return;
    }
    const auto retained = context_->retain_release_readback(std::move(decoded.value()));
    if (!retained) {
      (void)ledger_.complete_reconciliation(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation, completion.reconciliation->kind,
        completion.reconciliation->attempt_number, completion.arrived_at);
      inhibit("release readback violates authoritative lineage: " + retained.detail, now);
      return;
    }
    const auto disposition = ledger_.complete_reconciliation(
      completion.correlation.goal_generation,
      completion.correlation.operation_generation, completion.reconciliation->kind,
      completion.reconciliation->attempt_number, completion.arrived_at,
      retained.disposition == ReleaseReadbackDisposition::kExactReservationStillActive ?
      ReconciliationEvidence::kExactMutationStillApplied :
      ReconciliationEvidence::kInconclusive);
    if (disposition == ReconciliationCompletionDisposition::kExhausted) {
      inhibit("reservation-release readback reconciliation expired", now);
      return;
    }
    if (disposition != ReconciliationCompletionDisposition::kCompleted) {
      return;
    }
    if (retained.disposition == ReleaseReadbackDisposition::kExactReservationStillActive) {
      return;
    }
    if (!ledger_.resolve(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation) ||
      !admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReleaseTask))
    {
      inhibit("release readback proof could not resolve mutation ownership", now);
      return;
    }
    dispatch(RestockTaskEvent::kOperationSucceeded, "reservation absence proven", now);
    latch_deferred_inhibition(now);
    return;
  }

  const auto decision = classify_completion(completion);
  if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
    inhibit("snapshot completion has unknown operation correlation", now);
    return;
  }
  if (decision.disposition != OperationCompletionDisposition::kDeliver ||
    !decision.ticket)
  {
    if (decision.disposition == OperationCompletionDisposition::kDeliverTimeout) {
      fail_operation(RestockTaskEvent::kTimeout, "snapshot response missed its deadline", now);
    }
    return;
  }
  if (!completion.has_response() || !context_) {
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      completion.transport_error.empty() ? "snapshot response is absent" :
      completion.transport_error,
      now);
    return;
  }
  auto decoded = restocker_world_state::snapshot_from_message(
    completion.response->snapshot, config_.planning_frame);
  if (!decoded) {
    fail_operation(
      RestockTaskEvent::kTerminalFailure,
      "authoritative snapshot is malformed: " + decoded.error().detail, now);
    return;
  }
  // These commands read fresh evidence; verify_observed_evidence checks it after adoption.
  const auto observed_command = decision.ticket->command;
  const bool observes_fresh_evidence =
    observed_command == RestockTaskCommand::kObserveDestination ||
    observed_command == RestockTaskCommand::kSurveyDestination ||
    observed_command == RestockTaskCommand::kVerifyGrasp ||
    observed_command == RestockTaskCommand::kVerifyPlacement ||
    observed_command == RestockTaskCommand::kExecuteRecovery ||
    observed_command == RestockTaskCommand::kUpdateInventory;
  GoalContextResult retained;
  // The first validation snapshot is the immutable baseline; re-validation after a superseded
  // selection retains later snapshots as latest evidence.
  const bool revalidates_scene = observed_command == RestockTaskCommand::kValidateScene &&
    context_->initial_snapshot().has_value();
  if (revalidates_scene) {
    retained = context_->retain_latest_snapshot(std::move(decoded.value()));
  } else if (observed_command == RestockTaskCommand::kValidateScene) {
    retained = context_->retain_initial_snapshot(std::move(decoded.value()));
  } else if (observed_command == RestockTaskCommand::kReleaseTaskReservation) {
    retained = context_->retain_released_snapshot(std::move(decoded.value()));
  } else if (observes_fresh_evidence) {
    retained = context_->retain_latest_snapshot(std::move(decoded.value()));
    if (retained) {
      if (observed_command == RestockTaskCommand::kExecuteRecovery &&
        !settle_recovery_stop(now))
      {
        return;
      }
      if (auto verdict = verify_observed_evidence(observed_command); !verdict.empty()) {
        fail_operation(RestockTaskEvent::kRetryableFailure, std::move(verdict), now);
        return;
      }
      // Grasp verification is the last state with the arm at the planned grasp pose, so the
      // product-to-grasp coupling is retained here for later insertion planning. A bounded
      // recovery re-entry reaches this state again inside the same goal, and each accepted
      // verification re-derives the coupling rather than refusing it (Milestone 10 §1, Card 039),
      // so a repeated verification is a success the driver advances from, never a failure.
      if (observed_command == RestockTaskCommand::kVerifyGrasp) {
        const auto previous = context_->grasp_coupling();
        if (const auto coupling = context_->retain_grasp_coupling(); !coupling) {
          fail_operation(
            RestockTaskEvent::kRetryableFailure,
            "grasp coupling could not be retained: " + coupling.detail, now);
          return;
        }
        // Receipt: which verification of this goal owns the coupling, under which recovery
        // attempt, and whether a re-entry re-derived it — the interleaving a Terminal-A
        // recurrence needs and today's logs cannot reconstruct.
        const auto & retained = *context_->grasp_coupling();
        const auto status = context_->task_status();
        std::string receipt =
          "grasp coupling retained at verify_grasp: verify #" +
          std::to_string(context_->grasp_coupling_ordinal()) + " of this goal, recovery attempt " +
          std::to_string(status.recovery_attempt) + ", world revision " +
          std::to_string(retained.source_world_revision) + ", object revision " +
          std::to_string(retained.source_object_revision);
        if (!previous) {
          receipt += " (first capture)";
        } else {
          const bool unchanged =
            previous->source_world_revision == retained.source_world_revision &&
            previous->source_object_revision == retained.source_object_revision &&
            previous->product_from_grasp_center.matrix().isApprox(
            retained.product_from_grasp_center.matrix(), 1.0e-12);
          receipt +=
            unchanged ? " (re-verified unchanged)" : " (re-derived; previous world revision " +
            std::to_string(previous->source_world_revision) + ")";
        }
        report_task_receipt(receipt, now);
      }
    }
  } else {
    inhibit("snapshot completion does not match the active command", now);
    return;
  }
  if (!retained) {
    inhibit("snapshot evidence violates goal lineage: " + retained.detail, now);
    return;
  }
  dispatch(RestockTaskEvent::kOperationSucceeded, "authoritative snapshot retained", now);
  if (decision.ticket->command == RestockTaskCommand::kReleaseTaskReservation) {
    latch_deferred_inhibition(now);
  }
}

void RestockCoordinatorDriver::process_completion(
  const ReserveTaskCompletion & completion, SteadyTime now)
{
  const auto arrived_at = authorize_termination_cleanup_evidence_time(
    completion.correlation, completion.arrived_at);
  if (!arrived_at) {
    return;
  }
  if (*arrived_at != completion.arrived_at) {
    auto authorized = completion;
    authorized.arrived_at = *arrived_at;
    process_completion(authorized, now);
    return;
  }
  if (completion.reconciliation &&
    completion.reconciliation->kind != ReconciliationKind::kReplayMutation)
  {
    inhibit("reserve completion carries a non-replay reconciliation kind", now);
    return;
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->goal_generation == completion.correlation.goal_generation &&
    ticket->operation_generation == completion.correlation.operation_generation &&
    ticket->command != RestockTaskCommand::kReserveTask)
  {
    inhibit("reserve completion service does not match the pending command", now);
    return;
  }
  complete_matching_request_handle(completion.correlation, completion.reconciliation);
  if (!context_) {
    return;
  }
  OperationCompletionDisposition completion_disposition =
    OperationCompletionDisposition::kRejectUnknown;
  if (completion.reconciliation) {
    const auto disposition = ledger_.complete_reconciliation(
      completion.correlation.goal_generation,
      completion.correlation.operation_generation, completion.reconciliation->kind,
      completion.reconciliation->attempt_number, completion.arrived_at,
      completion.has_response() ? ReconciliationEvidence::kMutationResponseRetained :
      ReconciliationEvidence::kInconclusive);
    if (disposition == ReconciliationCompletionDisposition::kExhausted) {
      inhibit("reservation replay reconciliation expired", now);
      return;
    }
    if (disposition != ReconciliationCompletionDisposition::kCompleted ||
      !completion.has_response())
    {
      return;
    }
    completion_disposition = OperationCompletionDisposition::kReconcile;
  } else {
    const auto superseded_attempt = ledger_.active_reconciliation();
    const auto decision = classify_completion(completion);
    completion_disposition = decision.disposition;
    if (decision.disposition == OperationCompletionDisposition::kReconcile &&
      completion.has_response() && superseded_attempt)
    {
      remove_reconciliation_request(*superseded_attempt);
    }
    if (decision.disposition == OperationCompletionDisposition::kReconciliationExhausted) {
      inhibit("reservation response arrived after reconciliation expiry", now);
      return;
    }
    if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
      inhibit("reservation completion has unknown operation correlation", now);
      return;
    }
    if ((decision.disposition != OperationCompletionDisposition::kDeliver &&
      decision.disposition != OperationCompletionDisposition::kReconcile) ||
      !completion.has_response())
    {
      return;
    }
  }
  const auto retained = context_->retain_reserve_response(*completion.response);
  if (!retained) {
    inhibit("reservation response violates exact lineage: " + retained.detail, now);
    return;
  }
  const bool selection_superseded =
    retained.disposition == ReserveResponseDisposition::kSelectionSuperseded;
  if (inhibited_ &&
    completion_disposition == OperationCompletionDisposition::kReconcile)
  {
    if (!ledger_.resolve(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation) ||
      !admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReserveTask))
    {
      inhibit("inhibited reservation response could not resolve mutation ownership", now);
      return;
    }
    remove_operation_requests(completion.correlation);
    continue_deferred_inhibition_cleanup(completion.arrived_at, now);
    return;
  }
  if (!ledger_.phase()) {
    if (!admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReserveTask))
    {
      inhibit("reservation response could not resolve mutation submission ownership", now);
    }
  }
  continue_deferred_inhibition_cleanup(completion.arrived_at, now);
  // The task returns for fresh evidence only after the refused mutation released its submission
  // ownership; the re-derived reservation commits a new submission.
  if (selection_superseded && !inhibited_ && context_) {
    fail_operation(RestockTaskEvent::kSelectionSuperseded, retained.detail, now);
  }
}

void RestockCoordinatorDriver::process_completion(
  const ValidateReservationCompletion & completion, SteadyTime now)
{
  if (completion.arrived_at == SteadyTime::max()) {
    return;
  }
  if (completion.reconciliation &&
    completion.reconciliation->kind != ReconciliationKind::kReadback)
  {
    inhibit("validation completion carries a non-readback reconciliation kind", now);
    return;
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->goal_generation == completion.correlation.goal_generation &&
    ticket->operation_generation == completion.correlation.operation_generation &&
    ticket->command != RestockTaskCommand::kReserveTask)
  {
    inhibit("validation completion service does not match the pending command", now);
    return;
  }
  complete_matching_request_handle(completion.correlation, completion.reconciliation);
  if (completion.reconciliation) {
    const auto disposition = ledger_.complete_reconciliation(
      completion.correlation.goal_generation,
      completion.correlation.operation_generation, completion.reconciliation->kind,
      completion.reconciliation->attempt_number, completion.arrived_at);
    if (disposition == ReconciliationCompletionDisposition::kExhausted) {
      inhibit("reservation validation reconciliation expired", now);
      return;
    }
    if (disposition != ReconciliationCompletionDisposition::kCompleted ||
      !completion.has_response() || !context_)
    {
      return;
    }
    const auto retained = context_->retain_reservation_validation_response(
      *completion.response);
    if (!retained) {
      inhibit("reservation readback violates exact lineage: " + retained.detail, now);
      return;
    }
    if (!ledger_.resolve(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation) ||
      !admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReserveTask))
    {
      inhibit("reservation readback could not resolve mutation ownership", now);
      return;
    }
    if (deferred_inhibition_detail_) {
      continue_deferred_inhibition_cleanup(completion.arrived_at, now);
      return;
    }
    dispatch(RestockTaskEvent::kOperationSucceeded, "reservation capability validated", now);
    return;
  }
  const auto decision = classify_completion(completion);
  if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
    inhibit("reservation-validation completion has unknown correlation", now);
    return;
  }
  if (decision.disposition != OperationCompletionDisposition::kDeliver ||
    !decision.ticket)
  {
    if (decision.disposition == OperationCompletionDisposition::kDeliverTimeout) {
      fail_operation(
        RestockTaskEvent::kTimeout, "reservation validation missed its deadline", now);
    }
    return;
  }
  if (!completion.has_response() || !context_) {
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      completion.transport_error.empty() ? "reservation validation response is absent" :
      completion.transport_error,
      now);
    return;
  }
  const auto retained = context_->retain_reservation_validation_response(*completion.response);
  if (!retained) {
    inhibit("reservation validation violates exact lineage: " + retained.detail, now);
    return;
  }
  dispatch(RestockTaskEvent::kOperationSucceeded, "reservation capability validated", now);
}

void RestockCoordinatorDriver::process_completion(
  const ReleaseReservationCompletion & completion, SteadyTime now)
{
  const auto arrived_at = authorize_termination_cleanup_evidence_time(
    completion.correlation, completion.arrived_at);
  if (!arrived_at) {
    return;
  }
  if (*arrived_at != completion.arrived_at) {
    auto authorized = completion;
    authorized.arrived_at = *arrived_at;
    process_completion(authorized, now);
    return;
  }
  if (completion.reconciliation &&
    completion.reconciliation->kind != ReconciliationKind::kReplayMutation)
  {
    inhibit("release completion carries a non-replay reconciliation kind", now);
    return;
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->goal_generation == completion.correlation.goal_generation &&
    ticket->operation_generation == completion.correlation.operation_generation &&
    ticket->command != RestockTaskCommand::kReleaseTaskReservation)
  {
    inhibit("release completion service does not match the pending command", now);
    return;
  }
  complete_matching_request_handle(completion.correlation, completion.reconciliation);
  if (!context_) {
    return;
  }
  OperationCompletionDisposition completion_disposition =
    OperationCompletionDisposition::kRejectUnknown;
  if (completion.reconciliation) {
    const auto disposition = ledger_.complete_reconciliation(
      completion.correlation.goal_generation,
      completion.correlation.operation_generation, completion.reconciliation->kind,
      completion.reconciliation->attempt_number, completion.arrived_at,
      completion.has_response() ? ReconciliationEvidence::kMutationResponseRetained :
      ReconciliationEvidence::kInconclusive);
    if (disposition == ReconciliationCompletionDisposition::kExhausted) {
      inhibit("reservation-release replay reconciliation expired", now);
      return;
    }
    if (disposition != ReconciliationCompletionDisposition::kCompleted ||
      !completion.has_response())
    {
      return;
    }
    completion_disposition = OperationCompletionDisposition::kReconcile;
  } else {
    const auto superseded_attempt = ledger_.active_reconciliation();
    const auto decision = classify_completion(completion);
    completion_disposition = decision.disposition;
    if (decision.disposition == OperationCompletionDisposition::kReconcile &&
      completion.has_response() && superseded_attempt)
    {
      remove_reconciliation_request(*superseded_attempt);
    }
    if (decision.disposition == OperationCompletionDisposition::kReconciliationExhausted) {
      inhibit("release response arrived after reconciliation expiry", now);
      return;
    }
    if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
      inhibit("release completion has unknown operation correlation", now);
      return;
    }
    if ((decision.disposition != OperationCompletionDisposition::kDeliver &&
      decision.disposition != OperationCompletionDisposition::kReconcile) ||
      !completion.has_response())
    {
      return;
    }
  }
  const auto retained = context_->retain_release_response(*completion.response);
  if (!retained) {
    inhibit("release response violates exact lineage: " + retained.detail, now);
    return;
  }
  if (inhibited_ &&
    completion_disposition == OperationCompletionDisposition::kReconcile)
  {
    if (!ledger_.resolve(
        completion.correlation.goal_generation,
        completion.correlation.operation_generation) ||
      !admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReleaseTask))
    {
      inhibit("inhibited release response could not resolve mutation ownership", now);
      return;
    }
    remove_operation_requests(completion.correlation);
    return;
  }
  if (!ledger_.phase()) {
    if (!admission_.resolve_mutation_submission(
        context_->goal_id(), context_->generation(),
        completion.correlation.operation_generation, CoordinatorMutationKind::kReleaseTask))
    {
      inhibit("release response could not resolve mutation submission ownership", now);
    }
  }
}

void RestockCoordinatorDriver::check_deadlines(SteadyTime now)
{
  if (guard_generic_timeout()) {
    return;
  }
  if (!context_ || terminal_output_emitted_) {
    return;
  }
  if (task_deadline_ && now >= *task_deadline_ &&
    !admission_.snapshot().task_deadline_exceeded &&
    !deadline_observed_after_first_termination_)
  {
    // Card 044: the expiry is receipted here, before any downstream refusal can follow — one
    // line, both clocks, the task state and the recovery counters (Milestone 10 §6). Without it
    // the family's only visible symptom is the retreat identity refusal seconds later.
    const auto expiring = context_->task_status();
    const auto sim_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::nanoseconds((ros_now_() - context_->simulation_started()).nanoseconds()));
    std::ostringstream expiry;
    expiry << "whole-task steady deadline exceeded: " <<
      config_.task.total_timeout.count() << " ms budget spent at steady +" <<
      std::chrono::duration_cast<std::chrono::milliseconds>(
      now - context_->steady_started()).count() <<
      " ms since goal acceptance, sim +" << sim_elapsed.count() <<
      " ms, state=" << to_string(expiring.state) <<
      ", recovery attempt " << expiring.recovery_attempt << " of " <<
      config_.task.max_recovery_attempts;
    report_task_receipt(expiry.str(), now);
    // Card 051 review blocker: if any termination already won first (client cancellation,
    // drain, safe abort — one generic check over termination_intent), the deadline is
    // receipt-only: no policy is applied (so the latched intent is never upgraded), no
    // deadline event is dispatched (so the machine never gains the cleanup-retreat authority
    // on top of that termination), and no retreat budget opens. One-shot: without the
    // admission flag being set, later pumps would reprint the receipt.
    const auto admission = admission_.snapshot();
    if (admission.termination_intent != GoalTerminationIntent::kNone) {
      const char * winner = "an earlier termination";
      switch (admission.termination_intent) {
        case GoalTerminationIntent::kUserCancel: winner = "client cancellation"; break;
        case GoalTerminationIntent::kShutdownDrain: winner = "shutdown/drain"; break;
        case GoalTerminationIntent::kSafeAbort: winner = "safe abort"; break;
        case GoalTerminationIntent::kTaskDeadline: winner = "whole-task deadline"; break;
      }
      deadline_observed_after_first_termination_ = true;
      report_task_receipt(
        std::string("whole-task deadline observed after ") + winner +
        " already won the first termination; receipt-only — no deadline termination "
        "dispatched, no cleanup retreat armed",
        now);
      abort_read_only_for_termination(now, now);
      return;
    }
    // The deadline is classified like any other terminal (Milestone 10 §6, Card 051): at the
    // instant of expiry the stop may not be established yet, and the receipt says so honestly;
    // the verdict delivered after the cleanup re-classifies on the same evidence.
    report_task_receipt(
      recovery_classification_receipt(
        classify_current("whole-task steady deadline exceeded"),
        "whole-task steady deadline exceeded"),
      now);
    // The expiry opens the one budget that may still move the arm: the bounded cleanup retreat
    // of §6 (Card 051). Measured from this instant, separate from the expired whole-task budget.
    deadline_retreat_until_ =
      bounded_deadline(now, config_.task.deadline_retreat_timeout);
    if (!primary_outcome_ ||
      (*primary_outcome_ != RestockActionOutcome::kShutdown &&
      *primary_outcome_ != RestockActionOutcome::kExternalInconsistency &&
      *primary_outcome_ != RestockActionOutcome::kCanceled))
    {
      primary_outcome_ = RestockActionOutcome::kPlanningFailed;
      primary_detail_ = "whole-task steady deadline exceeded";
    }
    const auto pending = ledger_.pending();
    auto route = termination_router_.route_task_deadline(
      context_->goal_id(), context_->generation(), context_->generation(), now,
      pending ? pending->operation_generation : 0U,
      "whole-task steady deadline exceeded");
    auto routed_event = route.take_event();
    if (!termination_latch_succeeded(route.admission_decision()) || !routed_event) {
      inhibit("task deadline could not latch exact termination evidence", now);
      return;
    }
    dispatch(
      RestockTaskEvent::kTaskDeadlineExceeded, std::move(routed_event->detail),
      routed_event->arrived_at);
    abort_read_only_for_termination(routed_event->arrived_at, now);
  }
  const auto pending = ledger_.pending();
  // Milestone 10 §6 (Card 051): the deadline's bounded cleanup retreat must end in a verified
  // stop inside its own budget — a retreat still in flight when the budget expires is
  // cancelled and the goal fails closed for the operator. The not-yet-submitted and
  // not-yet-executed sides of the same rule live at the guards themselves.
  if (deadline_retreat_until_ && now >= *deadline_retreat_until_ && pending &&
    planned_motion_segment(pending->command) == MotionSegment::kRetreat)
  {
    if (motion_) {
      motion_->cancel();
    }
    inhibit("deadline retreat budget expired while the cleanup retreat was in flight", now);
    return;
  }
  if (motion_ && pending && planned_motion_segment(pending->command) &&
    now >= pending->deadline)
  {
    // Segment overran its deadline: stop the port and fail closed, since without a stop proof
    // the arm position is unknown and a retry would be blind.
    motion_->cancel();
    const auto stalled = planned_motion_segment(pending->command);
    inhibit(
      std::string("motion segment exceeded its command deadline: waiting on ") +
      motion_segment_name(*stalled) + " operation " +
      std::to_string(pending->operation_generation) +
      " of goal generation " + std::to_string(pending->goal_generation), now);
    return;
  }
  const auto active_attempt = ledger_.active_reconciliation();
  const auto timeout = ledger_.check_timeout(now);
  switch (timeout) {
    case OperationTimeoutDisposition::kNotDue:
      break;
    case OperationTimeoutDisposition::kDeliverTimeout:
      if (pending) {
        remove_operation_requests(
          {pending->goal_generation, pending->operation_generation});
      }
      fail_operation(RestockTaskEvent::kTimeout, "world-state request timed out", now);
      break;
    case OperationTimeoutDisposition::kReconcile:
      break;
    case OperationTimeoutDisposition::kReconciliationAttemptTimedOut:
      if (active_attempt) {
        remove_reconciliation_request(*active_attempt);
      }
      break;
    case OperationTimeoutDisposition::kReconciliationExhausted:
      if (pending) {
        remove_operation_requests(
          {pending->goal_generation, pending->operation_generation});
      }
      inhibit("bounded mutation reconciliation exhausted", now);
      break;
    case OperationTimeoutDisposition::kCancelAndVerify:
      inhibit("driver received an unsupported motion timeout", now);
      break;
    case OperationTimeoutDisposition::kStopProofExpired:
    case OperationTimeoutDisposition::kMotionLivenessUnknown:
      inhibit("motion liveness could not be proven by its immutable deadline", now);
      break;
  }
}

void RestockCoordinatorDriver::abort_read_only_for_termination(
  SteadyTime evidence_time, SteadyTime now)
{
  const auto ticket = ledger_.pending();
  if (!ticket || ticket->effect != OperationEffect::kReadOnly ||
    (ticket->command != RestockTaskCommand::kValidateScene &&
    ticket->command != RestockTaskCommand::kReserveTask))
  {
    return;
  }
  remove_operation_requests({ticket->goal_generation, ticket->operation_generation});
  const auto completion = ledger_.complete(
    ticket->goal_generation, ticket->operation_generation, evidence_time);
  if (completion.disposition != OperationCompletionDisposition::kDeliver &&
    completion.disposition != OperationCompletionDisposition::kDeliverTimeout)
  {
    inhibit("termination could not retire pending read-only work", now);
    return;
  }
  fail_operation(
    RestockTaskEvent::kTerminalFailure,
    "pending read-only request retired at the termination boundary", now);
}

bool RestockCoordinatorDriver::advance_planned_segment(
  const RestockTaskTransition & transition, SteadyTime now)
{
  const auto segment = *planned_motion_segment(transition.command);
  // Milestone 10 §6 (Card 051): exactly the terminations recognisable by the guards may command
  // this segment instead of stopping in place — the whole-task deadline's bounded cleanup
  // retreat (budget measured from the expiry) and a requested recoverable skip (rung 5). Every
  // other cancel or safe abort short-circuits exactly as before; the guards recognise the exit,
  // they are never silenced, and each admission is receipted.
  // Card 051 review blocker: a user cancel / drain / safe abort that arrived first (or later)
  // revokes the retreat authority — the stop-in-place short-circuit below then owns the
  // segment, so an operator's stop can never yield to the deadline's cleanup.
  const bool deadline_cleanup_retreat =
    segment == MotionSegment::kRetreat && transition.task_deadline_exceeded &&
    !transition.deadline_retreat_revoked;
  const bool skip_cleanup_retreat =
    segment == MotionSegment::kRetreat && transition.recoverable_skip &&
    !transition.deadline_retreat_revoked;
  if (deadline_cleanup_retreat || skip_cleanup_retreat) {
    if (deadline_cleanup_retreat && (!deadline_retreat_until_ || now >= *deadline_retreat_until_)) {
      inhibit(
        "deadline retreat budget expired before the cleanup retreat could be commanded", now);
      return false;
    }
    report_task_receipt(
      std::string("recovery rung 1 (safe retreat): the cleanup retreat is commandable under ") +
      (deadline_cleanup_retreat ? "the whole-task deadline, " :
      "the recoverable skip, ") +
      (deadline_cleanup_retreat ?
      std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          *deadline_retreat_until_ - now).count()) + " ms of the retreat budget remain" :
      "its budget the segment command deadline already bounds"),
      now);
  } else if (transition.cancel_requested || transition.safe_abort_requested) {
    // Terminating before a segment is submitted leaves the arm where it is.
    perception_reacquire_.reset();
    dispatch(
      RestockTaskEvent::kOperationSucceeded,
      std::string(motion_segment_name(segment)) +
      " planning terminated before any motion request", now);
    return true;
  }
  if (!motion_) {
    // No motion backend: hold.
    return false;
  }
  // Reachable when recovery resumes pre-grasp planning after every candidate was refused.
  if (segment == MotionSegment::kPreGrasp && context_ &&
    context_->grasp_candidate_batch() && context_->active_grasp_candidate() == nullptr)
  {
    fail_operation(
      RestockTaskEvent::kTerminalFailure,
      "no grasp candidate remains for pre-grasp planning: " +
      context_->grasp_candidate_refusal_summary(), now);
    return true;
  }
  // The liveness gate refuses a segment whose wrist stream is older than the horizon. Under
  // representative load the stream can dip past the horizon transiently while staying healthy
  // enough to resume, so the first refusal holds a bounded, observable reacquire instead of
  // ending the campaign. No ledger operation is booked until the unchanged predicate passes
  // again; expiry takes the same terminal path as an immediate failure, and request_motion_
  // segment re-checks the predicate at submission as the final fail-closed gate.
  if (perception_liveness_) {
    const bool for_this_request =
      perception_reacquire_ &&
      perception_reacquire_->goal_generation == context_->generation() &&
      perception_reacquire_->segment == segment;
    std::string liveness_detail;
    if (!perception_liveness_(liveness_detail)) {
      const auto budget = config_.perception_reacquire_timeout;
      if (budget.count() > 0) {
        if (!for_this_request) {
          perception_reacquire_ = PerceptionReacquire{
            now, bounded_deadline(now, budget), context_->generation(), segment};
          report_perception_reacquire(
            std::string("perception stream is unlive at ") + motion_segment_name(segment) +
            " request; bounded reacquire started (" + std::to_string(budget.count()) +
            " ms): " + liveness_detail, now);
        }
        if (now < perception_reacquire_->deadline) {
          return false;
        }
      }
      perception_reacquire_.reset();
      // Same stop-evidence record the in-request refusal writes, so recovery after expiry
      // refuses with the recorded "perception stream is unlive" reason.
      motion_stop_evidence_ = MotionStopEvidence{
        segment, context_->generation(), "perception stream is unlive"};
      std::string terminal_detail =
        liveness_detail.empty() ?
        "perception stream is older than the configured liveness horizon" : liveness_detail;
      if (budget.count() > 0) {
        terminal_detail +=
          "; bounded perception reacquire of " + std::to_string(budget.count()) +
          " ms did not restore it";
        report_perception_reacquire(
          std::string("perception reacquire expired at ") + motion_segment_name(segment) +
          " after " + std::to_string(budget.count()) + " ms: " + liveness_detail, now);
      }
      fail_operation(RestockTaskEvent::kTerminalFailure, std::move(terminal_detail), now);
      return true;
    }
    if (for_this_request) {
      const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - perception_reacquire_->started);
      perception_reacquire_.reset();
      report_perception_reacquire(
        std::string("perception stream reacquired after ") + std::to_string(waited.count()) +
        " ms; resuming " + motion_segment_name(segment), now);
    } else if (perception_reacquire_) {
      // A hold keyed to another goal generation or segment must never report (or bill) this
      // request's restoration; drop it silently.
      perception_reacquire_.reset();
    }
  }
  // Motion retry/fail-closed policy is enforced in process_completion and check_deadlines, not
  // by the ledger effect.
  const auto ticket = start_operation(
    transition.command, OperationEffect::kReadOnly, "", now);
  if (!ticket) {
    return false;
  }
  request_motion_segment(*ticket, segment);
  return false;
}

bool RestockCoordinatorDriver::advance_executed_segment(
  const RestockTaskTransition & transition, SteadyTime now)
{
  // The port plans and executes a segment in one step, so the trajectory has already run.
  // Record that evidence so a later cancellation knows the arm has moved.
  const auto segment = *executed_motion_segment(transition.command);
  if (executed_segment_ != segment) {
    inhibit(
      std::string("execute-") + motion_segment_name(segment) +
      " was reached without a completed motion segment", now);
    return false;
  }
  // Execution evidence needs a retained operation identity.
  std::string identity_refusal;
  const bool under_termination =
    transition.cancel_requested || transition.safe_abort_requested ||
    transition.recoverable_skip;
  if (transition.task_deadline_exceeded && segment == MotionSegment::kRetreat &&
    !transition.deadline_retreat_revoked)
  {
    // Milestone 10 §6 (Card 051): the deadline's bounded cleanup retreat may execute while
    // termination is latched — but only inside its budget; past it the stop can no longer be
    // promised, so the goal fails closed for the operator.
    if (!deadline_retreat_until_ || now >= *deadline_retreat_until_) {
      inhibit("deadline retreat budget expired before the cleanup retreat executed", now);
      return false;
    }
  }
  if (!context_->prepare_execution_operation_id(transition.command, &identity_refusal)) {
    inhibit(
      std::string("execute-") + motion_segment_name(segment) +
      " could not retain an execution operation identity: " +
      execution_identity_refusal_detail(identity_refusal, now), now);
    return false;
  }
  if (under_termination && segment == MotionSegment::kRetreat) {
    // Which guard admitted the exit, and why (Card 051 rung receipts).
    report_task_receipt(
      std::string(
        "recovery rung 1 (safe retreat): the execution identity was admitted for the cleanup "
        "retreat under ") +
      (transition.task_deadline_exceeded ? "the whole-task deadline; " :
      "the recoverable skip; ") +
      "the refusal for every other execution under termination is unchanged",
      now);
  }
  // A unique nonzero identity only; not a ledger operation, the segment has finished.
  const auto execution_generation = ++next_execution_generation_;
  const std::string detail =
    std::string(motion_segment_name(segment)) + " segment executed";
  dispatch_execution(
    RestockTaskEvent::kExecutionOperationStarted, detail, execution_generation, now);
  dispatch_execution(
    RestockTaskEvent::kTrajectoryExecutionAccepted, detail, execution_generation, now);
  executed_segment_.reset();
  dispatch(RestockTaskEvent::kOperationSucceeded, detail, now);
  return true;
}

bool RestockCoordinatorDriver::advance_motion_recovery(
  const RestockTaskTransition & transition, SteadyTime now)
{
  // Recovery replans a failed motion, refreshes a read-only operation after a successful motion
  // boundary, or retries after a fully rolled-back attachment transaction. It is offered only with
  // first-hand evidence that the arm is stopped and its held-object state is known.
  const bool settling = recovery_stop_settle_ &&
    recovery_stop_settle_->goal_generation == context_->generation() &&
    recovery_stop_settle_->recovery_attempt == transition.recovery_attempt;
  if (recovery_stop_settle_ && !settling) {
    // A hold keyed to another goal generation or recovery attempt never bills this one.
    recovery_stop_settle_.reset();
  }
  if (transition.cancel_requested || transition.safe_abort_requested) {
    recovery_stop_settle_.reset();
    forget_recovery_question();
    dispatch(
      RestockTaskEvent::kTerminalFailure,
      "recovery is refused while the task is terminating; the arm is already stopped", now);
    return true;
  }
  if (const auto refusal = refuse_motion_recovery(); !refusal.empty()) {
    recovery_stop_settle_.reset();
    forget_recovery_question();
    dispatch(RestockTaskEvent::kTerminalFailure, refusal, now);
    return true;
  }
  if (settling) {
    // Milestone 10 §6 (Card 060): the stop-settle hold re-observes at its poll period; the
    // advisory decision and the rung receipts belong to the first observation of this attempt.
    if (now < recovery_stop_settle_->next_poll) {
      return false;
    }
    const auto ticket = start_operation(
      transition.command, OperationEffect::kReadOnly, "", now);
    if (!ticket) {
      return false;
    }
    request_snapshot(*ticket);
    return false;
  }
  // The advisory recommendation can only choose to stop: every other primitive is refused by the
  // permitted set recomputed now or for being less cautious than the deterministic policy. With
  // no backend, no answer, a late answer or an invalid one, the default replan is taken.
  if (consume_recovery_advice() == restocker_reasoner::RecoveryPrimitive::kAbandonTask) {
    dispatch(
      RestockTaskEvent::kTerminalFailure,
      "recovery was declined by the advisory reasoner and the task stops instead", now);
    return true;
  }
  // The recovery snapshot observes that the robot is stopped, unfaulted and in the attachment
  // state the task expects, which the replan starts from.
  const auto ticket = start_operation(
    transition.command, OperationEffect::kReadOnly, "", now);
  if (!ticket) {
    return false;
  }
  // Milestone 10 §6 rung receipts (Card 051): the ladder's first two rungs are what the checks
  // above already established — a safe, verified stop (or a no-op when nothing moved) and a
  // fresh re-observation before rung 3's retry, which never consumes the retained evidence.
  report_task_receipt(
    std::string("recovery rung 1 (safe retreat): ") +
    (transition.first_trajectory_may_have_started ?
    "the arm is at the verified stop of the last segment; re-planning starts from it" :
    "no-op, nothing moved in this goal") +
    ", recovery attempt " + std::to_string(transition.recovery_attempt) + " of " +
    std::to_string(config_.task.max_recovery_attempts),
    now);
  report_task_receipt(
    "recovery rung 2 (re-observe): fresh world snapshot requested before the retry; the "
    "retained evidence that failed is not consumed",
    now);
  request_snapshot(*ticket);
  return false;
}

bool RestockCoordinatorDriver::advance_generate_placement(SteadyTime now)
{
  // The generator receives the retained reservation so admission is judged against the
  // destination policy captured at grant, never the lane's live intent.
  if (!placement_generator_) {
    inhibit("no placement generator is composed", now);
    return false;
  }
  const auto & snapshot = context_->latest_snapshot();
  const auto & selection = context_->selection();
  const auto & grasp = context_->grasp_candidate_batch();
  const auto & coupling = context_->grasp_coupling();
  const auto & reservation = context_->reservation();
  if (!snapshot || !selection || !grasp || !coupling || !reservation) {
    inhibit(
      "placement generation requires a fresh snapshot, retained grasp, coupling, and reservation",
      now);
    return false;
  }
  auto generated = placement_generator_(
    *snapshot, *selection, *grasp, *coupling, reservation->reservation);
  if (!generated) {
    // A blocked destination is retryable and contradicts nothing already proved.
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      "placement generation failed: " + generated.error().detail, now);
    return true;
  }
  if (const auto retained =
    context_->retain_placement_candidate(std::move(generated.value())); !retained)
  {
    inhibit("placement candidate could not be retained: " + retained.detail, now);
    return false;
  }
  dispatch(RestockTaskEvent::kOperationSucceeded, "placement candidate staged", now);
  return true;
}

bool RestockCoordinatorDriver::advance_selection(SteadyTime now)
{
  if (!context_->latest_snapshot()) {
    inhibit("selection command has no authoritative snapshot", now);
    return false;
  }
  const auto transition = context_->task_status();
  if (transition.cancel_requested || transition.safe_abort_requested) {
    // Terminating at selection books nothing and leaves the reservation untouched; the hold
    // must not delay the termination until its budget expires.
    selection_reacquire_.reset();
    dispatch(
      RestockTaskEvent::kOperationSucceeded,
      "task selection terminated before a pair was chosen", now);
    return true;
  }
  // Selection-time bounded reacquire: an unlive wrist stream at selection used to end
  // the goal immediately. The same steady-clock budget now holds kSelectPair while the
  // cheap liveness predicate — pumped at the coordinator period, never spun — waits for
  // a fresh observation; only a stream back inside the horizon re-runs the selector,
  // and expiry falls through to the selector, which re-reports the staleness and takes
  // the original terminal path. The 500 ms horizon itself is never widened, and the
  // budget never restarts within a goal.
  if (selection_reacquire_ &&
    selection_reacquire_->goal_generation == context_->generation())
  {
    if (!perception_liveness_) {
      // Without the composed predicate the stream cannot be re-checked cheaply, so
      // fall back to the original immediate refusal rather than retry selection blind.
      selection_reacquire_.reset();
    } else {
      std::string live_detail;
      if (!perception_liveness_(live_detail)) {
        if (now < selection_reacquire_->deadline) {
          return false;
        }
        // Budget spent: the selector below re-reports the staleness and terminals
        // under the same key with the expiry named.
      } else if (!selection_reacquire_->reported_live) {
        selection_reacquire_->reported_live = true;
        const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - selection_reacquire_->started);
        report_perception_reacquire(
          "perception stream reacquired after " + std::to_string(waited.count()) +
          " ms; retrying task selection", now);
      }
    }
  }
  auto selected = selector_(*context_->latest_snapshot(), context_->selection_request());
  if (guard_generic_advance()) {
    return false;
  }
  if (!selected) {
    const auto code = selected.error().code;
    const bool unlive = code == SelectionErrorCode::PerceptionUnlive;
    // The stale family is evidence age inside the retained snapshot itself: robot telemetry
    // and object/lane observation times are frozen at capture, and nothing can refresh them
    // while the goal holds kSelectPair.
    const bool stale_evidence =
      code == SelectionErrorCode::RobotStale ||
      code == SelectionErrorCode::ObjectStale ||
      code == SelectionErrorCode::LaneStale;
    const auto budget = config_.perception_reacquire_timeout;
    const bool keyed =
      selection_reacquire_ &&
      selection_reacquire_->goal_generation == context_->generation();
    if (unlive && budget.count() > 0 && perception_liveness_) {
      if (!keyed) {
        selection_reacquire_ = SelectionReacquire{
          now, bounded_deadline(now, budget), context_->generation(), false};
        std::string live_detail;
        (void)perception_liveness_(live_detail);
        report_perception_reacquire(
          "task selection found the perception stream unlive; bounded reacquire "
          "started (" + std::to_string(budget.count()) + " ms): " + live_detail, now);
        return false;
      }
      if (now < selection_reacquire_->deadline) {
        // The predicate passed but the selector still saw stale data: a race at the
        // horizon edge. Keep the original deadline; the budget never restarts.
        return false;
      }
    }
    if (stale_evidence && keyed) {
      // The reacquire waited out a stream gap against evidence captured before it: the
      // live predicate passes on the new frames while the retained snapshot's own stamps
      // have aged past their horizons, and no further wait can make them younger.
      // Supersede back to re-observation — the pre-existing bounded selection restart —
      // so the retry selects on a refreshed snapshot inside the same goal budget.
      dispatch(
        RestockTaskEvent::kSelectionSuperseded,
        "selection evidence aged during the perception reacquire; re-observing the "
        "scene: " + selected.error().detail, now);
      return true;
    }
    primary_outcome_ = selection_outcome(code);
    primary_detail_ = "task selection failed: " + selected.error().detail;
    if (unlive && keyed) {
      *primary_detail_ +=
        "; bounded perception reacquire of " + std::to_string(budget.count()) +
        " ms did not restore it";
      std::string live_detail;
      if (perception_liveness_) {
        (void)perception_liveness_(live_detail);
      }
      report_perception_reacquire(
        "perception reacquire expired during task selection after " +
        std::to_string(budget.count()) + " ms: " + live_detail, now);
    }
    selection_reacquire_.reset();
    fail_operation(RestockTaskEvent::kTerminalFailure, *primary_detail_, now);
    return true;
  }
  selection_reacquire_.reset();
  const auto retained = context_->retain_selection(std::move(selected.value()));
  if (!retained) {
    inhibit("selected pair violates snapshot lineage: " + retained.detail, now);
    return false;
  }
  dispatch(RestockTaskEvent::kOperationSucceeded, "task pair selected", now);
  return true;
}

void RestockCoordinatorDriver::advance(SteadyTime now)
{
  if (guard_generic_advance()) {
    return;
  }
  for (std::size_t step = 0U; context_ && step < kMaximumAdvanceSteps; ++step) {
    if (guard_generic_advance()) {
      return;
    }
    if (terminal_output_emitted_) {
      return;
    }
    if (inhibited_) {
      if (const auto ticket = ledger_.pending();
        ticket && ticket->effect == OperationEffect::kIdempotentMutation &&
        ledger_.phase() != PendingOperationPhase::kReconciliationExhausted)
      {
        if (ledger_.phase() == PendingOperationPhase::kReconciliationRequired &&
          !ledger_.active_reconciliation() &&
          ledger_.next_reconciliation_kind() == ReconciliationKind::kReadback)
        {
          begin_reconciliation(now);
        }
        return;
      }
      const auto state = context_->task_status().state;
      if (state == RestockTaskState::kRequestOperator ||
        state == RestockTaskState::kCanceled || state == RestockTaskState::kComplete)
      {
        finish_if_terminal(now);
        return;
      }
      if (state == RestockTaskState::kFault) {
        dispatch(RestockTaskEvent::kOperationSucceeded, "motion inhibition latched", now);
      } else {
        dispatch(
          RestockTaskEvent::kTerminalFailure,
          "external inconsistency requires operator intervention", now);
      }
      continue;
    }
    if (const auto phase = ledger_.phase()) {
      if (*phase == PendingOperationPhase::kReconciliationRequired &&
        !ledger_.active_reconciliation())
      {
        begin_reconciliation(now);
      }
      return;
    }

    const auto transition = context_->task_status();
    if (transition.safe_abort_requested && !transition.reservation_active &&
      (transition.state == RestockTaskState::kValidateScene ||
      transition.state == RestockTaskState::kSelectPair ||
      transition.state == RestockTaskState::kReserveTask))
    {
      fail_operation(
        RestockTaskEvent::kTerminalFailure,
        "pre-reservation work stopped at the coordinator drain boundary", now);
      continue;
    }
    switch (transition.command) {
      case RestockTaskCommand::kValidateScene:
        if (const auto ticket = start_operation(
            transition.command, OperationEffect::kReadOnly, "", now))
        {
          request_snapshot(*ticket);
        }
        return;
      case RestockTaskCommand::kSelectPair:
        if (!advance_selection(now)) {
          return;
        }
        continue;
      case RestockTaskCommand::kReserveTask:
        if (!context_->reserve_request()) {
          const auto operation_id = context_->operation_id_for(
            CoordinatorMutationKind::kReserveTask);
          if (!operation_id || !context_->latest_snapshot() || !context_->selection()) {
            inhibit("reserve command cannot construct an exact retained request", now);
            return;
          }
          auto request = make_reserve_task_request(
            *context_->latest_snapshot(), *context_->selection(), *operation_id);
          if (!request) {
            // The exact-validation guards are numerous; the receipt must say which one fired
            // or the terminal is undiagnosable from logs alone (Card 010 attempt 9).
            inhibit(
              "reserve request construction failed exact validation: " + request.error().detail,
              now);
            return;
          }
          if (const auto retained = context_->retain_reserve_request(request.value()); !retained) {
            inhibit(
              "reserve request construction failed exact validation: " + retained.detail, now);
            return;
          }
        }
        if (!context_->reservation()) {
          const auto ticket = start_operation(
            transition.command, OperationEffect::kIdempotentMutation,
            context_->reserve_request()->request_id, now);
          if (!ticket) {
            return;
          }
          const auto submission = admission_.commit_mutation_submission(
            context_->goal_id(), context_->generation(), ticket->operation_generation,
            CoordinatorMutationKind::kReserveTask, now, ticket->deadline);
          if (submission.decision != MutationSubmissionDecision::kCommitted &&
            submission.decision != MutationSubmissionDecision::kReplayCommitted)
          {
            (void)ledger_.complete(
              ticket->goal_generation, ticket->operation_generation, now);
            fail_operation(
              RestockTaskEvent::kTerminalFailure,
              "reservation submission blocked by termination or admission state", now);
            continue;
          }
          request_reservation(*ticket);
          return;
        }
        if (transition.cancel_requested || transition.safe_abort_requested) {
          dispatch(
            RestockTaskEvent::kOperationSucceeded,
            "termination retained the reservation and skipped forward validation", now);
          continue;
        }
        if (!context_->reservation_validation()) {
          const auto ticket = start_operation(
            transition.command, OperationEffect::kReadOnly, "", now);
          if (!ticket) {
            return;
          }
          request_validation(*ticket);
          return;
        }
        dispatch(RestockTaskEvent::kOperationSucceeded, "reservation boundary proven", now);
        continue;
      case RestockTaskCommand::kGenerateGrasps:
        if (transition.cancel_requested || transition.safe_abort_requested) {
          dispatch(
            RestockTaskEvent::kOperationSucceeded,
            "pre-motion reservation boundary terminated", now);
          continue;
        }
        if (!context_->latest_snapshot() || !context_->selection()) {
          inhibit("grasp generation has no retained authoritative inputs", now);
          return;
        }
        {
          // Recovery from kCloseGripper/kVerifyGrasp re-enters here after verify adopted a
          // fresher snapshot (observes_fresh_evidence retains latest before the verdict). The
          // retained pair still carries the pre-verify revision stamps, so generate's lineage
          // check would reject "snapshot, selection, and robot rail lineage are inconsistent"
          // even though the pair identity is unchanged. Rebind stamps to the adopted snapshot;
          // object_id/lane_id stay the ones selection already proved eligible.
          if (const auto rebound = context_->rebind_selection_to_latest_snapshot(); !rebound) {
            reject_grasp_staging(
              GraspCandidateError{
              GraspCandidateErrorCode::InvalidCandidateBatch, rebound.detail},
              "grasp generation could not rebind selection to the latest snapshot: ", now);
            continue;
          }
          const rclcpp::Time generated_at = ros_now_();
          if (guard_generic_advance()) {
            return;
          }
          auto authority = grasp_authority_(*context_->latest_snapshot());
          if (guard_generic_advance()) {
            return;
          }
          if (!authority) {
            reject_grasp_staging(
              authority.error(), "grasp authority rejected retained inputs: ", now);
            continue;
          }
          auto generated = grasp_generator_(
            *context_->latest_snapshot(), *context_->selection(), authority.value(),
            generated_at);
          if (guard_generic_advance()) {
            return;
          }
          if (!generated) {
            reject_grasp_staging(
              generated.error(), "grasp generation rejected authoritative inputs: ", now);
            continue;
          }
          const auto validated = validate_grasp_candidate_batch(
            generated.value(), *context_->latest_snapshot(), *context_->selection(),
            authority.value(), generated_at,
            config_.maximum_object_age, config_.maximum_robot_age,
            config_.maximum_future_skew);
          if (!validated) {
            reject_grasp_staging(
              validated.error(),
              "grasp generator contradicted deterministic lineage: ", now);
            continue;
          }
          const auto admission = admission_.snapshot();
          if (admission.cancel_requested || admission.safe_abort_requested) {
            if (admission.cancel_requested) {
              dispatch(
                RestockTaskEvent::kCancelRequested,
                "cancellation arrived during pure grasp generation", now);
            } else {
              dispatch(
                RestockTaskEvent::kSafeAbortRequested,
                "coordinator drain arrived during pure grasp generation", now);
            }
            dispatch(
              RestockTaskEvent::kOperationSucceeded,
              "pure grasp output discarded at the termination boundary", now);
            continue;
          }
          const auto retained = context_->retain_grasp_candidate_batch(
            std::move(generated.value()), authority.value(), generated_at,
            config_.maximum_object_age, config_.maximum_robot_age,
            config_.maximum_future_skew);
          if (!retained) {
            reject_grasp_staging(
              GraspCandidateError{
              retained.grasp_error.value_or(GraspCandidateErrorCode::InvalidCandidateBatch),
              retained.detail},
              "validated grasp output could not be retained: ", now);
            continue;
          }
          dispatch(RestockTaskEvent::kOperationSucceeded, "grasp candidates staged", now);
          continue;
        }
      case RestockTaskCommand::kRunAttachTransaction:
      case RestockTaskCommand::kRunDetachTransaction:
      case RestockTaskCommand::kRunCommitDetachment:
        {
          const bool attaching =
            transition.command == RestockTaskCommand::kRunAttachTransaction;
          const bool semantic_only =
            transition.command == RestockTaskCommand::kRunCommitDetachment;
          if (!attachment_) {
            inhibit("no attachment backend is composed", now);
            return;
          }
          const auto mutation_kind = attaching ? CoordinatorMutationKind::kCommitAttachment :
            (semantic_only ? CoordinatorMutationKind::kCommitDetachment :
            CoordinatorMutationKind::kSetPhysicalAttachment);
          const auto ticket = start_operation(
            transition.command, OperationEffect::kIdempotentMutation,
            context_->operation_id_for(mutation_kind).value_or(std::string{}), now);
          if (!ticket) {
            return;
          }
          request_attachment(*ticket, attaching, semantic_only);
        }
        return;
      case RestockTaskCommand::kCloseGripper:
      case RestockTaskCommand::kOpenGripper:
        {
          const bool closing = transition.command == RestockTaskCommand::kCloseGripper;
          // Both open states share one command; the state selects approach clearance or release.
          const bool approach_open =
            transition.state == RestockTaskState::kOpenGripperForApproach;
          const bool escape_open =
            transition.state == RestockTaskState::kOpenGripperForEscape;
          const GripperAperture aperture = closing ? GripperAperture::kHold :
            (escape_open ? GripperAperture::kEscapeClearance :
            (approach_open ? GripperAperture::kApproachClearance : GripperAperture::kRelease));
          if (!gripper_) {
            inhibit("no gripper backend is composed", now);
            return;
          }
          if (escape_open && !context_->grasp_escape()) {
            // Card 062: the record is retained only once a hold reached the gripper, so none
            // means the jaws were never commanded closed at this grasp: nothing to open.
            const std::string no_op =
              "grasp escape: no hold was ever submitted at this grasp, so there is nothing to "
              "open";
            report_task_receipt(no_op, now);
            dispatch(RestockTaskEvent::kOperationSucceeded, no_op, now);
            continue;
          }
          const auto ticket = start_operation(
            transition.command, OperationEffect::kReadOnly, "", now);
          if (!ticket) {
            return;
          }
          request_gripper(*ticket, aperture);
        }
        return;
      case RestockTaskCommand::kVerifyGrasp:
      case RestockTaskCommand::kVerifyPlacement:
      case RestockTaskCommand::kUpdateInventory:
      case RestockTaskCommand::kObserveDestination:
      case RestockTaskCommand::kSurveyDestination:
        {
          // Lane occupancy may have changed while the arm was busy, so these use fresh evidence.
          // SurveyDestination acquires at the retreat viewpoint first; that completion then
          // requests the snapshot proving last_verified cleared the post-release invalidate.
          const auto ticket = start_operation(
            transition.command, OperationEffect::kReadOnly, "", now);
          if (!ticket) {
            return;
          }
          if (transition.command == RestockTaskCommand::kSurveyDestination &&
            acquire_destination_observation_)
          {
            const auto & selection = context_->selection();
            if (!selection) {
              fail_operation(
                RestockTaskEvent::kRetryableFailure,
                "destination lane observation requires a retained selection",
                now);
              continue;
            }
            const auto inbox = inbox_;
            const auto gate = context_->generation_quiescence();
            const auto goal_id = context_->goal_id();
            const auto generation = context_->generation();
            const OperationCorrelation correlation{
              ticket->goal_generation, ticket->operation_generation};
            const bool submitted = acquire_destination_observation_(
              correlation, selection->lane_id.value,
              [inbox, gate, goal_id, generation, diagnostic = async_deposit_diagnostic_](
                CoordinatorLaneAcquireCompletion completion) {
                deposit_and_report(
                  diagnostic, "lane observation", gate, goal_id, generation, [&]() {
                    return inbox->push(std::move(completion));
                  });
              });
            if (!submitted) {
              fail_operation(
                RestockTaskEvent::kRetryableFailure,
                "destination lane observation could not be submitted at the retreat viewpoint",
                now);
              continue;
            }
            return;
          }
          request_snapshot(*ticket);
        }
        return;
      case RestockTaskCommand::kGeneratePlacement:
        if (!advance_generate_placement(now)) {
          return;
        }
        continue;
      case RestockTaskCommand::kPlanPreGrasp:
      case RestockTaskCommand::kPlanApproach:
      case RestockTaskCommand::kPlanRetract:
      case RestockTaskCommand::kPlanCarryStart:
      case RestockTaskCommand::kPlanPreInsert:
      case RestockTaskCommand::kPlanInsert:
      case RestockTaskCommand::kPlanRetreat:
        if (!advance_planned_segment(transition, now)) {
          return;
        }
        continue;
      case RestockTaskCommand::kExecutePreGrasp:
      case RestockTaskCommand::kExecuteApproach:
      case RestockTaskCommand::kExecuteRetract:
      case RestockTaskCommand::kExecuteCarryStart:
      case RestockTaskCommand::kExecutePreInsert:
      case RestockTaskCommand::kExecuteInsert:
      case RestockTaskCommand::kExecuteRetreat:
        if (!advance_executed_segment(transition, now)) {
          return;
        }
        continue;
      case RestockTaskCommand::kReleaseTaskReservation:
        {
          if (release_submission_exhausted_) {
            return;
          }
          const auto release_observed = steady_now_();
          if (guard_generic_advance()) {
            return;
          }
          if (!context_->release_proof() && command_deadline_ &&
            release_observed >= *command_deadline_)
          {
            stop_release_submission_without_proof(
              "terminal reservation release reached its immutable command deadline",
              *command_deadline_, release_observed);
            return;
          }
          if (!context_->release_request()) {
            const auto operation_id = context_->operation_id_for(
              CoordinatorMutationKind::kReleaseTask);
            if (!operation_id || !context_->reservation()) {
              inhibit("release command has no reservation capability", now);
              return;
            }
            // The outcome must match the stage being released: a detached, placed reservation
            // releases only as Succeeded, a reserved one only as Canceled or FailedSafe.
            // Milestone 10 §6 (Card 074): the stage decides when the terminal's intent
            // contradicts it — a placement world state already accepted releases as its own
            // success, so a deadline that lapsed a breath before the gripper confirmed the
            // release (Card 066 dense dev run 3: expired 0.285 s early, lane evidence then
            // committed the placement) is refused today by exact validation and latches an
            // operator over a product that is already in its lane. Everything else — the
            // transition-derived outcome, the fail-closed refusal — is unchanged.
            const auto & record = context_->reservation()->reservation;
            const bool placed = record.placed_in_destination;
            const bool succeeded =
              placed || transition.fault == RestockTaskFault::kNone;
            if (placed && transition.fault != RestockTaskFault::kNone) {
              report_task_receipt(
                "reservation " + std::to_string(record.reservation_id) +
                " observed placed at the terminal; releasing as Succeeded" +
                (transition.detail.empty() ? std::string() :
                " (terminal: " + transition.detail + ")") +
                " — Milestone 10 §6, Card 074",
                now);
            }
            // A recoverable skip releases like a cancellation: the reservation ends without a
            // placement (Milestone 10 §6 rung 5, Card 051) — the world state accepts the same
            // terminal outcome, while terminal_outcome() still reports the typed skip.
            const bool canceled =
              transition.fault == RestockTaskFault::kCanceled || transition.recoverable_skip;
            const bool normal_terminal = succeeded || canceled;
            auto request = make_release_task_reservation_request(
              *context_->reservation(), *operation_id,
              succeeded ? restocker_world_state::ReservationOutcome::Succeeded :
              (canceled ? restocker_world_state::ReservationOutcome::Canceled :
              restocker_world_state::ReservationOutcome::FailedSafe),
              normal_terminal ? restocker_world_state::TaskPhase::Idle :
              restocker_world_state::TaskPhase::Fault,
              normal_terminal ? restocker_world_state::FaultState::None :
              restocker_world_state::FaultState::Recoverable);
            if (!request || !context_->retain_release_request(request.value())) {
              // Name the record the request was built from, to tell a stale capability from a
              // wrong terminal outcome.
              const auto & built = context_->reservation()->reservation;
              std::ostringstream detail;
              detail << "release request construction failed exact validation: reservation "
                     << built.reservation_id << " observed at stage "
                     << static_cast<int>(built.stage) << " placed="
                     << (built.placed_in_destination ? "true" : "false") << " revision "
                     << built.revision << ", released as outcome "
                     << static_cast<int>(request ? 0 : 1);
              if (request) {
                detail << " (request built; retention refused)";
              }
              inhibit(detail.str(), now);
              return;
            }
          }
          if (!context_->release_acknowledgement()) {
            // Declared before submission: the admission slot admits a release only at a teardown
            // or completion boundary and cannot tell which from the task machine alone.
            const bool completion_release = transition.fault == RestockTaskFault::kNone;
            if (completion_release &&
              !admission_.begin_completion_cleanup(context_->goal_id(), context_->generation()))
            {
              inhibit("completed task could not claim its reservation-release boundary", now);
              return;
            }
            // Milestone 10 §6 rung 5 (Card 051): the recoverable skip releases its reservation
            // as its own teardown — neither a completion nor an external termination, so the
            // admission slot needs the skip's own claim before the cleanup mutation commits.
            if (!completion_release && transition.recoverable_skip &&
              !admission_.begin_skip_cleanup(context_->goal_id(), context_->generation()))
            {
              inhibit(
                "recoverable-skip release could not claim its cleanup boundary", now);
              return;
            }
            const auto ticket = start_operation(
              transition.command, OperationEffect::kIdempotentMutation,
              context_->release_request()->operation_id, release_observed);
            if (!ticket) {
              return;
            }
            const auto submission = admission_.commit_mutation_submission(
              context_->goal_id(), context_->generation(), ticket->operation_generation,
              CoordinatorMutationKind::kReleaseTask, release_observed, ticket->deadline);
            if (submission.decision != MutationSubmissionDecision::kCommitted &&
              submission.decision != MutationSubmissionDecision::kReplayCommitted)
            {
              (void)ledger_.complete(
                ticket->goal_generation, ticket->operation_generation, release_observed);
              inhibit(
                "terminal reservation release was blocked by admission state", release_observed);
              return;
            }
            request_release(*ticket);
            return;
          }
          if (!context_->release_proof()) {
            const auto ticket = start_operation(
              transition.command, OperationEffect::kReadOnly, "", release_observed);
            if (!ticket) {
              return;
            }
            request_snapshot(*ticket);
            return;
          }
          dispatch(RestockTaskEvent::kOperationSucceeded, "reservation release proven", now);
          latch_deferred_inhibition(now);
          continue;
        }
      case RestockTaskCommand::kExecuteRecovery:
        if (!advance_motion_recovery(transition, now)) {
          return;
        }
        continue;
      case RestockTaskCommand::kInhibitMotion:
        handle_inhibit_motion_latch(transition, now);
        continue;
      case RestockTaskCommand::kRequestOperator:
      case RestockTaskCommand::kNone:
        finish_if_terminal(now);
        return;
      default: {
          // Card 051 review note: this unsupported-command latch (the kCancelMotionAndVerifyStop
          // path the driver never implemented — master identical) is a terminal boundary too, so
          // it classifies before it latches, like the kInhibitMotion case above. After inhibit()
          // sets the flag the kInhibitMotion case is bypassed, so this receipt is the only one
          // this path gets.
          const std::string cause =
            transition.detail.empty() ? "driver reached an unsupported motion command" :
            transition.detail;
          report_task_receipt(recovery_classification_receipt(classify_current(cause), cause), now);
          inhibit("driver reached an unsupported motion command", now);
          return;
        }
    }
  }
  if (context_ && !terminal_output_emitted_) {
    inhibit("coordinator advance-step bound exhausted", now);
  }
}

void RestockCoordinatorDriver::handle_inhibit_motion_latch(
  const RestockTaskTransition & transition, SteadyTime now)
{
  // The latch boundary: every terminal that reaches here is classified against the
  // Milestone 10 §6 contract (Card 051) and receipted on one line. The class now also
  // decides the exit: RECOVERABLE takes the typed rung-5 skip, UNSAFE (or an exhausted
  // ladder — a second skip request) latches exactly as before.
  const std::string cause =
    transition.detail.empty() ? "task machine entered fault state" : transition.detail;
  const auto classification = classify_current(cause);
  report_task_receipt(recovery_classification_receipt(classification, cause), now);
  if (primary_outcome_ == RestockActionOutcome::kNoCompatiblePair) {
    dispatch(
      RestockTaskEvent::kOperationSucceeded,
      "selection ended safely without a compatible task pair", now);
    return;
  }
  if (primary_outcome_ == RestockActionOutcome::kObservationEvidenceStale) {
    dispatch(
      RestockTaskEvent::kOperationSucceeded,
      "selection ended safely because observation evidence is stale", now);
    return;
  }
  if (classification.klass == RecoveryClass::kRecoverable &&
    !transition.recoverable_skip)
  {
    if (transition.object_held) {
      // Documented boundary (Card 051): a held product needs a safe place, and the
      // skip's cleanup does not place one — latch, with the reason receipted.
      report_task_receipt(
        "recoverable skip refused: the arm holds a product and the cleanup cannot "
        "place it safely from here; latching for the operator", now);
    } else {
      const auto skip = context_->dispatch_task(
        RestockTaskEvent::kRecoverableSkipRequested,
        "recoverable skip (rung 5): " + cause);
      if (skip.accepted) {
        observe_transition(skip, now);
        primary_outcome_ = RestockActionOutcome::kRecoverableSkip;
        primary_detail_ = cause;
        report_task_receipt(
          "recovery rung 5 (recoverable skip): requested from the fault boundary; the "
          "cleanup retreat or release follows instead of an operator latch", now);
        return;
      }
      report_task_receipt(
        "recoverable skip refused by the task machine: " + skip.detail, now);
    }
  }
  // A skip whose cleanup could not succeed has latched: the typed outcome must not
  // survive the latch — the campaign has to see the operator terminal it really got
  // (a skip result beside an inhibited admission is exactly the mismatch that made
  // the next goal "rejected"). The budget charge for the accepted attempt stands.
  if (primary_outcome_ &&
    *primary_outcome_ == RestockActionOutcome::kRecoverableSkip)
  {
    primary_outcome_.reset();
    primary_detail_.reset();
  }
  inhibit(cause, now);
  dispatch(RestockTaskEvent::kOperationSucceeded, "motion inhibition latched", now);
  return;
}


void RestockCoordinatorDriver::reject_grasp_staging(
  const GraspCandidateError & error, std::string prefix, SteadyTime now)
{
  if (!context_) {
    inhibit("grasp staging rejection has no active goal context", now);
    return;
  }
  const bool internal = internal_grasp_error(error.code);
  primary_outcome_ = internal ? RestockActionOutcome::kExternalInconsistency :
    RestockActionOutcome::kValidationFailed;
  primary_detail_ = std::move(prefix) + error.detail;
  if (internal) {
    deferred_inhibition_detail_ = *primary_detail_;
  }
  const auto pending = ledger_.pending();
  auto route = termination_router_.route_safe_abort(
    context_->goal_id(), context_->generation(), now,
    pending ? pending->operation_generation : 0U, *primary_detail_);
  auto routed_event = route.take_event();
  if (!termination_latch_succeeded(route.admission_decision()) || !routed_event) {
    inhibit("grasp rejection could not latch exact failed-safe cleanup", now);
    return;
  }
  dispatch(
    RestockTaskEvent::kSafeAbortRequested, std::move(routed_event->detail),
    routed_event->arrived_at);
  dispatch(
    RestockTaskEvent::kOperationSucceeded,
    "grasp rejection reached the reservation cleanup boundary", now);
}

std::string feedback_detail_for_dispatch(
  RestockTaskEvent event, FeedbackDetailPolicy policy,
  const RestockTaskTransition & transition)
{
  const bool terminal =
    transition.state == RestockTaskState::kFault ||
    transition.state == RestockTaskState::kRequestOperator;
  const bool classified_planning_miss =
    event == RestockTaskEvent::kRetryableFailure &&
    policy == FeedbackDetailPolicy::kRetriablePlanningDiagnostic;
  if (!classified_planning_miss || terminal || transition.detail.empty()) {
    return transition.detail;
  }
  // The classification came from the kPlanningFailed call site (typed segment policy); the
  // command only names the leg in the diagnostic text.
  return std::string("retriable operation failure; bounded retry in progress: ") +
         to_string(transition.command) + " attempt " +
         std::to_string(transition.attempt) + ", recovery attempt " +
         std::to_string(transition.recovery_attempt);
}

void RestockCoordinatorDriver::observe_transition(
  const RestockTaskTransition & transition, SteadyTime now,
  std::optional<RestockTaskEvent> source_event, FeedbackDetailPolicy feedback_policy)
{
  if (!context_) {
    return;
  }
  const bool new_command = !observed_state_ || *observed_state_ != transition.state ||
    observed_attempt_ != transition.attempt ||
    observed_recovery_attempt_ != transition.recovery_attempt;
  if (new_command && perception_reacquire_ &&
    (perception_reacquire_->goal_generation != context_->generation() ||
    planned_motion_segment(transition.command) != perception_reacquire_->segment))
  {
    // The task machine left the plan state the hold belonged to (termination, inhibition or a
    // different command), so the hold is an orphan: drop it rather than report a restoration
    // for work it was never started for.
    perception_reacquire_.reset();
  }
  // fail_operation does not complete the ledger, so a failure raised while a motion segment is
  // outstanding leaves it booked and it would later expire as a motion deadline for work the port
  // is not doing. Report such an operation.
  if (new_command) {
    if (const auto stale = ledger_.pending();
      stale && stale->effect == OperationEffect::kReadOnly &&
      planned_motion_segment(stale->command) && async_deposit_diagnostic_)
    {
      try {
        async_deposit_diagnostic_(
          std::string("task machine advanced while ") +
          motion_segment_name(*planned_motion_segment(stale->command)) + " operation " +
          std::to_string(stale->operation_generation) +
          " was still pending: nothing owns that operation now and it will expire at its own "
          "deadline");
      } catch (...) {
        // An undeliverable diagnostic must not stop the pump.
      }
    }
  }
  if (new_command) {
    // Complete the audit record with the state the machine left recovery for.
    if (recovery_outcome_pending_ && observed_state_ &&
      *observed_state_ == RestockTaskState::kRecover &&
      transition.state != RestockTaskState::kRecover)
    {
      auto record = std::move(*recovery_outcome_pending_);
      recovery_outcome_pending_.reset();
      record.outcome = std::string("recovery left for ") + to_string(transition.state) +
        (transition.detail.empty() ? std::string() : ": " + transition.detail);
      if (recovery_audit_) {
        recovery_audit_->append(record);
      }
    }
    if (observed_state_ && *observed_state_ == RestockTaskState::kRecover &&
      transition.state != RestockTaskState::kRecover)
    {
      transaction_rollback_recovery_evidence_.reset();
      report_task_receipt(
        "recovery attempt " + std::to_string(observed_recovery_attempt_) + " of " +
        std::to_string(config_.task.max_recovery_attempts) + " resumed at " +
        to_string(transition.state), now);
      // Milestone 10 §6 rung 3 (Card 051): the retry runs against the world the recovery
      // re-observed (the kExecuteRecovery snapshot adopted as latest evidence), charged to the
      // existing operation and recovery budgets — one line carries the revision and both budgets.
      if (context_->latest_snapshot()) {
        report_task_receipt(
          std::string("recovery rung 3 (retry with fresh evidence): resuming at ") +
          to_string(transition.state) + " against world revision " +
          std::to_string(context_->latest_snapshot()->revision) + ", operation attempt " +
          std::to_string(transition.attempt) + " of " +
          std::to_string(config_.task.max_operation_retries + 1U) + ", recovery attempt " +
          std::to_string(transition.recovery_attempt) + " of " +
          std::to_string(config_.task.max_recovery_attempts),
          now);
      }
    }
    if (transition.state == RestockTaskState::kRecover &&
      transition.recovery_attempt > observed_recovery_attempt_)
    {
      // Written before observed_state_ is overwritten, so the line names the state recovery left.
      report_task_receipt(
        "recovery attempt " + std::to_string(transition.recovery_attempt) + " of " +
        std::to_string(config_.task.max_recovery_attempts) + " entered from " +
        (observed_state_ ? to_string(*observed_state_) : std::string("no observed state")) +
        (transition.detail.empty() ? std::string() : ": " + transition.detail), now);
    }
    observed_state_ = transition.state;
    observed_attempt_ = transition.attempt;
    observed_recovery_attempt_ = transition.recovery_attempt;
    const bool holds_without_backend =
      transition.state == RestockTaskState::kPlanPreGrasp && motion_ == nullptr;
    if (transition.command == RestockTaskCommand::kNone ||
      transition.command == RestockTaskCommand::kRequestOperator ||
      transition.state == RestockTaskState::kGenerateGrasps ||
      holds_without_backend)
    {
      command_deadline_.reset();
    } else if (planned_motion_segment(transition.command) && motion_) {
      // The port plans and executes in one operation, so the budget covers both.
      command_deadline_ = bounded_deadline(
        now, transition.timeout + config_.task.execution_timeout);
    } else {
      command_deadline_ = bounded_deadline(now, transition.timeout);
    }
  }
  outputs_.push_back(
    CoordinatorDriverOutput{
      CoordinatorDriverOutputKind::kFeedback, context_->generation(), transition,
      context_->selection(), RestockActionOutcome::kUnset, metrics(now),
      source_event ?
      feedback_detail_for_dispatch(*source_event, feedback_policy, transition) :
      transition.detail});
}

void RestockCoordinatorDriver::dispatch(
  RestockTaskEvent event, std::string detail, SteadyTime now,
  FeedbackDetailPolicy feedback_policy)
{
  if (!context_) {
    return;
  }
  const auto transition = context_->dispatch_task(event, std::move(detail));
  if (!transition.accepted) {
    inhibit("task machine rejected coordinator dispatch: " + transition.detail, now);
    return;
  }
  observe_transition(transition, now, event, feedback_policy);
}

void RestockCoordinatorDriver::dispatch_execution(
  RestockTaskEvent event, std::string detail, OperationGeneration operation_generation,
  SteadyTime now)
{
  if (!context_) {
    return;
  }
  const auto transition =
    context_->dispatch_task(event, std::move(detail), operation_generation);
  if (!transition.accepted) {
    inhibit("task machine rejected execution evidence: " + transition.detail, now);
    return;
  }
  observe_transition(transition, now);
}

void RestockCoordinatorDriver::begin_deferred_inhibition_cleanup(
  std::string detail, SteadyTime evidence_time, SteadyTime now)
{
  if (!context_) {
    inhibit(std::move(detail), now);
    return;
  }
  const auto pending = ledger_.pending();
  const bool mutation_authority_may_remain = pending &&
    pending->effect == OperationEffect::kIdempotentMutation;
  if (!context_->reservation() && !mutation_authority_may_remain) {
    inhibit(std::move(detail), now);
    return;
  }
  primary_outcome_ = RestockActionOutcome::kExternalInconsistency;
  primary_detail_ = detail;
  deferred_inhibition_detail_ = std::move(detail);
  continue_deferred_inhibition_cleanup(evidence_time, now);
}

void RestockCoordinatorDriver::continue_deferred_inhibition_cleanup(
  SteadyTime evidence_time, SteadyTime now)
{
  if (!deferred_inhibition_detail_ || !context_) {
    return;
  }
  const auto pending = ledger_.pending();
  if (pending && pending->effect == OperationEffect::kIdempotentMutation) {
    return;
  }
  if (!context_->reservation()) {
    latch_deferred_inhibition(now);
    return;
  }
  const auto task = context_->task_status();
  if (!task.safe_abort_requested && task.state != RestockTaskState::kReleaseTask) {
    dispatch(
      RestockTaskEvent::kSafeAbortRequested, *deferred_inhibition_detail_, evidence_time);
  }
  abort_read_only_for_termination(evidence_time, now);
}

void RestockCoordinatorDriver::stop_release_submission_without_proof(
  std::string detail, SteadyTime evidence_time, SteadyTime now)
{
  release_submission_exhausted_ = true;
  begin_deferred_inhibition_cleanup(std::move(detail), evidence_time, now);
}

void RestockCoordinatorDriver::fail_operation(
  RestockTaskEvent event, std::string detail, SteadyTime now,
  FeedbackDetailPolicy feedback_policy)
{
  dispatch(event, std::move(detail), now, feedback_policy);
}

void RestockCoordinatorDriver::inhibit(std::string detail, SteadyTime now)
{
  if (!inhibited_) {
    admission_.inhibit(detail);
    inhibited_ = true;
    outputs_.push_back(
      CoordinatorDriverOutput{
        CoordinatorDriverOutputKind::kInhibited,
        context_ ? context_->generation() : 0U,
        context_ ? context_->task_status() : RestockTaskTransition{},
        context_ ? context_->selection() : std::nullopt,
        RestockActionOutcome::kExternalInconsistency, metrics(now), detail});
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->effect == OperationEffect::kReadOnly)
  {
    remove_operation_requests({ticket->goal_generation, ticket->operation_generation});
    (void)ledger_.complete(ticket->goal_generation, ticket->operation_generation, now);
  }
  if (!context_ || context_->task_status().state == RestockTaskState::kFault ||
    context_->task_status().state == RestockTaskState::kRequestOperator)
  {
    return;
  }
  if (const auto ticket = ledger_.pending();
    ticket && ticket->effect == OperationEffect::kIdempotentMutation &&
    ledger_.phase() != PendingOperationPhase::kReconciliationExhausted)
  {
    return;
  }
  const auto transition = context_->dispatch_task(
    RestockTaskEvent::kTerminalFailure, std::move(detail));
  if (transition.accepted) {
    observe_transition(transition, now);
  }
}

void RestockCoordinatorDriver::latch_deferred_inhibition(SteadyTime now)
{
  if (!deferred_inhibition_detail_ || inhibited_) {
    return;
  }
  const auto admission = admission_.snapshot();
  if ((context_ && context_->reservation() && !context_->release_proof()) || ledger_.pending() ||
    admission.mutation_submission)
  {
    return;
  }
  admission_.inhibit(*deferred_inhibition_detail_);
  inhibited_ = true;
  outputs_.push_back(
    CoordinatorDriverOutput{
      CoordinatorDriverOutputKind::kInhibited,
      context_ ? context_->generation() : 0U,
      context_ ? context_->task_status() : RestockTaskTransition{},
      context_ ? context_->selection() : std::nullopt,
      RestockActionOutcome::kExternalInconsistency, metrics(now),
      *deferred_inhibition_detail_});
}

void RestockCoordinatorDriver::report_perception_reacquire(
  const std::string & detail, SteadyTime now)
{
  report_task_receipt(detail, now);
}

void RestockCoordinatorDriver::report_task_receipt(const std::string & detail, SteadyTime now)
{
  if (!context_) {
    return;
  }
  outputs_.push_back(
    CoordinatorDriverOutput{
      CoordinatorDriverOutputKind::kFeedback, context_->generation(),
      context_->task_status(), context_->selection(), RestockActionOutcome::kUnset, metrics(now),
      detail});
}

std::string RestockCoordinatorDriver::execution_identity_refusal_detail(
  const std::string & guard, SteadyTime now) const
{
  std::ostringstream stream;
  stream << guard;
  if (!context_) {
    return stream.str();
  }
  const auto admission = admission_.snapshot();
  if (admission.termination_intent == GoalTerminationIntent::kNone) {
    return stream.str();
  }
  const char * named = "termination";
  switch (admission.termination_intent) {
    case GoalTerminationIntent::kTaskDeadline:
      named = "whole-task steady deadline";
      break;
    case GoalTerminationIntent::kUserCancel:
      named = "client cancellation";
      break;
    case GoalTerminationIntent::kShutdownDrain:
      named = "coordinator drain";
      break;
    case GoalTerminationIntent::kSafeAbort:
      named = "safe abort";
      break;
    case GoalTerminationIntent::kNone:
      break;
  }
  stream << ": " << named << " at steady +" <<
    std::chrono::duration_cast<std::chrono::milliseconds>(
    now - context_->steady_started()).count() <<
    " ms of a " << config_.task.total_timeout.count() <<
    " ms budget since goal acceptance, sim +" <<
    std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::nanoseconds(
      (ros_now_() - context_->simulation_started()).nanoseconds())).count() <<
    " ms";
  return stream.str();
}

void RestockCoordinatorDriver::finish_if_terminal(SteadyTime now)
{
  if (guard_generic_cleanup()) {
    return;
  }
  if (!context_ || terminal_output_emitted_) {
    return;
  }
  const auto transition = context_->task_status();
  // Rung 5 for the whole-task deadline (Milestone 10 §6, Card 051): the cleanup ended at
  // kCanceled with the deadline's own fault. If the classification is RECOVERABLE (verified
  // stop, known held state) the delivered outcome is the typed recoverable skip — or, when the
  // placement already committed (Milestone 10 §6, Card 074), that placement's success;
  // otherwise the deadline keeps exactly today's verdict. Receipted either way.
  if (transition.state == RestockTaskState::kCanceled &&
    transition.fault == RestockTaskFault::kTimedOut &&
    primary_outcome_ && *primary_outcome_ == RestockActionOutcome::kPlanningFailed)
  {
    const auto classification = classify_current("whole-task steady deadline exceeded");
    report_task_receipt(
      recovery_classification_receipt(
        classification, "whole-task steady deadline terminal"),
      now);
    if (classification.klass == RecoveryClass::kRecoverable) {
      const bool placed = context_->reservation() &&
        context_->reservation()->reservation.placed_in_destination;
      if (placed) {
        // Milestone 10 §6 (Card 074): the placement committed before this terminal — the goal
        // delivers that placement's success, receipted with the expiry that asked for the
        // cleanup, so the campaign counts the transfer instead of blocking on a goal whose
        // product is already in its lane. The not-placed case keeps the typed skip below.
        primary_outcome_ = RestockActionOutcome::kSucceeded;
        primary_detail_ =
          "whole-task steady deadline exceeded: the placement committed before the terminal; "
          "delivered as success with the expiry receipted (Milestone 10 §6, Card 074)";
        report_task_receipt(
          "whole-task deadline terminal: reservation observed placed; delivering the "
          "placement's success instead of the typed skip (Milestone 10 §6, Card 074)",
          now);
      } else {
        primary_outcome_ = RestockActionOutcome::kRecoverableSkip;
        primary_detail_ =
          "whole-task steady deadline exceeded: verified stop and held state established; "
          "recoverable skip (rungs 1 and 5)";
      }
    }
  }
  // Operator-required counts as settled: the machine refuses a safe abort once a trajectory may
  // have moved the arm (kSafeAbortRequested), so no release proof can be obtained and the
  // coordinator does not auto-release a capability left in an unproven state. Waiting for a
  // proof would hide the operator request; startup reports such a reservation as orphaned.
  const bool operator_owns_reservation =
    transition.state == RestockTaskState::kRequestOperator;
  const bool reservation_authority_settled = operator_owns_reservation ||
    !context_->reservation() || context_->release_proof().has_value();
  if (!reservation_authority_settled || ledger_.pending() ||
    admission_.snapshot().mutation_submission)
  {
    // The action result is final; withhold it while reservation ownership or an idempotent
    // mutation may be unresolved.
    return;
  }
  latch_deferred_inhibition(now);
  CoordinatorDriverOutputKind output_kind;
  if (transition.state == RestockTaskState::kCanceled &&
    terminal_outcome() == RestockActionOutcome::kSucceeded)
  {
    // Milestone 10 §6 (Card 074): a machine canceled by the deadline whose placement
    // committed delivers success — and the output contract pairs kSucceeded only with a
    // kSucceeded kind, never with the kAborted this state would otherwise take.
    output_kind = CoordinatorDriverOutputKind::kSucceeded;
  } else if (transition.state == RestockTaskState::kCanceled) {
    output_kind = transition.fault == RestockTaskFault::kCanceled ?
      CoordinatorDriverOutputKind::kCanceled : CoordinatorDriverOutputKind::kAborted;
  } else if (transition.state == RestockTaskState::kComplete) {
    output_kind = CoordinatorDriverOutputKind::kSucceeded;
  } else if (transition.state == RestockTaskState::kRequestOperator) {
    output_kind = CoordinatorDriverOutputKind::kAborted;
  } else {
    return;
  }
  terminal_output_emitted_ = true;
  // A goal that terminals while a reacquire hold was still up must not report the hold
  // as active any longer; the hold's own terminal paths already cleared theirs.
  perception_reacquire_.reset();
  selection_reacquire_.reset();
  outputs_.push_back(
    CoordinatorDriverOutput{
      output_kind, context_->generation(), transition, context_->selection(), terminal_outcome(),
      metrics(now), primary_detail_.value_or(transition.detail)});
}

void RestockCoordinatorDriver::release_acknowledged_terminal()
{
  if (guard_generic_cleanup()) {
    return;
  }
  if (!context_ || !terminal_output_emitted_ || !terminal_delivery_acknowledged_) {
    return;
  }
  const bool capability_released = !context_->reservation() || context_->release_proof();
  if (!ledger_.pending() && !admission_.snapshot().mutation_submission && capability_released) {
    const auto goal_id = context_->goal_id();
    const auto generation = context_->generation();
    if (admission_.finish(goal_id, generation)) {
      context_.reset();
      commanded_ = {};
      command_deadline_.reset();
      task_deadline_.reset();
      deadline_retreat_until_.reset();
      deadline_observed_after_first_termination_ = false;
      observed_state_.reset();
      primary_outcome_.reset();
      primary_detail_.reset();
      deferred_inhibition_detail_.reset();
      // Nothing from the finished goal may reach the next one.
      forget_recovery_question();
      recovery_outcome_pending_.reset();
      perception_reacquire_.reset();
      selection_reacquire_.reset();
      inactive_after_terminal_ack_generation_ = generation;
    } else {
      admission_.inhibit("terminal goal could not release its admission capability");
      inhibited_ = true;
    }
  }
}

std::optional<OperationTicket> RestockCoordinatorDriver::start_operation(
  RestockTaskCommand command, OperationEffect effect, std::string operation_id,
  SteadyTime now)
{
  if (!context_ || !command_deadline_ || now >= *command_deadline_) {
    fail_operation(RestockTaskEvent::kTimeout, "command deadline expired before submission", now);
    return std::nullopt;
  }
  const auto started = ledger_.start(
    context_->generation(), command, effect, std::move(operation_id), now,
    *command_deadline_);
  if (!started.ticket) {
    inhibit("operation ledger rejected a non-overlapping command", now);
  }
  return started.ticket;
}

void RestockCoordinatorDriver::request_snapshot(const OperationTicket & ticket)
{
  auto request = std::make_shared<WorldStateCoordinatorPort::GetSnapshot::Request>();
  request->include_removed = false;
  request->include_events = false;
  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  const bool cleanup_readback = ticket.command == RestockTaskCommand::kReleaseTaskReservation;
  const auto sent = world_state_.get_snapshot(
    {ticket.goal_generation, ticket.operation_generation}, request,
    [inbox, gate, goal_id, generation, clock, cleanup_readback,
    diagnostic = async_deposit_diagnostic_](auto completion) {
      deposit_and_report(
        diagnostic, "snapshot", gate, goal_id, generation, [&]() {
          SnapshotCompletion retained{
            completion.correlation, std::move(completion.response),
            std::move(completion.transport_error), clock(), std::nullopt};
          if (cleanup_readback) {
            return inbox->push_cleanup(std::move(retained));
          }
          return inbox->push(std::move(retained));
        });
    });
  if (sent) {
    record_request_handle(*sent.handle);
    return;
  }
  const auto observed = steady_now_();
  const auto completion = ledger_.complete(
    ticket.goal_generation, ticket.operation_generation, observed);
  if (guard_generic_advance()) {
    return;
  }
  fail_operation(
    completion.disposition == OperationCompletionDisposition::kDeliverTimeout ?
    RestockTaskEvent::kTimeout : RestockTaskEvent::kRetryableFailure,
    sent.detail, observed);
}

void RestockCoordinatorDriver::request_reservation(
  const OperationTicket & ticket, bool reconciliation_replay, std::size_t attempt_number)
{
  auto request = std::make_shared<WorldStateCoordinatorPort::ReserveTask::Request>(
    *context_->reserve_request());
  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  const auto sent = world_state_.reserve_task(
    {ticket.goal_generation, ticket.operation_generation}, request,
    [inbox, gate, goal_id, generation, clock, reconciliation_replay, attempt_number](
      auto completion) {
      (void)deposit_for_generation(
        gate, goal_id, generation, [&]() {
          return inbox->push(
            ReserveTaskCompletion{
          completion.correlation, std::move(completion.response),
          std::move(completion.transport_error), clock(),
          reconciliation_replay ?
          std::optional<CoordinatorReconciliationCorrelation>(
            CoordinatorReconciliationCorrelation{
            ReconciliationKind::kReplayMutation, attempt_number}) : std::nullopt});
        });
    });
  if (sent) {
    record_request_handle(
      *sent.handle,
      reconciliation_replay ?
      std::optional<CoordinatorReconciliationCorrelation>(
        CoordinatorReconciliationCorrelation{
        ReconciliationKind::kReplayMutation, attempt_number}) : std::nullopt);
    return;
  }
  const auto observed = steady_now_();
  if (reconciliation_replay) {
    (void)ledger_.complete_reconciliation(
      ticket.goal_generation, ticket.operation_generation,
      ReconciliationKind::kReplayMutation, attempt_number, observed);
    return;
  }
  const bool submission_retired = admission_.confirm_mutation_not_submitted(
    context_->goal_id(), context_->generation(), ticket.operation_generation,
    CoordinatorMutationKind::kReserveTask) &&
    ledger_.confirm_not_submitted(ticket.goal_generation, ticket.operation_generation) &&
    admission_.resolve_mutation_submission(
    context_->goal_id(), context_->generation(), ticket.operation_generation,
    CoordinatorMutationKind::kReserveTask);
  if (guard_generic_advance()) {
    return;
  }
  if (!submission_retired) {
    inhibit("rejected reservation submission could not retire mutation ownership", observed);
    return;
  }
  fail_operation(RestockTaskEvent::kRetryableFailure, sent.detail, observed);
}

void RestockCoordinatorDriver::request_validation(
  const OperationTicket & ticket, bool reconciliation_readback,
  std::size_t attempt_number)
{
  auto checked = make_validate_task_reservation_request(*context_->reservation());
  if (!checked) {
    const auto observed = steady_now_();
    if (reconciliation_readback) {
      (void)ledger_.complete_reconciliation(
        ticket.goal_generation, ticket.operation_generation,
        ReconciliationKind::kReadback, attempt_number, observed);
    } else {
      (void)ledger_.complete(ticket.goal_generation, ticket.operation_generation, observed);
    }
    if (guard_generic_advance()) {
      return;
    }
    inhibit("reservation validation request cannot be constructed", observed);
    return;
  }
  auto request = std::make_shared<WorldStateCoordinatorPort::ValidateReservation::Request>(
    std::move(checked.value()));
  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  const auto sent = world_state_.validate_reservation(
    {ticket.goal_generation, ticket.operation_generation}, request,
    [inbox, gate, goal_id, generation, clock, reconciliation_readback, attempt_number](
      auto completion) {
      (void)deposit_for_generation(
        gate, goal_id, generation, [&]() {
          return inbox->push(
            ValidateReservationCompletion{
          completion.correlation, std::move(completion.response),
          std::move(completion.transport_error), clock(),
          reconciliation_readback ?
          std::optional<CoordinatorReconciliationCorrelation>(
            CoordinatorReconciliationCorrelation{
            ReconciliationKind::kReadback, attempt_number}) : std::nullopt});
        });
    });
  if (sent) {
    record_request_handle(
      *sent.handle,
      reconciliation_readback ?
      std::optional<CoordinatorReconciliationCorrelation>(
        CoordinatorReconciliationCorrelation{ReconciliationKind::kReadback, attempt_number}) :
      std::nullopt);
    return;
  }
  const auto observed = steady_now_();
  if (reconciliation_readback) {
    (void)ledger_.complete_reconciliation(
      ticket.goal_generation, ticket.operation_generation, ReconciliationKind::kReadback,
      attempt_number, observed);
  } else {
    const auto completion = ledger_.complete(
      ticket.goal_generation, ticket.operation_generation, observed);
    if (guard_generic_advance()) {
      return;
    }
    fail_operation(
      completion.disposition == OperationCompletionDisposition::kDeliverTimeout ?
      RestockTaskEvent::kTimeout : RestockTaskEvent::kRetryableFailure,
      sent.detail, observed);
  }
}

void RestockCoordinatorDriver::request_release(
  const OperationTicket & ticket, bool reconciliation_replay, std::size_t attempt_number)
{
  auto request = std::make_shared<WorldStateCoordinatorPort::ReleaseReservation::Request>(
    *context_->release_request());
  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  const auto sent = world_state_.release_reservation(
    {ticket.goal_generation, ticket.operation_generation}, request,
    [inbox, gate, goal_id, generation, clock, reconciliation_replay, attempt_number](
      auto completion) {
      (void)deposit_for_generation(
        gate, goal_id, generation, [&]() {
          return inbox->push_cleanup(
            ReleaseReservationCompletion{
          completion.correlation, std::move(completion.response),
          std::move(completion.transport_error), clock(),
          reconciliation_replay ?
          std::optional<CoordinatorReconciliationCorrelation>(
            CoordinatorReconciliationCorrelation{
            ReconciliationKind::kReplayMutation, attempt_number}) : std::nullopt});
        });
    });
  if (sent) {
    record_request_handle(
      *sent.handle,
      reconciliation_replay ?
      std::optional<CoordinatorReconciliationCorrelation>(
        CoordinatorReconciliationCorrelation{
        ReconciliationKind::kReplayMutation, attempt_number}) : std::nullopt);
    return;
  }
  const auto observed = steady_now_();
  if (reconciliation_replay) {
    (void)ledger_.complete_reconciliation(
      ticket.goal_generation, ticket.operation_generation,
      ReconciliationKind::kReplayMutation, attempt_number, observed);
    return;
  }
  const bool submission_retired = admission_.confirm_mutation_not_submitted(
    context_->goal_id(), context_->generation(), ticket.operation_generation,
    CoordinatorMutationKind::kReleaseTask) &&
    ledger_.confirm_not_submitted(ticket.goal_generation, ticket.operation_generation) &&
    admission_.resolve_mutation_submission(
    context_->goal_id(), context_->generation(), ticket.operation_generation,
    CoordinatorMutationKind::kReleaseTask);
  if (guard_generic_advance()) {
    return;
  }
  if (!submission_retired) {
    inhibit("rejected release submission could not retire mutation ownership", observed);
    return;
  }
  ++release_submission_rejections_;
  if (release_submission_rejections_ <= config_.task.max_operation_retries &&
    command_deadline_ && observed < *command_deadline_)
  {
    return;
  }
  stop_release_submission_without_proof(
    "terminal release transport rejected before submission after bounded retries: " +
    sent.detail,
    observed, observed);
}

void RestockCoordinatorDriver::request_release_readback(
  const ReconciliationAttempt & attempt)
{
  auto request = std::make_shared<WorldStateCoordinatorPort::GetSnapshot::Request>();
  request->include_removed = false;
  request->include_events = false;
  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  const auto sent = world_state_.get_snapshot(
    {attempt.ticket.goal_generation, attempt.ticket.operation_generation}, request,
    [inbox, gate, goal_id, generation, clock, attempt_number = attempt.attempt_number](
      auto completion) {
      (void)deposit_for_generation(
        gate, goal_id, generation, [&]() {
          return inbox->push_cleanup(
            SnapshotCompletion{
          completion.correlation, std::move(completion.response),
          std::move(completion.transport_error), clock(),
          CoordinatorReconciliationCorrelation{
            ReconciliationKind::kReadback, attempt_number}});
        });
    });
  if (sent) {
    record_request_handle(
      *sent.handle,
      CoordinatorReconciliationCorrelation{
        ReconciliationKind::kReadback, attempt.attempt_number});
    return;
  }
  const auto observed = steady_now_();
  (void)ledger_.complete_reconciliation(
    attempt.ticket.goal_generation, attempt.ticket.operation_generation,
    ReconciliationKind::kReadback, attempt.attempt_number, observed);
}

std::optional<Eigen::Isometry3d> RestockCoordinatorDriver::motion_segment_target(
  MotionSegment segment) const
{
  if (!context_) {
    return std::nullopt;
  }
  // Uses the context's active-candidate accessor, shared with the jaw targets and attachment
  // coupling, so a fall-through to a later candidate stays consistent.
  const auto grasp_pose =
    [this](Eigen::Isometry3d GraspPoseSequence::* member) -> std::optional<Eigen::Isometry3d> {
      const auto * candidate = context_->active_grasp_candidate();
      if (candidate == nullptr) {
        return std::nullopt;
      }
      return candidate->poses.*member;
    };
  switch (segment) {
    case MotionSegment::kPreGrasp:
      return grasp_pose(&GraspPoseSequence::world_from_pregrasp_tool0);
    case MotionSegment::kApproach:
      return grasp_pose(&GraspPoseSequence::world_from_grasp_tool0);
    case MotionSegment::kRetract:
      return grasp_pose(&GraspPoseSequence::world_from_retract_tool0);
    case MotionSegment::kCarryStart:
      {
        const auto * grasp = context_->active_grasp_candidate();
        const auto & placement = context_->placement_candidate();
        if (grasp == nullptr || !placement) {
          return std::nullopt;
        }
        const auto & selection = context_->selection();
        if (!selection) {
          return std::nullopt;
        }
        return build_carry_start_tool0_pose(
          grasp->poses.world_from_retract_tool0,
          placement->poses.world_from_preinsertion_tool0,
          selection->product_envelope);
      }
    case MotionSegment::kPreInsert:
    case MotionSegment::kInsert:
    case MotionSegment::kRetreat:
      {
        if (segment == MotionSegment::kRetreat && retreat_target_) {
          // With a survey provider wired, a failed viewpoint lookup refuses instead of falling
          // back to the placement retreat pose (wrong-place evidence would clear invalidation).
          if (!context_->selection()) {
            return std::nullopt;
          }
          return retreat_target_(*context_->selection());
        }
        const auto & placement = context_->placement_candidate();
        if (!placement) {
          return std::nullopt;
        }
        switch (segment) {
          case MotionSegment::kPreInsert:
            return placement->poses.world_from_preinsertion_tool0;
          case MotionSegment::kInsert:
            return placement->poses.world_from_final_tool0;
          default:
            return placement->poses.world_from_retreat_tool0;
        }
      }
  }
  return std::nullopt;
}

void RestockCoordinatorDriver::attach_grasp_escape(MotionGoal & goal) const
{
  // Milestone 10 §6 (Card 062): only while the jaws closed on a product that is not held and the
  // arm has not left that grasp yet. A held product's retract is its own egress.
  if (!context_ || !context_->grasp_escape() || context_->task_status().object_held) {
    return;
  }
  const auto & record = *context_->grasp_escape();
  goal.grasp_escape = GraspEscape{
    record.world_from_standoff_tool0, record.target_object_id,
    config_.grasp_escape_finger_links};
}

void RestockCoordinatorDriver::request_motion_segment(
  const OperationTicket & ticket, MotionSegment segment)
{
  // A segment never submitted leaves the same stop record as a completion would; nothing was
  // planned, so nothing suggests a replan would fare better.
  const auto refuse_recovery = [this, segment](std::string reason) {
    if (context_) {
      motion_stop_evidence_ = MotionStopEvidence{
        segment, context_->generation(), std::move(reason)};
    }
  };
  // Each refusal below abandons a booked ledger operation; fail_operation does not complete it,
  // so complete the ledger first or it would expire later as a motion deadline.
  const auto abandon = [this, &ticket](RestockTaskEvent event, std::string detail) {
    const auto observed = steady_now_();
    (void)ledger_.complete(ticket.goal_generation, ticket.operation_generation, observed);
    fail_operation(event, std::move(detail), observed);
  };

  const auto target = motion_segment_target(segment);
  if (!motion_ || !target) {
    refuse_recovery("the segment was never submitted to a motion backend");
    abandon(
      RestockTaskEvent::kRetryableFailure,
      std::string(motion_segment_name(segment)) + " motion has no retained target pose");
    return;
  }
  if (perception_liveness_) {
    std::string liveness_detail;
    if (!perception_liveness_(liveness_detail)) {
      refuse_recovery("perception stream is unlive");
      abandon(
        RestockTaskEvent::kTerminalFailure,
        liveness_detail.empty() ?
        "perception stream is older than the configured liveness horizon" :
        liveness_detail);
      return;
    }
  }

  MotionGoal goal;
  goal.planning_frame_from_tool0 = *target;
  goal.planning_time = std::chrono::duration_cast<std::chrono::milliseconds>(
    config_.task.planning_timeout);
  goal.label = motion_segment_name(segment);
  goal.path = linear_motion_segment(segment) ? MotionPathKind::kLinear :
    MotionPathKind::kFreeSpace;
  if (segment == MotionSegment::kPreGrasp) {
    goal.reset_planning_interface_before_plan = true;
    const auto * grasp = context_->active_grasp_candidate();
    const auto & selection = context_->selection();
    if (grasp == nullptr || !selection) {
      refuse_recovery("pre-grasp continuation check has no active grasp candidate");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        "pre-grasp motion has no retained grasp pose for its required approach");
      return;
    }
    goal.required_linear_continuation_pose = grasp->poses.world_from_grasp_tool0;
    goal.required_linear_continuation_gripper_joint_position_m =
      grasp->open_joint_position_m;
    goal.required_linear_continuation_vertical_margin_m =
      config_.pregrasp_continuation_vertical_margin_m;
    Eigen::Isometry3d preinsert_direction_hint = grasp->poses.world_from_retract_tool0;
    preinsert_direction_hint.translation().y() += 1.0;
    const auto carry_start = build_carry_start_tool0_pose(
      grasp->poses.world_from_retract_tool0, preinsert_direction_hint,
      selection->product_envelope);
    if (!carry_start) {
      refuse_recovery("pre-grasp continuation check cannot derive the required tray egress");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        "pre-grasp motion cannot prove its post-grasp tray egress");
      return;
    }
    goal.required_postcontinuation_linear_retract_pose =
      grasp->poses.world_from_retract_tool0;
    goal.required_postcontinuation_linear_egress_pose = *carry_start;
    goal.required_postcontinuation_gripper_joint_position_m =
      grasp->hold_joint_position_m;
    // A continuation-safe pose can have several disconnected IK branches; try a bounded set of
    // endpoints so one difficult branch does not burn the grasp.
    goal.free_space_plan_candidates = 4U;
    attach_grasp_escape(goal);
  }
  if (segment == MotionSegment::kRetreat) {
    const auto & lease = context_->physical_detach_lease_token();
    if (context_->physical_detach_released_at().has_value()) {
      if (!lease || lease->empty()) {
        refuse_recovery("post-detach retreat has no retained planning-scene lease capability");
        abandon(
          RestockTaskEvent::kTerminalFailure,
          "retreat motion requires the planning-scene lease retained by physical detach");
        return;
      }
      // Physical detach freezes the projector while the semantic world still reports the product
      // held. The retreat is the only motion permitted then; MoveIt revalidates this lease before
      // planning and execution.
      goal.planning_scene_lease_token = *lease;
      if (retreat_target_) {
        const auto & placement = context_->placement_candidate();
        if (!placement) {
          refuse_recovery("compound retreat has no retained placement egress pose");
          abandon(
            RestockTaskEvent::kTerminalFailure,
            "retreat motion requires the retained placement egress pose before survey "
            "reposition");
          return;
        }
        // Short linear withdrawal from the lane first, then free-space planning to the survey
        // viewpoint.
        goal.linear_egress_pose = placement->poses.world_from_retreat_tool0;
        goal.path = MotionPathKind::kFreeSpace;
        // The linear egress runs before this plan, so a transient failure cannot be retried as a
        // whole segment; compare a bounded set of plans within the same deadline instead.
        goal.free_space_plan_candidates = 4U;
      }
    } else {
      // Milestone 10 §6 (Card 051): the bounded cleanup retreat runs BEFORE physical detach —
      // no frozen projector, no lane to withdraw from, and no detach-retained lease exists. It
      // is an ordinary free-space motion to the survey viewpoint under the same scene authority
      // as every other pre-detach segment; the target is the wired survey provider (the
      // production node always wires it), never the placement fallback. kRetreat's default
      // path is LINEAR (the post-detach lane withdrawal), which a straight shot across the
      // cell can never satisfy — measured live stopping at 0 % with the collision-off control
      // at 14 % — so this branch plans free-space with the same bounded endpoint search the
      // compound retreat uses.
      if (!retreat_target_) {
        refuse_recovery("the cleanup retreat has no survey viewpoint provider");
        abandon(
          RestockTaskEvent::kRetryableFailure,
          "the pre-detach cleanup retreat requires a survey viewpoint target");
        return;
      }
      goal.path = MotionPathKind::kFreeSpace;
      goal.free_space_plan_candidates = 4U;
      attach_grasp_escape(goal);
    }
  }
  if (segment == MotionSegment::kRetract && context_) {
    // Card 062: the grip verified, so the retract is the product's egress; no escape is owed.
    context_->clear_grasp_escape();
  }
  if (segment == MotionSegment::kRetract || segment == MotionSegment::kCarryStart ||
    segment == MotionSegment::kPreInsert || segment == MotionSegment::kInsert)
  {
    // Side grasps make tool0 +X the held cylinder's upright axis from grasp closure through
    // insertion. PreInsert also gets a planner orientation constraint below; every trajectory is
    // checked before execution.
    goal.maximum_tool0_x_axis_tilt_rad = kHeldProductMaximumUprightTiltRad;
  }
  if (segment == MotionSegment::kInsert) {
    const auto & placement = context_->placement_candidate();
    const auto & grasps = context_->grasp_candidate_batch();
    if (!placement || !grasps) {
      refuse_recovery("post-release egress proof has no placement or grasp authority");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        "insert motion cannot prove post-release egress without retained geometry");
      return;
    }
    goal.required_postmotion_linear_egress_pose =
      placement->poses.world_from_retreat_tool0;
    goal.required_postmotion_linear_egress_gripper_joint_position_m =
      grasps->config.maximum_open_joint_position_m;
  }
  if (segment == MotionSegment::kPreInsert) {
    // Survey-derived AABB on the long egress-to-mouth free-space leg. Carry-start is a
    // collision-checked Cartesian Y egress; PreInsert owns the reorientation and rail travel.
    const auto * grasp = context_->active_grasp_candidate();
    const auto & selection = context_->selection();
    const auto & placement = context_->placement_candidate();
    if (grasp == nullptr || !selection || !placement) {
      refuse_recovery(
        "transfer path constraints need a retained grasp, selection, and placement");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        std::string(motion_segment_name(segment)) +
        " motion has no grasp, selection, or placement for transfer path constraints");
      return;
    }
    // A pose-only IK endpoint at the lane mouth can be reached on a branch that collides with
    // the roller bed during the straight insert; prove the continuation when choosing it.
    goal.required_linear_continuation_pose = placement->poses.world_from_final_tool0;
    goal.required_linear_continuation_gripper_joint_position_m =
      grasp->hold_joint_position_m;
    const auto carry_start = build_carry_start_tool0_pose(
      grasp->poses.world_from_retract_tool0,
      placement->poses.world_from_preinsertion_tool0,
      selection->product_envelope);
    if (!carry_start) {
      refuse_recovery(
        "carry-start pose could not be derived from surveyed retract/pre-insert (degenerate Y "
        "without aisle egress, or unusable held envelope)");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        "carry-start tool0 pose could not be derived from retract, pre-insert, and held-product "
        "geometry");
      return;
    }
    auto constraints = build_preinsert_transfer_path_constraints(
      *carry_start, *target, selection->product_envelope, kPlannerRobotPaddingM,
      kSurveyedLaneSideClearanceM, goal.position_tolerance_m,
      *goal.maximum_tool0_x_axis_tilt_rad);
    if (!constraints) {
      refuse_recovery("transfer envelope could not be derived from surveyed geometry");
      abandon(
        RestockTaskEvent::kRetryableFailure,
        std::string(motion_segment_name(segment)) +
        " transfer path constraints could not be derived from endpoints and held-product geometry");
      return;
    }
    constraints->planning_frame = config_.planning_frame;
    goal.path_constraints = std::move(*constraints);
    // The seven-DOF goal has several valid rail/arm branches; compare a bounded set so the first
    // sweeping solution is not executed (duration derives from the joint limits).
    goal.free_space_plan_candidates = 4U;
  }

  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  note_command_submitted();
  const auto sent = motion_->submit(
    {ticket.goal_generation, ticket.operation_generation}, goal,
    [inbox, gate, goal_id, generation, clock, diagnostic = async_deposit_diagnostic_](
      MotionCompletion completion) {
      deposit_and_report(
        diagnostic, "motion", gate, goal_id, generation, [&]() {
          return inbox->push(CoordinatorMotionCompletion{std::move(completion), clock()});
        });
    });
  if (sent) {
    return;
  }

  const auto observed = steady_now_();
  const auto completion = ledger_.complete(
    ticket.goal_generation, ticket.operation_generation, observed);
  if (guard_generic_advance()) {
    return;
  }
  // Only a refusal that proves nothing was handed to the port undoes the command: a malformed
  // goal, or a port that cannot take work at all (stopping). kBusy means the port still owns an
  // earlier goal, so the arm may be moving and `commanded` stays (review S1).
  if (sent.status == MotionSubmitStatus::kInvalidRequest ||
    sent.status == MotionSubmitStatus::kUnavailable)
  {
    note_motion_submission_undone();
  }
  refuse_recovery("the motion backend refused the submission, which replanning does not clear");
  fail_operation(
    completion.disposition == OperationCompletionDisposition::kDeliverTimeout ?
    RestockTaskEvent::kTimeout : RestockTaskEvent::kRetryableFailure,
    std::string(motion_segment_name(segment)) + " motion submission was refused: " + sent.detail,
    observed);
}

void RestockCoordinatorDriver::process_completion(
  const CoordinatorMotionCompletion & delivery, SteadyTime now)
{
  const auto & completion = delivery.completion;
  // A motion result is a definite outcome.
  const auto decision = ledger_.complete(
    completion.correlation.goal_generation, completion.correlation.operation_generation,
    delivery.arrived_at);
  if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
    inhibit("motion completion has unknown operation correlation", now);
    return;
  }
  if (decision.disposition != OperationCompletionDisposition::kDeliver || !decision.ticket) {
    return;
  }
  const auto segment = planned_motion_segment(decision.ticket->command);
  if (!segment) {
    inhibit("motion completion does not match the pending command", now);
    return;
  }

  const std::string detail =
    std::string(motion_segment_name(*segment)) + " motion " +
    motion_outcome_name(completion.outcome) +
    (completion.detail.empty() ? std::string{} : ": " + completion.detail);

  // Records or clears the stop evidence so an earlier segment's cannot authorise a later recovery.
  observe_motion_stop(*segment, completion);
  // Card 086 stage 1b. A completion that proves the backend started nothing (a plan refusal,
  // an unavailable backend) undoes this submission, unless a grasp escape leg already ran; a
  // terminal stop is recorded only from arrival or a failure carrying the backend's stop flag.
  // The port's submitted_to_backend bit decides: a kUnavailable (or any other not-started class)
  // after execute() was called, at any slice, proves nothing (review B1).
  const bool proven_not_started = motion_completion_definitely_not_started(completion);
  if (proven_not_started) {
    note_motion_submission_undone();
  }
  if (!proven_not_started &&
    (completion.outcome == MotionOutcome::kSucceeded || completion.execution_reached_terminal_stop))
  {
    commanded_.stop_observed = true;
  }
  if (completion.grasp_escape_executed && context_ && context_->grasp_escape()) {
    report_task_receipt(
      "grasp escape: the arm left the grasp of " + context_->grasp_escape()->target_object_id +
      " along the reversed approach; it is not commanded again", now);
    context_->clear_grasp_escape();
  }

  switch (completion.outcome) {
    case MotionOutcome::kSucceeded:
      executed_segment_ = *segment;
      dispatch(RestockTaskEvent::kOperationSucceeded, detail, delivery.arrived_at);
      return;
    case MotionOutcome::kPlanningFailed:
      // A pre-grasp or Cartesian approach the planner refuses is a verdict on this grasp
      // candidate (the arm has not moved), so fall through to the next candidate instead of
      // re-asking the same pose. Only the planner's own refusal counts: a planning-scene
      // authority failure, unavailable backend, deadline or execution failure must not consume
      // candidates, or a stalled obstacle stream would burn them all.
      if (context_ &&
        (*segment == MotionSegment::kPreGrasp || *segment == MotionSegment::kApproach) &&
        completion.planner_refused_the_goal)
      {
        const auto fallthrough = context_->refuse_active_grasp_candidate(detail);
        switch (fallthrough.disposition) {
          case GraspFallthroughDisposition::kNextCandidateActive:
            // Milestone 10 §6 rung 4 (Card 051): the pose-specific verdict spent one candidate;
            // the next one is planned from the re-observed state, budget unchanged.
            report_task_receipt(
              "recovery rung 4 (alternative candidate): " + fallthrough.detail, now);
            // Pre-grasp will submit the next candidate. Approach stands at the old candidate's
            // pre-grasp, so skip retries and enter recovery, which replans the new pre-grasp.
            fail_operation(
              *segment == MotionSegment::kApproach ? RestockTaskEvent::kTerminalFailure :
              RestockTaskEvent::kRetryableFailure,
              fallthrough.detail, delivery.arrived_at);
            return;
          case GraspFallthroughDisposition::kCandidatesExhausted:
            // Milestone 10 §6 rung 4 (Card 051): the batch is spent — this is the budget whose
            // exhaustion hands the goal to rung 5 (the recoverable skip) or, UNSAFE, to the
            // operator. End the attempt with what refused each candidate.
            report_task_receipt(
              "recovery rung 4 (alternative candidate): the candidate batch is exhausted — " +
              fallthrough.detail, now);
            fail_operation(
              RestockTaskEvent::kTerminalFailure, fallthrough.detail, delivery.arrived_at);
            return;
          case GraspFallthroughDisposition::kNotSelecting:
            break;
        }
      }
      // Classified here, at the planning-failure site, and only for the free-space OMPL legs
      // whose slice exhaustion is the measured retriable class: downstream dispatches never
      // infer the cause from the command or the text. Pre-grasp/approach (never refused) and
      // every non-planning outcome below keep raw evidence.
      fail_operation(
        RestockTaskEvent::kRetryableFailure, detail, delivery.arrived_at,
        (*segment == MotionSegment::kPreInsert || *segment == MotionSegment::kCarryStart) ?
        FeedbackDetailPolicy::kRetriablePlanningDiagnostic :
        FeedbackDetailPolicy::kRawEvidence);
      return;
    case MotionOutcome::kUnavailable:
      if (completion.submitted_to_backend) {
        // An execute-phase backend loss (review 3 S1, decided fail-closed): the trajectory was
        // already handed to the backend, so the arm may still be executing it. Same class as
        // kExecutionFailed: no blind in-goal retry by a fresh submission, and observe_motion_stop
        // above has recorded the stop as not established, so recovery is refused.
        fail_operation(
          RestockTaskEvent::kTerminalFailure,
          detail + " (the trajectory had already been handed to the backend)",
          delivery.arrived_at);
        return;
      }
      // Nothing was submitted, so an ordinary retry is safe. Raw evidence: a backend that is
      // down is not an OMPL planning miss and must never be redacted.
      fail_operation(RestockTaskEvent::kRetryableFailure, detail, delivery.arrived_at);
      return;
    case MotionOutcome::kTimedOut:
      fail_operation(RestockTaskEvent::kTimeout, detail, delivery.arrived_at);
      return;
    case MotionOutcome::kExecutionFailed:
    case MotionOutcome::kCanceled:
      // The arm may have moved; no blind retry. The recovery command decides on a replan from
      // the stop evidence recorded above.
      fail_operation(RestockTaskEvent::kTerminalFailure, detail, delivery.arrived_at);
      return;
  }
  inhibit("motion completion carried an unknown outcome", now);
}

void RestockCoordinatorDriver::request_gripper(
  const OperationTicket & ticket, GripperAperture aperture)
{
  const auto & grasp = context_ ? context_->grasp_candidate_batch() :
    std::optional<GraspCandidateBatch>{};
  const auto * candidate = context_ ? context_->active_grasp_candidate() : nullptr;
  // Card 062: the escape opens to the retained record of the candidate that closed, which may
  // already have been rejected, so it needs neither the batch nor an active candidate.
  const bool escape = aperture == GripperAperture::kEscapeClearance;
  const auto & escape_record = context_ ? context_->grasp_escape() :
    std::optional<GraspEscapeRecord>{};
  const bool hold = aperture == GripperAperture::kHold;
  if (!gripper_ || (escape ? !escape_record : (!grasp || candidate == nullptr)) ||
    (hold && !context_->selection()))
  {
    // Complete the booked operation or it would expire at its own deadline later.
    const auto observed = steady_now_();
    (void)ledger_.complete(ticket.goal_generation, ticket.operation_generation, observed);
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      escape ? "the grasp escape open has no retained record of the candidate that closed" :
      (hold && candidate != nullptr && grasp ?
      "closing the jaws requires the retained selection the grasp escape would leave" :
      "gripper command requires a retained grasp candidate"), observed);
    return;
  }

  // The candidate carries the hold and approach-clearance widths; the attachment plugin couples
  // only within a fraction of a millimetre of the hold width and decouples only at the configured
  // full-open target, so release uses that target, not the approach clearance.
  GripperGoal goal;
  switch (aperture) {
    case GripperAperture::kHold:
      goal.target_position_m = candidate->hold_joint_position_m;
      break;
    case GripperAperture::kEscapeClearance:
      goal.target_position_m = escape_record->open_joint_position_m;
      report_task_receipt(
        "grasp escape: opening the jaws to the closed candidate's approach clearance " +
        std::to_string(escape_record->open_joint_position_m) + " m before leaving the grasp of " +
        escape_record->target_object_id + " (not held)", steady_now_());
      break;
    case GripperAperture::kApproachClearance:
      goal.target_position_m = candidate->open_joint_position_m;
      break;
    case GripperAperture::kRelease:
      goal.target_position_m = grasp->config.maximum_open_joint_position_m;
      break;
  }

  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  note_command_submitted();
  const auto sent = gripper_->submit(
    {ticket.goal_generation, ticket.operation_generation}, goal,
    [inbox, gate, goal_id, generation, clock,
    diagnostic = async_deposit_diagnostic_](GripperCompletion completion) {
      deposit_and_report(
        diagnostic, "gripper", gate, goal_id, generation, [&]() {
          return inbox->push(CoordinatorGripperCompletion{std::move(completion), clock()});
        });
    });
  if (sent) {
    if (hold) {
      // Card 062: the hold reached the gripper, so the jaws may now close on the product; retain
      // how to leave it again. Retained only here, so no record means no hold was ever sent.
      context_->retain_grasp_escape(
        GraspEscapeRecord{
          candidate->poses.world_from_pregrasp_tool0, candidate->open_joint_position_m,
          "restocker/object/" + std::to_string(context_->selection()->object_id.value)});
    }
    return;
  }

  const auto observed = steady_now_();
  const auto completion = ledger_.complete(
    ticket.goal_generation, ticket.operation_generation, observed);
  if (guard_generic_advance()) {
    return;
  }
  fail_operation(
    completion.disposition == OperationCompletionDisposition::kDeliverTimeout ?
    RestockTaskEvent::kTimeout : RestockTaskEvent::kRetryableFailure,
    "gripper submission was refused: " + sent.detail, observed);
}

void RestockCoordinatorDriver::process_completion(
  const CoordinatorGripperCompletion & delivery, SteadyTime now)
{
  const auto & completion = delivery.completion;
  const auto decision = ledger_.complete(
    completion.correlation.goal_generation, completion.correlation.operation_generation,
    delivery.arrived_at);
  if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
    inhibit("gripper completion has unknown operation correlation", now);
    return;
  }
  if (decision.disposition != OperationCompletionDisposition::kDeliver || !decision.ticket) {
    return;
  }
  if (decision.ticket->command != RestockTaskCommand::kCloseGripper &&
    decision.ticket->command != RestockTaskCommand::kOpenGripper)
  {
    inhibit("gripper completion does not match the pending command", now);
    return;
  }

  const std::string detail =
    std::string("gripper ") + gripper_outcome_name(completion.outcome) +
    (completion.detail.empty() ? std::string{} : ": " + completion.detail);

  switch (completion.outcome) {
    case GripperOutcome::kSucceeded:
      dispatch(RestockTaskEvent::kOperationSucceeded, detail, delivery.arrived_at);
      return;
    case GripperOutcome::kRejected:
    case GripperOutcome::kUnavailable:
      // No jaw command reached the controller, so retrying changes nothing physical.
      fail_operation(RestockTaskEvent::kRetryableFailure, detail, delivery.arrived_at);
      return;
    case GripperOutcome::kTimedOut:
      fail_operation(RestockTaskEvent::kTimeout, detail, delivery.arrived_at);
      return;
    case GripperOutcome::kPositionNotVerified:
    case GripperOutcome::kExecutionFailed:
    case GripperOutcome::kCanceled:
      // The jaws moved without reaching the commanded width; closing again on a possibly
      // half-gripped product is not provably safe.
      fail_operation(RestockTaskEvent::kTerminalFailure, detail, delivery.arrived_at);
      return;
  }
  inhibit("gripper completion carried an unknown outcome", now);
}

void RestockCoordinatorDriver::request_attachment(
  const OperationTicket & ticket, bool attaching, bool semantic_only)
{
  const auto & grasp = context_ ? context_->grasp_candidate_batch() :
    std::optional<GraspCandidateBatch>{};
  const auto & reservation = context_ ? context_->reservation() :
    std::optional<TaskReservationCapability>{};
  const auto & selection = context_ ? context_->selection() : std::optional<SelectedTaskPair>{};
  const auto & snapshot = context_ ? context_->latest_snapshot() :
    std::optional<restocker_world_state::WorldStateSnapshot>{};
  // Attaching derives the coupling from the product's observed pose; detaching needs none.
  const bool product_observed = !attaching ||
    (snapshot && selection && snapshot->objects.contains(selection->object_id));
  const auto * candidate = context_ ? context_->active_grasp_candidate() : nullptr;
  if (!attachment_ || !grasp || candidate == nullptr || !reservation || !selection ||
    !product_observed)
  {
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      "attachment transaction requires a reserved, observed product and a retained grasp",
      steady_now_());
    return;
  }

  AttachmentGoal goal;
  goal.direction =
    attaching ? AttachmentDirection::kAttach : AttachmentDirection::kDetach;
  goal.scope = attaching ? AttachmentScope::kFull :
    (semantic_only ? AttachmentScope::kSemanticOnly : AttachmentScope::kPhysicalOnly);
  goal.object_id = selection->object_id.value;
  goal.reservation_token = reservation->token;
  if (attaching) {
    // The boundary rejects the attach if the observed grasp_center -> product body differs from
    // this transform, so the child is the product, not the tool frame.
    const auto & poses = candidate->poses;
    const auto & object = snapshot->objects.at(selection->object_id);
    goal.grasp_center_to_child = poses.world_from_grasp_center.inverse() * object.pose_in_world;
    // Pose covariance lets the boundary scale its grasp-residual tolerance. It is expressed in
    // the world frame while the residual is in the grasp frame, but the boundary reads only the
    // largest translational eigenvalue, which rotation preserves. Zero (no covariance) leaves the
    // boundary on its configured constant.
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t column = 0; column < 6; ++column) {
        goal.expected_pose_covariance[row * 6 + column] = object.pose_covariance(
          static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column));
      }
    }
  } else if (semantic_only) {
    const auto & released_at = context_->physical_detach_released_at();
    const auto & lease = context_->physical_detach_lease_token();
    if (!released_at || !lease) {
      fail_operation(
        RestockTaskEvent::kTerminalFailure,
        "semantic detach requires a retained physical detach proof",
        steady_now_());
      return;
    }
    goal.released_at = *released_at;
    goal.planning_scene_lease_token = *lease;
  }

  const auto inbox = inbox_;
  const auto gate = context_->generation_quiescence();
  const auto goal_id = context_->goal_id();
  const auto generation = context_->generation();
  const auto clock = async_evidence_now_;
  note_command_submitted();
  const auto sent = attachment_->submit(
    {ticket.goal_generation, ticket.operation_generation}, goal,
    [inbox, gate, goal_id, generation, clock,
    diagnostic = async_deposit_diagnostic_](AttachmentCompletion completion) {
      deposit_and_report(
        diagnostic, "attachment", gate, goal_id, generation, [&]() {
          return inbox->push(CoordinatorAttachmentCompletion{std::move(completion), clock()});
        });
    });
  if (sent) {
    return;
  }

  const auto observed = steady_now_();
  const auto completion = ledger_.complete(
    ticket.goal_generation, ticket.operation_generation, observed);
  if (guard_generic_advance()) {
    return;
  }
  fail_operation(
    completion.disposition == OperationCompletionDisposition::kDeliverTimeout ?
    RestockTaskEvent::kTimeout : RestockTaskEvent::kRetryableFailure,
    "attachment submission was refused: " + sent.detail, observed);
}

void RestockCoordinatorDriver::process_completion(
  const CoordinatorAttachmentCompletion & delivery, SteadyTime now)
{
  const auto & completion = delivery.completion;
  const auto decision = ledger_.complete(
    completion.correlation.goal_generation, completion.correlation.operation_generation,
    delivery.arrived_at);
  if (decision.disposition == OperationCompletionDisposition::kRejectUnknown) {
    inhibit("attachment completion has unknown operation correlation", now);
    return;
  }
  if (decision.disposition != OperationCompletionDisposition::kDeliver || !decision.ticket) {
    return;
  }
  if (decision.ticket->command != RestockTaskCommand::kRunAttachTransaction &&
    decision.ticket->command != RestockTaskCommand::kRunDetachTransaction &&
    decision.ticket->command != RestockTaskCommand::kRunCommitDetachment)
  {
    inhibit("attachment completion does not match the pending command", now);
    return;
  }

  const std::string detail =
    std::string("attachment ") + attachment_outcome_name(completion.outcome) +
    (completion.detail.empty() ? std::string{} : ": " + completion.detail);

  const bool physical_only_detach =
    decision.ticket->command == RestockTaskCommand::kRunDetachTransaction;

  // Branch on what the outcome proves about the world, not on which step reported it.
  switch (attachment_world_effect(completion.outcome)) {
    case AttachmentWorldEffect::kFullyApplied:
      if (physical_only_detach) {
        // A physical-only success reports kFullyApplied by the port's scope contract; the lease
        // is retained for the semantic half.
        if (const auto retained = context_->retain_physical_detach_proof(
            completion.released_at, completion.retained_lease_token); !retained)
        {
          inhibit("physical detach proof could not be retained: " + retained.detail, now);
          return;
        }
        if (invalidate_lane_evidence_) {
          const auto & selection = context_->selection();
          const auto & snapshot = context_->latest_snapshot();
          if (selection && snapshot) {
            const auto lane = snapshot->lanes.find(selection->lane_id);
            if (lane != snapshot->lanes.end()) {
              invalidate_lane_evidence_(
                selection->lane_id.value, lane->second.revision);
            }
          }
        }
      }
      dispatch(RestockTaskEvent::kOperationSucceeded, detail, delivery.arrived_at);
      return;
    case AttachmentWorldEffect::kNoneApplied:
      // Simulator and world state both untouched: roll back.
      if (context_) {
        if (decision.ticket->command == RestockTaskCommand::kRunAttachTransaction) {
          (void)context_->refuse_active_grasp_candidate(detail);
        }
        transaction_rollback_recovery_evidence_ = TransactionRollbackRecoveryEvidence{
          context_->generation(),
          decision.ticket->command == RestockTaskCommand::kRunDetachTransaction};
      }
      dispatch(RestockTaskEvent::kTransactionRolledBack, detail, delivery.arrived_at);
      return;
    case AttachmentWorldEffect::kPhysicalOnly:
    case AttachmentWorldEffect::kIndeterminate:
      // Simulator and world state disagree or cannot be proven to agree; an operator takes over.
      dispatch(RestockTaskEvent::kTransactionInhibited, detail, delivery.arrived_at);
      return;
  }
  inhibit("attachment completion carried an unknown outcome", now);
}

void RestockCoordinatorDriver::process_completion(
  const CoordinatorLaneAcquireCompletion & completion, SteadyTime now)
{
  const auto ticket = ledger_.pending();
  if (!context_ || !ticket ||
    ticket->goal_generation != completion.correlation.goal_generation ||
    ticket->operation_generation != completion.correlation.operation_generation ||
    ticket->command != RestockTaskCommand::kSurveyDestination)
  {
    return;
  }
  if (!completion.published) {
    fail_operation(
      RestockTaskEvent::kRetryableFailure,
      completion.detail.empty() ?
      "destination lane observation could not be acquired at the retreat viewpoint" :
      completion.detail,
      now);
    return;
  }
  request_snapshot(*ticket);
}

std::string RestockCoordinatorDriver::verify_observed_evidence(RestockTaskCommand command) const
{
  const auto & snapshot = context_->latest_snapshot();
  const auto & selection = context_->selection();
  if (!snapshot || !selection) {
    return "verification requires a fresh snapshot and a retained selection";
  }
  const auto object = snapshot->objects.find(selection->object_id);
  if (object == snapshot->objects.end()) {
    return "the reserved product is absent from the fresh snapshot";
  }
  const auto lane = snapshot->lanes.find(selection->lane_id);
  if (lane == snapshot->lanes.end()) {
    return "the destination lane is absent from the fresh snapshot";
  }

  switch (command) {
    case RestockTaskCommand::kObserveDestination:
      if (lane->second.obstructed) {
        return "the destination lane became obstructed while the product was in transit";
      }
      return {};
    case RestockTaskCommand::kSurveyDestination:
      {
        const auto & released_at = context_->physical_detach_released_at();
        if (!released_at) {
          return "destination survey requires a retained physical detach release stamp";
        }
        if (lane->second.evidence_invalidated) {
          return "destination survey did not clear the post-placement evidence invalidation";
        }
        if (lane->second.last_verified.nanoseconds() <=
          static_cast<std::int64_t>(released_at->sec) * 1'000'000'000LL +
          static_cast<std::int64_t>(released_at->nanosec))
        {
          return "destination survey must produce evidence strictly after the physical release";
        }
        return {};
      }
    case RestockTaskCommand::kVerifyGrasp:
      // robot.held_object is written by the attachment commit, the next state, so it cannot be
      // required here; this only proves the attachment is still admissible (jaw closure was
      // proven by the gripper port, coupling is re-checked by the simulator).
      if (snapshot->robot.held_object) {
        return *snapshot->robot.held_object == selection->object_id ?
               "the reserved product is already attached before the grasp was committed" :
               "the robot already reports holding a different product";
      }
      if (object->second.grasp_state != restocker_world_state::GraspState::Free) {
        return "the reserved product is no longer free to be grasped";
      }
      if (object->second.tracking_state != restocker_world_state::TrackingState::Tracked) {
        return "the reserved product is no longer tracked";
      }
      return {};
    case RestockTaskCommand::kExecuteRecovery:
      {
        // The arm must be at rest before planning from its current state.
        if (snapshot->robot.fault_state != restocker_world_state::FaultState::None) {
          return "the robot reports a fault, so recovery cannot claim a safe state";
        }
        // Rest is not re-checked here: settle_recovery_stop is the single rest gate (Card 060),
        // evaluated once on one clock sample immediately before this, so a sample at the age
        // bound can never pass the hold and then be refused without it.
        // A rolled-back attachment transaction changes no world state; retry only after this
        // snapshot confirms rest and the held-object state the rollback promised.
        const bool holds_reserved_product = snapshot->robot.held_object &&
          *snapshot->robot.held_object == selection->object_id;
        // Between physical detach and the semantic commit the snapshot still names the reserved
        // product as held; that is expected, not an unplanned grasp.
        const bool semantic_detach_pending =
          context_->physical_detach_released_at().has_value() &&
          !context_->task_status().object_held;
        if (transaction_rollback_recovery_evidence_) {
          if (transaction_rollback_recovery_evidence_->goal_generation !=
            context_->generation())
          {
            return "attachment rollback evidence belongs to an earlier goal generation";
          }
          if (transaction_rollback_recovery_evidence_->object_held &&
            !holds_reserved_product)
          {
            return "the rolled-back detach did not leave the reserved product held";
          }
          if (!transaction_rollback_recovery_evidence_->object_held &&
            snapshot->robot.held_object)
          {
            return "the rolled-back attach unexpectedly left the robot holding a product";
          }
          return {};
        }
        // A replan is safe only while the task and robot agree about what is held.
        if (!motion_stop_evidence_) {
          return "recovery has no record of which segment it is resuming";
        }
        if (carries_product(motion_stop_evidence_->segment) && !holds_reserved_product) {
          return "the robot no longer reports holding the reserved product, so the attachment "
                 "state is uncertain and recovery must not replan a carry";
        }
        if (!carries_product(motion_stop_evidence_->segment) && snapshot->robot.held_object &&
          !(semantic_detach_pending && holds_reserved_product))
        {
          return "the robot reports holding a product before the grasp was committed, so "
                 "recovery cannot claim an empty gripper";
        }
        return {};
      }
    case RestockTaskCommand::kVerifyPlacement:
    case RestockTaskCommand::kUpdateInventory:
      if (snapshot->robot.held_object) {
        return "the robot still reports a held product after placement";
      }
      if (std::ranges::find(lane->second.contents, selection->object_id) ==
        lane->second.contents.end())
      {
        return "the destination lane does not contain the placed product";
      }
      return {};
    default:
      return {};
  }
}

void RestockCoordinatorDriver::observe_motion_stop(
  MotionSegment segment, const MotionCompletion & completion)
{
  motion_stop_evidence_.reset();
  if (!context_) {
    return;
  }
  MotionStopEvidence evidence{segment, context_->generation(), {}};
  switch (completion.outcome) {
    case MotionOutcome::kExecutionFailed:
      if (!completion.execution_reached_terminal_stop) {
        evidence.refusal =
          "the motion backend never established that the trajectory reached a terminal state at "
          "the controllers, so the arm may still be under command";
      } else if (linear_motion_segment(segment)) {
        // Linear segments have millimetre clearance; a fresh straight line from a stop inside
        // one can start inside the product.
        evidence.refusal =
          std::string("the ") + motion_segment_name(segment) +
          " segment slides the jaws or the held product along a straight line with millimetres of "
          "clearance, so a stop inside it is not a state a replan may start from";
      }
      break;
    case MotionOutcome::kCanceled:
      evidence.refusal = "the segment was canceled, so a stop is being commanded rather than "
        "recovered from";
      break;
    case MotionOutcome::kTimedOut:
      evidence.refusal = "the segment passed its deadline without a terminal result";
      break;
    case MotionOutcome::kPlanningFailed:
      // Nothing was commanded, so the arm is where the last verified segment left it. A linear
      // segment that could not be planned goes back to the free-space traverse that chose its
      // configuration, which may choose differently; the recovery budget bounds the search.
      break;
    case MotionOutcome::kUnavailable:
      // A replan cannot bring the backend back; fail closed.
      evidence.refusal = "the motion backend was unavailable, which replanning does not clear";
      break;
    case MotionOutcome::kSucceeded:
      // Successful execution is also terminal-stop evidence; retain it for read-only refreshes.
      break;
  }
  motion_stop_evidence_ = std::move(evidence);
  if (completion.outcome == MotionOutcome::kSucceeded) {
    return;
  }
  // Ask now, decide later: a slow backend answers while the task machine transitions, and an
  // answer not yet arrived at decision time is ignored.
  ask_recovery_advisor(segment, completion);
}

bool RestockCoordinatorDriver::settle_recovery_stop(SteadyTime now)
{
  // A faulted robot is refused for its fault by verify_observed_evidence, never held for motion.
  if (context_->latest_snapshot()->robot.fault_state != restocker_world_state::FaultState::None) {
    recovery_stop_settle_.reset();
    return true;
  }
  const auto verdict = evaluate_recovery_rest(
    context_->latest_snapshot()->robot, ros_now_(), config_.maximum_robot_age,
    config_.maximum_future_skew, config_.recovery_rest_speed);
  const auto transition = context_->task_status();
  const auto budget = config_.recovery_stop_settle_timeout;
  const bool for_this_attempt = recovery_stop_settle_ &&
    recovery_stop_settle_->goal_generation == context_->generation() &&
    recovery_stop_settle_->recovery_attempt == transition.recovery_attempt;
  if (verdict.established) {
    if (for_this_attempt) {
      const auto waited = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - recovery_stop_settle_->started);
      report_task_receipt(
        "recovery stop established after " + std::to_string(waited.count()) + " ms of the " +
        std::to_string(budget.count()) + " ms stop-settle hold: " + verdict.detail, now);
    }
    recovery_stop_settle_.reset();
    return true;
  }
  if (budget.count() > 0 && !for_this_attempt) {
    recovery_stop_settle_ = RecoveryStopSettle{
      now, bounded_deadline(now, budget),
      bounded_deadline(now, config_.recovery_stop_settle_poll), context_->generation(),
      transition.recovery_attempt};
    report_task_receipt(
      "recovery stop not yet established; bounded stop-settle hold started (" +
      std::to_string(budget.count()) + " ms): " + verdict.detail, now);
    return false;
  }
  if (budget.count() > 0 && now < recovery_stop_settle_->deadline) {
    recovery_stop_settle_->next_poll = bounded_deadline(now, config_.recovery_stop_settle_poll);
    return false;
  }
  std::string refusal = verdict.detail;
  if (budget.count() > 0) {
    report_task_receipt(
      "recovery stop-settle hold expired after " + std::to_string(budget.count()) + " ms: " +
      verdict.detail, now);
    refusal += "; the bounded stop-settle hold of " + std::to_string(budget.count()) +
      " ms did not establish a stop";
  }
  recovery_stop_settle_.reset();
  fail_operation(RestockTaskEvent::kRetryableFailure, std::move(refusal), now);
  return false;
}

std::string RestockCoordinatorDriver::refuse_motion_recovery() const
{
  if (!context_) {
    return "recovery has no active goal context";
  }
  if (transaction_rollback_recovery_evidence_) {
    return transaction_rollback_recovery_evidence_->goal_generation == context_->generation() ?
           std::string{} :
           "recovery is refused: attachment rollback evidence belongs to an earlier goal "
           "generation";
  }
  if (!motion_stop_evidence_) {
    return "recovery is refused: no terminal motion established a stopped state for this task";
  }
  if (motion_stop_evidence_->goal_generation != context_->generation()) {
    return "recovery is refused: the stopped state on record belongs to an earlier goal "
           "generation";
  }
  if (!motion_stop_evidence_->refusal.empty()) {
    return "recovery is refused: " + motion_stop_evidence_->refusal;
  }
  return {};
}

RecoveryClassification RestockCoordinatorDriver::classify_current(const std::string & cause) const
{
  RecoveryContext context;
  context.cause = cause;
  if (!context_) {
    // No goal context: nothing can be established, so nothing may be retried.
    return classify_recovery(context);
  }
  const auto transition = context_->task_status();
  // Arm: a goal that never commanded a trajectory is stopped by definition; otherwise a
  // first-hand stop record for this goal generation with no refusal on it is the verified stop.
  const bool stop_recorded = motion_stop_evidence_ &&
    motion_stop_evidence_->goal_generation == context_->generation() &&
    motion_stop_evidence_->refusal.empty();
  if (!transition.first_trajectory_may_have_started ||
    (stop_recorded && !transition.current_execution_may_have_started))
  {
    context.arm_stop = RecoveryEvidence::kEstablished;
  } else {
    context.arm_stop = RecoveryEvidence::kNotEstablished;
  }
  // Held object: the machine's tracking is proof-based (set only by accepted attach/detach
  // receipts and full rollbacks), so it is Established while the fault says the reconciliation
  // held. An external inconsistency means the safe boundary or the attachment reconciliation
  // could not be proven — exactly the spec's "held object unknown" unsafe trigger — and a
  // failed deterministic recovery means the stopped state was never re-established.
  switch (transition.fault) {
    case RestockTaskFault::kExternalInconsistency:
      context.held_state = RecoveryEvidence::kUnknown;
      break;
    case RestockTaskFault::kRecoveryFailed:
      context.arm_stop = RecoveryEvidence::kUnknown;
      break;
    default:
      context.held_state = RecoveryEvidence::kEstablished;
      break;
  }
  return classify_recovery(context);
}

RecoveryAdvicePermissions RestockCoordinatorDriver::recovery_permissions_now() const
{
  RecoveryAdvicePermissions permitted;
  // Refusing to act is never less safe than acting.
  permitted.abandon_task = true;
  // A bounded replan is available when the deterministic authorisation says so. A plain retry is
  // not: the operation retries are already spent by the time recovery is reached.
  permitted.resume_at_recovery_state = refuse_motion_recovery().empty();
  return permitted;
}

void RestockCoordinatorDriver::forget_recovery_question()
{
  const bool was_asking = recovery_question_.has_value();
  recovery_question_.reset();
  if (recovery_advice_mailbox_) {
    recovery_advice_mailbox_->clear();
  }
  // The backend still completes; the answer lands in a mailbox that has forgotten the question.
  if (was_asking && recovery_advisor_ != nullptr) {
    recovery_advisor_->cancel();
  }
}

void RestockCoordinatorDriver::ask_recovery_advisor(
  MotionSegment segment, const MotionCompletion & completion)
{
  if (!recovery_advisor_ || !context_ || !recovery_advice_mailbox_) {
    return;
  }
  // The mailbox holds one slot; drop the old question first.
  forget_recovery_question();

  const auto permitted = recovery_permissions_now();
  // Skip the question when stopping is the only permitted primitive: no answer could differ.
  if (permitted.permitted().size() < 2U) {
    return;
  }
  restocker_reasoner::RecoveryQuery query;
  // Correlation: goal generation, failed operation and segment.
  query.request_id = std::to_string(context_->generation()) + "-" +
    std::to_string(completion.correlation.operation_generation) + "-" +
    motion_segment_name(segment);
  query.goal_generation = context_->generation();
  query.failed_segment = motion_segment_name(segment);
  query.motion_outcome = motion_outcome_name(completion.outcome);
  query.task_state = to_string(context_->task_status().state);
  query.observed_detail = completion.detail;
  query.deterministic_refusal = refuse_motion_recovery();
  query.deterministic_primitive = permitted.resume_at_recovery_state ?
    restocker_reasoner::RecoveryPrimitive::kResumeAtRecoveryState :
    restocker_reasoner::RecoveryPrimitive::kAbandonTask;
  query.permitted_primitives = permitted.permitted();
  query.recovery_attempt = context_->task_status().recovery_attempt;
  query.maximum_recovery_attempts = config_.task.max_recovery_attempts;

  const auto mailbox = recovery_advice_mailbox_;
  const auto submitted = recovery_advisor_->submit(
    query, [mailbox](restocker_reasoner::RecoveryAdviceCompletion answer) {
      mailbox->deposit(std::move(answer));
    });
  if (!submitted) {
    // Declined by the backend: the deterministic policy decides.
    return;
  }
  recovery_question_ = std::move(query);

  if (recovery_audit_) {
    restocker_reasoner::RecoveryAuditRecord record;
    record.event = restocker_reasoner::RecoveryAuditEvent::kAsked;
    record.request_id = recovery_question_->request_id;
    record.goal_generation = recovery_question_->goal_generation;
    record.query_json = render_recovery_query_json(*recovery_question_);
    record.permitted_at_decision = recovery_question_->permitted_primitives;
    record.deterministic_primitive = recovery_question_->deterministic_primitive;
    recovery_audit_->append(record);
  }
}

restocker_reasoner::RecoveryPrimitive RestockCoordinatorDriver::consume_recovery_advice()
{
  using restocker_reasoner::RecoveryPrimitive;

  // Recompute permissions now: the set the question carried is stale.
  const auto permitted = recovery_permissions_now();
  const auto deterministic = permitted.resume_at_recovery_state ?
    RecoveryPrimitive::kResumeAtRecoveryState : RecoveryPrimitive::kAbandonTask;

  if (!recovery_advisor_ || !recovery_question_ || !context_ || !recovery_advice_mailbox_) {
    return deterministic;
  }

  const auto question = *recovery_question_;
  const auto answer = recovery_advice_mailbox_->take(
    question.request_id, context_->generation());
  forget_recovery_question();

  std::optional<restocker_reasoner::RecoveryAdvice> advice;
  std::string rejection;
  if (answer && answer->responded) {
    auto parsed = restocker_reasoner::parse_recovery_advice(
      answer->document, question, config_.reasoner_minimum_confidence);
    advice = std::move(parsed.advice);
    rejection = std::move(parsed.rejection);
  }

  const auto decision = resolve_recovery_advice(deterministic, permitted, advice);

  if (recovery_audit_) {
    restocker_reasoner::RecoveryAuditRecord record;
    record.event = restocker_reasoner::RecoveryAuditEvent::kDecided;
    record.request_id = question.request_id;
    record.goal_generation = question.goal_generation;
    record.query_json = render_recovery_query_json(question);
    record.permitted_at_decision = permitted.permitted();
    record.deterministic_primitive = deterministic;
    record.applied_primitive = decision.primitive;
    record.used = decision.followed_advice;
    if (answer) {
      record.backend = answer->backend;
      record.latency_ms = answer->latency.count();
      record.response_verbatim = answer->document;
      record.transport_detail = answer->transport_detail;
    } else {
      record.transport_detail = "no answer had arrived when the decision was taken";
    }
    record.validated = advice.has_value();
    record.rejection = rejection.empty() ? decision.rejection : rejection;
    if (advice) {
      record.failure_class = advice->failure_class;
      record.recommended_primitive = advice->recommended_primitive;
      record.confidence = advice->confidence;
      record.explanation = advice->explanation;
    }
    recovery_audit_->append(record);
    // The record, minus its verdict, waits for the resulting outcome.
    record.event = restocker_reasoner::RecoveryAuditEvent::kOutcome;
    record.outcome.clear();
    recovery_outcome_pending_ = std::move(record);
  }
  return decision.primitive;
}

void RestockCoordinatorDriver::begin_reconciliation(SteadyTime now)
{
  const auto ticket = ledger_.pending();
  if (!ticket || !context_) {
    inhibit("reconciliation phase has no retained operation ticket", now);
    return;
  }
  const auto kind = ledger_.next_reconciliation_kind();
  if (!kind) {
    inhibit("reconciliation ledger has no next strategy", now);
    return;
  }
  const auto started = ledger_.begin_reconciliation(
    ticket->goal_generation, ticket->operation_generation, *kind, now);
  if (started.error == ReconciliationStartError::kExhausted) {
    inhibit("bounded mutation reconciliation exhausted", now);
    return;
  }
  if (!started.attempt) {
    if (started.error != ReconciliationStartError::kAttemptAlreadyActive) {
      inhibit("operation ledger rejected reconciliation strategy", now);
    }
    return;
  }
  if (*kind == ReconciliationKind::kReadback) {
    if (ticket->command == RestockTaskCommand::kReserveTask) {
      request_validation(*ticket, true, started.attempt->attempt_number);
    } else {
      request_release_readback(*started.attempt);
    }
  } else if (ticket->command == RestockTaskCommand::kReserveTask) {
    request_reservation(*ticket, true, started.attempt->attempt_number);
  } else {
    request_release(*ticket, true, started.attempt->attempt_number);
  }
}

void RestockCoordinatorDriver::record_request_handle(
  WorldStateRequestHandle handle,
  std::optional<CoordinatorReconciliationCorrelation> reconciliation)
{
  request_handles_.push_back(
    PendingTransportRequest{std::move(handle), std::move(reconciliation)});
}

void RestockCoordinatorDriver::complete_matching_request_handle(
  const OperationCorrelation & correlation,
  const std::optional<CoordinatorReconciliationCorrelation> & reconciliation)
{
  const auto matches_reconciliation = [&reconciliation](const auto & candidate) {
    if (!reconciliation || !candidate) {
      return !reconciliation && !candidate;
    }
    return reconciliation->kind == candidate->kind &&
           reconciliation->attempt_number == candidate->attempt_number;
  };
  const auto match = std::find_if(
    request_handles_.begin(), request_handles_.end(),
    [&correlation, &matches_reconciliation](const auto & request) {
      return request.handle.correlation.goal_generation == correlation.goal_generation &&
             request.handle.correlation.operation_generation ==
             correlation.operation_generation &&
             matches_reconciliation(request.reconciliation);
    });
  if (match != request_handles_.end()) {
    request_handles_.erase(match);
  }
}

void RestockCoordinatorDriver::remove_reconciliation_request(
  const ReconciliationAttempt & attempt)
{
  const auto match = std::find_if(
    request_handles_.begin(), request_handles_.end(), [&attempt](const auto & request) {
      return request.handle.correlation.goal_generation == attempt.ticket.goal_generation &&
             request.handle.correlation.operation_generation ==
             attempt.ticket.operation_generation && request.reconciliation &&
             request.reconciliation->kind == attempt.kind &&
             request.reconciliation->attempt_number == attempt.attempt_number;
    });
  if (match != request_handles_.end()) {
    (void)world_state_.remove_pending_request(match->handle);
    request_handles_.erase(match);
  }
}

void RestockCoordinatorDriver::remove_operation_requests(
  const OperationCorrelation & correlation)
{
  auto request = request_handles_.begin();
  while (request != request_handles_.end()) {
    if (request->handle.correlation.goal_generation == correlation.goal_generation &&
      request->handle.correlation.operation_generation == correlation.operation_generation)
    {
      (void)world_state_.remove_pending_request(request->handle);
      request = request_handles_.erase(request);
    } else {
      ++request;
    }
  }
}

SteadyTime RestockCoordinatorDriver::bounded_deadline(
  SteadyTime start, std::chrono::milliseconds duration) const
{
  const auto remaining = SteadyTime::max() - start;
  if (duration >= std::chrono::duration_cast<std::chrono::milliseconds>(remaining)) {
    return SteadyTime::max();
  }
  return start + std::chrono::duration_cast<SteadyTime::duration>(duration);
}

RestockActionOutcome RestockCoordinatorDriver::terminal_outcome() const
{
  if (!context_) {
    return RestockActionOutcome::kInternalError;
  }
  const auto transition = context_->task_status();
  if (transition.state == RestockTaskState::kComplete) {
    return RestockActionOutcome::kSucceeded;
  }
  if (transition.state == RestockTaskState::kCanceled &&
    transition.fault == RestockTaskFault::kCanceled)
  {
    return RestockActionOutcome::kCanceled;
  }
  if (primary_outcome_) {
    return *primary_outcome_;
  }
  if (transition.fault == RestockTaskFault::kExternalInconsistency || inhibited_) {
    return RestockActionOutcome::kExternalInconsistency;
  }
  if (transition.state == RestockTaskState::kRequestOperator) {
    return RestockActionOutcome::kOperatorRequired;
  }
  return RestockActionOutcome::kValidationFailed;
}

void RestockCoordinatorDriver::note_command_submitted() noexcept
{
  commanded_.prior_commanded = commanded_.commanded;
  commanded_.prior_stop_observed = commanded_.stop_observed;
  commanded_.commanded = true;
  commanded_.stop_observed = false;
}

void RestockCoordinatorDriver::note_motion_submission_undone() noexcept
{
  commanded_.commanded = commanded_.prior_commanded;
  commanded_.stop_observed = commanded_.prior_stop_observed;
}

RestockActionMetrics RestockCoordinatorDriver::metrics(SteadyTime now) const
{
  RestockActionMetrics value;
  if (!context_) {
    return value;
  }
  if (context_->initial_snapshot()) {
    value.initial_world_revision = context_->initial_snapshot()->revision;
  }
  if (context_->latest_snapshot()) {
    value.final_world_revision = context_->latest_snapshot()->revision;
  }
  const auto transition = context_->task_status();
  value.attempt_count = static_cast<std::uint32_t>(
    std::min<std::size_t>(transition.attempt, std::numeric_limits<std::uint32_t>::max()));
  value.retry_count = transition.attempt > 0U ? static_cast<std::uint32_t>(
    std::min<std::size_t>(
      transition.attempt - 1U, std::numeric_limits<std::uint32_t>::max())) : 0U;
  if (now > context_->steady_started()) {
    value.elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - context_->steady_started());
  }
  // Card 086 stage 1b: never claim anything while the coordinator is inhibited.
  if (!inhibited_ && !admission_.snapshot().inhibited) {
    value.motion_definitely_not_started = !commanded_.commanded;
    value.execution_reached_terminal_stop = commanded_.commanded && commanded_.stop_observed;
  }
  return value;
}

RecoveryRestVerdict evaluate_recovery_rest(
  const restocker_world_state::RobotExecutionState & robot, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_age, std::chrono::nanoseconds maximum_future_skew,
  double rest_speed)
{
  std::ostringstream detail;
  detail << std::fixed << std::setprecision(3);
  if (robot.telemetry_revision == 0U || robot.telemetry_time.nanoseconds() <= 0 ||
    robot.telemetry_time.get_clock_type() != now.get_clock_type())
  {
    return {false, "no admitted robot telemetry on the evaluation clock, so no stop can be "
      "established"};
  }
  const auto age_ns = now.nanoseconds() - robot.telemetry_time.nanoseconds();
  const double age_s = static_cast<double>(age_ns) * 1.0e-9;
  if (age_ns > maximum_age.count() || -age_ns > maximum_future_skew.count()) {
    detail << "robot telemetry is " << age_s << " s old (bound " <<
      std::chrono::duration<double>(maximum_age).count() << " s), so it establishes neither "
      "motion nor rest and recovery cannot replan from the current state";
    return {false, detail.str()};
  }
  std::string fastest_name = "none";
  double fastest = 0.0;
  const auto consider = [&](double speed, std::string name) {
    if (!std::isfinite(speed)) {
      fastest = std::numeric_limits<double>::infinity();
      fastest_name = std::move(name);
    } else if (std::abs(speed) > fastest) {
      fastest = std::abs(speed);
      fastest_name = std::move(name);
    }
  };
  for (std::size_t index = 0; index < robot.joint_velocities.size(); ++index) {
    consider(robot.joint_velocities[index], "arm joint " + std::to_string(index + 1U));
  }
  for (std::size_t index = 0; index < robot.gripper_joint_velocities.size(); ++index) {
    consider(robot.gripper_joint_velocities[index], "gripper joint " + std::to_string(index + 1U));
  }
  consider(robot.rail_velocity, "rail");
  if (fastest > rest_speed) {
    detail << "the robot is still moving (" << fastest_name << " speed " << fastest <<
      " above the rest speed " << rest_speed << ", telemetry " << age_s <<
      " s old), so recovery cannot replan from its current state";
    return {false, detail.str()};
  }
  detail << "fastest speed " << fastest << " (" << fastest_name <<
    ") at or below the rest speed " << rest_speed << ", telemetry " << age_s << " s old";
  return {true, detail.str()};
}

}  // namespace restocker_task_executor
