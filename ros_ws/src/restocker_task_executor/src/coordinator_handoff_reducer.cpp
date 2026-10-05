// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_handoff_reducer.hpp"

#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] constexpr bool pre_seal_phase(PendingAcceptedHandoffPhase phase) noexcept
{
  switch (phase) {
    case PendingAcceptedHandoffPhase::kAwaitingEpoch:
    case PendingAcceptedHandoffPhase::kAwaitingAcceptedHandle:
    case PendingAcceptedHandoffPhase::kWaitingForPendingRoutes:
    case PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease:
      return true;
    case PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope:
    case PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted:
    case PendingAcceptedHandoffPhase::kPublishingDeferredControl:
    case PendingAcceptedHandoffPhase::kReadyToActivate:
    case PendingAcceptedHandoffPhase::kCleanupOnly:
      return false;
  }
  return false;
}

[[nodiscard]] constexpr bool post_seal_route_phase(
  PendingAcceptedHandoffPhase phase) noexcept
{
  return phase == PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope ||
         phase == PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted;
}

[[nodiscard]] constexpr bool valid_relation(PendingRouteObservationRelation relation) noexcept
{
  switch (relation) {
    case PendingRouteObservationRelation::kNewDistinct:
    case PendingRouteObservationRelation::kExactDuplicate:
    case PendingRouteObservationRelation::kCollision:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_route_evidence(PendingRouteEvidenceResult evidence) noexcept
{
  switch (evidence) {
    case PendingRouteEvidenceResult::kExact:
    case PendingRouteEvidenceResult::kInvalid:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_merge_for_route(
  PendingControlMergeDecision merge, PendingRouteKind kind,
  PendingRouteDeliveryClass delivery_class) noexcept
{
  switch (merge) {
    case PendingControlMergeDecision::kStoreIncoming:
    case PendingControlMergeDecision::kKeepExistingDisposeIncoming:
      return true;
    case PendingControlMergeDecision::kReplaceExistingDisposeDisplaced:
      return delivery_class == PendingRouteDeliveryClass::kPreSeal &&
             kind == PendingRouteKind::kTransformAuthorityLoss;
    case PendingControlMergeDecision::kCleanupOnly:
      return false;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_preparation_failure(PendingPreparationFailure failure) noexcept
{
  switch (failure) {
    case PendingPreparationFailure::kSteadyClockFailure:
    case PendingPreparationFailure::kCallbackOrAdapterFailure:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_deferred_completion(
  PendingDeferredCompletion completion) noexcept
{
  switch (completion) {
    case PendingDeferredCompletion::kDelivered:
    case PendingDeferredCompletion::kInboxLossSecured:
    case PendingDeferredCompletion::kInboxLossUnsecured:
      return true;
  }
  return false;
}

}  // namespace

PendingRouteWorkToken::PendingRouteWorkToken(
  PendingHandoffBindingKey binding, PendingRouteKind kind,
  PendingRouteDeliveryClass delivery_class, PendingRouteTokenId token) noexcept
: binding_(binding), kind_(kind), delivery_class_(delivery_class), token_(token), live_(true)
{
}

PendingHandoffWorkToken::PendingHandoffWorkToken(
  PendingHandoffBindingKey binding, PendingHandoffWorkKind kind,
  PendingHandoffTokenId token, PendingRouteEvidenceRevision prepared_revision) noexcept
: binding_(binding), kind_(kind), token_(token), prepared_revision_(prepared_revision), live_(true)
{
}

PendingRouteClaimDecision::PendingRouteClaimDecision(PendingRouteClaimStatus status) noexcept
: status_(status)
{
}

PendingRouteClaimDecision::PendingRouteClaimDecision(PendingRouteWorkToken && work) noexcept
: status_(PendingRouteClaimStatus::kClaimed), work_(std::move(work))
{
}

std::optional<PendingRouteWorkToken> PendingRouteClaimDecision::take_work() noexcept
{
  if (!work_ || !work_->live()) {
    return std::nullopt;
  }
  return std::optional<PendingRouteWorkToken>{std::move(*work_)};
}

PendingHandoffAdvanceDecision::PendingHandoffAdvanceDecision(
  PendingHandoffAdvanceStatus status) noexcept
: status_(status)
{
}

PendingHandoffAdvanceDecision::PendingHandoffAdvanceDecision(
  PendingHandoffAdvanceStatus status, PendingHandoffWorkToken && work) noexcept
: status_(status), work_(std::move(work))
{
}

std::optional<PendingHandoffWorkToken> PendingHandoffAdvanceDecision::take_work() noexcept
{
  if (!work_ || !work_->live()) {
    return std::nullopt;
  }
  return std::optional<PendingHandoffWorkToken>{std::move(*work_)};
}

PendingPreparationCompletionDecision::PendingPreparationCompletionDecision(
  PendingPreparationCompletionStatus status) noexcept
: status_(status)
{
}

PendingPreparationCompletionDecision::PendingPreparationCompletionDecision(
  PendingPreparationCompletionStatus status, PendingRouteWorkToken && work) noexcept
: status_(status), route_work_(std::move(work))
{
}

PendingPreparationCompletionDecision::PendingPreparationCompletionDecision(
  PendingPreparationCompletionStatus status, PendingHandoffWorkToken && work) noexcept
: status_(status), handoff_work_(std::move(work))
{
}

std::optional<PendingRouteWorkToken>
PendingPreparationCompletionDecision::take_route_work() noexcept
{
  if (!route_work_ || !route_work_->live()) {
    return std::nullopt;
  }
  return std::optional<PendingRouteWorkToken>{std::move(*route_work_)};
}

std::optional<PendingHandoffWorkToken>
PendingPreparationCompletionDecision::take_handoff_work() noexcept
{
  if (!handoff_work_ || !handoff_work_->live()) {
    return std::nullopt;
  }
  return std::optional<PendingHandoffWorkToken>{std::move(*handoff_work_)};
}

PendingAcceptedHandoffMachine::PendingAcceptedHandoffMachine(
  PendingHandoffBindingKey binding, PendingHandoffLimits limits) noexcept
: binding_(binding), limits_(limits), phase_(PendingAcceptedHandoffPhase::kAwaitingEpoch)
{
  fail_if_invalid();
}

PendingHandoffPhaseEventStatus PendingAcceptedHandoffMachine::install_epoch(
  const PendingHandoffBindingKey & binding) noexcept
{
  fail_if_invalid();
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    return PendingHandoffPhaseEventStatus::kCleanupOnly;
  }
  if (!binding_matches(binding) || phase_ != PendingAcceptedHandoffPhase::kAwaitingEpoch) {
    enter_cleanup_only();
    return PendingHandoffPhaseEventStatus::kCleanupOnly;
  }
  phase_ = PendingAcceptedHandoffPhase::kAwaitingAcceptedHandle;
  return PendingHandoffPhaseEventStatus::kApplied;
}

PendingHandoffPhaseEventStatus PendingAcceptedHandoffMachine::adopt_accepted_handle(
  const PendingHandoffBindingKey & binding) noexcept
{
  fail_if_invalid();
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    return PendingHandoffPhaseEventStatus::kCleanupOnly;
  }
  if (!binding_matches(binding) ||
    phase_ != PendingAcceptedHandoffPhase::kAwaitingAcceptedHandle)
  {
    enter_cleanup_only();
    return PendingHandoffPhaseEventStatus::kCleanupOnly;
  }
  phase_ = PendingAcceptedHandoffPhase::kWaitingForPendingRoutes;
  return PendingHandoffPhaseEventStatus::kApplied;
}

PendingRouteClaimDecision PendingAcceptedHandoffMachine::observe_route(
  const PendingHandoffBindingKey & binding, PendingRouteKind kind,
  PendingRouteObservationRelation relation) noexcept
{
  fail_if_invalid();
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  if (!binding_matches(binding)) {
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  return observe_route_impl(kind, relation);
}

PendingRouteClaimDecision PendingAcceptedHandoffMachine::observe_route_impl(
  PendingRouteKind kind, PendingRouteObservationRelation relation) noexcept
{
  if (!valid_pending_route_kind(kind) || !valid_relation(relation) ||
    phase_ == PendingAcceptedHandoffPhase::kPublishingDeferredControl ||
    phase_ == PendingAcceptedHandoffPhase::kReadyToActivate)
  {
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  if (!pre_seal_phase(phase_) && !post_seal_route_phase(phase_)) {
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  if (route_work_) {
    if (relation == PendingRouteObservationRelation::kExactDuplicate &&
      route_work_->kind == kind)
    {
      return PendingRouteClaimDecision{PendingRouteClaimStatus::kCoalesced};
    }
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  if (relation == PendingRouteObservationRelation::kExactDuplicate) {
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCoalesced};
  }
  if (relation != PendingRouteObservationRelation::kNewDistinct) {
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }

  const auto delivery_class = pre_seal_phase(phase_) ?
    PendingRouteDeliveryClass::kPreSeal : PendingRouteDeliveryClass::kPostSeal;
  auto work = issue_route(kind, delivery_class);
  if (!work) {
    enter_cleanup_only();
    return PendingRouteClaimDecision{PendingRouteClaimStatus::kCleanupOnly};
  }
  return PendingRouteClaimDecision{std::move(*work)};
}

PendingRouteCompletionStatus PendingAcceptedHandoffMachine::complete_route(
  PendingRouteWorkToken & work, PendingRouteEvidenceResult evidence,
  PendingControlMergeDecision merge) noexcept
{
  fail_if_invalid();
  if (!route_token_matches(work)) {
    enter_cleanup_only();
    return PendingRouteCompletionStatus::kCleanupOnly;
  }

  const bool cleanup_only = phase_ == PendingAcceptedHandoffPhase::kCleanupOnly;
  const bool legal_phase = work.delivery_class() == PendingRouteDeliveryClass::kPreSeal ?
    pre_seal_phase(phase_) : post_seal_route_phase(phase_);
  consume_route(work);
  if (cleanup_only) {
    return PendingRouteCompletionStatus::kRetainCleanupEvidence;
  }
  if (!legal_phase || !valid_route_evidence(evidence) ||
    evidence != PendingRouteEvidenceResult::kExact ||
    !valid_merge_for_route(merge, work.kind(), work.delivery_class()))
  {
    enter_cleanup_only();
    return PendingRouteCompletionStatus::kCleanupOnly;
  }
  return PendingRouteCompletionStatus::kMerged;
}

PendingHandoffAdvanceDecision PendingAcceptedHandoffMachine::advance(
  PendingHandoffAdvanceFacts facts) noexcept
{
  fail_if_invalid();
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly ||
    !pump_gate_allows_preaccept(facts.pump_gate))
  {
    enter_cleanup_only();
    return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kCleanupOnly};
  }

  if (!pre_seal_phase(phase_) && facts.pump_gate.outstanding) {
    enter_cleanup_only();
    return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kCleanupOnly};
  }

  if (phase_ == PendingAcceptedHandoffPhase::kWaitingForPendingRoutes) {
    if (route_work_) {
      return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kWait};
    }
    phase_ = PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease;
  }

  if (phase_ == PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease) {
    if (route_work_ || handoff_work_ || !pump_gate_allows_handoff(facts.pump_gate)) {
      return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kWait};
    }
    auto work = issue_handoff(
      PendingHandoffWorkKind::kPrepareAcceptedEnvelope, route_evidence_revision_);
    if (!work) {
      enter_cleanup_only();
      return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kCleanupOnly};
    }
    return PendingHandoffAdvanceDecision{
      PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope, std::move(*work)};
  }

  if (phase_ == PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted) {
    if (route_work_ || handoff_work_) {
      return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kWait};
    }
    if (facts.deferred_control_occupied) {
      auto work = issue_handoff(
        PendingHandoffWorkKind::kPublishDeferredControl, route_evidence_revision_);
      if (!work) {
        enter_cleanup_only();
        return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kCleanupOnly};
      }
      phase_ = PendingAcceptedHandoffPhase::kPublishingDeferredControl;
      return PendingHandoffAdvanceDecision{
        PendingHandoffAdvanceStatus::kPublishDeferredControl, std::move(*work)};
    }
    phase_ = PendingAcceptedHandoffPhase::kReadyToActivate;
  }

  if (phase_ == PendingAcceptedHandoffPhase::kReadyToActivate) {
    return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kReadyToActivate};
  }
  return PendingHandoffAdvanceDecision{PendingHandoffAdvanceStatus::kWait};
}

PendingPreparationCompletionDecision PendingAcceptedHandoffMachine::complete_preparation(
  PendingHandoffWorkToken & work) noexcept
{
  fail_if_invalid();
  if (!handoff_token_matches(work) ||
    work.kind() != PendingHandoffWorkKind::kPrepareAcceptedEnvelope)
  {
    enter_cleanup_only();
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    consume_handoff(work);
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }
  if (phase_ != PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease) {
    consume_handoff(work);
    enter_cleanup_only();
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }

  const auto prepared_revision = work.prepared_revision();
  consume_handoff(work);
  if (route_work_ || prepared_revision != route_evidence_revision_) {
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kRetryPreparation};
  }
  auto publish = issue_handoff(
    PendingHandoffWorkKind::kPublishAcceptedEnvelope, route_evidence_revision_);
  if (!publish) {
    enter_cleanup_only();
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }
  phase_ = PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope;
  return PendingPreparationCompletionDecision{
    PendingPreparationCompletionStatus::kPublishAcceptedEnvelope, std::move(*publish)};
}

PendingPreparationCompletionDecision
PendingAcceptedHandoffMachine::complete_preparation_failure(
  PendingHandoffWorkToken & work, PendingPreparationFailure failure,
  PendingRouteObservationRelation failure_relation) noexcept
{
  fail_if_invalid();
  if (!handoff_token_matches(work) ||
    work.kind() != PendingHandoffWorkKind::kPrepareAcceptedEnvelope)
  {
    enter_cleanup_only();
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    consume_handoff(work);
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }
  if (phase_ != PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease ||
    !valid_preparation_failure(failure) || !valid_relation(failure_relation))
  {
    consume_handoff(work);
    enter_cleanup_only();
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kCleanupOnly};
  }

  consume_handoff(work);
  const auto kind = failure == PendingPreparationFailure::kSteadyClockFailure ?
    PendingRouteKind::kSteadyClockFailure : PendingRouteKind::kCallbackOrAdapterFailure;
  auto route = observe_route_impl(kind, failure_relation);
  if (route.status() == PendingRouteClaimStatus::kClaimed) {
    auto route_work = route.take_work();
    if (!route_work) {
      enter_cleanup_only();
      return PendingPreparationCompletionDecision{
        PendingPreparationCompletionStatus::kCleanupOnly};
    }
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kRouteClaimed, std::move(*route_work)};
  }
  if (route.status() == PendingRouteClaimStatus::kCoalesced) {
    return PendingPreparationCompletionDecision{
      PendingPreparationCompletionStatus::kRouteCoalesced};
  }
  return PendingPreparationCompletionDecision{
    PendingPreparationCompletionStatus::kCleanupOnly};
}

PendingAcceptedPublicationStatus PendingAcceptedHandoffMachine::complete_accepted_publication(
  PendingHandoffWorkToken & work, AcceptedEnvelopeDepositDecision decision) noexcept
{
  fail_if_invalid();
  if (!handoff_token_matches(work) ||
    work.kind() != PendingHandoffWorkKind::kPublishAcceptedEnvelope)
  {
    enter_cleanup_only();
    return PendingAcceptedPublicationStatus::kCleanupOnly;
  }
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    if (decision == AcceptedEnvelopeDepositDecision::kCommitted) {
      consume_handoff(work);
    } else {
      spend_handoff_token(work);
    }
    return PendingAcceptedPublicationStatus::kCleanupOnly;
  }
  if (phase_ != PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope) {
    consume_handoff(work);
    enter_cleanup_only();
    return PendingAcceptedPublicationStatus::kCleanupOnly;
  }
  if (decision != AcceptedEnvelopeDepositDecision::kCommitted) {
    // The external attempt is spent, but an ambiguous result does not prove that inbox ownership
    // was or was not transferred. Preserve the descriptor as unresolved cleanup evidence.
    spend_handoff_token(work);
    enter_cleanup_only();
    return PendingAcceptedPublicationStatus::kCleanupOnly;
  }
  consume_handoff(work);
  phase_ = PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted;
  return PendingAcceptedPublicationStatus::kCommitted;
}

PendingDeferredPublicationStatus PendingAcceptedHandoffMachine::complete_deferred_publication(
  PendingHandoffWorkToken & work, PendingDeferredCompletion completion) noexcept
{
  fail_if_invalid();
  if (!handoff_token_matches(work) ||
    work.kind() != PendingHandoffWorkKind::kPublishDeferredControl)
  {
    enter_cleanup_only();
    return PendingDeferredPublicationStatus::kCleanupOnly;
  }
  if (phase_ == PendingAcceptedHandoffPhase::kCleanupOnly) {
    consume_handoff(work);
    return PendingDeferredPublicationStatus::kCleanupOnly;
  }
  if (phase_ != PendingAcceptedHandoffPhase::kPublishingDeferredControl ||
    !valid_deferred_completion(completion))
  {
    consume_handoff(work);
    enter_cleanup_only();
    return PendingDeferredPublicationStatus::kCleanupOnly;
  }
  consume_handoff(work);
  if (completion == PendingDeferredCompletion::kInboxLossUnsecured) {
    enter_cleanup_only();
    return PendingDeferredPublicationStatus::kCleanupOnly;
  }
  phase_ = PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted;
  return PendingDeferredPublicationStatus::kCompleted;
}

void PendingAcceptedHandoffMachine::enter_cleanup_only() noexcept
{
  phase_ = PendingAcceptedHandoffPhase::kCleanupOnly;
}

PendingAcceptedHandoffSnapshot PendingAcceptedHandoffMachine::snapshot() const noexcept
{
  return PendingAcceptedHandoffSnapshot{
    binding_, phase_, route_evidence_revision_, last_route_token_, route_work_, handoff_work_};
}

bool PendingAcceptedHandoffMachine::valid() const noexcept
{
  if (!valid_pending_handoff_binding_key(binding_) || !valid_pending_handoff_phase(phase_) ||
    last_route_token_ > limits_.route_tokens ||
    route_evidence_revision_ > limits_.route_revisions ||
    last_handoff_token_ > limits_.handoff_tokens)
  {
    return false;
  }
  if (route_work_) {
    if (!valid_pending_route_kind(route_work_->kind) ||
      !valid_pending_route_delivery_class(route_work_->delivery_class) ||
      route_work_->token == 0U || route_work_->token != last_route_token_)
    {
      return false;
    }
    if (phase_ != PendingAcceptedHandoffPhase::kCleanupOnly &&
      ((route_work_->delivery_class == PendingRouteDeliveryClass::kPreSeal &&
      !pre_seal_phase(phase_)) ||
      (route_work_->delivery_class == PendingRouteDeliveryClass::kPostSeal &&
      !post_seal_route_phase(phase_))))
    {
      return false;
    }
  }
  if (handoff_work_) {
    if (!valid_pending_handoff_work_kind(handoff_work_->kind) ||
      handoff_work_->token == 0U || handoff_work_->token != last_handoff_token_ ||
      handoff_work_->prepared_revision > route_evidence_revision_)
    {
      return false;
    }
    if (!handoff_work_->capability_outstanding &&
      phase_ != PendingAcceptedHandoffPhase::kCleanupOnly)
    {
      return false;
    }
    if (phase_ != PendingAcceptedHandoffPhase::kCleanupOnly) {
      const bool phase_matches =
        (handoff_work_->kind == PendingHandoffWorkKind::kPrepareAcceptedEnvelope &&
        phase_ == PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease) ||
        (handoff_work_->kind == PendingHandoffWorkKind::kPublishAcceptedEnvelope &&
        phase_ == PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope) ||
        (handoff_work_->kind == PendingHandoffWorkKind::kPublishDeferredControl &&
        phase_ == PendingAcceptedHandoffPhase::kPublishingDeferredControl);
      if (!phase_matches) {
        return false;
      }
    }
  } else {
    if (phase_ == PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope ||
      phase_ == PendingAcceptedHandoffPhase::kPublishingDeferredControl)
    {
      return false;
    }
  }
  if (phase_ == PendingAcceptedHandoffPhase::kReadyToActivate && route_work_) {
    return false;
  }
  return true;
}

bool PendingAcceptedHandoffMachine::binding_matches(
  const PendingHandoffBindingKey & binding) const noexcept
{
  return binding == binding_;
}

bool PendingAcceptedHandoffMachine::route_token_matches(
  const PendingRouteWorkToken & work) const noexcept
{
  return work.live() && route_work_ && work.binding() == binding_ &&
         work.kind() == route_work_->kind && work.delivery_class() == route_work_->delivery_class &&
         work.token() == route_work_->token;
}

bool PendingAcceptedHandoffMachine::handoff_token_matches(
  const PendingHandoffWorkToken & work) const noexcept
{
  return work.live() && handoff_work_ && work.binding() == binding_ &&
         handoff_work_->capability_outstanding &&
         work.kind() == handoff_work_->kind && work.token() == handoff_work_->token &&
         work.prepared_revision() == handoff_work_->prepared_revision;
}

std::optional<PendingRouteWorkToken> PendingAcceptedHandoffMachine::issue_route(
  PendingRouteKind kind, PendingRouteDeliveryClass delivery_class) noexcept
{
  const auto next_token = checked_next_pending_route_token(last_route_token_);
  const auto next_revision =
    checked_next_pending_route_evidence_revision(route_evidence_revision_);
  if (!next_token || !next_revision || *next_token > limits_.route_tokens ||
    *next_revision > limits_.route_revisions)
  {
    return std::nullopt;
  }
  last_route_token_ = *next_token;
  route_evidence_revision_ = *next_revision;
  route_work_ = PendingRouteWorkSnapshot{kind, delivery_class, *next_token};
  PendingRouteWorkToken work{binding_, kind, delivery_class, *next_token};
  return std::optional<PendingRouteWorkToken>{std::move(work)};
}

std::optional<PendingHandoffWorkToken> PendingAcceptedHandoffMachine::issue_handoff(
  PendingHandoffWorkKind kind, PendingRouteEvidenceRevision prepared_revision) noexcept
{
  const auto next_token = checked_next_pending_handoff_token(last_handoff_token_);
  if (!next_token || *next_token > limits_.handoff_tokens) {
    return std::nullopt;
  }
  last_handoff_token_ = *next_token;
  handoff_work_ = PendingHandoffWorkSnapshot{kind, *next_token, prepared_revision, true};
  PendingHandoffWorkToken work{binding_, kind, *next_token, prepared_revision};
  return std::optional<PendingHandoffWorkToken>{std::move(work)};
}

void PendingAcceptedHandoffMachine::consume_route(PendingRouteWorkToken & work) noexcept
{
  route_work_.reset();
  work.live_.consume();
}

void PendingAcceptedHandoffMachine::consume_handoff(PendingHandoffWorkToken & work) noexcept
{
  handoff_work_.reset();
  work.live_.consume();
}

void PendingAcceptedHandoffMachine::spend_handoff_token(PendingHandoffWorkToken & work) noexcept
{
  handoff_work_->capability_outstanding = false;
  work.live_.consume();
}

void PendingAcceptedHandoffMachine::fail_if_invalid() noexcept
{
  if (!valid()) {
    enter_cleanup_only();
  }
}

}  // namespace restocker_task_executor
