// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{
namespace
{

// Each epoch admits one accepted terminal acknowledgement, so its lineage cookie is a per-epoch
// constant; the quiescence seal consumes it once.
constexpr AcceptedTerminalAckCookie kAcceptedTerminalAckCookie{1U};

[[nodiscard]] constexpr std::optional<FeedbackAttemptId> checked_next_feedback_attempt(
  FeedbackAttemptId current) noexcept
{
  if (current == std::numeric_limits<FeedbackAttemptId>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id) noexcept
{
  return std::any_of(
    goal_id.begin(), goal_id.end(), [](std::uint8_t value) {return value != 0U;});
}

[[nodiscard]] bool valid_route_kind(ActiveTerminationRouteKind kind) noexcept
{
  switch (kind) {
    case ActiveTerminationRouteKind::kCancel:
    case ActiveTerminationRouteKind::kDrain:
    case ActiveTerminationRouteKind::kTransformAuthorityLoss:
    case ActiveTerminationRouteKind::kSimulationTimeAuthorityLoss:
    case ActiveTerminationRouteKind::kProtocolFailure:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_invalidation_kind(ActiveFaultInvalidationKind kind) noexcept
{
  switch (kind) {
    case ActiveFaultInvalidationKind::kStartupAuthorityFailure:
    case ActiveFaultInvalidationKind::kAdapterInvariantFailure:
    case ActiveFaultInvalidationKind::kShutdownTimeout:
      return true;
  }
  return false;
}

[[nodiscard]] std::optional<ActiveTerminalClassification> valid_terminal_classification(
  const CoordinatorDriverOutput & output) noexcept
{
  switch (output.kind) {
    case CoordinatorDriverOutputKind::kCanceled:
      if (output.outcome == RestockActionOutcome::kCanceled) {
        return ActiveTerminalClassification::kOriginalCanceled;
      }
      return std::nullopt;
    case CoordinatorDriverOutputKind::kAborted:
      switch (output.outcome) {
        case RestockActionOutcome::kNoCompatiblePair:
        case RestockActionOutcome::kObservationEvidenceStale:
        case RestockActionOutcome::kValidationFailed:
        case RestockActionOutcome::kPlanningFailed:
        case RestockActionOutcome::kExecutionFailed:
        case RestockActionOutcome::kVerificationFailed:
        case RestockActionOutcome::kRecoveryExhausted:
        case RestockActionOutcome::kExternalInconsistency:
        case RestockActionOutcome::kOperatorRequired:
        case RestockActionOutcome::kShutdown:
        // Card 051 rung 5: the typed recoverable skip is a first-class aborted outcome. An
        // unknown outcome classifies as null and the epoch normalizes the terminal to an
        // internal fault — "coordinator fault won before terminal result delivery" — so the
        // campaign would block on an internal error instead of reading the skip.
        case RestockActionOutcome::kRecoverableSkip:
        case RestockActionOutcome::kInternalError:
          return ActiveTerminalClassification::kOriginalAborted;
        case RestockActionOutcome::kUnset:
        case RestockActionOutcome::kSucceeded:
        case RestockActionOutcome::kCanceled:
          return std::nullopt;
      }
      return std::nullopt;
    case CoordinatorDriverOutputKind::kSucceeded:
      if (output.outcome == RestockActionOutcome::kSucceeded) {
        return ActiveTerminalClassification::kOriginalSucceeded;
      }
      return std::nullopt;
    case CoordinatorDriverOutputKind::kFeedback:
    case CoordinatorDriverOutputKind::kInhibited:
      return std::nullopt;
  }
  return std::nullopt;
}

template<typename Capability>
[[nodiscard]] std::optional<Capability> take_optional(std::optional<Capability> & source) noexcept
{
  std::optional<Capability> result;
  if (source) {
    result.emplace(std::move(*source));
    source.reset();
  }
  return result;
}

}  // namespace

CoordinatorActiveFaultClaim::CoordinatorActiveFaultClaim(
  CoordinatorGoalId goal_id, GoalGeneration generation,
  ActiveTerminationRouteKind kind) noexcept
: goal_id_(goal_id), generation_(generation), kind_(kind), live_(true)
{
}

CoordinatorTerminalPublicationPermit::CoordinatorTerminalPublicationPermit(
  CoordinatorGoalId goal_id, GoalGeneration generation,
  CoordinatorDriverOutput && output, ActiveTerminalClassification classification) noexcept
: goal_id_(goal_id), generation_(generation),
  output_(std::move(output)), classification_(classification), live_(true)
{
}

CoordinatorTerminalPublicationPermit::CoordinatorTerminalPublicationPermit(
  CoordinatorTerminalPublicationPermit && other) noexcept
: goal_id_(other.goal_id_), generation_(other.generation_),
  output_(std::move(other.output_)), classification_(other.classification_),
  live_(std::move(other.live_))
{
}

CoordinatorTerminalPublicationPermit & CoordinatorTerminalPublicationPermit::operator=(
  CoordinatorTerminalPublicationPermit && other) noexcept
{
  if (this != &other) {
    goal_id_ = other.goal_id_;
    generation_ = other.generation_;
    output_ = std::move(other.output_);
    classification_ = other.classification_;
    live_ = std::move(other.live_);
  }
  return *this;
}

CoordinatorTerminalAcknowledgementPermit::CoordinatorTerminalAcknowledgementPermit(
  CoordinatorGoalId goal_id, GoalGeneration generation,
  CoordinatorDriverOutput && output, ActiveTerminalClassification classification) noexcept
: goal_id_(goal_id), generation_(generation),
  output_(std::move(output)), classification_(classification), live_(true)
{
}

CoordinatorTerminalAcknowledgementPermit::CoordinatorTerminalAcknowledgementPermit(
  CoordinatorTerminalAcknowledgementPermit && other) noexcept
: goal_id_(other.goal_id_), generation_(other.generation_),
  output_(std::move(other.output_)), classification_(other.classification_),
  live_(std::move(other.live_))
{
}

CoordinatorTerminalAcknowledgementPermit &
CoordinatorTerminalAcknowledgementPermit::operator=(
  CoordinatorTerminalAcknowledgementPermit && other) noexcept
{
  if (this != &other) {
    goal_id_ = other.goal_id_;
    generation_ = other.generation_;
    output_ = std::move(other.output_);
    classification_ = other.classification_;
    live_ = std::move(other.live_);
  }
  return *this;
}

CoordinatorAcceptedTerminalAckWitness::CoordinatorAcceptedTerminalAckWitness(
  CoordinatorGoalId goal_id, GoalGeneration generation,
  AcceptedTerminalAckCookie ack_cookie) noexcept
: goal_id_(goal_id), generation_(generation),
  ack_cookie_(ack_cookie), live_(true)
{
}

CoordinatorFeedbackPermit::CoordinatorFeedbackPermit(
  CoordinatorGoalId goal_id, GoalGeneration generation,
  FeedbackAttemptId attempt_id) noexcept
: goal_id_(goal_id), generation_(generation),
  attempt_id_(attempt_id), live_(true)
{
}

CoordinatorActiveFaultDecision::CoordinatorActiveFaultDecision(
  ActiveFaultClaimStatus status) noexcept
: status_(status)
{
}

CoordinatorActiveFaultDecision::CoordinatorActiveFaultDecision(
  CoordinatorActiveFaultClaim && claim) noexcept
: status_(ActiveFaultClaimStatus::kClaimed), claim_(std::move(claim))
{
}

std::optional<CoordinatorActiveFaultClaim> CoordinatorActiveFaultDecision::take_claim() noexcept
{
  return take_optional(claim_);
}

CoordinatorActiveTerminalDecision::CoordinatorActiveTerminalDecision(
  ActiveTerminalDisposition disposition, bool output_was_consumed,
  std::optional<CoordinatorTerminalPublicationPermit> publication,
  std::optional<CoordinatorActiveFaultClaim> claim) noexcept
: disposition_(disposition), output_was_consumed_(output_was_consumed),
  publication_(std::move(publication)), claim_(std::move(claim))
{
}

std::optional<CoordinatorTerminalPublicationPermit>
CoordinatorActiveTerminalDecision::take_publication_permit() noexcept
{
  return take_optional(publication_);
}

std::optional<CoordinatorActiveFaultClaim>
CoordinatorActiveTerminalDecision::take_fault_claim() noexcept
{
  return take_optional(claim_);
}

CoordinatorRetainedPublicationDecision::CoordinatorRetainedPublicationDecision(
  ActiveRetainedPublicationStatus status,
  std::optional<CoordinatorTerminalPublicationPermit> permit) noexcept
: status_(status), permit_(std::move(permit))
{
}

std::optional<CoordinatorTerminalPublicationPermit>
CoordinatorRetainedPublicationDecision::take_permit() noexcept
{
  return take_optional(permit_);
}

CoordinatorPublicationReturnDecision::CoordinatorPublicationReturnDecision(
  ActivePublicationReturnStatus status,
  std::optional<CoordinatorTerminalAcknowledgementPermit> permit) noexcept
: status_(status), permit_(std::move(permit))
{
}

std::optional<CoordinatorTerminalAcknowledgementPermit>
CoordinatorPublicationReturnDecision::take_ack_permit() noexcept
{
  return take_optional(permit_);
}

CoordinatorTerminalAckCompletionDecision::CoordinatorTerminalAckCompletionDecision(
  ActiveTerminalAcknowledgementStatus status) noexcept
: status_(status)
{
}

CoordinatorTerminalAckCompletionDecision::CoordinatorTerminalAckCompletionDecision(
  CoordinatorAcceptedTerminalAckWitness && witness) noexcept
: status_(ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible),
  accepted_ack_witness_(std::move(witness))
{
}

CoordinatorTerminalAckCompletionDecision::CoordinatorTerminalAckCompletionDecision(
  CoordinatorTerminalAckCompletionDecision && other) noexcept
: status_(other.status_), accepted_ack_witness_(take_optional(other.accepted_ack_witness_))
{
}

CoordinatorTerminalAckCompletionDecision & CoordinatorTerminalAckCompletionDecision::operator=(
  CoordinatorTerminalAckCompletionDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    accepted_ack_witness_ = take_optional(other.accepted_ack_witness_);
  }
  return *this;
}

std::optional<CoordinatorAcceptedTerminalAckWitness>
CoordinatorTerminalAckCompletionDecision::take_accepted_ack_witness() noexcept
{
  return take_optional(accepted_ack_witness_);
}

CoordinatorFeedbackReserveDecision::CoordinatorFeedbackReserveDecision(
  ActiveFeedbackReserveStatus status, std::optional<CoordinatorFeedbackPermit> permit) noexcept
: status_(status), permit_(std::move(permit))
{
}

std::optional<CoordinatorFeedbackPermit>
CoordinatorFeedbackReserveDecision::take_permit() noexcept
{
  return take_optional(permit_);
}

CoordinatorFeedbackFailureDecision::CoordinatorFeedbackFailureDecision(
  ActiveFeedbackCompletionStatus status,
  std::optional<CoordinatorActiveFaultClaim> claim) noexcept
: status_(status), claim_(std::move(claim))
{
}

std::optional<CoordinatorActiveFaultClaim>
CoordinatorFeedbackFailureDecision::take_fault_claim() noexcept
{
  return take_optional(claim_);
}

CoordinatorActiveFaultEpoch::CoordinatorActiveFaultEpoch(
  CoordinatorGoalId goal_id, GoalGeneration generation)
: goal_id_(goal_id), generation_(generation)
{
  if (!valid_goal_id(goal_id_)) {
    throw std::invalid_argument("active fault epoch requires a nonzero goal ID");
  }
  if (!admissible_goal_generation(generation_)) {
    throw std::invalid_argument("active fault epoch requires an admissible goal generation");
  }
}

bool CoordinatorActiveFaultEpoch::identity_matches(
  const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept
{
  return goal_id == goal_id_ && generation == generation_;
}

CoordinatorActiveFaultClaim CoordinatorActiveFaultEpoch::make_claim(
  ActiveTerminationRouteKind kind) noexcept
{
  outstanding_route_kind_ = kind;
  route_claim_outstanding_ = true;
  route_phase_ = ActiveFaultRoutePhase::kRoutingPending;
  return CoordinatorActiveFaultClaim{goal_id_, generation_, kind};
}

void CoordinatorActiveFaultEpoch::invalidate_claim() noexcept
{
  route_claim_outstanding_ = false;
}

CoordinatorActiveFaultDecision CoordinatorActiveFaultEpoch::claim_fault(
  ActiveTerminationRouteKind kind) noexcept
{
  if (!valid_route_kind(kind)) {
    return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kInvalidRouteKind};
  }
  if (terminal_phase_ == ActiveTerminalPhase::kPublicationReserved ||
    terminal_phase_ == ActiveTerminalPhase::kPublicationReturnedAwaitingAck ||
    (terminal_phase_ == ActiveTerminalPhase::kDelivered && retirement_eligible_))
  {
    return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kTerminalWon};
  }
  switch (route_phase_) {
    case ActiveFaultRoutePhase::kClear:
      return CoordinatorActiveFaultDecision{make_claim(kind)};
    case ActiveFaultRoutePhase::kRoutingPending:
      return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kAlreadyRouting};
    case ActiveFaultRoutePhase::kSecured:
      return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kAlreadySecured};
    case ActiveFaultRoutePhase::kUnsecured:
      return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kUnsecured};
  }
  return CoordinatorActiveFaultDecision{ActiveFaultClaimStatus::kInvalidArgument};
}

ActiveFaultCompletionStatus CoordinatorActiveFaultEpoch::complete_fault(
  CoordinatorActiveFaultClaim & claim, bool exact_authority_secured) noexcept
{
  if (!claim.live_.live()) {
    return ActiveFaultCompletionStatus::kNotLive;
  }
  if (!identity_matches(claim.goal_id_, claim.generation_)) {
    return ActiveFaultCompletionStatus::kIdentityMismatch;
  }
  if (!route_claim_outstanding_) {
    return ActiveFaultCompletionStatus::kStaleClaim;
  }
  if (route_phase_ != ActiveFaultRoutePhase::kRoutingPending) {
    return ActiveFaultCompletionStatus::kWrongPhase;
  }
  if (claim.kind_ != outstanding_route_kind_) {
    return ActiveFaultCompletionStatus::kClaimMismatch;
  }
  claim.live_.consume();
  route_claim_outstanding_ = false;
  route_phase_ = exact_authority_secured ?
    ActiveFaultRoutePhase::kSecured : ActiveFaultRoutePhase::kUnsecured;
  return exact_authority_secured ?
         ActiveFaultCompletionStatus::kSecured : ActiveFaultCompletionStatus::kUnsecured;
}

ActiveFaultInvalidationStatus CoordinatorActiveFaultEpoch::invalidate_fault(
  ActiveFaultInvalidationKind kind) noexcept
{
  if (!valid_invalidation_kind(kind)) {
    return ActiveFaultInvalidationStatus::kInvalidKind;
  }
  if (terminal_phase_ == ActiveTerminalPhase::kPublicationReserved ||
    terminal_phase_ == ActiveTerminalPhase::kPublicationReturnedAwaitingAck ||
    (terminal_phase_ == ActiveTerminalPhase::kDelivered && retirement_eligible_))
  {
    return ActiveFaultInvalidationStatus::kTerminalWon;
  }
  if (route_phase_ == ActiveFaultRoutePhase::kUnsecured) {
    return ActiveFaultInvalidationStatus::kAlreadyUnsecured;
  }
  invalidate_claim();
  route_phase_ = ActiveFaultRoutePhase::kUnsecured;
  return ActiveFaultInvalidationStatus::kInvalidatedUnsecured;
}

CoordinatorActiveTerminalDecision CoordinatorActiveFaultEpoch::offer_terminal(
  CoordinatorDriverOutput & output) noexcept
{
  if (terminal_phase_ == ActiveTerminalPhase::kDelivered) {
    return {ActiveTerminalDisposition::kAlreadyDeliveredOutputUnconsumed, false, {}, {}};
  }
  if (terminal_phase_ == ActiveTerminalPhase::kPublicationReserved ||
    terminal_phase_ == ActiveTerminalPhase::kPublicationReturnedAwaitingAck)
  {
    return {ActiveTerminalDisposition::kAlreadyReservedOutputUnconsumed, false, {}, {}};
  }
  if (feedback_outstanding_) {
    return {ActiveTerminalDisposition::kFeedbackOutstandingOutputUnconsumed, false, {}, {}};
  }
  if (retained_terminal_.occupied()) {
    contradiction_observed_ = true;
    invalidate_claim();
    route_phase_ = ActiveFaultRoutePhase::kUnsecured;
    return {ActiveTerminalDisposition::kContradictoryTerminalOutputUnconsumed, false, {}, {}};
  }

  if (output.goal_generation != generation_) {
    switch (route_phase_) {
      case ActiveFaultRoutePhase::kClear: {
          auto claim = make_claim(ActiveTerminationRouteKind::kProtocolFailure);
          return {
            ActiveTerminalDisposition::kProtocolClaimedOutputUnconsumed, false, {},
            std::optional<CoordinatorActiveFaultClaim>{std::move(claim)}};
        }
      case ActiveFaultRoutePhase::kRoutingPending:
        return {
          ActiveTerminalDisposition::kProtocolAlreadyPendingOutputUnconsumed, false, {}, {}};
      case ActiveFaultRoutePhase::kSecured:
        route_phase_ = ActiveFaultRoutePhase::kUnsecured;
        return {ActiveTerminalDisposition::kIdentityMismatchOutputUnconsumed, false, {}, {}};
      case ActiveFaultRoutePhase::kUnsecured:
        return {ActiveTerminalDisposition::kIdentityMismatchOutputUnconsumed, false, {}, {}};
    }
  }

  const auto classification = valid_terminal_classification(output);
  switch (route_phase_) {
    case ActiveFaultRoutePhase::kClear:
      if (classification) {
        terminal_phase_ = ActiveTerminalPhase::kPublicationReserved;
        CoordinatorTerminalPublicationPermit permit{
          goal_id_, generation_, std::move(output), *classification};
        return {
          ActiveTerminalDisposition::kPublicationPrepared, true,
          std::optional<CoordinatorTerminalPublicationPermit>{std::move(permit)}, {}};
      }
      (void)retained_terminal_.retain(std::move(output));
      terminal_phase_ = ActiveTerminalPhase::kRetained;
      {
        auto claim = make_claim(ActiveTerminationRouteKind::kProtocolFailure);
        return {
          ActiveTerminalDisposition::kRetainedAndFaultClaimed, true, {},
          std::optional<CoordinatorActiveFaultClaim>{std::move(claim)}};
      }
    case ActiveFaultRoutePhase::kRoutingPending:
      (void)retained_terminal_.retain(std::move(output));
      terminal_phase_ = ActiveTerminalPhase::kRetained;
      return {ActiveTerminalDisposition::kRetainedWhileRouting, true, {}, {}};
    case ActiveFaultRoutePhase::kSecured: {
        const auto effective = classification &&
          *classification != ActiveTerminalClassification::kOriginalSucceeded ?
          *classification : ActiveTerminalClassification::kInternalFaultAbort;
        terminal_phase_ = ActiveTerminalPhase::kPublicationReserved;
        CoordinatorTerminalPublicationPermit permit{
          goal_id_, generation_, std::move(output), effective};
        return {
          effective == ActiveTerminalClassification::kInternalFaultAbort ?
          ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization :
          ActiveTerminalDisposition::kPublicationPrepared,
          true, std::optional<CoordinatorTerminalPublicationPermit>{std::move(permit)}, {}};
      }
    case ActiveFaultRoutePhase::kUnsecured:
      (void)retained_terminal_.retain(std::move(output));
      terminal_phase_ = ActiveTerminalPhase::kRetained;
      return {ActiveTerminalDisposition::kRetainedUnsecured, true, {}, {}};
  }
  return {ActiveTerminalDisposition::kInvalidArgumentOutputUnconsumed, false, {}, {}};
}

CoordinatorRetainedPublicationDecision
CoordinatorActiveFaultEpoch::prepare_secured_retained_publication() noexcept
{
  if (terminal_phase_ == ActiveTerminalPhase::kDelivered) {
    return {ActiveRetainedPublicationStatus::kAlreadyDelivered, {}};
  }
  if (terminal_phase_ == ActiveTerminalPhase::kPublicationReserved ||
    terminal_phase_ == ActiveTerminalPhase::kPublicationReturnedAwaitingAck)
  {
    return {ActiveRetainedPublicationStatus::kAlreadyReserved, {}};
  }
  if (feedback_outstanding_) {
    return {ActiveRetainedPublicationStatus::kFeedbackOutstanding, {}};
  }
  if (route_phase_ == ActiveFaultRoutePhase::kUnsecured) {
    return {ActiveRetainedPublicationStatus::kUnsecured, {}};
  }
  if (route_phase_ != ActiveFaultRoutePhase::kSecured) {
    return {ActiveRetainedPublicationStatus::kNotSecured, {}};
  }
  if (!retained_terminal_.occupied()) {
    return {ActiveRetainedPublicationStatus::kNoRetainedTerminal, {}};
  }
  auto output = retained_terminal_.take();
  const auto classification = valid_terminal_classification(*output);
  const auto effective = classification &&
    *classification != ActiveTerminalClassification::kOriginalSucceeded ?
    *classification : ActiveTerminalClassification::kInternalFaultAbort;
  terminal_phase_ = ActiveTerminalPhase::kPublicationReserved;
  CoordinatorTerminalPublicationPermit permit{
    goal_id_, generation_, std::move(*output), effective};
  return {
    ActiveRetainedPublicationStatus::kPrepared,
    std::optional<CoordinatorTerminalPublicationPermit>{std::move(permit)}};
}

CoordinatorPublicationReturnDecision CoordinatorActiveFaultEpoch::terminal_publication_returned(
  CoordinatorTerminalPublicationPermit & permit) noexcept
{
  if (!permit.live_.live()) {
    return {ActivePublicationReturnStatus::kNotLive, {}};
  }
  if (!identity_matches(permit.goal_id_, permit.generation_)) {
    return {ActivePublicationReturnStatus::kIdentityMismatch, {}};
  }
  if (terminal_phase_ != ActiveTerminalPhase::kPublicationReserved) {
    return {ActivePublicationReturnStatus::kStalePermit, {}};
  }
  permit.live_.consume();
  terminal_phase_ = ActiveTerminalPhase::kPublicationReturnedAwaitingAck;
  middleware_publication_returned_ = true;
  CoordinatorTerminalAcknowledgementPermit acknowledgement{
    permit.goal_id_, permit.generation_, std::move(permit.output_),
    permit.classification_};
  return {
    ActivePublicationReturnStatus::kAcknowledgementPrepared,
    std::optional<CoordinatorTerminalAcknowledgementPermit>{std::move(acknowledgement)}};
}

ActivePublicationUnknownStatus CoordinatorActiveFaultEpoch::terminal_publication_unknown(
  CoordinatorTerminalPublicationPermit & permit) noexcept
{
  if (!permit.live_.live()) {
    return ActivePublicationUnknownStatus::kNotLive;
  }
  if (!identity_matches(permit.goal_id_, permit.generation_)) {
    return ActivePublicationUnknownStatus::kIdentityMismatch;
  }
  if (terminal_phase_ != ActiveTerminalPhase::kPublicationReserved) {
    return ActivePublicationUnknownStatus::kStalePermit;
  }
  permit.live_.consume();
  route_phase_ = ActiveFaultRoutePhase::kUnsecured;
  invalidate_claim();
  if (!retained_terminal_.retain(std::move(permit.output_))) {
    contradiction_observed_ = true;
    terminal_phase_ = ActiveTerminalPhase::kRetained;
    return ActivePublicationUnknownStatus::kRetainedSlotConflict;
  }
  terminal_phase_ = ActiveTerminalPhase::kRetained;
  return ActivePublicationUnknownStatus::kRestoredRetainedUnsecured;
}

CoordinatorTerminalAckCompletionDecision CoordinatorActiveFaultEpoch::complete_terminal_ack(
  CoordinatorTerminalAcknowledgementPermit & permit, bool accepted) noexcept
{
  if (!permit.live_.live()) {
    return CoordinatorTerminalAckCompletionDecision{
      ActiveTerminalAcknowledgementStatus::kNotLive};
  }
  if (!identity_matches(permit.goal_id_, permit.generation_)) {
    return CoordinatorTerminalAckCompletionDecision{
      ActiveTerminalAcknowledgementStatus::kIdentityMismatch};
  }
  if (terminal_phase_ != ActiveTerminalPhase::kPublicationReturnedAwaitingAck) {
    return CoordinatorTerminalAckCompletionDecision{
      ActiveTerminalAcknowledgementStatus::kStalePermit};
  }
  permit.live_.consume();
  terminal_phase_ = ActiveTerminalPhase::kDelivered;
  if (accepted && !feedback_outstanding_) {
    retirement_eligible_ = true;
    CoordinatorAcceptedTerminalAckWitness witness{
      goal_id_, generation_, kAcceptedTerminalAckCookie};
    return CoordinatorTerminalAckCompletionDecision{std::move(witness)};
  }
  route_phase_ = ActiveFaultRoutePhase::kUnsecured;
  retirement_eligible_ = false;
  return CoordinatorTerminalAckCompletionDecision{
    ActiveTerminalAcknowledgementStatus::kRejectedDeliveredUnsecured};
}

CoordinatorFeedbackReserveDecision CoordinatorActiveFaultEpoch::reserve_feedback(
  const CoordinatorGoalId & goal_id, GoalGeneration generation) noexcept
{
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation)) {
    return {ActiveFeedbackReserveStatus::kInvalidArgument, {}};
  }
  if (!identity_matches(goal_id, generation)) {
    return {ActiveFeedbackReserveStatus::kIdentityMismatch, {}};
  }
  if (feedback_outstanding_) {
    return {ActiveFeedbackReserveStatus::kAlreadyReserved, {}};
  }
  if (terminal_phase_ != ActiveTerminalPhase::kEmpty) {
    return {ActiveFeedbackReserveStatus::kTerminalReserved, {}};
  }
  switch (route_phase_) {
    case ActiveFaultRoutePhase::kRoutingPending:
      return {ActiveFeedbackReserveStatus::kRoutingPending, {}};
    case ActiveFaultRoutePhase::kSecured:
      return {ActiveFeedbackReserveStatus::kSecured, {}};
    case ActiveFaultRoutePhase::kUnsecured:
      return {ActiveFeedbackReserveStatus::kUnsecured, {}};
    case ActiveFaultRoutePhase::kClear:
      break;
  }
  const auto next_attempt = checked_next_feedback_attempt(last_feedback_attempt_);
  if (!next_attempt || *next_attempt == 0U) {
    route_phase_ = ActiveFaultRoutePhase::kUnsecured;
    return {ActiveFeedbackReserveStatus::kAttemptExhausted, {}};
  }
  last_feedback_attempt_ = *next_attempt;
  outstanding_feedback_attempt_ = *next_attempt;
  feedback_outstanding_ = true;
  CoordinatorFeedbackPermit permit{goal_id_, generation_, *next_attempt};
  return {
    ActiveFeedbackReserveStatus::kReserved,
    std::optional<CoordinatorFeedbackPermit>{std::move(permit)}};
}

ActiveFeedbackCompletionStatus CoordinatorActiveFaultEpoch::feedback_returned(
  CoordinatorFeedbackPermit & permit) noexcept
{
  if (!permit.live_.live()) {
    return ActiveFeedbackCompletionStatus::kNotLive;
  }
  if (!identity_matches(permit.goal_id_, permit.generation_)) {
    return ActiveFeedbackCompletionStatus::kIdentityMismatch;
  }
  if (!feedback_outstanding_ || permit.attempt_id_ != outstanding_feedback_attempt_) {
    return ActiveFeedbackCompletionStatus::kStalePermit;
  }
  if (terminal_phase_ != ActiveTerminalPhase::kEmpty) {
    return ActiveFeedbackCompletionStatus::kWrongPhase;
  }
  permit.live_.consume();
  feedback_outstanding_ = false;
  outstanding_feedback_attempt_ = 0U;
  return ActiveFeedbackCompletionStatus::kReturned;
}

CoordinatorFeedbackFailureDecision CoordinatorActiveFaultEpoch::feedback_failed(
  CoordinatorFeedbackPermit & permit) noexcept
{
  if (!permit.live_.live()) {
    return {ActiveFeedbackCompletionStatus::kNotLive, {}};
  }
  if (!identity_matches(permit.goal_id_, permit.generation_)) {
    return {ActiveFeedbackCompletionStatus::kIdentityMismatch, {}};
  }
  if (!feedback_outstanding_ || permit.attempt_id_ != outstanding_feedback_attempt_) {
    return {ActiveFeedbackCompletionStatus::kStalePermit, {}};
  }
  if (terminal_phase_ != ActiveTerminalPhase::kEmpty) {
    return {ActiveFeedbackCompletionStatus::kWrongPhase, {}};
  }
  permit.live_.consume();
  feedback_outstanding_ = false;
  outstanding_feedback_attempt_ = 0U;
  switch (route_phase_) {
    case ActiveFaultRoutePhase::kClear: {
        auto claim = make_claim(ActiveTerminationRouteKind::kProtocolFailure);
        return {
          ActiveFeedbackCompletionStatus::kFailedAndFaultClaimed,
          std::optional<CoordinatorActiveFaultClaim>{std::move(claim)}};
      }
    case ActiveFaultRoutePhase::kRoutingPending:
      return {ActiveFeedbackCompletionStatus::kFailedRouteAlreadyPending, {}};
    case ActiveFaultRoutePhase::kSecured:
      return {ActiveFeedbackCompletionStatus::kFailedRouteAlreadySecured, {}};
    case ActiveFaultRoutePhase::kUnsecured:
      return {ActiveFeedbackCompletionStatus::kFailedRouteUnsecured, {}};
  }
  return {ActiveFeedbackCompletionStatus::kWrongPhase, {}};
}

CoordinatorActiveFaultEpochSnapshot CoordinatorActiveFaultEpoch::snapshot() const noexcept
{
  return {
    goal_id_, generation_, route_phase_, terminal_phase_, last_feedback_attempt_,
    route_claim_outstanding_, feedback_outstanding_, retained_terminal_.occupied(),
    contradiction_observed_, middleware_publication_returned_, retirement_eligible_};
}

const CoordinatorDriverOutput * CoordinatorActiveFaultEpoch::retained_terminal_output()
const noexcept
{
  return retained_terminal_.inspect();
}

ActiveTerminalResult make_active_terminal_result(
  const CoordinatorDriverOutput & original, ActiveTerminalClassification classification,
  std::chrono::milliseconds elapsed, const std::string & internal_fault_detail)
{
  const bool fault = classification == ActiveTerminalClassification::kInternalFaultAbort;
  auto metrics = original.metrics;
  metrics.elapsed = elapsed;
  if (fault) {
    metrics.motion_definitely_not_started = false;
    metrics.execution_reached_terminal_stop = false;
  }
  ActiveTerminalResult terminal;
  terminal.outcome = fault ? RestockActionOutcome::kInternalError : original.outcome;
  terminal.detail = fault ? internal_fault_detail : original.detail;
  terminal.result = make_action_result(terminal.outcome, metrics, terminal.detail);
  // The product this goal was transferring, when selection named one: a recoverable skip carries
  // it so the campaign can charge max_product_skips against the right product.
  terminal.result.selected_object_id =
    original.selected_pair ? original.selected_pair->object_id.value : 0U;
  return terminal;
}

}  // namespace restocker_task_executor
