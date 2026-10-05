// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_driver_output.hpp"
#include "restocker_task_executor/coordinator_one_shot.hpp"

namespace restocker_task_executor
{

using FeedbackAttemptId = std::uint64_t;
using AcceptedTerminalAckCookie = std::uint64_t;

[[nodiscard]] constexpr std::optional<AcceptedTerminalAckCookie>
checked_next_accepted_terminal_ack_cookie(AcceptedTerminalAckCookie current) noexcept
{
  if (current == std::numeric_limits<AcceptedTerminalAckCookie>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

enum class ActiveTerminationRouteKind : std::uint8_t
{
  kCancel,
  kDrain,
  kTransformAuthorityLoss,
  kSimulationTimeAuthorityLoss,
  kProtocolFailure,
};

enum class ActiveFaultInvalidationKind : std::uint8_t
{
  kStartupAuthorityFailure,
  kAdapterInvariantFailure,
  kShutdownTimeout,
};

enum class ActiveFaultInvalidationStatus : std::uint8_t
{
  kInvalidatedUnsecured,
  kTerminalWon,
  kAlreadyUnsecured,
  kInvalidKind,
};

enum class ActiveFaultRoutePhase : std::uint8_t
{
  kClear,
  kRoutingPending,
  kSecured,
  kUnsecured,
};

enum class ActiveTerminalPhase : std::uint8_t
{
  kEmpty,
  kRetained,
  kPublicationReserved,
  kPublicationReturnedAwaitingAck,
  kDelivered,
};

enum class ActiveTerminalClassification : std::uint8_t
{
  kOriginalCanceled,
  kOriginalAborted,
  kOriginalSucceeded,
  kInternalFaultAbort,
};

enum class ActiveFaultClaimStatus : std::uint8_t
{
  kClaimed,
  kTerminalWon,
  kAlreadyRouting,
  kAlreadySecured,
  kUnsecured,
  kIdentityMismatch,
  kInvalidRouteKind,
  kInvalidArgument,
};

enum class ActiveFaultCompletionStatus : std::uint8_t
{
  kSecured,
  kUnsecured,
  kNotLive,
  kIdentityMismatch,
  kStaleClaim,
  kWrongPhase,
  kClaimMismatch,
};

enum class ActiveTerminalDisposition : std::uint8_t
{
  kPublicationPrepared,
  kPublicationPreparedWithFaultNormalization,
  kRetainedAndFaultClaimed,
  kRetainedWhileRouting,
  kRetainedUnsecured,
  kProtocolClaimedOutputUnconsumed,
  kProtocolAlreadyPendingOutputUnconsumed,
  kContradictoryTerminalOutputUnconsumed,
  kIdentityMismatchOutputUnconsumed,
  kAlreadyReservedOutputUnconsumed,
  kFeedbackOutstandingOutputUnconsumed,
  kAlreadyDeliveredOutputUnconsumed,
  kInvalidArgumentOutputUnconsumed,
};

enum class ActiveRetainedPublicationStatus : std::uint8_t
{
  kPrepared,
  kNotSecured,
  kNoRetainedTerminal,
  kAlreadyReserved,
  kFeedbackOutstanding,
  kAlreadyDelivered,
  kUnsecured,
};

enum class ActivePublicationReturnStatus : std::uint8_t
{
  kAcknowledgementPrepared,
  kNotLive,
  kIdentityMismatch,
  kStalePermit,
};

enum class ActivePublicationUnknownStatus : std::uint8_t
{
  kRestoredRetainedUnsecured,
  kNotLive,
  kIdentityMismatch,
  kStalePermit,
  kRetainedSlotConflict,
};

enum class ActiveTerminalAcknowledgementStatus : std::uint8_t
{
  kAcceptedRetirementEligible,
  kRejectedDeliveredUnsecured,
  kNotLive,
  kIdentityMismatch,
  kStalePermit,
};

enum class ActiveFeedbackReserveStatus : std::uint8_t
{
  kReserved,
  kAttemptExhausted,
  kAlreadyReserved,
  kTerminalReserved,
  kRoutingPending,
  kSecured,
  kUnsecured,
  kIdentityMismatch,
  kInvalidArgument,
};

enum class ActiveFeedbackCompletionStatus : std::uint8_t
{
  kReturned,
  kFailedAndFaultClaimed,
  kFailedRouteAlreadyPending,
  kFailedRouteAlreadySecured,
  kFailedRouteUnsecured,
  kNotLive,
  kIdentityMismatch,
  kStalePermit,
  kWrongPhase,
};

class CoordinatorActiveFaultEpoch;
class CoordinatorGenerationQuiescence;

class CoordinatorActiveFaultClaim final
{
public:
  CoordinatorActiveFaultClaim() = delete;
  CoordinatorActiveFaultClaim(const CoordinatorActiveFaultClaim &) = delete;
  CoordinatorActiveFaultClaim(CoordinatorActiveFaultClaim &&) noexcept = default;
  CoordinatorActiveFaultClaim & operator=(const CoordinatorActiveFaultClaim &) = delete;
  CoordinatorActiveFaultClaim & operator=(CoordinatorActiveFaultClaim &&) noexcept = default;
  ~CoordinatorActiveFaultClaim() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] ActiveTerminationRouteKind kind() const noexcept {return kind_;}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorActiveFaultClaim(
    CoordinatorGoalId goal_id, GoalGeneration generation,
    ActiveTerminationRouteKind kind) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  ActiveTerminationRouteKind kind_{ActiveTerminationRouteKind::kProtocolFailure};
  CoordinatorOneShot live_;
};

class CoordinatorTerminalPublicationPermit final
{
public:
  CoordinatorTerminalPublicationPermit() = delete;
  CoordinatorTerminalPublicationPermit(const CoordinatorTerminalPublicationPermit &) = delete;
  CoordinatorTerminalPublicationPermit(CoordinatorTerminalPublicationPermit && other) noexcept;
  CoordinatorTerminalPublicationPermit & operator=(
    const CoordinatorTerminalPublicationPermit &) = delete;
  CoordinatorTerminalPublicationPermit & operator=(
    CoordinatorTerminalPublicationPermit && other) noexcept;
  ~CoordinatorTerminalPublicationPermit() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}
  [[nodiscard]] const CoordinatorDriverOutput & original_output() const noexcept {return output_;}
  [[nodiscard]] ActiveTerminalClassification classification() const noexcept
  {
    return classification_;
  }

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorTerminalPublicationPermit(
    CoordinatorGoalId goal_id, GoalGeneration generation,
    CoordinatorDriverOutput && output, ActiveTerminalClassification classification) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  CoordinatorDriverOutput output_;
  ActiveTerminalClassification classification_{ActiveTerminalClassification::kInternalFaultAbort};
  CoordinatorOneShot live_;
};

class CoordinatorTerminalAcknowledgementPermit final
{
public:
  CoordinatorTerminalAcknowledgementPermit() = delete;
  CoordinatorTerminalAcknowledgementPermit(
    const CoordinatorTerminalAcknowledgementPermit &) = delete;
  CoordinatorTerminalAcknowledgementPermit(
    CoordinatorTerminalAcknowledgementPermit && other) noexcept;
  CoordinatorTerminalAcknowledgementPermit & operator=(
    const CoordinatorTerminalAcknowledgementPermit &) = delete;
  CoordinatorTerminalAcknowledgementPermit & operator=(
    CoordinatorTerminalAcknowledgementPermit && other) noexcept;
  ~CoordinatorTerminalAcknowledgementPermit() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}
  [[nodiscard]] const CoordinatorDriverOutput & original_output() const noexcept {return output_;}
  [[nodiscard]] ActiveTerminalClassification classification() const noexcept
  {
    return classification_;
  }

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorTerminalAcknowledgementPermit(
    CoordinatorGoalId goal_id, GoalGeneration generation,
    CoordinatorDriverOutput && output, ActiveTerminalClassification classification) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  CoordinatorDriverOutput output_;
  ActiveTerminalClassification classification_{ActiveTerminalClassification::kInternalFaultAbort};
  CoordinatorOneShot live_;
};

class CoordinatorAcceptedTerminalAckWitness final
{
public:
  CoordinatorAcceptedTerminalAckWitness() = delete;
  CoordinatorAcceptedTerminalAckWitness(
    const CoordinatorAcceptedTerminalAckWitness &) = delete;
  CoordinatorAcceptedTerminalAckWitness(
    CoordinatorAcceptedTerminalAckWitness &&) noexcept = default;
  CoordinatorAcceptedTerminalAckWitness & operator=(
    const CoordinatorAcceptedTerminalAckWitness &) = delete;
  CoordinatorAcceptedTerminalAckWitness & operator=(
    CoordinatorAcceptedTerminalAckWitness &&) noexcept = default;
  ~CoordinatorAcceptedTerminalAckWitness() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}

private:
  friend class CoordinatorActiveFaultEpoch;
  friend class CoordinatorGenerationQuiescence;
  CoordinatorAcceptedTerminalAckWitness(
    CoordinatorGoalId goal_id, GoalGeneration generation,
    AcceptedTerminalAckCookie ack_cookie) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  AcceptedTerminalAckCookie ack_cookie_{0U};
  CoordinatorOneShot live_;
};

class CoordinatorFeedbackPermit final
{
public:
  CoordinatorFeedbackPermit() = delete;
  CoordinatorFeedbackPermit(const CoordinatorFeedbackPermit &) = delete;
  CoordinatorFeedbackPermit(CoordinatorFeedbackPermit &&) noexcept = default;
  CoordinatorFeedbackPermit & operator=(const CoordinatorFeedbackPermit &) = delete;
  CoordinatorFeedbackPermit & operator=(CoordinatorFeedbackPermit &&) noexcept = default;
  ~CoordinatorFeedbackPermit() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorFeedbackPermit(
    CoordinatorGoalId goal_id, GoalGeneration generation,
    FeedbackAttemptId attempt_id) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  FeedbackAttemptId attempt_id_{0U};
  CoordinatorOneShot live_;
};

class CoordinatorActiveFaultDecision final
{
public:
  CoordinatorActiveFaultDecision() = delete;
  CoordinatorActiveFaultDecision(const CoordinatorActiveFaultDecision &) = delete;
  CoordinatorActiveFaultDecision(CoordinatorActiveFaultDecision &&) noexcept = default;
  CoordinatorActiveFaultDecision & operator=(const CoordinatorActiveFaultDecision &) = delete;
  CoordinatorActiveFaultDecision & operator=(CoordinatorActiveFaultDecision &&) noexcept = default;
  ~CoordinatorActiveFaultDecision() = default;

  [[nodiscard]] ActiveFaultClaimStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim> take_claim() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  explicit CoordinatorActiveFaultDecision(ActiveFaultClaimStatus status) noexcept;
  explicit CoordinatorActiveFaultDecision(CoordinatorActiveFaultClaim && claim) noexcept;

  ActiveFaultClaimStatus status_;
  std::optional<CoordinatorActiveFaultClaim> claim_;
};

class CoordinatorActiveTerminalDecision final
{
public:
  CoordinatorActiveTerminalDecision() = delete;
  CoordinatorActiveTerminalDecision(const CoordinatorActiveTerminalDecision &) = delete;
  CoordinatorActiveTerminalDecision(CoordinatorActiveTerminalDecision &&) noexcept = default;
  CoordinatorActiveTerminalDecision & operator=(const CoordinatorActiveTerminalDecision &) = delete;
  CoordinatorActiveTerminalDecision & operator=(
    CoordinatorActiveTerminalDecision &&) noexcept = default;
  ~CoordinatorActiveTerminalDecision() = default;

  [[nodiscard]] ActiveTerminalDisposition disposition() const noexcept {return disposition_;}
  [[nodiscard]] bool output_was_consumed() const noexcept {return output_was_consumed_;}
  [[nodiscard]] std::optional<CoordinatorTerminalPublicationPermit> take_publication_permit()
  noexcept;
  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim> take_fault_claim() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorActiveTerminalDecision(
    ActiveTerminalDisposition disposition, bool output_was_consumed,
    std::optional<CoordinatorTerminalPublicationPermit> publication,
    std::optional<CoordinatorActiveFaultClaim> claim) noexcept;

  ActiveTerminalDisposition disposition_;
  bool output_was_consumed_{false};
  std::optional<CoordinatorTerminalPublicationPermit> publication_;
  std::optional<CoordinatorActiveFaultClaim> claim_;
};

class CoordinatorRetainedPublicationDecision final
{
public:
  CoordinatorRetainedPublicationDecision() = delete;
  CoordinatorRetainedPublicationDecision(const CoordinatorRetainedPublicationDecision &) = delete;
  CoordinatorRetainedPublicationDecision(
    CoordinatorRetainedPublicationDecision &&) noexcept = default;
  CoordinatorRetainedPublicationDecision & operator=(
    const CoordinatorRetainedPublicationDecision &) = delete;
  CoordinatorRetainedPublicationDecision & operator=(
    CoordinatorRetainedPublicationDecision &&) noexcept = default;
  ~CoordinatorRetainedPublicationDecision() = default;

  [[nodiscard]] ActiveRetainedPublicationStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorTerminalPublicationPermit> take_permit() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorRetainedPublicationDecision(
    ActiveRetainedPublicationStatus status,
    std::optional<CoordinatorTerminalPublicationPermit> permit) noexcept;

  ActiveRetainedPublicationStatus status_;
  std::optional<CoordinatorTerminalPublicationPermit> permit_;
};

class CoordinatorPublicationReturnDecision final
{
public:
  CoordinatorPublicationReturnDecision() = delete;
  CoordinatorPublicationReturnDecision(const CoordinatorPublicationReturnDecision &) = delete;
  CoordinatorPublicationReturnDecision(CoordinatorPublicationReturnDecision &&) noexcept = default;
  CoordinatorPublicationReturnDecision & operator=(
    const CoordinatorPublicationReturnDecision &) = delete;
  CoordinatorPublicationReturnDecision & operator=(
    CoordinatorPublicationReturnDecision &&) noexcept = default;
  ~CoordinatorPublicationReturnDecision() = default;

  [[nodiscard]] ActivePublicationReturnStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorTerminalAcknowledgementPermit> take_ack_permit() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorPublicationReturnDecision(
    ActivePublicationReturnStatus status,
    std::optional<CoordinatorTerminalAcknowledgementPermit> permit) noexcept;

  ActivePublicationReturnStatus status_;
  std::optional<CoordinatorTerminalAcknowledgementPermit> permit_;
};

class CoordinatorTerminalAckCompletionDecision final
{
public:
  CoordinatorTerminalAckCompletionDecision() = delete;
  CoordinatorTerminalAckCompletionDecision(
    const CoordinatorTerminalAckCompletionDecision &) = delete;
  CoordinatorTerminalAckCompletionDecision(
    CoordinatorTerminalAckCompletionDecision && other) noexcept;
  CoordinatorTerminalAckCompletionDecision & operator=(
    const CoordinatorTerminalAckCompletionDecision &) = delete;
  CoordinatorTerminalAckCompletionDecision & operator=(
    CoordinatorTerminalAckCompletionDecision && other) noexcept;
  ~CoordinatorTerminalAckCompletionDecision() = default;

  [[nodiscard]] ActiveTerminalAcknowledgementStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorAcceptedTerminalAckWitness>
  take_accepted_ack_witness() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  explicit CoordinatorTerminalAckCompletionDecision(
    ActiveTerminalAcknowledgementStatus status) noexcept;
  explicit CoordinatorTerminalAckCompletionDecision(
    CoordinatorAcceptedTerminalAckWitness && witness) noexcept;

  ActiveTerminalAcknowledgementStatus status_;
  std::optional<CoordinatorAcceptedTerminalAckWitness> accepted_ack_witness_;
};

class CoordinatorFeedbackReserveDecision final
{
public:
  CoordinatorFeedbackReserveDecision() = delete;
  CoordinatorFeedbackReserveDecision(const CoordinatorFeedbackReserveDecision &) = delete;
  CoordinatorFeedbackReserveDecision(CoordinatorFeedbackReserveDecision &&) noexcept = default;
  CoordinatorFeedbackReserveDecision & operator=(
    const CoordinatorFeedbackReserveDecision &) = delete;
  CoordinatorFeedbackReserveDecision & operator=(
    CoordinatorFeedbackReserveDecision &&) noexcept = default;
  ~CoordinatorFeedbackReserveDecision() = default;

  [[nodiscard]] ActiveFeedbackReserveStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorFeedbackPermit> take_permit() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorFeedbackReserveDecision(
    ActiveFeedbackReserveStatus status, std::optional<CoordinatorFeedbackPermit> permit) noexcept;

  ActiveFeedbackReserveStatus status_;
  std::optional<CoordinatorFeedbackPermit> permit_;
};

class CoordinatorFeedbackFailureDecision final
{
public:
  CoordinatorFeedbackFailureDecision() = delete;
  CoordinatorFeedbackFailureDecision(const CoordinatorFeedbackFailureDecision &) = delete;
  CoordinatorFeedbackFailureDecision(CoordinatorFeedbackFailureDecision &&) noexcept = default;
  CoordinatorFeedbackFailureDecision & operator=(
    const CoordinatorFeedbackFailureDecision &) = delete;
  CoordinatorFeedbackFailureDecision & operator=(
    CoordinatorFeedbackFailureDecision &&) noexcept = default;
  ~CoordinatorFeedbackFailureDecision() = default;

  [[nodiscard]] ActiveFeedbackCompletionStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim> take_fault_claim() noexcept;

private:
  friend class CoordinatorActiveFaultEpoch;
  CoordinatorFeedbackFailureDecision(
    ActiveFeedbackCompletionStatus status,
    std::optional<CoordinatorActiveFaultClaim> claim) noexcept;

  ActiveFeedbackCompletionStatus status_;
  std::optional<CoordinatorActiveFaultClaim> claim_;
};

struct CoordinatorActiveFaultEpochSnapshot
{
  CoordinatorGoalId goal_id{};
  GoalGeneration goal_generation{0U};
  ActiveFaultRoutePhase route_phase{ActiveFaultRoutePhase::kClear};
  ActiveTerminalPhase terminal_phase{ActiveTerminalPhase::kEmpty};
  FeedbackAttemptId last_feedback_attempt{0U};
  bool route_claim_outstanding{false};
  bool feedback_outstanding{false};
  bool retained_terminal{false};
  bool contradiction_observed{false};
  bool middleware_publication_returned{false};
  bool retirement_eligible{false};
};

class CoordinatorActiveFaultEpoch final
{
public:
  CoordinatorActiveFaultEpoch(CoordinatorGoalId goal_id, GoalGeneration generation);
  CoordinatorActiveFaultEpoch(const CoordinatorActiveFaultEpoch &) = delete;
  CoordinatorActiveFaultEpoch(CoordinatorActiveFaultEpoch &&) = delete;
  CoordinatorActiveFaultEpoch & operator=(const CoordinatorActiveFaultEpoch &) = delete;
  CoordinatorActiveFaultEpoch & operator=(CoordinatorActiveFaultEpoch &&) = delete;
  ~CoordinatorActiveFaultEpoch() = default;

  [[nodiscard]] CoordinatorActiveFaultDecision claim_fault(
    ActiveTerminationRouteKind kind) noexcept;
  [[nodiscard]] ActiveFaultCompletionStatus complete_fault(
    CoordinatorActiveFaultClaim & claim, bool exact_authority_secured) noexcept;
  [[nodiscard]] ActiveFaultInvalidationStatus invalidate_fault(
    ActiveFaultInvalidationKind kind) noexcept;

  [[nodiscard]] CoordinatorActiveTerminalDecision offer_terminal(
    CoordinatorDriverOutput & output) noexcept;
  [[nodiscard]] CoordinatorRetainedPublicationDecision
  prepare_secured_retained_publication() noexcept;
  [[nodiscard]] CoordinatorPublicationReturnDecision terminal_publication_returned(
    CoordinatorTerminalPublicationPermit & permit) noexcept;
  [[nodiscard]] ActivePublicationUnknownStatus terminal_publication_unknown(
    CoordinatorTerminalPublicationPermit & permit) noexcept;
  [[nodiscard]] CoordinatorTerminalAckCompletionDecision complete_terminal_ack(
    CoordinatorTerminalAcknowledgementPermit & permit, bool accepted) noexcept;

  [[nodiscard]] CoordinatorFeedbackReserveDecision reserve_feedback(
    const CoordinatorGoalId & goal_id, GoalGeneration generation) noexcept;
  [[nodiscard]] ActiveFeedbackCompletionStatus feedback_returned(
    CoordinatorFeedbackPermit & permit) noexcept;
  [[nodiscard]] CoordinatorFeedbackFailureDecision feedback_failed(
    CoordinatorFeedbackPermit & permit) noexcept;

  [[nodiscard]] CoordinatorActiveFaultEpochSnapshot snapshot() const noexcept;

  // Borrowed read-only audit view. The pointer is valid only under the epoch's external
  // synchronization and until the next epoch mutation.
  [[nodiscard]] const CoordinatorDriverOutput * retained_terminal_output() const noexcept;

private:
  [[nodiscard]] CoordinatorActiveFaultClaim make_claim(
    ActiveTerminationRouteKind kind) noexcept;
  void invalidate_claim() noexcept;
  [[nodiscard]] bool identity_matches(
    const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  ActiveFaultRoutePhase route_phase_{ActiveFaultRoutePhase::kClear};
  ActiveTerminalPhase terminal_phase_{ActiveTerminalPhase::kEmpty};
  CoordinatorRetainedTerminal retained_terminal_;
  ActiveTerminationRouteKind outstanding_route_kind_{ActiveTerminationRouteKind::kProtocolFailure};
  FeedbackAttemptId last_feedback_attempt_{0U};
  FeedbackAttemptId outstanding_feedback_attempt_{0U};
  bool route_claim_outstanding_{false};
  bool feedback_outstanding_{false};
  bool contradiction_observed_{false};
  bool middleware_publication_returned_{false};
  bool retirement_eligible_{false};
};

namespace detail
{
template<typename T>
inline constexpr bool nothrow_authority_value_v =
  std::is_nothrow_move_constructible_v<T> &&
  std::is_nothrow_move_assignable_v<T> &&
  std::is_nothrow_destructible_v<T>;

}  // namespace detail

static_assert(detail::nothrow_authority_value_v<CoordinatorActiveFaultDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorActiveTerminalDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorRetainedPublicationDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorPublicationReturnDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorFeedbackReserveDecision>);
static_assert(detail::nothrow_authority_value_v<CoordinatorFeedbackFailureDecision>);

// What the node delivers for one terminal: the published outcome/detail and the action result built
// from them. A coordinator-fault abort replaces the original terminal with kInternalError, and no
// history proves the robot settled then, so the result carries no motion evidence (Card 086 stage
// 1b review S2). Pure so a test can pin it without the ROS node.
struct ActiveTerminalResult
{
  RestockActionOutcome outcome{RestockActionOutcome::kUnset};
  std::string detail;
  restocker_interfaces::action::RestockProduct::Result result;
};

[[nodiscard]] ActiveTerminalResult make_active_terminal_result(
  const CoordinatorDriverOutput & original, ActiveTerminalClassification classification,
  std::chrono::milliseconds elapsed, const std::string & internal_fault_detail);

}  // namespace restocker_task_executor
