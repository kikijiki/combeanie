// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>

#include "restocker_task_executor/coordinator_handoff_policy.hpp"
#include "restocker_task_executor/coordinator_one_shot.hpp"
#include "restocker_task_executor/coordinator_operation_types.hpp"

namespace restocker_task_executor
{

using PendingBindingIncarnation = std::uint64_t;
using PendingRouteTokenId = std::uint64_t;
using PendingRouteEvidenceRevision = std::uint64_t;
using PendingHandoffTokenId = std::uint64_t;

[[nodiscard]] constexpr std::optional<PendingBindingIncarnation>
checked_next_pending_binding_incarnation(PendingBindingIncarnation current) noexcept
{
  if (current == std::numeric_limits<PendingBindingIncarnation>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] constexpr std::optional<PendingRouteTokenId> checked_next_pending_route_token(
  PendingRouteTokenId current) noexcept
{
  if (current == std::numeric_limits<PendingRouteTokenId>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] constexpr std::optional<PendingRouteEvidenceRevision>
checked_next_pending_route_evidence_revision(PendingRouteEvidenceRevision current) noexcept
{
  if (current == std::numeric_limits<PendingRouteEvidenceRevision>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] constexpr std::optional<PendingHandoffTokenId> checked_next_pending_handoff_token(
  PendingHandoffTokenId current) noexcept
{
  if (current == std::numeric_limits<PendingHandoffTokenId>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

enum class PendingAcceptedHandoffPhase : std::uint8_t
{
  kAwaitingEpoch,
  kAwaitingAcceptedHandle,
  kWaitingForPendingRoutes,
  kWaitingForPriorPumpLease,
  kPublishingAcceptedEnvelope,
  kAcceptedEnvelopeCommitted,
  kPublishingDeferredControl,
  kReadyToActivate,
  kCleanupOnly,
};

enum class PendingHandoffWorkKind : std::uint8_t
{
  kPrepareAcceptedEnvelope,
  kPublishAcceptedEnvelope,
  kPublishDeferredControl,
};

enum class PendingRouteObservationRelation : std::uint8_t
{
  kNewDistinct,
  kExactDuplicate,
  kCollision,
};

enum class PendingRouteEvidenceResult : std::uint8_t
{
  kExact,
  kInvalid,
};

enum class PendingPreparationFailure : std::uint8_t
{
  kSteadyClockFailure,
  kCallbackOrAdapterFailure,
};

enum class PendingDeferredCompletion : std::uint8_t
{
  kDelivered,
  kInboxLossSecured,
  kInboxLossUnsecured,
};

struct PendingHandoffBindingKey
{
  CoordinatorGoalId goal_id{};
  GoalGeneration generation{0U};
  PendingBindingIncarnation incarnation{0U};

  [[nodiscard]] bool operator==(const PendingHandoffBindingKey &) const noexcept = default;
};

struct PendingHandoffLimits
{
  PendingRouteTokenId route_tokens{std::numeric_limits<PendingRouteTokenId>::max()};
  PendingRouteEvidenceRevision route_revisions{
    std::numeric_limits<PendingRouteEvidenceRevision>::max()};
  PendingHandoffTokenId handoff_tokens{std::numeric_limits<PendingHandoffTokenId>::max()};
};

struct PendingRouteWorkSnapshot
{
  PendingRouteKind kind{PendingRouteKind::kDrain};
  PendingRouteDeliveryClass delivery_class{PendingRouteDeliveryClass::kPreSeal};
  PendingRouteTokenId token{0U};
};

struct PendingHandoffWorkSnapshot
{
  PendingHandoffWorkKind kind{PendingHandoffWorkKind::kPrepareAcceptedEnvelope};
  PendingHandoffTokenId token{0U};
  PendingRouteEvidenceRevision prepared_revision{0U};
  // False denotes an unresolved external attempt retained only as cleanup evidence.
  bool capability_outstanding{false};
};

struct PendingAcceptedHandoffSnapshot
{
  PendingHandoffBindingKey binding{};
  PendingAcceptedHandoffPhase phase{PendingAcceptedHandoffPhase::kCleanupOnly};
  PendingRouteEvidenceRevision route_evidence_revision{0U};
  PendingRouteTokenId last_route_token{0U};
  std::optional<PendingRouteWorkSnapshot> route_work;
  std::optional<PendingHandoffWorkSnapshot> handoff_work;
};

struct PendingHandoffAdvanceFacts
{
  CoordinatorPumpLeaseGateSnapshot pump_gate{};
  bool deferred_control_occupied{false};
};

class PendingAcceptedHandoffMachine;

class PendingRouteWorkToken final
{
public:
  PendingRouteWorkToken() = delete;
  PendingRouteWorkToken(const PendingRouteWorkToken &) = delete;
  PendingRouteWorkToken(PendingRouteWorkToken &&) noexcept = default;
  PendingRouteWorkToken & operator=(const PendingRouteWorkToken &) = delete;
  PendingRouteWorkToken & operator=(PendingRouteWorkToken &&) = delete;
  ~PendingRouteWorkToken() = default;

  [[nodiscard]] const PendingHandoffBindingKey & binding() const noexcept {return binding_;}
  [[nodiscard]] PendingRouteKind kind() const noexcept {return kind_;}
  [[nodiscard]] PendingRouteDeliveryClass delivery_class() const noexcept
  {
    return delivery_class_;
  }
  [[nodiscard]] PendingRouteTokenId token() const noexcept {return token_;}
  [[nodiscard]] bool live() const noexcept {return live_.live();}

private:
  friend class PendingAcceptedHandoffMachine;
  PendingRouteWorkToken(
    PendingHandoffBindingKey binding, PendingRouteKind kind,
    PendingRouteDeliveryClass delivery_class, PendingRouteTokenId token) noexcept;

  PendingHandoffBindingKey binding_{};
  PendingRouteKind kind_{PendingRouteKind::kDrain};
  PendingRouteDeliveryClass delivery_class_{PendingRouteDeliveryClass::kPreSeal};
  PendingRouteTokenId token_{0U};
  CoordinatorOneShot live_;
};

class PendingHandoffWorkToken final
{
public:
  PendingHandoffWorkToken() = delete;
  PendingHandoffWorkToken(const PendingHandoffWorkToken &) = delete;
  PendingHandoffWorkToken(PendingHandoffWorkToken &&) noexcept = default;
  PendingHandoffWorkToken & operator=(const PendingHandoffWorkToken &) = delete;
  PendingHandoffWorkToken & operator=(PendingHandoffWorkToken &&) = delete;
  ~PendingHandoffWorkToken() = default;

  [[nodiscard]] const PendingHandoffBindingKey & binding() const noexcept {return binding_;}
  [[nodiscard]] PendingHandoffWorkKind kind() const noexcept {return kind_;}
  [[nodiscard]] PendingHandoffTokenId token() const noexcept {return token_;}
  [[nodiscard]] PendingRouteEvidenceRevision prepared_revision() const noexcept
  {
    return prepared_revision_;
  }
  [[nodiscard]] bool live() const noexcept {return live_.live();}

private:
  friend class PendingAcceptedHandoffMachine;
  PendingHandoffWorkToken(
    PendingHandoffBindingKey binding, PendingHandoffWorkKind kind,
    PendingHandoffTokenId token, PendingRouteEvidenceRevision prepared_revision) noexcept;

  PendingHandoffBindingKey binding_{};
  PendingHandoffWorkKind kind_{PendingHandoffWorkKind::kPrepareAcceptedEnvelope};
  PendingHandoffTokenId token_{0U};
  PendingRouteEvidenceRevision prepared_revision_{0U};
  CoordinatorOneShot live_;
};

enum class PendingHandoffPhaseEventStatus : std::uint8_t
{
  kApplied,
  kCleanupOnly,
};

enum class PendingRouteClaimStatus : std::uint8_t
{
  kClaimed,
  kCoalesced,
  kCleanupOnly,
};

class PendingRouteClaimDecision final
{
public:
  PendingRouteClaimDecision() = delete;
  PendingRouteClaimDecision(const PendingRouteClaimDecision &) = delete;
  PendingRouteClaimDecision(PendingRouteClaimDecision &&) noexcept = default;
  PendingRouteClaimDecision & operator=(const PendingRouteClaimDecision &) = delete;
  PendingRouteClaimDecision & operator=(PendingRouteClaimDecision &&) = delete;
  ~PendingRouteClaimDecision() = default;

  [[nodiscard]] PendingRouteClaimStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<PendingRouteWorkToken> take_work() noexcept;

private:
  friend class PendingAcceptedHandoffMachine;
  explicit PendingRouteClaimDecision(PendingRouteClaimStatus status) noexcept;
  explicit PendingRouteClaimDecision(PendingRouteWorkToken && work) noexcept;

  PendingRouteClaimStatus status_{PendingRouteClaimStatus::kCleanupOnly};
  std::optional<PendingRouteWorkToken> work_;
};

enum class PendingRouteCompletionStatus : std::uint8_t
{
  kMerged,
  kRetainCleanupEvidence,
  kCleanupOnly,
};

enum class PendingHandoffAdvanceStatus : std::uint8_t
{
  kWait,
  kPrepareAcceptedEnvelope,
  kPublishDeferredControl,
  kReadyToActivate,
  kCleanupOnly,
};

class PendingHandoffAdvanceDecision final
{
public:
  PendingHandoffAdvanceDecision() = delete;
  PendingHandoffAdvanceDecision(const PendingHandoffAdvanceDecision &) = delete;
  PendingHandoffAdvanceDecision(PendingHandoffAdvanceDecision &&) noexcept = default;
  PendingHandoffAdvanceDecision & operator=(const PendingHandoffAdvanceDecision &) = delete;
  PendingHandoffAdvanceDecision & operator=(PendingHandoffAdvanceDecision &&) = delete;
  ~PendingHandoffAdvanceDecision() = default;

  [[nodiscard]] PendingHandoffAdvanceStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<PendingHandoffWorkToken> take_work() noexcept;

private:
  friend class PendingAcceptedHandoffMachine;
  explicit PendingHandoffAdvanceDecision(PendingHandoffAdvanceStatus status) noexcept;
  PendingHandoffAdvanceDecision(
    PendingHandoffAdvanceStatus status, PendingHandoffWorkToken && work) noexcept;

  PendingHandoffAdvanceStatus status_{PendingHandoffAdvanceStatus::kCleanupOnly};
  std::optional<PendingHandoffWorkToken> work_;
};

enum class PendingPreparationCompletionStatus : std::uint8_t
{
  kPublishAcceptedEnvelope,
  kRetryPreparation,
  kRouteClaimed,
  kRouteCoalesced,
  kCleanupOnly,
};

class PendingPreparationCompletionDecision final
{
public:
  PendingPreparationCompletionDecision() = delete;
  PendingPreparationCompletionDecision(const PendingPreparationCompletionDecision &) = delete;
  PendingPreparationCompletionDecision(PendingPreparationCompletionDecision &&) noexcept =
    default;
  PendingPreparationCompletionDecision & operator=(
    const PendingPreparationCompletionDecision &) = delete;
  PendingPreparationCompletionDecision & operator=(
    PendingPreparationCompletionDecision &&) = delete;
  ~PendingPreparationCompletionDecision() = default;

  [[nodiscard]] PendingPreparationCompletionStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<PendingRouteWorkToken> take_route_work() noexcept;
  [[nodiscard]] std::optional<PendingHandoffWorkToken> take_handoff_work() noexcept;

private:
  friend class PendingAcceptedHandoffMachine;
  explicit PendingPreparationCompletionDecision(
    PendingPreparationCompletionStatus status) noexcept;
  PendingPreparationCompletionDecision(
    PendingPreparationCompletionStatus status, PendingRouteWorkToken && work) noexcept;
  PendingPreparationCompletionDecision(
    PendingPreparationCompletionStatus status, PendingHandoffWorkToken && work) noexcept;

  PendingPreparationCompletionStatus status_{
    PendingPreparationCompletionStatus::kCleanupOnly};
  std::optional<PendingRouteWorkToken> route_work_;
  std::optional<PendingHandoffWorkToken> handoff_work_;
};

enum class PendingAcceptedPublicationStatus : std::uint8_t
{
  kCommitted,
  kCleanupOnly,
};

enum class PendingDeferredPublicationStatus : std::uint8_t
{
  kCompleted,
  kCleanupOnly,
};

// This allocation-free reducer requires external synchronization by its owner. It owns only scalar
// protocol state; ROS handles, routed events, and diagnostics remain in the node binding.
class PendingAcceptedHandoffMachine final
{
public:
  explicit PendingAcceptedHandoffMachine(
    PendingHandoffBindingKey binding, PendingHandoffLimits limits = {}) noexcept;
  PendingAcceptedHandoffMachine(const PendingAcceptedHandoffMachine &) = delete;
  PendingAcceptedHandoffMachine(PendingAcceptedHandoffMachine &&) = delete;
  PendingAcceptedHandoffMachine & operator=(const PendingAcceptedHandoffMachine &) = delete;
  PendingAcceptedHandoffMachine & operator=(PendingAcceptedHandoffMachine &&) = delete;
  ~PendingAcceptedHandoffMachine() = default;

  [[nodiscard]] PendingHandoffPhaseEventStatus install_epoch(
    const PendingHandoffBindingKey & binding) noexcept;
  [[nodiscard]] PendingHandoffPhaseEventStatus adopt_accepted_handle(
    const PendingHandoffBindingKey & binding) noexcept;
  // relation is an authenticated comparison against node-owned sticky route metadata, computed by
  // the owner under the binding lock and bundled with claimed work before unlocking.
  [[nodiscard]] PendingRouteClaimDecision observe_route(
    const PendingHandoffBindingKey & binding, PendingRouteKind kind,
    PendingRouteObservationRelation relation) noexcept;
  // evidence and merge are authenticated facts derived from the router result and the node-owned
  // retained/deferred slot under the binding lock.
  [[nodiscard]] PendingRouteCompletionStatus complete_route(
    PendingRouteWorkToken & work, PendingRouteEvidenceResult evidence,
    PendingControlMergeDecision merge) noexcept;
  // deferred_control_occupied is sampled from the node-owned slot under the binding lock.
  [[nodiscard]] PendingHandoffAdvanceDecision advance(
    PendingHandoffAdvanceFacts facts) noexcept;
  // The success overload accepts no failure-only fact; the failure overload validates both closed
  // enums before it atomically exchanges preparation ownership for route ownership.
  [[nodiscard]] PendingPreparationCompletionDecision complete_preparation(
    PendingHandoffWorkToken & work) noexcept;
  [[nodiscard]] PendingPreparationCompletionDecision complete_preparation_failure(
    PendingHandoffWorkToken & work, PendingPreparationFailure failure,
    PendingRouteObservationRelation failure_relation) noexcept;
  [[nodiscard]] PendingAcceptedPublicationStatus complete_accepted_publication(
    PendingHandoffWorkToken & work, AcceptedEnvelopeDepositDecision decision) noexcept;
  [[nodiscard]] PendingDeferredPublicationStatus complete_deferred_publication(
    PendingHandoffWorkToken & work, PendingDeferredCompletion completion) noexcept;

  void enter_cleanup_only() noexcept;
  [[nodiscard]] PendingAcceptedHandoffSnapshot snapshot() const noexcept;
  [[nodiscard]] bool valid() const noexcept;

private:
  [[nodiscard]] bool binding_matches(const PendingHandoffBindingKey & binding) const noexcept;
  [[nodiscard]] bool route_token_matches(const PendingRouteWorkToken & work) const noexcept;
  [[nodiscard]] bool handoff_token_matches(const PendingHandoffWorkToken & work) const noexcept;
  [[nodiscard]] PendingRouteClaimDecision observe_route_impl(
    PendingRouteKind kind, PendingRouteObservationRelation relation) noexcept;
  [[nodiscard]] std::optional<PendingRouteWorkToken> issue_route(
    PendingRouteKind kind, PendingRouteDeliveryClass delivery_class) noexcept;
  [[nodiscard]] std::optional<PendingHandoffWorkToken> issue_handoff(
    PendingHandoffWorkKind kind, PendingRouteEvidenceRevision prepared_revision) noexcept;
  void consume_route(PendingRouteWorkToken & work) noexcept;
  void consume_handoff(PendingHandoffWorkToken & work) noexcept;
  void spend_handoff_token(PendingHandoffWorkToken & work) noexcept;
  void fail_if_invalid() noexcept;

  PendingHandoffBindingKey binding_{};
  PendingHandoffLimits limits_{};
  PendingAcceptedHandoffPhase phase_{PendingAcceptedHandoffPhase::kCleanupOnly};
  PendingRouteEvidenceRevision route_evidence_revision_{0U};
  PendingRouteTokenId last_route_token_{0U};
  PendingHandoffTokenId last_handoff_token_{0U};
  std::optional<PendingRouteWorkSnapshot> route_work_;
  std::optional<PendingHandoffWorkSnapshot> handoff_work_;
};

[[nodiscard]] constexpr bool valid_pending_handoff_binding_key(
  const PendingHandoffBindingKey & binding) noexcept
{
  return admissible_goal_generation(binding.generation) && binding.incarnation != 0U;
}

[[nodiscard]] constexpr bool valid_pending_handoff_phase(
  PendingAcceptedHandoffPhase phase) noexcept
{
  switch (phase) {
    case PendingAcceptedHandoffPhase::kAwaitingEpoch:
    case PendingAcceptedHandoffPhase::kAwaitingAcceptedHandle:
    case PendingAcceptedHandoffPhase::kWaitingForPendingRoutes:
    case PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease:
    case PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope:
    case PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted:
    case PendingAcceptedHandoffPhase::kPublishingDeferredControl:
    case PendingAcceptedHandoffPhase::kReadyToActivate:
    case PendingAcceptedHandoffPhase::kCleanupOnly:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_pending_handoff_work_kind(
  PendingHandoffWorkKind kind) noexcept
{
  switch (kind) {
    case PendingHandoffWorkKind::kPrepareAcceptedEnvelope:
    case PendingHandoffWorkKind::kPublishAcceptedEnvelope:
    case PendingHandoffWorkKind::kPublishDeferredControl:
      return true;
  }
  return false;
}

static_assert(std::is_trivially_copyable_v<PendingHandoffBindingKey>);
static_assert(std::is_trivially_copyable_v<PendingHandoffLimits>);
static_assert(std::is_trivially_copyable_v<PendingAcceptedHandoffSnapshot>);
}  // namespace restocker_task_executor
