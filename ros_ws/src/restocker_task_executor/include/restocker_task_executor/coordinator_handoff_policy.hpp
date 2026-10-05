// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "restocker_task_executor/coordinator_inbox_deposit_result.hpp"
#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"
#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

// These predicates read sticky failure bits, not the attempt value. A healthy gate at the maximum
// attempt stays usable for handoff and clean shutdown until another acquisition fails.
[[nodiscard]] constexpr bool pump_gate_allows_preaccept(
  const CoordinatorPumpLeaseGateSnapshot & snapshot) noexcept
{
  return !snapshot.fail_stopped && !snapshot.attempt_exhausted;
}

[[nodiscard]] constexpr bool pump_gate_allows_handoff(
  const CoordinatorPumpLeaseGateSnapshot & snapshot) noexcept
{
  return pump_gate_allows_preaccept(snapshot) && !snapshot.outstanding;
}

[[nodiscard]] constexpr bool pump_gate_allows_clean_shutdown(
  const CoordinatorPumpLeaseGateSnapshot & snapshot) noexcept
{
  return pump_gate_allows_handoff(snapshot);
}

// Owning inbox, admission, and driver snapshots contain strings, shared ownership, or optional
// operation records. The node derives these scalar-only observations outside its binding lock and
// publishes them under that lock. They are observations, not capabilities.
struct CachedCoordinatorInboxSnapshot
{
  std::size_t size{0U};
  std::size_t accepted_handoff_emergency_size{0U};
  std::size_t cleanup_emergency_size{0U};
  bool overflow_latched{false};
  bool overflow_notification_pending{false};
  bool cleanup_evidence_lost{false};
  bool generation_accounting_conflict{false};
};

struct CachedGoalAdmissionSnapshot
{
  GoalSlotPhase phase{GoalSlotPhase::kIdle};
  bool inhibited{false};
  bool mutation_submission_occupied{false};
};

struct CachedRestockCoordinatorDriverSnapshot
{
  bool active{false};
  bool inhibited{false};
  bool pending_operation_occupied{false};
  std::size_t pending_transport_requests{0U};
  bool reservation_capability_may_remain{false};
};

// Complete scalar input to clean-shutdown classification. The cache and startup bits default false
// so a default-constructed or partially initialized observation fails closed; the other scalar
// snapshots default to their quiescent values.
struct CoordinatorCleanShutdownFacts
{
  bool diagnostic_caches_initialized{false};
  bool diagnostic_caches_fresh{false};
  bool startup_transport_quiescent{false};
  CoordinatorPumpLeaseGateSnapshot pump_lease;

  bool pending_binding_exists{false};
  bool active_binding_exists{false};
  bool retained_accepted_handle_occupied{false};
  std::size_t orphaned_accepted_handle_count{0U};
  bool pending_route_token_live{false};
  bool handoff_work_token_live{false};
  bool retained_control_occupied{false};
  bool deferred_control_occupied{false};

  bool preaccept_transaction_failed{false};
  bool accepted_handle_adoption_fail_stopped{false};
  bool adapter_fail_stopped{false};

  CachedCoordinatorInboxSnapshot inbox;
  CachedGoalAdmissionSnapshot admission;
  CachedRestockCoordinatorDriverSnapshot driver;
};

[[nodiscard]] constexpr bool valid_cached_goal_slot_phase(GoalSlotPhase phase) noexcept
{
  switch (phase) {
    case GoalSlotPhase::kIdle:
    case GoalSlotPhase::kPendingAcceptance:
    case GoalSlotPhase::kActive:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool coordinator_allows_clean_shutdown(
  const CoordinatorCleanShutdownFacts & facts) noexcept
{
  if (!facts.diagnostic_caches_initialized || !facts.diagnostic_caches_fresh ||
    !facts.startup_transport_quiescent ||
    !pump_gate_allows_clean_shutdown(facts.pump_lease))
  {
    return false;
  }

  if (facts.pending_binding_exists || facts.active_binding_exists ||
    facts.retained_accepted_handle_occupied || facts.orphaned_accepted_handle_count != 0U ||
    facts.pending_route_token_live || facts.handoff_work_token_live ||
    facts.retained_control_occupied || facts.deferred_control_occupied ||
    facts.preaccept_transaction_failed || facts.accepted_handle_adoption_fail_stopped ||
    facts.adapter_fail_stopped)
  {
    return false;
  }

  if (facts.inbox.size != 0U || facts.inbox.accepted_handoff_emergency_size != 0U ||
    facts.inbox.cleanup_emergency_size != 0U ||
    facts.inbox.overflow_latched || facts.inbox.overflow_notification_pending ||
    facts.inbox.cleanup_evidence_lost || facts.inbox.generation_accounting_conflict)
  {
    return false;
  }

  if (!valid_cached_goal_slot_phase(facts.admission.phase) ||
    facts.admission.phase != GoalSlotPhase::kIdle ||
    facts.admission.mutation_submission_occupied)
  {
    return false;
  }

  return !facts.driver.active && !facts.driver.pending_operation_occupied &&
         facts.driver.pending_transport_requests == 0U &&
         !facts.driver.reservation_capability_may_remain;
}

enum class AcceptedEnvelopeDepositDecision : std::uint8_t
{
  kCommitted,
  kCleanupOnly,
};

[[nodiscard]] constexpr AcceptedEnvelopeDepositDecision classify_accepted_envelope_deposit(
  CoordinatorInboxDepositResult result) noexcept
{
  if (result.persistence != CoordinatorInboxPersistenceStatus::kEventOwned) {
    return AcceptedEnvelopeDepositDecision::kCleanupOnly;
  }
  switch (result.status) {
    case CoordinatorInboxDepositStatus::kAccepted:
    case CoordinatorInboxDepositStatus::kOverflowLatched:
      return AcceptedEnvelopeDepositDecision::kCommitted;
    case CoordinatorInboxDepositStatus::kInhibited:
    case CoordinatorInboxDepositStatus::kEvidenceLost:
    case CoordinatorInboxDepositStatus::kEvidenceConflict:
      return AcceptedEnvelopeDepositDecision::kCleanupOnly;
  }
  return AcceptedEnvelopeDepositDecision::kCleanupOnly;
}

enum class DeferredControlDepositDecision : std::uint8_t
{
  kDelivered,
  kRecordInboxLossThenContinueIfSecured,
  kRecordInboxLossThenCleanupOnly,
};

[[nodiscard]] constexpr DeferredControlDepositDecision classify_deferred_control_deposit(
  CoordinatorInboxDepositResult result) noexcept
{
  if (result.status == CoordinatorInboxDepositStatus::kAccepted &&
    result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
  {
    return DeferredControlDepositDecision::kDelivered;
  }
  if ((result.status == CoordinatorInboxDepositStatus::kOverflowLatched &&
    result.persistence == CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned) ||
    (result.status == CoordinatorInboxDepositStatus::kInhibited &&
    result.persistence == CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned))
  {
    return DeferredControlDepositDecision::kRecordInboxLossThenContinueIfSecured;
  }
  return DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly;
}

enum class PendingRouteKind : std::uint8_t
{
  kDrain,
  kTransformAuthorityLoss,
  kSteadyClockFailure,
  kCallbackOrAdapterFailure,
};

enum class PendingRouteDeliveryClass : std::uint8_t
{
  kPreSeal,
  kPostSeal,
};

enum class PendingControlMergeDecision : std::uint8_t
{
  kStoreIncoming,
  kKeepExistingDisposeIncoming,
  kReplaceExistingDisposeDisplaced,
  kCleanupOnly,
};

[[nodiscard]] constexpr bool valid_pending_route_kind(PendingRouteKind kind) noexcept
{
  switch (kind) {
    case PendingRouteKind::kDrain:
    case PendingRouteKind::kTransformAuthorityLoss:
    case PendingRouteKind::kSteadyClockFailure:
    case PendingRouteKind::kCallbackOrAdapterFailure:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_pending_route_delivery_class(
  PendingRouteDeliveryClass delivery_class) noexcept
{
  switch (delivery_class) {
    case PendingRouteDeliveryClass::kPreSeal:
    case PendingRouteDeliveryClass::kPostSeal:
      return true;
  }
  return false;
}

// exact_duplicate is an authenticated fact supplied only after the caller proves semantic tuple,
// immutable first-record pointer, and event equality. Inconsistent claims fail closed.
[[nodiscard]] constexpr PendingControlMergeDecision classify_pending_control_merge(
  std::optional<PendingRouteKind> existing_kind, PendingRouteKind incoming_kind,
  PendingRouteDeliveryClass delivery_class, bool exact_duplicate) noexcept
{
  if (!valid_pending_route_kind(incoming_kind) ||
    !valid_pending_route_delivery_class(delivery_class) ||
    (existing_kind && !valid_pending_route_kind(*existing_kind)))
  {
    return PendingControlMergeDecision::kCleanupOnly;
  }
  if (!existing_kind) {
    return exact_duplicate ? PendingControlMergeDecision::kCleanupOnly :
           PendingControlMergeDecision::kStoreIncoming;
  }
  if (exact_duplicate) {
    return *existing_kind == incoming_kind ?
           PendingControlMergeDecision::kKeepExistingDisposeIncoming :
           PendingControlMergeDecision::kCleanupOnly;
  }
  if (delivery_class == PendingRouteDeliveryClass::kPreSeal &&
    incoming_kind == PendingRouteKind::kTransformAuthorityLoss &&
    *existing_kind != PendingRouteKind::kTransformAuthorityLoss)
  {
    return PendingControlMergeDecision::kReplaceExistingDisposeDisplaced;
  }
  return PendingControlMergeDecision::kCleanupOnly;
}

static_assert(std::is_trivially_copyable_v<AcceptedEnvelopeDepositDecision>);
static_assert(std::is_trivially_copyable_v<DeferredControlDepositDecision>);
static_assert(std::is_trivially_copyable_v<PendingRouteKind>);
static_assert(std::is_trivially_copyable_v<PendingRouteDeliveryClass>);
static_assert(std::is_trivially_copyable_v<PendingControlMergeDecision>);
static_assert(std::is_trivially_copyable_v<CachedCoordinatorInboxSnapshot>);
static_assert(std::is_trivially_copyable_v<CachedGoalAdmissionSnapshot>);
static_assert(std::is_trivially_copyable_v<CachedRestockCoordinatorDriverSnapshot>);
static_assert(std::is_trivially_copyable_v<CoordinatorCleanShutdownFacts>);

}  // namespace restocker_task_executor
