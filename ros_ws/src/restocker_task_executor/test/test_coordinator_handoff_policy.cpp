// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string_view>
#include <utility>

#include "restocker_task_executor/coordinator_handoff_policy.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr std::array kDepositStatuses{
  CoordinatorInboxDepositStatus::kAccepted,
  CoordinatorInboxDepositStatus::kOverflowLatched,
  CoordinatorInboxDepositStatus::kInhibited,
  CoordinatorInboxDepositStatus::kEvidenceLost,
  CoordinatorInboxDepositStatus::kEvidenceConflict,
};

constexpr std::array kPersistenceStatuses{
  CoordinatorInboxPersistenceStatus::kEventOwned,
  CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned,
  CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned,
  CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned,
  CoordinatorInboxPersistenceStatus::kUnresolved,
};

constexpr std::array kRouteKinds{
  PendingRouteKind::kDrain,
  PendingRouteKind::kTransformAuthorityLoss,
  PendingRouteKind::kSteadyClockFailure,
  PendingRouteKind::kCallbackOrAdapterFailure,
};

[[nodiscard]] constexpr CoordinatorCleanShutdownFacts clean_shutdown_facts() noexcept
{
  CoordinatorCleanShutdownFacts facts;
  facts.diagnostic_caches_initialized = true;
  facts.diagnostic_caches_fresh = true;
  facts.startup_transport_quiescent = true;
  return facts;
}

using CleanShutdownFactMutator = void (*)(CoordinatorCleanShutdownFacts &);

TEST(CoordinatorHandoffPolicy, PumpSnapshotPredicatesExhaustStickyStateCombinations)
{
  for (const PumpLeaseAttempt attempt :
    {PumpLeaseAttempt{0U}, std::numeric_limits<PumpLeaseAttempt>::max()})
  {
    for (const bool outstanding : {false, true}) {
      for (const bool fail_stopped : {false, true}) {
        for (const bool exhausted : {false, true}) {
          const CoordinatorPumpLeaseGateSnapshot snapshot{attempt, outstanding, fail_stopped,
            exhausted};
          const bool healthy = !fail_stopped && !exhausted;
          EXPECT_EQ(pump_gate_allows_preaccept(snapshot), healthy);
          EXPECT_EQ(pump_gate_allows_handoff(snapshot), healthy && !outstanding);
          EXPECT_EQ(pump_gate_allows_clean_shutdown(snapshot), healthy && !outstanding);
        }
      }
    }
  }
}

TEST(CoordinatorHandoffPolicy, HealthyMaximumAttemptDoesNotImplyExhaustion)
{
  const CoordinatorPumpLeaseGateSnapshot healthy_maximum{
    std::numeric_limits<PumpLeaseAttempt>::max(), false, false, false};
  EXPECT_TRUE(pump_gate_allows_preaccept(healthy_maximum));
  EXPECT_TRUE(pump_gate_allows_handoff(healthy_maximum));
  EXPECT_TRUE(pump_gate_allows_clean_shutdown(healthy_maximum));
}

TEST(CoordinatorHandoffPolicy, CleanShutdownDefaultsFailClosedAndCompleteQuiescencePasses)
{
  EXPECT_FALSE(coordinator_allows_clean_shutdown(CoordinatorCleanShutdownFacts{}));
  EXPECT_TRUE(coordinator_allows_clean_shutdown(clean_shutdown_facts()));
}

TEST(CoordinatorHandoffPolicy, EveryCacheAuthorityAndNodeOccupancyFactBlocksCleanShutdown)
{
  constexpr std::array<std::pair<std::string_view, CleanShutdownFactMutator>, 14U> blockers{{
    {"uninitialized caches", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.diagnostic_caches_initialized = false;
      }},
    {"stale caches", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.diagnostic_caches_fresh = false;
      }},
    {"startup transport", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.startup_transport_quiescent = false;
      }},
    {"pending binding", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.pending_binding_exists = true;
      }},
    {"active binding", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.active_binding_exists = true;
      }},
    {"retained accepted handle",
      +[](CoordinatorCleanShutdownFacts & facts) {
        facts.retained_accepted_handle_occupied = true;
      }},
    {"orphaned accepted handle", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.orphaned_accepted_handle_count = 1U;
      }},
    {"pending route token", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.pending_route_token_live = true;
      }},
    {"handoff work token", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.handoff_work_token_live = true;
      }},
    {"retained control", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.retained_control_occupied = true;
      }},
    {"deferred control", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.deferred_control_occupied = true;
      }},
    {"preaccept failure", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.preaccept_transaction_failed = true;
      }},
    {"accepted-handle fail-stop",
      +[](CoordinatorCleanShutdownFacts & facts) {
        facts.accepted_handle_adoption_fail_stopped = true;
      }},
    {"adapter fail-stop", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.adapter_fail_stopped = true;
      }},
  }};

  for (const auto & [name, mutate] : blockers) {
    SCOPED_TRACE(name);
    auto facts = clean_shutdown_facts();
    mutate(facts);
    EXPECT_FALSE(coordinator_allows_clean_shutdown(facts));
  }

  auto maximum_orphans = clean_shutdown_facts();
  maximum_orphans.orphaned_accepted_handle_count = std::numeric_limits<std::size_t>::max();
  EXPECT_FALSE(coordinator_allows_clean_shutdown(maximum_orphans));
}

TEST(CoordinatorHandoffPolicy, EveryInboxOccupancyAndStickyEvidenceFactBlocksCleanShutdown)
{
  constexpr std::array<std::pair<std::string_view, CleanShutdownFactMutator>, 7U> blockers{{
    {"ordinary size", +[](CoordinatorCleanShutdownFacts & facts) {facts.inbox.size = 1U;}},
    {"accepted emergency", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.accepted_handoff_emergency_size = 1U;
      }},
    {"cleanup emergency", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.cleanup_emergency_size = 1U;
      }},
    {"overflow latch", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.overflow_latched = true;
      }},
    {"overflow notification", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.overflow_notification_pending = true;
      }},
    {"cleanup evidence loss", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.cleanup_evidence_lost = true;
      }},
    {"generation accounting conflict", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.inbox.generation_accounting_conflict = true;
      }},
  }};

  for (const auto & [name, mutate] : blockers) {
    SCOPED_TRACE(name);
    auto facts = clean_shutdown_facts();
    mutate(facts);
    EXPECT_FALSE(coordinator_allows_clean_shutdown(facts));
  }

  auto maximum_occupancy = clean_shutdown_facts();
  maximum_occupancy.inbox.size = std::numeric_limits<std::size_t>::max();
  maximum_occupancy.inbox.accepted_handoff_emergency_size =
    std::numeric_limits<std::size_t>::max();
  maximum_occupancy.inbox.cleanup_emergency_size = std::numeric_limits<std::size_t>::max();
  EXPECT_FALSE(coordinator_allows_clean_shutdown(maximum_occupancy));
}

TEST(CoordinatorHandoffPolicy, AdmissionMustBeKnownIdleAndMutationFree)
{
  for (const auto phase :
    {GoalSlotPhase::kIdle, GoalSlotPhase::kPendingAcceptance, GoalSlotPhase::kActive})
  {
    auto facts = clean_shutdown_facts();
    facts.admission.phase = phase;
    EXPECT_EQ(
      coordinator_allows_clean_shutdown(facts), phase == GoalSlotPhase::kIdle);
  }

  auto unknown = clean_shutdown_facts();
  unknown.admission.phase = static_cast<GoalSlotPhase>(0xffU);
  EXPECT_FALSE(coordinator_allows_clean_shutdown(unknown));
  EXPECT_FALSE(valid_cached_goal_slot_phase(unknown.admission.phase));

  auto inhibited = clean_shutdown_facts();
  inhibited.admission.inhibited = true;
  EXPECT_TRUE(coordinator_allows_clean_shutdown(inhibited));

  auto mutation = clean_shutdown_facts();
  mutation.admission.mutation_submission_occupied = true;
  EXPECT_FALSE(coordinator_allows_clean_shutdown(mutation));
}

TEST(CoordinatorHandoffPolicy, DriverInhibitionWithoutCapabilityAllowsCleanShutdown)
{
  auto inhibited = clean_shutdown_facts();
  inhibited.driver.inhibited = true;
  EXPECT_TRUE(coordinator_allows_clean_shutdown(inhibited));
}

TEST(CoordinatorHandoffPolicy, EveryDriverCapabilityOrActivityFactBlocksCleanShutdown)
{
  constexpr std::array<std::pair<std::string_view, CleanShutdownFactMutator>, 4U> blockers{{
    {"active", +[](CoordinatorCleanShutdownFacts & facts) {facts.driver.active = true;}},
    {"pending operation", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.driver.pending_operation_occupied = true;
      }},
    {"pending transport", +[](CoordinatorCleanShutdownFacts & facts) {
        facts.driver.pending_transport_requests = 1U;
      }},
    {"reservation capability",
      +[](CoordinatorCleanShutdownFacts & facts) {
        facts.driver.reservation_capability_may_remain = true;
      }},
  }};

  for (const auto & [name, mutate] : blockers) {
    SCOPED_TRACE(name);
    auto facts = clean_shutdown_facts();
    mutate(facts);
    EXPECT_FALSE(coordinator_allows_clean_shutdown(facts));
  }

  auto maximum_requests = clean_shutdown_facts();
  maximum_requests.driver.pending_transport_requests = std::numeric_limits<std::size_t>::max();
  EXPECT_FALSE(coordinator_allows_clean_shutdown(maximum_requests));
}

TEST(CoordinatorHandoffPolicy, CleanShutdownUsesPumpStickyBitsRatherThanAttemptMagnitude)
{
  for (const PumpLeaseAttempt attempt :
    {PumpLeaseAttempt{0U}, std::numeric_limits<PumpLeaseAttempt>::max()})
  {
    for (const bool outstanding : {false, true}) {
      for (const bool fail_stopped : {false, true}) {
        for (const bool exhausted : {false, true}) {
          auto facts = clean_shutdown_facts();
          facts.pump_lease = {attempt, outstanding, fail_stopped, exhausted};
          EXPECT_EQ(
            coordinator_allows_clean_shutdown(facts),
            !outstanding && !fail_stopped && !exhausted);
        }
      }
    }
  }
}

TEST(CoordinatorHandoffPolicy, AcceptedDepositClassifierExhaustsKnownPairs)
{
  for (const auto status : kDepositStatuses) {
    for (const auto persistence : kPersistenceStatuses) {
      const bool committed = persistence == CoordinatorInboxPersistenceStatus::kEventOwned &&
        (status == CoordinatorInboxDepositStatus::kAccepted ||
        status == CoordinatorInboxDepositStatus::kOverflowLatched);
      EXPECT_EQ(
        classify_accepted_envelope_deposit({status, persistence}),
        committed ? AcceptedEnvelopeDepositDecision::kCommitted :
        AcceptedEnvelopeDepositDecision::kCleanupOnly);
    }
  }
}

TEST(CoordinatorHandoffPolicy, AcceptedDepositClassifierRejectsUnknownEnums)
{
  constexpr auto unknown_status = static_cast<CoordinatorInboxDepositStatus>(0xffU);
  constexpr auto unknown_persistence = static_cast<CoordinatorInboxPersistenceStatus>(0xffU);
  for (const auto persistence : kPersistenceStatuses) {
    EXPECT_EQ(
      classify_accepted_envelope_deposit({unknown_status, persistence}),
      AcceptedEnvelopeDepositDecision::kCleanupOnly);
  }
  for (const auto status : kDepositStatuses) {
    EXPECT_EQ(
      classify_accepted_envelope_deposit({status, unknown_persistence}),
      AcceptedEnvelopeDepositDecision::kCleanupOnly);
  }
  EXPECT_EQ(
    classify_accepted_envelope_deposit({unknown_status, unknown_persistence}),
    AcceptedEnvelopeDepositDecision::kCleanupOnly);
}

TEST(CoordinatorHandoffPolicy, DeferredDepositClassifierExhaustsKnownPairs)
{
  for (const auto status : kDepositStatuses) {
    for (const auto persistence : kPersistenceStatuses) {
      DeferredControlDepositDecision expected =
        DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly;
      if (status == CoordinatorInboxDepositStatus::kAccepted &&
        persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
      {
        expected = DeferredControlDepositDecision::kDelivered;
      }
      if ((status == CoordinatorInboxDepositStatus::kOverflowLatched &&
        persistence == CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned) ||
        (status == CoordinatorInboxDepositStatus::kInhibited &&
        persistence == CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned))
      {
        expected = DeferredControlDepositDecision::kRecordInboxLossThenContinueIfSecured;
      }
      EXPECT_EQ(classify_deferred_control_deposit({status, persistence}), expected);
    }
  }
}

TEST(CoordinatorHandoffPolicy, DeferredDepositClassifierRejectsUnknownEnums)
{
  constexpr auto unknown_status = static_cast<CoordinatorInboxDepositStatus>(0xffU);
  constexpr auto unknown_persistence = static_cast<CoordinatorInboxPersistenceStatus>(0xffU);
  for (const auto persistence : kPersistenceStatuses) {
    EXPECT_EQ(
      classify_deferred_control_deposit({unknown_status, persistence}),
      DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly);
  }
  for (const auto status : kDepositStatuses) {
    EXPECT_EQ(
      classify_deferred_control_deposit({status, unknown_persistence}),
      DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly);
  }
  EXPECT_EQ(
    classify_deferred_control_deposit({unknown_status, unknown_persistence}),
    DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly);
}

TEST(CoordinatorHandoffPolicy, EmptyMergeSlotStoresEveryKnownIncomingKind)
{
  for (const auto delivery_class :
    {PendingRouteDeliveryClass::kPreSeal, PendingRouteDeliveryClass::kPostSeal})
  {
    for (const auto incoming : kRouteKinds) {
      EXPECT_EQ(
        classify_pending_control_merge(std::nullopt, incoming, delivery_class, false),
        PendingControlMergeDecision::kStoreIncoming);
      EXPECT_EQ(
        classify_pending_control_merge(std::nullopt, incoming, delivery_class, true),
        PendingControlMergeDecision::kCleanupOnly);
    }
  }
}

TEST(CoordinatorHandoffPolicy, ExactDuplicateKeepsOnlyMatchingExistingKind)
{
  for (const auto delivery_class :
    {PendingRouteDeliveryClass::kPreSeal, PendingRouteDeliveryClass::kPostSeal})
  {
    for (const auto existing : kRouteKinds) {
      for (const auto incoming : kRouteKinds) {
        EXPECT_EQ(
          classify_pending_control_merge(existing, incoming, delivery_class, true),
          existing == incoming ? PendingControlMergeDecision::kKeepExistingDisposeIncoming :
          PendingControlMergeDecision::kCleanupOnly);
      }
    }
  }
}

TEST(CoordinatorHandoffPolicy, AuthorityLossDominatesOnlyDistinctPreSealWeakerControl)
{
  for (const auto existing : kRouteKinds) {
    for (const auto incoming : kRouteKinds) {
      const bool replace = existing != PendingRouteKind::kTransformAuthorityLoss &&
        incoming == PendingRouteKind::kTransformAuthorityLoss;
      EXPECT_EQ(
        classify_pending_control_merge(
          existing, incoming,
          PendingRouteDeliveryClass::kPreSeal, false),
        replace ? PendingControlMergeDecision::kReplaceExistingDisposeDisplaced :
        PendingControlMergeDecision::kCleanupOnly);
    }
  }
}

TEST(CoordinatorHandoffPolicy, EveryDistinctPostSealMergeFailsClosed)
{
  for (const auto existing : kRouteKinds) {
    for (const auto incoming : kRouteKinds) {
      EXPECT_EQ(
        classify_pending_control_merge(
          existing, incoming,
          PendingRouteDeliveryClass::kPostSeal, false),
        PendingControlMergeDecision::kCleanupOnly);
    }
  }
}

TEST(CoordinatorHandoffPolicy, MergeClassifierRejectsUnknownEnums)
{
  constexpr auto unknown_kind = static_cast<PendingRouteKind>(0xffU);
  constexpr auto unknown_class = static_cast<PendingRouteDeliveryClass>(0xffU);
  EXPECT_EQ(
    classify_pending_control_merge(
      std::nullopt, unknown_kind,
      PendingRouteDeliveryClass::kPreSeal, false),
    PendingControlMergeDecision::kCleanupOnly);
  EXPECT_EQ(
    classify_pending_control_merge(
      unknown_kind, PendingRouteKind::kDrain,
      PendingRouteDeliveryClass::kPreSeal, false),
    PendingControlMergeDecision::kCleanupOnly);
  EXPECT_EQ(
    classify_pending_control_merge(std::nullopt, PendingRouteKind::kDrain, unknown_class, false),
    PendingControlMergeDecision::kCleanupOnly);
}

TEST(CoordinatorHandoffPolicy, MergeClassifierExhaustsKnownAndUnknownFacts)
{
  constexpr auto unknown_kind = static_cast<PendingRouteKind>(0xffU);
  constexpr auto unknown_class = static_cast<PendingRouteDeliveryClass>(0xffU);
  constexpr std::array<std::optional<PendingRouteKind>, 6U> existing_kinds{
    std::nullopt,
    PendingRouteKind::kDrain,
    PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteKind::kSteadyClockFailure,
    PendingRouteKind::kCallbackOrAdapterFailure,
    unknown_kind,
  };
  constexpr std::array incoming_kinds{
    PendingRouteKind::kDrain,
    PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteKind::kSteadyClockFailure,
    PendingRouteKind::kCallbackOrAdapterFailure,
    unknown_kind,
  };
  constexpr std::array delivery_classes{
    PendingRouteDeliveryClass::kPreSeal,
    PendingRouteDeliveryClass::kPostSeal,
    unknown_class,
  };

  for (const auto existing : existing_kinds) {
    for (const auto incoming : incoming_kinds) {
      for (const auto delivery_class : delivery_classes) {
        for (const bool exact_duplicate : {false, true}) {
          PendingControlMergeDecision expected = PendingControlMergeDecision::kCleanupOnly;
          const bool valid_existing = !existing || valid_pending_route_kind(*existing);
          if (valid_existing && valid_pending_route_kind(incoming) &&
            valid_pending_route_delivery_class(delivery_class))
          {
            if (!existing && !exact_duplicate) {
              expected = PendingControlMergeDecision::kStoreIncoming;
            }
            if (existing && exact_duplicate && *existing == incoming) {
              expected = PendingControlMergeDecision::kKeepExistingDisposeIncoming;
            }
            if (existing && !exact_duplicate &&
              delivery_class == PendingRouteDeliveryClass::kPreSeal &&
              incoming == PendingRouteKind::kTransformAuthorityLoss &&
              *existing != PendingRouteKind::kTransformAuthorityLoss)
            {
              expected = PendingControlMergeDecision::kReplaceExistingDisposeDisplaced;
            }
          }
          EXPECT_EQ(
            classify_pending_control_merge(existing, incoming, delivery_class, exact_duplicate),
            expected);
        }
      }
    }
  }
}

static_assert(pump_gate_allows_preaccept({0U, true, false, false}));
static_assert(!pump_gate_allows_handoff({0U, true, false, false}));
static_assert(
  classify_accepted_envelope_deposit(
    {CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned}) ==
  AcceptedEnvelopeDepositDecision::kCommitted);
static_assert(
  classify_deferred_control_deposit(
    {CoordinatorInboxDepositStatus::kInhibited,
      CoordinatorInboxPersistenceStatus::kUnresolved}) ==
  DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly);
static_assert(noexcept(pump_gate_allows_preaccept(CoordinatorPumpLeaseGateSnapshot{})));
static_assert(noexcept(classify_accepted_envelope_deposit(CoordinatorInboxDepositResult{})));
static_assert(noexcept(classify_deferred_control_deposit(CoordinatorInboxDepositResult{})));
static_assert(valid_cached_goal_slot_phase(GoalSlotPhase::kIdle));
static_assert(!valid_cached_goal_slot_phase(static_cast<GoalSlotPhase>(0xffU)));
static_assert(coordinator_allows_clean_shutdown(clean_shutdown_facts()));
static_assert(!coordinator_allows_clean_shutdown(CoordinatorCleanShutdownFacts{}));
static_assert(
  noexcept(coordinator_allows_clean_shutdown(CoordinatorCleanShutdownFacts{})));
static_assert(
  noexcept(classify_pending_control_merge(
    std::nullopt, PendingRouteKind::kDrain,
    PendingRouteDeliveryClass::kPreSeal, false)));

}  // namespace
}  // namespace restocker_task_executor
