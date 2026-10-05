// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_handoff_reducer.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr PendingHandoffBindingKey binding(
  std::uint8_t marker = 1U, GoalGeneration generation = 1U,
  PendingBindingIncarnation incarnation = 1U)
{
  CoordinatorGoalId id{};
  id.front() = marker;
  return PendingHandoffBindingKey{id, generation, incarnation};
}

constexpr CoordinatorPumpLeaseGateSnapshot healthy_gate(bool outstanding = false)
{
  return CoordinatorPumpLeaseGateSnapshot{1U, outstanding, false, false};
}

void install_and_adopt(
  PendingAcceptedHandoffMachine & machine,
  const PendingHandoffBindingKey & key)
{
  ASSERT_EQ(machine.install_epoch(key), PendingHandoffPhaseEventStatus::kApplied);
  ASSERT_EQ(machine.adopt_accepted_handle(key), PendingHandoffPhaseEventStatus::kApplied);
  ASSERT_TRUE(machine.valid());
}

std::optional<PendingHandoffWorkToken> take_prepare(PendingAcceptedHandoffMachine & machine)
{
  auto decision = machine.advance({healthy_gate(), false});
  EXPECT_EQ(decision.status(), PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope);
  auto work = decision.take_work();
  EXPECT_TRUE(work);
  EXPECT_FALSE(decision.take_work());
  return work;
}

std::optional<PendingHandoffWorkToken> reach_accepted_publication(
  PendingAcceptedHandoffMachine & machine, const PendingHandoffBindingKey & key)
{
  install_and_adopt(machine, key);
  auto prepare = take_prepare(machine);
  if (!prepare) {
    return std::nullopt;
  }
  auto completion = machine.complete_preparation(*prepare);
  EXPECT_EQ(
    completion.status(), PendingPreparationCompletionStatus::kPublishAcceptedEnvelope);
  EXPECT_FALSE(prepare->live());
  auto publication = completion.take_handoff_work();
  EXPECT_FALSE(completion.take_handoff_work());
  return publication;
}

void commit_accepted(
  PendingAcceptedHandoffMachine & machine, PendingHandoffWorkToken & publication)
{
  ASSERT_EQ(
    machine.complete_accepted_publication(
      publication, AcceptedEnvelopeDepositDecision::kCommitted),
    PendingAcceptedPublicationStatus::kCommitted);
  ASSERT_FALSE(publication.live());
  ASSERT_TRUE(machine.valid());
}

TEST(PendingHandoffCounterTest, CheckedSuccessorsReserveZeroAndPermitMaximum)
{
  constexpr auto maximum = std::numeric_limits<std::uint64_t>::max();
  constexpr std::array<std::uint64_t, 3U> ordinary{0U, 41U, maximum - 1U};
  for (const auto value : ordinary) {
    ASSERT_TRUE(checked_next_pending_binding_incarnation(value));
    EXPECT_EQ(*checked_next_pending_binding_incarnation(value), value + 1U);
    ASSERT_TRUE(checked_next_pending_route_token(value));
    EXPECT_EQ(*checked_next_pending_route_token(value), value + 1U);
    ASSERT_TRUE(checked_next_pending_route_evidence_revision(value));
    EXPECT_EQ(*checked_next_pending_route_evidence_revision(value), value + 1U);
    ASSERT_TRUE(checked_next_pending_handoff_token(value));
    EXPECT_EQ(*checked_next_pending_handoff_token(value), value + 1U);
  }
  EXPECT_FALSE(checked_next_pending_binding_incarnation(maximum));
  EXPECT_FALSE(checked_next_pending_route_token(maximum));
  EXPECT_FALSE(checked_next_pending_route_evidence_revision(maximum));
  EXPECT_FALSE(checked_next_pending_handoff_token(maximum));
}

TEST(PendingHandoffTypeTest, CapabilitiesAreSealedMoveOnlyAndSnapshotsAreValues)
{
  static_assert(!std::is_default_constructible_v<PendingRouteWorkToken>);
  static_assert(!std::is_copy_constructible_v<PendingRouteWorkToken>);
  static_assert(!std::is_copy_assignable_v<PendingRouteWorkToken>);
  static_assert(std::is_nothrow_move_constructible_v<PendingRouteWorkToken>);
  static_assert(!std::is_move_assignable_v<PendingRouteWorkToken>);
  static_assert(std::is_nothrow_destructible_v<PendingRouteWorkToken>);
  static_assert(!std::is_default_constructible_v<PendingHandoffWorkToken>);
  static_assert(!std::is_copy_constructible_v<PendingHandoffWorkToken>);
  static_assert(!std::is_copy_assignable_v<PendingHandoffWorkToken>);
  static_assert(std::is_nothrow_move_constructible_v<PendingHandoffWorkToken>);
  static_assert(!std::is_move_assignable_v<PendingHandoffWorkToken>);
  static_assert(std::is_nothrow_destructible_v<PendingHandoffWorkToken>);
  static_assert(std::is_trivially_copyable_v<PendingAcceptedHandoffSnapshot>);
  static_assert(noexcept(std::declval<PendingAcceptedHandoffMachine &>().snapshot()));
  static_assert(noexcept(std::declval<PendingAcceptedHandoffMachine &>().valid()));
  SUCCEED();
}

TEST(PendingHandoffConstructionTest, InvalidBindingStartsAbsorbingCleanupOnly)
{
  for (const auto key : std::array{
        binding(1U, 0U, 1U), binding(1U, kReservedCoordinatorGoalGeneration, 1U),
        binding(1U, 1U, 0U)})
  {
    PendingAcceptedHandoffMachine machine(key);
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
    EXPECT_FALSE(machine.valid());
    EXPECT_EQ(machine.install_epoch(key), PendingHandoffPhaseEventStatus::kCleanupOnly);
  }
}

TEST(PendingHandoffPhaseTest, HappyPathPublishesAcceptedThenActivates)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine machine(key);
  auto publication = reach_accepted_publication(machine, key);
  ASSERT_TRUE(publication);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kPublishingAcceptedEnvelope);
  commit_accepted(machine, *publication);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted);

  auto ready = machine.advance({healthy_gate(), false});
  EXPECT_EQ(ready.status(), PendingHandoffAdvanceStatus::kReadyToActivate);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kReadyToActivate);
  EXPECT_EQ(
    machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kReadyToActivate);
}

TEST(PendingHandoffPhaseTest, DeferredControlCompletesBeforeActivation)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine machine(key);
  auto publication = reach_accepted_publication(machine, key);
  ASSERT_TRUE(publication);
  commit_accepted(machine, *publication);

  auto deferred = machine.advance({healthy_gate(), true});
  EXPECT_EQ(deferred.status(), PendingHandoffAdvanceStatus::kPublishDeferredControl);
  auto work = deferred.take_work();
  ASSERT_TRUE(work);
  EXPECT_EQ(work->kind(), PendingHandoffWorkKind::kPublishDeferredControl);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kPublishingDeferredControl);
  EXPECT_EQ(
    machine.complete_deferred_publication(*work, PendingDeferredCompletion::kDelivered),
    PendingDeferredPublicationStatus::kCompleted);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted);
  EXPECT_EQ(
    machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kReadyToActivate);
}

TEST(PendingHandoffIdentityTest, WrongIdentityFailsClosedAtEachBindingEvent)
{
  const auto key = binding();
  for (const auto wrong :
    std::array{binding(2U, 1U, 1U), binding(1U, 2U, 1U), binding(1U, 1U, 2U)})
  {
    PendingAcceptedHandoffMachine epoch_machine(key);
    EXPECT_EQ(
      epoch_machine.install_epoch(wrong), PendingHandoffPhaseEventStatus::kCleanupOnly);
    EXPECT_EQ(epoch_machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);

    PendingAcceptedHandoffMachine handle_machine(key);
    ASSERT_EQ(
      handle_machine.install_epoch(key), PendingHandoffPhaseEventStatus::kApplied);
    EXPECT_EQ(
      handle_machine.adopt_accepted_handle(wrong),
      PendingHandoffPhaseEventStatus::kCleanupOnly);
    EXPECT_EQ(handle_machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
  }
}

TEST(PendingHandoffPumpTest, StickyFailurePrecedesOutstandingAndMaximumIsNotFailure)
{
  const auto key = binding();
  for (const auto gate : std::array{
        CoordinatorPumpLeaseGateSnapshot{1U, false, true, false},
        CoordinatorPumpLeaseGateSnapshot{1U, true, true, false},
        CoordinatorPumpLeaseGateSnapshot{1U, false, false, true},
        CoordinatorPumpLeaseGateSnapshot{1U, true, false, true}})
  {
    PendingAcceptedHandoffMachine machine(key);
    install_and_adopt(machine, key);
    EXPECT_EQ(
      machine.advance({gate, false}).status(), PendingHandoffAdvanceStatus::kCleanupOnly);
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
  }

  PendingAcceptedHandoffMachine waiting(key);
  install_and_adopt(waiting, key);
  EXPECT_EQ(
    waiting.advance({healthy_gate(true), false}).status(),
    PendingHandoffAdvanceStatus::kWait);
  EXPECT_EQ(
    waiting.snapshot().phase, PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease);

  PendingAcceptedHandoffMachine maximum(key);
  install_and_adopt(maximum, key);
  const CoordinatorPumpLeaseGateSnapshot max_gate{
    std::numeric_limits<PumpLeaseAttempt>::max(), false, false, false};
  EXPECT_EQ(
    maximum.advance({max_gate, false}).status(),
    PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope);
}

TEST(PendingHandoffPumpTest, OutstandingLeaseAfterSealFailsClosedBeforeDeferredOrActivation)
{
  const auto key = binding();
  for (const bool deferred : {false, true}) {
    PendingAcceptedHandoffMachine machine(key);
    auto publication = reach_accepted_publication(machine, key);
    ASSERT_TRUE(publication);
    commit_accepted(machine, *publication);
    EXPECT_EQ(
      machine.advance({healthy_gate(true), deferred}).status(),
      PendingHandoffAdvanceStatus::kCleanupOnly);
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
    EXPECT_FALSE(machine.snapshot().handoff_work);
  }

  PendingAcceptedHandoffMachine ready_machine(key);
  auto publication = reach_accepted_publication(ready_machine, key);
  ASSERT_TRUE(publication);
  commit_accepted(ready_machine, *publication);
  ASSERT_EQ(
    ready_machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kReadyToActivate);
  EXPECT_EQ(
    ready_machine.advance({healthy_gate(true), false}).status(),
    PendingHandoffAdvanceStatus::kCleanupOnly);
}

TEST(PendingHandoffRouteTest, NewClaimsAreClassifiedBySealAndExactDuplicatesCoalesce)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine pre(key);
  auto first = pre.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  EXPECT_EQ(first.status(), PendingRouteClaimStatus::kClaimed);
  auto first_work = first.take_work();
  ASSERT_TRUE(first_work);
  EXPECT_EQ(first_work->delivery_class(), PendingRouteDeliveryClass::kPreSeal);
  const auto before_duplicate = pre.snapshot();
  EXPECT_EQ(
    pre.observe_route(
      key, PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kExactDuplicate).status(),
    PendingRouteClaimStatus::kCoalesced);
  EXPECT_EQ(pre.snapshot().last_route_token, before_duplicate.last_route_token);
  EXPECT_EQ(
    pre.snapshot().route_evidence_revision, before_duplicate.route_evidence_revision);

  PendingAcceptedHandoffMachine post(key);
  auto publication = reach_accepted_publication(post, key);
  ASSERT_TRUE(publication);
  auto post_route = post.observe_route(
    key, PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteObservationRelation::kNewDistinct);
  auto post_work = post_route.take_work();
  ASSERT_TRUE(post_work);
  EXPECT_EQ(post_work->delivery_class(), PendingRouteDeliveryClass::kPostSeal);
}

TEST(PendingHandoffRouteTest, DistinctSecondRouteAndWrongIdentityFailClosed)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine collision(key);
  auto claim = collision.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  ASSERT_TRUE(claim.take_work());
  EXPECT_EQ(
    collision.observe_route(
      key, PendingRouteKind::kSteadyClockFailure,
      PendingRouteObservationRelation::kCollision).status(),
    PendingRouteClaimStatus::kCleanupOnly);
  EXPECT_TRUE(collision.snapshot().route_work.has_value());

  PendingAcceptedHandoffMachine foreign(key);
  EXPECT_EQ(
    foreign.observe_route(
      binding(1U, 1U, 2U), PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kNewDistinct).status(),
    PendingRouteClaimStatus::kCleanupOnly);
}

TEST(PendingHandoffRouteTest, UnknownInputsAndForbiddenPhasesFailClosed)
{
  const auto key = binding();
  const auto unknown_kind = static_cast<PendingRouteKind>(0xffU);
  const auto unknown_relation = static_cast<PendingRouteObservationRelation>(0xffU);
  for (const auto [kind, relation] : std::array{
        std::pair{unknown_kind, PendingRouteObservationRelation::kNewDistinct},
        std::pair{PendingRouteKind::kDrain, unknown_relation}})
  {
    PendingAcceptedHandoffMachine machine(key);
    EXPECT_EQ(
      machine.observe_route(key, kind, relation).status(),
      PendingRouteClaimStatus::kCleanupOnly);
  }

  PendingAcceptedHandoffMachine deferred_machine(key);
  auto publication = reach_accepted_publication(deferred_machine, key);
  ASSERT_TRUE(publication);
  commit_accepted(deferred_machine, *publication);
  auto deferred = deferred_machine.advance({healthy_gate(), true});
  ASSERT_TRUE(deferred.take_work());
  EXPECT_EQ(
    deferred_machine.observe_route(
      key, PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kNewDistinct).status(),
    PendingRouteClaimStatus::kCleanupOnly);

  PendingAcceptedHandoffMachine ready_machine(key);
  auto ready_publication = reach_accepted_publication(ready_machine, key);
  ASSERT_TRUE(ready_publication);
  commit_accepted(ready_machine, *ready_publication);
  ASSERT_EQ(
    ready_machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kReadyToActivate);
  EXPECT_EQ(
    ready_machine.observe_route(
      key, PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kNewDistinct).status(),
    PendingRouteClaimStatus::kCleanupOnly);
}

TEST(PendingHandoffRouteTest, PreSealCompletionSurvivesEveryLegalPhaseDrift)
{
  const auto key = binding();
  for (std::uint8_t target = 0U; target < 4U; ++target) {
    PendingAcceptedHandoffMachine machine(key);
    auto claim = machine.observe_route(
      key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
    auto work = claim.take_work();
    ASSERT_TRUE(work);
    if (target >= 1U) {
      ASSERT_EQ(machine.install_epoch(key), PendingHandoffPhaseEventStatus::kApplied);
    }
    if (target >= 2U) {
      ASSERT_EQ(
        machine.adopt_accepted_handle(key), PendingHandoffPhaseEventStatus::kApplied);
    }
    if (target >= 3U) {
      ASSERT_EQ(
        machine.advance({healthy_gate(true), false}).status(),
        PendingHandoffAdvanceStatus::kWait);
    }
    EXPECT_EQ(
      machine.complete_route(
        *work, PendingRouteEvidenceResult::kExact,
        PendingControlMergeDecision::kStoreIncoming),
      PendingRouteCompletionStatus::kMerged);
    EXPECT_FALSE(work->live());
    EXPECT_TRUE(machine.valid());
  }
}

TEST(PendingHandoffRouteTest, PostSealCompletionSurvivesAcceptedCommit)
{
  const auto key = binding();
  for (const bool commit_first : {false, true}) {
    PendingAcceptedHandoffMachine machine(key);
    auto publication = reach_accepted_publication(machine, key);
    ASSERT_TRUE(publication);
    auto claim = machine.observe_route(
      key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
    auto route = claim.take_work();
    ASSERT_TRUE(route);
    if (commit_first) {
      commit_accepted(machine, *publication);
    }
    EXPECT_EQ(
      machine.complete_route(
        *route, PendingRouteEvidenceResult::kExact,
        PendingControlMergeDecision::kStoreIncoming),
      PendingRouteCompletionStatus::kMerged);
    if (!commit_first) {
      commit_accepted(machine, *publication);
    }
    EXPECT_TRUE(machine.valid());
  }
}

TEST(PendingHandoffRouteTest, InvalidEvidenceConsumesExactWorkAndEntersCleanup)
{
  const auto key = binding();
  for (const auto evidence : {PendingRouteEvidenceResult::kInvalid,
      static_cast<PendingRouteEvidenceResult>(0xffU)})
  {
    PendingAcceptedHandoffMachine machine(key);
    auto claim = machine.observe_route(
      key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
    auto work = claim.take_work();
    ASSERT_TRUE(work);
    EXPECT_EQ(
      machine.complete_route(
        *work, evidence, PendingControlMergeDecision::kStoreIncoming),
      PendingRouteCompletionStatus::kCleanupOnly);
    EXPECT_FALSE(work->live());
    EXPECT_FALSE(machine.snapshot().route_work);
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
  }
}

TEST(PendingHandoffRouteTest, ReplacementRequiresPreSealAuthorityLoss)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine pre_drain(key);
  auto drain_claim = pre_drain.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto drain = drain_claim.take_work();
  ASSERT_TRUE(drain);
  EXPECT_EQ(
    pre_drain.complete_route(
      *drain, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kReplaceExistingDisposeDisplaced),
    PendingRouteCompletionStatus::kCleanupOnly);

  PendingAcceptedHandoffMachine pre_authority(key);
  auto authority_claim = pre_authority.observe_route(
    key, PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteObservationRelation::kNewDistinct);
  auto authority = authority_claim.take_work();
  ASSERT_TRUE(authority);
  EXPECT_EQ(
    pre_authority.complete_route(
      *authority, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kReplaceExistingDisposeDisplaced),
    PendingRouteCompletionStatus::kMerged);

  PendingAcceptedHandoffMachine post_authority(key);
  auto publication = reach_accepted_publication(post_authority, key);
  ASSERT_TRUE(publication);
  auto post_claim = post_authority.observe_route(
    key, PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteObservationRelation::kNewDistinct);
  auto post = post_claim.take_work();
  ASSERT_TRUE(post);
  EXPECT_EQ(
    post_authority.complete_route(
      *post, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kReplaceExistingDisposeDisplaced),
    PendingRouteCompletionStatus::kCleanupOnly);
}

TEST(PendingHandoffRouteTest, SettledDuplicateDoesNotAdvanceButNewEvidenceDoes)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine machine(key);
  auto first_claim = machine.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto first = first_claim.take_work();
  ASSERT_TRUE(first);
  ASSERT_EQ(
    machine.complete_route(
      *first, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kMerged);
  const auto settled = machine.snapshot();
  EXPECT_EQ(
    machine.observe_route(
      key, PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kExactDuplicate).status(),
    PendingRouteClaimStatus::kCoalesced);
  EXPECT_EQ(machine.snapshot().last_route_token, settled.last_route_token);
  EXPECT_EQ(machine.snapshot().route_evidence_revision, settled.route_evidence_revision);

  auto second_claim = machine.observe_route(
    key, PendingRouteKind::kTransformAuthorityLoss,
    PendingRouteObservationRelation::kNewDistinct);
  auto second = second_claim.take_work();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->token(), settled.last_route_token + 1U);
  EXPECT_EQ(
    machine.snapshot().route_evidence_revision, settled.route_evidence_revision + 1U);
}

TEST(PendingHandoffRouteTest, EveryKnownRouteKindCanClaimAndUnknownMergeFailsClosed)
{
  const auto key = binding();
  for (const auto kind : {PendingRouteKind::kDrain,
      PendingRouteKind::kTransformAuthorityLoss, PendingRouteKind::kSteadyClockFailure,
      PendingRouteKind::kCallbackOrAdapterFailure})
  {
    PendingAcceptedHandoffMachine machine(key);
    auto claim = machine.observe_route(
      key, kind, PendingRouteObservationRelation::kNewDistinct);
    auto work = claim.take_work();
    ASSERT_TRUE(work);
    EXPECT_EQ(work->kind(), kind);
    EXPECT_TRUE(machine.valid());
  }

  PendingAcceptedHandoffMachine unknown_merge(key);
  auto claim = unknown_merge.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto work = claim.take_work();
  ASSERT_TRUE(work);
  EXPECT_EQ(
    unknown_merge.complete_route(
      *work, PendingRouteEvidenceResult::kExact,
      static_cast<PendingControlMergeDecision>(0xffU)),
    PendingRouteCompletionStatus::kCleanupOnly);
}

TEST(PendingHandoffRouteTest, ForeignAndReplayCompletionsFailClosedWithoutStealingAuthority)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine owner(key);
  auto owner_claim = owner.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto owner_work = owner_claim.take_work();
  ASSERT_TRUE(owner_work);

  const auto foreign_key = binding(1U, 1U, 2U);
  PendingAcceptedHandoffMachine foreign(foreign_key);
  auto foreign_claim = foreign.observe_route(
    foreign_key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto foreign_work = foreign_claim.take_work();
  ASSERT_TRUE(foreign_work);
  EXPECT_EQ(
    owner.complete_route(
      *foreign_work, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kCleanupOnly);
  EXPECT_TRUE(foreign_work->live());
  EXPECT_TRUE(owner.snapshot().route_work);

  EXPECT_EQ(
    owner.complete_route(
      *owner_work, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kRetainCleanupEvidence);
  EXPECT_FALSE(owner_work->live());
  EXPECT_FALSE(owner.snapshot().route_work);
  EXPECT_EQ(
    owner.complete_route(
      *owner_work, PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kCleanupOnly);
}

TEST(PendingHandoffPreparationTest, ConcurrentRouteInvalidatesLiveAndSettledSamples)
{
  const auto key = binding();
  for (const bool settle_before_samples : {false, true}) {
    PendingAcceptedHandoffMachine machine(key);
    install_and_adopt(machine, key);
    auto prepare = take_prepare(machine);
    ASSERT_TRUE(prepare);
    const auto prepared_revision = prepare->prepared_revision();
    auto claim = machine.observe_route(
      key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
    auto route = claim.take_work();
    ASSERT_TRUE(route);
    EXPECT_GT(machine.snapshot().route_evidence_revision, prepared_revision);
    if (settle_before_samples) {
      ASSERT_EQ(
        machine.complete_route(
          *route, PendingRouteEvidenceResult::kExact,
          PendingControlMergeDecision::kStoreIncoming),
        PendingRouteCompletionStatus::kMerged);
    }
    auto completion = machine.complete_preparation(*prepare);
    EXPECT_EQ(
      completion.status(), PendingPreparationCompletionStatus::kRetryPreparation);
    EXPECT_FALSE(prepare->live());
    EXPECT_EQ(
      machine.snapshot().phase, PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease);
    if (!settle_before_samples) {
      ASSERT_EQ(
        machine.complete_route(
          *route, PendingRouteEvidenceResult::kExact,
          PendingControlMergeDecision::kStoreIncoming),
        PendingRouteCompletionStatus::kMerged);
    }
    EXPECT_EQ(
      machine.advance({healthy_gate(), false}).status(),
      PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope);
  }
}

TEST(PendingHandoffPreparationTest, SampleFailureAtomicallyClaimsOrCoalescesRoute)
{
  const auto key = binding();
  for (const auto [failure, expected_kind] : std::array{
        std::pair{PendingPreparationFailure::kSteadyClockFailure,
          PendingRouteKind::kSteadyClockFailure},
        std::pair{PendingPreparationFailure::kCallbackOrAdapterFailure,
          PendingRouteKind::kCallbackOrAdapterFailure}})
  {
    PendingAcceptedHandoffMachine machine(key);
    install_and_adopt(machine, key);
    auto prepare = take_prepare(machine);
    ASSERT_TRUE(prepare);
    auto completion = machine.complete_preparation_failure(
      *prepare, failure, PendingRouteObservationRelation::kNewDistinct);
    EXPECT_EQ(completion.status(), PendingPreparationCompletionStatus::kRouteClaimed);
    auto route = completion.take_route_work();
    ASSERT_TRUE(route);
    EXPECT_EQ(route->kind(), expected_kind);
    EXPECT_EQ(route->delivery_class(), PendingRouteDeliveryClass::kPreSeal);
  }

  PendingAcceptedHandoffMachine coalesced(key);
  install_and_adopt(coalesced, key);
  auto prepare = take_prepare(coalesced);
  ASSERT_TRUE(prepare);
  auto existing = coalesced.observe_route(
    key, PendingRouteKind::kSteadyClockFailure,
    PendingRouteObservationRelation::kNewDistinct);
  ASSERT_TRUE(existing.take_work());
  EXPECT_EQ(
    coalesced.complete_preparation_failure(
      *prepare, PendingPreparationFailure::kSteadyClockFailure,
      PendingRouteObservationRelation::kExactDuplicate).status(),
    PendingPreparationCompletionStatus::kRouteCoalesced);
  EXPECT_TRUE(coalesced.snapshot().route_work);
}

TEST(PendingHandoffPreparationTest, UnknownFailureInputsFailClosed)
{
  const auto key = binding();
  for (const auto [failure, relation] : std::array{
        std::pair{static_cast<PendingPreparationFailure>(0xffU),
          PendingRouteObservationRelation::kNewDistinct},
        std::pair{PendingPreparationFailure::kSteadyClockFailure,
          static_cast<PendingRouteObservationRelation>(0xffU)}})
  {
    PendingAcceptedHandoffMachine machine(key);
    install_and_adopt(machine, key);
    auto prepare = take_prepare(machine);
    ASSERT_TRUE(prepare);
    EXPECT_EQ(
      machine.complete_preparation_failure(*prepare, failure, relation).status(),
      PendingPreparationCompletionStatus::kCleanupOnly);
    EXPECT_FALSE(prepare->live());
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
  }
}

TEST(PendingHandoffPublicationTest, AcceptedResultsAreClosedAndNeverRetried)
{
  const auto key = binding();
  for (const auto result : {AcceptedEnvelopeDepositDecision::kCommitted,
      AcceptedEnvelopeDepositDecision::kCleanupOnly,
      static_cast<AcceptedEnvelopeDepositDecision>(0xffU)})
  {
    PendingAcceptedHandoffMachine machine(key);
    auto publication = reach_accepted_publication(machine, key);
    ASSERT_TRUE(publication);
    const auto status = machine.complete_accepted_publication(*publication, result);
    EXPECT_FALSE(publication->live());
    if (result == AcceptedEnvelopeDepositDecision::kCommitted) {
      EXPECT_EQ(status, PendingAcceptedPublicationStatus::kCommitted);
      EXPECT_EQ(
        machine.snapshot().phase, PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted);
    } else {
      EXPECT_EQ(status, PendingAcceptedPublicationStatus::kCleanupOnly);
      EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
      EXPECT_TRUE(machine.snapshot().handoff_work);
      EXPECT_FALSE(machine.snapshot().handoff_work->capability_outstanding);
    }
  }
}

TEST(PendingHandoffPublicationTest, CleanupCompletionRetainsOnlyAmbiguousDescriptor)
{
  const auto key = binding();
  for (const auto result : {AcceptedEnvelopeDepositDecision::kCommitted,
      AcceptedEnvelopeDepositDecision::kCleanupOnly,
      static_cast<AcceptedEnvelopeDepositDecision>(0xffU)})
  {
    PendingAcceptedHandoffMachine machine(key);
    auto publication = reach_accepted_publication(machine, key);
    ASSERT_TRUE(publication);
    machine.enter_cleanup_only();
    EXPECT_EQ(
      machine.complete_accepted_publication(*publication, result),
      PendingAcceptedPublicationStatus::kCleanupOnly);
    EXPECT_FALSE(publication->live());
    EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
    EXPECT_EQ(
      machine.snapshot().handoff_work.has_value(),
      result != AcceptedEnvelopeDepositDecision::kCommitted);
    if (machine.snapshot().handoff_work) {
      EXPECT_FALSE(machine.snapshot().handoff_work->capability_outstanding);
    }
  }
}

TEST(PendingHandoffPublicationTest, DeferredResultsAreClosed)
{
  const auto key = binding();
  for (const auto result : {PendingDeferredCompletion::kDelivered,
      PendingDeferredCompletion::kInboxLossSecured,
      PendingDeferredCompletion::kInboxLossUnsecured,
      static_cast<PendingDeferredCompletion>(0xffU)})
  {
    PendingAcceptedHandoffMachine machine(key);
    auto publication = reach_accepted_publication(machine, key);
    ASSERT_TRUE(publication);
    commit_accepted(machine, *publication);
    auto decision = machine.advance({healthy_gate(), true});
    auto work = decision.take_work();
    ASSERT_TRUE(work);
    const auto status = machine.complete_deferred_publication(*work, result);
    EXPECT_FALSE(work->live());
    if (result == PendingDeferredCompletion::kDelivered ||
      result == PendingDeferredCompletion::kInboxLossSecured)
    {
      EXPECT_EQ(status, PendingDeferredPublicationStatus::kCompleted);
      EXPECT_EQ(
        machine.snapshot().phase, PendingAcceptedHandoffPhase::kAcceptedEnvelopeCommitted);
    } else {
      EXPECT_EQ(status, PendingDeferredPublicationStatus::kCleanupOnly);
      EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
    }
  }
}

TEST(PendingHandoffExhaustionTest, RouteTokenAndRevisionAdvanceAtomically)
{
  const auto key = binding();
  for (const auto limits : {PendingHandoffLimits{0U, 1U, 4U},
      PendingHandoffLimits{1U, 0U, 4U}})
  {
    PendingAcceptedHandoffMachine machine(key, limits);
    EXPECT_EQ(
      machine.observe_route(
        key, PendingRouteKind::kDrain,
        PendingRouteObservationRelation::kNewDistinct).status(),
      PendingRouteClaimStatus::kCleanupOnly);
    EXPECT_EQ(machine.snapshot().last_route_token, 0U);
    EXPECT_EQ(machine.snapshot().route_evidence_revision, 0U);
    EXPECT_FALSE(machine.snapshot().route_work);
  }
}

TEST(PendingHandoffExhaustionTest, EveryHandoffBoundaryUsesCheckedLimit)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine no_prepare(key, PendingHandoffLimits{4U, 4U, 0U});
  install_and_adopt(no_prepare, key);
  EXPECT_EQ(
    no_prepare.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kCleanupOnly);

  PendingAcceptedHandoffMachine no_accepted_publish(
    key, PendingHandoffLimits{4U, 4U, 1U});
  install_and_adopt(no_accepted_publish, key);
  auto prepare = take_prepare(no_accepted_publish);
  ASSERT_TRUE(prepare);
  EXPECT_EQ(
    no_accepted_publish.complete_preparation(*prepare).status(),
    PendingPreparationCompletionStatus::kCleanupOnly);

  PendingAcceptedHandoffMachine no_deferred(key, PendingHandoffLimits{4U, 4U, 2U});
  auto publication = reach_accepted_publication(no_deferred, key);
  ASSERT_TRUE(publication);
  commit_accepted(no_deferred, *publication);
  EXPECT_EQ(
    no_deferred.advance({healthy_gate(), true}).status(),
    PendingHandoffAdvanceStatus::kCleanupOnly);
}

TEST(PendingHandoffOwnershipTest, TakingDecisionsIsOneShotAndAbandonmentBlocks)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine route_machine(key);
  auto route_decision = route_machine.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto route = route_decision.take_work();
  ASSERT_TRUE(route);
  EXPECT_FALSE(route_decision.take_work());
  route.reset();
  EXPECT_TRUE(route_machine.snapshot().route_work);
  EXPECT_EQ(
    route_machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kWait);

  PendingAcceptedHandoffMachine handoff_machine(key);
  install_and_adopt(handoff_machine, key);
  auto decision = handoff_machine.advance({healthy_gate(), false});
  auto prepare = decision.take_work();
  ASSERT_TRUE(prepare);
  EXPECT_FALSE(decision.take_work());
  prepare.reset();
  EXPECT_TRUE(handoff_machine.snapshot().handoff_work);
  EXPECT_EQ(
    handoff_machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kWait);
}

TEST(PendingHandoffOwnershipTest, ForeignAndReplayedHandoffTokensCannotStealWork)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine owner(key);
  install_and_adopt(owner, key);
  auto owner_prepare = take_prepare(owner);
  ASSERT_TRUE(owner_prepare);

  const auto foreign_key = binding(1U, 1U, 2U);
  PendingAcceptedHandoffMachine foreign(foreign_key);
  install_and_adopt(foreign, foreign_key);
  auto foreign_prepare = take_prepare(foreign);
  ASSERT_TRUE(foreign_prepare);
  EXPECT_EQ(
    owner.complete_preparation(*foreign_prepare).status(),
    PendingPreparationCompletionStatus::kCleanupOnly);
  EXPECT_TRUE(foreign_prepare->live());
  EXPECT_TRUE(owner.snapshot().handoff_work);
  EXPECT_EQ(
    owner.complete_preparation(*owner_prepare).status(),
    PendingPreparationCompletionStatus::kCleanupOnly);
  EXPECT_FALSE(owner_prepare->live());
  EXPECT_FALSE(owner.snapshot().handoff_work);

  PendingAcceptedHandoffMachine replay(key);
  auto publication = reach_accepted_publication(replay, key);
  ASSERT_TRUE(publication);
  commit_accepted(replay, *publication);
  EXPECT_EQ(
    replay.complete_accepted_publication(
      *publication, AcceptedEnvelopeDepositDecision::kCommitted),
    PendingAcceptedPublicationStatus::kCleanupOnly);
  EXPECT_EQ(replay.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
}

TEST(PendingHandoffCleanupTest, CleanupIsAbsorbingButExactCompletionsRetireDescriptors)
{
  const auto key = binding();
  PendingAcceptedHandoffMachine route_machine(key);
  auto claim = route_machine.observe_route(
    key, PendingRouteKind::kDrain, PendingRouteObservationRelation::kNewDistinct);
  auto route = claim.take_work();
  ASSERT_TRUE(route);
  route_machine.enter_cleanup_only();
  EXPECT_EQ(
    route_machine.complete_route(
      *route, PendingRouteEvidenceResult::kInvalid,
      PendingControlMergeDecision::kCleanupOnly),
    PendingRouteCompletionStatus::kRetainCleanupEvidence);
  EXPECT_FALSE(route_machine.snapshot().route_work);
  EXPECT_EQ(route_machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);

  PendingAcceptedHandoffMachine handoff_machine(key);
  install_and_adopt(handoff_machine, key);
  auto prepare = take_prepare(handoff_machine);
  ASSERT_TRUE(prepare);
  handoff_machine.enter_cleanup_only();
  EXPECT_EQ(
    handoff_machine.complete_preparation(*prepare).status(),
    PendingPreparationCompletionStatus::kCleanupOnly);
  EXPECT_FALSE(handoff_machine.snapshot().handoff_work);
  EXPECT_EQ(handoff_machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);

  EXPECT_EQ(
    handoff_machine.observe_route(
      key, PendingRouteKind::kDrain,
      PendingRouteObservationRelation::kNewDistinct).status(),
    PendingRouteClaimStatus::kCleanupOnly);
  EXPECT_EQ(
    handoff_machine.advance({healthy_gate(), false}).status(),
    PendingHandoffAdvanceStatus::kCleanupOnly);
  EXPECT_EQ(
    handoff_machine.install_epoch(key), PendingHandoffPhaseEventStatus::kCleanupOnly);
  EXPECT_EQ(
    handoff_machine.adopt_accepted_handle(key), PendingHandoffPhaseEventStatus::kCleanupOnly);
}

}  // namespace
}  // namespace restocker_task_executor
