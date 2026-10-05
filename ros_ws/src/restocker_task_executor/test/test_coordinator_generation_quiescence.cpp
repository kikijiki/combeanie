// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <barrier>
#include <cstddef>
#include <cstdlib>

#include <atomic>
#include <chrono>
#include <future>
#include <limits>
#include <memory>
#include <new>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"

namespace
{
thread_local std::optional<std::size_t> allocations_before_failure;
}

void * operator new(std::size_t size)
{
  if (allocations_before_failure) {
    if (*allocations_before_failure == 0U) {
      throw std::bad_alloc{};
    }
    --*allocations_before_failure;
  }
  if (void * storage = std::malloc(size)) {
    return storage;
  }
  throw std::bad_alloc{};
}

void operator delete(void * storage) noexcept
{
  std::free(storage);
}

void operator delete(void * storage, std::size_t) noexcept
{
  std::free(storage);
}

namespace restocker_task_executor
{
namespace
{

constexpr GoalGeneration kGeneration = 7U;

CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId id{};
  id.front() = seed;
  id.back() = static_cast<std::uint8_t>(seed + 1U);
  return id;
}

// Every quiescence is constructed for the identity of the epoch that shares its generation.
CoordinatorGoalId gate_goal(const CoordinatorActiveFaultEpoch & epoch)
{
  return epoch.snapshot().goal_id;
}

GoalGeneration gate_gen(const CoordinatorActiveFaultEpoch & epoch)
{
  return epoch.snapshot().goal_generation;
}

CoordinatorTerminalAckCompletionDecision complete_accepted_terminal(
  CoordinatorActiveFaultEpoch & epoch)
{
  CoordinatorDriverOutput output;
  output.kind = CoordinatorDriverOutputKind::kSucceeded;
  output.goal_generation = epoch.snapshot().goal_generation;
  output.outcome = RestockActionOutcome::kSucceeded;
  output.detail = "accepted";
  auto offered = epoch.offer_terminal(output);
  EXPECT_EQ(offered.disposition(), ActiveTerminalDisposition::kPublicationPrepared);
  auto publication = offered.take_publication_permit();
  EXPECT_TRUE(publication);
  auto returned = epoch.terminal_publication_returned(*publication);
  EXPECT_EQ(returned.status(), ActivePublicationReturnStatus::kAcknowledgementPrepared);
  auto acknowledgement = returned.take_ack_permit();
  EXPECT_TRUE(acknowledgement);
  return epoch.complete_terminal_ack(*acknowledgement, true);
}

std::optional<CoordinatorAcceptedTerminalAckWitness> accepted_witness(
  CoordinatorActiveFaultEpoch & epoch)
{
  auto completion = complete_accepted_terminal(epoch);
  EXPECT_EQ(
    completion.status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  return completion.take_accepted_ack_witness();
}

std::optional<CoordinatorDepositPermit> begin(
  CoordinatorGenerationQuiescence & gate,
  const CoordinatorGoalId & id = goal_id(), GoalGeneration generation = kGeneration)
{
  auto decision = gate.begin_deposit(id, generation);
  EXPECT_EQ(decision.status(), GenerationDepositBeginStatus::kStarted);
  return decision.take_deposit_permit();
}

void seal(
  CoordinatorGenerationQuiescence & gate,
  CoordinatorActiveFaultEpoch & epoch)
{
  auto witness = accepted_witness(epoch);
  ASSERT_TRUE(witness);
  EXPECT_EQ(
    gate.seal_after_accepted_ack(*witness),
    GenerationQuiescenceSealStatus::kSealed);
  EXPECT_FALSE(witness->live());
}

std::optional<CoordinatorGenerationQuiescenceReceipt> prepare_receipt(
  CoordinatorGenerationQuiescence & gate,
  CoordinatorActiveFaultEpoch & epoch)
{
  seal(gate, epoch);
  auto probe_decision = gate.prepare_probe();
  EXPECT_EQ(probe_decision.status(), GenerationQuiescenceProbeStatus::kPrepared);
  auto probe = probe_decision.take_probe_permit();
  EXPECT_TRUE(probe);
  auto completed = gate.complete_probe(*probe, kGeneration, true);
  EXPECT_EQ(
    completed.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);
  return completed.take_receipt();
}

template<typename Value>
void exercise_self_move(Value & value)
{
  auto * alias = &value;
  value = std::move(*alias);
}

TEST(CoordinatorGenerationQuiescenceValueTest, CheckedSuccessorsRejectOnlyMaximum)
{
  static_assert(
    checked_next_accepted_terminal_ack_cookie(0U) == AcceptedTerminalAckCookie{1U});
  const auto ack_penultimate = std::numeric_limits<AcceptedTerminalAckCookie>::max() - 1U;
  EXPECT_EQ(
    checked_next_accepted_terminal_ack_cookie(ack_penultimate),
    std::numeric_limits<AcceptedTerminalAckCookie>::max());
  EXPECT_FALSE(
    checked_next_accepted_terminal_ack_cookie(
      std::numeric_limits<AcceptedTerminalAckCookie>::max()));
}

TEST(CoordinatorGenerationQuiescenceValueTest, SealedTypesHaveNothrowMoveOwnership)
{
  static_assert(std::is_trivially_copyable_v<CoordinatorGenerationQuiescenceSnapshot>);
  static_assert(!std::is_default_constructible_v<CoordinatorDepositPermit>);
  static_assert(!std::is_copy_constructible_v<CoordinatorDepositPermit>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorDepositPermit>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorDepositPermit>);
  static_assert(!std::is_default_constructible_v<CoordinatorGenerationProbePermit>);
  static_assert(!std::is_copy_constructible_v<CoordinatorGenerationProbePermit>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorGenerationProbePermit>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorGenerationProbePermit>);
  static_assert(!std::is_default_constructible_v<CoordinatorGenerationQuiescenceReceipt>);
  static_assert(!std::is_copy_constructible_v<CoordinatorGenerationQuiescenceReceipt>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorGenerationQuiescenceReceipt>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorGenerationQuiescenceReceipt>);
  static_assert(!std::is_default_constructible_v<GenerationDepositBeginDecision>);
  static_assert(!std::is_copy_constructible_v<GenerationDepositBeginDecision>);
  static_assert(std::is_nothrow_move_constructible_v<GenerationDepositBeginDecision>);
  static_assert(std::is_nothrow_move_assignable_v<GenerationDepositBeginDecision>);
  static_assert(!std::is_default_constructible_v<CoordinatorGenerationProbeDecision>);
  static_assert(!std::is_copy_constructible_v<CoordinatorGenerationProbeDecision>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorGenerationProbeDecision>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorGenerationProbeDecision>);
  static_assert(!std::is_default_constructible_v<CoordinatorGenerationCompletionDecision>);
  static_assert(!std::is_copy_constructible_v<CoordinatorGenerationCompletionDecision>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorGenerationCompletionDecision>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorGenerationCompletionDecision>);
  static_assert(!std::is_move_constructible_v<CoordinatorGenerationQuiescence>);
  SUCCEED();
}

TEST(CoordinatorGenerationQuiescenceConstructionTest, ValidatesCapacityThenIdentity)
{
  CoordinatorGenerationQuiescence gate(goal_id(), kGeneration, 2U);
  EXPECT_EQ(gate.snapshot().goal_id, goal_id());
  EXPECT_EQ(gate.snapshot().generation, kGeneration);

  EXPECT_THROW(
    CoordinatorGenerationQuiescence(goal_id(), kGeneration, 0U), std::invalid_argument);
  EXPECT_THROW(
    CoordinatorGenerationQuiescence(CoordinatorGoalId{}, kGeneration, 1U),
    std::invalid_argument);
  EXPECT_THROW(
    CoordinatorGenerationQuiescence(goal_id(), 0U, 1U), std::invalid_argument);
  EXPECT_THROW(
    CoordinatorGenerationQuiescence(goal_id(), kReservedCoordinatorGoalGeneration, 1U),
    std::invalid_argument);
}

TEST(CoordinatorGenerationQuiescenceConstructionTest, PropagatesSlotAllocationFailure)
{
  bool slots_failed = false;
  allocations_before_failure = 0U;
  try {
    CoordinatorGenerationQuiescence gate(goal_id(), kGeneration, 2U);
  } catch (const std::bad_alloc &) {
    slots_failed = true;
  }
  allocations_before_failure.reset();
  EXPECT_TRUE(slots_failed);
}

TEST(CoordinatorGenerationQuiescenceDepositTest, SnapshotAndKnownCompletionsAreExact)
{
  for (const auto disposition : {
        GenerationDepositCompletion::kAccepted,
        GenerationDepositCompletion::kOverflowAccounted,
        GenerationDepositCompletion::kInhibitedAccounted,
        GenerationDepositCompletion::kEvidenceLostAccounted})
  {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
    const auto initial = gate.snapshot();
    EXPECT_EQ(initial.goal_id, goal_id());
    EXPECT_EQ(initial.generation, kGeneration);
    EXPECT_FALSE(initial.sealed);
    EXPECT_EQ(initial.active_deposit_count, 0U);
    EXPECT_FALSE(initial.deposit_unresolved);
    EXPECT_FALSE(initial.adapter_fault);
    EXPECT_EQ(
      initial.retirement_fence_state, GenerationRetirementFenceState::kHealthy);
    EXPECT_FALSE(initial.probe_outstanding);
    EXPECT_FALSE(initial.receipt_outstanding);
    EXPECT_EQ(initial.accepted_ack_cookie, 0U);

    auto permit = begin(gate);
    ASSERT_TRUE(permit);
    EXPECT_EQ(permit->slot(), 0U);
    EXPECT_EQ(permit->attempt(), 1U);
    EXPECT_EQ(permit->cookie(), 1U);
    EXPECT_EQ(gate.snapshot().active_deposit_count, 1U);
    EXPECT_EQ(
      gate.complete_deposit(*permit, disposition),
      GenerationDepositCompletionStatus::kCompleted);
    EXPECT_FALSE(permit->live());
    EXPECT_EQ(gate.snapshot().active_deposit_count, 0U);
    EXPECT_FALSE(gate.snapshot().deposit_unresolved);
  }
}

TEST(CoordinatorGenerationQuiescenceDepositTest, UnknownAndInvalidDispositionAreFailClosed)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto permit = begin(gate);
  ASSERT_TRUE(permit);
  EXPECT_EQ(
    gate.complete_deposit(*permit, static_cast<GenerationDepositCompletion>(255)),
    GenerationDepositCompletionStatus::kInvalidDisposition);
  EXPECT_TRUE(permit->live());
  EXPECT_EQ(gate.snapshot().active_deposit_count, 1U);
  EXPECT_EQ(
    gate.complete_deposit(*permit, GenerationDepositCompletion::kUnknown),
    GenerationDepositCompletionStatus::kCompletedUnresolved);
  EXPECT_FALSE(permit->live());
  EXPECT_TRUE(gate.snapshot().deposit_unresolved);

  seal(gate, epoch);
  EXPECT_EQ(
    gate.prepare_probe().status(),
    GenerationQuiescenceProbeStatus::kDepositUnresolved);
}

TEST(CoordinatorGenerationQuiescenceDepositTest, CapacityAndIdentityFailuresAreSticky)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto permit = begin(gate);
  ASSERT_TRUE(permit);
  auto exhausted = gate.begin_deposit(goal_id(), kGeneration);
  EXPECT_EQ(exhausted.status(), GenerationDepositBeginStatus::kCapacityExhausted);
  EXPECT_FALSE(exhausted.take_deposit_permit());
  EXPECT_TRUE(gate.snapshot().deposit_unresolved);
  EXPECT_TRUE(gate.snapshot().adapter_fault);
  EXPECT_EQ(
    gate.snapshot().retirement_fence_state, GenerationRetirementFenceState::kFailed);
  EXPECT_EQ(
    gate.complete_deposit(*permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);

  CoordinatorActiveFaultEpoch wrong_epoch(goal_id(4U), kGeneration);
  CoordinatorGenerationQuiescence wrong_gate(gate_goal(wrong_epoch), gate_gen(wrong_epoch), 1U);
  EXPECT_EQ(
    wrong_gate.begin_deposit(goal_id(), kGeneration).status(),
    GenerationDepositBeginStatus::kIdentityMismatch);
  EXPECT_TRUE(wrong_gate.snapshot().adapter_fault);

  CoordinatorActiveFaultEpoch invalid_epoch(goal_id(5U), kGeneration);
  CoordinatorGenerationQuiescence invalid_gate(
    gate_goal(invalid_epoch), gate_gen(invalid_epoch), 1U);
  EXPECT_EQ(
    invalid_gate.begin_deposit(CoordinatorGoalId{}, kGeneration).status(),
    GenerationDepositBeginStatus::kInvalidArgument);
  EXPECT_TRUE(invalid_gate.snapshot().adapter_fault);
}

TEST(CoordinatorGenerationQuiescenceDepositTest, LaterGenerationAndMovedPermitsPreserveOwner)
{
  CoordinatorGenerationQuiescence first(goal_id(), kGeneration, 1U);
  CoordinatorGenerationQuiescence second(goal_id(), kGeneration + 1U, 1U);
  auto permit = begin(first);
  ASSERT_TRUE(permit);
  EXPECT_EQ(
    second.complete_deposit(*permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(permit->live());

  CoordinatorDepositPermit moved{std::move(*permit)};
  EXPECT_FALSE(permit->live());
  EXPECT_EQ(
    first.complete_deposit(*permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kNotLive);
  EXPECT_EQ(
    first.complete_deposit(moved, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);
}

TEST(CoordinatorGenerationQuiescenceDecisionTest, BeginDecisionMovesAndTakesOnce)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  auto destination = first.begin_deposit(goal_id(), kGeneration);
  auto source = second.begin_deposit(goal_id(3U), kGeneration);
  auto constructed{std::move(source)};
  EXPECT_EQ(source.status(), GenerationDepositBeginStatus::kStarted);
  EXPECT_FALSE(source.take_deposit_permit());
  exercise_self_move(constructed);
  source = std::move(constructed);
  EXPECT_FALSE(constructed.take_deposit_permit());
  destination = std::move(source);
  EXPECT_EQ(source.status(), GenerationDepositBeginStatus::kStarted);
  EXPECT_FALSE(source.take_deposit_permit());
  EXPECT_EQ(destination.status(), GenerationDepositBeginStatus::kStarted);
  auto permit = destination.take_deposit_permit();
  ASSERT_TRUE(permit);
  EXPECT_FALSE(destination.take_deposit_permit());
  EXPECT_EQ(
    second.complete_deposit(*permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);
  EXPECT_EQ(first.snapshot().active_deposit_count, 1U);
}

TEST(CoordinatorGenerationQuiescenceSealTest, LaterGenerationWitnessIsPreservedForItsGate)
{
  CoordinatorActiveFaultEpoch owner(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch foreign(goal_id(), kGeneration + 1U);
  CoordinatorGenerationQuiescence owner_gate(gate_goal(owner), gate_gen(owner), 1U);
  CoordinatorGenerationQuiescence foreign_gate(gate_goal(foreign), gate_gen(foreign), 1U);
  auto witness = accepted_witness(foreign);
  ASSERT_TRUE(witness);
  EXPECT_EQ(
    owner_gate.seal_after_accepted_ack(*witness),
    GenerationQuiescenceSealStatus::kIdentityMismatch);
  EXPECT_TRUE(witness->live());
  EXPECT_TRUE(owner_gate.snapshot().adapter_fault);
  EXPECT_EQ(
    foreign_gate.seal_after_accepted_ack(*witness),
    GenerationQuiescenceSealStatus::kSealed);
  EXPECT_FALSE(witness->live());
  EXPECT_EQ(
    owner_gate.begin_deposit(goal_id(), kGeneration).status(),
    GenerationDepositBeginStatus::kSynchronizationFailed);
}

TEST(CoordinatorGenerationQuiescenceSealTest, BeginAndSealOrdersAreBounded)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  auto first_witness = accepted_witness(first_epoch);
  ASSERT_TRUE(first_witness);
  std::promise<void> began;
  std::promise<void> release;
  auto release_future = release.get_future();
  std::optional<CoordinatorDepositPermit> first_permit;
  std::thread begin_first([&]() {
      auto decision = first.begin_deposit(goal_id(), kGeneration);
      first_permit = decision.take_deposit_permit();
      began.set_value();
      release_future.wait();
    });
  ASSERT_EQ(began.get_future().wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_EQ(
    first.seal_after_accepted_ack(*first_witness),
    GenerationQuiescenceSealStatus::kSealed);
  EXPECT_EQ(first.snapshot().active_deposit_count, 1U);
  release.set_value();
  begin_first.join();
  ASSERT_TRUE(first_permit);
  EXPECT_EQ(
    first.complete_deposit(*first_permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);

  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  seal(second, second_epoch);
  std::promise<GenerationDepositBeginStatus> sealed_status;
  std::thread seal_first([&]() {
      sealed_status.set_value(second.begin_deposit(goal_id(3U), kGeneration).status());
    });
  EXPECT_EQ(
    sealed_status.get_future().get(), GenerationDepositBeginStatus::kSealed);
  seal_first.join();
  EXPECT_EQ(second.snapshot().active_deposit_count, 0U);
}

TEST(CoordinatorGenerationQuiescenceSealTest, ConcurrentBeginAndSealClassifyLegalOrders)
{
  constexpr std::size_t kIterations = 64U;
  std::size_t begin_won = 0U;
  std::size_t seal_won = 0U;
  for (std::size_t iteration = 0U; iteration < kIterations; ++iteration) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
    auto witness = accepted_witness(epoch);
    ASSERT_TRUE(witness);
    std::barrier start{3};
    std::optional<GenerationDepositBeginDecision> begin_result;
    GenerationQuiescenceSealStatus seal_result{GenerationQuiescenceSealStatus::kNotLive};
    std::thread begin_thread([&]() {
        start.arrive_and_wait();
        begin_result.emplace(gate.begin_deposit(goal_id(), kGeneration));
      });
    std::thread seal_thread([&]() {
        start.arrive_and_wait();
        seal_result = gate.seal_after_accepted_ack(*witness);
      });
    start.arrive_and_wait();
    begin_thread.join();
    seal_thread.join();

    ASSERT_TRUE(begin_result);
    EXPECT_EQ(seal_result, GenerationQuiescenceSealStatus::kSealed);
    if (begin_result->status() == GenerationDepositBeginStatus::kStarted) {
      ++begin_won;
      auto permit = begin_result->take_deposit_permit();
      ASSERT_TRUE(permit);
      EXPECT_EQ(gate.snapshot().active_deposit_count, 1U);
      EXPECT_EQ(
        gate.complete_deposit(*permit, GenerationDepositCompletion::kAccepted),
        GenerationDepositCompletionStatus::kCompleted);
    } else if (begin_result->status() == GenerationDepositBeginStatus::kSealed) {
      ++seal_won;
      EXPECT_FALSE(begin_result->take_deposit_permit());
      EXPECT_EQ(gate.snapshot().active_deposit_count, 0U);
    } else {
      ADD_FAILURE() << "unexpected begin result";
    }
  }
  EXPECT_EQ(begin_won + seal_won, kIterations);
}

TEST(CoordinatorGenerationQuiescenceProbeTest, NegativeProbeConsumesAndChangesCookie)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  seal(gate, epoch);
  auto first_decision = gate.prepare_probe();
  auto first = first_decision.take_probe_permit();
  ASSERT_TRUE(first);
  EXPECT_EQ(first->cookie(), 1U);
  EXPECT_EQ(
    gate.prepare_probe().status(), GenerationQuiescenceProbeStatus::kProbeOutstanding);
  EXPECT_EQ(
    gate.complete_probe(*first, std::nullopt, true).status(),
    GenerationQuiescenceCompletionStatus::kDriverLineageMismatch);
  EXPECT_FALSE(first->live());

  auto second_decision = gate.prepare_probe();
  auto second = second_decision.take_probe_permit();
  ASSERT_TRUE(second);
  EXPECT_EQ(second->cookie(), 2U);
  EXPECT_EQ(
    gate.complete_probe(*second, kGeneration, false).status(),
    GenerationQuiescenceCompletionStatus::kInboxNotEmpty);
  EXPECT_FALSE(second->live());
}

TEST(CoordinatorGenerationQuiescenceProbeTest, LaterProbeFinishesAcceptedAckRetirement)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  seal(gate, epoch);
  ASSERT_TRUE(epoch.snapshot().retirement_eligible);

  auto first_decision = gate.prepare_probe();
  auto first = first_decision.take_probe_permit();
  ASSERT_TRUE(first);
  auto not_yet_quiescent = gate.complete_probe(*first, kGeneration, false);
  EXPECT_EQ(
    not_yet_quiescent.status(),
    GenerationQuiescenceCompletionStatus::kInboxNotEmpty);
  EXPECT_FALSE(not_yet_quiescent.take_receipt());
  EXPECT_TRUE(epoch.snapshot().retirement_eligible);
  EXPECT_FALSE(gate.snapshot().receipt_outstanding);

  auto later_decision = gate.prepare_probe();
  auto later = later_decision.take_probe_permit();
  ASSERT_TRUE(later);
  EXPECT_NE(later->cookie(), first->cookie());
  auto now_quiescent = gate.complete_probe(*later, kGeneration, true);
  EXPECT_EQ(
    now_quiescent.status(),
    GenerationQuiescenceCompletionStatus::kReceiptIssued);
  auto receipt = now_quiescent.take_receipt();
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
  EXPECT_FALSE(receipt->live());
  EXPECT_EQ(
    gate.snapshot().retirement_fence_state,
    GenerationRetirementFenceState::kRetirementCommitted);
}

TEST(CoordinatorGenerationQuiescenceProbeTest, ProbeAndReceiptDecisionsMoveAndTakeOnce)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  seal(gate, epoch);
  auto original_probe = gate.prepare_probe();
  CoordinatorGenerationProbeDecision moved_probe{std::move(original_probe)};
  EXPECT_EQ(original_probe.status(), GenerationQuiescenceProbeStatus::kPrepared);
  EXPECT_FALSE(original_probe.take_probe_permit());
  exercise_self_move(moved_probe);
  auto probe = moved_probe.take_probe_permit();
  ASSERT_TRUE(probe);
  EXPECT_FALSE(moved_probe.take_probe_permit());

  auto original_completion = gate.complete_probe(*probe, kGeneration, true);
  CoordinatorGenerationCompletionDecision moved_completion{std::move(original_completion)};
  EXPECT_EQ(
    original_completion.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);
  EXPECT_FALSE(original_completion.take_receipt());
  exercise_self_move(moved_completion);
  auto receipt = moved_completion.take_receipt();
  ASSERT_TRUE(receipt);
  EXPECT_FALSE(moved_completion.take_receipt());
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
}

TEST(CoordinatorGenerationQuiescenceDecisionTest, ProbeDecisionMoveAssignmentAbandonsNoState)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  seal(first, first_epoch);
  seal(second, second_epoch);
  auto destination = first.prepare_probe();
  auto source = second.prepare_probe();
  ASSERT_EQ(destination.status(), GenerationQuiescenceProbeStatus::kPrepared);
  ASSERT_EQ(source.status(), GenerationQuiescenceProbeStatus::kPrepared);

  destination = std::move(source);
  EXPECT_EQ(source.status(), GenerationQuiescenceProbeStatus::kPrepared);
  EXPECT_FALSE(source.take_probe_permit());
  auto permit = destination.take_probe_permit();
  ASSERT_TRUE(permit);
  EXPECT_EQ(
    second.complete_probe(*permit, std::nullopt, true).status(),
    GenerationQuiescenceCompletionStatus::kDriverLineageMismatch);
  EXPECT_EQ(
    first.prepare_probe().status(), GenerationQuiescenceProbeStatus::kProbeOutstanding);
}

TEST(CoordinatorGenerationQuiescenceDecisionTest, ReceiptDecisionMoveAssignmentAbandonsNoState)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  seal(first, first_epoch);
  seal(second, second_epoch);
  auto first_probe_decision = first.prepare_probe();
  auto first_probe = first_probe_decision.take_probe_permit();
  auto second_probe_decision = second.prepare_probe();
  auto second_probe = second_probe_decision.take_probe_permit();
  ASSERT_TRUE(first_probe);
  ASSERT_TRUE(second_probe);
  auto destination = first.complete_probe(*first_probe, kGeneration, true);
  auto source = second.complete_probe(*second_probe, kGeneration, true);
  ASSERT_EQ(destination.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);
  ASSERT_EQ(source.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);

  destination = std::move(source);
  EXPECT_EQ(source.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);
  EXPECT_FALSE(source.take_receipt());
  auto receipt = destination.take_receipt();
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    second.consume_receipt(*receipt, goal_id(3U), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
  EXPECT_EQ(
    first.prepare_probe().status(), GenerationQuiescenceProbeStatus::kAlreadyIssued);
}

TEST(CoordinatorGenerationQuiescenceCapabilityTest, DepositMoveAssignmentAndSelfMoveAreExact)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  auto destination = begin(first);
  auto source = begin(second, goal_id(3U));
  ASSERT_TRUE(destination);
  ASSERT_TRUE(source);

  *destination = std::move(*source);
  EXPECT_FALSE(source->live());
  EXPECT_TRUE(destination->live());
  EXPECT_EQ(first.snapshot().active_deposit_count, 1U);
  exercise_self_move(*destination);
  EXPECT_TRUE(destination->live());
  EXPECT_EQ(
    second.complete_deposit(*destination, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);
  EXPECT_EQ(first.snapshot().active_deposit_count, 1U);
}

TEST(CoordinatorGenerationQuiescenceCapabilityTest, ProbeMoveAssignmentAndSelfMoveAreExact)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  seal(first, first_epoch);
  seal(second, second_epoch);
  auto first_decision = first.prepare_probe();
  auto destination = first_decision.take_probe_permit();
  auto second_decision = second.prepare_probe();
  auto source = second_decision.take_probe_permit();
  ASSERT_TRUE(destination);
  ASSERT_TRUE(source);

  *destination = std::move(*source);
  EXPECT_FALSE(source->live());
  EXPECT_TRUE(destination->live());
  EXPECT_TRUE(first.snapshot().probe_outstanding);
  exercise_self_move(*destination);
  EXPECT_TRUE(destination->live());
  EXPECT_EQ(
    second.complete_probe(*destination, std::nullopt, true).status(),
    GenerationQuiescenceCompletionStatus::kDriverLineageMismatch);
  EXPECT_EQ(
    first.prepare_probe().status(), GenerationQuiescenceProbeStatus::kProbeOutstanding);
}

TEST(CoordinatorGenerationQuiescenceCapabilityTest, ReceiptMoveAssignmentAndSelfMoveAreExact)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second_epoch(goal_id(3U), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(first_epoch), gate_gen(first_epoch), 1U);
  CoordinatorGenerationQuiescence second(gate_goal(second_epoch), gate_gen(second_epoch), 1U);
  auto destination = prepare_receipt(first, first_epoch);
  auto source = prepare_receipt(second, second_epoch);
  ASSERT_TRUE(destination);
  ASSERT_TRUE(source);

  *destination = std::move(*source);
  EXPECT_FALSE(source->live());
  EXPECT_TRUE(destination->live());
  EXPECT_TRUE(first.snapshot().receipt_outstanding);
  exercise_self_move(*destination);
  EXPECT_TRUE(destination->live());
  EXPECT_EQ(
    second.consume_receipt(*destination, goal_id(3U), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
  EXPECT_EQ(
    first.prepare_probe().status(), GenerationQuiescenceProbeStatus::kAlreadyIssued);
}

TEST(CoordinatorGenerationQuiescenceLifetimeTest, OldCapabilityRejectsAddressReusedGate)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::byte epoch_storage[sizeof(CoordinatorActiveFaultEpoch)];
  alignas(CoordinatorGenerationQuiescence)
  std::byte gate_storage[sizeof(CoordinatorGenerationQuiescence)];

  auto * old_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration);
  auto * old_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*old_epoch), gate_gen(*old_epoch), 1U);
  auto old_permit = begin(*old_gate);
  ASSERT_TRUE(old_permit);
  std::destroy_at(old_gate);
  std::destroy_at(old_epoch);

  auto * replacement_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration + 1U);
  auto * replacement_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*replacement_epoch), gate_gen(*replacement_epoch), 1U);
  EXPECT_EQ(
    replacement_gate->complete_deposit(
      *old_permit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(old_permit->live());
  std::destroy_at(replacement_gate);
  std::destroy_at(replacement_epoch);
  old_permit.reset();
}

TEST(CoordinatorGenerationQuiescenceLifetimeTest, OldProbeRejectsAddressReusedGate)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::byte epoch_storage[sizeof(CoordinatorActiveFaultEpoch)];
  alignas(CoordinatorGenerationQuiescence)
  std::byte gate_storage[sizeof(CoordinatorGenerationQuiescence)];

  auto * old_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration);
  auto * old_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*old_epoch), gate_gen(*old_epoch), 1U);
  seal(*old_gate, *old_epoch);
  auto old_probe_decision = old_gate->prepare_probe();
  auto old_probe = old_probe_decision.take_probe_permit();
  ASSERT_TRUE(old_probe);
  std::destroy_at(old_gate);
  std::destroy_at(old_epoch);

  auto * replacement_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration + 1U);
  auto * replacement_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*replacement_epoch), gate_gen(*replacement_epoch), 1U);
  EXPECT_EQ(
    replacement_gate->complete_probe(*old_probe, kGeneration, true).status(),
    GenerationQuiescenceCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(old_probe->live());
  std::destroy_at(replacement_gate);
  std::destroy_at(replacement_epoch);
  old_probe.reset();
}

TEST(CoordinatorGenerationQuiescenceLifetimeTest, OldReceiptRejectsAddressReusedGate)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::byte epoch_storage[sizeof(CoordinatorActiveFaultEpoch)];
  alignas(CoordinatorGenerationQuiescence)
  std::byte gate_storage[sizeof(CoordinatorGenerationQuiescence)];

  auto * old_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration);
  auto * old_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*old_epoch), gate_gen(*old_epoch), 1U);
  auto old_receipt = prepare_receipt(*old_gate, *old_epoch);
  ASSERT_TRUE(old_receipt);
  std::destroy_at(old_gate);
  std::destroy_at(old_epoch);

  auto * replacement_epoch = std::construct_at(
    reinterpret_cast<CoordinatorActiveFaultEpoch *>(epoch_storage), goal_id(), kGeneration + 1U);
  auto * replacement_gate = std::construct_at(
    reinterpret_cast<CoordinatorGenerationQuiescence *>(gate_storage),
    gate_goal(*replacement_epoch), gate_gen(*replacement_epoch), 1U);
  EXPECT_EQ(
    replacement_gate->consume_receipt(*old_receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kIdentityMismatch);
  EXPECT_TRUE(old_receipt->live());
  std::destroy_at(replacement_gate);
  std::destroy_at(replacement_epoch);
  old_receipt.reset();
}

TEST(CoordinatorGenerationQuiescenceProbeTest, LaterGenerationProbeAndReceiptPreserveOwner)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence first(gate_goal(epoch), gate_gen(epoch), 1U);
  CoordinatorGenerationQuiescence second(goal_id(), kGeneration + 1U, 1U);
  seal(first, epoch);
  auto probe_decision = first.prepare_probe();
  auto probe = probe_decision.take_probe_permit();
  ASSERT_TRUE(probe);
  EXPECT_EQ(
    second.complete_probe(*probe, kGeneration, true).status(),
    GenerationQuiescenceCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(probe->live());
  auto receipt_decision = first.complete_probe(*probe, kGeneration, true);
  auto receipt = receipt_decision.take_receipt();
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    second.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kIdentityMismatch);
  EXPECT_TRUE(receipt->live());
  EXPECT_EQ(
    first.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
}

TEST(CoordinatorGenerationQuiescenceFenceTest, MarkFirstPermanentlyRejectsRetirement)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto receipt = prepare_receipt(gate, epoch);
  ASSERT_TRUE(receipt);
  std::promise<GenerationSynchronizationFailureStatus> marked;
  std::thread marker([&]() {marked.set_value(gate.mark_synchronization_failure());});
  EXPECT_EQ(
    marked.get_future().get(), GenerationSynchronizationFailureStatus::kMarkedFailed);
  marker.join();
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kRetirementFenceFailed);
  EXPECT_TRUE(receipt->live());
  EXPECT_EQ(
    gate.mark_synchronization_failure(),
    GenerationSynchronizationFailureStatus::kAlreadyFailed);
}

TEST(CoordinatorGenerationQuiescenceFenceTest, ConcurrentMarkersHaveOneWinner)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  constexpr std::size_t kThreadCount = 16U;
  std::atomic<bool> start{false};
  std::atomic<std::size_t> marked{0U};
  std::atomic<std::size_t> already_failed{0U};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0U; index < kThreadCount; ++index) {
    threads.emplace_back(
      [&]() {
        while (!start.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        const auto status = gate.mark_synchronization_failure();
        if (status == GenerationSynchronizationFailureStatus::kMarkedFailed) {
          marked.fetch_add(1U, std::memory_order_relaxed);
        } else if (status == GenerationSynchronizationFailureStatus::kAlreadyFailed) {
          already_failed.fetch_add(1U, std::memory_order_relaxed);
        }
      });
  }
  start.store(true, std::memory_order_release);
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_EQ(marked.load(), 1U);
  EXPECT_EQ(already_failed.load(), kThreadCount - 1U);
  EXPECT_EQ(gate.snapshot().retirement_fence_state, GenerationRetirementFenceState::kFailed);
}

TEST(CoordinatorGenerationQuiescenceFenceTest, ConsumeFirstMakesLateMarkIrrelevant)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto receipt = prepare_receipt(gate, epoch);
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
  EXPECT_EQ(
    gate.snapshot().retirement_fence_state, GenerationRetirementFenceState::kRetirementCommitted);

  std::promise<GenerationSynchronizationFailureStatus> marked;
  std::thread marker([&]() {marked.set_value(gate.mark_synchronization_failure());});
  EXPECT_EQ(
    marked.get_future().get(), GenerationSynchronizationFailureStatus::kRetirementCommitted);
  marker.join();
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kNotLive);
}

TEST(CoordinatorGenerationQuiescenceFenceTest, ConcurrentMarkAndConsumeClassifyLegalOrders)
{
  constexpr std::size_t kIterations = 64U;
  std::size_t mark_won = 0U;
  std::size_t consume_won = 0U;
  for (std::size_t iteration = 0U; iteration < kIterations; ++iteration) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
    auto receipt = prepare_receipt(gate, epoch);
    ASSERT_TRUE(receipt);
    std::barrier start{3};
    GenerationSynchronizationFailureStatus mark_result{
      GenerationSynchronizationFailureStatus::kAlreadyFailed};
    GenerationQuiescenceConsumeStatus consume_result{
      GenerationQuiescenceConsumeStatus::kNotLive};
    std::thread marker([&]() {
        start.arrive_and_wait();
        mark_result = gate.mark_synchronization_failure();
      });
    std::thread consumer([&]() {
        start.arrive_and_wait();
        consume_result = gate.consume_receipt(*receipt, goal_id(), kGeneration);
      });
    start.arrive_and_wait();
    marker.join();
    consumer.join();

    const bool retirement_committed =
      mark_result == GenerationSynchronizationFailureStatus::kRetirementCommitted;
    if (mark_result == GenerationSynchronizationFailureStatus::kMarkedFailed) {
      ++mark_won;
      EXPECT_EQ(
        consume_result, GenerationQuiescenceConsumeStatus::kRetirementFenceFailed);
      EXPECT_TRUE(receipt->live());
      EXPECT_EQ(gate.snapshot().retirement_fence_state, GenerationRetirementFenceState::kFailed);
    } else if (retirement_committed) {
      ++consume_won;
      EXPECT_EQ(consume_result, GenerationQuiescenceConsumeStatus::kConsumed);
      EXPECT_FALSE(receipt->live());
      EXPECT_EQ(
        gate.snapshot().retirement_fence_state,
        GenerationRetirementFenceState::kRetirementCommitted);
    } else {
      ADD_FAILURE() << "unexpected mark result";
    }
  }
  EXPECT_EQ(mark_won + consume_won, kIterations);
}

TEST(CoordinatorGenerationQuiescenceFenceTest, PostCommitAdapterFaultHasExplicitPrecedence)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto receipt = prepare_receipt(gate, epoch);
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kConsumed);
  EXPECT_EQ(
    gate.begin_deposit(CoordinatorGoalId{}, kGeneration).status(),
    GenerationDepositBeginStatus::kInvalidArgument);
  const auto snapshot = gate.snapshot();
  EXPECT_TRUE(snapshot.adapter_fault);
  EXPECT_EQ(
    snapshot.retirement_fence_state, GenerationRetirementFenceState::kRetirementCommitted);
  EXPECT_EQ(
    gate.prepare_probe().status(),
    GenerationQuiescenceProbeStatus::kSynchronizationFailed);
}

TEST(CoordinatorGenerationQuiescencePrecedenceTest, PublicStatusesPreserveExactAuthority)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  EXPECT_EQ(
    gate.prepare_probe().status(), GenerationQuiescenceProbeStatus::kNotSealed);
  auto deposit = begin(gate);
  ASSERT_TRUE(deposit);
  seal(gate, epoch);
  EXPECT_EQ(gate.snapshot().accepted_ack_cookie, 1U);
  EXPECT_EQ(
    gate.prepare_probe().status(),
    GenerationQuiescenceProbeStatus::kDepositsOutstanding);
  EXPECT_EQ(
    gate.complete_deposit(*deposit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kCompleted);
  EXPECT_EQ(
    gate.complete_deposit(*deposit, GenerationDepositCompletion::kAccepted),
    GenerationDepositCompletionStatus::kNotLive);

  auto probe_decision = gate.prepare_probe();
  auto probe = probe_decision.take_probe_permit();
  ASSERT_TRUE(probe);
  EXPECT_EQ(
    gate.mark_synchronization_failure(),
    GenerationSynchronizationFailureStatus::kMarkedFailed);
  auto rejected = gate.complete_probe(*probe, kGeneration, true);
  EXPECT_EQ(
    rejected.status(), GenerationQuiescenceCompletionStatus::kSynchronizationFailed);
  EXPECT_TRUE(probe->live());
  EXPECT_EQ(
    gate.begin_deposit(goal_id(), kGeneration).status(),
    GenerationDepositBeginStatus::kSynchronizationFailed);
}

TEST(CoordinatorGenerationQuiescencePrecedenceTest, CallerIdentityFailurePreservesReceipt)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto receipt = prepare_receipt(gate, epoch);
  ASSERT_TRUE(receipt);
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(3U), kGeneration),
    GenerationQuiescenceConsumeStatus::kIdentityMismatch);
  EXPECT_TRUE(receipt->live());
  EXPECT_TRUE(gate.snapshot().adapter_fault);
  EXPECT_EQ(
    gate.consume_receipt(*receipt, goal_id(), kGeneration),
    GenerationQuiescenceConsumeStatus::kRetirementFenceFailed);
  EXPECT_TRUE(receipt->live());
}

TEST(CoordinatorGenerationQuiescenceAllocationTest, TransitionsAllocateNothingAfterLock)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  CoordinatorGenerationQuiescence gate(gate_goal(epoch), gate_gen(epoch), 1U);
  auto witness = accepted_witness(epoch);
  ASSERT_TRUE(witness);

  allocations_before_failure = 0U;
  auto begin_decision = gate.begin_deposit(goal_id(), kGeneration);
  allocations_before_failure.reset();
  auto deposit = begin_decision.take_deposit_permit();
  ASSERT_TRUE(deposit);

  allocations_before_failure = 0U;
  const auto completion =
    gate.complete_deposit(*deposit, GenerationDepositCompletion::kAccepted);
  allocations_before_failure.reset();
  EXPECT_EQ(completion, GenerationDepositCompletionStatus::kCompleted);

  allocations_before_failure = 0U;
  const auto seal_status = gate.seal_after_accepted_ack(*witness);
  allocations_before_failure.reset();
  EXPECT_EQ(seal_status, GenerationQuiescenceSealStatus::kSealed);

  allocations_before_failure = 0U;
  const auto snapshot = gate.snapshot();
  allocations_before_failure.reset();
  EXPECT_TRUE(snapshot.sealed);

  allocations_before_failure = 0U;
  auto probe_decision = gate.prepare_probe();
  allocations_before_failure.reset();
  auto probe = probe_decision.take_probe_permit();
  ASSERT_TRUE(probe);

  allocations_before_failure = 0U;
  auto receipt_decision = gate.complete_probe(*probe, kGeneration, true);
  allocations_before_failure.reset();
  auto receipt = receipt_decision.take_receipt();
  ASSERT_TRUE(receipt);

  allocations_before_failure = 0U;
  const auto consume_status = gate.consume_receipt(*receipt, goal_id(), kGeneration);
  allocations_before_failure.reset();
  EXPECT_EQ(consume_status, GenerationQuiescenceConsumeStatus::kConsumed);
}

}  // namespace
}  // namespace restocker_task_executor
