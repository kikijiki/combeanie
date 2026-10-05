// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include "restocker_task_executor/generation_scoped_inbox_deposit.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

constexpr GoalGeneration kGeneration = 11U;

CoordinatorGoalId goal_id(std::uint8_t seed = 3U)
{
  CoordinatorGoalId id{};
  id.front() = seed;
  id.back() = static_cast<std::uint8_t>(seed + 1U);
  return id;
}

struct GateFixture
{
  explicit GateFixture(std::size_t capacity = 4U)
  : epoch(goal_id(), kGeneration),
    gate(std::make_shared<CoordinatorGenerationQuiescence>(
        goal_id(), kGeneration, capacity))
  {
  }

  CoordinatorActiveFaultEpoch epoch;
  std::shared_ptr<CoordinatorGenerationQuiescence> gate;
};

CoordinatorAcceptedTerminalAckWitness accepted_witness(CoordinatorActiveFaultEpoch & epoch)
{
  CoordinatorDriverOutput output;
  output.kind = CoordinatorDriverOutputKind::kSucceeded;
  output.goal_generation = kGeneration;
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
  auto completed = epoch.complete_terminal_ack(*acknowledgement, true);
  EXPECT_EQ(completed.status(), ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  auto witness = completed.take_accepted_ack_witness();
  EXPECT_TRUE(witness);
  return std::move(*witness);
}

void seal(GateFixture & fixture)
{
  auto witness = accepted_witness(fixture.epoch);
  ASSERT_EQ(
    fixture.gate->seal_after_accepted_ack(witness),
    GenerationQuiescenceSealStatus::kSealed);
}

constexpr CoordinatorInboxDepositResult kAccepted{CoordinatorInboxDepositStatus::kAccepted,
  CoordinatorInboxPersistenceStatus::kEventOwned};

struct ObservableDeposit
{
  ObservableDeposit(
    std::size_t & copies_value, std::size_t & moves_value,
    std::size_t & destructions_value, std::size_t & inspections_value,
    std::size_t & clock_samples_value)
  : copies(copies_value), moves(moves_value), destructions(destructions_value),
    inspections(inspections_value), clock_samples(clock_samples_value)
  {
  }

  ObservableDeposit(const ObservableDeposit & other)
  : copies(other.copies), moves(other.moves), destructions(other.destructions),
    inspections(other.inspections), clock_samples(other.clock_samples)
  {
    ++copies;
  }

  ObservableDeposit(ObservableDeposit && other) noexcept
  : copies(other.copies), moves(other.moves), destructions(other.destructions),
    inspections(other.inspections), clock_samples(other.clock_samples)
  {
    ++moves;
  }

  ~ObservableDeposit() {++destructions;}

  CoordinatorInboxDepositResult operator()()
  {
    ++inspections;
    ++clock_samples;
    return kAccepted;
  }

  std::size_t & copies;
  std::size_t & moves;
  std::size_t & destructions;
  std::size_t & inspections;
  std::size_t & clock_samples;
};

TEST(GenerationScopedInboxDepositTest, ClassifierImplementsOnlyClosedKnownRows)
{
  struct KnownCase
  {
    CoordinatorInboxDepositResult result;
    GenerationDepositCompletion expected;
  };
  constexpr std::array known{
    KnownCase{{CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned},
      GenerationDepositCompletion::kAccepted},
    KnownCase{{CoordinatorInboxDepositStatus::kOverflowLatched,
      CoordinatorInboxPersistenceStatus::kEventOwned},
      GenerationDepositCompletion::kAccepted},
    KnownCase{{CoordinatorInboxDepositStatus::kOverflowLatched,
      CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned},
      GenerationDepositCompletion::kOverflowAccounted},
    KnownCase{{CoordinatorInboxDepositStatus::kInhibited,
      CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned},
      GenerationDepositCompletion::kInhibitedAccounted},
    KnownCase{{CoordinatorInboxDepositStatus::kEvidenceLost,
      CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned},
      GenerationDepositCompletion::kEvidenceLostAccounted}};

  constexpr std::array statuses{
    CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxDepositStatus::kInhibited, CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxDepositStatus::kEvidenceConflict};
  constexpr std::array persistence_values{
    CoordinatorInboxPersistenceStatus::kEventOwned,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned,
    CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned,
    CoordinatorInboxPersistenceStatus::kUnresolved};

  for (const auto status : statuses) {
    for (const auto persistence : persistence_values) {
      const CoordinatorInboxDepositResult candidate{status, persistence};
      auto expected = GenerationDepositCompletion::kUnknown;
      for (const auto & test_case : known) {
        if (test_case.result.status == status && test_case.result.persistence == persistence) {
          expected = test_case.expected;
        }
      }
      EXPECT_EQ(classify_generation_deposit(candidate), expected);
    }
  }

  EXPECT_EQ(
    classify_generation_deposit(
      {static_cast<CoordinatorInboxDepositStatus>(255U),
        CoordinatorInboxPersistenceStatus::kEventOwned}),
    GenerationDepositCompletion::kUnknown);
  EXPECT_EQ(
    classify_generation_deposit(
      {CoordinatorInboxDepositStatus::kAccepted,
        static_cast<CoordinatorInboxPersistenceStatus>(255U)}),
    GenerationDepositCompletion::kUnknown);
}

TEST(GenerationScopedInboxDepositTest, KnownResultCompletesExactlyOnePermit)
{
  GateFixture fixture;
  std::size_t invocations = 0U;

  const auto result = deposit_for_generation(
    fixture.gate, goal_id(), kGeneration, [&]() {
      ++invocations;
      EXPECT_EQ(fixture.gate->snapshot().active_deposit_count, 1U);
      return kAccepted;
    });

  EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kDeposited);
  EXPECT_EQ(result.inbox.status, kAccepted.status);
  EXPECT_EQ(result.inbox.persistence, kAccepted.persistence);
  EXPECT_EQ(invocations, 1U);
  const auto snapshot = fixture.gate->snapshot();
  EXPECT_EQ(snapshot.active_deposit_count, 0U);
  EXPECT_FALSE(snapshot.deposit_unresolved);
  EXPECT_FALSE(snapshot.adapter_fault);
}

TEST(GenerationScopedInboxDepositTest, ClosedUnknownResultsArePersistentlyUnresolved)
{
  for (const auto status : {
        CoordinatorInboxDepositStatus::kInhibited,
        CoordinatorInboxDepositStatus::kEvidenceConflict})
  {
    GateFixture fixture;
    const CoordinatorInboxDepositResult unresolved{
      status, CoordinatorInboxPersistenceStatus::kUnresolved};

    const auto result = deposit_for_generation(
      fixture.gate, goal_id(), kGeneration,
      [unresolved]() {return unresolved;});

    EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kUnresolved);
    EXPECT_EQ(result.inbox.status, unresolved.status);
    EXPECT_EQ(result.inbox.persistence, unresolved.persistence);
    const auto snapshot = fixture.gate->snapshot();
    EXPECT_EQ(snapshot.active_deposit_count, 0U);
    EXPECT_TRUE(snapshot.deposit_unresolved);
    EXPECT_FALSE(snapshot.adapter_fault);
  }
}

TEST(GenerationScopedInboxDepositTest, SealedGateDoesNotInteractWithObservableDeposit)
{
  GateFixture fixture;
  seal(fixture);
  std::size_t copies = 0U;
  std::size_t moves = 0U;
  std::size_t destructions = 0U;
  std::size_t inspections = 0U;
  std::size_t clock_samples = 0U;
  GenerationScopedInboxDepositResult result;
  {
    ObservableDeposit deposit{copies, moves, destructions, inspections, clock_samples};
    result = deposit_for_generation(fixture.gate, goal_id(), kGeneration, deposit);
    EXPECT_EQ(copies, 0U);
    EXPECT_EQ(moves, 0U);
    EXPECT_EQ(destructions, 0U);
    EXPECT_EQ(inspections, 0U);
    EXPECT_EQ(clock_samples, 0U);
  }

  EXPECT_EQ(destructions, 1U);
  EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kSealed);
  EXPECT_EQ(result.inbox.status, CoordinatorInboxDepositStatus::kEvidenceConflict);
  EXPECT_EQ(result.inbox.persistence, CoordinatorInboxPersistenceStatus::kUnresolved);
  EXPECT_EQ(fixture.gate->snapshot().active_deposit_count, 0U);
}

TEST(GenerationScopedInboxDepositTest, InvalidAuthorityNeverInvokesDepositAndFailsGate)
{
  for (const auto test_case : {0U, 1U, 2U, 3U, 4U}) {
    GateFixture fixture(test_case == 2U ? 1U : 4U);
    std::shared_ptr<CoordinatorGenerationQuiescence> supplied_gate = fixture.gate;
    auto supplied_id = goal_id();
    auto supplied_generation = kGeneration;
    std::optional<CoordinatorDepositPermit> capacity_permit;

    if (test_case == 0U) {
      supplied_gate.reset();
    } else if (test_case == 1U) {
      supplied_id = goal_id(44U);
    } else if (test_case == 4U) {
      supplied_generation = kGeneration + 1U;
    } else if (test_case == 2U) {
      auto beginning = fixture.gate->begin_deposit(goal_id(), kGeneration);
      ASSERT_EQ(beginning.status(), GenerationDepositBeginStatus::kStarted);
      capacity_permit = beginning.take_deposit_permit();
      ASSERT_TRUE(capacity_permit);
    } else {
      EXPECT_EQ(
        fixture.gate->mark_synchronization_failure(),
        GenerationSynchronizationFailureStatus::kMarkedFailed);
    }

    std::size_t invocations = 0U;
    const auto result =
      deposit_for_generation(
      supplied_gate, supplied_id, supplied_generation, [&]() {
        ++invocations;
        return kAccepted;
      });

    EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kFailed);
    EXPECT_EQ(result.inbox.status, CoordinatorInboxDepositStatus::kEvidenceConflict);
    EXPECT_EQ(result.inbox.persistence, CoordinatorInboxPersistenceStatus::kUnresolved);
    EXPECT_EQ(invocations, 0U);
    if (supplied_gate) {
      EXPECT_EQ(
        fixture.gate->snapshot().retirement_fence_state,
        GenerationRetirementFenceState::kFailed);
    }
  }
}

TEST(GenerationScopedInboxDepositTest, ThrowingDepositIsCaughtWithoutRetry)
{
  GateFixture fixture;
  std::size_t invocations = 0U;

  const auto result = deposit_for_generation(
    fixture.gate, goal_id(), kGeneration,
    [&]() -> CoordinatorInboxDepositResult {
      ++invocations;
      throw std::runtime_error("injected deposit failure");
    });

  EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kFailed);
  EXPECT_EQ(result.inbox.status, CoordinatorInboxDepositStatus::kEvidenceConflict);
  EXPECT_EQ(result.inbox.persistence, CoordinatorInboxPersistenceStatus::kUnresolved);
  EXPECT_EQ(invocations, 1U);
  const auto snapshot = fixture.gate->snapshot();
  EXPECT_EQ(snapshot.retirement_fence_state, GenerationRetirementFenceState::kFailed);
  EXPECT_FALSE(snapshot.adapter_fault);
  EXPECT_EQ(snapshot.active_deposit_count, 1U);
}

TEST(GenerationScopedInboxDepositTest, ConcurrentCallsOwnIndependentLivePermits)
{
  GateFixture fixture(2U);
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  std::atomic<std::size_t> invocations{0U};
  const auto invoke = [&]() {
    return deposit_for_generation(
      fixture.gate, goal_id(), kGeneration, [&]() {
        invocations.fetch_add(1U, std::memory_order_relaxed);
        release.wait();
        return kAccepted;
      });
  };

  auto first = std::async(std::launch::async, invoke);
  auto second = std::async(std::launch::async, invoke);
  const auto entered_deadline = std::chrono::steady_clock::now() + 2s;
  while (invocations.load(std::memory_order_relaxed) != 2U &&
    std::chrono::steady_clock::now() < entered_deadline)
  {
    std::this_thread::yield();
  }
  const auto entered_count = invocations.load(std::memory_order_relaxed);
  if (entered_count == 2U) {
    EXPECT_EQ(fixture.gate->snapshot().active_deposit_count, 2U);
  }
  release_promise.set_value();

  ASSERT_EQ(first.wait_for(2s), std::future_status::ready);
  ASSERT_EQ(second.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(first.get().outcome, GenerationScopedInboxDepositOutcome::kDeposited);
  EXPECT_EQ(second.get().outcome, GenerationScopedInboxDepositOutcome::kDeposited);
  EXPECT_EQ(entered_count, 2U);
  EXPECT_EQ(fixture.gate->snapshot().active_deposit_count, 0U);
}

static_assert(std::is_trivially_copyable_v<GenerationScopedInboxDepositResult>);
static_assert(std::is_nothrow_move_assignable_v<GenerationScopedInboxDepositResult>);
static_assert(noexcept(classify_generation_deposit(CoordinatorInboxDepositResult{})));

}  // namespace
}  // namespace restocker_task_executor
