// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

#include "detail/pending_route_claimant.hpp"
#include "restocker_task_executor/restock_coordinator_inbox.hpp"

namespace restocker_task_executor::detail
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId id{};
  for (std::size_t index = 0U; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

struct ReservedGoal
{
  explicit ReservedGoal(std::uint8_t seed = 1U)
  : id(goal_id(seed))
  {
    admission.update_readiness(true);
    const auto receipt = admission.reserve(id);
    EXPECT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
    generation = receipt.generation;
  }

  [[nodiscard]] PendingHandoffBindingKey binding(
    PendingBindingIncarnation incarnation = 1U) const noexcept
  {
    return PendingHandoffBindingKey{id, generation, incarnation};
  }

  GoalAdmissionSlot admission;
  CoordinatorGoalId id{};
  GoalGeneration generation{0U};
};

[[nodiscard]] std::optional<PendingRouteWorkToken> claim_route(
  PendingAcceptedHandoffMachine & machine, const PendingHandoffBindingKey & binding,
  PendingRouteKind kind)
{
  auto claim = machine.observe_route(binding, kind, PendingRouteObservationRelation::kNewDistinct);
  EXPECT_EQ(claim.status(), PendingRouteClaimStatus::kClaimed);
  auto token = claim.take_work();
  EXPECT_TRUE(token);
  EXPECT_FALSE(claim.take_work());
  return token;
}

struct RouteExpectation
{
  PendingRouteKind route_kind;
  GoalTerminationKind termination_kind;
  CoordinatorControlKind control_kind;
};

constexpr std::array kRouteExpectations{
  RouteExpectation{PendingRouteKind::kDrain, GoalTerminationKind::kCoordinatorDrain,
    CoordinatorControlKind::kDrainRequested},
  RouteExpectation{PendingRouteKind::kTransformAuthorityLoss, GoalTerminationKind::kAuthorityLoss,
    CoordinatorControlKind::kAuthorityFaultSafeAbortRequested},
  RouteExpectation{PendingRouteKind::kSteadyClockFailure, GoalTerminationKind::kProtocolFailure,
    CoordinatorControlKind::kShutdown},
  RouteExpectation{PendingRouteKind::kCallbackOrAdapterFailure,
    GoalTerminationKind::kProtocolFailure, CoordinatorControlKind::kShutdown},
};

class PromiseReleaseGuard final
{
public:
  explicit PromiseReleaseGuard(std::promise<void> & promise) noexcept
  : promise_(promise) {}

  PromiseReleaseGuard(const PromiseReleaseGuard &) = delete;
  PromiseReleaseGuard(PromiseReleaseGuard &&) = delete;
  PromiseReleaseGuard & operator=(const PromiseReleaseGuard &) = delete;
  PromiseReleaseGuard & operator=(PromiseReleaseGuard &&) = delete;

  ~PromiseReleaseGuard() noexcept {release();}

  void release() noexcept
  {
    if (released_) {
      return;
    }
    released_ = true;
    try {
      promise_.set_value();
    } catch (...) {
      // Test cleanup must not mask the original assertion or strand the worker.
    }
  }

private:
  std::promise<void> & promise_;
  bool released_{false};
};

TEST(PendingRouteExecutorTypeTest, WorkAndCompletionAreSealedMoveOnlyValues)
{
  static_assert(!std::is_default_constructible_v<PendingRouteExecutionWork>);
  static_assert(!std::is_copy_constructible_v<PendingRouteExecutionWork>);
  static_assert(!std::is_copy_assignable_v<PendingRouteExecutionWork>);
  static_assert(std::is_nothrow_move_constructible_v<PendingRouteExecutionWork>);
  static_assert(!std::is_move_assignable_v<PendingRouteExecutionWork>);
  static_assert(std::is_nothrow_destructible_v<PendingRouteExecutionWork>);
  static_assert(
    std::is_constructible_v<PendingRouteExecutionWork, PendingRouteWorkToken &&,
    SteadyTime, std::string &&>);
  static_assert(
    !std::is_constructible_v<PendingRouteExecutionWork, PendingRouteWorkToken &,
    SteadyTime, std::string &&>);
  static_assert(
    !std::is_constructible_v<PendingRouteExecutionWork, PendingRouteWorkToken &&,
    SteadyTime, std::string &>);

  static_assert(!std::is_default_constructible_v<PendingRouteExecutionCompletion>);
  static_assert(!std::is_copy_constructible_v<PendingRouteExecutionCompletion>);
  static_assert(!std::is_copy_assignable_v<PendingRouteExecutionCompletion>);
  static_assert(std::is_nothrow_move_constructible_v<PendingRouteExecutionCompletion>);
  static_assert(!std::is_move_assignable_v<PendingRouteExecutionCompletion>);
  static_assert(std::is_nothrow_destructible_v<PendingRouteExecutionCompletion>);
  static_assert(std::is_trivially_copyable_v<PendingRouteExecutionStatus>);
  static_assert(std::is_constructible_v<PendingRouteRouterAdapter, CoordinatorTerminationRouter &>);
  static_assert(
    std::is_constructible_v<PendingRouteRouterAdapter, const CoordinatorTerminationRouter &>);
  static_assert(
    !std::is_constructible_v<PendingRouteRouterAdapter, CoordinatorTerminationRouter &&>);
  static_assert(
    !std::is_constructible_v<PendingRouteRouterAdapter, const CoordinatorTerminationRouter && >);

  static_assert(noexcept(std::declval<const PendingRouteExecutionWork &>().token()));
  static_assert(noexcept(std::declval<const PendingRouteExecutionWork &>().binding()));
  static_assert(noexcept(std::declval<const PendingRouteExecutionWork &>().kind()));
  static_assert(noexcept(std::declval<const PendingRouteExecutionWork &>().arrived_at()));
  static_assert(noexcept(std::declval<const PendingRouteExecutionWork &>().detail()));
  static_assert(noexcept(std::declval<PendingRouteExecutionCompletion &>().token()));
  static_assert(noexcept(std::declval<PendingRouteExecutionCompletion &>().take_decision()));
  static_assert(
    noexcept(PendingRouteExecutor::execute(
      std::declval<PendingRouteExecutionWork &&>(), std::declval<PendingRouteRouterAdapter &>())));
  SUCCEED();
}

TEST(PendingRouteExecutor, ForwardsExactOwnedInputsOnceAndLeavesReducerUntouched)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding(7U);
  PendingAcceptedHandoffMachine machine(binding);
  auto token = claim_route(machine, binding, PendingRouteKind::kDrain);
  ASSERT_TRUE(token);
  const auto arrived_at = SteadyTime{} + 37ms;
  const std::string expected_detail{"drain accepted handoff"};
  PendingRouteExecutionWork work{std::move(*token), arrived_at, std::string{expected_detail}};
  CoordinatorTerminationRouter router{reserved.admission};
  PendingRouteRouterAdapter adapter{router};
  int calls = 0;

  auto completion = PendingRouteExecutor::execute(
    std::move(work), [&calls, &adapter, &binding, arrived_at,
    &expected_detail](const PendingRouteExecutionWork & observed) {
      ++calls;
      EXPECT_EQ(observed.binding(), binding);
      EXPECT_EQ(observed.kind(), PendingRouteKind::kDrain);
      EXPECT_EQ(observed.delivery_class(), PendingRouteDeliveryClass::kPreSeal);
      EXPECT_EQ(observed.route_token(), 1U);
      EXPECT_EQ(observed.arrived_at(), arrived_at);
      EXPECT_EQ(observed.detail(), expected_detail);
      return adapter(observed);
    });

  EXPECT_EQ(calls, 1);
  EXPECT_EQ(completion.status(), PendingRouteExecutionStatus::kCompleted);
  EXPECT_TRUE(completion.has_decision());
  EXPECT_TRUE(completion.token().live());
  EXPECT_EQ(completion.token().binding(), binding);
  ASSERT_TRUE(machine.snapshot().route_work);
  EXPECT_EQ(machine.snapshot().route_work->token, completion.token().token());

  auto decision = completion.take_decision();
  ASSERT_TRUE(decision);
  EXPECT_FALSE(completion.has_decision());
  EXPECT_FALSE(completion.take_decision());
  ASSERT_TRUE(decision->event());
  EXPECT_EQ(decision->event()->detail, expected_detail);
  EXPECT_EQ(decision->event()->operation_generation, 0U);

  EXPECT_EQ(
    machine.complete_route(
      completion.token(), PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kMerged);
  EXPECT_FALSE(completion.token().live());
  EXPECT_FALSE(machine.snapshot().route_work);
}

TEST(PendingRouteRouterAdapter, EveryPendingKindProducesExactAdmissionAndEventEvidence)
{
  for (std::size_t index = 0U; index < kRouteExpectations.size(); ++index) {
    const auto & expected = kRouteExpectations[index];
    SCOPED_TRACE(index);
    ReservedGoal reserved(static_cast<std::uint8_t>(index + 1U));
    const auto binding = reserved.binding(static_cast<PendingBindingIncarnation>(index + 1U));
    PendingAcceptedHandoffMachine machine(binding);
    auto token = claim_route(machine, binding, expected.route_kind);
    ASSERT_TRUE(token);
    const auto arrived_at = SteadyTime{} + std::chrono::milliseconds(50 + index);
    const std::string expected_detail = "route kind " + std::to_string(index);
    PendingRouteExecutionWork work{std::move(*token), arrived_at, std::string{expected_detail}};
    CoordinatorTerminationRouter router{reserved.admission};

    auto completion =
      PendingRouteExecutor::execute(std::move(work), PendingRouteRouterAdapter{router});

    EXPECT_EQ(completion.status(), PendingRouteExecutionStatus::kCompleted);
    ASSERT_TRUE(completion.decision());
    const auto & decision = *completion.decision();
    EXPECT_EQ(decision.admission_decision().status(), GoalTerminationLatchStatus::kLatched);
    const auto admission_record = decision.admission_decision().record();
    const auto snapshot = reserved.admission.snapshot();
    ASSERT_TRUE(admission_record);
    ASSERT_TRUE(snapshot.first_termination);
    EXPECT_EQ(admission_record.get(), snapshot.first_termination.get());
    EXPECT_EQ(admission_record->goal_id, binding.goal_id);
    EXPECT_EQ(admission_record->goal_generation, binding.generation);
    EXPECT_EQ(admission_record->kind, expected.termination_kind);
    EXPECT_EQ(admission_record->arrived_at, arrived_at);
    ASSERT_TRUE(decision.event());
    EXPECT_EQ(decision.event()->kind, expected.control_kind);
    EXPECT_EQ(decision.event()->goal_generation, binding.generation);
    EXPECT_EQ(decision.event()->operation_generation, 0U);
    EXPECT_EQ(decision.event()->arrived_at, arrived_at);
    EXPECT_EQ(decision.event()->detail, expected_detail);
    EXPECT_FALSE(decision.diagnostic_degraded());

    EXPECT_EQ(
      machine.complete_route(
        completion.token(), PendingRouteEvidenceResult::kExact,
        PendingControlMergeDecision::kStoreIncoming),
      PendingRouteCompletionStatus::kMerged);
    EXPECT_FALSE(completion.token().live());
  }
}

TEST(PendingRouteExecutor, StandardAndNonStandardExceptionsRetainAuthenticTokens)
{
  for (const bool standard_exception : {true, false}) {
    SCOPED_TRACE(standard_exception);
    ReservedGoal reserved(standard_exception ? 10U : 20U);
    const auto binding = reserved.binding(standard_exception ? 10U : 20U);
    PendingAcceptedHandoffMachine machine(binding);
    auto token = claim_route(machine, binding, PendingRouteKind::kCallbackOrAdapterFailure);
    ASSERT_TRUE(token);
    PendingRouteExecutionWork work{std::move(*token), SteadyTime{} + 75ms,
      std::string{"throwing route"}};
    int calls = 0;

    auto throwing_router =
      [&calls, standard_exception](
      const PendingRouteExecutionWork &) -> CoordinatorTerminationRouteDecision {
        ++calls;
        if (standard_exception) {
          throw std::runtime_error("router failed");
        }
        throw 17;
      };
    auto completion = PendingRouteExecutor::execute(std::move(work), throwing_router);

    EXPECT_EQ(calls, 1);
    EXPECT_EQ(
      completion.status(), standard_exception ?
      PendingRouteExecutionStatus::kStandardException :
      PendingRouteExecutionStatus::kNonStandardException);
    EXPECT_FALSE(completion.has_decision());
    EXPECT_EQ(completion.work().binding(), binding);
    EXPECT_TRUE(completion.token().live());
    ASSERT_TRUE(machine.snapshot().route_work);
    EXPECT_EQ(machine.snapshot().route_work->token, completion.token().token());

    EXPECT_EQ(
      machine.complete_route(
        completion.token(), PendingRouteEvidenceResult::kInvalid,
        PendingControlMergeDecision::kCleanupOnly),
      PendingRouteCompletionStatus::kCleanupOnly);
    EXPECT_FALSE(completion.token().live());
    EXPECT_FALSE(machine.snapshot().route_work);
  }
}

TEST(PendingRouteExecutor, RejectsMovedFromWorkWithoutCallingRouter)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding();
  PendingAcceptedHandoffMachine machine(binding);
  auto token = claim_route(machine, binding, PendingRouteKind::kDrain);
  ASSERT_TRUE(token);
  PendingRouteExecutionWork original{std::move(*token), SteadyTime{} + 90ms,
    std::string{"owned by live copy"}};
  PendingRouteExecutionWork live{std::move(original)};
  CoordinatorTerminationRouter router{reserved.admission};
  PendingRouteRouterAdapter adapter{router};
  int calls = 0;

  auto invalid = PendingRouteExecutor::execute(
    std::move(original), [&calls, &adapter](const PendingRouteExecutionWork & work) {
      ++calls;
      return adapter(work);
    });

  EXPECT_EQ(calls, 0);
  EXPECT_EQ(invalid.status(), PendingRouteExecutionStatus::kInvalidWork);
  EXPECT_FALSE(invalid.has_decision());
  EXPECT_FALSE(invalid.token().live());
  EXPECT_EQ(invalid.token().binding(), binding);
  EXPECT_EQ(invalid.token().kind(), PendingRouteKind::kDrain);
  EXPECT_EQ(invalid.token().delivery_class(), PendingRouteDeliveryClass::kPreSeal);
  EXPECT_EQ(invalid.token().token(), 1U);
  ASSERT_TRUE(machine.snapshot().route_work);

  auto completed = PendingRouteExecutor::execute(std::move(live), adapter);
  EXPECT_EQ(completed.status(), PendingRouteExecutionStatus::kCompleted);
  EXPECT_TRUE(completed.token().live());
  EXPECT_EQ(
    machine.complete_route(
      completed.token(), PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kMerged);
}

TEST(PendingRouteExecutor, BlockingRouterLeavesRealReducerRouteAuthorityUnchanged)
{
  // This detail test proves capability stability across blocking external work. The node
  // integration suite owns proof that the binding mutex is released before this boundary.
  ReservedGoal reserved;
  const auto binding = reserved.binding();
  PendingAcceptedHandoffMachine machine(binding);
  auto token = claim_route(machine, binding, PendingRouteKind::kDrain);
  ASSERT_TRUE(token);
  const auto expected_route_token = token->token();
  const auto before = machine.snapshot();
  ASSERT_TRUE(before.route_work);
  PendingRouteExecutionWork work{std::move(*token), SteadyTime{} + 100ms,
    std::string{"blocking route"}};
  CoordinatorTerminationRouter router{reserved.admission};
  PendingRouteRouterAdapter adapter{router};
  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  std::atomic_bool release_timed_out{false};

  auto completion_future = std::async(
    std::launch::async,
    [&entered_promise, release, &release_timed_out, &adapter, work = std::move(work)]() mutable {
      return PendingRouteExecutor::execute(
        std::move(work), [&entered_promise, release, &release_timed_out,
        &adapter](const PendingRouteExecutionWork & observed) {
          entered_promise.set_value();
          if (release.wait_for(5s) != std::future_status::ready) {
            release_timed_out.store(true);
          }
          return adapter(observed);
        });
    });
  // Declared after the future so early-return destruction releases the worker first.
  PromiseReleaseGuard release_guard{release_promise};

  ASSERT_EQ(entered.wait_for(2s), std::future_status::ready);
  EXPECT_EQ(completion_future.wait_for(20ms), std::future_status::timeout);
  const auto blocked = machine.snapshot();
  EXPECT_TRUE(machine.valid());
  EXPECT_EQ(blocked.binding, before.binding);
  EXPECT_EQ(blocked.phase, before.phase);
  EXPECT_EQ(blocked.route_evidence_revision, before.route_evidence_revision);
  EXPECT_EQ(blocked.last_route_token, before.last_route_token);
  ASSERT_TRUE(blocked.route_work);
  EXPECT_EQ(blocked.route_work->kind, before.route_work->kind);
  EXPECT_EQ(blocked.route_work->delivery_class, before.route_work->delivery_class);
  EXPECT_EQ(blocked.route_work->token, before.route_work->token);
  EXPECT_EQ(blocked.route_work->token, expected_route_token);

  release_guard.release();
  ASSERT_EQ(completion_future.wait_for(2s), std::future_status::ready);
  auto completion = completion_future.get();

  EXPECT_FALSE(release_timed_out.load());
  EXPECT_EQ(completion.status(), PendingRouteExecutionStatus::kCompleted);
  EXPECT_TRUE(completion.token().live());
  ASSERT_TRUE(machine.snapshot().route_work);
  EXPECT_EQ(
    machine.complete_route(
      completion.token(), PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kMerged);
}

TEST(PendingRouteClaimant, InstallsReducerTokenMetadataAndSealedWorkTogether)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding(41U);
  PendingAcceptedHandoffMachine machine(binding);
  PendingRouteMetadataSlots metadata;
  std::optional<PendingRouteExecutionWork> execution;
  const auto arrived_at = SteadyTime{} + 111ms;
  std::string work_detail{"atomic pending claim"};
  std::string metadata_detail{work_detail};

  EXPECT_EQ(
    PendingRouteClaimant::claim(
      machine, binding, metadata, PendingRouteKind::kDrain,
      arrived_at, work_detail, metadata_detail, execution),
    PendingRouteClaimStatus::kClaimed);

  ASSERT_TRUE(execution);
  EXPECT_TRUE(work_detail.empty());
  EXPECT_TRUE(metadata_detail.empty());
  EXPECT_EQ(execution->binding(), binding);
  EXPECT_EQ(execution->kind(), PendingRouteKind::kDrain);
  EXPECT_EQ(execution->arrived_at(), arrived_at);
  EXPECT_EQ(execution->detail(), "atomic pending claim");
  const auto snapshot = machine.snapshot();
  ASSERT_TRUE(snapshot.route_work);
  EXPECT_EQ(snapshot.route_work->token, execution->route_token());
  ASSERT_TRUE(metadata[0U]);
  EXPECT_EQ(metadata[0U]->kind, PendingRouteKind::kDrain);
  EXPECT_EQ(metadata[0U]->arrived_at, arrived_at);
  EXPECT_EQ(metadata[0U]->detail, execution->detail());
}

TEST(PendingRouteClaimant, CoalescesExactObservationAndRejectsAnyDistinctCollision)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding(42U);
  PendingAcceptedHandoffMachine machine(binding);
  PendingRouteMetadataSlots metadata;
  std::optional<PendingRouteExecutionWork> execution;
  const auto arrived_at = SteadyTime{} + 112ms;
  std::string work_detail{"same observation"};
  std::string metadata_detail{work_detail};
  ASSERT_EQ(
    PendingRouteClaimant::claim(
      machine, binding, metadata, PendingRouteKind::kDrain,
      arrived_at, work_detail, metadata_detail, execution),
    PendingRouteClaimStatus::kClaimed);

  std::optional<PendingRouteExecutionWork> duplicate_execution;
  std::string duplicate_work_detail{"same observation"};
  std::string duplicate_metadata_detail{duplicate_work_detail};
  EXPECT_EQ(
    PendingRouteClaimant::claim(
      machine, binding, metadata, PendingRouteKind::kDrain,
      arrived_at, duplicate_work_detail,
      duplicate_metadata_detail, duplicate_execution),
    PendingRouteClaimStatus::kCoalesced);
  EXPECT_FALSE(duplicate_execution);
  EXPECT_EQ(duplicate_work_detail, "same observation");
  EXPECT_EQ(duplicate_metadata_detail, "same observation");

  std::optional<PendingRouteExecutionWork> collision_execution;
  std::string collision_work_detail{"different observation"};
  std::string collision_metadata_detail{collision_work_detail};
  EXPECT_EQ(
    PendingRouteClaimant::claim(
      machine, binding, metadata, PendingRouteKind::kDrain,
      arrived_at + 1ms, collision_work_detail,
      collision_metadata_detail, collision_execution),
    PendingRouteClaimStatus::kCleanupOnly);
  EXPECT_FALSE(collision_execution);
  EXPECT_EQ(machine.snapshot().phase, PendingAcceptedHandoffPhase::kCleanupOnly);
}

TEST(PendingRouteClaimant, BlockingExecutorCannotOvertakeLiveAtomicClaim)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding(43U);
  PendingAcceptedHandoffMachine machine(binding);
  PendingRouteMetadataSlots metadata;
  std::optional<PendingRouteExecutionWork> execution;
  std::mutex binding_mutex;
  std::string work_detail{"blocked after atomic claim"};
  std::string metadata_detail{work_detail};
  {
    std::lock_guard lock(binding_mutex);
    ASSERT_EQ(
      PendingRouteClaimant::claim(
        machine, binding, metadata, PendingRouteKind::kSteadyClockFailure,
        SteadyTime{} + 113ms, work_detail, metadata_detail, execution),
      PendingRouteClaimStatus::kClaimed);
  }
  ASSERT_TRUE(execution);

  CoordinatorTerminationRouter router{reserved.admission};
  PendingRouteRouterAdapter adapter{router};
  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto completion_future = std::async(
    std::launch::async, [&entered_promise, release, &adapter,
    work = std::move(*execution)]() mutable {
      return PendingRouteExecutor::execute(
        std::move(work),
        [&entered_promise, release, &adapter](const PendingRouteExecutionWork & observed) {
          entered_promise.set_value();
          (void)release.wait_for(5s);
          return adapter(observed);
        });
    });
  PromiseReleaseGuard release_guard{release_promise};

  ASSERT_EQ(entered.wait_for(2s), std::future_status::ready);
  auto diagnostic = std::async(
    std::launch::async, [&binding_mutex, &machine]() {
      std::lock_guard lock(binding_mutex);
      const auto snapshot = machine.snapshot();
      const auto advance = machine.advance({CoordinatorPumpLeaseGateSnapshot{}, false});
      return std::pair{snapshot, advance.status()};
    });
  ASSERT_EQ(diagnostic.wait_for(250ms), std::future_status::ready);
  const auto [blocked_snapshot, advance_status] = diagnostic.get();
  ASSERT_TRUE(blocked_snapshot.route_work);
  EXPECT_EQ(advance_status, PendingHandoffAdvanceStatus::kWait);
  EXPECT_EQ(completion_future.wait_for(20ms), std::future_status::timeout);

  release_guard.release();
  ASSERT_EQ(completion_future.wait_for(2s), std::future_status::ready);
  auto completion = completion_future.get();
  ASSERT_EQ(completion.status(), PendingRouteExecutionStatus::kCompleted);
  {
    std::lock_guard lock(binding_mutex);
    EXPECT_EQ(
      machine.complete_route(
        completion.token(), PendingRouteEvidenceResult::kExact,
        PendingControlMergeDecision::kStoreIncoming),
      PendingRouteCompletionStatus::kMerged);
    EXPECT_FALSE(machine.snapshot().route_work);
  }
}

TEST(PendingRouteClaimant, PostSealRouteCannotOvertakeAcceptedInboxOwnership)
{
  ReservedGoal reserved;
  const auto binding = reserved.binding(44U);
  PendingAcceptedHandoffMachine machine(binding);
  ASSERT_EQ(machine.install_epoch(binding), PendingHandoffPhaseEventStatus::kApplied);
  ASSERT_EQ(machine.adopt_accepted_handle(binding), PendingHandoffPhaseEventStatus::kApplied);
  constexpr CoordinatorPumpLeaseGateSnapshot healthy_gate{1U, false, false, false};
  auto prepare_decision = machine.advance({healthy_gate, false});
  ASSERT_EQ(prepare_decision.status(), PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope);
  auto preparation = prepare_decision.take_work();
  ASSERT_TRUE(preparation);
  auto prepared = machine.complete_preparation(*preparation);
  ASSERT_EQ(prepared.status(), PendingPreparationCompletionStatus::kPublishAcceptedEnvelope);
  auto accepted_publication = prepared.take_handoff_work();
  ASSERT_TRUE(accepted_publication);

  PendingRouteMetadataSlots metadata;
  std::optional<PendingRouteExecutionWork> execution;
  const auto arrived_at = SteadyTime{} + 114ms;
  std::string work_detail{"post-seal transform loss"};
  std::string metadata_detail{work_detail};
  ASSERT_EQ(
    PendingRouteClaimant::claim(
      machine, binding, metadata,
      PendingRouteKind::kTransformAuthorityLoss, arrived_at,
      work_detail, metadata_detail, execution),
    PendingRouteClaimStatus::kClaimed);
  ASSERT_TRUE(execution);
  EXPECT_EQ(execution->delivery_class(), PendingRouteDeliveryClass::kPostSeal);

  CoordinatorTerminationRouter router{reserved.admission};
  PendingRouteRouterAdapter adapter{router};
  std::promise<void> entered_promise;
  auto entered = entered_promise.get_future();
  std::promise<void> release_promise;
  auto release = release_promise.get_future().share();
  auto completion_future = std::async(
    std::launch::async, [&entered_promise, release, &adapter,
    work = std::move(*execution)]() mutable {
      return PendingRouteExecutor::execute(
        std::move(work),
        [&entered_promise, release, &adapter](const PendingRouteExecutionWork & observed) {
          entered_promise.set_value();
          (void)release.wait_for(5s);
          return adapter(observed);
        });
    });
  PromiseReleaseGuard release_guard{release_promise};
  ASSERT_EQ(entered.wait_for(2s), std::future_status::ready);

  CoordinatorInbox inbox{4U};
  CoordinatorAcceptedGoal accepted;
  accepted.goal_id = binding.goal_id;
  accepted.goal_generation = binding.generation;
  accepted.sealed_pending_control_handoff = true;
  const auto accepted_deposit =
    inbox.push_accepted_goal(std::move(accepted), std::nullopt, arrived_at - 1ms);
  ASSERT_EQ(
    classify_accepted_envelope_deposit(accepted_deposit),
    AcceptedEnvelopeDepositDecision::kCommitted);
  ASSERT_EQ(
    machine.complete_accepted_publication(
      *accepted_publication,
      AcceptedEnvelopeDepositDecision::kCommitted),
    PendingAcceptedPublicationStatus::kCommitted);
  EXPECT_EQ(machine.advance({healthy_gate, false}).status(), PendingHandoffAdvanceStatus::kWait);
  EXPECT_EQ(inbox.snapshot().size, 1U);

  release_guard.release();
  ASSERT_EQ(completion_future.wait_for(2s), std::future_status::ready);
  auto completion = completion_future.get();
  ASSERT_EQ(completion.status(), PendingRouteExecutionStatus::kCompleted);
  auto route_decision = completion.take_decision();
  ASSERT_TRUE(route_decision);
  auto deferred = route_decision->take_event();
  ASSERT_TRUE(deferred);
  ASSERT_EQ(
    machine.complete_route(
      completion.token(), PendingRouteEvidenceResult::kExact,
      PendingControlMergeDecision::kStoreIncoming),
    PendingRouteCompletionStatus::kMerged);

  auto deferred_decision = machine.advance({healthy_gate, true});
  ASSERT_EQ(deferred_decision.status(), PendingHandoffAdvanceStatus::kPublishDeferredControl);
  auto deferred_publication = deferred_decision.take_work();
  ASSERT_TRUE(deferred_publication);
  const auto deferred_deposit = inbox.push(std::move(*deferred));
  ASSERT_EQ(
    classify_deferred_control_deposit(deferred_deposit),
    DeferredControlDepositDecision::kDelivered);
  ASSERT_EQ(
    machine.complete_deferred_publication(
      *deferred_publication,
      PendingDeferredCompletion::kDelivered),
    PendingDeferredPublicationStatus::kCompleted);
  EXPECT_EQ(
    machine.advance({healthy_gate, false}).status(),
    PendingHandoffAdvanceStatus::kReadyToActivate);

  auto first = inbox.try_pop();
  auto second = inbox.try_pop();
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  EXPECT_NE(std::get_if<CoordinatorAcceptedGoal>(&*first), nullptr);
  const auto * control = std::get_if<CoordinatorControlEvent>(&*second);
  ASSERT_NE(control, nullptr);
  EXPECT_EQ(control->kind, CoordinatorControlKind::kAuthorityFaultSafeAbortRequested);
  EXPECT_EQ(control->goal_generation, binding.generation);
  EXPECT_EQ(control->arrived_at, arrived_at);
  EXPECT_EQ(control->detail, "post-seal transform loss");
  EXPECT_FALSE(inbox.try_pop());
}

}  // namespace
}  // namespace restocker_task_executor::detail
