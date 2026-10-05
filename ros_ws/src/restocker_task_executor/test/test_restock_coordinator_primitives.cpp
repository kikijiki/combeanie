// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <barrier>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <new>
#include <set>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

namespace
{

using namespace std::chrono_literals;

TEST(GoalAdmissionGeneration, ReservesMaximumGenerationForNonGoalAuthority)
{
  EXPECT_FALSE(admissible_goal_generation(0U));
  EXPECT_TRUE(admissible_goal_generation(kReservedCoordinatorGoalGeneration - 1U));
  EXPECT_FALSE(admissible_goal_generation(kReservedCoordinatorGoalGeneration));
  EXPECT_EQ(
    kReservedCoordinatorGoalGeneration, std::numeric_limits<GoalGeneration>::max());
}

CoordinatorGoalId goal_id(std::uint8_t value)
{
  CoordinatorGoalId id{};
  id.front() = value;
  return id;
}

class BoundedRaceGate final
{
public:
  void arrive_and_wait()
  {
    std::unique_lock lock(mutex_);
    ++arrivals_;
    condition_.notify_all();
    condition_.wait(lock, [this]() {return released_;});
  }

  [[nodiscard]] bool wait_for_arrival(std::chrono::milliseconds timeout)
  {
    return wait_for_arrivals(1U, timeout);
  }

  [[nodiscard]] bool wait_for_arrivals(
    std::size_t count, std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this, count]() {return arrivals_ >= count;});
  }

  void release()
  {
    std::scoped_lock lock(mutex_);
    released_ = true;
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  std::size_t arrivals_{0U};
  bool released_{false};
};

[[nodiscard]] GoalTerminationLatchDecision request_termination_kind(
  GoalAdmissionSlot & slot, GoalTerminationKind kind, const CoordinatorGoalId & id,
  GoalGeneration generation, SteadyTime arrived_at)
{
  switch (kind) {
    case GoalTerminationKind::kUserCancel:
      return slot.request_cancel(id, generation, arrived_at);
    case GoalTerminationKind::kCoordinatorDrain:
      return slot.request_drain(id, generation, arrived_at);
    case GoalTerminationKind::kSafeAbort:
      return slot.request_safe_abort(id, generation, arrived_at);
    case GoalTerminationKind::kShutdown:
      return slot.request_shutdown(id, generation, arrived_at);
    case GoalTerminationKind::kAuthorityLoss:
      return slot.request_authority_loss(id, generation, arrived_at);
    case GoalTerminationKind::kInboxOverflow:
      return slot.request_inbox_loss(id, generation, arrived_at);
    case GoalTerminationKind::kProtocolFailure:
      return slot.request_protocol_failure(id, generation, arrived_at);
    case GoalTerminationKind::kTaskDeadline:
      return slot.request_task_deadline(id, generation, generation, arrived_at);
    case GoalTerminationKind::kMotionDeadline:
      return slot.request_motion_deadline(id, generation, generation, arrived_at);
  }
  return slot.request_cancel(CoordinatorGoalId{}, 0U, SteadyTime::max());
}

struct ExpectedTerminationPolicy
{
  bool cancel_requested{false};
  bool safe_abort_requested{false};
  bool task_deadline_exceeded{false};
  GoalTerminationIntent intent{GoalTerminationIntent::kNone};
};

void apply_expected_policy(ExpectedTerminationPolicy & policy, GoalTerminationKind kind)
{
  const auto strengthen = [&policy](GoalTerminationIntent incoming) {
    const auto priority = [](GoalTerminationIntent intent) {
      switch (intent) {
        case GoalTerminationIntent::kNone: return 0;
        case GoalTerminationIntent::kUserCancel: return 1;
        case GoalTerminationIntent::kShutdownDrain: return 2;
        case GoalTerminationIntent::kSafeAbort: return 3;
        case GoalTerminationIntent::kTaskDeadline: return 4;
      }
      return 0;
    };
    if (priority(incoming) > priority(policy.intent)) {
      policy.intent = incoming;
    }
  };
  switch (kind) {
    case GoalTerminationKind::kUserCancel:
      policy.cancel_requested = true;
      strengthen(GoalTerminationIntent::kUserCancel);
      return;
    case GoalTerminationKind::kCoordinatorDrain:
    case GoalTerminationKind::kShutdown:
      if (policy.intent != GoalTerminationIntent::kUserCancel) {
        policy.safe_abort_requested = true;
        strengthen(GoalTerminationIntent::kShutdownDrain);
      }
      return;
    case GoalTerminationKind::kSafeAbort:
    case GoalTerminationKind::kAuthorityLoss:
    case GoalTerminationKind::kInboxOverflow:
    case GoalTerminationKind::kProtocolFailure:
      policy.safe_abort_requested = true;
      strengthen(GoalTerminationIntent::kSafeAbort);
      return;
    case GoalTerminationKind::kTaskDeadline:
      policy.task_deadline_exceeded = true;
      strengthen(GoalTerminationIntent::kTaskDeadline);
      return;
    case GoalTerminationKind::kMotionDeadline:
      return;
  }
}

constexpr std::array kPublicTerminationKinds{
  GoalTerminationKind::kUserCancel,
  GoalTerminationKind::kCoordinatorDrain,
  GoalTerminationKind::kSafeAbort,
  GoalTerminationKind::kShutdown,
  GoalTerminationKind::kAuthorityLoss,
  GoalTerminationKind::kInboxOverflow,
  GoalTerminationKind::kProtocolFailure,
  GoalTerminationKind::kTaskDeadline,
};

TEST(GoalAdmissionSlot, RejectsInvalidNotReadyAndMismatchedTransitions)
{
  GoalAdmissionSlot slot;
  EXPECT_EQ(slot.reserve(CoordinatorGoalId{}).decision, GoalAdmissionDecision::kInvalidGoalId);
  EXPECT_EQ(slot.reserve(goal_id(1)).decision, GoalAdmissionDecision::kNotReady);
  slot.update_readiness(true, "ready");
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(receipt.generation, 1U);
  EXPECT_EQ(slot.reserve(goal_id(2)).decision, GoalAdmissionDecision::kBusy);
  EXPECT_FALSE(slot.activate(goal_id(2), receipt.generation));
  EXPECT_FALSE(slot.activate(goal_id(1), receipt.generation + 1));
  EXPECT_TRUE(slot.activate(goal_id(1), receipt.generation));
  EXPECT_EQ(
    slot.request_cancel(goal_id(2), receipt.generation, SteadyTime{} + 1ms).status(),
    GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(
    slot.request_cancel(goal_id(1), receipt.generation, SteadyTime{} + 2ms).status(),
    GoalTerminationLatchStatus::kLatched);
  EXPECT_TRUE(slot.snapshot().cancel_requested);
  EXPECT_FALSE(slot.finish(goal_id(1), receipt.generation + 1));
  EXPECT_TRUE(slot.finish(goal_id(1), receipt.generation));
  EXPECT_EQ(slot.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST(GoalAdmissionSlot, ActivationAuthorityObservationIsNarrowExactAndNoexcept)
{
  static_assert(std::is_nothrow_copy_constructible_v<GoalActivationAuthority>);
  static_assert(std::is_nothrow_move_constructible_v<GoalActivationAuthorityDecision>);
  static_assert(
    noexcept(
      std::declval<const GoalAdmissionSlot &>().observe_activation_authority(
        std::declval<const CoordinatorGoalId &>(), std::declval<GoalGeneration>())));

  GoalAdmissionSlot slot;
  const auto id = goal_id(1U);
  const auto other_id = goal_id(2U);
  EXPECT_EQ(
    slot.observe_activation_authority(CoordinatorGoalId{}, 1U).status,
    GoalActivationAuthorityStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.observe_activation_authority(id, 0U).status,
    GoalActivationAuthorityStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.observe_activation_authority(id, kReservedCoordinatorGoalGeneration).status,
    GoalActivationAuthorityStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.observe_activation_authority(id, 1U).status,
    GoalActivationAuthorityStatus::kInactive);

  slot.update_readiness(true, "ready");
  const auto receipt = slot.reserve(id);
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(
    slot.observe_activation_authority(other_id, receipt.generation).status,
    GoalActivationAuthorityStatus::kGoalMismatch);
  EXPECT_EQ(
    slot.observe_activation_authority(id, receipt.generation + 1U).status,
    GoalActivationAuthorityStatus::kGoalMismatch);

  const auto clean = slot.observe_activation_authority(id, receipt.generation);
  ASSERT_EQ(clean.status, GoalActivationAuthorityStatus::kObserved);
  ASSERT_TRUE(clean.authority);
  EXPECT_EQ(clean.authority->phase, GoalSlotPhase::kPendingAcceptance);
  EXPECT_EQ(clean.authority->goal_id, id);
  EXPECT_EQ(clean.authority->generation, receipt.generation);
  EXPECT_FALSE(clean.authority->inhibited);
  EXPECT_FALSE(clean.authority->cancel_requested);
  EXPECT_FALSE(clean.authority->safe_abort_requested);
  EXPECT_FALSE(clean.authority->task_deadline_exceeded);
  EXPECT_FALSE(clean.authority->mutation_submission);
  EXPECT_EQ(clean.authority->termination_intent, GoalTerminationIntent::kNone);
  EXPECT_FALSE(clean.authority->first_termination);

  const auto arrived_at = SteadyTime{} + 7ms;
  const auto termination = slot.request_drain(id, receipt.generation, arrived_at);
  ASSERT_EQ(termination.status(), GoalTerminationLatchStatus::kLatched);
  const auto terminated = slot.observe_activation_authority(id, receipt.generation);
  ASSERT_EQ(terminated.status, GoalActivationAuthorityStatus::kObserved);
  ASSERT_TRUE(terminated.authority);
  EXPECT_TRUE(terminated.authority->safe_abort_requested);
  EXPECT_EQ(
    terminated.authority->termination_intent, GoalTerminationIntent::kShutdownDrain);
  ASSERT_TRUE(terminated.authority->first_termination);
  EXPECT_EQ(
    terminated.authority->first_termination.get(), termination.record().get());

  slot.inhibit("test inhibition");
  ASSERT_EQ(
    slot.request_safe_abort(id, receipt.generation, arrived_at + 1ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  ASSERT_EQ(
    slot.request_task_deadline(
      id, receipt.generation, receipt.generation, arrived_at + 2ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  ASSERT_EQ(
    slot.request_cancel(id, receipt.generation, arrived_at + 3ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  const auto strengthened = slot.observe_activation_authority(id, receipt.generation);
  ASSERT_EQ(strengthened.status, GoalActivationAuthorityStatus::kObserved);
  ASSERT_TRUE(strengthened.authority);
  EXPECT_TRUE(strengthened.authority->inhibited);
  EXPECT_TRUE(strengthened.authority->cancel_requested);
  EXPECT_TRUE(strengthened.authority->safe_abort_requested);
  EXPECT_TRUE(strengthened.authority->task_deadline_exceeded);
  EXPECT_EQ(
    strengthened.authority->termination_intent, GoalTerminationIntent::kTaskDeadline);
  ASSERT_TRUE(strengthened.authority->first_termination);
  EXPECT_EQ(
    strengthened.authority->first_termination.get(), termination.record().get());

  ASSERT_TRUE(slot.activate(id, receipt.generation));
  EXPECT_EQ(
    slot.observe_activation_authority(id, receipt.generation).status,
    GoalActivationAuthorityStatus::kWrongPhase);
}

TEST(GoalAdmissionSlot, EveryPairwiseRequestOrderPreservesWinnerAndStrengthensPolicy)
{
  std::uint8_t seed = 1U;
  for (const auto first_kind : kPublicTerminationKinds) {
    for (const auto second_kind : kPublicTerminationKinds) {
      SCOPED_TRACE(
        std::to_string(static_cast<unsigned int>(first_kind)) + " then " +
        std::to_string(static_cast<unsigned int>(second_kind)));
      GoalAdmissionSlot slot;
      slot.update_readiness(true);
      const auto id = goal_id(seed++);
      const auto receipt = slot.reserve(id);
      ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);

      const auto first = request_termination_kind(
        slot, first_kind, id, receipt.generation, SteadyTime{} + 1ms);
      const auto second = request_termination_kind(
        slot, second_kind, id, receipt.generation, SteadyTime{} + 2ms);
      ASSERT_EQ(first.status(), GoalTerminationLatchStatus::kLatched);
      ASSERT_EQ(second.status(), GoalTerminationLatchStatus::kAlreadyLatched);
      ASSERT_TRUE(first.record());
      EXPECT_EQ(second.record().get(), first.record().get());
      EXPECT_EQ(first.record()->kind, first_kind);
      EXPECT_EQ(first.record()->arrived_at, SteadyTime{} + 1ms);

      ExpectedTerminationPolicy expected;
      apply_expected_policy(expected, first_kind);
      apply_expected_policy(expected, second_kind);
      const auto snapshot = slot.snapshot();
      EXPECT_EQ(snapshot.cancel_requested, expected.cancel_requested);
      EXPECT_EQ(snapshot.safe_abort_requested, expected.safe_abort_requested);
      EXPECT_EQ(snapshot.task_deadline_exceeded, expected.task_deadline_exceeded);
      EXPECT_EQ(snapshot.termination_intent, expected.intent);
    }
  }
}

TEST(GoalAdmissionSlot, ConcurrentRequestsPublishOneCompleteWinnerRecord)
{
  constexpr std::size_t kIterations = 128U;
  for (std::size_t iteration = 0U; iteration < kIterations; ++iteration) {
    GoalAdmissionSlot slot;
    slot.update_readiness(true);
    const auto receipt = slot.reserve(goal_id(1));
    ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
    BoundedRaceGate start_gate;
    GoalTerminationLatchStatus cancel_status = GoalTerminationLatchStatus::kInvalidArgument;
    GoalTerminationLatchStatus abort_status = GoalTerminationLatchStatus::kInvalidArgument;
    std::shared_ptr<const FirstGoalTerminationRecord> cancel_record;
    std::shared_ptr<const FirstGoalTerminationRecord> abort_record;
    std::jthread canceler(
      [&]() {
        start_gate.arrive_and_wait();
        auto decision = slot.request_cancel(
          goal_id(1), receipt.generation, SteadyTime{} + 1ms);
        cancel_status = decision.status();
        cancel_record = decision.record();
      });
    std::jthread aborter(
      [&]() {
        start_gate.arrive_and_wait();
        auto decision = slot.request_safe_abort(
          goal_id(1), receipt.generation, SteadyTime{} + 2ms);
        abort_status = decision.status();
        abort_record = decision.record();
      });

    const bool both_ready = start_gate.wait_for_arrivals(2U, 1s);
    start_gate.release();
    canceler.join();
    aborter.join();
    ASSERT_TRUE(both_ready);
    EXPECT_TRUE(
      (cancel_status == GoalTerminationLatchStatus::kLatched &&
      abort_status == GoalTerminationLatchStatus::kAlreadyLatched) ||
      (abort_status == GoalTerminationLatchStatus::kLatched &&
      cancel_status == GoalTerminationLatchStatus::kAlreadyLatched));
    ASSERT_TRUE(cancel_record);
    ASSERT_TRUE(abort_record);
    EXPECT_EQ(cancel_record.get(), abort_record.get());
    EXPECT_EQ(
      validate_goal_termination_record(*cancel_record), GoalTerminationValidationError::kNone);
    EXPECT_TRUE(
      (cancel_record->kind == GoalTerminationKind::kUserCancel &&
      cancel_record->arrived_at == SteadyTime{} + 1ms) ||
      (cancel_record->kind == GoalTerminationKind::kSafeAbort &&
      cancel_record->arrived_at == SteadyTime{} + 2ms));
    const auto snapshot = slot.snapshot();
    EXPECT_TRUE(snapshot.cancel_requested);
    EXPECT_TRUE(snapshot.safe_abort_requested);
    EXPECT_EQ(snapshot.termination_intent, GoalTerminationIntent::kSafeAbort);
  }
}

TEST(GoalAdmissionSlot, ConcurrentSnapshotsObserveNoRecordOrTheCompletePublishedRecord)
{
  constexpr std::size_t kIterations = 128U;
  for (std::size_t iteration = 0U; iteration < kIterations; ++iteration) {
    GoalAdmissionSlot slot;
    slot.update_readiness(true);
    const auto id = goal_id(1);
    const auto receipt = slot.reserve(id);
    ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);

    BoundedRaceGate start_gate;
    std::atomic<bool> request_complete{false};
    std::atomic<bool> observer_valid{true};
    const auto arrived_at = SteadyTime{} + 17ms;
    std::jthread requester(
      [&]() {
        start_gate.arrive_and_wait();
        const auto decision = slot.request_protocol_failure(
          id, receipt.generation, arrived_at);
        if (decision.status() != GoalTerminationLatchStatus::kLatched || !decision.record()) {
          observer_valid.store(false, std::memory_order_relaxed);
        }
        request_complete.store(true, std::memory_order_release);
      });
    std::jthread observer(
      [&]() {
        start_gate.arrive_and_wait();
        do {
          const auto decision = slot.snapshot_first_termination(id, receipt.generation);
          if (decision.status() == GoalTerminationSnapshotStatus::kPresent) {
            const auto record = decision.record();
            if (!record ||
            validate_goal_termination_record(*record) !=
            GoalTerminationValidationError::kNone ||
            record->goal_id != id || record->goal_generation != receipt.generation ||
            record->kind != GoalTerminationKind::kProtocolFailure ||
            record->arrived_at != arrived_at)
            {
              observer_valid.store(false, std::memory_order_relaxed);
            }
          }
          if (decision.status() != GoalTerminationSnapshotStatus::kPresent &&
          (decision.status() != GoalTerminationSnapshotStatus::kNone || decision.record()))
          {
            observer_valid.store(false, std::memory_order_relaxed);
          }
        } while (!request_complete.load(std::memory_order_acquire));
      });

    const bool both_ready = start_gate.wait_for_arrivals(2U, 1s);
    start_gate.release();
    requester.join();
    observer.join();
    ASSERT_TRUE(both_ready);
    EXPECT_TRUE(observer_valid.load(std::memory_order_relaxed));
    const auto final = slot.snapshot_first_termination(id, receipt.generation);
    ASSERT_EQ(final.status(), GoalTerminationSnapshotStatus::kPresent);
    ASSERT_TRUE(final.record());
    EXPECT_EQ(final.record()->kind, GoalTerminationKind::kProtocolFailure);
    EXPECT_EQ(final.record()->arrived_at, arrived_at);
  }
}

TEST(GoalAdmissionSlot, ValidatesTerminationArgumentsBeforeAdmissionState)
{
  GoalAdmissionSlot slot;
  EXPECT_EQ(
    slot.request_cancel(CoordinatorGoalId{}, 0U, SteadyTime::max()).status(),
    GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.request_cancel(goal_id(1), 1U, SteadyTime{}).status(),
    GoalTerminationLatchStatus::kInactive);
  EXPECT_EQ(
    slot.snapshot_first_termination(CoordinatorGoalId{}, 0U).status(),
    GoalTerminationSnapshotStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.snapshot_first_termination(goal_id(1), 1U).status(),
    GoalTerminationSnapshotStatus::kInactive);

  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(
    slot.request_safe_abort(
      goal_id(1), receipt.generation, SteadyTime::max()).status(),
    GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.request_task_deadline(
      goal_id(1), receipt.generation, 0U, SteadyTime{}).status(),
    GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_EQ(
    slot.request_task_deadline(
      goal_id(1), receipt.generation, receipt.generation + 1U, SteadyTime{}).status(),
    GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_EQ(slot.snapshot().termination_intent, GoalTerminationIntent::kNone);
}

TEST(GoalAdmissionSlot, PreservesFirstRecordAcrossPolicyStrengtheningActivationAndFinish)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(
    slot.snapshot_first_termination(goal_id(1), receipt.generation).status(),
    GoalTerminationSnapshotStatus::kNone);

  const auto first = slot.request_cancel(
    goal_id(1), receipt.generation, SteadyTime{} + 1ms);
  ASSERT_EQ(first.status(), GoalTerminationLatchStatus::kLatched);
  ASSERT_TRUE(first.record());
  EXPECT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto strengthened = slot.request_authority_loss(
    goal_id(1), receipt.generation, SteadyTime{} + 2ms);
  ASSERT_EQ(strengthened.status(), GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(strengthened.record().get(), first.record().get());
  EXPECT_EQ(strengthened.record()->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(slot.snapshot().termination_intent, GoalTerminationIntent::kSafeAbort);

  const auto mismatch = slot.request_drain(
    goal_id(2), receipt.generation, SteadyTime{} + 3ms);
  ASSERT_EQ(mismatch.status(), GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(mismatch.record().get(), first.record().get());
  const auto snapshot_mismatch =
    slot.snapshot_first_termination(goal_id(2), receipt.generation);
  ASSERT_EQ(snapshot_mismatch.status(), GoalTerminationSnapshotStatus::kGoalMismatch);
  EXPECT_EQ(snapshot_mismatch.record().get(), first.record().get());
  const auto present = slot.snapshot_first_termination(goal_id(1), receipt.generation);
  ASSERT_EQ(present.status(), GoalTerminationSnapshotStatus::kPresent);
  EXPECT_EQ(present.record().get(), first.record().get());

  ASSERT_TRUE(slot.finish(goal_id(1), receipt.generation));
  EXPECT_EQ(first.record()->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(
    slot.snapshot_first_termination(goal_id(1), receipt.generation).status(),
    GoalTerminationSnapshotStatus::kInactive);

  const auto next = slot.reserve(goal_id(2));
  ASSERT_EQ(next.decision, GoalAdmissionDecision::kAccepted);
  const auto next_first = slot.request_drain(
    goal_id(2), next.generation, SteadyTime{} + 4ms);
  ASSERT_EQ(next_first.status(), GoalTerminationLatchStatus::kLatched);
  ASSERT_TRUE(next_first.record());
  EXPECT_NE(next_first.record().get(), first.record().get());
}

TEST(GoalAdmissionSlot, RejectsSameIdStaleGenerationWithoutMutatingTheNewGoal)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto id = goal_id(1);
  const auto first_receipt = slot.reserve(id);
  ASSERT_EQ(first_receipt.decision, GoalAdmissionDecision::kAccepted);
  const auto historical = slot.request_cancel(
    id, first_receipt.generation, SteadyTime{} + 1ms);
  ASSERT_EQ(historical.status(), GoalTerminationLatchStatus::kLatched);
  ASSERT_TRUE(slot.finish(id, first_receipt.generation));

  const auto current_receipt = slot.reserve(id);
  ASSERT_EQ(current_receipt.decision, GoalAdmissionDecision::kAccepted);
  ASSERT_NE(current_receipt.generation, first_receipt.generation);
  const auto stale_before_publication = slot.request_safe_abort(
    id, first_receipt.generation, SteadyTime{} + 2ms);
  EXPECT_EQ(stale_before_publication.status(), GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_FALSE(stale_before_publication.record());
  auto current = slot.snapshot();
  EXPECT_EQ(current.termination_intent, GoalTerminationIntent::kNone);
  EXPECT_FALSE(current.first_termination);

  const auto current_first = slot.request_drain(
    id, current_receipt.generation, SteadyTime{} + 3ms);
  ASSERT_EQ(current_first.status(), GoalTerminationLatchStatus::kLatched);
  const auto stale_after_publication = slot.request_task_deadline(
    id, first_receipt.generation, first_receipt.generation, SteadyTime{} + 4ms);
  EXPECT_EQ(stale_after_publication.status(), GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(stale_after_publication.record().get(), current_first.record().get());
  current = slot.snapshot();
  EXPECT_EQ(current.termination_intent, GoalTerminationIntent::kShutdownDrain);
  EXPECT_FALSE(current.task_deadline_exceeded);
  EXPECT_EQ(current.first_termination.get(), current_first.record().get());
  EXPECT_NE(current.first_termination.get(), historical.record().get());
}

TEST(GoalAdmissionSlot, DecisionMovesPreserveEveryPublicShapeInvariant)
{
  GoalAdmissionSlot inactive_slot;
  const auto invalid_latch = inactive_slot.request_cancel(
    CoordinatorGoalId{}, 0U, SteadyTime::max());
  const auto inactive_latch = inactive_slot.request_cancel(
    goal_id(1), 1U, SteadyTime{});
  const auto invalid_snapshot = inactive_slot.snapshot_first_termination(
    CoordinatorGoalId{}, 0U);
  const auto inactive_snapshot = inactive_slot.snapshot_first_termination(
    goal_id(1), 1U);

  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  const auto mismatch_empty_latch = slot.request_cancel(
    goal_id(2), receipt.generation, SteadyTime{});
  const auto none_snapshot = slot.snapshot_first_termination(goal_id(1), receipt.generation);
  const auto mismatch_empty_snapshot =
    slot.snapshot_first_termination(goal_id(2), receipt.generation);
  const auto latched = slot.request_cancel(
    goal_id(1), receipt.generation, SteadyTime{} + 1ms);
  const auto already_latched = slot.request_safe_abort(
    goal_id(1), receipt.generation, SteadyTime{} + 2ms);
  const auto mismatch_record_latch = slot.request_drain(
    goal_id(2), receipt.generation, SteadyTime{} + 3ms);
  const auto present_snapshot = slot.snapshot_first_termination(goal_id(1), receipt.generation);
  const auto mismatch_record_snapshot =
    slot.snapshot_first_termination(goal_id(2), receipt.generation);

  struct LatchCase
  {
    GoalTerminationLatchStatus status;
    GoalTerminationLatchDecision decision;
    bool has_record;
  };
  const std::vector<LatchCase> latch_cases{
    {GoalTerminationLatchStatus::kInvalidArgument, invalid_latch, false},
    {GoalTerminationLatchStatus::kInactive, inactive_latch, false},
    {GoalTerminationLatchStatus::kGoalMismatch, mismatch_empty_latch, false},
    {GoalTerminationLatchStatus::kLatched, latched, true},
    {GoalTerminationLatchStatus::kAlreadyLatched, already_latched, true},
    {GoalTerminationLatchStatus::kGoalMismatch, mismatch_record_latch, true},
  };
  for (const auto & test_case : latch_cases) {
    SCOPED_TRACE(static_cast<unsigned int>(test_case.status));
    auto source = test_case.decision;
    const auto exact_record = source.record();
    auto moved = std::move(source);
    EXPECT_EQ(moved.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(moved.record()), test_case.has_record);
    EXPECT_EQ(moved.record().get(), exact_record.get());
    EXPECT_EQ(source.status(), GoalTerminationLatchStatus::kInvalidArgument);
    EXPECT_FALSE(source.record());

    auto assigned = invalid_latch;
    assigned = std::move(moved);
    EXPECT_EQ(assigned.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(assigned.record()), test_case.has_record);
    EXPECT_EQ(assigned.record().get(), exact_record.get());
    EXPECT_EQ(moved.status(), GoalTerminationLatchStatus::kInvalidArgument);
    EXPECT_FALSE(moved.record());
    auto * alias = &assigned;
    assigned = std::move(*alias);
    EXPECT_EQ(assigned.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(assigned.record()), test_case.has_record);
    EXPECT_EQ(assigned.record().get(), exact_record.get());
  }

  struct SnapshotCase
  {
    GoalTerminationSnapshotStatus status;
    GoalTerminationSnapshotDecision decision;
    bool has_record;
  };
  const std::vector<SnapshotCase> snapshot_cases{
    {GoalTerminationSnapshotStatus::kInvalidArgument, invalid_snapshot, false},
    {GoalTerminationSnapshotStatus::kInactive, inactive_snapshot, false},
    {GoalTerminationSnapshotStatus::kNone, none_snapshot, false},
    {GoalTerminationSnapshotStatus::kGoalMismatch, mismatch_empty_snapshot, false},
    {GoalTerminationSnapshotStatus::kPresent, present_snapshot, true},
    {GoalTerminationSnapshotStatus::kGoalMismatch, mismatch_record_snapshot, true},
  };
  for (const auto & test_case : snapshot_cases) {
    SCOPED_TRACE(static_cast<unsigned int>(test_case.status));
    auto source = test_case.decision;
    const auto exact_record = source.record();
    auto moved = std::move(source);
    EXPECT_EQ(moved.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(moved.record()), test_case.has_record);
    EXPECT_EQ(moved.record().get(), exact_record.get());
    EXPECT_EQ(source.status(), GoalTerminationSnapshotStatus::kInvalidArgument);
    EXPECT_FALSE(source.record());

    auto assigned = invalid_snapshot;
    assigned = std::move(moved);
    EXPECT_EQ(assigned.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(assigned.record()), test_case.has_record);
    EXPECT_EQ(assigned.record().get(), exact_record.get());
    EXPECT_EQ(moved.status(), GoalTerminationSnapshotStatus::kInvalidArgument);
    EXPECT_FALSE(moved.record());
    auto * alias = &assigned;
    assigned = std::move(*alias);
    EXPECT_EQ(assigned.status(), test_case.status);
    EXPECT_EQ(static_cast<bool>(assigned.record()), test_case.has_record);
    EXPECT_EQ(assigned.record().get(), exact_record.get());
  }
}

TEST(GoalAdmissionSlot, ConcurrentGoalCallbacksReserveExactlyOnePendingSlot)
{
  constexpr std::size_t kThreadCount = 32;
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  std::barrier start_line(static_cast<std::ptrdiff_t>(kThreadCount));
  std::array<GoalAdmissionReceipt, kThreadCount> receipts{};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back(
      [&, index]() {
        start_line.arrive_and_wait();
        receipts[index] = slot.reserve(goal_id(static_cast<std::uint8_t>(index + 1)));
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  std::size_t accepted = 0;
  std::size_t busy = 0;
  for (const auto & receipt : receipts) {
    accepted += receipt.decision == GoalAdmissionDecision::kAccepted ? 1U : 0U;
    busy += receipt.decision == GoalAdmissionDecision::kBusy ? 1U : 0U;
  }
  EXPECT_EQ(accepted, 1U);
  EXPECT_EQ(busy, kThreadCount - 1U);
  EXPECT_EQ(slot.snapshot().phase, GoalSlotPhase::kPendingAcceptance);
}

TEST(GoalAdmissionSlot, AllocationFailureLeavesAdmissionStateUnchanged)
{
  std::atomic<std::size_t> calls{0U};
  GoalAdmissionSlot slot(
    [&calls]() {
      return calls.fetch_add(1U, std::memory_order_relaxed) == 0U;
    });
  slot.update_readiness(true);

  EXPECT_EQ(slot.reserve(goal_id(1)).decision, GoalAdmissionDecision::kResourceExhausted);
  const auto accepted = slot.reserve(goal_id(1));
  ASSERT_EQ(accepted.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(accepted.generation, 1U);
  EXPECT_EQ(calls.load(std::memory_order_relaxed), 2U);
  EXPECT_TRUE(slot.activate(goal_id(1), accepted.generation));
  EXPECT_TRUE(slot.finish(goal_id(1), accepted.generation));
  EXPECT_EQ(slot.snapshot().phase, GoalSlotPhase::kIdle);

  GoalAdmissionSlot throwing([]() -> bool {throw std::bad_alloc{};});
  throwing.update_readiness(true);
  EXPECT_EQ(
    throwing.reserve(goal_id(2)).decision, GoalAdmissionDecision::kResourceExhausted);
  EXPECT_EQ(throwing.snapshot().phase, GoalSlotPhase::kIdle);

  GoalAdmissionSlot non_standard_throwing([]() -> bool {throw 7;});
  non_standard_throwing.update_readiness(true);
  EXPECT_EQ(
    non_standard_throwing.reserve(goal_id(3)).decision,
    GoalAdmissionDecision::kResourceExhausted);
  EXPECT_EQ(non_standard_throwing.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST(GoalAdmissionSlot, SkipsAllocationInjectorForFirstPassRejections)
{
  std::atomic<std::size_t> calls{0U};
  GoalAdmissionSlot slot(
    [&calls]() {
      calls.fetch_add(1U, std::memory_order_relaxed);
      return false;
    });

  EXPECT_EQ(slot.reserve(CoordinatorGoalId{}).decision, GoalAdmissionDecision::kInvalidGoalId);
  EXPECT_EQ(slot.reserve(goal_id(1)).decision, GoalAdmissionDecision::kNotReady);
  EXPECT_EQ(calls.load(std::memory_order_relaxed), 0U);

  slot.update_readiness(true);
  const auto accepted = slot.reserve(goal_id(1));
  ASSERT_EQ(accepted.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(slot.reserve(goal_id(2)).decision, GoalAdmissionDecision::kBusy);
  EXPECT_EQ(calls.load(std::memory_order_relaxed), 1U);

  ASSERT_TRUE(slot.finish(goal_id(1), accepted.generation));
  slot.inhibit("test inhibition");
  EXPECT_EQ(slot.reserve(goal_id(2)).decision, GoalAdmissionDecision::kInhibited);
  EXPECT_EQ(calls.load(std::memory_order_relaxed), 1U);
}

TEST(GoalAdmissionSlot, SecondAuthoritativeCheckPrecedesAllocationOutcome)
{
  BoundedRaceGate allocation_gate;
  GoalAdmissionSlot slot(
    [&]() {
      allocation_gate.arrive_and_wait();
      return true;
    });
  slot.update_readiness(true);

  GoalAdmissionReceipt receipt;
  std::jthread contender([&]() {receipt = slot.reserve(goal_id(1));});
  const bool reached_allocation = allocation_gate.wait_for_arrival(1s);
  if (reached_allocation) {
    slot.update_readiness(false);
  }
  allocation_gate.release();
  contender.join();

  ASSERT_TRUE(reached_allocation);
  EXPECT_EQ(receipt.decision, GoalAdmissionDecision::kNotReady);
  EXPECT_EQ(slot.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST(GoalAdmissionSlot, ConcurrentPrivateAllocationsPublishExactlyOneGoal)
{
  std::atomic<std::size_t> calls{0U};
  BoundedRaceGate first_allocation_gate;
  GoalAdmissionSlot slot(
    [&]() {
      if (calls.fetch_add(1U, std::memory_order_relaxed) == 0U) {
        first_allocation_gate.arrive_and_wait();
      }
      return false;
    });
  slot.update_readiness(true);

  GoalAdmissionReceipt first;
  std::jthread first_contender([&]() {first = slot.reserve(goal_id(1));});
  const bool reached_allocation = first_allocation_gate.wait_for_arrival(1s);
  const auto second = reached_allocation ? slot.reserve(goal_id(2)) : GoalAdmissionReceipt{};
  first_allocation_gate.release();
  first_contender.join();

  ASSERT_TRUE(reached_allocation);
  EXPECT_EQ(second.decision, GoalAdmissionDecision::kAccepted);
  EXPECT_EQ(first.decision, GoalAdmissionDecision::kBusy);
  const auto snapshot = slot.snapshot();
  ASSERT_TRUE(snapshot.goal_id);
  EXPECT_EQ(*snapshot.goal_id, goal_id(2));
  EXPECT_EQ(snapshot.generation, second.generation);
}

TEST(GoalAdmissionSlot, InhibitionPersistsAfterActiveGoalFinishes)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  slot.inhibit("authority disagreement");
  EXPECT_TRUE(slot.snapshot().inhibited);
  EXPECT_TRUE(slot.finish(goal_id(1), receipt.generation));
  EXPECT_EQ(slot.reserve(goal_id(2)).decision, GoalAdmissionDecision::kInhibited);
  EXPECT_EQ(slot.snapshot().inhibition_detail, "authority disagreement");
}

// A finished transfer must release its reservation, or the world state keeps it active and
// every later task is refused.
TEST(GoalAdmissionSlot, AdmitsACleanupReleaseAtACompletionBoundary)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto deadline = SteadyTime{} + 100ms;

  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReleaseTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCleanupWithoutTermination);

  // Only the active goal may claim the boundary.
  EXPECT_FALSE(slot.begin_completion_cleanup(goal_id(2), receipt.generation));
  ASSERT_TRUE(slot.begin_completion_cleanup(goal_id(1), receipt.generation));
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReleaseTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCommitted);
}

TEST(GoalAdmissionSlot, OwnsExactMutationSubmissionLifecycle)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto deadline = SteadyTime{} + 100ms;

  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 0, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kInvalidArgument);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      deadline, deadline).decision,
    MutationSubmissionDecision::kDeadlineExceeded);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReleaseTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCleanupWithoutTermination);

  const auto committed = slot.commit_mutation_submission(
    goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
    SteadyTime{}, deadline);
  ASSERT_EQ(committed.decision, MutationSubmissionDecision::kCommitted);
  ASSERT_TRUE(committed.record);
  EXPECT_EQ(committed.record->phase, MutationSubmissionPhase::kCommitted);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kReplayCommitted);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 2, CoordinatorMutationKind::kCheckpointTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kAlreadyPending);
  EXPECT_FALSE(slot.finish(goal_id(1), receipt.generation));

  EXPECT_FALSE(
    slot.confirm_mutation_not_submitted(
      goal_id(1), receipt.generation, 2, CoordinatorMutationKind::kReserveTask));
  ASSERT_TRUE(
    slot.confirm_mutation_not_submitted(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
  ASSERT_TRUE(slot.snapshot().mutation_submission);
  EXPECT_EQ(
    slot.snapshot().mutation_submission->phase,
    MutationSubmissionPhase::kConfirmedNotSubmitted);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCommitted);
  ASSERT_TRUE(
    slot.resolve_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
  EXPECT_FALSE(slot.snapshot().mutation_submission);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kStaleOperation);
  EXPECT_TRUE(slot.finish(goal_id(1), receipt.generation));
}

TEST(GoalAdmissionSlot, TerminationBlocksForwardMutationButPermitsTypedRelease)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto deadline = SteadyTime{} + 100ms;

  ASSERT_EQ(
    slot.request_cancel(goal_id(1), receipt.generation, SteadyTime{} + 1ms).status(),
    GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kTerminationRequested);
  EXPECT_FALSE(slot.snapshot().mutation_submission);
  const auto cleanup = slot.commit_mutation_submission(
    goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReleaseTask,
    SteadyTime{}, deadline);
  ASSERT_EQ(cleanup.decision, MutationSubmissionDecision::kCommitted);
  EXPECT_EQ(slot.snapshot().termination_intent, GoalTerminationIntent::kUserCancel);
  ASSERT_TRUE(
    slot.resolve_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReleaseTask));

  ASSERT_EQ(
    slot.request_safe_abort(goal_id(1), receipt.generation, SteadyTime{} + 2ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(slot.snapshot().termination_intent, GoalTerminationIntent::kSafeAbort);
  ASSERT_EQ(
    slot.request_task_deadline(
      goal_id(1), receipt.generation, receipt.generation, SteadyTime{} + 3ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  const auto snapshot = slot.snapshot();
  EXPECT_TRUE(snapshot.cancel_requested);
  EXPECT_TRUE(snapshot.safe_abort_requested);
  EXPECT_TRUE(snapshot.task_deadline_exceeded);
  EXPECT_EQ(snapshot.termination_intent, GoalTerminationIntent::kTaskDeadline);
}

TEST(GoalAdmissionSlot, DrainLinearizesAgainstForwardMutationAndPreservesEarlierCancel)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto deadline = SteadyTime{} + 100ms;

  EXPECT_EQ(
    slot.request_drain(goal_id(2), receipt.generation, SteadyTime{} + 1ms).status(),
    GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(
    slot.request_drain(goal_id(1), receipt.generation, SteadyTime{} + 2ms).status(),
    GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(
    slot.request_drain(goal_id(1), receipt.generation, SteadyTime{} + 3ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(slot.snapshot().termination_intent, GoalTerminationIntent::kShutdownDrain);
  EXPECT_TRUE(slot.snapshot().safe_abort_requested);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kTerminationRequested);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 2, CoordinatorMutationKind::kReleaseTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCommitted);

  GoalAdmissionSlot canceled;
  canceled.update_readiness(true);
  const auto canceled_receipt = canceled.reserve(goal_id(3));
  ASSERT_TRUE(canceled.activate(goal_id(3), canceled_receipt.generation));
  const auto cancel = canceled.request_cancel(
    goal_id(3), canceled_receipt.generation, SteadyTime{} + 4ms);
  ASSERT_EQ(cancel.status(), GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(
    canceled.request_drain(
      goal_id(3), canceled_receipt.generation, SteadyTime{} + 5ms).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  ASSERT_TRUE(cancel.record());
  EXPECT_EQ(cancel.record()->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(canceled.snapshot().termination_intent, GoalTerminationIntent::kUserCancel);
  EXPECT_FALSE(canceled.snapshot().safe_abort_requested);
}

TEST(GoalAdmissionSlot, ConfirmedNoSubmissionCannotRestartAfterCancellation)
{
  GoalAdmissionSlot slot;
  slot.update_readiness(true);
  const auto receipt = slot.reserve(goal_id(1));
  ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
  const auto deadline = SteadyTime{} + 100ms;
  ASSERT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kCommitted);
  ASSERT_TRUE(
    slot.confirm_mutation_not_submitted(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
  ASSERT_EQ(
    slot.request_cancel(goal_id(1), receipt.generation, SteadyTime{} + 1ms).status(),
    GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(
    slot.commit_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, deadline).decision,
    MutationSubmissionDecision::kTerminationRequested);
  ASSERT_TRUE(slot.snapshot().mutation_submission);
  EXPECT_EQ(
    slot.snapshot().mutation_submission->phase,
    MutationSubmissionPhase::kConfirmedNotSubmitted);
  EXPECT_TRUE(
    slot.resolve_mutation_submission(
      goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
}

TEST(GoalAdmissionSlot, PersistentInhibitionBlocksSubmissionCleanupAndReplay)
{
  GoalAdmissionSlot forward;
  forward.update_readiness(true);
  const auto forward_receipt = forward.reserve(goal_id(1));
  ASSERT_TRUE(forward.activate(goal_id(1), forward_receipt.generation));
  forward.inhibit("authority disagreement");
  EXPECT_EQ(
    forward.commit_mutation_submission(
      goal_id(1), forward_receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, SteadyTime{} + 100ms).decision,
    MutationSubmissionDecision::kInhibited);
  ASSERT_EQ(
    forward.request_cancel(
      goal_id(1), forward_receipt.generation, SteadyTime{} + 1ms).status(),
    GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(
    forward.commit_mutation_submission(
      goal_id(1), forward_receipt.generation, 1, CoordinatorMutationKind::kReleaseTask,
      SteadyTime{}, SteadyTime{} + 100ms).decision,
    MutationSubmissionDecision::kInhibited);

  GoalAdmissionSlot replay;
  replay.update_readiness(true);
  const auto replay_receipt = replay.reserve(goal_id(2));
  ASSERT_TRUE(replay.activate(goal_id(2), replay_receipt.generation));
  ASSERT_EQ(
    replay.commit_mutation_submission(
      goal_id(2), replay_receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, SteadyTime{} + 100ms).decision,
    MutationSubmissionDecision::kCommitted);
  replay.inhibit("queue overflow");
  EXPECT_EQ(
    replay.commit_mutation_submission(
      goal_id(2), replay_receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
      SteadyTime{}, SteadyTime{} + 100ms).decision,
    MutationSubmissionDecision::kInhibited);
  EXPECT_TRUE(
    replay.resolve_mutation_submission(
      goal_id(2), replay_receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
}

TEST(GoalAdmissionSlot, CancellationAndSubmissionHaveOneLinearizationOrder)
{
  constexpr std::size_t kIterations = 128;
  for (std::size_t iteration = 0; iteration < kIterations; ++iteration) {
    GoalAdmissionSlot slot;
    slot.update_readiness(true);
    const auto receipt = slot.reserve(goal_id(1));
    ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
    std::barrier start_line(2);
    MutationSubmissionDecision submission = MutationSubmissionDecision::kInvalidArgument;
    GoalTerminationLatchStatus cancel_status = GoalTerminationLatchStatus::kInvalidArgument;
    std::thread submitter(
      [&]() {
        start_line.arrive_and_wait();
        submission = slot.commit_mutation_submission(
          goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
          SteadyTime{}, SteadyTime{} + 100ms).decision;
      });
    std::thread canceler(
      [&]() {
        start_line.arrive_and_wait();
        cancel_status = slot.request_cancel(
          goal_id(1), receipt.generation, SteadyTime{} + 1ms).status();
      });
    submitter.join();
    canceler.join();

    ASSERT_EQ(cancel_status, GoalTerminationLatchStatus::kLatched);
    const auto snapshot = slot.snapshot();
    ASSERT_TRUE(snapshot.cancel_requested);
    if (submission == MutationSubmissionDecision::kCommitted) {
      ASSERT_TRUE(snapshot.mutation_submission);
      EXPECT_EQ(
        snapshot.mutation_submission->phase, MutationSubmissionPhase::kCommitted);
      EXPECT_TRUE(
        slot.resolve_mutation_submission(
          goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
    } else {
      EXPECT_EQ(submission, MutationSubmissionDecision::kTerminationRequested);
      EXPECT_FALSE(snapshot.mutation_submission);
    }
  }
}

TEST(GoalAdmissionSlot, DrainAndSubmissionHaveOneLinearizationOrder)
{
  constexpr std::size_t kIterations = 128;
  for (std::size_t iteration = 0; iteration < kIterations; ++iteration) {
    GoalAdmissionSlot slot;
    slot.update_readiness(true);
    const auto receipt = slot.reserve(goal_id(1));
    ASSERT_TRUE(slot.activate(goal_id(1), receipt.generation));
    std::barrier start_line(2);
    MutationSubmissionDecision submission = MutationSubmissionDecision::kInvalidArgument;
    GoalTerminationLatchStatus drain_status = GoalTerminationLatchStatus::kInvalidArgument;
    std::thread submitter(
      [&]() {
        start_line.arrive_and_wait();
        submission = slot.commit_mutation_submission(
          goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask,
          SteadyTime{}, SteadyTime{} + 100ms).decision;
      });
    std::thread drainer(
      [&]() {
        start_line.arrive_and_wait();
        drain_status = slot.request_drain(
          goal_id(1), receipt.generation, SteadyTime{} + 1ms).status();
      });
    submitter.join();
    drainer.join();

    ASSERT_EQ(drain_status, GoalTerminationLatchStatus::kLatched);
    const auto snapshot = slot.snapshot();
    EXPECT_EQ(snapshot.termination_intent, GoalTerminationIntent::kShutdownDrain);
    if (submission == MutationSubmissionDecision::kCommitted) {
      ASSERT_TRUE(snapshot.mutation_submission);
      EXPECT_TRUE(
        slot.resolve_mutation_submission(
          goal_id(1), receipt.generation, 1, CoordinatorMutationKind::kReserveTask));
    } else {
      EXPECT_EQ(submission, MutationSubmissionDecision::kTerminationRequested);
      EXPECT_FALSE(snapshot.mutation_submission);
    }
  }
}

TEST(PendingOperationLedger, DeliversNormalCompletionAndRejectsOverlap)
{
  PendingOperationLedger ledger;
  const auto start = ledger.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_EQ(start.error, OperationStartError::kNone);
  ASSERT_TRUE(start.ticket);
  EXPECT_EQ(
    ledger.start(
      1, RestockTaskCommand::kSelectPair, OperationEffect::kReadOnly, "",
      SteadyTime{}, SteadyTime{} + 100ms).error,
    OperationStartError::kAlreadyPending);
  const auto completion = ledger.complete(
    1, start.ticket->operation_generation, SteadyTime{} + 99ms);
  EXPECT_EQ(completion.disposition, OperationCompletionDisposition::kDeliver);
  EXPECT_FALSE(ledger.pending());
}

TEST(PendingOperationLedger, RejectsIntermediateEvidenceForMutationTickets)
{
  PendingOperationLedger ledger;
  const auto started = ledger.start(
    3U, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  const auto evidence = ledger.classify_read_only_evidence(
    3U, started.ticket->operation_generation, SteadyTime{} + 10ms);
  EXPECT_EQ(evidence.disposition, OperationEvidenceDisposition::kRejectUnknown);
  ASSERT_TRUE(evidence.ticket);
  EXPECT_TRUE(ledger.pending());
}

TEST(PendingOperationLedger, ReadOnlyTimeoutAllowsProgressAndDiscardsLateReply)
{
  PendingOperationLedger ledger;
  const auto start = ledger.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(start.ticket);
  EXPECT_EQ(ledger.check_timeout(SteadyTime{} + 99ms), OperationTimeoutDisposition::kNotDue);
  EXPECT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms),
    OperationTimeoutDisposition::kDeliverTimeout);
  EXPECT_FALSE(ledger.pending());
  EXPECT_EQ(
    ledger.complete(
      1, start.ticket->operation_generation, SteadyTime{} + 101ms).disposition,
    OperationCompletionDisposition::kDiscardResolvedOrReadOnly);
}

TEST(PendingOperationLedger, MutationTimeoutCannotBeReplayedBeforeReconciliation)
{
  PendingOperationLedger ledger;
  const auto start = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(start.ticket);
  EXPECT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationRequired);
  EXPECT_EQ(
    ledger.complete(
      1, start.ticket->operation_generation, SteadyTime{} + 101ms).disposition,
    OperationCompletionDisposition::kReconcile);
  EXPECT_EQ(
    ledger.start(
      1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
      "reserve-1", SteadyTime{}, SteadyTime{} + 100ms).error,
    OperationStartError::kAlreadyPending);
  EXPECT_FALSE(ledger.resolve(2, start.ticket->operation_generation));
  EXPECT_TRUE(ledger.resolve(1, start.ticket->operation_generation));
  EXPECT_EQ(
    ledger.complete(
      1, start.ticket->operation_generation, SteadyTime{} + 101ms).disposition,
    OperationCompletionDisposition::kDiscardResolvedOrReadOnly);
}

TEST(PendingOperationLedger, RejectsInvalidRequestsAndPreservesAbsoluteDeadline)
{
  PendingOperationLedger ledger;
  EXPECT_EQ(
    ledger.start(
      0, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
      SteadyTime{}, SteadyTime{} + 1ms).error,
    OperationStartError::kInvalidArgument);
  EXPECT_EQ(
    ledger.start(
      1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
      "", SteadyTime{}, SteadyTime{} + 1ms).error,
    OperationStartError::kInvalidArgument);
  EXPECT_EQ(
    ledger.start(
      1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
      SteadyTime{} + 1ms, SteadyTime{} + 1ms).error,
    OperationStartError::kInvalidArgument);
  EXPECT_EQ(
    ledger.start(
      1, RestockTaskCommand::kValidateScene, static_cast<OperationEffect>(255),
      "invalid-effect", SteadyTime{}, SteadyTime{} + 1ms).error,
    OperationStartError::kInvalidArgument);
  const auto start = ledger.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{} + 5ms, SteadyTime{} + 10ms);
  ASSERT_TRUE(start.ticket);
  EXPECT_EQ(start.ticket->deadline, SteadyTime{} + 10ms);

  PendingOperationLedger invalid_policy({0ms, 1ms, 1U});
  EXPECT_EQ(
    invalid_policy.start(
      1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
      SteadyTime{}, SteadyTime{} + 1ms).error,
    OperationStartError::kInvalidArgument);
}

TEST(PendingOperationLedger, OwnsBoundedReconciliationAttempts)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask,
    OperationEffect::kIdempotentMutation, "reserve-1",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  const auto operation_generation = started.ticket->operation_generation;
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);

  EXPECT_EQ(
    ledger.begin_reconciliation(
      2, operation_generation, ReconciliationKind::kReplayMutation,
      SteadyTime{} + 100ms).error,
    ReconciliationStartError::kNotRequired);
  const auto first = ledger.begin_reconciliation(
    1, operation_generation, ReconciliationKind::kReplayMutation,
    SteadyTime{} + 100ms);
  ASSERT_EQ(first.error, ReconciliationStartError::kNone);
  ASSERT_TRUE(first.attempt);
  EXPECT_EQ(first.attempt->attempt_number, 1U);
  EXPECT_EQ(first.attempt->deadline, SteadyTime{} + 200ms);
  EXPECT_EQ(first.attempt->window_deadline, SteadyTime{} + 600ms);
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, operation_generation, ReconciliationKind::kReadback,
      SteadyTime{} + 100ms).error,
    ReconciliationStartError::kAttemptAlreadyActive);
  EXPECT_EQ(
    ledger.complete_reconciliation(
      1, operation_generation, ReconciliationKind::kReplayMutation, 2U,
      SteadyTime{} + 150ms),
    ReconciliationCompletionDisposition::kRejectUnknown);
  ASSERT_EQ(
    ledger.complete_reconciliation(
      1, operation_generation, ReconciliationKind::kReplayMutation, 1U,
      SteadyTime{} + 150ms, ReconciliationEvidence::kMutationResponseRetained),
    ReconciliationCompletionDisposition::kCompleted);

  const auto second = ledger.begin_reconciliation(
    1, operation_generation, ReconciliationKind::kReadback,
    SteadyTime{} + 150ms);
  ASSERT_TRUE(second.attempt);
  EXPECT_EQ(second.attempt->attempt_number, 2U);
  EXPECT_EQ(ledger.check_timeout(SteadyTime{} + 249ms), OperationTimeoutDisposition::kNotDue);
  EXPECT_EQ(
    ledger.complete_reconciliation(
      1, operation_generation, ReconciliationKind::kReadback, 2U,
      SteadyTime{} + 250ms),
    ReconciliationCompletionDisposition::kAttemptTimedOut);
  EXPECT_EQ(
    ledger.complete_reconciliation(
      1, operation_generation, ReconciliationKind::kReadback, 2U,
      SteadyTime{} + 251ms),
    ReconciliationCompletionDisposition::kRejectUnknown);

  const auto third = ledger.begin_reconciliation(
    1, operation_generation, ReconciliationKind::kReadback,
    SteadyTime{} + 250ms);
  ASSERT_TRUE(third.attempt);
  EXPECT_EQ(third.attempt->attempt_number, 3U);
  EXPECT_EQ(
    ledger.check_timeout(SteadyTime{} + 350ms),
    OperationTimeoutDisposition::kReconciliationExhausted);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationExhausted);
  EXPECT_TRUE(ledger.pending());
  EXPECT_FALSE(ledger.active_reconciliation());
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, operation_generation, ReconciliationKind::kReadback,
      SteadyTime{} + 351ms).error,
    ReconciliationStartError::kExhausted);
  EXPECT_EQ(
    ledger.complete(1, operation_generation, SteadyTime{} + 351ms).disposition,
    OperationCompletionDisposition::kReconciliationExhausted);
  EXPECT_TRUE(ledger.resolve(1, operation_generation));
}

TEST(PendingOperationLedger, OriginalMutationReplyCannotOutrunWindowExpiry)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);

  EXPECT_EQ(
    ledger.complete(
      1, started.ticket->operation_generation, SteadyTime{} + 600ms).disposition,
    OperationCompletionDisposition::kReconciliationExhausted);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationExhausted);
  EXPECT_TRUE(ledger.pending());
  EXPECT_EQ(
    ledger.check_timeout(SteadyTime{} + 600ms), OperationTimeoutDisposition::kNotDue);
}

TEST(PendingOperationLedger, OriginalMutationReplySupersedesActiveReconciliationAttempt)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  ASSERT_TRUE(
    ledger.begin_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
      SteadyTime{} + 100ms).attempt);

  EXPECT_EQ(
    ledger.complete(
      1, started.ticket->operation_generation, SteadyTime{} + 150ms).disposition,
    OperationCompletionDisposition::kReconcile);
  EXPECT_FALSE(ledger.active_reconciliation());
  EXPECT_EQ(ledger.next_reconciliation_kind(), ReconciliationKind::kReadback);
}

TEST(PendingOperationLedger, ReleaseReadbackEvidenceSelectsExactReplay)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReleaseTaskReservation,
    OperationEffect::kIdempotentMutation, "release-1", SteadyTime{},
    SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  const auto readback = ledger.begin_reconciliation(
    1, started.ticket->operation_generation, ReconciliationKind::kReadback,
    SteadyTime{} + 100ms);
  ASSERT_TRUE(readback.attempt);
  ASSERT_EQ(
    ledger.complete_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReadback, 1U,
      SteadyTime{} + 150ms, ReconciliationEvidence::kExactMutationStillApplied),
    ReconciliationCompletionDisposition::kCompleted);
  EXPECT_EQ(ledger.next_reconciliation_kind(), ReconciliationKind::kReplayMutation);
}

TEST(PendingOperationLedger, RejectsEvidenceOutsideTheActiveStrategyMatrix)
{
  PendingOperationLedger reserve({500ms, 100ms, 3U});
  const auto reserve_started = reserve.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(reserve_started.ticket);
  ASSERT_EQ(
    reserve.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  ASSERT_TRUE(
    reserve.begin_reconciliation(
      1, reserve_started.ticket->operation_generation,
      ReconciliationKind::kReplayMutation, SteadyTime{} + 100ms).attempt);
  EXPECT_EQ(
    reserve.complete_reconciliation(
      1, reserve_started.ticket->operation_generation,
      ReconciliationKind::kReplayMutation, 1U, SteadyTime{} + 150ms,
      ReconciliationEvidence::kExactMutationStillApplied),
    ReconciliationCompletionDisposition::kRejectUnknown);
  EXPECT_TRUE(reserve.active_reconciliation());
  EXPECT_EQ(
    reserve.complete_reconciliation(
      1, reserve_started.ticket->operation_generation,
      ReconciliationKind::kReplayMutation, 1U, SteadyTime{} + 150ms,
      static_cast<ReconciliationEvidence>(255)),
    ReconciliationCompletionDisposition::kRejectUnknown);
  EXPECT_TRUE(reserve.active_reconciliation());
}

TEST(PendingOperationLedger, ConfirmedNotSubmittedNeverEntersReconciliation)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  EXPECT_TRUE(ledger.confirm_not_submitted(1, started.ticket->operation_generation));
  EXPECT_FALSE(ledger.pending());
  EXPECT_FALSE(ledger.confirm_not_submitted(1, started.ticket->operation_generation));
}

TEST(PendingOperationLedger, UnknownOutcomeUsesEffectSpecificSafetyBoundary)
{
  PendingOperationLedger read_only;
  const auto read_started = read_only.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(read_started.ticket);
  EXPECT_EQ(
    read_only.report_unknown_outcome(
      1, read_started.ticket->operation_generation, SteadyTime{} + 20ms).disposition,
    OperationCompletionDisposition::kDeliver);
  EXPECT_FALSE(read_only.pending());

  PendingOperationLedger mutation({500ms, 100ms, 3U});
  const auto mutation_started = mutation.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(mutation_started.ticket);
  EXPECT_EQ(
    mutation.report_unknown_outcome(
      1, mutation_started.ticket->operation_generation, SteadyTime{} + 20ms).disposition,
    OperationCompletionDisposition::kReconcile);
  EXPECT_EQ(mutation.phase(), PendingOperationPhase::kReconciliationRequired);
  const auto attempt = mutation.begin_reconciliation(
    1, mutation_started.ticket->operation_generation,
    ReconciliationKind::kReplayMutation, SteadyTime{} + 20ms);
  ASSERT_TRUE(attempt.attempt);
  EXPECT_EQ(attempt.attempt->window_deadline, SteadyTime{} + 520ms);
}

TEST(PendingOperationLedger, ClampsAttemptsToTheReconciliationWindow)
{
  PendingOperationLedger ledger({150ms, 100ms, 5U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReleaseTaskReservation,
    OperationEffect::kIdempotentMutation, "release-1",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  const auto attempt = ledger.begin_reconciliation(
    1, started.ticket->operation_generation, ReconciliationKind::kReadback,
    SteadyTime{} + 200ms);
  ASSERT_TRUE(attempt.attempt);
  EXPECT_EQ(attempt.attempt->deadline, SteadyTime{} + 250ms);
  EXPECT_EQ(attempt.attempt->window_deadline, SteadyTime{} + 250ms);
  EXPECT_EQ(
    ledger.complete_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReadback, 1U,
      SteadyTime{} + 250ms),
    ReconciliationCompletionDisposition::kExhausted);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationExhausted);
}

TEST(PendingOperationLedger, ReportsAttemptTimeoutBeforeReconciliationExhaustion)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  const auto first = ledger.begin_reconciliation(
    1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
    SteadyTime{} + 100ms);
  ASSERT_TRUE(first.attempt);
  EXPECT_EQ(
    ledger.check_timeout(SteadyTime{} + 200ms),
    OperationTimeoutDisposition::kReconciliationAttemptTimedOut);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationRequired);
  EXPECT_FALSE(ledger.active_reconciliation());
  const auto second = ledger.begin_reconciliation(
    1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
    SteadyTime{} + 200ms);
  ASSERT_TRUE(second.attempt);
  EXPECT_EQ(second.attempt->attempt_number, 2U);
}

TEST(PendingOperationLedger, ExhaustsAfterInconclusiveFinalAttempt)
{
  PendingOperationLedger ledger({500ms, 100ms, 1U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  const auto attempt = ledger.begin_reconciliation(
    1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
    SteadyTime{} + 100ms);
  ASSERT_TRUE(attempt.attempt);
  ASSERT_EQ(
    ledger.complete_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation, 1U,
      SteadyTime{} + 150ms),
    ReconciliationCompletionDisposition::kCompleted);
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
      SteadyTime{} + 150ms).error,
    ReconciliationStartError::kExhausted);
  EXPECT_EQ(ledger.phase(), PendingOperationPhase::kReconciliationExhausted);
}

TEST(PendingOperationLedger, ReserveReconciliationRequiresExactReplay)
{
  PendingOperationLedger ledger({500ms, 100ms, 3U});
  const auto started = ledger.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(started.ticket);
  ASSERT_EQ(
    ledger.check_timeout(SteadyTime{} + 100ms), OperationTimeoutDisposition::kReconcile);
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReadback,
      SteadyTime{} + 100ms).error,
    ReconciliationStartError::kStrategyNotAllowed);
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
      SteadyTime{} + 100ms).error,
    ReconciliationStartError::kNone);
  EXPECT_EQ(
    ledger.complete_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReplayMutation,
      1U, SteadyTime{} + 150ms,
      ReconciliationEvidence::kMutationResponseRetained),
    ReconciliationCompletionDisposition::kCompleted);
  EXPECT_EQ(
    ledger.begin_reconciliation(
      1, started.ticket->operation_generation, ReconciliationKind::kReadback,
      SteadyTime{} + 150ms).error,
    ReconciliationStartError::kNone);
}

TEST(PendingOperationLedger, ClassifiesCompletionsAtAbsoluteDeadline)
{
  PendingOperationLedger read_only;
  const auto read_started = read_only.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(read_started.ticket);
  EXPECT_EQ(
    read_only.complete(
      1, read_started.ticket->operation_generation, SteadyTime{} + 100ms).disposition,
    OperationCompletionDisposition::kDeliverTimeout);
  EXPECT_FALSE(read_only.pending());

  PendingOperationLedger mutation({500ms, 100ms, 3U});
  const auto mutation_started = mutation.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(mutation_started.ticket);
  EXPECT_EQ(
    mutation.complete(
      1, mutation_started.ticket->operation_generation,
      SteadyTime{} + 100ms).disposition,
    OperationCompletionDisposition::kReconcile);
  EXPECT_EQ(mutation.phase(), PendingOperationPhase::kReconciliationRequired);
  EXPECT_EQ(
    mutation.begin_reconciliation(
      1, mutation_started.ticket->operation_generation,
      ReconciliationKind::kReplayMutation, SteadyTime{} + 100ms).error,
    ReconciliationStartError::kNone);
}

TEST(PendingOperationLedger, ClassifiesUnknownOutcomesAtAbsoluteDeadline)
{
  PendingOperationLedger read_only;
  const auto read_started = read_only.start(
    1, RestockTaskCommand::kValidateScene, OperationEffect::kReadOnly, "",
    SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(read_started.ticket);
  EXPECT_EQ(
    read_only.report_unknown_outcome(
      1, read_started.ticket->operation_generation, SteadyTime{} + 100ms).disposition,
    OperationCompletionDisposition::kDeliverTimeout);

  PendingOperationLedger mutation({500ms, 100ms, 3U});
  const auto mutation_started = mutation.start(
    1, RestockTaskCommand::kReserveTask, OperationEffect::kIdempotentMutation,
    "reserve-1", SteadyTime{}, SteadyTime{} + 100ms);
  ASSERT_TRUE(mutation_started.ticket);
  EXPECT_EQ(
    mutation.report_unknown_outcome(
      1, mutation_started.ticket->operation_generation,
      SteadyTime{} + 101ms).disposition,
    OperationCompletionDisposition::kReconcile);
}

}  // namespace
}  // namespace restocker_task_executor
