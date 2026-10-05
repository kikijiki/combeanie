// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <barrier>
#include <ranges>
#include <array>
#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <thread>
#include <type_traits>
#include <vector>

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/restock_coordinator_inbox.hpp"

namespace restocker_task_executor
{
using CoordinatorInboxPushResult = CoordinatorInboxDepositStatus;
using PreGraspMoveGroupDeposit = CoordinatorInboxDepositStatus;
using CoordinatorCleanupDeposit = CoordinatorInboxDepositStatus;

[[nodiscard]] bool operator==(
  CoordinatorInboxDepositResult result, CoordinatorInboxDepositStatus status) noexcept
{
  if (result.status != status) {
    return false;
  }
  switch (status) {
    case CoordinatorInboxDepositStatus::kAccepted:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned;
    case CoordinatorInboxDepositStatus::kOverflowLatched:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned ||
             result.persistence == CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned;
    case CoordinatorInboxDepositStatus::kInhibited:
      return result.persistence == CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned ||
             result.persistence == CoordinatorInboxPersistenceStatus::kUnresolved;
    case CoordinatorInboxDepositStatus::kEvidenceLost:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned;
    case CoordinatorInboxDepositStatus::kEvidenceConflict:
      return result.persistence == CoordinatorInboxPersistenceStatus::kUnresolved;
  }
  return false;
}

namespace
{

using namespace std::chrono_literals;

CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId value{};
  for (std::size_t index = 0; index < value.size(); ++index) {
    value[index] = static_cast<std::uint8_t>(seed + index);
  }
  return value;
}

std::shared_ptr<CoordinatorGenerationQuiescence> generation_quiescence(
  GoalGeneration generation)
{
  return std::make_shared<CoordinatorGenerationQuiescence>(goal_id(), generation, 1U);
}

void expect_deposit(
  CoordinatorInboxDepositResult actual, CoordinatorInboxDepositStatus status,
  CoordinatorInboxPersistenceStatus persistence)
{
  EXPECT_EQ(actual.status, status);
  EXPECT_EQ(actual.persistence, persistence);
}


void fill_cleanup_emergency(
  CoordinatorInbox & inbox, GoalGeneration generation,
  const std::shared_ptr<restocker_interfaces::srv::ReleaseTaskReservation::Response> & response)
{
  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    expect_deposit(
      inbox.push_cleanup(
        ReleaseReservationCompletion{
          {generation, index + 1U}, response, "",
          SteadyTime{} + std::chrono::milliseconds(index), std::nullopt}),
      CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned);
  }
}

TEST(CoordinatorInbox, PreservesFifoOrderAcrossTypedIngress)
{
  CoordinatorInbox inbox(2U);
  SelectionRequest selection;
  selection.object_id = restocker_world_state::ObjectId{17U};
  selection.lane_id = restocker_world_state::LaneId{"lane_01"};
  auto gate = generation_quiescence(7U);
  const auto * gate_identity = gate.get();
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_gate = gate;
  EXPECT_EQ(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 7U, selection, SteadyTime{} + 1ms,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME), std::nullopt, gate, true},
      std::nullopt, SteadyTime{} + 1ms),
    CoordinatorInboxPushResult::kAccepted);
  gate.reset();
  EXPECT_FALSE(weak_gate.expired());

  auto response = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  EXPECT_EQ(
    inbox.push(
      SnapshotCompletion{
        {7U, 11U}, response, "", SteadyTime{} + 2ms, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);

  auto first = inbox.try_pop();
  ASSERT_TRUE(first);
  const auto * accepted = std::get_if<CoordinatorAcceptedGoal>(&*first);
  ASSERT_NE(accepted, nullptr);
  EXPECT_EQ(accepted->goal_id, goal_id());
  EXPECT_EQ(accepted->goal_generation, 7U);
  EXPECT_EQ(accepted->selection_request.object_id, selection.object_id);
  EXPECT_EQ(accepted->selection_request.lane_id, selection.lane_id);
  EXPECT_EQ(accepted->steady_started, SteadyTime{} + 1ms);
  EXPECT_EQ(accepted->simulation_started.nanoseconds(), 2'000'000'000LL);
  EXPECT_EQ(accepted->generation_quiescence.get(), gate_identity);
  EXPECT_TRUE(accepted->sealed_pending_control_handoff);
  first.reset();
  EXPECT_TRUE(weak_gate.expired());

  const auto second = inbox.try_pop();
  ASSERT_TRUE(second);
  const auto * completion = std::get_if<SnapshotCompletion>(&*second);
  ASSERT_NE(completion, nullptr);
  EXPECT_EQ(completion->correlation.goal_generation, 7U);
  EXPECT_EQ(completion->correlation.operation_generation, 11U);
  EXPECT_EQ(completion->response, response);
  EXPECT_EQ(completion->arrived_at, SteadyTime{} + 2ms);
  EXPECT_FALSE(inbox.try_pop());
}

TEST(CoordinatorInbox, AtomicallyRetainsAcceptedGoalBeforePendingTermination)
{
  CoordinatorInbox inbox(2U);
  const auto authority_at = SteadyTime{} + 7ms;
  auto gate = generation_quiescence(7U);
  const auto * gate_identity = gate.get();
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_gate = gate;
  expect_deposit(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 7U, {}, SteadyTime{} + 1ms,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME), std::nullopt, gate, true},
      CoordinatorControlEvent{
        CoordinatorControlKind::kAuthorityFaultSafeAbortRequested,
        7U, 0U, authority_at, "pending authority loss"},
      SteadyTime{} + 3ms),
    CoordinatorInboxDepositStatus::kAccepted,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  gate.reset();
  EXPECT_FALSE(weak_gate.expired());

  auto accepted_delivery = inbox.try_pop();
  ASSERT_TRUE(accepted_delivery);
  const auto * accepted = std::get_if<CoordinatorAcceptedGoal>(&*accepted_delivery);
  ASSERT_NE(accepted, nullptr);
  EXPECT_EQ(accepted->generation_quiescence.get(), gate_identity);
  EXPECT_TRUE(accepted->sealed_pending_control_handoff);
  accepted_delivery.reset();
  EXPECT_TRUE(weak_gate.expired());
  const auto termination_delivery = inbox.try_pop();
  ASSERT_TRUE(termination_delivery);
  const auto * termination = std::get_if<CoordinatorControlEvent>(&*termination_delivery);
  ASSERT_NE(termination, nullptr);
  EXPECT_EQ(termination->kind, CoordinatorControlKind::kAuthorityFaultSafeAbortRequested);
  EXPECT_EQ(termination->goal_generation, 7U);
  EXPECT_EQ(termination->arrived_at, authority_at);
  EXPECT_FALSE(inbox.try_pop());
}

TEST(CoordinatorInbox, CapacityOverflowRetainsWholeAcceptedGoalHandoffInEmergencyLane)
{
  CoordinatorInbox inbox(1U);
  auto gate = generation_quiescence(7U);
  const auto * gate_identity = gate.get();
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_gate = gate;
  expect_deposit(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 7U, {}, SteadyTime{} + 1ms,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME), std::nullopt, gate, true},
      CoordinatorControlEvent{
        CoordinatorControlKind::kAuthorityFaultSafeAbortRequested,
        7U, 0U, SteadyTime{} + 2ms, "pending authority loss"},
      SteadyTime{} + 3ms),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  gate.reset();
  EXPECT_FALSE(weak_gate.expired());

  const auto overflow_delivery = inbox.try_pop();
  ASSERT_TRUE(overflow_delivery);
  const auto * overflow = std::get_if<CoordinatorOverflowMarker>(&*overflow_delivery);
  ASSERT_NE(overflow, nullptr);
  EXPECT_EQ(overflow->goal_generation, 7U);
  EXPECT_EQ(overflow->arrived_at, SteadyTime{} + 3ms);
  EXPECT_EQ(
    overflow->detail, "coordinator accepted-goal handoff used emergency storage");
  auto accepted_delivery = inbox.try_pop();
  ASSERT_TRUE(accepted_delivery);
  const auto * accepted = std::get_if<CoordinatorAcceptedGoal>(&*accepted_delivery);
  ASSERT_NE(accepted, nullptr);
  EXPECT_EQ(accepted->goal_id, goal_id());
  EXPECT_EQ(accepted->goal_generation, 7U);
  EXPECT_EQ(accepted->generation_quiescence.get(), gate_identity);
  EXPECT_TRUE(accepted->sealed_pending_control_handoff);
  accepted_delivery.reset();
  EXPECT_TRUE(weak_gate.expired());
  const auto termination_delivery = inbox.try_pop();
  ASSERT_TRUE(termination_delivery);
  const auto * termination = std::get_if<CoordinatorControlEvent>(&*termination_delivery);
  ASSERT_NE(termination, nullptr);
  EXPECT_EQ(termination->kind, CoordinatorControlKind::kAuthorityFaultSafeAbortRequested);
  EXPECT_EQ(termination->goal_generation, 7U);
  EXPECT_FALSE(inbox.try_pop());
  const auto snapshot = inbox.snapshot();
  EXPECT_TRUE(snapshot.overflow_latched);
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.accepted_handoff_emergency_size, 0U);
}

TEST(CoordinatorInbox, ConflictingEmergencyHandoffPreservesFirstBatch)
{
  CoordinatorInbox inbox(1U);
  const CoordinatorGoalId first_goal_id = goal_id();
  CoordinatorGoalId second_goal_id = first_goal_id;
  second_goal_id[0] ^= 0xFFU;
  auto first_gate = generation_quiescence(7U);
  auto second_gate = generation_quiescence(8U);
  const auto * first_gate_identity = first_gate.get();
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_first_gate = first_gate;
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_second_gate = second_gate;
  ASSERT_EQ(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        first_goal_id, 7U, {}, SteadyTime{} + 1ms,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME), std::nullopt,
        std::move(first_gate)},
      CoordinatorControlEvent{
        CoordinatorControlKind::kShutdown, 7U, 0U, SteadyTime{} + 2ms, "first"},
      SteadyTime{} + 5ms),
    CoordinatorInboxPushResult::kOverflowLatched);
  EXPECT_EQ(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        second_goal_id, 8U, {}, SteadyTime{} + 3ms,
        rclcpp::Time(std::int64_t{3'000'000'000}, RCL_ROS_TIME), std::nullopt,
        std::move(second_gate)},
      CoordinatorControlEvent{
        CoordinatorControlKind::kShutdown, 8U, 0U, SteadyTime{} + 4ms, "second"},
      SteadyTime{} + 6ms),
    CoordinatorInboxPushResult::kInhibited);
  EXPECT_FALSE(weak_first_gate.expired());
  EXPECT_TRUE(weak_second_gate.expired());
  const auto before_delivery = inbox.snapshot();
  EXPECT_EQ(before_delivery.accepted_handoff_emergency_size, 2U);

  const auto overflow_delivery = inbox.try_pop();
  ASSERT_TRUE(overflow_delivery);
  EXPECT_NE(std::get_if<CoordinatorOverflowMarker>(&*overflow_delivery), nullptr);
  auto accepted_delivery = inbox.try_pop();
  ASSERT_TRUE(accepted_delivery);
  const auto * accepted = std::get_if<CoordinatorAcceptedGoal>(&*accepted_delivery);
  ASSERT_NE(accepted, nullptr);
  EXPECT_EQ(accepted->goal_id, first_goal_id);
  EXPECT_EQ(accepted->goal_generation, 7U);
  EXPECT_EQ(accepted->generation_quiescence.get(), first_gate_identity);
  accepted_delivery.reset();
  EXPECT_TRUE(weak_first_gate.expired());
  const auto termination_delivery = inbox.try_pop();
  ASSERT_TRUE(termination_delivery);
  const auto * termination = std::get_if<CoordinatorControlEvent>(&*termination_delivery);
  ASSERT_NE(termination, nullptr);
  EXPECT_EQ(termination->goal_generation, 7U);
  EXPECT_EQ(termination->detail, "first");
  EXPECT_FALSE(inbox.try_pop());
}

TEST(CoordinatorInbox, OverflowedInboxStillAcceptsOneEmergencyHandoff)
{
  CoordinatorInbox inbox(1U);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 1U, 0U, SteadyTime{} + 1ms, "queued"}),
    CoordinatorInboxPushResult::kAccepted);
  EXPECT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 1U, 0U, SteadyTime{} + 2ms, "overflow"}),
    CoordinatorInboxPushResult::kOverflowLatched);
  const auto overflow_delivery = inbox.try_pop();
  ASSERT_TRUE(overflow_delivery);
  EXPECT_NE(std::get_if<CoordinatorOverflowMarker>(&*overflow_delivery), nullptr);

  EXPECT_EQ(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 1U, {}, SteadyTime{} + 3ms,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME), std::nullopt},
      std::nullopt, SteadyTime{} + 3ms),
    CoordinatorInboxPushResult::kOverflowLatched);
  const auto repeated_overflow_delivery = inbox.try_pop();
  ASSERT_TRUE(repeated_overflow_delivery);
  EXPECT_NE(
    std::get_if<CoordinatorOverflowMarker>(&*repeated_overflow_delivery), nullptr);
  const auto accepted_delivery = inbox.try_pop();
  ASSERT_TRUE(accepted_delivery);
  EXPECT_NE(std::get_if<CoordinatorAcceptedGoal>(&*accepted_delivery), nullptr);
  const auto queued_delivery = inbox.try_pop();
  ASSERT_TRUE(queued_delivery);
  const auto * queued = std::get_if<CoordinatorControlEvent>(&*queued_delivery);
  ASSERT_NE(queued, nullptr);
  EXPECT_EQ(queued->detail, "queued");
  EXPECT_FALSE(inbox.try_pop());
}

TEST(CoordinatorInbox, RetainsReservationCleanupEvidenceAfterOrdinaryOverflow)
{
  CoordinatorInbox inbox(1U);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 4U, 0U, SteadyTime{}, "queued"}),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kShutdown, 4U, 0U, SteadyTime{}, "overflow"}),
    CoordinatorInboxPushResult::kOverflowLatched);

  auto release =
    std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  auto readback = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  EXPECT_EQ(
    inbox.push_cleanup(
      ReleaseReservationCompletion{
        {4U, 12U}, release, "", SteadyTime{} + 1ms, std::nullopt}),
    CoordinatorCleanupDeposit::kAccepted);
  EXPECT_EQ(
    inbox.push_cleanup(
      SnapshotCompletion{
        {4U, 13U}, readback, "", SteadyTime{} + 2ms, std::nullopt}),
    CoordinatorCleanupDeposit::kAccepted);

  const auto overflow = inbox.try_pop();
  ASSERT_TRUE(overflow);
  ASSERT_NE(std::get_if<CoordinatorOverflowMarker>(&*overflow), nullptr);
  const auto released = inbox.try_pop();
  ASSERT_TRUE(released);
  EXPECT_EQ(std::get<ReleaseReservationCompletion>(*released).response, release);
  const auto snapshot = inbox.try_pop();
  ASSERT_TRUE(snapshot);
  EXPECT_EQ(std::get<SnapshotCompletion>(*snapshot).response, readback);
  const auto queued = inbox.try_pop();
  ASSERT_TRUE(queued);
  EXPECT_EQ(
    std::get<CoordinatorControlEvent>(*queued).kind,
    CoordinatorControlKind::kHeartbeat);

  const auto state = inbox.snapshot();
  EXPECT_TRUE(state.overflow_latched);
  EXPECT_EQ(state.cleanup_emergency_size, 0U);
  EXPECT_FALSE(state.cleanup_evidence_lost);
}

TEST(CoordinatorInbox, ReportsCleanupEvidenceLossWhenReservedLaneIsExhausted)
{
  CoordinatorInbox inbox(1U);
  auto response =
    std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    EXPECT_EQ(
      inbox.push_cleanup(
        ReleaseReservationCompletion{
          {2U, index + 1U}, response, "", SteadyTime{} +
          std::chrono::milliseconds(index), std::nullopt}),
      CoordinatorCleanupDeposit::kAccepted);
  }
  EXPECT_EQ(
    inbox.push_cleanup(
      ReleaseReservationCompletion{
        {2U, 99U}, response, "", SteadyTime{} + 9ms, std::nullopt}),
    CoordinatorCleanupDeposit::kEvidenceLost);
  const auto state = inbox.snapshot();
  EXPECT_EQ(
    state.cleanup_emergency_size, CoordinatorInbox::kCleanupEmergencyCapacity);
  EXPECT_TRUE(state.cleanup_evidence_lost);
  EXPECT_TRUE(state.overflow_latched);
  EXPECT_FALSE(state.overflow_notification_pending);

  const auto fault = inbox.try_pop();
  ASSERT_TRUE(fault);
  const auto * marker = std::get_if<CoordinatorCleanupEvidenceLossMarker>(&*fault);
  ASSERT_NE(marker, nullptr);
  EXPECT_EQ(marker->kind, CoordinatorCleanupEvidenceKind::kReleaseReservation);
  EXPECT_EQ(marker->correlation.goal_generation, 2U);
  EXPECT_EQ(marker->correlation.operation_generation, 99U);
  EXPECT_EQ(marker->arrived_at, SteadyTime{} + 9ms);
}

TEST(CoordinatorInbox, PreservesOrderAcrossRingWraparound)
{
  CoordinatorInbox inbox(2U);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 1U, 1U, SteadyTime{}, "first"}),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 1U, 2U, SteadyTime{}, "second"}),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_TRUE(inbox.try_pop());
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 1U, 3U, SteadyTime{}, "third"}),
    CoordinatorInboxPushResult::kAccepted);

  const auto second = inbox.try_pop();
  const auto third = inbox.try_pop();
  ASSERT_TRUE(second);
  ASSERT_TRUE(third);
  EXPECT_EQ(std::get<CoordinatorControlEvent>(*second).operation_generation, 2U);
  EXPECT_EQ(std::get<CoordinatorControlEvent>(*third).operation_generation, 3U);
}

TEST(CoordinatorInbox, PreservesReconciliationAttemptCorrelation)
{
  CoordinatorInbox inbox(1U);
  auto response = std::make_shared<restocker_interfaces::srv::ReserveTask::Response>();
  ASSERT_EQ(
    inbox.push(
      ReserveTaskCompletion{
        {3U, 9U}, response, "", SteadyTime{} + 12ms,
        CoordinatorReconciliationCorrelation{ReconciliationKind::kReplayMutation, 2U}}),
    CoordinatorInboxPushResult::kAccepted);

  const auto delivered = inbox.try_pop();
  ASSERT_TRUE(delivered);
  const auto & completion = std::get<ReserveTaskCompletion>(*delivered);
  ASSERT_TRUE(completion.reconciliation);
  EXPECT_EQ(completion.reconciliation->kind, ReconciliationKind::kReplayMutation);
  EXPECT_EQ(completion.reconciliation->attempt_number, 2U);
  EXPECT_EQ(completion.arrived_at, SteadyTime{} + 12ms);
}

TEST(CoordinatorInbox, SerializesConcurrentProducersWithoutLoss)
{
  constexpr std::size_t kThreadCount = 32U;
  CoordinatorInbox inbox(kThreadCount);
  std::barrier start_line(static_cast<std::ptrdiff_t>(kThreadCount));
  std::array<CoordinatorInboxPushResult, kThreadCount> pushed{};
  std::vector<std::thread> threads;
  threads.reserve(kThreadCount);
  for (std::size_t index = 0; index < kThreadCount; ++index) {
    threads.emplace_back(
      [&, index]() {
        start_line.arrive_and_wait();
        pushed[index] = inbox.push(
          CoordinatorControlEvent{
            CoordinatorControlKind::kStateDeadline, 1U, index + 1U,
            SteadyTime{}, ""}).status;
      });
  }
  for (auto & thread : threads) {
    thread.join();
  }
  EXPECT_TRUE(
    std::ranges::all_of(
      pushed, [](CoordinatorInboxPushResult result) {
        return result == CoordinatorInboxPushResult::kAccepted;
      }));

  std::set<OperationGeneration> observed;
  while (const auto item = inbox.try_pop()) {
    observed.insert(std::get<CoordinatorControlEvent>(*item).operation_generation);
  }
  EXPECT_EQ(observed.size(), kThreadCount);
  EXPECT_FALSE(inbox.snapshot().overflow_latched);
}

TEST(CoordinatorInbox, RejectsZeroCapacity)
{
  EXPECT_THROW((void)CoordinatorInbox(0U), std::invalid_argument);
}

TEST(CoordinatorInbox, PublicGenericPushCannotAcceptAcceptedGoal)
{
  static_assert(std::is_trivially_copyable_v<CoordinatorInboxDepositResult>);
  static_assert(!std::is_constructible_v<CoordinatorInboxPushItem, CoordinatorAcceptedGoal>);
}

TEST(CoordinatorInbox, RejectsInvalidGenerationBeforeCapacityAndStaysConflicted)
{
  CoordinatorInbox inbox(1U);
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 0U, 0U, SteadyTime{} + 1ms, "invalid"}),
    CoordinatorInboxDepositStatus::kEvidenceConflict,
    CoordinatorInboxPersistenceStatus::kUnresolved);
  EXPECT_TRUE(inbox.snapshot().generation_accounting_conflict);
  EXPECT_FALSE(inbox.try_pop());
  EXPECT_FALSE(inbox.generation_empty(0U));
  EXPECT_FALSE(inbox.generation_empty(7U));
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 2ms, "valid"}),
    CoordinatorInboxDepositStatus::kEvidenceConflict,
    CoordinatorInboxPersistenceStatus::kUnresolved);
}

TEST(CoordinatorInbox, OverflowMarkerCoalescesAndConflictsWithoutOverwrite)
{
  CoordinatorInbox inbox(1U);
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 20ms, "queued"}),
    CoordinatorInboxDepositStatus::kAccepted,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 10ms, "first loss"}),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned);
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 5ms, "same loss"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned);
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 8U, 0U, SteadyTime{} + 1ms, "foreign loss"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kUnresolved);

  const auto delivery = inbox.try_pop();
  ASSERT_TRUE(delivery);
  const auto & marker = std::get<CoordinatorOverflowMarker>(*delivery);
  EXPECT_EQ(marker.goal_generation, 7U);
  EXPECT_EQ(marker.arrived_at, SteadyTime{} + 5ms);
  EXPECT_EQ(marker.detail, "coordinator inbox ordinary capacity exhausted");
  EXPECT_TRUE(inbox.snapshot().generation_accounting_conflict);
  EXPECT_FALSE(inbox.generation_empty(0U));
  EXPECT_FALSE(inbox.generation_empty(7U));
  EXPECT_FALSE(inbox.generation_empty(8U));
  EXPECT_FALSE(inbox.generation_empty(42U));
  EXPECT_FALSE(
    inbox.generation_empty(std::numeric_limits<GoalGeneration>::max()));
}

TEST(CoordinatorInbox, OverflowMarkerCanBeReinstalledAfterPop)
{
  CoordinatorInbox inbox(1U);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{}, "queued"}),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 1ms, "loss"}),
    CoordinatorInboxPushResult::kOverflowLatched);
  ASSERT_TRUE(inbox.try_pop());
  expect_deposit(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 2ms, "later loss"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned);
  const auto delivery = inbox.try_pop();
  ASSERT_TRUE(delivery);
  EXPECT_EQ(std::get<CoordinatorOverflowMarker>(*delivery).arrived_at, SteadyTime{} + 2ms);
}

TEST(CoordinatorInbox, CleanupLossMarkerCoalescesAndCanBeReused)
{
  CoordinatorInbox inbox(1U);
  auto response =
    std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    ASSERT_EQ(
      inbox.push_cleanup(
        ReleaseReservationCompletion{
          {7U, index + 1U}, response, "", SteadyTime{} +
          std::chrono::milliseconds(index), std::nullopt}),
      CoordinatorCleanupDeposit::kAccepted);
  }
  const ReleaseReservationCompletion lost{
    {7U, 99U}, response, "", SteadyTime{} + 9ms,
    CoordinatorReconciliationCorrelation{ReconciliationKind::kReadback, 2U}};
  expect_deposit(
    inbox.push_cleanup(lost), CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned);
  auto earlier = lost;
  earlier.arrived_at = SteadyTime{} + 8ms;
  expect_deposit(
    inbox.push_cleanup(earlier), CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned);
  const auto first = inbox.try_pop();
  ASSERT_TRUE(first);
  const auto & first_marker = std::get<CoordinatorCleanupEvidenceLossMarker>(*first);
  EXPECT_EQ(first_marker.kind, CoordinatorCleanupEvidenceKind::kReleaseReservation);
  EXPECT_EQ(first_marker.correlation.goal_generation, 7U);
  EXPECT_EQ(first_marker.correlation.operation_generation, 99U);
  ASSERT_TRUE(first_marker.reconciliation);
  EXPECT_EQ(first_marker.reconciliation->kind, ReconciliationKind::kReadback);
  EXPECT_EQ(first_marker.reconciliation->attempt_number, 2U);
  EXPECT_EQ(first_marker.arrived_at, SteadyTime{} + 8ms);

  auto later = lost;
  later.correlation.operation_generation = 100U;
  later.arrived_at = SteadyTime{} + 10ms;
  expect_deposit(
    inbox.push_cleanup(later), CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned);
  const auto second = inbox.try_pop();
  ASSERT_TRUE(second);
  EXPECT_EQ(
    std::get<CoordinatorCleanupEvidenceLossMarker>(*second).correlation.operation_generation,
    100U);
}

TEST(CoordinatorInbox, CleanupLossMarkerRejectsEveryIdentityMismatch)
{
  const auto run = [](CoordinatorCleanupItem first, CoordinatorCleanupItem second) {
    const auto expected_kind = std::holds_alternative<SnapshotCompletion>(first) ?
      CoordinatorCleanupEvidenceKind::kSnapshot :
      CoordinatorCleanupEvidenceKind::kReleaseReservation;
    const auto expected_correlation = std::visit(
      [](const auto & value) {return value.correlation;}, first);
    const auto expected_reconciliation = std::visit(
      [](const auto & value) {return value.reconciliation;}, first);
    const auto expected_arrival = std::visit(
      [](const auto & value) {return value.arrived_at;}, first);
    CoordinatorInbox inbox(1U);
    auto filler =
      std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    for (std::size_t index = 0U;
      index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index)
    {
      ASSERT_EQ(
        inbox.push_cleanup(
          ReleaseReservationCompletion{
            {7U, index + 1U}, filler, "", SteadyTime{}, std::nullopt}),
        CoordinatorCleanupDeposit::kAccepted);
    }
    expect_deposit(
      inbox.push_cleanup(std::move(first)), CoordinatorInboxDepositStatus::kEvidenceLost,
      CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned);
    expect_deposit(
      inbox.push_cleanup(std::move(second)),
      CoordinatorInboxDepositStatus::kEvidenceConflict,
      CoordinatorInboxPersistenceStatus::kUnresolved);
    EXPECT_TRUE(inbox.snapshot().generation_accounting_conflict);
    const auto marker_delivery = inbox.try_pop();
    ASSERT_TRUE(marker_delivery);
    const auto & marker =
      std::get<CoordinatorCleanupEvidenceLossMarker>(*marker_delivery);
    EXPECT_EQ(marker.kind, expected_kind);
    EXPECT_EQ(marker.correlation.goal_generation, expected_correlation.goal_generation);
    EXPECT_EQ(
      marker.correlation.operation_generation,
      expected_correlation.operation_generation);
    EXPECT_EQ(marker.reconciliation.has_value(), expected_reconciliation.has_value());
    if (expected_reconciliation) {
      ASSERT_TRUE(marker.reconciliation);
      EXPECT_EQ(marker.reconciliation->kind, expected_reconciliation->kind);
      EXPECT_EQ(
        marker.reconciliation->attempt_number,
        expected_reconciliation->attempt_number);
    }
    EXPECT_EQ(marker.arrived_at, expected_arrival);
  };
  auto release =
    std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  auto snapshot = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  const ReleaseReservationCompletion baseline{
    {7U, 99U}, release, "", SteadyTime{},
    CoordinatorReconciliationCorrelation{ReconciliationKind::kReadback, 2U}};
  run(
    baseline,
    SnapshotCompletion{
        baseline.correlation, snapshot, "", SteadyTime{}, baseline.reconciliation});
  auto other_goal = baseline;
  other_goal.correlation.goal_generation = 8U;
  run(baseline, other_goal);
  auto other_operation = baseline;
  other_operation.correlation.operation_generation = 100U;
  run(baseline, other_operation);
  auto other_kind = baseline;
  other_kind.reconciliation->kind = ReconciliationKind::kReplayMutation;
  run(baseline, other_kind);
  auto other_attempt = baseline;
  ++other_attempt.reconciliation->attempt_number;
  run(baseline, other_attempt);
  auto absent_reconciliation = baseline;
  absent_reconciliation.reconciliation.reset();
  run(baseline, absent_reconciliation);
  run(absent_reconciliation, baseline);
}

TEST(CoordinatorInbox, GenerationEmptyTracksAcceptedAndMarkerOwnership)
{
  CoordinatorInbox inbox(1U);
  EXPECT_TRUE(inbox.generation_empty(7U));
  expect_deposit(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 7U, {}, SteadyTime{},
        rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME), std::nullopt},
      std::nullopt, SteadyTime{} + 1ms),
    CoordinatorInboxDepositStatus::kAccepted,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  EXPECT_FALSE(inbox.generation_empty(7U));
  ASSERT_TRUE(inbox.try_pop());
  EXPECT_TRUE(inbox.generation_empty(7U));
  EXPECT_FALSE(inbox.generation_empty(0U));
  EXPECT_FALSE(
    inbox.generation_empty(std::numeric_limits<GoalGeneration>::max()));
}

TEST(CoordinatorInbox, TryRequeuePreservesPopOrderAndGenerationAccounting)
{
  CoordinatorInbox inbox(4U);
  auto response_a = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  auto response_b = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  EXPECT_EQ(
    inbox.push(
      SnapshotCompletion{{7U, 1U}, response_a, "", SteadyTime{} + 1ms, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  EXPECT_EQ(
    inbox.push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, 7U, 0U, SteadyTime{} + 2ms, "control"}),
    CoordinatorInboxPushResult::kAccepted);
  EXPECT_EQ(
    inbox.push(
      SnapshotCompletion{{7U, 2U}, response_b, "", SteadyTime{} + 3ms, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);

  auto first = inbox.try_pop();
  ASSERT_TRUE(first);
  ASSERT_TRUE(std::holds_alternative<SnapshotCompletion>(*first));
  auto second = inbox.try_pop();
  ASSERT_TRUE(second);
  ASSERT_TRUE(std::holds_alternative<CoordinatorControlEvent>(*second));
  auto third = inbox.try_pop();
  ASSERT_TRUE(third);
  ASSERT_TRUE(std::holds_alternative<SnapshotCompletion>(*third));

  EXPECT_TRUE(inbox.try_requeue(*third));
  EXPECT_TRUE(inbox.try_requeue(*first));
  EXPECT_EQ(inbox.snapshot().size, 2U);
  EXPECT_FALSE(inbox.generation_empty(7U));

  auto restored_first = inbox.try_pop();
  ASSERT_TRUE(restored_first);
  const auto * restored_a = std::get_if<SnapshotCompletion>(&*restored_first);
  ASSERT_NE(restored_a, nullptr);
  EXPECT_EQ(restored_a->correlation.operation_generation, 1U);
  EXPECT_EQ(restored_a->response, response_a);

  auto restored_second = inbox.try_pop();
  ASSERT_TRUE(restored_second);
  const auto * restored_b = std::get_if<SnapshotCompletion>(&*restored_second);
  ASSERT_NE(restored_b, nullptr);
  EXPECT_EQ(restored_b->correlation.operation_generation, 2U);
  EXPECT_EQ(restored_b->response, response_b);
  EXPECT_FALSE(inbox.try_pop());
  EXPECT_TRUE(inbox.generation_empty(7U));
}

TEST(CoordinatorInbox, DeferredOccupancyAboveCapacityBlocksAcceptedGoalOrdinaryPath)
{
  // Requeue emergency accepts into the deferred lane so deferred_size_ > capacity_ with
  // overflow_latched_ false. The ordinary gate must not fail open via unsigned subtraction wrap.
  CoordinatorInbox inbox(2U);
  auto response = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  auto release =
    std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  EXPECT_EQ(
    inbox.push(
      SnapshotCompletion{{7U, 1U}, response, "", SteadyTime{} + 1ms, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  EXPECT_EQ(
    inbox.push(
      SnapshotCompletion{{7U, 2U}, response, "", SteadyTime{} + 2ms, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  fill_cleanup_emergency(inbox, 7U, release);

  std::vector<CoordinatorInboxDelivery> parked;
  while (auto delivery = inbox.try_pop()) {
    parked.push_back(std::move(*delivery));
  }
  ASSERT_EQ(parked.size(), 2U + CoordinatorInbox::kCleanupEmergencyCapacity);
  for (auto iterator = parked.rbegin(); iterator != parked.rend(); ++iterator) {
    ASSERT_TRUE(inbox.try_requeue(*iterator));
  }
  EXPECT_FALSE(inbox.snapshot().overflow_latched);
  EXPECT_GT(inbox.snapshot().size, 2U);

  SelectionRequest selection;
  selection.object_id = restocker_world_state::ObjectId{17U};
  selection.lane_id = restocker_world_state::LaneId{"lane_01"};
  auto gate = generation_quiescence(7U);
  expect_deposit(
    inbox.push_accepted_goal(
      CoordinatorAcceptedGoal{
        goal_id(), 7U, selection, SteadyTime{} + 10ms,
        rclcpp::Time(std::int64_t{3'000'000'000}, RCL_ROS_TIME), std::nullopt, gate, false},
      std::nullopt, SteadyTime{} + 10ms),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  EXPECT_TRUE(inbox.snapshot().overflow_latched);

  // Deferred drains before emergency handoff / overflow marker.
  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    auto delivery = inbox.try_pop();
    ASSERT_TRUE(delivery);
    EXPECT_TRUE(
      std::holds_alternative<ReleaseReservationCompletion>(*delivery) ||
      std::holds_alternative<SnapshotCompletion>(*delivery));
  }
  auto restored_a = inbox.try_pop();
  ASSERT_TRUE(restored_a);
  ASSERT_NE(std::get_if<SnapshotCompletion>(&*restored_a), nullptr);
  auto restored_b = inbox.try_pop();
  ASSERT_TRUE(restored_b);
  ASSERT_NE(std::get_if<SnapshotCompletion>(&*restored_b), nullptr);

  auto overflow = inbox.try_pop();
  ASSERT_TRUE(overflow);
  EXPECT_NE(std::get_if<CoordinatorOverflowMarker>(&*overflow), nullptr);
  auto accepted = inbox.try_pop();
  ASSERT_TRUE(accepted);
  EXPECT_NE(std::get_if<CoordinatorAcceptedGoal>(&*accepted), nullptr);
  EXPECT_FALSE(inbox.try_pop());
}

}  // namespace
}  // namespace restocker_task_executor
