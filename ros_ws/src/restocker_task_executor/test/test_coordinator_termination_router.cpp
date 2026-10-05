// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_termination_router.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId value{};
  for (std::size_t index = 0U; index < value.size(); ++index) {
    value[index] = static_cast<std::uint8_t>(seed + index);
  }
  return value;
}

struct ReservedGoal
{
  ReservedGoal()
  {
    admission.update_readiness(true);
    const auto receipt = admission.reserve(id);
    EXPECT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
    generation = receipt.generation;
  }

  GoalAdmissionSlot admission;
  CoordinatorGoalId id{goal_id()};
  GoalGeneration generation{0U};
};

enum class RouteCase : std::uint8_t
{
  kCancel,
  kDrain,
  kSafeAbort,
  kShutdown,
  kTransformAuthorityLoss,
  kSimulationAuthorityLoss,
  kProtocolFailure,
  kTaskDeadline,
};

struct RouteExpectation
{
  RouteCase route;
  GoalTerminationKind termination_kind;
  CoordinatorControlKind control_kind;
  std::string_view detail;
};

constexpr std::array kRouteExpectations{
  RouteExpectation{
    RouteCase::kCancel, GoalTerminationKind::kUserCancel,
    CoordinatorControlKind::kCancelRequested, "client cancellation"},
  RouteExpectation{
    RouteCase::kDrain, GoalTerminationKind::kCoordinatorDrain,
    CoordinatorControlKind::kDrainRequested, "coordinator drain"},
  RouteExpectation{
    RouteCase::kSafeAbort, GoalTerminationKind::kSafeAbort,
    CoordinatorControlKind::kSafeAbortRequested, "driver requested safe abort"},
  RouteExpectation{
    RouteCase::kShutdown, GoalTerminationKind::kShutdown,
    CoordinatorControlKind::kShutdown, "internal shutdown"},
  RouteExpectation{
    RouteCase::kTransformAuthorityLoss, GoalTerminationKind::kAuthorityLoss,
    CoordinatorControlKind::kAuthorityFaultSafeAbortRequested, "transform authority lost"},
  RouteExpectation{
    RouteCase::kSimulationAuthorityLoss, GoalTerminationKind::kAuthorityLoss,
    CoordinatorControlKind::kShutdown, "simulation time regressed"},
  RouteExpectation{
    RouteCase::kProtocolFailure, GoalTerminationKind::kProtocolFailure,
    CoordinatorControlKind::kShutdown, "driver output is invalid"},
  RouteExpectation{
    RouteCase::kTaskDeadline, GoalTerminationKind::kTaskDeadline,
    CoordinatorControlKind::kTaskDeadline, "whole-task deadline reached"},
};

[[nodiscard]] CoordinatorTerminationRouteDecision invoke_route(
  RouteCase route, const CoordinatorTerminationRouter & router,
  const CoordinatorGoalId & id, GoalGeneration generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail)
{
  switch (route) {
    case RouteCase::kCancel:
      return router.route_cancel(id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kDrain:
      return router.route_drain(id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kSafeAbort:
      return router.route_safe_abort(id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kShutdown:
      return router.route_shutdown(id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kTransformAuthorityLoss:
      return router.route_transform_authority_loss(
        id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kSimulationAuthorityLoss:
      return router.route_simulation_time_authority_loss(
        id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kProtocolFailure:
      return router.route_protocol_failure(
        id, generation, arrived_at, operation_generation, detail);
    case RouteCase::kTaskDeadline:
      return router.route_task_deadline(
        id, generation, generation, arrived_at, operation_generation, detail);
  }
  throw std::logic_error("unknown route case");
}

void expect_closed(const CoordinatorTerminationRouteDecision & decision)
{
  EXPECT_EQ(
    decision.admission_decision().status(), GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_FALSE(decision.admission_decision().record());
  EXPECT_FALSE(decision.event());
  EXPECT_FALSE(decision.diagnostic_degraded());
}

static_assert(!std::is_default_constructible_v<CoordinatorTerminationRouteDecision>);
static_assert(!std::is_copy_constructible_v<CoordinatorTerminationRouteDecision>);
static_assert(!std::is_copy_assignable_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_destructible_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorControlEvent>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorControlEvent>);
static_assert(
  std::is_nothrow_move_constructible_v<std::optional<CoordinatorControlEvent>>);
static_assert(std::is_nothrow_move_assignable_v<std::optional<CoordinatorControlEvent>>);
static_assert(
  noexcept(
    std::declval<std::optional<CoordinatorControlEvent> &>().emplace(
      std::declval<CoordinatorControlEvent &&>())));
static_assert(
  noexcept(
    std::declval<const CoordinatorTerminationRouteDecision &>().admission_decision()));
static_assert(
  noexcept(
    std::declval<const CoordinatorTerminationRouteDecision &>().event()));
static_assert(
  noexcept(
    std::declval<const CoordinatorTerminationRouteDecision &>().diagnostic_degraded()));
static_assert(noexcept(std::declval<CoordinatorTerminationRouteDecision &>().take_event()));

TEST(CoordinatorTerminationRouter, EveryNamedRouteFixesExactAdmissionAndControlKinds)
{
  for (std::size_t index = 0U; index < kRouteExpectations.size(); ++index) {
    const auto & expected = kRouteExpectations[index];
    SCOPED_TRACE(index);
    ReservedGoal reserved;
    CoordinatorTerminationRouter router{reserved.admission};
    const auto arrived_at = SteadyTime{} + std::chrono::milliseconds(20 + index);
    const auto operation_generation = static_cast<OperationGeneration>(40U + index);

    auto decision = invoke_route(
      expected.route, router, reserved.id, reserved.generation,
      arrived_at, operation_generation, expected.detail);

    ASSERT_EQ(
      decision.admission_decision().status(), GoalTerminationLatchStatus::kLatched);
    const auto record = decision.admission_decision().record();
    ASSERT_TRUE(record);
    EXPECT_EQ(record->goal_id, reserved.id);
    EXPECT_EQ(record->goal_generation, reserved.generation);
    EXPECT_EQ(record->kind, expected.termination_kind);
    EXPECT_EQ(record->cancellation_reason, cancellation_reason_for(expected.termination_kind));
    EXPECT_EQ(record->arrived_at, arrived_at);
    if (expected.termination_kind == GoalTerminationKind::kTaskDeadline) {
      ASSERT_TRUE(record->source_generation);
      EXPECT_EQ(record->source_generation->source, GoalTerminationSource::kTaskDeadline);
      EXPECT_EQ(record->source_generation->generation, reserved.generation);
    } else {
      EXPECT_FALSE(record->source_generation);
    }

    ASSERT_TRUE(decision.event());
    EXPECT_EQ(decision.event()->kind, expected.control_kind);
    EXPECT_EQ(decision.event()->goal_generation, reserved.generation);
    EXPECT_EQ(decision.event()->operation_generation, operation_generation);
    EXPECT_EQ(decision.event()->arrived_at, arrived_at);
    EXPECT_EQ(decision.event()->detail, expected.detail);
    EXPECT_FALSE(decision.diagnostic_degraded());
  }
}

TEST(CoordinatorTerminationRouter, RepeatedRouteReturnsCurrentEventAndImmutableFirstRecord)
{
  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  const auto first_at = SteadyTime{} + 31ms;
  const auto repeated_at = SteadyTime{} + 47ms;
  auto first = router.route_cancel(
    reserved.id, reserved.generation, first_at, 5U, "cancel first");
  const auto first_record = first.admission_decision().record();
  ASSERT_TRUE(first_record);

  auto repeated = router.route_drain(
    reserved.id, reserved.generation, repeated_at, 9U, "drain later");
  EXPECT_EQ(
    repeated.admission_decision().status(), GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(repeated.admission_decision().record(), first_record);
  EXPECT_EQ(first_record->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(first_record->arrived_at, first_at);
  ASSERT_TRUE(repeated.event());
  EXPECT_EQ(repeated.event()->kind, CoordinatorControlKind::kDrainRequested);
  EXPECT_EQ(repeated.event()->operation_generation, 9U);
  EXPECT_EQ(repeated.event()->arrived_at, repeated_at);
  EXPECT_EQ(repeated.event()->detail, "drain later");
}

TEST(CoordinatorTerminationRouter, FailureSeamPreservesFallbackForFirstAndRepeatedLatch)
{
  ReservedGoal reserved;
  std::size_t calls = 0U;
  CoordinatorTerminationRouter router{
    reserved.admission,
    [&calls]() -> bool {
      ++calls;
      if (calls == 1U) {
        return true;
      }
      throw std::runtime_error("injected diagnostic construction failure");
    }};

  const auto first_at = SteadyTime{} + 53ms;
  auto first = router.route_transform_authority_loss(
    reserved.id, reserved.generation, first_at, 13U, "unavailable detail");
  ASSERT_EQ(first.admission_decision().status(), GoalTerminationLatchStatus::kLatched);
  const auto first_record = first.admission_decision().record();
  ASSERT_TRUE(first_record);
  EXPECT_EQ(first_record->kind, GoalTerminationKind::kAuthorityLoss);
  EXPECT_EQ(first_record->arrived_at, first_at);
  ASSERT_TRUE(first.event());
  EXPECT_EQ(first.event()->kind, CoordinatorControlKind::kAuthorityFaultSafeAbortRequested);
  EXPECT_EQ(first.event()->goal_generation, reserved.generation);
  EXPECT_EQ(first.event()->operation_generation, 13U);
  EXPECT_EQ(first.event()->arrived_at, first_at);
  EXPECT_TRUE(first.event()->detail.empty());
  EXPECT_TRUE(first.diagnostic_degraded());

  const auto repeated_at = SteadyTime{} + 59ms;
  auto repeated = router.route_protocol_failure(
    reserved.id, reserved.generation, repeated_at, 17U, "also unavailable");
  EXPECT_EQ(
    repeated.admission_decision().status(), GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(repeated.admission_decision().record(), first_record);
  ASSERT_TRUE(repeated.event());
  EXPECT_EQ(repeated.event()->kind, CoordinatorControlKind::kShutdown);
  EXPECT_EQ(repeated.event()->operation_generation, 17U);
  EXPECT_EQ(repeated.event()->arrived_at, repeated_at);
  EXPECT_TRUE(repeated.event()->detail.empty());
  EXPECT_TRUE(repeated.diagnostic_degraded());
  EXPECT_EQ(calls, 2U);
}

TEST(CoordinatorTerminationRouter, FalseFailureInjectionRetainsPreparedDetail)
{
  ReservedGoal reserved;
  std::size_t calls = 0U;
  CoordinatorTerminationRouter router{
    reserved.admission, [&calls]() {
      ++calls;
      return false;
    }};
  const std::string detail(512U, 'd');
  auto decision = router.route_safe_abort(
    reserved.id, reserved.generation, SteadyTime{}, 19U, detail);

  EXPECT_EQ(calls, 1U);
  EXPECT_FALSE(decision.diagnostic_degraded());
  ASSERT_TRUE(decision.event());
  EXPECT_EQ(decision.event()->detail, detail);
  ASSERT_TRUE(decision.admission_decision().record());
  EXPECT_EQ(decision.admission_decision().record()->arrived_at, SteadyTime{});
}

TEST(CoordinatorTerminationRouter, EmptyDefaultStringViewUsesThePrebuiltFallbackWithoutDegrading)
{
  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  auto decision = router.route_drain(
    reserved.id, reserved.generation, SteadyTime{} + 1ms, 3U, std::string_view{});

  EXPECT_EQ(decision.admission_decision().status(), GoalTerminationLatchStatus::kLatched);
  ASSERT_TRUE(decision.event());
  EXPECT_TRUE(decision.event()->detail.empty());
  EXPECT_FALSE(decision.diagnostic_degraded());
}

TEST(CoordinatorTerminationRouter, InvalidAdmissionStatusesNeverReturnAnEvent)
{
  const auto expect_rejected = [](const CoordinatorTerminationRouteDecision & decision,
    GoalTerminationLatchStatus expected_status, bool expected_degraded = false)
  {
    EXPECT_EQ(decision.admission_decision().status(), expected_status);
    EXPECT_FALSE(decision.event());
    EXPECT_EQ(decision.diagnostic_degraded(), expected_degraded);
  };

  GoalAdmissionSlot inactive;
  CoordinatorTerminationRouter inactive_router{inactive};
  auto inactive_result = inactive_router.route_cancel(
    goal_id(), 1U, SteadyTime{} + 1ms, 1U, "inactive");
  expect_rejected(inactive_result, GoalTerminationLatchStatus::kInactive);

  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  auto invalid_goal = router.route_cancel(
    CoordinatorGoalId{}, reserved.generation, SteadyTime{} + 2ms, 2U, "invalid goal");
  expect_rejected(invalid_goal, GoalTerminationLatchStatus::kInvalidArgument);

  auto invalid_generation = router.route_drain(
    reserved.id, 0U, SteadyTime{} + 3ms, 3U, "invalid generation");
  expect_rejected(invalid_generation, GoalTerminationLatchStatus::kInvalidArgument);

  auto invalid_time = router.route_safe_abort(
    reserved.id, reserved.generation, SteadyTime::max(), 4U, "invalid time");
  expect_rejected(invalid_time, GoalTerminationLatchStatus::kInvalidArgument);

  auto mismatch = router.route_shutdown(
    goal_id(31U), reserved.generation, SteadyTime{} + 5ms, 5U, "foreign goal");
  expect_rejected(mismatch, GoalTerminationLatchStatus::kGoalMismatch);

  auto invalid_deadline = router.route_task_deadline(
    reserved.id, reserved.generation, 0U, SteadyTime{} + 6ms, 6U, "invalid deadline");
  expect_rejected(invalid_deadline, GoalTerminationLatchStatus::kInvalidArgument);

  auto mismatched_deadline = router.route_task_deadline(
    reserved.id, reserved.generation, reserved.generation + 1U,
    SteadyTime{} + 6ms, 6U, "mismatched deadline");
  expect_rejected(mismatched_deadline, GoalTerminationLatchStatus::kInvalidArgument);

  std::size_t calls = 0U;
  CoordinatorTerminationRouter degraded_router{
    reserved.admission, [&calls]() {
      ++calls;
      return true;
    }};
  auto degraded_mismatch = degraded_router.route_protocol_failure(
    goal_id(63U), reserved.generation, SteadyTime{} + 7ms, 7U, "discarded detail");
  expect_rejected(
    degraded_mismatch, GoalTerminationLatchStatus::kGoalMismatch, true);
  EXPECT_EQ(calls, 1U);
}

TEST(CoordinatorTerminationRouter, GoalMismatchCarriesEvidenceButStillReturnsNoEvent)
{
  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  auto first = router.route_cancel(
    reserved.id, reserved.generation, SteadyTime{} + 11ms, 1U, "first");
  const auto record = first.admission_decision().record();
  ASSERT_TRUE(record);

  auto mismatch = router.route_drain(
    goal_id(91U), reserved.generation, SteadyTime{} + 12ms, 2U, "foreign");
  EXPECT_EQ(
    mismatch.admission_decision().status(), GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(mismatch.admission_decision().record(), record);
  EXPECT_FALSE(mismatch.event());
}

TEST(CoordinatorTerminationRouter, ProtocolFailureContextsUseOneClosedMapping)
{
  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  auto invalid_output = router.route_protocol_failure(
    reserved.id, reserved.generation, SteadyTime{} + 21ms, 23U,
    "driver output is invalid");
  const auto first_record = invalid_output.admission_decision().record();
  ASSERT_TRUE(first_record);
  EXPECT_EQ(first_record->kind, GoalTerminationKind::kProtocolFailure);
  ASSERT_TRUE(invalid_output.event());
  EXPECT_EQ(invalid_output.event()->kind, CoordinatorControlKind::kShutdown);

  auto feedback_failure = router.route_protocol_failure(
    reserved.id, reserved.generation, SteadyTime{} + 22ms, 29U,
    "action feedback publication failed");
  EXPECT_EQ(
    feedback_failure.admission_decision().status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(feedback_failure.admission_decision().record(), first_record);
  ASSERT_TRUE(feedback_failure.event());
  EXPECT_EQ(feedback_failure.event()->kind, CoordinatorControlKind::kShutdown);
  EXPECT_EQ(feedback_failure.event()->arrived_at, SteadyTime{} + 22ms);
  EXPECT_EQ(feedback_failure.event()->detail, "action feedback publication failed");
}

TEST(CoordinatorTerminationRouter, InboxLossLatchesWithoutConstructingOrInjectingAnEvent)
{
  ReservedGoal reserved;
  std::size_t injection_calls = 0U;
  CoordinatorTerminationRouter router{
    reserved.admission, [&injection_calls]() {
      ++injection_calls;
      return true;
    }};
  const auto first_at = SteadyTime{} + 71ms;
  auto first = router.record_inbox_loss(reserved.id, reserved.generation, first_at);
  ASSERT_EQ(first.status(), GoalTerminationLatchStatus::kLatched);
  const auto record = first.record();
  ASSERT_TRUE(record);
  EXPECT_EQ(record->kind, GoalTerminationKind::kInboxOverflow);
  EXPECT_EQ(record->cancellation_reason, MotionCancellationReason::kInboxOverflow);
  EXPECT_EQ(record->arrived_at, first_at);
  EXPECT_FALSE(record->source_generation);

  auto repeated = router.record_inbox_loss(
    reserved.id, reserved.generation, SteadyTime{} + 72ms);
  EXPECT_EQ(repeated.status(), GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(repeated.record(), record);
  EXPECT_EQ(injection_calls, 0U);

  auto mismatch = router.record_inbox_loss(
    goal_id(111U), reserved.generation, SteadyTime{} + 73ms);
  EXPECT_EQ(mismatch.status(), GoalTerminationLatchStatus::kGoalMismatch);
  EXPECT_EQ(mismatch.record(), record);

  auto invalid = router.record_inbox_loss(
    reserved.id, reserved.generation, SteadyTime::max());
  EXPECT_EQ(invalid.status(), GoalTerminationLatchStatus::kInvalidArgument);
  EXPECT_FALSE(invalid.record());
}

TEST(CoordinatorTerminationRouter, MoveClosesSourceAndExtractionConsumesOnlyEvent)
{
  ReservedGoal reserved;
  CoordinatorTerminationRouter router{reserved.admission};
  auto source = router.route_drain(
    reserved.id, reserved.generation, SteadyTime{} + 81ms, 37U, "move me");
  const auto record = source.admission_decision().record();
  ASSERT_TRUE(record);

  CoordinatorTerminationRouteDecision moved{std::move(source)};
  expect_closed(source);
  EXPECT_EQ(moved.admission_decision().record(), record);
  ASSERT_TRUE(moved.event());
  EXPECT_EQ(moved.event()->detail, "move me");

  auto extracted = moved.take_event();
  ASSERT_TRUE(extracted);
  EXPECT_EQ(extracted->kind, CoordinatorControlKind::kDrainRequested);
  EXPECT_EQ(extracted->goal_generation, reserved.generation);
  EXPECT_EQ(extracted->operation_generation, 37U);
  EXPECT_EQ(extracted->arrived_at, SteadyTime{} + 81ms);
  EXPECT_EQ(extracted->detail, "move me");
  EXPECT_EQ(
    moved.admission_decision().status(), GoalTerminationLatchStatus::kLatched);
  EXPECT_EQ(moved.admission_decision().record(), record);
  EXPECT_FALSE(moved.event());
  EXPECT_FALSE(moved.diagnostic_degraded());
  EXPECT_FALSE(moved.take_event());

  auto assignment_source = router.route_safe_abort(
    reserved.id, reserved.generation, SteadyTime{} + 82ms, 41U, "assignment source");
  auto assignment_destination = router.route_shutdown(
    reserved.id, reserved.generation, SteadyTime{} + 83ms, 43U, "discarded destination");
  assignment_destination = std::move(assignment_source);
  expect_closed(assignment_source);
  ASSERT_TRUE(assignment_destination.event());
  EXPECT_EQ(assignment_destination.event()->kind, CoordinatorControlKind::kSafeAbortRequested);
  EXPECT_EQ(assignment_destination.event()->detail, "assignment source");

  const auto assignment_record = assignment_destination.admission_decision().record();
  const auto assignment_event = *assignment_destination.event();
  auto * self = &assignment_destination;
  assignment_destination = std::move(*self);
  EXPECT_EQ(assignment_destination.admission_decision().record(), assignment_record);
  ASSERT_TRUE(assignment_destination.event());
  EXPECT_EQ(assignment_destination.event()->kind, assignment_event.kind);
  EXPECT_EQ(assignment_destination.event()->goal_generation, assignment_event.goal_generation);
  EXPECT_EQ(
    assignment_destination.event()->operation_generation,
    assignment_event.operation_generation);
  EXPECT_EQ(assignment_destination.event()->arrived_at, assignment_event.arrived_at);
  EXPECT_EQ(assignment_destination.event()->detail, assignment_event.detail);
}

}  // namespace
}  // namespace restocker_task_executor
