// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_termination_router.hpp"

#include <cassert>
#include <type_traits>
#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool route_applied(GoalTerminationLatchStatus status) noexcept
{
  return status == GoalTerminationLatchStatus::kLatched ||
         status == GoalTerminationLatchStatus::kAlreadyLatched;
}

static_assert(std::is_nothrow_move_constructible_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_move_assignable_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_destructible_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorControlEvent>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorControlEvent>);
static_assert(std::is_nothrow_destructible_v<CoordinatorControlEvent>);
static_assert(
  std::is_nothrow_move_constructible_v<std::optional<CoordinatorControlEvent>>);
static_assert(std::is_nothrow_move_assignable_v<std::optional<CoordinatorControlEvent>>);
static_assert(std::is_nothrow_destructible_v<std::optional<CoordinatorControlEvent>>);
static_assert(
  noexcept(
    std::declval<std::optional<CoordinatorControlEvent> &>().emplace(
      std::declval<CoordinatorControlEvent &&>())));
static_assert(std::is_nothrow_move_constructible_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorTerminationRouteDecision>);
static_assert(std::is_nothrow_destructible_v<CoordinatorTerminationRouteDecision>);
static_assert(
  noexcept(
    std::declval<std::optional<CoordinatorControlEvent> &>().reset()));

}  // namespace

CoordinatorTerminationRouteDecision::CoordinatorTerminationRouteDecision(
  GoalTerminationLatchDecision admission_decision,
  std::optional<CoordinatorControlEvent> event,
  bool diagnostic_degraded) noexcept
: admission_decision_(std::move(admission_decision)),
  event_(std::move(event)),
  diagnostic_degraded_(diagnostic_degraded)
{
  assert(route_applied(admission_decision_.status()) == event_.has_value());
}

CoordinatorTerminationRouteDecision::CoordinatorTerminationRouteDecision(
  CoordinatorTerminationRouteDecision && other) noexcept
: admission_decision_(std::move(other.admission_decision_)),
  event_(std::move(other.event_)),
  diagnostic_degraded_(std::exchange(other.diagnostic_degraded_, false))
{
  other.event_.reset();
}

CoordinatorTerminationRouteDecision & CoordinatorTerminationRouteDecision::operator=(
  CoordinatorTerminationRouteDecision && other) noexcept
{
  if (this == &other) {
    return *this;
  }
  admission_decision_ = std::move(other.admission_decision_);
  event_ = std::move(other.event_);
  diagnostic_degraded_ = std::exchange(other.diagnostic_degraded_, false);
  other.event_.reset();
  return *this;
}

const GoalTerminationLatchDecision &
CoordinatorTerminationRouteDecision::admission_decision() const noexcept
{
  return admission_decision_;
}

const std::optional<CoordinatorControlEvent> &
CoordinatorTerminationRouteDecision::event() const noexcept
{
  return event_;
}

bool CoordinatorTerminationRouteDecision::diagnostic_degraded() const noexcept
{
  return diagnostic_degraded_;
}

std::optional<CoordinatorControlEvent>
CoordinatorTerminationRouteDecision::take_event() noexcept
{
  std::optional<CoordinatorControlEvent> result{std::move(event_)};
  event_.reset();
  return result;
}

CoordinatorTerminationRouter::CoordinatorTerminationRouter(
  GoalAdmissionSlot & admission,
  CoordinatorTerminationRouterFailureInjector failure_injector)
: admission_(admission), failure_injector_(std::move(failure_injector))
{
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_cancel(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kCancelRequested, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_cancel(goal_id, goal_generation, arrived_at), std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_drain(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kDrainRequested, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_drain(goal_id, goal_generation, arrived_at), std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_safe_abort(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kSafeAbortRequested, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_safe_abort(goal_id, goal_generation, arrived_at), std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_shutdown(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kShutdown, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_shutdown(goal_id, goal_generation, arrived_at), std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_transform_authority_loss(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kAuthorityFaultSafeAbortRequested, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_authority_loss(goal_id, goal_generation, arrived_at),
    std::move(prepared));
}

CoordinatorTerminationRouteDecision
CoordinatorTerminationRouter::route_simulation_time_authority_loss(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kShutdown, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_authority_loss(goal_id, goal_generation, arrived_at),
    std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_protocol_failure(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kShutdown, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_protocol_failure(goal_id, goal_generation, arrived_at),
    std::move(prepared));
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::route_task_deadline(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  GoalGeneration deadline_generation, SteadyTime arrived_at,
  OperationGeneration operation_generation, std::string_view detail) const
{
  auto prepared = prepare_event(
    CoordinatorControlKind::kTaskDeadline, goal_generation, arrived_at,
    operation_generation, detail);
  return finish_route(
    admission_.request_task_deadline(
      goal_id, goal_generation, deadline_generation, arrived_at),
    std::move(prepared));
}

GoalTerminationLatchDecision CoordinatorTerminationRouter::record_inbox_loss(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  SteadyTime arrived_at) const
{
  return admission_.request_inbox_loss(goal_id, goal_generation, arrived_at);
}

CoordinatorTerminationRouter::PreparedEvent CoordinatorTerminationRouter::prepare_event(
  CoordinatorControlKind kind, GoalGeneration goal_generation,
  SteadyTime arrived_at, OperationGeneration operation_generation,
  std::string_view detail) const noexcept
{
  PreparedEvent prepared{
    CoordinatorControlEvent{kind, goal_generation, operation_generation, arrived_at, {}}, false};
  try {
    const bool inject_failure = failure_injector_ && failure_injector_();
    if (inject_failure) {
      prepared.diagnostic_degraded = true;
    } else if (!detail.empty()) {
      prepared.event.detail.assign(detail.data(), detail.size());
    }
  } catch (...) {
    prepared.event.detail.clear();
    prepared.diagnostic_degraded = true;
  }
  return prepared;
}

CoordinatorTerminationRouteDecision CoordinatorTerminationRouter::finish_route(
  GoalTerminationLatchDecision admission_decision,
  PreparedEvent prepared) noexcept
{
  std::optional<CoordinatorControlEvent> event;
  if (route_applied(admission_decision.status())) {
    event.emplace(std::move(prepared.event));
  }
  return CoordinatorTerminationRouteDecision{
    std::move(admission_decision), std::move(event), prepared.diagnostic_degraded};
}

}  // namespace restocker_task_executor
