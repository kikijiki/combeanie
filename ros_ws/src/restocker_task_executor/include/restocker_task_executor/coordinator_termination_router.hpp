// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <functional>
#include <optional>
#include <string_view>

#include "restocker_task_executor/restock_coordinator_inbox.hpp"

namespace restocker_task_executor
{

using CoordinatorTerminationRouterFailureInjector = std::function<bool ()>;

class CoordinatorTerminationRouter;

class CoordinatorTerminationRouteDecision final
{
public:
  CoordinatorTerminationRouteDecision(const CoordinatorTerminationRouteDecision &) = delete;
  CoordinatorTerminationRouteDecision(CoordinatorTerminationRouteDecision && other) noexcept;
  CoordinatorTerminationRouteDecision & operator=(
    const CoordinatorTerminationRouteDecision &) = delete;
  CoordinatorTerminationRouteDecision & operator=(
    CoordinatorTerminationRouteDecision && other) noexcept;
  ~CoordinatorTerminationRouteDecision() = default;

  [[nodiscard]] const GoalTerminationLatchDecision & admission_decision() const noexcept;
  [[nodiscard]] const std::optional<CoordinatorControlEvent> & event() const noexcept;
  [[nodiscard]] bool diagnostic_degraded() const noexcept;

  // Transfers the prebuilt event once while preserving immutable admission and diagnostic
  // evidence. This is the only event extraction path, so downstream inbox handoff never copies a
  // diagnostic string after latching.
  [[nodiscard]] std::optional<CoordinatorControlEvent> take_event() noexcept;

private:
  friend class CoordinatorTerminationRouter;

  explicit CoordinatorTerminationRouteDecision(
    GoalTerminationLatchDecision admission_decision,
    std::optional<CoordinatorControlEvent> event,
    bool diagnostic_degraded) noexcept;

  GoalTerminationLatchDecision admission_decision_;
  std::optional<CoordinatorControlEvent> event_;
  bool diagnostic_degraded_{false};
};

class CoordinatorTerminationRouter final
{
public:
  explicit CoordinatorTerminationRouter(
    GoalAdmissionSlot & admission,
    CoordinatorTerminationRouterFailureInjector failure_injector = {});

  CoordinatorTerminationRouter(const CoordinatorTerminationRouter &) = delete;
  CoordinatorTerminationRouter(CoordinatorTerminationRouter &&) = delete;
  CoordinatorTerminationRouter & operator=(const CoordinatorTerminationRouter &) = delete;
  CoordinatorTerminationRouter & operator=(CoordinatorTerminationRouter &&) = delete;
  ~CoordinatorTerminationRouter() = default;

  [[nodiscard]] CoordinatorTerminationRouteDecision route_cancel(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_drain(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_safe_abort(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_shutdown(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_transform_authority_loss(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_simulation_time_authority_loss(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_protocol_failure(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const;
  [[nodiscard]] CoordinatorTerminationRouteDecision route_task_deadline(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    GoalGeneration deadline_generation, SteadyTime arrived_at,
    OperationGeneration operation_generation, std::string_view detail) const;

  [[nodiscard]] GoalTerminationLatchDecision record_inbox_loss(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    SteadyTime arrived_at) const;

private:
  struct PreparedEvent
  {
    CoordinatorControlEvent event;
    bool diagnostic_degraded{false};
  };

  [[nodiscard]] PreparedEvent prepare_event(
    CoordinatorControlKind kind, GoalGeneration goal_generation,
    SteadyTime arrived_at, OperationGeneration operation_generation,
    std::string_view detail) const noexcept;
  [[nodiscard]] static CoordinatorTerminationRouteDecision finish_route(
    GoalTerminationLatchDecision admission_decision,
    PreparedEvent prepared) noexcept;

  GoalAdmissionSlot & admission_;
  const CoordinatorTerminationRouterFailureInjector failure_injector_;
};

}  // namespace restocker_task_executor
