// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>

#include "restocker_task_executor/restock_task_machine.hpp"

namespace restocker_task_executor
{

using CoordinatorGoalId = std::array<std::uint8_t, 16>;
using GoalGeneration = std::uint64_t;
using OperationGeneration = std::uint64_t;
using ExecutionAttemptGeneration = std::uint64_t;
using SteadyTime = std::chrono::steady_clock::time_point;
using CoordinatorSteadyNow = std::function<SteadyTime ()>;

inline constexpr GoalGeneration kReservedCoordinatorGoalGeneration =
  std::numeric_limits<GoalGeneration>::max();

[[nodiscard]] constexpr bool admissible_goal_generation(GoalGeneration generation) noexcept
{
  return generation > 0U && generation < kReservedCoordinatorGoalGeneration;
}

enum class OperationEffect : std::uint8_t
{
  kReadOnly,
  kCancelableMotion,
  kIdempotentMutation,
};

enum class PendingOperationPhase : std::uint8_t
{
  kAwaitingRequestBinding,
  kAwaitingCompletion,
  kCancelAndVerify,
  kAwaitingStopProof,
  kMotionLivenessUnknown,
  kReconciliationRequired,
  kReconciliationExhausted,
};

struct OperationTicket
{
  GoalGeneration goal_generation{0};
  OperationGeneration operation_generation{0};
  RestockTaskCommand command{RestockTaskCommand::kNone};
  OperationEffect effect{OperationEffect::kReadOnly};
  std::string operation_id;
  SteadyTime deadline{};

  [[nodiscard]] bool operator==(const OperationTicket &) const noexcept = default;
};

}  // namespace restocker_task_executor
