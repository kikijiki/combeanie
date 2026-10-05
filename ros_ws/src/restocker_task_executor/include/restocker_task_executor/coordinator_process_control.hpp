// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace restocker_task_executor
{

enum class CoordinatorShutdownStatus : std::uint8_t
{
  kRunning,
  kDraining,
  kClean,
  kTimedOut,
};

enum class CoordinatorProcessExitCode : int
{
  kClean = 0,
  kStartupFailure = 1,
  kDrainTimedOut = 2,
  kForcedBySecondSignal = 3,
  kExecutorFailure = 4,
};

enum class ExecutorRunState : std::uint8_t
{
  kRunning,
  kStoppedByOwner,
  kStoppedUnexpectedly,
  kFailed,
};

struct CoordinatorShutdownSnapshot
{
  CoordinatorShutdownStatus status{CoordinatorShutdownStatus::kRunning};
  std::optional<std::chrono::steady_clock::time_point> deadline;
  std::uint64_t goal_generation{0U};
  bool reservation_capability_may_remain{false};
};

struct CoordinatorProcessDecision
{
  bool request_node_drain{false};
  bool stop_executor{false};
  std::optional<CoordinatorProcessExitCode> exit_code;
};

// Pure steady-time policy shared by the executable and its unit tests. It owns only event
// precedence and creates no second shutdown deadline.
class CoordinatorTerminationPolicy
{
public:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  [[nodiscard]] CoordinatorProcessDecision observe_signal() noexcept;
  [[nodiscard]] CoordinatorProcessDecision observe(
    const CoordinatorShutdownSnapshot & shutdown, ExecutorRunState executor,
    SteadyTime now) noexcept;

private:
  bool signal_observed_{false};
  std::optional<CoordinatorProcessExitCode> latched_exit_;
};

[[nodiscard]] constexpr int exit_status(CoordinatorProcessExitCode code) noexcept
{
  return static_cast<int>(code);
}

}  // namespace restocker_task_executor
