// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_process_control.hpp"

namespace restocker_task_executor
{

CoordinatorProcessDecision CoordinatorTerminationPolicy::observe_signal() noexcept
{
  if (latched_exit_) {
    return {false, true, latched_exit_};
  }
  if (signal_observed_) {
    latched_exit_ = CoordinatorProcessExitCode::kForcedBySecondSignal;
    return {false, true, latched_exit_};
  }
  signal_observed_ = true;
  return {true, false, std::nullopt};
}

CoordinatorProcessDecision CoordinatorTerminationPolicy::observe(
  const CoordinatorShutdownSnapshot & shutdown, ExecutorRunState executor,
  SteadyTime now) noexcept
{
  if (latched_exit_) {
    return {false, true, latched_exit_};
  }
  if (shutdown.status == CoordinatorShutdownStatus::kClean) {
    latched_exit_ = CoordinatorProcessExitCode::kClean;
    return {false, true, latched_exit_};
  }
  if (shutdown.status == CoordinatorShutdownStatus::kTimedOut ||
    (shutdown.deadline && now >= *shutdown.deadline))
  {
    latched_exit_ = CoordinatorProcessExitCode::kDrainTimedOut;
    return {false, true, latched_exit_};
  }
  if (executor == ExecutorRunState::kStoppedUnexpectedly ||
    executor == ExecutorRunState::kFailed)
  {
    latched_exit_ = CoordinatorProcessExitCode::kExecutorFailure;
    return {false, true, latched_exit_};
  }
  return {};
}

}  // namespace restocker_task_executor
