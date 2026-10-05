// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_async_steady_clock_failure_latch.hpp"

namespace restocker_task_executor
{

bool CoordinatorAsyncSteadyClockFailureLatch::record_failure(
  CoordinatorSteadyClockSample sample)
{
  if (!sample.provider_failed) {
    return false;
  }

  std::lock_guard lock(mutex_);
  if (first_failure_) {
    return false;
  }
  first_failure_.emplace(sample);
  return true;
}

std::optional<CoordinatorSteadyClockSample>
CoordinatorAsyncSteadyClockFailureLatch::snapshot() const
{
  std::lock_guard lock(mutex_);
  return first_failure_;
}

}  // namespace restocker_task_executor
