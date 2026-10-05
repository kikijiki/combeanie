// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <mutex>
#include <optional>

#include "restocker_task_executor/coordinator_steady_clock.hpp"

namespace restocker_task_executor
{

// Retains the first failed sample observed by an asynchronous clock consumer. The component owns
// only scalar observation state and may therefore be strongly captured by callbacks that outlive
// their node, driver, or goal context.
class CoordinatorAsyncSteadyClockFailureLatch final
{
public:
  CoordinatorAsyncSteadyClockFailureLatch() = default;

  CoordinatorAsyncSteadyClockFailureLatch(const CoordinatorAsyncSteadyClockFailureLatch &) =
  delete;
  CoordinatorAsyncSteadyClockFailureLatch & operator=(
    const CoordinatorAsyncSteadyClockFailureLatch &) = delete;

  // Returns true only when this call installs the first valid failed sample. A non-failure sample
  // is not evidence and cannot occupy the latch.
  [[nodiscard]] bool record_failure(CoordinatorSteadyClockSample sample);

  [[nodiscard]] std::optional<CoordinatorSteadyClockSample> snapshot() const;

private:
  mutable std::mutex mutex_;
  std::optional<CoordinatorSteadyClockSample> first_failure_;
};

}  // namespace restocker_task_executor
