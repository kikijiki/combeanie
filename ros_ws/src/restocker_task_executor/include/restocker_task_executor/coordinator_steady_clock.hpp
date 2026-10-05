// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <mutex>

#include "restocker_task_executor/coordinator_operation_types.hpp"

namespace restocker_task_executor
{

struct CoordinatorSteadyClockSample
{
  SteadyTime time{};
  bool provider_failed{false};
  bool newly_failed{false};

  [[nodiscard]] bool operator==(const CoordinatorSteadyClockSample &) const noexcept = default;
};

// Node-wide injected steady clock. If the provider fails, the domain freezes permanently at one
// production-clock fallback. Callers use provider_failed to stop normal orchestration; the frozen
// value only lets fail-closed cleanup retain a valid timestamp.
class CoordinatorSteadyClock final
{
public:
  explicit CoordinatorSteadyClock(CoordinatorSteadyNow provider);

  CoordinatorSteadyClock(const CoordinatorSteadyClock &) = delete;
  CoordinatorSteadyClock & operator=(const CoordinatorSteadyClock &) = delete;

  [[nodiscard]] CoordinatorSteadyClockSample sample() noexcept;
  [[nodiscard]] bool provider_failed() const noexcept;

private:
  mutable std::mutex provider_mutex_;
  mutable std::mutex mutex_;
  CoordinatorSteadyNow provider_;
  std::atomic<bool> provider_failed_{false};
  SteadyTime fallback_{};
};

}  // namespace restocker_task_executor
