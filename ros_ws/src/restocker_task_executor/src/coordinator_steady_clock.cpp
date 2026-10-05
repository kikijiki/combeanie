// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_steady_clock.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{

CoordinatorSteadyClock::CoordinatorSteadyClock(CoordinatorSteadyNow provider)
: provider_(std::move(provider))
{
  if (!provider_) {
    throw std::invalid_argument("coordinator steady-clock provider must not be empty");
  }
}

CoordinatorSteadyClockSample CoordinatorSteadyClock::sample() noexcept
{
  if (provider_failed_.load(std::memory_order_acquire)) {
    std::lock_guard lock(mutex_);
    return {fallback_, true, false};
  }
  // Serialize provider entry. The second sticky check makes failure publication a fence:
  // after provider_failed_ becomes true, no thread can begin another provider invocation.
  std::lock_guard provider_lock(provider_mutex_);
  if (provider_failed_.load(std::memory_order_acquire)) {
    std::lock_guard lock(mutex_);
    return {fallback_, true, false};
  }
  try {
    const auto sampled = provider_();
    if (sampled == SteadyTime::max()) {
      throw std::runtime_error("coordinator steady-clock provider returned the invalid sentinel");
    }
    std::lock_guard lock(mutex_);
    if (provider_failed_.load(std::memory_order_relaxed)) {
      return {fallback_, true, false};
    }
    return {sampled, false, false};
  } catch (...) {
    std::lock_guard lock(mutex_);
    if (provider_failed_.load(std::memory_order_relaxed)) {
      return {fallback_, true, false};
    }
    // This is the sole fallback read. Once selected, the clock never resumes or mixes the injected
    // and production domains. The node treats provider_failed as a protocol failure and permits
    // this frozen time only for bounded fail-closed cleanup decisions.
    fallback_ = std::chrono::steady_clock::now();
    provider_failed_.store(true, std::memory_order_release);
    return {fallback_, true, true};
  }
}

bool CoordinatorSteadyClock::provider_failed() const noexcept
{
  return provider_failed_.load(std::memory_order_acquire);
}

}  // namespace restocker_task_executor
