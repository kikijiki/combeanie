// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "termination_signal_waiter.hpp"

#include <pthread.h>

#include <cerrno>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace restocker_task_executor
{

TerminationSignalWaiter::TerminationSignalWaiter(bool restore_mask_on_destruction)
: restore_mask_on_destruction_(restore_mask_on_destruction)
{
  if (sigemptyset(&signals_) != 0 || sigaddset(&signals_, SIGINT) != 0 ||
    sigaddset(&signals_, SIGTERM) != 0)
  {
    throw std::system_error(errno, std::generic_category(), "could not create termination set");
  }
  const int result = pthread_sigmask(SIG_BLOCK, &signals_, &previous_mask_);
  if (result != 0) {
    throw std::system_error(
            result, std::generic_category(), "could not block termination signals");
  }
  mask_installed_ = true;
}

TerminationSignalWaiter::~TerminationSignalWaiter()
{
  if (mask_installed_ && restore_mask_on_destruction_) {
    (void)pthread_sigmask(SIG_SETMASK, &previous_mask_, nullptr);
  }
}

std::optional<int> TerminationSignalWaiter::wait_for(std::chrono::milliseconds timeout) const
{
  if (timeout < std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("signal wait timeout must not be negative");
  }
  constexpr auto kMillisecondsPerSecond = std::int64_t{1000};
  const auto seconds = timeout.count() / kMillisecondsPerSecond;
  if (seconds > std::numeric_limits<time_t>::max()) {
    throw std::invalid_argument("signal wait timeout exceeds time_t");
  }
  const timespec wait{
    static_cast<time_t>(seconds),
    static_cast<decltype(timespec{}.tv_nsec)>(
      (timeout.count() % kMillisecondsPerSecond) * 1000000)};
  const int result = sigtimedwait(&signals_, nullptr, &wait);
  if (result >= 0) {
    return result;
  }
  if (errno == EAGAIN || errno == EINTR) {
    return std::nullopt;
  }
  throw std::system_error(errno, std::generic_category(), "sigtimedwait failed");
}

}  // namespace restocker_task_executor
