// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <signal.h>

#include <chrono>
#include <optional>

namespace restocker_task_executor
{

// Construct on main before rclcpp or worker threads. The executable keeps the mask installed
// through process exit; tests may restore it only after all pending signals are consumed.
class TerminationSignalWaiter
{
public:
  explicit TerminationSignalWaiter(bool restore_mask_on_destruction = false);
  ~TerminationSignalWaiter();

  TerminationSignalWaiter(const TerminationSignalWaiter &) = delete;
  TerminationSignalWaiter & operator=(const TerminationSignalWaiter &) = delete;
  TerminationSignalWaiter(TerminationSignalWaiter &&) = delete;
  TerminationSignalWaiter & operator=(TerminationSignalWaiter &&) = delete;

  [[nodiscard]] std::optional<int> wait_for(std::chrono::milliseconds timeout) const;

private:
  sigset_t signals_{};
  sigset_t previous_mask_{};
  bool restore_mask_on_destruction_{false};
  bool mask_installed_{false};
};

}  // namespace restocker_task_executor
