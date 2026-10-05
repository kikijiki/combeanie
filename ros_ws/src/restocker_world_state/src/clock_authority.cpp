// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include "restocker_world_state/clock_authority.hpp"

#include <stdexcept>
#include <utility>

namespace restocker_world_state
{

ClockAuthority::ClockAuthority(WorldStateStore & store, rclcpp::Clock::SharedPtr clock)
: store_(store), clock_(std::move(clock))
{
  if (!clock_) {
    throw std::invalid_argument("clock authority requires a clock");
  }
  if (clock_->get_clock_type() == RCL_ROS_TIME) {
    rcl_jump_threshold_t threshold{};
    threshold.on_clock_change = true;
    threshold.min_backward.nanoseconds = -1;
    threshold.min_forward.nanoseconds = 0;  // Zero disables ordinary forward notifications.
    // Registration itself locks the clock mutex. No clock/store lock is held here.
    jump_handler_ = clock_->create_jump_callback(
      [&store]() noexcept {store.notify_clock_discontinuity();}, nullptr, threshold);
  }
}

ClockAuthority::~ClockAuthority()
{
  // Removal locks the clock and waits out a managed pre-hook. Never hold the store mutex here;
  // the store and explicit clock owner remain alive until the handler has been removed.
  jump_handler_.reset();
}

}  // namespace restocker_world_state
