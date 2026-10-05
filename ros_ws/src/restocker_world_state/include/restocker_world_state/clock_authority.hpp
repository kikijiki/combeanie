// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include <rclcpp/clock.hpp>

#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{

// Own this after the store and before opening ingress. Drain ingress before destruction; the
// store must outlive this object. Managed TimeSource writers take Clock -> Store, matching the
// sample-and-admit path. Neither the store nor its pre-jump hook may call back into the clock.
class ClockAuthority
{
public:
  ClockAuthority(WorldStateStore & store, rclcpp::Clock::SharedPtr clock);
  ~ClockAuthority();

  ClockAuthority(const ClockAuthority &) = delete;
  ClockAuthority & operator=(const ClockAuthority &) = delete;

  // The callable must not re-enter this wrapper or manipulate the clock. It receives the
  // original sample while the update mutex stays held through its store admission.
  template<typename Function>
  auto with_sample(Function && function)
  {
    std::lock_guard lock(clock_->get_clock_mutex());
    const bool managed = clock_->get_clock_type() == RCL_ROS_TIME &&
      clock_->ros_time_is_active() && clock_->started();
    const auto sample = store_.capture_authority_clock(clock_->now(), managed);
    return std::invoke(std::forward<Function>(function), sample);
  }

private:
  WorldStateStore & store_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::JumpHandler::SharedPtr jump_handler_;
};

}  // namespace restocker_world_state
