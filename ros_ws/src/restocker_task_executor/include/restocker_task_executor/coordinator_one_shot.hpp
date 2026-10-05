// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <type_traits>

namespace restocker_task_executor
{

/// The liveness of a capability that may be exercised at most once.
///
/// A capability owner stores this instead of a bare `bool` so that its own move operations can be
/// defaulted: moving from a capability clears the source, and nothing else can set it live again.
/// Self-move preserves liveness, so `x = std::move(x)` does not destroy authority. Copying is
/// unavailable because two live copies would defeat the one-shot guarantee.
class CoordinatorOneShot final
{
public:
  CoordinatorOneShot() = default;
  explicit CoordinatorOneShot(bool live) noexcept
  : live_(live) {}
  CoordinatorOneShot(const CoordinatorOneShot &) = delete;
  CoordinatorOneShot & operator=(const CoordinatorOneShot &) = delete;
  CoordinatorOneShot(CoordinatorOneShot && other) noexcept
  : live_(other.live_) {other.live_ = false;}
  CoordinatorOneShot & operator=(CoordinatorOneShot && other) noexcept
  {
    if (this != &other) {
      live_ = other.live_;
      other.live_ = false;
    }
    return *this;
  }
  ~CoordinatorOneShot() = default;

  [[nodiscard]] bool live() const noexcept {return live_;}
  void consume() noexcept {live_ = false;}

private:
  bool live_{false};
};

static_assert(!std::is_copy_constructible_v<CoordinatorOneShot>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorOneShot>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorOneShot>);

}  // namespace restocker_task_executor
