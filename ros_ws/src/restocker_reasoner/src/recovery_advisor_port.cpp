// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_reasoner/recovery_advisor_port.hpp"

#include <utility>

namespace restocker_reasoner
{

void RecoveryAdviceMailbox::deposit(RecoveryAdviceCompletion completion)
{
  const std::lock_guard<std::mutex> guard(mutex_);
  // One slot, last writer wins: there is only one outstanding question, and the newer would be
  // the one awaited.
  slot_ = std::move(completion);
}

std::optional<RecoveryAdviceCompletion> RecoveryAdviceMailbox::take(
  std::string_view request_id, std::uint64_t goal_generation)
{
  const std::lock_guard<std::mutex> guard(mutex_);
  if (!slot_) {
    return std::nullopt;
  }
  auto held = std::move(*slot_);
  slot_.reset();
  if (held.request_id != request_id || held.goal_generation != goal_generation) {
    return std::nullopt;
  }
  return held;
}

void RecoveryAdviceMailbox::clear()
{
  const std::lock_guard<std::mutex> guard(mutex_);
  slot_.reset();
}

bool RecoveryAdviceMailbox::occupied() const
{
  const std::lock_guard<std::mutex> guard(mutex_);
  return slot_.has_value();
}

}  // namespace restocker_reasoner
