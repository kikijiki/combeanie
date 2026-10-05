// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_operation_types.hpp"
#include "restocker_task_executor/restock_action_contract.hpp"

namespace restocker_task_executor
{

enum class CoordinatorDriverOutputKind : std::uint8_t
{
  kFeedback,
  kCanceled,
  kAborted,
  kSucceeded,
  kInhibited,
};

struct CoordinatorDriverOutput
{
  CoordinatorDriverOutputKind kind{CoordinatorDriverOutputKind::kFeedback};
  GoalGeneration goal_generation{0};
  RestockTaskTransition transition;
  std::optional<SelectedTaskPair> selected_pair;
  RestockActionOutcome outcome{RestockActionOutcome::kUnset};
  RestockActionMetrics metrics;
  std::string detail;
};

static_assert(std::is_nothrow_move_constructible_v<CoordinatorDriverOutput>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorDriverOutput>);

// A one-place ownership boundary for terminal output withheld by the ROS adapter. A conflicting
// second terminal never replaces or moves from the first; callers must enter unsecured state.
class CoordinatorRetainedTerminal final
{
public:
  CoordinatorRetainedTerminal() = default;
  CoordinatorRetainedTerminal(const CoordinatorRetainedTerminal &) = delete;
  CoordinatorRetainedTerminal & operator=(const CoordinatorRetainedTerminal &) = delete;

  CoordinatorRetainedTerminal(CoordinatorRetainedTerminal && other) noexcept
  : output_(other.take()) {}

  CoordinatorRetainedTerminal & operator=(CoordinatorRetainedTerminal && other) noexcept
  {
    if (this != &other) {
      output_ = other.take();
    }
    return *this;
  }

  [[nodiscard]] bool retain(CoordinatorDriverOutput && output) noexcept
  {
    if (output_) {
      return false;
    }
    output_.emplace(std::move(output));
    return true;
  }

  [[nodiscard]] bool occupied() const noexcept {return output_.has_value();}

  // Borrowed inspection for audit and invariant verification. The pointer is valid only while the
  // caller holds the owner's external synchronization and until the next slot mutation.
  [[nodiscard]] const CoordinatorDriverOutput * inspect() const noexcept
  {
    return output_ ? &*output_ : nullptr;
  }

  [[nodiscard]] bool conflicts_with(const CoordinatorDriverOutput & output) const noexcept
  {
    const bool terminal = output.kind != CoordinatorDriverOutputKind::kFeedback &&
      output.kind != CoordinatorDriverOutputKind::kInhibited;
    return output_.has_value() && terminal;
  }

  [[nodiscard]] std::optional<CoordinatorDriverOutput> take() noexcept
  {
    std::optional<CoordinatorDriverOutput> result{std::move(output_)};
    output_.reset();
    return result;
  }

private:
  std::optional<CoordinatorDriverOutput> output_;
};

static_assert(std::is_nothrow_move_constructible_v<CoordinatorRetainedTerminal>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorRetainedTerminal>);
static_assert(!std::is_copy_constructible_v<CoordinatorRetainedTerminal>);
static_assert(!std::is_copy_assignable_v<CoordinatorRetainedTerminal>);

}  // namespace restocker_task_executor
