// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include <restocker_interfaces/action/restock_product.hpp>

#include "restocker_task_executor/restock_task_machine.hpp"
#include "restocker_task_executor/task_selection.hpp"

namespace restocker_task_executor
{

enum class RestockActionOutcome : std::uint8_t
{
  kUnset,
  kSucceeded,
  kCanceled,
  kNoCompatiblePair,
  kObservationEvidenceStale,
  kValidationFailed,
  kPlanningFailed,
  kExecutionFailed,
  kVerificationFailed,
  kRecoveryExhausted,
  kExternalInconsistency,
  kOperatorRequired,
  kShutdown,
  // Graded recovery rung 5 (Milestone 10 §6, Card 051): the typed recoverable skip terminal.
  kRecoverableSkip,
  kInternalError,
};

struct RestockActionMetrics
{
  restocker_world_state::Revision initial_world_revision{0};
  restocker_world_state::Revision final_world_revision{0};
  std::uint32_t attempt_count{0};
  std::uint32_t retry_count{0};
  std::chrono::milliseconds elapsed{0};
  double minimum_clearance_m{0.0};
  // Card 086 stage 1b: first-hand motion evidence the driver derived from its own submissions.
  bool motion_definitely_not_started{false};
  bool execution_reached_terminal_stop{false};
};

[[nodiscard]] SelectionResult<SelectionRequest> selection_request_from_goal(
  const restocker_interfaces::action::RestockProduct::Goal & goal);

[[nodiscard]] std::uint8_t action_state(RestockTaskState state) noexcept;
[[nodiscard]] std::uint16_t action_status(RestockActionOutcome outcome) noexcept;

[[nodiscard]] restocker_interfaces::action::RestockProduct::Feedback make_action_feedback(
  const RestockTaskTransition & transition, std::chrono::milliseconds elapsed,
  const std::optional<SelectedTaskPair> & selected_pair,
  RestockActionOutcome latest_outcome = RestockActionOutcome::kUnset,
  std::string detail = {});

[[nodiscard]] std::shared_ptr<restocker_interfaces::action::RestockProduct::Feedback>
make_action_feedback_message(
  const RestockTaskTransition & transition, std::chrono::milliseconds elapsed,
  const std::optional<SelectedTaskPair> & selected_pair,
  RestockActionOutcome latest_outcome = RestockActionOutcome::kUnset,
  std::string detail = {});

// Whether a terminal status may carry the metrics' motion evidence into the action result. The
// operator, inconsistency, shutdown and internal-error terminals never do: they mean the world or
// the coordinator itself is in doubt, so no history proves the robot settled.
[[nodiscard]] bool outcome_carries_motion_evidence(RestockActionOutcome outcome) noexcept;

[[nodiscard]] restocker_interfaces::action::RestockProduct::Result make_action_result(
  RestockActionOutcome outcome, const RestockActionMetrics & metrics, std::string detail = {});

}  // namespace restocker_task_executor
