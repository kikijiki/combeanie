// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include "restocker_task_executor/motion_port.hpp"
#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{

// What a failed MoveIt execute() means for the MotionPort completion (Card 086 stage 1a). The
// MoveIt port builds its failure completion from exactly this, so the mapping that decides
// whether the completion claims a terminal stop is testable without MoveIt or a simulator.
struct ExecuteVerdict
{
  PlanningResult planning;
  MotionOutcome outcome{MotionOutcome::kExecutionFailed};
  // Only CONTROL_FAILED proves the controllers reached a terminal stop; TIMED_OUT does not.
  bool reached_terminal_stop{false};
  // execute() was already called when this verdict was formed, so the arm may be moving whatever
  // the outcome class (kUnavailable included): never "not started" (stage 1b review B1).
  bool submitted_to_backend{true};
};

[[nodiscard]] ExecuteVerdict classify_execute_result(
  int moveit_error_code,
  const std::string & label);

// The MotionOutcome a classified planning or execution status maps to.
[[nodiscard]] MotionOutcome motion_outcome_for(PlanningPhase phase, PlanningStatus status) noexcept;

}  // namespace restocker_task_executor
