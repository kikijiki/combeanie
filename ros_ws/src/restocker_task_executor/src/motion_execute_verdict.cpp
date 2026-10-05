// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/motion_execute_verdict.hpp"

namespace restocker_task_executor
{

MotionOutcome motion_outcome_for(PlanningPhase phase, PlanningStatus status) noexcept
{
  if (status == PlanningStatus::kSuccess) {
    return MotionOutcome::kSucceeded;
  }
  if (status == PlanningStatus::kServerUnavailable) {
    return MotionOutcome::kUnavailable;
  }
  // Anything that fails before execute() is issued cannot have moved the arm.
  return phase == PlanningPhase::kPlan ? MotionOutcome::kPlanningFailed :
         MotionOutcome::kExecutionFailed;
}

ExecuteVerdict classify_execute_result(int moveit_error_code, const std::string & label)
{
  ExecuteVerdict verdict;
  verdict.planning = classify_moveit_error(PlanningPhase::kExecute, moveit_error_code, label);
  verdict.outcome = motion_outcome_for(PlanningPhase::kExecute, verdict.planning.status);
  verdict.reached_terminal_stop = execution_failure_reached_terminal_stop(moveit_error_code);
  return verdict;
}

}  // namespace restocker_task_executor
