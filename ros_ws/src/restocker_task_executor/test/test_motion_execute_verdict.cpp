// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Card 086 stage 1a, at the MoveIt port's call site: the completion a failed execute() produces.

#include <gtest/gtest.h>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "restocker_task_executor/motion_execute_verdict.hpp"

namespace restocker_task_executor
{
namespace
{

using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

TEST(MotionExecuteVerdict, TimedOutIsAnExecutionFailureWithoutAStopClaim)
{
  const auto verdict = classify_execute_result(ErrorCodes::TIMED_OUT, "joint goal");
  EXPECT_EQ(verdict.outcome, MotionOutcome::kExecutionFailed);
  EXPECT_FALSE(verdict.reached_terminal_stop);
  EXPECT_FALSE(verdict.planning.ok());
  // Not a not-started claim either: the arm may have moved.
  EXPECT_FALSE(motion_definitely_not_started(verdict.outcome));
}

TEST(MotionExecuteVerdict, ControlFailedStillClaimsTheStopTheControllersReported)
{
  const auto verdict = classify_execute_result(ErrorCodes::CONTROL_FAILED, "joint goal");
  EXPECT_EQ(verdict.outcome, MotionOutcome::kExecutionFailed);
  EXPECT_TRUE(verdict.reached_terminal_stop);
}

TEST(MotionExecuteVerdict, OtherExecuteFailuresClaimNoStop)
{
  for (const int code :
    {static_cast<int>(ErrorCodes::FAILURE), static_cast<int>(ErrorCodes::PREEMPTED),
      static_cast<int>(ErrorCodes::INVALID_ROBOT_STATE)})
  {
    const auto verdict = classify_execute_result(code, "joint goal");
    EXPECT_EQ(verdict.outcome, MotionOutcome::kExecutionFailed) << code;
    EXPECT_FALSE(verdict.reached_terminal_stop) << code;
  }
  // An execute-phase backend loss keeps its retryable kUnavailable class, but it is a post-
  // submission failure: the verdict says execute() was already called, and a completion built from
  // it can never prove "not started" (stage 1b review B1; the old test pinned the opposite).
  for (const int code :
    {static_cast<int>(ErrorCodes::COMMUNICATION_FAILURE), static_cast<int>(ErrorCodes::CRASH)})
  {
    const auto unavailable = classify_execute_result(code, "g");
    EXPECT_EQ(unavailable.outcome, MotionOutcome::kUnavailable) << code;
    EXPECT_FALSE(unavailable.reached_terminal_stop) << code;
    EXPECT_TRUE(unavailable.submitted_to_backend) << code;
    MotionCompletion completion;
    completion.outcome = unavailable.outcome;
    completion.submitted_to_backend = unavailable.submitted_to_backend;
    EXPECT_FALSE(motion_completion_definitely_not_started(completion)) << code;
  }
}

TEST(MotionExecuteVerdict, OnlyAPreSubmissionRefusalProvesNonStart)
{
  MotionCompletion completion;
  completion.outcome = MotionOutcome::kUnavailable;
  EXPECT_TRUE(motion_completion_definitely_not_started(completion));
  completion.outcome = MotionOutcome::kPlanningFailed;
  EXPECT_TRUE(motion_completion_definitely_not_started(completion));
  completion.grasp_escape_executed = true;
  EXPECT_FALSE(motion_completion_definitely_not_started(completion));
  completion.grasp_escape_executed = false;
  completion.submitted_to_backend = true;
  EXPECT_FALSE(motion_completion_definitely_not_started(completion));
  completion.submitted_to_backend = false;
  completion.outcome = MotionOutcome::kExecutionFailed;
  EXPECT_FALSE(motion_completion_definitely_not_started(completion));
}

TEST(MotionExecuteVerdict, SuccessMapsToSucceeded)
{
  EXPECT_EQ(classify_execute_result(ErrorCodes::SUCCESS, "g").outcome, MotionOutcome::kSucceeded);
  EXPECT_EQ(
    motion_outcome_for(PlanningPhase::kPlan, PlanningStatus::kPlanningRejected),
    MotionOutcome::kPlanningFailed);
}

}  // namespace
}  // namespace restocker_task_executor
