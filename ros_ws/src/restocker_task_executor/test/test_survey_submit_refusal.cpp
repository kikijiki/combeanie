// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Card 086 stage 1b review 3 S2b: the survey endpoints' busy-refusal proof fields, no simulator.

#include <gtest/gtest.h>

#include "restocker_task_executor/survey_submit_refusal.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_interfaces::action::SurveyLane;
using restocker_interfaces::action::SurveyViewpoint;

TEST(SurveySubmitRefusal, ABusyLaneRefusalClaimsNeitherNonStartNorStop)
{
  SurveyLane::Result result;
  result.motion_definitely_not_started = true;  // the node's pre-motion default
  apply_lane_submit_refusal(SurveySubmitStatus::kBusy, result);
  EXPECT_EQ(result.outcome, SurveyLane::Result::OUTCOME_UNAVAILABLE);
  EXPECT_FALSE(result.motion_definitely_not_started);
  EXPECT_FALSE(result.execution_reached_terminal_stop);
}

TEST(SurveySubmitRefusal, AnInvalidLaneRequestStillProvesNothingWasCommanded)
{
  SurveyLane::Result result;
  result.motion_definitely_not_started = true;
  apply_lane_submit_refusal(SurveySubmitStatus::kInvalidRequest, result);
  EXPECT_EQ(result.outcome, SurveyLane::Result::OUTCOME_INVALID_REQUEST);
  EXPECT_TRUE(result.motion_definitely_not_started);
}

TEST(SurveySubmitRefusal, ABusyViewpointRefusalIsNeverTheNeverEngagedOutcome)
{
  SurveyViewpoint::Result result;
  result.execution_reached_terminal_stop = true;
  apply_viewpoint_submit_refusal(SurveySubmitStatus::kBusy, result);
  EXPECT_EQ(result.outcome, SurveyViewpoint::Result::OUTCOME_EXECUTION_FAILED);
  EXPECT_NE(result.outcome, SurveyViewpoint::Result::OUTCOME_UNAVAILABLE);
  EXPECT_FALSE(result.execution_reached_terminal_stop);
}

TEST(SurveySubmitRefusal, AnInvalidViewpointRequestIsInvalid)
{
  SurveyViewpoint::Result result;
  apply_viewpoint_submit_refusal(SurveySubmitStatus::kInvalidRequest, result);
  EXPECT_EQ(result.outcome, SurveyViewpoint::Result::OUTCOME_INVALID_REQUEST);
}

}  // namespace
}  // namespace restocker_task_executor
