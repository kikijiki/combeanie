// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/survey_submit_refusal.hpp"

namespace restocker_task_executor
{

using restocker_interfaces::action::SurveyLane;
using restocker_interfaces::action::SurveyViewpoint;

void apply_lane_submit_refusal(SurveySubmitStatus status, SurveyLane::Result & result) noexcept
{
  if (status == SurveySubmitStatus::kBusy) {
    result.outcome = SurveyLane::Result::OUTCOME_UNAVAILABLE;
    result.motion_definitely_not_started = false;
  } else {
    result.outcome = SurveyLane::Result::OUTCOME_INVALID_REQUEST;
  }
}

void apply_viewpoint_submit_refusal(
  SurveySubmitStatus status, SurveyViewpoint::Result & result) noexcept
{
  result.outcome = status == SurveySubmitStatus::kBusy ?
    SurveyViewpoint::Result::OUTCOME_EXECUTION_FAILED :
    SurveyViewpoint::Result::OUTCOME_INVALID_REQUEST;
  result.execution_reached_terminal_stop = false;
}

}  // namespace restocker_task_executor
