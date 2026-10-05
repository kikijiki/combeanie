// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <restocker_interfaces/action/survey_lane.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>

#include "restocker_task_executor/viewpoint_survey.hpp"

namespace restocker_task_executor
{

// What the survey endpoints report when ViewpointSurvey::submit refuses a goal synchronously
// (Card 086 stage 1b review). kBusy means an earlier survey is still outstanding, so the arm may be
// moving under it: neither "no command was issued" nor a stop may be claimed. Any other refusal
// happened before anything was submitted.

// Sets the lane result's outcome and clears motion_definitely_not_started for kBusy (the other
// flag is left untouched; the caller's pre-motion default of true stands for a real non-start).
void apply_lane_submit_refusal(
  SurveySubmitStatus status, restocker_interfaces::action::SurveyLane::Result & result) noexcept;

// Sets the viewpoint result's outcome: OUTCOME_EXECUTION_FAILED (never the "never engaged"
// OUTCOME_UNAVAILABLE) for kBusy, OUTCOME_INVALID_REQUEST otherwise, with no stop claimed.
void apply_viewpoint_submit_refusal(
  SurveySubmitStatus status,
  restocker_interfaces::action::SurveyViewpoint::Result & result) noexcept;

}  // namespace restocker_task_executor
