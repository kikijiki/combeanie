// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/campaign_motion_evidence.hpp"

#include <string>
#include <utility>

namespace restocker_task_executor
{
namespace
{

using RestockProduct = restocker_interfaces::action::RestockProduct;
using SurveyLane = restocker_interfaces::action::SurveyLane;
using SurveyTray = restocker_interfaces::action::SurveyTray;
using SurveyViewpoint = restocker_interfaces::action::SurveyViewpoint;

MotionEvidence delivered(
  bool arrived, bool not_started, bool stop, bool descendant_unsettled, std::string note)
{
  MotionEvidence evidence;
  evidence.kind = MotionEvidence::Kind::kDeliveredTerminal;
  evidence.arrived = arrived;
  evidence.motion_definitely_not_started = not_started;
  evidence.execution_reached_terminal_stop = stop;
  evidence.descendant_unsettled = descendant_unsettled;
  evidence.note = std::move(note);
  return evidence;
}

}  // namespace

MotionEvidence evidence_of(rclcpp_action::ResultCode code, const SurveyLane::Result & result)
{
  return delivered(
    code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.outcome == SurveyLane::Result::OUTCOME_OBSERVED,
    result.motion_definitely_not_started, result.execution_reached_terminal_stop, false,
    "lane terminal outcome " + std::to_string(result.outcome));
}

MotionEvidence evidence_of(rclcpp_action::ResultCode code, const SurveyTray::Result & result)
{
  // A completed survey (including a candidate-less or refuted one) arrived at every leg it ran.
  const bool completed = code == rclcpp_action::ResultCode::SUCCEEDED &&
    (result.outcome == SurveyTray::Result::OUTCOME_CONFIRMED ||
    result.outcome == SurveyTray::Result::OUTCOME_NO_CANDIDATE ||
    result.outcome == SurveyTray::Result::OUTCOME_REFUTED ||
    result.outcome == SurveyTray::Result::OUTCOME_OVERVIEW_ONLY);
  const bool child_may_be_running = result.outcome == SurveyTray::Result::OUTCOME_CANCELED ||
    result.outcome == SurveyTray::Result::OUTCOME_UNAVAILABLE;
  return delivered(
    completed, result.motion_definitely_not_started, result.execution_reached_terminal_stop,
    child_may_be_running, "tray terminal outcome " + std::to_string(result.outcome));
}

MotionEvidence evidence_of(rclcpp_action::ResultCode code, const SurveyViewpoint::Result & result)
{
  return delivered(
    code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.outcome == SurveyViewpoint::Result::OUTCOME_ARRIVED,
    false, result.execution_reached_terminal_stop, false,
    "viewpoint terminal outcome " + std::to_string(result.outcome));
}

MotionEvidence evidence_of(rclcpp_action::ResultCode code, const RestockProduct::Result & result)
{
  const bool arrived =
    (code == rclcpp_action::ResultCode::SUCCEEDED &&
    result.status == RestockProduct::Result::STATUS_SUCCEEDED) ||
    result.status == RestockProduct::Result::STATUS_SKIPPED_RECOVERABLE;
  // Stage 1b: the coordinator's own proof fields count, except for the statuses that mean the
  // world or the coordinator is in doubt (the driver never sets them there; this repeats the rule
  // so a hand-built or future result cannot settle them either).
  const bool doubtful = result.status == RestockProduct::Result::STATUS_UNSET ||
    result.status == RestockProduct::Result::STATUS_EXTERNAL_INCONSISTENCY ||
    result.status == RestockProduct::Result::STATUS_OPERATOR_REQUIRED ||
    result.status == RestockProduct::Result::STATUS_SHUTDOWN ||
    result.status == RestockProduct::Result::STATUS_INTERNAL_ERROR;
  const bool not_started =
    result.status == RestockProduct::Result::STATUS_NO_COMPATIBLE_PAIR ||
    result.status == RestockProduct::Result::STATUS_OBSERVATION_EVIDENCE_STALE ||
    (!doubtful && result.motion_definitely_not_started);
  const bool stop = !doubtful && result.execution_reached_terminal_stop;
  return delivered(
    arrived, not_started, stop, false, "restock terminal status " + std::to_string(result.status));
}

}  // namespace restocker_task_executor
