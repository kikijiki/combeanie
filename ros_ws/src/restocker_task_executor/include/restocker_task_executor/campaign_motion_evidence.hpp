// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>
#include <restocker_interfaces/action/survey_lane.hpp>
#include <restocker_interfaces/action/survey_tray.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>

#include "restocker_task_executor/unresolved_motion.hpp"

namespace restocker_task_executor
{

// Card 086 stage 1 (CMB-SPEC-13): what a delivered terminal result proves about its attempt, per
// endpoint. Pure: the campaign node feeds the result to the registry through these, and the
// tests pin every row of the table against the real action messages.

[[nodiscard]] MotionEvidence evidence_of(
  rclcpp_action::ResultCode code, const restocker_interfaces::action::SurveyLane::Result & result);

// A tray terminal can be delivered while a viewpoint child leg is still unadmitted or running:
// the tray server returns on a stop request or after its 5 s send grace without awaiting the
// child, and tray_survey_node initialises motion_definitely_not_started=true and leaves it when
// the leg never "ran". CANCELED and UNAVAILABLE are the outcomes those returns carry, so for them
// no flag settles the attempt (DECIDED 5: only an explicit child-settled statement would, and the
// payload has none); every other outcome ran its legs to a terminal and is read by its flags.
[[nodiscard]] MotionEvidence evidence_of(
  rclcpp_action::ResultCode code, const restocker_interfaces::action::SurveyTray::Result & result);

[[nodiscard]] MotionEvidence evidence_of(
  rclcpp_action::ResultCode code,
  const restocker_interfaces::action::SurveyViewpoint::Result & result);

// Success and the typed recoverable skip arrive, the two selection-phase terminals end before any
// motion, and every other status is read from the coordinator's own proof fields
// (motion_definitely_not_started, execution_reached_terminal_stop; Card 086 stage 1b). Statuses
// that mean the world or the coordinator is in doubt (operator required, external inconsistency,
// shutdown, internal error, unset) never settle, whatever the fields say.
[[nodiscard]] MotionEvidence evidence_of(
  rclcpp_action::ResultCode code,
  const restocker_interfaces::action::RestockProduct::Result & result);

}  // namespace restocker_task_executor
