// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/survey_recovery_context.hpp"

#include <string>
#include <utility>

namespace restocker_task_executor
{

RecoveryContext survey_recovery_context(const SurveyFailureEvidence & evidence)
{
  RecoveryContext context;
  // Arm state: only a delivered terminal result carrying first-hand motion evidence can
  // establish a stop. A result with neither flag is a check that ran and did not hold, not
  // an unknown, so the classifier reports the refused stop rather than the unknown branch.
  const bool stop_established = evidence.motion_definitely_not_started ||
    evidence.execution_reached_terminal_stop;
  if (!evidence.terminal_result_received) {
    context.arm_stop = RecoveryEvidence::kUnknown;
  } else if (stop_established) {
    context.arm_stop = RecoveryEvidence::kEstablished;
  } else {
    context.arm_stop = RecoveryEvidence::kNotEstablished;
  }
  context.held_state = evidence.transfer_release_established ?
    RecoveryEvidence::kEstablished : RecoveryEvidence::kUnknown;
  // The campaign has no contact or hardware-fault channel for a survey action: every survey
  // segment is a padded-scene-checked free-space plan under the same gate §6 already trusts,
  // so "real contact beyond the padded scene" is not established by any evidence the
  // campaign holds, and a persistent controller fault surfaces as an action that never
  // delivers terminal evidence — which the arm-state rule above already fails closed on.
  context.contact_or_collision = false;
  context.persistent_hardware_fault = false;
  context.world_reobservable = evidence.world_reobservable;
  context.cause = evidence.cause;
  return context;
}

}  // namespace restocker_task_executor
