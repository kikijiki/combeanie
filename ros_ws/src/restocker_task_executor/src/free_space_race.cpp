// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/free_space_race.hpp"

#include <sstream>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{

bool is_start_state_code(int moveit_error_code)
{
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  return moveit_error_code == ErrorCodes::START_STATE_IN_COLLISION ||
         moveit_error_code == ErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS ||
         moveit_error_code == ErrorCodes::START_STATE_INVALID ||
         moveit_error_code == ErrorCodes::INVALID_ROBOT_STATE;
}

RaceDecision decide_race(
  std::span<const RacePipelineOutcome> outcomes, double wall_seconds)
{
  RaceDecision decision;
  decision.wall_seconds = wall_seconds;
  std::ostringstream receipt;
  for (std::size_t index = 0U; index < outcomes.size(); ++index) {
    const auto & outcome = outcomes[index];
    if (index > 0U) {
      receipt << "; ";
    }
    receipt << outcome.name << '=' << moveit_error_name(outcome.error_code) << " (code "
            << outcome.error_code << ") in " << outcome.seconds << " s";
    if (outcome.has_trajectory) {
      decision.success = true;
    }
  }
  decision.receipt = receipt.str();
  if (decision.success) {
    // No classification is spent on a slice that produced a trajectory; keep a defined code
    // for logging only.
    decision.classification_code = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return decision;
  }
  // All pipelines lost: a start-state code from either is a property of the shared start
  // state (Card 043 — never a pose verdict); otherwise the primary pipeline speaks.
  decision.classification_code = outcomes.empty() ?
    moveit_msgs::msg::MoveItErrorCodes::FAILURE : outcomes.front().error_code;
  for (const auto & outcome : outcomes) {
    if (is_start_state_code(outcome.error_code)) {
      decision.classification_code = outcome.error_code;
      break;
    }
  }
  return decision;
}

}  // namespace restocker_task_executor
