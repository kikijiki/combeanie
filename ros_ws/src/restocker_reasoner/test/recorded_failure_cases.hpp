// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "restocker_reasoner/recovery_advice.hpp"

namespace restocker_reasoner
{

// The failures the reasoner is asked to classify, in the coordinator's own words.
//
// These are the three motion failures observed across ten standalone runs of the manipulation
// acceptance test on this tree, which fails about four times in ten: a Cartesian approach whose
// straight line stopped 41.6667% of the way in, a pre-grasp traverse the controllers aborted with
// CONTROL_FAILED, and a pre-insert traverse the sampling planner found no solution for. The
// `observed_detail` strings are the terminal details the coordinator reported, verbatim.
//
// All three are failures the recovery authorisation permits a bounded replan for, so a reasoner
// is asked about them. A failure the authorisation refuses leaves stopping as the only permitted
// primitive and is never put to a backend.
//
// Fidelity caveat: a live query carries the motion port's own detail, which differs from the
// coordinator's terminal detail in prefix and sometimes length (for the truncated line it also
// names the MoveIt error code and whether the line was refused with collision checking off). The
// strings below are the coordinator's, as observed. The failure described is the same.
//
// The probe asks these questions and the recorded-response test replays the answers against them,
// so the two cannot drift apart.
struct RecordedFailureCase
{
  std::string_view name;
  RecoveryQuery query;
};

[[nodiscard]] inline std::vector<RecordedFailureCase> recorded_failure_cases()
{
  std::vector<RecordedFailureCase> cases;

  {
    // The deterministic policy already knows the answer: a straight line that cannot be
    // completed from the configuration the preceding traverse chose is sent back to that
    // traverse, which chooses a different configuration.
    RecoveryQuery query;
    query.request_id = "rec-linear-path-truncated";
    query.goal_generation = 1U;
    query.failed_segment = "approach";
    query.motion_outcome = "planning failed";
    query.task_state = "PlanApproach";
    query.observed_detail =
      "approach motion planning failed: linear path stopped 41.6667% of the way to the goal "
      "after 17 interpolated waypoints";
    query.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
    query.permitted_primitives = {
      RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
    query.recovery_attempt = 1U;
    query.maximum_recovery_attempts = 2U;
    cases.push_back({"linear_path_truncated", std::move(query)});
  }

  {
    RecoveryQuery query;
    query.request_id = "rec-execution-aborted";
    query.goal_generation = 1U;
    query.failed_segment = "pre-grasp";
    query.motion_outcome = "execution failed";
    query.task_state = "ExecutePreGrasp";
    query.observed_detail =
      "pre-grasp motion execution failed: trajectory execution was rejected or aborted (MoveIt "
      "reported CONTROL_FAILED, code -4)";
    query.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
    query.permitted_primitives = {
      RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
    query.recovery_attempt = 1U;
    query.maximum_recovery_attempts = 2U;
    cases.push_back({"trajectory_execution_aborted", std::move(query)});
  }

  {
    RecoveryQuery query;
    query.request_id = "rec-free-space-planning-failed";
    query.goal_generation = 1U;
    query.failed_segment = "pre-insert";
    query.motion_outcome = "planning failed";
    query.task_state = "PlanPreInsert";
    query.observed_detail =
      "pre-insert motion planning failed: motion planning did not produce a valid solution "
      "(MoveIt reported FAILURE)";
    query.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
    query.permitted_primitives = {
      RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
    query.recovery_attempt = 1U;
    query.maximum_recovery_attempts = 2U;
    cases.push_back({"free_space_planning_failed", std::move(query)});
  }

  return cases;
}

[[nodiscard]] inline std::optional<RecoveryQuery> recorded_failure_case(std::string_view name)
{
  for (auto & recorded : recorded_failure_cases()) {
    if (recorded.name == name) {
      return recorded.query;
    }
  }
  return std::nullopt;
}

}  // namespace restocker_reasoner
