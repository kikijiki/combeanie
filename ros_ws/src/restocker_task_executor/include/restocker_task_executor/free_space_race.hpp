// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <span>
#include <string>
#include <vector>

// Card 053 / Milestone 10 §6 "Parallel planning inside a free-space slice": the pure decision
// layer of the race. The port runs ompl (RRTConnect) and ompl_fallback (PRM) concurrently
// inside the leg's unchanged slice; this module turns the pipelines' outcomes into the one
// classification code Card 043's rules consume and into the per-pipeline refusal receipt the
// spec requires. Nothing here plans, sleeps, or touches ROS — every test is deterministic.

namespace restocker_task_executor
{

inline constexpr const char * kRacePrimaryPipeline = "ompl";
inline constexpr const char * kRaceFallbackPipeline = "ompl_fallback";
inline constexpr const char * kRaceFallbackPlannerId = "PRMkConfigDefault";

// One pipeline's answer inside one raced slice.
struct RacePipelineOutcome
{
  std::string name;
  int error_code{0};
  double seconds{0.0};
  bool has_trajectory{false};
};

struct RaceDecision
{
  // True when any pipeline produced a trajectory; the caller takes that trajectory.
  bool success{false};
  // The single code classify_plan_failure receives when every pipeline loses. Precedence:
  // a start-state code any pipeline reported (the start state is shared, so it is a scene
  // fact — Card 043), otherwise the primary pipeline's code. The timeout class still comes
  // from the wall time against the slice via PlanningSlice::exhausted(), exactly as today.
  int classification_code{0};
  double wall_seconds{0.0};
  // Per-pipeline receipt fragment: "ompl=TIMED_OUT (code 9) in 14.99 s; ompl_fallback=...".
  std::string receipt;
};

// Decide a raced slice from its outcomes and the wall time the race took. `outcomes` must
// carry every pipeline that ran (the spec: the classification uses ALL the evidence).
[[nodiscard]] RaceDecision decide_race(
  std::span<const RacePipelineOutcome> outcomes, double wall_seconds);

// True for the codes Card 043 classifies as start-state conditions (mirrors
// classify_moveit_error's set, including INVALID_ROBOT_STATE).
[[nodiscard]] bool is_start_state_code(int moveit_error_code);

}  // namespace restocker_task_executor
