// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <string>
#include <vector>

#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{

// Re-timing a controller slice may move its geometric path by at most this much in the mixed
// rail/revolute space (MoveIt warns that TOTG's tolerance mixes metres and radians here). For
// comparison, the planners time their output at 1e-3 (OMPL adapter) and 0.1 (move_group's
// Cartesian service, TOTG's default).
inline constexpr double kControllerSlicePathTolerance = 1e-6;
// TOTG with the strict tolerance fails numerically on some of move_group's own resampled output
// (Card 061). One retry at this tolerance, still two orders below the OMPL adapter's, is the only
// fallback; a slice neither can time is refused. Milestone 10 §6.
inline constexpr double kControllerSliceFallbackPathTolerance = 1e-5;
inline constexpr double kControllerSliceResampleDtS = 0.1;
inline constexpr double kControllerSliceMinimumAngleChange = 0.001;
// TOTG (move_group's and the slices' own) starts and ends a trajectory one 1 ms integration step
// from rest, so its endpoint velocities are a*dt, not zero. An endpoint is at rest when every
// joint is within its scaled acceleration bound times this window (two such steps).
inline constexpr double kControllerGoalRestWindowS = 0.002;

[[nodiscard]] TrajectoryObservation observe_trajectory(
  const moveit_msgs::msg::RobotTrajectory & trajectory);

struct ControllerGoalDivision
{
  // Contiguous controller goals, each starting and ending at rest; empty when refused.
  std::vector<moveit_msgs::msg::RobotTrajectory> goals;
  // Non-empty exactly when the trajectory was refused, naming the step that refused it.
  std::string refusal;
  // The whole planned trajectory fit one goal and already stopped at both ends, so it executes
  // with the planner's own timing, unmodified.
  bool planner_timing_kept{false};
  // Slices that the strict tolerance could not time and the fallback tolerance did.
  std::size_t fallback_retimes{0};

  [[nodiscard]] bool ok() const noexcept {return refusal.empty() && !goals.empty();}
};

// Divide a planned, time-parameterized trajectory into stop-to-stop controller goals no longer
// than maximum_duration (Milestone 10 §6, Card 061). The bound itself is never relaxed.
[[nodiscard]] ControllerGoalDivision divide_into_controller_goals(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const moveit::core::RobotState & planning_start, const std::string & planning_group,
  double velocity_scaling, double acceleration_scaling,
  std::chrono::milliseconds maximum_duration);

}  // namespace restocker_task_executor
