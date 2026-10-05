// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// SPDX-License-Identifier: MIT

#pragma once

#include <functional>
#include <string>

#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "restocker_task_executor/pregrasp_planning_authority.hpp"
#include "restocker_task_executor/race_planning_scene.hpp"

namespace restocker_task_executor
{

inline constexpr std::uint32_t kTrajectorySceneComponents = kRaceSceneComponents |
  moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
  moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
  moveit_msgs::msg::PlanningSceneComponents::OCTOMAP;

struct TrajectorySceneGateHooks
{
  // Must causally read projector-owned authority via its serialized RPC, including freshness
  // and live lease validation. Cached topic evidence is insufficient for submission.
  std::function<PlanningSceneAuthorityResult()> read_authority;
  // Fetch and collision-check exactly the next controller goal; empty means collision-free.
  std::function<std::string()> validate_next_goal;
  // The caller owns the existing operation cancellation and steady deadline.
  std::function<bool()> interrupted;
};

struct TrajectorySceneGateResult
{
  bool accepted{false};
  bool revalidated{false};
  std::string detail;
};

// Original provenance is immutable. Acceptance authorizes only the next submission, never a
// later slice; changed scenes get at most three validation attempts within the caller's budget.
[[nodiscard]] TrajectorySceneGateResult validate_trajectory_scene_submission(
  const PreGraspSceneAuthority & original, const TrajectorySceneGateHooks & hooks);

// No geometry exemptions. Full-scene padding/ACM/attachments are imported through the race
// builder, requiring every collision link's padding/scale. Checks the measured start bridge
// and controller linear/cubic/quintic spline at <=0.0025 m/rad, with a 50,000-sample ceiling.
// Only measured-bridge bounds may use the caller's existing start allowance; geometry stays actual.
[[nodiscard]] std::string validate_controller_goal_collision_scene(
  const moveit::core::RobotModelConstPtr & model,
  const moveit_msgs::msg::PlanningScene & snapshot,
  const moveit_msgs::msg::RobotTrajectory & trajectory, const std::string & group,
  const std::function<bool()> & interrupted,
  double maximum_start_state_bounds_correction = 0.0);

}  // namespace restocker_task_executor
