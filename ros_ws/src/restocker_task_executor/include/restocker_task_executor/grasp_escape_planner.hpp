// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT


#pragma once

#include <string>
#include <vector>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/joint_model_group.hpp>
#include <moveit/robot_model/link_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{

// Milestone 10 §6 (Card 062): the grasp escape's straight line, decided on a planning scene the
// caller owns (a local copy of move_group's). Nothing here talks to ROS, so the rule is testable
// against the real robot model and MoveIt's own collision checker.
struct GraspEscapeLine
{
  bool planned{false};
  // Why not, or on success the receipt: tolerated pairs, waypoint count, endpoint verdict.
  std::string detail;
  std::vector<moveit::core::RobotStatePtr> waypoints;
  std::vector<CollisionContactPair> tolerated_pairs;
};

// The scene's own allowed-collision matrix is never modified: the scoped entries live in a copy
// used only for the line, and the endpoint is judged by the unmodified matrix.
[[nodiscard]] GraspEscapeLine plan_grasp_escape_line(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & start,
  const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * link,
  const GraspEscape & escape, double cartesian_step_m, double minimum_fraction);

// The port's entry point: builds the local scene from move_group's /get_planning_scene snapshot
// through Card 068's make_race_scene (request kRaceSceneComponents), so a snapshot without link
// padding, scale or the allowed-collision matrix, or one MoveIt refuses to apply, refuses the
// escape (fail-closed) instead of planning on a laxer scene. The start takes its joint positions
// from `measured` and its attached bodies from the snapshot.
[[nodiscard]] GraspEscapeLine plan_grasp_escape_from_snapshot(
  const moveit_msgs::msg::PlanningScene & snapshot, const moveit::core::RobotState & measured,
  const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * link,
  const GraspEscape & escape, double cartesian_step_m, double minimum_fraction);

}  // namespace restocker_task_executor
