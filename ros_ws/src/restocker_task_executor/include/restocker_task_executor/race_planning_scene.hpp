// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

// Milestone 10 §6 "The race plans against move_group's scene" (Card 068): the free-space race
// plans each pipeline on a local scene built from one /get_planning_scene snapshot. Racing
// never relaxes a check, so that scene carries everything move_group collision-checks with —
// world geometry, attached objects, the allowed-collision matrix and the link padding/scale
// (move_group pads every link by robot_description_planning.default_robot_padding). Without
// the last two a fresh PlanningScene has zero padding and the SRDF-only matrix, and the race
// accepted states move_group's padded scene refuses.

namespace restocker_task_executor
{

// What the race asks move_group's /get_planning_scene for.
inline constexpr std::uint32_t kRaceSceneComponents =
  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
  moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
  moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;

struct RaceSceneBuild
{
  // Null when the snapshot cannot give the race move_group's collision model.
  planning_scene::PlanningScenePtr scene;
  // Why `scene` is null; empty on success.
  std::string refusal;
};

// Builds one pipeline's local scene on `model` from move_group's snapshot. Fail-closed: a
// snapshot without link padding, link scale or the allowed-collision matrix — or one MoveIt
// refuses to apply — yields no scene, and the caller plans through move_group's own service.
[[nodiscard]] RaceSceneBuild make_race_scene(
  const moveit::core::RobotModelConstPtr & model, const moveit_msgs::msg::PlanningScene & snapshot);

}  // namespace restocker_task_executor
