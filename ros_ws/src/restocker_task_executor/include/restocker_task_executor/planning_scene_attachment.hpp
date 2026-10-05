// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "restocker_task_executor/planning_scene_reconciliation.hpp"

namespace restocker_task_executor
{

struct AttachedObjectProjection
{
  moveit_msgs::msg::PlanningScene diff;
  moveit_msgs::msg::AttachedCollisionObject expected_attached_object;
};

struct DetachedObjectProjection
{
  moveit_msgs::msg::PlanningScene diff;
  moveit_msgs::msg::CollisionObject expected_world_object;
};

[[nodiscard]] ProjectionResult<AttachedObjectProjection> build_world_to_attached_diff(
  const moveit_msgs::msg::CollisionObject & world_object,
  const std::string & parent_link,
  const std::vector<std::string> & touch_links,
  const geometry_msgs::msg::Pose & observed_parent_to_child);

[[nodiscard]] ProjectionResult<DetachedObjectProjection> build_attached_to_world_diff(
  const moveit_msgs::msg::AttachedCollisionObject & attached_object,
  const std::string & planning_frame,
  const geometry_msgs::msg::Pose & observed_child_in_world);

[[nodiscard]] ProjectionResult<std::monostate> verify_world_to_attached_transition(
  const moveit_msgs::msg::AttachedCollisionObject & expected,
  const moveit_msgs::msg::PlanningScene & observed,
  const SceneVerificationConfig & config = {});

[[nodiscard]] ProjectionResult<std::monostate> verify_attached_to_world_transition(
  const moveit_msgs::msg::CollisionObject & expected,
  const moveit_msgs::msg::PlanningScene & observed,
  const SceneVerificationConfig & config = {});

}  // namespace restocker_task_executor
