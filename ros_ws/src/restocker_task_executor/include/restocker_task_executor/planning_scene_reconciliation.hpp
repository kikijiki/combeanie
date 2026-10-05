// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <set>
#include <string>
#include <variant>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>

#include "restocker_task_executor/world_snapshot_projection.hpp"

namespace restocker_task_executor
{

struct SceneVerificationConfig
{
  double dimension_tolerance{1.0e-9};
  double position_tolerance{1.0e-9};
  double orientation_tolerance_rad{1.0e-9};
};

[[nodiscard]] bool is_projector_managed_id(const std::string & id);

[[nodiscard]] ProjectionResult<moveit_msgs::msg::PlanningScene> build_planning_scene_diff(
  const std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const moveit_msgs::msg::PlanningScene & current_scene);

[[nodiscard]] ProjectionResult<std::monostate> verify_planning_scene(
  const std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const std::set<std::string> & required_attached_ids,
  const moveit_msgs::msg::PlanningScene & observed_scene,
  const SceneVerificationConfig & config = {});

// Adopt the pose a product already has in the scene, for every desired product the newest
// observation still puts in the same place to within `change_tolerance`.
//
// A product at rest is not still. Gazebo's contact solver moves a resting product by a few
// micrometres per physics step, so the pose the authoritative snapshot carries changes on almost
// every cycle, measured at 1 to 9 um per 0.5 s reconcile period, against a verification
// tolerance of 1e-8 m. Verification has to stay that tight, because what it proves is that the
// write landed exactly; what must not stay tight is the decision to write at all. Without this,
// every cycle finds a difference, every cycle writes a diff, and every diff advances the content
// generation that `evaluate_pregrasp_planning_authority` rejects a plan for, so no plan could
// outlive one reconcile period.
//
// This is the same rule `reconcile_accepted_obstacles` already applies to depth-derived
// obstacles, for the same reason, and it leaves dimensions and identity exact: only a pose the
// observation agrees is unchanged is retained, and only for `restocker/object/...` products,
// whose poses are the sole part of the desired set derived from a live pose stream.
void retain_unmoved_product_geometry(
  std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const moveit_msgs::msg::PlanningScene & observed_scene,
  const SceneVerificationConfig & change_tolerance);

}  // namespace restocker_task_executor
