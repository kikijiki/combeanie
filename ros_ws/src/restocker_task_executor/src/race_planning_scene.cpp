// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/race_planning_scene.hpp"

#include <exception>
#include <memory>
#include <string>

namespace restocker_task_executor
{

RaceSceneBuild make_race_scene(
  const moveit::core::RobotModelConstPtr & model, const moveit_msgs::msg::PlanningScene & snapshot)
{
  RaceSceneBuild build;
  if (!model) {
    build.refusal = "no robot model";
    return build;
  }
  // setPlanningSceneDiffMsg leaves padding, scale and the matrix at the fresh scene's defaults
  // when the message omits them — zero padding and the SRDF-only matrix, a scene less strict
  // than move_group's. That is the gap this refuses, not a default to fall back on.
  if (snapshot.link_padding.empty() || snapshot.link_scale.empty()) {
    build.refusal = "the snapshot carries no link padding/scale";
    return build;
  }
  if (snapshot.allowed_collision_matrix.entry_names.empty()) {
    build.refusal = "the snapshot carries no allowed-collision matrix";
    return build;
  }
  try {
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    if (!scene->setPlanningSceneDiffMsg(snapshot)) {
      build.refusal = "MoveIt refused to apply the snapshot";
      return build;
    }
    build.scene = std::move(scene);
  } catch (const std::exception & error) {
    build.refusal = std::string("applying the snapshot raised: ") + error.what();
  }
  return build;
}

}  // namespace restocker_task_executor
