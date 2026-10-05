// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include <moveit_msgs/msg/constraints.hpp>

#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{

// Converts a MotionGoal path-constraint payload into MoveIt Constraints. Used by MoveItMotionPort
// before free-space plan(); callers must clearPathConstraints afterward (fail closed: never plan
// unconstrained as a fallback).
[[nodiscard]] moveit_msgs::msg::Constraints to_moveit_path_constraints(
  const MotionPathConstraints & source, const std::string & default_frame,
  const std::string & default_link);

}  // namespace restocker_task_executor
