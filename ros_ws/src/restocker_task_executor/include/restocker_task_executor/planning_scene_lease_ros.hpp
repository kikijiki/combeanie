// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>
#include <restocker_interfaces/srv/acquire_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/release_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/validate_planning_scene_lease.hpp>

#include "restocker_task_executor/planning_scene_lease.hpp"

namespace restocker_task_executor
{

[[nodiscard]] restocker_interfaces::msg::PlanningSceneLease to_ros_message(
  const PlanningSceneLeaseSummary & lease);

[[nodiscard]] restocker_interfaces::msg::PlanningSceneLeaseOperationStatus to_ros_message(
  PlanningSceneLeaseCode code, const std::string & detail);

[[nodiscard]] AcquirePlanningSceneLeaseRequest from_ros_request(
  const restocker_interfaces::srv::AcquirePlanningSceneLease::Request & request);

[[nodiscard]] ReleasePlanningSceneLeaseRequest from_ros_request(
  const restocker_interfaces::srv::ReleasePlanningSceneLease::Request & request);

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::AcquirePlanningSceneLease::Response & response);

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::ValidatePlanningSceneLease::Response & response);

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::ReleasePlanningSceneLease::Response & response);

}  // namespace restocker_task_executor
