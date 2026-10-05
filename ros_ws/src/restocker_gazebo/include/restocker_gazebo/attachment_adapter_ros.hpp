// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include <restocker_interfaces/msg/simulation_attachment_operation_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <restocker_interfaces/srv/get_simulation_attachment_state.hpp>
#include <restocker_interfaces/srv/set_simulation_attachment.hpp>
#include <restocker_interfaces/srv/validate_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_gazebo/attachment_adapter_journal.hpp"
#include "restocker_gazebo/attachment_config.hpp"

namespace restocker_gazebo
{

struct AdapterRequestConversion
{
  AttachmentStatus status;
  std::optional<AdapterMutationRequest> request;
};

[[nodiscard]] AdapterRequestConversion adapter_request_from_ros(
  const restocker_interfaces::srv::SetSimulationAttachment::Request & message);

[[nodiscard]] AdapterAuthorizationResult adapter_authorization_from_ros(
  AttachmentCommand command, std::uint64_t requested_object_id,
  const restocker_interfaces::srv::ValidateTaskReservation::Response & reservation,
  const restocker_interfaces::srv::ValidatePlanningSceneLease::Response & lease,
  const AttachmentBoundaryConfig & config);

[[nodiscard]] restocker_interfaces::msg::SimulationAttachmentOperationStatus to_ros_message(
  const AttachmentStatus & status);

[[nodiscard]] restocker_interfaces::msg::SimulationAttachmentState to_ros_message(
  const AdapterReply & reply,
  const std::string & operation_id);

void populate_ros_response(
  const AdapterReply & reply, const std::string & operation_id,
  restocker_interfaces::srv::SetSimulationAttachment::Response & response);

void populate_ros_response(
  const AdapterReply & reply, const std::string & operation_id,
  restocker_interfaces::srv::GetSimulationAttachmentState::Response & response);

}  // namespace restocker_gazebo
