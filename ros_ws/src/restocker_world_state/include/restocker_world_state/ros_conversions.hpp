// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/robot_execution_state.hpp>
#include <restocker_interfaces/msg/robot_telemetry.hpp>
#include <restocker_interfaces/msg/shelf_lane.hpp>
#include <restocker_interfaces/msg/task_reservation.hpp>
#include <restocker_interfaces/msg/tracked_object.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>
#include <restocker_interfaces/msg/world_state_snapshot.hpp>
#include <restocker_interfaces/srv/checkpoint_task_state.hpp>
#include <restocker_interfaces/srv/commit_reserved_attachment.hpp>
#include <restocker_interfaces/srv/commit_reserved_detachment.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>

#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{

struct SnapshotMessageOptions
{
  std::string planning_frame{"world"};
  rclcpp::Time snapshot_time{std::int64_t{0}, RCL_ROS_TIME};
  bool include_removed{false};
  bool include_events{false};
};

[[nodiscard]] Result<ObjectObservation> object_observation_from_message(
  const restocker_interfaces::msg::ObjectObservation & message, double minimum_confidence);

[[nodiscard]] Result<LaneObservation> lane_observation_from_message(
  const restocker_interfaces::msg::LaneObservation & message, double minimum_confidence);

[[nodiscard]] Result<RobotTelemetryObservation> robot_telemetry_from_message(
  const restocker_interfaces::msg::RobotTelemetry & message);

[[nodiscard]] Result<WorldStateSnapshot> snapshot_from_message(
  const restocker_interfaces::msg::WorldStateSnapshot & message,
  const std::string & expected_planning_frame);

[[nodiscard]] Result<TaskReservation> task_reservation_from_message(
  const restocker_interfaces::msg::TaskReservation & message, Revision maximum_revision);

[[nodiscard]] Result<TrackedObject> tracked_object_from_message(
  const restocker_interfaces::msg::TrackedObject & message, Revision maximum_revision);

[[nodiscard]] Result<ShelfLane> shelf_lane_from_message(
  const restocker_interfaces::msg::ShelfLane & message, Revision maximum_revision);

[[nodiscard]] Result<RobotExecutionState> robot_execution_state_from_message(
  const restocker_interfaces::msg::RobotExecutionState & message, Revision maximum_revision);

[[nodiscard]] Result<ReserveTaskRequest> reserve_task_request_from_message(
  const restocker_interfaces::srv::ReserveTask::Request & message);

[[nodiscard]] Result<ReservedTaskCheckpoint> checkpoint_request_from_message(
  const restocker_interfaces::srv::CheckpointTaskState::Request & message,
  const rclcpp::Time & accepted_at);

// Historical inhibited success retains its original receipt but never emits ordinary OK.
[[nodiscard]] restocker_interfaces::srv::CommitReservedAttachment::Response
attachment_result_to_message(const ReservedAttachmentResult & result, Revision current_revision);

[[nodiscard]] Result<ReservedAttachmentRequest> attachment_request_from_message(
  const restocker_interfaces::srv::CommitReservedAttachment::Request & message);

[[nodiscard]] Result<ReservedDetachmentRequest> detachment_request_from_message(
  const restocker_interfaces::srv::CommitReservedDetachment::Request & message);

[[nodiscard]] Result<ReleaseReservationRequest> release_request_from_message(
  const restocker_interfaces::srv::ReleaseTaskReservation::Request & message);

[[nodiscard]] restocker_interfaces::msg::WorldStateOperationStatus operation_status_ok();

[[nodiscard]] restocker_interfaces::msg::WorldStateOperationStatus operation_status_from_error(
  const WorldStateError & error);

[[nodiscard]] restocker_interfaces::msg::TaskReservation task_reservation_to_message(
  const TaskReservation & reservation);

[[nodiscard]] restocker_interfaces::msg::TrackedObject tracked_object_to_message(
  const TrackedObject & object);

[[nodiscard]] restocker_interfaces::msg::ShelfLane shelf_lane_to_message(
  const ShelfLane & lane);

[[nodiscard]] restocker_interfaces::msg::RobotExecutionState robot_execution_state_to_message(
  const RobotExecutionState & robot);

[[nodiscard]] restocker_interfaces::msg::WorldStateSnapshot snapshot_to_message(
  const WorldStateSnapshot & snapshot, const SnapshotMessageOptions & options);

}  // namespace restocker_world_state
