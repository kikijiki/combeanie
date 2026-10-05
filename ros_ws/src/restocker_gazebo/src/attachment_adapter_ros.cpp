// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_adapter_ros.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>
#include <restocker_interfaces/msg/task_reservation.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

namespace restocker_gazebo
{
namespace
{

using RosState = restocker_interfaces::msg::SimulationAttachmentState;
using RosStatus = restocker_interfaces::msg::SimulationAttachmentOperationStatus;

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

[[nodiscard]] bool default_pose(const geometry_msgs::msg::Pose & pose)
{
  return pose.position.x == 0.0 && pose.position.y == 0.0 && pose.position.z == 0.0 &&
         pose.orientation.x == 0.0 && pose.orientation.y == 0.0 &&
         pose.orientation.z == 0.0 && pose.orientation.w == 1.0;
}

[[nodiscard]] std::optional<CanonicalPose> pose_from_ros(const geometry_msgs::msg::Pose & pose)
{
  return canonicalize_pose(
    {pose.position.x, pose.position.y, pose.position.z},
    {pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w});
}

[[nodiscard]] std::uint16_t ros_code(AttachmentStatusCode code)
{
  switch (code) {
    case AttachmentStatusCode::kUnset: return RosStatus::UNSET;
    case AttachmentStatusCode::kPending: return RosStatus::PENDING;
    case AttachmentStatusCode::kAttached: return RosStatus::ATTACHED;
    case AttachmentStatusCode::kDetached: return RosStatus::DETACHED;
    case AttachmentStatusCode::kInvalidArgument: return RosStatus::INVALID_ARGUMENT;
    case AttachmentStatusCode::kAuthorizationFailed: return RosStatus::AUTHORIZATION_FAILED;
    case AttachmentStatusCode::kExternalInconsistency: return RosStatus::EXTERNAL_INCONSISTENCY;
    case AttachmentStatusCode::kConflict: return RosStatus::CONFLICT;
    case AttachmentStatusCode::kTokenMismatch: return RosStatus::TOKEN_MISMATCH;
    case AttachmentStatusCode::kIdempotencyConflict: return RosStatus::IDEMPOTENCY_CONFLICT;
    case AttachmentStatusCode::kResourceExhausted: return RosStatus::RESOURCE_EXHAUSTED;
    case AttachmentStatusCode::kObjectNotFound: return RosStatus::OBJECT_NOT_FOUND;
    case AttachmentStatusCode::kEntityAmbiguous: return RosStatus::ENTITY_AMBIGUOUS;
    case AttachmentStatusCode::kGripperNotReady: return RosStatus::GRIPPER_NOT_READY;
    case AttachmentStatusCode::kOutOfTolerance: return RosStatus::OUT_OF_TOLERANCE;
    case AttachmentStatusCode::kStateMismatch: return RosStatus::STATE_MISMATCH;
    case AttachmentStatusCode::kSimulatorEpochChanged: return RosStatus::SIMULATOR_EPOCH_CHANGED;
    case AttachmentStatusCode::kOutcomeUnknown: return RosStatus::OUTCOME_UNKNOWN;
    case AttachmentStatusCode::kOperationNotFound: return RosStatus::OPERATION_NOT_FOUND;
    case AttachmentStatusCode::kInternalError: return RosStatus::INTERNAL_ERROR;
  }
  throw std::invalid_argument("unknown attachment status code");
}

[[nodiscard]] std::uint8_t ros_phase(AttachmentPhase phase)
{
  switch (phase) {
    case AttachmentPhase::kUnknown: return RosState::PHASE_UNKNOWN;
    case AttachmentPhase::kDetached: return RosState::PHASE_DETACHED;
    case AttachmentPhase::kValidatingAttach: return RosState::PHASE_VALIDATING_ATTACH;
    case AttachmentPhase::kApplyingAttach: return RosState::PHASE_APPLYING_ATTACH;
    case AttachmentPhase::kVerifyingAttach: return RosState::PHASE_VERIFYING_ATTACH;
    case AttachmentPhase::kAttached: return RosState::PHASE_ATTACHED;
    case AttachmentPhase::kValidatingDetach: return RosState::PHASE_VALIDATING_DETACH;
    case AttachmentPhase::kApplyingDetach: return RosState::PHASE_APPLYING_DETACH;
    case AttachmentPhase::kVerifyingDetach: return RosState::PHASE_VERIFYING_DETACH;
    case AttachmentPhase::kInconsistent: return RosState::PHASE_INCONSISTENT;
  }
  throw std::invalid_argument("unknown attachment phase");
}

[[nodiscard]] std::uint8_t ros_gate(AttachmentMotionGate gate)
{
  switch (gate) {
    case AttachmentMotionGate::kUnknown: return RosState::MOTION_GATE_UNKNOWN;
    case AttachmentMotionGate::kInhibited: return RosState::MOTION_GATE_INHIBITED;
    case AttachmentMotionGate::kValid: return RosState::MOTION_GATE_VALID;
  }
  throw std::invalid_argument("unknown attachment motion gate");
}

void populate_pose(const CanonicalPose & source, geometry_msgs::msg::Pose & destination)
{
  destination.position.x = source.translation[0];
  destination.position.y = source.translation[1];
  destination.position.z = source.translation[2];
  destination.orientation.x = source.rotation_xyzw[0];
  destination.orientation.y = source.rotation_xyzw[1];
  destination.orientation.z = source.rotation_xyzw[2];
  destination.orientation.w = source.rotation_xyzw[3];
}

[[nodiscard]] AdapterAuthorizationResult authorization_failure(
  AttachmentStatusCode code,
  std::string detail)
{
  return AdapterAuthorizationResult{status(code, std::move(detail)), std::nullopt};
}

}  // namespace

AdapterRequestConversion adapter_request_from_ros(
  const restocker_interfaces::srv::SetSimulationAttachment::Request & message)
{
  AdapterMutationRequest result;
  if (message.command == message.COMMAND_ATTACH) {
    result.command = AttachmentCommand::kAttach;
    const auto pose = pose_from_ros(message.expected_grasp_center_to_child);
    if (!pose) {
      return AdapterRequestConversion{
        status(AttachmentStatusCode::kInvalidArgument, "attach pose is not finite and rigid"),
        std::nullopt};
    }
    result.has_expected_grasp = true;
    result.expected_grasp_center_to_child = *pose;
    std::copy(
      message.expected_pose_covariance.begin(), message.expected_pose_covariance.end(),
      result.expected_pose_covariance.begin());
  } else if (message.command == message.COMMAND_DETACH) {
    if (!default_pose(message.expected_grasp_center_to_child)) {
      return AdapterRequestConversion{
        status(AttachmentStatusCode::kInvalidArgument, "detach pose must be default-constructed"),
        std::nullopt};
    }
    result.command = AttachmentCommand::kDetach;
  } else {
    return AdapterRequestConversion{
      status(AttachmentStatusCode::kInvalidArgument, "attachment command is unknown"),
      std::nullopt};
  }
  result.object_id = message.object_id;
  result.reservation_token = message.reservation_token;
  result.planning_scene_lease_token = message.planning_scene_lease_token;
  if (message.operation_id.empty() || result.object_id == 0 ||
    result.reservation_token.empty() || result.planning_scene_lease_token.empty())
  {
    return AdapterRequestConversion{
      status(AttachmentStatusCode::kInvalidArgument, "attachment request is incomplete"),
      std::nullopt};
  }
  return AdapterRequestConversion{
    status(AttachmentStatusCode::kPending, "attachment request converted"), result};
}

AdapterAuthorizationResult adapter_authorization_from_ros(
  AttachmentCommand command, std::uint64_t requested_object_id,
  const restocker_interfaces::srv::ValidateTaskReservation::Response & reservation,
  const restocker_interfaces::srv::ValidatePlanningSceneLease::Response & lease,
  const AttachmentBoundaryConfig & config)
{
  using WorldStatus = restocker_interfaces::msg::WorldStateOperationStatus;
  using LeaseStatus = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;
  using Reservation = restocker_interfaces::msg::TaskReservation;
  using Lease = restocker_interfaces::msg::PlanningSceneLease;

  if (reservation.status.code == WorldStatus::TOKEN_MISMATCH ||
    lease.status.code == LeaseStatus::TOKEN_MISMATCH)
  {
    return authorization_failure(AttachmentStatusCode::kTokenMismatch, "capability token mismatch");
  }
  if (reservation.status.code != WorldStatus::OK || !reservation.has_reservation ||
    lease.status.code != LeaseStatus::VALID || !lease.has_lease)
  {
    // The refusal reaches the coordinator as an unprovable attachment outcome several transitions
    // later, so it must name which of the two capabilities the two authorities rejected; otherwise
    // it cannot be told from a token mismatch.
    return authorization_failure(
      AttachmentStatusCode::kAuthorizationFailed,
      "capability validation was not successful (reservation code " +
      std::to_string(reservation.status.code) + " present=" +
      (reservation.has_reservation ? "true" : "false") + " detail=\"" +
      reservation.status.detail + "\"; lease code " + std::to_string(lease.status.code) +
      " present=" + (lease.has_lease ? "true" : "false") + " detail=\"" +
      lease.status.detail + "\")");
  }
  const std::uint8_t expected_stage = command == AttachmentCommand::kAttach ?
    Reservation::STAGE_RESERVED : Reservation::STAGE_ATTACHED;
  if ((command != AttachmentCommand::kAttach && command != AttachmentCommand::kDetach) ||
    reservation.reservation.object_id != requested_object_id ||
    reservation.reservation.stage != expected_stage ||
    reservation.reservation.reservation_id == 0 ||
    reservation.reservation.object_source_id.empty() ||
    lease.lease.phase != Lease::PHASE_HELD || lease.lease.lease_id == 0)
  {
    return authorization_failure(
      AttachmentStatusCode::kAuthorizationFailed,
      "capabilities do not authorize the requested object and operation stage");
  }
  const auto product = config.products_by_source_id.find(
    reservation.reservation.object_source_id);
  if (product == config.products_by_source_id.end()) {
    return authorization_failure(
      AttachmentStatusCode::kAuthorizationFailed,
      "reserved source identity is outside the immutable scenario mapping");
  }

  AdapterAuthorizationEvidence evidence;
  evidence.reservation_id = reservation.reservation.reservation_id;
  evidence.object_id = requested_object_id;
  evidence.object_source_id = reservation.reservation.object_source_id;
  evidence.reservation_stage = command == AttachmentCommand::kAttach ?
    AdapterReservationStage::kReserved : AdapterReservationStage::kAttached;
  evidence.lease_id = lease.lease.lease_id;
  evidence.lease_phase = AdapterLeasePhase::kHeld;
  evidence.resolved_identity = AttachmentIdentity{
    requested_object_id, evidence.object_source_id, config.robot_model_name,
    config.parent_link, product->second.model_name, product->second.child_link};
  return AdapterAuthorizationResult{
    status(AttachmentStatusCode::kPending, "capabilities authorize attachment operation"),
    evidence};
}

restocker_interfaces::msg::SimulationAttachmentOperationStatus to_ros_message(
  const AttachmentStatus & status_value)
{
  RosStatus result;
  result.code = ros_code(status_value.code);
  result.detail = status_value.detail;
  return result;
}

restocker_interfaces::msg::SimulationAttachmentState to_ros_message(
  const AdapterReply & reply,
  const std::string & operation_id)
{
  if (!reply.state) {
    throw std::invalid_argument("adapter reply has no physical state");
  }
  const AttachmentPhysicalState & source = *reply.state;
  RosState result;
  result.simulator_epoch = source.simulator_epoch;
  result.sequence = source.sequence;
  result.operation_id = operation_id;
  result.phase = ros_phase(source.phase);
  result.motion_gate = ros_gate(source.motion_gate);
  result.joint_observed = source.evidence && source.evidence->joint_observed;
  result.status = to_ros_message(reply.status);
  if (reply.transport_request) {
    result.reservation_id = reply.transport_request->reservation_id;
    result.object_id = reply.transport_request->identity.object_id;
    result.object_source_id = reply.transport_request->identity.object_source_id;
    result.parent_model = reply.transport_request->identity.parent_model;
    result.parent_link = reply.transport_request->identity.parent_link;
    result.child_model = reply.transport_request->identity.child_model;
    result.child_link = reply.transport_request->identity.child_link;
  } else if (source.attached_identity) {
    result.object_id = source.attached_identity->object_id;
    result.object_source_id = source.attached_identity->object_source_id;
    result.parent_model = source.attached_identity->parent_model;
    result.parent_link = source.attached_identity->parent_link;
    result.child_model = source.attached_identity->child_model;
    result.child_link = source.attached_identity->child_link;
  }
  if (source.fidelity) {
    result.has_fidelity = true;
    result.fidelity_translation_residual_m = source.fidelity->translation_residual_m;
    result.fidelity_rotation_residual_rad = source.fidelity->rotation_residual_rad;
    result.fidelity_translation_budget_m = source.fidelity->translation_budget_m;
    result.fidelity_claimed_translation_sigma_m = source.fidelity->claimed_translation_sigma_m;
    result.fidelity_translation_ceiling_m = source.fidelity->translation_ceiling_m;
  }
  if (source.evidence) {
    populate_pose(source.evidence->parent_to_child, result.parent_to_child);
    populate_pose(source.evidence->child_pose_in_world, result.child_pose_in_world);
    result.relative_twist_in_parent.linear.x = source.evidence->relative_twist_in_parent[0];
    result.relative_twist_in_parent.linear.y = source.evidence->relative_twist_in_parent[1];
    result.relative_twist_in_parent.linear.z = source.evidence->relative_twist_in_parent[2];
    result.relative_twist_in_parent.angular.x = source.evidence->relative_twist_in_parent[3];
    result.relative_twist_in_parent.angular.y = source.evidence->relative_twist_in_parent[4];
    result.relative_twist_in_parent.angular.z = source.evidence->relative_twist_in_parent[5];
    result.simulator_iteration = source.evidence->simulator_iteration;
    const std::int64_t seconds = source.evidence->simulation_time_ns / 1'000'000'000;
    const std::int64_t nanoseconds = source.evidence->simulation_time_ns % 1'000'000'000;
    if (seconds > std::numeric_limits<std::int32_t>::max()) {
      throw std::overflow_error("simulation time exceeds the ROS time representation");
    }
    result.observed_at.sec = static_cast<std::int32_t>(seconds);
    result.observed_at.nanosec = static_cast<std::uint32_t>(nanoseconds);
  }
  return result;
}

void populate_ros_response(
  const AdapterReply & reply, const std::string & operation_id,
  restocker_interfaces::srv::SetSimulationAttachment::Response & response)
{
  response.status = to_ros_message(reply.status);
  response.has_state = reply.state.has_value();
  if (reply.state) {
    response.state = to_ros_message(reply, operation_id);
  }
}

void populate_ros_response(
  const AdapterReply & reply, const std::string & operation_id,
  restocker_interfaces::srv::GetSimulationAttachmentState::Response & response)
{
  response.status = to_ros_message(reply.status);
  response.operation_found = !operation_id.empty() &&
    reply.status.code != AttachmentStatusCode::kOperationNotFound;
  response.has_state = reply.state.has_value();
  if (reply.state) {
    response.simulator_epoch = reply.state->simulator_epoch;
    response.state = to_ros_message(reply, operation_id);
  }
}

}  // namespace restocker_gazebo
