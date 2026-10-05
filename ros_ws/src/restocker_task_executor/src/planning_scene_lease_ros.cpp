// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/planning_scene_lease_ros.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

#include <rclcpp/time.hpp>

namespace restocker_task_executor
{
namespace
{

using RosLease = restocker_interfaces::msg::PlanningSceneLease;
using RosStatus = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;

[[nodiscard]] std::uint8_t to_ros_phase(PlanningSceneLeasePhase phase)
{
  switch (phase) {
    case PlanningSceneLeasePhase::None:
      return RosLease::PHASE_NONE;
    case PlanningSceneLeasePhase::Draining:
      return RosLease::PHASE_DRAINING;
    case PlanningSceneLeasePhase::Held:
      return RosLease::PHASE_HELD;
    case PlanningSceneLeasePhase::Releasing:
      return RosLease::PHASE_RELEASING;
  }
  throw std::invalid_argument("unknown planning-scene lease phase");
}

[[nodiscard]] std::uint16_t to_ros_code(PlanningSceneLeaseCode code)
{
  switch (code) {
    case PlanningSceneLeaseCode::Unset:
      return RosStatus::UNSET;
    case PlanningSceneLeaseCode::Draining:
      return RosStatus::DRAINING;
    case PlanningSceneLeaseCode::Granted:
      return RosStatus::GRANTED;
    case PlanningSceneLeaseCode::Valid:
      return RosStatus::VALID;
    case PlanningSceneLeaseCode::ReleaseAccepted:
      return RosStatus::RELEASE_ACCEPTED;
    case PlanningSceneLeaseCode::InvalidArgument:
      return RosStatus::INVALID_ARGUMENT;
    case PlanningSceneLeaseCode::Conflict:
      return RosStatus::CONFLICT;
    case PlanningSceneLeaseCode::TokenMismatch:
      return RosStatus::TOKEN_MISMATCH;
    case PlanningSceneLeaseCode::IdempotencyConflict:
      return RosStatus::IDEMPOTENCY_CONFLICT;
    case PlanningSceneLeaseCode::ResourceExhausted:
      return RosStatus::RESOURCE_EXHAUSTED;
    case PlanningSceneLeaseCode::InternalError:
      return RosStatus::INTERNAL_ERROR;
  }
  throw std::invalid_argument("unknown planning-scene lease result code");
}

template<typename Response>
void populate_common_response(const PlanningSceneLeaseReply & reply, Response & response)
{
  response.status = to_ros_message(reply.code, reply.detail);
  response.has_lease = reply.lease.has_value();
  if (reply.lease) {
    response.lease = to_ros_message(*reply.lease);
  }
}

}  // namespace

restocker_interfaces::msg::PlanningSceneLease to_ros_message(
  const PlanningSceneLeaseSummary & lease)
{
  RosLease message;
  message.lease_id = lease.lease_id;
  message.acquisition_operation_id = lease.acquisition_operation_id;
  message.minimum_applied_revision = lease.minimum_applied_revision;
  message.granted_applied_revision = lease.granted_applied_revision;
  message.verification_epoch = lease.verification_epoch;
  message.acquired_at = static_cast<builtin_interfaces::msg::Time>(
    rclcpp::Time(lease.acquired_at_ns, RCL_ROS_TIME));
  message.phase = to_ros_phase(lease.phase);
  return message;
}

restocker_interfaces::msg::PlanningSceneLeaseOperationStatus to_ros_message(
  PlanningSceneLeaseCode code, const std::string & detail)
{
  RosStatus message;
  message.code = to_ros_code(code);
  message.detail = detail;
  return message;
}

AcquirePlanningSceneLeaseRequest from_ros_request(
  const restocker_interfaces::srv::AcquirePlanningSceneLease::Request & request)
{
  return AcquirePlanningSceneLeaseRequest{
    request.operation_id, request.minimum_applied_revision};
}

ReleasePlanningSceneLeaseRequest from_ros_request(
  const restocker_interfaces::srv::ReleasePlanningSceneLease::Request & request)
{
  return ReleasePlanningSceneLeaseRequest{
    request.operation_id, request.token, request.required_semantic_revision};
}

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::AcquirePlanningSceneLease::Response & response)
{
  populate_common_response(reply, response);
  if (reply.code == PlanningSceneLeaseCode::Granted) {
    response.token = reply.token;
  }
}

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::ValidatePlanningSceneLease::Response & response)
{
  populate_common_response(reply, response);
}

void populate_ros_response(
  const PlanningSceneLeaseReply & reply,
  restocker_interfaces::srv::ReleasePlanningSceneLease::Response & response)
{
  populate_common_response(reply, response);
}

}  // namespace restocker_task_executor
