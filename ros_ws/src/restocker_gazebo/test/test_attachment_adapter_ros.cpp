// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_operation_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <restocker_interfaces/msg/task_reservation.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_gazebo/attachment_adapter_ros.hpp"

namespace restocker_gazebo
{
namespace
{

using GetState = restocker_interfaces::srv::GetSimulationAttachmentState;
using LeaseStatus = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;
using RosState = restocker_interfaces::msg::SimulationAttachmentState;
using RosStatus = restocker_interfaces::msg::SimulationAttachmentOperationStatus;
using SetAttachment = restocker_interfaces::srv::SetSimulationAttachment;
using TaskReservation = restocker_interfaces::msg::TaskReservation;
using ValidateLease = restocker_interfaces::srv::ValidatePlanningSceneLease;
using ValidateReservation = restocker_interfaces::srv::ValidateTaskReservation;
using WorldStatus = restocker_interfaces::msg::WorldStateOperationStatus;

AttachmentBoundaryConfig boundary_config()
{
  AttachmentBoundaryConfig config;
  config.robot_model_name = "restocker";
  config.parent_link = "gripper";
  AttachmentProductConfig product;
  product.source_object_id = "sim:stock_can_01";
  product.model_name = "stock_can_01";
  product.child_link = "product_body";
  config.products_by_source_id.emplace(product.source_object_id, product);
  return config;
}

ValidateReservation::Response reservation_response(
  std::uint8_t stage = TaskReservation::STAGE_RESERVED)
{
  ValidateReservation::Response response;
  response.status.code = WorldStatus::OK;
  response.has_reservation = true;
  response.reservation.reservation_id = 41;
  response.reservation.object_id = 7;
  response.reservation.object_source_id = "sim:stock_can_01";
  response.reservation.stage = stage;
  return response;
}

ValidateLease::Response lease_response()
{
  ValidateLease::Response response;
  response.status.code = LeaseStatus::VALID;
  response.has_lease = true;
  response.lease.lease_id = 17;
  response.lease.phase = response.lease.PHASE_HELD;
  return response;
}

SetAttachment::Request attach_request()
{
  SetAttachment::Request request;
  request.command = request.COMMAND_ATTACH;
  request.operation_id = "attach-1";
  request.object_id = 7;
  request.reservation_token = "reservation-secret";
  request.planning_scene_lease_token = "lease-secret";
  request.expected_grasp_center_to_child.position.x = 0.01;
  request.expected_grasp_center_to_child.orientation.w = 1.0;
  return request;
}

AttachmentIdentity identity()
{
  return AttachmentIdentity{
    7, "sim:stock_can_01", "restocker", "gripper", "stock_can_01", "product_body"};
}

AttachmentPhysicalState physical_state()
{
  AttachmentPhysicalState state;
  state.simulator_epoch = "sim-epoch";
  state.sequence = 9;
  state.phase = AttachmentPhase::kAttached;
  state.motion_gate = AttachmentMotionGate::kValid;
  state.status = AttachmentStatus{AttachmentStatusCode::kAttached, "physically attached"};
  state.attached_identity = identity();
  AttachmentEvidence evidence;
  evidence.joint_observed = true;
  evidence.parent_to_child.translation = {0.1, 0.2, 0.3};
  evidence.parent_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  evidence.child_pose_in_world.translation = {1.1, 1.2, 1.3};
  evidence.child_pose_in_world.rotation_xyzw = {0.0, 0.0, 0.0, 1.0};
  evidence.relative_twist_in_parent = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  evidence.simulator_iteration = 123;
  evidence.simulation_time_ns = 2'000'000'003;
  state.evidence = evidence;
  return state;
}

TEST(AttachmentAdapterRosTest, ConvertsCanonicalAttachAndStrictDetachRequests)
{
  const auto attach = adapter_request_from_ros(attach_request());
  ASSERT_TRUE(attach.request);
  EXPECT_EQ(attach.status.code, AttachmentStatusCode::kPending);
  EXPECT_EQ(attach.request->command, AttachmentCommand::kAttach);
  EXPECT_EQ(attach.request->object_id, 7U);
  EXPECT_TRUE(attach.request->has_expected_grasp);
  EXPECT_DOUBLE_EQ(attach.request->expected_grasp_center_to_child.translation[0], 0.01);

  auto detach_message = attach_request();
  detach_message.command = detach_message.COMMAND_DETACH;
  detach_message.expected_grasp_center_to_child = geometry_msgs::msg::Pose();
  const auto detach = adapter_request_from_ros(detach_message);
  ASSERT_TRUE(detach.request);
  EXPECT_EQ(detach.request->command, AttachmentCommand::kDetach);
  EXPECT_FALSE(detach.request->has_expected_grasp);

  detach_message.expected_grasp_center_to_child.position.x = 0.1;
  EXPECT_EQ(
    adapter_request_from_ros(detach_message).status.code,
    AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentAdapterRosTest, RejectsMalformedRequestsBeforeAuthorization)
{
  auto request = attach_request();
  request.operation_id.clear();
  EXPECT_EQ(
    adapter_request_from_ros(request).status.code, AttachmentStatusCode::kInvalidArgument);
  request = attach_request();
  request.expected_grasp_center_to_child.orientation.w = 2.0;
  EXPECT_EQ(
    adapter_request_from_ros(request).status.code, AttachmentStatusCode::kInvalidArgument);
  request = attach_request();
  request.command = request.COMMAND_UNSET;
  EXPECT_EQ(
    adapter_request_from_ros(request).status.code, AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentAdapterRosTest, ResolvesAuthorizedIdentityFromImmutableConfig)
{
  const auto authorization = adapter_authorization_from_ros(
    AttachmentCommand::kAttach, 7, reservation_response(), lease_response(), boundary_config());
  ASSERT_TRUE(authorization.evidence);
  EXPECT_EQ(authorization.status.code, AttachmentStatusCode::kPending);
  EXPECT_EQ(authorization.evidence->reservation_id, 41U);
  EXPECT_EQ(authorization.evidence->lease_id, 17U);
  EXPECT_EQ(authorization.evidence->reservation_stage, AdapterReservationStage::kReserved);
  EXPECT_EQ(authorization.evidence->resolved_identity, identity());

  const auto detach = adapter_authorization_from_ros(
    AttachmentCommand::kDetach, 7, reservation_response(TaskReservation::STAGE_ATTACHED),
    lease_response(), boundary_config());
  ASSERT_TRUE(detach.evidence);
  EXPECT_EQ(detach.evidence->reservation_stage, AdapterReservationStage::kAttached);
}

TEST(AttachmentAdapterRosTest, RejectsTokenStageLeaseAndIdentityMismatches)
{
  auto reservation = reservation_response();
  auto lease = lease_response();
  reservation.status.code = WorldStatus::TOKEN_MISMATCH;
  EXPECT_EQ(
    adapter_authorization_from_ros(
      AttachmentCommand::kAttach, 7, reservation, lease, boundary_config()).status.code,
    AttachmentStatusCode::kTokenMismatch);

  reservation = reservation_response(TaskReservation::STAGE_ATTACHED);
  EXPECT_EQ(
    adapter_authorization_from_ros(
      AttachmentCommand::kAttach, 7, reservation, lease, boundary_config()).status.code,
    AttachmentStatusCode::kAuthorizationFailed);

  reservation = reservation_response();
  lease.lease.phase = lease.lease.PHASE_DRAINING;
  EXPECT_EQ(
    adapter_authorization_from_ros(
      AttachmentCommand::kAttach, 7, reservation, lease, boundary_config()).status.code,
    AttachmentStatusCode::kAuthorizationFailed);

  lease = lease_response();
  reservation.reservation.object_source_id = "sim:unmapped";
  EXPECT_EQ(
    adapter_authorization_from_ros(
      AttachmentCommand::kAttach, 7, reservation, lease, boundary_config()).status.code,
    AttachmentStatusCode::kAuthorizationFailed);
}

TEST(AttachmentAdapterRosTest, MapsEveryStableStatusCode)
{
  const std::array<std::pair<AttachmentStatusCode, std::uint16_t>, 20> cases{{
    {AttachmentStatusCode::kUnset, RosStatus::UNSET},
    {AttachmentStatusCode::kPending, RosStatus::PENDING},
    {AttachmentStatusCode::kAttached, RosStatus::ATTACHED},
    {AttachmentStatusCode::kDetached, RosStatus::DETACHED},
    {AttachmentStatusCode::kInvalidArgument, RosStatus::INVALID_ARGUMENT},
    {AttachmentStatusCode::kAuthorizationFailed, RosStatus::AUTHORIZATION_FAILED},
    {AttachmentStatusCode::kExternalInconsistency, RosStatus::EXTERNAL_INCONSISTENCY},
    {AttachmentStatusCode::kConflict, RosStatus::CONFLICT},
    {AttachmentStatusCode::kTokenMismatch, RosStatus::TOKEN_MISMATCH},
    {AttachmentStatusCode::kIdempotencyConflict, RosStatus::IDEMPOTENCY_CONFLICT},
    {AttachmentStatusCode::kResourceExhausted, RosStatus::RESOURCE_EXHAUSTED},
    {AttachmentStatusCode::kObjectNotFound, RosStatus::OBJECT_NOT_FOUND},
    {AttachmentStatusCode::kEntityAmbiguous, RosStatus::ENTITY_AMBIGUOUS},
    {AttachmentStatusCode::kGripperNotReady, RosStatus::GRIPPER_NOT_READY},
    {AttachmentStatusCode::kOutOfTolerance, RosStatus::OUT_OF_TOLERANCE},
    {AttachmentStatusCode::kStateMismatch, RosStatus::STATE_MISMATCH},
    {AttachmentStatusCode::kSimulatorEpochChanged, RosStatus::SIMULATOR_EPOCH_CHANGED},
    {AttachmentStatusCode::kOutcomeUnknown, RosStatus::OUTCOME_UNKNOWN},
    {AttachmentStatusCode::kOperationNotFound, RosStatus::OPERATION_NOT_FOUND},
    {AttachmentStatusCode::kInternalError, RosStatus::INTERNAL_ERROR},
  }};
  for (const auto & [input, expected] : cases) {
    EXPECT_EQ(to_ros_message(AttachmentStatus{input, "detail"}).code, expected);
  }
}

TEST(AttachmentAdapterRosTest, SerializesTokenFreePhysicalEvidence)
{
  AttachmentRequest transport;
  transport.command = AttachmentCommand::kAttach;
  transport.reservation_id = 41;
  transport.identity = identity();
  AdapterReply reply{
    AttachmentStatus{AttachmentStatusCode::kAttached, "verified"}, false,
    AdapterAction::kNone, transport, physical_state()};

  const auto message = to_ros_message(reply, "attach-1");
  EXPECT_EQ(message.simulator_epoch, "sim-epoch");
  EXPECT_EQ(message.sequence, 9U);
  EXPECT_EQ(message.operation_id, "attach-1");
  EXPECT_EQ(message.reservation_id, 41U);
  EXPECT_EQ(message.object_id, 7U);
  EXPECT_EQ(message.phase, RosState::PHASE_ATTACHED);
  EXPECT_EQ(message.motion_gate, RosState::MOTION_GATE_VALID);
  EXPECT_TRUE(message.joint_observed);
  EXPECT_DOUBLE_EQ(message.parent_to_child.position.y, 0.2);
  EXPECT_DOUBLE_EQ(message.child_pose_in_world.position.z, 1.3);
  EXPECT_DOUBLE_EQ(message.relative_twist_in_parent.angular.z, 6.0);
  EXPECT_EQ(message.simulator_iteration, 123U);
  EXPECT_EQ(message.observed_at.sec, 2);
  EXPECT_EQ(message.observed_at.nanosec, 3U);
  EXPECT_EQ(message.status.code, RosStatus::ATTACHED);

  SetAttachment::Response set_response;
  populate_ros_response(reply, "attach-1", set_response);
  EXPECT_TRUE(set_response.has_state);
  EXPECT_EQ(set_response.state.object_source_id, "sim:stock_can_01");

  GetState::Response query_response;
  populate_ros_response(reply, "attach-1", query_response);
  EXPECT_TRUE(query_response.operation_found);
  EXPECT_TRUE(query_response.has_state);
  EXPECT_EQ(query_response.simulator_epoch, "sim-epoch");
}

TEST(AttachmentAdapterRosTest, RejectsUnrepresentableSimulationTime)
{
  auto state = physical_state();
  state.evidence->simulation_time_ns =
    (static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max()) + 1) *
    1'000'000'000;
  const AdapterReply reply{
    AttachmentStatus{AttachmentStatusCode::kAttached, "verified"}, false,
    AdapterAction::kNone, std::nullopt, state};
  EXPECT_THROW(static_cast<void>(to_ros_message(reply, "attach-1")), std::overflow_error);
}

TEST(AttachmentAdapterRosTest, RepresentsMissingStateAndUnknownOperationExplicitly)
{
  const AdapterReply reply{
    AttachmentStatus{AttachmentStatusCode::kOperationNotFound, "missing"}, false,
    AdapterAction::kNone, std::nullopt, std::nullopt};
  GetState::Response response;
  populate_ros_response(reply, "missing-op", response);
  EXPECT_FALSE(response.operation_found);
  EXPECT_FALSE(response.has_state);
  EXPECT_TRUE(response.simulator_epoch.empty());
  EXPECT_EQ(response.status.code, RosStatus::OPERATION_NOT_FOUND);
}

}  // namespace
}  // namespace restocker_gazebo
