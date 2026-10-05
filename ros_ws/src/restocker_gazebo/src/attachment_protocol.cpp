// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_protocol.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

namespace restocker_gazebo
{
namespace
{

msgs::AttachmentPose encode_pose(const CanonicalPose & pose)
{
  msgs::AttachmentPose result;
  result.set_x(pose.translation[0]);
  result.set_y(pose.translation[1]);
  result.set_z(pose.translation[2]);
  result.set_qx(pose.rotation_xyzw[0]);
  result.set_qy(pose.rotation_xyzw[1]);
  result.set_qz(pose.rotation_xyzw[2]);
  result.set_qw(pose.rotation_xyzw[3]);
  return result;
}

std::optional<CanonicalPose> decode_pose(const msgs::AttachmentPose & pose)
{
  return canonicalize_pose(
    {pose.x(), pose.y(), pose.z()},
    {pose.qx(), pose.qy(), pose.qz(), pose.qw()});
}

msgs::AttachmentIdentityValue encode_identity(const AttachmentIdentity & identity)
{
  msgs::AttachmentIdentityValue result;
  result.set_object_id(identity.object_id);
  result.set_object_source_id(identity.object_source_id);
  result.set_parent_model(identity.parent_model);
  result.set_parent_link(identity.parent_link);
  result.set_child_model(identity.child_model);
  result.set_child_link(identity.child_link);
  return result;
}

AttachmentIdentity decode_identity(const msgs::AttachmentIdentityValue & identity)
{
  return AttachmentIdentity{
    identity.object_id(), identity.object_source_id(), identity.parent_model(),
    identity.parent_link(), identity.child_model(), identity.child_link(),
  };
}

bool valid_identity(const AttachmentIdentity & identity)
{
  return identity.object_id != 0 && !identity.object_source_id.empty() &&
         !identity.parent_model.empty() && !identity.parent_link.empty() &&
         !identity.child_model.empty() && !identity.child_link.empty();
}

std::optional<AttachmentCommand> decode_command_value(msgs::AttachmentCommandValue command)
{
  switch (command) {
    case msgs::ATTACHMENT_COMMAND_ATTACH:
      return AttachmentCommand::kAttach;
    case msgs::ATTACHMENT_COMMAND_DETACH:
      return AttachmentCommand::kDetach;
    case msgs::ATTACHMENT_COMMAND_UNSET:
    default:
      return std::nullopt;
  }
}

std::optional<AttachmentStatusCode> decode_status_code(msgs::AttachmentStatusCodeValue code)
{
  switch (code) {
    case msgs::ATTACHMENT_STATUS_UNSET:
    case msgs::ATTACHMENT_STATUS_PENDING:
    case msgs::ATTACHMENT_STATUS_ATTACHED:
    case msgs::ATTACHMENT_STATUS_DETACHED:
    case msgs::ATTACHMENT_STATUS_INVALID_ARGUMENT:
    case msgs::ATTACHMENT_STATUS_AUTHORIZATION_FAILED:
    case msgs::ATTACHMENT_STATUS_EXTERNAL_INCONSISTENCY:
    case msgs::ATTACHMENT_STATUS_CONFLICT:
    case msgs::ATTACHMENT_STATUS_TOKEN_MISMATCH:
    case msgs::ATTACHMENT_STATUS_IDEMPOTENCY_CONFLICT:
    case msgs::ATTACHMENT_STATUS_RESOURCE_EXHAUSTED:
    case msgs::ATTACHMENT_STATUS_OBJECT_NOT_FOUND:
    case msgs::ATTACHMENT_STATUS_ENTITY_AMBIGUOUS:
    case msgs::ATTACHMENT_STATUS_GRIPPER_NOT_READY:
    case msgs::ATTACHMENT_STATUS_OUT_OF_TOLERANCE:
    case msgs::ATTACHMENT_STATUS_STATE_MISMATCH:
    case msgs::ATTACHMENT_STATUS_SIMULATOR_EPOCH_CHANGED:
    case msgs::ATTACHMENT_STATUS_OUTCOME_UNKNOWN:
    case msgs::ATTACHMENT_STATUS_OPERATION_NOT_FOUND:
    case msgs::ATTACHMENT_STATUS_INTERNAL_ERROR:
      return static_cast<AttachmentStatusCode>(code);
    default:
      return std::nullopt;
  }
}

std::optional<AttachmentPhase> decode_phase(msgs::AttachmentPhaseValue phase)
{
  switch (phase) {
    case msgs::ATTACHMENT_PHASE_UNKNOWN:
    case msgs::ATTACHMENT_PHASE_DETACHED:
    case msgs::ATTACHMENT_PHASE_VALIDATING_ATTACH:
    case msgs::ATTACHMENT_PHASE_APPLYING_ATTACH:
    case msgs::ATTACHMENT_PHASE_VERIFYING_ATTACH:
    case msgs::ATTACHMENT_PHASE_ATTACHED:
    case msgs::ATTACHMENT_PHASE_VALIDATING_DETACH:
    case msgs::ATTACHMENT_PHASE_APPLYING_DETACH:
    case msgs::ATTACHMENT_PHASE_VERIFYING_DETACH:
    case msgs::ATTACHMENT_PHASE_INCONSISTENT:
      return static_cast<AttachmentPhase>(phase);
    default:
      return std::nullopt;
  }
}

std::optional<AttachmentMotionGate> decode_motion_gate(msgs::AttachmentMotionGateValue gate)
{
  switch (gate) {
    case msgs::ATTACHMENT_MOTION_GATE_UNKNOWN:
    case msgs::ATTACHMENT_MOTION_GATE_INHIBITED:
    case msgs::ATTACHMENT_MOTION_GATE_VALID:
      return static_cast<AttachmentMotionGate>(gate);
    default:
      return std::nullopt;
  }
}

std::optional<AttachmentStatus> decode_status(const msgs::AttachmentStatusValue & status)
{
  const auto code = decode_status_code(status.code());
  if (!code) {
    return std::nullopt;
  }
  return AttachmentStatus{*code, status.detail()};
}

std::optional<AttachmentEvidence> decode_evidence(
  const msgs::AttachmentEvidenceValue & evidence)
{
  if (!evidence.has_parent_to_child() || !evidence.has_child_pose_in_world() ||
    evidence.relative_twist_in_parent_size() != 6 || evidence.simulation_time_ns() < 0)
  {
    return std::nullopt;
  }
  const auto parent_to_child = decode_pose(evidence.parent_to_child());
  const auto child_pose_in_world = decode_pose(evidence.child_pose_in_world());
  if (!parent_to_child || !child_pose_in_world) {
    return std::nullopt;
  }
  AttachmentEvidence result;
  result.joint_observed = evidence.joint_observed();
  result.parent_to_child = *parent_to_child;
  result.child_pose_in_world = *child_pose_in_world;
  std::copy_n(
    evidence.relative_twist_in_parent().begin(), result.relative_twist_in_parent.size(),
    result.relative_twist_in_parent.begin());
  result.simulator_iteration = evidence.simulator_iteration();
  result.simulation_time_ns = evidence.simulation_time_ns();
  return result;
}

std::optional<AttachmentFidelity> decode_fidelity(
  const msgs::AttachmentFidelityValue & fidelity)
{
  AttachmentFidelity result;
  result.translation_residual_m = fidelity.translation_residual_m();
  result.rotation_residual_rad = fidelity.rotation_residual_rad();
  result.translation_budget_m = fidelity.translation_budget_m();
  result.claimed_translation_sigma_m = fidelity.claimed_translation_sigma_m();
  result.translation_ceiling_m = fidelity.translation_ceiling_m();
  if (!std::isfinite(result.translation_residual_m) ||
    !std::isfinite(result.rotation_residual_rad) ||
    !std::isfinite(result.translation_budget_m) ||
    !std::isfinite(result.claimed_translation_sigma_m) ||
    !std::isfinite(result.translation_ceiling_m))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<AttachmentRequest> decode_request(
  const msgs::AttachmentRequestValue & request)
{
  const auto command = decode_command_value(request.command());
  if (!command || request.reservation_id() == 0 || !request.has_identity()) {
    return std::nullopt;
  }
  AttachmentRequest result;
  result.command = *command;
  result.reservation_id = request.reservation_id();
  result.identity = decode_identity(request.identity());
  result.has_expected_grasp = request.has_expected_grasp_center_to_child();
  if (!valid_identity(result.identity) ||
    (*command == AttachmentCommand::kAttach && !result.has_expected_grasp) ||
    (*command == AttachmentCommand::kDetach && result.has_expected_grasp))
  {
    return std::nullopt;
  }
  if (result.has_expected_grasp) {
    const auto pose = decode_pose(request.expected_grasp_center_to_child());
    if (!pose) {
      return std::nullopt;
    }
    result.expected_grasp_center_to_child = *pose;
  }
  // Absent is the same as unstated; a wrong-length matrix is a malformed caller, refused rather
  // than read as no claim.
  if (request.expected_pose_covariance_size() != 0) {
    if (static_cast<std::size_t>(request.expected_pose_covariance_size()) !=
      result.expected_pose_covariance.size())
    {
      return std::nullopt;
    }
    std::copy_n(
      request.expected_pose_covariance().begin(), result.expected_pose_covariance.size(),
      result.expected_pose_covariance.begin());
    for (const double entry : result.expected_pose_covariance) {
      if (!std::isfinite(entry)) {
        return std::nullopt;
      }
    }
  }
  return result;
}

std::optional<AttachmentOperationRecord> decode_operation(
  const msgs::AttachmentOperationValue & operation)
{
  if (operation.simulator_epoch().empty() || operation.sequence() == 0 ||
    operation.operation_id().empty() || !operation.has_request() || !operation.has_status())
  {
    return std::nullopt;
  }
  const auto request = decode_request(operation.request());
  const auto phase = decode_phase(operation.phase());
  const auto status = decode_status(operation.status());
  if (!request || !phase || !status) {
    return std::nullopt;
  }
  AttachmentOperationRecord result;
  result.simulator_epoch = operation.simulator_epoch();
  result.sequence = operation.sequence();
  result.operation_id = operation.operation_id();
  result.request = *request;
  result.phase = *phase;
  result.status = *status;
  result.mutation_started = operation.mutation_started();
  result.started_from_inconsistent = operation.started_from_inconsistent();
  if (operation.has_evidence()) {
    result.evidence = decode_evidence(operation.evidence());
    if (!result.evidence) {
      return std::nullopt;
    }
  }
  if (operation.has_fidelity()) {
    result.fidelity = decode_fidelity(operation.fidelity());
    if (!result.fidelity) {
      return std::nullopt;
    }
  }
  return result;
}

std::optional<AttachmentPhysicalState> decode_state(
  const msgs::AttachmentStateValue & state)
{
  if (state.simulator_epoch().empty() || state.sequence() == 0 || !state.has_status()) {
    return std::nullopt;
  }
  const auto phase = decode_phase(state.phase());
  const auto motion_gate = decode_motion_gate(state.motion_gate());
  const auto status = decode_status(state.status());
  if (!phase || !motion_gate || !status) {
    return std::nullopt;
  }
  AttachmentPhysicalState result;
  result.simulator_epoch = state.simulator_epoch();
  result.sequence = state.sequence();
  result.phase = *phase;
  result.motion_gate = *motion_gate;
  result.status = *status;
  if (state.has_attached_identity()) {
    result.attached_identity = decode_identity(state.attached_identity());
    if (!valid_identity(*result.attached_identity)) {
      return std::nullopt;
    }
  }
  if (state.has_evidence()) {
    result.evidence = decode_evidence(state.evidence());
    if (!result.evidence) {
      return std::nullopt;
    }
  }
  if (state.has_fidelity()) {
    result.fidelity = decode_fidelity(state.fidelity());
    if (!result.fidelity) {
      return std::nullopt;
    }
  }
  return result;
}

msgs::AttachmentStatusValue encode_status(const AttachmentStatus & status)
{
  msgs::AttachmentStatusValue result;
  result.set_code(static_cast<msgs::AttachmentStatusCodeValue>(status.code));
  result.set_detail(status.detail);
  return result;
}

msgs::AttachmentRequestValue encode_request(const AttachmentRequest & request)
{
  msgs::AttachmentRequestValue result;
  result.set_command(static_cast<msgs::AttachmentCommandValue>(request.command));
  result.set_reservation_id(request.reservation_id);
  *result.mutable_identity() = encode_identity(request.identity);
  if (request.has_expected_grasp) {
    *result.mutable_expected_grasp_center_to_child() =
      encode_pose(request.expected_grasp_center_to_child);
    for (const double entry : request.expected_pose_covariance) {
      result.add_expected_pose_covariance(entry);
    }
  }
  return result;
}

msgs::AttachmentFidelityValue encode_fidelity(const AttachmentFidelity & fidelity)
{
  msgs::AttachmentFidelityValue result;
  result.set_translation_residual_m(fidelity.translation_residual_m);
  result.set_rotation_residual_rad(fidelity.rotation_residual_rad);
  result.set_translation_budget_m(fidelity.translation_budget_m);
  result.set_claimed_translation_sigma_m(fidelity.claimed_translation_sigma_m);
  result.set_translation_ceiling_m(fidelity.translation_ceiling_m);
  return result;
}

msgs::AttachmentEvidenceValue encode_evidence(const AttachmentEvidence & evidence)
{
  msgs::AttachmentEvidenceValue result;
  result.set_joint_observed(evidence.joint_observed);
  *result.mutable_parent_to_child() = encode_pose(evidence.parent_to_child);
  *result.mutable_child_pose_in_world() = encode_pose(evidence.child_pose_in_world);
  for (const double component : evidence.relative_twist_in_parent) {
    result.add_relative_twist_in_parent(component);
  }
  result.set_simulator_iteration(evidence.simulator_iteration);
  result.set_simulation_time_ns(evidence.simulation_time_ns);
  return result;
}

msgs::AttachmentOperationValue encode_operation(const AttachmentOperationRecord & operation)
{
  msgs::AttachmentOperationValue result;
  result.set_simulator_epoch(operation.simulator_epoch);
  result.set_sequence(operation.sequence);
  result.set_operation_id(operation.operation_id);
  *result.mutable_request() = encode_request(operation.request);
  result.set_phase(static_cast<msgs::AttachmentPhaseValue>(operation.phase));
  *result.mutable_status() = encode_status(operation.status);
  result.set_mutation_started(operation.mutation_started);
  result.set_started_from_inconsistent(operation.started_from_inconsistent);
  if (operation.evidence) {
    *result.mutable_evidence() = encode_evidence(*operation.evidence);
  }
  if (operation.fidelity) {
    *result.mutable_fidelity() = encode_fidelity(*operation.fidelity);
  }
  return result;
}

msgs::AttachmentStateValue encode_state(const AttachmentPhysicalState & state)
{
  msgs::AttachmentStateValue result;
  result.set_simulator_epoch(state.simulator_epoch);
  result.set_sequence(state.sequence);
  result.set_phase(static_cast<msgs::AttachmentPhaseValue>(state.phase));
  result.set_motion_gate(static_cast<msgs::AttachmentMotionGateValue>(state.motion_gate));
  *result.mutable_status() = encode_status(state.status);
  if (state.attached_identity) {
    *result.mutable_attached_identity() = encode_identity(*state.attached_identity);
  }
  if (state.evidence) {
    *result.mutable_evidence() = encode_evidence(*state.evidence);
  }
  if (state.fidelity) {
    *result.mutable_fidelity() = encode_fidelity(*state.fidelity);
  }
  return result;
}

}  // namespace

msgs::SetAttachmentRequest encode_command(
  std::string expected_simulator_epoch,
  std::string operation_id,
  const AttachmentRequest & request)
{
  msgs::SetAttachmentRequest result;
  result.set_expected_simulator_epoch(std::move(expected_simulator_epoch));
  result.set_operation_id(std::move(operation_id));
  *result.mutable_request() = encode_request(request);
  return result;
}

msgs::QueryAttachmentRequest encode_query(
  std::string expected_simulator_epoch,
  std::string operation_id)
{
  msgs::QueryAttachmentRequest result;
  result.set_expected_simulator_epoch(std::move(expected_simulator_epoch));
  result.set_operation_id(std::move(operation_id));
  return result;
}

std::optional<DecodedAttachmentCommand> decode_command(
  const msgs::SetAttachmentRequest & message,
  AttachmentStatus & error)
{
  error = {};
  if (message.expected_simulator_epoch().empty() || message.operation_id().empty() ||
    !message.has_request() || !message.request().has_identity())
  {
    error = {AttachmentStatusCode::kInvalidArgument, "transport attachment command is incomplete"};
    return std::nullopt;
  }

  const auto request = decode_request(message.request());
  if (!request) {
    error = {AttachmentStatusCode::kInvalidArgument, "transport attachment request is invalid"};
    return std::nullopt;
  }
  return DecodedAttachmentCommand{
    message.expected_simulator_epoch(), message.operation_id(), *request,
  };
}

std::optional<DecodedAttachmentQuery> decode_query(
  const msgs::QueryAttachmentRequest & message,
  AttachmentStatus & error)
{
  error = {};
  if (message.expected_simulator_epoch().empty()) {
    error = {AttachmentStatusCode::kInvalidArgument, "query simulator epoch is required"};
    return std::nullopt;
  }
  return DecodedAttachmentQuery{message.expected_simulator_epoch(), message.operation_id()};
}

msgs::AttachmentReply encode_reply(const AttachmentJournalReply & reply)
{
  msgs::AttachmentReply result;
  *result.mutable_status() = encode_status(reply.status);
  result.set_replayed(reply.replayed);
  if (reply.operation) {
    *result.mutable_operation() = encode_operation(*reply.operation);
  }
  *result.mutable_state() = encode_state(reply.state);
  return result;
}

std::optional<AttachmentJournalReply> decode_reply(
  const msgs::AttachmentReply & message,
  AttachmentStatus & error)
{
  error = {};
  if (!message.has_status() || !message.has_state()) {
    error = {AttachmentStatusCode::kInvalidArgument, "transport attachment reply is incomplete"};
    return std::nullopt;
  }
  const auto status = decode_status(message.status());
  const auto state = decode_state(message.state());
  if (!status || !state || status->code == AttachmentStatusCode::kUnset) {
    error = {AttachmentStatusCode::kInvalidArgument, "transport attachment reply is invalid"};
    return std::nullopt;
  }
  AttachmentJournalReply result;
  result.status = *status;
  result.replayed = message.replayed();
  result.state = *state;
  if (message.has_operation()) {
    result.operation = decode_operation(message.operation());
    if (!result.operation) {
      error = {
        AttachmentStatusCode::kInvalidArgument,
        "transport attachment operation record is invalid",
      };
      return std::nullopt;
    }
  }
  return result;
}

}  // namespace restocker_gazebo
