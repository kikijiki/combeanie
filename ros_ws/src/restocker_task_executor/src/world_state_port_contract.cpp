// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/world_state_port_contract.hpp"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <restocker_interfaces/msg/robot_execution_state.hpp>
#include <restocker_interfaces/msg/task_reservation.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>
#include <restocker_interfaces/msg/world_state_snapshot.hpp>

#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_world_state::FaultState;
using restocker_world_state::LaneId;
using restocker_world_state::ReservationOutcome;
using restocker_world_state::ReservationStage;
using restocker_world_state::TaskPhase;
using Status = restocker_interfaces::msg::WorldStateOperationStatus;

constexpr std::size_t kMaximumOperationIdLength = 128;

class CapabilityFingerprint
{
public:
  void add_unsigned(std::uint64_t value) noexcept
  {
    for (std::size_t index = 0; index < sizeof(value); ++index) {
      value_ ^= static_cast<std::uint8_t>((value >> (index * 8U)) & 0xffU);
      value_ *= 1099511628211ULL;
    }
  }

  void add_string(const std::string & value) noexcept
  {
    add_unsigned(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char byte : value) {
      value_ ^= byte;
      value_ *= 1099511628211ULL;
    }
  }

  [[nodiscard]] std::uint64_t value() const noexcept {return value_ == 0U ? 1U : value_;}

private:
  std::uint64_t value_{14695981039346656037ULL};
};

template<typename T>
[[nodiscard]] WorldStatePortResult<T> failure(
  WorldStatePortErrorCode code, std::string detail,
  std::uint16_t remote_status = Status::UNSET)
{
  return WorldStatePortResult<T>::failure(
    WorldStatePortError{code, remote_status, std::move(detail)});
}

[[nodiscard]] bool valid_operation_id(const std::string & value)
{
  return !value.empty() && value.size() <= kMaximumOperationIdLength;
}

[[nodiscard]] bool known_status(std::uint16_t code)
{
  switch (code) {
    case Status::OK:
    case Status::INVALID_ARGUMENT:
    case Status::NOT_FOUND:
    case Status::REVISION_CONFLICT:
    case Status::RESERVATION_CONFLICT:
    case Status::TOKEN_MISMATCH:
    case Status::PREDICATE_FAILED:
    case Status::INVALID_TRANSITION:
    case Status::IDEMPOTENCY_CONFLICT:
    case Status::RESOURCE_EXHAUSTED:
    case Status::INTERNAL_ERROR:
      return true;
    case Status::UNSET:
    default:
      return false;
  }
}

[[nodiscard]] bool has_reservation_payload(
  const restocker_interfaces::msg::TaskReservation & message)
{
  return message.reservation_id != 0 || !message.request_id.empty() || message.object_id != 0 ||
         !message.object_source_id.empty() || message.product_class != 0 || message.has_sku ||
         !message.sku.empty() || message.has_source_lane || !message.source_lane_id.empty() ||
         !message.destination_lane_id.empty() || message.stage != 0 ||
         message.placed_in_destination || message.created_at.sec != 0 ||
         message.created_at.nanosec != 0 || message.created_revision != 0 ||
         message.admitted_robot_telemetry_revision != 0 || message.revision != 0;
}

[[nodiscard]] std::optional<LaneId> containing_lane(
  const restocker_world_state::WorldStateSnapshot & snapshot,
  restocker_world_state::ObjectId object_id, bool & duplicate)
{
  std::optional<LaneId> result;
  duplicate = false;
  for (const auto &[lane_id, lane] : snapshot.lanes) {
    if (std::find(lane.contents.begin(), lane.contents.end(), object_id) == lane.contents.end()) {
      continue;
    }
    if (result) {
      duplicate = true;
      return std::nullopt;
    }
    result = lane_id;
  }
  return result;
}

[[nodiscard]] std::optional<std::string> reservation_request_error(
  const restocker_interfaces::srv::ReserveTask::Request & request,
  const restocker_world_state::WorldStateSnapshot & snapshot)
{
  if (!valid_operation_id(request.request_id) || request.selected_snapshot_revision == 0 ||
    request.selected_snapshot_revision != snapshot.revision || request.object_id == 0 ||
    request.object_revision == 0 || request.destination_lane_id.empty() ||
    request.destination_lane_revision == 0 || snapshot.active_reservation ||
    snapshot.robot.held_object)
  {
    return "reservation request does not identify one unreserved snapshot";
  }

  const restocker_world_state::ObjectId object_id{request.object_id};
  const LaneId destination_id{request.destination_lane_id};
  const auto object = snapshot.objects.find(object_id);
  const auto destination = snapshot.lanes.find(destination_id);
  if (object == snapshot.objects.end() || destination == snapshot.lanes.end() ||
    object->second.id != object_id || destination->second.id != destination_id ||
    object->second.revision != request.object_revision ||
    destination->second.revision != request.destination_lane_revision ||
    request.object_revision > snapshot.revision ||
    request.destination_lane_revision > snapshot.revision)
  {
    return "reservation request entities or revisions disagree with the snapshot";
  }

  bool duplicate_membership = false;
  const auto source_lane = containing_lane(snapshot, object_id, duplicate_membership);
  if (duplicate_membership || request.has_source_lane != source_lane.has_value()) {
    return "reservation request source membership is ambiguous or stale";
  }
  if (!source_lane) {
    if (!request.source_lane_id.empty() || request.source_lane_revision != 0) {
      return "absent source lane contains a diagnostic payload";
    }
    return std::nullopt;
  }

  const auto source = snapshot.lanes.find(*source_lane);
  if (request.source_lane_id != source_lane->value || request.source_lane_revision == 0 ||
    source == snapshot.lanes.end() || source->second.id != *source_lane ||
    source->second.revision != request.source_lane_revision ||
    request.source_lane_revision > snapshot.revision || *source_lane == destination_id)
  {
    return "reservation request source lane disagrees with the snapshot";
  }
  return std::nullopt;
}

[[nodiscard]] bool valid_capability(const TaskReservationCapability & capability)
{
  const auto & reservation = capability.reservation;
  const bool known_product_class =
    reservation.product_class == restocker_world_state::ProductClass::Can ||
    reservation.product_class == restocker_world_state::ProductClass::SmallBottle ||
    reservation.product_class == restocker_world_state::ProductClass::LargeBottle;
  const bool known_stage = reservation.stage == ReservationStage::Reserved ||
    reservation.stage == ReservationStage::Attached ||
    reservation.stage == ReservationStage::Detached;
  return !capability.token.empty() && capability.world_revision != 0 &&
         reservation.reservation_id != 0 && !reservation.request_id.empty() &&
         static_cast<bool>(reservation.object_id) && !reservation.object_source_id.empty() &&
         known_product_class && (!reservation.sku || !reservation.sku->empty()) &&
         (!reservation.destination_expected_sku ||
         !reservation.destination_expected_sku->empty()) &&
         (!reservation.source_lane || !reservation.source_lane->value.empty()) &&
         !reservation.destination_lane.value.empty() &&
         reservation.source_lane != std::optional<LaneId>(reservation.destination_lane) &&
         known_stage && reservation.created_at.get_clock_type() == RCL_ROS_TIME &&
         reservation.created_at.nanoseconds() >= 0 &&
         reservation.created_revision != 0 &&
         reservation.admitted_robot_telemetry_revision != 0 &&
         reservation.admitted_robot_telemetry_revision <= reservation.created_revision &&
         reservation.created_revision <= reservation.revision &&
         reservation.revision <= capability.world_revision &&
         (reservation.stage == ReservationStage::Detached || !reservation.placed_in_destination);
}

[[nodiscard]] bool same_reservation(
  const restocker_world_state::TaskReservation & lhs,
  const restocker_world_state::TaskReservation & rhs)
{
  return lhs.reservation_id == rhs.reservation_id && lhs.request_id == rhs.request_id &&
         lhs.object_id == rhs.object_id && lhs.object_source_id == rhs.object_source_id &&
         lhs.product_class == rhs.product_class && lhs.sku == rhs.sku &&
         lhs.source_lane == rhs.source_lane && lhs.destination_lane == rhs.destination_lane &&
         lhs.stage == rhs.stage && lhs.placed_in_destination == rhs.placed_in_destination &&
         lhs.created_at == rhs.created_at &&
         lhs.created_revision == rhs.created_revision &&
         lhs.admitted_robot_telemetry_revision == rhs.admitted_robot_telemetry_revision &&
         lhs.revision == rhs.revision &&
         lhs.destination_expected_product_class == rhs.destination_expected_product_class &&
         lhs.destination_expected_sku == rhs.destination_expected_sku;
}

[[nodiscard]] bool has_execution_proof_payload(
  const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response & response)
{
  return response.has_proof || has_reservation_payload(response.reservation) ||
         !response.planning_frame.empty() ||
         response.object != restocker_interfaces::msg::TrackedObject{} ||
         response.destination_lane != restocker_interfaces::msg::ShelfLane{} ||
         response.has_source_lane ||
         response.source_lane != restocker_interfaces::msg::ShelfLane{} ||
         response.robot != restocker_interfaces::msg::RobotExecutionState{};
}

[[nodiscard]] bool contains_object(
  const restocker_world_state::ShelfLane & lane,
  restocker_world_state::ObjectId object_id)
{
  return std::find(lane.contents.begin(), lane.contents.end(), object_id) != lane.contents.end();
}

[[nodiscard]] std::optional<std::string> execution_proof_predicate_error(
  const restocker_world_state::ExecutionWorldAuthorityProof & proof)
{
  const auto & reservation = proof.reservation;
  const auto & object = proof.object;
  const auto & destination = proof.destination_lane;
  if (object.id != reservation.object_id ||
    object.source_object_id != reservation.object_source_id ||
    object.product_class != reservation.product_class || object.sku != reservation.sku ||
    object.tracking_state != restocker_world_state::TrackingState::Tracked ||
    destination.id != reservation.destination_lane ||
    // Captured at grant, not read from the live lane: lane intent may change while the transfer
    // runs, and this proof must not contradict the store's own captured-policy revalidation.
    !restocker_world_state::policy_accepts_object(
      reservation.destination_expected_product_class,
      reservation.destination_expected_sku, object) ||
    destination.evidence_revision == 0U ||
    proof.source_lane.has_value() != reservation.source_lane.has_value() ||
    (proof.source_lane && proof.source_lane->id != *reservation.source_lane) ||
    proof.robot.telemetry_revision < reservation.admitted_robot_telemetry_revision ||
    !restocker_world_state::valid_active_task_semantics(
      proof.robot.task_phase, proof.robot.fault_state))
  {
    return "execution authority entities or robot lineage violate the reservation";
  }

  const bool in_source = proof.source_lane && contains_object(*proof.source_lane, object.id);
  const bool in_destination = contains_object(destination, object.id);
  if (in_source && in_destination) {
    return "execution authority object appears in both returned lanes";
  }
  // Independent re-check using only what the proof itself carries: identity and lineage,
  // membership, robot coupling, obstruction, and (once placement is claimed) that any identities
  // the destination evidence supplies still account for the target. A producer that supplies no
  // lane identities at all (the wrist depth camera) leaves the catalogue-dependent growth proof
  // to the store, which enforces it authoritatively at commit and at every Detached
  // revalidation; this re-check cannot re-derive pitch from the payload and does not pretend to.
  // Destination room is not checked here: it is a depth measured against the product diameter
  // and the reserved free depth, which the granting authority owns.
  const auto & observed = destination.observed_source_object_ids;
  const bool destination_names_target =
    std::find(observed.begin(), observed.end(), object.source_object_id) != observed.end();
  const bool destination_identity_contradicts_placement =
    !observed.empty() && !destination_names_target;
  switch (reservation.stage) {
    case ReservationStage::Reserved:
      if (reservation.placed_in_destination || object.grasp_state !=
        restocker_world_state::GraspState::Free || proof.robot.held_object ||
        in_source != reservation.source_lane.has_value() || in_destination ||
        destination.obstructed)
      {
        return "reserved execution authority predicate failed";
      }
      break;
    case ReservationStage::Attached:
      if (reservation.placed_in_destination || object.grasp_state !=
        restocker_world_state::GraspState::Attached || proof.robot.held_object != object.id ||
        in_source || in_destination || destination.obstructed)
      {
        return "attached execution authority predicate failed";
      }
      break;
    case ReservationStage::Detached:
      if (object.grasp_state != restocker_world_state::GraspState::Free ||
        proof.robot.held_object || in_source ||
        in_destination != reservation.placed_in_destination ||
        (reservation.placed_in_destination &&
        (destination.obstructed || destination_identity_contradicts_placement)))
      {
        return "detached execution authority predicate failed";
      }
      break;
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::uint8_t> outcome_to_message(ReservationOutcome outcome)
{
  using Request = restocker_interfaces::srv::ReleaseTaskReservation::Request;
  switch (outcome) {
    case ReservationOutcome::Succeeded:
      return Request::OUTCOME_SUCCEEDED;
    case ReservationOutcome::Canceled:
      return Request::OUTCOME_CANCELED;
    case ReservationOutcome::FailedSafe:
      return Request::OUTCOME_FAILED_SAFE;
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::uint8_t> task_phase_to_message(TaskPhase phase)
{
  using Robot = restocker_interfaces::msg::RobotExecutionState;
  switch (phase) {
    case TaskPhase::Idle:
      return Robot::TASK_IDLE;
    case TaskPhase::ValidatingScene:
      return Robot::TASK_VALIDATING_SCENE;
    case TaskPhase::Executing:
      return Robot::TASK_EXECUTING;
    case TaskPhase::Recovering:
      return Robot::TASK_RECOVERING;
    case TaskPhase::Fault:
      return Robot::TASK_FAULT;
    case TaskPhase::RequestingOperator:
      return Robot::TASK_REQUESTING_OPERATOR;
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::uint8_t> fault_state_to_message(FaultState state)
{
  using Robot = restocker_interfaces::msg::RobotExecutionState;
  switch (state) {
    case FaultState::None:
      return Robot::FAULT_NONE;
    case FaultState::Recoverable:
      return Robot::FAULT_RECOVERABLE;
    case FaultState::NonRecoverable:
      return Robot::FAULT_NON_RECOVERABLE;
    case FaultState::ExternalInconsistency:
      return Robot::FAULT_EXTERNAL_INCONSISTENCY;
  }
  return std::nullopt;
}

[[nodiscard]] bool valid_release_disposition(
  const TaskReservationCapability & capability, ReservationOutcome outcome)
{
  return (capability.reservation.stage == ReservationStage::Reserved &&
         (outcome == ReservationOutcome::Canceled ||
         outcome == ReservationOutcome::FailedSafe)) ||
         (capability.reservation.stage == ReservationStage::Detached &&
         capability.reservation.placed_in_destination &&
         outcome == ReservationOutcome::Succeeded) ||
         (capability.reservation.stage == ReservationStage::Detached &&
         !capability.reservation.placed_in_destination &&
         outcome == ReservationOutcome::FailedSafe);
}

[[nodiscard]] bool valid_terminal_state(
  ReservationOutcome outcome, TaskPhase phase, FaultState fault)
{
  const bool normal_terminal =
    outcome == ReservationOutcome::Succeeded || outcome == ReservationOutcome::Canceled;
  return (normal_terminal && phase == TaskPhase::Idle && fault == FaultState::None) ||
         (!normal_terminal &&
         (phase == TaskPhase::Fault || phase == TaskPhase::RequestingOperator) &&
         fault != FaultState::None);
}

}  // namespace

std::optional<std::uint64_t> fingerprint_task_reservation_capability(
  const TaskReservationCapability & capability)
{
  if (!valid_capability(capability)) {
    return std::nullopt;
  }
  const auto & reservation = capability.reservation;
  CapabilityFingerprint fingerprint;
  fingerprint.add_string("task-reservation-capability:v1");
  fingerprint.add_string(capability.token);
  fingerprint.add_unsigned(capability.world_revision);
  fingerprint.add_unsigned(reservation.reservation_id);
  fingerprint.add_string(reservation.request_id);
  fingerprint.add_unsigned(reservation.object_id.value);
  fingerprint.add_string(reservation.object_source_id);
  fingerprint.add_unsigned(static_cast<std::uint8_t>(reservation.product_class));
  fingerprint.add_unsigned(reservation.sku.has_value() ? 1U : 0U);
  if (reservation.sku) {
    fingerprint.add_string(*reservation.sku);
  }
  fingerprint.add_unsigned(reservation.source_lane.has_value() ? 1U : 0U);
  if (reservation.source_lane) {
    fingerprint.add_string(reservation.source_lane->value);
  }
  fingerprint.add_string(reservation.destination_lane.value);
  fingerprint.add_unsigned(static_cast<std::uint8_t>(reservation.stage));
  fingerprint.add_unsigned(reservation.placed_in_destination ? 1U : 0U);
  fingerprint.add_unsigned(static_cast<std::uint64_t>(reservation.created_at.nanoseconds()));
  fingerprint.add_unsigned(
    static_cast<std::uint32_t>(reservation.created_at.get_clock_type()));
  fingerprint.add_unsigned(reservation.created_revision);
  fingerprint.add_unsigned(reservation.admitted_robot_telemetry_revision);
  fingerprint.add_unsigned(reservation.revision);
  fingerprint.add_unsigned(
    static_cast<std::uint8_t>(reservation.destination_expected_product_class));
  fingerprint.add_unsigned(reservation.destination_expected_sku.has_value() ? 1U : 0U);
  if (reservation.destination_expected_sku) {
    fingerprint.add_string(*reservation.destination_expected_sku);
  }
  return fingerprint.value();
}

WorldStatePortResult<restocker_interfaces::srv::ReserveTask::Request> make_reserve_task_request(
  const restocker_world_state::WorldStateSnapshot & snapshot, const SelectedTaskPair & selection,
  const std::string & request_id)
{
  // Each guard reports its own clause: the coordinator's terminal must name the precondition
  // that fired, or the receipt cannot distinguish a stale selection from a stale world view
  // (Card 010 attempt 9 was undiagnosable behind the combined message).
  if (!valid_operation_id(request_id) || !selection.object_id || selection.lane_id.value.empty()) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "operation id or selection identity is missing");
  }
  if (snapshot.revision == 0) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput, "snapshot revision is zero");
  }
  if (selection.snapshot_revision != snapshot.revision) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "selection is stamped at snapshot revision " +
      std::to_string(selection.snapshot_revision) + " but latest is " +
      std::to_string(snapshot.revision));
  }
  if (snapshot.robot.revision == 0 || snapshot.robot.revision > snapshot.revision) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "robot telemetry revision " + std::to_string(snapshot.robot.revision) +
      " is inconsistent with snapshot revision " + std::to_string(snapshot.revision));
  }
  if (snapshot.active_reservation) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput, "snapshot still carries an active reservation");
  }
  if (snapshot.robot.held_object) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "snapshot still reports the robot holding an object");
  }

  const auto object = snapshot.objects.find(selection.object_id);
  const auto destination = snapshot.lanes.find(selection.lane_id);
  if (object == snapshot.objects.end() || destination == snapshot.lanes.end() ||
    object->second.id != selection.object_id || destination->second.id != selection.lane_id ||
    object->second.revision != selection.object_revision ||
    destination->second.revision != selection.lane_revision || object->second.revision == 0 ||
    destination->second.revision == 0 || object->second.revision > snapshot.revision ||
    destination->second.revision > snapshot.revision)
  {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "selected identities or revisions disagree with the snapshot");
  }

  bool duplicate_membership = false;
  const auto source_lane = containing_lane(snapshot, selection.object_id, duplicate_membership);
  if (duplicate_membership || source_lane == std::optional<LaneId>(selection.lane_id)) {
    return failure<restocker_interfaces::srv::ReserveTask::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "selected object has ambiguous or destination membership");
  }

  restocker_interfaces::srv::ReserveTask::Request request;
  request.request_id = request_id;
  request.selected_snapshot_revision = selection.snapshot_revision;
  request.object_id = selection.object_id.value;
  request.object_revision = selection.object_revision;
  request.has_source_lane = source_lane.has_value();
  if (source_lane) {
    const auto source = snapshot.lanes.find(*source_lane);
    if (source == snapshot.lanes.end() || source->second.id != *source_lane ||
      source->second.revision == 0 || source->second.revision > snapshot.revision)
    {
      return failure<restocker_interfaces::srv::ReserveTask::Request>(
        WorldStatePortErrorCode::kInvalidInput, "source lane is invalid");
    }
    request.source_lane_id = source_lane->value;
    request.source_lane_revision = source->second.revision;
  }
  request.destination_lane_id = selection.lane_id.value;
  request.destination_lane_revision = selection.lane_revision;
  return WorldStatePortResult<restocker_interfaces::srv::ReserveTask::Request>::success(
    std::move(request));
}

WorldStatePortResult<TaskReservationCapability> validate_reserve_task_response(
  const restocker_interfaces::srv::ReserveTask::Request & request,
  const restocker_world_state::WorldStateSnapshot & selected_snapshot,
  const restocker_interfaces::srv::ReserveTask::Response & response)
{
  if (const auto error = reservation_request_error(request, selected_snapshot)) {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kInvalidInput, *error);
  }
  if (!known_status(response.status.code)) {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kMalformedResponse,
      "reservation response has unknown status code",
      response.status.code);
  }
  if (response.status.code != Status::OK) {
    if (!response.token.empty() || has_reservation_payload(response.reservation)) {
      return failure<TaskReservationCapability>(
        WorldStatePortErrorCode::kMalformedResponse,
        "rejected reservation response contains a capability payload", response.status.code);
    }
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kRemoteRejected,
      response.status.detail, response.status.code);
  }
  if (!response.status.detail.empty() ||
    response.world_revision <= request.selected_snapshot_revision || response.token.empty())
  {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kMalformedResponse,
      "successful reservation response has invalid status, revision, or token",
      response.status.code);
  }

  const auto object =
    selected_snapshot.objects.find(restocker_world_state::ObjectId{request.object_id});
  if (object == selected_snapshot.objects.end()) {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kInvalidInput,
      "reservation request object is absent from the selected snapshot");
  }
  auto converted = restocker_world_state::task_reservation_from_message(
    response.reservation,
    response.world_revision);
  if (!converted || converted.value().revision != response.world_revision ||
    converted.value().created_revision != response.world_revision ||
    converted.value().admitted_robot_telemetry_revision <
    selected_snapshot.robot.telemetry_revision)
  {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kMalformedResponse,
      converted ? "reservation response revisions are inconsistent" : converted.error().detail,
      response.status.code);
  }

  const auto & reservation = converted.value();
  const std::optional<LaneId> expected_source =
    request.has_source_lane ? std::optional<LaneId>(LaneId{request.source_lane_id}) :
    std::nullopt;
  if (reservation.request_id != request.request_id ||
    reservation.object_id.value != request.object_id ||
    reservation.object_source_id != object->second.source_object_id ||
    reservation.product_class != object->second.product_class ||
    reservation.sku != object->second.sku || reservation.source_lane != expected_source ||
    reservation.destination_lane != LaneId{request.destination_lane_id} ||
    reservation.stage != ReservationStage::Reserved || reservation.placed_in_destination)
  {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "reservation response does not describe the selected task", response.status.code);
  }
  // The store captured the destination's policy under the same lane-revision fence that
  // admitted the reservation, so the captured policy must equal the selected snapshot's lane
  // policy; anything else means the response describes a grant this selection never made.
  const auto destination = selected_snapshot.lanes.find(reservation.destination_lane);
  if (destination == selected_snapshot.lanes.end() ||
    reservation.destination_expected_product_class != destination->second.expected_product_class ||
    reservation.destination_expected_sku != destination->second.expected_sku)
  {
    return failure<TaskReservationCapability>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "reservation response captured destination policy differs from the selected snapshot",
      response.status.code);
  }
  return WorldStatePortResult<TaskReservationCapability>::success(
    TaskReservationCapability{
      response.token, std::move(converted.value()), response.world_revision});
}

WorldStatePortResult<restocker_interfaces::srv::ValidateTaskReservation::Request>
make_validate_task_reservation_request(const TaskReservationCapability & capability)
{
  if (!valid_capability(capability)) {
    return failure<restocker_interfaces::srv::ValidateTaskReservation::Request>(
      WorldStatePortErrorCode::kInvalidInput, "reservation capability is invalid");
  }
  restocker_interfaces::srv::ValidateTaskReservation::Request request;
  request.token = capability.token;
  return WorldStatePortResult<
    restocker_interfaces::srv::ValidateTaskReservation::Request>::success(std::move(request));
}

WorldStatePortResult<TaskReservationValidationProof> validate_task_reservation_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ValidateTaskReservation::Response & response)
{
  if (!valid_capability(capability)) {
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kInvalidInput, "reservation capability is invalid");
  }
  if (!known_status(response.status.code)) {
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "reservation validation response has unknown status code", response.status.code);
  }
  if (response.status.code != Status::OK) {
    if (response.has_reservation || has_reservation_payload(response.reservation)) {
      return failure<TaskReservationValidationProof>(
        WorldStatePortErrorCode::kMalformedResponse,
        "rejected validation response contains reservation data", response.status.code);
    }
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kRemoteRejected, response.status.detail, response.status.code);
  }
  if (!response.status.detail.empty() || !response.has_reservation ||
    response.world_revision < capability.world_revision)
  {
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "successful validation response omits a causal reservation", response.status.code);
  }

  auto converted = restocker_world_state::task_reservation_from_message(
    response.reservation, response.world_revision);
  if (!converted) {
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kMalformedResponse, converted.error().detail,
      response.status.code);
  }
  const auto & expected = capability.reservation;
  const auto & actual = converted.value();
  if (actual.reservation_id != expected.reservation_id ||
    actual.request_id != expected.request_id || actual.object_id != expected.object_id ||
    actual.object_source_id != expected.object_source_id ||
    actual.product_class != expected.product_class || actual.sku != expected.sku ||
    actual.source_lane != expected.source_lane ||
    actual.destination_lane != expected.destination_lane || actual.stage != expected.stage ||
    actual.placed_in_destination != expected.placed_in_destination ||
    actual.created_at != expected.created_at ||
    actual.created_revision != expected.created_revision ||
    actual.admitted_robot_telemetry_revision != expected.admitted_robot_telemetry_revision ||
    actual.destination_expected_product_class != expected.destination_expected_product_class ||
    actual.destination_expected_sku != expected.destination_expected_sku ||
    actual.revision < expected.revision)
  {
    return failure<TaskReservationValidationProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "validated reservation differs from the retained capability", response.status.code);
  }
  return WorldStatePortResult<TaskReservationValidationProof>::success(
    TaskReservationValidationProof{response.world_revision, std::move(converted.value())});
}

WorldStatePortResult<restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request>
make_validate_execution_world_authority_request(const TaskReservationCapability & capability)
{
  if (!valid_capability(capability)) {
    return failure<restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request>(
      WorldStatePortErrorCode::kInvalidInput, "reservation capability is invalid");
  }
  restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request request;
  request.token = capability.token;
  request.expected_reservation_id = capability.reservation.reservation_id;
  request.expected_reservation_revision = capability.reservation.revision;
  request.expected_object_id = capability.reservation.object_id.value;
  request.expected_destination_lane_id = capability.reservation.destination_lane.value;
  return WorldStatePortResult<
    restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request>::success(
    std::move(request));
}

WorldStatePortResult<restocker_world_state::ExecutionWorldAuthorityProof>
validate_execution_world_authority_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response & response,
  const std::string & expected_planning_frame)
{
  if (!valid_capability(capability) || expected_planning_frame.empty()) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kInvalidInput,
      "reservation capability and expected planning frame are required");
  }
  if (!known_status(response.status.code)) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "execution authority response has unknown status code", response.status.code);
  }
  if (response.status.code != Status::OK) {
    if (has_execution_proof_payload(response)) {
      return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
        WorldStatePortErrorCode::kMalformedResponse,
        "rejected execution authority response contains proof data", response.status.code);
    }
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kRemoteRejected, response.status.detail, response.status.code);
  }
  if (!response.status.detail.empty() || !response.has_proof || response.world_revision == 0U ||
    response.world_revision < capability.world_revision)
  {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "successful execution authority response omits a causal proof", response.status.code);
  }
  if (response.planning_frame != expected_planning_frame) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "execution authority planning frame differs from the configured planning frame",
      response.status.code);
  }
  if (!response.has_source_lane &&
    response.source_lane != restocker_interfaces::msg::ShelfLane{})
  {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "absent source lane contains diagnostic payload", response.status.code);
  }
  if (response.has_source_lane != capability.reservation.source_lane.has_value()) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "execution authority source-lane identity differs from the retained capability",
      response.status.code);
  }

  auto reservation = restocker_world_state::task_reservation_from_message(
    response.reservation, response.world_revision);
  auto object = restocker_world_state::tracked_object_from_message(
    response.object, response.world_revision);
  auto destination = restocker_world_state::shelf_lane_from_message(
    response.destination_lane, response.world_revision);
  auto robot = restocker_world_state::robot_execution_state_from_message(
    response.robot, response.world_revision);
  if (!reservation || !object || !destination || !robot) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kMalformedResponse,
      "execution authority response contains a malformed proof entity",
      response.status.code);
  }

  const auto & expected = capability.reservation;
  if (!same_reservation(reservation.value(), expected)) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "execution authority reservation differs from the retained capability",
      response.status.code);
  }
  std::optional<restocker_world_state::ShelfLane> source;
  if (expected.source_lane) {
    auto converted = restocker_world_state::shelf_lane_from_message(
      response.source_lane, response.world_revision);
    if (!converted) {
      return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
        WorldStatePortErrorCode::kMalformedResponse,
        "execution authority response contains a malformed source lane", response.status.code);
    }
    source = std::move(converted.value());
  }
  restocker_world_state::ExecutionWorldAuthorityProof proof{
    response.world_revision, response.planning_frame, std::move(reservation.value()),
    std::move(object.value()), std::move(destination.value()), std::move(source),
    std::move(robot.value())};
  if (const auto error = execution_proof_predicate_error(proof)) {
    return failure<restocker_world_state::ExecutionWorldAuthorityProof>(
      WorldStatePortErrorCode::kIdentityMismatch, *error, response.status.code);
  }
  return WorldStatePortResult<restocker_world_state::ExecutionWorldAuthorityProof>::success(
    std::move(proof));
}

WorldStatePortResult<restocker_interfaces::srv::ReleaseTaskReservation::Request>
make_release_task_reservation_request(
  const TaskReservationCapability & capability,
  const std::string & operation_id, ReservationOutcome outcome,
  TaskPhase terminal_task_phase,
  FaultState terminal_fault_state)
{
  const auto encoded_outcome = outcome_to_message(outcome);
  const auto encoded_phase = task_phase_to_message(terminal_task_phase);
  const auto encoded_fault = fault_state_to_message(terminal_fault_state);
  if (!valid_capability(capability) || !valid_operation_id(operation_id) ||
    operation_id == capability.reservation.request_id || !encoded_outcome || !encoded_phase ||
    !encoded_fault || !valid_release_disposition(capability, outcome) ||
    !valid_terminal_state(outcome, terminal_task_phase, terminal_fault_state))
  {
    return failure<restocker_interfaces::srv::ReleaseTaskReservation::Request>(
      WorldStatePortErrorCode::kInvalidInput,
      "capability, operation identity, or terminal state is invalid");
  }

  restocker_interfaces::srv::ReleaseTaskReservation::Request request;
  request.token = capability.token;
  request.operation_id = operation_id;
  request.expected_reservation_id = capability.reservation.reservation_id;
  request.expected_reservation_stage =
    static_cast<std::uint8_t>(capability.reservation.stage);
  request.expected_reservation_revision = capability.reservation.revision;
  request.outcome = *encoded_outcome;
  request.terminal_task_phase = *encoded_phase;
  request.terminal_fault_state = *encoded_fault;
  return WorldStatePortResult<restocker_interfaces::srv::ReleaseTaskReservation::Request>::success(
    std::move(request));
}

WorldStatePortResult<TaskReservationReleaseAcknowledgement>
validate_release_task_reservation_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ReleaseTaskReservation::Response & response)
{
  if (!valid_capability(capability)) {
    return failure<TaskReservationReleaseAcknowledgement>(
      WorldStatePortErrorCode::kInvalidInput, "reservation capability is invalid");
  }
  if (!known_status(response.status.code)) {
    return failure<TaskReservationReleaseAcknowledgement>(
      WorldStatePortErrorCode::kMalformedResponse,
      "release response has unknown status code",
      response.status.code);
  }
  if (response.status.code != Status::OK) {
    return failure<TaskReservationReleaseAcknowledgement>(
      WorldStatePortErrorCode::kRemoteRejected,
      response.status.detail, response.status.code);
  }
  if (!response.status.detail.empty() || response.world_revision <= capability.world_revision ||
    response.world_revision <= capability.reservation.revision)
  {
    return failure<TaskReservationReleaseAcknowledgement>(
      WorldStatePortErrorCode::kMalformedResponse,
      "successful release response has an invalid status or revision", response.status.code);
  }
  return WorldStatePortResult<TaskReservationReleaseAcknowledgement>::success(
    TaskReservationReleaseAcknowledgement{response.world_revision});
}

WorldStatePortResult<TaskReservationReleaseProof> validate_released_snapshot(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ReleaseTaskReservation::Request & request,
  const std::optional<TaskReservationReleaseAcknowledgement> & acknowledgement,
  const restocker_world_state::WorldStateSnapshot & snapshot)
{
  if (!valid_capability(capability) || request.token != capability.token ||
    !valid_operation_id(request.operation_id) ||
    request.operation_id == capability.reservation.request_id ||
    capability.world_revision == std::numeric_limits<restocker_world_state::Revision>::max() ||
    snapshot.revision <= capability.world_revision ||
    (acknowledgement &&
    (acknowledgement->world_revision <= capability.world_revision ||
    snapshot.revision < acknowledgement->world_revision)))
  {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kInvalidInput,
      "release proof inputs are invalid or causally inconsistent");
  }
  const auto converted = restocker_world_state::release_request_from_message(request);
  if (!converted) {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kInvalidInput, converted.error().detail);
  }
  if (converted.value().expected_reservation_id != capability.reservation.reservation_id ||
    converted.value().expected_reservation_stage != capability.reservation.stage ||
    converted.value().expected_reservation_revision != capability.reservation.revision ||
    !valid_release_disposition(capability, converted.value().outcome) ||
    !valid_terminal_state(
      converted.value().outcome, converted.value().terminal_task_phase,
      converted.value().terminal_fault_state))
  {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kInvalidInput,
      "release request disagrees with the retained reservation stage");
  }
  const auto minimum_release_revision = acknowledgement ?
    acknowledgement->world_revision : capability.world_revision + 1;
  if (snapshot.active_reservation || snapshot.robot.held_object ||
    snapshot.robot.revision < minimum_release_revision ||
    snapshot.robot.task_phase != converted.value().terminal_task_phase ||
    snapshot.robot.fault_state != converted.value().terminal_fault_state)
  {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "terminal snapshot does not prove reservation release and robot state");
  }

  const auto object = snapshot.objects.find(capability.reservation.object_id);
  if (object == snapshot.objects.end() ||
    object->second.source_object_id != capability.reservation.object_source_id ||
    object->second.product_class != capability.reservation.product_class ||
    object->second.sku != capability.reservation.sku ||
    object->second.grasp_state != restocker_world_state::GraspState::Free)
  {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "terminal snapshot does not contain the released free object");
  }

  const bool detached_stage = capability.reservation.stage == ReservationStage::Detached;
  if (detached_stage && object->second.revision < capability.reservation.revision) {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "terminal object revision predates the reserved detachment");
  }

  bool duplicate_membership = false;
  const auto membership = containing_lane(
    snapshot, capability.reservation.object_id, duplicate_membership);
  std::optional<LaneId> expected_membership;
  if (!detached_stage) {
    expected_membership = capability.reservation.source_lane;
  } else {
    switch (converted.value().outcome) {
      case ReservationOutcome::Succeeded:
        expected_membership = capability.reservation.destination_lane;
        break;
      case ReservationOutcome::FailedSafe:
        break;
      case ReservationOutcome::Canceled:
        return failure<TaskReservationReleaseProof>(
          WorldStatePortErrorCode::kInvalidInput,
          "detached reservation cannot have a canceled release outcome");
    }
  }
  if (duplicate_membership || membership != expected_membership) {
    return failure<TaskReservationReleaseProof>(
      WorldStatePortErrorCode::kIdentityMismatch,
      "terminal snapshot object membership differs from the release outcome");
  }
  if (converted.value().outcome == ReservationOutcome::Succeeded) {
    const auto destination = snapshot.lanes.find(capability.reservation.destination_lane);
    if (destination == snapshot.lanes.end() ||
      destination->second.revision < capability.reservation.revision)
    {
      return failure<TaskReservationReleaseProof>(
        WorldStatePortErrorCode::kIdentityMismatch,
        "terminal destination revision predates the reserved placement");
    }
  }
  return WorldStatePortResult<TaskReservationReleaseProof>::success(
    TaskReservationReleaseProof{snapshot.revision});
}

std::string to_string(WorldStatePortErrorCode code)
{
  switch (code) {
    case WorldStatePortErrorCode::kInvalidInput:
      return "invalid_input";
    case WorldStatePortErrorCode::kRemoteRejected:
      return "remote_rejected";
    case WorldStatePortErrorCode::kMalformedResponse:
      return "malformed_response";
    case WorldStatePortErrorCode::kIdentityMismatch:
      return "identity_mismatch";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
