// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_adapter_journal.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace restocker_gazebo
{
namespace
{

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

[[nodiscard]] bool secure_equal(std::string_view left, std::string_view right)
{
  const std::size_t width = std::max(left.size(), right.size());
  std::size_t difference = left.size() ^ right.size();
  for (std::size_t index = 0; index < width; ++index) {
    const unsigned char left_value = index < left.size() ?
      static_cast<unsigned char>(left[index]) : 0U;
    const unsigned char right_value = index < right.size() ?
      static_cast<unsigned char>(right[index]) : 0U;
    difference |= static_cast<std::size_t>(left_value ^ right_value);
  }
  return difference == 0;
}

[[nodiscard]] bool same_request(
  const AdapterMutationRequest & left,
  const AdapterMutationRequest & right)
{
  return left.command == right.command && left.object_id == right.object_id &&
         secure_equal(left.reservation_token, right.reservation_token) &&
         secure_equal(left.planning_scene_lease_token, right.planning_scene_lease_token) &&
         left.has_expected_grasp == right.has_expected_grasp &&
         left.expected_grasp_center_to_child == right.expected_grasp_center_to_child &&
         left.expected_pose_covariance == right.expected_pose_covariance;
}

[[nodiscard]] bool same_canonical_pose_after_wire_round_trip(
  const CanonicalPose & left, const CanonicalPose & right)
{
  if (left.translation != right.translation) {
    return false;
  }
  // The simulator decodes and canonicalizes the quaternion before journalling it, and the adapter
  // canonicalizes that journal reply once more. Normalizing an already-normalized IEEE double
  // quaternion is not bit-idempotent: a component can alternate by one ULP. Accept only that
  // arithmetic shell; all identities, translation, covariance and capability fields stay exact.
  constexpr double kWireNormalizationShell =
    64.0 * std::numeric_limits<double>::epsilon();
  for (std::size_t index = 0; index < left.rotation_xyzw.size(); ++index) {
    if (!std::isfinite(left.rotation_xyzw[index]) ||
      !std::isfinite(right.rotation_xyzw[index]) ||
      std::abs(left.rotation_xyzw[index] - right.rotation_xyzw[index]) >
      kWireNormalizationShell)
    {
      return false;
    }
  }
  return true;
}

[[nodiscard]] bool same_transport_request(
  const AttachmentRequest & left, const AttachmentRequest & right)
{
  return left.command == right.command && left.reservation_id == right.reservation_id &&
         left.identity == right.identity && left.has_expected_grasp == right.has_expected_grasp &&
         (!left.has_expected_grasp || same_canonical_pose_after_wire_round_trip(
           left.expected_grasp_center_to_child, right.expected_grasp_center_to_child)) &&
         left.expected_pose_covariance == right.expected_pose_covariance;
}

[[nodiscard]] bool valid_authorization_failure(AttachmentStatusCode code)
{
  return code == AttachmentStatusCode::kAuthorizationFailed ||
         code == AttachmentStatusCode::kTokenMismatch ||
         code == AttachmentStatusCode::kInvalidArgument ||
         code == AttachmentStatusCode::kInternalError ||
         code == AttachmentStatusCode::kOutcomeUnknown;
}

[[nodiscard]] bool safe_transport_rejection(const AttachmentJournalReply & reply)
{
  if (!reply.operation || reply.operation->mutation_started) {
    return false;
  }
  switch (reply.status.code) {
    case AttachmentStatusCode::kInvalidArgument:
    case AttachmentStatusCode::kAuthorizationFailed:
    case AttachmentStatusCode::kTokenMismatch:
    case AttachmentStatusCode::kObjectNotFound:
    case AttachmentStatusCode::kEntityAmbiguous:
    case AttachmentStatusCode::kGripperNotReady:
    case AttachmentStatusCode::kOutOfTolerance:
    case AttachmentStatusCode::kStateMismatch:
    case AttachmentStatusCode::kInternalError:
      return true;
    default:
      return false;
  }
}

[[nodiscard]] std::string make_instance_id()
{
  // Diagnostic only: degrade like the world-state store instead of throwing from a constructor.
  try {
    std::random_device entropy;
    std::ostringstream text;
    text << std::hex << std::setfill('0');
    for (int word = 0; word < 4; ++word) {
      text << std::setw(8) << entropy();
    }
    return text.str();
  } catch (const std::exception &) {
    return "unavailable";
  }
}

}  // namespace

AttachmentAdapterJournal::AttachmentAdapterJournal(
  std::size_t capacity, std::size_t detach_attempt_reserve)
: capacity_(capacity), detach_attempt_reserve_(detach_attempt_reserve),
  instance_id_(make_instance_id())
{
  if (capacity == 0 || detach_attempt_reserve == 0) {
    throw std::invalid_argument(
            "attachment adapter journal capacity and detach reserve must be positive");
  }
}

AdapterReply AttachmentAdapterJournal::initialize(
  const AttachmentPhysicalState & simulator_state,
  bool cross_system_consistent)
{
  if (initialized_ || !records_.empty() || active_operation_id_) {
    return reply(status(AttachmentStatusCode::kStateMismatch, "adapter is already initialized"));
  }
  latest_state_ = simulator_state;
  initialized_ = true;
  const bool clean_detached = !simulator_state.simulator_epoch.empty() &&
    simulator_state.sequence != 0 && simulator_state.phase == AttachmentPhase::kDetached &&
    simulator_state.motion_gate == AttachmentMotionGate::kValid &&
    simulator_state.status.code == AttachmentStatusCode::kDetached &&
    !simulator_state.attached_identity;
  if (!cross_system_consistent || !clean_detached) {
    motion_inhibited_ = true;
    return reply(
      status(
        AttachmentStatusCode::kExternalInconsistency,
        "startup state requires explicit cross-system reconciliation"));
  }
  motion_inhibited_ = false;
  return reply(status(AttachmentStatusCode::kDetached, "adapter startup reconciliation is clean"));
}

AdapterReply AttachmentAdapterJournal::submit(
  std::string operation_id,
  const AdapterMutationRequest & request)
{
  AdapterMutationRequest canonical = request;
  if (request.command == AttachmentCommand::kAttach && request.has_expected_grasp) {
    const auto pose = canonicalize_pose(
      request.expected_grasp_center_to_child.translation,
      request.expected_grasp_center_to_child.rotation_xyzw);
    if (!pose) {
      return reply(
        status(
          AttachmentStatusCode::kInvalidArgument,
          "expected grasp pose is invalid"));
    }
    canonical.expected_grasp_center_to_child = *pose;
  }
  if (request.command == AttachmentCommand::kDetach) {
    canonical.expected_grasp_center_to_child = CanonicalPose{};
    canonical.expected_pose_covariance = {};
  }
  if (operation_id.empty() || request.command == AttachmentCommand::kUnset ||
    request.object_id == 0 || request.reservation_token.empty() ||
    request.planning_scene_lease_token.empty() ||
    (request.command == AttachmentCommand::kAttach && !request.has_expected_grasp) ||
    (request.command == AttachmentCommand::kDetach && request.has_expected_grasp))
  {
    return reply(status(AttachmentStatusCode::kInvalidArgument, "adapter request is incomplete"));
  }
  if (const auto * existing = find(operation_id)) {
    if (!same_request(existing->request, canonical)) {
      return reply(
        status(
          AttachmentStatusCode::kIdempotencyConflict,
          "operation ID was already used with a different authorized payload"),
        existing);
    }
    return reply(existing->status, existing, true);
  }
  if (!initialized_) {
    return reply(
      status(
        AttachmentStatusCode::kOutcomeUnknown,
        "attachment adapter has not completed startup reconciliation"));
  }
  if (motion_inhibited_) {
    return reply(
      status(
        AttachmentStatusCode::kExternalInconsistency,
        "attachment adapter is motion-inhibited pending reconciliation"));
  }
  if (active_operation_id_) {
    return reply(status(AttachmentStatusCode::kConflict, "another adapter operation is active"));
  }
  const bool attach = request.command == AttachmentCommand::kAttach;
  if (attach && latest_state_ && latest_state_->attached_identity) {
    return reply(status(AttachmentStatusCode::kStateMismatch, "an object is already attached"));
  }
  const Record * owner = cleanup_attach_operation_id_ ?
    find(*cleanup_attach_operation_id_) : nullptr;
  // The lease is reacquired for detach, so only the reservation capability owns the cleanup
  // credit. Live authorization still checks the new lease before any transport side effect.
  const bool cleanup = request.command == AttachmentCommand::kDetach &&
    owner && reserved_detach_slots_ > 0 &&
    request.object_id == owner->request.object_id &&
    secure_equal(request.reservation_token, owner->request.reservation_token);
  if (attach ? (detach_attempt_reserve_ >= capacity_ ||
    !can_accept(1 + detach_attempt_reserve_)) : (!cleanup && !can_accept(1)))
  {
    return reply(
      status(
        AttachmentStatusCode::kResourceExhausted,
        "adapter journal has no unreserved capacity for this operation"));
  }

  Record record;
  record.operation_id = std::move(operation_id);
  record.request = std::move(canonical);
  record.phase = Phase::kInitialAuthorization;
  record.status = status(AttachmentStatusCode::kPending, "initial capability validation pending");
  const std::string key = record.operation_id;
  auto [iterator, inserted] = records_.emplace(key, std::move(record));
  if (!inserted) {
    throw std::logic_error("new adapter operation unexpectedly collided in journal");
  }
  active_operation_id_ = key;
  if (attach) {
    reserved_detach_slots_ = detach_attempt_reserve_;
    cleanup_attach_operation_id_ = key;
  } else if (cleanup) {
    --reserved_detach_slots_;
  }
  motion_inhibited_ = true;
  return reply(
    iterator->second.status, &iterator->second, false,
    AdapterAction::kValidateInitialCapabilities);
}

AdapterReply AttachmentAdapterJournal::complete_initial_authorization(
  std::string_view operation_id,
  const AdapterAuthorizationResult & authorization)
{
  Record * record = active(operation_id);
  if (!record || record->phase != Phase::kInitialAuthorization) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "initial authorization is not active"));
  }
  if (!authorization.evidence || authorization.status.code != AttachmentStatusCode::kPending ||
    !authorization_matches(*record, *authorization.evidence))
  {
    const AttachmentStatus rejection = valid_authorization_failure(authorization.status.code) ?
      authorization.status :
      status(AttachmentStatusCode::kAuthorizationFailed, "capability authorization did not match");
    record->phase = Phase::kTerminal;
    record->status = rejection;
    if (record->request.command == AttachmentCommand::kAttach) {
      release_detach_reserve();
    }
    active_operation_id_.reset();
    motion_inhibited_ = latest_state_ ?
      latest_state_->motion_gate != AttachmentMotionGate::kValid : true;
    return reply(record->status, record);
  }

  const auto & evidence = *authorization.evidence;
  AttachmentRequest transport;
  transport.command = record->request.command;
  transport.reservation_id = evidence.reservation_id;
  transport.identity = evidence.resolved_identity;
  transport.has_expected_grasp = record->request.has_expected_grasp;
  transport.expected_grasp_center_to_child = record->request.expected_grasp_center_to_child;
  transport.expected_pose_covariance = record->request.expected_pose_covariance;
  record->transport_request = transport;
  record->phase = Phase::kReadyForTransport;
  record->status = status(
    AttachmentStatusCode::kPending,
    "authorized transport submission pending");
  return reply(
    record->status, record, false, AdapterAction::kSubmitTransportCommand);
}

AdapterReply AttachmentAdapterJournal::mark_transport_submitted(std::string_view operation_id)
{
  Record * record = active(operation_id);
  if (!record || record->phase != Phase::kReadyForTransport || !record->transport_request) {
    return reply(status(AttachmentStatusCode::kStateMismatch, "transport submission is not ready"));
  }
  record->phase = Phase::kAwaitingTransport;
  record->status = status(AttachmentStatusCode::kPending, "Gazebo attachment result pending");
  return reply(record->status, record, false, AdapterAction::kQueryTransportOperation);
}

AdapterReply AttachmentAdapterJournal::observe_transport_reply(
  std::string_view operation_id,
  const AttachmentJournalReply & transport_reply)
{
  Record * record = active(operation_id);
  if (!record || (record->phase != Phase::kAwaitingTransport &&
    record->phase != Phase::kReconcilingTransport))
  {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "transport observation is not active"));
  }
  if (transport_reply.operation &&
    (transport_reply.operation->operation_id != operation_id || !record->transport_request ||
    !same_transport_request(
      transport_reply.operation->request, *record->transport_request)))
  {
    latch_inconsistency(
      *record,
      "Gazebo operation identity or payload disagreed with adapter journal");
    return reply(record->status, record);
  }
  latest_state_ = transport_reply.state;
  record->state = transport_reply.state;
  if (transport_reply.status.code == AttachmentStatusCode::kPending ||
    transport_reply.status.code == AttachmentStatusCode::kOperationNotFound)
  {
    record->phase = Phase::kReconcilingTransport;
    record->status = status(AttachmentStatusCode::kPending, "reconciling Gazebo operation journal");
    return reply(record->status, record, false, AdapterAction::kQueryTransportOperation);
  }

  const AttachmentStatusCode expected = record->request.command == AttachmentCommand::kAttach ?
    AttachmentStatusCode::kAttached : AttachmentStatusCode::kDetached;
  if (transport_reply.status.code == expected) {
    const AttachmentPhase expected_phase = record->request.command == AttachmentCommand::kAttach ?
      AttachmentPhase::kAttached : AttachmentPhase::kDetached;
    const bool attached_identity_matches = record->request.command == AttachmentCommand::kAttach ?
      transport_reply.state.attached_identity ==
      std::optional<AttachmentIdentity>(record->transport_request->identity) :
      !transport_reply.state.attached_identity;
    const bool joint_evidence_matches = record->request.command == AttachmentCommand::kAttach ?
      transport_reply.operation && transport_reply.operation->evidence &&
      transport_reply.operation->evidence->joint_observed :
      transport_reply.operation && transport_reply.operation->evidence &&
      !transport_reply.operation->evidence->joint_observed;
    if (!transport_reply.operation || !transport_reply.operation->evidence ||
      transport_reply.operation->simulator_epoch != transport_reply.state.simulator_epoch ||
      transport_reply.operation->phase != expected_phase ||
      transport_reply.operation->status.code != expected ||
      transport_reply.state.phase != expected_phase ||
      transport_reply.state.motion_gate != AttachmentMotionGate::kValid ||
      transport_reply.state.status.code != expected || !transport_reply.state.evidence ||
      !attached_identity_matches || !joint_evidence_matches)
    {
      latch_inconsistency(*record, "Gazebo terminal success omitted exact physical proof");
      return reply(record->status, record);
    }
    record->phase = Phase::kTerminalAuthorization;
    record->status = status(
      AttachmentStatusCode::kPending,
      "terminal capability validation pending");
    return reply(
      record->status, record, false, AdapterAction::kValidateTerminalCapabilities);
  }
  if (safe_transport_rejection(transport_reply)) {
    record->phase = Phase::kTerminal;
    record->status = transport_reply.status;
    if (record->request.command == AttachmentCommand::kAttach) {
      release_detach_reserve();
    }
    active_operation_id_.reset();
    motion_inhibited_ = transport_reply.state.motion_gate != AttachmentMotionGate::kValid;
    return reply(record->status, record);
  }
  latch_inconsistency(*record, "Gazebo mutation outcome requires explicit reconciliation");
  return reply(record->status, record);
}

AdapterReply AttachmentAdapterJournal::complete_terminal_authorization(
  std::string_view operation_id,
  const AdapterAuthorizationResult & authorization)
{
  Record * record = active(operation_id);
  if (!record || record->phase != Phase::kTerminalAuthorization || !record->state) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "terminal authorization is not active"));
  }
  if (!authorization.evidence || authorization.status.code != AttachmentStatusCode::kPending ||
    !authorization_matches(*record, *authorization.evidence))
  {
    latch_inconsistency(*record, "capability changed after physical mutation succeeded");
    return reply(record->status, record);
  }
  const AttachmentStatusCode terminal = record->request.command == AttachmentCommand::kAttach ?
    AttachmentStatusCode::kAttached : AttachmentStatusCode::kDetached;
  record->phase = Phase::kTerminal;
  record->status = status(terminal, "physical mutation and capabilities verified");
  if (record->request.command == AttachmentCommand::kDetach) {
    release_detach_reserve();
  }
  active_operation_id_.reset();
  motion_inhibited_ = record->state->motion_gate != AttachmentMotionGate::kValid;
  return reply(record->status, record);
}

AdapterReply AttachmentAdapterJournal::mark_reconciliation_deadline_exceeded(
  std::string_view operation_id,
  std::string detail)
{
  Record * record = active(operation_id);
  if (!record || (record->phase != Phase::kAwaitingTransport &&
    record->phase != Phase::kReconcilingTransport))
  {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "transport reconciliation is not active"));
  }
  record->phase = Phase::kReconcilingTransport;
  record->status = status(AttachmentStatusCode::kOutcomeUnknown, std::move(detail));
  motion_inhibited_ = true;
  return reply(record->status, record, false, AdapterAction::kQueryTransportOperation);
}

AdapterReply AttachmentAdapterJournal::mark_transport_inconsistency(
  std::string_view operation_id,
  std::string detail)
{
  Record * record = active(operation_id);
  if (!record || (record->phase != Phase::kAwaitingTransport &&
    record->phase != Phase::kReconcilingTransport))
  {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "transport inconsistency cannot be applied to this operation"));
  }
  latch_inconsistency(*record, std::move(detail));
  return reply(record->status, record);
}

AdapterReply AttachmentAdapterJournal::observe_idle_state(
  const AttachmentPhysicalState & simulator_state)
{
  if (!initialized_ || !latest_state_) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "adapter has no initialized physical state"));
  }
  if (active_operation_id_) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "idle physical observation received while an operation is active"));
  }
  if (motion_inhibited_) {
    return reply(
      status(
        AttachmentStatusCode::kExternalInconsistency,
        "adapter motion inhibition is latched pending explicit reconciliation"));
  }
  // Judged on the physical state alone. The status code alongside it is the outcome of the
  // simulator's last action, not where it now stands: after a mutation it refused before touching
  // anything, the boundary restores an idle phase and a valid motion gate while still reporting
  // the refusal. Requiring the code to restate the phase turned every such safe refusal into a
  // permanent motion inhibition on the next idle poll, which then refused the coordinator's retry.
  // Every state the simulator cannot vouch for is an inconsistent phase or an inhibited gate, and
  // both are still refused below.
  const bool expected_terminal =
    (simulator_state.phase == AttachmentPhase::kDetached &&
    !simulator_state.attached_identity) ||
    (simulator_state.phase == AttachmentPhase::kAttached &&
    simulator_state.attached_identity.has_value());
  const bool identity_stable =
    simulator_state.attached_identity == latest_state_->attached_identity;
  const bool observation_consistent =
    simulator_state.simulator_epoch == latest_state_->simulator_epoch &&
    simulator_state.sequence >= latest_state_->sequence &&
    simulator_state.phase == latest_state_->phase && identity_stable && expected_terminal &&
    simulator_state.motion_gate == AttachmentMotionGate::kValid;
  latest_state_ = simulator_state;
  if (!observation_consistent) {
    motion_inhibited_ = true;
    return reply(
      status(
        AttachmentStatusCode::kExternalInconsistency,
        "idle Gazebo state changed without an authorized adapter operation"));
  }
  motion_inhibited_ = false;
  return reply(simulator_state.status);
}

AdapterReply AttachmentAdapterJournal::mark_idle_inconsistency(std::string detail)
{
  if (!initialized_ || active_operation_id_) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "idle inconsistency cannot be applied in the current adapter state"));
  }
  motion_inhibited_ = true;
  return reply(status(AttachmentStatusCode::kExternalInconsistency, std::move(detail)));
}

AdapterReply AttachmentAdapterJournal::query(std::string_view operation_id) const
{
  if (operation_id.empty()) {
    return reply(
      latest_state_ ? latest_state_->status :
      status(AttachmentStatusCode::kOutcomeUnknown, "adapter has not reconciled simulator state"));
  }
  const Record * record = find(operation_id);
  if (!record) {
    return reply(
      status(
        AttachmentStatusCode::kOperationNotFound,
        "adapter operation was not found"));
  }
  return reply(record->status, record, true);
}

bool AttachmentAdapterJournal::motion_inhibited() const noexcept
{
  return motion_inhibited_;
}

std::size_t AttachmentAdapterJournal::size() const noexcept
{
  return records_.size();
}

std::size_t AttachmentAdapterJournal::capacity() const noexcept
{
  return capacity_;
}

JournalRetentionSnapshot AttachmentAdapterJournal::retention_snapshot() const
{
  JournalRetentionSnapshot result;
  result.journal = "adapter.operations";
  result.epoch_id = instance_id_;
  result.capacity = capacity_;
  result.size = records_.size();
  for (const auto &[operation_id, record] : records_) {
    // The attach record owning the protected detach credit stays an obligation after it settles.
    const bool owns_cleanup_credit = reserved_detach_slots_ > 0 &&
      cleanup_attach_operation_id_ && operation_id == *cleanup_attach_operation_id_;
    if (record.phase != Phase::kTerminal || owns_cleanup_credit) {
      ++result.open_obligations;
    }
  }
  result.terminal_receipts = result.size - result.open_obligations;
  result.reserved_credits = reserved_detach_slots_;
  result.held_attachment = latest_state_ && latest_state_->attached_identity.has_value();
  result.inhibited = motion_inhibited_;
  result.evicting = false;
  return result;
}

AdapterReply AttachmentAdapterJournal::reply(
  const AttachmentStatus & status_value, const Record * record,
  bool replayed, AdapterAction action) const
{
  AdapterReply result;
  result.status = status_value;
  result.replayed = replayed;
  result.action = action;
  if (record) {
    result.transport_request = record->transport_request;
    result.state = record->state;
  } else {
    result.state = latest_state_;
  }
  return result;
}

AttachmentAdapterJournal::Record * AttachmentAdapterJournal::active(
  std::string_view operation_id)
{
  if (!active_operation_id_ || *active_operation_id_ != operation_id) {
    return nullptr;
  }
  const auto iterator = records_.find(*active_operation_id_);
  return iterator == records_.end() ? nullptr : &iterator->second;
}

const AttachmentAdapterJournal::Record * AttachmentAdapterJournal::find(
  std::string_view operation_id) const
{
  const auto iterator = records_.find(std::string(operation_id));
  return iterator == records_.end() ? nullptr : &iterator->second;
}

bool AttachmentAdapterJournal::authorization_matches(
  const Record & record,
  const AdapterAuthorizationEvidence & evidence) const
{
  const AdapterReservationStage expected_stage =
    record.request.command == AttachmentCommand::kAttach ?
    AdapterReservationStage::kReserved : AdapterReservationStage::kAttached;
  const bool agrees_with_initial = !record.transport_request ||
    (record.transport_request->reservation_id == evidence.reservation_id &&
    record.transport_request->identity == evidence.resolved_identity);
  return evidence.reservation_id != 0 && evidence.object_id == record.request.object_id &&
         evidence.reservation_stage == expected_stage && evidence.lease_id != 0 &&
         evidence.lease_phase == AdapterLeasePhase::kHeld &&
         evidence.resolved_identity.object_id == record.request.object_id &&
         evidence.resolved_identity.object_source_id == evidence.object_source_id &&
         !evidence.object_source_id.empty() &&
         !evidence.resolved_identity.parent_model.empty() &&
         !evidence.resolved_identity.parent_link.empty() &&
         !evidence.resolved_identity.child_model.empty() &&
         !evidence.resolved_identity.child_link.empty() && agrees_with_initial;
}

bool AttachmentAdapterJournal::can_accept(std::size_t required_slots) const noexcept
{
  return reserved_detach_slots_ <= capacity_ &&
         required_slots <= capacity_ - reserved_detach_slots_ &&
         records_.size() <= capacity_ - reserved_detach_slots_ - required_slots;
}

void AttachmentAdapterJournal::release_detach_reserve()
{
  reserved_detach_slots_ = 0;
  cleanup_attach_operation_id_.reset();
}

void AttachmentAdapterJournal::latch_inconsistency(Record & record, std::string detail)
{
  record.phase = Phase::kInconsistent;
  record.status = status(AttachmentStatusCode::kExternalInconsistency, std::move(detail));
  motion_inhibited_ = true;
}

}  // namespace restocker_gazebo
