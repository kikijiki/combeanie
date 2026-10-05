// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_journal.hpp"

#include <algorithm>
#include <cmath>
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

[[nodiscard]] bool finite(const CanonicalPose & pose)
{
  return std::ranges::all_of(pose.translation, [](double value) {return std::isfinite(value);}) &&
         std::ranges::all_of(pose.rotation_xyzw, [](double value) {return std::isfinite(value);});
}

[[nodiscard]] bool finite(const AttachmentEvidence & evidence)
{
  return evidence.simulation_time_ns >= 0 && finite(evidence.parent_to_child) &&
         finite(evidence.child_pose_in_world) &&
         std::ranges::all_of(
    evidence.relative_twist_in_parent,
    [](double value) {return std::isfinite(value);});
}

[[nodiscard]] std::optional<AttachmentEvidence> canonicalize_evidence(
  const AttachmentEvidence & evidence)
{
  if (!finite(evidence)) {
    return std::nullopt;
  }
  const auto parent_to_child = canonicalize_pose(
    evidence.parent_to_child.translation,
    evidence.parent_to_child.rotation_xyzw);
  const auto child_pose_in_world = canonicalize_pose(
    evidence.child_pose_in_world.translation,
    evidence.child_pose_in_world.rotation_xyzw);
  if (!parent_to_child || !child_pose_in_world) {
    return std::nullopt;
  }
  AttachmentEvidence result = evidence;
  result.parent_to_child = *parent_to_child;
  result.child_pose_in_world = *child_pose_in_world;
  return result;
}

[[nodiscard]] bool valid_identity(const AttachmentIdentity & identity)
{
  return identity.object_id != 0 && !identity.object_source_id.empty() &&
         !identity.parent_model.empty() && !identity.parent_link.empty() &&
         !identity.child_model.empty() && !identity.child_link.empty();
}

[[nodiscard]] bool safe_rejection(AttachmentStatusCode code)
{
  switch (code) {
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

}  // namespace

std::optional<CanonicalPose> canonicalize_pose(
  const std::array<double, 3> & translation,
  const std::array<double, 4> & rotation_xyzw,
  double unit_tolerance)
{
  if (!std::isfinite(unit_tolerance) || unit_tolerance < 0.0 ||
    !std::ranges::all_of(
      translation, [](double value) {return std::isfinite(value);}) ||
    !std::ranges::all_of(rotation_xyzw, [](double value) {return std::isfinite(value);}))
  {
    return std::nullopt;
  }

  double squared_norm = 0.0;
  for (const double component : rotation_xyzw) {
    squared_norm += component * component;
  }
  const double norm = std::sqrt(squared_norm);
  if (!std::isfinite(norm) || norm == 0.0 || std::abs(norm - 1.0) > unit_tolerance) {
    return std::nullopt;
  }

  CanonicalPose result{translation, rotation_xyzw};
  for (double & component : result.rotation_xyzw) {
    component /= norm;
  }

  // q and -q represent the same rotation. Select the sign using w, then x/y/z at w == 0.
  const std::array<std::size_t, 4> sign_order{3, 0, 1, 2};
  for (const std::size_t index : sign_order) {
    if (result.rotation_xyzw[index] == 0.0) {
      result.rotation_xyzw[index] = 0.0;  // Canonicalize negative zero.
      continue;
    }
    if (result.rotation_xyzw[index] < 0.0) {
      for (double & component : result.rotation_xyzw) {
        component = -component;
      }
    }
    break;
  }
  return result;
}

AttachmentJournal::AttachmentJournal(
  std::string simulator_epoch, std::size_t capacity,
  std::optional<AttachmentIdentity> recovered_attachment,
  std::size_t detach_attempt_reserve)
: capacity_(capacity), detach_attempt_reserve_(detach_attempt_reserve)
{
  if (simulator_epoch.empty()) {
    throw std::invalid_argument("simulator epoch must not be empty");
  }
  if (detach_attempt_reserve == 0 || capacity < 1 + detach_attempt_reserve) {
    throw std::invalid_argument(
            "attachment journal capacity must include an attach and reserved detach attempts");
  }
  if (recovered_attachment && !valid_identity(*recovered_attachment)) {
    throw std::invalid_argument("recovered attachment identity is incomplete");
  }

  state_.simulator_epoch = std::move(simulator_epoch);
  state_.sequence = next_sequence();
  if (recovered_attachment) {
    state_.phase = AttachmentPhase::kInconsistent;
    state_.motion_gate = AttachmentMotionGate::kInhibited;
    state_.status = status(
      AttachmentStatusCode::kExternalInconsistency,
      "physical attachment recovered without an in-process operation journal");
    state_.attached_identity = std::move(recovered_attachment);
    reserved_detach_slots_ = detach_attempt_reserve_;
  } else {
    state_.phase = AttachmentPhase::kDetached;
    state_.motion_gate = AttachmentMotionGate::kValid;
    state_.status = status(AttachmentStatusCode::kDetached, "simulator is physically detached");
  }
}

AttachmentJournalReply AttachmentJournal::submit(
  std::string operation_id,
  const AttachmentRequest & request)
{
  AttachmentRequest canonical_request = request;
  if (request.command == AttachmentCommand::kAttach && request.has_expected_grasp) {
    const auto expected = canonicalize_pose(
      request.expected_grasp_center_to_child.translation,
      request.expected_grasp_center_to_child.rotation_xyzw);
    if (!expected) {
      return reply(
        status(AttachmentStatusCode::kInvalidArgument, "expected grasp pose is invalid"));
    }
    canonical_request.expected_grasp_center_to_child = *expected;
  }
  if (request.command == AttachmentCommand::kDetach) {
    // A detach has no expected grasp. Exclude the ignored storage from the semantic payload so an
    // otherwise identical retry cannot become an idempotency conflict.
    canonical_request.expected_grasp_center_to_child = CanonicalPose{};
  }
  if (operation_id.empty() || request.command == AttachmentCommand::kUnset ||
    request.reservation_id == 0 || !valid_identity(request.identity) ||
    (request.command == AttachmentCommand::kAttach && !request.has_expected_grasp) ||
    (request.command == AttachmentCommand::kDetach && request.has_expected_grasp))
  {
    return reply(
      status(AttachmentStatusCode::kInvalidArgument, "attachment request is incomplete"));
  }

  if (const auto * existing = find(operation_id)) {
    if (existing->request == canonical_request) {
      return reply(existing->status, existing, true);
    }
    return reply(
      status(
        AttachmentStatusCode::kIdempotencyConflict,
        "operation ID was already used with a different payload"),
      existing);
  }
  const bool recovery_detach = state_.phase == AttachmentPhase::kInconsistent &&
    request.command == AttachmentCommand::kDetach &&
    state_.attached_identity &&
    *state_.attached_identity == request.identity;
  if (state_.phase == AttachmentPhase::kInconsistent && !recovery_detach) {
    return reply(
      status(AttachmentStatusCode::kExternalInconsistency, "attachment state is inconsistent"));
  }
  if (active_operation_id_) {
    return reply(status(AttachmentStatusCode::kConflict, "another physical mutation is pending"));
  }

  std::size_t required_slots = 1;
  if (request.command == AttachmentCommand::kAttach) {
    if (state_.attached_identity) {
      return reply(status(AttachmentStatusCode::kStateMismatch, "an object is already attached"));
    }
    required_slots = 1 + detach_attempt_reserve_;
  } else {
    if (!state_.attached_identity || *state_.attached_identity != request.identity) {
      return reply(
        status(
          AttachmentStatusCode::kStateMismatch,
          "detach identity is not the attached object"));
    }
    if (reserved_detach_slots_ > 0) {
      --reserved_detach_slots_;
      required_slots = 0;
    }
  }
  if (!can_accept(required_slots)) {
    if (request.command == AttachmentCommand::kDetach && required_slots == 0) {
      ++reserved_detach_slots_;
    }
    return reply(
      status(AttachmentStatusCode::kResourceExhausted, "attachment operation journal is full"));
  }

  if (request.command == AttachmentCommand::kAttach) {
    reserved_detach_slots_ = detach_attempt_reserve_;
  }
  AttachmentOperationRecord record;
  record.simulator_epoch = state_.simulator_epoch;
  record.sequence = next_sequence();
  record.operation_id = std::move(operation_id);
  record.request = canonical_request;
  record.phase = request.command ==
    AttachmentCommand::kAttach ? AttachmentPhase::kValidatingAttach :
    AttachmentPhase::kValidatingDetach;
  record.status = status(AttachmentStatusCode::kPending, "physical mutation validation is pending");
  record.started_from_inconsistent = recovery_detach;
  const std::string key = record.operation_id;
  auto [iterator, inserted] = records_.emplace(key, std::move(record));
  if (!inserted) {
    throw std::logic_error("new attachment operation unexpectedly collided in journal");
  }
  active_operation_id_ = key;
  state_.sequence = iterator->second.sequence;
  state_.phase = iterator->second.phase;
  state_.motion_gate = AttachmentMotionGate::kInhibited;
  state_.status = iterator->second.status;
  return reply(iterator->second.status, &iterator->second);
}

AttachmentJournalReply AttachmentJournal::mark_validation_succeeded(
  std::string_view operation_id,
  std::optional<AttachmentFidelity> fidelity)
{
  auto * record = active_record(operation_id);
  if (!record || (record->phase != AttachmentPhase::kValidatingAttach &&
    record->phase != AttachmentPhase::kValidatingDetach))
  {
    return reply(
      status(AttachmentStatusCode::kStateMismatch, "validation transition is not active"));
  }
  if (fidelity) {
    record->fidelity = *fidelity;
    state_.fidelity = *fidelity;
  }
  record->phase = record->request.command == AttachmentCommand::kAttach ?
    AttachmentPhase::kApplyingAttach :
    AttachmentPhase::kApplyingDetach;
  record->sequence = next_sequence();
  record->status = status(AttachmentStatusCode::kPending, "physical mutation is ready to apply");
  state_.sequence = record->sequence;
  state_.phase = record->phase;
  state_.status = record->status;
  return reply(record->status, record);
}

AttachmentJournalReply AttachmentJournal::reject_before_mutation(
  std::string_view operation_id,
  AttachmentStatus rejection,
  std::optional<AttachmentFidelity> fidelity)
{
  auto * record = active_record(operation_id);
  if (!record || record->mutation_started || !safe_rejection(rejection.code) ||
    (record->phase != AttachmentPhase::kValidatingAttach &&
    record->phase != AttachmentPhase::kApplyingAttach &&
    record->phase != AttachmentPhase::kValidatingDetach &&
    record->phase != AttachmentPhase::kApplyingDetach))
  {
    return reply(status(AttachmentStatusCode::kStateMismatch, "safe rejection is not admissible"));
  }

  if (record->request.command == AttachmentCommand::kAttach) {
    reserved_detach_slots_ = 0;
  }
  if (fidelity) {
    record->fidelity = *fidelity;
    state_.fidelity = *fidelity;
  }
  record->phase = record->started_from_inconsistent ?
    AttachmentPhase::kInconsistent :
    (state_.attached_identity ? AttachmentPhase::kAttached : AttachmentPhase::kDetached);
  record->status = std::move(rejection);
  record->sequence = next_sequence();
  active_operation_id_.reset();
  if (record->started_from_inconsistent) {
    state_.sequence = record->sequence;
    state_.phase = AttachmentPhase::kInconsistent;
    state_.motion_gate = AttachmentMotionGate::kInhibited;
    state_.status = status(
      AttachmentStatusCode::kExternalInconsistency,
      "recovered attachment still requires physical reconciliation");
  } else {
    restore_idle_state(record->status);
  }
  return reply(record->status, record);
}

AttachmentJournalReply AttachmentJournal::mark_mutation_started(std::string_view operation_id)
{
  auto * record = active_record(operation_id);
  if (!record || (record->phase != AttachmentPhase::kApplyingAttach &&
    record->phase != AttachmentPhase::kApplyingDetach))
  {
    return reply(status(AttachmentStatusCode::kStateMismatch, "mutation transition is not active"));
  }
  record->mutation_started = true;
  record->phase = record->request.command == AttachmentCommand::kAttach ?
    AttachmentPhase::kVerifyingAttach :
    AttachmentPhase::kVerifyingDetach;
  record->sequence = next_sequence();
  record->status =
    status(AttachmentStatusCode::kPending, "physical mutation verification is pending");
  state_.sequence = record->sequence;
  state_.phase = record->phase;
  state_.status = record->status;
  return reply(record->status, record);
}

AttachmentJournalReply AttachmentJournal::mark_verification_succeeded(
  std::string_view operation_id, const AttachmentEvidence & evidence)
{
  auto * record = active_record(operation_id);
  if (!record || !record->mutation_started ||
    (record->phase != AttachmentPhase::kVerifyingAttach &&
    record->phase != AttachmentPhase::kVerifyingDetach))
  {
    return reply(
      status(AttachmentStatusCode::kStateMismatch, "verification transition is not active"));
  }
  const bool expect_joint = record->request.command == AttachmentCommand::kAttach;
  const auto canonical_evidence = canonicalize_evidence(evidence);
  if (!canonical_evidence || evidence.joint_observed != expect_joint) {
    return mark_outcome_unknown(operation_id, "verification evidence is invalid or contradictory");
  }

  record->evidence = *canonical_evidence;
  record->sequence = next_sequence();
  record->phase = expect_joint ? AttachmentPhase::kAttached : AttachmentPhase::kDetached;
  record->status =
    status(
    expect_joint ? AttachmentStatusCode::kAttached : AttachmentStatusCode::kDetached,
    expect_joint ? "physical attachment verified" : "physical detachment verified");
  active_operation_id_.reset();

  state_.sequence = record->sequence;
  state_.phase = record->phase;
  state_.motion_gate = AttachmentMotionGate::kValid;
  state_.status = record->status;
  state_.evidence = *canonical_evidence;
  if (expect_joint) {
    state_.attached_identity = record->request.identity;
  } else {
    state_.attached_identity.reset();
    reserved_detach_slots_ = 0;
  }
  return reply(record->status, record);
}

AttachmentJournalReply AttachmentJournal::mark_outcome_unknown(
  std::string_view operation_id,
  std::string detail)
{
  auto * record = active_record(operation_id);
  if (!record || !record->mutation_started) {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "unknown outcome requires a started mutation"));
  }
  record->sequence = next_sequence();
  record->phase = AttachmentPhase::kInconsistent;
  record->status = status(AttachmentStatusCode::kOutcomeUnknown, std::move(detail));
  active_operation_id_.reset();
  state_.sequence = record->sequence;
  state_.phase = AttachmentPhase::kInconsistent;
  state_.motion_gate = AttachmentMotionGate::kInhibited;
  state_.status = record->status;
  return reply(record->status, record);
}

AttachmentJournalReply AttachmentJournal::mark_external_inconsistency(
  std::string detail,
  std::optional<AttachmentEvidence> evidence)
{
  if (active_operation_id_ || state_.phase != AttachmentPhase::kAttached ||
    !state_.attached_identity || detail.empty())
  {
    return reply(
      status(
        AttachmentStatusCode::kStateMismatch,
        "external attachment fault requires an idle verified attachment"));
  }
  if (evidence) {
    const auto canonical_evidence = canonicalize_evidence(*evidence);
    if (!canonical_evidence || !canonical_evidence->joint_observed) {
      return reply(
        status(
          AttachmentStatusCode::kInvalidArgument,
          "external attachment fault evidence is invalid or contradictory"));
    }
    state_.evidence = *canonical_evidence;
  }
  state_.sequence = next_sequence();
  state_.phase = AttachmentPhase::kInconsistent;
  state_.motion_gate = AttachmentMotionGate::kInhibited;
  state_.status = status(AttachmentStatusCode::kExternalInconsistency, std::move(detail));
  return reply(state_.status);
}

AttachmentJournalReply AttachmentJournal::query(std::string_view operation_id) const
{
  if (operation_id.empty()) {
    return reply(state_.status);
  }
  const auto * record = find(operation_id);
  if (!record) {
    return reply(
      status(
        AttachmentStatusCode::kOperationNotFound,
        "operation ID is not in this epoch journal"));
  }
  return reply(record->status, record, true);
}

void AttachmentJournal::rotate_epoch(
  std::string simulator_epoch,
  std::optional<AttachmentIdentity> recovered_attachment)
{
  if (simulator_epoch.empty() || simulator_epoch == state_.simulator_epoch) {
    throw std::invalid_argument("new simulator epoch must be non-empty and distinct");
  }
  if (recovered_attachment && !valid_identity(*recovered_attachment)) {
    throw std::invalid_argument("recovered attachment identity is incomplete");
  }
  const bool unsafe_reset = active_operation_id_.has_value() ||
    state_.attached_identity.has_value() ||
    state_.phase == AttachmentPhase::kInconsistent;
  records_.clear();
  active_operation_id_.reset();
  reserved_detach_slots_ = recovered_attachment ? detach_attempt_reserve_ : 0;
  state_ = AttachmentPhysicalState{};
  state_.simulator_epoch = std::move(simulator_epoch);
  state_.sequence = next_sequence();
  state_.attached_identity = std::move(recovered_attachment);
  if (unsafe_reset || state_.attached_identity) {
    state_.phase = AttachmentPhase::kInconsistent;
    state_.motion_gate = AttachmentMotionGate::kInhibited;
    state_.status =
      status(
      AttachmentStatusCode::kSimulatorEpochChanged,
      "simulator epoch changed while attachment state required reconciliation");
  } else {
    state_.phase = AttachmentPhase::kDetached;
    state_.motion_gate = AttachmentMotionGate::kValid;
    state_.status =
      status(AttachmentStatusCode::kDetached, "clean detached simulator epoch started");
  }
}

const AttachmentPhysicalState & AttachmentJournal::state() const noexcept {return state_;}

std::size_t AttachmentJournal::size() const noexcept {return records_.size();}

std::size_t AttachmentJournal::capacity() const noexcept {return capacity_;}

std::size_t AttachmentJournal::reserved_detach_slots() const noexcept
{
  return reserved_detach_slots_;
}

JournalRetentionSnapshot AttachmentJournal::retention_snapshot() const
{
  JournalRetentionSnapshot result;
  result.journal = "simulator.attachment";
  result.epoch_id = state_.simulator_epoch;
  result.capacity = capacity_;
  result.size = records_.size();
  for (const auto &[operation_id, record] : records_) {
    static_cast<void>(operation_id);
    const bool in_flight = record.phase != AttachmentPhase::kAttached &&
      record.phase != AttachmentPhase::kDetached;
    // The attach record still backs the held object, so it stays an obligation.
    const bool backs_held_attachment = record.phase == AttachmentPhase::kAttached &&
      record.request.command == AttachmentCommand::kAttach && state_.attached_identity &&
      *state_.attached_identity == record.request.identity;
    if (in_flight || backs_held_attachment) {
      ++result.open_obligations;
    }
  }
  result.terminal_receipts = result.size - result.open_obligations;
  result.reserved_credits = reserved_detach_slots_;
  result.held_attachment = state_.attached_identity.has_value();
  result.inhibited = state_.motion_gate != AttachmentMotionGate::kValid;
  result.evicting = false;
  return result;
}

AttachmentJournalReply AttachmentJournal::reply(
  const AttachmentStatus & result_status,
  const AttachmentOperationRecord * operation,
  bool replayed) const
{
  AttachmentJournalReply result;
  result.status = result_status;
  result.replayed = replayed;
  if (operation) {
    result.operation = *operation;
  }
  result.state = state_;
  return result;
}

AttachmentOperationRecord * AttachmentJournal::active_record(std::string_view operation_id)
{
  if (!active_operation_id_ || *active_operation_id_ != operation_id) {
    return nullptr;
  }
  const auto found = records_.find(*active_operation_id_);
  return found == records_.end() ? nullptr : &found->second;
}

const AttachmentOperationRecord * AttachmentJournal::find(std::string_view operation_id) const
{
  const auto found = records_.find(std::string(operation_id));
  return found == records_.end() ? nullptr : &found->second;
}

bool AttachmentJournal::can_accept(std::size_t required_slots) const noexcept
{
  return records_.size() + reserved_detach_slots_ + required_slots <= capacity_;
}

std::uint64_t AttachmentJournal::next_sequence() noexcept {return ++sequence_;}

void AttachmentJournal::restore_idle_state(const AttachmentStatus & result_status)
{
  state_.sequence = sequence_;
  state_.phase = state_.attached_identity ? AttachmentPhase::kAttached : AttachmentPhase::kDetached;
  state_.motion_gate = AttachmentMotionGate::kValid;
  state_.status = result_status;
}

}  // namespace restocker_gazebo
