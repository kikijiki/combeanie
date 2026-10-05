// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/goal_termination_authority.hpp"

#include <algorithm>
#include <chrono>
#include <ratio>
#include <string_view>
#include <type_traits>

namespace restocker_task_executor
{
MotionAdmissionRegistrationRecord::MotionAdmissionRegistrationRecord(
  CoordinatorGoalId goal_id, GoalGeneration goal_generation,
  MotionAdmissionSessionGeneration session_generation,
  OperationTicket operation_ticket,
  ExecutionAttemptGeneration execution_attempt_generation) noexcept
: goal_id_(std::move(goal_id)),
  goal_generation_(goal_generation),
  session_generation_(session_generation),
  operation_ticket_(std::move(operation_ticket)),
  execution_attempt_generation_(execution_attempt_generation)
{
}

const CoordinatorGoalId & MotionAdmissionRegistrationRecord::goal_id() const noexcept
{
  return goal_id_;
}

GoalGeneration MotionAdmissionRegistrationRecord::goal_generation() const noexcept
{
  return goal_generation_;
}

MotionAdmissionSessionGeneration
MotionAdmissionRegistrationRecord::session_generation() const noexcept
{
  return session_generation_;
}

const OperationTicket & MotionAdmissionRegistrationRecord::operation_ticket() const noexcept
{
  return operation_ticket_;
}

ExecutionAttemptGeneration
MotionAdmissionRegistrationRecord::execution_attempt_generation() const noexcept
{
  return execution_attempt_generation_;
}

namespace
{

class EvidenceFingerprint final
{
public:
  void add_byte(std::uint8_t value) noexcept
  {
    value_ ^= value;
    value_ *= kPrime;
  }

  void add_bool(bool value) noexcept
  {
    add_byte(value ? 1U : 0U);
  }

  void add_u64(std::uint64_t value) noexcept
  {
    for (std::size_t index = 0U; index < sizeof(value); ++index) {
      add_byte(static_cast<std::uint8_t>(value & UINT64_C(0xff)));
      value >>= 8U;
    }
  }

  void add_i64(std::int64_t value) noexcept
  {
    add_u64(static_cast<std::uint64_t>(value));
  }

  void add_string(std::string_view value) noexcept
  {
    add_u64(static_cast<std::uint64_t>(value.size()));
    for (const unsigned char byte : value) {
      add_byte(byte);
    }
  }

  [[nodiscard]] std::uint64_t value() const noexcept
  {
    return value_;
  }

private:
  static constexpr std::uint64_t kOffset = 14695981039346656037ULL;
  static constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t value_{kOffset};
};

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id) noexcept
{
  return std::ranges::any_of(goal_id, [](std::uint8_t byte) {return byte != 0U;});
}

[[nodiscard]] bool valid_source(GoalTerminationSource source) noexcept
{
  switch (source) {
    case GoalTerminationSource::kAuthorityObservation:
    case GoalTerminationSource::kCoordinatorInbox:
    case GoalTerminationSource::kTaskDeadline:
    case GoalTerminationSource::kMotionFence:
      return true;
    case GoalTerminationSource::kNone:
      return false;
  }
  return false;
}

[[nodiscard]] GoalTerminationValidationError validate_source(
  const FirstGoalTerminationRecord & record) noexcept
{
  if (record.source_generation) {
    if (!valid_source(record.source_generation->source)) {
      return GoalTerminationValidationError::kInvalidSourceKind;
    }
    if (record.source_generation->generation == 0U) {
      return GoalTerminationValidationError::kInvalidSourceGeneration;
    }
  }

  const auto source_is = [&record](GoalTerminationSource source) {
    return record.source_generation && record.source_generation->source == source;
  };
  switch (record.kind) {
    case GoalTerminationKind::kUserCancel:
    case GoalTerminationKind::kCoordinatorDrain:
    case GoalTerminationKind::kSafeAbort:
    case GoalTerminationKind::kShutdown:
    case GoalTerminationKind::kProtocolFailure:
      return record.source_generation ? GoalTerminationValidationError::kUnexpectedSource :
             GoalTerminationValidationError::kNone;
    case GoalTerminationKind::kAuthorityLoss:
      return !record.source_generation || source_is(GoalTerminationSource::kAuthorityObservation) ?
             GoalTerminationValidationError::kNone :
             GoalTerminationValidationError::kUnexpectedSource;
    case GoalTerminationKind::kInboxOverflow:
      return !record.source_generation || source_is(GoalTerminationSource::kCoordinatorInbox) ?
             GoalTerminationValidationError::kNone :
             GoalTerminationValidationError::kUnexpectedSource;
    case GoalTerminationKind::kTaskDeadline:
      if (!record.source_generation) {
        return GoalTerminationValidationError::kMissingSource;
      }
      return source_is(GoalTerminationSource::kTaskDeadline) ?
             GoalTerminationValidationError::kNone :
             GoalTerminationValidationError::kUnexpectedSource;
    case GoalTerminationKind::kMotionDeadline:
      if (!record.source_generation) {
        return GoalTerminationValidationError::kMissingSource;
      }
      return source_is(GoalTerminationSource::kMotionFence) ?
             GoalTerminationValidationError::kNone :
             GoalTerminationValidationError::kUnexpectedSource;
  }
  return GoalTerminationValidationError::kInvalidKind;
}

[[nodiscard]] bool valid_latch_shape(
  GoalTerminationLatchStatus status, bool has_record) noexcept
{
  switch (status) {
    case GoalTerminationLatchStatus::kLatched:
    case GoalTerminationLatchStatus::kAlreadyLatched:
      return has_record;
    case GoalTerminationLatchStatus::kGoalMismatch:
      return true;
    case GoalTerminationLatchStatus::kInactive:
    case GoalTerminationLatchStatus::kInvalidArgument:
      return !has_record;
  }
  return false;
}

[[nodiscard]] bool valid_snapshot_shape(
  GoalTerminationSnapshotStatus status, bool has_record) noexcept
{
  switch (status) {
    case GoalTerminationSnapshotStatus::kPresent:
      return has_record;
    case GoalTerminationSnapshotStatus::kGoalMismatch:
      return true;
    case GoalTerminationSnapshotStatus::kNone:
    case GoalTerminationSnapshotStatus::kInactive:
    case GoalTerminationSnapshotStatus::kInvalidArgument:
      return !has_record;
  }
  return false;
}

}  // namespace

GoalTerminationLatchDecision::GoalTerminationLatchDecision(
  GoalTerminationLatchStatus status,
  std::shared_ptr<const FirstGoalTerminationRecord> record) noexcept
: status_(valid_latch_shape(status, static_cast<bool>(record)) ?
    status : GoalTerminationLatchStatus::kInvalidArgument),
  record_(status_ == GoalTerminationLatchStatus::kInvalidArgument ? nullptr : std::move(record))
{
}

GoalTerminationLatchDecision::GoalTerminationLatchDecision(
  GoalTerminationLatchDecision && other) noexcept
: status_(other.status_), record_(std::move(other.record_))
{
  other.status_ = GoalTerminationLatchStatus::kInvalidArgument;
  other.record_.reset();
}

GoalTerminationLatchDecision & GoalTerminationLatchDecision::operator=(
  GoalTerminationLatchDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    record_ = std::move(other.record_);
    other.status_ = GoalTerminationLatchStatus::kInvalidArgument;
    other.record_.reset();
  }
  return *this;
}

GoalTerminationLatchStatus GoalTerminationLatchDecision::status() const noexcept
{
  return status_;
}

std::shared_ptr<const FirstGoalTerminationRecord>
GoalTerminationLatchDecision::record() const noexcept
{
  return record_;
}

GoalTerminationSnapshotDecision::GoalTerminationSnapshotDecision(
  GoalTerminationSnapshotStatus status,
  std::shared_ptr<const FirstGoalTerminationRecord> record) noexcept
: status_(valid_snapshot_shape(status, static_cast<bool>(record)) ?
    status : GoalTerminationSnapshotStatus::kInvalidArgument),
  record_(status_ == GoalTerminationSnapshotStatus::kInvalidArgument ? nullptr : std::move(record))
{
}

GoalTerminationSnapshotDecision::GoalTerminationSnapshotDecision(
  GoalTerminationSnapshotDecision && other) noexcept
: status_(other.status_), record_(std::move(other.record_))
{
  other.status_ = GoalTerminationSnapshotStatus::kInvalidArgument;
  other.record_.reset();
}

GoalTerminationSnapshotDecision & GoalTerminationSnapshotDecision::operator=(
  GoalTerminationSnapshotDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    record_ = std::move(other.record_);
    other.status_ = GoalTerminationSnapshotStatus::kInvalidArgument;
    other.record_.reset();
  }
  return *this;
}

GoalTerminationSnapshotStatus GoalTerminationSnapshotDecision::status() const noexcept
{
  return status_;
}

std::shared_ptr<const FirstGoalTerminationRecord>
GoalTerminationSnapshotDecision::record() const noexcept
{
  return record_;
}

std::optional<MotionCancellationReason> cancellation_reason_for(
  GoalTerminationKind kind) noexcept
{
  switch (kind) {
    case GoalTerminationKind::kUserCancel:
      return MotionCancellationReason::kUserCancel;
    case GoalTerminationKind::kCoordinatorDrain:
      return MotionCancellationReason::kCoordinatorDrain;
    case GoalTerminationKind::kSafeAbort:
      return MotionCancellationReason::kSafeAbort;
    case GoalTerminationKind::kShutdown:
      return MotionCancellationReason::kShutdown;
    case GoalTerminationKind::kAuthorityLoss:
      return MotionCancellationReason::kAuthorityLoss;
    case GoalTerminationKind::kInboxOverflow:
      return MotionCancellationReason::kInboxOverflow;
    case GoalTerminationKind::kProtocolFailure:
      return MotionCancellationReason::kProtocolFailure;
    case GoalTerminationKind::kTaskDeadline:
      return MotionCancellationReason::kTaskDeadline;
    case GoalTerminationKind::kMotionDeadline:
      return MotionCancellationReason::kExpectedResultDeadline;
  }
  return std::nullopt;
}

GoalTerminationValidationError validate_goal_termination_record(
  const FirstGoalTerminationRecord & record) noexcept
{
  switch (record.scope) {
    case GoalTerminationScope::kGoal:
      if (record.motion_registration) {
        return GoalTerminationValidationError::kUnexpectedMotionRegistration;
      }
      break;
    case GoalTerminationScope::kRegisteredMotion:
      if (!record.motion_registration) {
        return GoalTerminationValidationError::kMissingMotionRegistration;
      }
      if (!valid_motion_admission_registration_record(*record.motion_registration)) {
        return GoalTerminationValidationError::kInvalidMotionRegistration;
      }
      if (record.goal_id != record.motion_registration->goal_id() ||
        record.goal_generation != record.motion_registration->goal_generation())
      {
        return GoalTerminationValidationError::kMotionRegistrationGoalMismatch;
      }
      break;
    default:
      return GoalTerminationValidationError::kInvalidScope;
  }
  if (!valid_goal_id(record.goal_id)) {
    return GoalTerminationValidationError::kInvalidGoalId;
  }
  if (!admissible_goal_generation(record.goal_generation)) {
    return GoalTerminationValidationError::kInvalidGoalGeneration;
  }
  const auto expected_reason = cancellation_reason_for(record.kind);
  if (!expected_reason) {
    return GoalTerminationValidationError::kInvalidKind;
  }
  if (record.cancellation_reason != *expected_reason) {
    return GoalTerminationValidationError::kCancellationReasonMismatch;
  }
  if (record.arrived_at == SteadyTime::max()) {
    return GoalTerminationValidationError::kInvalidArrivalTime;
  }
  return validate_source(record);
}

bool valid_motion_admission_registration_record(
  const MotionAdmissionRegistrationRecord & record) noexcept
{
  const auto & ticket = record.operation_ticket();
  return valid_goal_id(record.goal_id()) &&
         admissible_goal_generation(record.goal_generation()) &&
         record.session_generation() != 0U &&
         ticket.goal_generation == record.goal_generation() &&
         ticket.operation_generation != 0U &&
         ticket.command == RestockTaskCommand::kExecutePreGrasp &&
         ticket.effect == OperationEffect::kCancelableMotion &&
         !ticket.operation_id.empty() &&
         ticket.operation_id.size() <= 128U &&
         record.execution_attempt_generation() != 0U;
}

bool same_goal_termination_evidence(
  const std::shared_ptr<const FirstGoalTerminationRecord> & lhs,
  const std::shared_ptr<const FirstGoalTerminationRecord> & rhs) noexcept
{
  if (static_cast<bool>(lhs) != static_cast<bool>(rhs)) {
    return false;
  }
  return !lhs || *lhs == *rhs;
}

std::uint64_t goal_termination_evidence_fingerprint(
  const std::shared_ptr<const FirstGoalTerminationRecord> & value) noexcept
{
  EvidenceFingerprint fingerprint;
  fingerprint.add_string("goal-termination-evidence:v1");
  fingerprint.add_bool(static_cast<bool>(value));
  if (!value) {
    return fingerprint.value();
  }
  fingerprint.add_byte(static_cast<std::uint8_t>(value->scope));
  for (const std::uint8_t byte : value->goal_id) {
    fingerprint.add_byte(byte);
  }
  fingerprint.add_u64(value->goal_generation);
  fingerprint.add_byte(static_cast<std::uint8_t>(value->kind));
  fingerprint.add_byte(static_cast<std::uint8_t>(value->cancellation_reason));
  fingerprint.add_i64(static_cast<std::int64_t>(value->arrived_at.time_since_epoch().count()));
  fingerprint.add_bool(value->source_generation.has_value());
  if (value->source_generation) {
    fingerprint.add_byte(static_cast<std::uint8_t>(value->source_generation->source));
    fingerprint.add_u64(value->source_generation->generation);
  }
  if (value->scope == GoalTerminationScope::kRegisteredMotion &&
    value->motion_registration)
  {
    const auto & registration = *value->motion_registration;
    for (const std::uint8_t byte : registration.goal_id()) {
      fingerprint.add_byte(byte);
    }
    fingerprint.add_u64(registration.goal_generation());
    fingerprint.add_u64(registration.session_generation());
    fingerprint.add_u64(registration.operation_ticket().goal_generation);
    fingerprint.add_u64(registration.operation_ticket().operation_generation);
    fingerprint.add_byte(
      static_cast<std::uint8_t>(registration.operation_ticket().command));
    fingerprint.add_byte(
      static_cast<std::uint8_t>(registration.operation_ticket().effect));
    fingerprint.add_string(registration.operation_ticket().operation_id);
    fingerprint.add_i64(
      static_cast<std::int64_t>(
        registration.operation_ticket().deadline.time_since_epoch().count()));
    fingerprint.add_u64(registration.execution_attempt_generation());
  }
  return fingerprint.value();
}

static_assert(std::is_integral_v<SteadyTime::duration::rep>);
static_assert(std::is_signed_v<SteadyTime::duration::rep>);
static_assert(sizeof(SteadyTime::duration::rep) == sizeof(std::int64_t));
static_assert(std::ratio_equal_v<SteadyTime::duration::period, std::nano>);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kUserCancel) == 0U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kCoordinatorDrain) == 1U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kSafeAbort) == 2U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kAuthorityLoss) == 3U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kTaskDeadline) == 4U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kExpectedResultDeadline) == 5U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kInboxOverflow) == 6U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kProtocolFailure) == 7U);
static_assert(static_cast<std::uint8_t>(MotionCancellationReason::kShutdown) == 8U);

}  // namespace restocker_task_executor
