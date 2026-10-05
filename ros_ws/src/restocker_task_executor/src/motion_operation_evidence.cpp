// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/motion_operation_evidence.hpp"

#include <algorithm>
#include <type_traits>
#include <utility>

namespace restocker_task_executor
{
static_assert(std::is_nothrow_move_constructible_v<MotionStopProofRecord>);
static_assert(std::is_nothrow_move_assignable_v<MotionStopProofRecord>);
static_assert(std::is_nothrow_destructible_v<MotionStopProofRecord>);
static_assert(std::is_nothrow_move_constructible_v<DefinitelyNotSubmittedRecord>);
static_assert(std::is_nothrow_move_assignable_v<DefinitelyNotSubmittedRecord>);
static_assert(std::is_nothrow_destructible_v<DefinitelyNotSubmittedRecord>);
static_assert(std::is_nothrow_move_constructible_v<MotionStopProofReceipt>);
static_assert(std::is_nothrow_move_assignable_v<MotionStopProofReceipt>);
static_assert(std::is_nothrow_move_constructible_v<DefinitelyNotSubmittedReceipt>);
static_assert(std::is_nothrow_move_assignable_v<DefinitelyNotSubmittedReceipt>);
static_assert(
  noexcept(std::declval<MotionStopProofRecord &>() = std::declval<MotionStopProofRecord &&>()));
static_assert(
  noexcept(std::declval<DefinitelyNotSubmittedRecord &>() =
  std::declval<DefinitelyNotSubmittedRecord &&>()));
static_assert(noexcept(std::declval<MotionStopProofRecord &>().~MotionStopProofRecord()));
static_assert(
  noexcept(std::declval<DefinitelyNotSubmittedRecord &>().~DefinitelyNotSubmittedRecord()));

namespace
{

[[nodiscard]] bool valid_local_non_submission_reason(LocalNonSubmissionReason value) noexcept
{
  switch (value) {
    case LocalNonSubmissionReason::kLocalValidationRejected:
    case LocalNonSubmissionReason::kEndpointNotReady:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_transport_non_submission_reason(
  TransportNonSubmissionReason value) noexcept
{
  switch (value) {
    case TransportNonSubmissionReason::kPreboundaryTransportRejected:
    case TransportNonSubmissionReason::kRemoteGoalRejected:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_cancellation_reason(MotionCancellationReason value) noexcept
{
  switch (value) {
    case MotionCancellationReason::kUserCancel:
    case MotionCancellationReason::kCoordinatorDrain:
    case MotionCancellationReason::kSafeAbort:
    case MotionCancellationReason::kAuthorityLoss:
    case MotionCancellationReason::kTaskDeadline:
    case MotionCancellationReason::kExpectedResultDeadline:
    case MotionCancellationReason::kInboxOverflow:
    case MotionCancellationReason::kProtocolFailure:
    case MotionCancellationReason::kShutdown:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_terminal_state(TrajectoryActionTerminalState value) noexcept
{
  switch (value) {
    case TrajectoryActionTerminalState::Succeeded:
    case TrajectoryActionTerminalState::Aborted:
    case TrajectoryActionTerminalState::Canceled:
      return true;
    case TrajectoryActionTerminalState::Unknown:
      return false;
  }
  return false;
}

[[nodiscard]] std::optional<std::string> invalid_cancellation_detail(
  const MotionCancellationRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "cancellation identity is invalid: " + *invalid;
  }
  if (value.cancel_generation == 0U) {
    return "cancellation generation must be nonzero";
  }
  if (!valid_cancellation_reason(value.reason)) {
    return "cancellation reason is outside the closed set";
  }
  return std::nullopt;
}

}  // namespace

std::optional<std::string> invalid_motion_operation_deadlines_detail(
  const MotionOperationDeadlines & value)
{
  if (value.trajectory_fingerprint == 0U) {
    return "trajectory fingerprint must be nonzero";
  }
  if (value.timing_configuration_fingerprint == 0U) {
    return "timing configuration fingerprint must be nonzero";
  }
  if (!(value.submission < value.goal_response &&
    value.goal_response < value.expected_result &&
    value.expected_result < value.cancel_response &&
    value.cancel_response <= value.terminal_drain &&
    value.expected_result < value.terminal_drain &&
    value.terminal_drain < value.stop_proof &&
    value.expected_result <= value.whole_task))
  {
    return "motion deadlines do not satisfy the immutable execution ordering";
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_operation_identity_detail(
  const MotionOperationIdentity & value)
{
  if (!admissible_goal_generation(value.ticket.goal_generation)) {
    return "goal generation is invalid or reserved";
  }
  if (value.ticket.operation_generation == 0U) {
    return "operation generation must be nonzero";
  }
  if (value.ticket.command != RestockTaskCommand::kExecutePreGrasp ||
    value.ticket.effect != OperationEffect::kCancelableMotion)
  {
    return "operation ticket is not the execute-pre-grasp cancelable-motion pair";
  }
  if (value.ticket.operation_id.empty() || value.ticket.operation_id.size() > 128U) {
    return "operation ID must contain between 1 and 128 bytes";
  }
  if (value.execution_attempt_generation == 0U) {
    return "execution-attempt generation must be nonzero";
  }
  if (value.execution_request_fingerprint == 0U) {
    return "execution request fingerprint must be nonzero";
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_ingress_stamp_detail(
  const MotionIngressStamp & value)
{
  if (value.sequence == 0U) {
    return "ingress sequence must be nonzero";
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_ingress_fault_record_detail(
  const MotionIngressFaultRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "ingress-fault identity is invalid: " + *invalid;
  }
  if (value.fault_generation == 0U || value.sequence == 0U) {
    return "ingress-fault generation and sequence must be nonzero";
  }
  switch (value.kind) {
    case MotionIngressFaultKind::kCriticalEvidenceLost:
    case MotionIngressFaultKind::kProtocolViolation:
    case MotionIngressFaultKind::kSubmissionAmbiguous:
    case MotionIngressFaultKind::kClockRegression:
    case MotionIngressFaultKind::kSequenceExhausted:
    case MotionIngressFaultKind::kProducerAbandoned:
      break;
    default:
      return "ingress-fault kind is outside the closed set";
  }
  if (value.detail.empty()) {
    return "ingress-fault detail must be nonempty";
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_request_binding_record_detail(
  const MotionRequestBindingRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "request-binding identity is invalid: " + *invalid;
  }
  if (const auto invalid = invalid_motion_operation_deadlines_detail(value.deadlines)) {
    return "request-binding deadlines are invalid: " + *invalid;
  }
  if (value.identity.ticket.deadline != value.deadlines.expected_result) {
    return "operation ticket deadline differs from the expected-result deadline";
  }
  return std::nullopt;
}

bool valid_motion_goal_uuid(const MotionGoalUuid & value) noexcept
{
  return std::any_of(value.begin(), value.end(), [](std::uint8_t byte) {return byte != 0U;});
}

std::optional<std::string> invalid_motion_goal_accepted_record_detail(
  const MotionGoalAcceptedRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "accepted-goal identity is invalid: " + *invalid;
  }
  if (value.submission_attempt_id == 0U) {
    return "accepted-goal submission-attempt ID must be nonzero";
  }
  if (!valid_motion_goal_uuid(value.goal_uuid)) {
    return "accepted-goal UUID must be nonzero";
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.ingress)) {
    return "accepted-goal ingress is invalid: " + *invalid;
  }
  return std::nullopt;
}

std::optional<std::string> invalid_definitely_not_submitted_record_detail(
  const DefinitelyNotSubmittedRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "non-submission identity is invalid: " + *invalid;
  }
  const auto evidence_error = std::visit(
    [](const auto & evidence) -> std::optional<std::string> {
      using Evidence = std::decay_t<decltype(evidence)>;
      if constexpr (std::is_same_v<Evidence, LocalNonSubmissionRecord>) {
        if (!valid_local_non_submission_reason(evidence.reason)) {
          return "local non-submission reason is outside the closed set";
        }
      } else {
        if (!valid_transport_non_submission_reason(evidence.reason)) {
          return "transport non-submission reason is outside the closed set";
        }
        if (evidence.submission_attempt_id == 0U) {
          return "transport non-submission proof requires a nonzero submission-attempt ID";
        }
      }
      return std::nullopt;
    }, value.evidence);
  if (evidence_error) {
    return evidence_error;
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.ingress)) {
    return "non-submission ingress is invalid: " + *invalid;
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_cancel_submission_record_detail(
  const MotionCancelSubmissionRecord & value)
{
  const MotionCancellationRecord cancellation{
    value.identity, value.cancel_generation, value.reason, value.requested_at};
  if (const auto invalid = invalid_cancellation_detail(cancellation)) {
    return "cancel-submission record is invalid: " + *invalid;
  }
  if (value.submission_attempt_id == 0U) {
    return "cancel-submission attempt ID must be nonzero";
  }
  if (!valid_motion_goal_uuid(value.goal_uuid)) {
    return "cancel-submission goal UUID must be nonzero";
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.ingress)) {
    return "cancel-submission ingress is invalid: " + *invalid;
  }
  if (value.requested_at > value.ingress.arrived_at) {
    return "cancellation request time is after submission ingress";
  }
  return std::nullopt;
}

std::optional<std::string> invalid_motion_terminal_record_detail(
  const MotionTerminalRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.identity)) {
    return "terminal identity is invalid: " + *invalid;
  }
  if (value.submission_attempt_id == 0U) {
    return "terminal submission-attempt ID must be nonzero";
  }
  if (!valid_motion_goal_uuid(value.goal_uuid)) {
    return "terminal goal UUID must be nonzero";
  }
  if (!valid_terminal_state(value.terminal_state)) {
    return "terminal state is not a known terminal action state";
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.ingress)) {
    return "terminal ingress is invalid: " + *invalid;
  }
  if (value.cancellation_submission) {
    if (const auto invalid = invalid_motion_cancel_submission_record_detail(
        *value.cancellation_submission))
    {
      return *invalid;
    }
    if (value.cancellation_submission->identity != value.identity ||
      value.cancellation_submission->submission_attempt_id != value.submission_attempt_id ||
      value.cancellation_submission->goal_uuid != value.goal_uuid)
    {
      return "terminal cancellation lineage identifies a different motion";
    }
    if (!exact_cancel_preceded_terminal(value)) {
      return "terminal cancellation lineage is not causally before terminal ingress";
    }
  }
  return std::nullopt;
}

bool exact_cancel_preceded_terminal(const MotionTerminalRecord & value) noexcept
{
  return value.cancellation_submission &&
         value.cancellation_submission->ingress.arrived_at <= value.ingress.arrived_at &&
         value.cancellation_submission->ingress.sequence < value.ingress.sequence;
}

std::optional<std::string> invalid_motion_stop_proof_record_detail(
  const MotionStopProofRecord & value)
{
  if (const auto invalid = invalid_motion_operation_identity_detail(value.binding.identity)) {
    return "stop-proof identity is invalid: " + *invalid;
  }
  if (const auto invalid = invalid_motion_terminal_record_detail(value.binding.terminal)) {
    return "stop-proof terminal is invalid: " + *invalid;
  }
  if (value.binding.terminal.identity != value.binding.identity) {
    return "stop-proof terminal identifies a different motion";
  }
  if (value.binding.stop_proof_deadline != value.outer_deadline) {
    return "stop-proof outer deadline differs from its immutable binding";
  }
  if (value.first_telemetry_source_id.empty() ||
    value.first_telemetry_source_id != value.final_telemetry_source_id)
  {
    return "stop-proof telemetry source must be nonempty and unchanged";
  }
  if (value.first_telemetry_revision == 0U ||
    value.first_telemetry_revision > value.final_telemetry_revision)
  {
    return "stop-proof telemetry revisions are invalid or regressing";
  }
  if (value.first_source_time.clock != MotionSourceClock::kRosTime ||
    value.final_source_time.clock != MotionSourceClock::kRosTime ||
    value.first_source_time.nanoseconds <= 0 || value.final_source_time.nanoseconds <= 0 ||
    value.first_source_time.nanoseconds > value.final_source_time.nanoseconds)
  {
    return "stop-proof source timestamps are invalid, mixed-domain, or regressing";
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.first_ingress)) {
    return "first stop-proof ingress is invalid: " + *invalid;
  }
  if (const auto invalid = invalid_motion_ingress_stamp_detail(value.final_ingress)) {
    return "final stop-proof ingress is invalid: " + *invalid;
  }
  if (value.first_ingress.arrived_at < value.binding.terminal.ingress.arrived_at ||
    value.first_ingress.sequence <= value.binding.terminal.ingress.sequence)
  {
    return "first stop-proof sample is not strictly after terminal ingress";
  }
  if (value.final_ingress.arrived_at < value.first_ingress.arrived_at ||
    value.final_ingress.sequence < value.first_ingress.sequence)
  {
    return "stop-proof ingress range regresses";
  }
  if (value.final_ingress.arrived_at >= value.outer_deadline) {
    return "stop proof completed at or after its outer deadline";
  }
  if (value.sample_count < 3U) {
    return "stop proof requires at least three accepted samples";
  }
  const bool strictly_advancing_endpoints =
    value.first_telemetry_revision < value.final_telemetry_revision &&
    value.first_source_time.nanoseconds < value.final_source_time.nanoseconds &&
    value.first_ingress.arrived_at <= value.final_ingress.arrived_at &&
    value.first_ingress.sequence < value.final_ingress.sequence;
  if (!strictly_advancing_endpoints) {
    return "stop-proof endpoints disagree with the accepted sample count";
  }
  return std::nullopt;
}

const char * to_string(TrajectoryActionTerminalState value) noexcept
{
  switch (value) {
    case TrajectoryActionTerminalState::Succeeded: return "succeeded";
    case TrajectoryActionTerminalState::Aborted: return "aborted";
    case TrajectoryActionTerminalState::Canceled: return "canceled";
    case TrajectoryActionTerminalState::Unknown: return "unknown";
  }
  return "unknown";
}

const char * to_string(LocalNonSubmissionReason value) noexcept
{
  switch (value) {
    case LocalNonSubmissionReason::kLocalValidationRejected: return "local_validation_rejected";
    case LocalNonSubmissionReason::kEndpointNotReady: return "endpoint_not_ready";
  }
  return "unknown";
}

const char * to_string(TransportNonSubmissionReason value) noexcept
{
  switch (value) {
    case TransportNonSubmissionReason::kPreboundaryTransportRejected:
      return "preboundary_transport_rejected";
    case TransportNonSubmissionReason::kRemoteGoalRejected: return "remote_goal_rejected";
  }
  return "unknown";
}

const char * to_string(MotionCancellationReason value) noexcept
{
  switch (value) {
    case MotionCancellationReason::kUserCancel: return "user_cancel";
    case MotionCancellationReason::kCoordinatorDrain: return "coordinator_drain";
    case MotionCancellationReason::kSafeAbort: return "safe_abort";
    case MotionCancellationReason::kAuthorityLoss: return "authority_loss";
    case MotionCancellationReason::kTaskDeadline: return "task_deadline";
    case MotionCancellationReason::kExpectedResultDeadline:
      return "expected_result_deadline";
    case MotionCancellationReason::kInboxOverflow: return "inbox_overflow";
    case MotionCancellationReason::kProtocolFailure: return "protocol_failure";
    case MotionCancellationReason::kShutdown: return "shutdown";
  }
  return "unknown";
}

MotionRequestBindingReceipt::MotionRequestBindingReceipt(MotionRequestBindingRecord record)
: record_(std::move(record)) {}

MotionIngressFaultReceipt::MotionIngressFaultReceipt(MotionIngressFaultRecord record)
: record_(std::move(record)) {}

MotionStopProofReceipt::MotionStopProofReceipt(MotionStopProofRecord record)
: record_(std::move(record)), live_(true)
{
}

MotionStopProofReceipt::MotionStopProofReceipt(MotionStopProofReceipt && other) noexcept
: record_(std::move(other.record_)), live_(std::exchange(other.live_, false))
{
}

MotionStopProofReceipt & MotionStopProofReceipt::operator=(MotionStopProofReceipt && other) noexcept
{
  if (this == &other) {
    live_ = false;
    return *this;
  }
  record_ = std::move(other.record_);
  live_ = std::exchange(other.live_, false);
  return *this;
}

MotionGoalAcceptedReceipt::MotionGoalAcceptedReceipt(MotionGoalAcceptedRecord record)
: record_(std::move(record)) {}

DefinitelyNotSubmittedReceipt::DefinitelyNotSubmittedReceipt(
  DefinitelyNotSubmittedRecord record)
: record_(std::move(record)), live_(true)
{
}

DefinitelyNotSubmittedReceipt::DefinitelyNotSubmittedReceipt(
  DefinitelyNotSubmittedReceipt && other) noexcept
: record_(std::move(other.record_)), live_(std::exchange(other.live_, false))
{
}

DefinitelyNotSubmittedReceipt & DefinitelyNotSubmittedReceipt::operator=(
  DefinitelyNotSubmittedReceipt && other) noexcept
{
  if (this == &other) {
    live_ = false;
    return *this;
  }
  record_ = std::move(other.record_);
  live_ = std::exchange(other.live_, false);
  return *this;
}

MotionCancelSubmissionReceipt::MotionCancelSubmissionReceipt(
  MotionCancelSubmissionRecord record)
: record_(std::move(record)) {}

MotionTerminalEvidence::MotionTerminalEvidence(MotionTerminalRecord record)
: record_(std::move(record)) {}

}  // namespace restocker_task_executor
