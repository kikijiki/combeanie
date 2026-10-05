// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <variant>

#include "restocker_task_executor/coordinator_operation_types.hpp"

namespace restocker_task_executor
{

class PendingOperationLedger;
class PreGraspStopSampler;
class MotionIngressAuthority;
struct PreGraspExecutionRequest;
template<typename T>
class PreGraspExecutionContractResult;

class MotionRequestBindingReceipt;

[[nodiscard]] PreGraspExecutionContractResult<MotionRequestBindingReceipt>
make_motion_request_binding_receipt(
  const OperationTicket & ticket, const PreGraspExecutionRequest & request);

using MotionSubmissionAttemptId = std::uint64_t;
using MotionCancelGeneration = std::uint64_t;
using MotionIngressSequence = std::uint64_t;
inline constexpr std::size_t kMotionGoalUuidSize = 16U;
using MotionGoalUuid = std::array<std::uint8_t, kMotionGoalUuidSize>;

enum class MotionIngressFaultKind : std::uint8_t
{
  kCriticalEvidenceLost,
  kProtocolViolation,
  kSubmissionAmbiguous,
  kClockRegression,
  kSequenceExhausted,
  kProducerAbandoned,
};

enum class TrajectoryActionTerminalState : std::uint8_t
{
  Succeeded,
  Aborted,
  Canceled,
  Unknown,
};

enum class LocalNonSubmissionReason : std::uint8_t
{
  kLocalValidationRejected,
  kEndpointNotReady,
};

enum class TransportNonSubmissionReason : std::uint8_t
{
  kPreboundaryTransportRejected,
  kRemoteGoalRejected,
};

enum class MotionCancellationReason : std::uint8_t
{
  kUserCancel,
  kCoordinatorDrain,
  kSafeAbort,
  kAuthorityLoss,
  kTaskDeadline,
  kExpectedResultDeadline,
  kInboxOverflow,
  kProtocolFailure,
  kShutdown,
};

struct MotionOperationDeadlines
{
  SteadyTime submission{};
  SteadyTime whole_task{};
  SteadyTime goal_response{};
  SteadyTime expected_result{};
  SteadyTime cancel_response{};
  SteadyTime terminal_drain{};
  SteadyTime stop_proof{};
  std::uint64_t trajectory_fingerprint{0U};
  std::uint64_t timing_configuration_fingerprint{0U};

  [[nodiscard]] bool operator==(const MotionOperationDeadlines &) const noexcept = default;
};

struct MotionOperationIdentity
{
  OperationTicket ticket;
  ExecutionAttemptGeneration execution_attempt_generation{0U};
  std::uint64_t execution_request_fingerprint{0U};

  [[nodiscard]] bool operator==(const MotionOperationIdentity &) const noexcept = default;
};

struct MotionIngressStamp
{
  SteadyTime arrived_at{};
  MotionIngressSequence sequence{0U};

  [[nodiscard]] bool operator==(const MotionIngressStamp &) const noexcept = default;
};

struct MotionIngressFaultRecord
{
  MotionOperationIdentity identity;
  std::uint64_t fault_generation{0U};
  MotionIngressSequence sequence{0U};
  SteadyTime observed_at{};
  MotionIngressFaultKind kind{MotionIngressFaultKind::kProtocolViolation};
  std::string detail;

  [[nodiscard]] bool operator==(const MotionIngressFaultRecord &) const noexcept = default;
};

struct MotionRequestBindingRecord
{
  MotionOperationIdentity identity;
  MotionOperationDeadlines deadlines;

  [[nodiscard]] bool operator==(const MotionRequestBindingRecord &) const noexcept = default;
};

struct MotionGoalAcceptedRecord
{
  MotionOperationIdentity identity;
  MotionSubmissionAttemptId submission_attempt_id{0U};
  MotionGoalUuid goal_uuid{};
  MotionIngressStamp ingress;

  [[nodiscard]] bool operator==(const MotionGoalAcceptedRecord &) const noexcept = default;
};

struct MotionCancellationRecord
{
  MotionOperationIdentity identity;
  MotionCancelGeneration cancel_generation{0U};
  MotionCancellationReason reason{MotionCancellationReason::kProtocolFailure};
  SteadyTime requested_at{};

  [[nodiscard]] bool operator==(const MotionCancellationRecord &) const noexcept = default;
};

struct LocalNonSubmissionRecord
{
  LocalNonSubmissionReason reason{LocalNonSubmissionReason::kLocalValidationRejected};

  [[nodiscard]] bool operator==(const LocalNonSubmissionRecord &) const noexcept = default;
};

struct TransportNonSubmissionRecord
{
  TransportNonSubmissionReason reason{
    TransportNonSubmissionReason::kPreboundaryTransportRejected};
  MotionSubmissionAttemptId submission_attempt_id{0U};

  [[nodiscard]] bool operator==(const TransportNonSubmissionRecord &) const noexcept = default;
};

struct DefinitelyNotSubmittedRecord
{
  MotionOperationIdentity identity;
  std::variant<LocalNonSubmissionRecord, TransportNonSubmissionRecord> evidence;
  MotionIngressStamp ingress;

  [[nodiscard]] bool operator==(const DefinitelyNotSubmittedRecord &) const noexcept = default;
};

struct MotionCancelSubmissionRecord
{
  MotionOperationIdentity identity;
  MotionSubmissionAttemptId submission_attempt_id{0U};
  MotionGoalUuid goal_uuid{};
  MotionCancelGeneration cancel_generation{0U};
  MotionCancellationReason reason{MotionCancellationReason::kProtocolFailure};
  SteadyTime requested_at{};
  MotionIngressStamp ingress;

  [[nodiscard]] bool operator==(const MotionCancelSubmissionRecord &) const noexcept = default;
};

struct MotionTerminalRecord
{
  MotionOperationIdentity identity;
  MotionSubmissionAttemptId submission_attempt_id{0U};
  MotionGoalUuid goal_uuid{};
  TrajectoryActionTerminalState terminal_state{TrajectoryActionTerminalState::Unknown};
  std::optional<std::int32_t> moveit_error_code;
  std::optional<MotionCancelSubmissionRecord> cancellation_submission;
  MotionIngressStamp ingress;

  [[nodiscard]] bool operator==(const MotionTerminalRecord &) const noexcept = default;
};

enum class MotionSourceClock : std::int32_t
{
  kUninitialized = 0,
  kRosTime = 1,
};

struct MotionStopProofBinding
{
  MotionOperationIdentity identity;
  MotionTerminalRecord terminal;
  SteadyTime stop_proof_deadline{};

  [[nodiscard]] bool operator==(const MotionStopProofBinding &) const noexcept = default;
};

// Source timestamps remain backend-neutral. Only positive ROS-time samples can contribute to
// proof; no source timestamp is ever compared with a steady-clock time point.
struct MotionSourceTimestamp
{
  std::int64_t nanoseconds{0};
  MotionSourceClock clock{MotionSourceClock::kUninitialized};

  [[nodiscard]] bool operator==(const MotionSourceTimestamp &) const noexcept = default;
};

struct MotionStopProofRecord
{
  MotionStopProofBinding binding;
  std::string first_telemetry_source_id;
  std::string final_telemetry_source_id;
  std::uint64_t first_telemetry_revision{0U};
  std::uint64_t final_telemetry_revision{0U};
  MotionSourceTimestamp first_source_time;
  MotionSourceTimestamp final_source_time;
  MotionIngressStamp first_ingress;
  MotionIngressStamp final_ingress;
  std::size_t sample_count{0U};
  SteadyTime outer_deadline{};

  [[nodiscard]] bool operator==(const MotionStopProofRecord &) const noexcept = default;
};

[[nodiscard]] std::optional<std::string> invalid_motion_operation_deadlines_detail(
  const MotionOperationDeadlines & value);
[[nodiscard]] std::optional<std::string> invalid_motion_operation_identity_detail(
  const MotionOperationIdentity & value);
[[nodiscard]] std::optional<std::string> invalid_motion_ingress_stamp_detail(
  const MotionIngressStamp & value);
[[nodiscard]] std::optional<std::string> invalid_motion_ingress_fault_record_detail(
  const MotionIngressFaultRecord & value);
[[nodiscard]] std::optional<std::string> invalid_motion_request_binding_record_detail(
  const MotionRequestBindingRecord & value);
[[nodiscard]] std::optional<std::string> invalid_motion_goal_accepted_record_detail(
  const MotionGoalAcceptedRecord & value);
[[nodiscard]] std::optional<std::string> invalid_motion_cancel_submission_record_detail(
  const MotionCancelSubmissionRecord & value);
[[nodiscard]] std::optional<std::string> invalid_definitely_not_submitted_record_detail(
  const DefinitelyNotSubmittedRecord & value);
[[nodiscard]] std::optional<std::string> invalid_motion_terminal_record_detail(
  const MotionTerminalRecord & value);
[[nodiscard]] std::optional<std::string> invalid_motion_stop_proof_record_detail(
  const MotionStopProofRecord & value);
[[nodiscard]] bool valid_motion_goal_uuid(const MotionGoalUuid & value) noexcept;
[[nodiscard]] bool exact_cancel_preceded_terminal(
  const MotionTerminalRecord & value) noexcept;

[[nodiscard]] const char * to_string(TrajectoryActionTerminalState value) noexcept;
[[nodiscard]] const char * to_string(LocalNonSubmissionReason value) noexcept;
[[nodiscard]] const char * to_string(TransportNonSubmissionReason value) noexcept;
[[nodiscard]] const char * to_string(MotionCancellationReason value) noexcept;

class MotionRequestBindingReceipt
{
public:
  MotionRequestBindingReceipt(MotionRequestBindingReceipt &&) noexcept = default;
  MotionRequestBindingReceipt & operator=(MotionRequestBindingReceipt &&) noexcept = default;
  MotionRequestBindingReceipt(const MotionRequestBindingReceipt &) = delete;
  MotionRequestBindingReceipt & operator=(const MotionRequestBindingReceipt &) = delete;
  ~MotionRequestBindingReceipt() = default;

  [[nodiscard]] const MotionRequestBindingRecord & record() const noexcept {return record_;}

private:
  explicit MotionRequestBindingReceipt(MotionRequestBindingRecord record);

  MotionRequestBindingRecord record_;

  friend class PendingOperationLedger;
  friend PreGraspExecutionContractResult<MotionRequestBindingReceipt>
  make_motion_request_binding_receipt(
    const OperationTicket &, const PreGraspExecutionRequest &);
};

class MotionIngressFaultReceipt
{
public:
  MotionIngressFaultReceipt(MotionIngressFaultReceipt &&) noexcept = default;
  MotionIngressFaultReceipt & operator=(MotionIngressFaultReceipt &&) noexcept = default;
  MotionIngressFaultReceipt(const MotionIngressFaultReceipt &) = delete;
  MotionIngressFaultReceipt & operator=(const MotionIngressFaultReceipt &) = delete;
  ~MotionIngressFaultReceipt() = default;

  [[nodiscard]] const MotionIngressFaultRecord & record() const noexcept {return record_;}

private:
  explicit MotionIngressFaultReceipt(MotionIngressFaultRecord record);

  MotionIngressFaultRecord record_;

  friend class PendingOperationLedger;
  friend class MotionIngressAuthority;
};

class MotionStopProofReceipt final
{
public:
  MotionStopProofReceipt(MotionStopProofReceipt && other) noexcept;
  MotionStopProofReceipt & operator=(MotionStopProofReceipt && other) noexcept;
  MotionStopProofReceipt(const MotionStopProofReceipt &) = delete;
  MotionStopProofReceipt & operator=(const MotionStopProofReceipt &) = delete;
  ~MotionStopProofReceipt() = default;

  [[nodiscard]] bool live() const noexcept {return live_;}
  [[nodiscard]] const MotionStopProofRecord & record() const noexcept {return record_;}

private:
  explicit MotionStopProofReceipt(MotionStopProofRecord record);

  MotionStopProofRecord record_;
  bool live_{true};

  friend class PendingOperationLedger;
  friend class PreGraspStopSampler;
};

class MotionGoalAcceptedReceipt
{
public:
  MotionGoalAcceptedReceipt(MotionGoalAcceptedReceipt &&) noexcept = default;
  MotionGoalAcceptedReceipt & operator=(MotionGoalAcceptedReceipt &&) noexcept = default;
  MotionGoalAcceptedReceipt(const MotionGoalAcceptedReceipt &) = delete;
  MotionGoalAcceptedReceipt & operator=(const MotionGoalAcceptedReceipt &) = delete;
  ~MotionGoalAcceptedReceipt() = default;

  [[nodiscard]] const MotionGoalAcceptedRecord & record() const noexcept {return record_;}

private:
  explicit MotionGoalAcceptedReceipt(MotionGoalAcceptedRecord record);

  MotionGoalAcceptedRecord record_;

  friend class PendingOperationLedger;
  friend class MotionIngressAuthority;
};

class DefinitelyNotSubmittedReceipt final
{
public:
  DefinitelyNotSubmittedReceipt(DefinitelyNotSubmittedReceipt && other) noexcept;
  DefinitelyNotSubmittedReceipt & operator=(DefinitelyNotSubmittedReceipt && other) noexcept;
  DefinitelyNotSubmittedReceipt(const DefinitelyNotSubmittedReceipt &) = delete;
  DefinitelyNotSubmittedReceipt & operator=(const DefinitelyNotSubmittedReceipt &) = delete;
  ~DefinitelyNotSubmittedReceipt() = default;

  [[nodiscard]] bool live() const noexcept {return live_;}
  [[nodiscard]] const DefinitelyNotSubmittedRecord & record() const noexcept {return record_;}

private:
  explicit DefinitelyNotSubmittedReceipt(DefinitelyNotSubmittedRecord record);

  DefinitelyNotSubmittedRecord record_;
  bool live_{true};

  friend class PendingOperationLedger;
  friend class MotionIngressAuthority;
};

class MotionCancelSubmissionReceipt
{
public:
  MotionCancelSubmissionReceipt(MotionCancelSubmissionReceipt &&) noexcept = default;
  MotionCancelSubmissionReceipt & operator=(MotionCancelSubmissionReceipt &&) noexcept = default;
  MotionCancelSubmissionReceipt(const MotionCancelSubmissionReceipt &) = delete;
  MotionCancelSubmissionReceipt & operator=(const MotionCancelSubmissionReceipt &) = delete;
  ~MotionCancelSubmissionReceipt() = default;

  [[nodiscard]] const MotionCancelSubmissionRecord & record() const noexcept {return record_;}

private:
  explicit MotionCancelSubmissionReceipt(MotionCancelSubmissionRecord record);

  MotionCancelSubmissionRecord record_;

  friend class PendingOperationLedger;
  friend class MotionIngressAuthority;
};

class MotionTerminalEvidence
{
public:
  MotionTerminalEvidence(MotionTerminalEvidence &&) noexcept = default;
  MotionTerminalEvidence & operator=(MotionTerminalEvidence &&) noexcept = default;
  MotionTerminalEvidence(const MotionTerminalEvidence &) = delete;
  MotionTerminalEvidence & operator=(const MotionTerminalEvidence &) = delete;
  ~MotionTerminalEvidence() = default;

  [[nodiscard]] const MotionTerminalRecord & record() const noexcept {return record_;}

private:
  explicit MotionTerminalEvidence(MotionTerminalRecord record);

  MotionTerminalRecord record_;

  friend class PendingOperationLedger;
  friend class MotionIngressAuthority;
};

}  // namespace restocker_task_executor
