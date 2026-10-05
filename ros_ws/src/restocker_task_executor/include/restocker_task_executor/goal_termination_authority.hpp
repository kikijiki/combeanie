// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "restocker_task_executor/motion_operation_evidence.hpp"

namespace restocker_task_executor
{

class GoalAdmissionSlot;

enum class GoalTerminationKind : std::uint8_t
{
  kUserCancel = 0U,
  kCoordinatorDrain = 1U,
  kSafeAbort = 2U,
  kShutdown = 3U,
  kAuthorityLoss = 4U,
  kInboxOverflow = 5U,
  kProtocolFailure = 6U,
  kTaskDeadline = 7U,
  kMotionDeadline = 8U,
};

enum class GoalTerminationScope : std::uint8_t
{
  kGoal = 0U,
  kRegisteredMotion = 1U,
};

using MotionAdmissionSessionGeneration = std::uint64_t;

class MotionAdmissionRegistrationRecord final
{
public:
  MotionAdmissionRegistrationRecord() = default;
  MotionAdmissionRegistrationRecord(
    CoordinatorGoalId goal_id, GoalGeneration goal_generation,
    MotionAdmissionSessionGeneration session_generation,
    OperationTicket operation_ticket,
    ExecutionAttemptGeneration execution_attempt_generation) noexcept;
  MotionAdmissionRegistrationRecord(const MotionAdmissionRegistrationRecord &) = default;
  MotionAdmissionRegistrationRecord(
    MotionAdmissionRegistrationRecord &&) noexcept = default;
  MotionAdmissionRegistrationRecord & operator=(
    const MotionAdmissionRegistrationRecord &) = delete;
  MotionAdmissionRegistrationRecord & operator=(
    MotionAdmissionRegistrationRecord &&) = delete;

  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept;
  [[nodiscard]] GoalGeneration goal_generation() const noexcept;
  [[nodiscard]] MotionAdmissionSessionGeneration session_generation() const noexcept;
  [[nodiscard]] const OperationTicket & operation_ticket() const noexcept;
  [[nodiscard]] ExecutionAttemptGeneration execution_attempt_generation() const noexcept;

  [[nodiscard]] bool operator==(
    const MotionAdmissionRegistrationRecord &) const noexcept = default;

private:
  CoordinatorGoalId goal_id_{};
  GoalGeneration goal_generation_{0U};
  MotionAdmissionSessionGeneration session_generation_{0U};
  OperationTicket operation_ticket_;
  ExecutionAttemptGeneration execution_attempt_generation_{0U};
};

enum class GoalTerminationSource : std::uint8_t
{
  kNone = 0U,
  kAuthorityObservation = 1U,
  kCoordinatorInbox = 2U,
  kTaskDeadline = 3U,
  kMotionFence = 4U,
};

struct GoalTerminationSourceGeneration
{
  GoalTerminationSource source{GoalTerminationSource::kNone};
  std::uint64_t generation{0U};

  [[nodiscard]] bool operator==(const GoalTerminationSourceGeneration &) const noexcept = default;
};

struct FirstGoalTerminationRecord
{
  FirstGoalTerminationRecord() = default;
  FirstGoalTerminationRecord(
    GoalTerminationScope record_scope,
    CoordinatorGoalId record_goal_id,
    GoalGeneration record_goal_generation,
    GoalTerminationKind record_kind,
    MotionCancellationReason record_cancellation_reason,
    SteadyTime record_arrived_at,
    std::optional<GoalTerminationSourceGeneration> record_source_generation,
    std::shared_ptr<const MotionAdmissionRegistrationRecord> record_motion_registration = {})
  noexcept
  : scope(record_scope),
    goal_id(std::move(record_goal_id)),
    goal_generation(record_goal_generation),
    kind(record_kind),
    cancellation_reason(record_cancellation_reason),
    arrived_at(record_arrived_at),
    source_generation(std::move(record_source_generation)),
    motion_registration(std::move(record_motion_registration))
  {
  }

  GoalTerminationScope scope{GoalTerminationScope::kGoal};
  CoordinatorGoalId goal_id{};
  GoalGeneration goal_generation{0U};
  GoalTerminationKind kind{GoalTerminationKind::kSafeAbort};
  MotionCancellationReason cancellation_reason{MotionCancellationReason::kSafeAbort};
  SteadyTime arrived_at{};
  std::optional<GoalTerminationSourceGeneration> source_generation;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> motion_registration;

  [[nodiscard]] bool operator==(const FirstGoalTerminationRecord &) const noexcept = default;
};

enum class GoalTerminationValidationError : std::uint8_t
{
  kNone = 0U,
  kInvalidScope = 1U,
  kInvalidGoalId = 2U,
  kInvalidGoalGeneration = 3U,
  kInvalidKind = 4U,
  kCancellationReasonMismatch = 5U,
  kInvalidArrivalTime = 6U,
  kInvalidSourceKind = 7U,
  kUnexpectedSource = 8U,
  kMissingSource = 9U,
  kInvalidSourceGeneration = 10U,
  kMissingMotionRegistration = 11U,
  kUnexpectedMotionRegistration = 12U,
  kInvalidMotionRegistration = 13U,
  kMotionRegistrationGoalMismatch = 14U,
};

enum class GoalTerminationLatchStatus : std::uint8_t
{
  kLatched = 0U,
  kAlreadyLatched = 1U,
  kGoalMismatch = 2U,
  kInactive = 3U,
  kInvalidArgument = 4U,
};

enum class GoalTerminationSnapshotStatus : std::uint8_t
{
  kNone = 0U,
  kPresent = 1U,
  kGoalMismatch = 2U,
  kInactive = 3U,
  kInvalidArgument = 4U,
};

class GoalTerminationLatchDecision final
{
public:
  GoalTerminationLatchDecision(const GoalTerminationLatchDecision &) = default;
  GoalTerminationLatchDecision(GoalTerminationLatchDecision && other) noexcept;
  GoalTerminationLatchDecision & operator=(const GoalTerminationLatchDecision &) = default;
  GoalTerminationLatchDecision & operator=(GoalTerminationLatchDecision && other) noexcept;
  ~GoalTerminationLatchDecision() = default;

  [[nodiscard]] GoalTerminationLatchStatus status() const noexcept;
  [[nodiscard]] std::shared_ptr<const FirstGoalTerminationRecord> record() const noexcept;

private:
  friend class GoalAdmissionSlot;

  explicit GoalTerminationLatchDecision(
    GoalTerminationLatchStatus status,
    std::shared_ptr<const FirstGoalTerminationRecord> record) noexcept;

  GoalTerminationLatchStatus status_{GoalTerminationLatchStatus::kInvalidArgument};
  std::shared_ptr<const FirstGoalTerminationRecord> record_;
};

class GoalTerminationSnapshotDecision final
{
public:
  GoalTerminationSnapshotDecision(const GoalTerminationSnapshotDecision &) = default;
  GoalTerminationSnapshotDecision(GoalTerminationSnapshotDecision && other) noexcept;
  GoalTerminationSnapshotDecision & operator=(const GoalTerminationSnapshotDecision &) = default;
  GoalTerminationSnapshotDecision & operator=(GoalTerminationSnapshotDecision && other) noexcept;
  ~GoalTerminationSnapshotDecision() = default;

  [[nodiscard]] GoalTerminationSnapshotStatus status() const noexcept;
  [[nodiscard]] std::shared_ptr<const FirstGoalTerminationRecord> record() const noexcept;

private:
  friend class GoalAdmissionSlot;

  explicit GoalTerminationSnapshotDecision(
    GoalTerminationSnapshotStatus status,
    std::shared_ptr<const FirstGoalTerminationRecord> record) noexcept;

  GoalTerminationSnapshotStatus status_{GoalTerminationSnapshotStatus::kInvalidArgument};
  std::shared_ptr<const FirstGoalTerminationRecord> record_;
};

[[nodiscard]] std::optional<MotionCancellationReason> cancellation_reason_for(
  GoalTerminationKind kind) noexcept;

[[nodiscard]] bool valid_motion_admission_registration_record(
  const MotionAdmissionRegistrationRecord & record) noexcept;

[[nodiscard]] GoalTerminationValidationError validate_goal_termination_record(
  const FirstGoalTerminationRecord & record) noexcept;

[[nodiscard]] bool same_goal_termination_evidence(
  const std::shared_ptr<const FirstGoalTerminationRecord> & lhs,
  const std::shared_ptr<const FirstGoalTerminationRecord> & rhs) noexcept;

[[nodiscard]] std::uint64_t goal_termination_evidence_fingerprint(
  const std::shared_ptr<const FirstGoalTerminationRecord> & value) noexcept;

}  // namespace restocker_task_executor
