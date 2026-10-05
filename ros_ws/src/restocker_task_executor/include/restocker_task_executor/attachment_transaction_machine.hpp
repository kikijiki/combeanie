// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <string>

namespace restocker_task_executor
{

enum class AttachmentTransactionKind
{
  kAttach,
  kDetach,
};

enum class AttachmentTransactionState
{
  kIdle,
  kAcquireSceneLease,
  kReconcileSceneLeaseOutcome,
  kCommandGazebo,
  kReconcileGazeboOutcome,
  kProvePlacement,
  kApplyMoveItDiff,
  kVerifyMoveItDiff,
  kCommitWorldState,
  kReleaseSceneLease,
  kVerifyProjection,
  kCompensateGazebo,
  kCompensateMoveIt,
  kReleaseCompensationLease,
  kVerifyCompensationProjection,
  kReconcileCommittedState,
  kSucceeded,
  kRolledBack,
  kInhibitMotion,
};

enum class AttachmentTransactionCommand
{
  kNone,
  kAcquireSceneLease,
  kQuerySceneLeaseOutcome,
  kSendGazeboCommand,
  kQueryGazeboOutcome,
  kVerifySettledPlacement,
  kApplyMoveItDiff,
  kReadBackMoveItScene,
  kCommitWorldState,
  kReleaseSceneLease,
  kWaitForProjection,
  kSendInverseGazeboCommand,
  kApplyInverseMoveItDiff,
  kReleaseCompensationLease,
  kWaitForCompensationProjection,
  kReconcileCommittedState,
  kInhibitMotion,
};

enum class AttachmentTransactionEvent
{
  kOperationSucceeded,
  kConfirmedNotApplied,
  kTerminalFailure,
  kOutcomeUnknown,
  kRetryableFailure,
  kTimeout,
  kCancelRequested,
};

enum class AttachmentTransactionFault
{
  kNone,
  kCanceled,
  kOperationFailed,
  kRetryExhausted,
  kOutcomeUnknown,
  kInvalidConfiguration,
  kInvalidEvent,
  kCompensationFailed,
  kExternalInconsistency,
};

struct AttachmentTransactionConfig
{
  std::size_t max_operation_retries{1U};
  std::size_t max_reconciliation_retries{3U};
  std::chrono::milliseconds operation_timeout{5000};
  std::chrono::milliseconds reconciliation_timeout{10000};
  std::chrono::milliseconds compensation_timeout{10000};
};

struct AttachmentTransactionTransition
{
  bool accepted{false};
  AttachmentTransactionState state{AttachmentTransactionState::kIdle};
  AttachmentTransactionCommand command{AttachmentTransactionCommand::kNone};
  AttachmentTransactionFault fault{AttachmentTransactionFault::kNone};
  std::size_t attempt{0U};
  std::chrono::milliseconds timeout{0};
  bool cancel_requested{false};
  std::string detail;
};

class AttachmentTransactionMachine
{
public:
  explicit AttachmentTransactionMachine(AttachmentTransactionConfig config = {});

  [[nodiscard]] AttachmentTransactionTransition begin(AttachmentTransactionKind kind);

  [[nodiscard]] AttachmentTransactionTransition dispatch(
    AttachmentTransactionEvent event, std::string detail = {});

  [[nodiscard]] AttachmentTransactionTransition status() const;
  [[nodiscard]] AttachmentTransactionKind kind() const noexcept;
  [[nodiscard]] bool terminal() const noexcept;
  [[nodiscard]] bool motion_inhibited() const noexcept;

private:
  [[nodiscard]] AttachmentTransactionTransition transition_to(
    AttachmentTransactionState next, std::string detail = {});
  [[nodiscard]] AttachmentTransactionTransition handle_success();
  [[nodiscard]] AttachmentTransactionTransition reject(
    AttachmentTransactionFault fault, std::string detail) const;
  [[nodiscard]] AttachmentTransactionTransition begin_rollback(
    AttachmentTransactionFault fault, std::string detail);
  [[nodiscard]] AttachmentTransactionTransition inhibit(
    AttachmentTransactionFault fault, std::string detail);
  [[nodiscard]] AttachmentTransactionTransition retry_or_fail(
    AttachmentTransactionFault exhausted_fault, std::string detail);
  [[nodiscard]] std::size_t retry_limit() const noexcept;

  AttachmentTransactionConfig config_;
  AttachmentTransactionKind kind_{AttachmentTransactionKind::kAttach};
  AttachmentTransactionState state_{AttachmentTransactionState::kIdle};
  AttachmentTransactionFault fault_{AttachmentTransactionFault::kNone};
  std::size_t attempt_{0U};
  bool cancel_requested_{false};
  bool scene_lease_acquired_{false};
  bool physical_transition_applied_{false};
  bool moveit_transition_applied_{false};
  bool world_state_committed_{false};
  bool world_commit_outcome_unknown_{false};
  bool configuration_valid_{true};
  std::string detail_;
};

[[nodiscard]] const char * to_string(AttachmentTransactionKind kind) noexcept;
[[nodiscard]] const char * to_string(AttachmentTransactionState state) noexcept;
[[nodiscard]] const char * to_string(AttachmentTransactionCommand command) noexcept;
[[nodiscard]] const char * to_string(AttachmentTransactionFault fault) noexcept;

}  // namespace restocker_task_executor
