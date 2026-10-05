// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace restocker_task_executor
{

enum class RestockTaskState
{
  kIdle,
  kValidateScene,
  kSelectPair,
  kReserveTask,
  kGenerateGrasps,
  kPlanPreGrasp,
  kExecutePreGrasp,
  // The jaws must straddle the product before the approach is even planned: with the gripper at
  // its spawn width the goal state is in collision with the product, so planning fails outright.
  kOpenGripperForApproach,
  kPlanApproach,
  kExecuteApproach,
  kCloseGripper,
  kVerifyGrasp,
  kAttachTransaction,
  kPlanRetract,
  kExecuteRetract,
  kObserveDestination,
  kGeneratePlacement,
  // Short Cartesian stock-side egress before constrained PreInsert.
  kPlanCarryStart,
  kExecuteCarryStart,
  kPlanPreInsert,
  kExecutePreInsert,
  kPlanInsert,
  kExecuteInsert,
  kOpenGripper,
  // Physical place-detach only: jaws are open and Gazebo has released the product. The semantic
  // commit waits for a post-retreat lane survey so camera evidence can prove column growth.
  kDetachTransaction,
  kPlanRetreat,
  kExecuteRetreat,
  // Measure the destination lane from the retreat viewpoint (already aimed; measure_only).
  kSurveyDestination,
  // Semantic place-detach commit once last_verified > released_at and the column has grown.
  kCommitDetachment,
  kVerifyPlacement,
  kUpdateInventory,
  kCancelActiveMotion,
  kReleaseTask,
  kRecover,
  kFault,
  kRequestOperator,
  kComplete,
  kCanceled,
  // Milestone 10 §6 (Card 062): the jaws were closed at a grasp and the product is not held, so
  // they open to the closed candidate's approach clearance before any plan leaves the grasp.
  // Appended so the other states keep their positions.
  kOpenGripperForEscape,
};

enum class RestockTaskCommand
{
  kNone,
  kValidateScene,
  kSelectPair,
  kReserveTask,
  kGenerateGrasps,
  kPlanPreGrasp,
  kExecutePreGrasp,
  kPlanApproach,
  kExecuteApproach,
  kCloseGripper,
  kVerifyGrasp,
  kRunAttachTransaction,
  kPlanRetract,
  kExecuteRetract,
  kObserveDestination,
  kGeneratePlacement,
  kPlanCarryStart,
  kExecuteCarryStart,
  kPlanPreInsert,
  kExecutePreInsert,
  kPlanInsert,
  kExecuteInsert,
  kOpenGripper,
  kRunDetachTransaction,
  kPlanRetreat,
  kExecuteRetreat,
  kSurveyDestination,
  kRunCommitDetachment,
  kVerifyPlacement,
  kUpdateInventory,
  kCancelMotionAndVerifyStop,
  kReleaseTaskReservation,
  kExecuteRecovery,
  kInhibitMotion,
  kRequestOperator,
};

enum class RestockTaskEvent
{
  kReservationAcquired,
  kSelectionSuperseded,
  kExecutionOperationStarted,
  kTrajectoryExecutionAccepted,
  kOperationSucceeded,
  kRetryableFailure,
  kTerminalFailure,
  kTimeout,
  kTaskDeadlineExceeded,
  kCancelRequested,
  kDrainRequested,
  kSafeAbortRequested,
  kTransactionRolledBack,
  kTransactionInhibited,
  // Graded recovery rung 5 (Milestone 10 §6, Card 051): requested from the fault boundary only,
  // at most once per goal; routes the machine through the cleanup (retreat / release / done)
  // and ends in the typed recoverable-skip terminal instead of latching the operator.
  kRecoverableSkipRequested,
};

enum class RestockTaskFault
{
  kNone,
  kCanceled,
  kOperationFailed,
  kRetryExhausted,
  kTimedOut,
  kTransactionRolledBack,
  kExternalInconsistency,
  kRecoveryFailed,
  kInvalidConfiguration,
  kInvalidEvent,
};

struct RestockTaskConfig
{
  std::size_t max_operation_retries{1U};
  std::size_t max_recovery_attempts{2U};
  // Bounds how often a stale-but-unmutated reservation precondition may send the task back to
  // fresh observation. Observations advance entity revisions continuously, so losing this race
  // once is ordinary; losing it repeatedly means the world is unstable.
  std::size_t max_selection_restarts{4U};
  std::chrono::milliseconds validation_timeout{3000};
  std::chrono::milliseconds planning_timeout{5000};
  std::chrono::milliseconds execution_timeout{15000};
  std::chrono::milliseconds transaction_timeout{30000};
  std::chrono::milliseconds recovery_timeout{15000};
  std::chrono::milliseconds total_timeout{120000};
  // Milestone 10 §6 (Card 051): the bounded retreat that may be commanded after the whole-task
  // deadline expires, measured from the expiry — separate from total_timeout, which has already
  // lapsed when this budget starts.
  std::chrono::milliseconds deadline_retreat_timeout{60000};
};

struct RestockTaskTransition
{
  bool accepted{false};
  RestockTaskState state{RestockTaskState::kIdle};
  RestockTaskCommand command{RestockTaskCommand::kNone};
  RestockTaskFault fault{RestockTaskFault::kNone};
  std::size_t attempt{0U};
  std::size_t recovery_attempt{0U};
  std::chrono::milliseconds timeout{0};
  std::chrono::milliseconds total_timeout{0};
  bool cancel_requested{false};
  bool safe_abort_requested{false};
  // Set when the whole-task deadline routed the termination (never alongside a user cancel that
  // won first). The guards that otherwise refuse every execution under termination read this to
  // recognise the bounded cleanup retreat of Milestone 10 §6 (Card 051).
  bool task_deadline_exceeded{false};
  // Set once kRecoverableSkipRequested has been admitted (Milestone 10 §6 rung 5, Card 051):
  // the driver's plan short-circuit and identity guard read this to recognise the cleanup
  // retreat as a legitimate exit — visible, never a silenced guard.
  bool recoverable_skip{false};
  // Set when a user cancel, drain or safe abort arrives (Milestone 10 §6, Card 051 review
  // blocker): it revokes any cleanup-retreat authority the whole-task deadline had armed, so
  // the guards stop in place — an operator's stop never yields to a later deadline.
  bool deadline_retreat_revoked{false};
  bool first_trajectory_may_have_started{false};
  bool current_execution_may_have_started{false};
  std::uint64_t current_execution_operation_generation{0};
  bool object_held{false};
  bool reservation_active{false};
  std::string detail;
};

class RestockTaskMachine
{
public:
  explicit RestockTaskMachine(RestockTaskConfig config = {});

  [[nodiscard]] RestockTaskTransition begin();
  [[nodiscard]] RestockTaskTransition dispatch(
    RestockTaskEvent event, std::string detail = {},
    std::optional<std::uint64_t> operation_generation = std::nullopt);
  [[nodiscard]] RestockTaskTransition status() const;
  [[nodiscard]] std::size_t selection_restarts() const noexcept;
  [[nodiscard]] bool terminal() const noexcept;
  [[nodiscard]] bool motion_inhibited() const noexcept;

private:
  [[nodiscard]] RestockTaskTransition handle_success();
  [[nodiscard]] RestockTaskTransition handle_cancel(std::string detail);
  [[nodiscard]] RestockTaskTransition transition_to(
    RestockTaskState next, std::string detail = {});
  [[nodiscard]] RestockTaskTransition begin_recovery(
    RestockTaskFault fault, std::string detail);
  [[nodiscard]] RestockTaskTransition begin_fault(
    RestockTaskFault fault, std::string detail);
  [[nodiscard]] RestockTaskTransition reject(RestockTaskFault fault, std::string detail) const;
  [[nodiscard]] RestockTaskTransition retry_or_recover(std::string detail);
  [[nodiscard]] bool termination_requested() const noexcept;

  RestockTaskConfig config_;
  RestockTaskState state_{RestockTaskState::kIdle};
  RestockTaskState recovery_resume_state_{RestockTaskState::kValidateScene};
  RestockTaskState canceled_motion_state_{RestockTaskState::kIdle};
  RestockTaskFault fault_{RestockTaskFault::kNone};
  std::size_t attempt_{0U};
  std::size_t recovery_attempt_{0U};
  std::size_t selection_restart_{0U};
  bool cancel_requested_{false};
  bool safe_abort_requested_{false};
  // Rung 5 (Card 051): the recoverable-skip flag the cleanup routes on, and the single-shot
  // guard — a second skip request for the same goal is refused so the ladder always terminates.
  bool recoverable_skip_{false};
  bool recoverable_skip_attempted_{false};
  // Card 051 review blocker: a user cancel / drain / safe abort revokes the deadline's
  // cleanup-retreat authority (stop in place wins whenever the operator asked for it).
  bool deadline_retreat_revoked_{false};
  // True once the reservation is being released because the transfer finished, not because the
  // task is being torn down. Makes the release boundary end in kComplete instead of kCanceled.
  bool releasing_after_completion_{false};
  bool first_trajectory_may_have_started_{false};
  bool current_execution_may_have_started_{false};
  std::uint64_t current_execution_operation_generation_{0};
  bool object_held_{false};
  bool reservation_active_{false};
  // True once the physical place-detach half has applied. Distinct from object_held_: an early
  // cancellation also has an empty gripper without ever having released a product into a lane.
  bool physical_detach_applied_{false};
  // True once the semantic place-detach commit has applied. Physical detach alone clears
  // object_held_ but leaves the reservation un-releasable until this is set: cancellation after
  // the jaws open still owes the destination survey and commit.
  bool placement_committed_{false};
  // Card 062: set when the jaws are commanded closed at a grasp, cleared once the product is held
  // or the escape open succeeded. While set, a route that would leave the grasp is diverted
  // through kOpenGripperForEscape and then continues at grasp_escape_resume_state_.
  bool jaws_closed_on_unheld_product_{false};
  RestockTaskState grasp_escape_resume_state_{RestockTaskState::kPlanRetreat};
  bool task_deadline_exceeded_{false};
  bool configuration_valid_{true};
  std::string detail_;
};

[[nodiscard]] const char * to_string(RestockTaskState state) noexcept;
[[nodiscard]] const char * to_string(RestockTaskCommand command) noexcept;
[[nodiscard]] const char * to_string(RestockTaskFault fault) noexcept;

}  // namespace restocker_task_executor
