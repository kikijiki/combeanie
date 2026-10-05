// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_action_contract.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <utility>

#include <builtin_interfaces/msg/duration.hpp>

namespace restocker_task_executor
{
namespace
{

using Action = restocker_interfaces::action::RestockProduct;

[[nodiscard]] builtin_interfaces::msg::Duration duration_message(
  std::chrono::milliseconds elapsed)
{
  const auto bounded = elapsed < std::chrono::milliseconds::zero() ?
    std::chrono::milliseconds::zero() : elapsed;
  constexpr auto kMillisecondsPerSecond = 1000LL;
  constexpr auto kNanosecondsPerMillisecond = 1'000'000LL;
  const auto milliseconds = bounded.count();
  builtin_interfaces::msg::Duration message;
  const auto seconds = milliseconds / kMillisecondsPerSecond;
  if (seconds >= std::numeric_limits<std::int32_t>::max()) {
    message.sec = std::numeric_limits<std::int32_t>::max();
    message.nanosec = 999'999'999U;
  } else {
    message.sec = static_cast<std::int32_t>(seconds);
    message.nanosec = static_cast<std::uint32_t>(
      (milliseconds % kMillisecondsPerSecond) * kNanosecondsPerMillisecond);
  }
  return message;
}

[[nodiscard]] std::uint32_t bounded_u32(std::size_t value)
{
  return static_cast<std::uint32_t>(
    std::min(value, static_cast<std::size_t>(std::numeric_limits<std::uint32_t>::max())));
}

}  // namespace

SelectionResult<SelectionRequest> selection_request_from_goal(const Action::Goal & goal)
{
  if ((goal.has_object_id && goal.object_id == 0) || (!goal.has_object_id && goal.object_id != 0)) {
    return SelectionResult<SelectionRequest>::failure(
      SelectionError{
        SelectionErrorCode::RequestedIdentityIncomplete,
        "object selector presence and value are inconsistent"});
  }
  if ((goal.has_lane_id && goal.lane_id.empty()) || (!goal.has_lane_id && !goal.lane_id.empty())) {
    return SelectionResult<SelectionRequest>::failure(
      SelectionError{
        SelectionErrorCode::RequestedIdentityIncomplete,
        "lane selector presence and value are inconsistent"});
  }
  SelectionRequest request;
  if (goal.has_object_id) {
    request.object_id = restocker_world_state::ObjectId{goal.object_id};
  }
  if (goal.has_lane_id) {
    request.lane_id = restocker_world_state::LaneId{goal.lane_id};
  }
  return SelectionResult<SelectionRequest>::success(std::move(request));
}

std::uint8_t action_state(RestockTaskState state) noexcept
{
  switch (state) {
    case RestockTaskState::kIdle: return Action::Feedback::STATE_IDLE;
    case RestockTaskState::kValidateScene: return Action::Feedback::STATE_VALIDATE_SCENE;
    case RestockTaskState::kSelectPair: return Action::Feedback::STATE_SELECT_PAIR;
    case RestockTaskState::kReserveTask: return Action::Feedback::STATE_RESERVE_TASK;
    case RestockTaskState::kGenerateGrasps: return Action::Feedback::STATE_GENERATE_GRASPS;
    case RestockTaskState::kPlanPreGrasp: return Action::Feedback::STATE_PLAN_PRE_GRASP;
    case RestockTaskState::kExecutePreGrasp: return Action::Feedback::STATE_EXECUTE_PRE_GRASP;
    case RestockTaskState::kOpenGripperForApproach:
      return Action::Feedback::STATE_OPEN_GRIPPER_FOR_APPROACH;
    case RestockTaskState::kPlanApproach: return Action::Feedback::STATE_PLAN_APPROACH;
    case RestockTaskState::kExecuteApproach: return Action::Feedback::STATE_EXECUTE_APPROACH;
    case RestockTaskState::kCloseGripper: return Action::Feedback::STATE_CLOSE_GRIPPER;
    case RestockTaskState::kVerifyGrasp: return Action::Feedback::STATE_VERIFY_GRASP;
    case RestockTaskState::kAttachTransaction: return Action::Feedback::STATE_ATTACH_TRANSACTION;
    case RestockTaskState::kPlanRetract: return Action::Feedback::STATE_PLAN_RETRACT;
    case RestockTaskState::kExecuteRetract: return Action::Feedback::STATE_EXECUTE_RETRACT;
    case RestockTaskState::kObserveDestination:
      return Action::Feedback::STATE_OBSERVE_DESTINATION;
    case RestockTaskState::kGeneratePlacement:
      return Action::Feedback::STATE_GENERATE_PLACEMENT;
    case RestockTaskState::kPlanCarryStart: return Action::Feedback::STATE_PLAN_CARRY_START;
    case RestockTaskState::kExecuteCarryStart:
      return Action::Feedback::STATE_EXECUTE_CARRY_START;
    case RestockTaskState::kPlanPreInsert: return Action::Feedback::STATE_PLAN_PRE_INSERT;
    case RestockTaskState::kExecutePreInsert: return Action::Feedback::STATE_EXECUTE_PRE_INSERT;
    case RestockTaskState::kPlanInsert: return Action::Feedback::STATE_PLAN_INSERT;
    case RestockTaskState::kExecuteInsert: return Action::Feedback::STATE_EXECUTE_INSERT;
    case RestockTaskState::kOpenGripper: return Action::Feedback::STATE_OPEN_GRIPPER;
    case RestockTaskState::kDetachTransaction: return Action::Feedback::STATE_DETACH_TRANSACTION;
    case RestockTaskState::kPlanRetreat: return Action::Feedback::STATE_PLAN_RETREAT;
    case RestockTaskState::kExecuteRetreat: return Action::Feedback::STATE_EXECUTE_RETREAT;
    case RestockTaskState::kSurveyDestination:
      return Action::Feedback::STATE_SURVEY_DESTINATION;
    case RestockTaskState::kCommitDetachment:
      return Action::Feedback::STATE_COMMIT_DETACHMENT;
    case RestockTaskState::kVerifyPlacement: return Action::Feedback::STATE_VERIFY_PLACEMENT;
    case RestockTaskState::kUpdateInventory: return Action::Feedback::STATE_UPDATE_INVENTORY;
    case RestockTaskState::kCancelActiveMotion:
      return Action::Feedback::STATE_CANCEL_ACTIVE_MOTION;
    case RestockTaskState::kReleaseTask: return Action::Feedback::STATE_RELEASE_TASK;
    case RestockTaskState::kRecover: return Action::Feedback::STATE_RECOVER;
    case RestockTaskState::kFault: return Action::Feedback::STATE_FAULT;
    case RestockTaskState::kRequestOperator: return Action::Feedback::STATE_REQUEST_OPERATOR;
    case RestockTaskState::kComplete: return Action::Feedback::STATE_COMPLETE;
    case RestockTaskState::kCanceled: return Action::Feedback::STATE_CANCELED;
    case RestockTaskState::kOpenGripperForEscape:
      return Action::Feedback::STATE_OPEN_GRIPPER_FOR_ESCAPE;
  }
  return Action::Feedback::STATE_FAULT;
}

std::uint16_t action_status(RestockActionOutcome outcome) noexcept
{
  switch (outcome) {
    case RestockActionOutcome::kUnset: return Action::Result::STATUS_UNSET;
    case RestockActionOutcome::kSucceeded: return Action::Result::STATUS_SUCCEEDED;
    case RestockActionOutcome::kCanceled: return Action::Result::STATUS_CANCELED;
    case RestockActionOutcome::kNoCompatiblePair:
      return Action::Result::STATUS_NO_COMPATIBLE_PAIR;
    case RestockActionOutcome::kObservationEvidenceStale:
      return Action::Result::STATUS_OBSERVATION_EVIDENCE_STALE;
    case RestockActionOutcome::kValidationFailed:
      return Action::Result::STATUS_VALIDATION_FAILED;
    case RestockActionOutcome::kPlanningFailed: return Action::Result::STATUS_PLANNING_FAILED;
    case RestockActionOutcome::kExecutionFailed: return Action::Result::STATUS_EXECUTION_FAILED;
    case RestockActionOutcome::kVerificationFailed:
      return Action::Result::STATUS_VERIFICATION_FAILED;
    case RestockActionOutcome::kRecoveryExhausted:
      return Action::Result::STATUS_RECOVERY_EXHAUSTED;
    case RestockActionOutcome::kExternalInconsistency:
      return Action::Result::STATUS_EXTERNAL_INCONSISTENCY;
    case RestockActionOutcome::kOperatorRequired:
      return Action::Result::STATUS_OPERATOR_REQUIRED;
    case RestockActionOutcome::kShutdown: return Action::Result::STATUS_SHUTDOWN;
    case RestockActionOutcome::kRecoverableSkip:
      return Action::Result::STATUS_SKIPPED_RECOVERABLE;
    case RestockActionOutcome::kInternalError: return Action::Result::STATUS_INTERNAL_ERROR;
  }
  return Action::Result::STATUS_INTERNAL_ERROR;
}

Action::Feedback make_action_feedback(
  const RestockTaskTransition & transition, std::chrono::milliseconds elapsed,
  const std::optional<SelectedTaskPair> & selected_pair,
  RestockActionOutcome latest_outcome, std::string detail)
{
  Action::Feedback feedback;
  feedback.state = action_state(transition.state);
  feedback.attempt = bounded_u32(transition.attempt);
  feedback.recovery_attempt = bounded_u32(transition.recovery_attempt);
  feedback.elapsed_sim_time = duration_message(elapsed);
  feedback.has_selected_object_id = selected_pair.has_value();
  feedback.selected_object_id = selected_pair ? selected_pair->object_id.value : 0;
  feedback.has_selected_lane_id = selected_pair.has_value();
  feedback.selected_lane_id = selected_pair ? selected_pair->lane_id.value : "";
  feedback.latest_status = action_status(latest_outcome);
  feedback.detail = detail.empty() ? transition.detail : std::move(detail);
  return feedback;
}

std::shared_ptr<Action::Feedback> make_action_feedback_message(
  const RestockTaskTransition & transition, std::chrono::milliseconds elapsed,
  const std::optional<SelectedTaskPair> & selected_pair,
  RestockActionOutcome latest_outcome, std::string detail)
{
  return std::make_shared<Action::Feedback>(
    make_action_feedback(
      transition, elapsed, selected_pair, latest_outcome, std::move(detail)));
}

bool outcome_carries_motion_evidence(RestockActionOutcome outcome) noexcept
{
  switch (outcome) {
    case RestockActionOutcome::kSucceeded:
    case RestockActionOutcome::kCanceled:
    case RestockActionOutcome::kNoCompatiblePair:
    case RestockActionOutcome::kObservationEvidenceStale:
    case RestockActionOutcome::kValidationFailed:
    case RestockActionOutcome::kPlanningFailed:
    case RestockActionOutcome::kExecutionFailed:
    case RestockActionOutcome::kVerificationFailed:
    case RestockActionOutcome::kRecoveryExhausted:
    case RestockActionOutcome::kRecoverableSkip:
      return true;
    case RestockActionOutcome::kUnset:
    case RestockActionOutcome::kExternalInconsistency:
    case RestockActionOutcome::kOperatorRequired:
    case RestockActionOutcome::kShutdown:
    case RestockActionOutcome::kInternalError:
      return false;
  }
  return false;
}

Action::Result make_action_result(
  RestockActionOutcome outcome, const RestockActionMetrics & metrics, std::string detail)
{
  Action::Result result;
  result.status = action_status(outcome);
  result.detail = std::move(detail);
  result.initial_world_revision = metrics.initial_world_revision;
  result.final_world_revision = metrics.final_world_revision;
  result.attempt_count = metrics.attempt_count;
  result.retry_count = metrics.retry_count;
  result.elapsed_sim_time = duration_message(metrics.elapsed);
  result.minimum_clearance_m = metrics.minimum_clearance_m;
  if (outcome_carries_motion_evidence(outcome)) {
    result.motion_definitely_not_started = metrics.motion_definitely_not_started;
    result.execution_reached_terminal_stop = metrics.execution_reached_terminal_stop;
  }
  return result;
}

}  // namespace restocker_task_executor
