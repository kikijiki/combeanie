// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <set>
#include <utility>

#include "restocker_task_executor/restock_action_contract.hpp"

namespace restocker_task_executor
{
namespace
{

using Action = restocker_interfaces::action::RestockProduct;
using namespace std::chrono_literals;

TEST(RestockActionGoal, ConvertsUnspecifiedAndExplicitSelectors)
{
  Action::Goal automatic;
  const auto automatic_result = selection_request_from_goal(automatic);
  ASSERT_TRUE(automatic_result);
  EXPECT_FALSE(automatic_result.value().object_id);
  EXPECT_FALSE(automatic_result.value().lane_id);

  Action::Goal explicit_goal;
  explicit_goal.has_object_id = true;
  explicit_goal.object_id = 17;
  explicit_goal.has_lane_id = true;
  explicit_goal.lane_id = "lane_02";
  const auto explicit_result = selection_request_from_goal(explicit_goal);
  ASSERT_TRUE(explicit_result);
  EXPECT_EQ(explicit_result.value().object_id, restocker_world_state::ObjectId{17});
  EXPECT_EQ(explicit_result.value().lane_id, restocker_world_state::LaneId{"lane_02"});
}

// Milestone 10 §1 confirmed-identity contract (Card 037): an object-only goal names the
// product and leaves the destination to selection; the contract must not invent a lane.
TEST(RestockActionGoal, ConvertsAnObjectOnlySelectorWithoutInventingALane)
{
  Action::Goal object_only;
  object_only.has_object_id = true;
  object_only.object_id = 17;
  const auto result = selection_request_from_goal(object_only);
  ASSERT_TRUE(result);
  EXPECT_EQ(result.value().object_id, restocker_world_state::ObjectId{17});
  EXPECT_FALSE(result.value().lane_id);

  Action::Goal lane_only;
  lane_only.has_lane_id = true;
  lane_only.lane_id = "lane_02";
  const auto lane_result = selection_request_from_goal(lane_only);
  ASSERT_TRUE(lane_result);
  EXPECT_FALSE(lane_result.value().object_id);
  EXPECT_EQ(lane_result.value().lane_id, restocker_world_state::LaneId{"lane_02"});
}

TEST(RestockActionGoal, RejectsEveryAmbiguousOptionalEncoding)
{
  Action::Goal present_zero;
  present_zero.has_object_id = true;
  EXPECT_FALSE(selection_request_from_goal(present_zero));

  Action::Goal absent_nonzero;
  absent_nonzero.object_id = 17;
  EXPECT_FALSE(selection_request_from_goal(absent_nonzero));

  Action::Goal present_empty;
  present_empty.has_lane_id = true;
  EXPECT_FALSE(selection_request_from_goal(present_empty));

  Action::Goal absent_nonempty;
  absent_nonempty.lane_id = "lane_02";
  EXPECT_FALSE(selection_request_from_goal(absent_nonempty));
}

TEST(RestockActionState, ExhaustivelyMapsUniqueStableFeedbackConstants)
{
  const std::array expectations{
    std::pair{RestockTaskState::kIdle, Action::Feedback::STATE_IDLE},
    std::pair{RestockTaskState::kValidateScene, Action::Feedback::STATE_VALIDATE_SCENE},
    std::pair{RestockTaskState::kSelectPair, Action::Feedback::STATE_SELECT_PAIR},
    std::pair{RestockTaskState::kReserveTask, Action::Feedback::STATE_RESERVE_TASK},
    std::pair{RestockTaskState::kGenerateGrasps, Action::Feedback::STATE_GENERATE_GRASPS},
    std::pair{RestockTaskState::kPlanPreGrasp, Action::Feedback::STATE_PLAN_PRE_GRASP},
    std::pair{RestockTaskState::kExecutePreGrasp, Action::Feedback::STATE_EXECUTE_PRE_GRASP},
    std::pair{
      RestockTaskState::kOpenGripperForApproach,
      Action::Feedback::STATE_OPEN_GRIPPER_FOR_APPROACH},
    std::pair{RestockTaskState::kPlanApproach, Action::Feedback::STATE_PLAN_APPROACH},
    std::pair{RestockTaskState::kExecuteApproach, Action::Feedback::STATE_EXECUTE_APPROACH},
    std::pair{RestockTaskState::kCloseGripper, Action::Feedback::STATE_CLOSE_GRIPPER},
    std::pair{RestockTaskState::kVerifyGrasp, Action::Feedback::STATE_VERIFY_GRASP},
    std::pair{RestockTaskState::kAttachTransaction, Action::Feedback::STATE_ATTACH_TRANSACTION},
    std::pair{RestockTaskState::kPlanRetract, Action::Feedback::STATE_PLAN_RETRACT},
    std::pair{RestockTaskState::kExecuteRetract, Action::Feedback::STATE_EXECUTE_RETRACT},
    std::pair{
      RestockTaskState::kObserveDestination, Action::Feedback::STATE_OBSERVE_DESTINATION},
    std::pair{
      RestockTaskState::kGeneratePlacement, Action::Feedback::STATE_GENERATE_PLACEMENT},
    std::pair{RestockTaskState::kPlanCarryStart, Action::Feedback::STATE_PLAN_CARRY_START},
    std::pair{
      RestockTaskState::kExecuteCarryStart, Action::Feedback::STATE_EXECUTE_CARRY_START},
    std::pair{RestockTaskState::kPlanPreInsert, Action::Feedback::STATE_PLAN_PRE_INSERT},
    std::pair{RestockTaskState::kExecutePreInsert, Action::Feedback::STATE_EXECUTE_PRE_INSERT},
    std::pair{RestockTaskState::kPlanInsert, Action::Feedback::STATE_PLAN_INSERT},
    std::pair{RestockTaskState::kExecuteInsert, Action::Feedback::STATE_EXECUTE_INSERT},
    std::pair{RestockTaskState::kOpenGripper, Action::Feedback::STATE_OPEN_GRIPPER},
    std::pair{RestockTaskState::kDetachTransaction, Action::Feedback::STATE_DETACH_TRANSACTION},
    std::pair{RestockTaskState::kPlanRetreat, Action::Feedback::STATE_PLAN_RETREAT},
    std::pair{RestockTaskState::kExecuteRetreat, Action::Feedback::STATE_EXECUTE_RETREAT},
    std::pair{RestockTaskState::kSurveyDestination, Action::Feedback::STATE_SURVEY_DESTINATION},
    std::pair{RestockTaskState::kCommitDetachment, Action::Feedback::STATE_COMMIT_DETACHMENT},
    std::pair{RestockTaskState::kVerifyPlacement, Action::Feedback::STATE_VERIFY_PLACEMENT},
    std::pair{RestockTaskState::kUpdateInventory, Action::Feedback::STATE_UPDATE_INVENTORY},
    std::pair{
      RestockTaskState::kCancelActiveMotion, Action::Feedback::STATE_CANCEL_ACTIVE_MOTION},
    std::pair{RestockTaskState::kReleaseTask, Action::Feedback::STATE_RELEASE_TASK},
    std::pair{RestockTaskState::kRecover, Action::Feedback::STATE_RECOVER},
    std::pair{RestockTaskState::kFault, Action::Feedback::STATE_FAULT},
    std::pair{RestockTaskState::kRequestOperator, Action::Feedback::STATE_REQUEST_OPERATOR},
    std::pair{RestockTaskState::kComplete, Action::Feedback::STATE_COMPLETE},
    std::pair{RestockTaskState::kCanceled, Action::Feedback::STATE_CANCELED},
    std::pair{
      RestockTaskState::kOpenGripperForEscape, Action::Feedback::STATE_OPEN_GRIPPER_FOR_ESCAPE},
  };
  std::set<std::uint8_t> observed;
  for (const auto & [state, expected] : expectations) {
    EXPECT_EQ(action_state(state), expected);
    EXPECT_TRUE(observed.insert(action_state(state)).second);
  }
  EXPECT_EQ(observed.size(), 39U);
}

TEST(RestockActionStatus, ExhaustivelyMapsStableResultConstants)
{
  const std::array expectations{
    std::pair{RestockActionOutcome::kUnset, Action::Result::STATUS_UNSET},
    std::pair{RestockActionOutcome::kSucceeded, Action::Result::STATUS_SUCCEEDED},
    std::pair{RestockActionOutcome::kCanceled, Action::Result::STATUS_CANCELED},
    std::pair{
      RestockActionOutcome::kNoCompatiblePair, Action::Result::STATUS_NO_COMPATIBLE_PAIR},
    std::pair{
      RestockActionOutcome::kObservationEvidenceStale,
      Action::Result::STATUS_OBSERVATION_EVIDENCE_STALE},
    std::pair{
      RestockActionOutcome::kValidationFailed, Action::Result::STATUS_VALIDATION_FAILED},
    std::pair{RestockActionOutcome::kPlanningFailed, Action::Result::STATUS_PLANNING_FAILED},
    std::pair{RestockActionOutcome::kExecutionFailed, Action::Result::STATUS_EXECUTION_FAILED},
    std::pair{
      RestockActionOutcome::kVerificationFailed, Action::Result::STATUS_VERIFICATION_FAILED},
    std::pair{
      RestockActionOutcome::kRecoveryExhausted, Action::Result::STATUS_RECOVERY_EXHAUSTED},
    std::pair{
      RestockActionOutcome::kExternalInconsistency,
      Action::Result::STATUS_EXTERNAL_INCONSISTENCY},
    std::pair{
      RestockActionOutcome::kOperatorRequired, Action::Result::STATUS_OPERATOR_REQUIRED},
    std::pair{RestockActionOutcome::kShutdown, Action::Result::STATUS_SHUTDOWN},
    std::pair{
      RestockActionOutcome::kRecoverableSkip, Action::Result::STATUS_SKIPPED_RECOVERABLE},
    std::pair{RestockActionOutcome::kInternalError, Action::Result::STATUS_INTERNAL_ERROR},
  };
  std::set<std::uint16_t> observed;
  for (const auto & [outcome, expected] : expectations) {
    EXPECT_EQ(action_status(outcome), expected);
    EXPECT_TRUE(observed.insert(action_status(outcome)).second);
  }
  EXPECT_EQ(observed.size(), expectations.size());
}

TEST(RestockActionMessages, EncodesFeedbackAndResultWithoutAmbiguousOptionals)
{
  RestockTaskTransition transition;
  transition.state = RestockTaskState::kPlanApproach;
  transition.attempt = 2;
  transition.recovery_attempt = 1;
  transition.detail = "planning";
  SelectedTaskPair pair;
  pair.object_id = restocker_world_state::ObjectId{17};
  pair.lane_id = restocker_world_state::LaneId{"lane_02"};

  const auto feedback = make_action_feedback(
    transition, 1234ms, pair, RestockActionOutcome::kPlanningFailed);
  EXPECT_EQ(feedback.state, Action::Feedback::STATE_PLAN_APPROACH);
  EXPECT_EQ(feedback.attempt, 2U);
  EXPECT_EQ(feedback.recovery_attempt, 1U);
  EXPECT_EQ(feedback.elapsed_sim_time.sec, 1);
  EXPECT_EQ(feedback.elapsed_sim_time.nanosec, 234'000'000U);
  EXPECT_TRUE(feedback.has_selected_object_id);
  EXPECT_EQ(feedback.selected_object_id, 17U);
  EXPECT_TRUE(feedback.has_selected_lane_id);
  EXPECT_EQ(feedback.selected_lane_id, "lane_02");
  EXPECT_EQ(feedback.latest_status, Action::Result::STATUS_PLANNING_FAILED);
  EXPECT_EQ(feedback.detail, "planning");

  RestockActionMetrics metrics;
  metrics.initial_world_revision = 4;
  metrics.final_world_revision = 9;
  metrics.attempt_count = 3;
  metrics.retry_count = 2;
  metrics.elapsed = -10ms;
  metrics.minimum_clearance_m = 0.012;
  const auto result = make_action_result(
    RestockActionOutcome::kSucceeded, metrics, "complete");
  EXPECT_EQ(result.status, Action::Result::STATUS_SUCCEEDED);
  EXPECT_EQ(result.initial_world_revision, 4U);
  EXPECT_EQ(result.final_world_revision, 9U);
  EXPECT_EQ(result.attempt_count, 3U);
  EXPECT_EQ(result.retry_count, 2U);
  EXPECT_EQ(result.elapsed_sim_time.sec, 0);
  EXPECT_EQ(result.elapsed_sim_time.nanosec, 0U);
  EXPECT_DOUBLE_EQ(result.minimum_clearance_m, 0.012);
  EXPECT_EQ(result.detail, "complete");

  const auto no_pair = make_action_feedback(transition, 0ms, std::nullopt);
  EXPECT_FALSE(no_pair.has_selected_object_id);
  EXPECT_EQ(no_pair.selected_object_id, 0U);
  EXPECT_FALSE(no_pair.has_selected_lane_id);
  EXPECT_TRUE(no_pair.selected_lane_id.empty());
}

TEST(RestockActionMessages, SaturatesWireTypesInsteadOfWrapping)
{
  RestockTaskTransition transition;
  transition.attempt = std::numeric_limits<std::size_t>::max();
  transition.recovery_attempt = std::numeric_limits<std::size_t>::max();
  const auto feedback = make_action_feedback(
    transition, std::chrono::milliseconds::max(), std::nullopt);
  EXPECT_EQ(feedback.attempt, std::numeric_limits<std::uint32_t>::max());
  EXPECT_EQ(feedback.recovery_attempt, std::numeric_limits<std::uint32_t>::max());
  EXPECT_EQ(feedback.elapsed_sim_time.sec, std::numeric_limits<std::int32_t>::max());
  EXPECT_EQ(feedback.elapsed_sim_time.nanosec, 999'999'999U);
}

// Card 086 stage 1b: the motion evidence reaches the result only for statuses where the world and
// the coordinator are not in doubt.
TEST(RestockActionMessages, MotionEvidenceReachesTheResultOnlyForTrustworthyStatuses)
{
  RestockActionMetrics metrics;
  metrics.motion_definitely_not_started = true;
  metrics.execution_reached_terminal_stop = true;
  for (const auto outcome :
    {RestockActionOutcome::kCanceled, RestockActionOutcome::kValidationFailed,
      RestockActionOutcome::kPlanningFailed, RestockActionOutcome::kExecutionFailed,
      RestockActionOutcome::kVerificationFailed, RestockActionOutcome::kRecoveryExhausted})
  {
    const auto result = make_action_result(outcome, metrics);
    EXPECT_TRUE(result.motion_definitely_not_started) << static_cast<int>(outcome);
    EXPECT_TRUE(result.execution_reached_terminal_stop) << static_cast<int>(outcome);
  }
  for (const auto outcome :
    {RestockActionOutcome::kExternalInconsistency, RestockActionOutcome::kOperatorRequired,
      RestockActionOutcome::kShutdown, RestockActionOutcome::kInternalError,
      RestockActionOutcome::kUnset})
  {
    const auto result = make_action_result(outcome, metrics);
    EXPECT_FALSE(result.motion_definitely_not_started) << static_cast<int>(outcome);
    EXPECT_FALSE(result.execution_reached_terminal_stop) << static_cast<int>(outcome);
  }
}

TEST(RestockActionMessages, DefaultMetricsCarryNoMotionEvidence)
{
  const auto result = make_action_result(RestockActionOutcome::kPlanningFailed, {});
  EXPECT_FALSE(result.motion_definitely_not_started);
  EXPECT_FALSE(result.execution_reached_terminal_stop);
}

}  // namespace
}  // namespace restocker_task_executor
