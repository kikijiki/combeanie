// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include "restocker_task_executor/gripper_port.hpp"
#include "fake_gripper_port.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

[[nodiscard]] GripperGoal usable_goal()
{
  GripperGoal goal;
  goal.target_position_m = 0.012;
  goal.position_tolerance_m = kAttachmentJointToleranceM;
  goal.move_duration = std::chrono::milliseconds{800};
  goal.deadline = std::chrono::seconds{10};
  return goal;
}

TEST(GripperGoal, AcceptsAnInRangeTargetWithUsableToleranceAndDeadline)
{
  EXPECT_TRUE(valid_gripper_goal(usable_goal()));
}

TEST(GripperGoal, AcceptsBothTravelLimitsExactly)
{
  auto goal = usable_goal();
  goal.target_position_m = kFingerJointLowerM;
  EXPECT_TRUE(valid_gripper_goal(goal));

  goal.target_position_m = kFingerJointUpperM;
  EXPECT_TRUE(valid_gripper_goal(goal));
}

TEST(GripperGoal, RejectsNonFiniteTargetsAndTolerances)
{
  auto goal = usable_goal();
  goal.target_position_m = kNaN;
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal = usable_goal();
  goal.target_position_m = kInfinity;
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal = usable_goal();
  goal.position_tolerance_m = kNaN;
  EXPECT_FALSE(valid_gripper_goal(goal));
}

TEST(GripperGoal, RejectsTargetsOutsideTheFingerTravelRange)
{
  // The controller would refuse these anyway; the contract keeps them off the wire.
  auto goal = usable_goal();
  goal.target_position_m = -0.001;
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal = usable_goal();
  goal.target_position_m = kFingerJointUpperM + 0.001;
  EXPECT_FALSE(valid_gripper_goal(goal));
}

TEST(GripperGoal, RejectsNonPositiveVerificationTolerance)
{
  auto goal = usable_goal();
  goal.position_tolerance_m = 0.0;
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal = usable_goal();
  goal.position_tolerance_m = -kAttachmentJointToleranceM;
  EXPECT_FALSE(valid_gripper_goal(goal));
}

TEST(GripperGoal, RejectsNonPositiveDurations)
{
  auto goal = usable_goal();
  goal.move_duration = std::chrono::milliseconds{0};
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal = usable_goal();
  goal.deadline = std::chrono::milliseconds{-1};
  EXPECT_FALSE(valid_gripper_goal(goal));
}

TEST(GripperGoal, RejectsADeadlineShorterThanTheMoveItAuthorises)
{
  // Such a goal could only ever time out, however healthy the controller is.
  auto goal = usable_goal();
  goal.move_duration = std::chrono::seconds{5};
  goal.deadline = std::chrono::seconds{2};
  EXPECT_FALSE(valid_gripper_goal(goal));

  goal.deadline = goal.move_duration;
  EXPECT_TRUE(valid_gripper_goal(goal));
}

TEST(GripperGoal, DefaultsToTheAttachmentBandRatherThanTheControllerConstraint)
{
  // gripper_controller is satisfied at 0.003 m, but the attachment plugin needs 0.0005 m, so the
  // default must be the tighter of the two or a verified goal would not imply a graspable jaw.
  const GripperGoal goal;
  EXPECT_DOUBLE_EQ(goal.position_tolerance_m, kAttachmentJointToleranceM);
  EXPECT_LT(goal.position_tolerance_m, 0.003);
}

TEST(FingersAtTarget, AcceptsBothFingersInsideTheBand)
{
  const double target = 0.012;
  EXPECT_TRUE(
    fingers_at_target(GripperFingerState{target, target}, target, kAttachmentJointToleranceM));
  EXPECT_TRUE(
    fingers_at_target(
      GripperFingerState{target - 0.0004, target + 0.0004}, target,
      kAttachmentJointToleranceM));
}

TEST(FingersAtTarget, TreatsTheBandEdgeAsInside)
{
  // Exactly representable powers of two, so the boundary is the contract's inclusive comparison
  // and not a rounding artefact of the subtraction.
  constexpr double kTarget = 0.015625;     // 2^-6
  constexpr double kTolerance = 0.00048828125;     // 2^-11
  EXPECT_TRUE(
    fingers_at_target(
      GripperFingerState{kTarget - kTolerance, kTarget + kTolerance}, kTarget, kTolerance));
  EXPECT_FALSE(
    fingers_at_target(
      GripperFingerState{kTarget - 2.0 * kTolerance, kTarget}, kTarget, kTolerance));
}

TEST(FingersAtTarget, RejectsWhenEitherFingerMissesTheBand)
{
  const double target = 0.012;
  const double outside = target + 2.0 * kAttachmentJointToleranceM;
  EXPECT_FALSE(
    fingers_at_target(GripperFingerState{outside, target}, target, kAttachmentJointToleranceM));
  EXPECT_FALSE(
    fingers_at_target(GripperFingerState{target, outside}, target, kAttachmentJointToleranceM));
}

TEST(FingersAtTarget, RejectsAJawTheControllerWouldCallGoodButThePluginWouldNot)
{
  // 0.002 m of error passes gripper_controller's 0.003 m goal constraint yet is four times the
  // band the attachment plugin enforces, which is why this port re-verifies.
  const double target = 0.012;
  EXPECT_FALSE(
    fingers_at_target(
      GripperFingerState{target + 0.002, target + 0.002}, target, kAttachmentJointToleranceM));
}

TEST(FingersAtTarget, RejectsNonFiniteMeasurementsAndUnusableTolerances)
{
  const double target = 0.012;
  EXPECT_FALSE(
    fingers_at_target(GripperFingerState{kNaN, target}, target, kAttachmentJointToleranceM));
  EXPECT_FALSE(
    fingers_at_target(GripperFingerState{target, kInfinity}, target, kAttachmentJointToleranceM));
  EXPECT_FALSE(fingers_at_target(GripperFingerState{target, target}, kNaN, 0.001));
  EXPECT_FALSE(fingers_at_target(GripperFingerState{target, target}, target, 0.0));
  EXPECT_FALSE(fingers_at_target(GripperFingerState{target, target}, target, -0.001));
}

TEST(GripperOutcome, OnlyRejectionAndUnavailabilityProveNoFingerCommandWasIssued)
{
  EXPECT_TRUE(gripper_definitely_not_started(GripperOutcome::kRejected));
  EXPECT_TRUE(gripper_definitely_not_started(GripperOutcome::kUnavailable));

  // Everything else may have moved the jaw, so it must not be treated as pre-motion.
  EXPECT_FALSE(gripper_definitely_not_started(GripperOutcome::kSucceeded));
  EXPECT_FALSE(gripper_definitely_not_started(GripperOutcome::kExecutionFailed));
  EXPECT_FALSE(gripper_definitely_not_started(GripperOutcome::kCanceled));
  EXPECT_FALSE(gripper_definitely_not_started(GripperOutcome::kTimedOut));
}

TEST(GripperOutcome, AnUnverifiedPositionStillCountsAsMotionCommanded)
{
  // Reaching kPositionNotVerified means the controller ran the trajectory to completion, so the
  // fingers moved even though the grasp cannot be trusted.
  EXPECT_FALSE(gripper_definitely_not_started(GripperOutcome::kPositionNotVerified));
}

TEST(GripperOutcome, EveryOutcomeHasADistinctName)
{
  const GripperOutcome outcomes[]{
    GripperOutcome::kSucceeded, GripperOutcome::kRejected, GripperOutcome::kExecutionFailed,
    GripperOutcome::kPositionNotVerified, GripperOutcome::kCanceled, GripperOutcome::kTimedOut,
    GripperOutcome::kUnavailable};
  for (const auto outcome : outcomes) {
    const std::string name = gripper_outcome_name(outcome);
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, "unknown");
    for (const auto other : outcomes) {
      if (other != outcome) {
        EXPECT_NE(name, std::string(gripper_outcome_name(other)));
      }
    }
  }
}

TEST(GripperSubmitResult, OnlyAcceptedConvertsToTrue)
{
  EXPECT_TRUE(static_cast<bool>(GripperSubmitResult{GripperSubmitStatus::kAccepted, {}}));
  EXPECT_FALSE(
    static_cast<bool>(GripperSubmitResult{GripperSubmitStatus::kBusy, "outstanding"}));
  EXPECT_FALSE(
    static_cast<bool>(GripperSubmitResult{GripperSubmitStatus::kInvalidRequest, "bad"}));
  EXPECT_FALSE(
    static_cast<bool>(GripperSubmitResult{GripperSubmitStatus::kUnavailable, "down"}));
}

TEST(GripperSubmitResult, DefaultsToUnavailableSoAnUnsetResultNeverReadsAsAccepted)
{
  const GripperSubmitResult result;
  EXPECT_EQ(result.status, GripperSubmitStatus::kUnavailable);
  EXPECT_FALSE(static_cast<bool>(result));
}

TEST(GripperCompletion, DefaultsToUnavailableWithNoMeasurement)
{
  const GripperCompletion completion;
  EXPECT_EQ(completion.outcome, GripperOutcome::kUnavailable);
  // An absent sample means nothing was read; it is not a zero-position reading.
  EXPECT_FALSE(completion.measured.has_value());
}

TEST(FakeGripperPort, HoldsOneGoalAndCompletesItExactlyOnce)
{
  FakeGripperPort port;
  std::optional<GripperCompletion> observed;
  const OperationCorrelation correlation{GoalGeneration{3}, OperationGeneration{7}};

  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        correlation, usable_goal(),
        [&observed](GripperCompletion completion) {observed = std::move(completion);})));
  ASSERT_TRUE(port.outstanding());
  EXPECT_FALSE(observed.has_value());

  // A second goal while one is outstanding is refused, as the real port refuses it.
  const auto second = port.submit(
    correlation, usable_goal(), [](GripperCompletion) {});
  EXPECT_EQ(second.status, GripperSubmitStatus::kBusy);

  port.complete(GripperOutcome::kExecutionFailed, "controller aborted");
  ASSERT_TRUE(observed.has_value());
  EXPECT_EQ(observed->outcome, GripperOutcome::kExecutionFailed);
  EXPECT_EQ(observed->correlation.goal_generation, correlation.goal_generation);
  EXPECT_FALSE(observed->measured.has_value());
  EXPECT_FALSE(port.outstanding());
  EXPECT_EQ(port.submit_attempts, 2U);
}

TEST(FakeGripperPort, MeasurementDecidesBetweenSuccessAndAnUnverifiedPosition)
{
  FakeGripperPort port;
  const auto goal = usable_goal();

  std::optional<GripperCompletion> observed;
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        {}, goal,
        [&observed](GripperCompletion completion) {observed = std::move(completion);})));
  port.complete_with_measurement(
    GripperFingerState{goal.target_position_m, goal.target_position_m});
  ASSERT_TRUE(observed.has_value());
  EXPECT_EQ(observed->outcome, GripperOutcome::kSucceeded);
  ASSERT_TRUE(observed->measured.has_value());

  observed.reset();
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        {}, goal,
        [&observed](GripperCompletion completion) {observed = std::move(completion);})));
  // Inside gripper_controller's constraint, outside the attachment band.
  port.complete_with_measurement(
    GripperFingerState{goal.target_position_m + 0.002, goal.target_position_m});
  ASSERT_TRUE(observed.has_value());
  EXPECT_EQ(observed->outcome, GripperOutcome::kPositionNotVerified);
  ASSERT_TRUE(observed->measured.has_value());
  EXPECT_FALSE(gripper_definitely_not_started(observed->outcome));
}

TEST(FakeGripperPort, RefusalAndCancellationAreObservable)
{
  FakeGripperPort port;
  port.set_ready(false);
  EXPECT_FALSE(port.ready());

  port.refuse_next(GripperSubmitStatus::kUnavailable, "controller down");
  const auto refused = port.submit({}, usable_goal(), [](GripperCompletion) {});
  EXPECT_FALSE(static_cast<bool>(refused));
  EXPECT_EQ(refused.status, GripperSubmitStatus::kUnavailable);
  EXPECT_FALSE(port.outstanding());

  // The refusal is one-shot, so the following submission is taken normally.
  EXPECT_TRUE(
    static_cast<bool>(port.submit({}, usable_goal(), [](GripperCompletion) {})));
  port.cancel();
  EXPECT_EQ(port.cancel_requests, 1U);
  port.complete(GripperOutcome::kCanceled);
  EXPECT_FALSE(port.outstanding());
}

}  // namespace
}  // namespace restocker_task_executor
