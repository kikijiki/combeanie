// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include <restocker_perception/fake_viewpoint_port.hpp>

#include "fake_motion_port.hpp"
#include "restocker_task_executor/viewpoint_survey.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_perception::CameraViewpoint;
using restocker_perception::FramedTransform;
using restocker_perception::FakeViewpointPort;
using restocker_perception::Result;
using restocker_perception::ViewpointOutcome;
using restocker_perception::ViewpointSubmitStatus;
using restocker_perception::WristCameraMount;
using restocker_perception::kToolFrame;
using restocker_perception::kWristCameraOpticalFrame;

// The mount's own numbers do not matter to these tests, a fake accepts any transform, which is
// exactly why unit tests cannot prove the aiming arithmetic. What they can prove, and what this
// file is for, is the wiring: one completion per accepted submission, refusals surfaced rather
// than swallowed, and a tool goal that is the resolution's and not the viewpoint's.
[[nodiscard]] WristCameraMount some_mount()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.11, 0.0, 0.096);
  Result<FramedTransform> framed =
    FramedTransform::create(kWristCameraOpticalFrame, kToolFrame, transform);
  EXPECT_TRUE(framed.has_value());
  Result<WristCameraMount> mount = WristCameraMount::create(framed.value());
  EXPECT_TRUE(mount.has_value());
  return mount.value();
}

[[nodiscard]] CameraViewpoint lane_viewpoint()
{
  CameraViewpoint viewpoint;
  viewpoint.pose.frame_id = "world";
  viewpoint.pose.pose.translation() = Eigen::Vector3d(-1.0, 0.0, 1.6);
  viewpoint.label = "lane_01";
  return viewpoint;
}

class ViewpointSurveyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    viewpoints.set_mount(some_mount());
  }

  [[nodiscard]] SurveySubmitResult submit(CameraViewpoint viewpoint = lane_viewpoint())
  {
    return survey.submit(
      OperationCorrelation{3U, 9U}, std::move(viewpoint),
      [this](SurveyCompletion result) {
        ++deliveries;
        completion = std::move(result);
      });
  }

  FakeViewpointPort viewpoints;
  FakeMotionPort motion;
  ViewpointSurvey survey{viewpoints, motion};
  std::optional<SurveyCompletion> completion;
  std::size_t deliveries{0U};
};

TEST_F(ViewpointSurveyTest, PlansAFreeSpaceSegmentToTheResolvedToolPose)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  EXPECT_EQ(deliveries, 0U);
  ASSERT_TRUE(viewpoints.outstanding());
  EXPECT_FALSE(motion.outstanding());

  viewpoints.complete_with_mount();
  EXPECT_EQ(deliveries, 0U);
  ASSERT_TRUE(motion.outstanding());
  const MotionGoal & goal = motion.submission().goal;
  // A survey is free-space repositioning. Anything else would put it outside the segment kind the
  // planning-scene authority gate is written for.
  EXPECT_EQ(goal.path, MotionPathKind::kFreeSpace);
  EXPECT_EQ(goal.label, "survey lane_01");
  EXPECT_EQ(motion.submission().correlation.operation_generation, 9U);
  // The motion goal is the *tool* pose, not the camera pose. They differ by the mount offset, and
  // a survey that sent the camera pose straight to MoveIt would put the lens 0.146 m from where
  // it was asked to be.
  EXPECT_NEAR(
    (goal.planning_frame_from_tool0.translation() -
    lane_viewpoint().pose.pose.translation()).norm(), 0.146, 1.0e-3);

  motion.complete(MotionOutcome::kSucceeded, "arrived", true);
  EXPECT_EQ(deliveries, 1U);
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kArrived);
  EXPECT_EQ(completion->correlation.goal_generation, 3U);
  EXPECT_TRUE(completion->viewpoint_resolved);
  EXPECT_TRUE(completion->execution_reached_terminal_stop);
  // The completion echoes the camera pose that was asked for, so a failure report names the
  // viewpoint rather than the tool pose derived from it.
  EXPECT_EQ(completion->commanded_viewpoint.label, "lane_01");
  EXPECT_LT(
    (completion->commanded_viewpoint.pose.pose.translation() -
    lane_viewpoint().pose.pose.translation()).norm(), 1.0e-12);
  EXPECT_LT(
    (completion->commanded_goal.pose.pose.translation() -
    goal.planning_frame_from_tool0.translation()).norm(), 1.0e-12);
}

TEST_F(ViewpointSurveyTest, PassesAMotionFailureThroughWithoutReclassifyingIt)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  viewpoints.complete_with_mount();
  ASSERT_TRUE(motion.outstanding());
  motion.complete(MotionOutcome::kPlanningFailed, "no collision-free plan");
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kPlanningFailed);
  EXPECT_EQ(completion->detail, "no collision-free plan");
  // Planning failed means nothing was commanded, and a survey must not lose that: retrying a
  // station that never moved the arm is free, retrying one that did is not.
  EXPECT_TRUE(survey_definitely_not_started(completion->outcome));
  EXPECT_FALSE(survey_definitely_not_started(SurveyOutcome::kExecutionFailed));
}

// Review B1: the motion port reports kUnavailable for an execute-phase backend loss too; once a
// trajectory was handed to the backend the arm may be moving, so the survey must not read it as
// "never started" (it feeds the lane/tray evidence flags).
TEST_F(ViewpointSurveyTest, AnExecutePhaseBackendLossIsAnExecutionFailureNotANonStart)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  viewpoints.complete_with_mount();
  ASSERT_TRUE(motion.outstanding());
  motion.complete_after_execute(MotionOutcome::kUnavailable, "lost MoveIt after execute()");
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kExecutionFailed);
  EXPECT_FALSE(survey_definitely_not_started(completion->outcome));
  EXPECT_FALSE(completion->execution_reached_terminal_stop);
}

TEST_F(ViewpointSurveyTest, AnUnavailableBackendBeforeAnyExecuteStaysANonStart)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  viewpoints.complete_with_mount();
  ASSERT_TRUE(motion.outstanding());
  motion.complete(MotionOutcome::kUnavailable, "MoveIt is not running");
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kUnavailable);
  EXPECT_TRUE(survey_definitely_not_started(completion->outcome));
}

TEST_F(ViewpointSurveyTest, ABusyMotionPortIsNotANonStart)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  motion.refuse_next(MotionSubmitStatus::kBusy, "the port still owns a goal");
  viewpoints.complete_with_mount();
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kExecutionFailed);
  EXPECT_FALSE(survey_definitely_not_started(completion->outcome));
}

TEST_F(ViewpointSurveyTest, NeverCommandsAMotionWhenTheMountIsUnknown)
{
  viewpoints.clear_mount();
  ASSERT_TRUE(static_cast<bool>(submit()));
  viewpoints.complete_with_mount();
  EXPECT_FALSE(motion.outstanding());
  EXPECT_EQ(motion.submit_attempts, 0U);
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kViewpointUnresolved);
  EXPECT_FALSE(completion->viewpoint_resolved);
  EXPECT_TRUE(survey_definitely_not_started(completion->outcome));
  EXPECT_STREQ(survey_outcome_name(completion->outcome), "viewpoint unresolved");
}

TEST_F(ViewpointSurveyTest, RefusesASecondSurveyWhileOneIsOutstanding)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  const SurveySubmitResult second = submit();
  EXPECT_FALSE(static_cast<bool>(second));
  EXPECT_EQ(second.status, SurveySubmitStatus::kBusy);
  // The refused submission must not have consumed the outstanding one.
  viewpoints.complete_with_mount();
  motion.complete(MotionOutcome::kSucceeded);
  EXPECT_EQ(deliveries, 1U);
}

TEST_F(ViewpointSurveyTest, ReportsAViewpointPortRefusalSynchronouslyAndFreesTheSlot)
{
  viewpoints.refuse_next(ViewpointSubmitStatus::kUnavailable, "port shutting down");
  const SurveySubmitResult refused = submit();
  EXPECT_FALSE(static_cast<bool>(refused));
  EXPECT_EQ(refused.status, SurveySubmitStatus::kUnavailable);
  EXPECT_EQ(refused.detail, "port shutting down");
  // No completion was promised, so none must arrive; and the slot has to be free again, or a
  // refusal would wedge the survey for the rest of the run.
  EXPECT_EQ(deliveries, 0U);
  EXPECT_TRUE(static_cast<bool>(submit()));
}

TEST_F(ViewpointSurveyTest, ReportsAMotionPortRefusalAsASingleCompletion)
{
  // The port cannot take work at all (stopping): nothing was submitted. A kBusy refusal is a
  // different answer (an earlier goal may be moving) and has its own test below.
  motion.refuse_next(MotionSubmitStatus::kUnavailable, "motion port is stopping");
  ASSERT_TRUE(static_cast<bool>(submit()));
  viewpoints.complete_with_mount();
  EXPECT_EQ(deliveries, 1U);
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kUnavailable);
  EXPECT_NE(completion->detail.find("refused the survey segment"), std::string::npos);
  // The viewpoint did resolve, so the completion still carries the tool goal that was refused.
  EXPECT_TRUE(completion->viewpoint_resolved);
}

TEST_F(ViewpointSurveyTest, StopsBeforeCommandingWhenCancelledDuringResolution)
{
  ASSERT_TRUE(static_cast<bool>(submit()));
  survey.cancel();
  viewpoints.complete_with_mount();
  EXPECT_FALSE(motion.outstanding());
  ASSERT_TRUE(completion.has_value());
  EXPECT_EQ(completion->outcome, SurveyOutcome::kCanceled);
  EXPECT_EQ(viewpoints.cancel_requests, 1U);
  EXPECT_EQ(motion.cancel_requests, 1U);
}

TEST_F(ViewpointSurveyTest, RefusesAViewpointThatNamesNoFrame)
{
  CameraViewpoint viewpoint = lane_viewpoint();
  viewpoint.pose.frame_id.clear();
  const SurveySubmitResult refused = submit(viewpoint);
  EXPECT_EQ(refused.status, SurveySubmitStatus::kInvalidRequest);
  EXPECT_EQ(viewpoints.submit_attempts, 0U);
}

TEST_F(ViewpointSurveyTest, IsReadyOnlyWhenBothPortsAre)
{
  EXPECT_TRUE(survey.ready());
  motion.set_ready(false);
  EXPECT_FALSE(survey.ready());
  motion.set_ready(true);
  viewpoints.clear_mount();
  EXPECT_FALSE(survey.ready());
}

TEST_F(ViewpointSurveyTest, AppliesPerSurveyToleranceOverridesAndKeepsTheRest)
{
  ViewpointSurveyConfig overrides;
  overrides.position_tolerance_m = 0.05;
  overrides.orientation_tolerance_rad = 0.0;     // zero means "keep the configured default"
  overrides.velocity_scaling = 0.0;
  overrides.acceleration_scaling = 0.0;
  overrides.planning_time = std::chrono::milliseconds(0);
  ASSERT_TRUE(
    static_cast<bool>(
      survey.submit(
        OperationCorrelation{1U, 1U}, lane_viewpoint(), overrides,
        [this](SurveyCompletion result) {
          ++deliveries;
          completion = std::move(result);
        })));
  viewpoints.complete_with_mount();
  ASSERT_TRUE(motion.outstanding());
  EXPECT_NEAR(motion.submission().goal.position_tolerance_m, 0.05, 1.0e-12);
  EXPECT_NEAR(
    motion.submission().goal.orientation_tolerance_rad,
    ViewpointSurveyConfig{}.orientation_tolerance_rad, 1.0e-12);
  EXPECT_NEAR(
    motion.submission().goal.velocity_scaling, ViewpointSurveyConfig{}.velocity_scaling, 1.0e-12);
}

}  // namespace
}  // namespace restocker_task_executor
