// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

#include <moveit/utils/robot_model_test_utils.hpp>

#include "restocker_task_executor/trajectory_scene_validation.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr char kJoint[] = "base-tool-joint";

geometry_msgs::msg::Pose pose(double x = 0.0, double y = 0.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.orientation.w = 1.0;
  return result;
}

moveit_msgs::msg::CollisionObject box(const std::string & id, double x, double y)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "base";
  object.id = id;
  object.pose = pose(x, y);
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {0.05, 0.05, 0.05};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose());
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return object;
}

class CollisionScene : public ::testing::Test
{
protected:
  void SetUp() override
  {
    moveit::core::RobotModelBuilder builder("collision_guard", "base");
    builder.addChain("base->tool", "prismatic");
    builder.addGroupChain("base", "tool", "arm");
    builder.addCollisionBox("tool", {0.05, 0.05, 0.05}, pose());
    model = builder.build();
    ASSERT_TRUE(model);
    auto * joint = model->getJointModel(kJoint);
    auto bounds = joint->getVariableBounds(kJoint);
    bounds.position_bounded_ = true;
    bounds.min_position_ = -200.0;
    bounds.max_position_ = 200.0;
    joint->setVariableBounds(kJoint, bounds);
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    scene->getAllowedCollisionMatrixNonConst().setEntry("base", "tool", false);
    goal.joint_trajectory.joint_names = {kJoint};
    for (double x : {0.0, 1.0}) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      point.positions = {x};
      point.time_from_start.sec = static_cast<std::int32_t>(x);
      goal.joint_trajectory.points.push_back(point);
    }
  }

  moveit_msgs::msg::PlanningScene snapshot() const
  {
    moveit_msgs::msg::PlanningScene result;
    moveit_msgs::msg::PlanningSceneComponents components;
    components.components = kTrajectorySceneComponents;
    scene->getPlanningSceneMsg(result, components);
    return result;
  }

  std::string check() const
  {
    return validate_controller_goal_collision_scene(
      model, snapshot(), goal, "arm", [] {
        return false;
      });
  }

  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  moveit_msgs::msg::RobotTrajectory goal;
};

TEST_F(CollisionScene, ChangedOccupiedLaneRemainsUsableOnlyWhenEntireNextGoalIsClear)
{
  auto lane = box("restocker/workcell/occupied/lane01", 0.5, 0.4);
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  EXPECT_TRUE(check().empty()) << check();
  // A measured depth/centre change off the route must not exhaust progress retries.
  lane.primitives.front().dimensions[0] += 0.012;
  lane.pose.position.x += 0.006;
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  EXPECT_TRUE(check().empty()) << check();
  // Same identity moved into the middle of the route: both submitted endpoints are clear.
  lane.pose.position.y = 0.0;
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  EXPECT_NE(check().find("collides"), std::string::npos);
}

TEST_F(CollisionScene, FullScenePreservesPaddingAndAllowedCollisionMatrix)
{
  const auto obstacle = box("restocker/obstacle/edge", 0.5, 0.08);
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  ASSERT_TRUE(check().empty()) << check();
  scene->getCollisionEnvNonConst()->setLinkPadding("tool", 0.04);
  EXPECT_NE(check().find("collides"), std::string::npos);
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool", obstacle.id, true);
  EXPECT_TRUE(check().empty()) << check();
  scene->getAllowedCollisionMatrixNonConst().setEntry("tool", obstacle.id, false);
  EXPECT_NE(check().find("collides"), std::string::npos);
  scene->getCollisionEnvNonConst()->setLinkPadding("tool", 0.0);
  ASSERT_TRUE(check().empty()) << check();
  scene->getCollisionEnvNonConst()->setLinkScale("tool", 3.0);
  EXPECT_NE(check().find("collides"), std::string::npos);
}

TEST_F(CollisionScene, AttachedProductAndItsTouchLinksArePreserved)
{
  ASSERT_TRUE(scene->processCollisionObjectMsg(box("restocker/obstacle/carried", 0.5, 0.15)));
  ASSERT_TRUE(check().empty()) << check();
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tool";
  attached.touch_links = {"tool"};
  attached.object = box("restocker/object/held", 0.0, 0.15);
  attached.object.header.frame_id = "tool";
  ASSERT_TRUE(scene->processAttachedCollisionObjectMsg(attached));
  EXPECT_NE(check().find("restocker/object/held"), std::string::npos);
  auto away = box("restocker/obstacle/carried", 0.5, 0.5);
  ASSERT_TRUE(scene->processCollisionObjectMsg(away));
  EXPECT_TRUE(check().empty()) << check();
}

TEST_F(CollisionScene, MissingSceneConfigurationAndRobotStateRefuse)
{
  auto incomplete = snapshot();
  incomplete.robot_state.joint_state.position.clear();
  EXPECT_FALSE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).empty());
  incomplete = snapshot();
  incomplete.link_padding.clear();
  EXPECT_FALSE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).empty());
  incomplete = snapshot();
  incomplete.link_scale.clear();
  EXPECT_FALSE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).empty());
  incomplete = snapshot();
  incomplete.allowed_collision_matrix.entry_names.clear();
  EXPECT_FALSE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).empty());
}

TEST_F(CollisionScene, InvalidGoalAndFiniteWorkBudgetRefuse)
{
  goal.joint_trajectory.points.back().positions.front() = 150.0;
  EXPECT_NE(check().find("sample budget"), std::string::npos);
  goal.joint_trajectory.points.back().positions.front() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(check().find("non-finite"), std::string::npos);
  goal.joint_trajectory.points.back().positions.clear();
  EXPECT_NE(check().find("incomplete"), std::string::npos);
  goal.joint_trajectory.joint_names.push_back(kJoint);
  EXPECT_NE(check().find("complete bounded"), std::string::npos);
}

TEST_F(CollisionScene, PartialPaddingScaleAndInvalidValuesRefuse)
{
  auto incomplete = snapshot();
  incomplete.link_padding.front().link_name = "foreign";
  EXPECT_NE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).find("omits"), std::string::npos);
  incomplete = snapshot();
  incomplete.link_scale.front().link_name = "foreign";
  EXPECT_NE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).find("omits"), std::string::npos);
  incomplete = snapshot();
  incomplete.link_padding.front().padding = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(
    validate_controller_goal_collision_scene(
      model, incomplete, goal, "arm", [] {return false;}).find("invalid"), std::string::npos);
}

TEST_F(CollisionScene, CubicAndQuinticExcursionsOutsideTheWaypointChordAreChecked)
{
  ASSERT_TRUE(scene->processCollisionObjectMsg(box("restocker/obstacle/spline", 0.7, 0.0)));
  goal.joint_trajectory.points.back().positions.front() = 0.0;
  ASSERT_TRUE(check().empty());
  for (auto & point : goal.joint_trajectory.points) {
    point.velocities = {0.0};
  }
  goal.joint_trajectory.points.front().velocities = {8.0};
  EXPECT_NE(check().find("collides"), std::string::npos);
  for (auto & point : goal.joint_trajectory.points) {
    point.velocities = {0.0};
    point.accelerations = {100.0};
  }
  EXPECT_NE(check().find("collides"), std::string::npos);
}

TEST_F(CollisionScene, MeasuredStartAndItsBridgeToFirstCommandAreCollisionChecked)
{
  ASSERT_TRUE(scene->processCollisionObjectMsg(box("restocker/obstacle/start", -0.1, 0.0)));
  ASSERT_TRUE(check().empty());
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, -0.2);
  EXPECT_NE(check().find("collides"), std::string::npos);
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, -0.1);
  EXPECT_NE(check().find("sample 1"), std::string::npos);
}

TEST_F(CollisionScene, ExistingMeasuredStartAllowanceDoesNotClampCollisionGeometryOrCommandedSpline)
{
  auto * joint = model->getJointModel(kJoint);
  auto bounds = joint->getVariableBounds(kJoint);
  bounds.min_position_ = 0.0;
  bounds.max_position_ = 1.0;
  joint->setVariableBounds(kJoint, bounds);
  const auto admitted = [this] {
    return validate_controller_goal_collision_scene(
      model, snapshot(), goal, "arm", [] {return false;}, 0.001);
  };
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, -0.0005);
  EXPECT_NE(check().find("bounds"), std::string::npos);
  EXPECT_TRUE(admitted().empty()) << admitted();
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, -0.0015);
  EXPECT_NE(admitted().find("bounds"), std::string::npos);
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, 0.0);
  goal.joint_trajectory.points.front().positions.front() = -0.0005;
  EXPECT_NE(admitted().find("bounds"), std::string::npos);
  goal.joint_trajectory.points.front().positions.front() = 0.0;
  auto obstacle = box("restocker/obstacle/actual-start", -0.02535, 0.0);
  obstacle.primitives.front().dimensions[0] = 0.0002;
  ASSERT_TRUE(scene->processCollisionObjectMsg(obstacle));
  ASSERT_TRUE(admitted().empty()) << admitted();
  scene->getCurrentStateNonConst().setVariablePosition(kJoint, -0.0005);
  EXPECT_NE(admitted().find("collides"), std::string::npos);
}

TEST_F(CollisionScene, ImportedMeasuredVelocityIsNotMistakenForCommandedSplineDynamics)
{
  auto * joint = model->getJointModel(kJoint);
  auto bounds = joint->getVariableBounds(kJoint);
  bounds.velocity_bounded_ = true;
  bounds.min_velocity_ = -1.0;
  bounds.max_velocity_ = 1.0;
  joint->setVariableBounds(kJoint, bounds);
  scene->getCurrentStateNonConst().setVariableVelocity(kJoint, 2.0);
  EXPECT_TRUE(check().empty()) << check();
  ASSERT_TRUE(scene->processCollisionObjectMsg(box("restocker/obstacle/path", 0.5, 0.0)));
  EXPECT_NE(check().find("collides"), std::string::npos);
}

TEST_F(CollisionScene, CancellationOrDeadlineDuringInterpolationRefuses)
{
  unsigned checks = 0U;
  const auto result = validate_controller_goal_collision_scene(
    model, snapshot(), goal, "arm", [&checks] {return ++checks == 20U;});
  EXPECT_NE(result.find("canceled or its steady deadline expired"), std::string::npos);
  EXPECT_EQ(checks, 20U);
}

PreGraspSceneAuthority authority(std::uint64_t generation = 1U)
{
  PreGraspSceneAuthority value;
  value.planning_frame = "world";
  value.projector_epoch = "projector-original";
  value.applied_revision = 1U;
  value.verification_epoch = 1U;
  value.scene_content_generation = generation;
  return value;
}

PlanningSceneAuthorityResult certified(const PreGraspSceneAuthority & value)
{
  return {PreGraspAuthorityDecision::kAccepted, PreGraspAuthorityErrorCode::kNone, value, {}};
}

TEST_F(CollisionScene, InitialAndLaterSubmissionRequireFreshFullSceneProof)
{
  const auto original = authority();
  auto current = authority(2U);
  auto lane = box("restocker/lane/lane01", 0.5, 0.4);
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  unsigned submissions = 0U;
  unsigned validations = 0U;
  TrajectorySceneGateHooks hooks;
  hooks.read_authority = [&current] {return certified(current);};
  hooks.interrupted = [] {return false;};
  hooks.validate_next_goal = [this, &validations] {
    ++validations;
    return check();
  };
  const auto submit = [&] {
    const auto decision = validate_trajectory_scene_submission(original, hooks);
    if (decision.accepted) {
      ++submissions;
    }
    return decision;
  };
  EXPECT_TRUE(submit().revalidated);
  lane.primitives.front().dimensions[0] += 0.01;
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  ++current.scene_content_generation;
  EXPECT_TRUE(submit().revalidated);
  lane.pose.position.y = 0.0;
  ASSERT_TRUE(scene->processCollisionObjectMsg(lane));
  ++current.scene_content_generation;
  EXPECT_FALSE(submit().accepted);
  EXPECT_EQ(validations, 3U);
  EXPECT_EQ(submissions, 2U);
}

// The same seam used immediately before execute(): record a submission only after acceptance.
// Across calls, original remains the authority captured before planning, including later slices.
class SubmissionGate : public ::testing::Test
{
protected:
  SubmissionGate()
  {
    hooks.read_authority = [this] {
      ++reads;
      return certified(current);
    };
    hooks.validate_next_goal = [this] {
      ++validations;
      return collision_error;
    };
    hooks.interrupted = [this] {return interrupted;};
  }

  TrajectorySceneGateResult submit()
  {
    auto result = validate_trajectory_scene_submission(original, hooks);
    if (result.accepted) {
      ++submissions;
    }
    return result;
  }

  const PreGraspSceneAuthority original = authority();
  PreGraspSceneAuthority current = original;
  TrajectorySceneGateHooks hooks;
  std::string collision_error;
  bool interrupted = false;
  unsigned reads = 0U;
  unsigned validations = 0U;
  unsigned submissions = 0U;
};

TEST_F(SubmissionGate, NoDiffVerificationUsesOriginalProof)
{
  current.verification_epoch += 5U;
  current.applied_revision += 9U;
  EXPECT_TRUE(submit().accepted);
  EXPECT_EQ(validations, 0U);
  EXPECT_EQ(submissions, 1U);
}

TEST_F(SubmissionGate, ChangedContentNeedsIndependentProofForInitialAndEveryLaterSlice)
{
  current.scene_content_generation = 2U;
  EXPECT_TRUE(submit().revalidated);
  EXPECT_TRUE(submit().revalidated);
  EXPECT_EQ(validations, 2U);
  EXPECT_EQ(reads, 4U);
  EXPECT_EQ(submissions, 2U);
  EXPECT_EQ(original.scene_content_generation, 1U);
}

TEST_F(SubmissionGate, InitialOrLaterCollisionNeverSubmitsThatGoal)
{
  current.scene_content_generation = 2U;
  collision_error = "controller goal collides with restocker/obstacle/1";
  EXPECT_FALSE(submit().accepted);
  EXPECT_EQ(submissions, 0U);
  collision_error.clear();
  EXPECT_TRUE(submit().accepted);
  current.scene_content_generation = 3U;
  collision_error = "controller goal collides with restocker/obstacle/2";
  EXPECT_FALSE(submit().accepted);
  EXPECT_EQ(submissions, 1U);
}

TEST_F(SubmissionGate, UpdateDuringFetchOrValidationRequiresAnotherCompleteProof)
{
  current.scene_content_generation = 2U;
  hooks.validate_next_goal = [this] {
    if (++validations == 1U) {
      ++current.scene_content_generation;
      return std::string("superseded collision result");
    }
    return std::string{};
  };
  EXPECT_TRUE(submit().revalidated);
  EXPECT_EQ(validations, 2U);
  EXPECT_EQ(reads, 4U);
  EXPECT_EQ(submissions, 1U);
}

TEST_F(SubmissionGate, AuthorityCannotRegressBetweenRevalidationAttempts)
{
  hooks.read_authority = [this] {
    ++reads;
    return certified(authority(reads == 2U ? 3U : 2U));
  };
  EXPECT_FALSE(submit().accepted);
  EXPECT_EQ(reads, 3U);
  EXPECT_EQ(submissions, 0U);
}

TEST_F(SubmissionGate, ThreeUnstableProofsExhaustBudgetWithoutSubmission)
{
  current.scene_content_generation = 2U;
  hooks.validate_next_goal = [this] {
    ++validations;
    ++current.scene_content_generation;
    return std::string{};
  };
  EXPECT_FALSE(submit().accepted);
  EXPECT_EQ(validations, 3U);
  EXPECT_EQ(reads, 6U);
  EXPECT_EQ(submissions, 0U);
}

TEST_F(SubmissionGate, LineageChangesNeverUseCollisionValidationAsAnExemption)
{
  for (unsigned variant = 0U; variant < 6U; ++variant) {
    current = authority(2U);
    switch (variant) {
      case 0U: current.projector_epoch = "replacement"; break;
      case 1U: current.lease_id = 1U; break;
      case 2U: current.planning_frame = "replacement"; break;
      case 3U: current.applied_revision = 0U; break;
      case 4U: current.verification_epoch = 0U; break;
      case 5U: current.scene_content_generation = 0U; break;
    }
    EXPECT_FALSE(submit().accepted) << variant;
  }
  EXPECT_EQ(validations, 0U);
  EXPECT_EQ(submissions, 0U);
}

TEST_F(SubmissionGate, RestartOrLeaseRevocationDuringValidationRefuses)
{
  for (unsigned variant = 0U; variant < 2U; ++variant) {
    current = authority(2U);
    hooks.validate_next_goal = [this, variant] {
      if (variant == 0U) {
        current.projector_epoch = "replacement";
      } else {
        current.lease_id = 1U;
      }
      return std::string{};
    };
    EXPECT_FALSE(submit().accepted);
  }
  EXPECT_EQ(submissions, 0U);
}

TEST_F(SubmissionGate, UncertifiedReadsBeforeOrAfterProofRefuse)
{
  for (const auto decision : {PreGraspAuthorityDecision::kWait,
      PreGraspAuthorityDecision::kRejected})
  {
    for (unsigned fail_read : {1U, 2U}) {
      current = authority(2U);
      reads = 0U;
      hooks.read_authority = [this, decision, fail_read] {
        if (++reads == fail_read) {
          return PlanningSceneAuthorityResult{
          decision, PreGraspAuthorityErrorCode::kStaleEvidence, std::nullopt, "not certified"};
        }
        return certified(current);
      };
      EXPECT_FALSE(submit().accepted);
    }
  }
  EXPECT_EQ(submissions, 0U);
}

TEST_F(SubmissionGate, CancellationOrDeadlineAtEveryStagePreventsSubmission)
{
  for (unsigned stage = 0U; stage < 4U; ++stage) {
    current = authority(2U);
    interrupted = stage == 0U;
    reads = 0U;
    hooks.read_authority = [this, stage] {
      ++reads;
      interrupted = interrupted || (stage == 1U && reads == 1U) || (stage == 3U && reads == 2U);
      return certified(current);
    };
    hooks.validate_next_goal = [this, stage] {
      interrupted = stage == 2U;
      return std::string{};
    };
    EXPECT_FALSE(submit().accepted) << stage;
  }
  EXPECT_EQ(submissions, 0U);
}

}  // namespace
}  // namespace restocker_task_executor
