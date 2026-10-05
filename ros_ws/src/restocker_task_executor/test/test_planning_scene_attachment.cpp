// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/planning_scene_attachment.hpp"

namespace restocker_task_executor
{
namespace
{

geometry_msgs::msg::Pose pose(double x = 0.0, double y = 0.0, double z = 0.0)
{
  geometry_msgs::msg::Pose result;
  result.position.x = x;
  result.position.y = y;
  result.position.z = z;
  result.orientation.w = 1.0;
  return result;
}

moveit_msgs::msg::CollisionObject can_world_object()
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = "restocker/object/7";
  object.operation = object.ADD;
  object.pose = pose(0.1, 0.2, 0.3);
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions = {0.122, 0.033};
  object.primitives.push_back(primitive);
  object.primitive_poses.push_back(pose());
  return object;
}

const std::vector<std::string> kTouchLinks{"right_finger", "gripper", "left_finger"};

TEST(PlanningSceneAttachmentTest, BuildsAtomicWorldToAttachedDiffFromObservedTransform)
{
  const auto observed_transform = pose(0.0, 0.0, 0.14);
  const auto result = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, observed_transform);
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_TRUE(result.value().diff.is_diff);
  ASSERT_TRUE(result.value().diff.robot_state.is_diff);
  ASSERT_EQ(result.value().diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(
    result.value().diff.world.collision_objects.front().operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  ASSERT_EQ(result.value().diff.robot_state.attached_collision_objects.size(), 1U);
  const auto & attached = result.value().expected_attached_object;
  EXPECT_EQ(attached.link_name, "gripper");
  EXPECT_EQ(
    attached.touch_links,
    (std::vector<std::string>{"gripper", "left_finger", "right_finger"}));
  EXPECT_EQ(attached.object.header.frame_id, "gripper");
  EXPECT_EQ(attached.object.pose, observed_transform);
  EXPECT_EQ(attached.object.primitives, can_world_object().primitives);
}

TEST(PlanningSceneAttachmentTest, RejectsInvalidGeometryTransformAndTouchLinks)
{
  auto object = can_world_object();
  object.id = "foreign/object/7";
  EXPECT_FALSE(build_world_to_attached_diff(object, "gripper", kTouchLinks, pose()));
  object.id = "restocker/object/";
  EXPECT_FALSE(build_world_to_attached_diff(object, "gripper", kTouchLinks, pose()));
  object.id = "restocker/object/seven";
  EXPECT_FALSE(build_world_to_attached_diff(object, "gripper", kTouchLinks, pose()));
  object = can_world_object();
  auto invalid_pose = pose();
  invalid_pose.orientation.w = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(build_world_to_attached_diff(object, "gripper", kTouchLinks, invalid_pose));
  EXPECT_FALSE(
    build_world_to_attached_diff(object, "gripper", {"gripper", "gripper"}, pose()));
  EXPECT_FALSE(
    build_world_to_attached_diff(
      object, "gripper", {"left_finger", "right_finger"}, pose()));
}

TEST(PlanningSceneAttachmentTest, VerifiesExactAttachedRepresentationAndOppositeAbsence)
{
  const auto projection = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, pose(0.0, 0.0, 0.14));
  ASSERT_TRUE(projection);
  moveit_msgs::msg::PlanningScene observed;
  observed.robot_state.attached_collision_objects.push_back(
    projection.value().expected_attached_object);
  EXPECT_TRUE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));

  observed.world.collision_objects.push_back(can_world_object());
  EXPECT_FALSE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));
  observed.world.collision_objects.clear();
  observed.robot_state.attached_collision_objects.front().touch_links.pop_back();
  EXPECT_FALSE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));
  observed.robot_state.attached_collision_objects.front() =
    projection.value().expected_attached_object;
  observed.robot_state.attached_collision_objects.front()
  .object.primitives.front().dimensions.front() += 1.0e-5;
  EXPECT_FALSE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));
  observed.robot_state.attached_collision_objects.front() =
    projection.value().expected_attached_object;
  observed.robot_state.attached_collision_objects.push_back(
    projection.value().expected_attached_object);
  EXPECT_FALSE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));
}

TEST(PlanningSceneAttachmentTest, BuildsAndVerifiesAtomicAttachedToWorldDiff)
{
  const auto attachment = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, pose(0.0, 0.0, 0.14));
  ASSERT_TRUE(attachment);
  const auto settled_pose = pose(0.5, 0.6, 0.7);
  const auto detachment = build_attached_to_world_diff(
    attachment.value().expected_attached_object, "world", settled_pose);
  ASSERT_TRUE(detachment) << detachment.error().detail;
  ASSERT_EQ(detachment.value().diff.robot_state.attached_collision_objects.size(), 1U);
  EXPECT_EQ(
    detachment.value().diff.robot_state.attached_collision_objects.front().object.operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  ASSERT_EQ(detachment.value().diff.world.collision_objects.size(), 1U);
  EXPECT_EQ(detachment.value().expected_world_object.header.frame_id, "world");
  EXPECT_EQ(detachment.value().expected_world_object.pose, settled_pose);

  moveit_msgs::msg::PlanningScene observed;
  observed.world.collision_objects.push_back(detachment.value().expected_world_object);
  EXPECT_TRUE(
    verify_attached_to_world_transition(detachment.value().expected_world_object, observed));
  observed.robot_state.attached_collision_objects.push_back(
    attachment.value().expected_attached_object);
  EXPECT_FALSE(
    verify_attached_to_world_transition(detachment.value().expected_world_object, observed));
}

TEST(PlanningSceneAttachmentTest, RejectsMalformedAttachedSourceAndVerificationConfig)
{
  moveit_msgs::msg::AttachedCollisionObject malformed;
  malformed.link_name = "gripper";
  malformed.touch_links = kTouchLinks;
  malformed.object = can_world_object();
  EXPECT_FALSE(build_attached_to_world_diff(malformed, "world", pose()));

  const auto projection = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, pose());
  ASSERT_TRUE(projection);
  moveit_msgs::msg::PlanningScene observed;
  observed.robot_state.attached_collision_objects.push_back(
    projection.value().expected_attached_object);
  SceneVerificationConfig config;
  config.position_tolerance = -1.0;
  const auto result = verify_world_to_attached_transition(
    projection.value().expected_attached_object, observed, config);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, ProjectionErrorCode::InvalidConfiguration);

  auto malformed_expected = projection.value().expected_attached_object;
  malformed_expected.touch_links.push_back("gripper");
  const auto malformed_result = verify_world_to_attached_transition(malformed_expected, observed);
  ASSERT_FALSE(malformed_result);
  EXPECT_EQ(malformed_result.error().code, ProjectionErrorCode::InvalidScene);
}

TEST(PlanningSceneAttachmentTest, AcceptsTheKinematicPoseMoveItDerivesForAnAttachedProduct)
{
  // MoveIt carries an attached body off the link it hangs from, so once the transition has run
  // the product's pose is a consequence of that link and the grasp that put it there. Reading it
  // back and comparing it against the pose this same diff stated proves nothing about the world,
  // and MoveIt is free to hand back its own recomposition of that transform. What the transition
  // does have to prove is that the right product is on the right link with the right shape and
  // the right collision exemptions, and those are still checked.
  const auto projection = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, pose(0.0, 0.0, 0.14));
  ASSERT_TRUE(projection);
  moveit_msgs::msg::PlanningScene observed;
  observed.robot_state.attached_collision_objects.push_back(
    projection.value().expected_attached_object);
  observed.robot_state.attached_collision_objects.front().object.pose.position.x += 1.0e-4;
  EXPECT_TRUE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));

  observed.robot_state.attached_collision_objects.front().object.id = "restocker/object/8";
  EXPECT_FALSE(
    verify_world_to_attached_transition(projection.value().expected_attached_object, observed));
}

TEST(PlanningSceneAttachmentTest, MeasuresOrientationWithinTheToleranceItIsConfiguredWith)
{
  // A 20-degree yaw, written out so the test does not depend on the platform's sine and cosine.
  // Its components square-and-sum to one unit in the last place below 1.0, and `2 * acos` of that
  // is 3e-8 rad, three times the 1e-8 rad tolerance the projector runs with. Measured that way,
  // this pose would fail verification against a bit-identical copy of itself, and about a fifth
  // of yaw angles land the same way. The angle is therefore taken on the relative rotation, which
  // stays exact near identity.
  auto settled = pose(0.5, 0.6, 0.7);
  settled.orientation.w = 0.984807753012208;
  settled.orientation.z = 0.17364817766693033;
  const double self_dot = settled.orientation.w * settled.orientation.w +
    settled.orientation.z * settled.orientation.z;
  ASSERT_LT(self_dot, 1.0);
  ASSERT_GT(2.0 * std::acos(self_dot), 1.0e-8);

  const auto attachment = build_world_to_attached_diff(
    can_world_object(), "gripper", kTouchLinks, pose(0.0, 0.0, 0.14));
  ASSERT_TRUE(attachment);
  const auto detachment = build_attached_to_world_diff(
    attachment.value().expected_attached_object, "world", settled);
  ASSERT_TRUE(detachment) << detachment.error().detail;
  moveit_msgs::msg::PlanningScene observed;
  observed.world.collision_objects.push_back(detachment.value().expected_world_object);
  const auto result =
    verify_attached_to_world_transition(detachment.value().expected_world_object, observed);
  EXPECT_TRUE(result) << result.error().detail;

  observed.world.collision_objects.front().pose.position.x += 1.0e-5;
  const auto moved =
    verify_attached_to_world_transition(detachment.value().expected_world_object, observed);
  ASSERT_FALSE(moved);
  EXPECT_EQ(moved.error().code, ProjectionErrorCode::VerificationMismatch);
}

}  // namespace
}  // namespace restocker_task_executor
