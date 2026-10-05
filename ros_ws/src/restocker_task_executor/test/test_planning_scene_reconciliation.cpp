// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/planning_scene_reconciliation.hpp"

namespace restocker_task_executor
{
namespace
{

moveit_msgs::msg::CollisionObject cylinder(const std::string & id, double x = 0.0)
{
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.position.x = x;
  object.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions = {0.12, 0.03};
  object.primitives.push_back(std::move(primitive));
  geometry_msgs::msg::Pose primitive_pose;
  primitive_pose.orientation.w = 1.0;
  object.primitive_poses.push_back(primitive_pose);
  return object;
}

moveit_msgs::msg::PlanningScene scene_with(
  std::initializer_list<moveit_msgs::msg::CollisionObject> objects)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.world.collision_objects.assign(objects);
  return scene;
}

TEST(PlanningSceneReconciliation, BuildsOneSortedDiffAndRemovesOnlyManagedStaleObjects)
{
  const auto desired_product = cylinder("restocker/object/2", 0.2);
  const auto desired_workcell = cylinder("restocker/workcell/shelf/deck", 0.0);
  auto current = scene_with(
    {cylinder("foreign/benchmark"), cylinder("restocker/object/99")});
  auto result = build_planning_scene_diff({desired_workcell, desired_product}, current);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(result.value().is_diff);
  EXPECT_TRUE(result.value().robot_state.is_diff);
  ASSERT_EQ(result.value().world.collision_objects.size(), 3U);
  EXPECT_EQ(result.value().world.collision_objects[0].id, "restocker/object/2");
  EXPECT_EQ(
    result.value().world.collision_objects[0].operation,
    moveit_msgs::msg::CollisionObject::ADD);
  EXPECT_EQ(result.value().world.collision_objects[1].id, "restocker/object/99");
  EXPECT_EQ(
    result.value().world.collision_objects[1].operation,
    moveit_msgs::msg::CollisionObject::REMOVE);
  EXPECT_EQ(
    result.value().world.collision_objects[2].id,
    "restocker/workcell/shelf/deck");
  EXPECT_TRUE(
    std::ranges::none_of(
      result.value().world.collision_objects,
      [](const auto & object) {return object.id == "foreign/benchmark";}));
}

// Card 050: a declared-product seed is managed geometry — added while desired, removed the cycle
// desired drops it (world state started tracking the product), proven by verification.
TEST(PlanningSceneReconciliation, AddsAndRemovesDeclaredSeedsLikeAnyManagedObject)
{
  const auto seed = cylinder("restocker/declared/sim:stock_can_02", 0.1);
  auto added = build_planning_scene_diff({seed}, scene_with({}));
  ASSERT_TRUE(added) << added.error().detail;
  ASSERT_EQ(added.value().world.collision_objects.size(), 1U);
  EXPECT_EQ(added.value().world.collision_objects[0].id, seed.id);
  EXPECT_EQ(
    added.value().world.collision_objects[0].operation,
    moveit_msgs::msg::CollisionObject::ADD);

  auto removed = build_planning_scene_diff({}, scene_with({seed}));
  ASSERT_TRUE(removed) << removed.error().detail;
  ASSERT_EQ(removed.value().world.collision_objects.size(), 1U);
  EXPECT_EQ(removed.value().world.collision_objects[0].id, seed.id);
  EXPECT_EQ(
    removed.value().world.collision_objects[0].operation,
    moveit_msgs::msg::CollisionObject::REMOVE);

  EXPECT_TRUE(verify_planning_scene({seed}, {}, scene_with({seed})));
}

TEST(PlanningSceneReconciliation, RejectsMalformedForeignAndDuplicateDesiredGeometry)
{
  const moveit_msgs::msg::PlanningScene empty_scene;
  auto malformed = cylinder("restocker/object/1");
  malformed.pose.orientation.w = 0.0;
  auto malformed_result = build_planning_scene_diff({malformed}, empty_scene);
  ASSERT_FALSE(malformed_result);
  EXPECT_EQ(malformed_result.error().code, ProjectionErrorCode::InvalidScene);
  EXPECT_NE(malformed_result.error().detail.find(malformed.id), std::string::npos);

  auto missing_primitive_pose = cylinder("restocker/object/2");
  missing_primitive_pose.primitive_poses.clear();
  auto missing_pose_result = build_planning_scene_diff({missing_primitive_pose}, empty_scene);
  ASSERT_FALSE(missing_pose_result);
  EXPECT_NE(
    missing_pose_result.error().detail.find("primitive"), std::string::npos);

  auto foreign_result = build_planning_scene_diff(
    {cylinder("foreign/object")}, empty_scene);
  ASSERT_FALSE(foreign_result);
  EXPECT_EQ(foreign_result.error().code, ProjectionErrorCode::InvalidScene);

  const auto duplicate = cylinder("restocker/object/1");
  auto duplicate_result = build_planning_scene_diff(
    {duplicate, duplicate}, empty_scene);
  ASSERT_FALSE(duplicate_result);
  EXPECT_EQ(duplicate_result.error().code, ProjectionErrorCode::InvalidScene);
  EXPECT_NE(duplicate_result.error().detail.find(duplicate.id), std::string::npos);
}

TEST(PlanningSceneVerification, AcceptsExactManagedGeometryAndIgnoresForeignWorldObjects)
{
  const std::vector desired{
    cylinder("restocker/object/1", 0.2),
    cylinder("restocker/workcell/shelf/deck", 0.4),
  };
  auto observed = scene_with(
    {desired[0], cylinder("foreign/obstacle"), desired[1]});
  auto result = verify_planning_scene(desired, {}, observed);
  EXPECT_TRUE(result) << result.error().detail;
}

TEST(PlanningSceneVerification, DetectsMissingAndUnexpectedManagedObjects)
{
  const std::vector desired{cylinder("restocker/object/1")};
  auto missing = verify_planning_scene(desired, {}, scene_with({}));
  ASSERT_FALSE(missing);
  EXPECT_EQ(missing.error().code, ProjectionErrorCode::VerificationMismatch);
  EXPECT_NE(missing.error().detail.find("restocker/object/1"), std::string::npos);
  EXPECT_NE(missing.error().detail.find("observed {missing}"), std::string::npos);

  auto extra = verify_planning_scene(
    desired, {}, scene_with({desired[0], cylinder("restocker/object/2")}));
  ASSERT_FALSE(extra);
  EXPECT_EQ(extra.error().code, ProjectionErrorCode::VerificationMismatch);
  EXPECT_NE(extra.error().detail.find("restocker/object/2"), std::string::npos);
}

TEST(PlanningSceneVerification, DetectsFrameDimensionAndPoseDrift)
{
  const auto expected = cylinder("restocker/object/1", 0.2);
  auto wrong_frame = expected;
  wrong_frame.header.frame_id = "map";
  EXPECT_FALSE(verify_planning_scene({expected}, {}, scene_with({wrong_frame})));

  auto wrong_dimension = expected;
  wrong_dimension.primitives[0].dimensions[1] += 1.0e-6;
  const auto dimension_diff = verify_planning_scene({expected}, {}, scene_with({wrong_dimension}));
  ASSERT_FALSE(dimension_diff);
  EXPECT_NE(dimension_diff.error().detail.find("desired {frame="), std::string::npos);
  EXPECT_NE(dimension_diff.error().detail.find("observed {frame="), std::string::npos);
  EXPECT_NE(dimension_diff.error().detail.find("dimensions=["), std::string::npos);

  auto wrong_pose = expected;
  wrong_pose.pose.position.x += 1.0e-6;
  EXPECT_FALSE(verify_planning_scene({expected}, {}, scene_with({wrong_pose})));

  auto wrong_primitive_pose = expected;
  wrong_primitive_pose.primitive_poses[0].position.x += 1.0e-6;
  EXPECT_FALSE(
    verify_planning_scene({expected}, {}, scene_with({wrong_primitive_pose})));
}

TEST(PlanningSceneVerification, RequiresExactManagedAttachmentSet)
{
  moveit_msgs::msg::PlanningScene observed;
  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.object.id = "restocker/object/7";
  observed.robot_state.attached_collision_objects.push_back(attached);
  EXPECT_TRUE(verify_planning_scene({}, {"restocker/object/7"}, observed));
  EXPECT_FALSE(verify_planning_scene({}, {}, observed));
  EXPECT_FALSE(verify_planning_scene({}, {"restocker/object/8"}, observed));
}

TEST(PlanningSceneVerification, RejectsInvalidToleranceConfiguration)
{
  SceneVerificationConfig config;
  config.position_tolerance = std::numeric_limits<double>::quiet_NaN();
  const moveit_msgs::msg::PlanningScene empty_scene;
  auto result = verify_planning_scene({}, {}, empty_scene, config);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, ProjectionErrorCode::InvalidConfiguration);
  EXPECT_EQ(to_string(ProjectionErrorCode::VerificationMismatch), "verification_mismatch");
}

SceneVerificationConfig change_deadband()
{
  SceneVerificationConfig config;
  config.position_tolerance = 5.0e-4;
  config.orientation_tolerance_rad = 1.0e-3;
  config.dimension_tolerance = 1.0e-8;
  return config;
}

TEST(PlanningSceneRetention, KeepsTheSceneUnchangedForSubDeadbandProductMotion)
{
  const auto in_scene = cylinder("restocker/object/1", 0.2);
  auto crept = in_scene;
  crept.pose.position.x += 9.0e-6;
  crept.primitive_poses[0].position.z += 9.0e-6;

  // Without the deadband this is a mismatch, and every such cycle writes a diff.
  EXPECT_FALSE(verify_planning_scene({crept}, {}, scene_with({in_scene})));

  std::vector<moveit_msgs::msg::CollisionObject> desired{crept};
  retain_unmoved_product_geometry(desired, scene_with({in_scene}), change_deadband());
  EXPECT_TRUE(verify_planning_scene(desired, {}, scene_with({in_scene})));
}

TEST(PlanningSceneRetention, AdoptsAProductThatActuallyMoved)
{
  const auto in_scene = cylinder("restocker/object/1", 0.2);
  auto moved = in_scene;
  moved.pose.position.x += 0.01;

  std::vector<moveit_msgs::msg::CollisionObject> desired{moved};
  retain_unmoved_product_geometry(desired, scene_with({in_scene}), change_deadband());
  EXPECT_DOUBLE_EQ(desired.front().pose.position.x, moved.pose.position.x);
  EXPECT_FALSE(verify_planning_scene(desired, {}, scene_with({in_scene})));
}

TEST(PlanningSceneRetention, LeavesDimensionsIdentityAndNonProductGeometryAlone)
{
  const auto in_scene = cylinder("restocker/object/1", 0.2);
  auto resized = in_scene;
  resized.primitives[0].dimensions[0] += 1.0e-6;
  resized.pose.position.x += 9.0e-6;
  std::vector<moveit_msgs::msg::CollisionObject> desired{resized};
  retain_unmoved_product_geometry(desired, scene_with({in_scene}), change_deadband());
  EXPECT_DOUBLE_EQ(desired.front().pose.position.x, resized.pose.position.x);

  // The deadband is for products only: workcell geometry is static and depth obstacles carry
  // their own retention rule, so neither is pulled back to whatever the scene holds.
  const auto workcell = cylinder("restocker/workcell/shelf/divider", 1.0);
  auto shifted_workcell = workcell;
  shifted_workcell.pose.position.x += 9.0e-6;
  std::vector<moveit_msgs::msg::CollisionObject> workcell_desired{shifted_workcell};
  retain_unmoved_product_geometry(workcell_desired, scene_with({workcell}), change_deadband());
  EXPECT_DOUBLE_EQ(workcell_desired.front().pose.position.x, shifted_workcell.pose.position.x);
}

TEST(PlanningSceneRetention, RetainsNothingWhenTheDeadbandIsZeroOrInvalid)
{
  const auto in_scene = cylinder("restocker/object/1", 0.2);
  auto crept = in_scene;
  crept.pose.position.x += 9.0e-6;

  SceneVerificationConfig zero;
  zero.position_tolerance = 0.0;
  zero.orientation_tolerance_rad = 0.0;
  zero.dimension_tolerance = 1.0e-8;
  std::vector<moveit_msgs::msg::CollisionObject> desired{crept};
  retain_unmoved_product_geometry(desired, scene_with({in_scene}), zero);
  EXPECT_DOUBLE_EQ(desired.front().pose.position.x, crept.pose.position.x);

  SceneVerificationConfig invalid = change_deadband();
  invalid.position_tolerance = std::numeric_limits<double>::quiet_NaN();
  std::vector<moveit_msgs::msg::CollisionObject> also_desired{crept};
  retain_unmoved_product_geometry(also_desired, scene_with({in_scene}), invalid);
  EXPECT_DOUBLE_EQ(also_desired.front().pose.position.x, crept.pose.position.x);
}

}  // namespace
}  // namespace restocker_task_executor
