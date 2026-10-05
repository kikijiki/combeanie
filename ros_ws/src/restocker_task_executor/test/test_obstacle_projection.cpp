// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <string>
#include <utility>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/obstacle_projection.hpp"
#include "restocker_task_executor/planning_scene_reconciliation.hpp"

namespace
{

using restocker_task_executor::ObstacleProjectionConfig;
using restocker_task_executor::ProjectionErrorCode;
using restocker_task_executor::is_projector_managed_id;
using restocker_task_executor::obstacle_sets_match;
using restocker_task_executor::project_obstacle_observation;
using Observation = restocker_interfaces::msg::ObstacleObservation;
using Primitive = shape_msgs::msg::SolidPrimitive;

[[nodiscard]] ObstacleProjectionConfig config()
{
  ObstacleProjectionConfig value;
  value.planning_frame = "world";
  return value;
}

[[nodiscard]] Observation observation(
  const std::vector<std::array<double, 6>> & boxes, bool robot_static = true)
{
  Observation message;
  message.header.frame_id = "world";
  message.header.stamp.sec = 12;
  message.sensor_frame = "overhead_camera_optical_frame";
  message.sequence = 7U;
  message.robot_static = robot_static;
  for (const auto & box : boxes) {
    restocker_interfaces::msg::ObstacleBox entry;
    entry.center.x = box[0];
    entry.center.y = box[1];
    entry.center.z = box[2];
    entry.size.x = box[3];
    entry.size.y = box[4];
    entry.size.z = box[5];
    entry.point_count = 250U;
    message.boxes.push_back(entry);
  }
  return message;
}

[[nodiscard]] moveit_msgs::msg::CollisionObject known_box(
  std::string id, double x, double y, double z, double sx, double sy, double sz)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = std::move(id);
  object.header.frame_id = "world";
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.position.x = x;
  object.pose.position.y = y;
  object.pose.position.z = z;
  object.pose.orientation.w = 1.0;
  Primitive primitive;
  primitive.type = Primitive::BOX;
  primitive.dimensions = {sx, sy, sz};
  object.primitives.push_back(primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  object.primitive_poses.push_back(identity);
  return object;
}

[[nodiscard]] moveit_msgs::msg::CollisionObject known_cylinder(
  std::string id, double x, double y, double z, double radius, double height)
{
  auto object = known_box(std::move(id), x, y, z, 1.0, 1.0, 1.0);
  object.primitives.front().type = Primitive::CYLINDER;
  object.primitives.front().dimensions = {height, radius};
  return object;
}

TEST(ObstacleProjection, ProjectsPaddedBoxesIntoTheManagedNamespace) {
  const auto result = project_obstacle_observation(
    observation({{0.60, -0.70, 0.25, 0.15, 0.15, 0.50}}), {}, {}, config());

  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 1U);
  const auto & object = result.value().front();
  EXPECT_EQ(object.id, "restocker/obstacle/0");
  // The obstacle must live inside the namespace the projector already reconciles and verifies.
  // Outside it the diff would neither add nor remove it, and no proof would cover it.
  EXPECT_TRUE(is_projector_managed_id(object.id));
  EXPECT_EQ(object.header.frame_id, "world");
  EXPECT_EQ(object.operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(object.primitives.size(), 1U);
  EXPECT_EQ(object.primitives.front().type, Primitive::BOX);
  ASSERT_EQ(object.primitives.front().dimensions.size(), 3U);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[0], 0.15 + 2.0 * 0.03);
  EXPECT_DOUBLE_EQ(object.primitives.front().dimensions[2], 0.50 + 2.0 * 0.03);
  EXPECT_DOUBLE_EQ(object.pose.position.x, 0.60);
  EXPECT_DOUBLE_EQ(object.pose.orientation.w, 1.0);
  ASSERT_EQ(object.primitive_poses.size(), 1U);
  EXPECT_DOUBLE_EQ(object.primitive_poses.front().orientation.w, 1.0);
}

TEST(ObstacleProjection, DropsAReObservationOfModelledGeometryButKeepsWhatMerelyTouchesIt) {
  const std::vector<moveit_msgs::msg::CollisionObject> known{
    known_box("restocker/workcell/roller_bed", 0.0, 0.0, 0.80, 2.40, 0.90, 0.04)};

  // A cluster lying inside the lane floor is the floor itself, seen by the camera.
  const auto inside = project_obstacle_observation(
    observation({{0.0, 0.0, 0.80, 0.10, 0.10, 0.10}}), known, {}, config());
  ASSERT_TRUE(inside);
  EXPECT_TRUE(inside.value().empty());

  // A crate standing on that same floor touches it and is not it. Discarding an
  // obstacle for touching known geometry would turn "something is there" into "nothing is
  // there", the one direction this gate must never fail in.
  const auto standing = project_obstacle_observation(
    observation({{0.0, 0.0, 1.05, 0.30, 0.30, 0.30}}), known, {}, config());
  ASSERT_TRUE(standing);
  ASSERT_EQ(standing.value().size(), 1U);
  EXPECT_DOUBLE_EQ(standing.value().front().pose.position.z, 1.05);
}

TEST(ObstacleProjection, DropsTheProductTheGripperIsCarrying) {
  const std::vector<moveit_msgs::msg::CollisionObject> attached{
    known_cylinder("restocker/object/4", 0.50, -0.30, 0.90, 0.035, 0.20)};

  const auto result = project_obstacle_observation(
    observation({{0.50, -0.30, 0.90, 0.05, 0.05, 0.05}}), {}, attached, config());

  ASSERT_TRUE(result);
  EXPECT_TRUE(result.value().empty())
    << "a carried product reported as an obstacle makes every later insert unplannable";
}

TEST(ObstacleProjection, DiscardsClustersOutsideTheAdmissibleSizeBand) {
  auto narrow = config();
  narrow.obstacle_padding_m = 0.0;
  const auto result = project_obstacle_observation(
    observation(
      {{0.0, 0.0, 0.5, 0.01, 0.01, 0.01},
        {1.0, 0.0, 0.5, 3.00, 0.05, 0.05},
        {-1.0, 0.0, 0.5, 0.20, 0.20, 0.20}}),
    {}, {}, narrow);

  ASSERT_TRUE(result);
  ASSERT_EQ(result.value().size(), 1U);
  EXPECT_DOUBLE_EQ(result.value().front().pose.position.x, -1.0);
  // Identifiers are positional over the surviving set, so the survivor is numbered from zero.
  EXPECT_EQ(result.value().front().id, "restocker/obstacle/0");
}

TEST(ObstacleProjection, RefusesEvidenceItCannotPlace) {
  auto foreign = observation({{0.0, 0.0, 0.5, 0.2, 0.2, 0.2}});
  foreign.header.frame_id = "overhead_camera_optical_frame";
  const auto mismatched = project_obstacle_observation(foreign, {}, {}, config());
  ASSERT_FALSE(mismatched);
  EXPECT_EQ(mismatched.error().code, ProjectionErrorCode::FrameMismatch);

  auto unsequenced = observation({{0.0, 0.0, 0.5, 0.2, 0.2, 0.2}});
  unsequenced.sequence = 0U;
  const auto anonymous = project_obstacle_observation(unsequenced, {}, {}, config());
  ASSERT_FALSE(anonymous);
  EXPECT_EQ(anonymous.error().code, ProjectionErrorCode::InvalidIdentity);

  auto degenerate = observation({{0.0, 0.0, 0.5, 0.2, 0.0, 0.2}});
  const auto flat = project_obstacle_observation(degenerate, {}, {}, config());
  ASSERT_FALSE(flat);
  EXPECT_EQ(flat.error().code, ProjectionErrorCode::InvalidPose);

  auto broken = config();
  broken.known_containment_fraction = 0.0;
  const auto misconfigured = project_obstacle_observation(
    observation({{0.0, 0.0, 0.5, 0.2, 0.2, 0.2}}), {}, {}, broken);
  ASSERT_FALSE(misconfigured);
  EXPECT_EQ(misconfigured.error().code, ProjectionErrorCode::InvalidConfiguration);
}

TEST(ObstacleProjection, MatchesSetsThatHaveOnlyJittered) {
  const auto accepted = project_obstacle_observation(
    observation({{0.60, -0.70, 0.25, 0.15, 0.15, 0.50}}), {}, {}, config());
  ASSERT_TRUE(accepted);

  // Depth noise of a few millimetres per frame must not read as new geometry: every change to the
  // desired set produces a planning-scene diff, and every diff advances the content generation
  // that invalidates a plan in flight.
  const auto jittered = project_obstacle_observation(
    observation({{0.607, -0.694, 0.252, 0.152, 0.148, 0.503}}), {}, {}, config());
  ASSERT_TRUE(jittered);
  EXPECT_TRUE(obstacle_sets_match(accepted.value(), jittered.value(), config()));

  // A real move must read as new geometry.
  const auto moved = project_obstacle_observation(
    observation({{0.90, -0.70, 0.25, 0.15, 0.15, 0.50}}), {}, {}, config());
  ASSERT_TRUE(moved);
  EXPECT_FALSE(obstacle_sets_match(accepted.value(), moved.value(), config()));

  // So must a change in how many obstacles there are, in either direction.
  const auto arrived = project_obstacle_observation(
    observation({{0.60, -0.70, 0.25, 0.15, 0.15, 0.50}, {-0.60, -0.70, 0.25, 0.15, 0.15, 0.50}}),
    {}, {}, config());
  ASSERT_TRUE(arrived);
  EXPECT_FALSE(obstacle_sets_match(accepted.value(), arrived.value(), config()));
  EXPECT_FALSE(obstacle_sets_match(arrived.value(), accepted.value(), config()));
  EXPECT_TRUE(obstacle_sets_match({}, {}, config()));
}

TEST(ObstacleProjection, HonoursTheReportedObstacleCeiling) {
  auto capped = config();
  capped.max_obstacles = 2U;
  const auto result = project_obstacle_observation(
    observation(
      {{-0.9, 0.0, 0.5, 0.2, 0.2, 0.2},
        {-0.3, 0.0, 0.5, 0.2, 0.2, 0.2},
        {0.3, 0.0, 0.5, 0.2, 0.2, 0.2},
        {0.9, 0.0, 0.5, 0.2, 0.2, 0.2}}),
    {}, {}, capped);

  ASSERT_TRUE(result);
  EXPECT_EQ(result.value().size(), 2U);
}

}  // namespace
