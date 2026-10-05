// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/moveit_path_constraint_conversion.hpp"
#include "restocker_task_executor/transfer_path_constraints.hpp"

namespace restocker_task_executor
{
namespace
{

TEST(MoveItPathConstraintConversion, MapsPreInsertBoxToMoveItPositionConstraint)
{
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  start.linear().col(0) = Eigen::Vector3d::UnitZ();
  start.linear().col(1) = Eigen::Vector3d::UnitX();
  start.linear().col(2) = Eigen::Vector3d::UnitY();
  start.translation() = Eigen::Vector3d{0.0, -1.0, 0.1};
  Eigen::Isometry3d goal = start;
  goal.translation() = Eigen::Vector3d{0.4, 0.0, 0.2};
  auto built = build_preinsert_transfer_path_constraints(
    start, goal, CylinderEnvelope{0.034, 0.200});
  ASSERT_TRUE(built);
  built->planning_frame = "world";
  built->link_name = "tool0";

  const auto constraints = to_moveit_path_constraints(*built, "fallback", "other_link");
  ASSERT_EQ(constraints.position_constraints.size(), 1U);
  ASSERT_EQ(constraints.orientation_constraints.size(), 1U);
  EXPECT_EQ(
    constraints.orientation_constraints.front().parameterization,
    moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES);
  const auto & position = constraints.position_constraints.front();
  EXPECT_EQ(position.header.frame_id, "world");
  EXPECT_EQ(position.link_name, "tool0");
  ASSERT_EQ(position.constraint_region.primitives.size(), 1U);
  EXPECT_EQ(
    position.constraint_region.primitives.front().type, shape_msgs::msg::SolidPrimitive::BOX);
  const auto & dims = position.constraint_region.primitives.front().dimensions;
  ASSERT_EQ(dims.size(), 3U);
  const auto & box = *built->position_box;
  EXPECT_DOUBLE_EQ(dims[0], box.max_corner_m.x() - box.min_corner_m.x());
  EXPECT_DOUBLE_EQ(dims[1], box.max_corner_m.y() - box.min_corner_m.y());
  EXPECT_DOUBLE_EQ(dims[2], box.max_corner_m.z() - box.min_corner_m.z());
  ASSERT_EQ(position.constraint_region.primitive_poses.size(), 1U);
  EXPECT_DOUBLE_EQ(
    position.constraint_region.primitive_poses.front().position.z,
    0.5 * (box.min_corner_m.z() + box.max_corner_m.z()));
}

TEST(MoveItPathConstraintConversion, FillsDefaultFrameAndLinkWhenUnset)
{
  MotionPathConstraints source;
  source.position_box = MotionPathPositionBox{
    Eigen::Vector3d{-0.1, -0.1, -0.1}, Eigen::Vector3d{0.1, 0.1, 0.1}};
  const auto constraints = to_moveit_path_constraints(source, "world", "tool0");
  ASSERT_EQ(constraints.position_constraints.size(), 1U);
  EXPECT_EQ(constraints.position_constraints.front().header.frame_id, "world");
  EXPECT_EQ(constraints.position_constraints.front().link_name, "tool0");
}

TEST(MoveItPathConstraintConversion, MapsOrientationHoldWhenPresent)
{
  MotionPathConstraints source;
  source.orientation_hold = MotionPathOrientationHold{
    Eigen::Quaterniond::Identity(), 0.02, 0.03, 0.04};
  const auto constraints = to_moveit_path_constraints(source, "world", "tool0");
  ASSERT_EQ(constraints.orientation_constraints.size(), 1U);
  EXPECT_DOUBLE_EQ(constraints.orientation_constraints.front().absolute_x_axis_tolerance, 0.02);
  EXPECT_DOUBLE_EQ(constraints.orientation_constraints.front().absolute_y_axis_tolerance, 0.03);
  EXPECT_DOUBLE_EQ(constraints.orientation_constraints.front().absolute_z_axis_tolerance, 0.04);
}

}  // namespace
}  // namespace restocker_task_executor
