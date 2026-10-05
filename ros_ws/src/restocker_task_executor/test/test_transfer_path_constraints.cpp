// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "restocker_task_executor/transfer_path_constraints.hpp"

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] Eigen::Isometry3d upright_tool0()
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = Eigen::Vector3d::UnitZ();
  pose.linear().col(1) = Eigen::Vector3d::UnitX();
  pose.linear().col(2) = Eigen::Vector3d::UnitY();
  return pose;
}

TEST(TransferPathConstraints, BuildsAabbFromEndpointsHeldCylinderAndPlannerPadding)
{
  Eigen::Isometry3d start = upright_tool0();
  start.translation() = Eigen::Vector3d{0.10, -1.20, 0.15};
  Eigen::Isometry3d goal = upright_tool0();
  goal.translation() = Eigen::Vector3d{-0.20, 0.05, 0.22};
  const CylinderEnvelope held{0.033, 0.122};

  const auto constraints = build_preinsert_transfer_path_constraints(start, goal, held);
  ASSERT_TRUE(constraints.has_value());
  ASSERT_TRUE(constraints->position_box.has_value());
  ASSERT_TRUE(constraints->orientation_hold.has_value());
  EXPECT_NEAR(
    constraints->orientation_hold->absolute_x_axis_tolerance_rad, std::acos(-1.0), 1.0e-12);
  EXPECT_NEAR(
    constraints->orientation_hold->absolute_y_axis_tolerance_rad,
    kHeldProductMaximumUprightTiltRad / std::sqrt(2.0), 1.0e-12);
  EXPECT_NEAR(
    constraints->orientation_hold->absolute_z_axis_tolerance_rad,
    kHeldProductMaximumUprightTiltRad / std::sqrt(2.0), 1.0e-12);

  const double expand_xy =
    held.radius_m + kPlannerRobotPaddingM + kSurveyedLaneSideClearanceM +
    kDefaultMotionPositionToleranceM + kMoveItPositionConstraintNumericalShellM;
  const double expand_z = 0.5 * held.height_m + kPlannerRobotPaddingM +
    kDefaultMotionPositionToleranceM + kMoveItPositionConstraintNumericalShellM;
  const auto & box = *constraints->position_box;
  EXPECT_DOUBLE_EQ(box.min_corner_m.x(), -0.20 - expand_xy);
  EXPECT_DOUBLE_EQ(box.max_corner_m.x(), 0.10 + expand_xy);
  EXPECT_DOUBLE_EQ(box.min_corner_m.y(), -1.20 - expand_xy);
  EXPECT_DOUBLE_EQ(box.max_corner_m.y(), 0.05 + expand_xy);
  EXPECT_DOUBLE_EQ(box.min_corner_m.z(), 0.15 - expand_z);
  EXPECT_DOUBLE_EQ(box.max_corner_m.z(), 0.22 + expand_z);
  EXPECT_NE(constraints->description.find("z_max="), std::string::npos);
  EXPECT_TRUE(valid_motion_path_constraints(*constraints));
}

TEST(TransferPathConstraints, RefusesNonFiniteOrNonPositiveHeldGeometry)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(pose, pose, CylinderEnvelope{0.0, 0.1}));
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(pose, pose, CylinderEnvelope{0.03, -0.1}));
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      pose, pose, CylinderEnvelope{0.03, 0.1}, -0.001));
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      pose, pose, CylinderEnvelope{0.03, 0.1}, kPlannerRobotPaddingM, -0.001));
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      pose, pose, CylinderEnvelope{0.03, 0.1}, kPlannerRobotPaddingM,
      kSurveyedLaneSideClearanceM, -0.001));
  pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      pose, Eigen::Isometry3d::Identity(), CylinderEnvelope{0.03, 0.1}));
}

TEST(TransferPathConstraints, MeasuresHeldProductAxisTiltIndependentOfWristRoll)
{
  Eigen::Isometry3d upright = Eigen::Isometry3d::Identity();
  // Side-grasp convention: tool0 +X is the held cylinder axis and points up.
  upright.linear() = Eigen::AngleAxisd(
    -0.5 * std::acos(-1.0), Eigen::Vector3d::UnitY()).toRotationMatrix();
  ASSERT_TRUE(held_product_upright_tilt_rad(upright));
  EXPECT_NEAR(*held_product_upright_tilt_rad(upright), 0.0, 1.0e-12);

  Eigen::Isometry3d rolled = upright;
  rolled.linear() *= Eigen::AngleAxisd(
    std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  ASSERT_TRUE(held_product_upright_tilt_rad(rolled));
  EXPECT_NEAR(*held_product_upright_tilt_rad(rolled), 0.0, 1.0e-12);

  Eigen::Isometry3d tipped = upright;
  tipped.linear() = Eigen::AngleAxisd(
    0.05, Eigen::Vector3d::UnitY()).toRotationMatrix() * tipped.linear();
  ASSERT_TRUE(held_product_upright_tilt_rad(tipped));
  EXPECT_NEAR(*held_product_upright_tilt_rad(tipped), 0.05, 1.0e-12);
}

TEST(TransferPathConstraints, RefusesEndpointThatTipsHeldProductPastBudget)
{
  Eigen::Isometry3d upright = Eigen::Isometry3d::Identity();
  upright.linear().col(0) = Eigen::Vector3d::UnitZ();
  upright.linear().col(1) = Eigen::Vector3d::UnitX();
  upright.linear().col(2) = Eigen::Vector3d::UnitY();
  Eigen::Isometry3d tipped = upright;
  tipped.linear() = Eigen::AngleAxisd(
    1.01 * kHeldProductMaximumUprightTiltRad,
    Eigen::Vector3d::UnitY()).toRotationMatrix() * tipped.linear();

  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      upright, tipped, CylinderEnvelope{0.033, 0.122}));
  EXPECT_FALSE(
    build_preinsert_transfer_path_constraints(
      upright, upright, CylinderEnvelope{0.033, 0.122}, kPlannerRobotPaddingM,
      kSurveyedLaneSideClearanceM, kDefaultMotionPositionToleranceM, 0.0));
}

TEST(TransferPathConstraints, DescriptionNamesTheCeiling)
{
  Eigen::Isometry3d start = upright_tool0();
  start.translation() = Eigen::Vector3d::Zero();
  Eigen::Isometry3d goal = upright_tool0();
  goal.translation() = Eigen::Vector3d{1.0, 0.0, 0.5};
  const auto constraints =
    build_preinsert_transfer_path_constraints(start, goal, CylinderEnvelope{0.045, 0.290});
  ASSERT_TRUE(constraints);
  const std::string described = describe_motion_path_position_box(*constraints->position_box);
  EXPECT_NE(described.find("z_max="), std::string::npos);
  EXPECT_NEAR(
    constraints->position_box->max_corner_m.z(),
    0.5 + 0.145 + kPlannerRobotPaddingM + kDefaultMotionPositionToleranceM +
    kMoveItPositionConstraintNumericalShellM,
    1e-12);
}

// MoveIt planned against min X=-0.640500 m, then recomputed an interpolated tool0 pose at
// X=-0.640525 m and rejected the plan. The numerical shell absorbs that solver drift while
// staying smaller than collision padding and surveyed side clearance.
TEST(TransferPathConstraints, ContainsTheLoggedMoveItConstraintInterpolationDrift)
{
  Eigen::Isometry3d start = upright_tool0();
  start.translation() = Eigen::Vector3d{-0.26, -0.727, 0.689};
  Eigen::Isometry3d goal = upright_tool0();
  goal.translation() = Eigen::Vector3d{-0.60, 0.379, 0.9307};

  const auto constraints =
    build_preinsert_transfer_path_constraints(
    start, goal, CylinderEnvelope{0.034, 0.200}, kPlannerRobotPaddingM,
    kSurveyedLaneSideClearanceM, 0.0);
  ASSERT_TRUE(constraints);
  ASSERT_TRUE(constraints->position_box);
  EXPECT_LT(constraints->position_box->min_corner_m.x(), -0.640525);
  EXPECT_DOUBLE_EQ(constraints->position_box->min_corner_m.x(), -0.64075);
  EXPECT_LT(kMoveItPositionConstraintNumericalShellM, kSurveyedLaneSideClearanceM);
  EXPECT_NE(constraints->description.find("numerical_shell="), std::string::npos);
}

TEST(TransferPathConstraints, CarryStartEgressesTowardLaneWithoutCartesianReorientation)
{
  Eigen::Isometry3d retract = Eigen::Isometry3d::Identity();
  // Grasp-style retract: approach −Y (from the rail into the tray), +x up.
  retract.linear().col(0) = Eigen::Vector3d::UnitZ();
  retract.linear().col(1) = Eigen::Vector3d(-1.0, 0.0, 0.0);
  retract.linear().col(2) = Eigen::Vector3d(0.0, -1.0, 0.0);
  retract.translation() = Eigen::Vector3d{0.40, -1.20, 0.12};

  Eigen::Isometry3d preinsert = Eigen::Isometry3d::Identity();
  preinsert.linear().col(0) = Eigen::Vector3d::UnitZ();
  preinsert.linear().col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  preinsert.linear().col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);
  preinsert.translation() = Eigen::Vector3d{-1.0, -0.05, 0.18};

  const CylinderEnvelope can{0.033, 0.122};
  const double min_y_span = 2.0 * can.radius_m + kSurveyedLaneSideClearanceM;
  const auto carry = build_carry_start_tool0_pose(retract, preinsert, can);
  ASSERT_TRUE(carry);
  EXPECT_DOUBLE_EQ(carry->translation().x(), 0.40);
  EXPECT_DOUBLE_EQ(carry->translation().y(), -1.20 + min_y_span);
  EXPECT_DOUBLE_EQ(carry->translation().z(), 0.12);
  EXPECT_GT(carry->translation().y(), retract.translation().y());
  EXPECT_LT(carry->translation().y(), preinsert.translation().y());
  const Eigen::Quaterniond retract_orientation(retract.linear());
  const Eigen::Quaterniond carry_orientation(carry->linear());
  EXPECT_NEAR(retract_orientation.angularDistance(carry_orientation), 0.0, 1.0e-12);
}

// The old target changed orientation with negligible translation, and a later attempt flipped
// 180 degrees over a 71 mm line. The extended egress must make the next motion non-degenerate
// without forcing that flip through Cartesian IK.
TEST(TransferPathConstraints, CarryStartLoggedFailureEgressesWithoutReorientation)
{
  const CylinderEnvelope can{0.033, 0.122};
  const double min_y_span = 2.0 * can.radius_m + kSurveyedLaneSideClearanceM;

  Eigen::Isometry3d retract = Eigen::Isometry3d::Identity();
  retract.linear().col(0) = Eigen::Vector3d::UnitZ();
  retract.linear().col(1) = Eigen::Vector3d(-1.0, 0.0, 0.0);
  retract.linear().col(2) = Eigen::Vector3d(0.0, -1.0, 0.0);
  retract.translation() = Eigen::Vector3d{-0.4200, -0.5400, 0.6810};

  Eigen::Isometry3d preinsert = Eigen::Isometry3d::Identity();
  preinsert.linear().col(0) = Eigen::Vector3d::UnitZ();
  preinsert.linear().col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  preinsert.linear().col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);  // approach +Y into the lane
  preinsert.translation() = Eigen::Vector3d{-1.0000, 0.3780, 0.9217};

  const auto carry = build_carry_start_tool0_pose(retract, preinsert, can);
  ASSERT_TRUE(carry);
  EXPECT_DOUBLE_EQ(carry->translation().x(), -0.42);
  EXPECT_DOUBLE_EQ(carry->translation().z(), 0.6810);
  EXPECT_DOUBLE_EQ(carry->translation().y(), -0.5400 + min_y_span);
  EXPECT_NE(carry->translation().y(), retract.translation().y());
  EXPECT_GE(std::abs(carry->translation().y() - retract.translation().y()), min_y_span - 1e-12);
  const Eigen::Quaterniond retract_orientation(retract.linear());
  const Eigen::Quaterniond carry_orientation(carry->linear());
  EXPECT_NEAR(
    retract_orientation.angularDistance(carry_orientation), 0.0, 1.0e-12);

  // Goal stays inside the fail-closed retract→carry AABB.
  const auto box = build_preinsert_transfer_path_constraints(retract, *carry, can);
  ASSERT_TRUE(box);
  ASSERT_TRUE(box->position_box);
  EXPECT_LE(box->position_box->min_corner_m.y(), carry->translation().y());
  EXPECT_GE(box->position_box->max_corner_m.y(), retract.translation().y());
  EXPECT_LE(box->position_box->min_corner_m.x(), carry->translation().x());
  EXPECT_GE(box->position_box->max_corner_m.x(), carry->translation().x());
}

TEST(TransferPathConstraints, CarryStartUsesExactlyOneProductDiameterClearance)
{
  const CylinderEnvelope can{0.033, 0.122};
  const double min_y_span = 2.0 * can.radius_m + kSurveyedLaneSideClearanceM;
  Eigen::Isometry3d retract = Eigen::Isometry3d::Identity();
  retract.translation().y() = 0.0;
  Eigen::Isometry3d preinsert = Eigen::Isometry3d::Identity();
  preinsert.translation().y() = 0.25;

  const auto carry = build_carry_start_tool0_pose(retract, preinsert, can);
  ASSERT_TRUE(carry);
  EXPECT_NEAR(carry->translation().y(), min_y_span, 1.0e-12);
  EXPECT_LT(carry->translation().y(), preinsert.translation().y());
}

TEST(TransferPathConstraints, LargeBottleCarryStartDoesNotAddPerpendicularGripperWidth)
{
  Eigen::Isometry3d retract = Eigen::Isometry3d::Identity();
  retract.translation() = Eigen::Vector3d{0.60, -0.540, 0.780};
  Eigen::Isometry3d preinsert = Eigen::Isometry3d::Identity();
  preinsert.translation() = Eigen::Vector3d{-0.20, 0.380, 1.020};
  const CylinderEnvelope bottle{0.045, 0.290};

  const auto carry = build_carry_start_tool0_pose(retract, preinsert, bottle);

  ASSERT_TRUE(carry);
  EXPECT_NEAR(carry->translation().y(), -0.445, 1.0e-12);
  // Gripper width (0.11 m) is along the jaw axis and must not affect this +Y egress.
  EXPECT_LT(carry->translation().y(), -0.335);
  EXPECT_TRUE(carry->linear().isApprox(retract.linear(), 1.0e-12));
}

TEST(TransferPathConstraints, CarryStartRefusesNonFinitePoses)
{
  Eigen::Isometry3d bad = Eigen::Isometry3d::Identity();
  bad.translation().x() = std::numeric_limits<double>::quiet_NaN();
  const CylinderEnvelope can{0.033, 0.122};
  EXPECT_FALSE(build_carry_start_tool0_pose(bad, Eigen::Isometry3d::Identity(), can));
  EXPECT_FALSE(build_carry_start_tool0_pose(Eigen::Isometry3d::Identity(), bad, can));
}

TEST(TransferPathConstraints, CarryStartUsesLaneApproachForDegenerateY)
{
  const CylinderEnvelope can{0.033, 0.122};
  Eigen::Isometry3d retract = Eigen::Isometry3d::Identity();
  retract.translation() = Eigen::Vector3d{0.0, -0.54, 0.69};
  // Retract has no Y approach component; the lane frame still supplies the +Y egress direction.
  retract.linear().col(0) = Eigen::Vector3d::UnitZ();
  retract.linear().col(1) = Eigen::Vector3d(0.0, 1.0, 0.0);
  // Keep this a proper right-handed rotation (Z x Y = -X), not a reflection.
  retract.linear().col(2) = Eigen::Vector3d(-1.0, 0.0, 0.0);

  Eigen::Isometry3d preinsert = Eigen::Isometry3d::Identity();
  preinsert.linear().col(0) = Eigen::Vector3d::UnitZ();
  preinsert.linear().col(1) = Eigen::Vector3d(1.0, 0.0, 0.0);
  preinsert.linear().col(2) = Eigen::Vector3d(0.0, 1.0, 0.0);
  preinsert.translation() = Eigen::Vector3d{-0.6, -0.54, 0.69};
  const auto carry = build_carry_start_tool0_pose(retract, preinsert, can);
  ASSERT_TRUE(carry);
  EXPECT_GT(carry->translation().y(), retract.translation().y());
  const Eigen::Quaterniond retract_orientation(retract.linear());
  const Eigen::Quaterniond carry_orientation(carry->linear());
  EXPECT_NEAR(retract_orientation.angularDistance(carry_orientation), 0.0, 1.0e-12);
}

// An unconstrained OMPL ceiling above stocked columns rakes over a packed lane. The
// survey-derived transfer box must stay at endpoint height so it cannot authorise that flyover
// when its footprint crosses the column.
TEST(TransferPathConstraints, TransferBoxDoesNotAuthoriseOverflightAboveAPackedLaneColumn)
{
  Eigen::Isometry3d retract = upright_tool0();
  retract.translation() = Eigen::Vector3d{-0.42, -1.08, 0.72};
  Eigen::Isometry3d preinsert = upright_tool0();
  preinsert.translation() = Eigen::Vector3d{-1.0, 0.02, 0.78};
  const CylinderEnvelope can{0.033, 0.122};

  const auto constraints = build_preinsert_transfer_path_constraints(retract, preinsert, can);
  ASSERT_TRUE(constraints);
  ASSERT_TRUE(constraints->position_box);

  // Column standing in the transfer Y band (as a packed inventory obstacle the corridor crosses).
  Eigen::AlignedBox3d column;
  column.extend(Eigen::Vector3d{-1.05, -0.40, 0.70});
  column.extend(Eigen::Vector3d{-0.95, 0.00, 0.95});

  EXPECT_FALSE(path_box_overflies_lane_column(*constraints->position_box, column));
  EXPECT_LT(
    constraints->position_box->max_corner_m.z(), column.max().z());

  // A high unconstrained corridor with the same XY footprint would rake over that column.
  MotionPathPositionBox flyover = *constraints->position_box;
  flyover.max_corner_m.z() = column.max().z() + 0.25;
  EXPECT_TRUE(path_box_overflies_lane_column(flyover, column));
}

}  // namespace
}  // namespace restocker_task_executor
