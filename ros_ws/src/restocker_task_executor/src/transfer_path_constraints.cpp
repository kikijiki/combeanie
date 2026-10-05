// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/transfer_path_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "restocker_task_executor/manipulation_geometry.hpp"

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool finite_pose(const Eigen::Isometry3d & pose) noexcept
{
  const auto & matrix = pose.matrix();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!std::isfinite(matrix(row, column))) {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

std::string describe_motion_path_position_box(const MotionPathPositionBox & box)
{
  const Eigen::Vector3d extents = box.max_corner_m - box.min_corner_m;
  const Eigen::Vector3d center = 0.5 * (box.min_corner_m + box.max_corner_m);
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(4);
  stream << "transfer box min=(" << box.min_corner_m.x() << ", " << box.min_corner_m.y()
         << ", " << box.min_corner_m.z() << ") max=(" << box.max_corner_m.x() << ", "
         << box.max_corner_m.y() << ", " << box.max_corner_m.z() << ") center=("
         << center.x() << ", " << center.y() << ", " << center.z() << ") extents=("
         << extents.x() << ", " << extents.y() << ", " << extents.z() << ") z_max="
         << box.max_corner_m.z();
  return stream.str();
}

std::optional<double> held_product_upright_tilt_rad(
  const Eigen::Isometry3d & world_from_tool0) noexcept
{
  if (!finite_pose(world_from_tool0) || !world_from_tool0.linear().isUnitary(1.0e-6) ||
    std::abs(world_from_tool0.linear().determinant() - 1.0) > 1.0e-6)
  {
    return std::nullopt;
  }
  const double alignment = std::clamp(
    world_from_tool0.linear().col(0).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  return std::acos(alignment);
}

std::optional<MotionPathConstraints> build_preinsert_transfer_path_constraints(
  const Eigen::Isometry3d & start_tool0, const Eigen::Isometry3d & goal_tool0,
  const CylinderEnvelope & held_product, double planner_padding_m, double lateral_clearance_m,
  double goal_position_tolerance_m, double maximum_upright_tilt_rad)
{
  const auto start_tilt = held_product_upright_tilt_rad(start_tool0);
  const auto goal_tilt = held_product_upright_tilt_rad(goal_tool0);
  if (!finite_pose(start_tool0) || !finite_pose(goal_tool0) ||
    !std::isfinite(held_product.radius_m) || held_product.radius_m <= 0.0 ||
    !std::isfinite(held_product.height_m) || held_product.height_m <= 0.0 ||
    !std::isfinite(planner_padding_m) || planner_padding_m < 0.0 ||
    !std::isfinite(lateral_clearance_m) || lateral_clearance_m < 0.0 ||
    !std::isfinite(goal_position_tolerance_m) || goal_position_tolerance_m < 0.0 ||
    !std::isfinite(maximum_upright_tilt_rad) || maximum_upright_tilt_rad <= 0.0 ||
    maximum_upright_tilt_rad >= 0.5 * std::acos(-1.0) || !start_tilt || !goal_tilt ||
    *start_tilt > maximum_upright_tilt_rad || *goal_tilt > maximum_upright_tilt_rad)
  {
    return std::nullopt;
  }

  const Eigen::Vector3d start = start_tool0.translation();
  const Eigen::Vector3d goal = goal_tool0.translation();
  const double expand_xy =
    held_product.radius_m + planner_padding_m + lateral_clearance_m +
    goal_position_tolerance_m + kMoveItPositionConstraintNumericalShellM;
  const double expand_z = 0.5 * held_product.height_m + planner_padding_m +
    goal_position_tolerance_m + kMoveItPositionConstraintNumericalShellM;

  MotionPathPositionBox box;
  box.min_corner_m.x() = std::min(start.x(), goal.x()) - expand_xy;
  box.max_corner_m.x() = std::max(start.x(), goal.x()) + expand_xy;
  box.min_corner_m.y() = std::min(start.y(), goal.y()) - expand_xy;
  box.max_corner_m.y() = std::max(start.y(), goal.y()) + expand_xy;
  box.min_corner_m.z() = std::min(start.z(), goal.z()) - expand_z;
  box.max_corner_m.z() = std::max(start.z(), goal.z()) + expand_z;

  MotionPathConstraints constraints;
  constraints.position_box = box;
  // The start and goal differ by 180 degrees around tool0 +X: that wrist rotation changes the
  // approach direction but leaves the can vertical. Intrinsic XYZ lets MoveIt rotate freely about
  // +X while bounding both tipping components. The independent geodesic axis check below remains
  // authoritative because component tolerances are only a planning representation.
  const double component_tolerance = maximum_upright_tilt_rad / std::sqrt(2.0);
  constraints.orientation_hold = MotionPathOrientationHold{
    Eigen::Quaterniond(start_tool0.linear()).normalized(), std::acos(-1.0),
    component_tolerance, component_tolerance};
  constraints.description = describe_motion_path_position_box(box) +
    " held_r=" + std::to_string(held_product.radius_m) +
    " held_h=" + std::to_string(held_product.height_m) +
    " padding=" + std::to_string(planner_padding_m) +
    " lateral=" + std::to_string(lateral_clearance_m) +
    " goal_tolerance=" + std::to_string(goal_position_tolerance_m) +
    " numerical_shell=" + std::to_string(kMoveItPositionConstraintNumericalShellM) +
    " upright_tool0_x_max_rad=" + std::to_string(maximum_upright_tilt_rad);
  if (!valid_motion_path_constraints(constraints)) {
    return std::nullopt;
  }
  return constraints;
}

std::optional<Eigen::Isometry3d> build_carry_start_tool0_pose(
  const Eigen::Isometry3d & retract_tool0, const Eigen::Isometry3d & preinsert_tool0,
  const CylinderEnvelope & held_product, double lateral_clearance_m)
{
  if (!finite_pose(retract_tool0) || !finite_pose(preinsert_tool0) ||
    !std::isfinite(held_product.radius_m) || held_product.radius_m <= 0.0 ||
    !std::isfinite(held_product.height_m) || held_product.height_m <= 0.0 ||
    !std::isfinite(lateral_clearance_m) || lateral_clearance_m < 0.0)
  {
    return std::nullopt;
  }

  // Keep retract X/Z and orientation while clearing the held-product diameter and surveyed side
  // clearance. In this workcell +Y is from the stock tray toward the robot and rear lane mouth.
  // Gripper body Y is the jaw-width axis in this pose, perpendicular to this egress, so adding it
  // here over-translates the goal: the logged large-bottle path stopped at 97.619% of that target
  // even with collision checking disabled. Reorientation belongs to the collision-aware
  // free-space PreInsert: live Cartesian interpolation of that rotation repeatedly folded wrist_3
  // into the forearm even though the endpoint was reachable.
  const Eigen::Vector3d retract = retract_tool0.translation();
  const Eigen::Vector3d preinsert = preinsert_tool0.translation();
  const double min_y_span = 2.0 * held_product.radius_m + lateral_clearance_m;
  Eigen::Vector3d translation;
  translation.x() = retract.x();
  translation.z() = retract.z();

  const double delta_y = preinsert.y() - retract.y();
  if (std::abs(delta_y) < min_y_span) {
    Eigen::Vector3d egress = grasp_approach_axis(preinsert_tool0);
    egress.z() = 0.0;
    if (std::abs(egress.y()) <= 1.0e-9) {
      egress = grasp_approach_axis(retract_tool0);
      egress.z() = 0.0;
    }
    if (std::abs(egress.y()) <= 1.0e-9) {
      return std::nullopt;
    }
    translation.y() = retract.y() + std::copysign(min_y_span, egress.y());
  } else {
    translation.y() = retract.y() + std::copysign(min_y_span, delta_y);
  }

  Eigen::Isometry3d pose = retract_tool0;
  pose.translation() = translation;
  if (!finite_pose(pose)) {
    return std::nullopt;
  }
  return pose;
}

bool path_box_overflies_lane_column(
  const MotionPathPositionBox & box, const Eigen::AlignedBox3d & column_bounds_in_planning)
{
  if (column_bounds_in_planning.isEmpty() ||
    !column_bounds_in_planning.min().allFinite() ||
    !column_bounds_in_planning.max().allFinite() ||
    !box.min_corner_m.allFinite() || !box.max_corner_m.allFinite() ||
    (box.max_corner_m.array() < box.min_corner_m.array()).any())
  {
    return false;
  }
  const bool y_overlaps =
    box.min_corner_m.y() <= column_bounds_in_planning.max().y() &&
    box.max_corner_m.y() >= column_bounds_in_planning.min().y();
  const bool x_overlaps =
    box.min_corner_m.x() <= column_bounds_in_planning.max().x() &&
    box.max_corner_m.x() >= column_bounds_in_planning.min().x();
  // Clears the column top with no solid intersection required (the visual "sweep over" rake).
  const bool clears_top = box.max_corner_m.z() > column_bounds_in_planning.max().z();
  return y_overlaps && x_overlaps && clears_top;
}

}  // namespace restocker_task_executor
