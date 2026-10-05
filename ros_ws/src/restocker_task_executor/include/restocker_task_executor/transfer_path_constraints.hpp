// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <optional>
#include <string>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{

// Matches robot_description_planning.default_robot_padding in move_group.launch.py and
// restocker_gazebo.scenario_config.PLANNER_ROBOT_PADDING_M. The transfer box expands by this so
// the constrained tool0 corridor agrees with the padded collision model the planner actually uses.
inline constexpr double kPlannerRobotPaddingM = 0.0015;

// Matches lanes.*.side_clearance_m in workcell_geometry.yaml. Extra XY expand so the tool0 AABB
// leaves the same surveyed lateral seating margin the catalog already trusts: not a flyover
// height, and not an invented corridor width.
inline constexpr double kSurveyedLaneSideClearanceM = 0.005;

// MoveIt's constraint interpolation put tool0 0.025 mm outside an exact box face in the recorded
// second baseline transfer. The inner planning box now supplies 5 mm of face margin; this remains
// only a small numerical shell on the outer validator.
inline constexpr double kMoveItPositionConstraintNumericalShellM = 0.00025;

// Two degrees. The held upright-cylinder axis is tool0 +X for every generated side grasp. This is
// both the hard, densely sampled post-plan tilt budget and the endpoint-admission budget.
inline constexpr double kHeldProductMaximumUprightTiltRad = 0.03490658503988659;

// Builds the PreInsert free-space transfer box from segment endpoints and the held product's
// catalogued cylinder. Expands XY by radius + planner padding + surveyed side clearance and Z by
// half-height + padding. Both also include the planner's owned Cartesian goal tolerance: MoveIt is
// allowed to end that far from the requested endpoint, so omitting it would reject an otherwise
// in-contract endpoint. The numerical shell is added last. The planner receives a separate inset
// copy and the completed path is checked against this outer box, so the corridor accounts for the
// upright product extent without inventing a flyover height.
// Returns nullopt when inputs are not finite or the held envelope is unusable (fail closed).
[[nodiscard]] std::optional<MotionPathConstraints> build_preinsert_transfer_path_constraints(
  const Eigen::Isometry3d & start_tool0, const Eigen::Isometry3d & goal_tool0,
  const CylinderEnvelope & held_product, double planner_padding_m = kPlannerRobotPaddingM,
  double lateral_clearance_m = kSurveyedLaneSideClearanceM,
  double goal_position_tolerance_m = kDefaultMotionPositionToleranceM,
  double maximum_upright_tilt_rad = kHeldProductMaximumUprightTiltRad);

// Angle from tool0 +X (the held cylinder's axis) to world/planning +Z. A malformed transform has
// no measurement instead of being treated as upright.
[[nodiscard]] std::optional<double> held_product_upright_tilt_rad(
  const Eigen::Isometry3d & world_from_tool0) noexcept;

// Geometry-derived carry-start tool0 pose between retract and PreInsert. Keeps retract X/Z and
// orientation; Y moves toward the lane mouth by one product diameter + surveyed side clearance.
// The following collision-aware PreInsert performs the reorientation;
// Cartesian interpolation of it folded the wrist into the forearm on the logged failure path.
// No joint angles, flyover vias, or demo polyline.
// Returns nullopt when poses/envelope are unusable or the rearward Y direction cannot be derived.
[[nodiscard]] std::optional<Eigen::Isometry3d> build_carry_start_tool0_pose(
  const Eigen::Isometry3d & retract_tool0, const Eigen::Isometry3d & preinsert_tool0,
  const CylinderEnvelope & held_product,
  double lateral_clearance_m = kSurveyedLaneSideClearanceM);

// True when a tool0 path box's Y span overlaps a lane column's planning-frame Y interval while
// the box's max Z clears the column top: the over-shelf rake that would knock a packed lane in
// reality even when the box never intersects the column solid. Used by tests to lock the
// endpoint-derived transfer corridor below stocked-column height.
[[nodiscard]] bool path_box_overflies_lane_column(
  const MotionPathPositionBox & box, const Eigen::AlignedBox3d & column_bounds_in_planning);

// Formats the box corners and extents for planning logs.
[[nodiscard]] std::string describe_motion_path_position_box(const MotionPathPositionBox & box);

}  // namespace restocker_task_executor
