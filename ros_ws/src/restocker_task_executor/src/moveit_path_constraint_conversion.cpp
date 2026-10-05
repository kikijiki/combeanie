// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/moveit_path_constraint_conversion.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace restocker_task_executor
{

moveit_msgs::msg::Constraints to_moveit_path_constraints(
  const MotionPathConstraints & source, const std::string & default_frame,
  const std::string & default_link)
{
  moveit_msgs::msg::Constraints constraints;
  const std::string frame =
    source.planning_frame.empty() ? default_frame : source.planning_frame;
  const std::string link = source.link_name.empty() ? default_link : source.link_name;

  if (source.position_box) {
    const auto & box = *source.position_box;
    moveit_msgs::msg::PositionConstraint position;
    position.header.frame_id = frame;
    position.link_name = link;
    position.weight = 1.0;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    const Eigen::Vector3d extents = box.max_corner_m - box.min_corner_m;
    const Eigen::Vector3d center = 0.5 * (box.min_corner_m + box.max_corner_m);
    primitive.dimensions = {extents.x(), extents.y(), extents.z()};
    geometry_msgs::msg::Pose pose;
    pose.position.x = center.x();
    pose.position.y = center.y();
    pose.position.z = center.z();
    pose.orientation.w = 1.0;
    position.constraint_region.primitives.push_back(primitive);
    position.constraint_region.primitive_poses.push_back(pose);
    constraints.position_constraints.push_back(std::move(position));
  }

  if (source.orientation_hold) {
    const auto & hold = *source.orientation_hold;
    moveit_msgs::msg::OrientationConstraint orientation;
    orientation.header.frame_id = frame;
    orientation.link_name = link;
    orientation.orientation.x = hold.orientation.x();
    orientation.orientation.y = hold.orientation.y();
    orientation.orientation.z = hold.orientation.z();
    orientation.orientation.w = hold.orientation.w();
    orientation.absolute_x_axis_tolerance = hold.absolute_x_axis_tolerance_rad;
    orientation.absolute_y_axis_tolerance = hold.absolute_y_axis_tolerance_rad;
    orientation.absolute_z_axis_tolerance = hold.absolute_z_axis_tolerance_rad;
    // The upright product axis is tool0 +X. Intrinsic XYZ permits the necessary wrist roll about
    // that axis while the Y/Z component tolerances prevent the planner from tipping it.
    orientation.parameterization = moveit_msgs::msg::OrientationConstraint::XYZ_EULER_ANGLES;
    orientation.weight = 1.0;
    constraints.orientation_constraints.push_back(std::move(orientation));
  }
  return constraints;
}

}  // namespace restocker_task_executor
