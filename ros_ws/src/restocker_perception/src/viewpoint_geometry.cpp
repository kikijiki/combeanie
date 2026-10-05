// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/viewpoint_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace restocker_perception
{
namespace
{

template<typename Value>
[[nodiscard]] Result<Value> failure(PerceptionErrorCode code, std::string detail)
{
  return Result<Value>::failure(PerceptionError{code, std::move(detail)});
}

[[nodiscard]] std::string format_number(double value, int precision = 4)
{
  std::ostringstream stream;
  stream.setf(std::ios::fixed);
  stream.precision(precision);
  stream << value;
  return stream.str();
}

}  // namespace

WristCameraMount::WristCameraMount(FramedTransform mount)
: mount_(std::move(mount))
{
}

Result<WristCameraMount> WristCameraMount::create(FramedTransform tool0_from_optical)
{
  if (tool0_from_optical.source_frame() != kWristCameraOpticalFrame) {
    return failure<WristCameraMount>(
      PerceptionErrorCode::FrameMismatch,
      "wrist camera mount must convert from " + std::string(kWristCameraOpticalFrame) +
      ", not from " + tool0_from_optical.source_frame());
  }
  if (tool0_from_optical.target_frame() != kToolFrame) {
    return failure<WristCameraMount>(
      PerceptionErrorCode::FrameMismatch,
      "wrist camera mount must convert into " + std::string(kToolFrame) + ", not into " +
      tool0_from_optical.target_frame());
  }
  return Result<WristCameraMount>::success(WristCameraMount(std::move(tool0_from_optical)));
}

Result<Tool0ViewpointGoal> WristCameraMount::tool0_goal_for(const CameraViewpoint & viewpoint) const
{
  if (viewpoint.pose.frame_id.empty()) {
    return failure<Tool0ViewpointGoal>(
      PerceptionErrorCode::InvalidArgument, "viewpoint must name the frame it is expressed in");
  }
  if (!viewpoint.pose.pose.matrix().allFinite()) {
    return failure<Tool0ViewpointGoal>(
      PerceptionErrorCode::InvalidArgument, "viewpoint pose is not finite");
  }
  Tool0ViewpointGoal goal;
  goal.label = viewpoint.label;
  goal.pose.frame_id = viewpoint.pose.frame_id;
  // reference <- tool0 = (reference <- optical) * (optical <- tool0).
  goal.pose.pose = viewpoint.pose.pose * mount_.transform().inverse();
  // The uncertainty of a viewpoint is a property of whatever proposed it, and a rigid mount does
  // not change it. It travels unrotated because a survey viewpoint is a command, not a
  // measurement: nothing downstream fuses it, and rotating a covariance that means nothing here
  // would only invent precision.
  goal.pose.covariance = viewpoint.pose.covariance;
  return Result<Tool0ViewpointGoal>::success(std::move(goal));
}

Result<CameraViewpoint> WristCameraMount::viewpoint_for(const Tool0ViewpointGoal & goal) const
{
  if (goal.pose.frame_id.empty()) {
    return failure<CameraViewpoint>(
      PerceptionErrorCode::InvalidArgument, "tool goal must name the frame it is expressed in");
  }
  if (!goal.pose.pose.matrix().allFinite()) {
    return failure<CameraViewpoint>(
      PerceptionErrorCode::InvalidArgument, "tool goal pose is not finite");
  }
  CameraViewpoint viewpoint;
  viewpoint.label = goal.label;
  viewpoint.pose.frame_id = goal.pose.frame_id;
  // reference <- optical = (reference <- tool0) * (tool0 <- optical).
  viewpoint.pose.pose = goal.pose.pose * mount_.transform();
  viewpoint.pose.covariance = goal.pose.covariance;
  return Result<CameraViewpoint>::success(std::move(viewpoint));
}

bool SensorFrustum::valid() const noexcept
{
  return width_px > 0U && height_px > 0U && horizontal_fov_rad > 0.0 &&
         horizontal_fov_rad < M_PI && near_clip_m >= 0.0 && far_clip_m > near_clip_m;
}

double SensorFrustum::focal_length_px() const noexcept
{
  return (static_cast<double>(width_px) / 2.0) / std::tan(horizontal_fov_rad / 2.0);
}

double SensorFrustum::horizontal_half_angle_rad() const noexcept
{
  return horizontal_fov_rad / 2.0;
}

double SensorFrustum::vertical_half_angle_rad() const noexcept
{
  return std::atan((static_cast<double>(height_px) / 2.0) / focal_length_px());
}

double SensorFrustum::ground_sample_m(double range_m) const noexcept
{
  return range_m / focal_length_px();
}

ProjectedPoint project_into_image(
  const SensorFrustum & frustum, const Eigen::Vector3d & point_in_optical_frame)
{
  ProjectedPoint projected;
  projected.range_m = point_in_optical_frame.z();
  projected.in_front = point_in_optical_frame.z() > 0.0;
  if (!projected.in_front) {
    return projected;
  }
  const double focal = frustum.focal_length_px();
  projected.column_px = focal * point_in_optical_frame.x() / point_in_optical_frame.z() +
    static_cast<double>(frustum.width_px) / 2.0;
  projected.row_px = focal * point_in_optical_frame.y() / point_in_optical_frame.z() +
    static_cast<double>(frustum.height_px) / 2.0;
  return projected;
}

std::vector<Eigen::Vector3d> box_corners(
  const Eigen::Vector3d & center, const Eigen::Vector3d & size)
{
  std::vector<Eigen::Vector3d> corners;
  corners.reserve(8U);
  const Eigen::Vector3d half = 0.5 * size;
  for (const double x : {-half.x(), half.x()}) {
    for (const double y : {-half.y(), half.y()}) {
      for (const double z : {-half.z(), half.z()}) {
        corners.emplace_back(center + Eigen::Vector3d(x, y, z));
      }
    }
  }
  return corners;
}

FramingResult frames_region(
  const SensorFrustum & frustum, const Eigen::Isometry3d & reference_from_optical,
  const std::vector<Eigen::Vector3d> & region_points, double required_margin_px)
{
  FramingResult result;
  if (!frustum.valid()) {
    result.detail = "sensor frustum is not a usable pinhole model";
    return result;
  }
  if (region_points.empty()) {
    result.detail = "no region points to frame";
    return result;
  }
  const Eigen::Isometry3d optical_from_reference = reference_from_optical.inverse();
  double smallest_margin = std::numeric_limits<double>::max();
  result.nearest_range_m = std::numeric_limits<double>::max();
  result.farthest_range_m = 0.0;
  for (const Eigen::Vector3d & point : region_points) {
    const Eigen::Vector3d in_optical = optical_from_reference * point;
    const ProjectedPoint projected = project_into_image(frustum, in_optical);
    if (!projected.in_front) {
      result.detail = "a region point is behind the camera at range " +
        format_number(projected.range_m) + " m";
      result.margin_px = -std::numeric_limits<double>::infinity();
      return result;
    }
    result.nearest_range_m = std::min(result.nearest_range_m, projected.range_m);
    result.farthest_range_m = std::max(result.farthest_range_m, projected.range_m);
    const double margin = std::min(
      {projected.column_px, static_cast<double>(frustum.width_px) - projected.column_px,
        projected.row_px, static_cast<double>(frustum.height_px) - projected.row_px});
    if (margin < smallest_margin) {
      smallest_margin = margin;
      if (margin < required_margin_px) {
        result.detail = "a region point projects to (" + format_number(projected.column_px, 1) +
          ", " + format_number(projected.row_px, 1) + ") px, " + format_number(margin, 1) +
          " px from the image edge against a required " + format_number(required_margin_px, 1) +
          " px";
      }
    }
  }
  result.margin_px = smallest_margin;
  if (result.nearest_range_m < frustum.near_clip_m) {
    result.detail = "a region point at " + format_number(result.nearest_range_m) +
      " m is inside the " + format_number(frustum.near_clip_m) + " m near clip";
    return result;
  }
  if (result.farthest_range_m > frustum.far_clip_m) {
    result.detail = "a region point at " + format_number(result.farthest_range_m) +
      " m is beyond the " + format_number(frustum.far_clip_m) + " m far clip";
    return result;
  }
  result.framed = smallest_margin >= required_margin_px;
  if (result.framed) {
    result.detail.clear();
  }
  return result;
}

Result<Eigen::Isometry3d> look_at_pose(
  const Eigen::Vector3d & eye, const Eigen::Vector3d & target,
  const Eigen::Vector3d & preferred_right, const Eigen::Vector3d & up)
{
  const Eigen::Vector3d boresight = target - eye;
  if (boresight.norm() < 1.0e-9) {
    return failure<Eigen::Isometry3d>(
      PerceptionErrorCode::InvalidArgument, "look-at target coincides with the camera origin");
  }
  if (preferred_right.norm() < 1.0e-9) {
    return failure<Eigen::Isometry3d>(
      PerceptionErrorCode::InvalidArgument, "preferred image-right axis is degenerate");
  }
  if (up.norm() < 1.0e-9) {
    return failure<Eigen::Isometry3d>(
      PerceptionErrorCode::InvalidArgument, "up axis is degenerate");
  }
  const Eigen::Vector3d axis_z = boresight.normalized();
  const Eigen::Vector3d up_axis = up.normalized();
  // Card 098 / specs/camera-viewpoint-orientation.md: whenever `up` decides the roll, the frame
  // is UPRIGHT — image-up is the component of `up` off the boresight, with image-up . up > 0 —
  // and preferred_right does not choose the roll. The old contract took the hint's SIGN as the
  // roll authority, and +X_shelf against a -Y_shelf boresight (tray stations, confirmation)
  // is the 180-degree-rolled solution: those views shipped upside down. The lane stations'
  // +Y boresight agreed with the hint and is byte-identical here.
  const Eigen::Vector3d up_perp = up_axis - up_axis.dot(axis_z) * axis_z;
  Eigen::Vector3d axis_x;
  if (up_perp.norm() >= 1.0e-6) {
    const Eigen::Vector3d image_down = -up_perp.normalized();
    axis_x = image_down.cross(axis_z);
  } else {
    // Straight up or straight down: every roll is upright and `up` says nothing, so the hint's
    // boresight-orthogonal component pins one — and when that too is parallel to the boresight
    // the roll is undetermined, reported rather than guessed into a plausible wrong image.
    axis_x = preferred_right.normalized();
    axis_x -= axis_x.dot(axis_z) * axis_z;
    if (axis_x.norm() < 1.0e-6) {
      return failure<Eigen::Isometry3d>(
        PerceptionErrorCode::InvalidArgument,
        "preferred image-right axis is parallel to the boresight, so the camera roll is undefined");
    }
    axis_x.normalize();
  }
  const Eigen::Vector3d axis_y = axis_z.cross(axis_x);
  Eigen::Matrix3d rotation;
  rotation.col(0) = axis_x;
  rotation.col(1) = axis_y;
  rotation.col(2) = axis_z;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = eye;
  return Result<Eigen::Isometry3d>::success(pose);
}

}  // namespace restocker_perception
