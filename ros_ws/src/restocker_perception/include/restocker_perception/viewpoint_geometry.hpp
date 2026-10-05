// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

#include "restocker_perception/frame_geometry.hpp"

namespace restocker_perception
{

// The two frames this file converts between.
inline constexpr const char * kWristCameraOpticalFrame = "wrist_camera_optical_frame";
inline constexpr const char * kToolFrame = "tool0";

// A pose of the camera's own optical frame, expressed in some reference frame.
//
// FramedPose says which frame a pose is expressed in, not which body's pose it is. A viewpoint and
// the tool goal derived from it are both expressed in the planning frame, so frame_id alone does
// not separate them. The two wrappers are distinct types so a viewpoint cannot be handed to
// something that wants a tool goal.
struct CameraViewpoint
{
  // pose is reference_frame <- wrist_camera_optical_frame. Its frame_id is the reference frame.
  FramedPose pose;
  // Names this viewpoint in logs and failure details.
  std::string label;
};

// A pose of tool0, expressed in the same reference frame, that puts the optical frame at a
// requested viewpoint. This is what a MotionGoal takes.
struct Tool0ViewpointGoal
{
  // pose is reference_frame <- tool0. Its frame_id is the reference frame.
  FramedPose pose;
  std::string label;
};

// The fixed tool0 <- wrist_camera_optical_frame transform, and the two conversions through it.
//
// The camera is bolted 0.110 m off the grasp approach axis and its optical frame follows REP 103
// (+X right, +Y down, +Z along the boresight), while tool0 follows the link convention. Mixing the
// two yields a plausible pose that aims the camera elsewhere, so the transform is never composed
// by hand at a call site: it is validated once and applied here.
class WristCameraMount
{
public:
  // Fails unless the transform is tool0 <- wrist_camera_optical_frame, in that direction. The
  // frames are checked because the inverse is an equally well-formed rigid transform and applying
  // it would put the camera on the wrong side of the wrist.
  [[nodiscard]] static Result<WristCameraMount> create(FramedTransform tool0_from_optical);

  [[nodiscard]] const FramedTransform & tool0_from_optical() const noexcept {return mount_;}

  // reference <- tool0 = (reference <- optical) * (optical <- tool0).
  //
  // The mount composes on the right, because it relates two bodies rather than converting a
  // measurement between frames. FramedTransform::apply composes on the left and is the wrong
  // operation here.
  [[nodiscard]] Result<Tool0ViewpointGoal> tool0_goal_for(const CameraViewpoint & viewpoint) const;

  // The inverse conversion: where the optical frame ends up when tool0 is at a given pose. It
  // turns an achieved tool0 pose or a live TF reading back into an achieved viewpoint; a round
  // trip through both is the cheapest check of the arithmetic.
  [[nodiscard]] Result<CameraViewpoint> viewpoint_for(const Tool0ViewpointGoal & goal) const;

private:
  explicit WristCameraMount(FramedTransform mount);

  FramedTransform mount_;
};

// A pinhole frustum, in the numbers the description declares.
//
// No field defaults: each is supplied by whatever read the description (the expanded URDF for a
// static check, CameraInfo for a live one). A framing computation with a hardcoded resolution
// would keep passing after the sensor changed.
struct SensorFrustum
{
  std::uint32_t width_px{0U};
  std::uint32_t height_px{0U};
  double horizontal_fov_rad{0.0};
  double near_clip_m{0.0};
  double far_clip_m{0.0};

  [[nodiscard]] bool valid() const noexcept;
  // The single focal length a square-pixel pinhole model has, in pixels.
  [[nodiscard]] double focal_length_px() const noexcept;
  [[nodiscard]] double horizontal_half_angle_rad() const noexcept;
  // Derived from the horizontal field and the aspect ratio, never declared separately.
  [[nodiscard]] double vertical_half_angle_rad() const noexcept;
  // Metres per pixel at a given range along the boresight.
  [[nodiscard]] double ground_sample_m(double range_m) const noexcept;
};

// Where a point in the optical frame lands in the image, and whether it is in front of the lens.
struct ProjectedPoint
{
  double column_px{0.0};
  double row_px{0.0};
  double range_m{0.0};
  bool in_front{false};
};

[[nodiscard]] ProjectedPoint project_into_image(
  const SensorFrustum & frustum, const Eigen::Vector3d & point_in_optical_frame);

// How a region of interest sits inside the frame: every corner in front of the lens, inside the
// near and far clip, and inside the image with a stated margin.
struct FramingResult
{
  bool framed{false};
  // Smallest distance, in pixels, from any projected corner to the nearest image edge. Negative
  // when a corner is outside the image, by the amount of overflow.
  double margin_px{0.0};
  double nearest_range_m{0.0};
  double farthest_range_m{0.0};
  // Empty when framed; otherwise says which condition failed and by how much.
  std::string detail;
};

// The eight corners of an axis-aligned box, in the frame the box is stated in.
[[nodiscard]] std::vector<Eigen::Vector3d> box_corners(
  const Eigen::Vector3d & center, const Eigen::Vector3d & size);

// Test a region against a frustum placed at a known pose.
//
// reference_from_optical is the camera's pose (the value a CameraViewpoint carries) and the region
// points are in that same reference frame. Required margin is in pixels, measured against every
// image edge.
[[nodiscard]] FramingResult frames_region(
  const SensorFrustum & frustum, const Eigen::Isometry3d & reference_from_optical,
  const std::vector<Eigen::Vector3d> & region_points, double required_margin_px);

// Build the orientation of a camera that looks from `eye` at `target`, upright against `up`.
//
// Optical +Z is the boresight and optical +Y is image-down, per REP 103 (Card 098; contract in
// Backstage specs/camera-viewpoint-orientation.md):
//
// * Whenever the boresight is not parallel to `up`, the roll is the UPRIGHT one: image-up is
//   the component of `up` orthogonal to the boresight (image-up . up > 0) and +X completes the
//   right-handed frame. `preferred_right` does NOT choose the roll in this case — taking its
//   sign as the roll authority is what shipped the tray and confirmation views upside down.
// * Looking straight up or straight down along `up`, every roll is upright and the hint's
//   boresight-orthogonal component pins one; a hint parallel to the boresight there leaves the
//   roll undetermined and is refused rather than guessed into a plausible wrong image.
//
// Fails when the boresight is degenerate, when `up` is degenerate, or when the roll is
// undetermined as above.
[[nodiscard]] Result<Eigen::Isometry3d> look_at_pose(
  const Eigen::Vector3d & eye, const Eigen::Vector3d & target,
  const Eigen::Vector3d & preferred_right, const Eigen::Vector3d & up);

}  // namespace restocker_perception
