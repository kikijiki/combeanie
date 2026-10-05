// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

#include <restocker_world_state/world_state.hpp>

namespace restocker_perception
{

// The perception path reuses the world-state result and error taxonomy. FrameMismatch,
// StaleObservation and InvalidArgument mean here what they mean at the world-state admission
// boundary.
template<typename T>
using Result = restocker_world_state::Result<T>;
using PerceptionError = restocker_world_state::WorldStateError;
using PerceptionErrorCode = restocker_world_state::WorldStateErrorCode;
using restocker_world_state::PoseCovariance;

// A measured pose, its uncertainty, and the frame both are valid in, as one inseparable value.
//
// This package never accepts a bare Eigen::Isometry3d for a measured pose. The frame travels with
// the value, so every consumer must name the frame it expects and is told when it differs.
struct FramedPose
{
  std::string frame_id;
  Eigen::Isometry3d pose{Eigen::Isometry3d::Identity()};
  PoseCovariance covariance{PoseCovariance::Zero()};
};

// A rigid transform that knows both of its endpoints.
//
// Applying it requires the operand to declare its own frame, so applying a camera-to-world
// transform to something not in the camera frame fails with a diagnosis.
class FramedTransform
{
public:
  [[nodiscard]] static Result<FramedTransform> create(
    std::string source_frame, std::string target_frame, const Eigen::Isometry3d & transform);

  [[nodiscard]] const std::string & source_frame() const noexcept {return source_frame_;}
  [[nodiscard]] const std::string & target_frame() const noexcept {return target_frame_;}
  [[nodiscard]] const Eigen::Isometry3d & transform() const noexcept {return transform_;}

  // Fails with FrameMismatch unless pose.frame_id equals source_frame().
  //
  // Pose and covariance are always converted together and the result is stamped with
  // target_frame(), so a half-converted value (a rotated pose with unrotated uncertainty, or a
  // converted pose labelled with the source frame) cannot be produced.
  [[nodiscard]] Result<FramedPose> apply(const FramedPose & pose) const;

private:
  FramedTransform(std::string source_frame, std::string target_frame, Eigen::Isometry3d transform);

  std::string source_frame_;
  std::string target_frame_;
  Eigen::Isometry3d transform_;
};

// The variance of a uniform distribution over a full turn, and the value this package uses to say
// that a rotational degree of freedom was not measured at all.
//
// It is the absence of an uncertainty estimate, not a large one: a variance this size carries the
// information of a uniform prior, which is none. ColourDepthBackend uses it for yaw, which is
// unobservable on a surface of revolution; declares_axis_estimate below lets a consumer act on
// that declaration rather than infer it.
inline constexpr double kUnmeasuredRotationVariance = M_PI * M_PI / 3.0;

// Whether a pose's covariance claims to have measured the direction of the body's symmetry axis.
//
// The axis direction is fixed by the two rotational degrees of freedom about the body's X and Y,
// so both must be claimed. A pose that declares either unmeasured has an assumed orientation, and
// no error may be reported against it.
[[nodiscard]] bool declares_axis_estimate(const PoseCovariance & covariance);

// Error of an estimate against a reference pose.
struct PoseErrorSample
{
  double translation_error_m{0.0};
  // The geodesic angle between the two full orientations, in [0, pi].
  double rotation_error_rad{0.0};
  // The angle between the two bodies' local +Z, folded into [0, pi/2].
  //
  // The only rotational quantity estimable for this catalogue. Every catalogued product is a
  // cylinder with a symmetry axis, no yaw, and an unsigned axis, so rotation_error_rad above
  // charges an estimator for a yaw nothing can measure. Reported alongside it because
  // compare_framed_poses is also used on tool poses, for which the full geodesic is right.
  double axis_error_rad{0.0};
};

// Measures how far an estimate is from a reference.
//
// Fails with FrameMismatch unless both operands are expressed in the same frame.
//
// It measures the difference between two poses only. Whether the estimate's orientation was
// estimated at all is not visible here; see declares_axis_estimate and evaluate_pose_error.
[[nodiscard]] Result<PoseErrorSample> compare_framed_poses(
  const FramedPose & estimate, const FramedPose & reference);

}  // namespace restocker_perception
