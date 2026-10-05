// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/frame_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace restocker_perception
{
namespace
{

template<typename Value>
[[nodiscard]] Result<Value> failure(PerceptionErrorCode code, std::string detail)
{
  return Result<Value>::failure(PerceptionError{code, std::move(detail)});
}

[[nodiscard]] bool finite_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite();
}

[[nodiscard]] bool orthonormal_rotation(const Eigen::Matrix3d & rotation)
{
  const Eigen::Matrix3d residual =
    rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  return residual.cwiseAbs().maxCoeff() <= 1.0e-6 && rotation.determinant() > 0.0;
}

}  // namespace

FramedTransform::FramedTransform(
  std::string source_frame, std::string target_frame, Eigen::Isometry3d transform)
: source_frame_(std::move(source_frame)),
  target_frame_(std::move(target_frame)),
  transform_(std::move(transform))
{
}

Result<FramedTransform> FramedTransform::create(
  std::string source_frame, std::string target_frame, const Eigen::Isometry3d & transform)
{
  if (source_frame.empty() || target_frame.empty()) {
    return failure<FramedTransform>(
      PerceptionErrorCode::InvalidArgument, "transform frames must not be empty");
  }
  if (!finite_transform(transform)) {
    return failure<FramedTransform>(
      PerceptionErrorCode::InvalidArgument, "transform contains a non-finite value");
  }
  if (!orthonormal_rotation(transform.rotation())) {
    return failure<FramedTransform>(
      PerceptionErrorCode::InvalidArgument, "transform rotation is not a proper rotation");
  }
  return Result<FramedTransform>::success(
    FramedTransform(std::move(source_frame), std::move(target_frame), transform));
}

Result<FramedPose> FramedTransform::apply(const FramedPose & pose) const
{
  if (pose.frame_id != source_frame_) {
    return failure<FramedPose>(
      PerceptionErrorCode::FrameMismatch,
      "pose frame '" + pose.frame_id + "' cannot be transformed by a transform whose source frame "
      "is '" + source_frame_ + "'");
  }
  if (!finite_transform(pose.pose) || !pose.covariance.allFinite()) {
    return failure<FramedPose>(
      PerceptionErrorCode::InvalidArgument, "pose or covariance contains a non-finite value");
  }

  // Translation and rotation blocks share the same rotation, so the 6x6 uncertainty rotates by
  // blockdiag(R, R). Converting the pose without the covariance would leave uncertainty expressed
  // in the frame the pose just left.
  const Eigen::Matrix3d rotation = transform_.rotation();
  Eigen::Matrix<double, 6, 6> block = Eigen::Matrix<double, 6, 6>::Zero();
  block.topLeftCorner<3, 3>() = rotation;
  block.bottomRightCorner<3, 3>() = rotation;

  FramedPose converted;
  converted.frame_id = target_frame_;
  converted.pose = transform_ * pose.pose;
  converted.covariance = block * pose.covariance * block.transpose();
  return Result<FramedPose>::success(std::move(converted));
}

Result<PoseErrorSample> compare_framed_poses(
  const FramedPose & estimate, const FramedPose & reference)
{
  if (estimate.frame_id.empty() || reference.frame_id.empty()) {
    return failure<PoseErrorSample>(
      PerceptionErrorCode::InvalidArgument, "compared poses must both declare a frame");
  }
  if (estimate.frame_id != reference.frame_id) {
    return failure<PoseErrorSample>(
      PerceptionErrorCode::FrameMismatch,
      "cannot measure pose error between frame '" + estimate.frame_id + "' and frame '" +
      reference.frame_id + "'");
  }
  if (!finite_transform(estimate.pose) || !finite_transform(reference.pose)) {
    return failure<PoseErrorSample>(
      PerceptionErrorCode::InvalidArgument, "compared poses must be finite");
  }

  PoseErrorSample sample;
  sample.translation_error_m =
    (estimate.pose.translation() - reference.pose.translation()).norm();

  const Eigen::Quaterniond estimated(estimate.pose.rotation());
  const Eigen::Quaterniond referenced(reference.pose.rotation());
  // Quaternions double-cover SO(3); take the absolute dot product so q and -q measure the same
  // orientation rather than reporting a spurious pi-radian error.
  const double alignment = std::clamp(
    std::abs(estimated.normalized().dot(referenced.normalized())),
    0.0, 1.0);
  sample.rotation_error_rad = 2.0 * std::acos(alignment);

  // A cylinder's symmetry axis has no sign: a can rotated by pi about its own X is the same can
  // standing the same way. The absolute value folds the angle into [0, pi/2] so the ambiguity is
  // not reported as error.
  const double axis_alignment = std::clamp(
    std::abs(
      (estimate.pose.rotation() * Eigen::Vector3d::UnitZ())
      .dot(reference.pose.rotation() * Eigen::Vector3d::UnitZ())),
    0.0, 1.0);
  sample.axis_error_rad = std::acos(axis_alignment);
  return Result<PoseErrorSample>::success(sample);
}

bool declares_axis_estimate(const PoseCovariance & covariance)
{
  for (const Eigen::Index axis : {Eigen::Index{3}, Eigen::Index{4}}) {
    const double variance = covariance(axis, axis);
    if (!std::isfinite(variance) || variance >= kUnmeasuredRotationVariance) {
      return false;
    }
  }
  return true;
}

}  // namespace restocker_perception
