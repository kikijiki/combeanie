// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <numbers>

#include <limits>

#include "restocker_perception/frame_geometry.hpp"

namespace
{

using restocker_perception::FramedPose;
using restocker_perception::FramedTransform;
using restocker_perception::PerceptionErrorCode;
using restocker_perception::PoseCovariance;
using restocker_perception::compare_framed_poses;
using restocker_perception::declares_axis_estimate;
using restocker_perception::kUnmeasuredRotationVariance;

[[nodiscard]] FramedTransform camera_to_world()
{
  // Quarter turn about Z plus a translation: a transform that actually mixes the axes, so a
  // covariance that is not rotated shows up as a difference rather than staying accidentally equal.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
    Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  auto created = FramedTransform::create("camera_optical", "world", transform);
  EXPECT_TRUE(created);
  return created.value();
}

TEST(FramedTransform, RejectsEmptyFramesAndImproperRotations)
{
  EXPECT_FALSE(FramedTransform::create("", "world", Eigen::Isometry3d::Identity()));
  EXPECT_FALSE(FramedTransform::create("camera", "", Eigen::Isometry3d::Identity()));

  Eigen::Isometry3d skewed = Eigen::Isometry3d::Identity();
  skewed.linear() = Eigen::Matrix3d::Identity() * 2.0;
  EXPECT_FALSE(FramedTransform::create("camera", "world", skewed));
}

TEST(FramedTransform, RejectsAPoseThatIsNotInTheSourceFrame)
{
  const auto transform = camera_to_world();
  FramedPose already_world;
  already_world.frame_id = "world";

  const auto applied = transform.apply(already_world);
  ASSERT_FALSE(applied);
  EXPECT_EQ(applied.error().code, PerceptionErrorCode::FrameMismatch);
}

TEST(FramedTransform, ConvertsPoseAndCovarianceTogetherAndRestampsTheFrame)
{
  const auto transform = camera_to_world();
  FramedPose measured;
  measured.frame_id = "camera_optical";
  measured.pose.translation() = Eigen::Vector3d(1.0, 0.0, 0.0);
  measured.covariance = PoseCovariance::Zero();
  measured.covariance(0, 0) = 4.0;
  measured.covariance(1, 1) = 1.0;
  measured.covariance(3, 3) = 4.0;
  measured.covariance(4, 4) = 1.0;

  const auto applied = transform.apply(measured);
  ASSERT_TRUE(applied);
  EXPECT_EQ(applied.value().frame_id, "world");

  // A quarter turn about Z maps camera +X onto world +Y, then the translation shifts it.
  EXPECT_NEAR(applied.value().pose.translation().x(), 1.0, 1.0e-9);
  EXPECT_NEAR(applied.value().pose.translation().y(), 3.0, 1.0e-9);
  EXPECT_NEAR(applied.value().pose.translation().z(), 3.0, 1.0e-9);

  // The same rotation must have been applied to both 3x3 blocks of the uncertainty, so the
  // variances swap between the X and Y axes instead of staying in the frame the pose just left.
  EXPECT_NEAR(applied.value().covariance(0, 0), 1.0, 1.0e-9);
  EXPECT_NEAR(applied.value().covariance(1, 1), 4.0, 1.0e-9);
  EXPECT_NEAR(applied.value().covariance(3, 3), 1.0, 1.0e-9);
  EXPECT_NEAR(applied.value().covariance(4, 4), 4.0, 1.0e-9);
}

TEST(PoseError, RefusesToCompareAcrossFrames)
{
  FramedPose estimate;
  estimate.frame_id = "camera_optical";
  FramedPose reference;
  reference.frame_id = "world";

  const auto compared = compare_framed_poses(estimate, reference);
  ASSERT_FALSE(compared);
  EXPECT_EQ(compared.error().code, PerceptionErrorCode::FrameMismatch);
}

TEST(PoseError, ReportsZeroErrorForIdenticalPosesAndMeasuresOffsets)
{
  FramedPose reference;
  reference.frame_id = "world";
  reference.pose.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);

  const auto identical = compare_framed_poses(reference, reference);
  ASSERT_TRUE(identical);
  EXPECT_NEAR(identical.value().translation_error_m, 0.0, 1.0e-12);
  EXPECT_NEAR(identical.value().rotation_error_rad, 0.0, 1.0e-9);

  FramedPose estimate = reference;
  estimate.pose.translation() += Eigen::Vector3d(0.03, 0.04, 0.0);
  estimate.pose.linear() =
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto offset = compare_framed_poses(estimate, reference);
  ASSERT_TRUE(offset);
  EXPECT_NEAR(offset.value().translation_error_m, 0.05, 1.0e-9);
  EXPECT_NEAR(offset.value().rotation_error_rad, 0.25, 1.0e-9);
}

TEST(PoseError, SeparatesTheSymmetryAxisFromTheYawThatCannotBeMeasured)
{
  FramedPose reference;
  reference.frame_id = "world";

  // A quarter-radian of yaw is a quarter-radian of full-rotation error and no axis error at all.
  // For this catalogue, which is cylinders, only the second number describes something an
  // estimator could have got wrong.
  FramedPose yawed = reference;
  yawed.pose.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto spun = compare_framed_poses(yawed, reference);
  ASSERT_TRUE(spun);
  EXPECT_NEAR(spun.value().rotation_error_rad, 0.25, 1.0e-9);
  EXPECT_NEAR(spun.value().axis_error_rad, 0.0, 1.0e-9);

  FramedPose leaning = reference;
  leaning.pose.linear() = Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto tilted = compare_framed_poses(leaning, reference);
  ASSERT_TRUE(tilted);
  EXPECT_NEAR(tilted.value().axis_error_rad, 0.25, 1.0e-9);

  // A cylinder's axis has no sign, so end-over-end is the same axis and folds to zero rather than
  // to pi. Half a turn about a horizontal axis is the largest error the fold has to absorb.
  FramedPose inverted = reference;
  inverted.pose.linear() =
    Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto upended = compare_framed_poses(inverted, reference);
  ASSERT_TRUE(upended);
  EXPECT_NEAR(upended.value().rotation_error_rad, std::numbers::pi, 1.0e-6);
  EXPECT_NEAR(upended.value().axis_error_rad, 0.0, 1.0e-6);

  // And a container lying down is the quarter turn it should be.
  FramedPose lying = reference;
  lying.pose.linear() =
    Eigen::AngleAxisd(std::numbers::pi / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const auto fallen = compare_framed_poses(lying, reference);
  ASSERT_TRUE(fallen);
  EXPECT_NEAR(fallen.value().axis_error_rad, std::numbers::pi / 2.0, 1.0e-9);
}

TEST(PoseError, ReadsACovarianceAsADeclarationOfWhetherTheAxisWasMeasuredAtAll)
{
  // Zero is a claim of perfect knowledge, and is a declaration that the axis was estimated.
  EXPECT_TRUE(declares_axis_estimate(PoseCovariance::Zero()));

  // The variance of a uniform distribution over a full turn is not a large uncertainty; it is the
  // absence of one, and it must not be read as an estimate however loose.
  PoseCovariance blind = PoseCovariance::Zero();
  blind(3, 3) = kUnmeasuredRotationVariance;
  blind(4, 4) = kUnmeasuredRotationVariance;
  blind(5, 5) = kUnmeasuredRotationVariance;
  EXPECT_FALSE(declares_axis_estimate(blind));

  // Both axes have to be claimed: the axis direction has two degrees of freedom, and one of them
  // unmeasured leaves the direction unmeasured. Yaw alone unmeasured does not: that is true of
  // every cylinder estimator.
  PoseCovariance half = PoseCovariance::Zero();
  half(4, 4) = kUnmeasuredRotationVariance;
  EXPECT_FALSE(declares_axis_estimate(half));

  PoseCovariance cylinder = PoseCovariance::Zero();
  cylinder(3, 3) = 0.01;
  cylinder(4, 4) = 0.01;
  cylinder(5, 5) = kUnmeasuredRotationVariance;
  EXPECT_TRUE(declares_axis_estimate(cylinder));

  PoseCovariance broken = PoseCovariance::Zero();
  broken(3, 3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(declares_axis_estimate(broken));
}

TEST(PoseError, TreatsAntipodalQuaternionsAsTheSameOrientation)
{
  FramedPose reference;
  reference.frame_id = "world";
  reference.pose.linear() =
    Eigen::AngleAxisd(std::numbers::pi, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const FramedPose estimate = reference;
  const auto compared = compare_framed_poses(estimate, reference);
  ASSERT_TRUE(compared);
  EXPECT_NEAR(compared.value().rotation_error_rad, 0.0, 1.0e-6);
}

}  // namespace
