// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "restocker_perception/viewpoint_geometry.hpp"

namespace restocker_perception
{
namespace
{

// The mount as the description declares it:
// tool0 -> wrist_camera_link is xyz (0.11, 0, 0.07) rpy (0, -pi/2, 0), and
// wrist_camera_link -> wrist_camera_optical_frame is xyz (0.026, 0, 0) rpy (-pi/2, 0, -pi/2).
// The optical origin sits at tool0 (0.11, 0, 0.096) with its boresight along tool0 +Z. The runtime
// test checks the robot's published transform against this.
[[nodiscard]] Eigen::Isometry3d shipped_tool0_from_optical()
{
  Eigen::Isometry3d mount_joint = Eigen::Isometry3d::Identity();
  mount_joint.translation() = Eigen::Vector3d(0.11, 0.0, 0.07);
  mount_joint.linear() =
    Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitY()).toRotationMatrix();
  Eigen::Isometry3d optical_joint = Eigen::Isometry3d::Identity();
  optical_joint.translation() = Eigen::Vector3d(0.026, 0.0, 0.0);
  optical_joint.linear() =
    (Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX())).toRotationMatrix();
  return mount_joint * optical_joint;
}

[[nodiscard]] WristCameraMount shipped_mount()
{
  Result<FramedTransform> transform = FramedTransform::create(
    kWristCameraOpticalFrame, kToolFrame, shipped_tool0_from_optical());
  EXPECT_TRUE(transform.has_value());
  Result<WristCameraMount> mount = WristCameraMount::create(transform.value());
  EXPECT_TRUE(mount.has_value());
  return mount.value();
}

[[nodiscard]] CameraViewpoint viewpoint_at(const Eigen::Isometry3d & pose)
{
  CameraViewpoint viewpoint;
  viewpoint.pose.frame_id = "world";
  viewpoint.pose.pose = pose;
  viewpoint.label = "test viewpoint";
  return viewpoint;
}

TEST(WristCameraMount, RejectsATransformStatedInTheWrongDirection)
{
  Result<FramedTransform> inverted = FramedTransform::create(
    kToolFrame, kWristCameraOpticalFrame, shipped_tool0_from_optical().inverse());
  ASSERT_TRUE(inverted.has_value());
  const Result<WristCameraMount> mount = WristCameraMount::create(inverted.value());
  ASSERT_FALSE(mount.has_value());
  EXPECT_EQ(mount.error().code, PerceptionErrorCode::FrameMismatch);
}

TEST(WristCameraMount, RejectsATransformBetweenUnrelatedFrames)
{
  Result<FramedTransform> unrelated = FramedTransform::create(
    "overhead_camera_optical_frame", kToolFrame, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(unrelated.has_value());
  EXPECT_FALSE(WristCameraMount::create(unrelated.value()).has_value());
}

TEST(WristCameraMount, DescribesTheDeclaredOpticalOriginAndBoresight)
{
  const WristCameraMount mount = shipped_mount();
  const Eigen::Isometry3d & transform = mount.tool0_from_optical().transform();
  EXPECT_NEAR(transform.translation().x(), 0.11, 1.0e-9);
  EXPECT_NEAR(transform.translation().y(), 0.0, 1.0e-9);
  EXPECT_NEAR(transform.translation().z(), 0.096, 1.0e-9);
  // The boresight (optical +Z) is along tool0 +Z, parallel to the grasp approach axis.
  const Eigen::Vector3d boresight_in_tool0 = transform.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(boresight_in_tool0.z(), 1.0, 1.0e-9);
  EXPECT_NEAR(boresight_in_tool0.norm(), 1.0, 1.0e-9);
  // The camera is 0.11 m off the approach axis.
  EXPECT_NEAR(
    Eigen::Vector2d(transform.translation().x(), transform.translation().y()).norm(), 0.11,
    1.0e-9);
}

TEST(WristCameraMount, RoundTripsAViewpointThroughTheToolGoalAndBack)
{
  const WristCameraMount mount = shipped_mount();
  // A generic pose with no axis alignment, so a transposed rotation or wrong-side composition
  // cannot pass.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-0.37, 1.24, 1.61);
  pose.linear() = (Eigen::AngleAxisd(0.71, Eigen::Vector3d(0.3, -0.5, 0.81).normalized()) *
    Eigen::AngleAxisd(-1.13, Eigen::Vector3d::UnitY())).toRotationMatrix();
  const CameraViewpoint requested = viewpoint_at(pose);

  const Result<Tool0ViewpointGoal> goal = mount.tool0_goal_for(requested);
  ASSERT_TRUE(goal.has_value());
  EXPECT_EQ(goal.value().pose.frame_id, "world");
  EXPECT_EQ(goal.value().label, requested.label);

  const Result<CameraViewpoint> recovered = mount.viewpoint_for(goal.value());
  ASSERT_TRUE(recovered.has_value());
  EXPECT_LT(
    (recovered.value().pose.pose.translation() - pose.translation()).norm(), 1.0e-12);
  EXPECT_LT(
    (recovered.value().pose.pose.linear() - pose.linear()).cwiseAbs().maxCoeff(), 1.0e-12);

  // The tool goal differs from the viewpoint by the mount offset; a no-op conversion would pass
  // the round trip but fail here.
  EXPECT_NEAR(
    (goal.value().pose.pose.translation() - pose.translation()).norm(),
    shipped_tool0_from_optical().translation().norm(), 1.0e-12);
}

TEST(WristCameraMount, PutsTheToolBehindAndAboveACameraLookingStraightDown)
{
  // Sign test. The camera looks along world -Z with image-right along world +X (the tray survey
  // attitude), so tool0 sits 0.096 m above the lens and 0.11 m to its +Y side. Wrong-side
  // composition or a transposed rotation would move it to another quadrant with equal magnitudes.
  const WristCameraMount mount = shipped_mount();
  Eigen::Matrix3d rotation;
  rotation.col(0) = Eigen::Vector3d::UnitX();
  rotation.col(1) = -Eigen::Vector3d::UnitY();
  rotation.col(2) = -Eigen::Vector3d::UnitZ();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = rotation;
  pose.translation() = Eigen::Vector3d(0.4, -0.8, 1.47);

  const Result<Tool0ViewpointGoal> goal = mount.tool0_goal_for(viewpoint_at(pose));
  ASSERT_TRUE(goal.has_value());
  const Eigen::Vector3d offset = goal.value().pose.pose.translation() - pose.translation();
  EXPECT_NEAR(offset.x(), 0.0, 1.0e-9);
  EXPECT_NEAR(offset.y(), 0.11, 1.0e-9);
  EXPECT_NEAR(offset.z(), 0.096, 1.0e-9);
  // tool0 +Z (the approach axis) points down with the boresight.
  const Eigen::Vector3d approach = goal.value().pose.pose.linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(approach.z(), -1.0, 1.0e-9);
}

TEST(WristCameraMount, RefusesAViewpointWithNoFrame)
{
  const WristCameraMount mount = shipped_mount();
  CameraViewpoint viewpoint = viewpoint_at(Eigen::Isometry3d::Identity());
  viewpoint.pose.frame_id.clear();
  const Result<Tool0ViewpointGoal> goal = mount.tool0_goal_for(viewpoint);
  ASSERT_FALSE(goal.has_value());
  EXPECT_EQ(goal.error().code, PerceptionErrorCode::InvalidArgument);
}

TEST(SensorFrustum, DerivesTheVerticalFieldFromTheAspectRatioAndNotSeparately)
{
  // The wrist sensor's shipped numbers. 1280x720 at horizontal_fov 1.48 has the same vertical
  // half-angle as 640x480 at 1.20, so the vertical field follows from the aspect ratio.
  const SensorFrustum wide{1280U, 720U, 1.48, 0.05, 2.5};
  const SensorFrustum legacy{640U, 480U, 1.20, 0.05, 2.5};
  ASSERT_TRUE(wide.valid());
  ASSERT_TRUE(legacy.valid());
  EXPECT_NEAR(wide.vertical_half_angle_rad(), legacy.vertical_half_angle_rad(), 5.0e-4);
  EXPECT_NEAR(wide.vertical_half_angle_rad(), 0.4739, 1.0e-3);
  EXPECT_NEAR(wide.focal_length_px(), 700.92, 0.02);
  // Half the range gives half the millimetres per pixel.
  EXPECT_NEAR(wide.ground_sample_m(1.0) / wide.ground_sample_m(0.5), 2.0, 1.0e-9);
}

TEST(SensorFrustum, RejectsAModelThatIsNotAUsablePinhole)
{
  EXPECT_FALSE((SensorFrustum{0U, 720U, 1.48, 0.05, 2.5}).valid());
  EXPECT_FALSE((SensorFrustum{1280U, 720U, 0.0, 0.05, 2.5}).valid());
  EXPECT_FALSE((SensorFrustum{1280U, 720U, 1.48, 2.5, 0.05}).valid());
}

TEST(LookAtPose, BuildsARepOpticalFrameWithBoresightOnPlusZ)
{
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  const Eigen::Matrix3d rotation = pose.value().linear();
  // +Z is the boresight, +X is image-right, +Y is image-down. Looking down with image-right on
  // world +X puts image-down on world -Y.
  EXPECT_LT((rotation.col(2) - Eigen::Vector3d(0.0, 0.0, -1.0)).norm(), 1.0e-12);
  EXPECT_LT((rotation.col(0) - Eigen::Vector3d(1.0, 0.0, 0.0)).norm(), 1.0e-12);
  EXPECT_LT((rotation.col(1) - Eigen::Vector3d(0.0, -1.0, 0.0)).norm(), 1.0e-12);
  EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-12);
}

TEST(LookAtPose, RefusesARollItCannotDetermine)
{
  // Image-right parallel to the boresight leaves the roll undetermined.
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
  ASSERT_FALSE(pose.has_value());
  EXPECT_EQ(pose.error().code, PerceptionErrorCode::InvalidArgument);
}

TEST(LookAtPose, RefusesADegenerateBoresight)
{
  EXPECT_FALSE(
    look_at_pose(
      Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d(1.0, 1.0, 1.0), Eigen::Vector3d::UnitX(),
      Eigen::Vector3d::UnitZ()).has_value());
}

TEST(LookAtPose, RollsUprightAgainstTheUpAxisEvenWhenTheHintIsTheOtherSide)
{
  // The Card 098 contract (Backstage specs/camera-viewpoint-orientation.md): whenever the
  // boresight is not parallel to `up`, `up` decides the roll and preferred_right does not.
  // The tray stance — eye at +Y/+Z of the target, hint +X — is exactly the case the old
  // hint-sign rule got wrong (upside down, gravity -0.630): upright flips image-right to -X
  // while the boresight itself is untouched.
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.45, 0.555), Eigen::Vector3d::Zero(),
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  const Eigen::Matrix3d rotation = pose.value().linear();
  const Eigen::Vector3d expected_boresight =
    -Eigen::Vector3d(0.0, 0.45, 0.555).normalized();
  EXPECT_LT((rotation.col(2) - expected_boresight).norm(), 1.0e-12);
  EXPECT_GT(-(rotation.col(1).z()), 0.3) << "image-up must have a world-up component";
  EXPECT_LT((rotation.col(0) + Eigen::Vector3d::UnitX()).norm(), 1.0e-9)
    << "upright for this boresight puts image-right on -X";
  EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-12);
}

TEST(LookAtPose, RefusesADegenerateUpAxis)
{
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.5), Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector3d::UnitX(), Eigen::Vector3d::Zero());
  ASSERT_FALSE(pose.has_value());
  EXPECT_EQ(pose.error().code, PerceptionErrorCode::InvalidArgument);
}

TEST(FramesRegion, PlacesAPointOnTheBoresightAtTheImageCentre)
{
  const SensorFrustum frustum{1280U, 720U, 1.48, 0.05, 2.5};
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  const FramingResult result =
    frames_region(frustum, pose.value(), {Eigen::Vector3d::Zero()}, 1.0);
  EXPECT_TRUE(result.framed) << result.detail;
  EXPECT_NEAR(result.margin_px, 360.0, 1.0e-6);
  EXPECT_NEAR(result.nearest_range_m, 1.0, 1.0e-9);
}

TEST(FramesRegion, ReportsHowFarOutsideTheFrameAPointFalls)
{
  const SensorFrustum frustum{1280U, 720U, 1.48, 0.05, 2.5};
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  // 0.6 m off the boresight at 1.0 m range is 420.6 px, which overflows the 360 px half-height.
  const FramingResult result =
    frames_region(frustum, pose.value(), {Eigen::Vector3d(0.0, 0.6, 0.0)}, 0.0);
  EXPECT_FALSE(result.framed);
  EXPECT_LT(result.margin_px, 0.0);
  EXPECT_FALSE(result.detail.empty());
}

TEST(FramesRegion, RefusesAPointBehindTheCamera)
{
  const SensorFrustum frustum{1280U, 720U, 1.48, 0.05, 2.5};
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 1.0), Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  const FramingResult result =
    frames_region(frustum, pose.value(), {Eigen::Vector3d(0.0, 0.0, 1.4)}, 0.0);
  EXPECT_FALSE(result.framed);
  EXPECT_NE(result.detail.find("behind the camera"), std::string::npos);
}

TEST(FramesRegion, RefusesARegionInsideTheNearClip)
{
  const SensorFrustum frustum{1280U, 720U, 1.48, 0.20, 2.5};
  const Result<Eigen::Isometry3d> pose = look_at_pose(
    Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d::Zero(), Eigen::Vector3d::UnitX(),
    Eigen::Vector3d::UnitZ());
  ASSERT_TRUE(pose.has_value());
  const FramingResult result =
    frames_region(frustum, pose.value(), {Eigen::Vector3d::Zero()}, 0.0);
  EXPECT_FALSE(result.framed);
  EXPECT_NE(result.detail.find("near clip"), std::string::npos);
}

TEST(BoxCorners, ProducesTheEightCornersOfTheBox)
{
  const std::vector<Eigen::Vector3d> corners =
    box_corners(Eigen::Vector3d(1.0, 2.0, 3.0), Eigen::Vector3d(0.2, 0.4, 0.6));
  ASSERT_EQ(corners.size(), 8U);
  for (const Eigen::Vector3d & corner : corners) {
    EXPECT_NEAR(std::abs(corner.x() - 1.0), 0.1, 1.0e-12);
    EXPECT_NEAR(std::abs(corner.y() - 2.0), 0.2, 1.0e-12);
    EXPECT_NEAR(std::abs(corner.z() - 3.0), 0.3, 1.0e-12);
  }
}

}  // namespace
}  // namespace restocker_perception
