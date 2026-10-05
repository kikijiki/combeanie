// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <span>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "restocker_perception/depth_obstacle_extraction.hpp"

namespace
{

using restocker_perception::DepthCameraIntrinsics;
using restocker_perception::DepthObstacleConfig;
using restocker_perception::DepthObstacleErrorCode;
using restocker_perception::ObstacleBoxFit;
using restocker_perception::SelfFilterCapsule;
using restocker_perception::extract_obstacle_boxes;

constexpr std::uint32_t kWidth = 160U;
constexpr std::uint32_t kHeight = 120U;

[[nodiscard]] DepthCameraIntrinsics intrinsics()
{
  DepthCameraIntrinsics value;
  value.fx = 160.0;
  value.fy = 160.0;
  value.cx = 0.5 * static_cast<double>(kWidth);
  value.cy = 0.5 * static_cast<double>(kHeight);
  value.width = kWidth;
  value.height = kHeight;
  return value;
}

[[nodiscard]] DepthObstacleConfig config()
{
  DepthObstacleConfig value;
  value.min_depth_m = 0.1;
  value.max_depth_m = 6.0;
  value.pixel_stride = 1U;
  value.volume_min = Eigen::Vector3d(-2.0, -2.0, 0.02);
  value.volume_max = Eigen::Vector3d(2.0, 2.0, 2.0);
  value.voxel_size_m = 0.02;
  value.min_cluster_voxels = 8U;
  value.min_box_extent_m = 0.04;
  value.max_box_extent_m = 1.2;
  value.max_boxes = 8U;
  return value;
}

// A camera two metres above the origin looking straight down: optical +z is world -z, optical +x
// is world +x, optical +y is world -y. Every synthetic image below is rendered through exactly
// this pose, so a pixel's world position is analytically known and the assertions are on metres,
// not on pixels.
[[nodiscard]] Eigen::Isometry3d overhead_pose(double height_m)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, 0.0, height_m);
  Eigen::Matrix3d rotation;
  rotation.col(0) = Eigen::Vector3d::UnitX();
  rotation.col(1) = -Eigen::Vector3d::UnitY();
  rotation.col(2) = -Eigen::Vector3d::UnitZ();
  pose.linear() = rotation;
  return pose;
}

// Renders a ground plane at z = 0 seen from `height_m`, then stamps an axis-aligned box standing
// on that plane into it by shortening the range wherever the ray crosses the box footprint.
[[nodiscard]] std::vector<float> render(
  double height_m, const std::vector<Eigen::AlignedBox3d> & boxes, bool with_ground)
{
  const auto camera = intrinsics();
  std::vector<float> depth(
    static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight),
    std::numeric_limits<float>::quiet_NaN());
  for (std::uint32_t row = 0; row < kHeight; ++row) {
    for (std::uint32_t column = 0; column < kWidth; ++column) {
      // The ray through this pixel, in world coordinates, parameterised by optical range.
      const double u = (static_cast<double>(column) - camera.cx) / camera.fx;
      const double v = (static_cast<double>(row) - camera.cy) / camera.fy;
      const auto pose = overhead_pose(height_m);
      const Eigen::Vector3d direction =
        pose.linear() * Eigen::Vector3d(u, v, 1.0);
      double best = std::numeric_limits<double>::infinity();
      for (const auto & box : boxes) {
        // The box top is the first surface a downward ray meets, so intersecting the top plane
        // and testing the footprint is exact for this geometry.
        const double range = (box.max().z() - height_m) / direction.z();
        if (range <= 0.0) {
          continue;
        }
        const Eigen::Vector3d hit = pose.translation() + range * direction;
        if (hit.x() >= box.min().x() && hit.x() <= box.max().x() &&
          hit.y() >= box.min().y() && hit.y() <= box.max().y())
        {
          best = std::min(best, range);
        }
      }
      if (with_ground) {
        const double ground = (0.0 - height_m) / direction.z();
        if (ground > 0.0) {
          best = std::min(best, ground);
        }
      }
      if (std::isfinite(best)) {
        depth[static_cast<std::size_t>(row) * kWidth + column] = static_cast<float>(best);
      }
    }
  }
  return depth;
}

TEST(DepthObstacleExtraction, RecoversASingleBoxAtItsSurveyedPose) {
  const Eigen::AlignedBox3d obstacle(
    Eigen::Vector3d(0.35, -0.25, 0.0), Eigen::Vector3d(0.50, -0.10, 0.50));
  const auto depth = render(2.0, {obstacle}, false);

  const auto result = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());

  ASSERT_TRUE(result) << to_string(result.error().code) << ": " << result.error().detail;
  ASSERT_EQ(result.value().size(), 1U);
  const ObstacleBoxFit & fit = result.value().front();
  // The camera sees only the top face, so the reconstruction is the top slab of the obstacle: its
  // footprint is recovered, its height is not, and the box sits at the observed surface. That is
  // all a single depth view contains.
  EXPECT_NEAR(fit.center.x(), 0.425, 0.03);
  EXPECT_NEAR(fit.center.y(), -0.175, 0.03);
  EXPECT_NEAR(fit.size.x(), 0.15, 0.05);
  EXPECT_NEAR(fit.size.y(), 0.15, 0.05);
  EXPECT_NEAR(fit.center.z(), 0.50, 0.05);
  EXPECT_GT(fit.point_count, 0U);
}

TEST(DepthObstacleExtraction, SeparatesTwoObstaclesAndOrdersThemDeterministically) {
  const Eigen::AlignedBox3d left(
    Eigen::Vector3d(-0.60, -0.10, 0.0), Eigen::Vector3d(-0.45, 0.05, 0.40));
  const Eigen::AlignedBox3d right(
    Eigen::Vector3d(0.40, -0.10, 0.0), Eigen::Vector3d(0.55, 0.05, 0.40));
  const auto depth = render(2.0, {left, right}, false);

  const auto result = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());

  ASSERT_TRUE(result) << to_string(result.error().code) << ": " << result.error().detail;
  ASSERT_EQ(result.value().size(), 2U);
  EXPECT_LT(result.value()[0].center.x(), result.value()[1].center.x());
  EXPECT_NEAR(result.value()[0].center.x(), -0.525, 0.03);
  EXPECT_NEAR(result.value()[1].center.x(), 0.475, 0.03);

  // Byte-identical input must give byte-identical output. The planning-scene projector treats any
  // change in this list as changed geometry, and a non-deterministic order would announce a
  // changed scene on every frame and reject every plan in flight.
  const auto repeat = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());
  ASSERT_TRUE(repeat);
  ASSERT_EQ(repeat.value().size(), result.value().size());
  for (std::size_t index = 0; index < result.value().size(); ++index) {
    EXPECT_EQ(repeat.value()[index].center, result.value()[index].center);
    EXPECT_EQ(repeat.value()[index].size, result.value()[index].size);
  }
}

TEST(DepthObstacleExtraction, VolumeOfInterestRemovesTheFloorBelowIt) {
  const Eigen::AlignedBox3d obstacle(
    Eigen::Vector3d(0.35, -0.25, 0.0), Eigen::Vector3d(0.50, -0.10, 0.50));
  const auto depth = render(2.0, {obstacle}, true);

  // Reach below the floor and the ground plane is itself reported as an obstacle: it is a real
  // surface the camera really sees, and nothing downstream could tell it from a pallet.
  auto with_floor = config();
  with_floor.volume_min.z() = -0.10;
  with_floor.max_box_extent_m = 4.0;
  const auto flooded = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, with_floor);
  ASSERT_TRUE(flooded);
  ASSERT_EQ(flooded.value().size(), 2U);
  EXPECT_GT(
    std::max(flooded.value()[0].size.x(), flooded.value()[1].size.x()), 1.5)
    << "one of the two clusters should be the floor slab";

  // Starting the volume of interest above the floor is what leaves only the obstacle.
  const auto cropped = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());
  ASSERT_TRUE(cropped);
  ASSERT_EQ(cropped.value().size(), 1U);
  EXPECT_NEAR(cropped.value().front().center.x(), 0.425, 0.03);
}

TEST(DepthObstacleExtraction, SelfFilterRemovesReturnsInsideRobotVolumes) {
  const Eigen::AlignedBox3d arm(
    Eigen::Vector3d(-0.10, -0.10, 0.0), Eigen::Vector3d(0.10, 0.10, 0.90));
  const Eigen::AlignedBox3d obstacle(
    Eigen::Vector3d(0.45, -0.10, 0.0), Eigen::Vector3d(0.60, 0.05, 0.50));
  const auto depth = render(2.0, {arm, obstacle}, false);

  const auto unfiltered = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());
  ASSERT_TRUE(unfiltered);
  EXPECT_EQ(unfiltered.value().size(), 2U);

  // A zero-length segment is the degenerate capsule: a sphere at one point, which is what the
  // representation must still be able to express for a link with no length.
  const std::vector<SelfFilterCapsule> capsules{
    SelfFilterCapsule{Eigen::Vector3d(0.0, 0.0, 0.90), Eigen::Vector3d(0.0, 0.0, 0.90), 0.30}};
  const auto filtered = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0),
    std::span<const SelfFilterCapsule>(capsules), config());
  ASSERT_TRUE(filtered);
  ASSERT_EQ(filtered.value().size(), 1U);
  EXPECT_NEAR(filtered.value().front().center.x(), 0.525, 0.04);
}

TEST(DepthObstacleExtraction, CapsuleCoversALongLinkWithoutReachingAcrossTheCorridor) {
  // The blocker this representation exists to remove, at unit scale: a 0.90 m link standing at
  // the origin with obstacles 0.30 m away on either side of it.
  const Eigen::AlignedBox3d link(
    Eigen::Vector3d(-0.10, -0.10, 0.0), Eigen::Vector3d(0.10, 0.10, 0.90));
  const Eigen::AlignedBox3d near_left(
    Eigen::Vector3d(-0.375, -0.075, 0.0), Eigen::Vector3d(-0.225, 0.075, 0.50));
  const Eigen::AlignedBox3d near_right(
    Eigen::Vector3d(0.225, -0.075, 0.0), Eigen::Vector3d(0.375, 0.075, 0.50));
  const auto depth = render(2.0, {link, near_left, near_right}, false);

  // A sphere at the link's frame origin has one radius for two jobs. Enclosing the link costs
  // hypot(0.10, 0.10, 0.90) = 0.912, and that same 0.912 reaches sideways past the near
  // obstacles' top faces at hypot(0.30, 0.50) = 0.583: both are deleted, and nothing downstream
  // can tell they were ever there.
  const std::vector<SelfFilterCapsule> sphere{
    SelfFilterCapsule{Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), 0.912}};
  const auto by_sphere = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0),
    std::span<const SelfFilterCapsule>(sphere), config());
  ASSERT_TRUE(by_sphere);
  EXPECT_TRUE(by_sphere.value().empty());

  // The capsule spends the link's length on its segment, so its radius is only the link's
  // cross-section. It covers strictly more of the link and keeps both obstacles.
  const std::vector<SelfFilterCapsule> capsule{
    SelfFilterCapsule{
      Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 0.90), 0.15}};
  const auto by_capsule = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0),
    std::span<const SelfFilterCapsule>(capsule), config());
  ASSERT_TRUE(by_capsule);
  ASSERT_EQ(by_capsule.value().size(), 2U);
  EXPECT_NEAR(by_capsule.value()[0].center.x(), -0.30, 0.03);
  EXPECT_NEAR(by_capsule.value()[1].center.x(), 0.30, 0.03);
}

TEST(DepthObstacleExtraction, VerticallyBandedReturnsBecomeOneObstacleNotSeveral) {
  // Two surfaces of one upright obstacle, sampled 0.06 m apart in z over a shared footprint,
  // the shape a grazing view of a vertical face leaves after `pixel_stride` has thinned it.
  const Eigen::AlignedBox3d lower(
    Eigen::Vector3d(0.30, -0.10, 0.0), Eigen::Vector3d(0.45, 0.05, 0.40));
  const Eigen::AlignedBox3d upper(
    Eigen::Vector3d(0.44, -0.10, 0.0), Eigen::Vector3d(0.60, 0.05, 0.46));
  const auto depth = render(2.0, {lower, upper}, false);

  auto strict = config();
  strict.cluster_vertical_gap_voxels = 0;
  const auto banded = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, strict);
  ASSERT_TRUE(banded);
  EXPECT_EQ(banded.value().size(), 2U) << "26-connectivity alone reports one obstacle as two";

  const auto merged = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());
  ASSERT_TRUE(merged);
  ASSERT_EQ(merged.value().size(), 1U);
  EXPECT_NEAR(merged.value().front().size.x(), 0.30, 0.03);
}

TEST(DepthObstacleExtraction, SpeckleBelowTheClusterFloorIsDiscarded) {
  auto depth = render(2.0, {}, false);
  // Three isolated returns, far apart, at plausible ranges. Nothing here is an obstacle.
  depth[10U * kWidth + 10U] = 1.5F;
  depth[60U * kWidth + 80U] = 1.5F;
  depth[100U * kWidth + 140U] = 1.5F;

  const auto result = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, config());

  ASSERT_TRUE(result);
  EXPECT_TRUE(result.value().empty());
}

TEST(DepthObstacleExtraction, RejectsBuffersAndPosesItCannotTrust) {
  const std::vector<float> depth(
    static_cast<std::size_t>(kWidth) * static_cast<std::size_t>(kHeight), 1.0F);

  auto broken = config();
  broken.voxel_size_m = 0.0;
  const auto bad_config = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), overhead_pose(2.0), {}, broken);
  ASSERT_FALSE(bad_config);
  EXPECT_EQ(bad_config.error().code, DepthObstacleErrorCode::InvalidConfiguration);

  auto bad_camera = intrinsics();
  bad_camera.fx = 0.0;
  const auto rejected_camera = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, bad_camera, overhead_pose(2.0), {}, config());
  ASSERT_FALSE(rejected_camera);
  EXPECT_EQ(rejected_camera.error().code, DepthObstacleErrorCode::InvalidIntrinsics);

  const std::vector<float> truncated(depth.begin(), depth.end() - 1);
  const auto short_buffer = extract_obstacle_boxes(
    std::span<const float>(truncated), kWidth, intrinsics(), overhead_pose(2.0), {}, config());
  ASSERT_FALSE(short_buffer);
  EXPECT_EQ(short_buffer.error().code, DepthObstacleErrorCode::InvalidDepthBuffer);

  Eigen::Isometry3d broken_pose = overhead_pose(2.0);
  broken_pose.translation().x() = std::numeric_limits<double>::quiet_NaN();
  const auto bad_pose = extract_obstacle_boxes(
    std::span<const float>(depth), kWidth, intrinsics(), broken_pose, {}, config());
  ASSERT_FALSE(bad_pose);
  EXPECT_EQ(bad_pose.error().code, DepthObstacleErrorCode::InvalidTransform);
}

}  // namespace
