// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>

#include "restocker_task_executor/manipulation_geometry.hpp"

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] std::filesystem::path write_yaml(
  const YAML::Node & root, const std::string & name)
{
  const auto path = std::filesystem::path(::testing::TempDir()) / name;
  std::ofstream stream(path);
  stream << root;
  stream.close();
  return path;
}

TEST(ManipulationGeometry, LoadsDescriptionOwnedStockAndLaneVolumes)
{
  const auto result = load_manipulation_geometry(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(
    result.value().stock_region_in_shelf.max().isApprox(
      Eigen::Vector3d(0.68, -1.095, 0.17)));
  // Only the floor is lowered, by the configured settle tolerance below the tray surface.
  EXPECT_TRUE(
    result.value().stock_region_in_shelf.min().isApprox(
      Eigen::Vector3d(-0.68, -1.605, -0.182)));
  ASSERT_EQ(result.value().lanes.size(), 6U);
  const auto & lane = result.value().lanes.at("lane_01");
  EXPECT_EQ(lane.frame_id, "lane_01");
  EXPECT_DOUBLE_EQ(lane.center_x_m, -1.0);
  EXPECT_DOUBLE_EQ(lane.usable_bounds_in_lane.min().y(), 0.01);
  EXPECT_DOUBLE_EQ(lane.usable_bounds_in_lane.sizes().y(), 0.85);
  // The lane floor is the roller surface at the front rail, which is where a gravity-fed product
  // comes to rest, and the settle tolerance beside it is what a containment check lowers that one
  // face by.
  EXPECT_DOUBLE_EQ(lane.usable_bounds_in_lane.min().z(), 0.0);
  EXPECT_DOUBLE_EQ(lane.floor_settle_tolerance_m, 0.002);
  EXPECT_DOUBLE_EQ(lane.insert_entry_clearance_m, 0.045);
  // The release depth is per product, and the value the shipped clearance produces for the widest
  // catalogued product is the 0.10 the untruncated inserts were measured at. A narrower product is
  // released nearer the mouth, never deeper, so it leaves more of the lane behind it for the next.
  EXPECT_DOUBLE_EQ(lane_release_product_center_depth_m(lane, 0.045), 0.10);
  EXPECT_LT(
    lane_release_product_center_depth_m(lane, 0.033),
    lane_release_product_center_depth_m(lane, 0.045));
  EXPECT_TRUE(lane.insertion_axis.isApprox(Eigen::Vector3d::UnitY()));

  // The roller bed comes from the shelf survey, not from the lane, because one bed serves every
  // lane: 4 degrees about a datum at the rail face, shelf depth 0.9 less a 0.06 rail.
  EXPECT_DOUBLE_EQ(lane.floor_datum_depth_m, 0.84);
  EXPECT_DOUBLE_EQ(lane.incline_rad, 4.0 * std::acos(-1.0) / 180.0);
  for (const auto & [id, other] : result.value().lanes) {
    EXPECT_DOUBLE_EQ(other.incline_rad, lane.incline_rad) << id;
    EXPECT_DOUBLE_EQ(other.floor_datum_depth_m, lane.floor_datum_depth_m) << id;
  }
  // Level with the lane floor at the rail, rising behind it, and never below it inside the lane.
  EXPECT_DOUBLE_EQ(lane_floor_height_m(lane, lane.floor_datum_depth_m), 0.0);
  EXPECT_NEAR(lane_floor_height_m(lane, 0.0), 0.84 * std::tan(lane.incline_rad), 1.0e-12);
  // The bed is at or above the lane floor everywhere a product can travel, which is everywhere
  // behind the rail face. The lane volume reaches past that face, because a product leaning on
  // the rail leans past it, and there the bed keeps descending under the rail itself.
  EXPECT_GT(lane_floor_height_m(lane, lane.floor_datum_depth_m - 1.0e-9), 0.0);
  EXPECT_LT(lane_floor_height_m(lane, lane.usable_bounds_in_lane.max().y()), 0.0);
  // A product is carried in at the height of the bed's rear lip, because the straight horizontal
  // insert passes it over every part of the bed shallower than the insert depth.
  EXPECT_DOUBLE_EQ(lane_insertion_floor_height_m(lane), lane_floor_height_m(lane, 0.0));
  EXPECT_GT(
    lane_insertion_floor_height_m(lane),
    lane_floor_height_m(lane, lane_release_product_center_depth_m(lane, 0.045)));
  // The tallest catalogued product still fits above the highest point of the bed it stands on.
  EXPECT_LT(
    lane_insertion_floor_height_m(lane) + 0.290, lane.usable_bounds_in_lane.max().z());
}

// The bed can only carry a released product forward if the release point is short of the rail it
// delivers against.
TEST(ManipulationGeometry, RejectsAnInsertDepthAtOrPastTheFrontRail)
{
  YAML::Node root = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  root["lanes"]["lane_01"]["insert_entry_clearance_m"] = 0.85;
  const auto config = load_manipulation_geometry(
    write_yaml(root, "insert_depth_at_rail_geometry.yaml"));
  ASSERT_FALSE(config);
  EXPECT_EQ(config.error().code, ManipulationGeometryErrorCode::InvalidConfiguration);
}

TEST(ManipulationGeometry, RejectsAnInclineThatIsNotAShelf)
{
  YAML::Node root = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  root["shelf"]["lane_incline_deg"] = 45.0;
  const auto config = load_manipulation_geometry(
    write_yaml(root, "vertical_incline_geometry.yaml"));
  ASSERT_FALSE(config);
  EXPECT_EQ(config.error().code, ManipulationGeometryErrorCode::InvalidConfiguration);
}

TEST(ManipulationGeometry, ComputesExactCylinderBoundsForArbitraryOrientation)
{
  Eigen::Isometry3d frame_from_cylinder = Eigen::Isometry3d::Identity();
  frame_from_cylinder.translation() = Eigen::Vector3d(1.0, 2.0, 3.0);
  frame_from_cylinder.linear() =
    Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto bounds = cylinder_axis_aligned_bounds(
    frame_from_cylinder, CylinderEnvelope{0.03, 0.12});
  ASSERT_TRUE(bounds) << bounds.error().detail;
  EXPECT_TRUE(bounds.value().sizes().isApprox(Eigen::Vector3d(0.06, 0.12, 0.06), 1.0e-12));
}

TEST(ManipulationGeometry, ContainmentIncludesBoundaryAndHonorsMargin)
{
  const Eigen::AlignedBox3d volume(
    Eigen::Vector3d(-0.1, -0.1, -0.1), Eigen::Vector3d(0.1, 0.1, 0.1));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation().x() = 0.07;
  const CylinderEnvelope cylinder{0.03, 0.10};
  const auto boundary = contains_cylinder(volume, pose, cylinder);
  ASSERT_TRUE(boundary);
  EXPECT_TRUE(boundary.value());
  const auto margin = contains_cylinder(volume, pose, cylinder, 0.001);
  ASSERT_TRUE(margin);
  EXPECT_FALSE(margin.value());
}

// Regression: callers construct poses that seat a product flush against an inset face (the lane
// placement puts the product's base exactly on the lane floor) and then ask this predicate
// whether that pose is contained. The half-extent it is compared against is recovered from a
// composed rigid transform and carries a few units in the last place of rounding, so an exact
// comparison turned a deliberate flush fit into a placement failure.
TEST(ManipulationGeometry, ContainsAProductSeatedFlushAgainstAFaceDespiteRounding)
{
  const Eigen::AlignedBox3d volume(
    Eigen::Vector3d(-0.4, 0.0, 0.0), Eigen::Vector3d(0.4, 0.9, 0.4));
  const CylinderEnvelope cylinder{0.033, 0.122};
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.0, 0.45, 0.5 * cylinder.height_m);
  // A yaw composed the way the placement generator composes it, so the rotation block carries the
  // same rounding the real pose does rather than being exactly the identity.
  const Eigen::Matrix3d lane_from_world =
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  pose.linear() = lane_from_world.transpose() *
    Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  const auto seated = contains_cylinder(volume, pose, cylinder);
  ASSERT_TRUE(seated) << seated.error().detail;
  EXPECT_TRUE(seated.value());

  // The slack is a nanometre, so anything physically outside is still outside.
  Eigen::Isometry3d sunk = pose;
  sunk.translation().z() -= 1.0e-6;
  const auto below = contains_cylinder(volume, sunk, cylinder);
  ASSERT_TRUE(below) << below.error().detail;
  EXPECT_FALSE(below.value());
}

TEST(ManipulationGeometry, RejectsInvalidConfigurationTransformAndEnvelope)
{
  YAML::Node root = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  root["lanes"]["lane_01"]["insertion_axis"][1] = -1.0;
  const auto invalid_path = write_yaml(root, "invalid_manipulation_geometry.yaml");
  const auto config = load_manipulation_geometry(invalid_path);
  ASSERT_FALSE(config);
  EXPECT_EQ(config.error().code, ManipulationGeometryErrorCode::InvalidConfiguration);

  Eigen::Isometry3d invalid_transform = Eigen::Isometry3d::Identity();
  invalid_transform.translation().x() = std::numeric_limits<double>::infinity();
  const auto transform = cylinder_axis_aligned_bounds(
    invalid_transform, CylinderEnvelope{0.03, 0.12});
  ASSERT_FALSE(transform);
  EXPECT_EQ(transform.error().code, ManipulationGeometryErrorCode::InvalidTransform);

  const auto envelope = cylinder_axis_aligned_bounds(
    Eigen::Isometry3d::Identity(), CylinderEnvelope{-0.03, 0.12});
  ASSERT_FALSE(envelope);
  EXPECT_EQ(envelope.error().code, ManipulationGeometryErrorCode::InvalidEnvelope);
}

}  // namespace
}  // namespace restocker_task_executor
