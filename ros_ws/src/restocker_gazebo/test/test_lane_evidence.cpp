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
#include <string>

#include <restocker_interfaces/msg/lane_observation.hpp>

#include "restocker_gazebo/ground_truth_conversion.hpp"
#include "restocker_gazebo/lane_evidence.hpp"

namespace restocker_gazebo
{
namespace
{

using Observation = restocker_interfaces::msg::LaneObservation;

gz::msgs::Pose * add_pose(
  gz::msgs::Pose_V & sample, const std::string & name, double x, double y, double z)
{
  gz::msgs::Pose * pose = sample.add_pose();
  pose->set_name(name);
  pose->mutable_position()->set_x(x);
  pose->mutable_position()->set_y(y);
  pose->mutable_position()->set_z(z);
  pose->mutable_orientation()->set_w(1.0);
  return pose;
}

gz::msgs::Pose_V timestamped_sample()
{
  gz::msgs::Pose_V sample;
  sample.mutable_header()->mutable_stamp()->set_sec(21);
  sample.mutable_header()->mutable_stamp()->set_nsec(125'000'000);
  return sample;
}

class LaneEvidenceTest : public ::testing::Test
{
protected:
  const GroundTruthConfig ground_truth_{load_ground_truth_config(RESTOCKER_TEST_SCENARIO)};
  const LaneEvidenceConfig config_{load_lane_evidence_config(
      RESTOCKER_TEST_WORKCELL_GEOMETRY, RESTOCKER_TEST_PRODUCT_CATALOG, ground_truth_)};
};

TEST_F(LaneEvidenceTest, LoadsDescriptionOwnedVolumesAndProductEnvelopes)
{
  ASSERT_EQ(config_.lanes.size(), 6U);
  EXPECT_EQ(config_.lanes[0].id, "lane_01");
  EXPECT_EQ(config_.lanes[0].frame_id, "lane_01");
  EXPECT_DOUBLE_EQ(config_.lanes[0].center_x_m, -1.0);
  EXPECT_DOUBLE_EQ(config_.lanes[0].bounds_in_lane.min().y(), 0.01);
  EXPECT_DOUBLE_EQ(config_.lanes[0].bounds_in_lane.sizes().y(), 0.85);
  ASSERT_EQ(config_.products.size(), 3U);
  EXPECT_EQ(config_.products[0].model_name, "stock_can_01");
  EXPECT_DOUBLE_EQ(config_.products[0].radius_m, 0.033);
  EXPECT_DOUBLE_EQ(config_.products[0].height_m, 0.122);
  EXPECT_TRUE(config_.world_from_shelf.translation().isApprox(Eigen::Vector3d(0.0, 0.55, 0.75)));
}

TEST_F(LaneEvidenceTest, EmptyLanesRetainFullDepthWithTypedProvenance)
{
  const auto observations = convert_lane_evidence_sample(timestamped_sample(), config_);
  ASSERT_EQ(observations.size(), 6U);
  for (const auto & observation : observations) {
    EXPECT_EQ(observation.status, Observation::STATUS_OK);
    EXPECT_EQ(observation.header.frame_id, observation.lane_id);
    EXPECT_EQ(observation.header.stamp.sec, 21);
    EXPECT_EQ(observation.header.stamp.nanosec, 125'000'000U);
    EXPECT_DOUBLE_EQ(observation.available_depth_m, 0.85);
    EXPECT_FALSE(observation.obstructed);
    EXPECT_TRUE(observation.observed_source_object_ids.empty());
    EXPECT_EQ(observation.backend_name, "gazebo_ground_truth");
    EXPECT_FLOAT_EQ(observation.confidence, 1.0F);
  }
}

TEST_F(LaneEvidenceTest, FullyContainedProductIsOccupancyEvidence)
{
  auto sample = timestamped_sample();
  add_pose(sample, "stock_can_01", -1.0, 1.37, 0.816);

  const auto observations = convert_lane_evidence_sample(sample, config_);
  ASSERT_EQ(observations.size(), 6U);
  ASSERT_EQ(observations[0].observed_source_object_ids.size(), 1U);
  EXPECT_EQ(observations[0].observed_source_object_ids[0], "sim:stock_can_01");
  EXPECT_NEAR(observations[0].available_depth_m, 0.777, 1.0e-12);
  EXPECT_FALSE(observations[0].obstructed);
  EXPECT_TRUE(observations[1].observed_source_object_ids.empty());
}

// Poses a product the way the gravity-feed bed leaves it: lying flat on the bed, so its axis is
// the bed's surface normal rather than world +Z. The bed pitches nose-down toward the customer,
// which is a negative rotation about the shelf's +X.
gz::msgs::Pose * add_bedded_pose(
  gz::msgs::Pose_V & sample, const std::string & name, double x, double y, double z)
{
  gz::msgs::Pose * pose = add_pose(sample, name, x, y, z);
  const Eigen::Quaterniond tilt(
    Eigen::AngleAxisd(-4.0 * std::acos(-1.0) / 180.0, Eigen::Vector3d::UnitX()));
  pose->mutable_orientation()->set_x(tilt.x());
  pose->mutable_orientation()->set_y(tilt.y());
  pose->mutable_orientation()->set_z(tilt.z());
  pose->mutable_orientation()->set_w(tilt.w());
  return pose;
}

// The lane is a gravity-feed roller bed, so a placed product moves for a few seconds and its
// depth changes at every instant. Placement is proven by the lane naming the product, and the
// arm's retreat is planned against a scene certified from the same snapshots, so every reading
// between release and rest must hold.
//
// The three poses below are the ends and the middle of that run for the large bottle (the
// tallest and widest catalogued product), from the DART measurement in workcell_geometry.yaml.
// The world poses add the surveyed shelf datum (0, 0.55, 0.75) to a shelf-frame pose.
TEST_F(LaneEvidenceTest, ARollingProductStaysOccupancyEvidenceForTheWholeRun)
{
  // Released at the lane mouth: still upright, standing on the bed's rear lip 0.0587 above the
  // lane floor plus the planner's 0.002 insertion clearance, centre at half its 0.290 height.
  auto released = timestamped_sample();
  add_pose(released, "stock_large_bottle_01", -1.0, 0.55 + 0.10, 0.75 + 0.0607385 + 0.145);
  const auto at_release = convert_lane_evidence_sample(released, config_);
  ASSERT_EQ(at_release[0].observed_source_object_ids.size(), 1U);
  EXPECT_EQ(at_release[0].observed_source_object_ids[0], "sim:stock_large_bottle_01");
  EXPECT_FALSE(at_release[0].obstructed);
  // Its rear face is 0.055 into the lane, 0.045 past the lane's own rear clearance.
  EXPECT_NEAR(at_release[0].available_depth_m, 0.045, 1.0e-6);

  // Half way down, riding the bed and therefore tilted with it.
  auto rolling = timestamped_sample();
  add_bedded_pose(rolling, "stock_large_bottle_01", -1.0, 0.55 + 0.45, 0.75 + 0.171921);
  const auto mid_roll = convert_lane_evidence_sample(rolling, config_);
  ASSERT_EQ(mid_roll[0].observed_source_object_ids.size(), 1U);
  EXPECT_FALSE(mid_roll[0].obstructed);
  EXPECT_NEAR(mid_roll[0].available_depth_m, 0.385, 1.0e-4);

  // At rest against the front rail. This is the pose every completed placement now ends at, and
  // it leans past the rail face by the height it stands above the rail's top edge, which is what
  // the lane usable volume was extended past that face to contain.
  auto settled = timestamped_sample();
  add_bedded_pose(settled, "stock_large_bottle_01", -1.0, 0.55 + 0.799, 0.75 + 0.1482);
  const auto at_rest = convert_lane_evidence_sample(settled, config_);
  ASSERT_EQ(at_rest[0].observed_source_object_ids.size(), 1U);
  EXPECT_FALSE(at_rest[0].obstructed);
  EXPECT_NEAR(at_rest[0].available_depth_m, 0.734, 1.0e-4);

  // The lane always names the product, so a mid-roll reader cannot conclude the placement
  // failed. Free depth only grows as the product runs forward, so a mid-roll reading understates
  // the room left, which is the safe direction.
  EXPECT_LT(at_release[0].available_depth_m, mid_roll[0].available_depth_m);
  EXPECT_LT(mid_roll[0].available_depth_m, at_rest[0].available_depth_m);
  EXPECT_LT(at_rest[0].available_depth_m, config_.lanes[0].bounds_in_lane.sizes().y());
  // Depth is measured from the lane's rear edge, so once packed against the rail the missing
  // depth is the product's own footprint.
  EXPECT_NEAR(
    config_.lanes[0].bounds_in_lane.sizes().y() - at_rest[0].available_depth_m, 0.116, 1.0e-4);
  // No neighbour is disturbed at any point in the run.
  for (const auto & sampled : {at_release, mid_roll, at_rest}) {
    for (std::size_t index = 1; index < sampled.size(); ++index) {
      EXPECT_FALSE(sampled[index].obstructed) << index;
      EXPECT_TRUE(sampled[index].observed_source_object_ids.empty()) << index;
    }
  }
}

// A lane holds a column, and its free depth is the rearmost product's face, which is why
// `available_depth_m` can stand in for a count. The two products below are packed on the bed as
// gravity leaves them: axes one sum-of-radii apart along the incline, both tilted with it, the
// front one against the rail.
//
// This pins the increment: what one more product takes off the free depth. For two products of
// the same class it is one diameter foreshortened by the incline, `2 * radius * cos(incline)`,
// the pitch the world state's placement proof is expressed in. When heights differ, the tilt
// term in the depth footprint depends on each product's height, so a shorter product behind a
// taller one gives back `(taller - shorter) / 2 * sin(incline)` of depth. The catalogue's worst
// pair is a can behind a large bottle, which is the pair below.
TEST_F(LaneEvidenceTest, ALaneHoldsAColumnAndReportsItsRearmostFace)
{
  const double incline = 4.0 * std::acos(-1.0) / 180.0;
  auto column = timestamped_sample();
  // Front of the column: the large bottle at rest against the rail, the same pose the roll test
  // ends at.
  add_bedded_pose(column, "stock_large_bottle_01", -1.0, 0.55 + 0.799, 0.75 + 0.1482);
  // Behind it, touching: centres 0.045 + 0.033 apart along the bed.
  const double spacing = (0.045 + 0.033) * std::cos(incline);
  add_bedded_pose(column, "stock_can_01", -1.0, 0.55 + 0.799 - spacing, 0.75 + 0.07146);

  const auto observations = convert_lane_evidence_sample(column, config_);
  ASSERT_EQ(observations.size(), 6U);
  // Both are contained, so both are named, and the lane is not obstructed by holding two.
  ASSERT_EQ(observations[0].observed_source_object_ids.size(), 2U);
  EXPECT_EQ(observations[0].observed_source_object_ids[0], "sim:stock_can_01");
  EXPECT_EQ(observations[0].observed_source_object_ids[1], "sim:stock_large_bottle_01");
  EXPECT_FALSE(observations[0].obstructed);

  // The free depth is the can's rear face, not the bottle's: the lane reports the room behind the
  // whole column.
  const double can_extent = 0.033 * std::cos(incline) + 0.061 * std::sin(incline);
  const double expected_free = 0.799 - spacing - can_extent - 0.01;
  EXPECT_NEAR(observations[0].available_depth_m, expected_free, 1.0e-6);

  // The second product costs the lane its own pitch less the depth its shorter body gives back
  // on the incline. The placement proof compares against this; 0.0059 m is why the world state's
  // growth tolerance is just under a centimetre rather than the millimetre a same-class column
  // would need.
  const double one_bottle_free = 0.799 -
    (0.045 * std::cos(incline) + 0.145 * std::sin(incline)) - 0.01;
  const double increment = one_bottle_free - observations[0].available_depth_m;
  const double can_pitch = 2.0 * 0.033 * std::cos(incline);
  EXPECT_NEAR(increment, can_pitch - 0.5 * (0.290 - 0.122) * std::sin(incline), 1.0e-9);
  EXPECT_LT(increment, can_pitch);
  EXPECT_GT(increment, can_pitch - 0.010);

  // Neither product disturbs a neighbouring lane.
  for (std::size_t index = 1; index < observations.size(); ++index) {
    EXPECT_FALSE(observations[index].obstructed) << index;
    EXPECT_TRUE(observations[index].observed_source_object_ids.empty()) << index;
  }
}

TEST_F(LaneEvidenceTest, PartialRearOverlapMarksLaneObstructed)
{
  auto sample = timestamped_sample();
  add_pose(sample, "stock_can_01", -1.0, 0.55, 0.816);

  const auto observations = convert_lane_evidence_sample(sample, config_);
  EXPECT_TRUE(observations[0].obstructed);
  EXPECT_DOUBLE_EQ(observations[0].available_depth_m, 0.0);
  EXPECT_TRUE(observations[0].observed_source_object_ids.empty());
}

TEST_F(LaneEvidenceTest, DividerCrossingObstructsBothAdjacentLanes)
{
  auto sample = timestamped_sample();
  add_pose(sample, "stock_can_01", -0.8, 0.75, 0.816);

  const auto observations = convert_lane_evidence_sample(sample, config_);
  EXPECT_TRUE(observations[0].obstructed);
  EXPECT_TRUE(observations[1].obstructed);
  EXPECT_TRUE(observations[0].observed_source_object_ids.empty());
  EXPECT_TRUE(observations[1].observed_source_object_ids.empty());
}

TEST_F(LaneEvidenceTest, HorizontalCylinderUsesItsProjectedEnvelope)
{
  auto sample = timestamped_sample();
  auto * pose = add_pose(sample, "stock_can_01", -1.0, 0.70, 0.85);
  pose->mutable_orientation()->set_x(std::sqrt(0.5));
  pose->mutable_orientation()->set_w(std::sqrt(0.5));

  const auto observations = convert_lane_evidence_sample(sample, config_);
  ASSERT_EQ(observations[0].observed_source_object_ids.size(), 1U);
  EXPECT_NEAR(observations[0].available_depth_m, 0.079, 1.0e-12);
  EXPECT_FALSE(observations[0].obstructed);
}

TEST_F(LaneEvidenceTest, ShelfTransformIsAppliedBeforeLaneClassification)
{
  LaneEvidenceConfig rotated = config_;
  rotated.world_from_shelf = Eigen::Isometry3d::Identity();
  rotated.world_from_shelf.translation() = Eigen::Vector3d(2.0, 3.0, 0.5);
  rotated.world_from_shelf.linear() =
    Eigen::AngleAxisd(0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d shelf_center(-1.0, 0.82, 0.066);
  const Eigen::Vector3d world_center = rotated.world_from_shelf * shelf_center;
  auto sample = timestamped_sample();
  add_pose(
    sample, "stock_can_01", world_center.x(), world_center.y(), world_center.z());

  const auto observations = convert_lane_evidence_sample(sample, rotated);
  ASSERT_EQ(observations[0].observed_source_object_ids.size(), 1U);
  EXPECT_FALSE(observations[0].obstructed);
}

TEST_F(LaneEvidenceTest, InvalidSamplesRejectEveryLaneConsistently)
{
  gz::msgs::Pose_V missing_stamp;
  const auto missing_output = convert_lane_evidence_sample(missing_stamp, config_);
  for (const auto & observation : missing_output) {
    EXPECT_EQ(observation.status, Observation::STATUS_MISSING_TIMESTAMP);
  }

  auto duplicate = timestamped_sample();
  add_pose(duplicate, "stock_can_01", 0.0, 0.0, 0.0);
  add_pose(duplicate, "stock_can_01", 0.0, 0.0, 0.0);
  const auto duplicate_output = convert_lane_evidence_sample(duplicate, config_);
  for (const auto & observation : duplicate_output) {
    EXPECT_EQ(observation.status, Observation::STATUS_DUPLICATE_SOURCE);
  }

  auto invalid = timestamped_sample();
  add_pose(invalid, "stock_can_01", 0.0, 0.0, 0.0)
  ->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
  const auto invalid_output = convert_lane_evidence_sample(invalid, config_);
  for (const auto & observation : invalid_output) {
    EXPECT_EQ(observation.status, Observation::STATUS_INVALID_GEOMETRY);
  }
}

TEST_F(LaneEvidenceTest, RejectsScenarioCatalogSemanticDisagreement)
{
  YAML::Node catalog = YAML::LoadFile(RESTOCKER_TEST_PRODUCT_CATALOG);
  catalog["geometries"][0]["product_class"] = "large_bottle";
  const auto path = std::filesystem::path(::testing::TempDir()) / "mismatched_catalog.yaml";
  std::ofstream stream(path);
  stream << catalog;
  stream.close();
  EXPECT_THROW(
    static_cast<void>(load_lane_evidence_config(
      RESTOCKER_TEST_WORKCELL_GEOMETRY, path, ground_truth_)),
    std::invalid_argument);
}

TEST_F(LaneEvidenceTest, RejectsUnsupportedLaneFrameConvention)
{
  YAML::Node workcell = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  workcell["lanes"]["lane_01"]["insertion_axis"][1] = -1.0;
  const auto path = std::filesystem::path(::testing::TempDir()) / "invalid_lane_axis.yaml";
  std::ofstream stream(path);
  stream << workcell;
  stream.close();
  EXPECT_THROW(
    static_cast<void>(load_lane_evidence_config(
      path, RESTOCKER_TEST_PRODUCT_CATALOG, ground_truth_)),
    std::invalid_argument);
}

}  // namespace
}  // namespace restocker_gazebo
