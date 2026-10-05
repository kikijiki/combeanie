// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <restocker_interfaces/msg/object_observation.hpp>

#include "restocker_gazebo/ground_truth_conversion.hpp"

namespace restocker_gazebo
{
namespace
{

using Observation = restocker_interfaces::msg::ObjectObservation;

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
  sample.mutable_header()->mutable_stamp()->set_sec(12);
  sample.mutable_header()->mutable_stamp()->set_nsec(345'000'000);
  return sample;
}

class GroundTruthConversionTest : public ::testing::Test
{
protected:
  const GroundTruthConfig config_{load_ground_truth_config(RESTOCKER_TEST_SCENARIO)};
};

TEST_F(GroundTruthConversionTest, LoadsPinnedScenarioContract)
{
  EXPECT_EQ(config_.pose_topic, "/world/restocking/pose/info");
  EXPECT_EQ(config_.frame_id, "world");
  EXPECT_EQ(config_.backend_name, "gazebo_ground_truth");
  ASSERT_EQ(config_.products.size(), 3U);
  EXPECT_EQ(config_.products[0].source_object_id, "sim:stock_can_01");
  EXPECT_EQ(config_.products[0].product_class, Observation::PRODUCT_CLASS_CAN);
  EXPECT_EQ(config_.products[1].product_class, Observation::PRODUCT_CLASS_SMALL_BOTTLE);
  EXPECT_EQ(config_.products[2].product_class, Observation::PRODUCT_CLASS_LARGE_BOTTLE);
}

TEST_F(GroundTruthConversionTest, RejectsAmbiguousOrInvalidScenarioMetadata)
{
  const auto write_config = [](const YAML::Node & root, const std::string & suffix) {
    const auto path = std::filesystem::path(::testing::TempDir()) /
      ("restocker_ground_truth_" + suffix + ".yaml");
    std::ofstream stream(path);
    stream << root;
    stream.close();
    return path;
  };

  YAML::Node duplicate_model = YAML::LoadFile(RESTOCKER_TEST_SCENARIO);
  duplicate_model["products"][1]["model_name"] =
    duplicate_model["products"][0]["model_name"].as<std::string>();
  const auto duplicate_model_path = write_config(duplicate_model, "duplicate_model");
  EXPECT_THROW(
    static_cast<void>(load_ground_truth_config(duplicate_model_path)), std::invalid_argument);

  YAML::Node duplicate_source = YAML::LoadFile(RESTOCKER_TEST_SCENARIO);
  duplicate_source["products"][1]["source_object_id"] =
    duplicate_source["products"][0]["source_object_id"].as<std::string>();
  const auto duplicate_source_path = write_config(duplicate_source, "duplicate_source");
  EXPECT_THROW(
    static_cast<void>(load_ground_truth_config(duplicate_source_path)), std::invalid_argument);

  YAML::Node invalid_covariance = YAML::LoadFile(RESTOCKER_TEST_SCENARIO);
  invalid_covariance["pose_covariance_diagonal"][2] = -1.0;
  const auto invalid_covariance_path = write_config(invalid_covariance, "invalid_covariance");
  EXPECT_THROW(
    static_cast<void>(load_ground_truth_config(invalid_covariance_path)), std::invalid_argument);
}

TEST_F(GroundTruthConversionTest, FiltersUnconfiguredEntitiesAndMapsMetadata)
{
  auto sample = timestamped_sample();
  add_pose(sample, "floor", 0.0, 0.0, 0.0);
  add_pose(sample, "stock_can_01", -0.42, -0.8, 0.631);
  add_pose(sample, "stock_small_bottle_01", 0.0, -0.8, 0.67);

  const auto observations = convert_pose_sample(sample, config_);
  ASSERT_EQ(observations.size(), 2U);
  const auto & can = observations[0];
  EXPECT_EQ(can.status, Observation::STATUS_OK);
  EXPECT_EQ(can.header.frame_id, "world");
  EXPECT_EQ(can.header.stamp.sec, 12);
  EXPECT_EQ(can.header.stamp.nanosec, 345'000'000U);
  EXPECT_EQ(can.source_object_id, "sim:stock_can_01");
  EXPECT_EQ(can.product_class, Observation::PRODUCT_CLASS_CAN);
  EXPECT_TRUE(can.has_sku);
  EXPECT_EQ(can.sku, "SIM-CAN-STD");
  EXPECT_FLOAT_EQ(can.confidence, 1.0F);
  EXPECT_EQ(can.backend_name, "gazebo_ground_truth");
  EXPECT_DOUBLE_EQ(can.pose.pose.position.x, -0.42);
  EXPECT_EQ(can.orientation, Observation::ORIENTATION_UPRIGHT);
  for (std::size_t index = 0; index < 6; ++index) {
    EXPECT_DOUBLE_EQ(can.pose.covariance[index * 6 + index], 1.0e-8);
  }
}

TEST_F(GroundTruthConversionTest, SourceIdentityIsStableAcrossSamples)
{
  auto first = timestamped_sample();
  add_pose(first, "stock_can_01", 0.0, 0.0, 0.1);
  auto second = timestamped_sample();
  second.mutable_header()->mutable_stamp()->set_sec(13);
  add_pose(second, "stock_can_01", 0.2, 0.0, 0.1)->set_id(9999);

  const auto first_output = convert_pose_sample(first, config_);
  const auto second_output = convert_pose_sample(second, config_);
  ASSERT_EQ(first_output.size(), 1U);
  ASSERT_EQ(second_output.size(), 1U);
  EXPECT_EQ(first_output.front().source_object_id, second_output.front().source_object_id);
  EXPECT_DOUBLE_EQ(second_output.front().pose.pose.position.x, 0.2);
}

TEST_F(GroundTruthConversionTest, ClassifiesHorizontalContainer)
{
  auto sample = timestamped_sample();
  auto * pose = add_pose(sample, "stock_can_01", 0.0, 0.0, 0.1);
  pose->mutable_orientation()->set_x(std::sqrt(0.5));
  pose->mutable_orientation()->set_w(std::sqrt(0.5));

  const auto observations = convert_pose_sample(sample, config_);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().orientation, Observation::ORIENTATION_HORIZONTAL);
}

TEST_F(GroundTruthConversionTest, MissingTimestampProducesDiagnosticNotOkPose)
{
  gz::msgs::Pose_V sample;
  add_pose(sample, "stock_can_01", 0.0, 0.0, 0.1);

  const auto observations = convert_pose_sample(sample, config_);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().status, Observation::STATUS_MISSING_TIMESTAMP);
  EXPECT_FALSE(observations.front().status_detail.empty());
}

TEST_F(GroundTruthConversionTest, DuplicateConfiguredNameIsRejected)
{
  auto sample = timestamped_sample();
  add_pose(sample, "stock_can_01", 0.0, 0.0, 0.1);
  add_pose(sample, "stock_can_01", 0.2, 0.0, 0.1);

  const auto observations = convert_pose_sample(sample, config_);
  ASSERT_EQ(observations.size(), 1U);
  EXPECT_EQ(observations.front().status, Observation::STATUS_DUPLICATE_SOURCE);
}

TEST_F(GroundTruthConversionTest, NonUnitAndNonFiniteQuaternionsAreRejected)
{
  auto non_unit = timestamped_sample();
  add_pose(non_unit, "stock_can_01", 0.0, 0.0, 0.1)
  ->mutable_orientation()->set_w(2.0);
  auto non_finite = timestamped_sample();
  add_pose(non_finite, "stock_can_01", 0.0, 0.0, 0.1)
  ->mutable_orientation()->set_x(std::numeric_limits<double>::quiet_NaN());

  const auto non_unit_output = convert_pose_sample(non_unit, config_);
  const auto non_finite_output = convert_pose_sample(non_finite, config_);
  ASSERT_EQ(non_unit_output.size(), 1U);
  ASSERT_EQ(non_finite_output.size(), 1U);
  EXPECT_EQ(non_unit_output.front().status, Observation::STATUS_INVALID_POSE);
  EXPECT_EQ(non_finite_output.front().status, Observation::STATUS_INVALID_POSE);
}

}  // namespace
}  // namespace restocker_gazebo
