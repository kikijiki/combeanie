// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <string>

#include "restocker_perception/pose_error_evaluation.hpp"

namespace
{

using restocker_perception::FramedPose;
using restocker_perception::PoseErrorInputs;
using restocker_perception::PoseErrorPolicy;
using restocker_perception::evaluate_pose_error;
using SampleMessage = restocker_interfaces::msg::PoseErrorSample;

[[nodiscard]] FramedPose at(const std::string & frame, double x, double y, double z)
{
  FramedPose pose;
  pose.frame_id = frame;
  pose.pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

[[nodiscard]] PoseErrorInputs baseline()
{
  PoseErrorInputs inputs;
  inputs.source_object_id = "sim:stock_can_01";
  inputs.backend_name = "overhead_rgbd_colour_depth";
  inputs.backend_version = "1.0.0";
  inputs.estimate = at("world", 0.0, 0.0, 0.0);
  inputs.estimate_stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  inputs.ground_truth = at("world", 0.003, 0.004, 0.0);
  inputs.ground_truth_stamp = rclcpp::Time(10, 0, RCL_ROS_TIME);
  return inputs;
}

TEST(PoseErrorEvaluation, MeasuresAPairedEstimateAndReportsBothInstants)
{
  const auto sample = evaluate_pose_error(baseline(), PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_OK);
  EXPECT_EQ(sample.header.frame_id, "world");
  EXPECT_EQ(sample.source_object_id, "sim:stock_can_01");
  EXPECT_NEAR(sample.translation_error_m, 0.005, 1.0e-12);
  // The baseline estimate does not declare an orientation estimate, so the rotational half of the
  // sample is absent rather than zero. A 0.0 here is the whole defect this field exists to close:
  // it is what an orientation-blind pipeline and a perfect one would both report.
  EXPECT_EQ(sample.orientation_status, SampleMessage::ORIENTATION_NOT_ESTIMATED);
  EXPECT_TRUE(std::isnan(sample.axis_error_rad));
  EXPECT_TRUE(std::isnan(sample.observation_range_m));
  // Both instants are on the sample, so a reader can see what was compared with what rather than
  // having to trust that they were close.
  EXPECT_EQ(sample.estimate_stamp.sec, 10);
  EXPECT_EQ(sample.ground_truth_stamp.sec, 10);
  EXPECT_EQ(sample.backend_name, "overhead_rgbd_colour_depth");
}

TEST(PoseErrorEvaluation, CarriesObservationRangeOntoTheSample)
{
  auto inputs = baseline();
  inputs.observation_range_m = 0.30;
  const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_OK);
  EXPECT_DOUBLE_EQ(sample.observation_range_m, 0.30);
  EXPECT_EQ(
    restocker_perception::classify_observation_range(sample.observation_range_m),
    restocker_perception::ObservationRangeBand::Confirm);
}

TEST(PoseErrorEvaluation, ReportsThatNothingWasAssociatedRatherThanNothingAtAll)
{
  auto inputs = baseline();
  inputs.ground_truth.reset();
  const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_NO_GROUND_TRUTH);
  EXPECT_EQ(sample.translation_error_m, 0.0);
}

TEST(PoseErrorEvaluation, RefusesToPairAgainstAFrozenCarriedGroundTruth)
{
  // The hazard this exists for: Gazebo stops updating a model's pose while a DetachableJoint
  // holds it, so ground truth stays at the pre-grasp position while being republished with a
  // current stamp. Nothing about the sample looks stale, and the estimate is a metre away because
  // the robot has carried the product a metre.
  auto inputs = baseline();
  inputs.estimate = at("world", 0.0, 1.0, 0.4);
  inputs.ground_truth_frozen = true;
  const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_STALE_PAIRING);
  EXPECT_EQ(sample.translation_error_m, 0.0) << "a frozen pairing must report no error at all";
  EXPECT_NE(sample.status_detail.find("held"), std::string::npos);
}

TEST(PoseErrorEvaluation, RefusesToPairTwoInstantsTooFarApart)
{
  auto inputs = baseline();
  inputs.ground_truth_stamp = rclcpp::Time(10, 500000000, RCL_ROS_TIME);
  const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_STALE_PAIRING);

  // Symmetric: an estimate ahead of its ground truth is just as unpairable as one behind it.
  auto reversed = baseline();
  reversed.estimate_stamp = rclcpp::Time(10, 500000000, RCL_ROS_TIME);
  EXPECT_EQ(
    evaluate_pose_error(reversed, PoseErrorPolicy{}).status, SampleMessage::STATUS_STALE_PAIRING);

  PoseErrorPolicy tolerant;
  tolerant.maximum_pairing_skew = rclcpp::Duration(std::chrono::milliseconds(600));
  EXPECT_EQ(evaluate_pose_error(inputs, tolerant).status, SampleMessage::STATUS_OK);
}

TEST(PoseErrorEvaluation, RefusesToCompareAcrossFrames)
{
  auto inputs = baseline();
  inputs.ground_truth = at("camera_optical", 0.003, 0.004, 0.0);
  const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_FRAME_MISMATCH);
  EXPECT_EQ(sample.translation_error_m, 0.0);
}

TEST(PoseErrorEvaluation, ReportsNoOrientationErrorAtAllForAnEstimatorThatDoesNotEstimateOne)
{
  // The estimate is upright because this backend writes upright on every observation; the truth
  // is a container lying on its side. The translation is still reported. The rotation is not: the
  // sample must not report the quarter turn as found, nor a zero the estimator did not earn when
  // the truth is upright too.
  auto toppled = baseline();
  toppled.orientation_estimated = false;
  toppled.ground_truth = at("world", 0.003, 0.004, 0.0);
  toppled.ground_truth->pose.linear() =
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto sample = evaluate_pose_error(toppled, PoseErrorPolicy{});
  EXPECT_EQ(sample.status, SampleMessage::STATUS_OK);
  EXPECT_NEAR(sample.translation_error_m, 0.005, 1.0e-12);
  EXPECT_EQ(sample.orientation_status, SampleMessage::ORIENTATION_NOT_ESTIMATED);
  EXPECT_TRUE(std::isnan(sample.axis_error_rad));

  // And the same estimator against an upright truth reports the same absence, not a zero. These
  // two are the pair that has to stay distinguishable: an unmeasured quantity and a measured zero.
  auto upright = baseline();
  upright.orientation_estimated = false;
  const auto matching = evaluate_pose_error(upright, PoseErrorPolicy{});
  EXPECT_EQ(matching.orientation_status, SampleMessage::ORIENTATION_NOT_ESTIMATED);
  EXPECT_TRUE(std::isnan(matching.axis_error_rad));
}

TEST(PoseErrorEvaluation, MeasuresTheSymmetryAxisAndNotTheYaw)
{
  // Yaw about a cylinder's own axis moves nothing observable, and this used to be scored: the
  // previous version of this test asserted that a ground truth yawed by 0.25 rad produced 0.25 rad
  // of orientation error, against an estimator that had done nothing wrong. It is a property of
  // the scenario file, not of the pipeline.
  auto yawed = baseline();
  yawed.orientation_estimated = true;
  yawed.ground_truth = at("world", 0.0, 0.0, 0.0);
  yawed.ground_truth->pose.linear() =
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto unmoved = evaluate_pose_error(yawed, PoseErrorPolicy{});
  EXPECT_EQ(unmoved.status, SampleMessage::STATUS_OK);
  EXPECT_EQ(unmoved.orientation_status, SampleMessage::ORIENTATION_AXIS_ESTIMATED);
  EXPECT_NEAR(unmoved.axis_error_rad, 0.0, 1.0e-9);

  // Tilt does move it, and is reported.
  auto tilted = yawed;
  tilted.ground_truth->pose.linear() =
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto leaning = evaluate_pose_error(tilted, PoseErrorPolicy{});
  EXPECT_EQ(leaning.orientation_status, SampleMessage::ORIENTATION_AXIS_ESTIMATED);
  EXPECT_NEAR(leaning.axis_error_rad, 0.25, 1.0e-9);

  // A cylinder's axis has no sign: standing on its base and standing on its lid are the same
  // configuration, and half a turn about a horizontal axis must not read as pi radians of error.
  auto inverted = yawed;
  inverted.ground_truth->pose.linear() =
    Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_NEAR(evaluate_pose_error(inverted, PoseErrorPolicy{}).axis_error_rad, 0.0, 1.0e-9);
}

TEST(PoseErrorEvaluation, CarriesNoOrientationNumberOnAnyPathThatMeasuredNothing)
{
  // Every early return has to leave the rotational field unreadable as a measurement, not just the
  // ones that got as far as comparing poses. STATUS_NO_GROUND_TRUTH with axis_error_rad 0.0 would
  // land in an average as a perfect sample.
  for (const auto & inputs : {
      [] {auto value = baseline(); value.ground_truth.reset(); return value;}(),
      [] {auto value = baseline(); value.ground_truth_frozen = true; return value;}(),
      [] {
        auto value = baseline();
        value.orientation_estimated = true;
        value.ground_truth = at("camera_optical", 0.0, 0.0, 0.0);
        return value;
      }(),
    })
  {
    const auto sample = evaluate_pose_error(inputs, PoseErrorPolicy{});
    EXPECT_NE(sample.status, SampleMessage::STATUS_OK);
    EXPECT_TRUE(std::isnan(sample.axis_error_rad));
  }
}

TEST(PoseErrorEvaluation, RejectsAnEstimateWithNoIdentityOrNoFrame)
{
  auto anonymous = baseline();
  anonymous.source_object_id.clear();
  EXPECT_EQ(
    evaluate_pose_error(anonymous, PoseErrorPolicy{}).status,
    SampleMessage::STATUS_INTERNAL_ERROR);

  auto unframed = baseline();
  unframed.estimate.frame_id.clear();
  EXPECT_EQ(
    evaluate_pose_error(unframed, PoseErrorPolicy{}).status,
    SampleMessage::STATUS_INTERNAL_ERROR);
}

}  // namespace
