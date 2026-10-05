// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "restocker_perception/lane_depth_evaluation.hpp"

namespace
{

using restocker_perception::evaluate_lane_depth_error;
using restocker_perception::LaneDepthErrorInputs;
using restocker_perception::LaneDepthErrorPolicy;
using LaneObservation = restocker_interfaces::msg::LaneObservation;
using Sample = restocker_interfaces::msg::LaneDepthErrorSample;

[[nodiscard]] LaneObservation observation(
  const char * lane, double available_depth_m, std::int32_t seconds)
{
  LaneObservation message;
  message.header.frame_id = lane;
  message.header.stamp.sec = seconds;
  message.lane_id = lane;
  message.available_depth_m = available_depth_m;
  message.confidence = 1.0F;
  message.backend_name = "wrist_depth_lane_survey";
  message.backend_version = "1.0.0";
  message.status = LaneObservation::STATUS_OK;
  return message;
}

[[nodiscard]] LaneDepthErrorInputs baseline()
{
  LaneDepthErrorInputs inputs;
  inputs.measurement = observation("lane_03", 0.5040, 12);
  inputs.ground_truth = observation("lane_03", 0.5000, 12);
  inputs.ground_truth->backend_name = "gazebo_ground_truth";
  return inputs;
}

TEST(LaneDepthEvaluation, ReportsTheSignedDisagreement) {
  const Sample sample = evaluate_lane_depth_error(baseline(), LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_OK);
  EXPECT_EQ(sample.lane_id, "lane_03");
  EXPECT_NEAR(sample.available_depth_error_m, 0.0040, 1.0e-12);
  EXPECT_NEAR(sample.measured_available_depth_m, 0.5040, 1.0e-12);
  EXPECT_NEAR(sample.ground_truth_available_depth_m, 0.5000, 1.0e-12);
  // The sample names the measuring backend, not the simulator.
  EXPECT_EQ(sample.backend_name, "wrist_depth_lane_survey");
}

TEST(LaneDepthEvaluation, KeepsTheSignBecauseTheTwoDirectionsAreNotTheSame) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.measurement.available_depth_m = 0.4960;
  const Sample fuller = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_NEAR(fuller.available_depth_error_m, -0.0040, 1.0e-12);
  EXPECT_LT(fuller.available_depth_error_m, 0.0);
}

TEST(LaneDepthEvaluation, SaysSoWhenNoGroundTruthIsHeld) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.ground_truth.reset();
  const Sample sample = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_NO_GROUND_TRUTH);
  EXPECT_EQ(sample.lane_id, "lane_03");
}

TEST(LaneDepthEvaluation, RefusesToPairTwoDifferentLanes) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.ground_truth = observation("lane_04", 0.5, 12);
  const Sample sample = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_LANE_MISMATCH);
}

TEST(LaneDepthEvaluation, RefusesToPairAcrossThePairingTolerance) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.ground_truth = observation("lane_03", 0.5, 20);
  LaneDepthErrorPolicy policy;
  policy.maximum_pairing_skew = rclcpp::Duration(std::chrono::milliseconds(500));
  const Sample sample = evaluate_lane_depth_error(inputs, policy);
  EXPECT_EQ(sample.status, Sample::STATUS_STALE_PAIRING);
}

TEST(LaneDepthEvaluation, ReportsARefusedMeasurementWithItsErrorRatherThanDroppingIt) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.measurement.status = LaneObservation::STATUS_INSUFFICIENT_COVERAGE;
  inputs.measurement.status_detail = "coverage 0.04 is below the floor 0.90";
  inputs.measurement.available_depth_m = 0.85;
  inputs.measurement.confidence = 0.04F;
  const Sample sample = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_MEASUREMENT_UNUSABLE);
  // The error is kept, to show how wrong measurements below a candidate coverage floor would be.
  EXPECT_NEAR(sample.available_depth_error_m, 0.35, 1.0e-12);
  EXPECT_NEAR(static_cast<double>(sample.coverage), 0.04, 1.0e-6);
}

TEST(LaneDepthEvaluation, TreatsAGroundTruthThatIsNotOkAsNoGroundTruth) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.ground_truth->status = LaneObservation::STATUS_MISSING_TIMESTAMP;
  const Sample sample = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_NO_GROUND_TRUTH);
}

TEST(LaneDepthEvaluation, RefusesAMeasurementWithNoIdentity) {
  LaneDepthErrorInputs inputs = baseline();
  inputs.measurement.lane_id.clear();
  const Sample sample = evaluate_lane_depth_error(inputs, LaneDepthErrorPolicy{});
  EXPECT_EQ(sample.status, Sample::STATUS_INTERNAL_ERROR);
}

}  // namespace
