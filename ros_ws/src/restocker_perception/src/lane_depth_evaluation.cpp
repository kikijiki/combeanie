// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/lane_depth_evaluation.hpp"

#include <cmath>
#include <string>

namespace restocker_perception
{
namespace
{

using SampleMessage = restocker_interfaces::msg::LaneDepthErrorSample;
using LaneObservationMessage = restocker_interfaces::msg::LaneObservation;

}  // namespace

SampleMessage evaluate_lane_depth_error(
  const LaneDepthErrorInputs & inputs, const LaneDepthErrorPolicy & policy)
{
  const LaneObservationMessage & measurement = inputs.measurement;
  SampleMessage sample;
  sample.header.stamp = measurement.header.stamp;
  sample.header.frame_id = measurement.header.frame_id;
  sample.lane_id = measurement.lane_id;
  sample.measurement_stamp = measurement.header.stamp;
  sample.measured_available_depth_m = measurement.available_depth_m;
  sample.measured_obstructed = measurement.obstructed;
  sample.coverage = measurement.confidence;
  sample.backend_name = measurement.backend_name;
  sample.backend_version = measurement.backend_version;

  if (measurement.lane_id.empty() || measurement.header.frame_id.empty()) {
    sample.status = SampleMessage::STATUS_INTERNAL_ERROR;
    sample.status_detail = "the measurement names no lane or no frame";
    return sample;
  }
  if (!inputs.ground_truth.has_value()) {
    sample.status = SampleMessage::STATUS_NO_GROUND_TRUTH;
    sample.status_detail = "no ground-truth observation is held for lane " + measurement.lane_id;
    return sample;
  }

  const LaneObservationMessage & truth = *inputs.ground_truth;
  sample.ground_truth_stamp = truth.header.stamp;
  sample.ground_truth_available_depth_m = truth.available_depth_m;
  sample.ground_truth_obstructed = truth.obstructed;

  if (truth.lane_id != measurement.lane_id ||
    truth.header.frame_id != measurement.header.frame_id)
  {
    sample.status = SampleMessage::STATUS_LANE_MISMATCH;
    sample.status_detail = "measurement describes " + measurement.lane_id + " in " +
      measurement.header.frame_id + ", ground truth describes " + truth.lane_id + " in " +
      truth.header.frame_id;
    return sample;
  }
  if (truth.status != LaneObservationMessage::STATUS_OK) {
    sample.status = SampleMessage::STATUS_NO_GROUND_TRUTH;
    sample.status_detail =
      "the ground-truth observation is not OK: " + truth.status_detail;
    return sample;
  }

  const rclcpp::Time measurement_stamp(measurement.header.stamp, RCL_ROS_TIME);
  const rclcpp::Time truth_stamp(truth.header.stamp, RCL_ROS_TIME);
  const auto skew = measurement_stamp > truth_stamp ?
    measurement_stamp - truth_stamp : truth_stamp - measurement_stamp;
  if (skew > policy.maximum_pairing_skew) {
    sample.status = SampleMessage::STATUS_STALE_PAIRING;
    sample.status_detail = "the two instants are " +
      std::to_string(skew.nanoseconds() / 1000000) + " ms apart, beyond the pairing tolerance";
    return sample;
  }
  if (!std::isfinite(measurement.available_depth_m) ||
    !std::isfinite(truth.available_depth_m))
  {
    sample.status = SampleMessage::STATUS_INTERNAL_ERROR;
    sample.status_detail = "an available depth is not finite";
    return sample;
  }

  // Signed, and computed before the status is decided, so a refused measurement still carries its
  // error.
  sample.available_depth_error_m =
    measurement.available_depth_m - truth.available_depth_m;

  if (measurement.status != LaneObservationMessage::STATUS_OK) {
    sample.status = SampleMessage::STATUS_MEASUREMENT_UNUSABLE;
    sample.status_detail =
      "the measurement declined to answer: " + measurement.status_detail;
    return sample;
  }
  sample.status = SampleMessage::STATUS_OK;
  return sample;
}

}  // namespace restocker_perception
