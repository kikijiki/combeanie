// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/pose_error_evaluation.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace restocker_perception
{
namespace
{

using SampleMessage = restocker_interfaces::msg::PoseErrorSample;

}  // namespace

ObservationRangeBand classify_observation_range(double range_m) noexcept
{
  if (!std::isfinite(range_m) || range_m < 0.0) {
    return ObservationRangeBand::Unknown;
  }
  if (range_m < kConfirmRangeMaximumM) {
    return ObservationRangeBand::Confirm;
  }
  if (range_m < kOverviewRangeMaximumM) {
    return ObservationRangeBand::Overview;
  }
  return ObservationRangeBand::Far;
}

const char * observation_range_band_name(ObservationRangeBand band) noexcept
{
  switch (band) {
    case ObservationRangeBand::Confirm: return "confirm";
    case ObservationRangeBand::Overview: return "overview";
    case ObservationRangeBand::Far: return "far";
    case ObservationRangeBand::Unknown: return "unknown";
  }
  return "unknown";
}

SampleMessage evaluate_pose_error(
  const PoseErrorInputs & inputs, const PoseErrorPolicy & policy)
{
  SampleMessage sample;
  sample.header.stamp = inputs.estimate_stamp;
  sample.header.frame_id = inputs.estimate.frame_id;
  sample.source_object_id = inputs.source_object_id;
  sample.estimate_stamp = inputs.estimate_stamp;
  sample.ground_truth_stamp = inputs.ground_truth_stamp;
  sample.backend_name = inputs.backend_name;
  sample.backend_version = inputs.backend_version;
  // Set before any early return, so that every path out of here, including the ones that report
  // why no comparison was made, carries a rotational figure that cannot be read as measured.
  // The message's field default is 0.0, and 0.0 is the value this whole change exists to stop
  // appearing where nothing was measured.
  sample.axis_error_rad = std::numeric_limits<double>::quiet_NaN();
  sample.observation_range_m = inputs.observation_range_m;
  sample.orientation_status = inputs.orientation_estimated ?
    SampleMessage::ORIENTATION_AXIS_ESTIMATED : SampleMessage::ORIENTATION_NOT_ESTIMATED;

  if (inputs.source_object_id.empty() || inputs.estimate.frame_id.empty()) {
    sample.status = SampleMessage::STATUS_INTERNAL_ERROR;
    sample.status_detail = "the estimate carries no identity or no frame";
    return sample;
  }
  if (!inputs.ground_truth.has_value()) {
    sample.status = SampleMessage::STATUS_NO_GROUND_TRUTH;
    sample.status_detail = "no ground-truth sample is associated with this estimate";
    return sample;
  }
  if (inputs.ground_truth_frozen) {
    // Reported as a stale pairing rather than as a large error, which is the whole point: the
    // ground-truth value is stale even though the sample carrying it is not.
    sample.status = SampleMessage::STATUS_STALE_PAIRING;
    sample.status_detail =
      "the simulator is not updating this object's pose while it is held, so its ground truth is "
      "frozen at a pre-grasp value";
    return sample;
  }
  if (inputs.estimate_stamp.get_clock_type() != inputs.ground_truth_stamp.get_clock_type()) {
    sample.status = SampleMessage::STATUS_INTERNAL_ERROR;
    sample.status_detail = "estimate and ground-truth stamps use different clocks";
    return sample;
  }
  const auto skew = inputs.estimate_stamp > inputs.ground_truth_stamp ?
    inputs.estimate_stamp - inputs.ground_truth_stamp :
    inputs.ground_truth_stamp - inputs.estimate_stamp;
  if (skew > policy.maximum_pairing_skew) {
    sample.status = SampleMessage::STATUS_STALE_PAIRING;
    sample.status_detail =
      "the two instants are " + std::to_string(skew.nanoseconds() / 1000000) +
      " ms apart, beyond the pairing tolerance";
    return sample;
  }

  // compare_framed_poses refuses to subtract poses expressed in different frames, so a comparison
  // that was never geometrically meaningful reports no error rather than a small-looking one.
  const auto measured = compare_framed_poses(inputs.estimate, *inputs.ground_truth);
  if (!measured) {
    sample.status = measured.error().code == PerceptionErrorCode::FrameMismatch ?
      SampleMessage::STATUS_FRAME_MISMATCH : SampleMessage::STATUS_INTERNAL_ERROR;
    sample.status_detail = measured.error().detail;
    return sample;
  }

  sample.translation_error_m = measured.value().translation_error_m;
  // Only when the estimate said it had measured the axis. Otherwise the angle between a constant
  // and the truth is a measurement of the truth: it is zero for every scenario in this repository
  // because every product spawns upright, and it would move if the scenario file changed while
  // the pipeline did not. That is the wrong thing under the instrument.
  if (inputs.orientation_estimated) {
    sample.axis_error_rad = measured.value().axis_error_rad;
  }
  sample.status = SampleMessage::STATUS_OK;
  return sample;
}

}  // namespace restocker_perception
