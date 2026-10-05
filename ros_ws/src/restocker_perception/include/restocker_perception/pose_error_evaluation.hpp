// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <limits>
#include <optional>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/pose_error_sample.hpp>

#include "restocker_perception/frame_geometry.hpp"

namespace restocker_perception
{

// One estimate and the ground truth it is to be measured against, already associated.
//
// Both poses carry their frame, and both instants are carried separately from the poses: "were
// these the same object at the same instant" and "how far apart are they" are answered from
// different fields and neither may be inferred from the other.
struct PoseErrorInputs
{
  // The identity the sample is reported under: the ground-truth identity the estimate was
  // associated with, since perception cannot know the simulator's.
  std::string source_object_id;
  std::string backend_name;
  std::string backend_version;

  FramedPose estimate;
  rclcpp::Time estimate_stamp{std::int64_t{0}, RCL_ROS_TIME};

  // Absent when no ground-truth sample has been associated with this estimate at all.
  std::optional<FramedPose> ground_truth;
  rclcpp::Time ground_truth_stamp{std::int64_t{0}, RCL_ROS_TIME};

  // True while the simulator is not updating this object's pose, so its ground-truth sample keeps
  // arriving with a fresh stamp and a value frozen at some earlier instant. See
  // PoseErrorPolicy::held_settle for why this outlives the hold itself.
  bool ground_truth_frozen{false};

  // Whether the estimate's orientation is a measurement rather than a constant the backend writes
  // on every observation. Supplied rather than inferred from the estimate pose, as with
  // ground_truth_frozen: one sample of a constant is indistinguishable from one sample of a
  // correct estimate. The producing backend declares it through the rotational block of the
  // covariance it publishes; see declares_axis_estimate.
  bool orientation_estimated{false};

  // Camera-to-product range at the estimate stamp, when known. NaN when the evaluator has no
  // camera frame configured or TF could not resolve one; evaluate_pose_error copies it onto the
  // sample so a missing range does not become 0.0 m.
  double observation_range_m{std::numeric_limits<double>::quiet_NaN()};
};

struct PoseErrorPolicy
{
  // How far apart the two instants may be and still describe the same configuration of the cell.
  // The overhead camera runs at 6 Hz and ground truth at 10 Hz, so two independently sampled
  // streams are up to about 180 ms apart at worst; this admits that and nothing slower.
  rclcpp::Duration maximum_pairing_skew{std::chrono::milliseconds(200)};
};

// Observation-range bands. Confirm is the close view that authorises a grasp; overview is a tray
// survey station; far is everything else, including the overhead pipeline at 2.06 m. Confirm
// standoffs are 0.25-0.35 m to the product; range to the product centre for the tallest catalogued
// bottle at that elevation reaches about 0.42 m, so the confirm/overview edge sits at 0.50 m,
// still below every tray-station near range (~0.55 m). Overview tops out below the overhead
// working range.
inline constexpr double kConfirmRangeMaximumM = 0.50;
inline constexpr double kOverviewRangeMaximumM = 1.20;

enum class ObservationRangeBand
{
  Unknown,
  Confirm,
  Overview,
  Far,
};

// Classifies a camera-to-product range into the report buckets. Non-finite ranges are Unknown
// rather than Far, so a missing measurement does not average into the overhead bucket.
[[nodiscard]] ObservationRangeBand classify_observation_range(double range_m) noexcept;

[[nodiscard]] const char * observation_range_band_name(ObservationRangeBand band) noexcept;

// Measures one pose estimate against simulator ground truth, and reports why not when it cannot.
//
// Evaluation output. It never returns a failure: an unmeasurable pair is a sample whose status
// says so, so "no sample was produced" and "a sample was produced and was fine" look different.
// For the same reason the rotational error is NaN with ORIENTATION_NOT_ESTIMATED, not 0.0, when
// the estimate carries no orientation estimate: an orientation-blind pipeline and a perfect one
// must not produce the same number.
//
// Gazebo does not update a model's pose while a DetachableJoint holds it, so a carried product's
// ground truth is frozen at its pre-grasp position for the whole carry while still being
// republished with a current stamp. Nothing about that sample looks stale (age, frame,
// finiteness), and pairing an estimate against it would report the length of the carry as a pose
// error. ground_truth_frozen is the only signal, so it is an input rather than inferred here.
[[nodiscard]] restocker_interfaces::msg::PoseErrorSample evaluate_pose_error(
  const PoseErrorInputs & inputs, const PoseErrorPolicy & policy);

}  // namespace restocker_perception
