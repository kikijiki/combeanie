// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <optional>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/lane_depth_error_sample.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>

namespace restocker_perception
{

// One camera lane measurement and the simulator's own observation of the same lane.
//
// Pairing is by lane id, unlike the object evaluator, which matches spatially: a detector invents
// its own identifiers, so matching them to the simulator's would assume the answer. A lane
// identifier is configured, names a fixed volume of the workcell, and both producers read it from
// the same workcell_geometry.yaml, so pairing on it assumes nothing.
struct LaneDepthErrorInputs
{
  restocker_interfaces::msg::LaneObservation measurement;
  std::optional<restocker_interfaces::msg::LaneObservation> ground_truth;
};

struct LaneDepthErrorPolicy
{
  rclcpp::Duration maximum_pairing_skew{std::chrono::milliseconds(200)};
};

// Never returns a failure. An unmeasurable pair is a sample whose status says so, as in
// evaluate_pose_error: a run that produced no samples and a run whose samples all say "no ground
// truth" are different findings.
//
// A measurement the producer itself refused (insufficient coverage) still yields a sample with the
// depths and error filled in, under STATUS_MEASUREMENT_UNUSABLE, so the coverage floor can be
// chosen from a record that does not discard every measurement below the floor in force during the
// run.
[[nodiscard]] restocker_interfaces::msg::LaneDepthErrorSample evaluate_lane_depth_error(
  const LaneDepthErrorInputs & inputs, const LaneDepthErrorPolicy & policy);

}  // namespace restocker_perception
