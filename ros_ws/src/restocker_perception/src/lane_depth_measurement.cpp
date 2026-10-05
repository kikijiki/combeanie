// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/lane_depth_measurement.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>

namespace restocker_perception
{
namespace
{

using MeasurementResult = DepthObstacleResult<LaneDepthMeasurement>;

[[nodiscard]] DepthObstacleError invalid_configuration(std::string detail)
{
  return DepthObstacleError{DepthObstacleErrorCode::InvalidConfiguration, std::move(detail)};
}

[[nodiscard]] bool valid_intrinsics(const DepthCameraIntrinsics & intrinsics) noexcept
{
  return std::isfinite(intrinsics.fx) && std::isfinite(intrinsics.fy) &&
         std::isfinite(intrinsics.cx) && std::isfinite(intrinsics.cy) && intrinsics.fx > 0.0 &&
         intrinsics.fy > 0.0 && intrinsics.width > 0U && intrinsics.height > 0U;
}

}  // namespace

double LaneDepthWindow::bed_height_at(double lane_depth_m) const noexcept
{
  return (bed_datum_depth_m - lane_depth_m) * bed_slope;
}

double LaneDepthWindow::window_front_depth_m() const noexcept
{
  return bed_datum_depth_m - front_inset_m;
}

bool valid_lane_depth_window(const LaneDepthWindow & window) noexcept
{
  if (window.lane_id.empty()) {
    return false;
  }
  const bool finite = std::isfinite(window.half_width_m) &&
    std::isfinite(window.rear_clearance_m) && std::isfinite(window.usable_depth_m) &&
    std::isfinite(window.usable_height_m) && std::isfinite(window.bed_datum_depth_m) &&
    std::isfinite(window.bed_slope) && std::isfinite(window.floor_margin_m) &&
    std::isfinite(window.front_inset_m) && std::isfinite(window.obstruction_reach_m);
  if (!finite) {
    return false;
  }
  // The product window runs from the rear entrance to the retainer face, so the retainer has to
  // stand in front of the entrance or there is no window at all.
  return window.half_width_m > 0.0 && window.rear_clearance_m >= 0.0 &&
         window.usable_depth_m > 0.0 && window.usable_height_m > 0.0 && window.bed_slope >= 0.0 &&
         window.floor_margin_m > 0.0 && window.obstruction_reach_m >= 0.0 &&
         window.front_inset_m >= 0.0 &&
         window.window_front_depth_m() > window.rear_clearance_m;
}

bool valid_lane_depth_config(const LaneDepthConfig & config) noexcept
{
  return std::isfinite(config.minimum_depth_m) && std::isfinite(config.maximum_depth_m) &&
         config.minimum_depth_m > 0.0 && config.maximum_depth_m > config.minimum_depth_m &&
         config.pixel_stride > 0U && std::isfinite(config.bin_width_m) &&
         config.bin_width_m > 0.0 && config.minimum_surface_bin_returns > 0U;
}

DepthObstacleResult<LaneDepthWindow> lane_depth_window(
  const LaneSurveyGeometry & lane, const ShelfSurveyGeometry & shelf, double floor_margin_m,
  double front_inset_m, double obstruction_reach_m)
{
  LaneDepthWindow window;
  window.lane_id = lane.lane_id;
  // The dividers stand 0.01 m outside this band on the shipped workcell, the clear span between
  // consecutive dividers is 0.375 m against a 0.365 m usable width, so insetting by the same
  // side clearance the manipulation geometry uses keeps a divider face out of the histogram
  // without narrowing the band so far that a product against one side falls out of it.
  window.half_width_m = (lane.usable_width_m / 2.0) - lane.side_clearance_m;
  window.rear_clearance_m = lane.rear_clearance_m;
  window.usable_depth_m = lane.usable_depth_m;
  window.usable_height_m = lane.usable_height_m;
  window.bed_datum_depth_m = shelf.bed_datum_depth_m;
  window.bed_slope = std::tan(shelf.incline_rad);
  window.floor_margin_m = floor_margin_m;
  window.front_inset_m = front_inset_m;
  window.obstruction_reach_m = obstruction_reach_m;
  if (!valid_lane_depth_window(window)) {
    return DepthObstacleResult<LaneDepthWindow>::failure(
      invalid_configuration("lane " + lane.lane_id + " does not yield a usable depth window"));
  }
  return DepthObstacleResult<LaneDepthWindow>::success(std::move(window));
}

DepthObstacleResult<LaneDepthMeasurement> measure_lane_depth(
  std::span<const float> depth, std::uint32_t row_step_pixels,
  const DepthCameraIntrinsics & intrinsics, const Eigen::Isometry3d & lane_from_optical,
  const LaneDepthWindow & window, const LaneDepthConfig & config)
{
  if (!valid_lane_depth_window(window)) {
    return MeasurementResult::failure(invalid_configuration("lane depth window is not usable"));
  }
  if (!valid_lane_depth_config(config)) {
    return MeasurementResult::failure(
      invalid_configuration("lane depth configuration is not usable"));
  }
  if (!valid_intrinsics(intrinsics)) {
    return MeasurementResult::failure(
      DepthObstacleError{
        DepthObstacleErrorCode::InvalidIntrinsics, "depth camera intrinsics are not usable"});
  }
  if (row_step_pixels < intrinsics.width) {
    return MeasurementResult::failure(
      DepthObstacleError{
        DepthObstacleErrorCode::InvalidDepthBuffer, "depth row step is narrower than the image"});
  }
  const std::size_t required =
    static_cast<std::size_t>(row_step_pixels) * static_cast<std::size_t>(intrinsics.height);
  if (depth.size() < required) {
    return MeasurementResult::failure(
      DepthObstacleError{
        DepthObstacleErrorCode::InvalidDepthBuffer,
        "depth buffer is shorter than its declared geometry"});
  }
  if (!lane_from_optical.matrix().allFinite()) {
    return MeasurementResult::failure(
      DepthObstacleError{
        DepthObstacleErrorCode::InvalidTransform, "lane <- optical transform is not finite"});
  }

  // The histogram runs from the rear entrance plane to just short of the retainer face.
  // Everything at or past that face belongs to the retainer, not to the lane's contents.
  const double window_front_m = window.window_front_depth_m();
  const double histogram_span_m = window_front_m - window.rear_clearance_m;
  const std::size_t bin_count = std::max<std::size_t>(
    1U, static_cast<std::size_t>(std::ceil(histogram_span_m / config.bin_width_m)));

  std::vector<std::size_t> product_returns_per_bin(bin_count, 0U);
  std::vector<std::size_t> any_returns_per_bin(bin_count, 0U);
  std::vector<double> minimum_depth_per_bin(bin_count, window_front_m);

  LaneDepthMeasurement measurement;
  measurement.total_bins = bin_count;

  const double obstruction_rear_m = window.rear_clearance_m - window.obstruction_reach_m;

  for (std::uint32_t row = 0U; row < intrinsics.height; row += config.pixel_stride) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * row_step_pixels;
    for (std::uint32_t column = 0U; column < intrinsics.width; column += config.pixel_stride) {
      ++measurement.sampled_pixels;
      const float sample = depth[row_offset + column];
      if (!std::isfinite(sample) || sample <= 0.0F) {
        continue;
      }
      const double range = static_cast<double>(sample);
      if (range < config.minimum_depth_m || range > config.maximum_depth_m) {
        continue;
      }
      ++measurement.valid_depth_samples;
      const Eigen::Vector3d in_optical(
        (static_cast<double>(column) - intrinsics.cx) * range / intrinsics.fx,
        (static_cast<double>(row) - intrinsics.cy) * range / intrinsics.fy, range);
      const Eigen::Vector3d in_lane = lane_from_optical * in_optical;
      if (std::abs(in_lane.x()) > window.half_width_m) {
        continue;
      }
      const double height_above_bed = in_lane.z() - window.bed_height_at(in_lane.y());
      // Above the lane, or below its floor by more than the floor band itself. The second half
      // is not pedantry: a return under the bed cannot be the lane's contents and cannot be its
      // floor either, so counting it as coverage would credit the measurement for seeing
      // something that is not the lane.
      if (height_above_bed > window.usable_height_m ||
        height_above_bed < -window.floor_margin_m)
      {
        continue;
      }
      const bool above_floor_band = height_above_bed >= window.floor_margin_m;

      if (in_lane.y() >= window.rear_clearance_m && in_lane.y() <= window_front_m) {
        const std::size_t bin = std::min(
          bin_count - 1U,
          static_cast<std::size_t>((in_lane.y() - window.rear_clearance_m) / config.bin_width_m));
        ++any_returns_per_bin[bin];
        if (above_floor_band) {
          ++measurement.product_returns;
          ++product_returns_per_bin[bin];
          minimum_depth_per_bin[bin] = std::min(minimum_depth_per_bin[bin], in_lane.y());
        } else {
          ++measurement.bed_returns;
        }
        continue;
      }
      const bool behind_the_entrance =
        in_lane.y() >= obstruction_rear_m && in_lane.y() < window.rear_clearance_m;
      if (behind_the_entrance && above_floor_band) {
        ++measurement.obstruction_returns;
      }
    }
  }

  const std::size_t accepted = measurement.product_returns + measurement.bed_returns;
  for (std::size_t bin = 0U; bin < bin_count; ++bin) {
    if (product_returns_per_bin[bin] >= config.minimum_surface_bin_returns) {
      measurement.nearest_surface_found = true;
      measurement.nearest_surface_depth_m = minimum_depth_per_bin[bin];
      break;
    }
  }

  measurement.available_depth_m = measurement.nearest_surface_found ?
    std::clamp(
    measurement.nearest_surface_depth_m - window.rear_clearance_m, 0.0, window.usable_depth_m) :
    window.usable_depth_m;
  measurement.obstructed = measurement.obstruction_returns >= config.minimum_obstruction_returns;

  // Coverage. A bin counts when something landed in it, or when it lies in front of the nearest
  // surface and is therefore in that surface's shadow. When no surface was found nothing is in
  // shadow, so an acquisition that saw nothing gets no credit for the emptiness it reports.
  for (std::size_t bin = 0U; bin < bin_count; ++bin) {
    const double bin_start_m =
      window.rear_clearance_m + (static_cast<double>(bin) * config.bin_width_m);
    const bool shadowed = measurement.nearest_surface_found &&
      bin_start_m >= measurement.nearest_surface_depth_m;
    if (any_returns_per_bin[bin] > 0U || shadowed) {
      ++measurement.covered_bins;
    }
  }
  measurement.coverage = accepted >= config.minimum_accepted_returns ?
    static_cast<double>(measurement.covered_bins) / static_cast<double>(bin_count) :
    0.0;
  return MeasurementResult::success(std::move(measurement));
}

}  // namespace restocker_perception
