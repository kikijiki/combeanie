// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <span>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "restocker_perception/depth_obstacle_extraction.hpp"
#include "restocker_perception/survey_stations.hpp"

namespace restocker_perception
{

// How far back a lane's product column reaches, measured from the robot's own side.
//
// This is a depth histogram in a box: no detector, no network. Two pieces of geometry must be
// handled correctly.
//
// The bed. The lane floor is a gravity-feed roller surface inclined at `shelf.lane_incline_deg`,
// passing through the lane frame's zero height at the retainer face and rising behind it (0.059 m
// at the shelf's rear lip on the shipped workcell). A constant-height floor threshold would accept
// the bed itself over the rear two thirds of every lane, so the nearest surface would be the bed
// at the lane mouth and every lane would read as full. The floor test is therefore a height above
// the bed at that depth, and the incline is a required input.
//
// The retainer. It stands at the bed datum depth, 0.09 m tall, with its rear face in view from
// every lane station. Inside a naive window it is a surface at lane depth 0.84 well clear of the
// bed, so it reads as the nearest product: an empty lane measured 0.830 m of free depth against a
// true 0.850. The product window therefore stops short of that face by `front_inset_m`.

// The lane volume a measurement reads, in the lane's own frame. Built from the workcell geometry
// by `lane_depth_window`; the fields are public so a test can state one directly.
struct LaneDepthWindow
{
  std::string lane_id;
  // Half-width of the accepted band about the lane centre line, already inset by
  // side_clearance_m so the dividers are outside it.
  double half_width_m{0.0};
  // Lane depth of the rear entrance plane. `available_depth_m` is measured from here.
  double rear_clearance_m{0.0};
  // Full usable depth, and therefore the value reported for a lane seen to be empty.
  double usable_depth_m{0.0};
  // Ceiling of the accepted band, measured above the bed rather than above the lane frame's zero.
  double usable_height_m{0.0};
  // Lane depth at which the bed is at height zero. The retainer stands here.
  double bed_datum_depth_m{0.0};
  // How far behind the retainer face the product window stops.
  //
  // At zero the retainer's rear face (0.09 m tall, in view of every lane station, at exactly the
  // lane depth the window would end at) reads as the nearest product and every empty lane reports
  // 0.83 m of free depth instead of 0.85. Nothing legitimate is lost by stepping back, because the
  // frontmost product a lane can hold has its rear face one full diameter behind the retainer
  // (0.066 m for the narrowest catalogued product).
  double front_inset_m{0.0};
  // Rise of the bed per metre of depth behind the datum. tan(incline).
  double bed_slope{0.0};
  // How far above the bed a return has to be before it can be a product rather than the bed.
  double floor_margin_m{0.0};
  // How far behind the rear entrance plane to look for a protrusion out of the lane mouth.
  double obstruction_reach_m{0.0};

  // Height of the roller surface, in the lane frame, at a given lane depth.
  [[nodiscard]] double bed_height_at(double lane_depth_m) const noexcept;
  // Front boundary of the product window: the retainer face, less the inset.
  [[nodiscard]] double window_front_depth_m() const noexcept;
};

// Thresholds and sampling. See lane_depth_measurement.cpp beside each default.
struct LaneDepthConfig
{
  double minimum_depth_m{0.10};
  double maximum_depth_m{2.40};
  std::uint32_t pixel_stride{2U};
  // Width of the depth histogram's bins. Coverage and the search for the nearest surface are
  // counted in these; the reported depth is the least lane depth of the returns inside the bin and
  // is not quantised by it.
  //
  // The lower bound is how far apart consecutive sampled image rows land on the bed at the far end
  // of the lane, since a bin narrower than that gap has no returns however well the lane was seen.
  // From the shipped station the bed's far end is 1.49 m away, seen at 21.6 degrees of grazing, so
  // one image row is (1.49 / 700.9) / sin(21.6) = 5.8 mm of lane depth and `pixel_stride` of 2
  // makes it 11.6 mm. At a 10 mm bin an empty lane measured 80 of 83 bins covered (the three at
  // the far end, as predicted); 0.020 is that gap with most of a bin of margin.
  double bin_width_m{0.020};
  // Product returns a bin needs before it is believed to be a surface rather than speckle.
  std::size_t minimum_surface_bin_returns{12U};
  // Returns a measurement needs in total before it is worth reading at all.
  std::size_t minimum_accepted_returns{200U};
  // Returns behind the rear entrance plane before `obstructed` is raised.
  std::size_t minimum_obstruction_returns{40U};
};

struct LaneDepthMeasurement
{
  // Free depth from the lane's rear entrance to the nearest surface, clamped to the lane. When
  // no surface was found this is the full usable depth (an empty lane) and `coverage` is the only
  // thing that says whether that reading was earned.
  double available_depth_m{0.0};
  bool obstructed{false};
  // Fraction of the lane's depth, from the rear entrance forward, that this acquisition
  // accounted for: a bin counts when a return landed in it, or when it lies in front of the
  // nearest surface and is therefore legitimately in that surface's shadow.
  //
  // This separates "I looked and the lane is empty" from "I could not see", so available_depth_m
  // alone must never be believed. An empty lane and a lane the camera never resolved report the
  // same free depth (the full lane); the first is backed by a bed return in every bin and the
  // second by none.
  //
  // The shadow clause: without it a camera that saw the first 0.10 m of a lane and nothing beyond
  // would be credited for the whole lane once it found a surface. With it, a bin is excused only
  // when a surface was measured in front of it.
  //
  // Limits. It catches a sensor that returns nothing or too little, one aimed elsewhere, and one
  // whose view into the lane is blocked part of the way down, since all of them leave bins dark.
  // It cannot catch a surface the depth stream passes through, and no depth-based coverage measure
  // can: a ray through a transparent object that lands on the bed behind it is indistinguishable
  // from a ray across empty space. Measured: one can at mid-lane returning no depth reads as an
  // empty lane at coverage 1.000. A see-through product is therefore a limit of this producer, not
  // of its confidence, and at the rear of a packed column the error is bounded to one product
  // pitch, because everything in front of the missed product is still measured.
  double coverage{0.0};
  bool nearest_surface_found{false};
  double nearest_surface_depth_m{0.0};
  std::size_t product_returns{0U};
  std::size_t bed_returns{0U};
  std::size_t obstruction_returns{0U};
  std::size_t covered_bins{0U};
  std::size_t total_bins{0U};
  // Samples that reached the window test at all, before it. Reported so a coverage of zero can be
  // told apart from a depth image that was never unprojected.
  std::size_t valid_depth_samples{0U};
  std::size_t sampled_pixels{0U};
};

[[nodiscard]] bool valid_lane_depth_window(const LaneDepthWindow & window) noexcept;
[[nodiscard]] bool valid_lane_depth_config(const LaneDepthConfig & config) noexcept;

// Build the window for one lane from the workcell's own numbers.
//
// `floor_margin_m`, `front_inset_m` and `obstruction_reach_m` are properties of the measurement,
// not the cell, so they are passed here rather than read from workcell_geometry.yaml.
[[nodiscard]] DepthObstacleResult<LaneDepthWindow> lane_depth_window(
  const LaneSurveyGeometry & lane, const ShelfSurveyGeometry & shelf, double floor_margin_m,
  double front_inset_m, double obstruction_reach_m);

// Reproject a 32FC1 depth buffer into the lane frame and measure the gap at the rear.
//
// `lane_from_optical` must be the camera's pose in the lane's frame at the depth image's own
// stamp. This function cannot check that; it is the caller's obligation, as for
// extract_obstacle_boxes.
//
// `depth` is row-major with `row_step_pixels` elements per row. Non-finite and non-positive
// samples are no return, as REP-118 requires. A buffer of only such samples is a blinded camera,
// answered with coverage zero rather than an empty lane.
[[nodiscard]] DepthObstacleResult<LaneDepthMeasurement> measure_lane_depth(
  std::span<const float> depth, std::uint32_t row_step_pixels,
  const DepthCameraIntrinsics & intrinsics, const Eigen::Isometry3d & lane_from_optical,
  const LaneDepthWindow & window, const LaneDepthConfig & config);

}  // namespace restocker_perception
