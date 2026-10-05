// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/survey_stations.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include <algorithm>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace restocker_perception
{
namespace
{

[[nodiscard]] double required_double(
  const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key]) {
    throw std::invalid_argument(context + " is missing " + key);
  }
  return node[key].as<double>();
}

[[nodiscard]] Eigen::Vector3d required_vector3(
  const YAML::Node & node, const char * key, const std::string & context)
{
  if (!node[key] || !node[key].IsSequence() || node[key].size() != 3U) {
    throw std::invalid_argument(context + " requires a three-element " + key);
  }
  return Eigen::Vector3d(
    node[key][0].as<double>(), node[key][1].as<double>(), node[key][2].as<double>());
}

[[nodiscard]] std::string station_index_name(std::size_t index)
{
  std::ostringstream stream;
  stream << "tray_" << (index + 1U);
  return stream.str();
}

}  // namespace

WorkcellSurveyGeometry load_workcell_survey_geometry(
  const std::filesystem::path & workcell_geometry_path)
{
  if (!std::filesystem::exists(workcell_geometry_path)) {
    throw std::invalid_argument(
            "workcell geometry not found at " + workcell_geometry_path.string());
  }
  const YAML::Node root = YAML::LoadFile(workcell_geometry_path.string());
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("workcell geometry requires schema_version 1");
  }
  if (!root["lanes"] || !root["lanes"].IsMap() || root["lanes"].size() == 0U) {
    throw std::invalid_argument("workcell geometry must contain a lane map");
  }
  if (!root["stock_tray"] || !root["stock_tray"]["usable_volume"]) {
    throw std::invalid_argument("workcell geometry must contain stock_tray.usable_volume");
  }

  WorkcellSurveyGeometry geometry;
  for (const auto & entry : root["lanes"]) {
    const std::string id = entry.first.as<std::string>();
    const std::string context = "workcell geometry lane " + id;
    LaneSurveyGeometry lane;
    lane.lane_id = id;
    lane.center_x_m = required_double(entry.second, "center_x_m", context);
    lane.usable_width_m = required_double(entry.second, "usable_width_m", context);
    lane.usable_depth_m = required_double(entry.second, "usable_depth_m", context);
    lane.usable_height_m = required_double(entry.second, "usable_height_m", context);
    lane.rear_clearance_m = required_double(entry.second, "rear_clearance_m", context);
    lane.side_clearance_m = required_double(entry.second, "side_clearance_m", context);
    lane.floor_clearance_m = required_double(entry.second, "floor_clearance_m", context);
    lane.floor_settle_tolerance_m =
      required_double(entry.second, "floor_settle_tolerance_m", context);
    geometry.lanes.push_back(std::move(lane));
  }
  // yaml-cpp preserves file order for a map, but nothing in the format guarantees it and a
  // station's identity is its lane, not its position in a list. Sorting by centre makes "the
  // station left of this one" mean the same thing however the file is written.
  std::sort(
    geometry.lanes.begin(), geometry.lanes.end(),
    [](const LaneSurveyGeometry & left, const LaneSurveyGeometry & right) {
      return left.center_x_m < right.center_x_m;
    });

  const YAML::Node volume = root["stock_tray"]["usable_volume"];
  geometry.tray.usable_center_in_shelf =
    required_vector3(volume, "center_xyz_m", "workcell geometry stock_tray.usable_volume");
  geometry.tray.usable_size =
    required_vector3(volume, "size_xyz_m", "workcell geometry stock_tray.usable_volume");

  if (!root["shelf"]) {
    throw std::invalid_argument("workcell geometry must contain a shelf block");
  }
  const YAML::Node shelf = root["shelf"];
  const std::string shelf_context = "workcell geometry shelf";
  const double shelf_depth = required_double(shelf, "depth_m", shelf_context);
  const double retainer_depth = required_double(shelf, "front_retainer_depth_m", shelf_context);
  geometry.shelf.bed_datum_depth_m = shelf_depth - retainer_depth;
  geometry.shelf.incline_rad =
    required_double(shelf, "lane_incline_deg", shelf_context) * M_PI / 180.0;
  geometry.shelf.front_retainer_height_m =
    required_double(shelf, "front_retainer_height_m", shelf_context);
  if (!std::isfinite(geometry.shelf.bed_datum_depth_m) ||
    geometry.shelf.bed_datum_depth_m <= 0.0 || !std::isfinite(geometry.shelf.incline_rad) ||
    geometry.shelf.incline_rad < 0.0 || geometry.shelf.incline_rad >= M_PI_2)
  {
    throw std::invalid_argument(
            "workcell geometry shelf bed datum depth must be positive and the lane incline must "
            "be a non-negative angle below a right angle");
  }
  return geometry;
}

const LaneSurveyGeometry * find_lane_geometry(
  const WorkcellSurveyGeometry & geometry, const std::string & lane_id)
{
  const auto match = std::find_if(
    geometry.lanes.begin(), geometry.lanes.end(),
    [&lane_id](const LaneSurveyGeometry & lane) {return lane.lane_id == lane_id;});
  return match == geometry.lanes.end() ? nullptr : &*match;
}

std::vector<SurveyStation> nominal_survey_stations(
  const WorkcellSurveyGeometry & geometry, const SurveyStationConfig & config)
{
  if (geometry.lanes.empty()) {
    throw std::invalid_argument("no lanes to survey");
  }
  if (config.tray_station_count == 0U) {
    throw std::invalid_argument("at least one tray station is required");
  }
  if ((geometry.tray.usable_size.array() <= 0.0).any()) {
    throw std::invalid_argument("the stock tray usable volume must have positive extent");
  }

  std::vector<SurveyStation> stations;
  stations.reserve(geometry.lanes.size() + config.tray_station_count);

  // --- Lane stations ---------------------------------------------------------------------
  //
  // One station per lane, and that is not a design choice. The dividers stand 0.42 m above the
  // support and run the full lane depth, so a camera offset laterally from a lane's centre line
  // shadows the near part of that lane's floor by divider_height * offset / camera_height: at one
  // 0.4 m lane pitch and a 0.55 m camera that is 0.305 m of a 0.365 m usable width. Raising the
  // camera does not rescue it, covering a 0.4 m offset with a 0.05 m shadow needs the camera
  // 4.1 m up, so every lane is surveyed from over its own centre line.
  //
  // The camera stands behind the shelf's rear edge and looks down at mid-lane depth. Aiming at
  // the middle of the lane rather than at its rear entrance is what keeps the whole usable volume
  // inside the vertical field: the lane subtends about 0.45 rad from this station against a
  // vertical half-angle of 0.474 rad, so there is no room to waste on either side.
  //
  // The height remains 0.55 m rather than the specification's 0.85 m. The UR10e replacement has
  // 0.6127 m and 0.57155 m main links (1.18425 m combined) and a published 1.30 m reach. That
  // preserves the old 1.20 m two-link reach class while fitting the fixed rail-to-lane stations;
  // UR3e's 0.50 m and UR5e's 0.85 m reach do not. The lower camera pose also retains the measured
  // 61 px framing margin. Re-tuning survey trajectories is deliberately separate from the model
  // swap; this function only preserves the already shipped camera target.
  for (const LaneSurveyGeometry & lane : geometry.lanes) {
    SurveyStation station;
    station.name = lane.lane_id;
    const Eigen::Vector3d lane_origin_in_shelf(lane.center_x_m, 0.0, 0.0);
    const Eigen::Vector3d eye =
      lane_origin_in_shelf +
      Eigen::Vector3d(0.0, -config.lane_camera_setback_m, config.lane_camera_height_m);
    const double mid_depth = lane.rear_clearance_m + (lane.usable_depth_m / 2.0);
    const Eigen::Vector3d target = lane_origin_in_shelf + Eigen::Vector3d(0.0, mid_depth, 0.0);
    // Image-right along the shelf's +X puts the lane's depth on the image's short axis, which is
    // the axis with room for it, and its width on the long axis. The up axis (shelf +Z) pins the
    // roll upright; for this +Y-boresight stance that lands image-right on +X exactly as the
    // comment says — Card 098's corrected contract (specs/camera-viewpoint-orientation.md).
    const Result<Eigen::Isometry3d> pose =
      look_at_pose(eye, target, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    if (!pose) {
      throw std::invalid_argument(
              "lane " + lane.lane_id + " station is degenerate: " + pose.error().detail);
    }
    station.shelf_from_optical = pose.value();
    // The region to be framed is the lane's whole usable volume. The specification names the rear
    // cross-section plus the floor band in front of it; the whole volume contains both and is
    // what the depletion measurement of section 3 actually reads, so framing the smaller region
    // would be framing something no consumer looks at.
    const Eigen::Vector3d volume_center =
      lane_origin_in_shelf +
      Eigen::Vector3d(
      0.0, lane.rear_clearance_m + (lane.usable_depth_m / 2.0), lane.usable_height_m / 2.0);
    const Eigen::Vector3d volume_size(
      lane.usable_width_m, lane.usable_depth_m, lane.usable_height_m);
    station.target_points_in_shelf = box_corners(volume_center, volume_size);
    station.nominal_range_m = (volume_center - eye).norm();
    stations.push_back(std::move(station));
  }

  // --- Tray stations ---------------------------------------------------------------------
  //
  // Image long axis along the shelf's X, because the binding framing constraint is the tray's
  // 0.51 m depth against the short image axis and the long axis is what has slack. One station
  // cannot cover the tray's 1.36 m width from a range that also fits its depth, so the width is
  // split into equal slices and each is surveyed from over its own centre.
  //
  // The specification asks for these looked at straight down from world z 1.42. That pose is not
  // reachable and cannot be made reachable by moving it up or down: framing the 0.51 m depth sets
  // a floor on the range, the arm's upper-arm-plus-forearm span sets a ceiling on the height, and
  // on this arm the two do not overlap. The refusal is confirmed on the shipped arm: straight
  // down above the tray centre at (x, -0.80, 1.42) has no collision-free IK solution from any of
  // 60 seeds, both slices
  // (`tools/diagnostics/planning_probe --mode envelope --orientation straightdown`).
  //
  // What works is tilting the view back toward the shoulder rather than lowering it. Where
  // exactly is not something a reach model settles, an approximate one put a station at
  // (x, -0.20, 1.25) that move_group then refused from every seed, so the envelope has to be
  // measured rather than derived. The first measurement, committed 2026-09-10 (`ba05416`), swept
  // `/compute_ik` over a grid of camera poses aimed at the slice centre and recorded "solutions
  // exist for camera y in [-0.45, -0.25] and z in [1.15, 1.35], and nowhere else". That sweep's
  // program was never committed, its orientation convention was never recorded, and it predates
  // the 2026-09-13 UR10e swap — it measured the retired custom arm, so it is unrecoverable as a
  // claim about the shipped robot, and re-running the grid under either candidate convention does
  // not reproduce the block.
  //
  // The shipped arm's envelope comes from the committed probe instead, under the convention this
  // file ships: optical at (slice_x, y, z), boresight at the slice centre, image-right +X
  // (look_at_pose above), mapped to a tool0 goal through the fixed wrist-camera mount as
  // WristCameraMount::tool0_goal_for does, solved as collision-aware IK on `manipulator` at tip
  // tool0 with the planner's padding, 60 seeds. At both slice centres every camera y in
  // [-0.65, +0.05] with z in [1.00, 1.40] solves (60/60 or near, zero scene refusals); the only
  // failures are kinematic and live in the far-and-high corner. The frontier as pairs (y, first
  // failing z): (-0.65, 1.45), (-0.60, 1.50), (-0.525, 1.55) — equivalently the in-grid failing
  // ranges z 1.45 at y {-0.65, -0.625}, z 1.50 at y <= -0.55, z 1.55 at y <= -0.425. The old
  // block sits inside that region with margin on every side, so a station may move well past it
  // as long as z stays at or below 1.40.
  // The station below sits inside the measured envelope with room on every side.
  //
  // Two consequences belong to whoever consumes these. The ground sample improves, 0.60 to
  // 1.34 mm/px against the 1.2 to 1.8 the specification derived from a 2.06 m fixed camera. The
  // elevation does not: 51 degrees rather than 90, so a top-face disc projects to an ellipse of
  // axis ratio sin(51) = 0.78 and ColourDepthBackend's radius-ratio bands, depth band and pixel
  // floor had to be re-derived for the wrist tray duties. That re-derivation is
  // config/wrist_tray_perception.yaml, pinned by test_tray_duty_parameters.
  const Eigen::Vector3d tray_center = geometry.tray.usable_center_in_shelf;
  const Eigen::Vector3d tray_size = geometry.tray.usable_size;
  const double slice_width = tray_size.x() / static_cast<double>(config.tray_station_count);
  for (std::size_t index = 0U; index < config.tray_station_count; ++index) {
    SurveyStation station;
    station.name = station_index_name(index);
    const double slice_center_x = tray_center.x() - (tray_size.x() / 2.0) +
      (slice_width * (static_cast<double>(index) + 0.5));
    const Eigen::Vector3d target(slice_center_x, tray_center.y(), tray_center.z());
    // +Y in the shelf frame is toward the shelf, which is where the robot is: the tray sits at
    // negative Y and the camera stands between the two.
    const Eigen::Vector3d eye =
      target + Eigen::Vector3d(0.0, config.tray_camera_setback_m, config.tray_camera_height_m);
    // Card 098 (specs/camera-viewpoint-orientation.md): the shelf +Z up axis pins the roll
    // upright. This station looks from +Y_shelf toward -Y_shelf, where the old +X-hint-only
    // contract produced the 180-degree-rolled image-right (+X_shelf, upside-down view); upright
    // lands image-right on -X_shelf — still the shelf's X axis, so the long-axis framing intent
    // above is untouched, and eye/boresight/elevation/framing are all unchanged.
    const Result<Eigen::Isometry3d> pose =
      look_at_pose(eye, target, Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitZ());
    if (!pose) {
      throw std::invalid_argument(
              "tray station " + station.name + " is degenerate: " + pose.error().detail);
    }
    station.shelf_from_optical = pose.value();
    const Eigen::Vector3d slice_center(slice_center_x, tray_center.y(), tray_center.z());
    const Eigen::Vector3d slice_size(slice_width, tray_size.y(), tray_size.z());
    station.target_points_in_shelf = box_corners(slice_center, slice_size);
    station.nominal_range_m = (slice_center - eye).norm();
    stations.push_back(std::move(station));
  }

  return stations;
}

const SurveyStation * find_survey_station(
  const std::vector<SurveyStation> & stations, const std::string & name)
{
  const auto match = std::find_if(
    stations.begin(), stations.end(),
    [&name](const SurveyStation & station) {return station.name == name;});
  return match == stations.end() ? nullptr : &*match;
}

Result<CameraViewpoint> station_viewpoint(
  const SurveyStation & station, const FramedTransform & reference_from_shelf)
{
  if (reference_from_shelf.source_frame() != kShelfFrame) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::FrameMismatch,
        "survey stations are stated in " + std::string(kShelfFrame) + ", not in " +
        reference_from_shelf.source_frame()});
  }
  CameraViewpoint viewpoint;
  viewpoint.label = station.name;
  viewpoint.pose.frame_id = reference_from_shelf.target_frame();
  viewpoint.pose.pose = reference_from_shelf.transform() * station.shelf_from_optical;
  return Result<CameraViewpoint>::success(std::move(viewpoint));
}

namespace
{

[[nodiscard]] bool finite_positive(double value) noexcept
{
  return std::isfinite(value) && value > 0.0;
}

// One standpoint: the eye stands `standoff_m` from the AIM point along the approach direction,
// the boresight lands on the aim, and the framing test is made against the box around the
// candidate itself — the aim and the framed region are deliberately different points.
[[nodiscard]] Result<CameraViewpoint> confirm_at_standoff(
  const Eigen::Vector3d & aim, const Eigen::Vector3d & framing_center,
  const Eigen::Vector3d & approach_in_reference, const Eigen::Vector3d & right_in_reference,
  double standoff_m, const ConfirmationViewpointConfig & config,
  const FramedTransform & reference_from_shelf, const std::string & label)
{
  const Eigen::Vector3d eye = aim + (approach_in_reference * standoff_m);
  // The reference frame's up, the same vector the elevation check below derives: shelf +Z
  // carried through reference_from_shelf. look_at_pose pins the roll upright against it
  // (Card 098); right_in_reference stays the hint for the straight up/down fallback.
  const Eigen::Vector3d up_in_reference =
    reference_from_shelf.transform().rotation() * Eigen::Vector3d::UnitZ();
  const Result<Eigen::Isometry3d> pose =
    look_at_pose(eye, aim, right_in_reference, up_in_reference);
  if (!pose) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{PerceptionErrorCode::InvalidArgument, label + ": " + pose.error().detail});
  }

  // Elevation of the boresight below the horizontal plane of the reference frame, measured
  // through the shelf's own up axis so a rotated reference still means the same thing.
  const Eigen::Vector3d boresight = (aim - eye).normalized();
  const double elevation = std::asin(std::clamp(-boresight.dot(up_in_reference), -1.0, 1.0));
  if (elevation < config.minimum_elevation_rad) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
          PerceptionErrorCode::InvalidArgument,
          label + ": boresight elevation " + std::to_string(elevation) +
          " rad is below the configured floor " + std::to_string(config.minimum_elevation_rad) +
          " rad, so the top-face fit this viewpoint exists for cannot see a disc"});
  }

  const std::vector<Eigen::Vector3d> framing_points =
    box_corners(framing_center, config.framing_box_size);
  const FramingResult framing = frames_region(
    config.frustum, pose.value(), framing_points, config.required_margin_px);
  if (!framing.framed) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
          PerceptionErrorCode::InvalidArgument,
          label + ": the product does not frame at standoff " + std::to_string(standoff_m) +
          " m: " + framing.detail});
  }

  CameraViewpoint viewpoint;
  viewpoint.label = label;
  viewpoint.pose.frame_id = reference_from_shelf.target_frame();
  viewpoint.pose.pose = pose.value();
  return Result<CameraViewpoint>::success(std::move(viewpoint));
}

}  // namespace

Result<CameraViewpoint> confirmation_viewpoint(
  const Eigen::Vector3d & target_in_reference, const FramedTransform & reference_from_shelf,
  const ConfirmationViewpointConfig & config, const std::string & label)
{
  const std::string name = label.empty() ? std::string("confirm viewpoint") : label;
  if (reference_from_shelf.source_frame() != kShelfFrame) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::FrameMismatch,
        name + ": confirmation geometry is stated in " + std::string(kShelfFrame) + ", not in " +
        reference_from_shelf.source_frame()});
  }
  if (!target_in_reference.allFinite()) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{PerceptionErrorCode::InvalidArgument, name + ": target is not finite"});
  }
  if (!finite_positive(config.minimum_standoff_m) ||
    !finite_positive(config.maximum_standoff_m) || !finite_positive(config.standoff_m) ||
    config.standoff_m < config.minimum_standoff_m ||
    config.standoff_m > config.maximum_standoff_m)
  {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument,
        name + ": the commanded standoff must be a positive distance inside the configured "
        "band, and it is " + std::to_string(config.standoff_m) + " m in [" +
        std::to_string(config.minimum_standoff_m) + ", " +
        std::to_string(config.maximum_standoff_m) + "]"});
  }
  if (!finite_positive(config.minimum_elevation_rad) || config.minimum_elevation_rad >= M_PI_2) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument,
        name + ": the elevation floor must be a positive angle below a right angle"});
  }
  if (!config.frustum.valid()) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument,
        name + ": the sensor frustum the framing check needs is not populated"});
  }
  if (!finite_positive(config.required_margin_px)) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument, name + ": the framing margin must be positive"});
  }
  if (!config.framing_box_size.allFinite() || (config.framing_box_size.array() <= 0.0).any()) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument,
        name + ": the framing box must have positive extent"});
  }
  if (!config.approach_in_shelf.allFinite() || config.approach_in_shelf.norm() < 1.0e-9) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{
        PerceptionErrorCode::InvalidArgument, name + ": the approach direction is degenerate"});
  }
  if (!config.aim_bias_in_shelf.allFinite()) {
    return Result<CameraViewpoint>::failure(
      PerceptionError{PerceptionErrorCode::InvalidArgument, name + ": the aim bias is not finite"});
  }

  const Eigen::Isometry3d reference_from_shelf_transform = reference_from_shelf.transform();
  const Eigen::Vector3d approach_in_reference =
    (reference_from_shelf_transform.rotation() * config.approach_in_shelf).normalized();
  const Eigen::Vector3d right_in_reference =
    reference_from_shelf_transform.rotation() * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d aim =
    target_in_reference + (reference_from_shelf_transform.rotation() * config.aim_bias_in_shelf);

  // The commanded standoff first. The band's edges are fallbacks for framing, tried outward, so
  // a candidate near the edge of the image still gets a viewpoint that frames it rather than a
  // refusal the moment the nominal choice misses.
  std::vector<double> standoffs{config.standoff_m};
  if (config.standoff_m != config.maximum_standoff_m) {
    standoffs.push_back(config.maximum_standoff_m);
  }
  if (config.standoff_m != config.minimum_standoff_m) {
    standoffs.push_back(config.minimum_standoff_m);
  }
  std::string last_detail;
  for (const double standoff : standoffs) {
    Result<CameraViewpoint> attempt = confirm_at_standoff(
      aim, target_in_reference, approach_in_reference, right_in_reference, standoff, config,
      reference_from_shelf, name);
    if (attempt) {
      return attempt;
    }
    last_detail = attempt.error().detail;
  }
  return Result<CameraViewpoint>::failure(
    PerceptionError{PerceptionErrorCode::InvalidArgument, last_detail});
}

}  // namespace restocker_perception
