// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <filesystem>
#include <string>
#include <vector>

#include "restocker_perception/frame_geometry.hpp"
#include "restocker_perception/viewpoint_geometry.hpp"

namespace restocker_perception
{

// The frame every station below is expressed in: the workcell's own root link, so the stations
// move with the workcell.
inline constexpr const char * kShelfFrame = "shelf";

// The subset of workcell_geometry.yaml a survey needs. This package does not need divider
// collision thickness or roller pitch.
struct LaneSurveyGeometry
{
  std::string lane_id;
  double center_x_m{0.0};
  double usable_width_m{0.0};
  double usable_depth_m{0.0};
  double usable_height_m{0.0};
  double rear_clearance_m{0.0};
  // Read by the depletion measurement rather than the station table: separates the lane's
  // dividers from its contents, using the same inset as the manipulation geometry.
  double side_clearance_m{0.0};
  double floor_clearance_m{0.0};
  double floor_settle_tolerance_m{0.0};
};

// The parts of the shelf a lane measurement needs that are not per-lane. Both are properties of
// the gravity-fed bed, which is why a lane's floor is not a plane of constant height: the roller
// surface passes through zero at the retainer face and rises behind it.
struct ShelfSurveyGeometry
{
  // Lane depth at which the roller surface is at the lane frame's zero height:
  // shelf.depth_m - shelf.front_retainer_depth_m. The retainer stands here, so it is the front
  // boundary of the volume a product may occupy.
  double bed_datum_depth_m{0.0};
  double incline_rad{0.0};
  double front_retainer_height_m{0.0};
};

struct TraySurveyGeometry
{
  Eigen::Vector3d usable_center_in_shelf{Eigen::Vector3d::Zero()};
  Eigen::Vector3d usable_size{Eigen::Vector3d::Zero()};
};

struct WorkcellSurveyGeometry
{
  // Ordered by the file, which orders lanes left to right.
  std::vector<LaneSurveyGeometry> lanes;
  TraySurveyGeometry tray;
  ShelfSurveyGeometry shelf;
};

// The lane with this id, or nullptr.
[[nodiscard]] const LaneSurveyGeometry * find_lane_geometry(
  const WorkcellSurveyGeometry & geometry, const std::string & lane_id);

// Throws std::invalid_argument when the file is absent, the schema is wrong, or a lane is missing
// a field a station is derived from, rather than defaulting a station to zero.
[[nodiscard]] WorkcellSurveyGeometry load_workcell_survey_geometry(
  const std::filesystem::path & workcell_geometry_path);

// The parameters the nominal stations are derived from. The workcell does not state them, so they
// are here rather than in workcell_geometry.yaml, and each is justified where it is used in
// survey_stations.cpp.
//
// Every height below is set by the arm's reach, not by the sensor: the specification's own nominal
// poses are outside what this arm can hold. The arithmetic is in survey_stations.cpp beside the
// values that replace them.
struct SurveyStationConfig
{
  // Lane stations: the camera stands behind the shelf's rear edge and above the lane floor.
  //
  // The setback is measured from the lane frame's origin, the shelf's rear edge at the lane floor.
  // The shipped workcell stands 0.55 m in front of the rail, so this default puts the camera over
  // the rail line, where the arm can hold it. It is robot-relative expressed in the lane frame and
  // moves if the workcell moves.
  double lane_camera_setback_m{0.55};
  double lane_camera_height_m{0.55};

  // Tray stations: the camera stands this far behind the tray's usable centre (toward the robot)
  // and this far above it, looking down at it. This arm cannot hold a view straight down, so the
  // view is tilted back toward the shoulder.
  double tray_camera_setback_m{0.45};
  double tray_camera_height_m{0.555};
  // Tray coverage is split into this many equal slices along the shelf's X axis, one station
  // centred on each. Two is what the frustum needs; the number changes with the sensor.
  std::size_t tray_station_count{2U};
};

// One nominal survey station: where to put the camera, and what it has to see from there.
struct SurveyStation
{
  std::string name;
  // shelf <- wrist_camera_optical_frame. A camera pose, never a tool pose.
  Eigen::Isometry3d shelf_from_optical{Eigen::Isometry3d::Identity()};
  // The corners of the region this station must frame, in the shelf frame. Framing is tested per
  // point, and a rotated camera has no axis-aligned answer.
  std::vector<Eigen::Vector3d> target_points_in_shelf;
  // Straight-line range from the camera origin to the region's centroid. Reporting only.
  double nominal_range_m{0.0};
};

// The six lane stations and the tray stations, derived from the
// workcell's own numbers. Seeds for a solver, not surveyed constants.
//
// Throws std::invalid_argument when the geometry cannot produce a station (an empty lane list,
// a non-positive tray volume, a zero station count).
[[nodiscard]] std::vector<SurveyStation> nominal_survey_stations(
  const WorkcellSurveyGeometry & geometry, const SurveyStationConfig & config = {});

// The station with this name, or nullptr. Names are `lane_01` .. `lane_06` for lane stations and
// `tray_1` .. `tray_N` for tray stations, numbered along increasing shelf X.
[[nodiscard]] const SurveyStation * find_survey_station(
  const std::vector<SurveyStation> & stations, const std::string & name);

// Express a station as a viewpoint in another frame, given that frame's transform from the shelf.
//
// The result is a CameraViewpoint, not a bare pose, so it cannot reach a motion goal without going
// through the mount.
[[nodiscard]] Result<CameraViewpoint> station_viewpoint(
  const SurveyStation & station, const FramedTransform & reference_from_shelf);

// The parameters of a solved confirmation viewpoint. The confirmation pose is not tabulated like
// the overview stations: it is derived per candidate, and these are the bounds it is derived
// inside.
//
// Defaults reproduce the shipped tray stations' direction (tray_camera_setback_m,
// tray_camera_height_m) at the confirm duty's 0.25-0.35 m standoff band, which is the geometry
// config/wrist_tray_perception.yaml re-derives its radius-ratio bands against.
struct ConfirmationViewpointConfig
{
  // Commanded standoff along the approach direction. Must lie inside the band below.
  double standoff_m{0.30};
  double minimum_standoff_m{0.25};
  double maximum_standoff_m{0.35};
  // Direction from the target toward the eye, in the shelf frame. The tray stations' own
  // (0, setback, height): a 51-degree elevation, which is what keeps the product's top face
  // visible to the top-face fit the estimator performs.
  Eigen::Vector3d approach_in_shelf{Eigen::Vector3d(0.0, 0.45, 0.555)};
  // Minimum boresight depression below horizontal. At or above this the top-face disc is not
  // foreshortened past the radius bands measured for the shipped elevation; the floor exists so
  // a corrupted approach direction is refused rather than quietly aimed sideways.
  double minimum_elevation_rad{0.60};
  // Where the boresight lands, relative to the candidate's estimated position, in the shelf
  // frame. Centre-aiming at confirm standoff puts the product's near top corner outside the
  // frame: a 0.29 m bottle subtends more than the vertical field when half of it sits 0.19 m
  // from a camera 0.30 m away, and every standpoint in the band clips it. Biasing the aim up
  // the barrel by 0.05 m — still below the top face, still 51-degree elevation, top face still
  // visible — frames the largest catalogued product at every standoff in the band with margin
  // (measured: 35 to 126 px against the 30 px floor). The framing check below still tests the
  // box around the candidate itself; only the aim moves.
  Eigen::Vector3d aim_bias_in_shelf{Eigen::Vector3d(0.0, 0.0, 0.05)};
  // The wrist frustum the framing check is made against, from the sensor description or a live
  // CameraInfo. Never defaulted here: a framing computation against a hardcoded sensor would keep
  // passing after the sensor changed.
  SensorFrustum frustum{};
  // Every corner of the framing box must project at least this far inside the image. The same
  // 30 px floor the overview stations are checked against.
  double required_margin_px{30.0};
  // Conservative bounding box for the catalogued products, centred on the candidate's estimated
  // position: the largest product in the catalogue, so one admissible viewpoint frames them all.
  Eigen::Vector3d framing_box_size{0.09, 0.09, 0.29};
};

// Solve an admissible confirmation viewpoint for one candidate.
//
// `target_in_reference` is the candidate's estimated position in the same frame
// `reference_from_shelf` targets (the planning frame, for a candidate read off an observation).
// The eye stands `standoff_m` from the aim — the target shifted by `aim_bias_in_shelf`, which
// points the boresight up the product's barrel so the whole product frames at confirm range —
// along the shelf-frame approach direction rotated into that frame, and the image's +X follows
// the shelf's +X rotated into it: the same construction the tray stations use, at confirm
// range. Framing is tested against the box around the candidate itself, not around the aim.
//
// Admissibility here is the geometric half of the specification's three conditions: the standoff
// band, the elevation floor, and framing of the product's bounding box with a stated margin.
// Line of sight and reachability are discharged by the plan that carries the camera there —
// the same division of labour the overview stations rely on.
//
// Fails with InvalidArgument for a malformed config or a non-finite target, FrameMismatch when
// the transform does not start at the shelf frame, and a named failure when no standpoint in the
// band frames the product.
[[nodiscard]] Result<CameraViewpoint> confirmation_viewpoint(
  const Eigen::Vector3d & target_in_reference, const FramedTransform & reference_from_shelf,
  const ConfirmationViewpointConfig & config, const std::string & label = {});

}  // namespace restocker_perception
