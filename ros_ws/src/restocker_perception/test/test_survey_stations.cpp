// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "restocker_perception/survey_stations.hpp"
#include "restocker_perception/viewpoint_geometry.hpp"

namespace restocker_perception
{
namespace
{

[[nodiscard]] std::string required_environment(const char * name)
{
  const char * value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    throw std::runtime_error(std::string(name) + " must be set for this test");
  }
  return value;
}

// The wrist sensor's frustum, read out of the description rather than restated here.
//
// The declaration is literal XML inside the xacro, no expressions in these four fields, so a
// direct read is exact. If any of them ever becomes a xacro expression this stops matching and
// the test fails, which is the direction a framing check should fail in: loudly, rather than
// quietly carrying on with numbers that used to be true. The regressions this guards against are
// the ones that changed the sensor and left every other assertion passing.
[[nodiscard]] SensorFrustum wrist_frustum_from_description()
{
  const std::string path = required_environment("RESTOCKER_TEST_SENSORS_XACRO");
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("could not read the sensor description at " + path);
  }
  std::ostringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  const auto single = [&text](const char * pattern, const char * what) {
    std::smatch match;
    const std::regex expression(pattern);
    if (!std::regex_search(text, match, expression)) {
      throw std::runtime_error(
              std::string("the wrist camera declaration no longer states ") + what +
              " as a literal; the framing arithmetic cannot read it");
    }
    return std::stod(match[1].str());
  };

  SensorFrustum frustum;
  frustum.horizontal_fov_rad =
    single(R"(<horizontal_fov>\s*([0-9.eE+-]+)\s*</horizontal_fov>)", "horizontal_fov");
  frustum.width_px = static_cast<std::uint32_t>(
    single(R"(<width>\s*([0-9]+)\s*</width>)", "an image width"));
  frustum.height_px = static_cast<std::uint32_t>(
    single(R"(<height>\s*([0-9]+)\s*</height>)", "an image height"));
  frustum.near_clip_m = single(R"(<near>\s*([0-9.eE+-]+)\s*</near>)", "a near clip");
  frustum.far_clip_m = single(R"(<far>\s*([0-9.eE+-]+)\s*</far>)", "a far clip");
  return frustum;
}

[[nodiscard]] WorkcellSurveyGeometry shipped_geometry()
{
  return load_workcell_survey_geometry(required_environment("RESTOCKER_TEST_WORKCELL_GEOMETRY"));
}

// The shipped scenario stands the workcell here, and every generated scenario repeats it
// (restocker_gazebo/config/baseline_products.yaml, pinned by test_scenario_random.py). It is
// restated rather than loaded because this test is checking that the stations land where the
// specification's world-coordinate arithmetic says they do, and reading the same value the
// stations were derived from would only prove the file is self-consistent.
[[nodiscard]] Eigen::Isometry3d shipped_world_from_shelf()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.55, 0.75);
  return transform;
}

[[nodiscard]] Eigen::Vector3d station_origin_in_world(const SurveyStation & station)
{
  return shipped_world_from_shelf() * station.shelf_from_optical.translation();
}

TEST(SurveyStations, ProducesOneStationPerLaneAndTwoForTheTray)
{
  const std::vector<SurveyStation> stations = nominal_survey_stations(shipped_geometry());
  ASSERT_EQ(stations.size(), 8U);
  const std::vector<std::string> expected{
    "lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06", "tray_1", "tray_2"};
  for (std::size_t index = 0U; index < expected.size(); ++index) {
    EXPECT_EQ(stations[index].name, expected[index]);
    EXPECT_NE(find_survey_station(stations, expected[index]), nullptr);
  }
  EXPECT_EQ(find_survey_station(stations, "lane_07"), nullptr);
}

TEST(SurveyStations, PlacesEachLaneStationOverItsOwnCentreLine)
{
  // One station per lane is what the dividers force: they stand 0.42 m above the support and run
  // the full lane depth, so surveying lane N from over lane N+1 shadows 0.198 m of a 0.365 m
  // usable width. A station that drifted off its lane's centre line would be that failure.
  const WorkcellSurveyGeometry geometry = shipped_geometry();
  const std::vector<SurveyStation> stations = nominal_survey_stations(geometry);
  for (const LaneSurveyGeometry & lane : geometry.lanes) {
    const SurveyStation * station = find_survey_station(stations, lane.lane_id);
    ASSERT_NE(station, nullptr) << lane.lane_id;
    EXPECT_NEAR(station->shelf_from_optical.translation().x(), lane.center_x_m, 1.0e-12);
  }
}

TEST(SurveyStations, PutsTheLaneStationsOverTheRailAtAHeightTheArmCanHold)
{
  const std::vector<SurveyStation> stations = nominal_survey_stations(shipped_geometry());
  const SurveyStation * lane_01 = find_survey_station(stations, "lane_01");
  ASSERT_NE(lane_01, nullptr);
  const Eigen::Vector3d origin = station_origin_in_world(*lane_01);
  // Level with the rail, which is the arm's own y, and 0.55 m above the lane floor rather than
  // the specification's 0.85 m: at 0.85 the wrist needs 94% of the arm's two-link span and the
  // live test found no IK solution from any seed. See survey_stations.cpp for the arithmetic.
  EXPECT_NEAR(origin.x(), -1.0, 1.0e-9);
  EXPECT_NEAR(origin.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(origin.z(), 1.30, 1.0e-9);

  // The rail carries the carriage in X, so every lane station is the same arm attitude at a
  // different rail coordinate. Prove that rather than assume it.
  const SurveyStation * lane_06 = find_survey_station(stations, "lane_06");
  ASSERT_NE(lane_06, nullptr);
  EXPECT_LT(
    (lane_01->shelf_from_optical.linear() - lane_06->shelf_from_optical.linear())
    .cwiseAbs().maxCoeff(), 1.0e-12);
  const Eigen::Vector3d far_origin = station_origin_in_world(*lane_06);
  EXPECT_NEAR(far_origin.y(), origin.y(), 1.0e-12);
  EXPECT_NEAR(far_origin.z(), origin.z(), 1.0e-12);
  EXPECT_NEAR(far_origin.x(), 1.0, 1.0e-9);
}

TEST(SurveyStations, LooksDownTheWholeLaneFromTheHeightTheArmCanHold)
{
  const WorkcellSurveyGeometry geometry = shipped_geometry();
  const std::vector<SurveyStation> stations = nominal_survey_stations(geometry);
  const SurveyStation * station = find_survey_station(stations, "lane_03");
  ASSERT_NE(station, nullptr);
  const LaneSurveyGeometry & lane = geometry.lanes[2];
  ASSERT_EQ(lane.lane_id, "lane_03");

  const Eigen::Vector3d eye = station->shelf_from_optical.translation();
  const Eigen::Vector3d rear_entrance(lane.center_x_m, lane.rear_clearance_m, 0.0);
  const Eigen::Vector3d far_end(
    lane.center_x_m, lane.rear_clearance_m + lane.usable_depth_m, 0.0);
  const auto depression = [&eye](const Eigen::Vector3d & point) {
    const Eigen::Vector3d ray = point - eye;
    return std::atan2(-ray.z(), ray.y());
  };
  // The specification derives 56.6 and 31.1 degrees for a camera 0.85 m up; from the 0.55 m the
  // arm can actually hold, the same two rays are shallower and the lane subtends less.
  EXPECT_NEAR(depression(rear_entrance) * 180.0 / M_PI, 44.5, 0.1);
  EXPECT_NEAR(depression(far_end) * 180.0 / M_PI, 21.3, 0.1);
  EXPECT_NEAR((rear_entrance - eye).norm(), 0.785, 1.0e-3);
  // The lane's whole depth still has to fit the 0.474 rad vertical half-angle, which is why the
  // boresight is aimed at mid-lane and not at the entrance.
  EXPECT_NEAR(depression(rear_entrance) - depression(far_end), 0.405, 5.0e-3);
}

TEST(SurveyStations, SplitsTheTrayIntoTwoOverviewStationsThatTogetherCoverIt)
{
  const WorkcellSurveyGeometry geometry = shipped_geometry();
  const std::vector<SurveyStation> stations = nominal_survey_stations(geometry);
  const SurveyStation * left = find_survey_station(stations, "tray_1");
  const SurveyStation * right = find_survey_station(stations, "tray_2");
  ASSERT_NE(left, nullptr);
  ASSERT_NE(right, nullptr);

  const Eigen::Vector3d left_origin = station_origin_in_world(*left);
  const Eigen::Vector3d right_origin = station_origin_in_world(*right);
  EXPECT_NEAR(left_origin.x(), -0.34, 1.0e-9);
  EXPECT_NEAR(right_origin.x(), 0.34, 1.0e-9);
  // Tilted back toward the shoulder rather than straight down from world z 1.42, which the
  // committed probe confirms has no collision-free IK solution from any of 60 seeds on the
  // shipped arm, both slices (`tools/diagnostics/planning_probe --mode envelope --orientation
  // straightdown`). Inside the measured IK envelope under the shipped look-at convention
  // (same probe, `--mode envelope`): camera y in [-0.65, +0.05] with z in [1.00, 1.40] solves
  // at both slice centres, so this station sits inside it with room on every side rather than
  // at the specification's unreachable world z 1.42. (The older block recorded here — camera
  // y in [-0.45, -0.25], z in [1.15, 1.35] — was measured on the pre-UR10e arm with no
  // recorded orientation convention; see survey_stations.cpp.)
  EXPECT_NEAR(left_origin.y(), -0.35, 1.0e-9);
  EXPECT_NEAR(right_origin.y(), -0.35, 1.0e-9);
  EXPECT_NEAR(left_origin.z(), 1.30, 1.0e-9);
  EXPECT_NEAR(right_origin.z(), 1.30, 1.0e-9);

  // Boresight down and forward at 51 degrees, image long axis along the shelf's X. The elevation
  // is the number a top-face fit cares about: a disc at 51 degrees projects to an ellipse of
  // axis ratio 0.78, so the estimator's radius-ratio bands have to be re-derived before anything
  // fits a product from here.
  for (const SurveyStation * station : {left, right}) {
    const Eigen::Vector3d boresight = station->shelf_from_optical.linear() *
      Eigen::Vector3d::UnitZ();
    EXPECT_NEAR(std::asin(-boresight.z()) * 180.0 / M_PI, 51.0, 0.2) << station->name;
    EXPECT_NEAR(boresight.x(), 0.0, 1.0e-12) << station->name;
    EXPECT_LT(boresight.y(), 0.0) << station->name;
  }

  // The two target regions abut and together span the whole usable width.
  double lowest = std::numeric_limits<double>::max();
  double highest = std::numeric_limits<double>::lowest();
  for (const SurveyStation * station : {left, right}) {
    for (const Eigen::Vector3d & corner : station->target_points_in_shelf) {
      lowest = std::min(lowest, corner.x());
      highest = std::max(highest, corner.x());
    }
  }
  EXPECT_NEAR(
    lowest, geometry.tray.usable_center_in_shelf.x() -
    geometry.tray.usable_size.x() / 2.0, 1.0e-12);
  EXPECT_NEAR(
    highest, geometry.tray.usable_center_in_shelf.x() +
    geometry.tray.usable_size.x() / 2.0, 1.0e-12);
}

TEST(SurveyStations, FramesEveryStationsTargetRegionInTheDescribedSensor)
{
  // The measurement this whole stage exists to make safe. Every station's target region must
  // project entirely inside the image, and the margin is reported so a change that erodes it is
  // visible before it becomes a failure.
  //
  // 30 px is the floor rather than 0 because a station that only just frames its region has no
  // room for the pose error the arm actually achieves: 30 px is 8.3% of the image half-height,
  // and at the tray station's 0.55 m near range it is 23 mm of slack, an order of magnitude
  // more than the millimetre-scale placement error a plan-and-execute segment leaves.
  constexpr double kRequiredMarginPx = 30.0;
  const SensorFrustum frustum = wrist_frustum_from_description();
  ASSERT_TRUE(frustum.valid());
  const std::vector<SurveyStation> stations = nominal_survey_stations(shipped_geometry());
  for (const SurveyStation & station : stations) {
    const FramingResult result = frames_region(
      frustum, station.shelf_from_optical, station.target_points_in_shelf, kRequiredMarginPx);
    EXPECT_TRUE(result.framed) << station.name << ": " << result.detail;
    std::cout << "station " << station.name << ": margin " << result.margin_px
              << " px, range " << result.nearest_range_m << " to " << result.farthest_range_m
              << " m, ground sample " << frustum.ground_sample_m(result.nearest_range_m) * 1000.0
              << " to " << frustum.ground_sample_m(result.farthest_range_m) * 1000.0
              << " mm/px" << std::endl;
  }
}

TEST(SurveyStations, ExpressesAStationInAnotherFrameOnlyThroughAShelfTransform)
{
  const std::vector<SurveyStation> stations = nominal_survey_stations(shipped_geometry());
  const SurveyStation & station = stations.front();

  const Result<FramedTransform> wrong_source =
    FramedTransform::create("lane_01", "world", shipped_world_from_shelf());
  ASSERT_TRUE(wrong_source.has_value());
  const Result<CameraViewpoint> refused = station_viewpoint(station, wrong_source.value());
  ASSERT_FALSE(refused.has_value());
  EXPECT_EQ(refused.error().code, PerceptionErrorCode::FrameMismatch);

  const Result<FramedTransform> world_from_shelf =
    FramedTransform::create(kShelfFrame, "world", shipped_world_from_shelf());
  ASSERT_TRUE(world_from_shelf.has_value());
  const Result<CameraViewpoint> viewpoint =
    station_viewpoint(station, world_from_shelf.value());
  ASSERT_TRUE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.value().pose.frame_id, "world");
  EXPECT_EQ(viewpoint.value().label, station.name);
  EXPECT_LT(
    (viewpoint.value().pose.pose.translation() - station_origin_in_world(station)).norm(),
    1.0e-12);
}

TEST(SurveyStations, RefusesGeometryThatCannotProduceAStation)
{
  WorkcellSurveyGeometry geometry = shipped_geometry();
  const WorkcellSurveyGeometry empty_lanes{{}, geometry.tray, geometry.shelf};
  EXPECT_THROW(
    {[[maybe_unused]] const auto stations = nominal_survey_stations(empty_lanes);},
    std::invalid_argument);

  SurveyStationConfig config;
  config.tray_station_count = 0U;
  EXPECT_THROW(
    {[[maybe_unused]] const auto stations = nominal_survey_stations(geometry, config);},
    std::invalid_argument);

  geometry.tray.usable_size = Eigen::Vector3d(1.36, 0.0, 0.35);
  EXPECT_THROW(
    {[[maybe_unused]] const auto stations = nominal_survey_stations(geometry);},
    std::invalid_argument);
}

TEST(SurveyStations, RefusesAWorkcellGeometryFileItCannotRead)
{
  EXPECT_THROW(
      {
        [[maybe_unused]] const auto loaded =
        load_workcell_survey_geometry("/nonexistent/workcell_geometry.yaml");
      },
    std::invalid_argument);
}

TEST(SurveyStations, PutsATrayStationProbeOnTheUprightSideOfTheImage)
{
  // Card 098 regression for the tray path (the confirmation stance has its own in
  // test_confirmation_viewpoint.cpp; spec: specs/camera-viewpoint-orientation.md). The shipped
  // tray stations look from +Y_shelf toward -Y_shelf, where the old hint-signed roll put
  // image-right on +X_shelf — the 180-degree-rolled solution (gravity -0.630) — so every tray
  // overview in the demo rendered upside down. The probe is off BOTH axes of the aim far enough
  // to project >=150 px from centre on each, so the roll flip cannot hide inside a tolerance
  // (red before the fix: +258 px right, +179 px below; green after: -258, -179).
  const WorkcellSurveyGeometry geometry = shipped_geometry();
  const std::vector<SurveyStation> stations = nominal_survey_stations(geometry);
  const SurveyStation * station = find_survey_station(stations, "tray_1");
  ASSERT_NE(station, nullptr);

  const Eigen::Isometry3d & pose = station->shelf_from_optical;
  // The station's own aim: slice centre of the tray's usable volume (survey_stations.cpp).
  const Eigen::Vector3d target(
    pose.translation().x(), geometry.tray.usable_center_in_shelf.y(),
    geometry.tray.usable_center_in_shelf.z());
  const Eigen::Vector3d probe = target + Eigen::Vector3d(0.20, 0.0, 0.22);

  const Eigen::Vector3d image_up = -(pose.linear() * Eigen::Vector3d::UnitY());
  EXPECT_GT(image_up.z(), 0.3) << station->name << " station view is upside down again";

  const SensorFrustum frustum = wrist_frustum_from_description();
  const ProjectedPoint projected = project_into_image(frustum, pose.inverse() * probe);
  ASSERT_TRUE(projected.in_front);
  const double centre_column = static_cast<double>(frustum.width_px) / 2.0;
  const double centre_row = static_cast<double>(frustum.height_px) / 2.0;

  // Non-blindness guard, as in the confirmation test.
  EXPECT_GE(std::abs(projected.column_px - centre_column), 150.0);
  EXPECT_GE(std::abs(projected.row_px - centre_row), 150.0);

  EXPECT_LT(projected.column_px, centre_column)
    << "a +X_shelf probe must land left of centre under the upright roll";
  EXPECT_LT(projected.row_px, centre_row)
    << "a probe above the aim must land above centre under the upright roll";
}

TEST(SurveyStations, KeepsTheLaneStationUprightWithImageRightOnPlusX)
{
  // The other half of the Card 098 claim: the lane stations' +Y_shelf boresight always agreed
  // with the +X hint, so the corrected upright contract must leave them byte-identical —
  // image-right exactly +X_shelf, gravity +0.824 (spec: camera-viewpoint-orientation.md).
  const std::vector<SurveyStation> stations = nominal_survey_stations(shipped_geometry());
  const SurveyStation * lane = find_survey_station(stations, "lane_03");
  ASSERT_NE(lane, nullptr);

  const Eigen::Matrix3d & rotation = lane->shelf_from_optical.linear();
  const Eigen::Vector3d image_right = rotation * Eigen::Vector3d::UnitX();
  EXPECT_NEAR(image_right.x(), 1.0, 1.0e-9);
  EXPECT_NEAR(image_right.y(), 0.0, 1.0e-9);
  EXPECT_NEAR(image_right.z(), 0.0, 1.0e-9);
  const Eigen::Vector3d image_up = -(rotation * Eigen::Vector3d::UnitY());
  EXPECT_GT(image_up.z(), 0.3);
}

}  // namespace
}  // namespace restocker_perception
