// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

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

// The wrist sensor's frustum, read out of the description rather than restated here. The
// declaration is literal XML in these five fields; if any becomes a xacro expression the match
// fails loudly, which is the direction a framing check should fail in.
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

// The shipped world <- shelf transform, restated the same way test_survey_stations restates it.
[[nodiscard]] Eigen::Isometry3d shipped_world_from_shelf()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.0, 0.55, 0.75);
  return transform;
}

[[nodiscard]] FramedTransform world_from_shelf()
{
  const auto transform =
    FramedTransform::create(kShelfFrame, "world", shipped_world_from_shelf());
  EXPECT_TRUE(transform.has_value());
  return transform.value();
}

// A candidate standing on the tray floor where the shipped scenario stands the stock can.
[[nodiscard]] Eigen::Vector3d stock_can_world()
{
  return Eigen::Vector3d(-0.42, -0.25, 0.631);
}

[[nodiscard]] ConfirmationViewpointConfig shipped_config()
{
  ConfirmationViewpointConfig config;
  config.frustum = wrist_frustum_from_description();
  return config;
}

TEST(ConfirmationViewpoint, StandsAtTheCommandedStandoffOnTheTrayStationsApproach)
{
  const ConfirmationViewpointConfig config = shipped_config();
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_TRUE(viewpoint.has_value()) << viewpoint.error().detail;
  EXPECT_EQ(viewpoint.value().pose.frame_id, "world");
  EXPECT_EQ(viewpoint.value().label, "confirm viewpoint");

  const Eigen::Vector3d target = stock_can_world();
  // The boresight lands on the biased aim, not on the candidate; the eye stands at the
  // commanded standoff from that aim along the stations' own approach direction.
  const Eigen::Vector3d aim =
    target + (shipped_world_from_shelf().rotation() * config.aim_bias_in_shelf);
  const Eigen::Vector3d eye = viewpoint.value().pose.pose.translation();
  EXPECT_NEAR((eye - aim).norm(), config.standoff_m, 1.0e-9);

  const Eigen::Vector3d direction_in_world =
    (shipped_world_from_shelf().rotation() * config.approach_in_shelf).normalized();
  EXPECT_NEAR((eye - aim).normalized().dot(direction_in_world), 1.0, 1.0e-9);
  const Eigen::Vector3d boresight =
    (viewpoint.value().pose.pose.linear() * Eigen::Vector3d::UnitZ()).normalized();
  EXPECT_NEAR(std::asin(-boresight.dot(Eigen::Vector3d::UnitZ())) * 180.0 / M_PI, 51.0, 0.2);
  EXPECT_NEAR(boresight.x(), 0.0, 1.0e-9);
  EXPECT_LT(boresight.y(), 0.0);

  // The boresight lands on the aim: that is what look-at means, restated as a distance so a
  // solver that aimed near the product would fail here. The foot of the perpendicular from the
  // aim onto the ray starting at the eye along the boresight.
  const Eigen::Vector3d closest = eye + boresight * (aim - eye).dot(boresight);
  EXPECT_NEAR((closest - aim).norm(), 0.0, 1.0e-9);
}

TEST(ConfirmationViewpoint, FramesTheLargestCataloguedProductWithMargin)
{
  const ConfirmationViewpointConfig config = shipped_config();
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_TRUE(viewpoint.has_value()) << viewpoint.error().detail;

  // The framing box defaults to the largest catalogued product, so passing for it passes for
  // every product; measure the margin actually achieved rather than the boolean.
  const FramingResult framing = frames_region(
    config.frustum, viewpoint.value().pose.pose,
    box_corners(stock_can_world(), config.framing_box_size), 0.0);
  ASSERT_TRUE(framing.framed) << framing.detail;
  EXPECT_GE(framing.margin_px, config.required_margin_px);
  std::cout << "confirmation viewpoint margin " << framing.margin_px << " px over ranges "
            << framing.nearest_range_m << " to " << framing.farthest_range_m << " m"
            << std::endl;
  EXPECT_GT(framing.nearest_range_m, config.frustum.near_clip_m);
  EXPECT_LT(framing.farthest_range_m, config.frustum.far_clip_m);
}

TEST(ConfirmationViewpoint, KeepsTheEyeInsideTheConfirmDepthBandTheDutyWasDerivedAgainst)
{
  // wrist_tray_perception.yaml's confirm duty accepts depths in [0.10, 0.55] m. The deepest and
  // shallowest points of the largest catalogued product from the solved viewpoint must both fall
  // inside it, or the backend would filter out the very frames this viewpoint exists to take.
  const ConfirmationViewpointConfig config = shipped_config();
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_TRUE(viewpoint.has_value()) << viewpoint.error().detail;
  const Eigen::Vector3d eye = viewpoint.value().pose.pose.translation();
  const Eigen::Vector3d target = stock_can_world();
  const double half_height = config.framing_box_size.z() / 2.0;
  const double nearest = (eye - (target + Eigen::Vector3d(0.0, 0.0, half_height))).norm();
  const double farthest = (eye - (target - Eigen::Vector3d(0.0, 0.0, half_height))).norm();
  EXPECT_GT(nearest, 0.10);
  EXPECT_LT(farthest, 0.55);
  std::cout << "confirmation depths: nearest product point " << nearest << " m, farthest "
            << farthest << " m" << std::endl;
}

TEST(ConfirmationViewpoint, RotatesTheApproachWithAShelfTransformThatIsNotTheIdentity)
{
  // A workcell that is rotated must still get an eye on its own +Y side of the candidate: the
  // approach direction travels through the shelf transform, it is not a world-frame constant.
  Eigen::Isometry3d rotated = Eigen::Isometry3d::Identity();
  rotated.linear() = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  rotated.translation() = Eigen::Vector3d(1.0, 2.0, 0.75);
  const auto transform = FramedTransform::create(kShelfFrame, "world", rotated);
  ASSERT_TRUE(transform.has_value());

  ConfirmationViewpointConfig config = shipped_config();
  const Eigen::Vector3d target(0.0, 0.0, 0.70);
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(target, transform.value(), config);
  ASSERT_TRUE(viewpoint.has_value()) << viewpoint.error().detail;

  // A +90-degree Z rotation maps the shelf-frame aim bias (0, 0, 0.05) to itself and the
  // approach (0, 0.45, 0.555) onto (-0.45, 0, 0.555) up to normalization.
  const Eigen::Vector3d aim =
    target + (rotated.rotation() * config.aim_bias_in_shelf);
  const Eigen::Vector3d eye = viewpoint.value().pose.pose.translation();
  EXPECT_LT(eye.x(), aim.x());
  EXPECT_NEAR(eye.y(), aim.y(), 1.0e-9);
  EXPECT_GT(eye.z(), aim.z());
  EXPECT_NEAR((eye - aim).norm(), config.standoff_m, 1.0e-9);
}

TEST(ConfirmationViewpoint, RefusesAStandoffOutsideTheConfirmBand)
{
  ConfirmationViewpointConfig config = shipped_config();
  config.standoff_m = 0.60;
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::InvalidArgument);
  EXPECT_NE(viewpoint.error().detail.find("standoff"), std::string::npos)
    << viewpoint.error().detail;
}

TEST(ConfirmationViewpoint, RefusesATransformThatDoesNotStartAtTheShelf)
{
  const auto wrong =
    FramedTransform::create("lane_01", "world", shipped_world_from_shelf());
  ASSERT_TRUE(wrong.has_value());
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), wrong.value(), shipped_config());
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::FrameMismatch);
}

TEST(ConfirmationViewpoint, RefusesWhenTheProductCannotBeFramedFromAnywhereInTheBand)
{
  ConfirmationViewpointConfig config = shipped_config();
  // A margin no standpoint in the 0.25-0.35 m band can satisfy: the product would have to sit
  // further inside the image than the image is wide. The failure must name the framing, not
  // silently return some other pose.
  config.required_margin_px = 1.0e6;
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::InvalidArgument);
  EXPECT_NE(viewpoint.error().detail.find("frame"), std::string::npos)
    << viewpoint.error().detail;
}

TEST(ConfirmationViewpoint, RefusesAnElevationFloorTheApproachCannotSatisfy)
{
  ConfirmationViewpointConfig config = shipped_config();
  config.minimum_elevation_rad = 1.50;
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::InvalidArgument);
  EXPECT_NE(viewpoint.error().detail.find("elevation"), std::string::npos)
    << viewpoint.error().detail;
}

TEST(ConfirmationViewpoint, RefusesAnUnpopulatedFrustumRatherThanSkippingFraming)
{
  ConfirmationViewpointConfig config = shipped_config();
  config.frustum = SensorFrustum{};
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::InvalidArgument);
  EXPECT_NE(viewpoint.error().detail.find("frustum"), std::string::npos)
    << viewpoint.error().detail;
}

TEST(ConfirmationViewpoint, RefusesANonFiniteTarget)
{
  const Eigen::Vector3d target(
    std::numeric_limits<double>::quiet_NaN(), 0.0, 0.70);
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(target, world_from_shelf(), shipped_config());
  ASSERT_FALSE(viewpoint.has_value());
  EXPECT_EQ(viewpoint.error().code, PerceptionErrorCode::InvalidArgument);
}

TEST(ConfirmationViewpoint, PutsAnOffCentreProbeOnTheUprightSideOfTheImage)
{
  // Card 098 regression (spec: specs/camera-viewpoint-orientation.md). The old hint-signed roll
  // took preferred_right = +X_shelf as the roll authority, and against this stance's -Y_shelf
  // boresight image-right = +X is the 180-degree-rolled solution: the confirmation view shipped
  // upside down (image-up . world-up = -0.630 here, -0.807 measured live at the capture's
  // X-biased aim). The runtime projection check could not see it — its target sits within 8 px
  // of the image centre, where every flip passes its 10 px tolerance — so this probe is placed
  // off BOTH axes until it projects well outside 150 px of centre on each. The roll flip then
  // moves it twice that far: red before the fix (lands below-right), green after (above-left).
  const ConfirmationViewpointConfig config = shipped_config();
  const Result<CameraViewpoint> viewpoint =
    confirmation_viewpoint(stock_can_world(), world_from_shelf(), config);
  ASSERT_TRUE(viewpoint.has_value()) << viewpoint.error().detail;

  const Eigen::Isometry3d & pose = viewpoint.value().pose.pose;
  const Eigen::Vector3d aim =
    stock_can_world() + (shipped_world_from_shelf().rotation() * config.aim_bias_in_shelf);
  const Eigen::Vector3d probe = aim + Eigen::Vector3d(0.10, 0.0, 0.10);

  // The upright invariant itself: image-up is the opposite of the optical +Y column.
  const Eigen::Vector3d image_up = -(pose.linear() * Eigen::Vector3d::UnitY());
  EXPECT_GT(image_up.z(), 0.3) << "the confirmation viewpoint is upside down again";

  const ProjectedPoint projected = project_into_image(config.frustum, pose.inverse() * probe);
  ASSERT_TRUE(projected.in_front);
  const double centre_column = static_cast<double>(config.frustum.width_px) / 2.0;
  const double centre_row = static_cast<double>(config.frustum.height_px) / 2.0;

  // Non-blindness guard: if the probe ever drifts back toward the centre this test must fail
  // loudly rather than start passing under every flip the way the runtime check does.
  EXPECT_GE(std::abs(projected.column_px - centre_column), 150.0);
  EXPECT_GE(std::abs(projected.row_px - centre_row), 150.0);

  // Upright placement: world +Z is up in the image, and with the boresight toward -Y_shelf the
  // camera's image-right is -X_shelf, so the +X_shelf side of the probe lands LEFT of centre.
  // Before the fix both signs are inverted (measured: +315 px right, +199 px below).
  EXPECT_LT(projected.column_px, centre_column)
    << "a +X_shelf probe must land left of centre under the upright roll";
  EXPECT_LT(projected.row_px, centre_row)
    << "a probe above the aim must land above centre under the upright roll";
}

}  // namespace
}  // namespace restocker_perception
