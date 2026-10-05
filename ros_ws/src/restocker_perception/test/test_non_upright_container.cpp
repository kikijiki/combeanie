// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "restocker_perception/colour_depth_backend.hpp"

namespace
{

using restocker_perception::CameraIntrinsics;
using restocker_perception::ColourDepthBackend;
using restocker_perception::ColourDepthConfig;
using restocker_perception::FramedTransform;
using restocker_perception::ProductSignature;
using restocker_perception::RgbdFrame;
using DetectionMessage = restocker_interfaces::msg::ObjectDetection;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

constexpr const char * kCameraFrame = "restocker/workcell/overhead_camera_optical_frame";
constexpr const char * kPlanningFrame = "world";

// The shipped overhead camera, restated: 960x720 over a 1.50 rad horizontal field, looking at the
// stock area from 2.06 m and 55 degrees above its horizon. Those are workcell.xacro's numbers, and
// they matter here because every gate under test is a ratio measured on a projected top face.
//
// It is not the shipped camera to the millimetre: the mount is reconstructed from its field angle
// and range rather than read out of the URDF chain, the surface is a flat plane rather than the
// tray, and the products render as flat colour with no shading and no depth noise. So the
// translation errors below are this fixture's numbers and are not comparable with the 1.28, 2.09
// and 2.65 mm that test_perception_runtime measures against real physics and a real renderer. What
// is comparable is one row against another, which is what the finding rests on: the same camera,
// the same product, the same thresholds, one thing changing.
constexpr double kRangeM = 2.06;
constexpr double kElevationRad = 55.0 * M_PI / 180.0;
constexpr double kStockSurfaceZ = 0.90;

[[nodiscard]] CameraIntrinsics overhead_intrinsics()
{
  const double fx = 480.0 / std::tan(1.50 / 2.0);
  return CameraIntrinsics{fx, fx, 480.0, 360.0, 960, 720};
}

// The camera pose that looks at (0, 0, kStockSurfaceZ) from kRangeM away and kElevationRad above
// its horizon, in the optical convention: +Z along the view axis, +X right, +Y down.
[[nodiscard]] Eigen::Isometry3d overhead_camera_pose()
{
  const Eigen::Vector3d target(0.0, 0.0, kStockSurfaceZ);
  const Eigen::Vector3d to_camera(0.0, -std::cos(kElevationRad), std::sin(kElevationRad));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = target + kRangeM * to_camera;
  Eigen::Matrix3d rotation;
  rotation.col(2) = -to_camera;                 // optical +Z is the view axis
  rotation.col(0) = Eigen::Vector3d::UnitX();   // optical +X runs along the rail
  rotation.col(1) = rotation.col(2).cross(rotation.col(0));
  pose.linear() = rotation;
  return pose;
}

[[nodiscard]] FramedTransform overhead_transform()
{
  auto transform = FramedTransform::create(kCameraFrame, kPlanningFrame, overhead_camera_pose());
  EXPECT_TRUE(transform.has_value());
  return transform.value();
}

// config/overhead_rgbd_perception.yaml, restated. Nothing here is tuned for this test: if these
// thresholds move, this measurement has to be retaken rather than adjusted.
[[nodiscard]] ColourDepthConfig shipped_config()
{
  ColourDepthConfig config;
  config.workspace.minimum = Eigen::Vector3d(-1.25, -1.10, 0.55);
  config.workspace.maximum = Eigen::Vector3d(1.25, 1.45, 1.70);
  ProductSignature can;
  can.product_class = DetectionMessage::PRODUCT_CLASS_CAN;
  can.sku = "SIM-CAN-STD";
  can.reference_rgb = {200.0, 80.0, 70.0};
  can.nominal_radius_m = 0.033;
  can.nominal_height_m = 0.122;
  ProductSignature large_bottle;
  large_bottle.product_class = DetectionMessage::PRODUCT_CLASS_LARGE_BOTTLE;
  large_bottle.sku = "SIM-BOTTLE-LARGE";
  large_bottle.reference_rgb = {70.0, 140.0, 105.0};
  large_bottle.nominal_radius_m = 0.045;
  large_bottle.nominal_height_m = 0.290;
  config.signatures = {can, large_bottle};
  return config;
}

// A finite cylinder in the planning frame: where it is, which way its symmetry axis points, how
// big it is, and what colour it renders as.
struct Cylinder
{
  Eigen::Vector3d centre{Eigen::Vector3d::Zero()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double radius_m{0.033};
  double height_m{0.122};
  std::array<std::uint8_t, 3> colour{200U, 80U, 70U};
};

// A cylinder tilted `tilt_rad` off vertical, about the rail axis, resting on the stock surface. At
// 0 it is standing; at pi/2 it is lying on its side. The values between are a product caught
// mid-topple, or one standing on a tilted tray.
[[nodiscard]] Cylinder resting_cylinder(
  double tilt_rad, double radius_m, double height_m, std::array<std::uint8_t, 3> colour)
{
  Cylinder cylinder;
  cylinder.axis = Eigen::Vector3d(0.0, std::sin(tilt_rad), std::cos(tilt_rad)).normalized();
  cylinder.radius_m = radius_m;
  cylinder.height_m = height_m;
  cylinder.colour = colour;
  const double half = height_m / 2.0;
  const double vertical_reach = half * std::abs(cylinder.axis.z()) +
    radius_m * std::sqrt(std::max(0.0, 1.0 - cylinder.axis.z() * cylinder.axis.z()));
  cylinder.centre = Eigen::Vector3d(0.0, 0.0, kStockSurfaceZ + vertical_reach);
  return cylinder;
}

// Ray-casts a finite cylinder, caps included, into a colour and a depth image.
//
// This is the whole point of the fixture. The existing backend tests paint a filled disc at one
// depth, which can only ever be an upright cylinder seen from directly above, so they cannot
// answer what the backend does with one that is not upright. A ray-cast produces the real
// projection at any pose, including the ellipse a tilted cap projects to and the foreshortened
// barrel beside it.
class RaycastScene
{
public:
  RaycastScene(CameraIntrinsics intrinsics, Eigen::Isometry3d planning_from_camera)
  : intrinsics_(intrinsics), planning_from_camera_(std::move(planning_from_camera))
  {
    colour_ = std::make_shared<sensor_msgs::msg::Image>();
    colour_->header.frame_id = kCameraFrame;
    colour_->header.stamp.sec = 31;
    colour_->header.stamp.nanosec = 250000000U;
    colour_->width = intrinsics.width;
    colour_->height = intrinsics.height;
    colour_->encoding = "rgb8";
    colour_->step = static_cast<std::uint32_t>(intrinsics.width) * 3U;
    colour_->data.assign(static_cast<std::size_t>(colour_->step) * intrinsics.height, 0U);

    depth_ = std::make_shared<sensor_msgs::msg::Image>();
    depth_->header = colour_->header;
    depth_->width = intrinsics.width;
    depth_->height = intrinsics.height;
    depth_->encoding = "32FC1";
    depth_->step = static_cast<std::uint32_t>(intrinsics.width) * 4U;
    depth_->data.assign(static_cast<std::size_t>(depth_->step) * intrinsics.height, 0U);
    paint_stock_surface();
  }

  void paint(const Cylinder & cylinder)
  {
    for (std::size_t row = 0; row < colour_->height; ++row) {
      for (std::size_t column = 0; column < colour_->width; ++column) {
        const auto hit = intersect(cylinder, ray(column, row));
        if (!hit) {
          continue;
        }
        const std::size_t index = row * (depth_->step / 4U) + column;
        if (*hit >= read_depth(index)) {
          continue;   // the stock surface, or an earlier product, is in front of it
        }
        write_depth(index, static_cast<float>(*hit));
        std::uint8_t * pixel = colour_->data.data() + row * colour_->step + column * 3U;
        pixel[0] = cylinder.colour[0];
        pixel[1] = cylinder.colour[1];
        pixel[2] = cylinder.colour[2];
      }
    }
  }

  [[nodiscard]] RgbdFrame frame() const
  {
    auto created = RgbdFrame::create(intrinsics_, colour_, depth_);
    EXPECT_TRUE(created.has_value());
    return created.value();
  }

private:
  // The direction, in the planning frame, of the ray through pixel centre (column, row), scaled so
  // that the parameter along it is the quantity a 32FC1 depth image carries: metres along optical
  // +Z, not distance from the centre of projection.
  [[nodiscard]] Eigen::Vector3d ray(std::size_t column, std::size_t row) const
  {
    const Eigen::Vector3d in_camera(
      (static_cast<double>(column) - intrinsics_.cx) / intrinsics_.fx,
      (static_cast<double>(row) - intrinsics_.cy) / intrinsics_.fy, 1.0);
    return planning_from_camera_.linear() * in_camera;
  }

  [[nodiscard]] std::optional<double> intersect(
    const Cylinder & cylinder, const Eigen::Vector3d & direction) const
  {
    const Eigen::Vector3d & axis = cylinder.axis;
    const Eigen::Vector3d offset = planning_from_camera_.translation() - cylinder.centre;
    const double half = cylinder.height_m / 2.0;
    const Eigen::Vector3d direction_perp = direction - direction.dot(axis) * axis;
    const Eigen::Vector3d offset_perp = offset - offset.dot(axis) * axis;

    std::optional<double> nearest;
    const auto keep = [&nearest](double candidate) {
      if (candidate > 1.0e-6 && (!nearest || candidate < *nearest)) {
        nearest = candidate;
      }
    };

    const double a = direction_perp.squaredNorm();
    if (a > 1.0e-12) {
      const double b = 2.0 * offset_perp.dot(direction_perp);
      const double c = offset_perp.squaredNorm() - cylinder.radius_m * cylinder.radius_m;
      const double discriminant = b * b - 4.0 * a * c;
      if (discriminant >= 0.0) {
        const double root = std::sqrt(discriminant);
        for (const double candidate : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
          if (std::abs((offset + candidate * direction).dot(axis)) <= half) {
            keep(candidate);
          }
        }
      }
    }
    const double along = direction.dot(axis);
    if (std::abs(along) > 1.0e-12) {
      for (const double face : {half, -half}) {
        const double candidate = (face - offset.dot(axis)) / along;
        const Eigen::Vector3d point = offset + candidate * direction;
        if ((point - point.dot(axis) * axis).norm() <= cylinder.radius_m) {
          keep(candidate);
        }
      }
    }
    return nearest;
  }

  // A matte, unlit horizontal plane under the products. It carries a depth return wherever it is
  // visible, so a pixel that is not on a product is a measurement of something else rather than a
  // dropout, and it is dark enough that the backend's intensity floor discards its chromaticity.
  void paint_stock_surface()
  {
    for (std::size_t row = 0; row < depth_->height; ++row) {
      for (std::size_t column = 0; column < depth_->width; ++column) {
        const Eigen::Vector3d direction = ray(column, row);
        const std::size_t index = row * (depth_->step / 4U) + column;
        const double distance = std::abs(direction.z()) < 1.0e-9 ?
          -1.0 : (kStockSurfaceZ - planning_from_camera_.translation().z()) / direction.z();
        write_depth(index, distance > 0.0 ? static_cast<float>(distance) : 0.0F);
      }
    }
  }

  void write_depth(std::size_t index, float metres)
  {
    std::memcpy(depth_->data.data() + index * sizeof(float), &metres, sizeof(float));
  }

  [[nodiscard]] float read_depth(std::size_t index) const
  {
    float metres = 0.0F;
    std::memcpy(&metres, depth_->data.data() + index * sizeof(float), sizeof(float));
    return metres > 0.0F ? metres : std::numeric_limits<float>::max();
  }

  CameraIntrinsics intrinsics_;
  Eigen::Isometry3d planning_from_camera_;
  sensor_msgs::msg::Image::SharedPtr colour_;
  sensor_msgs::msg::Image::SharedPtr depth_;
};

[[nodiscard]] restocker_perception::PoseCovariance covariance_of(
  const ObservationMessage & observation)
{
  restocker_perception::PoseCovariance covariance;
  for (Eigen::Index row = 0; row < 6; ++row) {
    for (Eigen::Index column = 0; column < 6; ++column) {
      covariance(row, column) = observation.pose.covariance[row * 6 + column];
    }
  }
  return covariance;
}

struct Outcome
{
  std::size_t detections{0};
  std::vector<ObservationMessage> observations;
  // Card 050: which geometry gate refused the proposals that never became observations. The
  // silent half of the receipt — detect() proposed, estimate() dropped, nothing downstream saw.
  restocker_perception::EstimateDropCounts drops;
};

[[nodiscard]] Outcome run(const Cylinder & cylinder, const ColourDepthConfig & config)
{
  RaycastScene scene(overhead_intrinsics(), overhead_camera_pose());
  scene.paint(cylinder);
  auto backend = ColourDepthBackend::create(config);
  EXPECT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  EXPECT_EQ(detections.status, restocker_interfaces::msg::PerceptionFrame::STATUS_OK);
  auto observations = backend.value().estimate(detections, frame, overhead_transform());
  EXPECT_TRUE(observations.has_value());
  return Outcome{
    detections.detections.size(), observations.value(),
    backend.value().last_estimate_drop_counts()};
}

// The measurement, at the shipped thresholds, on the shipped camera geometry.
//
// `observations` is what the backend produced for one container at that tilt. The admitted set is
// not "upright, within a few degrees" but two disjoint bands with a rejected gap between them, and
// the second band contains the wrist tray tilt of 51 degrees.
struct TiltCase
{
  double degrees{0.0};
  std::size_t observations{0};
  double translation_error_m{0.0};   // ignored when observations == 0
};

void check(const Cylinder & cylinder, const TiltCase & expected, const ColourDepthConfig & config)
{
  const auto outcome = run(cylinder, config);
  SCOPED_TRACE("tilt " + std::to_string(expected.degrees) + " degrees");
  // Colour proposes at every tilt. Whatever the backend decides, it decides in estimate(), which
  // is why nothing downstream sees a proposal that was refused: detect() published one either way.
  EXPECT_EQ(outcome.detections, 1U);
  ASSERT_EQ(outcome.observations.size(), expected.observations);
  // Card 050: an admitted tilt has no refused proposals behind it, and a rejected one names its
  // gate in the drop counts — the receipt that separates "empty view" from "refused view".
  EXPECT_EQ(outcome.drops.total() > 0U, expected.observations == 0U)
    << "drop counts " << outcome.drops.total() << " against " << expected.observations
    << " observation(s)";
  for (const auto & observation : outcome.observations) {
    const Eigen::Vector3d estimated(
      observation.pose.pose.position.x, observation.pose.pose.position.y,
      observation.pose.pose.position.z);
    EXPECT_NEAR((estimated - cylinder.centre).norm(), expected.translation_error_m, 5.0e-4);
    // The point of the whole file. Whatever the container is doing, this is what is published.
    EXPECT_EQ(observation.orientation, ObservationMessage::ORIENTATION_UPRIGHT);
    EXPECT_DOUBLE_EQ(observation.pose.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(observation.pose.pose.orientation.x, 0.0);
    EXPECT_DOUBLE_EQ(observation.pose.pose.orientation.y, 0.0);
    EXPECT_DOUBLE_EQ(observation.pose.pose.orientation.z, 0.0);
    // And it arrives declaring that it measured no orientation, which is what lets the pose-error
    // evaluator refuse to score one.
    EXPECT_FALSE(
      restocker_perception::declares_axis_estimate(covariance_of(observation)));
  }
}

TEST(NonUprightContainer, PublishesAnUprightPoseForAContainerTiltedFarOffVertical)
{
  const auto config = shipped_config();
  // A can at 51 degrees (the wrist tray tilt) clears every geometry gate and is published as an
  // upright can 24 mm from where it is (re-measured with Card 036's axis estimator; was 16.5 mm
  // under the band mean): not rejected, not flagged, and indistinguishable from a good
  // observation except by a translation error no consumer knows.
  const auto tilted = resting_cylinder(51.0 * M_PI / 180.0, 0.033, 0.122, {200U, 80U, 70U});
  const auto outcome = run(tilted, config);
  ASSERT_EQ(outcome.observations.size(), 1U);
  const auto & observation = outcome.observations.front();
  EXPECT_EQ(observation.orientation, ObservationMessage::ORIENTATION_UPRIGHT);
  EXPECT_DOUBLE_EQ(observation.pose.pose.orientation.w, 1.0);
  EXPECT_GT(observation.confidence, 0.8F);

  // The true axis is 0.89 rad from the one implied by the identity quaternion. The backend used to
  // publish a tilt sigma of 0.10 rad beside it, which put this 8.9 sigma outside a distribution it
  // declared; it now declares the quantity unmeasured instead.
  const double axis_error = std::acos(std::abs(tilted.axis.z()));
  EXPECT_NEAR(axis_error, 0.89, 0.01);
  EXPECT_FALSE(restocker_perception::declares_axis_estimate(covariance_of(observation)));
}

// Card 063 review (N4): the split ships on the wrist tray duties, so the single-product invariant
// is pinned on the wrist confirm geometry too, for all three products and eight tilt azimuths.
// The wrist half-resolution pinhole and the confirm pose (the tray stations' 51-degree
// direction scaled to 0.30 m) are restated from test_tray_duty_parameters; the thresholds are
// tray_confirm_perception's in config/wrist_tray_perception.yaml, with a workspace that holds
// this fixture's stock surface.
[[nodiscard]] CameraIntrinsics wrist_intrinsics()
{
  const double fx = (640 / 2.0) / std::tan(1.48 / 2.0);
  return CameraIntrinsics{fx, fx, 320.0, 180.0, 640, 360};
}

[[nodiscard]] Eigen::Isometry3d wrist_confirm_pose(const Eigen::Vector3d & target)
{
  const double scale = 0.30 / std::hypot(0.45, 0.555);
  const Eigen::Vector3d eye = target + Eigen::Vector3d(0.0, 0.45 * scale, 0.555 * scale);
  const Eigen::Vector3d axis_z = (target - eye).normalized();
  Eigen::Vector3d axis_x = Eigen::Vector3d::UnitX();
  axis_x -= axis_x.dot(axis_z) * axis_z;
  axis_x.normalize();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = axis_x;
  pose.linear().col(1) = axis_z.cross(axis_x);
  pose.linear().col(2) = axis_z;
  pose.translation() = eye;
  return pose;
}

[[nodiscard]] ColourDepthConfig wrist_confirm_config(double split_gap_m)
{
  ColourDepthConfig config;
  config.workspace.minimum = Eigen::Vector3d(-1.0, -1.5, 0.5);
  config.workspace.maximum = Eigen::Vector3d(1.0, 0.5, 1.5);
  config.minimum_depth_m = 0.10;
  config.maximum_depth_m = 0.55;
  config.minimum_component_pixels = 200;
  config.minimum_mean_radius_ratio = 0.50;
  config.maximum_mean_radius_ratio = 0.95;
  config.minimum_extreme_radius_ratio = 0.75;
  config.maximum_extreme_radius_ratio = 1.40;
  config.translation_sigma_floor_m = 0.002;
  config.top_face_split_gap_m = split_gap_m;
  ProductSignature can;
  can.product_class = DetectionMessage::PRODUCT_CLASS_CAN;
  can.sku = "SIM-CAN-STD";
  can.reference_rgb = {200.0, 80.0, 70.0};
  can.nominal_radius_m = 0.033;
  can.nominal_height_m = 0.122;
  ProductSignature small_bottle;
  small_bottle.product_class = DetectionMessage::PRODUCT_CLASS_SMALL_BOTTLE;
  small_bottle.sku = "SIM-BOTTLE-SMALL";
  small_bottle.reference_rgb = {70.0, 130.0, 190.0};
  small_bottle.nominal_radius_m = 0.034;
  small_bottle.nominal_height_m = 0.200;
  ProductSignature large_bottle;
  large_bottle.product_class = DetectionMessage::PRODUCT_CLASS_LARGE_BOTTLE;
  large_bottle.sku = "SIM-BOTTLE-LARGE";
  large_bottle.reference_rgb = {70.0, 140.0, 105.0};
  large_bottle.nominal_radius_m = 0.045;
  large_bottle.nominal_height_m = 0.290;
  config.signatures = {can, small_bottle, large_bottle};
  return config;
}

TEST(NonUprightContainer, WristSplitAdmitsNoTiltTheWholeBandRefusesForAnyProductOrAzimuth)
{
  struct Product
  {
    double radius_m;
    double height_m;
    std::array<std::uint8_t, 3> colour;
  };
  const auto whole = wrist_confirm_config(0.0);
  const auto split = wrist_confirm_config(0.008);
  for (const Product & product : {
      Product{0.033, 0.122, {200U, 80U, 70U}},
      Product{0.034, 0.200, {70U, 130U, 190U}},
      Product{0.045, 0.290, {70U, 140U, 105U}}})
  {
    // The confirm pose aims at where the product would stand upright, as the survey does.
    const Eigen::Vector3d upright_centre(0.0, -0.80, kStockSurfaceZ + product.height_m / 2.0);
    const auto camera = wrist_confirm_pose(upright_centre);
    auto transform = FramedTransform::create(kCameraFrame, kPlanningFrame, camera);
    ASSERT_TRUE(transform.has_value());
    for (int degrees = 0; degrees <= 90; degrees += 6) {
      for (int azimuth = 0; azimuth < 360; azimuth += 45) {
        const double tilt = degrees * M_PI / 180.0;
        const double heading = azimuth * M_PI / 180.0;
        Cylinder cylinder;
        cylinder.axis = Eigen::Vector3d(
          std::sin(tilt) * std::cos(heading), std::sin(tilt) * std::sin(heading), std::cos(tilt));
        cylinder.radius_m = product.radius_m;
        cylinder.height_m = product.height_m;
        cylinder.colour = product.colour;
        const double vertical_reach = product.height_m / 2.0 * std::abs(cylinder.axis.z()) +
          product.radius_m * std::sqrt(std::max(0.0, 1.0 - cylinder.axis.z() * cylinder.axis.z()));
        cylinder.centre = Eigen::Vector3d(0.0, -0.80, kStockSurfaceZ + vertical_reach);

        RaycastScene scene(wrist_intrinsics(), camera);
        scene.paint(cylinder);
        const auto frame = scene.frame();
        std::size_t counts[2] = {0U, 0U};
        for (const int which : {0, 1}) {
          auto backend = ColourDepthBackend::create(which == 0 ? whole : split);
          ASSERT_TRUE(backend.has_value());
          const auto detections = backend.value().detect(frame);
          auto observations = backend.value().estimate(detections, frame, transform.value());
          ASSERT_TRUE(observations.has_value());
          counts[which] = observations.value().size();
        }
        SCOPED_TRACE(
          "radius " + std::to_string(product.radius_m) + " tilt " + std::to_string(degrees) +
          " azimuth " + std::to_string(azimuth));
        EXPECT_LE(counts[1], counts[0]);
        EXPECT_LE(counts[1], 1U);
        if (degrees == 0) {
          // The fixture sees the product: standing upright it is admitted either way.
          EXPECT_EQ(counts[0], 1U);
          EXPECT_EQ(counts[1], 1U);
        }
      }
    }
  }
}

TEST(NonUprightContainer, SplittingTheTopBandAdmitsNoTiltTheWholeBandRefuses)
{
  // Card 063: the disc split may only turn a two-disc band into single products. On one
  // container at any tilt it must admit exactly what the whole band admits, never more.
  const auto whole = shipped_config();
  auto split = shipped_config();
  split.top_face_split_gap_m = 0.008;
  for (double degrees = 0.0; degrees <= 90.0; degrees += 3.0) {
    for (const auto & cylinder : {
        resting_cylinder(degrees * M_PI / 180.0, 0.033, 0.122, {200U, 80U, 70U}),
        resting_cylinder(degrees * M_PI / 180.0, 0.045, 0.290, {70U, 140U, 105U})})
    {
      SCOPED_TRACE("tilt " + std::to_string(degrees) + " degrees");
      const auto kept = run(cylinder, whole);
      const auto divided = run(cylinder, split);
      EXPECT_LE(divided.observations.size(), kept.observations.size());
      EXPECT_LE(divided.observations.size(), 1U);
    }
  }
}

TEST(NonUprightContainer, AdmitsTwoDisjointBandsOfTiltRatherThanOne)
{
  const auto config = shipped_config();
  // Translation errors re-measured after Card 036's axis estimator (circle fit to the band's
  // convex hull instead of the band mean). Admission counts and the rejected gap are unchanged:
  // the radius gates still measure about the band mean. The upright case collapses from 5.6 mm
  // to 0.3 mm — the same world-Y bias the wrist duties had — while **tilted-input error
  // increased** (e.g. 10 deg 1.5 -> 7.9 mm, 51 deg can 16.5 -> 24.1 mm): a circle fit to an
  // elliptical silhouette is a different wrong answer than a mean, and both are still published
  // as upright, which is the defect this file exists to pin. Nothing new is admitted — only the
  // published pose of already-mis-admitted tilts moves (Card 036 review, section 2).
  for (const TiltCase & expected : {
      TiltCase{0.0, 1U, 0.0003},
      TiltCase{5.0, 1U, 0.0041},
      TiltCase{10.0, 1U, 0.0079},
      TiltCase{15.0, 1U, 0.0125},
      TiltCase{20.0, 1U, 0.0161},
      TiltCase{30.0, 0U, 0.0},
      TiltCase{39.0, 0U, 0.0},
      TiltCase{51.0, 1U, 0.0241},
      TiltCase{60.0, 1U, 0.0264},
      TiltCase{75.0, 0U, 0.0},
      TiltCase{90.0, 0U, 0.0},
    })
  {
    check(
      resting_cylinder(expected.degrees * M_PI / 180.0, 0.033, 0.122, {200U, 80U, 70U}),
      expected, config);
  }
}

TEST(NonUprightContainer, DoesTheSameToTheLargestProductInTheCatalogue)
{
  const auto config = shipped_config();
  // The large bottle is worse, not better: at 51 degrees it is published about 89 mm from where
  // it is, which is more than twice its own radius, and still as an upright bottle. Values
  // re-measured with Card 036's axis estimator; admission band unchanged.
  for (const TiltCase & expected : {
      TiltCase{0.0, 1U, 0.0004},
      TiltCase{10.0, 1U, 0.0222},
      TiltCase{20.0, 1U, 0.0436},
      TiltCase{30.0, 0U, 0.0},
      TiltCase{51.0, 1U, 0.0891},
      TiltCase{75.0, 0U, 0.0},
      TiltCase{90.0, 0U, 0.0},
    })
  {
    check(
      resting_cylinder(expected.degrees * M_PI / 180.0, 0.045, 0.290, {70U, 140U, 105U}),
      expected, config);
  }
}

// A container lying flat, a case worth pinning. It yields no
// observation at all: detect() proposes it, estimate() drops it at the radial-profile gate, and
// the returned vector is one shorter. Nothing records that a proposal was refused, so
// downstream this is indistinguishable from an empty tray.
TEST(NonUprightContainer, ProducesNothingAtAllForAContainerLyingOnItsSide)
{
  const auto config = shipped_config();
  for (const auto & cylinder : {
      resting_cylinder(M_PI / 2.0, 0.033, 0.122, {200U, 80U, 70U}),
      resting_cylinder(M_PI / 2.0, 0.045, 0.290, {70U, 140U, 105U}),
    })
  {
    const auto outcome = run(cylinder, config);
    EXPECT_EQ(outcome.detections, 1U) << "the colour stage still proposes it";
    EXPECT_TRUE(outcome.observations.empty());
    // Card 050: the drop is not invisible to the receipt any more — the gate that refused the
    // proposal is counted, so a zero-frame dwell caused by a tipped product names itself.
    EXPECT_GT(outcome.drops.total(), 0U)
      << "a refused proposal must be counted at its gate";
    EXPECT_GT(outcome.drops.radial_profile, 0U)
      << "a container on its side is refused by the radial-profile gate";
  }

  // And an empty tray, for the comparison that matters: no detection and no observation. The two
  // cases differ by one number that is published nowhere, detect()'s count, and by nothing a
  // consumer of the observation stream can see.
  RaycastScene empty(overhead_intrinsics(), overhead_camera_pose());
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = empty.frame();
  const auto detections = backend.value().detect(frame);
  EXPECT_TRUE(detections.detections.empty());
  const auto observations = backend.value().estimate(detections, frame, overhead_transform());
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty());
}

// Review note N3: the tilt sweeps above never reach a Card 065 set-aside, so these pin the tilted
// single products that do reach the split's new conditions (found by a finer wrist sweep). Each
// holds one admitted and one failing disc of the *same* product, and each must stay refused by
// the condition named, admitting nothing the whole band refuses.
struct SetAsideTiltCase
{
  double radius_m;
  double height_m;
  std::array<std::uint8_t, 3> colour;
  int tilt_degrees;
  int azimuth_degrees;
  double aim_dy_m;
  restocker_perception::SetAsideAnswer expected;
};

void expect_tilted_product_refused_by(const SetAsideTiltCase & tilt_case)
{
  const Eigen::Vector3d upright_centre(
    0.0, -0.80 + tilt_case.aim_dy_m, kStockSurfaceZ + tilt_case.height_m / 2.0);
  const auto camera = wrist_confirm_pose(upright_centre);
  auto transform = FramedTransform::create(kCameraFrame, kPlanningFrame, camera);
  ASSERT_TRUE(transform.has_value());
  const double tilt = tilt_case.tilt_degrees * M_PI / 180.0;
  const double heading = tilt_case.azimuth_degrees * M_PI / 180.0;
  Cylinder cylinder;
  cylinder.axis = Eigen::Vector3d(
    std::sin(tilt) * std::cos(heading), std::sin(tilt) * std::sin(heading), std::cos(tilt));
  cylinder.radius_m = tilt_case.radius_m;
  cylinder.height_m = tilt_case.height_m;
  cylinder.colour = tilt_case.colour;
  const double vertical_reach = tilt_case.height_m / 2.0 * std::abs(cylinder.axis.z()) +
    tilt_case.radius_m * std::sqrt(std::max(0.0, 1.0 - cylinder.axis.z() * cylinder.axis.z()));
  cylinder.centre = Eigen::Vector3d(0.0, -0.80, kStockSurfaceZ + vertical_reach);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(cylinder);
  const auto frame = scene.frame();
  std::size_t counts[2] = {0U, 0U};
  std::string answer;
  for (const int which : {0, 1}) {
    auto backend = ColourDepthBackend::create(wrist_confirm_config(which == 0 ? 0.0 : 0.008));
    ASSERT_TRUE(backend.has_value());
    backend.value().set_diagnostics_enabled(true);
    const auto detections = backend.value().detect(frame);
    auto observations = backend.value().estimate(detections, frame, transform.value());
    ASSERT_TRUE(observations.has_value());
    counts[which] = observations.value().size();
    if (which == 1) {
      for (const auto & proposal : backend.value().last_proposal_diagnostics()) {
        if (!proposal.set_aside_answer.empty()) {
          answer = proposal.set_aside_answer;
        }
      }
    }
  }
  EXPECT_EQ(answer, restocker_perception::set_aside_answer_name(tilt_case.expected))
    << "the split must reach the new conditions for this case to mean anything";
  EXPECT_EQ(counts[1], 0U);
  EXPECT_LE(counts[1], counts[0]);
}

TEST(NonUprightContainer, ATiltedCanWhoseTopPassesBesideItsOwnBarrelFragmentStaysRefused)
{
  // Condition 3: the failing fragment is the can's own barrel strip, 27.5 mm from its top's
  // fitted centre (inside 1.5 radii).
  expect_tilted_product_refused_by(
    {0.033, 0.122, {200U, 80U, 70U}, 36, 270, 0.0,
      restocker_perception::SetAsideAnswer::kInsideAdmittedFootprint});
  expect_tilted_product_refused_by(
    {0.033, 0.122, {200U, 80U, 70U}, 38, 270, -0.02,
      restocker_perception::SetAsideAnswer::kInsideAdmittedFootprint});
}

TEST(NonUprightContainer, ATiltedSmallBottleStaysRefusedByTheFootprintCondition)
{
  expect_tilted_product_refused_by(
    {0.034, 0.200, {70U, 130U, 190U}, 34, 270, -0.04,
      restocker_perception::SetAsideAnswer::kInsideAdmittedFootprint});
}

TEST(NonUprightContainer, ATiltedSmallBottleWhoseFragmentPassesBehindItsTopStaysRefused)
{
  // Condition 2: a 166-point fragment passes every gate while the bottle's real top fails
  // nearer the camera.
  expect_tilted_product_refused_by(
    {0.034, 0.200, {70U, 130U, 190U}, 28, 270, 0.0,
      restocker_perception::SetAsideAnswer::kFailingDiscNearer});
}

}  // namespace
