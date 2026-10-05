// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "restocker_perception/axis_fit.hpp"
#include "restocker_perception/colour_depth_backend.hpp"
#include "restocker_perception/pose_error_evaluation.hpp"

namespace
{

using restocker_perception::CameraIntrinsics;
using restocker_perception::ColourDepthBackend;
using restocker_perception::ColourDepthConfig;
using restocker_perception::FramedTransform;
using restocker_perception::ObservationRangeBand;
using restocker_perception::ProductSignature;
using restocker_perception::RgbdFrame;
using restocker_perception::classify_observation_range;
using restocker_perception::kConfirmRangeMaximumM;
using restocker_perception::kOverviewRangeMaximumM;
using DetectionMessage = restocker_interfaces::msg::ObjectDetection;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

constexpr const char * kCameraFrame = "wrist_camera_optical_frame";
constexpr const char * kPlanningFrame = "world";
constexpr double kTrayFloorZ = 0.57;
// Shipped tray station offset from survey_stations.hpp / test_wrist_viewpoint_runtime.py.
constexpr double kTraySetbackM = 0.45;
constexpr double kTrayHeightM = 0.555;

[[nodiscard]] double tray_standoff_m()
{
  return std::hypot(kTraySetbackM, kTrayHeightM);
}

// Half-resolution wrist pinhole: same 1.48 rad horizontal field as sensors.xacro, so fx scales
// with width and the projected size of a product is unchanged relative to a full-res frame.
[[nodiscard]] CameraIntrinsics wrist_intrinsics()
{
  constexpr double kHorizontalFov = 1.48;
  constexpr int kWidth = 640;
  constexpr int kHeight = 360;
  const double fx = (kWidth / 2.0) / std::tan(kHorizontalFov / 2.0);
  return CameraIntrinsics{fx, fx, kWidth / 2.0, kHeight / 2.0, kWidth, kHeight};
}

[[nodiscard]] Eigen::Isometry3d look_at(
  const Eigen::Vector3d & eye, const Eigen::Vector3d & target)
{
  const Eigen::Vector3d axis_z = (target - eye).normalized();
  Eigen::Vector3d axis_x = Eigen::Vector3d::UnitX();
  axis_x -= axis_x.dot(axis_z) * axis_z;
  axis_x.normalize();
  const Eigen::Vector3d axis_y = axis_z.cross(axis_x);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear().col(0) = axis_x;
  pose.linear().col(1) = axis_y;
  pose.linear().col(2) = axis_z;
  pose.translation() = eye;
  return pose;
}

// Overview: camera at the shipped tray-station offset from a point above the tray floor, elevation
// ~51 degrees. Confirm: same elevation, standoff scaled into the 0.25-0.35 m band.
[[nodiscard]] Eigen::Isometry3d overview_camera_pose(const Eigen::Vector3d & target)
{
  return look_at(target + Eigen::Vector3d(0.0, kTraySetbackM, kTrayHeightM), target);
}

[[nodiscard]] Eigen::Isometry3d confirm_camera_pose(const Eigen::Vector3d & target)
{
  const double scale = 0.30 / tray_standoff_m();
  return look_at(
    target + Eigen::Vector3d(0.0, kTraySetbackM * scale, kTrayHeightM * scale), target);
}

[[nodiscard]] FramedTransform framed(const Eigen::Isometry3d & planning_from_camera)
{
  auto transform = FramedTransform::create(kCameraFrame, kPlanningFrame, planning_from_camera);
  EXPECT_TRUE(transform.has_value());
  return transform.value();
}

[[nodiscard]] ColourDepthConfig overview_config()
{
  // Restates tray_overview_perception from config/wrist_tray_perception.yaml; the test fails if
  // the yaml moves.
  ColourDepthConfig config;
  config.backend_name = "wrist_rgbd_tray_overview";
  config.workspace.minimum = Eigen::Vector3d(-1.00, -1.15, 0.50);
  config.workspace.maximum = Eigen::Vector3d(1.00, -0.40, 1.15);
  config.minimum_depth_m = 0.20;
  config.maximum_depth_m = 1.40;
  config.minimum_component_pixels = 80;
  config.minimum_mean_radius_ratio = 0.50;
  config.maximum_mean_radius_ratio = 0.95;
  config.minimum_extreme_radius_ratio = 0.75;
  config.maximum_extreme_radius_ratio = 1.40;
  // Card 063: both tray duties split a same-colour column's top band into its discs.
  config.top_face_split_gap_m = 0.008;
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

[[nodiscard]] ColourDepthConfig confirm_config()
{
  ColourDepthConfig config = overview_config();
  config.backend_name = "wrist_rgbd_tray_confirm";
  config.minimum_depth_m = 0.10;
  config.maximum_depth_m = 0.55;
  config.minimum_component_pixels = 200;
  // Accuracy floor per Milestone 10 §7 / Cards 035+036; restated from wrist_tray_perception.
  // yaml's tray_confirm_perception. Card 035 raised it 0.002 → 0.004 when the estimator still
  // carried the ~5.5 mm world-Y bias; Card 036 removed the bias (hull circle fit) and the
  // post-fix confirm re-run measured norms 0.191–0.363 mm, so the floor returns to 0.002.
  config.translation_sigma_floor_m = 0.002;
  return config;
}

struct Cylinder
{
  Eigen::Vector3d centre{Eigen::Vector3d::Zero()};
  double radius_m{0.033};
  double height_m{0.122};
  std::array<std::uint8_t, 3> colour{200U, 80U, 70U};
};

[[nodiscard]] Cylinder upright_on_tray(
  double radius_m, double height_m, std::array<std::uint8_t, 3> colour)
{
  Cylinder cylinder;
  cylinder.radius_m = radius_m;
  cylinder.height_m = height_m;
  cylinder.colour = colour;
  cylinder.centre = Eigen::Vector3d(0.0, -0.80, kTrayFloorZ + height_m / 2.0);
  return cylinder;
}

class RaycastScene
{
public:
  RaycastScene(CameraIntrinsics intrinsics, Eigen::Isometry3d planning_from_camera)
  : intrinsics_(intrinsics), planning_from_camera_(std::move(planning_from_camera))
  {
    colour_ = std::make_shared<sensor_msgs::msg::Image>();
    colour_->header.frame_id = kCameraFrame;
    colour_->header.stamp.sec = 40;
    colour_->header.stamp.nanosec = 0U;
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
    paint_tray_surface();
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
          continue;
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

  // An occluder over part of the view: zero the depth (so estimate() finds no returns there)
  // and the colour (so detect() does not propose a fragment of the product on the far side).
  // Column/row bounds are half-open.
  void occlude(
    std::size_t column_begin, std::size_t column_end, std::size_t row_begin,
    std::size_t row_end)
  {
    column_end = std::min(column_end, static_cast<std::size_t>(colour_->width));
    row_end = std::min(row_end, static_cast<std::size_t>(colour_->height));
    for (std::size_t row = row_begin; row < row_end; ++row) {
      for (std::size_t column = column_begin; column < column_end; ++column) {
        const std::size_t depth_index = row * (depth_->step / 4U) + column;
        write_depth(depth_index, 0.0F);
        std::uint8_t * pixel = colour_->data.data() + row * colour_->step + column * 3U;
        pixel[0] = 0U;
        pixel[1] = 0U;
        pixel[2] = 0U;
      }
    }
  }

  // Deterministic ±amplitude depth noise (a fixed LCG), applied only where depth is valid.
  void add_depth_noise(double amplitude_m, std::uint32_t seed)
  {
    std::uint32_t state = seed;
    for (std::size_t row = 0; row < depth_->height; ++row) {
      for (std::size_t column = 0; column < depth_->width; ++column) {
        const std::size_t index = row * (depth_->step / 4U) + column;
        float metres = 0.0F;
        std::memcpy(&metres, depth_->data.data() + index * sizeof(float), sizeof(float));
        if (!(metres > 0.0F)) {
          continue;
        }
        state = state * 1664525U + 1013904223U;
        const double unit =
          static_cast<double>(static_cast<std::int32_t>(state >> 8U)) / 8388608.0;
        metres = static_cast<float>(
          static_cast<double>(metres) + unit * amplitude_m);
        write_depth(index, metres);
      }
    }
  }

private:
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
    const Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
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

  void paint_tray_surface()
  {
    for (std::size_t row = 0; row < depth_->height; ++row) {
      for (std::size_t column = 0; column < depth_->width; ++column) {
        const Eigen::Vector3d direction = ray(column, row);
        const std::size_t index = row * (depth_->step / 4U) + column;
        const double distance = std::abs(direction.z()) < 1.0e-9 ?
          -1.0 :
          (kTrayFloorZ - planning_from_camera_.translation().z()) / direction.z();
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

struct RatioSample
{
  double mean_ratio{0.0};
  double extreme_ratio{0.0};
  std::size_t observations{0};
  double translation_error_m{0.0};
  double range_m{0.0};
};

[[nodiscard]] RatioSample run_duty(
  const Cylinder & cylinder, const ColourDepthConfig & config,
  const Eigen::Isometry3d & planning_from_camera)
{
  RaycastScene scene(wrist_intrinsics(), planning_from_camera);
  scene.paint(cylinder);
  auto backend = ColourDepthBackend::create(config);
  EXPECT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  EXPECT_EQ(detections.status, restocker_interfaces::msg::PerceptionFrame::STATUS_OK);
  auto observations = backend.value().estimate(detections, frame, framed(planning_from_camera));
  EXPECT_TRUE(observations.has_value());
  RatioSample sample;
  sample.observations = observations.value().size();
  sample.range_m = (cylinder.centre - planning_from_camera.translation()).norm();
  if (sample.observations == 0) {
    return sample;
  }
  const auto & observation = observations.value().front();
  const Eigen::Vector3d estimated(
    observation.pose.pose.position.x, observation.pose.pose.position.y,
    observation.pose.pose.position.z);
  sample.translation_error_m = (estimated - cylinder.centre).norm();
  return sample;
}

TEST(TrayDutyParameters, PinsTheShippedTrayElevationAndStandoff)
{
  // The tray station is tilted ~51 degrees, not nadir, and confirm reuses that elevation at 0.30 m.
  EXPECT_NEAR(std::atan2(kTrayHeightM, kTraySetbackM) * 180.0 / M_PI, 51.0, 0.2);
  EXPECT_NEAR(tray_standoff_m(), 0.7145, 1.0e-3);

  const Eigen::Vector3d target(0.0, -0.80, kTrayFloorZ + 0.175);
  const Eigen::Vector3d boresight =
    overview_camera_pose(target).linear() * Eigen::Vector3d::UnitZ();
  EXPECT_NEAR(std::asin(-boresight.z()) * 180.0 / M_PI, 51.0, 0.5);

  const Eigen::Vector3d confirm_target(0.0, -0.80, kTrayFloorZ + 0.061);
  const double confirm_range =
    (confirm_target - confirm_camera_pose(confirm_target).translation()).norm();
  EXPECT_NEAR(confirm_range, 0.30, 1.0e-6);
}

TEST(TrayDutyParameters, PinsDepthAndPixelFloorsAgainstRange)
{
  // Pinned against the shipped full-resolution wrist pinhole (1280x720, hfov 1.48), not the
  // half-resolution fixture used for ray-casting.
  constexpr double kHorizontalFov = 1.48;
  const double full_fx = (1280.0 / 2.0) / std::tan(kHorizontalFov / 2.0);
  const double overview_px = 2.0 * 0.033 * full_fx / 0.55;
  const double confirm_px = 2.0 * 0.033 * full_fx / 0.30;
  EXPECT_NEAR(overview_px, 84.0, 3.0);
  EXPECT_NEAR(confirm_px, 154.0, 5.0);
  EXPECT_EQ(overview_config().minimum_component_pixels, 80U);
  EXPECT_EQ(confirm_config().minimum_component_pixels, 200U);
  EXPECT_LT(overview_config().minimum_component_pixels, overview_px);
  EXPECT_GT(confirm_config().minimum_component_pixels, overview_px);
  EXPECT_LT(confirm_config().minimum_component_pixels, confirm_px * 1.5);
  EXPECT_GT(confirm_config().minimum_component_pixels, overview_config().minimum_component_pixels);
  EXPECT_DOUBLE_EQ(overview_config().minimum_depth_m, 0.20);
  EXPECT_DOUBLE_EQ(overview_config().maximum_depth_m, 1.40);
  EXPECT_DOUBLE_EQ(confirm_config().minimum_depth_m, 0.10);
  EXPECT_DOUBLE_EQ(confirm_config().maximum_depth_m, 0.55);
}

TEST(TrayDutyParameters, OverviewDutyRecoversEveryCataloguedProduct)
{
  const auto config = overview_config();
  const Eigen::Vector3d target(0.0, -0.80, kTrayFloorZ + 0.175);
  const auto camera = overview_camera_pose(target);
  for (const auto & cylinder : {
      upright_on_tray(0.033, 0.122, {200U, 80U, 70U}),
      upright_on_tray(0.034, 0.200, {70U, 130U, 190U}),
      upright_on_tray(0.045, 0.290, {70U, 140U, 105U}),
    })
  {
    const auto sample = run_duty(cylinder, config, camera);
    SCOPED_TRACE(sample.range_m);
    ASSERT_EQ(sample.observations, 1U);
    EXPECT_LT(sample.translation_error_m, 0.020);
    EXPECT_GE(sample.range_m, 0.40);
    EXPECT_LE(sample.range_m, 1.20);
    EXPECT_EQ(classify_observation_range(sample.range_m), ObservationRangeBand::Overview);
    std::cout << "overview range " << sample.range_m << " m, translation error "
              << sample.translation_error_m * 1000.0 << " mm" << std::endl;
  }
}

TEST(TrayDutyParameters, ConfirmDutyRecoversEveryCataloguedProduct)
{
  const auto config = confirm_config();
  for (const auto & cylinder : {
      upright_on_tray(0.033, 0.122, {200U, 80U, 70U}),
      upright_on_tray(0.034, 0.200, {70U, 130U, 190U}),
      upright_on_tray(0.045, 0.290, {70U, 140U, 105U}),
    })
  {
    // Aim at the product centre at 0.30 m standoff. Aiming at the top face puts the tallest
    // bottle's centre past the confirm/overview edge.
    const auto camera = confirm_camera_pose(cylinder.centre);
    const auto sample = run_duty(cylinder, config, camera);
    SCOPED_TRACE(sample.range_m);
    ASSERT_EQ(sample.observations, 1U);
    // Card 036: the axis is a circle fit to the band's convex hull, so the upper-barrel and
    // near-rim density terms that used to land the large bottle's centre about 25 mm out are
    // gone; every product is within 3 mm at this pose.
    EXPECT_LT(sample.translation_error_m, 0.003);
    EXPECT_NEAR(sample.range_m, 0.30, 1.0e-6);
    EXPECT_EQ(classify_observation_range(sample.range_m), ObservationRangeBand::Confirm);
    std::cout << "confirm range " << sample.range_m << " m, translation error "
              << sample.translation_error_m * 1000.0 << " mm" << std::endl;
  }
}

TEST(TrayDutyParameters, ObliqueConfirmDoesNotBiasWorldYTowardTheCamera)
{
  // Card 036 root cause and regression: at the shipped elevation the top-face band mixes disc,
  // camera-facing barrel and a near-rim-dense sampling, so a plain XY mean published the axis
  // several millimetres toward the camera (world +Y at the tray stations; Card 004 measured
  // +2.2 to +7.4 mm one-signed on the twelve admitted confirm rows). The circle fit to the
  // band's convex hull must leave |Y residual| under 1 mm for every catalogued product, and the
  // residual must not grow with radius the way the barrel term did.
  const auto config = confirm_config();
  double worst_y = 0.0;
  for (const auto & cylinder : {
      upright_on_tray(0.033, 0.122, {200U, 80U, 70U}),
      upright_on_tray(0.034, 0.200, {70U, 130U, 190U}),
      upright_on_tray(0.045, 0.290, {70U, 140U, 105U}),
    })
  {
    const auto camera = confirm_camera_pose(cylinder.centre);
    const auto sample = run_duty(cylinder, config, camera);
    ASSERT_EQ(sample.observations, 1U);
    // Recover the signed residual: run_duty only keeps the norm, so re-estimate here.
    RaycastScene scene(wrist_intrinsics(), camera);
    scene.paint(cylinder);
    auto backend = ColourDepthBackend::create(config);
    ASSERT_TRUE(backend.has_value());
    const auto frame = scene.frame();
    const auto observations =
      backend.value().estimate(backend.value().detect(frame), frame, framed(camera));
    ASSERT_TRUE(observations.has_value());
    ASSERT_EQ(observations.value().size(), 1U);
    const auto & pose = observations.value().front().pose.pose.position;
    const double y_residual = pose.y - cylinder.centre.y();
    worst_y = std::max(worst_y, std::abs(y_residual));
    EXPECT_LT(std::abs(y_residual), 0.001) << "world-Y residual " << y_residual;
    EXPECT_LT(std::abs(pose.x - cylinder.centre.x()), 0.001);
  }
  EXPECT_LT(worst_y, 0.001);
}

TEST(TrayDutyParameters, OverheadPixelFloorWouldBeWrongAtConfirmRange)
{
  // The overhead pixel floor (40) would still admit the product at confirm range; the shipped
  // confirm floor is several times higher.
  EXPECT_LT(40U, confirm_config().minimum_component_pixels);
  EXPECT_GE(confirm_config().minimum_component_pixels, 5U * 40U);
}

TEST(TrayDutyParameters, ConfirmSigmaFloorIsAnAccuracyClaimUnderTheJawCeiling)
{
  // Milestone 10 §7 / Cards 035+036: k * floor + 1 mm must cover this duty's measured
  // post-fix accuracy and stay under the jaw-travel ceiling so the budget remains a bound
  // rather than a rubber stamp. Card 004's 2.2–7.5 mm and the 9.872 mm worst attach residual
  // were the *biased* estimator's body; Card 036's re-run (n=12 admitted) measures norms
  // 0.191–0.363 mm, Y bounded at ≤ 0.268 mm, which is what the restored 0.002 floor claims.
  constexpr double kSigmaMultiplier = 3.0;
  constexpr double kMechanicalMarginM = 0.001;
  constexpr double kJawTravelCeilingM = 0.016;
  constexpr double kWorstPostFixResidualM = 0.000363;
  const double floor = confirm_config().translation_sigma_floor_m;
  const double budget =
    std::min(kSigmaMultiplier * floor + kMechanicalMarginM, kJawTravelCeilingM);
  EXPECT_GE(budget, kWorstPostFixResidualM);
  EXPECT_LT(budget, kJawTravelCeilingM);
  EXPECT_DOUBLE_EQ(floor, 0.002);
}

TEST(ObservationRangeBand, SplitsConfirmOverviewAndFar)
{
  EXPECT_EQ(classify_observation_range(0.30), ObservationRangeBand::Confirm);
  EXPECT_EQ(
    classify_observation_range(kConfirmRangeMaximumM - 1.0e-9),
    ObservationRangeBand::Confirm);
  EXPECT_EQ(classify_observation_range(0.60), ObservationRangeBand::Overview);
  EXPECT_EQ(
    classify_observation_range(kOverviewRangeMaximumM - 1.0e-9),
    ObservationRangeBand::Overview);
  EXPECT_EQ(classify_observation_range(2.06), ObservationRangeBand::Far);
  EXPECT_EQ(
    classify_observation_range(std::numeric_limits<double>::quiet_NaN()),
    ObservationRangeBand::Unknown);
  EXPECT_EQ(classify_observation_range(-0.1), ObservationRangeBand::Unknown);
}

// Card 036 review follow-up 3: the hull circle fit is exact on a full silhouette and was
// previously untested on partial ones. Each case must end in either a bounded error or a
// refusal — never a silently wrong centre. The confirm camera aims at the product centre, so
// the product sits near the image centre (320, 180) with a radius of about 38 px at 0.30 m.
struct PartialViewSample
{
  std::size_t observations{0};
  double translation_error_m{0.0};
};

[[nodiscard]] PartialViewSample run_confirm_with_scene(
  RaycastScene & scene, const Cylinder & cylinder, const ColourDepthConfig & config,
  const Eigen::Isometry3d & planning_from_camera)
{
  auto backend = ColourDepthBackend::create(config);
  EXPECT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  EXPECT_EQ(detections.status, restocker_interfaces::msg::PerceptionFrame::STATUS_OK);
  auto observations = backend.value().estimate(detections, frame, framed(planning_from_camera));
  EXPECT_TRUE(observations.has_value());
  PartialViewSample sample;
  sample.observations = observations.value().size();
  if (sample.observations == 0) {
    return sample;
  }
  const auto & pose = observations.value().front().pose.pose.position;
  sample.translation_error_m =
    (Eigen::Vector3d(pose.x, pose.y, pose.z) - cylinder.centre).norm();
  return sample;
}

TEST(TrayDutyParameters, HalfMaskedBandBoundsTheAxisOrRefuses)
{
  // Mask the far half of the view (columns left of centre): the band keeps about half a turn
  // of rim, which the span gate accepts, so the fit must still land near the true axis — or
  // the radius/quality gates may refuse it. Either is acceptable; publishing a centre ~1 radius
  // off is not.
  const auto config = confirm_config();
  const auto cylinder = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  const auto camera = confirm_camera_pose(cylinder.centre);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(cylinder);
  scene.occlude(0U, 320U, 0U, 360U);
  const auto sample = run_confirm_with_scene(scene, cylinder, config, camera);
  if (sample.observations == 0U) {
    SUCCEED() << "half-masked band refused rather than mis-published";
    return;
  }
  EXPECT_EQ(sample.observations, 1U);
  // Bounded error: well inside the 3 mm confirm recovery bound the full-silhouette test uses,
  // and nowhere near a radius (33 mm) of miss.
  EXPECT_LT(sample.translation_error_m, 0.003);
}

TEST(TrayDutyParameters, ChordCutShortArcIsRefusedRatherThanPublished)
{
  // A chord cut that leaves only a ~46 px vertical sliver of the product. After the gate
  // reorder the *radius* gate answers first on this geometry (mean_ratio far below the 0.50
  // floor for a sliver), so the axis-fit gate never runs — last_axis_fit_refusal stays empty
  // and no pose is published. Before the reorder the fit gate ran first and refused it with
  // the offset check; either order is fail-closed. The fit-gate reason itself is pinned by
  // AxisFitGatePinsItsRefusalReason.
  const auto config = confirm_config();
  const auto cylinder = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  const auto camera = confirm_camera_pose(cylinder.centre);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(cylinder);
  // Keep only a right-hand ~100-degree wedge about the image centre (sliver after both cuts).
  scene.occlude(0U, 320U, 0U, 360U);
  for (std::size_t row = 0U; row < 360U; ++row) {
    for (std::size_t column = 320U; column < 640U; ++column) {
      const double du = static_cast<double>(column) - 320.0;
      const double dv = static_cast<double>(row) - 180.0;
      if (dv > 1.2 * du || dv < -1.2 * du) {
        scene.occlude(column, column + 1U, row, row + 1U);
      }
    }
  }
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  auto observations = backend.value().estimate(detections, frame, framed(camera));
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty())
    << "a short-arc silhouette must be refused, not published (error would be ~1 radius)";
  EXPECT_EQ(backend.value().last_axis_fit_refusal_count(), 0U)
    << "radius gate answers this shape before the fit gate runs";
  EXPECT_TRUE(backend.value().last_axis_fit_refusal().empty());
}

TEST(AxisFitGate, SpanBelowHalfATurnIsRefusedWithPinnedReason)
{
  // Span predicate (axis_fit::refusal_reason, second check) on a synthetic fit: 90 degrees of
  // span about the band mean is a short arc. The string is pinned — it is what
  // perception_node logs and what the end-to-end tests match on.
  restocker_perception::axis_fit::AxisFit fit;
  fit.solved = true;
  fit.rank = 3;
  fit.span_rad = M_PI / 2.0;
  fit.centre = Eigen::Vector2d::Zero();
  fit.mean_radius = 0.033;
  fit.geometric_rms_residual = 1.0e-6;
  const char * reason = restocker_perception::axis_fit::refusal_reason(
    fit, Eigen::Vector2d::Zero(), 0.033);
  ASSERT_NE(reason, nullptr);
  EXPECT_STREQ(
    reason,
    "band silhouette spans less than half a turn about the band mean (short arc or chord cut)");
}

TEST(AxisFitGate, OffsetBeyondExtremeRadiusIsRefusedWithPinnedReason)
{
  // Offset predicate: a fitted centre further from the band mean than the band's own extreme
  // radius cannot be an axis inside that band (short-arc / chord-cut failure mode).
  restocker_perception::axis_fit::AxisFit fit;
  fit.solved = true;
  fit.rank = 3;
  fit.span_rad = M_PI;  // clears the span check so the offset predicate answers
  fit.centre = Eigen::Vector2d(0.040, 0.0);  // 40 mm off
  fit.mean_radius = 0.033;
  fit.geometric_rms_residual = 1.0e-6;
  const char * reason = restocker_perception::axis_fit::refusal_reason(
    fit, Eigen::Vector2d::Zero(), 0.033);  // extreme radius only 33 mm
  ASSERT_NE(reason, nullptr);
  EXPECT_STREQ(
    reason,
    "fitted axis lies further from the band mean than the band's extreme radius");
}

TEST(AxisFitGate, HealthyFullSilhouetteIsAccepted)
{
  restocker_perception::axis_fit::AxisFit fit;
  fit.solved = true;
  fit.rank = 3;
  fit.span_rad = 2.0 * M_PI * 0.95;
  fit.centre = Eigen::Vector2d::Zero();
  fit.mean_radius = 0.033;
  fit.geometric_rms_residual = 1.0e-6;
  EXPECT_EQ(
    restocker_perception::axis_fit::refusal_reason(
      fit, Eigen::Vector2d(0.005, 0.0), 0.033),
    nullptr);
}

TEST(AxisFitGate, RankDeficientFitIsRefusedWithPinnedReason)
{
  restocker_perception::axis_fit::AxisFit fit;
  fit.solved = false;
  fit.rank = 2;
  const char * reason = restocker_perception::axis_fit::refusal_reason(
    fit, Eigen::Vector2d::Zero(), 0.033);
  ASSERT_NE(reason, nullptr);
  EXPECT_STREQ(
    reason,
    "circle-fit design is degenerate (fewer than three hull vertices, or rank-deficient)");
}

TEST(TrayDutyParameters, MergedSameColourProductsAreRefused)
{
  // Two same-colour cans whose pixel masks touch form one peanut-shaped component. The
  // pre-existing extreme-radius gate (extreme_ratio >> the 1.40 cap) rejects it one stage before
  // the axis-fit gate is reached — last_axis_fit_refusal stays empty — so no pose is published
  // for the merged blob. (The fit-quality gate would also refuse it via the geometric residual
  // if the radius gate were ever loosened; this test pins the shipped order.)
  const auto config = confirm_config();
  const auto left = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  Cylinder right = left;
  // 40 mm apart centre-to-centre: the 33 mm radii overlap, so the pixel masks merge.
  right.centre.x() += 0.040;
  const auto camera = confirm_camera_pose(left.centre);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(left);
  scene.paint(right);
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U)
    << "the two cans must merge into one component for this test to mean anything";
  auto observations = backend.value().estimate(detections, frame, framed(camera));
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty())
    << "a merged same-colour blob must be refused, not published between the two products";
  EXPECT_TRUE(backend.value().last_axis_fit_refusal().empty())
    << "refused by the radius gate, before the axis-fit gate runs";
  EXPECT_EQ(backend.value().last_axis_fit_refusal_count(), 0U);
}

TEST(TrayDutyParameters, ConfirmSeparatesADenseSameColourColumnIntoItsProducts)
{
  // Card 063, from the first dense sensor-driven run: the dense tray stands cans 0.085 m apart in
  // depth. At the shipped confirm pose the rear can's barrel fills the 19 mm gap between the two
  // tops, so colour makes one component of the column. Split into its discs, each can is
  // published where it stands, and nothing is published between them.
  const auto config = confirm_config();
  const auto front = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  const auto camera = confirm_camera_pose(front.centre);
  // Behind the front can as the camera sees it: further along the viewing direction's XY.
  Eigen::Vector3d away = front.centre - camera.translation();
  away.z() = 0.0;
  away.normalize();
  Cylinder rear = front;
  rear.centre += 0.085 * away;
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(front);
  scene.paint(rear);
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U)
    << "the column must merge into one component for this test to mean anything";
  auto observations = backend.value().estimate(detections, frame, framed(camera));
  ASSERT_TRUE(observations.has_value());
  ASSERT_GE(observations.value().size(), 1U) << "the confirmed front can must be published";
  bool front_seen = false;
  for (const auto & observation : observations.value()) {
    const Eigen::Vector3d estimated(
      observation.pose.pose.position.x, observation.pose.pose.position.y,
      observation.pose.pose.position.z);
    const double to_front = (estimated - front.centre).norm();
    const double to_rear = (estimated - rear.centre).norm();
    EXPECT_LT(std::min(to_front, to_rear), 0.003) << "published between the two cans";
    front_seen = front_seen || to_front < 0.003;
  }
  EXPECT_TRUE(front_seen);
}

TEST(TrayDutyParameters, DepthNoiseKeepsTheAxisBounded)
{
  // ±0.5 mm deterministic depth noise on the hull: the fit is an extreme statistic, so this is
  // the regime where error could grow without a signed bias. Must stay well inside the 3 mm
  // confirm bound (review probe: sigma ~0.22 mm at 1 mm noise).
  const auto config = confirm_config();
  const auto cylinder = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  const auto camera = confirm_camera_pose(cylinder.centre);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(cylinder);
  scene.add_depth_noise(0.0005, 0xC036U);
  const auto sample = run_confirm_with_scene(scene, cylinder, config, camera);
  ASSERT_EQ(sample.observations, 1U);
  EXPECT_LT(sample.translation_error_m, 0.003);
}

// -- Card 065: refusal diagnostics (diagnostic only) --------------------------------------------

// The dense can column of Card 063's dev run 2 (cycle 6), seen from the live confirm pose aimed at
// the third can: the can under the camera is cut by the frame edge, its sliver fails the radial
// gate, and the all-or-nothing split refuses the whole column.
struct DiagnosticScene
{
  Eigen::Isometry3d camera;
  std::unique_ptr<RaycastScene> scene;
};

[[nodiscard]] DiagnosticScene dense_rear_view()
{
  std::vector<Cylinder> cans;
  for (const double y : {-0.765, -0.85, -0.935, -1.02}) {
    auto can = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
    can.centre.x() = -0.15;
    can.centre.y() = y;
    cans.push_back(can);
  }
  // The live confirm aims 0.05 m above the centre (confirm_aim_bias_xyz_m) from 0.1895 m ahead
  // and 0.2327 m above that point.
  const Eigen::Vector3d aim = cans[2].centre + Eigen::Vector3d(0.0, 0.0, 0.05);
  DiagnosticScene view;
  view.camera = look_at(aim + Eigen::Vector3d(0.0, 0.1895, 0.2327), aim);
  view.scene = std::make_unique<RaycastScene>(wrist_intrinsics(), view.camera);
  for (const auto & can : cans) {
    view.scene->paint(can);
  }
  return view;
}

struct Judged
{
  std::vector<ObservationMessage> observations;
  restocker_perception::EstimateDropCounts drops;
  std::size_t axis_fit_refusals{0U};
  std::vector<restocker_perception::ProposalDiagnostic> proposals;
  std::vector<restocker_perception::SmallComponentDiagnostic> small;
  restocker_perception::CentreCensus census;
  std::size_t detections{0U};
};

[[nodiscard]] Judged judge(
  const RaycastScene & scene, const Eigen::Isometry3d & camera, bool diagnostics)
{
  auto backend = ColourDepthBackend::create(confirm_config());
  EXPECT_TRUE(backend.has_value());
  EXPECT_FALSE(backend.value().diagnostics_enabled()) << "diagnostics must be off by default";
  backend.value().set_diagnostics_enabled(diagnostics);
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  auto observations = backend.value().estimate(detections, frame, framed(camera));
  EXPECT_TRUE(observations.has_value());
  Judged judged;
  judged.observations = observations.value();
  judged.drops = backend.value().last_estimate_drop_counts();
  judged.axis_fit_refusals = backend.value().last_axis_fit_refusal_count();
  judged.proposals = backend.value().last_proposal_diagnostics();
  judged.small = backend.value().last_small_components();
  judged.census = backend.value().last_centre_census();
  judged.detections = detections.detections.size();
  return judged;
}

void expect_same_decisions(const Judged & off, const Judged & on)
{
  ASSERT_EQ(off.observations.size(), on.observations.size());
  for (std::size_t index = 0; index < off.observations.size(); ++index) {
    EXPECT_EQ(
      off.observations[index].pose.pose.position.x,
      on.observations[index].pose.pose.position.x);
    EXPECT_EQ(
      off.observations[index].pose.pose.position.y,
      on.observations[index].pose.pose.position.y);
    EXPECT_EQ(
      off.observations[index].pose.pose.position.z,
      on.observations[index].pose.pose.position.z);
    EXPECT_EQ(off.observations[index].confidence, on.observations[index].confidence);
  }
  EXPECT_EQ(off.drops.too_few_points, on.drops.too_few_points);
  EXPECT_EQ(off.drops.extent, on.drops.extent);
  EXPECT_EQ(off.drops.top_face_points, on.drops.top_face_points);
  EXPECT_EQ(off.drops.radial_profile, on.drops.radial_profile);
  EXPECT_EQ(off.drops.axis_fit, on.drops.axis_fit);
  EXPECT_EQ(off.drops.nonfinite_centre, on.drops.nonfinite_centre);
  EXPECT_EQ(off.drops.split_overlap, on.drops.split_overlap);
  EXPECT_EQ(off.axis_fit_refusals, on.axis_fit_refusals);
}

TEST(RefusalDiagnostics, AreOffByDefaultAndRecordNothing)
{
  const auto view = dense_rear_view();
  const Judged off = judge(*view.scene, view.camera, false);
  EXPECT_GT(off.drops.total(), 0U) << "the scene must refuse something for this test to mean it";
  EXPECT_TRUE(off.proposals.empty());
  EXPECT_TRUE(off.small.empty());
  EXPECT_EQ(off.census.pixels, 0U);
}

TEST(RefusalDiagnostics, NameTheStageThatRefusedAColumnAndChangeNoDecision)
{
  const auto view = dense_rear_view();
  const Judged off = judge(*view.scene, view.camera, false);
  const Judged on = judge(*view.scene, view.camera, true);
  expect_same_decisions(off, on);
  ASSERT_EQ(on.proposals.size(), on.detections) << "one record per judged detection";
  const auto column = std::ranges::find_if(
    on.proposals, [](const restocker_perception::ProposalDiagnostic & proposal) {
      return proposal.verdict != "published";
    });
  ASSERT_NE(column, on.proposals.end());
  EXPECT_EQ(column->verdict, "radial_profile") << column->detail;
  EXPECT_GE(column->discs, 2U);
  EXPECT_NEAR(column->centroid.x(), -0.15, 0.02);
  EXPECT_NE(column->detail.find("radial_profile mean="), std::string::npos) << column->detail;
  // The discs the refusal skipped are judged uncounted: the fully visible target passes.
  EXPECT_NE(column->detail.find("ok"), std::string::npos) << column->detail;
  // The confirm view aims at its target, so the image centre is that can: classified as a can
  // (signature 0), in the depth band, nothing too dark.
  const auto side = 2U * ColourDepthBackend::kCentreCensusHalfWidthPx;
  EXPECT_EQ(on.census.pixels, side * side);
  ASSERT_EQ(on.census.classified.size(), 3U);
  EXPECT_EQ(on.census.classified[0], on.census.pixels) << "every centre pixel is the target can";
  EXPECT_EQ(on.census.outside_depth_band + on.census.too_dark + on.census.no_signature, 0U);
  EXPECT_NEAR(on.census.mean_red, 200.0, 1.0);
}

TEST(RefusalDiagnostics, RecordAComponentDetectDroppedForSizeAndChangeNoDecision)
{
  // One can, mostly occluded: the visible sliver is a same-class component below the confirm
  // duty's 200-pixel floor, which detect() drops without a trace unless diagnostics are on.
  const auto can = upright_on_tray(0.033, 0.122, {200U, 80U, 70U});
  const auto camera = confirm_camera_pose(can.centre);
  RaycastScene scene(wrist_intrinsics(), camera);
  scene.paint(can);
  // Keep a 15 x 10 pixel window at the image centre, where the camera aims: 150 pixels.
  scene.occlude(0U, 640U, 0U, 175U);
  scene.occlude(0U, 640U, 185U, 360U);
  scene.occlude(0U, 315U, 175U, 185U);
  scene.occlude(330U, 640U, 175U, 185U);
  const Judged off = judge(scene, camera, false);
  const Judged on = judge(scene, camera, true);
  expect_same_decisions(off, on);
  EXPECT_EQ(on.detections, 0U);
  ASSERT_EQ(on.small.size(), 1U);
  EXPECT_EQ(on.small[0].product_class, DetectionMessage::PRODUCT_CLASS_CAN);
  EXPECT_GT(on.small[0].pixels, 0U);
  EXPECT_LT(on.small[0].pixels, 200U);
  EXPECT_GE(on.small[0].y_min, 175U);
  EXPECT_LE(on.small[0].y_max, 185U);
}

// -- Card 065: a column's partly hidden far end does not refuse its front -----------------------

[[nodiscard]] restocker_perception::JudgedDisc disc_at(
  double x, double y, double radius, bool admitted, std::size_t samples = 24U)
{
  restocker_perception::JudgedDisc disc;
  disc.admitted = admitted;
  disc.fitted_centre = Eigen::Vector2d(x, y);
  for (std::size_t index = 0; index < samples; ++index) {
    const double angle = 2.0 * M_PI * static_cast<double>(index) / static_cast<double>(samples);
    disc.points.emplace_back(x + radius * std::cos(angle), y + radius * std::sin(angle));
  }
  return disc;
}

TEST(FarEndSetAside, AFarPartialDiscClearOfEveryAdmittedFootprintIsSetAside)
{
  // The front2 capture: camera 0.19 m ahead of the front; three bottles pass, the far one's
  // partial top (a sliver around y -0.998) fails.
  const Eigen::Vector2d camera(0.43, -0.576);
  std::vector<restocker_perception::JudgedDisc> discs{
    disc_at(0.43, -0.765, 0.034, true), disc_at(0.43, -0.85, 0.034, true),
    disc_at(0.43, -0.935, 0.034, true), disc_at(0.44, -0.998, 0.012, false)};
  EXPECT_TRUE(restocker_perception::failed_discs_can_be_set_aside(discs, camera, 0.034));
}

TEST(FarEndSetAside, AFailingDiscNearerThanAnAdmittedOneRefuses)
{
  // Card 065's rear-row case: the sliver under the camera is the nearest disc.
  const Eigen::Vector2d camera(-0.15, -0.745);
  std::vector<restocker_perception::JudgedDisc> discs{
    disc_at(-0.15, -0.795, 0.006, false), disc_at(-0.15, -0.845, 0.033, true),
    disc_at(-0.15, -0.935, 0.033, true)};
  EXPECT_FALSE(restocker_perception::failed_discs_can_be_set_aside(discs, camera, 0.033));
}

TEST(FarEndSetAside, AFailingDiscInsideAnAdmittedFootprintRefuses)
{
  // Farther than the admitted disc, but its points reach within 1.5 radii of the admitted
  // centre: it may be the same product's own top or barrel strip (a tilted product's fragment).
  const Eigen::Vector2d camera(0.0, 0.0);
  std::vector<restocker_perception::JudgedDisc> discs{
    disc_at(0.0, -0.19, 0.033, true), disc_at(0.0, -0.245, 0.012, false)};
  EXPECT_FALSE(restocker_perception::failed_discs_can_be_set_aside(discs, camera, 0.033));
}

TEST(FarEndSetAside, NothingAdmittedOrNothingFailedIsNotASetAside)
{
  const Eigen::Vector2d camera(0.0, 0.0);
  const std::vector<restocker_perception::JudgedDisc> none_admitted{
    disc_at(0.0, -0.2, 0.033, false), disc_at(0.0, -0.3, 0.033, false)};
  EXPECT_FALSE(restocker_perception::failed_discs_can_be_set_aside(none_admitted, camera, 0.033));
  const std::vector<restocker_perception::JudgedDisc> all_admitted{
    disc_at(0.0, -0.2, 0.033, true), disc_at(0.0, -0.3, 0.033, true)};
  EXPECT_FALSE(restocker_perception::failed_discs_can_be_set_aside(all_admitted, camera, 0.033));
}

// The front2 capture's scene: the four-deep small-bottle column at x = 0.43, seen from the live
// confirm pose aimed at its front. The far bottle stands 15 mm farther back than the fixture's
// -1.02, which hides its top the way the live render does (flat shading at -1.02 admits it
// narrowly); its partial disc fails the radial gate.
[[nodiscard]] DiagnosticScene small_bottle_column_front_view(double far_y)
{
  std::vector<Cylinder> bottles;
  for (const double y : {-0.765, -0.85, -0.935, far_y}) {
    auto bottle = upright_on_tray(0.034, 0.200, {70U, 130U, 190U});
    bottle.centre.x() = 0.43;
    bottle.centre.y() = y;
    bottles.push_back(bottle);
  }
  const Eigen::Vector3d aim = bottles[0].centre + Eigen::Vector3d(0.0, 0.0, 0.05);
  DiagnosticScene view;
  view.camera = look_at(aim + Eigen::Vector3d(0.0, 0.1895, 0.2327), aim);
  view.scene = std::make_unique<RaycastScene>(wrist_intrinsics(), view.camera);
  for (const auto & bottle : bottles) {
    view.scene->paint(bottle);
  }
  return view;
}

TEST(FarEndSetAside, ConfirmPublishesAColumnFrontWhenItsFarEndIsPartlyHidden)
{
  const auto view = small_bottle_column_front_view(-1.035);
  const Judged judged = judge(*view.scene, view.camera, true);
  ASSERT_EQ(judged.proposals.size(), 1U) << "the column must be one component";
  ASSERT_GE(judged.proposals[0].discs, 4U);
  EXPECT_NE(judged.proposals[0].detail.find("radial_profile"), std::string::npos)
    << "the far disc must fail for this test to mean anything: " << judged.proposals[0].detail;
  const Eigen::Vector3d front(0.43, -0.765, kTrayFloorZ + 0.100);
  bool front_published = false;
  for (const auto & observation : judged.observations) {
    const Eigen::Vector3d at(
      observation.pose.pose.position.x, observation.pose.pose.position.y,
      observation.pose.pose.position.z);
    front_published = front_published || (at - front).norm() < 0.003;
    EXPECT_GT(std::abs(at.y() - (-1.035)), 0.05) << "the partial far disc must not be published";
  }
  EXPECT_TRUE(front_published) << judged.proposals[0].detail;
  EXPECT_GE(judged.drops.radial_profile, 1U) << "the set-aside disc still counts its refusal";
}

TEST(FarEndSetAside, AWhollyVisibleColumnIsUnchanged)
{
  // At the fixture's -1.02 the flat far disc passes: every disc is published, as before.
  const auto view = small_bottle_column_front_view(-1.02);
  const Judged judged = judge(*view.scene, view.camera, false);
  EXPECT_EQ(judged.observations.size(), 4U);
  EXPECT_EQ(judged.drops.total(), 0U);
}

TEST(FarEndSetAside, AnEmptyFrontPublishesNothingAtItsPlaceAndItsNeighbourWhereItStands)
{
  // Review note N1: the front bottle is gone and the confirm camera still aims at its place. The
  // rest of the column is admitted (the partial far disc set aside), so the neighbour one pitch
  // back is published at its own measured position and nothing is published at the empty front.
  // The survey's 0.10 m association radius is wider than the 0.085 m pitch, so CONFIRM then
  // confirms the candidate *as that neighbour* (pinned in test_tray_survey_logic); the campaign
  // acts on the neighbour's own identity and pose.
  std::vector<Cylinder> bottles;
  for (const double y : {-0.85, -0.935, -1.035}) {
    auto bottle = upright_on_tray(0.034, 0.200, {70U, 130U, 190U});
    bottle.centre.x() = 0.43;
    bottle.centre.y() = y;
    bottles.push_back(bottle);
  }
  const Eigen::Vector3d empty_front(0.43, -0.765, kTrayFloorZ + 0.100);
  const Eigen::Vector3d aim = empty_front + Eigen::Vector3d(0.0, 0.0, 0.05);
  const auto camera = look_at(aim + Eigen::Vector3d(0.0, 0.1895, 0.2327), aim);
  RaycastScene scene(wrist_intrinsics(), camera);
  for (const auto & bottle : bottles) {
    scene.paint(bottle);
  }
  const Judged judged = judge(scene, camera, true);
  bool neighbour_published = false;
  for (const auto & observation : judged.observations) {
    const Eigen::Vector3d at(
      observation.pose.pose.position.x, observation.pose.pose.position.y,
      observation.pose.pose.position.z);
    EXPECT_GT((at - empty_front).norm(), 0.03) << "nothing may be published at the empty front";
    neighbour_published = neighbour_published || (at - bottles[0].centre).norm() < 0.003;
  }
  EXPECT_TRUE(neighbour_published);
}

// Review note N2: condition 3 at the live 0.085 m pitch, on the discs themselves. The admitted
// neighbour's fitted centre stands one pitch (2.5 catalogued radii) ahead of the far product, so
// the far product's near rim is exactly 1.5 radii from it: the margin is the part of the near rim
// the view loses. A far disc that keeps its whole near rim stays refused (clearance == 1.5 r);
// one that has lost 2 mm of it is set aside.
[[nodiscard]] std::vector<restocker_perception::JudgedDisc> live_pitch_column(double far_near_y)
{
  std::vector<restocker_perception::JudgedDisc> discs;
  for (const double y : {-0.765, -0.85, -0.935}) {
    restocker_perception::JudgedDisc admitted;
    admitted.admitted = true;
    admitted.fitted_centre = Eigen::Vector2d(0.43, y);
    admitted.points.emplace_back(0.43, y);
    discs.push_back(admitted);
  }
  // The far top (centre y -1.02, r 0.034) seen from its near rim to the live centroid's far
  // side (y -1.008), on a 1 mm grid.
  // Integer millimetre offsets from the far centre, so the near rim point (0, +34 mm) is exact.
  restocker_perception::JudgedDisc far;
  const int near_mm = static_cast<int>(std::lround((far_near_y + 1.02) * 1000.0));
  for (int dx = -34; dx <= 34; ++dx) {
    for (int dy = 12; dy <= near_mm; ++dy) {
      if (dx * dx + dy * dy <= 34 * 34) {
        far.points.emplace_back(0.43 + dx * 0.001, -1.02 + dy * 0.001);
      }
    }
  }
  discs.push_back(far);
  return discs;
}

TEST(FarEndSetAside, AtTheLivePitchTheMarginIsTheNearRimTheViewLoses)
{
  const Eigen::Vector2d camera(0.43, -0.576);
  const auto whole_rim = restocker_perception::judge_failed_discs(
    live_pitch_column(-0.986), camera, 0.034);
  EXPECT_EQ(whole_rim.answer, restocker_perception::SetAsideAnswer::kInsideAdmittedFootprint);
  EXPECT_NEAR(whole_rim.minimum_clearance_m, 1.5 * 0.034, 0.001);
  const auto lost_two_mm = restocker_perception::judge_failed_discs(
    live_pitch_column(-0.988), camera, 0.034);
  EXPECT_EQ(lost_two_mm.answer, restocker_perception::SetAsideAnswer::kSetAside);
  EXPECT_GT(lost_two_mm.minimum_clearance_m, 1.5 * 0.034);
}

}  // namespace
