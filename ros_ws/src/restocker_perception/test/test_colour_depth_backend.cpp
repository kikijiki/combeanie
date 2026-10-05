// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
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

constexpr const char * kCameraFrame = "camera_optical";
constexpr const char * kPlanningFrame = "world";

// A camera looking straight down from altitude_m, in the optical convention (+Z view axis, +X
// right, +Y down). The nadir view exercises the full optical-to-planning rotation.
[[nodiscard]] FramedTransform nadir_camera(double altitude_m)
{
  Eigen::Isometry3d planning_from_camera = Eigen::Isometry3d::Identity();
  planning_from_camera.translation() = Eigen::Vector3d(0.0, 0.0, altitude_m);
  Eigen::Matrix3d rotation;
  rotation.col(0) = Eigen::Vector3d(1.0, 0.0, 0.0);    // optical +X -> planning +X
  rotation.col(1) = Eigen::Vector3d(0.0, -1.0, 0.0);   // optical +Y -> planning -Y
  rotation.col(2) = Eigen::Vector3d(0.0, 0.0, -1.0);   // optical +Z -> planning -Z
  planning_from_camera.linear() = rotation;
  auto transform =
    FramedTransform::create(kCameraFrame, kPlanningFrame, planning_from_camera);
  EXPECT_TRUE(transform.has_value());
  return transform.value();
}

[[nodiscard]] CameraIntrinsics test_intrinsics()
{
  return CameraIntrinsics{400.0, 400.0, 160.0, 120.0, 320, 240};
}

[[nodiscard]] ColourDepthConfig test_config()
{
  ColourDepthConfig config;
  config.workspace.minimum = Eigen::Vector3d(-1.0, -1.0, -0.1);
  config.workspace.maximum = Eigen::Vector3d(1.0, 1.0, 1.5);
  config.minimum_component_pixels = 20;
  config.minimum_top_face_points = 10;
  ProductSignature can;
  can.product_class = DetectionMessage::PRODUCT_CLASS_CAN;
  can.sku = "SIM-CAN-STD";
  can.reference_rgb = {200.0, 80.0, 70.0};
  can.nominal_radius_m = 0.033;
  can.nominal_height_m = 0.122;
  ProductSignature bottle;
  bottle.product_class = DetectionMessage::PRODUCT_CLASS_SMALL_BOTTLE;
  bottle.sku = "SIM-BOTTLE-SMALL";
  bottle.reference_rgb = {70.0, 130.0, 190.0};
  bottle.nominal_radius_m = 0.034;
  bottle.nominal_height_m = 0.200;
  config.signatures = {can, bottle};
  return config;
}

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

// Paints a filled disc of one colour at one depth: an upright cylinder's top face seen from above.
class SyntheticScene
{
public:
  explicit SyntheticScene(CameraIntrinsics intrinsics)
  : intrinsics_(intrinsics)
  {
    colour_ = std::make_shared<sensor_msgs::msg::Image>();
    colour_->header.frame_id = kCameraFrame;
    colour_->header.stamp.sec = 12;
    colour_->header.stamp.nanosec = 500000000U;
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
    // Far background so pixels off a product still carry a depth return.
    for (std::size_t index = 0; index < depth_->data.size() / 4U; ++index) {
      write_depth(index, 1.4F);
    }
  }

  void paint_disc(
    double centre_u, double centre_v, double radius_px, double depth_m,
    std::array<std::uint8_t, 3> colour)
  {
    for (std::size_t row = 0; row < colour_->height; ++row) {
      for (std::size_t column = 0; column < colour_->width; ++column) {
        const double du = static_cast<double>(column) - centre_u;
        const double dv = static_cast<double>(row) - centre_v;
        if (std::hypot(du, dv) > radius_px) {
          continue;
        }
        std::uint8_t * pixel = colour_->data.data() + row * colour_->step + column * 3U;
        pixel[0] = colour[0];
        pixel[1] = colour[1];
        pixel[2] = colour[2];
        write_depth(row * (depth_->step / 4U) + column, static_cast<float>(depth_m));
      }
    }
  }

  // A flat patch of one colour at one depth: the visible strip of a barrel, for example.
  void paint_rectangle(
    std::size_t u_min, std::size_t u_max, std::size_t v_min, std::size_t v_max, double depth_m,
    std::array<std::uint8_t, 3> colour)
  {
    for (std::size_t row = v_min; row < v_max; ++row) {
      for (std::size_t column = u_min; column < u_max; ++column) {
        std::uint8_t * pixel = colour_->data.data() + row * colour_->step + column * 3U;
        pixel[0] = colour[0];
        pixel[1] = colour[1];
        pixel[2] = colour[2];
        write_depth(row * (depth_->step / 4U) + column, static_cast<float>(depth_m));
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
  void write_depth(std::size_t index, float metres)
  {
    std::memcpy(depth_->data.data() + index * sizeof(float), &metres, sizeof(float));
  }

  CameraIntrinsics intrinsics_;
  sensor_msgs::msg::Image::SharedPtr colour_;
  sensor_msgs::msg::Image::SharedPtr depth_;
};

TEST(ColourDepthBackend, RejectsAConfigurationItCannotHonour)
{
  auto empty = test_config();
  empty.signatures.clear();
  EXPECT_FALSE(ColourDepthBackend::create(empty).has_value());

  auto inverted = test_config();
  inverted.workspace.maximum = inverted.workspace.minimum;
  EXPECT_FALSE(ColourDepthBackend::create(inverted).has_value());

  auto sizeless = test_config();
  sizeless.signatures.front().nominal_radius_m = 0.0;
  EXPECT_FALSE(ColourDepthBackend::create(sizeless).has_value());

  // Without a SKU every observation is refused by lane policy at the far end of the pipeline.
  auto anonymous = test_config();
  anonymous.signatures.front().sku.clear();
  EXPECT_FALSE(ColourDepthBackend::create(anonymous).has_value());
}

TEST(ColourDepthBackend, DetectionsCarryTheAcquisitionStampAndFrameVerbatim)
{
  SyntheticScene scene(test_intrinsics());
  scene.paint_disc(160.0, 120.0, 12.0, 1.0, {200, 80, 70});
  auto backend = ColourDepthBackend::create(test_config());
  ASSERT_TRUE(backend.has_value());

  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  EXPECT_EQ(detections.status, restocker_interfaces::msg::PerceptionFrame::STATUS_OK);
  EXPECT_EQ(detections.header.frame_id, frame.frame_id());
  EXPECT_EQ(rclcpp::Time(detections.header.stamp, RCL_ROS_TIME), frame.stamp());
  ASSERT_EQ(detections.detections.size(), 1U);
  EXPECT_EQ(detections.detections.front().product_class, DetectionMessage::PRODUCT_CLASS_CAN);
  // The mask covers its bounding box, as validate_estimation_inputs requires.
  EXPECT_TRUE(detections.detections.front().has_mask);
  std::uint64_t covered = 0;
  for (const std::uint32_t run : detections.detections.front().mask.rle_counts) {
    covered += run;
  }
  EXPECT_EQ(
    covered,
    static_cast<std::uint64_t>(detections.detections.front().mask.width) *
    detections.detections.front().mask.height);
}

TEST(ColourDepthBackend, RecoversTheCentreOfAnUprightCylinderFromItsTopFace)
{
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  // Top face 1.0 m from a camera 1.0 m up, so the top is at planning z = 0 and the centre half a
  // can-height below it. The offset from the principal point checks the deprojection.
  const double radius_px = 0.033 * intrinsics.fx / 1.0;
  scene.paint_disc(160.0 + 40.0, 120.0 - 24.0, radius_px, 1.0, {200, 80, 70});

  auto backend = ColourDepthBackend::create(test_config());
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U);

  auto observations = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value()) << observations.error().detail;
  ASSERT_EQ(observations.value().size(), 1U);
  const auto & observation = observations.value().front();

  EXPECT_EQ(observation.header.frame_id, kPlanningFrame);
  EXPECT_EQ(rclcpp::Time(observation.header.stamp, RCL_ROS_TIME), frame.stamp());
  EXPECT_EQ(observation.product_class, DetectionMessage::PRODUCT_CLASS_CAN);
  EXPECT_TRUE(observation.has_sku);
  EXPECT_EQ(observation.sku, "SIM-CAN-STD");
  EXPECT_EQ(observation.orientation, ObservationMessage::ORIENTATION_UPRIGHT);
  EXPECT_EQ(observation.status, ObservationMessage::STATUS_OK);
  // Identity is not assigned at this stage.
  EXPECT_TRUE(observation.source_object_id.empty());

  EXPECT_NEAR(observation.pose.pose.position.x, 40.0 / intrinsics.fx, 2.0e-3);
  EXPECT_NEAR(observation.pose.pose.position.y, 24.0 / intrinsics.fy, 2.0e-3);
  EXPECT_NEAR(observation.pose.pose.position.z, -0.122 / 2.0, 2.0e-3);
  EXPECT_NEAR(observation.pose.pose.orientation.w, 1.0, 1.0e-9);

  // The orientation is a constant. All three rotational variances are that of a uniform
  // distribution over a full turn, which declares them unmeasured (see declares_axis_estimate).
  EXPECT_FALSE(
    restocker_perception::declares_axis_estimate(covariance_of(observation)));
  for (const std::size_t entry : {21U, 28U, 35U}) {
    EXPECT_DOUBLE_EQ(
      observation.pose.covariance[entry], restocker_perception::kUnmeasuredRotationVariance);
  }
  EXPECT_LT(observation.pose.covariance[0], 1.0e-3);
  EXPECT_GT(observation.confidence, 0.5F);
  EXPECT_LE(observation.confidence, 1.0F);
}

// Card 063 (Milestone 10 section 6 amendment): two same-colour cans one pitch apart, the gap
// between their tops filled by the rear can's barrel a few centimetres lower, which is what the
// wrist confirm view of a dense tray column sees. The colour stage cannot help making one
// component of that; the top band then holds two separate discs, and each is judged on its own.
TEST(ColourDepthBackend, JudgesEachSeparateTopDiscOfOneColourComponentOnItsOwn)
{
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  const double radius_px = 0.033 * intrinsics.fx / 1.0;
  // Centres 0.086 m apart: a 20 mm gap between the two 66 mm tops.
  const double half_pitch_px = 0.043 * intrinsics.fx;
  scene.paint_disc(160.0 - half_pitch_px, 120.0, radius_px, 1.0, {200, 80, 70});
  scene.paint_disc(160.0 + half_pitch_px, 120.0, radius_px, 1.0, {200, 80, 70});
  scene.paint_rectangle(
    static_cast<std::size_t>(160.0 - half_pitch_px),
    static_cast<std::size_t>(160.0 + half_pitch_px),
    114U, 126U, 1.05, {200, 80, 70});

  const auto frame = scene.frame();
  // A duty that keeps the band whole (the overhead pipeline) refuses the column, as before.
  auto whole = ColourDepthBackend::create(test_config());
  ASSERT_TRUE(whole.has_value());
  const auto merged = whole.value().detect(frame);
  ASSERT_EQ(merged.detections.size(), 1U) << "the colour stage merges the column";
  auto refused = whole.value().estimate(merged, frame, nadir_camera(1.0));
  ASSERT_TRUE(refused.has_value());
  EXPECT_TRUE(refused.value().empty());

  auto config = test_config();
  config.top_face_split_gap_m = 0.008;
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U);

  auto observations = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value()) << observations.error().detail;
  ASSERT_EQ(observations.value().size(), 2U);
  std::vector<double> xs{
    observations.value()[0].pose.pose.position.x, observations.value()[1].pose.pose.position.x};
  std::ranges::sort(xs);
  EXPECT_NEAR(xs[0], -0.043, 2.0e-3);
  EXPECT_NEAR(xs[1], 0.043, 2.0e-3);
  for (const auto & observation : observations.value()) {
    EXPECT_NEAR(observation.pose.pose.position.y, 0.0, 2.0e-3);
    EXPECT_NEAR(observation.pose.pose.position.z, -0.122 / 2.0, 2.0e-3);
  }
}

TEST(ColourDepthBackend, StillRefusesTwoTopDiscsThatTouch)
{
  // Card 004's touching pair (spec Q11): no empty gap between the tops, so there is nothing to
  // separate them by, and the two-disc band is refused as it always was.
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  const double radius_px = 0.033 * intrinsics.fx / 1.0;
  scene.paint_disc(160.0 - radius_px, 120.0, radius_px, 1.0, {200, 80, 70});
  scene.paint_disc(160.0 + radius_px, 120.0, radius_px, 1.0, {200, 80, 70});

  auto config = test_config();
  config.top_face_split_gap_m = 0.008;
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U);
  auto observations = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value()) << observations.error().detail;
  EXPECT_TRUE(observations.value().empty());
}

TEST(ColourDepthBackend, RefusesTheWholeColumnWhenOneOfItsDiscsIsNotAProduct)
{
  // All or nothing: a column whose second "disc" is a small same-colour square at top height is
  // not two products, and the good disc beside it is not published either.
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  const double radius_px = 0.033 * intrinsics.fx / 1.0;
  const double half_pitch_px = 0.043 * intrinsics.fx;
  scene.paint_disc(160.0 - half_pitch_px, 120.0, radius_px, 1.0, {200, 80, 70});
  scene.paint_rectangle(
    static_cast<std::size_t>(160.0 + half_pitch_px - 4.0),
    static_cast<std::size_t>(160.0 + half_pitch_px + 4.0), 100U, 140U, 1.0, {200, 80, 70});
  scene.paint_rectangle(
    static_cast<std::size_t>(160.0 - half_pitch_px),
    static_cast<std::size_t>(160.0 + half_pitch_px),
    114U, 126U, 1.05, {200, 80, 70});
  auto config = test_config();
  config.top_face_split_gap_m = 0.008;
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U);
  auto observations = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty());
  EXPECT_GT(backend.value().last_estimate_drop_counts().total(), 0U);
}

TEST(ColourDepthBackend, RefusesANegativeOrNonFiniteSplitGap)
{
  auto config = test_config();
  config.top_face_split_gap_m = -0.001;
  EXPECT_FALSE(ColourDepthBackend::create(config).has_value());
  config.top_face_split_gap_m = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(ColourDepthBackend::create(config).has_value());
}

TEST(ColourDepthBackend, RejectsAColouredThingThatIsNotACylinderOfThatRadius)
{
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  // Four times the can's radius in the can's colour, as the robot's blue links look against the
  // small-bottle signature.
  scene.paint_disc(160.0, 120.0, 4.0 * 0.033 * intrinsics.fx, 1.0, {200, 80, 70});

  auto backend = ColourDepthBackend::create(test_config());
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  const auto detections = backend.value().detect(frame);
  ASSERT_EQ(detections.detections.size(), 1U) << "the colour stage should still propose it";
  auto observations = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty()) << "the geometry stage should dispose of it";
}

TEST(ColourDepthBackend, RejectsAProductOutsideTheWorkspace)
{
  const auto intrinsics = test_intrinsics();
  SyntheticScene scene(intrinsics);
  scene.paint_disc(160.0, 120.0, 0.033 * intrinsics.fx / 1.0, 1.0, {200, 80, 70});
  auto config = test_config();
  // Workspace shifted so the product is outside it.
  config.workspace.minimum = Eigen::Vector3d(-1.0, -1.0, 0.5);
  config.workspace.maximum = Eigen::Vector3d(1.0, 1.0, 1.5);
  auto backend = ColourDepthBackend::create(config);
  ASSERT_TRUE(backend.has_value());
  const auto frame = scene.frame();
  auto observations =
    backend.value().estimate(backend.value().detect(frame), frame, nadir_camera(1.0));
  ASSERT_TRUE(observations.has_value());
  EXPECT_TRUE(observations.value().empty());
}

TEST(ColourDepthBackend, RefusesToEstimateAgainstAnotherAcquisition)
{
  const auto intrinsics = test_intrinsics();
  SyntheticScene first(intrinsics);
  first.paint_disc(160.0, 120.0, 0.033 * intrinsics.fx, 1.0, {200, 80, 70});
  auto backend = ColourDepthBackend::create(test_config());
  ASSERT_TRUE(backend.has_value());
  const auto frame = first.frame();
  auto detections = backend.value().detect(frame);

  detections.header.stamp.sec += 1;
  const auto mismatched = backend.value().estimate(detections, frame, nadir_camera(1.0));
  ASSERT_FALSE(mismatched.has_value());
  EXPECT_EQ(mismatched.error().code, restocker_perception::PerceptionErrorCode::OutOfOrder);

  detections = backend.value().detect(frame);
  auto wrong_frame = FramedTransform::create(
    "some_other_frame", kPlanningFrame,
    Eigen::Isometry3d::Identity());
  ASSERT_TRUE(wrong_frame.has_value());
  const auto rejected = backend.value().estimate(detections, frame, wrong_frame.value());
  ASSERT_FALSE(rejected.has_value());
  EXPECT_EQ(rejected.error().code, restocker_perception::PerceptionErrorCode::FrameMismatch);
}

}  // namespace
