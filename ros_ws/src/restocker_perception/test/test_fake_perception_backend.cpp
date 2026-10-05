// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "restocker_perception/fake_perception_backend.hpp"

namespace
{

using restocker_perception::CameraIntrinsics;
using restocker_perception::FakePerceptionBackend;
using restocker_perception::FakePerceptionConfig;
using restocker_perception::FramedTransform;
using restocker_perception::PerceptionErrorCode;
using restocker_perception::RgbdFrame;
using restocker_perception::ScriptedObject;
using restocker_perception::make_synthetic_rgbd_frame;
using PerceptionFrameMessage = restocker_interfaces::msg::PerceptionFrame;

constexpr const char * kCameraFrame = "camera_optical";
constexpr const char * kPlanningFrame = "world";
constexpr std::int64_t kStampNs = 1'000'000'000;

[[nodiscard]] CameraIntrinsics intrinsics()
{
  return CameraIntrinsics{320.0, 320.0, 160.0, 120.0, 320, 240};
}

[[nodiscard]] FakePerceptionConfig scripted_config()
{
  ScriptedObject can;
  can.source_object_id = "can_0";
  can.instance_id = 7;
  can.pose_in_camera.translation() = Eigen::Vector3d(0.10, 0.0, 0.60);
  can.detection_score = 0.99F;
  can.pose_confidence = 0.995F;
  can.x_min = 100;
  can.y_min = 80;
  can.x_max = 140;
  can.y_max = 160;

  FakePerceptionConfig config;
  config.intrinsics = intrinsics();
  config.objects.push_back(can);
  return config;
}

[[nodiscard]] RgbdFrame acquisition(std::int64_t stamp_ns = kStampNs)
{
  auto frame = make_synthetic_rgbd_frame(
    intrinsics(), kCameraFrame, rclcpp::Time(stamp_ns, RCL_ROS_TIME));
  EXPECT_TRUE(frame);
  return frame.value();
}

[[nodiscard]] FramedTransform camera_to_planning()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.5, 0.0, 1.2);
  auto created = FramedTransform::create(kCameraFrame, kPlanningFrame, transform);
  EXPECT_TRUE(created);
  return created.value();
}

TEST(RgbdFrame, RejectsColourAndDepthThatDisagreeAboutFrameOrInstant)
{
  auto colour = std::make_shared<sensor_msgs::msg::Image>();
  colour->header.frame_id = kCameraFrame;
  colour->header.stamp = rclcpp::Time(kStampNs, RCL_ROS_TIME);
  colour->width = 320;
  colour->height = 240;

  auto other_frame = std::make_shared<sensor_msgs::msg::Image>(*colour);
  other_frame->header.frame_id = "some_other_optical";
  const auto mismatched_frame = RgbdFrame::create(intrinsics(), colour, other_frame);
  ASSERT_FALSE(mismatched_frame);
  EXPECT_EQ(mismatched_frame.error().code, PerceptionErrorCode::FrameMismatch);

  auto other_instant = std::make_shared<sensor_msgs::msg::Image>(*colour);
  other_instant->header.stamp = rclcpp::Time(kStampNs + 5'000'000, RCL_ROS_TIME);
  const auto mismatched_stamp = RgbdFrame::create(intrinsics(), colour, other_instant);
  ASSERT_FALSE(mismatched_stamp);
  EXPECT_EQ(mismatched_stamp.error().code, PerceptionErrorCode::OutOfOrder);

  EXPECT_FALSE(RgbdFrame::create(intrinsics(), nullptr, colour));
}

TEST(FakeDetection, CopiesTheAcquisitionFrameAndStampVerbatim)
{
  FakePerceptionBackend backend(scripted_config());
  const auto frame = acquisition();

  const auto detections = backend.detect(frame);
  EXPECT_EQ(detections.status, PerceptionFrameMessage::STATUS_OK);
  EXPECT_EQ(detections.header.frame_id, kCameraFrame);
  EXPECT_EQ(rclcpp::Time(detections.header.stamp, RCL_ROS_TIME), frame.stamp());
  EXPECT_EQ(detections.image_width, 320);
  EXPECT_EQ(detections.image_height, 240);
  ASSERT_EQ(detections.detections.size(), 1U);
  EXPECT_EQ(detections.detections.front().source_object_id, "can_0");
  EXPECT_EQ(detections.detections.front().instance_id, 7U);
}

TEST(FakeDetection, EmitsAMaskThatCoversItsOwnBoundingBoxExactly)
{
  FakePerceptionBackend backend(scripted_config());
  const auto detections = backend.detect(acquisition());

  ASSERT_EQ(detections.detections.size(), 1U);
  const auto & detection = detections.detections.front();
  ASSERT_TRUE(detection.has_mask);
  EXPECT_EQ(detection.mask.width, detection.x_max - detection.x_min);
  EXPECT_EQ(detection.mask.height, detection.y_max - detection.y_min);

  std::uint64_t covered = 0;
  for (const auto run : detection.mask.rle_counts) {
    covered += run;
  }
  EXPECT_EQ(covered, static_cast<std::uint64_t>(detection.mask.width) * detection.mask.height);
}

TEST(FakeDetection, ReportsBackendFailureThroughStatusRatherThanEmptyDetections)
{
  FakePerceptionBackend backend(scripted_config());
  backend.fail_detection_with(
    PerceptionFrameMessage::STATUS_BACKEND_UNAVAILABLE, "inference service is down");

  const auto detections = backend.detect(acquisition());
  EXPECT_EQ(detections.status, PerceptionFrameMessage::STATUS_BACKEND_UNAVAILABLE);
  EXPECT_EQ(detections.status_detail, "inference service is down");
  EXPECT_TRUE(detections.detections.empty());

  // A failed detection frame can never be turned into observations.
  const auto estimated = backend.estimate(detections, acquisition(), camera_to_planning());
  ASSERT_FALSE(estimated);
  EXPECT_EQ(estimated.error().code, PerceptionErrorCode::InvalidArgument);
}

TEST(FakePoseEstimation, ProducesPlanningFrameObservationsStampedAtTheAcquisition)
{
  FakePerceptionBackend backend(scripted_config());
  const auto frame = acquisition();
  const auto detections = backend.detect(frame);

  const auto estimated = backend.estimate(detections, frame, camera_to_planning());
  ASSERT_TRUE(estimated);
  ASSERT_EQ(estimated.value().size(), 1U);

  const auto & observation = estimated.value().front();
  // Observations leave the backend in the planning frame, the only frame the world state admits.
  EXPECT_EQ(observation.header.frame_id, kPlanningFrame);
  EXPECT_EQ(rclcpp::Time(observation.header.stamp, RCL_ROS_TIME), frame.stamp());
  EXPECT_EQ(observation.source_object_id, "can_0");
  EXPECT_NEAR(observation.pose.pose.position.x, 0.60, 1.0e-9);
  EXPECT_NEAR(observation.pose.pose.position.y, 0.00, 1.0e-9);
  EXPECT_NEAR(observation.pose.pose.position.z, 1.80, 1.0e-9);
  // Confidence is the weakest stage of the chain, not just the last one.
  EXPECT_FLOAT_EQ(observation.confidence, 0.99F);
  EXPECT_EQ(observation.backend_name, "fake_perception");
  EXPECT_FALSE(observation.backend_version.empty());
}

TEST(FakePoseEstimation, RejectsDetectionsFromAnotherFrameOrAnotherInstant)
{
  FakePerceptionBackend backend(scripted_config());
  const auto frame = acquisition();
  const auto detections = backend.detect(frame);

  auto foreign_frame = detections;
  foreign_frame.header.frame_id = "some_other_optical";
  const auto wrong_frame = backend.estimate(foreign_frame, frame, camera_to_planning());
  ASSERT_FALSE(wrong_frame);
  EXPECT_EQ(wrong_frame.error().code, PerceptionErrorCode::FrameMismatch);

  auto foreign_instant = detections;
  foreign_instant.header.stamp = rclcpp::Time(kStampNs + 33'000'000, RCL_ROS_TIME);
  const auto wrong_instant = backend.estimate(foreign_instant, frame, camera_to_planning());
  ASSERT_FALSE(wrong_instant);
  EXPECT_EQ(wrong_instant.error().code, PerceptionErrorCode::OutOfOrder);
}

TEST(FakePoseEstimation, RejectsATransformThatDoesNotConvertFromTheAcquisitionFrame)
{
  FakePerceptionBackend backend(scripted_config());
  const auto frame = acquisition();
  const auto detections = backend.detect(frame);

  auto unrelated = FramedTransform::create(
    "wrist_camera_optical", kPlanningFrame, Eigen::Isometry3d::Identity());
  ASSERT_TRUE(unrelated);

  const auto estimated = backend.estimate(detections, frame, unrelated.value());
  ASSERT_FALSE(estimated);
  EXPECT_EQ(estimated.error().code, PerceptionErrorCode::FrameMismatch);
}

TEST(FakePoseEstimation, IsDeterministicAcrossRepeatedRuns)
{
  FakePerceptionBackend backend(scripted_config());
  const auto frame = acquisition();

  const auto first = backend.estimate(backend.detect(frame), frame, camera_to_planning());
  const auto second = backend.estimate(backend.detect(frame), frame, camera_to_planning());
  ASSERT_TRUE(first);
  ASSERT_TRUE(second);
  ASSERT_EQ(first.value().size(), second.value().size());
  EXPECT_EQ(first.value().front(), second.value().front());
}

}  // namespace
