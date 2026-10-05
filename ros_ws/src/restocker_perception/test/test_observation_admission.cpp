// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Admission of perception observations by the world state, through the full ingestion path:
// fake backend -> ObjectObservation message -> object_observation_from_message ->
// WorldStateStore::observe_object. The admission policy lives in the world state.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>

#include <restocker_world_state/ros_conversions.hpp>
#include <restocker_world_state/world_state.hpp>

#include "restocker_perception/fake_perception_backend.hpp"

namespace
{

using restocker_perception::CameraIntrinsics;
using restocker_perception::FakePerceptionBackend;
using restocker_perception::FakePerceptionConfig;
using restocker_perception::FramedTransform;
using restocker_perception::RgbdFrame;
using restocker_perception::ScriptedObject;
using restocker_perception::make_synthetic_rgbd_frame;
using restocker_world_state::WorldStateConfig;
using restocker_world_state::WorldStateErrorCode;
using restocker_world_state::WorldStateStore;
using restocker_world_state::object_observation_from_message;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

constexpr const char * kCameraFrame = "camera_optical";
constexpr const char * kPlanningFrame = "world";
constexpr double kMinimumConfidence = 0.99;
// Matches the world_state_node default of maximum_observation_age_ms = 2000.
constexpr std::int64_t kMaximumAgeNs = 2'000'000'000;

[[nodiscard]] CameraIntrinsics intrinsics()
{
  return CameraIntrinsics{320.0, 320.0, 160.0, 120.0, 320, 240};
}

[[nodiscard]] WorldStateConfig store_config()
{
  WorldStateConfig config;
  config.planning_frame = kPlanningFrame;
  config.maximum_observation_age = std::chrono::nanoseconds(kMaximumAgeNs);
  return config;
}

[[nodiscard]] FakePerceptionConfig scripted_config(float confidence)
{
  ScriptedObject can;
  can.source_object_id = "can_0";
  can.pose_in_camera.translation() = Eigen::Vector3d(0.10, 0.0, 0.60);
  can.detection_score = confidence;
  can.pose_confidence = confidence;
  can.x_min = 100;
  can.y_min = 80;
  can.x_max = 140;
  can.y_max = 160;

  FakePerceptionConfig config;
  config.intrinsics = intrinsics();
  config.objects.push_back(can);
  return config;
}

[[nodiscard]] FramedTransform camera_to_planning()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(0.5, 0.0, 1.2);
  auto created = FramedTransform::create(kCameraFrame, kPlanningFrame, transform);
  EXPECT_TRUE(created);
  return created.value();
}

// Runs the perception pipeline once and returns the single observation message it produced.
[[nodiscard]] ObservationMessage observe_at(std::int64_t stamp_ns, float confidence = 1.0F)
{
  FakePerceptionBackend backend(scripted_config(confidence));
  auto frame = make_synthetic_rgbd_frame(
    intrinsics(), kCameraFrame, rclcpp::Time(stamp_ns, RCL_ROS_TIME));
  EXPECT_TRUE(frame);
  const auto detections = backend.detect(frame.value());
  auto estimated = backend.estimate(detections, frame.value(), camera_to_planning());
  EXPECT_TRUE(estimated);
  EXPECT_EQ(estimated.value().size(), 1U);
  return estimated.value().front();
}

TEST(ObservationAdmission, AdmitsAWellFormedFreshObservation)
{
  WorldStateStore store(store_config());
  const auto message = observe_at(10'000'000'000);

  const auto converted = object_observation_from_message(message, kMinimumConfidence);
  ASSERT_TRUE(converted);

  const auto receipt = store.observe_object(
    converted.value(), rclcpp::Time(10'050'000'000, RCL_ROS_TIME));
  ASSERT_TRUE(receipt);
  EXPECT_TRUE(receipt.value().created);
  EXPECT_GT(receipt.value().revision, 0U);

  const auto snapshot = store.snapshot();
  ASSERT_EQ(snapshot.objects.size(), 1U);
  const auto & tracked = snapshot.objects.begin()->second;
  EXPECT_EQ(tracked.source_object_id, "can_0");
  // The pose the world state kept is the planning-frame pose, not the camera-frame measurement.
  EXPECT_NEAR(tracked.pose_in_world.translation().x(), 0.60, 1.0e-9);
  EXPECT_NEAR(tracked.pose_in_world.translation().z(), 1.80, 1.0e-9);
  // Covariance is kept through ingestion.
  EXPECT_GT(tracked.pose_covariance.norm(), 0.0);
}

TEST(ObservationAdmission, RejectsAStaleObservation)
{
  WorldStateStore store(store_config());
  const auto message = observe_at(10'000'000'000);

  const auto converted = object_observation_from_message(message, kMinimumConfidence);
  ASSERT_TRUE(converted);

  // Received one nanosecond past the configured maximum age.
  const rclcpp::Time too_late(10'000'000'000 + kMaximumAgeNs + 1, RCL_ROS_TIME);
  const auto receipt = store.observe_object(converted.value(), too_late);
  ASSERT_FALSE(receipt);
  EXPECT_EQ(receipt.error().code, WorldStateErrorCode::StaleObservation);
  EXPECT_TRUE(store.snapshot().objects.empty());
}

TEST(ObservationAdmission, RejectsALowConfidenceObservation)
{
  const auto message = observe_at(10'000'000'000, 0.80F);
  EXPECT_FLOAT_EQ(message.confidence, 0.80F);

  const auto converted = object_observation_from_message(message, kMinimumConfidence);
  ASSERT_FALSE(converted);
  EXPECT_EQ(converted.error().code, WorldStateErrorCode::InvalidArgument);

  // Refused at the contract boundary, so it never reaches the store.
  WorldStateStore store(store_config());
  EXPECT_TRUE(store.snapshot().objects.empty());
}

TEST(ObservationAdmission, RejectsAnOutOfOrderObservationWithoutMutating)
{
  WorldStateStore store(store_config());

  const auto first = object_observation_from_message(
    observe_at(10'000'000'000), kMinimumConfidence);
  ASSERT_TRUE(first);
  const auto admitted = store.observe_object(
    first.value(), rclcpp::Time(10'010'000'000, RCL_ROS_TIME));
  ASSERT_TRUE(admitted);
  const auto revision_after_admission = admitted.value().revision;

  // Older than the stored observation but still within the staleness gate.
  const auto earlier = object_observation_from_message(
    observe_at(9'900'000'000), kMinimumConfidence);
  ASSERT_TRUE(earlier);
  const auto rejected = store.observe_object(
    earlier.value(), rclcpp::Time(10'020'000'000, RCL_ROS_TIME));
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::OutOfOrder);

  // A rejected observation must not advance the revision or change the tracked pose.
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.revision, revision_after_admission);
  ASSERT_EQ(snapshot.objects.size(), 1U);
  EXPECT_EQ(
    snapshot.objects.begin()->second.observation_time,
    rclcpp::Time(10'000'000'000, RCL_ROS_TIME));

  // Replaying the identical observation is equally refused: identical is not newer.
  const auto replayed = store.observe_object(
    first.value(), rclcpp::Time(10'030'000'000, RCL_ROS_TIME));
  ASSERT_FALSE(replayed);
  EXPECT_EQ(replayed.error().code, WorldStateErrorCode::OutOfOrder);
}

TEST(ObservationAdmission, RejectsAnObservationLeftInTheCameraFrame)
{
  WorldStateStore store(store_config());
  auto message = observe_at(10'000'000'000);
  // A backend that forgot to convert: pose unchanged, frame label is the camera frame.
  message.header.frame_id = kCameraFrame;

  const auto converted = object_observation_from_message(message, kMinimumConfidence);
  ASSERT_TRUE(converted);

  const auto receipt = store.observe_object(
    converted.value(), rclcpp::Time(10'010'000'000, RCL_ROS_TIME));
  ASSERT_FALSE(receipt);
  EXPECT_EQ(receipt.error().code, WorldStateErrorCode::FrameMismatch);
  EXPECT_TRUE(store.snapshot().objects.empty());
}

}  // namespace
