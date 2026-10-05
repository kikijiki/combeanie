// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/fake_perception_backend.hpp"

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace restocker_perception
{
namespace
{

using DetectionMessage = restocker_interfaces::msg::ObjectDetection;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;
using PerceptionFrameMessage = restocker_interfaces::msg::PerceptionFrame;

}  // namespace

FakePerceptionBackend::FakePerceptionBackend(FakePerceptionConfig config)
: config_(std::move(config))
{
}

void FakePerceptionBackend::fail_detection_with(std::uint8_t status, std::string detail)
{
  forced_status_ = status;
  forced_detail_ = std::move(detail);
}

void FakePerceptionBackend::clear_detection_failure()
{
  forced_status_ = PerceptionFrameMessage::STATUS_OK;
  forced_detail_.clear();
}

PerceptionFrameMessage FakePerceptionBackend::detect(const RgbdFrame & frame)
{
  PerceptionFrameMessage message;
  // The acquisition is the only authority for when and where these detections are valid; its frame
  // and stamp are copied verbatim, never re-derived from a clock.
  message.header.frame_id = frame.frame_id();
  message.header.stamp = frame.stamp();
  message.image_width = frame.intrinsics().width;
  message.image_height = frame.intrinsics().height;
  message.backend_name = config_.backend_name;
  message.backend_version = config_.backend_version;
  message.status = forced_status_;
  message.status_detail = forced_detail_;
  if (forced_status_ != PerceptionFrameMessage::STATUS_OK) {
    return message;
  }

  message.detections.reserve(config_.objects.size());
  for (const auto & scripted : config_.objects) {
    DetectionMessage detection;
    detection.x_min = scripted.x_min;
    detection.y_min = scripted.y_min;
    detection.x_max = scripted.x_max;
    detection.y_max = scripted.y_max;
    detection.product_class = scripted.product_class;
    detection.score = scripted.detection_score;
    detection.instance_id = scripted.instance_id;
    detection.source_object_id = scripted.source_object_id;
    detection.has_mask = scripted.has_mask;
    if (scripted.has_mask) {
      const auto box_width = static_cast<std::uint32_t>(scripted.x_max - scripted.x_min);
      const auto box_height = static_cast<std::uint32_t>(scripted.y_max - scripted.y_min);
      detection.mask.width = static_cast<std::uint16_t>(box_width);
      detection.mask.height = static_cast<std::uint16_t>(box_height);
      detection.mask.score = scripted.detection_score;
      // A fully foreground box: a leading zero-length background run followed by one run covering
      // every pixel, satisfying the run-length coverage contract.
      detection.mask.rle_counts = {0U, box_width * box_height};
    }
    message.detections.push_back(std::move(detection));
  }
  return message;
}

Result<std::vector<ObservationMessage>> FakePerceptionBackend::estimate(
  const PerceptionFrameMessage & detections, const RgbdFrame & frame,
  const FramedTransform & camera_to_planning)
{
  if (auto invalid = validate_estimation_inputs(detections, frame, camera_to_planning)) {
    return Result<std::vector<ObservationMessage>>::failure(std::move(*invalid));
  }

  std::vector<ObservationMessage> observations;
  observations.reserve(detections.detections.size());
  for (const auto & detection : detections.detections) {
    const auto scripted = std::find_if(
      config_.objects.begin(), config_.objects.end(),
      [&detection](const ScriptedObject & candidate) {
        return candidate.source_object_id == detection.source_object_id;
      });
    if (scripted == config_.objects.end()) {
      return Result<std::vector<ObservationMessage>>::failure(
        PerceptionError{
          PerceptionErrorCode::NotFound,
          "detection '" + detection.source_object_id + "' is not in the fake backend script"});
    }

    FramedPose measured;
    measured.frame_id = frame.frame_id();
    measured.pose = scripted->pose_in_camera;
    measured.covariance = scripted->covariance;
    auto converted = camera_to_planning.apply(measured);
    if (!converted) {
      return Result<std::vector<ObservationMessage>>::failure(converted.error());
    }

    ObservationMessage observation;
    // Stamped with the acquisition instant, not now(): the world state's staleness gate depends on
    // the measurement time.
    observation.header.stamp = detections.header.stamp;
    observation.header.frame_id = converted.value().frame_id;
    observation.source_object_id = detection.source_object_id;
    observation.product_class = detection.product_class;
    observation.has_sku = false;
    observation.orientation = scripted->orientation;

    const Eigen::Isometry3d & pose = converted.value().pose;
    const Eigen::Quaterniond rotation(pose.rotation());
    observation.pose.pose.position.x = pose.translation().x();
    observation.pose.pose.position.y = pose.translation().y();
    observation.pose.pose.position.z = pose.translation().z();
    observation.pose.pose.orientation.x = rotation.x();
    observation.pose.pose.orientation.y = rotation.y();
    observation.pose.pose.orientation.z = rotation.z();
    observation.pose.pose.orientation.w = rotation.w();
    for (std::size_t row = 0; row < 6; ++row) {
      for (std::size_t column = 0; column < 6; ++column) {
        observation.pose.covariance[row * 6 + column] = converted.value().covariance(row, column);
      }
    }

    // The chain is no more confident than its weakest stage, so detection and pose confidence
    // combine by minimum.
    observation.confidence = std::min(detection.score, scripted->pose_confidence);
    observation.backend_name = config_.backend_name;
    observation.backend_version = config_.backend_version;
    observation.status = ObservationMessage::STATUS_OK;
    observations.push_back(std::move(observation));
  }
  return Result<std::vector<ObservationMessage>>::success(std::move(observations));
}

Result<RgbdFrame> make_synthetic_rgbd_frame(
  const CameraIntrinsics & intrinsics, const std::string & frame_id, const rclcpp::Time & stamp)
{
  auto colour = std::make_shared<sensor_msgs::msg::Image>();
  colour->header.frame_id = frame_id;
  colour->header.stamp = stamp;
  colour->width = intrinsics.width;
  colour->height = intrinsics.height;
  colour->encoding = "rgb8";
  colour->step = static_cast<std::uint32_t>(intrinsics.width) * 3U;
  colour->data.assign(static_cast<std::size_t>(colour->step) * intrinsics.height, 0U);

  auto depth = std::make_shared<sensor_msgs::msg::Image>();
  depth->header.frame_id = frame_id;
  depth->header.stamp = stamp;
  depth->width = intrinsics.width;
  depth->height = intrinsics.height;
  depth->encoding = "32FC1";
  depth->step = static_cast<std::uint32_t>(intrinsics.width) * 4U;
  depth->data.assign(static_cast<std::size_t>(depth->step) * intrinsics.height, 0U);

  return RgbdFrame::create(intrinsics, std::move(colour), std::move(depth));
}

}  // namespace restocker_perception
