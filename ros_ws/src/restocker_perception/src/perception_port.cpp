// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/perception_port.hpp"

#include <cmath>
#include <cstdint>
#include <numeric>
#include <string>
#include <utility>

namespace restocker_perception
{
namespace
{

using PerceptionFrameMessage = restocker_interfaces::msg::PerceptionFrame;

[[nodiscard]] PerceptionError error(PerceptionErrorCode code, std::string detail)
{
  return PerceptionError{code, std::move(detail)};
}

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] std::optional<PerceptionError> validate_detection(
  const restocker_interfaces::msg::ObjectDetection & detection, std::uint16_t image_width,
  std::uint16_t image_height)
{
  if (detection.x_min >= detection.x_max || detection.y_min >= detection.y_max) {
    return error(PerceptionErrorCode::InvalidArgument, "detection bounding box is empty");
  }
  if (detection.x_max > image_width || detection.y_max > image_height) {
    return error(
      PerceptionErrorCode::InvalidArgument, "detection bounding box leaves the image");
  }
  if (!std::isfinite(detection.score) || detection.score < 0.0F || detection.score > 1.0F) {
    return error(PerceptionErrorCode::InvalidArgument, "detection score must be in [0, 1]");
  }
  if (!detection.has_mask) {
    return std::nullopt;
  }

  const auto box_width = static_cast<std::uint32_t>(detection.x_max - detection.x_min);
  const auto box_height = static_cast<std::uint32_t>(detection.y_max - detection.y_min);
  if (detection.mask.width != box_width || detection.mask.height != box_height) {
    return error(
      PerceptionErrorCode::InvalidArgument,
      "segmentation mask extent does not match its detection bounding box");
  }
  if (!std::isfinite(detection.mask.score) || detection.mask.score < 0.0F ||
    detection.mask.score > 1.0F)
  {
    return error(PerceptionErrorCode::InvalidArgument, "segmentation score must be in [0, 1]");
  }
  const std::uint64_t covered = std::accumulate(
    detection.mask.rle_counts.begin(), detection.mask.rle_counts.end(), std::uint64_t{0},
    [](std::uint64_t total, std::uint32_t run) {return total + run;});
  if (covered != static_cast<std::uint64_t>(box_width) * box_height) {
    return error(
      PerceptionErrorCode::InvalidArgument,
      "segmentation run lengths do not cover the bounding box exactly");
  }
  return std::nullopt;
}

}  // namespace

RgbdFrame::RgbdFrame(
  CameraIntrinsics intrinsics, Image::ConstSharedPtr colour, Image::ConstSharedPtr depth,
  std::string frame_id, rclcpp::Time stamp)
: intrinsics_(intrinsics),
  colour_(std::move(colour)),
  depth_(std::move(depth)),
  frame_id_(std::move(frame_id)),
  stamp_(stamp)
{
}

Result<RgbdFrame> RgbdFrame::create(
  const CameraIntrinsics & intrinsics, Image::ConstSharedPtr colour, Image::ConstSharedPtr depth)
{
  if (colour == nullptr || depth == nullptr) {
    return Result<RgbdFrame>::failure(
      error(PerceptionErrorCode::InvalidArgument, "colour and depth images are both required"));
  }
  if (!finite_positive(intrinsics.fx) || !finite_positive(intrinsics.fy) ||
    !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy))
  {
    return Result<RgbdFrame>::failure(
      error(PerceptionErrorCode::InvalidArgument, "camera intrinsics are not finite and positive"));
  }
  if (intrinsics.width == 0 || intrinsics.height == 0) {
    return Result<RgbdFrame>::failure(
      error(PerceptionErrorCode::InvalidArgument, "camera intrinsics declare an empty image"));
  }
  if (colour->header.frame_id.empty()) {
    return Result<RgbdFrame>::failure(
      error(PerceptionErrorCode::InvalidArgument, "acquisition frame must not be empty"));
  }
  // Colour and depth must share the optical frame and the capture instant.
  if (colour->header.frame_id != depth->header.frame_id) {
    return Result<RgbdFrame>::failure(
      error(
        PerceptionErrorCode::FrameMismatch,
        "colour frame '" + colour->header.frame_id + "' and depth frame '" +
        depth->header.frame_id + "' disagree"));
  }
  const rclcpp::Time colour_stamp(colour->header.stamp, RCL_ROS_TIME);
  const rclcpp::Time depth_stamp(depth->header.stamp, RCL_ROS_TIME);
  if (colour_stamp != depth_stamp) {
    return Result<RgbdFrame>::failure(
      error(
        PerceptionErrorCode::OutOfOrder, "colour and depth images were captured at different "
        "instants"));
  }
  if (colour->width != intrinsics.width || colour->height != intrinsics.height ||
    depth->width != intrinsics.width || depth->height != intrinsics.height)
  {
    return Result<RgbdFrame>::failure(
      error(
        PerceptionErrorCode::InvalidArgument,
        "image dimensions do not match the declared camera intrinsics"));
  }

  std::string frame_id = colour->header.frame_id;
  return Result<RgbdFrame>::success(
    RgbdFrame(intrinsics, std::move(colour), std::move(depth), std::move(frame_id), colour_stamp));
}

std::optional<PerceptionError> validate_estimation_inputs(
  const PerceptionFrameMessage & detections, const RgbdFrame & frame,
  const FramedTransform & camera_to_planning)
{
  if (detections.status != PerceptionFrameMessage::STATUS_OK) {
    return error(
      PerceptionErrorCode::InvalidArgument,
      "detection frame status is not OK: " + detections.status_detail);
  }
  if (detections.backend_name.empty() || detections.backend_version.empty()) {
    return error(
      PerceptionErrorCode::InvalidArgument, "detection backend name and version must not be empty");
  }
  if (detections.header.frame_id != frame.frame_id()) {
    return error(
      PerceptionErrorCode::FrameMismatch,
      "detections were measured in frame '" + detections.header.frame_id +
      "' but the depth acquisition is in frame '" + frame.frame_id() + "'");
  }
  if (rclcpp::Time(detections.header.stamp, RCL_ROS_TIME) != frame.stamp()) {
    return error(
      PerceptionErrorCode::OutOfOrder,
      "detections and depth acquisition are from different instants");
  }
  if (camera_to_planning.source_frame() != frame.frame_id()) {
    return error(
      PerceptionErrorCode::FrameMismatch,
      "transform converts from '" + camera_to_planning.source_frame() +
      "' but the acquisition is in '" + frame.frame_id() + "'");
  }
  if (detections.image_width != frame.intrinsics().width ||
    detections.image_height != frame.intrinsics().height)
  {
    return error(
      PerceptionErrorCode::InvalidArgument,
      "detection image dimensions do not match the acquisition");
  }
  for (const auto & detection : detections.detections) {
    if (auto invalid = validate_detection(
        detection, detections.image_width,
        detections.image_height))
    {
      return invalid;
    }
  }
  return std::nullopt;
}

}  // namespace restocker_perception
