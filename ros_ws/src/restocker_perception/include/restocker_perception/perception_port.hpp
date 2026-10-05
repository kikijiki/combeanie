// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/perception_frame.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "restocker_perception/frame_geometry.hpp"

namespace restocker_perception
{

struct CameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::uint16_t width{0};
  std::uint16_t height{0};
};

// One RGB-D acquisition, which cannot be constructed from components that disagree.
//
// The colour and depth images must carry the same frame and stamp, and match the declared
// intrinsics. The frame and stamp are derived from the validated components, so a frame whose
// depth was captured at a different instant or in a different optical frame from its colour is not
// representable.
class RgbdFrame
{
public:
  using Image = sensor_msgs::msg::Image;

  [[nodiscard]] static Result<RgbdFrame> create(
    const CameraIntrinsics & intrinsics, Image::ConstSharedPtr colour, Image::ConstSharedPtr depth);

  [[nodiscard]] const std::string & frame_id() const noexcept {return frame_id_;}
  [[nodiscard]] const rclcpp::Time & stamp() const noexcept {return stamp_;}
  [[nodiscard]] const CameraIntrinsics & intrinsics() const noexcept {return intrinsics_;}
  [[nodiscard]] const Image & colour() const noexcept {return *colour_;}
  [[nodiscard]] const Image & depth() const noexcept {return *depth_;}

private:
  RgbdFrame(
    CameraIntrinsics intrinsics, Image::ConstSharedPtr colour, Image::ConstSharedPtr depth,
    std::string frame_id, rclcpp::Time stamp);

  CameraIntrinsics intrinsics_;
  Image::ConstSharedPtr colour_;
  Image::ConstSharedPtr depth_;
  std::string frame_id_;
  rclcpp::Time stamp_;
};

// The backend-neutral detection and segmentation boundary.
//
// An implementation turns one RGB-D acquisition into the detections and instance masks found in
// it. Segmentation is not a separate port: a mask is only meaningful for its own detection and
// frame, so masks are delivered inside their detection, inside the single PerceptionFrame that
// owns the stamp and frame for the whole acquisition. A mask cannot be paired with a detection
// from another instant.
//
// Implementations must copy the input frame's frame_id and stamp into the result header verbatim
// and must report failure through PerceptionFrame.status rather than by omitting detections.
class DetectionPort
{
public:
  virtual ~DetectionPort() = default;

  [[nodiscard]] virtual restocker_interfaces::msg::PerceptionFrame detect(
    const RgbdFrame & frame) = 0;
};

// The backend-neutral 6-DoF pose-estimation boundary.
//
// An implementation turns detections plus depth into normalized ObjectObservation messages. It is
// handed the camera-to-planning transform explicitly and must express every observation in
// camera_to_planning.target_frame(), because the world state admits observations only in its
// configured planning frame. No overload omits the transform, so an implementation cannot emit
// camera-frame poses labelled as world poses.
//
// Pose estimation has no message type of its own. Its output is ObjectObservation, which carries
// frame, stamp, confidence, 6x6 covariance and backend provenance, and is the contract the world
// state ingests.
class PoseEstimationPort
{
public:
  virtual ~PoseEstimationPort() = default;

  [[nodiscard]] virtual Result<std::vector<restocker_interfaces::msg::ObjectObservation>> estimate(
    const restocker_interfaces::msg::PerceptionFrame & detections, const RgbdFrame & frame,
    const FramedTransform & camera_to_planning) = 0;
};

// The single frame-and-instant check every PoseEstimationPort implementation must run first.
//
// It rejects a detection set that did not come from this acquisition, or that was measured in a
// frame the supplied transform does not convert from. This keeps any backend from silently fusing
// a detection set with depth from a different instant.
[[nodiscard]] std::optional<PerceptionError> validate_estimation_inputs(
  const restocker_interfaces::msg::PerceptionFrame & detections, const RgbdFrame & frame,
  const FramedTransform & camera_to_planning);

}  // namespace restocker_perception
