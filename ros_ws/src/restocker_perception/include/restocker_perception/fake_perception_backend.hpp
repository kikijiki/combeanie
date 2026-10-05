// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <restocker_interfaces/msg/object_detection.hpp>

#include "restocker_perception/perception_port.hpp"

namespace restocker_perception
{

// One object the fake backend will report, described in the camera frame.
//
// Poses are scripted in the camera frame so the fake exercises the same camera-to-planning
// conversion a real backend must perform, instead of emitting world-frame poses that would hide a
// frame defect in every downstream test.
struct ScriptedObject
{
  std::string source_object_id;
  std::uint8_t product_class{restocker_interfaces::msg::ObjectDetection::PRODUCT_CLASS_CAN};
  std::uint8_t orientation{restocker_interfaces::msg::ObjectObservation::ORIENTATION_UPRIGHT};
  std::uint32_t instance_id{0};
  Eigen::Isometry3d pose_in_camera{Eigen::Isometry3d::Identity()};
  PoseCovariance covariance{PoseCovariance::Identity() * 1.0e-4};
  float detection_score{1.0F};
  float pose_confidence{1.0F};
  std::uint16_t x_min{0};
  std::uint16_t y_min{0};
  std::uint16_t x_max{16};
  std::uint16_t y_max{16};
  bool has_mask{true};
};

struct FakePerceptionConfig
{
  std::string backend_name{"fake_perception"};
  std::string backend_version{"1.0.0"};
  CameraIntrinsics intrinsics{320.0, 320.0, 160.0, 120.0, 320, 240};
  std::vector<ScriptedObject> objects;
};

// A deterministic detection, segmentation and pose-estimation backend.
//
// A pure function of its script and the acquisition it is handed: no clocks, no randomness, no
// I/O. It lets the ingestion path, the world-state admission policy and other consumers be tested
// without a camera or GPU.
//
// Installed rather than confined to tests because other packages use it to test their own
// perception consumers.
class FakePerceptionBackend final : public DetectionPort, public PoseEstimationPort
{
public:
  explicit FakePerceptionBackend(FakePerceptionConfig config);

  [[nodiscard]] restocker_interfaces::msg::PerceptionFrame detect(const RgbdFrame & frame) override;

  [[nodiscard]] Result<std::vector<restocker_interfaces::msg::ObjectObservation>> estimate(
    const restocker_interfaces::msg::PerceptionFrame & detections, const RgbdFrame & frame,
    const FramedTransform & camera_to_planning) override;

  [[nodiscard]] const FakePerceptionConfig & config() const noexcept {return config_;}

  // Makes the next and all subsequent detect() calls report a backend failure instead of
  // detections, so consumers can be tested against a degraded backend.
  void fail_detection_with(std::uint8_t status, std::string detail);
  void clear_detection_failure();

private:
  FakePerceptionConfig config_;
  std::uint8_t forced_status_{restocker_interfaces::msg::PerceptionFrame::STATUS_OK};
  std::string forced_detail_;
};

// Builds a zero-filled RGB-D acquisition for tests and for scripted runs without a camera.
//
// The images are consistent with each other and the intrinsics, so the result always satisfies
// RgbdFrame::create.
[[nodiscard]] Result<RgbdFrame> make_synthetic_rgbd_frame(
  const CameraIntrinsics & intrinsics, const std::string & frame_id, const rclcpp::Time & stamp);

}  // namespace restocker_perception
