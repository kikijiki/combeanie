// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <span>

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace restocker_perception
{

enum class DepthObstacleErrorCode : std::uint8_t
{
  InvalidConfiguration,
  InvalidIntrinsics,
  InvalidDepthBuffer,
  InvalidTransform,
};

struct DepthObstacleError
{
  DepthObstacleErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] DepthObstacleResult
{
public:
  [[nodiscard]] static DepthObstacleResult success(T value)
  {
    return DepthObstacleResult(std::move(value));
  }

  [[nodiscard]] static DepthObstacleResult failure(DepthObstacleError error)
  {
    return DepthObstacleResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const DepthObstacleError & error() const
  {
    return std::get<DepthObstacleError>(storage_);
  }

private:
  explicit DepthObstacleResult(T value)
  : storage_(std::move(value)) {}

  explicit DepthObstacleResult(DepthObstacleError error)
  : storage_(std::move(error)) {}

  std::variant<T, DepthObstacleError> storage_;
};

// Pinhole intrinsics, as CameraInfo carries them in K.
struct DepthCameraIntrinsics
{
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  std::uint32_t width{0U};
  std::uint32_t height{0U};
};

// One robot volume whose depth returns are attributed to the robot rather than to an obstacle:
// every point within `radius` of the segment from `start` to `end`.
//
// A capsule rather than a sphere because this robot's links are long. Covering a roughly 0.75 m
// UR10e link envelope with a sphere needs a radius set by the link's length, which reaches
// sideways just as far into the corridor beside the arm where obstacles stand. In a capsule the
// segment absorbs the length and the radius carries only the cross-section.
//
// Both endpoints are in the frame the extracted boxes are expressed in, resolved at the depth
// image's stamp. `start == end` is a sphere.
struct SelfFilterCapsule
{
  Eigen::Vector3d start{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

struct DepthObstacleConfig
{
  // Depth returns outside this range are discarded before anything else looks at them.
  double min_depth_m{0.20};
  double max_depth_m{4.00};
  // Every nth pixel in each axis. A 640x480 image at stride 4 is 19200 samples, ample for 0.15 m
  // boxes at low-millisecond cost per frame.
  std::uint32_t pixel_stride{4U};
  // Volume of interest, in the output frame. The first and cheapest self-filter: the shelf
  // interior, the stock tray and the robot's own column can be excluded outright.
  Eigen::Vector3d volume_min{Eigen::Vector3d::Zero()};
  Eigen::Vector3d volume_max{Eigen::Vector3d::Zero()};
  // Downsampling cell, and therefore the quantum of every reported box dimension.
  double voxel_size_m{0.02};
  // Occupied cells a cluster needs before it is believed. Isolated speckle never reaches this.
  std::size_t min_cluster_voxels{12U};
  // Empty voxel layers a cluster may bridge vertically. Neighbouring cells are 26-connected in x
  // and y; in z the search reaches this many layers further.
  //
  // A downward-looking camera sees an upright obstacle's side face at a grazing angle, so
  // `pixel_stride` consecutive image rows land far apart in z on that face while staying in the
  // same x-y column. The face is sampled as horizontal bands with empty layers between them, and
  // pure 26-connectivity reports each band as its own obstacle. `max_boxes` is a hard cap, so one
  // obstacle spending several slots can cause a second to be silently dropped. Bridging in z only
  // cannot hide geometry: it can merge two stacked obstacles into one taller box (over-reporting),
  // whereas splitting one obstacle under-reports.
  //
  // 4 is twice the measured minimum. Sweeping this value against the live overhead camera with one
  // and then two 0.15 x 0.15 x 0.50 boxes under it, the box count per obstacle was 5, 3, 1, 1, 1
  // for values 0 through 4. The bands close at 2, and at 0 two obstacles produced 8 boxes, exactly
  // `max_boxes`, with real bands already dropped. The same boxes at the ends of the corridor never
  // fragmented, since the face is seen from the side there; the worst case is directly under the
  // camera, where 2 was measured.
  std::int64_t cluster_vertical_gap_voxels{4};
  // A cluster whose largest extent is below this is discarded as noise; one whose largest extent
  // exceeds this is discarded as a surface the volume of interest should have excluded, because
  // a box that large would swallow the workspace.
  double min_box_extent_m{0.04};
  double max_box_extent_m{1.20};
  // Upper bound on reported boxes. Exceeding it is a scene this representation cannot describe,
  // so the most strongly supported clusters are kept and the rest are dropped.
  std::size_t max_boxes{8U};
};

struct ObstacleBoxFit
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  // Full extents. Never smaller than one voxel in any axis.
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  std::size_t point_count{0U};
};

[[nodiscard]] bool valid_depth_obstacle_config(const DepthObstacleConfig & config) noexcept;

// Unprojects a 32FC1 depth buffer, crops it to the configured volume, removes returns inside the
// self-filter capsules, downsamples to voxels, groups the survivors into clusters and bounds each
// cluster with an axis-aligned box.
//
// `output_from_optical` must be the sensor pose at the depth image's own stamp. This function
// cannot check that; it is the caller's obligation, and the published message carries the stamp
// it was resolved at.
//
// `depth` is row-major, `row_step_pixels` elements per row (usually equal to width, but a ROS
// image may pad). Non-finite and non-positive samples are treated as no return, as REP-118
// requires.
[[nodiscard]] DepthObstacleResult<std::vector<ObstacleBoxFit>> extract_obstacle_boxes(
  std::span<const float> depth, std::uint32_t row_step_pixels,
  const DepthCameraIntrinsics & intrinsics, const Eigen::Isometry3d & output_from_optical,
  std::span<const SelfFilterCapsule> self_filter, const DepthObstacleConfig & config);

[[nodiscard]] const char * to_string(DepthObstacleErrorCode code) noexcept;

}  // namespace restocker_perception
