// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/depth_obstacle_extraction.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace restocker_perception
{
namespace
{

// Integer voxel coordinate. std::map keeps iteration deterministic, so an unchanged scene produces
// a byte-identical box list. The downstream planning-scene projector treats any change in that
// list as new geometry, so an order that depended on hash seeding would announce a changed scene
// every frame and reject every plan in flight.
struct VoxelKey
{
  std::int64_t x{0};
  std::int64_t y{0};
  std::int64_t z{0};

  [[nodiscard]] bool operator<(const VoxelKey & other) const noexcept
  {
    if (x != other.x) {
      return x < other.x;
    }
    if (y != other.y) {
      return y < other.y;
    }
    return z < other.z;
  }
};

struct VoxelCell
{
  std::size_t point_count{0U};
  std::size_t component{0U};
  bool assigned{false};
};

template<typename T>
[[nodiscard]] DepthObstacleResult<T> failure(DepthObstacleErrorCode code, std::string detail)
{
  return DepthObstacleResult<T>::failure(DepthObstacleError{code, std::move(detail)});
}

[[nodiscard]] bool finite_vector(const Eigen::Vector3d & value) noexcept
{
  return value.allFinite();
}

[[nodiscard]] std::int64_t voxel_index(double value, double size) noexcept
{
  return static_cast<std::int64_t>(std::floor(value / size));
}

// Squared distance from a point to the segment [start, end]. A degenerate segment is a point, so
// the capsule collapses to a sphere.
[[nodiscard]] double squared_distance_to_segment(
  const Eigen::Vector3d & point, const SelfFilterCapsule & capsule) noexcept
{
  const Eigen::Vector3d along = capsule.end - capsule.start;
  const double length_squared = along.squaredNorm();
  const Eigen::Vector3d offset = point - capsule.start;
  if (length_squared <= 0.0) {
    return offset.squaredNorm();
  }
  const double fraction = std::clamp(offset.dot(along) / length_squared, 0.0, 1.0);
  return (offset - fraction * along).squaredNorm();
}

}  // namespace

bool valid_depth_obstacle_config(const DepthObstacleConfig & config) noexcept
{
  if (!std::isfinite(config.min_depth_m) || !std::isfinite(config.max_depth_m) ||
    config.min_depth_m <= 0.0 || config.max_depth_m <= config.min_depth_m)
  {
    return false;
  }
  if (config.pixel_stride == 0U) {
    return false;
  }
  if (!finite_vector(config.volume_min) || !finite_vector(config.volume_max)) {
    return false;
  }
  if ((config.volume_max - config.volume_min).minCoeff() <= 0.0) {
    return false;
  }
  if (!std::isfinite(config.voxel_size_m) || config.voxel_size_m <= 0.0) {
    return false;
  }
  if (config.min_cluster_voxels == 0U || config.max_boxes == 0U) {
    return false;
  }
  if (config.cluster_vertical_gap_voxels < 0) {
    return false;
  }
  if (!std::isfinite(config.min_box_extent_m) || !std::isfinite(config.max_box_extent_m) ||
    config.min_box_extent_m < 0.0 || config.max_box_extent_m <= config.min_box_extent_m)
  {
    return false;
  }
  // A voxel larger than the volume of interest would make the reported box dimensions meaningless.
  const double span = (config.volume_max - config.volume_min).minCoeff();
  return config.voxel_size_m <= span;
}

DepthObstacleResult<std::vector<ObstacleBoxFit>> extract_obstacle_boxes(
  std::span<const float> depth, std::uint32_t row_step_pixels,
  const DepthCameraIntrinsics & intrinsics, const Eigen::Isometry3d & output_from_optical,
  std::span<const SelfFilterCapsule> self_filter, const DepthObstacleConfig & config)
{
  using Boxes = std::vector<ObstacleBoxFit>;

  if (!valid_depth_obstacle_config(config)) {
    return failure<Boxes>(
      DepthObstacleErrorCode::InvalidConfiguration,
      "depth obstacle extraction configuration is invalid");
  }
  if (!std::isfinite(intrinsics.fx) || !std::isfinite(intrinsics.fy) ||
    !std::isfinite(intrinsics.cx) || !std::isfinite(intrinsics.cy) ||
    intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0 || intrinsics.width == 0U ||
    intrinsics.height == 0U)
  {
    return failure<Boxes>(
      DepthObstacleErrorCode::InvalidIntrinsics, "camera intrinsics are not a usable pinhole");
  }
  if (row_step_pixels < intrinsics.width) {
    return failure<Boxes>(
      DepthObstacleErrorCode::InvalidDepthBuffer, "row step is narrower than the image width");
  }
  const std::size_t required =
    static_cast<std::size_t>(row_step_pixels) * static_cast<std::size_t>(intrinsics.height);
  if (depth.size() < required) {
    return failure<Boxes>(
      DepthObstacleErrorCode::InvalidDepthBuffer,
      "depth buffer is shorter than the declared image");
  }
  if (!output_from_optical.matrix().allFinite()) {
    return failure<Boxes>(
      DepthObstacleErrorCode::InvalidTransform, "sensor pose is not a finite transform");
  }
  for (const auto & capsule : self_filter) {
    if (!finite_vector(capsule.start) || !finite_vector(capsule.end) ||
      !std::isfinite(capsule.radius) || capsule.radius < 0.0)
    {
      return failure<Boxes>(
        DepthObstacleErrorCode::InvalidConfiguration, "self-filter capsule is not finite");
    }
  }

  std::map<VoxelKey, VoxelCell> voxels;
  for (std::uint32_t row = 0; row < intrinsics.height; row += config.pixel_stride) {
    const std::size_t row_offset =
      static_cast<std::size_t>(row) * static_cast<std::size_t>(row_step_pixels);
    for (std::uint32_t column = 0; column < intrinsics.width; column += config.pixel_stride) {
      const double range = static_cast<double>(depth[row_offset + column]);
      if (!std::isfinite(range) || range < config.min_depth_m || range > config.max_depth_m) {
        continue;
      }
      const Eigen::Vector3d in_optical(
        (static_cast<double>(column) - intrinsics.cx) * range / intrinsics.fx,
        (static_cast<double>(row) - intrinsics.cy) * range / intrinsics.fy, range);
      const Eigen::Vector3d point = output_from_optical * in_optical;
      if ((point.array() < config.volume_min.array()).any() ||
        (point.array() > config.volume_max.array()).any())
      {
        continue;
      }
      // The self-filter runs after the crop because it is the expensive test and the crop removes
      // most of the image. `output_from_optical` and every capsule endpoint were resolved at the
      // depth image's stamp by the caller.
      const bool on_robot = std::ranges::any_of(
        self_filter, [&point](const SelfFilterCapsule & capsule) {
          return squared_distance_to_segment(point, capsule) <= capsule.radius * capsule.radius;
        });
      if (on_robot) {
        continue;
      }
      const VoxelKey key{
        voxel_index(point.x(), config.voxel_size_m),
        voxel_index(point.y(), config.voxel_size_m),
        voxel_index(point.z(), config.voxel_size_m)};
      ++voxels[key].point_count;
    }
  }

  // Connected components over the occupied cells: 26-connected, except that the vertical reach is
  // widened by `cluster_vertical_gap_voxels` to close the bands a grazing view of an upright face
  // leaves behind. The stack is explicit because a component can be thousands of cells deep and
  // recursion would overflow the stack.
  const std::int64_t vertical_reach = 1 + config.cluster_vertical_gap_voxels;
  std::size_t component_count = 0U;
  std::vector<VoxelKey> stack;
  for (auto & [seed_key, seed_cell] : voxels) {
    if (seed_cell.assigned) {
      continue;
    }
    const std::size_t component = component_count++;
    seed_cell.assigned = true;
    seed_cell.component = component;
    stack.clear();
    stack.push_back(seed_key);
    while (!stack.empty()) {
      const VoxelKey key = stack.back();
      stack.pop_back();
      for (std::int64_t dx = -1; dx <= 1; ++dx) {
        for (std::int64_t dy = -1; dy <= 1; ++dy) {
          for (std::int64_t dz = -vertical_reach; dz <= vertical_reach; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }
            const auto neighbour =
              voxels.find(VoxelKey{key.x + dx, key.y + dy, key.z + dz});
            if (neighbour == voxels.end() || neighbour->second.assigned) {
              continue;
            }
            neighbour->second.assigned = true;
            neighbour->second.component = component;
            stack.push_back(neighbour->first);
          }
        }
      }
    }
  }

  struct ComponentBounds
  {
    VoxelKey minimum{
      std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
      std::numeric_limits<std::int64_t>::max()};
    VoxelKey maximum{
      std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min(),
      std::numeric_limits<std::int64_t>::min()};
    std::size_t voxel_count{0U};
    std::size_t point_count{0U};
  };
  std::vector<ComponentBounds> bounds(component_count);
  for (const auto & [key, cell] : voxels) {
    auto & entry = bounds[cell.component];
    entry.minimum.x = std::min(entry.minimum.x, key.x);
    entry.minimum.y = std::min(entry.minimum.y, key.y);
    entry.minimum.z = std::min(entry.minimum.z, key.z);
    entry.maximum.x = std::max(entry.maximum.x, key.x);
    entry.maximum.y = std::max(entry.maximum.y, key.y);
    entry.maximum.z = std::max(entry.maximum.z, key.z);
    ++entry.voxel_count;
    entry.point_count += cell.point_count;
  }

  Boxes boxes;
  for (const auto & entry : bounds) {
    if (entry.voxel_count < config.min_cluster_voxels) {
      continue;
    }
    // The box bounds the occupied cells, not their centroids: a cell is occupied anywhere inside
    // its extent, so anything narrower would under-report the obstacle.
    const Eigen::Vector3d lower(
      static_cast<double>(entry.minimum.x) * config.voxel_size_m,
      static_cast<double>(entry.minimum.y) * config.voxel_size_m,
      static_cast<double>(entry.minimum.z) * config.voxel_size_m);
    const Eigen::Vector3d upper(
      static_cast<double>(entry.maximum.x + 1) * config.voxel_size_m,
      static_cast<double>(entry.maximum.y + 1) * config.voxel_size_m,
      static_cast<double>(entry.maximum.z + 1) * config.voxel_size_m);
    const Eigen::Vector3d size = upper - lower;
    if (size.maxCoeff() < config.min_box_extent_m || size.maxCoeff() > config.max_box_extent_m) {
      continue;
    }
    boxes.push_back(ObstacleBoxFit{0.5 * (lower + upper), size, entry.point_count});
  }

  // Keep the most strongly supported clusters when there are more than the representation
  // admits, then restore the deterministic spatial order the message contract promises.
  if (boxes.size() > config.max_boxes) {
    std::ranges::partial_sort(
      boxes, boxes.begin() + static_cast<std::ptrdiff_t>(config.max_boxes), std::greater<>{},
      [](const ObstacleBoxFit & box) {return box.point_count;});
    boxes.resize(config.max_boxes);
  }
  std::ranges::sort(
    boxes, [](const ObstacleBoxFit & left, const ObstacleBoxFit & right) {
      return std::tuple(left.center.x(), left.center.y(), left.center.z()) <
             std::tuple(right.center.x(), right.center.y(), right.center.z());
    });
  return DepthObstacleResult<Boxes>::success(std::move(boxes));
}

const char * to_string(DepthObstacleErrorCode code) noexcept
{
  switch (code) {
    case DepthObstacleErrorCode::InvalidConfiguration: return "invalid_configuration";
    case DepthObstacleErrorCode::InvalidIntrinsics: return "invalid_intrinsics";
    case DepthObstacleErrorCode::InvalidDepthBuffer: return "invalid_depth_buffer";
    case DepthObstacleErrorCode::InvalidTransform: return "invalid_transform";
  }
  return "unknown";
}

}  // namespace restocker_perception
