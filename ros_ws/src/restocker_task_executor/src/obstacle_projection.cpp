// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/obstacle_projection.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace restocker_task_executor
{
namespace
{

using Primitive = shape_msgs::msg::SolidPrimitive;

template<typename T>
[[nodiscard]] ProjectionResult<T> failure(ProjectionErrorCode code, std::string detail)
{
  return ProjectionResult<T>::failure(ProjectionError{code, std::move(detail)});
}

[[nodiscard]] Eigen::Vector3d to_eigen(const geometry_msgs::msg::Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

[[nodiscard]] Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion & value)
{
  return Eigen::Quaterniond(value.w, value.x, value.y, value.z);
}

// Half-extents of a primitive along its own axes, or nothing for a shape this projector has no
// contract for. A cylinder is bounded by the prism that contains it, which is conservative.
[[nodiscard]] std::optional<Eigen::Vector3d> local_half_extents(const Primitive & primitive)
{
  switch (primitive.type) {
    case Primitive::BOX:
      if (primitive.dimensions.size() != 3U) {
        return std::nullopt;
      }
      return Eigen::Vector3d(
        0.5 * primitive.dimensions[0], 0.5 * primitive.dimensions[1],
        0.5 * primitive.dimensions[2]);
    case Primitive::SPHERE:
      if (primitive.dimensions.size() != 1U) {
        return std::nullopt;
      }
      return Eigen::Vector3d::Constant(primitive.dimensions[0]);
    case Primitive::CYLINDER:
      if (primitive.dimensions.size() != 2U) {
        return std::nullopt;
      }
      return Eigen::Vector3d(
        primitive.dimensions[Primitive::CYLINDER_RADIUS],
        primitive.dimensions[Primitive::CYLINDER_RADIUS],
        0.5 * primitive.dimensions[Primitive::CYLINDER_HEIGHT]);
    default:
      return std::nullopt;
  }
}

// Axis-aligned bound of one managed collision object in the frame its pose is expressed in.
// Nothing is returned for an object this projector cannot bound; the caller must treat it as
// covering nothing, since an unbounded "known" volume would swallow obstacles.
[[nodiscard]] std::optional<Eigen::AlignedBox3d> known_object_bounds(
  const moveit_msgs::msg::CollisionObject & object, double padding)
{
  if (object.primitives.size() != 1U || object.primitive_poses.size() != 1U) {
    return std::nullopt;
  }
  const auto half = local_half_extents(object.primitives.front());
  if (!half) {
    return std::nullopt;
  }
  const Eigen::Quaterniond object_rotation = to_eigen(object.pose.orientation);
  const Eigen::Quaterniond primitive_rotation =
    to_eigen(object.primitive_poses.front().orientation);
  if (std::abs(object_rotation.norm() - 1.0) > 1.0e-6 ||
    std::abs(primitive_rotation.norm() - 1.0) > 1.0e-6)
  {
    return std::nullopt;
  }
  const Eigen::Isometry3d frame_from_object =
    Eigen::Translation3d(to_eigen(object.pose.position)) * object_rotation;
  const Eigen::Isometry3d object_from_primitive =
    Eigen::Translation3d(to_eigen(object.primitive_poses.front().position)) * primitive_rotation;
  const Eigen::Isometry3d frame_from_primitive = frame_from_object * object_from_primitive;
  if (!frame_from_primitive.matrix().allFinite()) {
    return std::nullopt;
  }
  // The axis-aligned bound of a rotated box is its half-extents projected onto the frame axes,
  // which is |R| * half. Padding is added afterwards so it is a true clearance, not a scaling.
  const Eigen::Vector3d extent =
    frame_from_primitive.linear().cwiseAbs() * *half + Eigen::Vector3d::Constant(padding);
  const Eigen::Vector3d center = frame_from_primitive.translation();
  return Eigen::AlignedBox3d(center - extent, center + extent);
}

[[nodiscard]] double volume(const Eigen::Vector3d & size)
{
  return size.x() * size.y() * size.z();
}

[[nodiscard]] double overlap_volume(
  const Eigen::AlignedBox3d & left, const Eigen::AlignedBox3d & right)
{
  const Eigen::Vector3d lower = left.min().cwiseMax(right.min());
  const Eigen::Vector3d upper = left.max().cwiseMin(right.max());
  const Eigen::Vector3d span = (upper - lower).cwiseMax(Eigen::Vector3d::Zero());
  return volume(span);
}

[[nodiscard]] moveit_msgs::msg::CollisionObject make_obstacle_object(
  std::size_t index, const std::string & planning_frame, const Eigen::Vector3d & center,
  const Eigen::Vector3d & size)
{
  moveit_msgs::msg::CollisionObject object;
  object.id = std::string(kObstacleIdPrefix) + std::to_string(index);
  object.header.frame_id = planning_frame;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  object.pose.position.x = center.x();
  object.pose.position.y = center.y();
  object.pose.position.z = center.z();
  object.pose.orientation.w = 1.0;
  Primitive primitive;
  primitive.type = Primitive::BOX;
  primitive.dimensions = {size.x(), size.y(), size.z()};
  object.primitives.push_back(primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  object.primitive_poses.push_back(identity);
  return object;
}

[[nodiscard]] bool obstacle_geometry(
  const moveit_msgs::msg::CollisionObject & object, Eigen::Vector3d & center,
  Eigen::Vector3d & size)
{
  if (object.primitives.size() != 1U ||
    object.primitives.front().type != Primitive::BOX ||
    object.primitives.front().dimensions.size() != 3U)
  {
    return false;
  }
  center = to_eigen(object.pose.position);
  size = Eigen::Vector3d(
    object.primitives.front().dimensions[0], object.primitives.front().dimensions[1],
    object.primitives.front().dimensions[2]);
  return true;
}

}  // namespace

bool valid_obstacle_projection_config(const ObstacleProjectionConfig & config) noexcept
{
  if (config.planning_frame.empty()) {
    return false;
  }
  const bool finite_lengths = std::isfinite(config.obstacle_padding_m) &&
    config.obstacle_padding_m >= 0.0 && std::isfinite(config.min_extent_m) &&
    config.min_extent_m > 0.0 && std::isfinite(config.max_extent_m) &&
    config.max_extent_m > config.min_extent_m && std::isfinite(config.known_padding_m) &&
    config.known_padding_m >= 0.0 && std::isfinite(config.change_position_tolerance_m) &&
    config.change_position_tolerance_m >= 0.0 &&
    std::isfinite(config.change_size_tolerance_m) && config.change_size_tolerance_m >= 0.0;
  if (!finite_lengths || config.max_obstacles == 0U) {
    return false;
  }
  return std::isfinite(config.known_containment_fraction) &&
         config.known_containment_fraction > 0.0 && config.known_containment_fraction <= 1.0;
}

ProjectionResult<std::vector<moveit_msgs::msg::CollisionObject>> project_obstacle_observation(
  const restocker_interfaces::msg::ObstacleObservation & observation,
  const std::vector<moveit_msgs::msg::CollisionObject> & known_world_objects,
  const std::vector<moveit_msgs::msg::CollisionObject> & known_attached_geometry,
  const ObstacleProjectionConfig & config)
{
  using Objects = std::vector<moveit_msgs::msg::CollisionObject>;

  if (!valid_obstacle_projection_config(config)) {
    return failure<Objects>(
      ProjectionErrorCode::InvalidConfiguration, "obstacle projection configuration is invalid");
  }
  if (observation.header.frame_id != config.planning_frame) {
    return failure<Objects>(
      ProjectionErrorCode::FrameMismatch,
      "obstacle observation is expressed in '" + observation.header.frame_id +
      "', not in the planning frame '" + config.planning_frame + "'");
  }
  if (observation.sequence == 0U) {
    return failure<Objects>(
      ProjectionErrorCode::InvalidIdentity, "obstacle observation carries no sequence");
  }

  std::vector<Eigen::AlignedBox3d> known;
  known.reserve(known_world_objects.size() + known_attached_geometry.size());
  for (const auto * source : {&known_world_objects, &known_attached_geometry}) {
    for (const auto & object : *source) {
      if (auto bounds = known_object_bounds(object, config.known_padding_m)) {
        known.push_back(*bounds);
      }
    }
  }

  Objects objects;
  for (const auto & box : observation.boxes) {
    const Eigen::Vector3d center(box.center.x, box.center.y, box.center.z);
    const Eigen::Vector3d observed(box.size.x, box.size.y, box.size.z);
    if (!center.allFinite() || !observed.allFinite() || observed.minCoeff() <= 0.0) {
      return failure<Objects>(
        ProjectionErrorCode::InvalidPose, "obstacle observation carries a non-finite box");
    }
    const Eigen::Vector3d padded =
      observed + Eigen::Vector3d::Constant(2.0 * config.obstacle_padding_m);
    if (padded.maxCoeff() < config.min_extent_m || padded.maxCoeff() > config.max_extent_m) {
      continue;
    }
    const Eigen::AlignedBox3d candidate(center - 0.5 * padded, center + 0.5 * padded);
    const double candidate_volume = volume(padded);
    const bool already_modelled = std::ranges::any_of(
      known, [&candidate, candidate_volume, &config](const Eigen::AlignedBox3d & bounds) {
        return overlap_volume(candidate, bounds) >=
               config.known_containment_fraction * candidate_volume;
      });
    if (already_modelled) {
      continue;
    }
    objects.push_back(
      make_obstacle_object(objects.size(), config.planning_frame, center, padded));
    if (objects.size() >= config.max_obstacles) {
      break;
    }
  }
  return ProjectionResult<Objects>::success(std::move(objects));
}

bool obstacle_sets_match(
  const std::vector<moveit_msgs::msg::CollisionObject> & accepted,
  const std::vector<moveit_msgs::msg::CollisionObject> & candidate,
  const ObstacleProjectionConfig & config)
{
  if (!valid_obstacle_projection_config(config) || accepted.size() != candidate.size()) {
    return false;
  }
  // The identifier is positional, so a set that matches pairwise in order is the same set. The
  // producer already orders its boxes deterministically by centre, which is what makes that true.
  for (std::size_t index = 0; index < accepted.size(); ++index) {
    Eigen::Vector3d accepted_center;
    Eigen::Vector3d accepted_size;
    Eigen::Vector3d candidate_center;
    Eigen::Vector3d candidate_size;
    if (!obstacle_geometry(accepted[index], accepted_center, accepted_size) ||
      !obstacle_geometry(candidate[index], candidate_center, candidate_size) ||
      accepted[index].id != candidate[index].id)
    {
      return false;
    }
    if ((accepted_center - candidate_center).cwiseAbs().maxCoeff() >
      config.change_position_tolerance_m)
    {
      return false;
    }
    if ((accepted_size - candidate_size).cwiseAbs().maxCoeff() > config.change_size_tolerance_m) {
      return false;
    }
  }
  return true;
}

}  // namespace restocker_task_executor
