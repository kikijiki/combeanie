// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/planning_scene_attachment.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <shape_msgs/msg/solid_primitive.hpp>

namespace restocker_task_executor
{
namespace
{

template<typename T>
[[nodiscard]] ProjectionResult<T> failure(ProjectionErrorCode code, std::string detail)
{
  return ProjectionResult<T>::failure(ProjectionError{code, std::move(detail)});
}

[[nodiscard]] bool finite_unit_pose(const geometry_msgs::msg::Pose & pose)
{
  const bool finite = std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
    std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
    std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
    std::isfinite(pose.orientation.w);
  const Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return finite && std::abs(rotation.norm() - 1.0) <= 1.0e-6;
}

[[nodiscard]] bool valid_primitive(const shape_msgs::msg::SolidPrimitive & primitive)
{
  const std::size_t expected = primitive.type == shape_msgs::msg::SolidPrimitive::BOX ? 3U :
    primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE ? 1U :
    primitive.type == shape_msgs::msg::SolidPrimitive::CYLINDER ? 2U : 0U;
  return expected != 0U && primitive.dimensions.size() == expected &&
         std::ranges::all_of(
    primitive.dimensions,
    [](double value) {return std::isfinite(value) && value > 0.0;});
}

[[nodiscard]] bool valid_product_id(const std::string & id)
{
  constexpr std::string_view kPrefix = "restocker/object/";
  if (!id.starts_with(kPrefix)) {
    return false;
  }
  const std::string_view suffix{id.data() + kPrefix.size(), id.size() - kPrefix.size()};
  return !suffix.empty() && std::ranges::all_of(
    suffix, [](char value) {return value >= '0' && value <= '9';});
}

[[nodiscard]] bool valid_geometry(const moveit_msgs::msg::CollisionObject & object)
{
  return valid_product_id(object.id) && !object.header.frame_id.empty() &&
         finite_unit_pose(object.pose) && object.primitives.size() == 1U &&
         valid_primitive(object.primitives.front()) && object.primitive_poses.size() == 1U &&
         finite_unit_pose(object.primitive_poses.front()) && object.meshes.empty() &&
         object.mesh_poses.empty() && object.planes.empty() && object.plane_poses.empty() &&
         object.subframe_names.empty() && object.subframe_poses.empty();
}

[[nodiscard]] std::optional<std::vector<std::string>> canonical_touch_links(
  const std::string & parent_link, const std::vector<std::string> & touch_links)
{
  if (parent_link.empty() || touch_links.empty() ||
    std::ranges::any_of(touch_links, [](const std::string & value) {return value.empty();}))
  {
    return std::nullopt;
  }
  std::vector<std::string> result = touch_links;
  std::ranges::sort(result);
  if (std::ranges::adjacent_find(result) != result.end() ||
    !std::ranges::binary_search(result, parent_link))
  {
    return std::nullopt;
  }
  return result;
}

[[nodiscard]] Eigen::Quaterniond rotation_of(const geometry_msgs::msg::Pose & pose)
{
  return Eigen::Quaterniond(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
}

// Angle between two orientations, measured on their relative rotation. `2 * acos(|dot|)` is the
// same quantity on paper, but near identity it loses every significant digit it has: one unit in
// the last place of a dot product that should be exactly 1.0 comes back as 3e-8 rad, so with that
// form a pose compared against a bit-identical copy of itself fails any tolerance tighter than
// that, which the 1e-8 rad this projector is configured with is. This form stays exact there.
[[nodiscard]] double orientation_difference_rad(
  const geometry_msgs::msg::Pose & expected, const geometry_msgs::msg::Pose & observed)
{
  const Eigen::Quaterniond error =
    rotation_of(expected).normalized().conjugate() * rotation_of(observed).normalized();
  return 2.0 * std::atan2(error.vec().norm(), std::abs(error.w()));
}

[[nodiscard]] double position_difference_m(
  const geometry_msgs::msg::Pose & expected, const geometry_msgs::msg::Pose & observed)
{
  return (Eigen::Vector3d(expected.position.x, expected.position.y, expected.position.z) -
         Eigen::Vector3d(observed.position.x, observed.position.y, observed.position.z)).norm();
}

[[nodiscard]] bool poses_match(
  const geometry_msgs::msg::Pose & expected,
  const geometry_msgs::msg::Pose & observed,
  const SceneVerificationConfig & config)
{
  return finite_unit_pose(expected) && finite_unit_pose(observed) &&
         position_difference_m(expected, observed) <= config.position_tolerance &&
         orientation_difference_rad(expected, observed) <= config.orientation_tolerance_rad;
}

[[nodiscard]] std::string describe_pose_difference(
  const geometry_msgs::msg::Pose & expected, const geometry_msgs::msg::Pose & observed)
{
  std::ostringstream stream;
  stream << position_difference_m(expected, observed) << " m and "
         << orientation_difference_rad(expected, observed) << " rad";
  return stream.str();
}

// Names the first way two collision objects differ in shape, or nothing when they agree. The
// object's own pose is not part of this: what that pose proves depends on which transition is being
// checked, so each caller judges it. A mismatch here is several service calls removed from its
// cause, so the description carries the numbers it judged.
[[nodiscard]] std::optional<std::string> shape_difference(
  const moveit_msgs::msg::CollisionObject & expected,
  const moveit_msgs::msg::CollisionObject & observed,
  const SceneVerificationConfig & config)
{
  if (!valid_geometry(expected) || !valid_geometry(observed)) {
    return "one of the two collision-object representations is malformed";
  }
  if (expected.id != observed.id) {
    return "id is '" + observed.id + "', expected '" + expected.id + "'";
  }
  if (expected.header.frame_id != observed.header.frame_id) {
    return "frame is '" + observed.header.frame_id + "', expected '" +
           expected.header.frame_id + "'";
  }
  if (!poses_match(expected.primitive_poses.front(), observed.primitive_poses.front(), config)) {
    return "primitive pose differs by " +
           describe_pose_difference(
      expected.primitive_poses.front(), observed.primitive_poses.front());
  }
  if (expected.primitives.front().type != observed.primitives.front().type ||
    expected.primitives.front().dimensions.size() !=
    observed.primitives.front().dimensions.size())
  {
    return "primitive shape differs from expectation";
  }
  for (std::size_t index = 0; index < expected.primitives.front().dimensions.size(); ++index) {
    const double difference = std::abs(
      expected.primitives.front().dimensions[index] -
      observed.primitives.front().dimensions[index]);
    if (difference > config.dimension_tolerance) {
      std::ostringstream stream;
      stream << "primitive dimension " << index << " differs by " << difference << " m";
      return stream.str();
    }
  }
  return std::nullopt;
}

[[nodiscard]] std::optional<std::string> geometry_difference(
  const moveit_msgs::msg::CollisionObject & expected,
  const moveit_msgs::msg::CollisionObject & observed,
  const SceneVerificationConfig & config)
{
  if (auto shape = shape_difference(expected, observed, config)) {
    return shape;
  }
  if (!poses_match(expected.pose, observed.pose, config)) {
    return "pose differs by " + describe_pose_difference(expected.pose, observed.pose);
  }
  return std::nullopt;
}

[[nodiscard]] bool valid_config(const SceneVerificationConfig & config)
{
  return std::isfinite(config.dimension_tolerance) && config.dimension_tolerance >= 0.0 &&
         std::isfinite(config.position_tolerance) && config.position_tolerance >= 0.0 &&
         std::isfinite(config.orientation_tolerance_rad) &&
         config.orientation_tolerance_rad >= 0.0;
}

}  // namespace

ProjectionResult<AttachedObjectProjection> build_world_to_attached_diff(
  const moveit_msgs::msg::CollisionObject & world_object,
  const std::string & parent_link,
  const std::vector<std::string> & touch_links,
  const geometry_msgs::msg::Pose & observed_parent_to_child)
{
  const auto canonical_links = canonical_touch_links(parent_link, touch_links);
  if (!valid_geometry(world_object) ||
    world_object.operation != moveit_msgs::msg::CollisionObject::ADD || !canonical_links ||
    !finite_unit_pose(observed_parent_to_child))
  {
    return failure<AttachedObjectProjection>(
      ProjectionErrorCode::InvalidScene,
      "world-to-attached request contains invalid geometry, transform, link, or touch links");
  }
  AttachedObjectProjection result;
  result.diff.is_diff = true;
  result.diff.robot_state.is_diff = true;
  moveit_msgs::msg::CollisionObject removal;
  removal.header.frame_id = world_object.header.frame_id;
  removal.id = world_object.id;
  removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  result.diff.world.collision_objects.push_back(std::move(removal));
  result.expected_attached_object.link_name = parent_link;
  result.expected_attached_object.touch_links = *canonical_links;
  result.expected_attached_object.object = world_object;
  result.expected_attached_object.object.header.frame_id = parent_link;
  result.expected_attached_object.object.pose = observed_parent_to_child;
  result.expected_attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  result.diff.robot_state.attached_collision_objects.push_back(
    result.expected_attached_object);
  return ProjectionResult<AttachedObjectProjection>::success(std::move(result));
}

ProjectionResult<DetachedObjectProjection> build_attached_to_world_diff(
  const moveit_msgs::msg::AttachedCollisionObject & attached_object,
  const std::string & planning_frame,
  const geometry_msgs::msg::Pose & observed_child_in_world)
{
  const auto canonical_links = canonical_touch_links(
    attached_object.link_name, attached_object.touch_links);
  if (!valid_geometry(attached_object.object) ||
    attached_object.object.operation != moveit_msgs::msg::CollisionObject::ADD ||
    attached_object.object.header.frame_id != attached_object.link_name || !canonical_links ||
    planning_frame.empty() || !finite_unit_pose(observed_child_in_world))
  {
    return failure<DetachedObjectProjection>(
      ProjectionErrorCode::InvalidScene,
      "attached-to-world request contains invalid geometry, transform, link, or touch links");
  }
  DetachedObjectProjection result;
  result.diff.is_diff = true;
  result.diff.robot_state.is_diff = true;
  moveit_msgs::msg::AttachedCollisionObject removal;
  removal.link_name = attached_object.link_name;
  removal.object.id = attached_object.object.id;
  removal.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  result.diff.robot_state.attached_collision_objects.push_back(std::move(removal));
  result.expected_world_object = attached_object.object;
  result.expected_world_object.header.frame_id = planning_frame;
  result.expected_world_object.pose = observed_child_in_world;
  result.expected_world_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  result.diff.world.collision_objects.push_back(result.expected_world_object);
  return ProjectionResult<DetachedObjectProjection>::success(std::move(result));
}

ProjectionResult<std::monostate> verify_world_to_attached_transition(
  const moveit_msgs::msg::AttachedCollisionObject & expected,
  const moveit_msgs::msg::PlanningScene & observed,
  const SceneVerificationConfig & config)
{
  if (!valid_config(config)) {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidConfiguration, "attachment verification tolerances are invalid");
  }
  const auto expected_links = canonical_touch_links(expected.link_name, expected.touch_links);
  if (!expected_links || !valid_geometry(expected.object) ||
    expected.object.operation != moveit_msgs::msg::CollisionObject::ADD ||
    expected.object.header.frame_id != expected.link_name)
  {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidScene, "expected attached-object representation is invalid");
  }
  if (std::ranges::any_of(
      observed.world.collision_objects,
      [&expected](const auto & object) {return object.id == expected.object.id;}))
  {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "attached object remains in the planning-scene world");
  }
  std::vector<const moveit_msgs::msg::AttachedCollisionObject *> matches;
  for (const auto & candidate : observed.robot_state.attached_collision_objects) {
    if (candidate.object.id == expected.object.id) {
      matches.push_back(&candidate);
    }
  }
  if (matches.size() != 1U) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "planning scene does not contain exactly one expected attached object");
  }
  auto observed_links = matches.front()->touch_links;
  std::ranges::sort(observed_links);
  if (matches.front()->link_name != expected.link_name) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "attached object hangs off '" + matches.front()->link_name + "', expected '" +
      expected.link_name + "'");
  }
  if (*expected_links != observed_links) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "attached object carries the wrong touch links");
  }
  // Shape, not pose. A gripped product is rigidly attached, so once MoveIt carries it the pose
  // follows from the link it hangs off and the grasp, derived from the transform this diff stated;
  // reading it back proves nothing new. Whether the product is still in the gripper is asked at the
  // grasp and release by the attachment transaction, which weighs real evidence. What is worth
  // proving here is that the scene carries the right product on the right link with the right
  // geometry and collision exemptions.
  if (auto difference = shape_difference(expected.object, matches.front()->object, config)) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch, "attached object " + *difference);
  }
  return ProjectionResult<std::monostate>::success(std::monostate{});
}

ProjectionResult<std::monostate> verify_attached_to_world_transition(
  const moveit_msgs::msg::CollisionObject & expected,
  const moveit_msgs::msg::PlanningScene & observed,
  const SceneVerificationConfig & config)
{
  if (!valid_config(config)) {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidConfiguration, "detachment verification tolerances are invalid");
  }
  if (!valid_geometry(expected) ||
    expected.operation != moveit_msgs::msg::CollisionObject::ADD)
  {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidScene,
      "expected detached world-object representation is invalid");
  }
  if (std::ranges::any_of(
      observed.robot_state.attached_collision_objects,
      [&expected](const auto & object) {return object.object.id == expected.id;}))
  {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "detached object remains attached in the planning scene");
  }
  std::vector<const moveit_msgs::msg::CollisionObject *> matches;
  for (const auto & candidate : observed.world.collision_objects) {
    if (candidate.id == expected.id) {
      matches.push_back(&candidate);
    }
  }
  if (matches.size() != 1U) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "planning scene does not contain exactly one expected detached world object");
  }
  // The released product's world pose is stated by this projector rather than derived by MoveIt
  // from the robot, so unlike the attached case it is worth reading back: it proves the scene
  // holds the product where the release said it was put down.
  if (auto difference = geometry_difference(expected, *matches.front(), config)) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch, "detached world object " + *difference);
  }
  return ProjectionResult<std::monostate>::success(std::monostate{});
}

}  // namespace restocker_task_executor
