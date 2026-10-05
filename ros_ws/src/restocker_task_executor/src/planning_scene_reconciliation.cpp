// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/planning_scene_reconciliation.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace restocker_task_executor
{
namespace
{

constexpr std::string_view kProductPrefix = "restocker/object/";

template<typename T>
[[nodiscard]] ProjectionResult<T> failure(ProjectionErrorCode code, std::string detail)
{
  return ProjectionResult<T>::failure(ProjectionError{code, std::move(detail)});
}

// Bounded numeric evidence distinguishes shape/pose chatter from additions or lost objects.
// Logging this at INFO on a real diff does not change verification or its tolerances.
std::string geometry_summary(const moveit_msgs::msg::CollisionObject & object)
{
  std::ostringstream out;
  out << std::setprecision(17) << "frame=" << object.header.frame_id;
  const auto write_pose = [&out](const geometry_msgs::msg::Pose & value) {
    out << " xyz=[" << value.position.x << "," << value.position.y << "," << value.position.z <<
      "] quat=[" << value.orientation.x << "," << value.orientation.y << "," <<
      value.orientation.z << "," << value.orientation.w << "]";
  };
  write_pose(object.pose);
  out << " primitives=" << object.primitives.size();
  if (!object.primitives.empty()) {
    const auto & primitive = object.primitives.front();
    out << " type=" << static_cast<unsigned>(primitive.type) << " dimensions=[";
    for (std::size_t index = 0; index < std::min(std::size_t{3}, primitive.dimensions.size());
      ++index)
    {
      out << (index == 0U ? "" : ",") << primitive.dimensions[index];
    }
    out << "]";
  }
  if (!object.primitive_poses.empty()) {
    out << " local";
    write_pose(object.primitive_poses.front());
  }
  return out.str();
}

[[nodiscard]] bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

[[nodiscard]] bool valid_primitive(const shape_msgs::msg::SolidPrimitive & primitive)
{
  if (primitive.type != shape_msgs::msg::SolidPrimitive::BOX &&
    primitive.type != shape_msgs::msg::SolidPrimitive::SPHERE &&
    primitive.type != shape_msgs::msg::SolidPrimitive::CYLINDER)
  {
    return false;
  }
  const std::size_t expected_dimensions =
    primitive.type == shape_msgs::msg::SolidPrimitive::BOX ? 3U :
    primitive.type == shape_msgs::msg::SolidPrimitive::SPHERE ? 1U : 2U;
  if (primitive.dimensions.size() != expected_dimensions) {
    return false;
  }
  return std::ranges::all_of(
    primitive.dimensions, [](double value) {return std::isfinite(value) && value > 0.0;});
}

[[nodiscard]] std::optional<std::string> invalid_expected_object_reason(
  const moveit_msgs::msg::CollisionObject & object)
{
  if (!is_projector_managed_id(object.id)) {
    return "ID is outside the projector-managed namespace";
  }
  if (object.header.frame_id.empty()) {
    return "planning frame is empty";
  }
  if (object.operation != moveit_msgs::msg::CollisionObject::ADD) {
    return "operation is not ADD";
  }
  if (!finite_pose(object.pose)) {
    return "object pose is non-finite";
  }
  if (object.primitives.size() != 1U || !valid_primitive(object.primitives.front())) {
    return "object does not contain exactly one valid primitive";
  }
  if (object.primitive_poses.size() != 1U || !finite_pose(object.primitive_poses.front())) {
    return "primitive does not contain exactly one finite local pose";
  }
  const Eigen::Quaterniond object_quaternion(
    object.pose.orientation.w, object.pose.orientation.x,
    object.pose.orientation.y, object.pose.orientation.z);
  const auto & primitive_pose = object.primitive_poses.front();
  const Eigen::Quaterniond primitive_quaternion(
    primitive_pose.orientation.w, primitive_pose.orientation.x,
    primitive_pose.orientation.y, primitive_pose.orientation.z);
  if (std::abs(object_quaternion.norm() - 1.0) > 1.0e-6 ||
    std::abs(primitive_quaternion.norm() - 1.0) > 1.0e-6)
  {
    return "object or primitive pose contains a non-unit quaternion";
  }
  if (!object.meshes.empty() || !object.mesh_poses.empty() || !object.planes.empty() ||
    !object.plane_poses.empty() || !object.subframe_names.empty() ||
    !object.subframe_poses.empty())
  {
    return "object contains unsupported auxiliary geometry";
  }
  return std::nullopt;
}

[[nodiscard]] ProjectionResult<std::map<std::string, moveit_msgs::msg::CollisionObject>>
desired_object_map(const std::vector<moveit_msgs::msg::CollisionObject> & objects)
{
  std::map<std::string, moveit_msgs::msg::CollisionObject> desired;
  for (const auto & object : objects) {
    if (const auto reason = invalid_expected_object_reason(object)) {
      return failure<std::map<std::string, moveit_msgs::msg::CollisionObject>>(
        ProjectionErrorCode::InvalidScene,
        "invalid desired collision object '" + object.id + "': " + *reason);
    }
    if (!desired.emplace(object.id, object).second) {
      return failure<std::map<std::string, moveit_msgs::msg::CollisionObject>>(
        ProjectionErrorCode::InvalidScene,
        "duplicate desired collision object ID: " + object.id);
    }
  }
  return ProjectionResult<std::map<std::string, moveit_msgs::msg::CollisionObject>>::success(
    std::move(desired));
}

[[nodiscard]] ProjectionResult<std::map<std::string, moveit_msgs::msg::CollisionObject>>
observed_managed_object_map(const moveit_msgs::msg::PlanningScene & scene)
{
  std::map<std::string, moveit_msgs::msg::CollisionObject> observed;
  for (const auto & object : scene.world.collision_objects) {
    if (is_projector_managed_id(object.id) && !observed.emplace(object.id, object).second) {
      return failure<std::map<std::string, moveit_msgs::msg::CollisionObject>>(
        ProjectionErrorCode::VerificationMismatch,
        "observed scene contains duplicate managed collision IDs");
    }
  }
  return ProjectionResult<std::map<std::string, moveit_msgs::msg::CollisionObject>>::success(
    std::move(observed));
}

[[nodiscard]] ProjectionResult<std::set<std::string>> attached_managed_ids(
  const moveit_msgs::msg::PlanningScene & scene)
{
  std::set<std::string> ids;
  for (const auto & attached : scene.robot_state.attached_collision_objects) {
    if (is_projector_managed_id(attached.object.id) &&
      !ids.insert(attached.object.id).second)
    {
      return failure<std::set<std::string>>(
        ProjectionErrorCode::VerificationMismatch,
        "observed scene contains duplicate managed attached IDs");
    }
  }
  return ProjectionResult<std::set<std::string>>::success(std::move(ids));
}

[[nodiscard]] bool poses_match(
  const geometry_msgs::msg::Pose & expected, const geometry_msgs::msg::Pose & observed,
  const SceneVerificationConfig & config)
{
  if (!finite_pose(observed)) {
    return false;
  }
  const Eigen::Vector3d expected_position(
    expected.position.x, expected.position.y, expected.position.z);
  const Eigen::Vector3d observed_position(
    observed.position.x, observed.position.y, observed.position.z);
  if ((expected_position - observed_position).norm() > config.position_tolerance) {
    return false;
  }
  const Eigen::Quaterniond expected_orientation(
    expected.orientation.w, expected.orientation.x,
    expected.orientation.y, expected.orientation.z);
  const Eigen::Quaterniond observed_orientation(
    observed.orientation.w, observed.orientation.x,
    observed.orientation.y, observed.orientation.z);
  if (std::abs(observed_orientation.norm() - 1.0) > 1.0e-6) {
    return false;
  }
  // Measured on the relative rotation rather than as `2 * acos(|dot|)`. The two agree on paper,
  // but near identity acos has no significant digits left: one unit in the last place of a dot
  // product that should be exactly 1.0 reads as 3e-8 rad, which is three times the tolerance this
  // projector runs with, so a pose would fail against a bit-identical copy of itself.
  const Eigen::Quaterniond error =
    expected_orientation.normalized().conjugate() * observed_orientation.normalized();
  return 2.0 * std::atan2(error.vec().norm(), std::abs(error.w())) <=
         config.orientation_tolerance_rad;
}

[[nodiscard]] bool geometry_matches(
  const moveit_msgs::msg::CollisionObject & expected,
  const moveit_msgs::msg::CollisionObject & observed,
  const SceneVerificationConfig & config)
{
  if (observed.header.frame_id != expected.header.frame_id ||
    observed.primitives.size() != 1U || !valid_primitive(observed.primitives.front()) ||
    observed.primitive_poses.size() != 1U ||
    !observed.meshes.empty() || !observed.mesh_poses.empty() || !observed.planes.empty() ||
    !observed.plane_poses.empty() || !observed.subframe_names.empty() ||
    !observed.subframe_poses.empty() || !poses_match(expected.pose, observed.pose, config))
  {
    return false;
  }
  if (!poses_match(
      expected.primitive_poses.front(), observed.primitive_poses.front(), config))
  {
    return false;
  }
  const auto & expected_primitive = expected.primitives.front();
  const auto & observed_primitive = observed.primitives.front();
  if (observed_primitive.type != expected_primitive.type ||
    observed_primitive.dimensions.size() != expected_primitive.dimensions.size())
  {
    return false;
  }
  for (std::size_t index = 0; index < expected_primitive.dimensions.size(); ++index) {
    if (std::abs(
        expected_primitive.dimensions[index] - observed_primitive.dimensions[index]) >
      config.dimension_tolerance)
    {
      return false;
    }
  }
  return true;
}

}  // namespace

bool is_projector_managed_id(const std::string & id)
{
  constexpr std::string_view kWorkcellPrefix = "restocker/workcell/";
  // Depth-derived obstacles are managed geometry like any other: the diff adds and removes them and
  // verify_planning_scene proves they arrived. Keeping them outside this namespace would need a
  // second, parallel path into the scene, and the change-detection counter that invalidates a plan
  // in flight only watches this one.
  constexpr std::string_view kObstaclePrefix = "restocker/obstacle/";
  // A lane's occupied volume, one box per lane, standing in for every product inside it. Products
  // in lanes carry no geometry, so this is the only thing in the scene between the arm and a
  // stocked column. It is managed on the same terms as the rest: added and removed by the diff,
  // proven by verify_planning_scene, and watched by the counter that invalidates plans in flight,
  // which changes every time a product is placed.
  constexpr std::string_view kLanePrefix = "restocker/lane/";
  // A scenario-declared product seeded before world state tracks it (Card 050). Managed on the
  // same terms as everything else so the diff can remove the seed in the cycle the tracked
  // object replaces it, and verify_planning_scene proves the seed arrived; it is never attached
  // (a product can only be grasped after it is tracked), so the product-specific paths above
  // never see it.
  constexpr std::string_view kDeclaredPrefix = "restocker/declared/";
  return id.starts_with(kProductPrefix) || id.starts_with(kWorkcellPrefix) ||
         id.starts_with(kObstaclePrefix) || id.starts_with(kLanePrefix) ||
         id.starts_with(kDeclaredPrefix);
}

ProjectionResult<moveit_msgs::msg::PlanningScene> build_planning_scene_diff(
  const std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const moveit_msgs::msg::PlanningScene & current_scene)
{
  auto desired = desired_object_map(desired_world_objects);
  if (!desired) {
    return failure<moveit_msgs::msg::PlanningScene>(
      desired.error().code, desired.error().detail);
  }

  moveit_msgs::msg::PlanningScene diff;
  diff.is_diff = true;
  diff.robot_state.is_diff = true;
  for (const auto & [id, object] : desired.value()) {
    (void)id;
    diff.world.collision_objects.push_back(object);
  }
  std::set<std::string> current_managed_ids;
  for (const auto & object : current_scene.world.collision_objects) {
    if (is_projector_managed_id(object.id)) {
      current_managed_ids.insert(object.id);
    }
  }
  for (const auto & id : current_managed_ids) {
    if (!desired.value().contains(id)) {
      moveit_msgs::msg::CollisionObject removal;
      removal.id = id;
      removal.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      diff.world.collision_objects.push_back(std::move(removal));
    }
  }
  std::ranges::sort(
    diff.world.collision_objects, {},
    [](const moveit_msgs::msg::CollisionObject & object) {return object.id;});
  return ProjectionResult<moveit_msgs::msg::PlanningScene>::success(std::move(diff));
}

ProjectionResult<std::monostate> verify_planning_scene(
  const std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const std::set<std::string> & required_attached_ids,
  const moveit_msgs::msg::PlanningScene & observed_scene,
  const SceneVerificationConfig & config)
{
  if (!std::isfinite(config.dimension_tolerance) || config.dimension_tolerance < 0.0 ||
    !std::isfinite(config.position_tolerance) || config.position_tolerance < 0.0 ||
    !std::isfinite(config.orientation_tolerance_rad) ||
    config.orientation_tolerance_rad < 0.0)
  {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidConfiguration, "scene verification tolerances are invalid");
  }
  auto desired = desired_object_map(desired_world_objects);
  if (!desired) {
    return failure<std::monostate>(desired.error().code, desired.error().detail);
  }
  auto observed = observed_managed_object_map(observed_scene);
  if (!observed) {
    return failure<std::monostate>(observed.error().code, observed.error().detail);
  }
  for (const auto & [id, expected] : desired.value()) {
    const auto iterator = observed.value().find(id);
    if (iterator == observed.value().end() || !geometry_matches(
        expected, iterator->second,
        config))
    {
      return failure<std::monostate>(
        ProjectionErrorCode::VerificationMismatch,
        "observed managed geometry differs for " + id + "; desired {" + geometry_summary(expected) +
        "}; observed {" + (iterator == observed.value().end() ? "missing" :
        geometry_summary(iterator->second)) + "}");
    }
  }
  for (const auto & [id, object] : observed.value()) {
    if (!desired.value().contains(id)) {
      return failure<std::monostate>(
        ProjectionErrorCode::VerificationMismatch,
        "observed unexpected managed geometry " + id + "; observed {" + geometry_summary(object) +
        "}");
    }
  }
  if (std::ranges::any_of(
      required_attached_ids,
      [](const std::string & id) {return !id.starts_with("restocker/object/");}))
  {
    return failure<std::monostate>(
      ProjectionErrorCode::InvalidScene,
      "required attachment set contains a non-product ID");
  }
  auto observed_attached = attached_managed_ids(observed_scene);
  if (!observed_attached) {
    return failure<std::monostate>(
      observed_attached.error().code, observed_attached.error().detail);
  }
  if (observed_attached.value() != required_attached_ids) {
    return failure<std::monostate>(
      ProjectionErrorCode::VerificationMismatch,
      "observed managed attached-object set differs from required set");
  }
  return ProjectionResult<std::monostate>::success(std::monostate{});
}

void retain_unmoved_product_geometry(
  std::vector<moveit_msgs::msg::CollisionObject> & desired_world_objects,
  const moveit_msgs::msg::PlanningScene & observed_scene,
  const SceneVerificationConfig & change_tolerance)
{
  if (!std::isfinite(change_tolerance.position_tolerance) ||
    change_tolerance.position_tolerance < 0.0 ||
    !std::isfinite(change_tolerance.orientation_tolerance_rad) ||
    change_tolerance.orientation_tolerance_rad < 0.0)
  {
    return;
  }
  std::map<std::string, const moveit_msgs::msg::CollisionObject *> observed;
  for (const auto & object : observed_scene.world.collision_objects) {
    if (object.id.starts_with(kProductPrefix)) {
      observed.emplace(object.id, &object);
    }
  }
  for (auto & desired : desired_world_objects) {
    if (!desired.id.starts_with(kProductPrefix)) {
      continue;
    }
    const auto found = observed.find(desired.id);
    // geometry_matches still holds dimensions and identity to the caller's exact tolerance; only
    // the pose comparison inside it is the loosened one, so a product whose shape or frame
    // changed is never retained, however little it moved.
    if (found == observed.end() ||
      !geometry_matches(desired, *found->second, change_tolerance))
    {
      continue;
    }
    desired.pose = found->second->pose;
    desired.primitive_poses = found->second->primitive_poses;
  }
}

}  // namespace restocker_task_executor
