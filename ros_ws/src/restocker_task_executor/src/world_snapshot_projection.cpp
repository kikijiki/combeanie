// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/world_snapshot_projection.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <restocker_interfaces/msg/tracked_object.hpp>

namespace restocker_task_executor
{
namespace
{

constexpr std::string_view kLanePrefix = "restocker/lane/";
[[nodiscard]] geometry_msgs::msg::Pose pose_to_message(const Eigen::Isometry3d & pose)
{
  const Eigen::Quaterniond rotation(pose.linear());
  geometry_msgs::msg::Pose message;
  message.position.x = pose.translation().x();
  message.position.y = pose.translation().y();
  message.position.z = pose.translation().z();
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

// Below this (one micrometre) the lane counts as empty; avoids a degenerate box primitive.
constexpr double kLaneOccupancyEpsilonM = 1.0e-6;

using TrackedObjectMessage = restocker_interfaces::msg::TrackedObject;

template<typename T>
[[nodiscard]] ProjectionResult<T> failure(ProjectionErrorCode code, std::string detail)
{
  return ProjectionResult<T>::failure(ProjectionError{code, std::move(detail)});
}

[[nodiscard]] std::string object_context(std::uint64_t object_id, const std::string & detail)
{
  std::ostringstream stream;
  stream << "object " << object_id << ": " << detail;
  return stream.str();
}

[[nodiscard]] std::optional<std::int64_t> time_nanoseconds(
  const builtin_interfaces::msg::Time & time)
{
  constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
  if (time.nanosec >= kNanosecondsPerSecond) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(time.sec) * kNanosecondsPerSecond +
         static_cast<std::int64_t>(time.nanosec);
}

[[nodiscard]] std::optional<CatalogProductClass> product_class(std::uint8_t value)
{
  switch (value) {
    case TrackedObjectMessage::PRODUCT_CLASS_CAN:
      return CatalogProductClass::Can;
    case TrackedObjectMessage::PRODUCT_CLASS_SMALL_BOTTLE:
      return CatalogProductClass::SmallBottle;
    case TrackedObjectMessage::PRODUCT_CLASS_LARGE_BOTTLE:
      return CatalogProductClass::LargeBottle;
    default:
      return std::nullopt;
  }
}

[[nodiscard]] bool valid_tracking_state(std::uint8_t value)
{
  return value <= TrackedObjectMessage::TRACKING_REMOVED;
}

[[nodiscard]] bool valid_grasp_state(std::uint8_t value)
{
  return value == TrackedObjectMessage::GRASP_FREE || value == TrackedObjectMessage::GRASP_ATTACHED;
}

[[nodiscard]] bool active_object(const TrackedObjectMessage & object)
{
  return object.tracking_state == TrackedObjectMessage::TRACKING_TRACKED ||
         object.tracking_state == TrackedObjectMessage::TRACKING_OCCLUDED ||
         object.grasp_state == TrackedObjectMessage::GRASP_ATTACHED;
}

[[nodiscard]] ProjectionResult<Eigen::Isometry3d> validated_pose(
  const TrackedObjectMessage & object,
  double quaternion_tolerance)
{
  const auto & pose = object.pose.pose;
  const Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !quaternion.coeffs().allFinite())
  {
    return failure<Eigen::Isometry3d>(
      ProjectionErrorCode::InvalidPose,
      object_context(object.id, "pose is not finite"));
  }
  if (std::abs(quaternion.norm() - 1.0) > quaternion_tolerance) {
    return failure<Eigen::Isometry3d>(
      ProjectionErrorCode::InvalidPose,
      object_context(object.id, "quaternion is not normalized"));
  }
  for (const double covariance : object.pose.covariance) {
    if (!std::isfinite(covariance)) {
      return failure<Eigen::Isometry3d>(
        ProjectionErrorCode::InvalidPose,
        object_context(object.id, "pose covariance is not finite"));
    }
  }

  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  transform.linear() = quaternion.normalized().toRotationMatrix();
  return ProjectionResult<Eigen::Isometry3d>::success(transform);
}

[[nodiscard]] ProjectionResult<std::int64_t> validated_observation_time(
  const TrackedObjectMessage & object, const SnapshotProjectionConfig & config)
{
  const auto observation_ns = time_nanoseconds(object.observation_time);
  if (!observation_ns) {
    return failure<std::int64_t>(
      ProjectionErrorCode::InvalidPose,
      object_context(object.id, "observation timestamp is malformed"));
  }
  if (*observation_ns > config.now_ns + config.max_future_skew_ns) {
    return failure<std::int64_t>(
      ProjectionErrorCode::FutureObservation,
      object_context(object.id, "observation exceeds future-skew limit"));
  }
  return ProjectionResult<std::int64_t>::success(*observation_ns);
}

// Pose of a held product, derived from the arm: the current grasp-center pose composed with the
// grasp transform verified at attachment. The grasp is fixed while held, so no second timestamped
// value can be mis-paired with it.
[[nodiscard]] ProjectionResult<Eigen::Isometry3d> held_product_pose(
  const restocker_interfaces::msg::WorldStateSnapshot & snapshot,
  const SnapshotProjectionConfig & config)
{
  if (!config.planning_from_grasp_center) {
    return failure<Eigen::Isometry3d>(
      ProjectionErrorCode::InvalidConfiguration,
      "a held product needs the grasp-center transform this projection was not given");
  }
  const auto & pose = snapshot.robot.grasp_center_from_held_object;
  const Eigen::Quaterniond rotation(pose.orientation.w, pose.orientation.x, pose.orientation.y,
    pose.orientation.z);
  if (!std::isfinite(pose.position.x) || !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !rotation.coeffs().allFinite() ||
    std::abs(rotation.norm() - 1.0) > config.quaternion_norm_tolerance)
  {
    return failure<Eigen::Isometry3d>(
      ProjectionErrorCode::InvalidPose,
      "held-product grasp transform is not a rigid transform");
  }
  Eigen::Isometry3d grasp_center_from_product = Eigen::Isometry3d::Identity();
  grasp_center_from_product.translation() =
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  grasp_center_from_product.linear() = rotation.normalized().toRotationMatrix();
  return ProjectionResult<Eigen::Isometry3d>::success(
    *config.planning_from_grasp_center *
    grasp_center_from_product);
}

// The occupied part of a lane, as one box in the planning frame.
//
// `available_depth_m` is free depth from the rear entrance and products pack against the front
// rail, so everything in the lane lies between `depth_m - available_depth_m` and the front.
// One box derived from that measurement covers all products in the lane, so they need no poses.
//
// Across the lane the box has the column's width, not the lane's: a lane holds one product class,
// a line of cylinders on the centre line. A full-width box would overlap the gripper fingers that
// straddle the just-released product and block the retreat. A lane whose class the catalogue
// cannot name falls back to the full width (conservative).
[[nodiscard]] ProjectionResult<std::optional<moveit_msgs::msg::CollisionObject>> lane_volume_object(
  const restocker_interfaces::msg::ShelfLane & lane, const LaneVolumeGeometry & geometry,
  double column_width_m, const std::string & planning_frame)
{
  const Eigen::AlignedBox3d & bounds = geometry.usable_bounds_in_lane;
  if (bounds.isEmpty() || !bounds.min().allFinite() || !bounds.max().allFinite() ||
    !geometry.planning_from_lane.matrix().allFinite() || !std::isfinite(lane.depth_m) ||
    lane.depth_m <= 0.0 || !std::isfinite(lane.available_depth_m) ||
    lane.available_depth_m < 0.0 || lane.available_depth_m > lane.depth_m ||
    std::abs(lane.depth_m - bounds.sizes().y()) > 1.0e-9)
  {
    return failure<std::optional<moveit_msgs::msg::CollisionObject>>(
      ProjectionErrorCode::InvalidConfiguration,
      "lane " + lane.id + " geometry disagrees with its published depth");
  }
  const double occupied_depth = lane.depth_m - lane.available_depth_m;
  if (occupied_depth <= kLaneOccupancyEpsilonM) {
    return ProjectionResult<std::optional<moveit_msgs::msg::CollisionObject>>::success(
      std::nullopt);
  }
  const Eigen::Vector3d size(column_width_m, occupied_depth, bounds.sizes().z());
  Eigen::Isometry3d lane_from_box = Eigen::Isometry3d::Identity();
  lane_from_box.translation() = Eigen::Vector3d(
    bounds.center().x(), bounds.max().y() - 0.5 * occupied_depth, bounds.center().z());

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame;
  object.id = std::string(kLanePrefix) + lane.id;
  const Eigen::Isometry3d planning_from_box = geometry.planning_from_lane * lane_from_box;
  object.pose = pose_to_message(planning_from_box);
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {size.x(), size.y(), size.z()};
  object.primitives.push_back(primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  object.primitive_poses.push_back(identity);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return ProjectionResult<std::optional<moveit_msgs::msg::CollisionObject>>::success(
    std::move(object));
}

[[nodiscard]] std::set<std::string> managed_attached_ids(const std::set<std::string> & attached_ids)
{
  constexpr std::string_view kProductPrefix = "restocker/object/";
  std::set<std::string> managed;
  for (const auto & id : attached_ids) {
    if (id.starts_with(kProductPrefix)) {
      managed.insert(id);
    }
  }
  return managed;
}

}  // namespace

ProjectionResult<DesiredProductProjection> project_world_snapshot(
  const restocker_interfaces::msg::WorldStateSnapshot & snapshot,
  const SnapshotProjectionConfig & config, const ProductCollisionCatalog & catalog,
  const std::set<std::string> & current_attached_ids)
{
  if (config.planning_frame.empty() || config.max_observation_age_ns < 0 ||
    config.max_future_skew_ns < 0 || !std::isfinite(config.quaternion_norm_tolerance) ||
    config.quaternion_norm_tolerance <= 0.0)
  {
    return failure<DesiredProductProjection>(
      ProjectionErrorCode::InvalidConfiguration,
      "snapshot projection configuration is invalid");
  }
  if (snapshot.header.frame_id != config.planning_frame) {
    return failure<DesiredProductProjection>(
      ProjectionErrorCode::FrameMismatch,
      "snapshot frame does not match planning frame");
  }
  if (snapshot.revision < config.last_verified_revision) {
    return failure<DesiredProductProjection>(
      ProjectionErrorCode::RevisionRegression,
      "snapshot revision predates verified projection");
  }

  DesiredProductProjection desired;
  desired.revision = snapshot.revision;
  const std::set<std::string> managed_attached = managed_attached_ids(current_attached_ids);
  std::optional<std::string> held_source_object_id;
  if (snapshot.robot.has_held_object) {
    const auto held =
      std::ranges::find_if(
      snapshot.objects, [&snapshot](const TrackedObjectMessage & object) {
        return object.id == snapshot.robot.held_object;
      });
    if (held != snapshot.objects.end() && !held->source_object_id.empty()) {
      held_source_object_id = held->source_object_id;
    }
  }
  std::optional<std::string> held_destination_lane_id;
  if (snapshot.has_active_reservation && snapshot.robot.has_held_object &&
    snapshot.active_reservation.stage ==
    restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED &&
    snapshot.active_reservation.object_id == snapshot.robot.held_object &&
    !snapshot.active_reservation.destination_lane_id.empty())
  {
    // Transaction authority, unlike a camera-produced source-object name. Freeze the
    // destination's aggregate volume for the whole attached stage: depth evidence can start
    // counting the carried product once the arm enters the lane.
    held_destination_lane_id = snapshot.active_reservation.destination_lane_id;
  }

  // Which products belong to a lane. Both committed membership (`contents`) and the lane's own
  // evidence (contained source IDs) count, since a lane can hold products this system never
  // placed (seeded scenarios).
  std::set<std::uint64_t> lane_owned_ids;
  std::set<std::string> lane_owned_source_ids;
  std::vector<moveit_msgs::msg::CollisionObject> lane_objects;
  for (const auto & lane : snapshot.lanes) {
    if (lane.id.empty()) {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidIdentity,
        "snapshot carries a lane without an ID");
    }
    const auto geometry = config.lane_volumes.find(lane.id);
    if (geometry == config.lane_volumes.end()) {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidConfiguration,
        "lane " + lane.id + " has no surveyed volume, so its contents cannot be projected");
    }
    lane_owned_ids.insert(lane.contents.begin(), lane.contents.end());
    lane_owned_source_ids.insert(
      lane.observed_source_object_ids.begin(),
      lane.observed_source_object_ids.end());
    double column_width = geometry->second.usable_bounds_in_lane.sizes().x();
    if (const auto lane_class = product_class(lane.expected_product_class)) {
      const std::optional<std::string> lane_sku =
        lane.has_expected_sku ? std::make_optional(lane.expected_sku) : std::nullopt;
      const auto envelope = catalog.resolve(*lane_class, lane_sku);
      if (envelope && envelope.value().primitive.dimensions.size() == 2U) {
        column_width = std::min(
          column_width,
          2.0 * envelope.value()
          .primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS]);
      }
    }
    const bool lane_may_count_held_product =
      (held_destination_lane_id && lane.id == *held_destination_lane_id) ||
      (held_source_object_id &&
      std::ranges::find(lane.observed_source_object_ids, *held_source_object_id) !=
      lane.observed_source_object_ids.end());
    if (lane_may_count_held_product) {
      // Depth sees the product as soon as it is inserted, before physical and semantic detach,
      // while MoveIt still carries it as attached. Rebuilding the box from that depth would
      // represent it twice and make the retreat start state self-collide. Retain the last exact
      // lane volume (including an empty one) until semantic detach transfers ownership.
      const auto retained = config.retained_lane_volume_objects.find(lane.id);
      if (retained == config.retained_lane_volume_objects.end()) {
        return failure<DesiredProductProjection>(
          ProjectionErrorCode::InvalidConfiguration,
          "lane " + lane.id +
          " counts the held product but has no retained verified occupied volume");
      }
      if (retained->second) {
        const std::string expected_id = std::string(kLanePrefix) + lane.id;
        if (retained->second->id != expected_id ||
          retained->second->header.frame_id != config.planning_frame)
        {
          return failure<DesiredProductProjection>(
            ProjectionErrorCode::InvalidConfiguration,
            "lane " + lane.id + " retained occupied volume has the wrong identity or frame");
        }
        lane_objects.push_back(*retained->second);
      }
    } else {
      auto lane_object =
        lane_volume_object(lane, geometry->second, column_width, config.planning_frame);
      if (!lane_object) {
        return failure<DesiredProductProjection>(
          lane_object.error().code,
          lane_object.error().detail);
      }
      if (lane_object.value()) {
        lane_objects.push_back(std::move(*lane_object.value()));
      }
    }
  }
  std::set<std::uint64_t> object_ids;
  std::set<std::string> source_ids;
  for (const auto & object : snapshot.objects) {
    if (object.id == 0 || object.source_object_id.empty() || !object_ids.insert(object.id).second ||
      !source_ids.insert(object.source_object_id).second)
    {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidIdentity,
        object_context(object.id, "identity is empty, invalid, or duplicated"));
    }
    if (object.revision == 0 || object.revision > snapshot.revision ||
      (object.has_sku && object.sku.empty()) || (!object.has_sku && !object.sku.empty()))
    {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidIdentity,
        object_context(object.id, "revision or SKU presence is inconsistent"));
    }
    if (!valid_tracking_state(object.tracking_state) || !valid_grasp_state(object.grasp_state)) {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidLifecycle,
        object_context(object.id, "lifecycle enumeration is invalid"));
    }
    if (object.grasp_state == TrackedObjectMessage::GRASP_ATTACHED &&
      object.tracking_state != TrackedObjectMessage::TRACKING_TRACKED)
    {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidLifecycle,
        object_context(object.id, "attached object must remain tracked"));
    }
    if (object.grasp_state == TrackedObjectMessage::GRASP_FREE &&
      (object.tracking_state == TrackedObjectMessage::TRACKING_LOST ||
      object.tracking_state == TrackedObjectMessage::TRACKING_REMOVED))
    {
      continue;
    }

    if (!active_object(object)) {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidLifecycle,
        object_context(object.id, "unsupported lifecycle combination"));
    }
    const bool attached = object.grasp_state == TrackedObjectMessage::GRASP_ATTACHED;
    const std::string collision_id = "restocker/object/" + std::to_string(object.id);
    // A product in a lane is part of the lane's occupied-volume box: not projected, pose not
    // read, observation not aged. A released product keeps rolling under gravity feed and must
    // not be tracked here.
    if (!attached && (lane_owned_ids.contains(object.id) ||
      lane_owned_source_ids.contains(object.source_object_id)))
    {
      desired.lane_owned_object_ids.insert(object.id);
      // A product MoveIt still holds attached must be detached to the world before the world
      // set (which no longer contains it) can delete it.
      if (managed_attached.contains(collision_id)) {
        desired.pending_detach_ids.insert(collision_id);
      }
      continue;
    }
    // A held product's pose is derived from the arm, not observed, so it has no freshness gate
    // (a camera cannot see through a closed gripper). The attachment transaction checks the
    // product is still held at grasp and release.
    Eigen::Isometry3d product_pose;
    std::optional<std::int64_t> aged_ns;
    if (attached) {
      auto held = held_product_pose(snapshot, config);
      if (!held) {
        return failure<DesiredProductProjection>(held.error().code, held.error().detail);
      }
      product_pose = held.value();
      desired.held_product = DerivedProductPose{object.id, product_pose, config.now_ns};
    } else {
      // A product outside a lane is drawn at its last observed pose however old that is. With one
      // moving camera, a tray product nobody is looking at ages as a matter of course; failing
      // the whole projection on it made every other fact unusable, and dropping it would turn
      // unobserved space into free space. Only lifecycle evidence (Removed, Lost) takes it out of
      // the scene. Validity of what the arm acts on is gated upstream (selection, reservation,
      // CONFIRM), never here.
      auto observation_time = validated_observation_time(object, config);
      if (!observation_time) {
        return failure<DesiredProductProjection>(
          observation_time.error().code,
          observation_time.error().detail);
      }
      auto pose = validated_pose(object, config.quaternion_norm_tolerance);
      if (!pose) {
        return failure<DesiredProductProjection>(pose.error().code, pose.error().detail);
      }
      product_pose = pose.value();
      if (const std::int64_t age_ns = config.now_ns - observation_time.value();
        age_ns > config.max_observation_age_ns)
      {
        aged_ns = age_ns;
      }
    }

    const auto build_collision_object =
      [&]() -> ProjectionResult<moveit_msgs::msg::CollisionObject> {
        const auto object_class = product_class(object.product_class);
        if (!object_class) {
          return failure<moveit_msgs::msg::CollisionObject>(
            ProjectionErrorCode::MissingGeometry,
            object_context(object.id, "product class has no collision geometry"));
        }
        const std::optional<std::string> sku =
          object.has_sku ? std::make_optional(object.sku) : std::nullopt;
        auto geometry = catalog.resolve(*object_class, sku);
        if (!geometry) {
          return failure<moveit_msgs::msg::CollisionObject>(
            ProjectionErrorCode::MissingGeometry,
            object_context(object.id, geometry.error().detail));
        }
        auto collision_object = make_product_collision_object(
          object.id, config.planning_frame,
          product_pose, geometry.value());
        if (!collision_object) {
          return failure<moveit_msgs::msg::CollisionObject>(
            ProjectionErrorCode::InvalidPose,
            object_context(object.id, collision_object.error().detail));
        }
        return ProjectionResult<moveit_msgs::msg::CollisionObject>::success(
          std::move(collision_object.value()));
      };

    if (attached) {
      desired.required_attached_ids.insert(collision_id);
      // Carried product geometry in the planning frame, so the obstacle projection recognises it.
      // Unstamped: there is no observation behind it.
      auto collision_object = build_collision_object();
      if (!collision_object) {
        return failure<DesiredProductProjection>(
          collision_object.error().code,
          collision_object.error().detail);
      }
      desired.attached_geometry.push_back(std::move(collision_object.value()));
      continue;
    }
    if (managed_attached.contains(collision_id)) {
      // Released in the snapshot but still attached in MoveIt: withhold it as a world object
      // (it would appear twice) until the detach completes.
      desired.pending_detach_ids.insert(collision_id);
      continue;
    }

    auto collision_object = build_collision_object();
    if (!collision_object) {
      return failure<DesiredProductProjection>(
        collision_object.error().code,
        collision_object.error().detail);
    }
    desired.world_objects.push_back(std::move(collision_object.value()));
    if (aged_ns) {
      desired.aged_object_ages_ns.emplace(object.id, *aged_ns);
    }
  }

  if (desired.required_attached_ids.size() > 1U ||
    snapshot.robot.has_held_object != !desired.required_attached_ids.empty())
  {
    return failure<DesiredProductProjection>(
      ProjectionErrorCode::InvalidLifecycle,
      "robot held-object state disagrees with semantic attachment state");
  }
  if (snapshot.robot.has_held_object) {
    const std::string held_id = "restocker/object/" + std::to_string(snapshot.robot.held_object);
    if (snapshot.robot.held_object == 0 || !desired.required_attached_ids.contains(held_id)) {
      return failure<DesiredProductProjection>(
        ProjectionErrorCode::InvalidLifecycle,
        "robot held-object identity disagrees with attached object");
    }
  }
  desired.world_objects.insert(
    desired.world_objects.end(),
    std::make_move_iterator(lane_objects.begin()),
    std::make_move_iterator(lane_objects.end()));
  // Card 050: a declared product the snapshot does not carry yet is still physically in the
  // cell — seed its geometry so no segment of a transfer can be planned through it. The moment
  // world state tracks the source id (first confirmation) the seed leaves this set and the diff
  // removes it from the scene in the same cycle the tracked object arrives, so the two never
  // coexist and a lane-owned product is already covered by its lane's volume above.
  for (const auto & seed : config.declared_product_seeds) {
    if (source_ids.contains(seed.source_object_id)) {
      continue;
    }
    desired.world_objects.push_back(seed.collision_object);
  }
  std::ranges::sort(
    desired.world_objects, {},
    [](const moveit_msgs::msg::CollisionObject & object) {return object.id;});
  std::ranges::sort(
    desired.attached_geometry, {},
    [](const moveit_msgs::msg::CollisionObject & object) {return object.id;});
  return ProjectionResult<DesiredProductProjection>::success(std::move(desired));
}

std::string to_string(ProjectionErrorCode code)
{
  switch (code) {
    case ProjectionErrorCode::InvalidConfiguration:
      return "invalid_configuration";
    case ProjectionErrorCode::FrameMismatch:
      return "frame_mismatch";
    case ProjectionErrorCode::RevisionRegression:
      return "revision_regression";
    case ProjectionErrorCode::InvalidIdentity:
      return "invalid_identity";
    case ProjectionErrorCode::InvalidLifecycle:
      return "invalid_lifecycle";
    case ProjectionErrorCode::InvalidPose:
      return "invalid_pose";
    case ProjectionErrorCode::FutureObservation:
      return "future_observation";
    case ProjectionErrorCode::MissingGeometry:
      return "missing_geometry";
    case ProjectionErrorCode::AttachmentMismatch:
      return "attachment_mismatch";
    case ProjectionErrorCode::InvalidScene:
      return "invalid_scene";
    case ProjectionErrorCode::VerificationMismatch:
      return "verification_mismatch";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
