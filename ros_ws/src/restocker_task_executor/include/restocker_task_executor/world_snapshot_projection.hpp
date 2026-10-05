// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <restocker_interfaces/msg/world_state_snapshot.hpp>

#include "restocker_task_executor/scene_geometry.hpp"

namespace restocker_task_executor
{

enum class ProjectionErrorCode : std::uint8_t
{
  InvalidConfiguration,
  FrameMismatch,
  RevisionRegression,
  InvalidIdentity,
  InvalidLifecycle,
  InvalidPose,
  FutureObservation,
  MissingGeometry,
  AttachmentMismatch,
  InvalidScene,
  VerificationMismatch,
};

struct ProjectionError
{
  ProjectionErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] ProjectionResult
{
public:
  [[nodiscard]] static ProjectionResult success(T value)
  {
    return ProjectionResult(std::move(value));
  }

  [[nodiscard]] static ProjectionResult failure(ProjectionError error)
  {
    return ProjectionResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const ProjectionError & error() const
  {
    return std::get<ProjectionError>(storage_);
  }

private:
  explicit ProjectionResult(T value)
  : storage_(std::move(value)) {}

  explicit ProjectionResult(ProjectionError error)
  : storage_(std::move(error)) {}

  std::variant<T, ProjectionError> storage_;
};

// A product's pose derived from the arm's own kinematics rather than observed: the gripper's
// `grasp_center` frame composed with the grasp the attachment boundary verified when the jaws
// closed. It is what the projection uses for a held product, and it is the one value that can
// still describe a product in the instant after the arm has put it down and is standing over it.
struct DerivedProductPose
{
  std::uint64_t object_id{0};
  Eigen::Isometry3d planning_from_product{Eigen::Isometry3d::Identity()};
  // The instant the arm's kinematics were read to produce it. A derivation kept past the release
  // is the one value in this projection that outlives the instant it was true at, so it carries
  // that instant with it and is bounded from it.
  std::int64_t derived_at_ns{0};
};

// A shelf lane's surveyed volume, placed in the planning frame.
//
// Once a product is in a lane it stops being an object and becomes part of the lane. Its pose is
// not tracked, not projected and not required to be observed, the robot loads the lane from
// behind and the customer empties it from the front, and in between it is a column packed against
// the front rail that nothing needs to name individually. What the planner has to avoid is the
// lane's occupied volume, and the lane's own `available_depth_m` describes it exactly: one box
// from the front retainer back to `depth_m - available_depth_m`.
//
// That is what replaced the release hold. The hold existed because a released product had to keep
// a pose for the seconds before the camera could see it again, and under gravity feed it did not
// have one, it was rolling. The question is now not asked: a released product is inside the
// lane's volume from the instant the jaws open, so the box already contains it and nothing is
// waiting to observe it.
struct LaneVolumeGeometry
{
  Eigen::Isometry3d planning_from_lane{Eigen::Isometry3d::Identity()};
  Eigen::AlignedBox3d usable_bounds_in_lane;
};

// A scenario-declared product the scene must carry while world state does not yet track it
// (Card 050). The collision object is built once at startup: a managed id
// (`restocker/declared/<source_object_id>`), the catalogue cylinder at the declared spawn pose,
// in the planning frame, operation ADD — everything build_planning_scene_diff and
// verify_planning_scene already require of a desired world object. It is obstacle geometry and
// nothing else: it is never evidence, world state never ingests it, and the projection drops it
// from the desired set the cycle the snapshot first carries `source_object_id`.
struct DeclaredProductSeed
{
  std::string source_object_id;
  moveit_msgs::msg::CollisionObject collision_object;
};

struct SnapshotProjectionConfig
{
  std::string planning_frame{"world"};
  std::int64_t now_ns{0};
  // Evidence older than this marks a free product as aged (reported, still projected); it is not
  // a freshness gate. A future stamp beyond `max_future_skew_ns` still fails the projection.
  std::int64_t max_observation_age_ns{500'000'000};
  std::int64_t max_future_skew_ns{50'000'000};
  std::uint64_t last_verified_revision{0};
  double quaternion_norm_tolerance{1.0e-6};
  // Pose of the gripper's `grasp_center` frame in the planning frame, read from TF. A held
  // product's world pose is this composed with the grasp the snapshot carries, which is the only
  // place the held product's pose comes from: it is never observed. Required whenever the
  // snapshot reports a held product, and unused otherwise.
  std::optional<Eigen::Isometry3d> planning_from_grasp_center;
  // The lane volumes this projection may derive occupied-column geometry in, by lane ID: each
  // lane's frame in the planning frame and its surveyed usable bounds in that frame. Every lane
  // the snapshot carries must have an entry, because a lane whose geometry is unknown cannot have
  // its occupied volume drawn and the products inside it are no longer drawn individually.
  std::map<std::string, LaneVolumeGeometry> lane_volumes;
  // The last verified occupied-volume object for each lane in the current MoveIt scene. An empty
  // optional means that lane was verified empty. When the still-held product is already visible
  // inside a lane, its aggregate depth has begun counting that same attached object; the previous
  // volume is retained until semantic detach so the scene never represents one product twice.
  std::map<std::string, std::optional<moveit_msgs::msg::CollisionObject>>
  retained_lane_volume_objects;
  // Scenario-declared products not yet tracked (Card 050): each contributes its collision object
  // to the desired world set for as long as the snapshot carries no object with the same source
  // id. Empty in deployments without a scenario document.
  std::vector<DeclaredProductSeed> declared_product_seeds;
};

struct DesiredProductProjection
{
  std::uint64_t revision{0};
  // Products the scene should carry as ordinary world geometry. A product that is attached in the
  // snapshot is absent here, and so is one the snapshot has released but MoveIt still holds
  // attached: re-adding that one to the world before the detach transition ran would leave the
  // same product in the scene twice.
  std::vector<moveit_msgs::msg::CollisionObject> world_objects;
  std::set<std::string> required_attached_ids;
  // Planning-frame collision geometry for exactly the products in `required_attached_ids`. Its
  // pose is the gripper's forward kinematics composed with the committed grasp, never an
  // observation: a camera cannot see into a closed gripper, and the arm's own state already says
  // where the product is. Consumed by the obstacle projection, which must not re-add the carried
  // product as an unmodelled box; the world-to-attached move states its own parent-link pose from
  // the same grasp, so nothing here is paired with a robot pose read at another instant.
  std::vector<moveit_msgs::msg::CollisionObject> attached_geometry;
  // Products MoveIt still reports attached although the snapshot has released them. Each needs an
  // attached-to-world transition before the world projection above can be reached.
  std::set<std::string> pending_detach_ids;
  // This cycle's kinematic derivation for the held product, empty when nothing is held. The
  // caller keeps it across the release for one purpose only: the attached-to-world transition
  // MoveIt requires has to name a pose, and the pose the arm let go at is the one pose that is
  // not an observation of a product that has already rolled away from it. The object is deleted
  // from the scene on the very next cycle, because by then it belongs to its lane's volume.
  std::optional<DerivedProductPose> held_product;
  // Products this projection left to their lanes: they are inside a lane's occupied volume, so
  // they carry no geometry of their own and no freshness gate was applied to them. Reported so a
  // caller can say which products the scene is describing as part of a lane rather than as
  // objects.
  std::set<std::uint64_t> lane_owned_object_ids;
  // Free products outside every lane whose evidence is older than `max_observation_age_ns`, by
  // ID, with that age. They are in `world_objects` at their last known pose like any other
  // product: an aged observation says "I have not looked", not "nothing is there", and age alone
  // never frees space (Milestone 10 §6, Card 071). Reported so the caller can name them.
  std::map<std::uint64_t, std::int64_t> aged_object_ages_ns;
};

[[nodiscard]] ProjectionResult<DesiredProductProjection> project_world_snapshot(
  const restocker_interfaces::msg::WorldStateSnapshot & snapshot,
  const SnapshotProjectionConfig & config, const ProductCollisionCatalog & catalog,
  const std::set<std::string> & current_attached_ids);

[[nodiscard]] std::string to_string(ProjectionErrorCode code);

}  // namespace restocker_task_executor
