// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <restocker_interfaces/msg/obstacle_observation.hpp>

#include "restocker_task_executor/world_snapshot_projection.hpp"

namespace restocker_task_executor
{

// Prefix every depth-derived obstacle carries. It is inside the projector-managed namespace, so
// the diff the projector already builds removes an obstacle that is no longer observed exactly as
// it removes a product that is no longer stocked, and the verification it already performs proves
// the obstacle reached the scene.
inline constexpr const char * kObstacleIdPrefix = "restocker/obstacle/";

struct ObstacleProjectionConfig
{
  std::string planning_frame{"world"};
  // Grown on every face before the box enters the scene. A depth-fitted box bounds the surfaces
  // the camera could see and nothing behind them, so the padding is what covers the unobserved
  // far side and the quantisation of the fit itself.
  double obstacle_padding_m{0.03};
  // A padded obstacle smaller than this is not worth a collision object; one larger than this is
  // a surface the volume of interest should have excluded, and admitting it would wall off the
  // workspace on the strength of one bad frame.
  double min_extent_m{0.04};
  double max_extent_m{1.50};
  // Ceiling on obstacles admitted from one observation, taken in the order the observation lists
  // them. The producer already caps and orders its own boxes, so this is a second-line bound on
  // how much geometry one message can add to the scene, not a ranking.
  std::size_t max_obstacles{8U};
  // Known geometry is grown by this much before an obstacle is tested against it. The observed
  // surface of a modelled object never coincides exactly with its model.
  double known_padding_m{0.05};
  // Fraction of the obstacle's own volume that must fall inside padded known geometry before it
  // is discarded as a re-observation of something already in the scene. Containment, not mere
  // contact, is the test: discarding every obstacle that merely touches the shelf would delete
  // real geometry, and this gate must never turn "something is there" into "nothing is there".
  double known_containment_fraction{0.75};
  // How far an obstacle may move, and how much it may change size, before it counts as different
  // geometry. Below these the previously accepted geometry is kept byte-for-byte, so an
  // unchanging scene never produces a planning-scene diff and never invalidates a plan in flight.
  double change_position_tolerance_m{0.03};
  double change_size_tolerance_m{0.03};
};

[[nodiscard]] bool valid_obstacle_projection_config(
  const ObstacleProjectionConfig & config) noexcept;

// Turns one observation into the collision objects the planning scene should carry for it.
//
// `known_world_objects` and `known_attached_geometry` are the geometry this projector already
// owns, in the planning frame. An obstacle substantially contained in one of them is a
// re-observation of modelled geometry, the shelf, a stocked product, or the product the gripper
// is carrying, and is dropped. Everything else is kept, including an obstacle that merely
// touches known geometry.
//
// The caller must supply known geometry describing the same instant as the observation. For the
// carried product that means the observation's `robot_static` flag, because the attached geometry
// is derived from the current snapshot and the observation from an earlier exposure.
[[nodiscard]] ProjectionResult<std::vector<moveit_msgs::msg::CollisionObject>>
project_obstacle_observation(
  const restocker_interfaces::msg::ObstacleObservation & observation,
  const std::vector<moveit_msgs::msg::CollisionObject> & known_world_objects,
  const std::vector<moveit_msgs::msg::CollisionObject> & known_attached_geometry,
  const ObstacleProjectionConfig & config);

// True when `candidate` describes the same obstacles as `accepted` within the configured
// tolerances. A true answer means the accepted geometry may be retained unchanged, which is what
// keeps a stationary obstacle from re-announcing itself on every reconciliation cycle.
[[nodiscard]] bool obstacle_sets_match(
  const std::vector<moveit_msgs::msg::CollisionObject> & accepted,
  const std::vector<moveit_msgs::msg::CollisionObject> & candidate,
  const ObstacleProjectionConfig & config);

}  // namespace restocker_task_executor
