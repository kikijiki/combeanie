// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

#include <map>
#include <set>
#include <stdexcept>
#include <string>

#include <cstdint>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <restocker_interfaces/msg/shelf_lane.hpp>
#include <restocker_interfaces/msg/tracked_object.hpp>
#include <restocker_interfaces/msg/world_state_snapshot.hpp>

#include "restocker_task_executor/planning_scene_reconciliation.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_task_executor/world_snapshot_projection.hpp"

namespace restocker_task_executor
{
namespace
{

using TrackedObject = restocker_interfaces::msg::TrackedObject;
using WorldStateSnapshot = restocker_interfaces::msg::WorldStateSnapshot;

TrackedObject tracked_object(std::uint64_t id)
{
  TrackedObject object;
  object.id = id;
  object.source_object_id = "sim-product-" + std::to_string(id);
  object.product_class = TrackedObject::PRODUCT_CLASS_CAN;
  object.has_sku = true;
  object.sku = "SIM-CAN-STD";
  object.pose.pose.position.x = 0.1 * static_cast<double>(id);
  object.pose.pose.position.y = -0.3;
  object.pose.pose.position.z = 0.8;
  object.pose.pose.orientation.w = 1.0;
  object.orientation = TrackedObject::ORIENTATION_UPRIGHT;
  object.tracking_state = TrackedObject::TRACKING_TRACKED;
  object.grasp_state = TrackedObject::GRASP_FREE;
  object.observation_time.sec = 10;
  object.transition_time.sec = 10;
  object.revision = 1;
  return object;
}

WorldStateSnapshot snapshot_with(std::initializer_list<TrackedObject> objects)
{
  WorldStateSnapshot snapshot;
  snapshot.header.frame_id = "world";
  snapshot.header.stamp.sec = 10;
  snapshot.revision = 10;
  snapshot.objects.assign(objects);
  return snapshot;
}

// Grasp datum pose from TF. A held product's pose is composed from this and the snapshot's grasp.
Eigen::Isometry3d planning_from_grasp_center()
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(0.4, -0.25, 1.1);
  value.linear() = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return value;
}

Eigen::Isometry3d grasp_center_from_product()
{
  Eigen::Isometry3d value = Eigen::Isometry3d::Identity();
  value.translation() = Eigen::Vector3d(0.002, 0.0, -0.03);
  return value;
}

SnapshotProjectionConfig config()
{
  SnapshotProjectionConfig value;
  value.planning_frame = "world";
  value.now_ns = 10'000'000'000;
  value.max_observation_age_ns = 500'000'000;
  value.max_future_skew_ns = 50'000'000;
  value.last_verified_revision = 9;
  value.planning_from_grasp_center = planning_from_grasp_center();
  return value;
}

// Snapshot while the arm carries object 7: attached, with the grasp the boundary verified.
WorldStateSnapshot held_product_snapshot()
{
  auto attached = tracked_object(7);
  attached.grasp_state = TrackedObject::GRASP_ATTACHED;
  auto snapshot = snapshot_with({attached});
  snapshot.robot.has_held_object = true;
  snapshot.robot.held_object = 7;
  const Eigen::Isometry3d grasp = grasp_center_from_product();
  snapshot.robot.grasp_center_from_held_object.position.x = grasp.translation().x();
  snapshot.robot.grasp_center_from_held_object.position.y = grasp.translation().y();
  snapshot.robot.grasp_center_from_held_object.position.z = grasp.translation().z();
  const Eigen::Quaterniond rotation(grasp.linear());
  snapshot.robot.grasp_center_from_held_object.orientation.x = rotation.x();
  snapshot.robot.grasp_center_from_held_object.orientation.y = rotation.y();
  snapshot.robot.grasp_center_from_held_object.orientation.z = rotation.z();
  snapshot.robot.grasp_center_from_held_object.orientation.w = rotation.w();
  return snapshot;
}

ProductCollisionCatalog catalog()
{
  auto result = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
  if (!result) {
    throw std::runtime_error(result.error().detail);
  }
  return std::move(result.value());
}

TEST(WorldSnapshotProjection, BuildsDeterministicallySortedFreeProductGeometry)
{
  auto second = tracked_object(2);
  second.product_class = TrackedObject::PRODUCT_CLASS_SMALL_BOTTLE;
  second.has_sku = false;
  second.sku.clear();
  auto result =
    project_world_snapshot(snapshot_with({second, tracked_object(1)}), config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().revision, 10U);
  ASSERT_EQ(result.value().world_objects.size(), 2U);
  EXPECT_EQ(result.value().world_objects[0].id, "restocker/object/1");
  EXPECT_EQ(result.value().world_objects[1].id, "restocker/object/2");
  EXPECT_DOUBLE_EQ(result.value().world_objects[0].primitives[0].dimensions[0], 0.122);
  EXPECT_DOUBLE_EQ(result.value().world_objects[1].primitives[0].dimensions[0], 0.2);
  EXPECT_TRUE(result.value().required_attached_ids.empty());
}

// Card 050: a scenario-declared product seeds the scene until world state tracks it, and the
// seed leaves the desired set the cycle tracking arrives (spec section 6).
[[nodiscard]] DeclaredProductSeed declared_seed(
  const ProductCollisionCatalog & catalog, const std::string & source_object_id)
{
  DeclaredProductSeed seed;
  seed.source_object_id = source_object_id;
  const auto & geometry = catalog.entries().at("bottle.small.standard");
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-0.46, -0.74, 0.67);
  auto object = make_declared_product_collision_object(source_object_id, "world", pose, geometry);
  if (!object) {
    throw std::runtime_error(object.error().detail);
  }
  seed.collision_object = std::move(object.value());
  return seed;
}

[[nodiscard]] bool contains_id(
  const std::vector<moveit_msgs::msg::CollisionObject> & objects, const std::string & id)
{
  return std::ranges::any_of(
    objects, [&id](const moveit_msgs::msg::CollisionObject & object) {
      return object.id == id;
    });
}

TEST(WorldSnapshotProjection, SeedsADeclaredProductUntilTheSnapshotTracksIt)
{
  auto with_seed = config();
  with_seed.declared_product_seeds = {declared_seed(catalog(), "sim:declared-bottle")};

  // The snapshot has never carried the declared product: its geometry is in the desired set
  // beside the tracked object's, at the declared spawn pose.
  auto untracked = project_world_snapshot(
    snapshot_with({tracked_object(1)}), with_seed, catalog(), {});
  ASSERT_TRUE(untracked) << untracked.error().detail;
  EXPECT_TRUE(
    contains_id(untracked.value().world_objects, "restocker/declared/sim:declared-bottle"));
  EXPECT_TRUE(contains_id(untracked.value().world_objects, "restocker/object/1"));
  const auto & seeded = std::ranges::find_if(
    untracked.value().world_objects, [](const moveit_msgs::msg::CollisionObject & object) {
      return object.id == "restocker/declared/sim:declared-bottle";
    });
  ASSERT_NE(seeded, untracked.value().world_objects.end());
  EXPECT_NEAR(seeded->pose.position.x, -0.46, 1.0e-12);
  EXPECT_NEAR(seeded->pose.position.y, -0.74, 1.0e-12);

  // World state now tracks that source id: the seed is gone and the tracked object carries the
  // geometry, so the two never coexist in one desired set.
  auto tracked = tracked_object(2);
  tracked.source_object_id = "sim:declared-bottle";
  auto superseded = project_world_snapshot(snapshot_with({tracked}), with_seed, catalog(), {});
  ASSERT_TRUE(superseded) << superseded.error().detail;
  EXPECT_FALSE(
    contains_id(superseded.value().world_objects, "restocker/declared/sim:declared-bottle"));
  EXPECT_TRUE(contains_id(superseded.value().world_objects, "restocker/object/2"));
}

TEST(WorldSnapshotProjection, OmitsLostAndRemovedObjectsWithoutRequiringFreshPoses)
{
  auto lost = tracked_object(1);
  lost.tracking_state = TrackedObject::TRACKING_LOST;
  lost.observation_time.sec = 1;
  lost.pose.pose.orientation.w = 0.0;
  auto removed = tracked_object(2);
  removed.tracking_state = TrackedObject::TRACKING_REMOVED;
  removed.observation_time.sec = 1;
  auto result = project_world_snapshot(snapshot_with({lost, removed}), config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(result.value().world_objects.empty());
  // Lifecycle evidence, not age, is what takes a product out of the scene (Card 071).
  EXPECT_TRUE(result.value().aged_object_ages_ns.empty());
}

TEST(WorldSnapshotProjection, RejectsFrameAndRevisionRegression)
{
  auto wrong_frame = snapshot_with({tracked_object(1)});
  wrong_frame.header.frame_id = "map";
  auto frame_result = project_world_snapshot(wrong_frame, config(), catalog(), {});
  ASSERT_FALSE(frame_result);
  EXPECT_EQ(frame_result.error().code, ProjectionErrorCode::FrameMismatch);

  auto old_revision = snapshot_with({tracked_object(1)});
  old_revision.revision = 8;
  old_revision.objects[0].revision = 8;
  auto revision_result = project_world_snapshot(old_revision, config(), catalog(), {});
  ASSERT_FALSE(revision_result);
  EXPECT_EQ(revision_result.error().code, ProjectionErrorCode::RevisionRegression);
}

// Card 071 (Milestone 10 §6): an aged product says "not looked at lately", not "gone". It is
// projected exactly as a fresh one would be, at its last known pose with the catalogue geometry,
// and reported as aged; age never fails the projection and never frees its space.
TEST(WorldSnapshotProjection, ProjectsAnAgedProductAtItsLastKnownPoseAndReportsIt)
{
  const auto fresh = project_world_snapshot(
    snapshot_with({tracked_object(1)}), config(), catalog(), {});
  ASSERT_TRUE(fresh) << fresh.error().detail;
  EXPECT_TRUE(fresh.value().aged_object_ages_ns.empty());

  // Card 066 dev run 1's shape: observed at the start of the run, more than 600 s ago, against the
  // shipped 600 s horizon.
  auto long_run = config();
  long_run.now_ns = 700'000'000'000;
  long_run.max_observation_age_ns = 600'000'000'000;
  auto aged = project_world_snapshot(
    snapshot_with({tracked_object(1)}), long_run, catalog(), {});
  ASSERT_TRUE(aged) << aged.error().detail;
  ASSERT_EQ(aged.value().world_objects.size(), 1U);
  EXPECT_EQ(aged.value().world_objects, fresh.value().world_objects);
  EXPECT_EQ(
    aged.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{1U, 690'000'000'000}}));

  // Exactly at the horizon is not aged; one nanosecond past it is.
  auto at_horizon = tracked_object(1);
  at_horizon.observation_time.sec = 9;
  at_horizon.observation_time.nanosec = 500'000'000;
  const auto boundary =
    project_world_snapshot(snapshot_with({at_horizon}), config(), catalog(), {});
  ASSERT_TRUE(boundary) << boundary.error().detail;
  EXPECT_TRUE(boundary.value().aged_object_ages_ns.empty());
  at_horizon.observation_time.nanosec = 499'999'999;
  const auto past = project_world_snapshot(snapshot_with({at_horizon}), config(), catalog(), {});
  ASSERT_TRUE(past) << past.error().detail;
  EXPECT_EQ(past.value().world_objects.size(), 1U);
  EXPECT_EQ(
    past.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{1U, 500'000'001}}));
}

// One aged product among fresh ones leaves every product in the scene, and only it is named.
TEST(WorldSnapshotProjection, OneAgedProductDoesNotTakeTheRestOfTheSceneWithIt)
{
  auto aged = tracked_object(2);
  aged.observation_time.sec = 1;
  const auto result = project_world_snapshot(
    snapshot_with({tracked_object(1), aged, tracked_object(3)}), config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 3U);
  EXPECT_EQ(result.value().world_objects[1].id, "restocker/object/2");
  EXPECT_NEAR(result.value().world_objects[1].pose.position.x, 0.2, 1.0e-12);
  EXPECT_EQ(
    result.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{2U, 9'000'000'000}}));
}

// An occluded product is active like a tracked one: aged, it is still drawn and reported.
TEST(WorldSnapshotProjection, AnAgedOccludedProductIsStillAnObstacle)
{
  auto occluded = tracked_object(4);
  occluded.tracking_state = TrackedObject::TRACKING_OCCLUDED;
  occluded.observation_time.sec = 1;
  const auto result = project_world_snapshot(snapshot_with({occluded}), config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(contains_id(result.value().world_objects, "restocker/object/4"));
  EXPECT_EQ(
    result.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{4U, 9'000'000'000}}));
}

// A product withheld for its attached-to-world transition is not drawn this cycle, so it is not
// reported as aged either.
TEST(WorldSnapshotProjection, AnAgedProductWithheldForDetachIsNotReportedAged)
{
  auto aged = tracked_object(1);
  aged.observation_time.sec = 1;
  const auto result = project_world_snapshot(
    snapshot_with({aged}), config(), catalog(), {"restocker/object/1"});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(result.value().world_objects.empty());
  EXPECT_EQ(result.value().pending_detach_ids, std::set<std::string>{"restocker/object/1"});
  EXPECT_TRUE(result.value().aged_object_ages_ns.empty());
}

TEST(WorldSnapshotProjection, StillRejectsAFutureStampedObservation)
{
  auto future = tracked_object(1);
  future.observation_time.sec = 10;
  future.observation_time.nanosec = 50'000'001;
  auto future_result = project_world_snapshot(snapshot_with({future}), config(), catalog(), {});
  ASSERT_FALSE(future_result);
  EXPECT_EQ(future_result.error().code, ProjectionErrorCode::FutureObservation);
}

TEST(WorldSnapshotProjection, RejectsDuplicateIdentityAndInconsistentSkuPresence)
{
  auto duplicate = tracked_object(2);
  duplicate.source_object_id = tracked_object(1).source_object_id;
  auto duplicate_result = project_world_snapshot(
    snapshot_with({tracked_object(1), duplicate}),
    config(), catalog(), {});
  ASSERT_FALSE(duplicate_result);
  EXPECT_EQ(duplicate_result.error().code, ProjectionErrorCode::InvalidIdentity);

  auto invalid_sku = tracked_object(1);
  invalid_sku.has_sku = false;
  auto sku_result = project_world_snapshot(snapshot_with({invalid_sku}), config(), catalog(), {});
  ASSERT_FALSE(sku_result);
  EXPECT_EQ(sku_result.error().code, ProjectionErrorCode::InvalidIdentity);
}

TEST(WorldSnapshotProjection, RejectsMalformedPoseAndUnresolvedGeometryAtomically)
{
  auto malformed = tracked_object(2);
  malformed.pose.pose.orientation.w = 2.0;
  auto pose_result = project_world_snapshot(
    snapshot_with({tracked_object(1), malformed}), config(),
    catalog(), {});
  ASSERT_FALSE(pose_result);
  EXPECT_EQ(pose_result.error().code, ProjectionErrorCode::InvalidPose);

  auto unknown = tracked_object(2);
  unknown.product_class = TrackedObject::PRODUCT_CLASS_UNKNOWN;
  auto geometry_result =
    project_world_snapshot(snapshot_with({tracked_object(1), unknown}), config(), catalog(), {});
  ASSERT_FALSE(geometry_result);
  EXPECT_EQ(geometry_result.error().code, ProjectionErrorCode::MissingGeometry);
}

TEST(WorldSnapshotProjection, RequiresConsistentSemanticAndMoveItAttachment)
{
  auto snapshot = held_product_snapshot();

  // Held in the snapshot but not yet attached in MoveIt is a pending transition, not an error.
  auto missing = project_world_snapshot(snapshot, config(), catalog(), {});
  ASSERT_TRUE(missing) << missing.error().detail;
  EXPECT_TRUE(missing.value().world_objects.empty());
  EXPECT_TRUE(missing.value().pending_detach_ids.empty());
  EXPECT_EQ(missing.value().required_attached_ids, std::set<std::string>{"restocker/object/7"});
  ASSERT_EQ(missing.value().attached_geometry.size(), 1U);
  EXPECT_EQ(missing.value().attached_geometry.front().id, "restocker/object/7");

  auto valid =
    project_world_snapshot(snapshot, config(), catalog(), {"foreign/tool", "restocker/object/7"});
  ASSERT_TRUE(valid) << valid.error().detail;
  EXPECT_TRUE(valid.value().world_objects.empty());
  EXPECT_EQ(valid.value().required_attached_ids, std::set<std::string>{"restocker/object/7"});

  snapshot.robot.held_object = 8;
  auto wrong_held = project_world_snapshot(snapshot, config(), catalog(), {"restocker/object/7"});
  ASSERT_FALSE(wrong_held);
  EXPECT_EQ(wrong_held.error().code, ProjectionErrorCode::InvalidLifecycle);
}

TEST(WorldSnapshotProjection, WithholdsAProductMoveItStillHoldsAttached)
{
  // Attached in MoveIt but released in the snapshot: withheld from the world set until the
  // attached-to-world transition runs, or the product would appear twice.
  auto result = project_world_snapshot(
    snapshot_with({tracked_object(1)}), config(), catalog(),
    {"restocker/object/1"});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(result.value().world_objects.empty());
  EXPECT_TRUE(result.value().required_attached_ids.empty());
  EXPECT_EQ(result.value().pending_detach_ids, std::set<std::string>{"restocker/object/1"});
  EXPECT_EQ(to_string(ProjectionErrorCode::AttachmentMismatch), "attachment_mismatch");
}

TEST(WorldSnapshotProjection, IgnoresAnAttachmentOutsideProjectorOwnership)
{
  auto result = project_world_snapshot(
    snapshot_with({tracked_object(1)}), config(), catalog(),
    {"foreign/tool"});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  EXPECT_TRUE(result.value().pending_detach_ids.empty());
}

TEST(WorldSnapshotProjection, DerivesAHeldProductPoseFromKinematicsRatherThanObservation)
{
  const auto result = project_world_snapshot(held_product_snapshot(), config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().attached_geometry.size(), 1U);
  const Eigen::Isometry3d expected = planning_from_grasp_center() * grasp_center_from_product();
  const auto & projected = result.value().attached_geometry.front().pose;
  EXPECT_NEAR(projected.position.x, expected.translation().x(), 1.0e-12);
  EXPECT_NEAR(projected.position.y, expected.translation().y(), 1.0e-12);
  EXPECT_NEAR(projected.position.z, expected.translation().z(), 1.0e-12);
  // The tracked object's observed pose (0.7, -0.3, 0.8) must not appear in the result.
  EXPECT_NE(projected.position.x, 0.7);

  // The carried product has no observation time.
  EXPECT_EQ(result.value().attached_geometry.front().header.stamp.sec, 0);
  EXPECT_EQ(result.value().attached_geometry.front().header.stamp.nanosec, 0U);
}

TEST(WorldSnapshotProjection, ProjectsAHeldProductWithoutAnyFreshObservationOfIt)
{
  // A closed gripper hides the product, so its observation goes stale or stops. Neither ends the
  // projection: the pose follows from arm state and the tracker's last pose is not read.
  auto stale = held_product_snapshot();
  stale.objects.front().observation_time.sec = 4;
  stale.objects.front().pose.pose.position.x = 1.0e9;
  stale.objects.front().pose.pose.orientation.w = 0.0;
  const auto result = project_world_snapshot(stale, config(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().attached_geometry.size(), 1U);
  const Eigen::Isometry3d expected = planning_from_grasp_center() * grasp_center_from_product();
  EXPECT_NEAR(
    result.value().attached_geometry.front().pose.position.x, expected.translation().x(),
    1.0e-12);

  // A free product in the same snapshot is drawn from its last observation, and is the only one
  // reported as aged: the held product's age is not read (Card 071).
  auto free_product = tracked_object(3);
  free_product.observation_time.sec = 4;
  stale.objects.push_back(free_product);
  const auto mixed = project_world_snapshot(stale, config(), catalog(), {});
  ASSERT_TRUE(mixed) << mixed.error().detail;
  EXPECT_TRUE(contains_id(mixed.value().world_objects, "restocker/object/3"));
  EXPECT_EQ(
    mixed.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{3U, 6'000'000'000}}));
}

// Snapshot just after the arm released object 7: free again, `transition_time` at the release,
// last observation older than the descent that put the gripper between it and the camera. It is
// inside its destination lane and the lane evidence names it for the whole roll to the front rail.
WorldStateSnapshot released_product_snapshot()
{
  auto released = tracked_object(7);
  released.observation_time.sec = 9;
  released.transition_time.sec = 10;
  auto snapshot = snapshot_with({released});
  snapshot.revision = 11;
  return snapshot;
}

// Surveyed destination lane: shipped lane cross-section and depth, frame in the planning frame.
LaneVolumeGeometry lane_geometry()
{
  Eigen::Isometry3d planning_from_lane = Eigen::Isometry3d::Identity();
  planning_from_lane.translation() = Eigen::Vector3d(-1.0, 0.55, 0.75);
  return LaneVolumeGeometry{planning_from_lane,
    Eigen::AlignedBox3d(
      Eigen::Vector3d(-0.1825, 0.01, 0.0),
      Eigen::Vector3d(0.1825, 0.86, 0.40))};
}

restocker_interfaces::msg::ShelfLane lane_state(
  double available_depth_m,
  std::vector<std::string> observed,
  std::vector<std::uint64_t> contents = {})
{
  restocker_interfaces::msg::ShelfLane lane;
  lane.id = "lane_01";
  // A can lane, like the shipped lane_01. The class sets the column width.
  lane.expected_product_class = restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_CAN;
  lane.has_expected_sku = true;
  lane.expected_sku = "SIM-CAN-STD";
  lane.depth_m = 0.85;
  lane.available_depth_m = available_depth_m;
  lane.observed_source_object_ids = std::move(observed);
  lane.contents = std::move(contents);
  lane.evidence_revision = 5;
  lane.revision = 5;
  return lane;
}

SnapshotProjectionConfig config_with_lane()
{
  auto value = config();
  value.lane_volumes.emplace("lane_01", lane_geometry());
  return value;
}

// Post-insert retreat: the lane sensor sees the inserted can before semantic detach while MoveIt
// still carries it attached, so the new lane box would overlap it at waypoint zero unless the last
// verified volume is retained for this transaction window.
TEST(WorldSnapshotProjection, HeldProductVisibleInLaneDoesNotGrowItsOccupiedVolumeTwice)
{
  auto prior_snapshot = released_product_snapshot();
  prior_snapshot.lanes.push_back(lane_state(0.734, {}, {7}));
  const auto prior = project_world_snapshot(prior_snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(prior) << prior.error().detail;
  ASSERT_EQ(prior.value().world_objects.size(), 1U);

  auto held = held_product_snapshot();
  held.has_active_reservation = true;
  held.active_reservation.object_id = 7;
  held.active_reservation.destination_lane_id = "lane_01";
  held.active_reservation.stage = restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED;
  // Near the rear entrance the held can makes the raw occupied box almost the full lane depth.
  // Source identity is omitted: the sensor does not attribute the blob.
  held.lanes.push_back(lane_state(0.045, {}, {7}));
  auto projection_config = config_with_lane();
  projection_config.retained_lane_volume_objects.emplace(
    "lane_01",
    prior.value().world_objects.front());
  const auto projected =
    project_world_snapshot(held, projection_config, catalog(), {"restocker/object/7"});
  ASSERT_TRUE(projected) << projected.error().detail;
  ASSERT_EQ(projected.value().world_objects.size(), 1U);
  EXPECT_EQ(projected.value().world_objects.front().id, "restocker/lane/lane_01");
  EXPECT_NEAR(
    projected.value().world_objects.front().primitives.front().dimensions[1], 0.116,
    1.0e-12);
  EXPECT_EQ(projected.value().required_attached_ids, std::set<std::string>{"restocker/object/7"});

  auto missing_prior = config_with_lane();
  const auto refused =
    project_world_snapshot(held, missing_prior, catalog(), {"restocker/object/7"});
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, ProjectionErrorCode::InvalidConfiguration);
}

TEST(WorldSnapshotProjection, ReportsTheHeldProductDerivationSoACallerCanKeepIt)
{
  const auto held = project_world_snapshot(held_product_snapshot(), config(), catalog(), {});
  ASSERT_TRUE(held) << held.error().detail;
  ASSERT_TRUE(held.value().held_product.has_value());
  EXPECT_EQ(held.value().held_product->object_id, 7U);
  const Eigen::Isometry3d expected = planning_from_grasp_center() * grasp_center_from_product();
  EXPECT_TRUE(held.value().held_product->planning_from_product.isApprox(expected, 1.0e-12));
  EXPECT_EQ(held.value().held_product->derived_at_ns, config().now_ns);

  // Nothing held, so no derivation is returned to carry forward.
  const auto free_only =
    project_world_snapshot(snapshot_with({tracked_object(1)}), config(), catalog(), {});
  ASSERT_TRUE(free_only) << free_only.error().detail;
  EXPECT_FALSE(free_only.value().held_product.has_value());
}

// The just-released product rolls down its lane and cannot be observed until the arm retreats.
// Nothing waits for it: the product is part of the lane and the scene carries the lane's free
// depth.
TEST(WorldSnapshotProjection, LeavesAProductInsideALaneToItsLanesVolume)
{
  // Outside any lane the aged product is its own obstacle, named as aged (Card 071).
  const auto loose = project_world_snapshot(released_product_snapshot(), config(), catalog(), {});
  ASSERT_TRUE(loose) << loose.error().detail;
  ASSERT_EQ(loose.value().world_objects.size(), 1U);
  EXPECT_EQ(loose.value().world_objects.front().id, "restocker/object/7");
  EXPECT_TRUE(loose.value().aged_object_ages_ns.contains(7U));

  auto snapshot = released_product_snapshot();
  snapshot.lanes.push_back(lane_state(0.734, {"sim-product-7"}));
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().lane_owned_object_ids, (std::set<std::uint64_t>{7}));
  // Inside the lane its age is not read at all.
  EXPECT_TRUE(result.value().aged_object_ages_ns.empty());

  // One box, no product geometry inside the lane.
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  const auto & box = result.value().world_objects.front();
  EXPECT_EQ(box.id, "restocker/lane/lane_01");
  // Must be inside the namespace the projector reconciles and verifies; outside it the diff
  // neither adds nor removes the box and no proof covers it.
  EXPECT_TRUE(is_projector_managed_id(box.id));
  ASSERT_EQ(box.primitives.size(), 1U);
  EXPECT_EQ(box.primitives.front().type, shape_msgs::msg::SolidPrimitive::BOX);
  // Depth runs from the front retainer back over the lane's full depth: 0.85 - 0.734. Width is one
  // catalogued can diameter, so the gripper that just released is not inside it. Height is the
  // lane's, since a product on the front rail reaches the top of the judged volume.
  ASSERT_EQ(box.primitives.front().dimensions.size(), 3U);
  EXPECT_NEAR(box.primitives.front().dimensions[0], 2.0 * 0.033, 1.0e-12);
  EXPECT_NEAR(box.primitives.front().dimensions[1], 0.116, 1.0e-12);
  EXPECT_NEAR(box.primitives.front().dimensions[2], 0.40, 1.0e-12);
  EXPECT_NEAR(box.pose.position.x, -1.0, 1.0e-12);
  EXPECT_NEAR(box.pose.position.y, 0.55 + 0.86 - 0.058, 1.0e-12);
  EXPECT_NEAR(box.pose.position.z, 0.75 + 0.20, 1.0e-12);
}

// From release to rail the lane contains the product and its free depth only grows. Every
// intermediate reading is a shorter column than the settled one, so the box is never smaller than
// the truth on the retreat side and no instant of the roll fails the projection.
TEST(WorldSnapshotProjection, ARollingProductNeverStalesTheProjection)
{
  for (const double available : {0.045, 0.385, 0.734}) {
    auto snapshot = released_product_snapshot();
    snapshot.lanes.push_back(lane_state(available, {"sim-product-7"}));
    const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
    ASSERT_TRUE(result) << available << ": " << result.error().detail;
    ASSERT_EQ(result.value().world_objects.size(), 1U) << available;
    EXPECT_NEAR(
      result.value().world_objects.front().primitives.front().dimensions[1],
      0.85 - available, 1.0e-12)
      << available;
  }
}

// A lane with committed world-state membership owns its product like one whose evidence names it,
// so a product placed before the projector started still gets geometry.
TEST(WorldSnapshotProjection, CommittedLaneMembershipAlsoOwnsItsProduct)
{
  auto snapshot = released_product_snapshot();
  snapshot.lanes.push_back(lane_state(0.734, {}, {7}));
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().lane_owned_object_ids, (std::set<std::uint64_t>{7}));
}

TEST(WorldSnapshotProjection, ADenselyOccupiedLaneProjectsADeepCollisionBox)
{
  // Sparse shelves leave available_depth ≈ depth and contribute no geometry. A packed lane must
  // put a deep occupied-volume box in the planning scene so approach/transfer sees inventory.
  auto snapshot = released_product_snapshot();
  // 10 cans leave ~0.167 m free (see dense_restock_columns); occupied depth ≈ 0.683 m.
  // Name the released product so it is lane-owned rather than a stale free observation.
  snapshot.lanes.push_back(lane_state(0.1673, {"sim-product-7"}));
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  EXPECT_EQ(result.value().world_objects.front().id, "restocker/lane/lane_01");
  EXPECT_NEAR(
    result.value().world_objects.front().primitives.front().dimensions[1], 0.85 - 0.1673,
    1.0e-9);
  EXPECT_GT(result.value().world_objects.front().primitives.front().dimensions[1], 0.5);
}

// An empty lane yields no box (zero depth is not a shape).
TEST(WorldSnapshotProjection, AnEmptyLaneContributesNoGeometry)
{
  auto snapshot = snapshot_with({tracked_object(1)});
  snapshot.lanes.push_back(lane_state(0.85, {}));
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  EXPECT_EQ(result.value().world_objects.front().id, "restocker/object/1");
  EXPECT_TRUE(result.value().lane_owned_object_ids.empty());
}

TEST(WorldSnapshotProjection, WithholdsALaneOwnedProductMoveItStillHoldsAttached)
{
  // First cycle after a release: world state has let go, MoveIt has not. The product is withheld
  // from the world set so the detach can run, and reported lane-owned so the caller knows the
  // transition's pose is the release derivation, not an observation.
  auto snapshot = released_product_snapshot();
  snapshot.lanes.push_back(lane_state(0.734, {"sim-product-7"}));
  const auto result =
    project_world_snapshot(snapshot, config_with_lane(), catalog(), {"restocker/object/7"});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  EXPECT_EQ(result.value().world_objects.front().id, "restocker/lane/lane_01");
  EXPECT_EQ(result.value().pending_detach_ids, std::set<std::string>{"restocker/object/7"});
  EXPECT_EQ(result.value().lane_owned_object_ids, (std::set<std::uint64_t>{7}));
}

TEST(WorldSnapshotProjection, AnAgedProductOutsideEveryLaneStaysAnObstacleBesideTheLane)
{
  // Card 071: a silent product in the tray is still in the scene, as its own obstacle beside the
  // lane box, and only it is named as aged; the lane-owned product is not.
  auto snapshot = released_product_snapshot();
  snapshot.lanes.push_back(lane_state(0.734, {"sim-product-7"}));
  auto other = tracked_object(3);
  other.observation_time.sec = 9;
  snapshot.objects.push_back(other);
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_TRUE(contains_id(result.value().world_objects, "restocker/lane/lane_01"));
  EXPECT_TRUE(contains_id(result.value().world_objects, "restocker/object/3"));
  EXPECT_FALSE(contains_id(result.value().world_objects, "restocker/object/7"));
  EXPECT_EQ(
    result.value().aged_object_ages_ns,
    (std::map<std::uint64_t, std::int64_t>{{3U, 1'000'000'000}}));
}

// A lane whose policy the catalogue cannot name gets the conservative box: the whole lane width.
// A guessed width would leave a hole wherever it was too narrow.
TEST(WorldSnapshotProjection, AnUnpolicedLaneGetsTheWholeLaneWidth)
{
  auto snapshot = released_product_snapshot();
  auto lane = lane_state(0.734, {"sim-product-7"});
  lane.expected_product_class = restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_UNKNOWN;
  lane.has_expected_sku = false;
  lane.expected_sku.clear();
  snapshot.lanes.push_back(lane);
  const auto result = project_world_snapshot(snapshot, config_with_lane(), catalog(), {});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().world_objects.size(), 1U);
  EXPECT_NEAR(
    result.value().world_objects.front().primitives.front().dimensions[0], 0.365,
    1.0e-12);
}

TEST(WorldSnapshotProjection, RefusesALaneItHasNoSurveyFor)
{
  // Unknown lane volume: occupied depth cannot be drawn and products are no longer drawn
  // individually, so guessing would empty the scene of the lane's contents.
  auto snapshot = snapshot_with({tracked_object(1)});
  snapshot.lanes.push_back(lane_state(0.5, {}));
  const auto result = project_world_snapshot(snapshot, config(), catalog(), {});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, ProjectionErrorCode::InvalidConfiguration);

  // A published depth that disagrees with the survey is the same fault.
  auto disagreeing = snapshot;
  disagreeing.lanes.front().depth_m = 0.80;
  const auto mismatched = project_world_snapshot(disagreeing, config_with_lane(), catalog(), {});
  ASSERT_FALSE(mismatched);
  EXPECT_EQ(mismatched.error().code, ProjectionErrorCode::InvalidConfiguration);
}

TEST(WorldSnapshotProjection, RefusesAHeldProductWithoutAUsableGraspTransform)
{
  auto missing_transform = config();
  missing_transform.planning_from_grasp_center.reset();
  const auto no_tf =
    project_world_snapshot(held_product_snapshot(), missing_transform, catalog(), {});
  ASSERT_FALSE(no_tf);
  EXPECT_EQ(no_tf.error().code, ProjectionErrorCode::InvalidConfiguration);

  // The grasp alone places the carried product, so a non-rigid rotation stops the projection.
  auto skewed_grasp = held_product_snapshot();
  skewed_grasp.robot.grasp_center_from_held_object.orientation.w = 0.5;
  const auto no_grasp = project_world_snapshot(skewed_grasp, config(), catalog(), {});
  ASSERT_FALSE(no_grasp);
  EXPECT_EQ(no_grasp.error().code, ProjectionErrorCode::InvalidPose);
}

// Card 029 SC-001(e), the scene half of plan ≡ scene: snapshots the reserved-pose hold produces
// — same product pose, advanced world/object revisions and a still-fresh observation stamp —
// must project byte-identical collision geometry, so nothing rewrites restocker/object/1 under
// the frozen grasp (the 0.5 mm deadband downstream then sees no diff at all). A divergence-
// applied pose must project at the new place: the hold never hides a real movement.
TEST(WorldSnapshotProjection, HeldReservationSnapshotsProjectIdenticalGeometryUntilDivergence)
{
  const auto staged = project_world_snapshot(
    snapshot_with({tracked_object(1)}), config(), catalog(), {});
  ASSERT_TRUE(staged) << staged.error().detail;
  ASSERT_EQ(staged.value().world_objects.size(), 1U);

  auto held = snapshot_with({tracked_object(1)});
  held.revision = 14;
  held.objects[0].revision = 5;
  held.objects[0].observation_time.sec = 9;
  held.objects[0].observation_time.nanosec = 950'000'000;
  held.objects[0].transition_time = held.objects[0].observation_time;
  // pose untouched: the shape observe_object admits while the reservation holds the pose.
  auto held_config = config();
  held_config.last_verified_revision = 13;
  const auto projected = project_world_snapshot(held, held_config, catalog(), {});
  ASSERT_TRUE(projected) << projected.error().detail;
  ASSERT_EQ(projected.value().world_objects.size(), 1U);
  EXPECT_EQ(
    projected.value().world_objects[0].pose.position.x,
    staged.value().world_objects[0].pose.position.x);
  EXPECT_EQ(
    projected.value().world_objects[0].pose.position.y,
    staged.value().world_objects[0].pose.position.y);
  EXPECT_EQ(
    projected.value().world_objects[0].pose.position.z,
    staged.value().world_objects[0].pose.position.z);
  EXPECT_EQ(projected.value().revision, 14U);

  auto diverged = held;
  diverged.revision = 15;
  diverged.objects[0].revision = 6;
  diverged.objects[0].pose.pose.position.x += 0.05;
  auto diverged_config = held_config;
  diverged_config.last_verified_revision = 14;
  const auto moved = project_world_snapshot(diverged, diverged_config, catalog(), {});
  ASSERT_TRUE(moved) << moved.error().detail;
  ASSERT_EQ(moved.value().world_objects.size(), 1U);
  EXPECT_NEAR(
    moved.value().world_objects[0].pose.position.x,
    staged.value().world_objects[0].pose.position.x + 0.05, 1.0e-12);
}

}  // namespace
}  // namespace restocker_task_executor
