// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/allowed_collision_entry.hpp>
#include <moveit_msgs/msg/link_padding.hpp>
#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/tracked_object.hpp>

#include "restocker_task_executor/obstacle_projection.hpp"
#include "restocker_task_executor/planning_scene_attachment.hpp"
#include "restocker_task_executor/planning_scene_lease.hpp"
#include "restocker_task_executor/pregrasp_planning_authority.hpp"
#include "restocker_task_executor/world_snapshot_projection.hpp"

namespace restocker_task_executor
{
namespace
{

using SceneStatus = restocker_interfaces::msg::PlanningSceneProjectionStatus;
using TrackedObject = restocker_interfaces::msg::TrackedObject;

// Exercise the production projection, reconciliation, and generation protocol together. The
// service transport is deterministic: a submitted diff lands before read-back verification.
class TrajectorySceneValidity : public testing::Test
{
protected:
  TrajectorySceneValidity()
  : protocol_(PlanningSceneLeaseConfig{4096U, "projector:test", 0U}, []() {
        return std::optional<std::string>{"0123456789abcdef0123456789abcdef"};
      })
  {
    auto loaded = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
    if (!loaded) {
      throw std::runtime_error(loaded.error().detail);
    }
    catalog_ = std::move(loaded.value());
    snapshot_.header.frame_id = "world";
    snapshot_.header.stamp.sec = 10;
    snapshot_.revision = 10U;
    TrackedObject product;
    product.id = 7U;
    product.source_object_id = "sim-product-7";
    product.product_class = TrackedObject::PRODUCT_CLASS_CAN;
    product.has_sku = true;
    product.sku = "SIM-CAN-STD";
    product.pose.pose.position.z = 0.8;
    product.pose.pose.orientation.w = 1.0;
    product.orientation = TrackedObject::ORIENTATION_UPRIGHT;
    product.tracking_state = TrackedObject::TRACKING_TRACKED;
    product.grasp_state = TrackedObject::GRASP_FREE;
    product.observation_time.sec = 10;
    product.transition_time.sec = 10;
    product.revision = 10U;
    snapshot_.objects.push_back(product);
    projection_config_.planning_frame = "world";
    projection_config_.now_ns = 10'000'000'000;
    projection_config_.planning_from_grasp_center = Eigen::Isometry3d::Identity();
    projection_config_.planning_from_grasp_center->translation().z() = 0.8;
  }

  DesiredProductProjection project()
  {
    std::set<std::string> attached_ids;
    for (const auto & attached : scene_.robot_state.attached_collision_objects) {
      attached_ids.insert(attached.object.id);
    }
    auto projected = project_world_snapshot(
      snapshot_, projection_config_, *catalog_, attached_ids);
    if (!projected) {
      throw std::runtime_error(projected.error().detail);
    }
    return std::move(projected.value());
  }

  void observe_carry_and_obstacle(const DesiredProductProjection & products, double obstacle_x)
  {
    restocker_interfaces::msg::ObstacleObservation observation;
    observation.header.frame_id = "world";
    observation.header.stamp.sec = 10;
    observation.sensor_frame = "overhead_camera_optical_frame";
    observation.sequence = snapshot_.revision;
    observation.robot_static = true;
    restocker_interfaces::msg::ObstacleBox box;
    box.center.x = obstacle_x;
    box.center.y = -0.7;
    box.center.z = 0.8;
    box.size.x = box.size.y = box.size.z = 0.10;
    box.point_count = 250U;
    observation.boxes.push_back(box);
    box.center = products.attached_geometry.front().pose.position;
    box.size.x = box.size.y = box.size.z = 0.05;
    observation.boxes.push_back(box);
    const auto projected = project_obstacle_observation(
      observation, products.world_objects, products.attached_geometry, {});
    ASSERT_TRUE(projected) << projected.error().detail;
    // The changing carried world pose masks only its own cluster. The real obstacle survives.
    ASSERT_EQ(projected.value().size(), 1U);
    if (!obstacle_sets_match(obstacles_, projected.value(), {})) {
      obstacles_ = projected.value();
    }
  }

  void certify(const DesiredProductProjection & products)
  {
    auto desired = products.world_objects;
    desired.insert(desired.end(), obstacles_.begin(), obstacles_.end());
    if (!verify_planning_scene(desired, products.required_attached_ids, scene_)) {
      const auto diff = build_planning_scene_diff(desired, scene_);
      ASSERT_TRUE(diff) << diff.error().detail;
      protocol_.record_scene_diff_submission();
      // All world objects in this fixture are managed; this is the result of applying the diff.
      scene_.world.collision_objects = desired;
    }
    ASSERT_TRUE(verify_planning_scene(desired, products.required_attached_ids, scene_));
    ASSERT_NE(
      protocol_.record_verification(snapshot_.revision, 10'000'000'000),
      VerificationTransition::RejectedWhileHeld);
  }

  SceneStatus status() const
  {
    const auto state = protocol_.state();
    SceneStatus value;
    value.header.frame_id = "world";
    value.header.stamp.sec = 10;
    value.projector_epoch = state.projector_epoch;
    value.state = SceneStatus::STATE_APPLIED;
    value.desired_revision = snapshot_.revision;
    value.applied_revision = state.verified_applied_revision;
    value.verification_epoch = state.verification_epoch;
    value.scene_content_generation = state.scene_content_generation;
    return value;
  }

  PlanningSceneAuthorityResult gate(
    const std::optional<PreGraspSceneAuthority> & baseline = std::nullopt) const
  {
    return evaluate_planning_scene_authority(
      status(), baseline ? baseline->applied_revision : snapshot_.revision,
      rclcpp::Time(10, 0, RCL_ROS_TIME), {}, baseline);
  }

  void attach()
  {
    auto & product = snapshot_.objects.front();
    product.grasp_state = TrackedObject::GRASP_ATTACHED;
    snapshot_.robot.has_held_object = true;
    snapshot_.robot.held_object = product.id;
    snapshot_.robot.grasp_center_from_held_object.orientation.w = 1.0;
    snapshot_.robot.grasp_center_from_held_object.position.z = -0.03;
    ++snapshot_.revision;
    const auto products = project();
    auto carried = products.attached_geometry.front();
    // The projector's settle allowance shortens the cylinder before attachment. It is fixed
    // collision geometry throughout carry, not an exemption for every update to this object ID.
    carried.primitives.front().dimensions.front() -= 0.002;
    const auto attached = build_world_to_attached_diff(
      carried, "gripper", {"gripper", "left_finger", "right_finger"},
      snapshot_.robot.grasp_center_from_held_object);
    ASSERT_TRUE(attached) << attached.error().detail;
    protocol_.record_scene_diff_submission();
    scene_.world.collision_objects.clear();
    scene_.robot_state.attached_collision_objects = {attached.value().expected_attached_object};
    // MoveIt can recompose the reported attachment pose. Existing transition semantics check
    // shape/link/touch links, not this residual; carry must not rewrite it every verification.
    scene_.robot_state.attached_collision_objects.front().object.pose.position.x += 1.0e-4;
    ASSERT_TRUE(
      verify_world_to_attached_transition(
        attached.value().expected_attached_object,
        scene_));
    certify(products);
  }

  PlanningSceneLeaseProtocol protocol_;
  std::optional<ProductCollisionCatalog> catalog_;
  restocker_interfaces::msg::WorldStateSnapshot snapshot_;
  SnapshotProjectionConfig projection_config_;
  moveit_msgs::msg::PlanningScene scene_;
  std::vector<moveit_msgs::msg::CollisionObject> obstacles_;
};

TEST_F(TrajectorySceneValidity, AppliedObstacleChangeInvalidatesFirstAndRemainingSlices)
{
  certify(project());
  const auto before_planning = gate();
  ASSERT_TRUE(before_planning);

  // An external obstacle appears while planning. It is already verified/APPLIED before execute,
  // so a state-only gate (the old motion-port configuration) would incorrectly permit this plan.
  auto obstacle = scene_.world.collision_objects.front();
  obstacle.id = "restocker/obstacle/1";
  obstacle.pose.position.x = 0.25;
  obstacles_.push_back(obstacle);
  certify(project());
  ASSERT_EQ(status().state, SceneStatus::STATE_APPLIED);
  const auto initial_execute = gate(before_planning.authority);
  EXPECT_EQ(initial_execute.error, PreGraspAuthorityErrorCode::kSceneContentChanged);
  EXPECT_FALSE(initial_execute);

  const auto new_plan = gate();
  ASSERT_TRUE(new_plan);
  // Two unchanged controller boundaries remain valid. Do not replace the plan's baseline with
  // the last accepted status: it belongs to the whole divided trajectory.
  for (int completed_slices = 0; completed_slices < 2; ++completed_slices) {
    ++snapshot_.revision;
    certify(project());
    ASSERT_TRUE(gate(new_plan.authority));
  }
  obstacles_.front().pose.position.x += 0.10;
  certify(project());
  const auto next_slice = gate(new_plan.authority);
  EXPECT_EQ(next_slice.error, PreGraspAuthorityErrorCode::kSceneContentChanged);
  EXPECT_FALSE(next_slice);
}

TEST_F(TrajectorySceneValidity, CarryProgressPreservesAttachedGeometryAndPlanAcrossSlices)
{
  certify(project());
  const auto before_attach = gate();
  ASSERT_TRUE(before_attach);
  attach();
  EXPECT_EQ(gate(before_attach.authority).error, PreGraspAuthorityErrorCode::kSceneContentChanged);
  observe_carry_and_obstacle(project(), 1.5);
  certify(project());
  const auto carry_plan = gate();
  ASSERT_TRUE(carry_plan);
  const auto attached = scene_.robot_state.attached_collision_objects;
  const auto generation = status().scene_content_generation;
  const auto verification = status().verification_epoch;
  const auto initial_world_pose = project().attached_geometry.front().pose;

  for (int completed_slices = 0; completed_slices < 4; ++completed_slices) {
    ++snapshot_.revision;
    snapshot_.robot.revision = snapshot_.revision;
    projection_config_.planning_from_grasp_center->translation().x() += 0.10;
    scene_.robot_state.joint_state.name = {"shoulder_pan_joint"};
    scene_.robot_state.joint_state.position = {0.1 * completed_slices};
    const auto products = project();
    EXPECT_NE(products.attached_geometry.front().pose, initial_world_pose);
    EXPECT_TRUE(products.world_objects.empty());
    observe_carry_and_obstacle(products, 1.5);
    certify(products);
    EXPECT_EQ(scene_.robot_state.attached_collision_objects, attached);
    EXPECT_EQ(status().scene_content_generation, generation);
    EXPECT_GT(status().verification_epoch, verification);
    ASSERT_TRUE(gate(carry_plan.authority));
  }

  // A real collision-geometry mutation remains invalidating even while carrying.
  observe_carry_and_obstacle(project(), 1.7);
  certify(project());
  EXPECT_EQ(gate(carry_plan.authority).error, PreGraspAuthorityErrorCode::kSceneContentChanged);
}

TEST_F(TrajectorySceneValidity, DetachRequiresANewSegmentBaseline)
{
  certify(project());
  attach();
  const auto carry_plan = gate();
  ASSERT_TRUE(carry_plan);
  const auto released = snapshot_.objects.front().pose.pose;
  const auto detached = build_attached_to_world_diff(
    scene_.robot_state.attached_collision_objects.front(), "world", released);
  ASSERT_TRUE(detached);
  protocol_.record_scene_diff_submission();
  scene_.robot_state.attached_collision_objects.clear();
  scene_.world.collision_objects = {detached.value().expected_world_object};
  ASSERT_TRUE(verify_attached_to_world_transition(detached.value().expected_world_object, scene_));
  snapshot_.objects.front().grasp_state = TrackedObject::GRASP_FREE;
  snapshot_.robot.has_held_object = false;
  snapshot_.robot.held_object = 0U;
  ++snapshot_.revision;
  // Reconciliation restores the free product's catalog dimensions after the settle allowance.
  certify(project());
  EXPECT_EQ(gate(carry_plan.authority).error, PreGraspAuthorityErrorCode::kSceneContentChanged);
  EXPECT_TRUE(gate());
}

TEST_F(TrajectorySceneValidity, LeaseReleaseFencesCollisionExemptionsEvenWithoutAWorldDiff)
{
  certify(project());
  const auto before_transaction = gate();
  ASSERT_TRUE(before_transaction);
  const auto acquired = protocol_.acquire({"acquire", snapshot_.revision}, ProjectorStage::Idle, 0);
  ASSERT_EQ(acquired.code, PlanningSceneLeaseCode::Granted);
  const auto validated = protocol_.validate(acquired.token);
  ASSERT_TRUE(validated.lease);
  const PlanningSceneLeaseAuthority lease{
    validated.lease->lease_id, validated.lease->granted_applied_revision,
    validated.lease->verification_epoch};
  auto held = status();
  held.state = SceneStatus::STATE_TRANSACTION_HELD;
  held.lease_id = lease.lease_id;
  held.lease_phase = restocker_interfaces::msg::PlanningSceneLease::PHASE_HELD;
  EXPECT_EQ(
    evaluate_planning_scene_authority(
      held, snapshot_.revision,
      rclcpp::Time(10, 0, RCL_ROS_TIME), {}, before_transaction.authority).error,
    PreGraspAuthorityErrorCode::kSceneLeaseActive);
  const auto leased_plan = evaluate_planning_scene_authority(
    held, snapshot_.revision, rclcpp::Time(10, 0, RCL_ROS_TIME), {}, std::nullopt, lease);
  ASSERT_TRUE(leased_plan);

  // Scene transactions can change ACM/padding outside the projector's ordinary world diff.
  // Completing release advances the generation unconditionally, including when no world changed.
  scene_.allowed_collision_matrix.entry_names = {"left_finger", "restocker/object/7"};
  moveit_msgs::msg::AllowedCollisionEntry row;
  row.enabled = {true, true};
  scene_.allowed_collision_matrix.entry_values = {row, row};
  moveit_msgs::msg::LinkPadding padding;
  padding.link_name = "left_finger";
  padding.padding = 0.0015;
  scene_.link_padding.push_back(padding);
  ASSERT_EQ(
    protocol_.release({"release", acquired.token, snapshot_.revision}).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  certify(project());
  EXPECT_EQ(
    gate(before_transaction.authority).error,
    PreGraspAuthorityErrorCode::kSceneContentChanged);
  EXPECT_FALSE(gate(leased_plan.authority));
  EXPECT_TRUE(gate());
}

}  // namespace
}  // namespace restocker_task_executor
