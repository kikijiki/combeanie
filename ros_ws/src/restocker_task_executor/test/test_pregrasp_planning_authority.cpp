// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>

#include "restocker_task_executor/pregrasp_planning_authority.hpp"

namespace
{

using namespace std::chrono_literals;
using restocker_interfaces::msg::PlanningSceneLease;
using restocker_interfaces::msg::PlanningSceneProjectionStatus;
using restocker_interfaces::msg::SimulationAttachmentState;
using restocker_task_executor::PreGraspAuthorityDecision;
using restocker_task_executor::PreGraspAuthorityErrorCode;
using restocker_task_executor::PreGraspPlanningAuthority;
using restocker_task_executor::PreGraspPlanningAuthorityConfig;
using restocker_task_executor::PlanningSceneLeaseAuthority;

constexpr std::uint64_t kRequiredRevision = 30U;

[[nodiscard]] rclcpp::Time time_at(std::chrono::nanoseconds value)
{
  return rclcpp::Time(value.count(), RCL_ROS_TIME);
}

[[nodiscard]] PlanningSceneProjectionStatus valid_scene()
{
  PlanningSceneProjectionStatus scene;
  scene.header.stamp = time_at(1900ms);
  scene.header.frame_id = "world";
  scene.projector_epoch = "projector:alpha";
  scene.state = PlanningSceneProjectionStatus::STATE_APPLIED;
  scene.desired_revision = kRequiredRevision;
  scene.applied_revision = kRequiredRevision;
  scene.verification_epoch = 4U;
  scene.scene_content_generation = 7U;
  scene.lease_id = 0U;
  scene.lease_phase = PlanningSceneLease::PHASE_NONE;
  scene.error_code = PlanningSceneProjectionStatus::ERROR_NONE;
  return scene;
}

[[nodiscard]] SimulationAttachmentState valid_attachment()
{
  SimulationAttachmentState attachment;
  attachment.simulator_epoch = "simulator:alpha";
  attachment.sequence = 11U;
  attachment.phase = SimulationAttachmentState::PHASE_DETACHED;
  attachment.joint_observed = false;
  attachment.motion_gate = SimulationAttachmentState::MOTION_GATE_VALID;
  attachment.observed_at = time_at(1900ms);
  return attachment;
}

[[nodiscard]] auto evaluate(
  const PlanningSceneProjectionStatus & scene,
  const SimulationAttachmentState & attachment,
  const std::optional<PreGraspPlanningAuthority> & baseline = std::nullopt,
  const PreGraspPlanningAuthorityConfig & config = {})
{
  return restocker_task_executor::evaluate_pregrasp_planning_authority(
    scene, attachment, kRequiredRevision, time_at(2s), config, baseline);
}

TEST(PreGraspPlanningAuthority, AcceptsAndNormalizesFreshDetachedAuthority)
{
  const auto result = evaluate(valid_scene(), valid_attachment());

  ASSERT_TRUE(result);
  EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kAccepted);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kNone);
  ASSERT_TRUE(result.authority);
  EXPECT_EQ(result.authority->scene.planning_frame, "world");
  EXPECT_EQ(result.authority->scene.projector_epoch, "projector:alpha");
  EXPECT_EQ(result.authority->scene.applied_revision, kRequiredRevision);
  EXPECT_EQ(result.authority->scene.verification_epoch, 4U);
  EXPECT_EQ(result.authority->scene.scene_content_generation, 7U);
  EXPECT_EQ(result.authority->scene.observed_at, time_at(1900ms));
  EXPECT_EQ(result.authority->attachment.simulator_epoch, "simulator:alpha");
  EXPECT_EQ(result.authority->attachment.sequence, 11U);
  EXPECT_EQ(result.authority->attachment.observed_at, time_at(1900ms));
}

TEST(PreGraspPlanningAuthority, SynchronizingTowardRequiredRevisionWaits)
{
  auto scene = valid_scene();
  scene.state = PlanningSceneProjectionStatus::STATE_SYNCHRONIZING;
  scene.applied_revision = kRequiredRevision - 1U;

  const auto result = evaluate(scene, valid_attachment());

  EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kWait);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneSynchronizing);
  EXPECT_FALSE(result.authority);
}

TEST(PreGraspPlanningAuthority, StaleEvidenceWaitsButFutureEvidenceRejects)
{
  auto scene = valid_scene();
  scene.header.stamp = time_at(1400ms);
  auto result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kWait);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kStaleEvidence);

  auto attachment = valid_attachment();
  attachment.observed_at = time_at(1400ms);
  result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kWait);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kStaleEvidence);

  scene = valid_scene();
  scene.header.stamp = time_at(2051ms);
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kRejected);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kFutureEvidence);
}

TEST(PreGraspPlanningAuthority, RejectsInvalidConfigurationAndTimestamps)
{
  auto config = PreGraspPlanningAuthorityConfig{};
  config.planning_frame.clear();
  auto result = evaluate(valid_scene(), valid_attachment(), std::nullopt, config);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kInvalidConfiguration);

  config = {};
  config.maximum_scene_age = -1ns;
  result = evaluate(valid_scene(), valid_attachment(), std::nullopt, config);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kInvalidConfiguration);

  auto scene = valid_scene();
  scene.header.stamp = time_at(0ns);
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kInvalidTimestamp);

  auto attachment = valid_attachment();
  attachment.observed_at = time_at(0ns);
  result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kInvalidTimestamp);
}

TEST(PreGraspPlanningAuthority, RejectsInvalidSceneIdentityAndProjectorState)
{
  auto scene = valid_scene();
  scene.header.frame_id = "map";
  auto result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  scene = valid_scene();
  scene.projector_epoch.clear();
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  scene = valid_scene();
  scene.scene_content_generation = 0U;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  scene = valid_scene();
  scene.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  scene = valid_scene();
  scene.error_code = PlanningSceneProjectionStatus::ERROR_VERIFICATION_FAILED;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);
}

TEST(PreGraspPlanningAuthority, RejectsActiveLeaseAndInvalidRevisionEvidence)
{
  auto scene = valid_scene();
  scene.lease_id = 9U;
  scene.lease_phase = PlanningSceneLease::PHASE_HELD;
  auto result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneLeaseActive);

  scene = valid_scene();
  scene.desired_revision = kRequiredRevision - 1U;
  scene.applied_revision = kRequiredRevision - 1U;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneRevisionInvalid);

  scene = valid_scene();
  scene.applied_revision = kRequiredRevision - 1U;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneRevisionInvalid);

  scene = valid_scene();
  scene.desired_revision = kRequiredRevision;
  scene.applied_revision = kRequiredRevision + 1U;
  result = evaluate(scene, valid_attachment());
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneRevisionInvalid);
}

TEST(PreGraspPlanningAuthority, BaselinePermitsNewVerificationOfUnchangedContent)
{
  const auto initial = evaluate(valid_scene(), valid_attachment());
  ASSERT_TRUE(initial);

  auto scene = valid_scene();
  scene.desired_revision += 1U;
  scene.applied_revision += 1U;
  scene.verification_epoch += 1U;
  scene.header.stamp = time_at(1950ms);
  auto attachment = valid_attachment();
  attachment.observed_at = time_at(1950ms);

  const auto result = evaluate(scene, attachment, initial.authority);

  ASSERT_TRUE(result);
  EXPECT_EQ(result.authority->scene.applied_revision, kRequiredRevision + 1U);
  EXPECT_EQ(result.authority->scene.verification_epoch, 5U);
}

TEST(PreGraspPlanningAuthority, BaselineRejectsSceneLineageChangesAndRegression)
{
  const auto initial = evaluate(valid_scene(), valid_attachment());
  ASSERT_TRUE(initial);

  auto scene = valid_scene();
  scene.projector_epoch = "projector:beta";
  auto result = evaluate(scene, valid_attachment(), initial.authority);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kProjectorEpochChanged);

  scene = valid_scene();
  scene.scene_content_generation += 1U;
  result = evaluate(scene, valid_attachment(), initial.authority);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneContentChanged);

  scene = valid_scene();
  scene.applied_revision -= 1U;
  scene.desired_revision = kRequiredRevision;
  result = evaluate(scene, valid_attachment(), initial.authority);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneRevisionInvalid);
}

TEST(PreGraspPlanningAuthority, RejectsInvalidAttachmentBoundary)
{
  auto attachment = valid_attachment();
  attachment.phase = SimulationAttachmentState::PHASE_ATTACHED;
  auto result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentStateInvalid);

  attachment = valid_attachment();
  attachment.motion_gate = SimulationAttachmentState::MOTION_GATE_INHIBITED;
  result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentStateInvalid);

  attachment = valid_attachment();
  attachment.joint_observed = true;
  result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentStateInvalid);

  attachment = valid_attachment();
  attachment.sequence = 0U;
  result = evaluate(valid_scene(), attachment);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentStateInvalid);
}

TEST(PreGraspPlanningAuthority, BaselineRejectsAttachmentLineageChanges)
{
  const auto initial = evaluate(valid_scene(), valid_attachment());
  ASSERT_TRUE(initial);

  auto attachment = valid_attachment();
  attachment.simulator_epoch = "simulator:beta";
  auto result = evaluate(valid_scene(), attachment, initial.authority);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentEpochChanged);

  attachment = valid_attachment();
  attachment.sequence += 1U;
  result = evaluate(valid_scene(), attachment, initial.authority);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kAttachmentSequenceChanged);
}

TEST(PreGraspPlanningAuthority, StringRepresentationsAreStable)
{
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityDecision::kAccepted), "accepted");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityDecision::kWait), "wait");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityDecision::kRejected), "rejected");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityErrorCode::kNone), "none");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityErrorCode::kSceneSynchronizing),
    "scene_synchronizing");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityErrorCode::kSceneProjectorSettling),
    "scene_projector_settling");
  EXPECT_STREQ(
    restocker_task_executor::to_string(PreGraspAuthorityErrorCode::kAttachmentSequenceChanged),
    "attachment_sequence_changed");
}

// The scene half on its own. It is what gates every motion segment, including the segments of a
// transfer, where the attachment half above is false by construction: the gripper is holding
// something.
[[nodiscard]] auto evaluate_scene(
  const PlanningSceneProjectionStatus & scene,
  const std::optional<restocker_task_executor::PreGraspSceneAuthority> & baseline = std::nullopt,
  const PreGraspPlanningAuthorityConfig & config = {})
{
  return restocker_task_executor::evaluate_planning_scene_authority(
    scene, kRequiredRevision, time_at(2s), config, baseline);
}

TEST(PlanningSceneAuthority, AcceptsACertifiedSceneWithoutConsultingAnyAttachment)
{
  const auto result = evaluate_scene(valid_scene());

  ASSERT_TRUE(result);
  EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kNone);
  ASSERT_TRUE(result.authority);
  EXPECT_EQ(result.authority->projector_epoch, "projector:alpha");
  EXPECT_EQ(result.authority->scene_content_generation, 7U);
}

TEST(PlanningSceneAuthority, RejectsAProjectionThatCannotBeCertified)
{
  auto degraded = valid_scene();
  degraded.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
  degraded.error_code = PlanningSceneProjectionStatus::ERROR_VERIFICATION_FAILED;
  const auto unverified = evaluate_scene(degraded);
  // The projector cannot certify the scene, so nothing may be executed against it. Verification
  // failing is the projector's verdict about the world rather than a report that it has fallen
  // behind, so it is answered once and for all.
  ASSERT_FALSE(unverified);
  EXPECT_EQ(unverified.decision, PreGraspAuthorityDecision::kRejected);
  EXPECT_EQ(unverified.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  auto leased = valid_scene();
  leased.lease_id = 42U;
  leased.lease_phase = PlanningSceneLease::PHASE_HELD;
  const auto held = evaluate_scene(leased);
  ASSERT_FALSE(held);
  EXPECT_EQ(held.error, PreGraspAuthorityErrorCode::kSceneLeaseActive);
}

// Regression for the live post-Insert failure: physical detach deliberately holds the projector
// lease longer than maximum_scene_age while retreat is planned. The current token validation is
// the freshness proof for that frozen projection. Without it the same status stays fail-closed.
TEST(PlanningSceneAuthority, ValidatedHeldLeaseAuthorizesItsFrozenProjectionOnly)
{
  auto held = valid_scene();
  held.header.stamp = time_at(1s);
  held.state = PlanningSceneProjectionStatus::STATE_TRANSACTION_HELD;
  held.lease_id = 42U;
  held.lease_phase = PlanningSceneLease::PHASE_HELD;
  const PlanningSceneLeaseAuthority capability{
    held.lease_id, held.applied_revision, held.verification_epoch};

  const auto established = restocker_task_executor::evaluate_planning_scene_authority(
    held, kRequiredRevision, time_at(10s), PreGraspPlanningAuthorityConfig{}, std::nullopt,
    capability);
  ASSERT_TRUE(established);
  ASSERT_TRUE(established.authority);
  EXPECT_EQ(established.authority->lease_id, held.lease_id);

  // The execution-side check accepts the same still-held capability and baseline.
  EXPECT_TRUE(
    restocker_task_executor::evaluate_planning_scene_authority(
      held, kRequiredRevision, time_at(11s), PreGraspPlanningAuthorityConfig{},
      established.authority, capability));

  // Merely observing an active lease is never authority, even if every public field looks right.
  const auto without_capability = restocker_task_executor::evaluate_planning_scene_authority(
    held, kRequiredRevision, time_at(10s), PreGraspPlanningAuthorityConfig{});
  EXPECT_FALSE(without_capability);

  auto wrong_capability = capability;
  ++wrong_capability.lease_id;
  const auto mismatched = restocker_task_executor::evaluate_planning_scene_authority(
    held, kRequiredRevision, time_at(10s), PreGraspPlanningAuthorityConfig{}, std::nullopt,
    wrong_capability);
  EXPECT_FALSE(mismatched);
  EXPECT_EQ(
    mismatched.error, PreGraspAuthorityErrorCode::kSceneLeaseCapabilityInvalid);
}

// Every condition below is the projector declining to certify the scene, and none of them lets
// anything execute: a wait is not an acceptance, and `operator bool` stays false throughout. What
// separates them is whether the next reconcile cycle can clear the condition with nobody
// intervening. Where it can, answering "rejected" on first sight turns a gap into a fault, because
// a refusal while a product is held latches the coordinator into `motion inhibited; operator
// required` and ends the run.
TEST(PlanningSceneAuthority, TransientProjectorConditionsWaitRatherThanReject)
{
  const std::array<std::uint16_t, 5> settling{
    PlanningSceneProjectionStatus::ERROR_WORLD_STATE_UNAVAILABLE,
    PlanningSceneProjectionStatus::ERROR_PLANNING_SCENE_UNAVAILABLE,
    PlanningSceneProjectionStatus::ERROR_TRANSFORM_UNAVAILABLE,
    PlanningSceneProjectionStatus::ERROR_OBSTACLE_EVIDENCE_STALE,
    PlanningSceneProjectionStatus::ERROR_OBSERVATION_EVIDENCE_STALE};

  for (const auto error_code : settling) {
    auto scene = valid_scene();
    scene.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
    scene.error_code = error_code;
    const auto result = evaluate_scene(scene);
    EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kWait) << "error code " << error_code;
    EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneProjectorSettling)
      << "error code " << error_code;
    // Still not permission to move.
    EXPECT_FALSE(result) << "error code " << error_code;
    EXPECT_FALSE(result.authority.has_value()) << "error code " << error_code;
  }

  // A projector that has not run its first cycle has legitimately not populated its identity or
  // revision fields, so it must not be refused for lacking them.
  auto starting = valid_scene();
  starting.state = PlanningSceneProjectionStatus::STATE_STARTING;
  starting.projector_epoch.clear();
  starting.verification_epoch = 0U;
  starting.scene_content_generation = 0U;
  starting.desired_revision = 0U;
  starting.applied_revision = 0U;
  const auto booting = evaluate_scene(starting);
  EXPECT_EQ(booting.decision, PreGraspAuthorityDecision::kWait);
  EXPECT_EQ(booting.error, PreGraspAuthorityErrorCode::kSceneProjectorSettling);
  EXPECT_FALSE(booting);
}

// The other half of the taxonomy, which must not soften. These are the projector
// reporting a verdict about the world rather than reporting that it has fallen behind, and
// waiting on any of them only delays the same answer.
TEST(PlanningSceneAuthority, TerminalProjectorConditionsStillRejectOnFirstSight)
{
  const std::array<std::uint16_t, 5> failed{
    PlanningSceneProjectionStatus::ERROR_CONFIGURATION,
    PlanningSceneProjectionStatus::ERROR_INVALID_SNAPSHOT,
    PlanningSceneProjectionStatus::ERROR_APPLY_REJECTED,
    PlanningSceneProjectionStatus::ERROR_VERIFICATION_FAILED,
    PlanningSceneProjectionStatus::ERROR_APPLY_OUTCOME_UNKNOWN};

  for (const auto error_code : failed) {
    auto scene = valid_scene();
    scene.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
    scene.error_code = error_code;
    const auto result = evaluate_scene(scene);
    EXPECT_EQ(result.decision, PreGraspAuthorityDecision::kRejected) << "error code " << error_code;
    EXPECT_EQ(result.error, PreGraspAuthorityErrorCode::kSceneStateInvalid)
      << "error code " << error_code;
    EXPECT_FALSE(result) << "error code " << error_code;
  }

  // An unrecognised error code is a projector this build does not understand, which is not a
  // condition anything may wait out.
  auto unknown = valid_scene();
  unknown.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
  unknown.error_code = 4242U;
  const auto refused = evaluate_scene(unknown);
  EXPECT_EQ(refused.decision, PreGraspAuthorityDecision::kRejected);
  EXPECT_EQ(refused.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);

  // A degraded projector that reports no error at all is still not an applied scene.
  auto silent = valid_scene();
  silent.state = PlanningSceneProjectionStatus::STATE_DEGRADED;
  const auto quiet = evaluate_scene(silent);
  EXPECT_EQ(quiet.decision, PreGraspAuthorityDecision::kRejected);
  EXPECT_EQ(quiet.error, PreGraspAuthorityErrorCode::kSceneStateInvalid);
}

TEST(PlanningSceneAuthority, AppliedContentChangeCannotRenewAnOldTrajectory)
{
  const auto established = evaluate_scene(valid_scene());
  ASSERT_TRUE(established);

  auto moved = valid_scene();
  moved.scene_content_generation += 1U;
  moved.verification_epoch += 1U;
  moved.applied_revision += 1U;
  moved.desired_revision = moved.applied_revision;
  ASSERT_EQ(moved.state, PlanningSceneProjectionStatus::STATE_APPLIED);
  const auto fenced = evaluate_scene(moved, established.authority);
  ASSERT_FALSE(fenced);
  EXPECT_EQ(fenced.error, PreGraspAuthorityErrorCode::kSceneContentChanged);

  // Only a newly planned segment may establish a baseline against the changed geometry.
  EXPECT_TRUE(evaluate_scene(moved));

  auto restarted = valid_scene();
  restarted.projector_epoch = "projector:beta";
  const auto reborn = evaluate_scene(restarted, established.authority);
  ASSERT_FALSE(reborn);
  EXPECT_EQ(reborn.error, PreGraspAuthorityErrorCode::kProjectorEpochChanged);

  auto regressed = valid_scene();
  regressed.applied_revision = kRequiredRevision - 1U;
  const auto backwards = evaluate_scene(regressed, established.authority);
  ASSERT_FALSE(backwards);
  EXPECT_EQ(backwards.error, PreGraspAuthorityErrorCode::kSceneRevisionInvalid);
}

}  // namespace
