// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <limits>
#include <stdexcept>

#include "restocker_gazebo/attachment_verifier.hpp"

namespace restocker_gazebo
{
namespace
{

AttachmentVerificationConfig config()
{
  return AttachmentVerificationConfig{3, 8, 0.001, 0.01, 0.02, 0.05};
}

AttachmentVerificationObservation observation(std::uint64_t iteration)
{
  AttachmentVerificationObservation result;
  result.exact_joint_observed = true;
  result.entity_identity_matches = true;
  result.simulator_iteration = iteration;
  result.simulation_time_ns = static_cast<std::int64_t>(iteration) * 1'000'000;
  return result;
}

TEST(AttachmentVerifier, RequiresConsecutiveLaterTicksForAttachSuccess)
{
  AttachmentMutationVerifier verifier(config());
  verifier.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto same_tick = observation(10);
  same_tick.simulation_time_ns = 10;
  EXPECT_EQ(
    verifier.observe(same_tick).disposition,
    AttachmentVerificationDisposition::kPending);
  EXPECT_EQ(
    verifier.observe(observation(11)).disposition,
    AttachmentVerificationDisposition::kPending);
  EXPECT_EQ(
    verifier.observe(observation(12)).disposition,
    AttachmentVerificationDisposition::kPending);
  const auto verified = verifier.observe(observation(13));
  EXPECT_EQ(verified.disposition, AttachmentVerificationDisposition::kSucceeded);
  EXPECT_EQ(verified.status.code, AttachmentStatusCode::kAttached);
  ASSERT_TRUE(verified.evidence);
  EXPECT_TRUE(verified.evidence->joint_observed);
  EXPECT_EQ(verified.evidence->simulator_iteration, 13U);
}

TEST(AttachmentVerifier, VelocityResetsConsecutiveAttachEvidence)
{
  AttachmentMutationVerifier verifier(config());
  verifier.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  EXPECT_EQ(
    verifier.observe(observation(11)).disposition,
    AttachmentVerificationDisposition::kPending);
  auto moving = observation(12);
  moving.relative_linear_velocity_in_parent.x() = 0.021;
  EXPECT_EQ(verifier.observe(moving).disposition, AttachmentVerificationDisposition::kPending);
  EXPECT_EQ(
    verifier.observe(observation(13)).disposition,
    AttachmentVerificationDisposition::kPending);
  EXPECT_EQ(
    verifier.observe(observation(14)).disposition,
    AttachmentVerificationDisposition::kPending);
  EXPECT_EQ(
    verifier.observe(observation(15)).disposition,
    AttachmentVerificationDisposition::kSucceeded);
}

TEST(AttachmentVerifier, MissingCreatedJointAndPoseDriftFailClosed)
{
  AttachmentMutationVerifier missing(config());
  missing.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto no_joint = observation(11);
  no_joint.exact_joint_observed = false;
  EXPECT_EQ(
    missing.observe(no_joint).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier drift(config());
  drift.begin("attach-18", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto displaced = observation(11);
  displaced.parent_from_child.translation().x() = 0.0011;
  EXPECT_EQ(
    drift.observe(displaced).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, RejectsIterationAndTimeRegression)
{
  AttachmentMutationVerifier verifier(config());
  verifier.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 100);
  EXPECT_EQ(
    verifier.observe(observation(9)).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier time(config());
  time.begin("attach-18", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 100);
  auto regressed = observation(11);
  regressed.simulation_time_ns = 99;
  EXPECT_EQ(
    time.observe(regressed).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, DetachWaitsForAbsenceAndThenConfirmsAcrossTicks)
{
  AttachmentMutationVerifier verifier(config());
  verifier.begin("detach-17", AttachmentCommand::kDetach, Eigen::Isometry3d::Identity(), 20, 20);
  EXPECT_EQ(
    verifier.observe(observation(21)).disposition,
    AttachmentVerificationDisposition::kPending);
  for (std::uint64_t iteration : {22U, 23U}) {
    auto removed = observation(iteration);
    removed.exact_joint_observed = false;
    EXPECT_EQ(
      verifier.observe(removed).disposition,
      AttachmentVerificationDisposition::kPending);
  }
  auto removed = observation(24);
  removed.exact_joint_observed = false;
  // Once removed, the dynamic child may move relative to the gripper; detach verification only
  // requires finite final evidence and persistent absence of the exact owned joint.
  removed.parent_from_child.translation().z() = -0.02;
  const auto verified = verifier.observe(removed);
  EXPECT_EQ(verified.disposition, AttachmentVerificationDisposition::kSucceeded);
  EXPECT_EQ(verified.status.code, AttachmentStatusCode::kDetached);
  ASSERT_TRUE(verified.evidence);
  EXPECT_FALSE(verified.evidence->joint_observed);
}

TEST(AttachmentVerifier, BoundedAttachAndDetachSettlingBecomeUnknown)
{
  AttachmentMutationVerifier attach(config());
  attach.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto moving = observation(18);
  moving.relative_angular_velocity_in_parent.z() = 0.051;
  EXPECT_EQ(
    attach.observe(moving).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier detach(config());
  detach.begin("detach-17", AttachmentCommand::kDetach, Eigen::Isometry3d::Identity(), 10, 10);
  EXPECT_EQ(
    detach.observe(observation(18)).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, WatchdogRejectsMissingIdentityMotionAndNonFiniteEvidence)
{
  auto valid = observation(30);
  EXPECT_EQ(
    validate_attached_watchdog(Eigen::Isometry3d::Identity(), config(), valid).code,
    AttachmentStatusCode::kAttached);

  auto missing = valid;
  missing.exact_joint_observed = false;
  EXPECT_EQ(
    validate_attached_watchdog(Eigen::Isometry3d::Identity(), config(), missing).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto wrong_identity = valid;
  wrong_identity.entity_identity_matches = false;
  EXPECT_EQ(
    validate_attached_watchdog(Eigen::Isometry3d::Identity(), config(), wrong_identity).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto moving = valid;
  moving.relative_angular_velocity_in_parent.z() = 0.051;
  EXPECT_EQ(
    validate_attached_watchdog(Eigen::Isometry3d::Identity(), config(), moving).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto invalid = valid;
  invalid.world_from_child.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_attached_watchdog(Eigen::Isometry3d::Identity(), config(), invalid).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, ConstructorAndBeginRejectInvalidLifecycle)
{
  auto invalid = config();
  invalid.required_consecutive_ticks = 0;
  EXPECT_THROW(static_cast<void>(AttachmentMutationVerifier{invalid}), std::invalid_argument);

  AttachmentMutationVerifier verifier(config());
  EXPECT_THROW(
    verifier.begin("", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 1, 1),
    std::invalid_argument);
  verifier.begin("attach", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 1, 1);
  EXPECT_THROW(
    verifier.begin("other", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 1, 1),
    std::logic_error);
  verifier.reset();
  EXPECT_FALSE(verifier.active());
}

}  // namespace
}  // namespace restocker_gazebo
