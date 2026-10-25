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
  AttachmentMutationVerifier verifier(config(), AttachmentHoldMode::kWeld);
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
  AttachmentMutationVerifier verifier(config(), AttachmentHoldMode::kWeld);
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
  AttachmentMutationVerifier missing(config(), AttachmentHoldMode::kWeld);
  missing.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto no_joint = observation(11);
  no_joint.exact_joint_observed = false;
  EXPECT_EQ(
    missing.observe(no_joint).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier drift(config(), AttachmentHoldMode::kWeld);
  drift.begin("attach-18", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto displaced = observation(11);
  displaced.parent_from_child.translation().x() = 0.0011;
  EXPECT_EQ(
    drift.observe(displaced).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, RejectsIterationAndTimeRegression)
{
  AttachmentMutationVerifier verifier(config(), AttachmentHoldMode::kWeld);
  verifier.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 100);
  EXPECT_EQ(
    verifier.observe(observation(9)).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier time(config(), AttachmentHoldMode::kWeld);
  time.begin("attach-18", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 100);
  auto regressed = observation(11);
  regressed.simulation_time_ns = 99;
  EXPECT_EQ(
    time.observe(regressed).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, DetachWaitsForAbsenceAndThenConfirmsAcrossTicks)
{
  AttachmentMutationVerifier verifier(config(), AttachmentHoldMode::kWeld);
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
  AttachmentMutationVerifier attach(config(), AttachmentHoldMode::kWeld);
  attach.begin("attach-17", AttachmentCommand::kAttach, Eigen::Isometry3d::Identity(), 10, 10);
  auto moving = observation(18);
  moving.relative_angular_velocity_in_parent.z() = 0.051;
  EXPECT_EQ(
    attach.observe(moving).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);

  AttachmentMutationVerifier detach(config(), AttachmentHoldMode::kWeld);
  detach.begin("detach-17", AttachmentCommand::kDetach, Eigen::Isometry3d::Identity(), 10, 10);
  EXPECT_EQ(
    detach.observe(observation(18)).disposition,
    AttachmentVerificationDisposition::kOutcomeUnknown);
}

TEST(AttachmentVerifier, WatchdogRejectsMissingIdentityMotionAndNonFiniteEvidence)
{
  auto valid = observation(30);
  EXPECT_EQ(
    validate_attached_watchdog(
      Eigen::Isometry3d::Identity(), config(), valid,
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kAttached);

  auto missing = valid;
  missing.exact_joint_observed = false;
  EXPECT_EQ(
    validate_attached_watchdog(
      Eigen::Isometry3d::Identity(), config(), missing,
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto wrong_identity = valid;
  wrong_identity.entity_identity_matches = false;
  EXPECT_EQ(
    validate_attached_watchdog(
      Eigen::Isometry3d::Identity(), config(), wrong_identity,
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto moving = valid;
  moving.relative_angular_velocity_in_parent.z() = 0.051;
  EXPECT_EQ(
    validate_attached_watchdog(
      Eigen::Isometry3d::Identity(), config(), moving,
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);

  auto invalid = valid;
  invalid.world_from_child.translation().x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_attached_watchdog(
      Eigen::Isometry3d::Identity(), config(), invalid,
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, ConstructorAndBeginRejectInvalidLifecycle)
{
  auto invalid = config();
  invalid.required_consecutive_ticks = 0;
  EXPECT_THROW(
    static_cast<void>(AttachmentMutationVerifier{invalid, AttachmentHoldMode::kWeld}),
    std::invalid_argument);

  AttachmentMutationVerifier verifier(config(), AttachmentHoldMode::kWeld);
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

// Spec M10 section 11 D5 amendment, as corrected: a side-held product, its origin 50 mm below the
// grasp centre along grasp +x, gripped by two pads on grasp y; slip is measured at each pad.
struct Pads
{
  Eigen::Vector3d left_point{0.0, 0.033, 0.14};
  Eigen::Vector3d right_point{0.0, -0.033, 0.14};
};

// A side-held upright product: its own axis (the child frame's z) runs along the pad height,
// the parent's x, and the grasp centre sits in the middle of the pads' reach.
Eigen::Isometry3d held_product()
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
  result.translation() = Eigen::Vector3d(-0.05, 0.0, 0.14);
  return result;
}

AttachmentVerificationConfig physical_config()
{
  auto result = config();
  result.maximum_swing_translation_m = 0.003;
  result.maximum_off_line_rotation_rad = 0.03;
  return result;
}

Eigen::Isometry3d finger_at(const Eigen::Vector3d & origin)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = origin;
  return result;
}

// The observation of a product now at parent_from_child, whose pads touched it at `pads` when the
// grip was verified at held_product(), with the fingers now shifted by finger_shift.
AttachmentVerificationObservation physical_observation(
  const Eigen::Isometry3d & parent_from_child, const Pads & pads = Pads{},
  const Eigen::Vector3d & finger_shift = Eigen::Vector3d::Zero())
{
  auto result = observation(40);
  result.exact_joint_observed = false;
  result.parent_from_child = parent_from_child;
  const Eigen::Isometry3d left_finger = finger_at(Eigen::Vector3d(0.0, 0.042, 0.07));
  const Eigen::Isometry3d right_finger = finger_at(Eigen::Vector3d(0.0, -0.042, 0.07));
  HeldPadObservation left;
  left.contact_in_finger = left_finger.inverse() * pads.left_point;
  left.contact_in_child = held_product().inverse() * pads.left_point;
  left.parent_from_finger = Eigen::Translation3d(finger_shift) * left_finger;
  HeldPadObservation right;
  right.contact_in_finger = right_finger.inverse() * pads.right_point;
  right.contact_in_child = held_product().inverse() * pads.right_point;
  right.parent_from_finger = Eigen::Translation3d(finger_shift) * right_finger;
  result.pads = std::array<HeldPadObservation, 2>{left, right};
  return result;
}

// The product rotated by angle_rad about the line through the two pad points.
Eigen::Isometry3d swung(double angle_rad, const Pads & pads = Pads{})
{
  const Eigen::Vector3d middle = 0.5 * (pads.left_point + pads.right_point);
  const Eigen::Vector3d axis = (pads.left_point - pads.right_point).normalized();
  return Eigen::Translation3d(middle) * Eigen::AngleAxisd(angle_rad, axis) *
         Eigen::Translation3d(-middle) * held_product();
}

AttachmentStatusCode physical_watchdog(const AttachmentVerificationObservation & observed)
{
  return validate_attached_watchdog(
    held_product(), physical_config(), observed, AttachmentHoldMode::kPhysical).code;
}

TEST(AttachmentVerifier, PhysicalWatchdogAcceptsASwingAboutTheJawAxis)
{
  // 0.04 rad moves the origin 2 mm; neither pad's material point moves.
  EXPECT_EQ(physical_watchdog(physical_observation(swung(0.04))), AttachmentStatusCode::kAttached);

  // The loaded contacts sit at different heights (4 mm over the 66 mm span, as measured), so the
  // hinge is tilted off grasp +y by ~0.06 rad. Swinging about it at 0.9 rad/s is still no slip,
  // and none of that rate is off the contact line.
  Pads tilted;
  tilted.left_point.x() = 0.002;
  tilted.right_point.x() = -0.002;
  auto moving = physical_observation(swung(0.02, tilted), tilted);
  moving.relative_angular_velocity_in_parent =
    0.9 * (tilted.left_point - tilted.right_point).normalized();
  EXPECT_EQ(physical_watchdog(moving), AttachmentStatusCode::kAttached);
}

TEST(AttachmentVerifier, PhysicalWatchdogMeasuresTiltFromTheJawAxisNotFromTheContactCentroids)
{
  // The contact centroids sit wherever the load falls among the pad rows: here 6 mm apart in
  // height over the 84 mm jaw span, a line 0.07 rad off the jaw axis (measured in the campaigns).
  // The product still swings about the jaw axis, the pads' common normal, so a 0.4 rad pendulum
  // swing is no tilt, however skewed the centroids are.
  Pads skewed;
  skewed.left_point.x() = 0.003;
  skewed.right_point.x() = -0.003;
  auto loose = physical_config();
  loose.maximum_swing_translation_m = 0.05;
  loose.maximum_translation_drift_m = 0.01;  // the skewed centroids sit 3 mm off the axis
  loose.maximum_off_line_rotation_rad = 0.05;
  const Eigen::Vector3d jaw_axis = Eigen::Vector3d::UnitY();
  const Eigen::Vector3d middle = 0.5 * (skewed.left_point + skewed.right_point);
  const Eigen::Isometry3d swung_about_jaw_axis =
    Eigen::Translation3d(middle) * Eigen::AngleAxisd(0.4, jaw_axis) *
    Eigen::Translation3d(-middle) * held_product();
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), loose, physical_observation(swung_about_jaw_axis, skewed),
      AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kAttached);
}

TEST(AttachmentVerifier, PhysicalWatchdogBoundsTheLeanOffTheJawAxisByTheTiltAllowance)
{
  // A lean about the approach axis through the pads' midpoint (the product's axis tipping toward
  // one finger) with the slip bound loosened so only the tilt is under test: inside the 0.05 rad
  // allowance it holds, beyond it it does not.
  auto loose = physical_config();
  loose.maximum_translation_drift_m = 0.01;
  loose.maximum_swing_translation_m = 0.01;
  loose.maximum_off_line_rotation_rad = 0.05;
  const Eigen::Vector3d middle(0.0, 0.0, 0.14);
  const auto leaned = [&middle](double angle) {
    return Eigen::Translation3d(middle) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(-middle) * held_product();
  };
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), loose, physical_observation(leaned(0.04)),
      AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kAttached);
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), loose, physical_observation(leaned(0.06)),
      AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, PhysicalWatchdogDoesNotCountTheFingersCommonModeShiftAsSlip)
{
  // Two force-saturated jaw servos have no common-mode stiffness: both fingers and the product
  // move 1.2 mm together along the jaw axis. The pads did not slip.
  Eigen::Isometry3d shifted = held_product();
  shifted.translation().y() += 0.0012;
  EXPECT_EQ(
    physical_watchdog(physical_observation(shifted, Pads{}, Eigen::Vector3d(0.0, 0.0012, 0.0))),
    AttachmentStatusCode::kAttached);
}

TEST(AttachmentVerifier, PhysicalWatchdogStillCatchesSlipAtThePads)
{
  // The product sliding 1.5 mm along both pads.
  Eigen::Isometry3d slid = held_product();
  slid.translation().z() += 0.0015;
  EXPECT_EQ(
    physical_watchdog(physical_observation(slid)), AttachmentStatusCode::kExternalInconsistency);

  // Slip at one pad only: turning 0.03 rad about the approach axis through the left contact
  // drags the right pad's material point 2 mm.
  const Pads pads;
  const Eigen::Isometry3d about_left =
    Eigen::Translation3d(pads.left_point) *
    Eigen::AngleAxisd(0.03, Eigen::Vector3d::UnitZ()) *
    Eigen::Translation3d(-pads.left_point) * held_product();
  EXPECT_EQ(
    physical_watchdog(physical_observation(about_left)),
    AttachmentStatusCode::kExternalInconsistency);

  // Slip that has since come back: the pad point is where it was, but 1.2 mm of slip happened.
  auto recovered = physical_observation(held_product());
  (*recovered.pads)[1].cumulative_slip_m = 0.0012;
  EXPECT_EQ(physical_watchdog(recovered), AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, PhysicalWatchdogBoundsTheSwingAndRotationOffTheContactLine)
{
  // 0.07 rad swings the origin 3.5 mm: over the allowance.
  EXPECT_EQ(
    physical_watchdog(physical_observation(swung(0.07))),
    AttachmentStatusCode::kExternalInconsistency);

  // Rotation off the contact line is bounded by angle, at the attach-time fidelity bound: 0.02 rad
  // about the approach axis through the pads' midpoint is inside it, 0.035 rad is not (0.035 rad
  // moves each pad point 1.16 mm, so it is slip as well).
  const Eigen::Vector3d middle(0.0, 0.0, 0.14);
  const auto twisted = [&middle](double angle) {
    return Eigen::Translation3d(middle) * Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()) *
           Eigen::Translation3d(-middle) * held_product();
  };
  EXPECT_EQ(
    physical_watchdog(physical_observation(twisted(0.02))), AttachmentStatusCode::kAttached);
  EXPECT_EQ(
    physical_watchdog(physical_observation(twisted(0.035))),
    AttachmentStatusCode::kExternalInconsistency);
  // An off-line angle past the bound with no slip at the pads: turned about the contact line's
  // perpendicular through both pads is impossible without slip, so the angle check is exercised
  // directly by a watchdog whose slip bound is loose.
  auto loose = physical_config();
  loose.maximum_translation_drift_m = 0.002;
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), loose, physical_observation(twisted(0.035)),
      AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kExternalInconsistency);

  // A turn about the product's own axis (parent x for this side hold) moves nothing for an
  // upright cylinder: it is removed from the observed pose, translation and tilt untouched, so
  // it neither drags the pads' material points (r * angle) nor counts as rotation.
  const auto spun = [&](double angle) {
    return Eigen::Translation3d(held_product().translation()) *
           Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()) *
           Eigen::Translation3d(-held_product().translation()) * held_product();
  };
  const Eigen::Isometry3d unspun =
    without_spin_about_product_axis(held_product(), Eigen::Isometry3d(spun(0.2)));
  EXPECT_TRUE(unspun.isApprox(held_product(), 1.0e-9));
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), loose, physical_observation(unspun), AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kAttached);
  // A tilt (a turn about parent z, perpendicular to that axis) is not removed.
  const Eigen::Isometry3d tilted_pose = twisted(0.035);
  const Eigen::Isometry3d kept = without_spin_about_product_axis(held_product(), tilted_pose);
  EXPECT_TRUE(kept.isApprox(tilted_pose, 1.0e-9));

  // No rate bound: a one-tick jolt of 1.35 rad/s off the line at 0.0014 rad (measured, T1).
  auto jolted = physical_observation(twisted(0.0014));
  jolted.relative_angular_velocity_in_parent = Eigen::Vector3d(1.35, 0.0, 0.0);
  EXPECT_EQ(physical_watchdog(jolted), AttachmentStatusCode::kAttached);

  // Unobserved pads, or no swing allowance: the watchdog cannot judge the grip, so it refuses.
  auto unobserved = physical_observation(held_product());
  unobserved.pads.reset();
  EXPECT_EQ(physical_watchdog(unobserved), AttachmentStatusCode::kExternalInconsistency);
  auto unconfigured = physical_config();
  unconfigured.maximum_swing_translation_m = 0.0;
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), unconfigured, physical_observation(held_product()),
      AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, PhysicalSlipIsTheNetExcursionNotTheDistanceTheContactWobbled)
{
  // Four 0.4 mm slips back and forth along the approach never take the product more than
  // 0.4 mm from the attach-time contact: it is still registered where it was verified, so it is
  // 0.4 mm of slip, with the 1.6 mm travelled kept as a receipt.
  PadSlipAccumulator accumulator;
  for (int repeat = 0; repeat < 2; ++repeat) {
    accumulate_pad_slip(accumulator, Eigen::Vector3d(0.0, 0.0, 0.0004), 0.0001);
    accumulate_pad_slip(accumulator, Eigen::Vector3d::Zero(), 0.0001);
  }
  EXPECT_NEAR(cumulative_pad_slip(accumulator), 0.0004, 1.0e-9);
  EXPECT_NEAR(accumulator.path_m, 0.0016, 1.0e-9);
  auto observed = physical_observation(held_product());
  (*observed.pads)[0].cumulative_slip_m = cumulative_pad_slip(accumulator);
  EXPECT_EQ(physical_watchdog(observed), AttachmentStatusCode::kAttached);

  // Slips in one direction: the excursion itself passes 1 mm, and the watchdog trips on it.
  PadSlipAccumulator drifting;
  for (int step = 1; step <= 4; ++step) {
    accumulate_pad_slip(drifting, Eigen::Vector3d(0.0003 * step, 0.0, 0.0), 0.0001);
  }
  EXPECT_NEAR(cumulative_pad_slip(drifting), 0.0012, 1.0e-9);
  (*observed.pads)[0].cumulative_slip_m = cumulative_pad_slip(drifting);
  EXPECT_EQ(physical_watchdog(observed), AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentVerifier, PhysicalSlipCountsNeitherJitterNorTheRateOfASmallPop)
{
  // Per-tick jitter of 20 um around the attach-time contact never reaches the 0.1 mm counting
  // step, so a long hold does not sum it into slip.
  PadSlipAccumulator accumulator;
  for (int tick = 0; tick < 20000; ++tick) {
    const double jitter = (tick % 2 == 0 ? 1.0 : -1.0) * 2.0e-5;
    accumulate_pad_slip(accumulator, Eigen::Vector3d(jitter, 0.0, 0.0), 0.0001);
  }
  EXPECT_LE(cumulative_pad_slip(accumulator), 2.0e-5 + 1.0e-12);

  // A 0.45 mm stick-slip pop within one tick (measured: 0.26 m/s) is 0.45 mm of slip, inside the
  // bound: there is no separate rate limit.
  auto popped = physical_observation(held_product());
  (*popped.pads)[0].cumulative_slip_m = 0.00045;
  EXPECT_EQ(physical_watchdog(popped), AttachmentStatusCode::kAttached);
}

TEST(AttachmentVerifier, WeldWatchdogKeepsItsOriginalBounds)
{
  // A weld cannot swing: the same 2 mm is external interference there.
  auto welded = physical_observation(swung(0.04));
  welded.exact_joint_observed = true;
  EXPECT_EQ(
    validate_attached_watchdog(
      held_product(), physical_config(), welded, AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);
}

}  // namespace
}  // namespace restocker_gazebo
