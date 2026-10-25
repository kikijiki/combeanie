// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <limits>

#include "restocker_gazebo/attachment_physics.hpp"

namespace restocker_gazebo
{
namespace
{

ParallelJawAttachmentGeometry gripper()
{
  return ParallelJawAttachmentGeometry{
    Eigen::AlignedBox3d(
      Eigen::Vector3d(-0.08, -0.055, -0.14),
      Eigen::Vector3d(0.08, 0.055, -0.07)),
    Eigen::AlignedBox2d(
      Eigen::Vector2d(-0.0175, -0.14),
      Eigen::Vector2d(0.0175, 0.0)),
    0.021,
    -0.021,
    0.0,
    0.035,
    0.035,
    0.0005,
    0.005};
}

CylinderAttachmentGeometry product()
{
  return CylinderAttachmentGeometry{0.033, 0.122, 0.013};
}

AttachmentPhysicalTolerances tolerances()
{
  return AttachmentPhysicalTolerances{0.0005, 0.003, 0.03, 0.02, 0.05, 3.0, 0.001};
}

// An observation claiming the same translational accuracy on every axis. The fidelity budget
// reads the largest translational eigenvalue, so this claims exactly sigma_m.
std::array<double, 36> claimed_translation_sigma(double sigma_m)
{
  std::array<double, 36> result{};
  result[0] = sigma_m * sigma_m;
  result[7] = sigma_m * sigma_m;
  result[14] = sigma_m * sigma_m;
  result[21] = 0.01;
  result[28] = 0.01;
  result[35] = 1.0;
  return result;
}

// open_target_m - hold_joint_target_m - minimum_inner_clearance_m for the fixtures above.
constexpr double kCeilingM = 0.035 - 0.013 - 0.0005;

AttachmentRequest request()
{
  AttachmentRequest result;
  result.command = AttachmentCommand::kAttach;
  result.reservation_id = 8;
  result.identity = AttachmentIdentity{17, "sim:can", "restocker", "gripper", "can", "body"};
  result.has_expected_grasp = true;
  result.expected_grasp_center_to_child = CanonicalPose{};
  return result;
}

AttachPhysicalObservation attach_observation()
{
  AttachPhysicalObservation result;
  result.left_finger_position_m = 0.013;
  result.right_finger_position_m = 0.013;
  result.child_dynamic = true;
  return result;
}

DetachPhysicalObservation detach_observation()
{
  DetachPhysicalObservation result;
  result.left_finger_position_m = 0.035;
  result.right_finger_position_m = 0.035;
  result.matching_owned_joint_count = 1;
  return result;
}

TEST(AttachmentPhysics, AcceptsFiniteCenteredCanWithPositiveClearance)
{
  const auto result = validate_physical_attach(
    request(), gripper(), product(), tolerances(), attach_observation(), AttachmentHoldMode::kWeld);
  ASSERT_TRUE(result) << result.status.detail;
  ASSERT_TRUE(result.evidence);
  EXPECT_NEAR(result.evidence->left_inner_clearance_m, 0.001, 1.0e-12);
  EXPECT_NEAR(result.evidence->right_inner_clearance_m, 0.001, 1.0e-12);
  EXPECT_DOUBLE_EQ(result.evidence->translation_error_m, 0.0);
  EXPECT_DOUBLE_EQ(result.evidence->rotation_error_rad, 0.0);
}

TEST(AttachmentPhysics, RejectsInvalidConfigurationBeforeObservations)
{
  auto invalid = gripper();
  invalid.minimum_inner_clearance_m = -0.1;
  const auto result = validate_physical_attach(
    request(), invalid, product(), tolerances(), attach_observation(), AttachmentHoldMode::kWeld);
  EXPECT_FALSE(result);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kInvalidArgument);

  auto invalid_tolerances = tolerances();
  invalid_tolerances.expected_translation_m = 0.0;
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), invalid_tolerances, attach_observation(),
      AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kInvalidArgument);

  auto invalid_product = product();
  invalid_product.hold_joint_target_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), invalid_product, tolerances(), attach_observation(),
      AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentPhysics, RejectsNonCanonicalExpectedQuaternion)
{
  auto invalid_request = request();
  invalid_request.expected_grasp_center_to_child.rotation_xyzw = {0.0, 0.0, 0.0, 0.0};
  const auto result = validate_physical_attach(
    invalid_request, gripper(), product(), tolerances(), attach_observation(),
    AttachmentHoldMode::kWeld);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentPhysics, RejectsStaticOrConstrainedChildren)
{
  auto observation = attach_observation();
  observation.child_dynamic = false;
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kStateMismatch);

  observation = attach_observation();
  observation.other_child_constraint_count = 1;
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentPhysics, RejectsNonFiniteAndExcessiveRelativeMotion)
{
  auto observation = attach_observation();
  observation.relative_linear_velocity_in_parent.x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kOutOfTolerance);

  observation = attach_observation();
  observation.relative_angular_velocity_in_parent.z() = 0.051;
  const auto moving = validate_physical_attach(
    request(), gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(moving.status.code, AttachmentStatusCode::kOutOfTolerance);
  // The refusal carries the measurement, not just the bound: 0.051 rad/s against the 0.05
  // limit, with the linear norm (0) still printed so both channels are visible on one line.
  EXPECT_NE(moving.status.detail.find("0.051"), std::string::npos) << moving.status.detail;
  EXPECT_NE(moving.status.detail.find("0.05"), std::string::npos) << moving.status.detail;
  EXPECT_NE(moving.status.detail.find("linear"), std::string::npos) << moving.status.detail;
  EXPECT_NE(moving.status.detail.find("angular"), std::string::npos) << moving.status.detail;
}

TEST(AttachmentPhysics, RequiresBothFingerHoldTargets)
{
  auto observation = attach_observation();
  observation.right_finger_position_m = 0.014;
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kGripperNotReady);
}

TEST(AttachmentPhysics, EnforcesAuthorizedRelativeTransform)
{
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.0031;
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kOutOfTolerance);

  observation = attach_observation();
  observation.grasp_center_from_child.linear() =
    Eigen::AngleAxisd(0.031, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(),
      observation, AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kOutOfTolerance);
}

TEST(AttachmentPhysics, AdmitsResidualInsideTheClaimedUncertainty)
{
  // 5 mm is a flat refusal against the 3 mm constant, and well inside what an observation
  // claiming 4 mm of translational deviation says about itself.
  auto authorized = request();
  authorized.expected_pose_covariance = claimed_translation_sigma(0.004);
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.005;

  EXPECT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(), observation,
      AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kOutOfTolerance);

  const auto result = validate_physical_attach(
    authorized, gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  ASSERT_TRUE(result) << result.status.detail;
  ASSERT_TRUE(result.evidence);
  EXPECT_NEAR(result.evidence->claimed_translation_sigma_m, 0.004, 1.0e-12);
  EXPECT_NEAR(result.evidence->translation_budget_m, 3.0 * 0.004 + 0.001, 1.0e-12);
  EXPECT_NEAR(result.evidence->translation_error_m, 0.005, 1.0e-12);
}

TEST(AttachmentPhysics, HoldsAConfidentClaimTighterThanTheConstant)
{
  // An observation claiming 0.1 mm is refused at 2 mm, which the fixed 3 mm tolerance admitted.
  auto authorized = request();
  authorized.expected_pose_covariance = claimed_translation_sigma(0.0001);
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.002;

  ASSERT_TRUE(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld));

  const auto result = validate_physical_attach(
    authorized, gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutOfTolerance);
  ASSERT_TRUE(result.evidence);
  EXPECT_NEAR(result.evidence->translation_budget_m, 3.0 * 0.0001 + 0.001, 1.0e-12);
  EXPECT_LT(result.evidence->translation_budget_m, 0.003);
}

TEST(AttachmentPhysics, CapsAnImplausibleClaimAtTheJawTravelCeiling)
{
  // Claimed uncertainty never buys slack past the offset at which the closing jaws stop
  // capturing the product.
  auto authorized = request();
  authorized.expected_pose_covariance = claimed_translation_sigma(1.0);
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.03;

  const auto result = validate_physical_attach(
    authorized, gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutOfTolerance);
  ASSERT_TRUE(result.evidence);
  EXPECT_NEAR(result.evidence->translation_ceiling_m, kCeilingM, 1.0e-12);
  EXPECT_NEAR(result.evidence->translation_budget_m, kCeilingM, 1.0e-12);
  EXPECT_NEAR(
    fidelity_translation_ceiling(gripper(), product()), kCeilingM, 1.0e-12);
}

TEST(AttachmentPhysics, ReportsTheResidualOnRefusalAsWellAsOnAcceptance)
{
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.0031;
  const auto result = validate_physical_attach(
    request(), gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutOfTolerance);
  ASSERT_TRUE(result.evidence) << "the residual is the measurement, not only the refusal";
  EXPECT_NEAR(result.evidence->translation_error_m, 0.0031, 1.0e-12);
  // No claim was stated, so the boundary keeps its configured constant.
  EXPECT_DOUBLE_EQ(result.evidence->claimed_translation_sigma_m, 0.0);
  EXPECT_NEAR(result.evidence->translation_budget_m, 0.003, 1.0e-12);
}

TEST(AttachmentPhysics, RefusesACovarianceThatIsNotOne)
{
  auto authorized = request();
  authorized.expected_pose_covariance = claimed_translation_sigma(0.004);
  // A negative variance is a malformed claim, not a confident one.
  authorized.expected_pose_covariance[7] = -1.0;
  EXPECT_EQ(
    validate_physical_attach(
      authorized, gripper(), product(), tolerances(),
      attach_observation(), AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kInvalidArgument);
}

TEST(AttachmentPhysics, RejectsFingerAndPalmPenetration)
{
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().y() = 0.0006;
  auto finger_result = validate_physical_attach(
    request(), gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(finger_result.status.code, AttachmentStatusCode::kOutOfTolerance);

  auto palm_request = request();
  palm_request.expected_grasp_center_to_child.translation[2] = -0.01;
  observation = attach_observation();
  observation.grasp_center_from_child.translation().z() = -0.01;
  auto palm_result = validate_physical_attach(
    palm_request, gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(palm_result.status.code, AttachmentStatusCode::kOutOfTolerance);
  EXPECT_NE(palm_result.status.detail.find("palm"), std::string::npos);
}

TEST(AttachmentPhysics, RequiresUsefulFingerContactOverlap)
{
  auto shifted_request = request();
  shifted_request.expected_grasp_center_to_child.translation[0] = 0.06;
  auto observation = attach_observation();
  observation.grasp_center_from_child.translation().x() = 0.06;
  const auto result = validate_physical_attach(
    shifted_request, gripper(), product(), tolerances(), observation, AttachmentHoldMode::kWeld);
  EXPECT_EQ(result.status.code, AttachmentStatusCode::kOutOfTolerance);
  EXPECT_NE(result.status.detail.find("contact"), std::string::npos);
}

TEST(AttachmentPhysics, AcceptsOnlyExactOpenAndOwnedDetachState)
{
  EXPECT_EQ(
    validate_physical_detach(
      gripper(), tolerances(), detach_observation(),
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kPending);

  auto closed = detach_observation();
  closed.left_finger_position_m = 0.013;
  EXPECT_EQ(
    validate_physical_detach(gripper(), tolerances(), closed, AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kGripperNotReady);

  auto ambiguous = detach_observation();
  ambiguous.matching_owned_joint_count = 2;
  EXPECT_EQ(
    validate_physical_detach(gripper(), tolerances(), ambiguous, AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentPhysics, RejectsNonFiniteDetachEvidence)
{
  auto observation = detach_observation();
  observation.world_from_child.translation().z() =
    std::numeric_limits<double>::infinity();
  EXPECT_EQ(
    validate_physical_detach(gripper(), tolerances(), observation, AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kOutOfTolerance);

  auto invalid_gripper = gripper();
  invalid_gripper.open_target_m = 0.05;
  EXPECT_EQ(
    validate_physical_detach(
      invalid_gripper, tolerances(), detach_observation(),
      AttachmentHoldMode::kWeld).code,
    AttachmentStatusCode::kInvalidArgument);
}

// ---------------------------------------------------------------------------
// Physical mode (Milestone 10 spec 11.4): the jaws are commanded past the surface, so the weld
// fixtures above — daylight under every finger, positions resting on the hold target — are
// exactly the state physical mode must refuse. The first test below is the red side of that
// pair: it fails under kPhysical while the weld-mode test of the same observation passes.
// ---------------------------------------------------------------------------

ParallelJawAttachmentGeometry physical_gripper()
{
  ParallelJawAttachmentGeometry result = gripper();
  result.maximum_penetration_m = 0.002;
  result.contact_tolerance_m = 0.0001;
  return result;
}

// hold target 0.5 * (diameter - 2 * interference - zero_gap) = 0.5 * (0.066 - 0.002 - 0.042).
CylinderAttachmentGeometry physical_product()
{
  return CylinderAttachmentGeometry{0.033, 0.122, 0.011};
}

// A grip at rest: each face sits 0.0005 m inside the catalogued surface (finger position 0.0115,
// since clearance = position - 0.012 for this fixture), both jaws stopped, product centred.
AttachPhysicalObservation physical_grip()
{
  AttachPhysicalObservation result;
  result.left_finger_position_m = 0.0115;
  result.right_finger_position_m = 0.0115;
  result.left_finger_velocity_mps = 0.0;
  result.right_finger_velocity_mps = 0.0;
  result.child_dynamic = true;
  return result;
}

TEST(AttachmentPhysics, PhysicalModeAcceptsAStalledGripInsideTheContactBand)
{
  const auto result = validate_physical_attach(
    request(), physical_gripper(), physical_product(), tolerances(), physical_grip(),
    AttachmentHoldMode::kPhysical);
  ASSERT_TRUE(result) << result.status.detail;
  ASSERT_TRUE(result.evidence);
  EXPECT_NEAR(result.evidence->left_inner_clearance_m, -0.0005, 1.0e-12);
  EXPECT_NEAR(result.evidence->right_inner_clearance_m, -0.0005, 1.0e-12);
}

TEST(AttachmentPhysics, PhysicalModeRefusesTheWeldHoverThatWeldModeAccepts)
{
  // The weld fixture: fingers one hold clearance off the product, still travelling nowhere near
  // the interference command. Weld mode accepts this exact observation (the first test in this
  // file); physical mode must not, because nothing is touching anything.
  const auto weld_observation = attach_observation();
  ASSERT_EQ(
    validate_physical_attach(
      request(), gripper(), product(), tolerances(), weld_observation,
      AttachmentHoldMode::kWeld).status.code,
    AttachmentStatusCode::kPending);

  const auto refused = validate_physical_attach(
    request(), physical_gripper(), physical_product(), tolerances(), weld_observation,
    AttachmentHoldMode::kPhysical);
  ASSERT_FALSE(refused) << refused.status.detail;
  EXPECT_EQ(refused.status.code, AttachmentStatusCode::kOutOfTolerance);
  ASSERT_TRUE(refused.evidence);
  EXPECT_GT(refused.evidence->left_inner_clearance_m, 0.0);
}

TEST(AttachmentPhysics, PhysicalModeRefusesAJawThatIsStillTravelling)
{
  auto observation = physical_grip();
  observation.left_finger_velocity_mps = 0.05;
  const auto travelling = validate_physical_attach(
    request(), physical_gripper(), physical_product(), tolerances(), observation,
    AttachmentHoldMode::kPhysical);
  EXPECT_EQ(travelling.status.code, AttachmentStatusCode::kGripperNotReady);
  EXPECT_NE(travelling.status.detail.find("0.05"), std::string::npos) << travelling.status.detail;
}

TEST(AttachmentPhysics, PhysicalModeRefusesAFingerThatClosedPastTheInterferenceCommand)
{
  // 0.010 is one joint tolerance below the 0.011 command: the jaw swept through where the
  // product should have stopped it, so it closed on nothing catalogued.
  auto observation = physical_grip();
  observation.left_finger_position_m = 0.010;
  observation.right_finger_position_m = 0.010;
  EXPECT_EQ(
    validate_physical_attach(
      request(), physical_gripper(), physical_product(), tolerances(), observation,
      AttachmentHoldMode::kPhysical).status.code,
    AttachmentStatusCode::kGripperNotReady);
}

TEST(AttachmentPhysics, PhysicalModeRefusesAFingerOutsideTheContactBand)
{
  // Still 1.5 mm off the surface: the far jaw never reached the product, which is a one-sided
  // hold and not a grip this boundary certifies.
  auto observation = physical_grip();
  observation.right_finger_position_m = 0.0135;
  EXPECT_EQ(
    validate_physical_attach(
      request(), physical_gripper(), physical_product(), tolerances(), observation,
      AttachmentHoldMode::kPhysical).status.code,
    AttachmentStatusCode::kOutOfTolerance);
}

TEST(AttachmentPhysics, PhysicalDetachInvertsTheOwnedJointRequirement)
{
  auto free = detach_observation();
  free.matching_owned_joint_count = 0;
  EXPECT_EQ(
    validate_physical_detach(
      physical_gripper(), tolerances(), free, AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kPending);

  // An owned joint existing in physical mode means something other than this boundary created
  // it, which is the external inconsistency the weld path expects the other way round.
  EXPECT_EQ(
    validate_physical_detach(
      physical_gripper(), tolerances(), detach_observation(), AttachmentHoldMode::kPhysical).code,
    AttachmentStatusCode::kExternalInconsistency);
}

TEST(AttachmentPhysics, FiniteDifferenceTwistMeasuresMotionFromPoses)
{
  Eigen::Isometry3d previous = Eigen::Isometry3d::Identity();
  previous.translation() = Eigen::Vector3d(0.4, -0.1, 0.9);
  previous.linear() = Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitX()).toRotationMatrix();
  Eigen::Isometry3d current = previous;
  current.translation() += Eigen::Vector3d(0.002, -0.001, 0.0005);
  current.linear() =
    Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitZ()).toRotationMatrix() * previous.linear();

  const auto twist = finite_difference_world_twist(previous, current, 0.001);
  ASSERT_TRUE(twist.has_value());
  EXPECT_NEAR((twist->linear - Eigen::Vector3d(2.0, -1.0, 0.5)).norm(), 0.0, 1.0e-9);
  EXPECT_NEAR((twist->angular - Eigen::Vector3d(0.0, 0.0, 10.0)).norm(), 0.0, 1.0e-9);
}

TEST(AttachmentPhysics, FiniteDifferenceTwistOfAnUnchangedPoseIsZero)
{
  // The case the engine gets wrong: a link at rest after a transient still reports the transient,
  // while two identical poses can only mean zero motion.
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.5, 0.2, 1.1);
  pose.linear() = Eigen::AngleAxisd(1.2, Eigen::Vector3d(0.0, 0.6, 0.8)).toRotationMatrix();
  const auto twist = finite_difference_world_twist(pose, pose, 0.001);
  ASSERT_TRUE(twist.has_value());
  EXPECT_EQ(twist->linear.norm(), 0.0);
  EXPECT_EQ(twist->angular.norm(), 0.0);
}

TEST(AttachmentPhysics, FiniteDifferenceTwistRefusesWithoutElapsedTimeOrFinitePoses)
{
  const Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(finite_difference_world_twist(pose, pose, 0.0).has_value());
  EXPECT_FALSE(finite_difference_world_twist(pose, pose, -0.001).has_value());
  EXPECT_FALSE(
    finite_difference_world_twist(pose, pose, std::numeric_limits<double>::quiet_NaN())
    .has_value());
  Eigen::Isometry3d broken = pose;
  broken.translation().x() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(finite_difference_world_twist(pose, broken, 0.001).has_value());
  EXPECT_FALSE(finite_difference_world_twist(broken, pose, 0.001).has_value());
}

}  // namespace
}  // namespace restocker_gazebo
