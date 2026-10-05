// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_physics.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>

namespace restocker_gazebo
{
namespace
{

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool finite_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] bool finite_box(const Eigen::AlignedBox3d & box)
{
  return !box.isEmpty() && box.min().allFinite() && box.max().allFinite() &&
         (box.sizes().array() > 0.0).all();
}

[[nodiscard]] bool finite_box(const Eigen::AlignedBox2d & box)
{
  return !box.isEmpty() && box.min().allFinite() && box.max().allFinite() &&
         (box.sizes().array() > 0.0).all();
}

[[nodiscard]] Eigen::Isometry3d transform(const CanonicalPose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.translation[0], pose.translation[1], pose.translation[2]);
  result.linear() = Eigen::Quaterniond(
    pose.rotation_xyzw[3], pose.rotation_xyzw[0], pose.rotation_xyzw[1],
    pose.rotation_xyzw[2]).toRotationMatrix();
  return result;
}

[[nodiscard]] CanonicalPose pose(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond rotation(transform.linear());
  const auto canonical = canonicalize_pose(
    {transform.translation().x(), transform.translation().y(), transform.translation().z()},
    {rotation.x(), rotation.y(), rotation.z(), rotation.w()});
  return canonical.value_or(CanonicalPose{});
}

[[nodiscard]] Eigen::Vector3d cylinder_extent(
  const Eigen::Matrix3d & rotation, const CylinderAttachmentGeometry & product)
{
  Eigen::Vector3d result;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    result[axis] = product.radius_m *
      std::hypot(rotation(axis, 0), rotation(axis, 1)) +
      0.5 * product.height_m * std::abs(rotation(axis, 2));
  }
  return result;
}

[[nodiscard]] bool separated_with_clearance(
  const Eigen::AlignedBox3d & left, const Eigen::AlignedBox3d & right,
  double clearance)
{
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    if (left.max()[axis] + clearance <= right.min()[axis] ||
      right.max()[axis] + clearance <= left.min()[axis])
    {
      return true;
    }
  }
  return false;
}

[[nodiscard]] double overlap(double left_min, double left_max, double right_min, double right_max)
{
  return std::min(left_max, right_max) - std::max(left_min, right_min);
}

[[nodiscard]] bool joint_near(double observed, double target, double tolerance)
{
  return std::isfinite(observed) && std::abs(observed - target) <= tolerance;
}

[[nodiscard]] AttachValidationResult rejected(AttachmentStatusCode code, std::string detail)
{
  return AttachValidationResult{status(code, std::move(detail)), std::nullopt};
}

// A rejection that still carries what the fidelity assertion measured. The residual says whether
// an estimated grasp works, on failed attempts as much as on passing ones.
[[nodiscard]] AttachValidationResult rejected(
  AttachmentStatusCode code, std::string detail, AttachValidationEvidence evidence)
{
  return AttachValidationResult{status(code, std::move(detail)), std::move(evidence)};
}

struct ClaimedTranslationUncertainty
{
  bool stated{false};
  bool valid{true};
  double sigma_m{0.0};
};

// What the authorizing observation claims about its own translational accuracy, as the standard
// deviation along the worst-conditioned axis. Rotation preserves eigenvalues, so the claim needs
// no transform out of the frame the observation was expressed in.
[[nodiscard]] ClaimedTranslationUncertainty claimed_translation_uncertainty(
  const std::array<double, 36> & covariance)
{
  ClaimedTranslationUncertainty result;
  for (const double entry : covariance) {
    if (!std::isfinite(entry)) {
      result.valid = false;
      return result;
    }
    result.stated = result.stated || entry != 0.0;
  }
  if (!result.stated) {
    return result;
  }
  Eigen::Matrix3d translational;
  for (Eigen::Index row = 0; row < 3; ++row) {
    for (Eigen::Index column = 0; column < 3; ++column) {
      translational(row, column) = covariance[static_cast<std::size_t>(row) * 6 +
          static_cast<std::size_t>(column)];
    }
  }
  translational = 0.5 * (translational + translational.transpose().eval());
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(translational);
  if (solver.info() != Eigen::Success) {
    result.valid = false;
    return result;
  }
  const double largest = solver.eigenvalues().maxCoeff();
  // A covariance is positive semi-definite. One that is not describes no distribution, so it is a
  // malformed claim, refused rather than read as confidence. The tolerance absorbs rounding of a
  // matrix that was symmetric on the wire.
  const double negative_limit = -1.0e-12 * std::max(1.0, std::abs(largest));
  if (!std::isfinite(largest) || solver.eigenvalues().minCoeff() < negative_limit) {
    result.valid = false;
    return result;
  }
  result.sigma_m = std::sqrt(std::max(largest, 0.0));
  return result;
}

}  // namespace

double fidelity_translation_ceiling(
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product)
{
  // A product offset laterally by more than left_inner_face_at_zero_m + open_target_m - radius_m
  // does not fit inside the opened jaws at all, so the closing faces push it aside instead of
  // sweeping it in. Each finger's travel, open_target_m - hold_joint_target_m, is that same bound
  // less the per-side hold clearance, because the hold target is where the faces sit one hold
  // clearance off the product surface. Holding back minimum_inner_clearance_m as well leaves this
  // ceiling a hold clearance plus an inner clearance inside the bare geometric bound.
  return gripper.open_target_m - product.hold_joint_target_m - gripper.minimum_inner_clearance_m;
}

AttachmentStatus validate_attachment_geometry(
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product,
  const AttachmentPhysicalTolerances & tolerances)
{
  if (!finite_box(gripper.palm_bounds_in_grasp_center) ||
    !finite_box(gripper.finger_contact_bounds_xz) ||
    !std::isfinite(gripper.left_inner_face_at_zero_m) ||
    !std::isfinite(gripper.right_inner_face_at_zero_m) ||
    gripper.left_inner_face_at_zero_m <= gripper.right_inner_face_at_zero_m ||
    !finite_nonnegative(gripper.joint_lower_m) ||
    !finite_positive(gripper.joint_upper_m) ||
    gripper.joint_lower_m >= gripper.joint_upper_m ||
    !finite_positive(gripper.minimum_inner_clearance_m) ||
    !finite_positive(gripper.minimum_contact_overlap_m) ||
    !finite_positive(product.radius_m) || !finite_positive(product.height_m) ||
    !std::isfinite(gripper.open_target_m) ||
    !std::isfinite(product.hold_joint_target_m))
  {
    return status(AttachmentStatusCode::kInvalidArgument, "attachment geometry is invalid");
  }
  if (gripper.open_target_m < gripper.joint_lower_m ||
    gripper.open_target_m > gripper.joint_upper_m ||
    product.hold_joint_target_m < gripper.joint_lower_m ||
    product.hold_joint_target_m > gripper.joint_upper_m)
  {
    return status(
      AttachmentStatusCode::kInvalidArgument,
      "attachment open or product hold target is outside the gripper limits");
  }
  if (!finite_positive(tolerances.joint_position_m) ||
    !finite_positive(tolerances.expected_translation_m) ||
    !finite_positive(tolerances.expected_rotation_rad) ||
    tolerances.expected_rotation_rad > std::acos(-1.0) ||
    !finite_positive(tolerances.relative_linear_velocity_mps) ||
    !finite_positive(tolerances.relative_angular_velocity_radps) ||
    !finite_positive(tolerances.fidelity_sigma_multiplier) ||
    !finite_positive(tolerances.fidelity_mechanical_margin_m))
  {
    return status(AttachmentStatusCode::kInvalidArgument, "attachment tolerances are invalid");
  }
  if (!finite_positive(fidelity_translation_ceiling(gripper, product))) {
    return status(
      AttachmentStatusCode::kInvalidArgument,
      "jaw travel from the open target to the hold target cannot clear the product");
  }
  return status(AttachmentStatusCode::kPending, "attachment geometry is valid");
}

AttachValidationResult validate_physical_attach(
  const AttachmentRequest & request,
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product,
  const AttachmentPhysicalTolerances & tolerances,
  const AttachPhysicalObservation & observation)
{
  const auto geometry_status = validate_attachment_geometry(gripper, product, tolerances);
  if (geometry_status.code != AttachmentStatusCode::kPending) {
    return rejected(geometry_status.code, geometry_status.detail);
  }
  if (request.command != AttachmentCommand::kAttach || !request.has_expected_grasp) {
    return rejected(
      AttachmentStatusCode::kInvalidArgument,
      "physical attach validation requires an attach request and expected grasp");
  }
  if (!observation.child_dynamic || observation.parent_and_child_share_rigid_body) {
    return rejected(
      AttachmentStatusCode::kStateMismatch,
      "attachment child must be dynamic and outside the parent rigid body");
  }
  if (observation.matching_owned_joint_count != 0 ||
    observation.other_child_constraint_count != 0)
  {
    return rejected(
      AttachmentStatusCode::kExternalInconsistency,
      "attachment child already participates in a constrained topology");
  }
  if (!finite_transform(observation.grasp_center_from_child) ||
    !observation.relative_linear_velocity_in_parent.allFinite() ||
    !observation.relative_angular_velocity_in_parent.allFinite())
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "attachment pose or relative velocity is non-finite");
  }
  if (observation.relative_linear_velocity_in_parent.norm() >
    tolerances.relative_linear_velocity_mps ||
    observation.relative_angular_velocity_in_parent.norm() >
    tolerances.relative_angular_velocity_radps)
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "attachment relative velocity exceeds the pre-attach limit");
  }
  if (!joint_near(
      observation.left_finger_position_m, product.hold_joint_target_m,
      tolerances.joint_position_m) ||
    !joint_near(
      observation.right_finger_position_m, product.hold_joint_target_m,
      tolerances.joint_position_m))
  {
    return rejected(
      AttachmentStatusCode::kGripperNotReady,
      "both fingers must be at the product-specific hold target");
  }

  const auto canonical_expected = canonicalize_pose(
    request.expected_grasp_center_to_child.translation,
    request.expected_grasp_center_to_child.rotation_xyzw);
  if (!canonical_expected) {
    return rejected(
      AttachmentStatusCode::kInvalidArgument, "expected grasp transform is invalid");
  }
  const auto claimed = claimed_translation_uncertainty(request.expected_pose_covariance);
  if (!claimed.valid) {
    return rejected(
      AttachmentStatusCode::kInvalidArgument,
      "expected pose covariance is not a covariance matrix");
  }
  const Eigen::Isometry3d expected = transform(*canonical_expected);
  const double translation_error =
    (expected.translation() - observation.grasp_center_from_child.translation()).norm();
  const double rotation_error = Eigen::Quaterniond(expected.linear()).angularDistance(
    Eigen::Quaterniond(observation.grasp_center_from_child.linear()));

  // The four checks above are physical predicates on the gripper and product. This one asks
  // whether the estimate that authorized the grasp is as accurate as it claimed, so it is
  // measured against the claim rather than a constant calibrated for another sensor. A caller
  // that states no uncertainty keeps the constant; a confident claim is held well inside it.
  const double ceiling = fidelity_translation_ceiling(gripper, product);
  const double claimed_budget = claimed.stated ?
    tolerances.fidelity_sigma_multiplier * claimed.sigma_m +
    tolerances.fidelity_mechanical_margin_m :
    tolerances.expected_translation_m;
  const double translation_budget = std::min(claimed_budget, ceiling);

  AttachValidationEvidence evidence;
  evidence.observed_grasp_center_to_child = pose(observation.grasp_center_from_child);
  evidence.translation_error_m = translation_error;
  evidence.rotation_error_rad = rotation_error;
  evidence.translation_budget_m = translation_budget;
  evidence.claimed_translation_sigma_m = claimed.sigma_m;
  evidence.translation_ceiling_m = ceiling;

  if (translation_error > translation_budget ||
    rotation_error > tolerances.expected_rotation_rad)
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "observed grasp transform differs from the authorized candidate", evidence);
  }

  const Eigen::Vector3d extent = cylinder_extent(
    observation.grasp_center_from_child.linear(), product);
  const Eigen::Vector3d product_min =
    observation.grasp_center_from_child.translation() - extent;
  const Eigen::Vector3d product_max =
    observation.grasp_center_from_child.translation() + extent;
  const Eigen::AlignedBox3d product_bounds(product_min, product_max);
  const double left_inner_face =
    gripper.left_inner_face_at_zero_m + observation.left_finger_position_m;
  const double right_inner_face =
    gripper.right_inner_face_at_zero_m - observation.right_finger_position_m;
  const double left_clearance = left_inner_face - product_max.y();
  const double right_clearance = product_min.y() - right_inner_face;
  evidence.left_inner_clearance_m = left_clearance;
  evidence.right_inner_clearance_m = right_clearance;
  if (left_clearance < gripper.minimum_inner_clearance_m ||
    right_clearance < gripper.minimum_inner_clearance_m)
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "product does not have positive clearance from both finger inner faces", evidence);
  }
  if (!separated_with_clearance(
      product_bounds, gripper.palm_bounds_in_grasp_center,
      gripper.minimum_inner_clearance_m))
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "product collision envelope penetrates the gripper palm clearance envelope", evidence);
  }
  const double contact_x_overlap = overlap(
    product_min.x(), product_max.x(), gripper.finger_contact_bounds_xz.min().x(),
    gripper.finger_contact_bounds_xz.max().x());
  const double contact_z_overlap = overlap(
    product_min.z(), product_max.z(), gripper.finger_contact_bounds_xz.min().y(),
    gripper.finger_contact_bounds_xz.max().y());
  if (contact_x_overlap < gripper.minimum_contact_overlap_m ||
    contact_z_overlap < gripper.minimum_contact_overlap_m)
  {
    return rejected(
      AttachmentStatusCode::kOutOfTolerance,
      "product does not overlap the configured finger contact region", evidence);
  }

  return AttachValidationResult{
    status(AttachmentStatusCode::kPending, "physical attach preconditions satisfied"),
    evidence};
}

AttachmentStatus validate_physical_detach(
  const ParallelJawAttachmentGeometry & gripper,
  const AttachmentPhysicalTolerances & tolerances,
  const DetachPhysicalObservation & observation)
{
  if (!finite_positive(tolerances.joint_position_m) ||
    !finite_nonnegative(gripper.joint_lower_m) ||
    !finite_positive(gripper.joint_upper_m) ||
    gripper.joint_lower_m >= gripper.joint_upper_m ||
    !std::isfinite(gripper.open_target_m) ||
    gripper.open_target_m < gripper.joint_lower_m ||
    gripper.open_target_m > gripper.joint_upper_m)
  {
    return status(AttachmentStatusCode::kInvalidArgument, "detach configuration is invalid");
  }
  if (observation.matching_owned_joint_count != 1 ||
    observation.other_owned_joint_count != 0)
  {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "detach requires exactly one matching owned joint and no other owned joints");
  }
  if (!finite_transform(observation.parent_from_child) ||
    !finite_transform(observation.world_from_child))
  {
    return status(
      AttachmentStatusCode::kOutOfTolerance, "detach pose observations are non-finite");
  }
  if (!joint_near(
      observation.left_finger_position_m, gripper.open_target_m,
      tolerances.joint_position_m) ||
    !joint_near(
      observation.right_finger_position_m, gripper.open_target_m,
      tolerances.joint_position_m))
  {
    return status(
      AttachmentStatusCode::kGripperNotReady,
      "both fingers must be at the open target before detachment");
  }
  return status(AttachmentStatusCode::kPending, "physical detach preconditions satisfied");
}

}  // namespace restocker_gazebo
