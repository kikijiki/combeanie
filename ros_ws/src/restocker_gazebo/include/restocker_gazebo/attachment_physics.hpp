// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <optional>

#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{

// All geometry is expressed in the semantic grasp-center frame. The Gazebo system constructs this
// immutable projection from the description-owned gripper and product configuration at startup.
struct ParallelJawAttachmentGeometry
{
  Eigen::AlignedBox3d palm_bounds_in_grasp_center;
  Eigen::AlignedBox2d finger_contact_bounds_xz;
  double left_inner_face_at_zero_m{0.0};
  double right_inner_face_at_zero_m{0.0};
  double joint_lower_m{0.0};
  double joint_upper_m{0.0};
  double open_target_m{0.0};
  double minimum_inner_clearance_m{0.0};
  double minimum_contact_overlap_m{0.0};
};

struct CylinderAttachmentGeometry
{
  double radius_m{0.0};
  double height_m{0.0};
  double hold_joint_target_m{0.0};
};

struct AttachmentPhysicalTolerances
{
  double joint_position_m{0.0};
  // The translation bound applied when the caller states no uncertainty for the observation its
  // authorized grasp came from. A caller that does state one is held to the budget below instead.
  double expected_translation_m{0.0};
  double expected_rotation_rad{0.0};
  double relative_linear_velocity_mps{0.0};
  double relative_angular_velocity_radps{0.0};
  // Multiplies the standard deviation the authorizing observation claims about itself, so the
  // fidelity assertion tracks the estimator instead of a constant: a confident estimate is held
  // well inside expected_translation_m, a coarse one is not held below its sensor's resolution.
  double fidelity_sigma_multiplier{0.0};
  // Added to that scaled deviation. The jaws close on the product and move it, so the observed
  // grasp differs from the authorized one even when the estimate was exact; this also absorbs the
  // forward-kinematic error in the grasp frame itself, which no observation covariance describes.
  double fidelity_mechanical_margin_m{0.0};
};

struct AttachPhysicalObservation
{
  Eigen::Isometry3d grasp_center_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d relative_linear_velocity_in_parent{Eigen::Vector3d::Zero()};
  Eigen::Vector3d relative_angular_velocity_in_parent{Eigen::Vector3d::Zero()};
  double left_finger_position_m{0.0};
  double right_finger_position_m{0.0};
  bool child_dynamic{false};
  bool parent_and_child_share_rigid_body{false};
  std::size_t matching_owned_joint_count{0};
  std::size_t other_child_constraint_count{0};
};

struct DetachPhysicalObservation
{
  Eigen::Isometry3d parent_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_child{Eigen::Isometry3d::Identity()};
  double left_finger_position_m{0.0};
  double right_finger_position_m{0.0};
  std::size_t matching_owned_joint_count{0};
  std::size_t other_owned_joint_count{0};
};

struct AttachValidationEvidence
{
  CanonicalPose observed_grasp_center_to_child;
  double translation_error_m{0.0};
  double rotation_error_rad{0.0};
  double left_inner_clearance_m{0.0};
  double right_inner_clearance_m{0.0};
  // The bound translation_error_m was actually held to, and the two quantities it came from.
  double translation_budget_m{0.0};
  double claimed_translation_sigma_m{0.0};
  double translation_ceiling_m{0.0};
};

struct AttachValidationResult
{
  AttachmentStatus status;
  std::optional<AttachValidationEvidence> evidence;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status.code == AttachmentStatusCode::kPending && evidence.has_value();
  }
};

// Largest lateral offset at which the closing jaws still capture the product rather than push it
// aside, less the clearance the faces must keep. No claimed uncertainty raises the fidelity budget
// above this: beyond it the jaws no longer capture the product.
[[nodiscard]] double fidelity_translation_ceiling(
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product);

[[nodiscard]] AttachmentStatus validate_attachment_geometry(
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product,
  const AttachmentPhysicalTolerances & tolerances);

[[nodiscard]] AttachValidationResult validate_physical_attach(
  const AttachmentRequest & request,
  const ParallelJawAttachmentGeometry & gripper,
  const CylinderAttachmentGeometry & product,
  const AttachmentPhysicalTolerances & tolerances,
  const AttachPhysicalObservation & observation);

[[nodiscard]] AttachmentStatus validate_physical_detach(
  const ParallelJawAttachmentGeometry & gripper,
  const AttachmentPhysicalTolerances & tolerances,
  const DetachPhysicalObservation & observation);

}  // namespace restocker_gazebo
