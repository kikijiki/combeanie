// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/attachment_port.hpp"

#include <cmath>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool finite_rigid_pose(const Eigen::Isometry3d & pose) noexcept
{
  const auto & matrix = pose.matrix();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!std::isfinite(matrix(row, column))) {
        return false;
      }
    }
  }
  // The attachment boundary canonicalizes the pose into a unit quaternion and compares the
  // observed transform against it within millimetre tolerances. A non-orthonormal rotation would
  // be normalized there, so the checked expectation would differ from the asserted one.
  const Eigen::Matrix3d rotation = pose.linear();
  const Eigen::Matrix3d residual = rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  return residual.cwiseAbs().maxCoeff() <= 1e-6;
}

}  // namespace

const char * attachment_direction_name(AttachmentDirection direction) noexcept
{
  switch (direction) {
    case AttachmentDirection::kAttach:
      return "attach";
    case AttachmentDirection::kDetach:
      return "detach";
  }
  return "unknown";
}

const char * attachment_scope_name(AttachmentScope scope) noexcept
{
  switch (scope) {
    case AttachmentScope::kFull:
      return "full";
    case AttachmentScope::kPhysicalOnly:
      return "physical-only";
    case AttachmentScope::kSemanticOnly:
      return "semantic-only";
  }
  return "unknown";
}

bool valid_attachment_goal(const AttachmentGoal & goal) noexcept
{
  if (goal.object_id == 0U || goal.reservation_token.empty()) {
    return false;
  }
  if (goal.lease_timeout.count() <= 0) {
    return false;
  }
  // Timeouts that a scope never uses may stay at their defaults; only the ones the saga will
  // honour have to be positive.
  if (goal.scope != AttachmentScope::kSemanticOnly && goal.physical_timeout.count() <= 0) {
    return false;
  }
  if (goal.scope != AttachmentScope::kPhysicalOnly && goal.commit_timeout.count() <= 0) {
    return false;
  }
  if (!finite_rigid_pose(goal.grasp_center_to_child)) {
    return false;
  }
  if (goal.direction == AttachmentDirection::kDetach) {
    // The boundary's request conversion rejects a detach whose expected pose is not the default.
    // Catching it here keeps the rejection out of the transaction, where it would surface as an
    // argument error instead of a caller mistake.
    if (goal.grasp_center_to_child.matrix() != Eigen::Matrix4d::Identity()) {
      return false;
    }
  }
  if (goal.scope == AttachmentScope::kSemanticOnly) {
    // A semantic-only detach commits a release that already happened; without the boundary's own
    // release stamp the placement proof has nothing to order evidence against.
    if (goal.direction != AttachmentDirection::kDetach) {
      return false;
    }
    if (goal.released_at.zero()) {
      return false;
    }
  }
  if (goal.scope == AttachmentScope::kPhysicalOnly &&
    goal.direction != AttachmentDirection::kDetach)
  {
    // Attach has no mid-saga camera proof to wait for; splitting it is not a supported mode.
    return false;
  }
  return true;
}

AttachmentWorldEffect attachment_world_effect(AttachmentOutcome outcome) noexcept
{
  switch (outcome) {
    case AttachmentOutcome::kSucceeded:
      return AttachmentWorldEffect::kFullyApplied;
    case AttachmentOutcome::kRejected:
    case AttachmentOutcome::kUnavailable:
    case AttachmentOutcome::kCanceled:
    case AttachmentOutcome::kPhysicalFailed:
      return AttachmentWorldEffect::kNoneApplied;
    case AttachmentOutcome::kUncommitted:
      return AttachmentWorldEffect::kPhysicalOnly;
    case AttachmentOutcome::kIndeterminate:
      return AttachmentWorldEffect::kIndeterminate;
  }
  // An outcome this function does not recognize cannot be proven safe.
  return AttachmentWorldEffect::kIndeterminate;
}

bool attachment_definitely_not_applied(AttachmentOutcome outcome) noexcept
{
  return attachment_world_effect(outcome) == AttachmentWorldEffect::kNoneApplied;
}

bool attachment_requires_operator(AttachmentOutcome outcome) noexcept
{
  const auto effect = attachment_world_effect(outcome);
  // A physical-only transition is as unrunnable as an unproven one: the arm holds a product world
  // state does not know about, and no automatic action can restore agreement.
  return effect == AttachmentWorldEffect::kIndeterminate ||
         effect == AttachmentWorldEffect::kPhysicalOnly;
}

const char * attachment_outcome_name(AttachmentOutcome outcome) noexcept
{
  switch (outcome) {
    case AttachmentOutcome::kSucceeded:
      return "succeeded";
    case AttachmentOutcome::kRejected:
      return "rejected before mutation";
    case AttachmentOutcome::kUnavailable:
      return "boundary unavailable";
    case AttachmentOutcome::kCanceled:
      return "canceled before mutation";
    case AttachmentOutcome::kPhysicalFailed:
      return "physical transition not applied";
    case AttachmentOutcome::kUncommitted:
      return "physically applied but not committed";
    case AttachmentOutcome::kIndeterminate:
      return "outcome unknown";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
