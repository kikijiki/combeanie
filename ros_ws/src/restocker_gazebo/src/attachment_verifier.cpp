// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_verifier.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace restocker_gazebo
{
namespace
{

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool finite_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] double rotation_distance(
  const Eigen::Isometry3d & left, const Eigen::Isometry3d & right)
{
  return Eigen::Quaterniond(left.linear()).angularDistance(Eigen::Quaterniond(right.linear()));
}

[[nodiscard]] AttachmentStatus status(AttachmentStatusCode code, std::string detail)
{
  return AttachmentStatus{code, std::move(detail)};
}

[[nodiscard]] AttachmentVerificationResult result(
  AttachmentVerificationDisposition disposition,
  AttachmentStatusCode code,
  std::string detail,
  std::optional<AttachmentEvidence> evidence = std::nullopt)
{
  return AttachmentVerificationResult{
    disposition, status(code, std::move(detail)), std::move(evidence)};
}

[[nodiscard]] CanonicalPose pose(const Eigen::Isometry3d & transform)
{
  const Eigen::Quaterniond rotation(transform.linear());
  const auto canonical = canonicalize_pose(
    {transform.translation().x(), transform.translation().y(), transform.translation().z()},
    {rotation.x(), rotation.y(), rotation.z(), rotation.w()});
  if (!canonical) {
    throw std::logic_error("finite rigid transform could not be canonicalized");
  }
  return *canonical;
}

[[nodiscard]] AttachmentEvidence evidence(
  bool joint_observed, const AttachmentVerificationObservation & observation)
{
  return AttachmentEvidence{
    joint_observed,
    pose(observation.parent_from_child),
    pose(observation.world_from_child),
    {
      observation.relative_linear_velocity_in_parent.x(),
      observation.relative_linear_velocity_in_parent.y(),
      observation.relative_linear_velocity_in_parent.z(),
      observation.relative_angular_velocity_in_parent.x(),
      observation.relative_angular_velocity_in_parent.y(),
      observation.relative_angular_velocity_in_parent.z(),
    },
    observation.simulator_iteration,
    observation.simulation_time_ns};
}

[[nodiscard]] AttachmentStatus validate_common_observation(
  const AttachmentVerificationObservation & observation)
{
  if (!observation.entity_identity_matches) {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "attachment entity identity changed during physical verification");
  }
  if (!finite_transform(observation.parent_from_child) ||
    !finite_transform(observation.world_from_child) ||
    !observation.relative_linear_velocity_in_parent.allFinite() ||
    !observation.relative_angular_velocity_in_parent.allFinite() ||
    observation.simulation_time_ns < 0)
  {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "attachment verification evidence is non-finite or has invalid simulation time");
  }
  return status(AttachmentStatusCode::kPending, "attachment evidence is geometrically coherent");
}

[[nodiscard]] AttachmentStatus validate_attached_transform(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationConfig & config,
  const AttachmentVerificationObservation & observation)
{
  const double translation_drift =
    (applied_parent_from_child.translation() - observation.parent_from_child.translation()).norm();
  const double angular_drift = rotation_distance(
    applied_parent_from_child, observation.parent_from_child);
  if (translation_drift > config.maximum_translation_drift_m ||
    angular_drift > config.maximum_rotation_drift_rad)
  {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "attachment relative transform drift exceeded the configured watchdog limit");
  }
  return status(AttachmentStatusCode::kPending, "attached transform remains within tolerance");
}

}  // namespace

AttachmentMutationVerifier::AttachmentMutationVerifier(AttachmentVerificationConfig config)
: config_(config)
{
  if (config.required_consecutive_ticks == 0 ||
    config.maximum_pending_ticks <= config.required_consecutive_ticks ||
    !finite_positive(config.maximum_translation_drift_m) ||
    !finite_positive(config.maximum_rotation_drift_rad) ||
    config.maximum_rotation_drift_rad > std::acos(-1.0) ||
    !finite_positive(config.maximum_relative_linear_velocity_mps) ||
    !finite_positive(config.maximum_relative_angular_velocity_radps))
  {
    throw std::invalid_argument("attachment verification configuration is invalid");
  }
}

void AttachmentMutationVerifier::begin(
  std::string operation_id,
  AttachmentCommand command,
  const Eigen::Isometry3d & applied_parent_from_child,
  std::uint64_t mutation_iteration,
  std::int64_t mutation_time_ns)
{
  if (active()) {
    throw std::logic_error("attachment verifier already has an active mutation");
  }
  if (operation_id.empty() ||
    (command != AttachmentCommand::kAttach && command != AttachmentCommand::kDetach) ||
    !finite_transform(applied_parent_from_child) || mutation_time_ns < 0)
  {
    throw std::invalid_argument("attachment verification start evidence is invalid");
  }
  operation_id_ = std::move(operation_id);
  command_ = command;
  applied_parent_from_child_ = applied_parent_from_child;
  mutation_iteration_ = mutation_iteration;
  mutation_time_ns_ = mutation_time_ns;
  consecutive_stable_ticks_ = 0;
}

AttachmentVerificationResult AttachmentMutationVerifier::observe(
  const AttachmentVerificationObservation & observation)
{
  if (!active()) {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kStateMismatch,
      "attachment verifier has no active mutation");
  }
  if (observation.simulator_iteration < mutation_iteration_ ||
    observation.simulation_time_ns < mutation_time_ns_)
  {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kOutcomeUnknown,
      "simulation iteration or time regressed during physical verification");
  }
  if (observation.simulator_iteration == mutation_iteration_) {
    return result(
      AttachmentVerificationDisposition::kPending,
      AttachmentStatusCode::kPending,
      "waiting for the first post-mutation simulation iteration");
  }
  const auto coherent = validate_common_observation(observation);
  if (coherent.code != AttachmentStatusCode::kPending) {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kOutcomeUnknown,
      coherent.detail);
  }

  const std::uint64_t elapsed_ticks = observation.simulator_iteration - mutation_iteration_;
  const bool expect_joint = command_ == AttachmentCommand::kAttach;
  if (expect_joint && !observation.exact_joint_observed) {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kOutcomeUnknown,
      "owned attachment joint disappeared after creation");
  }
  if (expect_joint) {
    const auto transform_status = validate_attached_transform(
      applied_parent_from_child_, config_, observation);
    if (transform_status.code != AttachmentStatusCode::kPending) {
      return result(
        AttachmentVerificationDisposition::kOutcomeUnknown,
        AttachmentStatusCode::kOutcomeUnknown,
        transform_status.detail);
    }
  }
  if (!expect_joint && observation.exact_joint_observed) {
    consecutive_stable_ticks_ = 0;
    if (elapsed_ticks >= config_.maximum_pending_ticks) {
      return result(
        AttachmentVerificationDisposition::kOutcomeUnknown,
        AttachmentStatusCode::kOutcomeUnknown,
        "owned attachment joint removal was not observed before the deadline");
    }
    return result(
      AttachmentVerificationDisposition::kPending,
      AttachmentStatusCode::kPending,
      "waiting for owned attachment joint removal");
  }

  const bool velocity_stable =
    observation.relative_linear_velocity_in_parent.norm() <=
    config_.maximum_relative_linear_velocity_mps &&
    observation.relative_angular_velocity_in_parent.norm() <=
    config_.maximum_relative_angular_velocity_radps;
  if (expect_joint && !velocity_stable) {
    consecutive_stable_ticks_ = 0;
    if (elapsed_ticks >= config_.maximum_pending_ticks) {
      return result(
        AttachmentVerificationDisposition::kOutcomeUnknown,
        AttachmentStatusCode::kOutcomeUnknown,
        "attached child did not settle before the verification deadline");
    }
    return result(
      AttachmentVerificationDisposition::kPending,
      AttachmentStatusCode::kPending,
      "waiting for attached child motion to settle");
  }

  ++consecutive_stable_ticks_;
  if (consecutive_stable_ticks_ < config_.required_consecutive_ticks) {
    return result(
      AttachmentVerificationDisposition::kPending,
      AttachmentStatusCode::kPending,
      "collecting consecutive post-physics attachment evidence");
  }
  return result(
    AttachmentVerificationDisposition::kSucceeded,
    expect_joint ? AttachmentStatusCode::kAttached : AttachmentStatusCode::kDetached,
    expect_joint ? "physical attachment verified" : "physical detachment verified",
    evidence(expect_joint, observation));
}

void AttachmentMutationVerifier::reset() noexcept
{
  operation_id_.clear();
  command_ = AttachmentCommand::kUnset;
  applied_parent_from_child_ = Eigen::Isometry3d::Identity();
  mutation_iteration_ = 0;
  mutation_time_ns_ = 0;
  consecutive_stable_ticks_ = 0;
}

bool AttachmentMutationVerifier::active() const noexcept
{
  return !operation_id_.empty();
}

const std::string & AttachmentMutationVerifier::operation_id() const noexcept
{
  return operation_id_;
}

AttachmentStatus validate_attached_watchdog(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationConfig & config,
  const AttachmentVerificationObservation & observation)
{
  if (!observation.exact_joint_observed) {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "owned attachment joint is absent while the object is held");
  }
  const auto coherent = validate_common_observation(observation);
  if (coherent.code != AttachmentStatusCode::kPending) {
    return coherent;
  }
  const auto transform_status = validate_attached_transform(
    applied_parent_from_child, config, observation);
  if (transform_status.code != AttachmentStatusCode::kPending) {
    return transform_status;
  }
  if (observation.relative_linear_velocity_in_parent.norm() >
    config.maximum_relative_linear_velocity_mps ||
    observation.relative_angular_velocity_in_parent.norm() >
    config.maximum_relative_angular_velocity_radps)
  {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "held object relative velocity exceeded the watchdog limit");
  }
  return status(AttachmentStatusCode::kAttached, "held-object watchdog evidence is valid");
}

}  // namespace restocker_gazebo
