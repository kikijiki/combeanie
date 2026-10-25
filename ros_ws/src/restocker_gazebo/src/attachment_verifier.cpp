// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_verifier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
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

// Spec M10 section 11 D5 amendment, as corrected. A friction grip under DART has no torsional
// friction (F13), so a side-held product swings about the line through its two loaded pad
// contacts after any carry acceleration. What the grip must not do is slip: the product's material
// point at a pad moving away from that pad. So slip is measured at each pad, relative to the pad,
// which also leaves out both fingers shifting together (no common-mode stiffness between two
// force-saturated servos); the product's origin may swing within its own allowance about the
// measured contact line, and rotation off that line stays bounded.
[[nodiscard]] AttachmentStatus validate_held_physical(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationConfig & config,
  const AttachmentVerificationObservation & observation)
{
  if (!observation.pads || !finite_positive(config.maximum_swing_translation_m) ||
    config.maximum_swing_translation_m < config.maximum_translation_drift_m ||
    !finite_positive(config.maximum_off_line_rotation_rad))
  {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      "physical hold watchdog has no pad contacts or no valid swing and rotation bounds");
  }
  static constexpr std::array<const char *, 2> kPadNames{"left", "right"};
  std::array<Eigen::Vector3d, 2> pad_points;
  std::array<Eigen::Vector3d, 2> reference_points;
  std::ostringstream measured;
  for (std::size_t index = 0; index < 2; ++index) {
    const HeldPadObservation & pad = (*observation.pads)[index];
    pad_points[index] = pad.parent_from_finger * pad.contact_in_finger;
    reference_points[index] = applied_parent_from_child * pad.contact_in_child;
    const Eigen::Vector3d material = observation.parent_from_child * pad.contact_in_child;
    if (!pad_points[index].allFinite() || !material.allFinite() ||
      !std::isfinite(pad.cumulative_slip_m))
    {
      return status(
        AttachmentStatusCode::kExternalInconsistency, "held pad contact evidence is not finite");
    }
    // Cumulative over the hold (the boundary accumulates it), never less than the slip now.
    const double slip = std::max(
      pad_plane_slip(material - pad_points[index], pad.parent_from_finger).norm(),
      pad.cumulative_slip_m);
    if (slip > config.maximum_translation_drift_m) {
      measured << "held object slipped at the " << kPadNames[index] << " pad: " << slip << " of "
               << config.maximum_translation_drift_m << " m cumulative";
      return status(AttachmentStatusCode::kExternalInconsistency, measured.str());
    }
  }
  const Eigen::Vector3d line = pad_points[0] - pad_points[1];
  if (line.norm() < 1.0e-3) {
    return status(
      AttachmentStatusCode::kExternalInconsistency, "held pad contacts do not define a line");
  }
  const HeldGripMeasure grip = measure_held_grip(applied_parent_from_child, observation);
  if (grip.swing_m > config.maximum_swing_translation_m) {
    measured << "held object swing exceeded the allowance: " << grip.swing_m << " of "
             << config.maximum_swing_translation_m << " m";
    return status(AttachmentStatusCode::kExternalInconsistency, measured.str());
  }
  if (grip.tilt_rad > config.maximum_off_line_rotation_rad) {
    measured << "held object tilted off the jaw axis: " << grip.tilt_rad << " of "
             << config.maximum_off_line_rotation_rad << " rad";
    return status(AttachmentStatusCode::kExternalInconsistency, measured.str());
  }
  return status(AttachmentStatusCode::kAttached, "held-object watchdog evidence is valid");
}

}  // namespace

HeldGripMeasure measure_held_grip(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationObservation & observation)
{
  HeldGripMeasure result;
  if (!observation.pads) {
    return result;
  }
  const auto & pads = *observation.pads;
  const Eigen::Vector3d middle_now = 0.5 * (pads[0].parent_from_finger * pads[0].contact_in_finger +
    pads[1].parent_from_finger * pads[1].contact_in_finger);
  const Eigen::Vector3d middle_then = 0.5 * (applied_parent_from_child * pads[0].contact_in_child +
    applied_parent_from_child * pads[1].contact_in_child);
  result.swing_m = ((observation.parent_from_child.translation() - middle_now) -
    (applied_parent_from_child.translation() - middle_then)).norm();

  // The free axis is the jaw axis, the common normal of the two flat pads (the finger frame's y):
  // the only axis a planar friction contact leaves free in DART (no torsional friction), so it is
  // the axis the product swings about. The line through the two contact centroids is only an
  // estimate of it -- the centroids sit wherever the load falls among the pad rows -- and
  // measured 0.07 rad off it, which made a 0.4 rad pendulum swing read as 0.03 rad of "tilt".
  const Eigen::Vector3d axis = pads[0].parent_from_finger.linear().col(1).normalized();
  const Eigen::AngleAxisd rotation(
    observation.parent_from_child.linear() * applied_parent_from_child.linear().transpose());
  Eigen::Vector3d rotation_vector = rotation.axis() * rotation.angle();
  // Every product is an upright cylinder: a turn about its own axis moves nothing (the pose
  // backend does not even estimate it), so it is free, as the swing about the jaw axis is. What
  // is bounded is the lean off the jaw axis, by angle only: a stick-slip pop spikes the rate for
  // a tick while the angle stays small, and it is the angle that matters (spec D5 amendment).
  const Eigen::Vector3d product_axis = applied_parent_from_child.linear().col(2).normalized();
  rotation_vector -= rotation_vector.dot(product_axis) * product_axis;
  result.tilt_rad = (rotation_vector - rotation_vector.dot(axis) * axis).norm();
  return result;
}

AttachmentMutationVerifier::AttachmentMutationVerifier(
  AttachmentVerificationConfig config,
  AttachmentHoldMode hold_mode)
: config_(config), hold_mode_(hold_mode)
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
  const bool is_attach = command_ == AttachmentCommand::kAttach;
  // Physical attach creates nothing, so its terminal evidence is the *absence* of the joint plus
  // a stable grip; weld attach created one and must show it (spec 11.4).
  const bool expect_joint = is_attach && hold_mode_ == AttachmentHoldMode::kWeld;
  if (expect_joint && !observation.exact_joint_observed) {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kOutcomeUnknown,
      "owned attachment joint disappeared after creation");
  }
  if (is_attach && !expect_joint && observation.exact_joint_observed) {
    return result(
      AttachmentVerificationDisposition::kOutcomeUnknown,
      AttachmentStatusCode::kOutcomeUnknown,
      "an owned attachment joint exists although hold mode is physical");
  }
  if (is_attach) {
    const auto transform_status = validate_attached_transform(
      applied_parent_from_child_, config_, observation);
    if (transform_status.code != AttachmentStatusCode::kPending) {
      return result(
        AttachmentVerificationDisposition::kOutcomeUnknown,
        AttachmentStatusCode::kOutcomeUnknown,
        transform_status.detail);
    }
  }
  if (!is_attach && observation.exact_joint_observed) {
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
  if (is_attach && !velocity_stable) {
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
    is_attach ? AttachmentStatusCode::kAttached : AttachmentStatusCode::kDetached,
    is_attach ? "physical attachment verified" : "physical detachment verified",
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
  const AttachmentVerificationObservation & observation,
  AttachmentHoldMode hold_mode)
{
  const bool expect_joint = hold_mode == AttachmentHoldMode::kWeld;
  if (observation.exact_joint_observed != expect_joint) {
    return status(
      AttachmentStatusCode::kExternalInconsistency,
      expect_joint ?
      "owned attachment joint is absent while the object is held" :
      "an owned attachment joint exists although hold mode is physical");
  }
  const auto coherent = validate_common_observation(observation);
  if (coherent.code != AttachmentStatusCode::kPending) {
    return coherent;
  }
  if (hold_mode == AttachmentHoldMode::kPhysical) {
    return validate_held_physical(applied_parent_from_child, config, observation);
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

Eigen::Isometry3d without_spin_about_product_axis(
  const Eigen::Isometry3d & applied_parent_from_child,
  const Eigen::Isometry3d & observed_parent_from_child)
{
  const Eigen::Vector3d axis = applied_parent_from_child.linear().col(2).normalized();
  const Eigen::Quaterniond relative(
    observed_parent_from_child.linear() * applied_parent_from_child.linear().transpose());
  const double twist =
    2.0 * std::atan2(relative.vec().dot(axis), relative.w());
  Eigen::Isometry3d result = observed_parent_from_child;
  result.linear() = Eigen::AngleAxisd(-twist, axis).toRotationMatrix() * result.linear();
  return result;
}

Eigen::Vector3d pad_plane_slip(
  const Eigen::Vector3d & displacement, const Eigen::Isometry3d & parent_from_finger)
{
  const Eigen::Vector3d normal = parent_from_finger.linear().col(1);
  return displacement - normal * normal.dot(displacement);
}

void accumulate_pad_slip(
  PadSlipAccumulator & accumulator, const Eigen::Vector3d & slip, double count_step_m)
{
  if (!slip.allFinite()) {
    return;
  }
  accumulator.peak_excursion_m = std::max(accumulator.peak_excursion_m, slip.norm());
  const double step = (slip - accumulator.last_counted).norm();
  if (step >= count_step_m) {
    accumulator.path_m += step;
    accumulator.last_counted = slip;
  }
}

double cumulative_pad_slip(const PadSlipAccumulator & accumulator)
{
  return accumulator.peak_excursion_m;
}

}  // namespace restocker_gazebo
