// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{

struct AttachmentVerificationConfig
{
  std::size_t required_consecutive_ticks{0};
  std::size_t maximum_pending_ticks{0};
  double maximum_translation_drift_m{0.0};
  double maximum_rotation_drift_rad{0.0};
  double maximum_relative_linear_velocity_mps{0.0};
  double maximum_relative_angular_velocity_radps{0.0};
  // Physical hold only (spec M10 section 11 D5 amendment): how far the product's origin may
  // swing about the jaw axis (the pads' common normal), and, by angle, how far it may lean off it.
  double maximum_swing_translation_m{0.0};
  double maximum_off_line_rotation_rad{0.0};
};

// One finger's contact with a physically held product (spec M10 section 11 D5 amendment, as
// corrected): where the grip touched when it was verified, as a point in the finger's frame and the
// same material point in the product's frame, plus the finger's pose now and the pad's slip
// accumulated over the hold. Slip at the pad is the distance between the two points.
struct HeldPadObservation
{
  Eigen::Vector3d contact_in_finger{Eigen::Vector3d::Zero()};
  Eigen::Vector3d contact_in_child{Eigen::Vector3d::Zero()};
  Eigen::Isometry3d parent_from_finger{Eigen::Isometry3d::Identity()};
  // The pad's slip accumulated over the whole hold (see PadSlipAccumulator).
  double cumulative_slip_m{0.0};
};

// Spec M10 section 11 D5 amendment (2026-10-17): slip at a pad is the peak excursion of the slip
// vector from its attach-time position (zero at the attach-time contact), i.e. how far the
// product has net-moved relative to the pad. The path the slip point travelled (counted in steps
// of at least count_step_m so per-tick jitter is not summed) is kept as a receipt but is not
// part of the verdict: wobble and swing about the grasp move the point back and forth without
// moving the product out of registration, and measured on a held can the path ran 2.3x the
// excursion within nine seconds of carry.
struct PadSlipAccumulator
{
  Eigen::Vector3d last_counted{Eigen::Vector3d::Zero()};
  double path_m{0.0};
  double peak_excursion_m{0.0};
};

// The part of a displacement at a pad that is slip: its component in the pad plane. Along the pad
// normal (the finger frame's y, the jaw axis) the finger-fixed reference point moves as the
// compliant pad row compresses and relaxes with the squeeze (40 N is 0.36 mm), which is not
// sliding; the squeeze is bounded by the aperture, contact-band and force checks instead.
[[nodiscard]] Eigen::Vector3d pad_plane_slip(
  const Eigen::Vector3d & displacement, const Eigen::Isometry3d & parent_from_finger);

void accumulate_pad_slip(
  PadSlipAccumulator & accumulator, const Eigen::Vector3d & slip, double count_step_m);

[[nodiscard]] double cumulative_pad_slip(const PadSlipAccumulator & accumulator);

struct AttachmentVerificationObservation
{
  bool exact_joint_observed{false};
  bool entity_identity_matches{false};
  Eigen::Isometry3d parent_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d relative_linear_velocity_in_parent{Eigen::Vector3d::Zero()};
  Eigen::Vector3d relative_angular_velocity_in_parent{Eigen::Vector3d::Zero()};
  // Physical hold only: both fingers' contacts, left then right. Absent means unobserved.
  std::optional<std::array<HeldPadObservation, 2>> pads;
  std::uint64_t simulator_iteration{0};
  std::int64_t simulation_time_ns{0};
};

// The observed product pose with its turn about its own axis (the child frame's z) removed. Every
// product is an upright cylinder, so that turn changes no pose that matters, but it drags a
// material point fixed on the product along the circumference (r * angle, 1 mm per 0.03 rad on a
// can) and would be counted as slip at the pads. Swing-twist: the twist about the axis is taken
// out exactly; translation and tilt are untouched.
[[nodiscard]] Eigen::Isometry3d without_spin_about_product_axis(
  const Eigen::Isometry3d & applied_parent_from_child,
  const Eigen::Isometry3d & observed_parent_from_child);

// How far a physically held product has moved in the grip since it was verified, in the two
// quantities the physical watchdog bounds besides slip: the swing of its origin about the jaw
// axis (a pendulum, spec M10 11 D5 amendment point 2) and its lean off that axis, by angle.
struct HeldGripMeasure
{
  double swing_m{0.0};
  double tilt_rad{0.0};
};

[[nodiscard]] HeldGripMeasure measure_held_grip(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationObservation & observation);

enum class AttachmentVerificationDisposition : std::uint8_t
{
  kPending = 0,
  kSucceeded = 1,
  kOutcomeUnknown = 2,
};

struct AttachmentVerificationResult
{
  AttachmentVerificationDisposition disposition{AttachmentVerificationDisposition::kPending};
  AttachmentStatus status;
  std::optional<AttachmentEvidence> evidence;
};

class AttachmentMutationVerifier
{
public:
  // hold_mode decides the terminal evidence an attach may present (spec 11.4): weld must show
  // the owned joint it created and hold the product against it; physical must show that no joint
  // exists and that the grip is stable, because nothing was created.
  explicit AttachmentMutationVerifier(
    AttachmentVerificationConfig config,
    AttachmentHoldMode hold_mode = AttachmentHoldMode::kPhysical);

  void begin(
    std::string operation_id,
    AttachmentCommand command,
    const Eigen::Isometry3d & applied_parent_from_child,
    std::uint64_t mutation_iteration,
    std::int64_t mutation_time_ns);
  [[nodiscard]] AttachmentVerificationResult observe(
    const AttachmentVerificationObservation & observation);
  void reset() noexcept;

  [[nodiscard]] bool active() const noexcept;
  [[nodiscard]] const std::string & operation_id() const noexcept;

private:
  AttachmentVerificationConfig config_;
  AttachmentHoldMode hold_mode_{AttachmentHoldMode::kPhysical};
  std::string operation_id_;
  AttachmentCommand command_{AttachmentCommand::kUnset};
  Eigen::Isometry3d applied_parent_from_child_{Eigen::Isometry3d::Identity()};
  std::uint64_t mutation_iteration_{0};
  std::int64_t mutation_time_ns_{0};
  std::size_t consecutive_stable_ticks_{0};
};

// hold_mode decides whether a held object is expected to have an owned joint: weld does (it
// created one), physical does not (spec 11.4) — so the watchdog's first question, "is the thing
// that should be there there?", is asked of the right mode.
[[nodiscard]] AttachmentStatus validate_attached_watchdog(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationConfig & config,
  const AttachmentVerificationObservation & observation,
  AttachmentHoldMode hold_mode);

}  // namespace restocker_gazebo
