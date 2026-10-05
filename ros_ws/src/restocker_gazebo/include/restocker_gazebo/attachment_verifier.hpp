// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

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
};

struct AttachmentVerificationObservation
{
  bool exact_joint_observed{false};
  bool entity_identity_matches{false};
  Eigen::Isometry3d parent_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_child{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d relative_linear_velocity_in_parent{Eigen::Vector3d::Zero()};
  Eigen::Vector3d relative_angular_velocity_in_parent{Eigen::Vector3d::Zero()};
  std::uint64_t simulator_iteration{0};
  std::int64_t simulation_time_ns{0};
};

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
  explicit AttachmentMutationVerifier(AttachmentVerificationConfig config);

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
  std::string operation_id_;
  AttachmentCommand command_{AttachmentCommand::kUnset};
  Eigen::Isometry3d applied_parent_from_child_{Eigen::Isometry3d::Identity()};
  std::uint64_t mutation_iteration_{0};
  std::int64_t mutation_time_ns_{0};
  std::size_t consecutive_stable_ticks_{0};
};

[[nodiscard]] AttachmentStatus validate_attached_watchdog(
  const Eigen::Isometry3d & applied_parent_from_child,
  const AttachmentVerificationConfig & config,
  const AttachmentVerificationObservation & observation);

}  // namespace restocker_gazebo
