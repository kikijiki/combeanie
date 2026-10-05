// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>

#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

struct PreGraspSceneAuthority
{
  std::string planning_frame;
  std::string projector_epoch;
  restocker_world_state::Revision applied_revision{0U};
  std::uint64_t verification_epoch{0U};
  // Fixed for the entire planned trajectory, including all controller slices. No-diff
  // verification/robot motion preserves it; a verified diff or lease release invalidates it.
  std::uint64_t scene_content_generation{0U};
  // Zero for an ordinary applied projection. Non-zero only when a live, validated capability
  // authorizes this segment to plan against the projector's intentionally frozen held lease.
  std::uint64_t lease_id{0U};
  rclcpp::Time observed_at{std::int64_t{0}, RCL_ROS_TIME};

  [[nodiscard]] bool operator==(const PreGraspSceneAuthority &) const = default;
};

// Public, non-secret facts returned by ValidatePlanningSceneLease for the capability supplied by
// one motion goal. The token itself deliberately never enters the pure authority gate.
struct PlanningSceneLeaseAuthority
{
  std::uint64_t lease_id{0U};
  restocker_world_state::Revision granted_applied_revision{0U};
  std::uint64_t verification_epoch{0U};

  [[nodiscard]] bool operator==(const PlanningSceneLeaseAuthority &) const = default;
};

struct PreGraspAttachmentAuthority
{
  std::string simulator_epoch;
  std::uint64_t sequence{0U};
  rclcpp::Time observed_at{std::int64_t{0}, RCL_ROS_TIME};

  [[nodiscard]] bool operator==(const PreGraspAttachmentAuthority &) const = default;
};

struct PreGraspPlanningAuthority
{
  PreGraspSceneAuthority scene;
  PreGraspAttachmentAuthority attachment;

  [[nodiscard]] bool operator==(const PreGraspPlanningAuthority &) const = default;
};

struct PreGraspPlanningAuthorityConfig
{
  std::string planning_frame{"world"};
  std::chrono::nanoseconds maximum_scene_age{std::chrono::milliseconds(500)};
  std::chrono::nanoseconds maximum_attachment_age{std::chrono::milliseconds(500)};
  std::chrono::nanoseconds maximum_future_skew{std::chrono::milliseconds(50)};
};

enum class PreGraspAuthorityDecision : std::uint8_t
{
  kAccepted,
  kWait,
  kRejected,
};

enum class PreGraspAuthorityErrorCode : std::uint8_t
{
  kNone,
  kInvalidConfiguration,
  kInvalidTimestamp,
  kStaleEvidence,
  kFutureEvidence,
  kSceneSynchronizing,
  // A projector condition that resolves without anyone intervening: it is starting, it is
  // waiting on a dependency, or its obstacle evidence is inside a gap in the stream rather
  // than past the end of one. Carried as a wait, bounded by the caller's settle budget.
  kSceneProjectorSettling,
  kSceneStateInvalid,
  kSceneLeaseActive,
  kSceneLeaseCapabilityInvalid,
  kSceneRevisionInvalid,
  kProjectorEpochChanged,
  kSceneContentChanged,
  kAttachmentStateInvalid,
  kAttachmentEpochChanged,
  kAttachmentSequenceChanged,
};

struct PreGraspAuthorityResult
{
  PreGraspAuthorityDecision decision{PreGraspAuthorityDecision::kRejected};
  PreGraspAuthorityErrorCode error{PreGraspAuthorityErrorCode::kInvalidConfiguration};
  std::optional<PreGraspPlanningAuthority> authority;
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return decision == PreGraspAuthorityDecision::kAccepted && authority.has_value();
  }
};

// The planning-scene half of the gate below, on its own.
//
// It exists separately because it is the half that applies to every motion segment. The
// attachment half asserts a cleanly detached gripper, which is true before a grasp and false
// during a transfer; the scene half asserts only that the geometry the plan was made against is
// still the geometry in the scene, which every segment needs and no segment outgrows.
struct PlanningSceneAuthorityResult
{
  PreGraspAuthorityDecision decision{PreGraspAuthorityDecision::kRejected};
  PreGraspAuthorityErrorCode error{PreGraspAuthorityErrorCode::kInvalidConfiguration};
  std::optional<PreGraspSceneAuthority> authority;
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return decision == PreGraspAuthorityDecision::kAccepted && authority.has_value();
  }
};

// Pure gate over one immutable planning-scene status. A baseline is omitted when the authority is
// first established and supplied afterwards, so that a scene whose collision content changed,
// which is what an obstacle appearing or a projection going stale does, cannot be executed
// against a plan that was made before it.
[[nodiscard]] PlanningSceneAuthorityResult evaluate_planning_scene_authority(
  const restocker_interfaces::msg::PlanningSceneProjectionStatus & scene,
  restocker_world_state::Revision required_world_revision, const rclcpp::Time & now,
  const PreGraspPlanningAuthorityConfig & config,
  const std::optional<PreGraspSceneAuthority> & baseline = std::nullopt,
  const std::optional<PlanningSceneLeaseAuthority> & lease = std::nullopt);

// Pure gate over immutable ROS observations. A baseline is omitted before submission and supplied
// after MoveGroup returns so scene content and physical-attachment authority cannot change while
// a plan is in flight.
[[nodiscard]] PreGraspAuthorityResult evaluate_pregrasp_planning_authority(
  const restocker_interfaces::msg::PlanningSceneProjectionStatus & scene,
  const restocker_interfaces::msg::SimulationAttachmentState & attachment,
  restocker_world_state::Revision required_world_revision, const rclcpp::Time & now,
  const PreGraspPlanningAuthorityConfig & config,
  const std::optional<PreGraspPlanningAuthority> & baseline = std::nullopt);

[[nodiscard]] const char * to_string(PreGraspAuthorityDecision decision) noexcept;
[[nodiscard]] const char * to_string(PreGraspAuthorityErrorCode code) noexcept;

}  // namespace restocker_task_executor
