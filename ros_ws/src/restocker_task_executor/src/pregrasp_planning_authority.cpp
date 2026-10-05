// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/pregrasp_planning_authority.hpp"

#include <exception>
#include <utility>

#include <restocker_interfaces/msg/planning_scene_lease.hpp>

namespace restocker_task_executor
{
namespace
{

using SceneStatus = restocker_interfaces::msg::PlanningSceneProjectionStatus;
using SceneLease = restocker_interfaces::msg::PlanningSceneLease;
using AttachmentState = restocker_interfaces::msg::SimulationAttachmentState;

[[nodiscard]] PreGraspAuthorityResult rejected(
  PreGraspAuthorityErrorCode error, std::string detail)
{
  return {PreGraspAuthorityDecision::kRejected, error, std::nullopt, std::move(detail)};
}

[[nodiscard]] PreGraspAuthorityResult wait(
  PreGraspAuthorityErrorCode error, std::string detail)
{
  return {PreGraspAuthorityDecision::kWait, error, std::nullopt, std::move(detail)};
}

enum class TimestampDisposition : std::uint8_t
{
  kFresh,
  kStale,
  kInvalid,
  kFuture,
};

struct TimestampResult
{
  TimestampDisposition disposition{TimestampDisposition::kInvalid};
  std::optional<rclcpp::Time> timestamp;
};

template<typename Stamp>
[[nodiscard]] TimestampResult check_timestamp(
  const Stamp & stamp, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_age,
  std::chrono::nanoseconds maximum_future_skew) noexcept
{
  if (now.get_clock_type() != RCL_ROS_TIME || now.nanoseconds() <= 0 ||
    maximum_age < std::chrono::nanoseconds::zero() ||
    maximum_future_skew < std::chrono::nanoseconds::zero())
  {
    return {};
  }
  try {
    const rclcpp::Time observed(stamp, RCL_ROS_TIME);
    if (observed.nanoseconds() <= 0) {
      return {};
    }
    if (observed > now) {
      const auto future = std::chrono::nanoseconds(
        observed.nanoseconds() - now.nanoseconds());
      return {
        future > maximum_future_skew ? TimestampDisposition::kFuture :
        TimestampDisposition::kFresh,
        observed};
    }
    const auto age = std::chrono::nanoseconds(now.nanoseconds() - observed.nanoseconds());
    return {
      age > maximum_age ? TimestampDisposition::kStale : TimestampDisposition::kFresh,
      observed};
  } catch (const std::exception &) {
    return {};
  }
}

[[nodiscard]] PreGraspAuthorityResult timestamp_failure(
  TimestampResult timestamp, std::string label)
{
  switch (timestamp.disposition) {
    case TimestampDisposition::kFresh:
      break;
    case TimestampDisposition::kStale:
      return wait(
        PreGraspAuthorityErrorCode::kStaleEvidence,
        std::move(label) + " observation is stale");
    case TimestampDisposition::kFuture:
      return rejected(
        PreGraspAuthorityErrorCode::kFutureEvidence,
        std::move(label) + " observation exceeds future-skew tolerance");
    case TimestampDisposition::kInvalid:
      return rejected(
        PreGraspAuthorityErrorCode::kInvalidTimestamp,
        std::move(label) + " observation timestamp is invalid");
  }
  return rejected(
    PreGraspAuthorityErrorCode::kInvalidTimestamp,
    std::move(label) + " timestamp failure requested for fresh evidence");
}

[[nodiscard]] bool valid_config(const PreGraspPlanningAuthorityConfig & config) noexcept
{
  return !config.planning_frame.empty() &&
         config.maximum_scene_age >= std::chrono::nanoseconds::zero() &&
         config.maximum_attachment_age >= std::chrono::nanoseconds::zero() &&
         config.maximum_future_skew >= std::chrono::nanoseconds::zero();
}

// How a projector condition resolves: whether the scene is worth waiting for or cannot be vouched
// for.
//
// The projector reconciles on a 0.5 s timer and republishes status every cycle, so for each non-
// certified condition the question is whether the next cycle can clear it unaided. Where it can,
// refusing on first sight converts a gap into a fault: a rejection while a product is held latches
// `motion inhibited; operator required`, which ends the run.
//
// Settling is not permission to move. The caller polls until the condition clears or its budget
// runs out, and a budget that runs out rejects exactly as this gate does today. Retained obstacle
// geometry stays retained throughout.
enum class ProjectorCondition : std::uint8_t
{
  kCertified,
  kSettling,
  kFailed,
};

[[nodiscard]] ProjectorCondition classify_projector(const SceneStatus & scene) noexcept
{
  switch (scene.error_code) {
    case SceneStatus::ERROR_NONE:
      break;

    // Waiting on a dependency, not a verdict about the world. Each of these is re-tested at the
    // top of the next reconcile cycle (a service readiness check or a transform lookup) and
    // clears the moment the dependency answers. None of them says anything about the geometry.
    case SceneStatus::ERROR_WORLD_STATE_UNAVAILABLE:
    case SceneStatus::ERROR_PLANNING_SCENE_UNAVAILABLE:
    case SceneStatus::ERROR_TRANSFORM_UNAVAILABLE:
      return ProjectorCondition::kSettling;

    // A gap in the depth stream, not the end of one. Over 1008 consecutive observations the sim-
    // time gaps run p50 0.166 s, p95 0.498 s, max 2.157 s against an obstacle_max_age_sec of 2.0 s,
    // so the tail crosses the window by ~0.16 s and the projector clears on 0.5 s cycles: the
    // degraded window a live stream can produce is about 1.2 s. A settle budget of seconds outlasts
    // every such gap while still expiring on a stream that has stopped. Widening
    // obstacle_max_age_sec would not draw that line: a window wider than the tail hides a dead
    // stream. The geometry stays in the scene either way and nothing executes until the projector
    // certifies again.
    case SceneStatus::ERROR_OBSTACLE_EVIDENCE_STALE:
      return ProjectorCondition::kSettling;

    // The same shape on the product side, and the one that reaches a retreat: a released product
    // must be observed again before the projection can be certified, and under load the observation
    // stream can fall behind its 0.5 s window for a cycle or two. A starved publisher catches up
    // well inside the budget, whereas a product the gripper is standing over stays unobserved until
    // something moves and still ends in a rejection. Waiting cannot invent evidence; it only stops
    // a gap that would have closed on its own from ending the run.
    case SceneStatus::ERROR_OBSERVATION_EVIDENCE_STALE:
      return ProjectorCondition::kSettling;

    // The projector has given up rather than fallen behind. A bad configuration and an unusable
    // snapshot fail identically on every retry; a rejected apply or failed verification means the
    // scene is not what was asked for; an unknown apply outcome is the strongest, because the
    // scene's contents are indeterminate. Waiting only delays the same answer.
    case SceneStatus::ERROR_CONFIGURATION:
    case SceneStatus::ERROR_INVALID_SNAPSHOT:
    case SceneStatus::ERROR_APPLY_REJECTED:
    case SceneStatus::ERROR_VERIFICATION_FAILED:
    case SceneStatus::ERROR_APPLY_OUTCOME_UNKNOWN:
    default:
      return ProjectorCondition::kFailed;
  }

  // No error reported. The projector publishes STATE_STARTING once, before its first cycle has
  // run, and its identity and revision fields are legitimately unpopulated until then.
  if (scene.state == SceneStatus::STATE_STARTING) {
    return ProjectorCondition::kSettling;
  }
  return ProjectorCondition::kCertified;
}

// Narrows a rejection or wait to the scene-only result. Only the outcome carries over; the
// authority is never populated on a non-accepting result, so nothing is lost.
[[nodiscard]] PlanningSceneAuthorityResult as_scene(PreGraspAuthorityResult result)
{
  return {result.decision, result.error, std::nullopt, std::move(result.detail)};
}

}  // namespace

PlanningSceneAuthorityResult evaluate_planning_scene_authority(
  const SceneStatus & scene, restocker_world_state::Revision required_world_revision,
  const rclcpp::Time & now, const PreGraspPlanningAuthorityConfig & config,
  const std::optional<PreGraspSceneAuthority> & baseline,
  const std::optional<PlanningSceneLeaseAuthority> & lease)
{
  if (!valid_config(config) || required_world_revision == 0U) {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kInvalidConfiguration,
        "planning frame, ages, future skew, and required revision must be valid"));
  }
  const auto scene_time = check_timestamp(
    scene.header.stamp, now, config.maximum_scene_age, config.maximum_future_skew);
  // A held lease deliberately stops projector reconciliation and status publication. Its status
  // therefore ages while it protects the exact scene a retreat must use. A successful live token
  // validation proves that frozen status is still current; it does not excuse an invalid or
  // future timestamp, and without the capability ordinary age rules remain unchanged.
  if (scene_time.disposition != TimestampDisposition::kFresh &&
    !(lease && scene_time.disposition == TimestampDisposition::kStale))
  {
    return as_scene(timestamp_failure(scene_time, "planning-scene projection"));
  }
  // Classified before the identity and revision fields are trusted: a projector that is starting
  // or waiting on a dependency has legitimately not populated them yet, and rejecting such a
  // status for "invalid identity" would report the wrong cause for a condition that clears itself.
  switch (classify_projector(scene)) {
    case ProjectorCondition::kCertified:
      break;
    case ProjectorCondition::kSettling:
      return as_scene(
        wait(
          PreGraspAuthorityErrorCode::kSceneProjectorSettling,
          "planning-scene projector cannot certify yet and is expected to resolve on its own"));
    case ProjectorCondition::kFailed:
      return as_scene(
        rejected(
          PreGraspAuthorityErrorCode::kSceneStateInvalid,
          "planning-scene projector reports an error"));
  }
  if (scene.header.frame_id != config.planning_frame || scene.projector_epoch.empty() ||
    scene.verification_epoch == 0U || scene.scene_content_generation == 0U)
  {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneStateInvalid,
        "planning-scene proof has invalid frame, epoch, or generation identity"));
  }
  const bool active_lease = scene.lease_id != 0U || scene.lease_phase != SceneLease::PHASE_NONE;
  if (!lease && active_lease) {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneLeaseActive,
        "planning-scene mutation lease is active or unresolved"));
  }
  if (lease &&
    (lease->lease_id == 0U || lease->granted_applied_revision == 0U ||
    lease->verification_epoch == 0U || scene.state != SceneStatus::STATE_TRANSACTION_HELD ||
    scene.error_code != SceneStatus::ERROR_NONE || scene.lease_id != lease->lease_id ||
    scene.lease_phase != SceneLease::PHASE_HELD ||
    scene.applied_revision != lease->granted_applied_revision ||
    scene.verification_epoch != lease->verification_epoch))
  {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneLeaseCapabilityInvalid,
        "validated planning-scene lease does not identify this frozen projection"));
  }
  if (scene.desired_revision == 0U || scene.desired_revision < required_world_revision ||
    scene.desired_revision < scene.applied_revision)
  {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneRevisionInvalid,
        "planning-scene desired revision does not cover the required authoritative snapshot"));
  }
  if (baseline) {
    if (scene.projector_epoch != baseline->projector_epoch) {
      return as_scene(
        rejected(
          PreGraspAuthorityErrorCode::kProjectorEpochChanged,
          "planning-scene projector epoch changed while planning"));
    }
    if (scene.scene_content_generation != baseline->scene_content_generation) {
      return as_scene(
        rejected(
          PreGraspAuthorityErrorCode::kSceneContentChanged,
          "planning-scene collision content changed since this trajectory was planned"));
    }
    if (scene.applied_revision < baseline->applied_revision) {
      return as_scene(
        rejected(
          PreGraspAuthorityErrorCode::kSceneRevisionInvalid,
          "planning-scene applied revision regressed while planning"));
    }
    const std::uint64_t current_lease_id = lease ? lease->lease_id : 0U;
    if (current_lease_id != baseline->lease_id) {
      return as_scene(
        rejected(
          PreGraspAuthorityErrorCode::kSceneLeaseCapabilityInvalid,
          "planning-scene lease changed while planning"));
    }
  }
  if (scene.state == SceneStatus::STATE_SYNCHRONIZING) {
    return as_scene(
      wait(
        PreGraspAuthorityErrorCode::kSceneSynchronizing,
        "planning-scene projector is synchronizing unchanged content"));
  }
  const std::uint8_t required_state =
    lease ? SceneStatus::STATE_TRANSACTION_HELD : SceneStatus::STATE_APPLIED;
  if (scene.state != required_state) {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneStateInvalid,
        "planning-scene projector is not in an applied state"));
  }
  if (scene.applied_revision == 0U || scene.applied_revision < required_world_revision) {
    return as_scene(
      rejected(
        PreGraspAuthorityErrorCode::kSceneRevisionInvalid,
        "applied planning-scene revision does not cover the required authoritative snapshot"));
  }

  return {
    PreGraspAuthorityDecision::kAccepted, PreGraspAuthorityErrorCode::kNone,
    PreGraspSceneAuthority{
      scene.header.frame_id, scene.projector_epoch, scene.applied_revision,
      scene.verification_epoch, scene.scene_content_generation, lease ? lease->lease_id : 0U,
      *scene_time.timestamp},
    "planning-scene authority accepted"};
}

PreGraspAuthorityResult evaluate_pregrasp_planning_authority(
  const SceneStatus & scene, const AttachmentState & attachment,
  restocker_world_state::Revision required_world_revision, const rclcpp::Time & now,
  const PreGraspPlanningAuthorityConfig & config,
  const std::optional<PreGraspPlanningAuthority> & baseline)
{
  std::optional<PreGraspSceneAuthority> scene_baseline;
  if (baseline) {
    scene_baseline = baseline->scene;
  }
  auto scene_authority = evaluate_planning_scene_authority(
    scene, required_world_revision, now, config, scene_baseline);
  if (!scene_authority) {
    return {
      scene_authority.decision, scene_authority.error, std::nullopt,
      std::move(scene_authority.detail)};
  }

  const auto attachment_time = check_timestamp(
    attachment.observed_at, now, config.maximum_attachment_age,
    config.maximum_future_skew);
  if (attachment_time.disposition != TimestampDisposition::kFresh) {
    return timestamp_failure(attachment_time, "simulation attachment");
  }
  if (attachment.simulator_epoch.empty() || attachment.sequence == 0U ||
    attachment.phase != AttachmentState::PHASE_DETACHED ||
    attachment.motion_gate != AttachmentState::MOTION_GATE_VALID ||
    attachment.joint_observed)
  {
    return rejected(
      PreGraspAuthorityErrorCode::kAttachmentStateInvalid,
      "simulated attachment boundary is not freshly and cleanly detached");
  }
  if (baseline) {
    if (attachment.simulator_epoch != baseline->attachment.simulator_epoch) {
      return rejected(
        PreGraspAuthorityErrorCode::kAttachmentEpochChanged,
        "simulator attachment epoch changed while planning");
    }
    if (attachment.sequence != baseline->attachment.sequence) {
      return rejected(
        PreGraspAuthorityErrorCode::kAttachmentSequenceChanged,
        "simulator attachment sequence changed while planning");
    }
  }

  PreGraspPlanningAuthority authority;
  authority.scene = *scene_authority.authority;
  authority.attachment = {
    attachment.simulator_epoch, attachment.sequence, *attachment_time.timestamp};
  return {
    PreGraspAuthorityDecision::kAccepted, PreGraspAuthorityErrorCode::kNone,
    std::move(authority), "planning authority accepted"};
}

const char * to_string(PreGraspAuthorityDecision decision) noexcept
{
  switch (decision) {
    case PreGraspAuthorityDecision::kAccepted: return "accepted";
    case PreGraspAuthorityDecision::kWait: return "wait";
    case PreGraspAuthorityDecision::kRejected: return "rejected";
  }
  return "unknown";
}

const char * to_string(PreGraspAuthorityErrorCode code) noexcept
{
  switch (code) {
    case PreGraspAuthorityErrorCode::kNone: return "none";
    case PreGraspAuthorityErrorCode::kInvalidConfiguration: return "invalid_configuration";
    case PreGraspAuthorityErrorCode::kInvalidTimestamp: return "invalid_timestamp";
    case PreGraspAuthorityErrorCode::kStaleEvidence: return "stale_evidence";
    case PreGraspAuthorityErrorCode::kFutureEvidence: return "future_evidence";
    case PreGraspAuthorityErrorCode::kSceneSynchronizing: return "scene_synchronizing";
    case PreGraspAuthorityErrorCode::kSceneProjectorSettling: return "scene_projector_settling";
    case PreGraspAuthorityErrorCode::kSceneStateInvalid: return "scene_state_invalid";
    case PreGraspAuthorityErrorCode::kSceneLeaseActive: return "scene_lease_active";
    case PreGraspAuthorityErrorCode::kSceneLeaseCapabilityInvalid:
      return "scene_lease_capability_invalid";
    case PreGraspAuthorityErrorCode::kSceneRevisionInvalid: return "scene_revision_invalid";
    case PreGraspAuthorityErrorCode::kProjectorEpochChanged: return "projector_epoch_changed";
    case PreGraspAuthorityErrorCode::kSceneContentChanged: return "scene_content_changed";
    case PreGraspAuthorityErrorCode::kAttachmentStateInvalid:
      return "attachment_state_invalid";
    case PreGraspAuthorityErrorCode::kAttachmentEpochChanged:
      return "attachment_epoch_changed";
    case PreGraspAuthorityErrorCode::kAttachmentSequenceChanged:
      return "attachment_sequence_changed";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
