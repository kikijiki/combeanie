// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

enum class AttachmentDirection : std::uint8_t
{
  kAttach,
  kDetach,
};

[[nodiscard]] const char * attachment_direction_name(AttachmentDirection direction) noexcept;

// What world state should record when a detachment commits. The reserved destination is the
// normal path; releasing without membership is the recovery path for a product that must leave
// the gripper without being credited to a lane.
enum class DetachDisposition : std::uint8_t
{
  kPlaceInReservedDestination,
  kReleaseWithoutMembership,
};

// How much of the attach/detach saga this submission is authorised to run. Place-detach is split
// so a camera survey can prove the destination column grew between the physical release and the
// semantic commit: physical-only opens the jaws in Gazebo, semantic-only commits once that proof
// exists, and full keeps the historical single-shot path (attach, and recovery detach).
enum class AttachmentScope : std::uint8_t
{
  kFull,
  kPhysicalOnly,
  kSemanticOnly,
};

[[nodiscard]] const char * attachment_scope_name(AttachmentScope scope) noexcept;

// Simulation-time stamp carried across the split place-detach halves without depending on a ROS
// message type in this pure library.
struct AttachmentStamp
{
  std::int32_t sec{0};
  std::uint32_t nanosec{0U};

  [[nodiscard]] bool zero() const noexcept
  {
    return sec == 0 && nanosec == 0U;
  }
};

// One complete attach or detach transaction. The coordinator owns the world-state reservation and
// hands its token in; everything else in the saga (the planning-scene lease, the simulated
// physical mutation, the semantic commit) belongs to the port and is never visible here.
struct AttachmentGoal
{
  AttachmentDirection direction{AttachmentDirection::kAttach};
  AttachmentScope scope{AttachmentScope::kFull};
  // World-state object identity. Must match the object the reservation names, or the simulated
  // attachment boundary refuses the mutation.
  std::uint64_t object_id{0};
  // Capability proving the coordinator holds the world-state reservation for this object. The
  // port forwards it to both the attachment boundary and the commit service; it is never logged.
  std::string reservation_token;
  // Grasp-center-to-product transform the boundary must observe before it will attach. Required
  // for attach; must be identity for detach, which is what the boundary's request conversion
  // enforces on the wire.
  Eigen::Isometry3d grasp_center_to_child{Eigen::Isometry3d::Identity()};
  // Covariance of the tracked object whose pose the grasp above was built from, row major in
  // geometry_msgs/PoseWithCovariance order. The boundary sizes the residual it will admit from
  // this, so an estimated grasp is judged against what its estimator claims rather than against a
  // constant calibrated for ground truth. Left zero, the boundary keeps that constant.
  std::array<double, 36> expected_pose_covariance{};
  // Ignored for attach.
  DetachDisposition detach_disposition{DetachDisposition::kPlaceInReservedDestination};
  // Simulation time at which the boundary verified the physical release. Required for a
  // semantic-only detach (the physical half already captured it); ignored otherwise.
  AttachmentStamp released_at;
  // Lease retained by a prior physical-only submission of the same place-detach. Semantic-only
  // reuses it so the projector stays frozen across the retreat survey; empty means acquire a new
  // lease as usual.
  std::string planning_scene_lease_token;
  // Upper bound on acquiring the exclusive planning-scene lease before the transaction is
  // abandoned with the world untouched.
  std::chrono::milliseconds lease_timeout{std::chrono::seconds(10)};
  // Upper bound on the simulated boundary reaching its terminal phase. The Gazebo verifier needs
  // several consecutive settled ticks, so this must comfortably exceed one control period.
  // Ignored for semantic-only, which never commands the boundary.
  std::chrono::milliseconds physical_timeout{std::chrono::seconds(15)};
  // Upper bound on the semantic commit, including idempotent replay of an unknown outcome.
  // Ignored for physical-only, which never commits.
  std::chrono::milliseconds commit_timeout{std::chrono::seconds(10)};
};

[[nodiscard]] bool valid_attachment_goal(const AttachmentGoal & goal) noexcept;

// Terminal results, named for the state of the world they leave behind rather than for the step
// that failed. The driver must be able to tell "nothing happened" from "the gripper is physically
// holding a product world state does not know about" from "we cannot prove which".
enum class AttachmentOutcome : std::uint8_t
{
  // Physical transition verified, semantic commit applied. Both systems agree.
  kSucceeded,
  // Refused before any mutation was attempted: a malformed goal, a capability the boundary would
  // not honour, or a lease that could not be won. The world is untouched.
  kRejected,
  // A service in the transaction was never reachable. Nothing was commanded.
  kUnavailable,
  // Cancellation was honoured at a boundary the port proved was still before the physical
  // mutation. The world is untouched. Cancellation arriving later cannot produce this outcome.
  kCanceled,
  // The simulated boundary refused or failed the transition and its own phase evidence proves the
  // transition was not applied. The world is untouched.
  kPhysicalFailed,
  // The physical transition is applied but the semantic commit is not, and both facts are proven.
  // Gazebo and world state disagree. The port deliberately does not compensate this; see
  // ros_attachment_port.hpp for why the inverse mutation cannot be authorized.
  kUncommitted,
  // The port cannot prove which side of a mutation the world is on. Motion must stop and an
  // operator must reconcile. This is the fail-closed default for every unclassified failure.
  kIndeterminate,
};

// What a terminal outcome proves about the world. This, not the outcome itself, is what a driver
// should branch on when deciding whether it may continue, must compensate, or must stop.
enum class AttachmentWorldEffect : std::uint8_t
{
  // Proven: neither Gazebo nor world state changed.
  kNoneApplied,
  // Proven: both changed, consistently.
  kFullyApplied,
  // Proven: Gazebo changed, world state did not.
  kPhysicalOnly,
  // Unproven. Operator required.
  kIndeterminate,
};

[[nodiscard]] AttachmentWorldEffect attachment_world_effect(AttachmentOutcome outcome) noexcept;

// True when the outcome proves no mutation of any kind occurred. The direct analogue of
// motion_definitely_not_started(): only these outcomes permit retrying or abandoning the task
// without a reconciliation step.
[[nodiscard]] bool attachment_definitely_not_applied(AttachmentOutcome outcome) noexcept;

// True when the outcome leaves the two systems in a state the coordinator must not paper over.
[[nodiscard]] bool attachment_requires_operator(AttachmentOutcome outcome) noexcept;

[[nodiscard]] const char * attachment_outcome_name(AttachmentOutcome outcome) noexcept;

enum class AttachmentSubmitStatus : std::uint8_t
{
  kAccepted,
  // The goal itself is malformed. Never retryable.
  kInvalidRequest,
  // A transaction is already outstanding. The caller must wait for its completion.
  kBusy,
  // The port cannot take work at all, for example during shutdown.
  kUnavailable,
};

struct AttachmentSubmitResult
{
  AttachmentSubmitStatus status{AttachmentSubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == AttachmentSubmitStatus::kAccepted;
  }
};

struct AttachmentCompletion
{
  OperationCorrelation correlation;
  AttachmentOutcome outcome{AttachmentOutcome::kIndeterminate};
  // World revision the semantic commit produced. Zero whenever nothing was committed, so a driver
  // can checkpoint on a non-zero value without re-reading the outcome.
  std::uint64_t world_revision{0};
  // False when the port acquired the exclusive planning-scene lease and could not prove it was
  // released. That freezes the projector, so it is reported on its own axis: it is a liveness
  // failure of the projection pipeline, not evidence about the attachment itself, and a
  // successful transaction can still leave it false. Physical-only success deliberately retains
  // the lease across the retreat survey; the token below is how the semantic half reclaims it.
  bool planning_scene_lease_released{true};
  // Non-empty when the lease was deliberately retained for a later semantic-only submission.
  std::string retained_lease_token;
  // Simulation time at which the boundary verified the physical release. Set on a successful
  // physical detach (full or physical-only); required input for the semantic-only half.
  AttachmentStamp released_at;
  std::string detail;
};

// Run one attach or detach transaction and observe its result. Implementations must invoke the
// completion callback exactly once per accepted submission, from a thread the coordinator does not
// own, and must not call back inline from submit().
class AttachmentPort
{
public:
  using CompletionCallback = std::function<void (AttachmentCompletion)>;

  virtual ~AttachmentPort() = default;

  // False while the attachment, lease or world-state services are unreachable. Submission is
  // still allowed; it will complete kUnavailable rather than block.
  [[nodiscard]] virtual bool ready() const = 0;

  // Accepts at most one outstanding transaction. A second submission before completion is
  // rejected: two concurrent sagas would contend for the single planning-scene lease.
  [[nodiscard]] virtual AttachmentSubmitResult submit(
    OperationCorrelation correlation, AttachmentGoal goal, CompletionCallback callback) = 0;

  // Request that an outstanding transaction stop. Cancellation is only honoured at boundaries the
  // port can prove are before the physical mutation; past that point the saga must run to a
  // terminal state, because abandoning it is what produces an unreconcilable world. The completion
  // still arrives. Safe to call with nothing outstanding.
  virtual void cancel() noexcept = 0;
};

}  // namespace restocker_task_executor
