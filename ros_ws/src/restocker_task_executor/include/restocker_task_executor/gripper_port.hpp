// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

// Travel limits of left_finger_joint and right_finger_joint, from
// restocker_description/config/gripper_geometry.yaml. Both joints are prismatic and symmetric, so
// one commanded value describes the whole jaw.
inline constexpr double kFingerJointLowerM = 0.0;
inline constexpr double kFingerJointUpperM = 0.035;

// The attachment plugin only latches while both fingers sit within this band of the hold target
// (restocker_gazebo/config/attachment_boundary.yaml, joint_position_m). gripper_controller's own
// goal constraint is 0.003 (six times looser), so a succeeded action does not prove a grasp.
inline constexpr double kAttachmentJointToleranceM = 0.0005;

// One commanded jaw position. The coordinator submits exactly one at a time and never sees the
// trajectory message: joint naming and controller wire format stay inside the port.
struct GripperGoal
{
  // Commanded position for both finger joints, in metres. 0.0 is fully closed.
  double target_position_m{kFingerJointUpperM};
  // Half-width the measured finger positions must land within before the port reports success.
  // Defaults to the attachment band; a looser value would report a good grasp the plugin refuses
  // to attach.
  double position_tolerance_m{kAttachmentJointToleranceM};
  // Time budget handed to the controller for the single trajectory point.
  std::chrono::milliseconds move_duration{std::chrono::milliseconds(800)};
  // Upper bound on the whole submission, including position verification, before kTimedOut.
  std::chrono::milliseconds deadline{std::chrono::seconds(10)};
};

[[nodiscard]] bool valid_gripper_goal(const GripperGoal & goal) noexcept;

// Measured finger positions taken from /joint_states. Both are required: the plugin checks each
// finger separately, so a single averaged value could hide an asymmetric jaw.
struct GripperFingerState
{
  double left_position_m{0.0};
  double right_position_m{0.0};
};

// True when both measured fingers sit within tolerance of the commanded target. Non-finite
// measurements are never within tolerance. Mirrors the plugin's joint_near check.
[[nodiscard]] bool fingers_at_target(
  const GripperFingerState & measured, double target_position_m,
  double position_tolerance_m) noexcept;

enum class GripperOutcome : std::uint8_t
{
  // The controller reported success and the measured fingers matched the commanded target.
  kSucceeded,
  // The controller refused the goal, for example a target outside the joint limits. No finger
  // command was issued.
  kRejected,
  // The goal was accepted but the controller aborted or reported a trajectory error. The fingers
  // may have moved part way.
  kExecutionFailed,
  // The controller reported success but /joint_states did not confirm the target, either because
  // the fingers missed the tight attachment band or because no usable sample arrived. The fingers
  // did move, so this is a failed grasp, not a no-op.
  kPositionNotVerified,
  // Cancellation was requested and the port stopped the trajectory.
  kCanceled,
  // The port did not reach a terminal result inside the goal's deadline.
  kTimedOut,
  // The gripper action server or the joint-state stream was not available. Nothing was commanded.
  kUnavailable,
};

// True when the outcome proves no finger command reached the controller. Only these outcomes
// permit continuing without re-checking where the jaw actually is.
//
// kPositionNotVerified is excluded: the controller ran the trajectory to completion, so the jaw
// has moved even though the grasp cannot be trusted.
[[nodiscard]] bool gripper_definitely_not_started(GripperOutcome outcome) noexcept;

[[nodiscard]] const char * gripper_outcome_name(GripperOutcome outcome) noexcept;

enum class GripperSubmitStatus : std::uint8_t
{
  kAccepted,
  // The goal itself is malformed. Never retryable.
  kInvalidRequest,
  // A goal is already outstanding. The caller must wait for its completion.
  kBusy,
  // The port cannot take work at all, for example during shutdown.
  kUnavailable,
};

struct GripperSubmitResult
{
  GripperSubmitStatus status{GripperSubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == GripperSubmitStatus::kAccepted;
  }
};

struct GripperCompletion
{
  OperationCorrelation correlation;
  GripperOutcome outcome{GripperOutcome::kUnavailable};
  std::string detail;
  // The sample the port verified against, when one was obtained. Absent otherwise, so presence is
  // evidence, not a default.
  std::optional<GripperFingerState> measured;
};

// Command one jaw position and observe its result. Implementations must invoke the completion
// callback exactly once per accepted submission, from a thread the coordinator does not own, and
// must not call back inline from submit().
class GripperPort
{
public:
  using CompletionCallback = std::function<void (GripperCompletion)>;

  virtual ~GripperPort() = default;

  // False while the gripper action server or the joint-state stream is unreachable. Submission is
  // still allowed; it will complete kUnavailable rather than block.
  [[nodiscard]] virtual bool ready() const = 0;

  // Accepts at most one outstanding goal. A second submission before completion is rejected.
  [[nodiscard]] virtual GripperSubmitResult submit(
    OperationCorrelation correlation, GripperGoal goal, CompletionCallback callback) = 0;

  // Request that an outstanding jaw motion stop. The completion still arrives, normally
  // kCanceled. Safe to call with nothing outstanding.
  virtual void cancel() noexcept = 0;
};

}  // namespace restocker_task_executor
