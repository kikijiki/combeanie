// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/gripper_port.hpp"

#include <cmath>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool joint_near(double observed, double target, double tolerance) noexcept
{
  return std::isfinite(observed) && std::abs(observed - target) <= tolerance;
}

}  // namespace

bool valid_gripper_goal(const GripperGoal & goal) noexcept
{
  if (!std::isfinite(goal.target_position_m) || !std::isfinite(goal.position_tolerance_m)) {
    return false;
  }
  // The controller enforces joint limits itself, but a target outside them can only ever be
  // rejected, so refusing it here keeps an impossible goal off the wire.
  if (goal.target_position_m < kFingerJointLowerM || goal.target_position_m > kFingerJointUpperM) {
    return false;
  }
  if (goal.position_tolerance_m <= 0.0) {
    return false;
  }
  if (goal.move_duration.count() <= 0 || goal.deadline.count() <= 0) {
    return false;
  }
  // A deadline shorter than the move it authorises would fire before the fingers could physically
  // arrive, turning every submission into a spurious kTimedOut.
  return goal.deadline >= goal.move_duration;
}

bool fingers_at_target(
  const GripperFingerState & measured, double target_position_m,
  double position_tolerance_m) noexcept
{
  if (!std::isfinite(target_position_m) || !std::isfinite(position_tolerance_m) ||
    position_tolerance_m <= 0.0)
  {
    return false;
  }
  return joint_near(measured.left_position_m, target_position_m, position_tolerance_m) &&
         joint_near(measured.right_position_m, target_position_m, position_tolerance_m);
}

bool gripper_definitely_not_started(GripperOutcome outcome) noexcept
{
  switch (outcome) {
    case GripperOutcome::kRejected:
    case GripperOutcome::kUnavailable:
      return true;
    case GripperOutcome::kSucceeded:
    case GripperOutcome::kExecutionFailed:
    case GripperOutcome::kPositionNotVerified:
    case GripperOutcome::kCanceled:
    case GripperOutcome::kTimedOut:
      return false;
  }
  return false;
}

const char * gripper_outcome_name(GripperOutcome outcome) noexcept
{
  switch (outcome) {
    case GripperOutcome::kSucceeded:
      return "succeeded";
    case GripperOutcome::kRejected:
      return "goal rejected";
    case GripperOutcome::kExecutionFailed:
      return "execution failed";
    case GripperOutcome::kPositionNotVerified:
      return "finger position not verified";
    case GripperOutcome::kCanceled:
      return "canceled";
    case GripperOutcome::kTimedOut:
      return "timed out";
    case GripperOutcome::kUnavailable:
      return "backend unavailable";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
