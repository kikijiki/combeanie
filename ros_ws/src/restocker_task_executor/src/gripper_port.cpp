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
  if (!std::isfinite(goal.stall_band_m) || goal.stall_band_m < 0.0) {
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

bool fingers_stalled_in_band(
  const GripperFingerState & measured, const GripperFingerState & previous,
  double target_position_m, double position_tolerance_m, double stall_band_m,
  double rest_m) noexcept
{
  if (!std::isfinite(target_position_m) || !std::isfinite(position_tolerance_m) ||
    position_tolerance_m <= 0.0 || !std::isfinite(stall_band_m) || stall_band_m <= 0.0 ||
    !std::isfinite(rest_m) || rest_m <= 0.0)
  {
    return false;
  }
  const auto in_band = [&](double position) {
    return std::isfinite(position) && position >= target_position_m - position_tolerance_m &&
           position <= target_position_m + stall_band_m;
  };
  const auto at_rest = [&](double now, double before) {
    return std::isfinite(before) && std::abs(now - before) <= rest_m;
  };
  return in_band(measured.left_position_m) && in_band(measured.right_position_m) &&
         at_rest(measured.left_position_m, previous.left_position_m) &&
         at_rest(measured.right_position_m, previous.right_position_m);
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
