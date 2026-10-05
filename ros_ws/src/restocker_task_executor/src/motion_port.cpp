// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/motion_port.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool finite_vector3(const Eigen::Vector3d & value) noexcept
{
  return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
}

}  // namespace

std::optional<BoundedPlanningPosition> normalize_bounded_planning_position(
  double measured, double minimum, double maximum, double maximum_correction) noexcept
{
  if (!std::isfinite(measured) || !std::isfinite(minimum) || !std::isfinite(maximum) ||
    !std::isfinite(maximum_correction) || minimum > maximum || maximum_correction < 0.0)
  {
    return std::nullopt;
  }

  if (measured < minimum) {
    if (minimum - measured > maximum_correction) {
      return std::nullopt;
    }
    return BoundedPlanningPosition{minimum, true};
  }
  if (measured > maximum) {
    if (measured - maximum > maximum_correction) {
      return std::nullopt;
    }
    return BoundedPlanningPosition{maximum, true};
  }
  return BoundedPlanningPosition{measured, false};
}

bool valid_motion_path_constraints(const MotionPathConstraints & constraints) noexcept
{
  if (!constraints.position_box && !constraints.orientation_hold) {
    return false;
  }
  if (constraints.position_box) {
    const auto & box = *constraints.position_box;
    if (!finite_vector3(box.min_corner_m) || !finite_vector3(box.max_corner_m)) {
      return false;
    }
    if ((box.max_corner_m.array() <= box.min_corner_m.array()).any()) {
      return false;
    }
  }
  if (constraints.orientation_hold) {
    const auto & hold = *constraints.orientation_hold;
    if (!std::isfinite(hold.orientation.w()) || !std::isfinite(hold.orientation.x()) ||
      !std::isfinite(hold.orientation.y()) || !std::isfinite(hold.orientation.z()) ||
      hold.orientation.norm() <= 0.0 ||
      !std::isfinite(hold.absolute_x_axis_tolerance_rad) ||
      hold.absolute_x_axis_tolerance_rad <= 0.0 ||
      !std::isfinite(hold.absolute_y_axis_tolerance_rad) ||
      hold.absolute_y_axis_tolerance_rad <= 0.0 ||
      !std::isfinite(hold.absolute_z_axis_tolerance_rad) ||
      hold.absolute_z_axis_tolerance_rad <= 0.0)
    {
      return false;
    }
  }
  return true;
}

bool motion_path_position_satisfies_box(
  const Eigen::Vector3d & position, const MotionPathPositionBox & box) noexcept
{
  if (!finite_vector3(position) || !finite_vector3(box.min_corner_m) ||
    !finite_vector3(box.max_corner_m))
  {
    return false;
  }
  return (position.array() >= box.min_corner_m.array()).all() &&
         (position.array() <= box.max_corner_m.array()).all();
}

std::optional<MotionPathPositionBox> inset_motion_path_position_box(
  const MotionPathPositionBox & box, double inset_m) noexcept
{
  if (!finite_vector3(box.min_corner_m) || !finite_vector3(box.max_corner_m) ||
    !std::isfinite(inset_m) || inset_m <= 0.0)
  {
    return std::nullopt;
  }
  MotionPathPositionBox inset;
  inset.min_corner_m = box.min_corner_m.array() + inset_m;
  inset.max_corner_m = box.max_corner_m.array() - inset_m;
  if ((inset.max_corner_m.array() <= inset.min_corner_m.array()).any()) {
    return std::nullopt;
  }
  return inset;
}

bool valid_grasp_escape(const GraspEscape & escape) noexcept
{
  const auto & matrix = escape.planning_frame_from_standoff_tool0.matrix();
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!std::isfinite(matrix(row, column))) {
        return false;
      }
    }
  }
  const Eigen::Matrix3d rotation = escape.planning_frame_from_standoff_tool0.linear();
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() >
    1e-6)
  {
    return false;
  }
  return !escape.target_object_id.empty() && !escape.tolerated_links.empty() &&
         std::ranges::none_of(
    escape.tolerated_links, [](const std::string & link) {return link.empty();});
}

GraspEscapeContactScope classify_grasp_escape_contacts(
  const std::vector<CollisionContactPair> & contacts, const GraspEscape & escape)
{
  GraspEscapeContactScope scope;
  if (escape.target_object_id.empty() || escape.tolerated_links.empty()) {
    scope.refusal = "the grasp escape names no target object or no tolerated finger link";
    return scope;
  }
  const auto tolerated_link = [&escape](const std::string & name) {
    return std::ranges::find(escape.tolerated_links, name) != escape.tolerated_links.end();
  };
  std::vector<std::string> offending;
  for (const auto & [first, second] : contacts) {
    std::optional<CollisionContactPair> pair;
    if (first == escape.target_object_id && tolerated_link(second)) {
      pair = CollisionContactPair{first, second};
    } else if (second == escape.target_object_id && tolerated_link(first)) {
      pair = CollisionContactPair{second, first};
    }
    if (!pair) {
      offending.push_back(first + " against " + second);
      continue;
    }
    if (std::ranges::find(scope.tolerated_pairs, *pair) == scope.tolerated_pairs.end()) {
      scope.tolerated_pairs.push_back(*pair);
    }
  }
  if (!offending.empty()) {
    scope.tolerated_pairs.clear();
    scope.refusal = "the start state touches more than the target product with a finger: ";
    for (std::size_t index = 0; index < offending.size(); ++index) {
      scope.refusal += (index == 0U ? "" : ", ") + offending[index];
    }
    return scope;
  }
  scope.admitted = true;
  return scope;
}

bool valid_motion_goal(const MotionGoal & goal) noexcept
{
  const auto valid_pose = [](const Eigen::Isometry3d & pose) {
    const auto & matrix = pose.matrix();
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
        if (!std::isfinite(matrix(row, column))) {
          return false;
        }
      }
    }
    // A pose whose rotation block is not orthonormal cannot be a valid planning goal, and MoveIt
    // would normalize it.
    const Eigen::Matrix3d rotation = pose.linear();
    const Eigen::Matrix3d residual =
      rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
    return residual.cwiseAbs().maxCoeff() <= 1e-6;
  };
  if (!valid_pose(goal.planning_frame_from_tool0) ||
    (goal.linear_egress_pose &&
    (!valid_pose(*goal.linear_egress_pose) || goal.path != MotionPathKind::kFreeSpace)) ||
    (goal.required_linear_continuation_pose &&
    (!valid_pose(*goal.required_linear_continuation_pose) ||
    goal.path != MotionPathKind::kFreeSpace)) ||
    goal.required_linear_continuation_pose.has_value() !=
    goal.required_linear_continuation_gripper_joint_position_m.has_value() ||
    (goal.required_linear_continuation_gripper_joint_position_m &&
    (!std::isfinite(*goal.required_linear_continuation_gripper_joint_position_m) ||
    *goal.required_linear_continuation_gripper_joint_position_m < 0.0)) ||
    !std::isfinite(goal.required_linear_continuation_vertical_margin_m) ||
    goal.required_linear_continuation_vertical_margin_m < 0.0 ||
    (!goal.required_linear_continuation_pose &&
    goal.required_linear_continuation_vertical_margin_m > 0.0) ||
    (goal.required_postcontinuation_linear_retract_pose &&
    !valid_pose(*goal.required_postcontinuation_linear_retract_pose)) ||
    (goal.required_postcontinuation_linear_egress_pose &&
    !valid_pose(*goal.required_postcontinuation_linear_egress_pose)) ||
    goal.required_postcontinuation_linear_retract_pose.has_value() !=
    goal.required_postcontinuation_linear_egress_pose.has_value() ||
    goal.required_postcontinuation_linear_retract_pose.has_value() !=
    goal.required_postcontinuation_gripper_joint_position_m.has_value() ||
    (goal.required_postcontinuation_linear_retract_pose &&
    !goal.required_linear_continuation_pose) ||
    (goal.required_postcontinuation_gripper_joint_position_m &&
    (!std::isfinite(*goal.required_postcontinuation_gripper_joint_position_m) ||
    *goal.required_postcontinuation_gripper_joint_position_m < 0.0)) ||
    (goal.required_postmotion_linear_egress_pose &&
    (!valid_pose(*goal.required_postmotion_linear_egress_pose) ||
    goal.path != MotionPathKind::kLinear)) ||
    goal.required_postmotion_linear_egress_pose.has_value() !=
    goal.required_postmotion_linear_egress_gripper_joint_position_m.has_value() ||
    (goal.required_postmotion_linear_egress_gripper_joint_position_m &&
    (!std::isfinite(*goal.required_postmotion_linear_egress_gripper_joint_position_m) ||
    *goal.required_postmotion_linear_egress_gripper_joint_position_m < 0.0)) ||
    (goal.grasp_escape && !valid_grasp_escape(*goal.grasp_escape)) ||
    (goal.grasp_escape_leg &&
    (!goal.grasp_escape || goal.path != MotionPathKind::kLinear)))
  {
    return false;
  }
  if (!(goal.position_tolerance_m > 0.0 && goal.orientation_tolerance_rad > 0.0 &&
    goal.velocity_scaling > 0.0 && goal.velocity_scaling <= 1.0 &&
    goal.acceleration_scaling > 0.0 && goal.acceleration_scaling <= 1.0 &&
    goal.planning_time.count() > 0 && goal.maximum_controller_goal_duration.count() > 0 &&
    goal.free_space_plan_candidates > 0U &&
    goal.free_space_plan_candidates <= 8U && goal.cartesian_step_m > 0.0 &&
    goal.minimum_cartesian_fraction > 0.0 && goal.minimum_cartesian_fraction <= 1.0 &&
    !goal.label.empty()))
  {
    return false;
  }
  if (goal.maximum_tool0_x_axis_tilt_rad &&
    (!std::isfinite(*goal.maximum_tool0_x_axis_tilt_rad) ||
    *goal.maximum_tool0_x_axis_tilt_rad <= 0.0 ||
    *goal.maximum_tool0_x_axis_tilt_rad >= 0.5 * std::acos(-1.0)))
  {
    return false;
  }
  // Path constraints belong to free-space sampling only. A linear segment that carried them would
  // either ignore them or change Cartesian behaviour; refuse both.
  if (goal.path_constraints) {
    if (goal.path != MotionPathKind::kFreeSpace) {
      return false;
    }
    if (!valid_motion_path_constraints(*goal.path_constraints)) {
      return false;
    }
  }
  return true;
}

FreeSpaceSlicePlan plan_free_space_slices(
  double budget_seconds, std::size_t candidate_count) noexcept
{
  if (!(budget_seconds > 0.0) || candidate_count == 0U) {
    return {};
  }
  if (candidate_count == 1U) {
    // One candidate keeps the whole floor as a single attempt: splitting it would restart the
    // search the floor was about to spend.
    return {budget_seconds, 0.0};
  }
  // pass1 × (N − 1) = floor / 2, so pass1 = floor / (2 (N − 1)) and the reserved remainder is
  // exactly floor / 2 — the preferred candidate's aggregate share inside the round.
  const double first_pass =
    budget_seconds / (2.0 * static_cast<double>(candidate_count - 1U));
  const double second_pass =
    budget_seconds - first_pass * static_cast<double>(candidate_count);
  return {first_pass, std::max(0.0, second_pass)};
}

bool motion_definitely_not_started(MotionOutcome outcome) noexcept
{
  switch (outcome) {
    case MotionOutcome::kPlanningFailed:
    case MotionOutcome::kUnavailable:
      return true;
    case MotionOutcome::kSucceeded:
    case MotionOutcome::kExecutionFailed:
    case MotionOutcome::kCanceled:
    case MotionOutcome::kTimedOut:
      return false;
  }
  return false;
}

bool motion_completion_definitely_not_started(const MotionCompletion & completion) noexcept
{
  return motion_definitely_not_started(completion.outcome) && !completion.submitted_to_backend &&
         !completion.grasp_escape_executed;
}

const char * motion_outcome_name(MotionOutcome outcome) noexcept
{
  switch (outcome) {
    case MotionOutcome::kSucceeded:
      return "succeeded";
    case MotionOutcome::kPlanningFailed:
      return "planning failed";
    case MotionOutcome::kExecutionFailed:
      return "execution failed";
    case MotionOutcome::kCanceled:
      return "canceled";
    case MotionOutcome::kTimedOut:
      return "timed out";
    case MotionOutcome::kUnavailable:
      return "backend unavailable";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
