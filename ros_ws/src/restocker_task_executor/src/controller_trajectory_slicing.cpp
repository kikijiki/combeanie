// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/controller_trajectory_slicing.hpp"

#include <cmath>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] double seconds_from_start(
  const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

// At rest as TOTG leaves an endpoint: every joint slower than its scaled acceleration bound
// sustained over kControllerGoalRestWindowS. A point with no velocities, or a joint with no
// acceleration bound, says nothing about rest and is not treated as at rest.
[[nodiscard]] bool at_rest(
  const trajectory_msgs::msg::JointTrajectoryPoint & point,
  const std::vector<std::string> & joint_names, const moveit::core::RobotModel & model,
  double acceleration_scaling)
{
  if (point.velocities.size() != joint_names.size() || !std::isfinite(acceleration_scaling) ||
    acceleration_scaling <= 0.0)
  {
    return false;
  }
  for (std::size_t index = 0U; index < joint_names.size(); ++index) {
    if (!model.hasJointModel(joint_names[index])) {
      return false;
    }
    const auto & bounds = model.getVariableBounds(joint_names[index]);
    const double velocity = point.velocities[index];
    if (!bounds.acceleration_bounded_ || !std::isfinite(velocity) ||
      std::abs(velocity) >
      bounds.max_acceleration_ * acceleration_scaling * kControllerGoalRestWindowS)
    {
      return false;
    }
  }
  return true;
}

struct Retimed
{
  std::optional<moveit_msgs::msg::RobotTrajectory> trajectory;
  bool used_fallback{false};
};

// Stop-to-stop time parameterisation of one slice: the strict tolerance first, then the single
// fallback. Nothing else is tried.
[[nodiscard]] Retimed retime_slice(
  const moveit_msgs::msg::RobotTrajectory & path, const moveit::core::RobotState & planning_start,
  const std::string & planning_group, double velocity_scaling, double acceleration_scaling)
{
  for (const double tolerance :
    {kControllerSlicePathTolerance, kControllerSliceFallbackPathTolerance})
  {
    const trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization(
      tolerance, kControllerSliceResampleDtS, kControllerSliceMinimumAngleChange);
    robot_trajectory::RobotTrajectory retimed(planning_start.getRobotModel(), planning_group);
    retimed.setRobotTrajectoryMsg(planning_start, path);
    if (time_parameterization.computeTimeStamps(
        retimed, velocity_scaling, acceleration_scaling))
    {
      moveit_msgs::msg::RobotTrajectory result;
      retimed.getRobotTrajectoryMsg(result);
      return {std::move(result), tolerance != kControllerSlicePathTolerance};
    }
  }
  return {};
}

}  // namespace

TrajectoryObservation observe_trajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
{
  TrajectoryObservation observation;
  observation.joint_names = trajectory.joint_trajectory.joint_names;
  observation.points.reserve(trajectory.joint_trajectory.points.size());
  for (const auto & point : trajectory.joint_trajectory.points) {
    observation.points.push_back(
      TrajectoryPointObservation{point.positions, seconds_from_start(point)});
  }
  return observation;
}

ControllerGoalDivision divide_into_controller_goals(
  const moveit_msgs::msg::RobotTrajectory & trajectory,
  const moveit::core::RobotState & planning_start, const std::string & planning_group,
  double velocity_scaling, double acceleration_scaling,
  std::chrono::milliseconds maximum_duration)
{
  ControllerGoalDivision division;
  const auto refuse = [&division](const std::string & detail) {
    division.goals.clear();
    division.refusal = detail;
    return division;
  };
  const auto chunks = controller_execution_chunks(
    observe_trajectory(trajectory), maximum_duration);
  if (!chunks || chunks->empty()) {
    return refuse(
      "its timing is not executable or one waypoint gap exceeds the per-goal bound");
  }
  // Multi-DOF paths are refused: copying one unsliced next to sliced scalar joints would describe
  // different instants.
  if (!trajectory.multi_dof_joint_trajectory.points.empty()) {
    return refuse("it carries multi-DOF joints, which cannot be sliced");
  }
  const auto & source = trajectory.joint_trajectory.points;
  const auto & names = trajectory.joint_trajectory.joint_names;
  const auto & model = planning_start.getRobotModel();
  if (!model) {
    return refuse("no robot model is available to time its slices");
  }

  const double maximum_seconds = std::chrono::duration<double>(maximum_duration).count();
  // The planner's own trajectory already fits one goal and already stops at both ends: re-timing
  // it cannot make it more executable and can only fail (Card 061). The chunking above has
  // already checked its timing; the controller runs time_from_start as given, so the bound is
  // checked from t = 0, not from the first waypoint.
  if (chunks->size() == 1U && source.size() >= 2U && chunks->front().first_point == 0U &&
    chunks->front().last_point + 1U == source.size() &&
    seconds_from_start(source.back()) <= maximum_seconds &&
    at_rest(source.front(), names, *model, acceleration_scaling) &&
    at_rest(source.back(), names, *model, acceleration_scaling))
  {
    division.goals.push_back(trajectory);
    division.planner_timing_kept = true;
    return division;
  }

  for (const auto & requested : *chunks) {
    if (requested.first_point > requested.last_point || requested.last_point >= source.size()) {
      return refuse("the chunking returned a range outside the trajectory");
    }
    std::size_t first = requested.first_point;
    while (first < requested.last_point ||
      (requested.first_point == requested.last_point && division.goals.empty()))
    {
      std::size_t last = requested.last_point;
      std::optional<moveit_msgs::msg::RobotTrajectory> accepted;
      while (last > first || first == requested.last_point) {
        moveit_msgs::msg::RobotTrajectory path = trajectory;
        path.joint_trajectory.points.assign(
          source.begin() + static_cast<std::ptrdiff_t>(first),
          source.begin() + static_cast<std::ptrdiff_t>(last + 1U));
        auto retimed = retime_slice(
          path, planning_start, planning_group, velocity_scaling, acceleration_scaling);
        if (!retimed.trajectory) {
          std::ostringstream detail;
          detail << "points " << first << ".." << last << " of " << source.size()
                 << " could not be time-parameterised stop-to-stop at path tolerance "
                 << kControllerSlicePathTolerance << " or " <<
            kControllerSliceFallbackPathTolerance;
          return refuse(detail.str());
        }
        const auto timing = validate_trajectory_timing(
          observe_trajectory(*retimed.trajectory), "controller execution slice");
        const auto & points = retimed.trajectory->joint_trajectory.points;
        const double duration_seconds = points.empty() ?
          std::numeric_limits<double>::infinity() : seconds_from_start(points.back());
        if (timing.ok() && duration_seconds <= maximum_seconds) {
          if (retimed.used_fallback) {
            ++division.fallback_retimes;
          }
          accepted = std::move(retimed.trajectory);
          break;
        }
        if (last == first + 1U || last == first) {
          break;
        }
        --last;
      }
      if (!accepted) {
        std::ostringstream detail;
        detail << "no slice starting at point " << first << " of " << source.size()
               << " re-times within the " << maximum_seconds << " s per-goal bound";
        return refuse(detail.str());
      }
      division.goals.push_back(std::move(*accepted));
      if (first == requested.last_point) {
        break;
      }
      first = last;
    }
  }
  return division;
}

}  // namespace restocker_task_executor
