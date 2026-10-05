// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// SPDX-License-Identifier: MIT

#include "restocker_task_executor/trajectory_scene_validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include <moveit/collision_detection/collision_common.hpp>

namespace restocker_task_executor
{
namespace
{

std::string lineage_error(
  const PreGraspSceneAuthority & original, const PreGraspSceneAuthority & current)
{
  if (current.projector_epoch != original.projector_epoch ||
    current.planning_frame != original.planning_frame || current.lease_id != original.lease_id)
  {
    return "planning-scene frame, projector epoch or lease changed since planning";
  }
  if (current.applied_revision < original.applied_revision ||
    current.scene_content_generation < original.scene_content_generation ||
    current.verification_epoch < original.verification_epoch)
  {
    return "planning-scene proof regressed since planning";
  }
  return {};
}

// Hermite interpolation used by JTC's default spline mode, expressed as Bezier control
// points. The derivative control polygon bounds every joint's speed in normalized time.
std::vector<double> spline_controls(
  const trajectory_msgs::msg::JointTrajectoryPoint & begin,
  const trajectory_msgs::msg::JointTrajectoryPoint & finish, std::size_t joint, double duration)
{
  const double p0 = begin.positions[joint];
  const double p1 = finish.positions[joint];
  if (begin.velocities.empty() || finish.velocities.empty()) {
    return {p0, p1};
  }
  const double v0 = begin.velocities[joint] * duration;
  const double v1 = finish.velocities[joint] * duration;
  if (begin.accelerations.empty() || finish.accelerations.empty()) {
    return {p0, p0 + v0 / 3.0, p1 - v1 / 3.0, p1};
  }
  const double a0 = begin.accelerations[joint] * duration * duration;
  const double a1 = finish.accelerations[joint] * duration * duration;
  return {p0, p0 + v0 / 5.0, p0 + 2.0 * v0 / 5.0 + a0 / 20.0,
    p1 - 2.0 * v1 / 5.0 + a1 / 20.0, p1 - v1 / 5.0, p1};
}

double spline_position(const std::vector<double> & controls, double fraction)
{
  std::array<double, 6> values{};
  std::copy(controls.begin(), controls.end(), values.begin());
  for (std::size_t remaining = controls.size() - 1U; remaining > 0U; --remaining) {
    for (std::size_t index = 0U; index < remaining; ++index) {
      values[index] = std::lerp(values[index], values[index + 1U], fraction);
    }
  }
  return values.front();
}

double trajectory_seconds(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         1.0e-9 * static_cast<double>(point.time_from_start.nanosec);
}

}  // namespace

TrajectorySceneGateResult validate_trajectory_scene_submission(
  const PreGraspSceneAuthority & original, const TrajectorySceneGateHooks & hooks)
{
  if (!hooks.read_authority || !hooks.validate_next_goal || !hooks.interrupted) {
    return {false, false, "scene submission gate is missing a required check"};
  }
  auto latest_seen = original;
  for (std::size_t attempt = 0; attempt < 3U; ++attempt) {
    if (hooks.interrupted()) {
      return {false, false, "scene validation canceled or its steady deadline expired"};
    }
    const auto before = hooks.read_authority();
    if (!before) {
      return {false, false, before.detail};
    }
    if (const auto error = lineage_error(latest_seen, *before.authority); !error.empty()) {
      return {false, false, error};
    }
    if (hooks.interrupted()) {
      return {false, false, "scene validation canceled or its steady deadline expired"};
    }
    if (before.authority->scene_content_generation == original.scene_content_generation) {
      return {true, false, {}};
    }
    const auto collision_error = hooks.validate_next_goal();
    if (hooks.interrupted()) {
      return {false, false, "scene validation canceled or its steady deadline expired"};
    }
    const auto after = hooks.read_authority();
    if (!after) {
      return {false, false, after.detail};
    }
    if (const auto error = lineage_error(*before.authority, *after.authority); !error.empty()) {
      return {false, false, error};
    }
    if (hooks.interrupted()) {
      return {false, false, "scene validation canceled or its steady deadline expired"};
    }
    if (after.authority->scene_content_generation != before.authority->scene_content_generation) {
      latest_seen = *after.authority;
      continue;
    }
    if (!collision_error.empty()) {
      return {false, false, collision_error};
    }
    return {true, true, {}};
  }
  return {false, false, "collision scene kept changing across three validation attempts"};
}

std::string validate_controller_goal_collision_scene(
  const moveit::core::RobotModelConstPtr & model,
  const moveit_msgs::msg::PlanningScene & snapshot,
  const moveit_msgs::msg::RobotTrajectory & trajectory, const std::string & group_name,
  const std::function<bool()> & interrupted, double maximum_start_state_bounds_correction)
{
  constexpr double step = 0.0025;
  constexpr std::size_t maximum_samples = 50'000U;
  if (!model || !interrupted || interrupted() ||
    !std::isfinite(maximum_start_state_bounds_correction) ||
    maximum_start_state_bounds_correction < 0.0)
  {
    return "collision validation has no model or was interrupted";
  }
  const auto * group = model->getJointModelGroup(group_name);
  const auto & names = trajectory.joint_trajectory.joint_names;
  const auto & points = trajectory.joint_trajectory.points;
  const std::set<std::string> trajectory_names(names.begin(), names.end());
  if (!group || names.empty() || points.empty() || points.size() > maximum_samples ||
    trajectory_names.size() != names.size() ||
    trajectory_names != std::set<std::string>(
      group->getVariableNames().begin(), group->getVariableNames().end()) ||
    !trajectory.multi_dof_joint_trajectory.points.empty())
  {
    return "collision validation requires a complete bounded scalar-joint controller goal";
  }
  std::map<std::string, double> padding;
  std::map<std::string, double> scale;
  for (const auto & entry : snapshot.link_padding) {
    if (!std::isfinite(entry.padding) || entry.padding < 0.0 ||
      !padding.emplace(entry.link_name, entry.padding).second)
    {
      return "collision scene has invalid or duplicate link padding";
    }
  }
  for (const auto & entry : snapshot.link_scale) {
    if (!std::isfinite(entry.scale) || entry.scale <= 0.0 ||
      !scale.emplace(entry.link_name, entry.scale).second)
    {
      return "collision scene has invalid or duplicate link scale";
    }
  }
  for (const auto * link : model->getLinkModelsWithCollisionGeometry()) {
    if (!padding.contains(link->getName()) || !scale.contains(link->getName())) {
      return "collision scene omits padding/scale for " + link->getName();
    }
  }
  const auto & joints = snapshot.robot_state.joint_state;
  if (joints.name.size() != joints.position.size()) {
    return "collision scene has incomplete robot state";
  }
  std::set<std::string> observed;
  for (std::size_t index = 0; index < joints.name.size(); ++index) {
    if (!observed.insert(joints.name[index]).second || !std::isfinite(joints.position[index])) {
      return "collision scene has invalid or duplicate robot state";
    }
  }
  for (const auto & variable : model->getVariableNames()) {
    if (!observed.contains(variable)) {
      return "collision scene omits robot state variable " + variable;
    }
  }
  double previous_time = -1.0;
  for (const auto & point : points) {
    for (const auto * values : {&point.positions, &point.velocities, &point.accelerations}) {
      if ((values == &point.positions || !values->empty()) &&
        (values->size() != names.size() ||
        !std::ranges::all_of(*values, [](double value) {return std::isfinite(value);})))
      {
        return "controller goal has incomplete or non-finite positions/derivatives";
      }
    }
    const double time = trajectory_seconds(point);
    if (point.time_from_start.nanosec >= 1'000'000'000U || time <= previous_time || time < 0.0 ||
      (previous_time < 0.0 && time != 0.0) ||
      (!point.accelerations.empty() && point.velocities.empty()))
    {
      return "controller goal has unsupported timing or derivative fields";
    }
    previous_time = time;
  }
  auto build = make_race_scene(model, snapshot);
  if (!build.scene) {
    return "collision scene could not be reconstructed: " + build.refusal;
  }
  auto state = build.scene->getCurrentState();
  collision_detection::CollisionRequest request;
  request.group_name = group_name;
  request.contacts = true;
  request.max_contacts = 1U;
  trajectory_msgs::msg::JointTrajectoryPoint measured_start;
  for (const auto & name : names) {
    measured_start.positions.push_back(state.getVariablePosition(name));
  }
  std::size_t samples = 0;
  for (std::size_t index = 0; index < points.size(); ++index) {
    const auto & finish = points[index];
    const auto & begin = index == 0U ? measured_start : points[index - 1U];
    const double duration = trajectory_seconds(finish) - trajectory_seconds(begin);
    std::vector<std::vector<double>> controls;
    double maximum_derivative = 0.0;
    for (std::size_t joint = 0; joint < names.size(); ++joint) {
      auto values = spline_controls(begin, finish, joint, duration);
      const double degree = static_cast<double>(values.size() - 1U);
      for (std::size_t part = 1U; part < values.size(); ++part) {
        const double derivative = degree * std::abs(values[part] - values[part - 1U]);
        if (!std::isfinite(derivative)) {
          return "controller spline has non-finite coefficients";
        }
        maximum_derivative = std::max(maximum_derivative, derivative);
      }
      controls.push_back(std::move(values));
    }
    const double required = std::max(1.0, std::ceil(maximum_derivative / step));
    if (!std::isfinite(required) ||
      required + (index == 0U ? 1.0 : 0.0) > static_cast<double>(maximum_samples - samples))
    {
      return "collision validation exceeded its finite sample budget";
    }
    const auto subdivisions = static_cast<std::size_t>(required);
    for (std::size_t part = index == 0U ? 0U : 1U; part <= subdivisions; ++part) {
      if (interrupted()) {
        return "collision validation canceled or its steady deadline expired";
      }
      const double fraction = static_cast<double>(part) / static_cast<double>(subdivisions);
      for (std::size_t joint = 0; joint < names.size(); ++joint) {
        state.setVariablePosition(
          names[joint],
          spline_position(controls[joint], fraction));
      }
      state.update();
      ++samples;
      // execute_goal already admits this bounded measured-start discrepancy. Preserve its
      // actual collision geometry; only bridge bounds eligibility gets the existing allowance.
      const double bounds_margin = index == 0U && part < subdivisions ?
        maximum_start_state_bounds_correction : 0.0;
      if (!std::ranges::all_of(
          group->getActiveJointModels(), [&](const auto * joint) {
            return state.satisfiesPositionBounds(joint, bounds_margin);
          }))
      {
        return "controller goal violates joint bounds during collision validation";
      }
      collision_detection::CollisionResult collision;
      build.scene->checkCollision(request, collision, state);
      if (collision.collision) {
        std::string detail = "controller goal collides in current scene at sample " +
          std::to_string(samples);
        if (!collision.contacts.empty()) {
          detail += ": " + collision.contacts.begin()->first.first + " against " +
            collision.contacts.begin()->first.second;
        }
        return detail;
      }
    }
  }
  return {};
}

}  // namespace restocker_task_executor
