// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT


#include "restocker_task_executor/grasp_escape_planner.hpp"

#include <algorithm>
#include <iomanip>
#include <map>
#include <optional>
#include <sstream>
#include <utility>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>

#include "restocker_task_executor/race_planning_scene.hpp"

namespace restocker_task_executor
{
namespace
{

// The deepest reported penetration per contact pair, as MoveIt names the two bodies.
using ContactDepths = std::map<CollisionContactPair, double>;

[[nodiscard]] ContactDepths contact_depths(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & state,
  const collision_detection::AllowedCollisionMatrix & matrix)
{
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 64U;
  request.max_contacts_per_pair = 4U;
  collision_detection::CollisionResult result;
  scene.checkCollision(request, result, state, matrix);
  ContactDepths depths;
  for (const auto & [pair, contacts] : result.contacts) {
    double deepest = 0.0;
    for (const auto & contact : contacts) {
      deepest = std::max(deepest, contact.depth);
    }
    depths[CollisionContactPair{pair.first, pair.second}] = deepest;
  }
  if (result.collision && depths.empty()) {
    // A collision without a named pair can never be proven inside the escape's scope.
    depths[CollisionContactPair{"an unreported body", "an unreported body"}] = 0.0;
  }
  return depths;
}

[[nodiscard]] std::vector<CollisionContactPair> pairs_of(const ContactDepths & depths)
{
  std::vector<CollisionContactPair> pairs;
  pairs.reserve(depths.size());
  for (const auto & entry : depths) {
    pairs.push_back(entry.first);
  }
  return pairs;
}

// MoveIt may name a pair in either order.
[[nodiscard]] std::optional<double> depth_of(
  const ContactDepths & depths, const CollisionContactPair & pair)
{
  if (const auto found = depths.find(pair); found != depths.end()) {
    return found->second;
  }
  if (const auto found = depths.find({pair.second, pair.first}); found != depths.end()) {
    return found->second;
  }
  return std::nullopt;
}

// Card 062 review N5: along the line the tolerated contact may not be driven deeper. FCL's
// penetration depth for a box finger against a cylinder is an estimate, not a monotone measure:
// on slot 16's own escape it read 2.50 mm at the start and 3.90-4.03 mm on every later waypoint
// while the finger slid straight out (test_grasp_escape_planner). The bound therefore allows that
// estimator noise and nothing like a finger crossing a body, which drives the depth towards the
// finger's 18 mm thickness.
constexpr double kToleratedDepthGrowthM = 0.002;

}  // namespace

GraspEscapeLine plan_grasp_escape_line(
  const planning_scene::PlanningScene & scene, const moveit::core::RobotState & start,
  const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * link,
  const GraspEscape & escape, double cartesian_step_m, double minimum_fraction)
{
  GraspEscapeLine line;
  if (group == nullptr || link == nullptr || !valid_grasp_escape(escape) ||
    !(cartesian_step_m > 0.0) || !(minimum_fraction > 0.0 && minimum_fraction <= 1.0))
  {
    line.detail = "the grasp escape request is incomplete";
    return line;
  }
  const auto & unmodified = scene.getAllowedCollisionMatrix();
  const auto start_depths = contact_depths(scene, start, unmodified);
  const auto scope = classify_grasp_escape_contacts(pairs_of(start_depths), escape);
  if (!scope.admitted) {
    line.detail = "start state is in collision outside the grasp escape's scope: " +
      scope.refusal;
    return line;
  }
  // Every waypoint is judged by the unmodified matrix: only the start's own finger–target pairs
  // may be in contact, and none deeper than at the start beyond the estimator bound above, so a
  // contact can be left but not driven further (a product lying across the finger's path is
  // refused).
  ContactDepths allowed_depth;
  for (const auto & pair : scope.tolerated_pairs) {
    allowed_depth[pair] = depth_of(start_depths, pair).value_or(0.0) + kToleratedDepthGrowthM;
  }
  const moveit::core::GroupStateValidityCallbackFn valid = [&scene, &unmodified, &allowed_depth](
    moveit::core::RobotState * state, const moveit::core::JointModelGroup * checked,
    const double * values) {
    state->setJointGroupPositions(checked, values);
    state->update();
    if (!state->satisfiesBounds(checked)) {
      return false;
    }
    for (const auto & [pair, depth] : contact_depths(scene, *state, unmodified)) {
      const auto allowed = depth_of(allowed_depth, pair);
      if (!allowed || depth > *allowed) {
        return false;
      }
    }
    return true;
  };
  moveit::core::RobotState seed(start);
  seed.update();
  const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
    &seed, group, line.waypoints, link, escape.planning_frame_from_standoff_tool0, true,
    moveit::core::MaxEEFStep(cartesian_step_m), moveit::core::CartesianPrecision{}, valid,
    kinematics::KinematicsQueryOptions());
  std::ostringstream stream;
  stream << std::setprecision(4);
  if (fraction < minimum_fraction || line.waypoints.empty()) {
    stream << "the straight line back to the pre-grasp standoff stopped at fraction " <<
      fraction << " with " << scope.tolerated_pairs.size() << " tolerated contact pair(s)";
    line.detail = stream.str();
    line.waypoints.clear();
    return line;
  }
  // The tolerance must not carry the contact out of the grasp.
  if (const auto remaining = contact_depths(scene, *line.waypoints.back(), unmodified);
    !remaining.empty())
  {
    stream << "the pre-grasp standoff is still in collision under the unmodified matrix: " <<
      remaining.begin()->first.first << " against " << remaining.begin()->first.second;
    line.detail = stream.str();
    line.waypoints.clear();
    return line;
  }
  stream << "straight line back to the pre-grasp standoff, " << line.waypoints.size() <<
    " waypoints; ";
  if (scope.tolerated_pairs.empty()) {
    stream << "the start state touches nothing, no contact tolerated";
  } else {
    stream << "tolerating only";
    for (std::size_t index = 0; index < scope.tolerated_pairs.size(); ++index) {
      stream << (index == 0U ? " " : ", ") << scope.tolerated_pairs[index].first <<
        " against " << scope.tolerated_pairs[index].second;
    }
    stream << ", never more than " << kToleratedDepthGrowthM * 1000.0 <<
      " mm deeper than at the start, in a local scene copy (move_group's matrix untouched)";
  }
  stream << "; " << scene.getWorld()->size() <<
    " world object(s), every other pair forbidden; standoff collision-free under the "
    "unmodified matrix";
  line.detail = stream.str();
  line.tolerated_pairs = scope.tolerated_pairs;
  line.planned = true;
  return line;
}

GraspEscapeLine plan_grasp_escape_from_snapshot(
  const moveit_msgs::msg::PlanningScene & snapshot, const moveit::core::RobotState & measured,
  const moveit::core::JointModelGroup * group, const moveit::core::LinkModel * link,
  const GraspEscape & escape, double cartesian_step_m, double minimum_fraction)
{
  const auto build = make_race_scene(measured.getRobotModel(), snapshot);
  if (!build.scene) {
    GraspEscapeLine line;
    line.detail = "the grasp escape refuses move_group's scene snapshot: " + build.refusal;
    return line;
  }
  moveit::core::RobotState start(build.scene->getCurrentState());
  start.setVariablePositions(measured.getVariablePositions());
  start.update();
  return plan_grasp_escape_line(
    *build.scene, start, group, link, escape, cartesian_step_m, minimum_fraction);
}

}  // namespace restocker_task_executor
