// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Milestone 10 §6 (Card 046): a straight line that comes back partial is retried at a finer
// Cartesian step before it becomes a refusal. `CartesianInterpolator` makes one zero-timeout IK
// attempt per waypoint, so a stop can be a single attempt that did not converge rather than a
// line no configuration can follow — Card 073's recorded fixture stopped at 92.7374 % at 0.005 m
// and completed 100 % in 1/1 replay at 0.001 m and 13/13 at 0.0005 m. The ladder below runs inside
// the segment's existing planning budget and never turns a partial path into an accepted one.

#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

namespace restocker_task_executor
{

// The refinement ladder of the shipped 0.005 m step: 5x finer, then 10x finer (§6, Card 046).
constexpr double kLinearRefinementStepFineM = 0.001;
constexpr double kLinearRefinementStepFinerM = 0.0005;
// Two refinements maximum; a step that is not strictly finer than the last one tried is skipped,
// so a coarser step can never be mistaken for a refinement.
constexpr std::size_t kMaximumLinearRefinements = 2U;

// One run of the same line at one step.
struct LinearPathAttempt
{
  double step_m{0.0};
  double fraction{0.0};
  moveit_msgs::msg::MoveItErrorCodes error_code;
  moveit_msgs::msg::RobotTrajectory path;
};

struct LinearPathRefinement
{
  // Ordered, oldest first. `attempts.front()` is the goal's own step, so a refusal describes the
  // line exactly as it did before refinement existed.
  std::vector<LinearPathAttempt> attempts;
  // Index of the first attempt that met the bar; unset when none did.
  std::optional<std::size_t> accepted;
  // The bar the attempts were judged against, for the receipt.
  double minimum_fraction{0.0};
  // Attempts may be empty if admission was already closed. A response received after either
  // boundary is recorded but never accepted.
  bool budget_spent{false};
  bool canceled{false};

  [[nodiscard]] bool complete() const {return accepted.has_value();}
  [[nodiscard]] const LinearPathAttempt & primary() const {return attempts.front();}
  [[nodiscard]] const LinearPathAttempt & executed() const
  {
    return accepted ? attempts[*accepted] : attempts.back();
  }
  // The console receipt clause a refusal carries: every step tried and what each reached.
  [[nodiscard]] std::string receipt() const;
};

// Runs `compute` at `first_step_m` and then, while the fraction stays below `minimum_fraction`,
// at each refinement step in `refinement_steps_m` that is strictly finer than the last one tried
// and that `budget_remaining` still allows. Check budget/cancellation before and after every
// synchronous call, including the first; an in-flight call itself cannot be interrupted here.
// The first successful, finite fraction meeting the bar within budget is accepted. The caller
// must handle budget/cancellation before accessing primary(), since no attempt may have run.
using ComputeLinearPath = std::function<double (
      double step_m, moveit_msgs::msg::RobotTrajectory & path,
      moveit_msgs::msg::MoveItErrorCodes & error_code)>;

LinearPathRefinement refine_linear_path(
  double first_step_m, double minimum_fraction, const ComputeLinearPath & compute,
  const std::function<bool()> & budget_remaining,
  std::vector<double> refinement_steps_m = {
    kLinearRefinementStepFineM, kLinearRefinementStepFinerM
  },
  const std::function<bool()> & cancel_requested = [] () {return false;});

}  // namespace restocker_task_executor
