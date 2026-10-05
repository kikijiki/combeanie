// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Milestone 10 §6 (Card 046): the linear-path refinement ladder.

#include "restocker_task_executor/linear_path_refinement.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace restocker_task_executor
{

std::string LinearPathRefinement::receipt() const
{
  std::ostringstream receipt;
  receipt << std::fixed << std::setprecision(6);
  receipt << "linear refinement: ";
  if (attempts.empty()) {
    receipt << "no step was tried";
  }
  bool first = true;
  for (const auto & attempt : attempts) {
    if (!first) {
      receipt << ", ";
    }
    first = false;
    receipt << "step " << attempt.step_m << " m reached ";
    if (attempt.fraction < 0.0) {
      // MoveIt's failed-request sentinel: nothing of the line was computed.
      receipt << "its error sentinel (" << attempt.error_code.val << ")";
    } else {
      receipt << attempt.fraction;
    }
  }
  receipt << " (bar " << minimum_fraction << ")";
  if (budget_spent) {
    receipt << "; refinement stopped with the segment's planning budget spent";
  }
  if (canceled) {
    receipt << "; refinement stopped after cancellation";
  }
  return receipt.str();
}

LinearPathRefinement refine_linear_path(
  double first_step_m, double minimum_fraction, const ComputeLinearPath & compute,
  const std::function<bool()> & budget_remaining, std::vector<double> refinement_steps_m,
  const std::function<bool()> & cancel_requested)
{
  LinearPathRefinement refinement;
  refinement.minimum_fraction = minimum_fraction;

  const auto stopped = [&refinement, &budget_remaining, &cancel_requested]() {
    // Cancellation wins if both boundaries closed while the service call was in flight.
    refinement.canceled = cancel_requested();
    refinement.budget_spent = !budget_remaining();
    return refinement.canceled || refinement.budget_spent;
  };
  const auto run = [&refinement, &compute, &stopped, minimum_fraction](double step_m) {
    if (stopped()) {
      return true;
    }
    LinearPathAttempt attempt;
    attempt.step_m = step_m;
    attempt.error_code.val = moveit_msgs::msg::MoveItErrorCodes::UNDEFINED;
    attempt.fraction = compute(step_m, attempt.path, attempt.error_code);
    refinement.attempts.push_back(std::move(attempt));
    if (stopped()) {
      return true;
    }
    const auto & result = refinement.attempts.back();
    if (result.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
      std::isfinite(result.fraction) && result.fraction >= minimum_fraction &&
      result.fraction <= 1.0)
    {
      refinement.accepted = refinement.attempts.size() - 1U;
    }
    return refinement.complete();
  };

  if (run(first_step_m)) {
    return refinement;
  }

  double last_step_m = first_step_m;
  std::size_t refinements = 0U;
  for (const double step_m : refinement_steps_m) {
    if (refinements >= kMaximumLinearRefinements) {
      break;
    }
    // Only a strictly finer step is a refinement; a coarser one never buys a second opinion.
    if (!std::isfinite(step_m) || !(step_m > 0.0 && step_m < last_step_m)) {
      continue;
    }
    if (run(step_m)) {
      break;
    }
    last_step_m = step_m;
    ++refinements;
  }
  return refinement;
}

}  // namespace restocker_task_executor
