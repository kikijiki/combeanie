// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/viewpoint_survey.hpp"

#include <string>
#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] SurveyOutcome outcome_for(MotionOutcome outcome) noexcept
{
  switch (outcome) {
    case MotionOutcome::kSucceeded:
      return SurveyOutcome::kArrived;
    case MotionOutcome::kPlanningFailed:
      return SurveyOutcome::kPlanningFailed;
    case MotionOutcome::kExecutionFailed:
      return SurveyOutcome::kExecutionFailed;
    case MotionOutcome::kCanceled:
      return SurveyOutcome::kCanceled;
    case MotionOutcome::kTimedOut:
      return SurveyOutcome::kTimedOut;
    case MotionOutcome::kUnavailable:
      return SurveyOutcome::kUnavailable;
  }
  return SurveyOutcome::kUnavailable;
}

[[nodiscard]] SurveyOutcome outcome_for(restocker_perception::ViewpointOutcome outcome) noexcept
{
  switch (outcome) {
    case restocker_perception::ViewpointOutcome::kResolved:
      return SurveyOutcome::kArrived;
    case restocker_perception::ViewpointOutcome::kMountUnavailable:
      return SurveyOutcome::kViewpointUnresolved;
    case restocker_perception::ViewpointOutcome::kInvalidRequest:
      return SurveyOutcome::kInvalidRequest;
    case restocker_perception::ViewpointOutcome::kCanceled:
      return SurveyOutcome::kCanceled;
    case restocker_perception::ViewpointOutcome::kUnavailable:
      return SurveyOutcome::kUnavailable;
  }
  return SurveyOutcome::kUnavailable;
}

}  // namespace

const char * survey_outcome_name(SurveyOutcome outcome) noexcept
{
  switch (outcome) {
    case SurveyOutcome::kArrived:
      return "arrived";
    case SurveyOutcome::kViewpointUnresolved:
      return "viewpoint unresolved";
    case SurveyOutcome::kPlanningFailed:
      return "planning failed";
    case SurveyOutcome::kExecutionFailed:
      return "execution failed";
    case SurveyOutcome::kCanceled:
      return "canceled";
    case SurveyOutcome::kTimedOut:
      return "timed out";
    case SurveyOutcome::kUnavailable:
      return "unavailable";
    case SurveyOutcome::kInvalidRequest:
      return "invalid request";
  }
  return "unknown";
}

bool survey_definitely_not_started(SurveyOutcome outcome) noexcept
{
  switch (outcome) {
    case SurveyOutcome::kViewpointUnresolved:
    case SurveyOutcome::kInvalidRequest:
      // The viewpoint never became a tool goal, so nothing was ever submitted for planning.
      return true;
    case SurveyOutcome::kPlanningFailed:
      // Uses the motion port's taxonomy so the two cannot drift.
      return motion_definitely_not_started(MotionOutcome::kPlanningFailed);
    case SurveyOutcome::kUnavailable:
      return motion_definitely_not_started(MotionOutcome::kUnavailable);
    case SurveyOutcome::kArrived:
    case SurveyOutcome::kExecutionFailed:
    case SurveyOutcome::kCanceled:
    case SurveyOutcome::kTimedOut:
      return false;
  }
  return false;
}

ViewpointSurvey::ViewpointSurvey(
  restocker_perception::ViewpointPort & viewpoints, MotionPort & motion,
  ViewpointSurveyConfig config)
: viewpoints_(viewpoints), motion_(motion), config_(config)
{
}

bool ViewpointSurvey::ready() const
{
  return viewpoints_.ready() && motion_.ready();
}

SurveySubmitResult ViewpointSurvey::submit(
  OperationCorrelation correlation, restocker_perception::CameraViewpoint viewpoint,
  ViewpointSurveyConfig overrides, CompletionCallback callback)
{
  if (!callback) {
    return {SurveySubmitStatus::kInvalidRequest, "a survey requires a completion callback"};
  }
  if (viewpoint.pose.frame_id.empty()) {
    return {SurveySubmitStatus::kInvalidRequest,
      "a viewpoint must name the frame its camera pose is expressed in"};
  }
  if (!viewpoint.pose.pose.matrix().allFinite()) {
    return {SurveySubmitStatus::kInvalidRequest, "viewpoint pose is not finite"};
  }

  ViewpointSurveyConfig effective = config_;
  if (overrides.position_tolerance_m > 0.0) {
    effective.position_tolerance_m = overrides.position_tolerance_m;
  }
  if (overrides.orientation_tolerance_rad > 0.0) {
    effective.orientation_tolerance_rad = overrides.orientation_tolerance_rad;
  }
  if (overrides.velocity_scaling > 0.0) {
    effective.velocity_scaling = overrides.velocity_scaling;
  }
  if (overrides.acceleration_scaling > 0.0) {
    effective.acceleration_scaling = overrides.acceleration_scaling;
  }
  if (overrides.planning_time.count() > 0) {
    effective.planning_time = overrides.planning_time;
  }

  {
    std::scoped_lock lock(mutex_);
    if (active_) {
      return {SurveySubmitStatus::kBusy, "a survey is already outstanding"};
    }
    cancel_requested_ = false;
    resolved_goal_.reset();
    active_.emplace(Active{correlation, viewpoint, effective, std::move(callback)});
  }

  const restocker_perception::ViewpointSubmitResult submitted = viewpoints_.submit(
    restocker_perception::ViewpointCorrelation{correlation.operation_generation},
    std::move(viewpoint),
    [this](restocker_perception::ViewpointCompletion completion) {
      on_viewpoint_resolved(std::move(completion));
    });
  if (submitted) {
    return {SurveySubmitStatus::kAccepted, {}};
  }

  // The viewpoint port refused, so nothing will call back for this survey. Release the slot and
  // report the refusal synchronously; a refused submission must not look like an accepted one.
  {
    std::scoped_lock lock(mutex_);
    active_.reset();
  }
  switch (submitted.status) {
    case restocker_perception::ViewpointSubmitStatus::kInvalidRequest:
      return {SurveySubmitStatus::kInvalidRequest, submitted.detail};
    case restocker_perception::ViewpointSubmitStatus::kBusy:
      return {SurveySubmitStatus::kBusy, submitted.detail};
    case restocker_perception::ViewpointSubmitStatus::kAccepted:
    case restocker_perception::ViewpointSubmitStatus::kUnavailable:
      break;
  }
  return {SurveySubmitStatus::kUnavailable, submitted.detail};
}

void ViewpointSurvey::cancel() noexcept
{
  {
    std::scoped_lock lock(mutex_);
    cancel_requested_ = true;
  }
  viewpoints_.cancel();
  motion_.cancel();
}

void ViewpointSurvey::on_viewpoint_resolved(
  restocker_perception::ViewpointCompletion completion)
{
  ViewpointSurveyConfig config;
  OperationCorrelation correlation;
  std::string label;
  bool cancelled = false;
  {
    std::scoped_lock lock(mutex_);
    if (!active_) {
      return;                       // a completion for a survey that is already finished
    }
    config = active_->config;
    correlation = active_->correlation;
    label = active_->viewpoint.label;
    cancelled = cancel_requested_;
    if (completion.outcome == restocker_perception::ViewpointOutcome::kResolved) {
      resolved_goal_ = completion.goal;
    }
  }

  if (completion.outcome != restocker_perception::ViewpointOutcome::kResolved) {
    finish(outcome_for(completion.outcome), std::move(completion.detail), std::nullopt, false);
    return;
  }
  if (cancelled) {
    // Cancelled between resolution and submission: report it here instead of submitting a segment
    // and cancelling it.
    finish(
      SurveyOutcome::kCanceled, "cancelled before the survey segment was submitted",
      completion.goal, false);
    return;
  }

  MotionGoal goal;
  goal.planning_frame_from_tool0 = completion.goal.pose.pose;
  goal.position_tolerance_m = config.position_tolerance_m;
  goal.orientation_tolerance_rad = config.orientation_tolerance_rad;
  goal.velocity_scaling = config.velocity_scaling;
  goal.acceleration_scaling = config.acceleration_scaling;
  goal.planning_time = config.planning_time;
  // A survey is free-space repositioning (no product in the jaws), so any collision-free path is
  // acceptable. It is the same kind of segment as every other free-space move, so it goes through
  // the same planning-scene authority gate.
  goal.path = MotionPathKind::kFreeSpace;
  goal.label = label.empty() ? std::string("survey viewpoint") : ("survey " + label);

  const MotionSubmitResult submitted = motion_.submit(
    correlation, std::move(goal),
    [this](MotionCompletion motion_completion) {
      on_motion_complete(std::move(motion_completion));
    });
  if (submitted) {
    return;
  }
  // kBusy means the motion port still owns an earlier goal, so the arm may be moving under it:
  // that is not a "never started" answer (Card 086 stage 1b review, same class as B1/S1).
  SurveyOutcome outcome = SurveyOutcome::kUnavailable;
  if (submitted.status == MotionSubmitStatus::kInvalidRequest) {
    outcome = SurveyOutcome::kInvalidRequest;
  } else if (submitted.status == MotionSubmitStatus::kBusy) {
    outcome = SurveyOutcome::kExecutionFailed;
  }
  finish(
    outcome, "the motion port refused the survey segment: " + submitted.detail, completion.goal,
    false);
}

void ViewpointSurvey::on_motion_complete(MotionCompletion completion)
{
  std::optional<restocker_perception::Tool0ViewpointGoal> goal;
  {
    std::scoped_lock lock(mutex_);
    if (!active_) {
      return;
    }
    goal = resolved_goal_;
  }
  // A not-started outcome class after the port handed a trajectory to the backend (an execute-phase
  // COMMUNICATION_FAILURE/CRASH arrives as kUnavailable) says nothing about the arm: report it as
  // an execution failure without a stop, which the lane/tray evidence flags read as UNSAFE
  // (Card 086 stage 1b review B1).
  auto outcome = outcome_for(completion.outcome);
  if (completion.submitted_to_backend && survey_definitely_not_started(outcome)) {
    outcome = SurveyOutcome::kExecutionFailed;
  }
  finish(
    outcome, std::move(completion.detail), goal, completion.execution_reached_terminal_stop);
}

void ViewpointSurvey::finish(
  SurveyOutcome outcome, std::string detail,
  const std::optional<restocker_perception::Tool0ViewpointGoal> & goal,
  bool execution_reached_terminal_stop)
{
  std::optional<Active> active;
  {
    std::scoped_lock lock(mutex_);
    if (!active_) {
      return;                       // one completion per accepted submission
    }
    active = std::move(active_);
    active_.reset();
    resolved_goal_.reset();
  }
  SurveyCompletion completion;
  completion.correlation = active->correlation;
  completion.outcome = outcome;
  completion.detail = std::move(detail);
  completion.commanded_viewpoint = std::move(active->viewpoint);
  completion.viewpoint_resolved = goal.has_value();
  if (goal) {
    completion.commanded_goal = *goal;
  }
  completion.execution_reached_terminal_stop = execution_reached_terminal_stop;
  active->callback(std::move(completion));
}

}  // namespace restocker_task_executor
