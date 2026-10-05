// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <utility>

#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{

// Deterministic MotionPort for driver tests. It does not plan or spin, and completes only when
// the test says so, so a single-threaded test controls the interleaving.
class FakeMotionPort final : public MotionPort
{
public:
  struct Submission
  {
    OperationCorrelation correlation;
    MotionGoal goal;
    CompletionCallback callback;
  };

  [[nodiscard]] bool ready() const override {return ready_;}
  void set_ready(bool value) noexcept {ready_ = value;}

  // Force the next submission to be refused, as a busy or unavailable backend would.
  void refuse_next(MotionSubmitStatus status, std::string detail)
  {
    refusal_.emplace(MotionSubmitResult{status, std::move(detail)});
  }

  [[nodiscard]] MotionSubmitResult submit(
    OperationCorrelation correlation, MotionGoal goal, CompletionCallback callback) override
  {
    ++submit_attempts;
    if (refusal_) {
      auto refusal = std::move(*refusal_);
      refusal_.reset();
      return refusal;
    }
    if (outstanding_) {
      return {MotionSubmitStatus::kBusy, "fake motion port already owns a goal"};
    }
    outstanding_.emplace(Submission{correlation, std::move(goal), std::move(callback)});
    return {MotionSubmitStatus::kAccepted, {}};
  }

  void cancel() noexcept override {++cancel_requests;}

  [[nodiscard]] bool outstanding() const noexcept {return outstanding_.has_value();}
  [[nodiscard]] const Submission & submission() const {return *outstanding_;}

  // Deliver the terminal result for the outstanding goal. execution_reached_terminal_stop
  // defaults to false, as for a port that never saw the trajectory end at the controllers; the
  // real port sets it only for a backend-reported terminal stop.
  void complete(
    MotionOutcome outcome, std::string detail = {}, bool execution_reached_terminal_stop = false,
    bool planner_refused_the_goal = false)
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    submission.callback(
      MotionCompletion{
        submission.correlation, outcome, std::move(detail), execution_reached_terminal_stop,
        planner_refused_the_goal});
  }

  // The planner was handed the goal and answered no. Separate from complete() because it is the
  // only planning failure that is a verdict on the pose; the real port reports it only for a
  // refusal that reached the planner.
  // Card 062: the port ran the goal's grasp escape leg before this outcome.
  void complete_after_grasp_escape(
    MotionOutcome outcome, std::string detail = {}, bool planner_refused_the_goal = false)
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    MotionCompletion completion{
      submission.correlation, outcome, std::move(detail), false, planner_refused_the_goal};
    completion.grasp_escape_executed = true;
    submission.callback(std::move(completion));
  }

  // The backend had already been handed a trajectory (MoveIt execute() was called) when the port
  // gave up with this outcome, as an execute-phase COMMUNICATION_FAILURE does.
  void complete_after_execute(
    MotionOutcome outcome, std::string detail = {}, bool execution_reached_terminal_stop = false)
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    MotionCompletion completion{
      submission.correlation, outcome, std::move(detail), execution_reached_terminal_stop, false};
    completion.submitted_to_backend = true;
    submission.callback(std::move(completion));
  }

  void complete_planner_refusal(std::string detail)
  {
    complete(MotionOutcome::kPlanningFailed, std::move(detail), false, true);
  }

  std::size_t submit_attempts{0U};
  std::size_t cancel_requests{0U};

private:
  bool ready_{true};
  std::optional<Submission> outstanding_;
  std::optional<MotionSubmitResult> refusal_;
};

}  // namespace restocker_task_executor
