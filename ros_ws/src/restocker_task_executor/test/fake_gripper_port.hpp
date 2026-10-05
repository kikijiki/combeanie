// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <utility>

#include "restocker_task_executor/gripper_port.hpp"

namespace restocker_task_executor
{

// Deterministic GripperPort for driver tests. It has no controller and no spin, and completes only
// when the test says so, so a single-threaded test controls the interleaving.
class FakeGripperPort final : public GripperPort
{
public:
  struct Submission
  {
    OperationCorrelation correlation;
    GripperGoal goal;
    CompletionCallback callback;
  };

  [[nodiscard]] bool ready() const override {return ready_;}
  void set_ready(bool value) noexcept {ready_ = value;}

  // Force the next submission to be refused, as a busy or unavailable backend would.
  void refuse_next(GripperSubmitStatus status, std::string detail)
  {
    refusal_.emplace(GripperSubmitResult{status, std::move(detail)});
  }

  // Refuse the next `count` submissions the same way.
  void refuse_next_n(std::size_t count, GripperSubmitStatus status, std::string detail)
  {
    refusal_.emplace(GripperSubmitResult{status, std::move(detail)});
    repeat_refusals_ = count > 0U ? count - 1U : 0U;
  }

  [[nodiscard]] GripperSubmitResult submit(
    OperationCorrelation correlation, GripperGoal goal, CompletionCallback callback) override
  {
    ++submit_attempts;
    if (refusal_ && repeat_refusals_ > 0U) {
      --repeat_refusals_;
      return *refusal_;
    }
    if (refusal_) {
      auto refusal = std::move(*refusal_);
      refusal_.reset();
      return refusal;
    }
    if (outstanding_) {
      return {GripperSubmitStatus::kBusy, "fake gripper port already owns a goal"};
    }
    outstanding_.emplace(Submission{correlation, std::move(goal), std::move(callback)});
    return {GripperSubmitStatus::kAccepted, {}};
  }

  void cancel() noexcept override {++cancel_requests;}

  [[nodiscard]] bool outstanding() const noexcept {return outstanding_.has_value();}
  [[nodiscard]] const Submission & submission() const {return *outstanding_;}

  // Deliver the terminal result for the outstanding goal. The measurement stays absent unless the
  // test supplies one, as for a port that never read /joint_states.
  void complete(
    GripperOutcome outcome, std::string detail = {},
    std::optional<GripperFingerState> measured = std::nullopt)
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    submission.callback(
      GripperCompletion{
        submission.correlation, outcome, std::move(detail), std::move(measured)});
  }

  // Complete as the real port would after reading the given sample: succeeded only when both
  // fingers landed inside the submitted goal's tolerance, kPositionNotVerified otherwise.
  void complete_with_measurement(const GripperFingerState & measured)
  {
    const auto & goal = outstanding_->goal;
    const bool verified =
      fingers_at_target(measured, goal.target_position_m, goal.position_tolerance_m);
    complete(
      verified ? GripperOutcome::kSucceeded : GripperOutcome::kPositionNotVerified,
      verified ? "fake fingers verified" : "fake fingers missed the target", measured);
  }

  std::size_t submit_attempts{0U};
  std::size_t cancel_requests{0U};

private:
  bool ready_{true};
  std::optional<Submission> outstanding_;
  std::optional<GripperSubmitResult> refusal_;
  std::size_t repeat_refusals_{0U};
};

}  // namespace restocker_task_executor
