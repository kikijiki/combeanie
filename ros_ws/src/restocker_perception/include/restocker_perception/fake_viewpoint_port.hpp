// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>

#include "restocker_perception/viewpoint_port.hpp"

namespace restocker_perception
{

// A deterministic ViewpointPort. It never spins, never reads TF, and completes only when the test
// says so, so a single-threaded test owns the exact interleaving.
//
// Installed rather than confined to this package's tests because the survey primitive that
// consumes it lives in restocker_task_executor, which must drive a viewpoint boundary without a
// robot.
//
// A fake accepts any mount transform, so tests against it cannot prove the aiming arithmetic: the
// mount it is given is the mount it uses. They prove the wiring: one completion per submission,
// refusals surfaced, cancellation observed.
class FakeViewpointPort final : public ViewpointPort
{
public:
  struct Submission
  {
    ViewpointCorrelation correlation;
    CameraViewpoint viewpoint;
    CompletionCallback callback;
  };

  // Supply the mount the fake resolves through. Without one, every resolution completes
  // kMountUnavailable, which is what a robot that has not published its description does.
  void set_mount(WristCameraMount mount) {mount_.emplace(std::move(mount));}
  void clear_mount() noexcept {mount_.reset();}

  [[nodiscard]] bool ready() const override {return ready_ && mount_.has_value();}
  void set_ready(bool value) noexcept {ready_ = value;}

  // Force the next submission to be refused, as a busy or unavailable port would.
  void refuse_next(ViewpointSubmitStatus status, std::string detail)
  {
    refusal_.emplace(ViewpointSubmitResult{status, std::move(detail)});
  }

  [[nodiscard]] ViewpointSubmitResult submit(
    ViewpointCorrelation correlation, CameraViewpoint viewpoint,
    CompletionCallback callback) override
  {
    ++submit_attempts;
    if (refusal_) {
      auto refusal = std::move(*refusal_);
      refusal_.reset();
      return refusal;
    }
    if (!callback) {
      return {ViewpointSubmitStatus::kInvalidRequest,
        "viewpoint submission requires a completion callback"};
    }
    if (outstanding_) {
      return {ViewpointSubmitStatus::kBusy, "fake viewpoint port already owns a request"};
    }
    outstanding_.emplace(Submission{correlation, std::move(viewpoint), std::move(callback)});
    return {ViewpointSubmitStatus::kAccepted, {}};
  }

  void cancel() noexcept override {++cancel_requests;}

  [[nodiscard]] bool outstanding() const noexcept {return outstanding_.has_value();}
  [[nodiscard]] const Submission & submission() const {return *outstanding_;}

  // Resolve the outstanding request through the configured mount, exactly as the real port would.
  void complete_with_mount()
  {
    auto submission = take_outstanding();
    ViewpointCompletion completion;
    completion.correlation = submission.correlation;
    if (!mount_) {
      completion.outcome = ViewpointOutcome::kMountUnavailable;
      completion.detail = "fake viewpoint port was given no mount transform";
      submission.callback(std::move(completion));
      return;
    }
    completion.tool0_from_optical = mount_->tool0_from_optical().transform();
    Result<Tool0ViewpointGoal> goal = mount_->tool0_goal_for(submission.viewpoint);
    if (!goal) {
      completion.outcome = ViewpointOutcome::kInvalidRequest;
      completion.detail = goal.error().detail;
      submission.callback(std::move(completion));
      return;
    }
    completion.outcome = ViewpointOutcome::kResolved;
    completion.goal = std::move(goal.value());
    submission.callback(std::move(completion));
  }

  // Deliver a chosen non-success outcome for the outstanding request.
  void complete_with_failure(ViewpointOutcome outcome, std::string detail = {})
  {
    auto submission = take_outstanding();
    ViewpointCompletion completion;
    completion.correlation = submission.correlation;
    completion.outcome = outcome;
    completion.detail = std::move(detail);
    submission.callback(std::move(completion));
  }

  std::size_t submit_attempts{0U};
  std::size_t cancel_requests{0U};

private:
  [[nodiscard]] Submission take_outstanding()
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    return submission;
  }

  bool ready_{true};
  std::optional<WristCameraMount> mount_;
  std::optional<Submission> outstanding_;
  std::optional<ViewpointSubmitResult> refusal_;
};

}  // namespace restocker_perception
