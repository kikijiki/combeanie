// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>

#include "restocker_task_executor/attachment_port.hpp"

namespace restocker_task_executor
{

// Deterministic AttachmentPort for driver tests. It has no service and no spin, and completes only
// when the test says so, so a single-threaded test controls the interleaving.
class FakeAttachmentPort final : public AttachmentPort
{
public:
  struct Submission
  {
    OperationCorrelation correlation;
    AttachmentGoal goal;
    CompletionCallback callback;
  };

  [[nodiscard]] bool ready() const override {return ready_;}
  void set_ready(bool value) noexcept {ready_ = value;}

  // Force the next submission to be refused, as a busy or unavailable backend would.
  void refuse_next(AttachmentSubmitStatus status, std::string detail)
  {
    refusal_.emplace(AttachmentSubmitResult{status, std::move(detail)});
  }

  [[nodiscard]] AttachmentSubmitResult submit(
    OperationCorrelation correlation, AttachmentGoal goal, CompletionCallback callback) override
  {
    ++submit_attempts;
    if (refusal_) {
      auto refusal = std::move(*refusal_);
      refusal_.reset();
      return refusal;
    }
    if (outstanding_) {
      return {AttachmentSubmitStatus::kBusy, "fake attachment port already owns a transaction"};
    }
    outstanding_.emplace(Submission{correlation, std::move(goal), std::move(callback)});
    return {AttachmentSubmitStatus::kAccepted, {}};
  }

  void cancel() noexcept override {++cancel_requests;}

  [[nodiscard]] bool outstanding() const noexcept {return outstanding_.has_value();}
  [[nodiscard]] const Submission & submission() const {return *outstanding_;}

  // Deliver the terminal result for the outstanding transaction, as the real port would. The
  // lease flag defaults to the real port's value for this outcome: the lease is retained whenever
  // the physical and semantic worlds cannot be shown to agree. Physical-only success also retains
  // the lease and surfaces the token the semantic half must reuse.
  void complete(
    AttachmentOutcome outcome, std::string detail = {}, std::uint64_t world_revision = 0)
  {
    const auto effect = attachment_world_effect(outcome);
    const bool intentional_physical_only =
      outstanding_ && outstanding_->goal.scope == AttachmentScope::kPhysicalOnly &&
      outcome == AttachmentOutcome::kSucceeded;
    const bool released = !intentional_physical_only &&
      (effect == AttachmentWorldEffect::kNoneApplied ||
      effect == AttachmentWorldEffect::kFullyApplied);
    std::string retained;
    AttachmentStamp released_at;
    if (outstanding_) {
      released_at = outstanding_->goal.released_at;
      if (intentional_physical_only) {
        retained = "fake-retained-lease";
        if (released_at.zero()) {
          released_at = AttachmentStamp{1, 0U};
        }
      }
    }
    complete_with(
      outcome, world_revision, released, std::move(detail), std::move(retained), released_at);
  }

  // Deliver a completion with an explicit lease disposition, for the case where a transaction
  // succeeded but the projector could not be un-frozen.
  void complete_with(
    AttachmentOutcome outcome, std::uint64_t world_revision, bool lease_released,
    std::string detail = {}, std::string retained_lease_token = {},
    AttachmentStamp released_at = {})
  {
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    submission.callback(
      AttachmentCompletion{
        submission.correlation, outcome, world_revision, lease_released,
        std::move(retained_lease_token), released_at, std::move(detail)});
  }

  std::size_t submit_attempts{0U};
  std::size_t cancel_requests{0U};

private:
  bool ready_{true};
  std::optional<Submission> outstanding_;
  std::optional<AttachmentSubmitResult> refusal_;
};

}  // namespace restocker_task_executor
