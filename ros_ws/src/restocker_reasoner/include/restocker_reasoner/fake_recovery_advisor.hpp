// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "restocker_reasoner/recovery_advisor_port.hpp"

namespace restocker_reasoner
{

// A RecoveryAdvisorPort that never opens a socket and answers only when a test says so, so tests
// need no model service. It lives beside the interface because the task executor's driver tests
// also use it. Header-only; nothing in the runtime links it.
class FakeRecoveryAdvisor final : public RecoveryAdvisorPort
{
public:
  struct Submission
  {
    RecoveryQuery query;
    CompletionCallback callback;
  };

  [[nodiscard]] bool ready() const override {return ready_;}

  void set_ready(bool ready) {ready_ = ready;}

  void refuse_next(RecoveryAdviceSubmitStatus status, std::string detail)
  {
    refusal_ = RecoveryAdviceSubmitResult{status, std::move(detail)};
  }

  [[nodiscard]] RecoveryAdviceSubmitResult submit(
    RecoveryQuery query, CompletionCallback callback) override
  {
    ++submit_attempts;
    if (refusal_) {
      auto refusal = *refusal_;
      refusal_.reset();
      return refusal;
    }
    if (outstanding_) {
      return {RecoveryAdviceSubmitStatus::kBusy, "a question is already outstanding"};
    }
    last_query_ = query;
    outstanding_ = Submission{std::move(query), std::move(callback)};
    if (armed_) {
      auto armed = std::move(*armed_);
      armed_.reset();
      complete(
        armed.responded, substitute_request_id(std::move(armed.document)),
        std::move(armed.transport_detail));
    }
    return {RecoveryAdviceSubmitStatus::kAccepted, {}};
  }

  void cancel() noexcept override {++cancel_requests;}

  [[nodiscard]] bool outstanding() const {return outstanding_.has_value();}
  [[nodiscard]] const RecoveryQuery & query() const {return last_query_;}

  // Answer the next question as soon as it is asked. Unlike a real port, this calls the
  // completion from inside submit(), which lets a single-threaded test cover an answer that was
  // already waiting when the decision was taken.
  //
  // Every occurrence of the literal {request_id} in the document is replaced with the request id
  // of the query; write a literal id to send a badly correlated answer.
  void answer_next(std::string document)
  {
    armed_ = Armed{true, std::move(document), {}};
  }

  void answer_next_with_nothing(std::string transport_detail)
  {
    armed_ = Armed{false, {}, std::move(transport_detail)};
  }

  // Answer the outstanding question with a document.
  void respond(std::string document)
  {
    complete(true, std::move(document), {});
  }

  // Answer with nothing, as a deadline, transport failure or absent backend would.
  void respond_with_nothing(std::string transport_detail)
  {
    complete(false, {}, std::move(transport_detail));
  }

  std::size_t submit_attempts{0U};
  std::size_t cancel_requests{0U};

private:
  [[nodiscard]] std::string substitute_request_id(std::string document) const
  {
    constexpr std::string_view kPlaceholder = "{request_id}";
    for (auto position = document.find(kPlaceholder); position != std::string::npos;
      position = document.find(kPlaceholder, position + last_query_.request_id.size()))
    {
      document.replace(position, kPlaceholder.size(), last_query_.request_id);
    }
    return document;
  }

  void complete(bool responded, std::string document, std::string transport_detail)
  {
    if (!outstanding_) {
      return;
    }
    auto submission = std::move(*outstanding_);
    outstanding_.reset();
    RecoveryAdviceCompletion completion;
    completion.request_id = submission.query.request_id;
    completion.goal_generation = submission.query.goal_generation;
    completion.responded = responded;
    completion.document = std::move(document);
    completion.transport_detail = std::move(transport_detail);
    completion.backend = "fake-recovery-advisor";
    submission.callback(std::move(completion));
  }

  struct Armed
  {
    bool responded{false};
    std::string document;
    std::string transport_detail;
  };

  bool ready_{true};
  RecoveryQuery last_query_;
  std::optional<Submission> outstanding_;
  std::optional<RecoveryAdviceSubmitResult> refusal_;
  std::optional<Armed> armed_;
};

}  // namespace restocker_reasoner
