// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>

#include "restocker_reasoner/recovery_advice.hpp"

namespace restocker_reasoner
{

enum class RecoveryAdviceSubmitStatus : std::uint8_t
{
  kAccepted,
  // The query itself is malformed. Never retryable.
  kInvalidRequest,
  // A question is already outstanding.
  kBusy,
  // No backend, shutting down, or the circuit breaker is open.
  kUnavailable,
};

struct RecoveryAdviceSubmitResult
{
  RecoveryAdviceSubmitStatus status{RecoveryAdviceSubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == RecoveryAdviceSubmitStatus::kAccepted;
  }
};

// What a backend said, before anything has looked at it.
//
// The port never validates or interprets: it carries bytes, or the reason there are none. An
// expired deadline, a transport failure, a backend that is not running and a nonsense answer all
// arrive the same way; only `transport_detail` (for the audit record) tells them apart. "No
// recommendation" must be a single behaviour.
struct RecoveryAdviceCompletion
{
  std::string request_id;
  std::uint64_t goal_generation{0U};
  // True only when a backend returned a response body within the deadline.
  bool responded{false};
  // The backend's answer, verbatim and bounded. Empty unless responded.
  std::string document;
  // Why there is no document. Empty when responded.
  std::string transport_detail;
  std::chrono::milliseconds latency{0};
  // Which backend produced this, for the audit record.
  std::string backend;
};

// Ask an optional advisory backend to classify one failure.
//
// Implementations must invoke the completion callback exactly once per accepted submission, from
// a thread the coordinator does not own, and must not call back inline from submit(). No
// implementation may block the caller of submit() on the network.
class RecoveryAdvisorPort
{
public:
  using CompletionCallback = std::function<void (RecoveryAdviceCompletion)>;

  virtual ~RecoveryAdvisorPort() = default;

  // False when the backend is known to be unusable. Submission is still allowed and completes
  // as not-responded rather than blocking.
  [[nodiscard]] virtual bool ready() const = 0;

  [[nodiscard]] virtual RecoveryAdviceSubmitResult submit(
    RecoveryQuery query, CompletionCallback callback) = 0;

  // Abandon anything outstanding. The completion still arrives, as not-responded.
  virtual void cancel() noexcept = 0;
};

// A one-slot letterbox between an advisor thread and the coordinator's pump.
//
// Advisory results do not travel through the coordinator inbox. That inbox carries evidence, and
// pressure on it is a safety fault that inhibits the task: an advisory answer must not occupy a
// slot a motion completion needed, and a late one must be droppable without a trace. The advisor
// writes here instead, the pump reads at the instant it would act, and anything that does not
// match the current question is discarded.
class RecoveryAdviceMailbox
{
public:
  void deposit(RecoveryAdviceCompletion completion);

  // Returns the deposited completion only when it answers exactly this request for exactly this
  // goal generation, and empties the slot either way: a superseded answer is never held for a
  // later question.
  [[nodiscard]] std::optional<RecoveryAdviceCompletion> take(
    std::string_view request_id, std::uint64_t goal_generation);

  void clear();

  [[nodiscard]] bool occupied() const;

private:
  mutable std::mutex mutex_;
  std::optional<RecoveryAdviceCompletion> slot_;
};

}  // namespace restocker_reasoner
