// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace restocker_task_executor
{

// Card 086 stage 1 (CMB-SPEC-13): the campaign's attempt registry. One record per outbound goal
// attempt whose motion outcome is not yet proven. While any record is unresolved the campaign
// sends nothing. This header is the pure state machine; the node owns the action clients and
// feeds it evidence. Stage 1 scopes ownership to the campaign process: a public endpoint client
// that bypasses the campaign is NOT excluded by it (cross-endpoint exclusion is later work).

enum class MotionEndpoint : std::uint8_t
{
  kLane,
  kTray,
  kViewpoint,
  kRestock,
};

[[nodiscard]] const char * to_string(MotionEndpoint endpoint) noexcept;

enum class UnresolvedCondition : std::uint8_t
{
  // The admission wait expired with no reply: the goal may be in flight (S1).
  kAdmissionUnresolved,
  // The goal was admitted but no delivered terminal settled it (S2, S3).
  kResultUnresolved,
  // A cancel by exact identity was requested and its terminal has not arrived (S2, S4).
  kSettlementPendingCancel,
};

[[nodiscard]] const char * to_string(UnresolvedCondition condition) noexcept;

// action_msgs/GoalInfo UUID bytes, the exact identity an attempt is keyed by once admitted.
using AttemptGoalId = std::array<std::uint8_t, 16>;

[[nodiscard]] std::string goal_id_text(const AttemptGoalId & goal_id);

// What one piece of evidence said. Only the rows of the spec's settlement table settle.
struct MotionEvidence
{
  enum class Kind : std::uint8_t
  {
    // An explicit admission rejection for the exact attempt: nothing was commanded.
    kAdmissionRejected,
    // A delivered terminal result for the exact goal; the payload flags below apply.
    kDeliveredTerminal,
    // Everything that proves nothing: an admission or result wait that expired, a result code
    // that is not a delivered terminal (UNKNOWN), a cancel answered "goal unknown", a quiet
    // joint-state sample, another endpoint's status, a world revision, elapsed backoff.
    kNonSettling,
  };

  Kind kind{Kind::kNonSettling};
  // Delivered terminal success or arrival: the controllers reported success for this attempt.
  bool arrived{false};
  // The payload's own proof that no motion was submitted.
  bool motion_definitely_not_started{false};
  // The payload's backend terminal-stop flag. Trustworthy for a MoveIt TIMED_OUT only since
  // stage 1a (the adapter no longer promotes it).
  bool execution_reached_terminal_stop{false};
  // The payload says a descendant goal may still be admitted or running (a parent terminal
  // that returned without awaiting its child): the parent attempt does not settle.
  bool descendant_unsettled{false};
  // What the evidence was, for the receipt ("late admission", "canceled terminal", ...).
  std::string note;
};

// The settlement table of CMB-SPEC-13 as a pure function: does this evidence prove the attempt
// is over and the motion stopped or never started?
[[nodiscard]] bool settles_attempt(const MotionEvidence & evidence) noexcept;

class UnresolvedMotionRegistry
{
public:
  // The send generation. Monotonic and never reused, so evidence for a retired attempt cannot
  // clear a newer one.
  using AttemptId = std::uint64_t;

  enum class Result : std::uint8_t
  {
    kSettled,
    kStillUnresolved,
    // No live record carries that identity: the evidence is for a retired or unknown attempt and
    // changes nothing.
    kUnknownAttempt,
  };

  struct Record
  {
    AttemptId id{0U};
    MotionEndpoint endpoint{MotionEndpoint::kLane};
    UnresolvedCondition condition{UnresolvedCondition::kAdmissionUnresolved};
    std::string label;
    std::optional<AttemptGoalId> goal_id;
    // Every evidence fragment received for this attempt, in arrival order.
    std::vector<std::string> fragments;
  };

  // Registers an attempt as unresolved.
  AttemptId open(MotionEndpoint endpoint, UnresolvedCondition condition, std::string label);

  // Late admission for a recorded attempt: the exact goal identity becomes known and the record
  // grows. It never clears anything (S4).
  Result bind_goal(AttemptId id, const AttemptGoalId & goal_id);

  Result set_condition(AttemptId id, UnresolvedCondition condition);

  // Routes evidence to one attempt. Idempotent: duplicate evidence settles once, evidence for a
  // retired or unknown attempt is a no-op.
  Result apply(AttemptId id, const MotionEvidence & evidence);

  // Routes evidence by exact goal UUID; a UUID no live record carries is a no-op.
  Result apply_for_goal(const AttemptGoalId & goal_id, const MotionEvidence & evidence);

  [[nodiscard]] bool unresolved() const noexcept {return !records_.empty();}
  [[nodiscard]] std::size_t size() const noexcept {return records_.size();}
  [[nodiscard]] const Record * find(AttemptId id) const noexcept;
  [[nodiscard]] std::vector<AttemptId> ids() const;

  // "motion_unresolved: 2 attempt(s): ..." naming every retained identity, or empty when clear.
  [[nodiscard]] std::string describe() const;

  // One entry per live record: "<endpoint>#<id> <condition> goal=<uuid> (<label>)".
  [[nodiscard]] std::vector<std::string> describe_each() const;

private:
  AttemptId next_id_{1U};
  std::map<AttemptId, Record> records_;
  std::set<AttemptId> retired_;
};

}  // namespace restocker_task_executor
