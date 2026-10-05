// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <string>

#include "restocker_task_executor/recovery_classification.hpp"

namespace restocker_task_executor
{

// What the campaign node knows first-hand when a survey or confirm action fails
// (Milestone 10 §6, Card 052). The context builder maps this evidence onto Card 051's
// classifier; nothing here decides whether a check passes, only what is established after
// one said no.
struct SurveyFailureEvidence
{
  // True when the action delivered a terminal result (succeed, canceled, or abort with a
  // result) that this campaign call waited for. False for an admission timeout, a rejection,
  // a server that never appeared, or a result wait that outlived the cancel's own wait —
  // in every one of those the arm's state cannot be read from this action.
  bool terminal_result_received{false};
  // Propagated from the action result: the attempt's own machinery proves no joint command
  // was issued (a refused plan, a pre-motion rejection, a measure-only survey).
  bool motion_definitely_not_started{false};
  // Propagated from the action result: the backend established a terminal stop at the
  // controllers (or the aim arrived — controllers reported success).
  bool execution_reached_terminal_stop{false};
  // The campaign's held-state rule: true when no transfer goal has ever been accepted this
  // run, or the most recent accepted transfer ended in an explicit release
  // (STATUS_SUCCEEDED or STATUS_SKIPPED_RECOVERABLE). False after any other accepted
  // transfer's terminal — the held-object state is then unknown, exactly as Card 051's
  // classification requires.
  bool transfer_release_established{false};
  // False when the measurement that frames this failure could not be taken (the world
  // service itself failed) or when a confirmed identity cannot be bound to the world-state
  // snapshot: the world the next attempt would rely on is then not re-observable, which is
  // an UNSAFE trigger, never a guess.
  bool world_reobservable{true};
  // Why the campaign asked (the action/lane/station and the detail), quoted into the receipt.
  std::string cause;
};

// Fail-closed mapping onto Card 051's classifier: no terminal result leaves the arm state
// unknown; a terminal result with neither evidence flag leaves it unestablished; the
// campaign holds no first-hand contact or hardware-fault channel for a survey action, so
// those two triggers stay false under the same padded-scene standard §6 already applies to
// every other segment (stated in the implementation, not re-decided per call site).
[[nodiscard]] RecoveryContext survey_recovery_context(const SurveyFailureEvidence & evidence);

}  // namespace restocker_task_executor
