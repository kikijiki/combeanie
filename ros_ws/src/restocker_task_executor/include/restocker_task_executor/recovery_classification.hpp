// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

namespace restocker_task_executor
{

// Graded recovery classification (Milestone 10 §6, Card 051). Every terminal outcome and every
// refusal is classified into exactly one of two classes, fail-closed: a condition that cannot be
// established as recoverable is unsafe. The class decides what the coordinator does next —
// climbing the recovery ladder (safe retreat, re-observe, retry with fresh evidence,
// alternative candidate, skip-and-revisit) versus latching for an operator — never whether a
// safety check passes; no check, tolerance, budget or age window is read here.
//
// Classes → rungs (the contract table of the spec, kept next to the code):
//
//   RECOVERABLE          → rung 1 safe retreat (if a trajectory may have run),
//                          rung 2 re-observe, rung 3 retry with fresh evidence
//                          (existing per-operation / per-goal budgets),
//                          rung 4 alternative candidate (existing fallthrough),
//                          rung 5 typed recoverable skip (no operator latch).
//   UNSAFE               → latch for the operator, exactly as before this card:
//                          kFault → kRequestOperator → motion inhibited.
//
// The unsafe triggers are the spec's four: real contact or collision, an unknown or lost held
// object, an arm state that cannot be shown stopped, a persistent hardware/controller fault —
// plus "the world is not re-observable", which makes any retry blind.
//
// Wiring note (Card 051 review): at the live boundary the driver populates `arm_stop` and
// `held_state` from first-hand stop evidence and the machine's fault; `contact_or_collision`
// and `persistent_hardware_fault` are second-hand covered on every reachable path rather than
// set here — real contact / an unreconcilable transaction maps to `kExternalInconsistency`
// (held Unknown), an unknown arm state to the stop-evidence refusal or `kRecoveryFailed`
// (arm Unknown/NotEstablished, pinned by RecoveryIsRefusedWhileTheRobotIsStillMoving), a dead
// backend to the `kUnavailable` refusal (arm NotEstablished), and the skip-reachable `kFault`
// boundary is only entered from pre-motion/recovery states, so the ladder cannot start after
// an irreversible event. The two fields stay in the pure API so a future caller that DOES
// observe contact or a persistent fault directly fails closed with the precise reason.

enum class RecoveryEvidence : std::uint8_t
{
  // The condition was positively established by first-hand evidence.
  kEstablished,
  // The condition was checked and does not hold.
  kNotEstablished,
  // The condition was never established (fail-closed input).
  kUnknown,
};

enum class RecoveryClass : std::uint8_t
{
  kRecoverable,
  kUnsafe,
};

struct RecoveryContext
{
  // Real contact or collision, not exonerated by the planning scene's own checks.
  bool contact_or_collision{false};
  // A hardware/controller fault that persists after a single re-initialisation attempt.
  bool persistent_hardware_fault{false};
  // The arm's stopped state: established when no trajectory was commanded, or a verified stop
  // for this goal generation exists.
  RecoveryEvidence arm_stop{RecoveryEvidence::kUnknown};
  // The held-object state: established when the task's attach/detach receipts and the robot
  // agree on what (if anything) is held.
  RecoveryEvidence held_state{RecoveryEvidence::kUnknown};
  // Whether the survey machinery can re-establish the world state the next attempt needs.
  bool world_reobservable{true};
  // Why the classifier was asked (the refusal or terminal detail), quoted into the receipt.
  std::string cause;
};

struct RecoveryClassification
{
  RecoveryClass klass{RecoveryClass::kUnsafe};
  // Short, receipt-ready reason. For UNSAFE it names the first failing trigger; for
  // RECOVERABLE it states why nothing irreversible happened.
  std::string reason;
};

// Fail-closed: the default context (everything unknown) classifies UNSAFE. Deterministic and
// pure so every rule is unit-testable.
[[nodiscard]] RecoveryClassification classify_recovery(const RecoveryContext & context);

// One receipt line: class, reason and the cause it applies to.
[[nodiscard]] std::string recovery_classification_receipt(
  const RecoveryClassification & classification, const std::string & cause);

[[nodiscard]] const char * to_string(RecoveryClass klass) noexcept;
[[nodiscard]] const char * to_string(RecoveryEvidence evidence) noexcept;

}  // namespace restocker_task_executor
