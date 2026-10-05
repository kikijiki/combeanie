// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/recovery_classification.hpp"

namespace restocker_task_executor
{

namespace
{

[[nodiscard]] bool established(RecoveryEvidence evidence)
{
  return evidence == RecoveryEvidence::kEstablished;
}

}  // namespace

RecoveryClassification classify_recovery(const RecoveryContext & context)
{
  // The order is the spec's: the first trigger that fails decides, so one receipt names one
  // reason. Anything not established below is unsafe — the fail-closed default.
  if (context.contact_or_collision) {
    return {RecoveryClass::kUnsafe, "real contact or collision occurred"};
  }
  if (context.persistent_hardware_fault) {
    return {
      RecoveryClass::kUnsafe,
      "a hardware/controller fault persists after re-initialisation"};
  }
  if (context.arm_stop == RecoveryEvidence::kUnknown) {
    return {RecoveryClass::kUnsafe, "the arm's stopped state is not established"};
  }
  if (!established(context.arm_stop)) {
    return {RecoveryClass::kUnsafe, "the arm has not reached a verified stop"};
  }
  if (context.held_state == RecoveryEvidence::kUnknown) {
    return {RecoveryClass::kUnsafe, "the held-object state is not established"};
  }
  if (!established(context.held_state)) {
    return {RecoveryClass::kUnsafe, "the held-object state is inconsistent with the task"};
  }
  if (!context.world_reobservable) {
    return {RecoveryClass::kUnsafe, "the world state is not re-observable"};
  }
  return {RecoveryClass::kRecoverable, "nothing irreversible happened"};
}

std::string recovery_classification_receipt(
  const RecoveryClassification & classification, const std::string & cause)
{
  std::string receipt = "recovery classification: ";
  receipt += to_string(classification.klass);
  receipt += " (";
  receipt += classification.reason;
  receipt += ")";
  if (!cause.empty()) {
    receipt += ": ";
    receipt += cause;
  }
  return receipt;
}

const char * to_string(RecoveryClass klass) noexcept
{
  switch (klass) {
    case RecoveryClass::kRecoverable: return "RECOVERABLE";
    case RecoveryClass::kUnsafe: return "UNSAFE";
  }
  return "UNSAFE";
}

const char * to_string(RecoveryEvidence evidence) noexcept
{
  switch (evidence) {
    case RecoveryEvidence::kEstablished: return "established";
    case RecoveryEvidence::kNotEstablished: return "not_established";
    case RecoveryEvidence::kUnknown: return "unknown";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
