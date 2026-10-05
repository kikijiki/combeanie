// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/recovery_advice_gate.hpp"

#include <string>
#include <utility>

namespace restocker_task_executor
{

namespace
{

using restocker_reasoner::RecoveryPrimitive;

[[nodiscard]] std::string quoted(RecoveryPrimitive primitive)
{
  return std::string("\"") + restocker_reasoner::recovery_primitive_name(primitive) + "\"";
}

[[nodiscard]] RecoveryAdviceDecision deterministic_decision(
  RecoveryPrimitive deterministic_choice, std::string rejection)
{
  RecoveryAdviceDecision decision;
  decision.primitive = deterministic_choice;
  decision.followed_advice = false;
  decision.changed_the_action = false;
  decision.rejection = std::move(rejection);
  decision.detail = "the deterministic choice " + quoted(deterministic_choice) + " stands: " +
    decision.rejection;
  return decision;
}

}  // namespace

bool RecoveryAdvicePermissions::permits(RecoveryPrimitive primitive) const noexcept
{
  switch (primitive) {
    case RecoveryPrimitive::kRetrySegment: return retry_segment;
    case RecoveryPrimitive::kResumeAtRecoveryState: return resume_at_recovery_state;
    case RecoveryPrimitive::kAbandonTask: return abandon_task;
  }
  return false;
}

std::vector<RecoveryPrimitive> RecoveryAdvicePermissions::permitted() const
{
  std::vector<RecoveryPrimitive> primitives;
  if (retry_segment) {
    primitives.push_back(RecoveryPrimitive::kRetrySegment);
  }
  if (resume_at_recovery_state) {
    primitives.push_back(RecoveryPrimitive::kResumeAtRecoveryState);
  }
  if (abandon_task) {
    primitives.push_back(RecoveryPrimitive::kAbandonTask);
  }
  return primitives;
}

RecoveryAdviceDecision resolve_recovery_advice(
  RecoveryPrimitive deterministic_choice, const RecoveryAdvicePermissions & permitted_now,
  const std::optional<restocker_reasoner::RecoveryAdvice> & advice)
{
  // The deterministic choice is the fallback in every branch below, so it has to be one the
  // permitted set allows. If a caller ever offers one it does not, the disagreement is the
  // caller's bug and the safest reading of it is that nothing may be taken but a refusal to act.
  if (!permitted_now.permits(deterministic_choice)) {
    auto decision = deterministic_decision(
      RecoveryPrimitive::kAbandonTask,
      "the deterministic choice " + quoted(deterministic_choice) +
      " is not permitted at the instant it would be taken");
    return decision;
  }

  if (!advice) {
    return deterministic_decision(
      deterministic_choice, "no recommendation was available at the moment of the decision");
  }
  if (!advice->recommended_primitive) {
    return deterministic_decision(deterministic_choice, "the reasoner recommended nothing");
  }

  const auto recommended = *advice->recommended_primitive;
  // The permitted set the request carried has already been checked by the schema validation. This
  // is the set recomputed now, and it is the one that governs.
  if (!permitted_now.permits(recommended)) {
    return deterministic_decision(
      deterministic_choice,
      "the recommendation " + quoted(recommended) +
      " is not permitted at the instant it would be taken");
  }
  // The second guard, stated as the two things a recommendation may be rather than an ordering over
  // the primitives. Agreeing with the deterministic choice changes nothing, and stopping is never
  // less safe than acting. Anything else, including swapping one kind of motion for another that
  // looks more careful, is the reasoner deciding what the robot does, which it may not do.
  if (recommended != deterministic_choice &&
    recommended != RecoveryPrimitive::kAbandonTask)
  {
    return deterministic_decision(
      deterministic_choice,
      "the recommendation " + quoted(recommended) + " neither agrees with the deterministic "
      "choice " + quoted(deterministic_choice) + " nor declines to act, and advice may only do "
      "one of those");
  }

  RecoveryAdviceDecision decision;
  decision.primitive = recommended;
  decision.followed_advice = true;
  decision.changed_the_action = recommended != deterministic_choice;
  decision.detail = decision.changed_the_action ?
    "the recommendation " + quoted(recommended) + " was taken in place of the deterministic "
    "choice " + quoted(deterministic_choice) :
    "the recommendation " + quoted(recommended) + " agreed with the deterministic choice";
  return decision;
}

}  // namespace restocker_task_executor
