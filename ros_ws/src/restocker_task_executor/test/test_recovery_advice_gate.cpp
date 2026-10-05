// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <optional>
#include <string>

#include "restocker_task_executor/recovery_advice_gate.hpp"

namespace restocker_task_executor
{

namespace
{

using restocker_reasoner::RecoveryAdvice;
using restocker_reasoner::RecoveryAdviceStatus;
using restocker_reasoner::RecoveryFailureClass;
using restocker_reasoner::RecoveryPrimitive;

// What the recovery authorisation permits when the driver's motion stop evidence says a bounded
// replan is safe: take it, or refuse to act. Never a retry, which the task machine has already
// spent by the time recovery is reached.
[[nodiscard]] RecoveryAdvicePermissions recovery_authorised()
{
  RecoveryAdvicePermissions permitted;
  permitted.resume_at_recovery_state = true;
  permitted.abandon_task = true;
  return permitted;
}

[[nodiscard]] RecoveryAdvicePermissions recovery_refused()
{
  RecoveryAdvicePermissions permitted;
  permitted.abandon_task = true;
  return permitted;
}

[[nodiscard]] RecoveryAdvice recommending(RecoveryPrimitive primitive)
{
  RecoveryAdvice advice;
  advice.request_id = "a1b2c3d4";
  advice.status = RecoveryAdviceStatus::kOk;
  advice.failure_class = RecoveryFailureClass::kFreeSpacePlanningFailed;
  advice.recommended_primitive = primitive;
  advice.confidence = 0.9;
  return advice;
}

}  // namespace

TEST(RecoveryAdvicePermissions, ReportsExactlyWhatItPermits)
{
  const auto permitted = recovery_authorised();
  EXPECT_FALSE(permitted.permits(RecoveryPrimitive::kRetrySegment));
  EXPECT_TRUE(permitted.permits(RecoveryPrimitive::kResumeAtRecoveryState));
  EXPECT_TRUE(permitted.permits(RecoveryPrimitive::kAbandonTask));
  EXPECT_EQ(permitted.permitted().size(), 2U);
  EXPECT_EQ(RecoveryAdvicePermissions{}.permitted().size(), 0U);
}

// With no reasoner present the driver takes the primitive the deterministic policy picks.
TEST(RecoveryAdviceGate, WithoutAdviceTheDeterministicChoiceStands)
{
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kResumeAtRecoveryState, recovery_authorised(), std::nullopt);
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_FALSE(decision.followed_advice);
  EXPECT_FALSE(decision.changed_the_action);
  EXPECT_FALSE(decision.rejection.empty());
}

TEST(RecoveryAdviceGate, AnEmptyRecommendationIsNotARecommendation)
{
  RecoveryAdvice silent;
  silent.status = RecoveryAdviceStatus::kNoRecommendation;
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kResumeAtRecoveryState, recovery_authorised(), silent);
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_FALSE(decision.followed_advice);
}

TEST(RecoveryAdviceGate, AgreementIsRecordedAsAgreementAndChangesNothing)
{
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kResumeAtRecoveryState, recovery_authorised(),
    recommending(RecoveryPrimitive::kResumeAtRecoveryState));
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_TRUE(decision.followed_advice);
  EXPECT_FALSE(decision.changed_the_action);
  EXPECT_TRUE(decision.rejection.empty());
}

// The only effect a recommendation has here: decline an action the deterministic policy
// was willing to take.
TEST(RecoveryAdviceGate, ARecommendationMayDeclineToAct)
{
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kResumeAtRecoveryState, recovery_authorised(),
    recommending(RecoveryPrimitive::kAbandonTask));
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kAbandonTask);
  EXPECT_TRUE(decision.followed_advice);
  EXPECT_TRUE(decision.changed_the_action);
}

// The safety property, from the other direction: a recommendation can never widen what happens.
TEST(RecoveryAdviceGate, ARecommendationMayNotTalkTheSystemIntoActing)
{
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kAbandonTask, recovery_authorised(),
    recommending(RecoveryPrimitive::kResumeAtRecoveryState));
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kAbandonTask);
  EXPECT_FALSE(decision.followed_advice);
  EXPECT_NE(decision.rejection.find("nor declines to act"), std::string::npos);
}

// A primitive the system implements, recommended where the authorisation does not allow
// it. The recommendation is valid against the schema and still refused.
TEST(RecoveryAdviceGate, ARealPrimitiveThatIsNotPermittedNowIsRefused)
{
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kAbandonTask, recovery_refused(),
    recommending(RecoveryPrimitive::kResumeAtRecoveryState));
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kAbandonTask);
  EXPECT_FALSE(decision.followed_advice);
  EXPECT_NE(decision.rejection.find("not permitted at the instant"), std::string::npos);

  const auto retry = resolve_recovery_advice(
    RecoveryPrimitive::kResumeAtRecoveryState, recovery_authorised(),
    recommending(RecoveryPrimitive::kRetrySegment));
  EXPECT_EQ(retry.primitive, RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_FALSE(retry.followed_advice);
}

// The permitted set that governs is the one recomputed at the decision, not the one the question
// carried. A recommendation generated while a replan was authorised is refused if the
// authorisation has since been withdrawn.
TEST(RecoveryAdviceGate, TheSetThatGovernsIsTheOneRecomputedAtTheDecision)
{
  const auto stale = recommending(RecoveryPrimitive::kResumeAtRecoveryState);
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kAbandonTask, recovery_refused(), stale);
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kAbandonTask);
  EXPECT_FALSE(decision.followed_advice);
}

TEST(RecoveryAdviceGate, ADeterministicChoiceThePermittedSetForbidsFallsClosed)
{
  RecoveryAdvicePermissions nothing;
  nothing.abandon_task = true;
  const auto decision = resolve_recovery_advice(
    RecoveryPrimitive::kRetrySegment, nothing,
    recommending(RecoveryPrimitive::kAbandonTask));
  EXPECT_EQ(decision.primitive, RecoveryPrimitive::kAbandonTask);
  EXPECT_FALSE(decision.followed_advice);
  EXPECT_NE(decision.rejection.find("not permitted at the instant"), std::string::npos);
}

}  // namespace restocker_task_executor
