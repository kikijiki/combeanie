// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <string>

#include "restocker_task_executor/recovery_classification.hpp"
#include "restocker_task_executor/survey_recovery_context.hpp"

namespace
{

using restocker_task_executor::RecoveryClass;
using restocker_task_executor::RecoveryEvidence;
using restocker_task_executor::SurveyFailureEvidence;
using restocker_task_executor::classify_recovery;
using restocker_task_executor::recovery_classification_receipt;
using restocker_task_executor::survey_recovery_context;

SurveyFailureEvidence recoverable_aim_failure()
{
  SurveyFailureEvidence evidence;
  evidence.terminal_result_received = true;
  evidence.motion_definitely_not_started = true;
  evidence.transfer_release_established = true;
  evidence.world_reobservable = true;
  evidence.cause = "SURVEY_SHELF lane_02: planning failed";
  return evidence;
}

TEST(SurveyRecoveryContext, AimFailureWithNotStartedEvidenceIsRecoverable)
{
  const auto context = survey_recovery_context(recoverable_aim_failure());
  EXPECT_EQ(context.arm_stop, RecoveryEvidence::kEstablished);
  EXPECT_EQ(context.held_state, RecoveryEvidence::kEstablished);
  EXPECT_TRUE(context.world_reobservable);
  EXPECT_EQ(classify_recovery(context).klass, RecoveryClass::kRecoverable);
}

TEST(SurveyRecoveryContext, TerminalStopEvidenceIsRecoverable)
{
  auto evidence = recoverable_aim_failure();
  evidence.motion_definitely_not_started = false;
  evidence.execution_reached_terminal_stop = true;
  const auto context = survey_recovery_context(evidence);
  EXPECT_EQ(context.arm_stop, RecoveryEvidence::kEstablished);
  EXPECT_EQ(classify_recovery(context).klass, RecoveryClass::kRecoverable);
}

TEST(SurveyRecoveryContext, MissingTerminalResultLeavesArmUnknownAndUnsafe)
{
  auto evidence = recoverable_aim_failure();
  evidence.terminal_result_received = false;
  evidence.motion_definitely_not_started = false;
  evidence.execution_reached_terminal_stop = false;
  const auto context = survey_recovery_context(evidence);
  EXPECT_EQ(context.arm_stop, RecoveryEvidence::kUnknown);
  EXPECT_EQ(classify_recovery(context).klass, RecoveryClass::kUnsafe);
}

TEST(SurveyRecoveryContext, TerminalResultWithoutEitherFlagRefusesTheStop)
{
  auto evidence = recoverable_aim_failure();
  evidence.motion_definitely_not_started = false;
  evidence.execution_reached_terminal_stop = false;
  const auto classification = classify_recovery(survey_recovery_context(evidence));
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_NE(classification.reason.find("verified stop"), std::string::npos);
}

// Card 086 stage 1a ripple: a MoveIt TIMED_OUT segment completes with neither flag (the adapter
// no longer claims a stop for it), so a survey/lane/viewpoint result built from it leaves the arm
// stop unestablished and the campaign classification UNSAFE.
TEST(SurveyRecoveryContext, ATimedOutExecutionResultDoesNotEstablishTheStop)
{
  auto evidence = recoverable_aim_failure();
  evidence.motion_definitely_not_started = false;  // execution was attempted
  evidence.execution_reached_terminal_stop = false;  // TIMED_OUT proves no stop (stage 1a)
  const auto context = survey_recovery_context(evidence);
  EXPECT_EQ(context.arm_stop, RecoveryEvidence::kNotEstablished);
  EXPECT_EQ(classify_recovery(context).klass, RecoveryClass::kUnsafe);
}

TEST(SurveyRecoveryContext, UnreleasedTransferLeavesHeldStateUnknownAndUnsafe)
{
  auto evidence = recoverable_aim_failure();
  evidence.transfer_release_established = false;
  const auto classification = classify_recovery(survey_recovery_context(evidence));
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_NE(classification.reason.find("held-object"), std::string::npos);
}

TEST(SurveyRecoveryContext, MeasurementOrIdentityFailureIsUnsafeByWorldReobservability)
{
  auto evidence = recoverable_aim_failure();
  evidence.world_reobservable = false;
  const auto classification = classify_recovery(survey_recovery_context(evidence));
  EXPECT_EQ(classification.klass, RecoveryClass::kUnsafe);
  EXPECT_NE(classification.reason.find("re-observable"), std::string::npos);
}

TEST(SurveyRecoveryContext, DefaultEvidenceClassifiesUnsafe)
{
  const SurveyFailureEvidence evidence;
  EXPECT_EQ(
    classify_recovery(survey_recovery_context(evidence)).klass, RecoveryClass::kUnsafe);
}

TEST(SurveyRecoveryContext, ReceiptCarriesClassReasonAndCause)
{
  const auto receipt = recovery_classification_receipt(
    classify_recovery(survey_recovery_context(recoverable_aim_failure())),
    "SURVEY_SHELF lane_02");
  EXPECT_NE(receipt.find("recovery classification: RECOVERABLE"), std::string::npos);
  EXPECT_NE(receipt.find("SURVEY_SHELF lane_02"), std::string::npos);
}

}  // namespace
