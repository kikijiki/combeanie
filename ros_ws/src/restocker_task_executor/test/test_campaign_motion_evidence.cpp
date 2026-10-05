// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Card 086 stage 1 (CMB-SPEC-13): what each endpoint's delivered terminal proves, against the real
// action messages. Fail-closed rows are asserted as hard as the settling ones.

#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "restocker_task_executor/campaign_motion_evidence.hpp"

namespace restocker_task_executor
{
namespace
{

using rclcpp_action::ResultCode;
using restocker_interfaces::action::RestockProduct;
using restocker_interfaces::action::SurveyLane;
using restocker_interfaces::action::SurveyTray;
using restocker_interfaces::action::SurveyViewpoint;

// -- tray: review B1 --------------------------------------------------------------------------

SurveyTray::Result tray(std::uint8_t outcome, bool not_started, bool stop)
{
  SurveyTray::Result result;
  result.outcome = outcome;
  result.motion_definitely_not_started = not_started;
  result.execution_reached_terminal_stop = stop;
  return result;
}

TEST(TrayEvidence, ACanceledTerminalOverTheDefaultNonStartDoesNotSettle)
{
  // tray_survey_node initialises motion_definitely_not_started=true and a leg that never ran
  // leaves it, while the viewpoint child may still be admitted (stop request, shutdown return).
  const auto result = tray(SurveyTray::Result::OUTCOME_CANCELED, true, false);
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::CANCELED, result)));
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
}

TEST(TrayEvidence, AnUnavailableTerminalDeliveredWithCodeSucceededDoesNotSettle)
{
  // The 5 s send-grace expiry returns UNAVAILABLE through succeed(): a code-based rule alone
  // would miss it.
  const auto result = tray(SurveyTray::Result::OUTCOME_UNAVAILABLE, true, false);
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::ABORTED, result)));
}

TEST(TrayEvidence, StopEvidenceFromAnEarlierLegDoesNotSettleAParentThatLeftAChild)
{
  for (const auto outcome :
    {SurveyTray::Result::OUTCOME_CANCELED, SurveyTray::Result::OUTCOME_UNAVAILABLE})
  {
    EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, tray(outcome, false, true))));
  }
}

TEST(TrayEvidence, CompletedSurveysArriveWhateverTheirFlagsSay)
{
  for (const auto outcome :
    {SurveyTray::Result::OUTCOME_CONFIRMED, SurveyTray::Result::OUTCOME_NO_CANDIDATE,
      SurveyTray::Result::OUTCOME_REFUTED, SurveyTray::Result::OUTCOME_OVERVIEW_ONLY})
  {
    EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, tray(outcome, false, false))));
  }
}

TEST(TrayEvidence, LegsThatRanAreReadByTheirOwnFlags)
{
  for (const auto outcome :
    {SurveyTray::Result::OUTCOME_VIEWPOINT_UNREACHED,
      SurveyTray::Result::OUTCOME_ACQUISITION_FAILED})
  {
    EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, tray(outcome, true, false))));
    EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, tray(outcome, false, true))));
    EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, tray(outcome, false, false))));
  }
}

// -- lane / viewpoint ---------------------------------------------------------------------------

TEST(LaneEvidence, ReadsArrivalNonStartAndStopAndRefusesNeither)
{
  SurveyLane::Result result;
  result.outcome = SurveyLane::Result::OUTCOME_OBSERVED;
  EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  result.outcome = SurveyLane::Result::OUTCOME_VIEWPOINT_UNREACHED;
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  result.motion_definitely_not_started = true;
  EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  result.motion_definitely_not_started = false;
  result.execution_reached_terminal_stop = true;
  EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::CANCELED, result)));
}

TEST(ViewpointEvidence, ARetreatSettlesOnArrivalOrAStopOnly)
{
  SurveyViewpoint::Result result;
  result.outcome = SurveyViewpoint::Result::OUTCOME_ARRIVED;
  EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  result.outcome = SurveyViewpoint::Result::OUTCOME_EXECUTION_FAILED;
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  result.execution_reached_terminal_stop = true;
  EXPECT_TRUE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
  // Stage 1a: a MoveIt TIMED_OUT reaches here as an execution failure WITHOUT the stop flag.
  result.outcome = SurveyViewpoint::Result::OUTCOME_TIMED_OUT;
  result.execution_reached_terminal_stop = false;
  EXPECT_FALSE(settles_attempt(evidence_of(ResultCode::SUCCEEDED, result)));
}

// -- restock: every status the action defines ---------------------------------------------------

bool restock_settles(std::uint16_t status, ResultCode code = ResultCode::ABORTED)
{
  RestockProduct::Result result;
  result.status = status;
  return settles_attempt(evidence_of(code, result));
}

TEST(RestockEvidence, OnlyProvenStatusesSettle)
{
  using R = RestockProduct::Result;
  EXPECT_TRUE(restock_settles(R::STATUS_SUCCEEDED, ResultCode::SUCCEEDED));
  EXPECT_TRUE(restock_settles(R::STATUS_SKIPPED_RECOVERABLE));
  // Selection-phase terminals end before any motion (driver: "selection ended safely").
  EXPECT_TRUE(restock_settles(R::STATUS_NO_COMPATIBLE_PAIR));
  EXPECT_TRUE(restock_settles(R::STATUS_OBSERVATION_EVIDENCE_STALE));
}

TEST(RestockEvidence, EveryOtherStatusStaysUnresolved)
{
  using R = RestockProduct::Result;
  for (const std::uint16_t status :
    {R::STATUS_UNSET, R::STATUS_CANCELED, R::STATUS_VALIDATION_FAILED,
      R::STATUS_PLANNING_FAILED, R::STATUS_EXECUTION_FAILED, R::STATUS_VERIFICATION_FAILED,
      R::STATUS_RECOVERY_EXHAUSTED, R::STATUS_EXTERNAL_INCONSISTENCY,
      R::STATUS_OPERATOR_REQUIRED, R::STATUS_SHUTDOWN, R::STATUS_INTERNAL_ERROR})
  {
    EXPECT_FALSE(restock_settles(status)) << "status " << status;
    EXPECT_FALSE(restock_settles(status, ResultCode::CANCELED)) << "status " << status;
  }
}

// Card 086 stage 1b: the coordinator's own proof fields settle the statuses that carry them.
RestockProduct::Result restock_with(
  std::uint16_t status, bool not_started, bool stop)
{
  RestockProduct::Result result;
  result.status = status;
  result.motion_definitely_not_started = not_started;
  result.execution_reached_terminal_stop = stop;
  return result;
}

TEST(RestockEvidence, ProvenPreMotionFailuresSettleExactlyLikeTheOtherEndpoints)
{
  using R = RestockProduct::Result;
  for (const std::uint16_t status :
    {R::STATUS_CANCELED, R::STATUS_VALIDATION_FAILED, R::STATUS_PLANNING_FAILED,
      R::STATUS_EXECUTION_FAILED, R::STATUS_VERIFICATION_FAILED, R::STATUS_RECOVERY_EXHAUSTED})
  {
    EXPECT_TRUE(
      settles_attempt(evidence_of(ResultCode::ABORTED, restock_with(status, true, false))))
      << "non-start, status " << status;
    EXPECT_TRUE(
      settles_attempt(evidence_of(ResultCode::ABORTED, restock_with(status, false, true))))
      << "stop, status " << status;
    EXPECT_FALSE(
      settles_attempt(evidence_of(ResultCode::ABORTED, restock_with(status, false, false))))
      << "neither flag, status " << status;
  }
}

TEST(RestockEvidence, DoubtfulStatusesNeverSettleWhateverTheFieldsSay)
{
  using R = RestockProduct::Result;
  for (const std::uint16_t status :
    {R::STATUS_UNSET, R::STATUS_EXTERNAL_INCONSISTENCY, R::STATUS_OPERATOR_REQUIRED,
      R::STATUS_SHUTDOWN, R::STATUS_INTERNAL_ERROR})
  {
    EXPECT_FALSE(
      settles_attempt(evidence_of(ResultCode::ABORTED, restock_with(status, true, true))))
      << "status " << status;
  }
}

TEST(RestockEvidence, ASucceededStatusUnderAnAbortedCodeStillRequiresTheDeliveredCode)
{
  EXPECT_FALSE(restock_settles(RestockProduct::Result::STATUS_SUCCEEDED, ResultCode::ABORTED));
}

}  // namespace
}  // namespace restocker_task_executor
