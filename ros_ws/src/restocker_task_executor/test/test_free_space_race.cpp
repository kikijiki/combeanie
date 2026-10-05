// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <random>
#include <string>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

#include "restocker_task_executor/free_space_race.hpp"
#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{
namespace
{
using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;

RacePipelineOutcome outcome(
  const std::string & name, int code, double seconds, bool has_trajectory)
{
  RacePipelineOutcome result;
  result.name = name;
  result.error_code = code;
  result.seconds = seconds;
  result.has_trajectory = has_trajectory;
  return result;
}

// Card 053: a race where the primary fails and the fallback wins is a success, and the
// receipt still names both pipelines' codes and times.
TEST(FreeSpaceRace, PrimaryFailsAndFallbackWinsIsASuccessWithBothPipelinesReceipted)
{
  const std::vector<RacePipelineOutcome> outcomes = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 14.99, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::SUCCESS, 0.84, true)};
  const auto decision = decide_race(outcomes, 0.84);
  EXPECT_TRUE(decision.success);
  EXPECT_EQ(decision.classification_code, ErrorCodes::SUCCESS);
  EXPECT_NE(decision.receipt.find("ompl=FAILURE (code 99999) in 14.99 s"), std::string::npos)
    << decision.receipt;
  EXPECT_NE(
    decision.receipt.find("ompl_fallback=SUCCESS (code 1) in 0.84 s"), std::string::npos)
    << decision.receipt;
}

// Both pipelines lose instantly: the pose stays what refused — Card 043's verdict class.
TEST(FreeSpaceRace, BothFailInstantlyKeepsTheCard043PoseVerdict)
{
  const std::vector<RacePipelineOutcome> outcomes = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 0.004, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::FAILURE, 0.006, false)};
  const auto decision = decide_race(outcomes, 0.007);
  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.classification_code, ErrorCodes::FAILURE);
  const StartStateCondition condition;
  const auto classified = classify_plan_failure(
    decision.classification_code, condition,
    PlanningSlice{decision.wall_seconds, 15.0}, "pre-insert");
  EXPECT_EQ(classified.status, PlanningStatus::kPlanningRejected) << classified.detail;
}

// Both pipelines burn the slice: the wall evidence keeps Card 043's kPlanningTimedOut
// non-verdict, whatever the generic code says.
TEST(FreeSpaceRace, BothFailAfterBurningTheSliceClassifiesAsPlanningTimedOut)
{
  const std::vector<RacePipelineOutcome> outcomes = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 14.98, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::TIMED_OUT, 15.0, false)};
  const auto decision = decide_race(outcomes, 15.01);
  EXPECT_FALSE(decision.success);
  EXPECT_EQ(decision.classification_code, ErrorCodes::FAILURE);
  const StartStateCondition condition;
  const auto classified = classify_plan_failure(
    decision.classification_code, condition,
    PlanningSlice{decision.wall_seconds, 15.0}, "pre-insert");
  EXPECT_EQ(classified.status, PlanningStatus::kPlanningTimedOut) << classified.detail;
}

// A start-state code from either pipeline is a property of the shared start state: the
// classification follows it, never the pose (Card 043 precedence over the race).
TEST(FreeSpaceRace, StartStateCodeFromEitherPipelineWinsTheClassification)
{
  const std::vector<RacePipelineOutcome> primary_reports = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 0.01, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::INVALID_ROBOT_STATE, 0.02, false)};
  EXPECT_EQ(
    decide_race(primary_reports, 0.03).classification_code,
    ErrorCodes::INVALID_ROBOT_STATE);
  const std::vector<RacePipelineOutcome> fallback_reports = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 14.9, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::START_STATE_IN_COLLISION, 0.01, false)};
  EXPECT_EQ(
    decide_race(fallback_reports, 14.91).classification_code,
    ErrorCodes::START_STATE_IN_COLLISION);
}

// Deterministic seed: the same seeded outcome sequence must produce byte-identical
// verdicts and receipts on every run (the race's decision layer has no randomness of its
// own; this pins that it never grows any).
TEST(FreeSpaceRace, DeterministicSeedOutcomesProduceIdenticalVerdictsAndReceipts)
{
  const auto generate = [] {
    std::mt19937 rng(42U);
    std::vector<std::vector<RacePipelineOutcome>> population;
    for (int sample = 0; sample < 64; ++sample) {
      const int primary = (rng() % 4 == 0) ? ErrorCodes::FAILURE :
        ((rng() % 3 == 0) ? ErrorCodes::INVALID_ROBOT_STATE : ErrorCodes::TIMED_OUT);
      const int fallback = (rng() % 2 == 0) ? ErrorCodes::FAILURE : ErrorCodes::TIMED_OUT;
      const bool fallback_wins = (rng() % 5 == 0);
      population.push_back(
          {
            outcome(
              kRacePrimaryPipeline, primary, 0.5 + static_cast<double>(rng() % 100) / 10.0,
              false),
            outcome(
              kRaceFallbackPipeline, fallback_wins ? ErrorCodes::SUCCESS : fallback,
              0.4 + static_cast<double>(rng() % 100) / 10.0, fallback_wins)});
    }
    return population;
  };
  const auto first = generate();
  const auto second = generate();
  ASSERT_EQ(first.size(), second.size());
  for (std::size_t index = 0U; index < first.size(); ++index) {
    const double wall = 1.0 + static_cast<double>(index);
    const auto left = decide_race(first[index], wall);
    const auto right = decide_race(second[index], wall);
    EXPECT_EQ(left.success, right.success) << "sample " << index;
    EXPECT_EQ(left.classification_code, right.classification_code) << "sample " << index;
    EXPECT_EQ(left.receipt, right.receipt) << "sample " << index;
  }
}

// The receipt names every pipeline that ran with its own code and time (spec §6).
TEST(FreeSpaceRace, ReceiptNamesEveryPipelineWithItsOwnCodeAndTime)
{
  const std::vector<RacePipelineOutcome> outcomes = {
    outcome(kRacePrimaryPipeline, ErrorCodes::FAILURE, 14.99, false),
    outcome(kRaceFallbackPipeline, ErrorCodes::TIMED_OUT, 15.0, false)};
  const auto decision = decide_race(outcomes, 15.0);
  EXPECT_NE(decision.receipt.find("ompl="), std::string::npos) << decision.receipt;
  EXPECT_NE(decision.receipt.find("ompl_fallback="), std::string::npos) << decision.receipt;
  EXPECT_NE(decision.receipt.find("in 14.99 s"), std::string::npos) << decision.receipt;
  EXPECT_NE(decision.receipt.find("in 15"), std::string::npos) << decision.receipt;
  EXPECT_NE(decision.receipt.find("; "), std::string::npos) << decision.receipt;
}

}  // namespace
}  // namespace restocker_task_executor
