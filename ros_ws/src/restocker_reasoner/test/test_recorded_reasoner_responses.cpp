// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <string_view>

#include "restocker_reasoner/recovery_advice.hpp"

#include "recorded_failure_cases.hpp"

namespace restocker_reasoner
{

namespace
{

// Answers a local llama.cpp backend actually gave, replayed against the same validator the live
// client uses.
//
// They are recorded so the suite needs no model service. The capture procedure and the exact
// questions are in `test/recorded_responses/README.md`; the failures are ones the executor really
// produces, in the coordinator's own words.
[[nodiscard]] std::string recorded(const std::string & name)
{
  const auto path = std::filesystem::path(RESTOCKER_RECORDED_RESPONSES) / name;
  std::ifstream stream(path);
  EXPECT_TRUE(stream.is_open()) << "missing recorded response " << path.string();
  std::ostringstream text;
  text << stream.rdbuf();
  auto document = text.str();
  // Files end with a newline; the contract's own text does not.
  while (!document.empty() && (document.back() == '\n' || document.back() == '\r')) {
    document.pop_back();
  }
  return document;
}

constexpr double kFloor = 0.5;

[[nodiscard]] RecoveryQuery question(std::string_view name)
{
  const auto found = recorded_failure_case(name);
  EXPECT_TRUE(found.has_value()) << "unknown recorded case " << name;
  return found.value_or(RecoveryQuery{});
}

}  // namespace

// The failure the deterministic policy already knows the answer for. Checks both the
// classification and that the recommendation is the primitive the policy had already chosen.
TEST(RecordedReasonerResponses, ClassifiesATruncatedCartesianApproach)
{
  const auto parsed = parse_recovery_advice(
    recorded("linear_path_truncated.json"), question("linear_path_truncated"), kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->failure_class, RecoveryFailureClass::kLinearPathTruncated);
  ASSERT_TRUE(parsed.advice->recommended_primitive);
  EXPECT_EQ(
    *parsed.advice->recommended_primitive, RecoveryPrimitive::kResumeAtRecoveryState);
}

TEST(RecordedReasonerResponses, ClassifiesAControllerAbortOnAFreeSpaceTraverse)
{
  const auto parsed = parse_recovery_advice(
    recorded("trajectory_execution_aborted.json"), question("trajectory_execution_aborted"),
    kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->failure_class, RecoveryFailureClass::kTrajectoryExecutionAborted);
}

TEST(RecordedReasonerResponses, ClassifiesAFreeSpacePlanThatFoundNothing)
{
  const auto parsed = parse_recovery_advice(
    recorded("free_space_planning_failed.json"), question("free_space_planning_failed"), kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->failure_class, RecoveryFailureClass::kFreeSpacePlanningFailed);
}

// The same question with constrained decoding switched off. The backend answered with members in
// its own order and the validator accepted it. Kept because it shows constrained decoding only
// makes an answer parseable more often; it is not a trust boundary. Validation runs identically
// either way.
TEST(RecordedReasonerResponses, AcceptsAValidAnswerGivenWithoutConstrainedDecoding)
{
  const auto parsed = parse_recovery_advice(
    recorded("unconstrained_free_form.json"), question("free_space_planning_failed"), kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->failure_class, RecoveryFailureClass::kFreeSpacePlanningFailed);
}

// The likeliest real malformed answer: the backend ran out of output budget in the middle of a
// string. Constrained decoding did not prevent it, and the validator refuses it.
TEST(RecordedReasonerResponses, RejectsAnAnswerTheBackendRanOutOfBudgetFor)
{
  const auto parsed = parse_recovery_advice(
    recorded("truncated_output.txt"), question("free_space_planning_failed"), kFloor);
  EXPECT_FALSE(parsed.advice);
  EXPECT_NE(parsed.rejection.find("not strict JSON"), std::string::npos) << parsed.rejection;
}

// A recorded answer belongs to its own question: replaying it against a different failure must
// fail.
TEST(RecordedReasonerResponses, ARecordedAnswerDoesNotAnswerADifferentQuestion)
{
  const auto parsed = parse_recovery_advice(
    recorded("linear_path_truncated.json"), question("free_space_planning_failed"), kFloor);
  EXPECT_FALSE(parsed.advice);
  EXPECT_NE(parsed.rejection.find("does not echo"), std::string::npos);
}

}  // namespace restocker_reasoner
