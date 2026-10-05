// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <string>

#include "restocker_reasoner/recovery_advice.hpp"
#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

constexpr double kFloor = 0.5;

[[nodiscard]] RecoveryQuery question()
{
  RecoveryQuery query;
  query.request_id = "a1b2c3d4";
  query.goal_generation = 7U;
  query.failed_segment = "insert";
  query.motion_outcome = "planning failed";
  query.task_state = "PlanInsert";
  query.observed_detail = "no straight-line plan was found from the current configuration";
  query.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
  query.permitted_primitives = {
    RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
  query.recovery_attempt = 1U;
  query.maximum_recovery_attempts = 2U;
  return query;
}

// A document that passes, so each rejection test can change exactly one thing about it.
[[nodiscard]] std::string valid_document()
{
  return
    R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
    R"("request_id":"a1b2c3d4","status":"ok",)"
    R"("failure_class":"free_space_planning_failed",)"
    R"("recommended_primitive":"resume_at_recovery_state","confidence":0.8,)"
    R"("explanation":"nothing was commanded, so the arm still stands where it was."})";
}

[[nodiscard]] std::string rejection_for(const std::string & document)
{
  const auto parsed = parse_recovery_advice(document, question(), kFloor);
  return parsed.rejection;
}

}  // namespace

TEST(RecoveryAdvice, AcceptsARecommendationThatMeetsTheContract)
{
  const auto parsed = parse_recovery_advice(valid_document(), question(), kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->request_id, "a1b2c3d4");
  EXPECT_EQ(parsed.advice->status, RecoveryAdviceStatus::kOk);
  EXPECT_EQ(parsed.advice->failure_class, RecoveryFailureClass::kFreeSpacePlanningFailed);
  ASSERT_TRUE(parsed.advice->recommended_primitive);
  EXPECT_EQ(*parsed.advice->recommended_primitive, RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_DOUBLE_EQ(parsed.advice->confidence, 0.8);
}

TEST(RecoveryAdvice, AcceptsAnHonestRefusalToRecommend)
{
  const auto parsed = parse_recovery_advice(
    R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
    R"("request_id":"a1b2c3d4","status":"no_recommendation","failure_class":"unknown",)"
    R"("recommended_primitive":null,"confidence":0.1,"explanation":"I do not recognise this."})",
    question(), kFloor);
  ASSERT_TRUE(parsed.advice) << parsed.rejection;
  EXPECT_EQ(parsed.advice->status, RecoveryAdviceStatus::kNoRecommendation);
  EXPECT_FALSE(parsed.advice->recommended_primitive);
  // A refusal is not held to the confidence floor: it recommends nothing.
  EXPECT_DOUBLE_EQ(parsed.advice->confidence, 0.1);
}

TEST(RecoveryAdvice, RejectsAnythingThatIsNotStrictJson)
{
  EXPECT_FALSE(rejection_for("").empty());
  EXPECT_FALSE(rejection_for("not json at all").empty());
  EXPECT_FALSE(rejection_for("[" + valid_document() + "]").empty());
  // The shape a chat model reaches for when it explains itself first.
  EXPECT_FALSE(
    rejection_for("Here is my answer:\n```json\n" + valid_document() + "\n```").empty());
}

TEST(RecoveryAdvice, RejectsUnknownAndMissingMembers)
{
  auto with_extra = valid_document();
  with_extra.insert(with_extra.size() - 1U, R"(,"joint_torque":[1,2,3])");
  EXPECT_NE(rejection_for(with_extra).find("unknown member"), std::string::npos);

  const std::string missing =
    R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
    R"("request_id":"a1b2c3d4","status":"ok",)"
    R"("recommended_primitive":"resume_at_recovery_state","confidence":0.8,)"
    R"("explanation":"x"})";
  EXPECT_NE(rejection_for(missing).find("omits the required member"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAPrimitiveOutsideTheClosedSet)
{
  auto invented = valid_document();
  const auto position = invented.find("resume_at_recovery_state");
  ASSERT_NE(position, std::string::npos);
  invented.replace(position, std::string("resume_at_recovery_state").size(), "push_to_clear");
  EXPECT_NE(
    rejection_for(invented).find("\"recommended_primitive\" is not one of"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAPrimitiveThatWasNotOffered)
{
  auto unoffered = valid_document();
  const auto position = unoffered.find("resume_at_recovery_state");
  ASSERT_NE(position, std::string::npos);
  unoffered.replace(position, std::string("resume_at_recovery_state").size(), "retry_segment");
  // retry_segment is a real primitive, and it is not in this question's permitted set.
  EXPECT_NE(
    rejection_for(unoffered).find("was not in the permitted set"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAnUnknownFailureClass)
{
  auto invented = valid_document();
  const auto position = invented.find("free_space_planning_failed");
  ASSERT_NE(position, std::string::npos);
  invented.replace(position, std::string("free_space_planning_failed").size(), "gremlins");
  EXPECT_NE(rejection_for(invented).find("\"failure_class\" is not one of"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsWrongTypes)
{
  EXPECT_FALSE(
    rejection_for(
      R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
      R"("request_id":"a1b2c3d4","status":"ok",)"
      R"("failure_class":"free_space_planning_failed",)"
      R"("recommended_primitive":"resume_at_recovery_state","confidence":"0.8",)"
      R"("explanation":"x"})").empty()) << "confidence as a string";
  EXPECT_FALSE(
    rejection_for(
      R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
      R"("request_id":"a1b2c3d4","status":"ok",)"
      R"("failure_class":"free_space_planning_failed",)"
      R"("recommended_primitive":["resume_at_recovery_state"],"confidence":0.8,)"
      R"("explanation":"x"})").empty()) << "a primitive as an array";
  EXPECT_FALSE(
    rejection_for(
      R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
      R"("request_id":"a1b2c3d4","status":"ok",)"
      R"("failure_class":"free_space_planning_failed",)"
      R"("recommended_primitive":"resume_at_recovery_state","confidence":0.8,)"
      R"("explanation":42})").empty()) << "an explanation as a number";
}

TEST(RecoveryAdvice, RejectsTheWrongSchemaVersion)
{
  auto wrong = valid_document();
  const auto position = wrong.find("restocker.reasoner.recovery.response.v1");
  ASSERT_NE(position, std::string::npos);
  wrong.replace(
    position, std::string("restocker.reasoner.recovery.response.v1").size(),
    "restocker.reasoner.recovery.request.v1");
  EXPECT_NE(rejection_for(wrong).find("\"schema_version\""), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAnAnswerToADifferentQuestion)
{
  auto other = valid_document();
  const auto position = other.find("a1b2c3d4");
  ASSERT_NE(position, std::string::npos);
  other.replace(position, std::string("a1b2c3d4").size(), "deadbeef");
  EXPECT_NE(rejection_for(other).find("does not echo"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAStatusThatDisagreesWithTheRecommendation)
{
  EXPECT_NE(
    rejection_for(
      R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
      R"("request_id":"a1b2c3d4","status":"ok",)"
      R"("failure_class":"free_space_planning_failed",)"
      R"("recommended_primitive":null,"confidence":0.8,"explanation":"x"})")
    .find("no primitive was recommended"), std::string::npos);
  EXPECT_NE(
    rejection_for(
      R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
      R"("request_id":"a1b2c3d4","status":"no_recommendation",)"
      R"("failure_class":"free_space_planning_failed",)"
      R"("recommended_primitive":"abandon_task","confidence":0.8,"explanation":"x"})")
    .find("a primitive was recommended"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsConfidenceOutsideItsRangeOrBelowTheFloor)
{
  auto too_high = valid_document();
  auto position = too_high.find("0.8");
  ASSERT_NE(position, std::string::npos);
  too_high.replace(position, 3U, "1.5");
  EXPECT_NE(rejection_for(too_high).find("outside [0, 1]"), std::string::npos);

  auto too_low = valid_document();
  position = too_low.find("0.8");
  ASSERT_NE(position, std::string::npos);
  too_low.replace(position, 3U, "0.2");
  EXPECT_NE(rejection_for(too_low).find("below the configured floor"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAnExplanationPastItsBound)
{
  const std::string long_explanation(kRecoveryExplanationMaximumBytes + 1U, 'x');
  const std::string document =
    R"({"schema_version":"restocker.reasoner.recovery.response.v1",)"
    R"("request_id":"a1b2c3d4","status":"ok",)"
    R"("failure_class":"free_space_planning_failed",)"
    R"("recommended_primitive":"resume_at_recovery_state","confidence":0.8,)"
    R"("explanation":")" + long_explanation + R"("})";
  EXPECT_NE(rejection_for(document).find("byte bound"), std::string::npos);
}

TEST(RecoveryAdvice, RejectsAMisconfiguredConfidenceFloor)
{
  EXPECT_FALSE(parse_recovery_advice(valid_document(), question(), -0.1).advice);
  EXPECT_FALSE(parse_recovery_advice(valid_document(), question(), 1.1).advice);
}

TEST(RecoveryAdvice, PrimitiveNamesRoundTripAndUnknownNamesDoNot)
{
  for (const auto primitive : {
      RecoveryPrimitive::kRetrySegment, RecoveryPrimitive::kResumeAtRecoveryState,
      RecoveryPrimitive::kAbandonTask})
  {
    const auto name = recovery_primitive_name(primitive);
    const auto parsed = recovery_primitive_from_name(name);
    ASSERT_TRUE(parsed);
    EXPECT_EQ(*parsed, primitive);
  }
  EXPECT_FALSE(recovery_primitive_from_name(""));
  EXPECT_FALSE(recovery_primitive_from_name("RETRY_SEGMENT"));
  EXPECT_FALSE(recovery_primitive_from_name("retry_segment "));
  EXPECT_FALSE(recovery_primitive_from_name("rm -rf /"));
}

TEST(RecoveryAdvice, ThePromptOffersOnlyThePermittedPrimitives)
{
  const auto query = question();
  const auto prompt = render_recovery_system_prompt(query);
  EXPECT_NE(prompt.find("resume_at_recovery_state"), std::string::npos);
  EXPECT_NE(prompt.find("abandon_task"), std::string::npos);
  EXPECT_EQ(prompt.find("retry_segment"), std::string::npos);
  EXPECT_NE(
    render_recovery_user_prompt(query).find("no straight-line plan was found"),
    std::string::npos);
}

TEST(RecoveryAdvice, TheCanonicalQueryRenderingIsValidJson)
{
  auto query = question();
  query.observed_detail = "a \"quoted\" detail\nwith a newline";
  const auto rendered = render_recovery_query_json(query);
  const auto parsed = parse_strict_json(rendered);
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_EQ(parsed.value->member("request_id")->as_string(), "a1b2c3d4");
  ASSERT_NE(parsed.value->member("permitted_primitives"), nullptr);
  EXPECT_EQ(parsed.value->member("permitted_primitives")->as_array().size(), 2U);
}

TEST(RecoveryAdvice, TheConstrainedDecodingSchemaIsValidJson)
{
  const auto parsed = parse_strict_json(recovery_advice_response_schema());
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_TRUE(parsed.value->is_object());
}

}  // namespace restocker_reasoner
