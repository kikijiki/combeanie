// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "restocker_reasoner/recovery_audit_log.hpp"
#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

[[nodiscard]] RecoveryAuditRecord decision_record()
{
  RecoveryAuditRecord record;
  record.event = RecoveryAuditEvent::kDecided;
  record.request_id = "a1b2c3d4";
  record.goal_generation = 7U;
  record.query_json = R"({"request_id":"a1b2c3d4"})";
  record.backend = "http://127.0.0.1:8080/v1/chat/completions";
  record.latency_ms = 1234;
  record.response_verbatim = R"({"status":"ok"})";
  record.permitted_at_decision = {
    RecoveryPrimitive::kResumeAtRecoveryState, RecoveryPrimitive::kAbandonTask};
  record.validated = true;
  record.recommended_primitive = RecoveryPrimitive::kAbandonTask;
  record.failure_class = RecoveryFailureClass::kLinearPathTruncated;
  record.confidence = 0.75;
  record.explanation = "the straight line stopped against a divider";
  record.used = true;
  record.deterministic_primitive = RecoveryPrimitive::kResumeAtRecoveryState;
  record.applied_primitive = RecoveryPrimitive::kAbandonTask;
  record.outcome = "recovery refused; the task was handed to an operator";
  return record;
}

}  // namespace

TEST(RecoveryAuditLog, RecordsTheDecisionAndNotOnlyTheOutcome)
{
  const auto line = render_audit_line(decision_record());
  const auto parsed = parse_strict_json(line, JsonParseLimits{65536U, 16U});
  ASSERT_TRUE(parsed.value) << parsed.error;

  // The record must carry: what was asked, the verbatim answer, validation, whether it was used,
  // the deterministic choice, and the outcome.
  EXPECT_EQ(parsed.value->member("event")->as_string(), "decided");
  EXPECT_EQ(parsed.value->member("request_id")->as_string(), "a1b2c3d4");
  EXPECT_TRUE(parsed.value->member("query")->is_object());
  EXPECT_EQ(parsed.value->member("response_verbatim")->as_string(), R"({"status":"ok"})");
  EXPECT_TRUE(parsed.value->member("validated")->as_bool());
  EXPECT_TRUE(parsed.value->member("used")->as_bool());
  EXPECT_EQ(
    parsed.value->member("deterministic_primitive")->as_string(), "resume_at_recovery_state");
  EXPECT_EQ(parsed.value->member("applied_primitive")->as_string(), "abandon_task");
  EXPECT_EQ(parsed.value->member("permitted_at_decision")->as_array().size(), 2U);
  EXPECT_DOUBLE_EQ(parsed.value->member("latency_ms")->as_number(), 1234.0);
  EXPECT_FALSE(parsed.value->member("outcome")->as_string().empty());
}

TEST(RecoveryAuditLog, RecordsARejectionWithItsReason)
{
  auto record = decision_record();
  record.validated = false;
  record.used = false;
  record.rejection = R"(the response carries the unknown member "joint_torque")";
  record.recommended_primitive.reset();
  record.failure_class.reset();
  record.applied_primitive = RecoveryPrimitive::kResumeAtRecoveryState;

  const auto parsed = parse_strict_json(
    render_audit_line(record), JsonParseLimits{65536U, 16U});
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_FALSE(parsed.value->member("validated")->as_bool());
  EXPECT_NE(parsed.value->member("rejection")->as_string().find("joint_torque"), std::string::npos);
  EXPECT_TRUE(parsed.value->member("recommended_primitive")->is_null());
  EXPECT_TRUE(parsed.value->member("failure_class")->is_null());
}

TEST(RecoveryAuditLog, AModelCannotInjectStructureIntoTheLog)
{
  auto record = decision_record();
  // Exactly what an untrusted producer would send to break out of its own field.
  record.explanation = "\",\"used\":true,\"x\":\"";
  record.response_verbatim = "line one\nline two";

  const auto line = render_audit_line(record);
  EXPECT_EQ(line.find('\n'), std::string::npos) << "one record is one line";
  const auto parsed = parse_strict_json(line, JsonParseLimits{65536U, 16U});
  ASSERT_TRUE(parsed.value) << parsed.error;
  EXPECT_EQ(parsed.value->member("explanation")->as_string(), "\",\"used\":true,\"x\":\"");
}

TEST(RecoveryAuditLog, WritesOneJsonObjectPerLineAndAppends)
{
  const auto path = std::filesystem::temp_directory_path() /
    "restocker_reasoner_audit_test.jsonl";
  std::filesystem::remove(path);

  {
    RecoveryAuditLog log(path.string());
    ASSERT_TRUE(log.open());
    log.append(decision_record());
    auto second = decision_record();
    second.event = RecoveryAuditEvent::kOutcome;
    log.append(second);
    EXPECT_EQ(log.appended(), 2U);
    EXPECT_EQ(log.failed_appends(), 0U);
  }

  std::ifstream stream(path);
  std::string line;
  std::size_t lines = 0U;
  while (std::getline(stream, line)) {
    ++lines;
    EXPECT_TRUE(parse_strict_json(line, JsonParseLimits{65536U, 16U}).value) << line;
  }
  EXPECT_EQ(lines, 2U);
  std::filesystem::remove(path);
}

TEST(RecoveryAuditLog, AnUnwritableLogIsCountedRatherThanThrown)
{
  // The deterministic system must behave identically whether or not the audit file can be
  // written, so a log that cannot open still accepts records and reports the loss.
  RecoveryAuditLog log("/proc/this/path/cannot/exist/audit.jsonl");
  EXPECT_FALSE(log.open());
  EXPECT_TRUE(log.configured());
  log.append(decision_record());
  EXPECT_EQ(log.appended(), 0U);
  EXPECT_EQ(log.failed_appends(), 1U);
}

TEST(RecoveryAuditLog, AnUnconfiguredLogIsNotAnError)
{
  RecoveryAuditLog log("");
  EXPECT_FALSE(log.open());
  EXPECT_FALSE(log.configured());
  log.append(decision_record());
  EXPECT_EQ(log.appended(), 0U);
  // Nothing was lost, because nothing was asked for.
  EXPECT_EQ(log.failed_appends(), 0U);
}

}  // namespace restocker_reasoner
