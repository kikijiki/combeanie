// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_reasoner/recovery_advice.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <utility>

#include "restocker_reasoner/strict_json.hpp"

namespace restocker_reasoner
{

namespace
{

constexpr std::array<RecoveryPrimitive, 3U> kAllPrimitives{
  RecoveryPrimitive::kRetrySegment,
  RecoveryPrimitive::kResumeAtRecoveryState,
  RecoveryPrimitive::kAbandonTask};

constexpr std::array<RecoveryFailureClass, 4U> kAllFailureClasses{
  RecoveryFailureClass::kLinearPathTruncated,
  RecoveryFailureClass::kTrajectoryExecutionAborted,
  RecoveryFailureClass::kFreeSpacePlanningFailed,
  RecoveryFailureClass::kUnknown};

// Exactly the member names the contract defines. Any other name is rejected rather than ignored,
// since an ignored member lets a backend and a reader disagree about what was said.
constexpr std::array<std::string_view, 7U> kRequiredMembers{
  "schema_version", "request_id", "status", "failure_class", "recommended_primitive",
  "confidence", "explanation"};

[[nodiscard]] std::string quoted_enum_list(const std::vector<std::string> & names)
{
  std::string out;
  for (const auto & name : names) {
    if (!out.empty()) {
      out += ", ";
    }
    out += '"';
    out += name;
    out += '"';
  }
  return out;
}

[[nodiscard]] std::string bounded(std::string_view text, std::size_t limit)
{
  if (text.size() <= limit) {
    return std::string(text);
  }
  return std::string(text.substr(0U, limit)) + "...[truncated]";
}

}  // namespace

const char * recovery_primitive_name(RecoveryPrimitive primitive) noexcept
{
  switch (primitive) {
    case RecoveryPrimitive::kRetrySegment: return "retry_segment";
    case RecoveryPrimitive::kResumeAtRecoveryState: return "resume_at_recovery_state";
    case RecoveryPrimitive::kAbandonTask: return "abandon_task";
  }
  return "abandon_task";
}

std::optional<RecoveryPrimitive> recovery_primitive_from_name(std::string_view name) noexcept
{
  for (const auto primitive : kAllPrimitives) {
    if (name == recovery_primitive_name(primitive)) {
      return primitive;
    }
  }
  return std::nullopt;
}

const char * recovery_failure_class_name(RecoveryFailureClass failure) noexcept
{
  switch (failure) {
    case RecoveryFailureClass::kLinearPathTruncated: return "linear_path_truncated";
    case RecoveryFailureClass::kTrajectoryExecutionAborted: return "trajectory_execution_aborted";
    case RecoveryFailureClass::kFreeSpacePlanningFailed: return "free_space_planning_failed";
    case RecoveryFailureClass::kUnknown: return "unknown";
  }
  return "unknown";
}

std::optional<RecoveryFailureClass> recovery_failure_class_from_name(
  std::string_view name) noexcept
{
  for (const auto failure : kAllFailureClasses) {
    if (name == recovery_failure_class_name(failure)) {
      return failure;
    }
  }
  return std::nullopt;
}

const char * recovery_advice_status_name(RecoveryAdviceStatus status) noexcept
{
  return status == RecoveryAdviceStatus::kOk ? "ok" : "no_recommendation";
}

std::optional<RecoveryAdviceStatus> recovery_advice_status_from_name(
  std::string_view name) noexcept
{
  if (name == "ok") {
    return RecoveryAdviceStatus::kOk;
  }
  if (name == "no_recommendation") {
    return RecoveryAdviceStatus::kNoRecommendation;
  }
  return std::nullopt;
}

RecoveryAdviceParse parse_recovery_advice(
  std::string_view document, const RecoveryQuery & query, double minimum_confidence)
{
  const auto reject = [](std::string reason) {
    return RecoveryAdviceParse{std::nullopt, std::move(reason)};
  };

  if (!std::isfinite(minimum_confidence) || minimum_confidence < 0.0 ||
    minimum_confidence > 1.0)
  {
    return reject("the configured minimum confidence is not a probability");
  }

  const auto parsed = parse_strict_json(document);
  if (!parsed.value) {
    return reject("the response is not strict JSON: " + parsed.error);
  }
  if (!parsed.value->is_object()) {
    return reject("the response is not a JSON object");
  }
  const auto & members = parsed.value->as_object();

  // Reject-by-default over the member set, in both directions: nothing missing and nothing extra.
  for (const auto & required : kRequiredMembers) {
    if (members.find(std::string(required)) == members.end()) {
      return reject("the response omits the required member \"" + std::string(required) + "\"");
    }
  }
  for (const auto & [name, value] : members) {
    (void)value;
    if (std::ranges::find(kRequiredMembers, name) == kRequiredMembers.end()) {
      return reject("the response carries the unknown member \"" + name + "\"");
    }
  }

  const auto * schema_version = parsed.value->member("schema_version");
  if (!schema_version->is_string()) {
    return reject("\"schema_version\" is not a string");
  }
  if (schema_version->as_string() != kRecoveryAdviceSchemaVersion) {
    return reject(
      "\"schema_version\" is not " + std::string(kRecoveryAdviceSchemaVersion));
  }

  const auto * request_id = parsed.value->member("request_id");
  if (!request_id->is_string()) {
    return reject("\"request_id\" is not a string");
  }
  // A response that does not echo the request may answer a different failure (a stale advisor).
  if (request_id->as_string() != query.request_id) {
    return reject("\"request_id\" does not echo the request it answers");
  }

  const auto * status_value = parsed.value->member("status");
  if (!status_value->is_string()) {
    return reject("\"status\" is not a string");
  }
  const auto status = recovery_advice_status_from_name(status_value->as_string());
  if (!status) {
    return reject("\"status\" is not one of \"ok\", \"no_recommendation\"");
  }

  const auto * failure_value = parsed.value->member("failure_class");
  if (!failure_value->is_string()) {
    return reject("\"failure_class\" is not a string");
  }
  const auto failure_class = recovery_failure_class_from_name(failure_value->as_string());
  if (!failure_class) {
    std::vector<std::string> names;
    for (const auto candidate : kAllFailureClasses) {
      names.emplace_back(recovery_failure_class_name(candidate));
    }
    return reject("\"failure_class\" is not one of " + quoted_enum_list(names));
  }

  const auto * primitive_value = parsed.value->member("recommended_primitive");
  std::optional<RecoveryPrimitive> primitive;
  if (!primitive_value->is_null()) {
    if (!primitive_value->is_string()) {
      return reject("\"recommended_primitive\" is neither a string nor null");
    }
    // Matched against a closed set; downstream never sees the model's string.
    primitive = recovery_primitive_from_name(primitive_value->as_string());
    if (!primitive) {
      std::vector<std::string> names;
      for (const auto candidate : kAllPrimitives) {
        names.emplace_back(recovery_primitive_name(candidate));
      }
      return reject(
        "\"recommended_primitive\" is not one of " + quoted_enum_list(names));
    }
  }

  if (*status == RecoveryAdviceStatus::kOk && !primitive) {
    return reject("\"status\" is \"ok\" but no primitive was recommended");
  }
  if (*status == RecoveryAdviceStatus::kNoRecommendation && primitive) {
    return reject("\"status\" is \"no_recommendation\" but a primitive was recommended");
  }

  const auto * confidence_value = parsed.value->member("confidence");
  if (!confidence_value->is_number()) {
    return reject("\"confidence\" is not a number");
  }
  const double confidence = confidence_value->as_number();
  if (confidence < 0.0 || confidence > 1.0) {
    return reject("\"confidence\" is outside [0, 1]");
  }

  const auto * explanation_value = parsed.value->member("explanation");
  if (!explanation_value->is_string()) {
    return reject("\"explanation\" is not a string");
  }
  std::string explanation = explanation_value->as_string();
  if (explanation.size() > kRecoveryExplanationMaximumBytes) {
    return reject("\"explanation\" exceeds its byte bound");
  }

  if (primitive) {
    if (confidence < minimum_confidence) {
      return reject(
        "the recommendation's confidence " + json_number(confidence, 4) +
        " is below the configured floor " + json_number(minimum_confidence, 4));
    }
    // Checked here against the set the question carried, and again against a freshly computed set
    // before anything acts on it. This catches a primitive the backend was never offered.
    if (std::ranges::find(query.permitted_primitives, *primitive) ==
      query.permitted_primitives.end())
    {
      return reject(
        std::string("\"recommended_primitive\" \"") + recovery_primitive_name(*primitive) +
        "\" was not in the permitted set the request carried");
    }
  }

  RecoveryAdvice advice;
  advice.request_id = request_id->as_string();
  advice.status = *status;
  advice.failure_class = *failure_class;
  advice.recommended_primitive = primitive;
  advice.confidence = confidence;
  advice.explanation = std::move(explanation);
  return {std::move(advice), {}};
}

std::string render_recovery_system_prompt(const RecoveryQuery & query)
{
  std::string permitted;
  for (const auto primitive : query.permitted_primitives) {
    if (!permitted.empty()) {
      permitted += ", ";
    }
    permitted += recovery_primitive_name(primitive);
  }
  if (permitted.empty()) {
    permitted = "(none)";
  }

  std::string classes;
  for (const auto failure : kAllFailureClasses) {
    if (!classes.empty()) {
      classes += ", ";
    }
    classes += recovery_failure_class_name(failure);
  }

  return
    "You advise a deterministic robotic restocking controller that has already decided what to "
    "do. Your recommendation is advice, not a command: the controller validates it against its "
    "own authorisation and ignores anything it does not permit.\n"
    "\n"
    "Classify the failure and name at most one recovery primitive.\n"
    "\n"
    "failure_class must be exactly one of: " + classes + "\n"
    "recommended_primitive must be exactly one of: " + permitted +
    ", or null when you have no recommendation.\n"
    "\n"
    "Set status to \"ok\" when you recommend a primitive and \"no_recommendation\" when you "
    "recommend none; the two must agree. confidence is a number in [0, 1]. explanation is at most "
    "1024 characters of plain prose. Echo request_id exactly. Set schema_version to \"" +
    std::string(kRecoveryAdviceSchemaVersion) +
    "\". Reply with one JSON object and nothing else: no prose before or after it, no code fence, "
    "and no members other than the ones named here.";
}

std::string render_recovery_user_prompt(const RecoveryQuery & query)
{
  std::string permitted;
  for (const auto primitive : query.permitted_primitives) {
    if (!permitted.empty()) {
      permitted += ", ";
    }
    permitted += recovery_primitive_name(primitive);
  }
  if (permitted.empty()) {
    permitted = "(none)";
  }

  std::string text;
  text += "request_id: " + query.request_id + "\n";
  text += "failed segment: " + query.failed_segment + "\n";
  text += "motion outcome: " + query.motion_outcome + "\n";
  text += "task state: " + query.task_state + "\n";
  text += "controller's report: " +
    bounded(query.observed_detail, kRecoveryObservationMaximumBytes) + "\n";
  text += "deterministic recovery authorisation: " +
    (query.deterministic_refusal.empty() ?
    std::string("a bounded replan from the arm's current state is authorised") :
    "refused because " + bounded(query.deterministic_refusal, kRecoveryObservationMaximumBytes)) +
    "\n";
  text += "what the controller will do without advice: " +
    std::string(recovery_primitive_name(query.deterministic_primitive)) + "\n";
  text += "permitted primitives: " + permitted + "\n";
  text += "recovery attempt " + std::to_string(query.recovery_attempt) + " of " +
    std::to_string(query.maximum_recovery_attempts) + "\n";
  return text;
}

std::string recovery_advice_response_schema()
{
  std::string failure_enum;
  for (const auto failure : kAllFailureClasses) {
    if (!failure_enum.empty()) {
      failure_enum += ",";
    }
    failure_enum += json_quote(recovery_failure_class_name(failure));
  }
  std::string primitive_enum;
  for (const auto primitive : kAllPrimitives) {
    if (!primitive_enum.empty()) {
      primitive_enum += ",";
    }
    primitive_enum += json_quote(recovery_primitive_name(primitive));
  }

  return
    std::string("{\"type\":\"object\",\"additionalProperties\":false,\"required\":[") +
    "\"schema_version\",\"request_id\",\"status\",\"failure_class\"," +
    "\"recommended_primitive\",\"confidence\",\"explanation\"],\"properties\":{" +
    "\"schema_version\":{\"type\":\"string\"}," +
    "\"request_id\":{\"type\":\"string\"}," +
    "\"status\":{\"enum\":[\"ok\",\"no_recommendation\"]}," +
    "\"failure_class\":{\"enum\":[" + failure_enum + "]}," +
    "\"recommended_primitive\":{\"anyOf\":[{\"enum\":[" + primitive_enum +
    "]},{\"type\":\"null\"}]}," +
    "\"confidence\":{\"type\":\"number\",\"minimum\":0,\"maximum\":1}," +
    "\"explanation\":{\"type\":\"string\"}}}";
}

std::string render_recovery_query_json(const RecoveryQuery & query)
{
  std::string permitted;
  for (const auto primitive : query.permitted_primitives) {
    if (!permitted.empty()) {
      permitted += ",";
    }
    permitted += json_quote(recovery_primitive_name(primitive));
  }

  std::string out = "{";
  out += "\"request_id\":" + json_quote(query.request_id);
  out += ",\"goal_generation\":" + std::to_string(query.goal_generation);
  out += ",\"failed_segment\":" + json_quote(query.failed_segment);
  out += ",\"motion_outcome\":" + json_quote(query.motion_outcome);
  out += ",\"task_state\":" + json_quote(query.task_state);
  out += ",\"observed_detail\":" +
    json_quote(bounded(query.observed_detail, kRecoveryObservationMaximumBytes));
  out += ",\"deterministic_refusal\":" +
    json_quote(bounded(query.deterministic_refusal, kRecoveryObservationMaximumBytes));
  out += ",\"deterministic_primitive\":" +
    json_quote(recovery_primitive_name(query.deterministic_primitive));
  out += ",\"permitted_primitives\":[" + permitted + "]";
  out += ",\"recovery_attempt\":" + std::to_string(query.recovery_attempt);
  out += ",\"maximum_recovery_attempts\":" + std::to_string(query.maximum_recovery_attempts);
  out += "}";
  return out;
}

}  // namespace restocker_reasoner
