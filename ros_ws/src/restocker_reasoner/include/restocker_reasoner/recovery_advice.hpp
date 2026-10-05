// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace restocker_reasoner
{

// The complete set of recovery actions a reasoner may name. Each is already implemented and
// validated by the deterministic task executor. Naming one does not permit it: the deterministic
// authorisation decides at the moment of action, independent of the model's output.
enum class RecoveryPrimitive : std::uint8_t
{
  // Resubmit the failed segment unchanged. This is the task machine's bounded operation retry,
  // available only while `max_operation_retries` has not been spent.
  kRetrySegment,
  // Take the deterministic recovery: replan from the arm's current state at the resume state
  // `recovery_resume_for()` chooses for the failed one. Available only when the driver's motion
  // stop evidence authorises a replan.
  kResumeAtRecoveryState,
  // Stop and hand the cell to an operator. Always available.
  kAbandonTask,
};

// The failure vocabulary the reasoner classifies into: failures the system produces that can
// reach the reasoner. A failure the recovery authorisation refuses to replan leaves stopping as
// the only permitted primitive, so no question is asked. Cancellations, port timeouts, an
// unavailable backend, a stop inside a linear segment and non-motion failures are therefore
// excluded. `kUnknown` lets a model say it does not recognise a failure.
enum class RecoveryFailureClass : std::uint8_t
{
  // "approach motion planning failed: linear path stopped 41.6667% of the way to the goal after
  // 17 interpolated waypoints". The Cartesian path could not be completed from the configuration
  // the preceding free-space traverse chose. Nothing was commanded; deterministic recovery
  // resumes at that traverse.
  kLinearPathTruncated,
  // "pre-grasp motion execution failed: trajectory execution was rejected or aborted (MoveIt
  // reported CONTROL_FAILED, code -4)". A free-space traverse the controllers stopped.
  kTrajectoryExecutionAborted,
  // "pre-insert motion planning failed: motion planning did not produce a valid solution (MoveIt
  // reported FAILURE)". The sampling planner found nothing in its budget; nothing was commanded.
  kFreeSpacePlanningFailed,
  kUnknown,
};

enum class RecoveryAdviceStatus : std::uint8_t
{
  kOk,
  kNoRecommendation,
};

[[nodiscard]] const char * recovery_primitive_name(RecoveryPrimitive primitive) noexcept;
[[nodiscard]] std::optional<RecoveryPrimitive> recovery_primitive_from_name(
  std::string_view name) noexcept;
[[nodiscard]] const char * recovery_failure_class_name(RecoveryFailureClass failure) noexcept;
[[nodiscard]] std::optional<RecoveryFailureClass> recovery_failure_class_from_name(
  std::string_view name) noexcept;
[[nodiscard]] const char * recovery_advice_status_name(RecoveryAdviceStatus status) noexcept;
[[nodiscard]] std::optional<RecoveryAdviceStatus> recovery_advice_status_from_name(
  std::string_view name) noexcept;

inline constexpr std::string_view kRecoveryAdviceSchemaVersion =
  "restocker.reasoner.recovery.response.v1";
inline constexpr std::size_t kRecoveryExplanationMaximumBytes = 1024U;
inline constexpr std::size_t kRecoveryObservationMaximumBytes = 1024U;

// What the deterministic system knows about one failure, as handed to a reasoner. Every field
// comes from the executor's own evidence.
struct RecoveryQuery
{
  // Correlates the request with the response and with the audit record. Generated locally.
  std::string request_id;
  std::uint64_t goal_generation{0U};
  // The segment that failed, the backend outcome it failed with, and the task state at the time,
  // in the executor's own words.
  std::string failed_segment;
  std::string motion_outcome;
  std::string task_state;
  // The backend's verbatim diagnosis, bounded.
  std::string observed_detail;
  // Empty when the deterministic authorisation would permit a replan; otherwise its refusal text.
  std::string deterministic_refusal;
  // What the deterministic policy would do with no advice at all.
  RecoveryPrimitive deterministic_primitive{RecoveryPrimitive::kAbandonTask};
  // The primitives the deterministic authorisation permitted when the question was asked. The set
  // is recomputed before the recommendation is acted on.
  std::vector<RecoveryPrimitive> permitted_primitives;
  std::size_t recovery_attempt{0U};
  std::size_t maximum_recovery_attempts{0U};
};

// A recommendation that has passed schema validation. It says the document was well formed and
// self-consistent, not that the recommendation may be taken.
struct RecoveryAdvice
{
  std::string request_id;
  RecoveryAdviceStatus status{RecoveryAdviceStatus::kNoRecommendation};
  RecoveryFailureClass failure_class{RecoveryFailureClass::kUnknown};
  // Set exactly when status is kOk.
  std::optional<RecoveryPrimitive> recommended_primitive;
  double confidence{0.0};
  // Bounded audit text. Never parsed, never matched against, never executed.
  std::string explanation;
};

struct RecoveryAdviceParse
{
  std::optional<RecoveryAdvice> advice;
  // Empty exactly when advice holds a value. Written into the audit record verbatim.
  std::string rejection;
};

// Validate one response document against the recommendation contract.
//
// The document must be strict JSON, a single object, with the contract's member names and types.
// Rejections: unknown or missing members, an unknown primitive name or failure class, a
// confidence outside [0, 1] or below the configured floor, an explanation over its bound, a
// request id other than the one asked, a status that disagrees with the presence of a
// recommendation, or a primitive not in the permitted set sent with the question.
//
// Passing is necessary, not sufficient: the permitted set is checked again against a freshly
// computed one before acting.
[[nodiscard]] RecoveryAdviceParse parse_recovery_advice(
  std::string_view document, const RecoveryQuery & query, double minimum_confidence);

// The instructions and the failure description sent to a chat backend. Deterministic in the
// query, so the same failure always asks the same question.
[[nodiscard]] std::string render_recovery_system_prompt(const RecoveryQuery & query);
[[nodiscard]] std::string render_recovery_user_prompt(const RecoveryQuery & query);

// The JSON Schema handed to backends that support constrained decoding. Validation runs
// identically whether or not the backend honoured it.
[[nodiscard]] std::string recovery_advice_response_schema();

// A canonical one-line rendering of the query, for the audit record.
[[nodiscard]] std::string render_recovery_query_json(const RecoveryQuery & query);

}  // namespace restocker_reasoner
