// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <type_traits>

#include "restocker_task_executor/motion_operation_evidence.hpp"

namespace restocker_task_executor
{

namespace
{

using namespace std::chrono_literals;

constexpr char kOperationId[] = "restock/test/execute_pre_grasp/0/1";

[[nodiscard]] MotionOperationDeadlines deadlines()
{
  const auto origin = SteadyTime{};
  return {
    origin + 1s, origin + 10s, origin + 2s, origin + 4s, origin + 5s,
    origin + 7s, origin + 9s, 41U, 43U};
}

[[nodiscard]] MotionOperationIdentity identity()
{
  const auto value = deadlines();
  return {
    {3U, 5U, RestockTaskCommand::kExecutePreGrasp, OperationEffect::kCancelableMotion,
      kOperationId, value.expected_result},
    7U, 47U};
}

[[nodiscard]] MotionGoalUuid uuid()
{
  MotionGoalUuid value{};
  value.back() = 1U;
  return value;
}

[[nodiscard]] MotionTerminalRecord terminal()
{
  return {
    identity(), 11U, uuid(), TrajectoryActionTerminalState::Succeeded, 1,
    std::nullopt, {SteadyTime{} + 6s, 13U}};
}

[[nodiscard]] MotionStopProofRecord stop_proof()
{
  return {
    {identity(), terminal(), SteadyTime{} + 9s},
    "test/telemetry", "test/telemetry", 20U, 22U,
    {1'000'000'000LL, MotionSourceClock::kRosTime},
    {1'200'000'000LL, MotionSourceClock::kRosTime},
    {SteadyTime{} + 7s, 14U}, {SteadyTime{} + 8s, 16U}, 3U,
    SteadyTime{} + 9s};
}

TEST(MotionOperationEvidence, StructuralValuesAreCopyableAndReceiptsAreMoveOnly)
{
  static_assert(std::is_copy_constructible_v<MotionRequestBindingRecord>);
  static_assert(std::is_copy_constructible_v<MotionGoalAcceptedRecord>);
  static_assert(std::is_copy_constructible_v<MotionTerminalRecord>);
  static_assert(std::is_copy_constructible_v<MotionStopProofRecord>);

  static_assert(std::is_move_constructible_v<MotionRequestBindingReceipt>);
  static_assert(!std::is_copy_constructible_v<MotionRequestBindingReceipt>);
  static_assert(std::is_move_constructible_v<MotionStopProofReceipt>);
  static_assert(!std::is_copy_constructible_v<MotionStopProofReceipt>);
}

TEST(MotionOperationEvidence, ValidatesExactDeadlineAndIdentityStructure)
{
  EXPECT_FALSE(invalid_motion_operation_deadlines_detail(deadlines()));
  EXPECT_FALSE(invalid_motion_operation_identity_detail(identity()));
  EXPECT_FALSE(invalid_motion_request_binding_record_detail({identity(), deadlines()}));

  auto changed = deadlines();
  changed.expected_result = changed.goal_response;
  EXPECT_TRUE(invalid_motion_operation_deadlines_detail(changed));
  changed = deadlines();
  changed.cancel_response = changed.terminal_drain + 1ns;
  EXPECT_TRUE(invalid_motion_operation_deadlines_detail(changed));
  changed = deadlines();
  changed.trajectory_fingerprint = 0U;
  EXPECT_TRUE(invalid_motion_operation_deadlines_detail(changed));

  auto changed_identity = identity();
  changed_identity.ticket.effect = OperationEffect::kReadOnly;
  EXPECT_TRUE(invalid_motion_operation_identity_detail(changed_identity));
  changed_identity = identity();
  changed_identity.execution_attempt_generation = 0U;
  EXPECT_TRUE(invalid_motion_operation_identity_detail(changed_identity));
  changed_identity = identity();
  changed_identity.ticket.goal_generation = kReservedCoordinatorGoalGeneration;
  EXPECT_TRUE(invalid_motion_operation_identity_detail(changed_identity));

  auto binding = MotionRequestBindingRecord{identity(), deadlines()};
  binding.identity.ticket.deadline += 1ns;
  EXPECT_TRUE(invalid_motion_request_binding_record_detail(binding));
}

TEST(MotionOperationEvidence, EnforcesAttemptAndUuidRules)
{
  const MotionGoalAcceptedRecord accepted{
    identity(), 11U, uuid(), {SteadyTime{} + 3s, 12U}};
  EXPECT_FALSE(invalid_motion_goal_accepted_record_detail(accepted));

  auto changed = accepted;
  changed.submission_attempt_id = 0U;
  EXPECT_TRUE(invalid_motion_goal_accepted_record_detail(changed));
  changed = accepted;
  changed.goal_uuid = {};
  EXPECT_TRUE(invalid_motion_goal_accepted_record_detail(changed));
  changed = accepted;
  changed.ingress.sequence = 0U;
  EXPECT_TRUE(invalid_motion_goal_accepted_record_detail(changed));

  EXPECT_FALSE(valid_motion_goal_uuid({}));
  EXPECT_TRUE(valid_motion_goal_uuid(uuid()));
}

TEST(MotionOperationEvidence, ValidatesClosedIngressFaultEvidence)
{
  const MotionIngressFaultKind valid_kinds[] = {
    MotionIngressFaultKind::kCriticalEvidenceLost,
    MotionIngressFaultKind::kProtocolViolation,
    MotionIngressFaultKind::kSubmissionAmbiguous,
    MotionIngressFaultKind::kClockRegression,
    MotionIngressFaultKind::kSequenceExhausted,
    MotionIngressFaultKind::kProducerAbandoned,
  };
  for (const auto kind : valid_kinds) {
    EXPECT_FALSE(
      invalid_motion_ingress_fault_record_detail(
        {identity(), 3U, 5U, SteadyTime{} + 3s, kind, "test fault"}));
  }

  auto changed = MotionIngressFaultRecord{
    identity(), 3U, 5U, SteadyTime{} + 3s,
    MotionIngressFaultKind::kProtocolViolation, "test fault"};
  changed.kind = static_cast<MotionIngressFaultKind>(255);
  EXPECT_TRUE(invalid_motion_ingress_fault_record_detail(changed));
  changed = {identity(), 0U, 5U, SteadyTime{} + 3s,
    MotionIngressFaultKind::kProtocolViolation, "test fault"};
  EXPECT_TRUE(invalid_motion_ingress_fault_record_detail(changed));
  changed = {identity(), 3U, 0U, SteadyTime{} + 3s,
    MotionIngressFaultKind::kProtocolViolation, "test fault"};
  EXPECT_TRUE(invalid_motion_ingress_fault_record_detail(changed));
  changed = {identity(), 3U, 5U, SteadyTime{} + 3s,
    MotionIngressFaultKind::kProtocolViolation, {}};
  EXPECT_TRUE(invalid_motion_ingress_fault_record_detail(changed));
  changed = {identity(), 3U, 5U, SteadyTime{} + 3s,
    MotionIngressFaultKind::kProtocolViolation, "test fault"};
  changed.identity.execution_attempt_generation = 0U;
  EXPECT_TRUE(invalid_motion_ingress_fault_record_detail(changed));
}

TEST(MotionOperationEvidence, EnforcesClosedNonSubmissionReasonMatrix)
{
  const MotionIngressStamp ingress{SteadyTime{} + 3s, 12U};
  EXPECT_FALSE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), LocalNonSubmissionRecord{
          LocalNonSubmissionReason::kLocalValidationRejected}, ingress}));
  EXPECT_FALSE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), LocalNonSubmissionRecord{LocalNonSubmissionReason::kEndpointNotReady},
        ingress}));
  EXPECT_FALSE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), TransportNonSubmissionRecord{
          TransportNonSubmissionReason::kPreboundaryTransportRejected, 11U}, ingress}));
  EXPECT_FALSE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), TransportNonSubmissionRecord{
          TransportNonSubmissionReason::kRemoteGoalRejected, 11U}, ingress}));

  const auto invalid_local = static_cast<LocalNonSubmissionReason>(255);
  EXPECT_TRUE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), LocalNonSubmissionRecord{invalid_local}, ingress}));
  const auto invalid_transport = static_cast<TransportNonSubmissionReason>(255);
  EXPECT_TRUE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), TransportNonSubmissionRecord{invalid_transport, 11U}, ingress}));
  EXPECT_TRUE(
    invalid_definitely_not_submitted_record_detail(
      {identity(), TransportNonSubmissionRecord{
          TransportNonSubmissionReason::kRemoteGoalRejected, 0U}, ingress}));
}

TEST(MotionOperationEvidence, ValidatesTerminalCancellationAndStopProofLineage)
{
  EXPECT_FALSE(invalid_motion_terminal_record_detail(terminal()));
  EXPECT_FALSE(invalid_motion_stop_proof_record_detail(stop_proof()));

  auto changed_terminal = terminal();
  changed_terminal.terminal_state = TrajectoryActionTerminalState::Unknown;
  EXPECT_TRUE(invalid_motion_terminal_record_detail(changed_terminal));
  changed_terminal = terminal();
  changed_terminal.cancellation_submission = MotionCancelSubmissionRecord{
    identity(), 11U, uuid(), 2U, MotionCancellationReason::kUserCancel,
    SteadyTime{} + 4s, {SteadyTime{} + 5s, 12U}};
  EXPECT_FALSE(invalid_motion_terminal_record_detail(changed_terminal));
  EXPECT_TRUE(exact_cancel_preceded_terminal(changed_terminal));
  changed_terminal.cancellation_submission->submission_attempt_id++;
  EXPECT_TRUE(invalid_motion_terminal_record_detail(changed_terminal));

  changed_terminal = terminal();
  changed_terminal.cancellation_submission = MotionCancelSubmissionRecord{
    identity(), 11U, uuid(), 2U, MotionCancellationReason::kUserCancel,
    SteadyTime{} + 6s, {SteadyTime{} + 7s, 14U}};
  EXPECT_TRUE(invalid_motion_terminal_record_detail(changed_terminal));
  EXPECT_FALSE(exact_cancel_preceded_terminal(changed_terminal));

  changed_terminal = terminal();
  changed_terminal.cancellation_submission = MotionCancelSubmissionRecord{
    identity(), 11U, uuid(), 2U, MotionCancellationReason::kUserCancel,
    SteadyTime{} + 5s, {changed_terminal.ingress.arrived_at, 12U}};
  EXPECT_FALSE(invalid_motion_terminal_record_detail(changed_terminal));
  EXPECT_TRUE(exact_cancel_preceded_terminal(changed_terminal));

  auto changed_stop = stop_proof();
  changed_stop.first_ingress.sequence = changed_stop.binding.terminal.ingress.sequence;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.first_ingress.arrived_at = changed_stop.binding.terminal.ingress.arrived_at;
  EXPECT_FALSE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.final_ingress.arrived_at = changed_stop.outer_deadline;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.final_telemetry_source_id = "other/source";
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));

  changed_stop = stop_proof();
  changed_stop.first_source_time.clock = MotionSourceClock::kUninitialized;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.first_source_time.nanoseconds = 0;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.final_telemetry_revision = changed_stop.first_telemetry_revision;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.final_source_time = changed_stop.first_source_time;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop = stop_proof();
  changed_stop.final_ingress = changed_stop.first_ingress;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));

  changed_stop = stop_proof();
  changed_stop.final_ingress.arrived_at = changed_stop.first_ingress.arrived_at;
  EXPECT_LT(changed_stop.first_ingress.sequence, changed_stop.final_ingress.sequence);
  EXPECT_FALSE(invalid_motion_stop_proof_record_detail(changed_stop));

  changed_stop = stop_proof();
  changed_stop.sample_count = 1U;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
  changed_stop.sample_count = 2U;
  EXPECT_TRUE(invalid_motion_stop_proof_record_detail(changed_stop));
}

TEST(MotionOperationEvidence, ProvidesStableClosedDiagnosticNames)
{
  EXPECT_STREQ(to_string(TrajectoryActionTerminalState::Succeeded), "succeeded");
  EXPECT_STREQ(
    to_string(TransportNonSubmissionReason::kPreboundaryTransportRejected),
    "preboundary_transport_rejected");
  EXPECT_STREQ(to_string(MotionCancellationReason::kAuthorityLoss), "authority_loss");
  EXPECT_STREQ(to_string(static_cast<MotionCancellationReason>(255)), "unknown");
}

}  // namespace
}  // namespace restocker_task_executor
