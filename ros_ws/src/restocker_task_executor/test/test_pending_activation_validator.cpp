// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <utility>

#include "detail/pending_activation_validator.hpp"

namespace restocker_task_executor::detail
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId value{};
  value[0] = seed;
  value[15] = static_cast<std::uint8_t>(seed + 1U);
  return value;
}

[[nodiscard]] SteadyTime at(std::chrono::milliseconds offset) {return SteadyTime{} + offset;}

[[nodiscard]] std::size_t route_index(PendingRouteKind kind)
{
  switch (kind) {
    case PendingRouteKind::kDrain:
      return 0U;
    case PendingRouteKind::kTransformAuthorityLoss:
      return 1U;
    case PendingRouteKind::kSteadyClockFailure:
      return 2U;
    case PendingRouteKind::kCallbackOrAdapterFailure:
      return 3U;
  }
  return kPendingActivationRouteCount;
}

[[nodiscard]] GoalTerminationKind termination_kind(PendingRouteKind kind)
{
  switch (kind) {
    case PendingRouteKind::kDrain:
      return GoalTerminationKind::kCoordinatorDrain;
    case PendingRouteKind::kTransformAuthorityLoss:
      return GoalTerminationKind::kAuthorityLoss;
    case PendingRouteKind::kSteadyClockFailure:
    case PendingRouteKind::kCallbackOrAdapterFailure:
      return GoalTerminationKind::kProtocolFailure;
  }
  return static_cast<GoalTerminationKind>(UINT8_MAX);
}

[[nodiscard]] MotionCancellationReason cancellation_reason(PendingRouteKind kind)
{
  switch (kind) {
    case PendingRouteKind::kDrain:
      return MotionCancellationReason::kCoordinatorDrain;
    case PendingRouteKind::kTransformAuthorityLoss:
      return MotionCancellationReason::kAuthorityLoss;
    case PendingRouteKind::kSteadyClockFailure:
    case PendingRouteKind::kCallbackOrAdapterFailure:
      return MotionCancellationReason::kProtocolFailure;
  }
  return static_cast<MotionCancellationReason>(UINT8_MAX);
}

[[nodiscard]] PendingActivationSummary base_summary()
{
  PendingActivationSummary value;
  value.binding = PendingHandoffBindingKey{goal_id(), 7U, 3U};
  return value;
}

[[nodiscard]] std::shared_ptr<const FirstGoalTerminationRecord> record_for(
  const PendingActivationSummary & summary, PendingRouteKind kind, SteadyTime arrived_at)
{
  return std::make_shared<const FirstGoalTerminationRecord>(
    FirstGoalTerminationRecord{
        GoalTerminationScope::kGoal, summary.binding.goal_id, summary.binding.generation,
        termination_kind(kind), cancellation_reason(kind), arrived_at, std::nullopt});
}

void observe_route(PendingActivationSummary & summary, PendingRouteKind kind, SteadyTime arrived_at)
{
  summary.routes.at(route_index(kind)) = PendingActivationRouteMetadata{true, kind, arrived_at};
  if (kind == PendingRouteKind::kDrain) {
    summary.drain_requested = true;
    summary.drain_requested_at = arrived_at;
  }
}

[[nodiscard]] GoalActivationAuthorityDecision decision_for(
  const PendingActivationSummary & summary,
  std::shared_ptr<const FirstGoalTerminationRecord> first = nullptr)
{
  const bool route_observed = std::any_of(
    summary.routes.cbegin(), summary.routes.cend(),
    [](const auto & route) {return route.observed;});
  const bool non_drain_observed = std::any_of(
    summary.routes.cbegin(), summary.routes.cend(),
    [](const auto & route) {return route.observed && route.kind != PendingRouteKind::kDrain;});
  GoalActivationAuthority authority;
  authority.phase = GoalSlotPhase::kPendingAcceptance;
  authority.goal_id = summary.binding.goal_id;
  authority.generation = summary.binding.generation;
  authority.safe_abort_requested = route_observed;
  authority.termination_intent =
    non_drain_observed ?
    GoalTerminationIntent::kSafeAbort :
    (route_observed ? GoalTerminationIntent::kShutdownDrain : GoalTerminationIntent::kNone);
  authority.first_termination = std::move(first);
  return {GoalActivationAuthorityStatus::kObserved, std::move(authority)};
}

[[nodiscard]] GoalActivationAuthorityDecision valid_route_decision(
  PendingActivationSummary & summary, PendingRouteKind kind, SteadyTime arrived_at = at(10ms))
{
  observe_route(summary, kind, arrived_at);
  auto first = record_for(summary, kind, arrived_at);
  summary.expected_first_termination = first.get();
  return decision_for(summary, std::move(first));
}

TEST(PendingActivationValidator, AcceptsNoRouteAndEveryPendingRouteKind)
{
  {
    const auto summary = base_summary();
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision_for(summary)),
      PendingActivationValidationError::kNone);
  }

  for (const auto kind :
    {PendingRouteKind::kDrain, PendingRouteKind::kTransformAuthorityLoss,
      PendingRouteKind::kSteadyClockFailure, PendingRouteKind::kCallbackOrAdapterFailure})
  {
    SCOPED_TRACE(static_cast<int>(kind));
    auto summary = base_summary();
    const auto decision = valid_route_decision(summary, kind);
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision),
      PendingActivationValidationError::kNone);
  }
}

TEST(PendingActivationValidator, AcceptsDrainFollowedByStrongerNonDrainRoute)
{
  auto summary = base_summary();
  observe_route(summary, PendingRouteKind::kDrain, at(10ms));
  observe_route(summary, PendingRouteKind::kTransformAuthorityLoss, at(20ms));
  auto first = record_for(summary, PendingRouteKind::kDrain, at(10ms));
  summary.expected_first_termination = first.get();
  const auto decision = decision_for(summary, std::move(first));

  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kNone);
}

TEST(PendingActivationValidator, RejectsEveryNonObservedAndUnknownDecisionStatus)
{
  const auto summary = base_summary();
  for (const auto status :
    {GoalActivationAuthorityStatus::kInvalidArgument, GoalActivationAuthorityStatus::kInactive,
      GoalActivationAuthorityStatus::kGoalMismatch, GoalActivationAuthorityStatus::kWrongPhase,
      GoalActivationAuthorityStatus::kMalformedState,
      GoalActivationAuthorityStatus::kSynchronizationFailed,
      static_cast<GoalActivationAuthorityStatus>(UINT8_MAX)})
  {
    SCOPED_TRACE(static_cast<int>(status));
    auto decision = decision_for(summary);
    decision.status = status;
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision),
      PendingActivationValidationError::kDecisionStatus);
  }
}

TEST(PendingActivationValidator, RejectsObservedDecisionWithoutAuthority)
{
  const auto summary = base_summary();
  const GoalActivationAuthorityDecision decision{GoalActivationAuthorityStatus::kObserved,
    std::nullopt};
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kMissingAuthority);
}

TEST(PendingActivationValidator, RejectsMalformedPendingBinding)
{
  for (std::size_t variant = 0U; variant < 4U; ++variant) {
    SCOPED_TRACE(variant);
    auto summary = base_summary();
    if (variant == 0U) {
      summary.binding.goal_id = {};
    } else if (variant == 1U) {
      summary.binding.generation = 0U;
    } else if (variant == 2U) {
      summary.binding.generation = kReservedCoordinatorGoalGeneration;
    } else {
      summary.binding.incarnation = 0U;
    }
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision_for(summary)),
      PendingActivationValidationError::kInvalidPendingBinding);
  }
}

TEST(PendingActivationValidator, RejectsWrongAndUnknownAdmissionPhase)
{
  const auto summary = base_summary();
  for (const auto phase :
    {GoalSlotPhase::kIdle, GoalSlotPhase::kActive, static_cast<GoalSlotPhase>(UINT8_MAX)})
  {
    SCOPED_TRACE(static_cast<int>(phase));
    auto decision = decision_for(summary);
    decision.authority->phase = phase;
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision),
      PendingActivationValidationError::kWrongPhase);
  }
}

TEST(PendingActivationValidator, RejectsGoalIdentityAndGenerationMismatch)
{
  const auto summary = base_summary();
  auto wrong_id = decision_for(summary);
  wrong_id.authority->goal_id = goal_id(9U);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, wrong_id),
    PendingActivationValidationError::kIdentityMismatch);

  auto wrong_generation = decision_for(summary);
  ++wrong_generation.authority->generation;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, wrong_generation),
    PendingActivationValidationError::kIdentityMismatch);
}

TEST(PendingActivationValidator, RejectsAdmissionInhibitionAndMutationSubmission)
{
  const auto summary = base_summary();
  auto inhibited = decision_for(summary);
  inhibited.authority->inhibited = true;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, inhibited),
    PendingActivationValidationError::kInhibited);

  auto mutation = decision_for(summary);
  mutation.authority->mutation_submission = true;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, mutation),
    PendingActivationValidationError::kMutationSubmission);
}

TEST(PendingActivationValidator, RejectsCancelAndDeadlineAuthority)
{
  const auto summary = base_summary();
  auto cancel = decision_for(summary);
  cancel.authority->cancel_requested = true;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, cancel),
    PendingActivationValidationError::kCancelRequested);

  auto deadline = decision_for(summary);
  deadline.authority->task_deadline_exceeded = true;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, deadline),
    PendingActivationValidationError::kTaskDeadlineExceeded);
}

TEST(PendingActivationValidator, RejectsMalformedRouteIndexKindAndTimestamp)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  summary.routes[0U].kind = PendingRouteKind::kTransformAuthorityLoss;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kRouteIndexMismatch);

  summary = base_summary();
  decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  summary.routes[0U].kind = static_cast<PendingRouteKind>(UINT8_MAX);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kRouteIndexMismatch);

  summary = base_summary();
  decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  summary.routes[0U].arrived_at = SteadyTime::max();
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kRouteTimestampInvalid);
}

TEST(PendingActivationValidator, RejectsSafeAbortAndIntentMismatch)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  decision.authority->safe_abort_requested = false;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kTerminationPolicyMismatch);

  decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  decision.authority->termination_intent = GoalTerminationIntent::kSafeAbort;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kTerminationPolicyMismatch);

  const auto no_route = base_summary();
  auto unknown = decision_for(no_route);
  unknown.authority->termination_intent = static_cast<GoalTerminationIntent>(UINT8_MAX);
  EXPECT_EQ(
    validate_pending_activation_authority(no_route, unknown),
    PendingActivationValidationError::kTerminationPolicyMismatch);
}

TEST(PendingActivationValidator, RequiresExactFirstTerminationPointerLineage)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  auto foreign =
    std::make_shared<const FirstGoalTerminationRecord>(*decision.authority->first_termination);
  decision.authority->first_termination = std::move(foreign);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationLineageMismatch);

  decision.authority->first_termination.reset();
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationLineageMismatch);
}

TEST(PendingActivationValidator, RejectsUnexpectedFirstTerminationWithoutRoute)
{
  auto summary = base_summary();
  auto first = record_for(summary, PendingRouteKind::kDrain, at(10ms));
  summary.expected_first_termination = first.get();
  const auto decision = decision_for(summary, std::move(first));
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationLineageMismatch);
}

TEST(PendingActivationValidator, RejectsInvalidFirstTerminationRecords)
{
  const auto expect_invalid = [](auto mutate) {
    auto summary = base_summary();
    auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
    auto changed = *decision.authority->first_termination;
    mutate(changed);
    auto replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
    summary.expected_first_termination = replacement.get();
    decision.authority->first_termination = std::move(replacement);
    EXPECT_EQ(
      validate_pending_activation_authority(summary, decision),
      PendingActivationValidationError::kInvalidFirstTermination);
  };

  expect_invalid([](auto & record) {record.scope = static_cast<GoalTerminationScope>(UINT8_MAX);});
  expect_invalid([](auto & record) {record.goal_id = {};});
  expect_invalid([](auto & record) {record.goal_generation = 0U;});
  expect_invalid([](auto & record) {record.kind = static_cast<GoalTerminationKind>(UINT8_MAX);});
  expect_invalid(
    [](auto & record) {record.cancellation_reason = MotionCancellationReason::kSafeAbort;});
  expect_invalid([](auto & record) {record.arrived_at = SteadyTime::max();});
  expect_invalid(
    [](auto & record) {
      record.source_generation = GoalTerminationSourceGeneration{GoalTerminationSource::kNone, 1U};
    });
}

TEST(PendingActivationValidator, RejectsFirstTerminationIdentityMismatch)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  auto changed = *decision.authority->first_termination;
  changed.goal_id = goal_id(8U);
  auto replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
  summary.expected_first_termination = replacement.get();
  decision.authority->first_termination = std::move(replacement);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationIdentityMismatch);

  decision = valid_route_decision(summary = base_summary(), PendingRouteKind::kDrain);
  changed = *decision.authority->first_termination;
  ++changed.goal_generation;
  replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
  summary.expected_first_termination = replacement.get();
  decision.authority->first_termination = std::move(replacement);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationIdentityMismatch);
}

TEST(PendingActivationValidator, RejectsOtherwiseValidSourceGeneration)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kTransformAuthorityLoss);
  auto changed = *decision.authority->first_termination;
  changed.source_generation =
    GoalTerminationSourceGeneration{GoalTerminationSource::kAuthorityObservation, 1U};
  auto replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
  summary.expected_first_termination = replacement.get();
  decision.authority->first_termination = std::move(replacement);
  ASSERT_EQ(validate_goal_termination_record(changed), GoalTerminationValidationError::kNone);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kUnexpectedFirstTerminationSource);
}

TEST(PendingActivationValidator, RequiresFirstRecordToMatchOneObservedRoute)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  auto changed = *decision.authority->first_termination;
  changed.arrived_at += 1ms;
  auto replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
  summary.expected_first_termination = replacement.get();
  decision.authority->first_termination = std::move(replacement);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationRouteMismatch);

  decision = valid_route_decision(summary = base_summary(), PendingRouteKind::kDrain);
  changed = *decision.authority->first_termination;
  changed.kind = GoalTerminationKind::kShutdown;
  changed.cancellation_reason = MotionCancellationReason::kShutdown;
  replacement = std::make_shared<const FirstGoalTerminationRecord>(changed);
  summary.expected_first_termination = replacement.get();
  decision.authority->first_termination = std::move(replacement);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kFirstTerminationRouteMismatch);
}

TEST(PendingActivationValidator, RequiresExactDrainTimestampPresenceAndValue)
{
  auto summary = base_summary();
  auto decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  summary.drain_requested = false;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kDrainTimestampMismatch);

  summary = base_summary();
  decision = valid_route_decision(summary, PendingRouteKind::kDrain);
  summary.drain_requested_at += 1ms;
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision),
    PendingActivationValidationError::kDrainTimestampMismatch);

  summary = base_summary();
  summary.drain_requested = true;
  summary.drain_requested_at = at(10ms);
  EXPECT_EQ(
    validate_pending_activation_authority(summary, decision_for(summary)),
    PendingActivationValidationError::kDrainTimestampMismatch);
}

static_assert(std::is_trivially_copyable_v<PendingActivationSummary>);
static_assert(
  noexcept(validate_pending_activation_authority(
    std::declval<const PendingActivationSummary &>(),
    std::declval<const GoalActivationAuthorityDecision &>())));

}  // namespace
}  // namespace restocker_task_executor::detail
