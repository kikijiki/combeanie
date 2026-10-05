// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "detail/pending_activation_validator.hpp"

#include <algorithm>
#include <optional>

namespace restocker_task_executor::detail
{
namespace
{

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id) noexcept
{
  return std::any_of(
    goal_id.cbegin(), goal_id.cend(),
    [](std::uint8_t byte) {return byte != 0U;});
}

[[nodiscard]] constexpr std::optional<std::size_t> route_index(PendingRouteKind kind) noexcept
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
  return std::nullopt;
}

[[nodiscard]] constexpr std::optional<GoalTerminationKind> termination_kind(
  PendingRouteKind kind) noexcept
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
  return std::nullopt;
}

}  // namespace

PendingActivationValidationError validate_pending_activation_authority(
  const PendingActivationSummary & pending,
  const GoalActivationAuthorityDecision & decision) noexcept
{
  if (decision.status != GoalActivationAuthorityStatus::kObserved) {
    return PendingActivationValidationError::kDecisionStatus;
  }
  if (!decision.authority) {
    return PendingActivationValidationError::kMissingAuthority;
  }
  if (!valid_goal_id(pending.binding.goal_id) ||
    !admissible_goal_generation(pending.binding.generation) ||
    pending.binding.incarnation == 0U)
  {
    return PendingActivationValidationError::kInvalidPendingBinding;
  }

  const auto & authority = *decision.authority;
  if (authority.phase != GoalSlotPhase::kPendingAcceptance) {
    return PendingActivationValidationError::kWrongPhase;
  }
  if (authority.goal_id != pending.binding.goal_id ||
    authority.generation != pending.binding.generation)
  {
    return PendingActivationValidationError::kIdentityMismatch;
  }
  if (authority.inhibited) {
    return PendingActivationValidationError::kInhibited;
  }
  if (authority.mutation_submission) {
    return PendingActivationValidationError::kMutationSubmission;
  }
  if (authority.cancel_requested) {
    return PendingActivationValidationError::kCancelRequested;
  }
  if (authority.task_deadline_exceeded) {
    return PendingActivationValidationError::kTaskDeadlineExceeded;
  }

  bool route_observed = false;
  bool non_drain_route_observed = false;
  bool drain_route_observed = false;
  for (std::size_t index = 0U; index < pending.routes.size(); ++index) {
    const auto & route = pending.routes[index];
    if (!route.observed) {
      continue;
    }
    const auto expected_index = route_index(route.kind);
    if (!expected_index || *expected_index != index) {
      return PendingActivationValidationError::kRouteIndexMismatch;
    }
    if (route.arrived_at == SteadyTime::max()) {
      return PendingActivationValidationError::kRouteTimestampInvalid;
    }
    route_observed = true;
    drain_route_observed |= route.kind == PendingRouteKind::kDrain;
    non_drain_route_observed |= route.kind != PendingRouteKind::kDrain;
  }

  const auto expected_intent =
    non_drain_route_observed ?
    GoalTerminationIntent::kSafeAbort :
    (route_observed ? GoalTerminationIntent::kShutdownDrain : GoalTerminationIntent::kNone);
  if (authority.safe_abort_requested != route_observed ||
    authority.termination_intent != expected_intent)
  {
    return PendingActivationValidationError::kTerminationPolicyMismatch;
  }

  if (authority.first_termination.get() != pending.expected_first_termination) {
    return PendingActivationValidationError::kFirstTerminationLineageMismatch;
  }
  if (!route_observed) {
    if (authority.first_termination) {
      return PendingActivationValidationError::kFirstTerminationLineageMismatch;
    }
    return pending.drain_requested ? PendingActivationValidationError::kDrainTimestampMismatch :
           PendingActivationValidationError::kNone;
  }
  if (!authority.first_termination) {
    return PendingActivationValidationError::kFirstTerminationLineageMismatch;
  }

  const auto & first = *authority.first_termination;
  if (validate_goal_termination_record(first) != GoalTerminationValidationError::kNone) {
    return PendingActivationValidationError::kInvalidFirstTermination;
  }
  if (first.goal_id != pending.binding.goal_id ||
    first.goal_generation != pending.binding.generation)
  {
    return PendingActivationValidationError::kFirstTerminationIdentityMismatch;
  }
  if (first.source_generation) {
    return PendingActivationValidationError::kUnexpectedFirstTerminationSource;
  }

  bool first_matches_route = false;
  for (const auto & route : pending.routes) {
    if (!route.observed) {
      continue;
    }
    const auto expected_kind = termination_kind(route.kind);
    first_matches_route |=
      expected_kind && first.kind == *expected_kind && first.arrived_at == route.arrived_at;
  }
  if (!first_matches_route) {
    return PendingActivationValidationError::kFirstTerminationRouteMismatch;
  }

  const auto & drain_route = pending.routes[0U];
  if (drain_route_observed != pending.drain_requested ||
    (drain_route_observed && pending.drain_requested_at != drain_route.arrived_at))
  {
    return PendingActivationValidationError::kDrainTimestampMismatch;
  }
  return PendingActivationValidationError::kNone;
}

}  // namespace restocker_task_executor::detail
