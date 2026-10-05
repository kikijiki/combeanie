// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "restocker_task_executor/coordinator_handoff_reducer.hpp"
#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor::detail
{

inline constexpr std::size_t kPendingActivationRouteCount = 4U;

// Scalar projection of one sticky pending-route observation. Unobserved entries carry no
// authority; their remaining fields are ignored.
struct PendingActivationRouteMetadata
{
  bool observed{false};
  PendingRouteKind kind{PendingRouteKind::kDrain};
  SteadyTime arrived_at{};
};

// Allocation-free view of the pending facts which admission must corroborate before activation.
// The raw first-record pointer is identity evidence only. Its shared owner remains in pending state
// for the whole validation call.
struct PendingActivationSummary
{
  PendingHandoffBindingKey binding{};
  std::array<PendingActivationRouteMetadata, kPendingActivationRouteCount> routes{};
  bool drain_requested{false};
  SteadyTime drain_requested_at{};
  const FirstGoalTerminationRecord * expected_first_termination{nullptr};
};

static_assert(std::is_trivially_copyable_v<PendingActivationRouteMetadata>);
static_assert(std::is_trivially_copyable_v<PendingActivationSummary>);

enum class PendingActivationValidationError : std::uint8_t
{
  kNone,
  kDecisionStatus,
  kMissingAuthority,
  kInvalidPendingBinding,
  kWrongPhase,
  kIdentityMismatch,
  kInhibited,
  kMutationSubmission,
  kCancelRequested,
  kTaskDeadlineExceeded,
  kRouteIndexMismatch,
  kRouteTimestampInvalid,
  kTerminationPolicyMismatch,
  kFirstTerminationLineageMismatch,
  kInvalidFirstTermination,
  kFirstTerminationIdentityMismatch,
  kUnexpectedFirstTerminationSource,
  kFirstTerminationRouteMismatch,
  kDrainTimestampMismatch,
};

[[nodiscard]] PendingActivationValidationError validate_pending_activation_authority(
  const PendingActivationSummary & pending,
  const GoalActivationAuthorityDecision & decision) noexcept;

}  // namespace restocker_task_executor::detail
