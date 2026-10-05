// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#ifndef DETAIL__PENDING_ROUTE_CLAIMANT_HPP_
#define DETAIL__PENDING_ROUTE_CLAIMANT_HPP_

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include "detail/pending_route_executor.hpp"
#include "restocker_task_executor/coordinator_handoff_reducer.hpp"

namespace restocker_task_executor::detail
{

struct PendingRouteMetadata
{
  PendingRouteKind kind{PendingRouteKind::kDrain};
  SteadyTime arrived_at{};
  std::string detail;
  bool diagnostic_degraded{false};
};

using PendingRouteMetadataSlots = std::array<std::optional<PendingRouteMetadata>, 4U>;

[[nodiscard]] constexpr std::size_t pending_route_metadata_index(PendingRouteKind kind) noexcept
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
  return 4U;
}

/// Atomically installs one pending-route reducer token and its owned node metadata.
///
/// The caller owns synchronization. Both strings and the empty execution slot must be prepared
/// before taking that lock. On kClaimed this function nothrow-moves them directly into metadata and
/// execution storage supplied by the caller; router work remains a separate out-of-lock step.
class PendingRouteClaimant final
{
public:
  PendingRouteClaimant() = delete;

  [[nodiscard]] static PendingRouteClaimStatus claim(
    PendingAcceptedHandoffMachine & handoff, const PendingHandoffBindingKey & binding,
    PendingRouteMetadataSlots & metadata, PendingRouteKind kind, SteadyTime arrived_at,
    std::string & owned_work_detail, std::string & owned_metadata_detail,
    std::optional<PendingRouteExecutionWork> & execution) noexcept
  {
    const auto index = pending_route_metadata_index(kind);
    if (!valid_pending_route_kind(kind) || index >= metadata.size() || execution ||
      owned_work_detail != owned_metadata_detail)
    {
      handoff.enter_cleanup_only();
      return PendingRouteClaimStatus::kCleanupOnly;
    }

    const auto & existing = metadata[index];
    const auto relation =
      !existing ? PendingRouteObservationRelation::kNewDistinct :
      (existing->arrived_at == arrived_at && existing->detail == owned_metadata_detail ?
      PendingRouteObservationRelation::kExactDuplicate :
      PendingRouteObservationRelation::kCollision);
    auto decision = handoff.observe_route(binding, kind, relation);
    if (decision.status() != PendingRouteClaimStatus::kClaimed) {
      return decision.status();
    }

    auto token = decision.take_work();
    if (!token) {
      handoff.enter_cleanup_only();
      return PendingRouteClaimStatus::kCleanupOnly;
    }
    metadata[index].emplace(
      PendingRouteMetadata{kind, arrived_at, std::move(owned_metadata_detail), false});
    execution.emplace(std::move(*token), arrived_at, std::move(owned_work_detail));
    return PendingRouteClaimStatus::kClaimed;
  }
};

static_assert(std::is_nothrow_move_constructible_v<PendingRouteMetadata>);
static_assert(std::is_nothrow_move_assignable_v<PendingRouteMetadata>);
static_assert(
  noexcept(std::declval<std::optional<PendingRouteMetadata> &>().emplace(
    std::declval<PendingRouteMetadata &&>())));
static_assert(std::is_nothrow_move_constructible_v<PendingRouteMetadataSlots>);
static_assert(std::is_nothrow_move_assignable_v<PendingRouteMetadataSlots>);

}  // namespace restocker_task_executor::detail

#endif  // DETAIL__PENDING_ROUTE_CLAIMANT_HPP_
