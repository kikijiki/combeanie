// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#ifndef DETAIL__ACCEPTED_HANDLE_ADOPTION_HPP_
#define DETAIL__ACCEPTED_HANDLE_ADOPTION_HPP_

#include <cstddef>
#include <cstdint>
#include <array>
#include <exception>
#include <functional>
#include <optional>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_operation_types.hpp"

namespace restocker_task_executor::detail
{

enum class AcceptedHandleAdoptionStatus : std::uint8_t
{
  kAdoptedPending,
  kDuplicatePending,
  kNullHandleFailStop,
  kAdoptedFirstOrphan,
  kAdoptedSecondDistinctOrphanFailStop,
  kDuplicateOrphan,
  kPendingIdentityMismatchFailStop,
};

template<typename StrongHandle>
struct InspectedAcceptedHandle
{
  StrongHandle handle{};
  std::optional<CoordinatorGoalId> goal_id;
  bool identity_extraction_failed{false};
};

/// Pins the callback-owned handle before performing fallible identity extraction.
///
/// This operation has no binding-lock parameter. Callers inspect first, then
/// acquire their binding mutex to adopt the still-pinned handle. A null handle never reaches
/// the extractor.
template<typename StrongHandle, typename Extractor>
[[nodiscard]] InspectedAcceptedHandle<StrongHandle> inspect_accepted_handle(
  StrongHandle handle, Extractor && extractor) noexcept
{
  InspectedAcceptedHandle<StrongHandle> inspected;
  inspected.handle = std::move(handle);
  if (!inspected.handle) {
    return inspected;
  }

  try {
    inspected.goal_id.emplace(
      std::invoke(std::forward<Extractor>(extractor), std::as_const(inspected.handle)));
  } catch (...) {
    inspected.identity_extraction_failed = true;
  }
  return inspected;
}

/// Owns every anomalous accepted handle without allocating while the binding lock is held.
///
/// Handle identity means stored-pointee identity (`handle.get()`), not control-block identity.
/// The caller retains duplicate inputs and must let them destruct only after releasing its lock.
template<typename StrongHandle>
class BoundedAcceptedHandleAdoption final
{
public:
  static_assert(std::is_default_constructible_v<StrongHandle>);

  BoundedAcceptedHandleAdoption() = default;
  BoundedAcceptedHandleAdoption(const BoundedAcceptedHandleAdoption &) = delete;
  BoundedAcceptedHandleAdoption & operator=(const BoundedAcceptedHandleAdoption &) = delete;
  BoundedAcceptedHandleAdoption(BoundedAcceptedHandleAdoption &&) = delete;
  BoundedAcceptedHandleAdoption & operator=(BoundedAcceptedHandleAdoption &&) = delete;
  ~BoundedAcceptedHandleAdoption() = default;

  /// Adopts an inspected handle. External synchronization is required.
  [[nodiscard]] AcceptedHandleAdoptionStatus adopt(
    InspectedAcceptedHandle<StrongHandle> & incoming,
    const CoordinatorGoalId * pending_goal_id,
    StrongHandle * pending_handle,
    bool active_binding_present) noexcept
  {
    if (!incoming.handle) {
      fail_stopped_ = true;
      return AcceptedHandleAdoptionStatus::kNullHandleFailStop;
    }

    if (incoming.identity_extraction_failed || !incoming.goal_id) {
      return adopt_orphan(incoming.handle, false);
    }

    const bool pending_binding_present =
      !active_binding_present && pending_goal_id != nullptr && pending_handle != nullptr;
    const bool exact_pending = pending_binding_present && *pending_goal_id == *incoming.goal_id;
    if (exact_pending && *pending_handle &&
      same_identity(*pending_handle, incoming.handle))
    {
      return AcceptedHandleAdoptionStatus::kDuplicatePending;
    }
    if (is_duplicate_orphan(incoming.handle)) {
      return AcceptedHandleAdoptionStatus::kDuplicateOrphan;
    }

    if (exact_pending && !*pending_handle) {
      *pending_handle = std::move(incoming.handle);
      return AcceptedHandleAdoptionStatus::kAdoptedPending;
    }
    if (exact_pending) {
      return adopt_orphan(incoming.handle, true);
    }
    return adopt_orphan(incoming.handle, false);
  }

  [[nodiscard]] bool fail_stopped() const noexcept {return fail_stopped_;}

  [[nodiscard]] std::size_t orphan_count() const noexcept
  {
    return static_cast<std::size_t>(static_cast<bool>(orphan_slots_[0])) +
           static_cast<std::size_t>(static_cast<bool>(orphan_slots_[1]));
  }

  [[nodiscard]] const StrongHandle & orphan(std::size_t index) const noexcept
  {
    return orphan_slots_[index];
  }

private:
  [[nodiscard]] static bool same_identity(
    const StrongHandle & lhs,
    const StrongHandle & rhs) noexcept
  {
    return lhs.get() == rhs.get();
  }

  [[nodiscard]] bool is_duplicate_orphan(const StrongHandle & incoming) const noexcept
  {
    return (orphan_slots_[0] && same_identity(orphan_slots_[0], incoming)) ||
           (orphan_slots_[1] && same_identity(orphan_slots_[1], incoming));
  }

  [[nodiscard]] AcceptedHandleAdoptionStatus adopt_orphan(
    StrongHandle & incoming,
    bool pending_identity_mismatch) noexcept
  {
    if (is_duplicate_orphan(incoming)) {
      return AcceptedHandleAdoptionStatus::kDuplicateOrphan;
    }

    fail_stopped_ = true;
    if (!orphan_slots_[0]) {
      orphan_slots_[0] = std::move(incoming);
      return pending_identity_mismatch ?
             AcceptedHandleAdoptionStatus::kPendingIdentityMismatchFailStop :
             AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan;
    }
    if (!orphan_slots_[1]) {
      orphan_slots_[1] = std::move(incoming);
      return pending_identity_mismatch ?
             AcceptedHandleAdoptionStatus::kPendingIdentityMismatchFailStop :
             AcceptedHandleAdoptionStatus::kAdoptedSecondDistinctOrphanFailStop;
    }

    // The callback argument still pins the third distinct handle. Returning would permit its
    // destruction without a retained authority record, so this process cannot continue safely.
    std::terminate();
  }

  std::array<StrongHandle, 2U> orphan_slots_{};
  bool fail_stopped_{false};
};

}  // namespace restocker_task_executor::detail

#endif  // DETAIL__ACCEPTED_HANDLE_ADOPTION_HPP_
