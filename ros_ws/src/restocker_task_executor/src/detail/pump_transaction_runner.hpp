// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#ifndef DETAIL__PUMP_TRANSACTION_RUNNER_HPP_
#define DETAIL__PUMP_TRANSACTION_RUNNER_HPP_

#include <concepts>
#include <cstdint>
#include <exception>
#include <functional>
#include <optional>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"

namespace restocker_task_executor::detail
{

enum class PumpWorkExit : std::uint8_t
{
  kCompleted,
  kStandardException,
  kNonStandardException,
  kInvalidLease,
};

struct PumpTransactionResult
{
  PumpWorkExit work_exit{PumpWorkExit::kCompleted};
  // A missing value means that the epilogue could not establish its synchronization boundary.
  std::optional<CoordinatorPumpLeaseReturnStatus> lease_return;
};

/// Executes pump work and its lease-return epilogue, which always runs.
///
/// The runner owns the acquired lease. Work exceptions do not escape; the epilogue receives the
/// classified work exit before deciding whether phase advancement is safe. The epilogue must be
/// nothrow because destroying a still-live lease leaves its gate outstanding (fail-closed). Work
/// may take the lease by reference and move it into a longer-lived retirement owner; the epilogue
/// then runs with the moved-from runner lease and must return the transferred lease from that
/// owner.
template<typename Work>
concept PumpWork =
  (std::invocable<Work &&, CoordinatorPumpLease &> &&
  std::same_as<std::invoke_result_t<Work &&, CoordinatorPumpLease &>, void>) ||
  (std::invocable<Work &&> && std::same_as<std::invoke_result_t<Work &&>, void>);

template<typename Work, typename Epilogue>
requires PumpWork<Work> &&
std::invocable<Epilogue &&, CoordinatorPumpLease &, PumpWorkExit> &&
std::same_as<
  std::invoke_result_t<Epilogue &&, CoordinatorPumpLease &, PumpWorkExit>,
  std::optional<CoordinatorPumpLeaseReturnStatus>> &&
std::is_nothrow_invocable_v<
  Epilogue &&, CoordinatorPumpLease &, PumpWorkExit>
[[nodiscard]] PumpTransactionResult run_pump_transaction(
  CoordinatorPumpLease lease, Work && work, Epilogue && epilogue) noexcept
{
  PumpWorkExit work_exit{PumpWorkExit::kCompleted};
  if (!lease.live()) {
    work_exit = PumpWorkExit::kInvalidLease;
  } else {
    try {
      if constexpr (std::invocable < Work &&, CoordinatorPumpLease & >) {
        std::invoke(std::forward<Work>(work), lease);
      } else {
        std::invoke(std::forward<Work>(work));
      }
    } catch (const std::exception &) {
      work_exit = PumpWorkExit::kStandardException;
    } catch (...) {
      work_exit = PumpWorkExit::kNonStandardException;
    }
  }

  auto lease_return = std::invoke(
    std::forward<Epilogue>(epilogue), lease, work_exit);
  return PumpTransactionResult{work_exit, lease_return};
}

static_assert(std::is_trivially_copyable_v<PumpTransactionResult>);
static_assert(std::is_nothrow_destructible_v<PumpTransactionResult>);

}  // namespace restocker_task_executor::detail

#endif  // DETAIL__PUMP_TRANSACTION_RUNNER_HPP_
