// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <type_traits>

namespace restocker_task_executor
{

enum class CoordinatorInboxDepositStatus : std::uint8_t
{
  kAccepted,
  kOverflowLatched,
  kInhibited,
  kEvidenceLost,
  kEvidenceConflict,
};

enum class CoordinatorInboxPersistenceStatus : std::uint8_t
{
  kEventOwned,
  kOverflowMarkerOwned,
  kInhibitedMarkerOwned,
  kEvidenceLossMarkerOwned,
  kUnresolved,
};

enum class GenerationDepositCompletion : std::uint8_t
{
  kAccepted,
  kOverflowAccounted,
  kInhibitedAccounted,
  kEvidenceLostAccounted,
  kUnknown,
};

enum class GenerationScopedInboxDepositOutcome : std::uint8_t
{
  kDeposited,
  kUnresolved,
  kSealed,
  kFailed,
};

struct CoordinatorInboxDepositResult
{
  CoordinatorInboxDepositStatus status{CoordinatorInboxDepositStatus::kEvidenceConflict};
  CoordinatorInboxPersistenceStatus persistence{
    CoordinatorInboxPersistenceStatus::kUnresolved};
};

struct GenerationScopedInboxDepositResult
{
  GenerationScopedInboxDepositOutcome outcome{GenerationScopedInboxDepositOutcome::kFailed};
  CoordinatorInboxDepositResult inbox{};
};

[[nodiscard]] constexpr GenerationDepositCompletion classify_generation_deposit(
  CoordinatorInboxDepositResult result) noexcept
{
  if (result.status == CoordinatorInboxDepositStatus::kAccepted &&
    result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
  {
    return GenerationDepositCompletion::kAccepted;
  }
  if (result.status == CoordinatorInboxDepositStatus::kOverflowLatched &&
    result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
  {
    return GenerationDepositCompletion::kAccepted;
  }
  if (result.status == CoordinatorInboxDepositStatus::kOverflowLatched &&
    result.persistence == CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned)
  {
    return GenerationDepositCompletion::kOverflowAccounted;
  }
  if (result.status == CoordinatorInboxDepositStatus::kInhibited &&
    result.persistence == CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned)
  {
    return GenerationDepositCompletion::kInhibitedAccounted;
  }
  if (result.status == CoordinatorInboxDepositStatus::kEvidenceLost &&
    result.persistence == CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned)
  {
    return GenerationDepositCompletion::kEvidenceLostAccounted;
  }
  return GenerationDepositCompletion::kUnknown;
}

static_assert(std::is_trivially_copyable_v<CoordinatorInboxDepositResult>);
static_assert(std::is_trivially_copyable_v<GenerationScopedInboxDepositResult>);

}  // namespace restocker_task_executor
