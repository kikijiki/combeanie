// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/coordinator_inbox_deposit_result.hpp"

namespace restocker_task_executor
{

template<typename Deposit>
requires std::same_as<std::invoke_result_t<Deposit &&>, CoordinatorInboxDepositResult>
[[nodiscard]] GenerationScopedInboxDepositResult deposit_for_generation(
  const std::shared_ptr<CoordinatorGenerationQuiescence> & gate, const CoordinatorGoalId & goal_id,
  GoalGeneration generation, Deposit && deposit) noexcept
{
  const auto fail = [&gate]() noexcept {
    if (gate) {
      (void)gate->mark_synchronization_failure();
    }
    return GenerationScopedInboxDepositResult{};
  };

  if (!gate) {
    return {};
  }

  try {
    auto beginning = gate->begin_deposit(goal_id, generation);
    if (beginning.status() == GenerationDepositBeginStatus::kSealed) {
      return {GenerationScopedInboxDepositOutcome::kSealed,
        {CoordinatorInboxDepositStatus::kEvidenceConflict,
          CoordinatorInboxPersistenceStatus::kUnresolved}};
    }
    if (beginning.status() != GenerationDepositBeginStatus::kStarted) {
      return fail();
    }

    auto permit = beginning.take_deposit_permit();
    if (!permit) {
      return fail();
    }

    const auto inbox_result = std::invoke(std::forward<Deposit>(deposit));
    const auto disposition = classify_generation_deposit(inbox_result);
    const auto completion = gate->complete_deposit(*permit, disposition);
    if (completion == GenerationDepositCompletionStatus::kCompleted &&
      disposition != GenerationDepositCompletion::kUnknown)
    {
      return {GenerationScopedInboxDepositOutcome::kDeposited, inbox_result};
    }
    if (completion == GenerationDepositCompletionStatus::kCompletedUnresolved &&
      disposition == GenerationDepositCompletion::kUnknown)
    {
      return {GenerationScopedInboxDepositOutcome::kUnresolved, inbox_result};
    }
    return fail();
  } catch (...) {
    return fail();
  }
}

static_assert(std::is_nothrow_move_constructible_v<GenerationScopedInboxDepositResult>);
static_assert(std::is_nothrow_destructible_v<GenerationScopedInboxDepositResult>);

}  // namespace restocker_task_executor
