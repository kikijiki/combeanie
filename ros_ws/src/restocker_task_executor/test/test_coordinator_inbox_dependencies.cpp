// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <type_traits>

#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"
#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/restock_coordinator_inbox.hpp"
#include "restocker_task_executor/restock_goal_context.hpp"

static_assert(
  std::is_trivially_copyable_v<
    restocker_task_executor::CoordinatorInboxDepositResult>);

int main()
{
  return 0;
}
