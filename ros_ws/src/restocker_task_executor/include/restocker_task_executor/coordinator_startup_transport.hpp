// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <restocker_interfaces/srv/get_world_state.hpp>

#include "restocker_task_executor/coordinator_startup.hpp"
#include "restocker_task_executor/world_state_async_port.hpp"

namespace restocker_task_executor
{

struct StartupSnapshotArrival
{
  OperationGeneration attempt_generation{0};
  SteadyTime arrived_at{};
  std::shared_ptr<const restocker_interfaces::srv::GetWorldState::Response> response;
  std::string transport_error;
};

enum class StartupMailboxDeposit : std::uint8_t
{
  kAccepted,
  kDiscarded,
  kCollision,
};

enum class StartupHandleInstall : std::uint8_t
{
  kInstalled,
  kRetired,
  kInvalidHandle,
  kDuplicate,
};

struct StartupMailboxTake
{
  StartupSnapshotArrival arrival;
  std::optional<WorldStateRequestHandle> request_handle;
  bool collision{false};
};

struct StartupMailboxRetirement
{
  bool retired{false};
  std::optional<WorldStateRequestHandle> request_handle;
};

// Thread-safe lifetime boundary between reentrant service callbacks and the orchestration pump.
class CoordinatorStartupMailbox
{
public:
  [[nodiscard]] bool arm(OperationGeneration attempt_generation);
  [[nodiscard]] StartupHandleInstall install_handle(
    OperationGeneration attempt_generation, WorldStateRequestHandle handle);
  [[nodiscard]] StartupMailboxDeposit deposit(StartupSnapshotArrival arrival);
  [[nodiscard]] std::optional<StartupMailboxTake> take_and_retire(
    OperationGeneration attempt_generation);
  [[nodiscard]] StartupMailboxRetirement retire(OperationGeneration attempt_generation);
  [[nodiscard]] StartupMailboxRetirement retire_any();
  [[nodiscard]] bool empty() const;

private:
  mutable std::mutex mutex_;
  OperationGeneration armed_generation_{0};
  std::optional<WorldStateRequestHandle> request_handle_;
  std::optional<StartupSnapshotArrival> arrival_;
  bool collision_{false};
};

}  // namespace restocker_task_executor
