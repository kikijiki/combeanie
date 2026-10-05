// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_startup_transport.hpp"

#include <utility>

namespace restocker_task_executor
{

bool CoordinatorStartupMailbox::arm(OperationGeneration attempt_generation)
{
  std::scoped_lock lock(mutex_);
  if (attempt_generation == 0U || armed_generation_ != 0U || request_handle_ || arrival_ ||
    collision_)
  {
    return false;
  }
  armed_generation_ = attempt_generation;
  return true;
}

StartupHandleInstall CoordinatorStartupMailbox::install_handle(
  OperationGeneration attempt_generation, WorldStateRequestHandle handle)
{
  std::scoped_lock lock(mutex_);
  const bool valid = handle.correlation.goal_generation == kStartupGoalGeneration &&
    handle.correlation.operation_generation == attempt_generation &&
    handle.service == WorldStateServiceKind::kGetSnapshot && handle.request_id > 0;
  if (!valid) {
    return StartupHandleInstall::kInvalidHandle;
  }
  if (armed_generation_ != attempt_generation) {
    return StartupHandleInstall::kRetired;
  }
  if (request_handle_) {
    return StartupHandleInstall::kDuplicate;
  }
  request_handle_ = std::move(handle);
  return StartupHandleInstall::kInstalled;
}

StartupMailboxDeposit CoordinatorStartupMailbox::deposit(StartupSnapshotArrival arrival)
{
  std::scoped_lock lock(mutex_);
  if (arrival.attempt_generation == 0U || arrival.attempt_generation != armed_generation_) {
    return StartupMailboxDeposit::kDiscarded;
  }
  if (arrival_) {
    collision_ = true;
    return StartupMailboxDeposit::kCollision;
  }
  arrival_ = std::move(arrival);
  return StartupMailboxDeposit::kAccepted;
}

std::optional<StartupMailboxTake> CoordinatorStartupMailbox::take_and_retire(
  OperationGeneration attempt_generation)
{
  std::scoped_lock lock(mutex_);
  if (attempt_generation == 0U || armed_generation_ != attempt_generation || !arrival_) {
    return std::nullopt;
  }
  StartupMailboxTake taken{
    std::move(*arrival_), std::move(request_handle_), collision_};
  armed_generation_ = 0U;
  request_handle_.reset();
  arrival_.reset();
  collision_ = false;
  return taken;
}

StartupMailboxRetirement CoordinatorStartupMailbox::retire(
  OperationGeneration attempt_generation)
{
  std::scoped_lock lock(mutex_);
  if (attempt_generation == 0U || armed_generation_ != attempt_generation) {
    return {};
  }
  StartupMailboxRetirement retired{true, std::move(request_handle_)};
  armed_generation_ = 0U;
  request_handle_.reset();
  arrival_.reset();
  collision_ = false;
  return retired;
}

StartupMailboxRetirement CoordinatorStartupMailbox::retire_any()
{
  std::scoped_lock lock(mutex_);
  StartupMailboxRetirement retired{
    armed_generation_ != 0U || request_handle_ || arrival_ || collision_,
    std::move(request_handle_)};
  armed_generation_ = 0U;
  request_handle_.reset();
  arrival_.reset();
  collision_ = false;
  return retired;
}

bool CoordinatorStartupMailbox::empty() const
{
  std::scoped_lock lock(mutex_);
  return armed_generation_ == 0U && !request_handle_ && !arrival_ && !collision_;
}

}  // namespace restocker_task_executor
