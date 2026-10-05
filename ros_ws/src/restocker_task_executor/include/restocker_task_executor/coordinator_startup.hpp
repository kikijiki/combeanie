// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

inline constexpr GoalGeneration kStartupGoalGeneration = kReservedCoordinatorGoalGeneration;

enum class CoordinatorStartupStatus : std::uint8_t
{
  kWaitingForAuthority,
  kProbePending,
  kReady,
  kOrphanedReservation,
  kFaulted,
};

struct CoordinatorStartupSnapshot
{
  CoordinatorStartupStatus status{CoordinatorStartupStatus::kWaitingForAuthority};
  OperationGeneration attempt_generation{0};
  std::optional<restocker_world_state::TaskReservation> orphan;
  std::string detail{"waiting for authoritative world-state snapshot"};
};

struct CoordinatorStartupConfig
{
  std::chrono::milliseconds snapshot_timeout{1000};
  std::chrono::milliseconds retry_period{250};
};

struct StartupAuthorityClassification
{
  CoordinatorStartupStatus status{CoordinatorStartupStatus::kFaulted};
  std::optional<restocker_world_state::TaskReservation> orphan;
  std::string detail;
};

enum class StartupProbeActionKind : std::uint8_t
{
  kNone,
  kSubmit,
  kRetireTimedOut,
};

struct StartupProbeAction
{
  StartupProbeActionKind kind{StartupProbeActionKind::kNone};
  OperationGeneration attempt_generation{0};
  SteadyTime deadline{};
};

enum class StartupCompletionDisposition : std::uint8_t
{
  kApplied,
  kRetryScheduled,
  kTimedOut,
  kDiscarded,
};

[[nodiscard]] std::optional<OperationGeneration> next_startup_attempt(
  OperationGeneration previous) noexcept;

[[nodiscard]] StartupAuthorityClassification classify_startup_authority(
  const restocker_world_state::WorldStateSnapshot & snapshot);

class CoordinatorStartupGate
{
public:
  explicit CoordinatorStartupGate(CoordinatorStartupConfig config = {});

  [[nodiscard]] StartupProbeAction poll(SteadyTime now, bool snapshot_service_available);
  [[nodiscard]] bool submission_failed(
    OperationGeneration attempt_generation, SteadyTime now, std::string detail);
  [[nodiscard]] StartupCompletionDisposition complete(
    OperationGeneration attempt_generation, SteadyTime arrived_at, SteadyTime handled_at,
    StartupAuthorityClassification classification);
  [[nodiscard]] StartupCompletionDisposition transport_failed(
    OperationGeneration attempt_generation, SteadyTime arrived_at, SteadyTime handled_at,
    std::string detail);
  [[nodiscard]] std::optional<OperationGeneration> retire_for_shutdown();
  [[nodiscard]] bool latch_fault(std::string detail);

  [[nodiscard]] CoordinatorStartupSnapshot snapshot() const;
  [[nodiscard]] std::optional<SteadyTime> active_deadline() const noexcept;

private:
  [[nodiscard]] bool schedule_retry(SteadyTime now, std::string detail);

  CoordinatorStartupConfig config_;
  CoordinatorStartupSnapshot snapshot_;
  std::optional<SteadyTime> deadline_;
  std::optional<SteadyTime> retry_not_before_;
};

[[nodiscard]] const char * to_string(CoordinatorStartupStatus status) noexcept;

[[nodiscard]] constexpr bool effective_admission_ready(
  const GoalAdmissionSnapshot & admission, bool shutdown_started) noexcept
{
  return admission.ready && !admission.inhibited && !shutdown_started;
}

}  // namespace restocker_task_executor
