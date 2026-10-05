// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_startup.hpp"

#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] std::optional<SteadyTime> checked_deadline(
  SteadyTime now, std::chrono::milliseconds duration) noexcept
{
  const auto converted = std::chrono::duration_cast<SteadyTime::duration>(duration);
  if (converted <= SteadyTime::duration::zero() || converted > SteadyTime::max() - now) {
    return std::nullopt;
  }
  return now + converted;
}

[[nodiscard]] bool terminal(CoordinatorStartupStatus status) noexcept
{
  return status == CoordinatorStartupStatus::kReady ||
         status == CoordinatorStartupStatus::kOrphanedReservation ||
         status == CoordinatorStartupStatus::kFaulted;
}

}  // namespace

std::optional<OperationGeneration> next_startup_attempt(OperationGeneration previous) noexcept
{
  if (previous == std::numeric_limits<OperationGeneration>::max()) {
    return std::nullopt;
  }
  return previous + 1U;
}

StartupAuthorityClassification classify_startup_authority(
  const restocker_world_state::WorldStateSnapshot & snapshot)
{
  using restocker_world_state::FaultState;
  using restocker_world_state::TaskPhase;

  if (snapshot.active_reservation) {
    if (!restocker_world_state::valid_active_task_semantics(
        snapshot.robot.task_phase, snapshot.robot.fault_state))
    {
      return {
        CoordinatorStartupStatus::kFaulted, std::nullopt,
        "active reservation has inconsistent robot task and fault semantics"};
    }
    return {
      CoordinatorStartupStatus::kOrphanedReservation, snapshot.active_reservation,
      "authoritative world state contains a reservation not owned by this process"};
  }

  if (snapshot.robot.task_phase != TaskPhase::Idle ||
    snapshot.robot.fault_state != FaultState::None || snapshot.robot.held_object)
  {
    return {
      CoordinatorStartupStatus::kFaulted, std::nullopt,
      "unreserved world state is not idle, fault-free, and empty-handed"};
  }

  return {
    CoordinatorStartupStatus::kReady, std::nullopt,
    "authoritative world state is clean for coordinator startup"};
}

CoordinatorStartupGate::CoordinatorStartupGate(CoordinatorStartupConfig config)
: config_(config)
{
  const auto maximum_duration =
    std::chrono::duration_cast<std::chrono::milliseconds>(SteadyTime::duration::max());
  if (config_.snapshot_timeout <= std::chrono::milliseconds::zero() ||
    config_.retry_period <= std::chrono::milliseconds::zero() ||
    config_.snapshot_timeout > maximum_duration || config_.retry_period > maximum_duration)
  {
    throw std::invalid_argument("startup timeout and retry period are outside the supported range");
  }
}

StartupProbeAction CoordinatorStartupGate::poll(
  SteadyTime now, bool snapshot_service_available)
{
  if (terminal(snapshot_.status)) {
    return {};
  }

  if (snapshot_.status == CoordinatorStartupStatus::kProbePending) {
    if (deadline_ && now >= *deadline_) {
      const auto attempt = snapshot_.attempt_generation;
      if (!schedule_retry(now, "authoritative startup snapshot request timed out")) {
        return {};
      }
      return {StartupProbeActionKind::kRetireTimedOut, attempt, {}};
    }
    return {};
  }

  if (!snapshot_service_available || (retry_not_before_ && now < *retry_not_before_)) {
    return {};
  }

  const auto attempt = next_startup_attempt(snapshot_.attempt_generation);
  const auto deadline = checked_deadline(now, config_.snapshot_timeout);
  if (!attempt || !deadline) {
    (void)latch_fault(
      !attempt ? "startup attempt generation exhausted" :
      "startup snapshot deadline overflowed");
    return {};
  }

  snapshot_.status = CoordinatorStartupStatus::kProbePending;
  snapshot_.attempt_generation = *attempt;
  snapshot_.orphan.reset();
  snapshot_.detail = "authoritative startup snapshot request pending";
  deadline_ = *deadline;
  retry_not_before_.reset();
  return {StartupProbeActionKind::kSubmit, *attempt, *deadline};
}

bool CoordinatorStartupGate::submission_failed(
  OperationGeneration attempt_generation, SteadyTime now, std::string detail)
{
  if (snapshot_.status != CoordinatorStartupStatus::kProbePending ||
    snapshot_.attempt_generation != attempt_generation)
  {
    return false;
  }
  if (detail.empty()) {
    detail = "authoritative startup snapshot submission failed";
  }
  return schedule_retry(now, std::move(detail));
}

StartupCompletionDisposition CoordinatorStartupGate::complete(
  OperationGeneration attempt_generation, SteadyTime arrived_at, SteadyTime handled_at,
  StartupAuthorityClassification classification)
{
  if (snapshot_.status != CoordinatorStartupStatus::kProbePending || !deadline_ ||
    snapshot_.attempt_generation != attempt_generation)
  {
    return StartupCompletionDisposition::kDiscarded;
  }
  if (arrived_at >= *deadline_) {
    (void)schedule_retry(handled_at, "authoritative startup snapshot response arrived too late");
    return StartupCompletionDisposition::kTimedOut;
  }
  if (classification.status != CoordinatorStartupStatus::kReady &&
    classification.status != CoordinatorStartupStatus::kOrphanedReservation &&
    classification.status != CoordinatorStartupStatus::kFaulted)
  {
    (void)latch_fault("startup classifier returned a nonterminal state");
    return StartupCompletionDisposition::kApplied;
  }
  if ((classification.status == CoordinatorStartupStatus::kOrphanedReservation) !=
    classification.orphan.has_value())
  {
    (void)latch_fault("startup classifier returned contradictory orphan identity");
    return StartupCompletionDisposition::kApplied;
  }

  snapshot_.status = classification.status;
  snapshot_.orphan = std::move(classification.orphan);
  snapshot_.detail = classification.detail.empty() ?
    "startup authority classification supplied no detail" : std::move(classification.detail);
  deadline_.reset();
  retry_not_before_.reset();
  return StartupCompletionDisposition::kApplied;
}

StartupCompletionDisposition CoordinatorStartupGate::transport_failed(
  OperationGeneration attempt_generation, SteadyTime arrived_at, SteadyTime handled_at,
  std::string detail)
{
  if (snapshot_.status != CoordinatorStartupStatus::kProbePending || !deadline_ ||
    snapshot_.attempt_generation != attempt_generation)
  {
    return StartupCompletionDisposition::kDiscarded;
  }
  if (arrived_at >= *deadline_) {
    (void)schedule_retry(handled_at, "authoritative startup snapshot response arrived too late");
    return StartupCompletionDisposition::kTimedOut;
  }
  if (detail.empty()) {
    detail = "authoritative startup snapshot transport failed";
  }
  if (!schedule_retry(handled_at, std::move(detail))) {
    return StartupCompletionDisposition::kApplied;
  }
  return StartupCompletionDisposition::kRetryScheduled;
}

std::optional<OperationGeneration> CoordinatorStartupGate::retire_for_shutdown()
{
  if (snapshot_.status != CoordinatorStartupStatus::kProbePending) {
    return std::nullopt;
  }
  const auto attempt = snapshot_.attempt_generation;
  snapshot_.status = CoordinatorStartupStatus::kWaitingForAuthority;
  snapshot_.detail = "startup authority probe retired for shutdown";
  deadline_.reset();
  retry_not_before_.reset();
  return attempt;
}

bool CoordinatorStartupGate::latch_fault(std::string detail)
{
  if (terminal(snapshot_.status)) {
    return false;
  }
  snapshot_.status = CoordinatorStartupStatus::kFaulted;
  snapshot_.orphan.reset();
  snapshot_.detail = detail.empty() ? "coordinator startup faulted" : std::move(detail);
  deadline_.reset();
  retry_not_before_.reset();
  return true;
}

CoordinatorStartupSnapshot CoordinatorStartupGate::snapshot() const
{
  return snapshot_;
}

std::optional<SteadyTime> CoordinatorStartupGate::active_deadline() const noexcept
{
  return deadline_;
}

bool CoordinatorStartupGate::schedule_retry(SteadyTime now, std::string detail)
{
  const auto retry = checked_deadline(now, config_.retry_period);
  if (!retry) {
    (void)latch_fault("startup retry deadline overflowed");
    return false;
  }
  snapshot_.status = CoordinatorStartupStatus::kWaitingForAuthority;
  snapshot_.orphan.reset();
  snapshot_.detail = std::move(detail);
  deadline_.reset();
  retry_not_before_ = *retry;
  return true;
}

const char * to_string(CoordinatorStartupStatus status) noexcept
{
  switch (status) {
    case CoordinatorStartupStatus::kWaitingForAuthority: return "waiting_for_authority";
    case CoordinatorStartupStatus::kProbePending: return "probe_pending";
    case CoordinatorStartupStatus::kReady: return "ready";
    case CoordinatorStartupStatus::kOrphanedReservation: return "orphaned_reservation";
    case CoordinatorStartupStatus::kFaulted: return "faulted";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
