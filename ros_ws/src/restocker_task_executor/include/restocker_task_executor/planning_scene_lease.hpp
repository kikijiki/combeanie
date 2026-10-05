// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace restocker_task_executor
{

enum class ProjectorStage : std::uint8_t
{
  Idle,
  Snapshot,
  CurrentScene,
  ApplyScene,
  VerifyScene,
};

enum class PlanningSceneLeasePhase : std::uint8_t
{
  None,
  Draining,
  Held,
  Releasing,
};

enum class PlanningSceneLeaseCode : std::uint16_t
{
  Unset = 0,
  Draining = 1,
  Granted = 2,
  Valid = 3,
  ReleaseAccepted = 4,
  InvalidArgument = 5,
  Conflict = 6,
  TokenMismatch = 7,
  IdempotencyConflict = 8,
  ResourceExhausted = 9,
  InternalError = 255,
};

enum class AcquisitionAction : std::uint8_t
{
  None,
  InvalidateReadOnlyGeneration,
  DrainSideEffectingGeneration,
  StartReconciliation,
};

enum class VerificationTransition : std::uint8_t
{
  None,
  LeaseGranted,
  ReleaseCompleted,
  RejectedWhileHeld,
};

struct PlanningSceneLeaseSummary
{
  std::uint64_t lease_id{0};
  std::string acquisition_operation_id;
  std::uint64_t minimum_applied_revision{0};
  std::uint64_t granted_applied_revision{0};
  std::uint64_t verification_epoch{0};
  std::int64_t acquired_at_ns{0};
  PlanningSceneLeasePhase phase{PlanningSceneLeasePhase::None};

  bool operator==(const PlanningSceneLeaseSummary &) const = default;
};

struct AcquirePlanningSceneLeaseRequest
{
  std::string operation_id;
  std::uint64_t minimum_applied_revision{0};

  bool operator==(const AcquirePlanningSceneLeaseRequest &) const = default;
};

struct ReleasePlanningSceneLeaseRequest
{
  std::string operation_id;
  std::string token;
  std::uint64_t required_semantic_revision{0};

  bool operator==(const ReleasePlanningSceneLeaseRequest &) const = default;
};

struct PlanningSceneLeaseReply
{
  PlanningSceneLeaseCode code{PlanningSceneLeaseCode::Unset};
  std::string detail;
  std::optional<PlanningSceneLeaseSummary> lease;
  std::string token;
  AcquisitionAction action{AcquisitionAction::None};
};

struct PlanningSceneLeaseProtocolState
{
  std::string projector_epoch;
  PlanningSceneLeasePhase phase{PlanningSceneLeasePhase::None};
  std::optional<PlanningSceneLeaseSummary> lease;
  std::uint64_t verified_applied_revision{0};
  std::uint64_t verification_epoch{0};
  std::uint64_t scene_content_generation{0};
  bool has_verified_scene{false};
  bool proof_dirty{false};
  bool scene_diff_awaiting_verification{false};
  std::uint64_t required_release_revision{0};
  bool reconciliation_allowed{true};
};

struct PlanningSceneLeaseConfig
{
  std::size_t operation_journal_capacity{4096};
  std::string projector_epoch;
  std::uint64_t initial_scene_content_generation{0};
};

// Read-only description of one journal's retention bound; counts and identifiers only, never an
// operation ID, token or payload. `open_obligations` counts journal RECORDS only, and
// `open_obligations + terminal_receipts == size`. An active lease that has no record yet
// (Draining: the acquire entry is journalled at grant) is reported by `active_lease_phase` and by
// `reserved_credits`, so open_obligations == 0 must never be read as "nothing owed".
// `reserved_credits` (entries pre-reserved for the active lease) are not part of `size`.
// `epoch_id` is the projector epoch. `inhibited` means "no verified, clean scene proof exists":
// it is true on a freshly constructed, idle lease and is NOT a fault signal. The lease journal
// never evicts: when full it refuses new acquisition.
struct JournalRetentionSnapshot
{
  std::string journal;
  std::string epoch_id;
  std::size_t capacity{0};
  std::size_t size{0};
  std::size_t open_obligations{0};
  std::size_t terminal_receipts{0};
  std::size_t reserved_credits{0};
  PlanningSceneLeasePhase active_lease_phase{PlanningSceneLeasePhase::None};
  bool inhibited{false};
  bool evicting{false};
};

class PlanningSceneLeaseProtocol
{
public:
  using TokenFactory = std::function<std::optional<std::string>()>;

  explicit PlanningSceneLeaseProtocol(
    PlanningSceneLeaseConfig config = {},
    TokenFactory token_factory = {});
  ~PlanningSceneLeaseProtocol();

  PlanningSceneLeaseProtocol(const PlanningSceneLeaseProtocol &) = delete;
  PlanningSceneLeaseProtocol & operator=(const PlanningSceneLeaseProtocol &) = delete;
  PlanningSceneLeaseProtocol(PlanningSceneLeaseProtocol &&) noexcept;
  PlanningSceneLeaseProtocol & operator=(PlanningSceneLeaseProtocol &&) noexcept;

  [[nodiscard]] PlanningSceneLeaseReply acquire(
    const AcquirePlanningSceneLeaseRequest & request,
    ProjectorStage stage, std::int64_t now_ns);

  [[nodiscard]] PlanningSceneLeaseReply validate(const std::string & token) const;

  [[nodiscard]] PlanningSceneLeaseReply release(const ReleasePlanningSceneLeaseRequest & request);

  [[nodiscard]] VerificationTransition record_verification(
    std::uint64_t applied_revision,
    std::int64_t verified_at_ns);

  void record_scene_diff_submission();
  void record_side_effect_unknown();

  [[nodiscard]] PlanningSceneLeaseProtocolState state() const;

  // Observability only: no admission or replay decision reads it.
  [[nodiscard]] JournalRetentionSnapshot retention_snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

[[nodiscard]] std::string to_string(PlanningSceneLeasePhase phase);
[[nodiscard]] std::string to_string(PlanningSceneLeaseCode code);

}  // namespace restocker_task_executor
