// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <type_traits>
#include <vector>

#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"
#include "restocker_task_executor/coordinator_inbox_deposit_result.hpp"

namespace restocker_task_executor
{

using GenerationDepositAttempt = std::uint64_t;
using GenerationQuiescenceCookie = std::uint64_t;

enum class GenerationRetirementFenceState : std::uint8_t
{
  kHealthy,
  kFailed,
  kRetirementCommitted,
};

enum class GenerationSynchronizationFailureStatus : std::uint8_t
{
  kMarkedFailed,
  kAlreadyFailed,
  kRetirementCommitted,
};

enum class GenerationDepositBeginStatus : std::uint8_t
{
  kStarted,
  kSealed,
  kSynchronizationFailed,
  kCapacityExhausted,
  kAttemptExhausted,
  kIdentityMismatch,
  kInvalidArgument,
};

enum class GenerationDepositCompletionStatus : std::uint8_t
{
  kCompleted,
  kCompletedUnresolved,
  kNotLive,
  kIdentityMismatch,
  kStaleOperation,
  kWrongPhase,
  kInvalidDisposition,
};

enum class GenerationQuiescenceSealStatus : std::uint8_t
{
  kSealed,
  kAlreadySealed,
  kSynchronizationFailed,
  kNotLive,
  kIdentityMismatch,
  kStaleWitness,
  kAckLineageExhausted,
  kInvalidArgument,
};

enum class GenerationQuiescenceProbeStatus : std::uint8_t
{
  kPrepared,
  kSynchronizationFailed,
  kNotSealed,
  kDepositsOutstanding,
  kDepositUnresolved,
  kProbeOutstanding,
  kAlreadyIssued,
};

enum class GenerationQuiescenceCompletionStatus : std::uint8_t
{
  kReceiptIssued,
  kSynchronizationFailed,
  kDriverLineageMismatch,
  kInboxNotEmpty,
  kNotLive,
  kIdentityMismatch,
  kStaleProbe,
  kWrongAckLineage,
};

enum class GenerationQuiescenceConsumeStatus : std::uint8_t
{
  kConsumed,
  kRetirementFenceFailed,
  kRetirementAlreadyCommitted,
  kNotLive,
  kIdentityMismatch,
  kStaleReceipt,
  kWrongAckLineage,
};

struct CoordinatorGenerationQuiescenceSnapshot
{
  CoordinatorGoalId goal_id{};
  GoalGeneration generation{0U};
  bool sealed{false};
  std::size_t active_deposit_count{0U};
  bool deposit_unresolved{false};
  bool adapter_fault{false};
  GenerationRetirementFenceState retirement_fence_state{
    GenerationRetirementFenceState::kHealthy};
  bool probe_outstanding{false};
  bool receipt_outstanding{false};
  AcceptedTerminalAckCookie accepted_ack_cookie{0U};
};

class CoordinatorGenerationQuiescence;

class CoordinatorDepositPermit final
{
public:
  CoordinatorDepositPermit() = delete;
  CoordinatorDepositPermit(const CoordinatorDepositPermit &) = delete;
  CoordinatorDepositPermit(CoordinatorDepositPermit &&) noexcept = default;
  CoordinatorDepositPermit & operator=(const CoordinatorDepositPermit &) = delete;
  CoordinatorDepositPermit & operator=(CoordinatorDepositPermit &&) noexcept = default;
  ~CoordinatorDepositPermit() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}
  [[nodiscard]] std::size_t slot() const noexcept {return slot_;}
  [[nodiscard]] GenerationDepositAttempt attempt() const noexcept {return attempt_;}
  [[nodiscard]] GenerationQuiescenceCookie cookie() const noexcept {return cookie_;}

private:
  friend class CoordinatorGenerationQuiescence;
  CoordinatorDepositPermit(
    CoordinatorGoalId goal_id, GoalGeneration generation, std::size_t slot,
    GenerationDepositAttempt attempt, GenerationQuiescenceCookie cookie) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  std::size_t slot_{0U};
  GenerationDepositAttempt attempt_{0U};
  GenerationQuiescenceCookie cookie_{0U};
  CoordinatorOneShot live_;
};

class CoordinatorGenerationProbePermit final
{
public:
  CoordinatorGenerationProbePermit() = delete;
  CoordinatorGenerationProbePermit(const CoordinatorGenerationProbePermit &) = delete;
  CoordinatorGenerationProbePermit(CoordinatorGenerationProbePermit &&) noexcept = default;
  CoordinatorGenerationProbePermit & operator=(const CoordinatorGenerationProbePermit &) = delete;
  CoordinatorGenerationProbePermit & operator=(
    CoordinatorGenerationProbePermit &&) noexcept = default;
  ~CoordinatorGenerationProbePermit() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}
  [[nodiscard]] GenerationQuiescenceCookie cookie() const noexcept {return cookie_;}

private:
  friend class CoordinatorGenerationQuiescence;
  CoordinatorGenerationProbePermit(
    CoordinatorGoalId goal_id, GoalGeneration generation, GenerationQuiescenceCookie cookie,
    AcceptedTerminalAckCookie accepted_ack_cookie) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  GenerationQuiescenceCookie cookie_{0U};
  AcceptedTerminalAckCookie accepted_ack_cookie_{0U};
  CoordinatorOneShot live_;
};

class CoordinatorGenerationQuiescenceReceipt final
{
public:
  CoordinatorGenerationQuiescenceReceipt() = delete;
  CoordinatorGenerationQuiescenceReceipt(
    const CoordinatorGenerationQuiescenceReceipt &) = delete;
  CoordinatorGenerationQuiescenceReceipt(
    CoordinatorGenerationQuiescenceReceipt &&) noexcept = default;
  CoordinatorGenerationQuiescenceReceipt & operator=(
    const CoordinatorGenerationQuiescenceReceipt &) = delete;
  CoordinatorGenerationQuiescenceReceipt & operator=(
    CoordinatorGenerationQuiescenceReceipt &&) noexcept = default;
  ~CoordinatorGenerationQuiescenceReceipt() = default;

  [[nodiscard]] bool live() const noexcept {return live_.live();}
  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept {return goal_id_;}
  [[nodiscard]] GoalGeneration goal_generation() const noexcept {return generation_;}
  [[nodiscard]] GenerationQuiescenceCookie cookie() const noexcept {return cookie_;}

private:
  friend class CoordinatorGenerationQuiescence;
  CoordinatorGenerationQuiescenceReceipt(
    CoordinatorGoalId goal_id, GoalGeneration generation, GenerationQuiescenceCookie cookie,
    AcceptedTerminalAckCookie accepted_ack_cookie) noexcept;

  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  GenerationQuiescenceCookie cookie_{0U};
  AcceptedTerminalAckCookie accepted_ack_cookie_{0U};
  CoordinatorOneShot live_;
};

class GenerationDepositBeginDecision final
{
public:
  GenerationDepositBeginDecision() = delete;
  GenerationDepositBeginDecision(const GenerationDepositBeginDecision &) = delete;
  GenerationDepositBeginDecision(GenerationDepositBeginDecision && other) noexcept;
  GenerationDepositBeginDecision & operator=(const GenerationDepositBeginDecision &) = delete;
  GenerationDepositBeginDecision & operator=(GenerationDepositBeginDecision && other) noexcept;
  ~GenerationDepositBeginDecision() = default;

  [[nodiscard]] GenerationDepositBeginStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorDepositPermit> take_deposit_permit() noexcept;

private:
  friend class CoordinatorGenerationQuiescence;
  explicit GenerationDepositBeginDecision(GenerationDepositBeginStatus status) noexcept;
  explicit GenerationDepositBeginDecision(CoordinatorDepositPermit && permit) noexcept;

  GenerationDepositBeginStatus status_;
  std::optional<CoordinatorDepositPermit> permit_;
};

class CoordinatorGenerationProbeDecision final
{
public:
  CoordinatorGenerationProbeDecision() = delete;
  CoordinatorGenerationProbeDecision(const CoordinatorGenerationProbeDecision &) = delete;
  CoordinatorGenerationProbeDecision(CoordinatorGenerationProbeDecision && other) noexcept;
  CoordinatorGenerationProbeDecision & operator=(const CoordinatorGenerationProbeDecision &) =
  delete;
  CoordinatorGenerationProbeDecision & operator=(
    CoordinatorGenerationProbeDecision && other) noexcept;
  ~CoordinatorGenerationProbeDecision() = default;

  [[nodiscard]] GenerationQuiescenceProbeStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorGenerationProbePermit> take_probe_permit() noexcept;

private:
  friend class CoordinatorGenerationQuiescence;
  explicit CoordinatorGenerationProbeDecision(GenerationQuiescenceProbeStatus status) noexcept;
  explicit CoordinatorGenerationProbeDecision(CoordinatorGenerationProbePermit && permit) noexcept;

  GenerationQuiescenceProbeStatus status_;
  std::optional<CoordinatorGenerationProbePermit> permit_;
};

class CoordinatorGenerationCompletionDecision final
{
public:
  CoordinatorGenerationCompletionDecision() = delete;
  CoordinatorGenerationCompletionDecision(
    const CoordinatorGenerationCompletionDecision &) = delete;
  CoordinatorGenerationCompletionDecision(
    CoordinatorGenerationCompletionDecision && other) noexcept;
  CoordinatorGenerationCompletionDecision & operator=(
    const CoordinatorGenerationCompletionDecision &) = delete;
  CoordinatorGenerationCompletionDecision & operator=(
    CoordinatorGenerationCompletionDecision && other) noexcept;
  ~CoordinatorGenerationCompletionDecision() = default;

  [[nodiscard]] GenerationQuiescenceCompletionStatus status() const noexcept {return status_;}
  [[nodiscard]] std::optional<CoordinatorGenerationQuiescenceReceipt> take_receipt() noexcept;

private:
  friend class CoordinatorGenerationQuiescence;
  explicit CoordinatorGenerationCompletionDecision(
    GenerationQuiescenceCompletionStatus status) noexcept;
  explicit CoordinatorGenerationCompletionDecision(
    CoordinatorGenerationQuiescenceReceipt && receipt) noexcept;

  GenerationQuiescenceCompletionStatus status_;
  std::optional<CoordinatorGenerationQuiescenceReceipt> receipt_;
};

class CoordinatorGenerationQuiescence final
{
public:
  CoordinatorGenerationQuiescence(
    CoordinatorGoalId goal_id, GoalGeneration generation, std::size_t deposit_capacity);
  CoordinatorGenerationQuiescence(const CoordinatorGenerationQuiescence &) = delete;
  CoordinatorGenerationQuiescence(CoordinatorGenerationQuiescence &&) = delete;
  CoordinatorGenerationQuiescence & operator=(const CoordinatorGenerationQuiescence &) = delete;
  CoordinatorGenerationQuiescence & operator=(CoordinatorGenerationQuiescence &&) = delete;
  ~CoordinatorGenerationQuiescence() = default;

  [[nodiscard]] CoordinatorGenerationQuiescenceSnapshot snapshot() const;
  [[nodiscard]] GenerationSynchronizationFailureStatus mark_synchronization_failure() noexcept;
  [[nodiscard]] GenerationDepositBeginDecision begin_deposit(
    const CoordinatorGoalId & goal_id, GoalGeneration generation);
  [[nodiscard]] GenerationDepositCompletionStatus complete_deposit(
    CoordinatorDepositPermit & permit, GenerationDepositCompletion disposition);
  [[nodiscard]] GenerationQuiescenceSealStatus seal_after_accepted_ack(
    CoordinatorAcceptedTerminalAckWitness & witness);
  [[nodiscard]] CoordinatorGenerationProbeDecision prepare_probe();
  [[nodiscard]] CoordinatorGenerationCompletionDecision complete_probe(
    CoordinatorGenerationProbePermit & permit,
    std::optional<GoalGeneration> inactive_after_ack_generation,
    bool inbox_generation_empty);
  [[nodiscard]] GenerationQuiescenceConsumeStatus consume_receipt(
    CoordinatorGenerationQuiescenceReceipt & receipt,
    const CoordinatorGoalId & goal_id, GoalGeneration generation);

private:
  struct DepositSlot
  {
    GenerationDepositAttempt attempt{0U};
    GenerationQuiescenceCookie cookie{0U};
    bool active{false};
  };

  void mark_adapter_fault_locked() noexcept;
  [[nodiscard]] bool identity_matches(
    const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept;

  mutable std::mutex mutex_;
  CoordinatorGoalId goal_id_{};
  GoalGeneration generation_{0U};
  std::vector<DepositSlot> slots_;
  std::atomic<GenerationRetirementFenceState> retirement_fence_state_{
    GenerationRetirementFenceState::kHealthy};
  std::size_t active_deposit_count_{0U};
  AcceptedTerminalAckCookie accepted_ack_lineage_{0U};
  AcceptedTerminalAckCookie accepted_ack_cookie_{0U};
  GenerationQuiescenceCookie last_probe_cookie_{0U};
  GenerationQuiescenceCookie outstanding_probe_cookie_{0U};
  GenerationQuiescenceCookie outstanding_receipt_cookie_{0U};
  bool sealed_{false};
  bool deposit_unresolved_{false};
  bool adapter_fault_{false};
  bool probe_outstanding_{false};
  bool receipt_outstanding_{false};
};

static_assert(std::is_trivially_copyable_v<CoordinatorGenerationQuiescenceSnapshot>);

}  // namespace restocker_task_executor
