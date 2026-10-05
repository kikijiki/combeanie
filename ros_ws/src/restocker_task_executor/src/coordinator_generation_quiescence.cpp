// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"

#include <algorithm>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] constexpr std::optional<GenerationDepositAttempt>
checked_next_generation_deposit_attempt(GenerationDepositAttempt current) noexcept
{
  if (current == std::numeric_limits<GenerationDepositAttempt>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] constexpr std::optional<GenerationQuiescenceCookie>
checked_next_generation_quiescence_cookie(GenerationQuiescenceCookie current) noexcept
{
  if (current == std::numeric_limits<GenerationQuiescenceCookie>::max()) {
    return std::nullopt;
  }
  return current + 1U;
}

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id) noexcept
{
  return std::any_of(
    goal_id.begin(), goal_id.end(), [](std::uint8_t value) {return value != 0U;});
}

[[nodiscard]] bool valid_deposit_completion(GenerationDepositCompletion completion) noexcept
{
  switch (completion) {
    case GenerationDepositCompletion::kAccepted:
    case GenerationDepositCompletion::kOverflowAccounted:
    case GenerationDepositCompletion::kInhibitedAccounted:
    case GenerationDepositCompletion::kEvidenceLostAccounted:
    case GenerationDepositCompletion::kUnknown:
      return true;
  }
  return false;
}

template<typename Capability>
[[nodiscard]] std::optional<Capability> take_optional(
  std::optional<Capability> & source) noexcept
{
  std::optional<Capability> result;
  if (source) {
    result.emplace(std::move(*source));
    source.reset();
  }
  return result;
}

}  // namespace

CoordinatorDepositPermit::CoordinatorDepositPermit(
  CoordinatorGoalId goal_id, GoalGeneration generation, std::size_t slot,
  GenerationDepositAttempt attempt, GenerationQuiescenceCookie cookie) noexcept
: goal_id_(goal_id),
  generation_(generation), slot_(slot), attempt_(attempt), cookie_(cookie), live_(true)
{
}

CoordinatorGenerationProbePermit::CoordinatorGenerationProbePermit(
  CoordinatorGoalId goal_id, GoalGeneration generation, GenerationQuiescenceCookie cookie,
  AcceptedTerminalAckCookie accepted_ack_cookie) noexcept
: goal_id_(goal_id),
  generation_(generation), cookie_(cookie), accepted_ack_cookie_(accepted_ack_cookie), live_(true)
{
}

CoordinatorGenerationQuiescenceReceipt::CoordinatorGenerationQuiescenceReceipt(
  CoordinatorGoalId goal_id, GoalGeneration generation, GenerationQuiescenceCookie cookie,
  AcceptedTerminalAckCookie accepted_ack_cookie) noexcept
: goal_id_(goal_id),
  generation_(generation), cookie_(cookie), accepted_ack_cookie_(accepted_ack_cookie), live_(true)
{
}

GenerationDepositBeginDecision::GenerationDepositBeginDecision(
  GenerationDepositBeginStatus status) noexcept
: status_(status)
{
}

GenerationDepositBeginDecision::GenerationDepositBeginDecision(
  CoordinatorDepositPermit && permit) noexcept
: status_(GenerationDepositBeginStatus::kStarted), permit_(std::move(permit))
{
}

GenerationDepositBeginDecision::GenerationDepositBeginDecision(
  GenerationDepositBeginDecision && other) noexcept
: status_(other.status_), permit_(take_optional(other.permit_))
{
}

GenerationDepositBeginDecision & GenerationDepositBeginDecision::operator=(
  GenerationDepositBeginDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    permit_ = take_optional(other.permit_);
  }
  return *this;
}

std::optional<CoordinatorDepositPermit>
GenerationDepositBeginDecision::take_deposit_permit() noexcept
{
  return take_optional(permit_);
}

CoordinatorGenerationProbeDecision::CoordinatorGenerationProbeDecision(
  GenerationQuiescenceProbeStatus status) noexcept
: status_(status)
{
}

CoordinatorGenerationProbeDecision::CoordinatorGenerationProbeDecision(
  CoordinatorGenerationProbePermit && permit) noexcept
: status_(GenerationQuiescenceProbeStatus::kPrepared), permit_(std::move(permit))
{
}

CoordinatorGenerationProbeDecision::CoordinatorGenerationProbeDecision(
  CoordinatorGenerationProbeDecision && other) noexcept
: status_(other.status_), permit_(take_optional(other.permit_))
{
}

CoordinatorGenerationProbeDecision & CoordinatorGenerationProbeDecision::operator=(
  CoordinatorGenerationProbeDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    permit_ = take_optional(other.permit_);
  }
  return *this;
}

std::optional<CoordinatorGenerationProbePermit>
CoordinatorGenerationProbeDecision::take_probe_permit() noexcept
{
  return take_optional(permit_);
}

CoordinatorGenerationCompletionDecision::CoordinatorGenerationCompletionDecision(
  GenerationQuiescenceCompletionStatus status) noexcept
: status_(status)
{
}

CoordinatorGenerationCompletionDecision::CoordinatorGenerationCompletionDecision(
  CoordinatorGenerationQuiescenceReceipt && receipt) noexcept
: status_(GenerationQuiescenceCompletionStatus::kReceiptIssued),
  receipt_(std::move(receipt))
{
}

CoordinatorGenerationCompletionDecision::CoordinatorGenerationCompletionDecision(
  CoordinatorGenerationCompletionDecision && other) noexcept
: status_(other.status_), receipt_(take_optional(other.receipt_))
{
}

CoordinatorGenerationCompletionDecision & CoordinatorGenerationCompletionDecision::operator=(
  CoordinatorGenerationCompletionDecision && other) noexcept
{
  if (this != &other) {
    status_ = other.status_;
    receipt_ = take_optional(other.receipt_);
  }
  return *this;
}

std::optional<CoordinatorGenerationQuiescenceReceipt>
CoordinatorGenerationCompletionDecision::take_receipt() noexcept
{
  return take_optional(receipt_);
}

CoordinatorGenerationQuiescence::CoordinatorGenerationQuiescence(
  CoordinatorGoalId goal_id, GoalGeneration generation, std::size_t deposit_capacity)
: goal_id_(goal_id), generation_(generation)
{
  if (deposit_capacity == 0U) {
    throw std::invalid_argument("generation quiescence requires positive deposit capacity");
  }
  if (!valid_goal_id(goal_id_) || !admissible_goal_generation(generation_)) {
    throw std::invalid_argument("generation quiescence requires an admissible goal identity");
  }
  slots_.resize(deposit_capacity);
}

bool CoordinatorGenerationQuiescence::identity_matches(
  const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept
{
  return goal_id == goal_id_ && generation == generation_;
}

void CoordinatorGenerationQuiescence::mark_adapter_fault_locked() noexcept
{
  adapter_fault_ = true;
  static_cast<void>(mark_synchronization_failure());
}

CoordinatorGenerationQuiescenceSnapshot CoordinatorGenerationQuiescence::snapshot() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return {
    goal_id_, generation_, sealed_, active_deposit_count_, deposit_unresolved_, adapter_fault_,
    retirement_fence_state_.load(std::memory_order_acquire), probe_outstanding_,
    receipt_outstanding_, accepted_ack_cookie_};
}

GenerationSynchronizationFailureStatus
CoordinatorGenerationQuiescence::mark_synchronization_failure() noexcept
{
  auto expected = GenerationRetirementFenceState::kHealthy;
  if (retirement_fence_state_.compare_exchange_strong(
      expected, GenerationRetirementFenceState::kFailed,
      std::memory_order_acq_rel, std::memory_order_acquire))
  {
    return GenerationSynchronizationFailureStatus::kMarkedFailed;
  }
  if (expected == GenerationRetirementFenceState::kFailed) {
    return GenerationSynchronizationFailureStatus::kAlreadyFailed;
  }
  return GenerationSynchronizationFailureStatus::kRetirementCommitted;
}

GenerationDepositBeginDecision CoordinatorGenerationQuiescence::begin_deposit(
  const CoordinatorGoalId & goal_id, GoalGeneration generation)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation)) {
    mark_adapter_fault_locked();
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kInvalidArgument};
  }
  if (!identity_matches(goal_id, generation)) {
    mark_adapter_fault_locked();
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kIdentityMismatch};
  }
  const auto fence = retirement_fence_state_.load(std::memory_order_acquire);
  if (fence == GenerationRetirementFenceState::kFailed) {
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kSynchronizationFailed};
  }
  if (fence == GenerationRetirementFenceState::kRetirementCommitted || sealed_) {
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kSealed};
  }
  const auto slot_iterator = std::find_if(
    slots_.begin(), slots_.end(), [](const DepositSlot & slot) {return !slot.active;});
  if (slot_iterator == slots_.end()) {
    deposit_unresolved_ = true;
    mark_adapter_fault_locked();
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kCapacityExhausted};
  }
  const auto next_attempt = checked_next_generation_deposit_attempt(slot_iterator->attempt);
  const auto next_cookie = checked_next_generation_deposit_attempt(slot_iterator->cookie);
  if (!next_attempt || !next_cookie || *next_attempt == 0U || *next_cookie == 0U) {
    deposit_unresolved_ = true;
    mark_adapter_fault_locked();
    return GenerationDepositBeginDecision{GenerationDepositBeginStatus::kAttemptExhausted};
  }
  slot_iterator->attempt = *next_attempt;
  slot_iterator->cookie = *next_cookie;
  slot_iterator->active = true;
  ++active_deposit_count_;
  const auto slot = static_cast<std::size_t>(std::distance(slots_.begin(), slot_iterator));
  return GenerationDepositBeginDecision{
    CoordinatorDepositPermit{
      goal_id_, generation_, slot,
      slot_iterator->attempt, slot_iterator->cookie}};
}

GenerationDepositCompletionStatus CoordinatorGenerationQuiescence::complete_deposit(
  CoordinatorDepositPermit & permit, GenerationDepositCompletion disposition)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!permit.live_.live()) {
    return GenerationDepositCompletionStatus::kNotLive;
  }
  if (!valid_goal_id(permit.goal_id_) || !admissible_goal_generation(permit.generation_) ||
    !identity_matches(permit.goal_id_, permit.generation_))
  {
    mark_adapter_fault_locked();
    return GenerationDepositCompletionStatus::kIdentityMismatch;
  }
  if (permit.slot_ >= slots_.size()) {
    return GenerationDepositCompletionStatus::kStaleOperation;
  }
  auto & slot = slots_[permit.slot_];
  if (permit.attempt_ != slot.attempt || permit.cookie_ != slot.cookie) {
    return GenerationDepositCompletionStatus::kStaleOperation;
  }
  if (!slot.active) {
    return GenerationDepositCompletionStatus::kWrongPhase;
  }
  if (!valid_deposit_completion(disposition)) {
    return GenerationDepositCompletionStatus::kInvalidDisposition;
  }
  slot.active = false;
  --active_deposit_count_;
  permit.live_.consume();
  if (disposition == GenerationDepositCompletion::kUnknown) {
    deposit_unresolved_ = true;
    return GenerationDepositCompletionStatus::kCompletedUnresolved;
  }
  return GenerationDepositCompletionStatus::kCompleted;
}

GenerationQuiescenceSealStatus CoordinatorGenerationQuiescence::seal_after_accepted_ack(
  CoordinatorAcceptedTerminalAckWitness & witness)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!witness.live_.live()) {
    return GenerationQuiescenceSealStatus::kNotLive;
  }
  if (!valid_goal_id(witness.goal_id_) || !admissible_goal_generation(witness.generation_) ||
    witness.ack_cookie_ == 0U)
  {
    mark_adapter_fault_locked();
    return GenerationQuiescenceSealStatus::kInvalidArgument;
  }
  if (!identity_matches(witness.goal_id_, witness.generation_)) {
    mark_adapter_fault_locked();
    return GenerationQuiescenceSealStatus::kIdentityMismatch;
  }
  const auto fence = retirement_fence_state_.load(std::memory_order_acquire);
  if (fence == GenerationRetirementFenceState::kFailed) {
    return GenerationQuiescenceSealStatus::kSynchronizationFailed;
  }
  if (fence == GenerationRetirementFenceState::kRetirementCommitted) {
    if (!sealed_) {
      mark_adapter_fault_locked();
      return GenerationQuiescenceSealStatus::kSynchronizationFailed;
    }
    return witness.ack_cookie_ == accepted_ack_cookie_ ?
           GenerationQuiescenceSealStatus::kAlreadySealed :
           GenerationQuiescenceSealStatus::kStaleWitness;
  }
  if (sealed_) {
    return witness.ack_cookie_ == accepted_ack_cookie_ ?
           GenerationQuiescenceSealStatus::kAlreadySealed :
           GenerationQuiescenceSealStatus::kStaleWitness;
  }
  const auto next_cookie = checked_next_accepted_terminal_ack_cookie(accepted_ack_lineage_);
  if (!next_cookie || *next_cookie == 0U) {
    mark_adapter_fault_locked();
    return GenerationQuiescenceSealStatus::kAckLineageExhausted;
  }
  if (witness.ack_cookie_ != *next_cookie) {
    return GenerationQuiescenceSealStatus::kStaleWitness;
  }
  accepted_ack_lineage_ = *next_cookie;
  accepted_ack_cookie_ = *next_cookie;
  sealed_ = true;
  witness.live_.consume();
  return GenerationQuiescenceSealStatus::kSealed;
}

CoordinatorGenerationProbeDecision CoordinatorGenerationQuiescence::prepare_probe()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  const auto fence = retirement_fence_state_.load(std::memory_order_acquire);
  if (adapter_fault_ || fence == GenerationRetirementFenceState::kFailed) {
    return CoordinatorGenerationProbeDecision{
      GenerationQuiescenceProbeStatus::kSynchronizationFailed};
  }
  if (fence == GenerationRetirementFenceState::kRetirementCommitted) {
    return CoordinatorGenerationProbeDecision{GenerationQuiescenceProbeStatus::kAlreadyIssued};
  }
  if (!sealed_) {
    return CoordinatorGenerationProbeDecision{GenerationQuiescenceProbeStatus::kNotSealed};
  }
  if (active_deposit_count_ != 0U) {
    return CoordinatorGenerationProbeDecision{
      GenerationQuiescenceProbeStatus::kDepositsOutstanding};
  }
  if (deposit_unresolved_) {
    return CoordinatorGenerationProbeDecision{
      GenerationQuiescenceProbeStatus::kDepositUnresolved};
  }
  if (probe_outstanding_) {
    return CoordinatorGenerationProbeDecision{
      GenerationQuiescenceProbeStatus::kProbeOutstanding};
  }
  if (receipt_outstanding_) {
    return CoordinatorGenerationProbeDecision{GenerationQuiescenceProbeStatus::kAlreadyIssued};
  }
  const auto next_cookie = checked_next_generation_quiescence_cookie(last_probe_cookie_);
  if (!next_cookie || *next_cookie == 0U) {
    mark_adapter_fault_locked();
    return CoordinatorGenerationProbeDecision{
      GenerationQuiescenceProbeStatus::kSynchronizationFailed};
  }
  last_probe_cookie_ = *next_cookie;
  outstanding_probe_cookie_ = *next_cookie;
  probe_outstanding_ = true;
  return CoordinatorGenerationProbeDecision{
    CoordinatorGenerationProbePermit{
      goal_id_, generation_, outstanding_probe_cookie_,
      accepted_ack_cookie_}};
}

CoordinatorGenerationCompletionDecision CoordinatorGenerationQuiescence::complete_probe(
  CoordinatorGenerationProbePermit & permit,
  std::optional<GoalGeneration> inactive_after_ack_generation,
  bool inbox_generation_empty)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!permit.live_.live()) {
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kNotLive};
  }
  if (!valid_goal_id(permit.goal_id_) || !admissible_goal_generation(permit.generation_) ||
    !identity_matches(permit.goal_id_, permit.generation_))
  {
    mark_adapter_fault_locked();
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kIdentityMismatch};
  }
  const auto fence = retirement_fence_state_.load(std::memory_order_acquire);
  if (adapter_fault_ || fence != GenerationRetirementFenceState::kHealthy) {
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kSynchronizationFailed};
  }
  if (!probe_outstanding_ || permit.cookie_ != outstanding_probe_cookie_) {
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kStaleProbe};
  }
  if (permit.accepted_ack_cookie_ != accepted_ack_cookie_) {
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kWrongAckLineage};
  }
  if (!inactive_after_ack_generation || *inactive_after_ack_generation != generation_) {
    probe_outstanding_ = false;
    outstanding_probe_cookie_ = 0U;
    permit.live_.consume();
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kDriverLineageMismatch};
  }
  if (!inbox_generation_empty) {
    probe_outstanding_ = false;
    outstanding_probe_cookie_ = 0U;
    permit.live_.consume();
    return CoordinatorGenerationCompletionDecision{
      GenerationQuiescenceCompletionStatus::kInboxNotEmpty};
  }
  probe_outstanding_ = false;
  outstanding_probe_cookie_ = 0U;
  receipt_outstanding_ = true;
  outstanding_receipt_cookie_ = permit.cookie_;
  permit.live_.consume();
  return CoordinatorGenerationCompletionDecision{
    CoordinatorGenerationQuiescenceReceipt{
      goal_id_, generation_, outstanding_receipt_cookie_,
      accepted_ack_cookie_}};
}

GenerationQuiescenceConsumeStatus CoordinatorGenerationQuiescence::consume_receipt(
  CoordinatorGenerationQuiescenceReceipt & receipt,
  const CoordinatorGoalId & goal_id, GoalGeneration generation)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!receipt.live_.live()) {
    return GenerationQuiescenceConsumeStatus::kNotLive;
  }
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation) ||
    !valid_goal_id(receipt.goal_id_) || !admissible_goal_generation(receipt.generation_) ||
    !identity_matches(goal_id, generation) ||
    !identity_matches(receipt.goal_id_, receipt.generation_))
  {
    mark_adapter_fault_locked();
    return GenerationQuiescenceConsumeStatus::kIdentityMismatch;
  }
  if (!receipt_outstanding_ || receipt.cookie_ != outstanding_receipt_cookie_) {
    return GenerationQuiescenceConsumeStatus::kStaleReceipt;
  }
  if (receipt.accepted_ack_cookie_ != accepted_ack_cookie_) {
    return GenerationQuiescenceConsumeStatus::kWrongAckLineage;
  }
  auto expected = GenerationRetirementFenceState::kHealthy;
  if (!retirement_fence_state_.compare_exchange_strong(
      expected, GenerationRetirementFenceState::kRetirementCommitted,
      std::memory_order_acq_rel, std::memory_order_acquire))
  {
    if (expected == GenerationRetirementFenceState::kFailed) {
      return GenerationQuiescenceConsumeStatus::kRetirementFenceFailed;
    }
    return GenerationQuiescenceConsumeStatus::kRetirementAlreadyCommitted;
  }
  receipt_outstanding_ = false;
  outstanding_receipt_cookie_ = 0U;
  receipt.live_.consume();
  return GenerationQuiescenceConsumeStatus::kConsumed;
}

}  // namespace restocker_task_executor
