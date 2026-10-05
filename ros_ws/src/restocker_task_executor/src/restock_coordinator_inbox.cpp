// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_coordinator_inbox.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_operation_types.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr CoordinatorInboxDepositResult kAccepted{
  CoordinatorInboxDepositStatus::kAccepted,
  CoordinatorInboxPersistenceStatus::kEventOwned};
constexpr CoordinatorInboxDepositResult kConflict{
  CoordinatorInboxDepositStatus::kEvidenceConflict,
  CoordinatorInboxPersistenceStatus::kUnresolved};

[[nodiscard]] bool valid_operation_correlation(
  const OperationCorrelation & correlation) noexcept
{
  return admissible_goal_generation(correlation.goal_generation) &&
         correlation.operation_generation != 0U &&
         correlation.operation_generation <
         std::numeric_limits<OperationGeneration>::max();
}

[[nodiscard]] bool valid_reconciliation(
  const std::optional<CoordinatorReconciliationCorrelation> & reconciliation) noexcept
{
  if (!reconciliation) {
    return true;
  }
  return (reconciliation->kind == ReconciliationKind::kReplayMutation ||
         reconciliation->kind == ReconciliationKind::kReadback) &&
         reconciliation->attempt_number != 0U;
}

template<typename Completion>
[[nodiscard]] bool valid_completion(const Completion & completion) noexcept
{
  return valid_operation_correlation(completion.correlation) &&
         valid_reconciliation(completion.reconciliation);
}

// True for the deliveries that wrap a port completion, which nest their correlation one level
// deeper than the world-state service completions do.
template<typename Item>
inline constexpr bool kWrapsPortCompletion =
  std::is_same_v<Item, CoordinatorMotionCompletion> ||
  std::is_same_v<Item, CoordinatorGripperCompletion> ||
  std::is_same_v<Item, CoordinatorAttachmentCompletion>;

template<typename Item>
[[nodiscard]] GoalGeneration source_generation(const Item & item) noexcept
{
  if constexpr (std::is_same_v<Item, CoordinatorControlEvent> ||
    std::is_same_v<Item, CoordinatorAcceptedGoal>)
  {
    return item.goal_generation;
  } else if constexpr (kWrapsPortCompletion<Item>) {
    return item.completion.correlation.goal_generation;
  } else {
    return item.correlation.goal_generation;
  }
}

template<typename Item>
[[nodiscard]] SteadyTime source_arrival(const Item & item) noexcept
{
  return item.arrived_at;
}

[[nodiscard]] bool valid_push_item(const CoordinatorInboxPushItem & item) noexcept
{
  return std::visit(
    [](const auto & value) noexcept {
      using Value = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<Value, CoordinatorControlEvent>) {
        return admissible_goal_generation(value.goal_generation);
      } else if constexpr (kWrapsPortCompletion<Value>) {
        return admissible_goal_generation(value.completion.correlation.goal_generation) &&
               value.completion.correlation.operation_generation != 0U;
      } else {
        return valid_completion(value);
      }
    }, item);
}

[[nodiscard]] bool same_reconciliation(
  const std::optional<CoordinatorReconciliationCorrelation> & lhs,
  const std::optional<CoordinatorReconciliationCorrelation> & rhs) noexcept
{
  if (lhs.has_value() != rhs.has_value()) {
    return false;
  }
  return !lhs ||
         (lhs->kind == rhs->kind && lhs->attempt_number == rhs->attempt_number);
}

[[nodiscard]] CoordinatorCleanupEvidenceLossMarker cleanup_marker(
  const CoordinatorCleanupItem & item) noexcept
{
  return std::visit(
    [](const auto & value) noexcept {
      using Value = std::decay_t<decltype(value)>;
      constexpr auto kind = std::is_same_v<Value, SnapshotCompletion>?
      CoordinatorCleanupEvidenceKind::kSnapshot :
      CoordinatorCleanupEvidenceKind::kReleaseReservation;
      return CoordinatorCleanupEvidenceLossMarker{
        kind, value.correlation, value.reconciliation, value.arrived_at};
    }, item);
}

[[nodiscard]] bool same_cleanup_marker(
  const CoordinatorCleanupEvidenceLossMarker & lhs,
  const CoordinatorCleanupEvidenceLossMarker & rhs) noexcept
{
  return lhs.kind == rhs.kind &&
         lhs.correlation.goal_generation == rhs.correlation.goal_generation &&
         lhs.correlation.operation_generation == rhs.correlation.operation_generation &&
         same_reconciliation(lhs.reconciliation, rhs.reconciliation);
}

template<typename Item>
[[nodiscard]] bool has_generation(const Item & item, GoalGeneration generation) noexcept
{
  return source_generation(item) == generation;
}

[[nodiscard]] bool delivery_has_generation(
  const CoordinatorInboxDelivery & delivery, GoalGeneration generation) noexcept
{
  return std::visit(
    [generation](const auto & value) noexcept -> bool {
      using Value = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<Value, CoordinatorOverflowMarker>) {
        return value.goal_generation == generation;
      } else if constexpr (std::is_same_v<Value, CoordinatorCleanupEvidenceLossMarker>) {
        return value.correlation.goal_generation == generation;
      } else {
        return has_generation(value, generation);
      }
    }, delivery);
}

}  // namespace

static_assert(std::is_nothrow_move_constructible_v<CoordinatorAcceptedGoal>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorControlEvent>);
static_assert(std::is_nothrow_move_constructible_v<SnapshotCompletion>);
static_assert(std::is_nothrow_move_constructible_v<ReserveTaskCompletion>);
static_assert(std::is_nothrow_move_constructible_v<ValidateReservationCompletion>);
static_assert(std::is_nothrow_move_constructible_v<ReleaseReservationCompletion>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorMotionCompletion>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorGripperCompletion>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorAttachmentCompletion>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorLaneAcquireCompletion>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorOverflowMarker>);
static_assert(
  std::is_nothrow_move_constructible_v<CoordinatorCleanupEvidenceLossMarker>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorInboxPushItem>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorInboxDelivery>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorCleanupItem>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorInboxDelivery>);
static_assert(std::is_nothrow_destructible_v<CoordinatorInboxDelivery>);
static_assert(
  noexcept(
    std::declval<std::optional<CoordinatorAcceptedGoal> &>().emplace(
      std::declval<CoordinatorAcceptedGoal &&>())));
static_assert(
  noexcept(
    std::declval<std::optional<CoordinatorControlEvent> &>().emplace(
      std::declval<CoordinatorControlEvent &&>())));
static_assert(
  noexcept(
    CoordinatorInboxDelivery{std::declval<CoordinatorOverflowMarker &&>()}));
static_assert(
  noexcept(
    CoordinatorInboxDelivery{std::declval<CoordinatorAcceptedGoal &&>()}));
static_assert(
  noexcept(
    CoordinatorInboxDelivery{
    std::declval<CoordinatorCleanupEvidenceLossMarker &&>()}));
static_assert(
  noexcept(
    std::optional<CoordinatorInboxDelivery>{
    std::declval<CoordinatorInboxDelivery &&>()}));

CoordinatorInbox::CoordinatorInbox(std::size_t capacity)
: capacity_(capacity),
  slots_(capacity),
  deferred_slots_(
    capacity + kAcceptedHandoffEmergencyCapacity +
    kCleanupEmergencyCapacity + kOverflowMarkerCapacity +
    kCleanupEvidenceLossMarkerCapacity)
{
  if (capacity_ == 0U) {
    throw std::invalid_argument("coordinator inbox capacity must be nonzero");
  }
}

std::string_view CoordinatorInbox::detail_for(OverflowDiagnostic diagnostic) noexcept
{
  switch (diagnostic) {
    case OverflowDiagnostic::kOrdinaryCapacity:
      return "coordinator inbox ordinary capacity exhausted";
    case OverflowDiagnostic::kAcceptedHandoff:
      return "coordinator accepted-goal handoff used emergency storage";
    case OverflowDiagnostic::kCleanupEvidence:
      return "coordinator cleanup evidence storage exhausted";
    case OverflowDiagnostic::kCount:
      break;
  }
  return "coordinator inbox evidence conflict";
}

void CoordinatorInbox::latch_overflow(OverflowDiagnostic diagnostic) noexcept
{
  if (!overflow_latched_) {
    overflow_diagnostic_ = diagnostic;
  }
  overflow_latched_ = true;
}

bool CoordinatorInbox::ensure_overflow_marker(
  GoalGeneration generation, SteadyTime arrived_at,
  OverflowDiagnostic diagnostic) noexcept
{
  if (!overflow_marker_) {
    overflow_marker_.emplace(
      CoordinatorOverflowMarker{generation, arrived_at, detail_for(diagnostic)});
    return true;
  }
  if (overflow_marker_->goal_generation != generation) {
    generation_accounting_conflict_ = true;
    return false;
  }
  overflow_marker_->arrived_at = std::min(overflow_marker_->arrived_at, arrived_at);
  return true;
}

CoordinatorInboxDepositResult CoordinatorInbox::push(CoordinatorInboxPushItem item)
{
  std::scoped_lock lock(mutex_);
  if (!valid_push_item(item)) {
    generation_accounting_conflict_ = true;
    return kConflict;
  }
  if (generation_accounting_conflict_) {
    return kConflict;
  }

  const auto generation = std::visit(
    [](const auto & value) noexcept {return source_generation(value);}, item);
  const auto arrived_at = std::visit(
    [](const auto & value) noexcept {return source_arrival(value);}, item);

  if (!overflow_latched_ && size_ + deferred_size_ < capacity_) {
    slots_[tail_].emplace(
      std::visit(
        [](auto && value) -> CoordinatorInboxStorageItem {
          return std::forward<decltype(value)>(value);
        }, std::move(item)));
    tail_ = (tail_ + 1U) % capacity_;
    ++size_;
    return kAccepted;
  }

  const bool first_overflow = !overflow_latched_;
  latch_overflow(OverflowDiagnostic::kOrdinaryCapacity);
  if (!ensure_overflow_marker(
      generation, arrived_at, OverflowDiagnostic::kOrdinaryCapacity))
  {
    return {
      CoordinatorInboxDepositStatus::kInhibited,
      CoordinatorInboxPersistenceStatus::kUnresolved};
  }
  return first_overflow ? CoordinatorInboxDepositResult{
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned} :
         CoordinatorInboxDepositResult{
           CoordinatorInboxDepositStatus::kInhibited,
           CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned};
}

CoordinatorInboxDepositResult CoordinatorInbox::push_accepted_goal(
  CoordinatorAcceptedGoal accepted,
  std::optional<CoordinatorControlEvent> retained_termination,
  SteadyTime arrived_at)
{
  std::scoped_lock lock(mutex_);
  if (!admissible_goal_generation(accepted.goal_generation) ||
    (retained_termination &&
    (!admissible_goal_generation(retained_termination->goal_generation) ||
    retained_termination->goal_generation != accepted.goal_generation)))
  {
    generation_accounting_conflict_ = true;
    return kConflict;
  }
  if (generation_accounting_conflict_) {
    return kConflict;
  }

  const std::size_t required_slots = retained_termination ? 2U : 1U;
  // Compare by addition, not capacity_ - size_ - deferred_size_: deferred can exceed capacity_
  // after requeue of emergency-lane deliveries, and unsigned subtraction would wrap fail-open.
  if (!overflow_latched_ && size_ + deferred_size_ <= capacity_ &&
    required_slots <= capacity_ - (size_ + deferred_size_))
  {
    slots_[tail_].emplace(std::move(accepted));
    tail_ = (tail_ + 1U) % capacity_;
    ++size_;
    if (retained_termination) {
      slots_[tail_].emplace(std::move(*retained_termination));
      tail_ = (tail_ + 1U) % capacity_;
      ++size_;
    }
    return kAccepted;
  }

  if (accepted_handoff_emergency_ || accepted_handoff_termination_emergency_) {
    if (!ensure_overflow_marker(
        accepted.goal_generation, arrived_at, OverflowDiagnostic::kAcceptedHandoff))
    {
      return {
        CoordinatorInboxDepositStatus::kInhibited,
        CoordinatorInboxPersistenceStatus::kUnresolved};
    }
    return {
      CoordinatorInboxDepositStatus::kInhibited,
      CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned};
  }

  const auto generation = accepted.goal_generation;
  accepted_handoff_emergency_.emplace(std::move(accepted));
  if (retained_termination) {
    accepted_handoff_termination_emergency_.emplace(std::move(*retained_termination));
  }
  latch_overflow(OverflowDiagnostic::kAcceptedHandoff);
  if (!ensure_overflow_marker(generation, arrived_at, OverflowDiagnostic::kAcceptedHandoff)) {
    return kConflict;
  }
  return {
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kEventOwned};
}

CoordinatorInboxDepositResult CoordinatorInbox::push_cleanup(CoordinatorCleanupItem item)
{
  std::scoped_lock lock(mutex_);
  const bool valid = std::visit(
    [](const auto & value) noexcept {return valid_completion(value);}, item);
  if (!valid) {
    generation_accounting_conflict_ = true;
    return kConflict;
  }
  if (generation_accounting_conflict_) {
    return kConflict;
  }
  if (cleanup_emergency_size_ < cleanup_emergency_slots_.size()) {
    cleanup_emergency_slots_[cleanup_emergency_tail_].emplace(std::move(item));
    cleanup_emergency_tail_ =
      (cleanup_emergency_tail_ + 1U) % cleanup_emergency_slots_.size();
    ++cleanup_emergency_size_;
    return kAccepted;
  }

  const auto marker = cleanup_marker(item);
  if (!cleanup_evidence_loss_marker_) {
    cleanup_evidence_loss_marker_.emplace(marker);
  } else if (!same_cleanup_marker(*cleanup_evidence_loss_marker_, marker)) {
    generation_accounting_conflict_ = true;
    return kConflict;
  } else {
    cleanup_evidence_loss_marker_->arrived_at =
      std::min(cleanup_evidence_loss_marker_->arrived_at, marker.arrived_at);
  }
  latch_overflow(OverflowDiagnostic::kCleanupEvidence);
  return {
    CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned};
}

std::optional<CoordinatorInboxDelivery> CoordinatorInbox::try_pop()
{
  std::scoped_lock lock(mutex_);
  if (deferred_size_ != 0U) {
    CoordinatorInboxDelivery delivery{std::move(*deferred_slots_[deferred_head_])};
    deferred_slots_[deferred_head_].reset();
    deferred_head_ = (deferred_head_ + 1U) % deferred_slots_.size();
    --deferred_size_;
    return delivery;
  }
  if (overflow_marker_) {
    CoordinatorInboxDelivery delivery{std::move(*overflow_marker_)};
    overflow_marker_.reset();
    return delivery;
  }
  if (accepted_handoff_emergency_) {
    CoordinatorInboxDelivery delivery{std::move(*accepted_handoff_emergency_)};
    accepted_handoff_emergency_.reset();
    return delivery;
  }
  if (accepted_handoff_termination_emergency_) {
    CoordinatorInboxDelivery delivery{
      std::move(*accepted_handoff_termination_emergency_)};
    accepted_handoff_termination_emergency_.reset();
    return delivery;
  }
  if (cleanup_evidence_loss_marker_) {
    CoordinatorInboxDelivery delivery{std::move(*cleanup_evidence_loss_marker_)};
    cleanup_evidence_loss_marker_.reset();
    return delivery;
  }
  if (cleanup_emergency_size_ != 0U) {
    CoordinatorCleanupItem item =
      std::move(*cleanup_emergency_slots_[cleanup_emergency_head_]);
    cleanup_emergency_slots_[cleanup_emergency_head_].reset();
    cleanup_emergency_head_ =
      (cleanup_emergency_head_ + 1U) % cleanup_emergency_slots_.size();
    --cleanup_emergency_size_;
    return std::visit(
      [](auto && value) -> CoordinatorInboxDelivery {
        return std::forward<decltype(value)>(value);
      }, std::move(item));
  }
  if (size_ == 0U) {
    return std::nullopt;
  }
  CoordinatorInboxStorageItem item = std::move(*slots_[head_]);
  slots_[head_].reset();
  head_ = (head_ + 1U) % capacity_;
  --size_;
  return std::visit(
    [](auto && value) -> CoordinatorInboxDelivery {
      return std::forward<decltype(value)>(value);
    }, std::move(item));
}

bool CoordinatorInbox::try_requeue(CoordinatorInboxDelivery & delivery) noexcept
{
  std::scoped_lock lock(mutex_);
  if (generation_accounting_conflict_) {
    return false;
  }
  if (deferred_size_ >= deferred_slots_.size()) {
    generation_accounting_conflict_ = true;
    return false;
  }
  // Push to the deferred front so reverse-order requeue restores original pop order.
  deferred_head_ =
    (deferred_head_ + deferred_slots_.size() - 1U) % deferred_slots_.size();
  deferred_slots_[deferred_head_].emplace(std::move(delivery));
  ++deferred_size_;
  return true;
}

CoordinatorInboxSnapshot CoordinatorInbox::snapshot() const
{
  std::scoped_lock lock(mutex_);
  const std::size_t accepted_handoff_emergency_size =
    static_cast<std::size_t>(accepted_handoff_emergency_.has_value()) +
    static_cast<std::size_t>(accepted_handoff_termination_emergency_.has_value());
  return {
    size_ + deferred_size_ + accepted_handoff_emergency_size +
    cleanup_emergency_size_ + static_cast<std::size_t>(overflow_marker_.has_value()) +
    static_cast<std::size_t>(cleanup_evidence_loss_marker_.has_value()),
    capacity_, accepted_handoff_emergency_size,
    cleanup_emergency_size_, overflow_latched_, overflow_marker_.has_value(),
    cleanup_evidence_loss_marker_.has_value(), generation_accounting_conflict_,
    overflow_latched_ ? std::string(detail_for(overflow_diagnostic_)) : std::string{}};
}

bool CoordinatorInbox::generation_empty(GoalGeneration generation) const
{
  std::scoped_lock lock(mutex_);
  if (!admissible_goal_generation(generation) || generation_accounting_conflict_) {
    return false;
  }
  if (overflow_marker_ && overflow_marker_->goal_generation == generation) {
    return false;
  }
  if (accepted_handoff_emergency_ &&
    accepted_handoff_emergency_->goal_generation == generation)
  {
    return false;
  }
  if (accepted_handoff_termination_emergency_ &&
    accepted_handoff_termination_emergency_->goal_generation == generation)
  {
    return false;
  }
  if (cleanup_evidence_loss_marker_ &&
    cleanup_evidence_loss_marker_->correlation.goal_generation == generation)
  {
    return false;
  }
  for (std::size_t index = 0U; index < deferred_size_; ++index) {
    const std::size_t slot =
      (deferred_head_ + index) % deferred_slots_.size();
    if (deferred_slots_[slot] &&
      delivery_has_generation(*deferred_slots_[slot], generation))
    {
      return false;
    }
  }
  for (const auto & slot : slots_) {
    if (slot && std::visit(
        [generation](const auto & value) noexcept {
          return has_generation(value, generation);
        }, *slot))
    {
      return false;
    }
  }
  for (const auto & slot : cleanup_emergency_slots_) {
    if (slot && std::visit(
        [generation](const auto & value) noexcept {
          return has_generation(value, generation);
        }, *slot))
    {
      return false;
    }
  }
  return true;
}

}  // namespace restocker_task_executor
