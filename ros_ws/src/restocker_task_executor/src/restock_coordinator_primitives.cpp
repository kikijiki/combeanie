// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

#include <algorithm>
#include <cassert>
#include <limits>
#include <type_traits>
#include <utility>

namespace restocker_task_executor
{
class MotionLedgerIssuerCore final
{
};

class MotionAdmissionIssuerCore final
{
};

static_assert(std::is_nothrow_move_constructible_v<MotionTimeoutCheckResult>);
static_assert(std::is_nothrow_destructible_v<MotionTimeoutCheckResult>);
using MutableGoalTerminationHandle = std::shared_ptr<FirstGoalTerminationRecord>;
using ImmutableGoalTerminationHandle = std::shared_ptr<const FirstGoalTerminationRecord>;
static_assert(std::is_nothrow_move_constructible_v<MutableGoalTerminationHandle>);
static_assert(std::is_nothrow_destructible_v<MutableGoalTerminationHandle>);
static_assert(std::is_nothrow_move_constructible_v<ImmutableGoalTerminationHandle>);
static_assert(std::is_nothrow_destructible_v<ImmutableGoalTerminationHandle>);
static_assert(
  noexcept(
    std::declval<ImmutableGoalTerminationHandle &>() =
    std::declval<MutableGoalTerminationHandle &&>()));
static_assert(
  std::is_nothrow_move_constructible_v<std::optional<MotionTimeoutCheckResult>>);
static_assert(std::is_nothrow_destructible_v<std::optional<MotionTimeoutCheckResult>>);
static_assert(
  noexcept(
    std::optional<MotionTimeoutCheckResult>{
    std::declval<std::optional<MotionTimeoutCheckResult> &&>()}));
static_assert(
  noexcept(std::declval<std::optional<MotionTimeoutCheckResult> &>().reset()));
static_assert(std::is_nothrow_move_constructible_v<MotionTimeoutCheckDecision>);
static_assert(std::is_nothrow_destructible_v<MotionTimeoutCheckDecision>);
static_assert(std::is_nothrow_move_constructible_v<EffectiveMotionTimeout>);
static_assert(
  std::is_nothrow_copy_assignable_v<std::optional<EffectiveMotionTimeout>>);
static_assert(std::is_nothrow_move_constructible_v<DefinitelyNotSubmittedRecord>);
static_assert(std::is_nothrow_move_constructible_v<MotionStopProofRecord>);
static_assert(std::is_nothrow_move_constructible_v<MotionLedgerClearanceRecord>);
static_assert(std::is_nothrow_destructible_v<DefinitelyNotSubmittedRecord>);
static_assert(std::is_nothrow_destructible_v<MotionStopProofRecord>);
static_assert(std::is_nothrow_destructible_v<MotionLedgerClearanceRecord>);
static_assert(std::is_nothrow_move_constructible_v<DefinitelyNotSubmittedReceipt>);
static_assert(std::is_nothrow_move_constructible_v<MotionStopProofReceipt>);
static_assert(std::is_nothrow_destructible_v<DefinitelyNotSubmittedReceipt>);
static_assert(std::is_nothrow_destructible_v<MotionStopProofReceipt>);
static_assert(std::is_nothrow_move_constructible_v<PreparedMotionLedgerClearance>);
static_assert(
  std::is_nothrow_move_constructible_v<std::optional<PreparedMotionLedgerClearance>>);
static_assert(std::is_nothrow_destructible_v<PreparedMotionLedgerClearance>);
static_assert(std::is_nothrow_move_constructible_v<MotionLedgerClearancePreparationDecision>);
static_assert(std::is_nothrow_destructible_v<MotionLedgerClearancePreparationDecision>);
static_assert(std::is_nothrow_move_constructible_v<MotionLedgerClearanceProof>);
static_assert(std::is_nothrow_move_constructible_v<std::optional<MotionLedgerClearanceProof>>);
static_assert(std::is_nothrow_destructible_v<MotionLedgerClearanceProof>);
static_assert(std::is_nothrow_move_constructible_v<MotionLedgerClearDecision>);
static_assert(std::is_nothrow_destructible_v<MotionLedgerClearDecision>);
static_assert(
  std::is_nothrow_copy_constructible_v<std::shared_ptr<const MotionLedgerIssuerCore>>);
static_assert(
  std::is_nothrow_move_constructible_v<std::shared_ptr<const MotionLedgerIssuerCore>>);
static_assert(
  std::is_nothrow_copy_constructible_v<std::shared_ptr<const MotionLedgerClearanceRecord>>);
static_assert(
  std::is_nothrow_move_constructible_v<std::shared_ptr<const MotionLedgerClearanceRecord>>);
static_assert(noexcept(std::declval<std::string &>().swap(std::declval<std::string &>())));
static_assert(std::is_nothrow_move_assignable_v<std::string>);
static_assert(std::is_nothrow_move_constructible_v<MotionAdmissionRegistrationRecord>);
static_assert(std::is_nothrow_destructible_v<MotionAdmissionRegistrationRecord>);
static_assert(std::is_nothrow_move_constructible_v<MotionAdmissionRegistrationReceipt>);
static_assert(std::is_nothrow_move_assignable_v<MotionAdmissionRegistrationReceipt>);
static_assert(std::is_nothrow_destructible_v<MotionAdmissionRegistrationReceipt>);
static_assert(std::is_nothrow_move_constructible_v<MotionAdmissionRegistrationDecision>);
static_assert(std::is_nothrow_destructible_v<MotionAdmissionRegistrationDecision>);
static_assert(std::is_nothrow_move_constructible_v<MotionAdmissionWithdrawalDecision>);
static_assert(std::is_nothrow_destructible_v<MotionAdmissionWithdrawalDecision>);
static_assert(
  std::is_nothrow_move_constructible_v<
    std::optional<MotionAdmissionRegistrationReceipt>>);
static_assert(
  std::is_nothrow_destructible_v<std::optional<MotionAdmissionRegistrationReceipt>>);
static_assert(
  std::is_nothrow_copy_constructible_v<
    std::shared_ptr<const MotionAdmissionIssuerCore>>);
static_assert(
  std::is_nothrow_move_constructible_v<
    std::shared_ptr<const MotionAdmissionIssuerCore>>);
static_assert(
  std::is_nothrow_copy_constructible_v<
    std::shared_ptr<const MotionAdmissionRegistrationRecord>>);
static_assert(
  std::is_nothrow_move_constructible_v<
    std::shared_ptr<const MotionAdmissionRegistrationRecord>>);
static_assert(
  std::is_nothrow_copy_constructible_v<MotionAdmissionIssuerBinding>);
static_assert(
  std::is_nothrow_copy_constructible_v<MotionSubmissionAuthorizationIssuerBinding>);
static_assert(
  std::is_nothrow_move_constructible_v<
    std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore>>);
static_assert(std::is_nothrow_move_constructible_v<MotionAdmissionRuntimeReservationWitness>);
static_assert(std::is_nothrow_destructible_v<MotionAdmissionRuntimeReservationWitness>);

namespace
{

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id)
{
  return std::ranges::any_of(goal_id, [](std::uint8_t value) {return value != 0;});
}

[[nodiscard]] constexpr bool valid_goal_slot_phase(GoalSlotPhase phase) noexcept
{
  switch (phase) {
    case GoalSlotPhase::kIdle:
    case GoalSlotPhase::kPendingAcceptance:
    case GoalSlotPhase::kActive:
      return true;
  }
  return false;
}

[[nodiscard]] constexpr bool valid_goal_termination_intent(GoalTerminationIntent intent) noexcept
{
  switch (intent) {
    case GoalTerminationIntent::kNone:
    case GoalTerminationIntent::kUserCancel:
    case GoalTerminationIntent::kShutdownDrain:
    case GoalTerminationIntent::kSafeAbort:
    case GoalTerminationIntent::kTaskDeadline:
      return true;
  }
  return false;
}

[[nodiscard]] bool matches(
  const CoordinatorGoalId & expected_id, GoalGeneration expected_generation,
  const CoordinatorGoalId & actual_id, GoalGeneration actual_generation)
{
  return expected_generation == actual_generation && expected_id == actual_id;
}

[[nodiscard]] constexpr std::uint64_t goal_id_word(
  const CoordinatorGoalId & goal_id, std::size_t offset) noexcept
{
  std::uint64_t word = 0U;
  for (std::size_t index = 0U; index < sizeof(word); ++index) {
    word |= static_cast<std::uint64_t>(goal_id[offset + index]) << (index * 8U);
  }
  return word;
}

static_assert(CoordinatorGoalId{}.size() == 2U * sizeof(std::uint64_t));

[[nodiscard]] bool valid_mutation_kind(CoordinatorMutationKind kind)
{
  switch (kind) {
    case CoordinatorMutationKind::kReserveTask:
    case CoordinatorMutationKind::kReleaseTask:
    case CoordinatorMutationKind::kCheckpointTask:
    case CoordinatorMutationKind::kAcquireSceneLease:
    case CoordinatorMutationKind::kReleaseSceneLease:
    case CoordinatorMutationKind::kSetPhysicalAttachment:
    case CoordinatorMutationKind::kApplyPlanningScene:
    case CoordinatorMutationKind::kCommitAttachment:
    case CoordinatorMutationKind::kCommitDetachment:
      return true;
  }
  return false;
}

[[nodiscard]] GoalTerminationIntent dominant_intent(
  GoalTerminationIntent current, GoalTerminationIntent incoming) noexcept
{
  const auto priority = [](GoalTerminationIntent intent) {
    switch (intent) {
      case GoalTerminationIntent::kNone: return 0;
      case GoalTerminationIntent::kUserCancel: return 1;
      case GoalTerminationIntent::kShutdownDrain: return 2;
      case GoalTerminationIntent::kSafeAbort: return 3;
      case GoalTerminationIntent::kTaskDeadline: return 4;
    }
    return 0;
  };
  return priority(incoming) > priority(current) ? incoming : current;
}

[[nodiscard]] SteadyTime bounded_deadline(
  SteadyTime now, std::chrono::milliseconds timeout)
{
  const auto remaining = SteadyTime::max() - now;
  if (timeout >= std::chrono::duration_cast<std::chrono::milliseconds>(remaining)) {
    return SteadyTime::max();
  }
  return now + std::chrono::duration_cast<SteadyTime::duration>(timeout);
}

[[nodiscard]] bool valid_reconciliation_kind(ReconciliationKind kind)
{
  switch (kind) {
    case ReconciliationKind::kReplayMutation:
    case ReconciliationKind::kReadback:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_operation_effect(OperationEffect effect)
{
  switch (effect) {
    case OperationEffect::kReadOnly:
    case OperationEffect::kCancelableMotion:
    case OperationEffect::kIdempotentMutation:
      return true;
  }
  return false;
}

[[nodiscard]] bool valid_generic_operation_pair(
  RestockTaskCommand command, OperationEffect effect)
{
  switch (command) {
    case RestockTaskCommand::kValidateScene:
    case RestockTaskCommand::kSelectPair:
    // Each plan command is one outstanding plan-and-execute segment, tracked as a single async
    // operation.
    case RestockTaskCommand::kPlanPreGrasp:
    case RestockTaskCommand::kPlanApproach:
    case RestockTaskCommand::kPlanRetract:
    case RestockTaskCommand::kPlanCarryStart:
    case RestockTaskCommand::kPlanPreInsert:
    case RestockTaskCommand::kPlanInsert:
    case RestockTaskCommand::kPlanRetreat:
    case RestockTaskCommand::kObserveDestination:
    case RestockTaskCommand::kSurveyDestination:
    case RestockTaskCommand::kVerifyGrasp:
    case RestockTaskCommand::kVerifyPlacement:
    case RestockTaskCommand::kUpdateInventory:
    case RestockTaskCommand::kCloseGripper:
    case RestockTaskCommand::kOpenGripper:
    // Recovery observation does not change the world.
    case RestockTaskCommand::kExecuteRecovery:
      return effect == OperationEffect::kReadOnly;
    case RestockTaskCommand::kReserveTask:
    case RestockTaskCommand::kReleaseTaskReservation:
    // Attachment transactions are replayable idempotent mutations. Physical-only detach is tracked
    // the same way but leaves world state untouched until the commit half.
    case RestockTaskCommand::kRunAttachTransaction:
    case RestockTaskCommand::kRunDetachTransaction:
    case RestockTaskCommand::kRunCommitDetachment:
      return effect == OperationEffect::kReadOnly ||
             effect == OperationEffect::kIdempotentMutation;
    default:
      return false;
  }
}

[[nodiscard]] bool valid_reconciliation_evidence(
  RestockTaskCommand command, ReconciliationKind kind, ReconciliationEvidence evidence)
{
  switch (evidence) {
    case ReconciliationEvidence::kInconclusive:
      return true;
    case ReconciliationEvidence::kMutationResponseRetained:
      return kind == ReconciliationKind::kReplayMutation &&
             (command == RestockTaskCommand::kReserveTask ||
             command == RestockTaskCommand::kReleaseTaskReservation);
    case ReconciliationEvidence::kExactMutationStillApplied:
      return kind == ReconciliationKind::kReadback &&
             command == RestockTaskCommand::kReleaseTaskReservation;
  }
  return false;
}

}  // namespace

MotionLedgerIssuerBinding::MotionLedgerIssuerBinding(
  std::shared_ptr<const MotionLedgerIssuerCore> issuer) noexcept
: issuer_(std::move(issuer))
{
}

bool MotionLedgerIssuerBinding::same_issuer_as(
  const MotionLedgerIssuerBinding & other) const noexcept
{
  return issuer_.get() == other.issuer_.get();
}

MotionAdmissionIssuerBinding::MotionAdmissionIssuerBinding(
  std::shared_ptr<const MotionAdmissionIssuerCore> issuer) noexcept
: issuer_(std::move(issuer))
{
}

bool MotionAdmissionIssuerBinding::same_issuer_as(
  const MotionAdmissionIssuerBinding & other) const noexcept
{
  return issuer_.get() == other.issuer_.get();
}

MotionSubmissionAuthorizationIssuerBinding::MotionSubmissionAuthorizationIssuerBinding(
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> issuer) noexcept
: issuer_(std::move(issuer))
{
}

bool MotionSubmissionAuthorizationIssuerBinding::same_issuer_as(
  const MotionSubmissionAuthorizationIssuerBinding & other) const noexcept
{
  return issuer_.get() == other.issuer_.get();
}

MotionAdmissionRuntimeReservationWitness::MotionAdmissionRuntimeReservationWitness(
  std::shared_ptr<const MotionAdmissionIssuerCore> admission_issuer,
  std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
  std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer,
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> runtime_issuer) noexcept
: admission_issuer_(std::move(admission_issuer)),
  registration_(std::move(registration)),
  ledger_issuer_(std::move(ledger_issuer)),
  runtime_issuer_(std::move(runtime_issuer)),
  state_(State::kLive)
{
}

MotionAdmissionRuntimeReservationWitness::MotionAdmissionRuntimeReservationWitness(
  MotionAdmissionRuntimeReservationWitness && other) noexcept
: admission_issuer_(std::move(other.admission_issuer_)),
  registration_(std::move(other.registration_)),
  ledger_issuer_(std::move(other.ledger_issuer_)),
  runtime_issuer_(std::move(other.runtime_issuer_)),
  state_(other.state_)
{
  other.state_ = State::kMovedFrom;
}

MotionAdmissionRegistrationReceipt::MotionAdmissionRegistrationReceipt(
  std::shared_ptr<const MotionAdmissionIssuerCore> issuer,
  std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
  std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer) noexcept
: issuer_(std::move(issuer)),
  registration_(std::move(registration)),
  ledger_issuer_(std::move(ledger_issuer)),
  state_(State::kLive),
  request_binding_permit_live_(true)
{
}

MotionAdmissionRegistrationReceipt::MotionAdmissionRegistrationReceipt(
  MotionAdmissionRegistrationReceipt && other) noexcept
: issuer_(std::move(other.issuer_)),
  registration_(std::move(other.registration_)),
  ledger_issuer_(std::move(other.ledger_issuer_)),
  state_(other.state_),
  request_binding_permit_live_(other.request_binding_permit_live_)
{
  other.issuer_.reset();
  other.registration_.reset();
  other.ledger_issuer_.reset();
  other.state_ = State::kMovedFrom;
  other.request_binding_permit_live_ = false;
}

MotionAdmissionRegistrationReceipt & MotionAdmissionRegistrationReceipt::operator=(
  MotionAdmissionRegistrationReceipt && other) noexcept
{
  if (this == &other) {
    return *this;
  }
  issuer_ = std::move(other.issuer_);
  registration_ = std::move(other.registration_);
  ledger_issuer_ = std::move(other.ledger_issuer_);
  state_ = other.state_;
  request_binding_permit_live_ = other.request_binding_permit_live_;
  other.issuer_.reset();
  other.registration_.reset();
  other.ledger_issuer_.reset();
  other.state_ = State::kMovedFrom;
  other.request_binding_permit_live_ = false;
  return *this;
}

bool MotionAdmissionRegistrationReceipt::live() const noexcept
{
  return state_ == State::kLive && issuer_ && registration_ && ledger_issuer_ &&
         request_binding_permit_live_;
}

bool MotionAdmissionRegistrationReceipt::withdrawn() const noexcept
{
  return state_ == State::kWithdrawn;
}

const MotionAdmissionRegistrationRecord *
MotionAdmissionRegistrationReceipt::registration() const noexcept
{
  return live() ? registration_.get() : nullptr;
}

bool MotionAdmissionRegistrationReceipt::issued_for_ledger(
  const MotionLedgerIssuerBinding & binding) const noexcept
{
  return live() && ledger_issuer_.get() == binding.issuer_.get();
}

MotionAdmissionRegistrationDecision::MotionAdmissionRegistrationDecision(
  MotionAdmissionRegistrationDecision && other) noexcept
{
  if (other.status_ == MotionAdmissionRegistrationStatus::kRegistered &&
    other.receipt_ && other.receipt_->live())
  {
    receipt_.emplace(std::move(*other.receipt_));
    status_ = MotionAdmissionRegistrationStatus::kRegistered;
  } else {
    if (other.status_ == MotionAdmissionRegistrationStatus::kTerminationWon &&
      other.first_termination_)
    {
      first_termination_ = std::move(other.first_termination_);
      status_ = MotionAdmissionRegistrationStatus::kTerminationWon;
    } else {
      status_ = other.status_;
    }
  }
  other.receipt_.reset();
  other.first_termination_.reset();
  other.status_ = MotionAdmissionRegistrationStatus::kInvalidRegistration;
}

MotionAdmissionRegistrationStatus MotionAdmissionRegistrationDecision::status() const noexcept
{
  return status_;
}

const MotionAdmissionRegistrationReceipt *
MotionAdmissionRegistrationDecision::receipt() const noexcept
{
  return status_ == MotionAdmissionRegistrationStatus::kRegistered && receipt_ &&
         receipt_->live() ? &*receipt_ : nullptr;
}

std::optional<MotionAdmissionRegistrationReceipt>
MotionAdmissionRegistrationDecision::take_receipt() noexcept
{
  std::optional<MotionAdmissionRegistrationReceipt> result;
  if (receipt() != nullptr) {
    result.emplace(std::move(*receipt_));
  }
  receipt_.reset();
  first_termination_.reset();
  status_ = MotionAdmissionRegistrationStatus::kInvalidRegistration;
  return result;
}

const std::shared_ptr<const FirstGoalTerminationRecord> &
MotionAdmissionRegistrationDecision::first_termination() const noexcept
{
  return first_termination_;
}

MotionAdmissionWithdrawalDecision::MotionAdmissionWithdrawalDecision(
  MotionAdmissionWithdrawalDecision && other) noexcept
: status_(other.status_), first_termination_(std::move(other.first_termination_))
{
  other.status_ = MotionAdmissionWithdrawalStatus::kReceiptMismatch;
  other.first_termination_.reset();
}

MotionAdmissionWithdrawalStatus MotionAdmissionWithdrawalDecision::status() const noexcept
{
  return status_;
}

const std::shared_ptr<const FirstGoalTerminationRecord> &
MotionAdmissionWithdrawalDecision::first_termination() const noexcept
{
  return first_termination_;
}

PreparedMotionLedgerClearance::PreparedMotionLedgerClearance(
  std::shared_ptr<const MotionLedgerIssuerCore> issuer,
  std::shared_ptr<const MotionLedgerClearanceRecord> record,
  MotionLedgerClearanceKind kind) noexcept
: issuer_(std::move(issuer)),
  record_(std::move(record)),
  kind_(kind),
  state_(State::kLive)
{
}

PreparedMotionLedgerClearance::PreparedMotionLedgerClearance(
  PreparedMotionLedgerClearance && other) noexcept
: issuer_(std::move(other.issuer_)),
  record_(std::move(other.record_)),
  kind_(other.kind_),
  state_(other.state_)
{
  other.issuer_.reset();
  other.record_.reset();
  other.kind_ = MotionLedgerClearanceKind::kNonSubmission;
  other.state_ = State::kMovedFrom;
}

bool PreparedMotionLedgerClearance::live() const noexcept
{
  if (state_ != State::kLive || !issuer_ || !record_) {
    return false;
  }
  switch (kind_) {
    case MotionLedgerClearanceKind::kNonSubmission:
      return std::holds_alternative<DefinitelyNotSubmittedRecord>(*record_);
    case MotionLedgerClearanceKind::kStopped:
      return std::holds_alternative<MotionStopProofRecord>(*record_);
  }
  return false;
}

MotionLedgerClearanceKind PreparedMotionLedgerClearance::kind() const noexcept
{
  return kind_;
}

const DefinitelyNotSubmittedRecord *
PreparedMotionLedgerClearance::non_submission_record() const noexcept
{
  return live() ? std::get_if<DefinitelyNotSubmittedRecord>(record_.get()) : nullptr;
}

const MotionStopProofRecord * PreparedMotionLedgerClearance::stop_record() const noexcept
{
  return live() ? std::get_if<MotionStopProofRecord>(record_.get()) : nullptr;
}

MotionLedgerClearancePreparationDecision::MotionLedgerClearancePreparationDecision(
  MotionLedgerClearancePreparationDecision && other) noexcept
{
  if (other.status_ == MotionLedgerClearancePreparationStatus::kPrepared &&
    other.preparation_ && other.preparation_->live())
  {
    preparation_.emplace(std::move(*other.preparation_));
    status_ = MotionLedgerClearancePreparationStatus::kPrepared;
  }
  other.preparation_.reset();
  other.status_ = MotionLedgerClearancePreparationStatus::kInvalidRecord;
}

MotionLedgerClearancePreparationStatus
MotionLedgerClearancePreparationDecision::status() const noexcept
{
  return status_;
}

const PreparedMotionLedgerClearance *
MotionLedgerClearancePreparationDecision::preparation() const noexcept
{
  return status_ == MotionLedgerClearancePreparationStatus::kPrepared &&
         preparation_ && preparation_->live() ? &*preparation_ : nullptr;
}

std::optional<PreparedMotionLedgerClearance>
MotionLedgerClearancePreparationDecision::take_preparation() noexcept
{
  std::optional<PreparedMotionLedgerClearance> result;
  if (preparation() != nullptr) {
    result.emplace(std::move(*preparation_));
  }
  preparation_.reset();
  status_ = MotionLedgerClearancePreparationStatus::kInvalidRecord;
  return result;
}

MotionLedgerClearanceProof::MotionLedgerClearanceProof(
  std::shared_ptr<const MotionLedgerIssuerCore> issuer,
  std::shared_ptr<const MotionLedgerClearanceRecord> record,
  MotionLedgerClearanceGeneration generation,
  MotionLedgerClearanceKind kind) noexcept
: issuer_(std::move(issuer)),
  record_(std::move(record)),
  generation_(generation),
  kind_(kind),
  state_(State::kLive),
  timeout_permit_state_(TimeoutPermitState::kAvailable)
{
}

MotionLedgerClearanceProof::MotionLedgerClearanceProof(
  MotionLedgerClearanceProof && other) noexcept
: issuer_(std::move(other.issuer_)),
  record_(std::move(other.record_)),
  generation_(other.generation_),
  kind_(other.kind_),
  state_(other.state_),
  timeout_permit_state_(other.timeout_permit_state_)
{
  other.issuer_.reset();
  other.record_.reset();
  other.generation_ = 0U;
  other.kind_ = MotionLedgerClearanceKind::kNonSubmission;
  other.state_ = State::kMovedFrom;
  other.timeout_permit_state_ = TimeoutPermitState::kConsumed;
}

bool MotionLedgerClearanceProof::live() const noexcept
{
  if (state_ != State::kLive || !issuer_ || !record_ || generation_ == 0U) {
    return false;
  }
  switch (kind_) {
    case MotionLedgerClearanceKind::kNonSubmission:
      return std::holds_alternative<DefinitelyNotSubmittedRecord>(*record_);
    case MotionLedgerClearanceKind::kStopped:
      return std::holds_alternative<MotionStopProofRecord>(*record_);
  }
  return false;
}

MotionLedgerClearanceGeneration MotionLedgerClearanceProof::generation() const noexcept
{
  return generation_;
}

MotionLedgerClearanceKind MotionLedgerClearanceProof::kind() const noexcept
{
  return kind_;
}

const MotionOperationIdentity * MotionLedgerClearanceProof::exact_identity() const noexcept
{
  if (const auto * record = non_submission_record()) {
    return &record->identity;
  }
  if (const auto * record = stop_record()) {
    return &record->binding.identity;
  }
  return nullptr;
}

const DefinitelyNotSubmittedRecord *
MotionLedgerClearanceProof::non_submission_record() const noexcept
{
  return live() ? std::get_if<DefinitelyNotSubmittedRecord>(record_.get()) : nullptr;
}

const MotionStopProofRecord * MotionLedgerClearanceProof::stop_record() const noexcept
{
  return live() ? std::get_if<MotionStopProofRecord>(record_.get()) : nullptr;
}

bool MotionLedgerClearanceProof::issued_by(
  const MotionLedgerIssuerBinding & binding) const noexcept
{
  return live() && issuer_.get() == binding.issuer_.get();
}

bool MotionLedgerClearanceProof::timeout_correlation_permit_available() const noexcept
{
  return live() && timeout_permit_state_ == TimeoutPermitState::kAvailable;
}

MotionLedgerClearDecision::MotionLedgerClearDecision(
  MotionLedgerClearDecision && other) noexcept
{
  if (other.status_ == MotionLedgerStatus::kApplied &&
    other.clearance_proof_ && other.clearance_proof_->live())
  {
    clearance_proof_.emplace(std::move(*other.clearance_proof_));
    status_ = MotionLedgerStatus::kApplied;
  }
  other.clearance_proof_.reset();
  other.status_ = MotionLedgerStatus::kInvalidArgument;
}

MotionLedgerStatus MotionLedgerClearDecision::status() const noexcept
{
  return status_;
}

const MotionLedgerClearanceProof * MotionLedgerClearDecision::clearance_proof() const noexcept
{
  return status_ == MotionLedgerStatus::kApplied && clearance_proof_ && clearance_proof_->live() ?
         &*clearance_proof_ : nullptr;
}

std::optional<MotionLedgerClearanceProof>
MotionLedgerClearDecision::take_clearance_proof() noexcept
{
  std::optional<MotionLedgerClearanceProof> result;
  if (clearance_proof() != nullptr) {
    result.emplace(std::move(*clearance_proof_));
  }
  clearance_proof_.reset();
  status_ = MotionLedgerStatus::kInvalidArgument;
  return result;
}

void GoalAdmissionSlot::update_readiness(bool ready, std::string detail)
{
  std::scoped_lock lock(mutex_);
  ready_ = ready;
  readiness_detail_ = std::move(detail);
}

GoalAdmissionSlot::GoalAdmissionSlot(
  GoalTerminationAllocationFailureInjector allocation_failure_injector)
: allocation_failure_injector_(std::move(allocation_failure_injector)),
  motion_admission_issuer_(std::make_shared<const MotionAdmissionIssuerCore>())
{
}

GoalAdmissionDecision GoalAdmissionSlot::reserve_status_locked(
  const CoordinatorGoalId & goal_id)
{
  if (!valid_goal_id(goal_id)) {
    return GoalAdmissionDecision::kInvalidGoalId;
  }
  if (inhibited_) {
    return GoalAdmissionDecision::kInhibited;
  }
  if (!ready_) {
    return GoalAdmissionDecision::kNotReady;
  }
  if (phase_ != GoalSlotPhase::kIdle) {
    return GoalAdmissionDecision::kBusy;
  }
  if (!admissible_goal_generation(next_generation_)) {
    inhibited_ = true;
    inhibition_detail_.swap(generation_exhaustion_detail_);
    return GoalAdmissionDecision::kInhibited;
  }
  return GoalAdmissionDecision::kAccepted;
}

void GoalAdmissionSlot::inhibit(std::string detail)
{
  if (detail.empty()) {
    detail = "motion is inhibited";
  }
  std::scoped_lock lock(mutex_);
  inhibited_ = true;
  inhibition_detail_ = std::move(detail);
}

GoalAdmissionReceipt GoalAdmissionSlot::reserve(const CoordinatorGoalId & goal_id)
{
  if (!valid_goal_id(goal_id)) {
    return {GoalAdmissionDecision::kInvalidGoalId, 0};
  }
  const std::uint64_t goal_id_word_0 = goal_id_word(goal_id, 0U);
  const std::uint64_t goal_id_word_1 = goal_id_word(goal_id, sizeof(goal_id_word_0));

  {
    std::scoped_lock lock(mutex_);
    const auto status = reserve_status_locked(goal_id);
    if (status != GoalAdmissionDecision::kAccepted) {
      return {status, 0};
    }
  }

  std::shared_ptr<FirstGoalTerminationRecord> termination_backing;
  try {
    const bool inject_failure = allocation_failure_injector_ && allocation_failure_injector_();
    if (!inject_failure) {
      termination_backing = std::make_shared<FirstGoalTerminationRecord>();
    }
  } catch (...) {
    termination_backing.reset();
  }

  std::scoped_lock lock(mutex_);
  const auto status = reserve_status_locked(goal_id);
  if (status != GoalAdmissionDecision::kAccepted) {
    return {status, 0};
  }
  if (!termination_backing) {
    return {GoalAdmissionDecision::kResourceExhausted, 0};
  }
  generation_ = next_generation_++;
  goal_id_word_0_ = goal_id_word_0;
  goal_id_word_1_ = goal_id_word_1;
  goal_id_ = goal_id;
  phase_ = GoalSlotPhase::kPendingAcceptance;
  cancel_requested_ = false;
  safe_abort_requested_ = false;
  task_deadline_exceeded_ = false;
  termination_intent_ = GoalTerminationIntent::kNone;
  completion_cleanup_ = false;
  skip_cleanup_ = false;
  last_resolved_operation_generation_ = 0;
  mutation_submission_.reset();
  unpublished_termination_record_ = std::move(termination_backing);
  first_termination_record_.reset();
  assert(unpublished_termination_record_ && !first_termination_record_);
  return {GoalAdmissionDecision::kAccepted, generation_};
}

bool GoalAdmissionSlot::activate(
  const CoordinatorGoalId & goal_id, GoalGeneration generation)
{
  const std::uint64_t goal_id_word_0 = goal_id_word(goal_id, 0U);
  const std::uint64_t goal_id_word_1 = goal_id_word(goal_id, sizeof(goal_id_word_0));
  std::scoped_lock lock(mutex_);
  if (phase_ != GoalSlotPhase::kPendingAcceptance || !goal_id_ ||
    generation_ != generation || goal_id_word_0_ != goal_id_word_0 ||
    goal_id_word_1_ != goal_id_word_1)
  {
    return false;
  }
  phase_ = GoalSlotPhase::kActive;
  return true;
}

bool GoalAdmissionSlot::apply_termination_policy_locked(GoalTerminationKind kind) noexcept
{
  switch (kind) {
    case GoalTerminationKind::kUserCancel:
      cancel_requested_ = true;
      termination_intent_ = dominant_intent(
        termination_intent_, GoalTerminationIntent::kUserCancel);
      return true;
    case GoalTerminationKind::kCoordinatorDrain:
    case GoalTerminationKind::kShutdown:
      if (termination_intent_ != GoalTerminationIntent::kUserCancel) {
        safe_abort_requested_ = true;
        termination_intent_ = dominant_intent(
          termination_intent_, GoalTerminationIntent::kShutdownDrain);
      }
      return true;
    case GoalTerminationKind::kSafeAbort:
    case GoalTerminationKind::kAuthorityLoss:
    case GoalTerminationKind::kInboxOverflow:
    case GoalTerminationKind::kProtocolFailure:
      safe_abort_requested_ = true;
      termination_intent_ = dominant_intent(
        termination_intent_, GoalTerminationIntent::kSafeAbort);
      return true;
    case GoalTerminationKind::kTaskDeadline:
      task_deadline_exceeded_ = true;
      termination_intent_ = dominant_intent(
        termination_intent_, GoalTerminationIntent::kTaskDeadline);
      return true;
    case GoalTerminationKind::kMotionDeadline:
      // Installs first-termination lineage without rewriting the whole-task policy flags.
      return true;
  }
  return false;
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_termination(
  FirstGoalTerminationRecord record)
{
  if (record.scope != GoalTerminationScope::kGoal ||
    validate_goal_termination_record(record) != GoalTerminationValidationError::kNone)
  {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kInvalidArgument, nullptr};
  }

  const std::uint64_t record_goal_id_word_0 = goal_id_word(record.goal_id, 0U);
  const std::uint64_t record_goal_id_word_1 =
    goal_id_word(record.goal_id, sizeof(record_goal_id_word_0));

  std::scoped_lock lock(mutex_);
  if (phase_ == GoalSlotPhase::kIdle) {
    return GoalTerminationLatchDecision{GoalTerminationLatchStatus::kInactive, nullptr};
  }
  if (!goal_id_ || generation_ != record.goal_generation ||
    goal_id_word_0_ != record_goal_id_word_0 || goal_id_word_1_ != record_goal_id_word_1)
  {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kGoalMismatch, first_termination_record_};
  }
  if ((!unpublished_termination_record_ && !first_termination_record_) ||
    (unpublished_termination_record_ && first_termination_record_))
  {
    assert(false && "reserved goal has invalid termination backing state");
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kInvalidArgument, nullptr};
  }
  if (!apply_termination_policy_locked(record.kind)) {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kInvalidArgument, nullptr};
  }
  if (first_termination_record_) {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kAlreadyLatched, first_termination_record_};
  }

  if (motion_workflow_active_) {
    if (!motion_registration_ || !motion_ledger_issuer_) {
      assert(false && "active motion workflow has incomplete registration authority");
      return GoalTerminationLatchDecision{
        GoalTerminationLatchStatus::kInvalidArgument, nullptr};
    }
    record.scope = GoalTerminationScope::kRegisteredMotion;
    record.motion_registration = motion_registration_;
  }

  *unpublished_termination_record_ = std::move(record);
  first_termination_record_ = std::move(unpublished_termination_record_);
  assert(!unpublished_termination_record_ && first_termination_record_);
  return GoalTerminationLatchDecision{
    GoalTerminationLatchStatus::kLatched, first_termination_record_};
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_cancel(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kUserCancel,
      MotionCancellationReason::kUserCancel, arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_drain(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation,
      GoalTerminationKind::kCoordinatorDrain, MotionCancellationReason::kCoordinatorDrain,
      arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_safe_abort(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kSafeAbort,
      MotionCancellationReason::kSafeAbort, arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_shutdown(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kShutdown,
      MotionCancellationReason::kShutdown, arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_authority_loss(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kAuthorityLoss,
      MotionCancellationReason::kAuthorityLoss, arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_inbox_loss(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kInboxOverflow,
      MotionCancellationReason::kInboxOverflow, arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_protocol_failure(
  const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at)
{
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation,
      GoalTerminationKind::kProtocolFailure, MotionCancellationReason::kProtocolFailure,
      arrived_at, std::nullopt});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_task_deadline(
  const CoordinatorGoalId & goal_id, GoalGeneration generation,
  GoalGeneration deadline_generation, SteadyTime captured_at)
{
  if (deadline_generation == 0U || deadline_generation != generation) {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kInvalidArgument, nullptr};
  }
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kTaskDeadline,
      MotionCancellationReason::kTaskDeadline, captured_at,
      GoalTerminationSourceGeneration{
        GoalTerminationSource::kTaskDeadline, deadline_generation}});
}

GoalTerminationLatchDecision GoalAdmissionSlot::request_motion_deadline(
  const CoordinatorGoalId & goal_id, GoalGeneration generation,
  std::uint64_t fence_generation, SteadyTime captured_at)
{
  if (fence_generation == 0U) {
    return GoalTerminationLatchDecision{
      GoalTerminationLatchStatus::kInvalidArgument, nullptr};
  }
  return request_termination(
    {GoalTerminationScope::kGoal, goal_id, generation, GoalTerminationKind::kMotionDeadline,
      MotionCancellationReason::kExpectedResultDeadline, captured_at,
      GoalTerminationSourceGeneration{
        GoalTerminationSource::kMotionFence, fence_generation}});
}

GoalTerminationSnapshotDecision GoalAdmissionSlot::snapshot_first_termination(
  const CoordinatorGoalId & goal_id, GoalGeneration generation) const
{
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation)) {
    return GoalTerminationSnapshotDecision{
      GoalTerminationSnapshotStatus::kInvalidArgument, nullptr};
  }

  std::scoped_lock lock(mutex_);
  if (phase_ == GoalSlotPhase::kIdle) {
    return GoalTerminationSnapshotDecision{
      GoalTerminationSnapshotStatus::kInactive, nullptr};
  }
  if (!goal_id_ || !matches(*goal_id_, generation_, goal_id, generation)) {
    return GoalTerminationSnapshotDecision{
      GoalTerminationSnapshotStatus::kGoalMismatch, first_termination_record_};
  }
  if (first_termination_record_) {
    return GoalTerminationSnapshotDecision{
      GoalTerminationSnapshotStatus::kPresent, first_termination_record_};
  }
  return GoalTerminationSnapshotDecision{GoalTerminationSnapshotStatus::kNone, nullptr};
}

GoalActivationAuthorityDecision GoalAdmissionSlot::observe_activation_authority(
  const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept
{
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation)) {
    return {GoalActivationAuthorityStatus::kInvalidArgument, std::nullopt};
  }
  try {
    std::scoped_lock lock(mutex_);
    if (!valid_goal_slot_phase(phase_) ||
      !valid_goal_termination_intent(termination_intent_))
    {
      return {GoalActivationAuthorityStatus::kMalformedState, std::nullopt};
    }
    if (phase_ == GoalSlotPhase::kIdle) {
      return {GoalActivationAuthorityStatus::kInactive, std::nullopt};
    }
    if (!goal_id_) {
      return {GoalActivationAuthorityStatus::kMalformedState, std::nullopt};
    }
    if (!matches(*goal_id_, generation_, goal_id, generation)) {
      return {GoalActivationAuthorityStatus::kGoalMismatch, std::nullopt};
    }
    if (phase_ != GoalSlotPhase::kPendingAcceptance) {
      return {GoalActivationAuthorityStatus::kWrongPhase, std::nullopt};
    }
    const bool valid_termination_backing =
      (unpublished_termination_record_ && !first_termination_record_) ||
      (!unpublished_termination_record_ && first_termination_record_);
    if (!valid_termination_backing || mutation_submission_) {
      return {GoalActivationAuthorityStatus::kMalformedState, std::nullopt};
    }
    return {
      GoalActivationAuthorityStatus::kObserved,
      GoalActivationAuthority{
        phase_, *goal_id_, generation_, inhibited_, cancel_requested_, safe_abort_requested_,
        task_deadline_exceeded_, mutation_submission_.has_value(), termination_intent_,
        first_termination_record_}};
  } catch (...) {
    return {GoalActivationAuthorityStatus::kSynchronizationFailed, std::nullopt};
  }
}

MotionAdmissionRegistrationDecision GoalAdmissionSlot::register_motion_workflow(
  std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
  const MotionLedgerIssuerBinding & ledger_issuer_binding) noexcept
{
  MotionAdmissionRegistrationDecision decision;
  if (!registration || !valid_motion_admission_registration_record(*registration) ||
    !ledger_issuer_binding.issuer_)
  {
    return decision;
  }

  {
    const std::uint64_t registration_goal_id_word_0 =
      goal_id_word(registration->goal_id(), 0U);
    const std::uint64_t registration_goal_id_word_1 =
      goal_id_word(registration->goal_id(), sizeof(registration_goal_id_word_0));
    std::scoped_lock lock(mutex_);
    if (phase_ == GoalSlotPhase::kIdle) {
      decision.status_ = MotionAdmissionRegistrationStatus::kInactive;
      return decision;
    }
    if (!goal_id_ || generation_ != registration->goal_generation() ||
      goal_id_word_0_ != registration_goal_id_word_0 ||
      goal_id_word_1_ != registration_goal_id_word_1)
    {
      decision.status_ = MotionAdmissionRegistrationStatus::kGoalMismatch;
      return decision;
    }
    if (phase_ != GoalSlotPhase::kActive) {
      decision.status_ = MotionAdmissionRegistrationStatus::kWrongPhase;
      return decision;
    }
    if (motion_workflow_active_) {
      if (!motion_registration_ || !motion_ledger_issuer_) {
        decision.status_ = MotionAdmissionRegistrationStatus::kSubmissionStatePresent;
      } else if (motion_ledger_issuer_.get() != ledger_issuer_binding.issuer_.get()) {
        decision.status_ = MotionAdmissionRegistrationStatus::kLedgerIssuerMismatch;
      } else {
        decision.status_ = MotionAdmissionRegistrationStatus::kWorkflowAlreadyActive;
      }
      return decision;
    }
    if (motion_registration_ || motion_ledger_issuer_ || motion_request_bound_ ||
      motion_request_identity_ || motion_submission_state_present_ ||
      motion_submission_commit_ || motion_runtime_reservation_issuer_ ||
      motion_runtime_reservation_session_generation_ != 0U ||
      motion_runtime_authorization_generation_ != 0U || mutation_submission_)
    {
      decision.status_ = MotionAdmissionRegistrationStatus::kSubmissionStatePresent;
      return decision;
    }
    if (first_termination_record_) {
      decision.first_termination_ = first_termination_record_;
      decision.status_ = MotionAdmissionRegistrationStatus::kTerminationWon;
      return decision;
    }
    if (!unpublished_termination_record_) {
      return decision;
    }

    decision.receipt_.emplace(
      MotionAdmissionRegistrationReceipt{
        motion_admission_issuer_, registration, ledger_issuer_binding.issuer_});
    motion_registration_ = std::move(registration);
    motion_ledger_issuer_ = ledger_issuer_binding.issuer_;
    motion_workflow_active_ = true;
    decision.status_ = MotionAdmissionRegistrationStatus::kRegistered;
  }
  return decision;
}

MotionAdmissionWithdrawalDecision GoalAdmissionSlot::withdraw_unbound_motion_workflow(
  MotionAdmissionRegistrationReceipt & receipt) noexcept
{
  MotionAdmissionWithdrawalDecision decision;
  const bool receipt_structure_valid =
    receipt.state_ == MotionAdmissionRegistrationReceipt::State::kLive &&
    receipt.issuer_ && receipt.registration_ && receipt.ledger_issuer_ &&
    valid_motion_admission_registration_record(*receipt.registration_);
  std::shared_ptr<const MotionAdmissionIssuerCore> retired_receipt_issuer;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> retired_receipt_registration;
  std::shared_ptr<const MotionLedgerIssuerCore> retired_receipt_ledger_issuer;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> retired_slot_registration;
  std::shared_ptr<const MotionLedgerIssuerCore> retired_slot_ledger_issuer;
  std::shared_ptr<const MotionOperationIdentity> retired_request_identity;
  std::shared_ptr<const MotionSubmissionCommitRecord> retired_submission_commit;

  {
    std::scoped_lock lock(mutex_);
    if (receipt.state_ == MotionAdmissionRegistrationReceipt::State::kWithdrawn) {
      decision.status_ = MotionAdmissionWithdrawalStatus::kAlreadyWithdrawn;
      return decision;
    }
    if (receipt.state_ != MotionAdmissionRegistrationReceipt::State::kLive ||
      !receipt_structure_valid ||
      !receipt.issuer_ || receipt.issuer_.get() != motion_admission_issuer_.get() ||
      !receipt.registration_ || !receipt.ledger_issuer_)
    {
      decision.status_ = MotionAdmissionWithdrawalStatus::kReceiptMismatch;
      return decision;
    }
    if (!motion_workflow_active_ || !motion_registration_ || !motion_ledger_issuer_ ||
      receipt.registration_.get() != motion_registration_.get() ||
      receipt.ledger_issuer_.get() != motion_ledger_issuer_.get() ||
      phase_ == GoalSlotPhase::kIdle || !goal_id_)
    {
      decision.status_ = MotionAdmissionWithdrawalStatus::kRegistrationMismatch;
      return decision;
    }
    if (motion_request_bound_ || motion_request_identity_ ||
      !receipt.request_binding_permit_live_)
    {
      decision.status_ = MotionAdmissionWithdrawalStatus::kAlreadyRequestBound;
      return decision;
    }
    if (motion_submission_state_present_ || motion_submission_commit_ || mutation_submission_) {
      decision.status_ = MotionAdmissionWithdrawalStatus::kSubmissionStatePresent;
      return decision;
    }

    decision.first_termination_ = first_termination_record_;
    retired_receipt_issuer = std::move(receipt.issuer_);
    retired_receipt_registration = std::move(receipt.registration_);
    retired_receipt_ledger_issuer = std::move(receipt.ledger_issuer_);
    receipt.state_ = MotionAdmissionRegistrationReceipt::State::kWithdrawn;
    receipt.request_binding_permit_live_ = false;
    retired_slot_registration = std::move(motion_registration_);
    retired_slot_ledger_issuer = std::move(motion_ledger_issuer_);
    retired_request_identity = std::move(motion_request_identity_);
    retired_submission_commit = std::move(motion_submission_commit_);
    motion_workflow_active_ = false;
    decision.status_ = MotionAdmissionWithdrawalStatus::kWithdrawn;
  }
  return decision;
}

MotionAdmissionRoutingOwnershipStatus GoalAdmissionSlot::observe_motion_routing_ownership(
  const CoordinatorGoalId & goal_id, GoalGeneration generation,
  const OperationTicket * pending_operation,
  const MotionLedgerIssuerBinding & ledger_issuer_binding) const noexcept
{
  if (!valid_goal_id(goal_id) || !admissible_goal_generation(generation) ||
    !ledger_issuer_binding.issuer_)
  {
    return MotionAdmissionRoutingOwnershipStatus::kInvalidArgument;
  }
  const std::uint64_t requested_goal_word_0 = goal_id_word(goal_id, 0U);
  const std::uint64_t requested_goal_word_1 =
    goal_id_word(goal_id, sizeof(requested_goal_word_0));

  try {
    std::scoped_lock lock(mutex_);
    if (!valid_goal_slot_phase(phase_) ||
      !valid_goal_termination_intent(termination_intent_))
    {
      return MotionAdmissionRoutingOwnershipStatus::kMalformedState;
    }
    const bool auxiliary_ownership = motion_registration_ || motion_ledger_issuer_ ||
      motion_request_bound_ || motion_request_identity_ || motion_submission_state_present_ ||
      motion_submission_commit_ ||
      motion_runtime_reservation_issuer_ ||
      motion_runtime_reservation_session_generation_ != 0U ||
      motion_runtime_authorization_generation_ != 0U;
    if (!motion_workflow_active_) {
      return auxiliary_ownership ? MotionAdmissionRoutingOwnershipStatus::kMalformedState :
             MotionAdmissionRoutingOwnershipStatus::kNotOwned;
    }
    if (phase_ != GoalSlotPhase::kActive || !goal_id_) {
      return MotionAdmissionRoutingOwnershipStatus::kInactive;
    }
    if (generation_ != generation || goal_id_word_0_ != requested_goal_word_0 ||
      goal_id_word_1_ != requested_goal_word_1)
    {
      return MotionAdmissionRoutingOwnershipStatus::kGoalMismatch;
    }
    if (!motion_registration_ || !motion_ledger_issuer_ ||
      !valid_motion_admission_registration_record(*motion_registration_) ||
      mutation_submission_)
    {
      return MotionAdmissionRoutingOwnershipStatus::kMalformedState;
    }
    const bool request_binding_shape_valid = motion_request_bound_ ?
      motion_request_identity_ &&
      motion_request_identity_->ticket == motion_registration_->operation_ticket() &&
      motion_request_identity_->execution_attempt_generation ==
      motion_registration_->execution_attempt_generation() :
      !motion_request_identity_;
    const bool runtime_reservation_shape_valid = motion_submission_state_present_ ?
      motion_runtime_reservation_issuer_ &&
      motion_runtime_reservation_session_generation_ ==
      motion_registration_->session_generation() :
      !motion_runtime_reservation_issuer_ &&
      motion_runtime_reservation_session_generation_ == 0U &&
      motion_runtime_authorization_generation_ == 0U;
    if (!request_binding_shape_valid || !runtime_reservation_shape_valid ||
      (motion_submission_commit_ && !motion_submission_state_present_))
    {
      return MotionAdmissionRoutingOwnershipStatus::kMalformedState;
    }
    if (motion_ledger_issuer_.get() != ledger_issuer_binding.issuer_.get()) {
      return MotionAdmissionRoutingOwnershipStatus::kLedgerIssuerMismatch;
    }
    if (motion_registration_->goal_generation() != generation ||
      motion_registration_->goal_id() != goal_id)
    {
      return MotionAdmissionRoutingOwnershipStatus::kMalformedState;
    }
    if (!pending_operation || motion_registration_->operation_ticket() != *pending_operation) {
      return MotionAdmissionRoutingOwnershipStatus::kOperationMismatch;
    }
    return MotionAdmissionRoutingOwnershipStatus::kOwned;
  } catch (...) {
    return MotionAdmissionRoutingOwnershipStatus::kSynchronizationFailed;
  }
}

MotionSubmissionAuthorizationIssuerConfigurationStatus
GoalAdmissionSlot::configure_motion_submission_authorization_issuer(
  const MotionSubmissionAuthorizationIssuerBinding & issuer) noexcept
{
  if (!issuer.issuer_) {
    return MotionSubmissionAuthorizationIssuerConfigurationStatus::kInvalidIssuer;
  }
  std::scoped_lock lock(mutex_);
  if (!motion_submission_authorization_issuer_) {
    motion_submission_authorization_issuer_ = issuer.issuer_;
    return MotionSubmissionAuthorizationIssuerConfigurationStatus::kConfigured;
  }
  return motion_submission_authorization_issuer_.get() == issuer.issuer_.get() ?
         MotionSubmissionAuthorizationIssuerConfigurationStatus::kAlreadyConfigured :
         MotionSubmissionAuthorizationIssuerConfigurationStatus::kIssuerMismatch;
}

MotionAdmissionIssuerBinding GoalAdmissionSlot::motion_admission_issuer_binding() const noexcept
{
  return MotionAdmissionIssuerBinding{motion_admission_issuer_};
}

std::optional<MotionAdmissionRuntimeReservationWitness>
GoalAdmissionSlot::begin_runtime_submission_reservation(
  const MotionAdmissionRegistrationReceipt & receipt,
  const MotionSubmissionAuthorizationIssuerBinding & runtime_issuer) noexcept
{
  const bool receipt_structure_valid =
    receipt.state_ == MotionAdmissionRegistrationReceipt::State::kLive &&
    receipt.issuer_ && receipt.registration_ && receipt.ledger_issuer_ &&
    valid_motion_admission_registration_record(*receipt.registration_);
  if (!receipt_structure_valid || !runtime_issuer.issuer_) {
    return std::nullopt;
  }

  std::scoped_lock lock(mutex_);
  if (!motion_submission_authorization_issuer_ ||
    motion_submission_authorization_issuer_.get() != runtime_issuer.issuer_.get() ||
    receipt.issuer_.get() != motion_admission_issuer_.get() ||
    !motion_workflow_active_ || !motion_registration_ || !motion_ledger_issuer_ ||
    receipt.registration_.get() != motion_registration_.get() ||
    receipt.ledger_issuer_.get() != motion_ledger_issuer_.get() ||
    motion_submission_state_present_ || motion_submission_commit_ ||
    motion_runtime_reservation_issuer_ ||
    motion_runtime_reservation_session_generation_ != 0U ||
    motion_runtime_authorization_generation_ != 0U ||
    phase_ != GoalSlotPhase::kActive || !ready_ || inhibited_ || first_termination_record_)
  {
    return std::nullopt;
  }

  motion_runtime_reservation_issuer_ = runtime_issuer.issuer_;
  motion_runtime_reservation_session_generation_ =
    receipt.registration_->session_generation();
  motion_submission_state_present_ = true;
  return MotionAdmissionRuntimeReservationWitness{
    motion_admission_issuer_, motion_registration_, motion_ledger_issuer_,
    motion_runtime_reservation_issuer_};
}

bool GoalAdmissionSlot::release_unissued_runtime_submission_reservation(
  MotionAdmissionRuntimeReservationWitness & witness) noexcept
{
  const bool witness_structure_valid =
    witness.state_ == MotionAdmissionRuntimeReservationWitness::State::kLive &&
    witness.admission_issuer_ && witness.registration_ && witness.ledger_issuer_ &&
    witness.runtime_issuer_ &&
    valid_motion_admission_registration_record(*witness.registration_);
  if (!witness_structure_valid) {
    return false;
  }

  std::shared_ptr<const MotionAdmissionIssuerCore> retired_witness_admission_issuer;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> retired_witness_registration;
  std::shared_ptr<const MotionLedgerIssuerCore> retired_witness_ledger_issuer;
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> retired_witness_runtime_issuer;
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> retired_runtime_issuer;
  {
    std::scoped_lock lock(mutex_);
    if (!motion_submission_authorization_issuer_ ||
      motion_submission_authorization_issuer_.get() != witness.runtime_issuer_.get() ||
      witness.admission_issuer_.get() != motion_admission_issuer_.get() ||
      !motion_workflow_active_ || !motion_registration_ || !motion_ledger_issuer_ ||
      witness.registration_.get() != motion_registration_.get() ||
      witness.ledger_issuer_.get() != motion_ledger_issuer_.get() ||
      !motion_submission_state_present_ || !motion_runtime_reservation_issuer_ ||
      motion_submission_commit_ ||
      motion_runtime_reservation_issuer_.get() != witness.runtime_issuer_.get() ||
      motion_runtime_reservation_session_generation_ !=
      witness.registration_->session_generation() ||
      motion_runtime_authorization_generation_ != 0U)
    {
      return false;
    }

    retired_witness_admission_issuer = std::move(witness.admission_issuer_);
    retired_witness_registration = std::move(witness.registration_);
    retired_witness_ledger_issuer = std::move(witness.ledger_issuer_);
    retired_witness_runtime_issuer = std::move(witness.runtime_issuer_);
    witness.state_ = MotionAdmissionRuntimeReservationWitness::State::kReleased;
    retired_runtime_issuer = std::move(motion_runtime_reservation_issuer_);
    motion_runtime_reservation_session_generation_ = 0U;
    motion_submission_state_present_ = false;
  }
  return true;
}

bool GoalAdmissionSlot::seal_runtime_submission_authorization(
  MotionAdmissionRuntimeReservationWitness & witness,
  std::uint64_t authorization_generation) noexcept
{
  const bool witness_structure_valid =
    witness.state_ == MotionAdmissionRuntimeReservationWitness::State::kLive &&
    witness.admission_issuer_ && witness.registration_ && witness.ledger_issuer_ &&
    witness.runtime_issuer_ &&
    valid_motion_admission_registration_record(*witness.registration_);
  if (!witness_structure_valid || authorization_generation == 0U) {
    return false;
  }

  std::scoped_lock lock(mutex_);
  if (!motion_submission_authorization_issuer_ ||
    motion_submission_authorization_issuer_.get() != witness.runtime_issuer_.get() ||
    witness.admission_issuer_.get() != motion_admission_issuer_.get() ||
    !motion_workflow_active_ || !motion_registration_ || !motion_ledger_issuer_ ||
    witness.registration_.get() != motion_registration_.get() ||
    witness.ledger_issuer_.get() != motion_ledger_issuer_.get() ||
    !motion_submission_state_present_ || !motion_runtime_reservation_issuer_ ||
    motion_submission_commit_ ||
    motion_runtime_reservation_issuer_.get() != witness.runtime_issuer_.get() ||
    motion_runtime_reservation_session_generation_ !=
    witness.registration_->session_generation() ||
    motion_runtime_authorization_generation_ != 0U ||
    phase_ != GoalSlotPhase::kActive || !ready_ || inhibited_ || first_termination_record_)
  {
    return false;
  }
  motion_runtime_authorization_generation_ = authorization_generation;
  witness.state_ = MotionAdmissionRuntimeReservationWitness::State::kAuthorizationSealed;
  return true;
}

bool GoalAdmissionSlot::begin_completion_cleanup(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation)
{
  std::scoped_lock lock(mutex_);
  if (phase_ != GoalSlotPhase::kActive || !goal_id_ ||
    !matches(*goal_id_, generation_, goal_id, goal_generation))
  {
    return false;
  }
  completion_cleanup_ = true;
  return true;
}

bool GoalAdmissionSlot::begin_skip_cleanup(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation)
{
  std::scoped_lock lock(mutex_);
  if (phase_ != GoalSlotPhase::kActive || !goal_id_ ||
    !matches(*goal_id_, generation_, goal_id, goal_generation))
  {
    return false;
  }
  skip_cleanup_ = true;
  return true;
}

MutationSubmissionResult GoalAdmissionSlot::commit_mutation_submission(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  OperationGeneration operation_generation, CoordinatorMutationKind kind,
  SteadyTime now, SteadyTime absolute_deadline)
{
  std::scoped_lock lock(mutex_);
  if (operation_generation == 0 || !valid_mutation_kind(kind)) {
    return {MutationSubmissionDecision::kInvalidArgument, std::nullopt};
  }
  if (phase_ != GoalSlotPhase::kActive) {
    return {MutationSubmissionDecision::kInactive, std::nullopt};
  }
  if (!goal_id_ || !matches(*goal_id_, generation_, goal_id, goal_generation)) {
    return {MutationSubmissionDecision::kGoalMismatch, std::nullopt};
  }
  if (inhibited_) {
    return {MutationSubmissionDecision::kInhibited, std::nullopt};
  }
  if (now >= absolute_deadline) {
    return {MutationSubmissionDecision::kDeadlineExceeded, std::nullopt};
  }
  if (operation_generation <= last_resolved_operation_generation_) {
    return {MutationSubmissionDecision::kStaleOperation, std::nullopt};
  }
  if (mutation_submission_) {
    const bool exact = mutation_submission_->operation_generation == operation_generation &&
      mutation_submission_->kind == kind;
    if (!exact) {
      return {MutationSubmissionDecision::kAlreadyPending, mutation_submission_};
    }
    if (mutation_submission_->phase == MutationSubmissionPhase::kCommitted) {
      return {MutationSubmissionDecision::kReplayCommitted, mutation_submission_};
    }
  }
  if (motion_workflow_active_ || motion_registration_ || motion_ledger_issuer_ ||
    motion_request_bound_ || motion_request_identity_ || motion_submission_state_present_ ||
    motion_submission_commit_ ||
    motion_runtime_reservation_issuer_ ||
    motion_runtime_reservation_session_generation_ != 0U ||
    motion_runtime_authorization_generation_ != 0U)
  {
    return {MutationSubmissionDecision::kAlreadyPending, mutation_submission_};
  }

  const bool terminating = termination_intent_ != GoalTerminationIntent::kNone;
  const bool cleanup = kind == CoordinatorMutationKind::kReleaseTask;
  if (terminating && !cleanup) {
    return {MutationSubmissionDecision::kTerminationRequested, mutation_submission_};
  }
  // Reservation release is a cleanup mutation: allowed only during teardown, after the reserved
  // transfer finished, or under a claimed recoverable-skip teardown (Card 051).
  if (!terminating && !completion_cleanup_ && !skip_cleanup_ && cleanup) {
    return {MutationSubmissionDecision::kCleanupWithoutTermination, mutation_submission_};
  }

  mutation_submission_ = MutationSubmissionRecord{
    operation_generation, kind, MutationSubmissionPhase::kCommitted};
  return {MutationSubmissionDecision::kCommitted, mutation_submission_};
}

bool GoalAdmissionSlot::confirm_mutation_not_submitted(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  OperationGeneration operation_generation, CoordinatorMutationKind kind)
{
  std::scoped_lock lock(mutex_);
  if (phase_ != GoalSlotPhase::kActive || !goal_id_ ||
    !matches(*goal_id_, generation_, goal_id, goal_generation) || !mutation_submission_ ||
    mutation_submission_->operation_generation != operation_generation ||
    mutation_submission_->kind != kind ||
    mutation_submission_->phase != MutationSubmissionPhase::kCommitted)
  {
    return false;
  }
  mutation_submission_->phase = MutationSubmissionPhase::kConfirmedNotSubmitted;
  return true;
}

bool GoalAdmissionSlot::resolve_mutation_submission(
  const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
  OperationGeneration operation_generation, CoordinatorMutationKind kind)
{
  std::scoped_lock lock(mutex_);
  if (phase_ != GoalSlotPhase::kActive || !goal_id_ ||
    !matches(*goal_id_, generation_, goal_id, goal_generation) || !mutation_submission_ ||
    mutation_submission_->operation_generation != operation_generation ||
    mutation_submission_->kind != kind)
  {
    return false;
  }
  last_resolved_operation_generation_ = std::max(
    last_resolved_operation_generation_, operation_generation);
  mutation_submission_.reset();
  return true;
}

bool GoalAdmissionSlot::finish(
  const CoordinatorGoalId & goal_id, GoalGeneration generation)
{
  std::shared_ptr<FirstGoalTerminationRecord> retired_unpublished_termination;
  std::shared_ptr<const FirstGoalTerminationRecord> retired_first_termination;
  std::shared_ptr<const MotionOperationIdentity> retired_motion_request_identity;
  std::shared_ptr<const MotionSubmissionCommitRecord> retired_motion_submission_commit;
  {
    std::scoped_lock lock(mutex_);
    if (phase_ == GoalSlotPhase::kIdle || !goal_id_ || mutation_submission_ ||
      motion_workflow_active_ || motion_registration_ || motion_ledger_issuer_ ||
      motion_request_bound_ || motion_request_identity_ || motion_submission_state_present_ ||
      motion_submission_commit_ ||
      motion_runtime_reservation_issuer_ ||
      motion_runtime_reservation_session_generation_ != 0U ||
      motion_runtime_authorization_generation_ != 0U ||
      !matches(*goal_id_, generation_, goal_id, generation))
    {
      return false;
    }
    phase_ = GoalSlotPhase::kIdle;
    cancel_requested_ = false;
    safe_abort_requested_ = false;
    task_deadline_exceeded_ = false;
    termination_intent_ = GoalTerminationIntent::kNone;
    completion_cleanup_ = false;
    skip_cleanup_ = false;
    generation_ = 0;
    goal_id_word_0_ = 0U;
    goal_id_word_1_ = 0U;
    last_resolved_operation_generation_ = 0;
    goal_id_.reset();
    retired_unpublished_termination = std::move(unpublished_termination_record_);
    retired_first_termination = std::move(first_termination_record_);
    retired_motion_request_identity = std::move(motion_request_identity_);
    retired_motion_submission_commit = std::move(motion_submission_commit_);
  }
  return true;
}

GoalAdmissionSnapshot GoalAdmissionSlot::snapshot() const
{
  std::scoped_lock lock(mutex_);
  return {
    phase_, ready_, inhibited_, cancel_requested_, safe_abort_requested_,
    task_deadline_exceeded_, termination_intent_, generation_, goal_id_, mutation_submission_,
    first_termination_record_, readiness_detail_, inhibition_detail_, motion_workflow_active_,
    motion_registration_, motion_request_bound_, motion_request_identity_,
    motion_submission_state_present_, motion_submission_commit_,
    static_cast<bool>(motion_submission_authorization_issuer_),
    consumed_motion_authorization_generation_high_water_};
}

MotionTimeoutCheckResult::MotionTimeoutCheckResult(
  MotionOperationIdentity exact_identity,
  MotionTimeoutAuthorityKind authority_kind,
  std::optional<PendingOperationPhase> pre_phase,
  std::optional<PendingOperationPhase> post_phase,
  OperationTimeoutDisposition disposition,
  MotionTimeoutCause cause,
  bool newly_effective,
  std::optional<SteadyTime> nominal_deadline,
  std::optional<SteadyTime> first_effective_at,
  SteadyTime checked_at) noexcept
: exact_identity_(std::move(exact_identity)),
  authority_kind_(authority_kind),
  pre_phase_(pre_phase),
  post_phase_(post_phase),
  disposition_(disposition),
  cause_(cause),
  newly_effective_(newly_effective),
  nominal_deadline_(nominal_deadline),
  first_effective_at_(first_effective_at),
  checked_at_(checked_at)
{
}

MotionTimeoutCheckDecision::MotionTimeoutCheckDecision(
  MotionTimeoutCheckDecision && other) noexcept
: status_(other.status_), result_(std::move(other.result_))
{
  other.result_.reset();
  other.status_ = MotionLedgerStatus::kInvalidArgument;
}

MotionLedgerStatus MotionTimeoutCheckDecision::status() const noexcept
{
  return status_;
}

const MotionTimeoutCheckResult * MotionTimeoutCheckDecision::result() const noexcept
{
  return status_ == MotionLedgerStatus::kApplied && result_ ? &*result_ : nullptr;
}

std::optional<MotionTimeoutCheckResult> MotionTimeoutCheckDecision::take_result() noexcept
{
  std::optional<MotionTimeoutCheckResult> result{std::move(result_)};
  result_.reset();
  status_ = MotionLedgerStatus::kInvalidArgument;
  return result;
}

PendingOperationLedger::PendingOperationLedger(
  ReconciliationPolicy policy,
  MotionTimeoutFailureInjector motion_timeout_failure_injector,
  MotionLedgerClearanceConfig clearance_config)
: issuer_(std::make_shared<const MotionLedgerIssuerCore>()),
  clearance_generation_limit_(clearance_config.generation_limit),
  clearance_failure_injector_(std::move(clearance_config.failure_injector)),
  policy_(policy),
  motion_timeout_failure_injector_(std::move(motion_timeout_failure_injector))
{
  static_assert(std::is_nothrow_move_constructible_v<PendingRecord>);
  static_assert(std::is_nothrow_move_constructible_v<std::optional<PendingRecord>>);
  static_assert(std::is_nothrow_destructible_v<PendingRecord>);
  static_assert(std::is_nothrow_destructible_v<std::optional<PendingRecord>>);
  static_assert(noexcept(std::declval<std::optional<PendingRecord> &>().reset()));
  static_assert(
    noexcept(
      std::declval<MotionLedgerClearanceGeneration &>() =
      std::declval<MotionLedgerClearanceGeneration>()));
  policy_valid_ = policy_.window.count() > 0 && policy_.attempt_timeout.count() > 0 &&
    policy_.max_attempts > 0U;
}

MotionLedgerIssuerBinding PendingOperationLedger::issuer_binding() const noexcept
{
  return MotionLedgerIssuerBinding{issuer_};
}

MotionLedgerClearancePreparationDecision PendingOperationLedger::prepare_motion_clearance(
  const DefinitelyNotSubmittedRecord & record) const
{
  MotionLedgerClearancePreparationDecision decision;
  if (invalid_definitely_not_submitted_record_detail(record)) {
    return decision;
  }
  {
    std::scoped_lock lock(mutex_);
    if (clearance_failure_injector_) {
      clearance_failure_injector_(
        MotionLedgerClearanceFailurePoint::kBeforeRecordStorageMaterialization);
    }
  }
  auto backing = std::make_shared<const MotionLedgerClearanceRecord>(
    std::in_place_type<DefinitelyNotSubmittedRecord>, record);
  auto preparation = PreparedMotionLedgerClearance{
    issuer_, std::move(backing), MotionLedgerClearanceKind::kNonSubmission};
  decision.preparation_.emplace(std::move(preparation));
  decision.status_ = MotionLedgerClearancePreparationStatus::kPrepared;
  return decision;
}

MotionLedgerClearancePreparationDecision PendingOperationLedger::prepare_motion_clearance(
  const MotionStopProofRecord & record) const
{
  MotionLedgerClearancePreparationDecision decision;
  if (invalid_motion_stop_proof_record_detail(record)) {
    return decision;
  }
  {
    std::scoped_lock lock(mutex_);
    if (clearance_failure_injector_) {
      clearance_failure_injector_(
        MotionLedgerClearanceFailurePoint::kBeforeRecordStorageMaterialization);
    }
  }
  auto backing = std::make_shared<const MotionLedgerClearanceRecord>(
    std::in_place_type<MotionStopProofRecord>, record);
  auto preparation = PreparedMotionLedgerClearance{
    issuer_, std::move(backing), MotionLedgerClearanceKind::kStopped};
  decision.preparation_.emplace(std::move(preparation));
  decision.status_ = MotionLedgerClearancePreparationStatus::kPrepared;
  return decision;
}

OperationStartResult PendingOperationLedger::start(
  GoalGeneration goal_generation, RestockTaskCommand command, OperationEffect effect,
  std::string operation_id, SteadyTime now, SteadyTime absolute_deadline)
{
  std::scoped_lock lock(mutex_);
  if (pending_) {
    return {OperationStartError::kAlreadyPending, std::nullopt};
  }
  if (!policy_valid_ || !admissible_goal_generation(goal_generation) ||
    command == RestockTaskCommand::kNone || !valid_operation_effect(effect) ||
    !valid_generic_operation_pair(command, effect) || now >= absolute_deadline ||
    (effect != OperationEffect::kReadOnly && operation_id.empty()) ||
    next_generation_ == std::numeric_limits<OperationGeneration>::max())
  {
    return {OperationStartError::kInvalidArgument, std::nullopt};
  }
  OperationTicket ticket{
    goal_generation, next_generation_++, command, effect, std::move(operation_id),
    absolute_deadline};
  PendingRecord record;
  record.ticket = ticket;
  record.phase = PendingOperationPhase::kAwaitingCompletion;
  record.next_reconciliation_kind =
    command == RestockTaskCommand::kReleaseTaskReservation ? ReconciliationKind::kReadback :
    ReconciliationKind::kReplayMutation;
  pending_ = std::move(record);
  return {OperationStartError::kNone, std::move(ticket)};
}

PendingOperationLedger::MotionTimeoutPlanDecision
PendingOperationLedger::plan_motion_timeout_locked(SteadyTime captured_at) const noexcept
{
  static_assert(std::is_nothrow_move_constructible_v<MotionTimeoutTransitionPlan>);
  if (!pending_ || !pending_->motion_binding || !pending_->motion_deadlines ||
    pending_->phase == PendingOperationPhase::kAwaitingRequestBinding)
  {
    return {MotionLedgerStatus::kWrongPhase, std::nullopt};
  }

  const auto & deadlines = *pending_->motion_deadlines;
  const auto deadline_for = [&deadlines](MotionTimeoutCause cause) -> std::optional<SteadyTime> {
    switch (cause) {
      case MotionTimeoutCause::kGoalResponseDeadline:
        return deadlines.goal_response;
      case MotionTimeoutCause::kExpectedResultDeadline:
        return deadlines.expected_result;
      case MotionTimeoutCause::kTerminalDrainDeadline:
        return deadlines.terminal_drain;
      case MotionTimeoutCause::kStopProofDeadline:
        return deadlines.stop_proof;
      case MotionTimeoutCause::kNone:
        return std::nullopt;
    }
    return std::nullopt;
  };
  const auto strength = [](MotionTimeoutCause cause) noexcept -> std::uint8_t {
    switch (cause) {
      case MotionTimeoutCause::kGoalResponseDeadline: return 1U;
      case MotionTimeoutCause::kExpectedResultDeadline: return 2U;
      case MotionTimeoutCause::kTerminalDrainDeadline: return 3U;
      case MotionTimeoutCause::kStopProofDeadline: return 4U;
      case MotionTimeoutCause::kNone: return 0U;
    }
    return 0U;
  };

  if (pending_->effective_motion_timeout) {
    const auto & effective = *pending_->effective_motion_timeout;
    const auto nominal = deadline_for(effective.cause);
    if (!nominal || effective.cause == MotionTimeoutCause::kNone ||
      effective.nominal_deadline != *nominal ||
      effective.first_effective_at < effective.nominal_deadline)
    {
      return {MotionLedgerStatus::kEvidenceMismatch, std::nullopt};
    }
    const bool goal_latched = pending_->goal_response_deadline_expired;
    const bool both_latched = goal_latched && pending_->public_deadline_expired;
    if ((effective.cause == MotionTimeoutCause::kGoalResponseDeadline && !goal_latched) ||
      ((effective.cause == MotionTimeoutCause::kExpectedResultDeadline ||
      effective.cause == MotionTimeoutCause::kTerminalDrainDeadline ||
      effective.cause == MotionTimeoutCause::kStopProofDeadline) && !both_latched))
    {
      return {MotionLedgerStatus::kEvidenceMismatch, std::nullopt};
    }
  }

  const auto cause = pending_->effective_motion_timeout ?
    pending_->effective_motion_timeout->cause : MotionTimeoutCause::kNone;
  const bool invalid_phase_shape =
    (pending_->phase == PendingOperationPhase::kAwaitingCompletion &&
    cause != MotionTimeoutCause::kNone) ||
    (pending_->phase == PendingOperationPhase::kCancelAndVerify &&
    (cause == MotionTimeoutCause::kTerminalDrainDeadline ||
    cause == MotionTimeoutCause::kStopProofDeadline)) ||
    (pending_->phase == PendingOperationPhase::kAwaitingStopProof &&
    (cause == MotionTimeoutCause::kTerminalDrainDeadline ||
    cause == MotionTimeoutCause::kStopProofDeadline)) ||
    (pending_->phase != PendingOperationPhase::kAwaitingCompletion &&
    pending_->phase != PendingOperationPhase::kCancelAndVerify &&
    pending_->phase != PendingOperationPhase::kAwaitingStopProof &&
    pending_->phase != PendingOperationPhase::kMotionLivenessUnknown);
  if (invalid_phase_shape) {
    return {MotionLedgerStatus::kWrongPhase, std::nullopt};
  }

  const auto capture_precedes = [captured_at](SteadyTime predecessor) noexcept {
    return captured_at < predecessor;
  };
  if (capture_precedes(deadlines.submission) ||
    (pending_->effective_motion_timeout &&
    capture_precedes(pending_->effective_motion_timeout->first_effective_at)) ||
    (pending_->accepted_goal &&
    capture_precedes(pending_->accepted_goal->ingress.arrived_at)) ||
    (pending_->cancel_submission &&
    capture_precedes(pending_->cancel_submission->ingress.arrived_at)) ||
    (pending_->post_terminal_cancel_submission &&
    capture_precedes(pending_->post_terminal_cancel_submission->ingress.arrived_at)) ||
    (pending_->terminal && capture_precedes(pending_->terminal->ingress.arrived_at)) ||
    (pending_->late_non_submission &&
    capture_precedes(pending_->late_non_submission->ingress.arrived_at)) ||
    (pending_->ingress_fault &&
    pending_->ingress_fault->kind != MotionIngressFaultKind::kClockRegression &&
    capture_precedes(pending_->ingress_fault->observed_at)) ||
    (pending_->late_stop_proof &&
    capture_precedes(pending_->late_stop_proof->final_ingress.arrived_at)))
  {
    return {MotionLedgerStatus::kInvalidArgument, std::nullopt};
  }

  MotionTimeoutTransitionPlan plan;
  plan.pre_phase = pending_->phase;
  plan.post_phase = pending_->phase;
  plan.goal_response_deadline_expired = pending_->goal_response_deadline_expired;
  plan.public_deadline_expired = pending_->public_deadline_expired;
  plan.liveness_was_unknown = pending_->liveness_was_unknown;
  plan.effective_motion_timeout = pending_->effective_motion_timeout;

  MotionTimeoutCause candidate = MotionTimeoutCause::kNone;
  switch (pending_->phase) {
    case PendingOperationPhase::kAwaitingCompletion:
      if (captured_at >= deadlines.terminal_drain) {
        plan.goal_response_deadline_expired = true;
        plan.public_deadline_expired = true;
        plan.liveness_was_unknown = true;
        plan.post_phase = PendingOperationPhase::kMotionLivenessUnknown;
        candidate = MotionTimeoutCause::kTerminalDrainDeadline;
      } else if (captured_at >= deadlines.expected_result) {
        plan.goal_response_deadline_expired = true;
        plan.public_deadline_expired = true;
        plan.post_phase = PendingOperationPhase::kCancelAndVerify;
        candidate = MotionTimeoutCause::kExpectedResultDeadline;
      } else if (!pending_->accepted_goal && captured_at >= deadlines.goal_response) {
        plan.goal_response_deadline_expired = true;
        plan.post_phase = PendingOperationPhase::kCancelAndVerify;
        candidate = MotionTimeoutCause::kGoalResponseDeadline;
      }
      break;
    case PendingOperationPhase::kCancelAndVerify:
      if (captured_at >= deadlines.terminal_drain) {
        plan.goal_response_deadline_expired = true;
        plan.public_deadline_expired = true;
        plan.liveness_was_unknown = true;
        plan.post_phase = PendingOperationPhase::kMotionLivenessUnknown;
        candidate = MotionTimeoutCause::kTerminalDrainDeadline;
      } else if (captured_at >= deadlines.expected_result) {
        plan.goal_response_deadline_expired = true;
        plan.public_deadline_expired = true;
        candidate = MotionTimeoutCause::kExpectedResultDeadline;
      }
      break;
    case PendingOperationPhase::kAwaitingStopProof:
      if (captured_at >= deadlines.stop_proof) {
        plan.goal_response_deadline_expired = true;
        plan.public_deadline_expired = true;
        plan.liveness_was_unknown = true;
        plan.post_phase = PendingOperationPhase::kMotionLivenessUnknown;
        candidate = MotionTimeoutCause::kStopProofDeadline;
      }
      break;
    case PendingOperationPhase::kMotionLivenessUnknown:
      break;
    default:
      return {MotionLedgerStatus::kWrongPhase, std::nullopt};
  }

  if (candidate != MotionTimeoutCause::kNone &&
    (!plan.effective_motion_timeout ||
    strength(candidate) > strength(plan.effective_motion_timeout->cause)))
  {
    plan.effective_motion_timeout = EffectiveMotionTimeout{
      candidate, *deadline_for(candidate), captured_at};
    plan.newly_effective = true;
  }

  const auto selected = plan.effective_motion_timeout ?
    plan.effective_motion_timeout->cause : MotionTimeoutCause::kNone;
  if (plan.post_phase == PendingOperationPhase::kMotionLivenessUnknown) {
    plan.disposition = selected == MotionTimeoutCause::kStopProofDeadline ?
      OperationTimeoutDisposition::kStopProofExpired :
      OperationTimeoutDisposition::kMotionLivenessUnknown;
    return {MotionLedgerStatus::kApplied, plan};
  }
  if (selected == MotionTimeoutCause::kGoalResponseDeadline ||
    selected == MotionTimeoutCause::kExpectedResultDeadline)
  {
    plan.disposition = OperationTimeoutDisposition::kCancelAndVerify;
    return {MotionLedgerStatus::kApplied, plan};
  }
  if (selected == MotionTimeoutCause::kNone) {
    plan.disposition = OperationTimeoutDisposition::kNotDue;
    return {MotionLedgerStatus::kApplied, plan};
  }
  return {MotionLedgerStatus::kWrongPhase, std::nullopt};
}

void PendingOperationLedger::apply_motion_timeout_plan_locked(
  const MotionTimeoutTransitionPlan & plan) noexcept
{
  pending_->phase = plan.post_phase;
  pending_->goal_response_deadline_expired = plan.goal_response_deadline_expired;
  pending_->public_deadline_expired = plan.public_deadline_expired;
  pending_->liveness_was_unknown = plan.liveness_was_unknown;
  pending_->effective_motion_timeout = plan.effective_motion_timeout;
}

MotionTimeoutCheckDecision PendingOperationLedger::check_motion_timeout(
  const MotionOperationIdentity & exact_live_identity,
  SteadyTime captured_at)
{
  const auto rejected = [](MotionLedgerStatus status) noexcept {
    MotionTimeoutCheckDecision decision;
    decision.status_ = status;
    return decision;
  };
  std::scoped_lock lock(mutex_);
  if (invalid_motion_operation_identity_detail(exact_live_identity)) {
    return rejected(MotionLedgerStatus::kInvalidArgument);
  }
  if (!pending_) {
    return rejected(
      exact_live_identity.ticket.operation_generation < next_generation_ ?
      MotionLedgerStatus::kStaleResolved : MotionLedgerStatus::kNoPendingOperation);
  }
  if (exact_live_identity.ticket.operation_generation <
    pending_->ticket.operation_generation)
  {
    return rejected(MotionLedgerStatus::kStaleResolved);
  }
  if (pending_->ticket.effect != OperationEffect::kCancelableMotion) {
    return rejected(MotionLedgerStatus::kWrongEffect);
  }
  if (pending_->ticket.goal_generation != exact_live_identity.ticket.goal_generation ||
    pending_->ticket.operation_generation !=
    exact_live_identity.ticket.operation_generation)
  {
    return rejected(MotionLedgerStatus::kCorrelationMismatch);
  }
  if (pending_->phase == PendingOperationPhase::kAwaitingRequestBinding ||
    !pending_->motion_binding)
  {
    return rejected(MotionLedgerStatus::kWrongPhase);
  }
  if (pending_->motion_binding->identity != exact_live_identity) {
    return rejected(MotionLedgerStatus::kCorrelationMismatch);
  }
  const auto planned = plan_motion_timeout_locked(captured_at);
  if (planned.status != MotionLedgerStatus::kApplied || !planned.plan) {
    return rejected(planned.status);
  }
  if (motion_timeout_failure_injector_) {
    motion_timeout_failure_injector_(
      MotionTimeoutFailurePoint::kBeforeResultIdentityMaterialization);
  }
  const auto & plan = *planned.plan;
  const auto selected = plan.effective_motion_timeout ?
    plan.effective_motion_timeout->cause : MotionTimeoutCause::kNone;
  auto result = MotionTimeoutCheckResult{
    pending_->motion_binding->identity,
    MotionTimeoutAuthorityKind::kLiveOperation,
    plan.pre_phase,
    plan.post_phase,
    plan.disposition,
    selected,
    plan.newly_effective,
    plan.effective_motion_timeout ?
    std::optional<SteadyTime>{plan.effective_motion_timeout->nominal_deadline} : std::nullopt,
    plan.effective_motion_timeout ?
    std::optional<SteadyTime>{plan.effective_motion_timeout->first_effective_at} : std::nullopt,
    captured_at};
  MotionTimeoutCheckDecision decision;
  decision.result_.emplace(std::move(result));
  decision.status_ = MotionLedgerStatus::kApplied;
  if (motion_timeout_failure_injector_) {
    motion_timeout_failure_injector_(
      MotionTimeoutFailurePoint::kAfterDecisionPreparationBeforeCommit);
  }
  apply_motion_timeout_plan_locked(plan);
  return decision;
}

MotionTimeoutCheckDecision PendingOperationLedger::check_cleared_motion_timeout(
  MotionLedgerClearanceProof & clearance_proof,
  SteadyTime captured_at)
{
  const auto rejected = [](MotionLedgerStatus status) noexcept {
    MotionTimeoutCheckDecision decision;
    decision.status_ = status;
    return decision;
  };
  std::scoped_lock lock(mutex_);

  switch (clearance_proof.state_) {
    case MotionLedgerClearanceProof::State::kMovedFrom:
      return rejected(MotionLedgerStatus::kInvalidArgument);
    case MotionLedgerClearanceProof::State::kLive:
      break;
    default:
      return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  if (!clearance_proof.issuer_ || !clearance_proof.record_ ||
    clearance_proof.generation_ == 0U)
  {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  if (clearance_proof.issuer_.get() != issuer_.get()) {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }

  const MotionOperationIdentity * exact_identity = nullptr;
  SteadyTime latest_clearing_ingress{};
  MotionTimeoutAuthorityKind authority_kind{MotionTimeoutAuthorityKind::kLiveOperation};
  switch (clearance_proof.kind_) {
    case MotionLedgerClearanceKind::kNonSubmission: {
        const auto * record = std::get_if<DefinitelyNotSubmittedRecord>(
          clearance_proof.record_.get());
        if (record == nullptr || invalid_definitely_not_submitted_record_detail(*record)) {
          return rejected(MotionLedgerStatus::kEvidenceMismatch);
        }
        exact_identity = &record->identity;
        latest_clearing_ingress = record->ingress.arrived_at;
        authority_kind = MotionTimeoutAuthorityKind::kClearedNonSubmission;
        break;
      }
    case MotionLedgerClearanceKind::kStopped: {
        const auto * record = std::get_if<MotionStopProofRecord>(
          clearance_proof.record_.get());
        if (record == nullptr || invalid_motion_stop_proof_record_detail(*record)) {
          return rejected(MotionLedgerStatus::kEvidenceMismatch);
        }
        exact_identity = &record->binding.identity;
        latest_clearing_ingress = record->final_ingress.arrived_at;
        authority_kind = MotionTimeoutAuthorityKind::kClearedStopped;
        break;
      }
    default:
      return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }

  if (clearance_proof.generation_ < last_issued_clearance_generation_) {
    return rejected(MotionLedgerStatus::kStaleResolved);
  }
  if (clearance_proof.generation_ > last_issued_clearance_generation_) {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  const auto operation_generation = exact_identity->ticket.operation_generation;
  if (operation_generation == std::numeric_limits<OperationGeneration>::max()) {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  const auto successor = operation_generation + 1U;
  if (successor > next_generation_) {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  if (successor < next_generation_) {
    return rejected(MotionLedgerStatus::kStaleResolved);
  }
  if (pending_) {
    return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  switch (clearance_proof.timeout_permit_state_) {
    case MotionLedgerClearanceProof::TimeoutPermitState::kConsumed:
      return rejected(MotionLedgerStatus::kAlreadyApplied);
    case MotionLedgerClearanceProof::TimeoutPermitState::kAvailable:
      break;
    default:
      return rejected(MotionLedgerStatus::kEvidenceMismatch);
  }
  if (captured_at < latest_clearing_ingress) {
    return rejected(MotionLedgerStatus::kInvalidArgument);
  }

  if (motion_timeout_failure_injector_) {
    motion_timeout_failure_injector_(
      MotionTimeoutFailurePoint::kBeforeResultIdentityMaterialization);
  }
  auto result = MotionTimeoutCheckResult{
    *exact_identity,
    authority_kind,
    std::nullopt,
    std::nullopt,
    OperationTimeoutDisposition::kNotDue,
    MotionTimeoutCause::kNone,
    false,
    std::nullopt,
    std::nullopt,
    captured_at};
  MotionTimeoutCheckDecision decision;
  decision.result_.emplace(std::move(result));
  decision.status_ = MotionLedgerStatus::kApplied;
  if (motion_timeout_failure_injector_) {
    motion_timeout_failure_injector_(
      MotionTimeoutFailurePoint::kAfterDecisionPreparationBeforeCommit);
  }
  static_assert(
    noexcept(
      std::declval<MotionLedgerClearanceProof::TimeoutPermitState &>() =
      MotionLedgerClearanceProof::TimeoutPermitState::kConsumed));
  clearance_proof.timeout_permit_state_ =
    MotionLedgerClearanceProof::TimeoutPermitState::kConsumed;
  return decision;
}

OperationTimeoutDisposition PendingOperationLedger::check_timeout(SteadyTime now)
{
  std::scoped_lock lock(mutex_);
  if (!pending_) {
    return OperationTimeoutDisposition::kNotDue;
  }
  if (pending_->ticket.effect == OperationEffect::kCancelableMotion) {
    if (!pending_->motion_binding ||
      pending_->phase == PendingOperationPhase::kAwaitingRequestBinding)
    {
      return OperationTimeoutDisposition::kNotDue;
    }
    const auto planned = plan_motion_timeout_locked(now);
    if (planned.status != MotionLedgerStatus::kApplied || !planned.plan ||
      !planned.plan->newly_effective)
    {
      return OperationTimeoutDisposition::kNotDue;
    }
    const auto disposition = planned.plan->disposition;
    apply_motion_timeout_plan_locked(*planned.plan);
    switch (disposition) {
      case OperationTimeoutDisposition::kCancelAndVerify:
      case OperationTimeoutDisposition::kStopProofExpired:
      case OperationTimeoutDisposition::kMotionLivenessUnknown:
        return disposition;
      default:
        return OperationTimeoutDisposition::kNotDue;
    }
  }
  if (pending_->phase == PendingOperationPhase::kReconciliationRequired) {
    if (now >= pending_->reconciliation_deadline) {
      pending_->active_reconciliation.reset();
      pending_->phase = PendingOperationPhase::kReconciliationExhausted;
      return OperationTimeoutDisposition::kReconciliationExhausted;
    }
    if (pending_->active_reconciliation &&
      now >= pending_->active_reconciliation->deadline)
    {
      pending_->active_reconciliation.reset();
      if (pending_->reconciliation_attempts >= policy_.max_attempts) {
        pending_->phase = PendingOperationPhase::kReconciliationExhausted;
        return OperationTimeoutDisposition::kReconciliationExhausted;
      }
      return OperationTimeoutDisposition::kReconciliationAttemptTimedOut;
    }
    if (!pending_->active_reconciliation &&
      pending_->reconciliation_attempts >= policy_.max_attempts)
    {
      pending_->phase = PendingOperationPhase::kReconciliationExhausted;
      return OperationTimeoutDisposition::kReconciliationExhausted;
    }
    return OperationTimeoutDisposition::kNotDue;
  }
  if (pending_->phase != PendingOperationPhase::kAwaitingCompletion ||
    now < pending_->ticket.deadline)
  {
    return OperationTimeoutDisposition::kNotDue;
  }
  switch (pending_->ticket.effect) {
    case OperationEffect::kReadOnly:
      pending_.reset();
      return OperationTimeoutDisposition::kDeliverTimeout;
    case OperationEffect::kCancelableMotion:
      pending_->phase = PendingOperationPhase::kCancelAndVerify;
      return OperationTimeoutDisposition::kCancelAndVerify;
    case OperationEffect::kIdempotentMutation:
      pending_->phase = PendingOperationPhase::kReconciliationRequired;
      pending_->reconciliation_deadline = bounded_deadline(now, policy_.window);
      return OperationTimeoutDisposition::kReconcile;
  }
  pending_->phase = PendingOperationPhase::kReconciliationExhausted;
  return OperationTimeoutDisposition::kReconciliationExhausted;
}

OperationCompletionDecision PendingOperationLedger::complete(
  GoalGeneration goal_generation, OperationGeneration operation_generation,
  SteadyTime completed_at)
{
  std::scoped_lock lock(mutex_);
  if (!pending_) {
    if (operation_generation > 0 && operation_generation < next_generation_) {
      return {OperationCompletionDisposition::kDiscardResolvedOrReadOnly, std::nullopt};
    }
    return {OperationCompletionDisposition::kRejectUnknown, std::nullopt};
  }
  if (pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation)
  {
    if (operation_generation > 0 && operation_generation < next_generation_) {
      return {OperationCompletionDisposition::kDiscardResolvedOrReadOnly, std::nullopt};
    }
    return {OperationCompletionDisposition::kRejectUnknown, std::nullopt};
  }
  const OperationTicket ticket = pending_->ticket;
  switch (pending_->phase) {
    case PendingOperationPhase::kAwaitingRequestBinding:
    case PendingOperationPhase::kAwaitingStopProof:
    case PendingOperationPhase::kMotionLivenessUnknown:
      return {OperationCompletionDisposition::kMotionTerminalRequired, ticket};
    case PendingOperationPhase::kAwaitingCompletion:
      if (ticket.effect == OperationEffect::kCancelableMotion) {
        return {OperationCompletionDisposition::kMotionTerminalRequired, ticket};
      }
      if (completed_at >= ticket.deadline) {
        switch (ticket.effect) {
          case OperationEffect::kReadOnly:
            pending_.reset();
            return {OperationCompletionDisposition::kDeliverTimeout, ticket};
          case OperationEffect::kCancelableMotion:
            pending_->phase = PendingOperationPhase::kCancelAndVerify;
            return {OperationCompletionDisposition::kCancelAndVerify, ticket};
          case OperationEffect::kIdempotentMutation:
            pending_->phase = PendingOperationPhase::kReconciliationRequired;
            pending_->reconciliation_deadline = bounded_deadline(completed_at, policy_.window);
            return {OperationCompletionDisposition::kReconcile, ticket};
        }
        pending_->phase = PendingOperationPhase::kReconciliationExhausted;
        return {OperationCompletionDisposition::kRejectUnknown, ticket};
      }
      pending_.reset();
      return {OperationCompletionDisposition::kDeliver, ticket};
    case PendingOperationPhase::kCancelAndVerify:
      return {
        ticket.effect == OperationEffect::kCancelableMotion ?
        OperationCompletionDisposition::kMotionTerminalRequired :
        OperationCompletionDisposition::kCancelEvidence,
        ticket};
    case PendingOperationPhase::kReconciliationRequired:
      if (completed_at >= pending_->reconciliation_deadline) {
        pending_->active_reconciliation.reset();
        pending_->phase = PendingOperationPhase::kReconciliationExhausted;
        return {OperationCompletionDisposition::kReconciliationExhausted, ticket};
      }
      pending_->active_reconciliation.reset();
      pending_->next_reconciliation_kind = ReconciliationKind::kReadback;
      return {OperationCompletionDisposition::kReconcile, ticket};
    case PendingOperationPhase::kReconciliationExhausted:
      return {OperationCompletionDisposition::kReconciliationExhausted, ticket};
  }
  return {OperationCompletionDisposition::kRejectUnknown, std::nullopt};
}

OperationEvidenceDecision PendingOperationLedger::classify_read_only_evidence(
  GoalGeneration goal_generation, OperationGeneration operation_generation,
  SteadyTime arrived_at) const
{
  std::scoped_lock lock(mutex_);
  if (!pending_) {
    if (operation_generation > 0U && operation_generation < next_generation_) {
      return {OperationEvidenceDisposition::kDiscardResolved, std::nullopt};
    }
    return {OperationEvidenceDisposition::kRejectUnknown, std::nullopt};
  }
  if (pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation)
  {
    if (operation_generation > 0U && operation_generation < next_generation_) {
      return {OperationEvidenceDisposition::kDiscardResolved, std::nullopt};
    }
    return {OperationEvidenceDisposition::kRejectUnknown, std::nullopt};
  }
  if (pending_->ticket.effect != OperationEffect::kReadOnly ||
    pending_->phase != PendingOperationPhase::kAwaitingCompletion)
  {
    return {OperationEvidenceDisposition::kRejectUnknown, pending_->ticket};
  }
  return {
    arrived_at >= pending_->ticket.deadline ?
    OperationEvidenceDisposition::kDeadlineWon : OperationEvidenceDisposition::kDeliver,
    pending_->ticket};
}

OperationCompletionDecision PendingOperationLedger::report_unknown_outcome(
  GoalGeneration goal_generation, OperationGeneration operation_generation,
  SteadyTime observed_at)
{
  std::scoped_lock lock(mutex_);
  if (!pending_ || pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation)
  {
    if (operation_generation > 0 && operation_generation < next_generation_) {
      return {OperationCompletionDisposition::kDiscardResolvedOrReadOnly, std::nullopt};
    }
    return {OperationCompletionDisposition::kRejectUnknown, std::nullopt};
  }
  const OperationTicket ticket = pending_->ticket;
  if (ticket.effect == OperationEffect::kCancelableMotion) {
    return {OperationCompletionDisposition::kMotionTerminalRequired, ticket};
  }
  if (pending_->phase == PendingOperationPhase::kAwaitingCompletion &&
    observed_at >= ticket.deadline)
  {
    switch (ticket.effect) {
      case OperationEffect::kReadOnly:
        pending_.reset();
        return {OperationCompletionDisposition::kDeliverTimeout, ticket};
      case OperationEffect::kCancelableMotion:
        pending_->phase = PendingOperationPhase::kCancelAndVerify;
        return {OperationCompletionDisposition::kCancelAndVerify, ticket};
      case OperationEffect::kIdempotentMutation:
        pending_->phase = PendingOperationPhase::kReconciliationRequired;
        pending_->reconciliation_deadline = bounded_deadline(observed_at, policy_.window);
        return {OperationCompletionDisposition::kReconcile, ticket};
    }
  }
  if (pending_->phase == PendingOperationPhase::kCancelAndVerify) {
    return {OperationCompletionDisposition::kCancelEvidence, ticket};
  }
  if (pending_->phase == PendingOperationPhase::kReconciliationRequired) {
    if (observed_at >= pending_->reconciliation_deadline) {
      pending_->active_reconciliation.reset();
      pending_->phase = PendingOperationPhase::kReconciliationExhausted;
      return {OperationCompletionDisposition::kReconciliationExhausted, ticket};
    }
    return {OperationCompletionDisposition::kReconcile, ticket};
  }
  if (pending_->phase == PendingOperationPhase::kReconciliationExhausted) {
    return {OperationCompletionDisposition::kReconciliationExhausted, ticket};
  }
  switch (ticket.effect) {
    case OperationEffect::kReadOnly:
      pending_.reset();
      return {OperationCompletionDisposition::kDeliver, ticket};
    case OperationEffect::kCancelableMotion:
      pending_->phase = PendingOperationPhase::kCancelAndVerify;
      return {OperationCompletionDisposition::kCancelAndVerify, ticket};
    case OperationEffect::kIdempotentMutation:
      pending_->phase = PendingOperationPhase::kReconciliationRequired;
      pending_->reconciliation_deadline = bounded_deadline(observed_at, policy_.window);
      return {OperationCompletionDisposition::kReconcile, ticket};
  }
  pending_->phase = PendingOperationPhase::kReconciliationExhausted;
  return {OperationCompletionDisposition::kReconciliationExhausted, ticket};
}

bool PendingOperationLedger::confirm_not_submitted(
  GoalGeneration goal_generation, OperationGeneration operation_generation)
{
  std::scoped_lock lock(mutex_);
  if (!pending_ || pending_->phase != PendingOperationPhase::kAwaitingCompletion ||
    pending_->ticket.effect != OperationEffect::kIdempotentMutation ||
    pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation)
  {
    return false;
  }
  pending_.reset();
  return true;
}

bool PendingOperationLedger::resolve(
  GoalGeneration goal_generation, OperationGeneration operation_generation)
{
  std::scoped_lock lock(mutex_);
  if (!pending_ || pending_->phase == PendingOperationPhase::kAwaitingCompletion ||
    pending_->phase == PendingOperationPhase::kAwaitingRequestBinding ||
    pending_->phase == PendingOperationPhase::kAwaitingStopProof ||
    pending_->phase == PendingOperationPhase::kMotionLivenessUnknown ||
    pending_->ticket.effect == OperationEffect::kCancelableMotion ||
    pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation)
  {
    return false;
  }
  pending_.reset();
  return true;
}

ReconciliationStartResult PendingOperationLedger::begin_reconciliation(
  GoalGeneration goal_generation, OperationGeneration operation_generation,
  ReconciliationKind kind, SteadyTime now)
{
  std::scoped_lock lock(mutex_);
  if (!valid_reconciliation_kind(kind) || goal_generation == 0 || operation_generation == 0) {
    return {ReconciliationStartError::kInvalidArgument, std::nullopt};
  }
  if (!pending_ || pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation ||
    pending_->phase == PendingOperationPhase::kAwaitingCompletion ||
    pending_->phase == PendingOperationPhase::kAwaitingRequestBinding ||
    pending_->phase == PendingOperationPhase::kCancelAndVerify ||
    pending_->phase == PendingOperationPhase::kAwaitingStopProof ||
    pending_->phase == PendingOperationPhase::kMotionLivenessUnknown ||
    pending_->ticket.effect == OperationEffect::kCancelableMotion)
  {
    return {ReconciliationStartError::kNotRequired, std::nullopt};
  }
  if (pending_->phase == PendingOperationPhase::kReconciliationExhausted ||
    now >= pending_->reconciliation_deadline ||
    pending_->reconciliation_attempts >= policy_.max_attempts)
  {
    pending_->active_reconciliation.reset();
    pending_->phase = PendingOperationPhase::kReconciliationExhausted;
    return {ReconciliationStartError::kExhausted, std::nullopt};
  }
  if (pending_->active_reconciliation) {
    return {ReconciliationStartError::kAttemptAlreadyActive, std::nullopt};
  }
  if (kind != pending_->next_reconciliation_kind) {
    return {ReconciliationStartError::kStrategyNotAllowed, std::nullopt};
  }
  ++pending_->reconciliation_attempts;
  ReconciliationAttempt attempt{
    pending_->ticket, kind, pending_->reconciliation_attempts,
    std::min(
      bounded_deadline(now, policy_.attempt_timeout), pending_->reconciliation_deadline),
    pending_->reconciliation_deadline};
  pending_->active_reconciliation = attempt;
  return {ReconciliationStartError::kNone, std::move(attempt)};
}

ReconciliationCompletionDisposition PendingOperationLedger::complete_reconciliation(
  GoalGeneration goal_generation, OperationGeneration operation_generation,
  ReconciliationKind kind, std::size_t attempt_number, SteadyTime completed_at,
  ReconciliationEvidence evidence)
{
  std::scoped_lock lock(mutex_);
  if (!pending_ || pending_->phase != PendingOperationPhase::kReconciliationRequired ||
    !pending_->active_reconciliation ||
    pending_->ticket.goal_generation != goal_generation ||
    pending_->ticket.operation_generation != operation_generation ||
    pending_->active_reconciliation->kind != kind ||
    pending_->active_reconciliation->attempt_number != attempt_number)
  {
    return ReconciliationCompletionDisposition::kRejectUnknown;
  }
  if (!valid_reconciliation_evidence(pending_->ticket.command, kind, evidence)) {
    return ReconciliationCompletionDisposition::kRejectUnknown;
  }
  if (completed_at >= pending_->reconciliation_deadline) {
    pending_->active_reconciliation.reset();
    pending_->phase = PendingOperationPhase::kReconciliationExhausted;
    return ReconciliationCompletionDisposition::kExhausted;
  }
  if (completed_at >= pending_->active_reconciliation->deadline) {
    pending_->active_reconciliation.reset();
    if (pending_->reconciliation_attempts >= policy_.max_attempts) {
      pending_->phase = PendingOperationPhase::kReconciliationExhausted;
      return ReconciliationCompletionDisposition::kExhausted;
    }
    return ReconciliationCompletionDisposition::kAttemptTimedOut;
  }
  pending_->active_reconciliation.reset();
  switch (evidence) {
    case ReconciliationEvidence::kInconclusive:
      break;
    case ReconciliationEvidence::kMutationResponseRetained:
      pending_->next_reconciliation_kind = ReconciliationKind::kReadback;
      break;
    case ReconciliationEvidence::kExactMutationStillApplied:
      pending_->next_reconciliation_kind = ReconciliationKind::kReplayMutation;
      break;
  }
  return ReconciliationCompletionDisposition::kCompleted;
}

std::optional<OperationTicket> PendingOperationLedger::pending() const
{
  std::scoped_lock lock(mutex_);
  return pending_ ? std::optional<OperationTicket>(pending_->ticket) : std::nullopt;
}

std::optional<PendingOperationSnapshot> PendingOperationLedger::snapshot_locked() const
{
  if (!pending_) {
    return std::nullopt;
  }
  PendingOperationSnapshot output;
  output.ticket = pending_->ticket;
  output.phase = pending_->phase;
  output.execution_attempt_generation = pending_->execution_attempt_generation;
  if (pending_->motion_binding &&
    pending_->motion_binding->identity.execution_request_fingerprint != 0U)
  {
    output.bound_request_fingerprint =
      pending_->motion_binding->identity.execution_request_fingerprint;
  }
  output.motion_deadlines = pending_->motion_deadlines;
  output.submission_attempt_id = pending_->submission_attempt_id;
  output.accepted_goal = pending_->accepted_goal;
  output.cancel_submission = pending_->cancel_submission;
  output.post_terminal_cancel_submission = pending_->post_terminal_cancel_submission;
  output.effective_motion_timeout = pending_->effective_motion_timeout;
  output.goal_response_deadline_expired = pending_->goal_response_deadline_expired;
  output.public_deadline_expired = pending_->public_deadline_expired;
  output.liveness_was_unknown = pending_->liveness_was_unknown;
  output.terminal = pending_->terminal;
  output.late_stop_proof = pending_->late_stop_proof;
  output.late_non_submission = pending_->late_non_submission;
  output.ingress_fault = pending_->ingress_fault;
  output.stop_proof_required = pending_->submission_attempt_id.has_value();
  return output;
}

std::optional<PendingOperationSnapshot> PendingOperationLedger::snapshot() const
{
  std::scoped_lock lock(mutex_);
  return snapshot_locked();
}

MotionLedgerAuthoritySnapshot PendingOperationLedger::authority_snapshot() const
{
  std::scoped_lock lock(mutex_);
  return MotionLedgerAuthoritySnapshot{
    snapshot_locked(), last_issued_clearance_generation_, clearance_generation_limit_,
    last_issued_clearance_generation_ >= clearance_generation_limit_, next_generation_};
}

std::optional<PendingOperationPhase> PendingOperationLedger::phase() const
{
  std::scoped_lock lock(mutex_);
  return pending_ ? std::optional<PendingOperationPhase>(pending_->phase) : std::nullopt;
}

std::optional<ReconciliationKind> PendingOperationLedger::next_reconciliation_kind() const
{
  std::scoped_lock lock(mutex_);
  if (!pending_ || pending_->phase != PendingOperationPhase::kReconciliationRequired) {
    return std::nullopt;
  }
  return pending_->next_reconciliation_kind;
}

std::optional<ReconciliationAttempt> PendingOperationLedger::active_reconciliation() const
{
  std::scoped_lock lock(mutex_);
  return pending_ ? pending_->active_reconciliation : std::nullopt;
}

}  // namespace restocker_task_executor
