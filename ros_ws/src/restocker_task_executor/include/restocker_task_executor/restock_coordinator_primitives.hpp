// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <variant>

#include "restocker_task_executor/coordinator_operation_types.hpp"
#include "restocker_task_executor/goal_termination_authority.hpp"
#include "restocker_task_executor/motion_operation_evidence.hpp"

namespace restocker_task_executor
{

class MotionSubmissionCommitRecord;

struct OperationCorrelation
{
  GoalGeneration goal_generation{0};
  OperationGeneration operation_generation{0};
};

enum class GoalSlotPhase : std::uint8_t
{
  kIdle,
  kPendingAcceptance,
  kActive,
};

enum class GoalAdmissionDecision : std::uint8_t
{
  kAccepted = 0U,
  kInvalidGoalId = 1U,
  kNotReady = 2U,
  kBusy = 3U,
  kInhibited = 4U,
  kResourceExhausted = 5U,
};

using GoalTerminationAllocationFailureInjector = std::function<bool ()>;

enum class GoalTerminationIntent : std::uint8_t
{
  kNone,
  kUserCancel,
  kShutdownDrain,
  kSafeAbort,
  kTaskDeadline,
};

enum class CoordinatorMutationKind : std::uint8_t
{
  kReserveTask,
  kReleaseTask,
  kCheckpointTask,
  kAcquireSceneLease,
  kReleaseSceneLease,
  kSetPhysicalAttachment,
  kApplyPlanningScene,
  kCommitAttachment,
  kCommitDetachment,
};

enum class MutationSubmissionPhase : std::uint8_t
{
  kCommitted,
  kConfirmedNotSubmitted,
};

struct MutationSubmissionRecord
{
  OperationGeneration operation_generation{0};
  CoordinatorMutationKind kind{CoordinatorMutationKind::kReserveTask};
  MutationSubmissionPhase phase{MutationSubmissionPhase::kCommitted};
};

enum class MutationSubmissionDecision : std::uint8_t
{
  kCommitted,
  kReplayCommitted,
  kInvalidArgument,
  kGoalMismatch,
  kInactive,
  kInhibited,
  kTerminationRequested,
  kCleanupWithoutTermination,
  kAlreadyPending,
  kDeadlineExceeded,
  kStaleOperation,
};

struct MutationSubmissionResult
{
  MutationSubmissionDecision decision{MutationSubmissionDecision::kInvalidArgument};
  std::optional<MutationSubmissionRecord> record;
};

struct GoalAdmissionReceipt
{
  GoalAdmissionDecision decision{GoalAdmissionDecision::kNotReady};
  GoalGeneration generation{0};
};

struct GoalAdmissionSnapshot
{
  GoalSlotPhase phase{GoalSlotPhase::kIdle};
  bool ready{false};
  bool inhibited{false};
  bool cancel_requested{false};
  bool safe_abort_requested{false};
  bool task_deadline_exceeded{false};
  GoalTerminationIntent termination_intent{GoalTerminationIntent::kNone};
  GoalGeneration generation{0};
  std::optional<CoordinatorGoalId> goal_id;
  std::optional<MutationSubmissionRecord> mutation_submission;
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
  std::string readiness_detail;
  std::string inhibition_detail;
  bool motion_workflow_active{false};
  std::shared_ptr<const MotionAdmissionRegistrationRecord> motion_registration;
  bool motion_request_bound{false};
  std::shared_ptr<const MotionOperationIdentity> motion_request_identity;
  bool motion_submission_state_present{false};
  std::shared_ptr<const MotionSubmissionCommitRecord> motion_submission_commit;
  bool motion_submission_authorization_issuer_configured{false};
  std::uint64_t consumed_motion_authorization_generation_high_water{0U};
};

enum class GoalActivationAuthorityStatus : std::uint8_t
{
  kObserved,
  kInvalidArgument,
  kInactive,
  kGoalMismatch,
  kWrongPhase,
  kMalformedState,
  kSynchronizationFailed,
};

// Narrow, allocation-free view used while the coordinator binding lock is held. Unlike the
// general snapshot, it copies no diagnostic strings or mutation record.
struct GoalActivationAuthority
{
  GoalSlotPhase phase{GoalSlotPhase::kIdle};
  CoordinatorGoalId goal_id{};
  GoalGeneration generation{0U};
  bool inhibited{false};
  bool cancel_requested{false};
  bool safe_abort_requested{false};
  bool task_deadline_exceeded{false};
  bool mutation_submission{false};
  GoalTerminationIntent termination_intent{GoalTerminationIntent::kNone};
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
};

struct GoalActivationAuthorityDecision
{
  GoalActivationAuthorityStatus status{GoalActivationAuthorityStatus::kSynchronizationFailed};
  std::optional<GoalActivationAuthority> authority;
};

class MotionLedgerIssuerCore;
class MotionLedgerIssuerBinding;
class MotionAdmissionIssuerCore;
class MotionAdmissionRegistrationReceipt;
class MotionAdmissionRegistrationDecision;
class MotionAdmissionWithdrawalDecision;
class MotionAdmissionIssuerBinding;
class MotionSubmissionAuthorizationIssuerCore;
class MotionSubmissionAuthorizationIssuerBinding;
class MotionSubmissionAuthorization;
class MotionAdmissionRequestIdentityReceipt;
class MotionAdmissionRequestBindingDecision;
class PreparedMotionSubmissionCommit;
class MotionSubmissionCommitDecision;
class MotionSubmissionCommitReceipt;
class MotionWorkflowNonSubmissionResolutionDecision;
class MotionLedgerClearanceProof;
class CoordinatorPreGraspMotionRuntime;

enum class MotionAdmissionRoutingOwnershipStatus : std::uint8_t
{
  kNotOwned,
  kOwned,
  kInvalidArgument,
  kInactive,
  kGoalMismatch,
  kLedgerIssuerMismatch,
  kOperationMismatch,
  kMalformedState,
  kSynchronizationFailed,
};

class MotionAdmissionRuntimeReservationWitness final
{
public:
  MotionAdmissionRuntimeReservationWitness(
    const MotionAdmissionRuntimeReservationWitness &) = delete;
  MotionAdmissionRuntimeReservationWitness & operator=(
    const MotionAdmissionRuntimeReservationWitness &) = delete;
  MotionAdmissionRuntimeReservationWitness(
    MotionAdmissionRuntimeReservationWitness && other) noexcept;
  MotionAdmissionRuntimeReservationWitness & operator=(
    MotionAdmissionRuntimeReservationWitness &&) = delete;
  ~MotionAdmissionRuntimeReservationWitness() = default;

private:
  enum class State : std::uint8_t {kLive, kAuthorizationSealed, kReleased, kMovedFrom};

  MotionAdmissionRuntimeReservationWitness(
    std::shared_ptr<const MotionAdmissionIssuerCore> admission_issuer,
    std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
    std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer,
    std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> runtime_issuer) noexcept;

  std::shared_ptr<const MotionAdmissionIssuerCore> admission_issuer_;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> registration_;
  std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer_;
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> runtime_issuer_;
  State state_{State::kMovedFrom};

  friend class GoalAdmissionSlot;
  friend class CoordinatorPreGraspMotionRuntime;
};

enum class MotionSubmissionAuthorizationIssuerConfigurationStatus : std::uint8_t
{
  kConfigured,
  kAlreadyConfigured,
  kInvalidIssuer,
  kIssuerMismatch,
};

class GoalAdmissionSlot
{
public:
  explicit GoalAdmissionSlot(
    GoalTerminationAllocationFailureInjector allocation_failure_injector = {});

  void update_readiness(bool ready, std::string detail = {});
  void inhibit(std::string detail);

  [[nodiscard]] GoalAdmissionReceipt reserve(const CoordinatorGoalId & goal_id);
  [[nodiscard]] bool activate(const CoordinatorGoalId & goal_id, GoalGeneration generation);
  [[nodiscard]] GoalTerminationLatchDecision request_cancel(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_drain(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_safe_abort(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_shutdown(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_authority_loss(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_inbox_loss(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_protocol_failure(
    const CoordinatorGoalId & goal_id, GoalGeneration generation, SteadyTime arrived_at);
  [[nodiscard]] GoalTerminationLatchDecision request_task_deadline(
    const CoordinatorGoalId & goal_id, GoalGeneration generation,
    GoalGeneration deadline_generation, SteadyTime captured_at);
  // Fenced motion-deadline latch. Requires nonzero fence_generation and records
  // GoalTerminationSource::kMotionFence. An earlier admission termination remains immutable.
  [[nodiscard]] GoalTerminationLatchDecision request_motion_deadline(
    const CoordinatorGoalId & goal_id, GoalGeneration generation,
    std::uint64_t fence_generation, SteadyTime captured_at);
  [[nodiscard]] GoalTerminationSnapshotDecision snapshot_first_termination(
    const CoordinatorGoalId & goal_id, GoalGeneration generation) const;
  [[nodiscard]] GoalActivationAuthorityDecision observe_activation_authority(
    const CoordinatorGoalId & goal_id, GoalGeneration generation) const noexcept;
  [[nodiscard]] MotionAdmissionRegistrationDecision register_motion_workflow(
    std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
    const MotionLedgerIssuerBinding & ledger_issuer_binding) noexcept;
  [[nodiscard]] MotionAdmissionWithdrawalDecision withdraw_unbound_motion_workflow(
    MotionAdmissionRegistrationReceipt & receipt) noexcept;
  [[nodiscard]] MotionAdmissionRequestBindingDecision bind_motion_request_identity(
    MotionAdmissionRegistrationReceipt & receipt,
    std::shared_ptr<const MotionOperationIdentity> request_identity) noexcept;
  [[nodiscard]] MotionSubmissionCommitDecision commit_motion_submission(
    MotionSubmissionAuthorization & authorization,
    PreparedMotionSubmissionCommit & preparation) noexcept;
  // Exact non-submission admission resolution. Precommit: no stored commit and no commit
  // receipt. Postcommit: stored commit plus exact move-only commit receipt. Foreign ledger
  // issuer, missing/unexpected/moved-from/substituted receipts reject without mutation. Exact
  // success consumes the registration receipt and moves-from the clearance proof; rejection
  // leaves both caller-owned. Nonallocating and nonthrowing under the admission mutex.
  [[nodiscard]] MotionWorkflowNonSubmissionResolutionDecision
  resolve_motion_workflow_non_submission(
    MotionLedgerClearanceProof & clearance_proof,
    MotionAdmissionRegistrationReceipt & registration_receipt,
    MotionSubmissionCommitReceipt * commit_receipt = nullptr) noexcept;
  [[nodiscard]] MotionAdmissionRoutingOwnershipStatus observe_motion_routing_ownership(
    const CoordinatorGoalId & goal_id, GoalGeneration generation,
    const OperationTicket * pending_operation,
    const MotionLedgerIssuerBinding & ledger_issuer_binding) const noexcept;
  [[nodiscard]] MotionSubmissionAuthorizationIssuerConfigurationStatus
  configure_motion_submission_authorization_issuer(
    const MotionSubmissionAuthorizationIssuerBinding & issuer) noexcept;
  // Declares that the goal has finished its work and its reservation release is the normal last
  // step, not a teardown. Without it a release is admitted only while the goal is being
  // terminated. False when the goal is not the active one.
  [[nodiscard]] bool begin_completion_cleanup(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation);
  // Declares that the task machine's recoverable skip (Milestone 10 §6 rung 5, Card 051) is
  // releasing its reservation as its own teardown — neither a user termination nor a completed
  // transfer. Authorizes the same cleanup mutation a termination does, and nothing else.
  // False when the goal is not the active one.
  [[nodiscard]] bool begin_skip_cleanup(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation);
  [[nodiscard]] MutationSubmissionResult commit_mutation_submission(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    OperationGeneration operation_generation, CoordinatorMutationKind kind,
    SteadyTime now, SteadyTime absolute_deadline);
  [[nodiscard]] bool confirm_mutation_not_submitted(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    OperationGeneration operation_generation, CoordinatorMutationKind kind);
  [[nodiscard]] bool resolve_mutation_submission(
    const CoordinatorGoalId & goal_id, GoalGeneration goal_generation,
    OperationGeneration operation_generation, CoordinatorMutationKind kind);
  [[nodiscard]] bool finish(const CoordinatorGoalId & goal_id, GoalGeneration generation);
  [[nodiscard]] GoalAdmissionSnapshot snapshot() const;

private:
  [[nodiscard]] GoalAdmissionDecision reserve_status_locked(
    const CoordinatorGoalId & goal_id);
  [[nodiscard]] GoalTerminationLatchDecision request_termination(
    FirstGoalTerminationRecord record);
  [[nodiscard]] bool apply_termination_policy_locked(GoalTerminationKind kind) noexcept;
  [[nodiscard]] MotionAdmissionIssuerBinding motion_admission_issuer_binding() const noexcept;
  [[nodiscard]] std::optional<MotionAdmissionRuntimeReservationWitness>
  begin_runtime_submission_reservation(
    const MotionAdmissionRegistrationReceipt & receipt,
    const MotionSubmissionAuthorizationIssuerBinding & runtime_issuer) noexcept;
  [[nodiscard]] bool release_unissued_runtime_submission_reservation(
    MotionAdmissionRuntimeReservationWitness & witness) noexcept;
  [[nodiscard]] bool seal_runtime_submission_authorization(
    MotionAdmissionRuntimeReservationWitness & witness,
    std::uint64_t authorization_generation) noexcept;

  mutable std::mutex mutex_;
  const GoalTerminationAllocationFailureInjector allocation_failure_injector_;
  GoalSlotPhase phase_{GoalSlotPhase::kIdle};
  bool ready_{false};
  bool inhibited_{false};
  bool cancel_requested_{false};
  bool safe_abort_requested_{false};
  bool task_deadline_exceeded_{false};
  GoalTerminationIntent termination_intent_{GoalTerminationIntent::kNone};
  // Set once the task has completed and is releasing its reservation. Authorizes the same cleanup
  // mutation a termination does, and nothing else.
  bool completion_cleanup_{false};
  // Set once the recoverable skip has claimed its reservation release (Card 051). Same
  // authorization scope as completion_cleanup_; a skip is a deliberate teardown of a goal that
  // neither completed nor was terminated by an external requester.
  bool skip_cleanup_{false};
  GoalGeneration next_generation_{1};
  GoalGeneration generation_{0};
  std::uint64_t goal_id_word_0_{0U};
  std::uint64_t goal_id_word_1_{0U};
  OperationGeneration last_resolved_operation_generation_{0};
  std::optional<CoordinatorGoalId> goal_id_;
  std::optional<MutationSubmissionRecord> mutation_submission_;
  std::shared_ptr<FirstGoalTerminationRecord> unpublished_termination_record_;
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination_record_;
  std::shared_ptr<const MotionAdmissionIssuerCore> motion_admission_issuer_;
  bool motion_workflow_active_{false};
  std::shared_ptr<const MotionAdmissionRegistrationRecord> motion_registration_;
  std::shared_ptr<const MotionLedgerIssuerCore> motion_ledger_issuer_;
  bool motion_request_bound_{false};
  std::shared_ptr<const MotionOperationIdentity> motion_request_identity_;
  bool motion_submission_state_present_{false};
  std::shared_ptr<const MotionSubmissionCommitRecord> motion_submission_commit_;
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore>
  motion_submission_authorization_issuer_;
  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore>
  motion_runtime_reservation_issuer_;
  MotionAdmissionSessionGeneration motion_runtime_reservation_session_generation_{0U};
  std::uint64_t motion_runtime_authorization_generation_{0U};
  std::uint64_t consumed_motion_authorization_generation_high_water_{0U};
  std::string generation_exhaustion_detail_{"goal-generation space is exhausted"};
  std::string readiness_detail_;
  std::string inhibition_detail_;

  friend class CoordinatorPreGraspMotionRuntime;
};

enum class ReconciliationKind : std::uint8_t
{
  kReplayMutation,
  kReadback,
};

struct ReconciliationPolicy
{
  std::chrono::milliseconds window{5000};
  std::chrono::milliseconds attempt_timeout{1000};
  std::size_t max_attempts{3U};
};

enum class OperationStartError : std::uint8_t
{
  kNone,
  kAlreadyPending,
  kInvalidArgument,
};

struct OperationStartResult
{
  OperationStartError error{OperationStartError::kNone};
  std::optional<OperationTicket> ticket;
};

enum class OperationTimeoutDisposition : std::uint8_t
{
  kNotDue,
  kDeliverTimeout,
  kCancelAndVerify,
  kReconcile,
  kReconciliationAttemptTimedOut,
  kReconciliationExhausted,
  kStopProofExpired,
  kMotionLivenessUnknown,
};

enum class OperationCompletionDisposition : std::uint8_t
{
  kDeliver,
  kDeliverTimeout,
  kCancelAndVerify,
  kDiscardResolvedOrReadOnly,
  kCancelEvidence,
  kReconcile,
  kReconciliationExhausted,
  kMotionTerminalRequired,
  kRejectUnknown,
};

enum class MotionLedgerStatus : std::uint8_t
{
  kApplied,
  kAlreadyApplied,
  kStaleResolved,
  kNoPendingOperation,
  kCorrelationMismatch,
  kWrongEffect,
  kWrongPhase,
  kInvalidArgument,
  kEvidenceMismatch,
  kDeadlineExpired,
  kLivenessUnknown,
  kClearanceGenerationExhausted,
};

using MotionLedgerClearanceGeneration = std::uint64_t;

enum class MotionLedgerClearanceKind : std::uint8_t
{
  kNonSubmission,
  kStopped,
};

using MotionLedgerClearanceRecord = std::variant<
  DefinitelyNotSubmittedRecord,
  MotionStopProofRecord>;

class MotionLedgerIssuerBinding final
{
public:
  MotionLedgerIssuerBinding(const MotionLedgerIssuerBinding &) noexcept = default;
  MotionLedgerIssuerBinding & operator=(
    const MotionLedgerIssuerBinding &) noexcept = default;

  [[nodiscard]] bool same_issuer_as(
    const MotionLedgerIssuerBinding & other) const noexcept;

private:
  explicit MotionLedgerIssuerBinding(
    std::shared_ptr<const MotionLedgerIssuerCore> issuer) noexcept;

  std::shared_ptr<const MotionLedgerIssuerCore> issuer_;

  friend class PendingOperationLedger;
  friend class MotionLedgerClearanceProof;
  friend class GoalAdmissionSlot;
  friend class MotionAdmissionRegistrationReceipt;
  friend class PreGraspMotionSession;
};

// Stable, opaque identity for the admission slot that issued an exact motion registration.
// Possession grants comparison authority only; it cannot register, withdraw, or mutate a goal.
class MotionAdmissionIssuerBinding final
{
public:
  MotionAdmissionIssuerBinding(const MotionAdmissionIssuerBinding &) noexcept = default;
  MotionAdmissionIssuerBinding & operator=(
    const MotionAdmissionIssuerBinding &) noexcept = default;

  [[nodiscard]] bool same_issuer_as(
    const MotionAdmissionIssuerBinding & other) const noexcept;

private:
  explicit MotionAdmissionIssuerBinding(
    std::shared_ptr<const MotionAdmissionIssuerCore> issuer) noexcept;

  std::shared_ptr<const MotionAdmissionIssuerCore> issuer_;

  friend class GoalAdmissionSlot;
  friend class CoordinatorPreGraspMotionRuntime;
};

// Stable identity of the sole live-runtime authorization issuer accepted by one admission slot.
// Construction is private to that runtime; public code can retain and compare it but cannot mint
// an authorization.
class MotionSubmissionAuthorizationIssuerBinding final
{
public:
  MotionSubmissionAuthorizationIssuerBinding(
    const MotionSubmissionAuthorizationIssuerBinding &) noexcept = default;
  MotionSubmissionAuthorizationIssuerBinding & operator=(
    const MotionSubmissionAuthorizationIssuerBinding &) noexcept = default;

  [[nodiscard]] bool same_issuer_as(
    const MotionSubmissionAuthorizationIssuerBinding & other) const noexcept;

private:
  explicit MotionSubmissionAuthorizationIssuerBinding(
    std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> issuer) noexcept;

  std::shared_ptr<const MotionSubmissionAuthorizationIssuerCore> issuer_;

  friend class GoalAdmissionSlot;
  friend class CoordinatorPreGraspMotionRuntime;
};

enum class MotionAdmissionRegistrationStatus : std::uint8_t
{
  kRegistered,
  kTerminationWon,
  kInvalidRegistration,
  kInactive,
  kGoalMismatch,
  kWrongPhase,
  kWorkflowAlreadyActive,
  kLedgerIssuerMismatch,
  kSubmissionStatePresent,
};

class MotionAdmissionRegistrationReceipt final
{
public:
  MotionAdmissionRegistrationReceipt(const MotionAdmissionRegistrationReceipt &) = delete;
  MotionAdmissionRegistrationReceipt & operator=(
    const MotionAdmissionRegistrationReceipt &) = delete;
  MotionAdmissionRegistrationReceipt(
    MotionAdmissionRegistrationReceipt && other) noexcept;
  MotionAdmissionRegistrationReceipt & operator=(
    MotionAdmissionRegistrationReceipt && other) noexcept;
  ~MotionAdmissionRegistrationReceipt() = default;

  [[nodiscard]] bool live() const noexcept;
  [[nodiscard]] bool withdrawn() const noexcept;
  [[nodiscard]] const MotionAdmissionRegistrationRecord * registration() const noexcept;
  [[nodiscard]] bool issued_for_ledger(
    const MotionLedgerIssuerBinding & binding) const noexcept;

private:
  enum class State : std::uint8_t {kLive, kWithdrawn, kMovedFrom};

  MotionAdmissionRegistrationReceipt(
    std::shared_ptr<const MotionAdmissionIssuerCore> issuer,
    std::shared_ptr<const MotionAdmissionRegistrationRecord> registration,
    std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer) noexcept;

  std::shared_ptr<const MotionAdmissionIssuerCore> issuer_;
  std::shared_ptr<const MotionAdmissionRegistrationRecord> registration_;
  std::shared_ptr<const MotionLedgerIssuerCore> ledger_issuer_;
  State state_{State::kMovedFrom};
  bool request_binding_permit_live_{false};

  friend class GoalAdmissionSlot;
  friend class CoordinatorPreGraspMotionRuntime;
};

class MotionAdmissionRegistrationDecision final
{
public:
  MotionAdmissionRegistrationDecision(const MotionAdmissionRegistrationDecision &) = delete;
  MotionAdmissionRegistrationDecision & operator=(
    const MotionAdmissionRegistrationDecision &) = delete;
  MotionAdmissionRegistrationDecision(
    MotionAdmissionRegistrationDecision && other) noexcept;
  MotionAdmissionRegistrationDecision & operator=(
    MotionAdmissionRegistrationDecision &&) = delete;
  ~MotionAdmissionRegistrationDecision() = default;

  [[nodiscard]] MotionAdmissionRegistrationStatus status() const noexcept;
  [[nodiscard]] const MotionAdmissionRegistrationReceipt * receipt() const noexcept;
  [[nodiscard]] std::optional<MotionAdmissionRegistrationReceipt> take_receipt() noexcept;
  [[nodiscard]] const std::shared_ptr<const FirstGoalTerminationRecord> &
  first_termination() const noexcept;

private:
  MotionAdmissionRegistrationDecision() = default;

  MotionAdmissionRegistrationStatus status_{
    MotionAdmissionRegistrationStatus::kInvalidRegistration};
  std::optional<MotionAdmissionRegistrationReceipt> receipt_;
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination_;

  friend class GoalAdmissionSlot;
};

enum class MotionAdmissionWithdrawalStatus : std::uint8_t
{
  kWithdrawn,
  kReceiptMismatch,
  kRegistrationMismatch,
  kAlreadyRequestBound,
  kSubmissionStatePresent,
  kAlreadyWithdrawn,
};

class MotionAdmissionWithdrawalDecision final
{
public:
  MotionAdmissionWithdrawalDecision(const MotionAdmissionWithdrawalDecision &) = delete;
  MotionAdmissionWithdrawalDecision & operator=(
    const MotionAdmissionWithdrawalDecision &) = delete;
  MotionAdmissionWithdrawalDecision(
    MotionAdmissionWithdrawalDecision && other) noexcept;
  MotionAdmissionWithdrawalDecision & operator=(
    MotionAdmissionWithdrawalDecision &&) = delete;
  ~MotionAdmissionWithdrawalDecision() = default;

  [[nodiscard]] MotionAdmissionWithdrawalStatus status() const noexcept;
  [[nodiscard]] const std::shared_ptr<const FirstGoalTerminationRecord> &
  first_termination() const noexcept;

private:
  MotionAdmissionWithdrawalDecision() = default;

  MotionAdmissionWithdrawalStatus status_{
    MotionAdmissionWithdrawalStatus::kReceiptMismatch};
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination_;

  friend class GoalAdmissionSlot;
};

enum class MotionWorkflowNonSubmissionResolutionStatus : std::uint8_t
{
  kResolved,
  kInvalidProof,
  kForeignLedgerIssuer,
  kProofKindMismatch,
  kProofIdentityMismatch,
  kReceiptMismatch,
  kRegistrationMismatch,
  kRequestIdentityMismatch,
  kPrecommitShapeMismatch,
  kPostcommitShapeMismatch,
  kCommitReceiptMismatch,
  kInactive,
  kWrongPhase,
};

class MotionWorkflowNonSubmissionResolutionDecision final
{
public:
  MotionWorkflowNonSubmissionResolutionDecision(
    const MotionWorkflowNonSubmissionResolutionDecision &) = delete;
  MotionWorkflowNonSubmissionResolutionDecision & operator=(
    const MotionWorkflowNonSubmissionResolutionDecision &) = delete;
  MotionWorkflowNonSubmissionResolutionDecision(
    MotionWorkflowNonSubmissionResolutionDecision && other) noexcept;
  MotionWorkflowNonSubmissionResolutionDecision & operator=(
    MotionWorkflowNonSubmissionResolutionDecision &&) = delete;
  ~MotionWorkflowNonSubmissionResolutionDecision() = default;

  [[nodiscard]] MotionWorkflowNonSubmissionResolutionStatus status() const noexcept;
  [[nodiscard]] const std::shared_ptr<const FirstGoalTerminationRecord> &
  first_termination() const noexcept;

private:
  MotionWorkflowNonSubmissionResolutionDecision() = default;

  MotionWorkflowNonSubmissionResolutionStatus status_{
    MotionWorkflowNonSubmissionResolutionStatus::kInvalidProof};
  std::shared_ptr<const FirstGoalTerminationRecord> first_termination_;

  friend class GoalAdmissionSlot;
};

class PreparedMotionLedgerClearance final
{
public:
  PreparedMotionLedgerClearance(const PreparedMotionLedgerClearance &) = delete;
  PreparedMotionLedgerClearance & operator=(
    const PreparedMotionLedgerClearance &) = delete;
  PreparedMotionLedgerClearance(PreparedMotionLedgerClearance && other) noexcept;
  PreparedMotionLedgerClearance & operator=(PreparedMotionLedgerClearance &&) = delete;
  ~PreparedMotionLedgerClearance() = default;

  [[nodiscard]] bool live() const noexcept;
  [[nodiscard]] MotionLedgerClearanceKind kind() const noexcept;
  [[nodiscard]] const DefinitelyNotSubmittedRecord * non_submission_record() const noexcept;
  [[nodiscard]] const MotionStopProofRecord * stop_record() const noexcept;

private:
  enum class State : std::uint8_t {kLive, kMovedFrom};

  PreparedMotionLedgerClearance(
    std::shared_ptr<const MotionLedgerIssuerCore> issuer,
    std::shared_ptr<const MotionLedgerClearanceRecord> record,
    MotionLedgerClearanceKind kind) noexcept;

  std::shared_ptr<const MotionLedgerIssuerCore> issuer_;
  std::shared_ptr<const MotionLedgerClearanceRecord> record_;
  MotionLedgerClearanceKind kind_{MotionLedgerClearanceKind::kNonSubmission};
  State state_{State::kMovedFrom};

  friend class PendingOperationLedger;
};

enum class MotionLedgerClearancePreparationStatus : std::uint8_t
{
  kPrepared,
  kInvalidRecord,
};

class MotionLedgerClearancePreparationDecision final
{
public:
  MotionLedgerClearancePreparationDecision(
    const MotionLedgerClearancePreparationDecision &) = delete;
  MotionLedgerClearancePreparationDecision & operator=(
    const MotionLedgerClearancePreparationDecision &) = delete;
  MotionLedgerClearancePreparationDecision(
    MotionLedgerClearancePreparationDecision && other) noexcept;
  MotionLedgerClearancePreparationDecision & operator=(
    MotionLedgerClearancePreparationDecision &&) = delete;

  [[nodiscard]] MotionLedgerClearancePreparationStatus status() const noexcept;
  [[nodiscard]] const PreparedMotionLedgerClearance * preparation() const noexcept;
  [[nodiscard]] std::optional<PreparedMotionLedgerClearance> take_preparation() noexcept;

private:
  MotionLedgerClearancePreparationDecision() = default;

  MotionLedgerClearancePreparationStatus status_{
    MotionLedgerClearancePreparationStatus::kInvalidRecord};
  std::optional<PreparedMotionLedgerClearance> preparation_;

  friend class PendingOperationLedger;
};

class MotionLedgerClearanceProof final
{
public:
  MotionLedgerClearanceProof(const MotionLedgerClearanceProof &) = delete;
  MotionLedgerClearanceProof & operator=(const MotionLedgerClearanceProof &) = delete;
  MotionLedgerClearanceProof(MotionLedgerClearanceProof && other) noexcept;
  MotionLedgerClearanceProof & operator=(MotionLedgerClearanceProof &&) = delete;
  ~MotionLedgerClearanceProof() = default;

  [[nodiscard]] bool live() const noexcept;
  [[nodiscard]] MotionLedgerClearanceGeneration generation() const noexcept;
  [[nodiscard]] MotionLedgerClearanceKind kind() const noexcept;
  [[nodiscard]] const MotionOperationIdentity * exact_identity() const noexcept;
  [[nodiscard]] const DefinitelyNotSubmittedRecord * non_submission_record() const noexcept;
  [[nodiscard]] const MotionStopProofRecord * stop_record() const noexcept;
  [[nodiscard]] bool issued_by(const MotionLedgerIssuerBinding & binding) const noexcept;
  [[nodiscard]] bool timeout_correlation_permit_available() const noexcept;

private:
  enum class State : std::uint8_t {kLive, kMovedFrom};
  enum class TimeoutPermitState : std::uint8_t {kAvailable, kConsumed};

  MotionLedgerClearanceProof(
    std::shared_ptr<const MotionLedgerIssuerCore> issuer,
    std::shared_ptr<const MotionLedgerClearanceRecord> record,
    MotionLedgerClearanceGeneration generation,
    MotionLedgerClearanceKind kind) noexcept;

  std::shared_ptr<const MotionLedgerIssuerCore> issuer_;
  std::shared_ptr<const MotionLedgerClearanceRecord> record_;
  MotionLedgerClearanceGeneration generation_{0U};
  MotionLedgerClearanceKind kind_{MotionLedgerClearanceKind::kNonSubmission};
  State state_{State::kMovedFrom};
  TimeoutPermitState timeout_permit_state_{TimeoutPermitState::kConsumed};

  friend class PendingOperationLedger;
};

class MotionLedgerClearDecision final
{
public:
  MotionLedgerClearDecision(const MotionLedgerClearDecision &) = delete;
  MotionLedgerClearDecision & operator=(const MotionLedgerClearDecision &) = delete;
  MotionLedgerClearDecision(MotionLedgerClearDecision && other) noexcept;
  MotionLedgerClearDecision & operator=(MotionLedgerClearDecision &&) = delete;

  [[nodiscard]] MotionLedgerStatus status() const noexcept;
  [[nodiscard]] const MotionLedgerClearanceProof * clearance_proof() const noexcept;
  [[nodiscard]] std::optional<MotionLedgerClearanceProof> take_clearance_proof() noexcept;

private:
  MotionLedgerClearDecision() = default;

  MotionLedgerStatus status_{MotionLedgerStatus::kInvalidArgument};
  std::optional<MotionLedgerClearanceProof> clearance_proof_;

  friend class PendingOperationLedger;
};

enum class MotionTimeoutCause : std::uint8_t
{
  kNone,
  kGoalResponseDeadline,
  kExpectedResultDeadline,
  kTerminalDrainDeadline,
  kStopProofDeadline,
};

enum class MotionTimeoutAuthorityKind : std::uint8_t
{
  kLiveOperation,
  kClearedNonSubmission,
  kClearedStopped,
};

struct EffectiveMotionTimeout
{
  MotionTimeoutCause cause{MotionTimeoutCause::kNone};
  SteadyTime nominal_deadline{};
  SteadyTime first_effective_at{};

  [[nodiscard]] bool operator==(const EffectiveMotionTimeout &) const noexcept = default;
};

class MotionTimeoutCheckResult final
{
public:
  MotionTimeoutCheckResult(const MotionTimeoutCheckResult &) = default;
  MotionTimeoutCheckResult & operator=(const MotionTimeoutCheckResult &) = delete;
  MotionTimeoutCheckResult(MotionTimeoutCheckResult &&) noexcept = default;
  MotionTimeoutCheckResult & operator=(MotionTimeoutCheckResult &&) = delete;
  ~MotionTimeoutCheckResult() = default;

  [[nodiscard]] const MotionOperationIdentity & exact_identity() const noexcept
  {
    return exact_identity_;
  }
  [[nodiscard]] MotionTimeoutAuthorityKind authority_kind() const noexcept
  {
    return authority_kind_;
  }
  [[nodiscard]] std::optional<PendingOperationPhase> pre_phase() const noexcept
  {
    return pre_phase_;
  }
  [[nodiscard]] std::optional<PendingOperationPhase> post_phase() const noexcept
  {
    return post_phase_;
  }
  [[nodiscard]] OperationTimeoutDisposition disposition() const noexcept
  {
    return disposition_;
  }
  [[nodiscard]] MotionTimeoutCause cause() const noexcept {return cause_;}
  [[nodiscard]] bool newly_effective() const noexcept {return newly_effective_;}
  [[nodiscard]] std::optional<SteadyTime> nominal_deadline() const noexcept
  {
    return nominal_deadline_;
  }
  [[nodiscard]] std::optional<SteadyTime> first_effective_at() const noexcept
  {
    return first_effective_at_;
  }
  [[nodiscard]] SteadyTime checked_at() const noexcept {return checked_at_;}

private:
  friend class PendingOperationLedger;

  MotionTimeoutCheckResult(
    MotionOperationIdentity exact_identity,
    MotionTimeoutAuthorityKind authority_kind,
    std::optional<PendingOperationPhase> pre_phase,
    std::optional<PendingOperationPhase> post_phase,
    OperationTimeoutDisposition disposition,
    MotionTimeoutCause cause,
    bool newly_effective,
    std::optional<SteadyTime> nominal_deadline,
    std::optional<SteadyTime> first_effective_at,
    SteadyTime checked_at) noexcept;

  MotionOperationIdentity exact_identity_;
  MotionTimeoutAuthorityKind authority_kind_{MotionTimeoutAuthorityKind::kLiveOperation};
  std::optional<PendingOperationPhase> pre_phase_;
  std::optional<PendingOperationPhase> post_phase_;
  OperationTimeoutDisposition disposition_{OperationTimeoutDisposition::kNotDue};
  MotionTimeoutCause cause_{MotionTimeoutCause::kNone};
  bool newly_effective_{false};
  std::optional<SteadyTime> nominal_deadline_;
  std::optional<SteadyTime> first_effective_at_;
  SteadyTime checked_at_{};
};

class MotionTimeoutCheckDecision final
{
public:
  MotionTimeoutCheckDecision(const MotionTimeoutCheckDecision &) = delete;
  MotionTimeoutCheckDecision & operator=(const MotionTimeoutCheckDecision &) = delete;
  MotionTimeoutCheckDecision(MotionTimeoutCheckDecision && other) noexcept;
  MotionTimeoutCheckDecision & operator=(MotionTimeoutCheckDecision &&) = delete;
  ~MotionTimeoutCheckDecision() = default;

  [[nodiscard]] MotionLedgerStatus status() const noexcept;
  [[nodiscard]] const MotionTimeoutCheckResult * result() const noexcept;
  [[nodiscard]] std::optional<MotionTimeoutCheckResult> take_result() noexcept;

private:
  MotionTimeoutCheckDecision() = default;

  MotionLedgerStatus status_{MotionLedgerStatus::kInvalidArgument};
  std::optional<MotionTimeoutCheckResult> result_;

  friend class PendingOperationLedger;
};

enum class MotionTimeoutFailurePoint : std::uint8_t
{
  kBeforeResultIdentityMaterialization,
  kAfterDecisionPreparationBeforeCommit,
};

using MotionTimeoutFailureInjector = std::function<void (MotionTimeoutFailurePoint)>;

enum class MotionLedgerClearanceFailurePoint : std::uint8_t
{
  kBeforeRecordStorageMaterialization,
  kAfterDecisionPreparationBeforeCommit,
};

using MotionLedgerClearanceFailureInjector =
  std::function<void (MotionLedgerClearanceFailurePoint)>;

struct MotionLedgerClearanceConfig
{
  MotionLedgerClearanceGeneration generation_limit{
    std::numeric_limits<MotionLedgerClearanceGeneration>::max()};
  MotionLedgerClearanceFailureInjector failure_injector;
};

struct MotionOperationStartResult
{
  MotionLedgerStatus status{MotionLedgerStatus::kInvalidArgument};
  std::optional<OperationTicket> ticket;
};

struct PendingOperationSnapshot
{
  OperationTicket ticket;
  PendingOperationPhase phase{PendingOperationPhase::kAwaitingCompletion};
  std::optional<ExecutionAttemptGeneration> execution_attempt_generation;
  std::optional<std::uint64_t> bound_request_fingerprint;
  std::optional<MotionOperationDeadlines> motion_deadlines;
  std::optional<MotionSubmissionAttemptId> submission_attempt_id;
  std::optional<MotionGoalAcceptedRecord> accepted_goal;
  std::optional<MotionCancelSubmissionRecord> cancel_submission;
  std::optional<MotionCancelSubmissionRecord> post_terminal_cancel_submission;
  std::optional<EffectiveMotionTimeout> effective_motion_timeout;
  bool goal_response_deadline_expired{false};
  bool public_deadline_expired{false};
  bool liveness_was_unknown{false};
  std::optional<MotionTerminalRecord> terminal;
  std::optional<MotionStopProofRecord> late_stop_proof;
  std::optional<DefinitelyNotSubmittedRecord> late_non_submission;
  std::optional<MotionIngressFaultRecord> ingress_fault;
  bool stop_proof_required{false};

  [[nodiscard]] bool operator==(const PendingOperationSnapshot &) const noexcept = default;
};

struct MotionLedgerAuthoritySnapshot
{
  std::optional<PendingOperationSnapshot> pending;
  MotionLedgerClearanceGeneration last_issued_clearance_generation{0U};
  MotionLedgerClearanceGeneration clearance_generation_limit{0U};
  bool clearance_generation_exhausted{false};
  OperationGeneration next_operation_generation{1U};
};

struct OperationCompletionDecision
{
  OperationCompletionDisposition disposition{OperationCompletionDisposition::kRejectUnknown};
  std::optional<OperationTicket> ticket;
};

enum class OperationEvidenceDisposition : std::uint8_t
{
  kDeliver,
  kDeadlineWon,
  kDiscardResolved,
  kRejectUnknown,
};

struct OperationEvidenceDecision
{
  OperationEvidenceDisposition disposition{OperationEvidenceDisposition::kRejectUnknown};
  std::optional<OperationTicket> ticket;
};

struct ReconciliationAttempt
{
  OperationTicket ticket;
  ReconciliationKind kind{ReconciliationKind::kReplayMutation};
  std::size_t attempt_number{0U};
  SteadyTime deadline{};
  SteadyTime window_deadline{};
};

enum class ReconciliationStartError : std::uint8_t
{
  kNone,
  kNotRequired,
  kAttemptAlreadyActive,
  kInvalidArgument,
  kStrategyNotAllowed,
  kExhausted,
};

struct ReconciliationStartResult
{
  ReconciliationStartError error{ReconciliationStartError::kNotRequired};
  std::optional<ReconciliationAttempt> attempt;
};

enum class ReconciliationCompletionDisposition : std::uint8_t
{
  kCompleted,
  kRejectUnknown,
  kAttemptTimedOut,
  kExhausted,
};

enum class ReconciliationEvidence : std::uint8_t
{
  kInconclusive,
  kMutationResponseRetained,
  kExactMutationStillApplied,
};

class PendingOperationLedger
{
public:
  explicit PendingOperationLedger(
    ReconciliationPolicy policy = {},
    MotionTimeoutFailureInjector motion_timeout_failure_injector = {},
    MotionLedgerClearanceConfig clearance_config = {});

  [[nodiscard]] MotionLedgerIssuerBinding issuer_binding() const noexcept;
  [[nodiscard]] MotionLedgerClearancePreparationDecision prepare_motion_clearance(
    const DefinitelyNotSubmittedRecord & record) const;
  [[nodiscard]] MotionLedgerClearancePreparationDecision prepare_motion_clearance(
    const MotionStopProofRecord & record) const;

  [[nodiscard]] OperationStartResult start(
    GoalGeneration goal_generation, RestockTaskCommand command, OperationEffect effect,
    std::string operation_id, SteadyTime now, SteadyTime absolute_deadline);

  [[nodiscard]] MotionOperationStartResult start_motion(
    GoalGeneration goal_generation, RestockTaskCommand command,
    std::string operation_id, ExecutionAttemptGeneration execution_attempt_generation,
    SteadyTime now, const MotionOperationDeadlines & deadlines);
  [[nodiscard]] MotionLedgerStatus abandon_unbound_motion(
    const OperationTicket & ticket,
    ExecutionAttemptGeneration execution_attempt_generation);
  [[nodiscard]] MotionLedgerStatus bind_motion_request(MotionRequestBindingReceipt receipt);
  [[nodiscard]] MotionLedgerStatus bind_motion_submission_attempt(
    const MotionOperationIdentity & identity,
    MotionSubmissionAttemptId submission_attempt_id);
  [[nodiscard]] MotionLedgerStatus bind_motion_accepted_goal(
    MotionGoalAcceptedReceipt receipt);
  [[nodiscard]] MotionLedgerStatus begin_motion_cancel(
    const MotionOperationIdentity & identity);
  [[nodiscard]] MotionLedgerStatus record_motion_cancel_submission(
    MotionCancelSubmissionReceipt receipt);
  [[nodiscard]] MotionLedgerClearDecision confirm_motion_not_submitted(
    DefinitelyNotSubmittedReceipt receipt,
    PreparedMotionLedgerClearance preparation);
  [[nodiscard]] MotionLedgerStatus record_motion_terminal(MotionTerminalEvidence evidence);
  [[nodiscard]] MotionLedgerClearDecision resolve_motion_stop_proof(
    MotionStopProofReceipt receipt,
    PreparedMotionLedgerClearance preparation);
  [[nodiscard]] MotionLedgerStatus record_motion_ingress_fault(
    MotionIngressFaultReceipt receipt);

  [[nodiscard]] MotionTimeoutCheckDecision check_motion_timeout(
    const MotionOperationIdentity & exact_live_identity,
    SteadyTime captured_at);
  [[nodiscard]] MotionTimeoutCheckDecision check_cleared_motion_timeout(
    MotionLedgerClearanceProof & clearance_proof,
    SteadyTime captured_at);

  [[nodiscard]] OperationTimeoutDisposition check_timeout(SteadyTime now);
  // Classifies intermediate evidence without retiring the exact read-only ticket. Multi-step
  // operations use one absolute deadline and call complete() only at their terminal boundary.
  [[nodiscard]] OperationEvidenceDecision classify_read_only_evidence(
    GoalGeneration goal_generation, OperationGeneration operation_generation,
    SteadyTime arrived_at) const;
  [[nodiscard]] OperationCompletionDecision complete(
    GoalGeneration goal_generation, OperationGeneration operation_generation,
    SteadyTime completed_at);
  [[nodiscard]] OperationCompletionDecision report_unknown_outcome(
    GoalGeneration goal_generation, OperationGeneration operation_generation,
    SteadyTime observed_at);
  [[nodiscard]] bool confirm_not_submitted(
    GoalGeneration goal_generation, OperationGeneration operation_generation);
  [[nodiscard]] bool resolve(
    GoalGeneration goal_generation, OperationGeneration operation_generation);
  [[nodiscard]] ReconciliationStartResult begin_reconciliation(
    GoalGeneration goal_generation, OperationGeneration operation_generation,
    ReconciliationKind kind, SteadyTime now);
  [[nodiscard]] ReconciliationCompletionDisposition complete_reconciliation(
    GoalGeneration goal_generation, OperationGeneration operation_generation,
    ReconciliationKind kind, std::size_t attempt_number, SteadyTime completed_at,
    ReconciliationEvidence evidence = ReconciliationEvidence::kInconclusive);
  [[nodiscard]] std::optional<OperationTicket> pending() const;
  [[nodiscard]] std::optional<PendingOperationSnapshot> snapshot() const;
  [[nodiscard]] MotionLedgerAuthoritySnapshot authority_snapshot() const;
  [[nodiscard]] std::optional<PendingOperationPhase> phase() const;
  [[nodiscard]] std::optional<ReconciliationKind> next_reconciliation_kind() const;
  [[nodiscard]] std::optional<ReconciliationAttempt> active_reconciliation() const;

private:
  struct PendingRecord
  {
    OperationTicket ticket;
    PendingOperationPhase phase{PendingOperationPhase::kAwaitingCompletion};
    SteadyTime reconciliation_deadline{};
    std::size_t reconciliation_attempts{0U};
    ReconciliationKind next_reconciliation_kind{ReconciliationKind::kReplayMutation};
    std::optional<ReconciliationAttempt> active_reconciliation;
    std::optional<ExecutionAttemptGeneration> execution_attempt_generation;
    std::optional<MotionOperationDeadlines> motion_deadlines;
    std::optional<MotionRequestBindingRecord> motion_binding;
    std::optional<MotionSubmissionAttemptId> submission_attempt_id;
    std::optional<MotionGoalAcceptedRecord> accepted_goal;
    std::optional<MotionCancelSubmissionRecord> cancel_submission;
    std::optional<MotionCancelSubmissionRecord> post_terminal_cancel_submission;
    std::optional<EffectiveMotionTimeout> effective_motion_timeout;
    bool goal_response_deadline_expired{false};
    bool public_deadline_expired{false};
    bool liveness_was_unknown{false};
    std::optional<MotionTerminalRecord> terminal;
    std::optional<MotionStopProofRecord> late_stop_proof;
    std::optional<DefinitelyNotSubmittedRecord> late_non_submission;
    std::optional<MotionIngressFaultRecord> ingress_fault;
  };

  [[nodiscard]] MotionLedgerStatus correlate_motion_locked(
    const MotionOperationIdentity & identity) const;
  struct MotionTimeoutTransitionPlan
  {
    PendingOperationPhase pre_phase{PendingOperationPhase::kAwaitingCompletion};
    PendingOperationPhase post_phase{PendingOperationPhase::kAwaitingCompletion};
    bool goal_response_deadline_expired{false};
    bool public_deadline_expired{false};
    bool liveness_was_unknown{false};
    std::optional<EffectiveMotionTimeout> effective_motion_timeout;
    OperationTimeoutDisposition disposition{OperationTimeoutDisposition::kNotDue};
    bool newly_effective{false};
  };
  struct MotionTimeoutPlanDecision
  {
    MotionLedgerStatus status{MotionLedgerStatus::kWrongPhase};
    std::optional<MotionTimeoutTransitionPlan> plan;
  };
  [[nodiscard]] MotionTimeoutPlanDecision plan_motion_timeout_locked(
    SteadyTime captured_at) const noexcept;
  void apply_motion_timeout_plan_locked(const MotionTimeoutTransitionPlan & plan) noexcept;
  void retain_motion_timeout_cause_locked(
    MotionTimeoutCause cause, SteadyTime first_effective_at) noexcept;
  void enter_motion_liveness_unknown_locked();
  [[nodiscard]] std::optional<PendingOperationSnapshot> snapshot_locked() const;

  std::shared_ptr<const MotionLedgerIssuerCore> issuer_;
  MotionLedgerClearanceGeneration clearance_generation_limit_;
  MotionLedgerClearanceGeneration last_issued_clearance_generation_{0U};
  MotionLedgerClearanceFailureInjector clearance_failure_injector_;
  ReconciliationPolicy policy_;
  MotionTimeoutFailureInjector motion_timeout_failure_injector_;
  bool policy_valid_{true};
  mutable std::mutex mutex_;
  OperationGeneration next_generation_{1};
  std::optional<PendingRecord> pending_;
};

}  // namespace restocker_task_executor
