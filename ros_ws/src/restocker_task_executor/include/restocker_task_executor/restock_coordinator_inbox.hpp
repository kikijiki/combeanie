// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_task_executor/coordinator_inbox_deposit_result.hpp"
#include "restocker_task_executor/attachment_port.hpp"
#include "restocker_task_executor/gripper_port.hpp"
#include "restocker_task_executor/motion_port.hpp"
#include "restocker_task_executor/restock_coordinator_primitives.hpp"
#include "restocker_task_executor/task_selection.hpp"

namespace restocker_task_executor
{

class CoordinatorGenerationQuiescence;

enum class CoordinatorControlKind : std::uint8_t
{
  kCancelRequested,
  kDrainRequested,
  kSafeAbortRequested,
  kAuthorityFaultSafeAbortRequested,
  kStateDeadline,
  kTaskDeadline,
  kHeartbeat,
  kShutdown,
};

struct CoordinatorAcceptedGoal
{
  CoordinatorGoalId goal_id{};
  GoalGeneration goal_generation{0};
  SelectionRequest selection_request;
  SteadyTime steady_started{};
  rclcpp::Time simulation_started{std::int64_t{0}, RCL_ROS_TIME};
  std::optional<SteadyTime> drain_requested_at;
  std::shared_ptr<CoordinatorGenerationQuiescence> generation_quiescence{};
  // The accepted-goal envelope was sealed by the handoff transaction. If a drain won after the
  // envelope contents were fixed, the driver permits the missing inline notice only behind its
  // one-pump, immediately-next-delivery drain fence.
  bool sealed_pending_control_handoff{false};
};

struct CoordinatorControlEvent
{
  CoordinatorControlKind kind{CoordinatorControlKind::kHeartbeat};
  GoalGeneration goal_generation{0};
  OperationGeneration operation_generation{0};
  SteadyTime arrived_at{};
  std::string detail;
};

struct CoordinatorReconciliationCorrelation
{
  ReconciliationKind kind{ReconciliationKind::kReplayMutation};
  std::size_t attempt_number{0U};
};

template<typename Response>
struct CoordinatorServiceCompletion
{
  OperationCorrelation correlation;
  std::shared_ptr<const Response> response;
  std::string transport_error;
  SteadyTime arrived_at{};
  std::optional<CoordinatorReconciliationCorrelation> reconciliation;

  [[nodiscard]] bool has_response() const noexcept
  {
    return response != nullptr && transport_error.empty();
  }
};

using SnapshotCompletion = CoordinatorServiceCompletion<
  restocker_interfaces::srv::GetWorldState::Response>;
using ReserveTaskCompletion = CoordinatorServiceCompletion<
  restocker_interfaces::srv::ReserveTask::Response>;
using ValidateReservationCompletion = CoordinatorServiceCompletion<
  restocker_interfaces::srv::ValidateTaskReservation::Response>;
using ReleaseReservationCompletion = CoordinatorServiceCompletion<
  restocker_interfaces::srv::ReleaseTaskReservation::Response>;

// A motion result stamped with the coordinator's own steady clock. The port stays clock-free.
struct CoordinatorMotionCompletion
{
  MotionCompletion completion;
  SteadyTime arrived_at{};
};

struct CoordinatorGripperCompletion
{
  GripperCompletion completion;
  SteadyTime arrived_at{};
};

struct CoordinatorAttachmentCompletion
{
  AttachmentCompletion completion;
  SteadyTime arrived_at{};
};

// Destination-lane acquire at the retreat viewpoint. Completes the SurveyDestination prelude
// without closing the ledger entry; a published observation is followed by the usual snapshot.
struct CoordinatorLaneAcquireCompletion
{
  OperationCorrelation correlation;
  bool published{false};
  std::string detail;
  SteadyTime arrived_at{};
  std::optional<CoordinatorReconciliationCorrelation> reconciliation;
};

using CoordinatorInboxPushItem = std::variant<
  CoordinatorControlEvent,
  SnapshotCompletion,
  ReserveTaskCompletion,
  ValidateReservationCompletion,
  ReleaseReservationCompletion,
  CoordinatorMotionCompletion,
  CoordinatorGripperCompletion,
  CoordinatorAttachmentCompletion,
  CoordinatorLaneAcquireCompletion>;

using CoordinatorCleanupItem = std::variant<SnapshotCompletion, ReleaseReservationCompletion>;

struct CoordinatorOverflowMarker
{
  GoalGeneration goal_generation{0U};
  SteadyTime arrived_at{};
  std::string_view detail;
};

enum class CoordinatorCleanupEvidenceKind : std::uint8_t
{
  kSnapshot,
  kReleaseReservation,
};

struct CoordinatorCleanupEvidenceLossMarker
{
  CoordinatorCleanupEvidenceKind kind{CoordinatorCleanupEvidenceKind::kSnapshot};
  OperationCorrelation correlation;
  std::optional<CoordinatorReconciliationCorrelation> reconciliation;
  SteadyTime arrived_at{};
};

using CoordinatorInboxDelivery = std::variant<
  CoordinatorOverflowMarker,
  CoordinatorAcceptedGoal,
  CoordinatorControlEvent,
  SnapshotCompletion,
  ReserveTaskCompletion,
  ValidateReservationCompletion,
  ReleaseReservationCompletion,
  CoordinatorMotionCompletion,
  CoordinatorGripperCompletion,
  CoordinatorAttachmentCompletion,
  CoordinatorLaneAcquireCompletion,
  CoordinatorCleanupEvidenceLossMarker>;

struct CoordinatorInboxSnapshot
{
  std::size_t size{0U};
  std::size_t capacity{0U};
  std::size_t accepted_handoff_emergency_size{0U};
  std::size_t cleanup_emergency_size{0U};
  bool overflow_latched{false};
  bool overflow_notification_pending{false};
  bool cleanup_evidence_lost{false};
  bool generation_accounting_conflict{false};
  std::string overflow_detail;
};

// The inbox is the sole callback-to-pump ingress. Overflow is a persistent safety fault represented
// by an exact generation marker that takes precedence over items already queued. Planner and
// cleanup evidence loss use separate exact-correlation marker slots. Dedicated emergency lanes
// retain accepted handoff, MoveGroup response/cancel/result, and reservation-cleanup evidence after
// ordinary overflow has inhibited forward work.
class CoordinatorInbox
{
public:
  static constexpr std::size_t kOverflowMarkerCapacity = 1U;
  static constexpr std::size_t kAcceptedHandoffEmergencyCapacity = 2U;
  static constexpr std::size_t kCleanupEvidenceLossMarkerCapacity = 1U;
  static constexpr std::size_t kCleanupEmergencyCapacity = 4U;

  explicit CoordinatorInbox(std::size_t capacity);

  [[nodiscard]] CoordinatorInboxDepositResult push(CoordinatorInboxPushItem item);
  [[nodiscard]] CoordinatorInboxDepositResult push_accepted_goal(
    CoordinatorAcceptedGoal accepted,
    std::optional<CoordinatorControlEvent> retained_termination, SteadyTime arrived_at);
  [[nodiscard]] CoordinatorInboxDepositResult push_cleanup(CoordinatorCleanupItem item);
  [[nodiscard]] std::optional<CoordinatorInboxDelivery> try_pop();
  // Single-consumer requeue for deliveries previously returned by try_pop. Parks the delivery in a
  // bounded deferred lane that try_pop drains before ordinary/emergency lanes. Counts toward
  // snapshot / generation_empty / ordinary capacity. Moves from delivery only on success;
  // fail-closed on generation accounting conflict or deferred-lane exhaustion.
  [[nodiscard]] bool try_requeue(CoordinatorInboxDelivery & delivery) noexcept;
  [[nodiscard]] CoordinatorInboxSnapshot snapshot() const;
  [[nodiscard]] bool generation_empty(GoalGeneration generation) const;

private:
  enum class OverflowDiagnostic : std::size_t
  {
    kOrdinaryCapacity = 0U,
    kAcceptedHandoff = 1U,
    kCleanupEvidence = 2U,
    kCount = 3U,
  };

  void latch_overflow(OverflowDiagnostic diagnostic) noexcept;
  [[nodiscard]] static std::string_view detail_for(OverflowDiagnostic diagnostic) noexcept;
  [[nodiscard]] bool ensure_overflow_marker(
    GoalGeneration generation, SteadyTime arrived_at,
    OverflowDiagnostic diagnostic) noexcept;

  using CoordinatorInboxStorageItem = std::variant<
    CoordinatorAcceptedGoal,
    CoordinatorControlEvent,
    SnapshotCompletion,
    ReserveTaskCompletion,
    ValidateReservationCompletion,
    ReleaseReservationCompletion,
    CoordinatorMotionCompletion,
    CoordinatorGripperCompletion,
    CoordinatorAttachmentCompletion,
    CoordinatorLaneAcquireCompletion>;

  const std::size_t capacity_;
  mutable std::mutex mutex_;
  std::vector<std::optional<CoordinatorInboxStorageItem>> slots_;
  std::size_t head_{0U};
  std::size_t tail_{0U};
  std::size_t size_{0U};
  std::vector<std::optional<CoordinatorInboxDelivery>> deferred_slots_;
  std::size_t deferred_head_{0U};
  std::size_t deferred_size_{0U};
  std::optional<CoordinatorAcceptedGoal> accepted_handoff_emergency_;
  std::optional<CoordinatorControlEvent> accepted_handoff_termination_emergency_;
  std::array<std::optional<CoordinatorCleanupItem>, kCleanupEmergencyCapacity>
  cleanup_emergency_slots_;
  std::size_t cleanup_emergency_head_{0U};
  std::size_t cleanup_emergency_tail_{0U};
  std::size_t cleanup_emergency_size_{0U};
  std::optional<CoordinatorOverflowMarker> overflow_marker_;
  std::optional<CoordinatorCleanupEvidenceLossMarker> cleanup_evidence_loss_marker_;
  bool overflow_latched_{false};
  bool generation_accounting_conflict_{false};
  OverflowDiagnostic overflow_diagnostic_{OverflowDiagnostic::kOrdinaryCapacity};
};

}  // namespace restocker_task_executor
