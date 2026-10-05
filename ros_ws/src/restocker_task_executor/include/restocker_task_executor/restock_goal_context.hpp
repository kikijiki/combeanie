// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include <rclcpp/time.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_task_executor/attachment_port.hpp"
#include "restocker_task_executor/grasp_candidates.hpp"
#include "restocker_task_executor/placement_candidates.hpp"
#include "restocker_task_executor/pregrasp_planning_authority.hpp"
#include "restocker_task_executor/restock_coordinator_primitives.hpp"
#include "restocker_task_executor/task_selection.hpp"
#include "restocker_task_executor/world_state_port_contract.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

class CoordinatorGenerationQuiescence;

enum class GoalContextErrorCode : std::uint8_t
{
  kNone,
  kInvalidInput,
  kInvalidPhase,
  kAlreadySet,
  kRevisionRegression,
  kIdentityMismatch,
};

struct GoalContextResult
{
  GoalContextErrorCode error{GoalContextErrorCode::kNone};
  std::string detail;
  std::optional<GraspCandidateErrorCode> grasp_error;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return error == GoalContextErrorCode::kNone;
  }
};

// A reservation the world state refused without mutating anything. The refusal is evaluated
// after the active-reservation check, so it proves the task owns no reservation.
enum class ReserveResponseDisposition : std::uint8_t
{
  kCapabilityAcquired,
  kSelectionSuperseded,
};

struct ReserveResponseResult
{
  GoalContextErrorCode error{GoalContextErrorCode::kNone};
  ReserveResponseDisposition disposition{ReserveResponseDisposition::kCapabilityAcquired};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return error == GoalContextErrorCode::kNone;
  }
};

// The rigid transform from the held product to the grasp center. It is a property of how the
// jaws hold the product, so it is captured at the one instant both poses describe the same
// configuration: the arm standing at the planned grasp pose with a fresh observation of the
// product. Placement derives the insertion pose through this coupling, so recomposing it later
// from a moved product and a planning-time grasp pose would mix two unrelated instants.
struct GraspCoupling
{
  Eigen::Isometry3d product_from_grasp_center{Eigen::Isometry3d::Identity()};
  restocker_world_state::Revision source_world_revision{0};
  restocker_world_state::Revision source_object_revision{0};
};

// What refusing the grasp candidate the task is currently executing did.
//
// The generator orders several candidates and the task executes exactly one of them at a time.
// A refusal is the planner's verdict on that one candidate, so it moves the task to the next
// candidate rather than ending the goal, but only while the task is still choosing, and only
// until the batch runs out.
enum class GraspFallthroughDisposition : std::uint8_t
{
  // The refusal is recorded and a later candidate is now the active one.
  kNextCandidateActive,
  // The refusal is recorded and every candidate in the batch has now been refused.
  kCandidatesExhausted,
  // Nothing was recorded. The task is past the phase in which the grasp may still change, so
  // changing it here would leave the arm executing one candidate's poses against another's.
  kNotSelecting,
};

struct GraspFallthroughResult
{
  GraspFallthroughDisposition disposition{GraspFallthroughDisposition::kNotSelecting};
  // Names the candidate that was refused and why, and then either the candidate now in use or
  // every refusal collected for this goal. Never empty, and never a bare "planning failed":
  // this string is what the operator is told when the goal ends here.
  std::string detail;
};

enum class ReleaseReadbackDisposition : std::uint8_t
{
  kReleased,
  kExactReservationStillActive,
};

struct ReleaseReadbackResult
{
  GoalContextErrorCode error{GoalContextErrorCode::kNone};
  ReleaseReadbackDisposition disposition{ReleaseReadbackDisposition::kReleased};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return error == GoalContextErrorCode::kNone;
  }
};

// Milestone 10 §6 (Card 062): what the grasp escape needs from the candidate whose jaws closed,
// retained when the close is commanded because the candidate may be rejected before the jaws open.
struct GraspEscapeRecord
{
  Eigen::Isometry3d world_from_standoff_tool0{Eigen::Isometry3d::Identity()};
  double open_joint_position_m{0.0};
  std::string target_object_id;
};

struct RestockGoalContextInit
{
  CoordinatorGoalId goal_id{};
  GoalGeneration generation{0};
  SelectionRequest selection_request;
  SteadyTime steady_started{};
  rclcpp::Time simulation_started{std::int64_t{0}, RCL_ROS_TIME};
  RestockTaskConfig task_config;
  std::shared_ptr<CoordinatorGenerationQuiescence> generation_quiescence{};
};

// Mutable task-local state has exactly one owner: the coordinator orchestration pump. ROS
// callbacks carry immutable completions and never access this context directly.
class RestockGoalContext
{
public:
  explicit RestockGoalContext(RestockGoalContextInit init);
  ~RestockGoalContext();

  RestockGoalContext(const RestockGoalContext &) = delete;
  RestockGoalContext & operator=(const RestockGoalContext &) = delete;
  RestockGoalContext(RestockGoalContext &&) = delete;
  RestockGoalContext & operator=(RestockGoalContext &&) = delete;

  [[nodiscard]] RestockTaskTransition begin_task();
  [[nodiscard]] RestockTaskTransition dispatch_task(
    RestockTaskEvent event, std::string detail = {},
    std::optional<OperationGeneration> operation_generation = std::nullopt);
  [[nodiscard]] RestockTaskTransition task_status() const;

  [[nodiscard]] GoalContextResult retain_initial_snapshot(
    restocker_world_state::WorldStateSnapshot snapshot);
  [[nodiscard]] GoalContextResult retain_latest_snapshot(
    restocker_world_state::WorldStateSnapshot snapshot);
  [[nodiscard]] GoalContextResult retain_selection(SelectedTaskPair selection);
  // Rebinds the retained pair's revision stamps to the latest retained snapshot without changing
  // object_id or lane_id. Recovery from kCloseGripper/kVerifyGrasp re-enters kGenerateGrasps after
  // verify adopted a fresher snapshot (observes_fresh_evidence), so generate must see one world
  // revision; the pair identity stays the one selection already proved eligible. When this runs
  // under an active recovery (recovery_attempt > 0) the previously staged batch is dropped so a
  // regenerated batch can be retained. Refuses when the object or lane is absent from the latest
  // snapshot. The grasp coupling is deliberately not rebound here: it is re-derived by the next
  // accepted kVerifyGrasp (see retain_grasp_coupling), because after a re-entry it describes the
  // grasp an earlier verification saw.
  [[nodiscard]] GoalContextResult rebind_selection_to_latest_snapshot();
  [[nodiscard]] GoalContextResult retain_reserve_request(
    restocker_interfaces::srv::ReserveTask::Request request);
  // Retains an acquired capability, or discards the stale selection lineage so the task can
  // re-derive it from fresh evidence. Both dispositions are successful results.
  [[nodiscard]] ReserveResponseResult retain_reserve_response(
    const restocker_interfaces::srv::ReserveTask::Response & response);
  [[nodiscard]] GoalContextResult retain_reservation_validation_response(
    const restocker_interfaces::srv::ValidateTaskReservation::Response & response);
  [[nodiscard]] GoalContextResult retain_grasp_candidate_batch(
    GraspCandidateBatch batch, const GraspGenerationAuthority & authority,
    const rclcpp::Time & now,
    std::chrono::nanoseconds maximum_object_age,
    std::chrono::nanoseconds maximum_robot_age,
    std::chrono::nanoseconds maximum_future_skew);
  [[nodiscard]] GoalContextResult retain_refreshed_pregrasp_candidate_batch(
    const GraspGenerationAuthority & authority, const rclcpp::Time & now,
    std::chrono::nanoseconds maximum_object_age,
    std::chrono::nanoseconds maximum_robot_age,
    std::chrono::nanoseconds maximum_future_skew);
  // Retained at every accepted kVerifyGrasp completion, where the observed product pose and the
  // planned grasp pose are simultaneously valid, and nowhere else. A bounded recovery re-entry
  // runs the grasp sequence again (Milestone 10 §1, Card 039), so the next accepted verification
  // re-derives the coupling from its own snapshot and active candidate instead of refusing it as
  // already retained; a verification that composes the same transform from the same world and
  // object revisions is an idempotent no-op. Every other gate of the first capture is unchanged,
  // so a coupling is never authorised outside verification.
  [[nodiscard]] GoalContextResult retain_grasp_coupling();
  // Retained once, at kGeneratePlacement, against the lane evidence observed at the destination.
  [[nodiscard]] GoalContextResult retain_placement_candidate(PlacementCandidate candidate);
  // Captured when the physical place-detach half succeeds. The semantic half and the placement
  // evidence check both need the boundary's release stamp, and the semantic half reuses the
  // planning-scene lease the physical half retained.
  [[nodiscard]] GoalContextResult retain_physical_detach_proof(
    AttachmentStamp released_at, std::string retained_lease_token);
  [[nodiscard]] GoalContextResult retain_release_request(
    restocker_interfaces::srv::ReleaseTaskReservation::Request request);
  [[nodiscard]] GoalContextResult retain_release_response(
    const restocker_interfaces::srv::ReleaseTaskReservation::Response & response);
  [[nodiscard]] GoalContextResult retain_released_snapshot(
    restocker_world_state::WorldStateSnapshot snapshot);
  [[nodiscard]] ReleaseReadbackResult retain_release_readback(
    restocker_world_state::WorldStateSnapshot snapshot);

  // Returns the same ID for retries of a retained logical mutation.
  [[nodiscard]] std::optional<std::string> operation_id_for(CoordinatorMutationKind kind);

  // Creates the deterministic ID for the current cancelable-motion attempt. Later safety gates
  // use the non-mutating accessor and must never allocate an ID while validating submission.
  // When the identity cannot be retained, `refusal` (if non-null) names the guard that refused
  // it: a termination-driven refusal is reported as such instead of reading as a lost identity
  // (Milestone 10 §6, Card 044).
  [[nodiscard]] std::optional<std::string> prepare_execution_operation_id(
    RestockTaskCommand command, std::string * refusal = nullptr);
  [[nodiscard]] const std::string * retained_execution_operation_id(
    RestockTaskCommand command, std::size_t recovery_attempt,
    std::size_t attempt) const noexcept;

  [[nodiscard]] const CoordinatorGoalId & goal_id() const noexcept;
  [[nodiscard]] GoalGeneration generation() const noexcept;
  [[nodiscard]] const SelectionRequest & selection_request() const noexcept;
  [[nodiscard]] SteadyTime steady_started() const noexcept;
  [[nodiscard]] const rclcpp::Time & simulation_started() const noexcept;
  [[nodiscard]] const std::shared_ptr<CoordinatorGenerationQuiescence> &
  generation_quiescence() const noexcept;
  [[nodiscard]] const std::optional<restocker_world_state::WorldStateSnapshot> &
  initial_snapshot() const noexcept;
  [[nodiscard]] const std::optional<restocker_world_state::WorldStateSnapshot> &
  latest_snapshot() const noexcept;
  [[nodiscard]] const std::optional<SelectedTaskPair> & selection() const noexcept;
  [[nodiscard]] const std::optional<restocker_interfaces::srv::ReserveTask::Request> &
  reserve_request() const noexcept;
  [[nodiscard]] const std::optional<TaskReservationCapability> & reservation() const noexcept;
  [[nodiscard]] const std::optional<TaskReservationValidationProof> &
  reservation_validation() const noexcept;
  [[nodiscard]] const std::optional<GraspCandidateBatch> & grasp_candidate_batch() const noexcept;
  // The one candidate the goal is executing, or nullptr when no batch is retained or every
  // candidate has been refused.
  //
  // Every candidate-specific quantity the task uses, the pre-grasp, grasp and retract tool
  // poses, the jaw hold and clearance targets, the attachment coupling and the verified grasp
  // coupling, is read through this accessor and through nothing else. That is what makes it
  // structurally impossible for two of those consumers to act on different candidates: there is
  // no second way to name one.
  [[nodiscard]] const GraspCandidate * active_grasp_candidate() const noexcept;
  [[nodiscard]] std::size_t active_grasp_candidate_index() const noexcept;
  // Records why the active candidate cannot be used and moves to the next one.
  //
  // Admissible only in pre-grasp planning, which is the last phase in which nothing has been
  // committed to a particular grasp: the jaws have not opened for the approach, no trajectory
  // has been executed toward the product, and no coupling has been captured. Outside it the
  // selection is fixed and this refuses rather than silently moving the target from underneath
  // an arm that is already acting on it.
  [[nodiscard]] GraspFallthroughResult refuse_active_grasp_candidate(std::string reason);
  // Every refusal recorded for this goal, in the order they happened, as one sentence. This is
  // what a caller is owed when the goal ends because nothing could be grasped: a list of what
  // refused each candidate, never a bare "no plan was found".
  [[nodiscard]] std::string grasp_candidate_refusal_summary() const;
  [[nodiscard]] const std::optional<GraspCandidateBatch> &
  refreshed_pregrasp_candidate_batch() const noexcept;
  [[nodiscard]] const std::optional<GraspCoupling> & grasp_coupling() const noexcept;
  // How many accepted kVerifyGrasp completions of this goal have owned the coupling: 0 before the
  // first verification, then 1, 2, … in the order the verifications ran. A number above 1 means a
  // bounded re-entry reached verification again, and is what a receipt needs to say which verify
  // a coupling came from.
  [[nodiscard]] std::size_t grasp_coupling_ordinal() const noexcept;
  [[nodiscard]] const std::optional<PlacementCandidate> & placement_candidate() const noexcept;
  [[nodiscard]] const std::optional<AttachmentStamp> &
  physical_detach_released_at() const noexcept;
  [[nodiscard]] const std::optional<std::string> &
  physical_detach_lease_token() const noexcept;
  // Card 062: retained at the close, cleared once the escape executed.
  void retain_grasp_escape(GraspEscapeRecord record);
  void clear_grasp_escape() noexcept;
  [[nodiscard]] const std::optional<GraspEscapeRecord> & grasp_escape() const noexcept;
  [[nodiscard]] const std::optional<restocker_interfaces::srv::ReleaseTaskReservation::Request> &
  release_request() const noexcept;
  [[nodiscard]] const std::optional<TaskReservationReleaseAcknowledgement> &
  release_acknowledgement() const noexcept;
  [[nodiscard]] const std::optional<TaskReservationReleaseProof> &
  release_proof() const noexcept;

private:
  [[nodiscard]] RestockTaskTransition reject_dispatch(std::string detail) const;
  [[nodiscard]] restocker_world_state::Revision evidence_revision_floor() const noexcept;
  // Carries the retained capability's reservation record forward to the newest authoritative
  // observation of the same reservation.
  void adopt_observed_reservation_record();
  void clear_capability() noexcept;
  void discard_superseded_selection();

  CoordinatorGoalId goal_id_;
  GoalGeneration generation_;
  SelectionRequest selection_request_;
  SteadyTime steady_started_;
  rclcpp::Time simulation_started_;
  RestockTaskMachine task_machine_;
  std::shared_ptr<CoordinatorGenerationQuiescence> generation_quiescence_;
  std::uint64_t next_operation_ordinal_{1};
  std::map<CoordinatorMutationKind, std::string> operation_ids_;
  // recovery_attempt is part of the logical attempt identity because the task machine resets the
  // per-state attempt counter after deterministic recovery.
  std::map<std::tuple<RestockTaskCommand, std::size_t, std::size_t>, std::string>
  execution_operation_ids_;
  std::optional<restocker_world_state::WorldStateSnapshot> initial_snapshot_;
  std::optional<restocker_world_state::WorldStateSnapshot> latest_snapshot_;
  std::optional<SelectedTaskPair> selection_;
  std::optional<restocker_world_state::WorldStateSnapshot> reserve_snapshot_;
  std::optional<restocker_interfaces::srv::ReserveTask::Request> reserve_request_;
  std::optional<TaskReservationCapability> reservation_;
  std::optional<TaskReservationValidationProof> reservation_validation_;
  std::optional<GraspCandidateBatch> grasp_candidate_batch_;
  // Index into grasp_candidate_batch_->candidates of the candidate the task is executing. It
  // only ever moves forward, and only through refuse_active_grasp_candidate(), so a candidate
  // this goal has already proven unusable is never planned for a second time.
  std::size_t active_grasp_candidate_{0U};
  std::vector<std::string> grasp_candidate_refusals_;
  std::optional<GraspCandidateBatch> refreshed_pregrasp_candidate_batch_;
  std::optional<GraspCoupling> grasp_coupling_;
  std::size_t grasp_coupling_ordinal_{0U};
  std::optional<PlacementCandidate> placement_candidate_;
  std::optional<AttachmentStamp> physical_detach_released_at_;
  std::optional<std::string> physical_detach_lease_token_;
  std::optional<GraspEscapeRecord> grasp_escape_;
  std::optional<restocker_interfaces::srv::ReleaseTaskReservation::Request> release_request_;
  std::optional<TaskReservationReleaseAcknowledgement> release_acknowledgement_;
  std::optional<TaskReservationReleaseProof> release_proof_;
};

[[nodiscard]] const char * to_string(CoordinatorMutationKind kind) noexcept;
[[nodiscard]] const char * to_string(GoalContextErrorCode code) noexcept;

}  // namespace restocker_task_executor
