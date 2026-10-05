// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_goal_context.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <utility>

#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool valid_goal_id(const CoordinatorGoalId & goal_id)
{
  return std::ranges::any_of(goal_id, [](std::uint8_t octet) {return octet != 0;});
}

[[nodiscard]] bool is_execution_command(RestockTaskCommand command) noexcept
{
  return command == RestockTaskCommand::kExecutePreGrasp ||
         command == RestockTaskCommand::kExecuteApproach ||
         command == RestockTaskCommand::kExecuteRetract ||
         command == RestockTaskCommand::kExecuteCarryStart ||
         command == RestockTaskCommand::kExecutePreInsert ||
         command == RestockTaskCommand::kExecuteInsert ||
         command == RestockTaskCommand::kExecuteRetreat;
}

[[nodiscard]] GoalContextResult failure(GoalContextErrorCode code, std::string detail)
{
  return {code, std::move(detail), std::nullopt};
}

[[nodiscard]] GoalContextResult grasp_failure(
  GraspCandidateErrorCode grasp_error, std::string detail)
{
  return {
    GoalContextErrorCode::kIdentityMismatch, std::move(detail), grasp_error};
}

[[nodiscard]] restocker_interfaces::msg::WorldStateSnapshot snapshot_message(
  const restocker_world_state::WorldStateSnapshot & snapshot)
{
  return restocker_world_state::snapshot_to_message(
    snapshot,
    restocker_world_state::SnapshotMessageOptions{
        "world", rclcpp::Time(std::int64_t{0}, RCL_ROS_TIME), true, true});
}

[[nodiscard]] bool valid_snapshot(const restocker_world_state::WorldStateSnapshot & snapshot)
{
  return static_cast<bool>(
    restocker_world_state::snapshot_from_message(snapshot_message(snapshot), "world"));
}

[[nodiscard]] std::string goal_id_hex(const CoordinatorGoalId & goal_id)
{
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << std::hex << std::setfill('0');
  for (const auto octet : goal_id) {
    stream << std::setw(2) << static_cast<unsigned int>(octet);
  }
  return stream.str();
}

[[nodiscard]] bool equivalent_snapshot(
  const restocker_world_state::WorldStateSnapshot & left,
  const restocker_world_state::WorldStateSnapshot & right)
{
  return snapshot_message(left) == snapshot_message(right);
}

[[nodiscard]] bool equivalent_reservation(
  const restocker_world_state::TaskReservation & left,
  const restocker_world_state::TaskReservation & right)
{
  return restocker_world_state::task_reservation_to_message(left) ==
         restocker_world_state::task_reservation_to_message(right);
}

[[nodiscard]] bool finite_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] std::string describe_candidate(
  const std::vector<GraspCandidate> & candidates, std::size_t index)
{
  if (index >= candidates.size()) {
    return "no remaining grasp candidate";
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "grasp candidate " << (index + 1U) << " of " << candidates.size()
         << " (approach yaw " << std::fixed << std::setprecision(4)
         << candidates[index].approach_yaw_rad << " rad, score " << candidates[index].score.total
         << ")";
  return stream.str();
}

}  // namespace

RestockGoalContext::RestockGoalContext(RestockGoalContextInit init)
: goal_id_(init.goal_id),
  generation_(init.generation),
  selection_request_(std::move(init.selection_request)),
  steady_started_(init.steady_started),
  simulation_started_(init.simulation_started),
  task_machine_(init.task_config),
  generation_quiescence_(std::move(init.generation_quiescence))
{
  if (!valid_goal_id(goal_id_) || generation_ == 0) {
    throw std::invalid_argument("goal ID and generation must be nonzero");
  }
  if (!generation_quiescence_) {
    throw std::invalid_argument("generation quiescence authority must be provided");
  }
  if (selection_request_.object_id && !*selection_request_.object_id) {
    throw std::invalid_argument("selected object ID must be nonzero");
  }
  if (selection_request_.lane_id && selection_request_.lane_id->value.empty()) {
    throw std::invalid_argument("selected lane ID must be nonempty");
  }
}

RestockGoalContext::~RestockGoalContext()
{
  clear_capability();
}

RestockTaskTransition RestockGoalContext::begin_task()
{
  return task_machine_.begin();
}

RestockTaskTransition RestockGoalContext::dispatch_task(
  RestockTaskEvent event, std::string detail,
  std::optional<OperationGeneration> operation_generation)
{
  if (event == RestockTaskEvent::kReservationAcquired) {
    return reject_dispatch(
      "reservation acquisition is recorded only by validated response retention");
  }
  if (event == RestockTaskEvent::kSelectionSuperseded &&
    (selection_ || reserve_request_ || reservation_))
  {
    return reject_dispatch(
      "a superseded selection must be discarded before the task re-selects");
  }
  if (event == RestockTaskEvent::kOperationSucceeded) {
    switch (task_machine_.status().state) {
      case RestockTaskState::kValidateScene:
        if (!initial_snapshot_) {
          return reject_dispatch("scene validation success requires an initial snapshot");
        }
        break;
      case RestockTaskState::kSelectPair:
        if (!selection_) {
          return reject_dispatch("pair-selection success requires a retained selection");
        }
        break;
      case RestockTaskState::kReserveTask:
        {
          const auto status = task_machine_.status();
          if (!reservation_ ||
            (!status.cancel_requested && !status.safe_abort_requested && !reservation_validation_))
          {
            return reject_dispatch("reservation success requires a validated capability");
          }
          break;
        }
      case RestockTaskState::kGenerateGrasps:
        {
          const auto status = task_machine_.status();
          const bool terminating = status.cancel_requested || status.safe_abort_requested;
          // Recovery from kCloseGripper/kVerifyGrasp re-enters here after the arm has already
          // moved (first_trajectory_may_have_started). A recovered regeneration must be allowed
          // to stage a new batch and plan the next free-space pre-grasp; the pre-motion path
          // still requires that no trajectory has started.
          const bool recovered = status.recovery_attempt > 0U;
          if (!reservation_ || (!terminating && !grasp_candidate_batch_) ||
            ((!recovered && status.first_trajectory_may_have_started) || status.object_held))
          {
            return reject_dispatch(
              "grasp staging success requires retained candidates or pre-motion termination");
          }
          break;
        }
      case RestockTaskState::kPlanPreGrasp:
      case RestockTaskState::kExecutePreGrasp:
      case RestockTaskState::kPlanApproach:
      case RestockTaskState::kExecuteApproach:
        {
          // Success is a clean pre-motion termination or a completed motion segment. Whether the
          // segment succeeded is the driver's call; the context only checks the task still owns
          // what it planned against and that nothing is grasped yet.
          const auto status = task_machine_.status();
          if (!reservation_ || !grasp_candidate_batch_ || status.object_held) {
            return reject_dispatch(
              "pre-grasp success requires a reservation and retained candidates");
          }
          break;
        }
      case RestockTaskState::kPlanRetract:
      case RestockTaskState::kExecuteRetract:
      case RestockTaskState::kObserveDestination:
        {
          // These run with the product held, so the object-held guard inverts here.
          const auto status = task_machine_.status();
          if (!reservation_ || !grasp_candidate_batch_ || !status.object_held) {
            return reject_dispatch(
              "post-grasp success requires a held object under reservation");
          }
          break;
        }
      case RestockTaskState::kOpenGripperForApproach:
      case RestockTaskState::kOpenGripperForEscape:
      case RestockTaskState::kCloseGripper:
      case RestockTaskState::kAttachTransaction:
        {
          // Jaw closing and attachment commit both precede the machine considering the product
          // held.
          const auto status = task_machine_.status();
          if (!reservation_ || !grasp_candidate_batch_ || status.object_held) {
            return reject_dispatch("grasp success requires an unattached reservation");
          }
          break;
        }
      case RestockTaskState::kOpenGripper:
      case RestockTaskState::kDetachTransaction:
        {
          // Releasing happens while the product is still held, against the retained placement.
          const auto status = task_machine_.status();
          if (!reservation_ || !placement_candidate_ || !status.object_held) {
            return reject_dispatch(
              "release success requires a held product and a retained placement");
          }
          break;
        }
      case RestockTaskState::kVerifyGrasp:
        {
          // Grasp is verified before the attachment transaction, so the machine does not yet
          // consider the object held; the driver checked the robot's own report.
          const auto status = task_machine_.status();
          if (!reservation_ || !grasp_candidate_batch_ || status.object_held) {
            return reject_dispatch("grasp verification requires an unattached reservation");
          }
          break;
        }
      case RestockTaskState::kPlanRetreat:
      case RestockTaskState::kExecuteRetreat:
      case RestockTaskState::kSurveyDestination:
      case RestockTaskState::kCommitDetachment:
      case RestockTaskState::kVerifyPlacement:
      case RestockTaskState::kUpdateInventory:
        {
          // After physical detachment the product is not held, but the semantic commit may be
          // pending, so the inserted placement must stay retained through survey, commit, and
          // verify. The §6 cleanup retreat (Card 051) is the exception: it runs before any
          // placement exists — sometimes still carrying the product — so only the reservation
          // is required for the two retreat legs; survey, commit and verify keep the full
          // post-placement guard.
          const auto status = task_machine_.status();
          const bool cleanup_retreat_leg =
            (status.task_deadline_exceeded || status.recoverable_skip) &&
            (status.state == RestockTaskState::kPlanRetreat ||
            status.state == RestockTaskState::kExecuteRetreat);
          const bool placement_retained = cleanup_retreat_leg || placement_candidate_;
          const bool released = cleanup_retreat_leg || !status.object_held;
          if (!reservation_ || !placement_retained || !released) {
            return reject_dispatch(
              "post-placement success requires a released product under reservation");
          }
          break;
        }
      case RestockTaskState::kGeneratePlacement:
      case RestockTaskState::kPlanCarryStart:
      case RestockTaskState::kExecuteCarryStart:
      case RestockTaskState::kPlanPreInsert:
      case RestockTaskState::kExecutePreInsert:
      case RestockTaskState::kPlanInsert:
      case RestockTaskState::kExecuteInsert:
        {
          // From placement generation onward the destination pose is the planning target: it must
          // be retained and the product held.
          const auto status = task_machine_.status();
          if (!reservation_ || !placement_candidate_ || !status.object_held) {
            return reject_dispatch(
              "placement success requires a retained placement candidate and a held object");
          }
          break;
        }
      case RestockTaskState::kRecover:
        {
          // Recovery resumes the interrupted segment, so the task must still own the reservation
          // and candidates it was planned against. Whether to resume is the driver's call on a
          // fresh observation; the context only checks the plan inputs survived.
          if (!reservation_ || !grasp_candidate_batch_) {
            return reject_dispatch(
              "recovery success requires a reservation and the candidates it replans against");
          }
          // Recovery right after retract or destination observation carries the product but has no
          // placement yet; GeneratePlacement is the next safe boundary. Later carry states enforce
          // the retained placement in their own cases.
          break;
        }
      case RestockTaskState::kReleaseTask:
        if (!release_proof_) {
          return reject_dispatch("reservation release success requires snapshot proof");
        }
        break;
      case RestockTaskState::kFault:
        break;
      default:
        return reject_dispatch(
          "operation success is not implemented by this coordinator context slice");
    }
  }
  const auto status = task_machine_.status();
  if (event == RestockTaskEvent::kExecutionOperationStarted ||
    event == RestockTaskEvent::kTrajectoryExecutionAccepted)
  {
    if (!operation_generation ||
      !retained_execution_operation_id(
        status.command, status.recovery_attempt, status.attempt))
    {
      return reject_dispatch(
        "execution events require an exact generation and retained operation identity");
    }
  } else if (operation_generation) {
    return reject_dispatch("operation generation is invalid for this task event");
  }
  return task_machine_.dispatch(event, std::move(detail), operation_generation);
}

RestockTaskTransition RestockGoalContext::task_status() const
{
  return task_machine_.status();
}

GoalContextResult RestockGoalContext::retain_initial_snapshot(
  restocker_world_state::WorldStateSnapshot snapshot)
{
  if (task_machine_.status().state != RestockTaskState::kValidateScene) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "initial snapshot is accepted only while validating the scene");
  }
  if (initial_snapshot_) {
    return failure(GoalContextErrorCode::kAlreadySet, "initial snapshot is already retained");
  }
  if (!valid_snapshot(snapshot)) {
    return failure(GoalContextErrorCode::kInvalidInput, "initial snapshot is malformed");
  }
  initial_snapshot_ = snapshot;
  latest_snapshot_ = std::move(snapshot);
  return {};
}

GoalContextResult RestockGoalContext::retain_latest_snapshot(
  restocker_world_state::WorldStateSnapshot snapshot)
{
  if (!initial_snapshot_) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "an initial snapshot must be retained before later snapshots");
  }
  if (!valid_snapshot(snapshot)) {
    return failure(GoalContextErrorCode::kInvalidInput, "latest snapshot is malformed");
  }
  if (snapshot.revision < evidence_revision_floor() ||
    (latest_snapshot_ && snapshot.revision < latest_snapshot_->revision))
  {
    return failure(
      GoalContextErrorCode::kRevisionRegression,
      "latest snapshot cannot precede retained authoritative evidence");
  }
  if (latest_snapshot_ && snapshot.revision == latest_snapshot_->revision) {
    if (!equivalent_snapshot(snapshot, *latest_snapshot_)) {
      return failure(
        GoalContextErrorCode::kIdentityMismatch,
        "same-revision snapshot contradicts retained authoritative state");
    }
    return {};
  }
  latest_snapshot_ = std::move(snapshot);
  adopt_observed_reservation_record();
  return {};
}

void RestockGoalContext::adopt_observed_reservation_record()
{
  // The capability token authorizes; the reservation record beside it is observed state that
  // advances with every committed mutation (attach -> Attached, detach -> Detached plus the
  // placement). Release and its read-back compare the whole record against the world, so a record
  // frozen at grant time is stale. Adopt only the same reservation, never regressing; leave the
  // rest for the release predicates to reject.
  if (!reservation_ || !latest_snapshot_ || !latest_snapshot_->active_reservation) {
    return;
  }
  const auto & observed = *latest_snapshot_->active_reservation;
  const auto & retained = reservation_->reservation;
  if (observed.reservation_id != retained.reservation_id ||
    observed.request_id != retained.request_id || observed.object_id != retained.object_id ||
    observed.destination_lane != retained.destination_lane ||
    observed.source_lane != retained.source_lane || observed.revision < retained.revision ||
    latest_snapshot_->revision < observed.revision ||
    latest_snapshot_->revision < reservation_->world_revision)
  {
    return;
  }
  // The capability's world revision names the world the record was observed in, and the record
  // must never be newer than it. Advance both together or neither, else the capability fails its
  // own validity check and cannot build the release request.
  reservation_->reservation = observed;
  reservation_->world_revision = latest_snapshot_->revision;
}

GoalContextResult RestockGoalContext::retain_selection(SelectedTaskPair selection)
{
  if (task_machine_.status().state != RestockTaskState::kSelectPair || !latest_snapshot_) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "selection is accepted only from a retained snapshot in select-pair state");
  }
  if (selection_) {
    return failure(GoalContextErrorCode::kAlreadySet, "task selection is already retained");
  }
  const auto object = latest_snapshot_->objects.find(selection.object_id);
  const auto lane = latest_snapshot_->lanes.find(selection.lane_id);
  if ((selection_request_.object_id && selection.object_id != *selection_request_.object_id) ||
    (selection_request_.lane_id && selection.lane_id != *selection_request_.lane_id) ||
    selection.snapshot_revision != latest_snapshot_->revision ||
    object == latest_snapshot_->objects.end() || lane == latest_snapshot_->lanes.end() ||
    selection.object_revision != object->second.revision ||
    selection.lane_revision != lane->second.revision)
  {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "selection is not bound to the retained authoritative snapshot");
  }
  selection_ = std::move(selection);
  return {};
}

GoalContextResult RestockGoalContext::rebind_selection_to_latest_snapshot()
{
  if (task_machine_.status().state != RestockTaskState::kGenerateGrasps || !selection_ ||
    !latest_snapshot_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "selection rebind requires a retained pair and snapshot in generate-grasps state");
  }
  const auto object = latest_snapshot_->objects.find(selection_->object_id);
  const auto lane = latest_snapshot_->lanes.find(selection_->lane_id);
  if (object == latest_snapshot_->objects.end() || lane == latest_snapshot_->lanes.end()) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "the retained selection is absent from the latest snapshot and cannot be rebound");
  }
  const bool stamps_changed =
    selection_->snapshot_revision != latest_snapshot_->revision ||
    selection_->object_revision != object->second.revision ||
    selection_->lane_revision != lane->second.revision;
  selection_->snapshot_revision = latest_snapshot_->revision;
  selection_->object_revision = object->second.revision;
  selection_->lane_revision = lane->second.revision;
  // When the stamps actually moved (verify adopted a newer snapshot), the previously staged
  // batch is refused against the old world: drop it so retain_grasp_candidate_batch can store
  // the regenerated candidates. A no-op rebind leaves the batch for generate success.
  if (stamps_changed && grasp_candidate_batch_) {
    grasp_candidate_batch_.reset();
    active_grasp_candidate_ = 0U;
    grasp_candidate_refusals_.clear();
  }
  return {};
}

GoalContextResult RestockGoalContext::retain_reserve_request(
  restocker_interfaces::srv::ReserveTask::Request request)
{
  if (task_machine_.status().state != RestockTaskState::kReserveTask || !selection_ ||
    !latest_snapshot_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "reserve request requires a retained selection in reserve-task state");
  }
  if (reserve_request_) {
    return failure(GoalContextErrorCode::kAlreadySet, "reserve request is already retained");
  }
  const auto checked = make_reserve_task_request(
    *latest_snapshot_, *selection_, request.request_id);
  if (!checked || checked.value() != request) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "reserve request differs from the exact retained snapshot and selection");
  }
  reserve_snapshot_ = *latest_snapshot_;
  reserve_request_ = std::move(request);
  return {};
}

ReserveResponseResult RestockGoalContext::retain_reserve_response(
  const restocker_interfaces::srv::ReserveTask::Response & response)
{
  if (task_machine_.status().state != RestockTaskState::kReserveTask || !reserve_request_ ||
    !selection_ || !reserve_snapshot_)
  {
    return {
      GoalContextErrorCode::kInvalidPhase,
      ReserveResponseDisposition::kCapabilityAcquired,
      "reservation capability requires its retained reserve request"};
  }
  auto validated = validate_reserve_task_response(
    *reserve_request_, *reserve_snapshot_, response);
  if (!validated) {
    // A precondition rejection is not a lineage violation: the world state evaluates both codes
    // only after proving no reservation is active, so nothing was mutated and our view of the
    // selected entities was stale. Discard the selection lineage and re-derive from fresh evidence.
    // If we already hold a capability, the same response is a contradiction: keep the mismatch.
    const bool superseded_precondition = !reservation_ &&
      validated.error().code == WorldStatePortErrorCode::kRemoteRejected &&
      (validated.error().remote_status ==
      restocker_interfaces::msg::WorldStateOperationStatus::REVISION_CONFLICT ||
      validated.error().remote_status ==
      restocker_interfaces::msg::WorldStateOperationStatus::PREDICATE_FAILED);
    if (superseded_precondition) {
      discard_superseded_selection();
      return {
        GoalContextErrorCode::kNone,
        ReserveResponseDisposition::kSelectionSuperseded,
        "reservation preconditions were superseded before the request landed: " +
        validated.error().detail};
    }
    return {
      GoalContextErrorCode::kIdentityMismatch,
      ReserveResponseDisposition::kCapabilityAcquired,
      "reservation response failed exact lineage validation: " + validated.error().detail};
  }
  if (reservation_) {
    const auto & candidate = validated.value();
    if (reservation_->token == candidate.token &&
      reservation_->world_revision == candidate.world_revision &&
      equivalent_reservation(reservation_->reservation, candidate.reservation))
    {
      return {};
    }
    return {
      GoalContextErrorCode::kIdentityMismatch,
      ReserveResponseDisposition::kCapabilityAcquired,
      "duplicate reservation response contradicts the retained capability"};
  }
  reservation_ = std::move(validated.value());
  const auto recorded = task_machine_.dispatch(RestockTaskEvent::kReservationAcquired);
  if (!recorded.accepted) {
    clear_capability();
    reservation_.reset();
    return {
      GoalContextErrorCode::kInvalidPhase,
      ReserveResponseDisposition::kCapabilityAcquired,
      "task machine rejected the acquired reservation capability"};
  }
  return {};
}

GoalContextResult RestockGoalContext::retain_reservation_validation_response(
  const restocker_interfaces::srv::ValidateTaskReservation::Response & response)
{
  if (task_machine_.status().state != RestockTaskState::kReserveTask || !reservation_) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "reservation validation requires a retained capability in reserve-task state");
  }
  if (reservation_validation_) {
    return failure(
      GoalContextErrorCode::kAlreadySet, "reservation validation proof is already retained");
  }
  const auto validated = validate_task_reservation_response(*reservation_, response);
  if (!validated) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "reservation validation response failed exact proof checks: " +
      validated.error().detail);
  }
  if (latest_snapshot_ && validated.value().world_revision < latest_snapshot_->revision) {
    return failure(
      GoalContextErrorCode::kRevisionRegression,
      "reservation validation response predates the latest retained snapshot");
  }
  reservation_validation_ = validated.value();
  return {};
}

GoalContextResult RestockGoalContext::retain_grasp_candidate_batch(
  GraspCandidateBatch batch, const GraspGenerationAuthority & authority,
  const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew)
{
  if (task_machine_.status().state != RestockTaskState::kGenerateGrasps ||
    !latest_snapshot_ || !selection_ || !reservation_ || !reservation_validation_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "grasp candidates require validated reservation authority in generate-grasps state");
  }
  if (grasp_candidate_batch_) {
    return failure(GoalContextErrorCode::kAlreadySet, "grasp candidates are already retained");
  }
  const auto validated = validate_grasp_candidate_batch(
    batch, *latest_snapshot_, *selection_, authority, now, maximum_object_age,
    maximum_robot_age, maximum_future_skew);
  if (!validated) {
    return grasp_failure(
      validated.error().code,
      "grasp candidate batch failed exact regeneration: " + validated.error().detail);
  }
  grasp_candidate_batch_ = std::move(batch);
  active_grasp_candidate_ = 0U;
  grasp_candidate_refusals_.clear();
  return {};
}

GoalContextResult RestockGoalContext::retain_refreshed_pregrasp_candidate_batch(
  const GraspGenerationAuthority & authority, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew)
{
  const auto status = task_machine_.status();
  if (status.state != RestockTaskState::kPlanPreGrasp || status.cancel_requested ||
    status.safe_abort_requested ||
    !latest_snapshot_ || !reservation_ || !reservation_validation_ ||
    !grasp_candidate_batch_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "pre-grasp refresh requires staged candidates and reservation authority in planning state");
  }
  if (refreshed_pregrasp_candidate_batch_) {
    return failure(
      GoalContextErrorCode::kAlreadySet,
      "refreshed pre-grasp candidates are already retained");
  }
  auto refreshed = refresh_pregrasp_candidate_batch(
    *grasp_candidate_batch_, *latest_snapshot_, reservation_->reservation,
    authority, now, maximum_object_age, maximum_robot_age, maximum_future_skew);
  if (!refreshed) {
    return grasp_failure(
      refreshed.error().code,
      "pre-grasp candidate refresh failed exact authority checks: " +
      refreshed.error().detail);
  }
  refreshed_pregrasp_candidate_batch_ = std::move(refreshed.value());
  return {};
}

GoalContextResult RestockGoalContext::retain_grasp_coupling()
{
  if (task_machine_.status().state != RestockTaskState::kVerifyGrasp || !latest_snapshot_ ||
    !selection_ || !reservation_ || !grasp_candidate_batch_ ||
    grasp_candidate_batch_->candidates.empty())
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "the grasp coupling requires retained candidates under a grasp-verification snapshot");
  }
  const auto object = latest_snapshot_->objects.find(selection_->object_id);
  if (object == latest_snapshot_->objects.end()) {
    return failure(
      GoalContextErrorCode::kInvalidInput,
      "the reserved product is absent from the grasp-verification snapshot");
  }
  // The arm still stands where the candidate placed it, so the product observation and the
  // planned grasp pose describe one configuration. Only in this state can they be composed.
  const auto * candidate = active_grasp_candidate();
  if (candidate == nullptr) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "the grasp coupling requires the candidate the task executed to still be the active one");
  }
  const Eigen::Isometry3d product_from_grasp_center = object->second.pose_in_world.inverse() *
    candidate->poses.world_from_grasp_center;
  if (!finite_rigid_transform(product_from_grasp_center)) {
    return failure(
      GoalContextErrorCode::kInvalidInput,
      "the observed grasp coupling is not a finite rigid transform");
  }
  // A bounded recovery re-entry reaches this state a second time in one goal (Milestone 10 §1,
  // Card 039), so the coupling a previous verification retained describes that earlier grasp —
  // an earlier snapshot, sometimes a different candidate. The accepted verification in front of
  // us re-derives it rather than refusing as already retained; a verification that composes the
  // same transform from the same revisions changes nothing, so the value is left alone.
  const bool unchanged = grasp_coupling_ &&
    grasp_coupling_->source_world_revision == latest_snapshot_->revision &&
    grasp_coupling_->source_object_revision == object->second.revision &&
    grasp_coupling_->product_from_grasp_center.matrix().isApprox(
    product_from_grasp_center.matrix(), 1.0e-12);
  if (!unchanged) {
    grasp_coupling_ = GraspCoupling{
      product_from_grasp_center, latest_snapshot_->revision, object->second.revision};
  }
  ++grasp_coupling_ordinal_;
  return {};
}

GoalContextResult RestockGoalContext::retain_placement_candidate(PlacementCandidate candidate)
{
  if (task_machine_.status().state != RestockTaskState::kGeneratePlacement ||
    !latest_snapshot_ || !selection_ || !reservation_ || !task_machine_.status().object_held)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "placement candidates require a held object in generate-placement state");
  }
  if (placement_candidate_) {
    return failure(GoalContextErrorCode::kAlreadySet, "a placement candidate is already retained");
  }
  if (candidate.selection.lane_id != selection_->lane_id ||
    candidate.selection.object_id != selection_->object_id)
  {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "placement candidate does not match the reserved object and destination lane");
  }
  placement_candidate_ = std::move(candidate);
  return {};
}

GoalContextResult RestockGoalContext::retain_physical_detach_proof(
  AttachmentStamp released_at, std::string retained_lease_token)
{
  if (task_machine_.status().state != RestockTaskState::kDetachTransaction ||
    !reservation_ || !placement_candidate_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "physical detach proof requires an active place-detach transaction");
  }
  if (physical_detach_released_at_) {
    return failure(
      GoalContextErrorCode::kAlreadySet, "physical detach proof is already retained");
  }
  if (released_at.zero()) {
    return failure(
      GoalContextErrorCode::kInvalidInput,
      "physical detach proof requires the boundary's release stamp");
  }
  if (retained_lease_token.empty()) {
    return failure(
      GoalContextErrorCode::kInvalidInput,
      "physical detach proof requires the retained planning-scene lease");
  }
  physical_detach_released_at_ = released_at;
  physical_detach_lease_token_ = std::move(retained_lease_token);
  return {};
}

GoalContextResult RestockGoalContext::retain_release_request(
  restocker_interfaces::srv::ReleaseTaskReservation::Request request)
{
  if (task_machine_.status().state != RestockTaskState::kReleaseTask || !reservation_) {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "release request requires an active reservation in release-task state");
  }
  if (release_request_) {
    return failure(GoalContextErrorCode::kAlreadySet, "release request is already retained");
  }
  const auto decoded = restocker_world_state::release_request_from_message(request);
  if (!decoded) {
    return failure(GoalContextErrorCode::kInvalidInput, decoded.error().detail);
  }
  const auto expected = make_release_task_reservation_request(
    *reservation_, request.operation_id, decoded.value().outcome,
    decoded.value().terminal_task_phase, decoded.value().terminal_fault_state);
  if (!expected || expected.value() != request) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "release request differs from the exact retained reservation capability");
  }
  release_request_ = std::move(request);
  return {};
}

GoalContextResult RestockGoalContext::retain_release_response(
  const restocker_interfaces::srv::ReleaseTaskReservation::Response & response)
{
  if (task_machine_.status().state != RestockTaskState::kReleaseTask || !release_request_ ||
    !reservation_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "release acknowledgement requires the retained release operation");
  }
  const auto validated = validate_release_task_reservation_response(*reservation_, response);
  if (!validated) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "release response failed acknowledgement validation: " + validated.error().detail);
  }
  if (latest_snapshot_ && validated.value().world_revision < latest_snapshot_->revision) {
    return failure(
      GoalContextErrorCode::kRevisionRegression,
      "release acknowledgement predates the latest retained snapshot");
  }
  if (release_acknowledgement_) {
    if (release_acknowledgement_->world_revision == validated.value().world_revision) {
      return {};
    }
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "duplicate release response contradicts the retained acknowledgement");
  }
  release_acknowledgement_ = validated.value();
  return {};
}

GoalContextResult RestockGoalContext::retain_released_snapshot(
  restocker_world_state::WorldStateSnapshot snapshot)
{
  if (task_machine_.status().state != RestockTaskState::kReleaseTask || !release_request_ ||
    !reservation_)
  {
    return failure(
      GoalContextErrorCode::kInvalidPhase,
      "release proof requires the retained release operation");
  }
  if (!valid_snapshot(snapshot)) {
    return failure(GoalContextErrorCode::kInvalidInput, "terminal snapshot is malformed");
  }
  if (snapshot.revision < evidence_revision_floor() ||
    (latest_snapshot_ && snapshot.revision < latest_snapshot_->revision))
  {
    return failure(
      GoalContextErrorCode::kRevisionRegression,
      "terminal snapshot cannot precede retained authoritative evidence");
  }
  if (latest_snapshot_ && snapshot.revision == latest_snapshot_->revision &&
    !equivalent_snapshot(snapshot, *latest_snapshot_))
  {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "same-revision terminal snapshot contradicts retained authoritative state");
  }
  const auto proof = validate_released_snapshot(
    *reservation_, *release_request_, release_acknowledgement_, snapshot);
  if (!proof) {
    return failure(
      GoalContextErrorCode::kIdentityMismatch,
      "terminal snapshot failed release validation: " + proof.error().detail);
  }
  latest_snapshot_ = std::move(snapshot);
  release_proof_ = proof.value();
  clear_capability();
  return {};
}

ReleaseReadbackResult RestockGoalContext::retain_release_readback(
  restocker_world_state::WorldStateSnapshot snapshot)
{
  if (task_machine_.status().state != RestockTaskState::kReleaseTask || !release_request_ ||
    !reservation_)
  {
    return {
      GoalContextErrorCode::kInvalidPhase, ReleaseReadbackDisposition::kReleased,
      "release readback requires the retained release operation"};
  }
  if (!valid_snapshot(snapshot)) {
    return {
      GoalContextErrorCode::kInvalidInput, ReleaseReadbackDisposition::kReleased,
      "release readback snapshot is malformed"};
  }
  if (snapshot.revision < evidence_revision_floor() ||
    (latest_snapshot_ && snapshot.revision < latest_snapshot_->revision))
  {
    return {
      GoalContextErrorCode::kRevisionRegression, ReleaseReadbackDisposition::kReleased,
      "release readback cannot precede retained authoritative evidence"};
  }
  if (latest_snapshot_ && snapshot.revision == latest_snapshot_->revision &&
    !equivalent_snapshot(snapshot, *latest_snapshot_))
  {
    return {
      GoalContextErrorCode::kIdentityMismatch, ReleaseReadbackDisposition::kReleased,
      "same-revision release readback contradicts retained authoritative state"};
  }
  if (snapshot.active_reservation) {
    if (!equivalent_reservation(
        *snapshot.active_reservation, reservation_->reservation))
    {
      return {
        GoalContextErrorCode::kIdentityMismatch, ReleaseReadbackDisposition::kReleased,
        "release readback contains a different active reservation"};
    }
    latest_snapshot_ = std::move(snapshot);
    return {
      GoalContextErrorCode::kNone,
      ReleaseReadbackDisposition::kExactReservationStillActive, {}};
  }
  const auto retained = retain_released_snapshot(std::move(snapshot));
  if (!retained) {
    return {retained.error, ReleaseReadbackDisposition::kReleased, retained.detail};
  }
  return {GoalContextErrorCode::kNone, ReleaseReadbackDisposition::kReleased, {}};
}

std::optional<std::string> RestockGoalContext::operation_id_for(CoordinatorMutationKind kind)
{
  const RestockTaskState state = task_machine_.status().state;
  const bool legal =
    (kind == CoordinatorMutationKind::kReserveTask && state == RestockTaskState::kReserveTask) ||
    (kind == CoordinatorMutationKind::kReleaseTask && state == RestockTaskState::kReleaseTask) ||
    (kind == CoordinatorMutationKind::kCommitAttachment &&
    state == RestockTaskState::kAttachTransaction) ||
    (kind == CoordinatorMutationKind::kSetPhysicalAttachment &&
    state == RestockTaskState::kDetachTransaction) ||
    (kind == CoordinatorMutationKind::kCommitDetachment &&
    state == RestockTaskState::kCommitDetachment);
  if (!legal || next_operation_ordinal_ == std::numeric_limits<std::uint64_t>::max() ||
    task_machine_.terminal())
  {
    return std::nullopt;
  }
  const auto retained = operation_ids_.find(kind);
  if (retained != operation_ids_.end()) {
    return retained->second;
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "restock/" << goal_id_hex(goal_id_) << '/' << to_string(kind) << '/' <<
    next_operation_ordinal_;
  std::string operation_id = stream.str();
  if (operation_id.size() > 128U) {
    return std::nullopt;
  }
  ++next_operation_ordinal_;
  operation_ids_.emplace(kind, operation_id);
  return operation_id;
}

std::optional<std::string> RestockGoalContext::prepare_execution_operation_id(
  RestockTaskCommand command, std::string * refusal)
{
  const auto refuse = [refusal](std::string detail) -> std::optional<std::string> {
    if (refusal) {
      *refusal = std::move(detail);
    }
    return std::nullopt;
  };
  const auto status = task_machine_.status();
  if (!is_execution_command(command)) {
    return refuse("the requested command is not an execution command");
  }
  if (status.command != command) {
    return refuse(
      "the task machine is in " + std::string(to_string(status.state)) +
      ", not the requested execution command");
  }
  if (status.attempt == 0U) {
    return refuse("the task attempt counter is 0");
  }
  if (status.current_execution_operation_generation != 0U) {
    return refuse(
      "execution operation generation " +
      std::to_string(status.current_execution_operation_generation) +
      " is still bound to this goal");
  }
  if (status.current_execution_may_have_started) {
    return refuse("the current execution operation may already have started");
  }
  if (status.cancel_requested || status.safe_abort_requested) {
    // Milestone 10 §6 (Card 051): the bounded cleanup retreat is the one execution a
    // termination may command — under the whole-task deadline and under a requested
    // recoverable skip, each visible on the transition and never after a user cancel / drain /
    // safe abort revoked it (review blocker: an operator's stop always wins). Every other
    // identity keeps Card 044's fail-closed refusal with its guard named.
    const bool cleanup_retreat = command == RestockTaskCommand::kExecuteRetreat &&
      !status.deadline_retreat_revoked &&
      (status.task_deadline_exceeded || status.recoverable_skip);
    if (!cleanup_retreat) {
      return refuse("termination is already latched");
    }
  }
  if (task_machine_.terminal()) {
    return refuse("the task machine is terminal");
  }
  const auto key = std::tuple{command, status.recovery_attempt, status.attempt};
  if (const auto retained = execution_operation_ids_.find(key);
    retained != execution_operation_ids_.end())
  {
    return retained->second;
  }
  std::ostringstream stream;
  stream.imbue(std::locale::classic());
  stream << "restock/" << goal_id_hex(goal_id_) << '/' << to_string(command) << '/' <<
    status.recovery_attempt << '/' << status.attempt;
  std::string operation_id = stream.str();
  if (operation_id.empty() || operation_id.size() > 128U) {
    return refuse("the generated operation identity is empty or longer than 128 bytes");
  }
  execution_operation_ids_.emplace(key, operation_id);
  return operation_id;
}

const std::string * RestockGoalContext::retained_execution_operation_id(
  RestockTaskCommand command, std::size_t recovery_attempt,
  std::size_t attempt) const noexcept
{
  const auto retained = execution_operation_ids_.find(
    std::tuple{command, recovery_attempt, attempt});
  return retained == execution_operation_ids_.end() ? nullptr : &retained->second;
}

const CoordinatorGoalId & RestockGoalContext::goal_id() const noexcept {return goal_id_;}

GoalGeneration RestockGoalContext::generation() const noexcept {return generation_;}

const SelectionRequest & RestockGoalContext::selection_request() const noexcept
{
  return selection_request_;
}

SteadyTime RestockGoalContext::steady_started() const noexcept {return steady_started_;}

const rclcpp::Time & RestockGoalContext::simulation_started() const noexcept
{
  return simulation_started_;
}

const std::shared_ptr<CoordinatorGenerationQuiescence> &
RestockGoalContext::generation_quiescence() const noexcept
{
  return generation_quiescence_;
}

const std::optional<restocker_world_state::WorldStateSnapshot> &
RestockGoalContext::initial_snapshot() const noexcept
{
  return initial_snapshot_;
}

const std::optional<restocker_world_state::WorldStateSnapshot> &
RestockGoalContext::latest_snapshot() const noexcept
{
  return latest_snapshot_;
}

const std::optional<SelectedTaskPair> & RestockGoalContext::selection() const noexcept
{
  return selection_;
}

const std::optional<restocker_interfaces::srv::ReserveTask::Request> &
RestockGoalContext::reserve_request() const noexcept
{
  return reserve_request_;
}

const std::optional<TaskReservationCapability> & RestockGoalContext::reservation() const noexcept
{
  return reservation_;
}

const std::optional<TaskReservationValidationProof> &
RestockGoalContext::reservation_validation() const noexcept
{
  return reservation_validation_;
}

const std::optional<GraspCandidateBatch> & RestockGoalContext::grasp_candidate_batch()
const noexcept
{
  return grasp_candidate_batch_;
}

const std::optional<GraspCandidateBatch> &
RestockGoalContext::refreshed_pregrasp_candidate_batch() const noexcept
{
  return refreshed_pregrasp_candidate_batch_;
}

const GraspCandidate * RestockGoalContext::active_grasp_candidate() const noexcept
{
  if (!grasp_candidate_batch_ ||
    active_grasp_candidate_ >= grasp_candidate_batch_->candidates.size())
  {
    return nullptr;
  }
  return &grasp_candidate_batch_->candidates[active_grasp_candidate_];
}

std::size_t RestockGoalContext::active_grasp_candidate_index() const noexcept
{
  return active_grasp_candidate_;
}

GraspFallthroughResult RestockGoalContext::refuse_active_grasp_candidate(std::string reason)
{
  // A candidate may change while its pre-grasp is planned, or after the arm reached it when
  // collision-checked approach planning rejects the straight line. Recovery then free-space plans
  // the next candidate's pre-grasp. An attach that changed neither world may reject the attempted
  // grasp for the same bounded fallthrough; later stages stay bound to the grasp actually
  // performed.
  const auto state = task_machine_.status().state;
  if (state != RestockTaskState::kPlanPreGrasp && state != RestockTaskState::kPlanApproach &&
    state != RestockTaskState::kAttachTransaction)
  {
    return {
      GraspFallthroughDisposition::kNotSelecting,
      "the grasp candidate in use is fixed outside pre-grasp, approach, or rolled-back attach"};
  }
  if (!grasp_candidate_batch_) {
    return {
      GraspFallthroughDisposition::kNotSelecting,
      "no grasp candidate batch is retained to refuse a candidate from"};
  }
  const auto & candidates = grasp_candidate_batch_->candidates;
  if (active_grasp_candidate_ >= candidates.size()) {
    return {
      GraspFallthroughDisposition::kCandidatesExhausted,
      "every grasp candidate was refused: " + grasp_candidate_refusal_summary()};
  }
  if (reason.empty()) {
    reason = "no reason was reported";
  }
  const std::string refusal = describe_candidate(candidates, active_grasp_candidate_) +
    " was refused: " + std::move(reason);
  grasp_candidate_refusals_.push_back(refusal);
  ++active_grasp_candidate_;
  if (active_grasp_candidate_ < candidates.size()) {
    return {
      GraspFallthroughDisposition::kNextCandidateActive,
      refusal + "; falling through to " +
      describe_candidate(candidates, active_grasp_candidate_)};
  }
  return {
    GraspFallthroughDisposition::kCandidatesExhausted,
    "every grasp candidate was refused: " + grasp_candidate_refusal_summary()};
}

std::string RestockGoalContext::grasp_candidate_refusal_summary() const
{
  std::string summary;
  for (const auto & refusal : grasp_candidate_refusals_) {
    if (!summary.empty()) {
      summary += "; ";
    }
    summary += refusal;
  }
  return summary.empty() ? std::string("no refusal was recorded") : summary;
}

const std::optional<GraspCoupling> & RestockGoalContext::grasp_coupling() const noexcept
{
  return grasp_coupling_;
}

std::size_t RestockGoalContext::grasp_coupling_ordinal() const noexcept
{
  return grasp_coupling_ordinal_;
}

const std::optional<PlacementCandidate> &
RestockGoalContext::placement_candidate() const noexcept
{
  return placement_candidate_;
}

const std::optional<AttachmentStamp> &
RestockGoalContext::physical_detach_released_at() const noexcept
{
  return physical_detach_released_at_;
}

const std::optional<std::string> &
RestockGoalContext::physical_detach_lease_token() const noexcept
{
  return physical_detach_lease_token_;
}

void RestockGoalContext::retain_grasp_escape(GraspEscapeRecord record)
{
  grasp_escape_ = std::move(record);
}

void RestockGoalContext::clear_grasp_escape() noexcept
{
  grasp_escape_.reset();
}

const std::optional<GraspEscapeRecord> & RestockGoalContext::grasp_escape() const noexcept
{
  return grasp_escape_;
}

const std::optional<restocker_interfaces::srv::ReleaseTaskReservation::Request> &
RestockGoalContext::release_request() const noexcept
{
  return release_request_;
}

const std::optional<TaskReservationReleaseAcknowledgement> &
RestockGoalContext::release_acknowledgement() const noexcept
{
  return release_acknowledgement_;
}

const std::optional<TaskReservationReleaseProof> &
RestockGoalContext::release_proof() const noexcept
{
  return release_proof_;
}

RestockTaskTransition RestockGoalContext::reject_dispatch(std::string detail) const
{
  RestockTaskTransition rejected = task_machine_.status();
  rejected.accepted = false;
  rejected.fault = RestockTaskFault::kInvalidEvent;
  rejected.detail = std::move(detail);
  return rejected;
}

restocker_world_state::Revision RestockGoalContext::evidence_revision_floor() const noexcept
{
  restocker_world_state::Revision floor = 0;
  if (reservation_) {
    floor = std::max(floor, reservation_->world_revision);
  }
  if (reservation_validation_) {
    floor = std::max(floor, reservation_validation_->world_revision);
  }
  if (refreshed_pregrasp_candidate_batch_) {
    floor = std::max(floor, refreshed_pregrasp_candidate_batch_->source_world_revision);
  }
  if (release_acknowledgement_) {
    floor = std::max(floor, release_acknowledgement_->world_revision);
  }
  return floor;
}

void RestockGoalContext::discard_superseded_selection()
{
  selection_.reset();
  reserve_snapshot_.reset();
  reserve_request_.reset();
  // The re-derived request needs a fresh ID: reusing the discarded ID with new revisions is a
  // different payload under one operation ID, which the journal rejects as an idempotency conflict.
  operation_ids_.erase(CoordinatorMutationKind::kReserveTask);
}

void RestockGoalContext::clear_capability() noexcept
{
  if (!reservation_) {
    return;
  }
  std::fill(reservation_->token.begin(), reservation_->token.end(), '\0');
  reservation_->token.clear();
  if (release_request_) {
    std::fill(release_request_->token.begin(), release_request_->token.end(), '\0');
    release_request_->token.clear();
  }
}

const char * to_string(CoordinatorMutationKind kind) noexcept
{
  switch (kind) {
    case CoordinatorMutationKind::kReserveTask: return "reserve-task";
    case CoordinatorMutationKind::kReleaseTask: return "release-task";
    case CoordinatorMutationKind::kCheckpointTask: return "checkpoint-task";
    case CoordinatorMutationKind::kAcquireSceneLease: return "acquire-scene-lease";
    case CoordinatorMutationKind::kReleaseSceneLease: return "release-scene-lease";
    case CoordinatorMutationKind::kSetPhysicalAttachment: return "set-physical-attachment";
    case CoordinatorMutationKind::kApplyPlanningScene: return "apply-planning-scene";
    case CoordinatorMutationKind::kCommitAttachment: return "commit-attachment";
    case CoordinatorMutationKind::kCommitDetachment: return "commit-detachment";
  }
  return "unknown";
}

const char * to_string(GoalContextErrorCode code) noexcept
{
  switch (code) {
    case GoalContextErrorCode::kNone: return "none";
    case GoalContextErrorCode::kInvalidInput: return "invalid_input";
    case GoalContextErrorCode::kInvalidPhase: return "invalid_phase";
    case GoalContextErrorCode::kAlreadySet: return "already_set";
    case GoalContextErrorCode::kRevisionRegression: return "revision_regression";
    case GoalContextErrorCode::kIdentityMismatch: return "identity_mismatch";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
