// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <ranges>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>
#include <restocker_reasoner/fake_recovery_advisor.hpp>
#include <restocker_reasoner/recovery_audit_log.hpp>
#include <restocker_reasoner/strict_json.hpp>

#include "restocker_task_executor/campaign_motion_evidence.hpp"
#include "restocker_task_executor/restock_action_contract.hpp"
#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/generation_scoped_inbox_deposit.hpp"
#include "restocker_task_executor/restock_coordinator_driver.hpp"
#include "restocker_task_executor/transfer_path_constraints.hpp"
#include "attachment_port_harness.hpp"
#include "fake_attachment_port.hpp"
#include "fake_gripper_port.hpp"
#include "fake_motion_port.hpp"
#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
using CoordinatorInboxPushResult = CoordinatorInboxDepositStatus;
using CoordinatorCleanupDeposit = CoordinatorInboxDepositStatus;

[[nodiscard]] bool operator==(
  CoordinatorInboxDepositResult result,
  CoordinatorInboxDepositStatus status) noexcept
{
  if (result.status != status) {
    return false;
  }
  switch (status) {
    case CoordinatorInboxDepositStatus::kAccepted:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned;
    case CoordinatorInboxDepositStatus::kOverflowLatched:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEventOwned ||
             result.persistence == CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned;
    case CoordinatorInboxDepositStatus::kInhibited:
      return result.persistence == CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned ||
             result.persistence == CoordinatorInboxPersistenceStatus::kUnresolved;
    case CoordinatorInboxDepositStatus::kEvidenceLost:
      return result.persistence == CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned;
    case CoordinatorInboxDepositStatus::kEvidenceConflict:
      return result.persistence == CoordinatorInboxPersistenceStatus::kUnresolved;
  }
  return false;
}

void expect_deposit(
  CoordinatorInboxDepositResult actual, CoordinatorInboxDepositStatus status,
  CoordinatorInboxPersistenceStatus persistence)
{
  EXPECT_EQ(actual.status, status);
  EXPECT_EQ(actual.persistence, persistence);
}

namespace
{

using namespace std::chrono_literals;
using restocker_world_state::FaultState;
using restocker_world_state::GraspState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationStage;
using restocker_world_state::TaskPhase;
using restocker_world_state::TrackingState;

CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId value{};
  for (std::size_t index = 0; index < value.size(); ++index) {
    value[index] = static_cast<std::uint8_t>(seed + index);
  }
  return value;
}

struct TestGenerationGate
{
  std::unique_ptr<CoordinatorActiveFaultEpoch> epoch;
  std::shared_ptr<CoordinatorGenerationQuiescence> gate;
};

TestGenerationGate make_generation_gate(
  const CoordinatorGoalId & id, GoalGeneration generation,
  std::size_t capacity = 64U)
{
  auto epoch = std::make_unique<CoordinatorActiveFaultEpoch>(id, generation);
  auto gate = std::make_shared<CoordinatorGenerationQuiescence>(id, generation, capacity);
  return {std::move(epoch), std::move(gate)};
}

std::optional<CoordinatorAcceptedTerminalAckWitness> make_accepted_ack_witness(
  CoordinatorActiveFaultEpoch & epoch, GoalGeneration generation)
{
  CoordinatorDriverOutput output;
  output.kind = CoordinatorDriverOutputKind::kSucceeded;
  output.goal_generation = generation;
  output.outcome = RestockActionOutcome::kSucceeded;
  output.detail = "accepted";
  auto offered = epoch.offer_terminal(output);
  if (offered.disposition() != ActiveTerminalDisposition::kPublicationPrepared) {
    return std::nullopt;
  }
  auto publication = offered.take_publication_permit();
  if (!publication) {
    return std::nullopt;
  }
  auto returned = epoch.terminal_publication_returned(*publication);
  if (returned.status() != ActivePublicationReturnStatus::kAcknowledgementPrepared) {
    return std::nullopt;
  }
  auto acknowledgement = returned.take_ack_permit();
  if (!acknowledgement) {
    return std::nullopt;
  }
  auto completed = epoch.complete_terminal_ack(*acknowledgement, true);
  if (completed.status() != ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible) {
    return std::nullopt;
  }
  return completed.take_accepted_ack_witness();
}

bool seal_generation_gate(TestGenerationGate & authority, GoalGeneration generation)
{
  auto witness = make_accepted_ack_witness(*authority.epoch, generation);
  return witness && authority.gate->seal_after_accepted_ack(*witness) ==
         GenerationQuiescenceSealStatus::kSealed;
}

[[nodiscard]] bool termination_latch_succeeded(
  const GoalTerminationLatchDecision & decision) noexcept
{
  return decision.status() == GoalTerminationLatchStatus::kLatched ||
         decision.status() == GoalTerminationLatchStatus::kAlreadyLatched;
}

class CallBlocker final
{
public:
  ~CallBlocker() {release();}

  CallBlocker(const CallBlocker &) = delete;
  CallBlocker & operator=(const CallBlocker &) = delete;
  CallBlocker() = default;

  void arm()
  {
    std::lock_guard lock(mutex_);
    armed_ = true;
    blocked_ = false;
    released_ = false;
  }

  void intercept()
  {
    std::unique_lock lock(mutex_);
    if (!armed_) {
      return;
    }
    armed_ = false;
    blocked_ = true;
    condition_.notify_all();
    (void)condition_.wait_for(lock, 5s, [this]() {return released_;});
  }

  [[nodiscard]] bool wait_until_blocked(std::chrono::milliseconds timeout)
  {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [this]() {return blocked_;});
  }

  void release()
  {
    {
      std::lock_guard lock(mutex_);
      released_ = true;
    }
    condition_.notify_all();
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool armed_{false};
  bool blocked_{false};
  bool released_{false};
};

restocker_world_state::WorldStateSnapshot world_snapshot(std::uint64_t revision = 10U)
{
  restocker_world_state::WorldStateSnapshot value;
  value.revision = revision;
  value.robot.telemetry_time = rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME);
  value.robot.telemetry_source_id = "test/coordinator-driver";
  value.robot.telemetry_revision = revision - 1U;
  value.robot.revision = revision - 1U;
  value.robot.rail_position = 0.0;
  value.robot.joint_positions = {0.0, -0.2, 0.3, 0.0, 0.1, 0.0};
  value.robot.joint_velocities = {};
  value.robot.rail_velocity = 0.0;
  value.robot.gripper_joint_positions = {0.01, 0.01};
  value.robot.gripper_joint_velocities = {};
  value.robot.task_phase = TaskPhase::Idle;
  value.robot.fault_state = FaultState::None;

  restocker_world_state::TrackedObject object;
  object.id = ObjectId{17U};
  object.source_object_id = "sim:can_17";
  object.product_class = ProductClass::Can;
  object.orientation = ObjectOrientation::Upright;
  object.tracking_state = TrackingState::Tracked;
  object.grasp_state = GraspState::Free;
  object.observation_time = rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME);
  object.transition_time = object.observation_time;
  object.revision = revision - 3U;
  value.objects.emplace(object.id, object);

  restocker_world_state::ShelfLane lane;
  lane.id = LaneId{"lane_01"};
  lane.expected_product_class = ProductClass::Can;
  lane.depth_m = 0.85;
  lane.available_depth_m = 0.85;
  lane.revision = revision - 2U;
  value.lanes.emplace(lane.id, lane);
  return value;
}

SelectedTaskPair selected_pair(const restocker_world_state::WorldStateSnapshot & snapshot)
{
  return {ObjectId{17U},
    LaneId{"lane_01"},
    snapshot.revision,
    snapshot.objects.at(ObjectId{17U}).revision,
    snapshot.lanes.at(LaneId{"lane_01"}).revision,
    {0.033, 0.122}};
}

GraspGenerationAuthority grasp_authority(const restocker_world_state::WorldStateSnapshot & snapshot)
{
  return GraspGenerationAuthority{ParallelJawGeometry{0.042, 0.0, 0.035},
    Eigen::Isometry3d::Identity(),
    GraspGenerationConfig{{0.0, 0.5 * std::acos(-1.0)},
      0.0,
      0.080,
      0.10,
      0.12,
      0.005,
      0.001,
      0.035,
      snapshot.robot.rail_position,
      1.0,
      1.0,
      1.0,
      0.05}};
}

GraspCandidateResult<GraspCandidateBatch> generated_grasps(
  const restocker_world_state::WorldStateSnapshot & snapshot, const SelectedTaskPair & selected,
  const GraspGenerationAuthority & authority, const rclcpp::Time & now)
{
  return generate_grasp_candidate_batch(
    snapshot, selected, authority.gripper,
    authority.tool0_from_grasp_center, authority.config, now,
    500ms, 500ms, 50ms);
}

restocker_interfaces::srv::GetWorldState::Response::SharedPtr snapshot_response(
  const restocker_world_state::WorldStateSnapshot & snapshot)
{
  auto response = std::make_shared<restocker_interfaces::srv::GetWorldState::Response>();
  response->snapshot = restocker_world_state::snapshot_to_message(
    snapshot,
    restocker_world_state::SnapshotMessageOptions{
        "world", rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME), false, false});
  return response;
}

restocker_interfaces::srv::ReserveTask::Response::SharedPtr reserve_response(
  const restocker_interfaces::srv::ReserveTask::Request & request)
{
  auto response = std::make_shared<restocker_interfaces::srv::ReserveTask::Response>();
  response->status = restocker_world_state::operation_status_ok();
  response->world_revision = request.selected_snapshot_revision + 1U;
  response->token = "private-capability";
  restocker_world_state::TaskReservation reservation;
  reservation.reservation_id = 71U;
  reservation.request_id = request.request_id;
  reservation.object_id = ObjectId{request.object_id};
  reservation.object_source_id = "sim:can_17";
  reservation.product_class = ProductClass::Can;
  reservation.destination_lane = LaneId{request.destination_lane_id};
  // Captured from the selected snapshot's destination policy at grant, as the store does.
  reservation.destination_expected_product_class = ProductClass::Can;
  reservation.stage = ReservationStage::Reserved;
  reservation.created_revision = response->world_revision;
  reservation.admitted_robot_telemetry_revision = response->world_revision - 1;
  reservation.revision = response->world_revision;
  response->reservation = restocker_world_state::task_reservation_to_message(reservation);
  return response;
}

restocker_interfaces::srv::ReserveTask::Response::SharedPtr rejected_reserve_response(
  std::uint16_t code)
{
  auto response = std::make_shared<restocker_interfaces::srv::ReserveTask::Response>();
  response->status.code = code;
  response->status.detail = "selected snapshot entity revisions changed";
  return response;
}

restocker_interfaces::srv::ValidateTaskReservation::Response::SharedPtr validation_response(
  const restocker_interfaces::srv::ReserveTask::Response & reserve)
{
  auto response = std::make_shared<restocker_interfaces::srv::ValidateTaskReservation::Response>();
  response->status = restocker_world_state::operation_status_ok();
  response->world_revision = reserve.world_revision;
  response->has_reservation = true;
  response->reservation = reserve.reservation;
  return response;
}

restocker_world_state::WorldStateSnapshot reserved_world_snapshot(
  const restocker_interfaces::srv::ReserveTask::Response & reserve)
{
  auto value = world_snapshot(reserve.world_revision);
  const auto staged = world_snapshot();
  value.objects.at(ObjectId{17U}).revision = staged.objects.at(ObjectId{17U}).revision;
  value.lanes.at(LaneId{"lane_01"}).revision = staged.lanes.at(LaneId{"lane_01"}).revision;
  auto reservation = restocker_world_state::task_reservation_from_message(
    reserve.reservation,
    reserve.world_revision);
  EXPECT_TRUE(reservation) << (reservation ? "" : reservation.error().detail);
  if (reservation) {
    value.active_reservation = std::move(reservation.value());
  }
  return value;
}

class FakeWorldStatePort final : public WorldStateCoordinatorPort
{
public:
  template<typename Service>
  struct Call
  {
    OperationCorrelation correlation;
    std::shared_ptr<typename Service::Request> request;
    CompletionCallback<Service> callback;
    WorldStateRequestHandle handle;
  };

  WorldStateServiceReadiness readiness() const override {return {true, true, true, true, true};}

  AsyncSendResult get_snapshot(
    OperationCorrelation correlation,
    std::shared_ptr<GetSnapshot::Request> request,
    CompletionCallback<GetSnapshot> callback) override
  {
    if (before_snapshot) {
      auto hook = std::exchange(before_snapshot, {});
      hook();
    }
    if (reject_snapshot_once) {
      reject_snapshot_once = false;
      return {AsyncSendErrorCode::kTransportRejected, std::nullopt, "snapshot rejected"};
    }
    return enqueue<GetSnapshot>(
      snapshots, WorldStateServiceKind::kGetSnapshot, correlation,
      std::move(request), std::move(callback));
  }

  AsyncSendResult reserve_task(
    OperationCorrelation correlation,
    std::shared_ptr<ReserveTask::Request> request,
    CompletionCallback<ReserveTask> callback) override
  {
    if (before_reserve) {
      auto hook = std::exchange(before_reserve, {});
      hook();
    }
    if (reject_reserve_once) {
      reject_reserve_once = false;
      return {AsyncSendErrorCode::kTransportRejected, std::nullopt, "reserve rejected"};
    }
    return enqueue<ReserveTask>(
      reservations, WorldStateServiceKind::kReserveTask, correlation,
      std::move(request), std::move(callback));
  }

  AsyncSendResult validate_reservation(
    OperationCorrelation correlation,
    std::shared_ptr<ValidateReservation::Request> request,
    CompletionCallback<ValidateReservation> callback) override
  {
    return enqueue<ValidateReservation>(
      validations, WorldStateServiceKind::kValidateReservation,
      correlation, std::move(request), std::move(callback));
  }

  AsyncSendResult release_reservation(
    OperationCorrelation correlation,
    std::shared_ptr<ReleaseReservation::Request> request,
    CompletionCallback<ReleaseReservation> callback) override
  {
    ++release_attempts;
    if (reject_release_count > 0U) {
      --reject_release_count;
      return {AsyncSendErrorCode::kTransportRejected, std::nullopt, "release rejected"};
    }
    return enqueue<ReleaseReservation>(
      releases, WorldStateServiceKind::kReleaseReservation,
      correlation, std::move(request), std::move(callback));
  }

  AsyncSendResult validate_execution_authority(
    OperationCorrelation correlation,
    std::shared_ptr<ValidateExecutionAuthority::Request> request,
    CompletionCallback<ValidateExecutionAuthority> callback) override
  {
    return enqueue<ValidateExecutionAuthority>(
      execution_authority_validations, WorldStateServiceKind::kValidateExecutionAuthority,
      correlation, std::move(request), std::move(callback));
  }

  bool remove_pending_request(const WorldStateRequestHandle & handle) override
  {
    removed_request_ids.push_back(handle.request_id);
    return true;
  }

  template<typename Service>
  void complete_front(
    std::deque<Call<Service>> & calls,
    std::shared_ptr<typename Service::Response> response,
    std::string transport_error = {})
  {
    ASSERT_FALSE(calls.empty());
    auto call = std::move(calls.front());
    calls.pop_front();
    call.callback(
      AsyncServiceCompletion<typename Service::Response>{
          call.correlation, std::move(response), std::move(transport_error)});
  }

  template<typename Service>
  void complete_at(
    std::deque<Call<Service>> & calls, std::size_t index,
    std::shared_ptr<typename Service::Response> response,
    std::string transport_error = {})
  {
    ASSERT_LT(index, calls.size());
    auto position = calls.begin() + static_cast<std::ptrdiff_t>(index);
    auto call = std::move(*position);
    calls.erase(position);
    call.callback(
      AsyncServiceCompletion<typename Service::Response>{
          call.correlation, std::move(response), std::move(transport_error)});
  }

  std::deque<Call<GetSnapshot>> snapshots;
  std::deque<Call<ReserveTask>> reservations;
  std::deque<Call<ValidateReservation>> validations;
  std::deque<Call<ReleaseReservation>> releases;
  std::deque<Call<ValidateExecutionAuthority>> execution_authority_validations;
  std::vector<std::int64_t> removed_request_ids;
  std::function<void()> before_snapshot;
  std::function<void()> before_reserve;
  bool reject_snapshot_once{false};
  bool reject_reserve_once{false};
  std::size_t reject_release_count{0U};
  std::size_t release_attempts{0U};

private:
  template<typename Service>
  AsyncSendResult enqueue(
    std::deque<Call<Service>> & calls, WorldStateServiceKind kind,
    OperationCorrelation correlation,
    std::shared_ptr<typename Service::Request> request,
    CompletionCallback<Service> callback)
  {
    WorldStateRequestHandle handle{correlation, kind, next_request_id_++};
    calls.push_back(Call<Service>{correlation, std::move(request), std::move(callback), handle});
    return {AsyncSendErrorCode::kNone, handle, {}};
  }

  std::int64_t next_request_id_{1};
};
class RestockCoordinatorDriverTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    admission_.update_readiness(true);
    const auto receipt = admission_.reserve(goal_id());
    ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
    generation_ = receipt.generation;
    ASSERT_TRUE(admission_.activate(goal_id(), generation_));
    accepted_gate_ = make_generation_gate(goal_id(), generation_);

    install_driver(generated_grasps);
  }

  // Stands in for the node's tf-backed generator. It only has to be self-consistent: the driver
  // never inspects the placement beyond retaining it and reading its tool0 poses. The reservation
  // must arrive and name the reserved destination, or the generator received something other
  // than the capability the transfer was granted under.
  [[nodiscard]] static CoordinatorPlacementGenerator fake_placement_generator()
  {
    return
      [](const auto & snapshot, const auto & selection, const auto & grasp, const auto & coupling,
      const auto & reservation) {
        (void)snapshot;
        (void)coupling;
        if (reservation.destination_lane != selection.lane_id) {
          return PlacementCandidateResult<PlacementCandidate>::failure(
            PlacementCandidateError{
              PlacementCandidateErrorCode::InvalidLane,
              "placement generator did not receive the reserved destination capability"});
        }
        PlacementCandidate candidate;
        candidate.selection = selection;
        candidate.world_from_product = Eigen::Isometry3d::Identity();
        candidate.world_from_product.translation() = Eigen::Vector3d{0.0, 0.6, 0.8};
        const auto & poses = grasp.candidates.front().poses;
        // Distinct lane-mouth tool0 so carry-start mid-pose Y is not degenerate with retract.
        // Face +Y into the lane (transfer approach). Do not copy a top-down pregrasp frame,
        // or aisle egress cannot be derived when Y degenerates.
        candidate.poses.world_from_preinsertion_tool0 = Eigen::Isometry3d::Identity();
        candidate.poses.world_from_preinsertion_tool0.linear().col(0) = Eigen::Vector3d::UnitZ();
        candidate.poses.world_from_preinsertion_tool0.linear().col(1) =
          Eigen::Vector3d(1.0, 0.0, 0.0);
        candidate.poses.world_from_preinsertion_tool0.linear().col(2) =
          Eigen::Vector3d(0.0, 1.0, 0.0);
        candidate.poses.world_from_preinsertion_tool0.translation() =
          Eigen::Vector3d{-1.0, 0.25, 0.18};
        candidate.poses.world_from_final_tool0 = poses.world_from_grasp_tool0;
        candidate.poses.world_from_retreat_tool0 = poses.world_from_retract_tool0;
        return PlacementCandidateResult<PlacementCandidate>::success(std::move(candidate));
      };
  }

  void install_driver(
    CoordinatorGraspGenerator generator, CoordinatorSteadyNow async_evidence_now = {},
    CoordinatorTaskSelector selector = {}, MotionPort * motion = nullptr,
    GripperPort * gripper = nullptr, AttachmentPort * attachment = nullptr,
    restocker_reasoner::RecoveryAdvisorPort * advisor = nullptr,
    restocker_reasoner::RecoveryAuditLog * audit = nullptr,
    CoordinatorRetreatTargetProvider retreat_target = {},
    CoordinatorLaneEvidenceInvalidator invalidate_lane_evidence = {},
    CoordinatorDestinationObservationAcquirer acquire_destination_observation = {},
    CoordinatorPerceptionLivenessCheck perception_liveness = {})
  {
    RestockCoordinatorDriverConfig config;
    config.task.validation_timeout = 100ms;
    config.task.total_timeout = 1000ms;
    // Card 051: the deadline cleanup retreat's own budget, short so the expiry paths of the
    // Milestone 10 §6 contract are testable (the shipped default is 60000 ms).
    config.task.deadline_retreat_timeout = 500ms;
    config.reconciliation = {500ms, 100ms, 4U};
    config.perception_reacquire_timeout = reacquire_timeout_;
    // Card 060: short enough to expire inside the fixture's 1 s whole-task deadline.
    config.recovery_stop_settle_timeout = stop_settle_timeout_;
    config.recovery_stop_settle_poll = 50ms;
    // Card 049: the segment-deadline tests mirror the shipped task budgets through this fixture
    // member, not a parameter — the reach_* helpers install their own driver through this
    // function and would drop a per-call override.
    if (task_configure_) {
      task_configure_(config);
    }
    driver_ = std::make_unique<RestockCoordinatorDriver>(
      admission_, port_,
      selector ? std::move(selector) :
      CoordinatorTaskSelector{[](const auto & snapshot, const auto &) {
          return SelectionResult<SelectedTaskPair>::success(selected_pair(snapshot));
        }},
      [](const auto & snapshot) {
        return GraspCandidateResult<GraspGenerationAuthority>::success(grasp_authority(snapshot));
      },
      std::move(generator), [this]() {return now_;},
      []() {return rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME);}, config,
      std::move(async_evidence_now), fake_placement_generator(), motion, gripper, attachment,
      advisor, audit, std::move(retreat_target), std::move(invalidate_lane_evidence),
      std::move(acquire_destination_observation), std::move(perception_liveness));
  }

  // Drive the whole manipulation chain far enough that the gripper must close on the product.
  void reach_close_gripper(CoordinatorRetreatTargetProvider retreat_target = {})
  {
    reach_pregrasp_with_motion(std::move(retreat_target));
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
    now_ += 1ms;
    driver_->pump(now_);

    // The jaws open to straddle the product before the approach is planned.
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForApproach);
    ASSERT_TRUE(gripper_.outstanding());
    gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kSucceeded, "approach reached");
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kCloseGripper);
  }

  // Reach PlanPreGrasp with a motion backend composed. The staged batch is kept so tests can name
  // which candidate a submitted goal should be. The optional retreat target mirrors the
  // production node, which always wires the destination survey pose (Card 051 rung 1).
  void reach_pregrasp_with_motion(CoordinatorRetreatTargetProvider retreat_target = {})
  {
    install_driver(
      [this](const auto & snapshot, const auto & selected, const auto & authority,
      const auto & now) {
        auto generated = generated_grasps(snapshot, selected, authority, now);
        if (generated) {
          staged_batch_ = generated.value();
        }
        return generated;
      },
      {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr,
      std::move(retreat_target));
    reserve_response_ = reach_staging_boundary();
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
    driver_->pump(now_);
  }

  // Card 062: close, verify, and leave the attach transaction outstanding.
  void reach_outstanding_attach(CoordinatorRetreatTargetProvider retreat_target = {})
  {
    reach_close_gripper(std::move(retreat_target));
    ASSERT_TRUE(gripper_.outstanding());
    gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U);
    ASSERT_TRUE(reserve_response_);
    auto observed = reserved_world_snapshot(*reserve_response_);
    observed.revision = ++observation_revision_;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(observed));
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_TRUE(attachment_.outstanding());
  }

  // Card 062: roll the attach back, recover, open the jaws for the escape, and leave the
  // escape-carrying pre-grasp retry outstanding.
  void reach_escape_carrying_pregrasp()
  {
    reach_outstanding_attach();
    attachment_.complete(AttachmentOutcome::kRejected, "physical grasp tolerance was not met");
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
    auto observed = reserved_world_snapshot(*reserve_response_);
    observed.revision = ++observation_revision_;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(observed));
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForEscape);
    ASSERT_TRUE(gripper_.outstanding());
    gripper_.complete(GripperOutcome::kSucceeded, "jaws opened for the escape");
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
    ASSERT_TRUE(motion_.outstanding());
    ASSERT_TRUE(motion_.submission().goal.grasp_escape);
  }

  // Drive from close-gripper through attach + retract + destination observe so PreInsert is the
  // outstanding free-space carry.
  void reach_preinsert_with_motion()
  {
    reach_close_gripper();
    ASSERT_TRUE(gripper_.outstanding());
    gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_EQ(port_.snapshots.size(), 1U);
    ASSERT_TRUE(reserve_response_);
    auto held = reserved_world_snapshot(*reserve_response_);
    held.revision = ++observation_revision_;
    ASSERT_TRUE(held.active_reservation);
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(held));
    now_ += 1ms;
    driver_->pump(now_);
    ASSERT_TRUE(attachment_.outstanding());
    attachment_.complete(AttachmentOutcome::kSucceeded, "boundary and world state agree");
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kSucceeded, "retract reached");
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_EQ(port_.snapshots.size(), 1U);
    auto carrying = reserved_world_snapshot(*reserve_response_);
    carrying.revision = ++observation_revision_;
    const auto object_id = selected_pair(carrying).object_id;
    carrying.robot.held_object = object_id;
    carrying.objects.at(object_id).grasp_state = restocker_world_state::GraspState::Attached;
    ASSERT_TRUE(carrying.active_reservation);
    carrying.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(carrying));
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanCarryStart);
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kSucceeded, "carry-start reached");
    now_ += 1ms;
    driver_->pump(now_);

    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreInsert);
    ASSERT_TRUE(motion_.outstanding());
  }

  std::shared_ptr<restocker_interfaces::srv::ReserveTask::Response> reserve_response_;
  std::optional<GraspCandidateBatch> staged_batch_;
  restocker_world_state::Revision observation_revision_{500U};
  FakeMotionPort motion_;
  FakeGripperPort gripper_;
  FakeAttachmentPort attachment_;
  // Kept below the fixture's 1000 ms task total_timeout so a reacquire expiry, not the task
  // deadline, is what ends a deliberately unlive hold.
  std::chrono::milliseconds reacquire_timeout_{400ms};
  std::chrono::milliseconds stop_settle_timeout_{300ms};
  // Card 049: applied by install_driver after the fixture defaults. Tests that mirror the
  // shipped task budgets set it before any reach_* helper (which installs its own driver).
  std::function<void(RestockCoordinatorDriverConfig &)> task_configure_;

  void accept_and_request_snapshot()
  {
    ASSERT_EQ(
      driver_->inbox()->push_accepted_goal(
        CoordinatorAcceptedGoal{goal_id(),
          generation_,
          {},
          now_,
          rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
          std::nullopt,
          accepted_gate_.gate},
        std::nullopt, now_),
      CoordinatorInboxPushResult::kAccepted);
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U);
  }

  void expect_accepted_gate_rejected(
    const std::shared_ptr<CoordinatorGenerationQuiescence> & gate,
    const CoordinatorGoalId & accepted_goal_id,
    GoalGeneration accepted_generation)
  {
    ASSERT_EQ(
      driver_->inbox()->push_accepted_goal(
        CoordinatorAcceptedGoal{accepted_goal_id,
          accepted_generation,
          {},
          now_,
          rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
          std::nullopt,
          gate},
        std::nullopt, now_),
      CoordinatorInboxPushResult::kAccepted);

    driver_->pump(now_);

    const auto snapshot = driver_->snapshot();
    EXPECT_FALSE(snapshot.active);
    EXPECT_TRUE(snapshot.inhibited);
    EXPECT_FALSE(snapshot.pending_operation);
    EXPECT_EQ(snapshot.pending_transport_requests, 0U);
    EXPECT_FALSE(snapshot.reservation_capability_may_remain);
    EXPECT_TRUE(admission_.snapshot().inhibited);
    EXPECT_TRUE(port_.snapshots.empty());
    EXPECT_TRUE(port_.reservations.empty());
    EXPECT_TRUE(port_.validations.empty());
    EXPECT_TRUE(port_.releases.empty());
    EXPECT_TRUE(port_.execution_authority_validations.empty());
    EXPECT_TRUE(driver_->take_outputs().empty());
  }

  void expect_accepted_gate_rejected(const std::shared_ptr<CoordinatorGenerationQuiescence> & gate)
  {
    expect_accepted_gate_rejected(gate, goal_id(), generation_);
  }

  std::shared_ptr<restocker_interfaces::srv::ReserveTask::Response> reach_reservation_request()
  {
    accept_and_request_snapshot();
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(world_snapshot()));
    driver_->pump(now_);
    EXPECT_TRUE(port_.snapshots.empty());
    EXPECT_EQ(port_.reservations.size(), 1U);
    return reserve_response(*port_.reservations.front().request);
  }

  std::shared_ptr<restocker_interfaces::srv::ReserveTask::Response> reach_staging_boundary()
  {
    auto reserve = reach_reservation_request();
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
    driver_->pump(now_);
    EXPECT_EQ(port_.validations.size(), 1U);
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
      port_.validations, validation_response(*reserve));
    driver_->pump(now_);
    EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
    EXPECT_EQ(driver_->snapshot().staged_candidate_count, 2U);
    return reserve;
  }

  void complete_failed_safe_release(
    const restocker_interfaces::srv::ReserveTask::Response & reserve)
  {
    ASSERT_EQ(port_.releases.size(), 1U);
    const auto terminal_task_phase =
      static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase);
    const auto terminal_fault_state =
      static_cast<FaultState>(port_.releases.front().request->terminal_fault_state);
    auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    release->status = restocker_world_state::operation_status_ok();
    release->world_revision = reserve.world_revision + 1U;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U);
    auto released = world_snapshot(release->world_revision);
    released.robot.revision = released.revision;
    released.robot.task_phase = terminal_task_phase;
    released.robot.fault_state = terminal_fault_state;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(released));
    driver_->pump(now_);
  }

  // The four backends' completion steps, shared by the transfer helpers below and by tests that
  // must stop mid-chain (Card 044's deadline-at-the-retreat reproduction).
  void satisfy_snapshot(bool held, bool placed, bool resurveyed = false)
  {
    ASSERT_EQ(port_.snapshots.size(), 1U);
    ASSERT_TRUE(reserve_response_);
    auto value = reserved_world_snapshot(*reserve_response_);
    // Each observation is a later view, so it carries a later revision; the context refuses a
    // same-revision snapshot with different content.
    value.revision = ++observation_revision_;
    const auto object_id = selected_pair(value).object_id;
    if (held) {
      value.robot.held_object = object_id;
      value.objects.at(object_id).grasp_state = restocker_world_state::GraspState::Attached;
      ASSERT_TRUE(value.active_reservation);
      value.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
    } else if (placed) {
      value.lanes.at(selected_pair(value).lane_id).contents.push_back(object_id);
      ASSERT_TRUE(value.active_reservation);
      value.active_reservation->stage = restocker_world_state::ReservationStage::Detached;
      value.active_reservation->placed_in_destination = true;
    }
    if (resurveyed) {
      // Physical detach retains released_at at 1 s in the fake port; the retreat survey must
      // produce lane evidence strictly after that stamp and clear any post-placement invalidate.
      auto & lane = value.lanes.at(selected_pair(value).lane_id);
      lane.last_verified = rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME);
      lane.evidence_invalidated = false;
      lane.evidence_invalidated_at = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(value));
    driver_->pump(now_);
  }

  void satisfy_gripper()
  {
    ASSERT_TRUE(gripper_.outstanding());
    gripper_.complete(GripperOutcome::kSucceeded, "jaws reached target");
    now_ += 1ms;
    driver_->pump(now_);
  }

  void satisfy_motion()
  {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kSucceeded, "segment reached");
    now_ += 1ms;
    driver_->pump(now_);
  }

  void satisfy_attachment()
  {
    ASSERT_TRUE(attachment_.outstanding());
    attachment_.complete(AttachmentOutcome::kSucceeded, "boundary and world state agree");
    now_ += 1ms;
    driver_->pump(now_);
  }

  // Drive from the close-gripper boundary to the retreat motion being outstanding, i.e. the arm
  // has placed the product and the final segment of the transfer is in flight.
  void reach_retreat_with_motion()
  {
    satisfy_gripper();  // close on the hold width
    // Grasp verification runs before the attach commits: product still free.
    satisfy_snapshot(false, false);  // verify grasp
    satisfy_attachment();            // attach
    satisfy_motion();                // retract
    satisfy_snapshot(true, false);   // observe destination, now carrying the product
    satisfy_motion();                // carry-start (short Cartesian tray egress)
    satisfy_motion();                // pre-insert
    ASSERT_TRUE(motion_.outstanding());
    ASSERT_TRUE(motion_.submission().goal.required_postmotion_linear_egress_pose);
    ASSERT_TRUE(
      motion_.submission().goal.required_postmotion_linear_egress_gripper_joint_position_m);
    EXPECT_DOUBLE_EQ(
      *motion_.submission().goal.required_postmotion_linear_egress_gripper_joint_position_m,
      0.035);
    satisfy_motion();                // insert
    satisfy_gripper();               // open to release
    satisfy_attachment();            // physical detach (lease retained)
    ASSERT_TRUE(motion_.outstanding());
    EXPECT_EQ(motion_.submission().goal.label, "retreat");
    EXPECT_EQ(motion_.submission().goal.planning_scene_lease_token, "fake-retained-lease");
  }

  // Drive the happy path from the close-gripper boundary to a completed task, the fake answering as
  // the backend would on success.
  void complete_task_from_close_gripper()
  {
    reach_retreat_with_motion();
    satisfy_motion();                      // retreat to the destination-lane viewpoint
    satisfy_snapshot(false, false, true);  // survey destination after release
    satisfy_attachment();                  // semantic commit
    satisfy_snapshot(false, true);         // verify placement
    satisfy_snapshot(false, true);         // update inventory
  }

  // Same happy path as complete_task_from_close_gripper, but SurveyDestination waits for an
  // asynchronous acquire completion before the post-retreat snapshot may be requested.
  void complete_task_from_close_gripper_with_acquire(
    std::optional<CoordinatorDestinationObservationDone> & pending_acquire,
    std::optional<OperationCorrelation> & pending_correlation)
  {
    const auto satisfy_snapshot = [this](bool held, bool placed, bool resurveyed = false) {
      ASSERT_EQ(port_.snapshots.size(), 1U);
      ASSERT_TRUE(reserve_response_);
      auto value = reserved_world_snapshot(*reserve_response_);
      value.revision = ++observation_revision_;
      const auto object_id = selected_pair(value).object_id;
      if (held) {
        value.robot.held_object = object_id;
        value.objects.at(object_id).grasp_state = restocker_world_state::GraspState::Attached;
        ASSERT_TRUE(value.active_reservation);
        value.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
      } else if (placed) {
        value.lanes.at(selected_pair(value).lane_id).contents.push_back(object_id);
        ASSERT_TRUE(value.active_reservation);
        value.active_reservation->stage = restocker_world_state::ReservationStage::Detached;
        value.active_reservation->placed_in_destination = true;
      }
      if (resurveyed) {
        auto & lane = value.lanes.at(selected_pair(value).lane_id);
        lane.last_verified = rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME);
        lane.evidence_invalidated = false;
        lane.evidence_invalidated_at = rclcpp::Time(0, 0, RCL_ROS_TIME);
      }
      now_ += 1ms;
      port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
        port_.snapshots,
        snapshot_response(value));
      driver_->pump(now_);
    };
    const auto satisfy_gripper = [this]() {
      ASSERT_TRUE(gripper_.outstanding());
      gripper_.complete(GripperOutcome::kSucceeded, "jaws reached target");
      now_ += 1ms;
      driver_->pump(now_);
    };
    const auto satisfy_motion = [this]() {
      ASSERT_TRUE(motion_.outstanding());
      motion_.complete(MotionOutcome::kSucceeded, "segment reached");
      now_ += 1ms;
      driver_->pump(now_);
    };
    const auto satisfy_attachment = [this]() {
      ASSERT_TRUE(attachment_.outstanding());
      attachment_.complete(AttachmentOutcome::kSucceeded, "boundary and world state agree");
      now_ += 1ms;
      driver_->pump(now_);
    };

    satisfy_gripper();
    satisfy_snapshot(false, false);
    satisfy_attachment();
    satisfy_motion();
    satisfy_snapshot(true, false);
    satisfy_motion();  // carry-start
    satisfy_motion();  // pre-insert
    satisfy_motion();  // insert
    satisfy_gripper();
    satisfy_attachment();
    ASSERT_TRUE(motion_.outstanding());
    EXPECT_EQ(motion_.submission().goal.label, "retreat");
    EXPECT_EQ(motion_.submission().goal.planning_scene_lease_token, "fake-retained-lease");
    EXPECT_EQ(motion_.submission().goal.path, MotionPathKind::kFreeSpace);
    EXPECT_EQ(motion_.submission().goal.free_space_plan_candidates, 4U);
    ASSERT_TRUE(motion_.submission().goal.linear_egress_pose);
    EXPECT_FALSE(
      motion_.submission().goal.linear_egress_pose->isApprox(
        motion_.submission().goal.planning_frame_from_tool0));
    satisfy_motion();
    ASSERT_TRUE(pending_acquire);
    ASSERT_TRUE(pending_correlation);
    (*pending_acquire)(
      CoordinatorLaneAcquireCompletion{*pending_correlation, true, {}, now_, std::nullopt});
    pending_acquire.reset();
    pending_correlation.reset();
    now_ += 1ms;
    driver_->pump(now_);
    satisfy_snapshot(false, false, true);
    satisfy_attachment();
    satisfy_snapshot(false, true);
    satisfy_snapshot(false, true);
  }

  // Pumps `steps` times (50 ms apart) and returns every output produced meanwhile.
  std::vector<CoordinatorDriverOutput> pump_collect(int steps)
  {
    std::vector<CoordinatorDriverOutput> collected;
    for (int step = 0; step < steps; ++step) {
      now_ += 50ms;
      driver_->pump(now_);
      for (auto & output : driver_->take_outputs()) {
        collected.push_back(std::move(output));
      }
    }
    return collected;
  }

  static bool has_terminal_output(const std::vector<CoordinatorDriverOutput> & outputs)
  {
    return std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted ||
               output.kind == CoordinatorDriverOutputKind::kCanceled ||
               output.kind == CoordinatorDriverOutputKind::kInhibited;
      });
  }

  void acknowledge_terminal()
  {
    ASSERT_TRUE(driver_->snapshot().terminal_output_emitted);
    ASSERT_TRUE(driver_->acknowledge_terminal_delivery(generation_));
  }

  void cancel_at_staging_and_complete_release()
  {
    const auto reserve = reach_staging_boundary();
    ASSERT_TRUE(
      termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{
          CoordinatorControlKind::kCancelRequested, generation_, 0U, now_, "test cancel"}),
      CoordinatorInboxPushResult::kAccepted);
    driver_->pump(now_);
    ASSERT_EQ(port_.releases.size(), 1U);
    complete_failed_safe_release(*reserve);
    ASSERT_TRUE(driver_->snapshot().terminal_output_emitted);
  }

  void admit_next_goal()
  {
    admission_.update_readiness(true);
    const auto receipt = admission_.reserve(goal_id());
    ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
    generation_ = receipt.generation;
    ASSERT_TRUE(admission_.activate(goal_id(), generation_));
    accepted_gate_ = make_generation_gate(goal_id(), generation_);
  }

  GoalAdmissionSlot admission_;
  FakeWorldStatePort port_;
  SteadyTime now_{};
  GoalGeneration generation_{0U};
  TestGenerationGate accepted_gate_;
  std::unique_ptr<RestockCoordinatorDriver> driver_;
  std::optional<PreGraspPlanningAuthority> last_planning_authority_;
  std::shared_ptr<restocker_interfaces::srv::ReserveTask::Response> last_reserve_;
  restocker_world_state::Revision last_world_authority_revision_{0U};
};

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGoalWithoutGenerationGate)
{
  expect_accepted_gate_rejected(nullptr);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithSameGenerationAndWrongGoalId)
{
  auto foreign = make_generation_gate(goal_id(41U), generation_);
  expect_accepted_gate_rejected(foreign.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithWrongGeneration)
{
  auto foreign = make_generation_gate(goal_id(), generation_ + 1U);
  expect_accepted_gate_rejected(foreign.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsMatchingForeignEnvelopeAndGateAgainstAdmission)
{
  const auto foreign_goal_id = goal_id(42U);
  auto foreign = make_generation_gate(foreign_goal_id, generation_);
  expect_accepted_gate_rejected(foreign.gate, foreign_goal_id, generation_);
}

TEST_F(RestockCoordinatorDriverTest, RejectsForeignEnvelopeAgainstMatchingAdmissionAndGate)
{
  expect_accepted_gate_rejected(accepted_gate_.gate, goal_id(), generation_ + 1U);
}

TEST_F(RestockCoordinatorDriverTest, RejectsSealedAcceptedGate)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  ASSERT_TRUE(seal_generation_gate(non_pristine, generation_));
  ASSERT_TRUE(non_pristine.gate->snapshot().sealed);
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsFailedAcceptedGate)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  ASSERT_EQ(
    non_pristine.gate->mark_synchronization_failure(),
    GenerationSynchronizationFailureStatus::kMarkedFailed);
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsUnresolvedAcceptedGate)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  auto beginning = non_pristine.gate->begin_deposit(goal_id(), generation_);
  ASSERT_EQ(beginning.status(), GenerationDepositBeginStatus::kStarted);
  auto permit = beginning.take_deposit_permit();
  ASSERT_TRUE(permit);
  ASSERT_EQ(
    non_pristine.gate->complete_deposit(*permit, GenerationDepositCompletion::kUnknown),
    GenerationDepositCompletionStatus::kCompletedUnresolved);
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithActiveDeposit)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  auto beginning = non_pristine.gate->begin_deposit(goal_id(), generation_);
  ASSERT_EQ(beginning.status(), GenerationDepositBeginStatus::kStarted);
  auto permit = beginning.take_deposit_permit();
  ASSERT_TRUE(permit);
  ASSERT_EQ(non_pristine.gate->snapshot().active_deposit_count, 1U);
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithOutstandingProbe)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  ASSERT_TRUE(seal_generation_gate(non_pristine, generation_));
  auto probe = non_pristine.gate->prepare_probe();
  ASSERT_EQ(probe.status(), GenerationQuiescenceProbeStatus::kPrepared);
  ASSERT_TRUE(probe.take_probe_permit());
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithOutstandingReceipt)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  ASSERT_TRUE(seal_generation_gate(non_pristine, generation_));
  auto probe = non_pristine.gate->prepare_probe();
  ASSERT_EQ(probe.status(), GenerationQuiescenceProbeStatus::kPrepared);
  auto permit = probe.take_probe_permit();
  ASSERT_TRUE(permit);
  auto completed = non_pristine.gate->complete_probe(*permit, generation_, true);
  ASSERT_EQ(completed.status(), GenerationQuiescenceCompletionStatus::kReceiptIssued);
  ASSERT_TRUE(completed.take_receipt());
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RejectsAcceptedGateWithAcknowledgementLineage)
{
  auto non_pristine = make_generation_gate(goal_id(), generation_);
  ASSERT_TRUE(seal_generation_gate(non_pristine, generation_));
  ASSERT_NE(non_pristine.gate->snapshot().accepted_ack_cookie, 0U);
  expect_accepted_gate_rejected(non_pristine.gate);
}

TEST_F(RestockCoordinatorDriverTest, RetainsExactAuthenticAcceptedGate)
{
  auto authentic = make_generation_gate(goal_id(), generation_);
  std::weak_ptr<CoordinatorGenerationQuiescence> observed_gate = authentic.gate;
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        authentic.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  authentic.gate.reset();

  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_FALSE(observed_gate.expired());
  EXPECT_EQ(port_.snapshots.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, SnapshotCallbackDepositCompletesAndRestoresCount)
{
  accept_and_request_snapshot();
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));

  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 1U);
  EXPECT_FALSE(driver_->inbox()->generation_empty(generation_));
}

TEST_F(RestockCoordinatorDriverTest, SealedSnapshotCallbackPerformsNoInboxWork)
{
  bool sampled = false;
  install_driver(
    generated_grasps, [this, &sampled]() {
      sampled = true;
      return now_;
    });
  accept_and_request_snapshot();
  ASSERT_FALSE(port_.snapshots.empty());
  auto call = std::move(port_.snapshots.front());
  port_.snapshots.pop_front();
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));

  call.callback(
    AsyncServiceCompletion<WorldStateCoordinatorPort::GetSnapshot::Response>{
        call.correlation, snapshot_response(world_snapshot()), {}});

  EXPECT_FALSE(sampled);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_TRUE(driver_->inbox()->generation_empty(generation_));
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  EXPECT_EQ(
    accepted_gate_.gate->prepare_probe().status(),
    GenerationQuiescenceProbeStatus::kPrepared);
}

TEST_F(RestockCoordinatorDriverTest, SnapshotCallbackDepositBlocksProbeUntilCompletion)
{
  CallBlocker blocker;
  install_driver(
    generated_grasps, [this, &blocker]() {
      blocker.intercept();
      return now_;
    });
  accept_and_request_snapshot();
  ASSERT_EQ(port_.snapshots.size(), 1U);
  const auto request_handle = port_.snapshots.front().handle;
  blocker.arm();
  auto completion = std::async(
    std::launch::async, [this]() {
      port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
        port_.snapshots, snapshot_response(world_snapshot()));
    });
  if (!blocker.wait_until_blocked(2s)) {
    blocker.release();
  }
  ASSERT_EQ(completion.wait_for(0s), std::future_status::timeout);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 1U);
  EXPECT_TRUE(port_.remove_pending_request(request_handle));
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  EXPECT_EQ(
    accepted_gate_.gate->prepare_probe().status(),
    GenerationQuiescenceProbeStatus::kDepositsOutstanding);

  blocker.release();
  ASSERT_EQ(completion.wait_for(2s), std::future_status::ready);
  completion.get();
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 1U);

  driver_->pump(now_);
  EXPECT_TRUE(driver_->inbox()->generation_empty(generation_));
  auto probe = accepted_gate_.gate->prepare_probe();
  ASSERT_EQ(probe.status(), GenerationQuiescenceProbeStatus::kPrepared);
  auto permit = probe.take_probe_permit();
  ASSERT_TRUE(permit);
  EXPECT_EQ(
    accepted_gate_.gate->complete_probe(*permit, generation_, true).status(),
    GenerationQuiescenceCompletionStatus::kReceiptIssued);
}

TEST_F(RestockCoordinatorDriverTest, RetainedSnapshotCallbackSurvivesDriverDestruction)
{
  CallBlocker blocker;
  install_driver(
    generated_grasps, [this, &blocker]() {
      blocker.intercept();
      return now_;
    });
  accept_and_request_snapshot();
  auto inbox = driver_->inbox();
  blocker.arm();
  auto completion = std::async(
    std::launch::async, [this]() {
      port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
        port_.snapshots, snapshot_response(world_snapshot()));
    });
  if (!blocker.wait_until_blocked(2s)) {
    blocker.release();
  }
  ASSERT_EQ(completion.wait_for(0s), std::future_status::timeout);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 1U);
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  driver_.reset();

  blocker.release();
  ASSERT_EQ(completion.wait_for(2s), std::future_status::ready);
  completion.get();
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  EXPECT_EQ(inbox->snapshot().size, 1U);
  EXPECT_FALSE(inbox->generation_empty(generation_));
}

// The world state republishes products and lanes at 10 Hz, so the entity revisions a selection is
// bound to often move between snapshot and reservation. The world state refuses that request
// without mutating anything; the task must go back for fresh evidence, not latch a terminal motion
// inhibition.
TEST_F(RestockCoordinatorDriverTest, ReSelectsAfterANonMutatingReservationRejection)
{
  (void)reach_reservation_request();
  const std::string stale_request_id = port_.reservations.front().request->request_id;
  const auto stale_object_revision = port_.reservations.front().request->object_revision;

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
    port_.reservations,
    rejected_reserve_response(
      restocker_interfaces::msg::WorldStateOperationStatus::REVISION_CONFLICT));
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_TRUE(port_.validations.empty());
  EXPECT_TRUE(port_.releases.empty());
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kValidateScene);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  // Fresh evidence carries advanced entity revisions, so re-selection binds to those.
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot(14U)));
  driver_->pump(now_);
  ASSERT_EQ(port_.reservations.size(), 1U);
  EXPECT_NE(port_.reservations.front().request->request_id, stale_request_id);
  EXPECT_GT(port_.reservations.front().request->object_revision, stale_object_revision);

  now_ += 1ms;
  const auto fresh = reserve_response(*port_.reservations.front().request);
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, fresh);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations,
    validation_response(*fresh));
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(driver_->snapshot().staged_candidate_count, 2U);
}

// A world that never settles still terminates — but under Milestone 10 §6 (Card 051) this
// exhaustion is RECOVERABLE (nothing reserved, nothing moved, nothing irreversible) and exits
// through the typed rung-5 skip instead of latching the operator for the campaign to stop on.
TEST_F(
  RestockCoordinatorDriverTest, SkipsRecoverablyWhenReSelectionKeepsLosingTheRevisionRace)
{
  (void)reach_reservation_request();
  RestockTaskConfig defaults;
  for (std::size_t attempt = 0U; attempt <= defaults.max_selection_restarts; ++attempt) {
    ASSERT_EQ(port_.reservations.size(), 1U) << "attempt " << attempt;
    EXPECT_FALSE(driver_->snapshot().inhibited) << "attempt " << attempt;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
      port_.reservations,
      rejected_reserve_response(
        restocker_interfaces::msg::WorldStateOperationStatus::REVISION_CONFLICT));
    driver_->pump(now_);
    if (port_.snapshots.empty()) {
      break;
    }
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(world_snapshot(static_cast<std::uint64_t>(14U + 4U * attempt))));
    driver_->pump(now_);
  }

  // Bounded, so the ladder terminates instead of spinning — and it terminates without a latch.
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.reservations.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery classification: RECOVERABLE", 0) == 0;
      })) << "the exhausted re-selection must classify recoverable";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery rung 5 (recoverable skip): requested", 0) == 0;
      })) << "the rung-5 request must be receipted";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kRecoverableSkip;
      })) << "the terminal must be the typed recoverable skip, not an operator latch";
}

// A reservation conflict proves someone holds a reservation, so re-selecting cannot help and the
// fail-closed inhibition stands.
TEST_F(RestockCoordinatorDriverTest, InhibitsOnAMutatingReservationRejection)
{
  (void)reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
    port_.reservations,
    rejected_reserve_response(
      restocker_interfaces::msg::WorldStateOperationStatus::RESERVATION_CONFLICT));
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_TRUE(port_.reservations.empty());
}

// Review S2: the inhibited guard must be exercised where `commanded` is genuinely false (nothing
// was
// ever submitted), so removing the guard would turn the evidence true and fail this.
TEST_F(RestockCoordinatorDriverTest, EvidenceInhibitedBeforeAnyCommandCarriesNone)
{
  (void)reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
    port_.reservations,
    rejected_reserve_response(
      restocker_interfaces::msg::WorldStateOperationStatus::RESERVATION_CONFLICT));
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  bool saw_inhibited = false;
  for (const auto & output : outputs) {
    if (output.kind == CoordinatorDriverOutputKind::kInhibited ||
      output.outcome == RestockActionOutcome::kExternalInconsistency)
    {
      saw_inhibited = true;
      EXPECT_FALSE(output.metrics.motion_definitely_not_started);
      EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
    }
  }
  EXPECT_TRUE(saw_inhibited);
}

TEST_F(RestockCoordinatorDriverTest, ReserveCallbackDepositCompletesAndRestoresCount)
{
  const auto reserve = reach_reservation_request();
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 1U);
}

TEST_F(RestockCoordinatorDriverTest, SealedReserveCallbackPerformsNoInboxWork)
{
  bool sampled = false;
  install_driver(
    generated_grasps, [this, &sampled]() {
      sampled = true;
      return now_;
    });
  const auto reserve = reach_reservation_request();
  ASSERT_FALSE(port_.reservations.empty());
  auto call = std::move(port_.reservations.front());
  port_.reservations.pop_front();
  sampled = false;
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  call.callback(
    AsyncServiceCompletion<WorldStateCoordinatorPort::ReserveTask::Response>{
        call.correlation, reserve, {}});
  EXPECT_FALSE(sampled);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
}

TEST_F(RestockCoordinatorDriverTest, SealedValidationCallbackPerformsNoInboxWork)
{
  bool sampled = false;
  install_driver(
    generated_grasps, [this, &sampled]() {
      sampled = true;
      return now_;
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  auto call = std::move(port_.validations.front());
  port_.validations.pop_front();
  sampled = false;
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  call.callback(
    AsyncServiceCompletion<WorldStateCoordinatorPort::ValidateReservation::Response>{
        call.correlation, validation_response(*reserve), {}});
  EXPECT_FALSE(sampled);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
}

TEST_F(RestockCoordinatorDriverTest, SealedReleaseCallbackPerformsNoInboxWork)
{
  bool sampled = false;
  install_driver(
    generated_grasps, [this, &sampled]() {
      sampled = true;
      return now_;
    });
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  auto call = std::move(port_.releases.front());
  port_.releases.pop_front();
  sampled = false;
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  call.callback(
    AsyncServiceCompletion<WorldStateCoordinatorPort::ReleaseReservation::Response>{
        call.correlation, release, {}});
  EXPECT_FALSE(sampled);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_EQ(driver_->inbox()->snapshot().cleanup_emergency_size, 0U);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
}

TEST_F(RestockCoordinatorDriverTest, SealedReleaseReadbackCallbackPerformsNoInboxWork)
{
  bool sampled = false;
  install_driver(
    generated_grasps, [this, &sampled]() {
      sampled = true;
      return now_;
    });
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
    port_.releases, nullptr, "release acknowledgement lost");
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  auto call = std::move(port_.snapshots.front());
  port_.snapshots.pop_front();
  sampled = false;
  ASSERT_TRUE(seal_generation_gate(accepted_gate_, generation_));
  call.callback(
    AsyncServiceCompletion<WorldStateCoordinatorPort::GetSnapshot::Response>{
        call.correlation, snapshot_response(world_snapshot(reserve->world_revision)), {}});
  EXPECT_FALSE(sampled);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_EQ(driver_->inbox()->snapshot().cleanup_emergency_size, 0U);
  EXPECT_EQ(accepted_gate_.gate->snapshot().active_deposit_count, 0U);
}

TEST_F(RestockCoordinatorDriverTest, RejectsIngressBudgetOverflowBeforeInboxAllocation)
{
  RestockCoordinatorDriverConfig config;
  config.inbox_capacity = std::numeric_limits<std::size_t>::max();

  EXPECT_THROW(
    (void)RestockCoordinatorDriver(
      admission_, port_,
      [](const auto & snapshot, const auto &) {
        return SelectionResult<SelectedTaskPair>::success(selected_pair(snapshot));
      },
      [](const auto & snapshot) {
        return GraspCandidateResult<GraspGenerationAuthority>::success(
          grasp_authority(snapshot));
      },
      generated_grasps, [this]() {return now_;},
      []() {return rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME);}, config),
    std::invalid_argument);
}

TEST(CoordinatorRetainedTerminalTest, PreservesValidFirstTerminalAgainstInvalidSecond)
{
  CoordinatorRetainedTerminal slot;
  CoordinatorDriverOutput first;
  first.kind = CoordinatorDriverOutputKind::kAborted;
  first.goal_generation = 7U;
  first.outcome = RestockActionOutcome::kInternalError;
  first.detail = "first terminal";
  CoordinatorDriverOutput second;
  second.kind = CoordinatorDriverOutputKind::kSucceeded;
  second.goal_generation = 8U;
  second.outcome = RestockActionOutcome::kInternalError;
  second.detail = "contradictory second terminal";

  EXPECT_TRUE(slot.retain(std::move(first)));
  EXPECT_TRUE(slot.conflicts_with(second));
  EXPECT_FALSE(slot.retain(std::move(second)));
  EXPECT_EQ(second.detail, "contradictory second terminal");
  auto retained = slot.take();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->kind, CoordinatorDriverOutputKind::kAborted);
  EXPECT_EQ(retained->detail, "first terminal");
  EXPECT_FALSE(slot.occupied());
}

TEST(CoordinatorRetainedTerminalTest, PreservesInvalidFirstTerminalAgainstValidSecond)
{
  CoordinatorRetainedTerminal slot;
  CoordinatorDriverOutput first;
  first.kind = CoordinatorDriverOutputKind::kSucceeded;
  first.goal_generation = 9U;
  first.outcome = RestockActionOutcome::kInternalError;
  first.detail = "invalid first terminal";
  CoordinatorDriverOutput second;
  second.kind = CoordinatorDriverOutputKind::kCanceled;
  second.goal_generation = 10U;
  second.outcome = RestockActionOutcome::kCanceled;
  second.detail = "valid second terminal";

  EXPECT_TRUE(slot.retain(std::move(first)));
  EXPECT_TRUE(slot.conflicts_with(second));
  EXPECT_FALSE(slot.retain(std::move(second)));
  auto retained = slot.take();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->kind, CoordinatorDriverOutputKind::kSucceeded);
  EXPECT_EQ(retained->detail, "invalid first terminal");
}

TEST(CoordinatorRetainedTerminalTest, MoveTransfersTheOnlyOccupiedSlot)
{
  CoordinatorRetainedTerminal original;
  CoordinatorDriverOutput output;
  output.kind = CoordinatorDriverOutputKind::kAborted;
  output.goal_generation = 11U;
  output.outcome = RestockActionOutcome::kInternalError;
  output.detail = "single owner";
  ASSERT_TRUE(original.retain(std::move(output)));

  CoordinatorRetainedTerminal moved{std::move(original)};
  EXPECT_FALSE(original.occupied());
  ASSERT_TRUE(moved.occupied());
  auto retained = moved.take();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->detail, "single owner");
}

TEST(CoordinatorRetainedTerminalTest, MoveAssignmentReplacesDestinationAndEmptiesSource)
{
  CoordinatorRetainedTerminal source;
  CoordinatorDriverOutput source_output;
  source_output.kind = CoordinatorDriverOutputKind::kAborted;
  source_output.goal_generation = 12U;
  source_output.outcome = RestockActionOutcome::kInternalError;
  source_output.detail = "source terminal";
  ASSERT_TRUE(source.retain(std::move(source_output)));

  CoordinatorRetainedTerminal destination;
  CoordinatorDriverOutput destination_output;
  destination_output.kind = CoordinatorDriverOutputKind::kCanceled;
  destination_output.goal_generation = 13U;
  destination_output.outcome = RestockActionOutcome::kCanceled;
  destination_output.detail = "replaced destination terminal";
  ASSERT_TRUE(destination.retain(std::move(destination_output)));

  destination = std::move(source);

  EXPECT_FALSE(source.occupied());
  ASSERT_TRUE(destination.occupied());
  auto retained = destination.take();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->goal_generation, 12U);
  EXPECT_EQ(retained->detail, "source terminal");
}

TEST(CoordinatorRetainedTerminalTest, SelfMoveAssignmentPreservesTheOnlySlot)
{
  CoordinatorRetainedTerminal slot;
  CoordinatorDriverOutput output;
  output.kind = CoordinatorDriverOutputKind::kAborted;
  output.goal_generation = 14U;
  output.outcome = RestockActionOutcome::kInternalError;
  output.detail = "self-move terminal";
  ASSERT_TRUE(slot.retain(std::move(output)));

  auto * alias = &slot;
  slot = std::move(*alias);

  ASSERT_TRUE(slot.occupied());
  auto retained = slot.take();
  ASSERT_TRUE(retained);
  EXPECT_EQ(retained->goal_generation, 14U);
  EXPECT_EQ(retained->detail, "self-move terminal");
}

TEST_F(RestockCoordinatorDriverTest, CancelsAtProvenReservationBoundaryAndReleases)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::all_of(
      outputs, [this](const auto & output) {return output.goal_generation == generation_;}));
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled &&
               output.outcome == RestockActionOutcome::kCanceled;
      }));
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kActive);
  EXPECT_TRUE(driver_->snapshot().active);
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_FALSE(driver_->acknowledge_terminal_delivery(generation_));
}

TEST_F(RestockCoordinatorDriverTest, CompletesTheLedgerWhenASegmentIsRefusedBeforeSubmission)
{
  // A planned segment books a ledger operation before building its goal. A refusal in between
  // (unlive perception stream) must complete that operation, or it expires later as "motion segment
  // exceeded its command deadline" naming an operation no port is working on (the dense demo gate
  // stall).
  //
  // Liveness is checked twice per segment: once before the operation is booked (where a failure
  // holds the bounded reacquire instead) and again at submission inside request_motion_segment.
  // The first two calls therefore pass for the pre-grasp (book + submit); the submission gate of
  // the approach then fails after its operation was booked, which is the refusal in between.
  int liveness_calls = 0;
  install_driver(
    [this](const auto & snapshot, const auto & selected, const auto & authority,
    const auto & now) {
      auto generated = generated_grasps(snapshot, selected, authority, now);
      if (generated) {
        staged_batch_ = generated.value();
      }
      return generated;
    },
    {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {}, {}, {},
    [&liveness_calls](std::string & detail) {
      if (++liveness_calls <= 3) {
        return true;
      }
      detail = "wrist depth stream is older than its liveness horizon";
      return false;
    });
  reserve_response_ = reach_staging_boundary();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForApproach);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);

  // The approach was refused before submission, so the arm cannot be moving.
  EXPECT_FALSE(motion_.outstanding());
  // Nothing is booked against a port that was never asked.
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  // The refusal passed through recovery and stopped the task instead of retrying forever.
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_EQ(liveness_calls, 4);
}

TEST_F(RestockCoordinatorDriverTest, BusyPreInsertWithoutProgressIsCancelledAtItsCommandDeadline)
{
  // Card 049 SC-001: the deterministic repro of the busy-form terminal. The port owns a booked,
  // executing pre-insert and no completion ever arrives; at the armed command deadline —
  // planning + execution on the steady clock — the driver cancels the port, latches the exact
  // receipt line and fails the goal closed. The bound is what the budget re-size must NOT touch.
  task_configure_ = [](RestockCoordinatorDriverConfig & config) {
    config.task.planning_timeout = 300ms;
    config.task.execution_timeout = 400ms;
    config.task.total_timeout = 600000ms;
  };
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_);
  reach_preinsert_with_motion();
  ASSERT_TRUE(driver_->snapshot().command_deadline);
  ASSERT_TRUE(driver_->snapshot().pending_operation);
  const auto ticket = *driver_->snapshot().pending_operation;
  const auto deadline = *driver_->snapshot().command_deadline;
  const auto expected =
    "motion segment exceeded its command deadline: waiting on pre-insert operation " +
    std::to_string(ticket.operation_generation) + " of goal generation " +
    std::to_string(ticket.goal_generation);

  now_ = deadline - 1ms;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited) << "one millisecond early must not latch";
  EXPECT_EQ(motion_.cancel_requests, 0U);

  now_ = deadline;
  driver_->pump(now_);
  EXPECT_EQ(motion_.cancel_requests, 1U) << "the deadline must stop the port";
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [&expected](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kInhibited &&
               output.detail.find(expected) != std::string::npos;
      })) << "the receipt must name the operation the port was working on";
}

TEST_F(RestockCoordinatorDriverTest, Card066Dev2PreInsertTimelineCompletesWithinTheSegmentBudget)
{
  // Card 049 SC-001/SC-002: a replay of the Card 066 dev-2 receipt
  // (evidence/cmbfeed-066/dev2-079708f-launch-ctest.log): pre-insert operation 60 planned in
  // 27.9 s and then executed controller goals 1..8 until the armed segment budget expired at
  // goal 9/10. The receipt's ledger timeline — the port returning 215 s after booking — must
  // complete WITHOUT the deadline terminal once the shipped budget covers it (Milestone 10 §6,
  // Card 049). The task budgets below mirror config/restock_action_coordinator.yaml; the
  // floor they must clear is pinned by test_coordinator_budgets.
  // The receipt's completion arrives 215 s after booking and must advance the machine; the
  // budgets below mirror config/restock_action_coordinator.yaml, so the floor they must clear
  // is pinned by test_coordinator_budgets (Card 049 measured floor, 215000 ms).
  task_configure_ = [](RestockCoordinatorDriverConfig & config) {
    config.task.planning_timeout = 60000ms;
    config.task.execution_timeout = 180000ms;
    config.task.total_timeout = 480000ms;
  };
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_);
  reach_preinsert_with_motion();
  ASSERT_TRUE(driver_->snapshot().command_deadline);
  const auto deadline = *driver_->snapshot().command_deadline;

  // The measured timeline: planning ~28 s, execution ~187 s (goal 9/10 was still running when
  // the old budget expired) — the completion arrives 215 s after booking.
  now_ += 215s;
  ASSERT_LT(now_, deadline) <<
    "the shipped segment budget must cover the receipt timeline (Card 049 measured floor)";
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited) <<
    "a progressing pre-insert must not be cancelled by its own budget";
  EXPECT_EQ(motion_.cancel_requests, 0U);
  EXPECT_TRUE(motion_.outstanding());

  motion_.complete(
    MotionOutcome::kSucceeded,
    "pre-insert pose reached through 10 bounded controller goal(s)");
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(
    driver_->snapshot().transition->state, RestockTaskState::kPlanPreInsert) <<
    "the receipt's completion must advance the machine past the plan state";
}

TEST_F(RestockCoordinatorDriverTest, DropsInvalidTimeOrdinarySnapshotCompletion)
{
  install_driver(generated_grasps, []() {return SteadyTime::max();});
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);

  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_TRUE(port_.snapshots.empty());
  ASSERT_TRUE(driver_->snapshot().pending_operation);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kValidateScene);
}

TEST_F(RestockCoordinatorDriverTest, AuthorizesInvalidTimeCleanupSnapshotFromFirstTermination)
{
  bool invalid_async_time = false;
  install_driver(
    generated_grasps, [this, &invalid_async_time]() {
      return invalid_async_time ? SteadyTime::max() : now_;
    });
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  invalid_async_time = true;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled &&
               output.outcome == RestockActionOutcome::kCanceled;
      }));
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, ActiveOverflowReleasesReservationBeforeInhibition)
{
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  for (std::size_t index = 0U; index < 128U; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill inbox"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_, 0U, now_, "reserved overflow"}),
    CoordinatorInboxPushResult::kOverflowLatched);

  now_ += 1ms;
  driver_->pump(now_);

  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kInboxOverflow);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission.inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);

  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, GeneratorRejectionReleasesWithoutGlobalInhibition)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{
          GraspCandidateErrorCode::NoValidJawTarget, "selected can exceeds jaw opening"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kSafeAbort);
  EXPECT_EQ(admission.first_termination->arrived_at, now_);
  EXPECT_FALSE(admission.first_termination->source_generation);
  complete_failed_safe_release(*reserve);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kValidationFailed;
      }));
}

TEST_F(RestockCoordinatorDriverTest, StaleGeneratorRejectionReleasesWithoutGlobalInhibition)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{GraspCandidateErrorCode::StaleEvidence, "object pose is stale"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  complete_failed_safe_release(*reserve);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kValidationFailed;
      }));
}

TEST_F(RestockCoordinatorDriverTest, InvalidObjectRejectionReleasesWithoutGlobalInhibition)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{GraspCandidateErrorCode::InvalidObject, "object is not graspable"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  complete_failed_safe_release(*reserve);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kValidationFailed;
      }));
}

TEST_F(RestockCoordinatorDriverTest, InternalGeneratorErrorCleansUpBeforeInhibitingAdmission)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{
          GraspCandidateErrorCode::InvalidConfiguration, "immutable config is invalid"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);

  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kExternalInconsistency;
      }));
}

TEST_F(RestockCoordinatorDriverTest, ProtocolFailureDefersInhibitionUntilReservationReleaseProof)
{
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  ASSERT_TRUE(driver_->snapshot().reservation_capability_may_remain);

  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown,
        generation_, 0U, failure_at,
        "steady-clock authority failed"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);

  EXPECT_FALSE(port_.removed_request_ids.empty());
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);

  complete_failed_safe_release(*reserve);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kExternalInconsistency;
      }));
}

TEST_F(RestockCoordinatorDriverTest, ProtocolFailureWaitsForAppliedReserveThenReleasesOnce)
{
  const auto reserve = reach_reservation_request();
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown, generation_, 0U,
        failure_at, "failure while reserve outcome is pending"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);

  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().pending_operation.has_value());
  EXPECT_TRUE(port_.releases.empty());

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);

  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, ProtocolFailureWithUnknownReserveWithholdsTerminality)
{
  (void)reach_reservation_request();
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown, generation_, 0U,
        failure_at, "failure while reserve outcome is unknown"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
    port_.reservations, nullptr,
    "reserve response lost");
  driver_->pump(now_);

  const auto state = driver_->snapshot();
  ASSERT_TRUE(state.pending_operation);
  EXPECT_EQ(state.pending_operation->effect, OperationEffect::kIdempotentMutation);
  EXPECT_FALSE(state.terminal_output_emitted);
  EXPECT_TRUE(state.reservation_capability_may_remain);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(port_.releases.empty());
}

TEST_F(
  RestockCoordinatorDriverTest,
  ProtocolFailureRoutesReconciledReserveReadbackDirectlyIntoCleanup)
{
  std::size_t grasp_generation_count = 0U;
  install_driver(
    [&grasp_generation_count](const auto & snapshot, const auto & selected,
    const auto & authority, const auto & generated_at) {
      ++grasp_generation_count;
      return generated_grasps(snapshot, selected, authority, generated_at);
    });
  const auto reserve = reach_reservation_request();
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kShutdown, generation_, 0U, failure_at,
        "failure while reserve reconciliation is required"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(
    port_.reservations, nullptr,
    "original reserve response lost");
  driver_->pump(now_);
  ASSERT_EQ(port_.reservations.size(), 1U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);

  EXPECT_EQ(grasp_generation_count, 0U);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);

  complete_failed_safe_release(*reserve);
  EXPECT_EQ(grasp_generation_count, 0U);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, ProtocolFailureDoesNotDuplicatePendingRelease)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, now_).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, now_, "initial drain"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown,
        generation_, 0U, failure_at,
        "failure while release is pending"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);

  EXPECT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, RejectedReleaseRetriesBeforeDeferredInhibition)
{
  const auto reserve = reach_staging_boundary();
  port_.reject_release_count = 1U;
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown,
        generation_, 0U, failure_at,
        "failure requiring release retry"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);

  EXPECT_EQ(port_.release_attempts, 1U);
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_FALSE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);

  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(port_.release_attempts, 2U);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);

  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, ExhaustedReleaseRejectionRetainsCapabilityAndTerminality)
{
  (void)reach_staging_boundary();
  port_.reject_release_count = 2U;
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown, generation_, 0U,
        failure_at, "persistent release submission failure"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);
  now_ += 1ms;
  driver_->pump(now_);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(port_.release_attempts, 2U);
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_FALSE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);
}

TEST_F(RestockCoordinatorDriverTest, RejectedReleaseAtDeadlineRetainsCapabilityAndTerminality)
{
  (void)reach_staging_boundary();
  port_.reject_release_count = 1U;
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown,
        generation_, 0U, failure_at,
        "release retry reaches deadline"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at;
  driver_->pump(now_);
  ASSERT_EQ(port_.release_attempts, 1U);
  ASSERT_TRUE(driver_->snapshot().command_deadline);

  now_ = *driver_->snapshot().command_deadline;
  driver_->pump(now_);

  EXPECT_EQ(port_.release_attempts, 1U);
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_FALSE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);
}

TEST_F(RestockCoordinatorDriverTest, ReleaseEntryObservedAtDeadlineRetainsCapabilityAndTerminality)
{
  (void)reach_staging_boundary();
  const auto failure_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kShutdown,
        generation_, 0U, failure_at,
        "delayed release-state entry"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ = failure_at + 100ms;
  driver_->pump(failure_at);

  EXPECT_EQ(port_.release_attempts, 0U);
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_FALSE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().reservation_capability_may_remain);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);
}

TEST_F(RestockCoordinatorDriverTest, ProtocolFailureAfterDefiniteReserveRejectionNeedsNoRelease)
{
  port_.reject_reserve_once = true;
  port_.before_reserve = [this]() {
    const auto failure_at = now_ + 1ms;
    EXPECT_EQ(
      admission_.request_protocol_failure(goal_id(), generation_, failure_at).status(),
      GoalTerminationLatchStatus::kLatched);
    EXPECT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kShutdown, generation_, 0U,
          failure_at, "failure at definite reserve rejection"}),
      CoordinatorInboxPushResult::kAccepted);
  };
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);
  driver_->pump(now_ + 1ms);

  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
  EXPECT_TRUE(admission_.snapshot().inhibited);
}

TEST_F(RestockCoordinatorDriverTest, InvalidToolErrorCleansUpBeforeInhibitingAdmission)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{
          GraspCandidateErrorCode::InvalidToolTransform, "verified tool datum was contradicted"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);

  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kExternalInconsistency;
      }));
}

TEST_F(RestockCoordinatorDriverTest, MalformedGeneratorCleansUpBeforeInhibitingAdmission)
{
  install_driver(
    [](const auto & snapshot, const auto & selected, const auto & authority, const auto & now) {
      auto result = generated_grasps(snapshot, selected, authority, now);
      if (result) {
        result.value().candidates.front().source_yaw_index = 15U;
      }
      return result;
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);

  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, CancellationDuringPureGenerationDiscardsCandidates)
{
  install_driver(
    [this](const auto & snapshot, const auto & selected, const auto & authority, const auto & now) {
      EXPECT_TRUE(
        termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
      EXPECT_EQ(
        driver_->inbox()->push(
          CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested, generation_,
            0U, now_, "queued cancel raced pure generation"}),
        CoordinatorInboxPushResult::kAccepted);
      return generated_grasps(snapshot, selected, authority, now);
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  driver_->pump(now_);

  EXPECT_EQ(driver_->snapshot().staged_candidate_count, 0U);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);
  EXPECT_EQ(port_.releases.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, DrainDuringPureGenerationDiscardsCandidates)
{
  install_driver(
    [this](const auto & snapshot, const auto & selected, const auto & authority, const auto & now) {
      EXPECT_TRUE(
        termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, now_)));
      EXPECT_EQ(
        driver_->inbox()->push(
          CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested, generation_,
            0U, now_, "queued drain raced pure generation"}),
        CoordinatorInboxPushResult::kAccepted);
      return generated_grasps(snapshot, selected, authority, now);
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  driver_->pump(now_);

  EXPECT_EQ(driver_->snapshot().staged_candidate_count, 0U);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);
  EXPECT_EQ(port_.releases.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, MalformedGenerationDuringCancelRaceStillInhibitsAfterCleanup)
{
  install_driver(
    [this](const auto & snapshot, const auto & selected, const auto & authority, const auto & now) {
      EXPECT_TRUE(
        termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
      auto result = generated_grasps(snapshot, selected, authority, now);
      if (result) {
        result.value().candidates.front().score.total = std::numeric_limits<double>::quiet_NaN();
      }
      return result;
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);

  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().staged_candidate_count, 0U);
}

TEST_F(RestockCoordinatorDriverTest, CompositeAcceptedDrainCannotStartForwardWork)
{
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        now_,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_TRUE(port_.reservations.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
  EXPECT_FALSE(admission_.snapshot().mutation_submission.has_value());
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, SealedCleanAcceptedGoalStartsNormally)
{
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_);

  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_TRUE(driver_->snapshot().task_deadline);
  EXPECT_TRUE(driver_->snapshot().pending_operation);
  EXPECT_EQ(port_.snapshots.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, SealedInlineDrainUsesCompositeHandoff)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        drain_at,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainRequiresExactNextDeliveryInSamePump)
{
  const auto drain_at = now_ + 1ms;
  const auto drain = admission_.request_drain(goal_id(), generation_, drain_at);
  ASSERT_TRUE(termination_latch_succeeded(drain));
  const auto first_record = drain.record();
  ASSERT_TRUE(first_record);
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, drain_at,
        "deferred exact drain"}),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination.get(), first_record.get());
  EXPECT_FALSE(admission.inhibited);
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_TRUE(port_.reservations.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
}

TEST_F(RestockCoordinatorDriverTest, SatisfiedDrainFenceDoesNotLeakIntoNextGoalLifecycle)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, drain_at,
        "deferred exact drain"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_ + 2ms);
  ASSERT_TRUE(driver_->snapshot().terminal_output_emitted);
  ASSERT_TRUE(driver_->acknowledge_terminal_delivery(generation_));
  ASSERT_FALSE(driver_->snapshot().active);

  const auto next_goal_id = goal_id(31U);
  const auto receipt = admission_.reserve(next_goal_id);
  ASSERT_EQ(receipt.decision, GoalAdmissionDecision::kAccepted);
  ASSERT_TRUE(admission_.activate(next_goal_id, receipt.generation));
  auto next_gate = make_generation_gate(next_goal_id, receipt.generation);
  now_ += 3ms;
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{next_goal_id,
        receipt.generation,
        {},
        now_,
        rclcpp::Time(std::int64_t{2'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        next_gate.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_);

  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_EQ(driver_->snapshot().goal_generation, receipt.generation);
  EXPECT_TRUE(driver_->snapshot().task_deadline);
  EXPECT_TRUE(driver_->snapshot().pending_operation);
  EXPECT_EQ(port_.snapshots.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainWithoutNextDeliveryInhibits)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_FALSE(driver_->snapshot().pending_operation);
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainAtIngressBudgetExhaustionInhibits)
{
  // Sized so the sealed accepted-goal refill is still reachable within the pump's ingress budget
  // (ordinary capacity plus every emergency and marker lane) while the turn still overruns it.
  constexpr std::size_t kRefillCount = 3U;
  struct RefillResults
  {
    std::array<CoordinatorInboxDepositResult, kRefillCount> heartbeats{};
    std::optional<CoordinatorInboxDepositResult> accepted;
    std::optional<CoordinatorInboxDepositResult> drain;
    bool callback_failed{false};
  };

  auto inbox = driver_->inbox();
  std::weak_ptr<CoordinatorInbox> weak_inbox = inbox;
  const auto results = std::make_shared<RefillResults>();
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));

  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    expect_deposit(
      inbox->push_cleanup(
        ReleaseReservationCompletion{
          {generation_, index + 1U}, nullptr, "", now_, std::nullopt}),
      CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned);
  }

  const auto capacity = inbox->snapshot().capacity;
  ASSERT_GT(capacity, kRefillCount);
  // The pump's ingress budget is the ordinary capacity plus every emergency and marker lane. Fill
  // to that budget so the sealed accepted drain exhausts it regardless of emergency lane count.
  const std::size_t ingress_budget = capacity + CoordinatorInbox::kOverflowMarkerCapacity +
    CoordinatorInbox::kAcceptedHandoffEmergencyCapacity +
    CoordinatorInbox::kCleanupEvidenceLossMarkerCapacity +
    CoordinatorInbox::kCleanupEmergencyCapacity;
  for (std::size_t index = 0U; index < capacity; ++index) {
    using Response = restocker_interfaces::srv::ReserveTask::Response;
    std::shared_ptr<const Response> response;
    if (index < kRefillCount) {
      response = std::shared_ptr<const Response>(
        new Response{},
        [weak_inbox, results, generation = generation_, index](const Response * owned) noexcept {
          delete owned;
          try {
            const auto retained = weak_inbox.lock();
            if (!retained) {
              results->callback_failed = true;
              return;
            }
            results->heartbeats[index] = retained->push(
              CoordinatorControlEvent{
                CoordinatorControlKind::kHeartbeat, generation, 0U, SteadyTime{}, {}});
          } catch (...) {
            results->callback_failed = true;
          }
        });
    } else if (index + 1U == capacity) {
      response = std::shared_ptr<const Response>(
        new Response{},
        [weak_inbox, results, goal = goal_id(), generation = generation_,
        gate = accepted_gate_.gate, started = now_, drain_at](const Response * owned) noexcept {
          delete owned;
          try {
            const auto retained = weak_inbox.lock();
            if (!retained) {
              results->callback_failed = true;
              return;
            }
            results->accepted = retained->push_accepted_goal(
              CoordinatorAcceptedGoal{goal,
                generation,
                {},
                started,
                rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
                std::nullopt,
                gate,
                true},
              std::nullopt, started);
            results->drain = retained->push(
              CoordinatorControlEvent{
                CoordinatorControlKind::kDrainRequested, generation, 0U, drain_at, {}});
          } catch (...) {
            results->callback_failed = true;
          }
        });
    } else {
      response = std::make_shared<Response>();
    }
    expect_deposit(
      inbox->push(
        ReserveTaskCompletion{
          {generation_, index + CoordinatorInbox::kCleanupEmergencyCapacity + 1U},
          std::move(response),
          "",
          now_,
          std::nullopt}),
      CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned);
  }

  driver_->pump(now_ + 2ms);

  EXPECT_FALSE(results->callback_failed);
  for (const auto & heartbeat : results->heartbeats) {
    expect_deposit(
      heartbeat, CoordinatorInboxDepositStatus::kAccepted,
      CoordinatorInboxPersistenceStatus::kEventOwned);
  }
  ASSERT_TRUE(results->accepted);
  expect_deposit(
    *results->accepted, CoordinatorInboxDepositStatus::kAccepted,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  ASSERT_TRUE(results->drain);
  expect_deposit(
    *results->drain, CoordinatorInboxDepositStatus::kAccepted,
    CoordinatorInboxPersistenceStatus::kEventOwned);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_FALSE(driver_->snapshot().task_deadline);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  // One turn offers the prefilled ordinary items, the cleanup emergency lane, and everything the
  // destructor callbacks refill. Whatever exceeds the pump's ingress budget must still be queued.
  const std::size_t offered =
    capacity + CoordinatorInbox::kCleanupEmergencyCapacity + kRefillCount + 2U;
  EXPECT_EQ(inbox->snapshot().size, offered - ingress_budget);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainRejectsInterveningDelivery)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_, 0U, drain_at, "intervening event"}),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kDrainRequested, generation_, 0U, drain_at, "late drain"}),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_EQ(driver_->inbox()->snapshot().size, 1U);
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainRejectsMalformedControl)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 7U, drain_at,
        "malformed drain operation generation"}),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainRejectsWrongGenerationControl)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_ + 1U, 0U, drain_at,
        "foreign drain"}),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, SealedAcceptedDrainRejectsWrongArrivalControl)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, drain_at + 1ms,
        "wrong drain arrival"}),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, LegacyAcceptedGoalCannotOmitLatchedDrainNotice)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, drain_at)));
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);

  driver_->pump(now_ + 2ms);

  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, MarkerFirstLossDominatesSealedDrainHandoff)
{
  const auto drain_at = now_ + 1ms;
  const auto drain = admission_.request_drain(goal_id(), generation_, drain_at);
  ASSERT_TRUE(termination_latch_succeeded(drain));
  const auto first_record = drain.record();
  ASSERT_TRUE(first_record);
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 0U; index < capacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kOverflowLatched);
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested, generation_,
        0U, drain_at, "marker-owned deferred drain loss"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned);

  driver_->pump(now_ + 2ms);

  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination.get(), first_record.get());
  EXPECT_TRUE(admission.inhibited);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kExternalInconsistency;
      }));
}

TEST_F(RestockCoordinatorDriverTest, AcceptsPendingDrainAfterLaterAuthorityPolicyStrengthening)
{
  const auto drain_at = now_ + 1ms;
  const auto authority_at = now_ + 2ms;
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, drain_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    admission_.request_authority_loss(goal_id(), generation_, authority_at).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kCoordinatorDrain);
  EXPECT_EQ(admission.termination_intent, GoalTerminationIntent::kSafeAbort);

  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        drain_at,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
}

TEST_F(RestockCoordinatorDriverTest, AcceptsPendingAuthorityThenLaterDrainNotice)
{
  const auto authority_at = now_ + 1ms;
  const auto drain_at = now_ + 2ms;
  ASSERT_EQ(
    admission_.request_authority_loss(goal_id(), generation_, authority_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, drain_at).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kAuthorityLoss);
  EXPECT_EQ(admission.first_termination->arrived_at, authority_at);
  EXPECT_EQ(admission.termination_intent, GoalTerminationIntent::kSafeAbort);

  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        drain_at,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
}

TEST_F(RestockCoordinatorDriverTest, SealedNonDrainRetainedControlIsConsumedBeforeAnyForwardAdvance)
{
  const auto authority_at = now_ + 1ms;
  const auto authority = admission_.request_authority_loss(goal_id(), generation_, authority_at);
  ASSERT_EQ(authority.status(), GoalTerminationLatchStatus::kLatched);
  const auto first_record = authority.record();
  ASSERT_TRUE(first_record);
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate,
        true},
      CoordinatorControlEvent{CoordinatorControlKind::kAuthorityFaultSafeAbortRequested,
        generation_, 0U, authority_at, "pending authority failure"},
      now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination.get(), first_record.get());
  EXPECT_EQ(admission.first_termination->arrived_at, authority_at);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kAuthorityLoss);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kExternalInconsistency;
      }));
}

TEST_F(RestockCoordinatorDriverTest, RejectsPendingDrainRecordWithoutNotice)
{
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, now_ + 1ms).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, RejectsPendingDrainNoticeWithoutTerminationRecord)
{
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        now_ + 1ms,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, RejectsPendingDrainNoticeWithMismatchedTime)
{
  const auto drain_at = now_ + 1ms;
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, drain_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        drain_at + 1ms,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, AcceptsDrainNoticePreservedBehindEarlierUserCancel)
{
  const auto cancel_at = now_ + 1ms;
  const auto drain_at = now_ + 2ms;
  ASSERT_EQ(
    admission_.request_cancel(goal_id(), generation_, cancel_at).status(),
    GoalTerminationLatchStatus::kLatched);
  ASSERT_EQ(
    admission_.request_drain(goal_id(), generation_, drain_at).status(),
    GoalTerminationLatchStatus::kAlreadyLatched);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(admission.termination_intent, GoalTerminationIntent::kUserCancel);
  EXPECT_FALSE(admission.safe_abort_requested);

  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        drain_at,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
}

TEST_F(RestockCoordinatorDriverTest, DrainAtStagingReleasesWithShutdownOutcome)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, now_, "process shutdown"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  released.robot.task_phase = TaskPhase::Fault;
  released.robot.fault_state = FaultState::Recoverable;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kShutdown;
      }));
  EXPECT_FALSE(admission_.snapshot().mutation_submission.has_value());
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, DrainReconcilesAnInFlightReserveBeforeRelease)
{
  const auto reserve = reach_reservation_request();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_drain(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kDrainRequested,
        generation_, 0U, now_, "process shutdown"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_TRUE(port_.releases.empty());

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
  EXPECT_TRUE(port_.validations.empty());
}

TEST_F(RestockCoordinatorDriverTest, DuplicateCancelPreservesTerminalReleaseReadback)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "first cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  const auto removed_before = port_.removed_request_ids.size();

  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "duplicate cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_EQ(port_.snapshots.size(), 1U);
  EXPECT_EQ(port_.removed_request_ids.size(), removed_before);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);
  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
}

TEST_F(RestockCoordinatorDriverTest, PreservesTypedNoCompatiblePairOutcome)
{
  RestockCoordinatorDriverConfig config;
  config.task.validation_timeout = 100ms;
  config.task.total_timeout = 1000ms;
  config.reconciliation = {500ms, 100ms, 4U};
  driver_ = std::make_unique<RestockCoordinatorDriver>(
    admission_, port_,
    [](const auto &, const auto &) {
      return SelectionResult<SelectedTaskPair>::failure(
        SelectionError{
          SelectionErrorCode::NoEligiblePair, "no eligible product and lane pair"});
    },
    [](const auto & snapshot) {
      return GraspCandidateResult<GraspGenerationAuthority>::success(grasp_authority(snapshot));
    },
    generated_grasps, [this]() {return now_;},
    []() {return rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME);}, config);

  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kNoCompatiblePair;
      }));
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kActive);
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_EQ(admission_.reserve(goal_id(9U)).decision, GoalAdmissionDecision::kAccepted);
}

TEST_F(RestockCoordinatorDriverTest, PreservesTypedObservationEvidenceStaleOutcome)
{
  RestockCoordinatorDriverConfig config;
  config.task.validation_timeout = 100ms;
  config.task.total_timeout = 1000ms;
  config.reconciliation = {500ms, 100ms, 4U};
  driver_ = std::make_unique<RestockCoordinatorDriver>(
    admission_, port_,
    [](const auto &, const auto &) {
      return SelectionResult<SelectedTaskPair>::failure(
        SelectionError{SelectionErrorCode::ObjectStale, "object observation is stale"});
    },
    [](const auto & snapshot) {
      return GraspCandidateResult<GraspGenerationAuthority>::success(grasp_authority(snapshot));
    },
    generated_grasps, [this]() {return now_;},
    []() {return rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME);}, config);

  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.outcome == RestockActionOutcome::kObservationEvidenceStale;
      }));
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kActive);
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  EXPECT_EQ(admission_.reserve(goal_id(9U)).decision, GoalAdmissionDecision::kAccepted);
}

TEST_F(RestockCoordinatorDriverTest, CancellationAfterReserveSubmissionSkipsForwardReadback)
{
  auto reserve = reach_reservation_request();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "cancel in flight"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);

  EXPECT_TRUE(port_.validations.empty());
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
}

TEST_F(RestockCoordinatorDriverTest, CancellationRetiresPendingReservationValidation)
{
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);

  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "cancel validation"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_FALSE(port_.removed_request_ids.empty());
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->token, reserve->token);
}

TEST_F(RestockCoordinatorDriverTest, InhibitionDuringReserveRetainsLateCapability)
{
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  // The pending reserve response consumes one ordinary inbox slot.
  for (std::size_t index = 0U; index < 127U; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill inbox"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  EXPECT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
        generation_, 0U, now_, "overflow"}),
    CoordinatorInboxPushResult::kOverflowLatched);

  driver_->pump(now_);
  const auto state = driver_->snapshot();
  ASSERT_TRUE(state.transition);
  EXPECT_EQ(state.transition->state, RestockTaskState::kReleaseTask);
  EXPECT_TRUE(state.transition->reservation_active);
  EXPECT_TRUE(state.active);
  ASSERT_TRUE(state.pending_operation);
  EXPECT_EQ(state.pending_operation->command, RestockTaskCommand::kReleaseTaskReservation);
  EXPECT_TRUE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  ASSERT_EQ(port_.releases.size(), 1U);

  complete_failed_safe_release(*reserve);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, InhibitedUnknownReserveWaitsForBoundedExhaustion)
{
  (void)reach_reservation_request();
  ASSERT_EQ(
    driver_->inbox()->push(
      SnapshotCompletion{
        {generation_, 99U}, snapshot_response(world_snapshot()), "", now_, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReserveTask);

  now_ += 100ms;
  driver_->pump(now_);
  EXPECT_EQ(driver_->snapshot().pending_operation->effect, OperationEffect::kIdempotentMutation);
  EXPECT_EQ(port_.reservations.size(), 1U);

  now_ += 500ms;
  driver_->pump(now_);
  const auto state = driver_->snapshot();
  ASSERT_TRUE(state.transition);
  EXPECT_EQ(state.transition->state, RestockTaskState::kRequestOperator);
  EXPECT_TRUE(state.pending_operation);
  EXPECT_TRUE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(state.terminal_output_emitted);
  EXPECT_TRUE(
    std::ranges::none_of(
      driver_->take_outputs(), [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled ||
               output.kind == CoordinatorDriverOutputKind::kAborted ||
               output.kind == CoordinatorDriverOutputKind::kSucceeded;
      }));
  EXPECT_FALSE(port_.removed_request_ids.empty());
}

TEST_F(RestockCoordinatorDriverTest, ReconcilesUnknownReleaseByReadbackReplayAndProof)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  const auto release_operation_id = port_.releases.front().request->operation_id;

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
    port_.releases, nullptr, "release acknowledgement lost");
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto still_reserved = world_snapshot(reserve->world_revision);
  still_reserved.robot.revision = still_reserved.revision;
  const auto decoded_reservation = restocker_world_state::task_reservation_from_message(
    reserve->reservation, reserve->world_revision);
  ASSERT_TRUE(decoded_reservation) << decoded_reservation.error().detail;
  still_reserved.active_reservation = decoded_reservation.value();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(still_reserved));
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  EXPECT_EQ(port_.releases.front().request->operation_id, release_operation_id);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled;
      }));
  acknowledge_terminal();
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, ConflictingReleaseReadbackInhibitsWithoutReplay)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
    port_.releases, nullptr, "release acknowledgement lost");
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto conflicting = world_snapshot(reserve->world_revision);
  conflicting.robot.revision = conflicting.revision;
  auto decoded = restocker_world_state::task_reservation_from_message(
    reserve->reservation,
    reserve->world_revision);
  ASSERT_TRUE(decoded) << decoded.error().detail;
  decoded.value().reservation_id += 1U;
  conflicting.active_reservation = decoded.value();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(conflicting));
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.releases.empty());
  EXPECT_TRUE(driver_->snapshot().pending_operation);
}

TEST_F(RestockCoordinatorDriverTest, StaleReleaseReadbackAfterReplayInhibits)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
    port_.releases, nullptr, "release acknowledgement lost");
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto still_reserved = world_snapshot(reserve->world_revision);
  still_reserved.robot.revision = still_reserved.revision;
  const auto decoded = restocker_world_state::task_reservation_from_message(
    reserve->reservation, reserve->world_revision);
  ASSERT_TRUE(decoded) << decoded.error().detail;
  still_reserved.active_reservation = decoded.value();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(still_reserved));
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(still_reserved));
  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.releases.empty());
}

TEST_F(RestockCoordinatorDriverTest, InhibitionDuringReleaseRetainsAcknowledgedCapability)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  const auto terminal_task_phase =
    static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase);
  const auto terminal_fault_state =
    static_cast<FaultState>(port_.releases.front().request->terminal_fault_state);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  // Release evidence uses the cleanup-reserved lane and does not consume ordinary capacity.
  for (std::size_t index = 0U; index < 128U; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill inbox"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  EXPECT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
        generation_, 0U, now_, "overflow"}),
    CoordinatorInboxPushResult::kOverflowLatched);

  driver_->pump(now_);
  const auto state = driver_->snapshot();
  ASSERT_TRUE(state.transition);
  EXPECT_EQ(state.transition->state, RestockTaskState::kReleaseTask);
  EXPECT_TRUE(state.transition->reservation_active);
  EXPECT_TRUE(state.active);
  ASSERT_TRUE(state.pending_operation);
  EXPECT_EQ(state.pending_operation->effect, OperationEffect::kReadOnly);
  EXPECT_FALSE(admission_.snapshot().mutation_submission);
  EXPECT_FALSE(admission_.snapshot().inhibited);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  released.robot.task_phase = terminal_task_phase;
  released.robot.fault_state = terminal_fault_state;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().reservation_capability_may_remain);
}

TEST_F(RestockCoordinatorDriverTest, InhibitedUnknownReleasePermitsReadbackProof)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);

  ASSERT_EQ(
    driver_->inbox()->push(
      SnapshotCompletion{
        {generation_, 99U}, snapshot_response(world_snapshot()), "", now_, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  now_ += 100ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  EXPECT_EQ(port_.releases.size(), 1U);

  auto released = world_snapshot(reserve->world_revision + 1U);
  released.robot.revision = released.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_TRUE(driver_->snapshot().inhibited);
}

TEST_F(RestockCoordinatorDriverTest, CancellationQueuedBeforeSelectionPreventsMutation)
{
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "early cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  EXPECT_TRUE(port_.reservations.empty());
  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled;
      }));
}

TEST_F(RestockCoordinatorDriverTest, CancellationRetiresPendingInitialSnapshot)
{
  accept_and_request_snapshot();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_,
        "cancel initial snapshot"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  EXPECT_FALSE(port_.removed_request_ids.empty());
}

TEST_F(RestockCoordinatorDriverTest, RejectedMutationAtDeadlineKeepsOwnershipCoherent)
{
  port_.reject_reserve_once = true;
  port_.before_reserve = [this]() {now_ += 100ms;};
  accept_and_request_snapshot();
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);

  const auto pending = driver_->snapshot().pending_operation;
  const auto admission = admission_.snapshot();
  EXPECT_EQ(pending.has_value(), admission.mutation_submission.has_value());
  if (pending && admission.mutation_submission) {
    EXPECT_EQ(pending->operation_generation, admission.mutation_submission->operation_generation);
  }
}

TEST_F(RestockCoordinatorDriverTest, RejectedReadOnlyAtDeadlineReportsTimeout)
{
  port_.reject_snapshot_once = true;
  port_.before_snapshot = [this]() {now_ += 100ms;};
  ASSERT_EQ(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.transition.fault == RestockTaskFault::kTimedOut;
      }));
  EXPECT_FALSE(driver_->snapshot().pending_operation);
}

TEST_F(RestockCoordinatorDriverTest, RejectsFutureCompletionAndCancelsReadOnlyRequest)
{
  accept_and_request_snapshot();
  auto stale = snapshot_response(world_snapshot());
  ASSERT_EQ(
    driver_->inbox()->push(SnapshotCompletion{{generation_, 99U}, stale, "", now_, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  EXPECT_FALSE(port_.removed_request_ids.empty());
}

TEST_F(RestockCoordinatorDriverTest, DiscardsResolvedCompletionWithoutClearingCurrentMutation)
{
  accept_and_request_snapshot();
  const auto resolved_correlation = port_.snapshots.front().correlation;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  driver_->pump(now_);
  ASSERT_EQ(port_.reservations.size(), 1U);
  const auto current = driver_->snapshot().pending_operation;
  ASSERT_TRUE(current);

  ASSERT_EQ(
    driver_->inbox()->push(
      SnapshotCompletion{
        resolved_correlation, snapshot_response(world_snapshot()), "", now_, std::nullopt}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited);
  ASSERT_TRUE(driver_->snapshot().pending_operation);
  EXPECT_EQ(
    driver_->snapshot().pending_operation->operation_generation,
    current->operation_generation);
}

TEST_F(RestockCoordinatorDriverTest, InconsistencyRetainsActiveReservationCapability)
{
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);

  auto invalid = validation_response(*reserve);
  invalid->reservation.object_id += 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(port_.validations, invalid);
  driver_->pump(now_);

  const auto driver_state = driver_->snapshot();
  ASSERT_TRUE(driver_state.transition);
  EXPECT_EQ(driver_state.transition->state, RestockTaskState::kRequestOperator);
  EXPECT_TRUE(driver_state.transition->reservation_active);
  EXPECT_TRUE(driver_state.active);
  EXPECT_TRUE(driver_state.inhibited);
  // The capability is retained for the operator, but the goal must say an operator is required;
  // withholding the result until an impossible release proof arrives leaves the reconciling party
  // waiting forever.
  EXPECT_TRUE(driver_state.terminal_output_emitted);
  const auto outputs = driver_->take_outputs();
  const auto aborted = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kAborted;
    });
  ASSERT_NE(aborted, outputs.end());
  EXPECT_EQ(aborted->outcome, RestockActionOutcome::kExternalInconsistency);
  EXPECT_TRUE(
    std::ranges::none_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled ||
               output.kind == CoordinatorDriverOutputKind::kSucceeded;
      }));
  const auto admission = admission_.snapshot();
  EXPECT_EQ(admission.phase, GoalSlotPhase::kActive);
  EXPECT_TRUE(admission.inhibited);
}

TEST_F(RestockCoordinatorDriverTest, WholeTaskDeadlineAbortsWithoutReservation)
{
  accept_and_request_snapshot();
  now_ = SteadyTime{} + 1000ms;
  driver_->pump(now_);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kTaskDeadline);
  EXPECT_EQ(admission.first_termination->arrived_at, now_);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kAborted &&
               output.transition.fault == RestockTaskFault::kTimedOut;
      }));
  EXPECT_FALSE(port_.removed_request_ids.empty());
  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, MarkerInstalledBeforeCancelDoesNotRetroactivelyWinAuthority)
{
  expect_deposit(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxPersistenceStatus::kEventOwned);
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 1U; index < capacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  const auto marker_at = now_ + 2ms;
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_, 0U, marker_at, "overflow"}),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned);

  const auto cancel_at = now_ + 7ms;
  const auto cancel = admission_.request_cancel(goal_id(), generation_, cancel_at);
  ASSERT_EQ(cancel.status(), GoalTerminationLatchStatus::kLatched);
  const auto first_record = cancel.record();
  ASSERT_TRUE(first_record);
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, cancel_at, "cancel"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kInhibitedMarkerOwned);

  driver_->pump(now_ + 10ms);
  const auto admission = admission_.snapshot();
  ASSERT_TRUE(admission.first_termination);
  EXPECT_EQ(admission.first_termination.get(), first_record.get());
  EXPECT_EQ(admission.first_termination->kind, GoalTerminationKind::kUserCancel);
  EXPECT_EQ(admission.first_termination->arrived_at, cancel_at);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, MarkerPumpedBeforeCancelKeepsInboxLossAuthority)
{
  expect_deposit(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxPersistenceStatus::kEventOwned);
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 1U; index < capacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  const auto marker_at = now_ + 11ms;
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_, 0U, marker_at, "overflow"}),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned);

  driver_->pump(now_ + 20ms);
  const auto first_record = admission_.snapshot().first_termination;
  ASSERT_TRUE(first_record);
  EXPECT_EQ(first_record->kind, GoalTerminationKind::kInboxOverflow);
  EXPECT_EQ(first_record->arrived_at, marker_at);

  // The later authority call has an earlier timestamp: first-call authority, not timestamp sorting,
  // determines the immutable record.
  const auto cancel = admission_.request_cancel(goal_id(), generation_, now_ + 3ms);
  EXPECT_EQ(cancel.status(), GoalTerminationLatchStatus::kAlreadyLatched);
  EXPECT_EQ(cancel.record().get(), first_record.get());
  EXPECT_EQ(cancel.record()->kind, GoalTerminationKind::kInboxOverflow);
  EXPECT_EQ(cancel.record()->arrived_at, marker_at);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, CleanupEvidenceLossMarkerFailStopsActiveGeneration)
{
  accept_and_request_snapshot();
  auto response = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  for (std::size_t index = 0U; index < CoordinatorInbox::kCleanupEmergencyCapacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push_cleanup(
        ReleaseReservationCompletion{
          {generation_, index + 1U}, response, "", now_, std::nullopt}),
      CoordinatorCleanupDeposit::kAccepted);
  }
  expect_deposit(
    driver_->inbox()->push_cleanup(
      ReleaseReservationCompletion{
        {generation_, 99U}, response, "", now_ + 1ms, std::nullopt}),
    CoordinatorInboxDepositStatus::kEvidenceLost,
    CoordinatorInboxPersistenceStatus::kEvidenceLossMarkerOwned);

  driver_->pump(now_ + 2ms);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, AccountingConflictStopsBeforePreContextForwardWork)
{
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 0U; index < capacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
        generation_, 0U, now_, "loss"}),
    CoordinatorInboxPushResult::kOverflowLatched);
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_ + 1U, 0U, now_, "foreign"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kUnresolved);
  ASSERT_TRUE(driver_->inbox()->try_pop());

  driver_->pump(now_);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_FALSE(admission_.snapshot().first_termination);
}

TEST_F(RestockCoordinatorDriverTest, AccountingConflictAfterForwardCheckStopsNextPump)
{
  expect_deposit(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxPersistenceStatus::kEventOwned);
  port_.before_snapshot = [this]() {
    expect_deposit(
      driver_->inbox()->push(
        CoordinatorControlEvent{
          CoordinatorControlKind::kHeartbeat, 0U, 0U, now_, "invalid generation"}),
      CoordinatorInboxDepositStatus::kEvidenceConflict,
      CoordinatorInboxPersistenceStatus::kUnresolved);
  };

  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(driver_->inbox()->snapshot().generation_accounting_conflict);

  port_.before_snapshot = {};
  driver_->pump(now_ + 1ms);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(port_.snapshots.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, ForeignOverflowMarkerCannotFaultActiveGeneration)
{
  accept_and_request_snapshot();
  const auto foreign_generation = generation_ + 1U;
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 0U; index < capacity; ++index) {
    expect_deposit(
      driver_->inbox()->push(
        CoordinatorControlEvent{
          CoordinatorControlKind::kHeartbeat, foreign_generation, 0U, now_, "foreign fill"}),
      CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxPersistenceStatus::kEventOwned);
  }
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
        foreign_generation, 0U, now_ + 1ms,
        "foreign overflow"}),
    CoordinatorInboxDepositStatus::kOverflowLatched,
    CoordinatorInboxPersistenceStatus::kOverflowMarkerOwned);

  driver_->pump(now_ + 2ms);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(admission_.snapshot().first_termination);
  EXPECT_EQ(port_.snapshots.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, GateFaultBeforePumpCheckpointStopsForwardWork)
{
  accept_and_request_snapshot();
  const auto snapshots_before = port_.snapshots.size();
  ASSERT_EQ(
    accepted_gate_.gate->mark_synchronization_failure(),
    GenerationSynchronizationFailureStatus::kMarkedFailed);

  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(port_.snapshots.size(), snapshots_before);
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, UnresolvedDepositAtPumpEntryStopsForwardWork)
{
  accept_and_request_snapshot();
  const auto snapshots_before = port_.snapshots.size();
  const auto result = deposit_for_generation(
    accepted_gate_.gate, goal_id(), generation_, []() {
      return CoordinatorInboxDepositResult{CoordinatorInboxDepositStatus::kEvidenceConflict,
      CoordinatorInboxPersistenceStatus::kUnresolved};
    });
  EXPECT_EQ(result.outcome, GenerationScopedInboxDepositOutcome::kUnresolved);
  EXPECT_TRUE(accepted_gate_.gate->snapshot().deposit_unresolved);

  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(port_.snapshots.size(), snapshots_before);
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, ThrowingCallbackClockFailsGateAndStopsForwardWork)
{
  install_driver(
    generated_grasps,
    []() -> SteadyTime {throw std::runtime_error("async evidence clock failed");});
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(world_snapshot()));
  EXPECT_EQ(
    accepted_gate_.gate->snapshot().retirement_fence_state,
    GenerationRetirementFenceState::kFailed);
  EXPECT_EQ(driver_->inbox()->snapshot().size, 0U);

  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_TRUE(port_.snapshots.empty());
}

TEST_F(RestockCoordinatorDriverTest, GateFaultAfterCheckedForwardPhaseStopsNextPump)
{
  expect_deposit(
    driver_->inbox()->push_accepted_goal(
      CoordinatorAcceptedGoal{goal_id(),
        generation_,
        {},
        now_,
        rclcpp::Time(std::int64_t{1'000'000'000}, RCL_ROS_TIME),
        std::nullopt,
        accepted_gate_.gate},
      std::nullopt, now_),
    CoordinatorInboxDepositStatus::kAccepted, CoordinatorInboxPersistenceStatus::kEventOwned);
  port_.before_snapshot = [this]() {
    ASSERT_EQ(
      accepted_gate_.gate->mark_synchronization_failure(),
      GenerationSynchronizationFailureStatus::kMarkedFailed);
  };

  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(
    accepted_gate_.gate->snapshot().retirement_fence_state,
    GenerationRetirementFenceState::kFailed);

  port_.before_snapshot = {};
  driver_->pump(now_ + 1ms);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(port_.snapshots.size(), 1U);
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, IdleDriverSnapshotHasNoInactiveAfterAckLineage)
{
  EXPECT_FALSE(driver_->snapshot().active);
  EXPECT_FALSE(driver_->snapshot().inactive_after_terminal_ack_generation);
}

TEST_F(RestockCoordinatorDriverTest, ActiveDriverSnapshotHasNoInactiveAfterAckLineage)
{
  accept_and_request_snapshot();
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_FALSE(driver_->snapshot().inactive_after_terminal_ack_generation);
}

TEST_F(RestockCoordinatorDriverTest, RejectedTerminalAckDoesNotPublishLineage)
{
  cancel_at_staging_and_complete_release();
  EXPECT_FALSE(driver_->acknowledge_terminal_delivery(generation_ + 1U));
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_FALSE(driver_->snapshot().inactive_after_terminal_ack_generation);
}

TEST_F(RestockCoordinatorDriverTest, AcknowledgedCleanupPublishesStableIdleLineage)
{
  const auto retired = generation_;
  cancel_at_staging_and_complete_release();
  acknowledge_terminal();
  EXPECT_FALSE(driver_->snapshot().active);
  ASSERT_TRUE(driver_->snapshot().inactive_after_terminal_ack_generation);
  EXPECT_EQ(*driver_->snapshot().inactive_after_terminal_ack_generation, retired);

  driver_->pump(now_ + 1ms);
  ASSERT_TRUE(driver_->snapshot().inactive_after_terminal_ack_generation);
  EXPECT_EQ(*driver_->snapshot().inactive_after_terminal_ack_generation, retired);
  EXPECT_EQ(admission_.snapshot().phase, GoalSlotPhase::kIdle);
}

TEST_F(RestockCoordinatorDriverTest, InvalidAcceptedIngressDoesNotClearPriorLineage)
{
  const auto retired = generation_;
  cancel_at_staging_and_complete_release();
  acknowledge_terminal();
  ASSERT_EQ(driver_->snapshot().inactive_after_terminal_ack_generation, retired);

  admit_next_goal();
  (void)driver_->take_outputs();
  expect_accepted_gate_rejected(nullptr, goal_id(), generation_);
  ASSERT_TRUE(driver_->snapshot().inactive_after_terminal_ack_generation);
  EXPECT_EQ(*driver_->snapshot().inactive_after_terminal_ack_generation, retired);
}

TEST_F(RestockCoordinatorDriverTest, LaterAcceptedContextClearsPriorLineage)
{
  const auto retired = generation_;
  cancel_at_staging_and_complete_release();
  acknowledge_terminal();
  ASSERT_EQ(driver_->snapshot().inactive_after_terminal_ack_generation, retired);

  admit_next_goal();
  accept_and_request_snapshot();
  EXPECT_TRUE(driver_->snapshot().active);
  EXPECT_EQ(driver_->snapshot().goal_generation, generation_);
  EXPECT_NE(generation_, retired);
  EXPECT_FALSE(driver_->snapshot().inactive_after_terminal_ack_generation);
}

TEST_F(RestockCoordinatorDriverTest, AccountingConflictStopsActiveForwardWork)
{
  accept_and_request_snapshot();
  const auto snapshots_before = port_.snapshots.size();
  const auto capacity = driver_->inbox()->snapshot().capacity;
  for (std::size_t index = 0U; index < capacity; ++index) {
    ASSERT_EQ(
      driver_->inbox()->push(
        CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
          generation_, 0U, now_, "fill"}),
      CoordinatorInboxPushResult::kAccepted);
  }
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kHeartbeat,
        generation_, 0U, now_, "loss"}),
    CoordinatorInboxPushResult::kOverflowLatched);
  expect_deposit(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kHeartbeat, generation_ + 1U, 0U, now_, "foreign"}),
    CoordinatorInboxDepositStatus::kInhibited,
    CoordinatorInboxPersistenceStatus::kUnresolved);
  ASSERT_TRUE(driver_->inbox()->try_pop());

  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_EQ(port_.snapshots.size(), snapshots_before);
}

TEST_F(RestockCoordinatorDriverTest, HoldsStagedCandidatesWhenNoMotionBackendIsComposed)
{
  (void)reach_staging_boundary();
  driver_->pump(now_);

  // Without a motion port the coordinator must not claim progress it cannot make.
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_FALSE(driver_->snapshot().pending_operation);
  EXPECT_EQ(motion_.submit_attempts, 0U);
}

TEST_F(RestockCoordinatorDriverTest, SubmitsTheStagedPreGraspPoseToTheMotionPort)
{
  reach_pregrasp_with_motion();

  ASSERT_EQ(motion_.submit_attempts, 1U);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(driver_->snapshot().pending_operation);

  // The submitted goal must be the staged candidate-zero pre-grasp pose.
  const auto & staged = driver_->snapshot();
  ASSERT_TRUE(staged.transition);
  EXPECT_TRUE(valid_motion_goal(motion_.submission().goal));
  ASSERT_TRUE(motion_.submission().goal.required_linear_continuation_pose);
  ASSERT_TRUE(
    motion_.submission().goal.required_linear_continuation_gripper_joint_position_m);
  EXPECT_DOUBLE_EQ(
    motion_.submission().goal.required_linear_continuation_vertical_margin_m, 0.005);
  EXPECT_TRUE(motion_.submission().goal.required_postcontinuation_linear_retract_pose);
  EXPECT_TRUE(motion_.submission().goal.required_postcontinuation_linear_egress_pose);
  EXPECT_TRUE(motion_.submission().goal.required_postcontinuation_gripper_joint_position_m);
  EXPECT_EQ(motion_.submission().goal.free_space_plan_candidates, 4U);
  EXPECT_TRUE(motion_.submission().goal.reset_planning_interface_before_plan);
  EXPECT_EQ(motion_.submission().correlation.goal_generation, generation_);
}

TEST_F(RestockCoordinatorDriverTest, SucceededMotionAdvancesPastTheStagingBoundary)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);

  // Execute-pre-grasp records that the trajectory ran; the jaws open before the approach.
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForApproach);
}

TEST_F(RestockCoordinatorDriverTest, PreGraspSuccessChainsIntoTheApproachSegment)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  const auto pregrasp_target = motion_.submission().goal.planning_frame_from_tool0;
  const auto required_approach =
    motion_.submission().goal.required_linear_continuation_pose;
  ASSERT_TRUE(required_approach);

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);

  // Jaws open first; then the approach is planned against the candidate's grasp pose, not the
  // pre-grasp pose.
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(motion_.submit_attempts, 2U);
  ASSERT_TRUE(motion_.outstanding());
  const auto approach_target = motion_.submission().goal.planning_frame_from_tool0;
  EXPECT_FALSE(approach_target.translation().isApprox(pregrasp_target.translation()));
  EXPECT_TRUE(approach_target.isApprox(*required_approach));
  EXPECT_TRUE(valid_motion_goal(motion_.submission().goal));
}

TEST_F(
  RestockCoordinatorDriverTest,
  SuccessfulRetractAuthorizesBoundedReadOnlyDestinationRefreshRecovery)
{
  reach_close_gripper();
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  const auto complete_snapshot = [this](bool held, bool obstructed) {
    ASSERT_EQ(port_.snapshots.size(), 1U);
    ASSERT_TRUE(reserve_response_);
    auto value = reserved_world_snapshot(*reserve_response_);
    value.revision = ++observation_revision_;
    const auto selected = selected_pair(value);
    value.lanes.at(selected.lane_id).obstructed = obstructed;
    if (held) {
      value.robot.held_object = selected.object_id;
      value.objects.at(selected.object_id).grasp_state =
        restocker_world_state::GraspState::Attached;
      ASSERT_TRUE(value.active_reservation);
      value.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
    }
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(value));
    driver_->pump(now_);
  };

  complete_snapshot(false, false);  // verify grasp
  ASSERT_TRUE(attachment_.outstanding());
  attachment_.complete(AttachmentOutcome::kSucceeded, "attachment committed");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_EQ(motion_.submission().goal.label, "retract");
  motion_.complete(MotionOutcome::kSucceeded, "retract reached a terminal stop");
  now_ += 1ms;
  driver_->pump(now_);

  // Two immediate snapshots see transient obstruction and exhaust operation retries. The successful
  // retract is still terminal-stop evidence, so recovery may take an independent stopped-robot
  // snapshot.
  complete_snapshot(true, true);
  complete_snapshot(true, true);
  ASSERT_FALSE(driver_->snapshot().inhibited) << driver_->snapshot().transition->detail;
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  complete_snapshot(true, true);  // recovery's stopped-robot evidence
  std::string output_details;
  for (const auto & output : driver_->take_outputs()) {
    output_details += output.detail + " | ";
  }
  ASSERT_FALSE(driver_->snapshot().inhibited) << output_details;
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kObserveDestination);

  complete_snapshot(true, false);
  ASSERT_FALSE(driver_->snapshot().inhibited);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_EQ(motion_.submission().goal.label, "carry-start");
}

TEST_F(RestockCoordinatorDriverTest, ApproachExecutionFailureTerminatesWithTheArmMoved)
{
  reach_pregrasp_with_motion();
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(motion_.submit_attempts, 2U);

  motion_.complete(MotionOutcome::kExecutionFailed, "controller aborted mid-approach");
  now_ += 1ms;
  driver_->pump(now_);

  // A part-way approach stop leaves the arm at an unknown pose near the product, so the task must
  // not replan.
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kPlanApproach);
}

TEST_F(RestockCoordinatorDriverTest, PlanningFailureIsRetriedBecauseNothingWasCommanded)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kPlanningFailed, "no collision-free plan");
  now_ += 1ms;
  driver_->pump(now_);

  // A refused plan cannot have moved the arm, so the task retries rather than terminating.
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_NE(driver_->snapshot().transition->fault, RestockTaskFault::kExternalInconsistency);
}

// The action's feedback classification comes from the call-site policy, not from the command or
// from string matching: a pre-insert/carry-start planning miss the kPlanningFailed site tagged
// kRetriablePlanningDiagnostic is published as a bounded-retry diagnostic without the forbidden
// abort markers, while every unclassified retryable — backend unavailable, submission refused,
// pre-grasp candidate fallthrough — keeps the raw detail verbatim, and terminal faults keep
// their detail so fail-closed markers stay visible on failed goals.
TEST(RestockFeedbackDetailForDispatch, ClassificationComesFromTheCallSitePolicy)
{
  RestockTaskTransition planning_miss;
  planning_miss.state = RestockTaskState::kPlanPreInsert;
  planning_miss.command = RestockTaskCommand::kPlanPreInsert;
  planning_miss.detail =
    "pre-insert motion planning failed: motion planning did not produce a valid solution "
    "(MoveIt reported FAILURE, code 99999)";
  planning_miss.attempt = 2U;

  const auto diagnostic = feedback_detail_for_dispatch(
    RestockTaskEvent::kRetryableFailure, FeedbackDetailPolicy::kRetriablePlanningDiagnostic,
    planning_miss);
  EXPECT_EQ(diagnostic.find("MoveIt reported FAILURE"), std::string::npos) << diagnostic;
  EXPECT_EQ(diagnostic.find("motion planning failed"), std::string::npos) << diagnostic;
  EXPECT_NE(diagnostic.find("bounded retry in progress"), std::string::npos) << diagnostic;
  EXPECT_NE(diagnostic.find("pre_insert"), std::string::npos) << diagnostic;

  // Counterexample: identical state and event, but the raw (non-planning) policy — the
  // backend-unavailable / submission-refused path — retains every byte of its detail.
  RestockTaskTransition backend_down = planning_miss;
  backend_down.detail = "motion submission was refused: the motion backend is busy";
  EXPECT_EQ(
    feedback_detail_for_dispatch(
      RestockTaskEvent::kRetryableFailure, FeedbackDetailPolicy::kRawEvidence, backend_down),
    backend_down.detail);

  // Pre-grasp candidate fallthrough with raw policy keeps the planner-refusal text, markers
  // included (the manipulation acceptance test catches grasp-quality refusals with it).
  RestockTaskTransition fallthrough = planning_miss;
  fallthrough.state = RestockTaskState::kPlanPreGrasp;
  fallthrough.command = RestockTaskCommand::kPlanPreGrasp;
  fallthrough.detail =
    "pre-grasp motion planning failed: candidate refused; falling through "
    "(MoveIt reported FAILURE, code 99999)";
  EXPECT_EQ(
    feedback_detail_for_dispatch(
      RestockTaskEvent::kRetryableFailure, FeedbackDetailPolicy::kRawEvidence, fallthrough),
    fallthrough.detail);

  // Terminal faults and non-retry events never redact, even with the diagnostic policy.
  RestockTaskTransition faulted = planning_miss;
  faulted.state = RestockTaskState::kFault;
  EXPECT_EQ(
    feedback_detail_for_dispatch(
      RestockTaskEvent::kRetryableFailure, FeedbackDetailPolicy::kRetriablePlanningDiagnostic,
      faulted),
    planning_miss.detail);
  EXPECT_EQ(
    feedback_detail_for_dispatch(
      RestockTaskEvent::kTerminalFailure, FeedbackDetailPolicy::kRetriablePlanningDiagnostic,
      planning_miss),
    planning_miss.detail);
  EXPECT_EQ(
    feedback_detail_for_dispatch(
      RestockTaskEvent::kOperationSucceeded, FeedbackDetailPolicy::kRetriablePlanningDiagnostic,
      planning_miss),
    planning_miss.detail);
}

TEST_F(RestockCoordinatorDriverTest, RetriablePreInsertMissFeedbackIsANonTerminalRetryDiagnostic)
{
  reach_preinsert_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_EQ(motion_.submission().goal.label, "pre-insert");
  (void)driver_->take_outputs();

  motion_.complete(
    MotionOutcome::kPlanningFailed,
    "motion planning did not produce a valid solution (MoveIt reported FAILURE, code 99999)");
  now_ += 1ms;
  driver_->pump(now_);

  const auto snapshot = driver_->snapshot();
  ASSERT_FALSE(snapshot.inhibited);
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kFault);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kRequestOperator);
  EXPECT_EQ(snapshot.transition->fault, RestockTaskFault::kNone);

  const auto outputs = driver_->take_outputs();
  bool saw_retry_diagnostic = false;
  for (const auto & output : outputs) {
    if (output.kind != CoordinatorDriverOutputKind::kFeedback) {
      continue;
    }
    EXPECT_EQ(output.detail.find("MoveIt reported FAILURE"), std::string::npos)
      << output.detail;
    EXPECT_EQ(output.detail.find("motion planning failed"), std::string::npos) << output.detail;
    if (output.detail.find("bounded retry in progress") != std::string::npos) {
      saw_retry_diagnostic = true;
    }
  }
  EXPECT_TRUE(saw_retry_diagnostic);
}

// Counterexample at the driver level: a non-planning retryable failure while the pre-insert is
// the outstanding leg (backend unavailable) is dispatched with the raw policy, so its evidence
// must reach the action unredacted.
TEST_F(RestockCoordinatorDriverTest, NonPlanningRetryableInPreInsertKeepsRawDetail)
{
  reach_preinsert_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();

  motion_.complete(MotionOutcome::kUnavailable, "the motion backend is unavailable right now");
  now_ += 1ms;
  driver_->pump(now_);

  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kFault);

  const auto outputs = driver_->take_outputs();
  bool saw_raw_backend_detail = false;
  for (const auto & output : outputs) {
    if (output.kind != CoordinatorDriverOutputKind::kFeedback) {
      continue;
    }
    if (output.detail.find("unavailable right now") != std::string::npos) {
      saw_raw_backend_detail = true;
    }
    EXPECT_EQ(output.detail.find("bounded retry in progress"), std::string::npos)
      << output.detail;
  }
  EXPECT_TRUE(saw_raw_backend_detail);
}

// Pre-grasp planner refusals fall through to the next grasp candidate with raw evidence; that
// text must survive onto the action so stock-side grasp-quality refusals stay catchable.
TEST_F(RestockCoordinatorDriverTest, PreGraspCandidateFallthroughKeepsRawRefusalEvidence)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();

  motion_.complete_planner_refusal(
    "pre-grasp motion planning failed: candidate refused; falling through "
    "(MoveIt reported FAILURE, code 99999)");
  now_ += 1ms;
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  bool saw_raw_fallthrough = false;
  for (const auto & output : outputs) {
    if (output.detail.find("falling through") == std::string::npos) {
      continue;
    }
    saw_raw_fallthrough = true;
    EXPECT_NE(output.detail.find("MoveIt reported FAILURE"), std::string::npos)
      << output.detail;
    EXPECT_NE(output.detail.find("motion planning failed"), std::string::npos)
      << output.detail;
    EXPECT_EQ(output.detail.find("bounded retry in progress"), std::string::npos)
      << output.detail;
  }
  EXPECT_TRUE(saw_raw_fallthrough);
}

TEST_F(RestockCoordinatorDriverTest, ExecutionFailureTerminatesInsteadOfRetryingBlind)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kExecutionFailed, "controller aborted mid-trajectory");
  now_ += 1ms;
  driver_->pump(now_);

  // The arm may have moved without reaching the goal; replanning from an unknown state is unsafe.
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kPlanPreGrasp);
}

// A free-space traverse aborted by the controllers leaves the arm stationary in a describable
// state. Bounded recovery replans that segment from there, separating a transient plant fault from
// a task needing an operator.
TEST_F(RestockCoordinatorDriverTest, FreeSpaceStopAtRestIsRecoveredByReplanningTheSegment)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_EQ(motion_.submit_attempts, 1U);

  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);

  // Recovery observes the authoritative world state before replanning instead of assuming the stop.
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  EXPECT_EQ(motion_.submit_attempts, 1U);

  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(stopped));
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
  EXPECT_TRUE(motion_.outstanding());
  // A task that recovered and resumed is not failing; the terminal reservation release reads this
  // to pick its outcome.
  EXPECT_EQ(driver_->snapshot().transition->fault, RestockTaskFault::kNone);
}

// Card 039 instrumentation: a recurrence of a re-entry terminal has to be reconstructible from one
// receipt, so every entry into and exit from bounded recovery names its attempt number, the budget,
// and the state it left or resumed at.
TEST_F(RestockCoordinatorDriverTest, ReportsTheRecoveryAttemptThatEnteredAndResumed)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().transition);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);

  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_NE(
    receipts.find("recovery attempt 1 of 2 entered from "),
    std::string::npos) << receipts;

  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  const auto recovered_revision = stopped.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(stopped));
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);

  receipts.clear();
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_NE(
    receipts.find("recovery attempt 1 of 2 resumed at plan_pre_grasp"),
    std::string::npos) << receipts;
  // Milestone 10 §6 rung 3 (Card 051): the retry is receipted against the fresh revision the
  // recovery adopted, with both budgets named — the retained evidence that failed is not reused.
  EXPECT_NE(
    receipts.find(
      "recovery rung 3 (retry with fresh evidence): resuming at plan_pre_grasp against world "
      "revision " + std::to_string(recovered_revision) + ", operation attempt"),
    std::string::npos) << receipts;
}

// A straight-line segment is interpolated from the configuration the preceding free-space traverse
// left, and not every configuration reaching the same tool pose admits the same straight travel.
// Replanning the same line asks the same question, so recovery goes back to the traverse that chose
// it.
TEST_F(RestockCoordinatorDriverTest, UnplannableLinearSegmentRecoversByRepositioning)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  const auto pregrasp_target = motion_.submission().goal.planning_frame_from_tool0;

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(motion_.submit_attempts, 2U);

  // Exhaust the bounded retries of the straight line; nothing was commanded.
  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(
      MotionOutcome::kPlanningFailed,
      "linear path stopped 20% of the way to the goal; forearm against a shelf divider");
    now_ += 1ms;
    driver_->pump(now_);
  }

  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(stopped));
  driver_->pump(now_);

  // The free-space traverse is planned again, not the straight line that could not be planned.
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(
    motion_.submission().goal.planning_frame_from_tool0.translation().isApprox(
      pregrasp_target.translation()));
}

// An absent backend is not fixed by a replan, so recovery fails closed without spending the budget.
TEST_F(RestockCoordinatorDriverTest, UnavailableMotionBackendIsNotRecovered)
{
  reach_pregrasp_with_motion();

  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kUnavailable, "MoveIt is not running");
    now_ += 1ms;
    driver_->pump(now_);
  }

  EXPECT_TRUE(port_.snapshots.empty());
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// Without the backend establishing that the trajectory ended, the robot may still be moving and
// recovery must refuse to replan.
TEST_F(RestockCoordinatorDriverTest, RecoveryIsRefusedWithoutEvidenceThatTheArmStopped)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kExecutionFailed, "execution ended", false);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_EQ(motion_.submit_attempts, 1U);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// Card 086 stage 1a ripple: a MoveIt execute() TIMED_OUT now completes as kExecutionFailed with
// execution_reached_terminal_stop=false (the adapter no longer promotes it to a stop), so this
// driver refuses the recovery replan exactly as for any unestablished stop, and says why.
TEST_F(RestockCoordinatorDriverTest, ATimedOutExecuteCompletionIsNotRecoveredFrom)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(
    MotionOutcome::kExecutionFailed, "pre-grasp: MoveIt timed out (TIMED_OUT, code -6)", false);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_EQ(motion_.submit_attempts, 1U);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_NE(
    receipts.find("never established that the trajectory reached a terminal state"),
    std::string::npos) << receipts;
}

// ---- Card 086 stage 1b: the motion evidence the driver puts on every output's metrics --------
//
// motion_definitely_not_started only when NO motion, gripper or attachment goal of the whole
// goal was commanded (a submission answered with a proven-not-started outcome does not count);
// execution_reached_terminal_stop only when the LAST command was a motion segment whose terminal
// controller result was observed. Never while the driver is inhibited.

static const CoordinatorDriverOutput & last_output(
  const std::vector<CoordinatorDriverOutput> & outputs)
{
  EXPECT_FALSE(outputs.empty());
  return outputs.back();
}

TEST_F(RestockCoordinatorDriverTest, EvidenceFirstLegPlanFailureProvesNothingWasCommanded)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();
  // While the first leg is submitted but unanswered, non-start is NOT yet provable.
  now_ += 1ms;
  driver_->pump(now_);
  for (const auto & output : driver_->take_outputs()) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started) << "a command may be in flight";
  }

  motion_.complete(MotionOutcome::kPlanningFailed, "no collision-free plan");
  now_ += 1ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  ASSERT_FALSE(outputs.empty());
  EXPECT_TRUE(last_output(outputs).metrics.motion_definitely_not_started);
  EXPECT_FALSE(last_output(outputs).metrics.execution_reached_terminal_stop);
}

TEST_F(RestockCoordinatorDriverTest, EvidenceRefusedSubmissionLeavesNonStartProven)
{
  // A submission the port refused because it cannot take work at all (stopping) never reached
  // MoveIt, so nothing was commanded.
  motion_.refuse_next(MotionSubmitStatus::kUnavailable, "the motion port is stopping");
  reach_pregrasp_with_motion();
  const auto outputs = driver_->take_outputs();
  // The driver retries after the refusal, so look at what the refusal itself left behind.
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.metrics.motion_definitely_not_started &&
               !output.metrics.execution_reached_terminal_stop;
      }));
}

// Review B1: an execute-phase "unavailable" (COMMUNICATION_FAILURE/CRASH after MoveIt execute() was
// called) reaches the driver as kUnavailable but the arm may be moving: never "not started".
TEST_F(RestockCoordinatorDriverTest, EvidenceUnavailableAfterExecuteIsNeverNonStart)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();
  motion_.complete_after_execute(
    MotionOutcome::kUnavailable, "pre-grasp: MoveIt lost contact after execute()");
  const auto outputs = pump_collect(4);
  ASSERT_FALSE(outputs.empty());
  for (const auto & output : outputs) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
  EXPECT_TRUE(has_terminal_output(outputs)) << "the evidence reaches a terminal output";
}

// Review 3 S1 (decided fail-closed): an execute-phase loss must not be retried in-goal by a fresh
// submission while the lost trajectory may still be executing. It takes the same refusal path as
// the other possibly-moving failures and the goal ends with the arm's stop not established.
TEST_F(RestockCoordinatorDriverTest, ExecutePhaseLossIsNotRetriedByAFreshSubmission)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_EQ(motion_.submit_attempts, 1U);
  (void)driver_->take_outputs();
  motion_.complete_after_execute(
    MotionOutcome::kUnavailable, "pre-grasp: MoveIt lost contact after execute()");
  const auto outputs = pump_collect(8);
  EXPECT_EQ(motion_.submit_attempts, 1U) << "no second motion goal while the arm may be moving";
  EXPECT_FALSE(motion_.outstanding());
  std::string receipts;
  for (const auto & output : outputs) {
    receipts += output.detail + "\n";
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
  EXPECT_NE(receipts.find("recovery is refused"), std::string::npos) << receipts;
  EXPECT_TRUE(has_terminal_output(outputs)) << receipts;
}

// The unchanged class: a backend that was unavailable BEFORE anything was submitted still retries.
TEST_F(RestockCoordinatorDriverTest, PreSubmissionUnavailableStillRetries)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kUnavailable, "MoveIt is not running");
  for (int step = 0; step < 8 && motion_.submit_attempts < 2U; ++step) {
    now_ += 50ms;
    driver_->pump(now_);
  }
  EXPECT_GE(motion_.submit_attempts, 2U) << "a plain pre-submission unavailable is retried";
}

// The multi-slice shape: an earlier segment completed with a stop, then a later one's execute() is
// handed to the backend and the backend becomes unavailable. The restore must not fall back to a
// state before the whole goal, and the stop must not survive the new command.
TEST_F(RestockCoordinatorDriverTest, EvidenceUnavailableAfterExecuteOfALaterSegmentKeepsCommanded)
{
  reach_pregrasp_with_motion();
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();
  motion_.complete_after_execute(
    MotionOutcome::kUnavailable,
    "slice 2 of approach lost the backend");
  const auto outputs = pump_collect(4);
  ASSERT_FALSE(outputs.empty());
  for (const auto & output : outputs) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
  EXPECT_TRUE(has_terminal_output(outputs));
}

// Review S1: a kBusy refusal means the port still owns an outstanding goal, so the arm may be
// moving.
TEST_F(RestockCoordinatorDriverTest, EvidenceBusyRefusalDoesNotProveNonStart)
{
  motion_.refuse_next(MotionSubmitStatus::kBusy, "the motion backend is busy");
  reach_pregrasp_with_motion();
  const auto outputs = driver_->take_outputs();
  // Everything the driver says from the refusal on (earlier outputs precede any command).
  const auto refused = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.detail.find("submission was refused") != std::string::npos;
    });
  ASSERT_NE(refused, outputs.end());
  for (auto output = refused; output != outputs.end(); ++output) {
    EXPECT_FALSE(output->metrics.motion_definitely_not_started);
  }
}

TEST_F(RestockCoordinatorDriverTest, EvidenceCancelBeforeAnyCommandProvesNonStart)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = reserve->world_revision + 1U;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(port_.releases, release);
  driver_->pump(now_);
  auto released = world_snapshot(release->world_revision);
  released.robot.revision = released.revision;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(released));
  driver_->pump(now_);

  bool saw_canceled = false;
  for (const auto & output : driver_->take_outputs()) {
    if (output.kind == CoordinatorDriverOutputKind::kCanceled) {
      saw_canceled = true;
      EXPECT_TRUE(output.metrics.motion_definitely_not_started);
      EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
    }
  }
  EXPECT_TRUE(saw_canceled);
}

TEST_F(RestockCoordinatorDriverTest, EvidenceValidationFailureBeforeAnyLegProvesNonStart)
{
  install_driver(
    [](const auto &, const auto &, const auto &, const auto &) {
      return GraspCandidateResult<GraspCandidateBatch>::failure(
        GraspCandidateError{
          GraspCandidateErrorCode::NoValidJawTarget, "selected can exceeds jaw opening"});
    });
  const auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  complete_failed_safe_release(*reserve);

  bool saw_terminal = false;
  for (const auto & output : driver_->take_outputs()) {
    if (output.kind == CoordinatorDriverOutputKind::kAborted &&
      output.outcome == RestockActionOutcome::kValidationFailed)
    {
      saw_terminal = true;
      EXPECT_TRUE(output.metrics.motion_definitely_not_started);
    }
  }
  EXPECT_TRUE(saw_terminal);
}

TEST_F(RestockCoordinatorDriverTest, EvidenceExecutionFailureWithoutStopProvesNeither)
{
  reach_pregrasp_with_motion();
  (void)driver_->take_outputs();
  motion_.complete(MotionOutcome::kExecutionFailed, "execution ended", false);
  now_ += 1ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  ASSERT_FALSE(outputs.empty());
  for (const auto & output : outputs) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
}

TEST_F(RestockCoordinatorDriverTest, EvidenceExecutionFailureAtAControllerStopReportsTheStop)
{
  reach_pregrasp_with_motion();
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(motion_.submit_attempts, 2U);
  (void)driver_->take_outputs();

  motion_.complete(MotionOutcome::kExecutionFailed, "controller aborted mid-approach", true);
  now_ += 1ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  ASSERT_FALSE(outputs.empty());
  // The tracker observes the stop on the outputs that follow the completion. (This linear-segment
  // failure then ends in an operator-class inhibit, whose terminal carries no evidence by design.)
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return !output.metrics.motion_definitely_not_started &&
               output.metrics.execution_reached_terminal_stop;
      }));
}

TEST_F(RestockCoordinatorDriverTest, EvidenceALaterCommandRetractsAnEarlierStop)
{
  reach_pregrasp_with_motion();
  (void)driver_->take_outputs();
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  // The next command (the gripper) is outstanding: the earlier segment's stop proves nothing now.
  ASSERT_TRUE(gripper_.outstanding());
  (void)driver_->take_outputs();
  // Once the gripper command completes, the outputs that follow no longer carry the arm's stop:
  // the jaws moved after it.
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  ASSERT_FALSE(outputs.empty());
  EXPECT_FALSE(last_output(outputs).metrics.motion_definitely_not_started);
  EXPECT_FALSE(last_output(outputs).metrics.execution_reached_terminal_stop);
}

TEST_F(
  RestockCoordinatorDriverTest,
  EvidenceDeadlineAfterACommandIsNeverNonStartAndInhibitedCarriesNone)
{
  task_configure_ = [](RestockCoordinatorDriverConfig & config) {
    config.task.planning_timeout = 300ms;
    config.task.execution_timeout = 400ms;
    config.task.total_timeout = 600000ms;
  };
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_);
  reach_preinsert_with_motion();
  (void)driver_->take_outputs();
  now_ = *driver_->snapshot().command_deadline;
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().inhibited);
  const auto outputs = driver_->take_outputs();
  ASSERT_FALSE(outputs.empty());
  for (const auto & output : outputs) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
}

TEST_F(RestockCoordinatorDriverTest, EvidenceCancelWhileAMotionIsOutstandingIsNeverNonStart)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  now_ += 1ms;
  driver_->pump(now_);
  for (const auto & output : driver_->take_outputs()) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
}

// A linear segment slides the jaws or held product with millimetres of clearance; a fresh straight
// line may not start from wherever the arm stopped inside one.
TEST_F(RestockCoordinatorDriverTest, LinearSegmentStopAtRestIsStillNotRecovered)
{
  reach_pregrasp_with_motion();
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(motion_.submit_attempts, 2U);

  motion_.complete(MotionOutcome::kExecutionFailed, "controller aborted mid-approach", true);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_TRUE(port_.snapshots.empty());
  EXPECT_EQ(motion_.submit_attempts, 2U);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanApproach);
}

// Milestone 10 §6 (Card 060): the pure rest predicate. Fresh telemetry at or below the rest
// speed establishes a stop; stale or future telemetry establishes neither motion nor rest; a
// fresh sample above the rest speed names its fastest joint.
TEST(RecoveryRestPredicate, EstablishesRestOnlyFromFreshTelemetryAtRest)
{
  restocker_world_state::RobotExecutionState robot;
  robot.telemetry_revision = 7U;
  robot.telemetry_time = rclcpp::Time(std::int64_t{10'000'000'000}, RCL_ROS_TIME);
  const auto at = [](std::int64_t ns) {return rclcpp::Time(ns, RCL_ROS_TIME);};
  constexpr auto kAge = std::chrono::milliseconds(500);
  constexpr auto kSkew = std::chrono::milliseconds(50);

  auto verdict = evaluate_recovery_rest(robot, at(10'400'000'000), kAge, kSkew, 0.05);
  EXPECT_TRUE(verdict.established) << verdict.detail;

  robot.joint_velocities[2] = 0.05;
  EXPECT_TRUE(evaluate_recovery_rest(robot, at(10'400'000'000), kAge, kSkew, 0.05).established)
    << "the rest speed itself is at rest";

  robot.joint_velocities[2] = 0.051;
  verdict = evaluate_recovery_rest(robot, at(10'400'000'000), kAge, kSkew, 0.05);
  EXPECT_FALSE(verdict.established);
  EXPECT_NE(
    verdict.detail.find("the robot is still moving (arm joint 3 speed 0.051"),
    std::string::npos) << verdict.detail;

  robot.joint_velocities[2] = 0.0;
  robot.rail_velocity = std::numeric_limits<double>::quiet_NaN();
  verdict = evaluate_recovery_rest(robot, at(10'400'000'000), kAge, kSkew, 0.05);
  EXPECT_FALSE(verdict.established);
  EXPECT_NE(verdict.detail.find("(rail speed inf"), std::string::npos) << verdict.detail;

  // Slot 13's shape: at rest, but ~120 s stale — neither motion nor rest.
  robot.rail_velocity = 0.0;
  verdict = evaluate_recovery_rest(robot, at(130'000'000'000), kAge, kSkew, 0.05);
  EXPECT_FALSE(verdict.established);
  EXPECT_NE(
    verdict.detail.find("robot telemetry is 120.000 s old (bound 0.500 s)"),
    std::string::npos) << verdict.detail;
  EXPECT_FALSE(
    evaluate_recovery_rest(robot, at(10'500'000'001), kAge, kSkew, 0.05).established)
    << "one nanosecond past the bound is stale";
  EXPECT_TRUE(evaluate_recovery_rest(robot, at(10'500'000'000), kAge, kSkew, 0.05).established);
  EXPECT_TRUE(evaluate_recovery_rest(robot, at(9'950'000'000), kAge, kSkew, 0.05).established);
  EXPECT_FALSE(evaluate_recovery_rest(robot, at(9'949'999'999), kAge, kSkew, 0.05).established)
    << "telemetry beyond the future skew is not evidence";

  robot.telemetry_revision = 0U;
  EXPECT_FALSE(evaluate_recovery_rest(robot, at(10'400'000'000), kAge, kSkew, 0.05).established)
    << "no admitted telemetry establishes nothing";
}

// A zero budget keeps the pre-Card-060 immediate refusal; a settling arm is not waited for.
TEST_F(RestockCoordinatorDriverTest, ZeroStopSettleBudgetRefusesAtOnce)
{
  stop_settle_timeout_ = 0ms;
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto settling = reserved_world_snapshot(*reserve_response_);
  settling.revision = ++observation_revision_;
  settling.robot.joint_velocities = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0};
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(settling));
  for (int pump = 0; pump < 8 && !driver_->snapshot().inhibited; ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty()) << "no re-observation without a budget";
  EXPECT_EQ(motion_.submit_attempts, 1U);
}

// Recovery replans only from a robot the world state shows at rest; a snapshot with joint speed is
// moving.
TEST_F(RestockCoordinatorDriverTest, RecoveryIsRefusedWhileTheRobotIsStillMoving)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  ASSERT_TRUE(reserve_response_);
  auto moving = reserved_world_snapshot(*reserve_response_);
  moving.revision = ++observation_revision_;
  moving.robot.joint_velocities = {0.0, 0.0, 0.9, 0.0, 0.0, 0.0};
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(moving));
  driver_->pump(now_);

  // The observation failed, so the recovery is reported failed, not resumed.
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 1U);
  // Card 051: a failed deterministic recovery is UNSAFE (the stopped state was never
  // re-established) — the ladder does not run; the goal latches. Card 060: an arm still moving
  // after the whole bounded stop-settle hold is exactly that case.
  for (int pump = 0; pump < 64 && !driver_->snapshot().inhibited; ++pump) {
    if (!port_.snapshots.empty()) {
      auto still_moving = reserved_world_snapshot(*reserve_response_);
      still_moving.revision = ++observation_revision_;
      still_moving.robot.joint_velocities = {0.0, 0.0, 0.9, 0.0, 0.0, 0.0};
      port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
        port_.snapshots, snapshot_response(still_moving));
    }
    now_ += 25ms;
    driver_->pump(now_);
  }
  EXPECT_TRUE(driver_->snapshot().inhibited)
    << "an UNSAFE classification latches without attempting the skip";
  EXPECT_EQ(motion_.submit_attempts, 1U);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind(
          "recovery stop-settle hold expired after 300 ms: the robot is still moving (arm joint 3",
          0) == 0;
      })) << "the hold's own expiry, not the task deadline, ends the recovery";
}

// A robot that reports a fault is refused for the fault at once, not held for motion.
TEST_F(RestockCoordinatorDriverTest, AFaultedRobotIsRefusedForItsFaultWithoutAHold)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto faulted = reserved_world_snapshot(*reserve_response_);
  faulted.revision = ++observation_revision_;
  faulted.robot.fault_state = FaultState::Recoverable;
  faulted.robot.joint_velocities = {0.0, 0.0, 0.9, 0.0, 0.0, 0.0};
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(faulted));
  for (int pump = 0; pump < 8 && !driver_->snapshot().inhibited; ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(port_.snapshots.empty()) << "no hold for a faulted robot";
  const auto outputs = driver_->take_outputs();
  EXPECT_FALSE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.find("stop-settle hold") != std::string::npos;
      }));
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.find("the robot reports a fault") != std::string::npos;
      }));
}

// Milestone 10 §6 (Card 060): Card 010's SC-004 slot 13 latched UNSAFE "still moving" from
// robot telemetry world state had frozen two minutes earlier. Stale telemetry establishes
// neither motion nor rest: a snapshot at rest whose telemetry is older than the robot age bound
// must never let recovery replan, however long it is offered.
TEST_F(RestockCoordinatorDriverTest, StaleTelemetryAtRestNeverEstablishesARecoveryStop)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);

  // The fixture's ROS clock reads 1.1 s; this telemetry is 1.0 s old against a 500 ms bound.
  for (int step = 0; step < 64 && !driver_->snapshot().inhibited; ++step) {
    if (!port_.snapshots.empty()) {
      auto frozen = reserved_world_snapshot(*reserve_response_);
      frozen.revision = ++observation_revision_;
      frozen.robot.telemetry_time = rclcpp::Time(std::int64_t{100'000'000}, RCL_ROS_TIME);
      frozen.robot.joint_velocities = {};
      port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
        port_.snapshots, snapshot_response(frozen));
    }
    now_ += 25ms;
    driver_->pump(now_);
  }
  EXPECT_EQ(motion_.submit_attempts, 1U) << "recovery must not replan from stale telemetry";
  EXPECT_TRUE(driver_->snapshot().inhibited) << "an unestablished stop stays UNSAFE";
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.find("robot telemetry is") != std::string::npos &&
               output.detail.find("old") != std::string::npos;
      })) << "the refusal names the telemetry age, not motion";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind(
          "recovery stop-settle hold expired after 300 ms: robot "
          "telemetry is", 0) == 0;
      })) << "the hold's own expiry, not the task deadline, ends the recovery";
}

// Telemetry that goes stale (an adapter hiccup) and comes back fresh inside the hold is a stop
// once the fresh sample shows rest.
TEST_F(RestockCoordinatorDriverTest, RecoveryResumesWhenStaleTelemetryTurnsFreshInsideTheHold)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);

  auto frozen = reserved_world_snapshot(*reserve_response_);
  frozen.revision = ++observation_revision_;
  frozen.robot.telemetry_time = rclcpp::Time(std::int64_t{100'000'000}, RCL_ROS_TIME);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(frozen));
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited);

  for (int step = 0; step < 16 && port_.snapshots.empty(); ++step) {
    now_ += 25ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.snapshots.size(), 1U);
  auto fresh = reserved_world_snapshot(*reserve_response_);
  fresh.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(fresh));
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind(
          "recovery stop not yet established; bounded stop-settle hold started (300 ms): robot "
          "telemetry is", 0) == 0;
      }));
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery stop established after ", 0) == 0;
      }));
}

// A cancel during the hold ends it: no further re-observation and no replan.
TEST_F(RestockCoordinatorDriverTest, CancelDuringTheStopSettleHoldEndsIt)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto settling = reserved_world_snapshot(*reserve_response_);
  settling.revision = ++observation_revision_;
  settling.robot.joint_velocities = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0};
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(settling));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(port_.snapshots.empty());

  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  for (int step = 0; step < 16; ++step) {
    now_ += 25ms;
    driver_->pump(now_);
  }
  EXPECT_TRUE(port_.snapshots.empty()) << "a canceled hold never re-observes";
  EXPECT_EQ(motion_.submit_attempts, 1U);
  const auto outputs = driver_->take_outputs();
  EXPECT_FALSE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery stop established after ", 0) == 0 ||
               output.detail.rfind("recovery stop-settle hold expired", 0) == 0;
      }));
}

// An arm still decelerating when recovery first looks is not a moving arm once it has settled:
// the bounded stop-settle hold re-observes and recovery resumes from the first fresh snapshot at
// rest.
TEST_F(RestockCoordinatorDriverTest, RecoveryResumesOnceTheArmSettlesInsideTheHold)
{
  reach_pregrasp_with_motion();
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);

  auto settling = reserved_world_snapshot(*reserve_response_);
  settling.revision = ++observation_revision_;
  settling.robot.joint_velocities = {0.0, 0.0, 0.2, 0.0, 0.0, 0.0};
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(settling));
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().inhibited) << "the first moving sample starts a hold";

  for (int step = 0; step < 16 && port_.snapshots.empty(); ++step) {
    now_ += 25ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.snapshots.size(), 1U) << "the hold re-requests a fresh snapshot";
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(stopped));
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_FALSE(driver_->snapshot().inhibited);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U) << "recovery replans once the stop is established";
  const auto outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery stop established after ", 0) == 0;
      }));
}

// The recovery budget belongs to the task machine; exhausting it never retries unboundedly.
// Under Milestone 10 §6 (Card 051) the exhaustion of a RECOVERABLE goal (verified stop, nothing
// held, nothing moved as the machine knows it) ends in the single-shot typed skip: the
// reservation releases and the campaign continues. The operator latch for exhaustion is pinned
// by ARecoverableSkipCannotLoopAndStillLatchesForTheOperator (skip cleanup cannot succeed) and
// by RecoveryIsRefusedWhileTheRobotIsStillMoving (UNSAFE skips the ladder entirely).
TEST_F(RestockCoordinatorDriverTest, ExhaustedRecoveryAttemptsEndInTheTypedSkipWhenRecoverable)
{
  reach_pregrasp_with_motion();

  const auto abort_and_recover = [this]() {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(
      MotionOutcome::kExecutionFailed,
      "controller aborted on a path tolerance violation", true);
    now_ += 1ms;
    driver_->pump(now_);
    if (port_.snapshots.empty()) {
      return;
    }
    ASSERT_TRUE(reserve_response_);
    auto stopped = reserved_world_snapshot(*reserve_response_);
    stopped.revision = ++observation_revision_;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(stopped));
    driver_->pump(now_);
  };

  // Two recoveries are the configured budget; the third abort has nothing left.
  abort_and_recover();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  abort_and_recover();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  abort_and_recover();

  // Card 051: the exhaustion is classified RECOVERABLE (terminal stop, nothing held), so the
  // single-shot rung-5 skip routes to the reservation release — not a latch.
  for (int pump = 0; pump < 24 && port_.releases.empty(); ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.releases.size(), 1U)
    << "the skip's reservation release must be submitted; state=" <<
    (driver_->snapshot().transition ?
  std::string(to_string(driver_->snapshot().transition->state)) + " detail=" +
  driver_->snapshot().transition->detail : std::string("none"));
  {
    const auto terminal_task_phase =
      static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase);
    const auto terminal_fault_state =
      static_cast<FaultState>(port_.releases.front().request->terminal_fault_state);
    auto release =
      std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    release->status = restocker_world_state::operation_status_ok();
    release->world_revision =
      std::max(observation_revision_, reserve_response_->world_revision) + 1U;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
      port_.releases, release);
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U) << "post-release readback snapshot";
    auto released = world_snapshot(release->world_revision);
    released.robot.revision = released.revision;
    released.robot.task_phase = terminal_task_phase;
    released.robot.fault_state = terminal_fault_state;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(released));
    driver_->pump(now_);
  }
  const auto outputs = driver_->take_outputs();
  EXPECT_FALSE(driver_->snapshot().inhibited)
    << "a recoverable exhaustion must not latch the coordinator";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.outcome == RestockActionOutcome::kRecoverableSkip &&
               (output.kind == CoordinatorDriverOutputKind::kAborted ||
               output.kind == CoordinatorDriverOutputKind::kCanceled);
      })) << "the exhausted goal must deliver the typed recoverable skip";
}

// Common path: when the first candidate plans, that pose is submitted and no second candidate is
// mentioned.
TEST_F(RestockCoordinatorDriverTest, PlansTheFirstGraspCandidateWhenItCanBePlanned)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_EQ(staged_batch_->candidates.size(), 2U);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(
    motion_.submission().goal.planning_frame_from_tool0.matrix().isApprox(
      staged_batch_->candidates.front().poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(motion_.submit_attempts, 1U);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForApproach);
}

// Previously the goal failed when the first candidate had no inverse-kinematics solution; the retry
// now goes to the next candidate.
TEST_F(RestockCoordinatorDriverTest, FallsThroughToTheNextGraspCandidateOnAPreGraspPlanningFailure)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_EQ(staged_batch_->candidates.size(), 2U);
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();

  motion_.complete_planner_refusal("wrist_3 stands inside the left neighbour");
  now_ += 1ms;
  driver_->pump(now_);

  // Planning the pre-grasp for the second candidate's pose.
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(
    motion_.submission().goal.planning_frame_from_tool0.matrix().isApprox(
      staged_batch_->candidates[1].poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));
  EXPECT_FALSE(
    motion_.submission().goal.planning_frame_from_tool0.matrix().isApprox(
      staged_batch_->candidates.front().poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));

  // The reason the first candidate was refused is reported, and the rung-4 receipt names the
  // alternative-candidate step (Milestone 10 §6, Card 051).
  const auto outputs = driver_->take_outputs();
  const auto reports_the_fallthrough = [](const auto & output) {
    const auto & detail = output.detail;
    return detail.find("wrist_3 stands inside the left neighbour") != std::string::npos &&
           detail.find("falling through to grasp candidate 2 of 2") != std::string::npos;
  };
  EXPECT_TRUE(std::ranges::any_of(outputs, reports_the_fallthrough));
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery rung 4 (alternative candidate): ", 0) == 0;
      })) << "the candidate fallthrough must be receipted as rung 4";
}

// A failure must be distinguishable from an absence: with nothing graspable, the goal says what
// refused each candidate.
TEST_F(RestockCoordinatorDriverTest, ReportsWhatRefusedEveryGraspCandidateWhenNoneCanBePlanned)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_EQ(staged_batch_->candidates.size(), 2U);
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();

  motion_.complete_planner_refusal("wrist_3 stands inside the left neighbour");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete_planner_refusal("no inverse-kinematics solution at all");
  now_ += 1ms;
  driver_->pump(now_);

  // Nothing is left to plan, so the attempt ends.
  EXPECT_EQ(motion_.submit_attempts, 2U);
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);

  const auto outputs = driver_->take_outputs();
  const auto names_every_refusal = [](const auto & output) {
    const auto & detail = output.detail;
    return detail.find("every grasp candidate was refused") != std::string::npos &&
           detail.find("wrist_3 stands inside the left neighbour") != std::string::npos &&
           detail.find("no inverse-kinematics solution at all") != std::string::npos;
  };
  EXPECT_TRUE(std::ranges::any_of(outputs, names_every_refusal));
  // Milestone 10 §6 rung 4 (Card 051): the exhausted batch is receipted under the rung's name —
  // this is the budget whose exhaustion hands the goal to rung 5 or the operator.
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind(
          "recovery rung 4 (alternative candidate): the candidate batch is exhausted", 0) == 0;
      })) << "the exhausted batch must be receipted as rung 4";
}

// A degraded projector makes the motion port refuse the segment before the planner sees the pose.
// That is a planning failure (nothing was commanded), not a verdict on the grasp: it must not burn
// all candidates and report "no grasp candidate remains".
TEST_F(RestockCoordinatorDriverTest, KeepsTheGraspCandidateWhenThePlanNeverReachedThePlanner)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(
    MotionOutcome::kPlanningFailed,
    "planning-scene authority could not be established before planning");
  now_ += 1ms;
  driver_->pump(now_);

  // The bounded retry re-asks the same question, which is right when the scene could not be
  // certified.
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(
    motion_.submission().goal.planning_frame_from_tool0.matrix().isApprox(
      staged_batch_->candidates.front().poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));
}

// The collision-checked approach stopped at 2.7% on every retry with no approach motion executed,
// so retain neither the failed yaw nor its pre-grasp configuration: recovery free-space plans the
// next candidate first.
TEST_F(RestockCoordinatorDriverTest, FallsThroughAndRepositionsWhenAnApproachCannotBePlanned)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanApproach);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete_planner_refusal("the straight line is blocked");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(stopped));
  now_ += 1ms;
  driver_->pump(now_);

  // The next submission is a free-space pre-grasp for candidate two, not a linear approach from
  // candidate one's pose.
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_EQ(motion_.submission().goal.path, MotionPathKind::kFreeSpace);
  EXPECT_TRUE(
    motion_.submission().goal.planning_frame_from_tool0.matrix().isApprox(
      staged_batch_->candidates[1].poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));
}

TEST_F(RestockCoordinatorDriverTest, RefusedMotionSubmissionFailsWithoutClaimingMotion)
{
  reach_pregrasp_with_motion();
  ASSERT_EQ(motion_.submit_attempts, 1U);

  // Fail the first attempt, then refuse the retry as an absent MoveIt would. A refused submission
  // never reached the planner, so no motion is claimed.
  motion_.refuse_next(MotionSubmitStatus::kUnavailable, "MoveIt is not running");
  motion_.complete(MotionOutcome::kPlanningFailed, "no collision-free plan");
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(motion_.submit_attempts, 2U);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().pending_operation);
}

TEST_F(RestockCoordinatorDriverTest, ClosesTheGripperOnTheCandidateHoldWidth)
{
  reach_close_gripper();
  driver_->pump(now_);

  // One open for the approach, then this close.
  ASSERT_EQ(gripper_.submit_attempts, 2U);
  ASSERT_TRUE(gripper_.outstanding());
  // Commanded to the product-specific hold width, not a generic closed pose.
  EXPECT_GT(gripper_.submission().goal.target_position_m, 0.0);
  EXPECT_TRUE(valid_gripper_goal(gripper_.submission().goal));
}

TEST_F(RestockCoordinatorDriverTest, GripperMissingTheAttachmentBandTerminates)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());

  // The controller reported success but the jaws are outside the plugin's band; closing again is
  // not provably safe.
  gripper_.complete(GripperOutcome::kPositionNotVerified, "fingers outside the attachment band");
  now_ += 1ms;
  driver_->pump(now_);

  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kCloseGripper);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kAttachTransaction);
}

// The Gazebo attachment boundary observes grasp_center -> product body and refuses the attach on
// disagreement with the authorized transform. Passing grasp_center -> tool0 fails every attach.
TEST_F(RestockCoordinatorDriverTest, AuthorizesTheAttachmentAgainstTheProductNotTheToolFrame)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto held = reserved_world_snapshot(*reserve_response_);
  const auto held_id = selected_pair(held).object_id;
  // Verify-grasp runs before the attach: product free, robot holding nothing.
  // Place the product away from the tool frame so the two candidate transforms differ.
  held.objects.at(held_id).pose_in_world.translation() += Eigen::Vector3d(0.0, 0.0, 0.02);
  ASSERT_TRUE(held.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(held));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());

  const auto staged = generated_grasps(
    world_snapshot(), selected_pair(world_snapshot()),
    grasp_authority(world_snapshot()),
    rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME));
  ASSERT_TRUE(staged) << staged.error().detail;
  const auto & poses = staged.value().candidates.front().poses;
  const Eigen::Isometry3d expected =
    poses.world_from_grasp_center.inverse() * held.objects.at(held_id).pose_in_world;
  const Eigen::Isometry3d tool_frame_transform =
    poses.world_from_grasp_center.inverse() * poses.world_from_grasp_tool0;
  ASSERT_FALSE(expected.matrix().isApprox(tool_frame_transform.matrix(), 1.0e-9));

  const auto & authorized = attachment_.submission().goal.grasp_center_to_child;
  EXPECT_TRUE(authorized.matrix().isApprox(expected.matrix(), 1.0e-12));
  EXPECT_FALSE(authorized.matrix().isApprox(tool_frame_transform.matrix(), 1.0e-9));
}

// Regression: grasp verification required the robot to already report holding the product, which
// the attachment commit writes later, so no task reached attachment. Before the commit, the
// provable fact is that the attachment is still admissible.
TEST_F(RestockCoordinatorDriverTest, VerifiesTheGraspAgainstEvidenceThePreAttachWorldCanShow)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto already_attached = reserved_world_snapshot(*reserve_response_);
  const auto object_id = selected_pair(already_attached).object_id;
  already_attached.robot.held_object = object_id;
  already_attached.objects.at(object_id).grasp_state = restocker_world_state::GraspState::Attached;
  ASSERT_TRUE(already_attached.active_reservation);
  already_attached.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(already_attached));
  now_ += 1ms;
  driver_->pump(now_);

  // A world already reporting the product attached contradicts an uncommitted grasp, so no
  // transaction starts.
  EXPECT_FALSE(attachment_.outstanding());
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kAttachTransaction);
}

// Card 039 instrumentation: the coupling's receipt must say which verification of the goal owns
// it and under which recovery attempt, because Terminal A's interleaving could not be rebuilt from
// operation-accept lines and the final inhibit alone.
TEST_F(RestockCoordinatorDriverTest, ReportsWhichVerificationOwnedTheGraspCoupling)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  // Verify-grasp reads the world before the attachment commits: product free, robot holding
  // nothing, so the coupling is retained against this snapshot.
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto held = reserved_world_snapshot(*reserve_response_);
  ASSERT_TRUE(held.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(held));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());

  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_NE(
    receipts.find(
      "grasp coupling retained at verify_grasp: verify #1 of this goal, recovery attempt 0"),
    std::string::npos) << receipts;
  EXPECT_NE(receipts.find(", world revision "), std::string::npos) << receipts;
  EXPECT_NE(receipts.find(", object revision "), std::string::npos) << receipts;
  EXPECT_NE(receipts.find("(first capture)"), std::string::npos) << receipts;
}

TEST_F(RestockCoordinatorDriverTest, UncommittedAttachmentStopsForAnOperator)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  // Verify-grasp reads the world before the attachment commits: product free, robot holding
  // nothing.
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto held = reserved_world_snapshot(*reserve_response_);
  ASSERT_TRUE(held.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(held));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());

  // Gazebo coupled the product but world state never recorded it: the disagreement must not be
  // papered over.
  attachment_.complete(AttachmentOutcome::kUncommitted, "commit refused after physical attach");
  now_ += 1ms;
  driver_->pump(now_);

  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_NE(snapshot.transition->state, RestockTaskState::kPlanRetract);
}

// Card 086 review S4: the 097 physical-applied/commit-refused outcome reaches the campaign through
// the coordinator's real terminal status. Whatever status it lands on must be one the campaign's
// settlement rules leave unresolved (a settling status here would fail open).
TEST_F(RestockCoordinatorDriverTest, UncommittedAttachmentTerminalIsNotASettlingCampaignStatus)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto held = reserved_world_snapshot(*reserve_response_);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(held));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());
  (void)driver_->take_outputs();

  attachment_.complete(AttachmentOutcome::kUncommitted, "commit refused after physical attach");
  std::vector<CoordinatorDriverOutput> outputs;
  for (int step = 0; step < 6; ++step) {
    now_ += 1ms;
    driver_->pump(now_);
    for (auto & output : driver_->take_outputs()) {
      outputs.push_back(std::move(output));
    }
  }

  std::string seen;
  bool terminal_seen = false;
  for (const auto & output : outputs) {
    seen += std::to_string(static_cast<int>(output.kind)) + ":" +
      std::to_string(static_cast<int>(output.outcome)) + " ";
    if (output.kind != CoordinatorDriverOutputKind::kAborted) {
      continue;
    }
    terminal_seen = true;
    restocker_interfaces::action::RestockProduct::Result result;
    result.status = action_status(output.outcome);
    EXPECT_FALSE(
      settles_attempt(
        evidence_of(rclcpp_action::ResultCode::ABORTED, result)))
      << "terminal status " << result.status << " would settle a physically-applied, "
      << "uncommitted attachment";
  }
  EXPECT_TRUE(terminal_seen) << "no aborted terminal output; saw " << seen;
  // Stage 1b: the attach command is sticky evidence that something was commanded, and no stop
  // is claimed after it, so the result proves neither.
  for (const auto & output : outputs) {
    EXPECT_FALSE(output.metrics.motion_definitely_not_started);
    EXPECT_FALSE(output.metrics.execution_reached_terminal_stop);
  }
  GTEST_LOG_(INFO) << "uncommitted attachment terminal outputs (kind:outcome): " << seen;
}

TEST_F(RestockCoordinatorDriverTest, RealAttachmentSagaInhibitedReplayCannotResumeDriverMotion)
{
  reach_close_gripper();
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(reserved_world_snapshot(*reserve_response_)));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());

  // Run the exact goal/correlation booked by the driver through the production worker/saga.
  // Only ROS I/O is scripted; the inhibited completion is not fabricated by FakeAttachmentPort.
  attachment_test::Harness io;
  io.steps = {{attachment_test::Behavior::kReply,
    attachment_test::Status::CLOCK_AUTHORITY_INHIBITED, true}};
  RosAttachmentPort real_port(io.config, io.operations());
  const auto motion_submissions = motion_.submit_attempts;
  const auto & pending = attachment_.submission();
  auto completion = io.run(real_port, pending.goal, pending.correlation);
  ASSERT_EQ(completion.outcome, AttachmentOutcome::kIndeterminate);
  EXPECT_FALSE(completion.planning_scene_lease_released);
  pending.callback(std::move(completion));
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(admission_.snapshot().inhibited);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_EQ(attachment_.submit_attempts, 1U);
  EXPECT_EQ(motion_.submit_attempts, motion_submissions);
  EXPECT_EQ(io.released, 0);
  EXPECT_EQ(io.physical_sends, 1);
  const auto record = RosAttachmentPortTestPeer::retained(real_port);
  ASSERT_TRUE(record);
  ASSERT_TRUE(record->historical_receipt);
  EXPECT_EQ(record->historical_receipt->world_revision, 73U);
  EXPECT_TRUE(record->unsafe_terminal);
}

TEST_F(RestockCoordinatorDriverTest, FullyRolledBackAttachmentRetriesFromFreshStoppedEvidence)
{
  reach_close_gripper();
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto unchanged = reserved_world_snapshot(*reserve_response_);
  unchanged.revision = ++observation_revision_;
  ASSERT_TRUE(unchanged.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(unchanged));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());

  // A rejection means neither the physical boundary nor world state changed. Recovery may
  // regenerate the grasp set after a new snapshot confirms the robot is stopped and holds nothing.
  attachment_.complete(AttachmentOutcome::kRejected, "physical grasp tolerance was not met");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);

  unchanged.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(unchanged));
  now_ += 1ms;
  driver_->pump(now_);

  const auto recovered = driver_->snapshot();
  const auto outputs = driver_->take_outputs();
  const auto inhibition = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kInhibited;
    });
  const std::string inhibition_detail =
    inhibition == outputs.end() ? std::string{} : inhibition->detail;
  ASSERT_TRUE(recovered.transition);
  EXPECT_FALSE(recovered.inhibited) << inhibition_detail;
  // Milestone 10 §6 (Card 062): the jaws closed on a product that is not held, so they open to
  // the closed candidate's approach clearance before the pre-grasp retry leaves the grasp.
  EXPECT_EQ(recovered.transition->state, RestockTaskState::kOpenGripperForEscape) <<
    recovered.transition->detail;
  EXPECT_FALSE(motion_.outstanding());
  ASSERT_TRUE(gripper_.outstanding());
  ASSERT_TRUE(staged_batch_);
  const auto closed = staged_batch_->candidates.front();
  EXPECT_DOUBLE_EQ(gripper_.submission().goal.target_position_m, closed.open_joint_position_m);
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened for the escape");
  now_ += 1ms;
  driver_->pump(now_);

  // The retry plans the NEXT candidate but first leaves the grasp of the one that closed, along
  // its own reversed approach, tolerating finger contact with this product only.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  const auto & goal = motion_.submission().goal;
  ASSERT_TRUE(goal.grasp_escape);
  EXPECT_FALSE(goal.grasp_escape_leg);
  EXPECT_TRUE(
    goal.grasp_escape->planning_frame_from_standoff_tool0.matrix().isApprox(
      closed.poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));
  EXPECT_EQ(goal.grasp_escape->target_object_id, "restocker/object/17");
  EXPECT_EQ(
    goal.grasp_escape->tolerated_links,
    (std::vector<std::string>{"left_finger", "right_finger"}));
  EXPECT_TRUE(valid_motion_goal(goal));

  // A refused escape moved nothing, so the retry still owes it.
  motion_.complete(MotionOutcome::kPlanningFailed, "grasp escape refused");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_TRUE(motion_.submission().goal.grasp_escape);

  // Card 062 dev run 1: the escape executed, then the planner refused this candidate's
  // pre-grasp. That verdict still spends the candidate (rung 4) instead of a recovery attempt,
  // and the executed escape is never commanded again.
  (void)driver_->take_outputs();
  motion_.complete_after_grasp_escape(
    MotionOutcome::kPlanningFailed, "pre-grasp endpoints rejected by the scene", true);
  now_ += 1ms;
  driver_->pump(now_);
  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_NE(receipts.find("recovery rung 4 (alternative candidate)"), std::string::npos) <<
    receipts;
  EXPECT_NE(
    receipts.find("grasp escape: the arm left the grasp of restocker/object/17"),
    std::string::npos) << receipts;
  EXPECT_FALSE(driver_->snapshot().inhibited) << receipts;
}

// Card 039 SC-001: a goal that verified the grasp once, rolled its attach back and recovered runs
// the grasp sequence a second time and reaches kVerifyGrasp again. Before the Milestone 10 §1
// contract that second verification refused with kAlreadySet, the driver turned it into
// fail_operation(kRetryableFailure), and the recovery budget it spent ended the goal in a terminal
// inhibition ("motion inhibited; operator required"). The second verification must re-derive the
// coupling and the task must advance.
TEST_F(RestockCoordinatorDriverTest, SecondVerificationAfterARolledBackAttachDoesNotTerminalInhibit)
{
  reach_close_gripper();
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  // The first verification adopts a fresh snapshot and retains the coupling from it.
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto observed = reserved_world_snapshot(*reserve_response_);
  observed.revision = ++observation_revision_;
  ASSERT_TRUE(observed.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(observed));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kAttachTransaction);

  // Neither the physical boundary nor world state changed, so the attach rolls back and bounded
  // recovery observes a stopped robot before the sequence runs again.
  attachment_.complete(AttachmentOutcome::kRejected, "physical grasp tolerance was not met");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  observed.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(observed));
  now_ += 1ms;
  driver_->pump(now_);
  // Card 062: the jaws open before the grasp is left.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForEscape);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened for the escape");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_TRUE(motion_.submission().goal.grasp_escape);
  (void)driver_->take_outputs();

  // Pre-grasp, approach and close run a second time inside this one goal.
  motion_.complete_after_grasp_escape(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "approach reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  // The second verification must re-derive rather than refuse: no inhibition, and the task moves
  // on to the attachment transaction with a receipt naming this verification as the owner.
  ASSERT_EQ(port_.snapshots.size(), 1U);
  observed.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(observed));
  now_ += 1ms;
  driver_->pump(now_);

  std::string receipts;
  bool saw_inhibition = false;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
    saw_inhibition = saw_inhibition ||
      output.kind == CoordinatorDriverOutputKind::kInhibited;
  }
  const auto continued = driver_->snapshot();
  ASSERT_TRUE(continued.transition);
  EXPECT_FALSE(saw_inhibition) << receipts;
  EXPECT_FALSE(continued.inhibited) << receipts;
  EXPECT_EQ(continued.transition->state, RestockTaskState::kAttachTransaction) << receipts;
  EXPECT_TRUE(attachment_.outstanding()) << receipts;
  EXPECT_NE(
    receipts.find(
      "grasp coupling retained at verify_grasp: verify #2 of this goal, recovery attempt 1"),
    std::string::npos) << receipts;
  EXPECT_NE(
    receipts.find("(re-derived; previous world revision "), std::string::npos) << receipts;
  // Card 062: the executed escape was receipted and is never commanded again.
  EXPECT_NE(
    receipts.find("grasp escape: the arm left the grasp of restocker/object/17"),
    std::string::npos) << receipts;
}

// Regression: segments that slide the jaws (or the held product) along an axis must be straight
// lines, or a sampling planner may sweep a finger through the product. Free-space repositioning
// segments must not be.
TEST(RestockCoordinatorMotionSegments, TravelStraightOnlyWhereTheJawsSlidePastSomething)
{
  EXPECT_TRUE(linear_motion_segment(MotionSegment::kApproach));
  EXPECT_TRUE(linear_motion_segment(MotionSegment::kRetract));
  EXPECT_TRUE(linear_motion_segment(MotionSegment::kCarryStart));
  EXPECT_TRUE(linear_motion_segment(MotionSegment::kInsert));
  EXPECT_TRUE(linear_motion_segment(MotionSegment::kRetreat));
  EXPECT_FALSE(linear_motion_segment(MotionSegment::kPreGrasp));
  EXPECT_FALSE(linear_motion_segment(MotionSegment::kPreInsert));
}

TEST_F(RestockCoordinatorDriverTest, PreInsertSubmitsSurveyDerivedPathConstraintsAndNeverDropsThem)
{
  reach_preinsert_with_motion();
  ASSERT_TRUE(staged_batch_);
  const auto & grasp = staged_batch_->candidates.front();
  const auto & goal = motion_.submission().goal;
  EXPECT_EQ(goal.path, MotionPathKind::kFreeSpace);
  ASSERT_TRUE(goal.path_constraints.has_value());
  EXPECT_TRUE(valid_motion_goal(goal));
  ASSERT_TRUE(goal.path_constraints->position_box.has_value());
  ASSERT_TRUE(goal.path_constraints->orientation_hold.has_value());
  ASSERT_TRUE(goal.maximum_tool0_x_axis_tilt_rad.has_value());
  ASSERT_TRUE(goal.required_linear_continuation_pose);
  ASSERT_TRUE(goal.required_linear_continuation_gripper_joint_position_m);
  EXPECT_GT(
    (goal.required_linear_continuation_pose->translation() -
    goal.planning_frame_from_tool0.translation()).norm(),
    1.0e-6);
  EXPECT_DOUBLE_EQ(
    *goal.required_linear_continuation_gripper_joint_position_m,
    grasp.hold_joint_position_m);
  EXPECT_DOUBLE_EQ(
    *goal.maximum_tool0_x_axis_tilt_rad,
    kHeldProductMaximumUprightTiltRad);
  EXPECT_NE(goal.path_constraints->description.find("z_max="), std::string::npos);

  const CylinderEnvelope held{0.033, 0.122};
  const auto carry_start = build_carry_start_tool0_pose(
    grasp.poses.world_from_retract_tool0,
    goal.planning_frame_from_tool0, held);
  ASSERT_TRUE(carry_start);
  const auto expected =
    build_preinsert_transfer_path_constraints(*carry_start, goal.planning_frame_from_tool0, held);
  ASSERT_TRUE(expected);
  ASSERT_TRUE(expected->position_box);
  EXPECT_TRUE(
    goal.path_constraints->position_box->min_corner_m.isApprox(
      expected->position_box->min_corner_m, 1.0e-12));
  EXPECT_TRUE(
    goal.path_constraints->position_box->max_corner_m.isApprox(
      expected->position_box->max_corner_m, 1.0e-12));

  // A constrained planning refusal must retry PreInsert still constrained, never falling back to
  // unconstrained free-space.
  const auto first_attempts = motion_.submit_attempts;
  motion_.complete_planner_refusal("no plan inside the transfer box");
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreInsert);
  EXPECT_EQ(motion_.submit_attempts, first_attempts + 1U);
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_TRUE(motion_.submission().goal.path_constraints.has_value());
  EXPECT_TRUE(valid_motion_goal(motion_.submission().goal));
}

TEST_F(RestockCoordinatorDriverTest, CarryStartSubmitsCartesianEgressBeforePreInsert)
{
  // Drive to PlanCarryStart and inspect the outstanding Cartesian goal: footprint-clearance Y
  // egress at the retract orientation. Completing it reaches free-space PreInsert under the
  // egress-to-mouth AABB corridor.
  reach_close_gripper();
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws at the hold width");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto held = reserved_world_snapshot(*reserve_response_);
  held.revision = ++observation_revision_;
  ASSERT_TRUE(held.active_reservation);
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(held));
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(attachment_.outstanding());
  attachment_.complete(AttachmentOutcome::kSucceeded, "boundary and world state agree");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "retract reached");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(port_.snapshots.size(), 1U);
  auto carrying = reserved_world_snapshot(*reserve_response_);
  carrying.revision = ++observation_revision_;
  const auto object_id = selected_pair(carrying).object_id;
  carrying.robot.held_object = object_id;
  carrying.objects.at(object_id).grasp_state = restocker_world_state::GraspState::Attached;
  ASSERT_TRUE(carrying.active_reservation);
  carrying.active_reservation->stage = restocker_world_state::ReservationStage::Attached;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(carrying));
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanCarryStart);
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_TRUE(staged_batch_);
  const auto & grasp = staged_batch_->candidates.front();
  const auto & goal = motion_.submission().goal;
  EXPECT_EQ(goal.path, MotionPathKind::kLinear);
  EXPECT_FALSE(goal.path_constraints.has_value());
  ASSERT_TRUE(goal.maximum_tool0_x_axis_tilt_rad.has_value());
  EXPECT_GT(
    goal.planning_frame_from_tool0.translation().y(),
    grasp.poses.world_from_retract_tool0.translation().y());
  EXPECT_DOUBLE_EQ(
    goal.planning_frame_from_tool0.translation().x(),
    grasp.poses.world_from_retract_tool0.translation().x());
  EXPECT_DOUBLE_EQ(
    goal.planning_frame_from_tool0.translation().z(),
    grasp.poses.world_from_retract_tool0.translation().z());
  const Eigen::Quaterniond retract_quaternion(grasp.poses.world_from_retract_tool0.linear());
  const Eigen::Quaterniond carry_quaternion(goal.planning_frame_from_tool0.linear());
  EXPECT_NEAR(
    retract_quaternion.angularDistance(carry_quaternion),
    0.0, 1.0e-12);
  const double min_y_span = 2.0 * 0.033 + kSurveyedLaneSideClearanceM;
  EXPECT_NEAR(
    goal.planning_frame_from_tool0.translation().y(),
    grasp.poses.world_from_retract_tool0.translation().y() + min_y_span, 1.0e-9);

  motion_.complete(MotionOutcome::kSucceeded, "carry-start reached");
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreInsert);
  ASSERT_TRUE(motion_.outstanding());
  ASSERT_TRUE(motion_.submission().goal.path_constraints.has_value());
}

// Card 044: the whole-task deadline lapses silently while the final retreat is in flight, and
// the visible family line used to be the identity refusal at the next execute-state entry
// (SC-003 run 7, Card 020 run 4, Card 009's loaded reds). One log must still order expiry →
// what happens next. Card 051 changed the verdict, not the receipts: the expiry is receipted
// first exactly as before, and under the Milestone 10 §6 contract the cleanup retreat identity
// is now ADMITTED (bounded) instead of refused — the refusal contract for every other
// execution under termination is pinned by ExecutionIdentityRefusalNamesTheLatchedTermination
// and UserCancelStillRefusesTheRetreatIdentity.
TEST_F(
  RestockCoordinatorDriverTest, WholeTaskDeadlineExpiryIsReceiptedAndTheCleanupRetreatIsAdmitted)
{
  reach_close_gripper();
  driver_->pump(now_);
  reach_retreat_with_motion();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat);

  // The budget expires while the retreat motion is still in flight: nothing refuses anything yet
  // (the planned segment owns the ledger), the expiry itself must be receipted on both clocks.
  now_ += 2000ms;
  driver_->pump(now_);
  const auto expiry_outputs = driver_->take_outputs();
  const auto expiry = std::ranges::find_if(
    expiry_outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kFeedback &&
             output.detail.rfind("whole-task steady deadline exceeded: ", 0) == 0;
    });
  ASSERT_NE(expiry, expiry_outputs.end()) << "the expiry must be receipted on its own line";
  EXPECT_NE(expiry->detail.find("1000 ms budget"), std::string::npos) << expiry->detail;
  EXPECT_NE(expiry->detail.find("since goal acceptance, sim +"), std::string::npos)
    << expiry->detail;
  EXPECT_NE(expiry->detail.find("state=plan_retreat"), std::string::npos) << expiry->detail;
  EXPECT_NE(expiry->detail.find("recovery attempt 0 of 2"), std::string::npos) << expiry->detail;
  // The deadline is classified at its source (Card 051): one receipt names the class.
  const auto classified = std::ranges::find_if(
    expiry_outputs, [](const auto & output) {
      return output.detail.rfind("recovery classification: ", 0) == 0;
    });
  ASSERT_NE(classified, expiry_outputs.end()) << "the expiry must be classified";
  EXPECT_TRUE(
    classified->detail.find("whole-task steady deadline exceeded") != std::string::npos)
    << classified->detail;
  EXPECT_TRUE(admission_.snapshot().task_deadline_exceeded);

  // The retreat completes after the expiry: under §6's bounded rule the machine enters
  // execute-retreat and the identity is ADMITTED with a rung-1 receipt — the arm retreats
  // to the safe pose instead of latching at the guard.
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "retreat reached");
  now_ += 1ms;
  driver_->pump(now_);
  const auto after_outputs = driver_->take_outputs();
  const auto inhibited = std::ranges::find_if(
    after_outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kInhibited;
    });
  ASSERT_EQ(inhibited, after_outputs.end())
    << "the bounded cleanup retreat must not latch the operator: " <<
    (inhibited == after_outputs.end() ? "" : inhibited->detail);
  const auto admitted = std::ranges::find_if(
    after_outputs, [](const auto & output) {
      return output.detail.rfind(
        "recovery rung 1 (safe retreat): the execution identity was admitted", 0) == 0;
    });
  ASSERT_NE(admitted, after_outputs.end())
    << "the guard must receipt which exit it admitted";
  // Detached but uncommitted: the termination cleanup finishes the placement it physically made.
  EXPECT_EQ(
    driver_->snapshot().transition->state, RestockTaskState::kSurveyDestination);
  EXPECT_TRUE(admission_.snapshot().task_deadline_exceeded);
}

// Card 051: the deadline may also command the retreat from the recovery/plan boundary — the arm
// that already moved gets rung 1 before the reservation is released (before this contract the
// machine released in place and no retreat motion was ever commanded).
TEST_F(
  RestockCoordinatorDriverTest, WholeTaskDeadlineCommandsTheCleanupRetreatFromThePlanBoundary)
{
  CoordinatorRetreatTargetProvider survey_retreat = [](const SelectedTaskPair &) {
    auto pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d{-1.0, 0.25, 1.1};
    return std::optional<Eigen::Isometry3d>{pose};
  };
  reach_pregrasp_with_motion(std::move(survey_retreat));
  ASSERT_TRUE(motion_.outstanding());
  // The pre-grasp runs for real (trajectory accepted → the arm has moved), then the approach
  // plan fails twice: bounded op retries, then bounded recovery resumes at plan_pre_grasp.
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForApproach);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanApproach);
  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kPlanningFailed, "no approach solution in this scene");
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(stopped));
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding()) << "the recovered pre-grasp is re-submitted";
  ASSERT_TRUE(driver_->snapshot().transition->first_trajectory_may_have_started);

  // The whole-task deadline expires while the recovered pre-grasp is in flight.
  now_ += 2000ms;
  driver_->pump(now_);
  EXPECT_TRUE(admission_.snapshot().task_deadline_exceeded);

  // The in-flight pre-grasp completes; the arm already moved, so §6 routes the termination
  // through the cleanup retreat instead of releasing in place.
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  const auto inhibited = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kInhibited;
    });
  ASSERT_EQ(inhibited, outputs.end()) << "no latch on the recoverable deadline path";
  ASSERT_EQ(
    driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat);
  ASSERT_TRUE(motion_.outstanding())
    << "the cleanup retreat must actually be commanded; detail=" <<
    driver_->snapshot().transition->detail;
  EXPECT_EQ(motion_.submission().goal.path, MotionPathKind::kFreeSpace)
    << "the pre-detach cleanup retreat repositions across the cell: linear (the post-detach "
    "lane withdrawal default) can never satisfy it — live receipts stopped at 0 %";
  const auto rung = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.detail.rfind(
        "recovery rung 1 (safe retreat): the cleanup retreat is commandable", 0) == 0;
    });
  ASSERT_NE(rung, outputs.end()) << "the plan guard must receipt the admitted exit";
}

// Card 074 (Milestone 10 §6): the deadline's RECOVERABLE aftermath when the placement has
// already committed. Card 066's dense dev run 3 expired 0.285 s before the gripper confirmed
// the release, the lane observation then accepted the placement, and the terminal released the
// reservation with the transition's outcome — exact validation refused `placed=true`, the
// refusal latched an operator, and the typed skip §6 promises became unreachable (no log in the
// repository containing `whole-task steady deadline exceeded` has ever contained a rung-5
// receipt). A reservation observed placed must release as Succeeded and the goal must deliver
// that placement's success, receipted with the expiry, with no latch.
TEST_F(
  RestockCoordinatorDriverTest,
  DeadlineAfterACommittedPlacementDeliversSuccessWithoutLatching)
{
  reach_close_gripper();
  driver_->pump(now_);
  reach_retreat_with_motion();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat);

  // The whole-task budget expires while the cleanup retreat is in flight (Card 044's shape).
  now_ += 2000ms;
  driver_->pump(now_);
  EXPECT_TRUE(admission_.snapshot().task_deadline_exceeded);

  // The retreat completes and the termination cleanup finishes the placement it physically
  // made: every world observation from here carries the committed placement (stage Detached,
  // placed_in_destination), exactly what world state accepted in the dev run.
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "retreat reached");
  now_ += 1ms;
  driver_->pump(now_);

  // Drive whatever the termination cleanup still requests until the reservation release is
  // attempted (or something refuses first). The first destination observation is the survey
  // (evidence strictly after the physical release); every later one carries the committed
  // placement, as world state accepted in the dev run.
  bool destination_surveyed = false;
  for (int step = 0; step < 32 && port_.releases.empty() && !driver_->snapshot().inhibited;
    ++step)
  {
    if (!port_.snapshots.empty()) {
      if (!destination_surveyed) {
        destination_surveyed = true;
        satisfy_snapshot(false, false, true);
      } else {
        satisfy_snapshot(false, true);
      }
    } else if (attachment_.outstanding()) {
      satisfy_attachment();
    } else if (motion_.outstanding()) {
      satisfy_motion();
    } else if (gripper_.outstanding()) {
      satisfy_gripper();
    } else {
      break;
    }
  }

  if (driver_->snapshot().inhibited) {
    const auto seen = driver_->take_outputs();
    for (const auto & output : seen) {
      ADD_FAILURE() << "output kind=" << static_cast<int>(output.kind) << " detail="
                    << output.detail;
    }
  }
  EXPECT_FALSE(driver_->snapshot().inhibited)
    << "a placement world state already accepted must not latch the operator: " <<
    (driver_->snapshot().transition ? driver_->snapshot().transition->detail : "");
  ASSERT_EQ(port_.releases.size(), 1U)
    << "the release must be built for the committed placement; state=" <<
    (driver_->snapshot().transition ?
  std::string(to_string(driver_->snapshot().transition->state)) + " detail=" +
  driver_->snapshot().transition->detail : std::string("none"));
  EXPECT_EQ(
    port_.releases.front().request->outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_SUCCEEDED)
    << "the stage decides the outcome: a placed reservation releases as Succeeded (§6, 074)";
  EXPECT_EQ(
    static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase),
    TaskPhase::Idle);

  // Complete the release and its readback; the terminal is the placement's own success.
  {
    auto release =
      std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    release->status = restocker_world_state::operation_status_ok();
    release->world_revision =
      std::max(observation_revision_, reserve_response_->world_revision) + 1U;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
      port_.releases, release);
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U) << "post-release readback snapshot";
    auto released = world_snapshot(release->world_revision);
    released.robot.revision = released.revision;
    released.robot.task_phase = TaskPhase::Idle;
    released.robot.fault_state = FaultState::None;
    // The release went out as Succeeded, so the terminal snapshot must show the product in its
    // destination lane (the proof's membership check) — the placement the dev run committed.
    released.lanes.at(LaneId{"lane_01"}).contents.push_back(ObjectId{17U});
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(released));
    driver_->pump(now_);
  }
  const auto outputs = driver_->take_outputs();
  const auto terminal = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kSucceeded;
    });
  if (terminal == outputs.end()) {
    for (const auto & output : outputs) {
      ADD_FAILURE() << "output kind=" << static_cast<int>(output.kind) << " outcome="
                    << static_cast<int>(output.outcome) << " detail=" << output.detail;
    }
  }
  ASSERT_NE(terminal, outputs.end()) <<
    "the committed placement's terminal is success, not an operator latch or a blocked goal";
  EXPECT_EQ(terminal->outcome, RestockActionOutcome::kSucceeded);
  EXPECT_NE(
    terminal->detail.find("whole-task steady deadline exceeded"), std::string::npos)
    << "the terminal is receipted with the expiry that asked for it: " << terminal->detail;
  EXPECT_TRUE(
    std::ranges::none_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kInhibited;
      }));
}

// Card 074 (Milestone 10 §6) review note 3, bullet 3: the reconciliation changed how the release
// request is BUILT, never what a world-state refusal means. The same committed placement, the
// same Succeeded request — and world state still refuses it, so the driver must inhibit exactly
// as it does on every other refused release: no success terminal, no silent continuation.
TEST_F(
  RestockCoordinatorDriverTest,
  RefusedReleaseAfterACommittedPlacementStillInhibitsForTheOperator)
{
  reach_close_gripper();
  driver_->pump(now_);
  reach_retreat_with_motion();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat);

  // The whole-task budget expires while the cleanup retreat is in flight (Card 044's shape).
  now_ += 2000ms;
  driver_->pump(now_);
  EXPECT_TRUE(admission_.snapshot().task_deadline_exceeded);

  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "retreat reached");
  now_ += 1ms;
  driver_->pump(now_);

  bool destination_surveyed = false;
  for (int step = 0; step < 32 && port_.releases.empty() && !driver_->snapshot().inhibited;
    ++step)
  {
    if (!port_.snapshots.empty()) {
      if (!destination_surveyed) {
        destination_surveyed = true;
        satisfy_snapshot(false, false, true);
      } else {
        satisfy_snapshot(false, true);
      }
    } else if (attachment_.outstanding()) {
      satisfy_attachment();
    } else if (motion_.outstanding()) {
      satisfy_motion();
    } else if (gripper_.outstanding()) {
      satisfy_gripper();
    } else {
      break;
    }
  }
  ASSERT_EQ(port_.releases.size(), 1U)
    << "the release must be built for the committed placement; state=" <<
    (driver_->snapshot().transition ?
  std::string(to_string(driver_->snapshot().transition->state)) + " detail=" +
  driver_->snapshot().transition->detail : std::string("none"));
  ASSERT_FALSE(driver_->snapshot().inhibited)
    << "the request must still be the reconciled Succeeded one before world state answers";
  EXPECT_EQ(
    port_.releases.front().request->outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_SUCCEEDED)
    << "Card 074's reconciliation: a placed reservation releases as Succeeded";

  // World state refuses anyway — the product's placement no longer stands by its own gates
  // (a non-OK status is exactly the refusal the port contract reports as kRemoteRejected).
  {
    auto release =
      std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    release->status.code =
      restocker_interfaces::msg::WorldStateOperationStatus::PREDICATE_FAILED;
    release->status.detail = "object is not Free in the destination lane";
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
      port_.releases, release);
    driver_->pump(now_);
  }

  const auto outputs = driver_->take_outputs();
  const auto inhibited = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kInhibited &&
             output.detail.find("release response violates exact lineage") != std::string::npos;
    });
  if (inhibited == outputs.end()) {
    for (const auto & output : outputs) {
      ADD_FAILURE() << "output kind=" << static_cast<int>(output.kind) << " outcome="
                    << static_cast<int>(output.outcome) << " detail=" << output.detail;
    }
  }
  ASSERT_NE(inhibited, outputs.end())
    << "a release world state still refuses must inhibit, exactly as before the "
    "reconciliation (§6 bullet 3)";
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_TRUE(
    std::ranges::none_of(
      outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kSucceeded;
      })) << "a refused release may never deliver the placement's success";
}

// Card 051: the deadline retreat's own budget is fail-closed — a retreat still in flight when
// `task.deadline_retreat_timeout_ms` expires is cancelled and latches UNSAFE, exactly as the
// spec requires (verified stop or operator).
TEST_F(RestockCoordinatorDriverTest, DeadlineRetreatBudgetExpiryLatchesForTheOperator)
{
  reach_close_gripper();
  driver_->pump(now_);
  reach_retreat_with_motion();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat);

  now_ += 2000ms;
  driver_->pump(now_);
  ASSERT_TRUE(admission_.snapshot().task_deadline_exceeded);
  ASSERT_TRUE(motion_.outstanding());

  // Past the fixture's 500 ms retreat budget while the retreat is still in flight.
  now_ += 1000ms;
  driver_->pump(now_);
  const auto outputs = driver_->take_outputs();
  const auto inhibited = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.kind == CoordinatorDriverOutputKind::kInhibited &&
             output.detail.find("deadline retreat budget expired") != std::string::npos;
    });
  ASSERT_NE(inhibited, outputs.end())
    << "the retreat budget must fail closed; outputs follow";
}

// Milestone 10 §6 (Card 051): the whole-task deadline whose cleanup never moved the arm and
// never touched the world delivers the TYPED recoverable skip — the clock and Card 044's expiry
// receipt are unchanged; only the verdict after the expiry is reclassified. The read-only
// snapshot the goal still owns is retired at the termination boundary (the designed path), the
// superseded pre-commitment work ends at kCanceled with the deadline's own fault, and
// finish_if_terminal reclassifies it on the established evidence.
TEST_F(RestockCoordinatorDriverTest, WholeTaskDeadlineEndsInTheTypedRecoverableSkip)
{
  accept_and_request_snapshot();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kValidateScene);

  now_ += 2000ms;
  driver_->pump(now_);

  const auto outputs = driver_->take_outputs();
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("whole-task steady deadline exceeded: ", 0) == 0;
      })) << "Card 044's expiry receipt must still come first";
  const auto terminal = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.outcome == RestockActionOutcome::kRecoverableSkip &&
             (output.kind == CoordinatorDriverOutputKind::kAborted ||
             output.kind == CoordinatorDriverOutputKind::kCanceled);
    });
  ASSERT_NE(terminal, outputs.end())
    << "the deadline terminal must be the typed recoverable skip";
  EXPECT_NE(
    terminal->detail.find("whole-task steady deadline exceeded"), std::string::npos)
    << terminal->detail;
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery classification: RECOVERABLE", 0) == 0;
      })) << "the terminal must be classified";
}

// Card 051 review blocker (the pinning test): a client cancellation that won first termination
// makes the whole-task deadline RECEIPT-ONLY — Card 044's expiry line still prints, but no
// deadline termination is applied (the machine never gains the cleanup-retreat authority, the
// latched kUserCancel intent is never upgraded), no retreat is ever commanded, and the
// terminal stays the operator's kCanceled.
TEST_F(RestockCoordinatorDriverTest, UserCancelFirstMakesTheWholeTaskDeadlineReceiptOnly)
{
  const auto reserve = reach_staging_boundary();
  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kCancelRequested, generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_EQ(port_.releases.size(), 1U);
  ASSERT_FALSE(driver_->snapshot().transition->task_deadline_exceeded);
  (void)driver_->take_outputs();

  // The whole-task deadline instant passes while the cancellation owns the termination.
  now_ += 2000ms;
  driver_->pump(now_);
  const auto deadline_outputs = driver_->take_outputs();
  EXPECT_TRUE(
    std::ranges::any_of(
      deadline_outputs, [](const auto & output) {
        return output.detail.rfind("whole-task steady deadline exceeded: ", 0) == 0;
      })) << "Card 044's expiry receipt must still come first";
  EXPECT_TRUE(
    std::ranges::any_of(
      deadline_outputs, [](const auto & output) {
        return output.detail.rfind(
          "whole-task deadline observed after client cancellation", 0) == 0;
      })) << "the receipt-only note must name the first termination";
  EXPECT_FALSE(driver_->snapshot().transition->task_deadline_exceeded)
    << "receipt-only: the machine must never gain the deadline's retreat authority";
  EXPECT_FALSE(admission_.snapshot().task_deadline_exceeded)
    << "receipt-only: the admission policy must not be applied over a latched cancellation";
  EXPECT_EQ(admission_.snapshot().termination_intent, GoalTerminationIntent::kUserCancel)
    << "the latched cancellation intent must not be upgraded to kTaskDeadline";
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(
    std::ranges::none_of(
      deadline_outputs, [](const auto & output) {
        return output.detail.rfind(
          "recovery rung 1 (safe retreat): the cleanup retreat is commandable", 0) == 0;
      })) << "no cleanup-retreat authority may be receipted";

  // The terminal is the operator's cancellation, not a deadline verdict and not a latch.
  complete_failed_safe_release(*reserve);
  const auto terminal_outputs = driver_->take_outputs();
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(
    std::ranges::any_of(
      terminal_outputs, [](const auto & output) {
        return output.kind == CoordinatorDriverOutputKind::kCanceled &&
               output.outcome == RestockActionOutcome::kCanceled;
      })) << "the terminal must stay the operator's kCanceled";
}

// Card 051 review blocker, reverse order: the deadline legitimately arms first, then the
// operator cancels — the revocation flag must be set so every retreat guard stops in place
// (the identity half is pinned by DeadlineThenUserCancelRevokesTheCleanupRetreatIdentity).
TEST_F(RestockCoordinatorDriverTest, UserCancelAfterTheDeadlineRevokesTheRetreatAuthority)
{
  const auto reserve = reach_staging_boundary();
  now_ += 2000ms;
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().transition->task_deadline_exceeded);
  EXPECT_FALSE(driver_->snapshot().transition->deadline_retreat_revoked);

  ASSERT_TRUE(
    termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{
        CoordinatorControlKind::kCancelRequested, generation_, 0U, now_, "stop after deadline"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().transition->deadline_retreat_revoked)
    << "a user cancel must revoke the deadline's retreat authority";
  EXPECT_FALSE(driver_->snapshot().inhibited);

  // Let the release (routed by the deadline termination) complete: the goal ends without ever
  // commanding a retreat and without an operator latch from this card's paths.
  for (int pump = 0; pump < 8 && port_.releases.empty(); ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.releases.size(), 1U);
  complete_failed_safe_release(*reserve);
  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
}

// Milestone 10 §6 rung 5 (Card 051): the goal-level budget-termination test — the skip is
// requested exactly once; its cleanup cannot succeed (no retreat target in this fixture), the
// ladder lands back on the fault boundary, and the second request is refused so the goal latches
// for the operator instead of looping. Bounded: request → refusal → latch.
TEST_F(RestockCoordinatorDriverTest, ARecoverableSkipCannotLoopAndStillLatchesForTheOperator)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanApproach);
  for (int attempt = 0; attempt < 2; ++attempt) {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete(MotionOutcome::kPlanningFailed, "no approach solution in this scene");
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);
  ASSERT_TRUE(reserve_response_);
  auto stopped = reserved_world_snapshot(*reserve_response_);
  stopped.revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(stopped));
  driver_->pump(now_);
  // Burn both grasp candidates with planner verdicts (rung 4), then exhaust the recovery budget.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  for (int candidate = 0; candidate < 2; ++candidate) {
    ASSERT_TRUE(motion_.outstanding());
    motion_.complete_planner_refusal("no reach from this posture");
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U)
    << "the exhausted-batch recovery requested the next fresh snapshot";
  auto exhausted = reserved_world_snapshot(*reserve_response_);
  exhausted.revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(exhausted));
  driver_->pump(now_);
  // Recovery resumes at plan_pre_grasp with an empty batch: the terminal failure exhausts the
  // recovery budget and the machine faults — the ladder boundary.
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kFault);

  // Pump through the rung-5 request, the untargetable retreat's bounded refusals, and the
  // second fault: the loop must end at an operator latch, never a second skip. The receipts
  // stay undrained so the single-shot count sees the whole goal.
  for (int pump = 0; pump < 24 && !driver_->snapshot().inhibited; ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
    if (driver_->snapshot().transition &&
      driver_->snapshot().transition->state == RestockTaskState::kReleaseTask &&
      !port_.releases.empty())
    {
      break;
    }
  }
  const auto outputs = driver_->take_outputs();
  const auto skip_count = std::count_if(
    outputs.begin(), outputs.end(), [](const auto & output) {
      return output.detail.rfind("recovery rung 5 (recoverable skip): requested", 0) == 0;
    });
  if (skip_count != 1 || !driver_->snapshot().inhibited) {
    std::string dump;
    for (const auto & output : outputs) {
      dump += std::to_string(static_cast<int>(output.kind)) + ": " + output.detail + "\n";
    }
    ADD_FAILURE() << "skip_count=" << skip_count << " inhibited=" <<
      (driver_->snapshot().inhibited ? "true" : "false") << "\noutputs:\n" << dump;
  }
  // A skip whose cleanup could not succeed has latched: the typed outcome must not survive it,
  // or the campaign would charge a skip beside an inhibited admission and its next goal would
  // come back "rejected" (the live receipt from the predeclared population at 92cb309).
  EXPECT_FALSE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.outcome == RestockActionOutcome::kRecoverableSkip;
      })) << "the failed skip's terminal must be the operator latch, not the typed skip";
  EXPECT_TRUE(driver_->snapshot().inhibited)
    << "the ladder must terminate at the operator latch after one skip attempt";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery classification: UNSAFE", 0) == 0;
      })) << "the latch must classify UNSAFE (stop evidence lost / recovery failed)";
}

TEST_F(RestockCoordinatorDriverTest, CompletesAWholeTransferAndReleasesTheReservation)
{
  reach_close_gripper();
  driver_->pump(now_);
  complete_task_from_close_gripper();

  // Success releases through the reservation; a held capability would strand later tasks.
  ASSERT_EQ(port_.releases.size(), 1U);
  const auto terminal_task_phase =
    static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase);
  const auto terminal_fault_state =
    static_cast<FaultState>(port_.releases.front().request->terminal_fault_state);
  auto release = std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
  release->status = restocker_world_state::operation_status_ok();
  release->world_revision = ++observation_revision_;
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
    port_.releases,
    std::move(release));
  driver_->pump(now_);

  // Readback: no reservation, product in its destination lane.
  ASSERT_EQ(port_.snapshots.size(), 1U);
  auto released = world_snapshot(observation_revision_);
  released.robot.revision = released.revision;
  released.robot.task_phase = terminal_task_phase;
  released.robot.fault_state = terminal_fault_state;
  released.lanes.at(selected_pair(released).lane_id)
  .contents.push_back(selected_pair(released).object_id);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots,
    snapshot_response(released));
  driver_->pump(now_);

  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_FALSE(snapshot.inhibited);
  EXPECT_EQ(snapshot.transition->state, RestockTaskState::kComplete);
  EXPECT_TRUE(snapshot.terminal_output_emitted);
}

// Milestone 10 §6 rung 5 (Card 051): a pre-motion skip (nothing ran, reservation active)
// releases through the admission slot's own skip-cleanup claim — without it the release is
// refused as "cleanup without termination", the coordinator latches, and the terminal arrives
// as a skip beside an inhibited admission (the live receipt chain from the predeclared
// population at 92cb309: "terminal reservation release was blocked by admission state" →
// outcome=13 → the next goal "rejected").
TEST_F(RestockCoordinatorDriverTest, RecoverableSkipReleasesAndDeliversTheTypedOutcome)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(staged_batch_);
  ASSERT_EQ(staged_batch_->candidates.size(), 2U);
  ASSERT_TRUE(motion_.outstanding());
  (void)driver_->take_outputs();

  // Both candidates refuse on planner verdicts: no trajectory ever runs, so the skip's
  // cleanup is a reservation release, not a retreat.
  motion_.complete_planner_refusal("no reach from this posture");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete_planner_refusal("no reach from this posture");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);

  for (int snapshot_round = 0; snapshot_round < 2; ++snapshot_round) {
    ASSERT_EQ(port_.snapshots.size(), 1U) << "snapshot round " << snapshot_round;
    ASSERT_TRUE(reserve_response_);
    auto fresh = reserved_world_snapshot(*reserve_response_);
    fresh.revision = ++observation_revision_;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(fresh));
    driver_->pump(now_);
  }
  // The fault and the rung-5 request happen inside the second snapshot's pump: the machine
  // lands at the release the skip routed it to.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kReleaseTask);

  // Pump through the rung-5 request to the release submission and its completion.
  for (int pump = 0; pump < 24 && port_.releases.empty(); ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.releases.size(), 1U)
    << "the skip's reservation release must be submitted; state=" <<
    (driver_->snapshot().transition ?
  to_string(driver_->snapshot().transition->state) : "?") <<
    " detail=" << (driver_->snapshot().transition ?
  driver_->snapshot().transition->detail : std::string());
  // Complete the release inline: this flow's two re-observation snapshots advanced the
  // retained revision past the original reserve response, so the acknowledgement must be
  // newer than both (the shared helper's +1 is pinned to the original reserve revision).
  {
    const auto terminal_task_phase =
      static_cast<TaskPhase>(port_.releases.front().request->terminal_task_phase);
    const auto terminal_fault_state =
      static_cast<FaultState>(port_.releases.front().request->terminal_fault_state);
    auto release =
      std::make_shared<restocker_interfaces::srv::ReleaseTaskReservation::Response>();
    release->status = restocker_world_state::operation_status_ok();
    release->world_revision =
      std::max(observation_revision_, reserve_response_->world_revision) + 1U;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::ReleaseReservation>(
      port_.releases, release);
    driver_->pump(now_);
    ASSERT_EQ(port_.snapshots.size(), 1U) << "post-release readback snapshot";
    auto released = world_snapshot(release->world_revision);
    released.robot.revision = released.revision;
    released.robot.task_phase = terminal_task_phase;
    released.robot.fault_state = terminal_fault_state;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots, snapshot_response(released));
    driver_->pump(now_);
  }

  const auto outputs = driver_->take_outputs();
  if (driver_->snapshot().inhibited) {
    std::string dump;
    for (const auto & output : outputs) {
      dump += std::to_string(static_cast<int>(output.kind)) + ": " + output.detail + "\n";
    }
    ADD_FAILURE() << "latched; state=" <<
      (driver_->snapshot().transition ?
    std::string(to_string(driver_->snapshot().transition->state)) + " detail=" +
    driver_->snapshot().transition->detail : std::string("none")) <<
      "\noutputs:\n" << dump;
  }
  EXPECT_FALSE(driver_->snapshot().inhibited)
    << "a clean pre-motion skip must not latch the coordinator";
  const auto terminal = std::ranges::find_if(
    outputs, [](const auto & output) {
      return output.outcome == RestockActionOutcome::kRecoverableSkip &&
             (output.kind == CoordinatorDriverOutputKind::kAborted ||
             output.kind == CoordinatorDriverOutputKind::kCanceled);
    });
  ASSERT_NE(terminal, outputs.end())
    << "the released skip must deliver the typed outcome";
  EXPECT_TRUE(
    std::ranges::any_of(
      outputs, [](const auto & output) {
        return output.detail.rfind("recovery rung 5 (recoverable skip): requested", 0) == 0;
      })) << "the rung-5 request must be receipted";
}

TEST_F(RestockCoordinatorDriverTest, PhysicalDetachInvalidatesLaneAndSurveyAcquireRefreshesEvidence)
{
  std::vector<std::pair<std::string, std::uint64_t>> invalidated;
  std::optional<CoordinatorDestinationObservationDone> pending_acquire;
  std::optional<OperationCorrelation> pending_correlation;
  std::string acquired_lane;
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr,
    [](const SelectedTaskPair &) {
      Eigen::Isometry3d viewpoint = Eigen::Isometry3d::Identity();
      viewpoint.translation() = Eigen::Vector3d{-0.8, -0.2, 1.4};
      return std::optional<Eigen::Isometry3d>{viewpoint};
    },
    [&](const std::string & lane_id, std::uint64_t revision) {
      invalidated.emplace_back(lane_id, revision);
    },
    [&](OperationCorrelation correlation, const std::string & lane_id,
    CoordinatorDestinationObservationDone done) {
      acquired_lane = lane_id;
      pending_correlation = correlation;
      pending_acquire = std::move(done);
      return true;
    });
  reserve_response_ = reach_staging_boundary();
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "approach reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kCloseGripper);
  driver_->pump(now_);
  complete_task_from_close_gripper_with_acquire(pending_acquire, pending_correlation);

  ASSERT_EQ(invalidated.size(), 1U);
  EXPECT_EQ(invalidated.front().first, "lane_01");
  EXPECT_EQ(acquired_lane, "lane_01");
  ASSERT_EQ(port_.releases.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, UnlivePerceptionFailsMotionWithinLivenessHorizon)
{
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {}, [](std::string & detail) {
      detail =
      "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  // Staging does not need the camera; PlanPreGrasp reaches the liveness gate, starts the
  // bounded reacquire, and must not submit any motion request while the stream is unlive.
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_TRUE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  {
    const auto outputs = driver_->take_outputs();
    bool reported_start = false;
    for (const auto & output : outputs) {
      if (output.detail.find("bounded reacquire started") != std::string::npos) {
        reported_start = true;
      }
    }
    EXPECT_TRUE(reported_start) << "the reacquire start must be observable";
  }
  // Still inside the budget the hold persists without submitting anything.
  now_ += reacquire_timeout_ / 2;
  driver_->pump(now_);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_TRUE(driver_->snapshot().perception_reacquire_active);
  // Expiry fails closed with the recorded refusal detail, still without any motion. The
  // terminal chain runs to operator-required in the same pump; the refusal details live in
  // the output history it published on the way there.
  now_ += reacquire_timeout_;
  driver_->pump(now_);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_EQ(snapshot.transition->state, RestockTaskState::kRequestOperator)
    << snapshot.transition->detail;
  bool reported_liveness = false;
  bool reported_reacquire = false;
  bool reported_unlive_refusal = false;
  bool reported_recovery_refusal = false;
  for (const auto & output : driver_->take_outputs()) {
    reported_liveness =
      reported_liveness ||
      (output.detail.find("liveness") != std::string::npos);
    reported_reacquire =
      reported_reacquire ||
      (output.detail.find("reacquire") != std::string::npos);
    reported_unlive_refusal =
      reported_unlive_refusal ||
      (output.detail.find("perception stream is unlive") != std::string::npos);
    reported_recovery_refusal =
      reported_recovery_refusal ||
      (output.detail.find("recovery is refused: perception stream is unlive") !=
      std::string::npos);
  }
  EXPECT_TRUE(reported_liveness) << "the liveness reason must stay observable";
  EXPECT_TRUE(reported_reacquire) << "the expired reacquire must be named";
  EXPECT_TRUE(reported_unlive_refusal) << "the recorded refusal reason must be retained";
  EXPECT_TRUE(reported_recovery_refusal)
    << "the fail-closed recovery refusal must carry the exact recorded unlive evidence "
    "(the demo-latch string: recovery is refused: perception stream is unlive)";
}

TEST_F(RestockCoordinatorDriverTest, ReacquireBoundariesCarryThePredicateDetail)
{
  // The retained console previously had the hold's budget but none of the numbers the
  // predicate itself produced, so producer, delivery and consumer could not be told apart
  // after a refusal. Both boundaries — start and expiry — must carry the detail verbatim.
  constexpr const char * kPredicateDetail =
    "wrist camera acquisition is 812 ms old (horizon 500 ms, last stamp 1 ns, "
    "now 813000000 ns, receipt lag 640 ms, arrivals=42 stamp_advances=7)";
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {}, [kPredicateDetail](std::string & detail) {
      detail = kPredicateDetail;
      return false;
    });
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().perception_reacquire_active);
  bool start_carried_detail = false;
  for (const auto & output : driver_->take_outputs()) {
    if (output.detail.find("bounded reacquire started") != std::string::npos &&
      output.detail.find(kPredicateDetail) != std::string::npos)
    {
      start_carried_detail = true;
    }
  }
  EXPECT_TRUE(start_carried_detail)
    << "the hold-start feedback must carry the predicate's arrival counters";
  now_ += reacquire_timeout_;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  bool expiry_carried_detail = false;
  bool expiry_named = false;
  for (const auto & output : driver_->take_outputs()) {
    if (output.detail.find("reacquire expired") != std::string::npos) {
      expiry_named = true;
      if (output.detail.find(kPredicateDetail) != std::string::npos) {
        expiry_carried_detail = true;
      }
    }
  }
  EXPECT_TRUE(expiry_named) << "the hold expiry must be its own observable feedback";
  EXPECT_TRUE(expiry_carried_detail)
    << "the hold-expiry feedback must carry the predicate's arrival counters";
}

TEST_F(RestockCoordinatorDriverTest, ZeroReacquireBudgetRefusesImmediatelyWithoutAHold)
{
  // Zero is documented as "skip the hold": the unchanged liveness predicate must refuse in the
  // same pump the segment is requested, never start a hold, and never mention a reacquire.
  reacquire_timeout_ = 0ms;
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {}, [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  auto reserve = reach_reservation_request();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ReserveTask>(port_.reservations, reserve);
  driver_->pump(now_);
  ASSERT_EQ(port_.validations.size(), 1U);
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::ValidateReservation>(
    port_.validations, validation_response(*reserve));
  driver_->pump(now_);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_EQ(snapshot.transition->state, RestockTaskState::kRequestOperator)
    << snapshot.transition->detail;
  bool reported_reacquire = false;
  bool reported_liveness = false;
  bool reported_unlive_refusal = false;
  for (const auto & output : driver_->take_outputs()) {
    reported_reacquire =
      reported_reacquire ||
      (output.detail.find("reacquire") != std::string::npos);
    reported_liveness =
      reported_liveness ||
      (output.detail.find("liveness") != std::string::npos);
    reported_unlive_refusal =
      reported_unlive_refusal ||
      (output.detail.find("perception stream is unlive") != std::string::npos);
  }
  EXPECT_FALSE(reported_reacquire) << "a zero budget must never mention a reacquire";
  EXPECT_TRUE(reported_liveness) << "the liveness reason must stay observable";
  EXPECT_TRUE(reported_unlive_refusal) << "the recorded refusal reason must be retained";
}

TEST_F(RestockCoordinatorDriverTest, CancellationDuringReacquireHoldClearsTheHold)
{
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {}, [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  reserve_response_ = reach_staging_boundary();
  ASSERT_TRUE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(motion_.outstanding());

  // Cancellation must abandon the hold without ever booking or submitting the segment, and
  // the released snapshot must not still claim a reacquire is holding.
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(motion_.outstanding());
  // The only ledger work the pump may book is the reservation release, never the segment.
  ASSERT_TRUE(driver_->snapshot().pending_operation.has_value());
  EXPECT_EQ(
    driver_->snapshot().pending_operation->command,
    RestockTaskCommand::kReleaseTaskReservation);
  EXPECT_EQ(port_.releases.size(), 1U);
}

TEST_F(RestockCoordinatorDriverTest, GateFaultDuringReacquireHoldClearsTheHold)
{
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {}, [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  reserve_response_ = reach_staging_boundary();
  ASSERT_TRUE(driver_->snapshot().perception_reacquire_active);

  // An inhibited pump leaves the plan state without the cancellation branch ever running; the
  // orphaned hold must not survive into the terminal snapshot.
  ASSERT_EQ(
    accepted_gate_.gate->mark_synchronization_failure(),
    GenerationSynchronizationFailureStatus::kMarkedFailed);
  driver_->pump(now_);
  EXPECT_TRUE(driver_->snapshot().inhibited);
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(motion_.outstanding());
}

TEST_F(RestockCoordinatorDriverTest, PerceptionReacquireWithinBudgetResumesSegmentSubmission)
{
  // SC-001, unit form: a transient unlive dip converts into a bounded reacquire that returns
  // to a submitted segment once the stream is live again, all inside the same task.
  std::atomic<bool> live{false};
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {},
    [&live](std::string & detail) {
      if (live.load(std::memory_order_acquire)) {
        return true;
      }
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  reserve_response_ = reach_staging_boundary();
  // The unlive dip held PlanPreGrasp instead of submitting or failing terminally.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_FALSE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().pending_operation.has_value());
  EXPECT_TRUE(driver_->snapshot().perception_reacquire_active);

  // The stream comes back inside the budget: the next pump re-checks the same predicate,
  // passes, and submits the pre-grasp.
  live.store(true, std::memory_order_release);
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_TRUE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  const auto outputs = driver_->take_outputs();
  bool reported_start = false;
  bool reported_restore = false;
  for (const auto & output : outputs) {
    if (output.detail.find("bounded reacquire started") != std::string::npos) {
      reported_start = true;
    }
    if (output.detail.find("reacquired after") != std::string::npos) {
      reported_restore = true;
    }
  }
  EXPECT_TRUE(reported_start);
  EXPECT_TRUE(reported_restore);
}

TEST_F(RestockCoordinatorDriverTest, PerceptionReacquireObservesRestorationFromAnotherThread)
{
  // The wrist-depth callback notes acquisitions from an ingress thread while the pump reads
  // liveness; the reacquire must observe a restoration published by another thread.
  std::mutex mutex;
  std::condition_variable cv;
  bool live = false;
  bool unlive_seen = false;
  install_driver(
    generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, nullptr, nullptr, {},
    {}, {},
    [&](std::string & detail) -> bool {
      std::lock_guard lock(mutex);
      if (live) {
        return true;
      }
      unlive_seen = true;
      cv.notify_all();
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  reserve_response_ = reach_staging_boundary();
  ASSERT_TRUE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(motion_.outstanding());
  ASSERT_TRUE(unlive_seen) << "the hold never consulted the liveness predicate";
  // Start the restorer only after the hold is proven, so no assertion above can abandon a
  // joinable thread; the flag it waits on is already set.
  std::thread restorer([&]() {
      std::unique_lock lock(mutex);
      cv.wait(lock, [&]() {return unlive_seen;});
      live = true;
      cv.notify_all();
    });
  // Wait for the other thread to publish the restoration, then hand the pump another tick.
  // On timeout, force the restoration so the join below cannot hang the test binary.
  {
    std::unique_lock lock(mutex);
    if (!cv.wait_for(lock, 5s, [&]() {return live;})) {
      live = true;
      cv.notify_all();
      restorer.join();
      FAIL() << "the restorer thread never published the restoration";
    }
  }
  now_ += 1ms;
  driver_->pump(now_);
  restorer.join();
  EXPECT_TRUE(motion_.outstanding());
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
}

TEST_F(RestockCoordinatorDriverTest, RejectsReacquireBudgetThatOutlivesItsPlanCommandDeadline)
{
  RestockCoordinatorDriverConfig config;
  config.task.validation_timeout = 100ms;
  config.task.total_timeout = 1000ms;
  config.reconciliation = {500ms, 100ms, 4U};
  config.perception_reacquire_timeout =
    config.task.planning_timeout + config.task.execution_timeout;
  EXPECT_THROW(
    RestockCoordinatorDriver(
      admission_, port_,
      CoordinatorTaskSelector{[](const auto & snapshot, const auto &) {
          return SelectionResult<SelectedTaskPair>::success(selected_pair(snapshot));
        }},
      [](const auto & snapshot) {
        return GraspCandidateResult<GraspGenerationAuthority>::success(grasp_authority(snapshot));
      },
      generated_grasps, [this]() {return now_;},
      []() {return rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME);}, config,
      CoordinatorSteadyNow{}, fake_placement_generator(), &motion_, &gripper_, &attachment_),
    std::invalid_argument);
}

TEST_F(RestockCoordinatorDriverTest, SelectionReacquireRetriesSelectionAfterAFreshObservation)
{
  // Selection-time regression (transfer-23 signature): an unlive wrist stream at kSelectPair
  // holds a bounded reacquire instead of terminal, the selector is not re-run while the
  // stream is unlive, and a fresh observation lets selection complete inside the budget.
  bool live = false;
  int selector_calls = 0;
  install_driver(
    generated_grasps, {},
    [&](const auto & snapshot, const auto &) -> SelectionResult<SelectedTaskPair> {
      ++selector_calls;
      if (!live) {
        return SelectionResult<SelectedTaskPair>::failure(
          SelectionError{SelectionErrorCode::PerceptionUnlive, "perception stream is stale"});
      }
      return SelectionResult<SelectedTaskPair>::success(selected_pair(snapshot));
    },
    {}, {}, {}, {}, {}, {}, {}, {},
    [&](std::string & detail) {
      if (live) {
        return true;
      }
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot()));
  driver_->pump(now_);
  // Selection failed unlive: held at kSelectPair, no reservation attempt, no terminal.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kSelectPair);
  EXPECT_TRUE(driver_->snapshot().selection_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().perception_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_EQ(selector_calls, 1);
  {
    bool reported_start = false;
    bool reported_selection = false;
    bool start_carried_detail = false;
    for (const auto & output : driver_->take_outputs()) {
      reported_start =
        reported_start ||
        (output.detail.find("bounded reacquire started") != std::string::npos);
      reported_selection =
        reported_selection ||
        (output.detail.find("task selection found the perception stream unlive") !=
        std::string::npos);
      start_carried_detail =
        start_carried_detail ||
        (output.detail.find("bounded reacquire started") != std::string::npos &&
        output.detail.find("wrist camera acquisition is older") != std::string::npos);
    }
    EXPECT_TRUE(reported_start);
    EXPECT_TRUE(reported_selection);
    EXPECT_TRUE(start_carried_detail)
      << "the selection hold-start feedback must carry the predicate detail";
  }
  // While the stream stays unlive the hold polls the cheap predicate only: the selector
  // (the full, expensive path) is not re-run, so this cannot hot-loop selection.
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(selector_calls, 1);
  EXPECT_TRUE(driver_->snapshot().selection_reacquire_active);
  // A fresh observation arrives inside the budget: selection retries and succeeds, the
  // reservation request is issued, and the hold is gone.
  live = true;
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(selector_calls, 2);
  EXPECT_FALSE(driver_->snapshot().selection_reacquire_active);
  EXPECT_EQ(port_.reservations.size(), 1U);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  {
    bool reported_restore = false;
    for (const auto & output : driver_->take_outputs()) {
      if (output.detail.find("reacquired after") != std::string::npos &&
        output.detail.find("retrying task selection") != std::string::npos)
      {
        reported_restore = true;
      }
    }
    EXPECT_TRUE(reported_restore) << "the restore must be observable";
  }
}

TEST_F(RestockCoordinatorDriverTest, SelectionReacquireExhaustsIntoTerminalSelectionFailure)
{
  // The stream never returns: the hold spends the shared steady-clock budget, then terminals
  // with the original selection failure and the expiry named — fail-closed, never bypassed.
  int selector_calls = 0;
  install_driver(
    generated_grasps, {},
    [&](const auto & snapshot, const auto &) -> SelectionResult<SelectedTaskPair> {
      ++selector_calls;
      return SelectionResult<SelectedTaskPair>::failure(
        SelectionError{SelectionErrorCode::PerceptionUnlive, "perception stream is stale"});
    },
    {}, {}, {}, {}, {}, {}, {}, {},
    [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot()));
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kSelectPair);
  ASSERT_TRUE(driver_->snapshot().selection_reacquire_active);
  ASSERT_EQ(selector_calls, 1);
  // Still inside the budget: held without re-running the selector.
  now_ += reacquire_timeout_ / 2;
  driver_->pump(now_);
  EXPECT_EQ(selector_calls, 1);
  EXPECT_TRUE(driver_->snapshot().selection_reacquire_active);
  EXPECT_TRUE(port_.reservations.empty());
  // Expiry: the selector re-reports the staleness and the goal terminals with both the
  // original detail and the expired reacquire named.
  now_ += reacquire_timeout_;
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().selection_reacquire_active);
  EXPECT_TRUE(port_.reservations.empty());
  EXPECT_EQ(selector_calls, 2);
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_EQ(snapshot.transition->state, RestockTaskState::kRequestOperator)
    << snapshot.transition->detail;
  bool reported_selection_failure = false;
  bool reported_expiry = false;
  bool reported_stale = false;
  bool stale_outcome = false;
  bool reported_reacquire_expiry_feedback = false;
  for (const auto & output : driver_->take_outputs()) {
    reported_selection_failure =
      reported_selection_failure ||
      (output.detail.find("task selection failed") != std::string::npos);
    reported_expiry =
      reported_expiry ||
      (output.detail.find("did not restore it") != std::string::npos);
    reported_stale =
      reported_stale ||
      (output.detail.find("perception stream is stale") != std::string::npos);
    stale_outcome =
      stale_outcome ||
      (output.outcome == RestockActionOutcome::kObservationEvidenceStale);
    reported_reacquire_expiry_feedback =
      reported_reacquire_expiry_feedback ||
      (output.detail.find("reacquire expired during task selection") != std::string::npos &&
      output.detail.find("wrist camera acquisition is older") != std::string::npos);
  }
  EXPECT_TRUE(reported_selection_failure);
  EXPECT_TRUE(reported_expiry);
  EXPECT_TRUE(reported_stale);
  EXPECT_TRUE(stale_outcome);
  EXPECT_TRUE(reported_reacquire_expiry_feedback)
    << "the selection hold's expiry must be observable with the predicate detail, "
    "not only folded into the terminal text";
}

TEST_F(RestockCoordinatorDriverTest, NonLivenessSelectionFailureStillFailsImmediately)
{
  // The reacquire widens nothing else: any non-liveness selection failure keeps the original
  // immediate terminal, even while the wrist predicate reports the stream unlive.
  int selector_calls = 0;
  install_driver(
    generated_grasps, {},
    [&](const auto & snapshot, const auto &) -> SelectionResult<SelectedTaskPair> {
      ++selector_calls;
      return SelectionResult<SelectedTaskPair>::failure(
        SelectionError{SelectionErrorCode::RobotUnavailable, "robot is not available"});
    },
    {}, {}, {}, {}, {}, {}, {}, {},
    [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot()));
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().selection_reacquire_active);
  EXPECT_TRUE(driver_->snapshot().terminal_output_emitted);
  EXPECT_EQ(selector_calls, 1);
  EXPECT_TRUE(port_.reservations.empty());
}

TEST_F(RestockCoordinatorDriverTest, SelectionReacquireSupersedesToFreshEvidenceWhenSnapshotAged)
{
  // The retained-evidence wall behind the selection hold: latest_snapshot is captured once
  // and cannot gain younger stamps while kSelectPair holds it. A selector faithful to
  // select_task_pair's own ordering (stream liveness, then the snapshot's robot and object
  // stamps) must fail stale against the retained copy once wall time passes the horizons —
  // even though the live predicate just passed — and the driver must supersede back to
  // re-observation rather than terminal, completing on a refreshed snapshot.
  rclcpp::Time now_ros{std::int64_t{1'100'000'000}, RCL_ROS_TIME};
  rclcpp::Time last_acq{std::int64_t{1'100'000'000 - 600'000'000}, RCL_ROS_TIME};
  int selector_calls = 0;
  std::vector<SelectionErrorCode> failures;
  auto unlive_now = [&]() {
    return (now_ros - last_acq).nanoseconds() > 500'000'000LL;
  };
  install_driver(
    generated_grasps, {},
    [&](const auto & snapshot, const auto &) -> SelectionResult<SelectedTaskPair> {
      ++selector_calls;
      if (unlive_now()) {
        failures.push_back(SelectionErrorCode::PerceptionUnlive);
        return SelectionResult<SelectedTaskPair>::failure(
          SelectionError{SelectionErrorCode::PerceptionUnlive, "perception stream is stale"});
      }
      if ((now_ros - snapshot.robot.telemetry_time).nanoseconds() > 500'000'000LL) {
        failures.push_back(SelectionErrorCode::RobotStale);
        return SelectionResult<SelectedTaskPair>::failure(
          SelectionError{SelectionErrorCode::RobotStale, "robot telemetry is stale"});
      }
      for (const auto & entry : snapshot.objects) {
        if ((now_ros - entry.second.observation_time).nanoseconds() > 500'000'000LL) {
          failures.push_back(SelectionErrorCode::ObjectStale);
          return SelectionResult<SelectedTaskPair>::failure(
            SelectionError{SelectionErrorCode::ObjectStale, "object is stale"});
        }
      }
      return SelectionResult<SelectedTaskPair>::success(selected_pair(snapshot));
    },
    {}, {}, {}, {}, {}, {}, {}, {},
    [&](std::string & detail) {
      if (unlive_now()) {
        detail = "wrist camera acquisition is older than the configured liveness horizon";
        return false;
      }
      return true;
    });
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot()));
  driver_->pump(now_);
  // Unlive at the first selection: held on the cheap predicate, selector not re-run, no
  // re-observation request, no terminal.
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kSelectPair);
  ASSERT_TRUE(driver_->snapshot().selection_reacquire_active);
  ASSERT_EQ(selector_calls, 1);
  ASSERT_EQ(failures.size(), 1U);
  EXPECT_EQ(failures.back(), SelectionErrorCode::PerceptionUnlive);
  now_ += 1ms;
  driver_->pump(now_);
  EXPECT_EQ(selector_calls, 1);
  EXPECT_TRUE(driver_->snapshot().selection_reacquire_active);
  // The stream restores, but wall time has carried the retained snapshot past both 500 ms
  // horizons: its stamps are frozen at 1.0 s while now is 1.7 s, and the fresh acquisition
  // itself is stamped at the new now.
  now_ros = rclcpp::Time(std::int64_t{1'100'000'000 + 600'000'000}, RCL_ROS_TIME);
  last_acq = now_ros;
  now_ += 1ms;
  driver_->pump(now_);
  // The live predicate passed, the selector still refused the aged copy (RobotStale), and
  // the driver superseded to a fresh observation instead of terminaling.
  ASSERT_EQ(selector_calls, 2);
  ASSERT_EQ(failures.size(), 2U);
  EXPECT_EQ(failures.back(), SelectionErrorCode::RobotStale)
    << "the retained snapshot must be what failed, not the predicate";
  const auto held = driver_->snapshot();
  ASSERT_TRUE(held.transition);
  EXPECT_EQ(held.transition->state, RestockTaskState::kValidateScene)
    << held.transition->detail;
  EXPECT_TRUE(held.selection_reacquire_active) << "the shared budget must survive re-observe";
  EXPECT_FALSE(held.terminal_output_emitted);
  EXPECT_EQ(port_.snapshots.size(), 1U) << "re-observation must request a fresh snapshot";
  // Deliver the refreshed snapshot with stamps at the current now.
  auto fresh = world_snapshot(11U);
  fresh.robot.telemetry_time = now_ros;
  for (auto & entry : fresh.objects) {
    entry.second.observation_time = now_ros;
    entry.second.transition_time = now_ros;
  }
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(fresh));
  driver_->pump(now_);
  // Selection re-runs against the refreshed evidence and reserves; the hold is gone.
  EXPECT_EQ(selector_calls, 3);
  EXPECT_TRUE(failures.size() <= 2U) << "the refreshed snapshot must not fail again";
  EXPECT_EQ(port_.reservations.size(), 1U);
  EXPECT_FALSE(driver_->snapshot().selection_reacquire_active);
  EXPECT_FALSE(driver_->snapshot().terminal_output_emitted);
  {
    bool reported_restore = false;
    bool reported_reobserve = false;
    for (const auto & output : driver_->take_outputs()) {
      reported_restore =
        reported_restore ||
        (output.detail.find("reacquired after") != std::string::npos);
      reported_reobserve =
        reported_reobserve ||
        (output.detail.find("re-observing the scene") != std::string::npos);
    }
    EXPECT_TRUE(reported_restore);
    EXPECT_TRUE(reported_reobserve) << "the supersede must be observable";
  }
}

TEST_F(RestockCoordinatorDriverTest, CancellationDuringSelectionReacquireStopsPromptly)
{
  // Termination must not wait out the hold's budget: cancel at kSelectPair abandons the
  // hold, never re-runs the selector, and reaches the canceled terminal in the same pump.
  int selector_calls = 0;
  install_driver(
    generated_grasps, {},
    [&](const auto &, const auto &) -> SelectionResult<SelectedTaskPair> {
      ++selector_calls;
      return SelectionResult<SelectedTaskPair>::failure(
        SelectionError{SelectionErrorCode::PerceptionUnlive, "perception stream is stale"});
    },
    {}, {}, {}, {}, {}, {}, {}, {},
    [](std::string & detail) {
      detail = "wrist camera acquisition is older than the configured liveness horizon";
      return false;
    });
  accept_and_request_snapshot();
  now_ += 1ms;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(world_snapshot()));
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().selection_reacquire_active);
  ASSERT_EQ(selector_calls, 1);
  ASSERT_TRUE(termination_latch_succeeded(admission_.request_cancel(goal_id(), generation_, now_)));
  ASSERT_EQ(
    driver_->inbox()->push(
      CoordinatorControlEvent{CoordinatorControlKind::kCancelRequested,
        generation_, 0U, now_, "test cancel"}),
    CoordinatorInboxPushResult::kAccepted);
  driver_->pump(now_);
  EXPECT_FALSE(driver_->snapshot().selection_reacquire_active);
  EXPECT_EQ(selector_calls, 1) << "cancel must not re-run the selector";
  EXPECT_TRUE(port_.reservations.empty());
  const auto snapshot = driver_->snapshot();
  ASSERT_TRUE(snapshot.transition);
  EXPECT_EQ(snapshot.transition->state, RestockTaskState::kCanceled)
    << snapshot.transition->detail;
  EXPECT_TRUE(snapshot.terminal_output_emitted);
}

TEST_F(RestockCoordinatorDriverTest, RejectsASecondGoalWhileTheFirstIsStillActive)
{
  reach_close_gripper();

  // One task at a time: a second reservation while the first owns the product would let two goals
  // command the arm.
  const auto receipt = admission_.reserve(goal_id(2U));
  EXPECT_NE(receipt.decision, GoalAdmissionDecision::kAccepted);
}

// ---------------------------------------------------------------------------------------------
// The advisory reasoner.
// Advisory reasoner: the backend cannot change what the deterministic authorisation permits;
// it can only decline a recovery.
// ---------------------------------------------------------------------------------------------

// The one recommendation shape a backend can send, rendered as a backend would send it.
[[nodiscard]] std::string advice_document(
  const std::string & request_id, const std::string & primitive,
  const std::string & failure_class = "trajectory_execution_aborted", double confidence = 0.9)
{
  return std::string(
    R"({"schema_version":"restocker.reasoner.recovery.response.v1","request_id":")") +
         request_id + R"(","status":"ok","failure_class":")" + failure_class +
         R"(","recommended_primitive":")" + primitive + R"(","confidence":)" +
         std::to_string(confidence) + R"(,"explanation":"recorded for a driver test."})";
}

class RestockCoordinatorAdvisoryTest : public RestockCoordinatorDriverTest
{
protected:
  // Reach the free-space execution failure used by the deterministic recovery test, with an
  // advisory backend.
  void reach_recovery_with_advisor()
  {
    install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, &advisor_, &audit_);
    reserve_response_ = reach_staging_boundary();
    ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
    driver_->pump(now_);
    ASSERT_TRUE(motion_.outstanding());

    motion_.complete(
      MotionOutcome::kExecutionFailed,
      "controller aborted on a path tolerance violation", true);
    now_ += 1ms;
    driver_->pump(now_);
  }

  // Complete the observation an authorised recovery waits on.
  void observe_stopped_robot()
  {
    ASSERT_TRUE(reserve_response_);
    auto stopped = reserved_world_snapshot(*reserve_response_);
    stopped.revision = ++observation_revision_;
    now_ += 1ms;
    port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
      port_.snapshots,
      snapshot_response(stopped));
    driver_->pump(now_);
  }

  [[nodiscard]] std::string asked_request_id() const {return advisor_.query().request_id;}

  restocker_reasoner::FakeRecoveryAdvisor advisor_;
  restocker_reasoner::RecoveryAuditLog audit_{""};
};

// The question carries the deterministic description of the failure and only the primitives the
// authorisation permits.
TEST_F(RestockCoordinatorAdvisoryTest, TheQuestionOffersOnlyWhatTheAuthorisationPermits)
{
  reach_recovery_with_advisor();

  EXPECT_EQ(advisor_.submit_attempts, 1U);
  const auto & question = advisor_.query();
  EXPECT_EQ(
    question.motion_outcome,
    std::string(motion_outcome_name(MotionOutcome::kExecutionFailed)));
  EXPECT_TRUE(question.deterministic_refusal.empty());
  EXPECT_EQ(
    question.deterministic_primitive,
    restocker_reasoner::RecoveryPrimitive::kResumeAtRecoveryState);
  ASSERT_EQ(question.permitted_primitives.size(), 2U);
  EXPECT_EQ(
    question.permitted_primitives.front(),
    restocker_reasoner::RecoveryPrimitive::kResumeAtRecoveryState);
  EXPECT_EQ(
    question.permitted_primitives.back(),
    restocker_reasoner::RecoveryPrimitive::kAbandonTask);
}

// An advisory backend that says nothing yields the deterministic outcome pinned by
// FreeSpaceStopAtRestIsRecoveredByReplanningTheSegment, step for step.
TEST_F(RestockCoordinatorAdvisoryTest, AnAdvisorThatNeverAnswersChangesNothing)
{
  reach_recovery_with_advisor();

  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover);
  ASSERT_EQ(port_.snapshots.size(), 1U);

  observe_stopped_robot();

  EXPECT_FALSE(driver_->snapshot().inhibited);
  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
  EXPECT_EQ(driver_->snapshot().transition->fault, RestockTaskFault::kNone);
}

// Same for a refusal to answer. A deadline, a dead service and a non-answer behave alike.
TEST_F(RestockCoordinatorAdvisoryTest, AnAdvisorThatFailsToAnswerChangesNothing)
{
  advisor_.answer_next_with_nothing("the advisory call passed its deadline");
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(driver_->snapshot().transition->fault, RestockTaskFault::kNone);
}

// A recommendation naming what the driver already chose changes nothing.
TEST_F(RestockCoordinatorAdvisoryTest, AnAgreeingRecommendationLeavesTheOutcomeUnchanged)
{
  advisor_.answer_next(advice_document("{request_id}", "resume_at_recovery_state"));
  reach_recovery_with_advisor();
  ASSERT_FALSE(asked_request_id().empty());
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
}

// A recommendation can only stop a recovery the deterministic authorisation would have taken. It
// fails closed like a refusal.
TEST_F(RestockCoordinatorAdvisoryTest, ARecommendationMayDeclineAnAuthorisedRecovery)
{
  advisor_.answer_next(advice_document("{request_id}", "abandon_task"));
  reach_recovery_with_advisor();

  // No observation is requested: the task stopped instead of replanning.
  EXPECT_TRUE(port_.snapshots.empty());
  ASSERT_TRUE(driver_->snapshot().transition);
  EXPECT_NE(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 1U);
}

// A recommendation the schema rejects never reaches the decision.
TEST_F(RestockCoordinatorAdvisoryTest, AnInvalidRecommendationIsRejectedAndRecoveryProceeds)
{
  advisor_.answer_next("I would abandon this task. It looks unrecoverable to me.");
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
}

// A primitive name outside the closed set never reaches anything that dispatches on it.
TEST_F(RestockCoordinatorAdvisoryTest, AnUnknownPrimitiveIsRejectedAndRecoveryProceeds)
{
  advisor_.answer_next(advice_document("{request_id}", "push_to_clear"));
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// A primitive the driver implements but the authorisation does not offer is not accepted.
TEST_F(RestockCoordinatorAdvisoryTest, AnUnpermittedPrimitiveIsRejectedAndRecoveryProceeds)
{
  advisor_.answer_next(advice_document("{request_id}", "retry_segment"));
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  EXPECT_EQ(motion_.submit_attempts, 2U);
}

// An answer to a different question (a slow advisor) is rejected: correlation is checked first.
TEST_F(RestockCoordinatorAdvisoryTest, AnAnswerToADifferentQuestionIsDiscarded)
{
  advisor_.answer_next(advice_document("999-999-insert", "abandon_task"));
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// Confidence below the configured floor is not a recommendation.
TEST_F(RestockCoordinatorAdvisoryTest, ALowConfidenceRecommendationIsRejected)
{
  advisor_.answer_next(
    advice_document("{request_id}", "abandon_task", "trajectory_execution_aborted", 0.10));
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// A segment that succeeded is not a failure, and there is nothing to advise about.
TEST_F(RestockCoordinatorAdvisoryTest, NoQuestionIsAskedAboutASegmentThatSucceeded)
{
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, &advisor_, &audit_);
  reserve_response_ = reach_staging_boundary();
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());

  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp pose reached");
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(advisor_.submit_attempts, 0U);
}

// A truncated straight-line approach is not replanned from. Stopping is then the only permitted
// primitive, so the reasoner is not asked.
TEST_F(RestockCoordinatorAdvisoryTest, ATruncatedLinearSegmentIsNotEvenPutToTheReasoner)
{
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, &advisor_, &audit_);
  reserve_response_ = reach_staging_boundary();
  driver_->pump(now_);

  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);

  ASSERT_TRUE(motion_.outstanding());
  ASSERT_EQ(advisor_.submit_attempts, 0U);
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "linear approach stopped 41% of the way in beside a shelf divider", true);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(advisor_.submit_attempts, 0U);
  EXPECT_TRUE(port_.snapshots.empty());
}

// The authorisation already refused replanning, so stopping is the only permitted primitive and no
// answer could differ; the reasoner is not asked.
TEST_F(RestockCoordinatorAdvisoryTest, NoQuestionIsAskedWhenStoppingIsTheOnlyThingPermitted)
{
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, &advisor_, &audit_);
  reserve_response_ = reach_staging_boundary();
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());

  // A backend that is not there is not a state a replan can improve, so recovery is refused.
  motion_.complete(MotionOutcome::kUnavailable, "MoveIt is not running");
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_EQ(advisor_.submit_attempts, 0U);
}

// A backend that refuses the question (shutting down, circuit breaker open) is the same as no
// backend.
TEST_F(RestockCoordinatorAdvisoryTest, ARefusedQuestionChangesNothing)
{
  advisor_.refuse_next(
    restocker_reasoner::RecoveryAdviceSubmitStatus::kUnavailable,
    "the circuit is open");
  reach_recovery_with_advisor();
  observe_stopped_robot();

  EXPECT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
}

// The audit log records what was decided, not only what happened.
TEST_F(RestockCoordinatorAdvisoryTest, TheAuditLogRecordsTheDecisionAndItsOutcome)
{
  const auto path =
    std::filesystem::temp_directory_path() / "restocker_driver_reasoner_audit.jsonl";
  std::filesystem::remove(path);
  restocker_reasoner::RecoveryAuditLog audit(path.string());
  ASSERT_TRUE(audit.open());

  advisor_.answer_next(advice_document("{request_id}", "abandon_task"));
  install_driver(generated_grasps, {}, {}, &motion_, &gripper_, &attachment_, &advisor_, &audit);
  reserve_response_ = reach_staging_boundary();
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(
    MotionOutcome::kExecutionFailed,
    "controller aborted on a path tolerance violation", true);
  now_ += 1ms;
  driver_->pump(now_);

  EXPECT_GE(audit.appended(), 2U);
  EXPECT_EQ(audit.failed_appends(), 0U);

  std::ifstream stream(path);
  std::string line;
  std::vector<std::string> lines;
  while (std::getline(stream, line)) {
    lines.push_back(line);
  }
  ASSERT_GE(lines.size(), 2U);
  // Every line is a whole JSON object, whatever the backend put in its explanation.
  for (const auto & recorded : lines) {
    EXPECT_TRUE(
      restocker_reasoner::parse_strict_json(
        recorded, restocker_reasoner::JsonParseLimits{65536U, 16U})
      .value)
      << recorded;
  }
  EXPECT_NE(lines.front().find("\"event\":\"asked\""), std::string::npos);
  const auto decided = std::ranges::find_if(
    lines, [](const auto & recorded) {
      return recorded.find("\"event\":\"decided\"") != std::string::npos;
    });
  ASSERT_NE(decided, lines.end());
  EXPECT_NE(decided->find("\"used\":true"), std::string::npos);
  EXPECT_NE(decided->find("\"applied_primitive\":\"abandon_task\""), std::string::npos);
  EXPECT_NE(
    decided->find("\"deterministic_primitive\":\"resume_at_recovery_state\""),
    std::string::npos);
  EXPECT_NE(decided->find("\"response_verbatim\":"), std::string::npos);
  std::filesystem::remove(path);
}

// Milestone 10 §6 (Card 062, review N1): a verified grip is the product's egress — the retract
// carries no grasp escape.
TEST_F(RestockCoordinatorDriverTest, AHeldProductsRetractCarriesNoGraspEscape)
{
  reach_outstanding_attach();
  attachment_.complete(AttachmentOutcome::kSucceeded, "boundary and world state agree");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetract);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_FALSE(motion_.submission().goal.grasp_escape);
}

// Card 062 review N1: once an escape executed, the retry that follows carries none.
TEST_F(RestockCoordinatorDriverTest, TheMotionAfterAnExecutedGraspEscapeCarriesNone)
{
  reach_escape_carrying_pregrasp();
  // The escape ran; the rest was refused for a reason that is not a verdict on the goal, so the
  // same operation is retried from the standoff.
  motion_.complete_after_grasp_escape(
    MotionOutcome::kPlanningFailed, "scene authority was synchronizing", false);
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp);
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_FALSE(motion_.submission().goal.grasp_escape);
  EXPECT_FALSE(driver_->snapshot().inhibited);
}

// Card 062 review N2: the whole-task deadline arrives while the attach is outstanding and the
// attach then rolls back. The escape open is admitted under the latched deadline, and the
// deadline's cleanup retreat carries the escape back to the closed candidate's standoff, inside
// the retreat budget.
TEST_F(RestockCoordinatorDriverTest, DeadlineCleanupAfterARolledBackAttachOpensThenEscapes)
{
  Eigen::Isometry3d survey = Eigen::Isometry3d::Identity();
  survey.translation() = Eigen::Vector3d{-0.2, -0.03, 1.44};
  reach_outstanding_attach([survey](const SelectedTaskPair &) {return survey;});
  ASSERT_TRUE(staged_batch_);
  const auto closed = staged_batch_->candidates.front();

  now_ += 1000ms;
  driver_->pump(now_);
  ASSERT_TRUE(driver_->snapshot().transition->task_deadline_exceeded);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kAttachTransaction);
  (void)driver_->take_outputs();

  attachment_.complete(AttachmentOutcome::kRejected, "physical grasp tolerance was not met");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kOpenGripperForEscape) <<
    driver_->snapshot().transition->detail;
  ASSERT_TRUE(gripper_.outstanding());
  EXPECT_DOUBLE_EQ(gripper_.submission().goal.target_position_m, closed.open_joint_position_m);
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened for the escape");
  now_ += 1ms;
  driver_->pump(now_);

  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanRetreat) << receipts;
  EXPECT_FALSE(driver_->snapshot().inhibited) << receipts;
  ASSERT_TRUE(motion_.outstanding()) << receipts;
  const auto & goal = motion_.submission().goal;
  ASSERT_TRUE(goal.grasp_escape);
  EXPECT_TRUE(
    goal.grasp_escape->planning_frame_from_standoff_tool0.matrix().isApprox(
      closed.poses.world_from_pregrasp_tool0.matrix(), 1.0e-12));
  EXPECT_TRUE(goal.planning_frame_from_tool0.matrix().isApprox(survey.matrix(), 1.0e-12));
  EXPECT_TRUE(valid_motion_goal(goal));
  EXPECT_NE(
    receipts.find("the cleanup retreat is commandable under the whole-task deadline"),
    std::string::npos) << receipts;
}

// Card 062 review N3: a hold that never reached the gripper leaves nothing to open. The escape
// open is a receipted no-op instead of a failure that would burn the ladder down to the operator,
// and nothing that follows carries an escape.
TEST_F(RestockCoordinatorDriverTest, AHoldThatNeverReachedTheGripperNeedsNoEscape)
{
  reach_pregrasp_with_motion();
  ASSERT_TRUE(motion_.outstanding());
  motion_.complete(MotionOutcome::kSucceeded, "pre-grasp reached");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(gripper_.outstanding());
  gripper_.complete(GripperOutcome::kSucceeded, "jaws opened");
  now_ += 1ms;
  driver_->pump(now_);
  ASSERT_TRUE(motion_.outstanding());
  // Every hold submission this attempt makes is refused, so the jaws never close.
  gripper_.refuse_next_n(2U, GripperSubmitStatus::kBusy, "gripper backend busy");
  motion_.complete(MotionOutcome::kSucceeded, "approach reached");
  now_ += 1ms;
  driver_->pump(now_);
  for (int pump = 0; pump < 4 &&
    driver_->snapshot().transition->state == RestockTaskState::kCloseGripper; ++pump)
  {
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kRecover) <<
    driver_->snapshot().transition->detail;
  const auto attempts = gripper_.submit_attempts;
  for (int pump = 0; pump < 4 && port_.snapshots.empty(); ++pump) {
    now_ += 1ms;
    driver_->pump(now_);
  }
  ASSERT_EQ(port_.snapshots.size(), 1U);
  auto observed = reserved_world_snapshot(*reserve_response_);
  observed.revision = ++observation_revision_;
  port_.complete_front<WorldStateCoordinatorPort::GetSnapshot>(
    port_.snapshots, snapshot_response(observed));
  now_ += 1ms;
  driver_->pump(now_);

  std::string receipts;
  for (const auto & output : driver_->take_outputs()) {
    receipts += output.detail + "\n";
  }
  EXPECT_FALSE(driver_->snapshot().inhibited) << receipts;
  EXPECT_EQ(gripper_.submit_attempts, attempts) << "no jaw motion for jaws that never closed";
  EXPECT_NE(receipts.find("no hold was ever submitted at this grasp"), std::string::npos) <<
    receipts;
  ASSERT_EQ(driver_->snapshot().transition->state, RestockTaskState::kPlanPreGrasp) << receipts;
  ASSERT_TRUE(motion_.outstanding());
  EXPECT_FALSE(motion_.submission().goal.grasp_escape);
}

}  // namespace
}  // namespace restocker_task_executor
