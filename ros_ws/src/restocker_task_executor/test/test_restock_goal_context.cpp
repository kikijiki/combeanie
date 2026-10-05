// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/restock_goal_context.hpp"
#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_world_state::FaultState;
using restocker_world_state::GraspState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationOutcome;
using restocker_world_state::ReservationStage;
using restocker_world_state::TaskPhase;
using restocker_world_state::TrackingState;

CoordinatorGoalId goal_id(std::uint8_t seed = 1)
{
  CoordinatorGoalId id{};
  for (std::size_t index = 0; index < id.size(); ++index) {
    id[index] = static_cast<std::uint8_t>(seed + index);
  }
  return id;
}

restocker_world_state::WorldStateSnapshot snapshot(std::uint64_t revision = 10)
{
  restocker_world_state::WorldStateSnapshot value;
  value.revision = revision;
  value.robot.telemetry_time = rclcpp::Time(
    std::int64_t{1'000'000'000}, RCL_ROS_TIME);
  value.robot.telemetry_source_id = "test/goal-context";
  value.robot.telemetry_revision = revision - 1U;
  value.robot.revision = revision - 1U;
  value.robot.task_phase = TaskPhase::Idle;
  value.robot.fault_state = FaultState::None;

  restocker_world_state::TrackedObject object;
  object.id = ObjectId{17};
  object.source_object_id = "sim:can_17";
  object.product_class = ProductClass::Can;
  object.orientation = ObjectOrientation::Upright;
  object.tracking_state = TrackingState::Tracked;
  object.grasp_state = GraspState::Free;
  object.observation_time = rclcpp::Time(
    std::int64_t{1'000'000'000}, RCL_ROS_TIME);
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

SelectedTaskPair selection(const restocker_world_state::WorldStateSnapshot & value)
{
  return {
    ObjectId{17}, LaneId{"lane_01"}, value.revision,
    value.objects.at(ObjectId{17}).revision, value.lanes.at(LaneId{"lane_01"}).revision,
    {0.033, 0.122}};
}

// The shipped coordinator configures three approach yaws; tests default to one candidate and ask
// for more when choosing between candidates.
GraspGenerationConfig grasp_config(
  const restocker_world_state::WorldStateSnapshot & value,
  std::vector<double> yaws = {0.0})
{
  return GraspGenerationConfig{
    std::move(yaws), 0.0, 0.080, 0.10, 0.12, 0.005, 0.001, 0.035,
    value.robot.rail_position, 1.0, 1.0, 1.0, 0.05};
}

GraspGenerationAuthority grasp_authority(
  const restocker_world_state::WorldStateSnapshot & value,
  std::vector<double> yaws = {0.0})
{
  return GraspGenerationAuthority{
    ParallelJawGeometry{0.042, 0.0, 0.035}, Eigen::Isometry3d::Identity(),
    grasp_config(value, std::move(yaws))};
}

GraspCandidateBatch grasp_batch(
  const restocker_world_state::WorldStateSnapshot & value,
  std::vector<double> yaws = {0.0})
{
  const auto authority = grasp_authority(value, std::move(yaws));
  const auto generated = generate_grasp_candidate_batch(
    value, selection(value), authority.gripper,
    authority.tool0_from_grasp_center, authority.config,
    rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  if (!generated) {
    throw std::runtime_error(generated.error().detail);
  }
  return generated.value();
}

// The two yaws these tests choose between: first and second in the shipped configuration.
const std::vector<double> & two_yaws()
{
  static const std::vector<double> yaws{0.0, 0.5 * std::acos(-1.0)};
  return yaws;
}

std::shared_ptr<CoordinatorGenerationQuiescence> generation_quiescence()
{
  return std::make_shared<CoordinatorGenerationQuiescence>(goal_id(), 4U, 1U);
}

RestockGoalContext make_context(
  SelectionRequest request = {},
  std::shared_ptr<CoordinatorGenerationQuiescence> gate = {})
{
  if (!gate) {
    gate = generation_quiescence();
  }
  return RestockGoalContext(
    RestockGoalContextInit{
        goal_id(), 4, std::move(request), SteadyTime{},
        rclcpp::Time(std::int64_t{5'000'000'000}, RCL_ROS_TIME), {}, std::move(gate)});
}

TaskReservationCapability capability_for(
  const restocker_interfaces::srv::ReserveTask::Request & request)
{
  TaskReservationCapability capability;
  capability.token = "private-capability";
  capability.world_revision = request.selected_snapshot_revision + 1U;
  capability.reservation.reservation_id = 71;
  capability.reservation.request_id = request.request_id;
  capability.reservation.object_id = ObjectId{request.object_id};
  capability.reservation.object_source_id = "sim:can_17";
  capability.reservation.product_class = ProductClass::Can;
  capability.reservation.destination_lane = LaneId{request.destination_lane_id};
  // What the store would have captured from the selected snapshot's destination at grant.
  capability.reservation.destination_expected_product_class = ProductClass::Can;
  capability.reservation.stage = ReservationStage::Reserved;
  capability.reservation.created_revision = capability.world_revision;
  capability.reservation.admitted_robot_telemetry_revision = capability.world_revision - 1;
  capability.reservation.revision = capability.world_revision;
  return capability;
}

restocker_interfaces::srv::ReserveTask::Response reserve_response_for(
  const restocker_interfaces::srv::ReserveTask::Request & request)
{
  const auto capability = capability_for(request);
  restocker_interfaces::srv::ReserveTask::Response response;
  response.status = restocker_world_state::operation_status_ok();
  response.world_revision = capability.world_revision;
  response.token = capability.token;
  response.reservation = restocker_world_state::task_reservation_to_message(
    capability.reservation);
  return response;
}

restocker_interfaces::srv::ValidateTaskReservation::Response validation_response_for(
  const TaskReservationCapability & capability)
{
  restocker_interfaces::srv::ValidateTaskReservation::Response response;
  response.status = restocker_world_state::operation_status_ok();
  response.world_revision = capability.world_revision;
  response.has_reservation = true;
  response.reservation = restocker_world_state::task_reservation_to_message(
    capability.reservation);
  return response;
}

void drive_to_generate_grasps(
  RestockGoalContext & context,
  const restocker_world_state::WorldStateSnapshot & value)
{
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  ASSERT_TRUE(context.retain_initial_snapshot(value));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);
  ASSERT_TRUE(context.retain_selection(selection(value)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);
  const auto request = make_reserve_task_request(
    value, *context.selection(), "grasp-reserve");
  ASSERT_TRUE(request);
  ASSERT_TRUE(context.retain_reserve_request(request.value()));
  ASSERT_TRUE(context.retain_reserve_response(reserve_response_for(request.value())));
  ASSERT_TRUE(
    context.retain_reservation_validation_response(
      validation_response_for(*context.reservation())));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kGenerateGrasps);
}

void drive_to_plan_pregrasp(
  RestockGoalContext & context,
  const restocker_world_state::WorldStateSnapshot & value,
  std::vector<double> yaws = {0.0})
{
  drive_to_generate_grasps(context, value);
  ASSERT_TRUE(
    context.retain_grasp_candidate_batch(
      grasp_batch(value, yaws), grasp_authority(value, yaws),
      rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanPreGrasp);
}

void complete_execution_state(
  RestockGoalContext & context, RestockTaskCommand command, std::uint64_t generation)
{
  ASSERT_EQ(context.task_status().command, command);
  ASSERT_TRUE(context.prepare_execution_operation_id(command));
  ASSERT_TRUE(
    context.dispatch_task(
      RestockTaskEvent::kExecutionOperationStarted, {}, generation).accepted);
  ASSERT_TRUE(
    context.dispatch_task(
      RestockTaskEvent::kTrajectoryExecutionAccepted, {}, generation).accepted);
}

// Everything between a planned pre-grasp and grasp verification. The executed candidate is fixed
// once this runs, so tests about choosing one must interpose before it.
void advance_pregrasp_to_verify_grasp(RestockGoalContext & context)
{
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecutePreGrasp);
  complete_execution_state(context, RestockTaskCommand::kExecutePreGrasp, 11U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kOpenGripperForApproach);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanApproach);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteApproach);
  complete_execution_state(context, RestockTaskCommand::kExecuteApproach, 12U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kCloseGripper);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kVerifyGrasp);
}

void drive_to_verify_grasp(
  RestockGoalContext & context,
  const restocker_world_state::WorldStateSnapshot & value,
  std::vector<double> yaws = {0.0})
{
  drive_to_plan_pregrasp(context, value, std::move(yaws));
  advance_pregrasp_to_verify_grasp(context);
}

// Every state between grasp verification and the retreat plan, walked by their own success
// transitions: attach → retract → observe → placement → carry → insert → release → detach.
void drive_to_plan_retreat(RestockGoalContext & context)
{
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kAttachTransaction);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanRetract);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteRetract);
  complete_execution_state(context, RestockTaskCommand::kExecuteRetract, 21U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kObserveDestination);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kGeneratePlacement);
  // From placement generation onward the guards require the retained destination pose the
  // driver would have generated; identity must match the reserved pair.
  PlacementCandidate placement;
  placement.selection = *context.selection();
  ASSERT_TRUE(context.retain_placement_candidate(std::move(placement)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanCarryStart);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteCarryStart);
  complete_execution_state(context, RestockTaskCommand::kExecuteCarryStart, 22U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanPreInsert);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecutePreInsert);
  complete_execution_state(context, RestockTaskCommand::kExecutePreInsert, 23U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanInsert);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteInsert);
  complete_execution_state(context, RestockTaskCommand::kExecuteInsert, 24U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kOpenGripper);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kDetachTransaction);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanRetreat);
}

restocker_world_state::WorldStateSnapshot reserved_snapshot(
  const RestockGoalContext & context,
  const restocker_world_state::WorldStateSnapshot & staged)
{
  auto value = staged;
  value.revision = context.reservation()->world_revision;
  value.robot.telemetry_revision =
    context.reservation()->reservation.admitted_robot_telemetry_revision;
  value.robot.revision = value.robot.telemetry_revision;
  value.robot.telemetry_time = rclcpp::Time(
    std::int64_t{1'050'000'000}, RCL_ROS_TIME);
  value.active_reservation = context.reservation()->reservation;
  return value;
}

TEST(RestockGoalContext, RejectsMalformedImmutableIdentity)
{
  auto init = RestockGoalContextInit{};
  EXPECT_THROW((void)RestockGoalContext{init}, std::invalid_argument);

  init.goal_id = goal_id();
  init.generation = 1;
  init.selection_request.object_id = ObjectId{};
  EXPECT_THROW((void)RestockGoalContext{init}, std::invalid_argument);

  init.selection_request.object_id.reset();
  init.selection_request.lane_id = LaneId{};
  EXPECT_THROW((void)RestockGoalContext{init}, std::invalid_argument);
}

TEST(RestockGoalContext, RetainsExactGenerationQuiescenceOwnership)
{
  auto missing_gate = RestockGoalContextInit{};
  missing_gate.goal_id = goal_id();
  missing_gate.generation = 4U;
  EXPECT_THROW((void)RestockGoalContext{std::move(missing_gate)}, std::invalid_argument);

  auto gate = generation_quiescence();
  const auto * gate_identity = gate.get();
  std::weak_ptr<CoordinatorGenerationQuiescence> weak_gate = gate;
  {
    auto context = make_context({}, std::move(gate));
    EXPECT_EQ(context.generation_quiescence().get(), gate_identity);
    EXPECT_FALSE(weak_gate.expired());
  }
  EXPECT_TRUE(weak_gate.expired());
}

// Card 044 / Milestone 10 §6: the retreat-boundary refusal must name the guard that refused the
// execution identity, so a deadline-driven refusal can never read as a lost operation identity.
// Fail-closed behaviour itself is unchanged: once termination is latched, no identity is minted.
TEST(RestockGoalContext, ExecutionIdentityRefusalNamesTheLatchedTermination)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_plan_pregrasp(context, value);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecutePreGrasp);
  ASSERT_EQ(context.task_status().command, RestockTaskCommand::kExecutePreGrasp);

  // A wrong-segment request names the state it found instead of silently refusing.
  std::string wrong_segment;
  EXPECT_FALSE(
    context.prepare_execution_operation_id(
      RestockTaskCommand::kExecuteRetreat, &wrong_segment));
  EXPECT_NE(wrong_segment.find("the task machine is in execute_pre_grasp"), std::string::npos)
    << wrong_segment;

  // The whole-task deadline latches termination while the machine sits on the execute state —
  // the shape of run 7's expiry mid-retreat, replayed at the context boundary.
  const auto expired = context.dispatch_task(RestockTaskEvent::kTaskDeadlineExceeded);
  ASSERT_TRUE(expired.cancel_requested);
  ASSERT_EQ(expired.state, RestockTaskState::kExecutePreGrasp);

  std::string refusal;
  EXPECT_FALSE(
    context.prepare_execution_operation_id(RestockTaskCommand::kExecutePreGrasp, &refusal));
  EXPECT_EQ(refusal, "termination is already latched");
  // The refusal must not have minted anything for this attempt.
  EXPECT_EQ(
    context.retained_execution_operation_id(
      RestockTaskCommand::kExecutePreGrasp, expired.recovery_attempt, expired.attempt),
    nullptr);
}

// Milestone 10 §6 (Card 051): the whole-task deadline's bounded cleanup retreat is the one
// execution a latched termination admits. The identity is minted (and retained), so the guard
// recognises the exit rather than being silenced.
TEST(RestockGoalContext, WholeTaskDeadlineAdmitsTheCleanupRetreatIdentity)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  drive_to_plan_retreat(context);

  const auto expired = context.dispatch_task(RestockTaskEvent::kTaskDeadlineExceeded);
  ASSERT_TRUE(expired.cancel_requested);
  ASSERT_TRUE(expired.task_deadline_exceeded);
  // The in-flight plan completes under termination: the machine enters execute-retreat.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteRetreat);

  std::string refusal;
  const auto admitted = context.prepare_execution_operation_id(
    RestockTaskCommand::kExecuteRetreat, &refusal);
  ASSERT_TRUE(admitted.has_value()) << "the cleanup retreat must be admitted: " << refusal;
  // Minted once and retained for the same (recovery, attempt) key.
  EXPECT_EQ(
    context.prepare_execution_operation_id(RestockTaskCommand::kExecuteRetreat, nullptr),
    admitted);
}

// A user cancel keeps stopping in place: even the retreat identity is refused, with Card 044's
// guard named (Milestone 10 §6, Card 051 — the deadline retreat is commandable only under the
// kTaskDeadline termination).
TEST(RestockGoalContext, UserCancelStillRefusesTheRetreatIdentity)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  drive_to_plan_retreat(context);

  const auto canceled = context.dispatch_task(RestockTaskEvent::kCancelRequested);
  ASSERT_TRUE(canceled.cancel_requested);
  ASSERT_FALSE(canceled.task_deadline_exceeded);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteRetreat);

  std::string refusal;
  EXPECT_FALSE(
    context.prepare_execution_operation_id(RestockTaskCommand::kExecuteRetreat, &refusal));
  EXPECT_EQ(refusal, "termination is already latched");
}

// Card 051 review blocker, both orderings: a user cancel revokes any cleanup-retreat authority
// the whole-task deadline carries or later receives — the identity is refused, so the arm
// never executes a retreat after an operator asked it to stop. (The driver's receipt-only
// dispatch guard means the deadline event never arrives after a cancel; this pins the machine
// side as defence in depth, and the reverse order where the deadline armed first.)
TEST(RestockGoalContext, UserCancelThenDeadlineNeverAdmitsTheCleanupRetreatIdentity)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  drive_to_plan_retreat(context);

  const auto canceled = context.dispatch_task(RestockTaskEvent::kCancelRequested, "stop");
  ASSERT_TRUE(canceled.cancel_requested);
  // Even if a deadline dispatch slipped through the driver's receipt-only guard, the machine
  // records it but the revoked flag keeps the identity refused.
  const auto expired = context.dispatch_task(RestockTaskEvent::kTaskDeadlineExceeded, "deadline");
  ASSERT_TRUE(expired.accepted);
  EXPECT_TRUE(expired.deadline_retreat_revoked);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteRetreat);

  std::string refusal;
  EXPECT_FALSE(
    context.prepare_execution_operation_id(RestockTaskCommand::kExecuteRetreat, &refusal));
  EXPECT_EQ(refusal, "termination is already latched");
}

TEST(RestockGoalContext, DeadlineThenUserCancelRevokesTheCleanupRetreatIdentity)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  drive_to_plan_retreat(context);

  // The deadline arms the retreat authority first (the approved Card 051 path) …
  const auto expired = context.dispatch_task(RestockTaskEvent::kTaskDeadlineExceeded, "deadline");
  ASSERT_TRUE(expired.accepted);
  EXPECT_TRUE(expired.task_deadline_exceeded);
  EXPECT_FALSE(expired.deadline_retreat_revoked);
  // … then the operator cancels: the revocation wins for motion purposes.
  const auto canceled = context.dispatch_task(RestockTaskEvent::kCancelRequested, "stop");
  ASSERT_TRUE(canceled.cancel_requested);
  EXPECT_TRUE(canceled.deadline_retreat_revoked);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecuteRetreat);

  std::string refusal;
  EXPECT_FALSE(
    context.prepare_execution_operation_id(RestockTaskCommand::kExecuteRetreat, &refusal));
  EXPECT_EQ(refusal, "termination is already latched");
}

TEST(RestockGoalContext, OwnsMachineAndExactReservationLineage)
{
  auto context = make_context();
  EXPECT_EQ(context.goal_id(), goal_id());
  EXPECT_EQ(context.generation(), 4U);
  EXPECT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);

  const auto selected_snapshot = snapshot();
  ASSERT_TRUE(context.retain_initial_snapshot(selected_snapshot));
  EXPECT_EQ(context.initial_snapshot()->revision, 10U);
  EXPECT_EQ(context.latest_snapshot()->revision, 10U);
  EXPECT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);

  const auto selected = selection(selected_snapshot);
  ASSERT_TRUE(context.retain_selection(selected));
  EXPECT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);

  const auto reserve_id = context.operation_id_for(CoordinatorMutationKind::kReserveTask);
  ASSERT_TRUE(reserve_id);
  EXPECT_EQ(context.operation_id_for(CoordinatorMutationKind::kReserveTask), reserve_id);
  const auto request = make_reserve_task_request(selected_snapshot, selected, *reserve_id);
  ASSERT_TRUE(request) << request.error().detail;
  ASSERT_TRUE(context.retain_reserve_request(request.value()));
  const auto reserve_response = reserve_response_for(request.value());
  ASSERT_TRUE(context.retain_reserve_response(reserve_response));
  EXPECT_TRUE(context.retain_reserve_response(reserve_response));
  EXPECT_EQ(context.reservation()->reservation.object_id, ObjectId{17});

  EXPECT_TRUE(context.dispatch_task(RestockTaskEvent::kCancelRequested).cancel_requested);
  EXPECT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReleaseTask);

  const auto release_id = context.operation_id_for(CoordinatorMutationKind::kReleaseTask);
  ASSERT_TRUE(release_id);
  EXPECT_EQ(context.operation_id_for(CoordinatorMutationKind::kReleaseTask), release_id);
  const auto release = make_release_task_reservation_request(
    *context.reservation(), *release_id, ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(release) << release.error().detail;
  ASSERT_TRUE(context.retain_release_request(release.value()));
  auto still_reserved = selected_snapshot;
  still_reserved.revision = context.reservation()->world_revision;
  still_reserved.robot.telemetry_revision =
    context.reservation()->reservation.admitted_robot_telemetry_revision;
  still_reserved.robot.revision = still_reserved.revision;
  still_reserved.active_reservation = context.reservation()->reservation;
  const auto active_readback = context.retain_release_readback(still_reserved);
  ASSERT_TRUE(active_readback) << active_readback.detail;
  EXPECT_EQ(
    active_readback.disposition, ReleaseReadbackDisposition::kExactReservationStillActive);
  restocker_interfaces::srv::ReleaseTaskReservation::Response release_response;
  release_response.status = restocker_world_state::operation_status_ok();
  release_response.world_revision = context.reservation()->world_revision + 1U;
  ASSERT_TRUE(context.retain_release_response(release_response));
  EXPECT_TRUE(context.retain_release_response(release_response));
  const auto stale_readback = context.retain_release_readback(std::move(still_reserved));
  EXPECT_EQ(stale_readback.error, GoalContextErrorCode::kRevisionRegression);
  const auto premature_release = context.dispatch_task(RestockTaskEvent::kOperationSucceeded);
  EXPECT_FALSE(premature_release.accepted);
  EXPECT_EQ(premature_release.state, RestockTaskState::kReleaseTask);

  auto contradictory_release = selected_snapshot;
  contradictory_release.revision = release_response.world_revision + 8U;
  contradictory_release.robot.telemetry_revision =
    context.reservation()->reservation.admitted_robot_telemetry_revision;
  contradictory_release.robot.revision = contradictory_release.revision;
  contradictory_release.active_reservation = context.reservation()->reservation;
  EXPECT_EQ(
    context.retain_released_snapshot(std::move(contradictory_release)).error,
    GoalContextErrorCode::kIdentityMismatch);
  ASSERT_TRUE(context.latest_snapshot());
  EXPECT_EQ(context.latest_snapshot()->revision, reserve_response.world_revision);

  auto released = selected_snapshot;
  released.revision = release_response.world_revision;
  released.robot.revision = released.revision;
  ASSERT_TRUE(context.retain_released_snapshot(std::move(released)));
  ASSERT_TRUE(context.release_proof());
  ASSERT_TRUE(context.reservation());
  EXPECT_TRUE(context.reservation()->token.empty());
  EXPECT_TRUE(context.release_request()->token.empty());
  EXPECT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kCanceled);
}

TEST(RestockGoalContext, RejectsCausallyReorderedReservationEvidence)
{
  auto context = make_context();
  const auto selected_snapshot = snapshot();
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  ASSERT_TRUE(context.retain_initial_snapshot(selected_snapshot));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);
  ASSERT_TRUE(context.retain_selection(selection(selected_snapshot)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);

  const auto request = make_reserve_task_request(
    selected_snapshot, *context.selection(), "causal-reserve");
  ASSERT_TRUE(request);
  ASSERT_TRUE(context.retain_reserve_request(request.value()));
  ASSERT_TRUE(context.retain_reserve_response(reserve_response_for(request.value())));
  EXPECT_EQ(
    context.retain_latest_snapshot(selected_snapshot).error,
    GoalContextErrorCode::kRevisionRegression);

  auto newer_snapshot = snapshot(context.reservation()->world_revision + 1U);
  ASSERT_TRUE(context.retain_latest_snapshot(newer_snapshot));
  auto delayed_validation = validation_response_for(*context.reservation());
  EXPECT_EQ(
    context.retain_reservation_validation_response(delayed_validation).error,
    GoalContextErrorCode::kRevisionRegression);

  delayed_validation.world_revision = newer_snapshot.revision;
  ASSERT_TRUE(context.retain_reservation_validation_response(delayed_validation));
  auto stale_snapshot = snapshot(newer_snapshot.revision - 1U);
  EXPECT_EQ(
    context.retain_latest_snapshot(std::move(stale_snapshot)).error,
    GoalContextErrorCode::kRevisionRegression);
}

TEST(RestockGoalContext, RejectsPhaseIdentityAndRevisionMismatches)
{
  auto context = make_context();
  const auto selected_snapshot = snapshot();
  EXPECT_EQ(
    context.retain_initial_snapshot(selected_snapshot).error,
    GoalContextErrorCode::kInvalidPhase);
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);

  auto malformed = selected_snapshot;
  malformed.robot.revision = malformed.revision + 1U;
  EXPECT_EQ(
    context.retain_initial_snapshot(malformed).error,
    GoalContextErrorCode::kInvalidInput);
  ASSERT_TRUE(context.retain_initial_snapshot(selected_snapshot));
  EXPECT_EQ(
    context.retain_initial_snapshot(selected_snapshot).error,
    GoalContextErrorCode::kAlreadySet);

  auto older = snapshot(9);
  EXPECT_EQ(
    context.retain_latest_snapshot(older).error,
    GoalContextErrorCode::kRevisionRegression);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);

  auto mismatched_selection = selection(selected_snapshot);
  ++mismatched_selection.object_revision;
  EXPECT_EQ(
    context.retain_selection(mismatched_selection).error,
    GoalContextErrorCode::kIdentityMismatch);
  ASSERT_TRUE(context.retain_selection(selection(selected_snapshot)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);

  const auto expected = make_reserve_task_request(
    selected_snapshot, *context.selection(), "reserve-exact");
  ASSERT_TRUE(expected);
  auto changed = expected.value();
  ++changed.object_revision;
  EXPECT_EQ(
    context.retain_reserve_request(changed).error,
    GoalContextErrorCode::kIdentityMismatch);
  ASSERT_TRUE(context.retain_reserve_request(expected.value()));

  auto substituted = reserve_response_for(expected.value());
  substituted.reservation.destination_lane_id = "lane_02";
  EXPECT_EQ(
    context.retain_reserve_response(substituted).error,
    GoalContextErrorCode::kIdentityMismatch);
  ASSERT_TRUE(context.retain_reserve_response(reserve_response_for(expected.value())));
}

TEST(RestockGoalContext, RejectsCandidateBatchWithContradictoryLineage)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_generate_grasps(context, value);

  auto batch = grasp_batch(value);
  batch.source_robot_telemetry_revision += 1U;
  const auto retained = context.retain_grasp_candidate_batch(
    std::move(batch), grasp_authority(value),
    rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  EXPECT_EQ(retained.error, GoalContextErrorCode::kIdentityMismatch);
  EXPECT_FALSE(context.grasp_candidate_batch());

  const auto abort = context.dispatch_task(RestockTaskEvent::kSafeAbortRequested);
  EXPECT_TRUE(abort.accepted);
  EXPECT_TRUE(abort.safe_abort_requested);
  EXPECT_FALSE(abort.cancel_requested);
  EXPECT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReleaseTask);
}

TEST(RestockGoalContext, RejectsEveryCandidateMutationAtTheRetentionBoundary)
{
  using Mutation = std::pair<std::string, std::function<void (GraspCandidateBatch &)>>;
  const std::vector<Mutation> mutations{
    {"yaw index", [](auto & batch) {batch.candidates.front().source_yaw_index = 9U;}},
    {"yaw value", [](auto & batch) {batch.candidates.front().approach_yaw_rad += 0.1;}},
    {"open jaw", [](auto & batch) {batch.candidates.front().open_joint_position_m -= 0.001;}},
    {"hold jaw", [](auto & batch) {batch.candidates.front().hold_joint_position_m += 0.001;}},
    {"grasp pose", [](auto & batch) {
        batch.candidates.front().poses.world_from_grasp_center.translation().x() += 0.001;
      }},
    {"tool pose", [](auto & batch) {
        batch.candidates.front().poses.world_from_grasp_tool0.translation().y() += 0.001;
      }},
    {"score", [](auto & batch) {batch.candidates.front().score.total += 0.001;}},
    {"ordering", [](auto & batch) {
        std::swap(batch.candidates.front(), batch.candidates.back());
      }},
  };
  const auto value = snapshot();
  auto authority = grasp_authority(value);
  authority.config.approach_yaws_rad.push_back(0.5 * std::acos(-1.0));
  const auto generated = generate_grasp_candidate_batch(
    value, selection(value), authority.gripper, authority.tool0_from_grasp_center,
    authority.config, rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(generated) << generated.error().detail;
  ASSERT_EQ(generated.value().candidates.size(), 2U);

  for (const auto & [name, mutate] : mutations) {
    auto context = make_context();
    drive_to_generate_grasps(context, value);
    auto batch = generated.value();
    mutate(batch);
    const auto retained = context.retain_grasp_candidate_batch(
      std::move(batch), authority,
      rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50));
    EXPECT_EQ(retained.error, GoalContextErrorCode::kIdentityMismatch) << name;
    ASSERT_TRUE(retained.grasp_error.has_value()) << name;
    EXPECT_EQ(*retained.grasp_error, GraspCandidateErrorCode::InvalidCandidateBatch) << name;
    EXPECT_FALSE(context.grasp_candidate_batch()) << name;
  }
}

TEST(RestockGoalContext, SafeAbortReleasesCapabilityWithoutReservationReadback)
{
  auto context = make_context();
  const auto selected_snapshot = snapshot();
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  ASSERT_TRUE(context.retain_initial_snapshot(selected_snapshot));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);
  ASSERT_TRUE(context.retain_selection(selection(selected_snapshot)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);

  const auto reserve_id = context.operation_id_for(CoordinatorMutationKind::kReserveTask);
  ASSERT_TRUE(reserve_id);
  const auto reserve = make_reserve_task_request(
    selected_snapshot, *context.selection(), *reserve_id);
  ASSERT_TRUE(reserve);
  ASSERT_TRUE(context.retain_reserve_request(reserve.value()));
  ASSERT_TRUE(context.retain_reserve_response(reserve_response_for(reserve.value())));
  ASSERT_FALSE(context.reservation_validation());

  const auto abort = context.dispatch_task(RestockTaskEvent::kSafeAbortRequested);
  ASSERT_TRUE(abort.accepted);
  EXPECT_EQ(abort.fault, RestockTaskFault::kOperationFailed);
  EXPECT_FALSE(abort.cancel_requested);
  EXPECT_TRUE(abort.safe_abort_requested);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReleaseTask);

  const auto release_id = context.operation_id_for(CoordinatorMutationKind::kReleaseTask);
  ASSERT_TRUE(release_id);
  const auto release = make_release_task_reservation_request(
    *context.reservation(), *release_id, ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::Recoverable);
  ASSERT_TRUE(release);
  ASSERT_TRUE(context.retain_release_request(release.value()));
  restocker_interfaces::srv::ReleaseTaskReservation::Response release_response;
  release_response.status = restocker_world_state::operation_status_ok();
  release_response.world_revision = context.reservation()->world_revision + 1U;
  ASSERT_TRUE(context.retain_release_response(release_response));
  auto released = selected_snapshot;
  released.revision = release_response.world_revision;
  released.robot.revision = released.revision;
  released.robot.task_phase = TaskPhase::Fault;
  released.robot.fault_state = FaultState::Recoverable;
  const auto proof = context.retain_released_snapshot(std::move(released));
  ASSERT_TRUE(proof) << proof.detail;

  const auto terminal = context.dispatch_task(RestockTaskEvent::kOperationSucceeded);
  EXPECT_EQ(terminal.state, RestockTaskState::kCanceled);
  EXPECT_EQ(terminal.fault, RestockTaskFault::kOperationFailed);
  EXPECT_FALSE(terminal.cancel_requested);
  EXPECT_TRUE(terminal.safe_abort_requested);
  EXPECT_FALSE(terminal.reservation_active);
}

TEST(RestockGoalContext, EnforcesEveryExplicitSelectorCombination)
{
  const auto selection_error = [](SelectionRequest request) {
    auto context = make_context(std::move(request));
    const auto value = snapshot();
    EXPECT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
    EXPECT_TRUE(context.retain_initial_snapshot(value));
    EXPECT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kSelectPair);
    return context.retain_selection(selection(value)).error;
  };

  EXPECT_EQ(
    selection_error(SelectionRequest{ObjectId{99}, std::nullopt}),
    GoalContextErrorCode::kIdentityMismatch);
  EXPECT_EQ(
    selection_error(SelectionRequest{std::nullopt, LaneId{"lane_02"}}),
    GoalContextErrorCode::kIdentityMismatch);
  EXPECT_EQ(
    selection_error(SelectionRequest{ObjectId{17}, LaneId{"lane_02"}}),
    GoalContextErrorCode::kIdentityMismatch);
  EXPECT_EQ(
    selection_error(SelectionRequest{ObjectId{17}, LaneId{"lane_01"}}),
    GoalContextErrorCode::kNone);
}

TEST(RestockGoalContext, RejectsContradictorySameRevisionSnapshot)
{
  auto context = make_context();
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  const auto initial = snapshot();
  ASSERT_TRUE(context.retain_initial_snapshot(initial));
  EXPECT_TRUE(context.retain_latest_snapshot(initial));

  auto contradiction = initial;
  contradiction.objects.at(ObjectId{17}).source_object_id = "substituted:can";
  EXPECT_EQ(
    context.retain_latest_snapshot(std::move(contradiction)).error,
    GoalContextErrorCode::kIdentityMismatch);
  EXPECT_EQ(
    context.latest_snapshot()->objects.at(ObjectId{17}).source_object_id,
    "sim:can_17");
}

TEST(RestockGoalContext, GeneratesStableBoundedGoalScopedMutationIds)
{
  auto first = make_context();
  auto second = make_context();
  EXPECT_FALSE(first.operation_id_for(CoordinatorMutationKind::kReserveTask));
  const auto advance_to_reserve = [](RestockGoalContext & context) {
    const auto value = snapshot();
    EXPECT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
    EXPECT_TRUE(context.retain_initial_snapshot(value));
    EXPECT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kSelectPair);
    EXPECT_TRUE(context.retain_selection(selection(value)));
    EXPECT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kReserveTask);
  };
  advance_to_reserve(first);
  advance_to_reserve(second);
  const auto reserve = first.operation_id_for(CoordinatorMutationKind::kReserveTask);
  const auto repeated = first.operation_id_for(CoordinatorMutationKind::kReserveTask);
  const auto repeated_goal = second.operation_id_for(CoordinatorMutationKind::kReserveTask);
  ASSERT_TRUE(reserve);
  ASSERT_TRUE(repeated);
  ASSERT_TRUE(repeated_goal);
  EXPECT_EQ(*reserve, *repeated);
  EXPECT_EQ(*reserve, *repeated_goal);
  EXPECT_LT(reserve->size(), 128U);
  EXPECT_NE(reserve->find("reserve-task/1"), std::string::npos);
  EXPECT_FALSE(first.operation_id_for(CoordinatorMutationKind::kReleaseTask));
}

restocker_interfaces::srv::ReserveTask::Response rejected_reserve_response(std::uint16_t code)
{
  restocker_interfaces::srv::ReserveTask::Response response;
  response.status.code = code;
  response.status.detail = "selected snapshot entity revisions changed";
  return response;
}

// The world state republishes entities at 10 Hz, so revisions routinely move between snapshot
// and reservation. That mutation-free rejection must discard the lineage and re-select, not latch
// a lineage violation.
TEST(RestockGoalContext, DiscardsSupersededSelectionOnNonMutatingReserveRejection)
{
  for (const std::uint16_t code :
    {restocker_interfaces::msg::WorldStateOperationStatus::REVISION_CONFLICT,
      restocker_interfaces::msg::WorldStateOperationStatus::PREDICATE_FAILED})
  {
    auto context = make_context();
    const auto value = snapshot();
    ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
    ASSERT_TRUE(context.retain_initial_snapshot(value));
    ASSERT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kSelectPair);
    ASSERT_TRUE(context.retain_selection(selection(value)));
    ASSERT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kReserveTask);
    const auto stale_id = context.operation_id_for(CoordinatorMutationKind::kReserveTask);
    ASSERT_TRUE(stale_id);
    const auto stale_request = make_reserve_task_request(value, *context.selection(), *stale_id);
    ASSERT_TRUE(stale_request) << stale_request.error().detail;
    ASSERT_TRUE(context.retain_reserve_request(stale_request.value()));

    const auto superseded = context.retain_reserve_response(rejected_reserve_response(code));
    ASSERT_TRUE(superseded) << superseded.detail;
    EXPECT_EQ(superseded.disposition, ReserveResponseDisposition::kSelectionSuperseded);
    EXPECT_FALSE(context.selection());
    EXPECT_FALSE(context.reserve_request());
    EXPECT_FALSE(context.reservation());
    // The baseline snapshot outlives the discarded selection; only the selection lineage goes.
    EXPECT_TRUE(context.initial_snapshot());

    ASSERT_EQ(
      context.dispatch_task(RestockTaskEvent::kSelectionSuperseded).state,
      RestockTaskState::kValidateScene);

    // Re-select against fresher evidence and reserve again under a distinct operation identity.
    auto fresher = snapshot(14);
    ASSERT_TRUE(context.retain_latest_snapshot(fresher));
    ASSERT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kSelectPair);
    ASSERT_TRUE(context.retain_selection(selection(fresher)));
    ASSERT_EQ(
      context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
      RestockTaskState::kReserveTask);
    const auto fresh_id = context.operation_id_for(CoordinatorMutationKind::kReserveTask);
    ASSERT_TRUE(fresh_id);
    EXPECT_NE(*fresh_id, *stale_id);
    const auto fresh_request = make_reserve_task_request(
      fresher, *context.selection(), *fresh_id);
    ASSERT_TRUE(fresh_request) << fresh_request.error().detail;
    ASSERT_TRUE(context.retain_reserve_request(fresh_request.value()));
    ASSERT_TRUE(context.retain_reserve_response(reserve_response_for(fresh_request.value())));
    EXPECT_EQ(
      context.retain_reserve_response(reserve_response_for(fresh_request.value())).disposition,
      ReserveResponseDisposition::kCapabilityAcquired);
    ASSERT_TRUE(context.reservation());
  }
}

TEST(RestockGoalContext, KeepsFailClosedMismatchForMutatingReserveRejections)
{
  auto context = make_context();
  const auto value = snapshot();
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  ASSERT_TRUE(context.retain_initial_snapshot(value));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);
  ASSERT_TRUE(context.retain_selection(selection(value)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);
  const auto request = make_reserve_task_request(value, *context.selection(), "reserve-conflict");
  ASSERT_TRUE(request);
  ASSERT_TRUE(context.retain_reserve_request(request.value()));

  // A reservation conflict means someone holds a reservation: no re-select and no second reserve.
  const auto conflicted = context.retain_reserve_response(
    rejected_reserve_response(
      restocker_interfaces::msg::WorldStateOperationStatus::RESERVATION_CONFLICT));
  EXPECT_EQ(conflicted.error, GoalContextErrorCode::kIdentityMismatch);
  EXPECT_TRUE(context.selection());
  EXPECT_TRUE(context.reserve_request());
}

TEST(RestockGoalContext, RejectsSupersededDispatchWhileSelectionLineageIsRetained)
{
  auto context = make_context();
  const auto value = snapshot();
  ASSERT_EQ(context.begin_task().state, RestockTaskState::kValidateScene);
  ASSERT_TRUE(context.retain_initial_snapshot(value));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kSelectPair);
  ASSERT_TRUE(context.retain_selection(selection(value)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kReserveTask);

  const auto rejected = context.dispatch_task(RestockTaskEvent::kSelectionSuperseded);
  EXPECT_FALSE(rejected.accepted);
  EXPECT_EQ(context.task_status().state, RestockTaskState::kReserveTask);
}

// The coupling depends on how the jaws hold the product, so read it at the planned grasp pose;
// recomposing from a carried product and a planning-time pose mixes two instants.
TEST(RestockGoalContext, CapturesTheGraspCouplingFromTheVerificationSnapshot)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  EXPECT_FALSE(context.grasp_coupling());
  EXPECT_EQ(context.grasp_coupling_ordinal(), 0U);

  // Grasp verification binds the coupling to its fresh observation, not the candidate snapshot.
  auto verified = snapshot(13);
  verified.objects.at(ObjectId{17}).pose_in_world.translation().y() += 0.004;
  ASSERT_TRUE(context.retain_latest_snapshot(verified));

  ASSERT_TRUE(context.retain_grasp_coupling());
  ASSERT_TRUE(context.grasp_coupling());
  // Receipt: the first accepted verification of this goal is the one that owns the coupling.
  EXPECT_EQ(context.grasp_coupling_ordinal(), 1U);
  const Eigen::Isometry3d expected =
    verified.objects.at(ObjectId{17}).pose_in_world.inverse() *
    context.grasp_candidate_batch()->candidates.front().poses.world_from_grasp_center;
  EXPECT_TRUE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      expected.matrix(), 1.0e-12));
  EXPECT_EQ(context.grasp_coupling()->source_world_revision, verified.revision);
  EXPECT_EQ(
    context.grasp_coupling()->source_object_revision,
    verified.objects.at(ObjectId{17}).revision);

  // The value is evidence about the verification that produced it, so a second verification of
  // the same snapshot and candidate composes the same value and changes nothing (Card 039).
  ASSERT_TRUE(context.retain_grasp_coupling());
  EXPECT_EQ(context.grasp_coupling_ordinal(), 2U);
  EXPECT_EQ(context.grasp_coupling()->source_world_revision, verified.revision);
  EXPECT_TRUE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      expected.matrix(), 1.0e-12));
}

// Card 039 SC-001: recovery from kCloseGripper/kVerifyGrasp re-enters kGenerateGrasps with the
// coupling a previous verification retained still in place, and the sequence then runs to a second
// kVerifyGrasp. The second verification must re-derive the coupling — this returned kAlreadySet
// before the Milestone 10 §1 contract, and the driver turned that into a terminal inhibition.
TEST(RestockGoalContext, ARecoveryReEntryReachesVerifyAgainAndReDerivesTheCoupling)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);

  // The first verification adopts fresher evidence and retains the coupling from it.
  auto first = snapshot(13);
  first.objects.at(ObjectId{17}).pose_in_world.translation().y() += 0.004;
  ASSERT_TRUE(context.retain_latest_snapshot(first));
  ASSERT_TRUE(context.retain_grasp_coupling());
  ASSERT_EQ(context.grasp_coupling_ordinal(), 1U);
  const Eigen::Isometry3d first_coupling =
    context.grasp_coupling()->product_from_grasp_center;

  // Two retryable failures at verify exhaust the operation retries, and bounded recovery
  // re-enters generate-grasps with the retained coupling intact.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kRetryableFailure, "verify evidence arrived late")
    .state, RestockTaskState::kVerifyGrasp);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kRetryableFailure, "verify evidence arrived late")
    .state, RestockTaskState::kRecover);
  ASSERT_EQ(context.task_status().recovery_attempt, 1U);
  // Card 062: the jaws closed on a product that is not held open before the grasp is left.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "recovery completed").state,
    RestockTaskState::kOpenGripperForEscape);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "jaws opened").state,
    RestockTaskState::kGenerateGrasps);
  ASSERT_TRUE(context.grasp_coupling());

  // Recovery rebinds the pair to the snapshot verify adopted, which drops the batch it staged
  // against the old world; the regenerated batch is what the second sequence runs.
  ASSERT_TRUE(context.rebind_selection_to_latest_snapshot());
  const auto authority = grasp_authority(*context.latest_snapshot());
  auto regenerated = generate_grasp_candidate_batch(
    *context.latest_snapshot(), *context.selection(), authority.gripper,
    authority.tool0_from_grasp_center, authority.config,
    rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(regenerated) << regenerated.error().detail;
  ASSERT_TRUE(
    context.retain_grasp_candidate_batch(
      regenerated.value(), authority, rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50)));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanPreGrasp);
  advance_pregrasp_to_verify_grasp(context);
  ASSERT_EQ(context.task_status().state, RestockTaskState::kVerifyGrasp);
  ASSERT_EQ(context.task_status().recovery_attempt, 1U);

  // The second verification of the same goal re-derives from its own snapshot instead of
  // refusing with kAlreadySet, and the receipt says which verification now owns the coupling.
  auto second = snapshot(17);
  second.objects.at(ObjectId{17}).pose_in_world.translation().y() -= 0.006;
  ASSERT_TRUE(context.retain_latest_snapshot(second));
  const auto rederived = context.retain_grasp_coupling();
  EXPECT_TRUE(rederived) << rederived.detail;
  EXPECT_EQ(context.grasp_coupling_ordinal(), 2U);
  EXPECT_EQ(context.grasp_coupling()->source_world_revision, 17U);
  const Eigen::Isometry3d expected =
    second.objects.at(ObjectId{17}).pose_in_world.inverse() *
    context.grasp_candidate_batch()->candidates.front().poses.world_from_grasp_center;
  EXPECT_TRUE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      expected.matrix(), 1.0e-12));
  EXPECT_FALSE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      first_coupling.matrix(), 1.0e-12));
}

// Only one of the ordered candidates is executed, and every consumer (driver, grasp coupling)
// must agree on which.
TEST(RestockGoalContext, FallsThroughGraspCandidatesInOrderWhilePlanningThePreGrasp)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_plan_pregrasp(context, value, two_yaws());
  const auto & candidates = context.grasp_candidate_batch()->candidates;
  ASSERT_EQ(candidates.size(), 2U);
  ASSERT_EQ(context.active_grasp_candidate(), &candidates.front());
  EXPECT_EQ(context.active_grasp_candidate_index(), 0U);

  const auto first = context.refuse_active_grasp_candidate(
    "wrist_3 stands inside the left neighbour");
  EXPECT_EQ(first.disposition, GraspFallthroughDisposition::kNextCandidateActive);
  EXPECT_NE(first.detail.find("wrist_3 stands inside the left neighbour"), std::string::npos);
  EXPECT_NE(first.detail.find("falling through to grasp candidate 2 of 2"), std::string::npos);
  EXPECT_EQ(context.active_grasp_candidate(), &candidates[1]);
  EXPECT_EQ(context.active_grasp_candidate_index(), 1U);

  // The second refusal exhausts the batch and carries each candidate's refusal cause.
  const auto second = context.refuse_active_grasp_candidate("no inverse-kinematics solution");
  EXPECT_EQ(second.disposition, GraspFallthroughDisposition::kCandidatesExhausted);
  EXPECT_EQ(context.active_grasp_candidate(), nullptr);
  EXPECT_NE(second.detail.find("wrist_3 stands inside the left neighbour"), std::string::npos);
  EXPECT_NE(second.detail.find("no inverse-kinematics solution"), std::string::npos);
  EXPECT_EQ(
    context.grasp_candidate_refusal_summary().find("grasp candidate 1 of 2"), 0U);

  // Asking again is answered the same way and records nothing new.
  const auto again = context.refuse_active_grasp_candidate("asked once more");
  EXPECT_EQ(again.disposition, GraspFallthroughDisposition::kCandidatesExhausted);
  EXPECT_EQ(again.detail.find("asked once more"), std::string::npos);
}

TEST(RestockGoalContext, MayRetireACandidateWhoseApproachPlanWasRefusedBeforeMotion)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_plan_pregrasp(context, value, two_yaws());
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kExecutePreGrasp);
  complete_execution_state(context, RestockTaskCommand::kExecutePreGrasp, 11U);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kOpenGripperForApproach);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded).state,
    RestockTaskState::kPlanApproach);

  const auto refused = context.refuse_active_grasp_candidate("approach collision at 2.7 percent");
  EXPECT_EQ(refused.disposition, GraspFallthroughDisposition::kNextCandidateActive);
  EXPECT_EQ(context.active_grasp_candidate_index(), 1U);
}

// The window closes once approach planning succeeds; later steps are bound to that grasp.
TEST(RestockGoalContext, RefusesToMoveTheGraspCandidateOnceTheArmHasActedOnIt)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value, two_yaws());
  const auto * active = context.active_grasp_candidate();
  ASSERT_NE(active, nullptr);

  const auto refused = context.refuse_active_grasp_candidate("too late to change the grasp");
  EXPECT_EQ(refused.disposition, GraspFallthroughDisposition::kNotSelecting);
  EXPECT_EQ(context.active_grasp_candidate(), active);
  EXPECT_EQ(context.active_grasp_candidate_index(), 0U);
  EXPECT_EQ(context.grasp_candidate_refusal_summary(), "no refusal was recorded");
}

// The coupling feeds placement insertion, so it must describe the grasp that happened, not the
// head of the batch after a fall-through.
TEST(RestockGoalContext, BindsTheGraspCouplingToTheCandidateTheTaskExecuted)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_plan_pregrasp(context, value, two_yaws());
  ASSERT_EQ(
    context.refuse_active_grasp_candidate("no plan").disposition,
    GraspFallthroughDisposition::kNextCandidateActive);
  advance_pregrasp_to_verify_grasp(context);

  auto verified = snapshot(13);
  ASSERT_TRUE(context.retain_latest_snapshot(verified));
  ASSERT_TRUE(context.retain_grasp_coupling());

  const auto & candidates = context.grasp_candidate_batch()->candidates;
  const Eigen::Isometry3d expected =
    verified.objects.at(ObjectId{17}).pose_in_world.inverse() *
    candidates[1].poses.world_from_grasp_center;
  EXPECT_TRUE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      expected.matrix(), 1.0e-12));
  // Not the head of the batch, which is what it used to read.
  const Eigen::Isometry3d head =
    verified.objects.at(ObjectId{17}).pose_in_world.inverse() *
    candidates.front().poses.world_from_grasp_center;
  EXPECT_FALSE(
    context.grasp_coupling()->product_from_grasp_center.matrix().isApprox(
      head.matrix(), 1.0e-12));
}

// Regression: the reservation record was frozen at capability grant, so after attach/detach
// the release request (built from and validated against the whole record) could not be built.
TEST(RestockGoalContext, CarriesTheReservationRecordForwardWithFreshEvidence)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  const auto granted = context.reservation()->reservation;
  const auto granted_world_revision = context.reservation()->world_revision;

  auto observed = reserved_snapshot(context, snapshot(13));
  observed.revision = 13U;
  ASSERT_TRUE(observed.active_reservation);
  observed.active_reservation->revision = observed.revision;
  ASSERT_TRUE(context.retain_latest_snapshot(observed));

  EXPECT_EQ(context.reservation()->reservation.revision, observed.revision);
  EXPECT_GT(context.reservation()->reservation.revision, granted.revision);
  // The record must not be newer than its world revision; the two move together.
  EXPECT_EQ(context.reservation()->world_revision, observed.revision);
  EXPECT_GT(context.reservation()->world_revision, granted_world_revision);
  EXPECT_EQ(context.reservation()->reservation.reservation_id, granted.reservation_id);

  // A record that is not this reservation, or that has gone backwards, is left alone.
  auto foreign = reserved_snapshot(context, snapshot(14));
  foreign.revision = 14U;
  ASSERT_TRUE(foreign.active_reservation);
  foreign.active_reservation->reservation_id = granted.reservation_id + 1U;
  foreign.active_reservation->revision = foreign.revision;
  ASSERT_TRUE(context.retain_latest_snapshot(foreign));
  EXPECT_EQ(context.reservation()->reservation.reservation_id, granted.reservation_id);
  EXPECT_EQ(context.reservation()->reservation.revision, observed.revision);
}

TEST(RestockGoalContext, RefusesTheGraspCouplingOutsideGraspVerification)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_generate_grasps(context, value);
  EXPECT_EQ(context.retain_grasp_coupling().error, GoalContextErrorCode::kInvalidPhase);
  EXPECT_FALSE(context.grasp_coupling());
}

// Acceptance attempt 3: kVerifyGrasp adopts a fresher snapshot, then recovery re-enters
// kGenerateGrasps with the pre-verify selection stamps and generate rejects the pair as
// "snapshot, selection, and robot rail lineage are inconsistent". The rebind keeps the
// pair identity and follows the adopted snapshot's revision stamps.
TEST(RestockGoalContext, RebindsSelectionToTheVerificationSnapshotBeforeRegeneratingGrasps)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  ASSERT_EQ(context.selection()->snapshot_revision, value.revision);

  // Verify adopts a later world revision (telemetry/observations advanced during motion).
  auto verified = snapshot(13);
  verified.objects.at(ObjectId{17}).pose_in_world.translation().y() += 0.002;
  ASSERT_TRUE(context.retain_latest_snapshot(verified));
  ASSERT_EQ(context.latest_snapshot()->revision, 13U);
  ASSERT_EQ(context.selection()->snapshot_revision, value.revision);
  ASSERT_NE(
    context.selection()->snapshot_revision, context.latest_snapshot()->revision);

  // Failed verify recovers to generate-grasps with the new latest retained. TerminalFailure at
  // a non-execution state enters recovery directly; the resume state for kVerifyGrasp is
  // kGenerateGrasps.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kTerminalFailure, "verify refused the grasp").state,
    RestockTaskState::kRecover);
  // Card 062: the jaws closed on a product that is not held open before the grasp is left.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "recovery completed").state,
    RestockTaskState::kOpenGripperForEscape);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "jaws opened").state,
    RestockTaskState::kGenerateGrasps);

  ASSERT_TRUE(context.rebind_selection_to_latest_snapshot());
  ASSERT_EQ(context.selection()->snapshot_revision, 13U);
  ASSERT_EQ(
    context.selection()->object_revision,
    context.latest_snapshot()->objects.at(ObjectId{17}).revision);
  ASSERT_EQ(
    context.selection()->lane_revision,
    context.latest_snapshot()->lanes.at(LaneId{"lane_01"}).revision);
  // Pair identity is unchanged: still the same object and lane selection proved eligible.
  EXPECT_EQ(context.selection()->object_id, ObjectId{17});
  EXPECT_EQ(context.selection()->lane_id, LaneId{"lane_01"});

  // Generate's lineage check now passes against the adopted snapshot.
  const auto authority = grasp_authority(*context.latest_snapshot());
  const auto generated = generate_grasp_candidate_batch(
    *context.latest_snapshot(), *context.selection(), authority.gripper,
    authority.tool0_from_grasp_center, authority.config,
    rclcpp::Time(std::int64_t{1'100'000'000}, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  EXPECT_TRUE(generated) << generated.error().detail;
}

TEST(RestockGoalContext, RebindRefusesWhenTheRetainedPairLeftTheLatestSnapshot)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  auto verified = snapshot(13);
  verified.objects.erase(ObjectId{17});
  ASSERT_TRUE(context.retain_latest_snapshot(verified));
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kTerminalFailure, "verify failed").state,
    RestockTaskState::kRecover);
  // Card 062: the jaws closed on a product that is not held open before the grasp is left.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "recovery completed").state,
    RestockTaskState::kOpenGripperForEscape);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "jaws opened").state,
    RestockTaskState::kGenerateGrasps);
  const auto rebound = context.rebind_selection_to_latest_snapshot();
  EXPECT_EQ(rebound.error, GoalContextErrorCode::kIdentityMismatch);
}

TEST(RestockGoalContext, RebindIsANoOpWhenStampsAlreadyMatchAndRefusesOutsideGenerate)
{
  auto context = make_context();
  const auto value = snapshot();
  drive_to_verify_grasp(context, value);
  // Outside generate-grasps (still at verify) the rebind refuses rather than mutating a fixed
  // pair. The no-op success case is covered by the first Rebind* test, which rebinds a moved
  // stamp set and then regenerates against the adopted snapshot.
  EXPECT_EQ(
    context.rebind_selection_to_latest_snapshot().error, GoalContextErrorCode::kInvalidPhase);

  // After recovery into generate-grasps with matching stamps, rebind is a successful no-op.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kTerminalFailure, "verify failed").state,
    RestockTaskState::kRecover);
  // Card 062: the jaws closed on a product that is not held open before the grasp is left.
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "recovery completed").state,
    RestockTaskState::kOpenGripperForEscape);
  ASSERT_EQ(
    context.dispatch_task(RestockTaskEvent::kOperationSucceeded, "jaws opened").state,
    RestockTaskState::kGenerateGrasps);
  const auto before = *context.selection();
  ASSERT_TRUE(context.rebind_selection_to_latest_snapshot());
  EXPECT_EQ(context.selection()->snapshot_revision, before.snapshot_revision);
  EXPECT_EQ(context.selection()->object_revision, before.object_revision);
  EXPECT_TRUE(context.grasp_candidate_batch());

  // Coverage for the gate the rebind fix exists for: recovered regeneration
  // (recovery_attempt > 0) with first_trajectory_may_have_started must still accept
  // generate-success once a batch is retained. Pre-motion paths still refuse that combination.
  const auto staged = context.dispatch_task(RestockTaskEvent::kOperationSucceeded);
  EXPECT_EQ(staged.accepted, true) << staged.detail;
  EXPECT_EQ(staged.state, RestockTaskState::kPlanPreGrasp) << staged.detail;
  EXPECT_EQ(
    context.rebind_selection_to_latest_snapshot().error, GoalContextErrorCode::kInvalidPhase);
}

TEST(RestockGoalContext, NamesClosedEnums)
{
  EXPECT_STREQ(to_string(CoordinatorMutationKind::kReserveTask), "reserve-task");
  EXPECT_STREQ(to_string(CoordinatorMutationKind::kCommitDetachment), "commit-detachment");
  EXPECT_STREQ(to_string(GoalContextErrorCode::kNone), "none");
  EXPECT_STREQ(to_string(GoalContextErrorCode::kIdentityMismatch), "identity_mismatch");
}

}  // namespace
}  // namespace restocker_task_executor
