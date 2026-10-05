// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <limits>
#include <string>

#include <restocker_interfaces/msg/robot_execution_state.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_task_executor/world_state_port_contract.hpp"
#include "restocker_world_state/ros_conversions.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_world_state::FaultState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationOutcome;
using restocker_world_state::ReservationStage;
using restocker_world_state::TaskPhase;
using restocker_world_state::TaskReservation;
using restocker_world_state::WorldStateSnapshot;
using Status = restocker_interfaces::msg::WorldStateOperationStatus;

WorldStateSnapshot selected_snapshot()
{
  WorldStateSnapshot snapshot;
  snapshot.revision = 20;

  restocker_world_state::TrackedObject object;
  object.id = ObjectId{17};
  object.source_object_id = "sim:can_17";
  object.product_class = ProductClass::Can;
  object.sku = "SIM-CAN";
  object.revision = 12;
  snapshot.objects.emplace(object.id, object);

  restocker_world_state::ShelfLane source;
  source.id = LaneId{"stock_lane"};
  source.contents = {object.id};
  source.evidence_revision = 11;
  source.revision = 13;
  snapshot.lanes.emplace(source.id, source);

  restocker_world_state::ShelfLane destination;
  destination.id = LaneId{"lane_01"};
  destination.evidence_revision = 14;
  destination.revision = 14;
  snapshot.lanes.emplace(destination.id, destination);

  snapshot.robot.revision = 15;
  snapshot.robot.telemetry_revision = 15;
  snapshot.robot.telemetry_time = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  snapshot.robot.telemetry_source_id = "test/world-state-port-contract";
  return snapshot;
}

SelectedTaskPair selection()
{
  return SelectedTaskPair{ObjectId{17}, LaneId{"lane_01"}, 20, 12, 14, {0.033, 0.122}};
}

restocker_interfaces::srv::ReserveTask::Response accepted_response()
{
  const TaskReservation reservation{42,
    "goal/reserve/1",
    ObjectId{17},
    "sim:can_17",
    ProductClass::Can,
    "SIM-CAN",
    LaneId{"stock_lane"},
    LaneId{"lane_01"},
    ReservationStage::Reserved,
    false,
    rclcpp::Time(10'500'000'000LL, RCL_ROS_TIME),
    21,
    15,
    0.85,
    {},
    21,
    ProductClass::Unknown,
    std::nullopt};
  restocker_interfaces::srv::ReserveTask::Response response;
  response.status.code = Status::OK;
  response.world_revision = 21;
  response.token = "secret-capability";
  response.reservation = restocker_world_state::task_reservation_to_message(reservation);
  return response;
}

TaskReservationCapability capability()
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  EXPECT_TRUE(request);
  auto validated = validate_reserve_task_response(request.value(), snapshot, accepted_response());
  EXPECT_TRUE(validated);
  return validated.value();
}

WorldStatePortResult<restocker_world_state::ExecutionWorldAuthorityProof>
validate_execution_world_authority_response(
  const TaskReservationCapability & retained,
  const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response & response)
{
  return restocker_task_executor::validate_execution_world_authority_response(
    retained, response, "world");
}

restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response authority_response(
  const TaskReservationCapability & retained)
{
  restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response response;
  response.status.code = Status::OK;
  response.world_revision = retained.world_revision + 1U;
  response.planning_frame = "world";
  response.has_proof = true;
  response.reservation =
    restocker_world_state::task_reservation_to_message(retained.reservation);

  restocker_world_state::TrackedObject object;
  object.id = retained.reservation.object_id;
  object.source_object_id = retained.reservation.object_source_id;
  object.product_class = retained.reservation.product_class;
  object.sku = retained.reservation.sku;
  object.revision = retained.reservation.created_revision;
  response.object = restocker_world_state::tracked_object_to_message(object);

  restocker_world_state::ShelfLane destination;
  destination.id = retained.reservation.destination_lane;
  destination.expected_product_class = retained.reservation.product_class;
  destination.expected_sku = retained.reservation.sku;
  destination.depth_m = 0.45;
  destination.available_depth_m = 0.45;
  destination.evidence_revision = retained.reservation.created_revision - 1U;
  destination.revision = retained.reservation.created_revision;
  response.destination_lane = restocker_world_state::shelf_lane_to_message(destination);

  response.has_source_lane = retained.reservation.source_lane.has_value();
  if (retained.reservation.source_lane) {
    restocker_world_state::ShelfLane source;
    source.id = *retained.reservation.source_lane;
    source.expected_product_class = retained.reservation.product_class;
    source.depth_m = 0.60;
    source.available_depth_m = 0.30;
    // The narrow proof retains unrelated membership. It is not a full snapshot.
    source.contents = {retained.reservation.object_id, ObjectId{99}};
    source.evidence_revision = retained.reservation.created_revision - 1U;
    source.revision = retained.reservation.created_revision;
    response.source_lane = restocker_world_state::shelf_lane_to_message(source);
  }

  restocker_world_state::RobotExecutionState robot;
  robot.task_phase = TaskPhase::Executing;
  robot.fault_state = FaultState::None;
  robot.telemetry_time = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  robot.telemetry_source_id = "test/execution-authority";
  robot.telemetry_revision = retained.reservation.admitted_robot_telemetry_revision;
  robot.revision = retained.reservation.revision;
  response.robot = restocker_world_state::robot_execution_state_to_message(robot);
  return response;
}

TEST(WorldStatePortCapabilityFingerprint, BindsTokenAndEveryRetainedSemanticField)
{
  const auto retained = capability();
  const auto fingerprint = fingerprint_task_reservation_capability(retained);
  ASSERT_TRUE(fingerprint);
  EXPECT_NE(*fingerprint, 0U);
  EXPECT_EQ(fingerprint_task_reservation_capability(retained), fingerprint);

  auto changed = retained;
  changed.token += "-other";
  EXPECT_NE(fingerprint_task_reservation_capability(changed), fingerprint);
  changed = retained;
  ++changed.world_revision;
  EXPECT_NE(fingerprint_task_reservation_capability(changed), fingerprint);
  changed = retained;
  changed.reservation.request_id += "-other";
  EXPECT_NE(fingerprint_task_reservation_capability(changed), fingerprint);
}

TEST(WorldStatePortCapabilityFingerprint, RejectsMalformedCapabilities)
{
  auto malformed = capability();
  malformed.token.clear();
  EXPECT_FALSE(fingerprint_task_reservation_capability(malformed));

  malformed = capability();
  malformed.reservation.revision = malformed.world_revision + 1U;
  EXPECT_FALSE(fingerprint_task_reservation_capability(malformed));
}

TEST(WorldStatePortReserveRequest, CapturesExactSnapshotAndSourceMembership)
{
  const auto request =
    make_reserve_task_request(selected_snapshot(), selection(), "goal/reserve/1");
  ASSERT_TRUE(request) << request.error().detail;
  EXPECT_EQ(request.value().request_id, "goal/reserve/1");
  EXPECT_EQ(request.value().selected_snapshot_revision, 20U);
  EXPECT_EQ(request.value().object_id, 17U);
  EXPECT_EQ(request.value().object_revision, 12U);
  EXPECT_TRUE(request.value().has_source_lane);
  EXPECT_EQ(request.value().source_lane_id, "stock_lane");
  EXPECT_EQ(request.value().source_lane_revision, 13U);
  EXPECT_EQ(request.value().destination_lane_id, "lane_01");
  EXPECT_EQ(request.value().destination_lane_revision, 14U);
}

TEST(WorldStatePortReserveRequest, EncodesAbsentSourceWithoutDiagnosticPayload)
{
  auto snapshot = selected_snapshot();
  snapshot.lanes.at(LaneId{"stock_lane"}).contents.clear();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request) << request.error().detail;
  EXPECT_FALSE(request.value().has_source_lane);
  EXPECT_TRUE(request.value().source_lane_id.empty());
  EXPECT_EQ(request.value().source_lane_revision, 0U);
}

TEST(WorldStatePortReserveRequest, RejectsDriftAndAmbiguousMembership)
{
  auto snapshot = selected_snapshot();
  auto drifted = selection();
  drifted.object_revision = 11;
  EXPECT_FALSE(make_reserve_task_request(snapshot, drifted, "goal/reserve/1"));
  EXPECT_FALSE(make_reserve_task_request(snapshot, selection(), ""));
  EXPECT_FALSE(make_reserve_task_request(snapshot, selection(), std::string(129, 'x')));

  restocker_world_state::ShelfLane duplicate;
  duplicate.id = LaneId{"stock_lane_02"};
  duplicate.contents = {ObjectId{17}};
  duplicate.evidence_revision = 16;
  duplicate.revision = 16;
  snapshot.lanes.emplace(duplicate.id, duplicate);
  const auto ambiguous = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_FALSE(ambiguous);
  EXPECT_EQ(ambiguous.error().code, WorldStatePortErrorCode::kInvalidInput);
}

TEST(WorldStatePortReserveResponse, AcceptsOnlyExactAuthoritativeCapability)
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request);
  const auto response =
    validate_reserve_task_response(request.value(), snapshot, accepted_response());
  ASSERT_TRUE(response) << response.error().detail;
  EXPECT_EQ(response.value().token, "secret-capability");
  EXPECT_EQ(response.value().world_revision, 21U);
  EXPECT_EQ(response.value().reservation.object_id, ObjectId{17});
  EXPECT_EQ(response.value().reservation.stage, ReservationStage::Reserved);
}

TEST(WorldStatePortReserveResponse, BranchesOnStatusAndRejectsErrorPayload)
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request);

  auto response = restocker_interfaces::srv::ReserveTask::Response{};
  response.status.code = Status::REVISION_CONFLICT;
  response.status.detail = "diagnostic text is not control flow";
  auto rejected = validate_reserve_task_response(request.value(), snapshot, response);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStatePortErrorCode::kRemoteRejected);
  EXPECT_EQ(rejected.error().remote_status, Status::REVISION_CONFLICT);

  response.token = "must-not-leak";
  auto malformed = validate_reserve_task_response(request.value(), snapshot, response);
  ASSERT_FALSE(malformed);
  EXPECT_EQ(malformed.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = restocker_interfaces::srv::ReserveTask::Response{};
  response.status.code = 77;
  EXPECT_EQ(
    validate_reserve_task_response(request.value(), snapshot, response).error().code,
    WorldStatePortErrorCode::kMalformedResponse);
}

TEST(WorldStatePortReserveResponse, TreatsEveryClosedFailureStatusAsRemoteRejection)
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request);
  constexpr std::array<std::uint16_t, 10> failure_codes{
    Status::INVALID_ARGUMENT, Status::NOT_FOUND, Status::REVISION_CONFLICT,
    Status::RESERVATION_CONFLICT, Status::TOKEN_MISMATCH, Status::PREDICATE_FAILED,
    Status::INVALID_TRANSITION, Status::IDEMPOTENCY_CONFLICT,
    Status::RESOURCE_EXHAUSTED, Status::INTERNAL_ERROR};
  for (const auto code : failure_codes) {
    restocker_interfaces::srv::ReserveTask::Response response;
    response.status.code = code;
    const auto result = validate_reserve_task_response(request.value(), snapshot, response);
    ASSERT_FALSE(result);
    EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kRemoteRejected);
    EXPECT_EQ(result.error().remote_status, code);
  }
}

TEST(WorldStatePortReserveResponse, RejectsIdentityAndRevisionSubstitution)
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request);

  auto identity = accepted_response();
  identity.reservation.object_source_id = "sim:different";
  const auto identity_result = validate_reserve_task_response(request.value(), snapshot, identity);
  ASSERT_FALSE(identity_result);
  EXPECT_EQ(identity_result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  auto revision = accepted_response();
  revision.reservation.revision = 20;
  const auto revision_result = validate_reserve_task_response(request.value(), snapshot, revision);
  ASSERT_FALSE(revision_result);
  EXPECT_EQ(revision_result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  auto regressed_telemetry = accepted_response();
  regressed_telemetry.reservation.admitted_robot_telemetry_revision = 14;
  const auto telemetry_result = validate_reserve_task_response(
    request.value(), snapshot, regressed_telemetry);
  ASSERT_FALSE(telemetry_result);
  EXPECT_EQ(telemetry_result.error().code, WorldStatePortErrorCode::kMalformedResponse);
}

TEST(WorldStatePortReserveResponse, RejectsRequestsThatDoNotMatchRetainedSnapshot)
{
  const auto snapshot = selected_snapshot();
  const auto generated = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(generated);

  auto request = generated.value();
  request.selected_snapshot_revision = 19;
  auto result = validate_reserve_task_response(request, snapshot, accepted_response());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kInvalidInput);

  request = generated.value();
  request.has_source_lane = false;
  result = validate_reserve_task_response(request, snapshot, accepted_response());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kInvalidInput);

  request = generated.value();
  request.source_lane_revision = 12;
  result = validate_reserve_task_response(request, snapshot, accepted_response());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kInvalidInput);
}

TEST(WorldStatePortReservationValidation, RequiresExactAuthoritativeReadback)
{
  const auto held_capability = capability();
  const auto request = make_validate_task_reservation_request(held_capability);
  ASSERT_TRUE(request) << request.error().detail;
  EXPECT_EQ(request.value().token, held_capability.token);

  restocker_interfaces::srv::ValidateTaskReservation::Response response;
  response.status.code = Status::OK;
  response.world_revision = held_capability.world_revision;
  response.has_reservation = true;
  response.reservation = restocker_world_state::task_reservation_to_message(
    held_capability.reservation);
  const auto accepted = validate_task_reservation_response(held_capability, response);
  ASSERT_TRUE(accepted) << accepted.error().detail;
  EXPECT_EQ(accepted.value().reservation.reservation_id, 42U);

  response.reservation.stage = restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED;
  const auto substituted = validate_task_reservation_response(held_capability, response);
  ASSERT_FALSE(substituted);
  EXPECT_EQ(substituted.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response.status.code = Status::TOKEN_MISMATCH;
  response.has_reservation = false;
  response.reservation = restocker_interfaces::msg::TaskReservation{};
  const auto rejected = validate_task_reservation_response(held_capability, response);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStatePortErrorCode::kRemoteRejected);
}

TEST(WorldStatePortExecutionAuthority, EncodesExactCapabilityExpectation)
{
  const auto retained = capability();
  const auto request = make_validate_execution_world_authority_request(retained);
  ASSERT_TRUE(request) << request.error().detail;
  EXPECT_EQ(request.value().token, retained.token);
  EXPECT_EQ(request.value().expected_reservation_id, retained.reservation.reservation_id);
  EXPECT_EQ(request.value().expected_reservation_revision, retained.reservation.revision);
  EXPECT_EQ(request.value().expected_object_id, retained.reservation.object_id.value);
  EXPECT_EQ(
    request.value().expected_destination_lane_id,
    retained.reservation.destination_lane.value);

  auto invalid = retained;
  invalid.token.clear();
  const auto rejected = make_validate_execution_world_authority_request(invalid);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStatePortErrorCode::kInvalidInput);
}

TEST(WorldStatePortExecutionAuthority, AcceptsNarrowAtomicProofWithUnrelatedSourceMembership)
{
  const auto retained = capability();
  const auto response = authority_response(retained);
  const auto result = validate_execution_world_authority_response(retained, response);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().revision, retained.world_revision + 1U);
  ASSERT_TRUE(result.value().source_lane);
  ASSERT_EQ(result.value().source_lane->contents.size(), 2U);
  EXPECT_EQ(result.value().source_lane->contents.at(1), ObjectId{99});
  EXPECT_EQ(result.value().reservation.reservation_id, retained.reservation.reservation_id);
  EXPECT_EQ(result.value().robot.telemetry_revision, 15U);
}

TEST(WorldStatePortExecutionAuthority, AcceptsReservationWithoutSourceLane)
{
  auto retained = capability();
  retained.reservation.source_lane.reset();
  auto response = authority_response(retained);
  ASSERT_FALSE(response.has_source_lane);
  const auto result = validate_execution_world_authority_response(retained, response);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_FALSE(result.value().source_lane);
}

TEST(WorldStatePortExecutionAuthority, DistinguishesCleanRejectionFromMalformedPayload)
{
  const auto retained = capability();
  restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response response;
  response.status.code = Status::PREDICATE_FAILED;
  response.status.detail = "destination changed";
  response.world_revision = retained.world_revision + 1U;
  auto result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kRemoteRejected);
  EXPECT_EQ(result.error().remote_status, Status::PREDICATE_FAILED);

  response.object = authority_response(retained).object;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.status.code = std::numeric_limits<std::uint16_t>::max();
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);
}

TEST(WorldStatePortExecutionAuthority, RejectsIncompleteOrMalformedSuccess)
{
  const auto retained = capability();

  auto response = authority_response(retained);
  response.status.detail = "unexpected success detail";
  auto result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.has_proof = false;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.object.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.robot.task_phase = std::numeric_limits<std::uint8_t>::max();
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.source_lane.revision = response.world_revision + 1U;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response = authority_response(retained);
  response.has_source_lane = false;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kMalformedResponse);
}

TEST(WorldStatePortExecutionAuthority, RejectsFrameAndDestinationEvidenceGaps)
{
  const auto retained = capability();

  auto response = authority_response(retained);
  response.planning_frame = "map";
  auto result = restocker_task_executor::validate_execution_world_authority_response(
    retained, response, "world");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.planning_frame.clear();
  result = restocker_task_executor::validate_execution_world_authority_response(
    retained, response, "world");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  result = restocker_task_executor::validate_execution_world_authority_response(
    retained, response, "");
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kInvalidInput);

  response = authority_response(retained);
  response.destination_lane.evidence_revision = 0U;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);
}

TEST(WorldStatePortExecutionAuthority, RejectsIdentityAndPredicateSubstitution)
{
  const auto retained = capability();

  auto response = authority_response(retained);
  response.reservation.request_id = "substituted/request";
  auto result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.object.id = 18U;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.destination_lane.id = "lane_02";
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.source_lane.id = "stock_lane_02";
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.destination_lane.obstructed = true;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response = authority_response(retained);
  response.robot.task_phase = restocker_interfaces::msg::RobotExecutionState::TASK_IDLE;
  result = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);
}

// Lane intent may change while a transfer runs. The proof is judged against the policy captured
// at grant — the same rule the store enforces — so a live policy change neither blocks the
// in-flight motion authorisation nor smuggles in an object the captured policy refuses.
TEST(WorldStatePortExecutionAuthority, JudgesTheCapturedPolicyNotTheLiveLaneIntent)
{
  auto retained = capability();
  retained.reservation.destination_expected_product_class = ProductClass::Can;
  retained.reservation.destination_expected_sku = "SIM-CAN";

  auto response = authority_response(retained);
  response.destination_lane.expected_product_class =
    restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_SMALL_BOTTLE;
  response.destination_lane.expected_sku = "SIM-BOTTLE-SMALL";
  const auto accepted = validate_execution_world_authority_response(retained, response);
  ASSERT_TRUE(accepted) << accepted.error().detail;

  retained.reservation.destination_expected_product_class = ProductClass::SmallBottle;
  retained.reservation.destination_expected_sku = "SIM-BOTTLE-SMALL";
  response = authority_response(retained);
  response.destination_lane.expected_product_class =
    restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_SMALL_BOTTLE;
  response.destination_lane.expected_sku = "SIM-BOTTLE-SMALL";
  const auto refused = validate_execution_world_authority_response(retained, response);
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStatePortErrorCode::kIdentityMismatch);
}

// A placed Detached proof judged after the store's §3 growth commit: identity-free destination
// evidence (the wrist depth producer never names lane contents) passes this payload re-check —
// the catalogue-dependent growth proof is the store's, enforced at commit and Detached
// revalidation. When a producer does supply identities, evidence that omits the target is a
// contradiction and is refused; evidence naming it passes.
TEST(WorldStatePortExecutionAuthority, JudgesPlacedDetachedProofsByIdentityWhenPresent)
{
  auto placed = capability();
  placed.reservation.stage = ReservationStage::Detached;
  placed.reservation.placed_in_destination = true;
  placed.reservation.revision = 22;
  placed.world_revision = 22;

  auto response = authority_response(placed);
  response.destination_lane.contents = {placed.reservation.object_id.value};
  response.source_lane.contents = {99U};
  const auto unnamed = validate_execution_world_authority_response(placed, response);
  ASSERT_TRUE(unnamed) << unnamed.error().detail;

  response.destination_lane.observed_source_object_ids = {"sim:other"};
  const auto foreign = validate_execution_world_authority_response(placed, response);
  ASSERT_FALSE(foreign);
  EXPECT_EQ(foreign.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  response.destination_lane.observed_source_object_ids = {
    placed.reservation.object_source_id};
  const auto named = validate_execution_world_authority_response(placed, response);
  ASSERT_TRUE(named) << named.error().detail;

  response.destination_lane.obstructed = true;
  const auto obstructed = validate_execution_world_authority_response(placed, response);
  ASSERT_FALSE(obstructed);
  EXPECT_EQ(obstructed.error().code, WorldStatePortErrorCode::kIdentityMismatch);
}

// The store captures the destination policy under the same lane-revision fence that admitted the
// reservation, so a response whose captured policy disagrees with the selected snapshot describes
// a grant this selection never made.
TEST(WorldStatePortReserveResponse, RejectsCapturedPolicyThatDiffersFromTheSelectedSnapshot)
{
  const auto snapshot = selected_snapshot();
  const auto request = make_reserve_task_request(snapshot, selection(), "goal/reserve/1");
  ASSERT_TRUE(request);

  auto response = accepted_response();
  response.reservation.destination_expected_product_class =
    restocker_interfaces::msg::TaskReservation::PRODUCT_CLASS_SMALL_BOTTLE;
  auto result = validate_reserve_task_response(request.value(), snapshot, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kIdentityMismatch);
}

TEST(WorldStatePortReleaseRequest, EncodesEveryLegalTerminalDisposition)
{
  const auto held_capability = capability();
  const auto canceled = make_release_task_reservation_request(
    held_capability, "goal/release/1",
    ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(canceled) << canceled.error().detail;
  EXPECT_EQ(
    canceled.value().outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_CANCELED);
  EXPECT_EQ(
    canceled.value().terminal_task_phase,
    restocker_interfaces::msg::RobotExecutionState::TASK_IDLE);
  EXPECT_EQ(
    canceled.value().terminal_fault_state,
    restocker_interfaces::msg::RobotExecutionState::FAULT_NONE);

  const auto failed_before_motion = make_release_task_reservation_request(
    held_capability, "goal/release/failed-before-motion",
    ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::NonRecoverable);
  ASSERT_TRUE(failed_before_motion) << failed_before_motion.error().detail;
  EXPECT_EQ(
    failed_before_motion.value().outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_FAILED_SAFE);
  EXPECT_EQ(
    failed_before_motion.value().expected_reservation_id,
    held_capability.reservation.reservation_id);
  EXPECT_EQ(
    failed_before_motion.value().expected_reservation_stage,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::EXPECTED_STAGE_RESERVED);
  EXPECT_EQ(
    failed_before_motion.value().expected_reservation_revision,
    held_capability.reservation.revision);

  auto safely_detached = held_capability;
  safely_detached.reservation.stage = ReservationStage::Detached;
  safely_detached.reservation.revision = 22;
  safely_detached.world_revision = 22;
  const auto failed = make_release_task_reservation_request(
    safely_detached, "goal/release/2", ReservationOutcome::FailedSafe,
    TaskPhase::RequestingOperator, FaultState::ExternalInconsistency);
  ASSERT_TRUE(failed) << failed.error().detail;
  EXPECT_EQ(
    failed.value().outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_FAILED_SAFE);

  auto placed = safely_detached;
  placed.reservation.placed_in_destination = true;
  const auto succeeded = make_release_task_reservation_request(
    placed, "goal/release/3", ReservationOutcome::Succeeded,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(succeeded) << succeeded.error().detail;
  EXPECT_EQ(
    succeeded.value().outcome,
    restocker_interfaces::srv::ReleaseTaskReservation::Request::OUTCOME_SUCCEEDED);
}

TEST(WorldStatePortReleaseRequest, RejectsInvalidTerminalCombinations)
{
  const auto held_capability = capability();
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, "goal/release/1",
      ReservationOutcome::Succeeded,
      TaskPhase::Fault, FaultState::NonRecoverable));
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, "goal/release/1",
      ReservationOutcome::FailedSafe,
      TaskPhase::Idle, FaultState::None));
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, "goal/release/1",
      ReservationOutcome::Succeeded,
      TaskPhase::Idle, FaultState::None));
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, "", ReservationOutcome::Canceled, TaskPhase::Idle, FaultState::None));
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, std::string(129, 'x'), ReservationOutcome::Canceled,
      TaskPhase::Idle, FaultState::None));
  EXPECT_FALSE(
    make_release_task_reservation_request(
      held_capability, held_capability.reservation.request_id,
      ReservationOutcome::Canceled, TaskPhase::Idle, FaultState::None));
}

TEST(WorldStatePortReleaseResponse, RequiresCausallyNewSuccessRevision)
{
  const auto held_capability = capability();
  restocker_interfaces::srv::ReleaseTaskReservation::Response response;
  response.status.code = Status::OK;
  response.world_revision = 22;
  const auto accepted = validate_release_task_reservation_response(held_capability, response);
  ASSERT_TRUE(accepted) << accepted.error().detail;
  EXPECT_EQ(accepted.value().world_revision, 22U);

  response.world_revision = held_capability.world_revision;
  const auto stale = validate_release_task_reservation_response(held_capability, response);
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStatePortErrorCode::kMalformedResponse);

  response.status.code = Status::TOKEN_MISMATCH;
  response.status.detail = "token mismatch";
  const auto rejected = validate_release_task_reservation_response(held_capability, response);
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStatePortErrorCode::kRemoteRejected);
  EXPECT_EQ(rejected.error().remote_status, Status::TOKEN_MISMATCH);
}

TEST(WorldStatePortReleaseResponse, RejectsInvalidInputCapability)
{
  auto held_capability = capability();
  held_capability.token.clear();
  restocker_interfaces::srv::ReleaseTaskReservation::Response response;
  response.status.code = Status::OK;
  response.world_revision = 22;
  const auto result = validate_release_task_reservation_response(held_capability, response);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, WorldStatePortErrorCode::kInvalidInput);
}

TEST(WorldStatePortReleaseProof, RequiresTerminalAuthoritativeSnapshot)
{
  const auto held_capability = capability();
  const auto request = make_release_task_reservation_request(
    held_capability, "goal/release/1", ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(request);
  const TaskReservationReleaseAcknowledgement acknowledgement{22};

  auto snapshot = selected_snapshot();
  snapshot.revision = 22;
  snapshot.robot.revision = 22;
  const auto proof = validate_released_snapshot(
    held_capability, request.value(), acknowledgement, snapshot);
  ASSERT_TRUE(proof) << proof.error().detail;
  EXPECT_EQ(proof.value().snapshot_revision, 22U);

  snapshot.active_reservation = held_capability.reservation;
  const auto still_reserved = validate_released_snapshot(
    held_capability, request.value(), acknowledgement, snapshot);
  ASSERT_FALSE(still_reserved);
  EXPECT_EQ(still_reserved.error().code, WorldStatePortErrorCode::kIdentityMismatch);

  const auto failed_request = make_release_task_reservation_request(
    held_capability, "goal/release/failed-safe", ReservationOutcome::FailedSafe,
    TaskPhase::Fault, FaultState::NonRecoverable);
  ASSERT_TRUE(failed_request);
  snapshot.active_reservation.reset();
  snapshot.robot.task_phase = TaskPhase::Fault;
  snapshot.robot.fault_state = FaultState::NonRecoverable;
  const auto failed_proof = validate_released_snapshot(
    held_capability, failed_request.value(), acknowledgement, snapshot);
  ASSERT_TRUE(failed_proof) << failed_proof.error().detail;

  auto wrong_membership = snapshot;
  wrong_membership.lanes.at(LaneId{"stock_lane"}).contents.clear();
  wrong_membership.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{17}};
  EXPECT_FALSE(
    validate_released_snapshot(
      held_capability, failed_request.value(), acknowledgement, wrong_membership));

  auto duplicate_membership = snapshot;
  duplicate_membership.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{17}};
  EXPECT_FALSE(
    validate_released_snapshot(
      held_capability, failed_request.value(), acknowledgement, duplicate_membership));
}

TEST(WorldStatePortReleaseProof, VerifiesSuccessfulDestinationMembership)
{
  auto placed = capability();
  placed.reservation.stage = ReservationStage::Detached;
  placed.reservation.placed_in_destination = true;
  placed.reservation.revision = 22;
  placed.world_revision = 22;
  const auto request = make_release_task_reservation_request(
    placed, "goal/release/1", ReservationOutcome::Succeeded,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(request);

  auto snapshot = selected_snapshot();
  snapshot.revision = 23;
  snapshot.robot.revision = 23;
  snapshot.objects.at(ObjectId{17}).revision = 22;
  snapshot.lanes.at(LaneId{"stock_lane"}).contents.clear();
  snapshot.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{17}};
  snapshot.lanes.at(LaneId{"lane_01"}).revision = 22;
  const auto proof = validate_released_snapshot(
    placed, request.value(), TaskReservationReleaseAcknowledgement{23}, snapshot);
  ASSERT_TRUE(proof) << proof.error().detail;

  snapshot.lanes.at(LaneId{"lane_01"}).contents.clear();
  EXPECT_FALSE(
    validate_released_snapshot(
      placed, request.value(), TaskReservationReleaseAcknowledgement{23}, snapshot));

  snapshot.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{17}};
  snapshot.objects.at(ObjectId{17}).revision = 21;
  EXPECT_FALSE(
    validate_released_snapshot(
      placed, request.value(), TaskReservationReleaseAcknowledgement{23}, snapshot));

  snapshot.objects.at(ObjectId{17}).revision = 22;
  snapshot.lanes.at(LaneId{"lane_01"}).revision = 21;
  EXPECT_FALSE(
    validate_released_snapshot(
      placed, request.value(), TaskReservationReleaseAcknowledgement{23}, snapshot));
}

TEST(WorldStatePortError, HasStableDiagnosticNames)
{
  EXPECT_EQ(to_string(WorldStatePortErrorCode::kInvalidInput), "invalid_input");
  EXPECT_EQ(to_string(WorldStatePortErrorCode::kRemoteRejected), "remote_rejected");
  EXPECT_EQ(to_string(WorldStatePortErrorCode::kMalformedResponse), "malformed_response");
  EXPECT_EQ(to_string(WorldStatePortErrorCode::kIdentityMismatch), "identity_mismatch");
}

}  // namespace
}  // namespace restocker_task_executor
