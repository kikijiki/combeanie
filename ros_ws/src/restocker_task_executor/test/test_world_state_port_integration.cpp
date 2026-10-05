// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>

#include "restocker_task_executor/world_state_port_contract.hpp"
#include "restocker_world_state/ros_conversions.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using restocker_world_state::FaultState;
using restocker_world_state::LaneDefinition;
using restocker_world_state::LaneId;
using restocker_world_state::LaneObservation;
using restocker_world_state::ObjectObservation;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::PoseCovariance;
using restocker_world_state::ProductClass;
using restocker_world_state::ReservationOutcome;
using restocker_world_state::RobotTelemetryObservation;
using restocker_world_state::TaskPhase;
using restocker_world_state::WorldStateConfig;
using restocker_world_state::WorldStateErrorCode;
using restocker_world_state::WorldStateStore;

[[nodiscard]] rclcpp::Time ros_time(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

struct AuthorityFixture
{
  WorldStateStore store;
  SelectedTaskPair selection;

  AuthorityFixture()
  : store([] {
        WorldStateConfig config;
        config.maximum_observation_age = 2s;
        config.maximum_future_skew = 100ms;
        // The catalogued can as the lane sees it: its diameter plus the lane's entry clearance
        // of free rear depth for one more, and that diameter foreshortened by the bed added to
        // the settled column. Without a profile the store admits no placement.
        config.product_lane_profiles = {
          restocker_world_state::ProductLaneProfile{
            ProductClass::Can, std::nullopt, 2.0 * 0.033 + 0.045,
            2.0 * 0.033 * 0.99756405025982420}};
        return config;
      }())
  {
    const auto lane = store.configure_lane(
      LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN", 0.85},
      ros_time(9'000'000'000));
    EXPECT_TRUE(lane);
    const auto evidence = store.update_lane(
      LaneObservation{LaneId{"lane_01"}, {}, 0.85, false, ros_time(9'200'000'000)},
      ros_time(10'000'000'000), lane.value().revision);
    EXPECT_TRUE(evidence);

    ObjectObservation observation;
    observation.source_object_id = "sim:can_17";
    observation.frame_id = "world";
    observation.product_class = ProductClass::Can;
    observation.sku = "SIM-CAN";
    observation.pose_covariance = PoseCovariance::Identity() * 1.0e-6;
    observation.orientation = ObjectOrientation::Upright;
    observation.observation_time = ros_time(9'300'000'000);
    const auto object = store.observe_object(observation, ros_time(10'000'000'000));
    EXPECT_TRUE(object);
    const auto robot = store.observe_robot_telemetry(
      RobotTelemetryObservation{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, {}, 0.0, 0.0,
        ros_time(9'400'000'000), {}, {}, "test/world-state-port"},
      ros_time(10'000'000'000));
    EXPECT_TRUE(robot);

    const auto snapshot = store.snapshot();
    selection = SelectedTaskPair{
      object.value().object_id, LaneId{"lane_01"}, snapshot.revision,
      snapshot.objects.at(object.value().object_id).revision,
      snapshot.lanes.at(LaneId{"lane_01"}).revision, {0.033, 0.122}};
  }
};

TEST(WorldStatePortIntegration, ReconcilesLostReleaseAndPreservesIdempotency)
{
  AuthorityFixture fixture;
  const auto selected_snapshot = fixture.store.snapshot();
  const auto wire_request = make_reserve_task_request(
    selected_snapshot, fixture.selection, "goal/reserve/1");
  ASSERT_TRUE(wire_request) << wire_request.error().detail;
  const auto domain_request = restocker_world_state::reserve_task_request_from_message(
    wire_request.value());
  ASSERT_TRUE(domain_request) << domain_request.error().detail;

  const auto reserved = fixture.store.reserve_task(
    domain_request.value(), ros_time(9'500'000'000));
  ASSERT_TRUE(reserved) << reserved.error().detail;
  const auto reserve_replay = fixture.store.reserve_task(
    domain_request.value(), ros_time(9'600'000'000));
  ASSERT_TRUE(reserve_replay) << reserve_replay.error().detail;
  EXPECT_EQ(reserve_replay.value().revision, reserved.value().revision);
  EXPECT_EQ(reserve_replay.value().token, reserved.value().token);
  EXPECT_EQ(
    reserve_replay.value().reservation.reservation_id,
    reserved.value().reservation.reservation_id);

  auto changed_reserve = domain_request.value();
  ++changed_reserve.object_revision;
  const auto reserve_conflict = fixture.store.reserve_task(
    changed_reserve, ros_time(9'600'000'000));
  ASSERT_FALSE(reserve_conflict);
  EXPECT_EQ(reserve_conflict.error().code, WorldStateErrorCode::IdempotencyConflict);

  restocker_interfaces::srv::ReserveTask::Response reserve_response;
  reserve_response.status = restocker_world_state::operation_status_ok();
  reserve_response.world_revision = reserved.value().revision;
  reserve_response.token = reserved.value().token;
  reserve_response.reservation = restocker_world_state::task_reservation_to_message(
    reserved.value().reservation);
  const auto capability = validate_reserve_task_response(
    wire_request.value(), selected_snapshot, reserve_response);
  ASSERT_TRUE(capability) << capability.error().detail;

  const auto authoritative_validation = fixture.store.validate_task_reservation(
    capability.value().token);
  ASSERT_TRUE(authoritative_validation) << authoritative_validation.error().detail;
  restocker_interfaces::srv::ValidateTaskReservation::Response validation_response;
  validation_response.status = restocker_world_state::operation_status_ok();
  validation_response.world_revision = authoritative_validation.value().revision;
  validation_response.has_reservation = true;
  validation_response.reservation = restocker_world_state::task_reservation_to_message(
    authoritative_validation.value().reservation);
  ASSERT_TRUE(validate_task_reservation_response(capability.value(), validation_response));

  const auto release_request = make_release_task_reservation_request(
    capability.value(), "goal/release/1", ReservationOutcome::Canceled,
    TaskPhase::Idle, FaultState::None);
  ASSERT_TRUE(release_request) << release_request.error().detail;
  const auto domain_release = restocker_world_state::release_request_from_message(
    release_request.value());
  ASSERT_TRUE(domain_release) << domain_release.error().detail;
  const auto released = fixture.store.release_task_reservation(
    domain_release.value(), ros_time(9'700'000'000));
  ASSERT_TRUE(released) << released.error().detail;

  // Model a lost service reply: reconcile only from causally newer authoritative state.
  const auto terminal_snapshot = fixture.store.snapshot();
  const auto proof = validate_released_snapshot(
    capability.value(), release_request.value(), std::nullopt, terminal_snapshot);
  ASSERT_TRUE(proof) << proof.error().detail;
  EXPECT_EQ(proof.value().snapshot_revision, released.value().revision);

  const auto release_replay = fixture.store.release_task_reservation(
    domain_release.value(), ros_time(9'800'000'000));
  ASSERT_TRUE(release_replay) << release_replay.error().detail;
  EXPECT_EQ(release_replay.value().revision, released.value().revision);
  EXPECT_EQ(fixture.store.snapshot().revision, released.value().revision);

  auto changed_release = domain_release.value();
  changed_release.terminal_fault_state = FaultState::ExternalInconsistency;
  const auto release_conflict = fixture.store.release_task_reservation(
    changed_release, ros_time(9'900'000'000));
  ASSERT_FALSE(release_conflict);
  EXPECT_EQ(release_conflict.error().code, WorldStateErrorCode::IdempotencyConflict);
  EXPECT_EQ(fixture.store.snapshot().revision, released.value().revision);
}

}  // namespace
}  // namespace restocker_task_executor
