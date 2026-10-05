// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{
namespace
{
// Representative grasp: product slightly forward of the grasp datum and yawed. The value is
// arbitrary; the tests check that it is recorded and returned unchanged.
[[nodiscard]] Eigen::Isometry3d test_grasp()
{
  Eigen::Isometry3d grasp = Eigen::Isometry3d::Identity();
  grasp.translation() = Eigen::Vector3d(0.004, -0.002, 0.031);
  grasp.linear() = Eigen::AngleAxisd(0.37, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return grasp;
}


using namespace std::chrono_literals;

rclcpp::Time ros_time(std::int64_t nanoseconds) {return rclcpp::Time(nanoseconds, RCL_ROS_TIME);}

ObjectObservation observation(
  std::string source_id, std::int64_t timestamp_ns,
  ProductClass product_class = ProductClass::Can)
{
  ObjectObservation value;
  value.source_object_id = std::move(source_id);
  value.frame_id = "world";
  value.product_class = product_class;
  value.pose_in_world = Eigen::Isometry3d::Identity();
  value.pose_covariance = PoseCovariance::Identity() * 0.001;
  value.orientation = ObjectOrientation::Upright;
  value.observation_time = ros_time(timestamp_ns);
  return value;
}

WorldStateConfig short_freshness_config(std::size_t event_capacity = 32)
{
  WorldStateConfig config;
  config.maximum_observation_age = 1s;
  config.maximum_future_skew = 100ms;
  config.event_capacity = event_capacity;
  return config;
}

RobotTelemetryObservation robot_telemetry(std::int64_t timestamp_ns)
{
  RobotTelemetryObservation observation;
  observation.joint_positions = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};
  observation.joint_velocities = {0.01, -0.02, 0.03, -0.04, 0.05, -0.06};
  observation.rail_position = 0.25;
  observation.rail_velocity = 0.025;
  observation.observation_time = ros_time(timestamp_ns);
  observation.gripper_joint_positions = {0.01, 0.02};
  observation.gripper_joint_velocities = {0.001, 0.002};
  observation.source_id = "test/world-state";
  return observation;
}

TEST(WorldStateConfiguration, RejectsInvalidStartupConfiguration)
{
  auto config = short_freshness_config();
  config.planning_frame.clear();
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.event_capacity = 0;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.maximum_observation_age = -1ns;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.clock_type = RCL_CLOCK_UNINITIALIZED;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.joint_lower_limits[2] = config.joint_upper_limits[2] + 0.1;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.joint_limit_tolerance = -0.001;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.joint_limit_tolerance = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.joint_limit_tolerance = 0.0101;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.joint_limit_tolerance = kMaximumJointLimitTolerance;
  EXPECT_NO_THROW(WorldStateStore store(config));

  config = short_freshness_config();
  config.rail_upper_limit = std::numeric_limits<double>::infinity();
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);

  config = short_freshness_config();
  config.gripper_joint_lower_limits[1] = config.gripper_joint_upper_limits[1] + 0.01;
  EXPECT_THROW(WorldStateStore store(config), std::invalid_argument);
}

// Milestone 10 §6 (Card 060): Card 010's SC-004 slot 13 parked shoulder_pan at its
// one-revolution bound with a float excess below twelve printed digits. The motion port clamps
// that excess (maximum start-state correction 0.001 rad); world state refused every such sample
// for two minutes, froze the robot block mid-motion, and recovery read the frozen velocities as
// "still moving". Admission tolerates the port's correction, stores the measured value, and still
// refuses anything beyond it.
TEST(WorldStateRobotTelemetry, AdmitsAJointRestingAtItsBoundWithinTheCorrectionTolerance)
{
  WorldStateStore store(short_freshness_config());
  const double bound = WorldStateConfig{}.joint_upper_limits[0];
  std::int64_t stamp = 9'000'000'000;

  auto at_bound = robot_telemetry(stamp++);
  at_bound.joint_positions[0] = bound + 4.0e-12;
  at_bound.joint_velocities = {};
  const auto admitted = store.observe_robot_telemetry(at_bound, ros_time(10'000'000'000));
  ASSERT_TRUE(admitted) << admitted.error().detail;
  EXPECT_EQ(store.snapshot().robot.joint_positions[0], bound + 4.0e-12)
    << "the measured value is stored, not clamped";

  auto lower = robot_telemetry(stamp++);
  lower.joint_positions[3] = -bound - 0.0009;
  EXPECT_TRUE(store.observe_robot_telemetry(lower, ros_time(10'000'000'000)));

  auto beyond = robot_telemetry(stamp++);
  beyond.joint_positions[0] = bound + 0.0011;
  const auto refused = store.observe_robot_telemetry(beyond, ros_time(10'000'000'000));
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(store.snapshot().robot.telemetry_time.nanoseconds(), stamp - 2)
    << "a refused sample leaves the last admitted one in place";
}

TEST(WorldStateRobotTelemetry, ValidatesFreshnessLimitsAndLeavesRejectedStateUntouched)
{
  WorldStateStore store(short_freshness_config());
  const auto accepted =
    store.observe_robot_telemetry(robot_telemetry(9'000'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(accepted) << accepted.error().detail;
  const auto baseline = store.snapshot();
  EXPECT_EQ(baseline.robot.revision, accepted.value().revision);
  EXPECT_EQ(
    baseline.robot.joint_positions,
    (std::array<double, kArmJointCount>{0.1, -0.2, 0.3, -0.4, 0.5, -0.6}));
  EXPECT_EQ(
    baseline.robot.joint_velocities,
    (std::array<double, kArmJointCount>{0.01, -0.02, 0.03, -0.04, 0.05, -0.06}));
  EXPECT_DOUBLE_EQ(baseline.robot.rail_position, 0.25);
  EXPECT_DOUBLE_EQ(baseline.robot.rail_velocity, 0.025);
  EXPECT_EQ(
    baseline.robot.gripper_joint_positions,
    (std::array<double, kGripperJointCount>{0.01, 0.02}));
  EXPECT_EQ(
    baseline.robot.gripper_joint_velocities,
    (std::array<double, kGripperJointCount>{0.001, 0.002}));
  EXPECT_EQ(baseline.robot.telemetry_time.nanoseconds(), 9'000'000'000);
  EXPECT_EQ(baseline.robot.telemetry_source_id, "test/world-state");
  EXPECT_EQ(baseline.robot.telemetry_revision, accepted.value().revision);
  EXPECT_TRUE(baseline.events.empty());

  WorldStateStore future_boundary_store(short_freshness_config());
  const auto future_boundary = future_boundary_store.observe_robot_telemetry(
    robot_telemetry(10'000'000'000), ros_time(10'000'000'000));
  EXPECT_TRUE(future_boundary) << future_boundary.error().detail;

  const auto duplicate =
    store.observe_robot_telemetry(robot_telemetry(9'000'000'000), ros_time(10'000'000'000));
  ASSERT_FALSE(duplicate);
  EXPECT_EQ(duplicate.error().code, WorldStateErrorCode::OutOfOrder);

  const auto stale =
    store.observe_robot_telemetry(robot_telemetry(8'999'999'999), ros_time(10'000'000'000));
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::StaleObservation);

  const auto future =
    store.observe_robot_telemetry(robot_telemetry(10'000'000'001), ros_time(10'000'000'000));
  ASSERT_FALSE(future);
  EXPECT_EQ(future.error().code, WorldStateErrorCode::FutureObservation);

  auto outside_limits = robot_telemetry(9'100'000'000);
  outside_limits.joint_positions[4] = 3.13;
  const auto invalid = store.observe_robot_telemetry(outside_limits, ros_time(10'000'000'000));
  ASSERT_FALSE(invalid);
  EXPECT_EQ(invalid.error().code, WorldStateErrorCode::InvalidArgument);

  auto invalid_gripper = robot_telemetry(9'100'000'001);
  invalid_gripper.gripper_joint_positions[1] = 0.036;
  const auto rejected_gripper =
    store.observe_robot_telemetry(invalid_gripper, ros_time(10'000'000'000));
  ASSERT_FALSE(rejected_gripper);
  EXPECT_EQ(rejected_gripper.error().code, WorldStateErrorCode::InvalidArgument);

  for (auto invalid_velocity : {
        robot_telemetry(9'100'000'002), robot_telemetry(9'100'000'003),
        robot_telemetry(9'100'000'004)})
  {
    if (invalid_velocity.observation_time.nanoseconds() == 9'100'000'002) {
      invalid_velocity.joint_velocities[2] = std::numeric_limits<double>::quiet_NaN();
    } else if (invalid_velocity.observation_time.nanoseconds() == 9'100'000'003) {
      invalid_velocity.rail_velocity = std::numeric_limits<double>::infinity();
    } else {
      invalid_velocity.gripper_joint_velocities[1] =
        std::numeric_limits<double>::quiet_NaN();
    }
    const auto rejected_velocity =
      store.observe_robot_telemetry(invalid_velocity, ros_time(10'000'000'000));
    ASSERT_FALSE(rejected_velocity);
    EXPECT_EQ(rejected_velocity.error().code, WorldStateErrorCode::InvalidArgument);
  }

  auto changed_source = robot_telemetry(9'100'000'005);
  changed_source.source_id = "test/substituted-source";
  const auto rejected_source =
    store.observe_robot_telemetry(changed_source, ros_time(10'000'000'000));
  ASSERT_FALSE(rejected_source);
  EXPECT_EQ(rejected_source.error().code, WorldStateErrorCode::IdentityConflict);

  const auto after = store.snapshot();
  EXPECT_EQ(after.revision, baseline.revision);
  EXPECT_EQ(after.robot.revision, baseline.robot.revision);
  EXPECT_EQ(after.robot.joint_positions, baseline.robot.joint_positions);
  EXPECT_EQ(after.robot.joint_velocities, baseline.robot.joint_velocities);
  EXPECT_EQ(after.robot.gripper_joint_positions, baseline.robot.gripper_joint_positions);
  EXPECT_EQ(after.robot.gripper_joint_velocities, baseline.robot.gripper_joint_velocities);
  EXPECT_EQ(after.robot.telemetry_source_id, baseline.robot.telemetry_source_id);
  EXPECT_EQ(after.events.size(), baseline.events.size());
}

TEST(WorldStateRobotTelemetry, PreservesAuthoritativeSemanticsAndAttachment)
{
  WorldStateStore store(short_freshness_config());
  const auto initial =
    store.observe_robot_telemetry(robot_telemetry(9'200'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(initial);
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'250'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  const auto attached = store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'300'000'000),
    AttachPreconditions{observed.value().revision, initial.value().revision, std::nullopt});
  ASSERT_TRUE(attached);
  const auto event_count = store.snapshot().events.size();

  const auto telemetry =
    store.observe_robot_telemetry(robot_telemetry(9'400'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(telemetry) << telemetry.error().detail;
  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.robot.held_object, observed.value().object_id);
  EXPECT_EQ(snapshot.robot.task_phase, TaskPhase::Idle);
  EXPECT_EQ(snapshot.robot.fault_state, FaultState::None);
  EXPECT_EQ(snapshot.objects.at(observed.value().object_id).grasp_state, GraspState::Attached);
  EXPECT_EQ(snapshot.events.size(), event_count);
  // The grasp is kept verbatim; telemetry updates do not touch it.
  EXPECT_TRUE(snapshot.robot.grasp_center_from_held_object.isApprox(test_grasp()));

  const auto detached = store.commit_detachment(
    observed.value().object_id, std::nullopt, ros_time(9'500'000'000),
    DetachPreconditions{snapshot.objects.at(observed.value().object_id).revision,
      snapshot.robot.revision, std::nullopt});
  ASSERT_TRUE(detached) << detached.error().detail;
  const auto released = store.snapshot();
  EXPECT_FALSE(released.robot.held_object);
  EXPECT_TRUE(
    released.robot.grasp_center_from_held_object.isApprox(Eigen::Isometry3d::Identity()));
}

TEST(WorldStateAttachment, RefusesAGraspThatIsNotARigidTransform)
{
  WorldStateStore store(short_freshness_config());
  const auto initial =
    store.observe_robot_telemetry(robot_telemetry(9'200'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(initial);
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'250'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);

  Eigen::Isometry3d skewed = test_grasp();
  skewed.linear() *= 1.5;
  const auto refused = store.commit_attachment(
    observed.value().object_id, skewed, ros_time(9'300'000'000),
    AttachPreconditions{observed.value().revision, initial.value().revision, std::nullopt});
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_FALSE(store.snapshot().robot.held_object);
}

TEST(WorldStateObservation, RetainsStableIdentityAndMonotonicRevisions)
{
  WorldStateStore store(short_freshness_config());

  auto first =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(first);
  EXPECT_TRUE(first.value().created);
  EXPECT_TRUE(first.value().object_id);

  auto second =
    store.observe_object(observation("sim/can_01", 9'800'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(second);
  EXPECT_FALSE(second.value().created);
  EXPECT_EQ(second.value().object_id, first.value().object_id);
  EXPECT_GT(second.value().revision, first.value().revision);

  const auto snapshot = store.snapshot();
  ASSERT_EQ(snapshot.objects.size(), 1U);
  EXPECT_EQ(snapshot.objects.at(first.value().object_id).revision, second.value().revision);
  EXPECT_EQ(snapshot.revision, second.value().revision);
}

TEST(WorldStateObservation, RejectsTimeAndFrameErrorsWithoutMutation)
{
  WorldStateStore store(short_freshness_config());

  auto frame_error = observation("sim/can_01", 9'500'000'000);
  frame_error.frame_id = "camera_optical";
  const auto wrong_frame = store.observe_object(frame_error, ros_time(10'000'000'000));
  ASSERT_FALSE(wrong_frame);
  EXPECT_EQ(wrong_frame.error().code, WorldStateErrorCode::FrameMismatch);

  const auto stale =
    store.observe_object(observation("sim/can_01", 8'000'000'000), ros_time(10'000'000'000));
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, WorldStateErrorCode::StaleObservation);

  const auto future =
    store.observe_object(observation("sim/can_01", 10'200'000'000), ros_time(10'000'000'000));
  ASSERT_FALSE(future);
  EXPECT_EQ(future.error().code, WorldStateErrorCode::FutureObservation);

  const auto wrong_clock = store.observe_object(
    observation("sim/can_01", 9'500'000'000),
    rclcpp::Time(10'000'000'000, RCL_SYSTEM_TIME));
  ASSERT_FALSE(wrong_clock);
  EXPECT_EQ(wrong_clock.error().code, WorldStateErrorCode::ClockMismatch);

  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.revision, 0U);
  EXPECT_TRUE(snapshot.objects.empty());
  EXPECT_TRUE(snapshot.events.empty());
}

TEST(WorldStateObservation, RejectsOutOfOrderAndInvalidGeometryWithoutMutation)
{
  WorldStateStore store(short_freshness_config());
  const auto accepted =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(accepted);
  const auto baseline = store.snapshot();

  const auto duplicate =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_FALSE(duplicate);
  EXPECT_EQ(duplicate.error().code, WorldStateErrorCode::OutOfOrder);

  auto invalid_pose = observation("sim/can_01", 9'600'000'000);
  invalid_pose.pose_in_world.translation().x() = std::numeric_limits<double>::quiet_NaN();
  const auto pose_result = store.observe_object(invalid_pose, ros_time(10'000'000'000));
  ASSERT_FALSE(pose_result);
  EXPECT_EQ(pose_result.error().code, WorldStateErrorCode::InvalidArgument);

  auto invalid_covariance = observation("sim/can_01", 9'600'000'000);
  invalid_covariance.pose_covariance(0, 1) = 1.0;
  const auto covariance_result = store.observe_object(invalid_covariance, ros_time(10'000'000'000));
  ASSERT_FALSE(covariance_result);
  EXPECT_EQ(covariance_result.error().code, WorldStateErrorCode::InvalidArgument);

  const auto after = store.snapshot();
  EXPECT_EQ(after.revision, baseline.revision);
  EXPECT_EQ(after.events.size(), baseline.events.size());
  EXPECT_EQ(
    after.objects.at(accepted.value().object_id).observation_time.nanoseconds(),
    9'500'000'000);
}

TEST(WorldStateObservation, AllowsIdentityEnrichmentButRejectsIdentityChanges)
{
  WorldStateStore store(short_freshness_config());
  auto unknown = observation("sim/product_01", 9'400'000'000, ProductClass::Unknown);
  const auto first = store.observe_object(unknown, ros_time(10'000'000'000));
  ASSERT_TRUE(first);

  auto identified = observation("sim/product_01", 9'500'000'000, ProductClass::Can);
  identified.sku = "can/example";
  const auto enriched = store.observe_object(identified, ros_time(10'000'000'000));
  ASSERT_TRUE(enriched);

  auto changed_class = observation("sim/product_01", 9'600'000'000, ProductClass::SmallBottle);
  changed_class.sku = "can/example";
  const auto class_result = store.observe_object(changed_class, ros_time(10'000'000'000));
  ASSERT_FALSE(class_result);
  EXPECT_EQ(class_result.error().code, WorldStateErrorCode::IdentityConflict);

  auto changed_sku = observation("sim/product_01", 9'600'000'000, ProductClass::Can);
  changed_sku.sku = "can/different";
  const auto sku_result = store.observe_object(changed_sku, ros_time(10'000'000'000));
  ASSERT_FALSE(sku_result);
  EXPECT_EQ(sku_result.error().code, WorldStateErrorCode::IdentityConflict);

  const auto object = store.snapshot().objects.at(first.value().object_id);
  EXPECT_EQ(object.product_class, ProductClass::Can);
  EXPECT_EQ(object.sku, std::optional<std::string>("can/example"));
  EXPECT_EQ(object.revision, enriched.value().revision);
}

TEST(WorldStateLifecycle, ReactivatesLostObjectsAndTombstonesRemovedSources)
{
  WorldStateStore store(short_freshness_config());
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);

  const auto lost = store.set_tracking_state(
    observed.value().object_id, TrackingState::Lost, ros_time(9'600'000'000),
    ObjectContextPreconditions{observed.value().revision, std::nullopt});
  ASSERT_TRUE(lost);

  const auto reobserved =
    store.observe_object(observation("sim/can_01", 9'700'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(reobserved);
  EXPECT_EQ(
    store.snapshot().objects.at(observed.value().object_id).tracking_state,
    TrackingState::Tracked);

  const auto removed = store.set_tracking_state(
    observed.value().object_id, TrackingState::Removed, ros_time(9'800'000'000),
    ObjectContextPreconditions{reobserved.value().revision, std::nullopt});
  ASSERT_TRUE(removed);

  const auto resurrect =
    store.observe_object(observation("sim/can_01", 9'900'000'000), ros_time(10'000'000'000));
  ASSERT_FALSE(resurrect);
  EXPECT_EQ(resurrect.error().code, WorldStateErrorCode::Removed);
  EXPECT_EQ(
    store.snapshot().objects.at(observed.value().object_id).revision,
    removed.value().revision);
}

TEST(WorldStateLanes, ValidatesLaneUpdatesAndProductCompatibility)
{
  WorldStateStore store(short_freshness_config());
  const auto lane = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, "can/example", 0.50},
    ros_time(9'000'000'000));
  ASSERT_TRUE(lane);

  const auto updated = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.35, true, ros_time(9'500'000'000)},
    ros_time(10'000'000'000), lane.value().revision);
  ASSERT_TRUE(updated);

  const auto stale_revision = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.30, false, ros_time(9'600'000'000)},
    ros_time(10'000'000'000), lane.value().revision);
  ASSERT_FALSE(stale_revision);
  EXPECT_EQ(stale_revision.error().code, WorldStateErrorCode::RevisionConflict);

  const auto invalid_depth = store.update_lane(
    LaneObservation{LaneId{"lane_01"}, {"sim:can_01"}, 0.60, false, ros_time(9'600'000'000)},
    ros_time(10'000'000'000), updated.value().revision);
  ASSERT_FALSE(invalid_depth);
  EXPECT_EQ(invalid_depth.error().code, WorldStateErrorCode::InvalidArgument);

  const auto snapshot = store.snapshot();
  EXPECT_DOUBLE_EQ(snapshot.lanes.at(LaneId{"lane_01"}).available_depth_m, 0.35);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).obstructed);
  EXPECT_EQ(
    snapshot.lanes.at(LaneId{"lane_01"}).observed_source_object_ids,
    std::vector<std::string>{"sim:can_01"});
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());

  const auto invalid_id_order = store.update_lane(
    LaneObservation{
        LaneId{"lane_01"}, {"sim:can_02", "sim:can_01"}, 0.30, false, ros_time(9'600'000'000)},
    ros_time(10'000'000'000), updated.value().revision);
  ASSERT_FALSE(invalid_id_order);
  EXPECT_EQ(invalid_id_order.error().code, WorldStateErrorCode::InvalidArgument);
  EXPECT_EQ(store.snapshot().revision, updated.value().revision);
}

TEST(WorldStateAttachment, CommitsCrossEntityChangesAtomically)
{
  WorldStateStore store(short_freshness_config());
  const auto lane =
    store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::Can, std::nullopt, 0.50},
    ros_time(9'000'000'000));
  ASSERT_TRUE(lane);
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);

  const auto attached =
    store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'600'000'000),
    AttachPreconditions{observed.value().revision, 0, std::nullopt});
  ASSERT_TRUE(attached);
  auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.robot.held_object, observed.value().object_id);
  EXPECT_EQ(snapshot.robot.revision, attached.value().revision);
  EXPECT_EQ(snapshot.objects.at(observed.value().object_id).grasp_state, GraspState::Attached);
  EXPECT_EQ(snapshot.objects.at(observed.value().object_id).revision, attached.value().revision);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());

  const auto stale_detach = store.commit_detachment(
    observed.value().object_id, LaneId{"lane_01"}, ros_time(9'700'000'000),
    DetachPreconditions{observed.value().revision, attached.value().revision,
      lane.value().revision});
  ASSERT_FALSE(stale_detach);
  EXPECT_EQ(stale_detach.error().code, WorldStateErrorCode::RevisionConflict);
  EXPECT_EQ(store.snapshot().revision, attached.value().revision);

  const auto detached = store.commit_detachment(
    observed.value().object_id, LaneId{"lane_01"}, ros_time(9'700'000'000),
    DetachPreconditions{attached.value().revision, attached.value().revision,
      lane.value().revision});
  ASSERT_TRUE(detached);
  snapshot = store.snapshot();
  EXPECT_FALSE(snapshot.robot.held_object);
  EXPECT_EQ(snapshot.objects.at(observed.value().object_id).grasp_state, GraspState::Free);
  ASSERT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).contents.size(), 1U);
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).contents.front(), observed.value().object_id);
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).revision, detached.value().revision);

  const auto missing_source_precondition = store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'800'000'000),
    AttachPreconditions{detached.value().revision, detached.value().revision, std::nullopt});
  ASSERT_FALSE(missing_source_precondition);
  EXPECT_EQ(missing_source_precondition.error().code, WorldStateErrorCode::RevisionConflict);
  EXPECT_EQ(store.snapshot().revision, detached.value().revision);

  const auto reattached = store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'800'000'000),
    AttachPreconditions{detached.value().revision, detached.value().revision,
      detached.value().revision});
  ASSERT_TRUE(reattached);
  snapshot = store.snapshot();
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).revision, reattached.value().revision);

  const auto detached_again = store.commit_detachment(
    observed.value().object_id, LaneId{"lane_01"}, ros_time(9'900'000'000),
    DetachPreconditions{reattached.value().revision, reattached.value().revision,
      reattached.value().revision});
  ASSERT_TRUE(detached_again);
  const auto removed = store.set_tracking_state(
    observed.value().object_id, TrackingState::Removed, ros_time(10'000'000'000),
    ObjectContextPreconditions{detached_again.value().revision, detached_again.value().revision});
  ASSERT_TRUE(removed);
  snapshot = store.snapshot();
  EXPECT_EQ(snapshot.objects.at(observed.value().object_id).tracking_state, TrackingState::Removed);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
  EXPECT_EQ(snapshot.lanes.at(LaneId{"lane_01"}).revision, removed.value().revision);
}

TEST(WorldStateAttachment, RejectsIncompatibleDestinationWithoutPartialDetach)
{
  WorldStateStore store(short_freshness_config());
  const auto lane = store.configure_lane(
    LaneDefinition{LaneId{"lane_01"}, ProductClass::SmallBottle, std::nullopt, 0.50},
    ros_time(9'000'000'000));
  ASSERT_TRUE(lane);
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  const auto attached =
    store.commit_attachment(
    observed.value().object_id, test_grasp(), ros_time(9'600'000'000),
    AttachPreconditions{observed.value().revision, 0, std::nullopt});
  ASSERT_TRUE(attached);

  const auto rejected = store.commit_detachment(
    observed.value().object_id, LaneId{"lane_01"}, ros_time(9'700'000'000),
    DetachPreconditions{attached.value().revision, attached.value().revision,
      lane.value().revision});
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, WorldStateErrorCode::InvariantViolation);

  const auto snapshot = store.snapshot();
  EXPECT_EQ(snapshot.revision, attached.value().revision);
  EXPECT_EQ(snapshot.robot.held_object, observed.value().object_id);
  EXPECT_TRUE(snapshot.lanes.at(LaneId{"lane_01"}).contents.empty());
}

TEST(WorldStateEvents, BoundsImportantEventsAndSuppressesPureTelemetry)
{
  WorldStateStore store(short_freshness_config(2));
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);

  const auto telemetry = store.observe_robot_telemetry(
    robot_telemetry(9'600'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(telemetry);
  EXPECT_EQ(store.snapshot().events.size(), 1U);

  const auto lost = store.set_tracking_state(
    observed.value().object_id, TrackingState::Lost, ros_time(9'800'000'000),
    ObjectContextPreconditions{observed.value().revision, std::nullopt});
  ASSERT_TRUE(lost);

  const auto snapshot = store.snapshot();
  ASSERT_EQ(snapshot.events.size(), 2U);
  EXPECT_EQ(snapshot.events.front().kind, EventKind::ObjectObserved);
  EXPECT_EQ(snapshot.events.back().kind, EventKind::ObjectTrackingChanged);
  EXPECT_EQ(snapshot.revision, lost.value().revision);
}

TEST(WorldStateSnapshots, AreIndependentValueCopies)
{
  WorldStateStore store(short_freshness_config());
  const auto observed =
    store.observe_object(observation("sim/can_01", 9'500'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(observed);
  auto old_snapshot = store.snapshot();

  auto moved = observation("sim/can_01", 9'600'000'000);
  moved.pose_in_world.translation().x() = 0.25;
  const auto updated = store.observe_object(moved, ros_time(10'000'000'000));
  ASSERT_TRUE(updated);

  EXPECT_DOUBLE_EQ(
    old_snapshot.objects.at(observed.value().object_id).pose_in_world.translation().x(), 0.0);
  old_snapshot.objects.at(observed.value().object_id).source_object_id = "mutated-copy";
  const auto current = store.snapshot();
  EXPECT_DOUBLE_EQ(
    current.objects.at(observed.value().object_id).pose_in_world.translation().x(),
    0.25);
  EXPECT_EQ(current.objects.at(observed.value().object_id).source_object_id, "sim/can_01");
}

TEST(WorldStateConcurrency, PreservesSnapshotInvariantsWithConcurrentReaders)
{
  WorldStateStore store(short_freshness_config(8));
  const auto first =
    store.observe_object(observation("sim/can_01", 9'000'000'000), ros_time(10'000'000'000));
  ASSERT_TRUE(first);

  std::atomic<bool> start{false};
  std::atomic<bool> done{false};
  std::atomic<bool> invalid{false};

  auto reader = [&]() {
    while (!start.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    while (!done.load(std::memory_order_acquire)) {
      const auto value = store.snapshot();
      if (value.objects.size() != 1U) {
        invalid.store(true, std::memory_order_release);
        continue;
      }
      const auto & object = value.objects.begin()->second;
      if (object.revision > value.revision || !object.pose_in_world.matrix().allFinite()) {
        invalid.store(true, std::memory_order_release);
      }
    }
  };

  std::thread reader_one(reader);
  std::thread reader_two(reader);
  start.store(true, std::memory_order_release);

  for (std::int64_t index = 1; index <= 100; ++index) {
    auto next = observation("sim/can_01", 9'000'000'000 + index * 1'000'000);
    next.pose_in_world.translation().x() = static_cast<double>(index) / 1000.0;
    const auto result = store.observe_object(next, ros_time(10'000'000'000));
    if (!result) {
      invalid.store(true, std::memory_order_release);
      break;
    }
  }

  done.store(true, std::memory_order_release);
  reader_one.join();
  reader_two.join();

  EXPECT_FALSE(invalid.load(std::memory_order_acquire));
  const auto snapshot = store.snapshot();
  EXPECT_NEAR(
    snapshot.objects.at(first.value().object_id).pose_in_world.translation().x(), 0.1,
    1e-12);
}

}  // namespace
}  // namespace restocker_world_state
