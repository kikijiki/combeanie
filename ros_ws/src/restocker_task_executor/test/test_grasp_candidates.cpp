// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "restocker_task_executor/grasp_candidates.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::TrackedObject;

[[nodiscard]] TrackedObject can()
{
  TrackedObject object;
  object.id = ObjectId{4};
  object.source_object_id = "sim:can_04";
  object.product_class = ProductClass::Can;
  object.sku = "SIM-CAN-STD";
  object.pose_in_world = Eigen::Isometry3d::Identity();
  object.pose_in_world.translation() = Eigen::Vector3d(0.3, -0.8, 0.631);
  object.orientation = ObjectOrientation::Upright;
  object.observation_time = rclcpp::Time(9'900'000'000LL, RCL_ROS_TIME);
  object.revision = 12;
  return object;
}

[[nodiscard]] SelectedTaskPair selection()
{
  return SelectedTaskPair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.033, 0.122}};
}

[[nodiscard]] ParallelJawGeometry gripper()
{
  return ParallelJawGeometry{0.042, 0.0, 0.035};
}

[[nodiscard]] Eigen::Isometry3d tool_offset()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = 0.14;
  return transform;
}

// Fixture minimum grasp-centre height is 0.010 m above the can's mid-height (0.071 = 0.061
// half-height + 0.010). The camera bound is 0.080, which this short product never reaches.
[[nodiscard]] GraspGenerationConfig config()
{
  return GraspGenerationConfig{
    {0.0, -0.5 * std::acos(-1.0), 0.5 * std::acos(-1.0)},
    0.071, 0.080, 0.10, 0.12, 0.005, 0.001, 0.035, 0.0, 1.0, 1.0, 1.0, 0.05};
}

[[nodiscard]] GraspGenerationAuthority authority()
{
  return GraspGenerationAuthority{gripper(), tool_offset(), config()};
}

[[nodiscard]] restocker_world_state::WorldStateSnapshot snapshot()
{
  restocker_world_state::WorldStateSnapshot value;
  value.revision = 20;
  value.objects.emplace(ObjectId{4}, can());
  restocker_world_state::ShelfLane lane;
  lane.id = LaneId{"lane_01"};
  lane.expected_product_class = ProductClass::Can;
  lane.depth_m = 0.85;
  lane.available_depth_m = 0.85;
  lane.revision = 13U;
  value.lanes.emplace(lane.id, lane);
  value.robot.rail_position = 0.0;
  value.robot.telemetry_time = rclcpp::Time(9'950'000'000LL, RCL_ROS_TIME);
  value.robot.telemetry_source_id = "test/grasp-candidates";
  value.robot.telemetry_revision = 18;
  value.robot.revision = 19;
  return value;
}

[[nodiscard]] restocker_world_state::TaskReservation reservation()
{
  restocker_world_state::TaskReservation value;
  value.reservation_id = 91U;
  value.request_id = "refresh-reservation";
  value.object_id = ObjectId{4};
  value.object_source_id = "sim:can_04";
  value.product_class = ProductClass::Can;
  value.sku = "SIM-CAN-STD";
  value.destination_lane = LaneId{"lane_01"};
  value.stage = restocker_world_state::ReservationStage::Reserved;
  value.created_at = rclcpp::Time(9'960'000'000LL, RCL_ROS_TIME);
  value.created_revision = 21U;
  value.admitted_robot_telemetry_revision = 18U;
  value.revision = 21U;
  return value;
}

[[nodiscard]] restocker_world_state::WorldStateSnapshot reserved_snapshot()
{
  auto value = snapshot();
  value.revision = 21U;
  value.robot.telemetry_time = rclcpp::Time(9'980'000'000LL, RCL_ROS_TIME);
  value.robot.telemetry_revision = 20U;
  value.robot.revision = 20U;
  value.active_reservation = reservation();
  return value;
}

[[nodiscard]] GraspCandidateBatch staged_batch()
{
  const auto generated = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), config(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  if (!generated) {
    throw std::runtime_error(generated.error().detail);
  }
  return generated.value();
}

TEST(GraspCandidates, ProducesPhysicalGripperAxesOffsetsAndToolConversion)
{
  const auto result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), tool_offset(), config());
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 3U);
  const auto & candidate = result.value().front();
  EXPECT_NEAR(candidate.approach_yaw_rad, -0.5 * std::acos(-1.0), 1.0e-12);
  const auto & grasp = candidate.poses.world_from_grasp_center;
  const Eigen::Vector3d approach(
    std::cos(candidate.approach_yaw_rad), std::sin(candidate.approach_yaw_rad), 0.0);

  // The grasp-centre frame is tool0's frame: gripper along the approach, jaws closing horizontally
  // across it, jaw-opening axis along the upright product's axis and pointing up so the wrist
  // camera clears the surface.
  EXPECT_TRUE(grasp_approach_axis(grasp).isApprox(approach, 1.0e-12));
  EXPECT_NEAR(grasp_closing_axis(grasp).dot(approach), 0.0, 1.0e-12);
  EXPECT_NEAR(grasp_closing_axis(grasp).dot(Eigen::Vector3d::UnitZ()), 0.0, 1.0e-12);
  EXPECT_TRUE(grasp_jaw_width_axis(grasp).isApprox(Eigen::Vector3d::UnitZ(), 1.0e-12));
  EXPECT_NEAR(grasp.linear().determinant(), 1.0, 1.0e-12);

  EXPECT_TRUE(grasp.translation().isApprox(Eigen::Vector3d(0.3, -0.8, 0.641), 1.0e-12));
  // Stand-off poses slide backwards along the approach, so they stay at the grasp height.
  EXPECT_TRUE(
    candidate.poses.world_from_pregrasp_center.translation().isApprox(
      grasp.translation() - 0.10 * approach, 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_retract_center.translation().isApprox(
      grasp.translation() - 0.12 * approach, 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_pregrasp_center.translation().isApprox(
      Eigen::Vector3d(0.3, -0.7, 0.641), 1.0e-12));
  // tool0 sits behind the grasp centre along the approach, never below it.
  EXPECT_TRUE(
    candidate.poses.world_from_grasp_tool0.translation().isApprox(
      grasp.translation() - tool_offset().translation().z() * approach, 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_grasp_tool0.translation().isApprox(
      Eigen::Vector3d(0.3, -0.66, 0.641), 1.0e-12));
  EXPECT_TRUE(
    (candidate.poses.world_from_grasp_tool0 * tool_offset()).matrix().isApprox(
      grasp.matrix(), 1.0e-12));
}

// Every catalogued product on the same 0.570 m surface, with the workcell's configured minimum
// grasp-centre height (not the fixture's reduced one).
[[nodiscard]] TrackedObject seated_product(
  ProductClass product_class, const std::string & sku, double height_m)
{
  TrackedObject object = can();
  object.product_class = product_class;
  object.sku = sku;
  object.pose_in_world.translation().z() = 0.570 + 0.5 * height_m;
  return object;
}

[[nodiscard]] GraspGenerationConfig workcell_config()
{
  auto value = config();
  value.minimum_center_height_above_base_m = 0.111;
  return value;
}

TEST(GraspCandidates, DerivesTheGraspHeightFromEachProductOwnHeight)
{
  struct Case
  {
    ProductClass product_class;
    const char * sku;
    double radius_m;
    double height_m;
    double expected_grasp_z;
  };
  // The can is lifted off the tray by the support clearance (its mid-height is too low for the
  // wrist). The bottles clear the tray; the camera bound decides, so each is gripped high enough
  // that its top stays 0.080 m above the grasp centre.
  const Case cases[] = {
    {ProductClass::Can, "SIM-CAN-STD", 0.033, 0.122, 0.570 + 0.111},
    {ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.034, 0.200, 0.570 + 0.120},
    {ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.045, 0.290, 0.570 + 0.210},
  };
  for (const Case & item : cases) {
    const auto object = seated_product(item.product_class, item.sku, item.height_m);
    const SelectedTaskPair pair{
      ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {item.radius_m, item.height_m}};
    const auto result = generate_upright_cylinder_grasps(
      object, pair, gripper(), tool_offset(), workcell_config());
    ASSERT_TRUE(result) << item.sku << ": " << result.error().detail;
    ASSERT_FALSE(result.value().empty()) << item.sku;
    for (const auto & candidate : result.value()) {
      EXPECT_NEAR(
        candidate.poses.world_from_grasp_center.translation().z(), item.expected_grasp_z, 1.0e-12)
        << item.sku;
      // A stand-off slides along the horizontal approach, so it must not change the height.
      EXPECT_NEAR(
        candidate.poses.world_from_pregrasp_center.translation().z(), item.expected_grasp_z,
        1.0e-12) << item.sku;
      EXPECT_NEAR(
        candidate.poses.world_from_retract_center.translation().z(), item.expected_grasp_z,
        1.0e-12) << item.sku;
    }
  }
}

TEST(GraspCandidates, KeepsEveryProductTopBelowTheWristCamera)
{
  // The camera bound applies to the product's top relative to the grasp centre, so assert it
  // there: nothing may stand more than the configured clearance above the jaws.
  const double heights[] = {0.122, 0.200, 0.290, 0.348};
  for (const double height : heights) {
    const SelectedTaskPair pair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.033, height}};
    const auto result = generate_upright_cylinder_grasps(
      seated_product(ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", height), pair, gripper(),
      tool_offset(), workcell_config());
    ASSERT_TRUE(result) << height << ": " << result.error().detail;
    const double product_top = 0.570 + height;
    const double grasp_z = result.value().front().poses.world_from_grasp_center.translation().z();
    EXPECT_LE(product_top - grasp_z, workcell_config().maximum_top_above_center_m + 1.0e-12)
      << height;
    // And never gripped above its top, which would put the fingers on nothing.
    EXPECT_LE(grasp_z, product_top) << height;
  }
}

TEST(GraspCandidates, SizesTheJawsFromEachProductRadiusWithinTheStroke)
{
  // The widest catalogued product can run out of jaw stroke, so its targets are checked against
  // the gripper limit.
  const SelectedTaskPair pair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.045, 0.290}};
  const auto result = generate_upright_cylinder_grasps(
    seated_product(ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.290), pair, gripper(),
    tool_offset(), workcell_config());
  ASSERT_TRUE(result) << result.error().detail;
  const auto & candidate = result.value().front();
  EXPECT_DOUBLE_EQ(candidate.open_joint_position_m, 0.029);
  EXPECT_DOUBLE_EQ(candidate.hold_joint_position_m, 0.025);
  EXPECT_LE(candidate.open_joint_position_m, gripper().joint_upper_m);
}

TEST(GraspCandidates, RejectsAProductShorterThanTheMinimumGraspHeight)
{
  // A grasp centre above the product's top would put the fingers on nothing; this fails instead.
  const SelectedTaskPair pair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.033, 0.100}};
  const auto result = generate_upright_cylinder_grasps(
    seated_product(ProductClass::Can, "SIM-CAN-STD", 0.100), pair, gripper(), tool_offset(),
    workcell_config());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidConfiguration);
}

TEST(GraspCandidates, RejectsAProductTheWorldStateCannotName)
{
  auto object = seated_product(ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.200);
  object.product_class = ProductClass::Unknown;
  const SelectedTaskPair pair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.034, 0.200}};
  const auto result = generate_upright_cylinder_grasps(
    object, pair, gripper(), tool_offset(), workcell_config());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidObject);
}

TEST(GraspCandidates, ComputesPositiveClearanceJawTargetsAndRetainsSelection)
{
  const auto result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), tool_offset(), config());
  ASSERT_TRUE(result) << result.error().detail;
  const auto & candidate = result.value().front();
  EXPECT_DOUBLE_EQ(candidate.open_joint_position_m, 0.017);
  EXPECT_DOUBLE_EQ(candidate.hold_joint_position_m, 0.013);
  EXPECT_GT(candidate.open_joint_position_m, candidate.hold_joint_position_m);
  EXPECT_EQ(candidate.selection.object_id, ObjectId{4});
  EXPECT_EQ(candidate.selection.lane_id, LaneId{"lane_01"});
  EXPECT_EQ(candidate.selection.snapshot_revision, 20U);
}

TEST(GraspCandidates, SortsByStableScoreWithOutwardApproachFirst)
{
  const auto result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), tool_offset(), config());
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 3U);
  // The can sits at negative y, so the arm reaches it only by pointing away from its rail. The
  // opposite approach may park tool0 beyond the product where no arm configuration exists, so the
  // stock-side approach sorts first. The other two keep their score/yaw order.
  EXPECT_NEAR(result.value()[0].approach_yaw_rad, -0.5 * std::acos(-1.0), 1.0e-12);
  EXPECT_NEAR(result.value()[1].approach_yaw_rad, 0.5 * std::acos(-1.0), 1.0e-12);
  EXPECT_NEAR(result.value()[2].approach_yaw_rad, 0.0, 1.0e-12);
  EXPECT_LT(result.value()[0].score.total, result.value()[1].score.total);
  EXPECT_LT(result.value()[1].score.total, result.value()[2].score.total);
  EXPECT_EQ(result.value()[0].source_yaw_index, 1U);

  const Eigen::Vector3d best_approach =
    grasp_approach_axis(result.value()[0].poses.world_from_grasp_center);
  const Eigen::Vector3d reach =
    (can().pose_in_world.translation() - Eigen::Vector3d(
      can().pose_in_world.translation().x(), 0.0,
      0.0))
    .cwiseProduct(Eigen::Vector3d(1.0, 1.0, 0.0)).normalized();
  EXPECT_GT(best_approach.dot(reach), 0.0);
}

TEST(GraspCandidates, PreviousLaneRailPositionCannotBiasDenseBottleToLateralApproach)
{
  auto bottle = seated_product(ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.290);
  bottle.pose_in_world.translation().x() = 0.60;
  auto bottle_config = workcell_config();
  bottle_config.current_rail_position_m = -0.60;
  bottle_config.pregrasp_distance_m = 0.18;
  const SelectedTaskPair pair{
    ObjectId{4}, LaneId{"lane_03"}, 20, 12, 13, {0.045, 0.290}};

  const auto result = generate_upright_cylinder_grasps(
    bottle, pair, gripper(), tool_offset(), bottle_config);

  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 3U);
  // Dense goal-3 setup. The rail aligns to x=0.60 during the motion, so the first grasp must
  // approach from the robot/stock side (-Y), not laterally from x=0.28.
  EXPECT_NEAR(result.value().front().approach_yaw_rad, -0.5 * std::acos(-1.0), 1.0e-12);
  EXPECT_TRUE(
    result.value().front().poses.world_from_pregrasp_center.translation().isApprox(
      Eigen::Vector3d(0.60, -0.62, 0.780), 1.0e-12));
}

TEST(GraspCandidates, RejectsJawLimitsDuplicateYawAndInvalidToolTransform)
{
  auto narrow = gripper();
  narrow.joint_upper_m = 0.01;
  auto narrow_config = config();
  narrow_config.maximum_open_joint_position_m = 0.01;
  const auto jaw_result = generate_upright_cylinder_grasps(
    can(), selection(), narrow, tool_offset(), narrow_config);
  ASSERT_FALSE(jaw_result);
  EXPECT_EQ(jaw_result.error().code, GraspCandidateErrorCode::NoValidJawTarget);

  auto duplicate = config();
  duplicate.approach_yaws_rad = {0.0, 2.0 * std::acos(-1.0)};
  const auto yaw_result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), tool_offset(), duplicate);
  ASSERT_FALSE(yaw_result);
  EXPECT_EQ(yaw_result.error().code, GraspCandidateErrorCode::InvalidConfiguration);

  auto invalid_tool = tool_offset();
  invalid_tool.translation().x() = std::numeric_limits<double>::infinity();
  const auto tool_result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), invalid_tool, config());
  ASSERT_FALSE(tool_result);
  EXPECT_EQ(tool_result.error().code, GraspCandidateErrorCode::InvalidToolTransform);
}

TEST(GraspCandidates, RejectsSelectionRevisionMismatchAndInvalidDistances)
{
  auto mismatched = selection();
  mismatched.object_revision = 99;
  const auto object_result = generate_upright_cylinder_grasps(
    can(), mismatched, gripper(), tool_offset(), config());
  ASSERT_FALSE(object_result);
  EXPECT_EQ(object_result.error().code, GraspCandidateErrorCode::InvalidObject);

  auto invalid_config = config();
  invalid_config.pregrasp_distance_m = 0.0;
  const auto config_result = generate_upright_cylinder_grasps(
    can(), selection(), gripper(), tool_offset(), invalid_config);
  ASSERT_FALSE(config_result);
  EXPECT_EQ(config_result.error().code, GraspCandidateErrorCode::InvalidConfiguration);

  auto tilted = can();
  tilted.pose_in_world.linear() =
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto tilt_result = generate_upright_cylinder_grasps(
    tilted, selection(), gripper(), tool_offset(), config());
  ASSERT_FALSE(tilt_result);
  EXPECT_EQ(tilt_result.error().code, GraspCandidateErrorCode::InvalidObject);
}

TEST(GraspCandidates, BatchRetainsWorldObjectAndRobotTelemetryLineage)
{
  const auto result = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), config(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().source_world_revision, 20U);
  EXPECT_EQ(result.value().source_object_observation_time.nanoseconds(), 9'900'000'000LL);
  EXPECT_EQ(result.value().source_robot_telemetry_revision, 18U);
  EXPECT_EQ(result.value().source_robot_telemetry_time.nanoseconds(), 9'950'000'000LL);
  const auto validated = validate_grasp_candidate_batch(
    result.value(), snapshot(), selection(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  EXPECT_TRUE(validated) << validated.error().detail;
}

TEST(GraspCandidates, BatchRejectsStaleFutureAndMismatchedTelemetryEvidence)
{
  const auto now = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  auto stale = snapshot();
  stale.objects.at(ObjectId{4}).observation_time = rclcpp::Time(
    9'000'000'000LL, RCL_ROS_TIME);
  auto result = generate_grasp_candidate_batch(
    stale, selection(), gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::StaleEvidence);

  auto future = snapshot();
  future.robot.telemetry_time = rclcpp::Time(10'100'000'000LL, RCL_ROS_TIME);
  result = generate_grasp_candidate_batch(
    future, selection(), gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::StaleEvidence);

  auto wrong_rail = config();
  wrong_rail.current_rail_position_m = 0.1;
  result = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), wrong_rail, now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

TEST(GraspCandidates, BatchRejectsCandidateMutationAndExcessiveCount)
{
  const auto now = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  auto generated = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(generated);
  generated.value().candidates.front().score.total += 0.01;
  auto validated = validate_grasp_candidate_batch(
    generated.value(), snapshot(), selection(), authority(), now,
    std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(validated);
  EXPECT_EQ(validated.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);

  generated = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(generated);
  generated.value().candidates.resize(17U, generated.value().candidates.front());
  validated = validate_grasp_candidate_batch(
    generated.value(), snapshot(), selection(), authority(), now,
    std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(validated);
  EXPECT_EQ(validated.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

TEST(GraspCandidates, BatchRejectsCoherentSubstitutionOfEveryAuthoritativeInput)
{
  using Mutation = std::pair<std::string, std::function<void (GraspGenerationAuthority &)>>;
  const std::vector<Mutation> mutations{
    {"inner gap", [](auto & value) {value.gripper.inner_gap_at_zero_m -= 0.001;}},
    {"jaw lower", [](auto & value) {value.gripper.joint_lower_m += 0.001;}},
    {"jaw upper", [](auto & value) {value.gripper.joint_upper_m += 0.001;}},
    {"tool transform", [](auto & value) {
        value.tool0_from_grasp_center.translation().x() += 0.001;
      }},
    {"approach yaws", [](auto & value) {value.config.approach_yaws_rad[0] += 0.1;}},
    {"minimum grasp-centre height",
      [](auto & value) {value.config.minimum_center_height_above_base_m += 0.001;}},
    {"pregrasp distance", [](auto & value) {value.config.pregrasp_distance_m += 0.001;}},
    {"retract distance", [](auto & value) {value.config.retract_distance_m += 0.001;}},
    {"open clearance", [](auto & value) {
        value.config.open_clearance_per_side_m += 0.001;
      }},
    {"hold clearance", [](auto & value) {
        value.config.hold_clearance_per_side_m += 0.0001;
      }},
    {"maximum open", [](auto & value) {
        value.config.maximum_open_joint_position_m -= 0.001;
      }},
    {"rail travel weight", [](auto & value) {value.config.rail_travel_weight += 0.1;}},
    {"rear access weight", [](auto & value) {value.config.rear_access_weight += 0.1;}},
    {"wrist clearance weight", [](auto & value) {
        value.config.wrist_clearance_weight += 0.1;
      }},
    {"upright tilt", [](auto & value) {value.config.maximum_upright_tilt_rad += 0.01;}},
  };
  const auto expected_authority = authority();
  const auto now = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  for (const auto & [name, mutate] : mutations) {
    auto substituted = expected_authority;
    mutate(substituted);
    auto generated = generate_grasp_candidate_batch(
      snapshot(), selection(), substituted.gripper, substituted.tool0_from_grasp_center,
      substituted.config, now, std::chrono::milliseconds(500),
      std::chrono::milliseconds(500), std::chrono::milliseconds(50));
    ASSERT_TRUE(generated) << name << ": " << generated.error().detail;
    const auto validated = validate_grasp_candidate_batch(
      generated.value(), snapshot(), selection(), expected_authority, now,
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50));
    ASSERT_FALSE(validated) << name;
    EXPECT_EQ(validated.error().code, GraspCandidateErrorCode::InvalidCandidateBatch) << name;
  }

  auto substituted_rail = generate_grasp_candidate_batch(
    snapshot(), selection(), gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(substituted_rail);
  substituted_rail.value().config.current_rail_position_m += 0.01;
  const auto validated_rail = validate_grasp_candidate_batch(
    substituted_rail.value(), snapshot(), selection(), expected_authority, now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(validated_rail);
  EXPECT_EQ(validated_rail.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

TEST(GraspCandidates, RejectsNonFiniteAndOverflowedCandidateData)
{
  const auto now = rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME);
  const auto expected_authority = authority();
  const auto rejects_mutation = [&](const auto & mutate) {
    auto generated = generate_grasp_candidate_batch(
      snapshot(), selection(), gripper(), tool_offset(), config(), now,
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50));
    EXPECT_TRUE(generated);
    if (!generated) {
      return;
    }
    mutate(generated.value().candidates.front());
    const auto validated = validate_grasp_candidate_batch(
      generated.value(), snapshot(), selection(), expected_authority, now,
      std::chrono::milliseconds(500), std::chrono::milliseconds(500),
      std::chrono::milliseconds(50));
    EXPECT_FALSE(validated);
    if (!validated) {
      EXPECT_EQ(validated.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
    }
  };
  rejects_mutation(
    [](auto & value) {
      value.poses.world_from_grasp_tool0.translation().x() =
      std::numeric_limits<double>::infinity();
    });
  rejects_mutation(
    [](auto & value) {
      value.open_joint_position_m = std::numeric_limits<double>::quiet_NaN();
    });
  rejects_mutation(
    [](auto & value) {
      value.score.total = std::numeric_limits<double>::infinity();
    });

  auto overflow_snapshot = snapshot();
  overflow_snapshot.robot.rail_position = -std::numeric_limits<double>::max();
  auto overflow_config = config();
  overflow_config.current_rail_position_m = overflow_snapshot.robot.rail_position;
  overflow_config.rail_travel_weight = std::numeric_limits<double>::max();
  const auto overflow = generate_grasp_candidate_batch(
    overflow_snapshot, selection(), gripper(), tool_offset(), overflow_config, now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(overflow);
  EXPECT_EQ(overflow.error().code, GraspCandidateErrorCode::InvalidConfiguration);

  auto overflow_selection = selection();
  overflow_selection.product_envelope.radius_m = std::numeric_limits<double>::max();
  const auto jaw_overflow = generate_upright_cylinder_grasps(
    can(), overflow_selection, gripper(), tool_offset(), config());
  ASSERT_FALSE(jaw_overflow);
  EXPECT_EQ(jaw_overflow.error().code, GraspCandidateErrorCode::InvalidConfiguration);
}

TEST(GraspCandidates, RefreshesStagedIntentAgainstExactReservationSnapshot)
{
  const auto staged = staged_batch();
  auto fresh = reserved_snapshot();
  fresh.robot.joint_positions = {0.10, -0.20, 0.30, -0.40, 0.50, -0.60};
  fresh.robot.joint_velocities = {};
  fresh.robot.rail_velocity = 0.0;
  fresh.robot.gripper_joint_positions = {0.01, 0.01};
  fresh.robot.gripper_joint_velocities = {};
  const auto result = refresh_pregrasp_candidate_batch(
    staged, fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));

  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(staged.selection.snapshot_revision, 20U);
  EXPECT_EQ(staged.source_world_revision, 20U);
  EXPECT_EQ(result.value().selection.snapshot_revision, 21U);
  EXPECT_EQ(result.value().source_world_revision, 21U);
  EXPECT_EQ(result.value().source_robot_telemetry_revision, 20U);
  ASSERT_FALSE(result.value().candidates.empty());
  EXPECT_EQ(result.value().candidates.front().selection.snapshot_revision, 21U);
  EXPECT_EQ(
    result.value().candidates.front().source_yaw_index,
    staged.candidates.front().source_yaw_index);
  EXPECT_TRUE(
    (result.value().candidates.front().poses.world_from_pregrasp_tool0.matrix().array() ==
    staged.candidates.front().poses.world_from_pregrasp_tool0.matrix().array()).all());
}

TEST(GraspCandidates, RefreshRejectsMissingOrSubstitutedReservation)
{
  auto fresh = reserved_snapshot();
  fresh.active_reservation.reset();
  auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::ReservationMismatch);

  fresh = reserved_snapshot();
  fresh.active_reservation->request_id = "substituted";
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::ReservationMismatch);
}

TEST(GraspCandidates, RefreshRejectsSelectionRevisionOrObservationChanges)
{
  auto fresh = reserved_snapshot();
  fresh.objects.at(ObjectId{4}).revision += 1U;
  auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::SelectionChanged);

  fresh = reserved_snapshot();
  fresh.lanes.at(LaneId{"lane_01"}).revision += 1U;
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::SelectionChanged);

  fresh = reserved_snapshot();
  fresh.objects.at(ObjectId{4}).observation_time =
    rclcpp::Time(9'990'000'000LL, RCL_ROS_TIME);
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::SelectionChanged);
}

TEST(GraspCandidates, RefreshRejectsUnchangedRevisionAuthorityContradictions)
{
  using Mutation = std::pair<
    std::string,
    std::function<void (restocker_world_state::WorldStateSnapshot &)>>;
  const std::vector<Mutation> mutations{
    {"object identity", [](auto & value) {
        value.objects.at(ObjectId{4}).id = ObjectId{9};
      }},
    {"lane identity", [](auto & value) {
        value.lanes.at(LaneId{"lane_01"}).id = LaneId{"lane_02"};
      }},
    {"source identity", [](auto & value) {
        value.objects.at(ObjectId{4}).source_object_id = "sim:substituted";
      }},
    {"product class", [](auto & value) {
        value.objects.at(ObjectId{4}).product_class = ProductClass::SmallBottle;
      }},
    {"SKU", [](auto & value) {
        value.objects.at(ObjectId{4}).sku = "SIM-CAN-OTHER";
      }},
    {"tracking state", [](auto & value) {
        value.objects.at(ObjectId{4}).tracking_state =
          restocker_world_state::TrackingState::Occluded;
      }},
    {"grasp state", [](auto & value) {
        value.objects.at(ObjectId{4}).grasp_state =
          restocker_world_state::GraspState::Attached;
      }},
    {"orientation", [](auto & value) {
        value.objects.at(ObjectId{4}).orientation = ObjectOrientation::Horizontal;
      }},
    {"held object", [](auto & value) {value.robot.held_object = ObjectId{4};}},
  };

  for (const auto & [name, mutate] : mutations) {
    auto fresh = reserved_snapshot();
    mutate(fresh);
    const auto result = refresh_pregrasp_candidate_batch(
      staged_batch(), fresh, reservation(), authority(),
      rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
      std::chrono::milliseconds(500), std::chrono::milliseconds(50));
    ASSERT_FALSE(result) << name;
    EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch) << name;
  }
}

TEST(GraspCandidates, RefreshRejectsMalformedReservationRevisionAuthority)
{
  using Mutation = std::pair<
    std::string, std::function<void (restocker_world_state::TaskReservation &)>>;
  const std::vector<Mutation> mutations{
    {"zero created revision", [](auto & value) {value.created_revision = 0U;}},
    {"creation predates staging", [](auto & value) {value.created_revision = 20U;}},
    {"creation exceeds reservation", [](auto & value) {value.created_revision = 22U;}},
    {"zero admitted telemetry", [](auto & value) {
        value.admitted_robot_telemetry_revision = 0U;
      }},
    {"admitted telemetry exceeds fresh state", [](auto & value) {
        value.admitted_robot_telemetry_revision = 21U;
      }},
  };

  for (const auto & [name, mutate] : mutations) {
    auto expected = reservation();
    mutate(expected);
    const auto result = refresh_pregrasp_candidate_batch(
      staged_batch(), reserved_snapshot(), expected, authority(),
      rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
      std::chrono::milliseconds(500), std::chrono::milliseconds(50));
    ASSERT_FALSE(result) << name;
    EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch) << name;
  }
}

TEST(GraspCandidates, RefreshRejectsRobotTelemetryRegression)
{
  auto fresh = reserved_snapshot();
  fresh.robot.telemetry_revision = 17U;
  auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);

  fresh = reserved_snapshot();
  fresh.robot.telemetry_time = rclcpp::Time(9'940'000'000LL, RCL_ROS_TIME);
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);

  fresh = reserved_snapshot();
  fresh.robot.telemetry_time = rclcpp::Time(9'980'000'000LL, RCL_SYSTEM_TIME);
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

TEST(GraspCandidates, RefreshRejectsCandidateZeroIntentDrift)
{
  auto forged_fresh = reserved_snapshot();
  forged_fresh.objects.at(ObjectId{4}).pose_in_world.translation().x() += 0.01;
  auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), forged_fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::CandidateIntentChanged);

  auto substituted_stage = staged_batch();
  substituted_stage.candidates.front().score.total += 0.01;
  result = refresh_pregrasp_candidate_batch(
    substituted_stage, reserved_snapshot(), reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::CandidateIntentChanged);
}

TEST(GraspCandidates, RefreshRejectsGenerationAuthoritySubstitution)
{
  auto substituted_authority = authority();
  substituted_authority.config.rear_access_weight += 0.01;

  const auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), reserved_snapshot(), reservation(), substituted_authority,
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME), std::chrono::milliseconds(500),
    std::chrono::milliseconds(500), std::chrono::milliseconds(50));

  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

TEST(GraspCandidates, RefreshPropagatesFreshnessAndRailAuthorityFailures)
{
  auto fresh = reserved_snapshot();
  auto result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'600'000'001LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(700),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::StaleEvidence);

  fresh = reserved_snapshot();
  fresh.robot.rail_position = 0.01;
  result = refresh_pregrasp_candidate_batch(
    staged_batch(), fresh, reservation(), authority(),
    rclcpp::Time(10'000'000'000LL, RCL_ROS_TIME),
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, GraspCandidateErrorCode::InvalidCandidateBatch);
}

// Card 029 SC-001(e), the plan ≡ scene half: the snapshots the reserved-pose hold produces —
// product pose frozen, world/object revisions and the observation stamp advanced — must
// regenerate the staged grasp exactly, so the frozen target and the obstacle the projector
// reads from the same snapshot cannot drift apart. When the divergence bound trips and the pose
// is applied, the regenerated grasp must move with it: the hold never hides a real movement.
TEST(GraspCandidates, HeldPoseSnapshotsRegenerateTheStagedGraspAndDivergedPosesMoveIt)
{
  const auto staged = staged_batch();
  const rclcpp::Time now(10'000'000'000LL, RCL_ROS_TIME);

  auto held = reserved_snapshot();
  held.revision = 24U;
  held.robot.telemetry_revision = 22U;
  held.robot.telemetry_time = rclcpp::Time(9'985'000'000LL, RCL_ROS_TIME);
  held.robot.revision = 23U;
  auto & held_object = held.objects.at(ObjectId{4});
  held_object.revision = 14U;
  held_object.observation_time = rclcpp::Time(9'950'000'000LL, RCL_ROS_TIME);
  // pose_in_world untouched: exactly what observe_object admits while holding the pose.

  auto held_selection = selection();
  held_selection.snapshot_revision = held.revision;
  held_selection.object_revision = held_object.revision;

  const auto regenerated = generate_grasp_candidate_batch(
    held, held_selection, gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(regenerated) << regenerated.error().detail;
  ASSERT_EQ(regenerated.value().candidates.size(), staged.candidates.size());
  for (std::size_t index = 0; index < staged.candidates.size(); ++index) {
    EXPECT_TRUE(
      regenerated.value().candidates[index].poses.world_from_grasp_center.matrix().isApprox(
        staged.candidates[index].poses.world_from_grasp_center.matrix(), 1.0e-12)) << index;
    EXPECT_TRUE(
      regenerated.value().candidates[index].poses.world_from_pregrasp_center.matrix().isApprox(
        staged.candidates[index].poses.world_from_pregrasp_center.matrix(), 1.0e-12)) << index;
    EXPECT_TRUE(
      regenerated.value().candidates[index].poses.world_from_grasp_tool0.matrix().isApprox(
        staged.candidates[index].poses.world_from_grasp_tool0.matrix(), 1.0e-12)) << index;
    EXPECT_EQ(
      regenerated.value().candidates[index].approach_yaw_rad,
      staged.candidates[index].approach_yaw_rad) << index;
  }
  const auto validated = validate_grasp_candidate_batch(
    regenerated.value(), held, held_selection, authority(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(validated) << validated.error().detail;

  // Divergence applied (the bound tripped and world state replaced the pose): every candidate's
  // target moves with the product, so a re-staged grasp aims where the product now is.
  auto diverged = held;
  diverged.revision = 25U;
  auto & moved_object = diverged.objects.at(ObjectId{4});
  moved_object.revision = 15U;
  moved_object.pose_in_world.translation().x() += 0.05;
  auto diverged_selection = held_selection;
  diverged_selection.snapshot_revision = diverged.revision;
  diverged_selection.object_revision = moved_object.revision;
  const auto moved_batch = generate_grasp_candidate_batch(
    diverged, diverged_selection, gripper(), tool_offset(), config(), now,
    std::chrono::milliseconds(500), std::chrono::milliseconds(500),
    std::chrono::milliseconds(50));
  ASSERT_TRUE(moved_batch) << moved_batch.error().detail;
  ASSERT_EQ(moved_batch.value().candidates.size(), staged.candidates.size());
  for (std::size_t index = 0; index < staged.candidates.size(); ++index) {
    EXPECT_NEAR(
      moved_batch.value().candidates[index].poses.world_from_grasp_center.translation().x(),
      staged.candidates[index].poses.world_from_grasp_center.translation().x() + 0.05,
      1.0e-9) << index;
    EXPECT_NEAR(
      moved_batch.value().candidates[index].poses.world_from_grasp_center.translation().y(),
      staged.candidates[index].poses.world_from_grasp_center.translation().y(),
      1.0e-12) << index;
  }
}

}  // namespace
}  // namespace restocker_task_executor
