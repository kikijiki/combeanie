// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_task_executor/task_selection.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using restocker_world_state::GraspState;
using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ShelfLane;
using restocker_world_state::TrackedObject;
using restocker_world_state::TrackingState;
using restocker_world_state::WorldStateSnapshot;

void expect_error(
  const SelectionResult<SelectedTaskPair> & result, SelectionErrorCode expected)
{
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, expected);
}

[[nodiscard]] rclcpp::Time ros_time(std::int64_t nanoseconds)
{
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

class TaskSelectionTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto loaded_catalog = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
    ASSERT_TRUE(loaded_catalog) << loaded_catalog.error().detail;
    catalog_ = loaded_catalog.value();
    const auto loaded_geometry = load_manipulation_geometry(RESTOCKER_TEST_WORKCELL_GEOMETRY);
    ASSERT_TRUE(loaded_geometry) << loaded_geometry.error().detail;
    geometry_ = loaded_geometry.value();
    world_from_shelf_.translation() = Eigen::Vector3d(0.0, 0.55, 0.75);
  }

  [[nodiscard]] TrackedObject can(ObjectId id, double shelf_x) const
  {
    TrackedObject object;
    object.id = id;
    object.source_object_id = "sim:can_" + std::to_string(id.value);
    object.product_class = ProductClass::Can;
    object.sku = "SIM-CAN-STD";
    object.pose_in_world = world_from_shelf_ * Eigen::Translation3d(shelf_x, -1.35, -0.10);
    object.pose_covariance.setIdentity();
    object.orientation = ObjectOrientation::Upright;
    object.tracking_state = TrackingState::Tracked;
    object.grasp_state = GraspState::Free;
    object.observation_time = ros_time(9'800'000'000);
    object.transition_time = object.observation_time;
    object.revision = id.value + 2;
    return object;
  }

  // Seated on the stock tray floor: the stock containment predicate is judged against the product's
  // own envelope, so a taller product at the can's centre height would be rejected for the wrong
  // reason.
  [[nodiscard]] TrackedObject seated_product(
    ObjectId id, double shelf_x, ProductClass product_class, const std::string & sku,
    double height_m) const
  {
    TrackedObject object = can(id, shelf_x);
    object.product_class = product_class;
    object.sku = sku;
    object.pose_in_world =
      world_from_shelf_ * Eigen::Translation3d(shelf_x, -1.35, -0.18 + 0.5 * height_m);
    return object;
  }

  [[nodiscard]] ShelfLane lane(const std::string & id, std::uint64_t revision) const
  {
    return ShelfLane{
      LaneId{id}, ProductClass::Can, "SIM-CAN-STD", 6, {}, {}, 0.85, 0.85, false,
      ros_time(9'900'000'000), false, ros_time(0), revision, false, revision};
  }

  // A snapshot with one can and the named lanes, all empty, fresh and compatible with it; only
  // rail position separates the lanes.
  [[nodiscard]] WorldStateSnapshot one_can_with_lanes(
    double object_shelf_x, const std::vector<std::string> & lane_ids) const
  {
    WorldStateSnapshot value;
    value.revision = 20;
    value.objects.emplace(ObjectId{1}, can(ObjectId{1}, object_shelf_x));
    std::uint64_t revision = 11;
    for (const std::string & id : lane_ids) {
      value.lanes.emplace(LaneId{id}, lane(id, revision));
      ++revision;
    }
    value.robot.telemetry_time = ros_time(9'950'000'000);
    value.robot.telemetry_source_id = "test/task-selection";
    value.robot.telemetry_revision = 19;
    value.robot.revision = 19;
    return value;
  }

  // Lane centre on the carriage's travel axis (world x, the rail position axis).
  [[nodiscard]] double lane_x_in_world(const std::string & id) const
  {
    return (world_from_shelf_ *
           Eigen::Vector3d(geometry_.lanes.at(id).center_x_m, 0.0, 0.0)).x();
  }

  [[nodiscard]] WorldStateSnapshot snapshot() const
  {
    WorldStateSnapshot value;
    value.revision = 20;
    value.objects.emplace(ObjectId{2}, can(ObjectId{2}, 0.25));
    value.objects.emplace(ObjectId{1}, can(ObjectId{1}, -0.25));
    value.lanes.emplace(LaneId{"lane_02"}, lane("lane_02", 12));
    value.lanes.emplace(LaneId{"lane_01"}, lane("lane_01", 11));
    value.robot.telemetry_time = ros_time(9'950'000'000);
    value.robot.telemetry_source_id = "test/task-selection";
    value.robot.telemetry_revision = 19;
    value.robot.revision = 19;
    return value;
  }

  [[nodiscard]] SelectionConfig config() const
  {
    SelectionConfig value;
    value.now = ros_time(10'000'000'000);
    value.maximum_object_age = 500ms;
    value.lane_evidence_validity = 60s;
    value.maximum_robot_age = 500ms;
    value.maximum_future_skew = 50ms;
    value.stock_containment_margin_m = 0.0;
    value.maximum_upright_tilt_rad = 0.05;
    return value;
  }

  ProductCollisionCatalog catalog_;
  WorkcellManipulationGeometry geometry_;
  Eigen::Isometry3d world_from_shelf_{Eigen::Isometry3d::Identity()};
};

TEST_F(TaskSelectionTest, ChoosesTheCheapestPairAndRetainsRevisions)
{
  // Both cans and lanes are eligible with equal deficits (both lanes empty at target 6), so the
  // §4 tie-break decides: rail distance from the carriage. The carriage starts at the rail
  // origin; lane_02's station sits 0.6 m away against lane_01's 1.0 m — and total travel agrees,
  // so cost and deficit policy point the same way. lane_01 is what a container-ordered search
  // would return.
  const auto result = select_task_pair(
    snapshot(), catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().object_id, ObjectId{1});
  EXPECT_EQ(result.value().lane_id, LaneId{"lane_02"});
  EXPECT_EQ(result.value().snapshot_revision, 20U);
  EXPECT_EQ(result.value().object_revision, 3U);
  EXPECT_EQ(result.value().lane_revision, 12U);
  EXPECT_DOUBLE_EQ(result.value().product_envelope.radius_m, 0.033);
  EXPECT_DOUBLE_EQ(result.value().product_envelope.height_m, 0.122);
  EXPECT_DOUBLE_EQ(result.value().score.approach_rail_travel_m, 0.25);
  EXPECT_DOUBLE_EQ(result.value().score.delivery_rail_travel_m, 0.35);
  EXPECT_DOUBLE_EQ(result.value().score.total, 0.60);
  EXPECT_EQ(result.value().destination_deficit, 6U);
  EXPECT_DOUBLE_EQ(result.value().destination_rail_distance_m, 0.6);
}

TEST_F(TaskSelectionTest, PrefersTheLaneNearerTheCarriageFromEitherEndOfTheRail)
{
  // Deficits tie (both lanes empty at target 6), so Milestone 10 §4 breaks the tie on rail
  // distance from the arm's current position — the carriage's position decides, not the
  // product's. Left end picks lane_01, right end lane_04; asserting both directions
  // distinguishes "nearer to the carriage" from "lane whose ID sorts first" and from the
  // object-relative cost the ordering replaced.
  auto left = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  left.robot.rail_position = -1.2;
  const auto near_lane_01 = select_task_pair(
    left, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(near_lane_01) << near_lane_01.error().detail;
  EXPECT_EQ(near_lane_01.value().lane_id, LaneId{"lane_01"});
  EXPECT_DOUBLE_EQ(near_lane_01.value().destination_rail_distance_m, 0.2);

  auto right = one_can_with_lanes(0.60, {"lane_01", "lane_04"});
  right.robot.rail_position = 1.2;
  const auto near_lane_04 = select_task_pair(
    right, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(near_lane_04) << near_lane_04.error().detail;
  EXPECT_EQ(near_lane_04.value().lane_id, LaneId{"lane_04"});
  EXPECT_DOUBLE_EQ(near_lane_04.value().destination_rail_distance_m, 1.0);
}

TEST_F(TaskSelectionTest, BreaksAnExactCostTieOnIdentityRatherThanVisitOrder)
{
  // A can midway between lane_02 and lane_05 reaches both with identical travel (the same sum of
  // the same magnitudes, so an exact tie). The identity tie-break decides: lane_02 must win, and
  // keep winning when the later-visited lane_05 ties, which a `<=` comparison would get wrong.
  const auto value = one_can_with_lanes(0.0, {"lane_02", "lane_05"});
  ASSERT_DOUBLE_EQ(
    std::abs(lane_x_in_world("lane_02")), std::abs(lane_x_in_world("lane_05")));
  const auto result = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().lane_id, LaneId{"lane_02"});

  // The same snapshot must always yield the same pair: the coordinator reserves against the pair
  // and revalidates it later, so wandering between equal-cost pairs would invalidate the
  // reservation.
  for (int repeat = 0; repeat < 8; ++repeat) {
    const auto again = select_task_pair(
      value, catalog_, geometry_, world_from_shelf_, config());
    ASSERT_TRUE(again) << again.error().detail;
    EXPECT_EQ(again.value().object_id, result.value().object_id);
    EXPECT_EQ(again.value().lane_id, result.value().lane_id);
    EXPECT_DOUBLE_EQ(again.value().score.total, result.value().score.total);
  }
}

TEST_F(TaskSelectionTest, BreaksAnExactRailCostTieOnTheFrontmostObject)
{
  // Two cans in a tray feed queue occupy the same rail point, so travel cannot separate them. The
  // +Y/front product must be removed before the one behind it has a collision-free straight grasp
  // approach, even when the rear product has the lower ID.
  auto value = one_can_with_lanes(0.0, {"lane_03"});
  value.objects.clear();
  TrackedObject in_front = can(ObjectId{7}, 0.30);
  in_front.pose_in_world.translation().y() += 0.15;
  TrackedObject behind = can(ObjectId{4}, 0.30);
  behind.pose_in_world.translation().y() -= 0.15;
  value.objects.emplace(in_front.id, in_front);
  value.objects.emplace(behind.id, behind);
  const auto result = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().object_id, ObjectId{7});
}

TEST_F(TaskSelectionTest, ClearsAFrontGridRowBeforeChoosingACheaperRearColumn)
{
  // The carriage is already aligned with the rear can, making it much cheaper than the front can in
  // the other column. The front row must still drain first to leave the arm a clear approach
  // corridor.
  auto value = one_can_with_lanes(0.60, {"lane_04"});
  value.objects.clear();
  TrackedObject front = can(ObjectId{7}, -0.30);
  front.pose_in_world.translation().y() += 0.15;
  TrackedObject cheap_rear = can(ObjectId{4}, 0.60);
  cheap_rear.pose_in_world.translation().y() -= 0.15;
  value.objects.emplace(front.id, front);
  value.objects.emplace(cheap_rear.id, cheap_rear);

  const auto result = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().object_id, front.id);
  EXPECT_GT(result.value().score.total, 0.0);
}

TEST_F(TaskSelectionTest, RoutesAroundAnObstructedLaneToItsCompatibleSibling)
{
  // Two lanes per class, so an obstruction in one sends the product to its sibling. The carriage
  // waits beside lane_01 (rail -1.2), so equal deficits fall to rail distance and lane_01 is the
  // baseline destination.
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  value.robot.rail_position = -1.2;
  ASSERT_EQ(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config()).value().lane_id,
    LaneId{"lane_01"});

  value.lanes.at(LaneId{"lane_01"}).obstructed = true;
  const auto rerouted = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(rerouted) << rerouted.error().detail;
  EXPECT_EQ(rerouted.value().lane_id, LaneId{"lane_04"});

  // Occupancy alone never skips a lane. With the sibling obstructed, the lane already holding a
  // product — and room behind the column for one more — is still the destination.
  auto occupied = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  occupied.robot.rail_position = -1.2;
  occupied.lanes.at(LaneId{"lane_01"}).observed_source_object_ids = {"sim:already_here"};
  occupied.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.734;
  occupied.lanes.at(LaneId{"lane_04"}).obstructed = true;
  const auto behind_the_column = select_task_pair(
    occupied, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(behind_the_column) << behind_the_column.error().detail;
  EXPECT_EQ(behind_the_column.value().lane_id, LaneId{"lane_01"});

  // With a free sibling, §4's ordering sends the product to the emptier lane: lane_04's six
  // missing cans outweigh lane_01's four, even though lane_01 is the one beside the carriage.
  auto deficit_first = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  deficit_first.robot.rail_position = -1.2;
  deficit_first.lanes.at(LaneId{"lane_01"}).observed_source_object_ids = {"sim:already_here"};
  deficit_first.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.734;
  const auto emptier_lane_wins = select_task_pair(
    deficit_first, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(emptier_lane_wins) << emptier_lane_wins.error().detail;
  EXPECT_EQ(emptier_lane_wins.value().lane_id, LaneId{"lane_04"});
  EXPECT_EQ(emptier_lane_wins.value().destination_deficit, 6U);

  // What sends it to the sibling when there is no deficit left to win on: the lane running out
  // of depth past where the can would have to be released.
  auto full = occupied;
  full.lanes.at(LaneId{"lane_04"}).obstructed = false;
  full.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.05;
  const auto around_full = select_task_pair(
    full, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(around_full) << around_full.error().detail;
  EXPECT_EQ(around_full.value().lane_id, LaneId{"lane_04"});

  // Simulator cylinder contact may later make depth look free even though the semantic column
  // already holds every product the surveyed geometry admits. That must route around the full
  // lane, not drain extra tray stock into overlapping products. The twelve ledger entries name a
  // real can so the lane passes the identity pre-check and reaches the capacity bound itself.
  auto semantically_full = occupied;
  semantically_full.lanes.at(LaneId{"lane_04"}).obstructed = false;
  semantically_full.lanes.at(LaneId{"lane_01"}).contents.resize(12U, ObjectId{99});
  semantically_full.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.734;
  TrackedObject ledger_can = can(ObjectId{99}, -0.15);
  ledger_can.revision = 15U;
  semantically_full.objects.emplace(ledger_can.id, ledger_can);
  const auto around_semantic_capacity = select_task_pair(
    semantically_full, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(around_semantic_capacity) << around_semantic_capacity.error().detail;
  EXPECT_EQ(around_semantic_capacity.value().lane_id, LaneId{"lane_04"});
}

TEST_F(TaskSelectionTest, ReportsNoPairWhenCompatibleLanesReachSemanticCapacity)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  for (auto & [id, lane] : value.lanes) {
    static_cast<void>(id);
    for (std::size_t index = 0; index < 12U; ++index) {
      lane.observed_source_object_ids.push_back("sim:front_" + std::to_string(index));
    }
    // Deliberately contradictory physical evidence reproduces overlapping simulated cylinders.
    lane.available_depth_m = 0.734;
  }
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config()),
    SelectionErrorCode::NoEligiblePair);
}

TEST_F(TaskSelectionTest, ReportsNoEligiblePairWhenEveryCompatibleLaneIsObstructed)
{
  // Nothing is wrong with the world here (the lanes are all blocked), so this must read as "no
  // compatible pair", which the driver turns into a clean action outcome, not a snapshot or
  // configuration fault.
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  for (auto & [id, shelf_lane] : value.lanes) {
    static_cast<void>(id);
    shelf_lane.obstructed = true;
  }
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config()),
    SelectionErrorCode::NoEligiblePair);
}

// Card 040's SC-LIVE run ended on `premature NO_COMPATIBLE_PAIR: a compatible front deficit and
// back product remain` whose only selection detail was the bare "no deterministic object/lane
// pair satisfies the baseline predicate": six destinations refused two objects and the receipt
// could not say which predicate fired, so the terminal could not be classified offline. The
// refusal tally is what makes it classifiable — one entry per (object, code) with a count and
// the first refusal's own detail, which names a destination through `pair_context`.
TEST_F(TaskSelectionTest, NoEligiblePairNamesEveryRefusedObjectAndReason)
{
  auto value = snapshot();
  // Object 1 is refused object-side everywhere (before any lane check runs), object 2 only by
  // the obstructed destinations: two codes, both with their counts, in id order.
  value.objects.at(ObjectId{1}).orientation = ObjectOrientation::Horizontal;
  for (auto & [id, shelf_lane] : value.lanes) {
    static_cast<void>(id);
    shelf_lane.obstructed = true;
  }
  const auto result = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, SelectionErrorCode::NoEligiblePair);
  const std::string & detail = result.error().detail;
  EXPECT_NE(
    detail.find("no deterministic object/lane pair satisfies the baseline predicate"),
    std::string::npos)
    << detail;
  EXPECT_NE(detail.find("object 1 unsupported_product x2"), std::string::npos) << detail;
  EXPECT_NE(detail.find("object 2 lane_unavailable x2"), std::string::npos) << detail;
  EXPECT_NE(detail.find("and lane lane_"), std::string::npos) << detail;
}

TEST_F(TaskSelectionTest, ObjectOnlyNoEligiblePairCarriesTheSameRefusalTally)
{
  // The pinned path (Milestone 10 §1) reports the same tally, so a refusal behind a confirmed
  // identity is classifiable from the action result too.
  auto value = snapshot();
  for (auto & [id, object] : value.objects) {
    static_cast<void>(id);
    object.orientation = ObjectOrientation::Horizontal;
  }
  const auto result = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, SelectionErrorCode::NoEligiblePair);
  const std::string & detail = result.error().detail;
  EXPECT_NE(detail.find("requested object 1 has no eligible destination lane"), std::string::npos)
    << detail;
  EXPECT_NE(detail.find("object 1 unsupported_product x2"), std::string::npos) << detail;
}

TEST_F(TaskSelectionTest, CostsCarriageTravelFromWhereTheCarriageActuallyStands)
{
  // The approach leg is measured from the snapshot's rail position, not the origin, so moving the
  // carriage past a product changes which pair wins once the deficit and rail-distance keys tie.
  auto value = one_can_with_lanes(0.0, {"lane_02", "lane_05"});
  value.robot.rail_position = 1.2;
  const auto result = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_DOUBLE_EQ(result.value().score.approach_rail_travel_m, 1.2);

  auto two_cans = value;
  two_cans.objects.emplace(ObjectId{2}, can(ObjectId{2}, 0.45));
  const auto nearer = select_task_pair(
    two_cans, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(nearer) << nearer.error().detail;
  EXPECT_EQ(nearer.value().object_id, ObjectId{2});
  EXPECT_EQ(nearer.value().lane_id, LaneId{"lane_05"});
}

TEST_F(TaskSelectionTest, TravelWeightsSteerTheChoiceAndMustBeFiniteAndNonNegative)
{
  // Zeroing the delivery weight leaves only "fetch the nearest product", which differs from the
  // default: proof the weights reach the comparison. One lane at shelf x -1.0; the first can stands
  // near it and far from the carriage, the second under the carriage and far from the lane. Total
  // travel picks the first, fetch alone picks the second.
  auto value = one_can_with_lanes(-0.60, {"lane_01"});
  value.objects.emplace(ObjectId{2}, can(ObjectId{2}, 0.05));

  auto balanced = config();
  const auto by_total = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, balanced);
  ASSERT_TRUE(by_total) << by_total.error().detail;
  EXPECT_EQ(by_total.value().object_id, ObjectId{1});

  auto approach_only = config();
  approach_only.delivery_rail_travel_weight = 0.0;
  const auto by_approach = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, approach_only);
  ASSERT_TRUE(by_approach) << by_approach.error().detail;
  EXPECT_EQ(by_approach.value().object_id, ObjectId{2});
  EXPECT_DOUBLE_EQ(by_approach.value().score.approach_rail_travel_m, 0.05);
  EXPECT_DOUBLE_EQ(by_approach.value().score.total, 0.05);

  // Fail on a weight that cannot order anything, instead of scoring every pair the same or
  // comparing against NaN.
  auto negative = config();
  negative.approach_rail_travel_weight = -1.0;
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, negative),
    SelectionErrorCode::InvalidConfiguration);

  auto not_a_number = config();
  not_a_number.delivery_rail_travel_weight = std::numeric_limits<double>::quiet_NaN();
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, not_a_number),
    SelectionErrorCode::InvalidConfiguration);
}

TEST_F(TaskSelectionTest, SelectsEveryCataloguedProductClassIntoItsMatchingLane)
{
  struct Case
  {
    ProductClass product_class;
    const char * sku;
    const char * lane_id;
    double radius_m;
    double height_m;
  };
  // Radii and heights are the catalogue's, restated so an edit to the catalogue that changes what
  // the jaws are sized from fails this test. Both lanes of each class appear: the second is a
  // usable alternative only if it accepts the class on the same terms, including the narrower lane
  // still holding the widest product (the large bottle's two rows).
  const Case cases[] = {
    {ProductClass::Can, "SIM-CAN-STD", "lane_01", 0.033, 0.122},
    {ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", "lane_02", 0.034, 0.200},
    {ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", "lane_03", 0.045, 0.290},
    {ProductClass::Can, "SIM-CAN-STD", "lane_04", 0.033, 0.122},
    {ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", "lane_05", 0.034, 0.200},
    {ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", "lane_06", 0.045, 0.290},
  };
  for (const Case & item : cases) {
    WorldStateSnapshot value;
    value.revision = 20;
    value.objects.emplace(
      ObjectId{1},
      seated_product(ObjectId{1}, 0.0, item.product_class, item.sku, item.height_m));
    ShelfLane destination = lane(item.lane_id, 11);
    destination.expected_product_class = item.product_class;
    destination.expected_sku = item.sku;
    value.lanes.emplace(LaneId{item.lane_id}, destination);
    value.robot.telemetry_time = ros_time(9'950'000'000);
    value.robot.telemetry_source_id = "test/task-selection";
    value.robot.telemetry_revision = 19;
    value.robot.revision = 19;

    const auto result = select_task_pair(
      value, catalog_, geometry_, world_from_shelf_, config(),
      SelectionRequest{ObjectId{1}, LaneId{item.lane_id}});
    ASSERT_TRUE(result) << item.sku << ": " << result.error().detail;
    EXPECT_EQ(result.value().lane_id, LaneId{item.lane_id}) << item.sku;
    EXPECT_DOUBLE_EQ(result.value().product_envelope.radius_m, item.radius_m) << item.sku;
    EXPECT_DOUBLE_EQ(result.value().product_envelope.height_m, item.height_m) << item.sku;
  }
}

TEST_F(TaskSelectionTest, RejectsAProductWhoseClassAndLanePolicyDisagree)
{
  // The class gate that rejected every non-can is gone; the lane policy alone keeps a bottle out
  // of the can's lane. Without this, removing the gate would widen which destinations accept which
  // products.
  WorldStateSnapshot value;
  value.revision = 20;
  value.objects.emplace(
    ObjectId{1},
    seated_product(ObjectId{1}, 0.0, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.290));
  value.lanes.emplace(LaneId{"lane_01"}, lane("lane_01", 11));
  value.robot.telemetry_time = ros_time(9'950'000'000);
  value.robot.telemetry_source_id = "test/task-selection";
  value.robot.telemetry_revision = 19;
  value.robot.revision = 19;

  expect_error(
    select_task_pair(
      value, catalog_, geometry_, world_from_shelf_, config(),
      SelectionRequest{ObjectId{1}, LaneId{"lane_01"}}),
    SelectionErrorCode::IncompatiblePair);
}

TEST_F(TaskSelectionTest, ExplicitIdentityUsesTheSameEligibilityPredicate)
{
  const auto selected = select_task_pair(
    snapshot(), catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{2}, LaneId{"lane_02"}});
  ASSERT_TRUE(selected) << selected.error().detail;
  EXPECT_EQ(selected.value().object_id, ObjectId{2});
  EXPECT_EQ(selected.value().lane_id, LaneId{"lane_02"});

  auto invalid = snapshot();
  invalid.objects.at(ObjectId{2}).orientation = ObjectOrientation::Horizontal;
  const auto rejected = select_task_pair(
    invalid, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{2}, LaneId{"lane_02"}});
  ASSERT_FALSE(rejected);
  EXPECT_EQ(rejected.error().code, SelectionErrorCode::UnsupportedProduct);
}

TEST_F(TaskSelectionTest, RejectsUnavailableStaleUnsupportedAndOutOfStockObjects)
{
  const SelectionRequest request{ObjectId{1}, LaneId{"lane_01"}};

  auto unavailable = snapshot();
  unavailable.objects.at(ObjectId{1}).grasp_state = GraspState::Attached;
  expect_error(
    select_task_pair(unavailable, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::ObjectUnavailable);

  auto stale = snapshot();
  stale.objects.at(ObjectId{1}).observation_time = ros_time(9'000'000'000);
  expect_error(
    select_task_pair(stale, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::ObjectStale);

  // A class the world state cannot name has no catalogue entry, so no envelope to size the jaws
  // from. Every class it can name is selectable and covered separately.
  auto unsupported = snapshot();
  unsupported.objects.at(ObjectId{1}).product_class = ProductClass::Unknown;
  expect_error(
    select_task_pair(unsupported, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::UnsupportedProduct);

  auto inconsistent_axis = snapshot();
  inconsistent_axis.objects.at(ObjectId{1}).pose_in_world.linear() =
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix();
  expect_error(
    select_task_pair(
      inconsistent_axis, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::UnsupportedProduct);

  auto outside = snapshot();
  outside.objects.at(ObjectId{1}).pose_in_world.translation().x() = 2.0;
  expect_error(
    select_task_pair(outside, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::ObjectOutsideStock);

  // A product generated tangent to the rear tray wall can settle through that boundary by the
  // millimetre-scale contact tolerance used at the tray floor. It stays selectable inside the
  // explicit 2 mm bound, never outside it.
  auto seated_at_wall = snapshot();
  const double radius_m = 0.033;
  auto shelf_pose = world_from_shelf_.inverse() *
    seated_at_wall.objects.at(ObjectId{1}).pose_in_world;
  shelf_pose.translation().y() = geometry_.stock_region_in_shelf.min().y() + radius_m - 0.001;
  seated_at_wall.objects.at(ObjectId{1}).pose_in_world = world_from_shelf_ * shelf_pose;
  EXPECT_TRUE(
    select_task_pair(
      seated_at_wall, catalog_, geometry_, world_from_shelf_, config(), request));

  shelf_pose.translation().y() = geometry_.stock_region_in_shelf.min().y() + radius_m - 0.003;
  seated_at_wall.objects.at(ObjectId{1}).pose_in_world = world_from_shelf_ * shelf_pose;
  expect_error(
    select_task_pair(seated_at_wall, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::ObjectOutsideStock);
}

TEST_F(TaskSelectionTest, RequiresFreshAvailableRobotTelemetry)
{
  const SelectionRequest request{ObjectId{1}, LaneId{"lane_01"}};

  auto unobserved = snapshot();
  unobserved.robot.telemetry_revision = 0;
  unobserved.robot.telemetry_time = ros_time(0);
  expect_error(
    select_task_pair(unobserved, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::InvalidSnapshot);

  auto missing_time = snapshot();
  missing_time.robot.telemetry_time = ros_time(0);
  expect_error(
    select_task_pair(missing_time, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::InvalidSnapshot);

  auto stale = snapshot();
  stale.robot.telemetry_time = ros_time(9'499'999'999);
  expect_error(
    select_task_pair(stale, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::RobotStale);

  auto unavailable = snapshot();
  unavailable.robot.task_phase = restocker_world_state::TaskPhase::Executing;
  expect_error(
    select_task_pair(unavailable, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::RobotUnavailable);

  auto boundary = snapshot();
  boundary.robot.telemetry_time = ros_time(9'500'000'000);
  EXPECT_TRUE(
    select_task_pair(
      boundary, catalog_, geometry_, world_from_shelf_, config(),
      request));
}

TEST_F(TaskSelectionTest, RejectsUnavailableStaleIncompatibleAndShallowLanes)
{
  const SelectionRequest request{ObjectId{1}, LaneId{"lane_01"}};

  auto obstructed = snapshot();
  obstructed.lanes.at(LaneId{"lane_01"}).obstructed = true;
  expect_error(
    select_task_pair(obstructed, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::LaneUnavailable);

  auto stale = snapshot();
  // Outside the 60 s validity horizon: last verified 61 s before now.
  stale.lanes.at(LaneId{"lane_01"}).last_verified = ros_time(10'000'000'000 - 61'000'000'000);
  expect_error(
    select_task_pair(stale, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::LaneStale);

  // Inside the horizon and not invalidated: tens of seconds old is still selectable.
  auto aged = snapshot();
  aged.lanes.at(LaneId{"lane_01"}).last_verified = ros_time(10'000'000'000 - 30'000'000'000);
  EXPECT_TRUE(
    select_task_pair(aged, catalog_, geometry_, world_from_shelf_, config(), request));

  auto invalidated = snapshot();
  invalidated.lanes.at(LaneId{"lane_01"}).evidence_invalidated = true;
  invalidated.lanes.at(LaneId{"lane_01"}).evidence_invalidated_at =
    ros_time(9'900'000'000);
  expect_error(
    select_task_pair(invalidated, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::LaneEvidenceInvalidated);

  // Stream liveness is independent of lane validity: a killed camera fails selection inside the
  // 500 ms horizon even when every lane observation is still inside its 60 s validity window.
  auto unlive = config();
  unlive.last_perception_acquisition = ros_time(10'000'000'000 - 501'000'000);
  expect_error(
    select_task_pair(snapshot(), catalog_, geometry_, world_from_shelf_, unlive, request),
    SelectionErrorCode::PerceptionUnlive);
  auto live = config();
  live.last_perception_acquisition = ros_time(10'000'000'000 - 100'000'000);
  EXPECT_TRUE(
    select_task_pair(snapshot(), catalog_, geometry_, world_from_shelf_, live, request));

  auto incompatible = snapshot();
  incompatible.lanes.at(LaneId{"lane_01"}).expected_sku = "OTHER-SKU";
  expect_error(
    select_task_pair(incompatible, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::IncompatiblePair);

  // The final semantic slot may reduce the preferred 0.045 m entry clearance while keeping the
  // product and placement margins inside the measured gap.
  auto final_slot = snapshot();
  final_slot.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.103;
  EXPECT_TRUE(
    select_task_pair(
      final_slot, catalog_, geometry_, world_from_shelf_, config(),
      request));

  // Less than the complete diameter plus both containment margins remains impossible.
  auto shallow = snapshot();
  shallow.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.075;
  expect_error(
    select_task_pair(shallow, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::InsufficientLaneDepth);

  auto malformed = snapshot();
  malformed.lanes.at(LaneId{"lane_01"}).available_depth_m = 1.0;
  expect_error(
    select_task_pair(malformed, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::InvalidSnapshot);
}

TEST_F(TaskSelectionTest, RejectsIncompleteRequestsAndReportsNoEligiblePair)
{
  // A lane selector without its object names no product to put there (Card 037): the request
  // is refused rather than half-interpreted. The object-only form is valid — see the
  // confirmed-identity tests below.
  const auto incomplete = select_task_pair(
    snapshot(), catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{std::nullopt, LaneId{"lane_01"}});
  ASSERT_FALSE(incomplete);
  EXPECT_EQ(incomplete.error().code, SelectionErrorCode::RequestedIdentityIncomplete);

  auto none = snapshot();
  for (auto & [id, object] : none.objects) {
    static_cast<void>(id);
    object.orientation = ObjectOrientation::Horizontal;
  }
  const auto result = select_task_pair(
    none, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, SelectionErrorCode::NoEligiblePair);
}

// Milestone 10 §1 confirmed-identity contract (Card 037): the mismatch that was live in Card
// 010's attempts 7 and 11. With equal deficits and the carriage at the rail origin,
// destination-first picks the admitted large bottle beside lane_03 over the confirmed can
// whose lane is farther away — the transfer then ran on never-confirmed evidence and attach
// refused. An object-only request naming the can must transfer the can.
TEST_F(TaskSelectionTest, ObjectOnlyRequestTransfersTheConfirmedCandidateOverDestinationFirst)
{
  WorldStateSnapshot value;
  value.revision = 20;
  value.objects.emplace(ObjectId{1}, can(ObjectId{1}, -0.25));
  value.objects.emplace(
    ObjectId{3},
    seated_product(ObjectId{3}, 0.40, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.290));
  value.lanes.emplace(LaneId{"lane_01"}, lane("lane_01", 11));
  ShelfLane bottle_lane = lane("lane_03", 12);
  bottle_lane.expected_product_class = ProductClass::LargeBottle;
  bottle_lane.expected_sku = "SIM-BOTTLE-LARGE";
  value.lanes.emplace(LaneId{"lane_03"}, bottle_lane);
  value.robot.telemetry_time = ros_time(9'950'000'000);
  value.robot.telemetry_source_id = "test/task-selection";
  value.robot.telemetry_revision = 19;
  value.robot.revision = 19;

  const auto automatic = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(automatic) << automatic.error().detail;
  EXPECT_EQ(automatic.value().object_id, ObjectId{3});
  EXPECT_EQ(automatic.value().lane_id, LaneId{"lane_03"});

  const auto confirmed = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_TRUE(confirmed) << confirmed.error().detail;
  EXPECT_EQ(confirmed.value().object_id, ObjectId{1});
  EXPECT_EQ(confirmed.value().lane_id, LaneId{"lane_01"});
}

// The pin names the product, not the lane: §4's destination-first ordering still picks the
// best compatible lane for the named object.
TEST_F(TaskSelectionTest, ObjectOnlyRequestKeepsDestinationFirstOrderingAmongCompatibleLanes)
{
  const auto value = one_can_with_lanes(0.0, {"lane_01", "lane_04"});
  const auto confirmed = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_TRUE(confirmed) << confirmed.error().detail;
  EXPECT_EQ(confirmed.value().object_id, ObjectId{1});
  EXPECT_EQ(confirmed.value().lane_id, LaneId{"lane_04"});
  EXPECT_DOUBLE_EQ(confirmed.value().destination_rail_distance_m, 0.2);
}

// Fail closed (Card 037): a named product with no eligible lane returns a typed refusal, and
// never a different eligible product — even though the automatic sweep has one.
TEST_F(TaskSelectionTest, ObjectOnlyRequestFailsClosedInsteadOfSubstitutingAnotherProduct)
{
  WorldStateSnapshot value;
  value.revision = 20;
  value.objects.emplace(ObjectId{1}, can(ObjectId{1}, -0.25));
  value.objects.emplace(
    ObjectId{3},
    seated_product(ObjectId{3}, 0.40, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.290));
  value.lanes.emplace(LaneId{"lane_01"}, lane("lane_01", 11));
  ShelfLane bottle_lane = lane("lane_03", 12);
  bottle_lane.expected_product_class = ProductClass::LargeBottle;
  bottle_lane.expected_sku = "SIM-BOTTLE-LARGE";
  value.lanes.emplace(LaneId{"lane_03"}, bottle_lane);
  value.robot.telemetry_time = ros_time(9'950'000'000);
  value.robot.telemetry_source_id = "test/task-selection";
  value.robot.telemetry_revision = 19;
  value.robot.revision = 19;

  // The automatic sweep still has a transfer to run: the bottle into lane_03.
  const auto automatic = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(automatic) << automatic.error().detail;
  EXPECT_EQ(automatic.value().object_id, ObjectId{3});

  // The can's only compatible lane is obstructed: the pin fails typed on that refusal plus
  // lane_03's incompatibility, instead of drifting to the eligible bottle.
  value.lanes.at(LaneId{"lane_01"}).obstructed = true;
  const auto refused = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, SelectionErrorCode::NoEligiblePair);
  EXPECT_NE(
    refused.error().detail.find("no eligible destination lane"), std::string::npos)
    << refused.error().detail;

  // An aged named object reports the staleness it actually hit (stale-only barrier), again
  // never substituting the fresh bottle.
  auto aged = select_task_pair(
    snapshot(), catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_TRUE(aged) << aged.error().detail;  // fresh baseline: both cans are selectable
  auto stale_snapshot = value;
  stale_snapshot.lanes.at(LaneId{"lane_01"}).obstructed = false;
  stale_snapshot.objects.at(ObjectId{1}).observation_time = ros_time(9'000'000'000);
  const auto stale = select_task_pair(
    stale_snapshot, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_FALSE(stale);
  EXPECT_EQ(stale.error().code, SelectionErrorCode::ObjectStale);
}

// Card 010 attempt 2 (= cmbdef2 attempt 9): a placement frees the product and appends it to the
// destination lane's `contents` without refreshing its tracked pose, so until the next
// observation arrives the same object reads as free stock *and* as a lane member. `eligible_object`
// judges only the pose, so the sweep used to pair it with the lane it already belongs to and the
// reserve guards refused that pair terminally, after the arm had been dispatched. Eligibility
// refuses it instead, and the run reports no eligible pair so the loop re-surveys.
TEST_F(TaskSelectionTest, RefusesAProductThatWorldStateAlreadyCountsInALane)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  // The can's pose still seats it on the stock tray — its last observation before the grasp —
  // while world state, not the pose, is what now knows it lives in lane_01.
  value.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{1}};

  const auto explicit_pair = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, LaneId{"lane_01"}});
  ASSERT_FALSE(explicit_pair);
  EXPECT_EQ(explicit_pair.error().code, SelectionErrorCode::ObjectUnavailable);
  EXPECT_NE(explicit_pair.error().detail.find("lane_01"), std::string::npos)
    << explicit_pair.error().detail;

  const auto sweep = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_FALSE(sweep);
  EXPECT_EQ(sweep.error().code, SelectionErrorCode::NoEligiblePair);

  // Control: identical snapshot with the membership cleared selects normally, so the refusals
  // above are the membership clause and nothing else.
  value.lanes.at(LaneId{"lane_01"}).contents.clear();
  const auto control = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(control) << control.error().detail;
  EXPECT_EQ(control.value().object_id, ObjectId{1});
}

// Card 037's confirmed-identity contract applied to a placed product: an object-only request
// naming a product world state already counts in a lane fails closed with no eligible
// destination — it never substitutes the other admitted product, and the automatic sweep does
// not hand back its own lane either.
TEST_F(TaskSelectionTest, ObjectOnlyRequestOnAPlacedProductNeverSubstitutesAnotherProduct)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  value.objects.emplace(ObjectId{2}, can(ObjectId{2}, -0.35));
  value.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{1}};

  const auto pinned = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{1}, std::nullopt});
  ASSERT_FALSE(pinned);
  EXPECT_EQ(pinned.error().code, SelectionErrorCode::NoEligiblePair);
  EXPECT_NE(pinned.error().detail.find("lane_01"), std::string::npos)
    << pinned.error().detail;

  // The unplaced sibling stays selectable, on its own name only.
  const auto sibling = select_task_pair(
    value, catalog_, geometry_, world_from_shelf_, config(),
    SelectionRequest{ObjectId{2}, std::nullopt});
  ASSERT_TRUE(sibling) << sibling.error().detail;
  EXPECT_EQ(sibling.value().object_id, ObjectId{2});
}

// The placed product's stale pose must also stop steering the tray feed queue: world state
// counts it in a lane, so it has left the tray and cannot go on refusing the product that
// stands behind it. Without the membership rule the queue check kept the next product pinned
// behind a can that no longer exists on the tray.
TEST_F(TaskSelectionTest, APlacedProductNoLongerBlocksTheTrayQueueBehindIt)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  // Two cans in one feed column: id 1 five centimetres nearer the front of the tray (the larger
  // shelf y is what the queue rule calls ahead), id 7 at the fixture's own seated pose.
  value.objects.emplace(ObjectId{7}, can(ObjectId{7}, -0.60));
  TrackedObject & front = value.objects.at(ObjectId{1});
  front.pose_in_world =
    world_from_shelf_ * Eigen::Translation3d(-0.60, -1.30, -0.10);

  // Control: while the front can is still on the tray, the queue rule refuses the product
  // behind it and the front can itself is what selects.
  const auto control = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(control) << control.error().detail;
  EXPECT_EQ(control.value().object_id, ObjectId{1});

  // Placed: the front can is a lane member, so it is neither a candidate nor a blocker, and the
  // product behind it becomes selectable.
  value.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{1}};
  const auto after = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(after) << after.error().detail;
  EXPECT_EQ(after.value().object_id, ObjectId{7});
}

TEST_F(TaskSelectionTest, AutoSelectReportsStaleWhenEveryPairFailsOnlyOnAgedEvidence)
{
  auto stale = snapshot();
  for (auto & [id, object] : stale.objects) {
    static_cast<void>(id);
    object.observation_time = ros_time(9'000'000'000);
  }
  const auto result = select_task_pair(stale, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, SelectionErrorCode::ObjectStale);

  auto mixed = snapshot();
  for (auto & [id, object] : mixed.objects) {
    static_cast<void>(id);
    object.observation_time = ros_time(9'000'000'000);
  }
  // A non-stale ineligibility must still collapse to NoEligiblePair once the sweep finds nothing
  // eligible: stale is reported only when it is the sole barrier class across every rejected pair.
  mixed.objects.at(ObjectId{1}).observation_time = config().now;
  mixed.objects.at(ObjectId{1}).orientation = ObjectOrientation::Horizontal;
  const auto still_none = select_task_pair(
    mixed, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_FALSE(still_none);
  EXPECT_EQ(still_none.error().code, SelectionErrorCode::NoEligiblePair);
}

// Milestone 10 §4's ordering, isolated: rail distance decides between equal deficits even when
// carriage-relative cost says otherwise, and a larger deficit outranks both.
TEST_F(TaskSelectionTest, OrdersByLargestDeficitThenRailDistance)
{
  // Equal deficits (both lanes empty at target 6): the product sits beside lane_01, so total
  // travel prefers lane_01 — but the carriage is at the origin and lane_04's station is the
  // nearer one, so the destination is chosen before the product is.
  auto tied = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  const auto by_rail = select_task_pair(tied, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(by_rail) << by_rail.error().detail;
  EXPECT_EQ(by_rail.value().lane_id, LaneId{"lane_04"});
  EXPECT_EQ(by_rail.value().destination_deficit, 6U);
  EXPECT_DOUBLE_EQ(by_rail.value().destination_rail_distance_m, 0.2);

  // Largest deficit first, ahead of rail distance and cost: lane_04 already holds four cans
  // (a four-pitch column), leaving deficit 2 against lane_01's 6. The bigger shortfall wins even
  // though lane_04 is both the nearer station and the cheaper delivery from this product.
  const double can_pitch_m = 2.0 * 0.033 * std::cos(geometry_.lanes.at("lane_04").incline_rad);
  auto hungry = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  hungry.lanes.at(LaneId{"lane_04"}).available_depth_m = 0.85 - 4.0 * can_pitch_m;
  const auto by_deficit = select_task_pair(
    hungry, catalog_, geometry_, world_from_shelf_,
    config());
  ASSERT_TRUE(by_deficit) << by_deficit.error().detail;
  EXPECT_EQ(by_deficit.value().lane_id, LaneId{"lane_01"});
  EXPECT_EQ(by_deficit.value().destination_deficit, 6U);
}

// SC-003: selection routes against target_count, and a runtime policy change to the want flips
// the destination without any geometry moving.
TEST_F(TaskSelectionTest, RoutesAgainstTargetCountAndChangesAfterAPolicyUpdate)
{
  const double can_pitch_m = 2.0 * 0.033 * std::cos(geometry_.lanes.at("lane_01").incline_rad);
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  // Both can lanes hold two cans: identical columns, identical deficits of 6 - 2.
  value.lanes.at(LaneId{"lane_01"}).available_depth_m = 0.85 - 2.0 * can_pitch_m;
  value.lanes.at(LaneId{"lane_04"}).available_depth_m = 0.85 - 2.0 * can_pitch_m;
  const auto before = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(before) << before.error().detail;
  EXPECT_EQ(before.value().lane_id, LaneId{"lane_04"});
  EXPECT_EQ(before.value().destination_deficit, 4U);

  // The owner drops lane_04's want to what it already holds (SetLanePolicy's effect on the
  // snapshot): its deficit zeroes and the next selection routes against the new target, to the
  // sibling still four short — geometry untouched.
  value.lanes.at(LaneId{"lane_04"}).target_count = 2;
  const auto after = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(after) << after.error().detail;
  EXPECT_EQ(after.value().lane_id, LaneId{"lane_01"});
  EXPECT_EQ(after.value().destination_deficit, 4U);
}

// Task 005 SC-003 / Milestone 10 §4: a lane whose ledger names products the current policy does
// not accept is reported and skipped, never routed into; when every destination is in that state
// the typed conflict itself is the selection outcome.
TEST_F(TaskSelectionTest, ReportsAndSkipsALaneWhoseLedgerNamesTheWrongProduct)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01", "lane_04"});
  TrackedObject stale_bottle = seated_product(
    ObjectId{7}, 0.35, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.200);
  TrackedObject stale_other = seated_product(
    ObjectId{8}, 0.50, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.200);
  value.objects.emplace(stale_bottle.id, stale_bottle);
  value.objects.emplace(stale_other.id, stale_other);
  value.lanes.at(LaneId{"lane_01"}).contents = {ObjectId{7}};

  // Explicit selection into the wrong-product lane names the conflict.
  expect_error(
    select_task_pair(
      value, catalog_, geometry_, world_from_shelf_, config(),
      SelectionRequest{ObjectId{1}, LaneId{"lane_01"}}),
    SelectionErrorCode::LaneWrongProduct);

  // A sweep skips that lane and still serves the clean sibling — one wrong lane must not cost
  // the task.
  const auto around = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(around) << around.error().detail;
  EXPECT_EQ(around.value().lane_id, LaneId{"lane_04"});

  // Every destination wrong: the typed conflict is reported rather than a bare no-pair.
  value.lanes.at(LaneId{"lane_04"}).contents = {ObjectId{8}};
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config()),
    SelectionErrorCode::LaneWrongProduct);
  EXPECT_EQ(
    to_string(SelectionErrorCode::LaneWrongProduct), "lane_wrong_product");
}

// SC-002's geometry half, on the selection side: an unreliable ledger surfaces on the pair but
// never bars it — the lane still routes on measured geometry alone.
TEST_F(TaskSelectionTest, KeepsSelectingAnUnreliableLedgerLaneOnGeometryAlone)
{
  auto value = one_can_with_lanes(-0.60, {"lane_04"});
  value.lanes.at(LaneId{"lane_04"}).ledger_unreliable = true;
  const auto result = select_task_pair(value, catalog_, geometry_, world_from_shelf_, config());
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().lane_id, LaneId{"lane_04"});
  EXPECT_TRUE(result.value().destination_ledger_unreliable);
  EXPECT_EQ(result.value().destination_deficit, 6U);
}

// A lane whose expected product has no catalogued geometry has no deficit to compute: the lane
// fails closed instead of inheriting a sibling product's pitch.
TEST_F(TaskSelectionTest, FailsClosedWhenTheExpectedProductHasNoCataloguedDeficitPitch)
{
  auto value = one_can_with_lanes(-0.60, {"lane_01"});
  value.lanes.at(LaneId{"lane_01"}).expected_product_class = ProductClass::Unknown;
  const SelectionRequest request{ObjectId{1}, LaneId{"lane_01"}};
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config(), request),
    SelectionErrorCode::MissingGeometry);
  expect_error(
    select_task_pair(value, catalog_, geometry_, world_from_shelf_, config()),
    SelectionErrorCode::NoEligiblePair);
}

// Property: held = round(column_length / pitch) and deficit = max(0, target - held) across every
// representable fill at every target, with the pitch the catalog and survey derive — never a
// tuned constant — and fail-closed when the pitch is not computable.
TEST_F(TaskSelectionTest, ComputesHeldCountAndDeficitFromTheCataloguedPitch)
{
  const auto & lane_geometry = geometry_.lanes.at("lane_01");
  const double pitch_m = 2.0 * 0.033 * std::cos(lane_geometry.incline_rad);
  ASSERT_GT(pitch_m, 0.0);
  const std::uint32_t maximum_held = static_cast<std::uint32_t>(std::floor(0.85 / pitch_m));

  std::uint32_t previous_deficit = 0U;
  for (std::uint32_t held = 0U; held <= maximum_held; ++held) {
    ShelfLane lane = this->lane("lane_01", 11U);
    lane.available_depth_m = 0.85 - static_cast<double>(held) * pitch_m;
    for (const std::uint32_t target : {0U, 1U, 6U, 12U, 40U}) {
      lane.target_count = target;
      const auto deficit = lane_deficit(lane, lane_geometry, catalog_);
      ASSERT_TRUE(deficit) << deficit.error().detail << " held=" << held;
      EXPECT_EQ(deficit.value().held_count, held) << "held=" << held;
      EXPECT_EQ(deficit.value().deficit, target > held ? target - held : 0U)
        << "held=" << held << " target=" << target;
      EXPECT_DOUBLE_EQ(deficit.value().pitch_m, pitch_m);
      // For a fixed want, more stock never increases the shortfall.
      if (target == 6U) {
        if (held > 0U) {
          EXPECT_LE(deficit.value().deficit, previous_deficit) << "held=" << held;
        }
        previous_deficit = deficit.value().deficit;
      }
    }
  }

  ShelfLane unknown = this->lane("lane_01", 11U);
  unknown.expected_product_class = ProductClass::Unknown;
  const auto refused = lane_deficit(unknown, lane_geometry, catalog_);
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, SelectionErrorCode::MissingGeometry);
}

TEST_F(TaskSelectionTest, LiveStreamDoesNotSaveARetainedSnapshotThatAgedOut)
{
  // The retained-snapshot wall the selection reacquire must contend with: a snapshot whose
  // robot and object stamps were captured at 9.95/9.80 s is selected against at 10.6 s with
  // a perception acquisition fresh 100 ms ago. The live predicate passes, but the snapshot
  // cannot have gained younger stamps while the goal held kSelectPair — selection must
  // refuse deterministically on the retained evidence for as long as the same copy is offered.
  const SelectionRequest request{ObjectId{1}, LaneId{"lane_01"}};
  auto aged = config();
  aged.now = ros_time(10'000'000'000 + 600'000'000);
  aged.last_perception_acquisition = ros_time(10'000'000'000 + 500'000'000);
  // Fresh stream (100 ms), aged robot telemetry (650 ms): refused before objects are read.
  expect_error(
    select_task_pair(snapshot(), catalog_, geometry_, world_from_shelf_, aged, request),
    SelectionErrorCode::RobotStale);
  // Even with telemetry refreshed inside the horizon, the wrist-derived object observation
  // stamps frozen in the same retained copy still fail on their own.
  auto aged_objects = aged;
  auto with_fresh_telemetry = snapshot();
  with_fresh_telemetry.robot.telemetry_time = ros_time(10'000'000'000 + 590'000'000);
  with_fresh_telemetry.robot.telemetry_revision = with_fresh_telemetry.revision;
  with_fresh_telemetry.robot.revision = with_fresh_telemetry.revision;
  expect_error(
    select_task_pair(
      with_fresh_telemetry, catalog_, geometry_, world_from_shelf_, aged_objects, request),
    SelectionErrorCode::ObjectStale);
}

// Contact penetration measured on the baseline: Gazebo settles every spawned product this far
// into its tray, so the seated envelope reaches below the surveyed tray surface.
constexpr double kObservedTraySettleM = 5.7e-7;

[[nodiscard]] ProductClass product_class_from_text(const std::string & value)
{
  if (value == "can") {
    return ProductClass::Can;
  }
  if (value == "small_bottle") {
    return ProductClass::SmallBottle;
  }
  if (value == "large_bottle") {
    return ProductClass::LargeBottle;
  }
  return ProductClass::Unknown;
}

// Guards the shipped baseline: surveyed workcell geometry, catalog envelopes and scenario spawn
// poses must together yield a selectable pair.
TEST(BaselineScenarioSelection, AcceptsTheShippedScenarioWithProductsSeatedOnTheTray)
{
  const auto catalog = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
  ASSERT_TRUE(catalog) << catalog.error().detail;
  const auto geometry = load_manipulation_geometry(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  ASSERT_TRUE(geometry) << geometry.error().detail;

  const YAML::Node scenario = YAML::LoadFile(RESTOCKER_TEST_BASELINE_SCENARIO);
  const YAML::Node workcell_pose = scenario["workcell_pose"];
  ASSERT_EQ(workcell_pose.size(), 6U);
  Eigen::Isometry3d world_from_shelf = Eigen::Isometry3d::Identity();
  world_from_shelf.translation() = Eigen::Vector3d(
    workcell_pose[0].as<double>(), workcell_pose[1].as<double>(), workcell_pose[2].as<double>());

  // Every lane the shipped geometry defines goes into the snapshot with the identity the scenario's
  // variant bounds declare for it, so this tests the real two-lane choice and fails if the two
  // files disagree on which lanes exist.
  std::map<std::string, std::pair<ProductClass, std::string>> lane_policy;
  for (const YAML::Node & variant : scenario["randomization"]["variants"]) {
    const ProductClass variant_class =
      product_class_from_text(variant["product_class"].as<std::string>());
    const auto variant_sku = variant["sku"].as<std::string>();
    for (const YAML::Node & lane_name : variant["destination_lanes"]) {
      lane_policy.emplace(lane_name.as<std::string>(), std::pair{variant_class, variant_sku});
    }
  }
  ASSERT_EQ(lane_policy.size(), geometry.value().lanes.size());

  WorldStateSnapshot snapshot;
  snapshot.revision = 20;
  std::uint64_t lane_revision = 10;
  for (const auto & [lane_name, policy] : lane_policy) {
    const auto lane_geometry = geometry.value().lanes.find(lane_name);
    ASSERT_NE(lane_geometry, geometry.value().lanes.end()) << lane_name;
    const double depth = lane_geometry->second.usable_bounds_in_lane.sizes().y();
    ++lane_revision;
    const LaneId lane_id{lane_name};
    snapshot.lanes.emplace(
      lane_id,
      ShelfLane{
          lane_id, policy.first, policy.second, 6, {}, {}, depth, depth, false,
          ros_time(9'900'000'000), false, ros_time(0), lane_revision, false, lane_revision});
  }

  std::uint64_t index = 0;
  for (const YAML::Node & product : scenario["products"]) {
    ++index;
    const YAML::Node spawn = product["spawn_pose"];
    ASSERT_EQ(spawn.size(), 6U);
    TrackedObject object;
    object.id = ObjectId{index};
    object.source_object_id = product["source_object_id"].as<std::string>();
    object.product_class = product_class_from_text(product["product_class"].as<std::string>());
    object.sku = product["sku"].as<std::string>();
    object.pose_in_world = Eigen::Isometry3d(
      Eigen::Translation3d(
        spawn[0].as<double>(), spawn[1].as<double>(),
        spawn[2].as<double>() - kObservedTraySettleM));
    object.pose_covariance.setIdentity();
    object.orientation = ObjectOrientation::Upright;
    object.tracking_state = TrackingState::Tracked;
    object.grasp_state = GraspState::Free;
    object.observation_time = ros_time(9'800'000'000);
    object.transition_time = object.observation_time;
    object.revision = index;
    snapshot.objects.emplace(object.id, object);
  }
  ASSERT_EQ(snapshot.objects.size(), 3U);
  snapshot.robot.telemetry_time = ros_time(9'950'000'000);
  snapshot.robot.telemetry_source_id = "test/baseline-scenario";
  snapshot.robot.telemetry_revision = 19;
  snapshot.robot.revision = 19;

  SelectionConfig selection;
  selection.now = ros_time(10'000'000'000);
  selection.maximum_object_age = 500ms;
  selection.lane_evidence_validity = 60s;
  selection.maximum_robot_age = 500ms;
  selection.maximum_future_skew = 50ms;
  selection.stock_containment_margin_m = 0.0;
  selection.maximum_upright_tilt_rad = 0.05;
  const auto result = select_task_pair(
    snapshot, catalog.value(), geometry.value(), world_from_shelf, selection);
  ASSERT_TRUE(result) << result.error().detail;
  // Deficit ordering decides the destination before cost does. Every lane starts empty at
  // target 6, so the deficits tie and rail distance from the carriage (at the origin) ranks the
  // can lane lane_04 (0.2 m) and the large-bottle lane lane_03 (0.2 m) ahead of the small-bottle
  // lanes (0.6 m) — the small bottle used to win on total travel (0.60 against 1.04) and no
  // longer does. Only lane_04 takes a can; against lane_03 the tray depth and total cost tie
  // (all three products share y = -0.80 and both pairs cost 1.04), so object identity breaks the
  // tie and the can (id 1) ships ahead of the large bottle (id 3).
  EXPECT_EQ(result.value().object_id, ObjectId{1});
  EXPECT_EQ(result.value().lane_id, LaneId{"lane_04"});
  EXPECT_DOUBLE_EQ(result.value().product_envelope.radius_m, 0.033);
  EXPECT_DOUBLE_EQ(result.value().product_envelope.height_m, 0.122);
  EXPECT_EQ(result.value().destination_deficit, 6U);
  EXPECT_DOUBLE_EQ(result.value().destination_rail_distance_m, 0.2);
  EXPECT_DOUBLE_EQ(result.value().score.approach_rail_travel_m, 0.42);
  EXPECT_DOUBLE_EQ(result.value().score.delivery_rail_travel_m, 0.62);

  const YAML::Node stock =
    YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY)["stock_tray"]["usable_volume"];
  const double surveyed_floor = stock["center_xyz_m"][2].as<double>() -
    0.5 * stock["size_xyz_m"][2].as<double>();
  const double selected_floor = (world_from_shelf.inverse() *
    snapshot.objects.at(result.value().object_id).pose_in_world).translation().z() -
    0.5 * result.value().product_envelope.height_m;
  EXPECT_LT(selected_floor, surveyed_floor);
  EXPECT_GE(selected_floor, geometry.value().stock_region_in_shelf.min().z());
}

TEST(DescribePerceptionLiveness, NeverObservedNamesBothCounters)
{
  PerceptionLivenessObservation observation;
  observation.arrivals = 0U;
  observation.stamp_advances = 0U;
  const auto detail =
    describe_perception_liveness(observation, ros_time(10'000'000'000), 500ms);
  EXPECT_NE(
    detail.find("no wrist camera acquisition has been observed yet"), std::string::npos)
    << detail;
  EXPECT_NE(detail.find("arrivals=0"), std::string::npos) << detail;
  EXPECT_NE(detail.find("stamp_advances=0"), std::string::npos) << detail;
}

TEST(DescribePerceptionLiveness, StaleDetailCarriesAgeHorizonAndBothCounters)
{
  // The separating numbers for the next live probe: how old the retained stamp is, what the
  // horizon said, how many callbacks ran, how many of them advanced the stamp, and how long
  // the advancing arrival itself took to reach the consumer.
  PerceptionLivenessObservation observation;
  observation.last_acquisition = ros_time(9'000'000'000);
  observation.last_received_at = ros_time(9'100'000'000);
  observation.arrivals = 1234U;
  observation.stamp_advances = 1200U;
  const auto detail =
    describe_perception_liveness(observation, ros_time(10'000'000'000), 500ms);
  EXPECT_NE(detail.find("wrist camera acquisition is 1000 ms old"), std::string::npos)
    << detail;
  EXPECT_NE(detail.find("horizon 500 ms"), std::string::npos) << detail;
  EXPECT_NE(detail.find("receipt lag 100 ms"), std::string::npos) << detail;
  EXPECT_NE(detail.find("arrivals=1234"), std::string::npos) << detail;
  EXPECT_NE(detail.find("stamp_advances=1200"), std::string::npos) << detail;
  EXPECT_NE(
    detail.find("older than the configured liveness horizon"), std::string::npos) << detail;
}

TEST(DescribePerceptionLiveness, FreshDetailOmitsTheHistoricUnlivePhrase)
{
  PerceptionLivenessObservation observation;
  observation.last_acquisition = ros_time(9'900'000'000);
  observation.last_received_at = ros_time(9'905'000'000);
  observation.arrivals = 7U;
  observation.stamp_advances = 7U;
  const auto detail =
    describe_perception_liveness(observation, ros_time(10'000'000'000), 500ms);
  EXPECT_NE(detail.find("wrist camera acquisition is 100 ms old"), std::string::npos)
    << detail;
  EXPECT_EQ(
    detail.find("older than the configured liveness horizon"), std::string::npos) << detail;
}

TEST(DescribePerceptionLiveness, ArrivalsWithoutStampAdvancesStayVisible)
{
  // Delivery ran but the stamp never moved: the age grows while arrivals climb. Both
  // counters in one line is what distinguishes that from a silent consumer.
  PerceptionLivenessObservation observation;
  observation.last_acquisition = ros_time(9'000'000'000);
  observation.last_received_at = ros_time(9'050'000'000);
  observation.arrivals = 50U;
  observation.stamp_advances = 10U;
  const auto detail =
    describe_perception_liveness(observation, ros_time(10'000'000'000), 500ms);
  EXPECT_NE(detail.find("arrivals=50"), std::string::npos) << detail;
  EXPECT_NE(detail.find("stamp_advances=10"), std::string::npos) << detail;
  EXPECT_NE(
    detail.find("older than the configured liveness horizon"), std::string::npos) << detail;
}

TEST(DescribePerceptionLiveness, MissingReceiptReportsNegativeLagRatherThanGuessing)
{
  PerceptionLivenessObservation observation;
  observation.last_acquisition = ros_time(9'000'000'000);
  observation.arrivals = 1U;
  observation.stamp_advances = 1U;
  const auto detail =
    describe_perception_liveness(observation, ros_time(10'000'000'000), 500ms);
  EXPECT_NE(detail.find("receipt lag -1 ms"), std::string::npos) << detail;
}

}  // namespace
}  // namespace restocker_task_executor
