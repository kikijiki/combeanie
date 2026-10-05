// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/placement_candidates.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{
namespace
{

using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::ShelfLane;
using restocker_world_state::TrackedObject;

[[nodiscard]] TrackedObject can()
{
  TrackedObject object;
  object.id = ObjectId{4};
  object.source_object_id = "sim:can_04";
  object.product_class = ProductClass::Can;
  object.sku = "SIM-CAN-STD";
  object.orientation = ObjectOrientation::Upright;
  object.revision = 12;
  return object;
}

[[nodiscard]] ShelfLane lane()
{
  ShelfLane value;
  value.id = LaneId{"lane_01"};
  value.expected_product_class = ProductClass::Can;
  value.expected_sku = "SIM-CAN-STD";
  value.depth_m = 0.85;
  value.available_depth_m = 0.85;
  value.evidence_revision = 13;
  value.revision = 13;
  return value;
}

[[nodiscard]] SelectedTaskPair selection()
{
  return SelectedTaskPair{ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {0.033, 0.122}};
}

// A reduced lane: its floor sits 0.005 above the lane frame's own zero so that "seated on the
// lane floor" cannot be confused with "seated at z = 0", and its roller bed is pitched at
// atan(1/14) about a datum 0.84 deep, which makes the bed's rear lip a round 0.06 above that
// floor. Both numbers are deliberately not the shipped ones; the shipped lane is loaded by name
// in SeatsEveryCataloguedProductClassOnItsLaneFloor, which is where the survey itself is judged.
[[nodiscard]] LaneManipulationGeometry geometry()
{
  return LaneManipulationGeometry{
    "lane_01", "lane_01", -0.8,
    Eigen::AlignedBox3d(
      Eigen::Vector3d(-0.3825, 0.01, 0.005),
      Eigen::Vector3d(0.3825, 0.86, 0.405)),
    0.002, 0.057, std::atan(1.0 / 14.0), 0.84, Eigen::Vector3d::UnitY()};
}

[[nodiscard]] Eigen::Isometry3d world_from_lane()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(-0.8, 0.55, 0.75);
  return transform;
}

// A verified grasp of the upright can, taken with the gripper pointing along the product's +y and
// its jaws opening along the product's own upright axis. The grasp-centre frame is tool0's frame:
// +z is where the gripper points, +y is the jaw closing axis, +x is the vertical jaw-width axis.
[[nodiscard]] Eigen::Isometry3d product_from_grasp_center()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear().col(0) = Eigen::Vector3d::UnitZ();
  transform.linear().col(1) = Eigen::Vector3d::UnitX();
  transform.linear().col(2) = Eigen::Vector3d::UnitY();
  transform.translation().z() = 0.01;
  return transform;
}

[[nodiscard]] Eigen::Isometry3d tool0_from_grasp_center()
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation().z() = 0.14;
  return transform;
}

[[nodiscard]] PlacementGenerationConfig config()
{
  return PlacementGenerationConfig{0.15, 0.12, 0.0, 0.002, 0.05};
}

TEST(PlacementCandidates, ProducesLaneCenteredUprightInsertionAndToolPoses)
{
  const auto result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 1U);
  const auto & candidate = result.value().front();
  EXPECT_TRUE(
    candidate.lane_from_product.translation().isApprox(
      Eigen::Vector3d(0.0, 0.10, 0.128), 1.0e-12));
  EXPECT_TRUE(
    candidate.world_from_product.translation().isApprox(
      Eigen::Vector3d(-0.8, 0.65, 0.878), 1.0e-12));
  EXPECT_TRUE(
    candidate.world_from_product.linear().isApprox(Eigen::Matrix3d::Identity(), 1.0e-12));
  // The gripper must point along the lane insertion axis and keep its jaw-width axis vertical, so
  // that the product it carries arrives standing up and the wrist trails it out of the lane.
  EXPECT_TRUE(
    grasp_approach_axis(candidate.poses.world_from_final_center).isApprox(
      Eigen::Vector3d::UnitY(), 1.0e-12));
  EXPECT_TRUE(
    grasp_jaw_width_axis(candidate.poses.world_from_final_center).isApprox(
      Eigen::Vector3d::UnitZ(), 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_final_center.translation().isApprox(
      Eigen::Vector3d(-0.8, 0.65, 0.888), 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_preinsertion_center.translation().isApprox(
      Eigen::Vector3d(-0.8, 0.50, 0.888), 1.0e-12));
  EXPECT_TRUE(
    candidate.poses.world_from_retreat_center.translation().isApprox(
      Eigen::Vector3d(-0.8, 0.53, 0.888), 1.0e-12));
  // tool0 stands back along the insertion axis, not below the roller bed.
  EXPECT_TRUE(
    candidate.poses.world_from_final_tool0.translation().isApprox(
      Eigen::Vector3d(-0.8, 0.51, 0.888), 1.0e-12));
  EXPECT_TRUE(
    (candidate.poses.world_from_final_tool0 * tool0_from_grasp_center()).matrix().isApprox(
      candidate.poses.world_from_final_center.matrix(), 1.0e-12));
  EXPECT_EQ(candidate.source_lane_revision, 13U);
  EXPECT_NEAR(candidate.required_lane_depth_m, 0.123, 1.0e-12);
}

TEST(PlacementCandidates, PreservesLaneRelativeResultWhenWorkcellRotatesAboutWorldZ)
{
  auto rotated_lane = world_from_lane();
  rotated_lane.linear() = Eigen::AngleAxisd(
    0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const auto result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), rotated_lane,
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  const auto & candidate = result.value().front();
  EXPECT_TRUE(
    candidate.lane_from_product.translation().isApprox(
      Eigen::Vector3d(0.0, 0.10, 0.128), 1.0e-12));
  EXPECT_TRUE(
    grasp_approach_axis(candidate.poses.world_from_final_center).isApprox(
      -Eigen::Vector3d::UnitX(), 1.0e-12));
  EXPECT_TRUE(
    candidate.world_from_product.linear().isApprox(rotated_lane.linear(), 1.0e-12));
}

TEST(PlacementCandidates, SeatsEveryCataloguedProductClassOnItsLaneFloor)
{
  struct Case
  {
    ProductClass product_class;
    const char * sku;
    double radius_m;
    double height_m;
  };
  const Case cases[] = {
    {ProductClass::Can, "SIM-CAN-STD", 0.033, 0.122},
    {ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.034, 0.200},
    {ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.045, 0.290},
  };
  // The shipped lane, not the reduced fixture the other cases use: whether a product fits depends
  // on the surveyed insert depth and on how high the roller bed's rear lip stands, and that is
  // what this must not fake. The tall products are what bind here: the release is
  // referenced to the bed's rear lip, so a 0.290 m bottle starts 0.059 m up the lane's own floor.
  const auto loaded = load_manipulation_geometry(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  ASSERT_TRUE(loaded) << loaded.error().detail;
  const auto lane_entry = loaded.value().lanes.find("lane_01");
  ASSERT_NE(lane_entry, loaded.value().lanes.end());
  const LaneManipulationGeometry shipped = lane_entry->second;
  for (const Case & item : cases) {
    auto held = can();
    held.product_class = item.product_class;
    held.sku = item.sku;
    auto destination = lane();
    destination.expected_product_class = item.product_class;
    destination.expected_sku = item.sku;
    const SelectedTaskPair pair{
      ObjectId{4}, LaneId{"lane_01"}, 20, 12, 13, {item.radius_m, item.height_m}};
    const auto result = generate_upright_cylinder_placements(
      held, destination, pair, shipped, world_from_lane(),
      product_from_grasp_center(), tool0_from_grasp_center(), config(),
      item.product_class, std::string{item.sku});
    ASSERT_TRUE(result) << item.sku << ": " << result.error().detail;
    ASSERT_EQ(result.value().size(), 1U) << item.sku;
    const auto & candidate = result.value().front();
    // Whatever the product's height, it is released clear of the bed's rear lip and its centre
    // rises by exactly half of itself: a taller product stands taller, it does not hang lower.
    const double base_z = shipped.usable_bounds_in_lane.min().z() +
      lane_insertion_floor_height_m(shipped) + config().insertion_floor_clearance_m;
    EXPECT_NEAR(
      candidate.lane_from_product.translation().z(),
      base_z + 0.5 * item.height_m, 1.0e-12) << item.sku;
    // And it is released clear of the bed under it along its whole footprint, not only under its
    // centre. The bed is highest at the product's uphill rim, and that rim is what a release
    // referenced to the bed under the centre would bury.
    EXPECT_GE(
      base_z,
      shipped.usable_bounds_in_lane.min().z() +
      lane_floor_height_m(
        shipped,
        lane_release_product_center_depth_m(shipped, item.radius_m) - item.radius_m) +
      config().insertion_floor_clearance_m) << item.sku;
    // A taller product must still fit under the lane's ceiling from that raised start.
    EXPECT_LT(
      base_z + item.height_m, shipped.usable_bounds_in_lane.max().z()) << item.sku;
    // The lane must still be deep enough for the wider product's own footprint, and only for
    // that. Because the release depth is derived from this product's radius, what the placement
    // costs the lane is its own diameter plus the entry clearance and nothing else, so a can
    // reserves less of the lane than a large bottle does.
    EXPECT_NEAR(
      candidate.required_lane_depth_m,
      2.0 * item.radius_m + shipped.insert_entry_clearance_m, 1.0e-12) << item.sku;
  }
  // Checked as a monotone property rather than three constants: the wider the product, the more of
  // the lane's rear it takes.
  double previous = 0.0;
  for (const Case & item : cases) {
    const double required = 2.0 * item.radius_m + shipped.insert_entry_clearance_m;
    EXPECT_GT(required, previous) << item.sku;
    previous = required;
  }
}

TEST(PlacementCandidates, RejectsAHeldProductTheWorldStateCannotName)
{
  // Placement reads the held product's class as evidence that the envelope it is about to seat is
  // still the one the lane was measured against, so an unnamed class is refused rather than
  // placed on the strength of the retained selection alone.
  auto unnamed = can();
  unnamed.product_class = ProductClass::Unknown;
  auto destination = lane();
  destination.expected_product_class = ProductClass::Unknown;
  destination.expected_sku = std::nullopt;
  const auto result = generate_upright_cylinder_placements(
    unnamed, destination, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Unknown, std::nullopt);
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidObject);
}

TEST(PlacementCandidates, RejectsDepthContainmentObstructionAndCompatibility)
{
  // The last semantic slot adapts the preferred entry clearance to measured depth while keeping
  // the complete can inside the gap (this reduced unit fixture configures a zero margin).
  auto final_slot = lane();
  final_slot.available_depth_m = 0.10;
  auto result = generate_upright_cylinder_placements(
    can(), final_slot, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_NEAR(result.value().front().required_lane_depth_m, 0.10, 1.0e-12);

  auto shallow = lane();
  shallow.available_depth_m = 0.065;
  result = generate_upright_cylinder_placements(
    can(), shallow, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::PlacementOutsideLane);

  // A lane that already holds a product and still has room takes another. This is the capability
  // the column is for, and it is decided by depth alone: the evidence naming an occupant is not a
  // refusal, and the free depth left behind that occupant is.
  auto occupied = lane();
  occupied.observed_source_object_ids = {"sim:existing"};
  occupied.available_depth_m = 0.734;
  result = generate_upright_cylinder_placements(
    can(), occupied, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  // A near-full observed column uses the same bounded adaptive release.
  auto packed = occupied;
  packed.available_depth_m = 0.12;
  result = generate_upright_cylinder_placements(
    can(), packed, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_NEAR(result.value().front().required_lane_depth_m, 0.12, 1.0e-12);

  // Obstruction still refuses, and means what it always meant: something overlaps the lane volume
  // without being inside it, so the free depth is not a column length at all.
  auto obstructed = lane();
  obstructed.obstructed = true;
  result = generate_upright_cylinder_placements(
    can(), obstructed, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidLane);

  // The policy captured at grant is what refuses a mismatched SKU; the live lane's own intent
  // is deliberately left compatible here so only the capture can be what failed.
  auto captured_mismatch = lane();
  result = generate_upright_cylinder_placements(
    can(), captured_mismatch, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"OTHER-SKU"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidLane);

  // A bed steep enough that the release no longer fits under the lane's ceiling. The product is
  // carried in at the height of the bed's rear lip, so the lane has to clear that lip plus the
  // whole product, and this is the axis the incline can push a placement out along.
  auto steep = geometry();
  steep.incline_rad = std::atan(0.5);
  result = generate_upright_cylinder_placements(
    can(), lane(), selection(), steep, world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::PlacementOutsideLane);
}

// Regression: the product used to be seated a containment margin above the lane floor, so
// letting go of it dropped it that far. It came to rest below the volume the lane is judged by,
// the lane read as obstructed rather than filled, and the world state refused to record the
// placement. The seat height is now its own small insertion clearance and the containment margin
// may not move it, whatever the margin is set to.
TEST(PlacementCandidates, SeatsTheProductOnTheLaneFloorWhateverTheMargin)
{
  auto margined = config();
  margined.containment_margin_m = 0.005;
  const auto result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), margined,
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;
  ASSERT_EQ(result.value().size(), 1U);
  const auto & candidate = result.value().front();
  const double floor =
    geometry().usable_bounds_in_lane.min().z() + lane_insertion_floor_height_m(geometry());
  EXPECT_NEAR(
    candidate.lane_from_product.translation().z(),
    floor + config().insertion_floor_clearance_m +
    0.5 * selection().product_envelope.height_m, 1.0e-12);
  // The clearance is the planner's, not the margin's: a five-times-larger margin moves nothing.
  EXPECT_GT(margined.containment_margin_m, config().insertion_floor_clearance_m);
  // The other five faces keep the margin: a lane too shallow for the product plus its margin is
  // still refused.
  auto narrow = geometry();
  narrow.usable_bounds_in_lane = Eigen::AlignedBox3d(
    Eigen::Vector3d(-0.034, 0.01, 0.005), Eigen::Vector3d(0.034, 0.86, 0.405));
  const auto refused = generate_upright_cylinder_placements(
    can(), lane(), selection(), narrow, world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), margined,
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(refused);
  EXPECT_EQ(refused.error().code, PlacementCandidateErrorCode::PlacementOutsideLane);
}

TEST(PlacementCandidates, RejectsRevisionGeometryAndTransformDrift)
{
  // Placement is generated long after selection, so the lane revision is expected to have moved
  // on; only a revision that has gone backwards contradicts the evidence the task was built on.
  auto advanced_lane = lane();
  advanced_lane.revision = 14;
  auto result = generate_upright_cylinder_placements(
    can(), advanced_lane, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;

  auto regressed_lane = lane();
  regressed_lane.revision = 12;
  result = generate_upright_cylinder_placements(
    can(), regressed_lane, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidLane);

  auto regressed_object = can();
  regressed_object.revision = 11;
  result = generate_upright_cylinder_placements(
    regressed_object, lane(), selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidObject);

  auto mismatched_geometry = geometry();
  mismatched_geometry.id = "lane_02";
  result = generate_upright_cylinder_placements(
    can(), lane(), selection(), mismatched_geometry, world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidGeometry);

  auto tilted_attachment = product_from_grasp_center();
  tilted_attachment.linear() =
    Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitX()).toRotationMatrix() *
    tilted_attachment.linear();
  result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), world_from_lane(),
    tilted_attachment, tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidAttachmentTransform);

  auto invalid_tool = tool0_from_grasp_center();
  invalid_tool.translation().x() = std::numeric_limits<double>::infinity();
  result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), invalid_tool, config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidToolTransform);
}

// SC-003: SetLanePolicy may repark the destination while the product is in the jaws. Placement
// generation runs mid-transfer (after attach, before insert), so admission must be judged against
// the policy captured at grant — the same capture the store's reservation predicate and the
// port's execution proof enforce — or a granted transfer is stranded here instead of completing.
TEST(PlacementCandidates, JudgesTheCapturedPolicyNotTheLiveLaneIntent)
{
  auto reparked = lane();
  reparked.expected_product_class = ProductClass::SmallBottle;
  reparked.expected_sku = "SIM-BOTTLE-SMALL";
  auto result = generate_upright_cylinder_placements(
    can(), reparked, selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::Can, std::string{"SIM-CAN-STD"});
  ASSERT_TRUE(result) << result.error().detail;

  // And the capture still refuses what it refused before, even though the live lane now accepts
  // the held product: the two must not be able to contradict each other in either direction.
  result = generate_upright_cylinder_placements(
    can(), lane(), selection(), geometry(), world_from_lane(),
    product_from_grasp_center(), tool0_from_grasp_center(), config(),
    ProductClass::SmallBottle, std::string{"SIM-BOTTLE-SMALL"});
  ASSERT_FALSE(result);
  EXPECT_EQ(result.error().code, PlacementCandidateErrorCode::InvalidLane);
}

}  // namespace
}  // namespace restocker_task_executor
