// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/placement_candidates.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>

namespace restocker_task_executor
{
namespace
{

constexpr double kDepthRoundingSlackM = 1.0e-9;

[[nodiscard]] std::string format_metres(double value)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4) << value << "m";
  return stream.str();
}

[[nodiscard]] std::string describe_box(const Eigen::AlignedBox3d & box)
{
  return "x[" + format_metres(box.min().x()) + "," + format_metres(box.max().x()) + "] y[" +
         format_metres(box.min().y()) + "," + format_metres(box.max().y()) + "] z[" +
         format_metres(box.min().z()) + "," + format_metres(box.max().z()) + "]";
}

template<typename T>
[[nodiscard]] PlacementCandidateResult<T> failure(
  PlacementCandidateErrorCode code, std::string detail)
{
  return PlacementCandidateResult<T>::failure(
    PlacementCandidateError{code, std::move(detail)});
}

[[nodiscard]] bool finite_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

// Placement describes an upright cylinder standing on the lane floor, not a beverage, so every
// product class the collision catalog can describe as one is placeable. Unknown is not: a product
// the world state can no longer name is not evidence that the held envelope is still the one the
// lane was measured against.
[[nodiscard]] bool placeable_upright_cylinder_class(
  restocker_world_state::ProductClass product_class) noexcept
{
  using restocker_world_state::ProductClass;
  switch (product_class) {
    case ProductClass::Can:
    case ProductClass::SmallBottle:
    case ProductClass::LargeBottle:
      return true;
    case ProductClass::Unknown:
      return false;
  }
  return false;
}

[[nodiscard]] Eigen::Isometry3d offset_against_insertion(
  const Eigen::Isometry3d & pose, const Eigen::Vector3d & insertion_axis, double distance)
{
  Eigen::Isometry3d offset = pose;
  offset.translation() -= distance * insertion_axis;
  return offset;
}

}  // namespace

PlacementCandidateResult<std::vector<PlacementCandidate>>
generate_upright_cylinder_placements(
  const restocker_world_state::TrackedObject & object,
  const restocker_world_state::ShelfLane & lane, const SelectedTaskPair & selection,
  const LaneManipulationGeometry & lane_geometry,
  const Eigen::Isometry3d & world_from_lane,
  const Eigen::Isometry3d & product_from_grasp_center,
  const Eigen::Isometry3d & tool0_from_grasp_center,
  const PlacementGenerationConfig & config,
  restocker_world_state::ProductClass destination_expected_product_class,
  const std::optional<std::string> & destination_expected_sku)
{
  using restocker_world_state::ObjectOrientation;

  const double half_pi = 0.5 * std::acos(-1.0);
  if (!finite_positive(config.preinsertion_distance_m) ||
    !finite_positive(config.retreat_distance_m) ||
    !finite_nonnegative(config.containment_margin_m) ||
    !finite_nonnegative(config.insertion_floor_clearance_m) ||
    !finite_nonnegative(config.maximum_axis_alignment_error_rad) ||
    config.maximum_axis_alignment_error_rad >= half_pi)
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidConfiguration,
      "placement distances, margin, or angular tolerance are invalid");
  }
  if (!selection.object_id || selection.lane_id.value.empty() ||
    selection.snapshot_revision == 0 || selection.object_revision == 0 ||
    selection.lane_revision == 0 || !finite_positive(selection.product_envelope.radius_m) ||
    !finite_positive(selection.product_envelope.height_m))
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidSelection,
      "selected object, lane, revisions, or product envelope are invalid");
  }
  // Placement is generated from a snapshot taken after the product was grasped, attached and
  // carried clear of the tray, so the selection's revisions cannot still be current: the attach
  // commit alone advances both the object and its source lane. The selection anchors lineage, not
  // freshness, so this evidence must show that it describes the same entities and has not regressed
  // behind the instant the pair was chosen. Each cause is reported separately, since these
  // predicates are checked once after a minute of physical motion and a lumped message names none
  // of them.
  if (object.id != selection.object_id) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidObject,
      "the observed product is not the selected product");
  }
  if (object.revision < selection.object_revision) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidObject,
      "the observed product revision regressed behind the selection");
  }
  if (!placeable_upright_cylinder_class(object.product_class)) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidObject,
      "the held product class has no catalogued upright-cylinder geometry");
  }
  if (object.orientation != ObjectOrientation::Upright) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidObject,
      "the held product is no longer observed upright");
  }
  if (lane.id != selection.lane_id) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane,
      "the observed lane is not the selected destination lane");
  }
  if (lane.revision < selection.lane_revision) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane,
      "the destination lane revision regressed behind the selection");
  }
  if (lane.evidence_revision == 0 || lane.evidence_revision > lane.revision) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane,
      "the destination lane carries no usable occupancy evidence");
  }
  if (!finite_positive(lane.depth_m) || !finite_nonnegative(lane.available_depth_m) ||
    lane.available_depth_m > lane.depth_m)
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane, "the destination lane depth is invalid");
  }
  // Occupancy is not a refusal: the lane is a column and this product joins the back of it. What
  // still refuses is obstruction, which says something overlaps the lane volume without being
  // inside it, so the free depth below is not the length of a column and proves nothing. Whether
  // the room that is left is enough for this product is the depth check further down, against
  // this transfer's own derived requirement.
  if (lane.obstructed) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane,
      "the destination lane became obstructed while the product was in transit");
  }
  // Admission is judged against the policy captured at grant, not the live lane: lane intent may
  // change while the product is in the jaws, and this generator must not contradict the store's
  // own captured-policy revalidation or strand a granted transfer at placement generation.
  if (!restocker_world_state::policy_accepts_object(
      destination_expected_product_class, destination_expected_sku, object))
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidLane,
      "the destination policy captured at grant does not accept the held product class or SKU");
  }
  if (lane_geometry.id != lane.id.value || lane_geometry.frame_id != lane.id.value ||
    lane_geometry.usable_bounds_in_lane.isEmpty() ||
    !lane_geometry.usable_bounds_in_lane.min().allFinite() ||
    !lane_geometry.usable_bounds_in_lane.max().allFinite() ||
    !std::isfinite(lane_geometry.insert_entry_clearance_m) ||
    lane_geometry.insert_entry_clearance_m < 0.0 ||
    !std::isfinite(lane_geometry.incline_rad) ||
    !std::isfinite(lane_geometry.floor_datum_depth_m) ||
    !lane_geometry.insertion_axis.isApprox(Eigen::Vector3d::UnitY(), 1.0e-12) ||
    std::abs(lane.depth_m - lane_geometry.usable_bounds_in_lane.sizes().y()) > 1.0e-9)
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidGeometry,
      "lane manipulation geometry disagrees with the selected lane contract");
  }
  if (!finite_rigid_transform(world_from_lane)) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidGeometry,
      "world to lane transform must be finite and rigid");
  }
  if (!finite_rigid_transform(product_from_grasp_center)) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidAttachmentTransform,
      "product to grasp-center transform must be finite and rigid");
  }
  if (!finite_rigid_transform(tool0_from_grasp_center)) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidToolTransform,
      "tool0 to grasp-center transform must be finite and rigid");
  }

  const Eigen::Vector3d world_insertion =
    world_from_lane.linear() * lane_geometry.insertion_axis;
  // The verified grasp is read through the grasp-centre accessors so that this file and the grasp
  // generator cannot disagree about which column is the approach: the gripper points along +z and
  // its vertical jaw-width axis (+x) is the one that must stay parallel to the product's upright
  // cylinder axis for the held product to arrive in the lane standing up.
  const Eigen::Vector3d product_grasp_approach =
    grasp_approach_axis(product_from_grasp_center);
  const Eigen::Vector3d product_grasp_jaw_width =
    grasp_jaw_width_axis(product_from_grasp_center);
  const double sine_tolerance = std::sin(config.maximum_axis_alignment_error_rad);
  const double cosine_tolerance = std::cos(config.maximum_axis_alignment_error_rad);
  if (std::abs(world_insertion.dot(Eigen::Vector3d::UnitZ())) > sine_tolerance ||
    world_from_lane.linear().col(2).dot(Eigen::Vector3d::UnitZ()) < cosine_tolerance ||
    std::abs(product_grasp_approach.dot(Eigen::Vector3d::UnitZ())) > sine_tolerance ||
    product_grasp_jaw_width.dot(Eigen::Vector3d::UnitZ()) < cosine_tolerance)
  {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidAttachmentTransform,
      "lane and verified grasp axes cannot produce an upright insertion pose");
  }

  // All but seated on the roller bed at the lane mouth: the product is released a hair above it and
  // drops onto it, and the bed carries it to the front rail. The height is referenced to the bed's
  // rear lip, not the bed under the release point, because the insert is a straight horizontal push
  // and the product's uphill rim passes over every part of the bed in between; see
  // lane_insertion_floor_height_m. The containment margin does not apply to this face: holding the
  // product a margin above the bed and letting go would drop it clear of the volume the lane is
  // judged by, which reads as an obstruction. Nor may the product ride flush against the bed for
  // the whole insertion, since the collision checker reports that contact intermittently and the
  // straight-line insert then stops part way.
  //
  // Prefer the configured entry clearance. For the last semantic slot, measured free depth may
  // be a few millimetres short of that nominal pose even though a complete product and both
  // containment margins fit. Approach the existing column only as far as its evidence permits;
  // selection derives this same pose before admitting the pair.
  const double release_depth_m = lane_adaptive_release_product_center_depth_m(
    lane_geometry, selection.product_envelope.radius_m, lane.available_depth_m,
    config.containment_margin_m);
  // The bed has to be able to carry this product forward rather than backward.
  if (release_depth_m >= lane_geometry.floor_datum_depth_m) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::PlacementOutsideLane,
      "the held product cannot be released behind the front rail face: release at " +
      format_metres(release_depth_m) + ", rail face at " +
      format_metres(lane_geometry.floor_datum_depth_m));
  }
  Eigen::Isometry3d lane_from_product = Eigen::Isometry3d::Identity();
  lane_from_product.translation() = Eigen::Vector3d(
    0.0, release_depth_m,
    lane_geometry.usable_bounds_in_lane.min().z() +
    lane_insertion_floor_height_m(lane_geometry) + config.insertion_floor_clearance_m +
    0.5 * selection.product_envelope.height_m);

  const double desired_approach_yaw =
    std::atan2(world_insertion.y(), world_insertion.x());
  const double product_approach_yaw =
    std::atan2(product_grasp_approach.y(), product_grasp_approach.x());
  Eigen::Isometry3d world_from_product = Eigen::Isometry3d::Identity();
  world_from_product.linear() = Eigen::AngleAxisd(
    desired_approach_yaw - product_approach_yaw,
    Eigen::Vector3d::UnitZ()).toRotationMatrix();
  world_from_product.translation() =
    world_from_lane * lane_from_product.translation();
  lane_from_product.linear() =
    world_from_lane.linear().transpose() * world_from_product.linear();

  // Lowering the floor by the margin cancels the inset contains_cylinder applies to every face,
  // and by the settle tolerance on top of it for the fraction of a millimetre a resting product
  // sinks into its support. The remaining five faces keep the full margin.
  Eigen::AlignedBox3d containment_bounds = lane_geometry.usable_bounds_in_lane;
  containment_bounds.min().z() -=
    config.containment_margin_m + lane_geometry.floor_settle_tolerance_m;
  const auto contained = contains_cylinder(
    containment_bounds, lane_from_product,
    selection.product_envelope, config.containment_margin_m);
  const auto product_bounds = cylinder_axis_aligned_bounds(
    lane_from_product, selection.product_envelope);
  if (!contained || !product_bounds) {
    const std::string detail = contained ? product_bounds.error().detail : contained.error().detail;
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InvalidGeometry, detail);
  }
  const double required_depth = product_bounds.value().max().y() -
    lane_geometry.usable_bounds_in_lane.min().y() + config.containment_margin_m;
  if (!contained.value()) {
    // Report which axis overflowed and by how much: the lane bounds, configured final depth and
    // product envelope come from three files, and the numbers say which to change.
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::PlacementOutsideLane,
      "complete product envelope and margin do not fit the lane usable volume: product spans " +
      describe_box(product_bounds.value()) + " in lane frame, usable volume is " +
      describe_box(containment_bounds) + " inset by " +
      format_metres(config.containment_margin_m));
  }
  if (lane.available_depth_m + kDepthRoundingSlackM < required_depth) {
    return failure<std::vector<PlacementCandidate>>(
      PlacementCandidateErrorCode::InsufficientLaneDepth,
      "current lane evidence does not leave enough rear depth for this product: " +
      format_metres(lane.available_depth_m) + " available, " + format_metres(required_depth) +
      " required");
  }

  const Eigen::Isometry3d final_center =
    world_from_product * product_from_grasp_center;
  const Eigen::Isometry3d preinsertion_center = offset_against_insertion(
    final_center, world_insertion, config.preinsertion_distance_m);
  const Eigen::Isometry3d retreat_center = offset_against_insertion(
    final_center, world_insertion, config.retreat_distance_m);
  const Eigen::Isometry3d grasp_center_from_tool0 = tool0_from_grasp_center.inverse();

  std::vector<PlacementCandidate> candidates;
  candidates.push_back(
    PlacementCandidate{
      selection, lane.revision, lane_from_product, world_from_product,
      PlacementPoseSequence{
        preinsertion_center, final_center, retreat_center,
        preinsertion_center * grasp_center_from_tool0,
        final_center * grasp_center_from_tool0,
        retreat_center * grasp_center_from_tool0},
      required_depth});
  return PlacementCandidateResult<std::vector<PlacementCandidate>>::success(
    std::move(candidates));
}

}  // namespace restocker_task_executor
