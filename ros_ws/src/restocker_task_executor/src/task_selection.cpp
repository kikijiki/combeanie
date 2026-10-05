// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/task_selection.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <utility>

namespace restocker_task_executor
{
namespace
{

// The dense fixture's declared can capacity sits 9.6 mm inside the analytic contact boundary.
// Below every product pitch (66+ mm); affects only the semantic bound, not the free-depth gate.
constexpr double kSemanticCapacityToleranceM = 0.010;
constexpr double kDepthRoundingSlackM = 1.0e-9;

using restocker_world_state::LaneId;
using restocker_world_state::ObjectId;
using restocker_world_state::FaultState;
using restocker_world_state::ProductClass;
using restocker_world_state::ShelfLane;
using restocker_world_state::TaskPhase;
using restocker_world_state::TrackedObject;
using restocker_world_state::WorldStateSnapshot;

template<typename T>
[[nodiscard]] SelectionResult<T> failure(SelectionErrorCode code, std::string detail)
{
  return SelectionResult<T>::failure(SelectionError{code, std::move(detail)});
}

[[nodiscard]] bool finite_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

// The world state and the collision catalog enumerate product classes independently, so a
// snapshot class is translated here. Unknown has no geometry and must not fall back to another
// class's envelope.
[[nodiscard]] std::optional<CatalogProductClass> catalog_product_class(
  ProductClass product_class) noexcept
{
  switch (product_class) {
    case ProductClass::Can: return CatalogProductClass::Can;
    case ProductClass::SmallBottle: return CatalogProductClass::SmallBottle;
    case ProductClass::LargeBottle: return CatalogProductClass::LargeBottle;
    case ProductClass::Unknown: return std::nullopt;
  }
  return std::nullopt;
}

[[nodiscard]] std::string pair_context(ObjectId object_id, const LaneId & lane_id)
{
  std::ostringstream stream;
  stream << "object " << object_id.value << " and lane " << lane_id.value << ": ";
  return stream.str();
}

// One entry per (object, refusal code): how many destinations refused the object that way, with
// the first refusal's own detail — which already names one destination through `pair_context`.
// A bare "no eligible pair" cannot be classified from a receipt (Card 010's cycle-2
// `premature NO_COMPATIBLE_PAIR` had no predicate in the log at all), while a full per-pair dump
// would bury the answer under six lanes. Ordered by object id then code, so the line is
// deterministic and diffable between runs.
using PairRefusalTally =
  std::map<std::pair<std::uint64_t, SelectionErrorCode>, std::pair<std::size_t, std::string>>;

void note_pair_refusal(
  PairRefusalTally & tally, const ObjectId & object_id, const SelectionError & error)
{
  auto & entry = tally[std::pair{object_id.value, error.code}];
  if (entry.first == 0U) {
    entry.second = error.detail;
  }
  ++entry.first;
}

[[nodiscard]] std::string describe_pair_refusals(const PairRefusalTally & tally)
{
  if (tally.empty()) {
    return {};
  }
  std::ostringstream stream;
  stream << "; pair refusals:";
  for (const auto & [key, entry] : tally) {
    stream << " object " << key.first << ' ' << to_string(key.second) << " x" << entry.first
           << " (" << entry.second << ')';
  }
  return stream.str();
}

[[nodiscard]] std::optional<SelectionError> freshness_error(
  const rclcpp::Time & observation_time, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_age, std::chrono::nanoseconds maximum_future_skew,
  SelectionErrorCode stale_code, const std::string & subject)
{
  if (observation_time.get_clock_type() != now.get_clock_type()) {
    return SelectionError{SelectionErrorCode::InvalidSnapshot, subject + " clock type mismatches"};
  }
  const std::int64_t age = now.nanoseconds() - observation_time.nanoseconds();
  if (age > maximum_age.count()) {
    return SelectionError{stale_code, subject + " is stale"};
  }
  if (age < -maximum_future_skew.count()) {
    return SelectionError{SelectionErrorCode::InvalidSnapshot, subject + " is too far in future"};
  }
  return std::nullopt;
}

[[nodiscard]] SelectionResult<CylinderEnvelope> eligible_object(
  const TrackedObject & object, const ProductCollisionCatalog & catalog,
  const WorkcellManipulationGeometry & geometry, const Eigen::Isometry3d & shelf_from_world,
  const SelectionConfig & config)
{
  using restocker_world_state::GraspState;
  using restocker_world_state::ObjectOrientation;
  using restocker_world_state::TrackingState;

  if (object.tracking_state != TrackingState::Tracked ||
    object.grasp_state != GraspState::Free)
  {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::ObjectUnavailable, "object is not tracked and free");
  }
  if (object.orientation != ObjectOrientation::Upright) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::UnsupportedProduct, "baseline supports only upright objects");
  }
  if (!finite_rigid_transform(object.pose_in_world)) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::InvalidSnapshot, "object pose is not finite and rigid");
  }
  const double upright_alignment = std::clamp(
    object.pose_in_world.linear().col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  if (std::acos(upright_alignment) > config.maximum_upright_tilt_rad) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::UnsupportedProduct,
      "object orientation label disagrees with its cylinder axis");
  }
  const auto catalog_class = catalog_product_class(object.product_class);
  if (!catalog_class) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::UnsupportedProduct,
      "product class has no catalogued upright-cylinder geometry");
  }
  if (auto error = freshness_error(
      object.observation_time, config.now, config.maximum_object_age,
      config.maximum_future_skew, SelectionErrorCode::ObjectStale, "object"))
  {
    return SelectionResult<CylinderEnvelope>::failure(std::move(*error));
  }

  auto resolved = catalog.resolve(*catalog_class, object.sku);
  if (!resolved || resolved.value().primitive.dimensions.size() != 2U) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::MissingGeometry,
      resolved ? "resolved product geometry is not a cylinder" : resolved.error().detail);
  }
  const CylinderEnvelope envelope{
    resolved.value().primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS],
    resolved.value().primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT]};
  Eigen::AlignedBox3d stock_bounds = geometry.stock_region_in_shelf;
  stock_bounds.min() -= Eigen::Vector3d(
    config.stock_boundary_tolerance_m, config.stock_boundary_tolerance_m, 0.0);
  stock_bounds.max() += Eigen::Vector3d(
    config.stock_boundary_tolerance_m, config.stock_boundary_tolerance_m,
    config.stock_boundary_tolerance_m);
  const auto contained = contains_cylinder(
    stock_bounds, shelf_from_world * object.pose_in_world, envelope,
    config.stock_containment_margin_m);
  if (!contained) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::InvalidConfiguration,
      contained.error().detail);
  }
  if (!contained.value()) {
    return failure<CylinderEnvelope>(
      SelectionErrorCode::ObjectOutsideStock,
      "complete product envelope is outside the stock usable volume");
  }
  return SelectionResult<CylinderEnvelope>::success(envelope);
}

// Strict weak ordering over eligible pairs. The first two keys are the deficit policy of
// Milestone 10 §4: the largest deficit first, ties broken by the rail distance from the arm's
// current position — the destination is chosen before the product is. What follows is the
// original object-side rule: within the winning destination, clear the frontmost tray row before
// a cheaper rear one, then total travel, then identity. Depth and totals are compared exactly
// because generated rows and true ties are bit-identical; a tolerance would make the result
// depend on visit order.
[[nodiscard]] bool cheaper_pair(
  const SelectedTaskPair & candidate, const SelectedTaskPair & best,
  const WorldStateSnapshot & snapshot, const Eigen::Isometry3d & shelf_from_world)
{
  if (candidate.destination_deficit != best.destination_deficit) {
    return candidate.destination_deficit > best.destination_deficit;
  }
  if (candidate.destination_rail_distance_m != best.destination_rail_distance_m) {
    return candidate.destination_rail_distance_m < best.destination_rail_distance_m;
  }
  const auto candidate_object = snapshot.objects.find(candidate.object_id);
  const auto best_object = snapshot.objects.find(best.object_id);
  if (candidate.object_id != best.object_id &&
    candidate_object != snapshot.objects.end() && best_object != snapshot.objects.end())
  {
    const double candidate_depth =
      (shelf_from_world * candidate_object->second.pose_in_world).translation().y();
    const double best_depth =
      (shelf_from_world * best_object->second.pose_in_world).translation().y();
    if (candidate_depth != best_depth) {
      return candidate_depth > best_depth;
    }
  }
  if (candidate.score.total != best.score.total) {
    return candidate.score.total < best.score.total;
  }
  if (candidate.object_id != best.object_id) {
    return candidate.object_id < best.object_id;
  }
  return candidate.lane_id < best.lane_id;
}

[[nodiscard]] std::size_t measured_lane_occupancy(
  const WorldStateSnapshot & snapshot, const ShelfLane & lane)
{
  std::set<std::string> observed_sources(
    lane.observed_source_object_ids.begin(), lane.observed_source_object_ids.end());
  std::size_t occupancy = observed_sources.size();
  for (const ObjectId content_id : lane.contents) {
    const auto object = snapshot.objects.find(content_id);
    if (object == snapshot.objects.end() ||
      !observed_sources.contains(object->second.source_object_id))
    {
      ++occupancy;
    }
  }
  return occupancy;
}

// A ledger that names products this lane's policy does not accept: typically a runtime policy
// change that re-parked the lane while the old column is still in it. The deficit formula would
// happily compare the old column against the new want; the spec's answer is to report the
// conflict and skip the lane, because a gravity-fed shelf cannot be emptied from behind. Each
// entry's tracked object keeps its class even when the ledger's length claim has been withdrawn,
// so this runs on unreliable lanes too — that is what makes "reported and skipped" hold after
// the pitch mismatch a policy change usually causes. Seeded lanes have empty ledgers and are
// unaffected.
[[nodiscard]] SelectionResult<std::monostate> lane_ledger_identity_error(
  const WorldStateSnapshot & snapshot, const ShelfLane & lane)
{
  for (const ObjectId content_id : lane.contents) {
    const auto object = snapshot.objects.find(content_id);
    if (object == snapshot.objects.end()) {
      return failure<std::monostate>(
        SelectionErrorCode::InvalidSnapshot,
        "lane " + lane.id.value + " contents reference an object absent from the snapshot");
    }
    if (!restocker_world_state::policy_accepts_object(
        lane.expected_product_class, lane.expected_sku, object->second))
    {
      return failure<std::monostate>(
        SelectionErrorCode::LaneWrongProduct,
        "lane " + lane.id.value + " ledger holds object " + std::to_string(content_id.value) +
        " of class/SKU the current lane policy does not accept; the lane is reported and skipped");
    }
  }
  return SelectionResult<std::monostate>::success(std::monostate{});
}

// World state counts a product inside a lane from `lane.contents`: commit_detachment appends it
// at the detach and commit_attachment removes it at the attach. Membership and the tracked pose
// can disagree, because a placement frees the object and appends it to the destination's
// contents without refreshing the pose — the product keeps the stock-region pose of its last
// tray observation until the next observation arrives. `eligible_object` judges only the pose,
// so without this clause a just-placed product still reads as free stock, the sweep pairs it
// with the lane it already belongs to, and the reserve guards refuse that pair terminally once
// the arm has been dispatched (Card 010 attempt 2 = cmbdef2 attempt 9: `reserve request
// construction failed exact validation: selected object has ambiguous or destination
// membership`). A member of any lane is not a selectable stock product, whatever its pose says;
// the loop then reports no eligible pair and re-surveys instead of inhibiting. The same fact
// keeps a placed product out of the tray feed-queue check below: it has left the tray, so its
// stale pose must not go on blocking the product that stands behind it.
[[nodiscard]] std::optional<LaneId> lane_membership(
  const WorldStateSnapshot & snapshot, const ObjectId & object_id, bool & ambiguous)
{
  std::optional<LaneId> member;
  ambiguous = false;
  for (const auto & [lane_id, lane] : snapshot.lanes) {
    if (std::find(lane.contents.begin(), lane.contents.end(), object_id) == lane.contents.end()) {
      continue;
    }
    if (member) {
      ambiguous = true;
      return member;
    }
    member = lane_id;
  }
  return member;
}

[[nodiscard]] bool has_product_ahead_in_tray_queue(
  const WorldStateSnapshot & snapshot, const TrackedObject & object,
  const CylinderEnvelope & envelope, const WorkcellManipulationGeometry & geometry,
  const Eigen::Isometry3d & shelf_from_world)
{
  const Eigen::Vector3d candidate = (shelf_from_world * object.pose_in_world).translation();
  const auto & stock = geometry.stock_region_in_shelf;
  const double same_queue_tolerance_m = 0.5 * envelope.radius_m;
  constexpr double kAheadToleranceM = 0.001;
  for (const auto & [other_id, other] : snapshot.objects) {
    if (other_id == object.id ||
      other.tracking_state != restocker_world_state::TrackingState::Tracked ||
      other.grasp_state != restocker_world_state::GraspState::Free)
    {
      continue;
    }
    // A product world state already counts in a lane has left the tray; its stock-region pose is
    // the last observation before the grasp, and letting it stand in the queue would keep
    // refusing the product behind it after its own placement.
    bool other_in_lane = false;
    if (lane_membership(snapshot, other_id, other_in_lane)) {
      continue;
    }
    const Eigen::Vector3d other_center =
      (shelf_from_world * other.pose_in_world).translation();
    const bool other_is_in_stock =
      other_center.x() >= stock.min().x() && other_center.x() <= stock.max().x() &&
      other_center.y() >= stock.min().y() && other_center.y() <= stock.max().y() &&
      other_center.z() >= stock.min().z() && other_center.z() <= stock.max().z();
    if (other_is_in_stock &&
      std::abs(other_center.x() - candidate.x()) <= same_queue_tolerance_m &&
      other_center.y() > candidate.y() + kAheadToleranceM)
    {
      return true;
    }
  }
  return false;
}

[[nodiscard]] SelectionResult<SelectedTaskPair> eligible_pair(
  const WorldStateSnapshot & snapshot, const TrackedObject & object, const ShelfLane & lane,
  const ProductCollisionCatalog & catalog, const WorkcellManipulationGeometry & geometry,
  const Eigen::Isometry3d & world_from_shelf, const Eigen::Isometry3d & shelf_from_world,
  const SelectionConfig & config)
{
  const std::string context = pair_context(object.id, lane.id);
  auto object_result = eligible_object(object, catalog, geometry, shelf_from_world, config);
  if (!object_result) {
    return failure<SelectedTaskPair>(
      object_result.error().code,
      context + object_result.error().detail);
  }
  bool ambiguous_membership = false;
  const auto member_lane = lane_membership(snapshot, object.id, ambiguous_membership);
  if (member_lane) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::ObjectUnavailable,
      context + (ambiguous_membership ?
      "world state counts this object in more than one lane, so it is not a free stock product" :
      "world state already counts this object in lane " + member_lane->value +
      ", so it is not a free stock product"));
  }
  if (has_product_ahead_in_tray_queue(
      snapshot, object, object_result.value(), geometry, shelf_from_world))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::ObjectUnavailable,
      context + "another free tray product blocks the front of this feed queue");
  }
  const auto lane_geometry = geometry.lanes.find(lane.id.value);
  if (lane_geometry == geometry.lanes.end()) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration, context + "lane has no manipulation geometry");
  }
  if (!std::isfinite(lane.depth_m) || lane.depth_m <= 0.0 ||
    !std::isfinite(lane.available_depth_m) || lane.available_depth_m < 0.0 ||
    lane.available_depth_m > lane.depth_m || lane.evidence_revision == 0)
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidSnapshot, context + "lane depth state is invalid");
  }
  if (std::abs(lane.depth_m - lane_geometry->second.usable_bounds_in_lane.sizes().y()) > 1.0e-9) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration,
      context + "world-state lane depth disagrees with manipulation geometry");
  }
  // A lane that already holds products and has room for one more is normal, so occupancy is not
  // a reason to skip a lane; the depth check below is. Obstruction is: something overlaps the
  // lane volume without being contained by it, so free depth is not a column length.
  if (lane.obstructed) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::LaneUnavailable,
      context + "destination is obstructed");
  }
  if (lane.evidence_invalidated) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::LaneEvidenceInvalidated,
      context + "destination depletion evidence was invalidated and has not been re-surveyed");
  }
  if (auto error = freshness_error(
      lane.last_verified, config.now, config.lane_evidence_validity,
      config.maximum_future_skew, SelectionErrorCode::LaneStale, "lane"))
  {
    return failure<SelectedTaskPair>(error->code, context + error->detail);
  }
  if ((lane.expected_product_class != ProductClass::Unknown &&
    lane.expected_product_class != object.product_class) ||
    (lane.expected_sku && lane.expected_sku != object.sku))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::IncompatiblePair,
      context + "product identity is incompatible with lane policy");
  }
  const auto identity_error = lane_ledger_identity_error(snapshot, lane);
  if (!identity_error) {
    return failure<SelectedTaskPair>(
      identity_error.error().code, context + identity_error.error().detail);
  }
  // Milestone 10 §4: held count and deficit against the owner's target_count, from the
  // catalogued pitch. Computed for every pair — it is both an eligibility fact (a lane whose
  // ledger names the wrong product already refused above) and the primary ordering key.
  const auto deficit = lane_deficit(lane, lane_geometry->second, catalog);
  if (!deficit) {
    return failure<SelectedTaskPair>(deficit.error().code, context + deficit.error().detail);
  }
  const double lane_world_x_m =
    (world_from_shelf * Eigen::Vector3d(lane_geometry->second.center_x_m, 0.0, 0.0)).x();
  const double lane_rail_distance_m =
    std::abs(lane_world_x_m - snapshot.robot.rail_position);
  if (!std::isfinite(lane_rail_distance_m)) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration,
      context + "destination rail distance is not finite");
  }

  const auto capacity = lane_column_capacity(
    lane_geometry->second, object_result.value(), kSemanticCapacityToleranceM);
  if (!capacity) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration, context + capacity.error().detail);
  }
  // Depth is the primary evidence; semantic occupancy is an independent upper bound. Free depth
  // has been observed growing back after a placement into a lane of pinned products, and without
  // the bound an autonomous run would overfill that lane.
  //
  // Both predicates derive from the same surveyed lane and catalogued cylinder, so disagreement
  // fails toward "full".
  if (measured_lane_occupancy(snapshot, lane) >= capacity.value()) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InsufficientLaneDepth,
      context + "semantic column occupancy reached the surveyed lane capacity");
  }

  const double nominal_required_depth = 2.0 * object_result.value().radius_m +
    lane_geometry->second.insert_entry_clearance_m;
  if (lane.available_depth_m + config.destination_entry_depth_tolerance_m <
    nominal_required_depth)
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InsufficientLaneDepth,
      context + "measured rear depth is beyond the bounded adaptive release tolerance");
  }

  // Same evidence-bounded release pose as placement generation. The configured entry clearance
  // is preferred; the last semantic slot may reduce it, keeping the product and containment
  // margin inside measured free depth.
  Eigen::Isometry3d lane_from_product = Eigen::Isometry3d::Identity();
  lane_from_product.translation() = Eigen::Vector3d(
    0.0,
    lane_adaptive_release_product_center_depth_m(
      lane_geometry->second, object_result.value().radius_m, lane.available_depth_m,
      config.destination_containment_margin_m),
    lane_geometry->second.usable_bounds_in_lane.min().z() +
    lane_insertion_floor_height_m(lane_geometry->second) +
    0.5 * object_result.value().height_m);
  const auto contained = contains_cylinder(
    lane_geometry->second.usable_bounds_in_lane, lane_from_product, object_result.value(),
    config.destination_containment_margin_m);
  const auto product_bounds = cylinder_axis_aligned_bounds(
    lane_from_product, object_result.value());
  if (!contained || !product_bounds) {
    const std::string detail = contained ? product_bounds.error().detail : contained.error().detail;
    return failure<SelectedTaskPair>(SelectionErrorCode::InvalidConfiguration, context + detail);
  }
  const double required_depth = product_bounds.value().max().y() -
    lane_geometry->second.usable_bounds_in_lane.min().y() +
    config.destination_containment_margin_m;
  if (!contained.value() ||
    lane.available_depth_m + kDepthRoundingSlackM < required_depth)
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InsufficientLaneDepth,
      context + "configured placement does not fit available lane depth");
  }

  // Both legs are measured along world x, the carriage axis (snapshot.robot.rail_position is
  // already on it). The lane contributes its release point, not its frame origin, so a lane with
  // an offset placement point is costed where the carriage has to stand.
  const Eigen::Vector3d shelf_from_placement(
    lane_geometry->second.center_x_m + lane_from_product.translation().x(),
    lane_from_product.translation().y(), lane_from_product.translation().z());
  const double product_x_m = object.pose_in_world.translation().x();
  SelectionScore score;
  score.approach_rail_travel_m = std::abs(product_x_m - snapshot.robot.rail_position);
  score.delivery_rail_travel_m = std::abs(
    (world_from_shelf * shelf_from_placement).x() - product_x_m);
  score.total = config.approach_rail_travel_weight * score.approach_rail_travel_m +
    config.delivery_rail_travel_weight * score.delivery_rail_travel_m;
  // A non-finite cost would order every comparison as "not less than": an unscoreable pair is a
  // configuration fault.
  if (!std::isfinite(score.approach_rail_travel_m) ||
    !std::isfinite(score.delivery_rail_travel_m) || !std::isfinite(score.total))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration, context + "pair cost is not finite");
  }
  return SelectionResult<SelectedTaskPair>::success(
    SelectedTaskPair{
        object.id, lane.id, snapshot.revision, object.revision, lane.revision,
        object_result.value(), score, deficit.value().deficit, lane_rail_distance_m,
        lane.ledger_unreliable});
}

}  // namespace

// Milestone 10 §4: held count and deficit from the measured column length at the catalogued
// pitch. The pitch is the expected product's own envelope foreshortened by the lane's incline —
// the same arithmetic `lane_column_capacity` and the world state's product-lane profiles use —
// so the three cannot drift. A lane with no catalogued geometry for its expected product has no
// deficit to compute and fails closed here rather than inheriting a sibling product's pitch.
// Declared in the header for tests and future Stage 7 consumers, so it lives at namespace scope.
[[nodiscard]] SelectionResult<LaneDeficit> lane_deficit(
  const ShelfLane & lane, const LaneManipulationGeometry & lane_geometry,
  const ProductCollisionCatalog & product_catalog)
{
  const auto catalog_class = catalog_product_class(lane.expected_product_class);
  if (!catalog_class) {
    return failure<LaneDeficit>(
      SelectionErrorCode::MissingGeometry,
      "lane " + lane.id.value +
      " has no catalogued product for its expected class, so no deficit can be computed");
  }
  auto resolved = product_catalog.resolve(*catalog_class, lane.expected_sku);
  if (!resolved || resolved.value().primitive.dimensions.size() != 2U) {
    return failure<LaneDeficit>(
      SelectionErrorCode::MissingGeometry,
      "lane " + lane.id.value +
      (resolved ? " resolved product geometry is not a cylinder" :
      ": " + resolved.error().detail));
  }
  const double radius_m = resolved.value().primitive.dimensions[
    shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS];
  if (!std::isfinite(radius_m) || radius_m <= 0.0 ||
    !std::isfinite(lane_geometry.incline_rad) || lane_geometry.incline_rad < 0.0 ||
    !std::isfinite(lane.depth_m) || lane.depth_m <= 0.0 ||
    !std::isfinite(lane.available_depth_m) || lane.available_depth_m < 0.0 ||
    lane.available_depth_m > lane.depth_m)
  {
    return failure<LaneDeficit>(
      SelectionErrorCode::InvalidConfiguration,
      "lane " + lane.id.value + " depth or catalogued radius cannot yield a product pitch");
  }
  const double pitch_m = 2.0 * radius_m * std::cos(lane_geometry.incline_rad);
  if (!std::isfinite(pitch_m) || pitch_m <= 0.0) {
    return failure<LaneDeficit>(
      SelectionErrorCode::InvalidConfiguration,
      "lane " + lane.id.value + " derived a non-positive product pitch");
  }
  const double column_length_m = lane.depth_m - lane.available_depth_m;
  const auto held = static_cast<std::uint32_t>(std::lround(column_length_m / pitch_m));
  LaneDeficit deficit;
  deficit.held_count = held;
  deficit.deficit = lane.target_count > held ? lane.target_count - held : 0U;
  deficit.column_length_m = column_length_m;
  deficit.pitch_m = pitch_m;
  return SelectionResult<LaneDeficit>::success(deficit);
}

SelectionResult<SelectedTaskPair> select_task_pair(
  const WorldStateSnapshot & snapshot, const ProductCollisionCatalog & product_catalog,
  const WorkcellManipulationGeometry & workcell_geometry,
  const Eigen::Isometry3d & world_from_shelf, const SelectionConfig & config,
  const SelectionRequest & request)
{
  if (config.maximum_object_age.count() < 0 || config.lane_evidence_validity.count() <= 0 ||
    config.maximum_robot_age.count() < 0 ||
    config.maximum_future_skew.count() < 0 ||
    config.perception_liveness_max_age.count() < 0 ||
    !std::isfinite(config.stock_containment_margin_m) ||
    config.stock_containment_margin_m < 0.0 ||
    !std::isfinite(config.stock_boundary_tolerance_m) ||
    config.stock_boundary_tolerance_m < 0.0 ||
    !std::isfinite(config.maximum_upright_tilt_rad) ||
    config.maximum_upright_tilt_rad < 0.0 ||
    config.maximum_upright_tilt_rad >= 0.5 * std::acos(-1.0) ||
    !std::isfinite(config.approach_rail_travel_weight) ||
    config.approach_rail_travel_weight < 0.0 ||
    !std::isfinite(config.delivery_rail_travel_weight) ||
    config.delivery_rail_travel_weight < 0.0 ||
    !std::isfinite(config.destination_containment_margin_m) ||
    config.destination_containment_margin_m < 0.0 ||
    !std::isfinite(config.destination_entry_depth_tolerance_m) ||
    config.destination_entry_depth_tolerance_m < 0.0 ||
    !finite_rigid_transform(world_from_shelf))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidConfiguration, "selection configuration is invalid");
  }
  if (config.last_perception_acquisition) {
    if (auto error = freshness_error(
        *config.last_perception_acquisition, config.now, config.perception_liveness_max_age,
        config.maximum_future_skew, SelectionErrorCode::PerceptionUnlive, "perception stream"))
    {
      return SelectionResult<SelectedTaskPair>::failure(std::move(*error));
    }
  }
  // A lane selector is only meaningful beside the object it selects (Milestone 10 §1
  // confirmed-identity contract, Card 037): an object-only request names the product and leaves
  // the destination to this sweep, which is how a close-confirmed candidate is transferred
  // without handing the campaign a second selection authority.
  if (!request.object_id.has_value() && request.lane_id.has_value()) {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::RequestedIdentityIncomplete,
      "a requested lane ID requires the object ID it selects");
  }
  if (snapshot.robot.telemetry_revision == 0 ||
    snapshot.robot.telemetry_time.nanoseconds() <= 0 ||
    snapshot.robot.telemetry_revision > snapshot.robot.revision ||
    snapshot.robot.revision > snapshot.revision ||
    !std::isfinite(snapshot.robot.rail_position) ||
    !std::ranges::all_of(
      snapshot.robot.joint_positions, [](double value) {return std::isfinite(value);}))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::InvalidSnapshot, "snapshot contains invalid robot telemetry");
  }
  if (auto error = freshness_error(
      snapshot.robot.telemetry_time, config.now, config.maximum_robot_age,
      config.maximum_future_skew, SelectionErrorCode::RobotStale, "robot telemetry"))
  {
    return failure<SelectedTaskPair>(error->code, error->detail);
  }
  if (snapshot.robot.held_object || snapshot.robot.fault_state != FaultState::None ||
    (snapshot.robot.task_phase != TaskPhase::Idle &&
    snapshot.robot.task_phase != TaskPhase::ValidatingScene))
  {
    return failure<SelectedTaskPair>(
      SelectionErrorCode::RobotUnavailable, "robot is not available for task selection");
  }
  for (const auto & [id, object] : snapshot.objects) {
    if (!id || id != object.id || object.source_object_id.empty() || object.revision == 0 ||
      object.revision > snapshot.revision)
    {
      return failure<SelectedTaskPair>(
        SelectionErrorCode::InvalidSnapshot, "snapshot contains invalid object identity/revision");
    }
  }
  for (const auto & [id, lane] : snapshot.lanes) {
    // evidence_revision == 0 (never surveyed) is a per-lane eligibility failure (eligible_pair),
    // not a corrupt snapshot. evidence_revision > revision is inconsistent and fails the snapshot.
    if (id.value.empty() || id != lane.id || lane.revision == 0 ||
      lane.revision > snapshot.revision ||
      (lane.evidence_revision != 0 && lane.evidence_revision > lane.revision))
    {
      return failure<SelectedTaskPair>(
        SelectionErrorCode::InvalidSnapshot, "snapshot contains invalid lane identity/revision");
    }
  }

  const Eigen::Isometry3d shelf_from_world = world_from_shelf.inverse();
  if (request.object_id) {
    const auto object = snapshot.objects.find(*request.object_id);
    if (object == snapshot.objects.end()) {
      return failure<SelectedTaskPair>(SelectionErrorCode::ObjectNotFound, "object was not found");
    }
    if (request.lane_id) {
      const auto lane = snapshot.lanes.find(*request.lane_id);
      if (lane == snapshot.lanes.end()) {
        return failure<SelectedTaskPair>(SelectionErrorCode::LaneNotFound, "lane was not found");
      }
      return eligible_pair(
        snapshot, object->second, lane->second, product_catalog, workcell_geometry,
        world_from_shelf, shelf_from_world, config);
    }
    // Object-only (Milestone 10 §1 confirmed-identity contract, Card 037): the product is named
    // and only the destination is chosen, among the lanes compatible with that object under the
    // same §4 ordering and eligibility the automatic sweep applies. A name with no eligible
    // destination fails closed with a typed error — the sweep can never return a different
    // object for a named request. The stale-only barrier mirrors the automatic sweep so an
    // aged confirmed object reports the staleness it actually hit.
    std::optional<SelectedTaskPair> best;
    std::optional<SelectionError> stale_barrier;
    PairRefusalTally refusals;
    bool saw_non_stale_ineligibility = false;
    for (const auto & [lane_id, lane] : snapshot.lanes) {
      static_cast<void>(lane_id);
      auto pair = eligible_pair(
        snapshot, object->second, lane, product_catalog, workcell_geometry,
        world_from_shelf, shelf_from_world, config);
      if (!pair) {
        note_pair_refusal(refusals, *request.object_id, pair.error());
        switch (pair.error().code) {
          case SelectionErrorCode::ObjectStale:
          case SelectionErrorCode::LaneStale:
            if (!stale_barrier) {
              stale_barrier = pair.error();
            }
            break;
          default:
            saw_non_stale_ineligibility = true;
            break;
        }
        continue;
      }
      if (!best || cheaper_pair(pair.value(), *best, snapshot, shelf_from_world)) {
        best = std::move(pair.value());
      }
    }
    if (best) {
      return SelectionResult<SelectedTaskPair>::success(std::move(*best));
    }
    if (stale_barrier && !saw_non_stale_ineligibility) {
      return SelectionResult<SelectedTaskPair>::failure(std::move(*stale_barrier));
    }
    return failure<SelectedTaskPair>(
      SelectionErrorCode::NoEligiblePair,
      "requested object " + std::to_string(request.object_id->value) +
      " has no eligible destination lane" + describe_pair_refusals(refusals));
  }

  // Every eligible pair is scored and the deficit ordering of §4 keeps the best (container order
  // is not a preference). Ineligible pairs are skipped so one obstructed lane does not block a
  // compatible sibling; if none is eligible the caller gets NoEligiblePair, a non-fault outcome.
  // Two named conditions are reported instead, because NoEligiblePair would hide them: when
  // every rejected pair failed only because evidence aged out (an empty shelf would wrongly
  // imply a dead producer), and when every destination's ledger names the wrong product (the
  // conflict itself is the answer — checked per lane before pairing, since a per-pair check can
  // never be the *sole* refusal while the conflicting entry's own object still exists to
  // mismatch the same policy).
  std::optional<SelectedTaskPair> best;
  std::optional<SelectionError> stale_barrier;
  std::optional<SelectionError> wrong_product_barrier;
  PairRefusalTally refusals;
  std::size_t wrong_product_lanes = 0U;
  for (const auto & [lane_id, lane] : snapshot.lanes) {
    static_cast<void>(lane_id);
    const auto identity_error = lane_ledger_identity_error(snapshot, lane);
    if (!identity_error) {
      ++wrong_product_lanes;
      if (!wrong_product_barrier) {
        wrong_product_barrier = identity_error.error();
      }
    }
  }
  bool saw_non_stale_ineligibility = false;
  for (const auto & [object_id, object] : snapshot.objects) {
    for (const auto & [lane_id, lane] : snapshot.lanes) {
      static_cast<void>(lane_id);
      if (!lane_ledger_identity_error(snapshot, lane)) {
        continue;
      }
      auto pair = eligible_pair(
        snapshot, object, lane, product_catalog, workcell_geometry, world_from_shelf,
        shelf_from_world, config);
      if (!pair) {
        note_pair_refusal(refusals, object_id, pair.error());
        switch (pair.error().code) {
          case SelectionErrorCode::ObjectStale:
          case SelectionErrorCode::LaneStale:
            if (!stale_barrier) {
              stale_barrier = pair.error();
            }
            break;
          default:
            saw_non_stale_ineligibility = true;
            break;
        }
        continue;
      }
      if (!best || cheaper_pair(pair.value(), *best, snapshot, shelf_from_world)) {
        best = std::move(pair.value());
      }
    }
  }
  if (best) {
    return SelectionResult<SelectedTaskPair>::success(std::move(*best));
  }
  if (stale_barrier && !saw_non_stale_ineligibility &&
    wrong_product_lanes < snapshot.lanes.size())
  {
    return SelectionResult<SelectedTaskPair>::failure(std::move(*stale_barrier));
  }
  if (wrong_product_barrier && wrong_product_lanes == snapshot.lanes.size() &&
    !snapshot.lanes.empty())
  {
    return SelectionResult<SelectedTaskPair>::failure(std::move(*wrong_product_barrier));
  }
  return failure<SelectedTaskPair>(
    SelectionErrorCode::NoEligiblePair,
    "no deterministic object/lane pair satisfies the baseline predicate" +
    describe_pair_refusals(refusals));
}

std::string describe_perception_liveness(
  const PerceptionLivenessObservation & observation, const rclcpp::Time & now,
  std::chrono::nanoseconds max_age)
{
  std::ostringstream stream;
  const auto horizon_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(max_age).count();
  const auto counters = [&observation]() {
    std::ostringstream tail;
    tail << "arrivals=" << observation.arrivals
         << " stamp_advances=" << observation.stamp_advances;
    return tail.str();
  };
  if (!observation.last_acquisition) {
    stream << "no wrist camera acquisition has been observed yet (" << counters() << ")";
    return stream.str();
  }
  if (observation.last_acquisition->get_clock_type() != now.get_clock_type()) {
    stream << "wrist camera acquisition clock type mismatches (" << counters() << ")";
    return stream.str();
  }
  const auto age_ms =
    (now - *observation.last_acquisition).nanoseconds() / 1'000'000;
  const auto receipt_lag_ms = observation.last_received_at ?
    (*observation.last_received_at - *observation.last_acquisition).nanoseconds() / 1'000'000 :
    std::int64_t{-1};
  stream << "wrist camera acquisition is " << age_ms << " ms old (horizon " << horizon_ms
         << " ms, last stamp " << observation.last_acquisition->nanoseconds()
         << " ns, now " << now.nanoseconds() << " ns, receipt lag " << receipt_lag_ms
         << " ms, " << counters() << ")";
  if (age_ms > horizon_ms) {
    // Keep the historic phrase so a retained log's grep for the unlive reason still matches.
    stream << "; wrist camera acquisition is older than the configured liveness horizon";
  }
  return stream.str();
}

std::string to_string(SelectionErrorCode code)
{
  switch (code) {
    case SelectionErrorCode::InvalidConfiguration: return "invalid_configuration";
    case SelectionErrorCode::InvalidSnapshot: return "invalid_snapshot";
    case SelectionErrorCode::RequestedIdentityIncomplete: return "requested_identity_incomplete";
    case SelectionErrorCode::ObjectNotFound: return "object_not_found";
    case SelectionErrorCode::LaneNotFound: return "lane_not_found";
    case SelectionErrorCode::ObjectUnavailable: return "object_unavailable";
    case SelectionErrorCode::ObjectStale: return "object_stale";
    case SelectionErrorCode::RobotUnavailable: return "robot_unavailable";
    case SelectionErrorCode::RobotStale: return "robot_stale";
    case SelectionErrorCode::UnsupportedProduct: return "unsupported_product";
    case SelectionErrorCode::MissingGeometry: return "missing_geometry";
    case SelectionErrorCode::ObjectOutsideStock: return "object_outside_stock";
    case SelectionErrorCode::LaneUnavailable: return "lane_unavailable";
    case SelectionErrorCode::LaneStale: return "lane_stale";
    case SelectionErrorCode::LaneEvidenceInvalidated: return "lane_evidence_invalidated";
    case SelectionErrorCode::PerceptionUnlive: return "perception_unlive";
    case SelectionErrorCode::IncompatiblePair: return "incompatible_pair";
    case SelectionErrorCode::LaneWrongProduct: return "lane_wrong_product";
    case SelectionErrorCode::InsufficientLaneDepth: return "insufficient_lane_depth";
    case SelectionErrorCode::NoEligiblePair: return "no_eligible_pair";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
