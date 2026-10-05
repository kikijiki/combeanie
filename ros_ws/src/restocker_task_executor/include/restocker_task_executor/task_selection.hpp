// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include <rclcpp/time.hpp>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

enum class SelectionErrorCode : std::uint8_t
{
  InvalidConfiguration,
  InvalidSnapshot,
  RequestedIdentityIncomplete,
  ObjectNotFound,
  LaneNotFound,
  ObjectUnavailable,
  ObjectStale,
  RobotUnavailable,
  RobotStale,
  UnsupportedProduct,
  MissingGeometry,
  ObjectOutsideStock,
  LaneUnavailable,
  LaneStale,
  LaneEvidenceInvalidated,
  PerceptionUnlive,
  IncompatiblePair,
  // The trusted contents ledger names products the lane's current policy does not accept —
  // typically after a runtime policy change left the old column in place. The lane is skipped
  // and the conflict is reported rather than acted on; a gravity-fed shelf cannot be emptied
  // from behind.
  LaneWrongProduct,
  InsufficientLaneDepth,
  NoEligiblePair,
};

struct SelectionError
{
  SelectionErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] SelectionResult
{
public:
  [[nodiscard]] static SelectionResult success(T value)
  {
    return SelectionResult(std::move(value));
  }

  [[nodiscard]] static SelectionResult failure(SelectionError error)
  {
    return SelectionResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const SelectionError & error() const {return std::get<SelectionError>(storage_);}

private:
  explicit SelectionResult(T value)
  : storage_(std::move(value)) {}

  explicit SelectionResult(SelectionError error)
  : storage_(std::move(error)) {}

  std::variant<T, SelectionError> storage_;
};

struct SelectionRequest
{
  std::optional<restocker_world_state::ObjectId> object_id;
  std::optional<restocker_world_state::LaneId> lane_id;
};

// One lane's stocking shortfall against its owner-declared want (Milestone 10 §4):
//   held    = round((depth_m - available_depth_m) / pitch(expected product))
//   deficit = max(0, target_count - held)
// Pitch is the catalogued diameter foreshortened by the lane's incline, derived from the product
// collision catalog and the surveyed lane — never a tuned constant. Held count and deficit are
// computed together because neither is meaningful without the other.
struct LaneDeficit
{
  std::uint32_t held_count{0};
  std::uint32_t deficit{0};
  double column_length_m{0.0};
  double pitch_m{0.0};
};

// Fail-closed: a lane whose expected product has no catalogued upright-cylinder geometry, or a
// lane geometry that cannot yield a positive finite pitch, has no computable deficit. The caller
// refuses the lane rather than substituting a guess.
[[nodiscard]] SelectionResult<LaneDeficit> lane_deficit(
  const restocker_world_state::ShelfLane & lane, const LaneManipulationGeometry & lane_geometry,
  const ProductCollisionCatalog & product_catalog);

struct SelectionConfig
{
  rclcpp::Time now{std::int64_t{0}, RCL_ROS_TIME};
  std::chrono::nanoseconds maximum_object_age{std::chrono::milliseconds(500)};
  // How long a surveyed lane's depletion evidence may be selected against without a re-survey,
  // unless it has been explicitly invalidated. Replaces the former 500 ms maximum_lane_age, which
  // a single wrist camera cannot satisfy across six stations.
  std::chrono::nanoseconds lane_evidence_validity{std::chrono::seconds(60)};
  std::chrono::nanoseconds maximum_robot_age{std::chrono::milliseconds(500)};
  std::chrono::nanoseconds maximum_future_skew{std::chrono::milliseconds(50)};
  // Stream-level perception liveness. Selection refuses when the last acquisition on the lane
  // producer's sensor is older than this, independently of how old any particular lane fact is.
  std::chrono::nanoseconds perception_liveness_max_age{std::chrono::milliseconds(500)};
  std::optional<rclcpp::Time> last_perception_acquisition;
  double stock_containment_margin_m{0.0};
  // Outward seating tolerance on the surveyed stock usable volume. This covers contact
  // penetration at a tray wall; unlike stock_containment_margin_m it never insets the region.
  double stock_boundary_tolerance_m{0.002};
  double maximum_upright_tilt_rad{0.05};
  // Weights on the two legs of the pair cost below. They are separate so a deployment can prefer
  // fetching the nearest product over the shortest delivery, or the reverse, without introducing a
  // second scoring concept; both default to 1.0, which simply minimises total carriage travel.
  double approach_rail_travel_weight{1.0};
  double delivery_rail_travel_weight{1.0};
  // Destination-side product-envelope margin used again by placement generation.
  double destination_containment_margin_m{0.005};
  // Shortfall permitted against the preferred entry clearance before adapting the release pose.
  double destination_entry_depth_tolerance_m{0.010};
};

// What an eligible pair costs to carry out, measured in rail travel. The carriage is the only
// degree of freedom that moves along the shelf: the arm's reach into a lane is the same for every
// lane and the same into every part of the tray, so what actually separates one eligible pair from
// another is how far the carriage has to travel to reach the product and then its destination.
// Shaped like GraspScore, weighted components plus the total they sum to, so the two scores in
// this package are read the same way.
struct SelectionScore
{
  // Carriage travel from where it stands now to the product this pair would pick.
  double approach_rail_travel_m{0.0};
  // Carriage travel onward from that product to the lane this pair would place it in.
  double delivery_rail_travel_m{0.0};
  double total{0.0};
};

struct SelectedTaskPair
{
  restocker_world_state::ObjectId object_id;
  restocker_world_state::LaneId lane_id;
  restocker_world_state::Revision snapshot_revision{0};
  restocker_world_state::Revision object_revision{0};
  restocker_world_state::Revision lane_revision{0};
  CylinderEnvelope product_envelope;
  // Why this pair was chosen over the others that were also eligible. It is reported rather than
  // consumed: nothing downstream re-derives a decision from it, so a pair rebound to a newer
  // snapshot keeps the score of the snapshot it was selected against.
  SelectionScore score{};
  // The destination's deficit against its owner-declared target_count, from the catalogued-pitch
  // held count (Milestone 10 §4). Reported, and the primary ordering key: largest deficit first.
  // Zero is legal and sorts last — a lane at or above its target still admits a placement when
  // nothing hungrier is eligible; the mode loop, not selection, decides when zero means idle.
  std::uint32_t destination_deficit{0};
  // Rail (carriage-axis) distance from the arm's current position to the destination lane's
  // station. The tie-break behind equal deficits, and reported beside the score.
  double destination_rail_distance_m{0.0};
  // Reported copy of the destination's ledger identity trust. An unreliable ledger does not bar
  // the pair — geometry stands — but every consumer can see that contents are withdrawn.
  bool destination_ledger_unreliable{false};
};

[[nodiscard]] SelectionResult<SelectedTaskPair> select_task_pair(
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const ProductCollisionCatalog & product_catalog,
  const WorkcellManipulationGeometry & workcell_geometry,
  const Eigen::Isometry3d & world_from_shelf, const SelectionConfig & config,
  const SelectionRequest & request = {});

// What the coordinator's wrist-depth subscription itself observed, independent of the age
// check. `arrivals` counts every admitted callback; `stamp_advances` counts the subset that
// moved the retained stamp forward. The pair separates the three failure layers a single age
// number cannot: no arrivals means the consumer never ran, arrivals without stamp advances
// mean frames arrived with non-advancing stamps, and both advancing with a stale age means
// delivery carried frames that were already older than the horizon when they landed.
struct PerceptionLivenessObservation
{
  std::optional<rclcpp::Time> last_acquisition;
  // Node time at the callback that delivered `last_acquisition`, when that arrival advanced
  // the stamp. Absent until the first advancing arrival.
  std::optional<rclcpp::Time> last_received_at;
  std::uint64_t arrivals{0U};
  std::uint64_t stamp_advances{0U};
};

// Diagnostic text for the liveness predicate. Keeps the historic unlive phrasing so existing
// readers still match, and appends the observation numbers a retained log needs to separate
// producer, delivery and consumer on the next live probe. Pure: no clock, no lock, no I/O.
[[nodiscard]] std::string describe_perception_liveness(
  const PerceptionLivenessObservation & observation, const rclcpp::Time & now,
  std::chrono::nanoseconds max_age);

[[nodiscard]] std::string to_string(SelectionErrorCode code);

}  // namespace restocker_task_executor
