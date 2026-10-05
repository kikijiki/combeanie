// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "restocker_task_executor/task_selection.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

enum class PlacementCandidateErrorCode : std::uint8_t
{
  InvalidConfiguration,
  InvalidSelection,
  InvalidObject,
  InvalidLane,
  InvalidGeometry,
  InvalidToolTransform,
  InvalidAttachmentTransform,
  InsufficientLaneDepth,
  PlacementOutsideLane,
};

struct PlacementCandidateError
{
  PlacementCandidateErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] PlacementCandidateResult
{
public:
  [[nodiscard]] static PlacementCandidateResult success(T value)
  {
    return PlacementCandidateResult(std::move(value));
  }

  [[nodiscard]] static PlacementCandidateResult failure(PlacementCandidateError error)
  {
    return PlacementCandidateResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const PlacementCandidateError & error() const
  {
    return std::get<PlacementCandidateError>(storage_);
  }

private:
  explicit PlacementCandidateResult(T value)
  : storage_(std::move(value)) {}

  explicit PlacementCandidateResult(PlacementCandidateError error)
  : storage_(std::move(error)) {}

  std::variant<T, PlacementCandidateError> storage_;
};

struct PlacementGenerationConfig
{
  double preinsertion_distance_m{0.0};
  double retreat_distance_m{0.0};
  double containment_margin_m{0.0};
  // How far above the roller bed's rear lip the product is carried and released. It is not a
  // placement margin, the product is meant to end up on the bed and does, by falling when the
  // jaws open, it is the clearance the *planner* needs. A product planned to arrive with its
  // base exactly on the bed is in contact with it for the whole insertion, and whether the
  // collision checker calls that a collision is decided by rounding, so the straight-line insert
  // failed part way on some runs and completed on others. Kept separate from the containment
  // margin because those are answers to different questions, and coupling the seat height to the
  // containment margin is a defect this generator has already had once.
  //
  // The rear lip is the reference rather than the bed under the release point because the insert
  // is a straight horizontal push: the product's uphill rim passes over every part of the bed
  // between the lane mouth and the insert depth, and the rear lip is the highest of those.
  double insertion_floor_clearance_m{0.0};
  double maximum_axis_alignment_error_rad{0.05};
};

struct PlacementPoseSequence
{
  Eigen::Isometry3d world_from_preinsertion_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_final_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_retreat_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_preinsertion_tool0{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_final_tool0{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_retreat_tool0{Eigen::Isometry3d::Identity()};
};

struct PlacementCandidate
{
  SelectedTaskPair selection;
  restocker_world_state::Revision source_lane_revision{0};
  Eigen::Isometry3d lane_from_product{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_product{Eigen::Isometry3d::Identity()};
  PlacementPoseSequence poses;
  double required_lane_depth_m{0.0};
};

// The destination policy captured at reservation grant must be passed in, never read from the
// live lane: SetLanePolicy may repark the lane while the product is in the jaws, and a granted
// transfer completes under the policy it was granted under (the same rule the store's
// reservation predicate and the port's execution proof enforce).
[[nodiscard]] PlacementCandidateResult<std::vector<PlacementCandidate>>
generate_upright_cylinder_placements(
  const restocker_world_state::TrackedObject & object,
  const restocker_world_state::ShelfLane & lane, const SelectedTaskPair & selection,
  const LaneManipulationGeometry & lane_geometry,
  const Eigen::Isometry3d & world_from_lane,
  const Eigen::Isometry3d & product_from_grasp_center,
  const Eigen::Isometry3d & tool0_from_grasp_center,
  const PlacementGenerationConfig & config,
  restocker_world_state::ProductClass destination_expected_product_class,
  const std::optional<std::string> & destination_expected_sku);

}  // namespace restocker_task_executor
