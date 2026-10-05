// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <rclcpp/time.hpp>

#include "restocker_task_executor/task_selection.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

enum class GraspCandidateErrorCode : std::uint8_t
{
  InvalidConfiguration,
  InvalidObject,
  InvalidToolTransform,
  NoValidJawTarget,
  StaleEvidence,
  InvalidCandidateBatch,
  ReservationMismatch,
  SelectionChanged,
  CandidateIntentChanged,
};

struct GraspCandidateError
{
  GraspCandidateErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] GraspCandidateResult
{
public:
  [[nodiscard]] static GraspCandidateResult success(T value)
  {
    return GraspCandidateResult(std::move(value));
  }

  [[nodiscard]] static GraspCandidateResult failure(GraspCandidateError error)
  {
    return GraspCandidateResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const GraspCandidateError & error() const
  {
    return std::get<GraspCandidateError>(storage_);
  }

private:
  explicit GraspCandidateResult(T value)
  : storage_(std::move(value)) {}

  explicit GraspCandidateResult(GraspCandidateError error)
  : storage_(std::move(error)) {}

  std::variant<T, GraspCandidateError> storage_;
};

struct ParallelJawGeometry
{
  double inner_gap_at_zero_m{0.0};
  double joint_lower_m{0.0};
  double joint_upper_m{0.0};
};

struct GraspGenerationConfig
{
  std::vector<double> approach_yaws_rad;
  // Lowest the grasp centre may sit above the product's own base, measured along its cylinder
  // axis. A product is only ever grasped standing on a support, the stock tray when picked, the
  // lane floor when placed, so this is equally its clearance above that support, which is what
  // the wrist geometry actually constrains. The per-product axial offset is derived from it and
  // the product's catalogued height rather than configured beside it, because an offset and a
  // height that are maintained separately can disagree about the same product.
  double minimum_center_height_above_base_m{0.0};
  // Furthest a product may rise above the grasp centre along its own axis. The wrist camera is a
  // body on the gripper, not a sensor pose: it hangs above the grasp centre and reaches back
  // toward it, so a product tall enough to stand up past the grasp centre grows into it. This is
  // a second lower bound on the same axial offset, grasp a tall product higher up its side,
  // and it is expressed as a height rather than an offset for the same reason as the one above.
  double maximum_top_above_center_m{0.0};
  double pregrasp_distance_m{0.0};
  double retract_distance_m{0.0};
  double open_clearance_per_side_m{0.0};
  double hold_clearance_per_side_m{0.0};
  double maximum_open_joint_position_m{0.0};
  double current_rail_position_m{0.0};
  double rail_travel_weight{1.0};
  double rear_access_weight{1.0};
  double wrist_clearance_weight{1.0};
  double maximum_upright_tilt_rad{0.05};
};

// Inputs owned by the coordinator rather than by a candidate-producing backend. A returned batch
// is valid only when it exactly reproduces candidates from this independently retained authority.
struct GraspGenerationAuthority
{
  ParallelJawGeometry gripper;
  Eigen::Isometry3d tool0_from_grasp_center{Eigen::Isometry3d::Identity()};
  GraspGenerationConfig config;
};

struct GraspPoseSequence
{
  Eigen::Isometry3d world_from_pregrasp_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_grasp_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_retract_center{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_pregrasp_tool0{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_grasp_tool0{Eigen::Isometry3d::Identity()};
  Eigen::Isometry3d world_from_retract_tool0{Eigen::Isometry3d::Identity()};
};

struct GraspScore
{
  double rail_travel{0.0};
  double rear_access_penalty{0.0};
  double wrist_clearance_penalty{0.0};
  double total{0.0};
};

struct GraspCandidate
{
  std::size_t source_yaw_index{0};
  double approach_yaw_rad{0.0};
  double open_joint_position_m{0.0};
  double hold_joint_position_m{0.0};
  GraspPoseSequence poses;
  GraspScore score;
  SelectedTaskPair selection;
};

struct GraspCandidateBatch
{
  std::vector<GraspCandidate> candidates;
  SelectedTaskPair selection;
  restocker_world_state::Revision source_world_revision{0};
  rclcpp::Time source_object_observation_time{std::int64_t{0}, RCL_ROS_TIME};
  restocker_world_state::Revision source_robot_telemetry_revision{0};
  rclcpp::Time source_robot_telemetry_time{std::int64_t{0}, RCL_ROS_TIME};
  ParallelJawGeometry gripper;
  Eigen::Isometry3d tool0_from_grasp_center{Eigen::Isometry3d::Identity()};
  GraspGenerationConfig config;
};

[[nodiscard]] GraspCandidateResult<std::vector<GraspCandidate>>
generate_upright_cylinder_grasps(
  const restocker_world_state::TrackedObject & object, const SelectedTaskPair & selection,
  const ParallelJawGeometry & gripper, const Eigen::Isometry3d & tool0_from_grasp_center,
  const GraspGenerationConfig & config);

[[nodiscard]] GraspCandidateResult<GraspCandidateBatch> generate_grasp_candidate_batch(
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const SelectedTaskPair & selection, const ParallelJawGeometry & gripper,
  const Eigen::Isometry3d & tool0_from_grasp_center, const GraspGenerationConfig & config,
  const rclcpp::Time & now, std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew);

// Validates lineage and recomputes the complete vector. This prevents a backend adapter from
// presenting plausible-looking but non-deterministic geometry as an authoritative batch.
[[nodiscard]] GraspCandidateResult<bool> validate_grasp_candidate_batch(
  const GraspCandidateBatch & batch,
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const SelectedTaskPair & selection, const GraspGenerationAuthority & authority,
  const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew);

// Rebinds immutable staged intent to a newer post-reservation snapshot. The staged batch is never
// modified, and its metadata must match the separately retained generation authority.
[[nodiscard]] GraspCandidateResult<GraspCandidateBatch> refresh_pregrasp_candidate_batch(
  const GraspCandidateBatch & staged_batch,
  const restocker_world_state::WorldStateSnapshot & fresh_snapshot,
  const restocker_world_state::TaskReservation & expected_reservation,
  const GraspGenerationAuthority & authority,
  const rclcpp::Time & now, std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew);

}  // namespace restocker_task_executor
