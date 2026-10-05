// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/grasp_candidates.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace restocker_task_executor
{
namespace
{

template<typename T>
[[nodiscard]] GraspCandidateResult<T> failure(
  GraspCandidateErrorCode code, std::string detail)
{
  return GraspCandidateResult<T>::failure(GraspCandidateError{code, std::move(detail)});
}

[[nodiscard]] bool finite_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] bool finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

[[nodiscard]] bool finite_candidate(const GraspCandidate & candidate)
{
  return std::isfinite(candidate.approach_yaw_rad) &&
         std::isfinite(candidate.open_joint_position_m) &&
         std::isfinite(candidate.hold_joint_position_m) &&
         finite_rigid_transform(candidate.poses.world_from_pregrasp_center) &&
         finite_rigid_transform(candidate.poses.world_from_grasp_center) &&
         finite_rigid_transform(candidate.poses.world_from_retract_center) &&
         finite_rigid_transform(candidate.poses.world_from_pregrasp_tool0) &&
         finite_rigid_transform(candidate.poses.world_from_grasp_tool0) &&
         finite_rigid_transform(candidate.poses.world_from_retract_tool0) &&
         std::isfinite(candidate.score.rail_travel) &&
         std::isfinite(candidate.score.rear_access_penalty) &&
         std::isfinite(candidate.score.wrist_clearance_penalty) &&
         std::isfinite(candidate.score.total);
}

// Every product class the collision catalog describes as an upright cylinder is admissible.
// Unknown is not: the envelope the jaws are sized from cannot be trusted.
[[nodiscard]] bool graspable_upright_cylinder_class(
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

[[nodiscard]] double normalized_yaw(double yaw)
{
  return std::atan2(std::sin(yaw), std::cos(yaw));
}

// A horizontal side grasp pins all three grasp-centre axes (see grasp_approach_axis): +z is the
// horizontal approach, +y the jaw closing axis (horizontal, perpendicular to the approach), and
// +x vertical. +x must point up: the wrist camera hangs 0.11 m along tool0 +x, so the opposite
// roll sweeps it through the stock tray and leaves no collision-free IK solution.
[[nodiscard]] Eigen::Isometry3d grasp_pose(
  const Eigen::Vector3d & center, double yaw)
{
  const Eigen::Vector3d approach(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = center;
  pose.linear().col(0) = Eigen::Vector3d::UnitZ();
  pose.linear().col(1) = approach.cross(Eigen::Vector3d::UnitZ());
  pose.linear().col(2) = approach;
  return pose;
}

// tool0 trails the grasp centre along the approach, so stand-off poses slide back along it.
[[nodiscard]] Eigen::Isometry3d offset_along_approach(
  const Eigen::Isometry3d & pose, double distance)
{
  Eigen::Isometry3d offset = pose;
  offset.translation() -= distance * grasp_approach_axis(pose);
  return offset;
}

[[nodiscard]] bool same_selection(
  const SelectedTaskPair & left, const SelectedTaskPair & right)
{
  return left.object_id == right.object_id && left.lane_id == right.lane_id &&
         left.snapshot_revision == right.snapshot_revision &&
         left.object_revision == right.object_revision &&
         left.lane_revision == right.lane_revision &&
         left.product_envelope.radius_m == right.product_envelope.radius_m &&
         left.product_envelope.height_m == right.product_envelope.height_m;
}

[[nodiscard]] bool same_config(
  const GraspGenerationConfig & left, const GraspGenerationConfig & right)
{
  return left.approach_yaws_rad == right.approach_yaws_rad &&
         left.minimum_center_height_above_base_m ==
         right.minimum_center_height_above_base_m &&
         left.maximum_top_above_center_m == right.maximum_top_above_center_m &&
         left.pregrasp_distance_m == right.pregrasp_distance_m &&
         left.retract_distance_m == right.retract_distance_m &&
         left.open_clearance_per_side_m == right.open_clearance_per_side_m &&
         left.hold_clearance_per_side_m == right.hold_clearance_per_side_m &&
         left.maximum_open_joint_position_m == right.maximum_open_joint_position_m &&
         left.current_rail_position_m == right.current_rail_position_m &&
         left.rail_travel_weight == right.rail_travel_weight &&
         left.rear_access_weight == right.rear_access_weight &&
         left.wrist_clearance_weight == right.wrist_clearance_weight &&
         left.maximum_upright_tilt_rad == right.maximum_upright_tilt_rad;
}

[[nodiscard]] bool same_candidate(
  const GraspCandidate & left, const GraspCandidate & right)
{
  constexpr double kTolerance = 1.0e-12;
  return left.source_yaw_index == right.source_yaw_index &&
         left.approach_yaw_rad == right.approach_yaw_rad &&
         left.open_joint_position_m == right.open_joint_position_m &&
         left.hold_joint_position_m == right.hold_joint_position_m &&
         left.poses.world_from_pregrasp_center.matrix().isApprox(
    right.poses.world_from_pregrasp_center.matrix(), kTolerance) &&
         left.poses.world_from_grasp_center.matrix().isApprox(
    right.poses.world_from_grasp_center.matrix(), kTolerance) &&
         left.poses.world_from_retract_center.matrix().isApprox(
    right.poses.world_from_retract_center.matrix(), kTolerance) &&
         left.poses.world_from_pregrasp_tool0.matrix().isApprox(
    right.poses.world_from_pregrasp_tool0.matrix(), kTolerance) &&
         left.poses.world_from_grasp_tool0.matrix().isApprox(
    right.poses.world_from_grasp_tool0.matrix(), kTolerance) &&
         left.poses.world_from_retract_tool0.matrix().isApprox(
    right.poses.world_from_retract_tool0.matrix(), kTolerance) &&
         left.score.rail_travel == right.score.rail_travel &&
         left.score.rear_access_penalty == right.score.rear_access_penalty &&
         left.score.wrist_clearance_penalty == right.score.wrist_clearance_penalty &&
         left.score.total == right.score.total &&
         same_selection(left.selection, right.selection);
}

[[nodiscard]] bool same_selection_intent(
  const SelectedTaskPair & left, const SelectedTaskPair & right)
{
  return left.object_id == right.object_id && left.lane_id == right.lane_id &&
         left.object_revision == right.object_revision &&
         left.lane_revision == right.lane_revision &&
         left.product_envelope.radius_m == right.product_envelope.radius_m &&
         left.product_envelope.height_m == right.product_envelope.height_m;
}

[[nodiscard]] bool same_transform_exact(
  const Eigen::Isometry3d & left, const Eigen::Isometry3d & right)
{
  return (left.matrix().array() == right.matrix().array()).all();
}

[[nodiscard]] bool same_candidate_intent(
  const GraspCandidate & left, const GraspCandidate & right)
{
  return left.source_yaw_index == right.source_yaw_index &&
         left.approach_yaw_rad == right.approach_yaw_rad &&
         left.open_joint_position_m == right.open_joint_position_m &&
         left.hold_joint_position_m == right.hold_joint_position_m &&
         same_transform_exact(
    left.poses.world_from_pregrasp_center, right.poses.world_from_pregrasp_center) &&
         same_transform_exact(
    left.poses.world_from_grasp_center, right.poses.world_from_grasp_center) &&
         same_transform_exact(
    left.poses.world_from_retract_center, right.poses.world_from_retract_center) &&
         same_transform_exact(
    left.poses.world_from_pregrasp_tool0, right.poses.world_from_pregrasp_tool0) &&
         same_transform_exact(
    left.poses.world_from_grasp_tool0, right.poses.world_from_grasp_tool0) &&
         same_transform_exact(
    left.poses.world_from_retract_tool0, right.poses.world_from_retract_tool0) &&
         left.score.rail_travel == right.score.rail_travel &&
         left.score.rear_access_penalty == right.score.rear_access_penalty &&
         left.score.wrist_clearance_penalty == right.score.wrist_clearance_penalty &&
         left.score.total == right.score.total &&
         same_selection_intent(left.selection, right.selection);
}

[[nodiscard]] bool same_reservation(
  const restocker_world_state::TaskReservation & left,
  const restocker_world_state::TaskReservation & right)
{
  return left.reservation_id == right.reservation_id &&
         left.request_id == right.request_id && left.object_id == right.object_id &&
         left.object_source_id == right.object_source_id &&
         left.product_class == right.product_class && left.sku == right.sku &&
         left.source_lane == right.source_lane &&
         left.destination_lane == right.destination_lane && left.stage == right.stage &&
         left.placed_in_destination == right.placed_in_destination &&
         left.created_at == right.created_at &&
         left.created_revision == right.created_revision &&
         left.admitted_robot_telemetry_revision == right.admitted_robot_telemetry_revision &&
         left.revision == right.revision &&
         left.destination_expected_product_class == right.destination_expected_product_class &&
         left.destination_expected_sku == right.destination_expected_sku;
}

[[nodiscard]] std::optional<std::string> freshness_error(
  const rclcpp::Time & evidence_time, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_age, std::chrono::nanoseconds maximum_future_skew,
  const std::string & subject)
{
  if (maximum_age.count() < 0 || maximum_future_skew.count() < 0 ||
    evidence_time.nanoseconds() <= 0 || now.nanoseconds() <= 0 ||
    evidence_time.get_clock_type() != now.get_clock_type())
  {
    return subject + " freshness configuration or clock is invalid";
  }
  if (now.nanoseconds() >= evidence_time.nanoseconds()) {
    if (now.nanoseconds() - evidence_time.nanoseconds() > maximum_age.count()) {
      return subject + " is stale at grasp generation";
    }
  } else {
    if (evidence_time.nanoseconds() - now.nanoseconds() > maximum_future_skew.count()) {
      return subject + " is too far in the future at grasp generation";
    }
  }
  return std::nullopt;
}

}  // namespace

GraspCandidateResult<std::vector<GraspCandidate>> generate_upright_cylinder_grasps(
  const restocker_world_state::TrackedObject & object, const SelectedTaskPair & selection,
  const ParallelJawGeometry & gripper, const Eigen::Isometry3d & tool0_from_grasp_center,
  const GraspGenerationConfig & config)
{
  using restocker_world_state::ObjectOrientation;

  constexpr std::size_t kMaximumYawCount = 16;
  if (config.approach_yaws_rad.empty() ||
    config.approach_yaws_rad.size() > kMaximumYawCount ||
    !finite_nonnegative(config.minimum_center_height_above_base_m) ||
    !std::isfinite(config.maximum_top_above_center_m) ||
    config.maximum_top_above_center_m <= 0.0 ||
    !std::isfinite(config.pregrasp_distance_m) || config.pregrasp_distance_m <= 0.0 ||
    !std::isfinite(config.retract_distance_m) || config.retract_distance_m <= 0.0 ||
    !finite_nonnegative(config.open_clearance_per_side_m) ||
    !finite_nonnegative(config.hold_clearance_per_side_m) ||
    config.open_clearance_per_side_m <= config.hold_clearance_per_side_m ||
    !std::isfinite(config.maximum_open_joint_position_m) ||
    config.maximum_open_joint_position_m < gripper.joint_lower_m ||
    config.maximum_open_joint_position_m > gripper.joint_upper_m ||
    !std::isfinite(config.current_rail_position_m) ||
    !finite_nonnegative(config.rail_travel_weight) ||
    !finite_nonnegative(config.rear_access_weight) ||
    !finite_nonnegative(config.wrist_clearance_weight) ||
    !finite_nonnegative(config.maximum_upright_tilt_rad) ||
    config.maximum_upright_tilt_rad >= 0.5 * std::acos(-1.0) ||
    !std::isfinite(gripper.inner_gap_at_zero_m) || gripper.inner_gap_at_zero_m <= 0.0 ||
    !finite_nonnegative(gripper.joint_lower_m) ||
    !std::isfinite(gripper.joint_upper_m) ||
    gripper.joint_upper_m <= gripper.joint_lower_m ||
    !std::isfinite(selection.product_envelope.radius_m) ||
    selection.product_envelope.radius_m <= 0.0 ||
    !std::isfinite(selection.product_envelope.height_m) ||
    selection.product_envelope.height_m <= 0.0)
  {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidConfiguration,
      "grasp generation configuration or geometry is invalid");
  }
  if (!selection.object_id || selection.lane_id.value.empty() ||
    selection.snapshot_revision == 0 || selection.object_revision == 0 ||
    selection.lane_revision == 0 || object.id != selection.object_id ||
    object.revision != selection.object_revision ||
    !graspable_upright_cylinder_class(object.product_class) ||
    object.orientation != ObjectOrientation::Upright ||
    !finite_rigid_transform(object.pose_in_world))
  {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidObject,
      "selected object identity, revision, class, orientation, or pose is invalid");
  }
  if (!finite_rigid_transform(tool0_from_grasp_center)) {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidToolTransform,
      "tool0 to grasp-center transform must be finite and rigid");
  }
  // Grasp at mid-height, raised only as needed: the wrist must clear the support surface and the
  // product must stay below the wrist camera. Derived from the envelope so one pair of clearances
  // serves all product heights.
  const double half_height = 0.5 * selection.product_envelope.height_m;
  const double axial_offset_m = std::max(
    {0.0, config.minimum_center_height_above_base_m - half_height,
      half_height - config.maximum_top_above_center_m});
  if (axial_offset_m > half_height) {
    // Only the support clearance can ask for this; the camera bound never exceeds mid-height.
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidConfiguration,
      "product is shorter than the minimum grasp-centre height above its base");
  }
  const double upright_alignment = std::clamp(
    object.pose_in_world.linear().col(2).dot(Eigen::Vector3d::UnitZ()), -1.0, 1.0);
  if (std::acos(upright_alignment) > config.maximum_upright_tilt_rad) {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidObject,
      "object cylinder axis exceeds the upright grasp tolerance");
  }

  const double diameter = 2.0 * selection.product_envelope.radius_m;
  const double open_gap = diameter + 2.0 * config.open_clearance_per_side_m;
  const double hold_gap = diameter + 2.0 * config.hold_clearance_per_side_m;
  const double open_joint = 0.5 * (open_gap - gripper.inner_gap_at_zero_m);
  const double hold_joint = 0.5 * (hold_gap - gripper.inner_gap_at_zero_m);
  if (!std::isfinite(diameter) || !std::isfinite(open_gap) || !std::isfinite(hold_gap) ||
    !std::isfinite(open_joint) || !std::isfinite(hold_joint))
  {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::InvalidConfiguration,
      "grasp jaw arithmetic produced a non-finite target");
  }
  if (open_joint < gripper.joint_lower_m || open_joint > gripper.joint_upper_m ||
    open_joint > config.maximum_open_joint_position_m ||
    hold_joint < gripper.joint_lower_m || hold_joint > gripper.joint_upper_m)
  {
    return failure<std::vector<GraspCandidate>>(
      GraspCandidateErrorCode::NoValidJawTarget,
      "object-specific open or hold target is outside symmetric jaw limits");
  }

  std::vector<double> unique_yaws;
  std::vector<GraspCandidate> candidates;
  candidates.reserve(config.approach_yaws_rad.size());
  const Eigen::Isometry3d grasp_center_from_tool0 = tool0_from_grasp_center.inverse();
  const Eigen::Vector3d center = object.pose_in_world.translation() +
    axial_offset_m * object.pose_in_world.linear().col(2);
  // The wrist trails tool0 backwards along the approach, so a grasp is reachable only when the
  // approach points away from the carriage. The carriage can translate along world X in the
  // pre-grasp plan, so reach is measured from the closest rail point (at the product's X), not
  // from the current rail position, which stays in the separate travel-cost term.
  // A product on the carriage axis has zero reach: every approach scores the same (a preference,
  // not a validity gate) and the yaw tie-break still orders candidates.
  Eigen::Vector3d reach = object.pose_in_world.translation() -
    Eigen::Vector3d(object.pose_in_world.translation().x(), 0.0, 0.0);
  reach.z() = 0.0;
  const double reach_distance = reach.norm();
  const Eigen::Vector3d reach_direction = reach_distance > 1.0e-9 ?
    Eigen::Vector3d(reach / reach_distance) : Eigen::Vector3d::Zero();
  for (std::size_t index = 0; index < config.approach_yaws_rad.size(); ++index) {
    const double input_yaw = config.approach_yaws_rad[index];
    if (!std::isfinite(input_yaw)) {
      return failure<std::vector<GraspCandidate>>(
        GraspCandidateErrorCode::InvalidConfiguration,
        "approach yaw set contains a non-finite value");
    }
    const double yaw = normalized_yaw(input_yaw);
    const bool duplicate = std::any_of(
      unique_yaws.begin(), unique_yaws.end(),
      [yaw](double existing) {
        return std::abs(std::remainder(yaw - existing, 2.0 * std::acos(-1.0))) <= 1.0e-12;
      });
    if (duplicate) {
      return failure<std::vector<GraspCandidate>>(
        GraspCandidateErrorCode::InvalidConfiguration,
        "approach yaw set contains equivalent duplicate values");
    }
    unique_yaws.push_back(yaw);

    const Eigen::Isometry3d grasp = grasp_pose(center, yaw);
    const Eigen::Isometry3d pregrasp = offset_along_approach(
      grasp, config.pregrasp_distance_m);
    const Eigen::Isometry3d retract = offset_along_approach(
      grasp, config.retract_distance_m);
    const Eigen::Vector3d approach = grasp_approach_axis(grasp);
    const Eigen::Vector3d closing = grasp_closing_axis(grasp);
    GraspScore score;
    score.rail_travel = std::abs(
      object.pose_in_world.translation().x() - config.current_rail_position_m);
    // 0 when reaching straight out from the carriage, 1 when pointing back at the arm.
    score.rear_access_penalty = 0.5 * (1.0 - approach.dot(reach_direction));
    // Jaws should straddle the reach direction, not lie along it, so the near finger avoids
    // the space the far finger and wrist occupy.
    score.wrist_clearance_penalty = std::abs(closing.dot(reach_direction));
    score.total = config.rail_travel_weight * score.rail_travel +
      config.rear_access_weight * score.rear_access_penalty +
      config.wrist_clearance_weight * score.wrist_clearance_penalty;

    GraspCandidate candidate{
      index, yaw, open_joint, hold_joint,
      GraspPoseSequence{
        pregrasp, grasp, retract, pregrasp * grasp_center_from_tool0,
        grasp * grasp_center_from_tool0, retract * grasp_center_from_tool0},
      score, selection};
    if (!finite_candidate(candidate)) {
      return failure<std::vector<GraspCandidate>>(
        GraspCandidateErrorCode::InvalidConfiguration,
        "grasp generation produced a non-finite pose, jaw target, or score");
    }
    candidates.push_back(std::move(candidate));
  }
  std::sort(
    candidates.begin(), candidates.end(),
    [](const GraspCandidate & left, const GraspCandidate & right) {
      if (left.score.total != right.score.total) {
        return left.score.total < right.score.total;
      }
      if (left.approach_yaw_rad != right.approach_yaw_rad) {
        return left.approach_yaw_rad < right.approach_yaw_rad;
      }
      return left.source_yaw_index < right.source_yaw_index;
    });
  return GraspCandidateResult<std::vector<GraspCandidate>>::success(std::move(candidates));
}

GraspCandidateResult<GraspCandidateBatch> generate_grasp_candidate_batch(
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const SelectedTaskPair & selection, const ParallelJawGeometry & gripper,
  const Eigen::Isometry3d & tool0_from_grasp_center, const GraspGenerationConfig & config,
  const rclcpp::Time & now, std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew)
{
  if (!std::isfinite(selection.product_envelope.radius_m) ||
    !std::isfinite(selection.product_envelope.height_m))
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "selection product envelope is not finite");
  }
  if (snapshot.revision != selection.snapshot_revision) {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "selection snapshot revision does not match the retained snapshot");
  }
  if (snapshot.robot.telemetry_revision == 0 ||
    snapshot.robot.telemetry_revision > snapshot.robot.revision ||
    !std::isfinite(snapshot.robot.rail_position))
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "snapshot, selection, and robot rail lineage are inconsistent");
  }
  if (snapshot.robot.rail_position != config.current_rail_position_m) {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "snapshot, selection, and robot rail lineage are inconsistent");
  }
  const auto object = snapshot.objects.find(selection.object_id);
  if (object == snapshot.objects.end() || object->second.revision != selection.object_revision) {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "selected object is absent or has a different revision");
  }
  if (const auto stale = freshness_error(
      object->second.observation_time, now, maximum_object_age, maximum_future_skew, "object"))
  {
    return failure<GraspCandidateBatch>(GraspCandidateErrorCode::StaleEvidence, *stale);
  }
  if (const auto stale = freshness_error(
      snapshot.robot.telemetry_time, now, maximum_robot_age, maximum_future_skew,
      "robot telemetry"))
  {
    return failure<GraspCandidateBatch>(GraspCandidateErrorCode::StaleEvidence, *stale);
  }
  auto candidates = generate_upright_cylinder_grasps(
    object->second, selection, gripper, tool0_from_grasp_center, config);
  if (!candidates) {
    return failure<GraspCandidateBatch>(candidates.error().code, candidates.error().detail);
  }
  return GraspCandidateResult<GraspCandidateBatch>::success(
    GraspCandidateBatch{
      std::move(candidates.value()), selection, snapshot.revision,
      object->second.observation_time, snapshot.robot.telemetry_revision,
      snapshot.robot.telemetry_time, gripper, tool0_from_grasp_center, config});
}

GraspCandidateResult<bool> validate_grasp_candidate_batch(
  const GraspCandidateBatch & batch,
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const SelectedTaskPair & selection, const GraspGenerationAuthority & authority,
  const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew)
{
  auto expected = generate_grasp_candidate_batch(
    snapshot, selection, authority.gripper, authority.tool0_from_grasp_center,
    authority.config, now,
    maximum_object_age, maximum_robot_age, maximum_future_skew);
  if (!expected) {
    return failure<bool>(expected.error().code, expected.error().detail);
  }
  if (!same_selection(batch.selection, selection) ||
    batch.source_world_revision != expected.value().source_world_revision ||
    batch.source_object_observation_time != expected.value().source_object_observation_time ||
    batch.source_robot_telemetry_revision !=
    expected.value().source_robot_telemetry_revision ||
    batch.source_robot_telemetry_time != expected.value().source_robot_telemetry_time ||
    !finite_rigid_transform(batch.tool0_from_grasp_center) ||
    batch.gripper.inner_gap_at_zero_m != expected.value().gripper.inner_gap_at_zero_m ||
    batch.gripper.joint_lower_m != expected.value().gripper.joint_lower_m ||
    batch.gripper.joint_upper_m != expected.value().gripper.joint_upper_m ||
    !batch.tool0_from_grasp_center.matrix().isApprox(
      expected.value().tool0_from_grasp_center.matrix(), 1.0e-12) ||
    !same_config(batch.config, expected.value().config) ||
    batch.candidates.size() != expected.value().candidates.size() ||
    batch.candidates.size() > batch.config.approach_yaws_rad.size() ||
    batch.candidates.size() > 16U)
  {
    return failure<bool>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "grasp candidate batch metadata or bounds contradict deterministic inputs");
  }
  for (std::size_t index = 0; index < batch.candidates.size(); ++index) {
    if (!finite_candidate(batch.candidates[index]) ||
      !same_candidate(batch.candidates[index], expected.value().candidates[index]))
    {
      return failure<bool>(
        GraspCandidateErrorCode::InvalidCandidateBatch,
        "grasp candidate batch differs from deterministic regeneration");
    }
  }
  return GraspCandidateResult<bool>::success(true);
}

GraspCandidateResult<GraspCandidateBatch> refresh_pregrasp_candidate_batch(
  const GraspCandidateBatch & staged_batch,
  const restocker_world_state::WorldStateSnapshot & fresh_snapshot,
  const restocker_world_state::TaskReservation & expected_reservation,
  const GraspGenerationAuthority & authority,
  const rclcpp::Time & now, std::chrono::nanoseconds maximum_object_age,
  std::chrono::nanoseconds maximum_robot_age,
  std::chrono::nanoseconds maximum_future_skew)
{
  if (staged_batch.candidates.empty() || staged_batch.source_world_revision == 0U ||
    staged_batch.selection.snapshot_revision != staged_batch.source_world_revision ||
    !same_selection(staged_batch.candidates.front().selection, staged_batch.selection) ||
    staged_batch.gripper.inner_gap_at_zero_m != authority.gripper.inner_gap_at_zero_m ||
    staged_batch.gripper.joint_lower_m != authority.gripper.joint_lower_m ||
    staged_batch.gripper.joint_upper_m != authority.gripper.joint_upper_m ||
    !same_transform_exact(
      staged_batch.tool0_from_grasp_center, authority.tool0_from_grasp_center) ||
    !same_config(staged_batch.config, authority.config) ||
    fresh_snapshot.revision <= staged_batch.source_world_revision ||
    fresh_snapshot.robot.telemetry_revision <
    staged_batch.source_robot_telemetry_revision ||
    fresh_snapshot.robot.telemetry_time.get_clock_type() !=
    staged_batch.source_robot_telemetry_time.get_clock_type() ||
    fresh_snapshot.robot.telemetry_time < staged_batch.source_robot_telemetry_time ||
    expected_reservation.reservation_id == 0U || expected_reservation.request_id.empty() ||
    expected_reservation.stage != restocker_world_state::ReservationStage::Reserved ||
    expected_reservation.placed_in_destination ||
    expected_reservation.created_revision == 0U ||
    expected_reservation.created_revision <= staged_batch.source_world_revision ||
    expected_reservation.created_revision > expected_reservation.revision ||
    expected_reservation.revision <= staged_batch.source_world_revision ||
    expected_reservation.revision > fresh_snapshot.revision ||
    expected_reservation.admitted_robot_telemetry_revision == 0U ||
    expected_reservation.admitted_robot_telemetry_revision >
    fresh_snapshot.robot.telemetry_revision)
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "staged batch, fresh snapshot, or expected reservation has invalid revision lineage");
  }
  if (!fresh_snapshot.active_reservation ||
    !same_reservation(*fresh_snapshot.active_reservation, expected_reservation) ||
    expected_reservation.object_id != staged_batch.selection.object_id ||
    expected_reservation.destination_lane != staged_batch.selection.lane_id)
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::ReservationMismatch,
      "fresh snapshot does not contain the exact expected task reservation");
  }

  const auto object = fresh_snapshot.objects.find(staged_batch.selection.object_id);
  const auto lane = fresh_snapshot.lanes.find(staged_batch.selection.lane_id);
  if (object == fresh_snapshot.objects.end() || lane == fresh_snapshot.lanes.end() ||
    object->second.revision != staged_batch.selection.object_revision ||
    lane->second.revision != staged_batch.selection.lane_revision ||
    object->second.observation_time != staged_batch.source_object_observation_time)
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::SelectionChanged,
      "selected object or lane is absent or changed after grasp staging");
  }
  if (object->second.id != staged_batch.selection.object_id ||
    lane->second.id != staged_batch.selection.lane_id ||
    object->second.source_object_id != expected_reservation.object_source_id ||
    object->second.product_class != expected_reservation.product_class ||
    object->second.sku != expected_reservation.sku ||
    object->second.tracking_state != restocker_world_state::TrackingState::Tracked ||
    object->second.grasp_state != restocker_world_state::GraspState::Free ||
    object->second.orientation != restocker_world_state::ObjectOrientation::Upright ||
    fresh_snapshot.robot.held_object)
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::InvalidCandidateBatch,
      "unchanged entity revision contradicts reservation identity or grasp lifecycle");
  }

  SelectedTaskPair rebound_selection = staged_batch.selection;
  rebound_selection.snapshot_revision = fresh_snapshot.revision;
  auto refreshed = generate_grasp_candidate_batch(
    fresh_snapshot, rebound_selection, authority.gripper,
    authority.tool0_from_grasp_center, authority.config, now,
    maximum_object_age, maximum_robot_age, maximum_future_skew);
  if (!refreshed) {
    return refreshed;
  }
  if (refreshed.value().candidates.empty() ||
    !same_candidate_intent(
      staged_batch.candidates.front(), refreshed.value().candidates.front()))
  {
    return failure<GraspCandidateBatch>(
      GraspCandidateErrorCode::CandidateIntentChanged,
      "candidate zero changed while rebinding to the post-reservation snapshot");
  }
  return refreshed;
}

}  // namespace restocker_task_executor
