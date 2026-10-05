// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/gripper_geometry.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace restocker_task_executor
{
namespace
{

constexpr double kGeometryTolerance = 1.0e-12;

template<typename T>
[[nodiscard]] GripperGeometryResult<T> failure(std::string detail)
{
  return GripperGeometryResult<T>::failure(
    GripperGeometryError{GripperGeometryErrorCode::kInvalidConfiguration, std::move(detail)});
}

[[nodiscard]] double finite_value(const YAML::Node & node, const std::string & field)
{
  const double value = node.as<double>();
  if (!std::isfinite(value)) {
    throw std::invalid_argument(field + " must be finite");
  }
  return value;
}

[[nodiscard]] double nonnegative(const YAML::Node & node, const std::string & field)
{
  const double value = finite_value(node, field);
  if (value < 0.0) {
    throw std::invalid_argument(field + " must be non-negative");
  }
  return value;
}

[[nodiscard]] Eigen::Vector3d vector3(const YAML::Node & node, const std::string & field)
{
  if (!node.IsSequence() || node.size() != 3U) {
    throw std::invalid_argument(field + " must contain exactly three values");
  }
  Eigen::Vector3d value(
    finite_value(node[0], field), finite_value(node[1], field),
    finite_value(node[2], field));
  return value;
}

[[nodiscard]] bool approximately_equal(double left, double right)
{
  return std::abs(left - right) <= kGeometryTolerance;
}

}  // namespace

GripperGeometryResult<GripperStagingGeometry> load_gripper_staging_geometry(
  const std::filesystem::path & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root.IsMap() || !root["schema_version"] || root["schema_version"].as<int>() != 1) {
      return failure<GripperStagingGeometry>(
        "gripper geometry requires schema_version 1");
    }

    const YAML::Node grasp_center = root["grasp_center"];
    const Eigen::Vector3d translation = vector3(
      grasp_center["xyz_m"], "grasp_center.xyz_m");
    const Eigen::Vector3d rpy = vector3(
      grasp_center["rpy_rad"], "grasp_center.rpy_rad");
    Eigen::Isometry3d tool0_from_grasp_center = Eigen::Isometry3d::Identity();
    tool0_from_grasp_center.translation() = translation;
    tool0_from_grasp_center.linear() =
      (Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();

    const YAML::Node finger = root["finger"];
    const Eigen::Vector3d finger_size = vector3(finger["size_xyz_m"], "finger.size_xyz_m");
    if ((finger_size.array() <= 0.0).any()) {
      return failure<GripperStagingGeometry>("finger size must be positive");
    }
    const YAML::Node joint = finger["joint"];
    const double joint_lower = nonnegative(joint["lower_m"], "finger.joint.lower_m");
    const double joint_upper = finite_value(joint["upper_m"], "finger.joint.upper_m");
    if (joint_upper <= joint_lower) {
      return failure<GripperStagingGeometry>(
        "finger joint upper bound must exceed its lower bound");
    }

    const Eigen::Vector3d left_origin = vector3(
      finger["left"]["origin_xyz_m"], "finger.left.origin_xyz_m");
    const Eigen::Vector3d right_origin = vector3(
      finger["right"]["origin_xyz_m"], "finger.right.origin_xyz_m");
    const Eigen::Vector3d left_axis = vector3(
      finger["left"]["axis"], "finger.left.axis");
    const Eigen::Vector3d right_axis = vector3(
      finger["right"]["axis"], "finger.right.axis");
    if (!left_axis.isApprox(Eigen::Vector3d::UnitY(), kGeometryTolerance) ||
      !right_axis.isApprox(-Eigen::Vector3d::UnitY(), kGeometryTolerance) ||
      !approximately_equal(left_origin.x(), right_origin.x()) ||
      !approximately_equal(left_origin.z(), right_origin.z()) ||
      !approximately_equal(left_origin.y(), -right_origin.y()) || left_origin.y() <= 0.0)
    {
      return failure<GripperStagingGeometry>(
        "finger origins and axes must form a symmetric opposing-Y jaw pair");
    }
    const double inner_gap_at_zero =
      left_origin.y() - right_origin.y() - finger_size.y();
    if (!std::isfinite(inner_gap_at_zero) || inner_gap_at_zero <= 0.0) {
      return failure<GripperStagingGeometry>(
        "derived zero-position inner jaw gap must be positive");
    }

    const YAML::Node attachment = root["attachment"];
    const double hold_clearance = nonnegative(
      attachment["hold_clearance_per_side_m"],
      "attachment.hold_clearance_per_side_m");
    const double open_clearance = nonnegative(
      attachment["open_clearance_per_side_m"],
      "attachment.open_clearance_per_side_m");
    const double maximum_open_target = nonnegative(
      attachment["open_target_m"], "attachment.open_target_m");
    if (open_clearance <= hold_clearance) {
      return failure<GripperStagingGeometry>(
        "open clearance must exceed hold clearance");
    }
    if (maximum_open_target < joint_lower || maximum_open_target > joint_upper) {
      return failure<GripperStagingGeometry>(
        "maximum open target must lie within the jaw joint bounds");
    }

    return GripperGeometryResult<GripperStagingGeometry>::success(
      GripperStagingGeometry{
        ParallelJawGeometry{inner_gap_at_zero, joint_lower, joint_upper},
        tool0_from_grasp_center, hold_clearance, open_clearance, maximum_open_target});
  } catch (const YAML::Exception & error) {
    return failure<GripperStagingGeometry>(
      "failed to parse gripper geometry: " + std::string(error.what()));
  } catch (const std::invalid_argument & error) {
    return failure<GripperStagingGeometry>(error.what());
  }
}

}  // namespace restocker_task_executor
