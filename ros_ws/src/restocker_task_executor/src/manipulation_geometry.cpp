// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/manipulation_geometry.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>

namespace restocker_task_executor
{
namespace
{

template<typename T>
[[nodiscard]] ManipulationGeometryResult<T> failure(
  ManipulationGeometryErrorCode code, std::string detail)
{
  return ManipulationGeometryResult<T>::failure(
    ManipulationGeometryError{code, std::move(detail)});
}

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool finite_rigid_transform(const Eigen::Isometry3d & transform)
{
  return transform.matrix().allFinite() && transform.linear().isUnitary(1.0e-6) &&
         std::abs(transform.linear().determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] Eigen::Vector3d vector3(const YAML::Node & node, const std::string & field)
{
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(field + " must contain exactly three values");
  }
  Eigen::Vector3d value(
    node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!value.allFinite()) {
    throw std::invalid_argument(field + " must be finite");
  }
  return value;
}

[[nodiscard]] double nonnegative(const YAML::Node & node, const std::string & field)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(field + " must be finite and non-negative");
  }
  return value;
}

[[nodiscard]] double positive(const YAML::Node & node, const std::string & field)
{
  const double value = node.as<double>();
  if (!finite_positive(value)) {
    throw std::invalid_argument(field + " must be finite and positive");
  }
  return value;
}

}  // namespace

ManipulationGeometryResult<WorkcellManipulationGeometry> load_manipulation_geometry(
  const std::filesystem::path & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
      return failure<WorkcellManipulationGeometry>(
        ManipulationGeometryErrorCode::InvalidConfiguration,
        "workcell manipulation geometry requires schema_version 1");
    }

    const YAML::Node stock = root["stock_tray"]["usable_volume"];
    const Eigen::Vector3d stock_center = vector3(stock["center_xyz_m"], "stock center");
    const Eigen::Vector3d stock_size = vector3(stock["size_xyz_m"], "stock size");
    if ((stock_size.array() <= 0.0).any()) {
      return failure<WorkcellManipulationGeometry>(
        ManipulationGeometryErrorCode::InvalidConfiguration,
        "stock usable-volume size must be positive");
    }
    const YAML::Node stock_settle = stock["floor_settle_tolerance_m"];
    const double stock_floor_settle_tolerance = stock_settle ?
      nonnegative(stock_settle, "stock floor settle tolerance") : 0.0;

    const YAML::Node lanes = root["lanes"];
    if (!lanes.IsMap() || lanes.size() == 0) {
      return failure<WorkcellManipulationGeometry>(
        ManipulationGeometryErrorCode::InvalidConfiguration,
        "workcell manipulation geometry must contain lanes");
    }

    // One roller bed serves every lane, so its two numbers come from the shelf survey rather than
    // from a lane. The datum is the front rail's face, because that is the depth at which the bed
    // is level with the lane usable volume's floor: a product resting on the rail is on the lane
    // floor, and everything behind it stands higher.
    const YAML::Node shelf = root["shelf"];
    if (!shelf || !shelf.IsMap()) {
      return failure<WorkcellManipulationGeometry>(
        ManipulationGeometryErrorCode::InvalidConfiguration,
        "workcell manipulation geometry must contain a shelf survey");
    }
    const double shelf_depth = positive(shelf["depth_m"], "shelf depth");
    const double retainer_depth = positive(
      shelf["front_retainer_depth_m"], "shelf front retainer depth");
    const double incline_deg = nonnegative(shelf["lane_incline_deg"], "shelf lane incline");
    const double floor_datum_depth = shelf_depth - retainer_depth;
    // A quarter turn would put the lane mouth over the customer's head; anything approaching it
    // is a units error, not a shelf.
    if (incline_deg >= 45.0 || floor_datum_depth <= 0.0) {
      return failure<WorkcellManipulationGeometry>(
        ManipulationGeometryErrorCode::InvalidConfiguration,
        "shelf lane incline must be under 45 degrees over a positive rail-face depth");
    }
    const double incline_rad = incline_deg * std::acos(-1.0) / 180.0;

    WorkcellManipulationGeometry geometry;
    // Stock sits on the tray, so the surveyed floor and a seated product's envelope floor are
    // the same plane. Only that face carries the settle tolerance; the remaining five stay
    // exactly where the survey puts them.
    geometry.stock_region_in_shelf = Eigen::AlignedBox3d(
      stock_center - 0.5 * stock_size -
      Eigen::Vector3d(0.0, 0.0, stock_floor_settle_tolerance),
      stock_center + 0.5 * stock_size);
    std::set<std::string> frame_ids;
    for (const auto & item : lanes) {
      const std::string id = item.first.as<std::string>();
      const YAML::Node lane = item.second;
      const std::string frame_id = lane["frame_id"].as<std::string>();
      if (id.empty() || frame_id.empty() || frame_id != id || !frame_ids.insert(frame_id).second) {
        return failure<WorkcellManipulationGeometry>(
          ManipulationGeometryErrorCode::InvalidConfiguration,
          "lane IDs and frame IDs must be non-empty, unique, and equal");
      }
      const double width = positive(lane["usable_width_m"], id + " usable width");
      const double depth = positive(lane["usable_depth_m"], id + " usable depth");
      const double height = positive(lane["usable_height_m"], id + " usable height");
      const double rear = nonnegative(lane["rear_clearance_m"], id + " rear clearance");
      const double floor = nonnegative(lane["floor_clearance_m"], id + " floor clearance");
      const YAML::Node lane_settle = lane["floor_settle_tolerance_m"];
      const double floor_settle = lane_settle ?
        nonnegative(lane_settle, id + " floor settle tolerance") : 0.0;
      static_cast<void>(nonnegative(lane["side_clearance_m"], id + " side clearance"));
      static_cast<void>(nonnegative(lane["front_clearance_m"], id + " front clearance"));
      const double center_x = lane["center_x_m"].as<double>();
      const double entry_clearance = nonnegative(
        lane["insert_entry_clearance_m"], id + " insert entry clearance");
      const Eigen::Vector3d insertion_axis = vector3(
        lane["insertion_axis"], id + " insertion axis");
      if (!std::isfinite(center_x) ||
        !insertion_axis.isApprox(Eigen::Vector3d::UnitY(), 1.0e-12))
      {
        return failure<WorkcellManipulationGeometry>(
          ManipulationGeometryErrorCode::InvalidConfiguration,
          "lane center must be finite and insertion axis must be positive Y");
      }
      const Eigen::AlignedBox3d bounds(
        Eigen::Vector3d(-0.5 * width, rear, floor),
        Eigen::Vector3d(0.5 * width, rear + depth, floor + height));
      // The release depth itself is per product, so what can be checked here is the part that is
      // not: an entry clearance alone must already leave the release behind the rail face, or no
      // product would ever be let go where the bed could carry it forward. The per-transfer check
      // that the whole envelope fits is the placement generator's, against the product it holds.
      if (bounds.min().y() + entry_clearance >= floor_datum_depth) {
        return failure<WorkcellManipulationGeometry>(
          ManipulationGeometryErrorCode::InvalidConfiguration,
          "lane insert entry clearance puts the release at or past the front rail face");
      }
      geometry.lanes.emplace(
        id,
        LaneManipulationGeometry{
          id, frame_id, center_x, bounds, floor_settle, entry_clearance, incline_rad,
          floor_datum_depth, insertion_axis});
    }
    return ManipulationGeometryResult<WorkcellManipulationGeometry>::success(
      std::move(geometry));
  } catch (const YAML::Exception & error) {
    return failure<WorkcellManipulationGeometry>(
      ManipulationGeometryErrorCode::InvalidConfiguration,
      "failed to parse workcell manipulation geometry: " + std::string(error.what()));
  } catch (const std::invalid_argument & error) {
    return failure<WorkcellManipulationGeometry>(
      ManipulationGeometryErrorCode::InvalidConfiguration, error.what());
  }
}

ManipulationGeometryResult<std::size_t> lane_column_capacity(
  const LaneManipulationGeometry & lane, const CylinderEnvelope & envelope,
  double depth_tolerance_m)
{
  if (!std::isfinite(envelope.radius_m) || envelope.radius_m <= 0.0 ||
    !std::isfinite(envelope.height_m) || envelope.height_m <= 0.0 ||
    !std::isfinite(lane.incline_rad) || lane.incline_rad < 0.0 ||
    lane.incline_rad >= 0.25 * std::acos(-1.0) ||
    !std::isfinite(lane.floor_datum_depth_m) ||
    !std::isfinite(lane.insert_entry_clearance_m) ||
    lane.insert_entry_clearance_m < 0.0 || !std::isfinite(depth_tolerance_m) ||
    depth_tolerance_m < 0.0 || lane.usable_bounds_in_lane.isEmpty())
  {
    return failure<std::size_t>(
      ManipulationGeometryErrorCode::InvalidConfiguration,
      "lane column capacity requires finite positive cylinder and lane geometry");
  }

  const double pitch_m = 2.0 * envelope.radius_m * std::cos(lane.incline_rad);
  const double required_rear_depth_m =
    2.0 * envelope.radius_m + lane.insert_entry_clearance_m;
  const double rear_depth_m = lane.usable_bounds_in_lane.min().y();
  const double full_depth_m = lane.usable_bounds_in_lane.sizes().y();
  const double settled_y_extent_m =
    envelope.radius_m * std::cos(lane.incline_rad) +
    0.5 * envelope.height_m * std::sin(lane.incline_rad);
  if (!std::isfinite(pitch_m) || pitch_m <= 0.0 || !std::isfinite(full_depth_m) ||
    full_depth_m <= 0.0)
  {
    return failure<std::size_t>(
      ManipulationGeometryErrorCode::InvalidConfiguration,
      "lane column capacity derived a non-positive depth");
  }

  std::size_t capacity = 0U;
  double available_depth_m = full_depth_m;
  // A geometry corrupt enough to need ten thousand products is not a useful lane. The bound also
  // makes this helper total if a future floating-point change prevents the depth from decreasing.
  while (available_depth_m + depth_tolerance_m + kContainmentRoundingSlackM >=
    required_rear_depth_m &&
    capacity < 10000U)
  {
    ++capacity;
    const double rearmost_center_depth_m =
      lane.floor_datum_depth_m - envelope.radius_m -
      static_cast<double>(capacity - 1U) * pitch_m;
    available_depth_m = std::clamp(
      rearmost_center_depth_m - settled_y_extent_m - rear_depth_m, 0.0, full_depth_m);
  }
  if (capacity == 10000U) {
    return failure<std::size_t>(
      ManipulationGeometryErrorCode::InvalidConfiguration,
      "lane column capacity exceeded its finite safety bound");
  }
  return ManipulationGeometryResult<std::size_t>::success(capacity);
}

ManipulationGeometryResult<Eigen::AlignedBox3d> cylinder_axis_aligned_bounds(
  const Eigen::Isometry3d & frame_from_cylinder, const CylinderEnvelope & envelope)
{
  if (!finite_rigid_transform(frame_from_cylinder)) {
    return failure<Eigen::AlignedBox3d>(
      ManipulationGeometryErrorCode::InvalidTransform,
      "cylinder transform must be finite and rigid");
  }
  if (!finite_positive(envelope.radius_m) || !finite_positive(envelope.height_m)) {
    return failure<Eigen::AlignedBox3d>(
      ManipulationGeometryErrorCode::InvalidEnvelope,
      "cylinder radius and height must be finite and positive");
  }

  const Eigen::Matrix3d & rotation = frame_from_cylinder.linear();
  Eigen::Vector3d extent;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    extent[axis] = envelope.radius_m *
      std::hypot(rotation(axis, 0), rotation(axis, 1)) +
      0.5 * envelope.height_m * std::abs(rotation(axis, 2));
  }
  return ManipulationGeometryResult<Eigen::AlignedBox3d>::success(
    Eigen::AlignedBox3d(
      frame_from_cylinder.translation() - extent,
      frame_from_cylinder.translation() + extent));
}

ManipulationGeometryResult<bool> contains_cylinder(
  const Eigen::AlignedBox3d & outer_bounds, const Eigen::Isometry3d & outer_from_cylinder,
  const CylinderEnvelope & envelope, double margin_m)
{
  if (outer_bounds.isEmpty() || !outer_bounds.min().allFinite() ||
    !outer_bounds.max().allFinite() || !std::isfinite(margin_m) || margin_m < 0.0)
  {
    return failure<bool>(
      ManipulationGeometryErrorCode::InvalidEnvelope,
      "containment bounds and margin must be finite and valid");
  }
  const Eigen::Vector3d inner_min = outer_bounds.min() + Eigen::Vector3d::Constant(margin_m);
  const Eigen::Vector3d inner_max = outer_bounds.max() - Eigen::Vector3d::Constant(margin_m);
  if ((inner_min.array() > inner_max.array()).any()) {
    return failure<bool>(
      ManipulationGeometryErrorCode::InvalidEnvelope,
      "containment margin consumes the outer volume");
  }
  auto cylinder_bounds = cylinder_axis_aligned_bounds(outer_from_cylinder, envelope);
  if (!cylinder_bounds) {
    return failure<bool>(cylinder_bounds.error().code, cylinder_bounds.error().detail);
  }
  // Callers construct poses that seat a product exactly on an inset face (the lane placement
  // puts the product's base at min().z() + margin by definition) and then ask this predicate
  // whether that pose is contained. The half-extent it is compared against is recovered from a
  // composed rigid transform, so it carries a few units in the last place of rounding, and an
  // exact comparison turns a deliberate flush fit into a containment failure. A nanometre is
  // nine orders of magnitude below any dimension in this workcell, so it excludes nothing
  // physical while making the boundary decidable.
  const Eigen::Vector3d slack = Eigen::Vector3d::Constant(kContainmentRoundingSlackM);
  return ManipulationGeometryResult<bool>::success(
    (cylinder_bounds.value().min().array() >= (inner_min - slack).array()).all() &&
    (cylinder_bounds.value().max().array() <= (inner_max + slack).array()).all());
}

}  // namespace restocker_task_executor
