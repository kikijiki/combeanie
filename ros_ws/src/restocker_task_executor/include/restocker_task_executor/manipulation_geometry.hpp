// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <variant>

namespace restocker_task_executor
{

enum class ManipulationGeometryErrorCode : std::uint8_t
{
  InvalidConfiguration,
  InvalidTransform,
  InvalidEnvelope,
};

struct ManipulationGeometryError
{
  ManipulationGeometryErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] ManipulationGeometryResult
{
public:
  [[nodiscard]] static ManipulationGeometryResult success(T value)
  {
    return ManipulationGeometryResult(std::move(value));
  }

  [[nodiscard]] static ManipulationGeometryResult failure(ManipulationGeometryError error)
  {
    return ManipulationGeometryResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const ManipulationGeometryError & error() const
  {
    return std::get<ManipulationGeometryError>(storage_);
  }

private:
  explicit ManipulationGeometryResult(T value)
  : storage_(std::move(value)) {}

  explicit ManipulationGeometryResult(ManipulationGeometryError error)
  : storage_(std::move(error)) {}

  std::variant<T, ManipulationGeometryError> storage_;
};

struct CylinderEnvelope
{
  double radius_m{0.0};
  double height_m{0.0};
};

struct LaneManipulationGeometry
{
  std::string id;
  std::string frame_id;
  double center_x_m{0.0};
  // Nominal volume: its floor is the roller surface at the front rail, which is where a
  // gravity-fed product comes to rest, so a settled product touches that face directly.
  // Containment is judged against this volume with the floor face lowered by
  // floor_settle_tolerance_m instead, because a seated product touches it exactly and settles a
  // fraction of a millimetre into it. Everywhere behind the rail the roller surface stands above
  // this floor rather than below it, which the volume already contains.
  Eigen::AlignedBox3d usable_bounds_in_lane;
  double floor_settle_tolerance_m{0.0};
  // Clear depth the release leaves behind the product's own rear face. It is the only surveyed
  // part of the release depth; the depth itself is derived per transfer from the held product's
  // radius by `lane_release_product_center_depth_m`, because a lane holds a column of products of
  // different widths and the narrower one can be let go nearer the mouth. Neither is where the
  // product ends up: the bed carries it from here to the front rail, or to the back of whatever
  // is already there.
  double insert_entry_clearance_m{0.0};
  // The roller bed. Its surface is level with usable_bounds_in_lane's floor at
  // floor_datum_depth_m, the rail face, and rises behind it at incline_rad, so its height at
  // any lane depth is lane_floor_height_m() below. Both come from the shelf survey rather than
  // from a lane, because one bed serves every lane.
  double incline_rad{0.0};
  double floor_datum_depth_m{0.0};
  Eigen::Vector3d insertion_axis{Eigen::Vector3d::UnitY()};
};

// Where the arm lets go of a product of this radius: just inside the lane's rear entrance, with
// the product's own rear face one entry clearance in from the usable volume's rear face.
//
// This is derived per transfer rather than surveyed per lane because a gravity-fed lane holds a
// column. The depth a placement *consumes* is the product's own footprint, so a wide product must
// be let go deeper than a narrow one merely to be inside the volume, and a single configured depth
// would charge the can the bottle's reach. It is also emphatically not where the product comes to
// rest, that depends on how many are already in the lane, which is the bed's business and not
// the arm's.
[[nodiscard]] inline double lane_release_product_center_depth_m(
  const LaneManipulationGeometry & lane, double product_radius_m) noexcept
{
  return lane.usable_bounds_in_lane.min().y() + product_radius_m + lane.insert_entry_clearance_m;
}

// Prefer the configured entry clearance, but let the last product in a measured column approach
// the existing column while preserving `containment_margin_m` both behind its rear face and ahead
// of its front face. Selection and placement share this pose so the final semantic slot does not
// become unreachable merely because the nominal entry gap is wider than the remaining free gap.
[[nodiscard]] inline double lane_adaptive_release_product_center_depth_m(
  const LaneManipulationGeometry & lane, double product_radius_m,
  double available_depth_m, double containment_margin_m) noexcept
{
  const double nominal = lane_release_product_center_depth_m(lane, product_radius_m);
  const double measured_limit = lane.usable_bounds_in_lane.min().y() + available_depth_m -
    product_radius_m - containment_margin_m;
  return std::min(nominal, measured_limit);
}

// Height of the roller surface above the lane usable volume's floor, at a lane depth. Positive
// everywhere behind the front rail and zero at it.
[[nodiscard]] inline double lane_floor_height_m(
  const LaneManipulationGeometry & lane, double depth_m) noexcept
{
  return (lane.floor_datum_depth_m - depth_m) * std::tan(lane.incline_rad);
}

// How high a product has to be carried to be inserted into a lane at all. The insert is a
// straight horizontal push, so the product passes over every part of the bed between the lane
// mouth and the insert depth, and the highest of those is the bed's rear lip at depth zero. A
// release referenced to the bed under the product itself would drag its uphill rim through the
// bed on the way in.
[[nodiscard]] inline double lane_insertion_floor_height_m(
  const LaneManipulationGeometry & lane) noexcept
{
  return lane_floor_height_m(lane, 0.0);
}

struct WorkcellManipulationGeometry
{
  Eigen::AlignedBox3d stock_region_in_shelf;
  std::map<std::string, LaneManipulationGeometry> lanes;
};

[[nodiscard]] ManipulationGeometryResult<WorkcellManipulationGeometry>
load_manipulation_geometry(const std::filesystem::path & path);

// The grasp-centre frame is a physical frame on the gripper rather than a convention this package
// may choose. restocker_description mounts grasp_center on tool0 with a pure translation, so the
// frame inherits tool0's axes: +z is the direction the gripper points, +y is the parallel-jaw
// closing axis that the two opposing fingers travel along, and +x is the jaw-width axis, which for
// an upright cylinder carries the product's own axis and along which the wrist camera is mounted.
// Every producer and consumer of a grasp-centre pose must read the axes through these accessors so
// the three files cannot drift into disagreeing about which column means what.
[[nodiscard]] inline Eigen::Vector3d grasp_approach_axis(
  const Eigen::Isometry3d & frame_from_grasp_center) noexcept
{
  return frame_from_grasp_center.linear().col(2);
}

[[nodiscard]] inline Eigen::Vector3d grasp_closing_axis(
  const Eigen::Isometry3d & frame_from_grasp_center) noexcept
{
  return frame_from_grasp_center.linear().col(1);
}

[[nodiscard]] inline Eigen::Vector3d grasp_jaw_width_axis(
  const Eigen::Isometry3d & frame_from_grasp_center) noexcept
{
  return frame_from_grasp_center.linear().col(0);
}

[[nodiscard]] ManipulationGeometryResult<Eigen::AlignedBox3d>
cylinder_axis_aligned_bounds(
  const Eigen::Isometry3d & frame_from_cylinder, const CylinderEnvelope & envelope);

// Absolute slack allowed on every containment face, in metres. It exists solely to make a pose
// that was constructed flush against a face compare as contained despite the rounding of the
// rigid-transform composition it passed through; it is far below any physical tolerance here.
inline constexpr double kContainmentRoundingSlackM = 1.0e-9;

[[nodiscard]] ManipulationGeometryResult<bool> contains_cylinder(
  const Eigen::AlignedBox3d & outer_bounds, const Eigen::Isometry3d & outer_from_cylinder,
  const CylinderEnvelope & envelope, double margin_m = 0.0);

// Maximum number of identical products the surveyed gravity-feed lane admits. This is derived
// from the same settled-cylinder envelope and rear release clearance used by lane evidence and
// placement, so semantic occupancy can fail closed if simulator contact lets cylinders overlap
// and momentarily makes measured free depth look larger than a physically possible column.
[[nodiscard]] ManipulationGeometryResult<std::size_t> lane_column_capacity(
  const LaneManipulationGeometry & lane, const CylinderEnvelope & envelope,
  double depth_tolerance_m = 0.0);

}  // namespace restocker_task_executor
