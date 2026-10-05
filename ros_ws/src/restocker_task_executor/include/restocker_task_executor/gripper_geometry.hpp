// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <filesystem>
#include <string>
#include <utility>
#include <variant>

#include "restocker_task_executor/grasp_candidates.hpp"

namespace restocker_task_executor
{

enum class GripperGeometryErrorCode : std::uint8_t
{
  kInvalidConfiguration,
};

struct GripperGeometryError
{
  GripperGeometryErrorCode code{GripperGeometryErrorCode::kInvalidConfiguration};
  std::string detail;
};

template<typename T>
class [[nodiscard]] GripperGeometryResult
{
public:
  [[nodiscard]] static GripperGeometryResult success(T value)
  {
    return GripperGeometryResult(std::move(value));
  }

  [[nodiscard]] static GripperGeometryResult failure(GripperGeometryError error)
  {
    return GripperGeometryResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const GripperGeometryError & error() const
  {
    return std::get<GripperGeometryError>(storage_);
  }

private:
  explicit GripperGeometryResult(T value)
  : storage_(std::move(value)) {}

  explicit GripperGeometryResult(GripperGeometryError error)
  : storage_(std::move(error)) {}

  std::variant<T, GripperGeometryError> storage_;
};

struct GripperStagingGeometry
{
  ParallelJawGeometry jaw;
  Eigen::Isometry3d tool0_from_grasp_center{Eigen::Isometry3d::Identity()};
  double hold_clearance_per_side_m{0.0};
  double open_clearance_per_side_m{0.0};
  double maximum_open_target_m{0.0};
};

// Loads the description-owned datum used by both Xacro and deterministic grasp staging. The
// result describes expected geometry; runtime TF remains independent evidence that must match it.
[[nodiscard]] GripperGeometryResult<GripperStagingGeometry> load_gripper_staging_geometry(
  const std::filesystem::path & path);

}  // namespace restocker_task_executor
