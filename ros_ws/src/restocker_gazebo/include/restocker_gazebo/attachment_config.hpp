// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "restocker_gazebo/attachment_physics.hpp"

namespace restocker_gazebo
{

// AttachmentHoldMode and its wire spelling come from attachment_journal.hpp, which
// attachment_physics.hpp already includes: one definition for the simulator boundary, the
// executor's jaw policy and the codec.

struct AttachmentProductConfig
{
  std::string source_object_id;
  std::string model_name;
  std::string child_link;
  std::string geometry_key;
  std::string product_class;
  std::string sku;
  CylinderAttachmentGeometry geometry;
};

struct AttachmentBoundaryConfig
{
  std::string robot_model_name;
  std::string parent_link;
  std::string left_finger_joint;
  std::string right_finger_joint;
  std::string set_service;
  std::string query_service;
  AttachmentHoldMode hold_mode{AttachmentHoldMode::kPhysical};
  Eigen::Isometry3d gripper_from_grasp_center{Eigen::Isometry3d::Identity()};
  ParallelJawAttachmentGeometry gripper;
  AttachmentPhysicalTolerances tolerances;
  double post_attach_translation_drift_m{0.0};
  double post_attach_rotation_drift_rad{0.0};
  // Spec M10 section 11 D5 amendment: how far a physically held product's origin may swing about
  // the jaw axis (DART has no torsional friction, section 11.2 F13); slip at the contact stays
  // bounded by post_attach_translation_drift_m.
  double held_swing_allowance_m{0.0};
  // Physical hold only: the cumulative slip a pad may show before the grip counts as lost
  // (post_attach_translation_drift_m keeps judging a weld), and the tilt of the product off the
  // jaw axis (its lean between the pads) that is tolerated.
  double held_slip_allowance_m{0.0};
  double held_tilt_allowance_rad{0.0};
  std::size_t journal_capacity{0};
  std::size_t detach_attempt_reserve{0};
  std::size_t required_verification_ticks{0};
  std::size_t maximum_pending_ticks{0};
  std::map<std::string, AttachmentProductConfig> products_by_source_id;
  std::map<std::string, std::string> source_id_by_model_name;
  // Spec M10 section 11 D10 item 2: the pad-to-product coefficient the attachment system sets on
  // the contact pair itself, keyed by product class. DART combines the two bodies with min()
  // (section 11.2 F5), so a pad's own mu would do nothing; overriding the pair is what makes the
  // 0.8/0.3 pad figures reach the contact. Every class in the scenario must be present here, or
  // the config load fails.
  std::map<std::string, double> pad_friction_by_product_class;
};

struct AttachmentConfigError
{
  std::string detail;
};

class [[nodiscard]] AttachmentConfigResult
{
public:
  [[nodiscard]] static AttachmentConfigResult success(AttachmentBoundaryConfig value)
  {
    return AttachmentConfigResult(std::move(value));
  }

  [[nodiscard]] static AttachmentConfigResult failure(std::string detail)
  {
    return AttachmentConfigResult(AttachmentConfigError{std::move(detail)});
  }

  [[nodiscard]] bool has_value() const noexcept
  {
    return std::holds_alternative<AttachmentBoundaryConfig>(storage_);
  }
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const AttachmentBoundaryConfig & value() const
  {
    return std::get<AttachmentBoundaryConfig>(storage_);
  }
  [[nodiscard]] AttachmentBoundaryConfig & value()
  {
    return std::get<AttachmentBoundaryConfig>(storage_);
  }
  [[nodiscard]] const AttachmentConfigError & error() const
  {
    return std::get<AttachmentConfigError>(storage_);
  }

private:
  explicit AttachmentConfigResult(AttachmentBoundaryConfig value)
  : storage_(std::move(value)) {}

  explicit AttachmentConfigResult(AttachmentConfigError error)
  : storage_(std::move(error)) {}

  std::variant<AttachmentBoundaryConfig, AttachmentConfigError> storage_;
};

[[nodiscard]] AttachmentConfigResult load_attachment_boundary_config(
  const std::filesystem::path & boundary_path,
  const std::filesystem::path & gripper_geometry_path,
  const std::filesystem::path & product_catalog_path,
  const std::filesystem::path & scenario_path);

}  // namespace restocker_gazebo
