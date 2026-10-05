// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>
#include <gz/msgs/pose_v.pb.h>

#include <filesystem>
#include <string>
#include <vector>

#include <restocker_interfaces/msg/lane_observation.hpp>

#include "restocker_gazebo/ground_truth_conversion.hpp"

namespace restocker_gazebo
{

struct ProductEnvelope
{
  std::string model_name;
  std::string source_object_id;
  double radius_m{0.0};
  double height_m{0.0};
};

struct LaneVolume
{
  std::string id;
  std::string frame_id;
  double center_x_m{0.0};
  Eigen::AlignedBox3d bounds_in_lane;
};

struct LaneEvidenceConfig
{
  Eigen::Isometry3d world_from_shelf{Eigen::Isometry3d::Identity()};
  std::string backend_name;
  std::string backend_version;
  std::vector<ProductEnvelope> products;
  std::vector<LaneVolume> lanes;
};

[[nodiscard]] LaneEvidenceConfig load_lane_evidence_config(
  const std::filesystem::path & workcell_geometry_path,
  const std::filesystem::path & product_catalog_path,
  const GroundTruthConfig & ground_truth_config);

[[nodiscard]] std::vector<restocker_interfaces::msg::LaneObservation>
convert_lane_evidence_sample(
  const gz::msgs::Pose_V & sample, const LaneEvidenceConfig & config);

}  // namespace restocker_gazebo
