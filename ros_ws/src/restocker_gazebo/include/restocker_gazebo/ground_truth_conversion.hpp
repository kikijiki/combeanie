// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <gz/msgs/pose_v.pb.h>

#include <array>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <restocker_interfaces/msg/object_observation.hpp>

namespace restocker_gazebo
{

struct ProductMetadata
{
  std::string model_name;
  std::string source_object_id;
  std::string geometry_key;
  std::uint8_t product_class{0};
  std::optional<std::string> sku;
};

struct GroundTruthConfig
{
  std::string pose_topic;
  std::string frame_id;
  std::string backend_name;
  std::string backend_version;
  std::array<double, 6> world_from_shelf_pose{};
  std::array<double, 6> covariance_diagonal{};
  std::vector<ProductMetadata> products;
};

[[nodiscard]] GroundTruthConfig load_ground_truth_config(const std::filesystem::path & path);

[[nodiscard]] std::vector<restocker_interfaces::msg::ObjectObservation> convert_pose_sample(
  const gz::msgs::Pose_V & sample, const GroundTruthConfig & config);

}  // namespace restocker_gazebo
