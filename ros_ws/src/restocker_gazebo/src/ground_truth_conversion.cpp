// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/ground_truth_conversion.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace restocker_gazebo
{
namespace
{

using Observation = restocker_interfaces::msg::ObjectObservation;

[[nodiscard]] std::uint8_t parse_product_class(const std::string & value)
{
  if (value == "can") {
    return Observation::PRODUCT_CLASS_CAN;
  }
  if (value == "small_bottle") {
    return Observation::PRODUCT_CLASS_SMALL_BOTTLE;
  }
  if (value == "large_bottle") {
    return Observation::PRODUCT_CLASS_LARGE_BOTTLE;
  }
  throw std::invalid_argument("unknown product_class: " + value);
}

void require_non_empty(const std::string & value, std::string_view field)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(field) + " must not be empty");
  }
}

[[nodiscard]] bool finite_pose(const gz::msgs::Pose & pose)
{
  const auto & position = pose.position();
  const auto & orientation = pose.orientation();
  return std::isfinite(position.x()) && std::isfinite(position.y()) &&
         std::isfinite(position.z()) && std::isfinite(orientation.x()) &&
         std::isfinite(orientation.y()) && std::isfinite(orientation.z()) &&
         std::isfinite(orientation.w());
}

[[nodiscard]] Observation make_common_observation(
  const ProductMetadata & product, const GroundTruthConfig & config)
{
  Observation observation;
  observation.header.frame_id = config.frame_id;
  observation.source_object_id = product.source_object_id;
  observation.product_class = product.product_class;
  observation.has_sku = product.sku.has_value();
  observation.sku = product.sku.value_or("");
  observation.backend_name = config.backend_name;
  observation.backend_version = config.backend_version;
  observation.confidence = 1.0F;
  for (std::size_t index = 0; index < config.covariance_diagonal.size(); ++index) {
    observation.pose.covariance[index * 6 + index] = config.covariance_diagonal[index];
  }
  return observation;
}

[[nodiscard]] std::uint8_t classify_orientation(double x, double y, double z, double w)
{
  // World Z expressed by the product quaternion; its Z component is the axis alignment.
  const double vertical_alignment = 1.0 - 2.0 * (x * x + y * y);
  constexpr double upright_threshold = 0.9659258262890683;    // cos(15 deg)
  constexpr double horizontal_threshold = 0.2588190451025207;  // sin(15 deg)
  if (vertical_alignment >= upright_threshold) {
    return Observation::ORIENTATION_UPRIGHT;
  }
  if (std::abs(vertical_alignment) <= horizontal_threshold) {
    return Observation::ORIENTATION_HORIZONTAL;
  }
  static_cast<void>(z);
  static_cast<void>(w);
  return Observation::ORIENTATION_TILTED;
}

}  // namespace

GroundTruthConfig load_ground_truth_config(const std::filesystem::path & path)
{
  const YAML::Node root = YAML::LoadFile(path.string());
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("ground-truth config requires schema_version 1");
  }

  GroundTruthConfig config;
  config.pose_topic = root["pose_topic"].as<std::string>();
  config.frame_id = root["frame_id"].as<std::string>();
  config.backend_name = root["backend"]["name"].as<std::string>();
  config.backend_version = root["backend"]["version"].as<std::string>();
  require_non_empty(config.pose_topic, "pose_topic");
  require_non_empty(config.frame_id, "frame_id");
  require_non_empty(config.backend_name, "backend.name");
  require_non_empty(config.backend_version, "backend.version");

  const YAML::Node workcell_pose = root["workcell_pose"];
  if (!workcell_pose.IsSequence() || workcell_pose.size() != config.world_from_shelf_pose.size()) {
    throw std::invalid_argument("workcell_pose must contain six values");
  }
  for (std::size_t index = 0; index < config.world_from_shelf_pose.size(); ++index) {
    const double value = workcell_pose[index].as<double>();
    if (!std::isfinite(value)) {
      throw std::invalid_argument("workcell_pose values must be finite");
    }
    config.world_from_shelf_pose[index] = value;
  }

  const YAML::Node covariance = root["pose_covariance_diagonal"];
  if (!covariance.IsSequence() || covariance.size() != config.covariance_diagonal.size()) {
    throw std::invalid_argument("pose_covariance_diagonal must contain six values");
  }
  for (std::size_t index = 0; index < config.covariance_diagonal.size(); ++index) {
    const double value = covariance[index].as<double>();
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("pose covariance diagonal values must be finite and positive");
    }
    config.covariance_diagonal[index] = value;
  }

  const YAML::Node products = root["products"];
  if (!products.IsSequence() || products.size() == 0) {
    throw std::invalid_argument("products must contain at least one entry");
  }
  std::unordered_set<std::string> model_names;
  std::unordered_set<std::string> source_ids;
  config.products.reserve(products.size());
  for (const YAML::Node & node : products) {
    ProductMetadata product;
    product.model_name = node["model_name"].as<std::string>();
    product.source_object_id = node["source_object_id"].as<std::string>();
    product.geometry_key = node["geometry_key"].as<std::string>();
    product.product_class = parse_product_class(node["product_class"].as<std::string>());
    if (node["sku"] && !node["sku"].as<std::string>().empty()) {
      product.sku = node["sku"].as<std::string>();
    }
    require_non_empty(product.model_name, "products[].model_name");
    require_non_empty(product.source_object_id, "products[].source_object_id");
    require_non_empty(product.geometry_key, "products[].geometry_key");
    if (!model_names.insert(product.model_name).second) {
      throw std::invalid_argument("duplicate product model_name: " + product.model_name);
    }
    if (!source_ids.insert(product.source_object_id).second) {
      throw std::invalid_argument(
              "duplicate product source_object_id: " +
              product.source_object_id);
    }
    config.products.push_back(std::move(product));
  }
  return config;
}

std::vector<Observation> convert_pose_sample(
  const gz::msgs::Pose_V & sample, const GroundTruthConfig & config)
{
  std::unordered_map<std::string, std::vector<const gz::msgs::Pose *>> matching_poses;
  matching_poses.reserve(config.products.size());
  for (const auto & pose : sample.pose()) {
    matching_poses[pose.name()].push_back(&pose);
  }

  const bool valid_stamp = sample.has_header() && sample.header().has_stamp() &&
    sample.header().stamp().sec() >= 0 && sample.header().stamp().nsec() >= 0 &&
    sample.header().stamp().nsec() < 1'000'000'000 &&
    sample.header().stamp().sec() <= std::numeric_limits<std::int32_t>::max();

  std::vector<Observation> observations;
  observations.reserve(config.products.size());
  for (const ProductMetadata & product : config.products) {
    const auto found = matching_poses.find(product.model_name);
    if (found == matching_poses.end()) {
      continue;
    }

    Observation observation = make_common_observation(product, config);
    if (!valid_stamp) {
      observation.status = Observation::STATUS_MISSING_TIMESTAMP;
      observation.status_detail = "Gazebo pose sample has no valid simulation timestamp";
      observations.push_back(std::move(observation));
      continue;
    }
    observation.header.stamp.sec = static_cast<std::int32_t>(sample.header().stamp().sec());
    observation.header.stamp.nanosec = static_cast<std::uint32_t>(sample.header().stamp().nsec());
    if (found->second.size() != 1) {
      observation.status = Observation::STATUS_DUPLICATE_SOURCE;
      observation.status_detail = "Gazebo pose sample contains duplicate configured model name";
      observations.push_back(std::move(observation));
      continue;
    }

    const gz::msgs::Pose & pose = *found->second.front();
    if (!finite_pose(pose)) {
      observation.status = Observation::STATUS_INVALID_POSE;
      observation.status_detail = "Gazebo product pose contains a non-finite value";
      observations.push_back(std::move(observation));
      continue;
    }
    const auto & quaternion = pose.orientation();
    const double norm = std::sqrt(
      quaternion.x() * quaternion.x() + quaternion.y() * quaternion.y() +
      quaternion.z() * quaternion.z() + quaternion.w() * quaternion.w());
    if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
      observation.status = Observation::STATUS_INVALID_POSE;
      observation.status_detail = "Gazebo product quaternion is not unit length";
      observations.push_back(std::move(observation));
      continue;
    }

    observation.pose.pose.position.x = pose.position().x();
    observation.pose.pose.position.y = pose.position().y();
    observation.pose.pose.position.z = pose.position().z();
    observation.pose.pose.orientation.x = quaternion.x() / norm;
    observation.pose.pose.orientation.y = quaternion.y() / norm;
    observation.pose.pose.orientation.z = quaternion.z() / norm;
    observation.pose.pose.orientation.w = quaternion.w() / norm;
    observation.orientation = classify_orientation(
      observation.pose.pose.orientation.x, observation.pose.pose.orientation.y,
      observation.pose.pose.orientation.z, observation.pose.pose.orientation.w);
    observation.status = Observation::STATUS_OK;
    observations.push_back(std::move(observation));
  }
  return observations;
}

}  // namespace restocker_gazebo
