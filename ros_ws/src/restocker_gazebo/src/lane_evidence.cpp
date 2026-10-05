// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/lane_evidence.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <unordered_map>
#include <unordered_set>

namespace restocker_gazebo
{
namespace
{

using LaneObservation = restocker_interfaces::msg::LaneObservation;
using ObjectObservation = restocker_interfaces::msg::ObjectObservation;

constexpr double kContainmentToleranceM = 1.0e-9;

void require_non_empty(const std::string & value, std::string_view field)
{
  if (value.empty()) {
    throw std::invalid_argument(std::string(field) + " must not be empty");
  }
}

[[nodiscard]] double positive_finite(const YAML::Node & node, const char * field)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(field) + " must be finite and positive");
  }
  return value;
}

[[nodiscard]] std::uint8_t product_class_from_text(const std::string & value)
{
  if (value == "can") {
    return ObjectObservation::PRODUCT_CLASS_CAN;
  }
  if (value == "small_bottle") {
    return ObjectObservation::PRODUCT_CLASS_SMALL_BOTTLE;
  }
  if (value == "large_bottle") {
    return ObjectObservation::PRODUCT_CLASS_LARGE_BOTTLE;
  }
  throw std::invalid_argument("unknown product_class: " + value);
}

[[nodiscard]] Eigen::Isometry3d pose_from_xyz_rpy(const std::array<double, 6> & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  result.linear() =
    (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

[[nodiscard]] bool valid_stamp(const gz::msgs::Pose_V & sample)
{
  return sample.has_header() && sample.header().has_stamp() &&
         sample.header().stamp().sec() >= 0 && sample.header().stamp().nsec() >= 0 &&
         sample.header().stamp().nsec() < 1'000'000'000 &&
         sample.header().stamp().sec() <= std::numeric_limits<std::int32_t>::max();
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

[[nodiscard]] std::optional<Eigen::Isometry3d> pose_from_message(const gz::msgs::Pose & pose)
{
  if (!finite_pose(pose)) {
    return std::nullopt;
  }
  const auto & orientation = pose.orientation();
  Eigen::Quaterniond quaternion(
    orientation.w(), orientation.x(), orientation.y(), orientation.z());
  const double norm = quaternion.norm();
  if (!std::isfinite(norm) || std::abs(norm - 1.0) > 1.0e-3) {
    return std::nullopt;
  }
  quaternion.normalize();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(
    pose.position().x(), pose.position().y(), pose.position().z());
  result.linear() = quaternion.toRotationMatrix();
  return result;
}

[[nodiscard]] Eigen::AlignedBox3d cylinder_bounds(
  const ProductEnvelope & product, const Eigen::Isometry3d & lane_from_product)
{
  const Eigen::Vector3d axis = lane_from_product.linear().col(2);
  Eigen::Vector3d extents;
  for (Eigen::Index index = 0; index < 3; ++index) {
    const double axial = std::clamp(std::abs(axis[index]), 0.0, 1.0);
    extents[index] = product.radius_m * std::sqrt(std::max(0.0, 1.0 - axial * axial)) +
      0.5 * product.height_m * axial;
  }
  return Eigen::AlignedBox3d(
    lane_from_product.translation() - extents,
    lane_from_product.translation() + extents);
}

[[nodiscard]] bool overlaps(
  const Eigen::AlignedBox3d & left, const Eigen::AlignedBox3d & right)
{
  return (left.max().array() > right.min().array()).all() &&
         (right.max().array() > left.min().array()).all();
}

[[nodiscard]] bool contains(
  const Eigen::AlignedBox3d & outer, const Eigen::AlignedBox3d & inner)
{
  return (inner.min().array() >= outer.min().array() - kContainmentToleranceM).all() &&
         (inner.max().array() <= outer.max().array() + kContainmentToleranceM).all();
}

void set_error(
  std::vector<LaneObservation> & observations, std::uint8_t status, const std::string & detail)
{
  for (auto & observation : observations) {
    observation.status = status;
    observation.status_detail = detail;
  }
}

}  // namespace

LaneEvidenceConfig load_lane_evidence_config(
  const std::filesystem::path & workcell_geometry_path,
  const std::filesystem::path & product_catalog_path,
  const GroundTruthConfig & ground_truth_config)
{
  LaneEvidenceConfig config;
  config.world_from_shelf = pose_from_xyz_rpy(ground_truth_config.world_from_shelf_pose);
  config.backend_name = ground_truth_config.backend_name;
  config.backend_version = ground_truth_config.backend_version;

  const YAML::Node catalog = YAML::LoadFile(product_catalog_path.string());
  if (!catalog["schema_version"] || catalog["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("product catalog requires schema_version 1");
  }
  const YAML::Node geometries = catalog["geometries"];
  if (!geometries.IsSequence() || geometries.size() == 0) {
    throw std::invalid_argument("product catalog must contain geometries");
  }
  std::unordered_map<std::string, YAML::Node> geometry_by_key;
  for (const YAML::Node & geometry : geometries) {
    const std::string key = geometry["geometry_key"].as<std::string>();
    require_non_empty(key, "geometries[].geometry_key");
    if (!geometry_by_key.emplace(key, geometry).second) {
      throw std::invalid_argument("duplicate product geometry_key: " + key);
    }
  }

  config.products.reserve(ground_truth_config.products.size());
  for (const ProductMetadata & product : ground_truth_config.products) {
    const auto geometry = geometry_by_key.find(product.geometry_key);
    if (geometry == geometry_by_key.end()) {
      throw std::invalid_argument("unknown product geometry_key: " + product.geometry_key);
    }
    const YAML::Node shape = geometry->second["shape"];
    if (!shape || shape["type"].as<std::string>() != "cylinder") {
      throw std::invalid_argument("lane evidence supports only cylindrical product geometry");
    }
    if (product_class_from_text(geometry->second["product_class"].as<std::string>()) !=
      product.product_class)
    {
      throw std::invalid_argument(
              "scenario product class disagrees with catalog geometry " + product.geometry_key);
    }
    if (geometry->second["sku"] && !geometry->second["sku"].as<std::string>().empty()) {
      const std::string catalog_sku = geometry->second["sku"].as<std::string>();
      if (!product.sku || catalog_sku != *product.sku) {
        throw std::invalid_argument(
                "scenario product SKU disagrees with catalog geometry " + product.geometry_key);
      }
    }
    config.products.push_back(
      ProductEnvelope{
        product.model_name, product.source_object_id,
        positive_finite(shape["radius_m"], "shape.radius_m"),
        positive_finite(shape["height_m"], "shape.height_m")});
  }

  const YAML::Node workcell = YAML::LoadFile(workcell_geometry_path.string());
  if (!workcell["schema_version"] || workcell["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("workcell geometry requires schema_version 1");
  }
  const YAML::Node lanes = workcell["lanes"];
  if (!lanes.IsMap() || lanes.size() == 0) {
    throw std::invalid_argument("workcell geometry must contain lanes");
  }
  std::unordered_set<std::string> frame_ids;
  config.lanes.reserve(lanes.size());
  for (const auto & item : lanes) {
    const std::string id = item.first.as<std::string>();
    const YAML::Node lane = item.second;
    const std::string frame_id = lane["frame_id"].as<std::string>();
    require_non_empty(id, "lanes key");
    require_non_empty(frame_id, "lanes[].frame_id");
    if (frame_id != id) {
      throw std::invalid_argument("lane frame_id must match its geometry key: " + id);
    }
    if (!frame_ids.insert(frame_id).second) {
      throw std::invalid_argument("duplicate lane frame_id: " + frame_id);
    }
    const YAML::Node insertion_axis = lane["insertion_axis"];
    if (!insertion_axis.IsSequence() || insertion_axis.size() != 3 ||
      insertion_axis[0].as<double>() != 0.0 || insertion_axis[1].as<double>() != 1.0 ||
      insertion_axis[2].as<double>() != 0.0)
    {
      throw std::invalid_argument(
              "lane evidence requires the documented positive-Y insertion axis");
    }
    const double width = positive_finite(lane["usable_width_m"], "usable_width_m");
    const double depth = positive_finite(lane["usable_depth_m"], "usable_depth_m");
    const double height = positive_finite(lane["usable_height_m"], "usable_height_m");
    const double rear = lane["rear_clearance_m"].as<double>();
    const double floor = lane["floor_clearance_m"].as<double>();
    // Only the floor face is lowered, by the same allowance as the manipulation geometry: a
    // correctly placed product settles a fraction of a millimetre into the shelf and would
    // otherwise read as overlapping but not contained.
    const YAML::Node settle_node = lane["floor_settle_tolerance_m"];
    const double floor_settle = settle_node ? settle_node.as<double>() : 0.0;
    if (!std::isfinite(floor_settle) || floor_settle < 0.0) {
      throw std::invalid_argument("lane floor settle tolerance must be finite and non-negative");
    }
    const double center_x = lane["center_x_m"].as<double>();
    if (!std::isfinite(rear) || rear < 0.0 || !std::isfinite(floor) || floor < 0.0 ||
      !std::isfinite(center_x))
    {
      throw std::invalid_argument(
              "lane center and rear/floor clearances must be finite; "
              "clearances cannot be negative");
    }
    config.lanes.push_back(
      LaneVolume{
        id, frame_id, center_x,
        Eigen::AlignedBox3d(
          Eigen::Vector3d(-0.5 * width, rear, floor - floor_settle),
          Eigen::Vector3d(0.5 * width, rear + depth, floor + height))});
  }
  std::sort(
    config.lanes.begin(), config.lanes.end(),
    [](const LaneVolume & left, const LaneVolume & right) {return left.id < right.id;});
  return config;
}

std::vector<LaneObservation> convert_lane_evidence_sample(
  const gz::msgs::Pose_V & sample, const LaneEvidenceConfig & config)
{
  std::vector<LaneObservation> observations;
  observations.reserve(config.lanes.size());
  for (const LaneVolume & lane : config.lanes) {
    LaneObservation observation;
    observation.header.frame_id = lane.frame_id;
    observation.lane_id = lane.id;
    observation.available_depth_m = lane.bounds_in_lane.sizes().y();
    observation.confidence = 1.0F;
    observation.backend_name = config.backend_name;
    observation.backend_version = config.backend_version;
    observation.status = LaneObservation::STATUS_OK;
    observations.push_back(std::move(observation));
  }

  if (!valid_stamp(sample)) {
    set_error(
      observations, LaneObservation::STATUS_MISSING_TIMESTAMP,
      "Gazebo pose sample has no valid simulation timestamp");
    return observations;
  }
  for (auto & observation : observations) {
    observation.header.stamp.sec = static_cast<std::int32_t>(sample.header().stamp().sec());
    observation.header.stamp.nanosec =
      static_cast<std::uint32_t>(sample.header().stamp().nsec());
  }

  std::unordered_map<std::string, std::vector<const gz::msgs::Pose *>> pose_by_name;
  pose_by_name.reserve(config.products.size());
  for (const gz::msgs::Pose & pose : sample.pose()) {
    pose_by_name[pose.name()].push_back(&pose);
  }

  const Eigen::Isometry3d shelf_from_world = config.world_from_shelf.inverse();
  for (const ProductEnvelope & product : config.products) {
    const auto found = pose_by_name.find(product.model_name);
    if (found == pose_by_name.end()) {
      continue;
    }
    if (found->second.size() != 1) {
      set_error(
        observations, LaneObservation::STATUS_DUPLICATE_SOURCE,
        "Gazebo pose sample contains duplicate configured model name");
      return observations;
    }
    const auto world_from_product = pose_from_message(*found->second.front());
    if (!world_from_product) {
      set_error(
        observations, LaneObservation::STATUS_INVALID_GEOMETRY,
        "Gazebo product pose is not a finite rigid transform");
      return observations;
    }
    const Eigen::Isometry3d shelf_from_product = shelf_from_world * *world_from_product;
    for (std::size_t index = 0; index < config.lanes.size(); ++index) {
      const LaneVolume & lane = config.lanes[index];
      LaneObservation & observation = observations[index];
      Eigen::Isometry3d lane_from_product = shelf_from_product;
      lane_from_product.translation().x() -= lane.center_x_m;
      const Eigen::AlignedBox3d bounds = cylinder_bounds(product, lane_from_product);
      if (!overlaps(lane.bounds_in_lane, bounds)) {
        continue;
      }
      const double free_depth = std::clamp(
        bounds.min().y() - lane.bounds_in_lane.min().y(), 0.0,
        lane.bounds_in_lane.sizes().y());
      observation.available_depth_m = std::min(observation.available_depth_m, free_depth);
      if (contains(lane.bounds_in_lane, bounds)) {
        observation.observed_source_object_ids.push_back(product.source_object_id);
      } else {
        observation.obstructed = true;
      }
    }
  }

  for (auto & observation : observations) {
    std::sort(
      observation.observed_source_object_ids.begin(),
      observation.observed_source_object_ids.end());
  }
  return observations;
}

}  // namespace restocker_gazebo
