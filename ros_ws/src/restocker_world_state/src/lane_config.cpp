// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_world_state/lane_config.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

namespace restocker_world_state
{
namespace
{

[[nodiscard]] ProductClass parse_product_class(const std::string & value)
{
  if (value == "can") {
    return ProductClass::Can;
  }
  if (value == "small_bottle") {
    return ProductClass::SmallBottle;
  }
  if (value == "large_bottle") {
    return ProductClass::LargeBottle;
  }
  throw std::invalid_argument("unknown expected_product_class: " + value);
}

[[nodiscard]] const char * product_class_name(ProductClass product_class)
{
  switch (product_class) {
    case ProductClass::Can:
      return "can";
    case ProductClass::SmallBottle:
      return "small_bottle";
    case ProductClass::LargeBottle:
      return "large_bottle";
    case ProductClass::Unknown:
      break;
  }
  throw std::invalid_argument("lane policy cannot serialize an unknown expected_product_class");
}

void require_schema(const YAML::Node & root, const char * name)
{
  if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument(std::string(name) + " requires schema_version 1");
  }
  if (!root["lanes"].IsMap() || root["lanes"].size() == 0) {
    throw std::invalid_argument(std::string(name) + " must contain a lane map");
  }
}

[[nodiscard]] std::map<std::string, YAML::Node> keyed_lanes(
  const YAML::Node & lanes, const char * name)
{
  std::map<std::string, YAML::Node> result;
  for (const auto & item : lanes) {
    const std::string id = item.first.as<std::string>();
    if (id.empty()) {
      throw std::invalid_argument(std::string(name) + " contains an empty lane ID");
    }
    if (!result.emplace(id, item.second).second) {
      throw std::invalid_argument(std::string(name) + " contains duplicate lane ID " + id);
    }
  }
  return result;
}

[[nodiscard]] std::optional<std::string> parse_expected_sku(
  const YAML::Node & semantic, const std::string & id)
{
  if (!semantic["expected_sku"]) {
    return std::nullopt;
  }
  const std::string value = semantic["expected_sku"].as<std::string>();
  if (value.empty()) {
    throw std::invalid_argument("expected_sku cannot be empty when present for " + id);
  }
  return value;
}

// Required, not defaulted: an owner who has not said how many they want has not declared a
// policy, and a silently invented count is exactly the kind of guess this loader refuses
// everywhere else.
[[nodiscard]] std::uint32_t parse_target_count(const YAML::Node & semantic, const std::string & id)
{
  const YAML::Node node = semantic["target_count"];
  if (!node || !node.IsScalar()) {
    throw std::invalid_argument("lane " + id + " requires a target_count");
  }
  std::int64_t value = 0;
  try {
    value = node.as<std::int64_t>();
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("target_count must be an integer for " + id);
  }
  if (value < 0 ||
    value > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max()))
  {
    throw std::invalid_argument(
            "target_count must fit in an unsigned 32-bit integer for " + id);
  }
  return static_cast<std::uint32_t>(value);
}

[[nodiscard]] LanePolicy parse_lane_policy(const std::string & id, const YAML::Node & semantic)
{
  try {
    if (!semantic["expected_product_class"] || !semantic["expected_product_class"].IsScalar()) {
      throw std::invalid_argument("lane " + id + " requires an expected_product_class");
    }
    const std::string class_name = semantic["expected_product_class"].as<std::string>();
    return LanePolicy{
      LaneId{id}, parse_product_class(class_name), parse_expected_sku(semantic, id),
      parse_target_count(semantic, id)};
  } catch (const YAML::Exception &) {
    throw std::invalid_argument("lane policy for " + id + " is malformed");
  }
}

// Durable replace: write and fsync a temporary file in the target's directory, rename it over
// the target, then fsync the directory. Nothing throws after the rename succeeds, because the
// caller's contract is that a thrown error left the previous document intact — reporting a
// failure after the new document is already visible would break exactly that.
void replace_file_durably(const std::filesystem::path & target, const std::string & content)
{
  const std::filesystem::path directory =
    target.parent_path().empty() ? std::filesystem::path(".") : target.parent_path();
  std::error_code directory_error;
  std::filesystem::create_directories(directory, directory_error);
  if (directory_error) {
    throw std::runtime_error(
            "cannot create lane policy state directory " + directory.string() + ": " +
            directory_error.message());
  }
  const std::filesystem::path temporary = target.string() + ".tmp";
  const int fd = ::open(temporary.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
  if (fd < 0) {
    throw std::runtime_error(
            "cannot open " + temporary.string() + ": " + std::strerror(errno));
  }
  auto discard_and_throw = [&temporary](const std::string & message) {
    ::unlink(temporary.c_str());
    throw std::runtime_error(message);
  };
  std::size_t offset = 0;
  while (offset < content.size()) {
    const ssize_t written = ::write(fd, content.data() + offset, content.size() - offset);
    if (written < 0) {
      if (errno == EINTR) {
        continue;
      }
      const std::string message =
        "cannot write " + temporary.string() + ": " + std::strerror(errno);
      ::close(fd);
      discard_and_throw(message);
    }
    offset += static_cast<std::size_t>(written);
  }
  if (::fsync(fd) != 0) {
    const std::string message =
      "cannot fsync " + temporary.string() + ": " + std::strerror(errno);
    ::close(fd);
    discard_and_throw(message);
  }
  if (::close(fd) != 0) {
    discard_and_throw("cannot close " + temporary.string() + ": " + std::strerror(errno));
  }
  if (::rename(temporary.c_str(), target.c_str()) != 0) {
    discard_and_throw(
      "cannot replace " + target.string() + ": " + std::strerror(errno));
  }
  const int directory_fd = ::open(directory.c_str(), O_RDONLY | O_DIRECTORY);
  if (directory_fd >= 0) {
    static_cast<void>(::fsync(directory_fd));
    static_cast<void>(::close(directory_fd));
  }
}

}  // namespace

std::vector<LaneDefinition> load_lane_definitions(
  const std::filesystem::path & semantic_config_path,
  const std::filesystem::path & workcell_geometry_path)
{
  const YAML::Node semantics = YAML::LoadFile(semantic_config_path.string());
  const YAML::Node geometry = YAML::LoadFile(workcell_geometry_path.string());
  require_schema(semantics, "lane semantic config");
  require_schema(geometry, "workcell geometry");

  const auto semantic_lanes = keyed_lanes(semantics["lanes"], "lane semantic config");
  const auto geometry_lanes = keyed_lanes(geometry["lanes"], "workcell geometry");
  if (semantic_lanes.size() != geometry_lanes.size() ||
    !std::equal(
      semantic_lanes.begin(), semantic_lanes.end(), geometry_lanes.begin(),
      [](const auto & semantic, const auto & geometric) {
        return semantic.first == geometric.first;
      }))
  {
    throw std::invalid_argument(
            "lane semantic config and workcell geometry must define identical lane IDs");
  }

  std::vector<LaneDefinition> definitions;
  definitions.reserve(semantic_lanes.size());
  for (const auto & [id, semantic] : semantic_lanes) {
    const LanePolicy policy = parse_lane_policy(id, semantic);
    const double depth = geometry_lanes.at(id)["usable_depth_m"].as<double>();
    if (!std::isfinite(depth) || depth <= 0.0) {
      throw std::invalid_argument("usable_depth_m must be finite and positive for " + id);
    }
    definitions.push_back(
      LaneDefinition{
        policy.id, policy.expected_product_class, policy.expected_sku, depth,
        policy.target_count});
  }
  return definitions;
}

std::vector<LanePolicy> load_lane_policy_state(
  const std::filesystem::path & policy_state_path)
{
  const YAML::Node root = YAML::LoadFile(policy_state_path.string());
  require_schema(root, "lane policy state");
  const auto lanes = keyed_lanes(root["lanes"], "lane policy state");
  std::vector<LanePolicy> policies;
  policies.reserve(lanes.size());
  for (const auto & [id, semantic] : lanes) {
    policies.push_back(parse_lane_policy(id, semantic));
  }
  return policies;
}

void save_lane_policy_state(
  const std::filesystem::path & policy_state_path,
  const std::vector<LanePolicy> & policies)
{
  if (policies.empty()) {
    throw std::invalid_argument("lane policy table is empty");
  }
  YAML::Node root;
  root["schema_version"] = 1;
  for (const LanePolicy & policy : policies) {
    if (policy.id.value.empty()) {
      throw std::invalid_argument("lane policy table contains an empty lane ID");
    }
    if (policy.expected_product_class == ProductClass::Unknown) {
      throw std::invalid_argument(
              "lane policy for " + policy.id.value + " has no known expected product class");
    }
    if (policy.expected_sku && policy.expected_sku->empty()) {
      throw std::invalid_argument(
              "expected_sku cannot be empty when present for " + policy.id.value);
    }
    YAML::Node lane;
    lane["expected_product_class"] = product_class_name(policy.expected_product_class);
    if (policy.expected_sku) {
      lane["expected_sku"] = *policy.expected_sku;
    }
    lane["target_count"] = policy.target_count;
    if (root["lanes"][policy.id.value]) {
      throw std::invalid_argument(
              "lane policy table contains duplicate lane ID " + policy.id.value);
    }
    root["lanes"][policy.id.value] = lane;
  }
  const std::string content = YAML::Dump(root) + "\n";
  replace_file_durably(policy_state_path, content);
}

void apply_lane_policy_state(
  std::vector<LaneDefinition> & definitions,
  const std::vector<LanePolicy> & policies)
{
  if (policies.size() != definitions.size()) {
    throw std::invalid_argument(
            "lane policy state and the configured shelf define different numbers of lanes");
  }
  std::map<std::string, const LanePolicy *> by_id;
  for (const LanePolicy & policy : policies) {
    if (!by_id.emplace(policy.id.value, &policy).second) {
      throw std::invalid_argument(
              "lane policy state contains duplicate lane ID " + policy.id.value);
    }
  }
  // Validate the whole overlay before writing any of it: a state file that names the wrong
  // shelf must leave the configured definitions exactly as the baseline set them.
  for (const LaneDefinition & definition : definitions) {
    if (!by_id.contains(definition.id.value)) {
      throw std::invalid_argument(
              "lane policy state does not name configured lane " + definition.id.value);
    }
  }
  for (LaneDefinition & definition : definitions) {
    const LanePolicy & policy = *by_id.at(definition.id.value);
    definition.expected_product_class = policy.expected_product_class;
    definition.expected_sku = policy.expected_sku;
    definition.target_count = policy.target_count;
  }
}


std::vector<ProductLaneProfile> load_product_lane_profiles(
  const std::filesystem::path & product_catalog_path,
  const std::filesystem::path & workcell_geometry_path)
{
  const YAML::Node catalog = YAML::LoadFile(product_catalog_path.string());
  if (!catalog["schema_version"] || catalog["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("product collision catalog requires schema_version 1");
  }
  const YAML::Node geometries = catalog["geometries"];
  if (!geometries.IsSequence() || geometries.size() == 0) {
    throw std::invalid_argument("product collision catalog must contain geometries");
  }

  const YAML::Node geometry = YAML::LoadFile(workcell_geometry_path.string());
  require_schema(geometry, "workcell geometry");
  const YAML::Node shelf = geometry["shelf"];
  if (!shelf || !shelf.IsMap() || !shelf["lane_incline_deg"]) {
    throw std::invalid_argument("workcell geometry must survey the shelf lane incline");
  }
  const double incline_deg = shelf["lane_incline_deg"].as<double>();
  if (!std::isfinite(incline_deg) || incline_deg < 0.0 || incline_deg >= 45.0) {
    throw std::invalid_argument("shelf lane incline must be finite and under 45 degrees");
  }
  const double incline_rad = incline_deg * std::acos(-1.0) / 180.0;

  // The room the release leaves behind the product, taken as the largest any lane asks for so one
  // profile is admissible in every lane. The shipped shelf gives every lane the same value; the
  // maximum stays correct if one ever differs, because a profile that fits the
  // tightest lane fits them all.
  double entry_clearance = 0.0;
  for (const auto & item : geometry["lanes"]) {
    const std::string id = item.first.as<std::string>();
    const YAML::Node node = item.second["insert_entry_clearance_m"];
    if (!node) {
      throw std::invalid_argument("lane " + id + " does not survey insert_entry_clearance_m");
    }
    const double value = node.as<double>();
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(
              "insert_entry_clearance_m must be finite and non-negative for " + id);
    }
    entry_clearance = std::max(entry_clearance, value);
  }

  std::vector<ProductLaneProfile> profiles;
  profiles.reserve(geometries.size());
  for (const YAML::Node & entry : geometries) {
    const YAML::Node shape = entry["shape"];
    if (!shape || !shape["type"] || shape["type"].as<std::string>() != "cylinder") {
      throw std::invalid_argument("lane placement profiles support only cylindrical products");
    }
    const double radius = shape["radius_m"].as<double>();
    if (!std::isfinite(radius) || radius <= 0.0) {
      throw std::invalid_argument("product shape.radius_m must be finite and positive");
    }
    std::optional<std::string> sku;
    if (entry["sku"] && !entry["sku"].as<std::string>().empty() &&
      !(entry["class_fallback"] && entry["class_fallback"].as<bool>()))
    {
      sku = entry["sku"].as<std::string>();
    }
    // Two products packed on the incline stand one diameter apart along the bed, and the lane's
    // depth axis is horizontal, so the depth each one adds is that diameter foreshortened by the
    // incline. The first product into an empty lane consumes more than this (its tilted
    // envelope adds half a height's worth of sine on top), which is why the pitch is what a
    // placement must *at least* consume rather than what it consumes exactly.
    const double pitch = 2.0 * radius * std::cos(incline_rad);
    // The release puts the product's own rear face `entry_clearance` inside the usable volume,
    // so the free depth it needs is its own diameter plus that clearance. Nothing here is the
    // depth it comes to rest at: the bed decides that, and it is always deeper.
    profiles.push_back(
      ProductLaneProfile{
        parse_product_class(entry["product_class"].as<std::string>()), std::move(sku),
        2.0 * radius + entry_clearance, pitch});
  }
  return profiles;
}

}  // namespace restocker_world_state
