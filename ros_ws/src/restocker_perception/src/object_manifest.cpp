// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/object_manifest.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <restocker_interfaces/msg/object_observation.hpp>

namespace restocker_perception
{
namespace
{

using ObservationMessage = restocker_interfaces::msg::ObjectObservation;

[[nodiscard]] std::uint8_t parse_product_class(const std::string & value)
{
  if (value == "can") {return ObservationMessage::PRODUCT_CLASS_CAN;}
  if (value == "small_bottle") {return ObservationMessage::PRODUCT_CLASS_SMALL_BOTTLE;}
  if (value == "large_bottle") {return ObservationMessage::PRODUCT_CLASS_LARGE_BOTTLE;}
  throw std::invalid_argument("unknown product_class in the object manifest: " + value);
}

}  // namespace

void require_declared_positions_where_named_by_pose(
  const std::vector<ManifestEntry> & manifest, const std::string & origin)
{
  for (const auto & entry : manifest) {
    const auto stocked = std::ranges::count_if(
      manifest, [&entry](const ManifestEntry & candidate) {
        return candidate.product_class == entry.product_class;
      });
    if (stocked > 1 &&
      (!entry.declared_position.has_value() || !entry.declared_position->allFinite()))
    {
      throw std::invalid_argument(
              origin + ": '" + entry.source_object_id +
              "' shares its category with another stocked product but declares no finite "
              "stocking position, so nothing could choose between their names");
    }
  }
}

std::vector<ManifestEntry> load_object_manifest(const std::string & path)
{
  if (path.empty()) {
    throw std::invalid_argument("object manifest path must not be empty");
  }
  YAML::Node document;
  try {
    document = YAML::LoadFile(path);
  } catch (const YAML::Exception & error) {
    throw std::invalid_argument("could not read object manifest " + path + ": " + error.what());
  }
  const YAML::Node products = document["products"];
  if (!products || !products.IsSequence() || products.size() == 0) {
    throw std::invalid_argument(path + " declares no stocked products");
  }

  std::vector<ManifestEntry> manifest;
  std::set<std::string> seen;
  for (const auto & product : products) {
    // The spawn pose's translation is read to name, never to measure: see object_manifest.hpp.
    ManifestEntry entry;
    entry.product_class = parse_product_class(product["product_class"].as<std::string>());
    entry.source_object_id = product["source_object_id"].as<std::string>();
    if (entry.source_object_id.empty()) {
      throw std::invalid_argument(path + " declares a product with an empty source identity");
    }
    if (const YAML::Node pose = product["spawn_pose"]; pose && pose.IsSequence() &&
      pose.size() >= 3U)
    {
      entry.declared_position =
        Eigen::Vector3d(pose[0].as<double>(), pose[1].as<double>(), pose[2].as<double>());
    }
    if (!seen.insert(entry.source_object_id).second) {
      throw std::invalid_argument(
              path + " declares '" + entry.source_object_id + "' more than once");
    }
    manifest.push_back(std::move(entry));
  }
  require_declared_positions_where_named_by_pose(manifest, path);
  return manifest;
}

}  // namespace restocker_perception
