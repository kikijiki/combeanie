// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace restocker_perception
{

// One entry of the inventory the cell is known to hold: a category, and the identity the rest of
// the system knows that item by.
//
// A manifest entry carries no measured pose. Appearance and position are perception's to measure;
// what the system calls a product is not something a camera can see. The one position it carries
// is the declared stocking position, used only to choose a name (Card 063).
struct ManifestEntry
{
  std::uint8_t product_class{0};
  std::string source_object_id;
  // Where the scenario declares this product was stocked (the translation of its spawn_pose).
  // It chooses between the names of a category stocked more than once and is never published
  // as, or mixed into, a measured pose; see observation_identity.hpp (Card 063).
  std::optional<Eigen::Vector3d> declared_position;
};

// Reads the stocked inventory out of a scenario document.
//
// The world state keys objects on an external identity, and the simulated attachment adapter
// resolves a reserved source_object_id to a Gazebo model through an immutable scenario mapping,
// failing closed on anything it does not recognise. A camera cannot know a model name, so identity
// comes from the inventory the cell is stocked from (a pick list, in a warehouse) while every pose
// comes from the sensor.
//
// This loader reads `product_class`, `source_object_id` and the translation of `spawn_pose`. The
// last is world construction, the same document the attachment adapter's mapping comes from, and
// it only picks which stocked name a measured detection gets when a category is stocked more than
// once. It never becomes an observation's pose, so perception still measures every position.
//
// Throws std::invalid_argument if the document cannot be read, has no products, names an unknown
// category, declares an empty or duplicated identity, or stocks a category more than once without
// a finite declared position for every such product.
[[nodiscard]] std::vector<ManifestEntry> load_object_manifest(const std::string & path);

// Throws std::invalid_argument naming `origin` when a category is stocked more than once and one
// of its entries has no finite declared position. The loader and the identity assigner both hold
// this line, so a manifest built in code cannot skip it.
void require_declared_positions_where_named_by_pose(
  const std::vector<ManifestEntry> & manifest, const std::string & origin);

}  // namespace restocker_perception
