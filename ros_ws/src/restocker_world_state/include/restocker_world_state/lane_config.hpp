// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{

[[nodiscard]] std::vector<LaneDefinition> load_lane_definitions(
  const std::filesystem::path & semantic_config_path,
  const std::filesystem::path & workcell_geometry_path);

// Loads the durable desired-stocking document written by save_lane_policy_state. Same schema as
// the lane semantic config (schema_version plus a lane map of expected class, optional SKU and
// target_count); throws std::invalid_argument on a malformed document, and never guesses a
// missing field.
[[nodiscard]] std::vector<LanePolicy> load_lane_policy_state(
  const std::filesystem::path & policy_state_path);

// Writes the full desired-stocking table to `policy_state_path` as that schema, replacing any
// previous document atomically: the content is written and fsynced to a temporary file in the
// same directory, then renamed over the target, then the directory is fsynced. A failure throws
// std::runtime_error with the previous document left intact. Validates every row before touching
// the filesystem so a caller defect cannot half-write a file.
void save_lane_policy_state(
  const std::filesystem::path & policy_state_path,
  const std::vector<LanePolicy> & policies);

// Overlays a loaded policy document onto configured definitions at restart, so a restart resumes
// the owner's intent instead of the shipped baseline. Every state row must name a configured lane
// and every configured lane must appear exactly once; any mismatch throws std::invalid_argument
// rather than applying half of the owner's table.
void apply_lane_policy_state(
  std::vector<LaneDefinition> & definitions,
  const std::vector<LanePolicy> & policies);

// Derives, for every catalogued product, the depth one more of it costs a gravity-fed lane.
//
// Lane capacity and "was one added" are depth questions, not occupancy questions. The depth comes
// from the product's catalogued envelope and the shelf's surveyed incline and rear entry
// clearance, read from the same two files as the rest of the system.
[[nodiscard]] std::vector<ProductLaneProfile> load_product_lane_profiles(
  const std::filesystem::path & product_catalog_path,
  const std::filesystem::path & workcell_geometry_path);

}  // namespace restocker_world_state
