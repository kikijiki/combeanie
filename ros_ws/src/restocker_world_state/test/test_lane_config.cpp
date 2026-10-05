// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>

#include "restocker_world_state/lane_config.hpp"

namespace restocker_world_state
{
namespace
{

[[nodiscard]] std::filesystem::path write_yaml(
  const YAML::Node & root, const std::string & name)
{
  const auto path = std::filesystem::path(::testing::TempDir()) / name;
  std::ofstream stream(path);
  stream << root;
  stream.close();
  return path;
}

TEST(LaneConfig, JoinsSemanticCompatibilityWithDescriptionDepth)
{
  const auto definitions = load_lane_definitions(
    RESTOCKER_TEST_LANE_SEMANTICS, RESTOCKER_TEST_WORKCELL_GEOMETRY);
  ASSERT_EQ(definitions.size(), 6U);
  EXPECT_EQ(definitions[0].id.value, "lane_01");
  EXPECT_EQ(definitions[0].expected_product_class, ProductClass::Can);
  EXPECT_EQ(definitions[0].expected_sku, "SIM-CAN-STD");
  EXPECT_DOUBLE_EQ(definitions[0].depth_m, 0.85);
  // The shipped baseline declares its want explicitly; the loader never invents one.
  EXPECT_EQ(definitions[0].target_count, 6U);
  EXPECT_EQ(definitions[1].expected_product_class, ProductClass::SmallBottle);
  EXPECT_EQ(definitions[1].target_count, 6U);
  EXPECT_EQ(definitions[2].expected_product_class, ProductClass::LargeBottle);
  EXPECT_EQ(definitions[2].target_count, 6U);
  // Every class owns two lanes, so a product always has an alternative destination when one of
  // them is occupied or obstructed. The sibling carries the identical policy: a lane pair that
  // disagreed about class or SKU would make the alternative unusable exactly when it is needed.
  EXPECT_EQ(definitions[3].id.value, "lane_04");
  EXPECT_EQ(definitions[3].expected_product_class, definitions[0].expected_product_class);
  EXPECT_EQ(definitions[3].expected_sku, definitions[0].expected_sku);
  EXPECT_EQ(definitions[4].expected_product_class, definitions[1].expected_product_class);
  EXPECT_EQ(definitions[4].expected_sku, definitions[1].expected_sku);
  EXPECT_EQ(definitions[5].expected_product_class, definitions[2].expected_product_class);
  EXPECT_EQ(definitions[5].expected_sku, definitions[2].expected_sku);
  EXPECT_EQ(definitions[5].target_count, 6U);
}

TEST(LaneConfig, SameShapePolicySplitsTheCanLanesBySku)
{
  // The policy that goes with the two same-shaped cans. Its whole content is that the two can
  // lanes disagree about SKU while agreeing about class, which is what makes reading the label
  // the only way to route either can, and is the one arrangement no shipped policy had before.
  // Asserted against the shipped file because a lane pair that quietly drifted back into
  // agreement would leave the scenario runnable and the measurement meaningless.
  const auto definitions = load_lane_definitions(
    RESTOCKER_TEST_SAME_SHAPE_LANE_SEMANTICS, RESTOCKER_TEST_WORKCELL_GEOMETRY);
  ASSERT_EQ(definitions.size(), 6U);
  EXPECT_EQ(definitions[0].id.value, "lane_01");
  EXPECT_EQ(definitions[0].expected_product_class, ProductClass::Can);
  EXPECT_EQ(definitions[0].expected_sku, "SIM-CAN-STD");
  EXPECT_EQ(definitions[3].id.value, "lane_04");
  EXPECT_EQ(definitions[3].expected_product_class, ProductClass::Can);
  EXPECT_EQ(definitions[3].expected_sku, "SIM-CAN-CITRUS");
  EXPECT_NE(definitions[0].expected_sku, definitions[3].expected_sku);
  EXPECT_DOUBLE_EQ(definitions[3].depth_m, definitions[0].depth_m);
  EXPECT_EQ(definitions[3].target_count, definitions[0].target_count);
}

TEST(LaneConfig, RejectsMissingOrExtraSemanticLanes)
{
  YAML::Node semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"].remove("lane_03");
  const auto missing = write_yaml(semantics, "missing_lane.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(missing, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);

  semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_07"] = semantics["lanes"]["lane_03"];
  const auto extra = write_yaml(semantics, "extra_lane.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(extra, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);
}

TEST(LaneConfig, RejectsUnknownClassAndEmptySku)
{
  YAML::Node semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_01"]["expected_product_class"] = "unknown_product";
  const auto unknown_class = write_yaml(semantics, "unknown_class.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(
      unknown_class, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);

  semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_01"]["expected_sku"] = "";
  const auto empty_sku = write_yaml(semantics, "empty_sku.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(
      empty_sku, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);
}

// A lane whose want was never declared is refused, not defaulted: target_count drives the
// deficit calculation, and a silently invented count would report a policy nobody stated.
TEST(LaneConfig, RejectsMissingOrMalformedTargetCount)
{
  YAML::Node semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_02"].remove("target_count");
  const auto missing = write_yaml(semantics, "missing_target_count.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(missing, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);

  semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_02"]["target_count"] = -1;
  const auto negative = write_yaml(semantics, "negative_target_count.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(negative, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);

  semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_02"]["target_count"] = 2.5;
  const auto fractional = write_yaml(semantics, "fractional_target_count.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(fractional, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);

  semantics = YAML::LoadFile(RESTOCKER_TEST_LANE_SEMANTICS);
  semantics["lanes"]["lane_02"]["target_count"] = 4294967296;
  const auto overflow = write_yaml(semantics, "overflow_target_count.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(overflow, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);
}

TEST(LaneConfig, RoundTripsTheDesiredPolicyDocumentItWrites)
{
  // Deliberately not in lane-ID order: the document loads as an ID-sorted map, exactly like the
  // baseline loader reads it, and the round trip must not depend on the writer's iteration order.
  const std::vector<LanePolicy> policies{
    LanePolicy{LaneId{"lane_02"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 3},
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, std::nullopt, 0},
    LanePolicy{LaneId{"lane_03"}, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 9}};
  const auto path = std::filesystem::path(::testing::TempDir()) / "lane_policy_roundtrip.yaml";
  std::filesystem::remove(path);
  save_lane_policy_state(path, policies);
  const auto loaded = load_lane_policy_state(path);
  ASSERT_EQ(loaded.size(), policies.size());
  EXPECT_EQ(loaded[0], policies[1]);
  EXPECT_EQ(loaded[1], policies[0]);
  EXPECT_EQ(loaded[2], policies[2]);
  // Zero is a declared "want none", and an absent SKU is a wildcard lane; both must survive
  // the round trip rather than being filled in with a default.
  EXPECT_EQ(loaded[0].target_count, 0U);
  EXPECT_FALSE(loaded[0].expected_sku.has_value());
}

// Validation happens before any I/O: a caller defect must not be able to half-write the
// durable document on its way to being rejected.
TEST(LaneConfig, RefusesInvalidPolicyRowsBeforeTouchingTheFilesystem)
{
  const auto path = std::filesystem::path(::testing::TempDir()) / "lane_policy_invalid_row.yaml";
  std::filesystem::remove(path);
  EXPECT_THROW(
    save_lane_policy_state(path, {}), std::invalid_argument);
  EXPECT_THROW(
    save_lane_policy_state(
      path, {LanePolicy{LaneId{""}, ProductClass::Can, std::nullopt, 1}}),
    std::invalid_argument);
  EXPECT_THROW(
    save_lane_policy_state(
      path,
      {LanePolicy{LaneId{"lane_01"}, ProductClass::Unknown, std::nullopt, 1}}),
    std::invalid_argument);
  EXPECT_THROW(
    save_lane_policy_state(
      path, {LanePolicy{LaneId{"lane_01"}, ProductClass::Can, std::string(""), 1}}),
    std::invalid_argument);
  EXPECT_THROW(
    save_lane_policy_state(
      path,
      {LanePolicy{LaneId{"lane_01"}, ProductClass::Can, std::nullopt, 1},
        LanePolicy{LaneId{"lane_01"}, ProductClass::Can, std::nullopt, 2}}),
    std::invalid_argument);
  EXPECT_FALSE(std::filesystem::exists(path));
}

// A write whose temporary file cannot even be created (here: the parent path is a regular file,
// so open(2) fails with ENOTDIR) throws, and no partial document is left anywhere.
TEST(LaneConfig, FailedPolicyWriteThrowsWithoutLeavingPartialFiles)
{
  const auto blocker = std::filesystem::path(::testing::TempDir()) / "lane_policy_blocker";
  std::filesystem::remove(blocker);
  {
    std::ofstream stream(blocker);
    stream << "not a directory\n";
  }
  const auto path = blocker / "lane_policy.yaml";
  const std::vector<LanePolicy> policies{
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 6}};
  EXPECT_THROW(save_lane_policy_state(path, policies), std::runtime_error);
  EXPECT_FALSE(std::filesystem::exists(path));
  EXPECT_FALSE(std::filesystem::exists(path.string() + ".tmp"));
  std::filesystem::remove(blocker);
}

TEST(LaneConfig, OverlaysPersistedIntentOntoTheShippedBaseline)
{
  auto definitions = load_lane_definitions(
    RESTOCKER_TEST_LANE_SEMANTICS, RESTOCKER_TEST_WORKCELL_GEOMETRY);
  const std::vector<LanePolicy> resumed{
    LanePolicy{LaneId{"lane_01"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 2},
    LanePolicy{LaneId{"lane_02"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 6},
    LanePolicy{LaneId{"lane_03"}, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 6},
    LanePolicy{LaneId{"lane_04"}, ProductClass::Can, "SIM-CAN-STD", 6},
    LanePolicy{LaneId{"lane_05"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 6},
    LanePolicy{LaneId{"lane_06"}, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 6}};
  apply_lane_policy_state(definitions, resumed);
  EXPECT_EQ(definitions[0].expected_product_class, ProductClass::SmallBottle);
  EXPECT_EQ(definitions[0].target_count, 2U);
  // Depth still comes from the geometry file; the overlay changes policy only.
  EXPECT_DOUBLE_EQ(definitions[0].depth_m, 0.85);
  EXPECT_EQ(definitions[3].expected_sku, "SIM-CAN-STD");
  EXPECT_EQ(definitions[3].target_count, 6U);
}

// The persisted document must describe exactly the configured shelf: half of the owner's table
// applied to the other half of the shelf is not a policy this system will run.
TEST(LaneConfig, RefusesAPolicyStateThatDoesNotMatchTheConfiguredShelf)
{
  auto definitions = load_lane_definitions(
    RESTOCKER_TEST_LANE_SEMANTICS, RESTOCKER_TEST_WORKCELL_GEOMETRY);
  auto five_lanes = std::vector<LanePolicy>{
    LanePolicy{LaneId{"lane_01"}, ProductClass::Can, "SIM-CAN-STD", 6},
    LanePolicy{LaneId{"lane_02"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 6},
    LanePolicy{LaneId{"lane_03"}, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 6},
    LanePolicy{LaneId{"lane_04"}, ProductClass::Can, "SIM-CAN-STD", 6},
    LanePolicy{LaneId{"lane_05"}, ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 6}};
  EXPECT_THROW(apply_lane_policy_state(definitions, five_lanes), std::invalid_argument);

  auto wrong_lane = five_lanes;
  wrong_lane.push_back(
    LanePolicy{LaneId{"lane_07"}, ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 6});
  const auto untouched = definitions;
  EXPECT_THROW(apply_lane_policy_state(definitions, wrong_lane), std::invalid_argument);
  EXPECT_EQ(definitions[0].expected_product_class, untouched[0].expected_product_class);
  EXPECT_EQ(definitions[0].target_count, untouched[0].target_count);
}

TEST(LaneConfig, RejectsInvalidDescriptionDepthWithoutFallback)
{
  YAML::Node geometry = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  geometry["lanes"]["lane_01"]["usable_depth_m"] = -0.1;
  const auto invalid_geometry = write_yaml(geometry, "invalid_depth.yaml");
  EXPECT_THROW(
    static_cast<void>(load_lane_definitions(
      RESTOCKER_TEST_LANE_SEMANTICS, invalid_geometry)),
    std::invalid_argument);
}

// What one more of each catalogued product costs a lane, derived rather than declared. These are
// the numbers every destination predicate is now expressed in, so their derivation is checked
// against the two files it comes from rather than restated as constants.
TEST(LaneConfig, DerivesTheLaneDepthEachCataloguedProductCosts)
{
  const auto profiles = load_product_lane_profiles(
    RESTOCKER_TEST_PRODUCT_CATALOG, RESTOCKER_TEST_WORKCELL_GEOMETRY);
  // Counted from the catalogue rather than restated, because the catalogue grows: `can.citrus`
  // was added for the same-shape recognition work after this test was written, and a hardcoded
  // three failed on a tree where nothing about lane depth had changed.
  const YAML::Node catalog = YAML::LoadFile(RESTOCKER_TEST_PRODUCT_CATALOG);
  ASSERT_EQ(profiles.size(), catalog["geometries"].size());
  const double incline = 4.0 * std::acos(-1.0) / 180.0;
  const double entry = 0.045;
  struct Case
  {
    ProductClass product_class;
    const char * sku;
    double radius_m;
  };
  const Case cases[] = {
    {ProductClass::Can, "SIM-CAN-STD", 0.033},
    // Dimensionally identical to the standard can by construction (it exists so that two SKUs
    // share a shape and nothing can tell them apart geometrically), so it must derive the same
    // depth, and this case is what notices if the catalogue's two cans ever drift apart.
    {ProductClass::Can, "SIM-CAN-CITRUS", 0.033},
    {ProductClass::SmallBottle, "SIM-BOTTLE-SMALL", 0.034},
    {ProductClass::LargeBottle, "SIM-BOTTLE-LARGE", 0.045},
  };
  for (const Case & item : cases) {
    const auto profile =
      resolve_product_lane_profile(profiles, item.product_class, std::string(item.sku));
    ASSERT_TRUE(profile.has_value()) << item.sku;
    // The room one more needs: its own diameter, plus the clear depth the release leaves behind
    // its rear face.
    EXPECT_DOUBLE_EQ(profile->required_rear_depth_m, 2.0 * item.radius_m + entry) << item.sku;
    // The depth one more adds to the settled column: a diameter along the bed, foreshortened on
    // the lane's horizontal depth axis by the incline.
    EXPECT_DOUBLE_EQ(profile->column_pitch_m, 2.0 * item.radius_m * std::cos(incline))
      << item.sku;
    // The pitch is smaller than the room, and smaller than a diameter; if either were the
    // other way round the two predicates would contradict each other on the lane's last product.
    EXPECT_LT(profile->column_pitch_m, profile->required_rear_depth_m) << item.sku;
    EXPECT_LT(profile->column_pitch_m, 2.0 * item.radius_m) << item.sku;
  }
  // The catalogue's entries are class fallbacks, so a product whose SKU the catalogue does not
  // carry still resolves by class, and a class the catalogue has no envelope for does not.
  EXPECT_TRUE(
    resolve_product_lane_profile(profiles, ProductClass::Can, "SOME-OTHER-CAN").has_value());
  EXPECT_FALSE(
    resolve_product_lane_profile(profiles, ProductClass::Unknown, std::nullopt).has_value());
  EXPECT_FALSE(resolve_product_lane_profile({}, ProductClass::Can, std::nullopt).has_value());
}

TEST(LaneConfig, RefusesAShelfItCannotDeriveAPitchFrom)
{
  YAML::Node geometry = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  geometry["lanes"]["lane_01"].remove("insert_entry_clearance_m");
  const auto no_clearance = write_yaml(geometry, "no_entry_clearance.yaml");
  EXPECT_THROW(
    static_cast<void>(load_product_lane_profiles(RESTOCKER_TEST_PRODUCT_CATALOG, no_clearance)),
    std::invalid_argument);

  geometry = YAML::LoadFile(RESTOCKER_TEST_WORKCELL_GEOMETRY);
  geometry["shelf"]["lane_incline_deg"] = 60.0;
  const auto not_a_shelf = write_yaml(geometry, "steep_incline_profiles.yaml");
  EXPECT_THROW(
    static_cast<void>(load_product_lane_profiles(RESTOCKER_TEST_PRODUCT_CATALOG, not_a_shelf)),
    std::invalid_argument);

  YAML::Node catalog = YAML::LoadFile(RESTOCKER_TEST_PRODUCT_CATALOG);
  catalog["geometries"][0]["shape"]["radius_m"] = -0.01;
  const auto negative_radius = write_yaml(catalog, "negative_radius.yaml");
  EXPECT_THROW(
    static_cast<void>(
      load_product_lane_profiles(negative_radius, RESTOCKER_TEST_WORKCELL_GEOMETRY)),
    std::invalid_argument);
}

}  // namespace
}  // namespace restocker_world_state
