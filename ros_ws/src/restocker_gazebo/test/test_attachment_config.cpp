// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#include "restocker_gazebo/attachment_config.hpp"

namespace restocker_gazebo
{
namespace
{

AttachmentConfigResult load(const std::string & boundary, const std::string & scenario)
{
  return load_attachment_boundary_config(
    boundary, RESTOCKER_TEST_GRIPPER_GEOMETRY, RESTOCKER_TEST_PRODUCT_CATALOG, scenario);
}

// Load the shipped jaw policy with one substitution applied, so a single-key rejection can be
// exercised without maintaining a second full copy of the file. The hold mode lives there,
// beside the aperture it decides, because that is the file the executor reads too.
AttachmentConfigResult load_rewritten(
  const std::string & from, const std::string & to, const std::string & scenario)
{
  std::ifstream input(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  EXPECT_TRUE(input.good());
  const std::string source{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>{}};
  const auto start = source.find(from);
  EXPECT_NE(start, std::string::npos);
  const auto temp = std::filesystem::temp_directory_path() / "restocker_hold_mode_gripper.yaml";
  {
    std::ofstream output(temp);
    output << source.substr(0, start) << to << source.substr(start + from.size());
  }
  // Passed as the *gripper* path, not the boundary: that is the file the key lives in, and the
  // shipped boundary stays the real one for every other key this loader validates.
  const auto result = load_attachment_boundary_config(
    RESTOCKER_TEST_ATTACHMENT_BOUNDARY, temp.string(), RESTOCKER_TEST_PRODUCT_CATALOG, scenario);
  std::error_code ignored;
  std::filesystem::remove(temp, ignored);
  return result;
}

TEST(AttachmentConfig, ProjectsSharedDescriptionAndScenarioIntoImmutableAllowlist)
{
  const auto result = load(RESTOCKER_TEST_ATTACHMENT_BOUNDARY, RESTOCKER_TEST_SCENARIO);
  ASSERT_TRUE(result) << result.error().detail;
  const auto & config = result.value();
  EXPECT_EQ(config.robot_model_name, "restocker");
  EXPECT_EQ(config.parent_link, "gripper");
  EXPECT_EQ(config.products_by_source_id.size(), 3U);
  EXPECT_EQ(config.source_id_by_model_name.at("stock_can_01"), "sim:stock_can_01");
  EXPECT_EQ(config.journal_capacity, 64U);
  EXPECT_EQ(config.detach_attempt_reserve, 3U);
  EXPECT_EQ(config.required_verification_ticks, 3U);
  // Pins the shipped hold mode, which is the default the whole stack runs under: weld, because the
  // physical hold passes the end-to-end acceptance run only about 40 % of the time (Card 057).
  EXPECT_EQ(config.hold_mode, AttachmentHoldMode::kWeld);
  EXPECT_EQ(hold_mode_name(config.hold_mode), "weld");
  EXPECT_TRUE(
    config.gripper_from_grasp_center.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.14), 1.0e-12));
  EXPECT_TRUE(
    config.gripper.palm_bounds_in_grasp_center.min().isApprox(
      Eigen::Vector3d(-0.08, -0.055, -0.14), 1.0e-12));
  EXPECT_TRUE(
    config.gripper.palm_bounds_in_grasp_center.max().isApprox(
      Eigen::Vector3d(0.08, 0.055, -0.07), 1.0e-12));
  EXPECT_DOUBLE_EQ(config.gripper.left_inner_face_at_zero_m, 0.021);
  EXPECT_DOUBLE_EQ(config.gripper.right_inner_face_at_zero_m, -0.021);
  // Weld mode: one hold clearance short of the product on each side.
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_can_01").geometry.hold_joint_target_m,
    0.0145, 1.0e-12);
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_small_bottle_01").geometry.hold_joint_target_m,
    0.0155, 1.0e-12);
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_large_bottle_01").geometry.hold_joint_target_m,
    0.0265, 1.0e-12);
  // The capture ceiling is measured from the clearance-based aperture (hold target plus the
  // hold clearance and the interference), so it is the weld-mode value 0.017 and does not follow
  // how hard the grip squeezes.
  EXPECT_NEAR(
    fidelity_translation_ceiling(
      config.gripper, config.products_by_source_id.at("sim:stock_can_01").geometry),
    0.0170, 1e-12);
}

// A box (CMB-119) validates against the catalogue but is not attachable yet: it gets no allowlist
// entry, so an attach request for it is refused as an unknown source id.
TEST(AttachmentConfig, ABoxScenarioProductIsValidatedButNeverAttachable)
{
  std::ifstream input(RESTOCKER_TEST_SCENARIO);
  ASSERT_TRUE(input.good());
  std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>{}};
  const std::string anchor = "    spawn_pose: [0.42, -0.80, 0.715, 0.0, 0.0, 0.0]\n";
  const auto at = text.find(anchor);
  ASSERT_NE(at, std::string::npos);
  text.insert(
    at + anchor.size(),
    "  - model_name: stock_box_01\n    source_object_id: sim:stock_box_01\n"
    "    geometry_key: box.gen019\n    product_class: box\n    sku: GEN-019\n"
    "    mass_kg: 1.03\n    rgba: [1.0, 1.0, 1.0, 1.0]\n    friction: 0.55\n"
    "    spawn_pose: [0.0, -0.9, 0.65, 0.0, 0.0, 0.0]\n");
  const auto path = std::filesystem::temp_directory_path() / "restocker_box_scenario.yaml";
  {
    std::ofstream output(path);
    output << text;
  }
  const auto result = load(RESTOCKER_TEST_ATTACHMENT_BOUNDARY, path.string());
  std::error_code ignored;
  std::filesystem::remove(path, ignored);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_EQ(result.value().products_by_source_id.size(), 3U);
  EXPECT_EQ(result.value().products_by_source_id.count("sim:stock_box_01"), 0U);
  // Neither map names it, so the adapter refuses an attach request as an unknown source id.
  EXPECT_EQ(result.value().source_id_by_model_name.count("stock_box_01"), 0U);
}

TEST(AttachmentConfig, RejectsUnsupportedSchemaAndUnknownCatalogGeometry)
{
  const auto schema = load(RESTOCKER_TEST_INVALID_BOUNDARY, RESTOCKER_TEST_SCENARIO);
  ASSERT_FALSE(schema);
  EXPECT_NE(schema.error().detail.find("schema_version 1"), std::string::npos);

  const auto unknown = load(
    RESTOCKER_TEST_ATTACHMENT_BOUNDARY, RESTOCKER_TEST_UNKNOWN_PRODUCT_SCENARIO);
  ASSERT_FALSE(unknown);
  EXPECT_NE(unknown.error().detail.find("unknown geometry_key"), std::string::npos);
}

TEST(AttachmentConfig, HoldModeIsRequiredAndRestrictedToTheTwoSpellings)
{
  // Everything downstream — whether an entity is created, which aperture is commanded, which
  // receipts a run carries, and which evidence may be cited — is decided by this key, so a
  // policy file that forgets it or misspells it must fail closed rather than inherit a mode.
  const auto missing = load_rewritten("hold_mode: weld\n", "", RESTOCKER_TEST_SCENARIO);
  ASSERT_FALSE(missing);
  EXPECT_NE(missing.error().detail.find("attachment.hold_mode is required"), std::string::npos);

  const auto unknown = load_rewritten(
    "hold_mode: weld", "hold_mode: glued", RESTOCKER_TEST_SCENARIO);
  ASSERT_FALSE(unknown);
  EXPECT_NE(unknown.error().detail.find("attachment.hold_mode must be"), std::string::npos);

  const auto physical = load_rewritten(
    "hold_mode: weld", "hold_mode: physical", RESTOCKER_TEST_SCENARIO);
  ASSERT_TRUE(physical) << physical.error().detail;
  EXPECT_EQ(physical.value().hold_mode, AttachmentHoldMode::kPhysical);
  // Physical mode: 0.5 * (diameter - 2 * hold_interference - zero_gap), the weld targets minus
  // hold_clearance + hold_interference per side, because the jaws are commanded past the surface.
  EXPECT_NEAR(
    physical.value().products_by_source_id.at("sim:stock_can_01").geometry.hold_joint_target_m,
    0.0093, 1.0e-12);
  EXPECT_NEAR(
    physical.value().products_by_source_id.at(
      "sim:stock_small_bottle_01").geometry.hold_joint_target_m,
    0.0103, 1.0e-12);
  EXPECT_NEAR(
    physical.value().products_by_source_id.at(
      "sim:stock_large_bottle_01").geometry.hold_joint_target_m,
    0.0213, 1.0e-12);
}

// The boundary-file twin of load_rewritten: one substitution in the shipped attachment boundary.
AttachmentConfigResult load_boundary_rewritten(const std::string & from, const std::string & to)
{
  std::ifstream input(RESTOCKER_TEST_ATTACHMENT_BOUNDARY);
  EXPECT_TRUE(input.good());
  const std::string source{
    std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>{}};
  const auto start = source.find(from);
  EXPECT_NE(start, std::string::npos);
  const auto temp = std::filesystem::temp_directory_path() / "restocker_swing_boundary.yaml";
  {
    std::ofstream output(temp);
    output << source.substr(0, start) << to << source.substr(start + from.size());
  }
  const auto result = load(temp.string(), RESTOCKER_TEST_SCENARIO);
  std::error_code ignored;
  std::filesystem::remove(temp, ignored);
  return result;
}

TEST(AttachmentConfig, HeldAllowancesAreRequiredAndOrderedSlipSwingTilt)
{
  // Spec M10 section 11 D5 amendments: the physical hold judges a pad's cumulative slip against
  // held_slip_allowance_m, the product's swing against held_swing_allowance_m and its lean off
  // the jaw axis against held_tilt_allowance_rad; a weld keeps post_attach_translation_drift_m.
  const auto shipped = load(RESTOCKER_TEST_ATTACHMENT_BOUNDARY, RESTOCKER_TEST_SCENARIO);
  ASSERT_TRUE(shipped) << shipped.error().detail;
  EXPECT_NEAR(shipped.value().post_attach_translation_drift_m, 0.001, 1.0e-12);
  EXPECT_NEAR(shipped.value().held_slip_allowance_m, 0.005, 1.0e-12);
  EXPECT_NEAR(shipped.value().held_swing_allowance_m, 0.02, 1.0e-12);
  EXPECT_NEAR(shipped.value().held_tilt_allowance_rad, 0.05, 1.0e-12);

  const auto missing_swing = load_boundary_rewritten("held_swing_allowance_m: 0.02", "");
  ASSERT_FALSE(missing_swing);
  EXPECT_NE(missing_swing.error().detail.find("held swing allowance"), std::string::npos);

  const auto missing_slip = load_boundary_rewritten("held_slip_allowance_m: 0.005", "");
  ASSERT_FALSE(missing_slip);
  EXPECT_NE(missing_slip.error().detail.find("held slip allowance"), std::string::npos);

  const auto missing_tilt = load_boundary_rewritten("held_tilt_allowance_rad: 0.05", "");
  ASSERT_FALSE(missing_tilt);
  EXPECT_NE(missing_tilt.error().detail.find("held tilt allowance"), std::string::npos);

  const auto slip_tighter_than_weld = load_boundary_rewritten(
    "held_slip_allowance_m: 0.005", "held_slip_allowance_m: 0.0005");
  ASSERT_FALSE(slip_tighter_than_weld);
  EXPECT_NE(slip_tighter_than_weld.error().detail.find("held slip allowance"), std::string::npos);

  const auto swing_tighter_than_slip = load_boundary_rewritten(
    "held_swing_allowance_m: 0.02", "held_swing_allowance_m: 0.004");
  ASSERT_FALSE(swing_tighter_than_slip);
  EXPECT_NE(
    swing_tighter_than_slip.error().detail.find("held swing allowance"), std::string::npos);

  const auto tilt_tighter_than_fidelity = load_boundary_rewritten(
    "held_tilt_allowance_rad: 0.05", "held_tilt_allowance_rad: 0.02");
  ASSERT_FALSE(tilt_tighter_than_fidelity);
  EXPECT_NE(
    tilt_tighter_than_fidelity.error().detail.find("held tilt allowance"), std::string::npos);
}

}  // namespace
}  // namespace restocker_gazebo
