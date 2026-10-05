// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

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
  // Each is the inner face at zero less the product radius less hold_clearance_per_side_m, so all
  // three move together with that clearance (0.013 / 0.014 / 0.025 at 0.001; raised to 0.0025 so
  // a padded finger clears a lane's column projection).
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_can_01").geometry.hold_joint_target_m,
    0.0145, 1.0e-12);
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_small_bottle_01").geometry.hold_joint_target_m,
    0.0155, 1.0e-12);
  EXPECT_NEAR(
    config.products_by_source_id.at("sim:stock_large_bottle_01").geometry.hold_joint_target_m,
    0.0265, 1.0e-12);
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

}  // namespace
}  // namespace restocker_gazebo
