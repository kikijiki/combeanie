// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include <filesystem>
#include <fstream>
#include <string>

#include "restocker_task_executor/gripper_geometry.hpp"

namespace restocker_task_executor
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

TEST(GripperGeometry, LoadsDescriptionOwnedJawAndToolDatum)
{
  const auto result = load_gripper_staging_geometry(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  ASSERT_TRUE(result) << result.error().detail;
  EXPECT_DOUBLE_EQ(result.value().jaw.inner_gap_at_zero_m, 0.042);
  EXPECT_DOUBLE_EQ(result.value().jaw.joint_lower_m, 0.0);
  EXPECT_DOUBLE_EQ(result.value().jaw.joint_upper_m, 0.035);
  EXPECT_DOUBLE_EQ(result.value().hold_clearance_per_side_m, 0.0025);
  EXPECT_DOUBLE_EQ(result.value().open_clearance_per_side_m, 0.005);
  // The release aperture stays strictly inside the jaw stroke asserted above; parking the jaws
  // on their joint limit during an arm traverse would push them past it.
  EXPECT_DOUBLE_EQ(result.value().maximum_open_target_m, 0.032);
  EXPECT_LT(result.value().maximum_open_target_m, result.value().jaw.joint_upper_m);
  EXPECT_TRUE(
    result.value().tool0_from_grasp_center.translation().isApprox(
      Eigen::Vector3d(0.0, 0.0, 0.14), 1.0e-12));
  EXPECT_TRUE(
    result.value().tool0_from_grasp_center.linear().isApprox(
      Eigen::Matrix3d::Identity(), 1.0e-12));
}

TEST(GripperGeometry, RejectsSchemaAndAsymmetricJawGeometry)
{
  YAML::Node root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["schema_version"] = 2;
  auto result = load_gripper_staging_geometry(write_yaml(root, "gripper_schema.yaml"));
  ASSERT_FALSE(result);

  root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["finger"]["right"]["origin_xyz_m"][1] = -0.031;
  result = load_gripper_staging_geometry(write_yaml(root, "gripper_asymmetry.yaml"));
  ASSERT_FALSE(result);

  root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["finger"]["right"]["axis"][1] = 1.0;
  result = load_gripper_staging_geometry(write_yaml(root, "gripper_axis.yaml"));
  ASSERT_FALSE(result);
}

TEST(GripperGeometry, RejectsBoundsAndClearanceContradictions)
{
  YAML::Node root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["finger"]["joint"]["upper_m"] = 0.0;
  auto result = load_gripper_staging_geometry(write_yaml(root, "gripper_bounds.yaml"));
  ASSERT_FALSE(result);

  root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["attachment"]["open_clearance_per_side_m"] = 0.001;
  result = load_gripper_staging_geometry(write_yaml(root, "gripper_clearance.yaml"));
  ASSERT_FALSE(result);

  root = YAML::LoadFile(RESTOCKER_TEST_GRIPPER_GEOMETRY);
  root["attachment"]["open_target_m"] = 0.036;
  result = load_gripper_staging_geometry(write_yaml(root, "gripper_open_target.yaml"));
  ASSERT_FALSE(result);
}

}  // namespace
}  // namespace restocker_task_executor
