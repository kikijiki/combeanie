// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/scene_geometry.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr char kWorkcellUrdf[] =
  R"(
<robot name="fixture">
  <link name="shelf">
    <collision name="support">
      <origin xyz="0 0.45 -0.02" rpy="0 0 0"/>
      <geometry><box size="2.4 0.9 0.04"/></geometry>
    </collision>
    <collision name="retainer">
      <origin xyz="0 0.885 0.045" rpy="0 0 0"/>
      <geometry><box size="2.4 0.03 0.09"/></geometry>
    </collision>
  </link>
  <link name="camera"/>
  <joint name="camera_mount" type="fixed">
    <parent link="shelf"/><child link="camera"/>
    <origin xyz="0 0.42 0.95" rpy="0 0 0"/>
  </joint>
  <link name="lens">
    <collision name="housing"><geometry><sphere radius="0.05"/></geometry></collision>
  </link>
  <joint name="lens_mount" type="fixed">
    <parent link="camera"/><child link="lens"/>
    <origin xyz="0.1 0 0" rpy="0 0 1.5707963267948966"/>
  </joint>
</robot>)";

std::filesystem::path write_fixture(const std::string & contents, const std::string & name)
{
  const auto path = std::filesystem::temp_directory_path() / name;
  std::ofstream stream(path);
  stream << contents;
  stream.close();
  return path;
}

TEST(SceneGeometryCatalogTest, LoadsPinnedCatalogAndResolvesExactThenFallback)
{
  auto catalog = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
  ASSERT_TRUE(catalog) << catalog.error().detail;
  EXPECT_EQ(catalog.value().entries().size(), 4U);

  auto exact = catalog.value().resolve(CatalogProductClass::Can, "SIM-CAN-STD");
  ASSERT_TRUE(exact) << exact.error().detail;
  EXPECT_EQ(exact.value().geometry_key, "can.standard");
  ASSERT_EQ(exact.value().primitive.dimensions.size(), 2U);
  EXPECT_DOUBLE_EQ(exact.value().primitive.dimensions[0], 0.122);
  EXPECT_DOUBLE_EQ(exact.value().primitive.dimensions[1], 0.033);

  // Second can: one class, two SKUs, one shape. Resolution must return the entry the SKU names,
  // not just one of the right size, because the returned geometry key identifies the product in
  // the planning scene, the attachment plugin and the lane evidence.
  auto sibling = catalog.value().resolve(CatalogProductClass::Can, "SIM-CAN-CITRUS");
  ASSERT_TRUE(sibling) << sibling.error().detail;
  EXPECT_EQ(sibling.value().geometry_key, "can.citrus");
  EXPECT_EQ(sibling.value().primitive.dimensions, exact.value().primitive.dimensions);

  auto fallback = catalog.value().resolve(CatalogProductClass::Can, "UNKNOWN-SKU");
  ASSERT_TRUE(fallback) << fallback.error().detail;
  EXPECT_EQ(fallback.value().geometry_key, "can.standard");
}

TEST(SceneGeometryCatalogTest, AcceptsTwoSkusOfOneClassWhenExactlyOneIsTheFallback)
{
  // The catalog rule is one fallback per class, not one entry per class. Asserted on a fixture
  // so that removing the second can from the shipped catalog does not drop the check.
  const auto shared_class = write_fixture(
    R"(schema_version: 1
geometries:
  - geometry_key: can.one
    product_class: can
    sku: SIM-CAN-ONE
    class_fallback: true
    shape: {type: cylinder, radius_m: 0.033, height_m: 0.122}
  - geometry_key: can.two
    product_class: can
    sku: SIM-CAN-TWO
    class_fallback: false
    shape: {type: cylinder, radius_m: 0.033, height_m: 0.122}
)",
    "restocker_shared_class.yaml");
  auto catalog = ProductCollisionCatalog::load(shared_class);
  ASSERT_TRUE(catalog) << catalog.error().detail;
  EXPECT_EQ(catalog.value().entries().size(), 2U);

  auto one = catalog.value().resolve(CatalogProductClass::Can, "SIM-CAN-ONE");
  ASSERT_TRUE(one) << one.error().detail;
  EXPECT_EQ(one.value().geometry_key, "can.one");
  auto two = catalog.value().resolve(CatalogProductClass::Can, "SIM-CAN-TWO");
  ASSERT_TRUE(two) << two.error().detail;
  EXPECT_EQ(two.value().geometry_key, "can.two");
}

TEST(SceneGeometryCatalogTest, RejectsAmbiguousAndInvalidCatalogs)
{
  const auto duplicate_fallback = write_fixture(
    R"(schema_version: 1
geometries:
  - geometry_key: one
    product_class: can
    class_fallback: true
    shape: {type: cylinder, radius_m: 0.03, height_m: 0.12}
  - geometry_key: two
    product_class: can
    class_fallback: true
    shape: {type: cylinder, radius_m: 0.04, height_m: 0.13}
)",
    "restocker_duplicate_fallback.yaml");
  auto duplicate_result = ProductCollisionCatalog::load(duplicate_fallback);
  ASSERT_FALSE(duplicate_result);
  EXPECT_EQ(
    duplicate_result.error().code, SceneGeometryErrorCode::InvalidConfiguration);

  const auto unsupported = write_fixture(
    R"(schema_version: 1
geometries:
  - geometry_key: box
    product_class: can
    class_fallback: true
    shape: {type: box, radius_m: 0.03, height_m: 0.12}
)",
    "restocker_unsupported_geometry.yaml");
  auto unsupported_result = ProductCollisionCatalog::load(unsupported);
  ASSERT_FALSE(unsupported_result);
  EXPECT_EQ(unsupported_result.error().code, SceneGeometryErrorCode::UnsupportedGeometry);
}

TEST(SceneGeometryProductTest, BuildsStableMoveItCylinderObject)
{
  auto catalog = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
  ASSERT_TRUE(catalog) << catalog.error().detail;
  auto geometry = catalog.value().resolve(CatalogProductClass::SmallBottle, std::nullopt);
  ASSERT_TRUE(geometry) << geometry.error().detail;

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(0.4, -0.8, 0.67);
  pose.linear() = Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
  auto object = make_product_collision_object(17, "world", pose, geometry.value());
  ASSERT_TRUE(object) << object.error().detail;
  EXPECT_EQ(object.value().id, "restocker/object/17");
  EXPECT_EQ(object.value().header.frame_id, "world");
  EXPECT_EQ(object.value().operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(object.value().primitives.size(), 1U);
  EXPECT_EQ(object.value().primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(object.value().primitives[0].dimensions[0], 0.2);
  EXPECT_DOUBLE_EQ(object.value().primitives[0].dimensions[1], 0.034);
  EXPECT_DOUBLE_EQ(object.value().pose.position.x, 0.4);
}

TEST(SceneGeometryProductTest, RejectsInvalidIdentityAndPose)
{
  ProductCollisionGeometry geometry;
  geometry.primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  geometry.primitive.dimensions = {0.2, 0.03};
  auto zero_id = make_product_collision_object(
    0, "world", Eigen::Isometry3d::Identity(), geometry);
  EXPECT_FALSE(zero_id);

  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().x() = std::numeric_limits<double>::quiet_NaN();
  auto invalid_pose = make_product_collision_object(1, "world", invalid, geometry);
  EXPECT_FALSE(invalid_pose);
}

// Card 050: the declared-product loader and its scene seed.

TEST(SceneGeometryScenarioTest, LoadsDeclaredProductsAtTheirWorldFrameSpawnPoses)
{
  const auto path = write_fixture(
    R"(schema_version: 1
workcell_pose: [0.0, 0.55, 0.75, 0.0, 0.0, 0.0]
products:
  - model_name: stock_small_bottle_01
    source_object_id: sim:stock_small_bottle_01
    geometry_key: bottle.small.standard
    spawn_pose: [-0.46, -0.74, 0.67, 0.0, 0.0, 0.0]
  - model_name: stock_can_02
    source_object_id: sim:stock_can_02
    geometry_key: can.standard
    spawn_pose: [-0.04, -0.88, 0.631, 0.0, 0.0, 0.05]
)",
    "card050_declared_scenario.yaml");
  auto declared = load_scenario_products(path);
  ASSERT_TRUE(declared) << declared.error().detail;
  ASSERT_EQ(declared.value().size(), 2U);
  EXPECT_EQ(declared.value()[0].source_object_id, "sim:stock_small_bottle_01");
  EXPECT_EQ(declared.value()[0].geometry_key, "bottle.small.standard");
  EXPECT_NEAR(declared.value()[0].planning_from_product.translation().x(), -0.46, 1.0e-12);
  EXPECT_NEAR(declared.value()[0].planning_from_product.translation().y(), -0.74, 1.0e-12);
  EXPECT_NEAR(declared.value()[0].planning_from_product.translation().z(), 0.67, 1.0e-12);
  EXPECT_TRUE(
    declared.value()[0].planning_from_product.linear().isApprox(Eigen::Matrix3d::Identity()));
  EXPECT_EQ(declared.value()[1].source_object_id, "sim:stock_can_02");
  EXPECT_NEAR(declared.value()[1].planning_from_product.translation().z(), 0.631, 1.0e-12);
  const Eigen::Quaterniond yaw(declared.value()[1].planning_from_product.linear());
  EXPECT_NEAR(yaw.z(), std::sin(0.025), 1.0e-9);
  EXPECT_NEAR(yaw.w(), std::cos(0.025), 1.0e-9);
}

TEST(SceneGeometryScenarioTest, RejectsDuplicateAndMalformedDeclaredProducts)
{
  const auto duplicate = write_fixture(
    R"(schema_version: 1
products:
  - source_object_id: sim:twice
    geometry_key: can.standard
    spawn_pose: [0.0, 0.0, 0.6, 0.0, 0.0, 0.0]
  - source_object_id: sim:twice
    geometry_key: can.standard
    spawn_pose: [0.1, 0.0, 0.6, 0.0, 0.0, 0.0]
)",
    "card050_duplicate_scenario.yaml");
  EXPECT_FALSE(load_scenario_products(duplicate));

  const auto short_pose = write_fixture(
    R"(schema_version: 1
products:
  - source_object_id: sim:short
    geometry_key: can.standard
    spawn_pose: [0.0, 0.0, 0.6, 0.0, 1.0]
)",
    "card050_short_pose_scenario.yaml");
  EXPECT_FALSE(load_scenario_products(short_pose));

  const auto wrong_schema = write_fixture(
    R"(schema_version: 2
products:
  - source_object_id: sim:future
    geometry_key: can.standard
    spawn_pose: [0.0, 0.0, 0.6, 0.0, 0.0, 0.0]
)",
    "card050_wrong_schema_scenario.yaml");
  EXPECT_FALSE(load_scenario_products(wrong_schema));
}

TEST(SceneGeometryProductTest, BuildsDeclaredSeedsUnderTheDeclaredNamespace)
{
  auto catalog = ProductCollisionCatalog::load(RESTOCKER_TEST_PRODUCT_CATALOG);
  ASSERT_TRUE(catalog) << catalog.error().detail;
  const auto & geometry = catalog.value().entries().at("can.standard");

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(-0.04, -0.88, 0.631);
  auto seed = make_declared_product_collision_object(
    "sim:stock_can_02", "world", pose, geometry);
  ASSERT_TRUE(seed) << seed.error().detail;
  EXPECT_EQ(seed.value().id, "restocker/declared/sim:stock_can_02");
  EXPECT_EQ(seed.value().header.frame_id, "world");
  EXPECT_EQ(seed.value().operation, moveit_msgs::msg::CollisionObject::ADD);
  ASSERT_EQ(seed.value().primitives.size(), 1U);
  EXPECT_EQ(seed.value().primitives[0].type, shape_msgs::msg::SolidPrimitive::CYLINDER);
  EXPECT_DOUBLE_EQ(seed.value().primitives[0].dimensions[0], 0.122);
  EXPECT_DOUBLE_EQ(seed.value().primitives[0].dimensions[1], 0.033);
  EXPECT_DOUBLE_EQ(seed.value().pose.position.x, -0.04);

  auto empty_source = make_declared_product_collision_object("", "world", pose, geometry);
  EXPECT_FALSE(empty_source);
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.translation().y() = std::numeric_limits<double>::quiet_NaN();
  auto invalid_pose = make_declared_product_collision_object("sim:x", "world", invalid, geometry);
  EXPECT_FALSE(invalid_pose);
}

TEST(SceneGeometryWorkcellTest, ExtractsNamedCollisionsAndComposesFixedTransforms)
{
  Eigen::Isometry3d world_from_shelf = Eigen::Isometry3d::Identity();
  world_from_shelf.translation() = Eigen::Vector3d(0.0, 0.55, 0.75);
  auto objects = extract_workcell_collision_objects(
    kWorkcellUrdf, "world", world_from_shelf);
  ASSERT_TRUE(objects) << objects.error().detail;
  ASSERT_EQ(objects.value().size(), 3U);
  EXPECT_EQ(
    objects.value()[0].id, "restocker/workcell/lens/housing");
  EXPECT_EQ(
    objects.value()[1].id, "restocker/workcell/shelf/retainer");
  EXPECT_EQ(
    objects.value()[2].id, "restocker/workcell/shelf/support");

  const auto & housing = objects.value()[0];
  EXPECT_NEAR(housing.pose.position.x, 0.1, 1.0e-12);
  EXPECT_NEAR(housing.pose.position.y, 0.97, 1.0e-12);
  EXPECT_NEAR(housing.pose.position.z, 1.70, 1.0e-12);
  EXPECT_EQ(housing.primitives[0].type, shape_msgs::msg::SolidPrimitive::SPHERE);
}

TEST(SceneGeometryWorkcellTest, RejectsMovingUnnamedAndMeshGeometry)
{
  const std::vector<std::string> invalid_descriptions{
    R"(<robot name="moving"><link name="a"/><link name="b"><collision name="box"><geometry><box size="1 1 1"/></geometry></collision></link><joint name="moving" type="prismatic"><parent link="a"/><child link="b"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>)",
    R"(<robot name="unnamed"><link name="root"><collision><geometry><box size="1 1 1"/></geometry></collision></link></robot>)",
    R"(<robot name="mesh"><link name="root"><collision name="mesh"><geometry><mesh filename="package://fixture/mesh.stl"/></geometry></collision></link></robot>)",
  };
  for (const auto & description : invalid_descriptions) {
    auto result = extract_workcell_collision_objects(
      description, "world", Eigen::Isometry3d::Identity());
    EXPECT_FALSE(result);
  }
}

}  // namespace
}  // namespace restocker_task_executor
