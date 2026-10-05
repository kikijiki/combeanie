// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/scene_geometry.hpp"

#include <urdf/urdf/model.h>
#include <urdf_model/link.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <string_view>

#include <geometry_msgs/msg/pose.hpp>

namespace restocker_task_executor
{
namespace
{

template<typename T>
[[nodiscard]] SceneGeometryResult<T> failure(
  SceneGeometryErrorCode code, std::string detail)
{
  return SceneGeometryResult<T>::failure(SceneGeometryError{code, std::move(detail)});
}

[[nodiscard]] std::optional<CatalogProductClass> parse_product_class(
  const std::string & value)
{
  if (value == "can") {
    return CatalogProductClass::Can;
  }
  if (value == "small_bottle") {
    return CatalogProductClass::SmallBottle;
  }
  if (value == "large_bottle") {
    return CatalogProductClass::LargeBottle;
  }
  return std::nullopt;
}

[[nodiscard]] bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

[[nodiscard]] bool finite_transform(const Eigen::Isometry3d & transform)
{
  const auto & rotation = transform.linear();
  return transform.matrix().allFinite() && rotation.isUnitary(1.0e-6) &&
         std::abs(rotation.determinant() - 1.0) <= 1.0e-6;
}

[[nodiscard]] SceneGeometryResult<Eigen::Isometry3d> pose_from_urdf(
  const urdf::Pose & pose, std::string_view context)
{
  const auto & position = pose.position;
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double w = 0.0;
  pose.rotation.getQuaternion(x, y, z, w);
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
    !std::isfinite(position.z) || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(z) || !std::isfinite(w))
  {
    return failure<Eigen::Isometry3d>(
      SceneGeometryErrorCode::InvalidPose,
      std::string(context) + " contains a non-finite transform");
  }
  Eigen::Quaterniond rotation(w, x, y, z);
  if (std::abs(rotation.norm() - 1.0) > 1.0e-6) {
    return failure<Eigen::Isometry3d>(
      SceneGeometryErrorCode::InvalidPose,
      std::string(context) + " contains a non-unit quaternion");
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(position.x, position.y, position.z);
  transform.linear() = rotation.normalized().toRotationMatrix();
  return SceneGeometryResult<Eigen::Isometry3d>::success(transform);
}

[[nodiscard]] geometry_msgs::msg::Pose pose_to_message(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond rotation(transform.linear());
  pose.orientation.x = rotation.x();
  pose.orientation.y = rotation.y();
  pose.orientation.z = rotation.z();
  pose.orientation.w = rotation.w();
  return pose;
}

[[nodiscard]] SceneGeometryResult<shape_msgs::msg::SolidPrimitive> primitive_from_urdf(
  const urdf::GeometrySharedPtr & geometry, const std::string & collision_id)
{
  if (!geometry) {
    return failure<shape_msgs::msg::SolidPrimitive>(
      SceneGeometryErrorCode::InvalidDescription,
      "collision " + collision_id + " has no geometry");
  }

  shape_msgs::msg::SolidPrimitive primitive;
  switch (geometry->type) {
    case urdf::Geometry::BOX:
      {
        const auto box = std::dynamic_pointer_cast<urdf::Box>(geometry);
        if (!box || !finite_positive(box->dim.x) || !finite_positive(box->dim.y) ||
          !finite_positive(box->dim.z))
        {
          return failure<shape_msgs::msg::SolidPrimitive>(
            SceneGeometryErrorCode::InvalidDescription,
            "collision " + collision_id + " has invalid box dimensions");
        }
        primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
        primitive.dimensions = {box->dim.x, box->dim.y, box->dim.z};
        break;
      }
    case urdf::Geometry::CYLINDER:
      {
        const auto cylinder = std::dynamic_pointer_cast<urdf::Cylinder>(geometry);
        if (!cylinder || !finite_positive(cylinder->length) ||
          !finite_positive(cylinder->radius))
        {
          return failure<shape_msgs::msg::SolidPrimitive>(
            SceneGeometryErrorCode::InvalidDescription,
            "collision " + collision_id + " has invalid cylinder dimensions");
        }
        primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        primitive.dimensions = {cylinder->length, cylinder->radius};
        break;
      }
    case urdf::Geometry::SPHERE:
      {
        const auto sphere = std::dynamic_pointer_cast<urdf::Sphere>(geometry);
        if (!sphere || !finite_positive(sphere->radius)) {
          return failure<shape_msgs::msg::SolidPrimitive>(
            SceneGeometryErrorCode::InvalidDescription,
            "collision " + collision_id + " has an invalid sphere radius");
        }
        primitive.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        primitive.dimensions = {sphere->radius};
        break;
      }
    case urdf::Geometry::MESH:
      return failure<shape_msgs::msg::SolidPrimitive>(
        SceneGeometryErrorCode::UnsupportedGeometry,
        "collision " + collision_id + " uses unsupported mesh geometry");
    default:
      return failure<shape_msgs::msg::SolidPrimitive>(
        SceneGeometryErrorCode::UnsupportedGeometry,
        "collision " + collision_id + " has an unknown geometry type");
  }
  return SceneGeometryResult<shape_msgs::msg::SolidPrimitive>::success(std::move(primitive));
}

}  // namespace

SceneGeometryResult<ProductCollisionCatalog> ProductCollisionCatalog::load(
  const std::filesystem::path & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
      return failure<ProductCollisionCatalog>(
        SceneGeometryErrorCode::InvalidConfiguration,
        "product collision catalog requires schema_version 1");
    }
    const YAML::Node geometries = root["geometries"];
    if (!geometries.IsSequence() || geometries.size() == 0) {
      return failure<ProductCollisionCatalog>(
        SceneGeometryErrorCode::InvalidConfiguration,
        "product collision catalog must contain at least one geometry");
    }

    ProductCollisionCatalog catalog;
    std::set<CatalogProductClass> represented_classes;
    std::set<CatalogProductClass> fallback_classes;
    std::set<std::string> skus;
    for (const YAML::Node & node : geometries) {
      ProductCollisionGeometry entry;
      entry.geometry_key = node["geometry_key"].as<std::string>();
      const auto product_class = parse_product_class(node["product_class"].as<std::string>());
      if (entry.geometry_key.empty() || !product_class) {
        return failure<ProductCollisionCatalog>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "catalog geometry key and product class must be valid");
      }
      entry.product_class = *product_class;
      represented_classes.insert(entry.product_class);
      if (node["sku"] && !node["sku"].as<std::string>().empty()) {
        entry.sku = node["sku"].as<std::string>();
        if (!skus.insert(*entry.sku).second) {
          return failure<ProductCollisionCatalog>(
            SceneGeometryErrorCode::InvalidConfiguration,
            "catalog contains duplicate SKU: " + *entry.sku);
        }
      }
      entry.class_fallback = node["class_fallback"].as<bool>(false);
      if (entry.class_fallback && !fallback_classes.insert(entry.product_class).second) {
        return failure<ProductCollisionCatalog>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "catalog contains multiple fallbacks for one product class");
      }

      const YAML::Node shape = node["shape"];
      if (!shape || shape["type"].as<std::string>("") != "cylinder") {
        return failure<ProductCollisionCatalog>(
          SceneGeometryErrorCode::UnsupportedGeometry,
          "catalog geometry " + entry.geometry_key + " is not a supported cylinder");
      }
      const double radius = shape["radius_m"].as<double>();
      const double height = shape["height_m"].as<double>();
      if (!finite_positive(radius) || !finite_positive(height)) {
        return failure<ProductCollisionCatalog>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "catalog geometry " + entry.geometry_key + " has invalid dimensions");
      }
      entry.primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
      entry.primitive.dimensions = {height, radius};
      if (!catalog.entries_.emplace(entry.geometry_key, std::move(entry)).second) {
        return failure<ProductCollisionCatalog>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "catalog contains duplicate geometry_key");
      }
    }
    if (fallback_classes != represented_classes) {
      return failure<ProductCollisionCatalog>(
        SceneGeometryErrorCode::InvalidConfiguration,
        "every represented product class requires exactly one fallback");
    }
    return SceneGeometryResult<ProductCollisionCatalog>::success(std::move(catalog));
  } catch (const YAML::Exception & error) {
    return failure<ProductCollisionCatalog>(
      SceneGeometryErrorCode::InvalidConfiguration,
      "failed to parse product collision catalog: " + std::string(error.what()));
  }
}

SceneGeometryResult<ProductCollisionGeometry> ProductCollisionCatalog::resolve(
  CatalogProductClass product_class, const std::optional<std::string> & sku) const
{
  const ProductCollisionGeometry * fallback_entry = nullptr;
  for (const auto & [key, entry] : entries_) {
    static_cast<void>(key);
    if (entry.product_class != product_class) {
      continue;
    }
    if (sku && entry.sku == sku) {
      return SceneGeometryResult<ProductCollisionGeometry>::success(entry);
    }
    if (entry.class_fallback) {
      fallback_entry = &entry;
    }
  }
  if (fallback_entry) {
    return SceneGeometryResult<ProductCollisionGeometry>::success(*fallback_entry);
  }
  return failure<ProductCollisionGeometry>(
    SceneGeometryErrorCode::MissingGeometry,
    "no deterministic collision geometry resolves the product class and SKU");
}

SceneGeometryResult<moveit_msgs::msg::CollisionObject> make_product_collision_object(
  std::uint64_t object_id, const std::string & planning_frame,
  const Eigen::Isometry3d & pose_in_planning_frame,
  const ProductCollisionGeometry & geometry)
{
  if (object_id == 0 || planning_frame.empty() || !finite_transform(pose_in_planning_frame)) {
    return failure<moveit_msgs::msg::CollisionObject>(
      SceneGeometryErrorCode::InvalidPose,
      "product collision object requires a nonzero ID, frame, and rigid finite pose");
  }
  if (geometry.primitive.type != shape_msgs::msg::SolidPrimitive::CYLINDER ||
    geometry.primitive.dimensions.size() != 2 ||
    !finite_positive(geometry.primitive.dimensions[0]) ||
    !finite_positive(geometry.primitive.dimensions[1]))
  {
    return failure<moveit_msgs::msg::CollisionObject>(
      SceneGeometryErrorCode::InvalidConfiguration,
      "product collision geometry is not a valid cylinder");
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame;
  object.id = "restocker/object/" + std::to_string(object_id);
  object.pose = pose_to_message(pose_in_planning_frame);
  object.primitives.push_back(geometry.primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  object.primitive_poses.push_back(identity);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return SceneGeometryResult<moveit_msgs::msg::CollisionObject>::success(std::move(object));
}

SceneGeometryResult<moveit_msgs::msg::CollisionObject> make_declared_product_collision_object(
  const std::string & source_object_id, const std::string & planning_frame,
  const Eigen::Isometry3d & pose_in_planning_frame, const ProductCollisionGeometry & geometry)
{
  if (source_object_id.empty() || planning_frame.empty() ||
    !finite_transform(pose_in_planning_frame))
  {
    return failure<moveit_msgs::msg::CollisionObject>(
      SceneGeometryErrorCode::InvalidPose,
      "declared product collision object requires a source ID, frame, and rigid finite pose");
  }
  if (geometry.primitive.type != shape_msgs::msg::SolidPrimitive::CYLINDER ||
    geometry.primitive.dimensions.size() != 2 ||
    !finite_positive(geometry.primitive.dimensions[0]) ||
    !finite_positive(geometry.primitive.dimensions[1]))
  {
    return failure<moveit_msgs::msg::CollisionObject>(
      SceneGeometryErrorCode::InvalidConfiguration,
      "declared product collision geometry is not a valid cylinder");
  }

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = planning_frame;
  object.id = "restocker/declared/" + source_object_id;
  object.pose = pose_to_message(pose_in_planning_frame);
  object.primitives.push_back(geometry.primitive);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  object.primitive_poses.push_back(identity);
  object.operation = moveit_msgs::msg::CollisionObject::ADD;
  return SceneGeometryResult<moveit_msgs::msg::CollisionObject>::success(std::move(object));
}

SceneGeometryResult<std::vector<DeclaredScenarioProduct>> load_scenario_products(
  const std::filesystem::path & path)
{
  try {
    const YAML::Node root = YAML::LoadFile(path.string());
    if (!root["schema_version"] || root["schema_version"].as<int>() != 1) {
      return failure<std::vector<DeclaredScenarioProduct>>(
        SceneGeometryErrorCode::InvalidConfiguration,
        "scenario requires schema_version 1");
    }
    const YAML::Node products = root["products"];
    if (!products.IsSequence() || products.size() == 0) {
      return failure<std::vector<DeclaredScenarioProduct>>(
        SceneGeometryErrorCode::InvalidConfiguration,
        "scenario must declare at least one product");
    }
    std::vector<DeclaredScenarioProduct> declared;
    declared.reserve(products.size());
    std::set<std::string> seen_source_ids;
    for (const YAML::Node & node : products) {
      DeclaredScenarioProduct product;
      product.source_object_id = node["source_object_id"].as<std::string>("");
      product.geometry_key = node["geometry_key"].as<std::string>("");
      if (product.source_object_id.empty() || product.geometry_key.empty()) {
        return failure<std::vector<DeclaredScenarioProduct>>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "every scenario product requires a non-empty source_object_id and geometry_key");
      }
      if (!seen_source_ids.insert(product.source_object_id).second) {
        return failure<std::vector<DeclaredScenarioProduct>>(
          SceneGeometryErrorCode::InvalidConfiguration,
          "scenario declares duplicate source_object_id: " + product.source_object_id);
      }
      const YAML::Node pose = node["spawn_pose"];
      if (!pose || !pose.IsSequence() || pose.size() != 6U) {
        return failure<std::vector<DeclaredScenarioProduct>>(
          SceneGeometryErrorCode::InvalidPose,
          "scenario product " + product.source_object_id +
          " requires a six-value spawn_pose (x y z roll pitch yaw), the same xyz/rpy order the "
          "spawner consumes");
      }
      const double x = pose[0].as<double>();
      const double y = pose[1].as<double>();
      const double z = pose[2].as<double>();
      const double roll = pose[3].as<double>();
      const double pitch = pose[4].as<double>();
      const double yaw = pose[5].as<double>();
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        !std::isfinite(roll) || !std::isfinite(pitch) || !std::isfinite(yaw))
      {
        return failure<std::vector<DeclaredScenarioProduct>>(
          SceneGeometryErrorCode::InvalidPose,
          "scenario product " + product.source_object_id + " has an unusable spawn_pose");
      }
      // The spawner's fixed-axis RPY order, so the seed lands exactly where the product spawns.
      const Eigen::Quaterniond orientation =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
      product.planning_from_product.translation() = Eigen::Vector3d(x, y, z);
      product.planning_from_product.linear() = orientation.toRotationMatrix();
      declared.push_back(std::move(product));
    }
    return SceneGeometryResult<std::vector<DeclaredScenarioProduct>>::success(std::move(declared));
  } catch (const YAML::Exception & error) {
    return failure<std::vector<DeclaredScenarioProduct>>(
      SceneGeometryErrorCode::InvalidConfiguration,
      "failed to parse scenario products: " + std::string(error.what()));
  }
}

SceneGeometryResult<std::vector<moveit_msgs::msg::CollisionObject>>
extract_workcell_collision_objects(
  const std::string & workcell_urdf, const std::string & planning_frame,
  const Eigen::Isometry3d & planning_from_workcell_root)
{
  using ObjectVector = std::vector<moveit_msgs::msg::CollisionObject>;
  if (workcell_urdf.empty() || planning_frame.empty() ||
    !finite_transform(planning_from_workcell_root))
  {
    return failure<ObjectVector>(
      SceneGeometryErrorCode::InvalidDescription,
      "workcell extraction requires URDF, planning frame, and rigid finite root transform");
  }

  urdf::Model model;
  if (!model.initString(workcell_urdf) || !model.getRoot()) {
    return failure<ObjectVector>(
      SceneGeometryErrorCode::InvalidDescription, "workcell URDF could not be parsed");
  }

  ObjectVector objects;
  std::set<std::string> ids;
  std::optional<SceneGeometryError> traversal_error;
  std::function<void(const urdf::LinkConstSharedPtr &, const Eigen::Isometry3d &)> visit;
  visit = [&](const urdf::LinkConstSharedPtr & link, const Eigen::Isometry3d & root_from_link) {
    if (traversal_error) {
      return;
    }
    for (const auto & collision : link->collision_array) {
      if (!collision || collision->name.empty()) {
        traversal_error = SceneGeometryError{
          SceneGeometryErrorCode::InvalidDescription,
          "every workcell collision requires a non-empty name"};
        return;
      }
      const std::string id =
        "restocker/workcell/" + link->name + "/" + collision->name;
      if (!ids.insert(id).second) {
        traversal_error = SceneGeometryError{
          SceneGeometryErrorCode::InvalidDescription,
          "workcell produces duplicate collision ID: " + id};
        return;
      }
      auto primitive = primitive_from_urdf(collision->geometry, id);
      auto link_from_collision = pose_from_urdf(collision->origin, id);
      if (!primitive || !link_from_collision) {
        traversal_error = primitive ? link_from_collision.error() : primitive.error();
        return;
      }
      const Eigen::Isometry3d planning_from_collision =
        planning_from_workcell_root * root_from_link * link_from_collision.value();
      if (!finite_transform(planning_from_collision)) {
        traversal_error = SceneGeometryError{
          SceneGeometryErrorCode::InvalidPose,
          "workcell collision transform is not finite and rigid: " + id};
        return;
      }

      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = planning_frame;
      object.id = id;
      object.pose = pose_to_message(planning_from_collision);
      object.primitives.push_back(std::move(primitive.value()));
      geometry_msgs::msg::Pose identity;
      identity.orientation.w = 1.0;
      object.primitive_poses.push_back(identity);
      object.operation = moveit_msgs::msg::CollisionObject::ADD;
      objects.push_back(std::move(object));
    }

    std::vector<urdf::JointConstSharedPtr> joints(
      link->child_joints.begin(), link->child_joints.end());
    std::sort(
      joints.begin(), joints.end(),
      [](const auto & left, const auto & right) {return left->name < right->name;});
    for (const auto & joint : joints) {
      if (!joint || joint->type != urdf::Joint::FIXED) {
        traversal_error = SceneGeometryError{
          SceneGeometryErrorCode::InvalidDescription,
          "workcell collision tree may contain only fixed joints"};
        return;
      }
      auto link_from_child = pose_from_urdf(
        joint->parent_to_joint_origin_transform, "joint " + joint->name);
      const auto child = model.getLink(joint->child_link_name);
      if (!link_from_child || !child) {
        traversal_error = link_from_child ?
          SceneGeometryError{SceneGeometryErrorCode::InvalidDescription,
          "workcell joint has no child link: " + joint->name} : link_from_child.error();
        return;
      }
      visit(child, root_from_link * link_from_child.value());
    }
  };

  visit(model.getRoot(), Eigen::Isometry3d::Identity());
  if (traversal_error) {
    return SceneGeometryResult<ObjectVector>::failure(std::move(*traversal_error));
  }
  std::sort(
    objects.begin(), objects.end(),
    [](const auto & left, const auto & right) {return left.id < right.id;});
  if (objects.empty()) {
    return failure<ObjectVector>(
      SceneGeometryErrorCode::InvalidDescription,
      "workcell URDF does not contain collision geometry");
  }
  return SceneGeometryResult<ObjectVector>::success(std::move(objects));
}

}  // namespace restocker_task_executor
