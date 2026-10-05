// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

namespace restocker_task_executor
{

enum class SceneGeometryErrorCode : std::uint8_t
{
  InvalidConfiguration,
  UnsupportedGeometry,
  InvalidDescription,
  MissingGeometry,
  InvalidPose,
};

struct SceneGeometryError
{
  SceneGeometryErrorCode code;
  std::string detail;
};

template<typename T>
class [[nodiscard]] SceneGeometryResult
{
public:
  [[nodiscard]] static SceneGeometryResult success(T value)
  {
    return SceneGeometryResult(std::move(value));
  }

  [[nodiscard]] static SceneGeometryResult failure(SceneGeometryError error)
  {
    return SceneGeometryResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const SceneGeometryError & error() const
  {
    return std::get<SceneGeometryError>(storage_);
  }

private:
  explicit SceneGeometryResult(T value)
  : storage_(std::move(value)) {}

  explicit SceneGeometryResult(SceneGeometryError error)
  : storage_(std::move(error)) {}

  std::variant<T, SceneGeometryError> storage_;
};

enum class CatalogProductClass : std::uint8_t
{
  Can = 1,
  SmallBottle = 2,
  LargeBottle = 3,
};

struct ProductCollisionGeometry
{
  std::string geometry_key;
  CatalogProductClass product_class{CatalogProductClass::Can};
  std::optional<std::string> sku;
  bool class_fallback{false};
  shape_msgs::msg::SolidPrimitive primitive;
};

class ProductCollisionCatalog
{
public:
  [[nodiscard]] static SceneGeometryResult<ProductCollisionCatalog> load(
    const std::filesystem::path & path);

  [[nodiscard]] SceneGeometryResult<ProductCollisionGeometry> resolve(
    CatalogProductClass product_class, const std::optional<std::string> & sku) const;

  [[nodiscard]] const std::map<std::string, ProductCollisionGeometry> & entries() const noexcept
  {
    return entries_;
  }

private:
  std::map<std::string, ProductCollisionGeometry> entries_;
};

[[nodiscard]] SceneGeometryResult<moveit_msgs::msg::CollisionObject>
make_product_collision_object(
  std::uint64_t object_id, const std::string & planning_frame,
  const Eigen::Isometry3d & pose_in_planning_frame,
  const ProductCollisionGeometry & geometry);

// A product the scenario document declares as physically present, at its declared spawn pose in
// the planning frame: scenario `spawn_pose` is world-frame xyz/rpy in exactly the six values the
// spawner consumes (restocker_gazebo's `_pose_parameters`), and the seed lands on that pose.
//
// Card 050: until world state tracks a declared product the planning scene carried nothing of it,
// so a transfer's free-space segments could be planned through the products still standing in the
// tray. The projector seeds each of these as world collision geometry from spawn and drops the
// seed when world state first carries the same source id.
struct DeclaredScenarioProduct
{
  std::string source_object_id;
  std::string geometry_key;
  Eigen::Isometry3d planning_from_product{Eigen::Isometry3d::Identity()};
};

// Loads and validates the declared products of a scenario document. Fails on a missing or
// non-1 schema_version, no products, an empty or duplicate source_object_id, an empty
// geometry_key, or a spawn pose that is not six finite numbers (xyz/rpy).
[[nodiscard]] SceneGeometryResult<std::vector<DeclaredScenarioProduct>>
load_scenario_products(const std::filesystem::path & path);

// The scene seed for one declared product: the same catalogue cylinder as
// make_product_collision_object, but under the projector-managed id
// `restocker/declared/<source_object_id>` so the diff can remove it when tracking supersedes it
// and it can never be mistaken for a tracked object's `restocker/object/<n>`.
[[nodiscard]] SceneGeometryResult<moveit_msgs::msg::CollisionObject>
make_declared_product_collision_object(
  const std::string & source_object_id, const std::string & planning_frame,
  const Eigen::Isometry3d & pose_in_planning_frame,
  const ProductCollisionGeometry & geometry);

[[nodiscard]] SceneGeometryResult<std::vector<moveit_msgs::msg::CollisionObject>>
extract_workcell_collision_objects(
  const std::string & workcell_urdf, const std::string & planning_frame,
  const Eigen::Isometry3d & planning_from_workcell_root);

}  // namespace restocker_task_executor
