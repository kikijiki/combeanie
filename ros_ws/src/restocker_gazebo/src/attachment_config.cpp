// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/attachment_config.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace restocker_gazebo
{
namespace
{

struct CatalogEntry
{
  std::string product_class;
  std::string sku;
  double radius_m{0.0};
  double height_m{0.0};
  // A box (carton) has no radius. It is registered so its scenario rows validate, but it is not
  // attachable yet: the jaw geometry, hold target and verifier are derived for cylinders.
  bool box{false};
};

[[nodiscard]] double positive(const YAML::Node & node, const std::string & field)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(field + " must be finite and positive");
  }
  return value;
}

[[nodiscard]] double nonnegative(const YAML::Node & node, const std::string & field)
{
  const double value = node.as<double>();
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(field + " must be finite and non-negative");
  }
  return value;
}

[[nodiscard]] std::size_t positive_size(const YAML::Node & node, const std::string & field)
{
  const auto value = node.as<std::uint64_t>();
  if (value == 0 || value > std::numeric_limits<std::size_t>::max()) {
    throw std::invalid_argument(field + " must be a representable positive integer");
  }
  return static_cast<std::size_t>(value);
}

// gripper_controller's per-joint goal tolerance, from
// restocker_control/config/controllers.yaml (gripper_controller.constraints.*.goal). Restated
// here rather than parsed so the boundary stays a self-contained loader; a physical grip whose
// stall leaves the fingers further from the command than this reports a failed close on a grip
// that physically succeeded, so the configured band is refused at load time instead.
constexpr double kGripperControllerGoalToleranceM = 0.003;

[[nodiscard]] std::string required_string(const YAML::Node & node, const std::string & field)
{
  if (!node || node.IsNull()) {
    // Name the field: yaml-cpp's own message for a missing key does not say which key it was
    // looking for, and the detail string is what every rejection test and receipt reads.
    throw std::invalid_argument(field + " is required");
  }
  const std::string value = node.as<std::string>();
  if (value.empty()) {
    throw std::invalid_argument(field + " must not be empty");
  }
  return value;
}

[[nodiscard]] Eigen::Vector3d vector3(const YAML::Node & node, const std::string & field)
{
  if (!node.IsSequence() || node.size() != 3) {
    throw std::invalid_argument(field + " must contain exactly three values");
  }
  Eigen::Vector3d result(node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
  if (!result.allFinite()) {
    throw std::invalid_argument(field + " must be finite");
  }
  return result;
}

[[nodiscard]] Eigen::Matrix3d rotation_from_rpy(const Eigen::Vector3d & rpy)
{
  return (
    Eigen::AngleAxisd(rpy.z(), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(rpy.y(), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rpy.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

[[nodiscard]] Eigen::AlignedBox3d transformed_box(
  const Eigen::Isometry3d & output_from_input,
  const Eigen::Vector3d & center,
  const Eigen::Vector3d & size)
{
  if ((size.array() <= 0.0).any()) {
    throw std::invalid_argument("box dimensions must be positive");
  }
  Eigen::AlignedBox3d result;
  const Eigen::Vector3d half = 0.5 * size;
  for (int x_sign : {-1, 1}) {
    for (int y_sign : {-1, 1}) {
      for (int z_sign : {-1, 1}) {
        result.extend(
          output_from_input *
          (center + half.cwiseProduct(Eigen::Vector3d(x_sign, y_sign, z_sign))));
      }
    }
  }
  return result;
}

void require_schema_one(const YAML::Node & root, const std::string & document)
{
  if (!root.IsMap() || !root["schema_version"] || root["schema_version"].as<int>() != 1) {
    throw std::invalid_argument(document + " requires schema_version 1");
  }
}

[[nodiscard]] std::unordered_map<std::string, CatalogEntry> load_catalog(
  const YAML::Node & root)
{
  require_schema_one(root, "product catalog");
  const YAML::Node geometries = root["geometries"];
  if (!geometries.IsSequence() || geometries.size() == 0) {
    throw std::invalid_argument("product catalog must contain geometries");
  }
  std::unordered_map<std::string, CatalogEntry> result;
  for (const auto & geometry : geometries) {
    const std::string key = required_string(geometry["geometry_key"], "geometry_key");
    const std::string product_class = required_string(
      geometry["product_class"], key + " product_class");
    const std::string sku = required_string(geometry["sku"], key + " sku");
    const YAML::Node shape = geometry["shape"];
    const std::string shape_type = required_string(shape["type"], key + " shape type");
    CatalogEntry entry;
    if (shape_type == "box") {
      const YAML::Node size = shape["size_xyz_m"];
      if (!size.IsSequence() || size.size() != 3) {
        throw std::invalid_argument(key + " box size_xyz_m must hold three values");
      }
      entry = CatalogEntry{
        product_class, sku, 0.0, positive(size[2], key + " size_xyz_m height"), true};
    } else if (shape_type == "cylinder") {
      entry = CatalogEntry{
        product_class, sku, positive(shape["radius_m"], key + " radius_m"),
        positive(shape["height_m"], key + " height_m"), false};
    } else {
      throw std::invalid_argument(key + " must use cylinder or box collision geometry");
    }
    if (!result.emplace(key, std::move(entry)).second) {
      throw std::invalid_argument("duplicate catalog geometry_key: " + key);
    }
  }
  return result;
}

}  // namespace

AttachmentConfigResult load_attachment_boundary_config(
  const std::filesystem::path & boundary_path,
  const std::filesystem::path & gripper_geometry_path,
  const std::filesystem::path & product_catalog_path,
  const std::filesystem::path & scenario_path)
{
  try {
    const YAML::Node boundary = YAML::LoadFile(boundary_path.string());
    const YAML::Node gripper_yaml = YAML::LoadFile(gripper_geometry_path.string());
    const YAML::Node catalog_yaml = YAML::LoadFile(product_catalog_path.string());
    const YAML::Node scenario = YAML::LoadFile(scenario_path.string());
    require_schema_one(boundary, "attachment boundary");
    require_schema_one(gripper_yaml, "gripper geometry");
    require_schema_one(scenario, "scenario");
    const auto catalog = load_catalog(catalog_yaml);

    AttachmentBoundaryConfig result;
    const YAML::Node robot = boundary["robot"];
    result.robot_model_name = required_string(robot["model_name"], "robot model_name");
    result.parent_link = required_string(robot["parent_link"], "robot parent_link");
    result.left_finger_joint = required_string(
      robot["left_finger_joint"], "robot left_finger_joint");
    result.right_finger_joint = required_string(
      robot["right_finger_joint"], "robot right_finger_joint");
    if (result.left_finger_joint == result.right_finger_joint) {
      throw std::invalid_argument("finger joint names must be distinct");
    }
    const std::string child_link = required_string(
      boundary["products"]["child_link"], "product child_link");
    result.set_service = required_string(
      boundary["transport"]["set_service"], "transport set_service");
    result.query_service = required_string(
      boundary["transport"]["query_service"], "transport query_service");
    if (result.set_service == result.query_service || result.set_service.front() != '/' ||
      result.query_service.front() != '/')
    {
      throw std::invalid_argument("transport services must be distinct absolute names");
    }

    // The mode lives in gripper_geometry.yaml, not here: that is the one file this boundary and
    // the task executor's grasp generator both read, so the aperture a run commands and the
    // aperture it verifies come from the same key. Required rather than defaulted — a policy
    // file that forgets it must not silently inherit one, because whether an entity is created
    // and which evidence a run may be cited for both follow from it.
    const std::string hold_mode =
      required_string(gripper_yaml["attachment"]["hold_mode"], "attachment.hold_mode");
    if (hold_mode == hold_mode_name(AttachmentHoldMode::kWeld)) {
      result.hold_mode = AttachmentHoldMode::kWeld;
    } else if (hold_mode == hold_mode_name(AttachmentHoldMode::kPhysical)) {
      result.hold_mode = AttachmentHoldMode::kPhysical;
    } else {
      throw std::invalid_argument(
              "attachment.hold_mode must be 'weld' or 'physical', got '" + hold_mode + "'");
    }

    result.journal_capacity = positive_size(
      boundary["journal"]["capacity"], "journal capacity");
    result.detach_attempt_reserve = positive_size(
      boundary["journal"]["detach_attempt_reserve"], "detach attempt reserve");
    if (result.journal_capacity < 1 + result.detach_attempt_reserve) {
      throw std::invalid_argument("journal capacity cannot satisfy detach reserve");
    }
    result.required_verification_ticks = positive_size(
      boundary["verification"]["required_ticks"], "required verification ticks");
    result.maximum_pending_ticks = positive_size(
      boundary["verification"]["maximum_pending_ticks"], "maximum pending ticks");
    if (result.maximum_pending_ticks <= result.required_verification_ticks) {
      throw std::invalid_argument("maximum pending ticks must exceed verification ticks");
    }

    const YAML::Node tolerance = boundary["tolerances"];
    result.tolerances = AttachmentPhysicalTolerances{
      positive(tolerance["joint_position_m"], "joint position tolerance"),
      positive(tolerance["expected_translation_m"], "expected translation tolerance"),
      positive(tolerance["expected_rotation_rad"], "expected rotation tolerance"),
      positive(tolerance["relative_linear_velocity_mps"], "relative linear velocity tolerance"),
      positive(
        tolerance["relative_angular_velocity_radps"],
        "relative angular velocity tolerance"),
      positive(tolerance["fidelity_sigma_multiplier"], "fidelity sigma multiplier"),
      positive(tolerance["fidelity_mechanical_margin_m"], "fidelity mechanical margin")};
    result.post_attach_translation_drift_m = positive(
      tolerance["post_attach_translation_drift_m"], "post-attach translation drift");
    result.post_attach_rotation_drift_rad = positive(
      tolerance["post_attach_rotation_drift_rad"], "post-attach rotation drift");
    if (!tolerance["held_swing_allowance_m"]) {
      throw std::invalid_argument("held swing allowance (held_swing_allowance_m) is required");
    }
    result.held_swing_allowance_m = positive(
      tolerance["held_swing_allowance_m"], "held swing allowance");
    if (!tolerance["held_slip_allowance_m"]) {
      throw std::invalid_argument("held slip allowance (held_slip_allowance_m) is required");
    }
    result.held_slip_allowance_m = positive(
      tolerance["held_slip_allowance_m"], "held slip allowance");
    if (result.held_slip_allowance_m < result.post_attach_translation_drift_m) {
      throw std::invalid_argument(
              "held slip allowance must not be tighter than post_attach_translation_drift_m");
    }
    if (result.held_swing_allowance_m < result.held_slip_allowance_m) {
      throw std::invalid_argument(
              "held swing allowance must not be tighter than the held slip allowance");
    }
    if (!tolerance["held_tilt_allowance_rad"]) {
      throw std::invalid_argument("held tilt allowance (held_tilt_allowance_rad) is required");
    }
    result.held_tilt_allowance_rad = positive(
      tolerance["held_tilt_allowance_rad"], "held tilt allowance");
    if (result.held_tilt_allowance_rad < result.tolerances.expected_rotation_rad ||
      result.held_tilt_allowance_rad > std::acos(-1.0))
    {
      throw std::invalid_argument(
              "held tilt allowance must lie between expected_rotation_rad and pi");
    }
    if (result.tolerances.expected_rotation_rad > std::acos(-1.0) ||
      result.post_attach_rotation_drift_rad > std::acos(-1.0))
    {
      throw std::invalid_argument("angular tolerances must not exceed pi");
    }

    const YAML::Node body = gripper_yaml["body"];
    const YAML::Node grasp = gripper_yaml["grasp_center"];
    const YAML::Node finger = gripper_yaml["finger"];
    const YAML::Node joint = finger["joint"];
    const YAML::Node attachment = gripper_yaml["attachment"];
    const YAML::Node pad = finger["pad"];
    if (!pad || !pad["friction_by_material"] || !pad["material_by_product_class"]) {
      throw std::invalid_argument(
              "finger.pad friction_by_material and material_by_product_class are required");
    }
    std::map<std::string, double> friction_by_material;
    for (const auto & entry : pad["friction_by_material"]) {
      const double mu = positive(entry.second, "pad friction by material");
      friction_by_material.emplace(entry.first.as<std::string>(), mu);
    }
    const YAML::Node material_by_class = pad["material_by_product_class"];
    result.gripper_from_grasp_center.translation() = vector3(
      grasp["xyz_m"], "grasp center xyz_m");
    result.gripper_from_grasp_center.linear() = rotation_from_rpy(
      vector3(grasp["rpy_rad"], "grasp center rpy_rad"));
    const Eigen::Isometry3d grasp_center_from_gripper =
      result.gripper_from_grasp_center.inverse();
    const Eigen::Vector3d finger_size = vector3(finger["size_xyz_m"], "finger size_xyz_m");
    const Eigen::Vector3d finger_center = vector3(
      finger["center_xyz_m"], "finger center_xyz_m");
    const Eigen::Vector3d left_origin = vector3(
      finger["left"]["origin_xyz_m"], "left finger origin_xyz_m");
    const Eigen::Vector3d right_origin = vector3(
      finger["right"]["origin_xyz_m"], "right finger origin_xyz_m");
    const Eigen::Vector3d left_axis = vector3(finger["left"]["axis"], "left finger axis");
    const Eigen::Vector3d right_axis = vector3(
      finger["right"]["axis"], "right finger axis");
    if (!left_axis.isApprox(Eigen::Vector3d::UnitY(), 1.0e-12) ||
      !right_axis.isApprox(-Eigen::Vector3d::UnitY(), 1.0e-12))
    {
      throw std::invalid_argument("attachment boundary requires symmetric Y-axis fingers");
    }
    const Eigen::AlignedBox3d left_box = transformed_box(
      grasp_center_from_gripper, left_origin + finger_center, finger_size);
    const Eigen::AlignedBox3d right_box = transformed_box(
      grasp_center_from_gripper, right_origin + finger_center, finger_size);
    if (!Eigen::Vector2d(left_box.min().x(), left_box.min().z()).isApprox(
        Eigen::Vector2d(right_box.min().x(), right_box.min().z()), 1.0e-12) ||
      !Eigen::Vector2d(left_box.max().x(), left_box.max().z()).isApprox(
        Eigen::Vector2d(right_box.max().x(), right_box.max().z()), 1.0e-12))
    {
      throw std::invalid_argument("finger contact regions must be symmetric in grasp XZ");
    }
    result.gripper.palm_bounds_in_grasp_center = transformed_box(
      grasp_center_from_gripper, vector3(body["center_xyz_m"], "body center_xyz_m"),
      vector3(body["size_xyz_m"], "body size_xyz_m"));
    result.gripper.finger_contact_bounds_xz = Eigen::AlignedBox2d(
      Eigen::Vector2d(left_box.min().x(), left_box.min().z()),
      Eigen::Vector2d(left_box.max().x(), left_box.max().z()));
    result.gripper.left_inner_face_at_zero_m = left_box.min().y();
    result.gripper.right_inner_face_at_zero_m = right_box.max().y();
    result.gripper.joint_lower_m = nonnegative(joint["lower_m"], "finger joint lower_m");
    result.gripper.joint_upper_m = positive(joint["upper_m"], "finger joint upper_m");
    result.gripper.open_target_m = nonnegative(
      attachment["open_target_m"], "attachment open_target_m");
    result.gripper.minimum_inner_clearance_m = positive(
      attachment["minimum_inner_clearance_m"], "minimum inner clearance");
    result.gripper.minimum_contact_overlap_m = positive(
      attachment["minimum_contact_overlap_m"], "minimum contact overlap");
    const double hold_clearance = positive(
      attachment["hold_clearance_per_side_m"], "hold clearance per side");
    const double open_clearance = positive(
      attachment["open_clearance_per_side_m"], "open clearance per side");
    if (hold_clearance < result.gripper.minimum_inner_clearance_m ||
      open_clearance <= hold_clearance)
    {
      throw std::invalid_argument(
              "attachment clearance policy must satisfy minimum <= hold < open");
    }
    const double hold_interference_m = positive(
      attachment["hold_interference_m"], "hold interference per side");
    if (result.hold_mode == AttachmentHoldMode::kPhysical) {
      result.gripper.hold_target_offset_m = hold_clearance + hold_interference_m;
    }
    result.gripper.maximum_penetration_m = nonnegative(
      attachment["maximum_penetration_m"], "maximum penetration per side");
    result.gripper.contact_tolerance_m = nonnegative(
      attachment["contact_tolerance_m"], "contact tolerance per side");
    if (result.hold_mode == AttachmentHoldMode::kPhysical &&
      (result.gripper.maximum_penetration_m <= 0.0 ||
      result.gripper.contact_tolerance_m <= 0.0))
    {
      throw std::invalid_argument(
              "physical hold mode requires a positive maximum penetration and contact tolerance");
    }
    // The interference must sit inside the jaw travel and stay clear of the controller's goal
    // tolerance when the finger stalls against the product (spec 11.3 D3): the residual the
    // gripper controller measures is the interference plus however deep the contact sank, and it
    // has to fit under that 0.003 m goal.
    if (result.hold_mode == AttachmentHoldMode::kPhysical) {
      if (hold_interference_m >= open_clearance) {
        throw std::invalid_argument(
                "hold interference must stay below the open clearance");
      }
      // The finger that stalls against the product sits one interference inboard of the command
      // at contact, plus however deep the contact sank, or that much outboard if it never got
      // there. Either way the residual the gripper controller measures against its own goal must
      // fit, or `kCloseGripper` reports failure on a grip that physically succeeded.
      const double worst_residual = std::max(
        hold_interference_m + result.gripper.contact_tolerance_m,
        result.gripper.maximum_penetration_m - hold_interference_m);
      if (worst_residual > kGripperControllerGoalToleranceM) {
        throw std::invalid_argument(
                "physical contact band leaves the fingers further from the hold command than the "
                "gripper controller's goal tolerance admits");
      }
    }

    const YAML::Node products = scenario["products"];
    if (!products.IsSequence() || products.size() == 0) {
      throw std::invalid_argument("scenario must contain products");
    }
    std::set<std::string> source_ids;
    std::set<std::string> model_names;
    for (const auto & product_yaml : products) {
      const std::string source_id = required_string(
        product_yaml["source_object_id"], "scenario source_object_id");
      const std::string model_name = required_string(
        product_yaml["model_name"], "scenario model_name");
      const std::string geometry_key = required_string(
        product_yaml["geometry_key"], "scenario geometry_key");
      const std::string product_class = required_string(
        product_yaml["product_class"], "scenario product_class");
      const std::string sku = required_string(product_yaml["sku"], "scenario sku");
      // Fail closed: a product class the pad friction table does not know would silently fall
      // back to the engine default, which is exactly the accident §11.5 exists to prevent.
      const YAML::Node material = material_by_class[product_class];
      if (!material) {
        throw std::invalid_argument(
                "scenario product_class has no pad material: " + product_class);
      }
      const auto coefficient = friction_by_material.find(material.as<std::string>());
      if (coefficient == friction_by_material.end()) {
        throw std::invalid_argument(
                "pad material has no friction coefficient: " + material.as<std::string>());
      }
      result.pad_friction_by_product_class[product_class] = coefficient->second;
      if (!source_ids.insert(source_id).second || !model_names.insert(model_name).second) {
        throw std::invalid_argument("scenario source IDs and model names must be unique");
      }
      const auto catalog_iterator = catalog.find(geometry_key);
      if (catalog_iterator == catalog.end()) {
        throw std::invalid_argument("scenario references unknown geometry_key: " + geometry_key);
      }
      const CatalogEntry & catalog_entry = catalog_iterator->second;
      if (product_class != catalog_entry.product_class || sku != catalog_entry.sku) {
        throw std::invalid_argument("scenario product semantics disagree with catalog");
      }
      if (catalog_entry.box) {
        // Not attachable: no AttachmentProductConfig, so an attach request for it is refused as
        // an unknown source id instead of being sized from a radius it does not have.
        continue;
      }
      const double diameter = 2.0 * catalog_entry.radius_m;
      const double zero_gap = result.gripper.left_inner_face_at_zero_m -
        result.gripper.right_inner_face_at_zero_m;
      // Weld mode stops one hold clearance short of the product, so the joint carries it.
      // Physical mode commands past the surface by the interference, so the fingers stop against
      // the product and friction carries it (spec 11.3 D3). The two apertures differ by exactly
      // hold_clearance + hold_interference per side, which is also how much further the fidelity
      // ceiling opens in physical mode, because that ceiling is measured from this target.
      const double hold_gap =
        result.hold_mode == AttachmentHoldMode::kPhysical ?
        diameter - 2.0 * hold_interference_m :
        diameter + 2.0 * hold_clearance;
      const double hold_target = 0.5 * (hold_gap - zero_gap);
      AttachmentProductConfig product{
        source_id, model_name, child_link, geometry_key, product_class, sku,
        CylinderAttachmentGeometry{
          catalog_entry.radius_m, catalog_entry.height_m, hold_target}};
      const auto geometry_status = validate_attachment_geometry(
        result.gripper, product.geometry, result.tolerances);
      if (geometry_status.code != AttachmentStatusCode::kPending) {
        throw std::invalid_argument(model_name + ": " + geometry_status.detail);
      }
      result.source_id_by_model_name.emplace(model_name, source_id);
      result.products_by_source_id.emplace(source_id, std::move(product));
    }
    return AttachmentConfigResult::success(std::move(result));
  } catch (const YAML::Exception & error) {
    return AttachmentConfigResult::failure(
      "failed to parse attachment configuration: " + std::string(error.what()));
  } catch (const std::invalid_argument & error) {
    return AttachmentConfigResult::failure(error.what());
  }
}

}  // namespace restocker_gazebo
