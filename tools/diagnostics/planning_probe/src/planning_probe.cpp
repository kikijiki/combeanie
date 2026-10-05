// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
//
// Offline planning probe: plans against the real RobotModel, SRDF and planning scene with no
// simulator running. A placement sweep is thousands of Cartesian plans; a live run takes minutes
// each.
//
// Read tools/diagnostics/README.md before changing anything here. Two rules must hold:
//
//   1. The planner's robot padding is applied to the collision environment. Without it the probe
//      measures a corridor no collision check uses, and a 1.5 mm interpenetration reads as
//      clearance.
//   2. A sweep is never reduced to its best branch. The pre-grasp traverse is a pose goal on the
//      redundant `manipulator` group, so MoveIt's sampler picks the rail and the elbow. Every mode
//      reports `starts` and `complete` separately; a placement is "clear" only when they are equal.
//
// Output is CSV on stdout with a leading `#` provenance block.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

namespace
{

// ---------------------------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------------------------

[[noreturn]] void fail(const std::string & message)
{
  std::cerr << "planning_probe: " << message << '\n';
  std::exit(2);
}

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    fail("cannot read " + path);
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

std::vector<double> parse_doubles(const std::string & text)
{
  std::vector<double> values;
  std::stringstream stream(text);
  std::string field;
  while (std::getline(stream, field, ',')) {
    values.push_back(std::stod(field));
  }
  return values;
}

Eigen::Isometry3d pose_from_xyzrpy(const std::vector<double> & values)
{
  if (values.size() != 6) {
    fail("a pose needs six comma-separated values x,y,z,roll,pitch,yaw");
  }
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(values[0], values[1], values[2]);
  pose.linear() = (Eigen::AngleAxisd(values[5], Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(values[4], Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(values[3], Eigen::Vector3d::UnitX())).toRotationMatrix();
  return pose;
}

Eigen::Isometry3d urdf_pose(const urdf::Pose & pose)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  result.linear() =
    Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z)
    .normalized().toRotationMatrix();
  return result;
}

// ---------------------------------------------------------------------------------------------
// The pieces of the shipped configuration the probe measures against
// ---------------------------------------------------------------------------------------------

// Each value is read from the file that owns it, so the probe cannot disagree with the tree. The
// exception is fk.py's duplicated kinematics, which is documented as intentional.
struct GripperGeometry
{
  double finger_half_width_m{0.0};      // half of finger.size_xyz_m[1]
  double finger_length_m{0.0};          // finger.size_xyz_m[2]
  double jaw_mount_offset_m{0.0};       // max |finger.<side>.origin_xyz_m[1]|
  double open_target_m{0.0};            // attachment.open_target_m, the release aperture
  double open_clearance_per_side_m{0.0};
  double hold_clearance_per_side_m{0.0};
  double joint_upper_m{0.0};
  double tool0_from_grasp_center_m{0.0};  // grasp_center.xyz_m[2]

  // The inner gap between the jaw faces with both finger joints at zero.
  [[nodiscard]] double inner_gap_at_zero_m() const
  {
    return 2.0 * (jaw_mount_offset_m - finger_half_width_m);
  }

  // The distance from the tool axis to the outer face of the further-reaching jaw, at aperture.
  [[nodiscard]] double finger_outer_face_m(double joint_position_m) const
  {
    return jaw_mount_offset_m + joint_position_m + finger_half_width_m;
  }
};

GripperGeometry load_gripper_geometry(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1) {
    fail("gripper_geometry schema_version must be 1");
  }
  const YAML::Node finger = root["finger"];
  const YAML::Node attachment = root["attachment"];
  GripperGeometry geometry;
  geometry.finger_half_width_m = finger["size_xyz_m"][1].as<double>() / 2.0;
  geometry.finger_length_m = finger["size_xyz_m"][2].as<double>();
  geometry.jaw_mount_offset_m = std::max(
    std::abs(finger["left"]["origin_xyz_m"][1].as<double>()),
    std::abs(finger["right"]["origin_xyz_m"][1].as<double>()));
  geometry.open_target_m = attachment["open_target_m"].as<double>();
  geometry.open_clearance_per_side_m = attachment["open_clearance_per_side_m"].as<double>();
  geometry.hold_clearance_per_side_m = attachment["hold_clearance_per_side_m"].as<double>();
  geometry.joint_upper_m = finger["joint"]["upper_m"].as<double>();
  geometry.tool0_from_grasp_center_m = root["grasp_center"]["xyz_m"][2].as<double>();
  return geometry;
}

struct ProductShape
{
  std::string geometry_key;
  std::string product_class;
  double radius_m{0.0};
  double height_m{0.0};
};

std::vector<ProductShape> load_product_catalog(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1) {
    fail("product_collision_catalog schema_version must be 1");
  }
  std::vector<ProductShape> shapes;
  for (const auto & entry : root["geometries"]) {
    ProductShape shape;
    shape.geometry_key = entry["geometry_key"].as<std::string>();
    shape.product_class = entry["product_class"].as<std::string>();
    shape.radius_m = entry["shape"]["radius_m"].as<double>();
    shape.height_m = entry["shape"]["height_m"].as<double>();
    shapes.push_back(shape);
  }
  return shapes;
}

// The surveyed stock-tray placement envelope, in the shelf frame, as
// restocker_gazebo.scenario_config.load_stock_region reads it.
struct StockRegion
{
  Eigen::Vector3d center_xyz_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d size_xyz_m{Eigen::Vector3d::Zero()};
};

StockRegion load_stock_region(const std::string & path)
{
  const YAML::Node root = YAML::LoadFile(path);
  if (root["schema_version"].as<int>() != 1) {
    fail("workcell_geometry schema_version must be 1");
  }
  const YAML::Node volume = root["stock_tray"]["usable_volume"];
  StockRegion region;
  for (int axis = 0; axis < 3; ++axis) {
    region.center_xyz_m[axis] = volume["center_xyz_m"][axis].as<double>();
    region.size_xyz_m[axis] = volume["size_xyz_m"][axis].as<double>();
  }
  return region;
}

// The coordinator's grasp configuration, parsed from the ROS parameter YAML by key (the file is a
// `<node>: ros__parameters:` document with dotted keys).
struct GraspConfig
{
  double minimum_center_height_above_base_m{0.0};
  double maximum_top_above_center_m{0.0};
  double pregrasp_distance_m{0.0};
  double retract_distance_m{0.0};
};

GraspConfig load_grasp_config(const std::string & path)
{
  GraspConfig config;
  std::ifstream stream(path);
  if (!stream) {
    fail("cannot read " + path);
  }
  const std::map<std::string, double *> wanted{
    {"grasp.minimum_center_height_above_base_m", &config.minimum_center_height_above_base_m},
    {"grasp.maximum_top_above_center_m", &config.maximum_top_above_center_m},
    {"grasp.pregrasp_distance_m", &config.pregrasp_distance_m},
    {"grasp.retract_distance_m", &config.retract_distance_m}};
  std::string line;
  while (std::getline(stream, line)) {
    const auto hash = line.find('#');
    const std::string body = hash == std::string::npos ? line : line.substr(0, hash);
    const auto colon = body.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string key = body.substr(0, colon);
    key.erase(0, key.find_first_not_of(" \t"));
    const auto found = wanted.find(key);
    if (found == wanted.end()) {
      continue;
    }
    *found->second = std::stod(body.substr(colon + 1));
  }
  for (const auto & [key, target] : wanted) {
    if (*target == 0.0) {
      fail("could not read " + key + " from " + path);
    }
  }
  return config;
}

// ---------------------------------------------------------------------------------------------
// The workcell, as the projector puts it into the planning scene
// ---------------------------------------------------------------------------------------------

// Same walk over the workcell URDF's fixed joints as
// restocker_task_executor::extract_workcell_collision_objects, reimplemented so the probe builds
// without the ROS workspace. Checked against that function by `--mode selftest`.
std::vector<moveit_msgs::msg::CollisionObject> workcell_collision_objects(
  const std::string & workcell_urdf, const std::string & planning_frame,
  const Eigen::Isometry3d & planning_from_root)
{
  urdf::Model model;
  if (!model.initString(workcell_urdf) || !model.getRoot()) {
    fail("workcell URDF could not be parsed");
  }

  std::vector<moveit_msgs::msg::CollisionObject> objects;
  std::function<void(const urdf::LinkConstSharedPtr &, const Eigen::Isometry3d &)> visit;
  visit = [&](const urdf::LinkConstSharedPtr & link, const Eigen::Isometry3d & root_from_link) {
      for (const auto & collision : link->collision_array) {
        if (!collision || collision->name.empty()) {
          fail("every workcell collision requires a non-empty name");
        }
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = planning_frame;
        object.id = "restocker/workcell/" + link->name + "/" + collision->name;
        shape_msgs::msg::SolidPrimitive primitive;
        if (collision->geometry->type == urdf::Geometry::BOX) {
          const auto * box = dynamic_cast<const urdf::Box *>(collision->geometry.get());
          primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
          primitive.dimensions = {box->dim.x, box->dim.y, box->dim.z};
        } else if (collision->geometry->type == urdf::Geometry::CYLINDER) {
          const auto * cylinder = dynamic_cast<const urdf::Cylinder *>(collision->geometry.get());
          primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
          primitive.dimensions = {cylinder->length, cylinder->radius};
        } else {
          fail("workcell collision " + object.id + " is not a box or a cylinder");
        }
        object.pose = tf2::toMsg(
          planning_from_root * root_from_link * urdf_pose(collision->origin));
        object.primitives.push_back(primitive);
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
          fail("workcell collision tree may contain only fixed joints");
        }
        visit(
          model.getLink(joint->child_link_name),
          root_from_link * urdf_pose(joint->parent_to_joint_origin_transform));
      }
    };
  visit(model.getRoot(), Eigen::Isometry3d::Identity());
  return objects;
}

// ---------------------------------------------------------------------------------------------
// Grasp geometry, reproduced from restocker_task_executor/src/grasp_candidates.cpp
// ---------------------------------------------------------------------------------------------

// A horizontal side grasp of an upright cylinder. +z of the grasp frame is the horizontal
// approach, +y is the jaw closing axis, +x is forced vertical and must point up.
Eigen::Isometry3d grasp_pose(const Eigen::Vector3d & center, double yaw)
{
  const Eigen::Vector3d approach(std::cos(yaw), std::sin(yaw), 0.0);
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = center;
  pose.linear().col(0) = Eigen::Vector3d::UnitZ();
  pose.linear().col(1) = approach.cross(Eigen::Vector3d::UnitZ());
  pose.linear().col(2) = approach;
  return pose;
}

// Height of the grasp centre above the product's mid-height: the same std::max of three terms as
// grasp_candidates.cpp.
double grasp_axial_offset_m(const GraspConfig & config, double product_height_m)
{
  const double half_height = 0.5 * product_height_m;
  return std::max(
    {0.0, config.minimum_center_height_above_base_m - half_height,
      half_height - config.maximum_top_above_center_m});
}

// The jaw position the coordinator parks at for the approach to one product.
double approach_jaw_position_m(const GripperGeometry & gripper, double radius_m)
{
  const double open_gap = 2.0 * radius_m + 2.0 * gripper.open_clearance_per_side_m;
  return 0.5 * (open_gap - gripper.inner_gap_at_zero_m());
}

// ---------------------------------------------------------------------------------------------
// The probe itself
// ---------------------------------------------------------------------------------------------

struct StartOutcome
{
  bool completed{false};
  double checked_fraction{0.0};
  double unchecked_fraction{0.0};
  // Empty when the truncation was kinematic. Otherwise the first contact pair MoveIt reports at
  // the refused configuration, in the form the live motion port logs.
  std::string blocker;
};

struct SweepOutcome
{
  int starts{0};
  int complete{0};
  int kinematic_truncations{0};
  int collision_truncations{0};
  std::map<std::string, int> blockers;

  [[nodiscard]] bool clear() const {return starts > 0 && complete == starts;}
};

class Probe
{
public:
  Probe(
    const rclcpp::Node::SharedPtr & node, const std::string & robot_urdf,
    const std::string & srdf, double padding_m)
  : node_(node)
  {
    robot_model_loader::RobotModelLoader::Options options(robot_urdf, srdf);
    options.load_kinematics_solvers = true;
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, options);
    model_ = loader_->getModel();
    if (!model_) {
      fail("the robot model did not load");
    }
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    // Rule 1. MoveIt inflates every robot link by default_robot_padding before collision checking.
    scene_->getCollisionEnvNonConst()->setPadding(padding_m);
    padding_m_ = padding_m;
    // Assert it took: `getCollisionEnvNonConst()` is the padded environment used by
    // `isStateColliding` and `checkCollision`; `checkCollisionUnpadded` uses the other one.
    for (const auto * link : model_->getLinkModelsWithCollisionGeometry()) {
      if (std::abs(scene_->getCollisionEnv()->getLinkPadding(link->getName()) - padding_m) >
        1.0e-12)
      {
        fail("the collision environment did not take the requested robot padding");
      }
    }
  }

  [[nodiscard]] const moveit::core::RobotModelPtr & model() const {return model_;}
  [[nodiscard]] const planning_scene::PlanningScenePtr & scene() const {return scene_;}
  [[nodiscard]] double padding_m() const {return padding_m_;}

  void add_objects(const std::vector<moveit_msgs::msg::CollisionObject> & objects)
  {
    for (const auto & object : objects) {
      if (!scene_->processCollisionObjectMsg(object)) {
        fail("the planning scene refused collision object " + object.id);
      }
    }
  }

  void add_product(
    const std::string & id, const Eigen::Vector3d & center, const ProductShape & shape)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = model_->getModelFrame();
    object.id = id;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = center;
    object.pose = tf2::toMsg(pose);
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    // Products enter the scene at their catalogued dimensions. The padding above is one-sided
    // and applies only to robot links; see scene_geometry.cpp.
    primitive.dimensions = {shape.height_m, shape.radius_m};
    object.primitives.push_back(primitive);
    geometry_msgs::msg::Pose identity;
    identity.orientation.w = 1.0;
    object.primitive_poses.push_back(identity);
    object.operation = moveit_msgs::msg::CollisionObject::ADD;
    if (!scene_->processCollisionObjectMsg(object)) {
      fail("the planning scene refused product " + id);
    }
  }

  void remove_object(const std::string & id)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = model_->getModelFrame();
    object.id = id;
    object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    scene_->processCollisionObjectMsg(object);
  }

  // Every distinct collision-free IK solution the sampler can land on for one tool0 pose, with the
  // jaws parked where the coordinator parks them.
  //
  // Rule 2: returns the whole family, never its best member, because a placement is safe only when
  // every one completes. `ik_solutions` and `first_blocker`, when supplied, separate the two empty
  // cases: `ik_solutions == 0` means the solver never reached the pose; `> 0` means the scene
  // refused it, and `first_blocker` names the refusing pair.
  std::vector<moveit::core::RobotState> collision_free_starts(
    const Eigen::Isometry3d & tool0_pose, double jaw_position_m, int seeds, unsigned int rng_seed,
    int * ik_solutions = nullptr, std::map<std::string, int> * blockers = nullptr)
  {
    const auto * group = model_->getJointModelGroup(group_name_);
    if (!group) {
      fail("the robot model has no group named " + group_name_);
    }
    random_numbers::RandomNumberGenerator rng(rng_seed);
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    apply_jaws(state, jaw_position_m);

    const auto validity = [this, jaw_position_m](
      moveit::core::RobotState * candidate, const moveit::core::JointModelGroup * candidate_group,
      const double * values) {
        candidate->setJointGroupPositions(candidate_group, values);
        apply_jaws(*candidate, jaw_position_m);
        candidate->update();
        return !scene_->isStateColliding(*candidate, candidate_group->getName());
      };

    std::vector<moveit::core::RobotState> starts;
    std::vector<std::vector<double>> seen;
    for (int attempt = 0; attempt < seeds; ++attempt) {
      moveit::core::RobotState candidate(state);
      candidate.setToRandomPositions(group, rng);
      apply_jaws(candidate, jaw_position_m);
      if (!candidate.setFromIK(group, tool0_pose, tip_link_, ik_timeout_s_, validity)) {
        // Re-solve with no validity callback to tell scene-refused poses from unreachable ones.
        if (ik_solutions != nullptr || blockers != nullptr) {
          moveit::core::RobotState unchecked(state);
          unchecked.setToRandomPositions(group, rng);
          apply_jaws(unchecked, jaw_position_m);
          if (unchecked.setFromIK(group, tool0_pose, tip_link_, ik_timeout_s_)) {
            apply_jaws(unchecked, jaw_position_m);
            unchecked.update();
            if (ik_solutions != nullptr) {
              ++*ik_solutions;
            }
            if (blockers != nullptr) {
              // Count every pair the scene reports, across configurations: a self-collision that
              // sorts first would otherwise hide the product refusing every branch.
              for (const std::string & pair : all_contacts(unchecked, jaw_position_m)) {
                (*blockers)[pair] += 1;
              }
            }
          }
        }
        continue;
      }
      if (ik_solutions != nullptr) {
        ++*ik_solutions;
      }
      apply_jaws(candidate, jaw_position_m);
      candidate.update();
      std::vector<double> values;
      candidate.copyJointGroupPositions(group, values);
      const bool duplicate = std::any_of(
        seen.begin(), seen.end(), [&values](const std::vector<double> & other) {
          double worst = 0.0;
          for (std::size_t index = 0; index < values.size(); ++index) {
            worst = std::max(worst, std::abs(values[index] - other[index]));
          }
          return worst < 1.0e-3;
        });
      if (duplicate) {
        continue;
      }
      seen.push_back(values);
      starts.push_back(candidate);
    }
    return starts;
  }

  // One straight line, run as MoveItMotionPort runs it: collision-checked at cartesian_step_m,
  // then again with no callback to tell "blocked" from "not kinematically reachable".
  StartOutcome run_linear(
    const moveit::core::RobotState & start, const Eigen::Isometry3d & target_tool0,
    double jaw_position_m, moveit::core::RobotState * reached = nullptr)
  {
    const auto * group = model_->getJointModelGroup(group_name_);
    const auto * tip = model_->getLinkModel(tip_link_);

    const auto validity = [this, jaw_position_m](
      moveit::core::RobotState * candidate, const moveit::core::JointModelGroup * candidate_group,
      const double * values) {
        candidate->setJointGroupPositions(candidate_group, values);
        apply_jaws(*candidate, jaw_position_m);
        candidate->update();
        return !scene_->isStateColliding(*candidate, candidate_group->getName());
      };

    const moveit::core::MaxEEFStep step(cartesian_step_m_);
    const moveit::core::CartesianPrecision precision;

    std::vector<moveit::core::RobotStatePtr> checked_path;
    moveit::core::RobotState checked_start(start);
    const double checked = moveit::core::CartesianInterpolator::computeCartesianPath(
      &checked_start, group, checked_path, tip, target_tool0, true, step, precision, validity,
      kinematics::KinematicsQueryOptions());

    std::vector<moveit::core::RobotStatePtr> unchecked_path;
    moveit::core::RobotState unchecked_start(start);
    const double unchecked = moveit::core::CartesianInterpolator::computeCartesianPath(
      &unchecked_start, group, unchecked_path, tip, target_tool0, true, step, precision,
      moveit::core::GroupStateValidityCallbackFn(), kinematics::KinematicsQueryOptions());

    StartOutcome outcome;
    outcome.checked_fraction = checked;
    outcome.unchecked_fraction = unchecked;
    outcome.completed = checked >= minimum_fraction_;
    // The state the checked run ended in. The next segment must start from it, not from a fresh IK
    // solution: the group is redundant, so re-solving could pick a different branch.
    if (reached != nullptr && !checked_path.empty()) {
      *reached = *checked_path.back();
    }
    if (!outcome.completed && unchecked >= minimum_fraction_ && unchecked_path.size() >= 2) {
      // Waypoint counts differ between the runs, so distance along the line is the comparable
      // coordinate. Matches describe_truncated_linear_path.
      const std::size_t last = unchecked_path.size() - 1U;
      const std::size_t refused = std::min(
        last, static_cast<std::size_t>(std::floor(checked * static_cast<double>(last))) + 1U);
      outcome.blocker = first_contact(*unchecked_path[refused], jaw_position_m);
    }
    return outcome;
  }

  SweepOutcome sweep_starts(
    const Eigen::Isometry3d & pregrasp_tool0, const Eigen::Isometry3d & grasp_tool0,
    double jaw_position_m, int seeds, unsigned int rng_seed)
  {
    SweepOutcome outcome;
    for (const auto & start : collision_free_starts(
        pregrasp_tool0, jaw_position_m, seeds, rng_seed))
    {
      ++outcome.starts;
      const StartOutcome result = run_linear(start, grasp_tool0, jaw_position_m);
      if (result.completed) {
        ++outcome.complete;
      } else if (result.unchecked_fraction < minimum_fraction_) {
        ++outcome.kinematic_truncations;
      } else {
        ++outcome.collision_truncations;
        outcome.blockers[result.blocker.empty() ? "unattributed" : result.blocker] += 1;
      }
    }
    return outcome;
  }

  [[nodiscard]] std::vector<std::string> all_contacts(
    const moveit::core::RobotState & state, double jaw_position_m)
  {
    moveit::core::RobotState checked(state);
    apply_jaws(checked, jaw_position_m);
    checked.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 32;
    request.max_contacts_per_pair = 1;
    request.group_name = group_name_;
    collision_detection::CollisionResult result;
    scene_->checkCollision(request, result, checked);
    std::vector<std::string> pairs;
    for (const auto & [pair, contacts] : result.contacts) {
      (void)contacts;
      const std::string first = strip_prefix(pair.first);
      const std::string second = strip_prefix(pair.second);
      pairs.push_back(first < second ? first + " against " + second : second + " against " + first);
    }
    return pairs;
  }

  [[nodiscard]] std::string first_contact(
    const moveit::core::RobotState & state, double jaw_position_m)
  {
    moveit::core::RobotState checked(state);
    apply_jaws(checked, jaw_position_m);
    checked.update();
    collision_detection::CollisionRequest request;
    request.contacts = true;
    request.max_contacts = 8;
    request.max_contacts_per_pair = 1;
    request.group_name = group_name_;
    collision_detection::CollisionResult result;
    scene_->checkCollision(request, result, checked);
    if (result.contacts.empty()) {
      return {};
    }
    // Report the pair deterministically (std::map order) so rows are comparable across a sweep.
    const auto & pair = result.contacts.begin()->first;
    const std::string first = strip_prefix(pair.first);
    const std::string second = strip_prefix(pair.second);
    return first < second ? first + " against " + second : second + " against " + first;
  }

  void apply_jaws(moveit::core::RobotState & state, double jaw_position_m) const
  {
    state.setVariablePosition("left_finger_joint", jaw_position_m);
    state.setVariablePosition("right_finger_joint", jaw_position_m);
  }

  static std::string strip_prefix(const std::string & id)
  {
    const auto slash = id.rfind('/');
    return slash == std::string::npos ? id : id.substr(slash + 1);
  }

  void set_group(const std::string & group) {group_name_ = group;}
  [[nodiscard]] const std::string & group() const {return group_name_;}
  void set_ik_timeout(double seconds) {ik_timeout_s_ = seconds;}
  void set_cartesian_step(double step) {cartesian_step_m_ = step;}
  void set_minimum_fraction(double fraction) {minimum_fraction_ = fraction;}
  [[nodiscard]] double minimum_fraction() const {return minimum_fraction_;}

private:
  rclcpp::Node::SharedPtr node_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  moveit::core::RobotModelPtr model_;
  planning_scene::PlanningScenePtr scene_;
  double padding_m_{0.0};
  std::string group_name_{"manipulator"};
  std::string tip_link_{"tool0"};
  // restocker_moveit_config/config/kinematics.yaml.
  double ik_timeout_s_{0.05};
  // restocker_task_executor MotionGoal defaults.
  double cartesian_step_m_{0.005};
  double minimum_fraction_{0.999};
};

// ---------------------------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------------------------

struct Arguments
{
  std::string mode;
  std::string robot_urdf;
  std::string srdf;
  std::string workcell_urdf;
  std::string gripper_geometry;
  std::string product_catalog;
  std::string workcell_geometry;
  std::string coordinator_config;
  std::vector<double> workcell_pose{0.0, 0.55, 0.75, 0.0, 0.0, 0.0};
  double padding_m{0.0015};
  int seeds{48};
  unsigned int rng_seed{1};
  // Sweep bounds. Meaning depends on the mode; each mode documents its own.
  double from{0.0};
  double to{0.0};
  double step{0.001};
  double along_from{-0.05};
  double along_to{0.35};
  double along_step{0.05};
  std::vector<double> rail_x{-0.6, 0.0, 0.6};
  std::string product_class;
  double neighbour_spacing_m{0.42};
  std::string jaw{"approach"};
  std::string placements;
  std::string neighbour_class;
  // `layout` mode only. A literal product arrangement, `geometry_key:x:y` separated by commas.
  // Each yaw in the coordinator's `grasp.approach_yaws_rad` is measured separately, since the
  // coordinator picks between them by a score the probe does not reproduce.
  std::string products;
  std::vector<double> yaws{-M_PI / 2.0};
  // `envelope` mode only. `lookat` poses the optical frame the way nominal_survey_stations does
  // (boresight at the slice centre); `straightdown` keeps the specification's boresight -Z so a
  // recorded straight-down claim can be tested against the tilted shipped one.
  std::string orientation{"lookat"};
  // `fixture` mode only. One recorded refusal: the start joint positions as `name=value`
  // separated by spaces, the target as `x,y,z,qx,qy,qz,qw`, and the step the collision-free
  // control is re-run at (the shipped 0.005 m is always run too).
  std::string start;
  std::string target;
  double cartesian_step{0.0005};
};

std::string argument_value(int argc, char ** argv, int & index)
{
  if (index + 1 >= argc) {
    fail(std::string("option ") + argv[index] + " needs a value");
  }
  return argv[++index];
}

Arguments parse_arguments(int argc, char ** argv)
{
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--mode") {arguments.mode = argument_value(argc, argv, index);} else if
    (option == "--robot-urdf") {arguments.robot_urdf = argument_value(argc, argv, index);} else if
    (option == "--srdf") {arguments.srdf = argument_value(argc, argv, index);} else if
    (option == "--workcell-urdf")
    {
      arguments.workcell_urdf = argument_value(argc, argv, index);
    } else if (option == "--gripper-geometry") {
      arguments.gripper_geometry = argument_value(argc, argv, index);
    } else if (option == "--product-catalog") {
      arguments.product_catalog = argument_value(argc, argv, index);
    } else if (option == "--workcell-geometry") {
      arguments.workcell_geometry = argument_value(argc, argv, index);
    } else if (option == "--coordinator-config") {
      arguments.coordinator_config = argument_value(argc, argv, index);
    } else if (option == "--workcell-pose") {
      arguments.workcell_pose = parse_doubles(argument_value(argc, argv, index));
    } else if (option == "--padding") {
      arguments.padding_m = std::stod(argument_value(argc, argv, index));
    } else if (option == "--seeds") {
      arguments.seeds = std::stoi(argument_value(argc, argv, index));
    } else if (option == "--rng-seed") {
      arguments.rng_seed = static_cast<unsigned int>(
        std::stoul(argument_value(argc, argv, index)));
    } else if (option == "--from") {
      arguments.from = std::stod(argument_value(argc, argv, index));
    } else if (option == "--to") {
      arguments.to = std::stod(argument_value(argc, argv, index));
    } else if (option == "--step") {
      arguments.step = std::stod(argument_value(argc, argv, index));
    } else if (option == "--along-from") {
      arguments.along_from = std::stod(argument_value(argc, argv, index));
    } else if (option == "--along-to") {
      arguments.along_to = std::stod(argument_value(argc, argv, index));
    } else if (option == "--along-step") {
      arguments.along_step = std::stod(argument_value(argc, argv, index));
    } else if (option == "--rail-x") {
      arguments.rail_x = parse_doubles(argument_value(argc, argv, index));
    } else if (option == "--product-class") {
      arguments.product_class = argument_value(argc, argv, index);
    } else if (option == "--neighbour-spacing") {
      arguments.neighbour_spacing_m = std::stod(argument_value(argc, argv, index));
    } else if (option == "--jaw") {
      arguments.jaw = argument_value(argc, argv, index);
    } else if (option == "--placements") {
      arguments.placements = argument_value(argc, argv, index);
    } else if (option == "--neighbour-class") {
      arguments.neighbour_class = argument_value(argc, argv, index);
    } else if (option == "--products") {
      arguments.products = argument_value(argc, argv, index);
    } else if (option == "--yaws") {
      arguments.yaws = parse_doubles(argument_value(argc, argv, index));
    } else if (option == "--orientation") {
      arguments.orientation = argument_value(argc, argv, index);
    } else if (option == "--start") {
      arguments.start = argument_value(argc, argv, index);
    } else if (option == "--target") {
      arguments.target = argument_value(argc, argv, index);
    } else if (option == "--cartesian-step") {
      arguments.cartesian_step = std::stod(argument_value(argc, argv, index));
    } else {
      fail("unknown option " + option);
    }
  }
  if (arguments.mode.empty()) {
    fail(
      "--mode is required (geometry, reach, corridor, approach, placements, layout, envelope, "
      "fixture)");
  }
  return arguments;
}

void print_provenance(const Arguments & arguments, const Probe & probe)
{
  std::cout << "# planning_probe mode=" << arguments.mode
            << " padding=" << arguments.padding_m
            << " group=" << probe.group()
            << " seeds=" << arguments.seeds
            << " rng_seed=" << arguments.rng_seed
            << " minimum_fraction=" << probe.minimum_fraction() << '\n';
  std::cout << "# robot_urdf=" << arguments.robot_urdf << " srdf=" << arguments.srdf
            << " workcell_urdf=" << arguments.workcell_urdf << '\n';
  std::cout << "# workcell_pose=";
  for (std::size_t index = 0; index < arguments.workcell_pose.size(); ++index) {
    std::cout << (index == 0 ? "" : ",") << arguments.workcell_pose[index];
  }
  std::cout << '\n';
  if (arguments.mode == "envelope") {
    std::cout << "# orientation=" << arguments.orientation << '\n';
  }
}

const ProductShape & find_class_fallback(
  const std::vector<ProductShape> & catalog, const std::string & product_class)
{
  for (const auto & shape : catalog) {
    if (shape.product_class == product_class) {
      return shape;
    }
  }
  fail("no catalogued product of class " + product_class);
}

}  // namespace

int main(int argc, char ** argv)
{
  const Arguments arguments = parse_arguments(argc, argv);

  // RobotModelLoader reads its kinematics plugin from
  // `<robot_description>_kinematics.<group>.kinematics_solver`; with string descriptions the prefix
  // is empty. Set both spellings, otherwise `No kinematics plugins defined` appears and every
  // setFromIK fails, which looks like an unreachable pose.
  std::vector<rclcpp::Parameter> overrides;
  for (const std::string & prefix : {"robot_description_kinematics", "_kinematics"}) {
    for (const std::string & group : {"manipulator", "arm"}) {
      overrides.emplace_back(
        prefix + "." + group + ".kinematics_solver",
        std::string("kdl_kinematics_plugin/KDLKinematicsPlugin"));
      overrides.emplace_back(prefix + "." + group + ".kinematics_solver_search_resolution", 0.005);
      overrides.emplace_back(prefix + "." + group + ".kinematics_solver_timeout", 0.05);
    }
  }
  // The probe's options are not ROS arguments: give rclcpp an empty argv, or rcl rejects every
  // `--mode`-style option.
  const char * empty_argv[] = {"planning_probe"};
  rclcpp::init(1, empty_argv);
  auto node = std::make_shared<rclcpp::Node>(
    "planning_probe",
    rclcpp::NodeOptions().parameter_overrides(overrides).automatically_declare_parameters_from_overrides(
      true));

  const GripperGeometry gripper = load_gripper_geometry(arguments.gripper_geometry);
  const std::vector<ProductShape> catalog = load_product_catalog(arguments.product_catalog);
  const StockRegion region = load_stock_region(arguments.workcell_geometry);
  const GraspConfig grasp = load_grasp_config(arguments.coordinator_config);
  const Eigen::Isometry3d world_from_shelf = pose_from_xyzrpy(arguments.workcell_pose);

  std::cout << std::fixed << std::setprecision(6);

  // `geometry` needs no planner: the corridor half-width and stand-off the generator derives,
  // recomputed from the files the generator reads, so a tree/generator disagreement is not hidden
  // inside a sweep.
  if (arguments.mode == "geometry") {
    const double finger_outer_release = gripper.finger_outer_face_m(gripper.open_target_m);
    std::cout << "# mode=geometry\n";
    std::cout << "quantity,value_m\n";
    std::cout << "finger_half_width," << gripper.finger_half_width_m << '\n';
    std::cout << "jaw_mount_offset," << gripper.jaw_mount_offset_m << '\n';
    std::cout << "inner_gap_at_zero," << gripper.inner_gap_at_zero_m() << '\n';
    std::cout << "release_aperture," << gripper.open_target_m << '\n';
    std::cout << "finger_outer_face_at_release," << finger_outer_release << '\n';
    std::cout << "finger_outer_face_at_release_padded,"
              << finger_outer_release + arguments.padding_m << '\n';
    std::cout << "tool0_from_grasp_center," << gripper.tool0_from_grasp_center_m << '\n';
    std::cout << "pregrasp_distance," << grasp.pregrasp_distance_m << '\n';
    std::cout << "pregrasp_standoff,"
              << gripper.tool0_from_grasp_center_m + grasp.pregrasp_distance_m << '\n';
    std::cout << "stock_region_center_y," << region.center_xyz_m[1] << '\n';
    std::cout << "stock_region_size_y," << region.size_xyz_m[1] << '\n';
    for (const auto & shape : catalog) {
      std::cout << "approach_jaw_position." << shape.geometry_key << ','
                << approach_jaw_position_m(gripper, shape.radius_m) << '\n';
      std::cout << "finger_outer_face_at_approach." << shape.geometry_key << ','
                << gripper.finger_outer_face_m(approach_jaw_position_m(gripper, shape.radius_m))
                << '\n';
      std::cout << "grasp_axial_offset." << shape.geometry_key << ','
                << grasp_axial_offset_m(grasp, shape.height_m) << '\n';
    }
    rclcpp::shutdown();
    return 0;
  }

  Probe probe(node, read_file(arguments.robot_urdf), read_file(arguments.srdf),
    arguments.padding_m);
  probe.add_objects(
    workcell_collision_objects(
      read_file(arguments.workcell_urdf), probe.model()->getModelFrame(), world_from_shelf));
  print_provenance(arguments, probe);

  const Eigen::Vector3d tray_center = world_from_shelf * region.center_xyz_m;
  const double tray_floor_z = tray_center.z() - region.size_xyz_m[2] / 2.0;
  // The approach into the stock tray runs along -y (rail axis y = 0, tray at negative y), so the
  // carriage-to-product direction is (0, -1, 0): the -pi/2 entry in grasp.approach_yaws_rad.
  const double yaw = -M_PI / 2.0;

  const auto jaw_for = [&](const ProductShape & shape) {
      if (arguments.jaw == "release") {
        return gripper.open_target_m;
      }
      if (arguments.jaw == "approach") {
        return approach_jaw_position_m(gripper, shape.radius_m);
      }
      return std::stod(arguments.jaw);
    };

  // Every product mode below places a product at a world (x, y) on the tray floor, derives the
  // coordinator's grasp and pre-grasp poses, and reports how many collision-free pre-grasp starts
  // complete the straight approach. (`envelope` is the exception: no product, no approach -- only
  // station IK; its block is at the end of main.)
  // The pre-grasp, grasp and retract tool0 frames for one product at one approach yaw. The retract
  // is a straight `grasp.retract_distance_m` back along the horizontal approach axis, not a lift
  // (see offset_along_approach in grasp_candidates.cpp).
  struct GraspFrames
  {
    Eigen::Isometry3d pregrasp;
    Eigen::Isometry3d grasp;
    Eigen::Isometry3d retract;
  };
  const auto frames_for = [&](const ProductShape & shape, double x, double y, double at_yaw) {
      const Eigen::Vector3d product_center(x, y, tray_floor_z + shape.height_m / 2.0);
      const Eigen::Vector3d center =
        product_center + grasp_axial_offset_m(grasp, shape.height_m) * Eigen::Vector3d::UnitZ();
      const Eigen::Isometry3d grasp_center = grasp_pose(center, at_yaw);
      Eigen::Isometry3d grasp_center_from_tool0 = Eigen::Isometry3d::Identity();
      grasp_center_from_tool0.translation() =
        Eigen::Vector3d(0.0, 0.0, -gripper.tool0_from_grasp_center_m);
      GraspFrames result;
      result.grasp = grasp_center * grasp_center_from_tool0;
      Eigen::Isometry3d pregrasp_center = grasp_center;
      pregrasp_center.translation() -= grasp.pregrasp_distance_m * grasp_center.linear().col(2);
      result.pregrasp = pregrasp_center * grasp_center_from_tool0;
      Eigen::Isometry3d retract_center = grasp_center;
      retract_center.translation() -= grasp.retract_distance_m * grasp_center.linear().col(2);
      result.retract = retract_center * grasp_center_from_tool0;
      return result;
    };

  const auto poses_for = [&](const ProductShape & shape, double x, double y) {
      const GraspFrames result = frames_for(shape, x, y, yaw);
      return std::make_pair(result.pregrasp, result.grasp);
    };

  // ---- reach ------------------------------------------------------------------------------
  //
  // The pre-grasp reach cliff behind MINIMUM_PREGRASP_REACH_Y_M: an empty tray and a lone product
  // swept across the drawable envelope. `--from`/`--to`/`--step` are the pre-grasp depth, the
  // distance from the rail axis (y = 0) to the pre-grasp tool0 frame.
  if (arguments.mode == "reach") {
    std::cout << "product_class,rail_x_m,pregrasp_depth_m,product_y_m,starts,complete,"
      "kinematic_truncations,collision_truncations,first_blocker\n";
    std::vector<std::string> classes;
    if (arguments.product_class.empty()) {
      for (const auto & shape : catalog) {
        if (std::find(classes.begin(), classes.end(), shape.product_class) == classes.end()) {
          classes.push_back(shape.product_class);
        }
      }
    } else {
      classes.push_back(arguments.product_class);
    }
    const double standoff = gripper.tool0_from_grasp_center_m + grasp.pregrasp_distance_m;
    for (const std::string & product_class : classes) {
      const ProductShape & shape = find_class_fallback(catalog, product_class);
      const double jaw = jaw_for(shape);
      for (const double x : arguments.rail_x) {
        for (double depth = arguments.from; depth <= arguments.to + 1.0e-9;
          depth += arguments.step)
        {
          const double product_y = -(depth + standoff);
          const auto [pregrasp, grasp_target] = poses_for(shape, x, product_y);
          probe.add_product("restocker/object/1", Eigen::Vector3d(
            x, product_y, tray_floor_z + shape.height_m / 2.0), shape);
          const SweepOutcome outcome = probe.sweep_starts(
            pregrasp, grasp_target, jaw, arguments.seeds, arguments.rng_seed);
          probe.remove_object("restocker/object/1");
          std::string blocker;
          if (!outcome.blockers.empty()) {
            blocker = outcome.blockers.begin()->first;
          }
          std::cout << product_class << ',' << x << ',' << depth << ',' << product_y << ','
                    << outcome.starts << ',' << outcome.complete << ','
                    << outcome.kinematic_truncations << ',' << outcome.collision_truncations
                    << ',' << blocker << '\n' << std::flush;
        }
      }
    }
    rclcpp::shutdown();
    return 0;
  }

  // ---- corridor ---------------------------------------------------------------------------
  //
  // The tube the arm sweeps to reach one product, measured by standing a neighbour in it. The
  // target is held at a placement the reach sweep found clear; the neighbour moves across the
  // approach (`--from`/`--to`/`--step`, world x offset) and along it (`--along-*`, world y offset,
  // positive towards the robot). For each along-offset, the first across-offset at which every
  // collision-free start completes is the implied half-width plus the neighbour's radius.
  if (arguments.mode == "corridor") {
    std::cout << "target_key,neighbour_key,along_m,across_m,implied_half_width_m,starts,"
      "complete,kinematic_truncations,collision_truncations,first_blocker\n";
    const ProductShape & target = find_class_fallback(
      catalog, arguments.product_class.empty() ? "can" : arguments.product_class);
    const double jaw = jaw_for(target);
    // Mid-tray in x, deep enough in y that the pre-grasp is well past the cliff.
    const double target_x = 0.0;
    const double target_y = tray_center.y() - 0.10;
    const auto [pregrasp, grasp_target] = poses_for(target, target_x, target_y);
    probe.add_product("restocker/object/1", Eigen::Vector3d(
      target_x, target_y, tray_floor_z + target.height_m / 2.0), target);
    const ProductShape & neighbour = find_class_fallback(
      catalog, arguments.neighbour_class.empty() ? "can" : arguments.neighbour_class);
    for (double along = arguments.along_from; along <= arguments.along_to + 1.0e-9;
      along += arguments.along_step)
    {
      for (double across = arguments.from; across <= arguments.to + 1.0e-9;
        across += arguments.step)
      {
        const Eigen::Vector3d neighbour_center(
          target_x + across, target_y + along, tray_floor_z + neighbour.height_m / 2.0);
        probe.add_product("restocker/object/2", neighbour_center, neighbour);
        const SweepOutcome outcome = probe.sweep_starts(
          pregrasp, grasp_target, jaw, arguments.seeds, arguments.rng_seed);
        probe.remove_object("restocker/object/2");
        std::string blocker;
        if (!outcome.blockers.empty()) {
          blocker = outcome.blockers.begin()->first;
        }
        // Implied half-width: the first across-distance at which every start completes, less the
        // neighbour's radius.
        std::cout << target.geometry_key << ',' << neighbour.geometry_key << ',' << along << ','
                  << across << ',' << across - neighbour.radius_m << ',' << outcome.starts << ','
                  << outcome.complete << ',' << outcome.kinematic_truncations << ','
                  << outcome.collision_truncations << ',' << blocker << '\n' << std::flush;
      }
    }
    probe.remove_object("restocker/object/1");
    rclcpp::shutdown();
    return 0;
  }

  // ---- approach ---------------------------------------------------------------------------
  //
  // Check that a truncating approach is not a starting-configuration problem: sweep a lone product
  // across the drawable y interval with neighbours at the fixed scenario's tray spacing, counting
  // placements where every collision-free start completes.
  if (arguments.mode == "approach") {
    std::cout << "product_class,product_x_m,product_y_m,pregrasp_depth_m,starts,complete,"
      "kinematic_truncations,collision_truncations,first_blocker\n";
    const ProductShape & shape = find_class_fallback(
      catalog, arguments.product_class.empty() ? "small_bottle" : arguments.product_class);
    const double jaw = jaw_for(shape);
    const double standoff = gripper.tool0_from_grasp_center_m + grasp.pregrasp_distance_m;
    // Neighbours of the same class at the scenario's spacing, standing level with the target.
    for (double y = arguments.from; y <= arguments.to + 1.0e-9; y += arguments.step) {
      for (const double x : arguments.rail_x) {
        probe.add_product("restocker/object/1", Eigen::Vector3d(
          x, y, tray_floor_z + shape.height_m / 2.0), shape);
        probe.add_product("restocker/object/2", Eigen::Vector3d(
          x - arguments.neighbour_spacing_m, y, tray_floor_z + shape.height_m / 2.0), shape);
        probe.add_product("restocker/object/3", Eigen::Vector3d(
          x + arguments.neighbour_spacing_m, y, tray_floor_z + shape.height_m / 2.0), shape);
        const auto [pregrasp, grasp_target] = poses_for(shape, x, y);
        const SweepOutcome outcome = probe.sweep_starts(
          pregrasp, grasp_target, jaw, arguments.seeds, arguments.rng_seed);
        probe.remove_object("restocker/object/1");
        probe.remove_object("restocker/object/2");
        probe.remove_object("restocker/object/3");
        std::string blocker;
        if (!outcome.blockers.empty()) {
          blocker = outcome.blockers.begin()->first;
        }
        std::cout << shape.product_class << ',' << x << ',' << y << ',' << -y - standoff << ','
                  << outcome.starts << ',' << outcome.complete << ','
                  << outcome.kinematic_truncations << ',' << outcome.collision_truncations << ','
                  << blocker << '\n' << std::flush;
      }
    }
    rclcpp::shutdown();
    return 0;
  }

  // ---- placements -------------------------------------------------------------------------
  //
  // Feasibility of whole generated scenarios, read from the CSV
  // `tools/diagnostics/generator_sweep.py --mode placements` writes: for each product of each
  // seed, with the seed's other products in the scene, how many collision-free pre-grasp starts
  // complete the approach. A product with no collision-free pre-grasp is counted separately from
  // one whose approach truncates; both surface as a pre-grasp planning failure in a live run.
  if (arguments.mode == "placements") {
    std::ifstream placements(arguments.placements);
    if (!placements) {
      fail("cannot read " + arguments.placements);
    }
    std::map<std::string, const ProductShape *> by_key;
    for (const auto & shape : catalog) {
      by_key[shape.geometry_key] = &shape;
    }

    struct Placement
    {
      const ProductShape * shape;
      double x;
      double y;
    };
    std::map<long, std::vector<Placement>> scenarios;
    std::string line;
    while (std::getline(placements, line)) {
      if (line.empty() || line[0] == '#' || line.rfind("scenario", 0) == 0) {
        continue;
      }
      std::stringstream fields(line);
      std::string scenario_id;
      std::string product_index;
      std::string key;
      std::string x;
      std::string y;
      std::getline(fields, scenario_id, ',');
      std::getline(fields, product_index, ',');
      std::getline(fields, key, ',');
      std::getline(fields, x, ',');
      std::getline(fields, y, ',');
      const auto found = by_key.find(key);
      if (found == by_key.end()) {
        fail("placement CSV names an uncatalogued geometry_key: " + key);
      }
      scenarios[std::stol(scenario_id)].push_back(
        Placement{found->second, std::stod(x), std::stod(y)});
    }

    std::cout << "seed,product,geometry_key,x_m,y_m,starts,complete,kinematic_truncations,"
      "collision_truncations,first_blocker\n";
    for (const auto & [seed, products] : scenarios) {
      for (std::size_t index = 0; index < products.size(); ++index) {
        probe.add_product(
          "restocker/object/" + std::to_string(index + 1),
          Eigen::Vector3d(
            products[index].x, products[index].y,
            tray_floor_z + products[index].shape->height_m / 2.0),
          *products[index].shape);
      }
      for (std::size_t index = 0; index < products.size(); ++index) {
        const Placement & product = products[index];
        const auto [pregrasp, grasp_target] = poses_for(*product.shape, product.x, product.y);
        const SweepOutcome outcome = probe.sweep_starts(
          pregrasp, grasp_target, jaw_for(*product.shape), arguments.seeds, arguments.rng_seed);
        std::string blocker;
        if (!outcome.blockers.empty()) {
          blocker = outcome.blockers.begin()->first;
        }
        std::cout << seed << ',' << index << ',' << product.shape->geometry_key << ','
                  << product.x << ',' << product.y << ',' << outcome.starts << ','
                  << outcome.complete << ',' << outcome.kinematic_truncations << ','
                  << outcome.collision_truncations << ',' << blocker << '\n' << std::flush;
      }
      for (std::size_t index = 0; index < products.size(); ++index) {
        probe.remove_object("restocker/object/" + std::to_string(index + 1));
      }
    }
    rclcpp::shutdown();
    return 0;
  }

  // ---- layout -----------------------------------------------------------------------------
  //
  // One literal product arrangement (the fixed scenario, or a candidate spacing), measured at every
  // approach yaw the coordinator generates (`grasp.approach_yaws_rad` holds three).
  //
  // Two segments per (product, yaw), both collision-checked at 0.999:
  //   * approach: pre-grasp tool0 -> grasp tool0, from every collision-free pre-grasp start;
  //   * retract:  grasp tool0 -> grasp - retract_distance along the same axis, started from the
  //     state the approach ended in.
  //
  // The retract is measured with the target still in the scene, unattached: it tests the arm
  // against neighbours and does not model the carried product. Rule 2 applies to both columns:
  // `starts` and `complete` are reported separately and a segment is clear only when equal.
  if (arguments.mode == "layout") {
    if (arguments.products.empty()) {
      fail("--mode layout needs --products geometry_key:x:y[,geometry_key:x:y...]");
    }
    std::map<std::string, const ProductShape *> by_key;
    for (const auto & shape : catalog) {
      by_key[shape.geometry_key] = &shape;
    }
    struct Placement
    {
      const ProductShape * shape;
      double x;
      double y;
    };
    std::vector<Placement> layout;
    std::stringstream entries(arguments.products);
    std::string entry;
    while (std::getline(entries, entry, ',')) {
      if (entry.empty()) {
        continue;
      }
      std::stringstream fields(entry);
      std::string key;
      std::string x;
      std::string y;
      std::getline(fields, key, ':');
      std::getline(fields, x, ':');
      std::getline(fields, y, ':');
      const auto found = by_key.find(key);
      if (found == by_key.end()) {
        fail("--products names an uncatalogued geometry_key: " + key);
      }
      layout.push_back(Placement{found->second, std::stod(x), std::stod(y)});
    }
    if (layout.empty()) {
      fail("--products parsed to no placements");
    }

    for (std::size_t index = 0; index < layout.size(); ++index) {
      probe.add_product(
        "restocker/object/" + std::to_string(index + 1),
        Eigen::Vector3d(
          layout[index].x, layout[index].y,
          tray_floor_z + layout[index].shape->height_m / 2.0),
        *layout[index].shape);
    }

    std::cout << "product,geometry_key,x_m,y_m,yaw_rad,ik_solutions,pregrasp_blocker,"
      "approach_starts,approach_complete,"
      "approach_kinematic,approach_collision,approach_blocker,retract_starts,retract_complete,"
      "retract_kinematic,retract_collision,retract_blocker\n";
    for (std::size_t index = 0; index < layout.size(); ++index) {
      const Placement & product = layout[index];
      const double jaw = jaw_for(*product.shape);
      for (const double at_yaw : arguments.yaws) {
        const GraspFrames motion = frames_for(*product.shape, product.x, product.y, at_yaw);
        SweepOutcome approach_outcome;
        SweepOutcome retract_outcome;
        int ik_solutions = 0;
        std::map<std::string, int> pregrasp_blockers;
        for (const auto & start : probe.collision_free_starts(
            motion.pregrasp, jaw, arguments.seeds, arguments.rng_seed, &ik_solutions,
            &pregrasp_blockers))
        {
          ++approach_outcome.starts;
          moveit::core::RobotState at_grasp(start);
          const StartOutcome approached = probe.run_linear(start, motion.grasp, jaw, &at_grasp);
          if (!approached.completed) {
            if (approached.unchecked_fraction < probe.minimum_fraction()) {
              ++approach_outcome.kinematic_truncations;
            } else {
              ++approach_outcome.collision_truncations;
              approach_outcome.blockers[
                approached.blocker.empty() ? "unattributed" : approached.blocker] += 1;
            }
            continue;
          }
          ++approach_outcome.complete;
          ++retract_outcome.starts;
          const StartOutcome retracted = probe.run_linear(at_grasp, motion.retract, jaw);
          if (retracted.completed) {
            ++retract_outcome.complete;
          } else if (retracted.unchecked_fraction < probe.minimum_fraction()) {
            ++retract_outcome.kinematic_truncations;
          } else {
            ++retract_outcome.collision_truncations;
            retract_outcome.blockers[
              retracted.blocker.empty() ? "unattributed" : retracted.blocker] += 1;
          }
        }
        // Ordered by how many refused configurations carried the pair, so the row leads with the
        // actual refuser.
        std::vector<std::pair<std::string, int>> ranked(
          pregrasp_blockers.begin(), pregrasp_blockers.end());
        std::sort(
          ranked.begin(), ranked.end(), [](const auto & left, const auto & right) {
            return left.second != right.second ? left.second > right.second :
            left.first < right.first;
          });
        std::string pregrasp_blocker;
        for (std::size_t rank = 0; rank < ranked.size() && rank < 3U; ++rank) {
          pregrasp_blocker +=
            (rank == 0 ? "" : "; ") + ranked[rank].first + " x" + std::to_string(
            ranked[rank].second);
        }
        std::string approach_blocker;
        if (!approach_outcome.blockers.empty()) {
          approach_blocker = approach_outcome.blockers.begin()->first;
        }
        std::string retract_blocker;
        if (!retract_outcome.blockers.empty()) {
          retract_blocker = retract_outcome.blockers.begin()->first;
        }
        std::cout << index << ',' << product.shape->geometry_key << ',' << product.x << ','
                  << product.y << ',' << at_yaw << ','
                  << ik_solutions << ',' << pregrasp_blocker << ','
                  << approach_outcome.starts << ',' << approach_outcome.complete << ','
                  << approach_outcome.kinematic_truncations << ','
                  << approach_outcome.collision_truncations << ',' << approach_blocker << ','
                  << retract_outcome.starts << ',' << retract_outcome.complete << ','
                  << retract_outcome.kinematic_truncations << ','
                  << retract_outcome.collision_truncations << ',' << retract_blocker
                  << '\n' << std::flush;
      }
    }
    rclcpp::shutdown();
    return 0;
  }

  // ---- fixture ----------------------------------------------------------------------------
  //
  // Replay one recorded refusal. Every linear refusal the motion port reports now carries
  // `fixture: start=[name=value ...] target xyz=(x, y, z) quat=(qx, qy, qz, qw) step=<s> m`
  // (Card 046), and this mode re-runs that exact line through the two computations the port
  // makes: collision-checked, then collision-free. It runs the recorded step and
  // `--cartesian-step` (10x finer by default), which is what separates a kinematic limit from
  // an interpolation artefact - a stop the finer step clears was numerical, one that holds at
  // both was not. The jaws come from `--start` when the fixture carries them (the port records
  // them), otherwise the release aperture, and for the kinematic verdict the variable closest
  // to one of its own bounds at the stop is named, since that is the joint a limit would pin.
  //
  // Two caveats a replay has to respect, both measured on this card:
  //
  //   * CartesianInterpolator makes ONE IK attempt per waypoint (it passes timeout 0.0 on
  //     purpose, so random restarts cannot create joint-space jumps), so a stop can be a single
  //     attempt that did not converge rather than a line no configuration can follow. Run the
  //     same fixture repeatedly: a stop that flips between fractions is that knife edge, a stop
  //     that reproduces every time at both steps is the kinematic limit. Raising the solver's
  //     configured timeout does not change this either: the interpolator does not use it.
  //   * The checked run is only as faithful as the scene this probe rebuilt from the scenario
  //     files: the console line carries no scene, so the collision-free control is the
  //     replayable half and the checked half is an approximation of the one that ran.
  if (arguments.mode == "fixture") {
    if (arguments.start.empty()) {
      fail("--mode fixture needs --start \"name=value name=value ...\"");
    }
    if (arguments.target.empty()) {
      fail("--mode fixture needs --target x,y,z,qx,qy,qz,qw");
    }
    if (!(arguments.cartesian_step > 0.0)) {
      fail("--cartesian-step must be positive");
    }
    const auto & known_variables = probe.model()->getVariableNames();
    moveit::core::RobotState state(probe.model());
    state.setToDefaultValues();
    std::stringstream assignments(arguments.start);
    std::string assignment;
    while (std::getline(assignments, assignment, ' ')) {
      if (assignment.empty()) {
        continue;
      }
      const auto equal = assignment.find('=');
      if (equal == std::string::npos || equal == 0U) {
        fail("--start entries are name=value: " + assignment);
      }
      const std::string name = assignment.substr(0, equal);
      if (std::find(
          known_variables.begin(), known_variables.end(), name) == known_variables.end())
      {
        fail("--start names an unknown variable: " + name);
      }
      try {
        state.setVariablePosition(name, std::stod(assignment.substr(equal + 1U)));
      } catch (const std::exception &) {
        fail("--start value is not a number: " + assignment);
      }
    }
    state.update();
    const auto numbers = parse_doubles(arguments.target);
    if (numbers.size() != 7U) {
      fail("--target needs x,y,z,qx,qy,qz,qw (seven comma-separated numbers)");
    }
    const Eigen::Quaterniond orientation(numbers[6], numbers[3], numbers[4], numbers[5]);
    if (std::abs(orientation.norm() - 1.0) > 1.0e-4) {
      fail("--target quaternion is not unit length");
    }
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(numbers[0], numbers[1], numbers[2]);
    target.linear() = orientation.normalized().toRotationMatrix();
    const bool start_carries_jaws =
      arguments.start.find("left_finger_joint=") != std::string::npos;
    const double jaw = start_carries_jaws ?
      state.getVariablePosition("left_finger_joint") : gripper.open_target_m;
    const auto * group = probe.model()->getJointModelGroup(probe.group());
    if (group == nullptr) {
      fail("planning group " + probe.group() + " is absent from the robot model");
    }

    std::cout << "# mode=fixture jaw_position=" << std::setprecision(9) << jaw << '\n';
    std::cout << "step_m,checked,unchecked,verdict,limiting_variable,limiting_margin\n";
    std::vector<double> steps{0.005, arguments.cartesian_step};
    if (arguments.cartesian_step == 0.005) {
      steps.pop_back();
    }
    for (const double step : steps) {
      probe.set_cartesian_step(step);
      moveit::core::RobotState reached(probe.model());
      const StartOutcome outcome = probe.run_linear(state, target, jaw, &reached);
      const bool kinematic = !outcome.completed &&
        outcome.unchecked_fraction < probe.minimum_fraction();
      const char * verdict = outcome.completed ? "complete" :
        (kinematic ? "kinematic" : "collision");
      std::string limiting_variable;
      double limiting_margin = 0.0;
      if (kinematic) {
        bool have_limit = false;
        for (const auto & variable : group->getVariableNames()) {
          const auto & bounds = probe.model()->getVariableBounds(variable);
          if (!bounds.position_bounded_) {
            continue;
          }
          const double value = reached.getVariablePosition(variable);
          const double margin = std::min(
            value - bounds.min_position_, bounds.max_position_ - value);
          if (!have_limit || margin < limiting_margin) {
            have_limit = true;
            limiting_margin = margin;
            limiting_variable = variable;
          }
        }
      }
      std::cout << std::setprecision(9) << step << ',' << outcome.checked_fraction << ',' <<
        outcome.unchecked_fraction << ',' << verdict << ',' << limiting_variable << ',';
      if (!limiting_variable.empty()) {
        std::cout << limiting_margin;
      }
      std::cout << '\n' << std::flush;
    }
    rclcpp::shutdown();
    return 0;
  }

  // ---- envelope ---------------------------------------------------------------------------
  //
  // The tray-station IK envelope of Milestone 10 section 2, re-derived under the convention the
  // shipped stations command (the original /compute_ik sweep never recorded one):
  //
  //   * A station is an optical-frame pose: the camera at (slice_x, y, z) in world, boresight
  //     aimed at the tray slice centre, UPRIGHT against the shelf's +Z (Card 098's corrected
  //     look_at_pose contract, specs/camera-viewpoint-orientation.md) — for this -Y-looking
  //     boresight that lands image-right on -X, the 180-degree flip from the hint-signed
  //     convention this envelope was originally measured under. Eye positions are unchanged.
  //   * That pose becomes the commanded goal through the fixed wrist-camera mount, exactly as
  //     WristCameraMount::tool0_goal_for: goal = optical_pose * (tool0_from_optical)^-1.
  //   * Reachability is IK on `manipulator` at tip tool0 with collision checking on, which is
  //     what test_wrist_viewpoint_runtime asks /compute_ik for.
  //
  // `--from`/`--to`/`--step` sweep the camera world y and `--along-from`/`--along-to`/
  // `--along-step` the camera world z; the slice x values come from the surveyed tray (two
  // slices, as tray_station_count ships), not from an argument. `--orientation` switches the
  // boresight: `lookat` aims at the slice centre (the shipped convention), `straightdown` holds
  // world -Z (the specification's, kept so a straight-down claim can be tested against the
  // tilted shipped one). Jaws sit at the open release aperture, the widest finger position, so
  // the envelope is not flattered by a closed gripper. A point reports how many of `--seeds`
  // random starts land on a collision-free IK solution; "solution exists" anywhere in the report
  // means at least one.
  if (arguments.mode == "envelope") {
    if (arguments.step <= 0.0 || arguments.along_step <= 0.0 || arguments.from > arguments.to ||
      arguments.along_from > arguments.along_to)
    {
      fail("--mode envelope needs a positive step and from <= to on both axes");
    }
    if (arguments.orientation != "lookat" && arguments.orientation != "straightdown") {
      fail("--orientation must be lookat or straightdown, not " + arguments.orientation);
    }
    // The mount is a chain of fixed joints, so tool0 <- optical is the same in every state and
    // can be read straight off the loaded model at its default state.
    moveit::core::RobotState base(probe.model());
    base.setToDefaultValues();
    base.update();
    const Eigen::Isometry3d world_from_tool0 = base.getGlobalLinkTransform("tool0");
    const Eigen::Isometry3d world_from_optical =
      base.getGlobalLinkTransform("wrist_camera_optical_frame");
    const Eigen::Isometry3d tool0_from_optical = world_from_tool0.inverse() * world_from_optical;
    // The up axis the stations' upright roll is pinned against: shelf +Z carried into world.
    const Eigen::Vector3d right_in_world = world_from_shelf.linear() * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up_in_world = world_from_shelf.linear() * Eigen::Vector3d::UnitZ();
    const std::size_t slices = 2U;
    const double slice_width = region.size_xyz_m[0] / static_cast<double>(slices);
    const double jaw = gripper.open_target_m;
    std::cout << "slice_x,camera_y,camera_z,ik_solutions,collision_free_starts,first_blocker\n";
    for (std::size_t slice = 0U; slice < slices; ++slice) {
      const double slice_x_in_shelf =
        region.center_xyz_m[0] - (region.size_xyz_m[0] / 2.0) +
        (slice_width * (static_cast<double>(slice) + 0.5));
      const Eigen::Vector3d target_in_shelf(
        slice_x_in_shelf, region.center_xyz_m[1], region.center_xyz_m[2]);
      const Eigen::Vector3d target_in_world = world_from_shelf * target_in_shelf;
      const double slice_x_in_world = target_in_world.x();
      for (double camera_y = arguments.from; camera_y <= arguments.to + 1.0e-9;
        camera_y += arguments.step)
      {
        for (double camera_z = arguments.along_from; camera_z <= arguments.along_to + 1.0e-9;
          camera_z += arguments.along_step)
        {
          const Eigen::Vector3d eye(slice_x_in_world, camera_y, camera_z);
          // look_at_pose, restated under Card 098's upright contract: when the boresight is not
          // parallel to the up axis, image-down is the negated projection of up off the boresight
          // and +X completes the frame (up decides the roll). straightdown holds world -Z —
          // parallel to up — and falls back to the right preference exactly as look_at_pose does;
          // a degenerate (eye == target) is off this grid.
          const Eigen::Vector3d axis_z =
            arguments.orientation == "straightdown" ?
            -Eigen::Vector3d::UnitZ() : (target_in_world - eye).normalized();
          Eigen::Vector3d axis_x;
          const Eigen::Vector3d up_perp =
            up_in_world - up_in_world.dot(axis_z) * axis_z;
          if (up_perp.norm() >= 1.0e-6) {
            axis_x = (-up_perp.normalized()).cross(axis_z);
          } else {
            axis_x = right_in_world - right_in_world.dot(axis_z) * axis_z;
            if (axis_x.norm() < 1.0e-6) {
              fail("envelope grid produced a degenerate look-at roll at z " +
                   std::to_string(camera_z));
            }
            axis_x.normalize();
          }
          Eigen::Isometry3d optical_in_world = Eigen::Isometry3d::Identity();
          optical_in_world.linear().col(0) = axis_x;
          optical_in_world.linear().col(1) = axis_z.cross(axis_x);
          optical_in_world.linear().col(2) = axis_z;
          optical_in_world.translation() = eye;
          // reference <- tool0 = (reference <- optical) * (optical <- tool0), the same
          // composition WristCameraMount::tool0_goal_for performs.
          const Eigen::Isometry3d tool0_in_world =
            optical_in_world * tool0_from_optical.inverse();
          int ik_solutions = 0;
          std::map<std::string, int> blockers;
          const std::vector<moveit::core::RobotState> starts =
            probe.collision_free_starts(
            tool0_in_world, jaw, arguments.seeds, arguments.rng_seed, &ik_solutions, &blockers);
          std::string first_blocker;
          if (!blockers.empty()) {
            first_blocker = std::max_element(
              blockers.begin(), blockers.end(),
              [](const auto & left, const auto & right) {
                return left.second != right.second ? left.second < right.second :
                left.first > right.first;
              })->first;
          }
          std::cout << slice_x_in_world << ',' << camera_y << ',' << camera_z << ','
                    << ik_solutions << ',' << starts.size() << ',' << first_blocker
                    << '\n' << std::flush;
        }
      }
    }
    rclcpp::shutdown();
    return 0;
  }

  fail("unknown mode " + arguments.mode);
}
