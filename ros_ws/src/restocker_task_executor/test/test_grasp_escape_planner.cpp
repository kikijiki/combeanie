// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT


// Milestone 10 §6 (Card 062): the grasp escape's straight line on the real restocker model and
// MoveIt's own collision checker. The start state reproduces Card 010 SC-004 slot 16: the tool at
// that run's grasp pose, the jaws at the approach clearance, and the product 1 mm inside the left
// finger (planner link padding included), so every plan from it was START_STATE_IN_COLLISION.

#include <geometric_shapes/shapes.h>
#include <gtest/gtest.h>
#include <random_numbers/random_numbers.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/allowed_collision_matrix.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>

#include "restocker_task_executor/grasp_escape_planner.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr double kPlannerPaddingM = 0.0015;  // move_group's robot link padding
constexpr double kApproachClearanceM = 0.029;  // the closed candidate's open width
constexpr double kFingerHalfThicknessM = 0.009;  // gripper_geometry.yaml finger size_xyz_m[1] / 2
constexpr double kProductRadiusM = 0.045;  // the 0.090 m catalogued large bottle
constexpr double kProductHeightM = 0.12;
const char * const kTarget = "restocker/object/1";

std::string run(const std::string & command)
{
  std::array<char, 4096> buffer{};
  std::string output;
  FILE * pipe = popen(command.c_str(), "r");
  if (pipe == nullptr) {
    return output;
  }
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe) != nullptr) {
    output += buffer.data();
  }
  pclose(pipe);
  return output;
}

std::string read_file(const std::string & path)
{
  std::ifstream stream(path);
  std::stringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

Eigen::Isometry3d tool0_pose(double x, double y, double z)
{
  // Slot 16's grasp and pre-grasp orientation, quat (x, y, z, w) = (0.5, -0.5, 0.5, 0.5).
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = Eigen::Quaterniond(0.5, 0.5, -0.5, 0.5).normalized().toRotationMatrix();
  return pose;
}

class GraspEscapePlannerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
    const auto description = ament_index_cpp::get_package_share_directory("restocker_description");
    const auto config = ament_index_cpp::get_package_share_directory("restocker_moveit_config");
    const std::string urdf =
      run("xacro " + description + "/urdf/restocker_planning.urdf.xacro 2>/dev/null");
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    options.parameter_overrides(
      {{"robot_description", urdf},
        {"robot_description_semantic", read_file(config + "/config/restocker.srdf")},
        {"robot_description_kinematics.manipulator.kinematics_solver",
          "kdl_kinematics_plugin/KDLKinematicsPlugin"},
        {"robot_description_kinematics.manipulator.kinematics_solver_search_resolution", 0.005},
        {"robot_description_kinematics.manipulator.kinematics_solver_timeout", 0.05}});
    node_ = std::make_shared<rclcpp::Node>("grasp_escape_planner_test", options);
    loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    model_ = loader_->getModel();
  }

  static void TearDownTestSuite()
  {
    // The model owns kinematics plugins the loader's class loader created: release it first.
    model_.reset();
    loader_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    ASSERT_TRUE(model_) << "the restocker planning model did not load";
    group_ = model_->getJointModelGroup("manipulator");
    ASSERT_NE(group_, nullptr);
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    scene_->getCollisionEnvNonConst()->setPadding(kPlannerPaddingM);
    start_ = std::make_unique<moveit::core::RobotState>(model_);
    start_->setToDefaultValues();
    start_->setVariablePosition("left_finger_joint", kApproachClearanceM);
    start_->setVariablePosition("right_finger_joint", kApproachClearanceM);
    // Production reaches the grasp by a straight approach from the pre-grasp standoff; build the
    // start the same way, from a fixed seed, so the arm is on the branch the approach leaves.
    random_numbers::RandomNumberGenerator rng(62U);
    bool solved = false;
    for (int attempt = 0; attempt < 20 && !solved; ++attempt) {
      start_->setToRandomPositions(group_, rng);
      if (!start_->setFromIK(group_, standoff_, "tool0", 0.5) || !valid_empty_scene()) {
        continue;
      }
      std::vector<moveit::core::RobotStatePtr> approach;
      const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
        start_.get(), group_, approach, model_->getLinkModel("tool0"), grasp_, true,
        moveit::core::MaxEEFStep(0.005), moveit::core::CartesianPrecision{},
        moveit::core::GroupStateValidityCallbackFn(), kinematics::KinematicsQueryOptions());
      if (fraction < 0.999 || approach.empty()) {
        continue;
      }
      *start_ = *approach.back();
      solved = valid_empty_scene();
    }
    ASSERT_TRUE(solved) << "no straight approach from slot 16's pre-grasp to its grasp";
    start_->update();
    grasp_center_ = start_->getGlobalLinkTransform("grasp_center");
    left_axis_ = grasp_center_.linear() * Eigen::Vector3d::UnitY();
    const double left_offset =
      (start_->getGlobalLinkTransform("left_finger").translation() - grasp_center_.translation())
      .dot(left_axis_);
    left_inner_face_m_ = left_offset - kFingerHalfThicknessM;
    // A side grasp: the fingers close horizontally around an upright product.
    ASSERT_LT(std::abs(left_axis_.z()), 1e-4);
    ASSERT_GT(left_inner_face_m_, kProductRadiusM);
  }

  bool valid_empty_scene()
  {
    start_->update();
    return !scene_->isStateColliding(*start_, "", false);
  }

  // An upright cylinder whose surface sits `penetration_m` inside the left finger's inner face.
  void add_product(const std::string & id, double penetration_m, double radius_m)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = grasp_center_.translation() +
      left_axis_ * (left_inner_face_m_ + penetration_m - radius_m);
    scene_->getWorldNonConst()->addToObject(
      id, std::make_shared<shapes::Cylinder>(radius_m, kProductHeightM), pose);
  }

  void add_box(const std::string & id, const Eigen::Vector3d & size, const Eigen::Vector3d & at)
  {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = at;
    scene_->getWorldNonConst()->addToObject(
      id, std::make_shared<shapes::Box>(size.x(), size.y(), size.z()), pose);
  }

  GraspEscapeLine escape() {return escape_to(standoff_);}

  GraspEscapeLine escape_to(const Eigen::Isometry3d & standoff)
  {
    GraspEscape request;
    request.planning_frame_from_standoff_tool0 = standoff;
    request.target_object_id = kTarget;
    request.tolerated_links = {"left_finger", "right_finger"};
    return plan_grasp_escape_line(
      *scene_, *start_, group_, model_->getLinkModel("tool0"), request, 0.005, 0.999);
  }

  static rclcpp::Node::SharedPtr node_;
  static std::unique_ptr<robot_model_loader::RobotModelLoader> loader_;
  static moveit::core::RobotModelPtr model_;
  const Eigen::Isometry3d grasp_ = tool0_pose(0.3964, -0.5279, 0.7800);
  const Eigen::Isometry3d standoff_ = tool0_pose(0.3964, -0.3479, 0.7800);
  const moveit::core::JointModelGroup * group_{nullptr};
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<moveit::core::RobotState> start_;
  Eigen::Isometry3d grasp_center_{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d left_axis_{Eigen::Vector3d::Zero()};
  double left_inner_face_m_{0.0};
};

rclcpp::Node::SharedPtr GraspEscapePlannerTest::node_;
std::unique_ptr<robot_model_loader::RobotModelLoader> GraspEscapePlannerTest::loader_;
moveit::core::RobotModelPtr GraspEscapePlannerTest::model_;

TEST_F(GraspEscapePlannerTest, LeavesTheSlot16StartStateToleratingOnlyTheLeftFinger)
{
  add_product(kTarget, 0.001, kProductRadiusM);
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 16U;
  collision_detection::CollisionResult result;
  scene_->checkCollision(request, result, *start_);
  ASSERT_TRUE(result.collision) << "the fixture must reproduce slot 16's start-state collision";
  ASSERT_EQ(result.contacts.size(), 1U);

  const auto line = escape();
  ASSERT_TRUE(line.planned) << line.detail;
  ASSERT_EQ(line.tolerated_pairs.size(), 1U);
  EXPECT_EQ(line.tolerated_pairs.front(), (CollisionContactPair{kTarget, "left_finger"}));
  ASSERT_GT(line.waypoints.size(), 2U);
  EXPECT_TRUE(
    line.waypoints.back()->getGlobalLinkTransform("tool0").isApprox(standoff_, 1e-3));
  // The scene's own matrix never learned the exemption.
  collision_detection::AllowedCollision::Type type{};
  EXPECT_FALSE(
    scene_->getAllowedCollisionMatrix().getEntry(kTarget, "left_finger", type) &&
    type == collision_detection::AllowedCollision::ALWAYS);
  EXPECT_NE(
    line.detail.find("tolerating only restocker/object/1 against left_finger"),
    std::string::npos) << line.detail;
}

TEST_F(GraspEscapePlannerTest, AContactFreeGraspEscapesWithNothingTolerated)
{
  add_product(kTarget, -0.004, kProductRadiusM);
  const auto line = escape();
  ASSERT_TRUE(line.planned) << line.detail;
  EXPECT_TRUE(line.tolerated_pairs.empty());
}

TEST_F(GraspEscapePlannerTest, RefusesWhenANeighbourIsAlsoInContact)
{
  add_product(kTarget, 0.001, kProductRadiusM);
  // A neighbouring product pressed against the outside of the left finger.
  const double finger_outer_m = left_inner_face_m_ + 2.0 * kFingerHalfThicknessM;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = grasp_center_.translation() + left_axis_ * (finger_outer_m + 0.03);
  scene_->getWorldNonConst()->addToObject(
    "restocker/object/2", std::make_shared<shapes::Cylinder>(0.0315, kProductHeightM), pose);
  const auto line = escape();
  EXPECT_FALSE(line.planned);
  EXPECT_NE(line.detail.find("outside the grasp escape's scope"), std::string::npos) <<
    line.detail;
  EXPECT_NE(line.detail.find("restocker/object/2"), std::string::npos) << line.detail;
}

TEST_F(GraspEscapePlannerTest, RefusesALineAnObstacleBlocks)
{
  add_product(kTarget, 0.001, kProductRadiusM);
  // A small block just behind the left finger's base, clear of everything at the start, in the
  // volume the finger sweeps on its way back to the standoff.
  const Eigen::Vector3d back = (standoff_.translation() - grasp_.translation()).normalized();
  const Eigen::Vector3d finger_centre =
    start_->getGlobalLinkTransform("left_finger") * Eigen::Vector3d(0.0, 0.0, 0.07);
  add_box(
    "restocker/obstacle/block", Eigen::Vector3d::Constant(0.008),
    finger_centre + back * 0.10 + left_axis_ * 0.003);
  collision_detection::CollisionRequest request;
  request.contacts = true;
  request.max_contacts = 16U;
  collision_detection::CollisionResult result;
  scene_->checkCollision(request, result, *start_);
  for (const auto & contact : result.contacts) {
    ASSERT_NE(contact.first.first, "restocker/obstacle/block");
    ASSERT_NE(contact.first.second, "restocker/obstacle/block");
  }
  const auto line = escape();
  EXPECT_FALSE(line.planned);
  EXPECT_NE(line.detail.find("stopped at fraction"), std::string::npos) << line.detail;
}

TEST_F(GraspEscapePlannerTest, RefusesWhenTheToleratedContactWouldFollowTheArmOut)
{
  // An escape that stops 2 cm back leaves the finger still inside the product: the exemption
  // must not carry the contact out of the grasp.
  add_product(kTarget, 0.001, kProductRadiusM);
  Eigen::Isometry3d near = grasp_;
  near.translation() += (standoff_.translation() - grasp_.translation()).normalized() * 0.02;
  const auto line = escape_to(near);
  EXPECT_FALSE(line.planned);
  EXPECT_NE(
    line.detail.find("still in collision under the unmodified matrix"),
    std::string::npos) << line.detail;
}

// Card 062 review N5: the tolerance covers the start's contact, not a new one with the same
// object. A second body of the target lies across the finger's way out (a product knocked over
// across the path) — same object id, so a pair-only exemption would let the line through.
TEST_F(GraspEscapePlannerTest, RefusesWhenTheToleratedContactWouldDeepen)
{
  add_product(kTarget, 0.001, kProductRadiusM);
  const Eigen::Vector3d back = (standoff_.translation() - grasp_.translation()).normalized();
  const Eigen::Vector3d finger_centre =
    start_->getGlobalLinkTransform("left_finger") * Eigen::Vector3d(0.0, 0.0, 0.07);
  Eigen::Isometry3d across = Eigen::Isometry3d::Identity();
  across.translation() = finger_centre + back * 0.10 + left_axis_ * 0.003;
  scene_->getWorldNonConst()->addToObject(
    kTarget, std::make_shared<shapes::Box>(0.008, 0.008, 0.008), across);
  const auto line = escape();
  EXPECT_FALSE(line.planned);
  EXPECT_NE(line.detail.find("stopped at fraction"), std::string::npos) << line.detail;
}

// Card 062 + Card 068: the port plans the escape on move_group's snapshot through
// make_race_scene. A complete snapshot plans; one without link padding/scale or without the
// allowed-collision matrix refuses the escape instead of planning on a laxer scene.
TEST_F(GraspEscapePlannerTest, PlansFromACompleteSnapshotAndRefusesAnIncompleteOne)
{
  add_product(kTarget, 0.001, kProductRadiusM);
  moveit_msgs::msg::PlanningScene snapshot;
  scene_->getPlanningSceneMsg(snapshot);
  ASSERT_FALSE(snapshot.link_padding.empty());
  ASSERT_FALSE(snapshot.allowed_collision_matrix.entry_names.empty());
  GraspEscape request;
  request.planning_frame_from_standoff_tool0 = standoff_;
  request.target_object_id = kTarget;
  request.tolerated_links = {"left_finger", "right_finger"};
  const auto plan = [&](const moveit_msgs::msg::PlanningScene & message) {
    return plan_grasp_escape_from_snapshot(
      message, *start_, group_, model_->getLinkModel("tool0"), request, 0.005, 0.999);
  };

  const auto complete = plan(snapshot);
  ASSERT_TRUE(complete.planned) << complete.detail;
  ASSERT_EQ(complete.tolerated_pairs.size(), 1U);

  auto unpadded = snapshot;
  unpadded.link_padding.clear();
  unpadded.link_scale.clear();
  const auto no_padding = plan(unpadded);
  EXPECT_FALSE(no_padding.planned);
  EXPECT_NE(no_padding.detail.find("refuses move_group's scene snapshot"), std::string::npos) <<
    no_padding.detail;
  EXPECT_NE(no_padding.detail.find("link padding"), std::string::npos) << no_padding.detail;

  auto no_matrix = snapshot;
  no_matrix.allowed_collision_matrix = moveit_msgs::msg::AllowedCollisionMatrix{};
  const auto without_matrix = plan(no_matrix);
  EXPECT_FALSE(without_matrix.planned);
  EXPECT_NE(without_matrix.detail.find("allowed-collision matrix"), std::string::npos) <<
    without_matrix.detail;
}

}  // namespace
}  // namespace restocker_task_executor
