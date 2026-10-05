// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Milestone 10 §6 "The race plans against move_group's scene" (Card 068), on the real restocker
// model. The monitored scene is a real PlanningSceneMonitor configured with move_group's own
// parameters (its robot padding read from move_group.launch.py), the snapshot is taken the way
// move_group's /get_planning_scene takes it, and the planners are the race's production ompl
// (RRTConnect) and ompl_fallback (PRM) pipelines, each on its own model load as in the port.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/free_space_race.hpp"
#include "restocker_task_executor/race_planning_scene.hpp"

namespace restocker_task_executor
{
namespace
{

// What the race asked for before Card 068: geometry only, no padding, no matrix. Kept to show
// the gap on the same states — the regression assertions below are all on the shipped builder.
constexpr std::uint32_t kPreCard068Components =
  moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
  moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;

constexpr const char * kGroup = "manipulator";
constexpr const char * kBox = "restocker/test/padding_probe";
constexpr double kBoxSideM = 0.10;

class RacePlanningSceneTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    const std::string parameters = std::string(::testing::TempDir()) + "race_scene_params.yaml";
    const std::string command = std::string("python3 ") + RESTOCKER_TEST_RACE_PARAMETERS_SCRIPT +
      " " + RESTOCKER_TEST_MOVE_GROUP_LAUNCH + " " + parameters;
    ASSERT_EQ(std::system(command.c_str()), 0) << command;
    const char * argv[] = {"test_race_planning_scene", "--ros-args", "--params-file",
      parameters.c_str()};
    rclcpp::init(4, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node_ = std::make_shared<rclcpp::Node>("race_planning_scene_test", options);
    node_->get_parameter("robot_description_planning.default_robot_padding", padding_m_);
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    model_ = loader_->getModel();
    ASSERT_TRUE(model_) << "the restocker model did not load";
    // move_group's monitor: it reads robot_description_planning.* off the node and pads the
    // links of its scene exactly as the running move_group does.
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_, loader_, "race_scene_test_monitor");
    // The port gives the fallback pipeline its own model load (its own KDL solver instance).
    fallback_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(
      node_, robot_model_loader::RobotModelLoader::Options());
    primary_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      model_, node_, kRacePrimaryPipeline);
    fallback_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      fallback_loader_->getModel(), node_, kRaceFallbackPipeline);
  }

  static void TearDownTestSuite()
  {
    primary_pipeline_.reset();
    fallback_pipeline_.reset();
    monitor_.reset();
    model_.reset();
    fallback_loader_.reset();
    loader_.reset();
    node_.reset();
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    ASSERT_TRUE(model_);
    ASSERT_TRUE(monitor_ && monitor_->getPlanningScene());
    ASSERT_GT(padding_m_, 0.0) << "move_group launches with a positive robot padding";
    start_ = std::make_unique<moveit::core::RobotState>(model_);
    start_->setToDefaultValues();
    ASSERT_TRUE(start_->setToDefaultValues(model_->getJointModelGroup(kGroup), "home"));
    start_->update();
    remove_box();
    reset_matrix();
    ASSERT_FALSE(monitored_colliding(*start_)) << "home must be valid in move_group's scene";
  }

  void TearDown() override
  {
    remove_box();
    reset_matrix();
  }

  static planning_scene::PlanningScenePtr monitored()
  {
    return monitor_->getPlanningScene();
  }

  // The box's world position when it has travelled `travel_m` from 0.5 m above tool0 straight
  // down towards the arm.
  Eigen::Vector3d box_centre(double travel_m) const
  {
    const Eigen::Vector3d tool0 = start_->getGlobalLinkTransform("tool0").translation();
    return tool0 + Eigen::Vector3d(0.0, 0.0, 0.5 - travel_m);
  }

  static void place_box(const Eigen::Vector3d & centre)
  {
    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = model_->getModelFrame();
    box.id = kBox;
    box.operation = moveit_msgs::msg::CollisionObject::ADD;
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {kBoxSideM, kBoxSideM, kBoxSideM};
    box.primitives.push_back(primitive);
    geometry_msgs::msg::Pose pose;
    pose.position.x = centre.x();
    pose.position.y = centre.y();
    pose.position.z = centre.z();
    pose.orientation.w = 1.0;
    box.pose = pose;
    box.primitive_poses.push_back(geometry_msgs::msg::Pose());
    box.primitive_poses.back().orientation.w = 1.0;
    ASSERT_TRUE(monitored()->processCollisionObjectMsg(box));
  }

  static void remove_box()
  {
    if (monitored()->getWorld()->hasObject(kBox)) {
      monitored()->getWorldNonConst()->removeObject(kBox);
    }
  }

  static void reset_matrix()
  {
    auto & matrix = monitored()->getAllowedCollisionMatrixNonConst();
    if (matrix.hasEntry(kBox)) {
      matrix.removeEntry(kBox);
    }
  }

  bool monitored_colliding(const moveit::core::RobotState & state) const
  {
    return monitored()->isStateColliding(state, kGroup);
  }

  // What move_group's /get_planning_scene answers: getPlanningSceneMsg on its monitored scene.
  static moveit_msgs::msg::PlanningScene snapshot(std::uint32_t components)
  {
    moveit_msgs::msg::PlanningScene message;
    moveit_msgs::msg::PlanningSceneComponents request;
    request.components = components;
    monitored()->getPlanningSceneMsg(message, request);
    return message;
  }

  static planning_scene::PlanningScenePtr pre_card_068_scene(
    const moveit::core::RobotModelConstPtr & model)
  {
    auto scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->setPlanningSceneDiffMsg(snapshot(kPreCard068Components));
    return scene;
  }

  // Lowers the box until the unpadded arm first touches it and returns the travel at which it
  // does, to 10 µm. The first touch is against whichever link sits highest under tool0.
  double first_unpadded_contact_travel()
  {
    const auto unpadded = [this](double travel) {
      place_box(box_centre(travel));
      return pre_card_068_scene(model_)->isStateColliding(*start_, kGroup);
    };
    double free_travel = 0.0;
    double contact_travel = 0.5;
    EXPECT_FALSE(unpadded(free_travel));
    EXPECT_TRUE(unpadded(contact_travel));
    while (contact_travel - free_travel > 1e-5) {
      const double middle = 0.5 * (free_travel + contact_travel);
      (unpadded(middle) ? contact_travel : free_travel) = middle;
    }
    return contact_travel;
  }

  moveit_msgs::msg::MotionPlanRequest request_from_start(const char * pipeline_id) const
  {
    moveit_msgs::msg::MotionPlanRequest request;
    request.group_name = kGroup;
    request.pipeline_id = pipeline_id;
    request.planner_id = std::string(pipeline_id) == kRaceFallbackPipeline ?
      kRaceFallbackPlannerId : "RRTConnectkConfigDefault";
    request.allowed_planning_time = 5.0;  // the survey/confirm slice
    request.num_planning_attempts = 1;
    request.max_velocity_scaling_factor = 0.2;
    request.max_acceleration_scaling_factor = 0.2;
    moveit::core::robotStateToRobotStateMsg(*start_, request.start_state);
    request.start_state.is_diff = false;
    // A joint goal: swing the shoulder a little. Any goal serves — the start state is what is
    // being judged.
    moveit_msgs::msg::Constraints goal;
    const auto * group = model_->getJointModelGroup(kGroup);
    std::vector<double> values;
    start_->copyJointGroupPositions(group, values);
    for (std::size_t index = 0; index < group->getVariableNames().size(); ++index) {
      moveit_msgs::msg::JointConstraint joint;
      joint.joint_name = group->getVariableNames()[index];
      joint.position = values[index] +
        (joint.joint_name == "shoulder_pan_joint" ? 0.3 : 0.0);
      joint.tolerance_above = 1e-3;
      joint.tolerance_below = 1e-3;
      joint.weight = 1.0;
      goal.joint_constraints.push_back(joint);
    }
    request.goal_constraints.push_back(goal);
    return request;
  }

  // One raced slice as the port runs it: each pipeline on the scene built from its own model.
  struct RaceCodes
  {
    int primary{0};
    int fallback{0};
  };

  RaceCodes race(
    const planning_scene::PlanningSceneConstPtr & primary_scene,
    const planning_scene::PlanningSceneConstPtr & fallback_scene) const
  {
    RaceCodes codes;
    codes.primary = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
      request_from_start(kRacePrimaryPipeline), primary_scene,
      {{kRacePrimaryPipeline, primary_pipeline_}}).error_code.val;
    codes.fallback = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
      request_from_start(kRaceFallbackPipeline), fallback_scene,
      {{kRaceFallbackPipeline, fallback_pipeline_}}).error_code.val;
    return codes;
  }

  static RaceSceneBuild shipped(const moveit::core::RobotModelConstPtr & model)
  {
    return make_race_scene(model, snapshot(kRaceSceneComponents));
  }

  static inline rclcpp::Node::SharedPtr node_;
  static inline robot_model_loader::RobotModelLoaderPtr loader_;
  static inline robot_model_loader::RobotModelLoaderPtr fallback_loader_;
  static inline moveit::core::RobotModelConstPtr model_;
  static inline planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  static inline planning_pipeline::PlanningPipelinePtr primary_pipeline_;
  static inline planning_pipeline::PlanningPipelinePtr fallback_pipeline_;
  static inline double padding_m_{0.0};
  std::unique_ptr<moveit::core::RobotState> start_;
};

using Code = moveit_msgs::msg::MoveItErrorCodes;

// SC-003: the race scene's collision model is move_group's — every link's padding and scale,
// and the allowed-collision matrix, equal the monitored scene's; and the padding is the one
// move_group launches with (so the monitor under test really is configured like move_group).
TEST_F(RacePlanningSceneTest, RaceSceneCarriesMoveGroupPaddingScaleAndMatrix)
{
  const auto build = shipped(model_);
  ASSERT_TRUE(build.scene) << build.refusal;
  const auto & expected = monitored()->getCollisionEnv();
  const auto & actual = build.scene->getCollisionEnv();
  std::size_t padded_links = 0U;
  for (const auto * link : model_->getLinkModelsWithCollisionGeometry()) {
    const auto & name = link->getName();
    EXPECT_DOUBLE_EQ(actual->getLinkPadding(name), expected->getLinkPadding(name)) << name;
    EXPECT_DOUBLE_EQ(actual->getLinkScale(name), expected->getLinkScale(name)) << name;
    EXPECT_DOUBLE_EQ(expected->getLinkPadding(name), padding_m_) << name;
    ++padded_links;
  }
  EXPECT_GT(padded_links, 0U);
  moveit_msgs::msg::AllowedCollisionMatrix expected_matrix;
  moveit_msgs::msg::AllowedCollisionMatrix actual_matrix;
  monitored()->getAllowedCollisionMatrix().getMessage(expected_matrix);
  build.scene->getAllowedCollisionMatrix().getMessage(actual_matrix);
  EXPECT_EQ(actual_matrix, expected_matrix);
  // Before Card 068 the race scene had none of it.
  const auto before = pre_card_068_scene(model_);
  for (const auto * link : model_->getLinkModelsWithCollisionGeometry()) {
    EXPECT_DOUBLE_EQ(before->getCollisionEnv()->getLinkPadding(link->getName()), 0.0);
  }
}

// SC-001: a start state inside move_group's padding but clear of the unpadded geometry. move_group
// refuses it; so must both race pipelines. Before Card 068 both planned from it.
TEST_F(RacePlanningSceneTest, PaddedContactStartIsRefusedByBothRacePipelines)
{
  const double contact = first_unpadded_contact_travel();
  // Half the padding short of touching: clear unpadded, inside the padded envelope.
  place_box(box_centre(contact - 0.5 * padding_m_));
  ASSERT_TRUE(monitored_colliding(*start_)) << "move_group's padded scene must refuse the start";

  const auto primary = shipped(model_);
  const auto fallback = shipped(fallback_loader_->getModel());
  ASSERT_TRUE(primary.scene) << primary.refusal;
  ASSERT_TRUE(fallback.scene) << fallback.refusal;
  EXPECT_TRUE(primary.scene->isStateColliding(*start_, kGroup));
  const auto codes = race(primary.scene, fallback.scene);
  EXPECT_EQ(codes.primary, Code::START_STATE_IN_COLLISION);
  EXPECT_EQ(codes.fallback, Code::START_STATE_IN_COLLISION);
  EXPECT_TRUE(is_start_state_code(codes.primary));

  // The gap this closes: the pre-068 scene saw the same start as free.
  const auto before_primary = pre_card_068_scene(model_);
  const auto before_fallback = pre_card_068_scene(fallback_loader_->getModel());
  EXPECT_FALSE(before_primary->isStateColliding(*start_, kGroup));
  const auto before = race(before_primary, before_fallback);
  EXPECT_NE(before.primary, Code::START_STATE_IN_COLLISION);
  EXPECT_NE(before.fallback, Code::START_STATE_IN_COLLISION);
}

// Clear of the padding too: the change refuses only what move_group refuses.
TEST_F(RacePlanningSceneTest, StartClearOfThePaddingIsStillPlanned)
{
  const double contact = first_unpadded_contact_travel();
  place_box(box_centre(contact - 2.0 * padding_m_));
  ASSERT_FALSE(monitored_colliding(*start_));
  const auto primary = shipped(model_);
  const auto fallback = shipped(fallback_loader_->getModel());
  ASSERT_TRUE(primary.scene && fallback.scene);
  const auto codes = race(primary.scene, fallback.scene);
  EXPECT_NE(codes.primary, Code::START_STATE_IN_COLLISION);
  EXPECT_NE(codes.fallback, Code::START_STATE_IN_COLLISION);
  EXPECT_TRUE(codes.primary == Code::SUCCESS || codes.fallback == Code::SUCCESS)
    << "primary " << codes.primary << ", fallback " << codes.fallback;
}

// SC-002: a contact move_group's matrix allows is not a collision for the race either. Before
// Card 068 the race rebuilt the SRDF-only matrix and refused it.
TEST_F(RacePlanningSceneTest, ContactAllowedByMoveGroupsMatrixIsAllowedByTheRace)
{
  const double contact = first_unpadded_contact_travel();
  place_box(box_centre(contact + 0.01));  // 1 cm into the arm
  ASSERT_TRUE(monitored_colliding(*start_));
  monitored()->getAllowedCollisionMatrixNonConst().setEntry(
    kBox, model_->getLinkModelNamesWithCollisionGeometry(), true);
  ASSERT_FALSE(monitored_colliding(*start_)) << "move_group's matrix now allows the contact";

  const auto primary = shipped(model_);
  const auto fallback = shipped(fallback_loader_->getModel());
  ASSERT_TRUE(primary.scene && fallback.scene);
  EXPECT_FALSE(primary.scene->isStateColliding(*start_, kGroup));
  const auto codes = race(primary.scene, fallback.scene);
  EXPECT_NE(codes.primary, Code::START_STATE_IN_COLLISION);
  EXPECT_NE(codes.fallback, Code::START_STATE_IN_COLLISION);

  EXPECT_TRUE(pre_card_068_scene(model_)->isStateColliding(*start_, kGroup));
}

// SC-003, fail-closed: a snapshot that cannot give the race move_group's collision model is not
// raced (the port then plans through move_group's service).
TEST_F(RacePlanningSceneTest, SnapshotWithoutPaddingOrMatrixIsRefused)
{
  auto without_padding = snapshot(kRaceSceneComponents);
  without_padding.link_padding.clear();
  const auto no_padding = make_race_scene(model_, without_padding);
  EXPECT_FALSE(no_padding.scene);
  EXPECT_NE(no_padding.refusal.find("padding"), std::string::npos) << no_padding.refusal;

  auto without_scale = snapshot(kRaceSceneComponents);
  without_scale.link_scale.clear();
  EXPECT_FALSE(make_race_scene(model_, without_scale).scene);

  auto without_matrix = snapshot(kRaceSceneComponents);
  without_matrix.allowed_collision_matrix = moveit_msgs::msg::AllowedCollisionMatrix();
  const auto no_matrix = make_race_scene(model_, without_matrix);
  EXPECT_FALSE(no_matrix.scene);
  EXPECT_NE(no_matrix.refusal.find("allowed-collision matrix"), std::string::npos)
    << no_matrix.refusal;

  EXPECT_FALSE(make_race_scene(model_, snapshot(kPreCard068Components)).scene);
  EXPECT_FALSE(make_race_scene(nullptr, snapshot(kRaceSceneComponents)).scene);
}

// Latency: the extra snapshot content and applying the padding cost milliseconds against the
// race's smallest slice (5 s). Printed for the card; the bound only catches a blow-up.
TEST_F(RacePlanningSceneTest, RaceSceneBuildStaysFarInsideTheSlice)
{
  place_box(box_centre(0.0));
  const auto time_builds = [](const auto & build_once) {
    std::vector<double> samples;
    for (int repeat = 0; repeat < 41; ++repeat) {
      const auto started = std::chrono::steady_clock::now();
      build_once();
      samples.push_back(
        std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started).count());
    }
    std::sort(samples.begin(), samples.end());
    return std::pair<double, double>{samples[samples.size() / 2], samples.back()};
  };
  const auto before = time_builds(
    [&] {
      (void)pre_card_068_scene(model_);
    });
  const auto after = time_builds(
    [&] {
      const auto build = shipped(model_);
      ASSERT_TRUE(build.scene);
    });
  rclcpp::Serialization<moveit_msgs::msg::PlanningScene> serializer;
  rclcpp::SerializedMessage before_bytes;
  rclcpp::SerializedMessage after_bytes;
  const auto before_message = snapshot(kPreCard068Components);
  const auto after_message = snapshot(kRaceSceneComponents);
  serializer.serialize_message(&before_message, &before_bytes);
  serializer.serialize_message(&after_message, &after_bytes);
  std::printf(
    "[card068-latency] snapshot+scene build per pipeline: pre-068 p50 %.3f ms max %.3f ms; "
    "068 p50 %.3f ms max %.3f ms; snapshot %zu -> %zu bytes; load1 at end ",
    before.first, before.second, after.first, after.second, before_bytes.size(),
    after_bytes.size());
  std::ifstream loadavg("/proc/loadavg");
  double load1 = -1.0;
  loadavg >> load1;
  std::printf("%.2f\n", load1);
  EXPECT_LT(after.first, 250.0) << "5 % of the 5 s survey/confirm slice";
}

}  // namespace
}  // namespace restocker_task_executor
