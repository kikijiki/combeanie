// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Card 053 Phase 2: planning-only benchmark. No Gazebo, no move_group, no execution.
//
// One in-process node loads the robot model, builds an acceptance-like 14-object planning
// scene (workcell + declared tray seeds + occupied lane columns), and replays pose goals
// recorded from production receipts against the same OMPL pipeline configuration
// (ompl_planning.yaml, flattened under `ompl.` and a second `ompl_fallback.` namespace
// whose groups plan with PRM). Three modes per query:
//   single — one attempt, the leg's production slice (planWithSinglePipeline),
//   retry2 — two sequential half-slice attempts inside the same total (survey/confirm only),
//   parallel — ompl + ompl_fallback raced with stopAtFirstSolution.
// Each query writes one CSV row (wall ms, process CPU ms, error code, load1) so quiet versus
// loaded bands and the three modes can be compared without a simulator.
//
// Card 068 padding audit (padding_audit:=true): instead of the three modes, every query is
// raced twice — on the unpadded scene the race built before Card 068, and on the same scene
// padded like move_group (robot_padding_m) — and the unpadded winner's waypoints are
// collision-checked in the padded scene: how often did the race accept a plan move_group's
// padded scene refuses, and what does planning in the padded scene cost?

#include <Eigen/Geometry>
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_pipeline_interfaces/planning_pipeline_interfaces.hpp>
#include <moveit/planning_pipeline_interfaces/solution_selection_functions.hpp>
#include <moveit/planning_pipeline_interfaces/stopping_criterion_functions.hpp>
#include <moveit/planning_pipeline/planning_pipeline.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/scene_geometry.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using Pipelines = std::unordered_map<std::string, planning_pipeline::PlanningPipelinePtr>;

struct RecordedGoal
{
  const char * leg;
  const char * label;
  double x, y, z;
  double qx, qy, qz, qw;
  bool transfer_slice;  // false => survey/confirm slice (5 s), true => coordinator slice (15 s)
};

// Pose goals copied from production receipts (evidence/cmbframe-050 and cmbrecov-051 logs:
// "planning <kind> <label> to tool0 pose xyz=... quat=..."). They are world-frame tool0 poses.
constexpr RecordedGoal kGoals[] = {
  {"survey", "lane_01", -1.0000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"survey", "lane_02", -0.6000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"survey", "lane_03", -0.2000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"survey", "lane_04", 0.2000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"survey", "lane_05", 0.6000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"survey", "lane_06", 1.0000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579, false},
  {"confirm", "tray_confirm", -0.0405, -0.5446, 0.9190, 0.6665, -0.6665, 0.2362, 0.2362,
    false},
  {"transfer", "pre_grasp_can", -0.0402, -0.5597, 0.6809, 0.5000, -0.5000, 0.5000, 0.5000, true},
  {"transfer", "pre_grasp_large", 0.3971, -0.3385, 0.7795, 0.5000, -0.5000, 0.5000, 0.5000,
    true},
  {"transfer", "pre_insert_lane01", -1.0000, 0.3780, 0.9217, 0.5000, 0.5000, 0.5000, -0.5000,
    true},
  {"transfer", "pre_insert_lane03", -0.2000, 0.3900, 1.0207, 0.5000, 0.5000, 0.5000, -0.5000,
    true},
  {"retreat", "retreat_reposition_lane01", -1.0000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579,
    -0.3579, true},
  {"retreat", "retreat_lane03", -0.2000, -0.0302, 1.4428, 0.6098, -0.6098, -0.3579, -0.3579,
    true},
};

constexpr double kPositionToleranceM = 0.005;   // MotionGoal default (motion_port.hpp)
constexpr double kOrientationToleranceRad = 0.02;
constexpr double kSurveySliceSec = 5.0;          // survey/confirm legs: planning_time_ms 5000
constexpr double kTransferSliceSec = 15.0;       // coordinator slice: 60 s / 4 candidates

double load1()
{
  std::ifstream stream("/proc/loadavg");
  double value = 0.0;
  if (stream) {
    stream >> value;
  }
  return value;
}

double processCpuSec()
{
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
  return static_cast<double>(usage.ru_utime.tv_sec) + usage.ru_utime.tv_usec * 1e-6 +
         static_cast<double>(usage.ru_stime.tv_sec) + usage.ru_stime.tv_usec * 1e-6;
}

geometry_msgs::msg::Pose poseMsg(
  double x, double y, double z, double qx, double qy, double qz,
  double qw)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;
  pose.orientation.x = qx;
  pose.orientation.y = qy;
  pose.orientation.z = qz;
  pose.orientation.w = qw;
  return pose;
}

moveit_msgs::msg::Constraints poseGoal(const RecordedGoal & goal)
{
  moveit_msgs::msg::Constraints constraints;
  moveit_msgs::msg::PositionConstraint position;
  position.header.frame_id = "world";
  position.link_name = "tool0";
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions.push_back(kPositionToleranceM);
  position.constraint_region.primitives.push_back(sphere);
  position.constraint_region.primitive_poses.push_back(
    poseMsg(goal.x, goal.y, goal.z, goal.qx, goal.qy, goal.qz, goal.qw));
  position.weight = 1.0;
  moveit_msgs::msg::OrientationConstraint orientation;
  orientation.header.frame_id = "world";
  orientation.link_name = "tool0";
  orientation.orientation = poseMsg(0.0, 0.0, 0.0, goal.qx, goal.qy, goal.qz, goal.qw).orientation;
  orientation.absolute_x_axis_tolerance = kOrientationToleranceRad;
  orientation.absolute_y_axis_tolerance = kOrientationToleranceRad;
  orientation.absolute_z_axis_tolerance = kOrientationToleranceRad;
  orientation.weight = 1.0;
  constraints.position_constraints.push_back(position);
  constraints.orientation_constraints.push_back(orientation);
  return constraints;
}

moveit::core::RobotState seedState(
  const moveit::core::RobotModelConstPtr & model,
  const std::vector<std::string> & seed_names, const std::vector<double> & seed_values)
{
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (size_t i = 0; i < seed_names.size() && i < seed_values.size(); ++i) {
    if (model->hasJointModel(seed_names[i])) {
      state.setVariablePosition(seed_names[i], seed_values[i]);
    }
  }
  state.update();
  return state;
}

moveit_msgs::msg::MotionPlanRequest makeRequest(
  const RecordedGoal & goal,
  const moveit::core::RobotState & start, const std::string & pipeline_id, double allowed_sec)
{
  moveit_msgs::msg::MotionPlanRequest request;
  request.pipeline_id = pipeline_id;
  request.group_name = "manipulator";
  moveit::core::robotStateToRobotStateMsg(start, request.start_state);
  request.start_state.is_diff = false;
  request.goal_constraints.push_back(poseGoal(goal));
  request.allowed_planning_time = allowed_sec;
  request.num_planning_attempts = 1;
  request.max_velocity_scaling_factor = 0.2;
  request.max_acceleration_scaling_factor = 0.2;
  return request;
}

bool isSuccess(const planning_interface::MotionPlanResponse & response)
{
  return response.error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS &&
         response.trajectory != nullptr;
}

planning_interface::MotionPlanResponse pickBest(
  const std::vector<planning_interface::MotionPlanResponse> & solutions)
{
  std::vector<planning_interface::MotionPlanResponse> successes;
  for (const auto & response : solutions) {
    if (isSuccess(response)) {
      successes.push_back(response);
    }
  }
  if (!successes.empty()) {
    return moveit::planning_pipeline_interfaces::getShortestSolution(successes);
  }
  if (solutions.empty()) {
    return planning_interface::MotionPlanResponse();
  }
  planning_interface::MotionPlanResponse fallback;
  fallback.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  return fallback;
}

struct QueryResult
{
  bool ok{false};
  int error_code{0};
  double wall_ms{0.0};
  double cpu_ms{0.0};
  double load{0.0};
};

QueryResult runSingle(
  const planning_interface::MotionPlanRequest & request,
  const planning_scene::PlanningSceneConstPtr & scene, const Pipelines & pipelines)
{
  QueryResult result;
  const double cpu_before = processCpuSec();
  const auto wall_before = std::chrono::steady_clock::now();
  auto response = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
    request, scene, pipelines);
  result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_before).count();
  result.cpu_ms = (processCpuSec() - cpu_before) * 1000.0;
  result.load = load1();
  result.ok = isSuccess(response);
  result.error_code = response.error_code.val;
  return result;
}

QueryResult runParallel(
  const RecordedGoal & goal, const moveit::core::RobotState & start,
  double allowed_sec, const planning_scene::PlanningSceneConstPtr & scene,
  const Pipelines & pipelines)
{
  std::vector<planning_interface::MotionPlanRequest> requests;
  requests.push_back(makeRequest(goal, start, "ompl", allowed_sec));
  auto fallback = makeRequest(goal, start, "ompl_fallback", allowed_sec);
  // Upstream resolves an empty planner_id to the group default, whose entry hardcodes
  // RRTConnect unless default_planner_config says otherwise; name PRM explicitly so the
  // fallback pipeline is what actually races.
  fallback.planner_id = "PRMkConfigDefault";
  requests.push_back(std::move(fallback));
  QueryResult result;
  const double cpu_before = processCpuSec();
  const auto wall_before = std::chrono::steady_clock::now();
  auto solutions = moveit::planning_pipeline_interfaces::planWithParallelPipelines(
    requests, scene,
    pipelines, moveit::planning_pipeline_interfaces::stopAtFirstSolution, pickBest);
  result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_before).count();
  result.cpu_ms = (processCpuSec() - cpu_before) * 1000.0;
  result.load = load1();
  if (solutions.empty()) {
    result.error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return result;
  }
  result.ok = isSuccess(solutions.front());
  result.error_code = solutions.front().error_code.val;
  return result;
}

struct RaceResponse
{
  planning_interface::MotionPlanResponse response;
  double wall_ms{0.0};
};

RaceResponse raceResponse(
  const RecordedGoal & goal, const moveit::core::RobotState & start,
  double allowed_sec, const planning_scene::PlanningSceneConstPtr & scene,
  const Pipelines & pipelines)
{
  std::vector<planning_interface::MotionPlanRequest> requests;
  requests.push_back(makeRequest(goal, start, "ompl", allowed_sec));
  auto fallback = makeRequest(goal, start, "ompl_fallback", allowed_sec);
  fallback.planner_id = "PRMkConfigDefault";
  requests.push_back(std::move(fallback));
  RaceResponse result;
  const auto wall_before = std::chrono::steady_clock::now();
  auto solutions = moveit::planning_pipeline_interfaces::planWithParallelPipelines(
    requests, scene,
    pipelines, moveit::planning_pipeline_interfaces::stopAtFirstSolution, pickBest);
  result.wall_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - wall_before).count();
  if (solutions.empty()) {
    result.response.error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  } else {
    result.response = solutions.front();
  }
  return result;
}

class PlanningOnlyBenchmark
{
public:
  PlanningOnlyBenchmark()
  : node_(std::make_shared<rclcpp::Node>(
        "planning_only_benchmark",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)))
  {
    output_csv_ = param<std::string>("output_csv", "/tmp/planning_bench.csv");
    band_label_ = param<std::string>("band_label", "quiet");
    repeats_ = static_cast<int>(param<std::int64_t>("repeats", 4));
    lane_occupied_depth_m_ = param<double>("lane_occupied_depth_m", 0.12);
    lane_column_width_m_ = param<double>("lane_column_width_m", 0.10);
    padding_audit_ = param<bool>("padding_audit", false);
    robot_padding_m_ = param<double>("robot_padding_m", 0.0015);
    scenario_path_ = param<std::string>("scenario_path", "");
    catalog_path_ = param<std::string>("product_catalog_path", "");
    geometry_path_ = param<std::string>("workcell_geometry_path", "");
    workcell_description_ = param<std::string>("workcell_description", "");
    const auto workcell_pose =
      param<std::vector<double>>("workcell_pose", {0.0, 0.55, 0.75, 0.0, 0.0, 0.0});
    if (scenario_path_.empty() || catalog_path_.empty() || geometry_path_.empty() ||
      workcell_description_.empty() || workcell_pose.size() != 6U)
    {
      throw std::invalid_argument(
              "planning_only_benchmark requires scenario_path, product_catalog_path, "
              "workcell_geometry_path, workcell_description and a six-value workcell_pose");
    }
    shelf_from_world_ = isometryFromXyzrpy(workcell_pose);

    robot_model_loader::RobotModelLoader::Options options;
    options.load_kinematics_solvers = true;
    model_loader_ = std::make_unique<robot_model_loader::RobotModelLoader>(node_, options);
    model_ = model_loader_->getModel();
    if (!model_) {
      throw std::invalid_argument("robot model failed to load");
    }
    scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    buildScene(*scene_);
    // move_group's scene: the same world with every robot link padded, as its planning scene
    // monitor does with robot_description_planning.default_robot_padding.
    padded_scene_ = std::make_shared<planning_scene::PlanningScene>(model_);
    buildScene(*padded_scene_);
    padded_scene_->getCollisionEnvNonConst()->setPadding(robot_padding_m_);
    pipelines_ = moveit::planning_pipeline_interfaces::createPlanningPipelineMap(
      {"ompl", "ompl_fallback"}, model_, node_);
    if (pipelines_.size() != 2U) {
      throw std::invalid_argument("expected the ompl and ompl_fallback pipelines on the node");
    }
  }

  int run()
  {
    if (padding_audit_) {
      return runPaddingAudit();
    }
    std::ofstream out(output_csv_, std::ios::trunc);
    if (!out) {
      throw std::invalid_argument("cannot open output_csv " + output_csv_);
    }
    out << "band,mode,leg,goal,seed,repeat,ok,error_code,wall_ms,cpu_ms,load1\n";
    const double start_load = load1();
    size_t rows = 0;
    for (const auto & goal : kGoals) {
      const double allowed = goal.transfer_slice ? kTransferSliceSec : kSurveySliceSec;
      for (const auto & seed : goalSeeds(goal)) {
        const auto start_state = seedState(model_, seed.names, seed.values);
        for (int repeat = 0; repeat < repeats_; ++repeat) {
          const auto single = makeRequest(goal, start_state, "ompl", allowed);
          writeRow(
            out, "single", goal, seed.label, repeat,
            runSingle(single, scene_, pipelines_), rows);
          if (!goal.transfer_slice) {
            QueryResult retry;
            const double cpu_before = processCpuSec();
            const auto wall_before = std::chrono::steady_clock::now();
            auto first = makeRequest(goal, start_state, "ompl", allowed * 0.5);
            auto first_response = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
              first, scene_, pipelines_);
            const bool first_ok = isSuccess(first_response);
            int code = first_response.error_code.val;
            if (!first_ok) {
              auto second = makeRequest(goal, start_state, "ompl", allowed * 0.5);
              auto second_response = moveit::planning_pipeline_interfaces::planWithSinglePipeline(
                second, scene_, pipelines_);
              code = second_response.error_code.val;
              retry.ok = isSuccess(second_response);
            } else {
              retry.ok = true;
            }
            retry.wall_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - wall_before).count();
            retry.cpu_ms = (processCpuSec() - cpu_before) * 1000.0;
            retry.load = load1();
            retry.error_code = code;
            writeRow(out, "retry2", goal, seed.label, repeat, retry, rows);
          }
          if (goal.transfer_slice) {
            writeRow(
              out, "parallel", goal, seed.label, repeat,
              runParallel(goal, start_state, allowed, scene_, pipelines_), rows);
          }
        }
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "planning_only_benchmark band=%s wrote %zu rows to %s (load1 %.2f -> %.2f)",
      band_label_.c_str(), rows, output_csv_.c_str(), start_load, load1());
    return 0;
  }

private:
  int runPaddingAudit()
  {
    std::ofstream out(output_csv_, std::ios::trunc);
    if (!out) {
      throw std::invalid_argument("cannot open output_csv " + output_csv_);
    }
    out << "band,leg,goal,seed,repeat,start_free_unpadded,start_free_padded,"
      "old_ok,old_code,old_wall_ms,old_waypoints,old_waypoints_in_padded_collision,"
      "old_end_in_padded_collision,new_ok,new_code,new_wall_ms,load1\n";
    const double start_load = load1();
    size_t rows = 0;
    for (const auto & goal : kGoals) {
      const double allowed = goal.transfer_slice ? kTransferSliceSec : kSurveySliceSec;
      for (const auto & seed : goalSeeds(goal)) {
        const auto start_state = seedState(model_, seed.names, seed.values);
        const bool start_free_unpadded = !scene_->isStateColliding(start_state, "manipulator");
        const bool start_free_padded =
          !padded_scene_->isStateColliding(start_state, "manipulator");
        for (int repeat = 0; repeat < repeats_; ++repeat) {
          const auto old_race = raceResponse(goal, start_state, allowed, scene_, pipelines_);
          const bool old_ok = isSuccess(old_race.response);
          size_t waypoints = 0;
          size_t in_collision = 0;
          bool end_in_collision = false;
          if (old_ok) {
            const auto & trajectory = *old_race.response.trajectory;
            waypoints = trajectory.getWayPointCount();
            for (size_t index = 0; index < waypoints; ++index) {
              if (padded_scene_->isStateColliding(trajectory.getWayPoint(index), "manipulator")) {
                ++in_collision;
              }
            }
            end_in_collision =
              padded_scene_->isStateColliding(trajectory.getLastWayPoint(), "manipulator");
          }
          const auto new_race =
            raceResponse(goal, start_state, allowed, padded_scene_, pipelines_);
          out << band_label_ << ',' << goal.leg << ',' << goal.label << ',' << seed.label << ','
              << repeat << ',' << start_free_unpadded << ',' << start_free_padded << ','
              << old_ok << ',' << old_race.response.error_code.val << ',' << old_race.wall_ms
              << ',' << waypoints << ',' << in_collision << ',' << end_in_collision << ','
              << isSuccess(new_race.response) << ',' << new_race.response.error_code.val << ','
              << new_race.wall_ms << ',' << load1() << '\n';
          ++rows;
        }
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "planning_only_benchmark padding audit band=%s padding=%.4f m wrote %zu rows to %s "
      "(load1 %.2f -> %.2f)",
      band_label_.c_str(), robot_padding_m_, rows, output_csv_.c_str(), start_load, load1());
    return 0;
  }

  template<typename T>
  T param(const std::string & name, const T & fallback)
  {
    if (!node_->has_parameter(name)) {
      return node_->declare_parameter<T>(name, fallback);
    }
    T value{};
    if (node_->get_parameter(name, value)) {
      return value;
    }
    return fallback;
  }

  static Eigen::Isometry3d isometryFromXyzrpy(const std::vector<double> & pose)
  {
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.translation() = Eigen::Vector3d(pose[0], pose[1], pose[2]);
    transform.linear() = (Eigen::AngleAxisd(pose[5], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pose[4], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(pose[3], Eigen::Vector3d::UnitX()))
      .toRotationMatrix();
    return transform;
  }

  void buildScene(planning_scene::PlanningScene & scene)
  {
    const Eigen::Isometry3d world_from_shelf = shelf_from_world_;
    auto workcell = extract_workcell_collision_objects(
      workcell_description_, "world", world_from_shelf);
    if (!workcell) {
      throw std::invalid_argument("workcell extraction failed: " + workcell.error().detail);
    }
    size_t applied = 0;
    for (const auto & object : workcell.value()) {
      scene.processCollisionObjectMsg(object);
      ++applied;
    }
    auto catalog = ProductCollisionCatalog::load(std::filesystem::path(catalog_path_));
    if (!catalog) {
      throw std::invalid_argument("catalog load failed: " + catalog.error().detail);
    }
    auto products = load_scenario_products(std::filesystem::path(scenario_path_));
    if (!products) {
      throw std::invalid_argument("scenario load failed: " + products.error().detail);
    }
    for (const auto & product : products.value()) {
      const auto geometry = catalog.value().entries().find(product.geometry_key);
      if (geometry == catalog.value().entries().end()) {
        throw std::invalid_argument("unknown geometry_key " + product.geometry_key);
      }
      auto object = make_declared_product_collision_object(
        product.source_object_id, "world", product.planning_from_product, geometry->second);
      if (!object) {
        throw std::invalid_argument("declared product seed failed: " + object.error().detail);
      }
      scene.processCollisionObjectMsg(object.value());
      ++applied;
    }
    auto manipulation = load_manipulation_geometry(std::filesystem::path(geometry_path_));
    if (!manipulation) {
      throw std::invalid_argument("manipulation geometry failed: " + manipulation.error().detail);
    }
    // Occupied columns for the three stocked lanes, mirroring world_snapshot_projection.cpp's
    // lane_volume_object math at a fixed occupied depth (the receipts' 12-14 object scenes).
    for (const char * lane_id : {"lane_01", "lane_02", "lane_03"}) {
      const auto lane = manipulation.value().lanes.find(lane_id);
      if (lane == manipulation.value().lanes.end()) {
        throw std::invalid_argument(std::string("missing lane geometry ") + lane_id);
      }
      const auto & bounds = lane->second.usable_bounds_in_lane;
      const double occupied = lane_occupied_depth_m_;
      moveit_msgs::msg::CollisionObject object;
      object.header.frame_id = "world";
      object.id = std::string("restocker/lane/") + lane_id;
      Eigen::Isometry3d shelf_from_box = Eigen::Isometry3d::Identity();
      shelf_from_box.translation() = Eigen::Vector3d(
        lane->second.center_x_m + bounds.center().x(), bounds.max().y() - 0.5 * occupied,
        bounds.center().z());
      const Eigen::Isometry3d world_from_box = world_from_shelf * shelf_from_box;
      object.pose.position.x = world_from_box.translation().x();
      object.pose.position.y = world_from_box.translation().y();
      object.pose.position.z = world_from_box.translation().z();
      const Eigen::Quaterniond orientation(world_from_box.linear());
      object.pose.orientation.x = orientation.x();
      object.pose.orientation.y = orientation.y();
      object.pose.orientation.z = orientation.z();
      object.pose.orientation.w = orientation.w();
      shape_msgs::msg::SolidPrimitive box;
      box.type = shape_msgs::msg::SolidPrimitive::BOX;
      box.dimensions.push_back(lane_column_width_m_);
      box.dimensions.push_back(occupied);
      box.dimensions.push_back(bounds.sizes().z());
      object.primitives.push_back(box);
      geometry_msgs::msg::Pose box_origin;
      box_origin.orientation.w = 1.0;
      object.primitive_poses.push_back(box_origin);
      scene.processCollisionObjectMsg(object);
      ++applied;
    }
    scene_count_ = applied;
    RCLCPP_INFO(node_->get_logger(), "planning scene built with %zu world objects", scene_count_);
  }

  struct SeedSpec
  {
    std::string label;
    std::vector<std::string> names;
    std::vector<double> values;
  };

  // Receipts carry no joint states, so the start states are documented approximations:
  // arm zeros at rail 0 (the wrist-test seed style), the SRDF home posture at rail 0, and arm
  // zeros with the rail under the goal's x (clamped to the rail's travel).
  std::vector<SeedSpec> goalSeeds(const RecordedGoal & goal) const
  {
    const std::vector<std::string> names = {
      "rail_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
      "wrist_2_joint", "wrist_3_joint"};
    const double rail = std::max(-1.5, std::min(1.5, goal.x));
    return {
      {"zeros_rail0", names, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"home_rail0", names, {0.0, 0.0, -1.57, 0.0, -1.57, 0.0, 0.0}},
      {"zeros_rail_goal", names, {rail, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    };
  }

  void writeRow(
    std::ofstream & out, const char * mode, const RecordedGoal & goal,
    const std::string & seed, int repeat, const QueryResult & result, size_t & rows)
  {
    out << band_label_ << ',' << mode << ',' << goal.leg << ',' << goal.label << ',' << seed << ','
        << repeat << ',' << (result.ok ? 1 : 0) << ',' << result.error_code << ','
        << result.wall_ms << ',' << result.cpu_ms << ',' << result.load << '\n';
    ++rows;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<robot_model_loader::RobotModelLoader> model_loader_;
  moveit::core::RobotModelConstPtr model_;
  planning_scene::PlanningScenePtr scene_;
  planning_scene::PlanningScenePtr padded_scene_;
  Pipelines pipelines_;
  Eigen::Isometry3d shelf_from_world_{Eigen::Isometry3d::Identity()};
  std::string output_csv_;
  std::string band_label_;
  std::string scenario_path_;
  std::string catalog_path_;
  std::string geometry_path_;
  std::string workcell_description_;
  int repeats_{4};
  double lane_occupied_depth_m_{0.12};
  double lane_column_width_m_{0.10};
  bool padding_audit_{false};
  double robot_padding_m_{0.0015};
  size_t scene_count_{0};
};

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 1;
  try {
    restocker_task_executor::PlanningOnlyBenchmark benchmark;
    code = benchmark.run();
  } catch (const std::exception & error) {
    fprintf(stderr, "planning_only_benchmark failed: %s\n", error.what());
  }
  rclcpp::shutdown();
  return code;
}
