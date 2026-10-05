// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT


// Milestone 10 §6 (Card 046): the refinement ladder on the real restocker model, with MoveIt's
// own interpolator and collision checker. The repro is Card 073 slot 2's recorded fixture — the
// first pure-family partial-linear refusal — replayed exactly as `planning_probe --mode fixture`
// replays it: red at the shipped 0.005 m, green once the ladder runs. The two H1 controls (an
// unreachable target and a box across the corridor) reproduce at every step and must still be
// refused.

#include <geometric_shapes/shapes.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <functional>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#include <moveit/robot_state/cartesian_interpolator.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include "restocker_task_executor/linear_path_refinement.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr double kPlannerPaddingM = 0.0015;  // move_group's robot link padding
constexpr double kBar = 0.999;
// Card 073 slot 2's `fixture:` segment, value for value (evidence/cmbpop-073, log lines 4103+).
constexpr double kShippedStepM = 0.005;
const std::array<std::pair<const char *, double>, 9> kFixtureStart = {{
  {"rail_joint", -1.021957737},
  {"shoulder_pan_joint", -0.703778488},
  {"shoulder_lift_joint", -1.390457037},
  {"elbow_joint", 0.200411784},
  {"wrist_1_joint", 0.446924026},
  {"wrist_2_joint", -0.841127698},
  {"wrist_3_joint", -1.012152769},
  {"left_finger_joint", 0.032000000},
  {"right_finger_joint", 0.032000000},
}};
const Eigen::Vector3d kFixtureTarget(-0.6, 0.379, 0.930738522);
// (w, x, y, z) of the receipt's quat (0.5, 0.5, 0.5, -0.5).
const Eigen::Quaterniond kFixtureOrientation(-0.5, 0.5, 0.5, 0.5);

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

class LinearPathRefinementTest : public ::testing::Test
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
    node_ = std::make_shared<rclcpp::Node>("linear_path_refinement_test", options);
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
    fixture_state();
  }

  // The recorded fixture's start state and target, exactly as the port recorded them.
  void fixture_state()
  {
    const auto & known_variables = model_->getVariableNames();
    start_ = std::make_unique<moveit::core::RobotState>(model_);
    start_->setToDefaultValues();
    for (const auto & [name, value] : kFixtureStart) {
      ASSERT_NE(
        std::find(known_variables.begin(), known_variables.end(), name), known_variables.end())
        << "the model has no variable named " << name;
      start_->setVariablePosition(name, value);
    }
    start_->update();
    target_ = Eigen::Isometry3d::Identity();
    target_.translation() = kFixtureTarget;
    target_.linear() = kFixtureOrientation.normalized().toRotationMatrix();
  }

  // The port's computation: MoveIt's own interpolator, collision-checked against this scene, one
  // zero-timeout IK attempt per waypoint — the same call `planning_probe --mode fixture` makes,
  // validity callback included (tools/diagnostics/planning_probe, `run_linear`).
  double compute(
    double step_m, moveit_msgs::msg::RobotTrajectory & path,
    moveit_msgs::msg::MoveItErrorCodes & error_code, double * elapsed_ms = nullptr)
  {
    const auto validity = [this](
      moveit::core::RobotState * candidate, const moveit::core::JointModelGroup * candidate_group,
      const double * values) {
      candidate->setJointGroupPositions(candidate_group, values);
      candidate->update();
      return !scene_->isStateColliding(*candidate, candidate_group->getName());
    };
    const auto started = std::chrono::steady_clock::now();
    const std::vector<double> original_positions(
      start_->getVariablePositions(), start_->getVariablePositions() + start_->getVariableCount());
    std::vector<moveit::core::RobotStatePtr> waypoints;
    const double fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
      start_.get(), group_, waypoints, model_->getLinkModel("tool0"), target_, true,
      moveit::core::MaxEEFStep(step_m), moveit::core::CartesianPrecision{}, validity,
      kinematics::KinematicsQueryOptions());
    EXPECT_EQ(
      original_positions, std::vector<double>(
        start_->getVariablePositions(),
        start_->getVariablePositions() + start_->getVariableCount()))
      << "every refinement must start from the recorded state";
    if (elapsed_ms != nullptr) {
      *elapsed_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    }
    path = moveit_msgs::msg::RobotTrajectory();
    path.joint_trajectory.joint_names = group_->getVariableNames();
    path.joint_trajectory.points.reserve(waypoints.size());
    for (const auto & state : waypoints) {
      trajectory_msgs::msg::JointTrajectoryPoint point;
      state->copyJointGroupPositions(group_, point.positions);
      path.joint_trajectory.points.push_back(std::move(point));
    }
    error_code.val = fraction < 0.0 ? moveit_msgs::msg::MoveItErrorCodes::FAILURE :
      moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
    return fraction;
  }

  ComputeLinearPath line()
  {
    return [this](
      double step_m, moveit_msgs::msg::RobotTrajectory & path,
      moveit_msgs::msg::MoveItErrorCodes & code) {
             return compute(step_m, path, code);
           };
  }

  static std::function<bool()> always_in_budget() {return []() {return true;};}
  static std::function<bool()> no_budget_left() {return []() {return false;};}

  static rclcpp::Node::SharedPtr node_;
  static robot_model_loader::RobotModelLoaderPtr loader_;
  static moveit::core::RobotModelConstPtr model_;
  const moveit::core::JointModelGroup * group_{nullptr};
  planning_scene::PlanningScenePtr scene_;
  std::unique_ptr<moveit::core::RobotState> start_;
  Eigen::Isometry3d target_;
};

rclcpp::Node::SharedPtr LinearPathRefinementTest::node_;
robot_model_loader::RobotModelLoaderPtr LinearPathRefinementTest::loader_;
moveit::core::RobotModelConstPtr LinearPathRefinementTest::model_;

// The recorded refusal itself: red at the shipped step, green with the ladder (§6, Card 046).
//
// The stop is a knife edge and this test says so out loud: two identical calls in one process
// have been measured one waypoint apart (0.932960894 then 0.927374302) and, rarely, the
// shipped-step call clears the bar outright (observed once, under ctest). The repro is therefore
// *sought* over a bounded number of calls instead of being assumed on call one — which is the
// card's own H1-vs-H2 point: a stop that only sometimes happens is not a kinematic limit.
TEST_F(LinearPathRefinementTest, RecordedFixtureIsPartialAtTheShippedStepAndCompletesWhenRefined)
{
  for (int call = 1; call <= 5; ++call) {
    std::vector<double> elapsed_ms;
    const LinearPathRefinement refinement = refine_linear_path(
      kShippedStepM, kBar,
      [this, &elapsed_ms](
        double step_m, moveit_msgs::msg::RobotTrajectory & attempt_path,
        moveit_msgs::msg::MoveItErrorCodes & attempt_code) {
        double ms = 0.0;
        const double fraction = compute(step_m, attempt_path, attempt_code, &ms);
        elapsed_ms.push_back(ms);
        return fraction;
      },
      always_in_budget());
    if (refinement.primary().fraction >= kBar) {
      // This call cleared the bar at the shipped step, so there is no stop to refine: try again.
      std::printf(
        "# card046 repro: call %d of 5 cleared the shipped step (%.9f), retrying\n", call,
        refinement.primary().fraction);
      continue;
    }
    std::printf(
      "# card046 repro: stopped at the shipped step on call %d of 5 (%.9f)\n", call,
      refinement.primary().fraction);
    EXPECT_TRUE(refinement.complete()) << refinement.receipt();
    ASSERT_GE(refinement.attempts.size(), 2U) << refinement.receipt();
    ASSERT_LE(refinement.attempts.size(), 3U) << refinement.receipt();
    EXPECT_GE(refinement.executed().fraction, kBar);        // the green the ladder buys
    EXPECT_LT(refinement.executed().step_m, kShippedStepM);
    EXPECT_GT(refinement.executed().path.joint_trajectory.points.size(), 1U);
    EXPECT_FALSE(refinement.budget_spent);

    std::printf("# card046 refinement timing on the recorded fixture (step=ms):");
    for (std::size_t index = 0U; index < refinement.attempts.size(); ++index) {
      std::printf(
        " %.4f=% .3f", refinement.attempts[index].step_m,
        index < elapsed_ms.size() ? elapsed_ms[index] : 0.0);
    }
    std::printf(" ms\n");
    return;
  }
  FAIL() << "the recorded fixture did not stop at the shipped 0.005 m step in 5 calls: the repro "
    "this card rests on did not reproduce";
}

// A genuine kinematic stop: a target no configuration on this line can reach stops at 0 % at
// every step, so the ladder exhausts and the refusal stands (§6, Card 046).
TEST_F(LinearPathRefinementTest, UnreachableTargetIsStillRefusedAtEveryStep)
{
  target_.translation() = Eigen::Vector3d(5.0, 0.379, 0.930738522);
  const LinearPathRefinement refinement = refine_linear_path(
    kShippedStepM, kBar, line(), always_in_budget());

  EXPECT_FALSE(refinement.complete());
  ASSERT_EQ(refinement.attempts.size(), 3U) << refinement.receipt();
  for (const auto & attempt : refinement.attempts) {
    EXPECT_LT(attempt.fraction, kBar) <<
      "step " << attempt.step_m << " reached " << attempt.fraction;
  }
  const std::string receipt = refinement.receipt();
  EXPECT_NE(receipt.find("linear refinement: step 0.005000 m reached"), std::string::npos) <<
    receipt;
  EXPECT_NE(receipt.find("step 0.001000 m reached"), std::string::npos) << receipt;
  EXPECT_NE(receipt.find("step 0.000500 m reached"), std::string::npos) << receipt;
  EXPECT_NE(receipt.find("bar 0.999000"), std::string::npos) << receipt;
  std::printf("# card046 H1 kinematic control: %s\n", receipt.c_str());
}

// A genuine collision stop: a box across the corridor stops the checked line at the same place
// at every step, so refining never rescues it and the refusal stands (§6, Card 046).
TEST_F(LinearPathRefinementTest, BoxAcrossTheCorridorIsStillRefusedAtEveryStep)
{
  const Eigen::Vector3d from = start_->getGlobalLinkTransform("tool0").translation();
  const Eigen::Vector3d at = (from + target_.translation()) / 2.0;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = at;
  scene_->getWorldNonConst()->addToObject(
    "corridor_blocker", std::make_shared<shapes::Box>(0.10, 0.10, 0.10), pose);
  start_->update();
  ASSERT_FALSE(scene_->isStateColliding(*start_, "", false)) <<
    "the blocker must sit on the line, not on the start state";

  const LinearPathRefinement refinement = refine_linear_path(
    kShippedStepM, kBar, line(), always_in_budget());

  EXPECT_FALSE(refinement.complete()) << refinement.receipt();
  ASSERT_EQ(refinement.attempts.size(), 3U) << refinement.receipt();
  double lowest = 1.0;
  double highest = 0.0;
  for (const auto & attempt : refinement.attempts) {
    EXPECT_LT(attempt.fraction, kBar) <<
      "step " << attempt.step_m << " reached " << attempt.fraction;
    lowest = std::min(lowest, attempt.fraction);
    highest = std::max(highest, attempt.fraction);
  }
  std::printf("# card046 H1 collision control: %s\n", refinement.receipt().c_str());
  EXPECT_LT(highest - lowest, 0.05) <<
    "a collision stop reproduces at every step, not one step away: " << lowest << ".." << highest;
}

// The ladder is bounded and never spends a budget it does not have. The unreachable target is
// used so no refinement can accidentally complete.
TEST_F(LinearPathRefinementTest, BudgetAndLadderBoundsAreEnforced)
{
  target_.translation() = Eigen::Vector3d(5.0, 0.379, 0.930738522);

  const LinearPathRefinement no_budget = refine_linear_path(
    kShippedStepM, kBar, line(), no_budget_left());
  EXPECT_FALSE(no_budget.complete());
  EXPECT_TRUE(no_budget.attempts.empty()) << "an expired budget admits no service request";
  EXPECT_TRUE(no_budget.budget_spent);
  EXPECT_NE(no_budget.receipt().find("budget spent"), std::string::npos) << no_budget.receipt();

  const LinearPathRefinement coarser = refine_linear_path(
    kShippedStepM, kBar, line(), always_in_budget(), {0.01, 0.02});
  EXPECT_FALSE(coarser.complete());
  EXPECT_EQ(coarser.attempts.size(), 1U) << "a coarser step is never a refinement";
  EXPECT_FALSE(coarser.budget_spent);

  const LinearPathRefinement too_many = refine_linear_path(
    kShippedStepM, kBar, line(), always_in_budget(), {0.004, 0.003, 0.002});
  EXPECT_FALSE(too_many.complete());
  EXPECT_EQ(too_many.attempts.size(), 1U + kMaximumLinearRefinements) << too_many.receipt();
}

// A line that meets the bar at the shipped step costs exactly one attempt — no ladder on the
// path every green run takes. The knife edge applies in reverse as well, so a line that stops
// once is retried rather than failed, up to five calls.
TEST_F(LinearPathRefinementTest, ACompleteLineIsNotRefined)
{
  target_ = start_->getGlobalLinkTransform("tool0");
  target_.translation().z() += 0.01;
  for (int call = 1; call <= 5; ++call) {
    const LinearPathRefinement refinement = refine_linear_path(
      kShippedStepM, kBar, line(), always_in_budget());
    if (refinement.complete() && refinement.attempts.size() == 1U) {
      return;
    }
    std::printf(
      "# card046 green-path check: call %d of 5 needed the ladder (%s), retrying\n", call,
      refinement.receipt().c_str());
  }
  FAIL() << "a 1 cm line never cleared the bar at the shipped step in 5 calls";
}


// Policy tests use scripted service responses and a fake monotonic clock. They do not depend on
// KDL choosing the same solution on consecutive calls or on scheduler timing.
class LinearPathRefinementPolicyTest : public ::testing::Test
{
protected:
  LinearPathRefinement run(
    const std::vector<double> & fractions,
    std::vector<double> steps = {kLinearRefinementStepFineM, kLinearRefinementStepFinerM})
  {
    std::size_t response_index = 0U;
    return refine_linear_path(
      kShippedStepM, kBar,
      [this, &fractions, &response_index](double, auto & path, auto & code) {
        const auto index = calls++;
        path.joint_trajectory.joint_names = {"attempt_" + std::to_string(index)};
        code.val = response_code;
        now += call_duration;
        if (calls == cancel_on_call) {
          canceled = true;
        }
        return fractions.at(response_index++);
      },
      [this]() {return now < deadline;}, steps, [this]() {return canceled;});
  }

  std::size_t calls{0U};
  std::size_t cancel_on_call{std::numeric_limits<std::size_t>::max()};
  bool canceled{false};
  std::chrono::milliseconds now{0};
  std::chrono::milliseconds deadline{100};
  std::chrono::milliseconds call_duration{1};
  int response_code{moveit_msgs::msg::MoveItErrorCodes::SUCCESS};
};

TEST_F(LinearPathRefinementPolicyTest, FirstCompleteResponseStopsAndPreservesItsTrajectory)
{
  const auto result = run({0.92, 1.0});
  ASSERT_TRUE(result.complete()) << result.receipt();
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_EQ(calls, 2U);
  EXPECT_EQ(result.accepted, 1U);
  EXPECT_EQ(
    result.executed().path.joint_trajectory.joint_names,
    std::vector<std::string>{"attempt_1"});
  EXPECT_DOUBLE_EQ(result.primary().fraction, 0.92);
}

TEST_F(LinearPathRefinementPolicyTest, CompletePrimaryDoesNotRequestRefinement)
{
  const auto result = run({1.0});
  ASSERT_TRUE(result.complete());
  EXPECT_EQ(calls, 1U);
  EXPECT_EQ(result.accepted, 0U);
}

TEST_F(LinearPathRefinementPolicyTest, ExpiredBudgetDoesNotStartPrimary)
{
  now = deadline;
  const auto result = run({});
  EXPECT_EQ(calls, 0U);
  EXPECT_TRUE(result.attempts.empty());
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.budget_spent);
  EXPECT_NE(result.receipt().find("budget spent"), std::string::npos);
}

TEST_F(LinearPathRefinementPolicyTest, CancellationDoesNotStartPrimary)
{
  canceled = true;
  const auto result = run({});
  EXPECT_EQ(calls, 0U);
  EXPECT_TRUE(result.attempts.empty());
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.canceled);
  EXPECT_FALSE(result.budget_spent);
  EXPECT_NE(result.receipt().find("cancellation"), std::string::npos);
}

TEST_F(LinearPathRefinementPolicyTest, CancellationDuringPartialPrimaryStopsBeforeRefinement)
{
  cancel_on_call = 1U;
  const auto result = run({0.92});
  EXPECT_EQ(calls, 1U);
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.canceled);
  ASSERT_EQ(result.attempts.size(), 1U);
  EXPECT_DOUBLE_EQ(result.primary().fraction, 0.92);
}

TEST_F(LinearPathRefinementPolicyTest, CompleteResponseAfterCancellationIsNeverAccepted)
{
  cancel_on_call = 2U;
  const auto result = run({0.92, 1.0});
  EXPECT_EQ(calls, 2U);
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.canceled);
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_DOUBLE_EQ(result.attempts.back().fraction, 1.0);
}

TEST_F(LinearPathRefinementPolicyTest, PartialResponseAtDeadlineStopsBeforeRefinement)
{
  call_duration = deadline;
  const auto result = run({0.92});
  EXPECT_EQ(calls, 1U);
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.budget_spent);
  EXPECT_FALSE(result.canceled);
}

TEST_F(LinearPathRefinementPolicyTest, CompleteResponseAtDeadlineIsNeverAccepted)
{
  call_duration = deadline / 2;
  const auto result = run({0.92, 1.0});
  EXPECT_EQ(calls, 2U);
  EXPECT_FALSE(result.complete());
  EXPECT_TRUE(result.budget_spent);
  ASSERT_EQ(result.attempts.size(), 2U);
  EXPECT_DOUBLE_EQ(result.attempts.back().fraction, 1.0);
}

TEST_F(LinearPathRefinementPolicyTest, RequiredEgressSharesTheCommandedLinesDeadline)
{
  call_duration = deadline * 3 / 4;
  ASSERT_TRUE(run({1.0}).complete());
  // The command spent 75 of 100 ms; a second 75 ms call cannot prove its required egress.
  const auto proof = run({1.0});
  EXPECT_EQ(calls, 2U);
  EXPECT_FALSE(proof.complete());
  EXPECT_TRUE(proof.budget_spent);
  ASSERT_EQ(proof.attempts.size(), 1U);
  EXPECT_DOUBLE_EQ(proof.primary().fraction, 1.0);
}

TEST_F(LinearPathRefinementPolicyTest, InvalidRefinementStepsAreSkippedAndAttemptsStayBounded)
{
  const auto result = run(
    {0.92, 0.93, 0.94},
    {0.01, 0.0, -0.001, std::numeric_limits<double>::quiet_NaN(), 0.004, 0.004, 0.003, 0.002});
  EXPECT_FALSE(result.complete());
  ASSERT_EQ(result.attempts.size(), 1U + kMaximumLinearRefinements);
  EXPECT_DOUBLE_EQ(result.attempts[1].step_m, 0.004);
  EXPECT_DOUBLE_EQ(result.attempts[2].step_m, 0.003);
  EXPECT_EQ(calls, 3U);
}

TEST_F(LinearPathRefinementPolicyTest, NonfiniteAndOutOfRangeFractionsCannotAuthorizeExecution)
{
  const auto result = run(
    {std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN(), 1.01});
  EXPECT_FALSE(result.complete());
  EXPECT_EQ(calls, 3U);
}

TEST_F(LinearPathRefinementPolicyTest, FailedServiceCannotAuthorizeExecutionWithACompleteFraction)
{
  response_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
  const auto result = run({1.0, 1.0, 1.0});
  EXPECT_FALSE(result.complete());
  EXPECT_EQ(calls, 3U);
}

}  // namespace
}  // namespace restocker_task_executor
