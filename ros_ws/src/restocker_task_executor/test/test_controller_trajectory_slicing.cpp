// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <string>
#include <vector>

#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit/utils/robot_model_test_utils.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "restocker_task_executor/controller_trajectory_slicing.hpp"

// Card 061 / Milestone 10 §6 "Dividing a planned trajectory into bounded controller goals".
// SC-004 slot 15 refused a 44-point, 4.246 s carry-start that fit one controller goal because
// re-timing it at path tolerance 1e-6 made MoveIt's TOTG fail ("Did not hit start trajectory").
// The fixtures below are paths for which that same re-time fails; each test first asserts that
// premise, so a MoveIt upgrade that stops failing on them is reported rather than passing
// vacuously.

namespace restocker_task_executor
{
namespace
{

using Waypoint = std::array<double, 7>;
constexpr double kScaling = 0.2;  // MotionGoal's default velocity and acceleration scaling
constexpr char kGroup[] = "arm";

// Rail plus six revolute joints with the project's joint_limits.yaml velocity/acceleration bounds.
moveit::core::RobotModelPtr rail_arm_model()
{
  moveit::core::RobotModelBuilder builder("rail_arm", "base");
  builder.addChain("base->rail", "prismatic");
  builder.addChain("rail->a->b->c->d->e->f", "revolute");
  builder.addGroupChain("base", "f", kGroup);
  auto model = builder.build();
  const std::array<double, 7> velocity{0.5, 1.8, 1.8, 2.8, 2.8, 2.8, 2.8};
  const std::array<double, 7> acceleration{0.5, 2.0, 2.0, 2.0, 3.0, 3.0, 4.0};
  const auto & names = model->getJointModelGroup(kGroup)->getActiveJointModelNames();
  for (std::size_t index = 0U; index < names.size(); ++index) {
    auto * joint = model->getJointModel(names[index]);
    auto bounds = joint->getVariableBounds(names[index]);
    bounds.velocity_bounded_ = true;
    bounds.min_velocity_ = -velocity[index];
    bounds.max_velocity_ = velocity[index];
    bounds.acceleration_bounded_ = true;
    bounds.min_acceleration_ = -acceleration[index];
    bounds.max_acceleration_ = acceleration[index];
    joint->setVariableBounds(names[index], bounds);
  }
  return model;
}

// What move_group returns for these waypoints: its default TOTG (tolerance 0.1, resample 0.1 s).
moveit_msgs::msg::RobotTrajectory planner_trajectory(
  const moveit::core::RobotModelPtr & model, const std::vector<Waypoint> & waypoints)
{
  const auto * group = model->getJointModelGroup(kGroup);
  const auto & names = group->getActiveJointModelNames();
  robot_trajectory::RobotTrajectory trajectory(model, group);
  moveit::core::RobotState state(model);
  state.setToDefaultValues();
  for (const auto & waypoint : waypoints) {
    for (std::size_t index = 0U; index < names.size(); ++index) {
      state.setVariablePosition(names[index], waypoint[index]);
    }
    state.update();
    trajectory.addSuffixWayPoint(state, 0.0);
  }
  const trajectory_processing::TimeOptimalTrajectoryGeneration move_group_default;
  EXPECT_TRUE(move_group_default.computeTimeStamps(trajectory, kScaling, kScaling));
  moveit_msgs::msg::RobotTrajectory message;
  trajectory.getRobotTrajectoryMsg(message);
  return message;
}

bool strict_retime_succeeds(
  const moveit::core::RobotModelPtr & model, const moveit_msgs::msg::RobotTrajectory & message)
{
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  robot_trajectory::RobotTrajectory trajectory(model, kGroup);
  trajectory.setRobotTrajectoryMsg(start, message);
  const trajectory_processing::TimeOptimalTrajectoryGeneration strict(
    kControllerSlicePathTolerance, kControllerSliceResampleDtS,
    kControllerSliceMinimumAngleChange);
  return strict.computeTimeStamps(trajectory, kScaling, kScaling);
}

double seconds(const trajectory_msgs::msg::JointTrajectoryPoint & point)
{
  return static_cast<double>(point.time_from_start.sec) +
         static_cast<double>(point.time_from_start.nanosec) * 1e-9;
}

ControllerGoalDivision divide(
  const moveit::core::RobotModelPtr & model, const moveit_msgs::msg::RobotTrajectory & message,
  std::chrono::milliseconds bound)
{
  moveit::core::RobotState start(model);
  start.setToDefaultValues();
  return divide_into_controller_goals(message, start, kGroup, kScaling, kScaling, bound);
}

// 14 waypoints; move_group times them as 47 points over 4.548 s — the shape of slot 15's
// 44-point, 4.246 s carry-start.
const std::vector<Waypoint> kStrictRetimeFailsWhole = {
  {0.40398204881968836, -0.71463767223806396, 0.97130956014165371, -0.26760927480465602,
    -0.34485597080265618, 0.2480932055629117, 0.38429374936003402},
  {0.43721370645048058, -0.70169890734706286, 0.88335773307003562, -0.27386561557374334,
    -0.34447723367836042, 0.27878471830619295, 0.41105701303957931},
  {0.46363090740223667, -0.69217256014235329, 0.80520240557859823, -0.28058640209203189,
    -0.34790975779163003, 0.31422429604429847, 0.42866920665322134},
  {0.47927134770406121, -0.68687432217567468, 0.74200605157300537, -0.28713845014965744,
    -0.35673280018385228, 0.35652313562869953, 0.43242230024026973},
  {0.48447589585920436, -0.68278369309446574, 0.69197685739220816, -0.29129561381165908,
    -0.36920256755222652, 0.40385943878049091, 0.42426887455870393},
  {0.48324014314993841, -0.67383338941030202, 0.64751320500804821, -0.28964218874369496,
    -0.38086814483667419, 0.45118856347011105, 0.41170757646768441},
  {0.48082036225444635, -0.6533729876948956, 0.59922311367702941, -0.27871377619383242,
    -0.38659977748644908, 0.49261424486850025, 0.40390474790553771},
  {0.48082036225444635, -0.61710181912473949, 0.54078555700111008, -0.2563353201221476,
    -0.38302155458913462, 0.5242378673168705, 0.40700916747918692},
  {0.48324014314993841, -0.56501988369983369, 0.47220053498029008, -0.22250682052864046,
    -0.37013347614473091, 0.54605943081522168, 0.42102083518863198},
  {0.48447589585920431, -0.50142785024368497, 0.39978907401261143, -0.17940333345323489,
    -0.35131145306565442, 0.56197755102234193, 0.43979097242694981},
  {0.47927134770406121, -0.43297614218458169, 0.33294315484156994, -0.13048925764786359,
    -0.33168523990265131, 0.57788849276729126, 0.45415323725581391},
  {0.46363090740223667, -0.36573204301094808, 0.27926439549532406, -0.079180297446868372,
    -0.31570575171580018, 0.59883689807963036, 0.45660898281606399},
  {0.43721370645048058, -0.30271605307534522, 0.24054460963492283, -0.027702598785210142,
    -0.30511678180790158, 0.62664456523826528, 0.44520562834972033},
  {0.40398204881968836, -0.24311248082603418, 0.2116213233547023, 0.023310654127246797,
    -0.29833907313756858, 0.65920029739172459, 0.42465120381747334},
};

// 10 waypoints, 3.357 s as planned; the whole re-times strictly, but one 1 s slice does not.
const std::vector<Waypoint> kStrictRetimeFailsOneSlice = {
  {0.7131370276534108, -0.18629507046377514, 0.69978661383836349, 0.91608488077991934,
    -0.50842200242421043, 0.43003760048113993, 0.93482422547061628},
  {0.72532707830890264, -0.14770463609459733, 0.69954597158858589, 0.95702407734169781,
    -0.55839244915390474, 0.47989066871968267, 0.99488455921789609},
  {0.74381718042659506, -0.11726653843394295, 0.71242440350455238, 1.018694852115416,
    -0.62437874049094166, 0.50506363940054999, 1.0679967717901584},
  {0.76637715480625024, -0.093447436159152575, 0.73726853924886337, 1.0981451575626455,
    -0.70595544368036023, 0.5056780328961642, 1.1533405484647448},
  {0.78251558411067978, -0.065213253509604302, 0.75879167746559995, 1.1690953808953062,
    -0.78630716207386175, 0.50664233035868167, 1.2363223230334659},
  {0.78251558411067978, -0.022062438577518793, 0.76210767811337166, 1.2062911446761857,
    -0.8487662502683857, 0.53282280935781734, 1.3026334216015378},
  {0.76637715480625024, 0.036005008637103875, 0.74721654119217862, 1.2097324489052843,
    -0.89333270826393196, 0.58421946989357132, 1.3522738441689606},
  {0.74381718042659506, 0.098487536226484498, 0.72900440674341105, 1.2046736710198136,
    -0.93667418146356118, 0.63596603439622856, 1.3995522646305183},
  {0.72532707830890264, 0.15435106843000107, 0.72275797612298798, 1.2173944238078545,
    -0.99560606651557204, 0.66315402171363269, 1.4590622491943996},
  {0.7131370276534108, 0.20206226392499427, 0.729630619668309, 1.250846754807835,
    -1.0705537961749256, 0.6656619114733614, 1.5316241125832639},
};

// Slot 15's case: the planned trajectory fits one goal and is at rest at both ends, so it
// executes as planned instead of being refused by a re-time it never needed.
TEST(ControllerTrajectorySlicing, TrajectoryThatFitsOneGoalKeepsThePlannerTimingInsteadOfRefusing)
{
  const auto model = rail_arm_model();
  const auto planned = planner_trajectory(model, kStrictRetimeFailsWhole);
  ASSERT_FALSE(strict_retime_succeeds(model, planned)) << "fixture no longer reproduces";

  const auto division = divide(model, planned, std::chrono::seconds(10));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_TRUE(division.planner_timing_kept);
  ASSERT_EQ(division.goals.size(), 1U);
  EXPECT_EQ(division.goals.front(), planned);
  EXPECT_EQ(division.fallback_retimes, 0U);
}

// A trajectory that must be divided still gets stop-to-stop slices; a slice the strict tolerance
// cannot time takes the one fallback, and the goals stay bounded and contiguous.
TEST(ControllerTrajectorySlicing, SliceTheStrictToleranceCannotTimeTakesTheSingleFallback)
{
  const auto model = rail_arm_model();
  const auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);
  ASSERT_TRUE(strict_retime_succeeds(model, planned));

  const auto division = divide(model, planned, std::chrono::seconds(1));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_FALSE(division.planner_timing_kept);
  EXPECT_GE(division.fallback_retimes, 1U);
  ASSERT_GT(division.goals.size(), 1U);
  const auto & source = planned.joint_trajectory.points;
  EXPECT_EQ(
    division.goals.front().joint_trajectory.points.front().positions,
    source.front().positions);
  EXPECT_EQ(
    division.goals.back().joint_trajectory.points.back().positions,
    source.back().positions);
  for (std::size_t index = 0U; index < division.goals.size(); ++index) {
    const auto & points = division.goals[index].joint_trajectory.points;
    ASSERT_FALSE(points.empty());
    EXPECT_LE(seconds(points.back()), 1.0);
    if (index + 1U < division.goals.size()) {
      // Adjacent goals share their boundary waypoint (TOTG re-samples it to within rounding).
      const auto & next = division.goals[index + 1U].joint_trajectory.points.front().positions;
      ASSERT_EQ(points.back().positions.size(), next.size());
      for (std::size_t joint = 0U; joint < next.size(); ++joint) {
        EXPECT_NEAR(points.back().positions[joint], next[joint], 1e-12);
      }
    }
  }
}

// The bound holds from time zero, not from the first waypoint: a planned trajectory whose first
// point is stamped after t = 0 lasts its last time_from_start as a controller goal, so it is not
// kept whole when that exceeds the bound (review N1).
TEST(ControllerTrajectorySlicing, KeptTimingIsBoundedFromTimeZeroNotFromTheFirstWaypoint)
{
  const auto model = rail_arm_model();
  auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);
  const double planned_seconds = seconds(planned.joint_trajectory.points.back());
  ASSERT_GT(planned_seconds, 3.0);
  ASSERT_LT(planned_seconds, 4.0);
  for (auto & point : planned.joint_trajectory.points) {
    point.time_from_start.sec += 1;
  }

  const auto division = divide(model, planned, std::chrono::seconds(4));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_FALSE(division.planner_timing_kept);
  for (const auto & goal : division.goals) {
    ASSERT_FALSE(goal.joint_trajectory.points.empty());
    EXPECT_LE(seconds(goal.joint_trajectory.points.back()), 4.0);
  }
}

// Fail-closed: an endpoint that is still moving is not "at rest", so the planner's timing is not
// reused as a stop-to-stop goal; the trajectory is re-timed instead.
TEST(ControllerTrajectorySlicing, MovingEndpointIsRetimedRatherThanReused)
{
  const auto model = rail_arm_model();
  auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);
  planned.joint_trajectory.points.back().velocities.front() = 0.1;

  const auto division = divide(model, planned, std::chrono::seconds(10));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_FALSE(division.planner_timing_kept);
  ASSERT_FALSE(division.goals.empty());
  EXPECT_NE(division.goals.front(), planned);
}

// Fail-closed: without velocities nothing says the endpoints are at rest.
TEST(ControllerTrajectorySlicing, MissingVelocitiesAreNotTakenAsRest)
{
  const auto model = rail_arm_model();
  auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);
  for (auto & point : planned.joint_trajectory.points) {
    point.velocities.clear();
    point.accelerations.clear();
  }

  const auto division = divide(model, planned, std::chrono::seconds(10));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_FALSE(division.planner_timing_kept);
}

// The bound is never relaxed: one waypoint gap longer than it is refused, and the refusal says so.
TEST(ControllerTrajectorySlicing, GapLongerThanTheBoundIsStillRefused)
{
  const auto model = rail_arm_model();
  auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);
  planned.joint_trajectory.points.back().time_from_start.sec += 20;

  const auto division = divide(model, planned, std::chrono::seconds(10));

  EXPECT_FALSE(division.ok());
  EXPECT_TRUE(division.goals.empty());
  EXPECT_NE(division.refusal.find("per-goal bound"), std::string::npos) << division.refusal;
}

// A trajectory longer than the bound is still divided, never passed through whole.
TEST(ControllerTrajectorySlicing, TrajectoryLongerThanTheBoundIsNeverPassedThroughWhole)
{
  const auto model = rail_arm_model();
  const auto planned = planner_trajectory(model, kStrictRetimeFailsOneSlice);

  const auto division = divide(model, planned, std::chrono::seconds(2));

  ASSERT_TRUE(division.ok()) << division.refusal;
  EXPECT_FALSE(division.planner_timing_kept);
  EXPECT_GT(division.goals.size(), 1U);
}

// Fail-closed: a model with no acceleration bounds cannot time a slice, and cannot vouch for
// rest either; the division refuses and names the step.
TEST(ControllerTrajectorySlicing, UntimeableSliceIsRefusedWithItsRange)
{
  const auto bounded = rail_arm_model();
  const auto planned = planner_trajectory(bounded, kStrictRetimeFailsOneSlice);
  moveit::core::RobotModelBuilder builder("rail_arm", "base");
  builder.addChain("base->rail", "prismatic");
  builder.addChain("rail->a->b->c->d->e->f", "revolute");
  builder.addGroupChain("base", "f", kGroup);
  const auto unbounded = builder.build();

  const auto division = divide(unbounded, planned, std::chrono::seconds(10));

  EXPECT_FALSE(division.ok());
  EXPECT_NE(division.refusal.find("could not be time-parameterised"), std::string::npos)
    << division.refusal;
}

}  // namespace
}  // namespace restocker_task_executor
