// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <limits>
#include <map>
#include <string>
#include <vector>

#include "restocker_control/control_contract.hpp"

namespace restocker_control
{
namespace
{

std::vector<ControllerObservation> healthy_controllers()
{
  return {
    {"joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster", "active", {}},
    {"arm_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"shoulder_pan_joint/velocity", "shoulder_lift_joint/velocity", "elbow_joint/velocity",
        "wrist_1_joint/velocity",
        "wrist_2_joint/velocity", "wrist_3_joint/velocity"}},
    {"rail_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"rail_joint/velocity"}},
    {"gripper_controller",
      "joint_trajectory_controller/JointTrajectoryController",
      "active",
      {"left_finger_joint/position", "right_finger_joint/position"}},
  };
}

TEST(ControlContract, AcceptsExactHealthyControllerSet) {
  EXPECT_TRUE(validate_controller_contract(healthy_controllers()).empty());
}

TEST(ControlContract, ReportsStateTypeInterfaceAndMembershipFailures) {
  auto controllers = healthy_controllers();
  controllers[0].state = "inactive";
  controllers[1].type = "wrong/Plugin";
  controllers[2].claimed_interfaces = {"wrong_joint/velocity"};
  controllers.pop_back();
  controllers.push_back({"surprise", "test/Controller", "active", {}});

  const auto errors = validate_controller_contract(controllers);
  EXPECT_EQ(errors.size(), 5U);
}

TEST(ControlContract, AcceptsPositionsInsideTolerance) {
  const std::map<std::string, double> positions{{"rail_joint", 0.401}, {"shoulder_pan_joint", 0.2}};
  const std::vector<JointTarget> targets{{"rail_joint", 0.4, 0.02},
    {"shoulder_pan_joint", 0.2, 0.05}};
  EXPECT_TRUE(validate_joint_positions(positions, targets).empty());
}

TEST(ControlContract, ReportsMissingNonFiniteAndOutOfTolerancePositions) {
  const std::map<std::string, double> positions{
    {"rail_joint", 0.5}, {"shoulder_pan_joint", std::numeric_limits<double>::quiet_NaN()}};
  const std::vector<JointTarget> targets{
    {"rail_joint", 0.4, 0.02}, {"shoulder_pan_joint", 0.2, 0.05},
    {"shoulder_lift_joint", 0.0, 0.05}};
  EXPECT_EQ(validate_joint_positions(positions, targets).size(), 3U);
}

}  // namespace
}  // namespace restocker_control
