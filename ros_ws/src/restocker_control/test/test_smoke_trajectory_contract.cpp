// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <vector>

#include "restocker_control/smoke_trajectory_contract.hpp"

namespace restocker_control
{
namespace
{

using namespace std::chrono_literals;

TEST(SmokeTrajectoryContract, DefinesStrictInteriorFingerInitialState)
{
  const auto targets = smoke_initial_state_contract();
  ASSERT_EQ(targets.size(), 2U);
  EXPECT_EQ(targets.at(0U).name, "left_finger_joint");
  EXPECT_DOUBLE_EQ(targets.at(0U).position, 0.005);
  EXPECT_DOUBLE_EQ(targets.at(0U).tolerance, 0.003);
  EXPECT_GT(targets.at(0U).position - targets.at(0U).tolerance, 0.0);
  EXPECT_EQ(targets.at(1U).name, "right_finger_joint");
  EXPECT_DOUBLE_EQ(targets.at(1U).position, 0.005);
  EXPECT_DOUBLE_EQ(targets.at(1U).tolerance, 0.003);
  EXPECT_GT(targets.at(1U).position - targets.at(1U).tolerance, 0.0);
}

TEST(SmokeTrajectoryContract, DefinesStrictSymmetricGripperCommand)
{
  const auto commands = smoke_trajectory_contract();
  ASSERT_EQ(commands.size(), 3U);
  const auto & gripper = commands.at(2U);
  EXPECT_EQ(gripper.controller, "gripper_controller");
  EXPECT_EQ(gripper.duration, 2s);
  ASSERT_EQ(gripper.targets.size(), 2U);
  EXPECT_EQ(gripper.targets.at(0U).name, "left_finger_joint");
  EXPECT_DOUBLE_EQ(gripper.targets.at(0U).position, 0.03);
  EXPECT_DOUBLE_EQ(gripper.targets.at(0U).tolerance, 0.003);
  EXPECT_EQ(gripper.targets.at(1U).name, "right_finger_joint");
  EXPECT_DOUBLE_EQ(gripper.targets.at(1U).position, 0.03);
  EXPECT_DOUBLE_EQ(gripper.targets.at(1U).tolerance, 0.003);
}

TEST(SmokeTrajectoryContract, KeepsControllerOrderAndJointPartitionStable)
{
  const auto commands = smoke_trajectory_contract();
  ASSERT_EQ(commands.size(), 3U);
  EXPECT_EQ(commands.at(0U).controller, "rail_controller");
  EXPECT_EQ(commands.at(0U).duration, 4s);
  EXPECT_EQ(commands.at(1U).controller, "arm_controller");
  EXPECT_EQ(commands.at(1U).duration, 4s);
  EXPECT_EQ(commands.at(2U).controller, "gripper_controller");

  std::vector<std::string> joints;
  for (const auto & command : commands) {
    for (const auto & target : command.targets) {
      joints.push_back(target.name);
    }
  }
  EXPECT_EQ(
    joints,
    (std::vector<std::string>{
        "rail_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
        "wrist_2_joint", "wrist_3_joint",
        "left_finger_joint", "right_finger_joint"}));
}

}  // namespace
}  // namespace restocker_control
