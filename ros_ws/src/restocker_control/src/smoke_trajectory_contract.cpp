// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/smoke_trajectory_contract.hpp"

#include <chrono>
#include <vector>

namespace restocker_control
{

using namespace std::chrono_literals;

std::vector<JointTarget> smoke_initial_state_contract()
{
  return {
    {"left_finger_joint", 0.005, 0.003},
    {"right_finger_joint", 0.005, 0.003},
  };
}

std::vector<SmokeTrajectoryCommand> smoke_trajectory_contract()
{
  return {
    {"rail_controller", {{"rail_joint", 0.4, 0.02}}, 4s},
    {"arm_controller",
      {{"shoulder_pan_joint", 0.2, 0.05},
        {"shoulder_lift_joint", -0.4, 0.05},
        {"elbow_joint", 0.6, 0.05},
        {"wrist_1_joint", 0.1, 0.05},
        {"wrist_2_joint", 0.2, 0.05},
        {"wrist_3_joint", -0.1, 0.05}},
      4s},
    {"gripper_controller",
      {{"left_finger_joint", 0.03, 0.003}, {"right_finger_joint", 0.03, 0.003}},
      2s},
  };
}

}  // namespace restocker_control
