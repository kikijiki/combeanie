// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/control_contract.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace restocker_control
{
namespace
{

struct ExpectedController
{
  std::string type;
  std::set<std::string> claimed_interfaces;
};

const std::map<std::string, ExpectedController> kExpectedControllers{
  {"joint_state_broadcaster", {"joint_state_broadcaster/JointStateBroadcaster", {}}},
  // Arm and rail feed planned velocity forward; the gripper keeps position for attachment
  // accuracy. See config/controllers.yaml.
  {"arm_controller",
    {"joint_trajectory_controller/JointTrajectoryController",
      {"shoulder_pan_joint/velocity", "shoulder_lift_joint/velocity", "elbow_joint/velocity",
        "wrist_1_joint/velocity",
        "wrist_2_joint/velocity", "wrist_3_joint/velocity"}}},
  {"rail_controller",
    {"joint_trajectory_controller/JointTrajectoryController", {"rail_joint/velocity"}}},
  {"gripper_controller",
    {"joint_trajectory_controller/JointTrajectoryController",
      {"left_finger_joint/position", "right_finger_joint/position"}}},
};

std::string join(const std::set<std::string> & values)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & value : values) {
    if (!first) {
      stream << ", ";
    }
    stream << value;
    first = false;
  }
  return stream.str();
}

}  // namespace

std::vector<std::string>
validate_controller_contract(const std::vector<ControllerObservation> & observations)
{
  std::vector<std::string> errors;
  std::map<std::string, ControllerObservation> by_name;
  for (const auto & observation : observations) {
    if (!by_name.emplace(observation.name, observation).second) {
      errors.emplace_back("duplicate controller: " + observation.name);
    }
  }

  for (const auto & [name, expected] : kExpectedControllers) {
    const auto found = by_name.find(name);
    if (found == by_name.end()) {
      errors.emplace_back("missing controller: " + name);
      continue;
    }

    const auto & actual = found->second;
    if (actual.state != "active") {
      errors.emplace_back(name + " is " + actual.state + ", expected active");
    }
    if (actual.type != expected.type) {
      errors.emplace_back(name + " has type " + actual.type + ", expected " + expected.type);
    }

    const std::set<std::string> claimed(actual.claimed_interfaces.begin(),
      actual.claimed_interfaces.end());
    if (claimed != expected.claimed_interfaces) {
      errors.emplace_back(
        name + " claims [" + join(claimed) + "], expected [" +
        join(expected.claimed_interfaces) + "]");
    }
  }

  for (const auto & [name, _observation] : by_name) {
    if (!kExpectedControllers.contains(name)) {
      errors.emplace_back("unexpected controller: " + name);
    }
  }
  return errors;
}

std::vector<std::string> validate_joint_positions(
  const std::map<std::string, double> & positions,
  const std::vector<JointTarget> & targets)
{
  std::vector<std::string> errors;
  for (const auto & target : targets) {
    const auto found = positions.find(target.name);
    if (found == positions.end()) {
      errors.emplace_back("joint state is missing " + target.name);
      continue;
    }
    if (!std::isfinite(found->second)) {
      errors.emplace_back("joint position is not finite: " + target.name);
      continue;
    }
    const double error = std::abs(found->second - target.position);
    if (error > target.tolerance) {
      std::ostringstream stream;
      stream << target.name << " position error " << error << " exceeds tolerance "
             << target.tolerance;
      errors.push_back(stream.str());
    }
  }
  return errors;
}

}  // namespace restocker_control
