// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <map>
#include <string>
#include <vector>

namespace restocker_control
{

struct ControllerObservation
{
  std::string name;
  std::string type;
  std::string state;
  std::vector<std::string> claimed_interfaces;
};

struct JointTarget
{
  std::string name;
  double position;
  double tolerance;
};

[[nodiscard]] std::vector<std::string>
validate_controller_contract(const std::vector<ControllerObservation> & observations);

[[nodiscard]] std::vector<std::string>
validate_joint_positions(
  const std::map<std::string, double> & positions,
  const std::vector<JointTarget> & targets);

}  // namespace restocker_control
