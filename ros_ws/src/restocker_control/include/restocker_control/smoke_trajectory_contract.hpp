// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "restocker_control/control_contract.hpp"

namespace restocker_control
{

struct SmokeTrajectoryCommand
{
  std::string controller;
  std::vector<JointTarget> targets;
  std::chrono::seconds duration;
};

[[nodiscard]] std::vector<JointTarget> smoke_initial_state_contract();

[[nodiscard]] std::vector<SmokeTrajectoryCommand> smoke_trajectory_contract();

}  // namespace restocker_control
