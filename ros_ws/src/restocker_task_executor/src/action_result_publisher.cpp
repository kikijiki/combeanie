// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/action_result_publisher.hpp"

#include <stdexcept>

namespace restocker_task_executor
{

void RosActionResultPublisher::publish(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<
    restocker_interfaces::action::RestockProduct>> & goal_handle,
  ActionTerminalKind kind,
  const std::shared_ptr<restocker_interfaces::action::RestockProduct::Result> & result)
{
  switch (kind) {
    case ActionTerminalKind::kCanceled:
      goal_handle->canceled(result);
      return;
    case ActionTerminalKind::kAborted:
      goal_handle->abort(result);
      return;
    case ActionTerminalKind::kSucceeded:
      goal_handle->succeed(result);
      return;
  }
  throw std::invalid_argument("unsupported action terminal kind");
}

}  // namespace restocker_task_executor
