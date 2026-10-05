// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/action_client_finality.hpp"

#include <memory>
#include <string>

#include <action_msgs/msg/goal_status.hpp>

namespace restocker_task_executor
{

bool delivered_terminal(std::int8_t result_code) noexcept
{
  using action_msgs::msg::GoalStatus;
  return result_code == GoalStatus::STATUS_SUCCEEDED ||
         result_code == GoalStatus::STATUS_CANCELED ||
         result_code == GoalStatus::STATUS_ABORTED;
}

GoalCancelClient::SharedPtr create_goal_cancel_client(
  rclcpp::Node & node, const std::string & action_name)
{
  return node.create_client<action_msgs::srv::CancelGoal>(action_name + "/_action/cancel_goal");
}

bool accept_delivered_terminal_or_cancel(
  std::int8_t result_code, const rclcpp_action::GoalUUID & goal_id,
  GoalCancelClient & cancel_client)
{
  if (delivered_terminal(result_code)) {
    return true;
  }
  // A non-zero goal ID with a zero stamp cancels exactly that goal (action_msgs/CancelGoal).
  auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
  request->goal_info.goal_id.uuid = goal_id;
  (void)cancel_client.async_send_request(request);
  return false;
}

}  // namespace restocker_task_executor
