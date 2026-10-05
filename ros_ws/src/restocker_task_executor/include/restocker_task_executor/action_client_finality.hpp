// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <string>

#include <action_msgs/srv/cancel_goal.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/types.hpp>

namespace restocker_task_executor
{

// Milestone 10 §6 result finality (Card 070): whether a result code an action client received is
// a terminal the server delivered. rclcpp_action fulfils a result future with code UNKNOWN (0)
// when its result request found no registered goal (an overtaken admission, an expired or
// restarted server), which says nothing about the goal's outcome: it may still be running. Codes
// are action_msgs/GoalStatus values, which rclcpp_action::ResultCode mirrors.
[[nodiscard]] bool delivered_terminal(std::int8_t result_code) noexcept;

using GoalCancelClient = rclcpp::Client<action_msgs::srv::CancelGoal>;

// The raw cancel service of `action_name`. rclcpp_action's client forgets a goal as soon as it has
// fulfilled that goal's result future, delivered terminal or not, so async_cancel_goal(handle)
// throws UnknownGoalHandleError exactly when a client needs to give up on an undelivered goal. The
// cancel for such a goal therefore goes by its ID through the action's own cancel service.
[[nodiscard]] GoalCancelClient::SharedPtr create_goal_cancel_client(
  rclcpp::Node & node, const std::string & action_name);

// True when `result_code` is a delivered terminal. Otherwise requests cancellation of exactly
// `goal_id` (fire and forget, like the clients' timeout branches) and returns false: the caller
// must treat the goal as possibly still running and its effects as unknown.
bool accept_delivered_terminal_or_cancel(
  std::int8_t result_code, const rclcpp_action::GoalUUID & goal_id,
  GoalCancelClient & cancel_client);

}  // namespace restocker_task_executor
