// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <memory>

#include <rclcpp_action/server_goal_handle.hpp>
#include <restocker_interfaces/action/restock_product.hpp>

namespace restocker_task_executor
{

enum class ActionTerminalKind : std::uint8_t
{
  kCanceled,
  kAborted,
  kSucceeded,
};

class ActionResultPublisher
{
public:
  virtual ~ActionResultPublisher() = default;

  virtual void publish(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      restocker_interfaces::action::RestockProduct>> & goal_handle,
    ActionTerminalKind kind,
    const std::shared_ptr<restocker_interfaces::action::RestockProduct::Result> & result) = 0;
};

class RosActionResultPublisher final : public ActionResultPublisher
{
public:
  void publish(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
      restocker_interfaces::action::RestockProduct>> & goal_handle,
    ActionTerminalKind kind,
    const std::shared_ptr<restocker_interfaces::action::RestockProduct::Result> & result) override;
};

}  // namespace restocker_task_executor
