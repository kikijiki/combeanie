// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <memory>
#include <optional>

#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/rclcpp.hpp>

#include "restocker_control/controller_contract_probe.hpp"

namespace restocker_control
{

class RclcppControllerProbeTransport final : public ControllerProbeTransport
{
public:
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using Client = rclcpp::Client<ListControllers>;

  RclcppControllerProbeTransport(
    rclcpp::Node::SharedPtr node, Client::SharedPtr client);

  [[nodiscard]] ControllerServiceWaitStatus wait_for_service(
    ControllerProbeDuration timeout) override;
  [[nodiscard]] ControllerRequestId send_request() override;
  [[nodiscard]] ControllerResponseWaitResult wait_for_response(
    ControllerRequestId request_id, ControllerProbeDuration timeout) override;
  [[nodiscard]] ControllerRequestRemovalStatus remove_pending_request(
    ControllerRequestId request_id) override;
  [[nodiscard]] ControllerBackoffStatus wait_for_retry(
    ControllerProbeDuration timeout) override;

private:
  void require_pending(ControllerRequestId request_id) const;

  rclcpp::Node::SharedPtr node_;
  Client::SharedPtr client_;
  std::optional<Client::FutureAndRequestId> pending_;
};

}  // namespace restocker_control
