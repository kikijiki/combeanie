// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_control/controller_contract_probe_ros.hpp"

#include <stdexcept>
#include <utility>

namespace restocker_control
{

RclcppControllerProbeTransport::RclcppControllerProbeTransport(
  rclcpp::Node::SharedPtr node, Client::SharedPtr client)
: node_(std::move(node)), client_(std::move(client))
{
  if (!node_ || !client_) {
    throw std::invalid_argument("controller probe ROS transport requires a node and client");
  }
}

ControllerServiceWaitStatus RclcppControllerProbeTransport::wait_for_service(
  ControllerProbeDuration timeout)
{
  if (client_->wait_for_service(timeout)) {
    return ControllerServiceWaitStatus::kAvailable;
  }
  const auto context = node_->get_node_base_interface()->get_context();
  return rclcpp::ok(context) ? ControllerServiceWaitStatus::kTimeout :
         ControllerServiceWaitStatus::kInterrupted;
}

ControllerRequestId RclcppControllerProbeTransport::send_request()
{
  if (pending_) {
    throw std::logic_error("controller probe already owns a pending request");
  }
  pending_.emplace(client_->async_send_request(std::make_shared<ListControllers::Request>()));
  return pending_->request_id;
}

ControllerResponseWaitResult RclcppControllerProbeTransport::wait_for_response(
  ControllerRequestId request_id, ControllerProbeDuration timeout)
{
  require_pending(request_id);
  const auto wait_status = rclcpp::spin_until_future_complete(node_, *pending_, timeout);
  if (wait_status == rclcpp::FutureReturnCode::TIMEOUT) {
    return {ControllerResponseWaitStatus::kTimeout, {}};
  }
  if (wait_status == rclcpp::FutureReturnCode::INTERRUPTED) {
    return {ControllerResponseWaitStatus::kInterrupted, {}};
  }

  std::vector<ControllerObservation> observations;
  const auto response = pending_->get();
  pending_.reset();
  observations.reserve(response->controller.size());
  for (const auto & controller : response->controller) {
    observations.push_back(
      {controller.name, controller.type, controller.state, controller.claimed_interfaces});
  }
  return {ControllerResponseWaitStatus::kResponse, std::move(observations)};
}

ControllerRequestRemovalStatus RclcppControllerProbeTransport::remove_pending_request(
  ControllerRequestId request_id)
{
  require_pending(request_id);
  const bool removed = client_->remove_pending_request(request_id);
  pending_.reset();
  return removed ? ControllerRequestRemovalStatus::kRemoved :
         ControllerRequestRemovalStatus::kAlreadyAbsent;
}

ControllerBackoffStatus RclcppControllerProbeTransport::wait_for_retry(
  ControllerProbeDuration timeout)
{
  const auto context = node_->get_node_base_interface()->get_context();
  return context->sleep_for(timeout) ? ControllerBackoffStatus::kElapsed :
         ControllerBackoffStatus::kInterrupted;
}

void RclcppControllerProbeTransport::require_pending(ControllerRequestId request_id) const
{
  if (!pending_ || pending_->request_id != request_id) {
    throw std::logic_error("controller probe request identity does not match pending request");
  }
}

}  // namespace restocker_control
