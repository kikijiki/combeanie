// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/world_state_async_port.hpp"

#include <exception>
#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] bool valid_correlation(OperationCorrelation correlation)
{
  return correlation.goal_generation > 0 && correlation.operation_generation > 0;
}

template<typename Service>
AsyncSendResult send_request(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  WorldStateServiceKind service, OperationCorrelation correlation,
  std::shared_ptr<typename Service::Request> request,
  WorldStateAsyncPort::CompletionCallback<Service> callback)
{
  if (!valid_correlation(correlation) || !request || !callback) {
    return {
      AsyncSendErrorCode::kInvalidArgument, std::nullopt,
      "correlation generations, request, and completion callback are required"};
  }

  try {
    auto pending = client->async_send_request(
      std::move(request),
      [correlation, callback = std::move(callback)](
        typename rclcpp::Client<Service>::SharedFuture future)
      {
        AsyncServiceCompletion<typename Service::Response> completion;
        completion.correlation = correlation;
        try {
          completion.response = future.get();
          if (!completion.response) {
            completion.transport_error = "service completed without a response";
          }
        } catch (const std::exception & error) {
          completion.transport_error = error.what();
        } catch (...) {
          completion.transport_error = "service completion raised an unknown exception";
        }
        callback(std::move(completion));
      });
    return {
      AsyncSendErrorCode::kNone,
      WorldStateRequestHandle{correlation, service, pending.request_id}, {}};
  } catch (const std::exception & error) {
    return {AsyncSendErrorCode::kTransportRejected, std::nullopt, error.what()};
  } catch (...) {
    return {
      AsyncSendErrorCode::kTransportRejected, std::nullopt,
      "service request submission raised an unknown exception"};
  }
}

}  // namespace

WorldStateAsyncPort::WorldStateAsyncPort(
  rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr completion_group,
  WorldStateServiceNames names)
{
  if (!completion_group) {
    throw std::invalid_argument("world-state completion callback group is required");
  }
  if (completion_group->type() != rclcpp::CallbackGroupType::Reentrant) {
    throw std::invalid_argument("world-state completion callback group must be reentrant");
  }
  completion_group_ = std::move(completion_group);
  get_snapshot_client_ = node.create_client<GetSnapshot>(
    names.get_snapshot, rclcpp::ServicesQoS(), completion_group_);
  reserve_task_client_ = node.create_client<ReserveTask>(
    names.reserve_task, rclcpp::ServicesQoS(), completion_group_);
  validate_reservation_client_ = node.create_client<ValidateReservation>(
    names.validate_reservation, rclcpp::ServicesQoS(), completion_group_);
  release_reservation_client_ = node.create_client<ReleaseReservation>(
    names.release_reservation, rclcpp::ServicesQoS(), completion_group_);
  validate_execution_authority_client_ = node.create_client<ValidateExecutionAuthority>(
    names.validate_execution_authority, rclcpp::ServicesQoS(), completion_group_);
}

WorldStateServiceReadiness WorldStateAsyncPort::readiness() const
{
  return {
    get_snapshot_client_->service_is_ready(), reserve_task_client_->service_is_ready(),
    validate_reservation_client_->service_is_ready(),
    release_reservation_client_->service_is_ready(),
    validate_execution_authority_client_->service_is_ready()};
}

AsyncSendResult WorldStateAsyncPort::get_snapshot(
  OperationCorrelation correlation, std::shared_ptr<GetSnapshot::Request> request,
  CompletionCallback<GetSnapshot> callback)
{
  return send_request<GetSnapshot>(
    get_snapshot_client_, WorldStateServiceKind::kGetSnapshot, correlation,
    std::move(request), std::move(callback));
}

AsyncSendResult WorldStateAsyncPort::reserve_task(
  OperationCorrelation correlation, std::shared_ptr<ReserveTask::Request> request,
  CompletionCallback<ReserveTask> callback)
{
  return send_request<ReserveTask>(
    reserve_task_client_, WorldStateServiceKind::kReserveTask, correlation,
    std::move(request), std::move(callback));
}

AsyncSendResult WorldStateAsyncPort::validate_reservation(
  OperationCorrelation correlation, std::shared_ptr<ValidateReservation::Request> request,
  CompletionCallback<ValidateReservation> callback)
{
  return send_request<ValidateReservation>(
    validate_reservation_client_, WorldStateServiceKind::kValidateReservation,
    correlation, std::move(request), std::move(callback));
}

AsyncSendResult WorldStateAsyncPort::release_reservation(
  OperationCorrelation correlation, std::shared_ptr<ReleaseReservation::Request> request,
  CompletionCallback<ReleaseReservation> callback)
{
  return send_request<ReleaseReservation>(
    release_reservation_client_, WorldStateServiceKind::kReleaseReservation,
    correlation, std::move(request), std::move(callback));
}

AsyncSendResult WorldStateAsyncPort::validate_execution_authority(
  OperationCorrelation correlation,
  std::shared_ptr<ValidateExecutionAuthority::Request> request,
  CompletionCallback<ValidateExecutionAuthority> callback)
{
  return send_request<ValidateExecutionAuthority>(
    validate_execution_authority_client_,
    WorldStateServiceKind::kValidateExecutionAuthority, correlation,
    std::move(request), std::move(callback));
}

bool WorldStateAsyncPort::remove_pending_request(const WorldStateRequestHandle & handle)
{
  if (!valid_correlation(handle.correlation) || handle.request_id <= 0) {
    return false;
  }
  switch (handle.service) {
    case WorldStateServiceKind::kGetSnapshot:
      return get_snapshot_client_->remove_pending_request(handle.request_id);
    case WorldStateServiceKind::kReserveTask:
      return reserve_task_client_->remove_pending_request(handle.request_id);
    case WorldStateServiceKind::kValidateReservation:
      return validate_reservation_client_->remove_pending_request(handle.request_id);
    case WorldStateServiceKind::kReleaseReservation:
      return release_reservation_client_->remove_pending_request(handle.request_id);
    case WorldStateServiceKind::kValidateExecutionAuthority:
      return validate_execution_authority_client_->remove_pending_request(handle.request_id);
  }
  return false;
}

const char * to_string(AsyncSendErrorCode code) noexcept
{
  switch (code) {
    case AsyncSendErrorCode::kNone: return "none";
    case AsyncSendErrorCode::kInvalidArgument: return "invalid_argument";
    case AsyncSendErrorCode::kTransportRejected: return "transport_rejected";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
