// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/validate_execution_world_authority.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

struct WorldStateServiceNames
{
  std::string get_snapshot{"/world_state/get_snapshot"};
  std::string reserve_task{"/world_state/reserve_task"};
  std::string validate_reservation{"/world_state/validate_reservation"};
  std::string release_reservation{"/world_state/release_reservation"};
  std::string validate_execution_authority{"/world_state/validate_execution_authority"};
};

enum class WorldStateServiceKind : std::uint8_t
{
  kGetSnapshot,
  kReserveTask,
  kValidateReservation,
  kReleaseReservation,
  kValidateExecutionAuthority,
};

struct WorldStateRequestHandle
{
  OperationCorrelation correlation;
  WorldStateServiceKind service{WorldStateServiceKind::kGetSnapshot};
  std::int64_t request_id{0};
};

enum class AsyncSendErrorCode : std::uint8_t
{
  kNone,
  kInvalidArgument,
  kTransportRejected,
};

struct AsyncSendResult
{
  AsyncSendResult(
    AsyncSendErrorCode result_error, std::optional<WorldStateRequestHandle> result_handle,
    std::string result_detail)
  : error(result_error), handle(std::move(result_handle)), detail(std::move(result_detail))
  {
    const bool accepted = error == AsyncSendErrorCode::kNone;
    if (accepted != handle.has_value() || (accepted && !detail.empty()) ||
      (!accepted && detail.empty()))
    {
      throw std::invalid_argument(
              "async send result must contain exactly one accepted handle or diagnosed error");
    }
  }

  const AsyncSendErrorCode error;
  const std::optional<WorldStateRequestHandle> handle;
  const std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return error == AsyncSendErrorCode::kNone && handle.has_value();
  }
};

template<typename Response>
struct AsyncServiceCompletion
{
  OperationCorrelation correlation;
  std::shared_ptr<const Response> response;
  std::string transport_error;

  [[nodiscard]] bool has_response() const noexcept
  {
    return response != nullptr && transport_error.empty();
  }
};

struct WorldStateServiceReadiness
{
  bool get_snapshot{false};
  bool reserve_task{false};
  bool validate_reservation{false};
  bool release_reservation{false};
  bool validate_execution_authority{false};

  [[nodiscard]] bool all_available() const noexcept
  {
    return get_snapshot && reserve_task && validate_reservation && release_reservation &&
           validate_execution_authority;
  }
};

class WorldStateCoordinatorPort
{
public:
  using GetSnapshot = restocker_interfaces::srv::GetWorldState;
  using ReserveTask = restocker_interfaces::srv::ReserveTask;
  using ValidateReservation = restocker_interfaces::srv::ValidateTaskReservation;
  using ReleaseReservation = restocker_interfaces::srv::ReleaseTaskReservation;
  using ValidateExecutionAuthority =
    restocker_interfaces::srv::ValidateExecutionWorldAuthority;

  template<typename Service>
  using CompletionCallback =
    std::function<void (AsyncServiceCompletion<typename Service::Response>)>;

  virtual ~WorldStateCoordinatorPort() = default;

  [[nodiscard]] virtual WorldStateServiceReadiness readiness() const = 0;

  [[nodiscard]] virtual AsyncSendResult get_snapshot(
    OperationCorrelation correlation, std::shared_ptr<GetSnapshot::Request> request,
    CompletionCallback<GetSnapshot> callback) = 0;
  [[nodiscard]] virtual AsyncSendResult reserve_task(
    OperationCorrelation correlation, std::shared_ptr<ReserveTask::Request> request,
    CompletionCallback<ReserveTask> callback) = 0;
  [[nodiscard]] virtual AsyncSendResult validate_reservation(
    OperationCorrelation correlation, std::shared_ptr<ValidateReservation::Request> request,
    CompletionCallback<ValidateReservation> callback) = 0;
  [[nodiscard]] virtual AsyncSendResult release_reservation(
    OperationCorrelation correlation, std::shared_ptr<ReleaseReservation::Request> request,
    CompletionCallback<ReleaseReservation> callback) = 0;
  [[nodiscard]] virtual AsyncSendResult validate_execution_authority(
    OperationCorrelation correlation,
    std::shared_ptr<ValidateExecutionAuthority::Request> request,
    CompletionCallback<ValidateExecutionAuthority> callback) = 0;

  virtual bool remove_pending_request(const WorldStateRequestHandle & handle) = 0;
};

class WorldStateAsyncPort final : public WorldStateCoordinatorPort
{
public:
  using GetSnapshot = WorldStateCoordinatorPort::GetSnapshot;
  using ReserveTask = WorldStateCoordinatorPort::ReserveTask;
  using ValidateReservation = WorldStateCoordinatorPort::ValidateReservation;
  using ReleaseReservation = WorldStateCoordinatorPort::ReleaseReservation;
  using ValidateExecutionAuthority = WorldStateCoordinatorPort::ValidateExecutionAuthority;

  template<typename Service>
  using CompletionCallback = WorldStateCoordinatorPort::CompletionCallback<Service>;

  WorldStateAsyncPort(
    rclcpp::Node & node, rclcpp::CallbackGroup::SharedPtr completion_group,
    WorldStateServiceNames names = {});

  [[nodiscard]] WorldStateServiceReadiness readiness() const override;

  [[nodiscard]] AsyncSendResult get_snapshot(
    OperationCorrelation correlation, std::shared_ptr<GetSnapshot::Request> request,
    CompletionCallback<GetSnapshot> callback) override;
  [[nodiscard]] AsyncSendResult reserve_task(
    OperationCorrelation correlation, std::shared_ptr<ReserveTask::Request> request,
    CompletionCallback<ReserveTask> callback) override;
  [[nodiscard]] AsyncSendResult validate_reservation(
    OperationCorrelation correlation, std::shared_ptr<ValidateReservation::Request> request,
    CompletionCallback<ValidateReservation> callback) override;
  [[nodiscard]] AsyncSendResult release_reservation(
    OperationCorrelation correlation, std::shared_ptr<ReleaseReservation::Request> request,
    CompletionCallback<ReleaseReservation> callback) override;
  [[nodiscard]] AsyncSendResult validate_execution_authority(
    OperationCorrelation correlation,
    std::shared_ptr<ValidateExecutionAuthority::Request> request,
    CompletionCallback<ValidateExecutionAuthority> callback) override;

  // Removing a pending request is suitable only after the caller has accounted for its
  // side-effect semantics. In particular, mutation timeouts require reconciliation first.
  [[nodiscard]] bool remove_pending_request(const WorldStateRequestHandle & handle) override;

private:
  // rclcpp nodes retain callback groups weakly. The port must keep this group alive for as long
  // as any client response can arrive.
  rclcpp::CallbackGroup::SharedPtr completion_group_;
  rclcpp::Client<GetSnapshot>::SharedPtr get_snapshot_client_;
  rclcpp::Client<ReserveTask>::SharedPtr reserve_task_client_;
  rclcpp::Client<ValidateReservation>::SharedPtr validate_reservation_client_;
  rclcpp::Client<ReleaseReservation>::SharedPtr release_reservation_client_;
  rclcpp::Client<ValidateExecutionAuthority>::SharedPtr validate_execution_authority_client_;
};

[[nodiscard]] const char * to_string(AsyncSendErrorCode code) noexcept;

}  // namespace restocker_task_executor
