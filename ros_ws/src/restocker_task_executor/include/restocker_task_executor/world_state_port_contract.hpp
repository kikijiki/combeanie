// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/validate_execution_world_authority.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_task_executor/task_selection.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_task_executor
{

enum class WorldStatePortErrorCode : std::uint8_t
{
  kInvalidInput,
  kRemoteRejected,
  kMalformedResponse,
  kIdentityMismatch,
};

struct WorldStatePortError
{
  WorldStatePortErrorCode code{WorldStatePortErrorCode::kInvalidInput};
  std::uint16_t remote_status{0};
  std::string detail;
};

template<typename T>
class [[nodiscard]] WorldStatePortResult
{
public:
  [[nodiscard]] static WorldStatePortResult success(T value)
  {
    return WorldStatePortResult(std::move(value));
  }

  [[nodiscard]] static WorldStatePortResult failure(WorldStatePortError error)
  {
    return WorldStatePortResult(std::move(error));
  }

  [[nodiscard]] bool has_value() const noexcept {return std::holds_alternative<T>(storage_);}
  [[nodiscard]] explicit operator bool() const noexcept {return has_value();}
  [[nodiscard]] const T & value() const {return std::get<T>(storage_);}
  [[nodiscard]] T & value() {return std::get<T>(storage_);}
  [[nodiscard]] const WorldStatePortError & error() const
  {
    return std::get<WorldStatePortError>(storage_);
  }

private:
  explicit WorldStatePortResult(T value)
  : storage_(std::move(value)) {}

  explicit WorldStatePortResult(WorldStatePortError error)
  : storage_(std::move(error)) {}

  std::variant<T, WorldStatePortError> storage_;
};

struct TaskReservationCapability
{
  std::string token;
  restocker_world_state::TaskReservation reservation;
  restocker_world_state::Revision world_revision{0};
};

// Semantic binding for correlating an accepted read with the exact owner-retained capability.
// This value is evidence lineage, not an authorization token, and must never substitute for the
// token when calling the world-state service.
[[nodiscard]] std::optional<std::uint64_t>
fingerprint_task_reservation_capability(const TaskReservationCapability & capability);

struct TaskReservationValidationProof
{
  restocker_world_state::Revision world_revision{0};
  restocker_world_state::TaskReservation reservation;
};

struct TaskReservationReleaseAcknowledgement
{
  restocker_world_state::Revision world_revision{0};
};

struct TaskReservationReleaseProof
{
  restocker_world_state::Revision snapshot_revision{0};
};

[[nodiscard]] WorldStatePortResult<restocker_interfaces::srv::ReserveTask::Request>
make_reserve_task_request(
  const restocker_world_state::WorldStateSnapshot & snapshot,
  const SelectedTaskPair & selection, const std::string & request_id);

[[nodiscard]] WorldStatePortResult<TaskReservationCapability> validate_reserve_task_response(
  const restocker_interfaces::srv::ReserveTask::Request & request,
  const restocker_world_state::WorldStateSnapshot & selected_snapshot,
  const restocker_interfaces::srv::ReserveTask::Response & response);

[[nodiscard]] WorldStatePortResult<restocker_interfaces::srv::ValidateTaskReservation::Request>
make_validate_task_reservation_request(const TaskReservationCapability & capability);

[[nodiscard]] WorldStatePortResult<TaskReservationValidationProof>
validate_task_reservation_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ValidateTaskReservation::Response & response);

[[nodiscard]] WorldStatePortResult<
  restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request>
make_validate_execution_world_authority_request(const TaskReservationCapability & capability);

[[nodiscard]] WorldStatePortResult<restocker_world_state::ExecutionWorldAuthorityProof>
validate_execution_world_authority_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response & response,
  const std::string & expected_planning_frame);

[[nodiscard]] WorldStatePortResult<restocker_interfaces::srv::ReleaseTaskReservation::Request>
make_release_task_reservation_request(
  const TaskReservationCapability & capability,
  const std::string & operation_id,
  restocker_world_state::ReservationOutcome outcome,
  restocker_world_state::TaskPhase terminal_task_phase,
  restocker_world_state::FaultState terminal_fault_state);

[[nodiscard]] WorldStatePortResult<TaskReservationReleaseAcknowledgement>
validate_release_task_reservation_response(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ReleaseTaskReservation::Response & response);

[[nodiscard]] WorldStatePortResult<TaskReservationReleaseProof> validate_released_snapshot(
  const TaskReservationCapability & capability,
  const restocker_interfaces::srv::ReleaseTaskReservation::Request & request,
  const std::optional<TaskReservationReleaseAcknowledgement> & acknowledgement,
  const restocker_world_state::WorldStateSnapshot & snapshot);

[[nodiscard]] std::string to_string(WorldStatePortErrorCode code);

}  // namespace restocker_task_executor
