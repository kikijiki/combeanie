// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_operation_status.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>

#include "restocker_task_executor/ros_attachment_port.hpp"

namespace restocker_task_executor
{

// Inspects the retained record only at a controlled worker seam or after completion publication.
class RosAttachmentPortTestPeer
{
public:
  static bool wait_for_shutdown_request(RosAttachmentPort & port)
  {
    std::unique_lock lock(port.mutex_);
    return port.work_available_.wait_for(
      lock, std::chrono::seconds(2), [&]() {
        return port.stopping_.load(std::memory_order_acquire);
      });
  }

  static auto retained(const RosAttachmentPort & port)
  {
    std::scoped_lock lock(port.mutex_);
    return port.retained_attachment_;
  }
};

namespace attachment_test
{
using Clock = AttachmentCommitClock;
using Commit = AttachmentCommitService;
using Status = restocker_interfaces::msg::WorldStateOperationStatus;
using Physical = restocker_interfaces::srv::SetSimulationAttachment;
using Lease = restocker_interfaces::srv::AcquirePlanningSceneLease;
using Release = restocker_interfaces::srv::ReleasePlanningSceneLease;
using Detach = restocker_interfaces::srv::CommitReservedDetachment;
using std::chrono_literals::operator""ms;
using std::chrono_literals::operator""s;

enum class Behavior {kReply, kTimeout, kSendThenThrow, kUnready, kGetThrows};

struct Step
{
  Behavior behavior{Behavior::kReply};
  std::uint16_t code{Status::ATTACHMENT_CLOCK_NOT_READY};
  bool receipt{false};
  Clock::duration ready_delay{0ms};
  Clock::duration send_delay{0ms};
  Clock::duration get_delay{0ms};
};

// Fake only the I/O boundaries, not the saga, admission, receipt storage or send classification.
// The same call_attachment_commit template wraps this client and the deployed rclcpp::Client.
class Harness
{
public:
  Harness()
  {
    config.call_timeout = 20ms;
    config.poll_period = 10ms;
    config.release_timeout = 30ms;
  }

  struct Future
  {
    Harness & owner;
    Step step;
    Commit::Response::SharedPtr response;

    std::future_status wait_until(Clock::time_point deadline)
    {
      owner.wait_deadlines.push_back(deadline);
      if (step.behavior == Behavior::kTimeout) {
        owner.now = deadline;
        return std::future_status::timeout;
      }
      return std::future_status::ready;
    }

    Commit::Response::SharedPtr get()
    {
      owner.now += step.get_delay;
      if (step.behavior == Behavior::kGetThrows) {
        throw std::runtime_error("matched future could not be read");
      }
      return response;
    }
  };

  bool service_is_ready()
  {
    if (steps.empty()) {
      return false;
    }
    now += steps.front().ready_delay;
    steps.front().ready_delay = 0ms;
    if (steps.front().behavior == Behavior::kUnready) {
      steps.pop_front();
      return false;
    }
    return true;
  }

  Future async_send_request(const Commit::Request::SharedPtr & request)
  {
    auto step = steps.front();
    steps.pop_front();
    // Observable physical send BEFORE the injected exception, like rcl_send_request followed
    // by an allocating pending_requests_.try_emplace in the pinned rclcpp implementation.
    requests.push_back(request);
    now += step.send_delay;
    if (on_send) {
      on_send();
    }
    if (step.behavior == Behavior::kSendThenThrow) {
      throw std::runtime_error("send happened before future registration failed");
    }
    auto response = std::make_shared<Commit::Response>();
    response->status.code = step.code;
    response->has_reservation = step.receipt;
    if (step.receipt) {
      response->world_revision = 73;
      response->reservation.object_id = object_id;
      response->reservation.stage = restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED;
      response->reservation.reservation_id = 31;
      response->reservation.revision = 73;
    }
    return {*this, step, std::move(response)};
  }

  void remove_pending_request(const Future &) {++removed;}

  RosAttachmentPortOperations operations()
  {
    RosAttachmentPortOperations ops;
    ops.now = [this]() {return now;};
    ops.wait_until = [this](Clock::time_point deadline) {
      polls.push_back(deadline);
      now = deadline;
      if (on_poll) {
        on_poll();
      }
      return poll_continues;
    };
    ops.ready = [](auto) {return true;};
    ops.acquire = [this](Lease::Request::SharedPtr request) {
      ++acquired;
      acquire_id = request->operation_id;
      Lease::Response response;
      response.status.code =
        restocker_interfaces::msg::PlanningSceneLeaseOperationStatus::GRANTED;
      response.has_lease = true;
      response.token = "retained-scene-capability";
      return std::optional{std::move(response)};
    };
    ops.physical = [this](Physical::Request::SharedPtr request) {
      ++physical_sends;
      physical_request = std::move(request);
      object_id = physical_request->object_id;
      if (on_physical) {
        on_physical();
      }
      const bool attaching = physical_request->command == Physical::Request::COMMAND_ATTACH;
      Physical::Response response;
      response.status.code = attaching ?
        restocker_interfaces::msg::SimulationAttachmentOperationStatus::ATTACHED :
        restocker_interfaces::msg::SimulationAttachmentOperationStatus::DETACHED;
      response.has_state = true;
      response.state.phase = attaching ?
        restocker_interfaces::msg::SimulationAttachmentState::PHASE_ATTACHED :
        restocker_interfaces::msg::SimulationAttachmentState::PHASE_DETACHED;
      response.state.operation_id = physical_request->operation_id;
      response.state.object_id = object_id;
      response.state.simulator_epoch = "original-epoch";
      response.state.sequence = 19;
      response.state.simulator_iteration = 211;
      response.state.observed_at.sec = 9;
      response.state.observed_at.nanosec = 650000000;
      response.state.has_fidelity = attaching;
      response.state.fidelity_translation_residual_m = attaching ? 0.001 : 0.0;
      return std::optional{std::move(response)};
    };
    ops.attach = [this](const Commit::Request::SharedPtr & request,
      Clock::time_point deadline, AttachmentCommitHistory & history) {
      semantic_deadlines.push_back(deadline);
      return call_attachment_commit(
        *this, request, deadline, config.call_timeout,
        history, [this]() {return now;});
    };
    ops.release = [this](Release::Request::SharedPtr request) {
      ++released;
      release_request = std::move(request);
      if (on_release) {
        on_release();
      }
      Release::Response response;
      response.status.code = release_status;
      return std::optional{std::move(response)};
    };
    ops.detach = [this](Detach::Request::SharedPtr request) {
      ++detach_sends;
      detach_requests.push_back(std::move(request));
      if (on_detach) {
        on_detach();
      }
      if (detach_codes.empty()) {
        return std::optional<Detach::Response>{};
      }
      Detach::Response response;
      response.status.code = detach_codes.front();
      detach_codes.pop_front();
      if (response.status.code == Status::OK) {
        response.world_revision = 83;
        response.has_reservation = true;
        response.reservation.object_id = object_id;
        response.reservation.stage = restocker_interfaces::msg::TaskReservation::STAGE_DETACHED;
      } else if (response.status.code == Status::PREDICATE_FAILED) {
        response.status.detail = "placement observation pending";
      }
      return std::optional{std::move(response)};
    };
    return ops;
  }

  AttachmentCompletion run(
    RosAttachmentPort & port, AttachmentGoal goal, OperationCorrelation correlation = {1, 1})
  {
    auto promise = std::make_shared<std::promise<AttachmentCompletion>>();
    auto result = promise->get_future();
    const auto submitted = port.submit(
      correlation, std::move(goal), [this, promise](auto completion) {
        ++completions;
        promise->set_value(std::move(completion));
      });
    if (!submitted) {
      throw std::runtime_error("harness submission refused: " + submitted.detail);
    }
    if (result.wait_for(2s) != std::future_status::ready) {
      throw std::runtime_error("deterministic saga did not complete");
    }
    return result.get();
  }

  RosAttachmentPortConfig config;
  Clock::time_point now{};
  std::deque<Step> steps;
  std::deque<std::uint16_t> detach_codes;
  std::vector<Commit::Request::SharedPtr> requests;
  std::vector<Detach::Request::SharedPtr> detach_requests;
  std::vector<Clock::time_point> wait_deadlines;
  std::vector<Clock::time_point> semantic_deadlines;
  std::vector<Clock::time_point> polls;
  Physical::Request::SharedPtr physical_request;
  Release::Request::SharedPtr release_request;
  std::string acquire_id;
  std::uint64_t object_id{7};
  std::uint16_t release_status{
    restocker_interfaces::msg::PlanningSceneLeaseOperationStatus::RELEASE_ACCEPTED};
  std::atomic<std::uint64_t> completions{0};
  int acquired{0};
  int physical_sends{0};
  int released{0};
  int detach_sends{0};
  int removed{0};
  bool poll_continues{true};
  std::function<void()> on_physical;
  std::function<void()> on_send;
  std::function<void()> on_poll;
  std::function<void()> on_detach;
  std::function<void()> on_release;
};

inline AttachmentGoal goal()
{
  AttachmentGoal result;
  result.object_id = 7;
  result.reservation_token = "reservation-capability";
  result.grasp_center_to_child.translation().z() = 0.03;
  result.commit_timeout = 120ms;
  return result;
}

}  // namespace attachment_test
}  // namespace restocker_task_executor
