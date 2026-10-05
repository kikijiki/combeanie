// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <utility>

#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>

#include "restocker_task_executor/action_result_publisher.hpp"
#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"
#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/coordinator_handoff_policy.hpp"
#include "restocker_task_executor/coordinator_handoff_reducer.hpp"
#include "restocker_task_executor/coordinator_operation_types.hpp"
#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"
#include "restocker_task_executor/coordinator_process_control.hpp"
#include "restocker_task_executor/coordinator_startup.hpp"

namespace restocker_task_executor
{

struct RestockActionCoordinatorNodeDependencies
{
  RestockActionCoordinatorNodeDependencies(
    CoordinatorSteadyNow steady_now_value = {},
    std::function<void()> after_pending_goal_reserved_value = {},
    std::function<void()> after_terminal_delivery_reserved_value = {},
    std::function<void()> before_active_fault_route_value = {},
    std::shared_ptr<ActionResultPublisher> action_result_publisher_value =
    std::make_shared<RosActionResultPublisher>(),
    std::function<void()> before_accepted_goal_adoption_value = {},
    std::function<void()> before_pending_handoff_value = {},
    std::function<void()> after_callback_failure_value = {})
  : steady_now(std::move(steady_now_value)),
    after_pending_goal_reserved(std::move(after_pending_goal_reserved_value)),
    after_terminal_delivery_reserved(std::move(after_terminal_delivery_reserved_value)),
    before_active_fault_route(std::move(before_active_fault_route_value)),
    action_result_publisher(std::move(action_result_publisher_value)),
    before_accepted_goal_adoption(std::move(before_accepted_goal_adoption_value)),
    before_pending_handoff(std::move(before_pending_handoff_value)),
    after_callback_failure(std::move(after_callback_failure_value))
  {
  }

  CoordinatorSteadyNow steady_now;
  std::function<void()> after_pending_goal_reserved;
  std::function<void()> after_terminal_delivery_reserved;
  std::function<void()> before_active_fault_route;
  std::shared_ptr<ActionResultPublisher> action_result_publisher;
  // Empty in production. These seams scope failure injection to the coordinator callbacks,
  // after middleware has delivered the accepted handle and before it resumes executor work.
  std::function<void()> before_accepted_goal_adoption;
  std::function<void()> before_pending_handoff;
  std::function<void()> after_callback_failure;
};

struct CoordinatorSteadyClockFailureSnapshot
{
  bool provider_failed{false};
  bool handled{false};
  bool authority_secured{false};
  bool first_observed_asynchronously{false};
};

struct CoordinatorGenerationAuthoritySnapshot
{
  std::size_t configured_deposit_capacity{0U};
  bool pending_binding{false};
  bool active_binding{false};
  bool accepted_handle_fail_stopped{false};
  std::size_t orphaned_accepted_handle_count{0U};
  std::optional<std::uintptr_t> pending_accepted_handle_identity;
  std::optional<std::uintptr_t> active_accepted_handle_identity;
  std::optional<std::uintptr_t> pending_epoch_identity;
  std::optional<std::uintptr_t> active_epoch_identity;
  std::optional<CoordinatorActiveFaultEpochSnapshot> pending_epoch;
  std::optional<CoordinatorGenerationQuiescenceSnapshot> pending_quiescence;
  std::optional<CoordinatorActiveFaultEpochSnapshot> active_epoch;
  std::optional<CoordinatorGenerationQuiescenceSnapshot> active_quiescence;
  CoordinatorPumpLeaseGateSnapshot pump_lease;
  std::optional<PendingAcceptedHandoffPhase> pending_handoff_phase;
  bool pending_route_token_live{false};
  bool pending_handoff_work_token_live{false};
  bool pending_retained_control_occupied{false};
  bool pending_deferred_control_occupied{false};
  bool diagnostic_caches_initialized{false};
  bool diagnostic_caches_fresh{false};
  CachedCoordinatorInboxSnapshot cached_inbox;
  CachedGoalAdmissionSnapshot cached_admission;
  CachedRestockCoordinatorDriverSnapshot cached_driver;
  // Card 082, §6: the allocation-free receipt. Half A of a callback failure is latched before
  // anything on that path touches the heap, so it is visible under the very condition that
  // caused the failure. `callback_failure_detail` points at a string literal with static
  // storage duration — it is never copied, and this struct never owns it.
  bool callback_failure_latched{false};
  const char * callback_failure_detail{nullptr};
};

// ROS action and transport adapter for the deterministic pre-motion coordinator driver.
class RestockActionCoordinatorNode final : public rclcpp::Node
{
public:
  explicit RestockActionCoordinatorNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  RestockActionCoordinatorNode(
    const rclcpp::NodeOptions & options,
    RestockActionCoordinatorNodeDependencies dependencies);
  ~RestockActionCoordinatorNode() override;

  RestockActionCoordinatorNode(const RestockActionCoordinatorNode &) = delete;
  RestockActionCoordinatorNode & operator=(const RestockActionCoordinatorNode &) = delete;

  [[nodiscard]] std::size_t executor_threads() const noexcept;
  void request_shutdown();
  [[nodiscard]] CoordinatorShutdownStatus shutdown_status() const noexcept;
  [[nodiscard]] CoordinatorShutdownSnapshot shutdown_snapshot() const noexcept;
  [[nodiscard]] CoordinatorStartupSnapshot startup_snapshot() const;
  [[nodiscard]] CoordinatorSteadyClockFailureSnapshot steady_clock_failure_snapshot() const;
  // In-process diagnostic view for tests and health tooling. Binding-owned scalar snapshots are
  // copied under the binding mutex; shared quiescence objects are pinned there and sampled after
  // unlock. Values are observational and grant no execution or deposit capability.
  [[nodiscard]] CoordinatorGenerationAuthoritySnapshot generation_authority_snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace restocker_task_executor
