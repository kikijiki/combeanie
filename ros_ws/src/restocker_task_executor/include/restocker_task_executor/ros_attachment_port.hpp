// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <restocker_interfaces/srv/acquire_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/commit_reserved_attachment.hpp>
#include <restocker_interfaces/srv/commit_reserved_detachment.hpp>
#include <restocker_interfaces/srv/release_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/set_simulation_attachment.hpp>

#include "restocker_task_executor/attachment_commit_rpc.hpp"
#include "restocker_task_executor/attachment_port.hpp"

namespace restocker_task_executor
{

struct RosAttachmentPortConfig
{
  std::string acquire_lease_service{"/planning_scene_projection/acquire_lease"};
  std::string release_lease_service{"/planning_scene_projection/release_lease"};
  std::string set_attachment_service{"/simulation_attachment/set"};
  std::string attachment_state_topic{"/simulation_attachment/state"};
  std::string commit_attachment_service{"/world_state/commit_attachment"};
  std::string commit_detachment_service{"/world_state/commit_detachment"};
  // Upper bound on one round trip. Every service in this transaction is pollable by operation id,
  // so a lapsed call is replayed rather than escalated.
  std::chrono::milliseconds call_timeout{std::chrono::seconds(2)};
  // Gap between idempotent polls of a pending operation.
  std::chrono::milliseconds poll_period{std::chrono::milliseconds(50)};
  // Bound on releasing the lease once the transaction is otherwise finished.
  std::chrono::milliseconds release_timeout{std::chrono::seconds(5)};
};

// Injectable I/O boundary for the same worker/saga used by the live port. Production binds these
// to its private ROS clients; deterministic tests bind the actual commit adapter to a fake client.
struct RosAttachmentPortOperations
{
  enum class Service : std::uint8_t {kAcquire, kRelease, kPhysical, kAttach, kDetach};
  std::function<AttachmentCommitClock::time_point()> now;
  std::function<bool(AttachmentCommitClock::time_point)> wait_until;
  std::function<bool(Service)> ready;
  std::function<std::optional<restocker_interfaces::srv::AcquirePlanningSceneLease::Response>(
      restocker_interfaces::srv::AcquirePlanningSceneLease::Request::SharedPtr)> acquire;
  std::function<std::optional<restocker_interfaces::srv::ReleasePlanningSceneLease::Response>(
      restocker_interfaces::srv::ReleasePlanningSceneLease::Request::SharedPtr)> release;
  std::function<std::optional<restocker_interfaces::srv::SetSimulationAttachment::Response>(
      restocker_interfaces::srv::SetSimulationAttachment::Request::SharedPtr)> physical;
  std::function<AttachmentCommitCall(
      const AttachmentCommitService::Request::SharedPtr &, AttachmentCommitClock::time_point,
      AttachmentCommitHistory &)> attach;
  std::function<std::optional<restocker_interfaces::srv::CommitReservedDetachment::Response>(
      restocker_interfaces::srv::CommitReservedDetachment::Request::SharedPtr)> detach;
};

// An AttachmentPort over the live lease, simulated-attachment and world-state services.
//
// Every step blocks on a service round trip, so this port owns a private node, a single-threaded
// executor thread for it, and one worker thread that runs the whole saga. That keeps the blocking
// calls off the coordinator's callback groups, the same arrangement MoveItMotionPort uses.
//
// Transaction order, as enforced by the deployed nodes:
//   1. acquire the exclusive planning-scene lease, freezing the projector
//   2. command /simulation_attachment/set with the reservation and lease capabilities
//   3. poll until the boundary reports its terminal phase (its verifier needs consecutive
//      settled ticks, so this is never immediate)
//   4. commit the semantic change to world state
//   5. release the lease, letting the projector re-derive the MoveIt scene from world state
//
// The MoveIt scene is not mutated here. The projector is the sole writer of managed geometry and
// derives attached objects from the world-state snapshot, so step 5 makes the planning scene
// agree; a diff applied from this port would race it.
//
// Two fail-closed rules:
//   - The lease is released only when the outcome proves Gazebo and world state agree. If the
//     physical outcome is unproven, or physical succeeded while the commit did not, the projector
//     stays frozen: unfreezing it would let it project a free-standing collision object into the
//     space the product occupies inside the gripper.
//   - A physical attach that fails to commit is never compensated automatically. The boundary
//     authorizes a detach only while the reservation is at STAGE_ATTACHED, which the failed commit
//     did not reach, so the inverse mutation cannot be authorized. The
//     port reports kUncommitted or kIndeterminate and leaves reconciliation to an operator.
class RosAttachmentPort final : public AttachmentPort
{
public:
  RosAttachmentPort(
    const rclcpp::NodeOptions & options, RosAttachmentPortConfig config = {},
    std::string node_name = "restock_attachment_client");
  RosAttachmentPort(RosAttachmentPortConfig config, RosAttachmentPortOperations operations);
  ~RosAttachmentPort() override;

  RosAttachmentPort(const RosAttachmentPort &) = delete;
  RosAttachmentPort & operator=(const RosAttachmentPort &) = delete;
  RosAttachmentPort(RosAttachmentPort &&) = delete;
  RosAttachmentPort & operator=(RosAttachmentPort &&) = delete;

  [[nodiscard]] bool ready() const override;

  [[nodiscard]] AttachmentSubmitResult submit(
    OperationCorrelation correlation, AttachmentGoal goal, CompletionCallback callback) override;

  void cancel() noexcept override;

  // Stop the worker and drop the private node. Idempotent; the destructor calls it.
  void shutdown() noexcept;

private:
  friend class RosAttachmentPortTestPeer;

  using AcquireLease = restocker_interfaces::srv::AcquirePlanningSceneLease;
  using ReleaseLease = restocker_interfaces::srv::ReleasePlanningSceneLease;
  using SetAttachment = restocker_interfaces::srv::SetSimulationAttachment;
  using CommitAttachment = restocker_interfaces::srv::CommitReservedAttachment;
  using CommitDetachment = restocker_interfaces::srv::CommitReservedDetachment;
  using AttachmentState = restocker_interfaces::msg::SimulationAttachmentState;

  struct PendingGoal
  {
    OperationCorrelation correlation;
    AttachmentGoal goal;
    CompletionCallback callback;
  };

  // Detach keeps its existing stack lifetime. Attach uses the one port-owned record below;
  // unsafe completion retains it until shutdown, without a recovery or persistence API.
  struct Transaction
  {
    std::string operation_id;
    std::string lease_token;
    std::uint64_t world_revision{0};
    bool lease_released{true};
    // Simulation time at which the boundary verified the terminal physical phase. World state
    // needs the release instant to tell a placement from the preceding pre-insert presence; the
    // value must not change between commit re-asks, so it is captured once.
    builtin_interfaces::msg::Time released_at;
    OperationCorrelation correlation;
    CommitAttachment::Request::SharedPtr attachment_request;
    std::optional<AttachmentState> physical_receipt;
    AttachmentCommitHistory commit_history;
    CommitAttachment::Response::SharedPtr historical_receipt;
    bool physical_send_attempted{false};
    bool physical_verified{false};
    bool unsafe_terminal{false};
  };

  void run_worker() noexcept;
  [[nodiscard]] AttachmentCompletion run_transaction(const PendingGoal & pending) noexcept;
  // The saga proper. Allowed to throw; run_transaction is what turns a throw into a fail-closed
  // completion, and what decides the lease disposition from the outcome.
  [[nodiscard]] AttachmentOutcome run_saga(
    const AttachmentGoal & goal, Transaction & transaction, std::string & detail);

  // Each step returns an outcome only when it decides the transaction; std::nullopt means carry on.
  [[nodiscard]] std::optional<AttachmentOutcome> acquire_lease(
    const AttachmentGoal & goal, Transaction & transaction, std::string & detail);
  [[nodiscard]] std::optional<AttachmentOutcome> apply_physical_transition(
    const AttachmentGoal & goal, Transaction & transaction, std::string & detail);
  [[nodiscard]] std::optional<AttachmentOutcome> commit_semantic_state(
    const AttachmentGoal & goal, Transaction & transaction, std::string & detail);
  [[nodiscard]] std::optional<AttachmentOutcome> commit_attachment(
    const AttachmentGoal & goal, Transaction & transaction, std::string & detail);
  void release_lease(Transaction & transaction);

  // Blocking round trip on the private node. std::nullopt on transport failure or timeout, both
  // of which leave the remote outcome unknown and are therefore always retried by operation id.
  template<typename Service>
  [[nodiscard]] std::optional<typename Service::Response> call(
    const typename rclcpp::Client<Service>::SharedPtr & client,
    typename Service::Request::SharedPtr request);

  // Waits out one poll period, returning false when shutdown interrupted the wait.
  [[nodiscard]] bool wait_poll_period();

  [[nodiscard]] bool canceled() const noexcept;
  [[nodiscard]] std::optional<AttachmentState> latest_state(
    const std::string & operation_id) const;
  [[nodiscard]] std::string next_operation_id(
    const AttachmentGoal & goal, OperationCorrelation correlation);

  RosAttachmentPortConfig config_;
  RosAttachmentPortOperations operations_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<AcquireLease>::SharedPtr acquire_lease_client_;
  rclcpp::Client<ReleaseLease>::SharedPtr release_lease_client_;
  rclcpp::Client<SetAttachment>::SharedPtr set_attachment_client_;
  rclcpp::Client<CommitAttachment>::SharedPtr commit_attachment_client_;
  rclcpp::Client<CommitDetachment>::SharedPtr commit_detachment_client_;
  rclcpp::Subscription<AttachmentState>::SharedPtr state_subscription_;

  mutable std::mutex state_mutex_;
  std::optional<AttachmentState> latest_state_;

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::optional<PendingGoal> pending_;
  bool busy_{false};
  // Only the worker writes the record while busy. Publication of unsafe_terminal is synchronized
  // by mutex_ when busy becomes false, before another submit can inspect or replace the record.
  std::optional<Transaction> retained_attachment_;
  std::uint64_t operation_sequence_{0};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> cancel_requested_{false};
  std::thread worker_;
};

}  // namespace restocker_task_executor
