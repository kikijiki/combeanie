// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/ros_attachment_port.hpp"

#include <algorithm>
#include <exception>
#include <future>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include <geometry_msgs/msg/pose.hpp>
#include <restocker_interfaces/msg/planning_scene_lease_operation_status.hpp>
#include <restocker_interfaces/msg/simulation_attachment_operation_status.hpp>
#include <restocker_interfaces/msg/world_state_operation_status.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace restocker_task_executor
{
namespace
{

using LeaseStatus = restocker_interfaces::msg::PlanningSceneLeaseOperationStatus;
using PhysicalStatus = restocker_interfaces::msg::SimulationAttachmentOperationStatus;
using WorldStatus = restocker_interfaces::msg::WorldStateOperationStatus;
using AttachmentStateMsg = restocker_interfaces::msg::SimulationAttachmentState;

using Clock = std::chrono::steady_clock;

// Boundary codes meaning "the request was refused". Trusted to mean no mutation only when the
// boundary's phase evidence agrees, since the adapter re-validates capabilities after mutating too.
[[nodiscard]] bool physical_refusal_code(std::uint16_t code) noexcept
{
  switch (code) {
    case PhysicalStatus::INVALID_ARGUMENT:
    case PhysicalStatus::AUTHORIZATION_FAILED:
    case PhysicalStatus::CONFLICT:
    case PhysicalStatus::TOKEN_MISMATCH:
    case PhysicalStatus::IDEMPOTENCY_CONFLICT:
    case PhysicalStatus::RESOURCE_EXHAUSTED:
    case PhysicalStatus::OBJECT_NOT_FOUND:
    case PhysicalStatus::ENTITY_AMBIGUOUS:
    case PhysicalStatus::GRIPPER_NOT_READY:
    case PhysicalStatus::OUT_OF_TOLERANCE:
    case PhysicalStatus::STATE_MISMATCH:
    case PhysicalStatus::OPERATION_NOT_FOUND:
      return true;
    default:
      return false;
  }
}

// World-state codes that prove the reservation was left unchanged. Any other code after a
// physical mutation is an unknown commit.
[[nodiscard]] bool commit_refusal_code(std::uint16_t code) noexcept
{
  switch (code) {
    case WorldStatus::INVALID_ARGUMENT:
    case WorldStatus::NOT_FOUND:
    case WorldStatus::REVISION_CONFLICT:
    case WorldStatus::RESERVATION_CONFLICT:
    case WorldStatus::TOKEN_MISMATCH:
    case WorldStatus::PREDICATE_FAILED:
    case WorldStatus::INVALID_TRANSITION:
    case WorldStatus::RESOURCE_EXHAUSTED:
      return true;
    default:
      return false;
  }
}

// A placement commit is the one refusal a completed physical transition can outrun. World state
// accepts a placement only when destination evidence proves the landing by the configured proof
// — column growth of one pitch less tolerance by default, post-release target identity when
// growth is disabled — and was recorded after the boundary verified the release. Ground truth
// reports every product pose, attached or not, so identity evidence cannot qualify before the
// detach either; qualifying evidence therefore cannot exist before the detach. The first
// PREDICATE_FAILED means the evidence has not caught up, not that the worlds disagree, so only
// this combination is re-asked. If it never settles, the transaction deadline ends it as an
// uncommitted physical mutation. Every other refusal is terminal on the first reply.
[[nodiscard]] bool placement_evidence_may_still_settle(
  const AttachmentGoal & goal, std::uint16_t code) noexcept
{
  return goal.direction == AttachmentDirection::kDetach &&
         goal.detach_disposition == DetachDisposition::kPlaceInReservedDestination &&
         code == WorldStatus::PREDICATE_FAILED;
}

[[nodiscard]] std::uint16_t applied_code(AttachmentDirection direction) noexcept
{
  return direction == AttachmentDirection::kAttach ? PhysicalStatus::ATTACHED :
         PhysicalStatus::DETACHED;
}

[[nodiscard]] std::uint8_t applied_phase(AttachmentDirection direction) noexcept
{
  return direction == AttachmentDirection::kAttach ? AttachmentStateMsg::PHASE_ATTACHED :
         AttachmentStateMsg::PHASE_DETACHED;
}

[[nodiscard]] std::uint8_t untouched_phase(AttachmentDirection direction) noexcept
{
  return direction == AttachmentDirection::kAttach ? AttachmentStateMsg::PHASE_DETACHED :
         AttachmentStateMsg::PHASE_ATTACHED;
}

[[nodiscard]] std::string describe(const std::string & prefix, const std::string & remote_detail)
{
  return remote_detail.empty() ? prefix : prefix + ": " + remote_detail;
}

}  // namespace

RosAttachmentPort::RosAttachmentPort(
  const rclcpp::NodeOptions & options, RosAttachmentPortConfig config, std::string node_name)
: config_(std::move(config)),
  node_(std::make_shared<rclcpp::Node>(std::move(node_name), options)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  acquire_lease_client_ = node_->create_client<AcquireLease>(
    config_.acquire_lease_service, rclcpp::ServicesQoS(), callback_group_);
  release_lease_client_ = node_->create_client<ReleaseLease>(
    config_.release_lease_service, rclcpp::ServicesQoS(), callback_group_);
  set_attachment_client_ = node_->create_client<SetAttachment>(
    config_.set_attachment_service, rclcpp::ServicesQoS(), callback_group_);
  commit_attachment_client_ = node_->create_client<CommitAttachment>(
    config_.commit_attachment_service, rclcpp::ServicesQoS(), callback_group_);
  commit_detachment_client_ = node_->create_client<CommitDetachment>(
    config_.commit_detachment_service, rclcpp::ServicesQoS(), callback_group_);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  // The adapter latches its state; transient-local yields the current phase immediately.
  state_subscription_ = node_->create_subscription<AttachmentState>(
    config_.attachment_state_topic, rclcpp::QoS(1).reliable().transient_local(),
    [this](AttachmentState::SharedPtr message) {
      std::scoped_lock lock(state_mutex_);
      latest_state_ = std::move(*message);
    },
    subscription_options);

  operations_.now = []() {return Clock::now();};
  operations_.wait_until = [this](Clock::time_point deadline) {
    std::unique_lock lock(mutex_);
    return !work_available_.wait_until(
      lock, deadline, [this]() {return stopping_.load(std::memory_order_acquire);});
  };
  operations_.ready = [this](RosAttachmentPortOperations::Service service) {
    switch (service) {
      case RosAttachmentPortOperations::Service::kAcquire:
        return acquire_lease_client_->service_is_ready();
      case RosAttachmentPortOperations::Service::kRelease:
        return release_lease_client_->service_is_ready();
      case RosAttachmentPortOperations::Service::kPhysical:
        return set_attachment_client_->service_is_ready();
      case RosAttachmentPortOperations::Service::kAttach:
        return commit_attachment_client_->service_is_ready();
      case RosAttachmentPortOperations::Service::kDetach:
        return commit_detachment_client_->service_is_ready();
    }
    return false;
  };
  operations_.acquire = [this](AcquireLease::Request::SharedPtr request) {
    return call<AcquireLease>(acquire_lease_client_, std::move(request));
  };
  operations_.release = [this](ReleaseLease::Request::SharedPtr request) {
    return call<ReleaseLease>(release_lease_client_, std::move(request));
  };
  operations_.physical = [this](SetAttachment::Request::SharedPtr request) {
    return call<SetAttachment>(set_attachment_client_, std::move(request));
  };
  operations_.detach = [this](CommitDetachment::Request::SharedPtr request) {
    return call<CommitDetachment>(commit_detachment_client_, std::move(request));
  };
  operations_.attach = [this](const CommitAttachment::Request::SharedPtr & request,
    Clock::time_point deadline, AttachmentCommitHistory & history) {
    return call_attachment_commit(
      *commit_attachment_client_, request, deadline, config_.call_timeout,
      history, operations_.now);
  };

  executor_->add_node(node_);
  executor_thread_ = std::thread(
    [this]() noexcept {
      try {
        executor_->spin();
      } catch (...) {
        // Without the private executor, later transactions report kUnavailable before commanding.
      }
    });
  worker_ = std::thread([this]() noexcept {run_worker();});
}

RosAttachmentPort::RosAttachmentPort(
  RosAttachmentPortConfig config, RosAttachmentPortOperations operations)
: config_(std::move(config)), operations_(std::move(operations))
{
  if (!operations_.now || !operations_.wait_until || !operations_.ready ||
    !operations_.acquire || !operations_.release || !operations_.physical ||
    !operations_.attach || !operations_.detach)
  {
    throw std::invalid_argument("attachment port requires every I/O operation");
  }
  worker_ = std::thread([this]() noexcept {run_worker();});
}

RosAttachmentPort::~RosAttachmentPort()
{
  shutdown();
}

void RosAttachmentPort::shutdown() noexcept
{
  if (stopping_.exchange(true)) {
    return;
  }
  cancel();
  work_available_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  try {
    if (executor_) {
      executor_->cancel();
    }
  } catch (...) {
    // Teardown: joining below is what matters.
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  try {
    if (executor_) {
      executor_->remove_node(node_);
    }
  } catch (...) {
    // Removal is best-effort during teardown.
  }
}

bool RosAttachmentPort::ready() const
{
  if (stopping_.load(std::memory_order_acquire)) {
    return false;
  }
  // /simulation_attachment/set appears only after the adapter reconciles the boundary against
  // world state, so it is the strongest readiness signal.
  using Service = RosAttachmentPortOperations::Service;
  return operations_.ready(Service::kAcquire) && operations_.ready(Service::kRelease) &&
         operations_.ready(Service::kPhysical) && operations_.ready(Service::kAttach) &&
         operations_.ready(Service::kDetach);
}

AttachmentSubmitResult RosAttachmentPort::submit(
  OperationCorrelation correlation, AttachmentGoal goal, CompletionCallback callback)
{
  if (!callback) {
    return {AttachmentSubmitStatus::kInvalidRequest,
      "attachment submission requires a completion callback"};
  }
  if (!valid_attachment_goal(goal)) {
    return {AttachmentSubmitStatus::kInvalidRequest,
      "attachment goal lacks an object, a reservation capability, usable timeouts, or a rigid "
      "expected grasp pose"};
  }
  if (stopping_.load(std::memory_order_acquire)) {
    return {AttachmentSubmitStatus::kUnavailable, "attachment port is shutting down"};
  }

  {
    std::scoped_lock lock(mutex_);
    if (pending_ || busy_) {
      return {AttachmentSubmitStatus::kBusy,
        "attachment port already owns an outstanding transaction"};
    }
    if (retained_attachment_ && retained_attachment_->unsafe_terminal) {
      return {AttachmentSubmitStatus::kUnavailable,
        "attachment port retains an unresolved physical/semantic obligation"};
    }
    cancel_requested_.store(false, std::memory_order_release);
    pending_.emplace(PendingGoal{correlation, std::move(goal), std::move(callback)});
  }
  work_available_.notify_one();
  return {AttachmentSubmitStatus::kAccepted, {}};
}

void RosAttachmentPort::cancel() noexcept
{
  cancel_requested_.store(true, std::memory_order_release);
  work_available_.notify_all();
}

bool RosAttachmentPort::canceled() const noexcept
{
  return cancel_requested_.load(std::memory_order_acquire) ||
         stopping_.load(std::memory_order_acquire);
}

void RosAttachmentPort::run_worker() noexcept
{
  for (;; ) {
    PendingGoal pending;
    {
      std::unique_lock lock(mutex_);
      work_available_.wait(
        lock, [this]() {return pending_.has_value() || stopping_.load(std::memory_order_acquire);});
      if (!pending_) {
        return;                     // shutting down with nothing outstanding
      }
      pending = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }

    auto completion = run_transaction(pending);

    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
    }
    try {
      pending.callback(std::move(completion));
    } catch (...) {
      // A throwing sink must not kill the worker, or later transactions would never complete.
    }
  }
}

AttachmentOutcome RosAttachmentPort::run_saga(
  const AttachmentGoal & goal, Transaction & transaction, std::string & detail)
{
  if (canceled()) {
    detail = "cancellation arrived before the planning-scene lease was requested";
    return AttachmentOutcome::kCanceled;
  }
  if (goal.scope == AttachmentScope::kSemanticOnly &&
    !goal.planning_scene_lease_token.empty())
  {
    // Continue the lease a physical-only half retained, keeping the projector frozen across the
    // retreat survey that proves the placement.
    transaction.lease_token = goal.planning_scene_lease_token;
    transaction.released_at.sec = goal.released_at.sec;
    transaction.released_at.nanosec = goal.released_at.nanosec;
  } else if (const auto lease_outcome = acquire_lease(goal, transaction, detail)) {
    return *lease_outcome;
  }
  if (canceled()) {
    // Nothing has been commanded, so the lease can be handed straight back, unless this is a
    // semantic-only continuation: a prior physical detach already mutated Gazebo.
    if (goal.scope == AttachmentScope::kSemanticOnly) {
      detail = "cancellation arrived after physical detach; semantic commit must still run";
      return AttachmentOutcome::kIndeterminate;
    }
    detail = "cancellation arrived before the physical transition was commanded";
    return AttachmentOutcome::kCanceled;
  }
  if (goal.scope != AttachmentScope::kSemanticOnly) {
    if (const auto physical_outcome = apply_physical_transition(goal, transaction, detail)) {
      return *physical_outcome;
    }
  } else {
    transaction.released_at.sec = goal.released_at.sec;
    transaction.released_at.nanosec = goal.released_at.nanosec;
  }
  if (goal.scope == AttachmentScope::kPhysicalOnly) {
    detail = describe(
      std::string(attachment_direction_name(goal.direction)) +
      " physical transition verified; semantic commit deferred for destination survey", detail);
    return AttachmentOutcome::kSucceeded;
  }
  if (const auto commit_outcome = commit_semantic_state(goal, transaction, detail)) {
    return *commit_outcome;
  }
  detail = describe(
    std::string(attachment_direction_name(goal.direction)) +
    " transition verified and committed", detail);
  return AttachmentOutcome::kSucceeded;
}

AttachmentCompletion RosAttachmentPort::run_transaction(const PendingGoal & pending) noexcept
{
  Transaction detached_transaction;
  // busy_ excludes another admission while the worker installs/writes this one bounded record.
  // Detach never replaces it. An unsafe attach also blocks every subsequent saga submission.
  Transaction & transaction = pending.goal.direction == AttachmentDirection::kAttach ?
    retained_attachment_.emplace() : detached_transaction;
  std::string detail;
  AttachmentOutcome outcome = AttachmentOutcome::kIndeterminate;

  try {
    transaction.correlation = pending.correlation;
    transaction.operation_id = next_operation_id(pending.goal, pending.correlation);
    if (pending.goal.direction == AttachmentDirection::kAttach) {
      transaction.attachment_request = std::make_shared<CommitAttachment::Request>();
      transaction.attachment_request->token = pending.goal.reservation_token;
      transaction.attachment_request->operation_id = transaction.operation_id + "-commit";
      transaction.attachment_request->grasp_center_from_held_object =
        tf2::toMsg(pending.goal.grasp_center_to_child);
    }
    outcome = run_saga(pending.goal, transaction, detail);
  } catch (const std::exception & error) {
    outcome = AttachmentOutcome::kIndeterminate;
    try {
      detail = std::string("attachment transaction raised: ") + error.what();
    } catch (...) {
      detail.clear();
    }
  } catch (...) {
    outcome = AttachmentOutcome::kIndeterminate;
    try {
      detail = "attachment transaction raised";
    } catch (...) {
      detail.clear();
    }
  }

  // Releasing the lease re-arms the projector, which re-derives managed geometry from world state.
  // That is safe only when the physical and semantic worlds are known to agree.
  // Physical-only success retains the lease so the retreat survey plans against a frozen scene;
  // the coordinator must pass the token to the semantic-only half.
  const auto effect = attachment_world_effect(outcome);
  std::string retained_lease_token;
  try {
    if (!transaction.lease_token.empty()) {
      const bool intentional_physical_only =
        pending.goal.scope == AttachmentScope::kPhysicalOnly &&
        outcome == AttachmentOutcome::kSucceeded;
      if (intentional_physical_only) {
        transaction.lease_released = false;
        retained_lease_token = transaction.lease_token;
        detail += "; planning-scene lease retained for the deferred semantic commit";
      } else {
        if (effect == AttachmentWorldEffect::kNoneApplied ||
          effect == AttachmentWorldEffect::kFullyApplied)
        {
          release_lease(transaction);
        } else {
          transaction.lease_released = false;
          detail +=
            "; planning-scene lease deliberately retained because the physical and semantic "
            "worlds cannot be shown to agree";
        }
      }
    }
  } catch (...) {
    // A release/diagnostic exception does not erase the already applied effect or its record.
    transaction.lease_released = false;
  }
  transaction.unsafe_terminal = pending.goal.direction == AttachmentDirection::kAttach &&
    (effect == AttachmentWorldEffect::kPhysicalOnly ||
    effect == AttachmentWorldEffect::kIndeterminate) && transaction.physical_send_attempted;

  return AttachmentCompletion{
    pending.correlation, outcome, transaction.world_revision, transaction.lease_released,
    std::move(retained_lease_token),
    AttachmentStamp{transaction.released_at.sec, transaction.released_at.nanosec},
    std::move(detail)};
}

std::optional<AttachmentOutcome> RosAttachmentPort::acquire_lease(
  const AttachmentGoal & goal, Transaction & transaction, std::string & detail)
{
  const auto deadline = operations_.now() + goal.lease_timeout;
  auto request = std::make_shared<AcquireLease::Request>();
  request->operation_id = transaction.operation_id + "-lease";
  // Exclusivity is needed, not a particular revision.
  request->minimum_applied_revision = 0;

  bool any_reply = false;
  for (;; ) {
    if (auto response = operations_.acquire(request)) {
      any_reply = true;
      const auto code = response->status.code;
      if (code == LeaseStatus::GRANTED) {
        if (!response->has_lease || response->token.empty()) {
          // Nothing was commanded, but a lease may be held under a name the port never learned.
          transaction.lease_released = false;
          detail = describe(
            "planning-scene lease was granted without a usable capability",
            response->status.detail);
          return AttachmentOutcome::kRejected;
        }
        transaction.lease_token = response->token;
        return std::nullopt;
      }
      if (code == LeaseStatus::INVALID_ARGUMENT || code == LeaseStatus::CONFLICT ||
        code == LeaseStatus::TOKEN_MISMATCH || code == LeaseStatus::IDEMPOTENCY_CONFLICT ||
        code == LeaseStatus::RESOURCE_EXHAUSTED)
      {
        detail = describe("planning-scene lease was refused", response->status.detail);
        return AttachmentOutcome::kRejected;
      }
      // DRAINING and INTERNAL_ERROR are polled: acquisition is idempotent under this operation id.
    }

    if (canceled()) {
      detail = "cancellation arrived while acquiring the planning-scene lease";
      // The acquisition may still land, freezing the projector under a lease the port cannot name.
      transaction.lease_released = false;
      return AttachmentOutcome::kCanceled;
    }
    if (operations_.now() >= deadline) {
      if (!any_reply && !operations_.ready(RosAttachmentPortOperations::Service::kAcquire)) {
        detail = "planning-scene lease service never became available";
        return AttachmentOutcome::kUnavailable;
      }
      detail = "planning-scene lease was not granted before its deadline";
      transaction.lease_released = false;
      return AttachmentOutcome::kRejected;
    }
    if (!wait_poll_period()) {
      detail = "attachment port shut down while acquiring the planning-scene lease";
      transaction.lease_released = false;
      return AttachmentOutcome::kCanceled;
    }
  }
}

std::optional<AttachmentOutcome> RosAttachmentPort::apply_physical_transition(
  const AttachmentGoal & goal, Transaction & transaction, std::string & detail)
{
  const auto deadline = operations_.now() + goal.physical_timeout;
  auto request = std::make_shared<SetAttachment::Request>();
  request->command = goal.direction == AttachmentDirection::kAttach ?
    SetAttachment::Request::COMMAND_ATTACH : SetAttachment::Request::COMMAND_DETACH;
  request->operation_id = transaction.operation_id;
  request->object_id = goal.object_id;
  request->reservation_token = goal.reservation_token;
  request->planning_scene_lease_token = transaction.lease_token;
  if (goal.direction == AttachmentDirection::kAttach) {
    request->expected_grasp_center_to_child = tf2::toMsg(goal.grasp_center_to_child);
    std::copy(
      goal.expected_pose_covariance.begin(), goal.expected_pose_covariance.end(),
      request->expected_pose_covariance.begin());
  }

  bool any_reply = false;
  for (;; ) {
    transaction.physical_send_attempted = true;
    std::optional<SetAttachment::Response> response = operations_.physical(request);
    if (response) {
      any_reply = true;
      const auto code = response->status.code;
      if (code != PhysicalStatus::PENDING && code != PhysicalStatus::UNSET) {
        // Prefer the reply's phase; fall back to the latched sample for this operation.
        std::optional<AttachmentState> state;
        if (response->has_state) {
          if (goal.direction == AttachmentDirection::kAttach) {
            state.emplace(std::move(response->state));
          } else {
            state = response->state;
          }
        } else {
          state = latest_state(transaction.operation_id);
        }
        const std::optional<std::uint8_t> phase =
          state ? std::optional<std::uint8_t>(state->phase) : std::nullopt;

        if (code == applied_code(goal.direction) && phase &&
          *phase == applied_phase(goal.direction))
        {
          // Simulator time at which the boundary verified this phase. Placement commits need
          // evidence newer than the release, so this must be the boundary's own observation.
          transaction.released_at = state->observed_at;
          transaction.physical_verified = true;
          if (goal.direction == AttachmentDirection::kAttach) {
            static_assert(std::is_nothrow_move_constructible_v<AttachmentState>);
            transaction.physical_receipt.emplace(std::move(*state));
          }
          return std::nullopt;
        }
        if (physical_refusal_code(code) && phase && *phase == untouched_phase(goal.direction)) {
          detail = describe(
            std::string("simulated ") + attachment_direction_name(goal.direction) +
            " was refused with the boundary still in its original phase",
            response->status.detail);
          return AttachmentOutcome::kPhysicalFailed;
        }
        detail = describe(
          std::string("simulated ") + attachment_direction_name(goal.direction) +
          " reached a terminal status the port cannot prove either way",
          response->status.detail);
        return AttachmentOutcome::kIndeterminate;
      }
    }

    if (operations_.now() >= deadline) {
      if (!any_reply && !operations_.ready(RosAttachmentPortOperations::Service::kPhysical) &&
        !latest_state(transaction.operation_id))
      {
        // The mutation service exists only after startup reconciliation; never reaching it means
        // nothing was commanded.
        detail = "simulated attachment service never became available";
        return AttachmentOutcome::kUnavailable;
      }
      detail = "simulated attachment did not reach a terminal phase before its deadline";
      return AttachmentOutcome::kIndeterminate;
    }
    // Cancellation is ignored here: the command is in flight and verified over several ticks, so
    // abandoning the poll would leave Gazebo and world state unreconciled. Only shutdown cuts it
    // short, and then the outcome is indeterminate.
    if (!wait_poll_period()) {
      detail = "attachment port shut down while the simulated transition was outstanding";
      return AttachmentOutcome::kIndeterminate;
    }
  }
}

std::optional<AttachmentOutcome> RosAttachmentPort::commit_semantic_state(
  const AttachmentGoal & goal, Transaction & transaction, std::string & detail)
{
  if (goal.direction == AttachmentDirection::kAttach) {
    return commit_attachment(goal, transaction, detail);
  }
  const auto deadline = operations_.now() + goal.commit_timeout;
  const std::string commit_operation_id = transaction.operation_id + "-commit";

  bool any_reply = false;
  std::string pending_refusal;
  for (;; ) {
    std::uint16_t code = WorldStatus::UNSET;
    std::string remote_detail;
    bool replied = false;

    {
      auto request = std::make_shared<CommitDetachment::Request>();
      request->token = goal.reservation_token;
      request->operation_id = commit_operation_id;
      request->disposition =
        goal.detach_disposition == DetachDisposition::kPlaceInReservedDestination ?
        CommitDetachment::Request::PLACE_IN_RESERVED_DESTINATION :
        CommitDetachment::Request::RELEASE_WITHOUT_MEMBERSHIP;
      request->released_at = transaction.released_at;
      if (auto response = operations_.detach(request)) {
        replied = true;
        code = response->status.code;
        remote_detail = response->status.detail;
        transaction.world_revision = response->world_revision;
      }
    }

    if (replied) {
      any_reply = true;
      if (code == WorldStatus::OK) {
        if (!pending_refusal.empty() && node_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "placement commit accepted once the destination evidence caught up");
        }
        return std::nullopt;
      }
      if (commit_refusal_code(code)) {
        // World state did not change but the physical transition did, and no automatic action can
        // restore agreement: the boundary would authorize the inverse mutation only from the
        // reservation stage this commit failed to reach. Exception: a placement whose evidence is
        // still catching up is re-asked until the deadline, then reaches this verdict.
        if (placement_evidence_may_still_settle(goal, code)) {
          if (pending_refusal.empty() && node_) {
            RCLCPP_INFO(
              node_->get_logger(),
              "placement commit is waiting for destination evidence to catch up with the applied "
              "detach: %s", remote_detail.c_str());
          }
          pending_refusal = remote_detail;
        } else {
          transaction.world_revision = 0;
          detail = describe(
            std::string("simulated ") + attachment_direction_name(goal.direction) +
            " is applied but world state refused the commit", remote_detail);
          return AttachmentOutcome::kUncommitted;
        }
      } else if (code == WorldStatus::IDEMPOTENCY_CONFLICT) {
        // The revision the reply carries belongs to whatever else claimed this identity.
        transaction.world_revision = 0;
        detail = describe(
          "world-state commit collided with a different operation under the same identity",
          remote_detail);
        return AttachmentOutcome::kIndeterminate;
      }
      // INTERNAL_ERROR and UNSET are replayed: the commit is idempotent under this operation id.
    }

    if (operations_.now() >= deadline) {
      transaction.world_revision = 0;
      if (!pending_refusal.empty()) {
        // Evidence never caught up: physically applied, never accepted by world state.
        detail = describe(
          std::string("simulated ") + attachment_direction_name(goal.direction) +
          " is applied but world state refused the commit until its deadline", pending_refusal);
        return AttachmentOutcome::kUncommitted;
      }
      if (!any_reply && !operations_.ready(RosAttachmentPortOperations::Service::kAttach) &&
        !operations_.ready(RosAttachmentPortOperations::Service::kDetach))
      {
        // Nothing reached world state but the physical transition is applied: a divergence, not a
        // clean unavailability.
        detail = "world-state commit service never became available after the physical transition";
        return AttachmentOutcome::kUncommitted;
      }
      detail = "world-state commit did not resolve before its deadline";
      return AttachmentOutcome::kIndeterminate;
    }
    if (!wait_poll_period()) {
      transaction.world_revision = 0;
      detail = "attachment port shut down while the world-state commit was outstanding";
      return AttachmentOutcome::kIndeterminate;
    }
  }
}

std::optional<AttachmentOutcome> RosAttachmentPort::commit_attachment(
  const AttachmentGoal & goal, Transaction & transaction, std::string & detail)
{
  // The request was prepared before the physical send. Neither observations nor retry outcomes
  // can substitute another grasp, token, operation identity, or deadline.
  const auto deadline = operations_.now() + goal.commit_timeout;
  auto & history = transaction.commit_history;
  const auto unresolved_outcome = [&history]() {
    return history.unresolved || history.inhibited ? AttachmentOutcome::kIndeterminate :
           AttachmentOutcome::kUncommitted;
  };
  for (;; ) {
    if (history.inhibited || canceled() || operations_.now() >= deadline) {
      detail = "physical attach retained; semantic settlement stopped or its deadline expired";
      return unresolved_outcome();
    }
    auto result = operations_.attach(transaction.attachment_request, deadline, history);
    if (result.dispatch == AttachmentCommitDispatch::kMatchedReply && result.response) {
      const auto code = result.response->status.code;
      history.last_status = code;
      const bool has_commit_receipt = result.response->has_reservation &&
        result.response->world_revision != 0U &&
        result.response->reservation.object_id == goal.object_id &&
        result.response->reservation.stage ==
        restocker_interfaces::msg::TaskReservation::STAGE_ATTACHED;
      if (code == WorldStatus::CLOCK_AUTHORITY_INHIBITED) {
        history.inhibited = true;
        if (has_commit_receipt) {
          history.unresolved = false;
          transaction.historical_receipt = std::move(result.response);
        }
        detail =
          "attachment authority is clock-inhibited; historical evidence cannot resume motion";
        return AttachmentOutcome::kIndeterminate;
      }
      if (code == WorldStatus::OK && has_commit_receipt) {
        // Exact immutable request/replay resolves all earlier possibly submitted calls, but does
        // not revive a deadline/cancellation-terminal transaction or override observed inhibition.
        history.unresolved = false;
        transaction.historical_receipt = std::move(result.response);
        if (operations_.now() >= deadline || canceled() || history.inhibited) {
          detail =
            "attachment success retained after settlement closed; operator reconciliation required";
          return AttachmentOutcome::kIndeterminate;
        }
        transaction.world_revision = transaction.historical_receipt->world_revision;
        return std::nullopt;
      }
      const bool readiness = code == WorldStatus::ATTACHMENT_CLOCK_NOT_READY &&
        !result.response->has_reservation;
      if (readiness || (commit_refusal_code(code) && !result.response->has_reservation)) {
        // A definite nonmutation reply resolves THIS call only. Earlier missing replies remain
        // unknown, irrespective of current service readiness or diagnostic text.
        history.unresolved = result.unresolved_before;
        if (!readiness) {
          detail = describe(
            "physical attach applied but semantic commit refused", result.response->status.detail);
          return unresolved_outcome();
        }
      } else {
        history.unresolved = true;
        if (code != WorldStatus::INTERNAL_ERROR && code != WorldStatus::UNSET) {
          detail =
            "attachment commit returned conflicting, malformed or unknown authority evidence";
          return AttachmentOutcome::kIndeterminate;
        }
      }
    }
    // A ready response can be processed after expiry; even exact success above cannot then take
    // the normal completion/release path. Each poll consumes this same original steady budget.
    if (canceled() || operations_.now() >= deadline) {
      detail = "physical attach retained; semantic settlement stopped or its deadline expired";
      return unresolved_outcome();
    }
    const auto next_poll = std::min(deadline, operations_.now() + config_.poll_period);
    if (!operations_.wait_until(next_poll)) {
      detail = "physical attach retained while the semantic settlement worker shut down";
      return unresolved_outcome();
    }
  }
}

void RosAttachmentPort::release_lease(Transaction & transaction)
{
  const auto deadline = operations_.now() + config_.release_timeout;
  auto request = std::make_shared<ReleaseLease::Request>();
  request->operation_id = transaction.operation_id + "-release";
  request->token = transaction.lease_token;
  // The projector waits for this revision before re-deriving the scene.
  request->required_semantic_revision = transaction.world_revision;

  for (;; ) {
    if (auto response = operations_.release(request)) {
      const auto code = response->status.code;
      if (code == LeaseStatus::RELEASE_ACCEPTED || code == LeaseStatus::TOKEN_MISMATCH) {
        // A token mismatch means this lease is already gone, which is the state release wanted.
        transaction.lease_released = true;
        return;
      }
      if (code == LeaseStatus::INVALID_ARGUMENT) {
        transaction.lease_released = false;
        return;
      }
    }
    if (operations_.now() >= deadline) {
      transaction.lease_released = false;
      return;
    }
    if (!wait_poll_period()) {
      transaction.lease_released = false;
      return;
    }
  }
}

template<typename Service>
std::optional<typename Service::Response> RosAttachmentPort::call(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  typename Service::Request::SharedPtr request)
{
  if (!client->service_is_ready()) {
    return std::nullopt;
  }
  try {
    auto future = client->async_send_request(request);
    if (future.wait_for(config_.call_timeout) != std::future_status::ready) {
      // Otherwise every lapsed poll leaks a pending slot in the client.
      client->remove_pending_request(future);
      return std::nullopt;
    }
    auto response = future.get();
    if (!response) {
      return std::nullopt;
    }
    return *response;
  } catch (const std::exception &) {
    return std::nullopt;
  } catch (...) {
    return std::nullopt;
  }
}

bool RosAttachmentPort::wait_poll_period()
{
  return operations_.wait_until(operations_.now() + config_.poll_period);
}

std::optional<RosAttachmentPort::AttachmentState> RosAttachmentPort::latest_state(
  const std::string & operation_id) const
{
  std::scoped_lock lock(state_mutex_);
  if (!latest_state_ || latest_state_->operation_id != operation_id) {
    // Idle-watchdog samples carry no operation id.
    return std::nullopt;
  }
  return latest_state_;
}

std::string RosAttachmentPort::next_operation_id(
  const AttachmentGoal & goal, OperationCorrelation correlation)
{
  std::uint64_t sequence = 0;
  {
    std::scoped_lock lock(mutex_);
    sequence = ++operation_sequence_;
  }
  // Unique within this process only; replay across a coordinator restart needs a persisted id.
  return "restock-" + std::string(attachment_direction_name(goal.direction)) + "-" +
         std::to_string(correlation.goal_generation) + "-" +
         std::to_string(correlation.operation_generation) + "-" + std::to_string(sequence);
}

}  // namespace restocker_task_executor
