// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/ros_gripper_port.hpp"

#include <algorithm>
#include <exception>
#include <future>
#include <sstream>
#include <utility>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace restocker_task_executor
{
namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

[[nodiscard]] std::string describe(const GripperFingerState & fingers)
{
  std::ostringstream stream;
  stream << "left=" << fingers.left_position_m << "m right=" << fingers.right_position_m << "m";
  return stream.str();
}

[[nodiscard]] std::string describe_result(const FollowJointTrajectory::Result::SharedPtr & result)
{
  if (!result) {
    return "controller returned no result";
  }
  std::ostringstream stream;
  stream << "controller error_code=" << result->error_code;
  if (!result->error_string.empty()) {
    stream << " (" << result->error_string << ")";
  }
  return stream.str();
}

}  // namespace

RosGripperPort::RosGripperPort(
  const rclcpp::NodeOptions & options, RosGripperPortConfig config, std::string node_name)
: config_(std::move(config)),
  node_(std::make_shared<rclcpp::Node>(std::move(node_name), options)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  // Both endpoints are created eagerly (neither blocks, unlike MoveGroupInterface). The sample
  // stream must be running before the first trajectory finishes, or there is nothing fresh to
  // verify against.
  client_ = rclcpp_action::create_client<FollowJointTrajectory>(node_, config_.action_name);
  joint_states_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    config_.joint_states_topic, rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState & message) {on_joint_state(message);});

  executor_->add_node(node_);
  executor_thread_ = std::thread(
    [this]() noexcept {
      try {
        executor_->spin();
      } catch (...) {
        // Losing the private executor makes every later submission report kUnavailable, which is
        // the fail-closed outcome the coordinator already handles.
      }
    });
  worker_ = std::thread([this]() noexcept {run_worker();});
}

RosGripperPort::~RosGripperPort()
{
  shutdown();
}

void RosGripperPort::shutdown() noexcept
{
  if (stopping_.exchange(true)) {
    return;
  }
  cancel();
  work_available_.notify_all();
  // A worker parked in await_fresh_sample would otherwise wait out the full verification window.
  sample_arrived_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  try {
    executor_->cancel();
  } catch (...) {
    // The executor is being torn down; the join below is what matters.
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  {
    std::scoped_lock lock(goal_handle_mutex_);
    active_goal_.reset();
  }
  joint_states_.reset();
  client_.reset();
  try {
    executor_->remove_node(node_);
  } catch (...) {
    // Removal is best-effort during teardown.
  }
}

bool RosGripperPort::ready() const
{
  if (stopping_.load(std::memory_order_acquire)) {
    return false;
  }
  if (!client_ || !client_->action_server_is_ready()) {
    return false;
  }
  // Without joint states the jaw cannot be verified, so readiness fails closed until a sample has
  // been seen.
  std::scoped_lock lock(sample_mutex_);
  return sample_.has_value();
}

GripperSubmitResult RosGripperPort::submit(
  OperationCorrelation correlation, GripperGoal goal, CompletionCallback callback)
{
  if (!callback) {
    return {GripperSubmitStatus::kInvalidRequest,
      "gripper submission requires a completion callback"};
  }
  if (!valid_gripper_goal(goal)) {
    return {GripperSubmitStatus::kInvalidRequest,
      "gripper goal is not an in-range target with a usable tolerance and deadline"};
  }
  if (stopping_.load(std::memory_order_acquire)) {
    return {GripperSubmitStatus::kUnavailable, "gripper port is shutting down"};
  }

  {
    std::scoped_lock lock(mutex_);
    if (pending_ || busy_) {
      return {GripperSubmitStatus::kBusy, "gripper port already owns an outstanding goal"};
    }
    cancel_requested_.store(false, std::memory_order_release);
    pending_.emplace(PendingGoal{correlation, std::move(goal), std::move(callback)});
  }
  work_available_.notify_one();
  return {GripperSubmitStatus::kAccepted, {}};
}

void RosGripperPort::cancel() noexcept
{
  cancel_requested_.store(true, std::memory_order_release);
  request_goal_cancel();
  // Release a worker blocked on verification so the cancellation is observed promptly.
  sample_arrived_.notify_all();
}

void RosGripperPort::request_goal_cancel() noexcept
{
  GoalHandle::SharedPtr handle;
  {
    std::scoped_lock lock(goal_handle_mutex_);
    handle = active_goal_;
  }
  if (!handle || !client_) {
    return;
  }
  try {
    // Fire and forget: the worker is already waiting on the result future, which turns terminal
    // once the controller honours the cancellation.
    (void)client_->async_cancel_goal(handle);
  } catch (...) {
    // A cancel that cannot be delivered leaves the goal to hit its deadline, still terminal.
  }
}

void RosGripperPort::on_joint_state(const sensor_msgs::msg::JointState & message)
{
  const auto count = std::min(message.name.size(), message.position.size());
  std::optional<double> left;
  std::optional<double> right;
  for (std::size_t index = 0; index < count; ++index) {
    if (message.name[index] == config_.left_joint) {
      left = message.position[index];
    } else if (message.name[index] == config_.right_joint) {
      right = message.position[index];
    }
  }
  // A sample naming only one finger cannot prove the jaw pair; it is dropped, not merged into a
  // half-stale pair.
  if (!left || !right) {
    return;
  }
  {
    std::scoped_lock lock(sample_mutex_);
    ++sample_sequence_;
    sample_.emplace(JointSample{GripperFingerState{*left, *right}, sample_sequence_});
  }
  sample_arrived_.notify_all();
}

std::uint64_t RosGripperPort::latest_sequence() const noexcept
{
  std::scoped_lock lock(sample_mutex_);
  return sample_sequence_;
}

std::optional<RosGripperPort::JointSample> RosGripperPort::await_fresh_sample(
  std::uint64_t baseline_sequence, std::chrono::steady_clock::time_point deadline)
{
  std::unique_lock lock(sample_mutex_);
  const bool arrived = sample_arrived_.wait_until(
    lock, deadline,
    [this, baseline_sequence]() {
      return (sample_ && sample_->sequence > baseline_sequence) ||
             stopping_.load(std::memory_order_acquire);
    });
  if (!arrived || !sample_ || sample_->sequence <= baseline_sequence) {
    return std::nullopt;
  }
  return sample_;
}

RosGripperPort::SettleObservation RosGripperPort::await_settled_sample(
  std::uint64_t baseline_sequence, double target_position_m, double position_tolerance_m,
  std::chrono::steady_clock::time_point deadline)
{
  SettleObservation observation;
  std::uint64_t observed = baseline_sequence;
  while (!stopping_.load(std::memory_order_acquire) &&
    !cancel_requested_.load(std::memory_order_acquire))
  {
    const auto sample = await_fresh_sample(observed, deadline);
    if (!sample) {
      break;                        // the settle window closed, or the stream went quiet
    }
    observed = sample->sequence;
    // Every fresh sample replaces the evidence: the newest one is where the jaw is, and the
    // caller must see it whether or not it settled.
    observation.measured = sample->fingers;
    if (fingers_at_target(sample->fingers, target_position_m, position_tolerance_m)) {
      observation.settled = true;
      break;
    }
  }
  return observation;
}

void RosGripperPort::run_worker() noexcept
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

    auto completion = execute_goal(pending);

    {
      std::scoped_lock lock(goal_handle_mutex_);
      active_goal_.reset();
    }
    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
    }
    // The coordinator's task fault detail is several transitions removed from the cause, so
    // record the outcome here.
    if (completion.measured) {
      RCLCPP_INFO(
        node_->get_logger(),
        "gripper %s: commanded %.4f m, measured left %.4f m right %.4f m (%s)",
        gripper_outcome_name(completion.outcome), pending.goal.target_position_m,
        completion.measured->left_position_m, completion.measured->right_position_m,
        completion.detail.c_str());
    } else {
      RCLCPP_INFO(
        node_->get_logger(), "gripper %s: commanded %.4f m, no measurement (%s)",
        gripper_outcome_name(completion.outcome), pending.goal.target_position_m,
        completion.detail.c_str());
    }
    try {
      pending.callback(std::move(completion));
    } catch (...) {
      // A throwing sink must not kill the worker, or later goals would never complete.
    }
  }
}

GripperCompletion RosGripperPort::execute_goal(const PendingGoal & pending) noexcept
{
  const auto fail = [&pending](GripperOutcome outcome, std::string detail) {
    return GripperCompletion{pending.correlation, outcome, std::move(detail), std::nullopt};
  };
  const auto canceled = [this]() {
    return cancel_requested_.load(std::memory_order_acquire);
  };

  if (canceled()) {
    return fail(GripperOutcome::kCanceled, "cancellation arrived before the goal was sent");
  }
  if (stopping_.load(std::memory_order_acquire) || !client_) {
    return fail(GripperOutcome::kUnavailable, "gripper port is shutting down");
  }

  const auto deadline = std::chrono::steady_clock::now() + pending.goal.deadline;
  const auto remaining = [deadline]() {
    return deadline - std::chrono::steady_clock::now();
  };

  try {
    if (!client_->wait_for_action_server(std::min(config_.server_wait, pending.goal.deadline))) {
      return fail(
        GripperOutcome::kUnavailable,
        "gripper action server " + config_.action_name + " is not available");
    }

    // The sequence is latched before the trajectory is sent, so verification only accepts samples
    // published after the fingers started moving.
    const std::uint64_t baseline_sequence = latest_sequence();

    FollowJointTrajectory::Goal goal;
    goal.trajectory.joint_names = {config_.left_joint, config_.right_joint};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {pending.goal.target_position_m, pending.goal.target_position_m};
    point.velocities = {0.0, 0.0};
    point.time_from_start = rclcpp::Duration(pending.goal.move_duration);
    goal.trajectory.points.push_back(std::move(point));
    // goal_tolerance is left empty. Tightening the controller's constraint to the attachment band
    // would turn a near miss into an opaque GOAL_TOLERANCE_VIOLATED abort instead of
    // kPositionNotVerified.

    auto goal_future = client_->async_send_goal(goal);
    if (goal_future.wait_for(remaining()) != std::future_status::ready) {
      return fail(
        GripperOutcome::kTimedOut, "gripper controller never answered the goal request");
    }
    const auto handle = goal_future.get();
    if (!handle) {
      return fail(
        GripperOutcome::kRejected, "gripper controller rejected the trajectory goal");
    }
    {
      std::scoped_lock lock(goal_handle_mutex_);
      active_goal_ = handle;
    }
    // A cancel racing acceptance would have found no handle to act on, so re-issue it here.
    if (canceled()) {
      request_goal_cancel();
    }

    auto result_future = client_->async_get_result(handle);
    if (result_future.wait_for(remaining()) != std::future_status::ready) {
      request_goal_cancel();
      return fail(
        GripperOutcome::kTimedOut, "gripper trajectory did not finish before its deadline");
    }
    const GoalHandle::WrappedResult wrapped = result_future.get();

    if (wrapped.code == rclcpp_action::ResultCode::CANCELED || canceled()) {
      return fail(
        GripperOutcome::kCanceled,
        "gripper trajectory stopped after cancellation: " + describe_result(wrapped.result));
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      return fail(GripperOutcome::kExecutionFailed, describe_result(wrapped.result));
    }

    // The controller reports success at its 0.003 m goal constraint while the jaw is still
    // converging. Wait for the fingers to come to rest inside the tighter band the attachment
    // plugin requires instead of judging the first sample.
    const auto verification_deadline =
      std::min(deadline, std::chrono::steady_clock::now() + config_.verification_wait);
    const auto settled = await_settled_sample(
      baseline_sequence, pending.goal.target_position_m, pending.goal.position_tolerance_m,
      verification_deadline);
    if (!settled.measured) {
      return fail(
        GripperOutcome::kPositionNotVerified,
        "no " + config_.joint_states_topic + " sample naming both fingers arrived after the move");
    }
    if (canceled()) {
      return GripperCompletion{
        pending.correlation, GripperOutcome::kCanceled,
        "cancellation arrived while the jaw was settling: " + describe(*settled.measured),
        settled.measured};
    }
    if (!settled.settled) {
      return GripperCompletion{
        pending.correlation, GripperOutcome::kPositionNotVerified,
        "fingers never settled at the commanded target: " + describe(*settled.measured),
        settled.measured};
    }
    return GripperCompletion{
      pending.correlation, GripperOutcome::kSucceeded,
      "fingers verified at the commanded target: " + describe(*settled.measured),
      settled.measured};
  } catch (const std::exception & error) {
    // The trajectory may already be running, so this cannot claim a not-started outcome.
    return fail(
      GripperOutcome::kExecutionFailed,
      std::string("gripper action raised during execution: ") + error.what());
  } catch (...) {
    return fail(GripperOutcome::kExecutionFailed, "gripper action raised during execution");
  }
}

}  // namespace restocker_task_executor
