// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "restocker_task_executor/gripper_port.hpp"

namespace restocker_task_executor
{

struct RosGripperPortConfig
{
  // The gripper is a plain joint_trajectory_controller, not a GripperActionController, so it is
  // commanded through FollowJointTrajectory like the arm and rail.
  std::string action_name{"/gripper_controller/follow_joint_trajectory"};
  std::string joint_states_topic{"/joint_states"};
  std::string left_joint{"left_finger_joint"};
  std::string right_joint{"right_finger_joint"};
  // How long to wait for the action server before reporting kUnavailable.
  std::chrono::milliseconds server_wait{std::chrono::seconds(10)};
  // How long to keep watching /joint_states after the trajectory finished for the fingers to
  // settle inside the caller's tolerance. A stale sample would describe where the fingers were
  // before the move, and the first fresh one is taken while they are still converging: the
  // controller declares success at its own 0.003 m constraint, which it reaches several samples
  // before the jaw comes to rest inside the 0.0005 m attachment band.
  std::chrono::milliseconds verification_wait{std::chrono::seconds(3)};
};

// A GripperPort backed by the gripper_controller FollowJointTrajectory action, with independent
// position verification from /joint_states.
//
// Verification is needed because gripper_controller's goal tolerance is 0.003 m while the Gazebo
// attachment plugin refuses to latch unless both fingers are within 0.0005 m of the hold target,
// so the action can report SUCCEEDED on a jaw that will never grasp. The port reports kSucceeded
// only after a fresh /joint_states sample confirms the target within the caller's tolerance. The
// jaw is still moving when the controller declares success, so the port watches the sample
// stream until it settles instead of judging the first sample.
//
// Like MoveItMotionPort, it owns a private node, a single-threaded executor thread for it, and one
// worker thread, so no action wait and no verification wait ever runs on a coordinator callback
// group.
class RosGripperPort final : public GripperPort
{
public:
  RosGripperPort(
    const rclcpp::NodeOptions & options, RosGripperPortConfig config = {},
    std::string node_name = "restock_gripper_client");
  ~RosGripperPort() override;

  RosGripperPort(const RosGripperPort &) = delete;
  RosGripperPort & operator=(const RosGripperPort &) = delete;
  RosGripperPort(RosGripperPort &&) = delete;
  RosGripperPort & operator=(RosGripperPort &&) = delete;

  [[nodiscard]] bool ready() const override;

  [[nodiscard]] GripperSubmitResult submit(
    OperationCorrelation correlation, GripperGoal goal, CompletionCallback callback) override;

  void cancel() noexcept override;

  // Stop the worker and release the action client. Idempotent; the destructor calls it.
  void shutdown() noexcept;

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  struct PendingGoal
  {
    OperationCorrelation correlation;
    GripperGoal goal;
    CompletionCallback callback;
  };

  // A joint-state sample plus the local sequence number that produced it. The sequence, not the
  // header stamp, distinguishes "published after the move" from "before it": /joint_states
  // carries simulation time, which this node need not follow.
  struct JointSample
  {
    GripperFingerState fingers;
    std::uint64_t sequence{0U};
  };

  // What watching the sample stream after a trajectory proved. `measured` holds the most recent
  // fresh sample seen, which is the evidence the failure detail and the coordinator report, and
  // is absent only when no post-move sample naming both fingers ever arrived.
  struct SettleObservation
  {
    std::optional<GripperFingerState> measured;
    bool settled{false};
  };

  void run_worker() noexcept;
  // Runs on the worker thread. Never throws; every failure maps to a GripperOutcome.
  [[nodiscard]] GripperCompletion execute_goal(const PendingGoal & pending) noexcept;
  // Blocks until a sample newer than `baseline_sequence` arrives or `deadline` passes.
  [[nodiscard]] std::optional<JointSample> await_fresh_sample(
    std::uint64_t baseline_sequence, std::chrono::steady_clock::time_point deadline);
  // Consumes samples newer than `baseline_sequence` until both fingers rest inside
  // `position_tolerance_m` of `target_position_m` or `deadline` passes. The controller reports
  // success at its far looser goal constraint, so the first post-move sample often shows the jaw
  // still short of the commanded width.
  [[nodiscard]] SettleObservation await_settled_sample(
    std::uint64_t baseline_sequence, double target_position_m, double position_tolerance_m,
    std::chrono::steady_clock::time_point deadline);
  [[nodiscard]] std::uint64_t latest_sequence() const noexcept;
  void on_joint_state(const sensor_msgs::msg::JointState & message);
  // Best-effort cancellation of whatever goal handle is currently outstanding.
  void request_goal_cancel() noexcept;

  RosGripperPortConfig config_;
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;

  mutable std::mutex sample_mutex_;
  std::condition_variable sample_arrived_;
  std::optional<JointSample> sample_;
  std::uint64_t sample_sequence_{0U};

  std::mutex goal_handle_mutex_;
  GoalHandle::SharedPtr active_goal_;

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::optional<PendingGoal> pending_;
  bool busy_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> cancel_requested_{false};
  std::thread worker_;
};

}  // namespace restocker_task_executor
