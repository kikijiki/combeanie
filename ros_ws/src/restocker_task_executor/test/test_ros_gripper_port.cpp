// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "restocker_task_executor/ros_gripper_port.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// A gripper_controller stand-in that accepts a trajectory and reports success immediately, the
// way joint_trajectory_controller does once its own far looser goal constraint is met, plus the
// /joint_states stream the port verifies against.
class ControllerStub
{
public:
  explicit ControllerStub(const std::string & suffix)
  : node_(std::make_shared<rclcpp::Node>("gripper_controller_stub_" + suffix))
  {
    server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      node_, "/gripper_controller_" + suffix + "/follow_joint_trajectory",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowJointTrajectory::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>>) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> handle) {
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        handle->succeed(result);
        ++accepted_goals_;
      });
    joint_states_ = node_->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states_" + suffix, rclcpp::SensorDataQoS());
    executor_.add_node(node_);
    thread_ = std::thread([this]() {executor_.spin();});
  }

  ~ControllerStub()
  {
    executor_.cancel();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ControllerStub(const ControllerStub &) = delete;
  ControllerStub & operator=(const ControllerStub &) = delete;

  void publish_fingers(double position)
  {
    sensor_msgs::msg::JointState sample;
    sample.name = {"left_finger_joint", "right_finger_joint"};
    sample.position = {position, position};
    joint_states_->publish(sample);
  }

  [[nodiscard]] std::size_t accepted_goals() const {return accepted_goals_.load();}

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr server_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
  std::atomic<std::size_t> accepted_goals_{0};
};

[[nodiscard]] RosGripperPortConfig stub_config(const std::string & suffix)
{
  RosGripperPortConfig config;
  config.action_name = "/gripper_controller_" + suffix + "/follow_joint_trajectory";
  config.joint_states_topic = "/joint_states_" + suffix;
  config.server_wait = 5s;
  config.verification_wait = 2s;
  return config;
}

[[nodiscard]] GripperGoal hold_goal()
{
  GripperGoal goal;
  goal.target_position_m = 0.013;
  goal.move_duration = 50ms;
  goal.deadline = 8s;
  return goal;
}

// Drives one submission to completion, feeding the finger positions in `samples` one at a time
// while the port waits. Returns the completion the port reported.
[[nodiscard]] GripperCompletion run_submission(
  const std::string & suffix, const std::vector<double> & samples)
{
  ControllerStub stub(suffix);
  rclcpp::NodeOptions options;
  RosGripperPort port(options, stub_config(suffix), "restock_gripper_client_" + suffix);

  // Readiness fails closed until a sample has been seen, so seed the stream first.
  const auto ready_deadline = std::chrono::steady_clock::now() + 10s;
  while (!port.ready() && std::chrono::steady_clock::now() < ready_deadline) {
    stub.publish_fingers(0.0);
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(port.ready());

  std::promise<GripperCompletion> promise;
  auto completed = promise.get_future();
  const auto sent = port.submit(
    OperationCorrelation{1U, 1U}, hold_goal(),
    [&promise](GripperCompletion completion) {promise.set_value(std::move(completion));});
  EXPECT_TRUE(static_cast<bool>(sent)) << sent.detail;

  // The jaw moves while the port is already waiting, exactly as it does in the simulator.
  for (const double position : samples) {
    for (int repeat = 0; repeat < 5 && completed.wait_for(0s) != std::future_status::ready;
      ++repeat)
    {
      stub.publish_fingers(position);
      std::this_thread::sleep_for(20ms);
    }
  }
  const auto arrived = completed.wait_for(10s);
  EXPECT_EQ(arrived, std::future_status::ready);
  auto completion = completed.get();
  port.shutdown();
  return completion;
}

class RosGripperPortTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  // Both the port and the stub own executor threads and nodes. Shutting the context down inside
  // the test, once every one of them has been destroyed, keeps that teardown ordered instead of
  // leaving it to static destruction at process exit.
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

// Regression: joint_trajectory_controller reports "Goal reached" at its own 0.003 m constraint,
// several samples before the jaw comes to rest. Judging the single sample that happened to arrive
// first therefore read the finger mid-travel (0.0162 m against a commanded 0.0170 m) and
// failed a grasp that was about to be perfect. The port has to watch the jaw settle.
TEST_F(RosGripperPortTest, WaitsForTheJawToSettleRatherThanJudgingTheFirstSample)
{
  const auto completion = run_submission("settle", {0.0100, 0.0122, 0.0130});
  EXPECT_EQ(completion.outcome, GripperOutcome::kSucceeded) << completion.detail;
  ASSERT_TRUE(completion.measured);
  EXPECT_NEAR(completion.measured->left_position_m, 0.013, 1.0e-9);
  EXPECT_NEAR(completion.measured->right_position_m, 0.013, 1.0e-9);
}

// A jaw that never reaches the band is still a failed grasp, and the evidence reported is the
// last place the fingers actually were.
TEST_F(RosGripperPortTest, ReportsTheLastMeasurementWhenTheJawNeverSettles)
{
  const auto completion = run_submission("stall", {0.0100, 0.0118});
  EXPECT_EQ(completion.outcome, GripperOutcome::kPositionNotVerified) << completion.detail;
  ASSERT_TRUE(completion.measured);
  EXPECT_NEAR(completion.measured->left_position_m, 0.0118, 1.0e-9);
}

}  // namespace
}  // namespace restocker_task_executor
