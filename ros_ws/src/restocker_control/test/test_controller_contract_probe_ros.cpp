// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <controller_manager_msgs/msg/controller_state.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "restocker_control/controller_contract_probe.hpp"
#include "restocker_control/controller_contract_probe_ros.hpp"

namespace restocker_control
{
namespace
{

using namespace std::chrono_literals;
using ListControllers = controller_manager_msgs::srv::ListControllers;

void add_healthy_controllers(ListControllers::Response & response)
{
  using ControllerState = controller_manager_msgs::msg::ControllerState;
  response.controller = {
    ControllerState{}.set__name("joint_state_broadcaster")
    .set__type("joint_state_broadcaster/JointStateBroadcaster").set__state("active"),
    ControllerState{}.set__name("arm_controller")
    .set__type("joint_trajectory_controller/JointTrajectoryController").set__state("active")
    .set__claimed_interfaces(
      {"shoulder_pan_joint/velocity", "shoulder_lift_joint/velocity", "elbow_joint/velocity",
        "wrist_1_joint/velocity",
        "wrist_2_joint/velocity", "wrist_3_joint/velocity"}),
    ControllerState{}.set__name("rail_controller")
    .set__type("joint_trajectory_controller/JointTrajectoryController").set__state("active")
    .set__claimed_interfaces({"rail_joint/velocity"}),
    ControllerState{}.set__name("gripper_controller")
    .set__type("joint_trajectory_controller/JointTrajectoryController").set__state("active")
    .set__claimed_interfaces({"left_finger_joint/position", "right_finger_joint/position"}),
  };
}

TEST(ControllerContractProbeRos, ValidContextBackoffElapses)
{
  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto node = std::make_shared<rclcpp::Node>("controller_probe_backoff_client");
  auto client = node->create_client<ListControllers>(
    "/controller_probe_backoff/unused_list_controllers");
  RclcppControllerProbeTransport transport(node, client);
  EXPECT_EQ(transport.wait_for_retry(5ms), ControllerBackoffStatus::kElapsed);
  rclcpp::shutdown();
}

TEST(ControllerContractProbeRos, LateFirstResponseCannotSatisfyRetriedRequest)
{
  int argc = 0;
  rclcpp::init(argc, nullptr);
  auto server_node = std::make_shared<rclcpp::Node>("controller_probe_retry_server");
  auto client_node = std::make_shared<rclcpp::Node>("controller_probe_retry_client");
  std::atomic<std::size_t> request_count{0U};
  auto service = server_node->create_service<ListControllers>(
    "/controller_probe_retry/list_controllers",
    [&request_count](
      ListControllers::Request::SharedPtr,
      ListControllers::Response::SharedPtr response) {
      if (request_count.fetch_add(1U, std::memory_order_acq_rel) == 0U) {
        std::this_thread::sleep_for(250ms);
      }
      add_healthy_controllers(*response);
    });
  auto client = client_node->create_client<ListControllers>(
    "/controller_probe_retry/list_controllers");
  rclcpp::executors::SingleThreadedExecutor server_executor;
  server_executor.add_node(server_node);
  std::jthread server_thread([&server_executor]() {server_executor.spin();});

  RclcppControllerProbeTransport transport(client_node, client);
  ControllerProbeConfig config{2s, 100ms, 20ms};
  const ControllerContractProbe probe(
    config, []() {return std::chrono::steady_clock::now();});
  const auto result = probe.run(transport);

  EXPECT_EQ(result.status, ControllerProbeStatus::kHealthy);
  EXPECT_GE(result.request_attempts, 2U);
  EXPECT_GE(result.timed_out_requests, 1U);
  EXPECT_GE(request_count.load(), 2U);

  rclcpp::shutdown();
  server_executor.cancel();
  server_thread.join();
  (void)service;
}

}  // namespace
}  // namespace restocker_control
