// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <cstdlib>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include "restocker_control/control_contract.hpp"
#include "restocker_control/controller_contract_probe.hpp"
#include "restocker_control/controller_contract_probe_ros.hpp"
#include "restocker_control/smoke_trajectory_contract.hpp"

namespace restocker_control
{
namespace
{

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
using namespace std::chrono_literals;

enum class ExitCode : int
{
  success = 0,
  controller_service_timeout = 10,
  controller_response_timeout = 11,
  controller_contract_failure = 12,
  controller_interrupted = 13,
  controller_transport_failure = 14,
  invalid_configuration = 15,
  action_server_timeout = 20,
  goal_response_timeout = 21,
  goal_rejected = 22,
  result_timeout = 23,
  execution_failure = 24,
  joint_state_timeout = 30,
  position_mismatch = 31,
  initial_joint_state_timeout = 32,
  initial_position_mismatch = 33,
  insufficient_motion = 34,
};

class ControlSmokeTest : public rclcpp::Node
{
public:
  ControlSmokeTest()
  : Node("control_smoke_test"),
    startup_timeout_(declare_parameter<double>("startup_timeout_sec", 20.0)),
    controller_response_attempt_timeout_sec_(
      declare_parameter<double>("controller_response_attempt_timeout_sec", 1.0)),
    action_timeout_(declare_parameter<double>("action_timeout_sec", 5.0)),
    execute_trajectory_(declare_parameter<bool>("execute_trajectory", true))
  {
    list_controllers_ = create_client<controller_manager_msgs::srv::ListControllers>(
      "/controller_manager/list_controllers");
    joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState & message) {
        std::map<std::string, double> positions;
        std::scoped_lock lock(joint_state_mutex_);
        const auto count = std::min(message.name.size(), message.position.size());
        for (std::size_t index = 0; index < count; ++index) {
          positions[message.name[index]] = message.position[index];
        }
        latest_positions_ = std::move(positions);
        received_joint_state_ = true;
      });
  }

  ExitCode run()
  {
    if (!std::isfinite(action_timeout_.count()) || action_timeout_.count() <= 0.0) {
      RCLCPP_ERROR(get_logger(), "action_timeout_sec must be finite and positive");
      return ExitCode::invalid_configuration;
    }

    auto result = validate_controllers();
    if (result != ExitCode::success || !execute_trajectory_) {
      return result;
    }

    discard_joint_state_evidence();
    result = validate_initial_positions();
    if (result != ExitCode::success) {
      return result;
    }

    const auto commands = smoke_trajectory_contract();

    if (commands.size() != 3U) {
      RCLCPP_ERROR(get_logger(), "smoke trajectory must define rail, arm, and gripper commands");
      return ExitCode::invalid_configuration;
    }
    // Rail and arm run together, as MoveIt does; the UR10e velocity motor saturation only
    // appears while the rail accelerates.
    result = execute_concurrently(commands.at(0U), commands.at(1U));
    if (result != ExitCode::success) {
      return result;
    }
    result = execute(commands.at(2U));
    if (result != ExitCode::success) {
      return result;
    }

    std::vector<JointTarget> final_targets;
    for (const auto & command : commands) {
      final_targets.insert(final_targets.end(), command.targets.begin(), command.targets.end());
    }
    result = validate_final_positions(final_targets);
    return result == ExitCode::success ? validate_nontrivial_motion() : result;
  }

private:
  ExitCode validate_controllers()
  {
    const auto config = make_controller_probe_config(
      startup_timeout_.count(),
      controller_response_attempt_timeout_sec_);
    if (!config.valid()) {
      RCLCPP_ERROR(
        get_logger(), "invalid controller probe configuration: %s",
        config.detail.c_str());
      return ExitCode::invalid_configuration;
    }

    RclcppControllerProbeTransport transport(shared_from_this(), list_controllers_);
    const ControllerContractProbe probe(config.config,
      []() {return std::chrono::steady_clock::now();});
    const auto result = probe.run(transport);
    for (const auto & error : result.contract_errors) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    }
    if (result.cleanup_contentions > 0U) {
      RCLCPP_WARN(
        get_logger(), "controller probe observed %zu request-cleanup contention event(s)",
        result.cleanup_contentions);
    }
    switch (result.status) {
      case ControllerProbeStatus::kHealthy:
        RCLCPP_INFO(
          get_logger(), "controller contract is healthy after %zu request(s)",
          result.request_attempts);
        return ExitCode::success;
      case ControllerProbeStatus::kServiceTimeout:
        RCLCPP_ERROR(get_logger(), "controller manager did not become available before timeout");
        return ExitCode::controller_service_timeout;
      case ControllerProbeStatus::kResponseTimeout:
        RCLCPP_ERROR(
          get_logger(), "controller manager did not answer after %zu bounded request(s)",
          result.request_attempts);
        return ExitCode::controller_response_timeout;
      case ControllerProbeStatus::kContractFailure:
        RCLCPP_ERROR(
          get_logger(), "controller contract remained unhealthy after %zu request(s)",
          result.request_attempts);
        return ExitCode::controller_contract_failure;
      case ControllerProbeStatus::kInterrupted:
        RCLCPP_ERROR(get_logger(), "controller discovery was interrupted by context shutdown");
        return ExitCode::controller_interrupted;
      case ControllerProbeStatus::kTransportFailure:
        RCLCPP_ERROR(get_logger(), "controller discovery transport failed");
        return ExitCode::controller_transport_failure;
      case ControllerProbeStatus::kInvalidConfiguration:
        RCLCPP_ERROR(get_logger(), "controller discovery rejected its timeout configuration");
        return ExitCode::invalid_configuration;
    }
    RCLCPP_ERROR(get_logger(), "controller discovery returned an unknown result");
    return ExitCode::controller_transport_failure;
  }

  ExitCode execute(const SmokeTrajectoryCommand & command)
  {
    return execute_on_node(shared_from_this(), command);
  }

  ExitCode execute_concurrently(
    const SmokeTrajectoryCommand & first,
    const SmokeTrajectoryCommand & second)
  {
    const auto run = [this](const SmokeTrajectoryCommand & command) {
      const auto worker = std::make_shared<rclcpp::Node>(
        "control_smoke_test_" + command.controller + "_worker",
        rclcpp::NodeOptions().use_global_arguments(false));
      return execute_on_node(worker, command);
    };
    auto first_result = std::async(std::launch::async, run, std::cref(first));
    auto second_result = std::async(std::launch::async, run, std::cref(second));
    const auto first_status = first_result.get();
    const auto second_status = second_result.get();
    return first_status != ExitCode::success ? first_status : second_status;
  }

  ExitCode execute_on_node(
    const rclcpp::Node::SharedPtr & node,
    const SmokeTrajectoryCommand & command)
  {
    const std::string action_name = "/" + command.controller + "/follow_joint_trajectory";
    auto client = rclcpp_action::create_client<FollowJointTrajectory>(node, action_name);
    if (!client->wait_for_action_server(startup_timeout_)) {
      RCLCPP_ERROR(node->get_logger(), "%s action server timed out", command.controller.c_str());
      return ExitCode::action_server_timeout;
    }

    FollowJointTrajectory::Goal goal;
    trajectory_msgs::msg::JointTrajectoryPoint point;
    for (const auto & target : command.targets) {
      goal.trajectory.joint_names.push_back(target.name);
      point.positions.push_back(target.position);
    }
    point.time_from_start.sec = static_cast<std::int32_t>(command.duration.count());
    goal.trajectory.points.push_back(std::move(point));

    auto goal_future = client->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(node, goal_future, action_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "%s goal response timed out", command.controller.c_str());
      return ExitCode::goal_response_timeout;
    }
    const auto goal_handle = goal_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(
        node->get_logger(), "%s rejected the trajectory goal",
        command.controller.c_str());
      return ExitCode::goal_rejected;
    }

    auto result_future = client->async_get_result(goal_handle);
    const auto result_timeout =
      command.duration + std::chrono::duration_cast<std::chrono::seconds>(action_timeout_);
    if (rclcpp::spin_until_future_complete(node, result_future, result_timeout) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(
        node->get_logger(), "%s trajectory result timed out",
        command.controller.c_str());
      return ExitCode::result_timeout;
    }

    const GoalHandle::WrappedResult wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
      wrapped.result->error_code != FollowJointTrajectory::Result::SUCCESSFUL)
    {
      const int error_code = wrapped.result ? wrapped.result->error_code : 0;
      const std::string error = wrapped.result ? wrapped.result->error_string : "missing result";
      RCLCPP_ERROR(
        node->get_logger(), "%s execution failed: action=%d controller=%d %s",
        command.controller.c_str(), static_cast<int>(wrapped.code), error_code,
        error.c_str());
      return ExitCode::execution_failure;
    }
    RCLCPP_INFO(node->get_logger(), "%s trajectory succeeded", command.controller.c_str());
    return ExitCode::success;
  }

  void discard_joint_state_evidence()
  {
    std::scoped_lock lock(joint_state_mutex_);
    latest_positions_.clear();
    received_joint_state_ = false;
  }

  ExitCode validate_initial_positions()
  {
    const auto targets = smoke_initial_state_contract();
    const auto deadline = std::chrono::steady_clock::now() + action_timeout_;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      {
        std::scoped_lock lock(joint_state_mutex_);
        if (received_joint_state_ && validate_joint_positions(latest_positions_, targets).empty()) {
          initial_positions_ = latest_positions_;
          RCLCPP_INFO(
            get_logger(),
            "fresh joint state confirms interior simulated finger positions");
          return ExitCode::success;
        }
      }
      std::this_thread::sleep_for(20ms);
    }

    std::scoped_lock lock(joint_state_mutex_);
    if (!received_joint_state_) {
      RCLCPP_ERROR(get_logger(), "no fresh joint state was received after controller readiness");
      return ExitCode::initial_joint_state_timeout;
    }
    for (const auto & error : validate_joint_positions(latest_positions_, targets)) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    }
    return ExitCode::initial_position_mismatch;
  }

  ExitCode validate_final_positions(const std::vector<JointTarget> & targets)
  {
    const auto deadline = std::chrono::steady_clock::now() + action_timeout_;
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(shared_from_this());
      {
        std::scoped_lock lock(joint_state_mutex_);
        if (received_joint_state_) {
          const auto errors = validate_joint_positions(latest_positions_, targets);
          if (errors.empty()) {
            RCLCPP_INFO(get_logger(), "all commanded joints reached their smoke-test targets");
            return ExitCode::success;
          }
        }
      }
      std::this_thread::sleep_for(20ms);
    }

    std::scoped_lock lock(joint_state_mutex_);
    if (!received_joint_state_) {
      RCLCPP_ERROR(get_logger(), "no joint state was received before timeout");
      return ExitCode::joint_state_timeout;
    }
    const auto errors = validate_joint_positions(latest_positions_, targets);
    for (const auto & error : errors) {
      RCLCPP_ERROR(get_logger(), "%s", error.c_str());
    }
    return ExitCode::position_mismatch;
  }

  ExitCode validate_nontrivial_motion()
  {
    const std::map<std::string, double> minimum_displacement{
      {"rail_joint", 0.2},
      {"shoulder_lift_joint", 0.8},
      {"wrist_1_joint", 0.8},
    };
    std::scoped_lock lock(joint_state_mutex_);
    for (const auto &[name, minimum] : minimum_displacement) {
      const auto initial = initial_positions_.find(name);
      const auto final = latest_positions_.find(name);
      if (initial == initial_positions_.end() || final == latest_positions_.end()) {
        RCLCPP_ERROR(get_logger(), "missing displacement evidence for %s", name.c_str());
        return ExitCode::insufficient_motion;
      }
      const double displacement = std::abs(final->second - initial->second);
      if (displacement < minimum) {
        RCLCPP_ERROR(
          get_logger(), "%s moved only %.6f; required at least %.6f", name.c_str(),
          displacement, minimum);
        return ExitCode::insufficient_motion;
      }
      RCLCPP_INFO(get_logger(), "%s measured displacement %.6f", name.c_str(), displacement);
    }
    return ExitCode::success;
  }

  std::chrono::duration<double> startup_timeout_;
  double controller_response_attempt_timeout_sec_;
  std::chrono::duration<double> action_timeout_;
  bool execute_trajectory_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_;
  std::mutex joint_state_mutex_;
  std::map<std::string, double> initial_positions_;
  std::map<std::string, double> latest_positions_;
  bool received_joint_state_{false};
};

}  // namespace
}  // namespace restocker_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node = std::make_shared<restocker_control::ControlSmokeTest>();
  const auto result = node->run();
  rclcpp::shutdown();
  return static_cast<int>(result);
}
