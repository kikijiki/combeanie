// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/robot_telemetry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "restocker_control/joint_state_telemetry.hpp"

namespace restocker_control
{
namespace
{

constexpr std::int64_t kMaximumQosDepth = 10'000;

class JointStateTelemetryAdapter final : public rclcpp::Node
{
public:
  JointStateTelemetryAdapter()
  : rclcpp::Node("joint_state_telemetry_adapter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/joint_states");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/robot_telemetry");
    source_id_ = declare_parameter<std::string>(
      "source_id", "restocker_control/joint_state_adapter");
    const auto output_qos_depth = declare_parameter<std::int64_t>("output_qos_depth", 10);

    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must be nonempty");
    }
    if (!is_valid_telemetry_source_id(source_id_)) {
      throw std::invalid_argument("source_id does not satisfy the normalized identity grammar");
    }
    if (output_qos_depth <= 0 || output_qos_depth > kMaximumQosDepth) {
      throw std::invalid_argument("output_qos_depth must be in [1, 10000]");
    }

    publisher_ = create_publisher<restocker_interfaces::msg::RobotTelemetry>(
      output_topic,
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(output_qos_depth)))
      .reliable()
      .durability_volatile());
    subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&JointStateTelemetryAdapter::on_joint_state, this, std::placeholders::_1));
    immutable_parameters_callback_ = add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = std::all_of(
          parameters.begin(), parameters.end(), [](const rclcpp::Parameter & parameter) {
            return parameter.get_name().starts_with("qos_overrides.");
          });
        if (!result.successful) {
          result.reason = "joint-state telemetry adapter parameters are immutable at runtime";
        }
        return result;
      });

    RCLCPP_INFO(
      get_logger(), "projecting controller telemetry from %s to %s as source %s",
      input_topic.c_str(), output_topic.c_str(), source_id_.c_str());
  }

private:
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr sample)
  {
    auto projected = project_joint_state_telemetry(*sample, source_id_, last_published_stamp_);
    if (!projected) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "joint-state telemetry rejection [%s]: %s",
        to_string(projected.error().code).data(), projected.error().detail.c_str());
      return;
    }

    try {
      publisher_->publish(projected.value());
      last_published_stamp_ = sample->header.stamp;
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "failed to publish normalized robot telemetry: %s", error.what());
    }
  }

  std::string source_id_;
  std::optional<builtin_interfaces::msg::Time> last_published_stamp_;
  rclcpp::Publisher<restocker_interfaces::msg::RobotTelemetry>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    immutable_parameters_callback_;
};

}  // namespace
}  // namespace restocker_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_control::JointStateTelemetryAdapter>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("joint_state_telemetry_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
