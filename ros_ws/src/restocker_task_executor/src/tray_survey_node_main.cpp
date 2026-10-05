// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <exception>

#include <rclcpp/rclcpp.hpp>

#include "restocker_task_executor/tray_survey_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  int status = 0;
  try {
    // One worker thread runs the survey; the executor only services the server's non-blocking
    // callbacks, the child goal's responses, and the two observation streams.
    auto server = std::make_shared<restocker_task_executor::TraySurveyNode>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server->node());
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tray_survey"), "tray survey node failed: %s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
