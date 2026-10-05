// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "restocker_gazebo/ground_truth_adapter_ros.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto adapter = std::make_shared<restocker_gazebo::GroundTruthAdapter>();
    rclcpp::spin(adapter);
    // Close the Gazebo subscription before the last reference to the node goes away, so the
    // transport thread has stopped delivering by the time the publishers are finalised. The
    // destructor also does this; repeated here to keep the shutdown ordering visible.
    adapter->close_publishing();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ground_truth_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
