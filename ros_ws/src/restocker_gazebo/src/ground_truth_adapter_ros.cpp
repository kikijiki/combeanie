// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_gazebo/ground_truth_adapter_ros.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>

namespace restocker_gazebo
{

GroundTruthAdapter::GroundTruthAdapter(const rclcpp::NodeOptions & options)
: Node("ground_truth_adapter", options)
{
  const std::string config_path = declare_parameter<std::string>("scenario_config", "");
  if (config_path.empty()) {
    throw std::invalid_argument("scenario_config parameter must not be empty");
  }
  config_ = load_ground_truth_config(config_path);
  const std::string workcell_geometry_path =
    declare_parameter<std::string>("workcell_geometry", "");
  const std::string product_catalog_path =
    declare_parameter<std::string>("product_catalog", "");
  if (workcell_geometry_path.empty() || product_catalog_path.empty()) {
    throw std::invalid_argument(
            "workcell_geometry and product_catalog parameters must not be empty");
  }
  lane_config_ = load_lane_evidence_config(
    workcell_geometry_path, product_catalog_path, config_);
  const std::string output_topic =
    declare_parameter<std::string>("output_topic", "/perception/object_observations");
  const std::string lane_output_topic =
    declare_parameter<std::string>(
    "lane_output_topic", "/perception/ground_truth/lane_observations");
  if (output_topic.empty() || lane_output_topic.empty()) {
    throw std::invalid_argument("observation output topics must not be empty");
  }
  const double maximum_publish_rate_hz =
    declare_parameter<double>("maximum_publish_rate_hz", 10.0);
  if (!std::isfinite(maximum_publish_rate_hz) || maximum_publish_rate_hz <= 0.0) {
    throw std::invalid_argument("maximum_publish_rate_hz must be finite and positive");
  }
  minimum_publish_period_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / maximum_publish_rate_hz));
  if (minimum_publish_period_.count() <= 0) {
    throw std::invalid_argument("maximum_publish_rate_hz is too large for nanosecond cadence");
  }

  publisher_ = create_publisher<restocker_interfaces::msg::ObjectObservation>(
    output_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
  lane_publisher_ = create_publisher<restocker_interfaces::msg::LaneObservation>(
    lane_output_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
  const std::function<void(const gz::msgs::Pose_V &)> on_pose_sample =
    [this](const gz::msgs::Pose_V & sample) {
      static_cast<void>(deliver_pose_sample(sample));
    };
  const bool subscribed = gz_node_.Subscribe(config_.pose_topic, on_pose_sample);
  if (!subscribed) {
    throw std::runtime_error("failed to subscribe to Gazebo topic " + config_.pose_topic);
  }
  RCLCPP_INFO(
    get_logger(),
    "subscribed to %s for %zu products and %zu lanes; publishing %s and %s",
    config_.pose_topic.c_str(), config_.products.size(), lane_config_.lanes.size(),
    output_topic.c_str(), lane_output_topic.c_str());
}

GroundTruthAdapter::~GroundTruthAdapter()
{
  // Close publishing here, before member destruction. The Gazebo transport thread keeps
  // delivering pose samples after `rclcpp::spin` returns, until `gz_node_` is destroyed. Without
  // this, publishers are finalised while `deliver_pose_sample` may still publish through a freed
  // `rcl_publisher_t` (use-after-free, seen as SIGSEGV at full-suite teardown).
  close_publishing();
}

void GroundTruthAdapter::close_publishing()
{
  // Unsubscribe before taking `publish_mutex_`. A delivery in flight holds `publish_mutex_` and
  // may hold transport's handler lock, so the reverse order could deadlock.
  static_cast<void>(gz_node_.Unsubscribe(config_.pose_topic));
  const std::scoped_lock lock(publish_mutex_);
  publishing_ = false;
}

bool GroundTruthAdapter::deliver_pose_sample(const gz::msgs::Pose_V & sample)
{
  const std::scoped_lock lock(publish_mutex_);
  if (!publishing_) {
    return false;
  }
  if (!cadence_accepts(sample)) {
    return false;
  }
  for (auto & observation : convert_pose_sample(sample, config_)) {
    if (observation.status != restocker_interfaces::msg::ObjectObservation::STATUS_OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ground-truth observation rejected: %s",
        observation.status_detail.c_str());
    }
    publisher_->publish(std::move(observation));
  }
  for (auto & observation : convert_lane_evidence_sample(sample, lane_config_)) {
    if (observation.status != restocker_interfaces::msg::LaneObservation::STATUS_OK) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "ground-truth lane observation rejected: %s",
        observation.status_detail.c_str());
    }
    lane_publisher_->publish(std::move(observation));
  }
  return true;
}

bool GroundTruthAdapter::cadence_accepts(const gz::msgs::Pose_V & sample)
{
  if (!sample.has_header() || !sample.header().has_stamp()) {
    return true;
  }
  const auto & stamp = sample.header().stamp();
  if (stamp.sec() < 0 || stamp.nsec() < 0 || stamp.nsec() >= 1'000'000'000 ||
    stamp.sec() > std::numeric_limits<std::int32_t>::max())
  {
    return true;
  }
  const auto sample_time = std::chrono::seconds(stamp.sec()) +
    std::chrono::nanoseconds(stamp.nsec());
  std::scoped_lock lock(cadence_mutex_);
  if (last_published_time_ && sample_time >= *last_published_time_ &&
    sample_time - *last_published_time_ < minimum_publish_period_)
  {
    return false;
  }
  if (!last_published_time_ || sample_time > *last_published_time_) {
    last_published_time_ = sample_time;
  }
  return true;
}

}  // namespace restocker_gazebo
