// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <mutex>
#include <optional>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>

#include "restocker_gazebo/ground_truth_conversion.hpp"
#include "restocker_gazebo/lane_evidence.hpp"

namespace restocker_gazebo
{

/// Republishes Gazebo pose samples as simulator-truth object and lane observations.
///
/// The subscription lives on a Gazebo transport thread, not the ROS executor: `gz::transport`
/// owns a receive thread that calls `deliver_pose_sample` directly, and `rclcpp::spin` returning
/// does not stop it. The publishers it writes to are members of this node and are finalised when
/// it is destroyed, so the two lifetimes must be ordered. `close_publishing` does that and the
/// destructor calls it; see the comments there.
class GroundTruthAdapter final : public rclcpp::Node
{
public:
  explicit GroundTruthAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GroundTruthAdapter() override;

  GroundTruthAdapter(const GroundTruthAdapter &) = delete;
  GroundTruthAdapter & operator=(const GroundTruthAdapter &) = delete;
  GroundTruthAdapter(GroundTruthAdapter &&) = delete;
  GroundTruthAdapter & operator=(GroundTruthAdapter &&) = delete;

  /// Convert one Gazebo pose sample and publish the observations it yields.
  ///
  /// Returns true when the sample reached the publishers, and false when it was dropped: either
  /// the cadence limiter rejected it, or publishing has been closed. This is the body the
  /// Gazebo subscription invokes, and it is public so a test can drive it without a simulator.
  bool deliver_pose_sample(const gz::msgs::Pose_V & sample);

  /// Stop delivering samples to the publishers, and wait for any delivery already running.
  ///
  /// Idempotent. After this returns, `deliver_pose_sample` publishes nothing and no other
  /// thread is inside it, so the publishers can be destroyed safely.
  void close_publishing();

private:
  [[nodiscard]] bool cadence_accepts(const gz::msgs::Pose_V & sample);

  GroundTruthConfig config_;
  LaneEvidenceConfig lane_config_;
  std::chrono::nanoseconds minimum_publish_period_{std::chrono::milliseconds(100)};
  std::mutex cadence_mutex_;
  std::optional<std::chrono::nanoseconds> last_published_time_;
  // Held for the whole of `deliver_pose_sample`, so `close_publishing` blocking on it proves no
  // delivery is in flight.
  std::mutex publish_mutex_;
  bool publishing_{true};
  rclcpp::Publisher<restocker_interfaces::msg::ObjectObservation>::SharedPtr publisher_;
  rclcpp::Publisher<restocker_interfaces::msg::LaneObservation>::SharedPtr lane_publisher_;
  // Declared last so it is destroyed first: unsubscribing before the publishers are finalised is
  // the ordering the destructor asks for, kept as a structural backstop.
  gz::transport::Node gz_node_;
};

}  // namespace restocker_gazebo
