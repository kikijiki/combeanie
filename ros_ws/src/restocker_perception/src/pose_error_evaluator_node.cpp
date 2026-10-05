// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_perception/pose_error_evaluation.hpp"

namespace restocker_perception
{
namespace
{

using AttachmentMessage = restocker_interfaces::msg::SimulationAttachmentState;
using ObservationMessage = restocker_interfaces::msg::ObjectObservation;
using SampleMessage = restocker_interfaces::msg::PoseErrorSample;

[[nodiscard]] FramedPose framed_pose(const ObservationMessage & observation)
{
  FramedPose pose;
  pose.frame_id = observation.header.frame_id;
  tf2::fromMsg(observation.pose.pose, pose.pose);
  for (std::size_t row = 0; row < 6; ++row) {
    for (std::size_t column = 0; column < 6; ++column) {
      pose.covariance(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(column)) =
        observation.pose.covariance[row * 6 + column];
    }
  }
  return pose;
}

}  // namespace

// Measures the perception pipeline against simulator ground truth. It is the only subscriber to
// the ground-truth topic, and its PoseErrorSample output must not be read on the execution path.
//
// Association is spatial, not by identity: pairing by name would report zero error for a pipeline
// that copied the name. An estimate is matched to the nearest ground-truth object of the same
// product class within a gate (association_radius_m) well below the 0.40 m product spacing, and
// the match is latched, so a pose that drifts is reported as a large error and not re-associated.
class PoseErrorEvaluatorNode final : public rclcpp::Node
{
public:
  PoseErrorEvaluatorNode()
  : Node("pose_error_evaluator"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto estimate_topic =
      declare_parameter<std::string>("estimate_topic", "/perception/object_observations");
    // Optional second estimate stream: the tray overview duty publishes its candidates on a
    // topic the world state does not ingest, and the range breakout is meaningless if the
    // overview-range estimates never reach the evaluator. Empty disables the subscription, which
    // is what the overhead pipeline wants (one estimate stream).
    const auto candidate_topic = declare_parameter<std::string>("candidate_topic", "");
    const auto ground_truth_topic = declare_parameter<std::string>(
      "ground_truth_topic", "/perception/ground_truth/object_observations");
    const auto attachment_topic = declare_parameter<std::string>(
      "attachment_state_topic", "/simulation_attachment/state");
    const auto sample_topic =
      declare_parameter<std::string>("sample_topic", "/perception/pose_error");
    camera_optical_frame_ =
      declare_parameter<std::string>("camera_optical_frame", "");
    if (estimate_topic.empty() || ground_truth_topic.empty() || attachment_topic.empty() ||
      sample_topic.empty())
    {
      throw std::invalid_argument("evaluator topics must not be empty");
    }
    if (estimate_topic == ground_truth_topic || candidate_topic == ground_truth_topic) {
      throw std::invalid_argument(
              "the estimate and ground-truth topics must differ, or the pipeline would be "
              "measured against itself");
    }
    association_radius_m_ = declare_parameter<double>("association_radius_m", 0.15);
    const auto pairing_skew_ms = declare_parameter<double>("maximum_pairing_skew_ms", 200.0);
    const auto held_settle_ms = declare_parameter<double>("held_settle_ms", 1000.0);
    const auto report_period_s = declare_parameter<double>("report_period_s", 20.0);
    if (!(association_radius_m_ > 0.0) || !std::isfinite(pairing_skew_ms) ||
      pairing_skew_ms < 0.0 || !std::isfinite(held_settle_ms) || held_settle_ms < 0.0 ||
      !std::isfinite(report_period_s) || report_period_s <= 0.0)
    {
      throw std::invalid_argument("evaluator tolerances must be finite and non-negative");
    }
    policy_.maximum_pairing_skew = rclcpp::Duration(
      std::chrono::nanoseconds(static_cast<std::int64_t>(pairing_skew_ms * 1.0e6)));
    held_settle_ = rclcpp::Duration(
      std::chrono::nanoseconds(static_cast<std::int64_t>(held_settle_ms * 1.0e6)));

    publisher_ = create_publisher<SampleMessage>(
      sample_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    ground_truth_subscription_ = create_subscription<ObservationMessage>(
      ground_truth_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](ObservationMessage::ConstSharedPtr message) {
        if (message->status == ObservationMessage::STATUS_OK) {
          ground_truth_[message->source_object_id] = *message;
        }
      });
    attachment_subscription_ = create_subscription<AttachmentMessage>(
      attachment_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](AttachmentMessage::ConstSharedPtr message) {on_attachment(*message);});
    estimate_subscription_ = create_subscription<ObservationMessage>(
      estimate_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](ObservationMessage::ConstSharedPtr message) {on_estimate(*message);});
    if (!candidate_topic.empty()) {
      candidate_subscription_ = create_subscription<ObservationMessage>(
        candidate_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
        [this](ObservationMessage::ConstSharedPtr message) {on_estimate(*message);});
    }
    report_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(report_period_s)),
      [this]() {report();});

    if (camera_optical_frame_.empty()) {
      RCLCPP_INFO(
        get_logger(), "measuring %s against %s, publishing samples on %s",
        estimate_topic.c_str(), ground_truth_topic.c_str(), sample_topic.c_str());
    } else {
      RCLCPP_INFO(
        get_logger(),
        "measuring %s against %s, publishing samples on %s, breaking range out from %s",
        estimate_topic.c_str(), ground_truth_topic.c_str(), sample_topic.c_str(),
        camera_optical_frame_.c_str());
    }
    if (!candidate_topic.empty()) {
      RCLCPP_INFO(
        get_logger(), "also measuring candidate estimates from %s", candidate_topic.c_str());
    }
  }

private:
  struct Statistics
  {
    std::size_t samples{0};
    double total_translation_m{0.0};
    double maximum_translation_m{0.0};
    // Separate from `samples`: a zero axis error over zero samples is not a small error.
    std::size_t axis_samples{0};
    double maximum_axis_rad{0.0};
  };

  void on_attachment(const AttachmentMessage & message)
  {
    // The DetachableJoint stops Gazebo updating the model's pose, and it exists briefly before and
    // after the phase settles, so either signal freezes ground truth.
    const bool frozen =
      message.joint_observed || message.phase != AttachmentMessage::PHASE_DETACHED;
    if (frozen) {
      if (!message.object_source_id.empty()) {
        held_.insert(message.object_source_id);
        released_at_.erase(message.object_source_id);
      }
      return;
    }
    // Not held. A settled detached publication with no identity releases every hold; one that
    // names an object releases only that object. Holds are per object because a release and the
    // next grasp can fall within one settle window.
    if (message.object_source_id.empty()) {
      for (const auto & source_object_id : held_) {
        released_at_[source_object_id] = now();
      }
      held_.clear();
      return;
    }
    if (held_.erase(message.object_source_id) > 0) {
      // The first ground-truth sample after a release can still carry the frozen value, so pairing
      // stays suppressed for held_settle_.
      released_at_[message.object_source_id] = now();
    }
  }

  [[nodiscard]] bool ground_truth_frozen(const std::string & source_object_id) const
  {
    if (held_.contains(source_object_id)) {
      return true;
    }
    const auto released = released_at_.find(source_object_id);
    return released != released_at_.end() && now() - released->second < held_settle_;
  }

  [[nodiscard]] std::optional<std::string> associate(const ObservationMessage & estimate)
  {
    if (const auto latched = association_.find(estimate.source_object_id);
      latched != association_.end())
    {
      return latched->second;
    }
    const Eigen::Vector3d position(
      estimate.pose.pose.position.x, estimate.pose.pose.position.y,
      estimate.pose.pose.position.z);
    std::optional<std::string> best;
    double best_distance = association_radius_m_;
    for (const auto & [source_object_id, truth] : ground_truth_) {
      if (truth.product_class != estimate.product_class ||
        truth.header.frame_id != estimate.header.frame_id)
      {
        continue;
      }
      // One ground-truth object serves one estimate, so errors are not conflated.
      if (std::ranges::any_of(
          association_, [&source_object_id](const auto & entry) {
            return entry.second == source_object_id;
          }))
      {
        continue;
      }
      const Eigen::Vector3d truth_position(
        truth.pose.pose.position.x, truth.pose.pose.position.y, truth.pose.pose.position.z);
      const double distance = (truth_position - position).norm();
      if (distance <= best_distance) {
        best_distance = distance;
        best = source_object_id;
      }
    }
    if (best.has_value()) {
      association_.emplace(estimate.source_object_id, *best);
      RCLCPP_INFO(
        get_logger(), "associated estimate '%s' with ground truth '%s' at %.4f m",
        estimate.source_object_id.c_str(), best->c_str(), best_distance);
    }
    return best;
  }

  [[nodiscard]] double observation_range(const ObservationMessage & estimate) const
  {
    if (camera_optical_frame_.empty() || estimate.header.frame_id.empty()) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        estimate.header.frame_id, camera_optical_frame_,
        rclcpp::Time(estimate.header.stamp, RCL_ROS_TIME),
        tf2::durationFromSec(0.0));
      const Eigen::Vector3d camera = tf2::transformToEigen(transform).translation();
      const Eigen::Vector3d product(
        estimate.pose.pose.position.x, estimate.pose.pose.position.y,
        estimate.pose.pose.position.z);
      return (product - camera).norm();
    } catch (const tf2::TransformException &) {
      return std::numeric_limits<double>::quiet_NaN();
    }
  }

  void on_estimate(const ObservationMessage & estimate)
  {
    if (estimate.status != ObservationMessage::STATUS_OK) {
      return;
    }
    PoseErrorInputs inputs;
    inputs.backend_name = estimate.backend_name;
    inputs.backend_version = estimate.backend_version;
    inputs.estimate = framed_pose(estimate);
    inputs.estimate_stamp = rclcpp::Time(estimate.header.stamp, RCL_ROS_TIME);
    // Read from the estimate's covariance, not ground truth, which always knows the orientation.
    inputs.orientation_estimated = declares_axis_estimate(inputs.estimate.covariance);
    inputs.observation_range_m = observation_range(estimate);

    const auto matched = associate(estimate);
    // Reported under the ground-truth identity (the one found in the scenario) when associated;
    // unassociated estimates keep their own.
    inputs.source_object_id = matched.value_or(estimate.source_object_id);
    if (matched.has_value()) {
      const auto & truth = ground_truth_.at(*matched);
      inputs.ground_truth = framed_pose(truth);
      inputs.ground_truth_stamp = rclcpp::Time(truth.header.stamp, RCL_ROS_TIME);
      inputs.ground_truth_frozen = ground_truth_frozen(*matched);
    }

    const auto sample = evaluate_pose_error(inputs, policy_);
    publisher_->publish(sample);
    if (sample.status != SampleMessage::STATUS_OK) {
      return;
    }
    auto & by_object = statistics_[inputs.source_object_id];
    ++by_object.samples;
    by_object.total_translation_m += sample.translation_error_m;
    by_object.maximum_translation_m =
      std::max(by_object.maximum_translation_m, sample.translation_error_m);
    if (sample.orientation_status == SampleMessage::ORIENTATION_AXIS_ESTIMATED) {
      ++by_object.axis_samples;
      by_object.maximum_axis_rad =
        std::max(by_object.maximum_axis_rad, sample.axis_error_rad);
    }
    auto & by_range = range_statistics_[classify_observation_range(sample.observation_range_m)];
    ++by_range.samples;
    by_range.total_translation_m += sample.translation_error_m;
    by_range.maximum_translation_m =
      std::max(by_range.maximum_translation_m, sample.translation_error_m);
  }

  void report()
  {
    for (const auto & [source_object_id, statistics] : statistics_) {
      if (statistics.samples == 0) {
        continue;
      }
      // A 0 rad axis error would look the same for an orientation-blind and a perfect backend.
      const std::string orientation = statistics.axis_samples == 0 ?
        std::string("orientation not estimated by this backend, so no axis error is measured") :
        "worst axis error " + std::to_string(statistics.maximum_axis_rad) + " rad over " +
        std::to_string(statistics.axis_samples) + " samples";
      RCLCPP_INFO(
        get_logger(),
        "%s: %zu samples, mean translation error %.2f mm, worst %.2f mm; %s",
        source_object_id.c_str(), statistics.samples,
        1000.0 * statistics.total_translation_m / static_cast<double>(statistics.samples),
        1000.0 * statistics.maximum_translation_m, orientation.c_str());
    }
    for (const auto band : {
        ObservationRangeBand::Confirm, ObservationRangeBand::Overview,
        ObservationRangeBand::Far, ObservationRangeBand::Unknown})
    {
      const auto found = range_statistics_.find(band);
      if (found == range_statistics_.end() || found->second.samples == 0) {
        continue;
      }
      const auto & statistics = found->second;
      RCLCPP_INFO(
        get_logger(),
        "range %s: %zu samples, mean translation error %.2f mm, worst %.2f mm",
        observation_range_band_name(band), statistics.samples,
        1000.0 * statistics.total_translation_m / static_cast<double>(statistics.samples),
        1000.0 * statistics.maximum_translation_m);
    }
  }

  PoseErrorPolicy policy_;
  double association_radius_m_{0.15};
  rclcpp::Duration held_settle_{std::chrono::milliseconds(1000)};
  std::string camera_optical_frame_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::map<std::string, ObservationMessage> ground_truth_;
  std::map<std::string, std::string> association_;
  std::map<std::string, Statistics> statistics_;
  std::map<ObservationRangeBand, Statistics> range_statistics_;
  std::set<std::string> held_;
  std::map<std::string, rclcpp::Time> released_at_;
  rclcpp::Subscription<ObservationMessage>::SharedPtr estimate_subscription_;
  rclcpp::Subscription<ObservationMessage>::SharedPtr candidate_subscription_;
  rclcpp::Subscription<ObservationMessage>::SharedPtr ground_truth_subscription_;
  rclcpp::Subscription<AttachmentMessage>::SharedPtr attachment_subscription_;
  rclcpp::Publisher<SampleMessage>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr report_timer_;
};

}  // namespace restocker_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_perception::PoseErrorEvaluatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pose_error_evaluator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
