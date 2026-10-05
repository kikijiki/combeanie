// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Pairs the camera's lane measurements against the simulator's own, and reports the
// disagreement as a distribution.
//
// The report is not a mean: an estimator that is excellent on an empty lane and useless on a full
// one has a fine mean and is unusable, and a lane confidence floor is to be set from this output.
// It is conditioned on how full the lane actually was, the variable the estimator's difficulty
// depends on. It also keeps the sign, because the directions differ in danger: reading a lane as
// emptier than it is opens the capacity gate on a lane with no room, while reading it as fuller
// only refuses work that could have been done.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/lane_depth_error_sample.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>

#include "restocker_perception/lane_depth_evaluation.hpp"

namespace restocker_perception
{
namespace
{

using LaneObservationMessage = restocker_interfaces::msg::LaneObservation;
using SampleMessage = restocker_interfaces::msg::LaneDepthErrorSample;

// Quantiles of a copy of the samples. Small vectors, reported at most every report period.
[[nodiscard]] double quantile(std::vector<double> values, double fraction)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t index = std::min(
    values.size() - 1U,
    static_cast<std::size_t>(fraction * static_cast<double>(values.size() - 1U) + 0.5));
  std::nth_element(
    values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index),
    values.end());
  return values[index];
}

struct Distribution
{
  std::vector<double> signed_error_m;

  void add(double error) {signed_error_m.push_back(error);}
  [[nodiscard]] bool empty() const noexcept {return signed_error_m.empty();}

  [[nodiscard]] std::string describe() const
  {
    std::vector<double> absolute;
    absolute.reserve(signed_error_m.size());
    for (const double value : signed_error_m) {
      absolute.push_back(std::abs(value));
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << signed_error_m.size() << " samples, signed "
           << "min " << quantile(signed_error_m, 0.0) * 1000.0 << " / p50 "
           << quantile(signed_error_m, 0.50) * 1000.0 << " / p95 "
           << quantile(signed_error_m, 0.95) * 1000.0 << " / max "
           << quantile(signed_error_m, 1.0) * 1000.0 << " mm, |error| p95 "
           << quantile(absolute, 0.95) * 1000.0 << " max " << quantile(absolute, 1.0) * 1000.0
           << " mm";
    return stream.str();
  }
};

class LaneDepthErrorEvaluatorNode : public rclcpp::Node
{
public:
  explicit LaneDepthErrorEvaluatorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lane_depth_error_evaluator", options)
  {
    const std::string measurement_topic =
      declare_parameter<std::string>("measurement_topic", "/perception/lane_observations");
    const std::string ground_truth_topic = declare_parameter<std::string>(
      "ground_truth_topic", "/perception/ground_truth/lane_observations");
    const std::string sample_topic =
      declare_parameter<std::string>("sample_topic", "/perception/lane_depth_error");
    if (measurement_topic.empty() || ground_truth_topic.empty() || sample_topic.empty()) {
      throw std::invalid_argument("lane depth evaluator topics must be named");
    }
    if (measurement_topic == ground_truth_topic) {
      throw std::invalid_argument(
              "the measurement and ground-truth topics must differ, or the producer would be "
              "measured against itself");
    }
    const double skew_ms = declare_parameter<double>("maximum_pairing_skew_ms", 500.0);
    report_period_s_ = declare_parameter<double>("report_period_s", 20.0);
    // Number of ground-truth fill buckets the distribution is conditioned on, as fractions of the
    // lane's usable depth. Ten shows a trend across the range without splitting a run's samples
    // so thinly that buckets are empty.
    bucket_count_ = static_cast<std::size_t>(declare_parameter<std::int64_t>("fill_buckets", 10));
    lane_depth_m_ = declare_parameter<double>("lane_depth_m", 0.85);
    if (!std::isfinite(skew_ms) || skew_ms < 0.0 || !std::isfinite(report_period_s_) ||
      report_period_s_ <= 0.0 || bucket_count_ == 0U || !std::isfinite(lane_depth_m_) ||
      lane_depth_m_ <= 0.0)
    {
      throw std::invalid_argument("lane depth evaluator parameters are invalid");
    }
    policy_.maximum_pairing_skew = rclcpp::Duration::from_seconds(skew_ms / 1000.0);
    buckets_.resize(bucket_count_);

    publisher_ = create_publisher<SampleMessage>(
      sample_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    ground_truth_subscription_ = create_subscription<LaneObservationMessage>(
      ground_truth_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](LaneObservationMessage::ConstSharedPtr message) {
        if (message->status == LaneObservationMessage::STATUS_OK && !message->lane_id.empty()) {
          ground_truth_[message->lane_id] = *message;
        }
      });
    measurement_subscription_ = create_subscription<LaneObservationMessage>(
      measurement_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      [this](LaneObservationMessage::ConstSharedPtr message) {on_measurement(*message);});
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(report_period_s_)), [this]() {report();});
    RCLCPP_INFO(
      get_logger(), "lane depth error evaluator: %s against %s, samples on %s",
      measurement_topic.c_str(), ground_truth_topic.c_str(), sample_topic.c_str());
  }

private:
  void on_measurement(const LaneObservationMessage & measurement)
  {
    LaneDepthErrorInputs inputs;
    inputs.measurement = measurement;
    const auto held = ground_truth_.find(measurement.lane_id);
    if (held != ground_truth_.end()) {
      inputs.ground_truth = held->second;
    }
    const SampleMessage sample = evaluate_lane_depth_error(inputs, policy_);
    publisher_->publish(sample);
    ++status_counts_[sample.status];

    // Only pairs that were compared enter the distribution. Both a usable measurement and one the
    // producer refused count as compared: the refused ones are needed to choose a coverage floor.
    const bool comparable = sample.status == SampleMessage::STATUS_OK ||
      sample.status == SampleMessage::STATUS_MEASUREMENT_UNUSABLE;
    if (!comparable) {
      return;
    }
    const double error = sample.available_depth_error_m;
    if (sample.status == SampleMessage::STATUS_OK) {
      admitted_.add(error);
      per_lane_[sample.lane_id].add(error);
      const double fill = std::clamp(
        1.0 - (sample.ground_truth_available_depth_m / lane_depth_m_), 0.0, 1.0);
      const std::size_t bucket = std::min(
        bucket_count_ - 1U,
        static_cast<std::size_t>(fill * static_cast<double>(bucket_count_)));
      buckets_[bucket].add(error);
    } else {
      refused_.add(error);
    }
  }

  void report()
  {
    if (admitted_.empty() && refused_.empty()) {
      return;
    }
    std::ostringstream stream;
    stream << "lane depth agreement -- admitted: " << admitted_.describe();
    if (!refused_.empty()) {
      stream << "; refused for coverage: " << refused_.describe();
    }
    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
    for (const auto & [lane, distribution] : per_lane_) {
      RCLCPP_INFO(
        get_logger(), "  %s: %s", lane.c_str(), distribution.describe().c_str());
    }
    for (std::size_t bucket = 0U; bucket < bucket_count_; ++bucket) {
      if (buckets_[bucket].empty()) {
        continue;
      }
      const double low = 100.0 * static_cast<double>(bucket) / static_cast<double>(bucket_count_);
      const double high =
        100.0 * static_cast<double>(bucket + 1U) / static_cast<double>(bucket_count_);
      RCLCPP_INFO(
        get_logger(), "  fill %3.0f-%3.0f%%: %s", low, high,
        buckets_[bucket].describe().c_str());
    }
    for (const auto & [status, count] : status_counts_) {
      RCLCPP_INFO(
        get_logger(), "  status %u: %zu samples", static_cast<unsigned>(status), count);
    }
  }

  LaneDepthErrorPolicy policy_;
  double report_period_s_{0.0};
  double lane_depth_m_{0.0};
  std::size_t bucket_count_{0U};

  std::map<std::string, LaneObservationMessage> ground_truth_;
  std::map<std::string, Distribution> per_lane_;
  std::vector<Distribution> buckets_;
  Distribution admitted_;
  Distribution refused_;
  std::map<std::uint8_t, std::size_t> status_counts_;

  rclcpp::Publisher<SampleMessage>::SharedPtr publisher_;
  rclcpp::Subscription<LaneObservationMessage>::SharedPtr measurement_subscription_;
  rclcpp::Subscription<LaneObservationMessage>::SharedPtr ground_truth_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace restocker_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(
      std::make_shared<restocker_perception::LaneDepthErrorEvaluatorNode>(rclcpp::NodeOptions()));
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lane_depth_error_evaluator"), "lane depth error evaluator failed: %s",
      error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
