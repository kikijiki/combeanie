// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// The acquisition half of a lane survey: one depth frame, one lane, one LaneObservation.
//
// It is a separate node from the behaviour that aims the camera, and the split is not
// bureaucratic. The most important property this stage has to prove is that a camera which sees
// nothing does not report an empty lane, and a proof of that has to be able to run a camera which
// sees nothing. With the acquisition inside the surveying behaviour, exercising it would mean a
// planner, a controller, a simulator and a renderer in the process tree, and blinding the
// renderer in the middle of it; with the acquisition here, it means pointing depth_topic at a
// publisher of blank frames. Every consumer sees the same service and the same topic either way.
//
// This node never moves anything and never checks that the camera is aimed at the lane it was
// asked about. Aiming is the survey's job, and a measurement taken from the wrong place answers
// with a coverage that says so.

#include <span>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>
#include <restocker_interfaces/srv/acquire_lane_observation.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_perception/lane_depth_measurement.hpp"
#include "restocker_perception/survey_stations.hpp"

namespace restocker_perception
{
namespace
{

using AcquireLaneObservation = restocker_interfaces::srv::AcquireLaneObservation;
using LaneObservationMessage = restocker_interfaces::msg::LaneObservation;
using Image = sensor_msgs::msg::Image;
using CameraInfo = sensor_msgs::msg::CameraInfo;

class LaneObservationNode : public rclcpp::Node
{
public:
  explicit LaneObservationNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("lane_observation", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const std::string geometry_path = declare_parameter<std::string>("workcell_geometry", "");
    if (geometry_path.empty()) {
      throw std::invalid_argument("workcell_geometry must name the surveyed workcell geometry");
    }
    geometry_ = load_workcell_survey_geometry(geometry_path);

    const std::string depth_topic =
      declare_parameter<std::string>("depth_topic", "/wrist_camera/depth_image");
    const std::string camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/wrist_camera/camera_info");
    const std::string observation_topic =
      declare_parameter<std::string>("lane_observation_topic", "/perception/lane_observations");
    const std::string service_name = declare_parameter<std::string>(
      "acquire_service", "/perception/acquire_lane_observation");
    backend_name_ =
      declare_parameter<std::string>("backend_name", "wrist_depth_lane_survey");
    backend_version_ = declare_parameter<std::string>("backend_version", "1.0.0");
    if (depth_topic.empty() || camera_info_topic.empty() || observation_topic.empty() ||
      service_name.empty() || backend_name_.empty() || backend_version_.empty())
    {
      throw std::invalid_argument("lane observation topics, service and backend must be named");
    }

    transform_timeout_sec_ = declare_parameter<double>("transform_timeout_sec", 0.20);
    default_acquisition_timeout_sec_ =
      declare_parameter<double>("acquisition_timeout_sec", 2.0);
    // Below this the producer refuses to answer rather than answering weakly. The floor is stated
    // in restocker_perception/config/lane_survey.yaml beside the value.
    minimum_coverage_ = declare_parameter<double>("minimum_coverage", 0.90);
    const double floor_margin_m = declare_parameter<double>("floor_margin_m", 0.010);
    const double front_inset_m = declare_parameter<double>("front_inset_m", 0.010);
    const double obstruction_reach_m = declare_parameter<double>("obstruction_reach_m", 0.060);
    config_.minimum_depth_m = declare_parameter<double>("minimum_depth_m", 0.10);
    config_.maximum_depth_m = declare_parameter<double>("maximum_depth_m", 2.40);
    config_.pixel_stride =
      static_cast<std::uint32_t>(declare_parameter<std::int64_t>("pixel_stride", 2));
    config_.bin_width_m = declare_parameter<double>("bin_width_m", 0.020);
    config_.minimum_surface_bin_returns = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("minimum_surface_bin_returns", 12));
    config_.minimum_accepted_returns = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("minimum_accepted_returns", 200));
    config_.minimum_obstruction_returns = static_cast<std::size_t>(
      declare_parameter<std::int64_t>("minimum_obstruction_returns", 40));
    if (!std::isfinite(transform_timeout_sec_) || transform_timeout_sec_ < 0.0 ||
      !std::isfinite(default_acquisition_timeout_sec_) ||
      default_acquisition_timeout_sec_ <= 0.0 || !std::isfinite(minimum_coverage_) ||
      minimum_coverage_ < 0.0 || minimum_coverage_ > 1.0)
    {
      throw std::invalid_argument("lane observation timing and coverage parameters are invalid");
    }
    if (!valid_lane_depth_config(config_)) {
      throw std::invalid_argument("lane depth measurement parameters are invalid");
    }

    for (const LaneSurveyGeometry & lane : geometry_.lanes) {
      auto window = lane_depth_window(
        lane, geometry_.shelf, floor_margin_m, front_inset_m, obstruction_reach_m);
      if (!window) {
        throw std::invalid_argument(
                "lane " + lane.lane_id + " has no usable depth window: " +
                window.error().detail);
      }
      windows_.push_back(std::move(window.value()));
    }

    // The service handler blocks waiting for a frame, so it may not share a callback group with
    // the subscriptions it is waiting on: a mutually exclusive group would let the handler wait
    // for a delivery the executor is not permitted to make.
    service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    subscription_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = subscription_group_;

    const bool reliable = declare_parameter<bool>("reliable_sensor_qos", true);
    const rclcpp::QoS sensor_qos = reliable ?
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable() : rclcpp::QoS(rclcpp::SensorDataQoS());
    publisher_ = create_publisher<LaneObservationMessage>(
      observation_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());
    camera_info_subscription_ = create_subscription<CameraInfo>(
      camera_info_topic, sensor_qos,
      [this](CameraInfo::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        camera_info_ = std::move(message);
      },
      subscription_options);
    depth_subscription_ = create_subscription<Image>(
      depth_topic, sensor_qos,
      [this](Image::ConstSharedPtr message) {
        {
          std::scoped_lock lock(mutex_);
          depth_ = std::move(message);
        }
        arrivals_.notify_all();
      },
      subscription_options);
    service_ = create_service<AcquireLaneObservation>(
      service_name,
      [this](
        const AcquireLaneObservation::Request::SharedPtr request,
        AcquireLaneObservation::Response::SharedPtr response) {acquire(*request, *response);},
      rclcpp::ServicesQoS(), service_group_);

    RCLCPP_INFO(
      get_logger(),
      "lane observation server ready on %s: %zu lanes, depth %s, publishing %s",
      service_name.c_str(), windows_.size(), depth_topic.c_str(), observation_topic.c_str());
  }

private:
  [[nodiscard]] const LaneDepthWindow * window_for(const std::string & lane_id) const
  {
    const auto match = std::find_if(
      windows_.begin(), windows_.end(),
      [&lane_id](const LaneDepthWindow & window) {return window.lane_id == lane_id;});
    return match == windows_.end() ? nullptr : &*match;
  }

  // Wait for a depth image stamped at or after `not_before`, and the calibration that goes with
  // it. Returns nullopt when the budget runs out.
  [[nodiscard]] std::optional<std::pair<Image::ConstSharedPtr, CameraInfo::ConstSharedPtr>>
  await_frame(const rclcpp::Time & not_before, std::chrono::nanoseconds budget)
  {
    const auto deadline = std::chrono::steady_clock::now() + budget;
    std::unique_lock lock(mutex_);
    while (true) {
      if (depth_ && camera_info_) {
        const rclcpp::Time stamp(depth_->header.stamp, RCL_ROS_TIME);
        if (stamp.nanoseconds() > 0 && stamp >= not_before) {
          return std::make_pair(depth_, camera_info_);
        }
      }
      if (arrivals_.wait_until(lock, deadline) == std::cv_status::timeout) {
        return std::nullopt;
      }
    }
  }

  void acquire(
    const AcquireLaneObservation::Request & request, AcquireLaneObservation::Response & response)
  {
    if (request.lane_id.empty()) {
      response.outcome = AcquireLaneObservation::Response::OUTCOME_INVALID_REQUEST;
      response.detail = "no lane was named";
      return;
    }
    const LaneDepthWindow * window = window_for(request.lane_id);
    if (window == nullptr) {
      response.outcome = AcquireLaneObservation::Response::OUTCOME_UNKNOWN_LANE;
      response.detail = "no surveyed lane named " + request.lane_id;
      return;
    }

    const double requested_timeout =
      rclcpp::Duration(request.timeout).nanoseconds() > 0 ?
      rclcpp::Duration(request.timeout).seconds() : default_acquisition_timeout_sec_;
    const auto budget = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(requested_timeout));
    // A caller that names no instant takes the next frame the camera produces, never one already
    // buffered. Reading zero as "any frame answers" hands back whatever the previous consumer
    // left there: for the destination acquire that refreshes a lane after a release, that is a
    // frame rendered while the arm was still in the lane mouth, and its measurement then describes
    // that earlier instant rather than the retreat viewpoint the caller is standing at
    // (Milestone 10 §3, "which frame is measured is part of the measurement").
    rclcpp::Time not_before(request.not_before, RCL_ROS_TIME);
    if (not_before.nanoseconds() <= 0) {
      not_before = now();
    }
    const auto frame = await_frame(not_before, budget);
    if (!frame) {
      // Nothing is published. A consumer is told nothing about this lane rather than being told
      // it is empty, and the lane's existing evidence ages.
      response.outcome = AcquireLaneObservation::Response::OUTCOME_NO_FRAME;
      response.detail = "no depth frame stamped at or after the requested instant arrived in " +
        std::to_string(requested_timeout) + " s";
      RCLCPP_WARN(get_logger(), "%s: %s", request.lane_id.c_str(), response.detail.c_str());
      return;
    }
    const Image & depth = *frame->first;
    const CameraInfo & info = *frame->second;

    std::string detail;
    if (depth.encoding != "32FC1") {
      detail = "depth encoding " + depth.encoding + " is not 32FC1";
    } else if (depth.is_bigendian != 0U) {
      detail = "big-endian depth images are not supported";
    } else if (depth.step % sizeof(float) != 0U) {
      detail = "depth row step is not a whole number of float samples";
    } else if (info.width != depth.width || info.height != depth.height) {
      detail = "camera_info geometry does not match the depth image";
    } else if (info.header.frame_id != depth.header.frame_id) {
      detail = "camera_info frame " + info.header.frame_id + " is not the depth image's " +
        depth.header.frame_id;
    }
    if (!detail.empty()) {
      response.outcome = AcquireLaneObservation::Response::OUTCOME_UNUSABLE_FRAME;
      response.detail = std::move(detail);
      RCLCPP_WARN(get_logger(), "%s: %s", request.lane_id.c_str(), response.detail.c_str());
      return;
    }

    // At the frame's own stamp, never "now". The camera-to-lane transform now runs through the
    // arm's forward kinematics, so reading it at the wrong instant is reading a different pose of
    // a moving eye.
    Eigen::Isometry3d lane_from_optical;
    try {
      lane_from_optical = tf2::transformToEigen(
        tf_buffer_.lookupTransform(
          request.lane_id, depth.header.frame_id, tf2_ros::fromMsg(depth.header.stamp),
          std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(transform_timeout_sec_))));
    } catch (const tf2::TransformException & error) {
      response.outcome = AcquireLaneObservation::Response::OUTCOME_NO_TRANSFORM;
      response.detail = "no " + request.lane_id + " <- " + depth.header.frame_id +
        " transform at the acquisition stamp: " + error.what();
      RCLCPP_WARN(get_logger(), "%s", response.detail.c_str());
      return;
    }

    DepthCameraIntrinsics intrinsics;
    intrinsics.fx = info.k[0];
    intrinsics.fy = info.k[4];
    intrinsics.cx = info.k[2];
    intrinsics.cy = info.k[5];
    intrinsics.width = info.width;
    intrinsics.height = info.height;

    const std::span<const float> samples(
      reinterpret_cast<const float *>(depth.data.data()), depth.data.size() / sizeof(float));
    const auto measured = measure_lane_depth(
      samples, static_cast<std::uint32_t>(depth.step / sizeof(float)), intrinsics,
      lane_from_optical, *window, config_);

    LaneObservationMessage observation;
    observation.header.stamp = depth.header.stamp;
    observation.header.frame_id = request.lane_id;
    observation.lane_id = request.lane_id;
    observation.backend_name = backend_name_;
    observation.backend_version = backend_version_;
    if (!measured) {
      observation.status = LaneObservationMessage::STATUS_INTERNAL_ERROR;
      observation.status_detail = measured.error().detail;
      observation.available_depth_m = 0.0;
      observation.confidence = 0.0F;
    } else {
      const LaneDepthMeasurement & result = measured.value();
      observation.available_depth_m = result.available_depth_m;
      observation.obstructed = result.obstructed;
      observation.confidence = static_cast<float>(result.coverage);
      if (result.coverage < minimum_coverage_) {
        observation.status = LaneObservationMessage::STATUS_INSUFFICIENT_COVERAGE;
        observation.status_detail = "coverage " + std::to_string(result.coverage) +
          " is below the floor " + std::to_string(minimum_coverage_) + ": " +
          std::to_string(result.covered_bins) + " of " + std::to_string(result.total_bins) +
          " depth bins accounted for, from " + std::to_string(result.valid_depth_samples) +
          " valid returns in " + std::to_string(result.sampled_pixels) + " samples";
      } else {
        observation.status = LaneObservationMessage::STATUS_OK;
      }
      RCLCPP_INFO(
        get_logger(),
        "%s: available depth %.4f m (%s), coverage %.3f (%zu/%zu bins), %zu product and %zu bed "
        "returns, %zu behind the entrance%s",
        request.lane_id.c_str(), result.available_depth_m,
        result.nearest_surface_found ? "nearest surface measured" : "no surface found",
        result.coverage, result.covered_bins, result.total_bins, result.product_returns,
        result.bed_returns, result.obstruction_returns,
        observation.status == LaneObservationMessage::STATUS_OK ? "" : " [REFUSED]");
    }

    publisher_->publish(observation);
    response.outcome = AcquireLaneObservation::Response::OUTCOME_PUBLISHED;
    response.detail = observation.status_detail;
    response.observation = observation;
  }

  WorkcellSurveyGeometry geometry_;
  std::vector<LaneDepthWindow> windows_;
  LaneDepthConfig config_;
  std::string backend_name_;
  std::string backend_version_;
  double transform_timeout_sec_{0.0};
  double default_acquisition_timeout_sec_{0.0};
  double minimum_coverage_{0.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  std::condition_variable arrivals_;
  Image::ConstSharedPtr depth_;
  CameraInfo::ConstSharedPtr camera_info_;

  rclcpp::CallbackGroup::SharedPtr service_group_;
  rclcpp::CallbackGroup::SharedPtr subscription_group_;
  rclcpp::Publisher<LaneObservationMessage>::SharedPtr publisher_;
  rclcpp::Subscription<Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Service<AcquireLaneObservation>::SharedPtr service_;
};

}  // namespace
}  // namespace restocker_perception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    auto node = std::make_shared<restocker_perception::LaneObservationNode>(rclcpp::NodeOptions());
    // The service handler waits for a subscription delivery, so the two must be able to run at
    // once.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(
      rclcpp::get_logger("lane_observation"), "lane observation node failed: %s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
