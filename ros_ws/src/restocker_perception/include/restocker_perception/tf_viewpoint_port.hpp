// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_perception/viewpoint_port.hpp"

namespace restocker_perception
{

struct TfViewpointPortConfig
{
  std::string tool_frame{kToolFrame};
  std::string optical_frame{kWristCameraOpticalFrame};
  // How long one lookup of the mount transform may take before the resolution completes
  // kMountUnavailable. The transform is static, so this only covers the wait for the first
  // /tf_static message; a long timeout would hold a survey open for a robot that is not publishing
  // its description.
  std::chrono::milliseconds mount_lookup_timeout{std::chrono::seconds(5)};
};

// A ViewpointPort backed by TF.
//
// The mount transform is read from the robot's published description, so this file contains no
// numbers. It owns a private node and a single-threaded executor thread for the TF listener, plus
// one worker thread so that a lookup waiting for /tf_static never runs on a caller's callback
// group.
//
// The transform is cached after the first successful lookup. It comes from two fixed joints, so a
// later reading can differ only if the description changed under a running robot, a fault this
// port cannot recover from anyway.
class TfViewpointPort final : public ViewpointPort
{
public:
  explicit TfViewpointPort(
    const rclcpp::NodeOptions & options, TfViewpointPortConfig config = {},
    std::string node_name = "wrist_viewpoint_client");
  ~TfViewpointPort() override;

  TfViewpointPort(const TfViewpointPort &) = delete;
  TfViewpointPort & operator=(const TfViewpointPort &) = delete;
  TfViewpointPort(TfViewpointPort &&) = delete;
  TfViewpointPort & operator=(TfViewpointPort &&) = delete;

  [[nodiscard]] bool ready() const override;

  [[nodiscard]] ViewpointSubmitResult submit(
    ViewpointCorrelation correlation, CameraViewpoint viewpoint,
    CompletionCallback callback) override;

  void cancel() noexcept override;

  // Stop the worker and release the listener. Idempotent; the destructor calls it.
  void shutdown() noexcept;

  [[nodiscard]] const rclcpp::Node::SharedPtr & node() const noexcept {return node_;}

private:
  struct PendingRequest
  {
    ViewpointCorrelation correlation;
    CameraViewpoint viewpoint;
    CompletionCallback callback;
  };

  void run_worker() noexcept;
  [[nodiscard]] ViewpointCompletion resolve(const PendingRequest & request) noexcept;
  // Returns the cached mount, looking it up once if it has not been established yet. Empty when
  // TF cannot answer inside the configured timeout, with `detail` set.
  [[nodiscard]] std::optional<WristCameraMount> mount(std::string & detail);

  TfViewpointPortConfig config_;
  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  mutable std::mutex mount_mutex_;
  std::optional<WristCameraMount> mount_;

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::optional<PendingRequest> pending_;
  bool busy_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> cancel_requested_{false};
  std::thread worker_;
};

}  // namespace restocker_perception
