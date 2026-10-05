// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_perception/tf_viewpoint_port.hpp"

#include <string>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace restocker_perception
{

TfViewpointPort::TfViewpointPort(
  const rclcpp::NodeOptions & options, TfViewpointPortConfig config, std::string node_name)
: config_(std::move(config)),
  node_(std::make_shared<rclcpp::Node>(std::move(node_name), options)),
  executor_(std::make_unique<rclcpp::executors::SingleThreadedExecutor>())
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  // The listener has no thread of its own; the executor below spins it.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
  executor_->add_node(node_);
  executor_thread_ = std::thread(
    [this]() noexcept {
      try {
        executor_->spin();
      } catch (...) {
        // Without the executor every later submission reports kMountUnavailable (fail closed).
      }
    });
  worker_ = std::thread([this]() noexcept {run_worker();});
}

TfViewpointPort::~TfViewpointPort()
{
  shutdown();
}

void TfViewpointPort::shutdown() noexcept
{
  if (stopping_.exchange(true)) {
    return;
  }
  cancel();
  work_available_.notify_all();
  if (worker_.joinable()) {
    worker_.join();
  }
  try {
    executor_->cancel();
  } catch (...) {
    // The executor is being torn down; joining below is what matters.
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  tf_listener_.reset();
  try {
    executor_->remove_node(node_);
  } catch (...) {
    // Removal is best-effort during teardown.
  }
}

bool TfViewpointPort::ready() const
{
  if (stopping_.load(std::memory_order_acquire)) {
    return false;
  }
  std::scoped_lock lock(mount_mutex_);
  return mount_.has_value();
}

ViewpointSubmitResult TfViewpointPort::submit(
  ViewpointCorrelation correlation, CameraViewpoint viewpoint, CompletionCallback callback)
{
  if (!callback) {
    return {ViewpointSubmitStatus::kInvalidRequest,
      "viewpoint submission requires a completion callback"};
  }
  if (viewpoint.pose.frame_id.empty()) {
    return {ViewpointSubmitStatus::kInvalidRequest,
      "a viewpoint must name the frame its camera pose is expressed in"};
  }
  if (!viewpoint.pose.pose.matrix().allFinite()) {
    return {ViewpointSubmitStatus::kInvalidRequest, "viewpoint pose is not finite"};
  }
  if (stopping_.load(std::memory_order_acquire)) {
    return {ViewpointSubmitStatus::kUnavailable, "viewpoint port is shutting down"};
  }
  {
    std::scoped_lock lock(mutex_);
    if (pending_ || busy_) {
      return {ViewpointSubmitStatus::kBusy, "viewpoint port already owns an outstanding request"};
    }
    cancel_requested_.store(false, std::memory_order_release);
    pending_.emplace(PendingRequest{correlation, std::move(viewpoint), std::move(callback)});
  }
  work_available_.notify_one();
  return {ViewpointSubmitStatus::kAccepted, {}};
}

void TfViewpointPort::cancel() noexcept
{
  cancel_requested_.store(true, std::memory_order_release);
}

void TfViewpointPort::run_worker() noexcept
{
  for (;; ) {
    PendingRequest request;
    {
      std::unique_lock lock(mutex_);
      work_available_.wait(
        lock, [this]() {return pending_.has_value() || stopping_.load(std::memory_order_acquire);});
      if (!pending_) {
        return;                     // shutting down with nothing outstanding
      }
      request = std::move(*pending_);
      pending_.reset();
      busy_ = true;
    }

    auto completion = resolve(request);

    {
      std::scoped_lock lock(mutex_);
      busy_ = false;
    }
    try {
      request.callback(std::move(completion));
    } catch (...) {
      // A throwing sink must not kill the worker, or later requests would never complete.
    }
  }
}

std::optional<WristCameraMount> TfViewpointPort::mount(std::string & detail)
{
  {
    std::scoped_lock lock(mount_mutex_);
    if (mount_) {
      return mount_;
    }
  }
  geometry_msgs::msg::TransformStamped message;
  try {
    // Latest available transform: the mount is a chain of fixed joints, and a stamped lookup would
    // fail whenever the buffer history did not reach back that far.
    message = tf_buffer_->lookupTransform(
      config_.tool_frame, config_.optical_frame, tf2::TimePointZero,
      tf2::durationFromSec(
        std::chrono::duration<double>(config_.mount_lookup_timeout).count()));
  } catch (const tf2::TransformException & error) {
    detail = "could not resolve " + config_.tool_frame + " <- " + config_.optical_frame + ": " +
      error.what();
    return std::nullopt;
  }
  Result<FramedTransform> framed = FramedTransform::create(
    config_.optical_frame, config_.tool_frame, tf2::transformToEigen(message));
  if (!framed) {
    detail = "TF returned an unusable mount transform: " + framed.error().detail;
    return std::nullopt;
  }
  Result<WristCameraMount> created = WristCameraMount::create(std::move(framed.value()));
  if (!created) {
    detail = created.error().detail;
    return std::nullopt;
  }
  RCLCPP_INFO(
    node_->get_logger(),
    "wrist camera mount established: %s <- %s is t=(%.4f, %.4f, %.4f)",
    config_.tool_frame.c_str(), config_.optical_frame.c_str(),
    created.value().tool0_from_optical().transform().translation().x(),
    created.value().tool0_from_optical().transform().translation().y(),
    created.value().tool0_from_optical().transform().translation().z());
  std::scoped_lock lock(mount_mutex_);
  mount_ = created.value();
  return mount_;
}

ViewpointCompletion TfViewpointPort::resolve(const PendingRequest & request) noexcept
{
  ViewpointCompletion completion;
  completion.correlation = request.correlation;
  if (cancel_requested_.load(std::memory_order_acquire)) {
    completion.outcome = ViewpointOutcome::kCanceled;
    completion.detail = "cancelled before the mount transform was read";
    return completion;
  }
  std::string detail;
  std::optional<WristCameraMount> camera_mount;
  try {
    camera_mount = mount(detail);
  } catch (const std::exception & error) {
    detail = error.what();
  }
  if (!camera_mount) {
    completion.outcome = ViewpointOutcome::kMountUnavailable;
    completion.detail = detail;
    return completion;
  }
  completion.tool0_from_optical = camera_mount->tool0_from_optical().transform();
  Result<Tool0ViewpointGoal> goal = camera_mount->tool0_goal_for(request.viewpoint);
  if (!goal) {
    completion.outcome = ViewpointOutcome::kInvalidRequest;
    completion.detail = goal.error().detail;
    return completion;
  }
  completion.outcome = ViewpointOutcome::kResolved;
  completion.goal = std::move(goal.value());
  return completion;
}

}  // namespace restocker_perception
