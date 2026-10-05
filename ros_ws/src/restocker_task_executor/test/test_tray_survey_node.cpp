// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// The coordinated tray survey, end to end, without a simulator: a fake /survey_viewpoint
// server stands in for the motion coordinator, synthetic duty publications stand in for the
// perception nodes, and the real TraySurveyNode runs its real action contract.
//
// What this covers that the pure-logic tests cannot: the orchestration itself — station order,
// the confirm leg only after selection, cancellation abandoning unvisited stations while
// retaining the candidates already collected and leaving every child goal terminal, and the
// typed refutation arriving as the action's own outcome. The live headless acceptance of the
// same contracts against the renderer is test_tray_survey_runtime.py.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/survey_tray.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/perception_frame.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>

#include "restocker_task_executor/tray_survey_node.hpp"
#include "ros_domain_lease_guard.hpp"

namespace restocker_task_executor
{
namespace
{

using SurveyTray = restocker_interfaces::action::SurveyTray;
using SurveyViewpoint = restocker_interfaces::action::SurveyViewpoint;
using Observation = restocker_interfaces::msg::ObjectObservation;
using PerceptionFrame = restocker_interfaces::msg::PerceptionFrame;
using TrayGoalHandle = rclcpp_action::ServerGoalHandle<SurveyTray>;
using ViewpointGoalHandle = rclcpp_action::ServerGoalHandle<SurveyViewpoint>;

constexpr auto kShortWait = std::chrono::seconds(10);
constexpr auto kGoalWait = std::chrono::seconds(30);

// Where the synthetic products stand, in the planning frame. The can is the shipped scenario's
// stock can; the positions matter only relative to the association radius and each other.
constexpr double kCanX = -0.42;
constexpr double kCanY = -0.25;
constexpr double kCanZ = 0.631;
constexpr double kFarX = 0.40;
// The dense tray's row pitch: a can this far along +Y (the shelf's +Y, nearer the robot) stands
// in front of the near can in its feed column.
constexpr double kFrontPitch = 0.085;

[[nodiscard]] std::string required_environment(const char * name)
{
  const char * value = std::getenv(name);
  if (value == nullptr || *value == '\0') {
    throw std::runtime_error(std::string(name) + " must be set for this test");
  }
  return value;
}

[[nodiscard]] Observation observation(
  double x, double y, double z, std::uint8_t product_class, const std::string & sku,
  const std::string & backend, const rclcpp::Time & stamp, float confidence = 0.85F)
{
  Observation message;
  message.header.frame_id = "world";
  message.header.stamp = stamp;
  message.product_class = product_class;
  message.has_sku = true;
  message.sku = sku;
  message.backend_name = backend;
  message.backend_version = "1.0.0";
  message.confidence = confidence;
  message.status = Observation::STATUS_OK;
  message.pose.pose.position.x = x;
  message.pose.pose.position.y = y;
  message.pose.pose.position.z = z;
  message.pose.covariance[0] = 1.0e-4;
  message.pose.covariance[7] = 1.0e-4;
  message.pose.covariance[14] = 1.0e-4;
  return message;
}

// What the fake /survey_viewpoint server has been asked to do, and whether it finished.
struct FakeLeg
{
  std::string station;
  bool has_explicit_pose{false};
  std::string label;
  std::atomic<bool> terminal{false};
};

// A stand-in for the real motion coordinator: accepts SurveyViewpoint goals, "travels" for a
// few milliseconds, and either arrives or blocks until canceled. It records every goal so a
// test can prove which legs were sent and that none outlived the survey.
class FakeViewpointServer
{
public:
  enum class Mode
  {
    kArrive,
    // Block the named station's goal until it is canceled, then end canceled. The rest arrive.
    kBlockStationUntilCancel,
  };

  FakeViewpointServer()
  : node_(std::make_shared<rclcpp::Node>("fake_survey_viewpoint"))
  {
    server_ = rclcpp_action::create_server<SurveyViewpoint>(
      node_, "survey_viewpoint",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SurveyViewpoint::Goal>) {
        std::scoped_lock lock(mutex_);
        if (reject_next_) {
          reject_next_ = false;
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<ViewpointGoalHandle> &) {
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ViewpointGoalHandle> & handle) {
        std::thread([this, handle] {execute(handle);}).detach();
      });
  }

  [[nodiscard]] rclcpp::Node::SharedPtr node() const {return node_;}

  void set_mode(Mode mode, std::string block_station = {})
  {
    std::scoped_lock lock(mutex_);
    mode_ = mode;
    block_station_ = std::move(block_station);
  }

  void reject_next_goal()
  {
    std::scoped_lock lock(mutex_);
    reject_next_ = true;
  }

  void reset()
  {
    std::scoped_lock lock(mutex_);
    mode_ = Mode::kArrive;
    block_station_.clear();
    reject_next_ = false;
    legs_.clear();
  }

  [[nodiscard]] std::size_t leg_count() const
  {
    std::scoped_lock lock(mutex_);
    return legs_.size();
  }

  [[nodiscard]] std::vector<std::pair<std::string, bool>> legs_snapshot() const
  {
    std::scoped_lock lock(mutex_);
    std::vector<std::pair<std::string, bool>> snapshot;
    snapshot.reserve(legs_.size());
    for (const auto & leg : legs_) {
      snapshot.emplace_back(leg->station, leg->has_explicit_pose);
    }
    return snapshot;
  }

  // Every leg that was accepted has delivered its terminal result. Polls, because the survey's
  // own result can land a few microseconds before the fake's bookkeeping does.
  [[nodiscard]] bool all_legs_terminal()
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      {
        std::scoped_lock lock(mutex_);
        bool all = true;
        for (const auto & leg : legs_) {
          all = all && leg->terminal.load();
        }
        if (all && !legs_.empty()) {
          return true;
        }
        if (all && legs_.empty()) {
          return true;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
  }

private:
  void execute(const std::shared_ptr<ViewpointGoalHandle> & handle)
  {
    const SurveyViewpoint::Goal goal = *handle->get_goal();
    auto leg = std::make_shared<FakeLeg>();
    leg->station = goal.station;
    leg->has_explicit_pose = !goal.camera_optical_pose.header.frame_id.empty();
    leg->label = goal.label;
    {
      std::scoped_lock lock(mutex_);
      legs_.push_back(leg);
    }

    Mode mode;
    std::string block_station;
    {
      std::scoped_lock lock(mutex_);
      mode = mode_;
      block_station = block_station_;
    }
    const bool blocked =
      mode == Mode::kBlockStationUntilCancel && !block_station.empty() &&
      goal.station == block_station;

    auto result = std::make_shared<SurveyViewpoint::Result>();
    result->station = goal.station;
    if (blocked) {
      while (!handle->is_canceling() && rclcpp::ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      leg->terminal = true;
      if (handle->is_canceling()) {
        result->outcome = SurveyViewpoint::Result::OUTCOME_CANCELED;
        result->detail = "canceled by the tray survey";
        handle->canceled(result);
        return;
      }
      result->outcome = SurveyViewpoint::Result::OUTCOME_UNAVAILABLE;
      result->detail = "the fake viewpoint server shut down mid-goal";
      handle->abort(result);
      return;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    leg->terminal = true;
    if (handle->is_canceling()) {
      // Cancel raced a completion that was already decided; honor the cancel.
      result->outcome = SurveyViewpoint::Result::OUTCOME_CANCELED;
      result->detail = "canceled after arriving";
      handle->canceled(result);
      return;
    }
    result->outcome = SurveyViewpoint::Result::OUTCOME_ARRIVED;
    result->detail = "arrived (fake)";
    handle->succeed(result);
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<SurveyViewpoint>::SharedPtr server_;
  mutable std::mutex mutex_;
  Mode mode_{Mode::kArrive};
  std::string block_station_;
  bool reject_next_{false};
  std::vector<std::shared_ptr<FakeLeg>> legs_;
};

// What the synthetic duty publishers should be emitting while a goal runs.
enum class ConfirmMode
{
  kSilent,
  // A confirm-backend observation of the same identity at the candidate's place.
  kAgrees,
  // A confirm-backend observation at the candidate's place with a different class.
  kContradictsClass,
  // A confirm-backend observation, but nowhere near the candidate.
  kAbsent,
  // A confirm-backend observation of the same identity at the far candidate's place, for the
  // reselection case where the near candidate was refuted earlier this cycle.
  kAgreesFar,
};

class TraySurveyHarness
{
public:
  static void SetUpTestSuite()
  {
    instance_ = std::make_unique<TraySurveyHarness>();
  }

  static void TearDownTestSuite()
  {
    instance_.reset();
  }

  [[nodiscard]] static TraySurveyHarness & get() {return *instance_;}

  TraySurveyHarness()
  {
    helper_ = std::make_shared<rclcpp::Node>("tray_survey_test_helper");
    static_tf_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(helper_);
    // The shipped world <- shelf transform: the confirmation solver asks for it by name.
    geometry_msgs::msg::TransformStamped world_from_shelf;
    world_from_shelf.header.frame_id = "world";
    world_from_shelf.child_frame_id = "shelf";
    world_from_shelf.header.stamp = helper_->now();
    world_from_shelf.transform.translation.x = 0.0;
    world_from_shelf.transform.translation.y = 0.55;
    world_from_shelf.transform.translation.z = 0.75;
    world_from_shelf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(world_from_shelf);

    overview_publisher_ = helper_->create_publisher<Observation>(
      "/perception/tray_candidates", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    confirmation_publisher_ = helper_->create_publisher<Observation>(
      "/perception/object_observations", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
    // Card 050's stage taps: the raw wrist colour stream and the overview duty's detection
    // topic, so a test can hold any stage silent and read the stage's name off the result's
    // per-station report instead of off a log line.
    image_publisher_ = helper_->create_publisher<sensor_msgs::msg::Image>(
      "/wrist_camera/image", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());
    detection_publisher_ = helper_->create_publisher<PerceptionFrame>(
      "/perception/tray_overview_detections", rclcpp::QoS(rclcpp::KeepLast(5)).reliable());

    fake_ = std::make_unique<FakeViewpointServer>();

    rclcpp::NodeOptions options;
    options.parameter_overrides(
        {
          {"workcell_geometry_path", required_environment("RESTOCKER_TEST_WORKCELL_GEOMETRY")},
          // Short windows: the orchestration under test does not care how long a station dwells.
          {"acquisition_dwell_ms", 50},
          {"overview_collection_ms", 250},
          {"arrival_grace_ms", 50},
          {"confirmation_timeout_ms", 400},
        });
    tray_ = std::make_shared<TraySurveyNode>(options);

    client_node_ = std::make_shared<rclcpp::Node>("tray_survey_test_client");
    tray_client_ = rclcpp_action::create_client<SurveyTray>(client_node_, "survey_tray");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(helper_);
    executor_->add_node(fake_->node());
    executor_->add_node(tray_->node());
    executor_->add_node(client_node_);
    spin_thread_ = std::thread([this] {executor_->spin();});

    publisher_thread_ = std::thread([this] {publish_loop();});

    if (!tray_client_->wait_for_action_server(kShortWait)) {
      throw std::runtime_error("the tray survey action server never appeared");
    }
  }

  ~TraySurveyHarness()
  {
    // Stop the publisher first: its loop must not wait for rclcpp::shutdown, which happens
    // only after this destructor returns — joining on that would deadlock the suite teardown.
    stopping_ = true;
    publish_overview_ = false;
    publish_confirm_ = false;
    publish_images_ = false;
    publish_detections_ = false;
    if (publisher_thread_.joinable()) {
      publisher_thread_.join();
    }
    // Stop the tray node before the executor: its destructor cancels any child goal and joins
    // the survey worker while the context is still up.
    tray_.reset();
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    fake_.reset();
    client_node_.reset();
    helper_.reset();
  }

  void reset_fakes()
  {
    fake_->reset();
    publish_overview_ = false;
    publish_confirm_ = false;
    publish_images_ = false;
    publish_detections_ = false;
    foreign_overview_backend_ = false;
    publish_front_can_ = false;
    observation_lag_ms_.store(0);
    confirm_mode_ = ConfirmMode::kSilent;
    last_feedback_phase_ = -1;
    last_feedback_station_.clear();
    feedback_seen_.store(false);
  }

  struct SentGoal
  {
    bool accepted{false};
    // Terminal ROS action status of the goal.
    int code{0};
    SurveyTray::Result result;
    std::vector<std::pair<uint8_t, std::string>> feedback;
  };

  [[nodiscard]] SentGoal send(
    const std::function<void(SurveyTray::Goal &)> & customize,
    const std::chrono::milliseconds timeout = kGoalWait)
  {
    auto goal = std::make_shared<SurveyTray::Goal>();
    customize(*goal);
    rclcpp_action::Client<SurveyTray>::SendGoalOptions options;
    options.feedback_callback =
      [this](rclcpp_action::ClientGoalHandle<SurveyTray>::SharedPtr,
      const std::shared_ptr<const SurveyTray::Feedback> feedback) {
        std::scoped_lock lock(feedback_mutex_);
        last_feedback_phase_ = feedback->phase;
        last_feedback_station_ = feedback->current_station;
        feedback_.emplace_back(feedback->phase, feedback->current_station);
        feedback_seen_ = true;
      };
    auto send_future = tray_client_->async_send_goal(*goal, options);
    SentGoal sent;
    if (send_future.wait_for(kShortWait) != std::future_status::ready) {
      return sent;
    }
    auto handle = send_future.get();
    if (!handle) {
      return sent;
    }
    sent.accepted = true;
    auto result_future = tray_client_->async_get_result(handle);
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      tray_client_->async_cancel_goal(handle);
      // Give up waiting; the caller sees the stale defaults and fails loudly.
      return sent;
    }
    const auto wrapped = result_future.get();
    sent.code = static_cast<int>(wrapped.code);
    if (wrapped.result) {
      sent.result = *wrapped.result;
    }
    return sent;
  }

  // Send a goal and, when the survey reports the given overview station, cancel it. Returns
  // the result of the canceled goal.
  [[nodiscard]] SentGoal send_and_cancel_on_station(
    const std::string & station, const std::function<void(SurveyTray::Goal &)> & customize)
  {
    auto goal = std::make_shared<SurveyTray::Goal>();
    customize(*goal);
    rclcpp_action::Client<SurveyTray>::SendGoalOptions options;
    options.feedback_callback =
      [this, station](rclcpp_action::ClientGoalHandle<SurveyTray>::SharedPtr,
      const std::shared_ptr<const SurveyTray::Feedback> feedback) {
        if (feedback->phase == SurveyTray::Feedback::PHASE_OVERVIEW &&
          feedback->current_station == station)
        {
          cancel_requested_ = true;
        }
        std::scoped_lock lock(feedback_mutex_);
        last_feedback_phase_ = feedback->phase;
        last_feedback_station_ = feedback->current_station;
        feedback_.emplace_back(feedback->phase, feedback->current_station);
        feedback_seen_ = true;
      };
    auto send_future = tray_client_->async_send_goal(*goal, options);
    SentGoal sent;
    if (send_future.wait_for(kShortWait) != std::future_status::ready) {
      return sent;
    }
    auto handle = send_future.get();
    if (!handle) {
      return sent;
    }
    sent.accepted = true;
    cancel_requested_ = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
    while (!cancel_requested_ && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto cancel_future = tray_client_->async_cancel_goal(handle);
    cancel_future.wait_for(kShortWait);
    auto result_future = tray_client_->async_get_result(handle);
    if (result_future.wait_for(kGoalWait) != std::future_status::ready) {
      return sent;
    }
    const auto wrapped = result_future.get();
    sent.code = static_cast<int>(wrapped.code);
    if (wrapped.result) {
      sent.result = *wrapped.result;
    }
    return sent;
  }

  [[nodiscard]] FakeViewpointServer & fake() {return *fake_;}
  [[nodiscard]] rclcpp::Node::SharedPtr tray_node() const {return tray_->node();}
  [[nodiscard]] std::vector<std::pair<uint8_t, std::string>> feedback() const
  {
    std::scoped_lock lock(feedback_mutex_);
    return feedback_;
  }
  void set_confirm_mode(ConfirmMode mode) {confirm_mode_ = mode;}
  void start_overview_publication()
  {
    publish_overview_ = true;
    // The healthy pipeline publishes all three stages at once: frames arrive, the duty's
    // detection topic carries them, and the duty publishes observations.
    publish_images_ = true;
    publish_detections_ = true;
  }
  void start_confirmation_publication() {publish_confirm_ = true;}
  void stop_publication()
  {
    publish_overview_ = false;
    publish_confirm_ = false;
    publish_images_ = false;
    publish_detections_ = false;
    foreign_overview_backend_ = false;
  }
  // Hold one stage silent while the others flow, so the per-station report has to name it.
  void set_stage_taps(const bool images, const bool detections)
  {
    publish_images_ = images;
    publish_detections_ = detections;
    publish_overview_ = false;
  }
  void set_foreign_overview_backend(const bool foreign) {foreign_overview_backend_ = foreign;}
  // Card 066: one more can standing a tray pitch in front of the near can, in its feed column.
  void set_front_can(const bool front) {publish_front_can_ = front;}
  // Stamp the synthetic duty observations this far behind the node clock: their arrival then
  // trails their stamp by the lag, which Card 050's zero-dwell drain exists to absorb.
  void set_observation_stamp_lag(const std::chrono::milliseconds lag)
  {
    observation_lag_ms_.store(static_cast<int>(lag.count()));
  }

private:
  void publish_loop()
  {
    using namespace std::chrono_literals;
    while (rclcpp::ok() && !stopping_) {
      const rclcpp::Time now = helper_->now();
      const rclcpp::Time observation_stamp = now - rclcpp::Duration(
        std::chrono::nanoseconds(
          static_cast<std::int64_t>(observation_lag_ms_.load()) *
          1'000'000LL));
      if (publish_images_) {
        sensor_msgs::msg::Image image;
        image.header.frame_id = "wrist_camera_optical_frame";
        image.header.stamp = now;
        image.width = 4U;
        image.height = 4U;
        image.encoding = "rgb8";
        image.step = 12U;
        image.data.assign(48U, 0U);
        image_publisher_->publish(image);
      }
      if (publish_detections_) {
        // The duty's detection frame for this acquisition: two proposals, matching the two
        // overview cans below — frames_with_detections therefore tracks images here.
        PerceptionFrame frame;
        frame.header.frame_id = "wrist_camera_optical_frame";
        frame.header.stamp = now;
        frame.backend_name = "wrist_rgbd_tray_overview";
        frame.backend_version = "synthetic_stage_tap";
        frame.status = PerceptionFrame::STATUS_OK;
        frame.detections.resize(2U);
        detection_publisher_->publish(frame);
      }
      if (publish_overview_) {
        const std::string backend =
          foreign_overview_backend_ ? "intruder_backend" : "wrist_rgbd_tray_overview";
        overview_publisher_->publish(
          observation(
            kCanX, kCanY, kCanZ, Observation::PRODUCT_CLASS_CAN, "SIM-CAN-STD",
            backend, observation_stamp));
        // A second can far along the tray: the reselection tests mark the near one refuted and
        // must still find a distinct candidate to confirm.
        overview_publisher_->publish(
          observation(
            kFarX, kCanY, kCanZ, Observation::PRODUCT_CLASS_CAN, "SIM-CAN-STD",
            backend, observation_stamp));
        if (publish_front_can_) {
          overview_publisher_->publish(
            observation(
              kCanX, kCanY + kFrontPitch, kCanZ, Observation::PRODUCT_CLASS_CAN, "SIM-CAN-STD",
              backend, observation_stamp));
        }
      }
      if (publish_confirm_) {
        switch (confirm_mode_) {
          case ConfirmMode::kSilent:
            break;
          case ConfirmMode::kAgrees:
            confirmation_publisher_->publish(
              observation(
                kCanX + 0.004, kCanY, kCanZ, Observation::PRODUCT_CLASS_CAN,
                "SIM-CAN-STD", "wrist_rgbd_tray_confirm", observation_stamp, 0.90F));
            break;
          case ConfirmMode::kAgreesFar:
            confirmation_publisher_->publish(
              observation(
                kFarX + 0.004, kCanY, kCanZ, Observation::PRODUCT_CLASS_CAN,
                "SIM-CAN-STD", "wrist_rgbd_tray_confirm", observation_stamp, 0.90F));
            break;
          case ConfirmMode::kContradictsClass:
            confirmation_publisher_->publish(
              observation(
                kCanX + 0.004, kCanY, kCanZ, Observation::PRODUCT_CLASS_LARGE_BOTTLE,
                "SIM-BOTTLE-LARGE", "wrist_rgbd_tray_confirm", observation_stamp, 0.90F));
            break;
          case ConfirmMode::kAbsent:
            confirmation_publisher_->publish(
              observation(
                kFarX, kCanY, kCanZ, Observation::PRODUCT_CLASS_CAN,
                "SIM-CAN-STD", "wrist_rgbd_tray_confirm", observation_stamp, 0.90F));
            break;
        }
      }
      std::this_thread::sleep_for(30ms);
    }
  }

  static std::unique_ptr<TraySurveyHarness> instance_;
  rclcpp::Node::SharedPtr helper_;
  rclcpp::Node::SharedPtr client_node_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  rclcpp::Publisher<Observation>::SharedPtr overview_publisher_;
  rclcpp::Publisher<Observation>::SharedPtr confirmation_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<PerceptionFrame>::SharedPtr detection_publisher_;
  std::unique_ptr<FakeViewpointServer> fake_;
  std::shared_ptr<TraySurveyNode> tray_;
  rclcpp_action::Client<SurveyTray>::SharedPtr tray_client_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::thread publisher_thread_;
  std::atomic<bool> stopping_{false};
  std::atomic<bool> publish_overview_{false};
  std::atomic<bool> publish_confirm_{false};
  std::atomic<bool> publish_images_{false};
  std::atomic<bool> publish_detections_{false};
  std::atomic<bool> foreign_overview_backend_{false};
  std::atomic<bool> publish_front_can_{false};
  std::atomic<int> observation_lag_ms_{0};
  std::atomic<bool> cancel_requested_{false};
  ConfirmMode confirm_mode_{ConfirmMode::kSilent};
  mutable std::mutex feedback_mutex_;
  int last_feedback_phase_{-1};
  std::string last_feedback_station_;
  std::vector<std::pair<uint8_t, std::string>> feedback_;
  std::atomic<bool> feedback_seen_{false};
};

std::unique_ptr<TraySurveyHarness> TraySurveyHarness::instance_;

// Registered and executed before TraySurveyNodeTest's tests: gtest runs suites in registration
// order, and both this test and the shared harness register /survey_tray and /survey_viewpoint
// on the same domain, so this one must be the only provider while it drives its goal.
//
// Regression for the teardown hang: the leg-result wait loop used to spin until the child's
// result arrived, but once this node is being destroyed nothing completes that future (the
// executor has stopped, or this node's callbacks are going away) and the destructor's join
// blocked until the launch escalated to SIGKILL. The loop now breaks on shutdown, after
// requesting the child's cancel.
TEST(TraySurveyShutdown, DestructorReturnsPromptlyWhileALegIsStillInFlight)
{
  FakeViewpointServer fake;
  fake.set_mode(FakeViewpointServer::Mode::kBlockStationUntilCancel, "tray_1");

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {
        {"workcell_geometry_path", required_environment("RESTOCKER_TEST_WORKCELL_GEOMETRY")},
        {"acquisition_dwell_ms", 50},
        {"overview_collection_ms", 250},
        {"arrival_grace_ms", 50},
        {"confirmation_timeout_ms", 400},
      });
  auto tray = std::make_shared<TraySurveyNode>(options);
  auto client_node = std::make_shared<rclcpp::Node>("tray_survey_shutdown_client");
  auto client = rclcpp_action::create_client<SurveyTray>(client_node, "survey_tray");

  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(fake.node());
  executor->add_node(tray->node());
  executor->add_node(client_node);
  std::thread spin([executor] {executor->spin();});

  if (!client->wait_for_action_server(kShortWait)) {
    executor->cancel();
    spin.join();
    FAIL() << "the tray survey action server never appeared";
  }

  SurveyTray::Goal goal;
  goal.overview_only = true;
  auto send_future = client->async_send_goal(goal);
  if (send_future.wait_for(kShortWait) != std::future_status::ready) {
    tray.reset();
    executor->cancel();
    spin.join();
    FAIL() << "the tray survey goal was never answered";
  }
  auto handle = send_future.get();
  if (!handle) {
    tray.reset();
    executor->cancel();
    spin.join();
    FAIL() << "the tray survey goal was rejected";
  }

  // The blocked leg reaches the fake server and then holds until it is canceled.
  const auto leg_deadline = std::chrono::steady_clock::now() + kShortWait;
  while (fake.leg_count() == 0U && std::chrono::steady_clock::now() < leg_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  if (fake.leg_count() == 0U) {
    tray.reset();
    executor->cancel();
    spin.join();
    FAIL() << "the overview leg never reached the fake viewpoint server";
  }
  // Let the goal response land and the worker install the child handle: the destructor can
  // only cancel a child it holds, and the response crosses the wire in milliseconds.
  std::this_thread::sleep_for(std::chrono::seconds(1));

  const auto destroyed_at = std::chrono::steady_clock::now();
  tray.reset();
  const auto destroy_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - destroyed_at).count();
  EXPECT_LT(destroy_ms, 5000)
    << "the destructor blocked for " << destroy_ms << " ms waiting on a leg result that can "
    << "never arrive";

  // The cancel the destructor requested reached the child: no motion goal outlives the node.
  EXPECT_TRUE(fake.all_legs_terminal());

  // Finish the parent goal's client side (its result was delivered during the destructor).
  auto result_future = client->async_get_result(handle);
  result_future.wait_for(kShortWait);

  executor->cancel();
  spin.join();
}

class TraySurveyNodeTest : public ::testing::Test
{
protected:
  // The harness is heavier than one test: one fake server, one tray node, one executor. Built
  // before the first test of the fixture and torn down after the last, which keeps the tray
  // node's destructor inside the rclcpp context started by main().
  static void SetUpTestSuite() {TraySurveyHarness::SetUpTestSuite();}
  static void TearDownTestSuite() {TraySurveyHarness::TearDownTestSuite();}

  void SetUp() override
  {
    harness_ = &TraySurveyHarness::get();
    harness_->reset_fakes();
  }

  TraySurveyHarness * harness_{nullptr};
};

TEST_F(TraySurveyNodeTest, OverviewOnlyVisitsBothStationsAndNeverAimsConfirmation)
{
  // SC-001, structurally: overview completes, candidates are returned as hypotheses, and the
  // survey never sends the confirmation leg, so no confirm-duty observation is even waited for.
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_OVERVIEW_ONLY)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  ASSERT_EQ(sent.result.overview_stations_visited.size(), 2U);
  EXPECT_EQ(sent.result.overview_stations_visited[0], "tray_1");
  EXPECT_EQ(sent.result.overview_stations_visited[1], "tray_2");
  ASSERT_FALSE(sent.result.overview_candidates.empty());
  for (const auto & candidate : sent.result.overview_candidates) {
    EXPECT_EQ(candidate.backend_name, "wrist_rgbd_tray_overview");
    EXPECT_EQ(candidate.header.frame_id, "world");
  }
  EXPECT_EQ(sent.result.selected_candidate.header.frame_id, "");
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "");
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_NONE);

  // Exactly the two station legs: no explicit-pose leg was ever sent, so confirmation was
  // never aimed, and every leg that ran is already terminal.
  const auto legs = harness_->fake().legs_snapshot();
  ASSERT_EQ(legs.size(), 2U);
  EXPECT_EQ(legs[0].first, "tray_1");
  EXPECT_FALSE(legs[0].second);
  EXPECT_EQ(legs[1].first, "tray_2");
  EXPECT_FALSE(legs[1].second);
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, ConfirmsWhenTheCloseViewAgreesAfterBothOverviewStations)
{
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kAgrees);
  harness_->start_confirmation_publication();
  const auto sent = harness_->send([](SurveyTray::Goal &) {});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_CONFIRMED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  ASSERT_EQ(sent.result.overview_stations_visited.size(), 2U);
  EXPECT_EQ(sent.result.selected_candidate.product_class, Observation::PRODUCT_CLASS_CAN);
  EXPECT_EQ(sent.result.confirmed_observation.backend_name, "wrist_rgbd_tray_confirm");
  EXPECT_EQ(sent.result.confirmed_observation.product_class, Observation::PRODUCT_CLASS_CAN);
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "world");
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_NONE);
  EXPECT_EQ(sent.result.refuting_observation.header.frame_id, "");

  // Both stations first, then exactly one explicit-pose confirmation leg.
  const auto legs = harness_->fake().legs_snapshot();
  ASSERT_EQ(legs.size(), 3U);
  EXPECT_EQ(legs[0].first, "tray_1");
  EXPECT_EQ(legs[1].first, "tray_2");
  EXPECT_TRUE(legs[2].second);
  EXPECT_TRUE(legs[2].first.empty());
  EXPECT_TRUE(harness_->fake().all_legs_terminal());

  // Overview stations precede confirmation in the feedback the client actually saw.
  bool saw_overview = false;
  bool saw_confirming_after_overview = false;
  for (const auto & phase : harness_->feedback()) {
    if (phase.first == SurveyTray::Feedback::PHASE_OVERVIEW) {
      saw_overview = true;
    }
    if (phase.first == SurveyTray::Feedback::PHASE_CONFIRMING && saw_overview) {
      saw_confirming_after_overview = true;
    }
  }
  EXPECT_TRUE(saw_confirming_after_overview);
}

TEST_F(TraySurveyNodeTest, SkipsACandidateRefutedThisCycleAndConfirmsTheReselection)
{
  // The campaign re-enters SURVEY_TRAY with the refuted candidate marked; the survey must not
  // re-propose it and must confirm the next candidate instead.
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kAgreesFar);
  harness_->start_confirmation_publication();
  const auto sent = harness_->send(
    [this](SurveyTray::Goal & goal) {
      geometry_msgs::msg::Point marked;
      marked.x = kCanX;
      marked.y = kCanY;
      marked.z = kCanZ;
      goal.refuted_positions.push_back(marked);
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_CONFIRMED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_NEAR(sent.result.selected_candidate.pose.pose.position.x, kFarX, 1.0e-6);
  EXPECT_EQ(sent.result.confirmed_observation.backend_name, "wrist_rgbd_tray_confirm");
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_NONE);
}

TEST_F(TraySurveyNodeTest, EveryMatchingCandidateRefutedReportsNoCandidate)
{
  // With both overview cans marked refuted, selection must report no candidate rather than
  // re-selecting a doomed one: that is the terminal edge of the refutation loop. The detail
  // names refutations, never skip marks (Card 058 receipt honesty).
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [this](SurveyTray::Goal & goal) {
      for (const double x : {kCanX, kFarX}) {
        geometry_msgs::msg::Point marked;
        marked.x = x;
        marked.y = kCanY;
        marked.z = kCanZ;
        goal.refuted_positions.push_back(marked);
      }
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(
    sent.result.detail,
    "every overview candidate matching the request was refuted earlier this cycle");
  EXPECT_EQ(sent.result.selected_candidate.header.frame_id, "");
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "");
}

// Milestone 10 §6, Card 066, through the node: feed order is judged in the looked-up shelf frame,
// a refuted front keeps the can behind it from being nominated, and the typed count reaches
// the result so the campaign can tell a feed-blocked tray from an empty one.
TEST_F(TraySurveyNodeTest, ARefutedFrontKeepsTheCanBehindItFromBeingNominated)
{
  harness_->set_front_can(true);
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      for (const auto & [x, y] : {std::pair{kCanX, kCanY + kFrontPitch}, std::pair{kFarX, kCanY}}) {
        geometry_msgs::msg::Point marked;
        marked.x = x;
        marked.y = y;
        marked.z = kCanZ;
        goal.refuted_positions.push_back(marked);
      }
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.feed_blocked_candidates, 1U);
  EXPECT_NE(sent.result.detail.find("feed column"), std::string::npos) << sent.result.detail;
  EXPECT_TRUE(harness_->fake().legs_snapshot().size() == 2U) << "no confirmation leg";
}

TEST_F(TraySurveyNodeTest, NominatesTheFrontCanOfAFeedColumnRatherThanTheOneBehindIt)
{
  harness_->set_front_can(true);
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kSilent);
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      geometry_msgs::msg::Point marked;
      marked.x = kFarX;
      marked.y = kCanY;
      marked.z = kCanZ;
      goal.refuted_positions.push_back(marked);
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  // Pre-066 order would pick the near can (smallest y); the front one is the only nominee.
  EXPECT_NEAR(sent.result.selected_candidate.pose.pose.position.y, kCanY + kFrontPitch, 1.0e-6)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.feed_blocked_candidates, 1U);
}

TEST_F(TraySurveyNodeTest, EveryMatchingCandidateSkipMarkedReportsSkipDetail)
{
  // Card 058: skip marks travel in their own goal field and the NO_CANDIDATE detail names
  // them as skip marks — it must never claim the candidates were refuted this cycle.
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [this](SurveyTray::Goal & goal) {
      for (const double x : {kCanX, kFarX}) {
        geometry_msgs::msg::Point marked;
        marked.x = x;
        marked.y = kCanY;
        marked.z = kCanZ;
        goal.skip_mark_positions.push_back(marked);
      }
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(
    sent.result.detail,
    "every overview candidate matching the request was excluded by an active skip mark");
  EXPECT_EQ(sent.result.selected_candidate.header.frame_id, "");
}

TEST_F(TraySurveyNodeTest, SkipMarkExcludesTheMarkedCandidateAndConfirmsTheReselection)
{
  // A skip mark excludes like a refutation (selection unchanged), so the far candidate is
  // confirmed; only the wording of a terminal no-candidate differs between the kinds.
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kAgreesFar);
  harness_->start_confirmation_publication();
  const auto sent = harness_->send(
    [this](SurveyTray::Goal & goal) {
      geometry_msgs::msg::Point marked;
      marked.x = kCanX;
      marked.y = kCanY;
      marked.z = kCanZ;
      goal.skip_mark_positions.push_back(marked);
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_CONFIRMED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_NEAR(sent.result.selected_candidate.pose.pose.position.x, kFarX, 1.0e-6);
}

// Card 066 × Card 058 integration (I1), through the node: a skip mark in its own goal field keeps
// the front can from being nominated, and the can standing behind it stays blocked. The per-
// candidate front report says which of the overview candidates is a front.
TEST_F(TraySurveyNodeTest, ASkipMarkedFrontStillBlocksTheCanBehindIt)
{
  harness_->set_front_can(true);
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      geometry_msgs::msg::Point front;
      front.x = kCanX;
      front.y = kCanY + kFrontPitch;
      front.z = kCanZ;
      goal.skip_mark_positions.push_back(front);
      geometry_msgs::msg::Point far;
      far.x = kFarX;
      far.y = kCanY;
      far.z = kCanZ;
      goal.refuted_positions.push_back(far);
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.feed_blocked_candidates, 1U);
  EXPECT_EQ(
    sent.result.detail,
    "every remaining matching candidate stands behind another tray product in its feed column "
    "(1 feed-blocked); the others are excluded by earlier refutations and active skip marks");
  ASSERT_EQ(
    sent.result.overview_candidate_feed_front.size(), sent.result.overview_candidates.size());
  std::size_t fronts = 0U;
  for (std::size_t index = 0U; index < sent.result.overview_candidates.size(); ++index) {
    const auto & position = sent.result.overview_candidates[index].pose.pose.position;
    const bool behind = std::abs(position.x - kCanX) < 0.02 && std::abs(position.y - kCanY) < 0.02;
    EXPECT_EQ(sent.result.overview_candidate_feed_front[index], !behind)
      << "candidate at (" << position.x << ", " << position.y << ")";
    fronts += sent.result.overview_candidate_feed_front[index] ? 1U : 0U;
  }
  EXPECT_EQ(fronts, 2U) << "the skip-marked front and the far can";
  EXPECT_EQ(harness_->fake().legs_snapshot().size(), 2U) << "no confirmation leg";
  // Review cmbrev066b: the blocked can and its blocker are named, nothing marked stands behind a
  // front, and the server reports the merge radius it applied.
  EXPECT_EQ(sent.result.marked_feed_blocked_candidates, 0U);
  EXPECT_EQ(
    sent.result.feed_block_example,
    "can at (-0.420, -0.250) stands behind can at (-0.420, -0.165)");
  EXPECT_DOUBLE_EQ(sent.result.candidate_merge_radius_m, 0.04);
}

// Review cmbrev066b N1, through the node: a skip-marked can behind a refuted front is counted as
// marked stock behind a front, and the detail names it, so the campaign does not read the tray
// as exhausted.
TEST_F(TraySurveyNodeTest, AMarkedCanBehindARefutedFrontIsReportedAsMarkedStockBehindAFront)
{
  harness_->set_front_can(true);
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      for (const auto & [x, y] : {std::pair{kCanX, kCanY + kFrontPitch}, std::pair{kFarX, kCanY}}) {
        geometry_msgs::msg::Point refuted;
        refuted.x = x;
        refuted.y = y;
        refuted.z = kCanZ;
        goal.refuted_positions.push_back(refuted);
      }
      geometry_msgs::msg::Point skipped;
      skipped.x = kCanX;
      skipped.y = kCanY;
      skipped.z = kCanZ;
      goal.skip_mark_positions.push_back(skipped);
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.feed_blocked_candidates, 0U);
  EXPECT_EQ(sent.result.marked_feed_blocked_candidates, 1U);
  EXPECT_EQ(
    sent.result.feed_block_example,
    "can at (-0.420, -0.250) stands behind can at (-0.420, -0.165)");
  EXPECT_NE(sent.result.detail.find("stand behind a front"), std::string::npos)
    << sent.result.detail;
}

TEST_F(TraySurveyNodeTest, ReturnsATypedRefutationWhenTheCloseViewContradictsTheClass)
{
  // SC-003: a contradicted close-range identity is a typed refutation outcome, and the
  // confirmed observation stays empty, so nothing in the result authorizes a grasp.
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kContradictsClass);
  harness_->start_confirmation_publication();
  const auto sent = harness_->send([](SurveyTray::Goal &) {});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_REFUTED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_CLASS_MISMATCH);
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "");
  EXPECT_EQ(
    sent.result.refuting_observation.product_class,
    Observation::PRODUCT_CLASS_LARGE_BOTTLE);
  EXPECT_EQ(sent.result.refuting_observation.backend_name, "wrist_rgbd_tray_confirm");
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, RefutesAsAbsentWhenTheCloseViewSawTheTrayButNotTheCandidate)
{
  harness_->start_overview_publication();
  harness_->set_confirm_mode(ConfirmMode::kAbsent);
  harness_->start_confirmation_publication();
  const auto sent = harness_->send([](SurveyTray::Goal &) {});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_REFUTED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_ABSENT);
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "");
  EXPECT_EQ(sent.result.refuting_observation.header.frame_id, "");
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, ReportsAcquisitionFailureWhenTheConfirmDutyNeverAnswers)
{
  harness_->start_overview_publication();
  // Confirmation publication stays off: the viewpoint is reached and nothing arrives.
  const auto sent = harness_->send([](SurveyTray::Goal &) {});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_ACQUISITION_FAILED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(sent.result.confirmed_observation.header.frame_id, "");
  EXPECT_EQ(sent.result.refutation, SurveyTray::Result::REFUTATION_NONE);
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, ReportsNoCandidateWhenTheOverviewDutyStaysSilent)
{
  const auto sent = harness_->send([](SurveyTray::Goal &) {});
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_NO_CANDIDATE)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  ASSERT_EQ(sent.result.overview_stations_visited.size(), 2U);
  EXPECT_TRUE(sent.result.overview_candidates.empty());
  EXPECT_EQ(sent.result.selected_candidate.header.frame_id, "");
  // Both stations ran; no confirmation leg followed selection's failure.
  EXPECT_EQ(harness_->fake().leg_count(), 2U);
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
  // Card 050: even a fully silent run reports its stages — the taps exist and counted nothing,
  // which is the "input gap" reading of a zero-admitted station.
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (const auto & report : sent.result.overview_station_reports) {
    EXPECT_TRUE(report.image_tap_configured);
    EXPECT_TRUE(report.detection_tap_configured);
    EXPECT_EQ(report.images, 0U);
    EXPECT_EQ(report.detection_frames, 0U);
    EXPECT_EQ(report.frames_with_detections, 0U);
    EXPECT_EQ(report.published, 0U);
    EXPECT_EQ(report.admitted, 0U);
  }
}

// -- Card 050: the per-station stage receipt names the stage a zero dwell lost ---------------

TEST_F(TraySurveyNodeTest, StageReceiptReportsEveryStageOnTheHealthyPipeline)
{
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (std::size_t index = 0U; index < sent.result.overview_stations_visited.size(); ++index) {
    const auto & report = sent.result.overview_station_reports[index];
    EXPECT_EQ(report.station, sent.result.overview_stations_visited[index]);
    EXPECT_TRUE(report.image_tap_configured);
    EXPECT_TRUE(report.detection_tap_configured);
    EXPECT_GT(report.images, 0U) << report.station;
    EXPECT_GT(report.detection_frames, 0U) << report.station;
    EXPECT_EQ(report.frames_with_detections, report.detection_frames) << report.station;
    EXPECT_GE(report.published, report.admitted) << report.station;
    EXPECT_GT(report.admitted, 0U) << report.station;
    // A station that admitted at the first snapshot never drains.
    EXPECT_FALSE(report.drained) << report.station;
    EXPECT_EQ(report.late_images, 0U) << report.station;
    EXPECT_EQ(report.late_published, 0U) << report.station;
  }
}

TEST_F(TraySurveyNodeTest, AdmitsLateInWindowObservationsOnTheZeroPathDrain)
{
  // Card 050's drain: delivery that outlasts arrival_grace_ms. The fixture's windows are
  // dwell 50 + collection 250 = a 300 ms stamp window, with the first snapshot at +350 ms and
  // the drain to +600 ms. Observations stamped 450 ms behind the node clock are therefore all
  // still outside the window at the snapshot (arrivals there carry stamps <= window start) and
  // land inside the window only during the drain — exactly the frames the old arrival grace
  // lost. Admission of those frames is what this pins; without the drain the stations report
  // zero and this test is red.
  harness_->set_observation_stamp_lag(std::chrono::milliseconds(450));
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_OVERVIEW_ONLY)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (const auto & report : sent.result.overview_station_reports) {
    EXPECT_TRUE(report.drained) << report.station;
    EXPECT_GT(report.late_published, 0U) << report.station;
    EXPECT_GT(report.admitted, 0U)
      << report.station << ": the drain did not admit the late in-window frames (images="
      << report.images << " published=" << report.published << ")";
  }
  EXPECT_FALSE(sent.result.overview_candidates.empty());
}

TEST_F(TraySurveyNodeTest, StageReceiptNamesTheDutyWhenImagesArriveButNothingIsProcessed)
{
  // Images flow into the window and the duty never processes them: images > 0 with an empty
  // detection stream names the duty stage, not the input stream.
  harness_->set_stage_taps(true, false);
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (const auto & report : sent.result.overview_station_reports) {
    EXPECT_GT(report.images, 0U) << report.station;
    EXPECT_EQ(report.detection_frames, 0U) << report.station;
    EXPECT_EQ(report.frames_with_detections, 0U) << report.station;
    EXPECT_EQ(report.published, 0U) << report.station;
    EXPECT_EQ(report.admitted, 0U) << report.station;
  }
}

TEST_F(TraySurveyNodeTest, StageReceiptNamesEstimationWhenProposalsNeverPublish)
{
  // The duty processes frames and every acquisition proposes, yet nothing reaches the
  // candidates topic: estimation or identity dropped them (the duty's geometry-gate counters
  // name the gate on the live graph).
  harness_->set_stage_taps(true, true);
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (const auto & report : sent.result.overview_station_reports) {
    EXPECT_GT(report.images, 0U) << report.station;
    EXPECT_GT(report.detection_frames, 0U) << report.station;
    EXPECT_EQ(report.frames_with_detections, report.detection_frames) << report.station;
    EXPECT_EQ(report.published, 0U) << report.station;
    EXPECT_EQ(report.admitted, 0U) << report.station;
  }
}

TEST_F(TraySurveyNodeTest, StageReceiptNamesTheBackendWhenAForeignWriterPublishes)
{
  // Observations arrive in the window but carry a foreign backend: published > 0 against
  // admitted == 0 is the backend/status/stamp-window reading, not a dead stream.
  harness_->set_foreign_overview_backend(true);
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.overview_station_reports.size(), 2U);
  for (const auto & report : sent.result.overview_station_reports) {
    EXPECT_GT(report.images, 0U) << report.station;
    EXPECT_GT(report.published, 0U) << report.station;
    EXPECT_EQ(report.admitted, 0U) << report.station;
    EXPECT_TRUE(sent.result.overview_candidates.empty()) << report.station;
  }
}

// Milestone 10 §6, Card 069 (review N5a): absence probes at the node, end to end. The empty
// probe (-0.30, -0.80, 0.80) sits inside the tray's usable volume (world, harness transform),
// in tray_1's slice; the synthetic cans stand off the tray, clear of its line of sight.
geometry_msgs::msg::Point probe_point(const double x, const double y, const double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

TEST_F(TraySurveyNodeTest, AbsenceProbesGetOneVerdictEachOnACompletedOverview)
{
  harness_->start_overview_publication();
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      goal.overview_only = true;
      goal.absence_probe_positions.push_back(probe_point(kCanX, kCanY, kCanZ));
      goal.absence_probe_positions.push_back(probe_point(-0.30, -0.80, 0.80));
      goal.absence_probe_positions.push_back(probe_point(10.0, 10.0, 10.0));
      // 0.07 m from a candidate: beyond the merge radius, inside the node's shipped
      // absence_seen_radius_m default (0.10 m) — pins that default (review cmbrev069b note 2).
      goal.absence_probe_positions.push_back(probe_point(kCanX + 0.07, kCanY, kCanZ));
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.overview_stations_visited.size(), 2U);
  ASSERT_EQ(sent.result.absence_probe_verdicts.size(), 4U);
  EXPECT_EQ(sent.result.absence_probe_verdicts[0], SurveyTray::Result::PROBE_SEEN)
    << "a probe on an overview candidate is seen";
  EXPECT_EQ(sent.result.absence_probe_verdicts[1], SurveyTray::Result::PROBE_VIEWED_EMPTY)
    << "an empty place framed by a healthy station with a clear line of sight";
  EXPECT_EQ(sent.result.absence_probe_verdicts[2], SurveyTray::Result::PROBE_NOT_COVERED)
    << "a place no station frames";
  EXPECT_EQ(sent.result.absence_probe_verdicts[3], SurveyTray::Result::PROBE_SEEN)
    << "0.07 m from a candidate is seen at the shipped 0.10 m radius";
}

TEST_F(TraySurveyNodeTest, AbsenceProbesAreNeverEvaluatedOnAnIncompleteOverview)
{
  harness_->start_overview_publication();
  harness_->fake().set_mode(
    FakeViewpointServer::Mode::kBlockStationUntilCancel, "tray_2");
  const auto sent = harness_->send_and_cancel_on_station(
    "tray_2", [](SurveyTray::Goal & goal) {
      goal.overview_only = true;
      goal.absence_probe_positions.push_back(probe_point(-0.30, -0.80, 0.80));
    });
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_CANCELED);
  EXPECT_TRUE(sent.result.absence_probe_verdicts.empty())
    << "an overview that did not visit every station is never evidence";
}

TEST_F(TraySurveyNodeTest, AbsenceProbesAreNoEvidenceWhenTheOverviewDutyStaysSilent)
{
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {
      goal.overview_only = true;
      goal.absence_probe_positions.push_back(probe_point(-0.30, -0.80, 0.80));
    });
  ASSERT_TRUE(sent.accepted);
  ASSERT_EQ(sent.result.absence_probe_verdicts.size(), 1U);
  EXPECT_EQ(sent.result.absence_probe_verdicts[0], SurveyTray::Result::PROBE_NO_EVIDENCE)
    << "a station that admitted nothing is never evidence of absence";
}

TEST_F(TraySurveyNodeTest, CancelDuringTheSecondStationAbandonsItAndKeepsTheCandidates)
{
  // SC-002: cancellation abandons unvisited stations, retains the candidates already
  // collected, and leaves no active motion goal — the fake server's every leg is terminal by
  // the time the canceled result is delivered.
  harness_->start_overview_publication();
  harness_->fake().set_mode(
    FakeViewpointServer::Mode::kBlockStationUntilCancel, "tray_2");
  const auto sent = harness_->send_and_cancel_on_station(
    "tray_2", [](SurveyTray::Goal & goal) {goal.overview_only = true;});
  harness_->stop_publication();
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.code, static_cast<int>(rclcpp_action::ResultCode::CANCELED));
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_CANCELED)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  ASSERT_EQ(sent.result.overview_stations_visited.size(), 1U);
  EXPECT_EQ(sent.result.overview_stations_visited[0], "tray_1");
  // The evidence collected before the cancel is still in the result: read-only observations
  // are retained, not rolled back.
  EXPECT_FALSE(sent.result.overview_candidates.empty());
  for (const auto & candidate : sent.result.overview_candidates) {
    EXPECT_EQ(candidate.backend_name, "wrist_rgbd_tray_overview");
  }
  // At most two legs were ever sent (tray_1 completed, tray_2 at most started and was
  // canceled), and none is still running when the result arrives.
  EXPECT_LE(harness_->fake().leg_count(), 2U);
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, RefusesUnknownOverviewStationsWithoutTouchingMotion)
{
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_stations = {"tray_9"};});
  ASSERT_TRUE(sent.accepted);
  EXPECT_EQ(sent.result.outcome, SurveyTray::Result::OUTCOME_INVALID_REQUEST)
    << "outcome " << unsigned(sent.result.outcome) << ": " << sent.result.detail;
  EXPECT_EQ(harness_->fake().leg_count(), 0U);
  EXPECT_TRUE(harness_->fake().all_legs_terminal());
}

TEST_F(TraySurveyNodeTest, RefusesAnEmptyOverviewStationEntryAtGoalAcceptance)
{
  const auto sent = harness_->send(
    [](SurveyTray::Goal & goal) {goal.overview_stations = {""};});
  EXPECT_FALSE(sent.accepted);
  EXPECT_EQ(harness_->fake().leg_count(), 0U);
}

TEST_F(TraySurveyNodeTest, SensorFrustumDefaultsMatchTheShippedWristCamera)
{
  // The confirmation framing check runs against these declared defaults — the baseline launch
  // passes no sensor overrides — and ConfirmationViewpointConfig's own contract is that the
  // frustum is never defaulted, because a framing computation against a hardcoded sensor keeps
  // passing after the sensor changes. Pin the node's defaults to sensors.xacro the same way
  // test_confirmation_viewpoint pins the solver's config, so a resolution or FOV change in the
  // description cannot leave this node solving viewpoints for a camera that no longer ships.
  const std::string path = required_environment("RESTOCKER_TEST_SENSORS_XACRO");
  std::ifstream file(path);
  ASSERT_TRUE(static_cast<bool>(file)) << "could not read " << path;
  std::ostringstream buffer;
  buffer << file.rdbuf();
  const std::string text = buffer.str();

  const auto literal = [&text](const char * pattern) {
    std::smatch match;
    EXPECT_TRUE(std::regex_search(text, match, std::regex(pattern)));
    return match[1].str();
  };
  const auto width = std::stol(literal(R"(<width>\s*([0-9]+)\s*</width>)"));
  const auto height = std::stol(literal(R"(<height>\s*([0-9]+)\s*</height>)"));
  const auto horizontal_fov =
    std::stod(literal(R"(<horizontal_fov>\s*([0-9.eE+-]+)\s*</horizontal_fov>)"));
  const auto near_clip = std::stod(literal(R"(<near>\s*([0-9.eE+-]+)\s*</near>)"));
  const auto far_clip = std::stod(literal(R"(<far>\s*([0-9.eE+-]+)\s*</far>)"));

  const rclcpp::Node::SharedPtr node = harness_->tray_node();
  EXPECT_EQ(node->get_parameter("sensor_width_px").as_int(), width);
  EXPECT_EQ(node->get_parameter("sensor_height_px").as_int(), height);
  EXPECT_DOUBLE_EQ(node->get_parameter("sensor_horizontal_fov_rad").as_double(), horizontal_fov);
  EXPECT_DOUBLE_EQ(node->get_parameter("sensor_near_clip_m").as_double(), near_clip);
  EXPECT_DOUBLE_EQ(node->get_parameter("sensor_far_clip_m").as_double(), far_clip);
}

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    status = RUN_ALL_TESTS();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "tray survey node test harness failed: %s\n", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
