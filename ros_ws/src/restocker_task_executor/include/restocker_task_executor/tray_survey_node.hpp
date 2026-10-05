// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/survey_tray.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/perception_frame.hpp>
#include <restocker_interfaces/msg/tray_station_acquisition.hpp>
#include <restocker_perception/survey_stations.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_task_executor/tray_survey_logic.hpp"

namespace restocker_task_executor
{

// The coordinated read-only tray survey action server.
//
// It orchestrates and owns no motion of its own: every leg is a /survey_viewpoint goal, the
// same action a client would send by hand, so there is exactly one motion coordinator in the
// system. Perception is likewise reused — overview candidates come from the tray-overview
// duty's candidates topic, which no world state ingests, and the confirming observation is
// whatever the tray-confirm duty published on the admitted object topic. This node publishes
// no observation and mutates no world state; its result reports the evidence the existing
// producers already wrote.
//
// The action-server callbacks return immediately and the survey runs on a worker thread, which
// is what lets a cancellation arrive while a leg is in flight. The canceled result is not
// delivered until the child goal has reached a terminal state, so no motion goal outlives this
// action.
class TraySurveyNode
{
public:
  explicit TraySurveyNode(const rclcpp::NodeOptions & options);

  TraySurveyNode(const TraySurveyNode &) = delete;
  TraySurveyNode & operator=(const TraySurveyNode &) = delete;
  TraySurveyNode(TraySurveyNode &&) = delete;
  TraySurveyNode & operator=(TraySurveyNode &&) = delete;

  ~TraySurveyNode();

  [[nodiscard]] rclcpp::Node::SharedPtr node() const {return node_;}

private:
  using SurveyTray = restocker_interfaces::action::SurveyTray;
  using SurveyViewpoint = restocker_interfaces::action::SurveyViewpoint;
  using TrayObservation = restocker_interfaces::msg::ObjectObservation;
  using TrayGoalHandle = rclcpp_action::ServerGoalHandle<SurveyTray>;
  using ViewpointGoalHandle = rclcpp_action::ClientGoalHandle<SurveyViewpoint>;

  // What a child /survey_viewpoint leg produced, normalized for the survey's own outcome
  // mapping. Not a SurveyViewpoint result: a leg that never ran and a leg that was abandoned
  // to this survey's cancellation are different things to the caller.
  struct LegResult
  {
    // False when the leg never got a handle: the server refused it, or this survey stopped
    // before the goal left this node.
    bool ran{false};
    // True when this survey's own stop is why the leg ended (or never began).
    bool stopped_by_cancel{false};
    std::uint8_t outcome{SurveyViewpoint::Result::OUTCOME_UNSET};
    std::string detail;
    // First-hand motion evidence propagated from the leg's own result (Milestone 10 §6,
    // Card 052). evidence_not_started: the leg's machinery proves no joint command was
    // issued. evidence_terminal_stop: the backend established the leg's terminal stop, or
    // the leg arrived (controllers reported success). A leg that produced no result carries
    // neither flag.
    bool evidence_not_started{false};
    bool evidence_terminal_stop{false};
  };

  template<typename Value>
  [[nodiscard]] Value declare(const std::string & name, const Value & fallback)
  {
    return node_->has_parameter(name) ? node_->get_parameter(name).get_value<Value>() :
           node_->declare_parameter<Value>(name, fallback);
  }

  [[nodiscard]] bool stop_requested() const;
  // True only while this node is going away (destructor), not on a client cancel: a cancel
  // must keep waiting for the child leg, a shutdown must not, because nothing will complete
  // the child's result once the executor has stopped or this node is being destroyed.
  [[nodiscard]] bool shutdown_requested() const;
  void request_stop(bool canceling);
  void cancel_child();

  [[nodiscard]] rclcpp_action::GoalResponse handle_goal(const SurveyTray::Goal & goal);
  void start(const std::shared_ptr<TrayGoalHandle> & handle);
  void run(const std::shared_ptr<TrayGoalHandle> & handle);
  void execute(
    const std::shared_ptr<TrayGoalHandle> & handle, const SurveyTray::Goal & goal,
    SurveyTray::Result & result);
  [[nodiscard]] LegResult run_leg(const SurveyViewpoint::Goal & goal);
  // Folds one leg's first-hand motion evidence into this attempt's result (Milestone 10 §6,
  // Card 052): a leg that never issued a command leaves the prior state untouched, a leg
  // that may have moved clears the not-started claim, and the stop flag takes the leg's own
  // verdict (arrival counts — the controllers reported success).
  static void note_leg_evidence(const LegResult & leg, SurveyTray::Result & result);
  void finish_early_stop(SurveyTray::Result & result);
  // Milestone 10 §6, Card 069: per-probe absence verdicts from the completed overview. Leaves
  // the verdicts empty (never evidence) when the planning <- shelf transform is unavailable.
  void classify_absence_probes(const SurveyTray::Goal & goal, SurveyTray::Result & result);
  [[nodiscard]] bool sleep_until(const rclcpp::Time & deadline);

  // One station's whole dwell-window accounting, taken under a single lock so the stage counts
  // and the admitted frames describe the same instant (Card 050's receipt). `admitted` is what
  // the survey already counted and merged; `report` is what the result and the log line carry.
  struct StationAcquisition
  {
    restocker_interfaces::msg::TrayStationAcquisition report;
    std::vector<TrayObservation> admitted;
  };
  [[nodiscard]] StationAcquisition acquire_station(
    const std::string & station, const rclcpp::Time & stopped,
    const std::chrono::milliseconds & stamp_window);
  [[nodiscard]] std::vector<TrayObservation> snapshot_window(
    const std::vector<TrayObservation> & buffer, const std::string & backend_name,
    const rclcpp::Time & stopped, const std::chrono::milliseconds & stamp_window);

  void terminate_outstanding_goal() noexcept;
  void finish(
    const std::shared_ptr<TrayGoalHandle> & handle,
    const std::shared_ptr<SurveyTray::Result> & result, bool canceled);

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string shelf_frame_;
  std::string survey_action_name_;
  std::string overview_topic_;
  std::string confirmation_topic_;
  // Card 050's stage taps: the raw wrist colour stream (did frames arrive in the window?) and
  // the overview duty's detection topic (did the duty process them, and did anything propose?).
  // Empty disables the tap, which the report records rather than counting as zero evidence.
  std::string overview_image_topic_;
  std::string overview_detection_topic_;
  std::chrono::milliseconds dwell_{150};
  std::chrono::milliseconds overview_collection_{1000};
  std::chrono::milliseconds arrival_grace_{150};
  std::chrono::milliseconds confirmation_timeout_{3000};
  TraySurveyLogicConfig logic_;
  restocker_perception::ConfirmationViewpointConfig confirm_;
  AbsenceProbeConfig absence_;
  std::vector<restocker_perception::SurveyStation> stations_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<TrayObservation>::SharedPtr overview_subscription_;
  rclcpp::Subscription<TrayObservation>::SharedPtr confirmation_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr overview_image_subscription_;
  rclcpp::Subscription<restocker_interfaces::msg::PerceptionFrame>::SharedPtr
    overview_detection_subscription_;
  rclcpp_action::Client<SurveyViewpoint>::SharedPtr viewpoint_client_;
  rclcpp_action::Server<SurveyTray>::SharedPtr server_;
  std::thread worker_;
  mutable std::mutex mutex_;
  bool busy_{false};
  bool cancel_requested_{false};
  bool shutdown_{false};
  bool collecting_{false};
  bool child_cancel_requested_{false};
  std::vector<TrayObservation> overview_buffer_;
  std::vector<TrayObservation> confirmation_buffer_;
  // Stage-tap receipts for the current goal: image and detection-frame records, stamped with
  // when they arrived so a zero dwell can say whether an in-window frame came late or never
  // existed. Records only — the survey never reads the image payload.
  struct StageFrameRecord
  {
    rclcpp::Time stamp;
    rclcpp::Time received_at;
    std::uint32_t proposals{0U};
  };
  std::vector<StageFrameRecord> overview_image_frames_;
  std::vector<StageFrameRecord> overview_detection_frames_;
  std::weak_ptr<TrayGoalHandle> outstanding_;
  std::shared_ptr<ViewpointGoalHandle> child_;
};

}  // namespace restocker_task_executor
