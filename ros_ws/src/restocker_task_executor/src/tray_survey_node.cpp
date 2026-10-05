// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// The coordinated read-only tray survey: overview stations, one selected candidate, one
// confirmation viewpoint, and a typed answer. See tray_survey_node.hpp for the contract; this
// file is the wiring.

#include "restocker_task_executor/tray_survey_node.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] geometry_msgs::msg::PoseStamped stamped(
  const std::string & frame_id, const rclcpp::Time & stamp, const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::PoseStamped message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.pose = tf2::toMsg(pose);
  return message;
}

void trim(std::vector<restocker_interfaces::msg::ObjectObservation> & buffer)
{
  // Defensive: a goal runs for minutes at a few hertz, but a stuck clock must not grow an
  // unbounded buffer.
  constexpr std::size_t kMaximumBuffered = 2000U;
  if (buffer.size() > kMaximumBuffered) {
    buffer.erase(
      buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(buffer.size() / 2U));
  }
}

// Maps a /survey_viewpoint result onto "this leg's machinery proves no joint command was
// issued" (Milestone 10 §6, Card 052). Mirrors survey_definitely_not_started for the
// viewpoint action's own outcome constants: the action contract states which outcomes
// never commanded motion.
[[nodiscard]] bool survey_viewpoint_leg_not_started(std::uint8_t outcome)
{
  using ViewpointResult = restocker_interfaces::action::SurveyViewpoint::Result;
  switch (outcome) {
    case ViewpointResult::OUTCOME_VIEWPOINT_UNRESOLVED:
    case ViewpointResult::OUTCOME_PLANNING_FAILED:
    case ViewpointResult::OUTCOME_UNAVAILABLE:
    case ViewpointResult::OUTCOME_INVALID_REQUEST:
      return true;
    default:
      // ARRIVED, EXECUTION_FAILED, CANCELED, TIMED_OUT, UNSET, and SHUTDOWN (review F2:
      // a server shutdown abort claims neither no-motion nor a stop): the leg may have
      // moved, or the outcome says nothing — never claim not-started on silence.
      return false;
  }
}

}  // namespace

TraySurveyNode::TraySurveyNode(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("tray_survey", options))
{
  planning_frame_ = declare("planning_frame", std::string("world"));
  shelf_frame_ = declare("shelf_frame", std::string(restocker_perception::kShelfFrame));
  const std::string geometry_path = declare("workcell_geometry_path", std::string());
  survey_action_name_ = declare("survey_action_name", std::string("/survey_viewpoint"));
  overview_topic_ = declare("overview_topic", std::string("/perception/tray_candidates"));
  confirmation_topic_ =
    declare("confirmation_topic", std::string("/perception/object_observations"));
  // Card 050's stage taps. Both default to the shipped wrist topics so the receipt exists on
  // every composition without launch edits; empty disables a tap, which the report records as
  // unconfigured instead of as a zero. The subscriptions mirror the overview duty's own sensor
  // QoS (reliable, keep last 30 — six seconds at the sensor's rate): a receipt that dropped
  // frames its own reader was late for would misreport an input gap, and the shipped bridge
  // publishes reliable (the duty relies on the same fact).
  overview_image_topic_ = declare("overview_image_topic", std::string("/wrist_camera/image"));
  overview_detection_topic_ =
    declare("overview_detection_topic", std::string("/perception/tray_overview_detections"));
  if (geometry_path.empty()) {
    throw std::invalid_argument(
            "workcell_geometry_path must name the surveyed workcell geometry: the default "
            "overview stations and the confirmation approach direction are derived from it");
  }
  if (survey_action_name_.empty() || overview_topic_.empty() || confirmation_topic_.empty()) {
    throw std::invalid_argument("tray survey topic and action names must not be empty");
  }
  stations_ = restocker_perception::nominal_survey_stations(
    restocker_perception::load_workcell_survey_geometry(geometry_path));

  logic_.overview_backend_name = declare("overview_backend_name", logic_.overview_backend_name);
  logic_.confirmation_backend_name =
    declare("confirmation_backend_name", logic_.confirmation_backend_name);
  logic_.candidate_merge_radius_m =
    declare("candidate_merge_radius_m", logic_.candidate_merge_radius_m);
  logic_.association_radius_m = declare("association_radius_m", logic_.association_radius_m);
  if (!(logic_.candidate_merge_radius_m > 0.0) || !(logic_.association_radius_m > 0.0)) {
    throw std::invalid_argument("tray survey radii must be positive");
  }
  // The merge radius is also the feed rule's same-product radius (Milestone 10 §6, Card 066): at
  // or beyond the closest two catalogued upright products can stand (0.066 m) a product directly
  // ahead would be taken for the candidate itself and stop blocking it.
  if (!(logic_.candidate_merge_radius_m < 0.066)) {
    throw std::invalid_argument(
            "candidate_merge_radius_m must stay below 0.066 m, the closest two catalogued "
            "products can stand");
  }
  // Milestone 10 §6, Card 066: never narrower than the coordinator's 0.5·r for any catalogued
  // product, never wide enough to merge two upright products standing side by side (≥ 0.066 m).
  logic_.feed_column_half_width_m =
    declare("feed_column_half_width_m", logic_.feed_column_half_width_m);
  if (!(logic_.feed_column_half_width_m > 0.0) || !(logic_.feed_column_half_width_m <= 0.05)) {
    throw std::invalid_argument("feed_column_half_width_m must be in (0, 0.05] m");
  }

  // How long after a leg completes before frames are allowed to count, how long overview
  // frames are gathered, how much arrival slack the transport gets on top, and how long the
  // confirmation leg waits for the confirm duty to answer. The dwell exists so the first
  // counted frame is rendered with the arm already stopped; the grace exists because a frame
  // stamped inside the window may arrive just after the window closes.
  dwell_ = std::chrono::milliseconds(declare("acquisition_dwell_ms", static_cast<int>(150)));
  overview_collection_ =
    std::chrono::milliseconds(declare("overview_collection_ms", static_cast<int>(1000)));
  arrival_grace_ = std::chrono::milliseconds(declare("arrival_grace_ms", static_cast<int>(150)));
  confirmation_timeout_ =
    std::chrono::milliseconds(declare("confirmation_timeout_ms", static_cast<int>(3000)));
  if (dwell_.count() < 0 || overview_collection_.count() <= 0 || arrival_grace_.count() < 0 ||
    confirmation_timeout_.count() <= 0)
  {
    throw std::invalid_argument("tray survey timing parameters are invalid");
  }

  confirm_.standoff_m = declare("confirm_standoff_m", confirm_.standoff_m);
  confirm_.minimum_standoff_m = declare("confirm_minimum_standoff_m", confirm_.minimum_standoff_m);
  confirm_.maximum_standoff_m = declare("confirm_maximum_standoff_m", confirm_.maximum_standoff_m);
  const double approach_setback = declare("confirm_approach_setback_m", 0.45);
  const double approach_height = declare("confirm_approach_height_m", 0.555);
  confirm_.approach_in_shelf = Eigen::Vector3d(0.0, approach_setback, approach_height);
  confirm_.minimum_elevation_rad =
    declare("confirm_minimum_elevation_rad", confirm_.minimum_elevation_rad);
  const auto aim_bias =
    declare("confirm_aim_bias_xyz_m", std::vector<double>{0.0, 0.0, 0.05});
  if (aim_bias.size() != 3U) {
    throw std::invalid_argument("confirm_aim_bias_xyz_m must carry three values");
  }
  confirm_.aim_bias_in_shelf = Eigen::Vector3d(aim_bias[0], aim_bias[1], aim_bias[2]);
  confirm_.frustum.width_px =
    static_cast<std::uint32_t>(declare("sensor_width_px", static_cast<int>(1280)));
  confirm_.frustum.height_px =
    static_cast<std::uint32_t>(declare("sensor_height_px", static_cast<int>(720)));
  confirm_.frustum.horizontal_fov_rad = declare("sensor_horizontal_fov_rad", 1.48);
  confirm_.frustum.near_clip_m = declare("sensor_near_clip_m", 0.05);
  confirm_.frustum.far_clip_m = declare("sensor_far_clip_m", 2.5);
  confirm_.required_margin_px = declare("framing_required_margin_px", 30.0);
  const auto framing_box =
    declare("framing_box_size_xyz_m", std::vector<double>{0.09, 0.09, 0.29});
  if (framing_box.size() != 3U) {
    throw std::invalid_argument("framing_box_size_xyz_m must carry three values");
  }
  confirm_.framing_box_size = Eigen::Vector3d(framing_box[0], framing_box[1], framing_box[2]);

  // Milestone 10 §6, Card 069: the absence probes read the same sensor model; "seen" defaults
  // to the association radius and is never narrower than the merge radius.
  absence_.frustum = confirm_.frustum;
  absence_.seen_radius_m = declare("absence_seen_radius_m", logic_.association_radius_m);
  if (absence_.seen_radius_m < logic_.candidate_merge_radius_m) {
    throw std::invalid_argument("absence_seen_radius_m must not be below candidate_merge_radius_m");
  }
  absence_.occluder_radius_m = declare("absence_occluder_radius_m", absence_.occluder_radius_m);
  absence_.framing_margin_px = declare("absence_framing_margin_px", absence_.framing_margin_px);
  if (!(absence_.occluder_radius_m > 0.0) || !(absence_.framing_margin_px >= 0.0)) {
    throw std::invalid_argument("tray survey absence probe parameters are invalid");
  }

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, true);

  overview_subscription_ = node_->create_subscription<TrayObservation>(
    overview_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    [this](TrayObservation::ConstSharedPtr message) {
      std::scoped_lock lock(mutex_);
      if (!collecting_) {
        return;
      }
      overview_buffer_.push_back(*message);
      trim(overview_buffer_);
    });
  confirmation_subscription_ = node_->create_subscription<TrayObservation>(
    confirmation_topic_, rclcpp::QoS(rclcpp::KeepLast(50)).reliable(),
    [this](TrayObservation::ConstSharedPtr message) {
      std::scoped_lock lock(mutex_);
      if (!collecting_) {
        return;
      }
      confirmation_buffer_.push_back(*message);
      trim(confirmation_buffer_);
    });
  if (!overview_image_topic_.empty()) {
    overview_image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
      overview_image_topic_, rclcpp::QoS(rclcpp::KeepLast(30)).reliable(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        if (!collecting_) {
          return;
        }
        overview_image_frames_.push_back(
          StageFrameRecord{rclcpp::Time(message->header.stamp), node_->now(), 0U});
        constexpr std::size_t kMaximumTrackedStamps = 4000U;
        if (overview_image_frames_.size() > kMaximumTrackedStamps) {
          overview_image_frames_.erase(
            overview_image_frames_.begin(),
            overview_image_frames_.begin() +
            static_cast<std::ptrdiff_t>(overview_image_frames_.size() / 2U));
        }
      });
  }
  if (!overview_detection_topic_.empty()) {
    overview_detection_subscription_ =
      node_->create_subscription<restocker_interfaces::msg::PerceptionFrame>(
      overview_detection_topic_, rclcpp::QoS(rclcpp::KeepLast(30)).reliable(),
      [this](restocker_interfaces::msg::PerceptionFrame::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        if (!collecting_) {
          return;
        }
        overview_detection_frames_.push_back(
          StageFrameRecord{
          rclcpp::Time(message->header.stamp), node_->now(),
          static_cast<std::uint32_t>(message->detections.size())});
        constexpr std::size_t kMaximumTrackedStamps = 4000U;
        if (overview_detection_frames_.size() > kMaximumTrackedStamps) {
          overview_detection_frames_.erase(
            overview_detection_frames_.begin(),
            overview_detection_frames_.begin() +
            static_cast<std::ptrdiff_t>(overview_detection_frames_.size() / 2U));
        }
      });
  }

  viewpoint_client_ = rclcpp_action::create_client<SurveyViewpoint>(node_, survey_action_name_);

  // The server's own callbacks stay non-blocking: handle_accepted starts the worker and
  // returns, which frees the default callback group for the cancel callback while a leg is in
  // flight.
  server_ = rclcpp_action::create_server<SurveyTray>(
    node_, "survey_tray",
    [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const SurveyTray::Goal> goal) {
      return handle_goal(*goal);
    },
    [this](const std::shared_ptr<TrayGoalHandle> &) {
      request_stop(true);
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [this](const std::shared_ptr<TrayGoalHandle> & handle) {start(handle);});
  RCLCPP_INFO(
    node_->get_logger(),
    "tray survey server ready on /survey_tray: overview candidates from %s, confirmation on "
    "%s through %s", overview_topic_.c_str(), confirmation_topic_.c_str(),
    survey_action_name_.c_str());
}

TraySurveyNode::~TraySurveyNode()
{
  {
    std::scoped_lock lock(mutex_);
    shutdown_ = true;
  }
  // Abandon the child leg first so the worker's wait ends, then join it: the worker delivers
  // this goal's own terminal result, and only a goal the worker never started is left for
  // terminate_outstanding_goal.
  cancel_child();
  if (worker_.joinable()) {
    worker_.join();
  }
  terminate_outstanding_goal();
}

bool TraySurveyNode::stop_requested() const
{
  std::scoped_lock lock(mutex_);
  return cancel_requested_ || shutdown_;
}

bool TraySurveyNode::shutdown_requested() const
{
  std::scoped_lock lock(mutex_);
  return shutdown_;
}

void TraySurveyNode::request_stop(const bool canceling)
{
  {
    std::scoped_lock lock(mutex_);
    if (canceling) {
      cancel_requested_ = true;
    } else {
      shutdown_ = true;
    }
  }
  cancel_child();
}

void TraySurveyNode::cancel_child()
{
  std::shared_ptr<ViewpointGoalHandle> child;
  bool already_requested;
  {
    std::scoped_lock lock(mutex_);
    child = child_;
    already_requested = child_cancel_requested_;
    if (child && !already_requested) {
      child_cancel_requested_ = true;
    }
  }
  if (!child || already_requested) {
    return;
  }
  try {
    viewpoint_client_->async_cancel_goal(child);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      node_->get_logger(), "could not cancel the outstanding %s goal: %s",
      survey_action_name_.c_str(), error.what());
  }
}

rclcpp_action::GoalResponse TraySurveyNode::handle_goal(const SurveyTray::Goal & goal)
{
  for (const std::string & station : goal.overview_stations) {
    if (station.empty()) {
      RCLCPP_WARN(
        node_->get_logger(), "rejecting a tray survey whose overview station list is empty");
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  std::scoped_lock lock(mutex_);
  if (busy_) {
    RCLCPP_WARN(node_->get_logger(), "rejecting a tray survey while another is outstanding");
    return rclcpp_action::GoalResponse::REJECT;
  }
  busy_ = true;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void TraySurveyNode::start(const std::shared_ptr<TrayGoalHandle> & handle)
{
  {
    std::scoped_lock lock(mutex_);
    outstanding_ = handle;
    cancel_requested_ = false;
    collecting_ = true;
    overview_buffer_.clear();
    confirmation_buffer_.clear();
    overview_image_frames_.clear();
    overview_detection_frames_.clear();
    child_.reset();
    child_cancel_requested_ = false;
  }
  // A previous worker has already delivered its result (busy_ was false at goal acceptance),
  // so this join is a formality that keeps exactly one worker per node.
  if (worker_.joinable()) {
    worker_.join();
  }
  worker_ = std::thread([this, handle] {run(handle);});
}

void TraySurveyNode::run(const std::shared_ptr<TrayGoalHandle> & handle)
{
  auto result = std::make_shared<SurveyTray::Result>();
  const SurveyTray::Goal goal = *handle->get_goal();
  try {
    execute(handle, goal, *result);
  } catch (const std::exception & error) {
    result->outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
    result->detail = std::string("tray survey failed: ") + error.what();
    RCLCPP_ERROR(node_->get_logger(), "%s", result->detail.c_str());
  }
  {
    std::scoped_lock lock(mutex_);
    collecting_ = false;
  }
  finish(handle, result, result->outcome == SurveyTray::Result::OUTCOME_CANCELED);
}

void TraySurveyNode::execute(
  const std::shared_ptr<TrayGoalHandle> & handle, const SurveyTray::Goal & goal,
  SurveyTray::Result & result)
{
  // Pre-motion default (Milestone 10 §6, Card 052): until some leg's own machinery may have
  // issued a joint command, this attempt proves no motion; every leg below replaces the
  // flags through note_leg_evidence when it ran.
  result.motion_definitely_not_started = true;
  result.execution_reached_terminal_stop = false;
  result.failed_station.clear();
  std::vector<std::string> stations;
  if (goal.overview_stations.empty()) {
    for (const restocker_perception::SurveyStation & station : stations_) {
      if (station.name.starts_with("tray_")) {
        stations.push_back(station.name);
      }
    }
    if (stations.empty()) {
      result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
      result.detail = "the workcell geometry derives no tray overview stations";
      return;
    }
  } else {
    stations = goal.overview_stations;
    for (const std::string & name : stations) {
      if (restocker_perception::find_survey_station(stations_, name) == nullptr) {
        result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
        result.detail = "no nominal survey station named " + name;
        RCLCPP_WARN(node_->get_logger(), "tray survey refused: %s", result.detail.c_str());
        return;
      }
    }
  }

  if (!viewpoint_client_->wait_for_action_server(std::chrono::seconds(2))) {
    result.outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
    result.detail = "the " + survey_action_name_ + " action server is not available";
    RCLCPP_ERROR(node_->get_logger(), "%s", result.detail.c_str());
    return;
  }

  std::vector<TrayObservation> candidates;
  auto feedback = std::make_shared<SurveyTray::Feedback>();
  for (const std::string & station : stations) {
    // Kept current so every early exit reports the candidates collected so far: cancellation
    // abandons unvisited stations, it does not discard the evidence already in hand.
    result.overview_candidates = candidates;
    if (stop_requested()) {
      finish_early_stop(result);
      return;
    }
    feedback->phase = SurveyTray::Feedback::PHASE_OVERVIEW;
    feedback->current_station = station;
    feedback->candidates_collected = static_cast<std::uint32_t>(candidates.size());
    handle->publish_feedback(feedback);

    SurveyViewpoint::Goal leg;
    leg.station = station;
    leg.label = "tray overview " + station;
    leg.position_tolerance_m = goal.position_tolerance_m;
    leg.orientation_tolerance_rad = goal.orientation_tolerance_rad;
    const LegResult leg_result = run_leg(leg);
    note_leg_evidence(leg_result, result);
    if (leg_result.stopped_by_cancel) {
      finish_early_stop(result);
      return;
    }
    if (!leg_result.ran) {
      result.outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
      result.detail = leg_result.detail;
      result.failed_station = station;
      RCLCPP_ERROR(node_->get_logger(), "tray survey aborted: %s", result.detail.c_str());
      return;
    }
    if (leg_result.outcome != SurveyViewpoint::Result::OUTCOME_ARRIVED) {
      if (leg_result.outcome == SurveyViewpoint::Result::OUTCOME_CANCELED) {
        // The leg was canceled without this survey having requested it: the survey cannot
        // continue, and the verdict is a cancellation either way.
        result.outcome = SurveyTray::Result::OUTCOME_CANCELED;
        result.failed_station = station;
        if (result.detail.empty()) {
          result.detail = "an overview leg was canceled before the tray survey completed";
        }
        return;
      }
      result.outcome = SurveyTray::Result::OUTCOME_VIEWPOINT_UNREACHED;
      result.detail = "overview station " + station + ": " + leg_result.detail;
      result.failed_station = station;
      RCLCPP_WARN(node_->get_logger(), "tray survey aborted: %s", result.detail.c_str());
      return;
    }
    result.overview_stations_visited.push_back(station);

    // The instant this leg's motion completed. Frames counted for this station must be
    // stamped after it, so they cannot predate the arm settling, and no further motion has
    // begun, so every such frame was taken from this station.
    const rclcpp::Time stopped = node_->now();
    if (!sleep_until(stopped + dwell_ + overview_collection_ + arrival_grace_)) {
      result.overview_candidates = candidates;
      finish_early_stop(result);
      return;
    }
    // One lock for every count, so the stage receipt and the admitted frames describe the same
    // instant (Card 050: a zero admitted count has to name the stage that lost the frames).
    StationAcquisition acquisition =
      acquire_station(station, stopped, dwell_ + overview_collection_);
    // Card 050's zero-dwell drain (Stage 4, spec-first): a station that would declare zero gets
    // one further overview_collection_ms of ARRIVAL time for frames stamped inside the original
    // window — the stamp window never grows, only the arrival deadline of a would-be-zero
    // station moves. A frame whose delivery outlasted arrival_grace_ms is counted instead of
    // lost; a station the camera never spoke for still reports zero, and the drain line says
    // which of the three it was.
    if (acquisition.admitted.empty() && !stop_requested()) {
      const auto before = acquisition.report;
      if (!sleep_until(node_->now() + overview_collection_)) {
        result.overview_candidates = candidates;
        finish_early_stop(result);
        return;
      }
      StationAcquisition drained =
        acquire_station(station, stopped, dwell_ + overview_collection_);
      drained.report.drained = true;
      drained.report.late_images = drained.report.images - before.images;
      drained.report.late_published = drained.report.published - before.published;
      const char * classification = classify_drained_station(drained.report);
      RCLCPP_INFO(
        node_->get_logger(),
        "zero-dwell drain for %s: one further overview_collection window -> admitted=%u; "
        "in-window arrivals after the snapshot: images=%u published=%u — %s",
        station.c_str(), drained.report.admitted, drained.report.late_images,
        drained.report.late_published, classification);
      acquisition = std::move(drained);
    }
    candidates = merge_candidates(
      candidates, acquisition.admitted, logic_.candidate_merge_radius_m);
    result.overview_candidates = candidates;
    result.overview_station_reports.push_back(acquisition.report);
    const auto & report = acquisition.report;
    const auto stage_value = [](const bool configured, const std::uint32_t value) {
      return configured ? std::to_string(value) : std::string("n/a");
    };
    RCLCPP_INFO(
      node_->get_logger(),
      "overview of %s: %zu frame(s) from %s, %zu distinct candidate(s) so far | stages: "
      "images=%s detection_frames=%s frames_with_detections=%s published=%u admitted=%u",
      station.c_str(), acquisition.admitted.size(), logic_.overview_backend_name.c_str(),
      candidates.size(), stage_value(report.image_tap_configured, report.images).c_str(),
      stage_value(report.detection_tap_configured, report.detection_frames).c_str(),
      stage_value(report.detection_tap_configured, report.frames_with_detections).c_str(),
      report.published, report.admitted);
  }

  result.overview_candidates = candidates;
  // Every requested station was visited: only now is the overview evidence about absence.
  classify_absence_probes(goal, result);
  feedback->current_station.clear();
  feedback->candidates_collected = static_cast<std::uint32_t>(candidates.size());

  if (goal.overview_only) {
    result.outcome = SurveyTray::Result::OUTCOME_OVERVIEW_ONLY;
    result.detail = "overview only: " + std::to_string(candidates.size()) +
      " candidate(s) collected; nothing was confirmed and nothing authorizes a grasp";
    return;
  }

  feedback->phase = SurveyTray::Feedback::PHASE_SELECTING;
  handle->publish_feedback(feedback);

  // The shelf frame is looked up before selection: feed order (Milestone 10 §6, Card 066) is
  // judged in it, and the confirmation viewpoint below is derived from it.
  geometry_msgs::msg::TransformStamped shelf_transform;
  try {
    shelf_transform = tf_buffer_->lookupTransform(
      planning_frame_, shelf_frame_, tf2::TimePointZero, tf2::durationFromSec(5.0));
  } catch (const tf2::TransformException & error) {
    result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
    result.detail = "could not resolve " + planning_frame_ + " <- " + shelf_frame_ + ": " +
      error.what();
    return;
  }
  const Eigen::Isometry3d planning_from_shelf = tf2::transformToEigen(shelf_transform);
  FeedOrder feed;
  feed.shelf_from_planning = planning_from_shelf.inverse();
  feed.column_half_width_m = logic_.feed_column_half_width_m;
  feed.same_product_radius_m = logic_.candidate_merge_radius_m;
  const CandidateSelection selection = select_feed_front_candidate(
    candidates, goal.product_class, goal.has_sku, goal.sku, goal.refuted_positions,
    logic_.candidate_merge_radius_m, feed, goal.skip_mark_positions);
  result.feed_blocked_candidates = selection.feed_blocked_candidates;
  result.marked_feed_blocked_candidates = selection.marked_feed_blocked_candidates;
  result.feed_block_example = selection.feed_block_example;
  // The radius this goal's marks were applied with, so the campaign matches them the same way
  // (review cmbrev066b N2).
  result.candidate_merge_radius_m = logic_.candidate_merge_radius_m;
  // Card 066 × 058: the campaign charges a skip-mark lift only for a product that would be a
  // feed-column front once lifted; the geometry is judged here, where the shelf frame is.
  result.overview_candidate_feed_front = feed_column_fronts(candidates, feed);
  const std::optional<TrayObservation> & selected = selection.selected;
  if (!selected) {
    result.outcome = SurveyTray::Result::OUTCOME_NO_CANDIDATE;
    result.detail = no_candidate_detail(
      classify_no_candidate(
        candidates, goal.product_class, goal.has_sku, goal.sku, goal.refuted_positions,
        goal.skip_mark_positions, logic_.candidate_merge_radius_m),
      selection.feed_blocked_candidates);
    if (selection.marked_feed_blocked_candidates > 0U) {
      result.detail += "; " + std::to_string(selection.marked_feed_blocked_candidates) +
        " marked candidate(s) stand behind a front: " + selection.feed_block_example;
    }
    RCLCPP_INFO(node_->get_logger(), "tray survey found no candidate: %s", result.detail.c_str());
    return;
  }
  result.selected_candidate = *selected;
  if (selected->header.frame_id != planning_frame_) {
    result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
    result.detail = "the selected candidate is expressed in " + selected->header.frame_id +
      ", not in the planning frame " + planning_frame_;
    return;
  }

  if (stop_requested()) {
    finish_early_stop(result);
    return;
  }

  const restocker_perception::Result<restocker_perception::FramedTransform> transform =
    restocker_perception::FramedTransform::create(
    shelf_frame_, planning_frame_, planning_from_shelf);
  if (!transform) {
    result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
    result.detail = transform.error().detail;
    return;
  }
  const Eigen::Vector3d target(
    selected->pose.pose.position.x, selected->pose.pose.position.y,
    selected->pose.pose.position.z);
  const restocker_perception::Result<restocker_perception::CameraViewpoint> viewpoint =
    restocker_perception::confirmation_viewpoint(
    target, transform.value(), confirm_, "tray confirm");
  if (!viewpoint) {
    result.outcome = SurveyTray::Result::OUTCOME_INVALID_REQUEST;
    result.detail = viewpoint.error().detail;
    RCLCPP_WARN(node_->get_logger(), "tray survey refused: %s", result.detail.c_str());
    return;
  }

  feedback->phase = SurveyTray::Feedback::PHASE_CONFIRMING;
  handle->publish_feedback(feedback);
  if (stop_requested()) {
    finish_early_stop(result);
    return;
  }

  SurveyViewpoint::Goal leg;
  leg.camera_optical_pose = stamped(planning_frame_, node_->now(), viewpoint.value().pose.pose);
  leg.label = viewpoint.value().label;
  leg.position_tolerance_m = goal.position_tolerance_m;
  leg.orientation_tolerance_rad = goal.orientation_tolerance_rad;
  const LegResult leg_result = run_leg(leg);
  note_leg_evidence(leg_result, result);
  if (leg_result.stopped_by_cancel) {
    finish_early_stop(result);
    return;
  }
  if (!leg_result.ran) {
    result.outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
    result.detail = leg_result.detail;
    result.failed_station = "confirmation";
    RCLCPP_ERROR(node_->get_logger(), "tray survey aborted: %s", result.detail.c_str());
    return;
  }
  if (leg_result.outcome != SurveyViewpoint::Result::OUTCOME_ARRIVED) {
    if (leg_result.outcome == SurveyViewpoint::Result::OUTCOME_CANCELED) {
      result.outcome = SurveyTray::Result::OUTCOME_CANCELED;
      result.failed_station = "confirmation";
      if (result.detail.empty()) {
        result.detail = "the confirmation leg was canceled before the tray survey completed";
      }
      return;
    }
    result.outcome = SurveyTray::Result::OUTCOME_VIEWPOINT_UNREACHED;
    result.detail = "confirmation viewpoint: " + leg_result.detail;
    result.failed_station = "confirmation";
    RCLCPP_WARN(node_->get_logger(), "tray survey aborted: %s", result.detail.c_str());
    return;
  }

  const rclcpp::Time stopped = node_->now();
  if (!sleep_until(stopped + dwell_ + confirmation_timeout_ + arrival_grace_)) {
    finish_early_stop(result);
    return;
  }
  const std::vector<TrayObservation> confirm_frames = snapshot_window(
    confirmation_buffer_, logic_.confirmation_backend_name, stopped,
    dwell_ + confirmation_timeout_);
  if (confirm_frames.empty()) {
    result.outcome = SurveyTray::Result::OUTCOME_ACQUISITION_FAILED;
    result.detail =
      "the confirmation viewpoint was reached but no confirm-duty observation arrived after "
      "the arm stopped: the sensor or " + logic_.confirmation_backend_name +
      " could not answer, so the survey did not look and nothing authorizes a grasp";
    RCLCPP_WARN(node_->get_logger(), "%s", result.detail.c_str());
    return;
  }

  const ConfirmationEvaluation evaluation =
    evaluate_confirmation(*selected, confirm_frames, logic_);
  switch (evaluation.verdict) {
    case ConfirmationVerdict::kConfirmed:
      result.outcome = SurveyTray::Result::OUTCOME_CONFIRMED;
      result.detail = evaluation.detail;
      if (evaluation.associated_observation) {
        result.confirmed_observation = *evaluation.associated_observation;
      }
      RCLCPP_INFO(
        node_->get_logger(), "tray survey confirmed '%s': %s", selected->sku.c_str(),
        result.detail.c_str());
      return;
    case ConfirmationVerdict::kRefutedClassMismatch:
      result.outcome = SurveyTray::Result::OUTCOME_REFUTED;
      result.refutation = SurveyTray::Result::REFUTATION_CLASS_MISMATCH;
      break;
    case ConfirmationVerdict::kRefutedSkuMismatch:
      result.outcome = SurveyTray::Result::OUTCOME_REFUTED;
      result.refutation = SurveyTray::Result::REFUTATION_SKU_MISMATCH;
      break;
    case ConfirmationVerdict::kRefutedAbsent:
      result.outcome = SurveyTray::Result::OUTCOME_REFUTED;
      result.refutation = SurveyTray::Result::REFUTATION_ABSENT;
      break;
    case ConfirmationVerdict::kNoObservation:
      // evaluate_confirmation only reports this for an empty window, handled above.
      result.outcome = SurveyTray::Result::OUTCOME_ACQUISITION_FAILED;
      result.detail = evaluation.detail;
      return;
  }
  result.detail = evaluation.detail;
  if (evaluation.associated_observation) {
    result.refuting_observation = *evaluation.associated_observation;
  }
  RCLCPP_WARN(
    node_->get_logger(), "tray survey refuted candidate '%s' (refutation %u): %s",
    selected->sku.c_str(), result.refutation, result.detail.c_str());
}

TraySurveyNode::LegResult TraySurveyNode::run_leg(const SurveyViewpoint::Goal & goal)
{
  LegResult outcome;
  if (stop_requested()) {
    outcome.stopped_by_cancel = true;
    return outcome;
  }

  auto send = viewpoint_client_->async_send_goal(goal);
  while (send.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (stop_requested()) {
      if (shutdown_requested()) {
        // The node is going away: the executor has stopped or this node is being destroyed,
        // so the goal response will never arrive. Return now rather than hold the
        // destructor's join for the full grace below.
        outcome.stopped_by_cancel = true;
        return outcome;
      }
      // The goal may still be in flight; wait it out so we either learn it was refused or
      // hold the handle needed to cancel it. Sliced so a shutdown arriving mid-grace is not
      // held for the whole of it.
      outcome.stopped_by_cancel = true;
      const auto grace_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
      while (send.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
        if (shutdown_requested()) {
          return outcome;
        }
        if (std::chrono::steady_clock::now() >= grace_deadline) {
          outcome.stopped_by_cancel = false;
          outcome.detail =
            "the " + survey_action_name_ + " server did not accept the goal in time";
          return outcome;
        }
      }
      break;
    }
  }
  const ViewpointGoalHandle::SharedPtr child = send.get();
  if (!child) {
    outcome.stopped_by_cancel = false;
    outcome.detail = "the " + survey_action_name_ + " server rejected the leg goal";
    return outcome;
  }
  {
    std::scoped_lock lock(mutex_);
    child_ = child;
  }
  if (stop_requested()) {
    cancel_child();
  }

  auto result_future = viewpoint_client_->async_get_result(child);
  bool cancel_sent;
  {
    std::scoped_lock lock(mutex_);
    cancel_sent = child_cancel_requested_;
  }
  while (result_future.wait_for(std::chrono::milliseconds(50)) != std::future_status::ready) {
    if (stop_requested() && !cancel_sent) {
      cancel_child();
      cancel_sent = true;
    }
    if (shutdown_requested()) {
      // This node is being destroyed. A client cancel keeps waiting here — the contract says
      // the canceled result is not delivered until the child is terminal — but a shutdown
      // must not: the executor has stopped (launch teardown) or this node's callbacks are
      // going away (test teardown), so the future can never complete and the destructor's
      // join would hang the process until SIGKILL. The cancel has been requested; let go and
      // let the shutdown path deliver this goal's result.
      outcome.ran = true;
      outcome.stopped_by_cancel = true;
      {
        std::scoped_lock lock(mutex_);
        child_.reset();
        child_cancel_requested_ = false;
      }
      return outcome;
    }
  }
  const ViewpointGoalHandle::WrappedResult wrapped = result_future.get();
  {
    std::scoped_lock lock(mutex_);
    child_.reset();
    child_cancel_requested_ = false;
  }

  outcome.ran = true;
  outcome.stopped_by_cancel = stop_requested();
  if (wrapped.result) {
    outcome.outcome = wrapped.result->outcome;
    outcome.detail = wrapped.result->detail;
    // First-hand motion evidence from the leg's own result (Milestone 10 §6, Card 052): a
    // refused plan or a backend that never engaged proves no joint command was issued, an
    // arrival is the controllers reporting success (a verified stop for this attempt), and
    // every other outcome takes the backend's terminal-stop flag as-is. A result that never
    // arrived (the branches below) carries neither flag — the fail-closed state.
    outcome.evidence_not_started = survey_viewpoint_leg_not_started(outcome.outcome);
    outcome.evidence_terminal_stop = wrapped.result->execution_reached_terminal_stop ||
      outcome.outcome == SurveyViewpoint::Result::OUTCOME_ARRIVED;
  } else if (wrapped.code == rclcpp_action::ResultCode::CANCELED) {
    outcome.outcome = SurveyViewpoint::Result::OUTCOME_CANCELED;
    outcome.detail = "the leg was canceled";
  } else {
    outcome.outcome = SurveyViewpoint::Result::OUTCOME_UNAVAILABLE;
    outcome.detail = "the leg ended without a result (code " +
      std::to_string(static_cast<int>(wrapped.code)) + ")";
  }
  // A cancel that raced the leg's own completion still counts as a stop for this survey.
  if (outcome.stopped_by_cancel && outcome.outcome != SurveyViewpoint::Result::OUTCOME_CANCELED) {
    outcome.detail += " (this survey was canceled while the leg was finishing)";
  }
  return outcome;
}

void TraySurveyNode::note_leg_evidence(const LegResult & leg, SurveyTray::Result & result)
{
  if (!leg.ran || leg.evidence_not_started) {
    // A leg that never issued a command cannot move the arm and cannot revoke a stop an
    // earlier leg already established (Milestone 10 §6, Card 052).
    return;
  }
  result.motion_definitely_not_started = false;
  result.execution_reached_terminal_stop = leg.evidence_terminal_stop;
}

void TraySurveyNode::classify_absence_probes(
  const SurveyTray::Goal & goal, SurveyTray::Result & result)
{
  result.absence_probe_verdicts.clear();
  if (goal.absence_probe_positions.empty()) {
    return;
  }
  Eigen::Isometry3d world_from_shelf = Eigen::Isometry3d::Identity();
  try {
    world_from_shelf = tf2::transformToEigen(
      tf_buffer_->lookupTransform(
        planning_frame_, shelf_frame_, tf2::TimePointZero, tf2::durationFromSec(5.0)));
  } catch (const tf2::TransformException & error) {
    RCLCPP_WARN(
      node_->get_logger(),
      "absence probes not evaluated: could not resolve %s <- %s: %s", planning_frame_.c_str(),
      shelf_frame_.c_str(), error.what());
    return;
  }
  std::vector<AbsenceStationView> views;
  for (std::size_t index = 0; index < result.overview_stations_visited.size(); ++index) {
    const restocker_perception::SurveyStation * station = restocker_perception::find_survey_station(
      stations_, result.overview_stations_visited[index]);
    if (station == nullptr || index >= result.overview_station_reports.size()) {
      continue;
    }
    AbsenceStationView view;
    view.world_from_optical = world_from_shelf * station->shelf_from_optical;
    view.healthy = station_acquisition_healthy(result.overview_station_reports[index]);
    views.push_back(view);
  }
  const auto to_eigen = [](const geometry_msgs::msg::Point & point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
  };
  std::vector<Eigen::Vector3d> candidates;
  for (const TrayObservation & candidate : result.overview_candidates) {
    candidates.push_back(to_eigen(candidate.pose.pose.position));
  }
  std::vector<Eigen::Vector3d> probes;
  for (const geometry_msgs::msg::Point & probe : goal.absence_probe_positions) {
    probes.push_back(to_eigen(probe));
  }
  std::vector<Eigen::Vector3d> known_bodies = candidates;
  for (const geometry_msgs::msg::Point & body : goal.absence_occluder_positions) {
    known_bodies.push_back(to_eigen(body));
  }
  for (std::size_t index = 0; index < probes.size(); ++index) {
    std::vector<Eigen::Vector3d> occluders = known_bodies;
    for (std::size_t other = 0; other < probes.size(); ++other) {
      if (other != index) {
        occluders.push_back(probes[other]);
      }
    }
    const AbsenceProbeVerdict verdict =
      classify_absence_probe(probes[index], candidates, occluders, views, absence_);
    result.absence_probe_verdicts.push_back(static_cast<std::uint8_t>(verdict));
    double nearest = -1.0;
    for (const Eigen::Vector3d & candidate : candidates) {
      const double distance = (candidate - probes[index]).norm();
      if (nearest < 0.0 || distance < nearest) {
        nearest = distance;
      }
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "absence probe (%.3f, %.3f, %.3f): %s (nearest overview candidate %.3f m, -1 = none)",
      probes[index].x(), probes[index].y(), probes[index].z(),
      absence_probe_verdict_name(verdict), nearest);
  }
}

void TraySurveyNode::finish_early_stop(SurveyTray::Result & result)
{
  const bool canceled = [this]() {
    std::scoped_lock lock(mutex_);
    return cancel_requested_;
  }();
  if (canceled) {
    result.outcome = SurveyTray::Result::OUTCOME_CANCELED;
    if (result.detail.empty()) {
      result.detail =
        "canceled: unvisited stations were abandoned; the candidates and observations "
        "already published remain, and no motion goal is outstanding";
    }
  } else {
    result.outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
    result.detail = "the tray survey node shut down before this survey completed";
  }
}

bool TraySurveyNode::sleep_until(const rclcpp::Time & deadline)
{
  while (true) {
    if (stop_requested()) {
      return false;
    }
    const rclcpp::Time now = node_->now();
    if (now >= deadline) {
      return true;
    }
    auto remaining = std::chrono::nanoseconds((deadline - now).nanoseconds());
    const auto slice = std::min(remaining, std::chrono::nanoseconds(50'000'000));
    if (slice.count() <= 0) {
      return !stop_requested();
    }
    std::this_thread::sleep_for(slice);
  }
}

std::vector<TraySurveyNode::TrayObservation> TraySurveyNode::snapshot_window(
  const std::vector<TrayObservation> & buffer, const std::string & backend_name,
  const rclcpp::Time & stopped, const std::chrono::milliseconds & stamp_window)
{
  std::vector<TrayObservation> copy;
  {
    std::scoped_lock lock(mutex_);
    copy = buffer;
  }
  const std::int64_t start_ns = stopped.nanoseconds();
  const std::int64_t end_ns =
    start_ns + std::chrono::duration_cast<std::chrono::nanoseconds>(stamp_window).count();
  return observations_in_window(copy, backend_name, start_ns, end_ns);
}

TraySurveyNode::StationAcquisition TraySurveyNode::acquire_station(
  const std::string & station, const rclcpp::Time & stopped,
  const std::chrono::milliseconds & stamp_window)
{
  StationAcquisition acquisition;
  auto & report = acquisition.report;
  report.station = station;
  report.image_tap_configured = !overview_image_topic_.empty();
  report.detection_tap_configured = !overview_detection_topic_.empty();
  const std::int64_t start_ns = stopped.nanoseconds();
  const std::int64_t end_ns =
    start_ns + std::chrono::duration_cast<std::chrono::nanoseconds>(stamp_window).count();
  // The same half-open window (start, end] the admission filter uses, so every count below is
  // about exactly the frames admission judged.
  const auto in_window = [start_ns, end_ns](const std::int64_t stamp_ns) {
    return stamp_ns > start_ns && stamp_ns <= end_ns;
  };
  std::vector<TrayObservation> copy;
  {
    std::scoped_lock lock(mutex_);
    for (const StageFrameRecord & frame : overview_image_frames_) {
      if (in_window(frame.stamp.nanoseconds())) {
        ++report.images;
      }
    }
    for (const StageFrameRecord & frame : overview_detection_frames_) {
      if (!in_window(frame.stamp.nanoseconds())) {
        continue;
      }
      ++report.detection_frames;
      if (frame.proposals > 0U) {
        ++report.frames_with_detections;
      }
    }
    for (const TrayObservation & observation : overview_buffer_) {
      const auto stamp = observation_stamp_ns(observation);
      if (stamp && in_window(*stamp)) {
        ++report.published;
      }
    }
    copy = overview_buffer_;
  }
  acquisition.admitted = observations_in_window(
    copy, logic_.overview_backend_name, start_ns,
    end_ns);
  report.admitted = static_cast<std::uint32_t>(acquisition.admitted.size());
  return acquisition;
}

void TraySurveyNode::terminate_outstanding_goal() noexcept
{
  std::shared_ptr<TrayGoalHandle> handle;
  {
    std::scoped_lock lock(mutex_);
    handle = outstanding_.lock();
    outstanding_.reset();
    busy_ = false;
    collecting_ = false;
  }
  if (!handle) {
    return;
  }
  try {
    auto result = std::make_shared<SurveyTray::Result>();
    result->outcome = SurveyTray::Result::OUTCOME_UNAVAILABLE;
    result->detail = "the tray survey node shut down before this survey completed";
    handle->abort(result);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      node_->get_logger(), "outstanding tray survey could not be aborted at shutdown: %s",
      error.what());
  } catch (...) {
    RCLCPP_WARN(node_->get_logger(), "outstanding tray survey could not be aborted at shutdown");
  }
}

void TraySurveyNode::finish(
  const std::shared_ptr<TrayGoalHandle> & handle,
  const std::shared_ptr<SurveyTray::Result> & result, bool canceled)
{
  {
    std::scoped_lock lock(mutex_);
    busy_ = false;
    outstanding_.reset();
    child_.reset();
  }
  // Every outcome is a completed survey: the verdict is the outcome field, not an aborted
  // goal the caller must guess at. A terminal transition after the context is down throws
  // from inside rclcpp; that is a shutdown race, not a reason to terminate the process.
  try {
    if (canceled && handle->is_canceling()) {
      handle->canceled(result);
      return;
    }
    handle->succeed(result);
  } catch (const std::exception & error) {
    RCLCPP_WARN(
      node_->get_logger(), "tray survey result could not be delivered: %s", error.what());
  } catch (...) {
    RCLCPP_WARN(node_->get_logger(), "tray survey result could not be delivered");
  }
}

}  // namespace restocker_task_executor
