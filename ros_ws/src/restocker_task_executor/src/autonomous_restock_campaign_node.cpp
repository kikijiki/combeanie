// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// The sensor-driven mode loop above the single-transfer RestockProduct action (Milestone 10
// section 1).
//
// Owns no world-state authority and no robot port. One work cycle stands in one position of the
// five-mode graph:
//
//   SURVEY_SHELF  survey every lane whose evidence is invalidated, expired, or never taken
//                 (all of them at start of run), then measure section 4 deficits;
//   IDLE          when no lane needs a survey and none is running: wait for an evidence event;
//                 when the deficit is zero or the tray reports no candidate: park there;
//   SURVEY_TRAY   only when a deficit exists and no valid admitted tray evidence names a
//                 candidate that could fill it, through the coordinated SurveyTray action;
//   CONFIRM       reported while that action selects and close-confirms one candidate; a typed
//                 refutation returns to SURVEY_TRAY with the candidate marked for this cycle;
//   TRANSFER      one RestockProduct goal per cycle, entered only after a
//                 confirmation on the survey path or against a valid admitted candidate —
//                 never from overview evidence and never against stale lane evidence. A
//                 confirming cycle names the confirmed product in the goal (Milestone 10 §1
//                 confirmed-identity contract, Card 037) so the coordinator cannot transfer a
//                 different product; a skip-path cycle keeps the empty selector.
//
// The transfer itself ends at the destination lane's survey station (the coordinator's retreat
// survey), so the loop simply re-reads evidence afterwards: the lane just filled is fresh and the
// rest stand on their validity horizons. Survey and transfer arm time are accumulated and
// reported separately.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cstdint>
#include <cinttypes>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>
#include <restocker_interfaces/action/survey_lane.hpp>
#include <restocker_interfaces/action/survey_tray.hpp>
#include <restocker_interfaces/action/survey_viewpoint.hpp>
#include <restocker_interfaces/msg/autonomous_restock_campaign_status.hpp>
#include <restocker_interfaces/msg/restock_coordinator_status.hpp>
#include <restocker_interfaces/msg/tray_station_acquisition.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_perception/survey_stations.hpp>
#include <restocker_world_state/ros_conversions.hpp>
#include <restocker_world_state/world_state.hpp>

#include "restocker_task_executor/action_client_finality.hpp"
#include "restocker_task_executor/campaign_confirm_report.hpp"
#include "restocker_task_executor/campaign_motion_evidence.hpp"
#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/recovery_classification.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_task_executor/survey_recovery_context.hpp"
#include "restocker_task_executor/task_selection.hpp"
#include "restocker_task_executor/tray_survey_logic.hpp"
#include "restocker_task_executor/unresolved_motion.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;
using RestockProduct = restocker_interfaces::action::RestockProduct;
using SurveyLane = restocker_interfaces::action::SurveyLane;
using SurveyTray = restocker_interfaces::action::SurveyTray;
using SurveyViewpoint = restocker_interfaces::action::SurveyViewpoint;
using CampaignStatus = restocker_interfaces::msg::AutonomousRestockCampaignStatus;
using GetWorldState = restocker_interfaces::srv::GetWorldState;
using restocker_world_state::ObjectOrientation;
using restocker_world_state::ProductClass;
using restocker_world_state::TrackingState;
using restocker_world_state::GraspState;

constexpr std::array<std::uint8_t, 3> kKnownProductClasses{
  CampaignStatus::PRODUCT_CLASS_CAN,
  CampaignStatus::PRODUCT_CLASS_SMALL_BOTTLE,
  CampaignStatus::PRODUCT_CLASS_LARGE_BOTTLE};
constexpr std::size_t kProductClassSlots = 4U;
constexpr char kPlanningFrame[] = "world";
// Milestone 10 §6 rung 5 (Card 051): a skip mark clears when the product is re-observed
// farther than this from where it was when skipped — the evidence changed, so the loop
// revisits instead of excluding forever. Matches the tray candidate merge radius.
constexpr double kSkipMarkClearRadiusM = 0.04;

[[nodiscard]] std::optional<std::size_t> class_slot(std::uint8_t product_class)
{
  if (product_class >= kProductClassSlots) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(product_class);
}

// Milestone 10 §6, Card 052: does a /survey_viewpoint result establish the arm's state for
// the retreat rung? Never-commanded outcomes and an arrival are established (an arrival is
// the controllers reporting success); anything else takes the backend's terminal-stop flag.
// OUTCOME_SHUTDOWN (review F2) carries no stop flag, so it — and every unknown constant —
// falls through the default to "not established": fail closed.
[[nodiscard]] bool viewpoint_leg_established(std::uint8_t outcome, bool terminal_stop)
{
  switch (outcome) {
    case SurveyViewpoint::Result::OUTCOME_VIEWPOINT_UNRESOLVED:
    case SurveyViewpoint::Result::OUTCOME_PLANNING_FAILED:
    case SurveyViewpoint::Result::OUTCOME_UNAVAILABLE:
    case SurveyViewpoint::Result::OUTCOME_INVALID_REQUEST:
    case SurveyViewpoint::Result::OUTCOME_ARRIVED:
      return true;
    default:
      return terminal_stop;
  }
}

[[nodiscard]] const char * product_class_name(std::uint8_t product_class)
{
  switch (product_class) {
    case CampaignStatus::PRODUCT_CLASS_CAN: return "can";
    case CampaignStatus::PRODUCT_CLASS_SMALL_BOTTLE: return "small_bottle";
    case CampaignStatus::PRODUCT_CLASS_LARGE_BOTTLE: return "large_bottle";
    default: return "unknown";
  }
}

class AutonomousRestockCampaign
{
public:
  explicit AutonomousRestockCampaign(const rclcpp::NodeOptions & options)
  : node_(std::make_shared<rclcpp::Node>("autonomous_restock_campaign", options))
  {
    const std::string geometry_path = parameter("workcell_geometry_path", std::string());
    const std::string product_catalog_path = parameter("product_catalog_path", std::string());
    restock_action_name_ = parameter("restock_action_name", std::string("/restock_product"));
    shelf_survey_action_name_ = parameter("shelf_survey_action_name", std::string("/survey_lane"));
    tray_survey_action_name_ = parameter("tray_survey_action_name", std::string("/survey_tray"));
    coordinator_status_topic_ =
      parameter("coordinator_status_topic", std::string("/restock_action_coordinator/status"));
    world_state_service_name_ =
      parameter("world_state_service_name", std::string("/world_state/get_snapshot"));
    campaign_status_topic_ =
      parameter("campaign_status_topic", std::string("/autonomous_restock_campaign/status"));
    max_cycles_ = parameter<std::int64_t>("max_cycles", 0);
    // Card 086 stage 1 (CMB-SPEC-13): unresolved-motion state is in memory only, so a restarted
    // campaign cannot know whether its previous incarnation left a goal in flight. It reports
    // PHASE_RECOVERING and sends nothing until an operator sets this parameter to true (launch
    // argument or `ros2 param set`). A fresh simulator world is the only case where launching
    // with it already true is honest.
    restart_acknowledged_parameter_ = "restart_acknowledged";
    restart_recovery_pending_ = !parameter<bool>(restart_acknowledged_parameter_, false);
    cycle_period_ = seconds_parameter("cycle_period_sec", 30.0, true);
    // A work cycle that made no progress (failed survey, refused or timed-out transfer) waits
    // at least this long before retrying, so a permanently failing boundary cannot spin.
    failed_cycle_backoff_ = seconds_parameter("failed_cycle_backoff_sec", 1.0, true);
    server_wait_timeout_ = seconds_parameter("server_wait_timeout_sec", 300.0, false);
    action_timeout_ = seconds_parameter("action_timeout_sec", 600.0, false);
    lane_evidence_validity_ =
      std::chrono::milliseconds(parameter<std::int64_t>("lane_evidence_validity_ms", 60000));
    tray_evidence_validity_ =
      std::chrono::milliseconds(parameter<std::int64_t>("tray_evidence_validity_ms", 120000));
    max_tray_attempts_ = parameter<std::int64_t>("max_tray_attempts_per_cycle", 8);
    // Milestone 10 §6 rung 3 (Card 052): how many attempts one lane gets per cycle before
    // rung 5 skips it for this cycle. The same value bounds one tray station's recoverable
    // failures before its rung 5 skip (the tray loop's own retry budget stays
    // max_tray_attempts_per_cycle).
    max_lane_survey_attempts_ =
      parameter<std::int64_t>("max_lane_survey_attempts_per_cycle", 2);
    // Milestone 10 §6 rung 5 (Card 052): the campaign-level survey-skip budget — how many
    // lane/station skips this run may charge before the skipped resource is reported
    // unavailable and the loop latches PHASE_BLOCKED naming the rung.
    max_survey_skips_ = parameter<std::int64_t>("max_survey_skips", 2);
    // Rung 1 (Card 052): where a failed survey attempt that may have moved the arm
    // retreats to. The server is the ordinary /survey_viewpoint action (launched whenever
    // the autonomous campaign is); the station defaults to the first tray overview station
    // derived from the workcell geometry when not configured.
    recovery_viewpoint_action_name_ =
      parameter("recovery_viewpoint_action_name", std::string("/survey_viewpoint"));
    recovery_safe_station_ = parameter("recovery_safe_station", std::string());
    // Milestone 10 §6 rung 5 (Card 051): how often one product may be skipped before the
    // campaign treats it as gone for the run and reports it. The budget that makes the
    // ladder terminate at the campaign level.
    max_product_skips_ = parameter<std::int64_t>("max_product_skips", 2);
    if (max_product_skips_ < 1) {
      throw std::invalid_argument("max_product_skips must be at least 1");
    }
    // Milestone 10 §6, Card 055: the premature-NO_COMPATIBLE_PAIR recovery budget — how many
    // forced tray re-surveys this run may charge before the recoverable boundary latches
    // PHASE_BLOCKED naming the rung. Monotone per run, like max_survey_skips.
    max_ncp_resurveys_ = parameter<std::int64_t>("max_ncp_resurveys", 2);
    if (max_ncp_resurveys_ < 1) {
      throw std::invalid_argument("max_ncp_resurveys must be at least 1");
    }
    // Milestone 10 §6, Card 066: how many consecutive feed-order rungs (a tray NO_CANDIDATE
    // whose matching stock stands behind a front the survey could not nominate) may force a
    // re-survey before the rung ends at a PHASE_BLOCKED naming feed order. A successful transfer
    // resets the count, so it bounds consecutive no-progress rungs; the loop itself stays
    // runnable after the latch (like Card 055's exhausted rung), and a transfer resets it.
    max_feed_order_resurveys_ = parameter<std::int64_t>("max_feed_order_resurveys", 2);
    if (max_feed_order_resurveys_ < 1) {
      throw std::invalid_argument("max_feed_order_resurveys must be at least 1");
    }
    // Milestone 10 §6, Card 058: the skip-mark lift recovery budget — how many forced
    // skip-lifted tray re-surveys this run may charge when OUTCOME_NO_CANDIDATE excluded
    // every class-matching overview candidate only by active skip marks. Monotone per run,
    // like max_ncp_resurveys.
    max_skip_resurveys_ = parameter<std::int64_t>("max_skip_resurveys", 2);
    if (max_skip_resurveys_ < 1) {
      throw std::invalid_argument("max_skip_resurveys must be at least 1");
    }
    // Milestone 10 §6, Card 069: how many consecutive completed tray surveys must view a
    // counted product's place empty before it is retired from back stock. At least two, so a
    // single missed detection can never retire a present product.
    absence_confirmations_ = parameter<std::int64_t>("absence_confirmations", 2);
    if (absence_confirmations_ < 2) {
      throw std::invalid_argument("absence_confirmations must be at least 2");
    }
    // Independence of the counted views (review N3): each counted empty view must come from a
    // different work cycle AND at least this long after the previous counted one, so one miss
    // repeated by back-to-back surveys of the same scene never counts twice.
    absence_min_separation_ = std::chrono::duration<double>(
      parameter<double>("absence_min_separation_sec", 30.0));
    if (!(absence_min_separation_.count() > 0.0)) {
      throw std::invalid_argument("absence_min_separation_sec must be positive");
    }
    // The coordinator's selection horizon (selection.maximum_object_age_ms), wired from the
    // same launch argument: an unpinned transfer may pick any tracked object this young, so the
    // ghost gate below holds for max(tray_evidence_validity, this) (review cmbrev069b B1).
    coordinator_object_max_age_ = std::chrono::milliseconds(
      parameter<std::int64_t>("coordinator_object_max_age_ms", 180000));
    if (coordinator_object_max_age_.count() < 0) {
      throw std::invalid_argument("coordinator_object_max_age_ms must not be negative");
    }

    if (geometry_path.empty() || product_catalog_path.empty()) {
      throw std::invalid_argument(
              "workcell_geometry_path and product_catalog_path must not be empty");
    }
    if (restock_action_name_.empty() || shelf_survey_action_name_.empty() ||
      tray_survey_action_name_.empty() || coordinator_status_topic_.empty() ||
      world_state_service_name_.empty() || campaign_status_topic_.empty() ||
      recovery_viewpoint_action_name_.empty())
    {
      throw std::invalid_argument("campaign action names must not be empty");
    }
    if (max_cycles_ < 0) {
      throw std::invalid_argument("max_cycles must be zero (unbounded) or positive");
    }
    if (lane_evidence_validity_.count() <= 0 || tray_evidence_validity_.count() <= 0) {
      throw std::invalid_argument("evidence validity horizons must be positive");
    }
    if (max_tray_attempts_ < 1) {
      throw std::invalid_argument("max_tray_attempts_per_cycle must be at least one");
    }
    if (max_lane_survey_attempts_ < 1) {
      throw std::invalid_argument("max_lane_survey_attempts_per_cycle must be at least one");
    }
    if (max_survey_skips_ < 1) {
      throw std::invalid_argument("max_survey_skips must be at least one");
    }
    if (max_ncp_resurveys_ < 1) {
      throw std::invalid_argument("max_ncp_resurveys must be at least one");
    }
    if (max_skip_resurveys_ < 1) {
      throw std::invalid_argument("max_skip_resurveys must be at least one");
    }

    const auto geometry = restocker_perception::load_workcell_survey_geometry(geometry_path);
    for (const auto & lane : geometry.lanes) {
      lane_ids_.push_back(lane.lane_id);
    }
    for (const auto & station : restocker_perception::nominal_survey_stations(geometry)) {
      if (station.name.starts_with("tray_")) {
        tray_station_names_.push_back(station.name);
      }
    }
    if (lane_ids_.empty() || tray_station_names_.empty()) {
      throw std::invalid_argument("campaign requires at least one lane and one tray station");
    }
    if (recovery_safe_station_.empty()) {
      recovery_safe_station_ = tray_station_names_.front();
    }

    auto catalog = ProductCollisionCatalog::load(product_catalog_path);
    if (!catalog) {
      throw std::invalid_argument("invalid product catalog: " + catalog.error().detail);
    }
    product_catalog_ = std::move(catalog.value());
    auto manipulation_geometry = load_manipulation_geometry(geometry_path);
    if (!manipulation_geometry) {
      throw std::invalid_argument(
              "invalid manipulation geometry: " + manipulation_geometry.error().detail);
    }
    manipulation_geometry_ = std::move(manipulation_geometry.value());

    shelf_survey_client_ =
      rclcpp_action::create_client<SurveyLane>(node_, shelf_survey_action_name_);
    tray_survey_client_ = rclcpp_action::create_client<SurveyTray>(node_, tray_survey_action_name_);
    viewpoint_client_ =
      rclcpp_action::create_client<SurveyViewpoint>(node_, recovery_viewpoint_action_name_);
    restock_client_ = rclcpp_action::create_client<RestockProduct>(node_, restock_action_name_);
    // Cancel by goal ID for a goal whose result was not a delivered terminal (Card 070): the
    // action clients forget such a goal, so they cannot cancel it through its handle.
    shelf_survey_cancel_client_ = create_goal_cancel_client(*node_, shelf_survey_action_name_);
    tray_survey_cancel_client_ = create_goal_cancel_client(*node_, tray_survey_action_name_);
    viewpoint_cancel_client_ =
      create_goal_cancel_client(*node_, recovery_viewpoint_action_name_);
    restock_cancel_client_ = create_goal_cancel_client(*node_, restock_action_name_);
    world_state_client_ = node_->create_client<GetWorldState>(world_state_service_name_);
    campaign_status_publisher_ = node_->create_publisher<CampaignStatus>(
      campaign_status_topic_, rclcpp::QoS(rclcpp::KeepLast(64)).reliable().transient_local());
    coordinator_status_subscription_ =
      node_->create_subscription<restocker_interfaces::msg::RestockCoordinatorStatus>(
      coordinator_status_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const restocker_interfaces::msg::RestockCoordinatorStatus & message) {
        coordinator_ready_.store(message.admission_ready, std::memory_order_release);
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "autonomous mode loop configured: %zu lanes surveyed on evidence events, coordinated tray "
      "action %s, lane validity %" PRId64 " ms, tray validity %" PRId64 " ms, max_cycles=%" PRId64,
      lane_ids_.size(), tray_survey_action_name_.c_str(),
      lane_evidence_validity_.count(), tray_evidence_validity_.count(), max_cycles_);
  }

  ~AutonomousRestockCampaign() {stop();}

  AutonomousRestockCampaign(const AutonomousRestockCampaign &) = delete;
  AutonomousRestockCampaign & operator=(const AutonomousRestockCampaign &) = delete;

  [[nodiscard]] rclcpp::Node::SharedPtr node() const {return node_;}

  void start()
  {
    worker_ = std::thread([this]() {run();});
  }

  void stop() noexcept
  {
    stopping_.store(true, std::memory_order_release);
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  struct Measurement
  {
    std::uint64_t world_revision{0U};
    std::array<std::uint32_t, kProductClassSlots> front_stock{};
    std::array<std::uint32_t, kProductClassSlots> front_deficits{};
    std::array<std::uint32_t, kProductClassSlots> back_stock{};
    std::array<bool, kProductClassSlots> has_valid_candidate{};
    std::uint32_t total_front_deficit{0U};
    std::uint32_t total_front_stock{0U};
    std::uint32_t total_back_stock{0U};
    // Lanes whose evidence is invalidated, expired, or never taken, in survey order.
    std::vector<std::string> lanes_needing_survey;
    bool any_lane_needs_survey{false};
    // The largest outstanding section 4 deficit's product class (CampaignStatus code), or zero.
    std::uint8_t largest_deficit_class{0U};
    // Whether some outstanding deficit has a valid admitted tray candidate that could fill it.
    bool has_valid_candidate_for_deficit{false};
    // Free tray (back-stock) objects by perception source id, so a close-confirmed
    // observation's source identity resolves to the world-state object id at the transfer
    // boundary (Milestone 10 §1, Card 037).
    std::map<std::string, std::uint64_t> tray_candidate_object_ids;
    // The same candidates' world positions by source id: a recoverable skip marks the product
    // for tray exclusion (Milestone 10 §6 rung 5, Card 051).
    std::map<std::string, geometry_msgs::msg::Point> tray_candidate_positions;
    // Milestone 10 §6, Card 066 (skip path): the world-state tray products the coordinator
    // could take on the skip path, with their class slot: counted, upright stock observed within
    // max(tray_evidence_validity, coordinator_object_max_age_ms). Ordered by source id for a
    // deterministic receipt.
    std::map<std::string, std::pair<geometry_msgs::msg::Point, std::size_t>> skip_path_candidates;
    // The products counted as back stock, by source id: their positions travel as the tray
    // survey's absence probes, and their last observation instant anchors the absence streak
    // (Milestone 10 §6, Card 069).
    std::map<std::string, geometry_msgs::msg::Point> tray_stock_positions;
    std::map<std::string, std::int64_t> tray_stock_observation_ns;
    std::map<std::string, std::size_t> tray_stock_slots;
    // A retired product whose last observation is still inside tray_evidence_validity: the skip
    // path is refused while one exists, so an unpinned transfer can never pick it (review N5d).
    bool retired_product_looks_fresh{false};
  };

  struct TraySurveyOutcome
  {
    bool completed{false};
    std::uint8_t outcome{SurveyTray::Result::OUTCOME_UNSET};
    std::string detail;
    std::optional<geometry_msgs::msg::Point> refuted_position;
    // Set on OUTCOME_CONFIRMED: the source identity of the confirming observation, which the
    // transfer goal must name. Empty when the confirmation carried no usable identity.
    std::string confirmed_source_object_id;
    // First-hand motion evidence propagated from the action result (Milestone 10 §6,
    // Card 052); meaningful when completed is true.
    bool motion_definitely_not_started{false};
    bool execution_reached_terminal_stop{false};
    // The station or "confirmation" whose leg failed, from the result; empty when no leg did.
    std::string failed_station;
    // Matching candidates passed over because a product stands ahead of them in their feed
    // column (Milestone 10 §6, Card 066).
    std::uint32_t feed_blocked_candidates{0U};
    // Parallel to overview_candidates: whether each is a feed-column front (Card 066 × 058).
    std::vector<bool> overview_candidate_feed_front;
    // Marked matching stock standing behind a front, and the server's example naming the first
    // blocked candidate and its blocker (Card 066, review cmbrev066b N1).
    std::uint32_t marked_feed_blocked_candidates{0U};
    std::string feed_block_example;
    // The distinct overview candidates the action collected (Milestone 10 §6, Card 058): the
    // NO_CANDIDATE classification reads them to decide whether a lift re-survey could select
    // one.
    std::vector<restocker_interfaces::msg::ObjectObservation> overview_candidates;
    // Per-probe absence verdicts, parallel to the goal's probes (Milestone 10 §6, Card 069);
    // empty when the overview did not complete.
    std::vector<std::uint8_t> absence_probe_verdicts;
    // The survey's per-station acquisition accounting (Milestone 10 §6, Card 050 reporting;
    // carried to the park's detail by Card 080's attribution contract); empty when the result
    // carried none.
    std::vector<restocker_interfaces::msg::TrayStationAcquisition> overview_station_reports;
  };

  // What one SurveyLane action attempt produced, including the first-hand motion evidence
  // the recovery classification reads (Milestone 10 §6, Card 052).
  struct LaneSurveyOutcome
  {
    bool observed{false};
    std::uint8_t outcome{SurveyLane::Result::OUTCOME_UNSET};
    std::string detail;
    // True only when a terminal result was delivered for this attempt and this call waited
    // for it. False for admission timeouts, rejections, a missing server, and a cancel that
    // outlived its own bounded wait — in each of those the arm's state cannot be read from
    // this action, and the classification must fail closed on it.
    bool terminal_result_received{false};
    bool motion_definitely_not_started{false};
    bool execution_reached_terminal_stop{false};
  };

  // What one work cycle did, which decides how the outer loop sleeps and whether IDLE is a
  // parked wait-for-evidence state or just the mode reported between work cycles.
  enum class CycleOutcome : std::uint8_t
  {
    // Progress: a transfer landed; re-evaluate immediately at the configured cadence.
    kContinue,
    // Parked in IDLE (front full, tray with no candidate, or exhausted stock): sleep, then
    // only an evidence event re-enters SURVEY_SHELF.
    kIdleWait,
    // A failed boundary; the backoff floor separates the retry and the loop stays runnable.
    kBlocked,
    // The campaign is shutting down.
    kStopped,
  };

  // What one shelf-survey pass did: the caller turns this into a CycleOutcome.
  enum class ShelfOutcome : std::uint8_t
  {
    kOk,
    kBlocked,
    kStopped,
  };

  template<typename Value>
  [[nodiscard]] Value parameter(const std::string & name, const Value & fallback)
  {
    return node_->has_parameter(name) ? node_->get_parameter(name).get_value<Value>() :
           node_->declare_parameter<Value>(name, fallback);
  }

  [[nodiscard]] std::chrono::milliseconds seconds_parameter(
    const std::string & name,
    double default_seconds, bool allow_zero)
  {
    const double value = parameter(name, default_seconds);
    if (value < 0.0 || (!allow_zero && value == 0.0)) {
      throw std::invalid_argument(
              name +
              (allow_zero ? " must be non-negative" : " must be positive"));
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(value));
  }

  [[nodiscard]] bool keep_running() const noexcept
  {
    return rclcpp::ok() && !stopping_.load(std::memory_order_acquire);
  }

  template<typename FutureT, typename PollT>
  [[nodiscard]] bool wait_for_future(
    FutureT & future, std::chrono::milliseconds timeout, PollT poll) const
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (keep_running() && std::chrono::steady_clock::now() < deadline) {
      const bool ready = future.wait_for(100ms) == std::future_status::ready;
      poll();
      if (ready) {
        return true;
      }
    }
    return false;
  }

  template<typename FutureT>
  [[nodiscard]] bool wait_for_future(FutureT & future, std::chrono::milliseconds timeout) const
  {
    return wait_for_future(future, timeout, []() {});
  }

  template<typename ClientT>
  [[nodiscard]] bool wait_for_server(const ClientT & client, const std::string & action_name) const
  {
    const auto deadline = std::chrono::steady_clock::now() + server_wait_timeout_;
    while (keep_running() && std::chrono::steady_clock::now() < deadline) {
      if (client->wait_for_action_server(1s)) {
        return true;
      }
    }
    if (keep_running()) {
      RCLCPP_ERROR(
        node_->get_logger(), "campaign action server %s was unavailable",
        action_name.c_str());
    }
    return false;
  }

  [[nodiscard]] bool wait_for_world_state_service() const
  {
    const auto deadline = std::chrono::steady_clock::now() + server_wait_timeout_;
    while (keep_running() && std::chrono::steady_clock::now() < deadline) {
      if (world_state_client_->wait_for_service(1s)) {
        return true;
      }
    }
    if (keep_running()) {
      RCLCPP_ERROR(
        node_->get_logger(), "world-state service %s was unavailable",
        world_state_service_name_.c_str());
    }
    return false;
  }

  // One snapshot, decoded through the world state's own converter so the campaign measures
  // against the same invariants the coordinator's selections do, plus the mode-loop facts the
  // graph needs: which lanes must be surveyed, section 4 deficits per class, and whether any
  // outstanding deficit already has a valid admitted tray candidate.
  [[nodiscard]] std::optional<Measurement> measure_world(std::string & error)
  {
    if (!wait_for_world_state_service()) {
      error = "world-state snapshot service unavailable";
      return std::nullopt;
    }
    auto future =
      world_state_client_->async_send_request(std::make_shared<GetWorldState::Request>());
    if (!wait_for_future(future, action_timeout_)) {
      error = "world-state snapshot request timed out";
      return std::nullopt;
    }
    const auto response = future.get();
    if (!response) {
      error = "world-state snapshot response was empty";
      return std::nullopt;
    }
    auto decoded = restocker_world_state::snapshot_from_message(response->snapshot, kPlanningFrame);
    if (!decoded) {
      error = "world-state snapshot rejected: " + decoded.error().detail;
      return std::nullopt;
    }
    const auto & snapshot = decoded.value();
    if (snapshot.lanes.size() != lane_ids_.size()) {
      error = "world-state snapshot lane set disagrees with the survey station set";
      return std::nullopt;
    }

    Measurement measured;
    measured.world_revision = snapshot.revision;
    const rclcpp::Time now = node_->now();
    std::uint32_t largest_deficit = 0U;
    std::set<std::uint64_t> front_object_ids;
    std::set<std::string> front_source_object_ids;

    for (const auto & lane_id : lane_ids_) {
      const auto lane = snapshot.lanes.find(restocker_world_state::LaneId{lane_id});
      if (lane == snapshot.lanes.end()) {
        error = "world-state snapshot is missing lane " + lane_id;
        return std::nullopt;
      }
      const auto geometry = manipulation_geometry_.lanes.find(lane_id);
      if (geometry == manipulation_geometry_.lanes.end()) {
        error = "lane " + lane_id + " has no manipulation geometry";
        return std::nullopt;
      }
      const auto deficit = lane_deficit(lane->second, geometry->second, product_catalog_);
      if (!deficit) {
        error = "lane " + lane_id + " deficit failed: " + deficit.error().detail;
        return std::nullopt;
      }
      const auto slot = class_slot(static_cast<std::uint8_t>(lane->second.expected_product_class));
      if (!slot || *slot == 0U) {
        error = "lane " + lane_id + " has no known expected product class";
        return std::nullopt;
      }
      measured.front_stock[*slot] += deficit.value().held_count;
      measured.front_deficits[*slot] += deficit.value().deficit;
      measured.total_front_stock += deficit.value().held_count;
      measured.total_front_deficit += deficit.value().deficit;
      if (deficit.value().deficit > largest_deficit) {
        largest_deficit = deficit.value().deficit;
        measured.largest_deficit_class =
          static_cast<std::uint8_t>(lane->second.expected_product_class);
      }
      // Section 5 validity: the horizon plus the event-driven invalidation flag. A lane that
      // was never surveyed (evidence_revision 0) or has no verification instant needs the eye.
      const bool expired =
        now.nanoseconds() - lane->second.last_verified.nanoseconds() >
        lane_evidence_validity_.count();
      if (lane->second.evidence_revision == 0 || lane->second.evidence_invalidated ||
        lane->second.last_verified.nanoseconds() <= 0 || expired)
      {
        measured.any_lane_needs_survey = true;
        measured.lanes_needing_survey.push_back(lane_id);
      }
      for (const auto content_id : lane->second.contents) {
        front_object_ids.insert(content_id.value);
      }
      front_source_object_ids.insert(
        lane->second.observed_source_object_ids.begin(),
        lane->second.observed_source_object_ids.end());
    }

    for (const auto & [object_id, object] : snapshot.objects) {
      static_cast<void>(object_id);
      if (object.tracking_state != TrackingState::Tracked ||
        object.grasp_state != GraspState::Free ||
        front_object_ids.contains(object.id.value) ||
        front_source_object_ids.contains(object.source_object_id))
      {
        continue;
      }
      measured.tray_candidate_object_ids.emplace(object.source_object_id, object.id.value);
      geometry_msgs::msg::Point point;
      point.x = object.pose_in_world.translation().x();
      point.y = object.pose_in_world.translation().y();
      point.z = object.pose_in_world.translation().z();
      measured.tray_candidate_positions.emplace(object.source_object_id, point);
      const auto slot = class_slot(static_cast<std::uint8_t>(object.product_class));
      if (!slot) {
        continue;
      }
      // Milestone 10 §6, Card 058 increment 2 (amended by review note 3): only positive
      // evidence of a fall — an observed horizontal or tilted orientation — removes a product
      // from back_stock and stock_remains. A merely unobserved product (stale, or orientation
      // unknown) still counts: it is stale, not gone, so a refusal against it takes Card 055's
      // forced re-survey, which refreshes it. The raw Tracked+Free set above still feeds the
      // skip-mark and identity maps.
      if (object.orientation == ObjectOrientation::Horizontal ||
        object.orientation == ObjectOrientation::Tilted)
      {
        continue;
      }
      // Milestone 10 §6, Card 069: a product retired on observed absence stops counting until
      // an observation newer than the retirement restores it (campaign-local, reversible).
      const std::int64_t observed_ns = object.observation_time.nanoseconds();
      const auto retired = retired_sources_.find(object.source_object_id);
      if (retired != retired_sources_.end() && observed_ns <= retired->second) {
        const auto ghost_window = std::max(
          std::chrono::nanoseconds(tray_evidence_validity_),
          std::chrono::nanoseconds(coordinator_object_max_age_));
        if (observed_ns > 0 && now.nanoseconds() - observed_ns <= ghost_window.count()) {
          measured.retired_product_looks_fresh = true;
        }
        continue;
      }
      measured.tray_stock_positions.emplace(object.source_object_id, point);
      measured.tray_stock_observation_ns.emplace(object.source_object_id, observed_ns);
      measured.tray_stock_slots.emplace(object.source_object_id, *slot);
      ++measured.back_stock[*slot];
      ++measured.total_back_stock;
      const bool fresh =
        object.observation_time.nanoseconds() > 0 &&
        now.nanoseconds() - object.observation_time.nanoseconds() <=
        tray_evidence_validity_.count();
      if (fresh && object.orientation == ObjectOrientation::Upright) {
        measured.has_valid_candidate[*slot] = true;
      }
      // Card 066 (skip path): what the coordinator could take is judged on its own selection
      // horizon, the window Card 069's ghost gate uses, not only on the tray window.
      const auto selection_horizon = std::max(
        std::chrono::nanoseconds(tray_evidence_validity_),
        std::chrono::nanoseconds(coordinator_object_max_age_));
      if (object.orientation == ObjectOrientation::Upright && observed_ns > 0 &&
        now.nanoseconds() - observed_ns <= selection_horizon.count())
      {
        measured.skip_path_candidates.emplace(object.source_object_id, std::pair{point, *slot});
      }
    }
    for (const std::uint8_t product_class : kKnownProductClasses) {
      const auto slot = class_slot(product_class);
      if (measured.front_deficits[*slot] > 0U && measured.has_valid_candidate[*slot]) {
        measured.has_valid_candidate_for_deficit = true;
      }
    }
    return measured;
  }

  void publish_status(
    std::uint8_t phase, std::uint8_t mode, bool front_survey_complete,
    bool back_survey_complete, const Measurement * measurement, const std::string & detail)
  {
    CampaignStatus status;
    status.header.stamp = node_->now();
    status.header.frame_id = "world";
    status.sequence = ++status_sequence_;
    status.cycle = cycle_;
    // Card 086 stage 1 invariant 2: while any attempt is unresolved the campaign never reports
    // completion or a parked front-full; it reports the block and names the retained identities.
    status.motion_unresolved = unresolved_motion_.unresolved() || restart_recovery_pending_;
    status.restart_recovery_pending = restart_recovery_pending_;
    status.unresolved_attempts = unresolved_motion_.describe_each();
    if (unresolved_motion_.unresolved() &&
      (phase == CampaignStatus::PHASE_COMPLETE || phase == CampaignStatus::PHASE_FRONT_FULL))
    {
      phase = CampaignStatus::PHASE_BLOCKED;
    }
    status.phase = phase;
    status.mode = mode;
    status.front_survey_complete = front_survey_complete;
    status.back_survey_complete = back_survey_complete;
    status.successful_transfers = successful_transfers_;
    status.detail = unresolved_motion_.unresolved() ?
      unresolved_motion_.describe() + "; " + detail : detail;
    status.survey_arm_time_sec = survey_arm_time_sec_;
    status.transfer_arm_time_sec = transfer_arm_time_sec_;
    status.product_classes.assign(kKnownProductClasses.begin(), kKnownProductClasses.end());
    for (const auto product_class : kKnownProductClasses) {
      const auto slot = class_slot(product_class);
      status.front_stock.push_back(measurement ? measurement->front_stock[*slot] : 0U);
      status.front_deficits.push_back(
        measurement ? measurement->front_deficits[*slot] : 0U);
      status.back_stock.push_back(measurement ? measurement->back_stock[*slot] : 0U);
    }
    if (measurement) {
      status.world_revision = measurement->world_revision;
      status.total_front_stock = measurement->total_front_stock;
      status.total_front_deficit = measurement->total_front_deficit;
      status.total_back_stock = measurement->total_back_stock;
      last_measurement_ = *measurement;
    }
    campaign_status_publisher_->publish(status);

    if (measurement) {
      std::ostringstream summary;
      for (const auto product_class : kKnownProductClasses) {
        const auto slot = class_slot(product_class);
        summary << ' ' << product_class_name(product_class)
                << "=(front:" << measurement->front_stock[*slot]
                << ",missing:" << measurement->front_deficits[*slot]
                << ",back:" << measurement->back_stock[*slot] << ')';
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "CAMPAIGN_STATUS sequence=%" PRIu64 " cycle=%" PRId64
        " phase=%u mode=%u revision=%" PRIu64 " transfers=%" PRIu64
        " survey_arm=%.3f s transfer_arm=%.3f s%s%s%s",
        status.sequence, cycle_, static_cast<unsigned>(phase),
        static_cast<unsigned>(mode), measurement->world_revision, successful_transfers_,
        survey_arm_time_sec_, transfer_arm_time_sec_, summary.str().c_str(),
        detail.empty() ? "" : " detail=", detail.c_str());
    }
  }

  // Called only by the campaign worker, like every other status publication. The action
  // callback records request-local evidence and never reads or writes campaign state.
  void publish_confirm_status(CampaignConfirmReport::Source source)
  {
    publish_status(
      CampaignStatus::PHASE_SURVEYING_BACK, CampaignStatus::MODE_CONFIRM,
      front_survey_complete_, false, last_measurement_ ? &*last_measurement_ : nullptr,
      source == CampaignConfirmReport::Source::kFeedback ?
      "close-confirming the selected tray candidate" :
      "close-confirming the selected tray candidate "
      "(confirm feedback lost the ordering race to this result; emitted at the result)");
  }

  [[nodiscard]] bool wait_for_coordinator_ready() const
  {
    const auto deadline = std::chrono::steady_clock::now() + server_wait_timeout_;
    while (keep_running() && std::chrono::steady_clock::now() < deadline) {
      if (coordinator_ready_.load(std::memory_order_acquire)) {
        return true;
      }
      std::this_thread::sleep_for(100ms);
    }
    if (keep_running()) {
      RCLCPP_ERROR(
        node_->get_logger(), "coordinator did not report admission_ready on %s",
        coordinator_status_topic_.c_str());
    }
    return false;
  }

  // ---- Card 086 stage 1 (CMB-SPEC-13): unresolved-motion ownership ----------------------------
  // Everything below runs on the worker thread only, like the rest of the campaign state. The
  // registry is in memory: it does not survive a restart (see restart_recovery_pending_).

  [[nodiscard]] static MotionEvidence non_settling(std::string note)
  {
    MotionEvidence evidence;
    evidence.kind = MotionEvidence::Kind::kNonSettling;
    evidence.note = std::move(note);
    return evidence;
  }

  template<typename ActionT>
  struct LateAttempt
  {
    using Handle = typename rclcpp_action::ClientGoalHandle<ActionT>::SharedPtr;
    using Wrapped = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
    typename rclcpp_action::Client<ActionT>::SharedPtr client;
    GoalCancelClient::SharedPtr cancel_client;
    std::shared_future<Handle> admission;
    bool admission_pending{false};
    Handle handle;
    std::shared_future<Wrapped> result;
    bool result_pending{false};
  };

  // Registers an attempt whose outcome is unproven and keeps what can still settle it: the
  // admission future (a late admission is routed to this record, canceled by exact UUID, never
  // started) and/or the result future of an already-admitted goal. `admission` and `result`
  // may be invalid when that half is already known.
  template<typename ActionT>
  void retain_unresolved(
    MotionEndpoint endpoint, std::string label, UnresolvedCondition condition,
    typename rclcpp_action::Client<ActionT>::SharedPtr client,
    GoalCancelClient::SharedPtr cancel_client,
    std::shared_future<typename LateAttempt<ActionT>::Handle> admission,
    typename LateAttempt<ActionT>::Handle handle,
    std::shared_future<typename LateAttempt<ActionT>::Wrapped> result)
  {
    const auto id = unresolved_motion_.open(endpoint, condition, std::move(label));
    auto state = std::make_shared<LateAttempt<ActionT>>();
    state->client = std::move(client);
    state->cancel_client = std::move(cancel_client);
    state->admission = std::move(admission);
    state->admission_pending = state->admission.valid();
    state->handle = std::move(handle);
    state->result = std::move(result);
    state->result_pending = state->result.valid();
    if (state->handle) {
      unresolved_motion_.bind_goal(id, state->handle->get_goal_id());
    }
    RCLCPP_WARN(
      node_->get_logger(), "%s", unresolved_motion_.describe().c_str());
    late_attempt_pollers_[id] = [this, id, state]() {poll_late_attempt<ActionT>(id, *state);};
  }

  // A delivered terminal that does not settle (S3): nothing more can arrive for this goal, so
  // the record stays until the process restarts and an operator acknowledges.
  void record_unsettled_terminal(
    MotionEndpoint endpoint, std::string label, const rclcpp_action::GoalUUID & goal_id,
    const MotionEvidence & evidence)
  {
    const auto id = unresolved_motion_.open(
      endpoint, UnresolvedCondition::kResultUnresolved, std::move(label));
    unresolved_motion_.bind_goal(id, goal_id);
    unresolved_motion_.apply(id, evidence);
    RCLCPP_WARN(node_->get_logger(), "%s", unresolved_motion_.describe().c_str());
  }

  // True when the terminal settles, so the caller records nothing.
  template<typename ResultT>
  bool settle_or_record_terminal(
    MotionEndpoint endpoint, const std::string & label, const rclcpp_action::GoalUUID & goal_id,
    rclcpp_action::ResultCode code, const ResultT & result)
  {
    const auto evidence = evidence_of(code, result);
    if (settles_attempt(evidence)) {
      return true;
    }
    record_unsettled_terminal(endpoint, label, goal_id, evidence);
    return false;
  }

  template<typename ActionT>
  void poll_late_attempt(UnresolvedMotionRegistry::AttemptId id, LateAttempt<ActionT> & state)
  {
    if (state.admission_pending) {
      if (state.admission.wait_for(0s) != std::future_status::ready) {
        return;
      }
      state.admission_pending = false;
      state.handle = state.admission.get();
      if (!state.handle) {
        MotionEvidence rejection;
        rejection.kind = MotionEvidence::Kind::kAdmissionRejected;
        rejection.note = "late admission reply: rejected";
        unresolved_motion_.apply(id, rejection);
        return;
      }
      unresolved_motion_.bind_goal(id, state.handle->get_goal_id());
      unresolved_motion_.set_condition(id, UnresolvedCondition::kSettlementPendingCancel);
      RCLCPP_WARN(
        node_->get_logger(), "late admission for an unresolved attempt; canceling by exact id: %s",
        unresolved_motion_.describe().c_str());
      try {
        (void)state.client->async_cancel_goal(state.handle);
        state.result = state.client->async_get_result(state.handle);
        state.result_pending = true;
      } catch (const std::exception & error) {
        unresolved_motion_.apply(
          id, non_settling(std::string("late goal unknown to its client: ") + error.what()));
        cancel_by_goal_id(*state.cancel_client, state.handle->get_goal_id());
      }
    }
    if (state.result_pending) {
      if (state.result.wait_for(0s) != std::future_status::ready) {
        return;
      }
      state.result_pending = false;
      const auto wrapped = state.result.get();
      if (delivered_terminal(static_cast<std::int8_t>(wrapped.code)) && wrapped.result) {
        (void)unresolved_motion_.apply(id, evidence_of(wrapped.code, *wrapped.result));
        return;
      }
      // UNKNOWN (or a terminal without a payload) proves nothing and the client has forgotten
      // the goal: cancel by id again and keep the record.
      unresolved_motion_.apply(
        id, non_settling(
          "result code " + std::to_string(static_cast<int>(wrapped.code)) +
          " is not a delivered terminal"));
      if (state.handle) {
        cancel_by_goal_id(*state.cancel_client, state.handle->get_goal_id());
      }
    }
  }

  static void cancel_by_goal_id(GoalCancelClient & client, const rclcpp_action::GoalUUID & goal_id)
  {
    auto request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
    request->goal_info.goal_id.uuid = goal_id;
    (void)client.async_send_request(request);
  }

  // Non-blocking: routes whatever late admissions and results have arrived to their records.
  void pump_unresolved_motion()
  {
    std::vector<UnresolvedMotionRegistry::AttemptId> ids;
    ids.reserve(late_attempt_pollers_.size());
    for (const auto & [id, poller] : late_attempt_pollers_) {
      static_cast<void>(poller);
      ids.push_back(id);
    }
    for (const auto id : ids) {
      late_attempt_pollers_.at(id)();
      if (unresolved_motion_.find(id) == nullptr) {
        RCLCPP_INFO(
          node_->get_logger(),
          "unresolved motion attempt #%" PRIu64 " settled by exact-identity evidence", id);
        late_attempt_pollers_.erase(id);
      }
    }
  }

  // The send gate (invariant 1): while any attempt is unresolved, or a restart awaits its
  // acknowledgment, no goal of any endpoint is sent. Conjunctive with the 052 ladders.
  [[nodiscard]] bool motion_send_refused(std::string & detail)
  {
    pump_unresolved_motion();
    if (restart_recovery_pending_) {
      detail = "motion_unresolved: restart recovery awaits operator acknowledgment";
      return true;
    }
    if (unresolved_motion_.unresolved()) {
      detail = unresolved_motion_.describe();
      return true;
    }
    return false;
  }

  [[nodiscard]] LaneSurveyOutcome survey_lane(const std::string & lane_id)
  {
    LaneSurveyOutcome attempt;
    if (std::string refusal; motion_send_refused(refusal)) {
      // Nothing is sent: the arm's state cannot be read from a goal that was never issued.
      attempt.detail = "shelf survey not sent: " + refusal;
      return attempt;
    }
    if (!wait_for_server(shelf_survey_client_, shelf_survey_action_name_)) {
      // No goal was sent, so no terminal result can be read: the arm's state stays unknown
      // and the classification fails closed (Milestone 10 §6, Card 052).
      attempt.detail = "the shelf survey server was unavailable";
      return attempt;
    }
    RCLCPP_INFO(node_->get_logger(), "SURVEY_SHELF %s", lane_id.c_str());
    const auto started = std::chrono::steady_clock::now();
    SurveyLane::Goal goal;
    goal.lane_id = lane_id;
    auto accepted = shelf_survey_client_->async_send_goal(goal);
    if (!wait_for_future(accepted, action_timeout_)) {
      RCLCPP_ERROR(
        node_->get_logger(), "shelf survey %s timed out before admission",
        lane_id.c_str());
      attempt.detail = "shelf survey timed out before admission";
      retain_unresolved<SurveyLane>(
        MotionEndpoint::kLane, "lane survey " + lane_id, UnresolvedCondition::kAdmissionUnresolved,
        shelf_survey_client_, shelf_survey_cancel_client_, accepted, nullptr, {});
      return attempt;
    }
    const auto handle = accepted.get();
    if (!handle) {
      RCLCPP_ERROR(node_->get_logger(), "shelf survey %s was rejected", lane_id.c_str());
      attempt.detail = "shelf survey was rejected";
      return attempt;
    }
    auto completed = shelf_survey_client_->async_get_result(handle);
    if (!wait_for_future(completed, action_timeout_)) {
      (void)shelf_survey_client_->async_cancel_goal(handle);
      // Wait out the canceled goal's terminal result so the action never outlives this
      // call; a server that never answers loses only this bounded second wait.
      auto canceled = shelf_survey_client_->async_get_result(handle);
      const bool canceled_delivered = wait_for_future(canceled, action_timeout_);
      // The arm moved until the cancel: bill that in-flight time to the survey budget
      // even though the goal never reported a terminal success.
      survey_arm_time_sec_ +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      RCLCPP_ERROR(
        node_->get_logger(), "shelf survey %s timed out and was canceled",
        lane_id.c_str());
      attempt.detail = "shelf survey timed out and was canceled";
      if (canceled_delivered) {
        // The cancel's own terminal result arrived: the attempt is over and its evidence
        // flags may be read (a canceled result without either flag still refuses the stop).
        const auto canceled_wrapped = canceled.get();
        if (canceled_wrapped.result) {
          attempt.terminal_result_received = true;
          attempt.outcome = canceled_wrapped.result->outcome;
          attempt.motion_definitely_not_started =
            canceled_wrapped.result->motion_definitely_not_started;
          attempt.execution_reached_terminal_stop =
            canceled_wrapped.result->execution_reached_terminal_stop;
          static_cast<void>(settle_or_record_terminal(
            MotionEndpoint::kLane, "lane survey " + lane_id, handle->get_goal_id(),
            canceled_wrapped.code, *canceled_wrapped.result));
        } else {
          retain_unresolved<SurveyLane>(
            MotionEndpoint::kLane, "lane survey " + lane_id,
            UnresolvedCondition::kSettlementPendingCancel, shelf_survey_client_,
            shelf_survey_cancel_client_, {}, handle, completed);
        }
      } else {
        retain_unresolved<SurveyLane>(
          MotionEndpoint::kLane, "lane survey " + lane_id,
          UnresolvedCondition::kSettlementPendingCancel, shelf_survey_client_,
          shelf_survey_cancel_client_, {}, handle, canceled);
      }
      return attempt;
    }
    survey_arm_time_sec_ +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const auto wrapped = completed.get();
    // Only a delivered terminal carries evidence (Milestone 10 §6 result finality, Card 070);
    // any other code leaves the goal possibly running, so it is canceled by ID and read as no
    // terminal result.
    if (!accept_delivered_terminal_or_cancel(
        static_cast<std::int8_t>(wrapped.code), handle->get_goal_id(),
        *shelf_survey_cancel_client_))
    {
      attempt.detail = "shelf survey returned no terminal result (code " +
        std::to_string(static_cast<int>(wrapped.code)) + ") and was canceled";
      {
        const auto id = unresolved_motion_.open(
          MotionEndpoint::kLane, UnresolvedCondition::kSettlementPendingCancel,
          "lane survey " + lane_id);
        unresolved_motion_.bind_goal(id, handle->get_goal_id());
        unresolved_motion_.apply(id, non_settling("result code UNKNOWN; canceled by exact id"));
      }
      RCLCPP_ERROR(
        node_->get_logger(), "shelf survey %s: %s", lane_id.c_str(), attempt.detail.c_str());
      return attempt;
    }
    attempt.terminal_result_received = wrapped.result != nullptr;
    attempt.observed = wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
      wrapped.result->outcome == SurveyLane::Result::OUTCOME_OBSERVED;
    if (wrapped.result) {
      attempt.outcome = wrapped.result->outcome;
      attempt.detail = wrapped.result->detail;
      attempt.motion_definitely_not_started = wrapped.result->motion_definitely_not_started;
      attempt.execution_reached_terminal_stop =
        wrapped.result->execution_reached_terminal_stop;
    }
    if (wrapped.result) {
      static_cast<void>(settle_or_record_terminal(
        MotionEndpoint::kLane, "lane survey " + lane_id, handle->get_goal_id(), wrapped.code,
        *wrapped.result));
    }
    if (!attempt.observed) {
      RCLCPP_ERROR(
        node_->get_logger(), "shelf survey %s failed%s%s", lane_id.c_str(),
        !attempt.detail.empty() ? ": " : "", attempt.detail.c_str());
    }
    return attempt;
  }

  // Milestone 10 §6, Card 052: classify a survey/confirm action failure at the campaign
  // boundary with Card 051's rules, and receipt it on one line exactly as the coordinator
  // does at its own boundaries. What each call site does with the class is the ladder's
  // decision (behaviour lands in the rung commits); this only reports what is established
  // after a check said no.
  [[nodiscard]] RecoveryClassification receipt_survey_failure(
    SurveyFailureEvidence evidence, const bool world_reobservable = true)
  {
    evidence.world_reobservable = world_reobservable;
    evidence.transfer_release_established = transfer_release_established_;
    const RecoveryClassification classification =
      classify_recovery(survey_recovery_context(evidence));
    RCLCPP_INFO(
      node_->get_logger(), "%s",
      recovery_classification_receipt(classification, evidence.cause).c_str());
    return classification;
  }

  // A measurement-boundary failure: no action is in flight and nothing was commanded, so
  // the arm half is established; what fails is re-observability itself — the world service
  // that would feed the next attempt just did not answer, which is UNSAFE by definition.
  // Not [[nodiscard]] (review F4): every call site uses it purely for the receipt side
  // effect — the class is implied by the cause.
  RecoveryClassification receipt_world_failure(const std::string & cause)
  {
    SurveyFailureEvidence evidence;
    evidence.terminal_result_received = true;
    evidence.motion_definitely_not_started = true;
    evidence.cause = cause;
    return receipt_survey_failure(evidence, false);
  }

  // A confirmation-identity defect: the tray survey's own motion completed (its stop is
  // established), but the confirmed identity cannot be bound to the world-state snapshot
  // the transfer would rely on — UNSAFE by world re-observability, never retried blind
  // (Milestone 10 §1 confirmed-identity contract, Card 037; Milestone 10 §6, Card 052).
  // Not [[nodiscard]] (review F4): every call site uses it purely for the receipt side
  // effect — the class is implied by the cause.
  RecoveryClassification receipt_identity_failure(const std::string & cause)
  {
    SurveyFailureEvidence evidence;
    evidence.terminal_result_received = true;
    evidence.execution_reached_terminal_stop = true;
    evidence.cause = cause;
    return receipt_survey_failure(evidence, false);
  }

  // Milestone 10 §6 rung 5 (Card 052): charge one campaign survey-skip against
  // max_survey_skips. False means the budget is already spent — the caller latches
  // PHASE_BLOCKED naming the rung; true means the resource is skipped for THIS cycle only
  // and is revisited next cycle (the charge itself is per run).
  [[nodiscard]] bool charge_survey_skip(const std::string & what)
  {
    if (survey_skip_charged_ >= max_survey_skips_) {
      return false;
    }
    ++survey_skip_charged_;
    RCLCPP_INFO(
      node_->get_logger(),
      "recovery rung 5 (skip): %s skipped for this cycle (survey skips %" PRId64
      " of %" PRId64 "; revisited next cycle)",
      what.c_str(), survey_skip_charged_, max_survey_skips_);
    return true;
  }

  [[nodiscard]] static std::string survey_skip_exhausted_detail(const std::string & what)
  {
    return "survey recovery exhausted at rung 5 (skip): " + what +
           " — max_survey_skips is spent for this run";
  }

  // Milestone 10 §6 rung 1 (Card 052): after a recoverable failure whose attempt may have
  // moved the arm, retreat to the known-safe survey station before anything else is
  // commanded — at most one retreat per recovery attempt (the caller invokes this once per
  // failed attempt). False only when the retreat itself leaves the arm's state
  // unestablished, so the caller can classify UNSAFE; a retreat that never commanded (no
  // server, refused, plan refused) leaves the failed attempt's established stop standing
  // and is the rung's honest no-op with its receipt. Every exit carries a
  // `recovery classification:` line: UNSAFE before a `return false` (the caller latches
  // PHASE_BLOCKED right after), RECOVERABLE before a `return true` no-op that continues
  // the ladder (review F3).
  [[nodiscard]] bool retreat_to_safe_survey_pose(const std::string & cause)
  {
    if (std::string refusal; motion_send_refused(refusal)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): retreat not sent, motion is unresolved; the arm's "
        "state is not established (%s): %s", cause.c_str(), refusal.c_str());
      return false;
    }
    if (!wait_for_server(viewpoint_client_, recovery_viewpoint_action_name_)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): retreat server %s unavailable; the failed "
        "attempt's established stop stands (%s)",
        recovery_viewpoint_action_name_.c_str(), cause.c_str());
      RCLCPP_INFO(
        node_->get_logger(), "%s",
        recovery_classification_receipt(
          {RecoveryClass::kRecoverable,
            "the retreat commanded nothing; the failed attempt's "
            "established stop stands"},
          "SURVEY recovery rung 1 retreat server unavailable: " + cause).c_str());
      return true;
    }
    SurveyViewpoint::Goal goal;
    goal.station = recovery_safe_station_;
    goal.label = "survey recovery retreat";
    auto accepted = viewpoint_client_->async_send_goal(goal);
    if (!wait_for_future(accepted, action_timeout_)) {
      // Review F1 (Card 052), fail closed: this wait ends on the deadline or the moment a
      // termination cuts it short — in both cases the goal may already have been admitted
      // while we hold no handle. Request cancellation of whatever this dedicated retreat
      // client may have admitted (only while the context is still live: a handle that
      // arrived at the wait's boundary is cancelled directly), and report the arm's state
      // as no longer established instead of claiming "nothing was commanded".
      if (keep_running()) {
        if (accepted.wait_for(0s) == std::future_status::ready) {
          const auto late_handle = accepted.get();
          if (late_handle) {
            (void)viewpoint_client_->async_cancel_goal(late_handle);
          }
        }
        // The client is dedicated to retreat goals, so cancelling everything it owns
        // cannot touch any other action.
        (void)viewpoint_client_->async_cancel_all_goals();
      }
      // The late admission is routed to this record, canceled by exact UUID and only a delivered
      // terminal with settling evidence clears it (Card 086 stage 1, S1/S4).
      retain_unresolved<SurveyViewpoint>(
        MotionEndpoint::kViewpoint, "survey recovery retreat",
        UnresolvedCondition::kAdmissionUnresolved, viewpoint_client_, viewpoint_cancel_client_,
        accepted, nullptr, {});
      RCLCPP_WARN(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): no retreat admission answer within the wait; "
        "a possible admitted goal is being cancel-requested%s and the arm's state is no "
        "longer established (%s)",
        keep_running() ? "" : " (skipped: the context is going down)", cause.c_str());
      SurveyFailureEvidence evidence;
      evidence.cause = "SURVEY recovery rung 1 retreat admission unanswered: " + cause;
      (void)receipt_survey_failure(evidence);
      return false;
    }
    const auto handle = accepted.get();
    if (!handle) {
      RCLCPP_INFO(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): retreat refused before any command; the "
        "failed attempt's stop stands (%s)",
        cause.c_str());
      RCLCPP_INFO(
        node_->get_logger(), "%s",
        recovery_classification_receipt(
          {RecoveryClass::kRecoverable,
            "the server refused the retreat before any "
            "command; the failed attempt's established stop stands"},
          "SURVEY recovery rung 1 retreat refused: " + cause).c_str());
      return true;
    }
    auto completed = viewpoint_client_->async_get_result(handle);
    if (!wait_for_future(completed, action_timeout_)) {
      (void)viewpoint_client_->async_cancel_goal(handle);
      auto canceled = viewpoint_client_->async_get_result(handle);
      std::shared_ptr<const SurveyViewpoint::Result> terminal;
      if (wait_for_future(canceled, action_timeout_)) {
        const auto canceled_wrapped = canceled.get();
        terminal = canceled_wrapped.result;
        if (terminal) {
          static_cast<void>(settle_or_record_terminal(
            MotionEndpoint::kViewpoint, "survey recovery retreat", handle->get_goal_id(),
            canceled_wrapped.code, *terminal));
        } else {
          record_unsettled_terminal(
            MotionEndpoint::kViewpoint, "survey recovery retreat", handle->get_goal_id(),
            non_settling("canceled retreat delivered without a payload"));
        }
      } else {
        retain_unresolved<SurveyViewpoint>(
          MotionEndpoint::kViewpoint, "survey recovery retreat",
          UnresolvedCondition::kSettlementPendingCancel, viewpoint_client_,
          viewpoint_cancel_client_, {}, handle, completed);
      }
      if (!terminal) {
        RCLCPP_WARN(
          node_->get_logger(),
          "recovery rung 1 (safe survey pose): the retreat ended without terminal evidence; "
          "the arm's state is no longer established (%s)",
          cause.c_str());
        SurveyFailureEvidence evidence;
        evidence.cause =
          "SURVEY recovery rung 1 retreat canceled without terminal evidence: " + cause;
        (void)receipt_survey_failure(evidence);
        return false;
      }
      const bool established = viewpoint_leg_established(
        terminal->outcome, terminal->execution_reached_terminal_stop);
      if (!established) {
        SurveyFailureEvidence evidence;
        evidence.terminal_result_received = true;
        evidence.cause = "SURVEY recovery rung 1 canceled retreat outcome " +
          std::to_string(terminal->outcome) + " did not establish a stop: " + cause;
        (void)receipt_survey_failure(evidence);
      }
      RCLCPP_INFO(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): canceled retreat outcome %u (%s)",
        static_cast<unsigned>(terminal->outcome), cause.c_str());
      return established;
    }
    const auto wrapped = completed.get();
    // Card 070: a code that is not a delivered terminal is no result; the retreat may still be
    // running, so it is canceled by ID.
    if (!accept_delivered_terminal_or_cancel(
        static_cast<std::int8_t>(wrapped.code), handle->get_goal_id(),
        *viewpoint_cancel_client_) || !wrapped.result)
    {
      {
        const auto id = unresolved_motion_.open(
          MotionEndpoint::kViewpoint, UnresolvedCondition::kSettlementPendingCancel,
          "survey recovery retreat");
        unresolved_motion_.bind_goal(id, handle->get_goal_id());
        unresolved_motion_.apply(
          id, non_settling(
            "retreat result code " +
            std::to_string(static_cast<int>(wrapped.code)) + " is not a settling terminal"));
      }
      RCLCPP_WARN(
        node_->get_logger(),
        "recovery rung 1 (safe survey pose): the retreat ended without a result; the arm's "
        "state is no longer established (%s)",
        cause.c_str());
      SurveyFailureEvidence evidence;
      evidence.cause = "SURVEY recovery rung 1 retreat ended without a result: " + cause;
      (void)receipt_survey_failure(evidence);
      return false;
    }
    static_cast<void>(settle_or_record_terminal(
      MotionEndpoint::kViewpoint, "survey recovery retreat", handle->get_goal_id(), wrapped.code,
      *wrapped.result));
    const bool established = viewpoint_leg_established(
      wrapped.result->outcome, wrapped.result->execution_reached_terminal_stop);
    if (!established) {
      SurveyFailureEvidence evidence;
      evidence.terminal_result_received = true;
      evidence.cause = "SURVEY recovery rung 1 retreat outcome " +
        std::to_string(wrapped.result->outcome) + " did not establish a stop: " + cause;
      (void)receipt_survey_failure(evidence);
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "recovery rung 1 (safe survey pose): retreat to %s outcome %u%s (%s)",
      recovery_safe_station_.c_str(), static_cast<unsigned>(wrapped.result->outcome),
      established ? "" : " without an established stop", cause.c_str());
    return established;
  }

  // Milestone 10 §6, Card 058: would a re-survey with the live skip marks lifted select
  // something? True iff some class-matching STATUS_OK overview candidate of this goal is
  // excluded only by skip marks whose product skip budget is still alive — a genuine
  // confirm-refutation or a gone-for-the-run mark keeps the candidate excluded, so those
  // return false for that candidate. The radius is the campaign's own merge-radius twin
  // (merge_radius_m_: the radius the tray server last reported applying, 0.04 m until then).
  // Card 066 × Card 058 (integration): lifting a mark cannot make a product nominable while
  // something stands ahead of it in its feed column, so only a candidate the server reports as a
  // feed-column front (`feed_fronts`, parallel to `candidates`) counts. A missing or mismatched
  // report counts nothing: no lift is charged on geometry the campaign cannot see.
  [[nodiscard]] bool skip_lift_would_select(
    const std::vector<restocker_interfaces::msg::ObjectObservation> & candidates,
    const std::vector<bool> & feed_fronts, const std::uint8_t product_class) const
  {
    if (feed_fronts.size() != candidates.size()) {
      return false;
    }
    using Observation = restocker_interfaces::msg::ObjectObservation;
    const double radius_squared = merge_radius_m_ * merge_radius_m_;
    const auto near = [radius_squared](
      const geometry_msgs::msg::Point & mark, const geometry_msgs::msg::Point & position) {
      const double dx = mark.x - position.x;
      const double dy = mark.y - position.y;
      const double dz = mark.z - position.z;
      return dx * dx + dy * dy + dz * dz <= radius_squared;
    };
    for (std::size_t index = 0U; index < candidates.size(); ++index) {
      const auto & candidate = candidates[index];
      if (candidate.status != Observation::STATUS_OK || candidate.product_class != product_class ||
        !feed_fronts[index])
      {
        continue;
      }
      const auto & position = candidate.pose.pose.position;
      const bool refuted_here = std::ranges::any_of(
        refuted_positions_, [&near, &position](const geometry_msgs::msg::Point & mark) {
          return near(mark, position);
        });
      if (refuted_here) {
        continue;
      }
      bool near_live = false;
      bool near_gone = false;
      for (const auto & [source, point] : skip_marks_) {
        if (!near(point, position)) {
          continue;
        }
        const auto counted = product_skip_counts_.find(source);
        if (counted != product_skip_counts_.end() &&
          counted->second >= static_cast<std::size_t>(max_product_skips_))
        {
          near_gone = true;
        } else {
          near_live = true;
        }
      }
      if (near_live && !near_gone) {
        return true;
      }
    }
    return false;
  }

  // Milestone 10 §6, Card 069: the bounded confirmation of observed absence. Only
  // PROBE_VIEWED_EMPTY extends a product's streak; PROBE_SEEN or an observation newer than the
  // streak's anchor resets it; the other verdicts are neither evidence of absence nor of
  // presence. A streak of absence_confirmations consecutive completed surveys retires the
  // product (campaign-local; measure_world honours it until a newer observation).
  void record_absence_verdicts(
    const std::vector<std::string> & sources, const std::vector<std::uint8_t> & verdicts)
  {
    if (sources.empty() || verdicts.size() != sources.size() || !last_measurement_) {
      return;
    }
    // A streak for a product this survey did not probe is stale: it fell out of the count,
    // left for the front, or was retired (review N8).
    for (auto it = absence_streaks_.begin(); it != absence_streaks_.end(); ) {
      if (std::ranges::find(sources, it->first) == sources.end()) {
        it = absence_streaks_.erase(it);
      } else {
        ++it;
      }
    }
    const auto now = std::chrono::steady_clock::now();
    for (std::size_t index = 0; index < sources.size(); ++index) {
      const std::string & source = sources[index];
      const std::uint8_t verdict = verdicts[index];
      if (verdict == SurveyTray::Result::PROBE_SEEN) {
        absence_streaks_.erase(source);
        continue;
      }
      if (verdict != SurveyTray::Result::PROBE_VIEWED_EMPTY) {
        continue;
      }
      const auto observed = last_measurement_->tray_stock_observation_ns.find(source);
      if (observed == last_measurement_->tray_stock_observation_ns.end()) {
        continue;
      }
      // Already retired against this observation (an earlier survey of the same cycle, whose
      // measurement still lists the product): nothing further to count.
      const auto retired = retired_sources_.find(source);
      if (retired != retired_sources_.end() && observed->second <= retired->second) {
        continue;
      }
      AbsenceStreak & streak = absence_streaks_[source];
      if (streak.count > 0 && streak.observation_ns != observed->second) {
        streak.count = 0;
      }
      if (streak.count > 0 &&
        (streak.cycle == cycle_ || now - streak.counted_at < absence_min_separation_))
      {
        RCLCPP_INFO(
          node_->get_logger(),
          "tray product %s: place viewed empty again, not counted — not independent of the "
          "previous counted view (same cycle, or within absence_min_separation_sec=%.1f)",
          source.c_str(), absence_min_separation_.count());
        continue;
      }
      streak.observation_ns = observed->second;
      streak.cycle = cycle_;
      streak.counted_at = now;
      ++streak.count;
      if (streak.count < static_cast<std::size_t>(absence_confirmations_)) {
        RCLCPP_INFO(
          node_->get_logger(),
          "tray product %s: place viewed empty by a completed survey (%zu of %" PRId64 ")",
          source.c_str(), streak.count, absence_confirmations_);
        continue;
      }
      retired_sources_[source] = observed->second;
      absence_streaks_.erase(source);
      const std::string receipt = "tray product " + source +
        " retired from back stock: completed tray surveys viewed its place empty in " +
        std::to_string(absence_confirmations_) +
        " independent counted views (absence_confirmations=" +
        std::to_string(absence_confirmations_) + "); a newer observation restores it";
      RCLCPP_WARN(node_->get_logger(), "%s", receipt.c_str());
      // The receipt carries the counts as they stand after the retirement (review N7).
      Measurement after = *last_measurement_;
      const auto slot = after.tray_stock_slots.find(source);
      if (slot != after.tray_stock_slots.end() && after.back_stock[slot->second] > 0U) {
        --after.back_stock[slot->second];
        --after.total_back_stock;
      }
      publish_status(
        CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_SURVEY_TRAY, true,
        back_survey_complete_, &after, receipt);
    }
  }

  // Milestone 10 §6, "The no-candidate park names what each overview station saw"
  // (Card 080): one attribution segment per visited station, reporting only. A station that
  // admitted nothing carries its Card 050/064 stage label and counts (empty view vs refused
  // proposals vs input gap are distinguishable at the terminal itself); a station that
  // admitted frames carries its admitted count. No reports -> empty string, so the park's
  // detail stays byte-identical to what it reads today.
  [[nodiscard]] static std::string station_attribution(
    const std::vector<restocker_interfaces::msg::TrayStationAcquisition> & reports)
  {
    std::string text;
    for (const auto & report : reports) {
      text += "; station " + report.station + ": ";
      if (report.admitted > 0U) {
        text += "admitted=" + std::to_string(report.admitted);
      } else {
        text += classify_drained_station(report);
        text += " (images=" + std::to_string(report.images) +
          ", detection_frames=" + std::to_string(report.detection_frames) +
          ", frames_with_detections=" + std::to_string(report.frames_with_detections) +
          ", published=" + std::to_string(report.published) + ")";
      }
    }
    return text;
  }

  // The coordinated tray survey: overview stations, selection, close confirmation. Mode
  // reporting rides the action's own feedback so CONFIRM is published when the close view is
  // actually being taken, not when the goal was sent. An empty `overview_stations` keeps the
  // server's nominal derivation (the pre-ladder behaviour); a non-empty list is the campaign
  // ladder's rotated/skipped station set (Milestone 10 §6, Card 052).
  [[nodiscard]] TraySurveyOutcome survey_tray(
    std::uint8_t product_class, const std::vector<std::string> & overview_stations = {})
  {
    TraySurveyOutcome outcome;
    if (std::string refusal; motion_send_refused(refusal)) {
      outcome.detail = "tray survey not sent: " + refusal;
      return outcome;
    }
    if (!wait_for_server(tray_survey_client_, tray_survey_action_name_)) {
      outcome.detail = "the coordinated tray survey server was unavailable";
      return outcome;
    }
    RCLCPP_INFO(
      node_->get_logger(),
      "SURVEY_TRAY class=%u with %zu refuted candidate(s), %zu skip mark(s), %zu station(s)",
      static_cast<unsigned>(product_class), refuted_positions_.size(), skip_marks_.size(),
      overview_stations.size());
    const auto started = std::chrono::steady_clock::now();
    SurveyTray::Goal goal;
    goal.product_class = product_class;
    goal.refuted_positions = refuted_positions_;
    goal.overview_stations = overview_stations;
    // Skip marks travel with every survey in their own field (Milestone 10 §6 rung 5, Card 051;
    // split from refuted_positions per Card 058): a product the ladder skipped is excluded from
    // reselection until its evidence changes or its budget runs out, and the split lets the
    // result detail name which exclusion kind actually reached the candidates. The lift rung
    // (§6, Card 058) re-surveys with the marks of products whose skip budget is still alive
    // omitted — "try others first" ends when nothing else is selectable — while gone-for-the-
    // run marks and every confirm-refutation stay excluded. The lift applies only to a goal of
    // the class that qualified and is spent the moment that survey is sent, whichever way the
    // survey ends.
    const bool lift = active_skip_lift_class_ == product_class;
    for (const auto & [source, point] : skip_marks_) {
      if (lift) {
        const auto counted = product_skip_counts_.find(source);
        if (counted != product_skip_counts_.end() &&
          counted->second < static_cast<std::size_t>(max_product_skips_))
        {
          continue;
        }
      }
      goal.skip_mark_positions.push_back(point);
    }
    if (lift) {
      active_skip_lift_class_.reset();
    }
    // Milestone 10 §6, Card 069: every product counted as back stock is probed for absence.
    std::vector<std::string> probe_sources;
    if (last_measurement_) {
      for (const auto & [source, point] : last_measurement_->tray_stock_positions) {
        probe_sources.push_back(source);
        goal.absence_probe_positions.push_back(point);
      }
      // Every other tracked tray object (fallen, retired) may stand in a line of sight
      // (review N4): it is sent as a potential occluder, never probed.
      for (const auto & [source, point] : last_measurement_->tray_candidate_positions) {
        if (!last_measurement_->tray_stock_positions.contains(source)) {
          goal.absence_occluder_positions.push_back(point);
        }
      }
    }
    // Each request owns its inbox. Late callbacks retain only that inbox, not this campaign
    // or the next request's report slot. The worker drains it before advancing on every exit.
    CampaignConfirmReport confirm_report(
      [this](CampaignConfirmReport::Source source) {publish_confirm_status(source);});
    rclcpp_action::Client<SurveyTray>::SendGoalOptions options;
    options.feedback_callback =
      [record_confirm = confirm_report.feedback_callback()](
      rclcpp_action::ClientGoalHandle<SurveyTray>::SharedPtr,
      const std::shared_ptr<const SurveyTray::Feedback> feedback) {
        if (feedback && feedback->phase == SurveyTray::Feedback::PHASE_CONFIRMING) {
          record_confirm();
        }
      };
    auto accepted = tray_survey_client_->async_send_goal(goal, options);
    if (!wait_for_future(
        accepted, action_timeout_, [&confirm_report]() {confirm_report.poll();}))
    {
      confirm_report.finish();
      outcome.detail = "tray survey timed out before admission";
      retain_unresolved<SurveyTray>(
        MotionEndpoint::kTray, "tray survey class " + std::to_string(product_class),
        UnresolvedCondition::kAdmissionUnresolved, tray_survey_client_,
        tray_survey_cancel_client_, accepted, nullptr, {});
      return outcome;
    }
    const auto handle = accepted.get();
    if (!handle) {
      confirm_report.finish();
      outcome.detail = "tray survey was rejected";
      return outcome;
    }
    auto completed = tray_survey_client_->async_get_result(handle);
    if (!wait_for_future(
        completed, action_timeout_, [&confirm_report]() {confirm_report.poll();}))
    {
      confirm_report.finish();
      (void)tray_survey_client_->async_cancel_goal(handle);
      // Bill the in-flight arm time the cancel cut short; pre-admission exits above the
      // handle never moved the arm and stay unbilled.
      survey_arm_time_sec_ +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      outcome.detail = "tray survey timed out and was canceled";
      retain_unresolved<SurveyTray>(
        MotionEndpoint::kTray, "tray survey class " + std::to_string(product_class),
        UnresolvedCondition::kSettlementPendingCancel, tray_survey_client_,
        tray_survey_cancel_client_, {}, handle, completed);
      return outcome;
    }
    survey_arm_time_sec_ +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const auto wrapped = completed.get();
    // Close and drain before result processing can change measurements, mode, or cycle.
    // Only these delivered outcomes establish confirmation without observed feedback.
    confirm_report.finish(
      wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && wrapped.result &&
      (wrapped.result->outcome == SurveyTray::Result::OUTCOME_CONFIRMED ||
      wrapped.result->outcome == SurveyTray::Result::OUTCOME_REFUTED));
    // Card 070: a code that is not a delivered terminal leaves the survey possibly running.
    if (!accept_delivered_terminal_or_cancel(
        static_cast<std::int8_t>(wrapped.code), handle->get_goal_id(),
        *tray_survey_cancel_client_))
    {
      outcome.detail = "tray survey returned no terminal result (code " +
        std::to_string(static_cast<int>(wrapped.code)) + ") and was canceled";
      {
        const auto id = unresolved_motion_.open(
          MotionEndpoint::kTray, UnresolvedCondition::kSettlementPendingCancel,
          "tray survey class " + std::to_string(product_class));
        unresolved_motion_.bind_goal(id, handle->get_goal_id());
        unresolved_motion_.apply(id, non_settling("result code UNKNOWN; canceled by exact id"));
      }
      return outcome;
    }
    // Delivered terminal: settle it by its own payload, or retain it (S3). The tray payload
    // carries no statement about a child leg it may have left running (tray_survey_node
    // run_leg), so only arrival, non-start or stop evidence settles it.
    if (wrapped.result) {
      static_cast<void>(settle_or_record_terminal(
        MotionEndpoint::kTray, "tray survey class " + std::to_string(product_class),
        handle->get_goal_id(), wrapped.code, *wrapped.result));
    } else {
      record_unsettled_terminal(
        MotionEndpoint::kTray, "tray survey class " + std::to_string(product_class),
        handle->get_goal_id(), non_settling("delivered terminal without a payload"));
    }
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      outcome.detail = "tray survey ended without a result";
      return outcome;
    }
    outcome.completed = true;
    outcome.outcome = wrapped.result->outcome;
    outcome.detail = wrapped.result->detail;
    outcome.motion_definitely_not_started = wrapped.result->motion_definitely_not_started;
    outcome.execution_reached_terminal_stop = wrapped.result->execution_reached_terminal_stop;
    outcome.failed_station = wrapped.result->failed_station;
    outcome.feed_blocked_candidates = wrapped.result->feed_blocked_candidates;
    outcome.overview_candidate_feed_front = wrapped.result->overview_candidate_feed_front;
    outcome.marked_feed_blocked_candidates = wrapped.result->marked_feed_blocked_candidates;
    outcome.feed_block_example = wrapped.result->feed_block_example;
    // One merge radius (review cmbrev066b N2): match marks, candidates and world-state products
    // with the radius the server applied, once it has reported one.
    if (std::isfinite(wrapped.result->candidate_merge_radius_m) &&
      wrapped.result->candidate_merge_radius_m > 0.0)
    {
      merge_radius_m_ = wrapped.result->candidate_merge_radius_m;
    }
    // Milestone 10 §6, Card 066 (skip path): the latest overview that can vouch for fronts. An
    // empty overview vouches for nothing and does not replace it.
    if (!wrapped.result->overview_candidates.empty() &&
      wrapped.result->overview_candidate_feed_front.size() ==
      wrapped.result->overview_candidates.size())
    {
      reference_overview_ = wrapped.result->overview_candidates;
      reference_overview_fronts_ = wrapped.result->overview_candidate_feed_front;
    }
    outcome.overview_candidates = wrapped.result->overview_candidates;
    outcome.absence_probe_verdicts = wrapped.result->absence_probe_verdicts;
    outcome.overview_station_reports = wrapped.result->overview_station_reports;
    record_absence_verdicts(probe_sources, outcome.absence_probe_verdicts);
    if (wrapped.result->outcome == SurveyTray::Result::OUTCOME_CONFIRMED) {
      outcome.confirmed_source_object_id =
        wrapped.result->confirmed_observation.source_object_id;
    }
    if (wrapped.result->outcome == SurveyTray::Result::OUTCOME_REFUTED) {
      geometry_msgs::msg::Point marked;
      marked.x = wrapped.result->selected_candidate.pose.pose.position.x;
      marked.y = wrapped.result->selected_candidate.pose.pose.position.y;
      marked.z = wrapped.result->selected_candidate.pose.pose.position.z;
      outcome.refuted_position = marked;
    }
    return outcome;
  }

  [[nodiscard]] std::optional<Measurement> measurement_snapshot()
  {
    return last_measurement_;
  }

  void set_last_measurement(const Measurement & measurement)
  {
    last_measurement_ = measurement;
  }

  // One transfer goal. The optional pin names the product only, never the lane: a cycle that
  // close-confirmed a candidate sends that product's identity so selection cannot substitute a
  // different one (Milestone 10 §1 confirmed-identity contract, Card 037), while the destination
  // stays the coordinator's — the campaign never becomes a second manipulation authority. A
  // cycle that skipped the tray survey pins nothing and keeps today's empty selector.
  [[nodiscard]] bool request_transfer(
    std::string & detail,
    const std::optional<restocker_world_state::ObjectId> & confirmed_object,
    bool & recoverable_skip,
    std::uint64_t & skipped_object_id)
  {
    recoverable_skip = false;
    skipped_object_id = 0U;
    if (std::string refusal; motion_send_refused(refusal)) {
      // Nothing was sent, so the held-object state is whatever it was.
      detail = "restock goal not sent: " + refusal;
      return false;
    }
    if (!wait_for_server(restock_client_, restock_action_name_)) {
      // Nothing was sent; the held-object state is whatever the previous terminal left it.
      detail = "restock action server unavailable";
      return false;
    }
    const auto started = std::chrono::steady_clock::now();
    RestockProduct::Goal goal;
    if (confirmed_object) {
      goal.has_object_id = true;
      goal.object_id = confirmed_object->value;
    }
    auto accepted = restock_client_->async_send_goal(goal);
    if (!wait_for_future(accepted, action_timeout_)) {
      // The goal may have been accepted server-side without an answer: whether anything is
      // held can no longer be established from this campaign's own receipts, so the held
      // half of the classification fails closed (Milestone 10 §6, Card 052).
      transfer_release_established_ = false;
      detail = "restock goal timed out before admission";
      retain_unresolved<RestockProduct>(
        MotionEndpoint::kRestock, "restock transfer", UnresolvedCondition::kAdmissionUnresolved,
        restock_client_, restock_cancel_client_, accepted, nullptr, {});
      return false;
    }
    const auto handle = accepted.get();
    if (!handle) {
      // Rejected: the goal never started, nothing attachable happened this call.
      detail = "restock goal was rejected";
      return false;
    }
    auto completed = restock_client_->async_get_result(handle);
    if (!wait_for_future(completed, action_timeout_)) {
      (void)restock_client_->async_cancel_goal(handle);
      // Bill the in-flight arm time the cancel cut short (same rule as the surveys).
      transfer_arm_time_sec_ +=
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
      // A goal cancelled from in flight may have attached on its way out: unknown held
      // state, fail closed (Milestone 10 §6, Card 052).
      transfer_release_established_ = false;
      detail = "restock goal timed out and was canceled";
      retain_unresolved<RestockProduct>(
        MotionEndpoint::kRestock, "restock transfer",
        UnresolvedCondition::kSettlementPendingCancel, restock_client_, restock_cancel_client_,
        {}, handle, completed);
      return false;
    }
    transfer_arm_time_sec_ +=
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const auto wrapped = completed.get();
    if (!accept_delivered_terminal_or_cancel(
        static_cast<std::int8_t>(wrapped.code), handle->get_goal_id(), *restock_cancel_client_))
    {
      // Not a terminal the coordinator delivered (Milestone 10 §6 result finality, Card 070):
      // rclcpp_action reports UNKNOWN when its result request found no registered goal. The
      // goal may still be running and holding its reservation, so it was canceled by ID (the
      // client no longer knows its handle) and the held state fails closed.
      transfer_release_established_ = false;
      detail = "restock action returned no terminal result (code " +
        std::to_string(static_cast<int>(wrapped.code)) +
        ") and was canceled; the goal's outcome and held state are unknown";
      {
        const auto id = unresolved_motion_.open(
          MotionEndpoint::kRestock, UnresolvedCondition::kSettlementPendingCancel,
          "restock transfer");
        unresolved_motion_.bind_goal(id, handle->get_goal_id());
        unresolved_motion_.apply(id, non_settling("result code UNKNOWN; canceled by exact id"));
      }
      return false;
    }
    if (wrapped.result) {
      static_cast<void>(settle_or_record_terminal(
        MotionEndpoint::kRestock, "restock transfer", handle->get_goal_id(), wrapped.code,
        *wrapped.result));
    } else {
      record_unsettled_terminal(
        MotionEndpoint::kRestock, "restock transfer", handle->get_goal_id(),
        non_settling("delivered terminal without a payload"));
    }
    if (!wrapped.result) {
      transfer_release_established_ = false;
      detail = "restock action ended without a usable result";
      return false;
    }
    if (wrapped.code == rclcpp_action::ResultCode::SUCCEEDED &&
      wrapped.result->status == RestockProduct::Result::STATUS_SUCCEEDED)
    {
      // Placed and released: nothing is held (Milestone 10 §6, Card 052's held-state rule).
      transfer_release_established_ = true;
      return true;
    }
    if (wrapped.result->status == RestockProduct::Result::STATUS_NO_COMPATIBLE_PAIR) {
      // Refused before any motion: this goal neither attaches nor releases, so the held
      // state is whatever the previous terminal established — unchanged.
      detail = "no compatible pair";
      last_no_pair_detail_ = wrapped.result->detail;
      return false;
    }
    // Milestone 10 §6 rung 5 (Card 051): the typed recoverable skip — the goal ended without
    // latching an operator, and the result names the product it was transferring so the budget
    // is charged against the right one.
    if (wrapped.result->status == RestockProduct::Result::STATUS_SKIPPED_RECOVERABLE) {
      // The skip's contract releases the reservation at the safe pose: nothing is held.
      transfer_release_established_ = true;
      recoverable_skip = true;
      skipped_object_id = wrapped.result->selected_object_id;
      detail = wrapped.result->detail.empty() ?
        "recoverable skip" : wrapped.result->detail;
      return false;
    }
    // Every other terminal (canceled, operator-required, verification/planning/execution
    // failures, internal error): this goal may have attached and did not prove a release,
    // so the held-object state is unknown until a later explicit release restores it.
    transfer_release_established_ = false;
    detail = wrapped.result->detail.empty() ?
      "restock action reported a blocking status" : wrapped.result->detail;
    return false;
  }

  void sleep_between_cycles(std::chrono::milliseconds period)
  {
    const auto deadline = std::chrono::steady_clock::now() + period;
    while (keep_running() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(100ms);
      // Late admissions and results settle unresolved attempts while the loop idles; only that
      // evidence ever does, never the elapsed time.
      pump_unresolved_motion();
    }
  }

  // Card 086 stage 1 restart gate: reports PHASE_RECOVERING and returns true only once an
  // operator acknowledged (parameter restart_acknowledged=true), false when shutting down.
  [[nodiscard]] bool wait_for_restart_acknowledgment()
  {
    auto last_report = std::chrono::steady_clock::time_point{};
    while (keep_running()) {
      if (node_->get_parameter(restart_acknowledged_parameter_).as_bool()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "operator acknowledged the restart (%s=true); motion may resume",
          restart_acknowledged_parameter_.c_str());
        restart_recovery_pending_ = false;
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (last_report == std::chrono::steady_clock::time_point{} || now - last_report >= 5s) {
        last_report = now;
        publish_status(
          CampaignStatus::PHASE_RECOVERING, CampaignStatus::MODE_IDLE, false, false, nullptr,
          "motion_unresolved: the campaign restarted; whether the previous incarnation left a "
          "goal in flight is unknown. Set " + restart_acknowledged_parameter_ +
          "=true once the robot is verified settled; no goal is sent before that");
      }
      std::this_thread::sleep_for(100ms);
    }
    return false;
  }

  [[nodiscard]] std::chrono::milliseconds cycle_period_for(bool progress) const
  {
    if (progress) {
      return cycle_period_;
    }
    return std::max(cycle_period_, failed_cycle_backoff_);
  }

  // Milestone 10 §6, Card 066 (skip path): nullopt when the skip path may be taken; otherwise
  // the receipt naming the first world-state candidate the reference overview does not vouch
  // for as a feed-column front. No reference overview yet (ground truth mode, or before the
  // first survey with a front report): the coordinator's own rule is the only one, as before.
  [[nodiscard]] std::optional<std::string> skip_path_feed_refusal(
    const Measurement & measurement) const
  {
    using Observation = restocker_interfaces::msg::ObjectObservation;
    if (reference_overview_.empty() ||
      reference_overview_fronts_.size() != reference_overview_.size())
    {
      return std::nullopt;
    }
    const double radius_squared = merge_radius_m_ * merge_radius_m_;
    for (const auto & [source, entry] : measurement.skip_path_candidates) {
      const auto & [point, slot] = entry;
      if (measurement.front_deficits[slot] == 0U) {
        continue;
      }
      bool seen = false;
      bool front = false;
      for (std::size_t index = 0U; index < reference_overview_.size(); ++index) {
        const auto & candidate = reference_overview_[index];
        if (candidate.status != Observation::STATUS_OK) {
          continue;
        }
        const double dx = candidate.pose.pose.position.x - point.x;
        const double dy = candidate.pose.pose.position.y - point.y;
        const double dz = candidate.pose.pose.position.z - point.z;
        if (dx * dx + dy * dy + dz * dz > radius_squared) {
          continue;
        }
        seen = true;
        front = front || reference_overview_fronts_[index];
      }
      if (!front) {
        return "skip path refused: tray candidate " + source +
               (seen ? " stands behind a front in the latest tray overview" :
               " was not seen by the latest tray overview") +
               ", so the coordinator's world-state feed queue cannot vouch for it; surveying "
               "the tray for a front-nominated, pinned transfer";
      }
    }
    return std::nullopt;
  }

  // Milestone 10 §6, Card 066: the tray answered NO_CANDIDATE for every outstanding class, and
  // at least one answer passed over matching stock that stands behind a front it could not
  // nominate (refuted, skip-marked, or another product). That is not stock exhaustion: the
  // front is re-observed on a forced re-survey (the per-cycle refutations are cleared at the
  // next cycle's start), bounded by max_feed_order_resurveys consecutive no-progress rungs.
  [[nodiscard]] CycleOutcome feed_order_rung()
  {
    refuted_positions_.clear();
    if (feed_order_resurveys_charged_ < max_feed_order_resurveys_) {
      ++feed_order_resurveys_charged_;
      force_tray_resurvey_ = true;
      const std::string receipt =
        "feed-order NO_CANDIDATE recovery: every remaining candidate stands behind a tray "
        "product this cycle could not nominate; re-observing the column fronts, attempt " +
        std::to_string(feed_order_resurveys_charged_) + " of " +
        std::to_string(max_feed_order_resurveys_) + feed_block_suffix();
      RCLCPP_INFO(node_->get_logger(), "%s", receipt.c_str());
      // MEASURED, never BLOCKED or STOCK_EXHAUSTED, while the budget lasts: stock remains.
      publish_status(
        CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true, false,
        &*last_measurement_, receipt);
      return CycleOutcome::kBlocked;
    }
    publish_status(
      CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
      &*last_measurement_,
      "feed order: every remaining candidate stands behind a tray product the survey could "
      "not nominate; recovery exhausted at the feed-order re-survey rung within "
      "max_feed_order_resurveys=" + std::to_string(max_feed_order_resurveys_) +
      "; the cycle is blocked" + feed_block_suffix());
    return CycleOutcome::kBlocked;
  }

  [[nodiscard]] std::string feed_block_suffix() const
  {
    return last_feed_block_example_.empty() ?
           std::string() : " (" + last_feed_block_example_ + ")";
  }

  [[nodiscard]] std::string coordinator_refusal_suffix() const
  {
    return last_no_pair_detail_.empty() ?
           std::string() : " (coordinator: " + last_no_pair_detail_ + ")";
  }

  // Milestone 10 §6 rung 5 (Card 051): charge the campaign-level skip budget against the
  // product (by source id — stable across re-admission), mark it for tray exclusion, and
  // continue the loop without ever publishing PHASE_BLOCKED. When the budget runs out the
  // product is treated as gone for the run and reported — that, plus an UNSAFE operator latch
  // at the goal, is what may stop the campaign on this path.
  [[nodiscard]] CycleOutcome handle_recoverable_skip(
    std::uint64_t object_id, const std::string & detail)
  {
    std::string key = "object:" + std::to_string(object_id);
    if (object_id != 0U && last_measurement_) {
      for (const auto & [source, id] : last_measurement_->tray_candidate_object_ids) {
        if (id == object_id) {
          key = source;
          break;
        }
      }
    }
    // No-mark edge (Card 051 review): an id that is not in the boundary measurement charges the
    // synthetic object:N key and writes no position mark. Benign by construction — such a
    // product is not a candidate this cycle (the candidate map and the position map are built
    // from the same loop), so the skip path selects a different, present candidate or the
    // deficit forces a survey; selection ids always exist in the snapshot they were selected
    // against, which is why no fixture can reach this corner. The synthetic key still charges
    // the budget, so skips remain bounded.
    if (last_measurement_) {
      const auto position = last_measurement_->tray_candidate_positions.find(key);
      if (position != last_measurement_->tray_candidate_positions.end()) {
        skip_marks_[key] = position->second;
      }
    }
    const auto count = ++product_skip_counts_[key];
    std::string report = "recoverable skip: " + detail;
    if (count >= static_cast<std::size_t>(max_product_skips_)) {
      RCLCPP_WARN(
        node_->get_logger(),
        "%s — product %s skipped %zu times (max_product_skips=%" PRId64
        "); treated as gone for the run",
        report.c_str(), key.c_str(), count, max_product_skips_);
      report += "; the product is treated as gone for the run";
    } else {
      RCLCPP_WARN(
        node_->get_logger(),
        "recoverable skip: product %s skipped %zu of %" PRId64
        "; continuing with the next candidate",
        key.c_str(), count, max_product_skips_);
      report += "; skipped " + std::to_string(count) + " of " +
        std::to_string(max_product_skips_) +
        "; continuing with the next candidate";
    }
    publish_status(
      CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true,
      back_survey_complete_, last_measurement_ ? &*last_measurement_ : nullptr, report);
    // No-progress backoff, but never a blocked phase: the loop retries with the mark in force.
    return CycleOutcome::kBlocked;
  }

  // A skip mark clears when the product is re-observed farther than the merge radius from
  // where it was skipped: the evidence changed, so the revisit is allowed again (Milestone 10
  // §6 rung 5, Card 051). An absent product keeps its mark — it cannot be selected anyway.
  //
  // Decision (Card 051 review): the skip BUDGET does not reset with the mark — counts are
  // monotone for the run. Once a product is reported "gone for the run" (count >=
  // max_product_skips) its mark also stands regardless of motion: un-goning a product the
  // operator was told is gone would contradict the report, while the mark-clear rule is the
  // spec's revisit clause for products whose budget is still alive.
  void refresh_skip_marks()
  {
    if (skip_marks_.empty() || !last_measurement_) {
      return;
    }
    for (auto it = skip_marks_.begin(); it != skip_marks_.end(); ) {
      const auto found = last_measurement_->tray_candidate_positions.find(it->first);
      if (found == last_measurement_->tray_candidate_positions.end()) {
        ++it;
        continue;
      }
      const double dx = it->second.x - found->second.x;
      const double dy = it->second.y - found->second.y;
      const double dz = it->second.z - found->second.z;
      if (dx * dx + dy * dy + dz * dz >
        merge_radius_m_ * merge_radius_m_)
      {
        const auto counted = product_skip_counts_.find(it->first);
        if (counted != product_skip_counts_.end() &&
          counted->second >= static_cast<std::size_t>(max_product_skips_))
        {
          // Gone for the run: the mark stands even though the product moved; the budget is
          // never reset (review decision above).
          ++it;
          continue;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "skip mark cleared for %s: the product was re-observed elsewhere; revisiting "
          "(the skip budget is unchanged)",
          it->first.c_str());
        it = skip_marks_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // One SURVEY_SHELF pass over `needing` (in the order given), then a refresh measure. Used
  // both for the start-of-run/validity entry into the cycle and for the transfer-boundary
  // re-entry, so both charge survey arm time and both leave last_measurement_ current.
  //
  // Milestone 10 §6, Card 052: a recoverable failure climbs the campaign ladder instead of
  // publishing PHASE_BLOCKED — rung 1 retreats when the attempt may have moved the arm
  // (no-op with a receipt when nothing moved), rung 2/4 order the other lanes first, rung 3
  // retries within max_lane_survey_attempts_per_cycle, and rung 5 skips whatever never
  // observed, charging max_survey_skips. Only an UNSAFE classification, an unestablished
  // retreat, or the exhausted skip budget publishes PHASE_BLOCKED — the detail names the
  // rung and the budget.
  [[nodiscard]] ShelfOutcome survey_lanes(
    const std::vector<std::string> & needing, const bool start_of_run)
  {
    if (needing.empty()) {
      return ShelfOutcome::kOk;
    }
    front_survey_complete_ = false;
    publish_status(
      CampaignStatus::PHASE_SURVEYING_FRONT, CampaignStatus::MODE_SURVEY_SHELF, false,
      back_survey_complete_, &*last_measurement_,
      start_of_run ? "start-of-run survey of every lane" :
      "surveying lanes whose evidence is invalid, expired, or missing");
    std::vector<std::string> pending = needing;
    std::int64_t round = 0;
    while (!pending.empty() && round < max_lane_survey_attempts_) {
      if (!keep_running()) {
        return ShelfOutcome::kStopped;
      }
      ++round;
      std::vector<std::string> still_pending;
      for (const auto & lane_id : pending) {
        const LaneSurveyOutcome attempt = survey_lane(lane_id);
        if (attempt.observed) {
          continue;
        }
        SurveyFailureEvidence evidence;
        evidence.terminal_result_received = attempt.terminal_result_received;
        evidence.motion_definitely_not_started = attempt.motion_definitely_not_started;
        evidence.execution_reached_terminal_stop = attempt.execution_reached_terminal_stop;
        evidence.cause = "SURVEY_SHELF " + lane_id + ": " + attempt.detail;
        const RecoveryClassification classification = receipt_survey_failure(evidence);
        if (classification.klass == RecoveryClass::kUnsafe) {
          publish_status(
            CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false,
            back_survey_complete_, nullptr,
            "shelf survey failed; the cycle is blocked (" + classification.reason + ")");
          return ShelfOutcome::kBlocked;
        }
        // RECOVERABLE: rung 1 first — a safe survey pose before anything else is commanded.
        if (!attempt.motion_definitely_not_started) {
          if (!retreat_to_safe_survey_pose("SURVEY_SHELF " + lane_id)) {
            publish_status(
              CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false,
              back_survey_complete_, nullptr,
              "survey recovery rung 1: the cleanup retreat could not establish a verified "
              "stop; the cycle is blocked");
            return ShelfOutcome::kBlocked;
          }
        } else {
          RCLCPP_INFO(
            node_->get_logger(),
            "recovery rung 1 (safe survey pose): nothing moved this attempt; the not-started "
            "evidence stands (SURVEY_SHELF %s)",
            lane_id.c_str());
        }
        if (round < max_lane_survey_attempts_) {
          RCLCPP_INFO(
            node_->get_logger(),
            "recovery rung 3 (retry with fresh evidence): SURVEY_SHELF %s attempt %" PRId64
            " of %" PRId64 " charged; rung 2/4 orders the remaining lanes first",
            lane_id.c_str(), round, max_lane_survey_attempts_);
          still_pending.push_back(lane_id);
        } else {
          RCLCPP_INFO(
            node_->get_logger(),
            "recovery rung 3 exhausted for SURVEY_SHELF %s this cycle (attempt %" PRId64
            " of %" PRId64 "); rung 5 decides",
            lane_id.c_str(), round, max_lane_survey_attempts_);
          still_pending.push_back(lane_id);
        }
      }
      pending = std::move(still_pending);
    }
    // Rung 5: whatever never observed within the per-cycle attempts budget is skipped for
    // this cycle — or the campaign skip budget latches, naming the rung.
    const bool none_observed = pending.size() == needing.size();
    for (const auto & lane_id : pending) {
      if (!charge_survey_skip("lane " + lane_id)) {
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false,
          back_survey_complete_, nullptr, survey_skip_exhausted_detail("lane " + lane_id));
        return ShelfOutcome::kBlocked;
      }
    }
    if (none_observed) {
      // Every needing lane was skipped this cycle: this is a failed boundary for
      // progress purposes — the caller returns kBlocked WITHOUT publishing PHASE_BLOCKED,
      // so failed_cycle_backoff_ separates the revisit from the skip instead of the loop
      // hammering straight back into the same lane at full speed (Milestone 10 §6,
      // Card 052: "skip for now, revisit later").
      return ShelfOutcome::kBlocked;
    }
    std::string measurement_error;
    const auto refreshed = measure_world(measurement_error);
    if (!refreshed) {
      receipt_world_failure(measurement_error);
      publish_status(
        CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false,
        back_survey_complete_, nullptr, measurement_error);
      return ShelfOutcome::kBlocked;
    }
    set_last_measurement(*refreshed);
    return ShelfOutcome::kOk;
  }

  // True when the work cycle made progress (or parked cleanly in IDLE); false when a failed
  // boundary should charge the backoff floor before the next attempt.
  [[nodiscard]] CycleOutcome run_work_cycle()
  {
    // Refutation marks are per the spec "for this cycle": a blocked or completed cycle must
    // not carry them into the next one, or repeated refusals eventually exclude every
    // candidate and the tray reports NO_CANDIDATE for the wrong reason. Skip marks are NOT
    // per-cycle: they persist until the product's evidence changes or its budget runs out
    // (Milestone 10 §6 rung 5, Card 051).
    refuted_positions_.clear();
    refresh_skip_marks();
    // A lift armed by the previous cycle's rung is live for this cycle only (Card 058).
    active_skip_lift_class_ = std::exchange(skip_lift_class_, std::nullopt);
    bool surveyed = false;
    // Set when this cycle's SURVEY_TRAY close-confirms a candidate: the transfer goal must
    // name exactly that product (Milestone 10 §1, Card 037). Empty = skip path, no pin.
    std::optional<std::string> confirmed_source;
    {
      const bool start_of_run = !initial_shelf_survey_done_;
      const std::vector<std::string> needing =
        start_of_run ? lane_ids_ : last_measurement_->lanes_needing_survey;
      if (!needing.empty()) {
        const ShelfOutcome shelf = survey_lanes(needing, start_of_run);
        if (shelf != ShelfOutcome::kOk) {
          return shelf == ShelfOutcome::kStopped ? CycleOutcome::kStopped :
                 CycleOutcome::kBlocked;
        }
        surveyed = true;
        initial_shelf_survey_done_ = true;
      }
    }
    initial_shelf_survey_done_ = true;
    front_survey_complete_ = true;
    back_survey_complete_ = last_measurement_->has_valid_candidate_for_deficit;
    publish_status(
      CampaignStatus::PHASE_MEASURED,
      surveyed ? CampaignStatus::MODE_SURVEY_SHELF : CampaignStatus::MODE_IDLE,
      true, back_survey_complete_, &*last_measurement_,
      "section 4 deficits measured against valid or refreshed lane evidence");

    if (last_measurement_->total_front_deficit == 0U) {
      refuted_positions_.clear();
      publish_status(
        CampaignStatus::PHASE_FRONT_FULL, CampaignStatus::MODE_IDLE, true,
        back_survey_complete_, &*last_measurement_,
        "every lane is at its declared target; the loop is idle until evidence changes");
      return CycleOutcome::kIdleWait;
    }

    // Milestone 10 §6, Card 058 re-review note B (Card 069): an armed lift whose class has no
    // front deficit at this cycle's measurement lapses here, so the survey never starts at a
    // class with nothing to fill and no goal carries lifted marks for it.
    if (active_skip_lift_class_) {
      const auto slot = class_slot(*active_skip_lift_class_);
      if (!slot || last_measurement_->front_deficits[*slot] == 0U) {
        RCLCPP_INFO(
          node_->get_logger(),
          "skip-mark lift for %s lapses: the class has no front deficit this cycle",
          product_class_name(*active_skip_lift_class_));
        active_skip_lift_class_.reset();
      }
    }

    // SURVEY_TRAY is entered only when no valid admitted tray evidence names a candidate for an
    // outstanding deficit, and after any refutation the survey path is forced so reselection
    // cannot be bypassed by the very evidence the close view just refused. A skip mark forces
    // the same survey (Milestone 10 §6 rung 5, Card 051): the skipped product must not be
    // reselected on the skip path. Milestone 10 §6, Card 055: a charged premature-NCP recovery
    // also forces it — the skip path dispatched a transfer against evidence selection refused,
    // so the next attempt re-surveys the tray instead of repeating the refusal.
    // Milestone 10 §6, Card 069 (review N5d): while a retired product still looks fresh, the
    // transfer must be pinned to a close-confirmed product, so the survey path is forced.
    bool skip_tray = refuted_positions_.empty() && skip_marks_.empty() &&
      !force_tray_resurvey_ && !active_skip_lift_class_ &&
      !last_measurement_->retired_product_looks_fresh &&
      last_measurement_->has_valid_candidate_for_deficit;
    // Milestone 10 §6, Card 066: the skip path is feed-safe too. The coordinator's feed-queue
    // rule sees only world state, which in camera mode lacks every product only the overview
    // saw; so with a reference overview, the skip path needs every candidate the coordinator
    // could take vouched for as a feed-column front, or the cycle surveys (front-nominated and
    // pinned) instead.
    if (skip_tray) {
      if (const auto refusal = skip_path_feed_refusal(*last_measurement_)) {
        RCLCPP_INFO(node_->get_logger(), "%s", refusal->c_str());
        publish_status(
          CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true, back_survey_complete_,
          &*last_measurement_, *refusal);
        skip_tray = false;
      }
    }
    if (!skip_tray) {
      // The forced attempt's job was fresh tray evidence; it is spent the moment the survey
      // runs, whichever way the survey ends (confirm, refuted, no candidate, or a ladder
      // failure under Card 052's classification).
      force_tray_resurvey_ = false;
      back_survey_complete_ = false;
      publish_status(
        CampaignStatus::PHASE_SURVEYING_BACK, CampaignStatus::MODE_SURVEY_TRAY, true, false,
        &*last_measurement_,
        "a deficit lacks valid tray evidence; surveying the tray for a candidate");
      // The lifted survey starts at the class the rung charged for (Card 058 review note 1).
      std::uint8_t tray_class =
        active_skip_lift_class_.value_or(last_measurement_->largest_deficit_class);
      if (tray_class == 0U) {
        tray_class = kKnownProductClasses[0];
      }
      std::int64_t attempts = 0;
      // The first class asked this cycle whose NO_CANDIDATE a lift would have answered: the
      // rung's qualification is ORed across every class the fallthrough asks, and the lift is
      // armed for that class, not whichever class happened to be asked last.
      std::optional<std::uint8_t> skip_lift_candidate_class;
      bool confirmed = false;
      // Classes already asked this cycle. NO_CANDIDATE for the largest deficit class must not
      // park a fillable multi-class campaign: the loop tries every other outstanding deficit
      // class that still lacks a valid candidate before it idles.
      std::set<std::uint8_t> asked_classes;
      // Milestone 10 §6, Card 066: whether any survey this cycle passed over matching stock
      // because something stands ahead of it in its feed column.
      bool feed_blocked_this_cycle = false;
      last_feed_block_example_.clear();
      // Milestone 10 §6, Card 052: the tray side of the campaign ladder. The station list
      // stays empty — the server derives the nominal order — until a recoverable failure
      // rotates or skips a station. Station failures and skips are per cycle (rung 5's
      // marks are "for this cycle"); the survey-skip charge is per run.
      std::vector<std::string> tray_stations;
      bool tray_stations_dirty = false;
      std::map<std::string, std::int64_t> station_failures;
      while (keep_running() && !confirmed) {
        if (tray_stations_dirty && tray_stations.empty()) {
          // Every station is skipped this cycle (rung 5): back off WITHOUT PHASE_BLOCKED
          // and revisit next cycle — the charge persists, the per-cycle skips do not.
          publish_status(
            CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true, false,
            &*last_measurement_,
            "every tray station is skipped this cycle (survey skips " +
            std::to_string(survey_skip_charged_) + " of " + std::to_string(max_survey_skips_) +
            "); backing off to revisit next cycle");
          return CycleOutcome::kBlocked;
        }
        if (++attempts > max_tray_attempts_) {
          publish_status(
            CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
            &*last_measurement_,
            "tray survey recovery exhausted at rung 3 within max_tray_attempts_per_cycle=" +
            std::to_string(max_tray_attempts_) + "; the cycle is blocked");
          return CycleOutcome::kBlocked;
        }
        asked_classes.insert(tray_class);
        const TraySurveyOutcome tray = survey_tray(tray_class, tray_stations);
        // Unmarked or marked, matching stock behind a front is not exhaustion (Card 066,
        // review cmbrev066b N1).
        if (tray.completed &&
          (tray.feed_blocked_candidates > 0U || tray.marked_feed_blocked_candidates > 0U))
        {
          feed_blocked_this_cycle = true;
          if (!tray.feed_block_example.empty()) {
            last_feed_block_example_ = tray.feed_block_example;
          }
        }
        if (!tray.completed) {
          // No terminal result was delivered (admission timeout, rejection, a server that
          // never appeared, a cancel without an answer): the arm's state cannot be read
          // from this attempt, so the class is UNSAFE and the block stands
          // (Milestone 10 §6, Card 052).
          SurveyFailureEvidence evidence;
          evidence.cause = "SURVEY_TRAY: " + tray.detail;
          (void)receipt_survey_failure(evidence);
          publish_status(
            CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
            &*last_measurement_, "tray survey failed; the cycle is blocked: " + tray.detail);
          return keep_running() ? CycleOutcome::kBlocked : CycleOutcome::kStopped;
        }
        switch (tray.outcome) {
          case SurveyTray::Result::OUTCOME_CONFIRMED:
            confirmed = true;
            back_survey_complete_ = true;
            confirmed_source = tray.confirmed_source_object_id;
            break;
          case SurveyTray::Result::OUTCOME_REFUTED:
            if (tray.refuted_position) {
              refuted_positions_.push_back(*tray.refuted_position);
            }
            RCLCPP_WARN(
              node_->get_logger(),
              "CONFIRM refuted the tray candidate (%zu refuted this cycle); reselecting",
              refuted_positions_.size());
            break;
          case SurveyTray::Result::OUTCOME_NO_CANDIDATE: {
              // Milestone 10 §6, Card 058: classify while refuted_positions_ still holds what
              // the goal carried — would a re-survey with the live skip marks lifted have a
              // candidate to select? Computed before the clear below; consumed only once the
              // class fallthrough has found nothing else ("try others first" first).
              if (!skip_lift_candidate_class &&
                skip_lift_would_select(
                  tray.overview_candidates, tray.overview_candidate_feed_front, tray_class))
              {
                skip_lift_candidate_class = tray_class;
              }
              refuted_positions_.clear();
              std::uint8_t next_class = 0U;
              for (const std::uint8_t product_class : kKnownProductClasses) {
                if (asked_classes.contains(product_class)) {
                  continue;
                }
                // Every outstanding deficit class, including one whose valid candidate is
                // held back by a skip mark or refutation: this survey was forced, so a
                // valid candidate elsewhere is exactly what the fallthrough must ask for
                // (Card 058 review note 1).
                const auto slot = class_slot(product_class);
                if (slot && last_measurement_->front_deficits[*slot] > 0U) {
                  next_class = product_class;
                  break;
                }
              }
              if (next_class != 0U) {
                RCLCPP_INFO(
                  node_->get_logger(),
                  "tray has no %s candidate; trying outstanding deficit class %s instead of "
                  "parking",
                  product_class_name(tray_class), product_class_name(next_class));
                tray_class = next_class;
                break;
              }
              // Card 066 × Card 058 (integration): the lift comes first. Only it addresses a
              // skipped front, and the feed rung keeps skip marks, so running the feed rung first
              // would spend its budget while the right recovery (retry the skipped front) never
              // ran. skip_lift_would_select already counts only a product that is a feed-column
              // front once lifted.
              if (skip_lift_candidate_class) {
                // A recoverable boundary, not a terminal state: the only class-matching
                // candidates are excluded by skip marks that still have budget, so the rung
                // re-surveys with those marks lifted and retries the skipped product
                // (Milestone 10 §6, Card 058). Never PHASE_BLOCKED on a charged attempt.
                if (skip_resurveys_charged_ < max_skip_resurveys_) {
                  ++skip_resurveys_charged_;
                  skip_lift_class_ = skip_lift_candidate_class;
                  const std::string receipt =
                    "skip-mark NO_CANDIDATE recovery: forced tray re-survey with skip marks "
                    "lifted for " + std::string(product_class_name(*skip_lift_candidate_class)) +
                    ", attempt " + std::to_string(skip_resurveys_charged_) + " of " +
                    std::to_string(max_skip_resurveys_) +
                    " — every class-matching overview candidate is excluded by active skip "
                    "marks; retrying the skipped product";
                  RCLCPP_INFO(node_->get_logger(), "%s", receipt.c_str());
                  publish_status(
                    CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true,
                    back_survey_complete_, &*last_measurement_, receipt);
                  return CycleOutcome::kBlocked;
                }
                if (feed_blocked_this_cycle) {
                  // The lift is spent, but stock still stands behind a front: that is the feed
                  // rung's boundary, never stock exhaustion (Milestone 10 §6, Card 066).
                  return feed_order_rung();
                }
                publish_status(
                  CampaignStatus::PHASE_STOCK_EXHAUSTED, CampaignStatus::MODE_IDLE, true, true,
                  &*last_measurement_,
                  "the tray survey found no candidate for any outstanding deficit class; "
                  "recovery exhausted at the skip-mark lift rung within max_skip_resurveys=" +
                  std::to_string(max_skip_resurveys_) +
                  station_attribution(tray.overview_station_reports) +
                  "; the loop is idle until evidence changes");
                return CycleOutcome::kIdleWait;
              }
              if (feed_blocked_this_cycle) {
                return feed_order_rung();
              }
              publish_status(
                CampaignStatus::PHASE_STOCK_EXHAUSTED, CampaignStatus::MODE_IDLE, true, true,
                &*last_measurement_,
                "the tray survey found no candidate for any outstanding deficit class" +
                station_attribution(tray.overview_station_reports) +
                "; the loop is idle until evidence changes");
              return CycleOutcome::kIdleWait;
            }
          default: {
              // A completed result that still blocks (UNAVAILABLE, VIEWPOINT_UNREACHED,
              // ACQUISITION_FAILED, CANCELED, INVALID_REQUEST): classified from its own
              // evidence flags (Milestone 10 §6, Card 052). UNSAFE latches as before; a
              // RECOVERABLE class climbs the tray ladder instead — rung 1 retreats when
              // the attempt may have moved the arm, rungs 2/4 rotate or skip the failed
              // station, and rung 3 retries inside max_tray_attempts_per_cycle (the loop
              // itself); only UNSAFE or an exhausted budget publishes PHASE_BLOCKED.
              SurveyFailureEvidence evidence;
              evidence.terminal_result_received = true;
              evidence.motion_definitely_not_started = tray.motion_definitely_not_started;
              evidence.execution_reached_terminal_stop = tray.execution_reached_terminal_stop;
              evidence.cause = "SURVEY_TRAY (" + tray.failed_station + "): " + tray.detail;
              const RecoveryClassification classification = receipt_survey_failure(evidence);
              if (classification.klass == RecoveryClass::kUnsafe) {
                publish_status(
                  CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
                  &*last_measurement_,
                  "tray survey reported a blocking outcome (" + classification.reason +
                  "): " + tray.detail);
                return CycleOutcome::kBlocked;
              }
              if (!tray.motion_definitely_not_started) {
                if (!retreat_to_safe_survey_pose("SURVEY_TRAY (" + tray.failed_station + ")")) {
                  publish_status(
                    CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
                    &*last_measurement_,
                    "survey recovery rung 1: the cleanup retreat could not establish a "
                    "verified stop; the cycle is blocked");
                  return CycleOutcome::kBlocked;
                }
              } else {
                RCLCPP_INFO(
                  node_->get_logger(),
                  "recovery rung 1 (safe survey pose): nothing moved this attempt; the "
                  "not-started evidence stands (SURVEY_TRAY %s)",
                  tray.failed_station.c_str());
              }
              if (!tray.failed_station.empty()) {
                if (tray_stations.empty() && !tray_stations_dirty) {
                  // First touch: adopt the nominal list so rotation and skip are visible
                  // and the server keeps answering with the same stations we track.
                  tray_stations = tray_station_names_;
                }
                const auto found = std::find(
                  tray_stations.begin(), tray_stations.end(), tray.failed_station);
                if (found != tray_stations.end()) {
                  const std::int64_t failures = ++station_failures[tray.failed_station];
                  if (failures >= max_lane_survey_attempts_) {
                    // Rung 5: this station failed often enough within the cycle — skip it
                    // for the rest of the cycle, charged to the per-run skip budget.
                    if (!charge_survey_skip("station " + tray.failed_station)) {
                      publish_status(
                        CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true, false,
                        &*last_measurement_,
                        survey_skip_exhausted_detail("station " + tray.failed_station));
                      return CycleOutcome::kBlocked;
                    }
                    tray_stations.erase(found);
                  } else {
                    // Rung 2/4: rotate the failed station to the end so the next attempt
                    // re-surveys ANOTHER station first instead of repeating this pose.
                    std::rotate(found, found + 1, tray_stations.end());
                    RCLCPP_INFO(
                      node_->get_logger(),
                      "recovery rung 2 (another station first): %s rotated to the end of "
                      "the overview list (failure %" PRId64 " of %" PRId64 ")",
                      tray.failed_station.c_str(), failures, max_lane_survey_attempts_);
                  }
                  tray_stations_dirty = true;
                }
              }
              RCLCPP_INFO(
                node_->get_logger(),
                "recovery rung 3 (retry with fresh evidence): SURVEY_TRAY attempt %" PRId64
                " of %" PRId64 " within max_tray_attempts_per_cycle",
                attempts, max_tray_attempts_);
              break;
            }
        }
      }
      if (!confirmed) {
        return keep_running() ? CycleOutcome::kBlocked : CycleOutcome::kStopped;
      }
    }

    // Fresh evidence authorises doing it. The tray step can outlast the lanes' validity
    // horizon — a measured cold-start sweep plus one coordinated confirm ran well past 60 s —
    // so the loop re-reads the snapshot at the transfer boundary and re-enters SURVEY_SHELF
    // for anything that passed its horizon before the goal is emitted. The coordinator
    // refuses stale destinations outright; the loop must not offer them.
    std::string boundary_error;
    const auto boundary = measure_world(boundary_error);
    if (!boundary) {
      receipt_world_failure(boundary_error);
      publish_status(
        CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
        back_survey_complete_, nullptr, boundary_error);
      return CycleOutcome::kBlocked;
    }
    set_last_measurement(*boundary);
    if (boundary->any_lane_needs_survey) {
      RCLCPP_WARN(
        node_->get_logger(),
        "lane evidence passed its validity horizon before the transfer; re-entering "
        "SURVEY_SHELF for %zu lane(s)",
        boundary->lanes_needing_survey.size());
      const ShelfOutcome shelf = survey_lanes(boundary->lanes_needing_survey, false);
      if (shelf != ShelfOutcome::kOk) {
        return shelf == ShelfOutcome::kStopped ? CycleOutcome::kStopped :
               CycleOutcome::kBlocked;
      }
    }
    front_survey_complete_ = true;

    // Milestone 10 §1 confirmed-identity contract (Card 037): resolve this cycle's
    // confirmation to the world-state object the goal names. Unresolvable or defective
    // confirmations block the cycle — never fall back to an unpinned goal after a
    // confirmation, which would re-open the confirm-vs-selection mismatch.
    std::optional<restocker_world_state::ObjectId> confirmed_object;
    if (confirmed_source) {
      if (confirmed_source->empty()) {
        receipt_identity_failure(
          "the tray confirmation carried no source identity");
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
          back_survey_complete_, &*last_measurement_,
          "the tray confirmation carried no source identity; the cycle is blocked rather than "
          "transferring an unconfirmed product");
        return CycleOutcome::kBlocked;
      }
      const auto pinned = last_measurement_->tray_candidate_object_ids.find(*confirmed_source);
      if (pinned == last_measurement_->tray_candidate_object_ids.end()) {
        // Milestone 10 §6, the product-fell case (Card 051): the confirmed product is absent
        // from the boundary snapshot — it fell, was knocked away, or was never admitted. That
        // is a positive absence, not an error state: mark it gone (it cannot be selected while
        // absent) and continue with the next candidate instead of blocking the cycle. A
        // confirmation that carried no source identity at all stays blocked — a defective
        // confirm is not a fallen product.
        RCLCPP_WARN(
          node_->get_logger(),
          "the close-confirmed product '%s' fell or left the tray: absent from the world-state "
          "snapshot at the transfer boundary; marking it gone and continuing with the next "
          "candidate",
          confirmed_source->c_str());
        publish_status(
          CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true,
          back_survey_complete_, &*last_measurement_,
          "the confirmed product is gone from the tray; continuing with the next candidate");
        return CycleOutcome::kBlocked;
      }
      confirmed_object = restocker_world_state::ObjectId{pinned->second};
    } else if (!skip_tray) {
      // Unreachable: !skip_tray implies the tray loop confirmed above. Kept fail-closed.
      receipt_identity_failure("a confirming cycle produced no confirmed identity to pin");
      publish_status(
        CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
        back_survey_complete_, &*last_measurement_,
        "a confirming cycle produced no confirmed identity to pin; the cycle is blocked");
      return CycleOutcome::kBlocked;
    }

    publish_status(
      CampaignStatus::PHASE_RESTOCKING, CampaignStatus::MODE_TRANSFER, true,
      back_survey_complete_, &*last_measurement_,
      skip_tray ? "transferring against a valid admitted candidate" :
      "transferring the close-confirmed candidate (object " +
      std::to_string(confirmed_object->value) + ")");
    std::string transfer_detail;
    bool recoverable_skip = false;
    std::uint64_t skipped_object_id = 0U;
    if (request_transfer(
        transfer_detail, confirmed_object, recoverable_skip, skipped_object_id))
    {
      ++successful_transfers_;
      feed_order_resurveys_charged_ = 0;
      refuted_positions_.clear();
      publish_status(
        CampaignStatus::PHASE_RESTOCKING, CampaignStatus::MODE_TRANSFER, true,
        back_survey_complete_, &*last_measurement_, "transfer succeeded");
      return CycleOutcome::kContinue;
    }
    if (recoverable_skip) {
      // Milestone 10 §6 rung 5 (Card 051): the typed skip charges the budget and continues —
      // PHASE_BLOCKED is never published for it; only an UNSAFE latch or an exhausted skip
      // budget stops the loop.
      return handle_recoverable_skip(skipped_object_id, transfer_detail);
    }
    if (transfer_detail == "no compatible pair") {
      std::string measurement_error;
      const auto terminal = measure_world(measurement_error);
      if (!terminal) {
        receipt_world_failure(measurement_error);
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
          back_survey_complete_, nullptr, measurement_error);
        return CycleOutcome::kBlocked;
      }
      set_last_measurement(*terminal);
      if (terminal->total_front_deficit == 0U) {
        refuted_positions_.clear();
        publish_status(
          CampaignStatus::PHASE_FRONT_FULL, CampaignStatus::MODE_IDLE, true,
          back_survey_complete_, &*last_measurement_,
          "coordinator reported no compatible pair because every front lane is full");
        return CycleOutcome::kIdleWait;
      }
      const bool stock_remains = std::any_of(
        kKnownProductClasses.begin(), kKnownProductClasses.end(),
        [this](std::uint8_t product_class) {
          const auto slot = class_slot(product_class);
          return last_measurement_->front_deficits[*slot] > 0U &&
                 last_measurement_->back_stock[*slot] > 0U;
        });
      if (stock_remains) {
        // Milestone 10 §6, Card 055: a recoverable boundary, not a terminal state. The sweep's
        // refusal is correct for the evidence it holds; the campaign's answer is to force one
        // bounded tray re-survey so close-confirm refreshes the very poses selection refused.
        // Nothing moved on a NO_COMPATIBLE_PAIR (the coordinator refuses before motion), so the
        // next survey is commanded exactly as any other cycle's. Only an exhausted budget
        // publishes PHASE_BLOCKED — naming the rung and the budget — and a genuinely exhausted
        // state never reaches here (stock_remains is false above → PHASE_STOCK_EXHAUSTED).
        if (ncp_resurveys_charged_ < max_ncp_resurveys_) {
          ++ncp_resurveys_charged_;
          force_tray_resurvey_ = true;
          const std::string receipt =
            "premature NO_COMPATIBLE_PAIR recovery: forced tray re-survey attempt " +
            std::to_string(ncp_resurveys_charged_) + " of " +
            std::to_string(max_ncp_resurveys_) +
            " — a compatible front deficit and back product remain; re-confirming before the "
            "next transfer" + coordinator_refusal_suffix();
          RCLCPP_INFO(node_->get_logger(), "%s", receipt.c_str());
          // MEASURED with the receipt, never PHASE_BLOCKED: the acceptance fixture fails the
          // run on the first blocked status, and this boundary is recoverable until its budget
          // is spent. kBlocked charges the no-progress backoff before the forced survey cycle.
          publish_status(
            CampaignStatus::PHASE_MEASURED, CampaignStatus::MODE_IDLE, true,
            back_survey_complete_, &*last_measurement_, receipt);
          return CycleOutcome::kBlocked;
        }
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
          back_survey_complete_, &*last_measurement_,
          "premature NO_COMPATIBLE_PAIR: a compatible front deficit and back product remain; "
          "recovery exhausted at the forced tray re-survey rung within max_ncp_resurveys=" +
          std::to_string(max_ncp_resurveys_) + "; the cycle is blocked" +
          coordinator_refusal_suffix());
        return CycleOutcome::kBlocked;
      }
      refuted_positions_.clear();
      publish_status(
        CampaignStatus::PHASE_STOCK_EXHAUSTED, CampaignStatus::MODE_IDLE, true,
        back_survey_complete_, &*last_measurement_,
        "coordinator reported no compatible pair before the front was full because compatible "
        "back stock was exhausted");
      return CycleOutcome::kIdleWait;
    }
    publish_status(
      CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, true,
      back_survey_complete_, &*last_measurement_, transfer_detail);
    RCLCPP_WARN(node_->get_logger(), "transfer blocked: %s", transfer_detail.c_str());
    return keep_running() ? CycleOutcome::kBlocked : CycleOutcome::kStopped;
  }

  void run()
  {
    publish_status(
      CampaignStatus::PHASE_WAITING_FOR_COORDINATOR, CampaignStatus::MODE_IDLE, false, false,
      nullptr, "waiting for coordinator admission before the first mode-loop cycle");
    RCLCPP_INFO(node_->get_logger(), "waiting for coordinator admission before the mode loop");
    if (!wait_for_coordinator_ready()) {
      return;
    }
    if (!wait_for_restart_acknowledgment()) {
      return;
    }
    while (keep_running() && (max_cycles_ == 0 || cycle_ < max_cycles_)) {
      ++cycle_;
      // Card 086 stage 1: an unresolved attempt blocks every cycle. Backoff, fresh observations
      // and the cycle bound do not clear it; only exact-identity settlement does.
      pump_unresolved_motion();
      if (unresolved_motion_.unresolved()) {
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, front_survey_complete_,
          back_survey_complete_, last_measurement_ ? &*last_measurement_ : nullptr,
          "no goal is sent while motion is unresolved");
        sleep_between_cycles(cycle_period_for(false));
        continue;
      }
      if (waiting_for_evidence_) {
        // IDLE: wake for the evidence events that re-enter the work cycle — a lane that
        // needs a survey, or a deficit that has gained a valid tray candidate while parked
        // (for example a confirm observation admitted after the park). Zero deficit with
        // no new candidate stays asleep.
        std::string measurement_error;
        const auto snapshot = measure_world(measurement_error);
        if (!snapshot) {
          receipt_world_failure(measurement_error);
          publish_status(
            CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false, false, nullptr,
            measurement_error);
          sleep_between_cycles(cycle_period_for(false));
          continue;
        }
        set_last_measurement(*snapshot);
        const bool fresh_candidate_for_deficit =
          snapshot->total_front_deficit > 0U && snapshot->has_valid_candidate_for_deficit;
        if (snapshot->any_lane_needs_survey || fresh_candidate_for_deficit) {
          waiting_for_evidence_ = false;
        } else {
          sleep_between_cycles(std::max(cycle_period_, failed_cycle_backoff_));
          continue;
        }
      }
      std::string measurement_error;
      const auto measured = measure_world(measurement_error);
      if (!measured) {
        receipt_world_failure(measurement_error);
        publish_status(
          CampaignStatus::PHASE_BLOCKED, CampaignStatus::MODE_IDLE, false, false, nullptr,
          measurement_error);
        sleep_between_cycles(cycle_period_for(false));
        continue;
      }
      set_last_measurement(*measured);
      RCLCPP_INFO(
        node_->get_logger(), "autonomous mode-loop cycle %" PRId64 " starting (deficit %u)",
        cycle_, measured->total_front_deficit);
      const CycleOutcome outcome = run_work_cycle();
      if (outcome == CycleOutcome::kStopped || !keep_running()) {
        break;
      }
      const bool progress = outcome != CycleOutcome::kBlocked;
      if (outcome == CycleOutcome::kIdleWait) {
        waiting_for_evidence_ = true;
      }
      if (max_cycles_ == 0 || cycle_ < max_cycles_) {
        sleep_between_cycles(cycle_period_for(progress));
      }
    }
    if (keep_running()) {
      const auto final_measurement = measurement_snapshot();
      publish_status(
        CampaignStatus::PHASE_COMPLETE, CampaignStatus::MODE_IDLE,
        front_survey_complete_,
        back_survey_complete_, final_measurement ? &*final_measurement : nullptr,
        unresolved_motion_.unresolved() ?
        "campaign reached configured cycle bound with unresolved motion" :
        "campaign reached configured cycle bound");
      RCLCPP_INFO(
        node_->get_logger(), "autonomous campaign reached max_cycles=%" PRId64,
        max_cycles_);
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<RestockProduct>::SharedPtr restock_client_;
  rclcpp_action::Client<SurveyLane>::SharedPtr shelf_survey_client_;
  rclcpp_action::Client<SurveyTray>::SharedPtr tray_survey_client_;
  rclcpp_action::Client<SurveyViewpoint>::SharedPtr viewpoint_client_;
  GoalCancelClient::SharedPtr restock_cancel_client_;
  GoalCancelClient::SharedPtr shelf_survey_cancel_client_;
  GoalCancelClient::SharedPtr tray_survey_cancel_client_;
  GoalCancelClient::SharedPtr viewpoint_cancel_client_;
  rclcpp::Client<GetWorldState>::SharedPtr world_state_client_;
  rclcpp::Publisher<CampaignStatus>::SharedPtr campaign_status_publisher_;
  rclcpp::Subscription<restocker_interfaces::msg::RestockCoordinatorStatus>::SharedPtr
    coordinator_status_subscription_;
  std::string restock_action_name_;
  std::string shelf_survey_action_name_;
  std::string tray_survey_action_name_;
  std::string coordinator_status_topic_;
  std::string world_state_service_name_;
  std::string campaign_status_topic_;
  std::string recovery_viewpoint_action_name_;
  std::string recovery_safe_station_;
  std::vector<std::string> lane_ids_;
  std::vector<std::string> tray_station_names_;
  std::int64_t max_cycles_{0};
  std::chrono::milliseconds cycle_period_{0};
  std::chrono::milliseconds failed_cycle_backoff_{0};
  std::chrono::milliseconds server_wait_timeout_{0};
  std::chrono::milliseconds action_timeout_{0};
  std::chrono::nanoseconds lane_evidence_validity_{std::chrono::seconds(60)};
  std::chrono::nanoseconds tray_evidence_validity_{std::chrono::seconds(120)};
  std::int64_t max_tray_attempts_{8};
  // Milestone 10 §6 rung 3/5 (Card 052): the per-cycle lane attempt bound and the
  // per-run campaign survey-skip budget; survey_skip_charged_ persists across cycles.
  std::int64_t max_lane_survey_attempts_{2};
  std::int64_t max_survey_skips_{2};
  std::int64_t survey_skip_charged_{0};
  // Milestone 10 §6 rung 5 (Card 051): the campaign-level skip budget and the marks it keeps,
  // keyed by perception source id (stable across re-admission, unlike the object id).
  std::int64_t max_product_skips_{2};
  std::map<std::string, std::size_t> product_skip_counts_;
  std::map<std::string, geometry_msgs::msg::Point> skip_marks_;
  // Milestone 10 §6, Card 055: the premature-NO_COMPATIBLE_PAIR recovery budget and the flag
  // that suspends the skip path for one attempt so the forced tray re-survey runs. The charge
  // is monotone per run; the flag clears when the tray survey runs (its job was to obtain
  // fresh tray evidence, whichever way that ends).
  std::int64_t max_ncp_resurveys_{2};
  std::int64_t ncp_resurveys_charged_{0};
  bool force_tray_resurvey_{false};
  // The coordinator's own detail for the last STATUS_NO_COMPATIBLE_PAIR, so Card 055's receipts
  // name the refusal (a feed-queue refusal reads as one; Milestone 10 §6, Card 066).
  std::string last_no_pair_detail_;
  // Milestone 10 §6, Card 066: the feed-order rung's budget, reset by a successful transfer.
  std::int64_t max_feed_order_resurveys_{2};
  std::int64_t feed_order_resurveys_charged_{0};
  // The latest server example of a matching product standing behind a front (review
  // cmbrev066b N1): the feed rung's receipts and its block name it.
  std::string last_feed_block_example_;
  // The server's reported candidate-merge radius (review cmbrev066b N2); the 0.04 m default until
  // a survey reports one.
  double merge_radius_m_{kSkipMarkClearRadiusM};
  // Milestone 10 §6, Card 066 (skip path): the reference overview and its per-candidate
  // feed-column front report. Empty until a survey with a front report has completed.
  std::vector<restocker_interfaces::msg::ObjectObservation> reference_overview_;
  std::vector<bool> reference_overview_fronts_;
  // Milestone 10 §6, Card 058: the skip-mark lift rung. Charged when OUTCOME_NO_CANDIDATE
  // excluded every class-matching candidate only by live skip marks; the flag makes the next
  // SURVEY_TRAY goal carry only the gone-for-the-run marks.
  std::int64_t max_skip_resurveys_{2};
  std::int64_t skip_resurveys_charged_{0};
  // The lift rung is tied to the class that qualified (Card 058 review note 1): the rung arms
  // skip_lift_class_ for the next cycle; the cycle takes it into active_skip_lift_class_ at
  // its start, so an armed lift never outlives the one cycle it was charged for, and only a
  // survey goal of that class carries the lifted marks.
  std::optional<std::uint8_t> skip_lift_class_;
  // Milestone 10 §6, Card 069: observed-absence streaks and the products they retired, by
  // source id; the retirement holds the observation instant it was decided against.
  struct AbsenceStreak
  {
    std::size_t count{0U};
    std::int64_t observation_ns{0};
    // The work cycle and instant of the last counted view (independence, review N3).
    std::int64_t cycle{-1};
    std::chrono::steady_clock::time_point counted_at{};
  };
  std::int64_t absence_confirmations_{2};
  std::chrono::duration<double> absence_min_separation_{30.0};
  std::chrono::milliseconds coordinator_object_max_age_{180000};
  std::map<std::string, AbsenceStreak> absence_streaks_;
  std::map<std::string, std::int64_t> retired_sources_;
  std::optional<std::uint8_t> active_skip_lift_class_;
  ProductCollisionCatalog product_catalog_;
  WorkcellManipulationGeometry manipulation_geometry_;
  // Campaign state below has one owner: worker_. Executor callbacks only update the
  // request-local confirm inbox or the explicitly atomic readiness/stopping flags.
  std::optional<Measurement> last_measurement_;
  std::vector<geometry_msgs::msg::Point> refuted_positions_;
  std::uint64_t status_sequence_{0U};
  std::int64_t cycle_{0};
  std::uint64_t successful_transfers_{0U};
  double survey_arm_time_sec_{0.0};
  double transfer_arm_time_sec_{0.0};
  bool front_survey_complete_{false};
  bool back_survey_complete_{false};
  bool initial_shelf_survey_done_{false};
  bool waiting_for_evidence_{false};
  // Milestone 10 §6, Card 052: the held-object half of the survey-failure classification.
  // True from a cold start (no transfer goal has ever been accepted, so nothing attachable
  // happened), set true again only by a terminal that proves a release (STATUS_SUCCEEDED or
  // STATUS_SKIPPED_RECOVERABLE), and set false by any other accepted transfer's terminal or
  // an unanswered goal — unknown held state, fail closed.
  bool transfer_release_established_{true};
  // Card 086 stage 1 (CMB-SPEC-13): the attempt registry (worker-thread owned), the pollers that
  // route late admissions and results to its records, and the restart gate.
  UnresolvedMotionRegistry unresolved_motion_;
  std::map<UnresolvedMotionRegistry::AttemptId, std::function<void()>> late_attempt_pollers_;
  std::string restart_acknowledged_parameter_;
  bool restart_recovery_pending_{true};
  std::atomic_bool stopping_{false};
  std::atomic_bool coordinator_ready_{false};
  std::thread worker_;
};

}  // namespace
}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  int status = 0;
  try {
    auto campaign = std::make_shared<restocker_task_executor::AutonomousRestockCampaign>(options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(campaign->node());
    campaign->start();
    executor.spin();
    campaign->stop();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      rclcpp::get_logger("autonomous_restock_campaign"), "campaign node failed: %s",
      error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
