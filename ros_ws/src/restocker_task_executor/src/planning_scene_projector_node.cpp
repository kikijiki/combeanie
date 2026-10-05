// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <exception>
#include <filesystem>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/srv/apply_planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/obstacle_observation.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/srv/acquire_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/get_planning_scene_projection_status.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/release_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/validate_planning_scene_lease.hpp>
#include <tf2/time.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/obstacle_projection.hpp"
#include "restocker_task_executor/planning_scene_attachment.hpp"
#include "restocker_task_executor/planning_scene_lease.hpp"
#include "restocker_task_executor/planning_scene_lease_ros.hpp"
#include "restocker_task_executor/planning_scene_reconciliation.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_task_executor/world_snapshot_projection.hpp"

namespace restocker_task_executor
{

class PlanningSceneProjectorNode final : public rclcpp::Node
{
public:
  PlanningSceneProjectorNode()
  : Node("planning_scene_projector"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    planning_frame_ = declare_parameter<std::string>("planning_frame", "world");
    workcell_root_frame_ = declare_parameter<std::string>("workcell_root_frame", "shelf");
    workcell_description_ = declare_parameter<std::string>("workcell_description", "");
    const auto catalog_path = declare_parameter<std::string>("product_catalog_path", "");
    // Card 050: the scenario document whose products are spawned into the cell. Nonempty seeds
    // every declared product into the planning scene until world state tracks it (spec §6); a
    // deployment that does not spawn the document's products must leave this empty.
    scenario_path_ = declare_parameter<std::string>("scenario_path", "");
    snapshot_service_ = declare_parameter<std::string>(
      "snapshot_service", "/world_state/get_snapshot");
    get_scene_service_ = declare_parameter<std::string>(
      "get_scene_service", "/get_planning_scene");
    apply_scene_service_ = declare_parameter<std::string>(
      "apply_scene_service", "/apply_planning_scene");
    reconcile_period_sec_ = declare_parameter<double>("reconcile_period_sec", 0.5);
    service_timeout_sec_ = declare_parameter<double>("service_timeout_sec", 2.0);
    // A round trip at or beyond this fraction of the deadline is logged at INFO (throttled), so
    // an ordinary run carries the tail of the service-latency distribution instead of only the
    // trips that happened to miss the deadline. Defaults to half the deadline.
    slow_stage_report_sec_ = declare_parameter<double>(
      "slow_stage_report_sec", 0.5 * service_timeout_sec_);
    // Total wait for one read-only service request before it is cancelled and re-issued. A
    // read-only request that overruns service_timeout_sec is held rather than abandoned: a
    // service that consistently answers slower than the deadline can never be served if every
    // attempt is cancelled before its answer lands (Milestone 10 §5, "A slow dependency is not a
    // dead dependency"). Kept below scene_authority_settle_timeout (5 s) so a dead service is
    // still refused inside that window, with the rest of the cycle inside it too.
    service_total_timeout_sec_ = declare_parameter<double>("service_total_timeout_sec", 4.0);
    // Product evidence older than this is reported as aged. It is not a freshness gate: an aged
    // product stays in the scene at its last known pose (Milestone 10 §6, Card 071).
    max_observation_age_sec_ = declare_parameter<double>("max_observation_age_sec", 0.5);
    max_future_skew_sec_ = declare_parameter<double>("max_future_skew_sec", 0.05);
    // Robot link a held product is carried on, and the links MoveIt must not collision-check it
    // against while held (the jaws close on the product).
    attachment_parent_link_ = declare_parameter<std::string>("attachment_parent_link", "gripper");
    attachment_touch_links_ = declare_parameter<std::vector<std::string>>(
      "attachment_touch_links",
      std::vector<std::string>{"gripper", "left_finger", "right_finger"});
    // Frame in which world state records a held product's pose; both halves of the carried-product
    // projection are composed from it.
    grasp_center_frame_ = declare_parameter<std::string>("grasp_center_frame", "grasp_center");
    // A stocked product touches its support exactly (seated at tray_top + height/2; see also
    // floor_settle_tolerance_m in workcell_geometry.yaml). As robot geometry that contact reads as
    // a start-state collision, so the attached representation gives back this clearance at each end
    // of the product axis. MoveIt's default_attached_padding has no effect on attached bodies,
    // hence the allowance is applied to the geometry itself.
    attachment_settle_clearance_m_ =
      declare_parameter<double>("attachment_settle_clearance_m", 0.002);
    // The shelf survey: a product inside a lane is drawn as the lane's occupied volume (surveyed
    // cross-section over the depth its evidence says is full).
    const auto workcell_geometry_path =
      declare_parameter<std::string>("workcell_geometry_path", "");
    if (workcell_geometry_path.empty()) {
      throw std::invalid_argument("workcell_geometry_path is required");
    }
    auto lane_geometry = load_manipulation_geometry(
      std::filesystem::path(workcell_geometry_path));
    if (!lane_geometry) {
      throw std::invalid_argument(
              "invalid workcell manipulation geometry: " + lane_geometry.error().detail);
    }
    lane_geometry_ = std::move(lane_geometry.value().lanes);
    obstacle_topic_ = declare_parameter<std::string>(
      "obstacle_topic", "/perception/obstacle_observations");
    // Max age of the newest obstacle observation before the projection stops being certified.
    // Generous relative to the ~5 Hz depth stream: one dropped frame is not evidence loss.
    obstacle_max_age_sec_ = declare_parameter<double>("obstacle_max_age_sec", 2.0);
    // When true, obstacle evidence is required from startup. When false, only once the first
    // observation has arrived; afterwards silence is evidence loss. The depth launch sets this
    // true.
    require_obstacle_evidence_ = declare_parameter<bool>("require_obstacle_evidence", false);
    obstacle_config_.obstacle_padding_m =
      declare_parameter<double>("obstacle_padding_m", 0.03);
    obstacle_config_.min_extent_m = declare_parameter<double>("obstacle_min_extent_m", 0.04);
    obstacle_config_.max_extent_m = declare_parameter<double>("obstacle_max_extent_m", 1.50);
    obstacle_config_.max_obstacles =
      static_cast<std::size_t>(declare_parameter<std::int64_t>("max_obstacles", 8));
    obstacle_config_.known_padding_m =
      declare_parameter<double>("obstacle_known_padding_m", 0.05);
    obstacle_config_.known_containment_fraction =
      declare_parameter<double>("obstacle_known_containment_fraction", 0.75);
    obstacle_config_.change_position_tolerance_m =
      declare_parameter<double>("obstacle_change_position_tolerance_m", 0.03);
    obstacle_config_.change_size_tolerance_m =
      declare_parameter<double>("obstacle_change_size_tolerance_m", 0.03);
    obstacle_config_.planning_frame = planning_frame_;
    const auto operation_journal_capacity = declare_parameter<std::int64_t>(
      "lease_operation_journal_capacity", 4096);
    if (operation_journal_capacity < 2) {
      throw std::invalid_argument("lease_operation_journal_capacity must be at least two");
    }
    PlanningSceneLeaseConfig lease_config;
    lease_config.operation_journal_capacity =
      static_cast<std::size_t>(operation_journal_capacity);
    lease_protocol_ =
      std::make_unique<PlanningSceneLeaseProtocol>(std::move(lease_config));
    verification_config_.dimension_tolerance = declare_parameter<double>(
      "dimension_tolerance", 1.0e-8);
    verification_config_.position_tolerance = declare_parameter<double>(
      "position_tolerance", 1.0e-8);
    verification_config_.orientation_tolerance_rad = declare_parameter<double>(
      "orientation_tolerance_rad", 1.0e-8);
    // Deadband on deciding to write a newer product pose (verification stays exact). Measured
    // contact-solver noise is 1 to 9 um per reconcile period, under 0.1 mm over 30 s; 0.5 mm is
    // well inside the 2 mm runtime position agreement and the 2 mm settle clearance. Zero rewrites
    // the scene on every observation.
    product_change_tolerance_.position_tolerance = declare_parameter<double>(
      "product_change_position_tolerance_m", 5.0e-4);
    product_change_tolerance_.orientation_tolerance_rad = declare_parameter<double>(
      "product_change_orientation_tolerance_rad", 1.0e-3);
    // Dimensions and identity have no deadband: a changed shape is a different projection.
    product_change_tolerance_.dimension_tolerance = verification_config_.dimension_tolerance;

    validate_configuration(catalog_path);
    auto catalog = ProductCollisionCatalog::load(std::filesystem::path(catalog_path));
    if (!catalog) {
      throw std::invalid_argument("invalid product collision catalog: " + catalog.error().detail);
    }
    catalog_ = std::make_unique<ProductCollisionCatalog>(std::move(catalog.value()));

    // Card 050 (spec §6): build the declared-product seeds once. A scenario product the snapshot
    // does not carry is still physically in the cell, and until this seed existed the scene knew
    // nothing of it — a transfer's free-space segments could be planned through it, tipping it,
    // after which the wrist overview backend's admission bands make it invisible to every later
    // survey. A load or catalog mismatch fails startup rather than seeding a wrong obstacle.
    if (!scenario_path_.empty()) {
      auto declared = load_scenario_products(std::filesystem::path(scenario_path_));
      if (!declared) {
        throw std::invalid_argument(
                "invalid scenario products for declared scene seeds: " +
                declared.error().detail);
      }
      for (const auto & product : declared.value()) {
        const auto geometry = catalog_->entries().find(product.geometry_key);
        if (geometry == catalog_->entries().end()) {
          throw std::invalid_argument(
                  "scenario product " + product.source_object_id +
                  " names a geometry_key absent from the product collision catalog: " +
                  product.geometry_key);
        }
        auto seed = make_declared_product_collision_object(
          product.source_object_id, planning_frame_, product.planning_from_product,
          geometry->second);
        if (!seed) {
          throw std::invalid_argument(
                  "scenario product " + product.source_object_id +
                  " cannot be seeded into the planning scene: " + seed.error().detail);
        }
        declared_seeds_.push_back(
          DeclaredProductSeed{product.source_object_id, std::move(seed.value())});
      }
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions obstacle_options;
    obstacle_options.callback_group = callback_group_;
    obstacle_subscription_ = create_subscription<
      restocker_interfaces::msg::ObstacleObservation>(
      obstacle_topic_, rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](restocker_interfaces::msg::ObstacleObservation::ConstSharedPtr observation) {
        latest_obstacles_ = std::move(observation);
      },
      obstacle_options);
    snapshot_client_ = create_client<restocker_interfaces::srv::GetWorldState>(
      snapshot_service_, rclcpp::ServicesQoS(), callback_group_);
    get_scene_client_ = create_client<moveit_msgs::srv::GetPlanningScene>(
      get_scene_service_, rclcpp::ServicesQoS(), callback_group_);
    apply_scene_client_ = create_client<moveit_msgs::srv::ApplyPlanningScene>(
      apply_scene_service_, rclcpp::ServicesQoS(), callback_group_);
    status_publisher_ = create_publisher<
      restocker_interfaces::msg::PlanningSceneProjectionStatus>(
      "/planning_scene_projection/status", rclcpp::QoS(1).reliable().transient_local());
    status_service_ = create_service<restocker_interfaces::srv::GetPlanningSceneProjectionStatus>(
      "/planning_scene_projection/get_status",
      [this](const std::shared_ptr<restocker_interfaces::srv::GetPlanningSceneProjectionStatus::
      Request>,
      std::shared_ptr<restocker_interfaces::srv::GetPlanningSceneProjectionStatus::Response>
      response) {
        // Same mutually-exclusive group as every apply/verify/lease callback. Preserve the
        // published evidence timestamp: this is an ordering barrier, not new verification.
        response->status = published_status_;
      }, rclcpp::ServicesQoS(), callback_group_);
    acquire_lease_service_ = create_service<
      restocker_interfaces::srv::AcquirePlanningSceneLease>(
      "/planning_scene_projection/acquire_lease",
      std::bind(
        &PlanningSceneProjectorNode::on_acquire_lease, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    validate_lease_service_ = create_service<
      restocker_interfaces::srv::ValidatePlanningSceneLease>(
      "/planning_scene_projection/validate_lease",
      std::bind(
        &PlanningSceneProjectorNode::on_validate_lease, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    release_lease_service_ = create_service<
      restocker_interfaces::srv::ReleasePlanningSceneLease>(
      "/planning_scene_projection/release_lease",
      std::bind(
        &PlanningSceneProjectorNode::on_release_lease, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);

    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(reconcile_period_sec_));
    timer_ = create_wall_timer(
      period, std::bind(&PlanningSceneProjectorNode::on_timer, this),
      callback_group_);
    publish_status(
      restocker_interfaces::msg::PlanningSceneProjectionStatus::STATE_STARTING,
      restocker_interfaces::msg::PlanningSceneProjectionStatus::ERROR_NONE,
      "waiting for world state and MoveIt services");
  }

private:
  using Status = restocker_interfaces::msg::PlanningSceneProjectionStatus;
  using SnapshotService = restocker_interfaces::srv::GetWorldState;
  using GetSceneService = moveit_msgs::srv::GetPlanningScene;
  using ApplySceneService = moveit_msgs::srv::ApplyPlanningScene;
  using AcquireLeaseService = restocker_interfaces::srv::AcquirePlanningSceneLease;
  using ValidateLeaseService = restocker_interfaces::srv::ValidatePlanningSceneLease;
  using ReleaseLeaseService = restocker_interfaces::srv::ReleasePlanningSceneLease;

  // Wait for a transform that is not there yet; only spent at start-up (the lookups are of
  // continuously published transforms).
  static constexpr auto kAttachmentTransformWait = std::chrono::milliseconds(200);

  enum class Stage : std::uint8_t
  {
    Idle,
    Snapshot,
    CurrentScene,
    ApplyScene,
    VerifyScene,
  };

  // A world-to-attached or attached-to-world move in flight. The product is momentarily neither a
  // plain world object nor proven attached, so the transition's own checker verifies it and the
  // whole projection is re-verified on the next cycle.
  struct PendingAttachmentTransition
  {
    bool attaching{false};
    moveit_msgs::msg::AttachedCollisionObject expected_attached_object;
    moveit_msgs::msg::CollisionObject expected_world_object;
    std::string object_id;
  };

  [[nodiscard]] static ProjectorStage to_protocol_stage(Stage stage)
  {
    switch (stage) {
      case Stage::Idle:
        return ProjectorStage::Idle;
      case Stage::Snapshot:
        return ProjectorStage::Snapshot;
      case Stage::CurrentScene:
        return ProjectorStage::CurrentScene;
      case Stage::ApplyScene:
        return ProjectorStage::ApplyScene;
      case Stage::VerifyScene:
        return ProjectorStage::VerifyScene;
    }
    throw std::logic_error("unknown projector stage");
  }

  [[nodiscard]] static const char * stage_name(Stage stage)
  {
    switch (stage) {
      case Stage::Idle:
        return "idle";
      case Stage::Snapshot:
        return "snapshot";
      case Stage::CurrentScene:
        return "current_scene";
      case Stage::ApplyScene:
        return "apply_scene";
      case Stage::VerifyScene:
        return "verify_scene";
    }
    throw std::logic_error("unknown projector stage");
  }

  // Service round-trip time, recorded where the response lands so a response arriving after its
  // stage was abandoned is still measured. A stage with no response leaves only a `timeout` record.
  //
  // The DEBUG record is per call (two per reconcile cycle at most). The INFO record is the tail:
  // a round trip that reached half the service timeout is the interesting one under load, and it
  // is what an ordinary log needs to show whether a deadline was a spike or the norm. Throttled
  // so a persistently slow service reports once per window instead of once per cycle.
  void record_stage_response(Stage stage, std::chrono::steady_clock::time_point sent, bool active)
  {
    const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - sent).count();
    RCLCPP_DEBUG(
      get_logger(), "stage timing: stage=%s outcome=%s seconds=%.4f", stage_name(stage),
      active ? "response" : "late_response", seconds);
    if (seconds >= slow_stage_report_sec_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "slow stage: stage=%s outcome=%s seconds=%.3f deadline=%.3f", stage_name(stage),
        active ? "response" : "late_response", seconds, service_timeout_sec_);
    }
  }

  void record_stage_timeout()
  {
    const double seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - stage_started_).count();
    RCLCPP_DEBUG(
      get_logger(), "stage timing: stage=%s outcome=timeout seconds=%.4f", stage_name(stage_),
      seconds);
    if (seconds >= slow_stage_report_sec_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "slow stage: stage=%s outcome=timeout seconds=%.3f deadline=%.3f", stage_name(stage_),
        seconds, service_timeout_sec_);
    }
  }

  void validate_configuration(const std::string & catalog_path) const
  {
    constexpr double kMinimumTimerSeconds = 0.001;
    constexpr double kMaximumDurationSeconds =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1.0e9;
    const bool positive_durations = std::isfinite(reconcile_period_sec_) &&
      reconcile_period_sec_ >= kMinimumTimerSeconds &&
      reconcile_period_sec_ <= kMaximumDurationSeconds &&
      std::isfinite(service_timeout_sec_) &&
      service_timeout_sec_ >= kMinimumTimerSeconds &&
      service_timeout_sec_ <= kMaximumDurationSeconds &&
      std::isfinite(service_total_timeout_sec_) &&
      service_total_timeout_sec_ >= service_timeout_sec_ &&
      service_total_timeout_sec_ <= kMaximumDurationSeconds &&
      std::isfinite(slow_stage_report_sec_) && slow_stage_report_sec_ >= 0.0 &&
      std::isfinite(max_observation_age_sec_) && max_observation_age_sec_ >= 0.0 &&
      max_observation_age_sec_ <= kMaximumDurationSeconds &&
      std::isfinite(max_future_skew_sec_) && max_future_skew_sec_ >= 0.0 &&
      max_future_skew_sec_ <= kMaximumDurationSeconds;
    const bool valid_tolerances = std::isfinite(verification_config_.dimension_tolerance) &&
      verification_config_.dimension_tolerance >= 0.0 &&
      std::isfinite(verification_config_.position_tolerance) &&
      verification_config_.position_tolerance >= 0.0 &&
      std::isfinite(verification_config_.orientation_tolerance_rad) &&
      verification_config_.orientation_tolerance_rad >= 0.0 &&
      std::isfinite(product_change_tolerance_.position_tolerance) &&
      product_change_tolerance_.position_tolerance >= 0.0 &&
      std::isfinite(product_change_tolerance_.orientation_tolerance_rad) &&
      product_change_tolerance_.orientation_tolerance_rad >= 0.0;
    // build_world_to_attached_diff rejects a touch-link list that omits the parent link; check here
    // so it does not first surface at the first grasp.
    const bool valid_settle_clearance = std::isfinite(attachment_settle_clearance_m_) &&
      attachment_settle_clearance_m_ >= 0.0;
    const bool valid_attachment_links = !attachment_parent_link_.empty() &&
      !grasp_center_frame_.empty() &&
      std::ranges::find(attachment_touch_links_, attachment_parent_link_) !=
      attachment_touch_links_.end() &&
      std::ranges::none_of(
      attachment_touch_links_, [](const std::string & link) {return link.empty();});
    const bool valid_obstacles = !obstacle_topic_.empty() &&
      std::isfinite(obstacle_max_age_sec_) && obstacle_max_age_sec_ > 0.0 &&
      obstacle_max_age_sec_ <= kMaximumDurationSeconds &&
      valid_obstacle_projection_config(obstacle_config_);
    if (planning_frame_.empty() || workcell_root_frame_.empty() ||
      workcell_description_.empty() || catalog_path.empty() || snapshot_service_.empty() ||
      get_scene_service_.empty() || apply_scene_service_.empty() || !positive_durations ||
      !valid_tolerances || !valid_attachment_links || !valid_settle_clearance || !valid_obstacles)
    {
      throw std::invalid_argument("planning-scene projector configuration is invalid");
    }
  }

  void on_timer()
  {
    if (!lease_protocol_->state().reconciliation_allowed) {
      return;
    }
    if (in_flight_) {
      if (std::chrono::steady_clock::now() >= service_deadline_) {
        handle_stage_deadline();
      }
      return;
    }
    if (!snapshot_client_->service_is_ready()) {
      publish_status(
        Status::STATE_DEGRADED, Status::ERROR_WORLD_STATE_UNAVAILABLE,
        "world-state snapshot service is unavailable");
      return;
    }
    if (!get_scene_client_->service_is_ready() || !apply_scene_client_->service_is_ready()) {
      publish_status(
        Status::STATE_DEGRADED, Status::ERROR_PLANNING_SCENE_UNAVAILABLE,
        "MoveIt planning-scene services are unavailable");
      return;
    }
    // Obstacle evidence is fixed here and read nowhere else, so the freshness proved now is that of
    // the value projected. Stale evidence stops the cycle and leaves scene obstacles untouched.
    if (const auto reason = stale_obstacle_evidence()) {
      publish_status(Status::STATE_DEGRADED, Status::ERROR_OBSTACLE_EVIDENCE_STALE, *reason);
      return;
    }
    candidate_obstacle_observation_ = latest_obstacles_;
    start_snapshot_request();
  }

  // A stale product observation keeps the product at its last known pose (Card 071), and a stale
  // obstacle observation likewise means the sensor cannot see: neither may become "nothing is
  // there".
  [[nodiscard]] std::optional<std::string> stale_obstacle_evidence() const
  {
    if (!latest_obstacles_) {
      if (!require_obstacle_evidence_) {
        return std::nullopt;
      }
      return "obstacle evidence is required but no observation has ever arrived";
    }
    const rclcpp::Time stamp(latest_obstacles_->header.stamp);
    if (stamp.nanoseconds() <= 0) {
      return "newest obstacle observation carries no stamp";
    }
    const auto age = now() - stamp;
    if (age > rclcpp::Duration::from_seconds(obstacle_max_age_sec_)) {
      return "newest obstacle observation is " + std::to_string(age.seconds()) +
             " s old, beyond the configured maximum";
    }
    if (age.nanoseconds() < -seconds_to_nanoseconds(max_future_skew_sec_)) {
      return "newest obstacle observation is stamped beyond the future-skew tolerance";
    }
    return std::nullopt;
  }

  void on_acquire_lease(
    const std::shared_ptr<AcquireLeaseService::Request> request,
    std::shared_ptr<AcquireLeaseService::Response> response)
  {
    auto reply = lease_protocol_->acquire(
      from_ros_request(*request), to_protocol_stage(stage_), now().nanoseconds());
    populate_ros_response(reply, *response);

    if (reply.action == AcquisitionAction::InvalidateReadOnlyGeneration) {
      invalidate_read_only_stage();
    }
    publish_transaction_status("planning-scene lease acquisition updated");
  }

  void on_validate_lease(
    const std::shared_ptr<ValidateLeaseService::Request> request,
    std::shared_ptr<ValidateLeaseService::Response> response)
  {
    populate_ros_response(lease_protocol_->validate(request->token), *response);
  }

  void on_release_lease(
    const std::shared_ptr<ReleaseLeaseService::Request> request,
    std::shared_ptr<ReleaseLeaseService::Response> response)
  {
    const auto reply = lease_protocol_->release(from_ros_request(*request));
    populate_ros_response(reply, *response);
    publish_transaction_status("planning-scene lease release updated");
  }

  void invalidate_read_only_stage()
  {
    if (stage_ == Stage::Snapshot) {
      snapshot_client_->prune_pending_requests();
    } else if (stage_ == Stage::CurrentScene) {
      get_scene_client_->prune_pending_requests();
    } else {
      return;
    }
    ++generation_;
    in_flight_ = false;
    stage_ = Stage::Idle;
  }

  void start_snapshot_request()
  {
    const std::uint64_t generation = ++generation_;
    begin_stage(Stage::Snapshot);
    publish_status(Status::STATE_SYNCHRONIZING, Status::ERROR_NONE, "requesting world snapshot");
    auto request = std::make_shared<SnapshotService::Request>();
    request->include_removed = false;
    request->include_events = false;
    const auto sent = std::chrono::steady_clock::now();
    snapshot_client_->async_send_request(
      request,
      [this, generation, sent](rclcpp::Client<SnapshotService>::SharedFuture future) {
        const bool active = active_generation(generation, Stage::Snapshot);
        record_stage_response(Stage::Snapshot, sent, active);
        if (!active) {
          return;
        }
        try {
          on_snapshot(future.get()->snapshot, generation);
        } catch (const std::exception & error) {
          fail(Status::ERROR_WORLD_STATE_UNAVAILABLE, error.what());
        }
      });
  }

  void on_snapshot(
    restocker_interfaces::msg::WorldStateSnapshot snapshot, std::uint64_t generation)
  {
    candidate_snapshot_ = std::move(snapshot);
    desired_revision_ = candidate_snapshot_.revision;
    Eigen::Isometry3d planning_from_workcell;
    try {
      const auto transform = tf_buffer_.lookupTransform(
        planning_frame_, workcell_root_frame_, tf2::TimePointZero);
      planning_from_workcell = tf2::transformToEigen(transform);
      candidate_planning_from_workcell_ = planning_from_workcell;
    } catch (const tf2::TransformException & error) {
      fail(Status::ERROR_TRANSFORM_UNAVAILABLE, error.what());
      return;
    }
    auto workcell = extract_workcell_collision_objects(
      workcell_description_, planning_frame_, planning_from_workcell);
    if (!workcell) {
      fail(Status::ERROR_CONFIGURATION, workcell.error().detail);
      return;
    }
    candidate_workcell_objects_ = std::move(workcell.value());
    request_scene(Stage::CurrentScene, generation);
  }

  void request_scene(Stage stage, std::uint64_t generation)
  {
    if (!get_scene_client_->service_is_ready()) {
      fail(Status::ERROR_PLANNING_SCENE_UNAVAILABLE, "planning-scene query service disappeared");
      return;
    }
    begin_stage(stage);
    auto request = std::make_shared<GetSceneService::Request>();
    request->components.components =
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
    const auto sent = std::chrono::steady_clock::now();
    get_scene_client_->async_send_request(
      request,
      [this, generation, stage, sent](rclcpp::Client<GetSceneService>::SharedFuture future) {
        const bool active = active_generation(generation, stage);
        record_stage_response(stage, sent, active);
        if (!active) {
          return;
        }
        try {
          if (stage == Stage::CurrentScene) {
            on_current_scene(future.get()->scene, generation);
          } else {
            on_verification_scene(future.get()->scene);
          }
        } catch (const std::exception & error) {
          fail(Status::ERROR_PLANNING_SCENE_UNAVAILABLE, error.what());
        }
      });
  }

  void on_current_scene(
    const moveit_msgs::msg::PlanningScene & current_scene, std::uint64_t generation)
  {
    std::set<std::string> attached_ids;
    for (const auto & attached : current_scene.robot_state.attached_collision_objects) {
      attached_ids.insert(attached.object.id);
    }
    SnapshotProjectionConfig projection_config;
    projection_config.planning_frame = planning_frame_;
    projection_config.now_ns = now().nanoseconds();
    projection_config.max_observation_age_ns = seconds_to_nanoseconds(max_observation_age_sec_);
    projection_config.max_future_skew_ns = seconds_to_nanoseconds(max_future_skew_sec_);
    projection_config.last_verified_revision = applied_revision_;
    // Card 050: the declared-product seeds, constant for the life of the node; the projection
    // keeps only those the snapshot does not carry.
    projection_config.declared_product_seeds = declared_seeds_;
    // Every surveyed lane in the planning frame: the workcell root transform with the lane's rail
    // offset applied, plus the survey bounds. The projection combines each with the lane's free
    // depth into one occupied-volume box.
    for (const auto & [id, lane] : lane_geometry_) {
      Eigen::Isometry3d workcell_from_lane = Eigen::Isometry3d::Identity();
      workcell_from_lane.translation().x() = lane.center_x_m;
      projection_config.lane_volumes.emplace(
        id,
        LaneVolumeGeometry{
          candidate_planning_from_workcell_ * workcell_from_lane, lane.usable_bounds_in_lane});
      const std::string collision_id = "restocker/lane/" + id;
      const auto retained = std::ranges::find_if(
        current_scene.world.collision_objects,
        [&collision_id](const moveit_msgs::msg::CollisionObject & object) {
          return object.id == collision_id;
        });
      if (retained == current_scene.world.collision_objects.end()) {
        projection_config.retained_lane_volume_objects.emplace(id, std::nullopt);
      } else {
        projection_config.retained_lane_volume_objects.emplace(id, *retained);
      }
    }
    if (candidate_snapshot_.robot.has_held_object) {
      // Carried product pose = current gripper pose composed with the grasp recorded when the jaws
      // closed, so it follows the arm without observing the product.
      auto planning_from_grasp_center = lookup_transform(planning_frame_, grasp_center_frame_);
      if (!planning_from_grasp_center) {
        return;
      }
      projection_config.planning_from_grasp_center = *planning_from_grasp_center;
    }
    auto products = project_world_snapshot(
      candidate_snapshot_, projection_config, *catalog_, attached_ids);
    if (!products) {
      // A future observation is reported apart from an unreadable snapshot: it is a temporal
      // evidence condition the next reconcile cycle clears when now() advances, while the
      // structural code was treated as terminal downstream. (An aged observation is not a failure
      // at all; the product is projected and named in the applied detail, Card 071.)
      // Under full-suite load the projector's /clock processing can lag the producer's stamp by
      // 50 to 100 ms, which is exactly the window where world state (100 ms future skew) accepts
      // the observation and this projection (50 ms) sees it as future; classifying that as
      // ERROR_INVALID_SNAPSHOT rejected the scene on first sight and, while a product is held,
      // latched operator-required instead of settling for one 0.5 s cycle. The cycle is abandoned
      // and scene geometry retained either way.
      const bool evidence_temporal =
        products.error().code == ProjectionErrorCode::FutureObservation;
      fail(
        evidence_temporal ? Status::ERROR_OBSERVATION_EVIDENCE_STALE :
        Status::ERROR_INVALID_SNAPSHOT,
        to_string(products.error().code) + ": " + products.error().detail);
      return;
    }

    if (products.value().held_product) {
      carried_product_ = *products.value().held_product;
    }
    products_left_to_their_lanes_ = std::move(products.value().lane_owned_object_ids);
    // Committed only when this cycle is certified (complete_verification), so a log line or note
    // never names a cycle that was abandoned later.
    candidate_aged_products_ = std::move(products.value().aged_object_ages_ns);
    candidate_required_attached_ids_ = std::move(products.value().required_attached_ids);
    const std::set<std::string> pending_detach_ids =
      std::move(products.value().pending_detach_ids);
    candidate_attached_geometry_ = std::move(products.value().attached_geometry);
    candidate_desired_objects_ = candidate_workcell_objects_;
    candidate_desired_objects_.insert(
      candidate_desired_objects_.end(),
      std::make_move_iterator(products.value().world_objects.begin()),
      std::make_move_iterator(products.value().world_objects.end()));

    // Obstacles are folded in against this cycle's modelled geometry, so known-geometry subtraction
    // sees the workcell, stocked products and carried product as this cycle understands them.
    if (!reconcile_accepted_obstacles()) {
      return;
    }
    candidate_desired_objects_.insert(
      candidate_desired_objects_.end(), accepted_obstacles_.begin(), accepted_obstacles_.end());

    // A world/robot product transition comes first: until it happens the desired world set omits
    // the product, so the ordinary diff would delete its geometry instead of transferring it. At
    // most one product is held, so at most one transition is outstanding.
    if (start_attachment_transition(current_scene, pending_detach_ids, generation)) {
      return;
    }
    pending_transition_.reset();

    // A product still where the scene has it keeps the scene's pose, so contact-solver noise does
    // not produce a diff. See retain_unmoved_product_geometry.
    retain_unmoved_product_geometry(
      candidate_desired_objects_, current_scene, product_change_tolerance_);

    auto current_verification = verify_planning_scene(
      candidate_desired_objects_, candidate_required_attached_ids_, current_scene,
      verification_config_);
    if (current_verification) {
      complete_verification("verified existing planning-scene projection");
      return;
    }
    // Every diff advances the content generation that invalidates plans in flight; log why this
    // cycle writes instead of certifying.
    RCLCPP_INFO(
      get_logger(), "scene diff needed: %s: %s",
      to_string(current_verification.error().code).c_str(),
      current_verification.error().detail.c_str());
    auto diff = build_planning_scene_diff(candidate_desired_objects_, current_scene);
    if (!diff) {
      fail(Status::ERROR_INVALID_SNAPSHOT, diff.error().detail);
      return;
    }
    request_apply(std::move(diff.value()), generation);
  }

  // Decides the obstacle geometry this cycle carries. False when the cycle has been failed.
  //
  // Accepted geometry is retained byte-for-byte while the newest observation describes the same
  // obstacles: any desired-set change produces a diff, which invalidates plans in flight. A changed
  // set is adopted only from an observation taken while the robot was still, because the carried
  // product comes from this cycle's snapshot and the boxes from an earlier exposure.
  [[nodiscard]] bool reconcile_accepted_obstacles()
  {
    if (!candidate_obstacle_observation_) {
      return true;
    }
    auto projected = project_obstacle_observation(
      *candidate_obstacle_observation_, candidate_desired_objects_, candidate_attached_geometry_,
      obstacle_config_);
    if (!projected) {
      fail(
        Status::ERROR_INVALID_SNAPSHOT,
        to_string(projected.error().code) + ": " + projected.error().detail);
      return false;
    }
    if (obstacle_sets_match(accepted_obstacles_, projected.value(), obstacle_config_)) {
      return true;
    }
    if (!candidate_obstacle_observation_->robot_static) {
      // Not an error: keep the accepted obstacles; the next still observation adopts the change.
      return true;
    }
    accepted_obstacles_ = std::move(projected.value());
    return true;
  }

  // Applies the one world-to-attached or attached-to-world move the snapshot still needs, if any.
  // Returns true when a transition was submitted and this reconciliation cycle now belongs to it.
  [[nodiscard]] bool start_attachment_transition(
    const moveit_msgs::msg::PlanningScene & current_scene,
    const std::set<std::string> & pending_detach_ids, std::uint64_t generation)
  {
    std::set<std::string> observed_attached;
    for (const auto & attached : current_scene.robot_state.attached_collision_objects) {
      if (is_projector_managed_id(attached.object.id)) {
        observed_attached.insert(attached.object.id);
      }
    }
    std::set<std::string> to_attach;
    std::ranges::set_difference(
      candidate_required_attached_ids_, observed_attached,
      std::inserter(to_attach, to_attach.end()));

    if (!to_attach.empty()) {
      return start_attach_transition(*to_attach.begin(), generation);
    }
    if (!pending_detach_ids.empty()) {
      return start_detach_transition(current_scene, *pending_detach_ids.begin(), generation);
    }
    return false;
  }

  // A transform between two robot frames, or nullopt when TF cannot supply it yet. Fails this cycle
  // only; the product stays a world object meanwhile, which is the safe state to plan against.
  [[nodiscard]] std::optional<Eigen::Isometry3d> lookup_transform(
    const std::string & target_frame, const std::string & source_frame)
  {
    try {
      return tf2::transformToEigen(
        tf_buffer_.lookupTransform(
          target_frame, source_frame, tf2::TimePointZero, kAttachmentTransformWait));
    } catch (const tf2::TransformException & error) {
      fail(Status::ERROR_TRANSFORM_UNAVAILABLE, error.what());
      return std::nullopt;
    }
  }

  [[nodiscard]] bool start_attach_transition(
    const std::string & object_id, std::uint64_t generation)
  {
    const auto world_object = std::ranges::find_if(
      candidate_attached_geometry_,
      [&object_id](const moveit_msgs::msg::CollisionObject & object) {
        return object.id == object_id;
      });
    if (world_object == candidate_attached_geometry_.end()) {
      fail(
        Status::ERROR_INVALID_SNAPSHOT,
        "attached product " + object_id + " carries no collision geometry");
      return true;
    }
    // Carry pose = parent <- grasp_center (fixed joint, constant) composed with the grasp recorded
    // at jaw closure. No observed product pose is paired with a robot pose from another moment.
    auto parent_from_grasp_center =
      lookup_transform(attachment_parent_link_, grasp_center_frame_);
    if (!parent_from_grasp_center) {
      return true;
    }
    Eigen::Isometry3d grasp_center_from_product;
    tf2::fromMsg(
      candidate_snapshot_.robot.grasp_center_from_held_object,
      grasp_center_from_product);
    const geometry_msgs::msg::Pose parent_from_product =
      tf2::toMsg(*parent_from_grasp_center * grasp_center_from_product);

    moveit_msgs::msg::CollisionObject carried = *world_object;
    if (!apply_settle_clearance(carried)) {
      fail(
        Status::ERROR_INVALID_SNAPSHOT,
        "attached product " + object_id + " geometry cannot carry the settle clearance");
      return true;
    }
    auto projection = build_world_to_attached_diff(
      carried, attachment_parent_link_, attachment_touch_links_, parent_from_product);
    if (!projection) {
      fail(Status::ERROR_INVALID_SNAPSHOT, projection.error().detail);
      return true;
    }
    PendingAttachmentTransition transition;
    transition.attaching = true;
    transition.expected_attached_object = projection.value().expected_attached_object;
    transition.object_id = object_id;
    pending_transition_ = std::move(transition);
    request_apply(std::move(projection.value().diff), generation);
    return true;
  }

  // Shortens the product along its axis by the settle clearance at each end. False when the
  // geometry is not a single cylinder tall enough (no attached-object contract for other shapes).
  [[nodiscard]] bool apply_settle_clearance(moveit_msgs::msg::CollisionObject & object) const
  {
    if (attachment_settle_clearance_m_ <= 0.0) {
      return true;
    }
    if (object.primitives.size() != 1U ||
      object.primitives.front().type != shape_msgs::msg::SolidPrimitive::CYLINDER ||
      object.primitives.front().dimensions.size() !=
      shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS + 1U)
    {
      return false;
    }
    auto & height =
      object.primitives.front().dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT];
    const double shortened = height - 2.0 * attachment_settle_clearance_m_;
    if (!std::isfinite(shortened) || shortened <= 0.0) {
      return false;
    }
    height = shortened;
    return true;
  }

  [[nodiscard]] bool start_detach_transition(
    const moveit_msgs::msg::PlanningScene & current_scene, const std::string & object_id,
    std::uint64_t generation)
  {
    const auto attached = std::ranges::find_if(
      current_scene.robot_state.attached_collision_objects,
      [&object_id](const moveit_msgs::msg::AttachedCollisionObject & candidate) {
        return candidate.object.id == object_id;
      });
    if (attached == current_scene.robot_state.attached_collision_objects.end()) {
      return false;
    }
    const auto released = std::ranges::find_if(
      candidate_snapshot_.objects,
      [&object_id](const restocker_interfaces::msg::TrackedObject & object) {
        return "restocker/object/" + std::to_string(object.id) == object_id;
      });
    if (released == candidate_snapshot_.objects.end()) {
      fail(
        Status::ERROR_INVALID_SNAPSHOT,
        "attached product " + object_id + " is absent from the authoritative snapshot");
      return true;
    }
    // MoveIt must be handed an attached object back to the world before forgetting it, and that
    // transition needs a pose. The last observation predates the grasp and the product is rolling
    // down its lane, so use the pose the arm let go at. It lasts one cycle: the product then
    // belongs to its lane's occupied volume and the next diff deletes it.
    const geometry_msgs::msg::Pose world_from_product =
      products_left_to_their_lanes_.contains(released->id) && carried_product_ ?
      tf2::toMsg(carried_product_->planning_from_product) : released->pose.pose;
    auto projection = build_attached_to_world_diff(
      *attached, planning_frame_, world_from_product);
    if (!projection) {
      fail(Status::ERROR_INVALID_SNAPSHOT, projection.error().detail);
      return true;
    }
    PendingAttachmentTransition transition;
    transition.attaching = false;
    transition.expected_world_object = projection.value().expected_world_object;
    transition.object_id = object_id;
    pending_transition_ = std::move(transition);
    request_apply(std::move(projection.value().diff), generation);
    return true;
  }

  void request_apply(moveit_msgs::msg::PlanningScene diff, std::uint64_t generation)
  {
    if (!apply_scene_client_->service_is_ready()) {
      fail(Status::ERROR_PLANNING_SCENE_UNAVAILABLE, "planning-scene apply service disappeared");
      return;
    }
    lease_protocol_->record_scene_diff_submission();
    begin_stage(Stage::ApplyScene);
    lease_protocol_->record_side_effect_unknown();
    publish_status(
      Status::STATE_SYNCHRONIZING, Status::ERROR_NONE,
      "submitting planning-scene diff");
    apply_response_rejected_ = false;
    auto request = std::make_shared<ApplySceneService::Request>();
    request->scene = std::move(diff);
    const auto sent = std::chrono::steady_clock::now();
    apply_scene_client_->async_send_request(
      request,
      [this, generation, sent](rclcpp::Client<ApplySceneService>::SharedFuture future) {
        const bool active = active_generation(generation, Stage::ApplyScene);
        record_stage_response(Stage::ApplyScene, sent, active);
        if (!active) {
          return;
        }
        try {
          apply_response_rejected_ = !future.get()->success;
          request_scene(Stage::VerifyScene, generation);
        } catch (const std::exception & error) {
          mark_apply_outcome_unknown(error.what());
        }
      });
  }

  void on_verification_scene(const moveit_msgs::msg::PlanningScene & observed_scene)
  {
    if (pending_transition_) {
      verify_attachment_transition(observed_scene);
      return;
    }
    auto verification = verify_planning_scene(
      candidate_desired_objects_, candidate_required_attached_ids_, observed_scene,
      verification_config_);
    if (!verification) {
      lease_protocol_->record_side_effect_unknown();
      const auto error_code = apply_response_rejected_ ?
        Status::ERROR_APPLY_REJECTED : Status::ERROR_VERIFICATION_FAILED;
      fail(
        error_code,
        to_string(verification.error().code) + ": " + verification.error().detail);
      return;
    }
    complete_verification("planning-scene projection verified");
  }

  void verify_attachment_transition(const moveit_msgs::msg::PlanningScene & observed_scene)
  {
    const auto & transition = *pending_transition_;
    auto verification = transition.attaching ?
      verify_world_to_attached_transition(
      transition.expected_attached_object, observed_scene, verification_config_) :
      verify_attached_to_world_transition(
      transition.expected_world_object, observed_scene, verification_config_);
    if (!verification) {
      lease_protocol_->record_side_effect_unknown();
      const auto error_code = apply_response_rejected_ ?
        Status::ERROR_APPLY_REJECTED : Status::ERROR_VERIFICATION_FAILED;
      pending_transition_.reset();
      fail(
        error_code,
        to_string(verification.error().code) + ": " + verification.error().detail);
      return;
    }
    // The rest of the projection is not re-derived yet, so applied_revision_ does not advance; the
    // next cycle verifies the whole scene and records that proof against the lease.
    const std::string detail = std::string(transition.attaching ? "attached " : "detached ") +
      transition.object_id + " in the planning scene";
    pending_transition_.reset();
    in_flight_ = false;
    stage_ = Stage::Idle;
    publish_status(Status::STATE_SYNCHRONIZING, Status::ERROR_NONE, detail);
  }

  void begin_stage(Stage stage)
  {
    stage_ = stage;
    in_flight_ = true;
    stage_started_ = std::chrono::steady_clock::now();
    stage_overdue_reported_ = false;
    if (stage == Stage::ApplyScene) {
      apply_timeout_reported_ = false;
    }
    service_deadline_ = stage_started_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(service_timeout_sec_));
    stage_total_deadline_ = stage_started_ +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(service_total_timeout_sec_));
  }

  [[nodiscard]] bool active_generation(std::uint64_t generation, Stage stage) const
  {
    return in_flight_ && generation == generation_ && stage == stage_;
  }

  // One stage deadline has passed. The apply stage waits for the response it must never
  // re-send. A read-only stage waits too — a late answer is still an answer, and cancelling at
  // the first deadline makes a service that answers in T > service_timeout_sec unserviceable for
  // ever — but only up to the total budget, after which the request is abandoned so a request
  // lost by a restarted server cannot wedge the projector. While the request is held the status
  // stays DEGRADED with a fresh stamp, so the authority gate keeps seeing kSettling and nothing
  // moves (Milestone 10 §5, "A slow dependency is not a dead dependency").
  void handle_stage_deadline()
  {
    if (stage_ == Stage::ApplyScene) {
      record_stage_timeout();
      if (!apply_timeout_reported_) {
        mark_apply_outcome_unknown(
          "planning-scene apply exceeded its timeout; waiting for the original response");
      }
      return;
    }

    const bool service_alive = stage_ == Stage::Snapshot ?
      snapshot_client_->service_is_ready() : get_scene_client_->service_is_ready();
    const bool within_total_budget =
      std::chrono::steady_clock::now() < stage_total_deadline_;
    if (service_alive && within_total_budget) {
      if (!stage_overdue_reported_) {
        stage_overdue_reported_ = true;
        record_stage_timeout();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "stage %s exceeded its %.3f s deadline and is held (total budget %.3f s)",
          stage_name(stage_), service_timeout_sec_, service_total_timeout_sec_);
      }
      // Republished on every tick so the status stamp keeps moving: the authority gate ages the
      // status itself, and a held request must read as kSettling now, not as a status left behind
      // seconds ago. Published without the console line and without moving the log-dedup state: a
      // service that is merely slow would otherwise write one degradation per reconcile cycle for
      // as long as the load lasts, and the recovery line after it. The give-up path below still
      // logs in full.
      const std::uint16_t error = stage_ == Stage::Snapshot ?
        Status::ERROR_WORLD_STATE_UNAVAILABLE : Status::ERROR_PLANNING_SCENE_UNAVAILABLE;
      publish_status(
        Status::STATE_DEGRADED, error,
        "service request exceeded configured timeout; waiting for the response",
        /*log_to_console=*/ false);
      return;
    }
    abandon_stage("service request exceeded configured timeout");
  }

  // Cancels the in-flight read-only request, invalidates any answer that is still in flight, and
  // fails the cycle. The request is issued again by a later reconcile cycle.
  void abandon_stage(const std::string & detail)
  {
    record_stage_timeout();
    if (stage_ == Stage::Snapshot) {
      snapshot_client_->prune_pending_requests();
    } else {
      get_scene_client_->prune_pending_requests();
      if (stage_ == Stage::VerifyScene) {
        lease_protocol_->record_side_effect_unknown();
      }
    }
    const std::uint16_t error = stage_ == Stage::Snapshot ?
      Status::ERROR_WORLD_STATE_UNAVAILABLE : Status::ERROR_PLANNING_SCENE_UNAVAILABLE;
    ++generation_;
    fail(error, detail);
  }

  void mark_apply_outcome_unknown(const std::string & detail)
  {
    lease_protocol_->record_side_effect_unknown();
    apply_timeout_reported_ = true;
    publish_status(Status::STATE_DEGRADED, Status::ERROR_APPLY_OUTCOME_UNKNOWN, detail);
  }

  void fail(std::uint16_t error, const std::string & detail)
  {
    in_flight_ = false;
    stage_ = Stage::Idle;
    publish_status(Status::STATE_DEGRADED, error, detail);
  }

  void complete_verification(const std::string & detail)
  {
    const auto transition = lease_protocol_->record_verification(
      candidate_snapshot_.revision, now().nanoseconds());
    if (transition == VerificationTransition::RejectedWhileHeld) {
      fail(
        Status::ERROR_VERIFICATION_FAILED,
        "planning-scene verification completed while transaction lease was held");
      return;
    }
    in_flight_ = false;
    stage_ = Stage::Idle;
    applied_revision_ = candidate_snapshot_.revision;
    report_aged_products(candidate_aged_products_);
    aged_products_ = candidate_aged_products_;
    publish_status(
      Status::STATE_APPLIED, Status::ERROR_NONE, detail + lane_owned_note() + aged_note());
  }

  // Names products drawn at a last known pose older than max_observation_age_sec. They are
  // obstacles like any other product; the note keeps "not looked at lately" visible in the
  // certified status instead of turning it into a scene fault (Milestone 10 §6, Card 071).
  // IDs only, and at most kAgedNoteMaxIds of them: the status is logged whenever its detail
  // changes, so the note must change only when the aged set does. Ages are in the transition log.
  [[nodiscard]] std::string aged_note() const
  {
    constexpr std::size_t kAgedNoteMaxIds = 8;
    if (aged_products_.empty()) {
      return {};
    }
    std::string note = "; held at last known pose, evidence older than max_observation_age:";
    std::size_t named = 0;
    for (const auto & aged : aged_products_) {
      if (named == kAgedNoteMaxIds) {
        note += " (+" + std::to_string(aged_products_.size() - named) + " more)";
        break;
      }
      note += " restocker/object/" + std::to_string(aged.first);
      ++named;
    }
    return note;
  }

  // Logs when a product first ages past max_observation_age_sec and when fresh evidence brings it
  // back, so a run's log carries each transition once rather than every cycle.
  void report_aged_products(const std::map<std::uint64_t, std::int64_t> & aged)
  {
    for (const auto & [id, age_ns] : aged) {
      if (!aged_products_.contains(id)) {
        RCLCPP_INFO(
          get_logger(),
          "product restocker/object/%s evidence is %.1f s old (> max_observation_age_sec "
          "%.1f s): kept in the scene at its last known pose",
          std::to_string(id).c_str(), static_cast<double>(age_ns) * 1.0e-9,
          max_observation_age_sec_);
      }
    }
    for (const auto & [id, age_ns] : aged_products_) {
      if (!aged.contains(id)) {
        RCLCPP_INFO(
          get_logger(),
          "product restocker/object/%s is no longer aged (re-observed, lane-owned or removed)",
          std::to_string(id).c_str());
      }
    }
  }

  // Names products the scene describes as part of a lane. They carry no geometry or freshness
  // gate (the lane's occupied-volume box contains them), so the note keeps such a scene distinct
  // from one with the products loose in the tray.
  [[nodiscard]] std::string lane_owned_note() const
  {
    if (products_left_to_their_lanes_.empty()) {
      return {};
    }
    std::string note = "; carried by their lane's occupied volume rather than individually:";
    for (const auto id : products_left_to_their_lanes_) {
      note += " restocker/object/" + std::to_string(id);
    }
    return note;
  }

  void publish_transaction_status(const std::string & detail)
  {
    if (lease_protocol_->state().phase == PlanningSceneLeasePhase::None) {
      return;
    }
    if (apply_timeout_reported_ && stage_ == Stage::ApplyScene) {
      publish_status(Status::STATE_DEGRADED, Status::ERROR_APPLY_OUTCOME_UNKNOWN, detail);
      return;
    }
    publish_status(Status::STATE_SYNCHRONIZING, Status::ERROR_NONE, detail);
  }

  [[nodiscard]] static std::uint8_t transaction_state(PlanningSceneLeasePhase phase)
  {
    switch (phase) {
      case PlanningSceneLeasePhase::None:
        return Status::STATE_APPLIED;
      case PlanningSceneLeasePhase::Draining:
        return Status::STATE_TRANSACTION_DRAINING;
      case PlanningSceneLeasePhase::Held:
        return Status::STATE_TRANSACTION_HELD;
      case PlanningSceneLeasePhase::Releasing:
        return Status::STATE_TRANSACTION_RELEASING;
    }
    throw std::logic_error("unknown planning-scene lease phase");
  }

  [[nodiscard]] static std::uint8_t ros_lease_phase(PlanningSceneLeasePhase phase)
  {
    using LeaseMessage = restocker_interfaces::msg::PlanningSceneLease;
    switch (phase) {
      case PlanningSceneLeasePhase::None:
        return LeaseMessage::PHASE_NONE;
      case PlanningSceneLeasePhase::Draining:
        return LeaseMessage::PHASE_DRAINING;
      case PlanningSceneLeasePhase::Held:
        return LeaseMessage::PHASE_HELD;
      case PlanningSceneLeasePhase::Releasing:
        return LeaseMessage::PHASE_RELEASING;
    }
    throw std::logic_error("unknown planning-scene lease phase");
  }

  // `log_to_console` false publishes the status but neither writes it to the console nor moves
  // the dedup state, so a condition that is expected to clear itself (a held request under load)
  // can be reported to consumers on every cycle without writing a line per cycle, and without
  // hiding the give-up line that follows it.
  void publish_status(
    std::uint8_t state, std::uint16_t error, const std::string & detail,
    bool log_to_console = true)
  {
    const auto protocol_state = lease_protocol_->state();
    if (state != Status::STATE_DEGRADED && protocol_state.phase != PlanningSceneLeasePhase::None) {
      state = transaction_state(protocol_state.phase);
    }
    Status status;
    status.header.stamp = now();
    status.header.frame_id = planning_frame_;
    status.projector_epoch = protocol_state.projector_epoch;
    status.state = state;
    status.desired_revision = desired_revision_;
    status.applied_revision = applied_revision_;
    status.verification_epoch = protocol_state.verification_epoch;
    status.scene_content_generation = protocol_state.scene_content_generation;
    status.lease_id = protocol_state.lease ? protocol_state.lease->lease_id : 0;
    status.lease_phase = ros_lease_phase(protocol_state.phase);
    status.error_code = error;
    status.detail = detail;
    published_status_ = status;
    status_publisher_->publish(status);
    if (state == Status::STATE_SYNCHRONIZING) {
      return;
    }
    if (state != last_status_state_ || error != last_status_error_ ||
      detail != last_status_detail_)
    {
      if (!log_to_console) {
        return;
      }
      if (state == Status::STATE_DEGRADED) {
        RCLCPP_WARN(get_logger(), "projection degraded (%u): %s", error, detail.c_str());
      } else {
        RCLCPP_INFO(get_logger(), "projection status: %s", detail.c_str());
      }
      last_status_state_ = state;
      last_status_error_ = error;
      last_status_detail_ = detail;
    }
  }

  [[nodiscard]] static std::int64_t seconds_to_nanoseconds(double seconds)
  {
    return static_cast<std::int64_t>(seconds * 1.0e9);
  }

  std::string planning_frame_;
  std::string workcell_root_frame_;
  std::string workcell_description_;
  std::string snapshot_service_;
  std::string get_scene_service_;
  std::string apply_scene_service_;
  double reconcile_period_sec_{0.5};
  double service_timeout_sec_{2.0};
  double service_total_timeout_sec_{4.0};
  double slow_stage_report_sec_{1.0};
  double max_observation_age_sec_{0.5};
  double max_future_skew_sec_{0.05};
  SceneVerificationConfig verification_config_;
  SceneVerificationConfig product_change_tolerance_;
  std::unique_ptr<ProductCollisionCatalog> catalog_;
  // Card 050: scenario-declared products seeded into the scene until world state tracks them.
  std::string scenario_path_;
  std::vector<DeclaredProductSeed> declared_seeds_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<SnapshotService>::SharedPtr snapshot_client_;
  rclcpp::Client<GetSceneService>::SharedPtr get_scene_client_;
  rclcpp::Client<ApplySceneService>::SharedPtr apply_scene_client_;
  rclcpp::Publisher<Status>::SharedPtr status_publisher_;
  rclcpp::Service<AcquireLeaseService>::SharedPtr acquire_lease_service_;
  rclcpp::Service<ValidateLeaseService>::SharedPtr validate_lease_service_;
  rclcpp::Service<restocker_interfaces::srv::GetPlanningSceneProjectionStatus>::SharedPtr
    status_service_;
  Status published_status_;
  rclcpp::Service<ReleaseLeaseService>::SharedPtr release_lease_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<PlanningSceneLeaseProtocol> lease_protocol_;

  bool in_flight_{false};
  bool apply_timeout_reported_{false};
  bool apply_response_rejected_{false};
  Stage stage_{Stage::Idle};
  std::chrono::steady_clock::time_point stage_started_{};
  std::uint64_t generation_{0};
  std::chrono::steady_clock::time_point service_deadline_;
  std::chrono::steady_clock::time_point stage_total_deadline_{};
  bool stage_overdue_reported_{false};
  restocker_interfaces::msg::WorldStateSnapshot candidate_snapshot_;
  std::vector<moveit_msgs::msg::CollisionObject> candidate_workcell_objects_;
  std::vector<moveit_msgs::msg::CollisionObject> candidate_desired_objects_;
  std::set<std::string> candidate_required_attached_ids_;
  std::vector<moveit_msgs::msg::CollisionObject> candidate_attached_geometry_;
  std::string attachment_parent_link_;
  std::string grasp_center_frame_;
  std::vector<std::string> attachment_touch_links_;
  double attachment_settle_clearance_m_{0.002};
  std::string obstacle_topic_;
  double obstacle_max_age_sec_{2.0};
  bool require_obstacle_evidence_{false};
  ObstacleProjectionConfig obstacle_config_;
  rclcpp::Subscription<restocker_interfaces::msg::ObstacleObservation>::SharedPtr
    obstacle_subscription_;
  // Newest observation received, and the one this cycle may use. Separate because the subscription
  // fires between cycles; only the candidate has had its freshness checked.
  restocker_interfaces::msg::ObstacleObservation::ConstSharedPtr latest_obstacles_;
  restocker_interfaces::msg::ObstacleObservation::ConstSharedPtr candidate_obstacle_observation_;
  // Obstacle geometry in the scene. Retained across a stale or moving-robot cycle: dropping it
  // would report a clear corridor without having looked.
  std::vector<moveit_msgs::msg::CollisionObject> accepted_obstacles_;
  std::optional<PendingAttachmentTransition> pending_transition_;
  // Pose derivation for the held product, kept after release to name the pose for the
  // attached-to-world transition.
  std::optional<DerivedProductPose> carried_product_;
  // Products this cycle left to their lanes' occupied volumes.
  std::set<std::uint64_t> products_left_to_their_lanes_;
  std::map<std::uint64_t, std::int64_t> aged_products_;
  std::map<std::uint64_t, std::int64_t> candidate_aged_products_;
  // Shelf survey by lane ID, and this cycle's workcell root transform; they place each lane's box.
  std::map<std::string, LaneManipulationGeometry> lane_geometry_;
  Eigen::Isometry3d candidate_planning_from_workcell_{Eigen::Isometry3d::Identity()};
  std::uint64_t desired_revision_{0};
  std::uint64_t applied_revision_{0};
  std::uint8_t last_status_state_{std::numeric_limits<std::uint8_t>::max()};
  std::uint16_t last_status_error_{std::numeric_limits<std::uint16_t>::max()};
  std::string last_status_detail_;
};

}  // namespace restocker_task_executor

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_task_executor::PlanningSceneProjectorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("planning_scene_projector"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
