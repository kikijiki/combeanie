// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <cinttypes>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/lane_observation.hpp>
#include <restocker_interfaces/msg/object_observation.hpp>
#include <restocker_interfaces/msg/robot_telemetry.hpp>
#include <restocker_interfaces/msg/shelf_lane.hpp>
#include <restocker_interfaces/srv/checkpoint_task_state.hpp>
#include <restocker_interfaces/srv/commit_reserved_attachment.hpp>
#include <restocker_interfaces/srv/commit_reserved_detachment.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/invalidate_lane_evidence.hpp>
#include <restocker_interfaces/srv/release_task_reservation.hpp>
#include <restocker_interfaces/srv/reserve_task.hpp>
#include <restocker_interfaces/srv/set_lane_policy.hpp>
#include <restocker_interfaces/srv/validate_execution_world_authority.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_world_state/clock_authority.hpp"
#include "restocker_world_state/lane_config.hpp"
#include "restocker_world_state/ros_conversions.hpp"
#include "restocker_world_state/world_state.hpp"

namespace restocker_world_state
{

class WorldStateNode final : public rclcpp::Node
{
public:
  WorldStateNode()
  : Node("world_state")
  {
    WorldStateConfig config;
    config.planning_frame = declare_parameter<std::string>("planning_frame", "world");
    const auto maximum_age_ms = declare_parameter<std::int64_t>("maximum_observation_age_ms", 2000);
    const auto lane_evidence_validity_ms =
      declare_parameter<std::int64_t>("lane_evidence_validity_ms", 60000);
    const auto future_skew_ms = declare_parameter<std::int64_t>("maximum_future_skew_ms", 100);
    if (maximum_age_ms <= 0 || future_skew_ms < 0 || lane_evidence_validity_ms <= 0) {
      throw std::invalid_argument("world-state freshness parameters are invalid");
    }
    const auto operation_journal_capacity =
      declare_parameter<std::int64_t>("operation_journal_capacity", 4096);
    if (operation_journal_capacity <= 0) {
      throw std::invalid_argument("operation_journal_capacity must be positive");
    }
    config.maximum_observation_age = std::chrono::milliseconds(maximum_age_ms);
    config.lane_evidence_validity = std::chrono::milliseconds(lane_evidence_validity_ms);
    config.maximum_future_skew = std::chrono::milliseconds(future_skew_ms);
    config.operation_journal_capacity = static_cast<std::size_t>(operation_journal_capacity);
    // How far a confirm frame may move the reserved product before world state replaces the held
    // pose during a reservation's pre-attach window (ADR-0016). The store validates the value.
    config.reserved_pose_divergence_bound_m = declare_parameter<double>(
      "reserved_pose_divergence_bound_m", config.reserved_pose_divergence_bound_m);
    const auto joint_lower_limits = declare_parameter<std::vector<double>>(
      "robot_joint_lower_limits",
      std::vector<double>(config.joint_lower_limits.begin(), config.joint_lower_limits.end()));
    const auto joint_upper_limits = declare_parameter<std::vector<double>>(
      "robot_joint_upper_limits",
      std::vector<double>(config.joint_upper_limits.begin(), config.joint_upper_limits.end()));
    if (joint_lower_limits.size() != kArmJointCount ||
      joint_upper_limits.size() != kArmJointCount)
    {
      throw std::invalid_argument("robot joint limit parameters must contain exactly six values");
    }
    std::copy(
      joint_lower_limits.begin(), joint_lower_limits.end(),
      config.joint_lower_limits.begin());
    std::copy(
      joint_upper_limits.begin(), joint_upper_limits.end(),
      config.joint_upper_limits.begin());
    config.joint_limit_tolerance =
      declare_parameter<double>("robot_joint_limit_tolerance", config.joint_limit_tolerance);
    config.rail_lower_limit =
      declare_parameter<double>("robot_rail_lower_limit", config.rail_lower_limit);
    config.rail_upper_limit =
      declare_parameter<double>("robot_rail_upper_limit", config.rail_upper_limit);
    const auto gripper_joint_lower_limits = declare_parameter<std::vector<double>>(
      "robot_gripper_joint_lower_limits",
      std::vector<double>(
        config.gripper_joint_lower_limits.begin(), config.gripper_joint_lower_limits.end()));
    const auto gripper_joint_upper_limits = declare_parameter<std::vector<double>>(
      "robot_gripper_joint_upper_limits",
      std::vector<double>(
        config.gripper_joint_upper_limits.begin(), config.gripper_joint_upper_limits.end()));
    if (gripper_joint_lower_limits.size() != kGripperJointCount ||
      gripper_joint_upper_limits.size() != kGripperJointCount)
    {
      throw std::invalid_argument(
              "robot gripper joint limit parameters must contain exactly two values");
    }
    std::copy(
      gripper_joint_lower_limits.begin(), gripper_joint_lower_limits.end(),
      config.gripper_joint_lower_limits.begin());
    std::copy(
      gripper_joint_upper_limits.begin(), gripper_joint_upper_limits.end(),
      config.gripper_joint_upper_limits.begin());
    planning_frame_ = config.planning_frame;

    const std::string lane_semantics_config =
      declare_parameter<std::string>("lane_semantics_config", "");
    const std::string workcell_geometry = declare_parameter<std::string>("workcell_geometry", "");
    const std::string product_catalog = declare_parameter<std::string>("product_catalog", "");
    // Durable desired-stocking document, same schema as lane_semantics_config. At start-up its
    // rows overlay the shipped baseline so a restart resumes the owner's intent; at runtime it
    // is the file SetLanePolicy rewrites atomically. Empty disables both: SetLanePolicy then
    // refuses rather than making a change that would not survive the next restart.
    lane_policy_state_ = declare_parameter<std::string>("lane_policy_state", "");
    if (lane_semantics_config.empty() != workcell_geometry.empty()) {
      throw std::invalid_argument(
              "lane_semantics_config and workcell_geometry must be provided together");
    }
    config.placement_growth_tolerance_m = declare_parameter<double>(
      "placement_growth_tolerance_m", config.placement_growth_tolerance_m);
    if (!std::isfinite(config.placement_growth_tolerance_m) ||
      config.placement_growth_tolerance_m < 0.0)
    {
      throw std::invalid_argument("placement_growth_tolerance_m must be finite and non-negative");
    }
    config.placement_entry_depth_tolerance_m = declare_parameter<double>(
      "placement_entry_depth_tolerance_m", config.placement_entry_depth_tolerance_m);
    if (!std::isfinite(config.placement_entry_depth_tolerance_m) ||
      config.placement_entry_depth_tolerance_m < 0.0)
    {
      throw std::invalid_argument(
              "placement_entry_depth_tolerance_m must be finite and non-negative");
    }
    config.placement_require_column_growth = declare_parameter<bool>(
      "placement_require_column_growth", config.placement_require_column_growth);
    // Depth one more of each catalogued product costs a lane. Without it no placement can be
    // admitted, since a destination cannot be shown to have room for a product of unknown size.
    if (!product_catalog.empty()) {
      if (workcell_geometry.empty()) {
        throw std::invalid_argument("product_catalog requires workcell_geometry");
      }
      config.product_lane_profiles =
        load_product_lane_profiles(product_catalog, workcell_geometry);
    }
    store_ = std::make_unique<WorldStateStore>(std::move(config));

    if (!lane_semantics_config.empty()) {
      std::vector<LaneDefinition> definitions =
        load_lane_definitions(lane_semantics_config, workcell_geometry);
      if (!lane_policy_state_.empty() &&
        std::filesystem::exists(std::filesystem::path(lane_policy_state_)))
      {
        // A first run has no state file yet and starts from the baseline; every later run
        // resumes the policy the owner left behind.
        const auto policies = load_lane_policy_state(lane_policy_state_);
        apply_lane_policy_state(definitions, policies);
        RCLCPP_INFO(
          get_logger(), "resumed %zu lane policies from %s",
          policies.size(), lane_policy_state_.c_str());
      }
      for (const auto & definition : definitions) {
        auto configured = store_->configure_lane(definition, now());
        if (!configured) {
          throw std::runtime_error(
                  "failed to configure " + definition.id.value + ": " +
                  configured.error().detail);
        }
      }
    }

    // The object confidence is `min(detection score, pose coverage)`. The detection score is the
    // mean posterior that a blob's pixels belong to their classified category (a blob on the color
    // acceptance boundary scores 0.5). Valid products measure 0.71 to 0.86 on this cell, the floor
    // set by the translucent small bottle. 0.60 sits between the two. The default matches the
    // launch file so compositions that omit the parameter (test fixtures, bare `ros2 run`) do
    // not discard every camera observation.
    minimum_confidence_ = declare_parameter<double>("minimum_confidence", 0.60);
    if (!std::isfinite(minimum_confidence_) || minimum_confidence_ < 0.0 ||
      minimum_confidence_ > 1.0)
    {
      throw std::invalid_argument("minimum_confidence must be finite and in [0, 1]");
    }
    // The wrist-camera lane producer reports confidence as coverage: the fraction of the lane's
    // depth, from the rear entrance to the nearest measured surface, that the acquisition
    // accounted for. Valid readings score 1.0, a blank frame 0.0, a half-blinded frame 0.46, so
    // the producer's own minimum_coverage of 0.90 sits in the empty gap. The floor matches it.
    // It cannot be zero: "no return inside the window" reads as an empty lane, and coverage
    // separates "empty" from "could not see". Do not reuse the object gate's 0.60, which is a
    // detection posterior.
    //
    // This is a trust floor, not a provenance gate; publisher authentication is. The comparison
    // is `>=` at wire-type precision (see `confidence_meets_floor` in ros_conversions.cpp).
    minimum_lane_confidence_ = declare_parameter<double>("minimum_lane_confidence", 0.90);
    if (!std::isfinite(minimum_lane_confidence_) || minimum_lane_confidence_ < 0.0 ||
      minimum_lane_confidence_ > 1.0)
    {
      throw std::invalid_argument("minimum_lane_confidence must be finite and in [0, 1]");
    }
    observation_topic_ =
      declare_parameter<std::string>("observation_topic", "/perception/object_observations");
    // The sensor-driven default: the wrist camera survey's topic, matching baseline.launch.py
    // (Milestone 10 Stage 7). A survey is required before the first transfer either way. The
    // ground-truth path pins lane_observation_topic to
    // /perception/ground_truth/lane_observations explicitly.
    lane_observation_topic_ = declare_parameter<std::string>(
      "lane_observation_topic", "/perception/lane_observations");
    robot_telemetry_topic_ =
      declare_parameter<std::string>("robot_telemetry_topic", "/robot_telemetry");
    const auto robot_telemetry_qos_depth =
      declare_parameter<std::int64_t>("robot_telemetry_qos_depth", 10);
    if (robot_telemetry_topic_.empty() || robot_telemetry_qos_depth <= 0) {
      throw std::invalid_argument("robot telemetry topic must be nonempty and QoS depth positive");
    }
    if (observation_topic_.empty()) {
      throw std::invalid_argument("observation_topic must be nonempty");
    }
    if (lane_observation_topic_.empty()) {
      throw std::invalid_argument("lane_observation_topic must be nonempty");
    }
    const std::string snapshot_service =
      declare_parameter<std::string>("snapshot_service", "/world_state/get_snapshot");

    // Register before any ingress can admit temporal authority. Static lane configuration above
    // is not evidence and does not arm inhibition. The clock can already be active at this point.
    clock_authority_ = std::make_unique<ClockAuthority>(*store_, get_clock());

    observation_subscription_ = create_subscription<restocker_interfaces::msg::ObjectObservation>(
      observation_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(
        &WorldStateNode::on_observation, this, std::placeholders::_1,
        std::placeholders::_2));
    lane_observation_subscription_ =
      create_subscription<restocker_interfaces::msg::LaneObservation>(
      lane_observation_topic_, rclcpp::QoS(rclcpp::KeepLast(20)).reliable(),
      std::bind(
        &WorldStateNode::on_lane_observation, this, std::placeholders::_1,
        std::placeholders::_2));
    robot_telemetry_subscription_ = create_subscription<restocker_interfaces::msg::RobotTelemetry>(
      robot_telemetry_topic_,
      rclcpp::QoS(rclcpp::KeepLast(static_cast<std::size_t>(robot_telemetry_qos_depth)))
      .reliable(),
      std::bind(
        &WorldStateNode::on_robot_telemetry, this, std::placeholders::_1,
        std::placeholders::_2));
    snapshot_service_ = create_service<restocker_interfaces::srv::GetWorldState>(
      snapshot_service, std::bind(
        &WorldStateNode::on_snapshot_request, this,
        std::placeholders::_1, std::placeholders::_2));
    invalidate_lane_service_ = create_service<restocker_interfaces::srv::InvalidateLaneEvidence>(
      "/world_state/invalidate_lane_evidence",
      std::bind(
        &WorldStateNode::on_invalidate_lane_evidence, this, std::placeholders::_1,
        std::placeholders::_2));
    set_lane_policy_service_ = create_service<restocker_interfaces::srv::SetLanePolicy>(
      "/world_state/set_lane_policy", std::bind(
        &WorldStateNode::on_set_lane_policy, this,
        std::placeholders::_1, std::placeholders::_2));
    reserve_service_ = create_service<restocker_interfaces::srv::ReserveTask>(
      "/world_state/reserve_task", std::bind(
        &WorldStateNode::on_reserve_task, this,
        std::placeholders::_1, std::placeholders::_2));
    validate_service_ = create_service<restocker_interfaces::srv::ValidateTaskReservation>(
      "/world_state/validate_reservation",
      std::bind(
        &WorldStateNode::on_validate_reservation, this, std::placeholders::_1,
        std::placeholders::_2));
    execution_authority_service_ =
      create_service<restocker_interfaces::srv::ValidateExecutionWorldAuthority>(
      "/world_state/validate_execution_authority",
      std::bind(
        &WorldStateNode::on_validate_execution_authority, this, std::placeholders::_1,
        std::placeholders::_2));
    checkpoint_service_ = create_service<restocker_interfaces::srv::CheckpointTaskState>(
      "/world_state/checkpoint_task", std::bind(
        &WorldStateNode::on_checkpoint_task, this,
        std::placeholders::_1, std::placeholders::_2));
    attachment_service_ = create_service<restocker_interfaces::srv::CommitReservedAttachment>(
      "/world_state/commit_attachment", std::bind(
        &WorldStateNode::on_commit_attachment, this,
        std::placeholders::_1, std::placeholders::_2));
    detachment_service_ = create_service<restocker_interfaces::srv::CommitReservedDetachment>(
      "/world_state/commit_detachment", std::bind(
        &WorldStateNode::on_commit_detachment, this,
        std::placeholders::_1, std::placeholders::_2));
    release_service_ = create_service<restocker_interfaces::srv::ReleaseTaskReservation>(
      "/world_state/release_reservation",
      std::bind(
        &WorldStateNode::on_release_reservation, this, std::placeholders::_1,
        std::placeholders::_2));
    parameter_callback_handle_ =
      add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = std::all_of(
          parameters.begin(), parameters.end(), [](const rclcpp::Parameter & parameter) {
            return parameter.get_name().starts_with("qos_overrides.");
          });
        if (!result.successful) {
          result.reason = "world-state runtime parameters are immutable";
        }
        return result;
      });
    // Liveness of this executor. The snapshot service shares one single-threaded executor with
    // every observation, lane and telemetry callback, and the planning-scene projector abandons a
    // snapshot that does not answer inside its deadline; a wall timer's own lateness is therefore
    // the measurement that separates "the handler is slow" from "this process was not scheduled",
    // and it is reported only when it happens.
    executor_heartbeat_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&WorldStateNode::on_executor_heartbeat, this));
    RCLCPP_INFO(
      get_logger(),
      "accepting object observations on %s, lane observations on %s, robot "
      "telemetry on %s, and serving snapshots on %s",
      observation_topic_.c_str(), lane_observation_topic_.c_str(),
      robot_telemetry_topic_.c_str(), snapshot_service.c_str());
  }

private:
  void on_observation(
    const restocker_interfaces::msg::ObjectObservation::SharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto publisher_count = count_publishers(observation_topic_);
    if (publisher_count != 1U) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "object observation rejected: expected exactly one publisher, observed %zu",
        publisher_count);
      return;
    }
    const auto & publisher_gid = message_info.get_rmw_message_info().publisher_gid;
    std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> observed_publisher_gid{};
    std::copy_n(publisher_gid.data, observed_publisher_gid.size(), observed_publisher_gid.begin());
    std::scoped_lock lock(observation_writer_mutex_);
    if ((observation_publisher_gid_ && *observation_publisher_gid_ != observed_publisher_gid) ||
      (observation_backend_name_ && *observation_backend_name_ != message->backend_name))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "object observation rejected: publisher or backend identity changed");
      return;
    }

    auto converted = object_observation_from_message(*message, minimum_confidence_);
    if (!converted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "observation contract rejection: %s",
        converted.error().detail.c_str());
      return;
    }
    auto receipt = clock_authority_->with_sample(
      [&](const AuthorityClockSample & sample) {
        return store_->observe_object(converted.value(), sample.time, sample);
      });
    if (!receipt) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "world-state observation rejection: %s", receipt.error().detail.c_str());
      return;
    }
    if (!observation_publisher_gid_) {
      observation_publisher_gid_ = observed_publisher_gid;
      observation_backend_name_ = message->backend_name;
      RCLCPP_INFO(
        get_logger(), "latched object observation backend '%s' at revision %" PRIu64,
        message->backend_name.c_str(), receipt.value().revision);
    }
  }

  void on_lane_observation(
    const restocker_interfaces::msg::LaneObservation::SharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto publisher_count = count_publishers(lane_observation_topic_);
    if (publisher_count != 1U) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lane observation rejected: expected exactly one publisher, observed %zu",
        publisher_count);
      return;
    }
    const auto & publisher_gid = message_info.get_rmw_message_info().publisher_gid;
    std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> observed_publisher_gid{};
    std::copy_n(publisher_gid.data, observed_publisher_gid.size(), observed_publisher_gid.begin());
    std::scoped_lock lock(lane_writer_mutex_);
    if ((lane_publisher_gid_ && *lane_publisher_gid_ != observed_publisher_gid) ||
      (lane_backend_name_ && *lane_backend_name_ != message->backend_name))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lane observation rejected: publisher or backend identity changed");
      return;
    }

    auto converted = lane_observation_from_message(*message, minimum_lane_confidence_);
    if (!converted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lane observation contract rejection: %s",
        converted.error().detail.c_str());
      return;
    }
    const auto snapshot = store_->snapshot();
    const auto lane = snapshot.lanes.find(converted.value().id);
    if (lane == snapshot.lanes.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "lane observation rejection: unconfigured lane %s",
        converted.value().id.value.c_str());
      return;
    }
    auto receipt = clock_authority_->with_sample(
      [&](const AuthorityClockSample & sample) {
        return store_->update_lane(converted.value(), sample.time, lane->second.revision, sample);
      });
    if (!receipt) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "world-state lane observation rejection: %s",
        receipt.error().detail.c_str());
      return;
    }
    if (!lane_publisher_gid_) {
      lane_publisher_gid_ = observed_publisher_gid;
      lane_backend_name_ = message->backend_name;
      RCLCPP_INFO(
        get_logger(), "latched lane observation backend '%s' at revision %" PRIu64,
        message->backend_name.c_str(), receipt.value().revision);
    }
  }

  void on_invalidate_lane_evidence(
    const restocker_interfaces::srv::InvalidateLaneEvidence::Request::SharedPtr request,
    restocker_interfaces::srv::InvalidateLaneEvidence::Response::SharedPtr response)
  {
    if (request->lane_id.empty() || request->expected_lane_revision == 0) {
      response->status = operation_status_from_error(
        WorldStateError{
          WorldStateErrorCode::InvalidArgument,
          "lane id and expected revision are required to invalidate evidence"});
      response->world_revision = store_->snapshot().revision;
      return;
    }
    auto receipt = store_->invalidate_lane_evidence(
      LaneId{request->lane_id}, now(), request->expected_lane_revision);
    if (!receipt) {
      response->status = operation_status_from_error(receipt.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = receipt.value().revision;
  }

  // Applies one lane's new policy through the store's revision fence, which persists the full
  // table before anything in memory changes: a stale expected_lane_revision or a failed write
  // leaves both the lane and the policy file exactly as they were.
  void on_set_lane_policy(
    const restocker_interfaces::srv::SetLanePolicy::Request::SharedPtr request,
    restocker_interfaces::srv::SetLanePolicy::Response::SharedPtr response)
  {
    const auto product_class = bounded_lane_policy_class(request->expected_product_class);
    if (request->lane_id.empty() || request->expected_lane_revision == 0 || !product_class) {
      response->status = operation_status_from_error(
        WorldStateError{
          WorldStateErrorCode::InvalidArgument,
          "lane id, expected revision, and a known product class are required to set policy"});
      response->world_revision = store_->snapshot().revision;
      return;
    }
    if (request->has_expected_sku && request->expected_sku.empty()) {
      response->status = operation_status_from_error(
        WorldStateError{
          WorldStateErrorCode::InvalidArgument,
          "expected_sku cannot be empty when has_expected_sku is set"});
      response->world_revision = store_->snapshot().revision;
      return;
    }
    if (!request->has_expected_sku && !request->expected_sku.empty()) {
      response->status = operation_status_from_error(
        WorldStateError{
          WorldStateErrorCode::InvalidArgument,
          "expected_sku must be empty when has_expected_sku is unset"});
      response->world_revision = store_->snapshot().revision;
      return;
    }
    if (lane_policy_state_.empty()) {
      response->status = operation_status_from_error(
        WorldStateError{
          WorldStateErrorCode::InvalidArgument,
          "lane_policy_state is not configured, so a policy change could not be persisted"});
      response->world_revision = store_->snapshot().revision;
      return;
    }
    LanePolicy policy;
    policy.id = LaneId{request->lane_id};
    policy.expected_product_class = *product_class;
    if (request->has_expected_sku) {
      policy.expected_sku = request->expected_sku;
    }
    policy.target_count = request->target_count;
    const std::filesystem::path policy_path(lane_policy_state_);
    const auto receipt = store_->set_lane_policy(
      policy, request->expected_lane_revision, now(),
      [&policy_path](const std::vector<LanePolicy> & table) {
        try {
          save_lane_policy_state(policy_path, table);
          return true;
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            rclcpp::get_logger("world_state"),
            "lane policy persistence to %s failed: %s",
            policy_path.string().c_str(), error.what());
          return false;
        }
      });
    if (!receipt) {
      response->status = operation_status_from_error(receipt.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = receipt.value().revision;
  }

  [[nodiscard]] static std::optional<ProductClass> bounded_lane_policy_class(std::uint8_t value)
  {
    switch (value) {
      case restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_CAN:
        return ProductClass::Can;
      case restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_SMALL_BOTTLE:
        return ProductClass::SmallBottle;
      case restocker_interfaces::msg::ShelfLane::PRODUCT_CLASS_LARGE_BOTTLE:
        return ProductClass::LargeBottle;
      default:
        return std::nullopt;
    }
  }

  void on_robot_telemetry(
    const restocker_interfaces::msg::RobotTelemetry::SharedPtr message,
    const rclcpp::MessageInfo & message_info)
  {
    const auto publisher_count = count_publishers(robot_telemetry_topic_);
    if (publisher_count != 1U) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "robot telemetry rejected: expected exactly one publisher, observed %zu",
        publisher_count);
      return;
    }
    const auto & publisher_gid = message_info.get_rmw_message_info().publisher_gid;
    std::array<std::uint8_t, RMW_GID_STORAGE_SIZE> observed_publisher_gid{};
    std::copy_n(publisher_gid.data, observed_publisher_gid.size(), observed_publisher_gid.begin());
    std::scoped_lock lock(telemetry_writer_mutex_);
    if ((telemetry_publisher_gid_ && *telemetry_publisher_gid_ != observed_publisher_gid) ||
      (telemetry_source_id_ && *telemetry_source_id_ != message->source_id))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "robot telemetry rejected: publisher or normalized source identity changed");
      return;
    }

    auto converted = robot_telemetry_from_message(*message);
    if (!converted) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "robot telemetry contract rejection: %s",
        converted.error().detail.c_str());
      return;
    }
    auto receipt = clock_authority_->with_sample(
      [&](const AuthorityClockSample & sample) {
        return store_->observe_robot_telemetry(converted.value(), sample.time, sample);
      });
    if (!receipt) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "world-state robot telemetry rejection: %s",
        receipt.error().detail.c_str());
      return;
    }
    if (!telemetry_publisher_gid_) {
      telemetry_publisher_gid_ = observed_publisher_gid;
      telemetry_source_id_ = message->source_id;
      RCLCPP_INFO(
        get_logger(), "latched robot telemetry source '%s' at revision %" PRIu64,
        message->source_id.c_str(), receipt.value().revision);
    }
  }

  void on_snapshot_request(
    const restocker_interfaces::srv::GetWorldState::Request::SharedPtr request,
    restocker_interfaces::srv::GetWorldState::Response::SharedPtr response)
  {
    const auto started = std::chrono::steady_clock::now();
    response->snapshot = snapshot_to_message(
      store_->snapshot(), SnapshotMessageOptions{planning_frame_, now(), request->include_removed,
        request->include_events});
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - started);
    ++snapshot_requests_;
    if (elapsed > worst_snapshot_ms_) {
      worst_snapshot_ms_ = elapsed;
    }
    // The projector's own deadline is 2.0 s by default, so a snapshot handler worth knowing
    // about is one that spent a visible part of it. Throttled: a persistently slow handler
    // reports once per window with the worst time seen in that window.
    if (elapsed >= slow_snapshot_report_ms_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "world snapshot served in %.1f ms (worst %.1f ms since the last report, %.0f served; "
        "objects=%zu)",
        static_cast<double>(elapsed.count()),
        static_cast<double>(worst_snapshot_ms_.count()),
        static_cast<double>(snapshot_requests_),
        response->snapshot.objects.size());
      worst_snapshot_ms_ = std::chrono::milliseconds::zero();
    }
  }

  void on_executor_heartbeat()
  {
    const auto fired = std::chrono::steady_clock::now();
    if (last_heartbeat_fire_.time_since_epoch().count() != 0) {
      const auto lateness = std::chrono::duration_cast<std::chrono::milliseconds>(
        fired - last_heartbeat_fire_ - kExecutorHeartbeatPeriod);
      if (lateness > std::chrono::milliseconds::zero()) {
        if (lateness > worst_heartbeat_lateness_) {
          worst_heartbeat_lateness_ = lateness;
        }
        if (lateness >= kExecutorHeartbeatPeriod) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "world-state executor lagged %.1f ms behind its %.1f ms heartbeat (worst %.1f ms "
            "since the last report; %.0f snapshots served)",
            static_cast<double>(lateness.count()),
            static_cast<double>(
              std::chrono::duration_cast<std::chrono::milliseconds>(
                kExecutorHeartbeatPeriod).count()),
            static_cast<double>(worst_heartbeat_lateness_.count()),
            static_cast<double>(snapshot_requests_));
          worst_heartbeat_lateness_ = std::chrono::milliseconds::zero();
        }
      }
    }
    last_heartbeat_fire_ = fired;
  }

  void on_reserve_task(
    const restocker_interfaces::srv::ReserveTask::Request::SharedPtr request,
    restocker_interfaces::srv::ReserveTask::Response::SharedPtr response)
  {
    const auto converted = reserve_task_request_from_message(*request);
    if (!converted) {
      response->status = operation_status_from_error(converted.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const auto result = clock_authority_->with_sample(
      [&](const AuthorityClockSample & sample) {
        return store_->reserve_task(converted.value(), sample.time, sample);
      });
    if (!result) {
      response->status = operation_status_from_error(result.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = result.value().revision;
    response->token = result.value().token;
    response->reservation = task_reservation_to_message(result.value().reservation);
  }

  void on_validate_reservation(
    const restocker_interfaces::srv::ValidateTaskReservation::Request::SharedPtr request,
    restocker_interfaces::srv::ValidateTaskReservation::Response::SharedPtr response)
  {
    const auto result = store_->validate_task_reservation(request->token);
    if (!result) {
      response->status = operation_status_from_error(result.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = result.value().revision;
    response->has_reservation = true;
    response->reservation = task_reservation_to_message(result.value().reservation);
  }

  void on_validate_execution_authority(
    const restocker_interfaces::srv::ValidateExecutionWorldAuthority::Request::SharedPtr request,
    restocker_interfaces::srv::ValidateExecutionWorldAuthority::Response::SharedPtr response)
  {
    const ExecutionWorldAuthorityExpectation expected{
      request->expected_reservation_id, request->expected_reservation_revision,
      ObjectId{request->expected_object_id}, LaneId{request->expected_destination_lane_id}};
    const auto result = store_->validate_execution_world_authority(request->token, expected);
    if (!result) {
      response->status = operation_status_from_error(result.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const auto & proof = result.value();
    response->status = operation_status_ok();
    response->world_revision = proof.revision;
    response->planning_frame = proof.planning_frame;
    response->has_proof = true;
    response->reservation = task_reservation_to_message(proof.reservation);
    response->object = tracked_object_to_message(proof.object);
    response->destination_lane = shelf_lane_to_message(proof.destination_lane);
    response->has_source_lane = proof.source_lane.has_value();
    if (proof.source_lane) {
      response->source_lane = shelf_lane_to_message(*proof.source_lane);
    }
    response->robot = robot_execution_state_to_message(proof.robot);
  }

  void on_checkpoint_task(
    const restocker_interfaces::srv::CheckpointTaskState::Request::SharedPtr request,
    restocker_interfaces::srv::CheckpointTaskState::Response::SharedPtr response)
  {
    const auto converted = checkpoint_request_from_message(*request, now());
    if (!converted) {
      response->status = operation_status_from_error(converted.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const auto result = store_->checkpoint_reserved_task(converted.value());
    populate_reservation_response(result, response);
  }

  void on_commit_attachment(
    const restocker_interfaces::srv::CommitReservedAttachment::Request::SharedPtr request,
    restocker_interfaces::srv::CommitReservedAttachment::Response::SharedPtr response)
  {
    const auto converted = attachment_request_from_message(*request);
    if (!converted) {
      response->status = operation_status_from_error(converted.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const auto result = clock_authority_->with_sample(
      [&](const AuthorityClockSample & sample) {
        return store_->commit_reserved_attachment(converted.value(), sample.time, sample);
      });
    *response = attachment_result_to_message(result, store_->snapshot().revision);
    if (!result) {
      report_attachment_temporal_refusal(result.error());
    }
  }

  void report_attachment_temporal_refusal(const WorldStateError & error) noexcept
  {
    if (!error.attachment_temporal_refusal) {
      return;
    }
    const auto & decision = *error.attachment_temporal_refusal;
    // The store has unlocked. Log only the captured decision, never a newer snapshot or time.
    // Availability flags distinguish absent provenance from a captured zero.
    // No capability is logged.
    try {
      const auto operation_hex = decision.operation_id.hex_prefix();
      const auto source_hex = decision.source_id.hex_prefix();
      RCLCPP_WARN(
        get_logger(),
        "attachment temporal refusal: committed_ns=%" PRId64
        " object_observed_ns=%" PRId64 " object_transition_ns=%" PRId64
        " robot_telemetry_ns=%" PRId64 " object_postdates_commit=%d robot_postdates_commit=%d"
        " world_revision=%" PRIu64 " object_id=%" PRIu64 " object_revision=%" PRIu64
        " robot_revision=%" PRIu64 " robot_telemetry_revision=%" PRIu64
        " reservation_id=%" PRIu64 " reservation_revision=%" PRIu64 " reservation_stage=%u"
        " object_receipt_available=%d object_receipt_ns=%" PRId64
        " object_lineage_available=%d object_clock_lineage=%" PRIu64
        " commit_lineage_available=%d commit_clock_lineage=%" PRIu64
        " object_managed_ros_started=%d commit_managed_ros_started=%d"
        " decision_kind=%u clock_inhibited=%d authority_clock_lineage=%" PRIu64
        " operation_id_bytes=%" PRIu64 " operation_id_hex=%s"
        " source_id_bytes=%" PRIu64 " source_id_truncated=%d source_id_prefix_hex=%s",
        decision.committed_at_ns, decision.object_observation_ns, decision.object_transition_ns,
        decision.robot_telemetry_ns, static_cast<int>(decision.object_postdates_commit),
        static_cast<int>(decision.robot_postdates_commit), decision.world_revision,
        decision.object_id.value, decision.object_revision, decision.robot_revision,
        decision.robot_telemetry_revision, decision.reservation_id, decision.reservation_revision,
        static_cast<unsigned int>(decision.reservation_stage),
        static_cast<int>(decision.object_received_at_ns.has_value()),
        decision.object_received_at_ns.value_or(0),
        static_cast<int>(decision.object_clock_lineage.has_value()),
        decision.object_clock_lineage.value_or(0),
        static_cast<int>(decision.commit_clock_lineage.has_value()),
        decision.commit_clock_lineage.value_or(0),
        static_cast<int>(decision.object_managed_ros_started),
        static_cast<int>(decision.commit_managed_ros_started),
        static_cast<unsigned int>(decision.decision_kind),
        static_cast<int>(decision.clock_inhibited), decision.authority_clock_lineage,
        decision.operation_id.original_size, operation_hex.data(),
        decision.source_id.original_size, static_cast<int>(decision.source_id.truncated()),
        source_hex.data());
    } catch (...) {
      // The original refusal response is already populated; logging cannot change its outcome.
    }
  }

  void on_commit_detachment(
    const restocker_interfaces::srv::CommitReservedDetachment::Request::SharedPtr request,
    restocker_interfaces::srv::CommitReservedDetachment::Response::SharedPtr response)
  {
    const auto converted = detachment_request_from_message(*request);
    if (!converted) {
      response->status = operation_status_from_error(converted.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const rclcpp::Time committed_at = now();
    const auto result = store_->commit_reserved_detachment(converted.value(), committed_at);
    if (result && result.value().reservation.placed_in_destination) {
      report_accepted_placement(result.value().reservation.destination_lane, committed_at);
    }
    populate_reservation_response(result, response);
  }

  // Logs the destination evidence age when a placement is accepted, as refusals already do.
  void report_accepted_placement(const LaneId & destination, const rclcpp::Time & committed_at)
  {
    const auto snapshot = store_->snapshot();
    const auto lane = snapshot.lanes.find(destination);
    if (lane == snapshot.lanes.end()) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "placement accepted into lane %s on evidence aged %" PRId64
      " ms (last_verified_ns=%" PRId64 ", committed_ns=%" PRId64 ")",
      destination.value.c_str(),
      (committed_at.nanoseconds() - lane->second.last_verified.nanoseconds()) / 1'000'000,
      lane->second.last_verified.nanoseconds(), committed_at.nanoseconds());
  }

  void on_release_reservation(
    const restocker_interfaces::srv::ReleaseTaskReservation::Request::SharedPtr request,
    restocker_interfaces::srv::ReleaseTaskReservation::Response::SharedPtr response)
  {
    const auto converted = release_request_from_message(*request);
    if (!converted) {
      response->status = operation_status_from_error(converted.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    const auto result = store_->release_task_reservation(converted.value(), now());
    if (!result) {
      response->status = operation_status_from_error(result.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = result.value().revision;
  }

  template<typename Response>
  void populate_reservation_response(
    const Result<ReservationReceipt> & result,
    const std::shared_ptr<Response> & response)
  {
    if (!result) {
      response->status = operation_status_from_error(result.error());
      response->world_revision = store_->snapshot().revision;
      return;
    }
    response->status = operation_status_ok();
    response->world_revision = result.value().revision;
    response->has_reservation = true;
    response->reservation = task_reservation_to_message(result.value().reservation);
  }

  std::string planning_frame_;
  std::string observation_topic_;
  std::string robot_telemetry_topic_;
  std::string lane_observation_topic_;
  std::string lane_policy_state_;
  double minimum_confidence_{0.60};
  double minimum_lane_confidence_{0.90};
  std::unique_ptr<WorldStateStore> store_;
  // Declared after the store and before ingress: reverse destruction removes ingress, then the
  // jump handler (which drains a managed pre-hook), while store and explicit clock owner live.
  // main's single-threaded spin has drained application callbacks before node destruction.
  std::unique_ptr<ClockAuthority> clock_authority_;
  rclcpp::Subscription<restocker_interfaces::msg::ObjectObservation>::SharedPtr
    observation_subscription_;
  rclcpp::Subscription<restocker_interfaces::msg::LaneObservation>::SharedPtr
    lane_observation_subscription_;
  rclcpp::Subscription<restocker_interfaces::msg::RobotTelemetry>::SharedPtr
    robot_telemetry_subscription_;
  std::mutex observation_writer_mutex_;
  std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> observation_publisher_gid_;
  std::optional<std::string> observation_backend_name_;
  std::mutex telemetry_writer_mutex_;
  std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> telemetry_publisher_gid_;
  std::optional<std::string> telemetry_source_id_;
  std::mutex lane_writer_mutex_;
  std::optional<std::array<std::uint8_t, RMW_GID_STORAGE_SIZE>> lane_publisher_gid_;
  std::optional<std::string> lane_backend_name_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rclcpp::Service<restocker_interfaces::srv::GetWorldState>::SharedPtr snapshot_service_;
  rclcpp::Service<restocker_interfaces::srv::InvalidateLaneEvidence>::SharedPtr
    invalidate_lane_service_;
  rclcpp::Service<restocker_interfaces::srv::SetLanePolicy>::SharedPtr set_lane_policy_service_;
  rclcpp::Service<restocker_interfaces::srv::ReserveTask>::SharedPtr reserve_service_;
  rclcpp::Service<restocker_interfaces::srv::ValidateTaskReservation>::SharedPtr validate_service_;
  rclcpp::Service<restocker_interfaces::srv::ValidateExecutionWorldAuthority>::SharedPtr
    execution_authority_service_;
  rclcpp::Service<restocker_interfaces::srv::CheckpointTaskState>::SharedPtr checkpoint_service_;
  rclcpp::Service<restocker_interfaces::srv::CommitReservedAttachment>::SharedPtr
    attachment_service_;
  rclcpp::Service<restocker_interfaces::srv::CommitReservedDetachment>::SharedPtr
    detachment_service_;
  rclcpp::Service<restocker_interfaces::srv::ReleaseTaskReservation>::SharedPtr release_service_;
  // Measurement only: the snapshot duration worth reporting and the heartbeat period (see the
  // two methods above).
  static constexpr std::chrono::milliseconds slow_snapshot_report_ms_{100};
  static constexpr std::chrono::milliseconds kExecutorHeartbeatPeriod{250};
  rclcpp::TimerBase::SharedPtr executor_heartbeat_timer_;
  std::chrono::steady_clock::time_point last_heartbeat_fire_{};
  std::chrono::milliseconds worst_heartbeat_lateness_{0};
  std::uint64_t snapshot_requests_{0U};
  std::chrono::milliseconds worst_snapshot_ms_{0};
};

}  // namespace restocker_world_state

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_world_state::WorldStateNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("world_state"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
