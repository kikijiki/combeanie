// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/restock_action_coordinator_node.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <rclcpp/create_timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/action/restock_product.hpp>
#include <restocker_interfaces/msg/restock_coordinator_status.hpp>
#include <restocker_interfaces/srv/acquire_lane_observation.hpp>
#include <restocker_interfaces/srv/invalidate_lane_evidence.hpp>
#include <restocker_perception/survey_stations.hpp>
#include <restocker_perception/viewpoint_geometry.hpp>
#include <restocker_reasoner/http_recovery_advisor.hpp>
#include <restocker_reasoner/recovery_audit_log.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include "restocker_task_executor/action_result_publisher.hpp"
#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"
#include "restocker_task_executor/coordinator_async_steady_clock_failure_latch.hpp"
#include "restocker_task_executor/coordinator_generation_quiescence.hpp"
#include "restocker_task_executor/coordinator_handoff_policy.hpp"
#include "restocker_task_executor/coordinator_handoff_reducer.hpp"
#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"
#include "restocker_task_executor/coordinator_startup_transport.hpp"
#include "restocker_task_executor/coordinator_steady_clock.hpp"
#include "restocker_task_executor/coordinator_termination_router.hpp"
#include "restocker_task_executor/moveit_motion_port.hpp"
#include "restocker_task_executor/ros_attachment_port.hpp"
#include "restocker_task_executor/ros_gripper_port.hpp"
#include "restocker_task_executor/gripper_geometry.hpp"
#include "restocker_task_executor/manipulation_geometry.hpp"
#include "restocker_task_executor/restock_action_contract.hpp"
#include "restocker_task_executor/restock_coordinator_driver.hpp"
#include "restocker_task_executor/scene_geometry.hpp"
#include "restocker_task_executor/task_selection.hpp"
#include "restocker_task_executor/world_state_async_port.hpp"
#include "restocker_world_state/ros_conversions.hpp"
#include "detail/accepted_handle_adoption.hpp"
#include "detail/pending_activation_validator.hpp"
#include "detail/pending_route_claimant.hpp"
#include "detail/pending_route_executor.hpp"
#include "detail/pump_transaction_runner.hpp"

namespace restocker_task_executor
{
namespace
{

using Action = restocker_interfaces::action::RestockProduct;
using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
using GoalHandlePtr = std::shared_ptr<GoalHandle>;
using AcceptedHandleAdoption = detail::BoundedAcceptedHandleAdoption<GoalHandlePtr>;
using AcceptedHandleAdoptionStatus = detail::AcceptedHandleAdoptionStatus;
using PendingRouteMetadata = detail::PendingRouteMetadata;
using PendingRouteMetadataSlots = detail::PendingRouteMetadataSlots;
using PendingRouteExecutionWork = detail::PendingRouteExecutionWork;
using PendingRouteExecutionCompletion = detail::PendingRouteExecutionCompletion;
using namespace std::chrono_literals;
constexpr double kHalfPi = 1.57079632679489661923;
constexpr std::string_view kPendingSteadyClockFailureDetail =
  "coordinator steady-clock provider failed during accepted handoff";

[[nodiscard]] constexpr std::string_view accepted_handle_adoption_detail(
  AcceptedHandleAdoptionStatus status, bool identity_extraction_failed) noexcept
{
  if (identity_extraction_failed) {
    return "accepted action handle identity extraction failed; handle retained as orphan";
  }
  switch (status) {
    case AcceptedHandleAdoptionStatus::kAdoptedPending:
      return "accepted action handle adopted by its pending goal";
    case AcceptedHandleAdoptionStatus::kDuplicatePending:
      return "duplicate accepted action handle matched its pending goal";
    case AcceptedHandleAdoptionStatus::kNullHandleFailStop:
      return "accepted-goal callback received a null handle";
    case AcceptedHandleAdoptionStatus::kAdoptedFirstOrphan:
      return "accepted action handle has no matching pending admission";
    case AcceptedHandleAdoptionStatus::kAdoptedSecondDistinctOrphanFailStop:
      return "second distinct orphaned accepted action handle retained";
    case AcceptedHandleAdoptionStatus::kDuplicateOrphan:
      return "duplicate orphaned accepted action handle observed";
    case AcceptedHandleAdoptionStatus::kPendingIdentityMismatchFailStop:
      return "accepted action handle identity conflicts with its pending binding";
  }
  return "unknown accepted action handle adoption status";
}

[[nodiscard]] std::chrono::milliseconds positive_milliseconds(
  rclcpp::Node & node,
  const std::string & name,
  std::int64_t default_value)
{
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return std::chrono::milliseconds(value);
}

// Like positive_milliseconds, but zero is a meaningful setting: it disables the bounded hold
// and keeps the immediate refusal.
[[nodiscard]] std::chrono::milliseconds nonnegative_milliseconds(
  rclcpp::Node & node,
  const std::string & name,
  std::int64_t default_value)
{
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value < 0) {
    throw std::invalid_argument(name + " must be nonnegative");
  }
  return std::chrono::milliseconds(value);
}

[[nodiscard]] std::size_t positive_size(
  rclcpp::Node & node, const std::string & name,
  std::int64_t default_value, std::size_t minimum = 1U)
{
  const auto value = node.declare_parameter<std::int64_t>(name, default_value);
  if (value < 0 ||
    static_cast<std::uint64_t>(value) >
    static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()) ||
    static_cast<std::size_t>(value) < minimum)
  {
    throw std::invalid_argument(name + " is outside its supported positive range");
  }
  return static_cast<std::size_t>(value);
}

[[nodiscard]] std::optional<std::chrono::milliseconds> checked_add(
  std::chrono::milliseconds left,
  std::chrono::milliseconds right)
{
  if (right.count() > std::chrono::milliseconds::max().count() - left.count()) {
    return std::nullopt;
  }
  return left + right;
}

[[nodiscard]] std::chrono::milliseconds simulation_elapsed(
  const rclcpp::Time & started,
  const rclcpp::Time & now)
{
  if (started.get_clock_type() != now.get_clock_type() || now < started) {
    throw std::runtime_error("ROS simulation time regressed during an active restock goal");
  }
  return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::nanoseconds((now - started).nanoseconds()));
}

[[nodiscard]] bool valid_uuid(const CoordinatorGoalId & goal_id) noexcept
{
  return std::any_of(
    goal_id.begin(), goal_id.end(),
    [](std::uint8_t value) {return value != 0U;});
}

[[nodiscard]] std::shared_ptr<ActionResultPublisher> require_action_result_publisher(
  std::shared_ptr<ActionResultPublisher> publisher)
{
  if (!publisher) {
    throw std::invalid_argument("action result publisher must not be null");
  }
  return publisher;
}

[[nodiscard]] constexpr std::optional<ActionTerminalKind> action_terminal_kind(
  ActiveTerminalClassification classification) noexcept
{
  switch (classification) {
    case ActiveTerminalClassification::kOriginalCanceled:
      return std::optional{ActionTerminalKind::kCanceled};
    case ActiveTerminalClassification::kOriginalAborted:
    case ActiveTerminalClassification::kInternalFaultAbort:
      return std::optional{ActionTerminalKind::kAborted};
    case ActiveTerminalClassification::kOriginalSucceeded:
      return std::optional{ActionTerminalKind::kSucceeded};
  }
  return std::nullopt;
}

[[nodiscard]] std::string_view invalid_goal_selection_detail(const Action::Goal & goal) noexcept
{
  if ((goal.has_object_id && goal.object_id == 0U) ||
    (!goal.has_object_id && goal.object_id != 0U))
  {
    return "object selector presence and value are inconsistent";
  }
  if ((goal.has_lane_id && goal.lane_id.empty()) || (!goal.has_lane_id && !goal.lane_id.empty())) {
    return "lane selector presence and value are inconsistent";
  }
  return {};
}

[[nodiscard]] bool valid_output_contract(const CoordinatorDriverOutput & output) noexcept
{
  switch (output.kind) {
    case CoordinatorDriverOutputKind::kFeedback:
      return output.outcome == RestockActionOutcome::kUnset;
    case CoordinatorDriverOutputKind::kCanceled:
      return output.outcome == RestockActionOutcome::kCanceled;
    case CoordinatorDriverOutputKind::kAborted:
      return output.outcome != RestockActionOutcome::kUnset &&
             output.outcome != RestockActionOutcome::kSucceeded &&
             output.outcome != RestockActionOutcome::kCanceled;
    case CoordinatorDriverOutputKind::kSucceeded:
      return output.outcome == RestockActionOutcome::kSucceeded;
    case CoordinatorDriverOutputKind::kInhibited:
      return output.outcome == RestockActionOutcome::kExternalInconsistency;
  }
  return false;
}

[[nodiscard]] bool exact_termination_latched(
  const GoalTerminationLatchDecision & decision,
  const CoordinatorGoalId & goal_id,
  GoalGeneration generation) noexcept
{
  const auto record = decision.record();
  return (decision.status() == GoalTerminationLatchStatus::kLatched ||
         decision.status() == GoalTerminationLatchStatus::kAlreadyLatched) &&
         record &&
         validate_goal_termination_record(*record) == GoalTerminationValidationError::kNone &&
         record->goal_id == goal_id && record->goal_generation == generation;
}

[[nodiscard]] bool rigid_transform(const Eigen::Isometry3d & transform) noexcept
{
  const auto & rotation = transform.linear();
  return transform.matrix().allFinite() &&
         (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1.0e-9) &&
         std::abs(rotation.determinant() - 1.0) <= 1.0e-9;
}

[[nodiscard]] bool transform_differs(
  const Eigen::Isometry3d & reference,
  const Eigen::Isometry3d & candidate,
  double translation_tolerance_m,
  double rotation_tolerance_rad) noexcept
{
  const double translation_error = (reference.translation() - candidate.translation()).norm();
  const Eigen::Matrix3d rotation_error = reference.linear().transpose() * candidate.linear();
  const double rotation_error_rad = Eigen::AngleAxisd(rotation_error).angle();
  return translation_error > translation_tolerance_m || rotation_error_rad > rotation_tolerance_rad;
}

[[nodiscard]] std::uint8_t startup_status_to_message(CoordinatorStartupStatus status) noexcept
{
  using Message = restocker_interfaces::msg::RestockCoordinatorStatus;
  switch (status) {
    case CoordinatorStartupStatus::kWaitingForAuthority:
      return Message::STARTUP_WAITING_FOR_AUTHORITY;
    case CoordinatorStartupStatus::kProbePending:
      return Message::STARTUP_PROBE_PENDING;
    case CoordinatorStartupStatus::kReady:
      return Message::STARTUP_READY;
    case CoordinatorStartupStatus::kOrphanedReservation:
      return Message::STARTUP_ORPHANED_RESERVATION;
    case CoordinatorStartupStatus::kFaulted:
      return Message::STARTUP_FAULTED;
  }
  return Message::STARTUP_FAULTED;
}

class SteadyClockFailureObserver final
{
public:
  using Callback = std::function<bool (const CoordinatorSteadyClockSample &)>;

  explicit SteadyClockFailureObserver(Callback callback)
  : callback_(std::move(callback)) {}

  [[nodiscard]] bool notify(const CoordinatorSteadyClockSample & sample) noexcept
  {
    std::lock_guard lock(mutex_);
    if (!callback_) {
      return false;
    }
    try {
      return callback_(sample);
    } catch (...) {
      // The async caller deposits an invalid-time sentinel; a later pump sees the sticky failure
      // and retries fail-closed routing.
      return false;
    }
  }

  void detach() noexcept
  {
    std::lock_guard lock(mutex_);
    callback_ = {};
  }

private:
  std::mutex mutex_;
  Callback callback_;
};

[[nodiscard]] SteadyTime sample_async_evidence_time(
  const std::shared_ptr<CoordinatorSteadyClock> & clock,
  const std::shared_ptr<SteadyClockFailureObserver> & observer) noexcept
{
  const auto sample = clock->sample();
  if (!sample.provider_failed) {
    return sample.time;
  }
  // Latch matching admission authority before the callback can deposit completion evidence.
  // The sentinel prevents forward handling even if routing faults.
  return observer->notify(sample) ? sample.time : SteadyTime::max();
}

[[nodiscard]] SteadyTime sample_driver_evidence_time(
  const std::shared_ptr<CoordinatorSteadyClock> & clock,
  const std::shared_ptr<CoordinatorAsyncSteadyClockFailureLatch> & failure_latch) noexcept
{
  const auto sample = clock->sample();
  if (!sample.provider_failed) {
    return sample.time;
  }
  try {
    (void)failure_latch->record_failure(sample);
  } catch (...) {
    // The sentinel still withholds ordinary evidence; a later pump sees the sticky clock failure
    // even if this latch record could not be stored.
  }
  return SteadyTime::max();
}

static_assert(
  noexcept(sample_driver_evidence_time(
    std::declval<const std::shared_ptr<CoordinatorSteadyClock> &>(),
    std::declval<const std::shared_ptr<CoordinatorAsyncSteadyClockFailureLatch> &>())));
static_assert(std::atomic<bool>::is_always_lock_free);
static_assert(std::atomic<const char *>::is_always_lock_free);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorDriverOutput>);
static_assert(
  noexcept(std::declval<std::optional<CoordinatorDriverOutput> &>().emplace(
    std::declval<CoordinatorDriverOutput &&>())));

}  // namespace

class RestockActionCoordinatorNode::Impl
{
public:
  Impl(RestockActionCoordinatorNode & node, RestockActionCoordinatorNodeDependencies dependencies)
  : node_(node),
    termination_router_(admission_),
    steady_clock_(std::make_shared<CoordinatorSteadyClock>(std::move(dependencies.steady_now))),
    async_clock_failure_latch_(
      std::make_shared<CoordinatorAsyncSteadyClockFailureLatch>()),
    clock_failure_observer_(std::make_shared<SteadyClockFailureObserver>(
        [this](const auto & sample) {return observe_steady_clock_failure(sample, true);})),
    after_pending_goal_reserved_(std::move(dependencies.after_pending_goal_reserved)),
    after_terminal_delivery_reserved_(std::move(dependencies.after_terminal_delivery_reserved)),
    before_active_fault_route_(std::move(dependencies.before_active_fault_route)),
    before_accepted_goal_adoption_(std::move(dependencies.before_accepted_goal_adoption)),
    before_pending_handoff_(std::move(dependencies.before_pending_handoff)),
    after_callback_failure_(std::move(dependencies.after_callback_failure)),
    action_result_publisher_(
      require_action_result_publisher(std::move(dependencies.action_result_publisher)))
  {
    load_configuration();
    immutable_parameters_callback_ =
      node_.add_on_set_parameters_callback(
      [](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful =
        std::all_of(
          parameters.begin(), parameters.end(), [](const auto & parameter) {
            return parameter.get_name().rfind("qos_overrides.", 0U) == 0U;
          });
        if (!result.successful) {
          result.reason = "restock coordinator parameters are immutable after startup";
        }
        return result;
      });
    ingress_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    // rclcpp_action sends a goal response before it registers the goal, and answers a result
    // request for an unregistered goal with STATUS_UNKNOWN. Serialising the action server's
    // goal, cancel and result requests keeps a result request from overtaking admission, so a
    // client only ever receives the terminal result this coordinator delivers (Milestone 10 §6
    // result finality, Card 070).
    action_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    orchestration_group_ =
      node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    completion_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_.get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, &node_, false);
    world_state_ = std::make_unique<WorldStateAsyncPort>(node_, completion_group_, service_names_);
    startup_gate_ = std::make_unique<CoordinatorStartupGate>(startup_config_);
    startup_mailbox_ = std::make_shared<CoordinatorStartupMailbox>();
    startup_status_publisher_ =
      node_.create_publisher<restocker_interfaces::msg::RestockCoordinatorStatus>(
      status_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    if (motion_enabled_) {
      // MoveGroupInterface reads the robot model from its own node, so forward this node's
      // parameter overrides (robot_description and friends come from the launch file).
      rclcpp::NodeOptions motion_options;
      motion_options.automatically_declare_parameters_from_overrides(true);
      motion_options.parameter_overrides(node_.get_node_options().parameter_overrides());
      MoveItMotionPortConfig motion_config;
      motion_config.planning_frame = planning_frame_;
      motion_config.require_planning_scene_authority = require_planning_scene_authority_;
      motion_ = std::make_unique<MoveItMotionPort>(motion_options, motion_config);
      // Gripper and attachment boundaries are part of the same capability: moving without being
      // able to grasp would stall mid-task.
      rclcpp::NodeOptions manipulation_options;
      manipulation_options.automatically_declare_parameters_from_overrides(true);
      gripper_ = std::make_unique<RosGripperPort>(manipulation_options);
      attachment_ = std::make_unique<RosAttachmentPort>(manipulation_options);
      RCLCPP_INFO(
        node_.get_logger(),
        "motion, gripper and attachment backends composed for manipulation");
    } else {
      RCLCPP_WARN(
        node_.get_logger(),
        "motion backend disabled; the coordinator will hold staged candidates at PlanPreGrasp");
    }

    {
      rclcpp::SubscriptionOptions options;
      options.callback_group = ingress_group_;
      // Liveness tracks depth alone: CameraInfo can outlive a dead depth stream, and counting it
      // would leave motion authorised past a killed camera.
      const auto depth_qos = reliable_wrist_depth_qos_ ?
        rclcpp::QoS(rclcpp::KeepLast(5)).reliable() : rclcpp::QoS(rclcpp::SensorDataQoS());
      wrist_depth_subscription_ = node_.create_subscription<sensor_msgs::msg::Image>(
        wrist_depth_topic_, depth_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
          note_perception_acquisition(message->header.stamp);
        },
        options);
    }
    invalidate_lane_client_ =
      node_.create_client<restocker_interfaces::srv::InvalidateLaneEvidence>(
      invalidate_lane_evidence_service_, rclcpp::ServicesQoS(), completion_group_);
    acquire_lane_client_ =
      node_.create_client<restocker_interfaces::srv::AcquireLaneObservation>(
      acquire_lane_observation_service_, rclcpp::ServicesQoS(), completion_group_);
    if (reasoner_enabled_) {
      // Optional: the driver treats a null advisor and one that never answers identically.
      if (!reasoner_audit_path_.empty()) {
        recovery_audit_ =
          std::make_unique<restocker_reasoner::RecoveryAuditLog>(reasoner_audit_path_);
      }
      recovery_advisor_ =
        std::make_unique<restocker_reasoner::HttpRecoveryAdvisor>(reasoner_config_);
      RCLCPP_INFO(
        node_.get_logger(),
        "advisory recovery reasoner composed at %s with a %" PRId64 " ms deadline; it may only "
        "decline a recovery the deterministic authorisation had already permitted",
        reasoner_config_.describe().c_str(),
        static_cast<std::int64_t>(reasoner_config_.deadline.count()));
      if (!reasoner_audit_path_.empty() && !recovery_audit_->open()) {
        RCLCPP_WARN(
          node_.get_logger(), "reasoner audit log '%s' could not be opened for append",
          reasoner_audit_path_.c_str());
      }
    }
    driver_ = std::make_unique<RestockCoordinatorDriver>(
      admission_, *world_state_,
      [this](const auto & snapshot, const auto & request) {
        if (!world_from_shelf_) {
          return SelectionResult<SelectedTaskPair>::failure(
            SelectionError{SelectionErrorCode::InvalidConfiguration,
              "planning-frame to shelf transform is unavailable"});
        }
        auto config = selection_config_;
        config.now = node_.now();
        {
          std::lock_guard lock(perception_mutex_);
          config.last_perception_acquisition = last_perception_acquisition_;
        }
        return select_task_pair(
          snapshot, *product_catalog_, workcell_geometry_,
          *world_from_shelf_, config, request);
      },
      [this](const auto & snapshot) {
        if (!tool0_from_grasp_center_) {
          return GraspCandidateResult<GraspGenerationAuthority>::failure(
            GraspCandidateError{GraspCandidateErrorCode::InvalidToolTransform,
              "verified tool0 to grasp-center transform is unavailable"});
        }
        auto config = grasp_generation_config_;
        config.current_rail_position_m = snapshot.robot.rail_position;
        return GraspCandidateResult<GraspGenerationAuthority>::success(
          GraspGenerationAuthority{gripper_geometry_.jaw, *tool0_from_grasp_center_, config});
      },
      [this](const auto & snapshot, const auto & selection, const auto & authority,
      const auto & now) {
        return generate_grasp_candidate_batch(
          snapshot, selection, authority.gripper, authority.tool0_from_grasp_center,
          authority.config, now, driver_config_.maximum_object_age,
          driver_config_.maximum_robot_age, driver_config_.maximum_future_skew);
      },
      [clock = steady_clock_]() {
        return clock->sample().time;
      },
      [this]() {return node_.now();}, driver_config_,
      [clock = steady_clock_, failure_latch = async_clock_failure_latch_]() {
        return sample_driver_evidence_time(clock, failure_latch);
      },
      [this](const auto & snapshot, const auto & selection, const auto & grasp,
      const auto & coupling, const auto & reservation) {
        return generate_destination_placement(snapshot, selection, grasp, coupling, reservation);
      },
      motion_.get(), gripper_.get(), attachment_.get(), recovery_advisor_.get(),
      recovery_audit_.get(),
      [this](const SelectedTaskPair & selection) {
        return destination_lane_retreat_tool0(selection);
      },
      [this](const std::string & lane_id, std::uint64_t lane_revision) {
        invalidate_destination_lane_evidence(lane_id, lane_revision);
      },
      [this](
        OperationCorrelation correlation, const std::string & lane_id,
        CoordinatorDestinationObservationDone done) {
        return submit_lane_observation_acquire(correlation, lane_id, std::move(done));
      },
      [this](std::string & detail) {
        return perception_stream_live(detail);
      },
      // Runs on a port's own thread: log only. A completion that never reaches the pump stalls
      // the goal until its command deadline.
      [this](const std::string & detail) {
        RCLCPP_ERROR(node_.get_logger(), "%s", detail.c_str());
      });

    action_server_ = rclcpp_action::create_server<Action>(
      node_.get_node_base_interface(), node_.get_node_clock_interface(),
      node_.get_node_logging_interface(), node_.get_node_waitables_interface(), action_name_,
      [this](const auto & uuid, const auto goal) {
        try {
          return handle_goal(uuid, goal);
        } catch (...) {
          record_callback_failure("goal admission callback failed");
          return rclcpp_action::GoalResponse::REJECT;
        }
      },
      [this](const auto goal_handle) {
        try {
          return handle_cancel(goal_handle);
        } catch (...) {
          record_callback_failure("goal cancellation callback failed");
          return rclcpp_action::CancelResponse::REJECT;
        }
      },
      [this](auto goal_handle) noexcept {handle_accepted(std::move(goal_handle));},
      rcl_action_server_get_default_options(), action_group_);

    timer_ = node_.create_wall_timer(
      pump_period_,
      [this]() {
        try {
          pump();
        } catch (...) {
          record_callback_failure("coordinator pump callback failed");
        }
      },
      orchestration_group_);
    RCLCPP_INFO(
      node_.get_logger(), "pre-motion restock action coordinator available at %s",
      action_name_.c_str());
    RCLCPP_INFO(
      node_.get_logger(), "authoritative gripper geometry loaded from %s",
      gripper_geometry_path_.c_str());
    publish_startup_status();
  }

  ~Impl()
  {
    clock_failure_observer_->detach();
    bool unclean = false;
    {
      std::lock_guard lock(binding_mutex_);
      unclean = pending_ || active_ || accepted_handle_adoption_.fail_stopped() ||
        preaccept_transaction_fault_;
    }
    if (unclean) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "restock coordinator destroyed without clean drain; external authority may remain");
    }
  }

  [[nodiscard]] std::size_t executor_threads() const noexcept {return executor_threads_;}

  void request_shutdown()
  {
    const auto clock_sample = steady_clock_->sample();
    const auto arrived_at = clock_sample.time;
    shutdown_started_.store(true, std::memory_order_release);
    retire_startup_for_shutdown();
    admission_.update_readiness(false, "coordinator shutdown is in progress");
    if (clock_sample.provider_failed) {
      (void)observe_steady_clock_failure(clock_sample);
      std::lock_guard lock(binding_mutex_);
      if (shutdown_status_ == CoordinatorShutdownStatus::kRunning) {
        shutdown_status_ = CoordinatorShutdownStatus::kDraining;
        shutdown_deadline_ = arrived_at + shutdown_timeout_;
      }
      return;
    }
    std::optional<CoordinatorActiveFaultEpochSnapshot> observed_active_epoch;
    {
      std::lock_guard lock(binding_mutex_);
      observed_active_epoch = active_epoch_snapshot_locked();
    }
    std::optional<GoalTerminationSnapshotDecision> observed_termination;
    std::shared_ptr<const FirstGoalTerminationRecord> observed_termination_record;
    if (observed_active_epoch) {
      try {
        observed_termination.emplace(
          admission_.snapshot_first_termination(
            observed_active_epoch->goal_id, observed_active_epoch->goal_generation));
        observed_termination_record = observed_termination->record();
      } catch (...) {
        observed_termination.reset();
        observed_termination_record.reset();
      }
    }
    const bool observed_existing_cancel_or_drain = observed_active_epoch &&
      observed_termination &&
      observed_termination->status() == GoalTerminationSnapshotStatus::kPresent &&
      observed_termination_record &&
      validate_goal_termination_record(*observed_termination_record) ==
      GoalTerminationValidationError::kNone &&
      observed_termination_record->goal_id == observed_active_epoch->goal_id &&
      observed_termination_record->goal_generation == observed_active_epoch->goal_generation &&
      (observed_termination_record->kind == GoalTerminationKind::kUserCancel ||
      observed_termination_record->kind == GoalTerminationKind::kCoordinatorDrain);
    constexpr std::string_view detail = "coordinator process shutdown requested";
    std::optional<CoordinatorActiveFaultClaim> drain_claim;
    std::optional<PendingRouteExecutionWork> pending_execution;
    bool pending_diagnostic_mutation_fenced = false;
    bool drain_diagnostic_mutation_fenced = false;
    try {
      std::string owned_work_detail{detail};
      std::string owned_metadata_detail{detail};
      {
        std::lock_guard lock(binding_mutex_);
        if (shutdown_status_ != CoordinatorShutdownStatus::kRunning) {
          return;
        }
        shutdown_status_ = CoordinatorShutdownStatus::kDraining;
        shutdown_deadline_ = arrived_at + shutdown_timeout_;
        invalidate_diagnostic_caches_locked();
        if (pending_) {
          (void)claim_pending_termination_locked(
            pending_->key, PendingRouteKind::kDrain, arrived_at, owned_work_detail,
            owned_metadata_detail, pending_execution, pending_diagnostic_mutation_fenced);
        } else if (!active_) {
          return;
        } else if (!active_->epoch) {
          adapter_fail_stopped_ = true;
          return;
        } else {
          const auto epoch = active_->epoch->snapshot();
          if (observed_existing_cancel_or_drain && observed_active_epoch &&
            epoch.goal_id == observed_active_epoch->goal_id &&
            epoch.goal_generation == observed_active_epoch->goal_generation &&
            epoch.route_phase == ActiveFaultRoutePhase::kClear)
          {
            return;
          }
          auto decision = active_->epoch->claim_fault(
            ::restocker_task_executor::ActiveTerminationRouteKind::kDrain);
          drain_claim = decision.take_claim();
          if (!drain_claim) {
            return;
          }
          invalidate_diagnostic_caches_locked();
          drain_diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
          if (!drain_diagnostic_mutation_fenced) {
            (void)complete_epoch_fault_locked(*drain_claim, false);
            adapter_fail_stopped_ = true;
            return;
          }
        }
      }
    } catch (...) {
      std::lock_guard lock(binding_mutex_);
      adapter_fail_stopped_ = true;
      if (shutdown_status_ == CoordinatorShutdownStatus::kRunning) {
        shutdown_status_ = CoordinatorShutdownStatus::kDraining;
        shutdown_deadline_ = arrived_at + shutdown_timeout_;
      }
      if (drain_claim && drain_claim->live()) {
        (void)complete_epoch_fault_locked(*drain_claim, false);
      } else if (pending_) {
        enter_pending_cleanup_locked();
      } else if (active_ && active_->epoch) {
        (void)active_->epoch->invalidate_fault(
          ActiveFaultInvalidationKind::kAdapterInvariantFailure);
        invalidate_diagnostic_caches_locked();
      }
      return;
    }
    if (pending_execution && pending_diagnostic_mutation_fenced) {
      execute_pending_termination(
        std::move(*pending_execution),
        pending_diagnostic_mutation_fenced);
      return;
    }
    if (!drain_claim) {
      return;
    }
    const bool secured = complete_claimed_epoch_drain(
      *drain_claim, arrived_at, detail, drain_diagnostic_mutation_fenced);
    if (steady_clock_->provider_failed()) {
      (void)observe_steady_clock_failure(steady_clock_->sample());
    }
    if (!secured) {
      std::lock_guard lock(binding_mutex_);
      adapter_fail_stopped_ = true;
    }
  }

  [[nodiscard]] CoordinatorShutdownStatus shutdown_status() const noexcept
  {
    std::lock_guard lock(binding_mutex_);
    return shutdown_status_;
  }

  [[nodiscard]] CoordinatorShutdownSnapshot shutdown_snapshot() const noexcept
  {
    std::lock_guard lock(binding_mutex_);
    CoordinatorShutdownSnapshot snapshot;
    snapshot.status = shutdown_status_;
    snapshot.deadline = shutdown_deadline_;
    const auto active_epoch = active_epoch_snapshot_locked();
    snapshot.goal_generation = active_epoch ? active_epoch->goal_generation :
      (pending_ ? pending_->generation : cached_goal_generation_);
    snapshot.reservation_capability_may_remain = cached_driver_.reservation_capability_may_remain ||
      cached_admission_.mutation_submission_occupied;
    return snapshot;
  }

  [[nodiscard]] CoordinatorStartupSnapshot startup_snapshot() const
  {
    std::lock_guard lock(startup_mutex_);
    return startup_gate_->snapshot();
  }

  [[nodiscard]] CoordinatorSteadyClockFailureSnapshot steady_clock_failure_snapshot() const
  {
    // Preserve the node-wide clock-before-binding lock order used by orchestration paths.
    const bool provider_failed = steady_clock_->provider_failed();
    std::lock_guard lock(binding_mutex_);
    const bool first_observed_asynchronously = steady_clock_failure_observation_recorded_ ?
      steady_clock_failure_first_observed_asynchronously_ :
      async_clock_failure_latch_->snapshot().has_value();
    return {provider_failed, steady_clock_failure_handled_, steady_clock_failure_secured_,
      first_observed_asynchronously};
  }

  [[nodiscard]] CoordinatorGenerationAuthoritySnapshot generation_authority_snapshot() const
  {
    CoordinatorGenerationAuthoritySnapshot snapshot;
    std::shared_ptr<CoordinatorGenerationQuiescence> pending_quiescence;
    std::shared_ptr<CoordinatorGenerationQuiescence> active_quiescence;
    {
      std::lock_guard lock(binding_mutex_);
      snapshot.configured_deposit_capacity = generation_deposit_capacity_;
      snapshot.pending_binding = pending_.has_value();
      snapshot.active_binding = active_.has_value();
      snapshot.accepted_handle_fail_stopped = accepted_handle_adoption_.fail_stopped();
      snapshot.orphaned_accepted_handle_count = accepted_handle_adoption_.orphan_count();
      snapshot.pending_epoch = pending_epoch_snapshot_locked();
      snapshot.active_epoch = active_epoch_snapshot_locked();
      snapshot.pump_lease = pump_lease_gate_.snapshot();
      snapshot.diagnostic_caches_initialized = diagnostic_caches_initialized_;
      snapshot.diagnostic_caches_fresh = diagnostic_caches_fresh_;
      snapshot.cached_inbox = cached_inbox_;
      snapshot.cached_admission = cached_admission_;
      snapshot.cached_driver = cached_driver_;
      // Card 082, §6 Half A: the allocation-free receipt, read through atomics so this view
      // works even when the allocating half of the failure path could not run.
      snapshot.callback_failure_latched = callback_failure_latched_.load(std::memory_order_acquire);
      snapshot.callback_failure_detail = callback_failure_detail_.load(std::memory_order_acquire);
      if (pending_) {
        const auto handoff = pending_->handoff.snapshot();
        snapshot.pending_handoff_phase = handoff.phase;
        snapshot.pending_route_token_live = handoff.route_work.has_value();
        snapshot.pending_handoff_work_token_live = handoff.handoff_work.has_value();
        snapshot.pending_retained_control_occupied = pending_->retained_termination.has_value();
        snapshot.pending_deferred_control_occupied = pending_->deferred_termination.has_value();
      }
      if (pending_ && pending_->accepted_handle) {
        snapshot.pending_accepted_handle_identity =
          reinterpret_cast<std::uintptr_t>(pending_->accepted_handle.get());
      }
      if (active_ && active_->handle) {
        snapshot.active_accepted_handle_identity =
          reinterpret_cast<std::uintptr_t>(active_->handle.get());
      }
      if (pending_ && pending_->epoch) {
        snapshot.pending_epoch_identity = reinterpret_cast<std::uintptr_t>(pending_->epoch.get());
      }
      if (active_ && active_->epoch) {
        snapshot.active_epoch_identity = reinterpret_cast<std::uintptr_t>(active_->epoch.get());
      }
      if (pending_) {
        pending_quiescence = pending_->quiescence;
      }
      if (active_) {
        active_quiescence = active_->quiescence;
      }
    }
    if (pending_quiescence) {
      snapshot.pending_quiescence = pending_quiescence->snapshot();
    }
    if (active_quiescence) {
      snapshot.active_quiescence = active_quiescence->snapshot();
    }
    return snapshot;
  }

private:
  struct PendingBinding
  {
    PendingBinding(
      PendingHandoffBindingKey key_value, SteadyTime steady_started_value,
      SelectionRequest && selection_request_value) noexcept
    : key(key_value),
      goal_id(key_value.goal_id),
      generation(key_value.generation),
      steady_started(steady_started_value),
      selection_request(std::move(selection_request_value)),
      handoff(key_value)
    {
    }

    PendingBinding(const PendingBinding &) = delete;
    PendingBinding(PendingBinding &&) = delete;
    PendingBinding & operator=(const PendingBinding &) = delete;
    PendingBinding & operator=(PendingBinding &&) = delete;

    PendingHandoffBindingKey key{};
    CoordinatorGoalId goal_id{};
    GoalGeneration generation{0U};
    SteadyTime steady_started{};
    SelectionRequest selection_request;
    std::optional<SteadyTime> drain_requested_at;
    std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
    std::optional<CoordinatorControlEvent> retained_termination;
    std::optional<PendingRouteKind> retained_termination_kind;
    std::optional<CoordinatorControlEvent> deferred_termination;
    std::optional<PendingRouteKind> deferred_termination_kind;
    PendingRouteMetadataSlots route_metadata;
    bool route_or_handoff_failed{false};
    bool sampled_simulation_started{false};
    rclcpp::Time simulation_started{std::int64_t{0}, RCL_ROS_TIME};
    GoalHandlePtr accepted_handle;
    std::unique_ptr<CoordinatorActiveFaultEpoch> epoch;
    std::shared_ptr<CoordinatorGenerationQuiescence> quiescence;
    PendingAcceptedHandoffMachine handoff;
  };

  struct ActiveBinding
  {
    GoalHandlePtr handle;
    rclcpp::Time simulation_started{std::int64_t{0}, RCL_ROS_TIME};
    rclcpp::Time last_simulation_time{std::int64_t{0}, RCL_ROS_TIME};
    std::chrono::milliseconds last_elapsed{0};
    std::unique_ptr<CoordinatorActiveFaultEpoch> epoch;
    std::shared_ptr<CoordinatorGenerationQuiescence> quiescence;
  };

  struct ActiveRetirementBundle
  {
    ActiveRetirementBundle(
      ActiveBinding && binding_value,
      CoordinatorGenerationQuiescenceReceipt && receipt_value,
      CoordinatorPumpLease && pump_lease_value) noexcept
    : binding(std::move(binding_value)),
      quiescence(std::move(receipt_value)),
      pump_lease(std::move(pump_lease_value))
    {
    }

    ActiveRetirementBundle(const ActiveRetirementBundle &) = delete;
    ActiveRetirementBundle(ActiveRetirementBundle &&) noexcept = default;
    ActiveRetirementBundle & operator=(const ActiveRetirementBundle &) = delete;
    ActiveRetirementBundle & operator=(ActiveRetirementBundle &&) noexcept = default;
    ~ActiveRetirementBundle() = default;

    ActiveBinding binding;
    CoordinatorGenerationQuiescenceReceipt quiescence;
    CoordinatorPumpLease pump_lease;
  };

  static_assert(!std::is_copy_constructible_v<PendingBinding>);
  static_assert(!std::is_move_constructible_v<PendingBinding>);
  static_assert(
    noexcept(std::declval<std::optional<PendingBinding> &>().emplace(
      std::declval<PendingHandoffBindingKey>(), std::declval<SteadyTime>(),
      std::declval<SelectionRequest &&>())));
  static_assert(!std::is_copy_constructible_v<ActiveBinding>);
  static_assert(std::is_nothrow_move_constructible_v<ActiveBinding>);
  static_assert(std::is_nothrow_move_assignable_v<ActiveBinding>);
  static_assert(std::is_nothrow_destructible_v<ActiveBinding>);
  static_assert(
    noexcept(
      std::declval<std::optional<ActiveBinding> &>().emplace(std::declval<ActiveBinding &&>())));
  static_assert(noexcept(std::declval<std::optional<ActiveBinding> &>().reset()));
  static_assert(!std::is_copy_constructible_v<ActiveRetirementBundle>);
  static_assert(std::is_nothrow_move_constructible_v<ActiveRetirementBundle>);
  static_assert(std::is_nothrow_move_assignable_v<ActiveRetirementBundle>);
  static_assert(std::is_nothrow_destructible_v<ActiveRetirementBundle>);
  static_assert(
    noexcept(std::declval<std::optional<ActiveRetirementBundle> &>().emplace(
      std::declval<ActiveBinding &&>(),
      std::declval<CoordinatorGenerationQuiescenceReceipt &&>(),
      std::declval<CoordinatorPumpLease &&>())));
  static_assert(noexcept(std::declval<std::optional<PendingBinding> &>().reset()));
  static_assert(
    noexcept(std::declval<std::optional<std::string> &>().swap(
      std::declval<std::optional<std::string> &>())));

  [[nodiscard]] std::optional<CoordinatorActiveFaultEpochSnapshot> pending_epoch_snapshot_locked()
  const noexcept
  {
    if (!pending_ || !pending_->epoch) {
      return std::nullopt;
    }
    return pending_->epoch->snapshot();
  }

  [[nodiscard]] std::optional<CoordinatorActiveFaultEpochSnapshot> active_epoch_snapshot_locked()
  const noexcept
  {
    if (!active_ || !active_->epoch) {
      return std::nullopt;
    }
    return active_->epoch->snapshot();
  }

  [[nodiscard]] static constexpr std::size_t pending_route_index(PendingRouteKind kind) noexcept
  {
    return detail::pending_route_metadata_index(kind);
  }

  [[nodiscard]] static constexpr CoordinatorControlKind pending_control_kind(
    PendingRouteKind kind) noexcept
  {
    return kind == PendingRouteKind::kDrain ? CoordinatorControlKind::kDrainRequested :
           kind == PendingRouteKind::kTransformAuthorityLoss ?
           CoordinatorControlKind::kAuthorityFaultSafeAbortRequested :
           CoordinatorControlKind::kShutdown;
  }

  [[nodiscard]] static constexpr GoalTerminationKind pending_route_record_kind(
    PendingRouteKind kind) noexcept
  {
    return kind == PendingRouteKind::kDrain ? GoalTerminationKind::kCoordinatorDrain :
           kind == PendingRouteKind::kTransformAuthorityLoss ?
           GoalTerminationKind::kAuthorityLoss :
           GoalTerminationKind::kProtocolFailure;
  }

  void enter_pending_cleanup_locked() noexcept
  {
    invalidate_diagnostic_caches_locked();
    if (pending_) {
      pending_->route_or_handoff_failed = true;
      pending_->handoff.enter_cleanup_only();
    }
  }

  void invalidate_diagnostic_caches_locked() noexcept
  {
    diagnostic_caches_fresh_ = false;
    if (diagnostic_cache_revision_ == std::numeric_limits<std::uint64_t>::max()) {
      adapter_fail_stopped_ = true;
      return;
    }
    ++diagnostic_cache_revision_;
  }

  [[nodiscard]] bool begin_diagnostic_external_mutation_locked() noexcept
  {
    if (diagnostic_external_mutations_ == std::numeric_limits<std::size_t>::max()) {
      adapter_fail_stopped_ = true;
      return false;
    }
    ++diagnostic_external_mutations_;
    invalidate_diagnostic_caches_locked();
    return true;
  }

  void finish_diagnostic_external_mutation_locked() noexcept
  {
    if (diagnostic_external_mutations_ == 0U) {
      adapter_fail_stopped_ = true;
      return;
    }
    --diagnostic_external_mutations_;
    invalidate_diagnostic_caches_locked();
  }

  void finish_diagnostic_external_mutation() noexcept
  {
    try {
      std::lock_guard lock(binding_mutex_);
      finish_diagnostic_external_mutation_locked();
    } catch (...) {
      // An uncompleted fence keeps cached shutdown diagnostics fail-closed.
    }
  }

  [[nodiscard]] PendingRouteClaimStatus claim_pending_termination_locked(
    const PendingHandoffBindingKey & binding, PendingRouteKind kind, SteadyTime arrived_at,
    std::string & owned_work_detail, std::string & owned_metadata_detail,
    std::optional<PendingRouteExecutionWork> & execution,
    bool & diagnostic_mutation_fenced) noexcept
  {
    if (!pending_ || pending_->key != binding) {
      return PendingRouteClaimStatus::kCleanupOnly;
    }
    const auto status = detail::PendingRouteClaimant::claim(
      pending_->handoff, pending_->key, pending_->route_metadata, kind, arrived_at,
      owned_work_detail, owned_metadata_detail, execution);
    if (status == PendingRouteClaimStatus::kCleanupOnly) {
      enter_pending_cleanup_locked();
    } else if (status == PendingRouteClaimStatus::kClaimed) {
      invalidate_diagnostic_caches_locked();
      diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
      if (!diagnostic_mutation_fenced) {
        enter_pending_cleanup_locked();
      }
    }
    return status;
  }

  void execute_pending_termination(
    PendingRouteExecutionWork && execution,
    bool diagnostic_mutation_fenced) noexcept
  {
    auto completion = detail::PendingRouteExecutor::execute(
      std::move(execution), detail::PendingRouteRouterAdapter{termination_router_});
    complete_pending_route(std::move(completion), diagnostic_mutation_fenced);
  }

  void complete_pending_route(
    PendingRouteExecutionCompletion && completion,
    bool diagnostic_mutation_fenced) noexcept
  {
    std::optional<CoordinatorControlEvent> incoming;
    std::optional<CoordinatorControlEvent> displaced;
    GoalActivationAuthorityDecision activation_authority;
    std::shared_ptr<const FirstGoalTerminationRecord> route_record;
    bool resume = false;
    try {
      auto decision = completion.take_decision();
      {
        std::lock_guard lock(binding_mutex_);
        const auto & binding = completion.work().binding();
        if (diagnostic_mutation_fenced) {
          finish_diagnostic_external_mutation_locked();
          diagnostic_mutation_fenced = false;
        }
        if (!pending_ || pending_->key != binding) {
          return;
        }
        auto evidence = PendingRouteEvidenceResult::kInvalid;
        auto merge = PendingControlMergeDecision::kCleanupOnly;
        const auto index = pending_route_index(completion.work().kind());
        activation_authority =
          admission_.observe_activation_authority(binding.goal_id, binding.generation);
        if (completion.status() == detail::PendingRouteExecutionStatus::kCompleted && decision &&
          index < pending_->route_metadata.size() && pending_->route_metadata[index] &&
          activation_authority.status == GoalActivationAuthorityStatus::kObserved &&
          activation_authority.authority)
        {
          auto & routed = *decision;
          route_record = routed.admission_decision().record();
          incoming = routed.take_event();
          const auto & metadata = *pending_->route_metadata[index];
          const bool admission_record_exact =
            route_record && activation_authority.authority->first_termination &&
            route_record.get() == activation_authority.authority->first_termination.get() &&
            validate_goal_termination_record(*route_record) ==
            GoalTerminationValidationError::kNone &&
            route_record->goal_id == binding.goal_id &&
            route_record->goal_generation == binding.generation;
          const auto latch_status = routed.admission_decision().status();
          const auto expected_record_kind = pending_route_record_kind(completion.work().kind());
          const bool latch_record_exact =
            latch_status == GoalTerminationLatchStatus::kLatched ?
            (route_record && route_record->kind == expected_record_kind &&
            route_record->arrived_at == completion.work().arrived_at()) :
            (latch_status == GoalTerminationLatchStatus::kAlreadyLatched &&
            pending_->first_termination &&
            route_record.get() == pending_->first_termination.get());
          const bool exact_event =
            incoming && incoming->kind == pending_control_kind(completion.work().kind()) &&
            incoming->goal_generation == binding.generation &&
            incoming->operation_generation == 0U &&
            incoming->arrived_at == completion.work().arrived_at() &&
            ((incoming->detail == completion.work().detail()) ||
            (incoming->detail.empty() && routed.diagnostic_degraded()));
          if ((latch_status == GoalTerminationLatchStatus::kLatched ||
            latch_status == GoalTerminationLatchStatus::kAlreadyLatched) &&
            admission_record_exact && latch_record_exact && exact_event &&
            metadata.kind == completion.work().kind() &&
            metadata.arrived_at == completion.work().arrived_at() &&
            metadata.detail == completion.work().detail())
          {
            evidence = PendingRouteEvidenceResult::kExact;
            pending_->first_termination =
              std::move(activation_authority.authority->first_termination);
            pending_->route_metadata[index]->diagnostic_degraded = routed.diagnostic_degraded();
            if (completion.work().kind() == PendingRouteKind::kDrain &&
              !pending_->drain_requested_at)
            {
              pending_->drain_requested_at = completion.work().arrived_at();
            }
            auto & slot = completion.work().delivery_class() ==
              PendingRouteDeliveryClass::kPreSeal ?
              pending_->retained_termination :
              pending_->deferred_termination;
            auto & slot_kind =
              completion.work().delivery_class() == PendingRouteDeliveryClass::kPreSeal ?
              pending_->retained_termination_kind :
              pending_->deferred_termination_kind;
            const bool duplicate = slot && slot_kind && *slot_kind == completion.work().kind() &&
              slot->kind == incoming->kind &&
              slot->goal_generation == incoming->goal_generation &&
              slot->operation_generation == incoming->operation_generation &&
              slot->arrived_at == incoming->arrived_at &&
              slot->detail == incoming->detail;
            merge = classify_pending_control_merge(
              slot_kind, completion.work().kind(),
              completion.work().delivery_class(), duplicate);
            const auto route_status =
              pending_->handoff.complete_route(completion.token(), evidence, merge);
            if (route_status == PendingRouteCompletionStatus::kMerged) {
              invalidate_diagnostic_caches_locked();
              if (completion.work().kind() == PendingRouteKind::kSteadyClockFailure) {
                steady_clock_failure_secured_ = true;
                steady_clock_failure_handled_ = true;
                if (!steady_clock_failure_observation_recorded_) {
                  steady_clock_failure_observation_recorded_ = true;
                  steady_clock_failure_first_observed_asynchronously_ = false;
                }
              }
              switch (merge) {
                case PendingControlMergeDecision::kStoreIncoming:
                  slot = std::move(incoming);
                  slot_kind = completion.work().kind();
                  break;
                case PendingControlMergeDecision::kKeepExistingDisposeIncoming:
                  break;
                case PendingControlMergeDecision::kReplaceExistingDisposeDisplaced:
                  displaced = std::move(slot);
                  slot = std::move(incoming);
                  slot_kind = completion.work().kind();
                  break;
                case PendingControlMergeDecision::kCleanupOnly:
                  break;
              }
              if (callback_failure_latched_.load(std::memory_order_acquire) &&
                completion.work().kind() == PendingRouteKind::kCallbackOrAdapterFailure &&
                completion.work().delivery_class() == PendingRouteDeliveryClass::kPreSeal &&
                pending_->first_termination->kind == GoalTerminationKind::kProtocolFailure &&
                pending_->retained_termination &&
                pending_->retained_termination->kind == CoordinatorControlKind::kShutdown)
              {
                // No driver context or motion exists yet. The exact shutdown control travels
                // in the sealed accepted envelope; permit only this generation to report its
                // fault through the normal terminal/ack path. Failed routing grants nothing.
                callback_failure_cleanup_generation_ = pending_->generation;
              }
              resume = true;
            } else if (route_status == PendingRouteCompletionStatus::kCleanupOnly) {
              enter_pending_cleanup_locked();
            }
          } else {
            (void)pending_->handoff.complete_route(
              completion.token(), evidence,
              PendingControlMergeDecision::kCleanupOnly);
            enter_pending_cleanup_locked();
          }
        } else {
          (void)pending_->handoff.complete_route(
            completion.token(), evidence,
            PendingControlMergeDecision::kCleanupOnly);
          enter_pending_cleanup_locked();
        }
      }
    } catch (...) {
      std::lock_guard lock(binding_mutex_);
      if (diagnostic_mutation_fenced) {
        finish_diagnostic_external_mutation_locked();
      }
      if (pending_ && pending_->key == completion.work().binding()) {
        const auto handoff = pending_->handoff.snapshot();
        const bool exact_route_token =
          handoff.route_work && handoff.route_work->kind == completion.work().kind() &&
          handoff.route_work->delivery_class == completion.work().delivery_class() &&
          handoff.route_work->token == completion.work().route_token();
        if (exact_route_token) {
          enter_pending_cleanup_locked();
        }
      }
      return;
    }
    if (resume) {
      advance_pending_handoff();
    }
  }

  void fail_pending_preparation(
    PendingHandoffWorkToken & token, PendingPreparationFailure failure,
    PendingRouteKind kind, SteadyTime arrived_at,
    std::string_view detail) noexcept
  {
    std::optional<PendingRouteExecutionWork> execution;
    bool diagnostic_mutation_fenced = false;
    bool resume = false;
    try {
      std::string owned_detail{detail};
      std::string metadata_detail{detail};
      {
        std::lock_guard lock(binding_mutex_);
        if (!pending_ || pending_->key != token.binding()) {
          return;
        }
        const auto index = pending_route_index(kind);
        if (index >= pending_->route_metadata.size()) {
          enter_pending_cleanup_locked();
          return;
        }
        const auto & existing = pending_->route_metadata[index];
        const auto relation =
          !existing ? PendingRouteObservationRelation::kNewDistinct :
          (existing->arrived_at == arrived_at && existing->detail == detail ?
          PendingRouteObservationRelation::kExactDuplicate :
          PendingRouteObservationRelation::kCollision);
        auto completion = pending_->handoff.complete_preparation_failure(token, failure, relation);
        if (completion.status() == PendingPreparationCompletionStatus::kCleanupOnly) {
          enter_pending_cleanup_locked();
          return;
        }
        auto route = completion.take_route_work();
        if (!route) {
          resume = completion.status() == PendingPreparationCompletionStatus::kRouteCoalesced;
        } else {
          invalidate_diagnostic_caches_locked();
          pending_->route_metadata[index].emplace(
            PendingRouteMetadata{kind, arrived_at, std::move(metadata_detail), false});
          execution.emplace(std::move(*route), arrived_at, std::move(owned_detail));
          diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
          if (!diagnostic_mutation_fenced) {
            enter_pending_cleanup_locked();
            return;
          }
        }
      }
      if (execution) {
        auto completion = detail::PendingRouteExecutor::execute(
          std::move(*execution), detail::PendingRouteRouterAdapter{termination_router_});
        complete_pending_route(std::move(completion), diagnostic_mutation_fenced);
        diagnostic_mutation_fenced = false;
      } else if (resume) {
        advance_pending_handoff();
      }
    } catch (...) {
      std::lock_guard lock(binding_mutex_);
      if (diagnostic_mutation_fenced) {
        finish_diagnostic_external_mutation_locked();
      }
      if (pending_ && pending_->key == token.binding()) {
        enter_pending_cleanup_locked();
      }
    }
  }

  [[nodiscard]] bool pending_clock_cleanup_authority_secured(
    const PendingHandoffWorkToken & token,
    SteadyTime frozen_time) const noexcept
  {
    try {
      std::lock_guard lock(binding_mutex_);
      if (!pending_ || pending_->key != token.binding() || !steady_clock_failure_handled_ ||
        !steady_clock_failure_secured_ || !pending_->first_termination)
      {
        return false;
      }
      const auto route_index = pending_route_index(PendingRouteKind::kSteadyClockFailure);
      if (route_index >= pending_->route_metadata.size() ||
        !pending_->route_metadata[route_index])
      {
        return false;
      }
      const auto handoff = pending_->handoff.snapshot();
      const auto & record = *pending_->first_termination;
      const auto & metadata = *pending_->route_metadata[route_index];
      const auto & retained = pending_->retained_termination;
      const FirstGoalTerminationRecord expected_record{GoalTerminationScope::kGoal,
        token.binding().goal_id,
        token.binding().generation,
        GoalTerminationKind::kProtocolFailure,
        MotionCancellationReason::kProtocolFailure,
        frozen_time,
        std::nullopt};
      return handoff.phase == PendingAcceptedHandoffPhase::kWaitingForPriorPumpLease &&
             !handoff.route_work && handoff.handoff_work &&
             handoff.handoff_work->kind == PendingHandoffWorkKind::kPrepareAcceptedEnvelope &&
             handoff.handoff_work->token == token.token() &&
             handoff.handoff_work->prepared_revision == token.prepared_revision() &&
             validate_goal_termination_record(record) == GoalTerminationValidationError::kNone &&
             record == expected_record && metadata.kind == PendingRouteKind::kSteadyClockFailure &&
             metadata.arrived_at == frozen_time &&
             pending_->retained_termination_kind == PendingRouteKind::kSteadyClockFailure &&
             retained && retained->kind == CoordinatorControlKind::kShutdown &&
             retained->goal_generation == token.binding().generation &&
             retained->operation_generation == 0U && retained->arrived_at == frozen_time &&
             (retained->detail == metadata.detail ||
             (retained->detail.empty() && metadata.diagnostic_degraded));
    } catch (...) {
      return false;
    }
  }

  void execute_pending_preparation(PendingHandoffWorkToken && token)
  {
    rclcpp::Time simulation_started{std::int64_t{0}, RCL_ROS_TIME};
    auto clock_sample = steady_clock_->sample();
    if (clock_sample.provider_failed &&
      !pending_clock_cleanup_authority_secured(token, clock_sample.time))
    {
      fail_pending_preparation(
        token, PendingPreparationFailure::kSteadyClockFailure,
        PendingRouteKind::kSteadyClockFailure, clock_sample.time,
        kPendingSteadyClockFailureDetail);
      return;
    }
    try {
      simulation_started = node_.now();
    } catch (...) {
      fail_pending_preparation(
        token, PendingPreparationFailure::kCallbackOrAdapterFailure,
        PendingRouteKind::kCallbackOrAdapterFailure, clock_sample.time,
        "accepted-envelope clock sampling failed");
      return;
    }
    std::optional<PendingHandoffWorkToken> publication_token;
    std::optional<CoordinatorAcceptedGoal> accepted;
    std::optional<CoordinatorControlEvent> retained;
    bool diagnostic_mutation_fenced = false;
    bool retry_preparation = false;
    {
      std::lock_guard lock(binding_mutex_);
      if (!pending_ || pending_->key != token.binding() || !pending_->accepted_handle ||
        !pending_->epoch || !pending_->quiescence)
      {
        enter_pending_cleanup_locked();
        return;
      }
      auto completion = pending_->handoff.complete_preparation(token);
      const bool publication_not_ready =
        completion.status() != PendingPreparationCompletionStatus::kPublishAcceptedEnvelope;
      if (completion.status() == PendingPreparationCompletionStatus::kRetryPreparation) {
        retry_preparation = true;
      } else if (publication_not_ready) {
        enter_pending_cleanup_locked();
        return;
      } else {
        auto claimed_publication = completion.take_handoff_work();
        if (!claimed_publication) {
          enter_pending_cleanup_locked();
          return;
        }
        publication_token.emplace(std::move(*claimed_publication));
        invalidate_diagnostic_caches_locked();
        pending_->simulation_started = simulation_started;
        pending_->sampled_simulation_started = true;
        accepted.emplace(
          CoordinatorAcceptedGoal{
            pending_->goal_id, pending_->generation, std::move(pending_->selection_request),
            pending_->steady_started, simulation_started, pending_->drain_requested_at,
            pending_->quiescence, true});
        retained = std::move(pending_->retained_termination);
        pending_->retained_termination_kind.reset();
        diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
        if (!diagnostic_mutation_fenced) {
          enter_pending_cleanup_locked();
          return;
        }
      }
    }
    if (retry_preparation) {
      advance_pending_handoff();
      return;
    }

    CoordinatorInboxDepositResult deposit;
    bool attempted = false;
    try {
      deposit = driver_->inbox()->push_accepted_goal(
        std::move(*accepted), std::move(retained),
        clock_sample.time);
      attempted = true;
    } catch (...) {
      attempted = false;
    }
    bool resume = false;
    {
      std::lock_guard lock(binding_mutex_);
      if (diagnostic_mutation_fenced) {
        finish_diagnostic_external_mutation_locked();
        diagnostic_mutation_fenced = false;
      }
      if (!pending_ || pending_->key != publication_token->binding()) {
        return;
      }
      const auto decision = attempted ? classify_accepted_envelope_deposit(deposit) :
        AcceptedEnvelopeDepositDecision::kCleanupOnly;
      const auto status =
        pending_->handoff.complete_accepted_publication(*publication_token, decision);
      if (status == PendingAcceptedPublicationStatus::kCommitted) {
        invalidate_diagnostic_caches_locked();
        resume = true;
      } else {
        enter_pending_cleanup_locked();
      }
    }
    if (resume) {
      advance_pending_handoff();
    }
  }

  void execute_pending_deferred(
    PendingHandoffWorkToken && token, CoordinatorControlEvent && event,
    bool diagnostic_mutation_fenced)
  {
    const auto arrived_at = event.arrived_at;
    CoordinatorInboxDepositResult deposit;
    bool attempted = false;
    try {
      deposit = driver_->inbox()->push(std::move(event));
      attempted = true;
    } catch (...) {
      attempted = false;
    }
    auto classified = attempted ? classify_deferred_control_deposit(deposit) :
      DeferredControlDepositDecision::kRecordInboxLossThenCleanupOnly;
    bool loss_secured = false;
    if (classified != DeferredControlDepositDecision::kDelivered) {
      try {
        const auto loss = termination_router_.record_inbox_loss(
          token.binding().goal_id, token.binding().generation, arrived_at);
        const auto record = loss.record();
        std::lock_guard lock(binding_mutex_);
        loss_secured = pending_ && pending_->key == token.binding() &&
          pending_->first_termination && record &&
          record.get() == pending_->first_termination.get() &&
          (loss.status() == GoalTerminationLatchStatus::kLatched ||
          loss.status() == GoalTerminationLatchStatus::kAlreadyLatched);
      } catch (...) {
        loss_secured = false;
      }
    }
    bool resume = false;
    {
      std::lock_guard lock(binding_mutex_);
      if (diagnostic_mutation_fenced) {
        finish_diagnostic_external_mutation_locked();
        diagnostic_mutation_fenced = false;
      }
      if (!pending_ || pending_->key != token.binding()) {
        return;
      }
      const auto completion = classified == DeferredControlDepositDecision::kDelivered ?
        PendingDeferredCompletion::kDelivered :
        (classified == DeferredControlDepositDecision::
        kRecordInboxLossThenContinueIfSecured &&
        loss_secured ?
        PendingDeferredCompletion::kInboxLossSecured :
        PendingDeferredCompletion::kInboxLossUnsecured);
      const auto status = pending_->handoff.complete_deferred_publication(token, completion);
      if (status == PendingDeferredPublicationStatus::kCompleted) {
        invalidate_diagnostic_caches_locked();
        resume = true;
      } else {
        enter_pending_cleanup_locked();
      }
    }
    if (resume) {
      advance_pending_handoff();
    }
  }

  struct PendingBindingDisposal
  {
    std::optional<SelectionRequest> selection_request;
    std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
    std::optional<CoordinatorControlEvent> retained;
    std::optional<CoordinatorControlEvent> deferred;
    std::array<std::optional<PendingRouteMetadata>, 4U> route_metadata;
  };

  static_assert(std::is_nothrow_default_constructible_v<PendingBindingDisposal>);
  static_assert(std::is_nothrow_move_constructible_v<PendingBindingDisposal>);
  static_assert(std::is_nothrow_move_assignable_v<PendingBindingDisposal>);
  static_assert(
    noexcept(std::declval<std::optional<PendingBindingDisposal> &>().emplace(
      std::declval<PendingBindingDisposal &&>())));
  static_assert(
    noexcept(std::declval<std::optional<SelectionRequest> &>().emplace(
      std::declval<SelectionRequest &&>())));
  static_assert(
    noexcept(std::declval<std::shared_ptr<const FirstGoalTerminationRecord> &>() =
    std::declval<std::shared_ptr<const FirstGoalTerminationRecord> &&>()));
  static_assert(
    noexcept(std::declval<std::optional<CoordinatorControlEvent> &>() =
    std::declval<std::optional<CoordinatorControlEvent> &&>()));
  static_assert(
    noexcept(std::declval<std::array<std::optional<PendingRouteMetadata>, 4U> &>() =
    std::declval<std::array<std::optional<PendingRouteMetadata>, 4U> &&>()));
  static_assert(std::is_nothrow_move_assignable_v<GoalActivationAuthorityDecision>);
  static_assert(std::is_nothrow_destructible_v<GoalActivationAuthorityDecision>);

  [[nodiscard]] std::optional<PendingBindingDisposal> activate_pending_locked(
    GoalActivationAuthorityDecision & activation_authority_disposal) noexcept
  {
    if (!pending_ || active_) {
      enter_pending_cleanup_locked();
      return std::nullopt;
    }
    const auto handoff = pending_->handoff.snapshot();
    activation_authority_disposal =
      admission_.observe_activation_authority(pending_->goal_id, pending_->generation);
    detail::PendingActivationSummary pending_summary;
    pending_summary.binding = pending_->key;
    pending_summary.drain_requested = pending_->drain_requested_at.has_value();
    if (pending_->drain_requested_at) {
      pending_summary.drain_requested_at = *pending_->drain_requested_at;
    }
    pending_summary.expected_first_termination = pending_->first_termination.get();
    for (std::size_t index = 0U; index < pending_->route_metadata.size(); ++index) {
      const auto & metadata = pending_->route_metadata[index];
      if (metadata) {
        pending_summary.routes[index] = {true, metadata->kind, metadata->arrived_at};
      }
    }
    const auto activation_validation = detail::validate_pending_activation_authority(
      pending_summary, activation_authority_disposal);
    const bool activation_preconditions =
      handoff.phase == PendingAcceptedHandoffPhase::kReadyToActivate && !handoff.route_work &&
      !handoff.handoff_work && !pending_->deferred_termination && pending_->accepted_handle &&
      pending_->epoch && pending_->quiescence && pending_->sampled_simulation_started &&
      pump_gate_allows_handoff(pump_lease_gate_.snapshot()) &&
      activation_validation == detail::PendingActivationValidationError::kNone;
    bool activated = false;
    if (activation_preconditions) {
      try {
        activated = admission_.activate(pending_->goal_id, pending_->generation);
      } catch (...) {
        activated = false;
      }
    }
    if (!activation_preconditions || !activated) {
      enter_pending_cleanup_locked();
      return std::nullopt;
    }
    ActiveBinding binding;
    binding.simulation_started = pending_->simulation_started;
    binding.last_simulation_time = pending_->simulation_started;
    binding.handle = std::move(pending_->accepted_handle);
    binding.epoch = std::move(pending_->epoch);
    binding.quiescence = std::move(pending_->quiescence);
    active_.emplace(std::move(binding));
    invalidate_diagnostic_caches_locked();
    PendingBindingDisposal disposal;
    disposal.first_termination = std::move(pending_->first_termination);
    disposal.retained = std::move(pending_->retained_termination);
    disposal.deferred = std::move(pending_->deferred_termination);
    disposal.route_metadata = std::move(pending_->route_metadata);
    disposal.selection_request.emplace(std::move(pending_->selection_request));
    pending_.reset();
    return disposal;
  }

  void advance_pending_handoff() noexcept
  {
    try {
      std::optional<PendingHandoffWorkToken> work;
      std::optional<CoordinatorControlEvent> deferred;
      std::optional<PendingBindingDisposal> disposal;
      GoalActivationAuthorityDecision activation_authority_disposal;
      bool diagnostic_mutation_fenced = false;
      PendingHandoffAdvanceStatus status{PendingHandoffAdvanceStatus::kWait};
      {
        std::lock_guard lock(binding_mutex_);
        if (!pending_ || (callback_failure_latched_.load(std::memory_order_acquire) &&
          (callback_failure_repeated_.load(std::memory_order_acquire) ||
          callback_failure_cleanup_generation_ != pending_->generation)))
        {
          return;
        }
        if (pending_->accepted_handle && before_pending_handoff_) {
          before_pending_handoff_();
        }
        auto decision = pending_->handoff.advance(
          {pump_lease_gate_.snapshot(), pending_->deferred_termination.has_value()});
        status = decision.status();
        if (auto claimed = decision.take_work()) {
          work.emplace(std::move(*claimed));
        }
        if (status == PendingHandoffAdvanceStatus::kPublishDeferredControl) {
          if (!work || !pending_->deferred_termination) {
            enter_pending_cleanup_locked();
            return;
          }
          deferred = std::move(pending_->deferred_termination);
          invalidate_diagnostic_caches_locked();
          pending_->deferred_termination_kind.reset();
          diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
          if (!diagnostic_mutation_fenced) {
            enter_pending_cleanup_locked();
            return;
          }
        } else if (status == PendingHandoffAdvanceStatus::kReadyToActivate) {
          disposal = activate_pending_locked(activation_authority_disposal);
        } else if (status == PendingHandoffAdvanceStatus::kCleanupOnly) {
          enter_pending_cleanup_locked();
        }
      }
      if (status == PendingHandoffAdvanceStatus::kPrepareAcceptedEnvelope && work) {
        execute_pending_preparation(std::move(*work));
      } else {
        if (status == PendingHandoffAdvanceStatus::kPublishDeferredControl && work &&
          deferred)
        {
          execute_pending_deferred(
            std::move(*work), std::move(*deferred),
            diagnostic_mutation_fenced);
          diagnostic_mutation_fenced = false;
        }
      }
      (void)disposal;
    } catch (...) {
      record_callback_failure("pending accepted-goal handoff failed");
    }
  }

  void latch_callback_failure(const char * detail) noexcept
  {
    // Callers supply only static literals. Preserve the first failure without allocating or
    // taking the binding lock, even if routing and reporting subsequently fail.
    const char * empty = nullptr;
    if (!callback_failure_detail_.compare_exchange_strong(
        empty, detail, std::memory_order_acq_rel))
    {
      callback_failure_repeated_.store(true, std::memory_order_release);
    }
    callback_failure_latched_.store(true, std::memory_order_release);
  }

  template<std::size_t N>
  void record_callback_failure(
    const char (& literal)[N],
    detail::InspectedAcceptedHandle<GoalHandlePtr> * unadopted = nullptr) noexcept
  {
    latch_callback_failure(literal);
    const std::string_view detail{literal, N - 1U};
    std::optional<CoordinatorActiveFaultClaim> callback_claim;
    std::optional<PendingRouteExecutionWork> pending_execution;
    bool pending_diagnostic_mutation_fenced = false;
    bool callback_diagnostic_mutation_fenced = false;
    bool inhibit_without_binding = false;
    try {
      if (unadopted && unadopted->handle) {
        (void)adopt_accepted_handle(*unadopted);
      }
      const auto clock_sample = steady_clock_->sample();
      if (clock_sample.provider_failed) {
        (void)observe_steady_clock_failure(clock_sample);
      }
      std::string owned_work_detail{detail};
      std::string owned_metadata_detail{detail};
      {
        std::lock_guard lock(binding_mutex_);
        if (active_) {
          if (!active_->epoch) {
            adapter_fail_stopped_ = true;
          } else {
            auto decision = active_->epoch->claim_fault(
              ::restocker_task_executor::ActiveTerminationRouteKind::kProtocolFailure);
            const auto claim_status = decision.status();
            callback_claim = decision.take_claim();
            if (claim_status == ActiveFaultClaimStatus::kClaimed) {
              if (!callback_claim || !callback_claim->live()) {
                (void)active_->epoch->invalidate_fault(
                  ActiveFaultInvalidationKind::kAdapterInvariantFailure);
                adapter_fail_stopped_ = true;
              } else {
                invalidate_diagnostic_caches_locked();
                callback_diagnostic_mutation_fenced =
                  begin_diagnostic_external_mutation_locked();
                if (!callback_diagnostic_mutation_fenced) {
                  (void)complete_epoch_fault_locked(*callback_claim, false);
                }
              }
            } else {
              const bool invalid_claim_status =
                claim_status == ActiveFaultClaimStatus::kIdentityMismatch ||
                claim_status == ActiveFaultClaimStatus::kInvalidRouteKind ||
                claim_status == ActiveFaultClaimStatus::kInvalidArgument;
              if (invalid_claim_status) {
                (void)active_->epoch->invalidate_fault(
                  ActiveFaultInvalidationKind::kAdapterInvariantFailure);
                adapter_fail_stopped_ = true;
                invalidate_diagnostic_caches_locked();
              }
            }
          }
        } else if (pending_) {
          (void)claim_pending_termination_locked(
            pending_->key, PendingRouteKind::kCallbackOrAdapterFailure, clock_sample.time,
            owned_work_detail, owned_metadata_detail, pending_execution,
            pending_diagnostic_mutation_fenced);
        } else {
          inhibit_without_binding = begin_diagnostic_external_mutation_locked();
        }
      }
      if (callback_claim) {
        const auto route_clock = steady_clock_->sample();
        (void)complete_claimed_epoch_clock(
          *callback_claim, route_clock.time, detail, callback_diagnostic_mutation_fenced);
        if (route_clock.provider_failed) {
          (void)observe_steady_clock_failure(route_clock);
        }
      }
      if (pending_execution && pending_diagnostic_mutation_fenced) {
        execute_pending_termination(
          std::move(*pending_execution),
          pending_diagnostic_mutation_fenced);
      }
      if (inhibit_without_binding) {
        try {
          admission_.inhibit(std::string(detail));
        } catch (...) {
          // Cache fencing is completed below even if the diagnostic cannot be allocated.
        }
        finish_diagnostic_external_mutation();
      }
    } catch (...) {
      // Outer ROS callback boundary: no exception may reach the executor. Keep a fail-closed
      // binding even if diagnostics cannot be prepared.
      try {
        std::lock_guard lock(binding_mutex_);
        adapter_fail_stopped_ = true;
        if (callback_claim && callback_claim->live()) {
          (void)complete_epoch_fault_locked(*callback_claim, false);
        } else if (pending_) {
          enter_pending_cleanup_locked();
        } else if (active_ && active_->epoch) {
          (void)active_->epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kAdapterInvariantFailure);
          invalidate_diagnostic_caches_locked();
        }
      } catch (...) {
        // The callback boundary remains noexcept under synchronization failure.
      }
    }
    try {
      RCLCPP_ERROR(node_.get_logger(), "%.*s", static_cast<int>(detail.size()), detail.data());
    } catch (...) {
      // Logging is part of the best-effort receipt too.
    }
    try {
      if (after_callback_failure_) {
        after_callback_failure_();
      }
    } catch (...) {
      // A test observer must not compromise the callback boundary.
    }
  }

  [[nodiscard]] bool observe_steady_clock_failure(
    const CoordinatorSteadyClockSample & sample,
    bool observed_asynchronously = false) noexcept
  {
    try {
      return handle_steady_clock_failure(sample, observed_asynchronously);
    } catch (...) {
      // Exception boundary shared by timer and async observations. The clock's sticky
      // provider-failure bit independently blocks ingress and orchestration; returning false
      // forces async completions to deposit the invalid-time sentinel.
      bool goal_bound = true;
      bool inhibit_without_binding = false;
      try {
        std::lock_guard lock(binding_mutex_);
        goal_bound = pending_.has_value() || active_.has_value();
        adapter_fail_stopped_ = true;
        if (pending_) {
          enter_pending_cleanup_locked();
        } else if (active_ && active_->epoch) {
          (void)active_->epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kAdapterInvariantFailure);
          invalidate_diagnostic_caches_locked();
        } else if (!goal_bound) {
          inhibit_without_binding = begin_diagnostic_external_mutation_locked();
        }
      } catch (...) {
        // Treat uncertain binding ownership as live capability; never hard-inhibit speculatively.
      }
      if (inhibit_without_binding) {
        try {
          admission_.inhibit(
            "steady-clock failure handling raised; coordinator remains fail-closed");
        } catch (...) {
          // The sticky clock gate remains authoritative even if diagnostics cannot be allocated.
        }
        finish_diagnostic_external_mutation();
      }
      return false;
    }
  }

  [[nodiscard]] bool handle_steady_clock_failure(
    const CoordinatorSteadyClockSample & sample,
    bool observed_asynchronously)
  {
    if (!sample.provider_failed) {
      return true;
    }
    constexpr std::string_view detail =
      "coordinator steady-clock provider failed; normal orchestration is stopped";
    std::string owned_work_detail{detail};
    std::string owned_metadata_detail{detail};
    std::optional<CoordinatorActiveFaultClaim> clock_claim;
    std::optional<PendingRouteExecutionWork> pending_execution;
    bool pending_diagnostic_mutation_fenced = false;
    bool clock_diagnostic_mutation_fenced = false;
    bool inhibit_without_binding = false;
    try {
      std::lock_guard lock(binding_mutex_);
      if (steady_clock_failure_handled_) {
        return steady_clock_failure_secured_;
      }
      if (!steady_clock_failure_observation_recorded_) {
        steady_clock_failure_first_observed_asynchronously_ =
          observed_asynchronously || async_clock_failure_latch_->snapshot().has_value();
        steady_clock_failure_observation_recorded_ = true;
      }
      if (pending_) {
        (void)claim_pending_termination_locked(
          pending_->key, PendingRouteKind::kSteadyClockFailure, sample.time, owned_work_detail,
          owned_metadata_detail, pending_execution, pending_diagnostic_mutation_fenced);
      } else if (active_) {
        if (!active_->epoch) {
          adapter_fail_stopped_ = true;
          steady_clock_failure_secured_ = false;
        } else {
          auto decision = active_->epoch->claim_fault(
            ::restocker_task_executor::ActiveTerminationRouteKind::kProtocolFailure);
          const auto claim_status = decision.status();
          clock_claim = decision.take_claim();
          switch (claim_status) {
            case ActiveFaultClaimStatus::kClaimed:
              if (!clock_claim || !clock_claim->live()) {
                (void)active_->epoch->invalidate_fault(
                  ActiveFaultInvalidationKind::kAdapterInvariantFailure);
                adapter_fail_stopped_ = true;
                steady_clock_failure_secured_ = false;
                break;
              }
              invalidate_diagnostic_caches_locked();
              clock_diagnostic_mutation_fenced =
                begin_diagnostic_external_mutation_locked();
              if (!clock_diagnostic_mutation_fenced) {
                (void)complete_epoch_fault_locked(*clock_claim, false);
                steady_clock_failure_secured_ = false;
              }
              break;
            case ActiveFaultClaimStatus::kAlreadyRouting:
              return false;
            case ActiveFaultClaimStatus::kAlreadySecured:
            case ActiveFaultClaimStatus::kTerminalWon:
              steady_clock_failure_secured_ = true;
              break;
            case ActiveFaultClaimStatus::kUnsecured:
              steady_clock_failure_secured_ = false;
              break;
            case ActiveFaultClaimStatus::kIdentityMismatch:
            case ActiveFaultClaimStatus::kInvalidRouteKind:
            case ActiveFaultClaimStatus::kInvalidArgument:
              (void)active_->epoch->invalidate_fault(
                ActiveFaultInvalidationKind::kAdapterInvariantFailure);
              adapter_fail_stopped_ = true;
              invalidate_diagnostic_caches_locked();
              steady_clock_failure_secured_ = false;
              break;
          }
        }
      } else {
        inhibit_without_binding = begin_diagnostic_external_mutation_locked();
        steady_clock_failure_secured_ = true;
      }
      if (!clock_claim && !pending_execution && !pending_) {
        steady_clock_failure_handled_ = true;
      }
    } catch (...) {
      std::lock_guard lock(binding_mutex_);
      if (clock_claim && clock_claim->live()) {
        (void)complete_epoch_fault_locked(*clock_claim, false);
      }
      throw;
    }
    if (inhibit_without_binding) {
      try {
        admission_.inhibit(std::string(detail));
      } catch (...) {
        // The sticky clock bit is the allocation-free ingress gate.
      }
      finish_diagnostic_external_mutation();
    }
    if (pending_execution && pending_diagnostic_mutation_fenced) {
      execute_pending_termination(
        std::move(*pending_execution),
        pending_diagnostic_mutation_fenced);
    }
    if (clock_claim) {
      const bool secured = complete_claimed_epoch_clock(
        *clock_claim, sample.time, detail, clock_diagnostic_mutation_fenced);
      std::lock_guard lock(binding_mutex_);
      steady_clock_failure_secured_ = secured;
      steady_clock_failure_handled_ = true;
    }
    RCLCPP_ERROR(node_.get_logger(), "%s", detail.data());
    std::lock_guard lock(binding_mutex_);
    return steady_clock_failure_secured_;
  }

  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim> claim_active_protocol_fault_locked(
    bool & diagnostic_mutation_fenced) noexcept
  {
    diagnostic_mutation_fenced = false;
    if (!active_) {
      return std::nullopt;
    }
    if (!active_->epoch) {
      adapter_fail_stopped_ = true;
      invalidate_diagnostic_caches_locked();
      return std::nullopt;
    }
    auto decision = active_->epoch->claim_fault(
      ::restocker_task_executor::ActiveTerminationRouteKind::kProtocolFailure);
    const auto claim_status = decision.status();
    auto claim = decision.take_claim();
    switch (claim_status) {
      case ActiveFaultClaimStatus::kClaimed:
        if (!claim || !claim->live()) {
          (void)active_->epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kAdapterInvariantFailure);
          adapter_fail_stopped_ = true;
          break;
        }
        invalidate_diagnostic_caches_locked();
        diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
        if (!diagnostic_mutation_fenced) {
          (void)complete_epoch_fault_locked(*claim, false);
        }
        break;
      case ActiveFaultClaimStatus::kAlreadyRouting:
      case ActiveFaultClaimStatus::kAlreadySecured:
      case ActiveFaultClaimStatus::kTerminalWon:
      case ActiveFaultClaimStatus::kUnsecured:
        break;
      case ActiveFaultClaimStatus::kIdentityMismatch:
      case ActiveFaultClaimStatus::kInvalidRouteKind:
      case ActiveFaultClaimStatus::kInvalidArgument:
        (void)active_->epoch->invalidate_fault(
          ActiveFaultInvalidationKind::kAdapterInvariantFailure);
        adapter_fail_stopped_ = true;
        invalidate_diagnostic_caches_locked();
        break;
    }
    return claim;
  }

  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim>
  complete_feedback_publication_failure_locked(
    CoordinatorFeedbackPermit & permit, GoalGeneration generation,
    bool & diagnostic_mutation_fenced) noexcept
  {
    diagnostic_mutation_fenced = false;
    std::optional<CoordinatorActiveFaultClaim> claim;
    const auto epoch_snapshot = active_epoch_snapshot_locked();
    if (active_ && active_->epoch && epoch_snapshot &&
      epoch_snapshot->goal_generation == generation)
    {
      auto failure = active_->epoch->feedback_failed(permit);
      const auto status = failure.status();
      claim = failure.take_fault_claim();
      switch (status) {
        case ActiveFeedbackCompletionStatus::kFailedAndFaultClaimed:
          if (!claim || !claim->live()) {
            (void)active_->epoch->invalidate_fault(
              ActiveFaultInvalidationKind::kAdapterInvariantFailure);
            adapter_fail_stopped_ = true;
            invalidate_diagnostic_caches_locked();
            break;
          }
          invalidate_diagnostic_caches_locked();
          diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
          if (!diagnostic_mutation_fenced) {
            (void)complete_epoch_fault_locked(*claim, false);
          }
          break;
        case ActiveFeedbackCompletionStatus::kReturned:
        case ActiveFeedbackCompletionStatus::kFailedRouteAlreadyPending:
        case ActiveFeedbackCompletionStatus::kFailedRouteAlreadySecured:
        case ActiveFeedbackCompletionStatus::kFailedRouteUnsecured:
        case ActiveFeedbackCompletionStatus::kNotLive:
        case ActiveFeedbackCompletionStatus::kIdentityMismatch:
        case ActiveFeedbackCompletionStatus::kStalePermit:
        case ActiveFeedbackCompletionStatus::kWrongPhase:
          break;
      }
    }
    return claim;
  }

  void fail_terminal_adapter_locked() noexcept
  {
    adapter_fail_stopped_ = true;
    if (!active_) {
      return;
    }
    if (active_->epoch) {
      (void)active_->epoch->invalidate_fault(
        ActiveFaultInvalidationKind::kAdapterInvariantFailure);
    }
    invalidate_diagnostic_caches_locked();
  }

  [[nodiscard]] bool prepare_terminal_fault_route_locked(
    std::optional<CoordinatorActiveFaultClaim> & claim,
    bool & diagnostic_mutation_fenced) noexcept
  {
    diagnostic_mutation_fenced = false;
    if (!active_ || !active_->epoch || !claim || !claim->live()) {
      fail_terminal_adapter_locked();
      return false;
    }
    invalidate_diagnostic_caches_locked();
    diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
    if (!diagnostic_mutation_fenced) {
      (void)complete_epoch_fault_locked(*claim, false);
      return false;
    }
    return true;
  }

  void route_claimed_terminal_fault(
    std::optional<CoordinatorActiveFaultClaim> & claim, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    if (!claim) {
      return;
    }
    const auto route_clock = steady_clock_->sample();
    (void)complete_claimed_epoch_clock(
      *claim, route_clock.time, detail, diagnostic_mutation_fenced);
    if (route_clock.provider_failed) {
      (void)observe_steady_clock_failure(route_clock);
    }
  }

  void route_claimed_feedback_failure(
    std::optional<CoordinatorActiveFaultClaim> & claim,
    bool diagnostic_mutation_fenced) noexcept
  {
    if (!claim) {
      return;
    }
    const auto route_clock = steady_clock_->sample();
    (void)complete_claimed_epoch_clock(
      *claim, route_clock.time, "action feedback publication failed",
      diagnostic_mutation_fenced);
    if (route_clock.provider_failed) {
      (void)observe_steady_clock_failure(route_clock);
    }
  }

  [[nodiscard]] bool complete_epoch_fault_locked(
    CoordinatorActiveFaultClaim & claim, bool exact_secured) noexcept
  {
    const auto epoch_snapshot = active_epoch_snapshot_locked();
    if (!active_ || !active_->epoch || !epoch_snapshot ||
      epoch_snapshot->goal_id != claim.goal_id() ||
      epoch_snapshot->goal_generation != claim.goal_generation())
    {
      return false;
    }
    const auto completion = active_->epoch->complete_fault(claim, exact_secured);
    if (completion == ActiveFaultCompletionStatus::kSecured) {
      invalidate_diagnostic_caches_locked();
      return true;
    }
    if (completion == ActiveFaultCompletionStatus::kUnsecured) {
      invalidate_diagnostic_caches_locked();
    }
    return false;
  }

  [[nodiscard]] bool complete_claimed_epoch_cancel(
    CoordinatorActiveFaultClaim & claim, SteadyTime arrived_at, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    using EpochRouteKind = ::restocker_task_executor::ActiveTerminationRouteKind;
    bool secured = false;
    try {
      if (before_active_fault_route_) {
        before_active_fault_route_();
      }
      std::optional<CoordinatorTerminationRouteDecision> route;
      switch (claim.kind()) {
        case EpochRouteKind::kCancel:
          route = termination_router_.route_cancel(
            claim.goal_id(), claim.goal_generation(), arrived_at, 0U, detail);
          break;
        case EpochRouteKind::kDrain:
        case EpochRouteKind::kTransformAuthorityLoss:
        case EpochRouteKind::kSimulationTimeAuthorityLoss:
        case EpochRouteKind::kProtocolFailure:
          break;
      }
      if (route) {
        const auto status = route->admission_decision().status();
        auto event = route->take_event();
        if (status == GoalTerminationLatchStatus::kAlreadyLatched) {
          secured = exact_termination_latched(
            route->admission_decision(), claim.goal_id(), claim.goal_generation());
        } else if (status == GoalTerminationLatchStatus::kLatched && event) {
          const auto pushed = driver_->inbox()->push(std::move(*event));
          if (pushed.status == CoordinatorInboxDepositStatus::kAccepted &&
            pushed.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
          {
            secured = true;
          } else {
            const auto loss = termination_router_.record_inbox_loss(
              claim.goal_id(), claim.goal_generation(), arrived_at);
            secured = exact_termination_latched(
              loss, claim.goal_id(), claim.goal_generation());
          }
        }
      }
    } catch (...) {
      secured = false;
    }
    std::lock_guard lock(binding_mutex_);
    if (diagnostic_mutation_fenced) {
      finish_diagnostic_external_mutation_locked();
    }
    return complete_epoch_fault_locked(claim, secured);
  }

  [[nodiscard]] bool complete_claimed_epoch_drain(
    CoordinatorActiveFaultClaim & claim, SteadyTime arrived_at, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    using EpochRouteKind = ::restocker_task_executor::ActiveTerminationRouteKind;
    bool secured = false;
    try {
      if (before_active_fault_route_) {
        before_active_fault_route_();
      }
      std::optional<CoordinatorTerminationRouteDecision> route;
      switch (claim.kind()) {
        case EpochRouteKind::kDrain:
          route = termination_router_.route_drain(
            claim.goal_id(), claim.goal_generation(), arrived_at, 0U, detail);
          break;
        case EpochRouteKind::kCancel:
        case EpochRouteKind::kTransformAuthorityLoss:
        case EpochRouteKind::kSimulationTimeAuthorityLoss:
        case EpochRouteKind::kProtocolFailure:
          break;
      }
      if (route) {
        const auto status = route->admission_decision().status();
        auto event = route->take_event();
        if (status == GoalTerminationLatchStatus::kAlreadyLatched) {
          secured = exact_termination_latched(
            route->admission_decision(), claim.goal_id(), claim.goal_generation());
        } else if (status == GoalTerminationLatchStatus::kLatched && event) {
          const auto pushed = driver_->inbox()->push(std::move(*event));
          if (pushed.status == CoordinatorInboxDepositStatus::kAccepted &&
            pushed.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
          {
            secured = true;
          } else {
            const auto loss = termination_router_.record_inbox_loss(
              claim.goal_id(), claim.goal_generation(), arrived_at);
            secured = exact_termination_latched(
              loss, claim.goal_id(), claim.goal_generation());
          }
        }
      }
    } catch (...) {
      secured = false;
    }
    std::lock_guard lock(binding_mutex_);
    if (diagnostic_mutation_fenced) {
      finish_diagnostic_external_mutation_locked();
    }
    return complete_epoch_fault_locked(claim, secured);
  }

  [[nodiscard]] bool complete_claimed_epoch_transform(
    CoordinatorActiveFaultClaim & claim, SteadyTime arrived_at, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    using EpochRouteKind = ::restocker_task_executor::ActiveTerminationRouteKind;
    bool secured = false;
    try {
      if (before_active_fault_route_) {
        before_active_fault_route_();
      }
      std::optional<CoordinatorTerminationRouteDecision> route;
      switch (claim.kind()) {
        case EpochRouteKind::kTransformAuthorityLoss:
          route = termination_router_.route_transform_authority_loss(
            claim.goal_id(), claim.goal_generation(), arrived_at, 0U, detail);
          break;
        case EpochRouteKind::kCancel:
        case EpochRouteKind::kDrain:
        case EpochRouteKind::kSimulationTimeAuthorityLoss:
        case EpochRouteKind::kProtocolFailure:
          break;
      }
      if (route) {
        const auto status = route->admission_decision().status();
        auto event = route->take_event();
        if (status == GoalTerminationLatchStatus::kAlreadyLatched) {
          secured = exact_termination_latched(
            route->admission_decision(), claim.goal_id(), claim.goal_generation());
        } else if (status == GoalTerminationLatchStatus::kLatched && event) {
          const auto pushed = driver_->inbox()->push(std::move(*event));
          if (pushed.status == CoordinatorInboxDepositStatus::kAccepted &&
            pushed.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
          {
            secured = true;
          } else {
            const auto loss = termination_router_.record_inbox_loss(
              claim.goal_id(), claim.goal_generation(), arrived_at);
            secured = exact_termination_latched(
              loss, claim.goal_id(), claim.goal_generation());
          }
        }
      }
    } catch (...) {
      secured = false;
    }
    std::lock_guard lock(binding_mutex_);
    if (diagnostic_mutation_fenced) {
      finish_diagnostic_external_mutation_locked();
    }
    return complete_epoch_fault_locked(claim, secured);
  }

  [[nodiscard]] bool complete_claimed_epoch_clock(
    CoordinatorActiveFaultClaim & claim, SteadyTime arrived_at, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    using EpochRouteKind = ::restocker_task_executor::ActiveTerminationRouteKind;
    bool secured = false;
    try {
      if (before_active_fault_route_) {
        before_active_fault_route_();
      }
      std::optional<CoordinatorTerminationRouteDecision> route;
      switch (claim.kind()) {
        case EpochRouteKind::kProtocolFailure:
          route = termination_router_.route_protocol_failure(
            claim.goal_id(), claim.goal_generation(), arrived_at, 0U, detail);
          break;
        case EpochRouteKind::kCancel:
        case EpochRouteKind::kDrain:
        case EpochRouteKind::kTransformAuthorityLoss:
        case EpochRouteKind::kSimulationTimeAuthorityLoss:
          break;
      }
      if (route) {
        const auto status = route->admission_decision().status();
        auto event = route->take_event();
        if (status == GoalTerminationLatchStatus::kAlreadyLatched) {
          secured = exact_termination_latched(
            route->admission_decision(), claim.goal_id(), claim.goal_generation());
        } else if (status == GoalTerminationLatchStatus::kLatched && event) {
          const auto pushed = driver_->inbox()->push(std::move(*event));
          if (pushed.status == CoordinatorInboxDepositStatus::kAccepted &&
            pushed.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
          {
            secured = true;
          } else {
            const auto loss = termination_router_.record_inbox_loss(
              claim.goal_id(), claim.goal_generation(), arrived_at);
            secured = exact_termination_latched(
              loss, claim.goal_id(), claim.goal_generation());
          }
        }
      }
    } catch (...) {
      secured = false;
    }
    std::lock_guard lock(binding_mutex_);
    if (diagnostic_mutation_fenced) {
      finish_diagnostic_external_mutation_locked();
    }
    return complete_epoch_fault_locked(claim, secured);
  }

  [[nodiscard]] bool complete_claimed_epoch_simulation_time(
    CoordinatorActiveFaultClaim & claim, SteadyTime arrived_at, std::string_view detail,
    bool diagnostic_mutation_fenced) noexcept
  {
    using EpochRouteKind = ::restocker_task_executor::ActiveTerminationRouteKind;
    bool secured = false;
    try {
      if (before_active_fault_route_) {
        before_active_fault_route_();
      }
      std::optional<CoordinatorTerminationRouteDecision> route;
      switch (claim.kind()) {
        case EpochRouteKind::kSimulationTimeAuthorityLoss:
          route = termination_router_.route_simulation_time_authority_loss(
            claim.goal_id(), claim.goal_generation(), arrived_at, 0U, detail);
          break;
        case EpochRouteKind::kCancel:
        case EpochRouteKind::kDrain:
        case EpochRouteKind::kTransformAuthorityLoss:
        case EpochRouteKind::kProtocolFailure:
          break;
      }
      if (route) {
        const auto status = route->admission_decision().status();
        auto event = route->take_event();
        if (status == GoalTerminationLatchStatus::kAlreadyLatched) {
          secured = exact_termination_latched(
            route->admission_decision(), claim.goal_id(), claim.goal_generation());
        } else if (status == GoalTerminationLatchStatus::kLatched && event) {
          const auto pushed = driver_->inbox()->push(std::move(*event));
          if (pushed.status == CoordinatorInboxDepositStatus::kAccepted &&
            pushed.persistence == CoordinatorInboxPersistenceStatus::kEventOwned)
          {
            secured = true;
          } else {
            const auto loss = termination_router_.record_inbox_loss(
              claim.goal_id(), claim.goal_generation(), arrived_at);
            secured = exact_termination_latched(
              loss, claim.goal_id(), claim.goal_generation());
          }
        }
      }
    } catch (...) {
      secured = false;
    }
    std::lock_guard lock(binding_mutex_);
    if (diagnostic_mutation_fenced) {
      finish_diagnostic_external_mutation_locked();
    }
    return complete_epoch_fault_locked(claim, secured);
  }

  void load_configuration()
  {
    action_name_ = node_.declare_parameter<std::string>("action_name", "/restock_product");
    status_topic_ = node_.declare_parameter<std::string>("status_topic", "~/status");
    planning_frame_ = node_.declare_parameter<std::string>("planning_frame", "world");
    workcell_root_frame_ = node_.declare_parameter<std::string>("workcell_root_frame", "shelf");
    motion_enabled_ = node_.declare_parameter<bool>("motion_enabled", false);
    // Gates every motion segment on the planning-scene projection. Default on; set false only
    // for a deployment that runs no projector.
    require_planning_scene_authority_ =
      node_.declare_parameter<bool>("require_planning_scene_authority", true);
    const auto catalog_path = node_.declare_parameter<std::string>("product_catalog_path", "");
    const auto geometry_path = node_.declare_parameter<std::string>("workcell_geometry_path", "");
    gripper_geometry_path_ = node_.declare_parameter<std::string>("gripper_geometry_path", "");
    if (action_name_.empty() || status_topic_.empty() || planning_frame_.empty() ||
      workcell_root_frame_.empty() || catalog_path.empty() || geometry_path.empty() ||
      gripper_geometry_path_.empty())
    {
      throw std::invalid_argument(
              "action, frame, catalog, workcell, and gripper geometry parameters must be set");
    }

    auto catalog = ProductCollisionCatalog::load(std::filesystem::path(catalog_path));
    if (!catalog) {
      throw std::invalid_argument("invalid product catalog: " + catalog.error().detail);
    }
    product_catalog_ = std::make_unique<ProductCollisionCatalog>(std::move(catalog.value()));
    auto geometry = load_manipulation_geometry(std::filesystem::path(geometry_path));
    if (!geometry) {
      throw std::invalid_argument("invalid workcell geometry: " + geometry.error().detail);
    }
    workcell_geometry_ = std::move(geometry.value());
    try {
      survey_geometry_ = restocker_perception::load_workcell_survey_geometry(
        std::filesystem::path(geometry_path));
      survey_stations_ = restocker_perception::nominal_survey_stations(survey_geometry_);
    } catch (const std::exception & error) {
      throw std::invalid_argument(
              std::string("invalid workcell survey geometry: ") + error.what());
    }
    auto gripper = load_gripper_staging_geometry(std::filesystem::path(gripper_geometry_path_));
    if (!gripper) {
      throw std::invalid_argument("invalid gripper geometry: " + gripper.error().detail);
    }
    gripper_geometry_ = std::move(gripper.value());
    tool_frame_ = node_.declare_parameter<std::string>("grasp.tool_frame", "tool0");
    grasp_center_frame_ =
      node_.declare_parameter<std::string>("grasp.center_frame", "grasp_center");
    if (tool_frame_.empty() || grasp_center_frame_.empty() || tool_frame_ == grasp_center_frame_) {
      throw std::invalid_argument("grasp tool and center frames must be distinct and non-empty");
    }

    service_names_.get_snapshot = node_.declare_parameter<std::string>(
      "world_state.get_snapshot_service", "/world_state/get_snapshot");
    service_names_.reserve_task = node_.declare_parameter<std::string>(
      "world_state.reserve_task_service", "/world_state/reserve_task");
    service_names_.validate_reservation = node_.declare_parameter<std::string>(
      "world_state.validate_reservation_service", "/world_state/validate_reservation");
    service_names_.release_reservation = node_.declare_parameter<std::string>(
      "world_state.release_reservation_service", "/world_state/release_reservation");
    service_names_.validate_execution_authority =
      node_.declare_parameter<std::string>(
      "world_state.validate_execution_authority_service",
      "/world_state/validate_execution_authority");
    invalidate_lane_evidence_service_ = node_.declare_parameter<std::string>(
      "world_state.invalidate_lane_evidence_service",
      "/world_state/invalidate_lane_evidence");
    if (service_names_.get_snapshot.empty() || service_names_.reserve_task.empty() ||
      service_names_.validate_reservation.empty() || service_names_.release_reservation.empty() ||
      service_names_.validate_execution_authority.empty() ||
      invalidate_lane_evidence_service_.empty())
    {
      throw std::invalid_argument("world-state service names must not be empty");
    }

    // Declared with startup config: declare_parameter after the immutability gate is FATAL.
    wrist_depth_topic_ = node_.declare_parameter<std::string>(
      "perception.wrist_depth_topic", "/wrist_camera/depth_image");
    wrist_camera_info_topic_ = node_.declare_parameter<std::string>(
      "perception.wrist_camera_info_topic", "/wrist_camera/camera_info");
    acquire_lane_observation_service_ = node_.declare_parameter<std::string>(
      "perception.acquire_lane_observation_service",
      "/perception/acquire_lane_observation");
    // Same default as depth_obstacle_node / perception_node: the bridge publishes reliably and a
    // best-effort subscriber loses most depth frames, so liveness sees stamp gaps over the
    // 500 ms horizon and refuses selection.
    reliable_wrist_depth_qos_ =
      node_.declare_parameter<bool>("perception.reliable_sensor_qos", true);
    // Bounded observable reacquire for the same liveness horizon at motion-segment requests.
    // The horizon is never widened here: expiry keeps the terminal refusal. Zero is accepted
    // and skips the hold, refusing immediately as before.
    driver_config_.perception_reacquire_timeout =
      nonnegative_milliseconds(node_, "perception.reacquire_timeout_ms", 5000);
    // Milestone 10 §6 (Card 060): bounded stop-settle hold before recovery refuses a stop it has
    // not established from fresh telemetry. Zero refuses at once; expiry stays UNSAFE.
    driver_config_.recovery_stop_settle_timeout =
      nonnegative_milliseconds(node_, "recovery.stop_settle_timeout_ms", 2000);
    driver_config_.recovery_stop_settle_poll =
      positive_milliseconds(node_, "recovery.stop_settle_poll_ms", 100);
    if (wrist_depth_topic_.empty() || wrist_camera_info_topic_.empty() ||
      acquire_lane_observation_service_.empty())
    {
      throw std::invalid_argument("perception topic and acquire service names must not be empty");
    }

    pump_period_ = positive_milliseconds(node_, "pump_period_ms", 20);
    startup_config_.snapshot_timeout =
      positive_milliseconds(node_, "startup.snapshot_timeout_ms", 1000);
    startup_config_.retry_period = positive_milliseconds(node_, "startup.retry_period_ms", 250);
    driver_config_.inbox_capacity = positive_size(node_, "inbox_capacity", 128);
    executor_threads_ = positive_size(node_, "executor_threads", 3, 3U);
    generation_deposit_capacity_ =
      positive_size(node_, "generation_deposit_capacity", 64, executor_threads_);
    if (generation_deposit_capacity_ > 4096U) {
      throw std::invalid_argument("generation_deposit_capacity must not exceed 4096");
    }
    driver_config_.planning_frame = planning_frame_;
    driver_config_.task.max_operation_retries =
      positive_size(node_, "task.max_operation_retries", 1);
    driver_config_.task.max_recovery_attempts =
      positive_size(node_, "task.max_recovery_attempts", 2);
    driver_config_.task.validation_timeout =
      positive_milliseconds(node_, "task.validation_timeout_ms", 3000);
    driver_config_.task.planning_timeout =
      positive_milliseconds(node_, "task.planning_timeout_ms", 5000);
    driver_config_.pregrasp_continuation_vertical_margin_m = node_.declare_parameter<double>(
      "motion.pregrasp_continuation_vertical_margin_m", 0.005);
    if (!std::isfinite(driver_config_.pregrasp_continuation_vertical_margin_m) ||
      driver_config_.pregrasp_continuation_vertical_margin_m < 0.0)
    {
      throw std::invalid_argument(
              "motion.pregrasp_continuation_vertical_margin_m must be finite and nonnegative");
    }
    driver_config_.task.execution_timeout =
      positive_milliseconds(node_, "task.execution_timeout_ms", 15000);
    driver_config_.task.transaction_timeout =
      positive_milliseconds(node_, "task.transaction_timeout_ms", 30000);
    driver_config_.task.recovery_timeout =
      positive_milliseconds(node_, "task.recovery_timeout_ms", 15000);
    driver_config_.task.total_timeout =
      positive_milliseconds(node_, "task.total_timeout_ms", 120000);
    // Milestone 10 §6 (Card 051): bounded cleanup retreat after the whole-task expiry; a
    // positive floor, because zero would silently turn the approved retreat into a latch.
    driver_config_.task.deadline_retreat_timeout =
      positive_milliseconds(node_, "task.deadline_retreat_timeout_ms", 60000);
    driver_config_.reconciliation.window =
      positive_milliseconds(node_, "reconciliation.window_ms", 5000);
    driver_config_.reconciliation.attempt_timeout =
      positive_milliseconds(node_, "reconciliation.attempt_timeout_ms", 1000);
    driver_config_.reconciliation.max_attempts =
      positive_size(node_, "reconciliation.max_attempts", 3);
    // The advisory reasoner is off by default and never widens deterministic authorisation; the
    // parameters below are inert while it is off.
    reasoner_enabled_ = node_.declare_parameter<bool>("reasoner.enabled", false);
    reasoner_config_.host = node_.declare_parameter<std::string>("reasoner.host", "127.0.0.1");
    const auto reasoner_port = node_.declare_parameter<int>("reasoner.port", 8080);
    if (reasoner_port <= 0 || reasoner_port > 65535) {
      throw std::invalid_argument("reasoner.port must be a TCP port");
    }
    reasoner_config_.port = static_cast<std::uint16_t>(reasoner_port);
    reasoner_config_.path =
      node_.declare_parameter<std::string>("reasoner.path", "/v1/chat/completions");
    reasoner_config_.model = node_.declare_parameter<std::string>("reasoner.model", "");
    reasoner_config_.deadline = positive_milliseconds(node_, "reasoner.timeout_ms", 4000);
    driver_config_.reasoner_minimum_confidence =
      node_.declare_parameter<double>("reasoner.minimum_confidence", 0.5);
    if (!std::isfinite(driver_config_.reasoner_minimum_confidence) ||
      driver_config_.reasoner_minimum_confidence < 0.0 ||
      driver_config_.reasoner_minimum_confidence > 1.0)
    {
      throw std::invalid_argument("reasoner.minimum_confidence must lie in [0, 1]");
    }
    reasoner_audit_path_ = node_.declare_parameter<std::string>("reasoner.audit_path", "");
    shutdown_timeout_ = positive_milliseconds(node_, "shutdown_timeout_ms", 20000);
    auto minimum_shutdown = std::chrono::milliseconds::zero();
    for (const auto component :
      {driver_config_.task.validation_timeout, driver_config_.task.validation_timeout,
        driver_config_.reconciliation.window, driver_config_.reconciliation.window, pump_period_,
        pump_period_, pump_period_, pump_period_})
    {
      const auto sum = checked_add(minimum_shutdown, component);
      if (!sum) {
        throw std::invalid_argument("shutdown timeout lower bound overflows milliseconds");
      }
      minimum_shutdown = *sum;
    }
    if (shutdown_timeout_ < minimum_shutdown) {
      throw std::invalid_argument(
              "shutdown_timeout_ms is shorter than the pre-motion cleanup lower bound");
    }

    selection_config_.maximum_object_age =
      positive_milliseconds(node_, "selection.maximum_object_age_ms", 500);
    selection_config_.lane_evidence_validity =
      positive_milliseconds(node_, "selection.lane_evidence_validity_ms", 60000);
    selection_config_.maximum_robot_age =
      positive_milliseconds(node_, "selection.maximum_robot_age_ms", 500);
    selection_config_.perception_liveness_max_age =
      positive_milliseconds(node_, "selection.perception_liveness_max_age_ms", 500);
    selection_config_.maximum_future_skew =
      positive_milliseconds(node_, "selection.maximum_future_skew_ms", 50);
    selection_config_.stock_containment_margin_m =
      node_.declare_parameter<double>("selection.stock_containment_margin_m", 0.0);
    selection_config_.stock_boundary_tolerance_m =
      node_.declare_parameter<double>("selection.stock_boundary_tolerance_m", 0.002);
    selection_config_.destination_entry_depth_tolerance_m = node_.declare_parameter<double>(
      "selection.destination_entry_depth_tolerance_m", 0.010);
    selection_config_.maximum_upright_tilt_rad =
      node_.declare_parameter<double>("selection.maximum_upright_tilt_rad", 0.05);
    // Two weights: scaling both legs together cannot change an ordering, so one weight could not
    // express fetch-near versus deliver-near.
    selection_config_.approach_rail_travel_weight =
      node_.declare_parameter<double>("selection.score.approach_rail_travel_weight", 1.0);
    selection_config_.delivery_rail_travel_weight =
      node_.declare_parameter<double>("selection.score.delivery_rail_travel_weight", 1.0);
    driver_config_.maximum_object_age = selection_config_.maximum_object_age;
    driver_config_.maximum_robot_age = selection_config_.maximum_robot_age;
    driver_config_.maximum_future_skew = selection_config_.maximum_future_skew;
    // Length of the run into the lane left to the straight-line insert; everything before it is a
    // collision-checked free-space traverse. The insert is validated only at the tool, so it can
    // be refused for the forearm. A deeper boundary is a harder pre-insertion goal and overran
    // the segment deadline in measured batches. Recovery replans the traverse if the push is
    // not admitted.
    placement_config_.preinsertion_distance_m =
      node_.declare_parameter<double>("placement.preinsertion_distance_m", 0.12);
    placement_config_.retreat_distance_m =
      node_.declare_parameter<double>("placement.retreat_distance_m", 0.12);
    placement_config_.containment_margin_m =
      node_.declare_parameter<double>("placement.containment_margin_m", 0.005);
    selection_config_.destination_containment_margin_m =
      placement_config_.containment_margin_m;
    placement_config_.insertion_floor_clearance_m =
      node_.declare_parameter<double>("placement.insertion_floor_clearance_m", 0.002);
    placement_config_.maximum_axis_alignment_error_rad =
      node_.declare_parameter<double>("placement.maximum_axis_alignment_error_rad", 0.05);
    grasp_generation_config_.approach_yaws_rad = node_.declare_parameter<std::vector<double>>(
      "grasp.approach_yaws_rad", {kHalfPi, -kHalfPi, 0.0});
    // Defaults are measured. A mid-height grasp puts the flange and wrist cylinders (r = 0.09 m)
    // through the supporting surface unless it is clear of it, hence the minimum grasp-centre
    // height above the product base. At a 0.10 m stand-off the closed gripper occupies the tray
    // and MoveIt rejects the approach.
    grasp_generation_config_.minimum_center_height_above_base_m =
      node_.declare_parameter<double>("grasp.minimum_center_height_above_base_m", 0.111);
    grasp_generation_config_.maximum_top_above_center_m =
      node_.declare_parameter<double>("grasp.maximum_top_above_center_m", 0.080);
    grasp_generation_config_.pregrasp_distance_m =
      node_.declare_parameter<double>("grasp.pregrasp_distance_m", 0.18);
    grasp_generation_config_.retract_distance_m =
      node_.declare_parameter<double>("grasp.retract_distance_m", 0.12);
    grasp_generation_config_.open_clearance_per_side_m =
      gripper_geometry_.open_clearance_per_side_m;
    grasp_generation_config_.hold_clearance_per_side_m =
      gripper_geometry_.hold_clearance_per_side_m;
    grasp_generation_config_.maximum_open_joint_position_m =
      gripper_geometry_.maximum_open_target_m;
    grasp_generation_config_.rail_travel_weight =
      node_.declare_parameter<double>("grasp.score.rail_travel_weight", 1.0);
    grasp_generation_config_.rear_access_weight =
      node_.declare_parameter<double>("grasp.score.rear_access_weight", 1.0);
    grasp_generation_config_.wrist_clearance_weight =
      node_.declare_parameter<double>("grasp.score.wrist_clearance_weight", 1.0);
    grasp_generation_config_.maximum_upright_tilt_rad = selection_config_.maximum_upright_tilt_rad;
    grasp_transform_translation_tolerance_m_ =
      node_.declare_parameter<double>("grasp.transform_translation_tolerance_m", 1.0e-6);
    grasp_transform_rotation_tolerance_rad_ =
      node_.declare_parameter<double>("grasp.transform_rotation_tolerance_rad", 1.0e-6);
    if (!std::isfinite(selection_config_.stock_containment_margin_m) ||
      selection_config_.stock_containment_margin_m < 0.0 ||
      !std::isfinite(selection_config_.maximum_upright_tilt_rad) ||
      selection_config_.maximum_upright_tilt_rad <= 0.0 ||
      selection_config_.maximum_upright_tilt_rad >= kHalfPi ||
      grasp_generation_config_.approach_yaws_rad.empty() ||
      grasp_generation_config_.approach_yaws_rad.size() > 16U ||
      !std::ranges::all_of(
        grasp_generation_config_.approach_yaws_rad,
        [](double value) {return std::isfinite(value);}) ||
      !std::isfinite(grasp_generation_config_.minimum_center_height_above_base_m) ||
      grasp_generation_config_.minimum_center_height_above_base_m < 0.0 ||
      !std::isfinite(grasp_generation_config_.maximum_top_above_center_m) ||
      grasp_generation_config_.maximum_top_above_center_m <= 0.0 ||
      !std::isfinite(grasp_generation_config_.pregrasp_distance_m) ||
      grasp_generation_config_.pregrasp_distance_m <= 0.0 ||
      !std::isfinite(grasp_generation_config_.retract_distance_m) ||
      grasp_generation_config_.retract_distance_m <= 0.0 ||
      !std::isfinite(grasp_generation_config_.rail_travel_weight) ||
      grasp_generation_config_.rail_travel_weight < 0.0 ||
      !std::isfinite(grasp_generation_config_.rear_access_weight) ||
      grasp_generation_config_.rear_access_weight < 0.0 ||
      !std::isfinite(grasp_generation_config_.wrist_clearance_weight) ||
      grasp_generation_config_.wrist_clearance_weight < 0.0 ||
      !std::isfinite(grasp_transform_translation_tolerance_m_) ||
      grasp_transform_translation_tolerance_m_ < 0.0 ||
      !std::isfinite(grasp_transform_rotation_tolerance_rad_) ||
      grasp_transform_rotation_tolerance_rad_ < 0.0)
    {
      throw std::invalid_argument("selection geometry tolerances are invalid");
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Action::Goal> goal)
  {
    const CoordinatorGoalId goal_id = uuid;
    if (!valid_uuid(goal_id)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto invalid_selection_detail = invalid_goal_selection_detail(*goal);
    if (!invalid_selection_detail.empty()) {
      RCLCPP_WARN(
        node_.get_logger(), "rejecting malformed restock goal: %.*s",
        static_cast<int>(invalid_selection_detail.size()),
        invalid_selection_detail.data());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const auto clock_sample = steady_clock_->sample();
    if (clock_sample.provider_failed) {
      (void)observe_steady_clock_failure(clock_sample);
      return rclcpp_action::GoalResponse::REJECT;
    }
    auto selection = selection_request_from_goal(*goal);
    if (!selection) {
      RCLCPP_ERROR(
        node_.get_logger(), "validated restock goal could not be converted: %s",
        selection.error().detail.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    const bool clock_failed_before_binding = steady_clock_->provider_failed();
    if (clock_failed_before_binding) {
      (void)observe_steady_clock_failure(steady_clock_->sample());
      return rclcpp_action::GoalResponse::REJECT;
    }
    PendingHandoffBindingKey pending_key;
    GoalGeneration generation = 0U;
    {
      std::lock_guard lock(binding_mutex_);
      const bool binding_unavailable = pending_ || active_ ||
        diagnostic_external_mutations_ != 0U ||
        accepted_handle_adoption_.fail_stopped() ||
        preaccept_transaction_fault_ || adapter_fail_stopped_ ||
        // Card 082, §6: a latched callback failure is the allocation-free half of the receipt
        // and must stop admission on its own, even when the allocating half never ran.
        callback_failure_latched_.load(std::memory_order_acquire);
      if (shutdown_status_ != CoordinatorShutdownStatus::kRunning) {
        return rclcpp_action::GoalResponse::REJECT;
      } else if (binding_unavailable) {
        return rclcpp_action::GoalResponse::REJECT;
      } else if (!pump_gate_allows_preaccept(pump_lease_gate_.snapshot())) {
        preaccept_transaction_fault_ = true;
        return rclcpp_action::GoalResponse::REJECT;
      } else {
        const auto incarnation =
          checked_next_pending_binding_incarnation(last_pending_binding_incarnation_);
        if (!incarnation) {
          preaccept_transaction_fault_ = true;
          return rclcpp_action::GoalResponse::REJECT;
        }
        const auto receipt = admission_.reserve(goal_id);
        if (receipt.decision != GoalAdmissionDecision::kAccepted) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        generation = receipt.generation;
        invalidate_diagnostic_caches_locked();
        pending_key = PendingHandoffBindingKey{goal_id, generation, *incarnation};
        pending_.emplace(pending_key, clock_sample.time, std::move(selection.value()));
        last_pending_binding_incarnation_ = *incarnation;
      }
    }

    std::unique_ptr<CoordinatorActiveFaultEpoch> epoch_candidate;
    std::shared_ptr<CoordinatorGenerationQuiescence> quiescence_candidate;
    try {
      epoch_candidate = std::make_unique<CoordinatorActiveFaultEpoch>(goal_id, generation);
      quiescence_candidate = std::make_shared<CoordinatorGenerationQuiescence>(
        goal_id, generation, generation_deposit_capacity_);
    } catch (...) {
      std::optional<SelectionRequest> pending_selection_disposal;
      {
        std::lock_guard lock(binding_mutex_);
        const bool exact_pending =
          pending_ && pending_->key == pending_key && !active_ &&
          pending_->handoff.snapshot().phase == PendingAcceptedHandoffPhase::kAwaitingEpoch &&
          pending_->handoff.snapshot().route_evidence_revision == 0U &&
          !pending_->handoff.snapshot().route_work &&
          !pending_->handoff.snapshot().handoff_work && !pending_->epoch &&
          !pending_->quiescence && !pending_->accepted_handle && !pending_->first_termination &&
          !pending_->retained_termination && !pending_->deferred_termination &&
          std::all_of(
          pending_->route_metadata.begin(), pending_->route_metadata.end(),
          [](const auto & item) {return !item.has_value();});
        bool admission_finished = false;
        if (exact_pending) {
          try {
            admission_finished = admission_.finish(goal_id, generation);
          } catch (...) {
            admission_finished = false;
          }
        }
        if (admission_finished) {
          invalidate_diagnostic_caches_locked();
          pending_selection_disposal.emplace(std::move(pending_->selection_request));
          pending_.reset();
        } else {
          preaccept_transaction_fault_ = true;
          if (pending_ && pending_->key == pending_key) {
            enter_pending_cleanup_locked();
          }
        }
      }
      return rclcpp_action::GoalResponse::REJECT;
    }

    bool installed = false;
    {
      std::lock_guard lock(binding_mutex_);
      const bool exact_pending =
        pending_ && pending_->key == pending_key && !active_ &&
        pending_->handoff.snapshot().phase == PendingAcceptedHandoffPhase::kAwaitingEpoch &&
        !pending_->epoch && !pending_->quiescence && !pending_->accepted_handle;
      if (exact_pending) {
        pending_->epoch = std::move(epoch_candidate);
        pending_->quiescence = std::move(quiescence_candidate);
        installed = pending_->handoff.install_epoch(pending_->key) ==
          PendingHandoffPhaseEventStatus::kApplied;
      } else {
        preaccept_transaction_fault_ = true;
      }
    }
    if (!installed) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (after_pending_goal_reserved_) {
      try {
        after_pending_goal_reserved_();
      } catch (...) {
        std::optional<PendingRouteExecutionWork> pending_execution;
        bool pending_diagnostic_mutation_fenced = false;
        try {
          std::string owned_work_detail{"post-reservation goal hook failed"};
          std::string owned_metadata_detail{"post-reservation goal hook failed"};
          {
            std::lock_guard lock(binding_mutex_);
            if (!pending_ || pending_->key != pending_key) {
              adapter_fail_stopped_ = true;
            } else {
              (void)claim_pending_termination_locked(
                pending_->key, PendingRouteKind::kCallbackOrAdapterFailure, clock_sample.time,
                owned_work_detail, owned_metadata_detail, pending_execution,
                pending_diagnostic_mutation_fenced);
            }
          }
          if (pending_execution && pending_diagnostic_mutation_fenced) {
            execute_pending_termination(
              std::move(*pending_execution),
              pending_diagnostic_mutation_fenced);
          }
        } catch (...) {
          std::lock_guard lock(binding_mutex_);
          adapter_fail_stopped_ = true;
          if (pending_ && pending_->key == pending_key) {
            enter_pending_cleanup_locked();
          }
        }
      }
    }
    const auto post_hook_clock = steady_clock_->sample();
    if (post_hook_clock.provider_failed) {
      (void)observe_steady_clock_failure(post_hook_clock);
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const GoalHandlePtr goal_handle)
  {
    const auto clock_sample = steady_clock_->sample();
    const auto arrived_at = clock_sample.time;
    if (clock_sample.provider_failed) {
      (void)observe_steady_clock_failure(clock_sample);
      return rclcpp_action::CancelResponse::REJECT;
    }
    CoordinatorGoalId goal_id = goal_handle->get_goal_id();
    std::optional<CoordinatorActiveFaultEpochSnapshot> observed_epoch;
    {
      std::lock_guard lock(binding_mutex_);
      observed_epoch = active_epoch_snapshot_locked();
      if (!active_ || !active_->epoch || !observed_epoch || observed_epoch->goal_id != goal_id ||
        shutdown_status_ != CoordinatorShutdownStatus::kRunning ||
        observed_epoch->route_phase != ActiveFaultRoutePhase::kClear)
      {
        return rclcpp_action::CancelResponse::REJECT;
      }
    }
    std::optional<GoalTerminationSnapshotDecision> observed_termination;
    std::shared_ptr<const FirstGoalTerminationRecord> observed_termination_record;
    try {
      observed_termination.emplace(
        admission_.snapshot_first_termination(
          observed_epoch->goal_id, observed_epoch->goal_generation));
      observed_termination_record = observed_termination->record();
    } catch (...) {
      observed_termination.reset();
      observed_termination_record.reset();
    }
    const bool observed_existing_cancel = observed_termination &&
      observed_termination->status() == GoalTerminationSnapshotStatus::kPresent &&
      observed_termination_record &&
      validate_goal_termination_record(*observed_termination_record) ==
      GoalTerminationValidationError::kNone &&
      observed_termination_record->goal_id == observed_epoch->goal_id &&
      observed_termination_record->goal_generation == observed_epoch->goal_generation &&
      observed_termination_record->kind == GoalTerminationKind::kUserCancel;
    std::optional<CoordinatorActiveFaultClaim> claim;
    bool diagnostic_mutation_fenced = false;
    {
      std::lock_guard lock(binding_mutex_);
      const auto epoch = active_epoch_snapshot_locked();
      if (!active_ || !active_->epoch || !epoch || !observed_epoch ||
        epoch->goal_id != observed_epoch->goal_id ||
        epoch->goal_generation != observed_epoch->goal_generation || epoch->goal_id != goal_id ||
        shutdown_status_ != CoordinatorShutdownStatus::kRunning)
      {
        return rclcpp_action::CancelResponse::REJECT;
      }
      if (observed_existing_cancel && epoch->route_phase == ActiveFaultRoutePhase::kClear) {
        return rclcpp_action::CancelResponse::ACCEPT;
      }
      auto decision = active_->epoch->claim_fault(
        ::restocker_task_executor::ActiveTerminationRouteKind::kCancel);
      claim = decision.take_claim();
      if (!claim) {
        return rclcpp_action::CancelResponse::REJECT;
      }
      invalidate_diagnostic_caches_locked();
      diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
      if (!diagnostic_mutation_fenced) {
        (void)complete_epoch_fault_locked(*claim, false);
        return rclcpp_action::CancelResponse::REJECT;
      }
    }
    const bool secured = complete_claimed_epoch_cancel(
      *claim, arrived_at, "client requested safe cancellation", diagnostic_mutation_fenced);
    if (steady_clock_->provider_failed()) {
      (void)observe_steady_clock_failure(steady_clock_->sample());
    }
    return secured ?
           rclcpp_action::CancelResponse::ACCEPT :
           rclcpp_action::CancelResponse::REJECT;
  }

  AcceptedHandleAdoptionStatus adopt_accepted_handle(
    detail::InspectedAcceptedHandle<GoalHandlePtr> & inspected)
  {
    AcceptedHandleAdoptionStatus adoption_status;
    {
      std::lock_guard lock(binding_mutex_);
      invalidate_diagnostic_caches_locked();
      const CoordinatorGoalId * pending_goal_id = pending_ ? &pending_->goal_id : nullptr;
      GoalHandlePtr * pending_handle = pending_ ? &pending_->accepted_handle : nullptr;
      adoption_status = accepted_handle_adoption_.adopt(
        inspected, pending_goal_id, pending_handle,
        active_.has_value());
      if (adoption_status == AcceptedHandleAdoptionStatus::kAdoptedPending && pending_ &&
        pending_->handoff.adopt_accepted_handle(pending_->key) !=
        PendingHandoffPhaseEventStatus::kApplied)
      {
        pending_->route_or_handoff_failed = true;
        pending_->handoff.enter_cleanup_only();
      }
      if (accepted_handle_adoption_.fail_stopped()) {
        if (active_ && active_->epoch) {
          (void)active_->epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kAdapterInvariantFailure);
          invalidate_diagnostic_caches_locked();
        }
        if (pending_) {
          pending_->route_or_handoff_failed = true;
          pending_->handoff.enter_cleanup_only();
          preaccept_transaction_fault_ = true;
        }
      }
    }
    return adoption_status;
  }

  void handle_accepted(GoalHandlePtr goal_handle) noexcept
  {
    auto inspected = detail::inspect_accepted_handle(
      std::move(goal_handle), [](const GoalHandlePtr & handle) {return handle->get_goal_id();});
    const bool identity_extraction_failed = inspected.identity_extraction_failed;
    try {
      if (before_accepted_goal_adoption_) {
        before_accepted_goal_adoption_();
      }
      const auto adoption_status = adopt_accepted_handle(inspected);

      if (adoption_status != AcceptedHandleAdoptionStatus::kAdoptedPending) {
        const auto detail =
          accepted_handle_adoption_detail(adoption_status, identity_extraction_failed);
        if (adoption_status == AcceptedHandleAdoptionStatus::kDuplicatePending) {
          RCLCPP_WARN(node_.get_logger(), "%.*s", static_cast<int>(detail.size()), detail.data());
        } else {
          RCLCPP_ERROR(node_.get_logger(), "%.*s", static_cast<int>(detail.size()), detail.data());
        }
        return;
      }

      advance_pending_handoff();
    } catch (...) {
      // The callback still pins an unadopted handle. Half A is latched before the receipt
      // retains that handle in the exact binding or an existing inline orphan slot.
      record_callback_failure("accepted-goal adoption callback failed", &inspected);
    }
  }

  void drive_startup(SteadyTime now)
  {
    std::optional<WorldStateRequestHandle> request_to_remove;
    StartupProbeAction action;
    {
      std::lock_guard lock(startup_mutex_);
      const auto before = startup_gate_->snapshot();
      if (before.status == CoordinatorStartupStatus::kProbePending) {
        auto taken = startup_mailbox_->take_and_retire(before.attempt_generation);
        if (taken) {
          if (taken->collision) {
            (void)startup_gate_->latch_fault(
              "multiple completions arrived for the active startup snapshot request");
          } else if (!taken->arrival.response || !taken->arrival.transport_error.empty()) {
            const std::string detail =
              taken->arrival.transport_error.empty() ?
              "startup snapshot service completed without a response" :
              "startup snapshot transport failed: " + taken->arrival.transport_error;
            (void)startup_gate_->transport_failed(
              before.attempt_generation,
              taken->arrival.arrived_at, now, detail);
          } else {
            auto converted = restocker_world_state::snapshot_from_message(
              taken->arrival.response->snapshot, planning_frame_);
            StartupAuthorityClassification classification;
            if (!converted) {
              classification = {
                CoordinatorStartupStatus::kFaulted, std::nullopt,
                "authoritative startup snapshot is invalid: " + converted.error().detail};
            } else {
              classification = classify_startup_authority(converted.value());
            }
            (void)startup_gate_->complete(
              before.attempt_generation, taken->arrival.arrived_at, now,
              std::move(classification));
          }
        }
      }

      const bool may_submit = !shutdown_started_.load(std::memory_order_acquire);
      action = startup_gate_->poll(now, may_submit && world_state_->readiness().get_snapshot);
      if (action.kind == StartupProbeActionKind::kRetireTimedOut) {
        auto retired = startup_mailbox_->retire(action.attempt_generation);
        request_to_remove = std::move(retired.request_handle);
      }
      if (action.kind == StartupProbeActionKind::kSubmit &&
        !startup_mailbox_->arm(action.attempt_generation))
      {
        auto retired = startup_mailbox_->retire_any();
        request_to_remove = std::move(retired.request_handle);
        (void)startup_gate_->latch_fault(
          "startup transport mailbox could not arm a new generation");
        action = {};
      }
    }

    if (request_to_remove) {
      (void)world_state_->remove_pending_request(*request_to_remove);
    }

    if (action.kind == StartupProbeActionKind::kSubmit) {
      auto request = std::make_shared<WorldStateCoordinatorPort::GetSnapshot::Request>();
      request->include_removed = false;
      request->include_events = false;
      std::weak_ptr<CoordinatorStartupMailbox> weak_mailbox = startup_mailbox_;
      const auto clock = steady_clock_;
      const auto observer = clock_failure_observer_;
      auto sent = world_state_->get_snapshot(
        OperationCorrelation{kStartupGoalGeneration, action.attempt_generation}, request,
        [weak_mailbox, clock, observer](auto completion) {
          try {
            if (auto mailbox = weak_mailbox.lock()) {
              (void)mailbox->deposit(
                StartupSnapshotArrival{
                completion.correlation.operation_generation,
                sample_async_evidence_time(clock, observer), std::move(completion.response),
                std::move(completion.transport_error)});
            }
          } catch (...) {
            // A completion that cannot be retained is retired by the bounded startup timeout.
            // No completion exception may escape into the ROS executor.
          }
        });
      if (!sent) {
        std::lock_guard lock(startup_mutex_);
        (void)startup_mailbox_->retire(action.attempt_generation);
        (void)startup_gate_->submission_failed(
          action.attempt_generation, now, "startup snapshot submission failed: " + sent.detail);
      }
      if (sent) {
        const auto installed =
          startup_mailbox_->install_handle(action.attempt_generation, *sent.handle);
        if (installed != StartupHandleInstall::kInstalled) {
          (void)world_state_->remove_pending_request(*sent.handle);
        }
        if (installed == StartupHandleInstall::kInvalidHandle ||
          installed == StartupHandleInstall::kDuplicate)
        {
          std::optional<WorldStateRequestHandle> stored_request;
          {
            std::lock_guard lock(startup_mutex_);
            auto retired = startup_mailbox_->retire_any();
            stored_request = std::move(retired.request_handle);
            (void)startup_gate_->latch_fault(
              installed == StartupHandleInstall::kInvalidHandle ?
              "startup transport returned an invalid request handle" :
              "startup transport returned duplicate request handles");
          }
          if (stored_request) {
            (void)world_state_->remove_pending_request(*stored_request);
          }
        }
      }
    }

    const auto after = startup_snapshot();
    if (after.status == CoordinatorStartupStatus::kOrphanedReservation ||
      after.status == CoordinatorStartupStatus::kFaulted)
    {
      std::optional<std::string> adapter_fault_candidate{after.detail};
      std::string owned_work_detail{after.detail};
      std::string owned_metadata_detail{after.detail};
      std::optional<PendingRouteExecutionWork> pending_execution;
      bool pending_diagnostic_mutation_fenced = false;
      bool inhibit_without_binding = false;
      {
        std::lock_guard lock(binding_mutex_);
        if (active_) {
          if (!active_->epoch) {
            adapter_fail_stopped_ = true;
          } else {
            const auto invalidated = active_->epoch->invalidate_fault(
              ActiveFaultInvalidationKind::kStartupAuthorityFailure);
            if (invalidated == ActiveFaultInvalidationStatus::kInvalidKind) {
              adapter_fail_stopped_ = true;
            }
            invalidate_diagnostic_caches_locked();
          }
          if (!adapter_fault_) {
            adapter_fault_.swap(adapter_fault_candidate);
          }
        } else if (pending_) {
          (void)claim_pending_termination_locked(
            pending_->key, PendingRouteKind::kCallbackOrAdapterFailure, now, owned_work_detail,
            owned_metadata_detail, pending_execution, pending_diagnostic_mutation_fenced);
        } else {
          inhibit_without_binding = begin_diagnostic_external_mutation_locked();
        }
      }
      if (pending_execution && pending_diagnostic_mutation_fenced) {
        execute_pending_termination(
          std::move(*pending_execution),
          pending_diagnostic_mutation_fenced);
      } else if (inhibit_without_binding) {
        try {
          admission_.inhibit(after.detail);
        } catch (...) {
          // Cache fencing is completed below even if the diagnostic cannot be allocated.
        }
        finish_diagnostic_external_mutation();
      }
    }
  }

  void retire_startup_for_shutdown()
  {
    std::optional<WorldStateRequestHandle> request_to_remove;
    {
      std::lock_guard lock(startup_mutex_);
      (void)startup_gate_->retire_for_shutdown();
      auto retired = startup_mailbox_->retire_any();
      request_to_remove = std::move(retired.request_handle);
    }
    if (request_to_remove) {
      (void)world_state_->remove_pending_request(*request_to_remove);
    }
  }

  [[nodiscard]] bool startup_transport_quiescent() const
  {
    std::lock_guard lock(startup_mutex_);
    return startup_gate_->snapshot().status != CoordinatorStartupStatus::kProbePending &&
           startup_mailbox_->empty();
  }

  void publish_startup_status()
  {
    const auto startup = startup_snapshot();
    const auto admission = admission_.snapshot();
    restocker_interfaces::msg::RestockCoordinatorStatus message;
    message.header.frame_id = planning_frame_;
    message.startup_state = startup_status_to_message(startup.status);
    message.attempt_generation = startup.attempt_generation;
    const bool clock_failed = steady_clock_->provider_failed();
    bool orchestration_fault = false;
    bool orchestration_fault_unsecured = false;
    {
      std::lock_guard lock(binding_mutex_);
      const auto active_epoch = active_epoch_snapshot_locked();
      orchestration_fault =
        preaccept_transaction_fault_ ||
        (pending_ &&
        (pending_->route_or_handoff_failed ||
        pending_->handoff.snapshot().phase == PendingAcceptedHandoffPhase::kCleanupOnly)) ||
        accepted_handle_adoption_.fail_stopped() ||
        (active_epoch && active_epoch->route_phase != ActiveFaultRoutePhase::kClear);
      orchestration_fault_unsecured =
        preaccept_transaction_fault_ ||
        (pending_ &&
        (pending_->route_or_handoff_failed ||
        pending_->handoff.snapshot().phase == PendingAcceptedHandoffPhase::kCleanupOnly)) ||
        accepted_handle_adoption_.fail_stopped() ||
        (active_epoch && active_epoch->route_phase == ActiveFaultRoutePhase::kUnsecured);
    }
    message.admission_ready =
      !clock_failed && !orchestration_fault &&
      effective_admission_ready(admission, shutdown_started_.load(std::memory_order_acquire));
    message.inhibited = admission.inhibited || clock_failed || orchestration_fault_unsecured;
    if (admission.inhibited) {
      message.detail = admission.inhibition_detail;
    } else if (orchestration_fault_unsecured) {
      message.detail =
        "coordinator orchestration fault is unsecured; operator reconciliation is required";
    } else if (clock_failed) {
      message.detail = "coordinator steady-clock provider failed; normal orchestration is stopped";
    } else if (orchestration_fault) {
      message.detail = "coordinator fault cleanup pending; termination authority is secured";
    } else {
      message.detail = startup.status == CoordinatorStartupStatus::kReady ?
        admission.readiness_detail :
        startup.detail;
    }
    message.has_orphaned_reservation = startup.orphan.has_value();
    if (startup.orphan) {
      message.orphaned_reservation =
        restocker_world_state::task_reservation_to_message(*startup.orphan);
    }
    // Compare before stamping so clock progress does not republish unchanged state.
    if (last_startup_status_ && *last_startup_status_ == message) {
      return;
    }
    last_startup_status_ = message;
    message.header.stamp = node_.now();
    startup_status_publisher_->publish(message);
    RCLCPP_INFO(
      node_.get_logger(), "coordinator startup state=%s attempt=%" PRIu64 " detail=%s",
      to_string(startup.status), startup.attempt_generation, message.detail.c_str());
  }

  void note_perception_acquisition(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec == 0 && stamp.nanosec == 0U) {
      return;
    }
    const rclcpp::Time stamped{stamp.sec, stamp.nanosec, node_.get_clock()->get_clock_type()};
    const rclcpp::Time received_at = node_.now();
    std::lock_guard lock(perception_mutex_);
    ++perception_arrivals_;
    if (!last_perception_acquisition_ || stamped > *last_perception_acquisition_) {
      last_perception_acquisition_ = stamped;
      last_perception_received_at_ = received_at;
      ++perception_stamp_advances_;
    }
  }

  [[nodiscard]] bool perception_stream_live(std::string & detail)
  {
    PerceptionLivenessObservation observation;
    {
      std::lock_guard lock(perception_mutex_);
      observation.last_acquisition = last_perception_acquisition_;
      observation.last_received_at = last_perception_received_at_;
      observation.arrivals = perception_arrivals_;
      observation.stamp_advances = perception_stamp_advances_;
    }
    const auto now = node_.now();
    detail = describe_perception_liveness(
      observation, now, selection_config_.perception_liveness_max_age);
    bool live = false;
    if (observation.last_acquisition) {
      const auto age = now - *observation.last_acquisition;
      live = age <= selection_config_.perception_liveness_max_age;
    }
    // One line per state change, not per pump tick: the hold re-runs this predicate at the
    // coordinator period, and a 5 s unlive window would otherwise bury the console. Written
    // only from the pump's caller, so the transition flag needs no lock of its own.
    if (!perception_liveness_reported_ || *perception_liveness_reported_ != live) {
      if (perception_liveness_reported_) {
        if (live) {
          RCLCPP_INFO(
            node_.get_logger(), "wrist depth liveness restored: %s", detail.c_str());
        } else {
          RCLCPP_WARN(node_.get_logger(), "wrist depth liveness lost: %s", detail.c_str());
        }
      }
      perception_liveness_reported_ = live;
    }
    return live;
  }

  [[nodiscard]] std::optional<Eigen::Isometry3d> destination_lane_retreat_tool0(
    const SelectedTaskPair & selection) const
  {
    const restocker_perception::SurveyStation * station =
      restocker_perception::find_survey_station(survey_stations_, selection.lane_id.value);
    if (station == nullptr || !world_from_shelf_) {
      return std::nullopt;
    }
    const auto framed = restocker_perception::FramedTransform::create(
      restocker_perception::kShelfFrame, planning_frame_, *world_from_shelf_);
    if (!framed) {
      return std::nullopt;
    }
    const auto viewpoint = restocker_perception::station_viewpoint(*station, framed.value());
    if (!viewpoint) {
      return std::nullopt;
    }
    Eigen::Isometry3d tool0_from_optical = Eigen::Isometry3d::Identity();
    try {
      const auto transform = tf_buffer_->lookupTransform(
        "tool0", restocker_perception::kWristCameraOpticalFrame, tf2::TimePointZero);
      tool0_from_optical = tf2::transformToEigen(transform.transform);
    } catch (const tf2::TransformException &) {
      return std::nullopt;
    }
    const auto mount_transform = restocker_perception::FramedTransform::create(
      restocker_perception::kWristCameraOpticalFrame, restocker_perception::kToolFrame,
      tool0_from_optical);
    if (!mount_transform) {
      return std::nullopt;
    }
    const auto mount = restocker_perception::WristCameraMount::create(mount_transform.value());
    if (!mount) {
      return std::nullopt;
    }
    const auto goal = mount.value().tool0_goal_for(viewpoint.value());
    if (!goal) {
      return std::nullopt;
    }
    return goal.value().pose.pose;
  }

  [[nodiscard]] bool submit_lane_observation_acquire(
    OperationCorrelation correlation, const std::string & lane_id,
    CoordinatorDestinationObservationDone done)
  {
    if (!done || !acquire_lane_client_ || lane_id.empty() ||
      !acquire_lane_client_->service_is_ready())
    {
      return false;
    }
    auto request = std::make_shared<restocker_interfaces::srv::AcquireLaneObservation::Request>();
    request->lane_id = lane_id;
    const auto clock = steady_clock_;
    const auto failure_latch = async_clock_failure_latch_;
    acquire_lane_client_->async_send_request(
      request,
      [correlation, done = std::move(done), clock, failure_latch](
        rclcpp::Client<restocker_interfaces::srv::AcquireLaneObservation>::SharedFuture future)
      {
        CoordinatorLaneAcquireCompletion completion;
        completion.correlation = correlation;
        completion.arrived_at = sample_driver_evidence_time(clock, failure_latch);
        try {
          const auto response = future.get();
          completion.published = response &&
          response->outcome ==
          restocker_interfaces::srv::AcquireLaneObservation::Response::OUTCOME_PUBLISHED;
          if (!completion.published) {
            completion.detail = response ?
            (response->detail.empty() ?
            "lane observation acquire did not publish" : response->detail) :
            "lane observation acquire returned no response";
          }
        } catch (const std::exception & error) {
          completion.published = false;
          completion.detail = std::string("lane observation acquire failed: ") + error.what();
        } catch (...) {
          completion.published = false;
          completion.detail = "lane observation acquire failed";
        }
        done(std::move(completion));
      });
    return true;
  }

  void invalidate_destination_lane_evidence(
    const std::string & lane_id, std::uint64_t lane_revision)
  {
    if (!invalidate_lane_client_) {
      RCLCPP_WARN(
        node_.get_logger(),
        "lane evidence invalidate skipped for %s: no invalidate client", lane_id.c_str());
      return;
    }
    if (!invalidate_lane_client_->service_is_ready()) {
      RCLCPP_WARN(
        node_.get_logger(),
        "lane evidence invalidate skipped for %s rev %" PRIu64 ": service not ready",
        lane_id.c_str(), lane_revision);
      return;
    }
    auto request = std::make_shared<restocker_interfaces::srv::InvalidateLaneEvidence::Request>();
    request->lane_id = lane_id;
    request->expected_lane_revision = lane_revision;
    // Fire-and-forget: the retreat survey makes the lane selectable again. Log revision
    // mismatches and transport failures.
    invalidate_lane_client_->async_send_request(
      request,
      [this, lane_id, lane_revision](
        rclcpp::Client<restocker_interfaces::srv::InvalidateLaneEvidence>::SharedFuture future)
      {
        try {
          const auto response = future.get();
          if (!response ||
          response->status.code !=
          restocker_interfaces::msg::WorldStateOperationStatus::OK)
          {
            RCLCPP_WARN(
              node_.get_logger(),
              "lane evidence invalidate failed for %s rev %" PRIu64 ": %s",
              lane_id.c_str(), lane_revision,
              response ? response->status.detail.c_str() : "no response");
          }
        } catch (const std::exception & error) {
          RCLCPP_WARN(
            node_.get_logger(),
            "lane evidence invalidate failed for %s rev %" PRIu64 ": %s",
            lane_id.c_str(), lane_revision, error.what());
        } catch (...) {
          RCLCPP_WARN(
            node_.get_logger(),
            "lane evidence invalidate failed for %s rev %" PRIu64,
            lane_id.c_str(), lane_revision);
        }
      });
  }

  // Placement needs the destination lane's own frame, which only the ROS layer can resolve.
  [[nodiscard]] PlacementCandidateResult<PlacementCandidate> generate_destination_placement(
    const restocker_world_state::WorldStateSnapshot & snapshot,
    const SelectedTaskPair & selection, const GraspCandidateBatch & grasp,
    const GraspCoupling & coupling, const restocker_world_state::TaskReservation & reservation)
  {
    const auto placement_failure = [](std::string detail) {
      return PlacementCandidateResult<PlacementCandidate>::failure(
        PlacementCandidateError{
          PlacementCandidateErrorCode::InvalidGeometry, std::move(detail)});
    };

    const auto lane_geometry = workcell_geometry_.lanes.find(selection.lane_id.value);
    if (lane_geometry == workcell_geometry_.lanes.end()) {
      return placement_failure("destination lane has no configured manipulation geometry");
    }
    const auto lane = snapshot.lanes.find(selection.lane_id);
    const auto object = snapshot.objects.find(selection.object_id);
    if (lane == snapshot.lanes.end() || object == snapshot.objects.end()) {
      return placement_failure("destination lane or product is absent from the fresh snapshot");
    }
    if (!tool0_from_grasp_center_) {
      return placement_failure("verified tool0 to grasp-center transform is unavailable");
    }
    if (grasp.candidates.empty()) {
      return placement_failure("no retained grasp candidate defines the held product pose");
    }

    Eigen::Isometry3d world_from_lane = Eigen::Isometry3d::Identity();
    try {
      const auto transform = tf_buffer_->lookupTransform(
        planning_frame_, lane_geometry->second.frame_id, tf2::TimePointZero);
      world_from_lane = tf2::transformToEigen(transform.transform);
    } catch (const tf2::TransformException & error) {
      return placement_failure(
        std::string("destination lane transform is unavailable: ") + error.what());
    }
    if (!rigid_transform(world_from_lane)) {
      return placement_failure("destination lane transform is non-rigid");
    }

    // The coupling was captured at grasp verification with the arm at the planned grasp pose;
    // this snapshot is post-retract, so composing it with the planning-time grasp pose would
    // mix two instants. The destination policy likewise comes from the reservation (captured at
    // grant), not from the live lane in this snapshot: SetLanePolicy may have reparked the lane
    // while the product was in the jaws, and the transfer completes under its capture.
    auto generated = generate_upright_cylinder_placements(
      object->second, lane->second, selection, lane_geometry->second, world_from_lane,
      coupling.product_from_grasp_center, *tool0_from_grasp_center_, placement_config_,
      reservation.destination_expected_product_class, reservation.destination_expected_sku);
    if (!generated) {
      return PlacementCandidateResult<PlacementCandidate>::failure(generated.error());
    }
    if (generated.value().empty()) {
      return placement_failure("placement generation produced no candidate");
    }
    return PlacementCandidateResult<PlacementCandidate>::success(
      std::move(generated.value().front()));
  }

  void refresh_readiness()
  {
    try {
      const auto transform =
        tf_buffer_->lookupTransform(planning_frame_, workcell_root_frame_, tf2::TimePointZero);
      const auto converted = tf2::transformToEigen(transform.transform);
      if (rigid_transform(converted)) {
        shelf_transform_observation_detail_ = "planning-frame to shelf transform is valid";
        if (!world_from_shelf_) {
          world_from_shelf_ = converted;
        } else {
          if (transform_differs(
              *world_from_shelf_, converted,
              grasp_transform_translation_tolerance_m_,
              grasp_transform_rotation_tolerance_rad_))
          {
            request_transform_fault_cleanup(
              "planning-frame to shelf transform changed beyond configured tolerances");
          }
        }
      } else if (world_from_shelf_) {
        request_transform_fault_cleanup(
          "planning-frame to shelf transform became non-rigid after readiness");
      } else {
        shelf_transform_observation_detail_ = "planning-frame to shelf transform is non-rigid";
      }
    } catch (const tf2::TransformException & error) {
      if (world_from_shelf_) {
        request_transform_fault_cleanup(
          "planning-frame to shelf transform was lost after readiness: " +
          std::string(error.what()));
      } else {
        shelf_transform_observation_detail_ =
          "waiting for planning-frame to shelf transform; it is missing or was rejected as "
          "non-rigid: " +
          std::string(error.what());
      }
    }
    try {
      const auto transform =
        tf_buffer_->lookupTransform(tool_frame_, grasp_center_frame_, tf2::TimePointZero);
      const auto converted = tf2::transformToEigen(transform.transform);
      if (rigid_transform(converted)) {
        tool_transform_observation_detail_ = "tool transform agrees with gripper geometry";
        if (transform_differs(
            gripper_geometry_.tool0_from_grasp_center, converted,
            grasp_transform_translation_tolerance_m_,
            grasp_transform_rotation_tolerance_rad_))
        {
          request_transform_fault_cleanup(
            "runtime tool transform disagrees with authoritative gripper geometry");
        } else if (!tool0_from_grasp_center_) {
          tool0_from_grasp_center_ = converted;
        } else {
          if (transform_differs(
              *tool0_from_grasp_center_, converted,
              grasp_transform_translation_tolerance_m_,
              grasp_transform_rotation_tolerance_rad_))
          {
            request_transform_fault_cleanup(
              "tool0 to grasp-center transform changed beyond configured tolerances");
          }
        }
      } else if (tool0_from_grasp_center_) {
        request_transform_fault_cleanup(
          "tool0 to grasp-center transform became non-rigid after readiness");
      } else {
        tool_transform_observation_detail_ = "tool0 to grasp-center transform is non-rigid";
      }
    } catch (const tf2::TransformException & error) {
      if (tool0_from_grasp_center_) {
        request_transform_fault_cleanup(
          "tool0 to grasp-center transform was lost after readiness: " +
          std::string(error.what()));
      } else {
        tool_transform_observation_detail_ =
          "waiting for tool0 to grasp-center transform; it is missing or was rejected as "
          "non-rigid: " +
          std::string(error.what());
      }
    }
    const auto services_ready = world_state_->readiness().all_available();
    // Readiness must cover the whole motion path: MoveIt can accept a plan and then abort because
    // its controller clients have not discovered freshly spawned controllers, and a failed
    // execution is terminal since the arm may have moved.
    const bool backends_ready = (!motion_ || motion_->ready()) &&
      (!gripper_ || gripper_->ready()) && (!attachment_ || attachment_->ready());
    const auto startup = startup_snapshot();
    const bool ready = startup.status == CoordinatorStartupStatus::kReady && services_ready &&
      world_from_shelf_.has_value() && tool0_from_grasp_center_.has_value() &&
      !transform_fault_detail_ && backends_ready;
    bool running = false;
    {
      std::lock_guard lock(binding_mutex_);
      running = shutdown_status_ == CoordinatorShutdownStatus::kRunning;
    }
    if (!running) {
      admission_.update_readiness(false, "coordinator shutdown is in progress");
      return;
    }
    auto readiness_detail =
      ready ? std::string{"coordinator ready"} :
    (startup.status != CoordinatorStartupStatus::kReady ?
    startup.detail :
    (!services_ready ?
    std::string{"waiting for world-state services"} :
    (transform_fault_detail_ ?
    "transform fault cleanup pending: " + *transform_fault_detail_ :
    (!world_from_shelf_ ? shelf_transform_observation_detail_ :
    (!tool0_from_grasp_center_ ? tool_transform_observation_detail_ :
    std::string{"waiting for the composed motion backends to become reachable"})))));
    admission_.update_readiness(ready, std::move(readiness_detail));
  }

  void request_transform_fault_cleanup(const std::string & detail)
  {
    const auto clock_sample = steady_clock_->sample();
    const auto arrived_at = clock_sample.time;
    if (clock_sample.provider_failed) {
      (void)observe_steady_clock_failure(clock_sample);
      return;
    }
    admission_.update_readiness(false, "transform fault cleanup pending: " + detail);
    std::optional<std::string> transform_fault_candidate{detail};
    std::string owned_work_detail{detail};
    std::string owned_metadata_detail{detail};
    std::optional<CoordinatorActiveFaultClaim> transform_claim;
    std::optional<PendingRouteExecutionWork> pending_execution;
    bool pending_diagnostic_mutation_fenced = false;
    bool transform_diagnostic_mutation_fenced = false;
    bool inhibit_without_binding = false;
    try {
      {
        std::lock_guard lock(binding_mutex_);
        if (transform_fault_detail_) {
          return;
        }
        transform_fault_detail_.swap(transform_fault_candidate);
        if (pending_) {
          (void)claim_pending_termination_locked(
            pending_->key, PendingRouteKind::kTransformAuthorityLoss, arrived_at, owned_work_detail,
            owned_metadata_detail, pending_execution, pending_diagnostic_mutation_fenced);
        } else if (!active_) {
          inhibit_without_binding = begin_diagnostic_external_mutation_locked();
        } else if (!active_->epoch) {
          adapter_fail_stopped_ = true;
          return;
        } else {
          auto decision = active_->epoch->claim_fault(
            ::restocker_task_executor::ActiveTerminationRouteKind::kTransformAuthorityLoss);
          transform_claim = decision.take_claim();
          if (!transform_claim) {
            return;
          }
          invalidate_diagnostic_caches_locked();
          transform_diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
          if (!transform_diagnostic_mutation_fenced) {
            (void)complete_epoch_fault_locked(*transform_claim, false);
            adapter_fail_stopped_ = true;
            return;
          }
        }
      }
    } catch (...) {
      std::lock_guard lock(binding_mutex_);
      adapter_fail_stopped_ = true;
      if (transform_claim && transform_claim->live()) {
        (void)complete_epoch_fault_locked(*transform_claim, false);
      }
      return;
    }
    if (inhibit_without_binding) {
      try {
        admission_.inhibit(detail);
      } catch (...) {
        // Cache fencing is completed below even if the diagnostic cannot be allocated.
      }
      finish_diagnostic_external_mutation();
      return;
    }
    if (pending_execution && pending_diagnostic_mutation_fenced) {
      execute_pending_termination(
        std::move(*pending_execution),
        pending_diagnostic_mutation_fenced);
      return;
    }
    if (!transform_claim) {
      return;
    }
    const bool secured = complete_claimed_epoch_transform(
      *transform_claim, arrived_at, detail, transform_diagnostic_mutation_fenced);
    if (steady_clock_->provider_failed()) {
      (void)observe_steady_clock_failure(steady_clock_->sample());
    }
    if (!secured) {
      std::lock_guard lock(binding_mutex_);
      adapter_fail_stopped_ = true;
    }
  }

  [[nodiscard]] std::optional<CoordinatorActiveFaultClaim> observe_simulation_time(
    ActiveBinding & binding, const rclcpp::Time & now, bool & diagnostic_mutation_fenced)
  {
    try {
      if (now.get_clock_type() != binding.last_simulation_time.get_clock_type() ||
        now < binding.last_simulation_time)
      {
        throw std::runtime_error("ROS simulation time regressed during an active restock goal");
      }
      binding.last_elapsed = simulation_elapsed(binding.simulation_started, now);
      binding.last_simulation_time = now;
    } catch (const std::runtime_error &) {
      if (!binding.epoch) {
        adapter_fail_stopped_ = true;
        return std::nullopt;
      }
      std::optional<CoordinatorActiveFaultClaim> claim;
      try {
        auto decision = binding.epoch->claim_fault(
          ::restocker_task_executor::ActiveTerminationRouteKind::kSimulationTimeAuthorityLoss);
        const auto status = decision.status();
        claim = decision.take_claim();
        if (!claim) {
          if (status == ActiveFaultClaimStatus::kIdentityMismatch ||
            status == ActiveFaultClaimStatus::kInvalidRouteKind ||
            status == ActiveFaultClaimStatus::kInvalidArgument)
          {
            (void)binding.epoch->invalidate_fault(
              ActiveFaultInvalidationKind::kAdapterInvariantFailure);
            adapter_fail_stopped_ = true;
            invalidate_diagnostic_caches_locked();
          }
          return std::nullopt;
        }
        invalidate_diagnostic_caches_locked();
        diagnostic_mutation_fenced = begin_diagnostic_external_mutation_locked();
        if (!diagnostic_mutation_fenced) {
          (void)complete_epoch_fault_locked(*claim, false);
          return std::nullopt;
        }
        return claim;
      } catch (...) {
        if (claim && claim->live()) {
          (void)complete_epoch_fault_locked(*claim, false);
        } else {
          (void)binding.epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kAdapterInvariantFailure);
          adapter_fail_stopped_ = true;
          invalidate_diagnostic_caches_locked();
        }
        return std::nullopt;
      }
    }
    return std::nullopt;
  }

  void pump()
  {
    bool callback_blocked = false;
    {
      std::lock_guard lock(binding_mutex_);
      const auto epoch = active_epoch_snapshot_locked();
      callback_blocked = callback_failure_latched_.load(std::memory_order_acquire) &&
        (callback_failure_repeated_.load(std::memory_order_acquire) ||
        !callback_failure_cleanup_generation_ ||
        (pending_ ? callback_failure_cleanup_generation_ != pending_->generation :
        (!epoch || callback_failure_cleanup_generation_ != epoch->goal_generation)));
    }
    if (callback_blocked) {
      // Half A alone prevents another startup probe, handoff or driver iteration. Keep only
      // shutdown deadline observation alive; a latched fault can never certify clean shutdown.
      const auto shutdown_clock = steady_clock_->sample();
      update_shutdown_status(shutdown_clock.time, startup_transport_quiescent());
      return;
    }
    std::optional<CoordinatorPumpLease> lease;
    CoordinatorPumpLeaseAcquireStatus acquire_status;
    {
      std::lock_guard lock(binding_mutex_);
      auto decision = pump_lease_gate_.acquire(pending_.has_value());
      acquire_status = decision.status();
      lease = decision.take_lease();
      if (acquire_status == CoordinatorPumpLeaseAcquireStatus::kAttemptExhausted) {
        adapter_fail_stopped_ = true;
        enter_pending_cleanup_locked();
      } else if (lease) {
        invalidate_diagnostic_caches_locked();
      }
    }
    if (acquire_status == CoordinatorPumpLeaseAcquireStatus::kPendingBinding) {
      advance_pending_handoff();
      const auto shutdown_clock = steady_clock_->sample();
      update_shutdown_status(shutdown_clock.time, startup_transport_quiescent());
      return;
    }
    if (!lease) {
      if (acquire_status != CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased) {
        const auto shutdown_clock = steady_clock_->sample();
        update_shutdown_status(shutdown_clock.time, startup_transport_quiescent());
      }
      return;
    }

    bool advance_after_return = false;
    std::optional<ActiveRetirementBundle> retirement;
    const auto transaction = detail::run_pump_transaction(
      std::move(*lease),
      [this, &retirement](CoordinatorPumpLease & active_pump_lease) {
        const auto clock_sample = steady_clock_->sample();
        const auto steady_now = clock_sample.time;
        if (clock_sample.provider_failed) {
          (void)observe_steady_clock_failure(clock_sample);
        } else {
          drive_startup(steady_now);
          refresh_readiness();
          const auto simulation_now = node_.now();
          std::optional<CoordinatorActiveFaultClaim> time_fault;
          bool time_fault_diagnostic_mutation_fenced = false;
          {
            std::lock_guard lock(binding_mutex_);
            if (active_) {
              time_fault = observe_simulation_time(
                *active_, simulation_now, time_fault_diagnostic_mutation_fenced);
            }
          }
          if (time_fault) {
            (void)complete_claimed_epoch_simulation_time(
              *time_fault, steady_now,
              "ROS simulation time regressed during an active restock goal",
              time_fault_diagnostic_mutation_fenced);
          }
        }
        if (const auto asynchronous_failure = async_clock_failure_latch_->snapshot()) {
          (void)observe_steady_clock_failure(*asynchronous_failure, true);
        }
        bool driver_blocked = false;
        {
          std::lock_guard lock(binding_mutex_);
          const auto active_epoch = active_epoch_snapshot_locked();
          driver_blocked =
          preaccept_transaction_fault_ || adapter_fail_stopped_ ||
          (callback_failure_latched_.load(std::memory_order_acquire) &&
          (callback_failure_repeated_.load(std::memory_order_acquire) || !active_epoch ||
          callback_failure_cleanup_generation_ != active_epoch->goal_generation)) ||
          accepted_handle_adoption_.fail_stopped() || pending_ ||
          (active_ && (!active_epoch ||
          active_epoch->route_phase == ActiveFaultRoutePhase::kRoutingPending ||
          active_epoch->route_phase == ActiveFaultRoutePhase::kUnsecured));
        }
        if (!driver_blocked) {
          driver_->pump(steady_now);
        }
        if (steady_clock_->provider_failed() && !clock_sample.provider_failed) {
          (void)observe_steady_clock_failure(steady_clock_->sample());
        }
        if (const auto asynchronous_failure = async_clock_failure_latch_->snapshot()) {
          (void)observe_steady_clock_failure(*asynchronous_failure, true);
        }
        auto outputs = driver_->take_outputs();
        for (auto & output : outputs) {
          deliver(output);
        }
        std::optional<CoordinatorTerminalPublicationPermit> retained_terminal;
        GoalHandlePtr retained_terminal_handle;
        std::shared_ptr<ActionResultPublisher> retained_terminal_publisher;
        std::chrono::milliseconds retained_terminal_elapsed{0};
        bool publish_retained_terminal = false;
        {
          std::lock_guard lock(binding_mutex_);
          const auto active_epoch = active_epoch_snapshot_locked();
          if (active_ && active_->epoch && active_epoch &&
          active_epoch->route_phase == ActiveFaultRoutePhase::kSecured)
          {
            auto prepared = active_->epoch->prepare_secured_retained_publication();
            auto permit = prepared.take_permit();
            if (prepared.status() == ActiveRetainedPublicationStatus::kPrepared &&
            permit && permit->live())
            {
              retained_terminal_handle = active_->handle;
              retained_terminal_publisher = action_result_publisher_;
              retained_terminal_elapsed = active_->last_elapsed;
              publish_retained_terminal = true;
            } else if (prepared.status() == ActiveRetainedPublicationStatus::kPrepared || permit) {
              fail_terminal_adapter_locked();
            } else {
              invalidate_diagnostic_caches_locked();
            }
            if (permit) {
              retained_terminal = std::move(permit);
            }
          }
        }
        if (publish_retained_terminal && retained_terminal) {
          publish_terminal(
            *retained_terminal, std::move(retained_terminal_handle),
            std::move(retained_terminal_publisher), retained_terminal_elapsed);
        }
        deliver_adapter_fault();
        attempt_active_retirement(active_pump_lease, retirement);

        std::uint64_t snapshot_revision = 0U;
        {
          std::lock_guard lock(binding_mutex_);
          snapshot_revision = diagnostic_cache_revision_;
        }
        const auto driver_snapshot = driver_->snapshot();
        const auto inbox_snapshot = driver_->inbox()->snapshot();
        const auto admission_snapshot = admission_.snapshot();
        {
          std::lock_guard lock(binding_mutex_);
          if (snapshot_revision != diagnostic_cache_revision_ ||
          diagnostic_external_mutations_ != 0U)
          {
            diagnostic_caches_fresh_ = false;
            return;
          }
          cached_driver_ = {driver_snapshot.active, driver_snapshot.inhibited,
            driver_snapshot.pending_operation.has_value(),
            driver_snapshot.pending_transport_requests,
            driver_snapshot.reservation_capability_may_remain};
          cached_inbox_ = {inbox_snapshot.size,
            inbox_snapshot.accepted_handoff_emergency_size,
            inbox_snapshot.cleanup_emergency_size,
            inbox_snapshot.overflow_latched,
            inbox_snapshot.overflow_notification_pending,
            inbox_snapshot.cleanup_evidence_lost,
            inbox_snapshot.generation_accounting_conflict};
          cached_admission_ = {admission_snapshot.phase, admission_snapshot.inhibited,
            admission_snapshot.mutation_submission.has_value()};
          cached_goal_generation_ = driver_snapshot.goal_generation;
          diagnostic_caches_initialized_ = true;
          diagnostic_caches_fresh_ = true;
        }
        publish_startup_status();
      },
      [this, &advance_after_return, &retirement](CoordinatorPumpLease & owned_lease,
      detail::PumpWorkExit exit) noexcept {
        try {
          std::lock_guard lock(binding_mutex_);
          auto & returnable_lease = retirement ? retirement->pump_lease : owned_lease;
          const bool lease_ownership_is_exact = retirement ?
          !owned_lease.live() && returnable_lease.live() : owned_lease.live();
          const auto returned = lease_ownership_is_exact ?
          pump_lease_gate_.return_lease(returnable_lease) :
          CoordinatorPumpLeaseReturnStatus::kNotLive;
          last_pump_return_status_ = returned;
          if (returned != CoordinatorPumpLeaseReturnStatus::kReturned ||
          exit != detail::PumpWorkExit::kCompleted)
          {
            adapter_fail_stopped_ = true;
            enter_pending_cleanup_locked();
          } else {
            advance_after_return = pending_.has_value();
          }
          return std::optional{returned};
        } catch (...) {
          return std::optional<CoordinatorPumpLeaseReturnStatus>{};
        }
      });
    const auto shutdown_clock = steady_clock_->sample();
    update_shutdown_status(shutdown_clock.time, startup_transport_quiescent());
    if (transaction.work_exit != detail::PumpWorkExit::kCompleted || !transaction.lease_return ||
      *transaction.lease_return != CoordinatorPumpLeaseReturnStatus::kReturned)
    {
      return;
    }
    if (advance_after_return) {
      advance_pending_handoff();
    }
  }

  void update_shutdown_status(std::chrono::steady_clock::time_point now, bool startup_quiescent)
  {
    std::lock_guard lock(binding_mutex_);
    if (shutdown_status_ != CoordinatorShutdownStatus::kDraining || !shutdown_deadline_) {
      return;
    }
    if (now >= *shutdown_deadline_) {
      shutdown_status_ = CoordinatorShutdownStatus::kTimedOut;
      if (active_) {
        if (active_->epoch) {
          const auto invalidated = active_->epoch->invalidate_fault(
            ActiveFaultInvalidationKind::kShutdownTimeout);
          if (invalidated == ActiveFaultInvalidationStatus::kInvalidKind) {
            adapter_fail_stopped_ = true;
          }
          invalidate_diagnostic_caches_locked();
        }
        adapter_fail_stopped_ = true;
      } else if (pending_) {
        enter_pending_cleanup_locked();
        adapter_fail_stopped_ = true;
      } else {
        adapter_fail_stopped_ = true;
      }
      return;
    }
    const auto handoff = pending_ ? std::optional{pending_->handoff.snapshot()} : std::nullopt;
    if (coordinator_allows_clean_shutdown(
        CoordinatorCleanShutdownFacts{
        diagnostic_caches_initialized_, diagnostic_caches_fresh_, startup_quiescent,
        pump_lease_gate_.snapshot(), pending_.has_value(), active_.has_value(),
        pending_ && pending_->accepted_handle, accepted_handle_adoption_.orphan_count(),
        handoff && handoff->route_work.has_value(),
        handoff && handoff->handoff_work.has_value(),
        pending_ && pending_->retained_termination.has_value(),
        pending_ && pending_->deferred_termination.has_value(), preaccept_transaction_fault_,
        accepted_handle_adoption_.fail_stopped(),
        adapter_fail_stopped_ || adapter_fault_.has_value() ||
        callback_failure_latched_.load(std::memory_order_acquire), cached_inbox_, cached_admission_,
        cached_driver_}))
    {
      shutdown_status_ = CoordinatorShutdownStatus::kClean;
    }
  }

  void terminal_publication_became_unknown_locked(
    CoordinatorTerminalPublicationPermit & permit) noexcept
  {
    const auto epoch = active_epoch_snapshot_locked();
    if (!active_ || !active_->epoch || !epoch ||
      epoch->goal_generation != permit.goal_generation())
    {
      fail_terminal_adapter_locked();
      return;
    }
    const auto status = active_->epoch->terminal_publication_unknown(permit);
    invalidate_diagnostic_caches_locked();
    if (status != ActivePublicationUnknownStatus::kRestoredRetainedUnsecured) {
      adapter_fail_stopped_ = true;
    }
  }

  [[nodiscard]] bool exact_pump_lease_is_outstanding_locked(
    const CoordinatorPumpLease & lease) const noexcept
  {
    const auto pump = pump_lease_gate_.snapshot();
    return lease.live() && pump.outstanding && !pump.fail_stopped && !pump.attempt_exhausted &&
           lease.attempt() == pump.attempt;
  }

  void fail_active_retirement(
    const CoordinatorGoalId & goal_id, GoalGeneration generation,
    const std::shared_ptr<CoordinatorGenerationQuiescence> & quiescence,
    std::string_view detail) noexcept
  {
    {
      std::lock_guard lock(binding_mutex_);
      const auto epoch = active_epoch_snapshot_locked();
      if (active_ && epoch && epoch->goal_id == goal_id &&
        epoch->goal_generation == generation && active_->quiescence == quiescence)
      {
        fail_terminal_adapter_locked();
      } else {
        adapter_fail_stopped_ = true;
      }
    }
    RCLCPP_ERROR(node_.get_logger(), "%s", detail.data());
  }

  void attempt_active_retirement(
    CoordinatorPumpLease & pump_lease,
    std::optional<ActiveRetirementBundle> & retirement)
  {
    std::shared_ptr<CoordinatorGenerationQuiescence> quiescence;
    CoordinatorGoalId goal_id{};
    GoalGeneration generation{0U};
    {
      std::lock_guard lock(binding_mutex_);
      if (!active_ || !active_->epoch) {
        return;
      }
      const auto epoch = active_->epoch->snapshot();
      if (!epoch.retirement_eligible) {
        return;
      }
      if (epoch.feedback_outstanding || !active_->handle || !active_->quiescence ||
        !exact_pump_lease_is_outstanding_locked(pump_lease))
      {
        fail_terminal_adapter_locked();
        return;
      }
      goal_id = epoch.goal_id;
      generation = epoch.goal_generation;
      quiescence = active_->quiescence;
    }

    std::optional<CoordinatorGenerationProbePermit> probe;
    GenerationQuiescenceProbeStatus probe_status{
      GenerationQuiescenceProbeStatus::kSynchronizationFailed};
    try {
      auto prepared = quiescence->prepare_probe();
      probe_status = prepared.status();
      probe = prepared.take_probe_permit();
    } catch (...) {
      (void)quiescence->mark_synchronization_failure();
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence probe preparation failed");
      return;
    }
    if (probe_status == GenerationQuiescenceProbeStatus::kDepositsOutstanding) {
      return;
    }
    if (probe_status != GenerationQuiescenceProbeStatus::kPrepared || !probe || !probe->live()) {
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence probe could not be prepared for an acknowledged terminal");
      return;
    }

    RestockCoordinatorDriverSnapshot driver_snapshot;
    bool inbox_generation_empty = false;
    try {
      driver_snapshot = driver_->snapshot();
      const auto inbox = driver_->inbox();
      inbox_generation_empty = inbox && inbox->generation_empty(generation);
    } catch (...) {
      (void)quiescence->mark_synchronization_failure();
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence driver or inbox snapshot failed");
      return;
    }

    std::optional<CoordinatorGenerationQuiescenceReceipt> receipt;
    GenerationQuiescenceCompletionStatus completion_status{
      GenerationQuiescenceCompletionStatus::kSynchronizationFailed};
    try {
      auto completed = quiescence->complete_probe(
        *probe, driver_snapshot.inactive_after_terminal_ack_generation,
        inbox_generation_empty);
      completion_status = completed.status();
      receipt = completed.take_receipt();
    } catch (...) {
      (void)quiescence->mark_synchronization_failure();
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence probe completion failed");
      return;
    }
    if (completion_status == GenerationQuiescenceCompletionStatus::kDriverLineageMismatch ||
      completion_status == GenerationQuiescenceCompletionStatus::kInboxNotEmpty)
    {
      return;
    }
    if (completion_status != GenerationQuiescenceCompletionStatus::kReceiptIssued ||
      !receipt || !receipt->live())
    {
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence receipt could not be issued for an acknowledged terminal");
      return;
    }

    bool consume_threw = false;
    bool consume_rejected = false;
    // The function-scope probe, receipt, and quiescence pin outlive this guard, including every
    // early return. consume_receipt() is synchronized and allocation-free, with no callback,
    // logging, or owner destruction. kConsumed is the only operation preceding the nothrow
    // bundle move; capability shells and retired owners are destroyed after the guard unlocks.
    {
      std::lock_guard lock(binding_mutex_);
      if (!active_ || !active_->epoch || active_->quiescence != quiescence ||
        !active_->handle || retirement ||
        !receipt->live() ||
        receipt->goal_id() != goal_id || receipt->goal_generation() != generation ||
        !exact_pump_lease_is_outstanding_locked(pump_lease))
      {
        fail_terminal_adapter_locked();
        return;
      }
      const auto epoch = active_->epoch->snapshot();
      if (!epoch.retirement_eligible || epoch.feedback_outstanding ||
        epoch.retained_terminal || epoch.goal_id != goal_id || epoch.goal_generation != generation)
      {
        fail_terminal_adapter_locked();
        return;
      }
      try {
        const auto consumed = quiescence->consume_receipt(*receipt, goal_id, generation);
        if (consumed == GenerationQuiescenceConsumeStatus::kConsumed) {
          // Move binding, receipt, and still-live lease as one nothrow retirement transaction.
          // The reset destroys only the moved-from ActiveBinding shell; retired owners stay in
          // the outer pump-scope bundle.
          retirement.emplace(
            std::move(*active_), std::move(*receipt), std::move(pump_lease));
          active_.reset();
          invalidate_diagnostic_caches_locked();
        } else {
          consume_rejected = true;
          fail_terminal_adapter_locked();
        }
      } catch (...) {
        consume_threw = true;
      }
    }
    if (consume_threw) {
      (void)quiescence->mark_synchronization_failure();
      fail_active_retirement(
        goal_id, generation, quiescence,
        "generation-quiescence receipt consumption failed");
    } else if (consume_rejected) {
      RCLCPP_ERROR(
        node_.get_logger(),
        "generation-quiescence receipt was rejected during active retirement");
    }
  }

  void publish_terminal(
    CoordinatorTerminalPublicationPermit & publication, GoalHandlePtr handle,
    std::shared_ptr<ActionResultPublisher> publisher,
    std::chrono::milliseconds elapsed)
  {
    constexpr std::string_view internal_fault_detail =
      "coordinator fault won before terminal result delivery";
    const auto generation = publication.goal_generation();
    const auto terminal_kind = action_terminal_kind(publication.classification());
    if (!handle || !publisher || !terminal_kind) {
      {
        std::lock_guard lock(binding_mutex_);
        terminal_publication_became_unknown_locked(publication);
      }
      if (!terminal_kind) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "terminal result reservation has an invalid terminal classification");
      } else {
        RCLCPP_ERROR(
          node_.get_logger(),
          "terminal result reservation has no exact ROS handle or result publisher");
      }
      return;
    }

    try {
      if (after_terminal_delivery_reserved_) {
        after_terminal_delivery_reserved_();
      }
      const auto & original = publication.original_output();
      auto terminal = make_active_terminal_result(
        original, publication.classification(), elapsed, std::string{internal_fault_detail});
      const auto outcome = terminal.outcome;
      const std::string detail = terminal.detail;
      auto result = std::make_shared<Action::Result>(std::move(terminal.result));
      RCLCPP_INFO(
        node_.get_logger(), "delivering terminal restock result: outcome=%u detail=%s",
        static_cast<unsigned int>(outcome), detail.c_str());
      publisher->publish(handle, *terminal_kind, result);
    } catch (const std::exception & error) {
      {
        std::lock_guard lock(binding_mutex_);
        terminal_publication_became_unknown_locked(publication);
      }
      RCLCPP_ERROR(
        node_.get_logger(), "terminal result delivery failed after reservation: %s",
        error.what());
      return;
    } catch (...) {
      {
        std::lock_guard lock(binding_mutex_);
        terminal_publication_became_unknown_locked(publication);
      }
      RCLCPP_ERROR(node_.get_logger(), "terminal result delivery failed after reservation");
      return;
    }

    std::optional<CoordinatorTerminalAcknowledgementPermit> acknowledgement;
    bool acknowledgement_fenced = false;
    bool acknowledgement_preparation_failed = false;
    {
      std::lock_guard lock(binding_mutex_);
      const auto epoch = active_epoch_snapshot_locked();
      if (!active_ || !active_->epoch || !epoch || epoch->goal_generation != generation) {
        fail_terminal_adapter_locked();
        acknowledgement_preparation_failed = true;
      } else {
        auto returned = active_->epoch->terminal_publication_returned(publication);
        const auto return_status = returned.status();
        acknowledgement = returned.take_ack_permit();
        if (return_status != ActivePublicationReturnStatus::kAcknowledgementPrepared ||
          !acknowledgement || !acknowledgement->live())
        {
          fail_terminal_adapter_locked();
          acknowledgement_preparation_failed = true;
        } else {
          acknowledgement_fenced = begin_diagnostic_external_mutation_locked();
          if (!acknowledgement_fenced) {
            auto completion = active_->epoch->complete_terminal_ack(*acknowledgement, false);
            (void)completion.take_accepted_ack_witness();
            invalidate_diagnostic_caches_locked();
            acknowledgement_preparation_failed = true;
          }
        }
      }
    }
    if (acknowledgement_preparation_failed) {
      return;
    }

    bool acknowledged = false;
    try {
      acknowledged = driver_->acknowledge_terminal_delivery(
        acknowledgement->original_output().goal_generation);
    } catch (...) {
      acknowledged = false;
    }

    std::optional<CoordinatorAcceptedTerminalAckWitness> accepted_ack_witness;
    std::shared_ptr<CoordinatorGenerationQuiescence> acknowledged_quiescence;
    bool seal_accepted_ack = false;
    {
      std::lock_guard lock(binding_mutex_);
      if (acknowledgement_fenced) {
        finish_diagnostic_external_mutation_locked();
        acknowledgement_fenced = false;
      }
      const auto epoch = active_epoch_snapshot_locked();
      if (!active_ || !active_->epoch || !epoch || epoch->goal_generation != generation) {
        fail_terminal_adapter_locked();
        return;
      }
      auto completion = active_->epoch->complete_terminal_ack(*acknowledgement, acknowledged);
      const auto completion_status = completion.status();
      accepted_ack_witness = completion.take_accepted_ack_witness();
      invalidate_diagnostic_caches_locked();
      if (completion_status ==
        ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible)
      {
        if (!acknowledged || !accepted_ack_witness || !accepted_ack_witness->live()) {
          adapter_fail_stopped_ = true;
          return;
        }
        if (!active_->quiescence) {
          fail_terminal_adapter_locked();
          return;
        }
        acknowledged_quiescence = active_->quiescence;
        seal_accepted_ack = true;
        invalidate_diagnostic_caches_locked();
      } else {
        if (completion_status !=
          ActiveTerminalAcknowledgementStatus::kRejectedDeliveredUnsecured)
        {
          fail_terminal_adapter_locked();
        }
      }
    }
    if (!seal_accepted_ack) {
      return;
    }

    GenerationQuiescenceSealStatus seal_status{
      GenerationQuiescenceSealStatus::kSynchronizationFailed};
    try {
      seal_status = acknowledged_quiescence->seal_after_accepted_ack(*accepted_ack_witness);
    } catch (...) {
      (void)acknowledged_quiescence->mark_synchronization_failure();
    }
    if (seal_status != GenerationQuiescenceSealStatus::kSealed) {
      fail_active_retirement(
        publication.goal_id(), generation, acknowledged_quiescence,
        "accepted terminal acknowledgement could not seal generation quiescence");
      return;
    }
    {
      std::lock_guard lock(binding_mutex_);
      const auto epoch = active_epoch_snapshot_locked();
      if (!active_ || !active_->epoch || !epoch || epoch->goal_id != publication.goal_id() ||
        epoch->goal_generation != generation ||
        active_->quiescence != acknowledged_quiescence ||
        !epoch->retirement_eligible)
      {
        fail_terminal_adapter_locked();
      } else {
        invalidate_diagnostic_caches_locked();
      }
    }
  }

  void deliver_terminal(CoordinatorDriverOutput & output)
  {
    constexpr std::string_view invalid_contract_detail =
      "driver produced an invalid output kind/outcome combination";
    constexpr std::string_view correlation_detail =
      "driver output does not match the active ROS action generation";
    const bool valid_contract = valid_output_contract(output);
    for (;; ) {
      const bool clock_failed_before_binding = steady_clock_->provider_failed();
      std::optional<CoordinatorTerminalPublicationPermit> publication;
      std::optional<CoordinatorActiveFaultClaim> protocol_claim;
      GoalHandlePtr handle;
      std::shared_ptr<ActionResultPublisher> publisher;
      std::chrono::milliseconds elapsed{0};
      bool protocol_diagnostic_mutation_fenced = false;
      bool route_protocol_claim = false;
      bool observe_clock_failure = false;
      bool correlation_failure = false;
      ActiveTerminalDisposition disposition{
        ActiveTerminalDisposition::kInvalidArgumentOutputUnconsumed};
      {
        std::lock_guard lock(binding_mutex_);
        if (!active_ || !active_->epoch) {
          fail_terminal_adapter_locked();
          return;
        }
        if (clock_failed_before_binding && !steady_clock_failure_handled_) {
          observe_clock_failure = true;
        } else {
          const auto epoch = active_->epoch->snapshot();
          correlation_failure = epoch.goal_generation != output.goal_generation;

          auto decision = active_->epoch->offer_terminal(output);
          disposition = decision.disposition();
          const bool output_consumed = decision.output_was_consumed();
          publication = decision.take_publication_permit();
          protocol_claim = decision.take_fault_claim();
          invalidate_diagnostic_caches_locked();

          switch (disposition) {
            case ActiveTerminalDisposition::kPublicationPrepared:
            case ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization:
              if (!output_consumed || !publication || !publication->live() || protocol_claim) {
                fail_terminal_adapter_locked();
                return;
              }
              handle = active_->handle;
              publisher = action_result_publisher_;
              elapsed = active_->last_elapsed;
              break;
            case ActiveTerminalDisposition::kRetainedAndFaultClaimed:
            case ActiveTerminalDisposition::kProtocolClaimedOutputUnconsumed:
              if (output_consumed !=
                (disposition == ActiveTerminalDisposition::kRetainedAndFaultClaimed) ||
                publication)
              {
                fail_terminal_adapter_locked();
                return;
              }
              route_protocol_claim = prepare_terminal_fault_route_locked(
                protocol_claim, protocol_diagnostic_mutation_fenced);
              break;
            case ActiveTerminalDisposition::kRetainedWhileRouting:
            case ActiveTerminalDisposition::kRetainedUnsecured:
              if (!output_consumed || publication || protocol_claim) {
                fail_terminal_adapter_locked();
              }
              break;
            case ActiveTerminalDisposition::kContradictoryTerminalOutputUnconsumed:
            case ActiveTerminalDisposition::kIdentityMismatchOutputUnconsumed:
            case ActiveTerminalDisposition::kProtocolAlreadyPendingOutputUnconsumed:
            case ActiveTerminalDisposition::kAlreadyReservedOutputUnconsumed:
            case ActiveTerminalDisposition::kFeedbackOutstandingOutputUnconsumed:
            case ActiveTerminalDisposition::kAlreadyDeliveredOutputUnconsumed:
              if (output_consumed || publication || protocol_claim) {
                fail_terminal_adapter_locked();
              }
              break;
            case ActiveTerminalDisposition::kInvalidArgumentOutputUnconsumed:
              fail_terminal_adapter_locked();
              break;
          }
        }
      }
      if (observe_clock_failure) {
        (void)observe_steady_clock_failure(steady_clock_->sample());
        continue;
      }
      if (route_protocol_claim) {
        route_claimed_terminal_fault(
          protocol_claim,
          correlation_failure ? correlation_detail : invalid_contract_detail,
          protocol_diagnostic_mutation_fenced);
      }
      if (!valid_contract) {
        RCLCPP_ERROR(node_.get_logger(), "%s", invalid_contract_detail.data());
      } else if (correlation_failure) {
        RCLCPP_ERROR(node_.get_logger(), "%s", correlation_detail.data());
      }
      if (publication) {
        publish_terminal(*publication, std::move(handle), std::move(publisher), elapsed);
      }
      return;
    }
  }

  void deliver(CoordinatorDriverOutput & output)
  {
    constexpr std::string_view invalid_contract_detail =
      "driver produced an invalid output kind/outcome combination";
    constexpr std::string_view correlation_detail =
      "driver output does not match the active ROS action generation";
    const bool terminal_kind = output.kind != CoordinatorDriverOutputKind::kFeedback &&
      output.kind != CoordinatorDriverOutputKind::kInhibited;
    if (terminal_kind) {
      deliver_terminal(output);
      return;
    }
    if (!valid_output_contract(output)) {
      std::optional<CoordinatorActiveFaultClaim> protocol_claim;
      bool protocol_diagnostic_mutation_fenced = false;
      {
        std::lock_guard lock(binding_mutex_);
        protocol_claim = claim_active_protocol_fault_locked(
          protocol_diagnostic_mutation_fenced);
      }
      if (protocol_claim) {
        const auto route_clock = steady_clock_->sample();
        (void)complete_claimed_epoch_clock(
          *protocol_claim, route_clock.time, invalid_contract_detail,
          protocol_diagnostic_mutation_fenced);
        if (route_clock.provider_failed) {
          (void)observe_steady_clock_failure(route_clock);
        }
      }
      RCLCPP_ERROR(node_.get_logger(), "%s", invalid_contract_detail.data());
      return;
    }
    GoalHandlePtr handle;
    std::chrono::milliseconds elapsed{0};
    for (;; ) {
      const bool clock_failed_before_binding = steady_clock_->provider_failed();
      std::optional<CoordinatorActiveFaultClaim> correlation_claim;
      bool correlation_diagnostic_mutation_fenced = false;
      bool observe_clock_failure = false;
      {
        std::lock_guard lock(binding_mutex_);
        const auto epoch = active_epoch_snapshot_locked();
        if (!active_ || !active_->epoch || !epoch ||
          epoch->goal_generation != output.goal_generation ||
          epoch->terminal_phase == ActiveTerminalPhase::kDelivered)
        {
          correlation_claim = claim_active_protocol_fault_locked(
            correlation_diagnostic_mutation_fenced);
        } else if (clock_failed_before_binding && !steady_clock_failure_handled_) {
          observe_clock_failure = true;
        } else {
          switch (epoch->route_phase) {
            case ActiveFaultRoutePhase::kRoutingPending:
              return;
            case ActiveFaultRoutePhase::kUnsecured:
              return;
            case ActiveFaultRoutePhase::kSecured:
              return;
            case ActiveFaultRoutePhase::kClear:
              break;
          }
          handle = active_->handle;
          elapsed = active_->last_elapsed;
        }
      }
      if (correlation_claim) {
        const auto route_clock = steady_clock_->sample();
        (void)complete_claimed_epoch_clock(
          *correlation_claim, route_clock.time, correlation_detail,
          correlation_diagnostic_mutation_fenced);
        if (route_clock.provider_failed) {
          (void)observe_steady_clock_failure(route_clock);
        }
        RCLCPP_ERROR(node_.get_logger(), "%s", correlation_detail.data());
        return;
      }
      if (observe_clock_failure) {
        (void)observe_steady_clock_failure(steady_clock_->sample());
        continue;
      }
      if (!handle) {
        RCLCPP_ERROR(
          node_.get_logger(),
          "driver output arrived without an exact active ROS action binding");
        return;
      }
      break;
    }

    {
      if (output.kind == CoordinatorDriverOutputKind::kInhibited) {
        RCLCPP_ERROR(
          node_.get_logger(), "coordinator motion inhibition latched: %s",
          output.detail.c_str());
      } else if (output.kind == CoordinatorDriverOutputKind::kFeedback) {
        // Two bounded echo classes, one line each, never per pump tick: the reacquire hold's
        // boundaries (previously action feedback only, so a retained console could not tell an
        // unlive hold had even begun) and the task receipts that name which verification owned
        // the grasp coupling and which recovery attempt entered or resumed (Card 039).
        const bool task_receipt =
          output.detail.rfind("grasp coupling retained", 0) == 0 ||
          output.detail.rfind("recovery attempt", 0) == 0 ||
          output.detail.rfind("recovery rung", 0) == 0 ||
          output.detail.rfind("recoverable skip", 0) == 0 ||
          output.detail.rfind("whole-task steady deadline", 0) == 0 ||
          output.detail.rfind("recovery classification", 0) == 0;
        if (task_receipt) {
          RCLCPP_INFO(node_.get_logger(), "%s", output.detail.c_str());
        } else if (output.detail.find("reacquire") != std::string::npos) {
          RCLCPP_INFO(node_.get_logger(), "perception reacquire: %s", output.detail.c_str());
        }
      }
      std::optional<CoordinatorFeedbackPermit> feedback_permit;
      std::optional<CoordinatorActiveFaultClaim> feedback_fault;
      bool feedback_diagnostic_mutation_fenced = false;
      const bool clock_failed_before_feedback = steady_clock_->provider_failed();
      {
        std::lock_guard lock(binding_mutex_);
        const auto epoch = active_epoch_snapshot_locked();
        if (!active_ || !active_->epoch || !epoch ||
          epoch->goal_generation != output.goal_generation || clock_failed_before_feedback)
        {
          return;
        }
        auto reserved = active_->epoch->reserve_feedback(
          epoch->goal_id, epoch->goal_generation);
        auto permit = reserved.take_permit();
        if (reserved.status() != ActiveFeedbackReserveStatus::kReserved ||
          !permit || !permit->live())
        {
          invalidate_diagnostic_caches_locked();
          return;
        }
        handle = active_->handle;
        elapsed = active_->last_elapsed;
        feedback_permit = std::move(permit);
      }
      if (!feedback_permit) {
        return;
      }
      if (!handle) {
        {
          std::lock_guard lock(binding_mutex_);
          feedback_fault = complete_feedback_publication_failure_locked(
            *feedback_permit, output.goal_generation, feedback_diagnostic_mutation_fenced);
        }
        route_claimed_feedback_failure(feedback_fault, feedback_diagnostic_mutation_fenced);
        return;
      }
      try {
        auto feedback = make_action_feedback_message(
          output.transition, elapsed, output.selected_pair, output.outcome, output.detail);
        handle->publish_feedback(feedback);
        {
          std::lock_guard lock(binding_mutex_);
          const auto epoch = active_epoch_snapshot_locked();
          if (active_ && active_->epoch && epoch &&
            epoch->goal_generation == output.goal_generation)
          {
            (void)active_->epoch->feedback_returned(*feedback_permit);
            invalidate_diagnostic_caches_locked();
          }
        }
      } catch (const std::exception & error) {
        {
          std::lock_guard lock(binding_mutex_);
          feedback_fault = complete_feedback_publication_failure_locked(
            *feedback_permit, output.goal_generation, feedback_diagnostic_mutation_fenced);
        }
        route_claimed_feedback_failure(feedback_fault, feedback_diagnostic_mutation_fenced);
        RCLCPP_ERROR(node_.get_logger(), "action feedback publication failed: %s", error.what());
      } catch (...) {
        {
          std::lock_guard lock(binding_mutex_);
          feedback_fault = complete_feedback_publication_failure_locked(
            *feedback_permit, output.goal_generation, feedback_diagnostic_mutation_fenced);
        }
        route_claimed_feedback_failure(feedback_fault, feedback_diagnostic_mutation_fenced);
        RCLCPP_ERROR(node_.get_logger(), "action feedback publication failed");
      }
      return;
    }
  }

  void deliver_adapter_fault()
  {
    std::optional<std::string> fault;
    {
      std::lock_guard lock(binding_mutex_);
      if (!adapter_fault_ || !active_) {
        return;
      }
      // An adapter fault does not prove that the driver's reservation capability is settled.
      // Keep the binding and withhold finality until cleanup can prove settlement.
      if (!active_->epoch) {
        adapter_fail_stopped_ = true;
      }
      fault.swap(adapter_fault_);
    }
    RCLCPP_ERROR(node_.get_logger(), "coordinator adapter fault: %s", fault->c_str());
  }

  RestockActionCoordinatorNode & node_;
  std::string action_name_;
  std::string status_topic_;
  std::string planning_frame_;
  std::string workcell_root_frame_;
  std::string tool_frame_;
  std::string grasp_center_frame_;
  std::string gripper_geometry_path_;
  WorldStateServiceNames service_names_;
  std::string invalidate_lane_evidence_service_;
  std::string wrist_depth_topic_;
  std::string wrist_camera_info_topic_;
  std::string acquire_lane_observation_service_;
  bool reliable_wrist_depth_qos_{true};
  std::chrono::milliseconds pump_period_{20ms};
  std::chrono::milliseconds shutdown_timeout_{20000ms};
  CoordinatorStartupConfig startup_config_;
  std::size_t executor_threads_{3U};
  std::size_t generation_deposit_capacity_{64U};
  RestockCoordinatorDriverConfig driver_config_;
  SelectionConfig selection_config_;
  GraspGenerationConfig grasp_generation_config_;
  PlacementGenerationConfig placement_config_;
  GripperStagingGeometry gripper_geometry_;
  double grasp_transform_translation_tolerance_m_{1.0e-6};
  double grasp_transform_rotation_tolerance_rad_{1.0e-6};
  std::unique_ptr<ProductCollisionCatalog> product_catalog_;
  WorkcellManipulationGeometry workcell_geometry_;
  restocker_perception::WorkcellSurveyGeometry survey_geometry_;
  std::vector<restocker_perception::SurveyStation> survey_stations_;
  std::optional<Eigen::Isometry3d> world_from_shelf_;
  std::optional<Eigen::Isometry3d> tool0_from_grasp_center_;
  std::optional<std::string> transform_fault_detail_;
  std::string shelf_transform_observation_detail_{"waiting for shelf transform"};
  std::string tool_transform_observation_detail_{"waiting for verified gripper transform"};
  mutable std::mutex perception_mutex_;
  std::optional<rclcpp::Time> last_perception_acquisition_;
  std::optional<rclcpp::Time> last_perception_received_at_;
  std::uint64_t perception_arrivals_{0U};
  std::uint64_t perception_stamp_advances_{0U};
  // Touched only by the thread that runs the liveness predicate (the pump), so a live/lost
  // console line is emitted once per transition rather than once per coordinator period.
  std::optional<bool> perception_liveness_reported_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr wrist_depth_subscription_;
  rclcpp::Client<restocker_interfaces::srv::InvalidateLaneEvidence>::SharedPtr
    invalidate_lane_client_;
  rclcpp::Client<restocker_interfaces::srv::AcquireLaneObservation>::SharedPtr
    acquire_lane_client_;

  GoalAdmissionSlot admission_;
  CoordinatorTerminationRouter termination_router_;
  std::shared_ptr<CoordinatorSteadyClock> steady_clock_;
  std::shared_ptr<CoordinatorAsyncSteadyClockFailureLatch> async_clock_failure_latch_;
  std::shared_ptr<SteadyClockFailureObserver> clock_failure_observer_;
  std::function<void()> after_pending_goal_reserved_;
  std::function<void()> after_terminal_delivery_reserved_;
  std::function<void()> before_active_fault_route_;
  std::function<void()> before_accepted_goal_adoption_;
  std::function<void()> before_pending_handoff_;
  std::function<void()> after_callback_failure_;
  const std::shared_ptr<ActionResultPublisher> action_result_publisher_;
  rclcpp::CallbackGroup::SharedPtr ingress_group_;
  rclcpp::CallbackGroup::SharedPtr action_group_;
  rclcpp::CallbackGroup::SharedPtr orchestration_group_;
  rclcpp::CallbackGroup::SharedPtr completion_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<WorldStateAsyncPort> world_state_;
  std::unique_ptr<CoordinatorStartupGate> startup_gate_;
  std::shared_ptr<CoordinatorStartupMailbox> startup_mailbox_;
  // Declared before the driver so it outlives the raw pointer the driver holds.
  bool motion_enabled_{true};
  bool require_planning_scene_authority_{true};
  std::unique_ptr<MoveItMotionPort> motion_;
  std::unique_ptr<RosGripperPort> gripper_;
  std::unique_ptr<RosAttachmentPort> attachment_;
  // Declared before the driver for the same reason the motion backend is: the driver holds raw
  // pointers to them and must not outlive what they point at.
  bool reasoner_enabled_{false};
  restocker_reasoner::HttpRecoveryAdvisorConfig reasoner_config_;
  std::string reasoner_audit_path_;
  std::unique_ptr<restocker_reasoner::RecoveryAuditLog> recovery_audit_;
  std::unique_ptr<restocker_reasoner::HttpRecoveryAdvisor> recovery_advisor_;
  std::unique_ptr<RestockCoordinatorDriver> driver_;
  rclcpp::Publisher<restocker_interfaces::msg::RestockCoordinatorStatus>::SharedPtr
    startup_status_publisher_;
  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr immutable_parameters_callback_;

  mutable std::mutex binding_mutex_;
  mutable std::mutex startup_mutex_;
  std::optional<restocker_interfaces::msg::RestockCoordinatorStatus> last_startup_status_;
  std::atomic<bool> shutdown_started_{false};
  std::optional<PendingBinding> pending_;
  std::optional<ActiveBinding> active_;
  std::optional<std::string> adapter_fault_;
  bool steady_clock_failure_handled_{false};
  bool steady_clock_failure_secured_{false};
  bool steady_clock_failure_observation_recorded_{false};
  bool steady_clock_failure_first_observed_asynchronously_{false};
  bool preaccept_transaction_fault_{false};
  bool adapter_fail_stopped_{false};
  // Card 082, §6 Half A: the allocation-free receipt. Written without taking binding_mutex_ (the
  // failure may have arrived with it held) and without touching the heap, so it survives the
  // condition that caused the failure. Read by handle_goal's binding_unavailable and published on
  // generation_authority_snapshot(). `callback_failure_detail_` holds the address of a string
  // literal with static storage duration, never a copy.
  // Only an exact pre-seal protocol-failure route may finish its terminal handshake. A second
  // callback failure revokes this exception independently of allocation or binding-lock access.
  std::optional<GoalGeneration> callback_failure_cleanup_generation_;
  std::atomic<bool> callback_failure_repeated_{false};
  std::atomic<bool> callback_failure_latched_{false};
  std::atomic<const char *> callback_failure_detail_{nullptr};
  PendingBindingIncarnation last_pending_binding_incarnation_{0U};
  CoordinatorPumpLeaseGate pump_lease_gate_;
  std::optional<CoordinatorPumpLeaseReturnStatus> last_pump_return_status_;
  bool diagnostic_caches_initialized_{false};
  bool diagnostic_caches_fresh_{false};
  std::uint64_t diagnostic_cache_revision_{0U};
  std::size_t diagnostic_external_mutations_{0U};
  CachedCoordinatorInboxSnapshot cached_inbox_;
  CachedGoalAdmissionSnapshot cached_admission_;
  CachedRestockCoordinatorDriverSnapshot cached_driver_;
  GoalGeneration cached_goal_generation_{0U};
  AcceptedHandleAdoption accepted_handle_adoption_;
  CoordinatorShutdownStatus shutdown_status_{CoordinatorShutdownStatus::kRunning};
  std::optional<std::chrono::steady_clock::time_point> shutdown_deadline_;
};

RestockActionCoordinatorNode::RestockActionCoordinatorNode(const rclcpp::NodeOptions & options)
: RestockActionCoordinatorNode(options,
    RestockActionCoordinatorNodeDependencies{
    []() {return std::chrono::steady_clock::now();}, {}, {}})
{
}

RestockActionCoordinatorNode::RestockActionCoordinatorNode(
  const rclcpp::NodeOptions & options, RestockActionCoordinatorNodeDependencies dependencies)
: rclcpp::Node("restock_action_coordinator", options),
  impl_(std::make_unique<Impl>(*this, std::move(dependencies)))
{
}

RestockActionCoordinatorNode::~RestockActionCoordinatorNode() = default;

std::size_t RestockActionCoordinatorNode::executor_threads() const noexcept
{
  return impl_->executor_threads();
}

void RestockActionCoordinatorNode::request_shutdown() {impl_->request_shutdown();}

CoordinatorShutdownStatus RestockActionCoordinatorNode::shutdown_status() const noexcept
{
  return impl_->shutdown_status();
}

CoordinatorShutdownSnapshot RestockActionCoordinatorNode::shutdown_snapshot() const noexcept
{
  return impl_->shutdown_snapshot();
}

CoordinatorStartupSnapshot RestockActionCoordinatorNode::startup_snapshot() const
{
  return impl_->startup_snapshot();
}

CoordinatorSteadyClockFailureSnapshot RestockActionCoordinatorNode::steady_clock_failure_snapshot()
const
{
  return impl_->steady_clock_failure_snapshot();
}

CoordinatorGenerationAuthoritySnapshot RestockActionCoordinatorNode::generation_authority_snapshot()
const
{
  return impl_->generation_authority_snapshot();
}

}  // namespace restocker_task_executor
