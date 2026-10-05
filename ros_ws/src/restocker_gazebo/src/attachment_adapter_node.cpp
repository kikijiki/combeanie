// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/msg/planning_scene_lease.hpp>
#include <restocker_interfaces/msg/simulation_attachment_state.hpp>
#include <restocker_interfaces/srv/get_simulation_attachment_state.hpp>
#include <restocker_interfaces/srv/get_world_state.hpp>
#include <restocker_interfaces/srv/set_simulation_attachment.hpp>
#include <restocker_interfaces/srv/validate_planning_scene_lease.hpp>
#include <restocker_interfaces/srv/validate_task_reservation.hpp>

#include "restocker_gazebo/attachment_adapter_journal.hpp"
#include "restocker_gazebo/attachment_adapter_ros.hpp"
#include "restocker_gazebo/attachment_config.hpp"
#include "restocker_gazebo/attachment_transport_worker.hpp"

namespace restocker_gazebo
{
namespace
{

using namespace std::chrono_literals;
using GetAttachment = restocker_interfaces::srv::GetSimulationAttachmentState;
using GetWorldState = restocker_interfaces::srv::GetWorldState;
using ProjectionStatus = restocker_interfaces::msg::PlanningSceneProjectionStatus;
using SetAttachment = restocker_interfaces::srv::SetSimulationAttachment;
using ValidateLease = restocker_interfaces::srv::ValidatePlanningSceneLease;
using ValidateReservation = restocker_interfaces::srv::ValidateTaskReservation;

[[nodiscard]] std::chrono::milliseconds positive_milliseconds(
  std::int64_t value, const std::string & name)
{
  if (value <= 0) {
    throw std::invalid_argument(name + " must be positive");
  }
  return std::chrono::milliseconds(value);
}

[[nodiscard]] AdapterAuthorizationResult unavailable_authorization(std::string detail)
{
  return AdapterAuthorizationResult{
    AttachmentStatus{AttachmentStatusCode::kOutcomeUnknown, std::move(detail)}, std::nullopt};
}

class AttachmentAdapterNode final : public rclcpp::Node
{
public:
  AttachmentAdapterNode()
  : Node("simulation_attachment_adapter")
  {
    const auto gazebo_share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("restocker_gazebo"));
    const auto description_share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("restocker_description"));
    const auto boundary_path = declare_parameter<std::string>(
      "attachment_boundary_path", (gazebo_share / "config/attachment_boundary.yaml").string());
    const auto gripper_path = declare_parameter<std::string>(
      "gripper_geometry_path",
      (description_share / "config/gripper_geometry.yaml").string());
    const auto catalog_path = declare_parameter<std::string>(
      "product_catalog_path",
      (description_share / "config/product_collision_catalog.yaml").string());
    const auto scenario_path = declare_parameter<std::string>(
      "scenario_path", (gazebo_share / "config/baseline_products.yaml").string());
    auto loaded = load_attachment_boundary_config(
      boundary_path, gripper_path, catalog_path, scenario_path);
    if (!loaded) {
      throw std::invalid_argument(
              "invalid attachment adapter configuration: " +
              loaded.error().detail);
    }
    config_ = std::move(loaded.value());

    transport_timeout_ = positive_milliseconds(
      declare_parameter<std::int64_t>("transport_timeout_ms", 100),
      "transport_timeout_ms");
    validation_timeout_ = positive_milliseconds(
      declare_parameter<std::int64_t>("validation_timeout_ms", 2000),
      "validation_timeout_ms");
    reconciliation_timeout_ = positive_milliseconds(
      declare_parameter<std::int64_t>("reconciliation_timeout_ms", 5000),
      "reconciliation_timeout_ms");
    poll_period_ = positive_milliseconds(
      declare_parameter<std::int64_t>("poll_period_ms", 50), "poll_period_ms");
    watchdog_period_ = positive_milliseconds(
      declare_parameter<std::int64_t>("watchdog_period_ms", 250), "watchdog_period_ms");

    journal_ = std::make_unique<AttachmentAdapterJournal>(
      config_.journal_capacity, config_.detach_attempt_reserve);
    transport_ = std::make_unique<AttachmentTransportWorker>(
      config_.set_service, config_.query_service, transport_timeout_);
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    reservation_client_ = create_client<ValidateReservation>(
      declare_parameter<std::string>(
        "reservation_validation_service", "/world_state/validate_reservation"),
      rclcpp::ServicesQoS(), callback_group_);
    lease_client_ = create_client<ValidateLease>(
      declare_parameter<std::string>(
        "lease_validation_service", "/planning_scene_projection/validate_lease"),
      rclcpp::ServicesQoS(), callback_group_);
    snapshot_client_ = create_client<GetWorldState>(
      declare_parameter<std::string>("world_snapshot_service", "/world_state/get_snapshot"),
      rclcpp::ServicesQoS(), callback_group_);

    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    projector_subscription_ = create_subscription<ProjectionStatus>(
      declare_parameter<std::string>(
        "projection_status_topic", "/planning_scene_projection/status"),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](ProjectionStatus::SharedPtr message) {projector_status_ = std::move(*message);},
      options);
    state_publisher_ = create_publisher<restocker_interfaces::msg::SimulationAttachmentState>(
      "/simulation_attachment/state", rclcpp::QoS(1).reliable().transient_local());
    query_service_ = create_service<GetAttachment>(
      "/simulation_attachment/get_state",
      std::bind(
        &AttachmentAdapterNode::on_query, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    timer_ = create_wall_timer(
      20ms, std::bind(&AttachmentAdapterNode::on_timer, this), callback_group_);
    next_transport_poll_ = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "reconciling the simulator attachment boundary before serving mutations");
  }

private:
  enum class Stage
  {
    kStartup,
    kIdle,
    kNeedInitialValidation,
    kWaitingInitialValidation,
    kAwaitingTransport,
    kNeedTransportQuery,
    kNeedTerminalValidation,
    kWaitingTerminalValidation,
    kInconsistent,
  };

  struct ActiveOperation
  {
    std::string operation_id;
    AdapterMutationRequest request;
    std::chrono::steady_clock::time_point deadline;
    bool hard_deadline_reported{false};
  };

  void on_set(
    const SetAttachment::Request::SharedPtr request,
    SetAttachment::Response::SharedPtr response)
  {
    const auto converted = adapter_request_from_ros(*request);
    if (!converted.request) {
      populate_ros_response(
        AdapterReply{
            converted.status, false, AdapterAction::kNone, std::nullopt, std::nullopt},
        request->operation_id, *response);
      return;
    }
    auto reply = journal_->submit(request->operation_id, *converted.request);
    trace_reply(reply, request->operation_id, "mutation submission");
    if (reply.action == AdapterAction::kValidateInitialCapabilities) {
      active_ = ActiveOperation{
        request->operation_id, *converted.request,
        std::chrono::steady_clock::now() + validation_timeout_, false};
      stage_ = Stage::kNeedInitialValidation;
    }
    populate_ros_response(reply, request->operation_id, *response);
  }

  void on_query(
    const GetAttachment::Request::SharedPtr request,
    GetAttachment::Response::SharedPtr response)
  {
    auto reply = journal_->query(request->operation_id);
    if (!request->expected_simulator_epoch.empty() && reply.state &&
      request->expected_simulator_epoch != reply.state->simulator_epoch)
    {
      reply.status = AttachmentStatus{
        AttachmentStatusCode::kSimulatorEpochChanged,
        "requested simulator epoch does not match the adapter observation"};
    }
    populate_ros_response(reply, request->operation_id, *response);
  }

  void on_timer()
  {
    if (stage_ == Stage::kStartup) {
      drive_startup();
      return;
    }
    if (auto completion = transport_->take_completion()) {
      handle_transport_completion(std::move(*completion));
    }
    const auto now = std::chrono::steady_clock::now();
    if (active_ && now >= active_->deadline) {
      handle_deadline();
    }
    if (stage_ == Stage::kNeedInitialValidation || stage_ == Stage::kNeedTerminalValidation) {
      request_capability_validation();
      return;
    }
    if (stage_ == Stage::kNeedTransportQuery && now >= next_transport_poll_ &&
      !transport_->busy())
    {
      if (transport_->submit_query(simulator_epoch_, active_->operation_id)) {
        stage_ = Stage::kAwaitingTransport;
      }
      return;
    }
    if (stage_ == Stage::kIdle && now >= next_transport_poll_ && !transport_->busy()) {
      if (transport_->submit_query(simulator_epoch_)) {
        next_transport_poll_ = now + watchdog_period_;
      }
    }
  }

  // True once the projector's report is a verdict rather than a waypoint. A projection that holds
  // or is moving a lease is settled and dirty: another transaction owns the scene, a cross-system
  // inconsistency this adapter must refuse to start over. Startup, synchronization and
  // degradation are states the projector leaves by itself.
  [[nodiscard]] static bool settled_projection(const ProjectionStatus & status) noexcept
  {
    using Lease = restocker_interfaces::msg::PlanningSceneLease;
    if (status.lease_id != 0 || status.lease_phase != Lease::PHASE_NONE) {
      return true;
    }
    switch (status.state) {
      case ProjectionStatus::STATE_APPLIED:
      case ProjectionStatus::STATE_TRANSACTION_DRAINING:
      case ProjectionStatus::STATE_TRANSACTION_HELD:
      case ProjectionStatus::STATE_TRANSACTION_RELEASING:
        return true;
      default:
        return false;
    }
  }

  void drive_startup()
  {
    if (auto completion = transport_->take_completion()) {
      if (completion->result.reply) {
        startup_physical_state_ = completion->result.reply->state;
        simulator_epoch_ = startup_physical_state_->simulator_epoch;
      }
    }
    if (!startup_physical_state_ && !transport_->busy()) {
      static_cast<void>(transport_->submit_query("attachment-adapter-discovery"));
    }
    // A world-state snapshot request can be accepted and never answered (the server logs "failed
    // to send response ... client will not receive response" when its reply times out under
    // startup load). Without a deadline the outstanding flag stays set and this adapter never
    // reconciles or offers its mutation service.
    const auto startup_now = std::chrono::steady_clock::now();
    if (snapshot_requested_ && startup_now >= snapshot_request_deadline_) {
      ++snapshot_generation_;
      snapshot_requested_ = false;
      RCLCPP_WARN(
        get_logger(), "startup world-state query went unanswered; retrying");
    }
    if (!startup_snapshot_ && !snapshot_requested_ && snapshot_client_->service_is_ready()) {
      snapshot_requested_ = true;
      snapshot_request_deadline_ = startup_now + reconciliation_timeout_;
      const auto generation = ++snapshot_generation_;
      auto request = std::make_shared<GetWorldState::Request>();
      snapshot_client_->async_send_request(
        request, [this, generation](rclcpp::Client<GetWorldState>::SharedFuture future) {
          if (generation != snapshot_generation_) {
            return;                     // a retry already replaced this attempt
          }
          try {
            startup_snapshot_ = future.get()->snapshot;
          } catch (const std::exception & error) {
            RCLCPP_WARN(get_logger(), "startup world-state query failed: %s", error.what());
          }
          snapshot_requested_ = false;
        });
    }
    if (!startup_physical_state_ || !startup_snapshot_ || !projector_status_) {
      return;
    }
    // The projector converges on its own after this node comes up: STARTING, SYNCHRONIZING, then
    // DEGRADED while the world state and MoveIt services are still arriving, then APPLIED.
    // Latching a permanent inconsistency from whichever status arrived first made attachment
    // unavailable for the whole session. Anything short of a settled projection is "not yet
    // decided", and the caller's bounded wait turns endless convergence into a failure.
    if (!settled_projection(*projector_status_)) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "attachment startup is waiting for the planning-scene projection to settle: state=%u "
        "detail=%s", static_cast<unsigned>(projector_status_->state),
        projector_status_->detail.c_str());
      return;
    }
    const bool clean_world = !startup_snapshot_->has_active_reservation &&
      !startup_snapshot_->robot.has_held_object;
    const bool clean_projection = projector_status_->state == ProjectionStatus::STATE_APPLIED &&
      projector_status_->lease_id == 0 &&
      projector_status_->lease_phase == restocker_interfaces::msg::PlanningSceneLease::PHASE_NONE;
    const auto reply = journal_->initialize(
      *startup_physical_state_, clean_world && clean_projection);
    publish(reply, {});
    if (reply.status.code != AttachmentStatusCode::kDetached) {
      stage_ = Stage::kInconsistent;
      RCLCPP_ERROR(
        get_logger(), "attachment startup reconciliation failed: %s",
        reply.status.detail.c_str());
      return;
    }
    mutation_service_ = create_service<SetAttachment>(
      "/simulation_attachment/set",
      std::bind(
        &AttachmentAdapterNode::on_set, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    stage_ = Stage::kIdle;
    next_transport_poll_ = std::chrono::steady_clock::now() + watchdog_period_;
    RCLCPP_INFO(get_logger(), "simulation attachment adapter is reconciled and available");
  }

  void request_capability_validation()
  {
    if (!active_ || !reservation_client_->service_is_ready() ||
      !lease_client_->service_is_ready())
    {
      return;
    }
    const bool terminal = stage_ == Stage::kNeedTerminalValidation;
    validation_generation_++;
    const auto generation = validation_generation_;
    reservation_validation_.reset();
    lease_validation_.reset();
    auto reservation = std::make_shared<ValidateReservation::Request>();
    reservation->token = active_->request.reservation_token;
    reservation_client_->async_send_request(
      reservation, [this, generation](rclcpp::Client<ValidateReservation>::SharedFuture future) {
        if (generation == validation_generation_) {
          try {
            reservation_validation_ = *future.get();
            finish_capability_validation_if_ready();
          } catch (const std::exception & error) {
            fail_capability_validation(error.what());
          }
        }
      });
    auto lease = std::make_shared<ValidateLease::Request>();
    lease->token = active_->request.planning_scene_lease_token;
    lease_client_->async_send_request(
      lease, [this, generation](rclcpp::Client<ValidateLease>::SharedFuture future) {
        if (generation == validation_generation_) {
          try {
            lease_validation_ = *future.get();
            finish_capability_validation_if_ready();
          } catch (const std::exception & error) {
            fail_capability_validation(error.what());
          }
        }
      });
    stage_ = terminal ? Stage::kWaitingTerminalValidation : Stage::kWaitingInitialValidation;
  }

  void finish_capability_validation_if_ready()
  {
    if (!reservation_validation_ || !lease_validation_ || !active_) {
      return;
    }
    const bool terminal = stage_ == Stage::kWaitingTerminalValidation;
    const auto authorization = adapter_authorization_from_ros(
      active_->request.command, active_->request.object_id, *reservation_validation_,
      *lease_validation_, config_);
    const auto operation_id = active_->operation_id;
    auto reply = terminal ?
      journal_->complete_terminal_authorization(operation_id, authorization) :
      journal_->complete_initial_authorization(operation_id, authorization);
    handle_adapter_reply(std::move(reply), operation_id);
  }

  void fail_capability_validation(const std::string & detail)
  {
    if (!active_ || (stage_ != Stage::kWaitingInitialValidation &&
      stage_ != Stage::kWaitingTerminalValidation))
    {
      return;
    }
    validation_generation_++;
    const bool terminal = stage_ == Stage::kWaitingTerminalValidation;
    const auto operation_id = active_->operation_id;
    const auto unavailable = unavailable_authorization(
      "capability validation call failed: " + detail);
    auto reply = terminal ?
      journal_->complete_terminal_authorization(operation_id, unavailable) :
      journal_->complete_initial_authorization(operation_id, unavailable);
    handle_adapter_reply(std::move(reply), operation_id);
  }

  void handle_adapter_reply(AdapterReply reply, const std::string & operation_id)
  {
    publish(reply, operation_id);
    if (reply.action == AdapterAction::kSubmitTransportCommand && reply.transport_request) {
      active_->deadline = std::chrono::steady_clock::now() + reconciliation_timeout_;
      if (!transport_->submit_command(simulator_epoch_, operation_id, *reply.transport_request)) {
        auto failed = journal_->mark_transport_inconsistency(
          operation_id, "transport worker rejected an authorized command");
        publish(failed, operation_id);
        stage_ = Stage::kInconsistent;
        return;
      }
      const auto submitted = journal_->mark_transport_submitted(operation_id);
      publish(submitted, operation_id);
      stage_ = Stage::kAwaitingTransport;
      return;
    }
    if (reply.action == AdapterAction::kQueryTransportOperation) {
      stage_ = Stage::kNeedTransportQuery;
      next_transport_poll_ = std::chrono::steady_clock::now() + poll_period_;
      return;
    }
    if (reply.action == AdapterAction::kValidateTerminalCapabilities) {
      stage_ = Stage::kNeedTerminalValidation;
      return;
    }
    if ((reply.status.code == AttachmentStatusCode::kExternalInconsistency ||
      reply.status.code == AttachmentStatusCode::kOutcomeUnknown) &&
      journal_->motion_inhibited())
    {
      stage_ = Stage::kInconsistent;
      return;
    }
    if (reply.status.code != AttachmentStatusCode::kPending) {
      active_.reset();
      stage_ = Stage::kIdle;
    }
  }

  void handle_transport_completion(AttachmentTransportCompletion completion)
  {
    if (stage_ == Stage::kIdle && completion.operation_id.empty()) {
      if (completion.result.reply) {
        publish(journal_->observe_idle_state(completion.result.reply->state), {});
      } else if (completion.result.outcome == AttachmentTransportOutcome::kMalformedReply) {
        publish(journal_->mark_idle_inconsistency(completion.result.status.detail), {});
        stage_ = Stage::kInconsistent;
      }
      return;
    }
    if (!active_ || completion.operation_id != active_->operation_id) {
      return;
    }
    if (completion.result.reply) {
      handle_adapter_reply(
        journal_->observe_transport_reply(active_->operation_id, *completion.result.reply),
        active_->operation_id);
    } else if (completion.result.outcome == AttachmentTransportOutcome::kMalformedReply) {
      handle_adapter_reply(
        journal_->mark_transport_inconsistency(
          active_->operation_id, completion.result.status.detail),
        active_->operation_id);
    } else {
      stage_ = Stage::kNeedTransportQuery;
      next_transport_poll_ = std::chrono::steady_clock::now() + poll_period_;
    }
  }

  void handle_deadline()
  {
    if (!active_) {
      return;
    }
    if (stage_ == Stage::kNeedInitialValidation || stage_ == Stage::kWaitingInitialValidation) {
      validation_generation_++;
      handle_adapter_reply(
        journal_->complete_initial_authorization(
          active_->operation_id,
          unavailable_authorization("initial capability validation deadline exceeded")),
        active_->operation_id);
      return;
    }
    if (stage_ == Stage::kNeedTerminalValidation ||
      stage_ == Stage::kWaitingTerminalValidation)
    {
      validation_generation_++;
      handle_adapter_reply(
        journal_->complete_terminal_authorization(
          active_->operation_id,
          unavailable_authorization("terminal capability validation deadline exceeded")),
        active_->operation_id);
      return;
    }
    if (!active_->hard_deadline_reported) {
      active_->hard_deadline_reported = true;
      handle_adapter_reply(
        journal_->mark_reconciliation_deadline_exceeded(
          active_->operation_id, "attachment transport reconciliation deadline exceeded"),
        active_->operation_id);
    }
  }

  // Motion inhibition is latched here but only reported to the coordinator on the next mutation it
  // refuses, by which time the causing observation is gone. Naming the cause when it latches is
  // what makes a failed run diagnosable.
  void trace_reply(const AdapterReply & reply, const std::string & operation_id, const char * site)
  {
    const bool inhibited = journal_->motion_inhibited();
    // Accepting a mutation inhibits motion for the duration of the transaction; that is routine.
    // Only a latch that survives with no work left to drive is reportable.
    const bool newly_inhibited = inhibited && !motion_inhibited_reported_ &&
      reply.action == AdapterAction::kNone;
    motion_inhibited_reported_ = inhibited;
    const bool faulted = reply.status.code == AttachmentStatusCode::kExternalInconsistency ||
      reply.status.code == AttachmentStatusCode::kOutcomeUnknown;
    if (!newly_inhibited && !faulted) {
      return;
    }
    RCLCPP_WARN(
      get_logger(),
      "attachment adapter %s at %s: \"%s\" (operation \"%s\", motion_inhibited=%s, "
      "simulator phase=%u gate=%u status=\"%s\")",
      newly_inhibited ? "latched motion inhibition" : "reported a fault", site,
      reply.status.detail.c_str(), operation_id.empty() ? "<idle>" : operation_id.c_str(),
      inhibited ? "true" : "false",
      reply.state ? static_cast<unsigned>(reply.state->phase) : 0U,
      reply.state ? static_cast<unsigned>(reply.state->motion_gate) : 0U,
      reply.state ? reply.state->status.detail.c_str() : "");
  }

  void publish(const AdapterReply & reply, const std::string & operation_id)
  {
    trace_reply(reply, operation_id, "publish");
    if (!reply.state) {
      return;
    }
    try {
      state_publisher_->publish(to_ros_message(reply, operation_id));
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "attachment state serialization failed: %s", error.what());
    }
  }

  AttachmentBoundaryConfig config_;
  std::unique_ptr<AttachmentAdapterJournal> journal_;
  std::unique_ptr<AttachmentTransportWorker> transport_;
  std::chrono::milliseconds transport_timeout_{100};
  std::chrono::milliseconds validation_timeout_{2000};
  std::chrono::milliseconds reconciliation_timeout_{5000};
  std::chrono::milliseconds poll_period_{50};
  std::chrono::milliseconds watchdog_period_{250};
  Stage stage_{Stage::kStartup};
  // Mirrors the journal's own latch so a transition is logged once, not on every reply.
  bool motion_inhibited_reported_{true};
  std::optional<ActiveOperation> active_;
  std::string simulator_epoch_;
  std::optional<AttachmentPhysicalState> startup_physical_state_;
  std::optional<restocker_interfaces::msg::WorldStateSnapshot> startup_snapshot_;
  std::optional<ProjectionStatus> projector_status_;
  bool snapshot_requested_{false};
  std::uint64_t snapshot_generation_{0};
  std::chrono::steady_clock::time_point snapshot_request_deadline_{};
  std::uint64_t validation_generation_{0};
  std::optional<ValidateReservation::Response> reservation_validation_;
  std::optional<ValidateLease::Response> lease_validation_;
  std::chrono::steady_clock::time_point next_transport_poll_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<ValidateReservation>::SharedPtr reservation_client_;
  rclcpp::Client<ValidateLease>::SharedPtr lease_client_;
  rclcpp::Client<GetWorldState>::SharedPtr snapshot_client_;
  rclcpp::Subscription<ProjectionStatus>::SharedPtr projector_subscription_;
  rclcpp::Publisher<restocker_interfaces::msg::SimulationAttachmentState>::SharedPtr
    state_publisher_;
  rclcpp::Service<GetAttachment>::SharedPtr query_service_;
  rclcpp::Service<SetAttachment>::SharedPtr mutation_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace
}  // namespace restocker_gazebo

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<restocker_gazebo::AttachmentAdapterNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("simulation_attachment_adapter"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
