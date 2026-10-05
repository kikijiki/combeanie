// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "restocker_gazebo/attachment_journal.hpp"

namespace restocker_gazebo
{

enum class AdapterReservationStage : std::uint8_t
{
  kUnknown = 0,
  kReserved = 1,
  kAttached = 2,
  kDetached = 3,
};

enum class AdapterLeasePhase : std::uint8_t
{
  kUnknown = 0,
  kHeld = 1,
};

enum class AdapterAction : std::uint8_t
{
  kNone = 0,
  kValidateInitialCapabilities = 1,
  kSubmitTransportCommand = 2,
  kQueryTransportOperation = 3,
  kValidateTerminalCapabilities = 4,
};

struct AdapterMutationRequest
{
  AttachmentCommand command{AttachmentCommand::kUnset};
  std::uint64_t object_id{0};
  std::string reservation_token;
  std::string planning_scene_lease_token;
  bool has_expected_grasp{false};
  CanonicalPose expected_grasp_center_to_child;
  // Forwarded verbatim to the boundary, which sizes its fidelity budget from it. Row major in
  // geometry_msgs/PoseWithCovariance order; all zero means the caller stated no uncertainty.
  std::array<double, 36> expected_pose_covariance{};
};

struct AdapterAuthorizationEvidence
{
  std::uint64_t reservation_id{0};
  std::uint64_t object_id{0};
  std::string object_source_id;
  AdapterReservationStage reservation_stage{AdapterReservationStage::kUnknown};
  std::uint64_t lease_id{0};
  AdapterLeasePhase lease_phase{AdapterLeasePhase::kUnknown};
  AttachmentIdentity resolved_identity;
};

struct AdapterAuthorizationResult
{
  AttachmentStatus status;
  std::optional<AdapterAuthorizationEvidence> evidence;
};

struct AdapterReply
{
  AttachmentStatus status;
  bool replayed{false};
  AdapterAction action{AdapterAction::kNone};
  std::optional<AttachmentRequest> transport_request;
  std::optional<AttachmentPhysicalState> state;
};

// Pure, single-active-operation journal for the ROS authorization boundary. Capability values are
// retained only in private records and never appear in AdapterReply or the Gazebo request.
class AttachmentAdapterJournal
{
public:
  explicit AttachmentAdapterJournal(std::size_t capacity, std::size_t detach_attempt_reserve = 2);

  [[nodiscard]] AdapterReply initialize(
    const AttachmentPhysicalState & simulator_state,
    bool cross_system_consistent);
  [[nodiscard]] AdapterReply submit(
    std::string operation_id,
    const AdapterMutationRequest & request);
  [[nodiscard]] AdapterReply complete_initial_authorization(
    std::string_view operation_id,
    const AdapterAuthorizationResult & authorization);
  [[nodiscard]] AdapterReply mark_transport_submitted(std::string_view operation_id);
  [[nodiscard]] AdapterReply observe_transport_reply(
    std::string_view operation_id,
    const AttachmentJournalReply & transport_reply);
  [[nodiscard]] AdapterReply complete_terminal_authorization(
    std::string_view operation_id,
    const AdapterAuthorizationResult & authorization);
  [[nodiscard]] AdapterReply mark_reconciliation_deadline_exceeded(
    std::string_view operation_id,
    std::string detail);
  [[nodiscard]] AdapterReply mark_transport_inconsistency(
    std::string_view operation_id,
    std::string detail);
  [[nodiscard]] AdapterReply observe_idle_state(
    const AttachmentPhysicalState & simulator_state);
  [[nodiscard]] AdapterReply mark_idle_inconsistency(std::string detail);

  [[nodiscard]] AdapterReply query(std::string_view operation_id = {}) const;
  [[nodiscard]] bool motion_inhibited() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  // Diagnostic incarnation id: fresh per construction, never a wire fence.
  [[nodiscard]] JournalRetentionSnapshot retention_snapshot() const;

private:
  enum class Phase : std::uint8_t
  {
    kInitialAuthorization,
    kReadyForTransport,
    kAwaitingTransport,
    kReconcilingTransport,
    kTerminalAuthorization,
    kTerminal,
    kInconsistent,
  };

  struct Record
  {
    std::string operation_id;
    AdapterMutationRequest request;
    Phase phase{Phase::kInitialAuthorization};
    AttachmentStatus status;
    std::optional<AttachmentRequest> transport_request;
    std::optional<AttachmentPhysicalState> state;
  };

  [[nodiscard]] AdapterReply reply(
    const AttachmentStatus & status, const Record * record = nullptr,
    bool replayed = false, AdapterAction action = AdapterAction::kNone) const;
  [[nodiscard]] Record * active(std::string_view operation_id);
  [[nodiscard]] const Record * find(std::string_view operation_id) const;
  [[nodiscard]] bool authorization_matches(
    const Record & record,
    const AdapterAuthorizationEvidence & evidence) const;
  [[nodiscard]] bool can_accept(std::size_t required_slots) const noexcept;
  void release_detach_reserve();
  void latch_inconsistency(Record & record, std::string detail);

  std::size_t capacity_;
  std::size_t detach_attempt_reserve_;
  std::size_t reserved_detach_slots_{0};
  std::string instance_id_;
  std::optional<std::string> cleanup_attach_operation_id_;
  std::unordered_map<std::string, Record> records_;
  std::optional<std::string> active_operation_id_;
  bool initialized_{false};
  bool motion_inhibited_{true};
  std::optional<AttachmentPhysicalState> latest_state_;
};

}  // namespace restocker_gazebo
