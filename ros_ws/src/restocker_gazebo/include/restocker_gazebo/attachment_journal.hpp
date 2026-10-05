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

namespace restocker_gazebo
{

enum class AttachmentCommand : std::uint8_t
{
  kUnset = 0,
  kAttach = 1,
  kDetach = 2,
};

enum class AttachmentStatusCode : std::uint16_t
{
  kUnset = 0,
  kPending = 1,
  kAttached = 2,
  kDetached = 3,
  kInvalidArgument = 4,
  kAuthorizationFailed = 5,
  kExternalInconsistency = 6,
  kConflict = 7,
  kTokenMismatch = 8,
  kIdempotencyConflict = 9,
  kResourceExhausted = 10,
  kObjectNotFound = 11,
  kEntityAmbiguous = 12,
  kGripperNotReady = 13,
  kOutOfTolerance = 14,
  kStateMismatch = 15,
  kSimulatorEpochChanged = 16,
  kOutcomeUnknown = 17,
  kOperationNotFound = 18,
  kInternalError = 255,
};

enum class AttachmentPhase : std::uint8_t
{
  kUnknown = 0,
  kDetached = 1,
  kValidatingAttach = 2,
  kApplyingAttach = 3,
  kVerifyingAttach = 4,
  kAttached = 5,
  kValidatingDetach = 6,
  kApplyingDetach = 7,
  kVerifyingDetach = 8,
  kInconsistent = 9,
};

enum class AttachmentMotionGate : std::uint8_t
{
  kUnknown = 0,
  kInhibited = 1,
  kValid = 2,
};

struct AttachmentStatus
{
  AttachmentStatusCode code{AttachmentStatusCode::kUnset};
  std::string detail;

  bool operator==(const AttachmentStatus &) const = default;
};

// Translation followed by a canonical unit quaternion in x, y, z, w order.
struct CanonicalPose
{
  std::array<double, 3> translation{};
  std::array<double, 4> rotation_xyzw{0.0, 0.0, 0.0, 1.0};

  bool operator==(const CanonicalPose &) const = default;
};

[[nodiscard]] std::optional<CanonicalPose> canonicalize_pose(
  const std::array<double, 3> & translation, const std::array<double, 4> & rotation_xyzw,
  double unit_tolerance = 1.0e-6);

struct AttachmentIdentity
{
  std::uint64_t object_id{0};
  std::string object_source_id;
  std::string parent_model;
  std::string parent_link;
  std::string child_model;
  std::string child_link;

  bool operator==(const AttachmentIdentity &) const = default;
};

struct AttachmentRequest
{
  AttachmentCommand command{AttachmentCommand::kUnset};
  std::uint64_t reservation_id{0};
  AttachmentIdentity identity;
  bool has_expected_grasp{false};
  CanonicalPose expected_grasp_center_to_child;
  // Covariance of the observation the expected grasp was derived from, row major in
  // geometry_msgs/PoseWithCovariance order (x, y, z, rotation about X, Y, Z). All zero means the
  // caller stated no uncertainty; the boundary then holds the grasp to its configured constant.
  std::array<double, 36> expected_pose_covariance{};

  bool operator==(const AttachmentRequest &) const = default;
};

// What the fidelity assertion measured and the budget it was measured against. Recorded whenever
// the assertion is evaluated, including passes.
struct AttachmentFidelity
{
  double translation_residual_m{0.0};
  double rotation_residual_rad{0.0};
  double translation_budget_m{0.0};
  double claimed_translation_sigma_m{0.0};
  double translation_ceiling_m{0.0};

  bool operator==(const AttachmentFidelity &) const = default;
};

struct AttachmentEvidence
{
  bool joint_observed{false};
  CanonicalPose parent_to_child;
  CanonicalPose child_pose_in_world;
  std::array<double, 6> relative_twist_in_parent{};
  std::uint64_t simulator_iteration{0};
  std::int64_t simulation_time_ns{0};

  bool operator==(const AttachmentEvidence &) const = default;
};

struct AttachmentOperationRecord
{
  std::string simulator_epoch;
  std::uint64_t sequence{0};
  std::string operation_id;
  AttachmentRequest request;
  AttachmentPhase phase{AttachmentPhase::kUnknown};
  AttachmentStatus status;
  bool mutation_started{false};
  bool started_from_inconsistent{false};
  std::optional<AttachmentEvidence> evidence;
  std::optional<AttachmentFidelity> fidelity;

  bool operator==(const AttachmentOperationRecord &) const = default;
};

struct AttachmentPhysicalState
{
  std::string simulator_epoch;
  std::uint64_t sequence{0};
  AttachmentPhase phase{AttachmentPhase::kUnknown};
  AttachmentMotionGate motion_gate{AttachmentMotionGate::kUnknown};
  AttachmentStatus status;
  std::optional<AttachmentIdentity> attached_identity;
  std::optional<AttachmentEvidence> evidence;
  // Fidelity of the most recent attach that reached the comparison, published on rejection as
  // well as acceptance.
  std::optional<AttachmentFidelity> fidelity;

  bool operator==(const AttachmentPhysicalState &) const = default;
};

struct AttachmentJournalReply
{
  AttachmentStatus status;
  bool replayed{false};
  std::optional<AttachmentOperationRecord> operation;
  AttachmentPhysicalState state;

  bool operator==(const AttachmentJournalReply &) const = default;
};

// Read-only description of one journal's retention bound; counts and identifiers only, never an
// operation ID, capability or payload. `open_obligations` counts journal RECORDS only, and
// `open_obligations + terminal_receipts == size`. An open record is an unsettled side effect, a
// reserved credit's owner, or an unknown outcome; a terminal receipt is a settled record kept only
// so an exact retry replays. An obligation carried as state with no record (a held attachment
// recovered after an epoch rotation) is reported by `held_attachment`, so open_obligations == 0
// must never be read as "nothing owed". `reserved_credits` are not part of `size`. `inhibited`
// is the journal's own motion-gate flag (simulator: gate != valid; adapter: motion_inhibited_).
// Operation journals never evict: `evicting` is false and a full journal refuses admission.
// `epoch_id` names the incarnation or epoch that owns the journal; diagnostic, not a wire field.
struct JournalRetentionSnapshot
{
  std::string journal;
  std::string epoch_id;
  std::size_t capacity{0};
  std::size_t size{0};
  std::size_t open_obligations{0};
  std::size_t terminal_receipts{0};
  std::size_t reserved_credits{0};
  // A physical attachment is held according to this journal's state, whether or not a record
  // still references it.
  bool held_attachment{false};
  bool inhibited{false};
  bool evicting{false};
};

class AttachmentJournal
{
public:
  explicit AttachmentJournal(
    std::string simulator_epoch, std::size_t capacity,
    std::optional<AttachmentIdentity> recovered_attachment = std::nullopt,
    std::size_t detach_attempt_reserve = 2);

  [[nodiscard]] AttachmentJournalReply submit(
    std::string operation_id,
    const AttachmentRequest & request);
  [[nodiscard]] AttachmentJournalReply mark_validation_succeeded(
    std::string_view operation_id,
    std::optional<AttachmentFidelity> fidelity = std::nullopt);
  [[nodiscard]] AttachmentJournalReply reject_before_mutation(
    std::string_view operation_id,
    AttachmentStatus rejection,
    std::optional<AttachmentFidelity> fidelity = std::nullopt);
  [[nodiscard]] AttachmentJournalReply mark_mutation_started(std::string_view operation_id);
  [[nodiscard]] AttachmentJournalReply mark_verification_succeeded(
    std::string_view operation_id, const AttachmentEvidence & evidence);
  [[nodiscard]] AttachmentJournalReply mark_outcome_unknown(
    std::string_view operation_id,
    std::string detail);
  [[nodiscard]] AttachmentJournalReply mark_external_inconsistency(
    std::string detail,
    std::optional<AttachmentEvidence> evidence = std::nullopt);

  [[nodiscard]] AttachmentJournalReply query(std::string_view operation_id = {}) const;
  void rotate_epoch(
    std::string simulator_epoch,
    std::optional<AttachmentIdentity> recovered_attachment = std::nullopt);

  [[nodiscard]] const AttachmentPhysicalState & state() const noexcept;
  [[nodiscard]] std::size_t size() const noexcept;
  [[nodiscard]] std::size_t capacity() const noexcept;
  [[nodiscard]] std::size_t reserved_detach_slots() const noexcept;
  [[nodiscard]] JournalRetentionSnapshot retention_snapshot() const;

private:
  [[nodiscard]] AttachmentJournalReply reply(
    const AttachmentStatus & status,
    const AttachmentOperationRecord * operation = nullptr,
    bool replayed = false) const;
  [[nodiscard]] AttachmentOperationRecord * active_record(std::string_view operation_id);
  [[nodiscard]] const AttachmentOperationRecord * find(std::string_view operation_id) const;
  [[nodiscard]] bool can_accept(std::size_t required_slots) const noexcept;
  [[nodiscard]] std::uint64_t next_sequence() noexcept;
  void restore_idle_state(const AttachmentStatus & status);

  std::size_t capacity_;
  std::size_t detach_attempt_reserve_;
  std::size_t reserved_detach_slots_{0};
  std::uint64_t sequence_{0};
  std::unordered_map<std::string, AttachmentOperationRecord> records_;
  std::optional<std::string> active_operation_id_;
  AttachmentPhysicalState state_;
};

}  // namespace restocker_gazebo
