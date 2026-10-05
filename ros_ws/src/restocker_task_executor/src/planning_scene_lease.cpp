// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/planning_scene_lease.hpp"

#include <sys/random.h>

#include <array>
#include <cerrno>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace restocker_task_executor
{
namespace
{

constexpr std::size_t kMaximumOperationIdLength = 128;
constexpr std::size_t kMinimumTokenLength = 32;

[[nodiscard]] bool valid_operation_id(const std::string & value)
{
  return !value.empty() && value.size() <= kMaximumOperationIdLength;
}

[[nodiscard]] bool constant_time_equal(const std::string & left, const std::string & right)
{
  const std::size_t maximum_size = left.size() > right.size() ? left.size() : right.size();
  std::size_t difference = left.size() ^ right.size();
  for (std::size_t index = 0; index < maximum_size; ++index) {
    const unsigned char left_byte =
      index < left.size() ? static_cast<unsigned char>(left[index]) : 0U;
    const unsigned char right_byte =
      index < right.size() ? static_cast<unsigned char>(right[index]) : 0U;
    difference |= static_cast<std::size_t>(left_byte ^ right_byte);
  }
  return difference == 0;
}

[[nodiscard]] std::optional<std::string> secure_token()
{
  std::array<unsigned char, 16> bytes{};
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const ssize_t count = getrandom(bytes.data() + offset, bytes.size() - offset, GRND_NONBLOCK);
    if (count < 0) {
      if (errno == EINTR) {
        continue;
      }
      return std::nullopt;
    }
    if (count == 0) {
      return std::nullopt;
    }
    offset += static_cast<std::size_t>(count);
  }

  constexpr char kHex[] = "0123456789abcdef";
  std::string token(bytes.size() * 2, '0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    token[2 * index] = kHex[bytes[index] >> 4U];
    token[2 * index + 1] = kHex[bytes[index] & 0x0fU];
  }
  return token;
}

[[nodiscard]] bool is_read_only(ProjectorStage stage)
{
  return stage == ProjectorStage::Snapshot || stage == ProjectorStage::CurrentScene;
}

[[nodiscard]] bool is_side_effecting(ProjectorStage stage)
{
  return stage == ProjectorStage::ApplyScene || stage == ProjectorStage::VerifyScene;
}

[[nodiscard]] AcquisitionAction action_for(ProjectorStage stage)
{
  if (is_read_only(stage)) {
    return AcquisitionAction::InvalidateReadOnlyGeneration;
  }
  if (is_side_effecting(stage)) {
    return AcquisitionAction::DrainSideEffectingGeneration;
  }
  return AcquisitionAction::StartReconciliation;
}

}  // namespace

class PlanningSceneLeaseProtocol::Impl
{
public:
  enum class JournalKind : std::uint8_t
  {
    Acquire,
    Release,
  };

  struct JournalEntry
  {
    JournalKind kind{JournalKind::Acquire};
    AcquirePlanningSceneLeaseRequest acquire_request;
    ReleasePlanningSceneLeaseRequest release_request;
    PlanningSceneLeaseReply reply;
  };

  struct ActiveLease
  {
    PlanningSceneLeaseSummary summary;
    std::string token;
    std::uint64_t fence_verification_epoch{0};
    bool requires_post_fence_verification{false};
  };

  Impl(PlanningSceneLeaseConfig input_config, TokenFactory input_token_factory)
  : config(std::move(input_config)), token_factory(std::move(input_token_factory))
  {
    if (config.operation_journal_capacity < 2) {
      throw std::invalid_argument(
              "operation_journal_capacity must reserve acquisition and release entries");
    }
    if (!token_factory) {
      token_factory = secure_token;
    }
    if (config.projector_epoch.empty()) {
      const auto generated_epoch = secure_token();
      if (!generated_epoch) {
        throw std::runtime_error("secure projector-epoch generation failed");
      }
      projector_epoch = *generated_epoch;
    } else {
      projector_epoch = config.projector_epoch;
    }
    scene_content_generation = config.initial_scene_content_generation;
  }

  [[nodiscard]] PlanningSceneLeaseReply acquire(
    const AcquirePlanningSceneLeaseRequest & request,
    ProjectorStage stage, std::int64_t now_ns)
  {
    if (!valid_operation_id(request.operation_id) || now_ns < 0) {
      return reply(
        PlanningSceneLeaseCode::InvalidArgument,
        "acquisition operation ID or timestamp is invalid");
    }

    if (const auto existing = journal.find(request.operation_id); existing != journal.end()) {
      if (existing->second.kind != JournalKind::Acquire ||
        existing->second.acquire_request != request)
      {
        return reply(
          PlanningSceneLeaseCode::IdempotencyConflict,
          "operation ID was already used with a different operation or payload");
      }
      return existing->second.reply;
    }

    if (active) {
      if (active->summary.phase == PlanningSceneLeasePhase::Draining &&
        active->summary.acquisition_operation_id == request.operation_id)
      {
        if (active->summary.minimum_applied_revision != request.minimum_applied_revision) {
          return reply(
            PlanningSceneLeaseCode::IdempotencyConflict,
            "pending acquisition operation ID was reused with a different revision");
        }
        return current_acquisition_reply();
      }
      return reply(
        PlanningSceneLeaseCode::Conflict,
        "another planning-scene lease transaction is active");
    }

    if (journal.size() > config.operation_journal_capacity - 2) {
      return reply(
        PlanningSceneLeaseCode::ResourceExhausted,
        "operation journal cannot reserve acquisition and release entries");
    }

    const auto token = token_factory();
    if (!token || token->size() < kMinimumTokenLength || token_already_used(*token)) {
      return reply(
        PlanningSceneLeaseCode::InternalError,
        "secure unique lease-token generation failed");
    }

    PlanningSceneLeaseSummary summary;
    if (next_lease_id == 0) {
      return reply(PlanningSceneLeaseCode::InternalError, "lease ID space is exhausted");
    }
    summary.lease_id = next_lease_id++;
    summary.acquisition_operation_id = request.operation_id;
    summary.minimum_applied_revision = request.minimum_applied_revision;
    summary.phase = PlanningSceneLeasePhase::Draining;
    active = ActiveLease{std::move(summary), *token, verification_epoch,
      proof_dirty || !has_verified_scene || is_side_effecting(stage)};

    if (is_side_effecting(stage)) {
      proof_dirty = true;
    }

    const bool cached_proof_eligible =
      !active->requires_post_fence_verification &&
      verified_applied_revision >= request.minimum_applied_revision;
    const AcquisitionAction action = is_read_only(stage) ?
      AcquisitionAction::InvalidateReadOnlyGeneration :
      AcquisitionAction::None;
    if (cached_proof_eligible) {
      grant(now_ns);
      auto result = current_acquisition_reply();
      result.action = action;
      return result;
    }

    auto result = current_acquisition_reply();
    result.action = action_for(stage);
    return result;
  }

  [[nodiscard]] PlanningSceneLeaseReply validate(const std::string & token) const
  {
    if (!active || active->summary.phase != PlanningSceneLeasePhase::Held || token.empty() ||
      !constant_time_equal(active->token, token))
    {
      return reply(
        PlanningSceneLeaseCode::TokenMismatch,
        "planning-scene lease token is not active");
    }
    return reply(PlanningSceneLeaseCode::Valid, "planning-scene lease is held", active->summary);
  }

  [[nodiscard]] PlanningSceneLeaseReply release(const ReleasePlanningSceneLeaseRequest & request)
  {
    if (!valid_operation_id(request.operation_id) || request.token.empty()) {
      return reply(
        PlanningSceneLeaseCode::InvalidArgument,
        "release operation ID or token is invalid");
    }

    if (const auto existing = journal.find(request.operation_id); existing != journal.end()) {
      if (existing->second.kind != JournalKind::Release ||
        existing->second.release_request != request)
      {
        return reply(
          PlanningSceneLeaseCode::IdempotencyConflict,
          "operation ID was already used with a different operation or payload");
      }
      return existing->second.reply;
    }

    if (!active || active->summary.phase != PlanningSceneLeasePhase::Held ||
      !constant_time_equal(active->token, request.token))
    {
      return reply(
        PlanningSceneLeaseCode::TokenMismatch,
        "planning-scene lease token is not active");
    }
    if (journal.size() >= config.operation_journal_capacity) {
      return reply(PlanningSceneLeaseCode::ResourceExhausted, "operation journal is full");
    }

    active->summary.phase = PlanningSceneLeasePhase::Releasing;
    active_release_operation_id = request.operation_id;
    required_release_revision = request.required_semantic_revision;
    active->token.clear();
    const auto result =
      reply(
      PlanningSceneLeaseCode::ReleaseAccepted,
      "planning-scene lease release accepted; reconciliation required", active->summary);
    journal.emplace(request.operation_id, JournalEntry{JournalKind::Release, {}, request, result});
    return result;
  }

  [[nodiscard]] VerificationTransition record_verification(
    std::uint64_t applied_revision,
    std::int64_t verified_at_ns)
  {
    if (verified_at_ns < 0) {
      throw std::invalid_argument("verified_at_ns cannot be negative");
    }
    if (active && active->summary.phase == PlanningSceneLeasePhase::Held) {
      return VerificationTransition::RejectedWhileHeld;
    }
    const bool release_will_complete =
      active && active->summary.phase == PlanningSceneLeasePhase::Releasing &&
      applied_revision >= required_release_revision;
    const bool establish_content_proof = !has_verified_scene;
    const bool advance_content_generation =
      establish_content_proof || scene_diff_awaiting_verification || release_will_complete;
    if (verification_epoch == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("planning-scene verification epoch is exhausted");
    }
    if (advance_content_generation &&
      scene_content_generation == std::numeric_limits<std::uint64_t>::max())
    {
      throw std::overflow_error("planning-scene content generation is exhausted");
    }

    ++verification_epoch;
    if (advance_content_generation) {
      ++scene_content_generation;
      scene_diff_awaiting_verification = false;
    }
    verified_applied_revision = applied_revision;
    has_verified_scene = true;
    proof_dirty = false;

    if (!active) {
      return VerificationTransition::None;
    }
    if (active->summary.phase == PlanningSceneLeasePhase::Draining &&
      applied_revision >= active->summary.minimum_applied_revision &&
      (!active->requires_post_fence_verification ||
      verification_epoch > active->fence_verification_epoch))
    {
      grant(verified_at_ns);
      return VerificationTransition::LeaseGranted;
    }
    if (active->summary.phase == PlanningSceneLeasePhase::Releasing &&
      applied_revision >= required_release_revision)
    {
      active.reset();
      active_release_operation_id.reset();
      required_release_revision = 0;
      return VerificationTransition::ReleaseCompleted;
    }
    return VerificationTransition::None;
  }

  void record_scene_diff_submission()
  {
    if (scene_content_generation == std::numeric_limits<std::uint64_t>::max()) {
      throw std::overflow_error("planning-scene content generation is exhausted");
    }
    scene_diff_awaiting_verification = true;
    proof_dirty = true;
  }

  void record_side_effect_unknown()
  {
    proof_dirty = true;
    if (active && active->summary.phase == PlanningSceneLeasePhase::Draining) {
      active->requires_post_fence_verification = true;
    }
  }

  [[nodiscard]] PlanningSceneLeaseProtocolState state() const
  {
    PlanningSceneLeaseProtocolState result;
    result.projector_epoch = projector_epoch;
    result.phase = active ? active->summary.phase : PlanningSceneLeasePhase::None;
    if (active) {
      result.lease = active->summary;
    }
    result.verified_applied_revision = verified_applied_revision;
    result.verification_epoch = verification_epoch;
    result.scene_content_generation = scene_content_generation;
    result.has_verified_scene = has_verified_scene;
    result.proof_dirty = proof_dirty;
    result.scene_diff_awaiting_verification = scene_diff_awaiting_verification;
    result.required_release_revision = required_release_revision;
    result.reconciliation_allowed =
      !active || active->summary.phase != PlanningSceneLeasePhase::Held;
    return result;
  }

  [[nodiscard]] JournalRetentionSnapshot retention_snapshot() const
  {
    JournalRetentionSnapshot result;
    result.journal = "projector.lease";
    result.epoch_id = projector_epoch;
    result.capacity = config.operation_journal_capacity;
    result.size = journal.size();
    if (active) {
      result.active_lease_phase = active->summary.phase;
      // Acquisition and release entries are pre-reserved: two while draining, then one.
      switch (active->summary.phase) {
        case PlanningSceneLeasePhase::Draining:
          result.reserved_credits = 2;
          break;
        case PlanningSceneLeasePhase::Held:
          result.reserved_credits = 1;
          break;
        case PlanningSceneLeasePhase::None:
        case PlanningSceneLeasePhase::Releasing:
          break;
      }
      if (journal.contains(active->summary.acquisition_operation_id)) {
        ++result.open_obligations;
      }
      if (active_release_operation_id && journal.contains(*active_release_operation_id)) {
        ++result.open_obligations;
      }
    }
    result.terminal_receipts = result.size - result.open_obligations;
    result.inhibited = proof_dirty || !has_verified_scene;
    result.evicting = false;
    return result;
  }

private:
  [[nodiscard]] static PlanningSceneLeaseReply reply(
    PlanningSceneLeaseCode code, std::string detail,
    std::optional<PlanningSceneLeaseSummary> lease = std::nullopt, std::string token = {})
  {
    return PlanningSceneLeaseReply{code, std::move(detail), std::move(lease), std::move(token),
      AcquisitionAction::None};
  }

  [[nodiscard]] PlanningSceneLeaseReply current_acquisition_reply() const
  {
    if (!active) {
      return reply(PlanningSceneLeaseCode::InternalError, "active lease state is unavailable");
    }
    if (active->summary.phase == PlanningSceneLeasePhase::Draining) {
      return reply(
        PlanningSceneLeaseCode::Draining, "planning-scene lease acquisition is draining",
        active->summary);
    }
    if (active->summary.phase == PlanningSceneLeasePhase::Held) {
      return reply(
        PlanningSceneLeaseCode::Granted, "planning-scene lease granted", active->summary,
        active->token);
    }
    return reply(
      PlanningSceneLeaseCode::Conflict,
      "planning-scene lease acquisition is no longer active", active->summary);
  }

  void grant(std::int64_t acquired_at_ns)
  {
    active->summary.granted_applied_revision = verified_applied_revision;
    active->summary.verification_epoch = verification_epoch;
    active->summary.acquired_at_ns = acquired_at_ns;
    active->summary.phase = PlanningSceneLeasePhase::Held;
    const AcquirePlanningSceneLeaseRequest request{active->summary.acquisition_operation_id,
      active->summary.minimum_applied_revision};
    const auto granted = current_acquisition_reply();
    journal.emplace(request.operation_id, JournalEntry{JournalKind::Acquire, request, {}, granted});
  }

  [[nodiscard]] bool token_already_used(const std::string & candidate) const
  {
    for (const auto &[operation_id, entry] : journal) {
      static_cast<void>(operation_id);
      if (entry.kind == JournalKind::Acquire && constant_time_equal(entry.reply.token, candidate)) {
        return true;
      }
    }
    return false;
  }

  PlanningSceneLeaseConfig config;
  TokenFactory token_factory;
  std::map<std::string, JournalEntry> journal;
  std::optional<ActiveLease> active;
  std::optional<std::string> active_release_operation_id;
  std::uint64_t next_lease_id{1};
  std::uint64_t verified_applied_revision{0};
  std::uint64_t verification_epoch{0};
  std::uint64_t required_release_revision{0};
  std::uint64_t scene_content_generation{0};
  bool has_verified_scene{false};
  bool proof_dirty{false};
  bool scene_diff_awaiting_verification{false};
  std::string projector_epoch;
};

PlanningSceneLeaseProtocol::PlanningSceneLeaseProtocol(
  PlanningSceneLeaseConfig config,
  TokenFactory token_factory)
: impl_(std::make_unique<Impl>(std::move(config), std::move(token_factory)))
{
}

PlanningSceneLeaseProtocol::~PlanningSceneLeaseProtocol() = default;
PlanningSceneLeaseProtocol::PlanningSceneLeaseProtocol(PlanningSceneLeaseProtocol &&) noexcept =
  default;
PlanningSceneLeaseProtocol & PlanningSceneLeaseProtocol::operator=(
  PlanningSceneLeaseProtocol &&) noexcept = default;

PlanningSceneLeaseReply PlanningSceneLeaseProtocol::acquire(
  const AcquirePlanningSceneLeaseRequest & request, ProjectorStage stage, std::int64_t now_ns)
{
  return impl_->acquire(request, stage, now_ns);
}

PlanningSceneLeaseReply PlanningSceneLeaseProtocol::validate(const std::string & token) const
{
  return impl_->validate(token);
}

PlanningSceneLeaseReply PlanningSceneLeaseProtocol::release(
  const ReleasePlanningSceneLeaseRequest & request)
{
  return impl_->release(request);
}

VerificationTransition PlanningSceneLeaseProtocol::record_verification(
  std::uint64_t applied_revision, std::int64_t verified_at_ns)
{
  return impl_->record_verification(applied_revision, verified_at_ns);
}

void PlanningSceneLeaseProtocol::record_side_effect_unknown()
{
  impl_->record_side_effect_unknown();
}

void PlanningSceneLeaseProtocol::record_scene_diff_submission()
{
  impl_->record_scene_diff_submission();
}

PlanningSceneLeaseProtocolState PlanningSceneLeaseProtocol::state() const {return impl_->state();}

JournalRetentionSnapshot PlanningSceneLeaseProtocol::retention_snapshot() const
{
  return impl_->retention_snapshot();
}

std::string to_string(PlanningSceneLeasePhase phase)
{
  switch (phase) {
    case PlanningSceneLeasePhase::None:
      return "none";
    case PlanningSceneLeasePhase::Draining:
      return "draining";
    case PlanningSceneLeasePhase::Held:
      return "held";
    case PlanningSceneLeasePhase::Releasing:
      return "releasing";
  }
  return "unknown";
}

std::string to_string(PlanningSceneLeaseCode code)
{
  switch (code) {
    case PlanningSceneLeaseCode::Unset:
      return "unset";
    case PlanningSceneLeaseCode::Draining:
      return "draining";
    case PlanningSceneLeaseCode::Granted:
      return "granted";
    case PlanningSceneLeaseCode::Valid:
      return "valid";
    case PlanningSceneLeaseCode::ReleaseAccepted:
      return "release_accepted";
    case PlanningSceneLeaseCode::InvalidArgument:
      return "invalid_argument";
    case PlanningSceneLeaseCode::Conflict:
      return "conflict";
    case PlanningSceneLeaseCode::TokenMismatch:
      return "token_mismatch";
    case PlanningSceneLeaseCode::IdempotencyConflict:
      return "idempotency_conflict";
    case PlanningSceneLeaseCode::ResourceExhausted:
      return "resource_exhausted";
    case PlanningSceneLeaseCode::InternalError:
      return "internal_error";
  }
  return "unknown";
}

}  // namespace restocker_task_executor
