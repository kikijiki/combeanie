// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "restocker_task_executor/planning_scene_lease.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr char kTokenA[] = "11111111111111111111111111111111";
constexpr char kTokenB[] = "22222222222222222222222222222222";
constexpr char kTokenC[] = "33333333333333333333333333333333";
constexpr char kProjectorEpoch[] = "projector-epoch-for-tests";

PlanningSceneLeaseProtocol protocol_with_tokens(
  std::size_t capacity = 4096,
  std::string first = kTokenA,
  std::string second = kTokenB,
  std::string third = kTokenC)
{
  auto tokens =
    [values = std::vector<std::string>{std::move(first), std::move(second), std::move(third)},
      index = std::size_t{0}]() mutable -> std::optional<std::string> {
      if (index >= values.size()) {
        return std::nullopt;
      }
      return values[index++];
    };
  PlanningSceneLeaseConfig config;
  config.operation_journal_capacity = capacity;
  config.projector_epoch = kProjectorEpoch;
  return PlanningSceneLeaseProtocol{std::move(config), std::move(tokens)};
}

void seed_verification(
  PlanningSceneLeaseProtocol & protocol, std::uint64_t revision = 10,
  std::int64_t time_ns = 100)
{
  EXPECT_EQ(protocol.record_verification(revision, time_ns), VerificationTransition::None);
}

PlanningSceneLeaseReply acquire(
  PlanningSceneLeaseProtocol & protocol,
  std::string operation_id = "acquire-1",
  std::uint64_t minimum_revision = 10,
  ProjectorStage stage = ProjectorStage::Idle,
  std::int64_t now_ns = 200)
{
  return protocol.acquire(
    AcquirePlanningSceneLeaseRequest{std::move(operation_id), minimum_revision}, stage, now_ns);
}

PlanningSceneLeaseReply release(
  PlanningSceneLeaseProtocol & protocol,
  std::string operation_id = "release-1", std::string token = kTokenA,
  std::uint64_t required_revision = 11)
{
  return protocol.release(
    ReleasePlanningSceneLeaseRequest{std::move(operation_id),
      std::move(token), required_revision});
}

TEST(PlanningSceneLeaseProtocol, GrantsImmediatelyFromEligibleIdleProof)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 12);

  const auto result = acquire(protocol, "acquire-immediate", 10);
  EXPECT_EQ(result.code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(result.action, AcquisitionAction::None);
  EXPECT_EQ(result.token, kTokenA);
  ASSERT_TRUE(result.lease);
  EXPECT_EQ(result.lease->phase, PlanningSceneLeasePhase::Held);
  EXPECT_EQ(result.lease->granted_applied_revision, 12U);
  EXPECT_EQ(result.lease->verification_epoch, 1U);
  EXPECT_EQ(result.lease->acquired_at_ns, 200);
  EXPECT_FALSE(result.lease->acquisition_operation_id.empty());
  EXPECT_FALSE(protocol.state().reconciliation_allowed);
  EXPECT_EQ(protocol.state().projector_epoch, kProjectorEpoch);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);
}

TEST(PlanningSceneLeaseProtocol, EstablishesContentProofOnceAndIgnoresPeriodicNoOps)
{
  auto protocol = protocol_with_tokens();
  EXPECT_EQ(protocol.state().scene_content_generation, 0U);

  seed_verification(protocol, 10, 100);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);
  EXPECT_EQ(protocol.record_verification(11, 101), VerificationTransition::None);
  EXPECT_EQ(protocol.state().verification_epoch, 2U);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);
}

TEST(PlanningSceneLeaseProtocol, ChangedDiffEvidenceSurvivesUnknownOutcomeUntilReadback)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol);

  protocol.record_scene_diff_submission();
  protocol.record_side_effect_unknown();
  protocol.record_side_effect_unknown();
  EXPECT_TRUE(protocol.state().scene_diff_awaiting_verification);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);

  EXPECT_EQ(protocol.record_verification(11, 200), VerificationTransition::None);
  EXPECT_FALSE(protocol.state().scene_diff_awaiting_verification);
  EXPECT_EQ(protocol.state().scene_content_generation, 2U);
  EXPECT_EQ(protocol.record_verification(12, 201), VerificationTransition::None);
  EXPECT_EQ(protocol.state().scene_content_generation, 2U);
}

TEST(PlanningSceneLeaseProtocol, ReconcilesUntilMinimumRevisionIsVerified)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 9);

  const auto pending = acquire(protocol, "acquire-newer", 10);
  EXPECT_EQ(pending.code, PlanningSceneLeaseCode::Draining);
  EXPECT_EQ(pending.action, AcquisitionAction::StartReconciliation);
  EXPECT_TRUE(pending.token.empty());
  EXPECT_EQ(protocol.state().phase, PlanningSceneLeasePhase::Draining);

  EXPECT_EQ(protocol.record_verification(9, 201), VerificationTransition::None);
  EXPECT_EQ(protocol.state().phase, PlanningSceneLeasePhase::Draining);
  EXPECT_EQ(protocol.record_verification(10, 202), VerificationTransition::LeaseGranted);
  EXPECT_EQ(protocol.state().phase, PlanningSceneLeasePhase::Held);

  const auto poll = acquire(protocol, "acquire-newer", 10, ProjectorStage::Idle, 999);
  EXPECT_EQ(poll.code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(poll.token, kTokenA);
  EXPECT_EQ(poll.lease->acquired_at_ns, 202);
}

TEST(PlanningSceneLeaseProtocol, InvalidatesOnlyReadOnlyGenerations)
{
  for (const auto stage : {ProjectorStage::Snapshot, ProjectorStage::CurrentScene}) {
    auto protocol = protocol_with_tokens();
    seed_verification(protocol, 10);
    const auto result = acquire(protocol, "acquire-read", 10, stage);
    EXPECT_EQ(result.code, PlanningSceneLeaseCode::Granted);
    EXPECT_EQ(result.action, AcquisitionAction::InvalidateReadOnlyGeneration);
  }
}

TEST(PlanningSceneLeaseProtocol, DrainsApplyAndVerifyBeforeGrant)
{
  for (const auto stage : {ProjectorStage::ApplyScene, ProjectorStage::VerifyScene}) {
    auto protocol = protocol_with_tokens();
    seed_verification(protocol, 10);
    const auto pending = acquire(protocol, "acquire-write", 10, stage);
    EXPECT_EQ(pending.code, PlanningSceneLeaseCode::Draining);
    EXPECT_EQ(pending.action, AcquisitionAction::DrainSideEffectingGeneration);
    EXPECT_TRUE(protocol.state().proof_dirty);
    EXPECT_EQ(protocol.validate(kTokenA).code, PlanningSceneLeaseCode::TokenMismatch);

    EXPECT_EQ(protocol.record_verification(10, 300), VerificationTransition::LeaseGranted);
    EXPECT_EQ(protocol.validate(kTokenA).code, PlanningSceneLeaseCode::Valid);
  }
}

TEST(PlanningSceneLeaseProtocol, UnknownApplyOutcomeInvalidatesCachedProof)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  protocol.record_side_effect_unknown();
  EXPECT_TRUE(protocol.state().proof_dirty);

  const auto pending = acquire(protocol, "acquire-dirty", 10);
  EXPECT_EQ(pending.code, PlanningSceneLeaseCode::Draining);
  EXPECT_EQ(pending.action, AcquisitionAction::StartReconciliation);
  EXPECT_EQ(protocol.state().verification_epoch, 1U);

  EXPECT_EQ(protocol.record_verification(10, 301), VerificationTransition::LeaseGranted);
  EXPECT_EQ(protocol.state().verification_epoch, 2U);
  EXPECT_FALSE(protocol.state().proof_dirty);
}

TEST(PlanningSceneLeaseProtocol, UnknownOutcomeDuringDrainStillRequiresNewProof)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  EXPECT_EQ(
    acquire(protocol, "acquire-apply", 10, ProjectorStage::ApplyScene).code,
    PlanningSceneLeaseCode::Draining);
  protocol.record_side_effect_unknown();

  const auto poll = acquire(protocol, "acquire-apply", 10);
  EXPECT_EQ(poll.code, PlanningSceneLeaseCode::Draining);
  EXPECT_TRUE(poll.token.empty());
  EXPECT_EQ(protocol.record_verification(10, 400), VerificationTransition::LeaseGranted);
}

TEST(PlanningSceneLeaseProtocol, ReleaseInvalidatesCapabilityUntilThresholdIsVerified)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  ASSERT_EQ(acquire(protocol).code, PlanningSceneLeaseCode::Granted);

  const auto accepted = release(protocol, "release-one", kTokenA, 12);
  EXPECT_EQ(accepted.code, PlanningSceneLeaseCode::ReleaseAccepted);
  ASSERT_TRUE(accepted.lease);
  EXPECT_EQ(accepted.lease->phase, PlanningSceneLeasePhase::Releasing);
  EXPECT_EQ(protocol.validate(kTokenA).code, PlanningSceneLeaseCode::TokenMismatch);
  EXPECT_TRUE(protocol.state().reconciliation_allowed);
  EXPECT_EQ(protocol.state().required_release_revision, 12U);

  EXPECT_EQ(protocol.record_verification(11, 500), VerificationTransition::None);
  EXPECT_EQ(protocol.state().phase, PlanningSceneLeasePhase::Releasing);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);
  EXPECT_EQ(acquire(protocol, "acquire-too-soon", 11).code, PlanningSceneLeaseCode::Conflict);

  EXPECT_EQ(protocol.record_verification(12, 501), VerificationTransition::ReleaseCompleted);
  EXPECT_EQ(protocol.state().phase, PlanningSceneLeasePhase::None);
  EXPECT_EQ(protocol.state().required_release_revision, 0U);
  EXPECT_EQ(protocol.state().scene_content_generation, 2U);
}

TEST(PlanningSceneLeaseProtocol, ChangedDiffAndReleaseCompletionAdvanceContentOnce)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  ASSERT_EQ(acquire(protocol).code, PlanningSceneLeaseCode::Granted);
  ASSERT_EQ(
    release(protocol, "release-combined", kTokenA, 11).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  protocol.record_scene_diff_submission();

  EXPECT_EQ(protocol.record_verification(11, 501), VerificationTransition::ReleaseCompleted);
  EXPECT_EQ(protocol.state().scene_content_generation, 2U);
  EXPECT_FALSE(protocol.state().scene_diff_awaiting_verification);
}

TEST(PlanningSceneLeaseProtocol, AcquisitionAndReleaseRepliesAreExactlyReplayable)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  const auto first_acquire = acquire(protocol, "acquire-replay", 10);
  const auto acquire_replay =
    acquire(protocol, "acquire-replay", 10, ProjectorStage::ApplyScene, 999);
  EXPECT_EQ(acquire_replay.code, first_acquire.code);
  EXPECT_EQ(acquire_replay.token, first_acquire.token);
  EXPECT_EQ(acquire_replay.lease, first_acquire.lease);
  EXPECT_EQ(acquire_replay.action, AcquisitionAction::None);

  const auto first_release = release(protocol, "release-replay", kTokenA, 11);
  ASSERT_EQ(protocol.record_verification(11, 600), VerificationTransition::ReleaseCompleted);
  const auto release_replay = release(protocol, "release-replay", kTokenA, 11);
  EXPECT_EQ(release_replay.code, first_release.code);
  EXPECT_EQ(release_replay.lease, first_release.lease);

  const auto late_acquire_replay = acquire(protocol, "acquire-replay", 10);
  EXPECT_EQ(late_acquire_replay.code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(late_acquire_replay.token, kTokenA);
  EXPECT_EQ(protocol.validate(kTokenA).code, PlanningSceneLeaseCode::TokenMismatch);
}

TEST(PlanningSceneLeaseProtocol, ChangedPayloadAndCrossKindIdsConflict)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 10);
  ASSERT_EQ(acquire(protocol, "shared-id", 10).code, PlanningSceneLeaseCode::Granted);

  EXPECT_EQ(acquire(protocol, "shared-id", 11).code, PlanningSceneLeaseCode::IdempotencyConflict);
  EXPECT_EQ(
    release(protocol, "shared-id", kTokenA, 11).code,
    PlanningSceneLeaseCode::IdempotencyConflict);

  ASSERT_EQ(
    release(protocol, "release-id", kTokenA, 11).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  EXPECT_EQ(
    release(protocol, "release-id", kTokenA, 12).code,
    PlanningSceneLeaseCode::IdempotencyConflict);
  EXPECT_EQ(
    release(protocol, "release-id", kTokenB, 11).code,
    PlanningSceneLeaseCode::IdempotencyConflict);
}

TEST(PlanningSceneLeaseProtocol, PendingAcquisitionIsIdempotentAndExclusive)
{
  auto protocol = protocol_with_tokens();
  const auto pending = acquire(protocol, "pending", 4);
  EXPECT_EQ(pending.code, PlanningSceneLeaseCode::Draining);
  const auto replay = acquire(protocol, "pending", 4, ProjectorStage::ApplyScene);
  EXPECT_EQ(replay.code, PlanningSceneLeaseCode::Draining);
  EXPECT_EQ(replay.lease, pending.lease);
  EXPECT_EQ(acquire(protocol, "pending", 5).code, PlanningSceneLeaseCode::IdempotencyConflict);
  EXPECT_EQ(acquire(protocol, "other", 4).code, PlanningSceneLeaseCode::Conflict);
}

TEST(PlanningSceneLeaseProtocol, RejectsInvalidInputAndWrongCapabilities)
{
  auto protocol = protocol_with_tokens();
  EXPECT_EQ(acquire(protocol, "", 0).code, PlanningSceneLeaseCode::InvalidArgument);
  EXPECT_EQ(
    acquire(protocol, std::string(129, 'a'), 0).code,
    PlanningSceneLeaseCode::InvalidArgument);
  EXPECT_EQ(
    protocol.acquire({"negative-time", 0}, ProjectorStage::Idle, -1).code,
    PlanningSceneLeaseCode::InvalidArgument);
  EXPECT_EQ(protocol.validate("").code, PlanningSceneLeaseCode::TokenMismatch);
  EXPECT_EQ(release(protocol, "", kTokenA, 0).code, PlanningSceneLeaseCode::InvalidArgument);

  seed_verification(protocol, 1);
  ASSERT_EQ(acquire(protocol, "held", 1).code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(protocol.validate(kTokenB).code, PlanningSceneLeaseCode::TokenMismatch);
  EXPECT_EQ(
    release(protocol, "wrong-token", kTokenB, 2).code,
    PlanningSceneLeaseCode::TokenMismatch);
}

TEST(PlanningSceneLeaseProtocol, JournalReservesReleaseCapacityAndNeverEvicts)
{
  auto protocol = protocol_with_tokens(4);
  seed_verification(protocol, 1);
  ASSERT_EQ(acquire(protocol, "acquire-a", 1).code, PlanningSceneLeaseCode::Granted);
  ASSERT_EQ(
    release(protocol, "release-a", kTokenA, 2).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  ASSERT_EQ(protocol.record_verification(2, 700), VerificationTransition::ReleaseCompleted);

  ASSERT_EQ(acquire(protocol, "acquire-b", 2).code, PlanningSceneLeaseCode::Granted);
  ASSERT_EQ(
    release(protocol, "release-b", kTokenB, 3).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  ASSERT_EQ(protocol.record_verification(3, 701), VerificationTransition::ReleaseCompleted);

  EXPECT_EQ(acquire(protocol, "acquire-c", 3).code, PlanningSceneLeaseCode::ResourceExhausted);
  EXPECT_EQ(acquire(protocol, "acquire-a", 1).code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(
    release(protocol, "release-a", kTokenA, 2).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
}

TEST(PlanningSceneLeaseProtocol, TokenGenerationFailureAndReuseFailClosed)
{
  PlanningSceneLeaseProtocol failing{{},
    []() -> std::optional<std::string> {return std::nullopt;}};
  seed_verification(failing, 1);
  EXPECT_EQ(acquire(failing, "no-randomness", 1).code, PlanningSceneLeaseCode::InternalError);
  EXPECT_EQ(failing.state().phase, PlanningSceneLeasePhase::None);

  auto short_token = protocol_with_tokens(4096, "too-short");
  seed_verification(short_token, 1);
  EXPECT_EQ(acquire(short_token, "short-token", 1).code, PlanningSceneLeaseCode::InternalError);

  auto duplicate = protocol_with_tokens(4, kTokenA, kTokenA);
  seed_verification(duplicate, 1);
  ASSERT_EQ(acquire(duplicate, "first", 1).code, PlanningSceneLeaseCode::Granted);
  ASSERT_EQ(
    release(duplicate, "first-release", kTokenA, 2).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  ASSERT_EQ(duplicate.record_verification(2, 800), VerificationTransition::ReleaseCompleted);
  EXPECT_EQ(acquire(duplicate, "duplicate-token", 2).code, PlanningSceneLeaseCode::InternalError);
}

TEST(PlanningSceneLeaseProtocol, HeldStateRejectsUnexpectedProjectorVerification)
{
  auto protocol = protocol_with_tokens();
  seed_verification(protocol, 1);
  ASSERT_EQ(acquire(protocol, "held", 1).code, PlanningSceneLeaseCode::Granted);
  EXPECT_EQ(protocol.record_verification(99, 900), VerificationTransition::RejectedWhileHeld);
  EXPECT_EQ(protocol.state().verified_applied_revision, 1U);
  EXPECT_EQ(protocol.state().verification_epoch, 1U);
  EXPECT_EQ(protocol.state().scene_content_generation, 1U);
}

TEST(PlanningSceneLeaseProtocol, ContentGenerationOverflowFailsBeforeProofMutation)
{
  PlanningSceneLeaseConfig config;
  config.projector_epoch = kProjectorEpoch;
  config.initial_scene_content_generation = std::numeric_limits<std::uint64_t>::max();
  PlanningSceneLeaseProtocol protocol{std::move(config)};

  EXPECT_THROW(protocol.record_scene_diff_submission(), std::overflow_error);
  EXPECT_THROW(static_cast<void>(protocol.record_verification(1, 100)), std::overflow_error);
  EXPECT_EQ(protocol.state().verification_epoch, 0U);
  EXPECT_FALSE(protocol.state().has_verified_scene);
  EXPECT_FALSE(protocol.state().scene_diff_awaiting_verification);
}

TEST(PlanningSceneLeaseProtocol, ConfigurationAndStringConversionsAreClosed)
{
  PlanningSceneLeaseConfig invalid_config;
  invalid_config.operation_journal_capacity = 1;
  EXPECT_THROW(PlanningSceneLeaseProtocol(std::move(invalid_config)), std::invalid_argument);
  EXPECT_EQ(to_string(PlanningSceneLeasePhase::None), "none");
  EXPECT_EQ(to_string(PlanningSceneLeasePhase::Draining), "draining");
  EXPECT_EQ(to_string(PlanningSceneLeasePhase::Held), "held");
  EXPECT_EQ(to_string(PlanningSceneLeasePhase::Releasing), "releasing");
  EXPECT_EQ(to_string(PlanningSceneLeaseCode::Unset), "unset");
  EXPECT_EQ(to_string(PlanningSceneLeaseCode::Granted), "granted");
  EXPECT_EQ(to_string(PlanningSceneLeaseCode::InternalError), "internal_error");
}

// CMB-SPEC-10 Stage 1: retention bounds are named and observable.
TEST(PlanningSceneLeaseRetention, ReportsProjectorEpochAndClassifiesLeaseLifecycle)
{
  auto protocol = protocol_with_tokens(8);
  auto snapshot = protocol.retention_snapshot();
  EXPECT_EQ(snapshot.journal, "projector.lease");
  EXPECT_EQ(snapshot.epoch_id, kProjectorEpoch);
  EXPECT_EQ(snapshot.capacity, 8U);
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);
  EXPECT_TRUE(snapshot.inhibited) << "a fresh idle lease is inhibited: no verified scene yet";
  EXPECT_EQ(snapshot.active_lease_phase, PlanningSceneLeasePhase::None);
  EXPECT_FALSE(snapshot.evicting);

  seed_verification(protocol, 10);
  EXPECT_FALSE(protocol.retention_snapshot().inhibited);

  const auto pending = acquire(protocol, "acquire-write", 10, ProjectorStage::ApplyScene);
  ASSERT_EQ(pending.code, PlanningSceneLeaseCode::Draining);
  snapshot = protocol.retention_snapshot();
  EXPECT_EQ(snapshot.size, 0U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 2U) << "acquire and release entries are pre-reserved";
  // open_obligations counts records only; the draining lease is visible through its phase.
  EXPECT_EQ(snapshot.active_lease_phase, PlanningSceneLeasePhase::Draining);
  EXPECT_TRUE(snapshot.inhibited);

  ASSERT_EQ(protocol.record_verification(10, 300), VerificationTransition::LeaseGranted);
  snapshot = protocol.retention_snapshot();
  EXPECT_EQ(snapshot.size, 1U);
  EXPECT_EQ(snapshot.open_obligations, 1U);
  EXPECT_EQ(snapshot.terminal_receipts, 0U);
  EXPECT_EQ(snapshot.reserved_credits, 1U);
  EXPECT_EQ(snapshot.active_lease_phase, PlanningSceneLeasePhase::Held);

  ASSERT_EQ(
    release(protocol, "release-1", kTokenA, 11).code, PlanningSceneLeaseCode::ReleaseAccepted);
  snapshot = protocol.retention_snapshot();
  EXPECT_EQ(snapshot.size, 2U);
  EXPECT_EQ(snapshot.open_obligations, 2U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);
  EXPECT_EQ(snapshot.active_lease_phase, PlanningSceneLeasePhase::Releasing);

  ASSERT_EQ(protocol.record_verification(11, 400), VerificationTransition::ReleaseCompleted);
  snapshot = protocol.retention_snapshot();
  EXPECT_EQ(snapshot.size, 2U);
  EXPECT_EQ(snapshot.open_obligations, 0U);
  EXPECT_EQ(snapshot.terminal_receipts, 2U);
  EXPECT_EQ(snapshot.reserved_credits, 0U);
  EXPECT_EQ(snapshot.open_obligations + snapshot.terminal_receipts, snapshot.size);
  EXPECT_EQ(snapshot.active_lease_phase, PlanningSceneLeasePhase::None);
  EXPECT_EQ(snapshot.epoch_id, kProjectorEpoch);
}

TEST(PlanningSceneLeaseRetention, FullJournalRetainsEveryReceiptAndRefusesWithoutEviction)
{
  auto protocol = protocol_with_tokens(2);
  seed_verification(protocol, 10);
  ASSERT_EQ(acquire(protocol).code, PlanningSceneLeaseCode::Granted);
  ASSERT_EQ(
    release(protocol, "release-1", kTokenA, 11).code,
    PlanningSceneLeaseCode::ReleaseAccepted);
  ASSERT_EQ(protocol.record_verification(11, 400), VerificationTransition::ReleaseCompleted);
  const auto before = protocol.retention_snapshot();
  EXPECT_EQ(before.size, before.capacity);

  EXPECT_EQ(
    acquire(protocol, "acquire-2", 11).code, PlanningSceneLeaseCode::ResourceExhausted);
  const auto after = protocol.retention_snapshot();
  EXPECT_EQ(after.size, before.size);
  EXPECT_EQ(after.terminal_receipts, before.terminal_receipts);
  EXPECT_EQ(after.reserved_credits, 0U);
  EXPECT_EQ(acquire(protocol, "acquire-1", 10).code, PlanningSceneLeaseCode::Granted)
    << "terminal replay still answers";
}

TEST(PlanningSceneLeaseRetention, GeneratedProjectorEpochsDifferPerConstruction)
{
  PlanningSceneLeaseProtocol first;
  PlanningSceneLeaseProtocol second;
  EXPECT_FALSE(first.retention_snapshot().epoch_id.empty());
  EXPECT_NE(first.retention_snapshot().epoch_id, second.retention_snapshot().epoch_id);
}

}  // namespace
}  // namespace restocker_task_executor
