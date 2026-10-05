// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_active_fault_epoch.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr GoalGeneration kGeneration = 7U;

CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId id{};
  id.front() = seed;
  id.back() = static_cast<std::uint8_t>(seed + 1U);
  return id;
}

CoordinatorDriverOutput terminal(
  CoordinatorDriverOutputKind kind, RestockActionOutcome outcome,
  GoalGeneration generation = kGeneration, std::string detail = "terminal")
{
  CoordinatorDriverOutput output;
  output.kind = kind;
  output.goal_generation = generation;
  output.outcome = outcome;
  output.detail = std::move(detail);
  return output;
}

void expect_same_output(
  const CoordinatorDriverOutput & actual, const CoordinatorDriverOutput & expected)
{
  EXPECT_EQ(actual.kind, expected.kind);
  EXPECT_EQ(actual.goal_generation, expected.goal_generation);
  EXPECT_EQ(actual.transition.accepted, expected.transition.accepted);
  EXPECT_EQ(actual.transition.state, expected.transition.state);
  EXPECT_EQ(actual.transition.command, expected.transition.command);
  EXPECT_EQ(actual.transition.fault, expected.transition.fault);
  EXPECT_EQ(actual.transition.attempt, expected.transition.attempt);
  EXPECT_EQ(actual.transition.recovery_attempt, expected.transition.recovery_attempt);
  EXPECT_EQ(actual.transition.timeout, expected.transition.timeout);
  EXPECT_EQ(actual.transition.total_timeout, expected.transition.total_timeout);
  EXPECT_EQ(actual.transition.cancel_requested, expected.transition.cancel_requested);
  EXPECT_EQ(actual.transition.safe_abort_requested, expected.transition.safe_abort_requested);
  EXPECT_EQ(
    actual.transition.first_trajectory_may_have_started,
    expected.transition.first_trajectory_may_have_started);
  EXPECT_EQ(
    actual.transition.current_execution_may_have_started,
    expected.transition.current_execution_may_have_started);
  EXPECT_EQ(
    actual.transition.current_execution_operation_generation,
    expected.transition.current_execution_operation_generation);
  EXPECT_EQ(actual.transition.object_held, expected.transition.object_held);
  EXPECT_EQ(actual.transition.reservation_active, expected.transition.reservation_active);
  EXPECT_EQ(actual.transition.detail, expected.transition.detail);
  EXPECT_EQ(actual.selected_pair.has_value(), expected.selected_pair.has_value());
  if (actual.selected_pair && expected.selected_pair) {
    EXPECT_EQ(actual.selected_pair->object_id, expected.selected_pair->object_id);
    EXPECT_EQ(actual.selected_pair->lane_id, expected.selected_pair->lane_id);
    EXPECT_EQ(actual.selected_pair->snapshot_revision, expected.selected_pair->snapshot_revision);
    EXPECT_EQ(actual.selected_pair->object_revision, expected.selected_pair->object_revision);
    EXPECT_EQ(actual.selected_pair->lane_revision, expected.selected_pair->lane_revision);
    EXPECT_DOUBLE_EQ(
      actual.selected_pair->product_envelope.radius_m,
      expected.selected_pair->product_envelope.radius_m);
    EXPECT_DOUBLE_EQ(
      actual.selected_pair->product_envelope.height_m,
      expected.selected_pair->product_envelope.height_m);
  }
  EXPECT_EQ(actual.outcome, expected.outcome);
  EXPECT_EQ(actual.metrics.initial_world_revision, expected.metrics.initial_world_revision);
  EXPECT_EQ(actual.metrics.final_world_revision, expected.metrics.final_world_revision);
  EXPECT_EQ(actual.metrics.attempt_count, expected.metrics.attempt_count);
  EXPECT_EQ(actual.metrics.retry_count, expected.metrics.retry_count);
  EXPECT_EQ(actual.metrics.elapsed, expected.metrics.elapsed);
  EXPECT_DOUBLE_EQ(actual.metrics.minimum_clearance_m, expected.metrics.minimum_clearance_m);
  EXPECT_EQ(actual.detail, expected.detail);
}

void expect_same_snapshot(
  const CoordinatorActiveFaultEpochSnapshot & actual,
  const CoordinatorActiveFaultEpochSnapshot & expected)
{
  EXPECT_EQ(actual.goal_id, expected.goal_id);
  EXPECT_EQ(actual.goal_generation, expected.goal_generation);
  EXPECT_EQ(actual.route_phase, expected.route_phase);
  EXPECT_EQ(actual.terminal_phase, expected.terminal_phase);
  EXPECT_EQ(actual.last_feedback_attempt, expected.last_feedback_attempt);
  EXPECT_EQ(actual.route_claim_outstanding, expected.route_claim_outstanding);
  EXPECT_EQ(actual.feedback_outstanding, expected.feedback_outstanding);
  EXPECT_EQ(actual.retained_terminal, expected.retained_terminal);
  EXPECT_EQ(actual.contradiction_observed, expected.contradiction_observed);
  EXPECT_EQ(actual.middleware_publication_returned, expected.middleware_publication_returned);
  EXPECT_EQ(actual.retirement_eligible, expected.retirement_eligible);
}

void enrich_evidence(CoordinatorDriverOutput & output)
{
  output.transition.accepted = true;
  output.transition.state = RestockTaskState::kRecover;
  output.transition.command = RestockTaskCommand::kReleaseTaskReservation;
  output.transition.fault = RestockTaskFault::kOperationFailed;
  output.transition.attempt = 17U;
  output.transition.recovery_attempt = 3U;
  output.transition.timeout = std::chrono::milliseconds(1234);
  output.transition.total_timeout = std::chrono::milliseconds(5678);
  output.transition.cancel_requested = true;
  output.transition.safe_abort_requested = true;
  output.transition.first_trajectory_may_have_started = true;
  output.transition.current_execution_may_have_started = true;
  output.transition.current_execution_operation_generation = 91U;
  output.transition.object_held = true;
  output.transition.reservation_active = true;
  output.transition.detail = "first transition evidence";
  output.selected_pair = SelectedTaskPair{
    restocker_world_state::ObjectId{37U}, restocker_world_state::LaneId{"lane_audit"},
    101U, 97U, 89U, CylinderEnvelope{0.033, 0.155}};
  output.metrics.initial_world_revision = 11U;
  output.metrics.final_world_revision = 19U;
  output.metrics.attempt_count = 5U;
  output.metrics.retry_count = 2U;
  output.metrics.elapsed = std::chrono::milliseconds(4242);
  output.metrics.minimum_clearance_m = 0.0125;
}

std::optional<CoordinatorActiveFaultClaim> claim(
  CoordinatorActiveFaultEpoch & epoch,
  ActiveTerminationRouteKind kind = ActiveTerminationRouteKind::kProtocolFailure)
{
  auto decision = epoch.claim_fault(kind);
  EXPECT_EQ(decision.status(), ActiveFaultClaimStatus::kClaimed);
  return decision.take_claim();
}

std::optional<CoordinatorTerminalPublicationPermit> offer_publishable(
  CoordinatorActiveFaultEpoch & epoch, CoordinatorDriverOutput & output,
  ActiveTerminalDisposition expected = ActiveTerminalDisposition::kPublicationPrepared)
{
  auto decision = epoch.offer_terminal(output);
  EXPECT_EQ(decision.disposition(), expected);
  EXPECT_TRUE(decision.output_was_consumed());
  return decision.take_publication_permit();
}

CoordinatorTerminalAckCompletionDecision complete_accepted_terminal(
  CoordinatorActiveFaultEpoch & epoch, std::string detail = "accepted terminal")
{
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded,
    kGeneration, std::move(detail));
  auto publication = offer_publishable(epoch, output).value();
  auto returned = epoch.terminal_publication_returned(publication);
  EXPECT_EQ(returned.status(), ActivePublicationReturnStatus::kAcknowledgementPrepared);
  auto acknowledgement = returned.take_ack_permit().value();
  auto completion = epoch.complete_terminal_ack(acknowledgement, true);
  EXPECT_EQ(
    completion.status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  return completion;
}

void complete_route(CoordinatorActiveFaultEpoch & epoch, bool secured)
{
  auto route_claim = claim(epoch);
  ASSERT_TRUE(route_claim);
  ASSERT_EQ(
    epoch.complete_fault(*route_claim, secured),
    secured ? ActiveFaultCompletionStatus::kSecured : ActiveFaultCompletionStatus::kUnsecured);
}

template<typename Capability>
void exercise_self_move(Capability & capability)
{
  auto * alias = &capability;
  capability = std::move(*alias);
}

TEST(CoordinatorActiveFaultEpochConstructionTest, RejectsInvalidIdentity)
{
  EXPECT_THROW(
    (void)CoordinatorActiveFaultEpoch(CoordinatorGoalId{}, kGeneration),
    std::invalid_argument);
  EXPECT_THROW((void)CoordinatorActiveFaultEpoch(goal_id(), 0U), std::invalid_argument);
  EXPECT_THROW(
    (void)CoordinatorActiveFaultEpoch(goal_id(), kReservedCoordinatorGoalGeneration),
    std::invalid_argument);
}

TEST(CoordinatorActiveFaultEpochClaimTest, LaterGenerationDoesNotConsumeOwnersClaim)
{
  CoordinatorActiveFaultEpoch first(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch second(goal_id(), kGeneration + 1U);
  auto first_claim = claim(first, ActiveTerminationRouteKind::kTransformAuthorityLoss);
  ASSERT_TRUE(first_claim);

  EXPECT_EQ(
    second.complete_fault(*first_claim, true), ActiveFaultCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(first_claim->live());
  EXPECT_EQ(first.complete_fault(*first_claim, true), ActiveFaultCompletionStatus::kSecured);
  EXPECT_FALSE(first_claim->live());
  EXPECT_EQ(first.snapshot().route_phase, ActiveFaultRoutePhase::kSecured);
}

TEST(CoordinatorActiveFaultEpochClaimTest, RejectsInvalidKindWithoutChangingFence)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto decision = epoch.claim_fault(static_cast<ActiveTerminationRouteKind>(255U));
  EXPECT_EQ(decision.status(), ActiveFaultClaimStatus::kInvalidRouteKind);
  EXPECT_FALSE(decision.take_claim());
  EXPECT_EQ(epoch.snapshot().route_phase, ActiveFaultRoutePhase::kClear);
}

TEST(CoordinatorActiveFaultEpochClaimTest, RepeatedClaimsReportTheClosedRouteState)
{
  CoordinatorActiveFaultEpoch pending(goal_id(), kGeneration);
  auto pending_claim = claim(pending, ActiveTerminationRouteKind::kCancel);
  ASSERT_TRUE(pending_claim);
  EXPECT_EQ(
    pending.claim_fault(ActiveTerminationRouteKind::kDrain).status(),
    ActiveFaultClaimStatus::kAlreadyRouting);
  ASSERT_EQ(
    pending.complete_fault(*pending_claim, true), ActiveFaultCompletionStatus::kSecured);
  EXPECT_EQ(
    pending.claim_fault(ActiveTerminationRouteKind::kDrain).status(),
    ActiveFaultClaimStatus::kAlreadySecured);

  CoordinatorActiveFaultEpoch unsecured(goal_id(), kGeneration);
  complete_route(unsecured, false);
  EXPECT_EQ(
    unsecured.claim_fault(ActiveTerminationRouteKind::kDrain).status(),
    ActiveFaultClaimStatus::kUnsecured);
}

TEST(CoordinatorActiveFaultEpochClaimTest, MoveTransfersClaimAndSelfMovePreservesIt)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto original = claim(epoch, ActiveTerminationRouteKind::kCancel);
  ASSERT_TRUE(original);
  CoordinatorActiveFaultClaim moved{std::move(*original)};
  EXPECT_FALSE(original->live());
  EXPECT_TRUE(moved.live());
  EXPECT_EQ(
    epoch.complete_fault(*original, true), ActiveFaultCompletionStatus::kNotLive);

  exercise_self_move(moved);
  EXPECT_TRUE(moved.live());
  EXPECT_EQ(epoch.complete_fault(moved, true), ActiveFaultCompletionStatus::kSecured);
}

TEST(CoordinatorActiveFaultEpochClaimTest, OverwriteAbandonsOldClaimWithoutAdvancingItsEpoch)
{
  CoordinatorActiveFaultEpoch abandoned_epoch(goal_id(1U), kGeneration);
  CoordinatorActiveFaultEpoch owner_epoch(goal_id(2U), kGeneration);
  auto abandoned = claim(abandoned_epoch, ActiveTerminationRouteKind::kCancel);
  auto owner = claim(owner_epoch, ActiveTerminationRouteKind::kDrain);
  ASSERT_TRUE(abandoned);
  ASSERT_TRUE(owner);

  *abandoned = std::move(*owner);
  EXPECT_FALSE(owner->live());
  EXPECT_TRUE(abandoned->live());
  EXPECT_EQ(
    abandoned_epoch.snapshot().route_phase, ActiveFaultRoutePhase::kRoutingPending);
  EXPECT_TRUE(abandoned_epoch.snapshot().route_claim_outstanding);
  EXPECT_EQ(
    owner_epoch.complete_fault(*abandoned, true), ActiveFaultCompletionStatus::kSecured);
}

TEST(CoordinatorActiveFaultEpochClaimTest, ClaimCanOutliveEpochAndCannotMutateReusedStorage)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::array<std::byte, sizeof(CoordinatorActiveFaultEpoch)> storage{};
  auto * original = new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration);
  auto old_claim = claim(*original, ActiveTerminationRouteKind::kDrain);
  ASSERT_TRUE(old_claim);
  original->~CoordinatorActiveFaultEpoch();

  auto * replacement =
    new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration + 1U);
  EXPECT_EQ(
    replacement->complete_fault(*old_claim, true), ActiveFaultCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(old_claim->live());
  EXPECT_EQ(replacement->snapshot().route_phase, ActiveFaultRoutePhase::kClear);
  replacement->~CoordinatorActiveFaultEpoch();
  old_claim.reset();
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, ValidKindsCloseEveryOpenRoutePhase)
{
  constexpr std::array kinds{
    ActiveFaultInvalidationKind::kStartupAuthorityFailure,
    ActiveFaultInvalidationKind::kAdapterInvariantFailure,
    ActiveFaultInvalidationKind::kShutdownTimeout};
  constexpr std::array phases{
    ActiveFaultRoutePhase::kClear,
    ActiveFaultRoutePhase::kRoutingPending,
    ActiveFaultRoutePhase::kSecured};

  for (const auto kind : kinds) {
    for (const auto phase : phases) {
      CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
      std::optional<CoordinatorActiveFaultClaim> route_claim;
      if (phase != ActiveFaultRoutePhase::kClear) {
        route_claim = claim(epoch, ActiveTerminationRouteKind::kTransformAuthorityLoss);
        ASSERT_TRUE(route_claim);
      }
      if (phase == ActiveFaultRoutePhase::kSecured) {
        ASSERT_EQ(
          epoch.complete_fault(*route_claim, true), ActiveFaultCompletionStatus::kSecured);
      }

      const auto before = epoch.snapshot();
      ASSERT_EQ(before.route_phase, phase);
      EXPECT_EQ(
        epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kInvalidatedUnsecured);
      const auto after = epoch.snapshot();
      EXPECT_EQ(after.route_phase, ActiveFaultRoutePhase::kUnsecured);
      EXPECT_FALSE(after.route_claim_outstanding);
      EXPECT_EQ(after.goal_id, before.goal_id);
      EXPECT_EQ(after.goal_generation, before.goal_generation);
      EXPECT_EQ(after.terminal_phase, before.terminal_phase);
      EXPECT_EQ(after.last_feedback_attempt, before.last_feedback_attempt);
      EXPECT_EQ(after.feedback_outstanding, before.feedback_outstanding);
      EXPECT_EQ(after.retained_terminal, before.retained_terminal);
      EXPECT_EQ(after.contradiction_observed, before.contradiction_observed);
      EXPECT_EQ(
        after.middleware_publication_returned, before.middleware_publication_returned);
      EXPECT_EQ(after.retirement_eligible, before.retirement_eligible);
      if (phase == ActiveFaultRoutePhase::kRoutingPending) {
        EXPECT_TRUE(route_claim->live());
      }
    }
  }
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, PendingClaimBecomesStaleButRemainsLive)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto route_claim = claim(epoch, ActiveTerminationRouteKind::kDrain);
  ASSERT_TRUE(route_claim);

  ASSERT_EQ(
    epoch.invalidate_fault(ActiveFaultInvalidationKind::kAdapterInvariantFailure),
    ActiveFaultInvalidationStatus::kInvalidatedUnsecured);
  EXPECT_TRUE(route_claim->live());
  EXPECT_EQ(
    epoch.complete_fault(*route_claim, true), ActiveFaultCompletionStatus::kStaleClaim);
  EXPECT_TRUE(route_claim->live());
  EXPECT_EQ(epoch.snapshot().route_phase, ActiveFaultRoutePhase::kUnsecured);
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, PreservesRetainedTerminalByteForByte)
{
  constexpr std::array kinds{
    ActiveFaultInvalidationKind::kStartupAuthorityFailure,
    ActiveFaultInvalidationKind::kAdapterInvariantFailure,
    ActiveFaultInvalidationKind::kShutdownTimeout};

  for (const auto kind : kinds) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto route_claim = claim(epoch, ActiveTerminationRouteKind::kCancel);
    ASSERT_TRUE(route_claim);
    auto output = terminal(
      CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded,
      kGeneration, "retained invalidation evidence");
    enrich_evidence(output);
    const auto expected = output;
    ASSERT_EQ(
      epoch.offer_terminal(output).disposition(),
      ActiveTerminalDisposition::kRetainedWhileRouting);

    ASSERT_EQ(
      epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kInvalidatedUnsecured);
    ASSERT_NE(epoch.retained_terminal_output(), nullptr);
    expect_same_output(*epoch.retained_terminal_output(), expected);
    const auto snapshot = epoch.snapshot();
    EXPECT_EQ(snapshot.route_phase, ActiveFaultRoutePhase::kUnsecured);
    EXPECT_EQ(snapshot.terminal_phase, ActiveTerminalPhase::kRetained);
    EXPECT_TRUE(snapshot.retained_terminal);
    EXPECT_TRUE(route_claim->live());
  }
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, PreservesOutstandingFeedbackByteForByte)
{
  constexpr std::array kinds{
    ActiveFaultInvalidationKind::kStartupAuthorityFailure,
    ActiveFaultInvalidationKind::kAdapterInvariantFailure,
    ActiveFaultInvalidationKind::kShutdownTimeout};

  for (const auto kind : kinds) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
    auto feedback = reserved.take_permit();
    ASSERT_TRUE(feedback);
    const auto before = epoch.snapshot();
    const auto expected_goal_id = feedback->goal_id();
    const auto expected_generation = feedback->goal_generation();

    ASSERT_EQ(
      epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kInvalidatedUnsecured);
    const auto after = epoch.snapshot();
    EXPECT_EQ(after.route_phase, ActiveFaultRoutePhase::kUnsecured);
    EXPECT_EQ(after.last_feedback_attempt, before.last_feedback_attempt);
    EXPECT_EQ(after.feedback_outstanding, before.feedback_outstanding);
    EXPECT_TRUE(feedback->live());
    EXPECT_EQ(feedback->goal_id(), expected_goal_id);
    EXPECT_EQ(feedback->goal_generation(), expected_generation);
    EXPECT_EQ(
      epoch.feedback_returned(*feedback), ActiveFeedbackCompletionStatus::kReturned);
    EXPECT_EQ(epoch.snapshot().route_phase, ActiveFaultRoutePhase::kUnsecured);
  }
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, EstablishedTerminalAlwaysWinsUnchanged)
{
  constexpr std::array kinds{
    ActiveFaultInvalidationKind::kStartupAuthorityFailure,
    ActiveFaultInvalidationKind::kAdapterInvariantFailure,
    ActiveFaultInvalidationKind::kShutdownTimeout};

  for (const auto kind : kinds) {
    CoordinatorActiveFaultEpoch reserved_epoch(goal_id(), kGeneration);
    auto reserved_output = terminal(
      CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
      kGeneration, "reserved terminal");
    auto publication = offer_publishable(reserved_epoch, reserved_output);
    ASSERT_TRUE(publication);
    const auto reserved_before = reserved_epoch.snapshot();
    const auto reserved_payload = publication->original_output();
    EXPECT_EQ(
      reserved_epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kTerminalWon);
    expect_same_snapshot(reserved_epoch.snapshot(), reserved_before);
    EXPECT_TRUE(publication->live());
    expect_same_output(publication->original_output(), reserved_payload);

    CoordinatorActiveFaultEpoch awaiting_epoch(goal_id(), kGeneration);
    auto awaiting_output = terminal(
      CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled,
      kGeneration, "awaiting acknowledgement terminal");
    auto awaiting_publication = offer_publishable(awaiting_epoch, awaiting_output);
    ASSERT_TRUE(awaiting_publication);
    auto returned = awaiting_epoch.terminal_publication_returned(*awaiting_publication);
    auto acknowledgement = returned.take_ack_permit();
    ASSERT_TRUE(acknowledgement);
    const auto awaiting_before = awaiting_epoch.snapshot();
    const auto awaiting_payload = acknowledgement->original_output();
    EXPECT_EQ(
      awaiting_epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kTerminalWon);
    expect_same_snapshot(awaiting_epoch.snapshot(), awaiting_before);
    EXPECT_TRUE(acknowledgement->live());
    expect_same_output(acknowledgement->original_output(), awaiting_payload);

    CoordinatorActiveFaultEpoch delivered_epoch(goal_id(), kGeneration);
    auto delivered_output = terminal(
      CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
    auto delivered_publication = offer_publishable(delivered_epoch, delivered_output);
    ASSERT_TRUE(delivered_publication);
    auto delivered_returned =
      delivered_epoch.terminal_publication_returned(*delivered_publication);
    auto delivered_acknowledgement = delivered_returned.take_ack_permit();
    ASSERT_TRUE(delivered_acknowledgement);
    ASSERT_EQ(
      delivered_epoch.complete_terminal_ack(*delivered_acknowledgement, true).status(),
      ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
    const auto delivered_before = delivered_epoch.snapshot();
    EXPECT_EQ(
      delivered_epoch.invalidate_fault(kind), ActiveFaultInvalidationStatus::kTerminalWon);
    expect_same_snapshot(delivered_epoch.snapshot(), delivered_before);
  }
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, UnsecuredStatesStayUnchanged)
{
  constexpr std::array kinds{
    ActiveFaultInvalidationKind::kStartupAuthorityFailure,
    ActiveFaultInvalidationKind::kAdapterInvariantFailure,
    ActiveFaultInvalidationKind::kShutdownTimeout};

  for (const auto kind : kinds) {
    CoordinatorActiveFaultEpoch rejected_epoch(goal_id(), kGeneration);
    auto rejected_output = terminal(
      CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
    auto rejected_publication = offer_publishable(rejected_epoch, rejected_output);
    ASSERT_TRUE(rejected_publication);
    auto returned = rejected_epoch.terminal_publication_returned(*rejected_publication);
    auto acknowledgement = returned.take_ack_permit();
    ASSERT_TRUE(acknowledgement);
    ASSERT_EQ(
      rejected_epoch.complete_terminal_ack(*acknowledgement, false).status(),
      ActiveTerminalAcknowledgementStatus::kRejectedDeliveredUnsecured);
    const auto rejected_before = rejected_epoch.snapshot();
    EXPECT_EQ(
      rejected_epoch.invalidate_fault(kind),
      ActiveFaultInvalidationStatus::kAlreadyUnsecured);
    expect_same_snapshot(rejected_epoch.snapshot(), rejected_before);

    CoordinatorActiveFaultEpoch unsecured_epoch(goal_id(), kGeneration);
    complete_route(unsecured_epoch, false);
    const auto unsecured_before = unsecured_epoch.snapshot();
    EXPECT_EQ(
      unsecured_epoch.invalidate_fault(kind),
      ActiveFaultInvalidationStatus::kAlreadyUnsecured);
    expect_same_snapshot(unsecured_epoch.snapshot(), unsecured_before);
  }
}

TEST(CoordinatorActiveFaultEpochInvalidationTest, UnknownKindsHaveFirstPrecedenceAndNeverMutate)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
    kGeneration, "unknown invalidation kind terminal");
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  const auto before = epoch.snapshot();
  const auto expected_output = publication->original_output();

  for (const auto raw : {std::uint8_t{3U}, std::uint8_t{127U}, std::uint8_t{255U}}) {
    EXPECT_EQ(
      epoch.invalidate_fault(static_cast<ActiveFaultInvalidationKind>(raw)),
      ActiveFaultInvalidationStatus::kInvalidKind);
    expect_same_snapshot(epoch.snapshot(), before);
    EXPECT_TRUE(publication->live());
    expect_same_output(publication->original_output(), expected_output);
  }
}

// Card 051 rung 5: the typed recoverable skip must classify as an ordinary aborted terminal.
// An outcome the switch does not know classifies as null and the epoch normalizes the terminal
// to an internal fault — "coordinator fault won before terminal result delivery" — which blocks
// the campaign on exactly the path that exists to keep it running (observed live as outcome=14
// in the predeclared AFTER population at a560ec5, slots 3 and 6).
TEST(CoordinatorActiveFaultEpochTerminalTest, RecoverableSkipClassifiesAsOriginalAbort)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kRecoverableSkip,
    kGeneration, "recoverable skip (rung 5)");
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->classification(), ActiveTerminalClassification::kOriginalAborted);
  EXPECT_EQ(
    publication->original_output().outcome, RestockActionOutcome::kRecoverableSkip);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, TerminalFirstWinsAndAcceptedAckEnablesRetirement)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->classification(), ActiveTerminalClassification::kOriginalSucceeded);

  auto late_fault = epoch.claim_fault(ActiveTerminationRouteKind::kProtocolFailure);
  EXPECT_EQ(late_fault.status(), ActiveFaultClaimStatus::kTerminalWon);
  EXPECT_FALSE(late_fault.take_claim());

  auto returned = epoch.terminal_publication_returned(*publication);
  EXPECT_EQ(returned.status(), ActivePublicationReturnStatus::kAcknowledgementPrepared);
  EXPECT_FALSE(publication->live());
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  auto completion = epoch.complete_terminal_ack(*acknowledgement, true);
  EXPECT_EQ(
    completion.status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  auto accepted_ack = completion.take_accepted_ack_witness();
  ASSERT_TRUE(accepted_ack);
  EXPECT_TRUE(accepted_ack->live());
  EXPECT_EQ(accepted_ack->goal_id(), goal_id());
  EXPECT_EQ(accepted_ack->goal_generation(), kGeneration);
  EXPECT_FALSE(completion.take_accepted_ack_witness());
  EXPECT_FALSE(acknowledgement->live());
  const auto snapshot = epoch.snapshot();
  EXPECT_EQ(snapshot.terminal_phase, ActiveTerminalPhase::kDelivered);
  EXPECT_TRUE(snapshot.middleware_publication_returned);
  EXPECT_TRUE(snapshot.retirement_eligible);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, FaultFirstRetainsSuccessAndNormalizesAfterSecuring)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto route_claim = claim(epoch, ActiveTerminationRouteKind::kCancel);
  ASSERT_TRUE(route_claim);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded,
    kGeneration, "exact retained output");
  auto offered = epoch.offer_terminal(output);
  EXPECT_EQ(offered.disposition(), ActiveTerminalDisposition::kRetainedWhileRouting);
  EXPECT_TRUE(offered.output_was_consumed());
  EXPECT_TRUE(epoch.snapshot().retained_terminal);

  EXPECT_EQ(
    epoch.complete_fault(*route_claim, true), ActiveFaultCompletionStatus::kSecured);
  auto prepared = epoch.prepare_secured_retained_publication();
  EXPECT_EQ(prepared.status(), ActiveRetainedPublicationStatus::kPrepared);
  auto publication = prepared.take_permit();
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->classification(), ActiveTerminalClassification::kInternalFaultAbort);
  EXPECT_EQ(publication->original_output().detail, "exact retained output");
  EXPECT_FALSE(epoch.snapshot().retained_terminal);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, UnsecuredRouteNeverReleasesRetainedOutput)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto route_claim = claim(epoch);
  ASSERT_TRUE(route_claim);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted,
    RestockActionOutcome::kInternalError);
  EXPECT_EQ(
    epoch.offer_terminal(output).disposition(), ActiveTerminalDisposition::kRetainedWhileRouting);
  EXPECT_EQ(
    epoch.complete_fault(*route_claim, false), ActiveFaultCompletionStatus::kUnsecured);
  auto prepared = epoch.prepare_secured_retained_publication();
  EXPECT_EQ(prepared.status(), ActiveRetainedPublicationStatus::kUnsecured);
  EXPECT_FALSE(prepared.take_permit());
  EXPECT_TRUE(epoch.snapshot().retained_terminal);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, ContradictionMatrixPreservesFirstAndInvalidatesClaim)
{
  struct Case
  {
    bool first_valid;
    CoordinatorDriverOutputKind second_kind;
    RestockActionOutcome second_outcome;
    GoalGeneration second_generation;
  };
  const std::array cases{
    Case{true, CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled,
      kGeneration},
    Case{true, CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kCanceled,
      kGeneration},
    Case{true, CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
      kGeneration + 1U},
    Case{false, CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled,
      kGeneration},
  };

  for (const auto & test_case : cases) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto route_claim = claim(epoch);
    ASSERT_TRUE(route_claim);
    auto first = terminal(
      CoordinatorDriverOutputKind::kAborted,
      test_case.first_valid ? RestockActionOutcome::kPlanningFailed :
      RestockActionOutcome::kSucceeded,
      kGeneration, "first evidence");
    enrich_evidence(first);
    const auto expected_first = first;
    EXPECT_EQ(
      epoch.offer_terminal(first).disposition(), ActiveTerminalDisposition::kRetainedWhileRouting);

    auto second = terminal(
      test_case.second_kind, test_case.second_outcome, test_case.second_generation,
      "second evidence");
    const auto expected_second = second;
    auto contradiction = epoch.offer_terminal(second);
    EXPECT_EQ(
      contradiction.disposition(),
      ActiveTerminalDisposition::kContradictoryTerminalOutputUnconsumed);
    EXPECT_FALSE(contradiction.output_was_consumed());
    expect_same_output(second, expected_second);
    ASSERT_NE(epoch.retained_terminal_output(), nullptr);
    expect_same_output(*epoch.retained_terminal_output(), expected_first);
    EXPECT_EQ(
      epoch.complete_fault(*route_claim, true), ActiveFaultCompletionStatus::kStaleClaim);
    EXPECT_TRUE(route_claim->live());
    const auto snapshot = epoch.snapshot();
    EXPECT_TRUE(snapshot.retained_terminal);
    EXPECT_TRUE(snapshot.contradiction_observed);
    EXPECT_EQ(snapshot.route_phase, ActiveFaultRoutePhase::kUnsecured);
  }
}

TEST(CoordinatorActiveFaultEpochTerminalTest, InvalidClearTerminalIsRetainedAndRoutesProtocolFault)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto invalid = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kCanceled,
    kGeneration, "invalid contract");
  auto decision = epoch.offer_terminal(invalid);
  EXPECT_EQ(decision.disposition(), ActiveTerminalDisposition::kRetainedAndFaultClaimed);
  EXPECT_TRUE(decision.output_was_consumed());
  auto route_claim = decision.take_fault_claim();
  ASSERT_TRUE(route_claim);
  EXPECT_EQ(route_claim->kind(), ActiveTerminationRouteKind::kProtocolFailure);
  EXPECT_TRUE(epoch.snapshot().retained_terminal);
}

TEST(
  CoordinatorActiveFaultEpochTerminalTest,
  WrongGenerationRoutesCurrentIdentityWithoutConsumption)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto foreign = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
    kGeneration + 1U, "foreign output");
  auto decision = epoch.offer_terminal(foreign);
  EXPECT_EQ(
    decision.disposition(), ActiveTerminalDisposition::kProtocolClaimedOutputUnconsumed);
  EXPECT_FALSE(decision.output_was_consumed());
  EXPECT_EQ(foreign.detail, "foreign output");
  auto route_claim = decision.take_fault_claim();
  ASSERT_TRUE(route_claim);
  EXPECT_EQ(route_claim->goal_generation(), kGeneration);
  EXPECT_EQ(route_claim->goal_id(), goal_id());
}

TEST(CoordinatorActiveFaultEpochTerminalTest, SecuredInvalidOutputUsesInternalFaultClassification)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto route_claim = claim(epoch);
  ASSERT_TRUE(route_claim);
  ASSERT_EQ(epoch.complete_fault(*route_claim, true), ActiveFaultCompletionStatus::kSecured);
  auto invalid = terminal(
    static_cast<CoordinatorDriverOutputKind>(255U), RestockActionOutcome::kSucceeded,
    kGeneration, "unknown kind");
  auto publication = offer_publishable(
    epoch, invalid, ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization);
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->classification(), ActiveTerminalClassification::kInternalFaultAbort);
  EXPECT_EQ(publication->original_output().detail, "unknown kind");
  EXPECT_EQ(static_cast<std::uint8_t>(publication->original_output().kind), 255U);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, UnknownOutcomeNeverPassesTheClosedContract)
{
  constexpr auto unknown_outcome = static_cast<RestockActionOutcome>(255U);

  CoordinatorActiveFaultEpoch clear_epoch(goal_id(), kGeneration);
  auto clear_output = terminal(
    CoordinatorDriverOutputKind::kAborted, unknown_outcome, kGeneration,
    "unknown outcome in clear phase");
  auto clear_decision = clear_epoch.offer_terminal(clear_output);
  EXPECT_EQ(
    clear_decision.disposition(), ActiveTerminalDisposition::kRetainedAndFaultClaimed);
  EXPECT_TRUE(clear_decision.output_was_consumed());
  EXPECT_TRUE(clear_decision.take_fault_claim());
  ASSERT_NE(clear_epoch.retained_terminal_output(), nullptr);
  EXPECT_EQ(
    static_cast<std::uint8_t>(clear_epoch.retained_terminal_output()->outcome), 255U);

  CoordinatorActiveFaultEpoch secured_epoch(goal_id(), kGeneration);
  complete_route(secured_epoch, true);
  auto secured_output = terminal(
    CoordinatorDriverOutputKind::kAborted, unknown_outcome, kGeneration,
    "unknown outcome in secured phase");
  auto publication = offer_publishable(
    secured_epoch, secured_output,
    ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization);
  ASSERT_TRUE(publication);
  EXPECT_EQ(publication->classification(), ActiveTerminalClassification::kInternalFaultAbort);
  EXPECT_EQ(static_cast<std::uint8_t>(publication->original_output().outcome), 255U);
}

TEST(CoordinatorActiveFaultEpochTerminalTest, ValidAndInvalidTerminalMatrixIsClosedByRoutePhase)
{
  struct Case
  {
    ActiveFaultRoutePhase phase;
    bool valid;
    ActiveTerminalDisposition disposition;
    bool publication;
    bool claim;
  };
  const std::array cases{
    Case{ActiveFaultRoutePhase::kClear, true,
      ActiveTerminalDisposition::kPublicationPrepared, true, false},
    Case{ActiveFaultRoutePhase::kClear, false,
      ActiveTerminalDisposition::kRetainedAndFaultClaimed, false, true},
    Case{ActiveFaultRoutePhase::kRoutingPending, true,
      ActiveTerminalDisposition::kRetainedWhileRouting, false, false},
    Case{ActiveFaultRoutePhase::kRoutingPending, false,
      ActiveTerminalDisposition::kRetainedWhileRouting, false, false},
    Case{ActiveFaultRoutePhase::kSecured, true,
      ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization, true, false},
    Case{ActiveFaultRoutePhase::kSecured, false,
      ActiveTerminalDisposition::kPublicationPreparedWithFaultNormalization, true, false},
    Case{ActiveFaultRoutePhase::kUnsecured, true,
      ActiveTerminalDisposition::kRetainedUnsecured, false, false},
    Case{ActiveFaultRoutePhase::kUnsecured, false,
      ActiveTerminalDisposition::kRetainedUnsecured, false, false},
  };

  for (const auto & test_case : cases) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    std::optional<CoordinatorActiveFaultClaim> pending_claim;
    if (test_case.phase == ActiveFaultRoutePhase::kRoutingPending) {
      pending_claim = claim(epoch);
    } else if (test_case.phase == ActiveFaultRoutePhase::kSecured) {
      complete_route(epoch, true);
    } else if (test_case.phase == ActiveFaultRoutePhase::kUnsecured) {
      complete_route(epoch, false);
    }
    auto output = test_case.valid ?
      terminal(CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded) :
      terminal(CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kCanceled);
    auto decision = epoch.offer_terminal(output);
    EXPECT_EQ(decision.disposition(), test_case.disposition);
    EXPECT_TRUE(decision.output_was_consumed());
    EXPECT_EQ(decision.take_publication_permit().has_value(), test_case.publication);
    EXPECT_EQ(decision.take_fault_claim().has_value(), test_case.claim);
  }
}

TEST(CoordinatorActiveFaultEpochTerminalTest, SecuredRoutePreservesCancelAndAbortClassification)
{
  for (const auto kind : {
        CoordinatorDriverOutputKind::kCanceled, CoordinatorDriverOutputKind::kAborted})
  {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    complete_route(epoch, true);
    auto output = terminal(
      kind,
      kind == CoordinatorDriverOutputKind::kCanceled ?
      RestockActionOutcome::kCanceled : RestockActionOutcome::kInternalError);
    auto publication = offer_publishable(epoch, output);
    ASSERT_TRUE(publication);
    EXPECT_EQ(
      publication->classification(),
      kind == CoordinatorDriverOutputKind::kCanceled ?
      ActiveTerminalClassification::kOriginalCanceled :
      ActiveTerminalClassification::kOriginalAborted);
  }
}

TEST(CoordinatorActiveFaultEpochTerminalTest, WrongGenerationFollowsEveryClosedRoutePhase)
{
  for (const auto phase : {
        ActiveFaultRoutePhase::kRoutingPending, ActiveFaultRoutePhase::kSecured,
        ActiveFaultRoutePhase::kUnsecured})
  {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    std::optional<CoordinatorActiveFaultClaim> pending_claim;
    if (phase == ActiveFaultRoutePhase::kRoutingPending) {
      pending_claim = claim(epoch);
    } else {
      complete_route(epoch, phase == ActiveFaultRoutePhase::kSecured);
    }
    auto output = terminal(
      CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
      kGeneration + 1U, "wrong-generation evidence");
    auto decision = epoch.offer_terminal(output);
    EXPECT_EQ(
      decision.disposition(),
      phase == ActiveFaultRoutePhase::kRoutingPending ?
      ActiveTerminalDisposition::kProtocolAlreadyPendingOutputUnconsumed :
      ActiveTerminalDisposition::kIdentityMismatchOutputUnconsumed);
    EXPECT_FALSE(decision.output_was_consumed());
    EXPECT_EQ(output.detail, "wrong-generation evidence");
    EXPECT_FALSE(decision.take_fault_claim());
    EXPECT_EQ(
      epoch.snapshot().route_phase,
      phase == ActiveFaultRoutePhase::kSecured ? ActiveFaultRoutePhase::kUnsecured : phase);
  }
}

TEST(CoordinatorActiveFaultEpochPublicationTest, UnknownPublicationRestoresExactOutputUnsecured)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
    kGeneration, "publisher ambiguity evidence");
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  EXPECT_EQ(
    epoch.terminal_publication_unknown(*publication),
    ActivePublicationUnknownStatus::kRestoredRetainedUnsecured);
  EXPECT_FALSE(publication->live());
  const auto snapshot = epoch.snapshot();
  EXPECT_EQ(snapshot.route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_EQ(snapshot.terminal_phase, ActiveTerminalPhase::kRetained);
  EXPECT_TRUE(snapshot.retained_terminal);
  EXPECT_FALSE(snapshot.retirement_eligible);
  EXPECT_EQ(
    epoch.terminal_publication_unknown(*publication), ActivePublicationUnknownStatus::kNotLive);
}

TEST(CoordinatorActiveFaultEpochPublicationTest, RejectedAcknowledgementIsDeliveredUnsecured)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  auto returned = epoch.terminal_publication_returned(*publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  auto rejected = epoch.complete_terminal_ack(*acknowledgement, false);
  EXPECT_EQ(
    rejected.status(),
    ActiveTerminalAcknowledgementStatus::kRejectedDeliveredUnsecured);
  EXPECT_FALSE(rejected.take_accepted_ack_witness());
  const auto snapshot = epoch.snapshot();
  EXPECT_EQ(snapshot.terminal_phase, ActiveTerminalPhase::kDelivered);
  EXPECT_EQ(snapshot.route_phase, ActiveFaultRoutePhase::kUnsecured);
  EXPECT_FALSE(snapshot.retirement_eligible);
  auto not_live = epoch.complete_terminal_ack(*acknowledgement, true);
  EXPECT_EQ(
    not_live.status(),
    ActiveTerminalAcknowledgementStatus::kNotLive);
  EXPECT_FALSE(not_live.take_accepted_ack_witness());
}

TEST(CoordinatorActiveFaultEpochPublicationTest, LateFaultReflectsAcceptedOrRejectedAck)
{
  for (const bool accepted : {false, true}) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto output = terminal(
      CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
    auto publication = offer_publishable(epoch, output);
    ASSERT_TRUE(publication);
    auto returned = epoch.terminal_publication_returned(*publication);
    auto acknowledgement = returned.take_ack_permit();
    ASSERT_TRUE(acknowledgement);
    ASSERT_EQ(
      epoch.complete_terminal_ack(*acknowledgement, accepted).status(),
      accepted ? ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible :
      ActiveTerminalAcknowledgementStatus::kRejectedDeliveredUnsecured);

    auto late_fault = epoch.claim_fault(ActiveTerminationRouteKind::kProtocolFailure);
    EXPECT_EQ(
      late_fault.status(),
      accepted ? ActiveFaultClaimStatus::kTerminalWon : ActiveFaultClaimStatus::kUnsecured);
    EXPECT_FALSE(late_fault.take_claim());
  }
}

TEST(CoordinatorActiveFaultEpochPublicationTest, EstablishedTerminalFenceRejectsLaterOutputs)
{
  CoordinatorActiveFaultEpoch reserved_epoch(goal_id(), kGeneration);
  auto first = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(reserved_epoch, first);
  ASSERT_TRUE(publication);
  auto while_reserved = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
    kGeneration, "reserved caller evidence");
  auto reserved_decision = reserved_epoch.offer_terminal(while_reserved);
  EXPECT_EQ(
    reserved_decision.disposition(),
    ActiveTerminalDisposition::kAlreadyReservedOutputUnconsumed);
  EXPECT_FALSE(reserved_decision.output_was_consumed());
  EXPECT_EQ(while_reserved.detail, "reserved caller evidence");
  EXPECT_EQ(
    reserved_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReserved);

  CoordinatorActiveFaultEpoch delivered_epoch(goal_id(), kGeneration);
  auto delivered_first = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto delivered_publication = offer_publishable(delivered_epoch, delivered_first);
  ASSERT_TRUE(delivered_publication);
  auto returned = delivered_epoch.terminal_publication_returned(*delivered_publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  ASSERT_EQ(
    delivered_epoch.complete_terminal_ack(*acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  auto after_delivery = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError,
    kGeneration, "delivered caller evidence");
  auto delivered_decision = delivered_epoch.offer_terminal(after_delivery);
  EXPECT_EQ(
    delivered_decision.disposition(),
    ActiveTerminalDisposition::kAlreadyDeliveredOutputUnconsumed);
  EXPECT_FALSE(delivered_decision.output_was_consumed());
  EXPECT_EQ(after_delivery.detail, "delivered caller evidence");
  EXPECT_TRUE(delivered_epoch.snapshot().retirement_eligible);
}

TEST(CoordinatorActiveFaultEpochPublicationTest, RetainedPublicationReportsEveryFenceState)
{
  CoordinatorActiveFaultEpoch clear_epoch(goal_id(), kGeneration);
  EXPECT_EQ(
    clear_epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kNotSecured);
  EXPECT_EQ(clear_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kEmpty);

  CoordinatorActiveFaultEpoch secured_empty_epoch(goal_id(), kGeneration);
  complete_route(secured_empty_epoch, true);
  EXPECT_EQ(
    secured_empty_epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kNoRetainedTerminal);
  EXPECT_EQ(secured_empty_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kEmpty);

  CoordinatorActiveFaultEpoch reserved_epoch(goal_id(), kGeneration);
  auto reserved_output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(reserved_epoch, reserved_output);
  ASSERT_TRUE(publication);
  EXPECT_EQ(
    reserved_epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kAlreadyReserved);
  EXPECT_EQ(
    reserved_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReserved);

  CoordinatorActiveFaultEpoch delivered_epoch(goal_id(), kGeneration);
  auto delivered_output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto delivered_publication = offer_publishable(delivered_epoch, delivered_output);
  ASSERT_TRUE(delivered_publication);
  auto returned = delivered_epoch.terminal_publication_returned(*delivered_publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  ASSERT_EQ(
    delivered_epoch.complete_terminal_ack(*acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  EXPECT_EQ(
    delivered_epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kAlreadyDelivered);
  EXPECT_TRUE(delivered_epoch.snapshot().retirement_eligible);
}

TEST(CoordinatorActiveFaultEpochPublicationTest, LaterGenerationLeavesPublicationUsableByOwner)
{
  CoordinatorActiveFaultEpoch owner(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch foreign(goal_id(), kGeneration + 1U);
  auto output = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto publication = offer_publishable(owner, output);
  ASSERT_TRUE(publication);
  auto rejected = foreign.terminal_publication_returned(*publication);
  EXPECT_EQ(rejected.status(), ActivePublicationReturnStatus::kIdentityMismatch);
  EXPECT_TRUE(publication->live());
  auto accepted = owner.terminal_publication_returned(*publication);
  EXPECT_EQ(accepted.status(), ActivePublicationReturnStatus::kAcknowledgementPrepared);
}

TEST(CoordinatorActiveFaultEpochPublicationTest, ReturnedPublicationAndAckAreStrictlyOneShot)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  auto returned = epoch.terminal_publication_returned(*publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  EXPECT_EQ(
    epoch.terminal_publication_returned(*publication).status(),
    ActivePublicationReturnStatus::kNotLive);
  EXPECT_EQ(
    epoch.complete_terminal_ack(*acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  EXPECT_EQ(
    epoch.complete_terminal_ack(*acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kNotLive);
}

TEST(
  CoordinatorActiveFaultEpochPublicationTest,
  AckDecisionMovesNormalizeSourceAndExtractionIsOneShot)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto original = complete_accepted_terminal(epoch);

  CoordinatorTerminalAckCompletionDecision moved{std::move(original)};
  EXPECT_EQ(
    original.status(), ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  EXPECT_FALSE(original.take_accepted_ack_witness());
  exercise_self_move(moved);

  auto original_witness = moved.take_accepted_ack_witness();
  ASSERT_TRUE(original_witness);
  EXPECT_FALSE(moved.take_accepted_ack_witness());
  const auto expected_id = original_witness->goal_id();
  const auto expected_generation = original_witness->goal_generation();

  CoordinatorAcceptedTerminalAckWitness moved_witness{std::move(*original_witness)};
  EXPECT_FALSE(original_witness->live());
  EXPECT_TRUE(moved_witness.live());
  EXPECT_EQ(moved_witness.goal_id(), expected_id);
  EXPECT_EQ(moved_witness.goal_generation(), expected_generation);
  exercise_self_move(moved_witness);
  EXPECT_TRUE(moved_witness.live());
}

TEST(CoordinatorActiveFaultEpochPublicationTest, AckDecisionOverwriteAbandonsUntakenWitness)
{
  CoordinatorActiveFaultEpoch abandoned_epoch(goal_id(1U), kGeneration);
  std::optional<CoordinatorTerminalAckCompletionDecision> destination{
    complete_accepted_terminal(abandoned_epoch, "abandoned accepted terminal")};
  EXPECT_TRUE(abandoned_epoch.snapshot().retirement_eligible);

  CoordinatorActiveFaultEpoch owner_epoch(goal_id(2U), kGeneration);
  auto source = complete_accepted_terminal(owner_epoch, "owner accepted terminal");
  EXPECT_TRUE(owner_epoch.snapshot().retirement_eligible);

  *destination = std::move(source);
  EXPECT_EQ(
    source.status(), ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  EXPECT_FALSE(source.take_accepted_ack_witness());
  auto surviving = destination->take_accepted_ack_witness();
  ASSERT_TRUE(surviving);
  EXPECT_EQ(surviving->goal_id(), goal_id(2U));
}

TEST(CoordinatorActiveFaultEpochPublicationTest, AckWitnessOverwritePoisonsSource)
{
  CoordinatorActiveFaultEpoch first_epoch(goal_id(1U), kGeneration);
  auto first_decision = complete_accepted_terminal(first_epoch, "first accepted terminal");
  auto first = first_decision.take_accepted_ack_witness();
  ASSERT_TRUE(first);

  CoordinatorActiveFaultEpoch second_epoch(goal_id(2U), kGeneration);
  auto second_decision = complete_accepted_terminal(second_epoch, "second accepted terminal");
  auto second = second_decision.take_accepted_ack_witness();
  ASSERT_TRUE(second);
  const auto second_id = second->goal_id();

  *first = std::move(*second);
  EXPECT_FALSE(second->live());
  EXPECT_TRUE(first->live());
  EXPECT_EQ(first->goal_id(), second_id);
}

TEST(CoordinatorActiveFaultEpochPublicationTest, LaterGenerationLeavesAckUsableByOwner)
{
  CoordinatorActiveFaultEpoch owner(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch foreign(goal_id(), kGeneration + 1U);
  auto output = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto publication = offer_publishable(owner, output);
  ASSERT_TRUE(publication);
  auto returned = owner.terminal_publication_returned(*publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  const auto foreign_before = foreign.snapshot();
  const auto owner_before = owner.snapshot();
  auto foreign_completion = foreign.complete_terminal_ack(*acknowledgement, true);
  EXPECT_EQ(
    foreign_completion.status(),
    ActiveTerminalAcknowledgementStatus::kIdentityMismatch);
  EXPECT_FALSE(foreign_completion.take_accepted_ack_witness());
  expect_same_snapshot(foreign.snapshot(), foreign_before);
  expect_same_snapshot(owner.snapshot(), owner_before);
  EXPECT_TRUE(acknowledgement->live());
  auto owner_completion = owner.complete_terminal_ack(*acknowledgement, true);
  EXPECT_EQ(
    owner_completion.status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
  EXPECT_TRUE(owner_completion.take_accepted_ack_witness());
}

TEST(CoordinatorActiveFaultEpochPublicationTest, PermitMovesPoisonSourcesAndSelfMoveIsSafe)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  CoordinatorTerminalPublicationPermit moved_publication{std::move(*publication)};
  EXPECT_FALSE(publication->live());
  exercise_self_move(moved_publication);
  EXPECT_TRUE(moved_publication.live());
  auto returned = epoch.terminal_publication_returned(moved_publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  CoordinatorTerminalAcknowledgementPermit moved_acknowledgement{std::move(*acknowledgement)};
  EXPECT_FALSE(acknowledgement->live());
  exercise_self_move(moved_acknowledgement);
  EXPECT_TRUE(moved_acknowledgement.live());
  EXPECT_EQ(
    epoch.complete_terminal_ack(moved_acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, PublicationPermitOutlivesReusedEpochStorage)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::array<std::byte, sizeof(CoordinatorActiveFaultEpoch)> storage{};
  auto * original = new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError);
  auto publication = offer_publishable(*original, output);
  ASSERT_TRUE(publication);
  original->~CoordinatorActiveFaultEpoch();

  auto * replacement =
    new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration + 1U);
  EXPECT_EQ(
    replacement->terminal_publication_returned(*publication).status(),
    ActivePublicationReturnStatus::kIdentityMismatch);
  EXPECT_TRUE(publication->live());
  replacement->~CoordinatorActiveFaultEpoch();
  publication.reset();
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, AckPermitOutlivesReusedEpochStorage)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::array<std::byte, sizeof(CoordinatorActiveFaultEpoch)> storage{};
  auto * original = new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(*original, output);
  ASSERT_TRUE(publication);
  auto returned = original->terminal_publication_returned(*publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  publication.reset();
  original->~CoordinatorActiveFaultEpoch();

  auto * replacement =
    new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration + 1U);
  EXPECT_EQ(
    replacement->complete_terminal_ack(*acknowledgement, true).status(),
    ActiveTerminalAcknowledgementStatus::kIdentityMismatch);
  EXPECT_TRUE(acknowledgement->live());
  replacement->~CoordinatorActiveFaultEpoch();
  acknowledgement.reset();
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, AcceptedAckWitnessOutlivesReusedEpochStorage)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::array<std::byte, sizeof(CoordinatorActiveFaultEpoch)> storage{};
  auto * original = new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration);
  auto completion = complete_accepted_terminal(*original);
  auto accepted_ack = completion.take_accepted_ack_witness();
  ASSERT_TRUE(accepted_ack);
  EXPECT_EQ(accepted_ack->goal_id(), goal_id());
  EXPECT_EQ(accepted_ack->goal_generation(), kGeneration);
  original->~CoordinatorActiveFaultEpoch();

  auto * replacement =
    new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration + 1U);
  replacement->~CoordinatorActiveFaultEpoch();

  EXPECT_TRUE(accepted_ack->live());
  EXPECT_EQ(accepted_ack->goal_generation(), kGeneration);
  accepted_ack.reset();
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, FeedbackPermitOutlivesReusedEpochStorage)
{
  alignas(CoordinatorActiveFaultEpoch)
  std::array<std::byte, sizeof(CoordinatorActiveFaultEpoch)> storage{};
  auto * original = new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration);
  auto reserved = original->reserve_feedback(goal_id(), kGeneration);
  auto feedback = reserved.take_permit();
  ASSERT_TRUE(feedback);
  original->~CoordinatorActiveFaultEpoch();

  auto * replacement =
    new (storage.data()) CoordinatorActiveFaultEpoch(goal_id(), kGeneration + 1U);
  EXPECT_EQ(
    replacement->feedback_returned(*feedback), ActiveFeedbackCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(feedback->live());
  replacement->~CoordinatorActiveFaultEpoch();
  feedback.reset();
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, AbandonedPublicationPermitLeavesFenceBlocked)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  publication.reset();

  EXPECT_EQ(epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReserved);
  EXPECT_FALSE(epoch.snapshot().retirement_eligible);
  EXPECT_EQ(
    epoch.claim_fault(ActiveTerminationRouteKind::kProtocolFailure).status(),
    ActiveFaultClaimStatus::kTerminalWon);
  auto later = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError);
  EXPECT_EQ(
    epoch.offer_terminal(later).disposition(),
    ActiveTerminalDisposition::kAlreadyReservedOutputUnconsumed);
  EXPECT_EQ(
    epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kAlreadyReserved);
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, PublicationOverwriteAbandonsOriginalEpoch)
{
  CoordinatorActiveFaultEpoch abandoned_epoch(goal_id(1U), kGeneration);
  CoordinatorActiveFaultEpoch owner_epoch(goal_id(2U), kGeneration);
  auto abandoned_output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto owner_output = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto abandoned = offer_publishable(abandoned_epoch, abandoned_output);
  auto owner = offer_publishable(owner_epoch, owner_output);
  ASSERT_TRUE(abandoned);
  ASSERT_TRUE(owner);

  *abandoned = std::move(*owner);
  EXPECT_FALSE(owner->live());
  EXPECT_TRUE(abandoned->live());
  EXPECT_EQ(
    abandoned_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReserved);
  EXPECT_FALSE(abandoned_epoch.snapshot().retirement_eligible);
  EXPECT_EQ(
    owner_epoch.terminal_publication_returned(*abandoned).status(),
    ActivePublicationReturnStatus::kAcknowledgementPrepared);
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, AbandonedAckPermitLeavesFenceBlocked)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(epoch, output);
  ASSERT_TRUE(publication);
  auto returned = epoch.terminal_publication_returned(*publication);
  auto acknowledgement = returned.take_ack_permit();
  ASSERT_TRUE(acknowledgement);
  acknowledgement.reset();

  EXPECT_EQ(
    epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReturnedAwaitingAck);
  EXPECT_FALSE(epoch.snapshot().retirement_eligible);
  EXPECT_EQ(
    epoch.claim_fault(ActiveTerminationRouteKind::kProtocolFailure).status(),
    ActiveFaultClaimStatus::kTerminalWon);
  auto later = terminal(
    CoordinatorDriverOutputKind::kAborted, RestockActionOutcome::kInternalError);
  EXPECT_EQ(
    epoch.offer_terminal(later).disposition(),
    ActiveTerminalDisposition::kAlreadyReservedOutputUnconsumed);
}

TEST(CoordinatorActiveFaultEpochLifetimeTest, AckOverwriteAbandonsOriginalEpoch)
{
  CoordinatorActiveFaultEpoch abandoned_epoch(goal_id(1U), kGeneration);
  CoordinatorActiveFaultEpoch owner_epoch(goal_id(2U), kGeneration);
  auto abandoned_output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto owner_output = terminal(
    CoordinatorDriverOutputKind::kCanceled, RestockActionOutcome::kCanceled);
  auto abandoned_publication = offer_publishable(abandoned_epoch, abandoned_output);
  auto owner_publication = offer_publishable(owner_epoch, owner_output);
  ASSERT_TRUE(abandoned_publication);
  ASSERT_TRUE(owner_publication);
  auto abandoned_return = abandoned_epoch.terminal_publication_returned(*abandoned_publication);
  auto owner_return = owner_epoch.terminal_publication_returned(*owner_publication);
  auto abandoned = abandoned_return.take_ack_permit();
  auto owner = owner_return.take_ack_permit();
  ASSERT_TRUE(abandoned);
  ASSERT_TRUE(owner);

  *abandoned = std::move(*owner);
  EXPECT_FALSE(owner->live());
  EXPECT_TRUE(abandoned->live());
  EXPECT_EQ(
    abandoned_epoch.snapshot().terminal_phase,
    ActiveTerminalPhase::kPublicationReturnedAwaitingAck);
  EXPECT_FALSE(abandoned_epoch.snapshot().retirement_eligible);
  EXPECT_EQ(
    owner_epoch.complete_terminal_ack(*abandoned, true).status(),
    ActiveTerminalAcknowledgementStatus::kAcceptedRetirementEligible);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, ReturnClearsReservationAndPreservesClearRoute)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
  EXPECT_EQ(reserved.status(), ActiveFeedbackReserveStatus::kReserved);
  auto permit = reserved.take_permit();
  ASSERT_TRUE(permit);
  EXPECT_EQ(epoch.snapshot().last_feedback_attempt, 1U);
  EXPECT_TRUE(epoch.snapshot().feedback_outstanding);
  EXPECT_EQ(epoch.feedback_returned(*permit), ActiveFeedbackCompletionStatus::kReturned);
  EXPECT_FALSE(permit->live());
  EXPECT_FALSE(epoch.snapshot().feedback_outstanding);
  EXPECT_EQ(epoch.snapshot().route_phase, ActiveFaultRoutePhase::kClear);

  auto second = epoch.reserve_feedback(goal_id(), kGeneration);
  auto second_permit = second.take_permit();
  ASSERT_TRUE(second_permit);
  EXPECT_EQ(epoch.snapshot().last_feedback_attempt, 2U);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, ReserveReportsEveryOccupiedOrClosedFence)
{
  CoordinatorActiveFaultEpoch reserved_epoch(goal_id(), kGeneration);
  auto first_reservation = reserved_epoch.reserve_feedback(goal_id(), kGeneration);
  auto first_permit = first_reservation.take_permit();
  ASSERT_TRUE(first_permit);
  EXPECT_EQ(
    reserved_epoch.reserve_feedback(goal_id(), kGeneration).status(),
    ActiveFeedbackReserveStatus::kAlreadyReserved);
  EXPECT_TRUE(reserved_epoch.snapshot().feedback_outstanding);

  CoordinatorActiveFaultEpoch terminal_epoch(goal_id(), kGeneration);
  auto terminal_output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded);
  auto publication = offer_publishable(terminal_epoch, terminal_output);
  ASSERT_TRUE(publication);
  EXPECT_EQ(
    terminal_epoch.reserve_feedback(goal_id(), kGeneration).status(),
    ActiveFeedbackReserveStatus::kTerminalReserved);
  EXPECT_EQ(
    terminal_epoch.snapshot().terminal_phase, ActiveTerminalPhase::kPublicationReserved);

  CoordinatorActiveFaultEpoch pending_epoch(goal_id(), kGeneration);
  auto pending_claim = claim(pending_epoch);
  ASSERT_TRUE(pending_claim);
  EXPECT_EQ(
    pending_epoch.reserve_feedback(goal_id(), kGeneration).status(),
    ActiveFeedbackReserveStatus::kRoutingPending);
  EXPECT_TRUE(pending_epoch.snapshot().route_claim_outstanding);

  for (const bool secured : {false, true}) {
    CoordinatorActiveFaultEpoch closed_epoch(goal_id(), kGeneration);
    complete_route(closed_epoch, secured);
    EXPECT_EQ(
      closed_epoch.reserve_feedback(goal_id(), kGeneration).status(),
      secured ? ActiveFeedbackReserveStatus::kSecured : ActiveFeedbackReserveStatus::kUnsecured);
    EXPECT_EQ(
      closed_epoch.snapshot().route_phase,
      secured ? ActiveFaultRoutePhase::kSecured : ActiveFaultRoutePhase::kUnsecured);
    EXPECT_FALSE(closed_epoch.snapshot().feedback_outstanding);
  }
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, OutstandingFeedbackBlocksTerminalWithoutConsumption)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
  auto permit = reserved.take_permit();
  ASSERT_TRUE(permit);
  auto output = terminal(
    CoordinatorDriverOutputKind::kSucceeded, RestockActionOutcome::kSucceeded,
    kGeneration, "still caller-owned");
  auto blocked = epoch.offer_terminal(output);
  EXPECT_EQ(
    blocked.disposition(), ActiveTerminalDisposition::kFeedbackOutstandingOutputUnconsumed);
  EXPECT_FALSE(blocked.output_was_consumed());
  EXPECT_EQ(output.detail, "still caller-owned");
  EXPECT_FALSE(epoch.snapshot().retirement_eligible);
  EXPECT_EQ(
    epoch.prepare_secured_retained_publication().status(),
    ActiveRetainedPublicationStatus::kFeedbackOutstanding);

  permit.reset();
  EXPECT_TRUE(epoch.snapshot().feedback_outstanding);
  auto still_blocked = epoch.offer_terminal(output);
  EXPECT_EQ(
    still_blocked.disposition(),
    ActiveTerminalDisposition::kFeedbackOutstandingOutputUnconsumed);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, FailureClaimsAtMostOneProtocolRoute)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
  auto permit = reserved.take_permit();
  ASSERT_TRUE(permit);
  auto failed = epoch.feedback_failed(*permit);
  EXPECT_EQ(failed.status(), ActiveFeedbackCompletionStatus::kFailedAndFaultClaimed);
  auto route_claim = failed.take_fault_claim();
  ASSERT_TRUE(route_claim);
  EXPECT_EQ(route_claim->kind(), ActiveTerminationRouteKind::kProtocolFailure);
  EXPECT_FALSE(epoch.snapshot().feedback_outstanding);
  EXPECT_EQ(epoch.snapshot().route_phase, ActiveFaultRoutePhase::kRoutingPending);
  EXPECT_EQ(
    epoch.feedback_failed(*permit).status(), ActiveFeedbackCompletionStatus::kNotLive);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, FailureAfterRouteTransitionClearsOnlyFeedback)
{
  for (const bool secured : {false, true}) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
    auto feedback = reserved.take_permit();
    ASSERT_TRUE(feedback);
    auto route_claim = claim(epoch, ActiveTerminationRouteKind::kSimulationTimeAuthorityLoss);
    ASSERT_TRUE(route_claim);
    ASSERT_EQ(
      epoch.complete_fault(*route_claim, secured),
      secured ? ActiveFaultCompletionStatus::kSecured : ActiveFaultCompletionStatus::kUnsecured);

    auto failure = epoch.feedback_failed(*feedback);
    EXPECT_EQ(
      failure.status(),
      secured ? ActiveFeedbackCompletionStatus::kFailedRouteAlreadySecured :
      ActiveFeedbackCompletionStatus::kFailedRouteUnsecured);
    EXPECT_FALSE(failure.take_fault_claim());
    EXPECT_FALSE(epoch.snapshot().feedback_outstanding);
    EXPECT_EQ(
      epoch.snapshot().route_phase,
      secured ? ActiveFaultRoutePhase::kSecured : ActiveFaultRoutePhase::kUnsecured);
  }
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, CompletionAfterPendingRouteKeepsTheRoutePending)
{
  CoordinatorActiveFaultEpoch returned_epoch(goal_id(), kGeneration);
  auto returned_reservation = returned_epoch.reserve_feedback(goal_id(), kGeneration);
  auto returned_feedback = returned_reservation.take_permit();
  ASSERT_TRUE(returned_feedback);
  auto returned_route = claim(returned_epoch, ActiveTerminationRouteKind::kCancel);
  ASSERT_TRUE(returned_route);
  EXPECT_EQ(
    returned_epoch.feedback_returned(*returned_feedback),
    ActiveFeedbackCompletionStatus::kReturned);
  EXPECT_EQ(
    returned_epoch.snapshot().route_phase, ActiveFaultRoutePhase::kRoutingPending);
  EXPECT_TRUE(returned_epoch.snapshot().route_claim_outstanding);

  CoordinatorActiveFaultEpoch failed_epoch(goal_id(), kGeneration);
  auto failed_reservation = failed_epoch.reserve_feedback(goal_id(), kGeneration);
  auto failed_feedback = failed_reservation.take_permit();
  ASSERT_TRUE(failed_feedback);
  auto failed_route = claim(failed_epoch, ActiveTerminationRouteKind::kDrain);
  ASSERT_TRUE(failed_route);
  auto failed = failed_epoch.feedback_failed(*failed_feedback);
  EXPECT_EQ(
    failed.status(), ActiveFeedbackCompletionStatus::kFailedRouteAlreadyPending);
  EXPECT_FALSE(failed.take_fault_claim());
  EXPECT_EQ(failed_epoch.snapshot().route_phase, ActiveFaultRoutePhase::kRoutingPending);
  EXPECT_TRUE(failed_epoch.snapshot().route_claim_outstanding);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, ReturnAfterClosedRouteNeverReopensClear)
{
  for (const bool secured : {false, true}) {
    CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
    auto reserved = epoch.reserve_feedback(goal_id(), kGeneration);
    auto feedback = reserved.take_permit();
    ASSERT_TRUE(feedback);
    auto route_claim = claim(epoch);
    ASSERT_TRUE(route_claim);
    ASSERT_EQ(
      epoch.complete_fault(*route_claim, secured),
      secured ? ActiveFaultCompletionStatus::kSecured : ActiveFaultCompletionStatus::kUnsecured);
    EXPECT_EQ(epoch.feedback_returned(*feedback), ActiveFeedbackCompletionStatus::kReturned);
    EXPECT_EQ(
      epoch.snapshot().route_phase,
      secured ? ActiveFaultRoutePhase::kSecured : ActiveFaultRoutePhase::kUnsecured);
  }
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, MoveOverwriteAbandonsOldReservation)
{
  CoordinatorActiveFaultEpoch abandoned_epoch(goal_id(1U), kGeneration);
  CoordinatorActiveFaultEpoch owner_epoch(goal_id(2U), kGeneration);
  auto abandoned_reservation = abandoned_epoch.reserve_feedback(goal_id(1U), kGeneration);
  auto owner_reservation = owner_epoch.reserve_feedback(goal_id(2U), kGeneration);
  auto abandoned = abandoned_reservation.take_permit();
  auto owner = owner_reservation.take_permit();
  ASSERT_TRUE(abandoned);
  ASSERT_TRUE(owner);

  *abandoned = std::move(*owner);
  EXPECT_FALSE(owner->live());
  EXPECT_TRUE(abandoned->live());
  EXPECT_TRUE(abandoned_epoch.snapshot().feedback_outstanding);
  EXPECT_EQ(owner_epoch.feedback_returned(*abandoned), ActiveFeedbackCompletionStatus::kReturned);
  EXPECT_TRUE(abandoned_epoch.snapshot().feedback_outstanding);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, LaterGenerationDoesNotClearOwnersReservation)
{
  CoordinatorActiveFaultEpoch owner(goal_id(), kGeneration);
  CoordinatorActiveFaultEpoch foreign(goal_id(), kGeneration + 1U);
  auto reserved = owner.reserve_feedback(goal_id(), kGeneration);
  auto permit = reserved.take_permit();
  ASSERT_TRUE(permit);
  EXPECT_EQ(
    foreign.feedback_returned(*permit), ActiveFeedbackCompletionStatus::kIdentityMismatch);
  EXPECT_TRUE(permit->live());
  EXPECT_TRUE(owner.snapshot().feedback_outstanding);
  EXPECT_EQ(owner.feedback_returned(*permit), ActiveFeedbackCompletionStatus::kReturned);
}

TEST(CoordinatorActiveFaultEpochFeedbackTest, RejectsInvalidAndMismatchedIdentity)
{
  CoordinatorActiveFaultEpoch epoch(goal_id(), kGeneration);
  EXPECT_EQ(
    epoch.reserve_feedback(CoordinatorGoalId{}, kGeneration).status(),
    ActiveFeedbackReserveStatus::kInvalidArgument);
  EXPECT_EQ(
    epoch.reserve_feedback(goal_id(9U), kGeneration).status(),
    ActiveFeedbackReserveStatus::kIdentityMismatch);
  EXPECT_EQ(
    epoch.reserve_feedback(goal_id(), kGeneration + 1U).status(),
    ActiveFeedbackReserveStatus::kIdentityMismatch);
  EXPECT_FALSE(epoch.snapshot().feedback_outstanding);
}

static_assert(!std::is_default_constructible_v<CoordinatorActiveFaultDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorActiveTerminalDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorRetainedPublicationDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorPublicationReturnDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(!std::is_copy_constructible_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(!std::is_copy_assignable_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(std::is_nothrow_destructible_v<CoordinatorTerminalAckCompletionDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(!std::is_copy_constructible_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(!std::is_copy_assignable_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(std::is_nothrow_move_constructible_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(std::is_nothrow_move_assignable_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(std::is_nothrow_destructible_v<CoordinatorAcceptedTerminalAckWitness>);
static_assert(!std::is_default_constructible_v<CoordinatorFeedbackReserveDecision>);
static_assert(!std::is_default_constructible_v<CoordinatorFeedbackFailureDecision>);
static_assert(std::is_trivially_copyable_v<CoordinatorActiveFaultEpochSnapshot>);
static_assert(noexcept(std::declval<CoordinatorActiveFaultEpoch &>().snapshot()));
static_assert(
  noexcept(std::declval<CoordinatorActiveFaultEpoch &>().claim_fault(
    ActiveTerminationRouteKind::kCancel)));
static_assert(
  noexcept(std::declval<const CoordinatorTerminalAckCompletionDecision &>().status()));
static_assert(
  noexcept(
    std::declval<CoordinatorTerminalAckCompletionDecision &>().
    take_accepted_ack_witness()));
static_assert(noexcept(std::declval<const CoordinatorAcceptedTerminalAckWitness &>().live()));

// Card 086 stage 1b review S2: the result the node builds for a coordinator-fault terminal
// carries no
// motion evidence even though the original output's metrics (nothing commanded yet) say "not
// started", while an ordinary terminal carries what its metrics say.
TEST(ActiveTerminalResultTest, AFaultedTerminalCarriesNoMotionEvidence)
{
  CoordinatorDriverOutput original;
  original.kind = CoordinatorDriverOutputKind::kAborted;
  original.outcome = RestockActionOutcome::kValidationFailed;
  original.detail = "original detail";
  original.metrics.motion_definitely_not_started = true;
  original.metrics.execution_reached_terminal_stop = true;

  const auto faulted = make_active_terminal_result(
    original, ActiveTerminalClassification::kInternalFaultAbort, std::chrono::milliseconds{5},
    "coordinator fault");
  EXPECT_EQ(faulted.outcome, RestockActionOutcome::kInternalError);
  EXPECT_EQ(faulted.detail, "coordinator fault");
  EXPECT_FALSE(faulted.result.motion_definitely_not_started);
  EXPECT_FALSE(faulted.result.execution_reached_terminal_stop);

  const auto ordinary = make_active_terminal_result(
    original, ActiveTerminalClassification::kOriginalAborted, std::chrono::milliseconds{5},
    "coordinator fault");
  EXPECT_EQ(ordinary.outcome, RestockActionOutcome::kValidationFailed);
  EXPECT_EQ(ordinary.detail, "original detail");
  EXPECT_TRUE(ordinary.result.motion_definitely_not_started);
  EXPECT_TRUE(ordinary.result.execution_reached_terminal_stop);
}

}  // namespace
}  // namespace restocker_task_executor
