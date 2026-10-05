// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/goal_termination_authority.hpp"

namespace restocker_task_executor
{
namespace
{

using namespace std::chrono_literals;

[[nodiscard]] CoordinatorGoalId goal_id(std::uint8_t seed = 1U)
{
  CoordinatorGoalId value{};
  value.front() = seed;
  return value;
}

[[nodiscard]] std::optional<GoalTerminationSourceGeneration> source_for(
  GoalTerminationKind kind)
{
  switch (kind) {
    case GoalTerminationKind::kTaskDeadline:
      return GoalTerminationSourceGeneration{GoalTerminationSource::kTaskDeadline, 7U};
    case GoalTerminationKind::kMotionDeadline:
      return GoalTerminationSourceGeneration{GoalTerminationSource::kMotionFence, 11U};
    default:
      return std::nullopt;
  }
}

[[nodiscard]] FirstGoalTerminationRecord record(
  GoalTerminationKind kind = GoalTerminationKind::kSafeAbort)
{
  return FirstGoalTerminationRecord{
    GoalTerminationScope::kGoal,
    goal_id(),
    3U,
    kind,
    *cancellation_reason_for(kind),
    SteadyTime{} + 42ms,
    source_for(kind)};
}

[[nodiscard]] std::shared_ptr<const FirstGoalTerminationRecord> evidence(
  FirstGoalTerminationRecord value)
{
  return std::make_shared<const FirstGoalTerminationRecord>(std::move(value));
}

TEST(GoalTerminationAuthority, EnforcesClosedSourceCorrelationMatrix)
{
  auto value = record(GoalTerminationKind::kUserCancel);
  value.source_generation =
    GoalTerminationSourceGeneration{GoalTerminationSource::kNone, 1U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kInvalidSourceKind);

  value = record(GoalTerminationKind::kTaskDeadline);
  value.source_generation = GoalTerminationSourceGeneration{
    static_cast<GoalTerminationSource>(255U), 1U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kInvalidSourceKind);

  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kCoordinatorInbox, 0U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kInvalidSourceGeneration);

  for (const auto kind :
    {GoalTerminationKind::kUserCancel, GoalTerminationKind::kCoordinatorDrain,
      GoalTerminationKind::kSafeAbort, GoalTerminationKind::kShutdown,
      GoalTerminationKind::kProtocolFailure})
  {
    value = record(kind);
    value.source_generation = GoalTerminationSourceGeneration{
      GoalTerminationSource::kCoordinatorInbox, 1U};
    EXPECT_EQ(
      validate_goal_termination_record(value),
      GoalTerminationValidationError::kUnexpectedSource);
  }

  for (const auto kind :
    {GoalTerminationKind::kTaskDeadline, GoalTerminationKind::kMotionDeadline})
  {
    value = record(kind);
    value.source_generation.reset();
    EXPECT_EQ(
      validate_goal_termination_record(value), GoalTerminationValidationError::kMissingSource);
  }

  value = record(GoalTerminationKind::kTaskDeadline);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kMotionFence, 7U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kUnexpectedSource);

  value = record(GoalTerminationKind::kMotionDeadline);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kTaskDeadline, 11U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kUnexpectedSource);

  value = record(GoalTerminationKind::kAuthorityLoss);
  EXPECT_EQ(
    validate_goal_termination_record(value), GoalTerminationValidationError::kNone);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kAuthorityObservation, 5U};
  EXPECT_EQ(
    validate_goal_termination_record(value), GoalTerminationValidationError::kNone);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kCoordinatorInbox, 5U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kUnexpectedSource);

  value = record(GoalTerminationKind::kInboxOverflow);
  EXPECT_EQ(
    validate_goal_termination_record(value), GoalTerminationValidationError::kNone);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kCoordinatorInbox, 9U};
  EXPECT_EQ(
    validate_goal_termination_record(value), GoalTerminationValidationError::kNone);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kAuthorityObservation, 9U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kUnexpectedSource);

  value = record(GoalTerminationKind::kProtocolFailure);
  value.source_generation = GoalTerminationSourceGeneration{
    GoalTerminationSource::kCoordinatorInbox, 1U};
  EXPECT_EQ(
    validate_goal_termination_record(value),
    GoalTerminationValidationError::kUnexpectedSource);
}

TEST(GoalTerminationAuthority, ComparesEvidenceByValueRatherThanBackingIdentity)
{
  const auto lhs = evidence(record());
  const auto rhs = evidence(record());
  ASSERT_NE(lhs.get(), rhs.get());
  EXPECT_TRUE(same_goal_termination_evidence(lhs, rhs));
  EXPECT_EQ(
    goal_termination_evidence_fingerprint(lhs), goal_termination_evidence_fingerprint(rhs));

  auto changed_value = record();
  changed_value.arrived_at += 1ns;
  const auto changed = evidence(changed_value);
  EXPECT_FALSE(same_goal_termination_evidence(lhs, changed));
  EXPECT_NE(
    goal_termination_evidence_fingerprint(lhs), goal_termination_evidence_fingerprint(changed));

  EXPECT_TRUE(same_goal_termination_evidence({}, {}));
  EXPECT_FALSE(same_goal_termination_evidence(lhs, {}));
  EXPECT_FALSE(same_goal_termination_evidence({}, rhs));
  EXPECT_NE(goal_termination_evidence_fingerprint(lhs), goal_termination_evidence_fingerprint({}));
}

TEST(GoalTerminationAuthority, FingerprintHasStableGoldenVectors)
{
  EXPECT_EQ(goal_termination_evidence_fingerprint({}), UINT64_C(0xbb16407c5bde1ccc));
  EXPECT_EQ(
    goal_termination_evidence_fingerprint(evidence(record(GoalTerminationKind::kTaskDeadline))),
    UINT64_C(0xf8bc6f5db18fb167));
}

static_assert(std::is_nothrow_copy_constructible_v<GoalTerminationSourceGeneration>);
static_assert(std::is_nothrow_move_constructible_v<GoalTerminationSourceGeneration>);
static_assert(std::is_nothrow_copy_assignable_v<GoalTerminationSourceGeneration>);
static_assert(std::is_nothrow_move_assignable_v<GoalTerminationSourceGeneration>);
static_assert(std::is_nothrow_destructible_v<GoalTerminationSourceGeneration>);
static_assert(std::is_trivially_copyable_v<GoalTerminationSourceGeneration>);
static_assert(std::is_standard_layout_v<GoalTerminationSourceGeneration>);
static_assert(std::is_nothrow_copy_constructible_v<FirstGoalTerminationRecord>);
static_assert(std::is_nothrow_move_constructible_v<FirstGoalTerminationRecord>);
static_assert(std::is_nothrow_copy_assignable_v<FirstGoalTerminationRecord>);
static_assert(std::is_nothrow_move_assignable_v<FirstGoalTerminationRecord>);
static_assert(std::is_nothrow_destructible_v<FirstGoalTerminationRecord>);
static_assert(!std::is_trivially_copyable_v<FirstGoalTerminationRecord>);
static_assert(std::is_standard_layout_v<FirstGoalTerminationRecord>);
using ImmutableTerminationHandle = std::shared_ptr<const FirstGoalTerminationRecord>;
static_assert(std::is_nothrow_copy_constructible_v<ImmutableTerminationHandle>);
static_assert(std::is_nothrow_move_constructible_v<ImmutableTerminationHandle>);
static_assert(std::is_nothrow_copy_assignable_v<ImmutableTerminationHandle>);
static_assert(std::is_nothrow_move_assignable_v<ImmutableTerminationHandle>);
static_assert(std::is_nothrow_destructible_v<ImmutableTerminationHandle>);
static_assert(
  noexcept(cancellation_reason_for(std::declval<GoalTerminationKind>())));
static_assert(
  noexcept(validate_goal_termination_record(
    std::declval<const FirstGoalTerminationRecord &>())));
static_assert(
  noexcept(
    std::declval<const FirstGoalTerminationRecord &>() ==
    std::declval<const FirstGoalTerminationRecord &>()));
static_assert(
  noexcept(same_goal_termination_evidence(
    std::declval<const std::shared_ptr<const FirstGoalTerminationRecord> &>(),
    std::declval<const std::shared_ptr<const FirstGoalTerminationRecord> &>())));
static_assert(
  noexcept(goal_termination_evidence_fingerprint(
    std::declval<const std::shared_ptr<const FirstGoalTerminationRecord> &>())));
static_assert(!std::is_default_constructible_v<GoalTerminationLatchDecision>);
static_assert(!std::is_default_constructible_v<GoalTerminationSnapshotDecision>);
static_assert(
  !std::is_constructible_v<GoalTerminationLatchDecision, GoalTerminationLatchStatus,
  ImmutableTerminationHandle>);
static_assert(
  !std::is_constructible_v<GoalTerminationSnapshotDecision, GoalTerminationSnapshotStatus,
  ImmutableTerminationHandle>);
static_assert(std::is_final_v<GoalTerminationLatchDecision>);
static_assert(std::is_final_v<GoalTerminationSnapshotDecision>);
static_assert(std::is_nothrow_copy_constructible_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_move_constructible_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_copy_assignable_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_move_assignable_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_destructible_v<GoalTerminationLatchDecision>);
static_assert(std::is_nothrow_copy_constructible_v<GoalTerminationSnapshotDecision>);
static_assert(std::is_nothrow_move_constructible_v<GoalTerminationSnapshotDecision>);
static_assert(std::is_nothrow_copy_assignable_v<GoalTerminationSnapshotDecision>);
static_assert(std::is_nothrow_move_assignable_v<GoalTerminationSnapshotDecision>);
static_assert(std::is_nothrow_destructible_v<GoalTerminationSnapshotDecision>);
static_assert(
  noexcept(std::declval<const GoalTerminationLatchDecision &>().status()));
static_assert(
  noexcept(std::declval<const GoalTerminationLatchDecision &>().record()));
static_assert(
  noexcept(std::declval<const GoalTerminationSnapshotDecision &>().status()));
static_assert(
  noexcept(std::declval<const GoalTerminationSnapshotDecision &>().record()));

}  // namespace
}  // namespace restocker_task_executor
