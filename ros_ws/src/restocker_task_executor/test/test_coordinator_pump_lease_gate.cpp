// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>
#include <new>
#include <optional>
#include <type_traits>
#include <utility>

#include "restocker_task_executor/coordinator_pump_lease_gate.hpp"

namespace restocker_task_executor
{
namespace
{

std::optional<CoordinatorPumpLease> acquire_lease(CoordinatorPumpLeaseGate & gate)
{
  auto decision = gate.acquire(false);
  EXPECT_EQ(decision.status(), CoordinatorPumpLeaseAcquireStatus::kAcquired);
  EXPECT_TRUE(decision.has_lease());
  return decision.take_lease();
}

TEST(CoordinatorPumpLeaseGateValueTest, CheckedAttemptRejectsOnlyMaximum)
{
  static_assert(checked_next_pump_lease_attempt(0U) == PumpLeaseAttempt{1U});
  ASSERT_TRUE(checked_next_pump_lease_attempt(41U));
  EXPECT_EQ(*checked_next_pump_lease_attempt(41U), 42U);
  const auto penultimate = std::numeric_limits<PumpLeaseAttempt>::max() - 1U;
  ASSERT_TRUE(checked_next_pump_lease_attempt(penultimate));
  EXPECT_EQ(
    *checked_next_pump_lease_attempt(penultimate),
    std::numeric_limits<PumpLeaseAttempt>::max());
  EXPECT_FALSE(
    checked_next_pump_lease_attempt(std::numeric_limits<PumpLeaseAttempt>::max()));
}

TEST(CoordinatorPumpLeaseGateValueTest, SealedTypesExposeOnlyNothrowMoveOwnership)
{
  static_assert(!std::is_default_constructible_v<CoordinatorPumpLease>);
  static_assert(!std::is_copy_constructible_v<CoordinatorPumpLease>);
  static_assert(!std::is_copy_assignable_v<CoordinatorPumpLease>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorPumpLease>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorPumpLease>);
  static_assert(std::is_nothrow_destructible_v<CoordinatorPumpLease>);

  static_assert(!std::is_default_constructible_v<CoordinatorPumpLeaseDecision>);
  static_assert(!std::is_copy_constructible_v<CoordinatorPumpLeaseDecision>);
  static_assert(!std::is_copy_assignable_v<CoordinatorPumpLeaseDecision>);
  static_assert(std::is_nothrow_move_constructible_v<CoordinatorPumpLeaseDecision>);
  static_assert(std::is_nothrow_move_assignable_v<CoordinatorPumpLeaseDecision>);
  static_assert(std::is_nothrow_destructible_v<CoordinatorPumpLeaseDecision>);

  static_assert(!std::is_copy_constructible_v<CoordinatorPumpLeaseGate>);
  static_assert(!std::is_move_constructible_v<CoordinatorPumpLeaseGate>);
  static_assert(noexcept(std::declval<CoordinatorPumpLeaseGate &>().acquire(false)));
  static_assert(
    noexcept(
      std::declval<CoordinatorPumpLeaseGate &>().return_lease(
        std::declval<CoordinatorPumpLease &>())));
  static_assert(noexcept(std::declval<const CoordinatorPumpLeaseGate &>().snapshot()));

  SUCCEED();
}

TEST(CoordinatorPumpLeaseGateTest, PendingDecisionDoesNotAdvanceSource)
{
  CoordinatorPumpLeaseGate gate;
  const auto before = gate.snapshot();
  auto decision = gate.acquire(true);

  EXPECT_EQ(decision.status(), CoordinatorPumpLeaseAcquireStatus::kPendingBinding);
  EXPECT_FALSE(decision.has_lease());
  EXPECT_FALSE(decision.take_lease());
  const auto after = gate.snapshot();
  EXPECT_EQ(after.attempt, before.attempt);
  EXPECT_EQ(after.outstanding, before.outstanding);
  EXPECT_FALSE(after.fail_stopped);
  EXPECT_FALSE(after.attempt_exhausted);
}

TEST(CoordinatorPumpLeaseGateTest, ExactReturnAdvancesAttemptsAndExcludesSecondLease)
{
  CoordinatorPumpLeaseGate gate;
  auto first = acquire_lease(gate);
  ASSERT_TRUE(first);
  EXPECT_TRUE(first->live());
  EXPECT_EQ(first->attempt(), 1U);

  auto pending_takes_precedence = gate.acquire(true);
  EXPECT_EQ(
    pending_takes_precedence.status(),
    CoordinatorPumpLeaseAcquireStatus::kPendingBinding);
  EXPECT_FALSE(pending_takes_precedence.has_lease());

  auto occupied = gate.acquire(false);
  EXPECT_EQ(occupied.status(), CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased);
  EXPECT_FALSE(occupied.has_lease());
  EXPECT_EQ(gate.snapshot().attempt, 1U);
  EXPECT_TRUE(gate.snapshot().outstanding);

  EXPECT_EQ(gate.return_lease(*first), CoordinatorPumpLeaseReturnStatus::kReturned);
  EXPECT_FALSE(first->live());
  EXPECT_FALSE(gate.snapshot().outstanding);
  EXPECT_EQ(gate.return_lease(*first), CoordinatorPumpLeaseReturnStatus::kNotLive);

  auto second = acquire_lease(gate);
  ASSERT_TRUE(second);
  EXPECT_EQ(second->attempt(), 2U);
  EXPECT_FALSE(gate.snapshot().fail_stopped);
  EXPECT_FALSE(gate.snapshot().attempt_exhausted);
  EXPECT_EQ(gate.return_lease(*second), CoordinatorPumpLeaseReturnStatus::kReturned);
}

TEST(CoordinatorPumpLeaseGateTest, ForeignReturnPreservesOwnersLiveLease)
{
  CoordinatorPumpLeaseGate owner;
  CoordinatorPumpLeaseGate foreign;
  auto lease = acquire_lease(owner);
  ASSERT_TRUE(lease);

  EXPECT_EQ(
    foreign.return_lease(*lease), CoordinatorPumpLeaseReturnStatus::kForeignIssuer);
  EXPECT_TRUE(lease->live());
  EXPECT_TRUE(owner.snapshot().outstanding);
  EXPECT_FALSE(foreign.snapshot().outstanding);
  EXPECT_EQ(owner.return_lease(*lease), CoordinatorPumpLeaseReturnStatus::kReturned);
  EXPECT_EQ(foreign.return_lease(*lease), CoordinatorPumpLeaseReturnStatus::kNotLive);
}

TEST(CoordinatorPumpLeaseGateTest, DecisionTakeIsOneShotAndMoveNormalizesSource)
{
  CoordinatorPumpLeaseGate gate;
  auto original = gate.acquire(false);
  ASSERT_TRUE(original.has_lease());

  CoordinatorPumpLeaseDecision moved(std::move(original));
  EXPECT_EQ(original.status(), CoordinatorPumpLeaseAcquireStatus::kAcquired);
  EXPECT_FALSE(original.has_lease());
  EXPECT_FALSE(original.take_lease());
  EXPECT_EQ(moved.status(), CoordinatorPumpLeaseAcquireStatus::kAcquired);
  ASSERT_TRUE(moved.has_lease());

  auto lease = moved.take_lease();
  ASSERT_TRUE(lease);
  EXPECT_FALSE(moved.has_lease());
  EXPECT_FALSE(moved.take_lease());
  EXPECT_EQ(gate.return_lease(*lease), CoordinatorPumpLeaseReturnStatus::kReturned);
}

TEST(CoordinatorPumpLeaseGateTest, DecisionOverwriteAbandonsDestinationWithoutClearingGate)
{
  CoordinatorPumpLeaseGate source_gate;
  CoordinatorPumpLeaseGate overwritten_gate;
  auto source = source_gate.acquire(false);
  auto destination = overwritten_gate.acquire(false);
  ASSERT_TRUE(source.has_lease());
  ASSERT_TRUE(destination.has_lease());

  destination = std::move(source);
  EXPECT_FALSE(source.has_lease());
  EXPECT_TRUE(destination.has_lease());
  EXPECT_TRUE(overwritten_gate.snapshot().outstanding);
  auto blocked = overwritten_gate.acquire(false);
  EXPECT_EQ(blocked.status(), CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased);

  auto lease = destination.take_lease();
  ASSERT_TRUE(lease);
  EXPECT_EQ(
    source_gate.return_lease(*lease), CoordinatorPumpLeaseReturnStatus::kReturned);
}

TEST(CoordinatorPumpLeaseGateTest, LeaseMoveAndSelfMovePreserveOnlySourceAuthority)
{
  CoordinatorPumpLeaseGate source_gate;
  CoordinatorPumpLeaseGate overwritten_gate;
  auto source = acquire_lease(source_gate);
  auto destination = acquire_lease(overwritten_gate);
  ASSERT_TRUE(source);
  ASSERT_TRUE(destination);

  *destination = std::move(*source);
  EXPECT_FALSE(source->live());
  EXPECT_TRUE(destination->live());
  EXPECT_TRUE(overwritten_gate.snapshot().outstanding);
  auto * alias = &*destination;
  *destination = std::move(*alias);
  EXPECT_TRUE(destination->live());
  EXPECT_EQ(
    source_gate.return_lease(*destination), CoordinatorPumpLeaseReturnStatus::kReturned);
  EXPECT_EQ(
    overwritten_gate.acquire(false).status(),
    CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased);
}

TEST(CoordinatorPumpLeaseGateTest, AbandonedLeaseAndDecisionRemainFailClosed)
{
  CoordinatorPumpLeaseGate lease_gate;
  {
    auto lease = acquire_lease(lease_gate);
    ASSERT_TRUE(lease);
  }
  EXPECT_TRUE(lease_gate.snapshot().outstanding);
  EXPECT_EQ(
    lease_gate.acquire(false).status(), CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased);

  CoordinatorPumpLeaseGate decision_gate;
  {
    auto decision = decision_gate.acquire(false);
    ASSERT_TRUE(decision.has_lease());
  }
  EXPECT_TRUE(decision_gate.snapshot().outstanding);
  EXPECT_EQ(
    decision_gate.acquire(false).status(),
    CoordinatorPumpLeaseAcquireStatus::kAlreadyLeased);
}

TEST(CoordinatorPumpLeaseGateLifetimeTest, ReusedStorageCannotAuthenticateOldLease)
{
  alignas(CoordinatorPumpLeaseGate)
  std::byte storage[sizeof(CoordinatorPumpLeaseGate)];

  auto * original = new (storage) CoordinatorPumpLeaseGate();
  auto old_lease = acquire_lease(*original);
  ASSERT_TRUE(old_lease);
  original->~CoordinatorPumpLeaseGate();

  auto * replacement = new (storage) CoordinatorPumpLeaseGate();
  EXPECT_EQ(
    replacement->return_lease(*old_lease),
    CoordinatorPumpLeaseReturnStatus::kForeignIssuer);
  EXPECT_TRUE(old_lease->live());
  EXPECT_FALSE(replacement->snapshot().outstanding);

  auto replacement_lease = acquire_lease(*replacement);
  ASSERT_TRUE(replacement_lease);
  EXPECT_EQ(
    replacement->return_lease(*replacement_lease),
    CoordinatorPumpLeaseReturnStatus::kReturned);
  replacement->~CoordinatorPumpLeaseGate();
}

}  // namespace
}  // namespace restocker_task_executor
