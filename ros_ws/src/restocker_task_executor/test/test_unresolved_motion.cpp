// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

// Card 086 stage 1 (CMB-SPEC-13): the settlement table and the attempt registry, pure. No ROS.

#include <gtest/gtest.h>

#include <string>

#include "restocker_task_executor/unresolved_motion.hpp"

namespace restocker_task_executor
{
namespace
{

using Kind = MotionEvidence::Kind;
using Result = UnresolvedMotionRegistry::Result;

MotionEvidence terminal(
  bool arrived, bool not_started, bool stop, bool descendant_unsettled = false)
{
  MotionEvidence evidence;
  evidence.kind = Kind::kDeliveredTerminal;
  evidence.arrived = arrived;
  evidence.motion_definitely_not_started = not_started;
  evidence.execution_reached_terminal_stop = stop;
  evidence.descendant_unsettled = descendant_unsettled;
  return evidence;
}

MotionEvidence non_settling(const std::string & note)
{
  MotionEvidence evidence;
  evidence.kind = Kind::kNonSettling;
  evidence.note = note;
  return evidence;
}

AttemptGoalId goal_id(std::uint8_t seed)
{
  AttemptGoalId id{};
  id.fill(seed);
  return id;
}

// -- the settlement table ---------------------------------------------------------------------

TEST(SettlementTable, ExplicitAdmissionRejectionSettles)
{
  MotionEvidence evidence;
  evidence.kind = Kind::kAdmissionRejected;
  EXPECT_TRUE(settles_attempt(evidence));
}

TEST(SettlementTable, DeliveredTerminalWithNonStartSettles)
{
  EXPECT_TRUE(settles_attempt(terminal(false, true, false)));
}

TEST(SettlementTable, DeliveredArrivalSettles)
{
  EXPECT_TRUE(settles_attempt(terminal(true, false, false)));
}

TEST(SettlementTable, DeliveredTerminalWithStopEvidenceSettles)
{
  EXPECT_TRUE(settles_attempt(terminal(false, false, true)));
}

TEST(SettlementTable, DeliveredFailureWithNeitherNonStartNorStopDoesNotSettle)
{
  EXPECT_FALSE(settles_attempt(terminal(false, false, false)));
}

TEST(SettlementTable, ParentTerminalWithAnUnsettledDescendantDoesNotSettle)
{
  // The tray shutdown leg: a parent that returned while its child may still be admitted.
  EXPECT_FALSE(settles_attempt(terminal(false, false, true, true)));
  EXPECT_FALSE(settles_attempt(terminal(true, false, false, true)));
  EXPECT_FALSE(settles_attempt(terminal(false, true, false, true)));
}

TEST(SettlementTable, NonSettlingEvidenceNeverSettles)
{
  // UNKNOWN result code, admission timeout, cancel answered "goal unknown", quiet telemetry,
  // another endpoint's status, a fresh world revision: none of them is evidence about this
  // attempt's motion, whatever flags a careless caller leaves set on them.
  for (const char * note :
    {"result code UNKNOWN", "admission timeout elapsed", "cancel response: goal unknown",
      "quiet joint-state sample", "coordinator status inhibited=false",
      "fresh world revision", "elapsed backoff"})
  {
    auto evidence = non_settling(note);
    EXPECT_FALSE(settles_attempt(evidence)) << note;
    evidence.arrived = true;
    evidence.motion_definitely_not_started = true;
    evidence.execution_reached_terminal_stop = true;
    EXPECT_FALSE(settles_attempt(evidence)) << note << " with flags set";
  }
}

// -- the registry -------------------------------------------------------------------------------

TEST(UnresolvedMotionRegistry, StartsClearAndOpenedAttemptsAreUnresolved)
{
  UnresolvedMotionRegistry registry;
  EXPECT_FALSE(registry.unresolved());
  EXPECT_TRUE(registry.describe().empty());
  const auto id = registry.open(
    MotionEndpoint::kTray, UnresolvedCondition::kAdmissionUnresolved, "tray class 1");
  EXPECT_TRUE(registry.unresolved());
  EXPECT_EQ(registry.size(), 1U);
  EXPECT_NE(registry.describe().find("motion_unresolved"), std::string::npos);
  EXPECT_NE(registry.describe().find("tray#" + std::to_string(id)), std::string::npos);
  EXPECT_NE(registry.describe().find("admission_unresolved"), std::string::npos);
}

TEST(UnresolvedMotionRegistry, LateAdmissionGrowsTheRecordAndNeverClearsIt)
{
  UnresolvedMotionRegistry registry;
  const auto id = registry.open(
    MotionEndpoint::kViewpoint, UnresolvedCondition::kAdmissionUnresolved, "retreat");
  EXPECT_EQ(registry.bind_goal(id, goal_id(0xab)), Result::kStillUnresolved);
  EXPECT_EQ(
    registry.set_condition(id, UnresolvedCondition::kSettlementPendingCancel),
    Result::kStillUnresolved);
  ASSERT_NE(registry.find(id), nullptr);
  EXPECT_EQ(*registry.find(id)->goal_id, goal_id(0xab));
  EXPECT_TRUE(registry.unresolved());
  EXPECT_NE(registry.describe().find("abababab"), std::string::npos);
  EXPECT_NE(registry.describe().find("settlement_pending_cancel"), std::string::npos);
}

TEST(UnresolvedMotionRegistry, NonSettlingEvidenceAddsAFragmentAndKeepsTheRecord)
{
  UnresolvedMotionRegistry registry;
  const auto id = registry.open(
    MotionEndpoint::kLane, UnresolvedCondition::kResultUnresolved, "lane_01");
  EXPECT_EQ(registry.apply(id, non_settling("result code UNKNOWN")), Result::kStillUnresolved);
  EXPECT_EQ(registry.apply(id, terminal(false, false, false)), Result::kStillUnresolved);
  ASSERT_NE(registry.find(id), nullptr);
  EXPECT_EQ(registry.find(id)->fragments.size(), 2U);
  EXPECT_TRUE(registry.unresolved());
}

TEST(UnresolvedMotionRegistry, SettlementIsIdempotent)
{
  UnresolvedMotionRegistry registry;
  const auto id = registry.open(
    MotionEndpoint::kRestock, UnresolvedCondition::kResultUnresolved, "transfer");
  EXPECT_EQ(registry.apply(id, terminal(true, false, false)), Result::kSettled);
  EXPECT_FALSE(registry.unresolved());
  // The duplicate settles nothing a second time and does not resurrect or throw.
  EXPECT_EQ(registry.apply(id, terminal(true, false, false)), Result::kUnknownAttempt);
  EXPECT_FALSE(registry.unresolved());
}

TEST(UnresolvedMotionRegistry, StaleEvidenceForARetiredAttemptCannotClearANewerOne)
{
  UnresolvedMotionRegistry registry;
  const auto first = registry.open(
    MotionEndpoint::kTray, UnresolvedCondition::kResultUnresolved, "first");
  ASSERT_EQ(registry.apply(first, terminal(true, false, false)), Result::kSettled);
  const auto second = registry.open(
    MotionEndpoint::kTray, UnresolvedCondition::kResultUnresolved, "second");
  EXPECT_NE(first, second);
  // Late duplicate evidence for the retired first attempt.
  EXPECT_EQ(registry.apply(first, terminal(true, false, false)), Result::kUnknownAttempt);
  EXPECT_TRUE(registry.unresolved());
  EXPECT_NE(registry.find(second), nullptr);
}

TEST(UnresolvedMotionRegistry, EvidenceRoutesByExactGoalIdentityOnly)
{
  UnresolvedMotionRegistry registry;
  const auto first = registry.open(
    MotionEndpoint::kLane, UnresolvedCondition::kSettlementPendingCancel, "a");
  const auto second = registry.open(
    MotionEndpoint::kLane, UnresolvedCondition::kSettlementPendingCancel, "b");
  ASSERT_EQ(registry.bind_goal(first, goal_id(1)), Result::kStillUnresolved);
  ASSERT_EQ(registry.bind_goal(second, goal_id(2)), Result::kStillUnresolved);
  // A UUID no live record carries is a no-op, even with settling evidence.
  EXPECT_EQ(
    registry.apply_for_goal(goal_id(9), terminal(true, false, false)),
    Result::kUnknownAttempt);
  EXPECT_EQ(registry.size(), 2U);
  EXPECT_EQ(registry.apply_for_goal(goal_id(2), terminal(true, false, false)), Result::kSettled);
  EXPECT_EQ(registry.size(), 1U);
  EXPECT_NE(registry.find(first), nullptr);
  EXPECT_EQ(registry.find(second), nullptr);
}

TEST(UnresolvedMotionRegistry, ARejectionSettlesExactlyOneAttempt)
{
  // The inhibited coordinator refuses the campaign's restock goal (082 latch): that settles that
  // attempt only; an unrelated unresolved record persists.
  UnresolvedMotionRegistry registry;
  const auto lane = registry.open(
    MotionEndpoint::kLane, UnresolvedCondition::kAdmissionUnresolved, "lane_03");
  const auto restock = registry.open(
    MotionEndpoint::kRestock, UnresolvedCondition::kAdmissionUnresolved, "transfer");
  MotionEvidence rejection;
  rejection.kind = Kind::kAdmissionRejected;
  EXPECT_EQ(registry.apply(restock, rejection), Result::kSettled);
  EXPECT_TRUE(registry.unresolved());
  EXPECT_NE(registry.find(lane), nullptr);
  EXPECT_EQ(registry.size(), 1U);
}

TEST(UnresolvedMotionRegistry, ClearsOnlyWhenEveryTrackedAttemptSettles)
{
  UnresolvedMotionRegistry registry;
  const auto a = registry.open(
    MotionEndpoint::kTray, UnresolvedCondition::kAdmissionUnresolved, "a");
  const auto b = registry.open(
    MotionEndpoint::kViewpoint, UnresolvedCondition::kAdmissionUnresolved, "b");
  ASSERT_EQ(registry.apply(a, terminal(false, true, false)), Result::kSettled);
  EXPECT_TRUE(registry.unresolved());
  ASSERT_EQ(registry.apply(b, terminal(false, false, true)), Result::kSettled);
  EXPECT_FALSE(registry.unresolved());
  EXPECT_TRUE(registry.describe().empty());
}

}  // namespace
}  // namespace restocker_task_executor
