// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "restocker_task_executor/attachment_port.hpp"

#include "fake_attachment_port.hpp"

namespace restocker_task_executor
{
namespace
{

constexpr AttachmentOutcome kAllOutcomes[]{
  AttachmentOutcome::kSucceeded, AttachmentOutcome::kRejected, AttachmentOutcome::kUnavailable,
  AttachmentOutcome::kCanceled, AttachmentOutcome::kPhysicalFailed,
  AttachmentOutcome::kUncommitted, AttachmentOutcome::kIndeterminate};

[[nodiscard]] AttachmentGoal usable_attach_goal()
{
  AttachmentGoal goal;
  goal.direction = AttachmentDirection::kAttach;
  goal.object_id = 7U;
  goal.reservation_token = "reservation-capability";
  goal.grasp_center_to_child = Eigen::Isometry3d::Identity();
  goal.grasp_center_to_child.translation() = Eigen::Vector3d{0.0, 0.0, 0.03};
  goal.grasp_center_to_child.linear() =
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return goal;
}

[[nodiscard]] AttachmentGoal usable_detach_goal()
{
  AttachmentGoal goal;
  goal.direction = AttachmentDirection::kDetach;
  goal.object_id = 7U;
  goal.reservation_token = "reservation-capability";
  return goal;
}

TEST(AttachmentGoal, AcceptsScopedPhysicalAndSemanticDetachGoals)
{
  auto physical = usable_detach_goal();
  physical.scope = AttachmentScope::kPhysicalOnly;
  EXPECT_TRUE(valid_attachment_goal(physical));

  auto semantic = usable_detach_goal();
  semantic.scope = AttachmentScope::kSemanticOnly;
  semantic.released_at.sec = 1;
  EXPECT_TRUE(valid_attachment_goal(semantic));

  auto attach_physical = usable_attach_goal();
  attach_physical.scope = AttachmentScope::kPhysicalOnly;
  EXPECT_FALSE(valid_attachment_goal(attach_physical));

  semantic.released_at = {};
  EXPECT_FALSE(valid_attachment_goal(semantic));
}

TEST(AttachmentGoal, AcceptsAnAttachWithACapabilityAndARigidExpectedGrasp)
{
  EXPECT_TRUE(valid_attachment_goal(usable_attach_goal()));
}

TEST(AttachmentGoal, AcceptsADetachWithTheDefaultPoseTheBoundaryDemands)
{
  EXPECT_TRUE(valid_attachment_goal(usable_detach_goal()));
}

TEST(AttachmentGoal, RejectsAMissingObjectOrReservationCapability)
{
  auto goal = usable_attach_goal();
  goal.object_id = 0U;
  EXPECT_FALSE(valid_attachment_goal(goal));

  goal = usable_attach_goal();
  goal.reservation_token.clear();
  EXPECT_FALSE(valid_attachment_goal(goal));
}

TEST(AttachmentGoal, RejectsNonFiniteExpectedGraspComponents)
{
  auto goal = usable_attach_goal();
  goal.grasp_center_to_child.translation().y() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_attachment_goal(goal));

  goal = usable_attach_goal();
  goal.grasp_center_to_child.translation().z() = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_attachment_goal(goal));
}

TEST(AttachmentGoal, RejectsANonOrthonormalExpectedGrasp)
{
  // The boundary canonicalizes the pose into a unit quaternion before comparing it against the
  // observed transform, so a scaled rotation would silently become a different expectation.
  auto goal = usable_attach_goal();
  goal.grasp_center_to_child.linear() *= 1.5;
  EXPECT_FALSE(valid_attachment_goal(goal));
}

TEST(AttachmentGoal, RejectsADetachThatCarriesAnExpectedGrasp)
{
  // The boundary's request conversion refuses any detach whose expected pose is not the default.
  auto goal = usable_detach_goal();
  goal.grasp_center_to_child.translation().x() = 0.001;
  EXPECT_FALSE(valid_attachment_goal(goal));

  goal = usable_detach_goal();
  goal.grasp_center_to_child.linear() =
    Eigen::AngleAxisd(0.01, Eigen::Vector3d::UnitX()).toRotationMatrix();
  EXPECT_FALSE(valid_attachment_goal(goal));
}

TEST(AttachmentGoal, RejectsUnusableTimeouts)
{
  auto goal = usable_attach_goal();
  goal.lease_timeout = std::chrono::milliseconds{0};
  EXPECT_FALSE(valid_attachment_goal(goal));

  goal = usable_attach_goal();
  goal.physical_timeout = std::chrono::milliseconds{-1};
  EXPECT_FALSE(valid_attachment_goal(goal));

  goal = usable_attach_goal();
  goal.commit_timeout = std::chrono::milliseconds{0};
  EXPECT_FALSE(valid_attachment_goal(goal));
}

TEST(AttachmentOutcome, ClassifiesEveryOutcomeIntoExactlyOneWorldEffect)
{
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kSucceeded),
    AttachmentWorldEffect::kFullyApplied);

  // Only these four prove the transaction touched neither Gazebo nor world state.
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kRejected), AttachmentWorldEffect::kNoneApplied);
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kUnavailable), AttachmentWorldEffect::kNoneApplied);
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kCanceled), AttachmentWorldEffect::kNoneApplied);
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kPhysicalFailed),
    AttachmentWorldEffect::kNoneApplied);

  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kUncommitted),
    AttachmentWorldEffect::kPhysicalOnly);
  EXPECT_EQ(
    attachment_world_effect(AttachmentOutcome::kIndeterminate),
    AttachmentWorldEffect::kIndeterminate);
}

TEST(AttachmentOutcome, OnlyProvenCleanOutcomesAllowContinuingWithoutReconciliation)
{
  EXPECT_TRUE(attachment_definitely_not_applied(AttachmentOutcome::kRejected));
  EXPECT_TRUE(attachment_definitely_not_applied(AttachmentOutcome::kUnavailable));
  EXPECT_TRUE(attachment_definitely_not_applied(AttachmentOutcome::kCanceled));
  EXPECT_TRUE(attachment_definitely_not_applied(AttachmentOutcome::kPhysicalFailed));

  // Success mutated both systems, so it is not "nothing happened".
  EXPECT_FALSE(attachment_definitely_not_applied(AttachmentOutcome::kSucceeded));
  EXPECT_FALSE(attachment_definitely_not_applied(AttachmentOutcome::kUncommitted));
  EXPECT_FALSE(attachment_definitely_not_applied(AttachmentOutcome::kIndeterminate));
}

TEST(AttachmentOutcome, BothDivergentOutcomesDemandAnOperator)
{
  // A physically applied but uncommitted transition cannot be undone automatically: the boundary
  // only authorizes the inverse mutation from the reservation stage the failed commit never
  // reached. It is as unrunnable as an unproven outcome.
  EXPECT_TRUE(attachment_requires_operator(AttachmentOutcome::kUncommitted));
  EXPECT_TRUE(attachment_requires_operator(AttachmentOutcome::kIndeterminate));

  EXPECT_FALSE(attachment_requires_operator(AttachmentOutcome::kSucceeded));
  EXPECT_FALSE(attachment_requires_operator(AttachmentOutcome::kRejected));
  EXPECT_FALSE(attachment_requires_operator(AttachmentOutcome::kUnavailable));
  EXPECT_FALSE(attachment_requires_operator(AttachmentOutcome::kCanceled));
  EXPECT_FALSE(attachment_requires_operator(AttachmentOutcome::kPhysicalFailed));
}

TEST(AttachmentOutcome, NoOutcomeIsBothCleanAndOperatorRequired)
{
  for (const auto outcome : kAllOutcomes) {
    EXPECT_FALSE(
      attachment_definitely_not_applied(outcome) && attachment_requires_operator(outcome))
      << attachment_outcome_name(outcome);
  }
}

TEST(AttachmentOutcome, EveryOutcomeHasADistinctName)
{
  std::set<std::string> names;
  for (const auto outcome : kAllOutcomes) {
    const std::string name = attachment_outcome_name(outcome);
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, "unknown");
    EXPECT_TRUE(names.insert(name).second) << name;
  }
}

TEST(AttachmentOutcome, AnUnmodelledOutcomeFailsClosed)
{
  // A value added to the enum without updating the classifier must not be mistaken for a clean
  // world. The switch's fallthrough is what guarantees that.
  const auto invented = static_cast<AttachmentOutcome>(200);
  EXPECT_EQ(attachment_world_effect(invented), AttachmentWorldEffect::kIndeterminate);
  EXPECT_FALSE(attachment_definitely_not_applied(invented));
  EXPECT_TRUE(attachment_requires_operator(invented));
}

TEST(AttachmentDirection, EveryDirectionHasADistinctName)
{
  const std::string attach = attachment_direction_name(AttachmentDirection::kAttach);
  const std::string detach = attachment_direction_name(AttachmentDirection::kDetach);
  EXPECT_NE(attach, detach);
  EXPECT_NE(attach, "unknown");
  EXPECT_NE(detach, "unknown");
}

TEST(AttachmentSubmitResult, OnlyAcceptedConvertsToTrue)
{
  EXPECT_TRUE(static_cast<bool>(AttachmentSubmitResult{AttachmentSubmitStatus::kAccepted, {}}));
  EXPECT_FALSE(
    static_cast<bool>(AttachmentSubmitResult{AttachmentSubmitStatus::kBusy, "outstanding"}));
  EXPECT_FALSE(
    static_cast<bool>(AttachmentSubmitResult{AttachmentSubmitStatus::kInvalidRequest, "bad"}));
  EXPECT_FALSE(
    static_cast<bool>(AttachmentSubmitResult{AttachmentSubmitStatus::kUnavailable, "down"}));
}

TEST(AttachmentCompletion, DefaultsToTheFailClosedOutcome)
{
  // A completion nobody filled in must never read as a clean or successful transaction.
  const AttachmentCompletion completion;
  EXPECT_EQ(completion.outcome, AttachmentOutcome::kIndeterminate);
  EXPECT_TRUE(attachment_requires_operator(completion.outcome));
  EXPECT_EQ(completion.world_revision, 0U);
}

TEST(FakeAttachmentPort, AcceptsOneTransactionAtATimeAndCompletesItExactlyOnce)
{
  FakeAttachmentPort port;
  std::optional<AttachmentCompletion> observed;
  std::size_t completions = 0U;

  const OperationCorrelation correlation{4U, 9U};
  ASSERT_TRUE(
    static_cast<bool>(
      port.submit(
        correlation, usable_attach_goal(),
        [&](AttachmentCompletion completion) {
          ++completions;
          observed = std::move(completion);
        })));
  EXPECT_TRUE(port.outstanding());
  EXPECT_EQ(port.submission().correlation.goal_generation, correlation.goal_generation);

  // A second transaction would contend for the single planning-scene lease.
  const auto second = port.submit(
    correlation, usable_attach_goal(), [](AttachmentCompletion) {});
  EXPECT_EQ(second.status, AttachmentSubmitStatus::kBusy);
  EXPECT_EQ(completions, 0U);

  port.complete(AttachmentOutcome::kSucceeded, "committed", 42U);
  ASSERT_TRUE(observed.has_value());
  EXPECT_EQ(completions, 1U);
  EXPECT_EQ(observed->outcome, AttachmentOutcome::kSucceeded);
  EXPECT_EQ(observed->world_revision, 42U);
  EXPECT_EQ(observed->correlation.operation_generation, correlation.operation_generation);
  EXPECT_FALSE(port.outstanding());
}

TEST(FakeAttachmentPort, MirrorsThePortsLeaseRuleForDivergentOutcomes)
{
  const auto complete_once = [](AttachmentOutcome outcome) {
    FakeAttachmentPort port;
    std::optional<AttachmentCompletion> observed;
    static_cast<void>(
      port.submit(
        {1U, 1U}, usable_attach_goal(),
        [&](AttachmentCompletion completion) {observed = std::move(completion);}));
    port.complete(outcome);
    return observed.value();
  };

  // The real port hands the lease back only when both worlds are known to agree; a fake that
  // released it unconditionally would let a driver test pass against behaviour that cannot happen.
  EXPECT_TRUE(complete_once(AttachmentOutcome::kSucceeded).planning_scene_lease_released);
  EXPECT_TRUE(complete_once(AttachmentOutcome::kPhysicalFailed).planning_scene_lease_released);
  EXPECT_FALSE(complete_once(AttachmentOutcome::kUncommitted).planning_scene_lease_released);
  EXPECT_FALSE(complete_once(AttachmentOutcome::kIndeterminate).planning_scene_lease_released);
}

TEST(FakeAttachmentPort, RefusesOnDemandAndRecordsCancellation)
{
  FakeAttachmentPort port;
  port.refuse_next(AttachmentSubmitStatus::kUnavailable, "boundary down");
  const auto refused = port.submit(
    {1U, 1U}, usable_attach_goal(), [](AttachmentCompletion) {});
  EXPECT_FALSE(static_cast<bool>(refused));
  EXPECT_EQ(refused.status, AttachmentSubmitStatus::kUnavailable);
  EXPECT_FALSE(port.outstanding());

  // The refusal is consumed, not sticky.
  EXPECT_TRUE(
    static_cast<bool>(
      port.submit({1U, 2U}, usable_attach_goal(), [](AttachmentCompletion) {})));
  EXPECT_EQ(port.submit_attempts, 2U);

  port.cancel();
  EXPECT_EQ(port.cancel_requests, 1U);
  // Cancellation never resolves the transaction on its own; a completion still has to arrive.
  EXPECT_TRUE(port.outstanding());

  port.set_ready(false);
  EXPECT_FALSE(port.ready());
}

}  // namespace
}  // namespace restocker_task_executor
