// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "restocker_task_executor/motion_port.hpp"

namespace restocker_task_executor
{
namespace
{

[[nodiscard]] MotionGoal usable_goal()
{
  MotionGoal goal;
  goal.planning_frame_from_tool0 = Eigen::Isometry3d::Identity();
  goal.planning_frame_from_tool0.translation() = Eigen::Vector3d{0.4, -0.2, 0.9};
  goal.planning_frame_from_tool0.linear() =
    Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  return goal;
}

TEST(MotionGoal, AcceptsAFiniteOrthonormalPoseWithUsableLimits)
{
  EXPECT_TRUE(valid_motion_goal(usable_goal()));
}

TEST(MotionPort, NormalizesOnlyNumericalExcursionsPastPlanningJointBounds)
{
  const auto below = normalize_bounded_planning_position(-2.5800001, -2.58, 2.58, 0.001);
  ASSERT_TRUE(below);
  EXPECT_DOUBLE_EQ(below->value, -2.58);
  EXPECT_TRUE(below->corrected);

  const auto inside = normalize_bounded_planning_position(0.4, -2.58, 2.58, 0.001);
  ASSERT_TRUE(inside);
  EXPECT_DOUBLE_EQ(inside->value, 0.4);
  EXPECT_FALSE(inside->corrected);

  EXPECT_FALSE(normalize_bounded_planning_position(-2.582, -2.58, 2.58, 0.001));
  EXPECT_FALSE(
    normalize_bounded_planning_position(
      std::numeric_limits<double>::quiet_NaN(), -2.58, 2.58, 0.001));
}

TEST(MotionGoal, RejectsNonFinitePoseComponents)
{
  auto goal = usable_goal();
  goal.planning_frame_from_tool0.translation().x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.planning_frame_from_tool0.translation().z() =
    std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionGoal, RejectsANonOrthonormalRotation)
{
  auto goal = usable_goal();
  // MoveIt would normalize a scaled rotation, so the contract must refuse it first.
  goal.planning_frame_from_tool0.linear() *= 1.5;
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionGoal, RejectsUnusableTolerancesAndScaling)
{
  auto goal = usable_goal();
  goal.position_tolerance_m = 0.0;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.orientation_tolerance_rad = -0.1;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.velocity_scaling = 0.0;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.velocity_scaling = 1.5;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.acceleration_scaling = 1.5;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.planning_time = std::chrono::milliseconds{0};
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.maximum_controller_goal_duration = std::chrono::milliseconds{0};
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.free_space_plan_candidates = 0U;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.free_space_plan_candidates = 9U;
  EXPECT_FALSE(valid_motion_goal(goal));
}

// Card 074 (Milestone 10 §6): the split of a planning floor is not itself a budget. Every
// candidate keeps a first-pass slice (a measured green solve is 1.8–5.2 s and must never wait
// behind a hard one), the reserved remainder goes to the preferred candidate's single second
// attempt, and that candidate's aggregate share inside the round is exactly half the floor —
// the even split handed the preferred continuation endpoint only floor/N seconds and Card 066's
// dense dev run 3 refused five rounds that way (21 × ~15 s timeouts, then the same endpoint
// planned in 11.1 s).
TEST(MotionPort, FreeSpaceSlicePlanKeepsFirstPassesAndGivesThePreferredHalfTheFloor)
{
  const auto plan = plan_free_space_slices(60.0, 4U);
  EXPECT_DOUBLE_EQ(plan.first_pass_slice_seconds, 10.0);
  EXPECT_DOUBLE_EQ(plan.second_pass_slice_seconds, 20.0);
  EXPECT_GE(plan.first_pass_slice_seconds, 5.2) << "the measured green tail must fit pass one";
  // The round's spend never exceeds the floor, and the preferred aggregate is half of it.
  EXPECT_DOUBLE_EQ(
    plan.first_pass_slice_seconds * 4.0 + plan.second_pass_slice_seconds, 60.0);
  EXPECT_DOUBLE_EQ(plan.first_pass_slice_seconds + plan.second_pass_slice_seconds, 30.0);

  for (std::size_t candidates = 1U; candidates <= 8U; ++candidates) {
    const auto slices = plan_free_space_slices(60.0, candidates);
    EXPECT_GT(slices.first_pass_slice_seconds, 0.0) << candidates;
    EXPECT_LE(
      slices.first_pass_slice_seconds * static_cast<double>(candidates) +
      slices.second_pass_slice_seconds,
      60.0 + 1.0e-9) << candidates;
    EXPECT_GE(
      slices.first_pass_slice_seconds + slices.second_pass_slice_seconds, 30.0 - 1.0e-9)
      << "the preferred candidate holds at least half the floor: " << candidates;
  }

  // One candidate is one attempt over the whole floor: a split would restart the search the
  // floor was about to spend.
  const auto single = plan_free_space_slices(60.0, 1U);
  EXPECT_DOUBLE_EQ(single.first_pass_slice_seconds, 60.0);
  EXPECT_DOUBLE_EQ(single.second_pass_slice_seconds, 0.0);

  // Degenerate inputs fail closed to an all-zero plan the caller refuses before planning.
  const auto empty = plan_free_space_slices(0.0, 4U);
  EXPECT_DOUBLE_EQ(empty.first_pass_slice_seconds, 0.0);
  EXPECT_DOUBLE_EQ(empty.second_pass_slice_seconds, 0.0);
  const auto no_candidates = plan_free_space_slices(60.0, 0U);
  EXPECT_DOUBLE_EQ(no_candidates.first_pass_slice_seconds, 0.0);
  EXPECT_DOUBLE_EQ(no_candidates.second_pass_slice_seconds, 0.0);
}

namespace
{
// Card 074 review note 1: the round that spends a floor, driven by a scripted planner instead
// of MoveIt. Every attempt consumes exactly the slice it was granted (what a planner that runs
// into its allowed time does) plus an optional per-attempt overhead for the work the real loop
// does outside the slice, and answers from a per-candidate queue of verdicts — the reserved
// second pass re-attempts a candidate, so it dequeues that candidate's next verdict.
class ScriptedFloor
{
public:
  ScriptedFloor(double floor_seconds, std::size_t candidates)
  : floor_seconds_(floor_seconds),
    candidates_(candidates),
    plan_(plan_free_space_slices(floor_seconds, candidates))
  {
  }

  ScriptedFloor & answers(std::size_t candidate, std::vector<FreeSpaceSliceAttempt> verdicts)
  {
    verdicts_[candidate] = std::move(verdicts);
    return *this;
  }

  ScriptedFloor & overhead(double seconds)
  {
    overhead_seconds_ = seconds;
    return *this;
  }

  [[nodiscard]] const std::vector<std::pair<std::size_t, double>> & attempts() const
  {
    return attempts_;
  }

  [[nodiscard]] const FreeSpaceSlicePlan & plan() const {return plan_;}

  FreeSpaceSliceRoundResult run(bool stop_after_first_accept)
  {
    attempts_.clear();
    spent_seconds_ = 0.0;
    return run_free_space_slice_round(
      plan_, candidates_, stop_after_first_accept,
      [this]() {return remaining();},
      [this](std::size_t candidate, double slice_seconds) {
        attempts_.emplace_back(candidate, slice_seconds);
        spent_seconds_ += slice_seconds + overhead_seconds_;
        const auto found = verdicts_.find(candidate);
        EXPECT_NE(found, verdicts_.end()) << "candidate " << candidate << " was not scripted";
        if (found == verdicts_.end() || found->second.empty()) {
          ADD_FAILURE() << "candidate " << candidate << " has no verdict left";
          return FreeSpaceSliceAttempt{};
        }
        const auto verdict = found->second.front();
        found->second.erase(found->second.begin());
        return verdict;
      });
  }

private:
  [[nodiscard]] std::chrono::steady_clock::duration remaining() const
  {
    return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(floor_seconds_ - spent_seconds_));
  }

  double floor_seconds_;
  std::size_t candidates_;
  FreeSpaceSlicePlan plan_;
  std::map<std::size_t, std::vector<FreeSpaceSliceAttempt>> verdicts_;
  std::vector<std::pair<std::size_t, double>> attempts_;
  double overhead_seconds_{0.0};
  double spent_seconds_{0.0};
};

constexpr FreeSpaceSliceAttempt kTimedOut{false, true, false, false};
constexpr FreeSpaceSliceAttempt kAccepted{true, false, false, false};
// A refusal that is not a deadline: a post-plan rejection, the scene's own verdict.
constexpr FreeSpaceSliceAttempt kRefused{false, false, false, false};
constexpr FreeSpaceSliceAttempt kCanceledAttempt{false, false, true, false};
constexpr FreeSpaceSliceAttempt kEndpointRejectedAttempt{false, false, false, true};
}  // namespace

// Card 074 (Milestone 10 §6) review note 1: pin the LOOP, not just plan_free_space_slices.
// The reserved remainder must be spent once, by the first candidate that timed out, against the
// floor that is left — the seam that stops an even split from handing the preferred endpoint
// only floor/N seconds (Card 066's dense dev run 3 refused five rounds that way and then
// planned the same endpoint in 11.1 s).
TEST(MotionPort, FreeSpaceSliceRoundSpendsTheReservedSecondPassOnTheFirstTimedOutCandidate)
{
  ScriptedFloor floor(60.0, 4U);
  floor.answers(0U, {kTimedOut, kAccepted})
  .answers(1U, {kTimedOut})
  .answers(2U, {kTimedOut})
  .answers(3U, {kTimedOut});
  EXPECT_DOUBLE_EQ(floor.plan().first_pass_slice_seconds, 10.0);
  EXPECT_DOUBLE_EQ(floor.plan().second_pass_slice_seconds, 20.0);

  const auto round = floor.run(false);

  ASSERT_EQ(round.status, FreeSpaceSliceRoundStatus::kCompleted);
  EXPECT_EQ(round.second_pass_candidate, 0U);
  EXPECT_TRUE(round.second_pass_ran);
  EXPECT_DOUBLE_EQ(round.second_pass_slice_seconds, 20.0);
  EXPECT_EQ(round.accepted_candidates, 1U);
  const auto & attempts = floor.attempts();
  ASSERT_EQ(attempts.size(), 5U) << "four first passes and exactly one second pass";
  EXPECT_EQ(attempts.front().first, 0U);
  EXPECT_DOUBLE_EQ(attempts.front().second, 10.0) << "pass one is the divided first-pass slice";
  for (std::size_t index = 1U; index < 4U; ++index) {
    EXPECT_EQ(attempts[index].first, index) << index;
    EXPECT_DOUBLE_EQ(attempts[index].second, 10.0) << index;
  }
  EXPECT_EQ(attempts.back().first, 0U) << "the first candidate that timed out gets the remainder";
  EXPECT_DOUBLE_EQ(attempts.back().second, 20.0) << "the whole remaining floor, not floor/N";
}

// The second pass exists to rescue a timeout, so an accepted candidate suppresses it — and the
// remaining candidates still get their own first passes (the goal keeps searching for the
// shortest of them).
TEST(MotionPort, FreeSpaceSliceRoundWithholdsTheSecondPassOnceACandidatePlanned)
{
  ScriptedFloor floor(60.0, 4U);
  floor.answers(0U, {kTimedOut})
  .answers(1U, {kAccepted})
  .answers(2U, {kRefused})
  .answers(3U, {kRefused});

  const auto round = floor.run(false);

  ASSERT_EQ(round.status, FreeSpaceSliceRoundStatus::kCompleted);
  EXPECT_EQ(round.accepted_candidates, 1U);
  EXPECT_EQ(round.second_pass_candidate, 0U) << "a candidate did time out…";
  EXPECT_FALSE(round.second_pass_ran) << "…but nothing may be re-attempted after a plan exists";
  EXPECT_EQ(floor.attempts().size(), 4U);
}

// With endpoint continuation the round stops at the first planned endpoint (the rest are
// fallbacks for a disconnected IK branch), so it never reaches a second pass either.
TEST(MotionPort, FreeSpaceSliceRoundStopsAtTheFirstPlannedEndpoint)
{
  ScriptedFloor floor(60.0, 4U);
  floor.answers(0U, {kTimedOut}).answers(1U, {kAccepted});

  const auto round = floor.run(true);

  EXPECT_EQ(round.status, FreeSpaceSliceRoundStatus::kStoppedOnAccept);
  EXPECT_EQ(round.accepted_candidates, 1U);
  EXPECT_FALSE(round.second_pass_ran);
  ASSERT_EQ(floor.attempts().size(), 2U);
  EXPECT_EQ(floor.attempts().back().first, 1U);
}

// The remainder goes to the FIRST candidate that timed out, not to candidate 0 by position: a
// refusal that is not a deadline (post-plan rejection) does not claim it.
TEST(MotionPort, FreeSpaceSliceRoundNamesTheFirstTimedOutCandidateAsItsSecondPass)
{
  ScriptedFloor floor(60.0, 4U);
  floor.answers(0U, {kRefused})
  .answers(1U, {kTimedOut, kAccepted})
  .answers(2U, {kTimedOut})
  .answers(3U, {kTimedOut});

  const auto round = floor.run(false);

  EXPECT_EQ(round.second_pass_candidate, 1U);
  ASSERT_TRUE(round.second_pass_ran);
  const auto & attempts = floor.attempts();
  ASSERT_EQ(attempts.size(), 5U);
  EXPECT_EQ(attempts.back().first, 1U);
  EXPECT_DOUBLE_EQ(attempts.back().second, 20.0);
}

// The second pass is bounded by the same floor: once the four first passes (and the work they
// do outside their slices) have spent it, nothing is re-attempted and the round refuses.
TEST(MotionPort, FreeSpaceSliceRoundWithholdsTheSecondPassWhenTheFloorIsAlreadySpent)
{
  ScriptedFloor floor(60.0, 4U);
  floor.overhead(5.0)
  .answers(0U, {kTimedOut})
  .answers(1U, {kTimedOut})
  .answers(2U, {kTimedOut})
  .answers(3U, {kTimedOut});

  const auto round = floor.run(false);

  EXPECT_EQ(round.status, FreeSpaceSliceRoundStatus::kCompleted);
  EXPECT_EQ(round.accepted_candidates, 0U);
  EXPECT_EQ(round.second_pass_candidate, 0U) << "a candidate did time out";
  EXPECT_FALSE(round.second_pass_ran) << "the floor is gone";
  EXPECT_EQ(floor.attempts().size(), 4U);
}

// Terminal verdicts inside the round are not swallowed by the loop: the caller still reports
// cancellation and the continuation-endpoint refusal, in either pass.
TEST(MotionPort, FreeSpaceSliceRoundPropagatesCancellationAndEndpointRejection)
{
  {
    ScriptedFloor floor(60.0, 4U);
    floor.answers(0U, {kCanceledAttempt});
    const auto round = floor.run(false);
    EXPECT_EQ(round.status, FreeSpaceSliceRoundStatus::kCanceled);
    EXPECT_FALSE(round.second_pass_ran);
    EXPECT_EQ(floor.attempts().size(), 1U);
  }
  {
    ScriptedFloor floor(60.0, 4U);
    floor.answers(0U, {kEndpointRejectedAttempt});
    const auto round = floor.run(false);
    EXPECT_EQ(round.status, FreeSpaceSliceRoundStatus::kEndpointRejected);
    EXPECT_EQ(floor.attempts().size(), 1U);
  }
  {
    // The cancel arrives during the reserved second attempt itself.
    ScriptedFloor floor(60.0, 4U);
    floor.answers(0U, {kTimedOut, kCanceledAttempt})
    .answers(1U, {kTimedOut})
    .answers(2U, {kTimedOut})
    .answers(3U, {kTimedOut});
    const auto round = floor.run(false);
    EXPECT_EQ(round.status, FreeSpaceSliceRoundStatus::kCanceled);
    EXPECT_TRUE(round.second_pass_ran);
    EXPECT_EQ(floor.attempts().size(), 5U);
  }
}

TEST(MotionOutcome, OnlyPlanningFailureAndUnavailabilityProveNoMotionStarted)
{
  EXPECT_TRUE(motion_definitely_not_started(MotionOutcome::kPlanningFailed));
  EXPECT_TRUE(motion_definitely_not_started(MotionOutcome::kUnavailable));

  // Everything else may have commanded joints, so it must not be treated as pre-motion.
  EXPECT_FALSE(motion_definitely_not_started(MotionOutcome::kSucceeded));
  EXPECT_FALSE(motion_definitely_not_started(MotionOutcome::kExecutionFailed));
  EXPECT_FALSE(motion_definitely_not_started(MotionOutcome::kCanceled));
  EXPECT_FALSE(motion_definitely_not_started(MotionOutcome::kTimedOut));
}

TEST(MotionOutcome, EveryOutcomeHasADistinctName)
{
  const MotionOutcome outcomes[]{
    MotionOutcome::kSucceeded, MotionOutcome::kPlanningFailed, MotionOutcome::kExecutionFailed,
    MotionOutcome::kCanceled, MotionOutcome::kTimedOut, MotionOutcome::kUnavailable};
  for (const auto outcome : outcomes) {
    const std::string name = motion_outcome_name(outcome);
    EXPECT_FALSE(name.empty());
    EXPECT_NE(name, "unknown");
  }
}

TEST(MotionSubmitResult, OnlyAcceptedConvertsToTrue)
{
  EXPECT_TRUE(static_cast<bool>(MotionSubmitResult{MotionSubmitStatus::kAccepted, {}}));
  EXPECT_FALSE(
    static_cast<bool>(MotionSubmitResult{MotionSubmitStatus::kBusy, "outstanding"}));
  EXPECT_FALSE(
    static_cast<bool>(MotionSubmitResult{MotionSubmitStatus::kInvalidRequest, "bad"}));
  EXPECT_FALSE(
    static_cast<bool>(MotionSubmitResult{MotionSubmitStatus::kUnavailable, "down"}));
}


TEST(MotionPort, RejectsAnUnusableLinearPathBudgetOrUnnamedSegment)
{
  auto goal = usable_goal();
  EXPECT_EQ(goal.path, MotionPathKind::kFreeSpace);
  goal.path = MotionPathKind::kLinear;
  EXPECT_TRUE(valid_motion_goal(goal));

  auto no_step = goal;
  no_step.cartesian_step_m = 0.0;
  EXPECT_FALSE(valid_motion_goal(no_step));

  auto whole_path_optional = goal;
  whole_path_optional.minimum_cartesian_fraction = 0.0;
  EXPECT_FALSE(valid_motion_goal(whole_path_optional));

  auto more_than_whole = goal;
  more_than_whole.minimum_cartesian_fraction = 1.5;
  EXPECT_FALSE(valid_motion_goal(more_than_whole));

  // Every failure detail names the segment, so an unnamed goal would report the wrong motion.
  auto unnamed = goal;
  unnamed.label.clear();
  EXPECT_FALSE(valid_motion_goal(unnamed));
}

TEST(MotionPort, AcceptsFreeSpaceGoalsWithAValidPathBoxAndRejectsLinearPlusConstraints)
{
  auto goal = usable_goal();
  MotionPathConstraints constraints;
  constraints.position_box = MotionPathPositionBox{
    Eigen::Vector3d{-0.5, -1.5, 0.0}, Eigen::Vector3d{0.5, 0.5, 0.4}};
  constraints.description = "test box";
  goal.path_constraints = constraints;
  EXPECT_TRUE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kLinear;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kFreeSpace;
  goal.path_constraints->position_box->max_corner_m =
    goal.path_constraints->position_box->min_corner_m;
  EXPECT_FALSE(valid_motion_goal(goal));

  // Empty constraint payload must not pass as "present but unconstrained".
  MotionPathConstraints empty;
  goal.path_constraints = empty;
  EXPECT_FALSE(valid_motion_path_constraints(empty));
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionPort, PositionBoxCheckIsInclusiveButAddsNoHiddenTolerance)
{
  MotionPathPositionBox box;
  box.min_corner_m = Eigen::Vector3d{-1.039750, -0.5, 0.6};
  box.max_corner_m = Eigen::Vector3d{-0.38, 0.42, 0.99};

  EXPECT_TRUE(motion_path_position_satisfies_box(box.min_corner_m, box));
  EXPECT_TRUE(motion_path_position_satisfies_box(box.max_corner_m, box));
  // The constrained-state planner can produce a face-hugging path (X=-1.039795). The port must
  // reject that trajectory before execution instead of adding a second corridor expansion.
  EXPECT_FALSE(
    motion_path_position_satisfies_box(
      Eigen::Vector3d{-1.039795, -0.036605, 0.869006}, box));
  EXPECT_FALSE(
    motion_path_position_satisfies_box(
      Eigen::Vector3d{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.8}, box));
}

TEST(MotionGoal, RejectsAnInvalidUprightPostPlanBudget)
{
  auto goal = usable_goal();
  goal.maximum_tool0_x_axis_tilt_rad = 0.03;
  EXPECT_TRUE(valid_motion_goal(goal));
  goal.maximum_tool0_x_axis_tilt_rad = 0.0;
  EXPECT_FALSE(valid_motion_goal(goal));
  goal.maximum_tool0_x_axis_tilt_rad = 0.5 * std::acos(-1.0);
  EXPECT_FALSE(valid_motion_goal(goal));
  goal.maximum_tool0_x_axis_tilt_rad = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionPort, PlanningInsetLeavesOuterValidationMarginAndFailsClosed)
{
  MotionPathPositionBox outer;
  outer.min_corner_m = Eigen::Vector3d{-1.18, -0.54, 0.60};
  outer.max_corner_m = Eigen::Vector3d{-0.23, 0.56, 0.99};
  const auto inner = inset_motion_path_position_box(outer, 0.005);
  ASSERT_TRUE(inner);
  EXPECT_TRUE(
    inner->min_corner_m.isApprox((outer.min_corner_m.array() + 0.005).matrix(), 1.0e-12));
  EXPECT_TRUE(
    inner->max_corner_m.isApprox((outer.max_corner_m.array() - 0.005).matrix(), 1.0e-12));
  EXPECT_TRUE(motion_path_position_satisfies_box(inner->min_corner_m, outer));
  EXPECT_TRUE(motion_path_position_satisfies_box(inner->max_corner_m, outer));

  EXPECT_FALSE(inset_motion_path_position_box(outer, 0.0));
  EXPECT_FALSE(inset_motion_path_position_box(outer, -0.001));
  EXPECT_FALSE(inset_motion_path_position_box(outer, std::numeric_limits<double>::quiet_NaN()));
  MotionPathPositionBox narrow = outer;
  narrow.max_corner_m.z() = narrow.min_corner_m.z() + 0.009;
  EXPECT_FALSE(inset_motion_path_position_box(narrow, 0.005));
}

TEST(MotionPort, CompoundRetreatRequiresAValidLinearEgressAndFreeSpaceFinalLeg)
{
  auto goal = usable_goal();
  goal.linear_egress_pose = Eigen::Isometry3d::Identity();
  goal.linear_egress_pose->translation() = Eigen::Vector3d{0.4, -0.1, 0.9};
  EXPECT_TRUE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kLinear;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kFreeSpace;
  goal.linear_egress_pose->translation().y() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionPort, RequiredLinearContinuationNeedsAValidPoseAndFreeSpacePlan)
{
  auto goal = usable_goal();
  goal.required_linear_continuation_pose = Eigen::Isometry3d::Identity();
  goal.required_linear_continuation_gripper_joint_position_m = 0.02;
  goal.required_linear_continuation_vertical_margin_m = 0.005;
  goal.required_linear_continuation_pose->translation() = Eigen::Vector3d{0.4, -0.3, 0.9};
  EXPECT_TRUE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kLinear;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kFreeSpace;
  goal.required_linear_continuation_pose->translation().x() =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.required_linear_continuation_pose = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.required_linear_continuation_gripper_joint_position_m = -0.01;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.required_linear_continuation_vertical_margin_m = 0.005;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.required_linear_continuation_vertical_margin_m =
    std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionPort, RequiredPostMotionEgressNeedsAValidPoseJawTargetAndLinearPlan)
{
  auto goal = usable_goal();
  goal.path = MotionPathKind::kLinear;
  goal.required_postmotion_linear_egress_pose = Eigen::Isometry3d::Identity();
  goal.required_postmotion_linear_egress_pose->translation() =
    Eigen::Vector3d{0.4, -0.3, 0.9};
  goal.required_postmotion_linear_egress_gripper_joint_position_m = 0.032;
  EXPECT_TRUE(valid_motion_goal(goal));

  goal.path = MotionPathKind::kFreeSpace;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.path = MotionPathKind::kLinear;
  goal.required_postmotion_linear_egress_pose = Eigen::Isometry3d::Identity();
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.required_postmotion_linear_egress_gripper_joint_position_m = -0.001;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.required_postmotion_linear_egress_pose->translation().z() =
    std::numeric_limits<double>::quiet_NaN();
  goal.required_postmotion_linear_egress_gripper_joint_position_m = 0.032;
  EXPECT_FALSE(valid_motion_goal(goal));
}

TEST(MotionPort, RequiredPostContinuationChainIsCompleteAndFollowsAnApproach)
{
  auto goal = usable_goal();
  goal.required_linear_continuation_pose = Eigen::Isometry3d::Identity();
  goal.required_linear_continuation_gripper_joint_position_m = 0.02;
  goal.required_postcontinuation_linear_retract_pose = Eigen::Isometry3d::Identity();
  goal.required_postcontinuation_linear_retract_pose->translation().y() = -0.1;
  goal.required_postcontinuation_linear_egress_pose = Eigen::Isometry3d::Identity();
  goal.required_postcontinuation_linear_egress_pose->translation().y() = -0.02;
  goal.required_postcontinuation_gripper_joint_position_m = 0.015;
  EXPECT_TRUE(valid_motion_goal(goal));

  goal.required_postcontinuation_linear_egress_pose.reset();
  EXPECT_FALSE(valid_motion_goal(goal));

  goal = usable_goal();
  goal.required_postcontinuation_linear_retract_pose = Eigen::Isometry3d::Identity();
  goal.required_postcontinuation_linear_egress_pose = Eigen::Isometry3d::Identity();
  goal.required_postcontinuation_gripper_joint_position_m = 0.015;
  EXPECT_FALSE(valid_motion_goal(goal));

  goal.required_linear_continuation_pose = Eigen::Isometry3d::Identity();
  goal.required_linear_continuation_gripper_joint_position_m = 0.02;
  goal.required_postcontinuation_gripper_joint_position_m = -0.001;
  EXPECT_FALSE(valid_motion_goal(goal));
}

// Milestone 10 §6 (Card 062): the grasp escape tolerates exactly the finger–target contacts.
GraspEscape card_062_escape()
{
  GraspEscape escape;
  escape.target_object_id = "restocker/object/1";
  escape.tolerated_links = {"left_finger", "right_finger"};
  return escape;
}

TEST(GraspEscape, AdmitsOnlyFingerContactsWithTheTargetProduct)
{
  const auto escape = card_062_escape();
  // Card 010 SC-004 slot 16's receipt, reported in MoveIt's order and reversed.
  const auto scope = classify_grasp_escape_contacts(
    {{"left_finger", "restocker/object/1"}, {"restocker/object/1", "left_finger"},
      {"restocker/object/1", "right_finger"}}, escape);
  ASSERT_TRUE(scope.admitted) << scope.refusal;
  ASSERT_EQ(scope.tolerated_pairs.size(), 2U);
  EXPECT_EQ(scope.tolerated_pairs[0], (CollisionContactPair{"restocker/object/1", "left_finger"}));
  EXPECT_EQ(
    scope.tolerated_pairs[1], (CollisionContactPair{"restocker/object/1", "right_finger"}));
}

TEST(GraspEscape, AContactFreeStartToleratesNothing)
{
  const auto scope = classify_grasp_escape_contacts({}, card_062_escape());
  EXPECT_TRUE(scope.admitted);
  EXPECT_TRUE(scope.tolerated_pairs.empty());
}

TEST(GraspEscape, RefusesAnyOtherObjectOrLink)
{
  const auto escape = card_062_escape();
  // A neighbouring product, the target against the palm, the target against the arm, a fixture,
  // and two products touching each other.
  for (const auto & offending : std::vector<CollisionContactPair>{
        {"left_finger", "restocker/object/2"},
        {"restocker/object/1", "gripper_body"},
        {"wrist_3_link", "restocker/object/1"},
        {"left_finger", "tray_1"},
        {"restocker/object/2", "restocker/object/1"}})
  {
    const auto scope = classify_grasp_escape_contacts(
      {{"restocker/object/1", "left_finger"}, offending}, escape);
    EXPECT_FALSE(scope.admitted) << offending.first << " / " << offending.second;
    EXPECT_TRUE(scope.tolerated_pairs.empty());
    EXPECT_NE(scope.refusal.find(offending.first), std::string::npos);
  }
}

TEST(GraspEscape, RefusesAnEscapeWithoutATargetOrLinks)
{
  auto no_target = card_062_escape();
  no_target.target_object_id.clear();
  EXPECT_FALSE(classify_grasp_escape_contacts({}, no_target).admitted);
  EXPECT_FALSE(valid_grasp_escape(no_target));
  auto no_links = card_062_escape();
  no_links.tolerated_links.clear();
  EXPECT_FALSE(classify_grasp_escape_contacts({}, no_links).admitted);
  EXPECT_FALSE(valid_grasp_escape(no_links));
  EXPECT_TRUE(valid_grasp_escape(card_062_escape()));
}

TEST(GraspEscape, GoalValidationBindsTheEscapeLegToALinearPath)
{
  auto goal = usable_goal();
  goal.grasp_escape = card_062_escape();
  EXPECT_TRUE(valid_motion_goal(goal));
  goal.grasp_escape_leg = true;
  goal.path = MotionPathKind::kFreeSpace;
  EXPECT_FALSE(valid_motion_goal(goal));
  goal.path = MotionPathKind::kLinear;
  EXPECT_TRUE(valid_motion_goal(goal));
  goal.grasp_escape.reset();
  EXPECT_FALSE(valid_motion_goal(goal));
  goal.grasp_escape = card_062_escape();
  goal.grasp_escape->tolerated_links = {""};
  EXPECT_FALSE(valid_motion_goal(goal));
}

}  // namespace
}  // namespace restocker_task_executor
