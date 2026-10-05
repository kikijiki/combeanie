// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include "restocker_task_executor/planning_contract.hpp"

namespace restocker_task_executor
{
namespace
{

const std::vector<std::string> kManipulatorJoints{
  "rail_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
  "wrist_2_joint", "wrist_3_joint"};

TEST(PlanningContract, AcceptsExactModelContract) {
  const auto result = validate_model_contract("world", "tool0", kManipulatorJoints);
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.exit_code(), 0);
}

TEST(PlanningContract, RejectsFrameToolAndJointOrderDrift) {
  EXPECT_EQ(
    validate_model_contract("rail_base", "tool0", kManipulatorJoints).status,
    PlanningStatus::kModelContractMismatch);
  EXPECT_EQ(
    validate_model_contract("world", "gripper", kManipulatorJoints).status,
    PlanningStatus::kModelContractMismatch);
  auto reordered = kManipulatorJoints;
  std::swap(reordered[0], reordered[1]);
  EXPECT_EQ(
    validate_model_contract("world", "tool0", reordered).status,
    PlanningStatus::kModelContractMismatch);
}

TEST(PlanningContract, DistinguishesPlanAndExecutionFailures) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  EXPECT_EQ(
    classify_moveit_error(PlanningPhase::kPlan, ErrorCodes::PLANNING_FAILED, "joint goal")
    .status,
    PlanningStatus::kPlanningRejected);
  EXPECT_EQ(
    classify_moveit_error(PlanningPhase::kPlan, ErrorCodes::TIMED_OUT, "joint goal").status,
    PlanningStatus::kPlanningTimedOut);
  EXPECT_EQ(
    classify_moveit_error(PlanningPhase::kExecute, ErrorCodes::CONTROL_FAILED, "joint goal")
    .status,
    PlanningStatus::kExecutionRejected);
  EXPECT_EQ(
    classify_moveit_error(PlanningPhase::kExecute, ErrorCodes::TIMED_OUT, "joint goal")
    .status,
    PlanningStatus::kExecutionTimedOut);
  EXPECT_EQ(
    classify_moveit_error(PlanningPhase::kPlan, ErrorCodes::COMMUNICATION_FAILURE, "goal")
    .status,
    PlanningStatus::kServerUnavailable);
}

// Card 086 stage 1a: the segment is only known to be over at the backend when a controller said
// so. A MoveIt TIMED_OUT must never be promoted to a terminal stop; the campaign's settlement
// rules read the flag this feeds.
TEST(PlanningContract, OnlyControlFailedProvesATerminalStop) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  EXPECT_TRUE(execution_failure_reached_terminal_stop(ErrorCodes::CONTROL_FAILED));
  EXPECT_FALSE(execution_failure_reached_terminal_stop(ErrorCodes::TIMED_OUT));
  for (const int code :
    {static_cast<int>(ErrorCodes::FAILURE), static_cast<int>(ErrorCodes::PREEMPTED),
      static_cast<int>(ErrorCodes::COMMUNICATION_FAILURE), static_cast<int>(ErrorCodes::CRASH),
      static_cast<int>(ErrorCodes::INVALID_ROBOT_STATE), static_cast<int>(ErrorCodes::SUCCESS)})
  {
    EXPECT_FALSE(execution_failure_reached_terminal_stop(code)) << moveit_error_name(code);
  }
}

// TIMED_OUT stays an execution failure (not a planning timeout, not a success): only its
// stop evidence changes.
TEST(PlanningContract, ExecutionTimeoutRemainsAnExecutionFailureWithoutStopEvidence) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  const auto timed_out = classify_moveit_error(
    PlanningPhase::kExecute, ErrorCodes::TIMED_OUT, "joint goal");
  EXPECT_EQ(timed_out.status, PlanningStatus::kExecutionTimedOut);
  EXPECT_FALSE(timed_out.ok());
  EXPECT_FALSE(execution_failure_reached_terminal_stop(ErrorCodes::TIMED_OUT));
}

TEST(PlanningContract, DistinguishesInvalidStartState) {
  const auto result = classify_moveit_error(
    PlanningPhase::kPlan, moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION,
    "joint goal");
  EXPECT_EQ(result.status, PlanningStatus::kStartStateInvalid);
}

// MoveIt's CheckStartStateCollision request adapter aborts the pipeline before the planner runs
// and move_group reports that abort as the generic FAILURE code — the same code a genuine "no
// solution for this pose" carries (SC-003 runs 6/7: refusal 33 ms after the request, all three
// candidates consumed, every one sharing the same start state). When the scene is what refused,
// the classification must say so with its contacts, and must not read as a verdict on the pose,
// or the coordinator spends grasp candidates on a cause every candidate shares.
TEST(PlanningContract, StartStateCollisionUnderAGenericCodeIsNotAVerdictOnThePose) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  condition.in_collision = true;
  condition.contacts = {"gripper against restocker/obstacle/0",
    "left_finger against restocker/obstacle/0"};
  condition.suppressed_contacts = 5U;
  const auto result = classify_plan_failure(
    ErrorCodes::FAILURE, condition, PlanningSlice{},
    "pre-grasp");
  EXPECT_EQ(result.status, PlanningStatus::kStartStateInvalid);
  EXPECT_NE(result.detail.find("pre-grasp: start state is in collision"), std::string::npos)
    << result.detail;
  EXPECT_NE(result.detail.find("gripper against restocker/obstacle/0"), std::string::npos)
    << result.detail;
  EXPECT_NE(result.detail.find("and 5 more"), std::string::npos) << result.detail;
  // The generic sentence must not be what the receipt reads: it is the one that cannot say
  // which of the candidate causes applied.
  EXPECT_EQ(result.detail.find("did not produce a valid solution"), std::string::npos)
    << result.detail;
}

TEST(PlanningContract, AValidStartStateKeepsThePoseVerdictAndSaysItWasRuledOut) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  condition.in_collision = false;
  const auto result = classify_plan_failure(
    ErrorCodes::FAILURE, condition, PlanningSlice{},
    "pre-grasp");
  EXPECT_EQ(result.status, PlanningStatus::kPlanningRejected);
  EXPECT_NE(result.detail.find("did not produce a valid solution"), std::string::npos)
    << result.detail;
  // One receipt, both halves: the pose refused, and the start state ruled out.
  EXPECT_NE(result.detail.find("start state is valid"), std::string::npos) << result.detail;
}

TEST(PlanningContract, AnUnestablishedConditionNeverGuessesACause) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.unestablished = "/check_state_validity did not answer within 2000 ms";
  const auto result = classify_plan_failure(
    ErrorCodes::FAILURE, condition, PlanningSlice{},
    "pre-grasp");
  EXPECT_EQ(result.status, PlanningStatus::kPlanningRejected);
  EXPECT_NE(result.detail.find("start-state condition not established"), std::string::npos)
    << result.detail;
  EXPECT_NE(result.detail.find("/check_state_validity did not answer"), std::string::npos)
    << result.detail;
}

TEST(PlanningContract, OnlyTheGenericRejectionIsRefined) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  condition.in_collision = true;
  condition.contacts = {"gripper against restocker/obstacle/0"};
  // A timeout is already a non-verdict and must stay one: the budget is what ran out, not the
  // scene, and raising a timeout into a start-state claim would say the opposite.
  EXPECT_EQ(
    classify_plan_failure(
      ErrorCodes::TIMED_OUT, condition, PlanningSlice{}, "pre-insert").status,
    PlanningStatus::kPlanningTimedOut);
  // MoveIt naming the start state itself needs no refinement.
  EXPECT_EQ(
    classify_plan_failure(
      ErrorCodes::START_STATE_IN_COLLISION, condition, PlanningSlice{}, "pre-grasp").status,
    PlanningStatus::kStartStateInvalid);
}

// move_action reports the planner's own TIMED_OUT as the generic FAILURE by the time plan()
// answers, so the budget running out is only visible as an attempt that consumed its slice
// (receipt: an OMPL TIMED_OUT two lines above a FAILURE 99999 refusal whose attempt ran
// 15.005 s of a 14.996 s slice). That must classify as a timeout — never as a verdict on the
// goal, which would spend a grasp candidate on a pose a retry plans moments later.
TEST(PlanningContract, AnAttemptThatConsumedItsSliceIsATimeoutNotAVerdict) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  const PlanningSlice slice{15.005, 14.996};
  ASSERT_TRUE(slice.exhausted());
  const auto result = classify_plan_failure(ErrorCodes::FAILURE, condition, slice, "pre-insert");
  EXPECT_EQ(result.status, PlanningStatus::kPlanningTimedOut);
  EXPECT_NE(result.detail.find("consumed its slice"), std::string::npos) << result.detail;
  EXPECT_NE(result.detail.find("15.005"), std::string::npos) << result.detail;
  EXPECT_NE(result.detail.find("start state is valid"), std::string::npos) << result.detail;
}

TEST(PlanningContract, AnInstantFailureWithAValidStartStateStaysAPoseVerdict) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  const PlanningSlice slice{0.004, 15.0};
  ASSERT_FALSE(slice.exhausted());
  const auto result = classify_plan_failure(ErrorCodes::FAILURE, condition, slice, "pre-grasp");
  EXPECT_EQ(result.status, PlanningStatus::kPlanningRejected);
  EXPECT_NE(result.detail.find("did not produce a valid solution"), std::string::npos)
    << result.detail;
}

TEST(PlanningContract, TheStartStateConditionOutranksAnExhaustedSlice) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  StartStateCondition condition;
  condition.established = true;
  condition.in_collision = true;
  condition.contacts = {"wrist_1_link against restocker/obstacle/0"};
  const auto result = classify_plan_failure(
    ErrorCodes::FAILURE, condition, PlanningSlice{15.0, 15.0}, "pre-grasp");
  EXPECT_EQ(result.status, PlanningStatus::kStartStateInvalid);
  EXPECT_NE(result.detail.find("start state is in collision"), std::string::npos) << result.detail;
}

TEST(PlanningContract, ASliceThatWasNeverGivenIsNeverExhausted) {
  EXPECT_FALSE(PlanningSlice{}.exhausted());
  EXPECT_FALSE((PlanningSlice{15.0, 0.0}.exhausted()));
  EXPECT_FALSE((PlanningSlice{14.249, 15.0}.exhausted()));
  EXPECT_TRUE((PlanningSlice{14.25, 15.0}.exhausted()));
}

TEST(PlanningContract, DescribesTheStartStateCondition) {
  StartStateCondition not_asked;
  not_asked.unestablished = "/check_state_validity is unavailable";
  EXPECT_EQ(
    describe_start_state_condition(not_asked),
    "start-state condition not established (/check_state_validity is unavailable)");

  StartStateCondition valid;
  valid.established = true;
  EXPECT_EQ(
    describe_start_state_condition(valid), "start state is valid in the current planning scene");

  StartStateCondition collision;
  collision.established = true;
  collision.in_collision = true;
  EXPECT_EQ(
    describe_start_state_condition(collision),
    "start state was rejected by the planning scene, which named no contact");
  collision.contacts = {"wrist_1_link against restocker/obstacle/0"};
  EXPECT_EQ(
    describe_start_state_condition(collision),
    "start state is in collision: wrist_1_link against restocker/obstacle/0");
}

TEST(PlanningContract, ValidatesCompleteFreshJointState) {
  const std::vector<double> positions(kManipulatorJoints.size(), 0.0);
  EXPECT_TRUE(validate_joint_state(kManipulatorJoints, positions, 0.05, 0.5).ok());

  auto incomplete = kManipulatorJoints;
  incomplete.pop_back();
  EXPECT_EQ(
    validate_joint_state(incomplete, std::vector<double>(incomplete.size(), 0.0), 0.05, 0.5)
    .status,
    PlanningStatus::kRobotStateIncomplete);
  EXPECT_EQ(
    validate_joint_state(kManipulatorJoints, positions, 0.8, 0.5).status,
    PlanningStatus::kRobotStateStale);
}

TEST(PlanningContract, ValidatesPostExecutionJointAndPoseError) {
  const std::vector<double> target{0.2, 0.1, -0.2, 0.3, 0.1, -0.1, 0.2};
  auto actual = target;
  actual[3] += 0.005;
  EXPECT_TRUE(validate_joint_target(kManipulatorJoints, actual, target, 0.01).ok());

  actual[3] += 0.02;
  EXPECT_EQ(
    validate_joint_target(kManipulatorJoints, actual, target, 0.01).status,
    PlanningStatus::kPostExecutionMismatch);
  EXPECT_TRUE(validate_pose_error(0.002, 0.01, 0.005, 0.02).ok());
  EXPECT_EQ(
    validate_pose_error(0.02, 0.01, 0.005, 0.02).status,
    PlanningStatus::kPostExecutionMismatch);
}

TEST(PlanningContract, RejectsMalformedTrajectory) {
  TrajectoryObservation trajectory{
    kManipulatorJoints,
    {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.1},
      {{0.2, 0.1, -0.2, 0.3, 0.1, -0.1, 0.2}, 1.5}}};
  EXPECT_TRUE(validate_trajectory(trajectory).ok());

  trajectory.points.clear();
  EXPECT_EQ(validate_trajectory(trajectory).status, PlanningStatus::kInvalidTrajectory);

  trajectory.points = {{{0.0}, 0.1}};
  EXPECT_EQ(validate_trajectory(trajectory).status, PlanningStatus::kInvalidTrajectory);

  trajectory.points = {
    {{0.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<double>::infinity(), 0.0}, 0.1}};
  EXPECT_EQ(validate_trajectory(trajectory).status, PlanningStatus::kInvalidTrajectory);
}

// A failure detail that quotes MoveIt's own name for the code is the difference between saying
// what MoveIt reported and inferring a cause from a percentage.
TEST(PlanningContract, NamesMoveItErrorCodes) {
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  EXPECT_STREQ(moveit_error_name(ErrorCodes::SUCCESS), "SUCCESS");
  EXPECT_STREQ(moveit_error_name(ErrorCodes::CONTROL_FAILED), "CONTROL_FAILED");
  EXPECT_STREQ(moveit_error_name(ErrorCodes::NO_IK_SOLUTION), "NO_IK_SOLUTION");
  EXPECT_STREQ(moveit_error_name(ErrorCodes::GOAL_IN_COLLISION), "GOAL_IN_COLLISION");
  EXPECT_STREQ(moveit_error_name(ErrorCodes::FAILURE), "FAILURE");
  EXPECT_STREQ(moveit_error_name(12345), "an error code MoveIt does not name");
}

// MoveIt's Cartesian path service discards the result of its own time parameterisation and
// answers SUCCESS regardless, so a path it could not time arrives looking like a plan, with every
// waypoint still stamped at the zero it was added with. Executing that commands the whole segment
// at one instant, so it has to be caught before it reaches the controllers, and it has to be said
// as what it is rather than as a generic out-of-order trajectory.
TEST(PlanningContract, RejectsAnUntimedCartesianSolution) {
  const TrajectoryObservation untimed{
    kManipulatorJoints,
    {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0},
      {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0},
      {{0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0}}};
  const auto result = validate_trajectory_timing(untimed, "insert");
  EXPECT_EQ(result.status, PlanningStatus::kInvalidTrajectory);
  EXPECT_NE(result.detail.find("insert"), std::string::npos);
  EXPECT_NE(result.detail.find("time parameterisation"), std::string::npos);
}

TEST(PlanningContract, AcceptsAndOrdersCartesianSolutionTiming) {
  const TrajectoryObservation timed{
    kManipulatorJoints,
    {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0},
      {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.25},
      {{0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.5}}};
  EXPECT_TRUE(validate_trajectory_timing(timed, "insert").ok());

  // A single waypoint is a segment that is already at its goal, not an untimed path.
  const TrajectoryObservation single{
    kManipulatorJoints, {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0}}};
  EXPECT_TRUE(validate_trajectory_timing(single, "insert").ok());

  EXPECT_EQ(
    validate_trajectory_timing(TrajectoryObservation{kManipulatorJoints, {}}, "insert").status,
    PlanningStatus::kInvalidTrajectory);

  const TrajectoryObservation out_of_order{
    kManipulatorJoints,
    {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0},
      {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.5},
      {{0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.25}}};
  const auto disordered = validate_trajectory_timing(out_of_order, "insert");
  EXPECT_EQ(disordered.status, PlanningStatus::kInvalidTrajectory);
  EXPECT_NE(disordered.detail.find("waypoint 2"), std::string::npos);
}

TEST(PlanningContract, SplitsLongControllerGoalsWithoutSkippingThePlannedPath) {
  TrajectoryObservation trajectory{kManipulatorJoints, {}};
  for (std::size_t index = 0U; index < 8U; ++index) {
    trajectory.points.push_back(
      TrajectoryPointObservation{
          std::vector<double>(kManipulatorJoints.size(), static_cast<double>(index)),
          2.0 * static_cast<double>(index)});
  }

  const auto chunks = controller_execution_chunks(trajectory, std::chrono::seconds(5));
  ASSERT_TRUE(chunks);
  ASSERT_EQ(chunks->size(), 4U);
  EXPECT_EQ((*chunks)[0].first_point, 0U);
  EXPECT_EQ((*chunks)[0].last_point, 2U);
  EXPECT_EQ((*chunks)[1].first_point, 2U);
  EXPECT_EQ((*chunks)[1].last_point, 4U);
  EXPECT_EQ((*chunks)[2].first_point, 4U);
  EXPECT_EQ((*chunks)[2].last_point, 6U);
  EXPECT_EQ((*chunks)[3].first_point, 6U);
  EXPECT_EQ((*chunks)[3].last_point, 7U);
}

TEST(PlanningContract, KeepsShortGoalsWholeAndRefusesAnUnsplittableGap) {
  const TrajectoryObservation short_trajectory{
    kManipulatorJoints,
    {{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0},
      {{0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 4.0}}};
  const auto whole = controller_execution_chunks(short_trajectory, std::chrono::seconds(5));
  ASSERT_TRUE(whole);
  ASSERT_EQ(whole->size(), 1U);
  EXPECT_EQ(whole->front().first_point, 0U);
  EXPECT_EQ(whole->front().last_point, 1U);

  EXPECT_FALSE(controller_execution_chunks(short_trajectory, std::chrono::seconds(3)));
  EXPECT_FALSE(controller_execution_chunks(short_trajectory, std::chrono::milliseconds::zero()));
}

// MoveIt hands the coordinator CONTROL_FAILED whatever the controller actually said, so the
// reason has to be read from the controller's own log line. The two sentences below are the ones
// joint_trajectory_controller writes in the same branch that sets the FollowJointTrajectory
// error code, which is why they can be named; anything else is quoted and left unnamed rather
// than given a code it might not carry.
TEST(PlanningContract, ReadsTheControllersOwnAbortReason) {
  const auto aborted =
    parse_controller_abort("arm_controller", "Aborted due to state tolerance violation");
  ASSERT_TRUE(aborted.has_value());
  EXPECT_EQ(aborted->controller, "arm_controller");
  EXPECT_EQ(aborted->error, "PATH_TOLERANCE_VIOLATED");
  EXPECT_EQ(aborted->reported, "Aborted due to state tolerance violation");

  const auto goal_time = parse_controller_abort(
    "rail_controller", "Aborted due to goal_time_tolerance exceeding by 1.006496 seconds");
  ASSERT_TRUE(goal_time.has_value());
  EXPECT_EQ(goal_time->controller, "rail_controller");
  EXPECT_EQ(goal_time->error, "GOAL_TOLERANCE_VIOLATED");

  // An abort this release does not name is still an abort, and is reported as one.
  const auto unnamed = parse_controller_abort("arm_controller", "Aborted due to command timeout");
  ASSERT_TRUE(unnamed.has_value());
  EXPECT_TRUE(unnamed->error.empty());
  EXPECT_EQ(unnamed->reported, "Aborted due to command timeout");

  EXPECT_FALSE(parse_controller_abort("arm_controller", "Goal reached, success!"));
  EXPECT_FALSE(parse_controller_abort("arm_controller", "Holding position due to state tolerance"));
  EXPECT_FALSE(parse_controller_abort("", "Aborted due to state tolerance violation"));
}

// The clause has to separate three states of knowledge, because a benchmark groups on it:
// the fingerprint was measured and found, measured and not found, or nothing was measured at all.
// Collapsing the third into the second would report an absence that was never looked for.
TEST(PlanningContract, SaysWhetherTheVelocityLimitFingerprintWasFoundOrOnlyLookedFor) {
  const ControllerAbortReport report{
    "arm_controller", "Aborted due to state tolerance violation",
    "PATH_TOLERANCE_VIOLATED"};

  SegmentVelocityEvidence found;
  found.joint = "wrist_2_joint";
  found.urdf_limit = 3.1;
  found.planned_maximum = 0.095;
  found.samples = 412U;
  const auto impulse = describe_controller_abort(report, found);
  EXPECT_NE(impulse.find("aborted with PATH_TOLERANCE_VIOLATED"), std::string::npos);
  EXPECT_NE(impulse.find("Aborted due to state tolerance violation"), std::string::npos);
  EXPECT_NE(
    impulse.find("wrist_2_joint feedback velocity reached its URDF velocity limit of 3.1000"),
    std::string::npos);
  EXPECT_NE(impulse.find("no faster than 0.0950"), std::string::npos);

  SegmentVelocityEvidence looked;
  looked.samples = 412U;
  const auto lagged = describe_controller_abort(report, looked);
  EXPECT_NE(lagged.find("feedback velocity was observed at its URDF"), std::string::npos);
  // The two must not share the phrase the benchmark taxonomy separates them on.
  EXPECT_EQ(lagged.find("reached its URDF velocity limit of"), std::string::npos);

  const auto unmeasured = describe_controller_abort(report, SegmentVelocityEvidence{});
  EXPECT_NE(unmeasured.find("velocity limits were not checked"), std::string::npos);
  EXPECT_EQ(unmeasured.find("feedback velocity was observed at its URDF"), std::string::npos);
}

// The saturation test is sufficient for the impulse and not necessary: joint_limits.yaml margins
// elbow_joint to 1.8 rad/s against a URDF 2.4, so an uncommanded excursion that stops inside that
// band satisfies no test in the port. Reported as a bare absence, seven of the ten archived
// controller aborts could not be told from ordinary tracking lag. The near miss has to carry its
// own number.
TEST(PlanningContract, ReportsHowNearAnUnsaturatedSegmentCameToTheVelocityBound) {
  const ControllerAbortReport report{
    "arm_controller", "Aborted due to state tolerance violation",
    "PATH_TOLERANCE_VIOLATED"};

  SegmentVelocityEvidence near;
  near.samples = 1659U;
  near.nearest_joint = "elbow_joint";
  near.nearest_peak = 1.9012;
  near.nearest_limit = 2.4;
  near.nearest_planned = 0.36;
  const auto detail = describe_controller_abort(report, near);
  EXPECT_NE(detail.find("feedback velocity was observed at its URDF"), std::string::npos);
  EXPECT_NE(detail.find("the nearest was elbow_joint at 1.9012 rad/s"), std::string::npos);
  EXPECT_NE(detail.find("0.7922 of its URDF bound of 2.4000"), std::string::npos);
  EXPECT_NE(detail.find("planned maximum of 0.3600"), std::string::npos);
  // Still must not collide with the phrase the taxonomy separates the saturated row on.
  EXPECT_EQ(detail.find("reached its URDF velocity limit of"), std::string::npos);

  // A trajectory with no velocities cannot answer "against what reference" here either.
  SegmentVelocityEvidence unplanned = near;
  unplanned.nearest_planned = -1.0;
  const auto without = describe_controller_abort(report, unplanned);
  EXPECT_NE(without.find("0.7922 of its URDF bound"), std::string::npos);
  EXPECT_EQ(without.find("planned maximum"), std::string::npos);

  // And when no joint carried a usable URDF bound there is nothing to be near, so the clause
  // stays off rather than reporting a fraction of zero.
  SegmentVelocityEvidence unbounded;
  unbounded.samples = 412U;
  const auto silent = describe_controller_abort(report, unbounded);
  EXPECT_EQ(silent.find("the nearest was"), std::string::npos);
}

// A trajectory MoveIt timed but did not fill velocities for cannot answer "against what
// reference", and the clause must leave the question out rather than answer it with a zero.
TEST(PlanningContract, OmitsThePlannedReferenceWhenTheTrajectoryCarriedNone) {
  SegmentVelocityEvidence found;
  found.joint = "shoulder_lift_joint";
  found.urdf_limit = 2.0;
  found.samples = 90U;
  const ControllerAbortReport report{
    "arm_controller", "Aborted due to state tolerance violation",
    "PATH_TOLERANCE_VIOLATED"};
  const auto detail = describe_controller_abort(report, found);
  EXPECT_NE(detail.find("reached its URDF velocity limit of 2.0000"), std::string::npos);
  EXPECT_EQ(detail.find("no faster than"), std::string::npos);
}

TEST(PlanningContract, ExposesStableStatusNamesAndExitCodes) {
  const PlanningResult result{PlanningStatus::kExpectedCollisionMissing, "not colliding"};
  EXPECT_STREQ(to_string(result.status), "expected_collision_missing");
  EXPECT_EQ(result.exit_code(), 22);
  EXPECT_FALSE(result.ok());
}

}  // namespace
}  // namespace restocker_task_executor
