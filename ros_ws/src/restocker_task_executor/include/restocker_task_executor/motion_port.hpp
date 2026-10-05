// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "restocker_task_executor/restock_coordinator_primitives.hpp"

namespace restocker_task_executor
{

// One planned, collision-checked motion segment. The coordinator submits exactly one at a time
// and never inspects the trajectory: planning and collision validation stay inside the port.
// How the end-effector is required to travel to the goal.
enum class MotionPathKind : std::uint8_t
{
  // Any collision-free path a sampling planner finds. Correct for free-space repositioning.
  kFreeSpace,
  // A straight line in Cartesian space from the current pose to the goal. Required wherever the
  // jaws must slide past a product rather than around it: a sampling planner is free to reach the
  // same grasp pose by sweeping a finger through the very object it is about to pick up, and both
  // the solution validator and the physics reject that.
  kLinear,
};

inline constexpr double kDefaultMotionPositionToleranceM = 0.005;

// Axis-aligned box in the planning frame that the end-effector link origin must stay inside for
// every state of a free-space path. Derived by the coordinator from surveyed geometry and segment
// endpoints; the port validates the finished trajectory before it can be executed.
struct MotionPathPositionBox
{
  Eigen::Vector3d min_corner_m{Eigen::Vector3d::Zero()};
  Eigen::Vector3d max_corner_m{Eigen::Vector3d::Zero()};
};

// Optional path orientation hold. Absolute axis tolerances match MoveIt's OrientationConstraint.
struct MotionPathOrientationHold
{
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  double absolute_x_axis_tolerance_rad{0.0};
  double absolute_y_axis_tolerance_rad{0.0};
  double absolute_z_axis_tolerance_rad{0.0};
};

// A bounded joint position prepared for use as an explicit planning start state. `corrected` is
// true only when a tiny feedback excursion beyond a reduced planning bound was clamped back onto
// that bound.
struct BoundedPlanningPosition
{
  double value{0.0};
  bool corrected{false};
};

// Normalize only a numerical excursion just beyond a bounded planning interval. A larger
// excursion is refused rather than hidden because it may be a real robot/model inconsistency.
[[nodiscard]] std::optional<BoundedPlanningPosition> normalize_bounded_planning_position(
  double measured, double minimum, double maximum, double maximum_correction) noexcept;

// Path constraints for free-space planning. Absent means unconstrained sampling (legacy behaviour).
// When present, the port applies them for that plan only and never retries without them.
struct MotionPathConstraints
{
  std::optional<MotionPathPositionBox> position_box;
  std::optional<MotionPathOrientationHold> orientation_hold;
  // Planning-frame id and constrained link. Empty strings mean the MoveIt port fills them from its
  // configured planning frame and end-effector link.
  std::string planning_frame;
  std::string link_name;
  // Human-readable box numbers for first-hand planning logs (max-Z comparisons later).
  std::string description;
};

// Milestone 10 §6 (Card 062): the straight-line exit from a grasp whose jaws closed on a product
// the arm does not hold. The jaws are already open when a goal carries it; the escape runs as a
// compound first leg back to the closed candidate's pre-grasp standoff (the approach reversed),
// and the goal's own plan starts from there.
struct GraspEscape
{
  Eigen::Isometry3d planning_frame_from_standoff_tool0{Eigen::Isometry3d::Identity()};
  // The one planning-scene object whose contact with a finger may be tolerated.
  std::string target_object_id;
  // The finger links that may touch it. Every other link, and every other object, stays forbidden.
  std::vector<std::string> tolerated_links;
};

[[nodiscard]] bool valid_grasp_escape(const GraspEscape & escape) noexcept;

// One contact MoveIt reported, as the two body names in either order.
using CollisionContactPair = std::pair<std::string, std::string>;

struct GraspEscapeContactScope
{
  bool admitted{false};
  // (target object, finger link) pairs actually in contact — exactly what the escape tolerates.
  std::vector<CollisionContactPair> tolerated_pairs;
  std::string refusal;
};

// Admits the escape's start state only when every contact is between the target object and a
// tolerated link; no contact at all is admitted with nothing tolerated. Fail-closed on anything
// else, including an escape with no target or no links.
[[nodiscard]] GraspEscapeContactScope classify_grasp_escape_contacts(
  const std::vector<CollisionContactPair> & contacts, const GraspEscape & escape);

struct MotionGoal
{
  // Target pose for the end-effector link, expressed in the planning frame.
  Eigen::Isometry3d planning_frame_from_tool0{Eigen::Isometry3d::Identity()};
  double position_tolerance_m{kDefaultMotionPositionToleranceM};
  double orientation_tolerance_rad{0.02};
  // Independent post-plan safety limit for an attached upright cylinder. Side grasps align the
  // product axis with tool0 +X; this bounds it against planning-frame +Z at every densely
  // interpolated FK sample. Valid for linear held-product segments as well as free-space carry.
  std::optional<double> maximum_tool0_x_axis_tilt_rad;
  double velocity_scaling{0.2};
  double acceleration_scaling{0.2};
  std::chrono::milliseconds planning_time{std::chrono::seconds(5)};
  // Bound the duration of one FollowJointTrajectory goal. Long sampled traverses are executed as
  // contiguous, overlapping pieces of the same collision-checked plan; this avoids the simulator
  // controller fault whose measured pre-insert population occurs only after 14 seconds in one
  // controller goal. Ten seconds leaves a measured four-second margin to that onset.
  std::chrono::milliseconds maximum_controller_goal_duration{std::chrono::seconds(10)};
  // Number of independently sampled free-space solutions to compare before execution. The
  // shortest time-parameterized candidate wins. A goal with a required continuation instead uses
  // this as the maximum number of distinct continuation-safe IK endpoints and executes the first
  // one OMPL can reach. Linear paths ignore it because their geometry fixes the interpolation. A
  // value above one is useful for long redundant-group transfers, where accepting the first
  // pose-goal IK branch can double the commanded travel.
  std::size_t free_space_plan_candidates{1U};
  MotionPathKind path{MotionPathKind::kFreeSpace};
  // Maximum end-effector displacement between consecutive interpolated configurations of a linear
  // path. Ignored for free-space motion.
  double cartesian_step_m{0.005};
  // Fraction of a linear path that must be achievable before it is accepted. A partial approach
  // stops short of the product, so anything less than the whole path is a planning failure.
  double minimum_cartesian_fraction{0.999};
  // Names this segment in first-hand logs and failure details. Without it every segment reports
  // whichever label happened to be hard-coded, and a failure names the wrong motion.
  std::string label{"motion segment"};
  // Optional capability for planning while the planning-scene projector is deliberately frozen
  // by the transaction that physically detached a placed product. Empty for every ordinary
  // segment. The motion backend must validate a non-empty token with the projector both before
  // planning and immediately before execution; possession alone is never treated as authority.
  std::string planning_scene_lease_token;
  // Optional first leg for a compound retreat: move linearly to this surveyed placement-retreat
  // pose before free-space planning to the final camera viewpoint above. This is not an arbitrary
  // flyover; it is the placement generator's short, collision-checked withdrawal from the lane.
  // Valid only when `path` is kFreeSpace.
  std::optional<Eigen::Isometry3d> linear_egress_pose;
  // Optional Cartesian continuation that a free-space plan's endpoint must support before that
  // plan may be executed. Pre-grasp uses this to reject an IK branch whose wrist reaches the pose
  // but collides with the stock tray on the required straight approach. The continuation is only
  // checked here; it remains a separately authorized and executed motion segment.
  std::optional<Eigen::Isometry3d> required_linear_continuation_pose;
  // Jaw position used while validating that continuation. Pre-grasp is executed before the jaws
  // open, but the actual approach starts afterward; checking with the stale aperture would reject
  // a valid approach on collisions that cannot exist when it runs. Present exactly when the pose
  // above is present.
  std::optional<double> required_linear_continuation_gripper_joint_position_m;
  // Extra downward probe applied only while validating the continuation. Requiring the same IK
  // branch to survive this stricter line leaves clearance for controller endpoint tolerance; the
  // separately authorized approach still targets the unmodified pose above.
  double required_linear_continuation_vertical_margin_m{0.0};
  // Optional held-product continuation after the approach above. Pre-grasp uses this pair to
  // prove that the same IK branch can retract and clear one product diameter from the tray before
  // committing to a grasp. All three fields are present together or absent together.
  std::optional<Eigen::Isometry3d> required_postcontinuation_linear_retract_pose;
  std::optional<Eigen::Isometry3d> required_postcontinuation_linear_egress_pose;
  std::optional<double> required_postcontinuation_gripper_joint_position_m;
  // Optional line that the endpoint of this linear motion must support at a specified later jaw
  // aperture. Insert uses it to prove the post-release lane egress before any insertion executes.
  std::optional<Eigen::Isometry3d> required_postmotion_linear_egress_pose;
  std::optional<double> required_postmotion_linear_egress_gripper_joint_position_m;
  // Starts this transfer with a fresh MoveGroup action client. Long campaigns otherwise retain
  // every prior goal handle in one interface; after enough successful segments, a result callback
  // can be logged but never wake the synchronous caller. Only a pre-motion boundary sets this.
  bool reset_planning_interface_before_plan{false};
  // Optional free-space path corridor/ceiling. Ignored by linear Cartesian segments; a linear goal
  // that carries constraints is rejected by valid_motion_goal.
  std::optional<MotionPathConstraints> path_constraints;
  // Card 062: leave a failed grasp along the reversed approach before this goal's own plan.
  std::optional<GraspEscape> grasp_escape;
  // Set by the port on the escape's own linear leg: plan it in a local scene copy with only the
  // contact pairs classify_grasp_escape_contacts() admitted allowed.
  bool grasp_escape_leg{false};
};

[[nodiscard]] bool valid_motion_path_constraints(
  const MotionPathConstraints & constraints) noexcept;

// Inclusive point check used by the post-plan trajectory gate. No extra epsilon is added here:
// the coordinator already owns the explicitly bounded numerical shell in the constructed box.
[[nodiscard]] bool motion_path_position_satisfies_box(
  const Eigen::Vector3d & position, const MotionPathPositionBox & box) noexcept;

// Returns a strictly smaller planning box. OMPL may project interpolated states onto its path
// constraint face; planning against an inset box keeps those states inside the outer box that is
// independently revalidated before execution. Invalid/non-finite insets fail closed.
[[nodiscard]] std::optional<MotionPathPositionBox> inset_motion_path_position_box(
  const MotionPathPositionBox & box, double inset_m) noexcept;

[[nodiscard]] bool valid_motion_goal(const MotionGoal & goal) noexcept;

// How one round of a free-space goal's planning floor is divided across its candidates
// (Milestone 10 §6, Card 074: the split of a floor is not itself a budget). Every candidate
// keeps a first-pass slice so a fast green solve never waits behind a hard one, and the
// remainder of the floor is reserved for a single second attempt by the preferred candidate —
// the endpoint the loop stops at the moment it plans — whose aggregate share inside the round
// is then exactly half the floor. One candidate keeps the whole floor as one attempt (a split
// would restart the search it was about to spend). Degenerate inputs (no budget, no candidates)
// yield an all-zero plan and the caller refuses before planning.
struct FreeSpaceSlicePlan
{
  double first_pass_slice_seconds{0.0};
  double second_pass_slice_seconds{0.0};
};

[[nodiscard]] FreeSpaceSlicePlan plan_free_space_slices(
  double budget_seconds, std::size_t candidate_count) noexcept;

// One candidate's verdict inside a free-space round: accepted, refused by the deadline
// (the only verdict the reserved second pass reacts to), canceled, or rejected as a joint goal.
struct FreeSpaceSliceAttempt
{
  bool accepted{false};
  bool timed_out{false};
  bool canceled{false};
  bool endpoint_rejected{false};
};

enum class FreeSpaceSliceRoundStatus : std::uint8_t
{
  // Every candidate ran (or the floor ran out first): the caller falls through to its refusal.
  kCompleted,
  // A candidate planned and the goal stops at the first planned endpoint.
  kStoppedOnAccept,
  // Cancellation arrived during planning: the caller reports MotionOutcome::kCanceled.
  kCanceled,
  // The continuation endpoint was rejected as a joint goal: the caller reports a refusal.
  kEndpointRejected,
};

struct FreeSpaceSliceRoundResult
{
  FreeSpaceSliceRoundStatus status{FreeSpaceSliceRoundStatus::kCompleted};
  std::size_t accepted_candidates{0U};
  // The first candidate that timed out in pass one; == candidate_count when none did.
  std::size_t second_pass_candidate{0U};
  bool second_pass_ran{false};
  double second_pass_slice_seconds{0.0};
};

// The round that spends one free-space planning floor (Milestone 10 §6, Card 074): every
// candidate gets its first-pass slice, then — only if nothing was accepted, a candidate timed
// out and the floor still has time left — that first timed-out candidate gets one longer attempt
// against whatever remains. `attempt` is the caller's per-candidate planner (it owns the budget
// bookkeeping) and `remaining` its live floor left, so the round's own control flow is testable
// without MoveIt. Extracted from `plan_free_space` verbatim: same order, same breaks, same
// terminal conditions.
template<typename AttemptFn, typename RemainingFn>
FreeSpaceSliceRoundResult run_free_space_slice_round(
  const FreeSpaceSlicePlan & plan, std::size_t candidate_count,
  bool stop_after_first_accept, RemainingFn && remaining, AttemptFn && attempt)
{
  FreeSpaceSliceRoundResult result;
  result.second_pass_candidate = candidate_count;
  const auto settle = [&](const FreeSpaceSliceAttempt & verdict) {
    if (verdict.canceled) {
      result.status = FreeSpaceSliceRoundStatus::kCanceled;
      return false;
    }
    if (verdict.endpoint_rejected) {
      result.status = FreeSpaceSliceRoundStatus::kEndpointRejected;
      return false;
    }
    if (verdict.accepted) {
      ++result.accepted_candidates;
    }
    return true;
  };
  for (std::size_t candidate = 0U; candidate < candidate_count; ++candidate) {
    const auto left = remaining();
    if (left <= std::chrono::steady_clock::duration::zero()) {
      break;
    }
    const double slice_seconds = std::min(
      plan.first_pass_slice_seconds, std::chrono::duration<double>(left).count());
    if (!(slice_seconds > 0.0)) {
      break;
    }
    const auto verdict = attempt(candidate, slice_seconds);
    if (!settle(verdict)) {
      return result;
    }
    if (verdict.timed_out && result.second_pass_candidate == candidate_count) {
      result.second_pass_candidate = candidate;
    }
    if (verdict.accepted && stop_after_first_accept) {
      result.status = FreeSpaceSliceRoundStatus::kStoppedOnAccept;
      return result;
    }
  }
  // The reserved remainder: one longer attempt for the first candidate that timed out, spent
  // against whatever is left of the floor, and only while nothing has been accepted.
  if (result.accepted_candidates == 0U &&
    result.second_pass_candidate != candidate_count)
  {
    const auto left = remaining();
    if (left > std::chrono::steady_clock::duration::zero()) {
      result.second_pass_ran = true;
      result.second_pass_slice_seconds = std::chrono::duration<double>(left).count();
      const auto verdict = attempt(
        result.second_pass_candidate, result.second_pass_slice_seconds);
      if (!settle(verdict)) {
        return result;
      }
    }
  }
  return result;
}

enum class MotionOutcome : std::uint8_t
{
  // The arm reached the requested pose and the controllers reported success.
  kSucceeded,
  // No collision-free plan was found, or the goal was unreachable.
  kPlanningFailed,
  // A plan existed but execution did not complete. The arm may have moved.
  kExecutionFailed,
  // Cancellation was requested and the port stopped the motion.
  kCanceled,
  // The port did not reach a terminal result inside its own deadline.
  kTimedOut,
  // MoveIt or the controllers were not available. No motion was attempted.
  kUnavailable,
};

// True when the outcome proves no joint command was issued for this goal. Only these outcomes
// permit continuing without a stopped-state check.
[[nodiscard]] bool motion_definitely_not_started(MotionOutcome outcome) noexcept;

[[nodiscard]] const char * motion_outcome_name(MotionOutcome outcome) noexcept;

enum class MotionSubmitStatus : std::uint8_t
{
  kAccepted,
  // The goal itself is malformed. Never retryable.
  kInvalidRequest,
  // A goal is already outstanding. The caller must wait for its completion.
  kBusy,
  // The port cannot take work at all, for example during shutdown.
  kUnavailable,
};

struct MotionSubmitResult
{
  MotionSubmitStatus status{MotionSubmitStatus::kUnavailable};
  std::string detail;

  [[nodiscard]] explicit operator bool() const noexcept
  {
    return status == MotionSubmitStatus::kAccepted;
  }
};

struct MotionCompletion
{
  OperationCorrelation correlation;
  MotionOutcome outcome{MotionOutcome::kUnavailable};
  std::string detail;
  // True only when the backend established that this segment's execution reached a terminal state
  // and the controllers were stopped: a controller reported a terminal failure for the
  // trajectory, or the backend's own duration monitor gave up on it and cancelled them. Either
  // way the segment is over and nothing further is commanded for it. A cancellation still in
  // flight, a rejected goal, or a backend the port lost contact with cannot establish that and
  // leave this false.
  //
  // It is a necessary condition for replanning from the current state, not a sufficient one: that
  // the arm has actually come to rest is a claim about the robot, and only fresh robot telemetry
  // can carry that.
  bool execution_reached_terminal_stop{false};
  // True only when the planner was handed this goal pose and could not produce a plan for it:
  // the robot model rejected the pose, no collision-free path was found, a straight line
  // truncated, or the resulting trajectory could not be timed. Every one of those is a verdict
  // on *this pose*.
  //
  // It exists because kPlanningFailed is broader than that. A planning-scene authority that
  // could not be established, or that changed while the planner searched, also completes as
  // kPlanningFailed (correctly, because nothing was commanded) and says nothing whatever
  // about the pose. Attributing one of those to the goal would blame the grasp for a stalled
  // obstacle stream.
  bool planner_refused_the_goal{false};
  // Card 062: the goal's grasp escape leg executed, so the arm left the grasp even if a later
  // leg failed; the coordinator must not command the escape again.
  bool grasp_escape_executed{false};
  // Card 086 stage 1b review B1: set by the port at the moment a trajectory was handed to the
  // backend (MoveIt execute() called, for any slice). From then on the arm may be moving, so no
  // outcome of this completion, kUnavailable and kPlanningFailed included, proves that nothing
  // was commanded. False means execute() was never called for this goal.
  bool submitted_to_backend{false};
};

// Whether a completion proves no joint command was issued for its goal: the outcome class allows it
// AND the port never handed a trajectory to the backend (no execute() call, any slice) AND no grasp
// escape leg ran. kUnavailable and kPlanningFailed after a submitted execute prove nothing
// (Card 086 stage 1b review B1).
[[nodiscard]] bool motion_completion_definitely_not_started(
  const MotionCompletion & completion) noexcept;

// Submit one motion segment and observe its result. Implementations must invoke the completion
// callback exactly once per accepted submission, from a thread the coordinator does not own, and
// must not call back inline from submit().
class MotionPort
{
public:
  using CompletionCallback = std::function<void (MotionCompletion)>;

  virtual ~MotionPort() = default;

  // False while the planning or execution backend is unreachable. Submission is still allowed;
  // it will complete kUnavailable rather than block.
  [[nodiscard]] virtual bool ready() const = 0;

  // Accepts at most one outstanding goal. A second submission before completion is rejected.
  [[nodiscard]] virtual MotionSubmitResult submit(
    OperationCorrelation correlation, MotionGoal goal, CompletionCallback callback) = 0;

  // Request that an outstanding motion stop. The completion still arrives, normally kCanceled.
  // Safe to call with nothing outstanding.
  virtual void cancel() noexcept = 0;
};

}  // namespace restocker_task_executor
