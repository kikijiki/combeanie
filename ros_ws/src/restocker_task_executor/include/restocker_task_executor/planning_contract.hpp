// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace restocker_task_executor
{

enum class PlanningPhase
{
  kPlan,
  kExecute,
};

enum class PlanningStatus
{
  kSuccess = 0,
  kServerUnavailable = 10,
  kRobotStateIncomplete = 11,
  kRobotStateStale = 12,
  kModelContractMismatch = 13,
  kStartStateInvalid = 14,
  kPlanningRejected = 15,
  kPlanningTimedOut = 16,
  kInvalidTrajectory = 17,
  kSceneSynchronizationTimedOut = 18,
  kExecutionRejected = 19,
  kExecutionTimedOut = 20,
  kPostExecutionMismatch = 21,
  kExpectedCollisionMissing = 22,
};

struct PlanningResult
{
  PlanningStatus status;
  std::string detail;

  [[nodiscard]] bool ok() const noexcept;
  [[nodiscard]] int exit_code() const noexcept;
};

struct TrajectoryPointObservation
{
  std::vector<double> positions;
  double time_from_start_sec;
};

struct TrajectoryObservation
{
  std::vector<std::string> joint_names;
  std::vector<TrajectoryPointObservation> points;
};

struct TrajectoryExecutionChunk
{
  std::size_t first_point{0U};
  std::size_t last_point{0U};
};

[[nodiscard]] const char * to_string(PlanningStatus status) noexcept;

// MoveIt's own name for one of its error-code values. A failure detail that quotes this reports
// what MoveIt said; a detail that infers a cause from a fraction reports what the caller guessed.
[[nodiscard]] const char * moveit_error_name(int moveit_error_code) noexcept;

[[nodiscard]] PlanningResult classify_moveit_error(
  PlanningPhase phase, int moveit_error_code, const std::string & context);

// Whether a failed MoveIt execute() proves the trajectory controllers reached a terminal stop
// for the segment (Card 086 stage 1a, CMB-SPEC-13). Only CONTROL_FAILED does: a controller
// reported a terminal status. TIMED_OUT does not: MoveIt's trajectory-execution duration monitor
// cancels the controllers and returns without waiting for their terminal results (pinned TEM
// 2.12.4 source, inventory §backend finality), so the arm may still be decelerating. Every other
// code leaves the stop unestablished too.
[[nodiscard]] bool execution_failure_reached_terminal_stop(int moveit_error_code) noexcept;

// What the scene says about the state a failed plan was asked to start from.
//
// MoveIt's CheckStartStateCollision request adapter aborts the planning pipeline before the
// planner runs and move_group reports that abort as the generic FAILURE code — the same code a
// genuine "no solution for this pose" carries — so the code alone cannot say whether the scene
// or the goal refused. The scene is asked directly through /check_state_validity, and a question
// that could not be answered is recorded as unestablished rather than as a verdict: a cause that
// cannot be shown is never guessed at.
struct StartStateCondition
{
  // True only when /check_state_validity answered inside its timeout.
  bool established{false};
  // Meaningful only when established: the state was rejected by the current planning scene.
  bool in_collision{false};
  // "body against body" pairs, capped by whoever built this; the count of contacts beyond the
  // cap is reported instead of silently dropped.
  std::vector<std::string> contacts;
  std::size_t suppressed_contacts{0U};
  // Why the condition could not be established. Empty when established.
  std::string unestablished;
};

// The receipt for one start-state condition: the contacts when it is a collision, the plain
// negative when the scene accepted the state, and the reason when nothing could be asked.
[[nodiscard]] std::string describe_start_state_condition(const StartStateCondition & condition);

// How long the last planning attempt ran against the slice of the budget it was given.
//
// move_action reports the planner's own TIMED_OUT as the generic FAILURE by the time plan()
// answers (measured: an OMPL `Planner 'OMPL' failed with error code TIMED_OUT` two lines above a
// `MoveIt FAILURE (code 99999)` refusal whose attempt ran 15.005 s of a 14.996 s slice), so the
// code alone never says the budget ran out. Consuming the slice is the mechanical evidence that
// it did, and it is what keeps a budget timeout from reading as a verdict on the goal.
struct PlanningSlice
{
  double attempted_seconds{0.0};
  double budget_seconds{0.0};

  // True when the attempt ran at least this fraction of its slice before failing.
  [[nodiscard]] bool exhausted() const noexcept;
};

// Fraction of a slice an attempt must consume before its failure is read as the budget running
// out rather than as a verdict on the goal. An ordinary rejection returns in milliseconds.
inline constexpr double kPlanningSliceExhaustedFraction = 0.95;

// The classification for a failed plan, refined by what the scene says about its start state and
// by how long the attempt ran.
//
// Only kPlanningRejected is refined: a start state the scene refuses becomes kStartStateInvalid —
// the status classify_moveit_error already maps MoveIt's own START_STATE_* codes to — because
// that is the one status that reads as a verdict on the goal and makes the coordinator spend a
// grasp candidate on it, and a scene condition is not a property of the candidate. An attempt
// that consumed its slice becomes kPlanningTimedOut, which was always a non-verdict. A code
// MoveIt named itself and an unestablished condition leave the classification alone, with the
// fact appended.
[[nodiscard]] PlanningResult classify_plan_failure(
  int moveit_error_code, const StartStateCondition & start_state, const PlanningSlice & slice,
  const std::string & context);

// What a trajectory controller said about the trajectory it gave up on.
//
// MoveIt collapses every controller failure into CONTROL_FAILED before the coordinator can see
// it, so the reason has to be read from the controller itself. It is read from the controller's
// own log line and not from the one MoveIt prints beside it, for a mechanical reason:
// moveit_simple_controller_manager logs through a free logger named
// "moveit.simple_controller_manager....", and rcl only publishes a logger to /rosout when it is
// a node logger or a child of one, so that line reaches the console and never leaves the
// process. joint_trajectory_controller logs through its own node logger, which does.
//
// ``reported`` is the controller's sentence verbatim. ``error`` is the matching
// FollowJointTrajectory error name, which is what the rest of this repository
// calls the fault, and is empty for a sentence this does not recognise, an
// unrecognised abort is quoted rather than given a name it might not have.
struct ControllerAbortReport
{
  std::string controller;
  std::string reported;
  std::string error;
};

// Reads one controller's own abort line: the logger name it came from and a message beginning
// "Aborted due to ". Returns nullopt for anything else, so an ordinary controller log line
// cannot be mistaken for an abort.
[[nodiscard]] std::optional<ControllerAbortReport> parse_controller_abort(
  const std::string & logger_name, const std::string & log_message);

// What the joints did during one executed segment, measured against the bounds the physics
// engine enforces rather than the margined ones MoveIt plans inside.
//
// The discriminating fact is a joint whose *feedback* velocity reached its URDF limit. A healthy
// segment cannot produce one: MoveIt plans inside the margined limits in joint_limits.yaml and
// scales them further, and the arm controller commands ff + p * error with the path constraint
// aborting at 0.15 rad, so neither term can drive a joint to its URDF bound before the abort. A
// feedback velocity sitting on that bound is the simulator clamping a motion nothing commanded,
// the impulse recorded as Signature B in the retired arm's taxonomy.
// Ordinary tracking lag leaves every
// joint well inside it, which is why the negative reading is worth reporting too.
struct SegmentVelocityEvidence
{
  // Empty unless some joint of the executed trajectory reached its URDF velocity limit, within
  // the tenth of a percent the reader of this struct treats as "reached".
  std::string joint;
  double urdf_limit{0.0};
  // The fastest this segment planned that joint. Negative when the trajectory carried no
  // velocities, which is the one case the comparison cannot be made.
  double planned_maximum{-1.0};
  // How many joint-state samples covered the segment. Zero means nothing was measured, which is
  // reported as such rather than as an absence of the fingerprint.
  std::size_t samples{0};

  // The saturation test above is a sufficient condition for the impulse and not a necessary one,
  // and the gap matters: joint_limits.yaml margins elbow_joint to 1.8 rad/s and wrist_2_joint to
  // 2.0 while the URDF permits 2.4 and 3.1, so an uncommanded excursion that stops anywhere
  // inside that band satisfies no test here and used to be reported only as an absence. Measured
  // over the archived campaigns, three aborts saturated and seven did not. Nothing recorded about
  // the seven could say whether they were ordinary tracking or a weaker impulse, which is the
  // question the whole family turns on.
  //
  // These fields close that band by reporting the same comparison continuously: the joint that
  // came nearest its own URDF bound, as a fraction of that bound, beside what the segment planned
  // for it. Ranking on peak/limit rather than on peak/planned is deliberate, it is dimensionless,
  // defined even for a joint the trajectory never asks to move, and it is the exact quantity the
  // saturation test thresholds at 0.999, so the negative reading is now the same measurement
  // rather than a different one.
  std::string nearest_joint;
  double nearest_peak{0.0};
  double nearest_limit{0.0};
  // As planned_maximum, for nearest_joint. Negative when the trajectory carried no velocities.
  double nearest_planned{-1.0};
};

// The clause appended to an execution failure's detail: what the controller said, and whether the
// segment carries the velocity-limit fingerprint. Never claims a cause the evidence does not
// carry, and says so when nothing was measured.
[[nodiscard]] std::string describe_controller_abort(
  const ControllerAbortReport & report, const SegmentVelocityEvidence & evidence);

[[nodiscard]] PlanningResult validate_model_contract(
  const std::string & planning_frame, const std::string & end_effector_link,
  const std::vector<std::string> & active_joints);

[[nodiscard]] PlanningResult validate_joint_state(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  double age_sec, double max_age_sec);

[[nodiscard]] PlanningResult validate_joint_target(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const std::vector<double> & target_positions, double tolerance);

[[nodiscard]] PlanningResult validate_pose_error(
  double translation_error_m, double rotation_error_rad, double translation_tolerance_m,
  double rotation_tolerance_rad);

[[nodiscard]] PlanningResult validate_trajectory(const TrajectoryObservation & trajectory);

// Timing-only check for a trajectory MoveIt has already accepted as its own solution.
//
// MoveIt's Cartesian path service ignores the return value of its time parameterisation and
// answers SUCCESS regardless, so a path it could not time comes back looking exactly like a plan:
// the waypoints are there and every time_from_start is still the zero it was added with.
// Executing that asks the controllers to reach every configuration at the same instant. This
// separates that case from an ordinary out-of-order trajectory, because the two need different
// things said about them.
[[nodiscard]] PlanningResult validate_trajectory_timing(
  const TrajectoryObservation & trajectory, const std::string & context);

// Divide a time-parameterized trajectory into overlapping controller goals. Each range includes
// both endpoints, and adjacent ranges share exactly one point so no part of the planned path is
// skipped. A gap between two supplied waypoints that is itself longer than the bound is refused.
[[nodiscard]] std::optional<std::vector<TrajectoryExecutionChunk>>
controller_execution_chunks(
  const TrajectoryObservation & trajectory, std::chrono::milliseconds maximum_duration);

}  // namespace restocker_task_executor
