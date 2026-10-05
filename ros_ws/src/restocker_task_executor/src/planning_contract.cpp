// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#include "restocker_task_executor/planning_contract.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

namespace restocker_task_executor
{
namespace
{

const std::vector<std::string> kExpectedActiveJoints{
  "rail_joint", "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
  "wrist_2_joint", "wrist_3_joint"};

PlanningResult failure(PlanningStatus status, const std::string & detail)
{
  return {status, detail};
}

std::string with_context(const std::string & context, const std::string & detail)
{
  if (context.empty()) {
    return detail;
  }
  return context + ": " + detail;
}

}  // namespace

bool PlanningResult::ok() const noexcept
{
  return status == PlanningStatus::kSuccess;
}

int PlanningResult::exit_code() const noexcept
{
  return static_cast<int>(status);
}

const char * to_string(PlanningStatus status) noexcept
{
  switch (status) {
    case PlanningStatus::kSuccess:
      return "success";
    case PlanningStatus::kServerUnavailable:
      return "server_unavailable";
    case PlanningStatus::kRobotStateIncomplete:
      return "robot_state_incomplete";
    case PlanningStatus::kRobotStateStale:
      return "robot_state_stale";
    case PlanningStatus::kModelContractMismatch:
      return "model_contract_mismatch";
    case PlanningStatus::kStartStateInvalid:
      return "start_state_invalid";
    case PlanningStatus::kPlanningRejected:
      return "planning_rejected";
    case PlanningStatus::kPlanningTimedOut:
      return "planning_timed_out";
    case PlanningStatus::kInvalidTrajectory:
      return "invalid_trajectory";
    case PlanningStatus::kSceneSynchronizationTimedOut:
      return "scene_synchronization_timed_out";
    case PlanningStatus::kExecutionRejected:
      return "execution_rejected";
    case PlanningStatus::kExecutionTimedOut:
      return "execution_timed_out";
    case PlanningStatus::kPostExecutionMismatch:
      return "post_execution_mismatch";
    case PlanningStatus::kExpectedCollisionMissing:
      return "expected_collision_missing";
  }
  return "unknown";
}

const char * moveit_error_name(int moveit_error_code) noexcept
{
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  switch (moveit_error_code) {
    case ErrorCodes::SUCCESS: return "SUCCESS";
    case ErrorCodes::UNDEFINED: return "UNDEFINED";
    case ErrorCodes::FAILURE: return "FAILURE";
    case ErrorCodes::PLANNING_FAILED: return "PLANNING_FAILED";
    case ErrorCodes::INVALID_MOTION_PLAN: return "INVALID_MOTION_PLAN";
    case ErrorCodes::MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
      return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
    case ErrorCodes::CONTROL_FAILED: return "CONTROL_FAILED";
    case ErrorCodes::UNABLE_TO_AQUIRE_SENSOR_DATA: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
    case ErrorCodes::TIMED_OUT: return "TIMED_OUT";
    case ErrorCodes::PREEMPTED: return "PREEMPTED";
    case ErrorCodes::START_STATE_IN_COLLISION: return "START_STATE_IN_COLLISION";
    case ErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS:
      return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
    case ErrorCodes::START_STATE_INVALID: return "START_STATE_INVALID";
    case ErrorCodes::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case ErrorCodes::GOAL_VIOLATES_PATH_CONSTRAINTS: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
    case ErrorCodes::GOAL_CONSTRAINTS_VIOLATED: return "GOAL_CONSTRAINTS_VIOLATED";
    case ErrorCodes::GOAL_STATE_INVALID: return "GOAL_STATE_INVALID";
    case ErrorCodes::UNRECOGNIZED_GOAL_TYPE: return "UNRECOGNIZED_GOAL_TYPE";
    case ErrorCodes::INVALID_GROUP_NAME: return "INVALID_GROUP_NAME";
    case ErrorCodes::INVALID_GOAL_CONSTRAINTS: return "INVALID_GOAL_CONSTRAINTS";
    case ErrorCodes::INVALID_ROBOT_STATE: return "INVALID_ROBOT_STATE";
    case ErrorCodes::INVALID_LINK_NAME: return "INVALID_LINK_NAME";
    case ErrorCodes::INVALID_OBJECT_NAME: return "INVALID_OBJECT_NAME";
    case ErrorCodes::FRAME_TRANSFORM_FAILURE: return "FRAME_TRANSFORM_FAILURE";
    case ErrorCodes::COLLISION_CHECKING_UNAVAILABLE: return "COLLISION_CHECKING_UNAVAILABLE";
    case ErrorCodes::ROBOT_STATE_STALE: return "ROBOT_STATE_STALE";
    case ErrorCodes::SENSOR_INFO_STALE: return "SENSOR_INFO_STALE";
    case ErrorCodes::COMMUNICATION_FAILURE: return "COMMUNICATION_FAILURE";
    case ErrorCodes::CRASH: return "CRASH";
    case ErrorCodes::ABORT: return "ABORT";
    case ErrorCodes::NO_IK_SOLUTION: return "NO_IK_SOLUTION";
    default: break;
  }
  return "an error code MoveIt does not name";
}

bool execution_failure_reached_terminal_stop(int moveit_error_code) noexcept
{
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  return moveit_error_code == ErrorCodes::CONTROL_FAILED;
}

PlanningResult classify_moveit_error(
  PlanningPhase phase, int moveit_error_code, const std::string & context)
{
  using ErrorCodes = moveit_msgs::msg::MoveItErrorCodes;
  if (moveit_error_code == ErrorCodes::SUCCESS) {
    return {PlanningStatus::kSuccess, with_context(context, "MoveIt reported success")};
  }
  if (moveit_error_code == ErrorCodes::TIMED_OUT) {
    const auto status = phase == PlanningPhase::kPlan ? PlanningStatus::kPlanningTimedOut :
      PlanningStatus::kExecutionTimedOut;
    return failure(status, with_context(context, "MoveIt timed out"));
  }
  if (
    moveit_error_code == ErrorCodes::COMMUNICATION_FAILURE ||
    moveit_error_code == ErrorCodes::CRASH)
  {
    return failure(
      PlanningStatus::kServerUnavailable,
      with_context(context, "MoveIt communication endpoint is unavailable"));
  }
  if (
    moveit_error_code == ErrorCodes::START_STATE_IN_COLLISION ||
    moveit_error_code == ErrorCodes::START_STATE_VIOLATES_PATH_CONSTRAINTS ||
    moveit_error_code == ErrorCodes::START_STATE_INVALID ||
    moveit_error_code == ErrorCodes::INVALID_ROBOT_STATE)
  {
    return failure(
      PlanningStatus::kStartStateInvalid, with_context(context, "MoveIt rejected start state"));
  }
  if (phase == PlanningPhase::kExecute) {
    return failure(
      PlanningStatus::kExecutionRejected,
      with_context(context, "trajectory execution was rejected or aborted"));
  }
  return failure(
    PlanningStatus::kPlanningRejected,
    with_context(context, "motion planning did not produce a valid solution"));
}

std::string describe_start_state_condition(const StartStateCondition & condition)
{
  if (!condition.established) {
    return "start-state condition not established (" + condition.unestablished + ")";
  }
  if (!condition.in_collision) {
    return "start state is valid in the current planning scene";
  }
  if (condition.contacts.empty()) {
    return "start state was rejected by the planning scene, which named no contact";
  }
  std::ostringstream stream;
  stream << "start state is in collision: " << condition.contacts.front();
  for (std::size_t index = 1U; index < condition.contacts.size(); ++index) {
    stream << ", " << condition.contacts[index];
  }
  if (condition.suppressed_contacts > 0U) {
    stream << " and " << condition.suppressed_contacts << " more";
  }
  return stream.str();
}

PlanningResult classify_plan_failure(
  int moveit_error_code, const StartStateCondition & start_state, const PlanningSlice & slice,
  const std::string & context)
{
  PlanningResult result = classify_moveit_error(PlanningPhase::kPlan, moveit_error_code, context);
  if (result.status != PlanningStatus::kPlanningRejected) {
    // A timeout, a server MoveIt itself could not reach, or a start-state code MoveIt named: the
    // refinement exists only for the generic codes that cannot speak for themselves.
    return result;
  }
  if (start_state.established && start_state.in_collision) {
    return failure(
      PlanningStatus::kStartStateInvalid, with_context(
        context, describe_start_state_condition(start_state)));
  }
  const std::string start_state_clause = "; " + describe_start_state_condition(start_state);
  if (slice.exhausted()) {
    std::ostringstream stream;
    stream << "the planning attempt consumed its slice (" << std::fixed << std::setprecision(3)
           << slice.attempted_seconds << " s of " << slice.budget_seconds << " s)";
    return failure(
      PlanningStatus::kPlanningTimedOut, with_context(context, stream.str()) +
      start_state_clause);
  }
  // The pose is still what refused, but the receipt says the start state was ruled out (or that
  // nothing could be established about it) so one line separates the causes.
  return failure(result.status, result.detail + start_state_clause);
}

bool PlanningSlice::exhausted() const noexcept
{
  return budget_seconds > 0.0 && attempted_seconds >= kPlanningSliceExhaustedFraction *
         budget_seconds;
}

std::optional<ControllerAbortReport> parse_controller_abort(
  const std::string & logger_name, const std::string & log_message)
{
  // joint_trajectory_controller prefixes every abort it decides with this and nothing else.
  static constexpr std::string_view kPrefix{"Aborted due to "};
  if (logger_name.empty() || !log_message.starts_with(kPrefix)) {
    return std::nullopt;
  }
  ControllerAbortReport report;
  report.controller = logger_name;
  report.reported = log_message;
  // The controller sets its FollowJointTrajectory error code in the same branch that logs these
  // sentences, so the mapping follows that branch: the path-tolerance branch sets
  // PATH_TOLERANCE_VIOLATED and logs "state tolerance violation" while writing "path tolerance
  // violation" into its result string, and the goal-time branch sets GOAL_TOLERANCE_VIOLATED. Both
  // were confirmed against the campaign console, where MoveIt reported the matching name 35 ms
  // after each line. Any other abort (a command timeout, a sentence from a later release) is quoted
  // and left unnamed.
  if (log_message.starts_with("Aborted due to state tolerance violation") ||
    log_message.starts_with("Aborted due to path tolerance violation"))
  {
    report.error = "PATH_TOLERANCE_VIOLATED";
  } else if (log_message.starts_with("Aborted due to goal_time_tolerance exceeding")) {
    report.error = "GOAL_TOLERANCE_VIOLATED";
  }
  return report;
}

std::string describe_controller_abort(
  const ControllerAbortReport & report, const SegmentVelocityEvidence & evidence)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(4);
  stream << "controller '" << report.controller << "' aborted";
  if (!report.error.empty()) {
    stream << " with " << report.error;
  }
  stream << ", reporting \"" << report.reported << "\"";
  if (evidence.samples == 0U) {
    stream << "; no joint-state sample covered this segment, so its velocity limits were not "
      "checked";
    return stream.str();
  }
  if (evidence.joint.empty()) {
    stream << "; no joint's feedback velocity was observed at its URDF velocity limit in the "
           << evidence.samples << " joint-state samples covering this segment";
    // Say how close it came: an absence alone cannot distinguish ordinary tracking lag from an
    // impulse that stopped short of the bound, and the margined planning limits leave a wide band
    // for the latter. The clause avoids the phrase the benchmark taxonomy uses for the saturated
    // row, so a near miss stays a near miss.
    if (!evidence.nearest_joint.empty() && evidence.nearest_limit > 0.0) {
      stream << "; the nearest was " << evidence.nearest_joint << " at " << evidence.nearest_peak
             << " rad/s, " << (evidence.nearest_peak / evidence.nearest_limit)
             << " of its URDF bound of " << evidence.nearest_limit << " rad/s";
      if (evidence.nearest_planned >= 0.0) {
        stream << ", against a planned maximum of " << evidence.nearest_planned << " rad/s";
      }
    }
    return stream.str();
  }
  stream << "; " << evidence.joint << " feedback velocity reached its URDF velocity limit of "
         << evidence.urdf_limit << " rad/s during this segment";
  if (evidence.planned_maximum >= 0.0) {
    stream << ", which planned that joint no faster than " << evidence.planned_maximum
           << " rad/s";
  }
  return stream.str();
}

PlanningResult validate_model_contract(
  const std::string & planning_frame, const std::string & end_effector_link,
  const std::vector<std::string> & active_joints)
{
  if (planning_frame != "world") {
    return failure(
      PlanningStatus::kModelContractMismatch,
      "planning frame is '" + planning_frame + "', expected 'world'");
  }
  if (end_effector_link != "tool0") {
    return failure(
      PlanningStatus::kModelContractMismatch,
      "end-effector link is '" + end_effector_link + "', expected 'tool0'");
  }
  if (active_joints != kExpectedActiveJoints) {
    return failure(
      PlanningStatus::kModelContractMismatch,
      "manipulator active-joint order does not match the rail-plus-arm contract");
  }
  return {PlanningStatus::kSuccess, "MoveIt robot model matches the planning contract"};
}

PlanningResult validate_joint_state(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  double age_sec, double max_age_sec)
{
  if (names.size() != positions.size()) {
    return failure(
      PlanningStatus::kRobotStateIncomplete,
      "joint-state names and positions have different sizes");
  }
  if (!std::isfinite(age_sec) || !std::isfinite(max_age_sec) || max_age_sec <= 0.0) {
    return failure(PlanningStatus::kRobotStateStale, "joint-state freshness input is invalid");
  }
  if (age_sec < 0.0 || age_sec > max_age_sec) {
    std::ostringstream stream;
    stream << "joint-state age " << age_sec << " s exceeds allowed age " << max_age_sec << " s";
    return failure(PlanningStatus::kRobotStateStale, stream.str());
  }

  std::set<std::string> observed_names;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (!observed_names.insert(names[index]).second) {
      return failure(
        PlanningStatus::kRobotStateIncomplete,
        "joint state contains duplicate joint " + names[index]);
    }
    if (!std::isfinite(positions[index])) {
      return failure(
        PlanningStatus::kRobotStateIncomplete,
        "joint state contains non-finite position for " + names[index]);
    }
  }
  const std::set<std::string> expected(kExpectedActiveJoints.begin(), kExpectedActiveJoints.end());
  if (!std::includes(
      observed_names.begin(), observed_names.end(), expected.begin(),
      expected.end()))
  {
    return failure(
      PlanningStatus::kRobotStateIncomplete, "joint state does not contain all manipulator joints");
  }
  return {PlanningStatus::kSuccess, "joint state is complete, finite, and fresh"};
}

PlanningResult validate_joint_target(
  const std::vector<std::string> & names, const std::vector<double> & positions,
  const std::vector<double> & target_positions, double tolerance)
{
  if (names.size() != positions.size() || target_positions.size() != kExpectedActiveJoints.size()) {
    return failure(
      PlanningStatus::kPostExecutionMismatch, "post-execution joint vectors have invalid sizes");
  }
  if (!std::isfinite(tolerance) || tolerance <= 0.0) {
    return failure(
      PlanningStatus::kPostExecutionMismatch, "post-execution joint tolerance is invalid");
  }

  for (std::size_t target_index = 0; target_index < kExpectedActiveJoints.size(); ++target_index) {
    const auto found = std::find(
      names.begin(), names.end(), kExpectedActiveJoints[target_index]);
    if (found == names.end()) {
      return failure(
        PlanningStatus::kPostExecutionMismatch,
        "post-execution state is missing " + kExpectedActiveJoints[target_index]);
    }
    const auto position_index = static_cast<std::size_t>(std::distance(names.begin(), found));
    const double error = std::abs(positions[position_index] - target_positions[target_index]);
    if (!std::isfinite(error) || error > tolerance) {
      std::ostringstream stream;
      stream << kExpectedActiveJoints[target_index] << " post-execution error " << error
             << " exceeds tolerance " << tolerance;
      return failure(PlanningStatus::kPostExecutionMismatch, stream.str());
    }
  }
  return {PlanningStatus::kSuccess, "post-execution joints match the commanded target"};
}

PlanningResult validate_pose_error(
  double translation_error_m, double rotation_error_rad, double translation_tolerance_m,
  double rotation_tolerance_rad)
{
  const bool inputs_are_valid =
    std::isfinite(translation_error_m) && std::isfinite(rotation_error_rad) &&
    std::isfinite(translation_tolerance_m) && std::isfinite(rotation_tolerance_rad) &&
    translation_error_m >= 0.0 && rotation_error_rad >= 0.0 &&
    translation_tolerance_m > 0.0 && rotation_tolerance_rad > 0.0;
  if (!inputs_are_valid) {
    return failure(PlanningStatus::kPostExecutionMismatch, "pose verification input is invalid");
  }
  if (
    translation_error_m > translation_tolerance_m ||
    rotation_error_rad > rotation_tolerance_rad)
  {
    std::ostringstream stream;
    stream << "tool pose errors are " << translation_error_m << " m and " << rotation_error_rad
           << " rad; tolerances are " << translation_tolerance_m << " m and "
           << rotation_tolerance_rad << " rad";
    return failure(PlanningStatus::kPostExecutionMismatch, stream.str());
  }
  return {PlanningStatus::kSuccess, "post-execution tool pose is within tolerance"};
}

PlanningResult validate_trajectory(const TrajectoryObservation & trajectory)
{
  if (trajectory.joint_names != kExpectedActiveJoints) {
    return failure(
      PlanningStatus::kInvalidTrajectory,
      "trajectory joint order does not cover the exact manipulator contract");
  }
  if (trajectory.points.empty()) {
    return failure(PlanningStatus::kInvalidTrajectory, "trajectory has no points");
  }

  double previous_time = -1.0;
  for (const auto & point : trajectory.points) {
    if (point.positions.size() != trajectory.joint_names.size()) {
      return failure(
        PlanningStatus::kInvalidTrajectory,
        "trajectory point does not contain every joint position");
    }
    if (!std::all_of(
        point.positions.begin(), point.positions.end(), [](double value) {
          return std::isfinite(value);
        }))
    {
      return failure(PlanningStatus::kInvalidTrajectory, "trajectory contains non-finite position");
    }
    if (!std::isfinite(point.time_from_start_sec) || point.time_from_start_sec < 0.0 ||
      point.time_from_start_sec <= previous_time)
    {
      return failure(
        PlanningStatus::kInvalidTrajectory,
        "trajectory timestamps must be finite, nonnegative, and strictly increasing");
    }
    previous_time = point.time_from_start_sec;
  }
  return {PlanningStatus::kSuccess, "trajectory is complete, finite, and time ordered"};
}

PlanningResult validate_trajectory_timing(
  const TrajectoryObservation & trajectory, const std::string & context)
{
  if (trajectory.points.empty()) {
    return failure(
      PlanningStatus::kInvalidTrajectory,
      with_context(context, "MoveIt returned a solution with no trajectory points"));
  }
  const bool untimed = std::all_of(
    trajectory.points.begin(), trajectory.points.end(), [&trajectory](const auto & point) {
      return point.time_from_start_sec == trajectory.points.front().time_from_start_sec;
    });
  if (untimed && trajectory.points.size() > 1U) {
    std::ostringstream stream;
    stream << "MoveIt returned " << trajectory.points.size()
           << " waypoints that all share one timestamp, which is what its Cartesian path service "
      "answers when its own time parameterisation failed";
    return failure(PlanningStatus::kInvalidTrajectory, with_context(context, stream.str()));
  }
  double previous_time = -1.0;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const double time = trajectory.points[index].time_from_start_sec;
    if (!std::isfinite(time) || time < 0.0 || time <= previous_time) {
      std::ostringstream stream;
      stream << "waypoint " << index << " carries time_from_start " << time
             << " s, which is not finite, nonnegative and strictly after " << previous_time
             << " s";
      return failure(PlanningStatus::kInvalidTrajectory, with_context(context, stream.str()));
    }
    previous_time = time;
  }
  return {PlanningStatus::kSuccess, with_context(context, "trajectory timing is executable")};
}

std::optional<std::vector<TrajectoryExecutionChunk>> controller_execution_chunks(
  const TrajectoryObservation & trajectory, std::chrono::milliseconds maximum_duration)
{
  if (maximum_duration <= std::chrono::milliseconds::zero() ||
    !validate_trajectory_timing(trajectory, "controller execution").ok())
  {
    return std::nullopt;
  }
  if (trajectory.points.size() == 1U) {
    return std::vector<TrajectoryExecutionChunk>{{0U, 0U}};
  }

  const double maximum_seconds =
    std::chrono::duration<double>(maximum_duration).count();
  std::vector<TrajectoryExecutionChunk> chunks;
  std::size_t first = 0U;
  while (first + 1U < trajectory.points.size()) {
    std::size_t last = first;
    while (last + 1U < trajectory.points.size() &&
      trajectory.points[last + 1U].time_from_start_sec -
      trajectory.points[first].time_from_start_sec <= maximum_seconds)
    {
      ++last;
    }
    if (last == first) {
      return std::nullopt;
    }
    chunks.push_back(TrajectoryExecutionChunk{first, last});
    first = last;
  }
  return chunks;
}

}  // namespace restocker_task_executor
