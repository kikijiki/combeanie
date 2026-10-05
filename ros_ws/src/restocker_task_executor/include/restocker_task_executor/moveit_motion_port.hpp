// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <restocker_interfaces/msg/planning_scene_projection_status.hpp>
#include <restocker_interfaces/srv/get_planning_scene_projection_status.hpp>
#include <restocker_interfaces/srv/validate_planning_scene_lease.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "restocker_task_executor/free_space_race.hpp"
#include "restocker_task_executor/motion_port.hpp"
#include "restocker_task_executor/planning_contract.hpp"
#include "restocker_task_executor/pregrasp_planning_authority.hpp"

namespace moveit
{
namespace planning_interface
{
class MoveGroupInterface;
}  // namespace planning_interface
}  // namespace moveit

namespace restocker_task_executor
{

struct MoveItMotionPortConfig
{
  std::string planning_group{"manipulator"};
  std::string end_effector_link{"tool0"};
  std::string planning_frame{"world"};
  std::string planning_pipeline{"ompl"};
  std::string planner_id{"RRTConnectkConfigDefault"};
  std::vector<std::string> gripper_joint_names{"left_finger_joint", "right_finger_joint"};
  // Controllers MoveIt may drive for this group. The gripper is deliberately excluded: it is
  // commanded separately through its own joint-trajectory action.
  std::vector<std::string> execution_controllers{"arm_controller", "rail_controller"};
  std::chrono::milliseconds startup_timeout{std::chrono::seconds(30)};
  // Upper bound on one plan-and-execute segment before the port reports kTimedOut.
  std::chrono::milliseconds segment_timeout{std::chrono::seconds(60)};
  // A current-state read made with no wait races a simulated clock: the requested timestamp can
  // advance between reading `now` and receiving the next joint sample. Wait briefly for that
  // sample instead of declaring a healthy motion backend unavailable.
  std::chrono::milliseconds current_state_timeout{std::chrono::seconds(1)};
  // Bounds diagnostic reads and the complete scene fetch required to revalidate a controller
  // goal after collision content changes. Revalidation additionally shares the scene-settle
  // deadline and cancellation boundary; an incomplete read cannot authorize execution.
  std::chrono::milliseconds state_validity_timeout{std::chrono::seconds(2)};
  // MoveIt's reduced joint limits can reject a freshly measured start state that landed only a
  // few floating-point ulps outside the boundary after a successful prior motion. Clamp at most
  // this much; a larger mismatch remains a hard planning failure. This stays far inside both the
  // URDF-to-planning-limit margin and the controller's allowed start tolerance.
  double maximum_start_state_bounds_correction{0.001};
  // When true, every segment is gated on the planning-scene projection: the projector must be in
  // applied, error-free state (or a validated held lease) when planning and immediately before
  // each execute(). Changed content requires fresh collision validation of that next goal;
  // epoch/lease/freshness checks remain mandatory and original provenance never changes.
  // False only for a deployment that runs no projector at all, where there is no scene authority
  // to consult and the port would otherwise refuse every segment.
  bool require_planning_scene_authority{true};
  std::string planning_scene_status_topic{"/planning_scene_projection/status"};
  std::string planning_scene_status_service{"/planning_scene_projection/get_status"};
  std::string planning_scene_lease_validation_service{
    "/planning_scene_projection/validate_lease"};
  // A capability check is a local service round-trip to the projector. It is deliberately much
  // shorter than a segment: timing out means the frozen scene cannot be authorized.
  std::chrono::milliseconds planning_scene_lease_validation_timeout{
    std::chrono::seconds(1)};
  // Oldest projection status the gate will act on. Beyond this the projector is not reporting,
  // which is itself a reason not to execute.
  std::chrono::milliseconds maximum_scene_status_age{std::chrono::milliseconds(2500)};
  std::chrono::milliseconds maximum_scene_status_future_skew{std::chrono::milliseconds(50)};
  // The projector republishes STATE_SYNCHRONIZING at the start of every reconciliation cycle, so
  // a single reading lands mid-cycle often enough to matter. Synchronizing is not a refusal,
  // the authority says "wait", not "no", and this is how long the port waits for the cycle to
  // settle before treating the silence as a refusal. It must stay well inside segment_timeout.
  std::chrono::milliseconds scene_authority_settle_timeout{std::chrono::seconds(5)};
  // Where the controller's own abort reason arrives. MoveIt's controller manager reads
  // FollowJointTrajectory's error code, logs it, and then reports plain CONTROL_FAILED to
  // whoever asked for the execution, so the log is the only place that reason survives into
  // another process.
  std::string controller_log_topic{"/rosout"};
  // How long to wait after CONTROL_FAILED for that line to arrive. It is published before MoveIt
  // answers, so this covers transport only; expiring means the reason is reported as not
  // established rather than guessed at.
  std::chrono::milliseconds controller_abort_report_timeout{std::chrono::milliseconds(750)};
  // Feedback for the velocity-limit reading that separates the recorded impulse from ordinary
  // tracking lag. Nothing gates a motion on it.
  std::string joint_state_topic{"/joint_states"};
};

// A MotionPort backed by MoveIt's MoveGroupInterface.
//
// MoveGroupInterface is blocking and needs a spinning node, so this port owns a private node, a
// single-threaded executor thread for it, and one worker thread that runs plan-and-execute. That
// keeps every blocking MoveIt call off the coordinator's callback groups, which is the same
// arrangement planning_smoke_test uses.
//
// The MoveIt robot model is read from this port's own node, so its NodeOptions must carry
// robot_description, robot_description_semantic, robot_description_kinematics and joint limits.
class MoveItMotionPort final : public MotionPort
{
public:
  MoveItMotionPort(
    const rclcpp::NodeOptions & options, MoveItMotionPortConfig config = {},
    std::string node_name = "restock_motion_client");
  ~MoveItMotionPort() override;

  MoveItMotionPort(const MoveItMotionPort &) = delete;
  MoveItMotionPort & operator=(const MoveItMotionPort &) = delete;
  MoveItMotionPort(MoveItMotionPort &&) = delete;
  MoveItMotionPort & operator=(MoveItMotionPort &&) = delete;

  [[nodiscard]] bool ready() const override;

  [[nodiscard]] MotionSubmitResult submit(
    OperationCorrelation correlation, MotionGoal goal, CompletionCallback callback) override;

  void cancel() noexcept override;

  // Stop the worker and release MoveIt. Idempotent; the destructor calls it.
  void shutdown() noexcept;

private:
  struct PendingGoal
  {
    OperationCorrelation correlation;
    MotionGoal goal;
    CompletionCallback callback;
  };

  // Outcome of one free-space planning attempt. `planned` is true when `trajectory` holds an
  // accepted plan; otherwise `failure` carries the refusal, including whether the planner
  // refused the goal itself (which is what spends a grasp candidate).
  struct FreeSpacePlanResult
  {
    bool planned{false};
    MotionCompletion failure;
  };

  class MoveGroupHolder;

  // Card 053 / Milestone 10 §6: the in-process race of the ompl (RRTConnect) and ompl_fallback
  // (PRM) pipelines inside one unchanged free-space slice. Built lazily on the first
  // free-space plan; stays null (and planning falls back to the move_group service) when this
  // node does not carry both pipeline parameter namespaces.
  class RacePipelines;
  struct RaceRun
  {
    RaceDecision decision;
    moveit_msgs::msg::RobotTrajectory trajectory;
  };
  std::unique_ptr<RacePipelines> race_pipelines_;
  bool race_unavailable_{false};

  void run_worker() noexcept;
  // Runs on the worker thread. Never throws; every failure maps to a MotionOutcome.
  [[nodiscard]] MotionCompletion execute_goal(const PendingGoal & pending) noexcept;
  // Plans one free-space segment: optional path constraints, continuation-endpoint selection and
  // the budget split across the goal's free-space candidates. Fills `trajectory` on success and
  // returns why not otherwise, without ever commanding anything.
  [[nodiscard]] FreeSpacePlanResult plan_free_space(
    const PendingGoal & pending, moveit::planning_interface::MoveGroupInterface * interface,
    const moveit::core::RobotState & current_state,
    const moveit::core::JointModelGroup * planning_group,
    const moveit::core::RobotModelConstPtr & robot_model,
    const geometry_msgs::msg::Pose & target, moveit_msgs::msg::RobotTrajectory & trajectory);
  // Milestone 10 §6 (Card 062): the grasp escape's straight line, planned in a local copy of
  // move_group's scene in which only the finger–target contact pairs the start state actually
  // has are allowed. Refuses (without commanding anything) on any other contact, an incomplete
  // line, or an endpoint still in contact under the unmodified matrix; `detail` says which, or
  // on success carries the receipt.
  struct GraspEscapePlan
  {
    bool planned{false};
    std::string detail;
  };
  [[nodiscard]] GraspEscapePlan plan_grasp_escape(
    const PendingGoal & pending, const moveit::core::RobotState & current_state,
    const moveit::core::JointModelGroup * planning_group,
    moveit_msgs::msg::RobotTrajectory & trajectory);
  // Loads the ompl + ompl_fallback pipelines once (Card 053). False when the node lacks the
  // pipeline parameters or a pipeline fails to construct; the caller then plans through the
  // move_group service exactly as before.
  [[nodiscard]] bool ensure_race_pipelines(const moveit::core::RobotModelConstPtr & robot_model);
  // Races both pipelines inside `slice_seconds` against one snapshot of move_group's scene
  // (the same /get_planning_scene the refusal receipts use), first solution stops the race.
  // Nullopt when the race cannot run here (scene snapshot unavailable, pipeline error) — the
  // caller must fall back to the move_group service for this slice.
  [[nodiscard]] std::optional<RaceRun> race_plan(
    const moveit_msgs::msg::MotionPlanRequest & base_request, double slice_seconds,
    const moveit::core::RobotModelConstPtr & robot_model);
  // Says what happened to a linear path that did not reach its goal, in terms of what MoveIt
  // reported and what a second, collision-free interpolation and /check_state_validity establish.
  // It never names a cause that none of those three established.
  [[nodiscard]] std::string describe_truncated_linear_path(
    MoveGroupHolder & holder, const MotionGoal & goal,
    const geometry_msgs::msg::Pose & target,
    const moveit_msgs::msg::RobotTrajectory & achieved, double fraction,
    const moveit_msgs::msg::MoveItErrorCodes & error_code);
  // Asks MoveIt whether one interpolated configuration is valid, and names the contacts it
  // reports. Returns what could be established, including that nothing could be.
  [[nodiscard]] std::string describe_state_validity(
    const trajectory_msgs::msg::JointTrajectory & path, std::size_t point_index);
  // What the scene says about the state a failed plan would have started from. Established only
  // on a failure path, so the green path never pays for it, and an answer that does not arrive
  // inside state_validity_timeout is returned as unestablished rather than as a cause.
  [[nodiscard]] StartStateCondition establish_start_state_condition(
    const moveit::core::RobotState & start_state);
  // How many world objects MoveIt's scene carried, and which of them are projected obstacles.
  // Receipt only: it is read when a plan is refused so one log line separates "the budget ran
  // out" from "the scene refused", and never gates anything.
  [[nodiscard]] std::string describe_scene_world_objects();
  // Reads the projection status until it says something conclusive, treating "wait" and a status
  // that has not arrived yet as transient rather than as a refusal.
  [[nodiscard]] PlanningSceneAuthorityResult settled_scene_authority(
    const std::optional<PreGraspSceneAuthority> & baseline,
    const std::optional<PlanningSceneLeaseAuthority> & lease, std::string & projector_detail,
    std::chrono::steady_clock::time_point outer_deadline =
    std::chrono::steady_clock::time_point::max());
  // Validates the secret capability with the projector and returns only its public lease facts.
  // Empty tokens need no lease authority and return an empty optional without an error.
  [[nodiscard]] std::optional<PlanningSceneLeaseAuthority> validate_scene_lease(
    const std::string & token, std::string & detail,
    std::chrono::steady_clock::time_point deadline = std::chrono::steady_clock::time_point::max());
  // Establishes the planning-scene authority a plan will be made against, or says why it cannot
  // be established. Returns nullopt with `detail` set on refusal.
  [[nodiscard]] std::optional<PreGraspSceneAuthority> establish_scene_authority(
    const std::string & lease_token, std::string & detail);
  // Re-checks that authority immediately before execute(). Empty on success; otherwise the reason
  // the segment must not be executed.
  [[nodiscard]] std::string scene_authority_lost(
    const PreGraspSceneAuthority & baseline, const std::string & lease_token,
    const moveit_msgs::msg::RobotTrajectory & next_goal,
    const moveit::core::RobotModelConstPtr & robot_model, bool may_revalidate);
  [[nodiscard]] PreGraspPlanningAuthorityConfig scene_authority_config() const;
  // Clears the per-segment observations so what is read after execute() describes this segment
  // and no earlier one.
  void begin_segment_observation();
  // The controller's abort line for the segment just executed, waited for briefly. Empty when
  // none arrived, which is reported rather than filled in.
  [[nodiscard]] std::optional<ControllerAbortReport> observed_controller_abort();
  // What the executed trajectory's own joints did against the URDF velocity bounds the simulator
  // enforces. Measures; it does not diagnose.
  [[nodiscard]] SegmentVelocityEvidence segment_velocity_evidence(
    MoveGroupHolder & holder, const moveit_msgs::msg::RobotTrajectory & trajectory);

  MoveItMotionPortConfig config_;
  std::shared_ptr<rclcpp::Node> node_;
  // One client per configured execution controller, created eagerly and used only to answer
  // ready(). MoveIt owns the trajectory it sends, but it aborts outright when its own controller
  // clients have not discovered the controllers yet, and an aborted execution is terminal for the
  // task, so the port has to be able to say when that discovery has happened.
  std::vector<rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr>
  execution_clients_;
  // Used to say what refused a plan: the start-state condition behind a failed plan or a
  // truncated linear path, and the contacts of one interpolated configuration. Only ever read on
  // a path that has already failed, so it never gates a motion.
  rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr state_validity_client_;
  // Complete collision scene for planner races, refusal diagnostics and changed-scene
  // revalidation immediately before the next controller goal.
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr planning_scene_client_;
  rclcpp::Client<restocker_interfaces::srv::GetPlanningSceneProjectionStatus>::SharedPtr
    scene_status_client_;
  rclcpp::Subscription<restocker_interfaces::msg::PlanningSceneProjectionStatus>::SharedPtr
    scene_status_subscription_;
  rclcpp::Client<restocker_interfaces::srv::ValidatePlanningSceneLease>::SharedPtr
    scene_lease_validation_client_;
  // Written by the executor thread, read by the worker thread, so it is guarded. The value is an
  // immutable message used to establish planning authority. Execution uses causal RPC reads,
  // because delayed topic delivery cannot bracket a scene mutation.
  mutable std::mutex scene_status_mutex_;
  restocker_interfaces::msg::PlanningSceneProjectionStatus::ConstSharedPtr scene_status_;
  // The last controller abort line seen, cleared at the start of every segment. Written by the
  // executor thread and read by the worker, so it is guarded.
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr controller_log_subscription_;
  mutable std::mutex controller_abort_mutex_;
  std::optional<ControllerAbortReport> controller_abort_;
  // Peak absolute feedback speed per joint since the current segment started, and how many
  // samples that peak was taken over. A running maximum rather than a trace: the question this
  // answers is whether any joint reached a bound, which a peak settles and a buffer of history
  // would only restate at more cost.
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  mutable std::mutex joint_speed_mutex_;
  std::unordered_map<std::string, double> joint_peak_speed_;
  std::size_t joint_state_samples_{0};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;

  std::unique_ptr<MoveGroupHolder> move_group_;
  std::mutex move_group_mutex_;

  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::optional<PendingGoal> pending_;
  bool busy_{false};
  std::atomic<bool> stopping_{false};
  std::atomic<bool> cancel_requested_{false};
  // Set when MoveIt execute() is called for the goal being run (any slice); copied onto the
  // completion as submitted_to_backend (Card 086 stage 1b review B1).
  std::atomic<bool> execute_called_{false};
  std::thread worker_;
};

}  // namespace restocker_task_executor
