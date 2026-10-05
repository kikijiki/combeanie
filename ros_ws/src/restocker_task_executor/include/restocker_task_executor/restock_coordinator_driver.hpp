// Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
//
// SPDX-License-Identifier: MIT

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <restocker_reasoner/recovery_advisor_port.hpp>
#include <restocker_reasoner/recovery_audit_log.hpp>

#include "restocker_task_executor/coordinator_driver_output.hpp"
#include "restocker_task_executor/attachment_port.hpp"
#include "restocker_task_executor/gripper_port.hpp"
#include "restocker_task_executor/motion_port.hpp"
#include "restocker_task_executor/coordinator_termination_router.hpp"
#include "restocker_task_executor/restock_action_contract.hpp"
#include "restocker_task_executor/recovery_advice_gate.hpp"
#include "restocker_task_executor/recovery_classification.hpp"
#include "restocker_task_executor/restock_coordinator_inbox.hpp"
#include "restocker_task_executor/restock_goal_context.hpp"
#include "restocker_task_executor/world_state_async_port.hpp"

namespace restocker_task_executor
{

struct RestockCoordinatorDriverConfig
{
  RestockTaskConfig task;
  ReconciliationPolicy reconciliation;
  std::size_t inbox_capacity{128U};
  std::string planning_frame{"world"};
  std::chrono::nanoseconds maximum_object_age{std::chrono::milliseconds(500)};
  std::chrono::nanoseconds maximum_robot_age{std::chrono::milliseconds(500)};
  std::chrono::nanoseconds maximum_future_skew{std::chrono::milliseconds(50)};
  // Largest joint speed a freshly observed robot may still carry and be called stopped, in rad/s
  // for the revolute joints and m/s for the rail. Recovery replans from the current state, so it
  // must first see a robot that is not still moving away from it. Held position under a joint
  // trajectory controller settles far below this; anything near a commanded speed is not at rest.
  double recovery_rest_speed{0.05};
  // Downward collision probe for pre-grasp continuation selection. The executed grasp remains at
  // its generated height; this only rejects tray-skimming redundant IK branches.
  double pregrasp_continuation_vertical_margin_m{0.005};
  // Minimum confidence an advisory recommendation must have to be considered. Validated at
  // startup to lie in [0, 1]; unused when no advisory backend is composed.
  double reasoner_minimum_confidence{0.5};
  // Bounded observable reacquire when the wrist perception stream is unlive at a
  // planned-segment request. The liveness horizon itself is never widened: the unchanged
  // predicate must pass again before the segment is submitted, and expiry of this steady-clock
  // budget takes the same terminal refusal as an immediate failure. Must fit inside a plan
  // state's command deadline (planning + execution) so a restored stream can still submit.
  // Zero skips the hold and refuses immediately.
  std::chrono::milliseconds perception_reacquire_timeout{5000};
  // Bounded stop-settle hold before recovery refuses a stop it has not established (Milestone 10
  // §6, Card 060). Recovery's fresh snapshot must show fresh robot telemetry at rest; when it does
  // not, the driver re-requests a snapshot every recovery_stop_settle_poll until the unchanged
  // predicate holds or this steady-clock budget is spent, and expiry fails the recovery exactly as
  // an immediate refusal would (UNSAFE). Must be shorter than task.recovery_timeout. Zero skips
  // the hold and refuses at once.
  std::chrono::milliseconds recovery_stop_settle_timeout{2000};
  std::chrono::milliseconds recovery_stop_settle_poll{100};
  // Milestone 10 §6 (Card 062): the only links whose contact with the target product the grasp
  // escape may tolerate. Every other link stays forbidden.
  std::vector<std::string> grasp_escape_finger_links{"left_finger", "right_finger"};
};

// Whether a robot telemetry block establishes that the arm is at rest (Milestone 10 §6, Card 060).
// Stale telemetry establishes neither motion nor rest: the sample must be no older than
// maximum_age and no further ahead than maximum_future_skew on the ROS clock at `now`, and every
// arm, gripper and rail speed must be finite and at most rest_speed. `detail` names the cause of a
// refusal (the telemetry age, or the fastest joint), or the evidence of an established stop.
struct RecoveryRestVerdict
{
  bool established{false};
  std::string detail;
};

[[nodiscard]] RecoveryRestVerdict evaluate_recovery_rest(
  const restocker_world_state::RobotExecutionState & robot, const rclcpp::Time & now,
  std::chrono::nanoseconds maximum_age, std::chrono::nanoseconds maximum_future_skew,
  double rest_speed);

enum class MotionWorkflowRoutingGuardState : std::uint8_t
{
  kGeneric,
  kMotionOwned,
  kOwnershipUncertain,
};

struct RestockCoordinatorDriverSnapshot
{
  bool active{false};
  bool terminal_output_emitted{false};
  bool inhibited{false};
  GoalGeneration goal_generation{0};
  std::optional<RestockTaskTransition> transition;
  std::optional<OperationTicket> pending_operation;
  std::size_t pending_transport_requests{0U};
  bool reservation_capability_may_remain{false};
  std::size_t staged_candidate_count{0U};
  std::optional<SteadyTime> command_deadline;
  std::optional<SteadyTime> task_deadline;
  std::optional<GoalGeneration> inactive_after_terminal_ack_generation;
  MotionWorkflowRoutingGuardState motion_workflow_routing{
    MotionWorkflowRoutingGuardState::kGeneric};
  std::size_t guarded_generic_timeout_count{0U};
  std::size_t guarded_generic_advance_count{0U};
  std::size_t guarded_generic_cleanup_count{0U};
  // True while the bounded perception reacquire is holding a planned segment before
  // any ledger operation for it has been booked.
  bool perception_reacquire_active{false};
  // True while the bounded selection reacquire is holding kSelectPair before the
  // selector is retried.
  bool selection_reacquire_active{false};
};

using CoordinatorTaskSelector = std::function<SelectionResult<SelectedTaskPair>(
      const restocker_world_state::WorldStateSnapshot &, const SelectionRequest &)>;
using CoordinatorGraspAuthorityProvider =
  std::function<GraspCandidateResult<GraspGenerationAuthority>(
      const restocker_world_state::WorldStateSnapshot &)>;
using CoordinatorGraspGenerator = std::function<GraspCandidateResult<GraspCandidateBatch>(
      const restocker_world_state::WorldStateSnapshot &, const SelectedTaskPair &,
      const GraspGenerationAuthority &, const rclcpp::Time &)>;
using CoordinatorRosNow = std::function<rclcpp::Time ()>;
// Reports an asynchronous completion that never reached the inbox. Called off the pump thread, so
// an implementation must be thread-safe.
using CoordinatorAsyncDepositDiagnostic = std::function<void (const std::string & detail)>;
// Produces the destination placement for the held product. The node owns the lane geometry and
// frame transforms this needs, so the driver receives it as an injected callback, like grasp
// generation. The grasp coupling is retained evidence and must not be re-derived: the fresh
// snapshot shows the product already carried away from where it was grasped. The reservation is
// passed so admission is judged against the destination policy it was captured with at grant,
// never the lane's live intent, which SetLanePolicy may have changed mid-transfer.
using CoordinatorPlacementGenerator =
  std::function<PlacementCandidateResult<PlacementCandidate>(
      const restocker_world_state::WorldStateSnapshot &, const SelectedTaskPair &,
      const GraspCandidateBatch &, const GraspCoupling &,
      const restocker_world_state::TaskReservation &)>;
// Tool0 retreat pose for the destination lane's survey station. Empty refuses PlanRetreat.
using CoordinatorRetreatTargetProvider =
  std::function<std::optional<Eigen::Isometry3d>(const SelectedTaskPair &)>;
// Fire-and-forget invalidation after physical place-detach. Best-effort with logged failures:
// the retreat survey is what makes the lane selectable again.
using CoordinatorLaneEvidenceInvalidator =
  std::function<void (const std::string & lane_id, std::uint64_t lane_revision)>;
// Completes a destination-lane acquire without closing the SurveyDestination ledger entry.
using CoordinatorDestinationObservationDone =
  std::function<void (CoordinatorLaneAcquireCompletion completion)>;
// Submit one AcquireLaneObservation. Returns false when the call could not be armed; otherwise
// `done` is invoked (possibly later) with the published/refused outcome.
using CoordinatorDestinationObservationAcquirer =
  std::function<bool (
      OperationCorrelation correlation, const std::string & lane_id,
      CoordinatorDestinationObservationDone done)>;
// False when the wrist perception stream is older than the configured liveness horizon.
using CoordinatorPerceptionLivenessCheck =
  std::function<bool (std::string & detail)>;

// One planned segment of the restocking trajectory. The motion port plans and executes a segment
// in a single call, so each kPlan* command submits the segment and the paired kExecute* command
// consumes the evidence that it ran.
enum class MotionSegment : std::uint8_t
{
  kPreGrasp,
  kApproach,
  kRetract,
  kCarryStart,
  kPreInsert,
  kInsert,
  kRetreat,
};

// The three jaw widths a restock task commands. Two are open but differ: the approach clearance
// only has to straddle the product, while release must satisfy the simulator's detach
// precondition, the configured full-open target.
enum class GripperAperture : std::uint8_t
{
  kApproachClearance,
  kHold,
  kRelease,
  // Card 062: the approach clearance of the candidate whose jaws closed, from the retained escape
  // record — never wider than the width that approach was proven at.
  kEscapeClearance,
};

[[nodiscard]] const char * motion_segment_name(MotionSegment segment) noexcept;
// True for the segments that must travel in a straight Cartesian line, not a sampled route.
[[nodiscard]] bool linear_motion_segment(MotionSegment segment) noexcept;
// The segment a plan-phase or execute-phase command drives, or nullopt when the command is not a
// motion command at all.
[[nodiscard]] std::optional<MotionSegment> planned_motion_segment(
  RestockTaskCommand command) noexcept;
[[nodiscard]] std::optional<MotionSegment> executed_motion_segment(
  RestockTaskCommand command) noexcept;
// How one dispatch's failure detail may appear on the action's feedback. The policy is decided
// at the call site that knows the cause — never inferred downstream from command or strings —
// so only failures already classified as retriable planning misses can be rewritten.
enum class FeedbackDetailPolicy : std::uint8_t
{
  // Publish the failure text exactly as produced (the default for every failure kind).
  kRawEvidence,
  // Intermediate retriable OMPL slice exhaustion: publish a bounded-retry diagnostic instead.
  kRetriablePlanningDiagnostic,
};

// Action-feedback detail for one dispatch. With kRetriablePlanningDiagnostic on a still-live
// (non-fault) retryable dispatch, the feedback carries a bounded-retry diagnostic instead of the
// raw abort text, so intermediate planning misses never read as manipulation-abort evidence on
// the action. Every other combination keeps the detail verbatim: pre-grasp candidate fallthrough
// retains its raw planner-refusal text (the acceptance test catches stock-side grasp-quality
// refusals with it), terminal faults keep their detail so fail-closed markers stay visible on
// failed goals, and non-planning retryables such as backend-unavailable are never redacted. The
// raw failure text always remains in the node logs and the recovery audit.
[[nodiscard]] std::string feedback_detail_for_dispatch(
  RestockTaskEvent event, FeedbackDetailPolicy policy,
  const RestockTaskTransition & transition);

// A deterministic single-consumer orchestration pump. ROS callbacks only enqueue immutable
// ingress through inbox(); all context, machine, deadline, and port-submission mutations occur in
// pump().
class RestockCoordinatorDriver
{
public:
  RestockCoordinatorDriver(
    GoalAdmissionSlot & admission, WorldStateCoordinatorPort & world_state,
    CoordinatorTaskSelector selector, CoordinatorGraspAuthorityProvider grasp_authority,
    CoordinatorGraspGenerator grasp_generator,
    CoordinatorSteadyNow steady_now, CoordinatorRosNow ros_now,
    RestockCoordinatorDriverConfig config = {},
    CoordinatorSteadyNow async_evidence_now = {},
    CoordinatorPlacementGenerator placement_generator = {},
    // Optional until a motion backend is composed. Without it the coordinator holds staged
    // candidates at PlanPreGrasp instead of moving.
    MotionPort * motion = nullptr,
    GripperPort * gripper = nullptr,
    AttachmentPort * attachment = nullptr,
    // Optional: with these null the driver takes its deterministic recovery decisions. Nothing
    // downstream of them can widen what the deterministic authorisation permits.
    restocker_reasoner::RecoveryAdvisorPort * recovery_advisor = nullptr,
    restocker_reasoner::RecoveryAuditLog * recovery_audit = nullptr,
    CoordinatorRetreatTargetProvider retreat_target = {},
    CoordinatorLaneEvidenceInvalidator invalidate_lane_evidence = {},
    CoordinatorDestinationObservationAcquirer acquire_destination_observation = {},
    CoordinatorPerceptionLivenessCheck perception_liveness = {},
    // Called from a port's own thread when an asynchronous completion does not reach the inbox.
    // The deposit result is the only evidence and the ports discard it, so the goal can then only
    // end at its command deadline.
    CoordinatorAsyncDepositDiagnostic async_deposit_diagnostic = {});

  [[nodiscard]] std::shared_ptr<CoordinatorInbox> inbox() const noexcept;
  void pump(SteadyTime now);
  [[nodiscard]] std::vector<CoordinatorDriverOutput> take_outputs();
  // Acknowledge exactly one delivered terminal output so the goal slot can be released.
  bool acknowledge_terminal_delivery(GoalGeneration generation);
  [[nodiscard]] RestockCoordinatorDriverSnapshot snapshot() const;

private:
  struct PendingTransportRequest
  {
    WorldStateRequestHandle handle;
    std::optional<CoordinatorReconciliationCorrelation> reconciliation;
  };

  struct RetainedPreContextOverflow
  {
    std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
    SteadyTime arrived_at{};
    std::string detail;
  };

  struct PendingAcceptedDrainFence
  {
    GoalGeneration goal_generation{0U};
    SteadyTime arrived_at{};
    std::shared_ptr<const FirstGoalTerminationRecord> first_termination;
  };

  // What the driver established at the latest motion boundary. Recovery replans a failed segment,
  // or refreshes a later read-only operation, from the current arm state, so it is offered only
  // against a record showing the prior motion reached a terminal stop and retrying cannot make
  // the state worse. Every motion completion replaces this record.
  struct MotionStopEvidence
  {
    MotionSegment segment;
    GoalGeneration goal_generation{0U};
    // Empty when the motion succeeded or a bounded replan of its stopped state is safe; otherwise
    // why it is not.
    std::string refusal;
  };

  // A fully rolled-back attachment transaction proves that neither the simulator boundary nor
  // world state changed. Distinct from motion recovery: every prior motion completed, and
  // recovery is authorised only after a fresh snapshot proves the robot is stopped and its
  // held-object state matches the rollback.
  struct TransactionRollbackRecoveryEvidence
  {
    GoalGeneration goal_generation{0U};
    bool object_held{false};
  };

  // Bounded hold while the wrist perception stream is unlive at a planned-segment request.
  // Keyed to one goal generation and segment so a hold orphaned by termination or cancellation
  // restarts the budget for later work instead of inheriting an expired deadline.
  struct PerceptionReacquire
  {
    SteadyTime started{};
    SteadyTime deadline{};
    GoalGeneration goal_generation{0U};
    MotionSegment segment{MotionSegment::kPreGrasp};
  };

  // The selection-time twin of PerceptionReacquire: task selection reported the wrist stream
  // stale, so kSelectPair holds on the cheap liveness predicate until a fresh observation
  // lets the selector run again, or the shared steady-clock budget expires into the original
  // terminal failure. The retained snapshot cannot gain younger stamps while the state holds
  // it, so a selector that then fails on aged robot/object evidence supersedes back to
  // re-observation (the bounded selection-restart counter) instead of terminaling. Keyed to
  // the goal generation so the budget never restarts mid-goal.
  struct SelectionReacquire
  {
    SteadyTime started{};
    SteadyTime deadline{};
    GoalGeneration goal_generation{0U};
    // Set once the restore has been reported, so a horizon-edge disagreement between the
    // predicate and the selector cannot emit it repeatedly.
    bool reported_live{false};
  };

  void process(CoordinatorInboxDelivery delivery, SteadyTime now);
  [[nodiscard]] bool validate_pending_accepted_drain_fence(
    const CoordinatorInboxDelivery & delivery) const;
  void reject_pending_accepted_drain_fence(std::string detail, SteadyTime now);
  void process_control(const CoordinatorControlEvent & event, SteadyTime now);
  void process_completion(const SnapshotCompletion & completion, SteadyTime now);
  void process_completion(const ReserveTaskCompletion & completion, SteadyTime now);
  void process_completion(const ValidateReservationCompletion & completion, SteadyTime now);
  void process_completion(const ReleaseReservationCompletion & completion, SteadyTime now);
  void process_completion(const CoordinatorMotionCompletion & completion, SteadyTime now);
  void process_completion(const CoordinatorGripperCompletion & completion, SteadyTime now);
  void process_completion(const CoordinatorAttachmentCompletion & completion, SteadyTime now);
  void process_completion(const CoordinatorLaneAcquireCompletion & completion, SteadyTime now);
  void accept_goal(CoordinatorAcceptedGoal accepted, SteadyTime now);
  void check_deadlines(SteadyTime now);
  void abort_read_only_for_termination(SteadyTime evidence_time, SteadyTime now);
  void advance(SteadyTime now);
  // The kInhibitMotion latch boundary (Milestone 10 §6, Card 051): classify, act on the
  // classification (typed skip vs operator latch) and receipt — extracted from advance() for
  // the 500-line cap.
  void handle_inhibit_motion_latch(const RestockTaskTransition & transition, SteadyTime now);
  // One case of advance()'s command switch each, so the loop stays readable. Each returns true
  // when advance() should keep stepping the machine and false when it must hand the turn back.
  [[nodiscard]] bool advance_planned_segment(
    const RestockTaskTransition & transition, SteadyTime now);
  [[nodiscard]] bool advance_executed_segment(
    const RestockTaskTransition & transition, SteadyTime now);
  [[nodiscard]] bool advance_motion_recovery(
    const RestockTaskTransition & transition, SteadyTime now);
  [[nodiscard]] bool advance_generate_placement(SteadyTime now);
  // Runs kSelectPair: the selection-time bounded perception reacquire and the selector call
  // itself. Returns true when advance() should keep stepping the machine.
  [[nodiscard]] bool advance_selection(SteadyTime now);
  [[nodiscard]] MotionWorkflowRoutingGuardState motion_workflow_routing_state() const noexcept;
  void report_guarded_route(const char * route, MotionWorkflowRoutingGuardState state) noexcept;
  [[nodiscard]] bool guard_all_generic_routes() noexcept;
  [[nodiscard]] bool guard_generic_timeout() noexcept;
  [[nodiscard]] bool guard_generic_advance() noexcept;
  [[nodiscard]] bool guard_generic_cleanup() noexcept;

  void observe_transition(
    const RestockTaskTransition & transition, SteadyTime now,
    std::optional<RestockTaskEvent> source_event = std::nullopt,
    FeedbackDetailPolicy feedback_policy = FeedbackDetailPolicy::kRawEvidence);
  void dispatch(
    RestockTaskEvent event, std::string detail, SteadyTime now,
    FeedbackDetailPolicy feedback_policy = FeedbackDetailPolicy::kRawEvidence);
  // Execution events carry the generation of the operation whose trajectory ran.
  void dispatch_execution(
    RestockTaskEvent event, std::string detail, OperationGeneration operation_generation,
    SteadyTime now);
  void fail_operation(
    RestockTaskEvent event, std::string detail, SteadyTime now,
    FeedbackDetailPolicy feedback_policy = FeedbackDetailPolicy::kRawEvidence);
  void reject_grasp_staging(
    const GraspCandidateError & error, std::string prefix, SteadyTime now);
  void begin_deferred_inhibition_cleanup(
    std::string detail, SteadyTime evidence_time, SteadyTime now);
  void continue_deferred_inhibition_cleanup(SteadyTime evidence_time, SteadyTime now);
  void stop_release_submission_without_proof(
    std::string detail, SteadyTime evidence_time, SteadyTime now);
  void inhibit(std::string detail, SteadyTime now);
  void latch_deferred_inhibition(SteadyTime now);
  void finish_if_terminal(SteadyTime now);
  void release_acknowledged_terminal();
  // One feedback output for the perception reacquire's start or restoration, so the hold is
  // observable on the same channel as every other coordinator transition.
  void report_perception_reacquire(const std::string & detail, SteadyTime now);
  // One feedback output for a receipt a later recurrence must be classifiable from: which
  // verification of the goal owned the grasp coupling, and which recovery attempt entered from
  // which state. The node echoes these to the console beside the reacquire lines, so a retained
  // log answers the question without re-running the scenario.
  void report_task_receipt(const std::string & detail, SteadyTime now);
  // Names which precondition refused an execution-operation identity and, when a termination
  // already latched the goal, which termination did it and where both clocks stand — so the
  // retreat-boundary refusal reads as the deadline's consequence, not a lost identity
  // (Milestone 10 §6, Card 044).
  [[nodiscard]] std::string execution_identity_refusal_detail(
    const std::string & guard, SteadyTime now) const;

  [[nodiscard]] std::optional<OperationTicket> start_operation(
    RestockTaskCommand command, OperationEffect effect, std::string operation_id,
    SteadyTime now);
  void request_snapshot(const OperationTicket & ticket);
  // Empty when the freshly observed world state satisfies what this command must prove;
  // otherwise the reason it does not.
  [[nodiscard]] std::string verify_observed_evidence(RestockTaskCommand command) const;
  // Card 060: evaluates the rest predicate on recovery's fresh snapshot and runs the bounded
  // stop-settle hold. True when the stop is established and verification may continue; false
  // when the hold re-observes or the refusal has been dispatched.
  [[nodiscard]] bool settle_recovery_stop(SteadyTime now);
  void request_reservation(
    const OperationTicket & ticket, bool reconciliation_replay = false,
    std::size_t attempt_number = 0U);
  void request_validation(
    const OperationTicket & ticket, bool reconciliation_readback = false,
    std::size_t attempt_number = 0U);
  void request_release(
    const OperationTicket & ticket, bool reconciliation_replay = false,
    std::size_t attempt_number = 0U);
  void request_release_readback(
    const ReconciliationAttempt & attempt);
  // Submit the staged pre-grasp pose as one plan-and-execute segment.
  void request_motion_segment(const OperationTicket & ticket, MotionSegment segment);
  // Command the jaws to the retained candidate's hold or open target.
  void request_gripper(const OperationTicket & ticket, GripperAperture aperture);
  void attach_grasp_escape(MotionGoal & goal) const;
  // Run the attach or detach transaction across Gazebo, the scene lease and world state.
  void request_attachment(
    const OperationTicket & ticket, bool attaching, bool semantic_only = false);
  // The tool0 goal for a segment, drawn from the retained grasp or placement candidate.
  [[nodiscard]] std::optional<Eigen::Isometry3d> motion_segment_target(
    MotionSegment segment) const;
  void begin_reconciliation(SteadyTime now);
  // Record what the driver established about a motion failure, including why a replan is refused.
  void observe_motion_stop(
    MotionSegment segment, const MotionCompletion & completion);
  // Empty when a bounded replan-from-current-state is authorised for the recovery now in
  // progress; otherwise the reason it is refused, in the words of what was established.
  [[nodiscard]] std::string refuse_motion_recovery() const;

  // Graded-recovery classification of the current goal against the Milestone 10 §6 contract
  // (Card 051), built from first-hand driver evidence: the recorded motion stop, the machine's
  // fault and its tracked held-object state. Fail-closed — evidence that is missing classifies
  // UNSAFE. Pure decision, no side effects; callers receipt the result themselves.
  [[nodiscard]] RecoveryClassification classify_current(const std::string & cause) const;
  // What the recovery authorisation permits right now, recomputed from the same evidence
  // refuse_motion_recovery() reads. Never cached across a pump.
  [[nodiscard]] RecoveryAdvicePermissions recovery_permissions_now() const;
  // Send the advisory backend one question about a motion failure. Fire and forget: the answer
  // may arrive, arrive too late, or never arrive; only a timely one can change anything.
  void ask_recovery_advisor(MotionSegment segment, const MotionCompletion & completion);
  // Take whatever answer is waiting, validate it against a freshly computed permitted set, and
  // return the primitive the driver will act on. Returns the deterministic choice when there is no
  // usable recommendation.
  [[nodiscard]] restocker_reasoner::RecoveryPrimitive consume_recovery_advice();
  void forget_recovery_question();


  template<typename Completion>
  [[nodiscard]] OperationCompletionDecision classify_completion(
    const Completion & completion);
  [[nodiscard]] std::optional<SteadyTime> authorize_termination_cleanup_evidence_time(
    const OperationCorrelation & correlation, SteadyTime arrived_at) const;

  void record_request_handle(
    WorldStateRequestHandle handle,
    std::optional<CoordinatorReconciliationCorrelation> reconciliation = std::nullopt);
  void complete_matching_request_handle(
    const OperationCorrelation & correlation,
    const std::optional<CoordinatorReconciliationCorrelation> & reconciliation);
  void remove_reconciliation_request(const ReconciliationAttempt & attempt);
  void remove_operation_requests(const OperationCorrelation & correlation);
  [[nodiscard]] SteadyTime bounded_deadline(
    SteadyTime start, std::chrono::milliseconds duration) const;
  [[nodiscard]] RestockActionOutcome terminal_outcome() const;
  [[nodiscard]] RestockActionMetrics metrics(SteadyTime now) const;

  GoalAdmissionSlot & admission_;
  CoordinatorTerminationRouter termination_router_;
  WorldStateCoordinatorPort & world_state_;
  CoordinatorPlacementGenerator placement_generator_;
  CoordinatorRetreatTargetProvider retreat_target_;
  CoordinatorLaneEvidenceInvalidator invalidate_lane_evidence_;
  CoordinatorDestinationObservationAcquirer acquire_destination_observation_;
  CoordinatorPerceptionLivenessCheck perception_liveness_;
  MotionPort * motion_{nullptr};
  GripperPort * gripper_{nullptr};
  AttachmentPort * attachment_{nullptr};
  // Set when a segment completed successfully, consumed by the paired kExecute* command.
  std::optional<MotionSegment> executed_segment_;
  std::optional<MotionStopEvidence> motion_stop_evidence_;
  // Card 086 stage 1b: what this goal has commanded, for the terminal result's motion evidence.
  // `commanded` is set BEFORE any motion, gripper or attachment goal is submitted and is cleared
  // again only when a motion submission is refused outright or its completion proves nothing was
  // started; `stop_observed` is set only by a motion completion that carries a terminal stop and is
  // cleared by every later command. The prior_* pair lets the proven-not-started cases restore.
  struct CommandedEvidence
  {
    bool commanded{false};
    bool stop_observed{false};
    bool prior_commanded{false};
    bool prior_stop_observed{false};
  };
  CommandedEvidence commanded_;
  void note_command_submitted() noexcept;
  void note_motion_submission_undone() noexcept;
  std::optional<TransactionRollbackRecoveryEvidence> transaction_rollback_recovery_evidence_;
  std::optional<PerceptionReacquire> perception_reacquire_;
  std::optional<SelectionReacquire> selection_reacquire_;
  // The bounded stop-settle hold of one recovery attempt (Card 060).
  struct RecoveryStopSettle
  {
    SteadyTime started{};
    SteadyTime deadline{};
    SteadyTime next_poll{};
    GoalGeneration goal_generation{0U};
    std::size_t recovery_attempt{0U};
  };
  std::optional<RecoveryStopSettle> recovery_stop_settle_;
  // The optional advisory client, its audit log, and the single-slot letterbox its worker thread
  // writes into. The letterbox is separate from the coordinator inbox: pressure on that inbox
  // inhibits the task, so an advisory answer must never occupy a slot a motion completion needs.
  // Held by shared_ptr because an answer may arrive after this driver is gone.
  restocker_reasoner::RecoveryAdvisorPort * recovery_advisor_{nullptr};
  restocker_reasoner::RecoveryAuditLog * recovery_audit_{nullptr};
  std::shared_ptr<restocker_reasoner::RecoveryAdviceMailbox> recovery_advice_mailbox_;
  // The question outstanding for this goal generation, retained to check the answer against it.
  std::optional<restocker_reasoner::RecoveryQuery> recovery_question_;
  // Set when a decision record has been written and its outcome has not. Cleared by the outcome
  // record.
  std::optional<restocker_reasoner::RecoveryAuditRecord> recovery_outcome_pending_;
  // Identifies each attempt's executed trajectory to the task machine. Not a ledger operation.
  OperationGeneration next_execution_generation_{0U};
  CoordinatorTaskSelector selector_;
  CoordinatorGraspAuthorityProvider grasp_authority_;
  CoordinatorGraspGenerator grasp_generator_;
  CoordinatorSteadyNow steady_now_;
  CoordinatorSteadyNow async_evidence_now_;
  CoordinatorAsyncDepositDiagnostic async_deposit_diagnostic_;
  MotionWorkflowRoutingGuardState last_reported_routing_state_{
    MotionWorkflowRoutingGuardState::kGeneric};
  std::size_t guarded_route_reports_{0U};
  CoordinatorRosNow ros_now_;
  RestockCoordinatorDriverConfig config_;
  const std::size_t ingress_budget_;
  std::shared_ptr<CoordinatorInbox> inbox_;
  PendingOperationLedger ledger_;
  std::unique_ptr<RestockGoalContext> context_;
  std::optional<RetainedPreContextOverflow> pre_context_overflow_;
  std::optional<PendingAcceptedDrainFence> pending_accepted_drain_fence_;
  std::vector<PendingTransportRequest> request_handles_;
  std::optional<SteadyTime> command_deadline_;
  std::optional<SteadyTime> task_deadline_;
  // Milestone 10 §6 (Card 051): measured from the whole-task expiry, bounds the one cleanup
  // retreat that expiry may command. Absent until the expiry fires.
  std::optional<SteadyTime> deadline_retreat_until_;
  // Card 051 review blocker: set when the whole-task deadline instant passed while another
  // termination had already won first — the expiry is receipted once, then never applied, so
  // it can never arm the cleanup retreat on top of that termination.
  bool deadline_observed_after_first_termination_{false};
  std::optional<RestockTaskState> observed_state_;
  std::size_t observed_attempt_{0U};
  std::size_t observed_recovery_attempt_{0U};
  bool terminal_output_emitted_{false};
  bool terminal_delivery_acknowledged_{false};
  std::optional<GoalGeneration> inactive_after_terminal_ack_generation_;
  std::size_t guarded_generic_timeout_count_{0U};
  std::size_t guarded_generic_advance_count_{0U};
  std::size_t guarded_generic_cleanup_count_{0U};
  bool inhibited_{false};
  std::size_t release_submission_rejections_{0U};
  bool release_submission_exhausted_{false};
  std::optional<std::string> deferred_inhibition_detail_;
  std::optional<RestockActionOutcome> primary_outcome_;
  std::optional<std::string> primary_detail_;
  std::vector<CoordinatorDriverOutput> outputs_;
};

}  // namespace restocker_task_executor
