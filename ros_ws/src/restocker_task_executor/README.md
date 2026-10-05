# restocker_task_executor

The decision layer of the restocking robot. It chooses which product goes into which shelf lane,
drives one transfer from grasp to verified placement, decides what to do when something goes
wrong, and runs the autonomous loop that keeps the shelf filled.

For the system-level picture (control path, world state, ports) see the repository
[README](../../../README.md) and the
[task execution](../../../website/docs/architecture/task-execution.mdx) page of the manual. This
file covers the package itself.

## Design ideas

- **Deterministic pipeline.** A task state machine validates the scene, selects a product and lane,
  generates grasp and placement candidates, and hands each motion segment to MoveIt. Nothing
  outside this chain can command a joint or skip planning and collision checks. An optional
  advisory reasoner (`reasoner.enabled`, off by default) can only decline a recovery that the
  deterministic rules already allowed.
- **World-state verification.** The task does not trust a component's own report. A reservation
  is checked by reading it back, a placement is accepted only when authoritative world state
  shows the product in the lane, and a release is provisional until a later snapshot shows the
  reservation gone.
- **Fail closed.** Missing or stale evidence, unknown outcomes and ambiguous results are treated
  as unsafe. Unknown mutations are reconciled by reading state back, never replayed blindly.
- **Recoverable versus unrecoverable.** Every refusal is classified before anything is latched.
  Recoverable faults go through a bounded recovery ladder; only unsafe states or exhausted
  budgets stop the campaign and ask for an operator.
- **Bounded retries.** Every retry, skip and re-survey is charged to an explicit budget, so a
  persistent problem ends in a clear stop instead of a loop.

## Executables and public interface

| Executable | Role |
| --- | --- |
| `restock_action_coordinator` | Serves `RestockProduct` and runs one transfer at a time |
| `autonomous_restock_campaign` | Continuous loop that surveys, decides and requests transfers |
| `tray_survey_node` | Serves `SurveyTray` (read-only tray survey and close confirmation) |
| `lane_survey_node` | Serves `SurveyLane` (shelf lane survey) |
| `survey_viewpoint_node` | Serves `SurveyViewpoint` (moves to a named survey station) |
| `planning_scene_projector_node` | Keeps the MoveIt planning scene in step with world state |
| `planning_smoke_test`, `planning_only_benchmark` | Planning diagnostics |

Actions (definitions in `restocker_interfaces/action`):

- `/restock_product` (`RestockProduct`). The goal has optional `object_id` and `lane_id`
  selectors (with `has_*` flags). Both unset: the coordinator picks the pair. Object only: the
  product is named and only the destination is chosen; if it is no longer eligible the goal ends
  with a typed refusal instead of substituting another product. Both set: the pair is checked
  against the same eligibility rules. A lane without an object is refused. Feedback carries the
  task state and attempt counters. The result carries a stable `status`
  (`STATUS_SUCCEEDED`, `STATUS_NO_COMPATIBLE_PAIR`, `STATUS_PLANNING_FAILED`,
  `STATUS_EXECUTION_FAILED`, `STATUS_VERIFICATION_FAILED`, `STATUS_RECOVERY_EXHAUSTED`,
  `STATUS_OBSERVATION_EVIDENCE_STALE`, `STATUS_SKIPPED_RECOVERABLE`, `STATUS_OPERATOR_REQUIRED`
  and others), the selected object, world revisions, and two evidence flags,
  `motion_definitely_not_started` and `execution_reached_terminal_stop`, which both default to
  false (unknown).
- `/survey_tray` (`SurveyTray`), `/survey_lane` (`SurveyLane`) and `/survey_viewpoint`
  (`SurveyViewpoint`), used by the campaign. `SurveyTray` results carry an `outcome`:
  `OUTCOME_CONFIRMED`, `OUTCOME_NO_CANDIDATE`, `OUTCOME_REFUTED`, `OUTCOME_OVERVIEW_ONLY`,
  `OUTCOME_VIEWPOINT_UNREACHED`, `OUTCOME_ACQUISITION_FAILED`, `OUTCOME_CANCELED`,
  `OUTCOME_UNAVAILABLE` or `OUTCOME_INVALID_REQUEST`.

Topics:

- `/restock_action_coordinator/status` (`RestockCoordinatorStatus`). Clients wait for
  `admission_ready: true` before sending goals. Startup must first prove that the world is
  unreserved, idle, fault-free and empty-handed.
- `/autonomous_restock_campaign/status` (`AutonomousRestockCampaignStatus`), reliable and
  transient-local. Phases include `PHASE_SURVEYING_FRONT`, `PHASE_SURVEYING_BACK`,
  `PHASE_MEASURED`, `PHASE_RESTOCKING`, `PHASE_FRONT_FULL`, `PHASE_STOCK_EXHAUSTED`,
  `PHASE_BLOCKED`, `PHASE_RECOVERING` and `PHASE_COMPLETE`.
- Tray overview candidates are read from `/perception/tray_candidates`, which world state does not
  ingest. Close-range confirmations are read from `/perception/object_observations`.

## Parameters

Coordinator defaults are in `config/restock_action_coordinator.yaml`, which carries comments on
the geometry and timing values. The ones most worth knowing:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `task.total_timeout_ms` | 480000 | Whole-task deadline for one transfer |
| `task.planning_timeout_ms` / `task.execution_timeout_ms` | 60000 / 180000 | Budget for planning and executing one segment |
| `task.max_operation_retries` / `task.max_recovery_attempts` | 1 / 2 | Retries per operation and recoveries per goal |
| `selection.maximum_object_age_ms` | 500 | Oldest object observation selection will trust |
| `selection.lane_evidence_validity_ms` | 60000 | How long lane evidence stays valid |
| `selection.perception_liveness_max_age_ms` | 500 | Wrist depth stream liveness horizon |
| `perception.reacquire_timeout_ms` | 5000 | Wait for the wrist stream to return before refusing |
| `grasp.pregrasp_distance_m` / `grasp.retract_distance_m` | 0.18 / 0.12 | Standoff and retract distances |
| `reasoner.enabled` | false | Optional advisory reasoner |

Campaign parameters (set on `/autonomous_restock_campaign`):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `failed_cycle_backoff_sec` | 1.0 | Back-off after a cycle that made no progress |
| `max_lane_survey_attempts_per_cycle` | 2 | Shelf survey retries per cycle |
| `max_tray_attempts_per_cycle` | 8 | Tray survey attempts per cycle |
| `max_survey_skips` | 2 | Survey stations or lanes that may be skipped per run |
| `max_product_skips` | 2 | Times one product may be skipped after recoverable failures |
| `max_ncp_resurveys` | 2 | Forced re-surveys after a premature `NO_COMPATIBLE_PAIR` |
| `max_feed_order_resurveys` | 2 | Forced re-surveys when feed order blocked every candidate |
| `max_skip_resurveys` | 2 | Re-surveys with skip marks lifted |
| `absence_confirmations` | 2 | Independent empty views before a product is retired |
| `recovery_safe_station` | first tray overview station | Where the arm retreats after a failed survey |
| `restart_acknowledged` | false | Operator acknowledgement after a campaign restart |

## How one transfer works

The coordinator runs the transfer as a state machine (`RestockTaskMachine`, task states mirrored
in `RestockProduct` feedback). Each state exposes one command and a timeout, and there is also a
whole-task deadline.

```text
ValidateScene -> SelectPair -> ReserveTask
  -> GenerateGrasps -> PlanPreGrasp -> ExecutePreGrasp -> approach -> CloseGripper
  -> VerifyGrasp -> AttachTransaction -> retract
  -> ObserveDestination -> GeneratePlacement -> carry, pre-insert, insert
  -> OpenGripper -> DetachTransaction -> retreat -> SurveyDestination
  -> CommitDetachment -> VerifyPlacement -> UpdateInventory -> Complete
```

Side paths are `CancelActiveMotion`, `Recover`, `OpenGripperForEscape`, `ReleaseTask`, `Fault`,
`RequestOperator` and `Canceled`.

- **Selection.** Task selection works on an immutable world-state snapshot. One pair predicate
  applies to automatic, object-only and explicit requests. Products must be fresh, tracked, free,
  upright and inside the stock region. Lanes must be valid, unobstructed, compatible and have
  enough measured rear depth for another product. Eligible destinations are ordered by largest
  deficit first, then rail distance. A product that world state already counts inside a lane is
  never a stock candidate, even if its pose has not been refreshed. When no pair is eligible, the
  refusal reports a tally of which rule rejected which object.
- **Reservation.** The chosen pair is reserved in world state. The returned token is treated as a
  capability and read back before motion. Cancellation can release it without waiting for a
  failed read-back.
- **Candidates.** `grasp_candidates` produces a bounded, stably scored set of yaw candidates and
  jaw targets; `placement_candidates` centres the product in the lane and emits pre-insertion,
  final and retreat poses. Neither claims reachability. MoveIt validates every pose and path.
- **Motion.** Each segment is planned and executed through `MoveItMotionPort` in one submission.
  Before planning and again before execution, the planning scene is checked for a clean,
  unchanged state (no lease, no epoch or content change). The wrist depth stream must be live,
  with a bounded wait if it dips (see below). Linear paths that come back partial are retried
  at a finer Cartesian step, and a partial path is never executed.
- **Attachment.** Grasping and releasing are transactions across Gazebo, MoveIt and world state.
  Failures before the semantic commit compensate the physical state; failures after it reconcile
  forward. Detachment waits for evidence that the product settled in the lane.
- **Verification.** Release is split into a physical detach, a retreat and a lane survey, and
  only then a semantic commit. `VerifyPlacement` accepts the transfer only when authoritative
  world state shows the product in the destination lane. If the whole-task deadline passes after
  world state has already accepted the placement, the goal still ends `STATUS_SUCCEEDED`.
- **Cancellation and deadlines.** Cancellation is latched until non-motion work reaches a defined
  boundary. Active free-space motion first cancels and verifies a stop. Timeouts are never
  reported as safe failures, and an old trajectory is never retried after an execution failure.

## Obstacles and the planning scene

The projector writes workcell geometry, products and depth-derived obstacles into the MoveIt
scene under `restocker/...` names, as the only writer. Depth boxes already contained in known
geometry (shelf, stock, the carried product) are dropped, and a changed obstacle set is accepted
only from an observation taken while the arm was still. Stale obstacle evidence degrades the
projector with `ERROR_OBSTACLE_EVIDENCE_STALE` but keeps the geometry. Products that are not
currently in view are held at their last known pose rather than removed; only `Removed` or `Lost`
lifecycle evidence takes a product out of the scene. Small pose changes from physics jitter
(`product_change_position_tolerance_m`, 0.5 mm by default) do not trigger scene writes.

## Recovery and refusal

Every terminal outcome is classified, on one log line
(`recovery classification: RECOVERABLE|UNSAFE (reason): cause`), before anything is latched.

- **RECOVERABLE** means nothing irreversible happened: no unexpected contact, a known held-object
  state, an arm at a verified stop (or one that never moved), and a world that can be
  re-surveyed.
- **UNSAFE** means real contact, an unknown or lost held object, an arm whose stop cannot be
  shown, a persistent hardware fault, or anything that cannot be established. Unknown counts as
  unsafe.

A recoverable transfer fault climbs a ladder of bounded steps:

1. Safe retreat to the survey viewpoint, if a trajectory may have run.
2. Re-observe, so a retry never reuses the evidence that just failed.
3. Retry with fresh evidence, within `task.max_operation_retries` and
   `task.max_recovery_attempts`.
4. Try the next grasp candidate, if the planner refused a pose. Scene conditions and timeouts do
   not consume candidates.
5. End the goal with `STATUS_SKIPPED_RECOVERABLE`. The campaign charges `max_product_skips`
   against that product, prefers others, and revisits it later.

Only an UNSAFE classification or an exhausted budget latches the operator
(`STATUS_OPERATOR_REQUIRED`, campaign `PHASE_BLOCKED`). No safety check, tolerance or age window
is relaxed by the ladder. It only decides what happens after a check refuses.

Specific behaviours:

- **Failed grasp.** If the jaws closed and the grasp then failed, the gripper opens to the
  approach clearance and the next motion backs out along the reversed approach. Only the contacts
  the start state really has between the product and the fingers are tolerated, in a local copy
  of the scene. move_group's own scene is never changed.
- **Failed plans.** A refused plan is attributed before it spends anything. An invalid start
  state or a planner timeout is not a verdict on the grasp candidate and does not consume it.
  Every refused plan logs MoveIt's code, time spent against the budget and the start-state
  condition.
- **Stale wrist stream.** A brief dip in the depth stream starts a bounded wait
  (`perception.reacquire_timeout_ms`, steady clock). Selection or the segment proceeds if the
  same liveness check passes within it. Otherwise the original refusal stands.
- **Survey failures.** A failed survey or confirm action goes through the same classification.
  Recoverable failures retreat to `recovery_safe_station`, try another station or lane first,
  retry with fresh evidence, and finally skip the station or lane for the cycle. `PHASE_BLOCKED`
  is published only for an UNSAFE failure or an exhausted budget.
- **Unresolved motion.** The campaign keeps an in-memory record of every goal whose motion
  outcome is not proven (lost reply, non-terminal result code, no stop evidence). While any
  record is open it sends no goals and reports `PHASE_BLOCKED` with `motion_unresolved: true`.
  Only exact-identity evidence settles a record. After a campaign restart it reports
  `PHASE_RECOVERING` and waits until `restart_acknowledged` is set to true. This guards only the
  campaign process, not direct clients of the survey actions.

## Autonomous campaign

`autonomous_restock_campaign` loops through five modes: `IDLE`, `SURVEY_SHELF`, `SURVEY_TRAY`,
`CONFIRM`, `TRANSFER`. Each cycle:

1. Decodes one world-state snapshot and surveys only the lanes whose evidence is invalid, expired
   or missing (all lanes at start).
2. Computes each lane's deficit (target count minus products held at the catalogued pitch), using
   the same measure as task selection.
3. For a deficit with no valid admitted tray candidate, runs `SurveyTray`: overview from the
   tray stations, pick one candidate, then a close-range confirmation.
4. Sends a `RestockProduct` goal only after `OUTCOME_CONFIRMED` (naming the confirmed product) or
   for an already-valid admitted candidate. Overview evidence and stale lane evidence never
   authorise a transfer. A refuted candidate is marked and the survey re-entered.
5. Parks in `IDLE` when there is no deficit, or when the tray has no candidate for any
   outstanding deficit class. It wakes on new lane evidence or a new admitted candidate.

Tray selection nominates only the front product of a feed column, because the coordinator refuses
a product standing behind another one. Ties are broken by confidence, then smallest X, Y, Z. If
the only candidates are blocked by feed order, the result reports `feed_blocked_candidates > 0`,
which distinguishes it from an empty tray.

When the campaign gets `NO_CANDIDATE` or a premature `NO_COMPATIBLE_PAIR` while stock may still
exist, it does not simply stop. Each case has a bounded forced re-survey:

| Situation | Response | Budget |
| --- | --- | --- |
| `NO_COMPATIBLE_PAIR` while the campaign sees a deficit and back stock | Re-survey the tray to refresh evidence | `max_ncp_resurveys` |
| Tray `NO_CANDIDATE` caused only by feed order | Re-survey with this cycle's refutations cleared | `max_feed_order_resurveys` |
| Tray `NO_CANDIDATE` caused only by skip marks | Re-survey with the marks lifted ("try others first", not "never again") | `max_skip_resurveys` |

A refutation from a close view is never lifted. A truly empty state (zero deficit, or a deficit
with no back stock) still ends at `PHASE_FRONT_FULL` or `PHASE_STOCK_EXHAUSTED` without spending
budget. A tray product stops counting as back stock only on positive evidence: an observed fall,
or `absence_confirmations` independent empty views from separate work cycles
(`absence_probe_verdicts` of `PROBE_VIEWED_EMPTY`). Retirement is local to the campaign and is
undone by any newer observation; world state is never modified.

## Library modules

The coordinator is built from small pure components, each with its own unit tests:

| Module | Purpose |
| --- | --- |
| `task_selection` | Pair predicate and ordering over world-state snapshots |
| `grasp_candidates`, `placement_candidates`, `gripper_geometry`, `manipulation_geometry` | Candidate generation from description-owned geometry |
| `restock_task_machine`, `restock_goal_context` | Task lifecycle and the single mutable owner of one goal |
| `attachment_transaction_machine` | Attach and detach transactions with readback reconciliation |
| `restock_coordinator_driver` | Single-consumer pump that sequences ports, deadlines and reconciliation |
| `restock_coordinator_primitives`, `restock_coordinator_inbox` | Single-goal slot, bounded inbox, operation ledger |
| `pregrasp_planning_authority`, `planning_contract`, `planning_scene_lease` | Gates and contracts around planning and scene writes |
| `world_state_port_contract`, `WorldStateAsyncPort` | Reservation and release against world state |
| `restock_action_contract`, `action_client_finality` | Action wire mapping and terminal-result handling |
| `recovery_classification` | Recoverable versus unsafe decision |
| `tray_survey_logic` | Windowing, candidate merging, selection and the confirm verdict |

ROS-backed ports and deterministic fake ports implement the same interfaces, so the driver and
state machine are tested without a simulator. Coordinator shutdown fails closed: a goal's result
is delivered only after its reservation release is proven, an allocation failure in goal adoption
latches the coordinator against further goals, and a clean-shutdown claim requires that no
reservation, handle, mutation or transport request remains.

## Running the tests

From the repository root, inside the Nix shell:

```bash
just test-package restocker_task_executor   # unit and node tests for this package
just test-package restocker_bringup         # full-stack acceptance tests (headless, slow)
```

Most tests are plain gtest or pytest cases that need no simulator. The campaign tests drive the
real node against fake action servers and a fake world. Tests that render a simulated world share one machine-wide
simulator slot, so they run one at a time. The dense-tray gate is excluded from the normal suite
and runs only through `just dense-gate`.
