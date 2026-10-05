---
id: contracts
title: Public contracts
sidebar_position: 1
---

# Public contracts

Quick index of the operator-visible ROS surface. Full architecture lives under
[Architecture](/docs/architecture).

## Actions

| Action | Role |
| --- | --- |
| `/restock_product` (`RestockProduct`) | Public task API |
| `/survey_tray` (`SurveyTray`) | Coordinated overview/confirmation; reports acquisition, refutations, feed blockers and motion evidence |
| `/survey_viewpoint` (`SurveyViewpoint`) | Aim wrist optical frame via motion port |
| `/survey_lane` (`SurveyLane`) | Aim + acquire lane observation |

## World-state services

`/world_state/get_snapshot`, `reserve_task`, `validate_reservation`,
`validate_execution_authority`, `checkpoint_task`, `commit_attachment`,
`commit_detachment`, `release_reservation`, `invalidate_lane_evidence`,
`set_lane_policy`.

`set_lane_policy` is the runtime lane-intent API (expected class, optional SKU, target count,
caller-captured lane revision). It persists atomically and is refused outright on a stale
revision; see [World state](/docs/architecture/world-state).

Snapshot events include `LANE_LEDGER_UNRELIABLE`, appended when measured lane geometry and the
contents ledger diverge beyond half a product pitch: identity trust is withdrawn, geometry is
retained, and the condition is operator-visible without stopping the run.

## Status topics

| Topic | Role |
| --- | --- |
| `/restock_action_coordinator/status` | Admission gate (`admission_ready`) |
| `/planning_scene_projection/status` | Projector applied / degraded + revision |
| `/autonomous_restock_campaign/status` | Sequence, cycle, phase, mode, deficits, successful transfers, detail and the unresolved-motion marker (when enabled) |

### Restock result: motion evidence

`RestockProduct.Result` carries `motion_definitely_not_started` and
`execution_reached_terminal_stop`, named and used like the survey results' fields. Both default to
`false` (unknown); see
[Task execution](/docs/architecture/task-execution#unresolved-motion-campaign) for when the
coordinator sets them and which statuses never carry them.

### Survey results: busy and backend-loss outcomes

A survey goal that the survey layer refuses as busy (an earlier survey may still be moving the arm),
and a motion backend lost after the trajectory was handed to it, are no longer reported as
`OUTCOME_UNAVAILABLE` / a retryable non-start. `SurveyViewpoint` reports `OUTCOME_EXECUTION_FAILED`;
`SurveyLane` reports its usual unreached or unavailable outcome with
`motion_definitely_not_started` and `execution_reached_terminal_stop` both false. A client that
retried on `OUTCOME_UNAVAILABLE` for these cases must not retry on them now: `OUTCOME_UNAVAILABLE`
means the backend was unavailable before anything was submitted.

### Campaign status: unresolved motion

`/autonomous_restock_campaign/status` carries an explicit marker for motion the campaign cannot
prove stopped (see [Task execution](/docs/architecture/task-execution#unresolved-motion-campaign)):

| Field | Meaning |
| --- | --- |
| `motion_unresolved` | `true` while any goal attempt is unresolved or a restart awaits acknowledgment. The campaign sends no goal and reports neither `PHASE_COMPLETE` nor `PHASE_FRONT_FULL` while it is set; `detail` begins with `motion_unresolved:` |
| `unresolved_attempts` | One entry per retained attempt: endpoint, send generation, condition (`admission_unresolved`, `result_unresolved`, `settlement_pending_cancel`) and the exact goal UUID once known |
| `restart_recovery_pending` | `true` from process start until the operator sets the `restart_acknowledged` parameter; `phase` is `PHASE_RECOVERING` |

Only exact-identity settlement clears the marker. The marker covers the campaign process; it does
not fence clients of the public endpoints, and it does not survive a restart.

## Perception topics

`/perception/object_observations`, `/perception/lane_observations`,
`/perception/obstacle_observations`, plus `/perception/ground_truth/*` peers when producers
are swapped. See [Perception and models](/docs/architecture/perception-and-models).

`admission_ready` is distinct from having fresh product/lane evidence. Campaign status reports
progress; a blocked phase, CLI timeout or cancel acknowledgement is not terminal motion proof.
See the [operator diagnosis guide](/docs/run/tests-and-benchmarks#diagnose-a-stopped-or-blocked-run)
and [survey prerequisites](/docs/run/scenarios#readiness-gates).
