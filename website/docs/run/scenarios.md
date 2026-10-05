---
id: scenarios
title: Scenarios and parameters
sidebar_position: 3
---

# Scenarios and parameters

Pass launch arguments after the recipe name, for example
`just launch-sim gui:=false ground_truth:=false`.

## `just launch-rviz`

Robot model in RViz only. No Gazebo, no controllers.

```bash
just launch-rviz
just launch-rviz gui:=false rviz:=false
```

The joint-state GUI is visualization-only and never becomes authoritative robot evidence.

## `just launch-sim`

Gazebo + `ros2_control` + telemetry adapter + ground-truth observations + world-state node.
Does **not** start MoveIt, the planning-scene projector, the attachment ROS adapter, or the
coordinator.

```bash
just launch-sim
just launch-sim gui:=false
```

### Simulation launch arguments

These are `simulation.launch.py`'s own defaults; `baseline.launch.py` (which includes it)
overrides several of them for the sensor-driven default — see the baseline table below.

| Argument | Default | Meaning |
| --- | --- | --- |
| `gui` | `true` | Start the Gazebo graphical client |
| `controller_timeout` | `30.0` | Controller startup deadline (seconds) |
| `scenario_seed` | `fixed` | Scenario RNG seed (`fixed` or integer) |
| `spawn_scenario` | `true` | Spawn shelf and configured products |
| `ground_truth` | `true` | Publish deterministic product/lane observations |
| `cameras` | `true` | Bridge cameras from Gazebo |
| `wrist_camera` | `true` | Include wrist camera |
| `perception` | `false` | Overhead **object** producer (off: the overhead camera is obstacles-only) |
| `object_observation_topic` | `/perception/object_observations` | Where the ground-truth adapter publishes — still the ingest topic in this composition |
| `lane_observation_topic` | `/perception/ground_truth/lane_observations` | Lane ingest topic (ground truth, in this composition) |
| `lane_survey` | `true` | Lane survey composition |
| `tray_overview_perception` | `false` | Tray overview perception |
| `tray_confirm_perception` | `false` | Tray confirm perception |

Useful inspection from a second flake terminal:

```bash
just controllers
just topics
just control-smoke
ros2 topic echo --once /robot_telemetry
```

## `just launch-baseline`

Full deterministic stack: simulation + MoveIt + planning-scene projector + attachment adapter +
task coordinator. With no arguments it is the **sensor-driven default**: the wrist tray duties
own the world state's object ingest topic, the wrist lane survey owns its lane topic, ground
truth is demoted to `/perception/ground_truth/*` for the evaluators, and the overhead camera is
used for **obstacle detection only**.

```bash
just launch-baseline
just launch-baseline gui:=false rviz:=false
just launch-baseline scenario_seed:=42
```

### Readiness gates

```bash
ros2 topic echo --once /planning_scene_projection/status
ros2 topic echo --once /restock_action_coordinator/status
# want STARTUP_READY + admission_ready: true
```

Discovery is not authority. Wait for `admission_ready: true` before sending goals. This proves
coordinator admission readiness, not a usable object/lane pair: inspect the world snapshot and
projector evidence too. A plain baseline starts the wrist producers but neither the autonomous
campaign nor the coordinated tray-survey action. `tray_survey:=true` enables `/survey_tray` and
its viewpoint server; `autonomous_campaign:=true` enables both and drives the evidence cycle.
`just demo` supplies the acceptance fixture and campaign. An unaddressed manual restock goal
can correctly refuse selection before those surveys have supplied evidence.

Use one behavior at a time. Do not send manual survey goals while a campaign or restock goal
owns motion; complete cross-endpoint behavior ownership is an implementation limitation.

### Unaddressed goal (system chooses product and lane)

```bash
ros2 action send_goal /restock_product \
  restocker_interfaces/action/RestockProduct \
  '{has_object_id: false, object_id: 0, has_lane_id: false, lane_id: ""}' \
  --feedback --timeout 180
```

The `--timeout 180` option bounds this CLI's wait, not the coordinator's task. The task's
whole-job budget is 480 s by default, with separate motion budgets. If the CLI times out,
inspect coordinator and action status: it has not established a terminal robot state or
canceled/stopped motion. To observe a normal full task, allow a longer CLI wait than its task
budget; retain the terminal result before submitting another goal.

An object-only goal (`has_object_id: true` with the object id, `has_lane_id: false`) names the
product and leaves only the destination to selection — the autonomous campaign's confirm path
transfers its close-confirmed candidate this way. A lane without an object is refused.

### Baseline-specific arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `gui` | `true` | Gazebo graphical client |
| `rviz` | `true` | MoveIt RViz |
| `rviz_config` | `restocker_moveit_config/rviz/moveit.rviz` | Absolute RViz config path; [`just demo`](./demo) passes its camera layout here |
| `motion_enabled` | `true` | Coordinator plans and executes motion |
| `planning_scene_projection` | `true` | Project world/workcell geometry into MoveIt |
| `attachment_adapter` | `true` | Capability-authorized simulator attachment |
| `task_coordinator` | `true` | Fail-closed restock action coordinator |
| `reasoner_enabled` | `false` | Optional advisory recovery reasoner |
| `reasoner_model` | _(empty)_ | Model name for the advisory backend |
| `reasoner_audit_path` | _(empty)_ | JSONL audit file; empty writes none |
| `obstacle_perception` | `true` | Depth obstacle → planning scene (the overhead camera's job on the default path) |
| `perception` | `false` | Overhead **object** producer — off by default: the overhead camera is obstacles-only |
| `tray_overview_perception` | `true` | Wrist tray overview → `/perception/tray_candidates` (hypotheses) |
| `tray_confirm_perception` | `true` | Wrist tray confirm → `/perception/object_observations` (the world state's object source) |
| `object_observation_topic` | `/perception/ground_truth/object_observations` | Where **ground truth** publishes; demoted off the ingest topic on the default path |
| `lane_observation_topic` | `/perception/lane_observations` | World-state lane ingest topic (wrist lane survey on the default path) |
| `product_observation_max_age` | `600.0` | Projector reports older product evidence as aged; the product stays in the scene (seconds) |
| `selection_object_max_age_ms` | `180000` | Selection evidence age limit |
| `lane_evidence_validity_ms` | `180000` | Lane evidence validity window |
| `maximum_observation_age_ms` | `180000` | Reservation observation age limit |
| `tray_survey` | `false` | Coordinated tray overview/confirmation action; also enables viewpoint action |
| `survey_viewpoint` | `false` | Wrist survey viewpoint action |
| `autonomous_campaign` | `false` | Enable shelf/tray evidence acquisition and one-transfer cycles until idle |
| `campaign_max_cycles` | `0` | Maximum mode-loop cycles, at most one transfer per progress cycle; zero is unbounded |
| `campaign_cycle_period` | `30.0` | Delay between campaign cycles (seconds) |
| `campaign_restart_acknowledged` | `false` | Operator acknowledgment that the robot is verified settled; while `false` the campaign reports `PHASE_RECOVERING` and sends no goal (set it later with `ros2 param set /autonomous_restock_campaign restart_acknowledged true`). `just demo` passes `true` because its world is fresh |
| `planning_smoke` | `false` | Finite planning acceptance client |
| `planning_timeout` | `5.0` | OMPL planning time per smoke request |
| `scenario_seed` | `fixed` | Seeded scenario replay |
| `ground_truth` | `true` | Deterministic observations |
| `controller_timeout` | `30.0` | Controller startup deadline |

Shared simulation arguments (`spawn_scenario`, camera flags, observation topics, …) also apply
because baseline includes the simulation composition.

## `just launch-perception`

```bash
just launch-perception
# Avoid colliding GT and camera on the same ingest topic:
just launch-perception object_observation_topic:=/perception/ground_truth/object_observations
```

Starts `simulation.launch.py` with `perception:=true`. When cameras are on, the overhead
perception node publishes camera-derived observations — this is the overhead **object** pipeline,
which the default baseline path deliberately leaves off (the overhead camera there is for
obstacle detection only). `simulation.launch.py` keeps ground truth on the default ingest topic,
so move the ground-truth output aside (as above) or both producers collide on it.

## `just launch-full`

Intentionally unimplemented. Exits with a clear error.

## `just planning-smoke`

```bash
just planning-smoke
```

Headless baseline with `planning_smoke:=true` and `planning_scene_projection:=false`. Sends joint
and pose goals, executes valid trajectories, and verifies known collision geometry is rejected.
Planning success and trajectory execution success are separate outcomes.

## Scenario seeding

Seeded generation is byte-deterministic, so a failure found once can be reproduced:

```bash
just launch-baseline scenario_seed:=42
```

## Perception topic collision (important)

The world state ingests `/perception/object_observations` and admits **one publisher** per ingest
topic. On the default `baseline.launch.py` path ground truth is already demoted to
`/perception/ground_truth/object_observations` and the wrist confirm duty is the ingest topic's
only producer, so nothing collides until a second producer is added: enabling the overhead
`perception:=true` pipeline while the confirm duty is still on puts two publishers on the ingest
topic and the world state refuses both (fail-closed / refused authentication).

To put a second producer on the ingest topic you must move the first one aside; that is the whole
rule — `tray_confirm_perception:=false` with `perception:=true` on the baseline (ground truth is
already demoted there), or `object_observation_topic:=/perception/ground_truth/object_observations`
on `simulation.launch.py`, whose own defaults still put ground truth on the ingest topic. The
overhead object pipeline ships off for a deeper reason than the collision: as the executor's pose
source the swap deadlocks the retreat after a release (see the `restocker_perception` README),
which is why the default path keeps the wrist confirm duty there and leaves the overhead camera
to obstacle detection.
`obstacle_perception` defaults to **on** and feeds `/perception/obstacle_observations` from
`depth_obstacle_node` alone; `test_dynamic_obstacle_runtime` is the one composition that pins it
off, so its synthetic publisher owns that topic. Both the detector and the projector's
`require_obstacle_evidence` additionally gate on `cameras`, so `cameras:=false` disables the
whole pipeline together instead of demanding evidence no producer can send.

## Coordinator timeouts (not launch args)

From `restocker_task_executor/config/restock_action_coordinator.yaml`:

| Key | Default | Meaning |
| --- | ---: | --- |
| `task.total_timeout_ms` | `480000` | Whole-task deadline (480 s) |
| `task.execution_timeout_ms` | `180000` | Per motion segment |
| `reasoner.*` | host `127.0.0.1`, port `8080`, timeout `4000` ms | Sidecar only if enabled |

## `just control-smoke` parameters

Requires a running simulation. Useful flags:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `startup_timeout_sec` | `20.0` | Discovery / contract budget |
| `execute_trajectory` | `true` | If false, stop after controller contract |

## Headless vs display

| Entry | Display |
| --- | --- |
| `just demo` | Required (hard check) |
| `launch-*` defaults | Gazebo/RViz required processes, crash without display |
| `gui:=false` / `rviz:=false` | Headless |
| `planning-smoke`, `benchmark*` | Forced headless; still need OpenGL/EGL for cameras |

## Not implemented

`just launch-full` prints an unavailable message and exits **2**. Prefer
`just launch-baseline` plus explicit optional flags (`perception`, `obstacle_perception`,
`reasoner_enabled`) instead of waiting for a monolithic “full” recipe.
