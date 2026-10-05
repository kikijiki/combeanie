---
id: demo
title: Demo
sidebar_position: 2
---

# Demo

Fastest path to watch the robot restock:

```bash
just demo            # sensor-driven: the shipped default, no evidence pins
just demo 3          # at most three campaign cycles
just demo gt:=true   # labelled opt-in: the dense ground-truth demo
```

This brings up Gazebo, MoveIt, RViz, and the coordinator, waits until the coordinator reports
ready, then starts the autonomous mode loop. **By default it runs the shipped sensor-driven
configuration** — the same defaults as a plain `baseline.launch.py`: the wrist lane survey and
the wrist tray-confirm duty own the world state's ingest topics, simulator ground truth is
demoted to `/perception/ground_truth/*` where only the error evaluators read it, the evidence
ages are the shipped 180 s profile, and the overhead camera is on for **obstacle detection
only** (its object pipeline stays off). Nothing about the default path is pinned by the demo
script: what you watch is what the product ships.

At start of run the robot sweeps every front-lane station with its wrist camera, measures each
lane's deficit against the declared target, and submits unaddressed `/restock_product` goals.
A transfer goes out only after that survey's close-range confirmation (or against an
already-admitted candidate) while every lane's evidence is inside its validity horizon. The
loop parks in `IDLE` when every lane reaches its target.

## The ground-truth opt-in

`just demo gt:=true` switches to the **labelled ground-truth mode**: it prints
`GROUND-TRUTH MODE (opt-in)` and pins ground truth back onto the ingest topics with both wrist
tray duties off (one producer per topic), on the dense fixture — six partially stocked front
lanes and three dense-but-partial back grids, so a full run makes thirteen transfers before the
front is full. Ground truth refreshes continuously there, which is why that mode can pin the
whole pre-flip evidence profile the deterministic demo was written against —
`product_observation_max_age` 2.0 s, `selection_object_max_age_ms` 2000,
`maximum_observation_age_ms` 2000 and `lane_evidence_validity_ms` 60000 — instead of the shipped
180 s sensor profile. The default path pins none of them.

Use it when you want the long, busy run; the default is what the system actually runs.

## What you see

RViz opens with a predefined three-pane layout instead of the plain MoveIt window:

- **Centre — planning view.** The same MoveIt `MotionPlanning` display as
  [`just launch-baseline`](./scenarios): robot model, planning scene, fixed frame `world`.
- **Right, top — Overhead Camera.** The fixed shelf-facing RGB-D camera's colour stream on
  `/overhead_camera/image` (960x720, about 6 Hz): the whole shelf face, so you can watch deficits
  empty and transfers land. On the default path this camera is feeding obstacle detection, not
  product perception.
- **Right, bottom — Wrist Camera.** The gripper-mounted colour stream on
  `/wrist_camera/image` (1280x720, about 5 Hz): whatever the arm is looking at — lane survey
  stations and tray confirmation viewpoints as they happen. On the default path this is the
  camera the evidence comes from.
- **Left — Displays and Views panels**, docked rather than floating.

Both image panes subscribe reliably, matching the Gazebo→ROS bridge's publisher. The layout lives
in `ros_ws/src/restocker_bringup/rviz/demo_camera_view.rviz` and is selected only by this demo
through the `rviz_config` launch argument; `just launch-baseline` and every headless test or
benchmark keep the default `moveit.rviz`, so nothing else changes behaviour. Depth images,
`camera_info`, and Gazebo's own GUI are not part of the layout.

The optional number limits mode-loop **cycles**, and every progress cycle submits at most one
transfer goal — so `just demo 3` moves about three products at most, not three survey-and-drain
phases. With no number the campaign continues until Ctrl-C.

## What to expect

The default fixture is the sensor acceptance one: three lanes each wanting one product, one
product per class in the tray behind, so the loop needs **three transfers** to reach
`PHASE_FRONT_FULL` with zero deficit and then idles until evidence changes. That is the table the
acceptance test proves. Camera identity also names the dense tray's repeated products, by their
stocking position. However, the dense scenario has not yet completed a sensor-driven run, so it
stays the ground-truth opt-in (see the repository README's *Known limitations*). The dense
sensor-driven end-to-end check exists as its own serialized gate, `just dense-gate`, which is
excluded by label from `just test` and `nix flake check` — a green default suite never implies
that coverage (see [Tests and benchmarks](tests-and-benchmarks)).

In either mode the acceptance gate is the same shape: fill every measured deficit against the
declared targets and end in `PHASE_FRONT_FULL`, not `PHASE_STOCK_EXHAUSTED` or
`PHASE_BLOCKED`. A full dense run takes about 45 minutes; the default's three transfers take a
few minutes after the cold-start sweep. Individual transfers fail closed if motion, evidence, or
outcome verification is invalid; a blocked phase returns to surveying on the next cycle instead
of claiming success or retrying a failed pair in a tight loop.

Measured reliability of that loop on the default path: a predeclared 24-run campaign on
2026-09-29 passed **14/24 (58.3 %, Wilson 95 % 38.8–75.5 %)**; every failure ended at a
named refusal (the tray survey finding no candidate, the recovery ladder ending in
`motion inhibited`, or the survey-recovery skip budget being spent).

A later 24-attempt weld population, measured at the time of the release, passed **19/24** (79.2%, Wilson 95%
59.5–90.8%). The comparison to 14/24 is not statistically separated (Fisher p=0.2124), and
passing console summaries do not retain enough detail to compare intermediate refusals.
These are historical fixture-specific outcomes, not a current reliability guarantee; see
[the evidence limitations](./tests-and-benchmarks#interpret-reliability-evidence).

## Requirements

- Graphical session: `DISPLAY` or `WAYLAND_DISPLAY` (Xwayland counts)
- Built workspace (`just build`)
- Flake shell active (`direnv` or `nix develop .`)

Without a display, `scripts/demo.bash` exits immediately and points you at headless alternatives.

## Isolation

The demo leases a `ROS_DOMAIN_ID` and `GZ_PARTITION` via `scripts/domain_isolation.py`. A second
terminal that inspects topics must use the same domain the demo printed.

## Under the hood

1. Launch `baseline.launch.py` with `motion_enabled:=true`, `autonomous_campaign:=true`,
   `campaign_restart_acknowledged:=true` (the demo world is fresh, so the campaign's restart gate
   is acknowledged up front),
   `lane_policy_state:=…/demo_lane_policy_state.absent` (no persisted SetLanePolicy document can
   overlay the fixture's targets), and `rviz_config:=…/restocker_bringup/rviz/demo_camera_view.rviz`
   (the camera layout above) — **and, on the default path, no evidence arguments at all**; the
   ground-truth opt-in additionally passes `object_observation_topic`,
   `lane_observation_topic`, both tray duties off, `product_observation_max_age:=2.0`,
   `selection_object_max_age_ms:=2000` and `placement_require_column_growth:=false`
2. Wait up to 300 s for `/restock_action_coordinator/status` with `admission_ready: true`
3. Sweep all six front-lane stations and both back-tray stations before the first transfer
4. Restock from current evidence until the front is full or no compatible pair remains, then
   repeat the scheduled survey cycle
5. Leave the GUI up until Ctrl-C or the optional maximum cycle count is reached

Default fixture (`sensor_acceptance_products.yaml` + `sensor_acceptance_lanes.yaml`): lanes
`lane_01`/`lane_02`/`lane_03` each want one product, `lane_04`–`lane_06` want none, and the tray
stocks exactly one can, one small bottle and one large bottle at interleaved poses. The
ground-truth opt-in pairs the dense scenario with `dense_restock_lanes.yaml`, whose targets
(can 10, small bottle 9, large bottle 6 per lane) are what the shipped front columns can reach
cleanly; the shipped baseline want of six per lane is already satisfied by that overstocked
fixture, so the loop would otherwise idle immediately, and packed-capacity targets drive the last
placements where the depth-derived held count under-counts them.

Rebuild the dense fixture with `tools/generate_dense_restock_scenario.py` and the default one
with `tools/generate_sensor_acceptance_scenario.py` if a surveyed layout changes. To drive the
sparse baseline yourself, use [`just launch-baseline`](./scenarios) and send a `RestockProduct`
action explicitly.