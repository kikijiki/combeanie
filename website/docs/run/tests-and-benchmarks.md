---
id: tests-and-benchmarks
title: Tests and benchmarks
sidebar_position: 4
---

# Tests and benchmarks

## Package and acceptance tests

```bash
just test
just test-package restocker_bringup
```

A single launch test (paths are required because the workspace lives under `ros_ws/`):

```bash
nix develop -c scripts/with_workspace.bash colcon test \
  --base-paths ros_ws/src --build-base ros_ws/build --install-base ros_ws/install \
  --packages-select restocker_bringup \
  --ctest-args -R test_restock_lane_routing_runtime
```

Capability-defining runtime tests include:

| Behaviour | Test | Typical timeout |
| --- | --- | ---: |
| Autonomous dense fixture reaches `PHASE_FRONT_FULL` after at least 13 transfers | `test_autonomous_restock_demo_runtime` | 4500 s |
| Dense fixture sensor-driven front filled from camera-named products (excluded from the default suites; run with `just dense-gate`) | `test_dense_sensor_driven_runtime` | 4500 s |
| Explicit 3-category transfers | `test_restock_manipulation_runtime` | 1800 s |
| Coordinator chooses product *and* lane | `test_restock_lane_routing_runtime` | 1800 s |
| Obstructed lane routing | `test_restock_obstructed_lane_runtime` | 900 s |
| Unmodeled obstacle changes executable set | `test_dynamic_obstacle_runtime` | 900 s |
| Camera depth → obstacle gate | `test_obstacle_perception_runtime` | 300 s |
| Perception producer swap | `test_perception_runtime` | 600 s |
| Baseline composition smoke | `test_baseline_runtime` | 120 s |

At the time of the release (2026-09-30 review), twenty tests carry the CMake label
**`renderer`** (eighteen in `restocker_bringup`, two in `restocker_gazebo`) and are excluded
from `nix flake check`. One of them, `test_dense_sensor_driven_runtime`, additionally carries
the label **`dense-gate`**: `just test` and `just test-package` deselect it with
`--label-exclude dense-gate`, and `nix flake check` deselects `'renderer|dense-gate'`. The only
path that runs it is the named, serialized gate:

```bash
just dense-gate   # one run, one machine-wide simulator slot, quiet-start preconditions
```

Even a green `just test` does not establish that dense sensor gate — the exclusion is by label,
and only `just dense-gate` covers it (its exit codes: 0 green, the ctest code red with the
failing case named, 2 not-run with the failed precondition). Use the generated test inventory to
check the count and label status for your revision.

The Nix `ros-workspace` check caps its build and test concurrency at two jobs, or one when
`NIX_BUILD_CORES=1`. Builds run one package at a time with bounded compiler parallelism;
tests run up to two packages with one CTest job each. This keeps colcon and Ninja from
expanding to the host CPU count. An unset core allocation defaults to one; zero (unlimited)
is capped at two. To also serialize separate Nix derivations, use
`nix flake check --max-jobs 1 --cores 2`.

Launch tests, the gtests and pytests of every package with ROS tests, and `just benchmark` /
`just demo` lease `ROS_DOMAIN_ID` + `GZ_PARTITION`.
`test_tray_survey_node` and `test_restock_action_coordinator_node` fail at once when run outside
`ctest` without that lease; run them through `ctest -R`.
Interactive `launch-*` recipes do **not**; pick a free domain if other sessions exist.

## Controller smoke

With `just launch-sim` already running:

```bash
just control-smoke
just control-smoke --ros-args -p execute_trajectory:=false
```

## Benchmarks

The harness reports a failure taxonomy. It exits 0 when it **records** a failure, since recording
is the job:

```bash
just benchmark baseline_transfers
just benchmark baseline_transfers --repeat 8
just benchmark seeded_autonomous --seed 1 --seed 2
just benchmark-all          # then report
just benchmark-report
```

| Scenario | Goals | Seedable |
| --- | --- | --- |
| `baseline_transfers` | addressed can / small bottle / large bottle | no |
| `dense_restock_transfers` | the same three addressed goals against the dense fixture | no |
| `lane_routing` | two bottles into one lane, then unaddressed | no |
| `seeded_autonomous` | one unaddressed goal per stocked product | yes |

Benchmarks are always headless (`gui`/`rviz` false). They need a working OpenGL/EGL path even
without a window. Run campaigns sequentially, not in parallel.

Each run record's `provenance` carries **`obstacle_perception`** — the *effective* depth-obstacle
pipeline value that run launched with (an explicit launch argument if the harness passes one,
otherwise the shipped `baseline.launch.py` default, read from the launch file itself). That default is `true`, so every benchmark runs `depth_obstacle_node` with the projector
requiring its evidence unless a campaign pins the argument. Campaigns must **predeclare** the
value they intend to measure under; stored records without the field predate schema 3.

### Paired transfer-reliability campaign

`just benchmark-paired` is the predeclared instrument for a campaign-scale UR10e transfer rate.
Its design is frozen in code (`restocker_benchmarks/transfer_reliability.py`) before any sample
exists, so the population and denominator cannot be refit to results:

| Element | Frozen definition |
| --- | --- |
| Configurations | `baseline_transfers` (A) and `dense_restock_transfers` (B), each three addressed goals |
| Pairing | One run of each per pair, interleaved: A then B on odd pairs, B then A on even pairs, so neither fixture systematically inherits the other's teardown or a drifting load |
| Population | 24 pairs by default: the first 24 attempts per configuration, including attempts that fail at startup |
| Transfer denominator (primary) | Every goal record inside population runs; success is terminal `SUCCEEDED`. Goals never dispatched by a startup-failed run do not exist as attempts — that failure is charged to the run denominator |
| Run denominator (secondary) | Every population run; *clean* means completed with every recorded goal succeeded |
| Seeds | Both fixtures are fixed-pose (`seed == "fixed"`); the stock file digest is recorded per run. A numeric seed is a predeclaration violation |
| Machine load | 1-minute load average at start **and** finish, CPU count, GPU busy%; at aggregation a run is *quiet* iff `load1 < 0.5 × cpu_count`, else *contended*. Load annotates every rate and never excludes a run |
| Terminal taxonomy | Goals: the failure taxonomy above (family + masked signature), recomputed at aggregation. Runs: exactly one of `clean`, `completed_with_failure`, `startup_failed`, `launch_exited`, `harness_exception`, `unclassified` — an unknown outcome fails the classification check rather than being bucketed |
| Arm gate | Population eligibility requires `provenance.arm.label == "ur10e"` parsed from `arm.xacro`. A record that cannot name that arm (anything from before the UR10e swap) is retained, reported, and never counted |
| No replacement | No record is deleted, rewritten or superseded. A re-run is a new record beyond the declared population: retained, excluded from rates, never swapped into the failed row |
| Readiness gate | Before any goal is dispatched the harness waits for **scenario completeness**: every product in the scenario document must be spawned *and* observed in the world-state snapshot (observed == stocked), on top of coordinator admission readiness and admitted robot telemetry. The wait is bounded (`SCENARIO_READY_TIMEOUT_S`, 180 s); a fixture that never finishes is recorded as a classified `startup_failed` with the partial observation archived — never a hang, and never a dispatch into a half-spawned fixture. A `--dense-rerun` addendum re-measured `dense_restock_transfers` under this gate after its first campaign exposed the race |

**Sample size and what it supports.** 72 planned transfers and 24 runs per configuration
(144 and 48 pooled). At a mid-range pass rate of 0.65 the design half-widths are **±11.0 pp**
(transfers, per configuration), **±7.8 pp** (pooled) and **±19.1 pp** (runs); zero run failures in
24 bounds the run-failure rate at **11.7%** (one-sided 95%, exact). If the true run-failure rate
were 0.35, the chance of seeing zero failures in 24 runs is ≈ 3.2 × 10⁻⁵, so a zero-failure
population rejects that rate. It does **not** support sub-10 pp discrimination between the two
configurations, any claim about a different arm, or the autonomous dense-demo gate's population.

```bash
just benchmark-paired                 # full predeclared campaign into a fresh directory
just benchmark-paired --pairs 1       # one smoke pair; not a claim population
just benchmark-paired --dense-rerun   # the frozen dense-only addendum population into its own directory
just benchmark-paired --summarise-only  # re-aggregate a stored campaign, no simulator
```

One invocation writes one campaign directory under
`benchmark-results/ur10e-transfer-reliability/` (or `benchmark-results/ur10e-transfer-reliability-dense-rerun/`
for `--dense-rerun`) and refuses a directory that already holds
records, so no attempt can replace another. Each run archives its console log and spawned
scenario document beside its `run.json` with commit, stock digest, seeds, arm identity and load.
A run of the three-transfer baseline scenario measured **172–405 s (median ≈ 4.2 min)** on the
campaign machine, so the full 48-run paired campaign took about **103 minutes** end to end (the
dense configuration's harness-terminal runs are far shorter); the dense-only addendum re-run took
about **122 minutes** for 24 attempts once its runs reached full transfers. The aggregation writes `summary.json`,
`summary.txt` and `report.md`; the harness exits 0 when it records failures, because recording
them is the job.

## Diagnose a stopped or blocked run

Use the domain printed by the launch or demo in every inspection terminal. Source the built
workspace (`source ros_ws/install/setup.bash`) inside the flake shell. These commands inspect
state; they do not submit motion or change inventory:

```bash
ros2 topic echo --once /restock_action_coordinator/status
ros2 topic echo --once /planning_scene_projection/status
# Only present when autonomous_campaign:=true:
ros2 topic echo --once /autonomous_restock_campaign/status
ros2 service call /world_state/get_snapshot restocker_interfaces/srv/GetWorldState \
  '{include_removed: true, include_events: true}'
ros2 topic info --verbose /perception/object_observations
ros2 topic info --verbose /perception/lane_observations
ros2 topic info --verbose /perception/obstacle_observations
ros2 action list -t
```

Retain the full launch console, effective launch arguments, git revision, status sequence and
detail, world revision/events, and action terminal result. `/restock_product/_action/status`
can show whether a goal is still active, but a CLI timeout or cancellation acknowledgement is
not proof that motion has stopped. Avoid overlapping manual survey/restock goals: complete
behavior ownership and late cancellation finality are not yet enforced across all endpoints.

| Symptom | Inspect and next step |
| --- | --- |
| Qt failure immediately after launch | Use a graphical session, or set `gui:=false rviz:=false`; camera rendering still needs a working graphics driver. |
| `admission_ready: false` | Read coordinator `startup_state`, `inhibited`, `detail` and `has_orphaned_reservation`. Check projector status and robot telemetry; server discovery alone does not prove readiness. |
| Tray survey returns no candidate | Retain `SurveyTray` acquisition reports, `overview_candidates`, `feed_blocked_candidates`, `marked_feed_blocked_candidates` and `feed_block_example`. Check class/SKU intent, occlusion, front-to-back feed ordering, and refutation/skip marks. A blocked or unseen candidate does not prove an empty tray. |
| Motion inhibited or unknown outcome | Preserve coordinator/projector details and the action result. Do not retry through another endpoint or assume a passed deadline stopped the arm. Establish terminal execution and reconcile held state before further commands; there is no documented generic “clear inhibition” shortcut. |
| No usable object/lane evidence | A plain baseline enables camera producers but does not run the coordinated tray survey or campaign. Check timestamps, TF at acquisition, single publisher authority and the survey prerequisites in [Scenarios](./scenarios#readiness-gates). Overview hypotheses alone cannot authorize a pick. |
| Contested observations | Inspect publisher GIDs with `topic info --verbose`. Keep one authorized publisher per ingest topic; move ground truth to its evaluator topic and avoid enabling overhead object perception beside wrist confirmation. |
| Projector degraded / stale depth | Inspect its reason and evidence timestamps; check cameras and `/perception/obstacle_observations`. Aged geometry remains collision geometry. Do not delete it to make planning pass. |
| `LANE_LEDGER_UNRELIABLE` | Compare snapshot lane evidence, measured geometry and contents. Identity trust has been withdrawn; geometry remains. This event does not by itself stop the campaign. Do not treat a visually empty region as authoritative removal or manually fabricate inventory commits. |
| `STARTUP_ORPHANED_RESERVATION` | Preserve the snapshot and prior logs. Tokens are opaque; inventing a release token or restarting only world state cannot reconcile physical attachments and in-memory inventory. |

For a disposable simulation, the supported clean restart is to stop the entire composition,
wait for its processes/controllers to terminate, then launch a fresh fixture with matching lane
policy in an isolated domain. This resets the experiment; it is not recovery of the old run's
inventory. Preserve failure evidence first. Attached products, uncertain motion and orphaned
reservations require reconciliation before treating an existing run as ready.

## Interpret reliability evidence

A test pass applies to its revision, scenario, hold mode and effective configuration. A historical
weld-grasp measurement is not a physical-grip result. Keep startup failures in the run population,
and distinguish dispatched transfer attempts from planned goals. Retain complete console output
for passing and failing attempts; a short ctest success summary proves the terminal verdict but
cannot establish that intermediate refusals or retries were absent.

The 2026-09-29 sensor acceptance population passed 14/24 (Wilson 95% 38.8–75.5%). A later
24-attempt weld population, measured at the time of the release, passed 19/24 (79.2%, Wilson 95% 59.5–90.8%). A two-sided
Fisher comparison gives p=0.2124, so these populations do not establish an improvement. All 19
passing logs in the later population are ctest summaries, so overall intermediate-refusal
incidence cannot be compared. The raw run logs are not published; this summary is not
an independently reproducible raw dataset or a claim about the current revision. Neither
population establishes a dense sensor-driven or physical-grip success rate.
