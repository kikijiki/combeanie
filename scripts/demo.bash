#!/usr/bin/env bash
# Watch the system do the thing it exists to do: bring up the workcell, survey the lanes whose
# evidence needs it, and run the mode loop -- deficit-driven tray surveys, transfers, and
# idle-until-evidence changes -- until every lane reaches its declared target. The optional
# numeric argument is a maximum mode-loop cycle count; zero is unbounded.
#
# Three scenes, chosen by one option each; the DEFAULT_MODE line below is the only thing that
# decides which one runs with no option:
#
#   showcase (default; `showcase:=true`): the generated, labelled catalogue cans and bottles --
#     three shapes, five SKUs -- as a stocked shelf (six lanes, some at target, some short) and a
#     back room of packed one-SKU blocks, in an art-directed world. It runs on the GROUND-TRUTH
#     evidence path: ground truth is pinned onto the ingest topics and both wrist tray duties are
#     off, so exactly one producer owns each topic. The sensor-driven path cannot run this scene
#     yet, because its identity assigner names a product by its class and so cannot tell several
#     SKUs of one class apart at tray and lane range. Nothing about identity is simulated: the
#     ground-truth producer reports each product's declared SKU, which is what that path is.
#   gt (`gt:=true`): the dense plain-coloured fixture, thirteen transfers, same ground-truth path.
#   sensor (`sensor:=true`): the shipped sensor-driven path -- no evidence pins at all, so this
#     runs exactly what `baseline.launch.py` runs: wrist lane survey and wrist tray confirm own the
#     world state's ingest topics, simulator ground truth is demoted to /perception/ground_truth/*
#     where only the error evaluators read it, and the overhead camera is on for obstacle
#     detection only. It drives the sensor acceptance fixture (three lanes, three transfers, then
#     PHASE_FRONT_FULL and an idle loop). The dense sensor-driven end-to-end check runs only as its
#     own gate: `just dense-gate` (excluded by label from `just test` / `nix flake check`).
#
# Every start prints which evidence path it is on.
#
# This is deliberately not a test and not a benchmark. The acceptance tests run headless and assert;
# the benchmark harness runs headless and tallies. This one shows you the robot.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Fail now, with a reason, rather than in five minutes with a symptom.
#
# The Gazebo client is a *required* process in this launch, so with no graphical session it dies on
# the Qt platform plugin within seconds and takes the whole composition down with it. What you then
# see is this script sitting out its 300 s readiness deadline and reporting that the coordinator
# never became ready, which is true and useless: the coordinator was never started. Checking here
# turns a five-minute mystery into one line.
if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  cat >&2 <<'NODISPLAY'
error: no graphical session (both DISPLAY and WAYLAND_DISPLAY are unset)

This demo shows you the robot, so it needs a window. The Gazebo client is a required process in
the launch; without a display it exits immediately and takes the rest of the composition with it.

If you are on the machine's desktop, run this from a terminal inside that session. Over SSH, or in
CI, use a headless entry point instead, these assert rather than display, and prove the same
behaviour:

  just test-package restocker_bringup   # the acceptance tests: real physics, planning, world state
  just benchmark baseline_transfers     # the same transfers, tallied rather than watched
NODISPLAY
  exit 2
fi

# Run on a domain and a Gazebo partition of our own, the way every launch test does.
#
# On the default domain the readiness probe below is not probing this demo's coordinator, it is
# probing whichever coordinator is reachable, and if another baseline is already up on this
# machine, that one answers immediately. The demo then reports ready before its own Gazebo has
# even loaded the world, and sends its goals to the other system's arm. Nothing about that looks
# like a failure; it looks like a suspiciously fast success.
#
# Both are forced rather than defaulted from the environment. Honouring an inherited
# ROS_DOMAIN_ID is exactly the case that goes wrong, and the point here is isolation, not
# configurability. The domain used to be the fixed 55; it is now leased for the life of this
# process from the same machine-wide pool the launch tests draw from, because a fixed one is
# only unique within a workspace and this repository is routinely worked on from several
# worktrees at once. The lease also reaps this demo's own orphans, a Gazebo that outlives a
# `kill -9` is the other half of the same problem. Re-executing under the lease is what holds
# it: the descriptor lives in the parent, so the kernel releases it however this exits.
#
# The lease replaces a `pgrep`-plus-`ppid=1` reap that used to stand here, on three measured
# counts. It matched process *names* (`gz sim|robot_state_publisher|ros_gz_bridge`), so it walked
# past every other node this demo starts, including the tf2 static publishers named in the
# campaign that motivated it. It gated on `ppid == 1`, which a detached grandchild does not
# satisfy: a `kill -9` here left a launcher at `ppid=1` and a `setsid` child at `ppid=<launcher>`,
# and only the tag caught both. And it sent `SIGTERM` with no escalation, to a simulator this
# file's own `cleanup` already documents as not always answering one. The lease matches
# `RESTOCKER_ISOLATION_TAG` as a whole `/proc/<pid>/environ` entry, kills by pid, escalates, and
# runs both before the demo and after it. Holding the lease is what proves nothing else is live on
# the domain, which is the thing `ppid` was being asked to prove and never could.
if [[ -z "${RESTOCKER_ISOLATION_TAG:-}" ]]; then
  # The lease suffixes this, so the partition stays readable in `gz topic -l` while being
  # unique per run: restocker_demo_<workspace>-<domain>.
  export GZ_PARTITION=restocker_demo
  exec python3 "$repo_root/scripts/domain_isolation.py" --seed demo -- "${BASH_SOURCE[0]}" "$@"
fi

# The one line that decides what a bare `just demo` runs: showcase, gt or sensor.
DEFAULT_MODE=showcase

max_cycles=0
mode="$DEFAULT_MODE"
for argument in "$@"; do
  case "$argument" in
    showcase:=true | gt:=true | sensor:=true) mode="${argument%%:=*}" ;;
    showcase:=false | gt:=false | sensor:=false) ;;
    *[!0-9]* | "")
      printf 'error: unknown argument %s (expected a cycle count, showcase:=true, gt:=true or sensor:=true)\n' \
        "$argument" >&2
      exit 2
      ;;
    *) max_cycles="$argument" ;;
  esac
done
ground_truth=false
[[ "$mode" == "sensor" ]] || ground_truth=true

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    printf '\n-- shutting down --\n'
    kill -INT "$launch_pid" 2>/dev/null || true
    # Gazebo does not always answer SIGINT; give it a grace period, then insist.
    for _ in $(seq 1 30); do
      kill -0 "$launch_pid" 2>/dev/null || return
      sleep 1
    done
    kill -KILL "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

# lane_policy_state points at a path that does not exist so a stale persisted SetLanePolicy
# document cannot overlay the fixture's targets. ros2 launch refuses a literally empty value on
# the command line, and world_state only reads the document when the file is present.
#
# rviz_config selects the demo camera layout: the MoveIt planning view plus docked panes for
# the bridged /overhead_camera/image and /wrist_camera/image colour streams. The argument's
# default is still restocker_moveit_config's moveit.rviz, so no other launch, test, or
# benchmark changes by this line existing.
demo_rviz_config="$repo_root/ros_ws/src/restocker_bringup/rviz/demo_camera_view.rviz"
absent_lane_policy="$repo_root/ros_ws/log/demo_lane_policy_state.absent"

if [[ "$ground_truth" == "true" ]]; then
  # Labelled opt-in: the dense ground-truth demo. Evidence pins go here and nowhere else -- the
  # default path below carries none, so it is the shipped sensor-driven configuration.
  evidence_args=(
    lane_observation_topic:=/perception/ground_truth/lane_observations
    object_observation_topic:=/perception/object_observations
    tray_overview_perception:=false
    tray_confirm_perception:=false
    product_observation_max_age:=2.0
    selection_object_max_age_ms:=2000
    lane_evidence_validity_ms:=60000
    maximum_observation_age_ms:=2000
    placement_require_column_growth:=false
  )
  if [[ "$mode" == "showcase" ]]; then
    # The showcase also selects its own dressed world (same physics and plugins as the default
    # one) whose GUI opens framed on the `wide` view of website/scripts/showcase_views.py.
    scenario_args=(
      scenario_config:="$repo_root/ros_ws/src/restocker_gazebo/config/showcase_products.yaml"
      lane_semantics:="$repo_root/ros_ws/src/restocker_world_state/config/showcase_lanes.yaml"
      world:="$repo_root/ros_ws/src/restocker_gazebo/worlds/restocking_showcase.sdf"
      # The workcell and every product are written into the world file, so the window opens on
      # the complete scene instead of watching eighty-odd models spawn one at a time.
      bundle_scenario:=true
      scene_materials:="$repo_root/ros_ws/src/restocker_gazebo/config/showcase_materials.yaml"
    )
    printf -- '-- SHOWCASE SCENE, GROUND-TRUTH EVIDENCE PATH --\n'
    printf -- '-- labelled cans and bottles, five SKUs: a stocked shelf and a packed back room.\n'
    printf -- '-- Ground truth is pinned onto the ingest topics and the wrist tray duties are off,\n'
    printf -- '-- so each ingest topic has exactly one producer. The sensor-driven path cannot name\n'
    printf -- '-- several SKUs of one class yet, so this scene does not run on it. --\n'
    printf -- '-- The sensor-driven path: just demo sensor:=true. Dense plain fixture: gt:=true. --\n'
  else
    scenario_args=(
      scenario_config:="$repo_root/ros_ws/src/restocker_gazebo/config/dense_restock_products.yaml"
      lane_semantics:="$repo_root/ros_ws/src/restocker_world_state/config/dense_restock_lanes.yaml"
    )
    printf -- '-- GROUND-TRUTH MODE (gt:=true) --\n'
    printf -- '-- dense fixture, 13 transfers; ground truth pinned onto the ingest topics and the\n'
    printf -- '-- wrist tray duties off, so each ingest topic has exactly one producer. --\n'
  fi
else
  # The shipped configuration: no evidence pins. baseline.launch.py's own defaults are the
  # sensor-driven configuration (ground truth demoted to /perception/ground_truth/*, wrist lane
  # survey and wrist tray confirm owning the ingest topics, 180 s evidence ages, overhead camera
  # obstacles-only). Leaving the pins out is the point: the demo runs what the product ships.
  evidence_args=()
  scenario_args=(
    scenario_config:="$repo_root/ros_ws/src/restocker_gazebo/config/sensor_acceptance_products.yaml"
    lane_semantics:="$repo_root/ros_ws/src/restocker_world_state/config/sensor_acceptance_lanes.yaml"
  )
  printf -- '-- SENSOR-DRIVEN MODE (sensor:=true; the shipped configuration, no evidence pins) --\n'
  printf -- '-- wrist lane survey and wrist tray confirm own the evidence, ground truth is demoted\n'
  printf -- '-- to /perception/ground_truth/* for the evaluators, the overhead camera is on for\n'
  printf -- '-- obstacle detection only. Three lanes, three transfers, then PHASE_FRONT_FULL. --\n'
  printf -- '-- The stocked, textured showcase (ground-truth path): just demo --\n'
fi

printf -- '-- starting the workcell (Gazebo, controllers, MoveIt, coordinator) --\n'
"$repo_root/scripts/with_workspace.bash" \
  ros2 launch restocker_bringup baseline.launch.py motion_enabled:=true \
  task_execution_timeout_ms:=180000 \
  autonomous_campaign:=true campaign_restart_acknowledged:=true \
  campaign_max_cycles:="$max_cycles" \
  lane_policy_state:="$absent_lane_policy" \
  rviz_config:="$demo_rviz_config" \
  ${evidence_args[@]+"${evidence_args[@]}"} \
  ${scenario_args[@]+"${scenario_args[@]}"} &
launch_pid=$!

# `ros2 topic echo --field` prints a Python bool, so the value is `True` and not `true`; the
# match below is case-insensitive because the obvious `grep -qx true` never fires and leaves
# this loop spinning until its timeout with the simulator up and nothing happening.
#
# The coordinator answers its action server well before it has found the motion, gripper and
# attachment backends. Sending a goal in that window is rejected outright, so wait for the
# readiness it publishes rather than for the server to appear.
printf -- '-- waiting for the coordinator to report ready --\n'
if ! timeout 300 "$repo_root/scripts/with_workspace.bash" bash -c '
  until ros2 topic echo --once --field admission_ready \
      /restock_action_coordinator/status 2>/dev/null | grep -qix true; do
    sleep 2
  done'; then
  printf 'error: the coordinator never reported ready\n' >&2
  exit 1
fi

printf -- '\n-- autonomous mode loop active: evidence-driven shelf surveys, deficit-driven tray --\n'
printf -- '-- surveys with confirmation, then transfers until every lane is at target. Ctrl-C. --\n'
wait "$launch_pid"
