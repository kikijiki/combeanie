#!/usr/bin/env bash
# Watch the system do the thing it exists to do: bring up the workcell, survey the lanes whose
# evidence needs it, and run the sensor-driven mode loop -- deficit-driven tray surveys with
# confirmation, transfers, and idle-until-evidence changes -- until every lane reaches its
# declared target. The optional first argument is a maximum mode-loop cycle count; zero is
# unbounded. The optional second argument is `gt`, the labelled ground-truth opt-in.
#
# The DEFAULT is the shipped sensor-driven path: no evidence pins at all, so this demo runs
# exactly what `baseline.launch.py` runs -- wrist lane survey and wrist tray confirm own the
# world state's ingest topics, simulator ground truth is demoted to /perception/ground_truth/*
# where only the error evaluators read it, and the overhead camera is on for obstacle detection
# only (its object pipeline stays off). It drives the sensor acceptance fixture
# (sensor_acceptance_products/lanes) -- three lanes, three transfers, then PHASE_FRONT_FULL and
# an idle loop. Camera identity also names the dense tray's repeated products (by stocking
# position), but the dense scenario has not yet completed a sensor-driven run, so the default
# stays on this fixture. The dense sensor-driven end-to-end check runs only as its own gate:
# `just dense-gate` (one serialized run; excluded by label from `just test` / `nix flake check`).
#
# `demo.bash 0 true` (just demo gt:=true) is the opt-in ground-truth demo: the dense fixture
# with 13 transfers, ground truth pinned back onto the ingest topics and both wrist tray duties
# off, so exactly one producer owns each topic. It is labelled every time it starts.
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

max_cycles="${1:-0}"
ground_truth="${2:-false}"
if [[ ! "$max_cycles" =~ ^[0-9]+$ ]]; then
  printf 'error: max_cycles must be a non-negative integer (zero runs until Ctrl-C)\n' >&2
  exit 2
fi
if [[ "$ground_truth" != "true" && "$ground_truth" != "false" ]]; then
  printf 'error: the ground-truth opt-in must be true or false, got %s\n' "$ground_truth" >&2
  exit 2
fi

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
  scenario_args=(
    scenario_config:="$repo_root/ros_ws/src/restocker_gazebo/config/dense_restock_products.yaml"
    lane_semantics:="$repo_root/ros_ws/src/restocker_world_state/config/dense_restock_lanes.yaml"
  )
  printf -- '-- GROUND-TRUTH MODE (opt-in: gt:=true) --\n'
  printf -- '-- dense fixture, 13 transfers; ground truth pinned onto the ingest topics and the\n'
  printf -- '-- wrist tray duties off, so each ingest topic has exactly one producer. --\n'
else
  # The shipped default: no evidence pins. baseline.launch.py's own defaults are the
  # sensor-driven configuration (ground truth demoted to /perception/ground_truth/*, wrist lane
  # survey and wrist tray confirm owning the ingest topics, 180 s evidence ages, overhead camera
  # obstacles-only). Leaving the pins out is the point: the demo runs what the product ships.
  evidence_args=()
  scenario_args=(
    scenario_config:="$repo_root/ros_ws/src/restocker_gazebo/config/sensor_acceptance_products.yaml"
    lane_semantics:="$repo_root/ros_ws/src/restocker_world_state/config/sensor_acceptance_lanes.yaml"
  )
  printf -- '-- SENSOR-DRIVEN MODE (the shipped default; no evidence pins) --\n'
  printf -- '-- wrist lane survey and wrist tray confirm own the evidence, ground truth is demoted\n'
  printf -- '-- to /perception/ground_truth/* for the evaluators, the overhead camera is on for\n'
  printf -- '-- obstacle detection only. Three lanes, three transfers, then PHASE_FRONT_FULL. --\n'
  printf -- '-- Opt in to the dense ground-truth demo with: just demo gt:=true --\n'
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
