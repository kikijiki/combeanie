#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Record the showcase campaign (the `just demo` scene: labelled products, five SKUs, a stocked shelf
# and a packed back room) from one 1920x1080 Gazebo camera. showcase_director.py cuts the camera as
# the robot's state changes (survey, the tray pick, the carry, the shelf placement) and writes the
# cuts to a log; showcase_edit.py keeps every frame (6 Hz of simulated time) of the close-ups,
# thins the rest and encodes the 1280x720 web assets.
set -uo pipefail

repo="$(cd "$(dirname "$0")/../.." && pwd)"
rec_dir="${REC_DIR:-}"
cam_model=showcase_cam_recording
scenario="$repo/ros_ws/src/restocker_gazebo/config/showcase_products.yaml"
world="$repo/ros_ws/src/restocker_gazebo/worlds/restocking_showcase.sdf"
lanes="$repo/ros_ws/src/restocker_world_state/config/showcase_lanes.yaml"
materials="$repo/ros_ws/src/restocker_gazebo/config/showcase_materials.yaml"
out_mp4="$repo/website/static/media/restocking-showcase.mp4"
out_poster="$repo/website/static/media/restocking-showcase-poster.png"
out_loop="$repo/website/static/media/restocking-showcase-loop.webp"
target_transfers="${REC_TARGET_TRANSFERS:-3}"
max_seconds="${REC_MAX_SECONDS:-1500}"
max_cycles="${REC_MAX_CYCLES:-12}"

if [[ ! "$target_transfers" =~ ^[1-9][0-9]*$ || ! "$max_seconds" =~ ^[1-9][0-9]*$ || ! "$max_cycles" =~ ^[1-9][0-9]*$ ]]; then
  printf 'REC_TARGET_TRANSFERS, REC_MAX_SECONDS and REC_MAX_CYCLES must be positive integers\n' >&2
  exit 2
fi

if (( target_transfers < 2 || max_cycles < target_transfers )); then
  printf 'require at least two transfers and REC_MAX_CYCLES >= REC_TARGET_TRANSFERS\n' >&2
  exit 2
fi

# Same showcase product/policy pair, world and ground-truth evidence pins as scripts/demo.bash.
# --print-launch validates configuration without acquiring a lease or starting ROS/Gazebo.
launch_args=(
  gui:=false rviz:=false planning_smoke:=false
  motion_enabled:=true controller_timeout:=30.0
  lane_observation_topic:=/perception/ground_truth/lane_observations
  object_observation_topic:=/perception/object_observations
  tray_overview_perception:=false tray_confirm_perception:=false
  product_observation_max_age:=2.0 selection_object_max_age_ms:=2000
  lane_evidence_validity_ms:=60000 maximum_observation_age_ms:=2000
  task_execution_timeout_ms:=180000 placement_require_column_growth:=false
  autonomous_campaign:=true campaign_restart_acknowledged:=true
  campaign_max_cycles:="$max_cycles" campaign_cycle_period:=0.0
  scenario_config:="$scenario" lane_semantics:="$lanes" world:="$world"
  bundle_scenario:=true scene_materials:="$materials"
)
if [[ "${1:-}" == "--print-launch" ]]; then
  printf '%s\n' "${launch_args[@]}" 'lane_policy_state:=<fresh-recording-directory>/lane-policy.absent'
  exit 0
fi
if [[ "${RESTOCKER_DOCS_SIM_SLOT:-}" != "1" ]]; then
  export RESTOCKER_DOCS_SIM_SLOT=1 RESTOCKER_SIM_SLOTS=1
  exec "$repo/scripts/with_simulator_slot.bash" bash "${BASH_SOURCE[0]}" "$@"
fi
if [[ -z "${RESTOCKER_ISOLATION_TAG:-}" ]]; then
  export GZ_PARTITION=restocker_docs
  exec python3 "$repo/scripts/domain_isolation.py" --seed docs-recording -- "${BASH_SOURCE[0]}" "$@"
fi

# Each capture owns its files and an absent persisted-policy path. Never truncate a prior run.
if [[ -n "$rec_dir" ]]; then
  mkdir "$rec_dir" || exit 2
else
  rec_dir="$(mktemp -d /tmp/combeanie-docs-rec.XXXXXX)"
fi
progress="$rec_dir/progress.txt"
frame_dir="$rec_dir/raw_frames"
launch_args+=(lane_policy_state:="$rec_dir/lane-policy.absent")
log() { printf '%s\n' "$*" | tee -a "$progress"; }

cd "$repo" || exit 1
set +u
# shellcheck disable=SC1091
source ros_ws/install/setup.bash
set -u

log "capture=$rec_dir domain=$ROS_DOMAIN_ID partition=$GZ_PARTITION"
log "GROUND-TRUTH showcase campaign (target=$target_transfers cycles=$max_cycles timeout=${max_seconds}s)"
setsid stdbuf -oL -eL ros2 launch restocker_bringup baseline.launch.py \
  "${launch_args[@]}" >"$rec_dir/sim.log" 2>&1 &
sim_pid=$!
director_pid=""
bridge_pid=""
status_pid=""

cleanup() {
  log 'cleanup'
  [[ -n "$director_pid" ]] && kill -TERM "$director_pid" 2>/dev/null || true
  [[ -n "$bridge_pid" ]] && kill -TERM "$bridge_pid" 2>/dev/null || true
  [[ -n "$status_pid" ]] && kill -TERM "$status_pid" 2>/dev/null || true
  kill -INT -- "-$sim_pid" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-$sim_pid" 2>/dev/null || true
  jobs -p | xargs -r kill 2>/dev/null || true
}
trap cleanup EXIT

log 'waiting for Gazebo entity service'
service_ready=0
for i in $(seq 1 180); do
  kill -0 "$sim_pid" 2>/dev/null || { log 'baseline exited'; exit 1; }
  if gz service -l 2>/dev/null | grep -Fqx '/world/restocking/create'; then
    log "entity service ready at poll $i"
    service_ready=1
    break
  fi
  sleep 1
done
[[ "$service_ready" -eq 1 ]] || { log 'Gazebo entity service not ready'; exit 1; }


python3 -u - <<'PY' >"$rec_dir/status.log" 2>"$rec_dir/status.err" &
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import AutonomousRestockCampaignStatus

rclpy.init()
node = Node("docs_campaign_status")
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
node.create_subscription(
    AutonomousRestockCampaignStatus,
    "/autonomous_restock_campaign/status",
    lambda msg: print(msg.sequence, msg.phase, msg.successful_transfers, flush=True),
    qos,
)
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
status_pid=$!

log 'waiting for the autonomous front sweep'
survey_started=0
for i in $(seq 1 180); do
  kill -0 "$sim_pid" 2>/dev/null || { log 'baseline exited before survey'; exit 1; }
  phase="$(tail -1 "$rec_dir/status.log" 2>/dev/null | awk '{print $2}')"
  if [[ "$phase" == "1" ]]; then
    log "front sweep active at poll $i"
    survey_started=1
    break
  fi
  sleep 1
done
[[ "$survey_started" -eq 1 ]] || { log 'campaign did not enter front survey'; exit 1; }

log 'spawning the recording camera at the start of the survey'
# The sensor writes every frame as a PNG itself: a 6 MB image message every 170 ms does not survive
# the UDP-only ROS transport under load. The bridge exists only because a camera that nothing
# subscribes to is never rendered. Spawning after the scene is up keeps the products' appearing
# out of the footage.
mkdir -p "$frame_dir"
python3 "$repo/website/scripts/showcase_views.py" sdf "${REC_START_VIEW:-wide}" \
  --width 1920 --height 1080 --rate "${REC_RATE:-6}" --fov 0.95 \
  --save-dir "$frame_dir" >"$rec_dir/camera.sdf"
gz service -s /world/restocking/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 8000 \
  --req "sdf_filename: \"$rec_dir/camera.sdf\", name: \"$cam_model\"" \
  | tee "$rec_dir/spawn.log"
# The camera model is renamed above but the sensor keeps the view's topic name.
ros2 run ros_gz_bridge parameter_bridge \
  "/showcase_cam_${REC_START_VIEW:-wide}@sensor_msgs/msg/Image[gz.msgs.Image" \
  >"$rec_dir/bridge.log" 2>&1 &
bridge_pid=$!
# Frames arrive at REC_RATE per simulated second; the director cuts the camera from the robot's state.
python3 -u "$repo/website/scripts/showcase_director.py" "$frame_dir" "$cam_model" \
  "$rec_dir/shots.jsonl" >"$rec_dir/director.log" 2>&1 &
director_pid=$!
recording_started=$SECONDS
observed_transfers=0
log 'recording survey and evidence-selected transfers (2x time compression)'

while (( SECONDS - recording_started < max_seconds )); do
  kill -0 "$sim_pid" 2>/dev/null || { log 'baseline exited while recording'; break; }
  read -r _ phase transfers < <(tail -1 "$rec_dir/status.log" 2>/dev/null)
  if [[ "${transfers:-}" =~ ^[0-9]+$ ]] && (( transfers > observed_transfers )); then
    observed_transfers=$transfers
    log "campaign successful_transfers=$observed_transfers phase=${phase:-unknown}"
  fi
  if (( observed_transfers >= target_transfers )); then
    log 'target transfer count observed; keeping ten seconds of settling motion'
    sleep 10
    break
  fi
  if [[ "$phase" == "7" ]]; then
    log "campaign entered PHASE_BLOCKED after $observed_transfers transfers"
    break
  fi
  sleep 2
done

kill -TERM "$bridge_pid" 2>/dev/null || true
bridge_pid=""

frame_count="$(find "$frame_dir" -name '*.png' 2>/dev/null | wc -l | tr -d ' ')"
log "frames saved=$frame_count successful_transfers=$observed_transfers"
kill -TERM "$director_pid" 2>/dev/null || true
if [[ "${frame_count:-0}" -lt 90 || "$observed_transfers" -lt 2 ]]; then
  log 'recording lacks a useful survey plus several transfers; refusing to overwrite site media'
  tail -80 "$rec_dir/sim.log" | tee -a "$progress"
  exit 2
fi

log 'editing and encoding'
poster_args=()
[[ -n "${REC_POSTER_FRAME:-}" ]] && poster_args=(--poster-frame "$REC_POSTER_FRAME")
python3 "$repo/website/scripts/showcase_edit.py" "$frame_dir" "$rec_dir/shots.jsonl" "$rec_dir/out" \
  "${poster_args[@]}" 2>&1 | tee -a "$progress"
[[ -s "$rec_dir/out/restocking-showcase.mp4" ]] || { log 'edit failed'; exit 1; }
size="$(stat -c %s "$rec_dir/out/restocking-showcase.mp4")"
if (( size >= 12000000 )); then
  log "encoded video is too large ($size bytes); refusing media handoff"
  exit 2
fi
duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$rec_dir/out/restocking-showcase.mp4")"
mv "$rec_dir/out/restocking-showcase.mp4" "$out_mp4"
mv "$rec_dir/out/restocking-showcase-poster.png" "$out_poster"
mv "$rec_dir/out/restocking-showcase-loop.webp" "$out_loop"
log "done duration=${duration}s size=${size}B transfers=$observed_transfers"
ffprobe -v error -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames \
  -show_entries format=duration,size -of default=nw=1 "$out_mp4" | tee -a "$progress"
