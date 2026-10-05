#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Record the autonomous dense campaign from a low-rate Gazebo side camera. Frames are sampled at
# 2 fps and encoded at 8 fps (4x time compression).
set -uo pipefail

repo="$(cd "$(dirname "$0")/../.." && pwd)"
rec_dir="${REC_DIR:-}"
cam_sdf="$repo/website/scripts/docs_side_camera.sdf"
dense_scenario="$repo/ros_ws/src/restocker_gazebo/config/dense_restock_products.yaml"
dense_lanes="$repo/ros_ws/src/restocker_world_state/config/dense_restock_lanes.yaml"
out_mp4="$repo/website/static/media/restocking-demo.mp4"
out_poster="$repo/website/static/media/restocking-demo-poster.png"
out_loop="$repo/website/static/media/restocking-demo-loop.webp"
target_transfers="${REC_TARGET_TRANSFERS:-3}"
max_seconds="${REC_MAX_SECONDS:-720}"
max_cycles="${REC_MAX_CYCLES:-12}"

if [[ ! "$target_transfers" =~ ^[1-9][0-9]*$ || ! "$max_seconds" =~ ^[1-9][0-9]*$ || ! "$max_cycles" =~ ^[1-9][0-9]*$ ]]; then
  printf 'REC_TARGET_TRANSFERS, REC_MAX_SECONDS and REC_MAX_CYCLES must be positive integers\n' >&2
  exit 2
fi

if (( target_transfers < 2 || max_cycles < target_transfers )); then
  printf 'require at least two transfers and REC_MAX_CYCLES >= REC_TARGET_TRANSFERS\n' >&2
  exit 2
fi

# Same dense product/policy pair and evidence pins as scripts/demo.bash's gt=true path.
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
  scenario_config:="$dense_scenario" lane_semantics:="$dense_lanes"
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
log "GROUND-TRUTH dense campaign (target=$target_transfers cycles=$max_cycles timeout=${max_seconds}s)"
setsid stdbuf -oL -eL ros2 launch restocker_bringup baseline.launch.py \
  "${launch_args[@]}" >"$rec_dir/sim.log" 2>&1 &
sim_pid=$!
dump_pid=""
bridge_pid=""
status_pid=""

cleanup() {
  log 'cleanup'
  [[ -n "$dump_pid" ]] && kill -TERM "$dump_pid" 2>/dev/null || true
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

log 'spawning low-rate side camera before coordinator admission'
gz service -s /world/restocking/create --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean --timeout 8000 \
  --req "sdf_filename: \"$cam_sdf\", name: \"docs_side_camera\"" \
  | tee "$rec_dir/spawn.log"
sleep 2

ros2 run ros_gz_bridge parameter_bridge \
  '/docs_side_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args -r /docs_side_camera:=/docs_side_camera/image \
  >"$rec_dir/bridge.log" 2>&1 &
bridge_pid=$!
topic=""
for i in $(seq 1 40); do
  topics="$(ros2 topic list --no-daemon 2>/dev/null || true)"
  if echo "$topics" | grep -Fqx '/docs_side_camera/image'; then
    topic='/docs_side_camera/image'
    break
  fi
  if echo "$topics" | grep -Fqx '/docs_side_camera'; then
    topic='/docs_side_camera'
    break
  fi
  sleep 1
done
[[ -n "$topic" ]] || { log 'no side-camera ROS topic'; exit 1; }
log "camera topic=$topic"

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

mkdir -p "$frame_dir"
python3 -u "$repo/website/scripts/dump_image_frames.py" "$topic" "$frame_dir" \
  --seconds "$max_seconds" --every-n 4 >"$rec_dir/frames.log" 2>&1 &
dump_pid=$!
recording_started=$SECONDS
observed_transfers=0
log 'recording survey and evidence-selected transfers (4x time compression)'

while (( SECONDS - recording_started < max_seconds )); do
  kill -0 "$sim_pid" 2>/dev/null || { log 'baseline exited while recording'; break; }
  kill -0 "$dump_pid" 2>/dev/null || { log 'frame dump exited while recording'; break; }
  read -r _ phase transfers < <(tail -1 "$rec_dir/status.log" 2>/dev/null)
  if [[ "${transfers:-}" =~ ^[0-9]+$ ]] && (( transfers > observed_transfers )); then
    observed_transfers=$transfers
    log "campaign successful_transfers=$observed_transfers phase=${phase:-unknown}"
  fi
  if (( observed_transfers >= target_transfers )); then
    log 'target transfer count observed; keeping five seconds of settling motion'
    sleep 5
    break
  fi
  if [[ "$phase" == "7" ]]; then
    log "campaign entered PHASE_BLOCKED after $observed_transfers transfers"
    break
  fi
  sleep 2
done

kill -TERM "$dump_pid" 2>/dev/null || true
wait "$dump_pid" 2>/dev/null || true
dump_pid=""

frame_count="$(find "$frame_dir" -name 'frame_*.raw' 2>/dev/null | wc -l | tr -d ' ')"
log "frames saved=$frame_count successful_transfers=$observed_transfers"
if [[ "${frame_count:-0}" -lt 90 || "$observed_transfers" -lt 2 ]]; then
  log 'recording lacks a useful survey plus several transfers; refusing to overwrite site media'
  tail -80 "$rec_dir/sim.log" | tee -a "$progress"
  exit 2
fi

log 'encoding sampled frames'
python3 -u "$repo/website/scripts/frames_to_mp4.py" "$frame_dir" "$rec_dir/raw.mp4" --fps 8 \
  >"$rec_dir/encode.log" 2>&1 || { log 'encode failed'; cat "$rec_dir/encode.log"; exit 1; }

# Keep every accepted frame: keeping only the beginning could omit the late transfers that
# qualified this capture. The size guard applies to the complete encoded clip.
ffmpeg -y -i "$rec_dir/raw.mp4" -vf 'scale=960:-2' \
  -c:v libx264 -crf 23 -preset medium -an -movflags +faststart "$rec_dir/final.mp4" || exit 1
size="$(stat -c %s "$rec_dir/final.mp4")"
if (( size >= 25000000 )); then
  log "encoded video is too large ($size bytes); refusing media handoff"
  exit 2
fi

duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$rec_dir/final.mp4")"
poster_at="$(python3 -c "print(max(1.0, min(float('$duration') * 0.55, float('$duration') - 1.0)))")"
loop_at="$(python3 -c "print(max(0.0, min(float('$duration') * 0.65, float('$duration') - 6.0)))")"
ffmpeg -y -ss "$poster_at" -i "$rec_dir/final.mp4" -frames:v 1 -update 1 "$rec_dir/poster.png" || exit 1
ffmpeg -y -ss "$loop_at" -t 6 -i "$rec_dir/final.mp4" \
  -vf 'fps=8,scale=640:-2' -loop 0 -an "$rec_dir/loop.webp" || exit 1
ffmpeg -y -ss "$poster_at" -i "$rec_dir/final.mp4" -frames:v 1 -update 1 "$rec_dir/review.png" || exit 1
mv "$rec_dir/final.mp4" "$out_mp4"
mv "$rec_dir/poster.png" "$out_poster"
mv "$rec_dir/loop.webp" "$out_loop"
log "done duration=${duration}s size=${size}B transfers=$observed_transfers"
ffprobe -v error -show_entries stream=codec_name,width,height,r_frame_rate,nb_frames \
  -show_entries format=duration,size -of default=nw=1 "$out_mp4" | tee -a "$progress"
