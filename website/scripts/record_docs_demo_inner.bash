#!/usr/bin/env bash
# Inner recorder: expects ROS/Gazebo env already active (nix develop + setup.bash).
set -euo pipefail

repo="$(cd "$(dirname "$0")/../.." && pwd)"
rec_dir="${REC_DIR:-/tmp/combeanie-docs-rec}"
cam_sdf="$repo/website/scripts/docs_side_camera.sdf"

cd "$repo"
set +u
# shellcheck disable=SC1091
source ros_ws/install/setup.bash
set -u

mkdir -p "$rec_dir"
rm -f "$rec_dir"/{sim.log,bridge.log,spawn.log,rec.log,motion.log,raw.mp4}

echo "launching simulation (gui:=false)"
ros2 launch restocker_bringup simulation.launch.py gui:=false cameras:=true wrist_camera:=false \
  >"$rec_dir/sim.log" 2>&1 &
sim_pid=$!
export SIM_PID=$sim_pid

echo "waiting for controllers + /overhead_camera/image"
for i in $(seq 1 180); do
  if grep -q 'Successfully switched controllers' "$rec_dir/sim.log" 2>/dev/null; then
    topics="$(ros2 topic list 2>/dev/null || true)"
    if echo "$topics" | grep -q '/overhead_camera/image' \
      && echo "$topics" | grep -q '/joint_states'; then
      echo "ready after ${i}s"
      break
    fi
  fi
  sleep 1
  if ! kill -0 "$sim_pid" 2>/dev/null; then
    echo "simulation exited early" >&2
    tail -80 "$rec_dir/sim.log" >&2
    exit 1
  fi
  if [[ $i -eq 180 ]]; then
    echo "timeout waiting for sim" >&2
    tail -40 "$rec_dir/sim.log" >&2
    ros2 topic list 2>/dev/null | head -40 >&2 || true
    exit 1
  fi
done

sleep 5

echo "spawning docs_side_camera"
gz service -s /world/restocking/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 8000 \
  --req "sdf_filename: \"$cam_sdf\", name: \"docs_side_camera\"" \
  | tee "$rec_dir/spawn.log"

sleep 2
echo "gz topics containing docs_side:"
gz topic -l | grep -i docs_side || true

echo "bridging docs_side_camera image"
ros2 run ros_gz_bridge parameter_bridge \
  '/docs_side_camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  --ros-args -r /docs_side_camera:=/docs_side_camera/image \
  >"$rec_dir/bridge.log" 2>&1 &

topic=""
for i in $(seq 1 40); do
  topics="$(ros2 topic list 2>/dev/null || true)"
  if echo "$topics" | grep -Fxe '/docs_side_camera/image' >/dev/null; then
    topic='/docs_side_camera/image'
    echo "bridge ready after ${i}s"
    break
  fi
  if echo "$topics" | grep -Fxe '/docs_side_camera' >/dev/null; then
    topic='/docs_side_camera'
    echo "bridge ready (bare topic) after ${i}s"
    break
  fi
  sleep 1
done

if [[ -z "$topic" ]]; then
  echo "no docs_side_camera image topic" >&2
  ros2 topic list | grep -i camera || true
  gz topic -l | grep -i camera || true
  cat "$rec_dir/bridge.log" >&2 || true
  exit 1
fi

echo "recording topic=$topic"
python3 "$repo/website/scripts/record_camera_topic.py" "$rec_dir/raw.mp4" \
  --topic "$topic" --seconds 45 --fps 20 \
  >"$rec_dir/rec.log" 2>&1 &
rec_pid=$!
sleep 2

python3 "$repo/website/scripts/docs_demo_motion.py" >"$rec_dir/motion.log" 2>&1 || {
  echo "motion failed" >&2
  tail -40 "$rec_dir/motion.log" >&2
}

wait "$rec_pid" || true
echo "recording process finished"

if [[ -n "${sim_pid:-}" ]]; then
  kill "$sim_pid" 2>/dev/null || true
fi

if [[ ! -s "$rec_dir/raw.mp4" ]]; then
  echo "raw.mp4 missing" >&2
  tail -50 "$rec_dir/rec.log" >&2 || true
  exit 1
fi

ffprobe -hide_banner "$rec_dir/raw.mp4" 2>&1 | tee "$rec_dir/probe.txt" | head -20
