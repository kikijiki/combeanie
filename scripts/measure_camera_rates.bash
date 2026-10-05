#!/usr/bin/env bash
# Measure one camera configuration end to end: patch the *installed* description, take a
# simulator slot, lease a ROS domain, launch, measure, append a JSON record.
#
# Only ros_ws/install is rewritten, so the sweep never shows up as a source change; `--restore`
# puts the installed copy back.
#
# Usage:
#   REPO=/path/to/worktree scripts/measure_camera_rates.bash <label> [configure options...]
#   REPO=/path/to/worktree scripts/measure_camera_rates.bash --restore
#
# Environment:
#   EXPECT_WRIST=0   the wrist sensor is not in this configuration; do not wait for its topics
#   TOPICS           comma-separated subset to subscribe to (default: all six, as the test does)
#   QOS              'sensor_data' (best-effort, as the test uses) or 'reliable'
#   COMPOSITION      'gazebo' (simulator alone) or 'bringup' (what test_camera_runtime launches)
#   RECORDS          where the NDJSON records are appended
#   LOGDIR           where each run's launch log is written (default: outside the worktree)
set -euo pipefail

repo="${REPO:?set REPO to the worktree root}"
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
records="${RECORDS:-$repo/camera-rate-records.ndjson}"

if [[ "${1:-}" == "--restore" ]]; then
  exec python3 "$here/configure_camera_sweep.py" --repo "$repo" --restore
fi

label="$1"
shift

# Fail before taking a slot: a half-built workspace would otherwise yield a record full of zeroes.
if [[ ! -f "$repo/ros_ws/install/setup.bash" ]]; then
  echo "error: $repo/ros_ws/install/setup.bash is missing; run 'just build' first" >&2
  exit 2
fi

logdir="${LOGDIR:-${TMPDIR:-/tmp}/camera-rate-logs}"
mkdir -p "$logdir"

python3 "$here/configure_camera_sweep.py" --repo "$repo" "$@"

# The slot is held for the whole measurement, which owns the simulator's lifetime.
"$here/with_simulator_slot.bash" \
  python3 "$here/domain_isolation.py" --seed "camera-rate-$label" -- \
  python3 "$here/measure_camera_rates.py" \
    --label "$label" \
    --repo "$repo" \
    --logdir "$logdir" \
    --expect-wrist "${EXPECT_WRIST:-1}" \
    --topics "${TOPICS:-}" \
    --qos "${QOS:-sensor_data}" \
    --composition "${COMPOSITION:-gazebo}" \
  >>"$records"

python3 "$here/summarise_camera_rates.py" "$records"
