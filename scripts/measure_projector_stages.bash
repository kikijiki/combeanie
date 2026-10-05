#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Measure what each planning-scene projector stage costs, across repeated runs.
#
# The projector's reconcile cadence is bimodal: nominal cycles at reconcile_period_sec, and stalls
# pinned to multiples of service_timeout_sec (a deadline expiring, not contention). The status
# topic does not say which stage was waiting, so this runs
# `test_planning_scene_projection_runtime` repeatedly with the projector's logger at debug, which
# records each service stage's round trip, and keeps each log for `analyse_projector_stages.py`.
#
# Usage: measure_projector_stages.bash [runs]
set -uo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$repo_root/ros_ws"
runs="${1:-6}"
package=restocker_bringup
relative="test/test_planning_scene_projection_runtime.py"

if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "error: workspace is not built; run 'just build' first" >&2
  exit 2
fi

out="${RESTOCKER_MEASURE_DIR:-/tmp/restocker-projector-stages}"
mkdir -p "$out"
records="$out/records.tsv"
printf 'run\tload_at_start\tgz_at_start\texit\tseconds\n' > "$records"

load_now() { awk '{print $1}' /proc/loadavg; }
# `pgrep -c` prints its count *and* exits non-zero at zero, which writes a stray line into a TSV
# row. Count lines instead. Anchored, because an unanchored match also finds the asking shell.
gz_now() { pgrep -f '^gz sim ' 2>/dev/null | wc -l; }

for run in $(seq 1 "$runs"); do
  label="run-$(printf '%02d' "$run")"
  load="$(load_now)"
  gz="$(gz_now)"
  started="$SECONDS"
  status=0
  RESTOCKER_PROJECTOR_LOG_LEVEL=debug \
    "$repo_root/scripts/with_simulator_slot.bash" \
    "$repo_root/scripts/with_workspace.bash" python3 -u \
    "$repo_root/scripts/run_isolated_test.py" \
    "$out/$label.xunit.xml" --package-name "$package" \
    --output-file "$out/$label.txt" \
    --env "GZ_PARTITION=restocker_scene_projection_test" \
    --command python3 -m launch_testing.launch_test "$workspace/src/$package/$relative" \
    "--junit-xml=$out/$label.xunit.xml" "--package-name=$package" \
    > "$out/$label.log" 2>&1 || status=$?
  printf '%s\t%s\t%s\t%s\t%s\n' \
    "$label" "$load" "$gz" "$status" "$((SECONDS - started))" >> "$records"
  printf '  %s load=%s gz=%s exit=%s seconds=%s\n' \
    "$label" "$load" "$gz" "$status" "$((SECONDS - started))"
done

echo
echo "logs under $out"
