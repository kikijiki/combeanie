#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Measure what leasing a domain costs one launch test, paired and alternating.
#
# Paired and order-alternating: each pair runs both arms and which goes first flips every pair.
# A fixed order confounds the treatment, because the second arm always starts in the wake of the
# other simulator's teardown, and `test_planning_scene_projection_runtime` ends by asserting that
# the scene stops settling within five wall-clock seconds, which a busy moment breaks. Every run
# records the 1-minute load average and the machine-wide `gz sim` count at its start so the
# balance can be checked afterwards.
#
# The arms differ only in the ament test runner:
#   pinned  the stock runner with a fixed ROS_DOMAIN_ID and GZ_PARTITION
#   leased  scripts/run_isolated_test.py, which leases both
#
# Usage: measure_lease_cost.bash [pairs] [package] [test-relative-path]
set -uo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$repo_root/ros_ws"
pairs="${1:-10}"
package="${2:-restocker_bringup}"
relative="${3:-test/test_planning_scene_projection_runtime.py}"

if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "error: workspace is not built; run 'just build' first" >&2
  exit 2
fi

out="${RESTOCKER_MEASURE_DIR:-/tmp/restocker-lease-cost}"
rm -rf "$out"
mkdir -p "$out"
records="$out/records.tsv"
printf 'pair\tarm\torder\tload_at_start\tgz_at_start\texit\tseconds\n' > "$records"

pinned_domain="${RESTOCKER_PINNED_DOMAIN:-45}"
pinned_partition="restocker_scene_projection_test"
stock_runner='import sys; import ament_cmake_test; sys.exit(ament_cmake_test.main(sys.argv[1:]))'

load_now() { awk '{print $1}' /proc/loadavg; }

# `pgrep -c ... || echo 0` prints "0" twice at a zero count and breaks a TSV row, so count lines.
# Anchored, because an unanchored `-f 'gz sim'` also matches the asking shell.
gz_now() { pgrep -f '^gz sim ' 2>/dev/null | wc -l; }

run_arm() {
  local pair="$1" arm="$2" order="$3"
  local label="${arm}-${pair}"
  local load gz started finished status=0
  load="$(load_now)"
  gz="$(gz_now)"
  started="$(date +%s)"

  if [[ "$arm" == pinned ]]; then
    "$repo_root/scripts/with_simulator_slot.bash" \
      "$repo_root/scripts/with_workspace.bash" python3 -u \
      -c "$stock_runner" \
      "$out/$label.xunit.xml" --package-name "$package" \
      --output-file "$out/$label.txt" \
      --env "ROS_DOMAIN_ID=$pinned_domain" "GZ_PARTITION=$pinned_partition" \
      --command python3 -m launch_testing.launch_test "$workspace/src/$package/$relative" \
      "--junit-xml=$out/$label.xunit.xml" "--package-name=$package" \
      > "$out/$label.log" 2>&1 || status=$?
  else
    "$repo_root/scripts/with_simulator_slot.bash" \
      "$repo_root/scripts/with_workspace.bash" python3 -u \
      "$repo_root/scripts/run_isolated_test.py" \
      "$out/$label.xunit.xml" --package-name "$package" \
      --output-file "$out/$label.txt" \
      --env "GZ_PARTITION=$pinned_partition" \
      --command python3 -m launch_testing.launch_test "$workspace/src/$package/$relative" \
      "--junit-xml=$out/$label.xunit.xml" "--package-name=$package" \
      > "$out/$label.log" 2>&1 || status=$?
  fi

  finished="$(date +%s)"
  # Bash arithmetic: `bc` is not available in this environment.
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$pair" "$arm" "$order" "$load" "$gz" "$status" \
    "$(( finished - started ))" >> "$records"
  printf '  pair %2s %-6s (%s first) load=%s gz=%s exit=%s\n' \
    "$pair" "$arm" "$order" "$load" "$gz" "$status"
}

# Wait for the machine's simulator count to stop moving, then add a fixed margin, so the next run
# does not start while the previous Gazebo is still shutting down. The pinned arm has no tag to
# wait on, and a baseline captured once at startup goes stale when foreign simulators exit.
settle() {
  local deadline=$(( SECONDS + 90 )) previous stable=0
  previous="$(gz_now)"
  while (( SECONDS < deadline )); do
    sleep 3
    local current
    current="$(gz_now)"
    if [[ "$current" == "$previous" ]]; then
      stable=$(( stable + 1 ))
      (( stable >= 3 )) && break
    else
      stable=0
      previous="$current"
    fi
  done
  sleep 15
}

echo "simulators running on this machine at start: $(gz_now)"
echo "running $pairs pairs of $package/$relative"

for pair in $(seq 1 "$pairs"); do
  if (( pair % 2 == 1 )); then
    run_arm "$pair" leased "leased"
    settle
    run_arm "$pair" pinned "leased"
  else
    run_arm "$pair" pinned "pinned"
    settle
    run_arm "$pair" leased "pinned"
  fi
  settle
done

echo
python3 "$repo_root/scripts/analyse_lease_cost.py" "$out"
echo "MEASUREMENT DONE"
