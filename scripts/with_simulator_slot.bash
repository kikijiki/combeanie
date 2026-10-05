#!/usr/bin/env bash
# Run a command holding one of a fixed number of simulator slots, machine-wide.
#
# Several agents share this machine, and a Gazebo run is the expensive thing on it. The obvious
# guard, poll `pgrep` and start if fewer than N are running, is a check-then-act race: when
# several pollers wake at the same moment they all see the same count and all proceed. That is
# how six agents produced five concurrent simulators against a two-slot budget, on a box whose
# measured failure *distribution* changes with load.
#
# This takes a real counting semaphore instead: N lock files, `flock -n` to claim one, and the
# descriptor held for the life of the command so the kernel releases it however the command dies.
# The lock lives outside any worktree because the point is to coordinate between them.
set -euo pipefail

slots="${RESTOCKER_SIM_SLOTS:-2}"
lock_dir="${RESTOCKER_SIM_LOCK_DIR:-/tmp/restocker-sim-slots}"
wait_seconds="${RESTOCKER_SIM_WAIT_SECONDS:-3600}"

if [[ $# -eq 0 ]]; then
  echo "usage: with_simulator_slot.bash <command> [args...]" >&2
  exit 2
fi

mkdir -p "$lock_dir"
deadline=$(( SECONDS + wait_seconds ))

while :; do
  for slot in $(seq 1 "$slots"); do
    exec {fd}>"$lock_dir/slot-$slot" || continue
    if flock -n "$fd"; then
      printf 'simulator slot %d of %d acquired\n' "$slot" "$slots" >&2
      # The descriptor stays open for the command, so the slot is released on any exit path
      # including a kill -9 of this shell.
      "$@"
      exit $?
    fi
    exec {fd}>&-
  done
  if (( SECONDS >= deadline )); then
    echo "error: no simulator slot within ${wait_seconds}s (${slots} slots, ${lock_dir})" >&2
    exit 1
  fi
  sleep 5
done
