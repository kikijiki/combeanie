#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Demonstrate that two workspaces running the same launch test at the same time do not interfere,
# and that a run cannot see the orphans of the run before it.
#
# Both hazards used to be guaranteed rather than possible. Every launch test carried a fixed
# ROS_DOMAIN_ID in its CMakeLists, so two worktrees running the same test landed on one domain by
# construction, and a run killed hard left nodes on the domain its own next run would reuse. This
# script reproduces each hazard and shows it closed.
#
# `--pinned` is what makes it a before/after rather than an assertion. It runs the pre-change
# invocation: the stock ament test runner with a fixed `--env ROS_DOMAIN_ID`, exactly what the
# CMakeLists used to write. So the failure the fix removes can still be produced on demand from
# the fixed tree, with no need to check out the old one.
#
#   concurrent   two workspaces, same launch test, at the same time
#                  --pinned: both on one fixed domain    -> expected to FAIL
#                  default:  each leases its own domain  -> expected to PASS
#   orphan       a simulator orphaned by a kill -9, then the same test run again
#                  --pinned: second run on the orphan's fixed domain -> shares it
#                  default:  second run leases and reaps             -> cannot share it
#
# Usage: prove_domain_isolation.bash [concurrent|orphan|both] [--pinned]
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$repo_root/ros_ws"
case_name="${1:-both}"
pinned="${2:-}"

if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "error: workspace is not built; run 'just build' first" >&2
  exit 2
fi

scratch="${RESTOCKER_PROOF_DIR:-/tmp/restocker-isolation-proof}"
mkdir -p "$scratch"

# The launch test used for the concurrent case. It is the cheapest one in the tree that can
# actually observe a collision rather than merely tolerate one: it asserts an exact publisher
# count, an exact subscriber count and an exact message count on its topics, so a second copy of
# itself on the same domain is visible to it. It starts no simulator, so it costs seconds.
concurrent_package="restocker_control"
concurrent_test="test/test_joint_state_telemetry_adapter.py"

# The launch test used for the orphan case. Ninety seconds, one simulator.
orphan_package="restocker_gazebo"
orphan_test="test/test_simulation_runtime.py"

gate_dir="$scratch/gate"

# The pre-change invocation, reconstructed exactly: the stock ament runner, and the domain and
# partition handed to it as fixed `--env` entries, which is what the CMakeLists used to write.
# Forcing a domain through RESTOCKER_FORCE_DOMAIN_ID would not reproduce it, the lease is still
# exclusive, so two runs asking for one domain take turns instead of colliding, which is the fix
# working rather than the defect showing.
legacy_domain="${RESTOCKER_PROOF_DOMAIN:-95}"
legacy_partition="restocker_proof_fixed_partition"
stock_runner='import sys; import ament_cmake_test; sys.exit(ament_cmake_test.main(sys.argv[1:]))'

# `leased` or `legacy`, set by whichever case is running. The orphan case needs both in one run:
# its seeded run must be leased so it carries a tag the teardown can find, while the run that
# follows it is what changes between before and after.
run_mode="leased"

run_launch_test() {
  # Run one launch test the way ctest does, through the isolation runner, with the same
  # argument shape ament_add_test uses, so the domain is leased and recorded exactly as it is
  # in a real run. A fourth argument of `slot` takes a simulator slot for the duration; a fifth
  # of `gate` holds the launch at a rendezvous until every gated run has reached it.
  #
  # The rendezvous is what makes the concurrent case deterministic rather than lucky. The
  # telemetry adapter test runs in about a second and the two runs' start-up staggers by more
  # than that under load, so without a gate they take turns and a collision that is
  # guaranteed by construction never happens.
  local package="$1" relative="$2" label="$3" slot="${4:-}" gate="${5:-}"
  local results="$scratch/$label"
  local -a prefix=()
  local rendezvous=""
  mkdir -p "$results"
  rm -f "$results/$label.txt"
  if [[ "$slot" == "slot" ]]; then
    prefix=("$repo_root/scripts/with_simulator_slot.bash")
  fi
  if [[ -n "$gate" ]]; then
    mkdir -p "$gate"
    rendezvous=": > '$gate/$label';
      for _ in \$(seq 1 600); do
        (( \$(find '$gate' -maxdepth 1 -type f | wc -l) >= 2 )) && break
        sleep 0.05
      done; "
  fi
  local -a runner=("$repo_root/scripts/run_isolated_test.py")
  local -a fixed_env=()
  if [[ "$run_mode" == "legacy" ]]; then
    runner=(-c "$stock_runner")
    fixed_env=(--env "ROS_DOMAIN_ID=$legacy_domain" "GZ_PARTITION=$legacy_partition")
  fi

  "${prefix[@]}" "$repo_root/scripts/with_workspace.bash" python3 -u \
    "${runner[@]}" \
    "$results/$label.xunit.xml" \
    --package-name "$package" \
    --output-file "$results/$label.txt" \
    "${fixed_env[@]}" \
    --command bash -c "${rendezvous}exec \"\$0\" \"\$@\"" \
    python3 -m launch_testing.launch_test \
    "$workspace/src/$package/$relative" \
    "--junit-xml=$results/$label.xunit.xml" \
    "--package-name=$package" \
    > "$results/$label.console" 2>&1
}

# Kill a run's Python launcher processes by pid and leave everything else it started running.
# That is what a ctest timeout or a Ctrl-C produces: the launcher dies, the simulator, the bridge
# and the static transform publishers do not. Selection happens inside the set of processes
# already proven to carry this run's own tag, so nothing outside this run can be reached.
orphan_the_run() {
  python3 -c "
import os
import signal
import sys
sys.path.insert(0, '$repo_root/scripts')
import domain_isolation

killed, spared = [], []
for pid in domain_isolation.tagged_pids(sys.argv[1]):
    try:
        name = open(f'/proc/{pid}/comm').read().strip()
    except OSError:
        continue
    if not name.startswith('python'):
        spared.append((pid, name))
        continue
    try:
        os.kill(pid, signal.SIGKILL)
        killed.append((pid, name))
    except OSError:
        pass
print(f'   kill -9 by pid: {killed}')
print(f'   left running:   {spared}')
" "$1"
}

tagged_process_count() {
  python3 -c "
import sys
sys.path.insert(0, '$repo_root/scripts')
import domain_isolation
print(len(domain_isolation.tagged_pids(sys.argv[1])))
" "$1"
}

# Count a run's non-Python processes: the simulator, the bridges, the state publishers. Waiting
# for these is what says the launch is up, rather than sleeping a guessed interval,
# test_simulation_runtime finishes in well under a minute, so a fixed sleep can miss it entirely.
tagged_node_count() {
  python3 -c "
import sys
sys.path.insert(0, '$repo_root/scripts')
import domain_isolation

count = 0
for pid in domain_isolation.tagged_pids(sys.argv[1]):
    try:
        name = open(f'/proc/{pid}/comm').read().strip()
    except OSError:
        continue
    if not name.startswith('python'):
        count += 1
print(count)
" "$1"
}

report_domain() {
  local label="$1"
  grep -m1 -o 'ROS_DOMAIN_ID=[0-9]*' "$scratch/$label/$label.txt" 2>/dev/null || echo "ROS_DOMAIN_ID=?"
}

prove_concurrent() {
  # Repeated rather than single-shot. The rendezvous aligns the
  # two runs to the same instant, but each then spends one to three seconds importing rclpy and
  # launch_testing before its node exists, and the window in which this test can observe a
  # foreign node is about a second long. A single pair can miss each other entirely on a loaded
  # machine, measured, twice. Iterating puts the two sides through enough phase relationships
  # that a collision guaranteed by construction happens, and one failure on either side
  # is the result: these runs must never see each other.
  local iterations="${RESTOCKER_PROOF_ITERATIONS:-6}"
  echo "== concurrent: two workspaces, same launch test, at the same time =="
  echo "   $iterations iterations per side"
  if [[ "$pinned" == "--pinned" ]]; then
    run_mode="legacy"
    echo "   pre-change path: stock runner, fixed ROS_DOMAIN_ID=$legacy_domain for both"
  fi
  rm -rf "$gate_dir"

  run_side() {
    local side="$1" failures=0 index status
    for index in $(seq 1 "$iterations"); do
      status=0
      run_launch_test "$concurrent_package" "$concurrent_test" \
        "${side}_${index}" "" "$gate_dir/iteration_${index}" || status=$?
      (( status == 0 )) || failures=$(( failures + 1 ))
    done
    return "$failures"
  }

  local first_failures=0 second_failures=0
  run_side workspace_a & local a=$!
  run_side workspace_b & local b=$!
  wait "$a" || first_failures=$?
  wait "$b" || second_failures=$?

  echo "   workspace A: $(report_domain workspace_a_1) $first_failures/$iterations failed"
  echo "   workspace B: $(report_domain workspace_b_1) $second_failures/$iterations failed"
  if (( first_failures == 0 && second_failures == 0 )); then
    echo "   RESULT: every run passed -- the two workspaces did not see each other"
    return 0
  fi
  echo "   RESULT: runs failed -- the two workspaces interfered"
  echo "   (consoles under $scratch/workspace_a_*/ and $scratch/workspace_b_*/)"
  return 1
}

prove_orphan() {
  echo "== orphan: a simulator that outlived its own run =="

  # Start the test, let its simulator come up, then kill the runner outright, so the simulator is
  # left behind exactly as a ctest timeout or a Ctrl-C leaves it.
  run_launch_test "$orphan_package" "$orphan_test" orphan_seed slot &
  local seed_pid=$!

  local seed_file="$scratch/orphan_seed/orphan_seed.txt"
  local seed_domain=""
  # Generous, because the seeded run queues for a simulator slot behind everything else on the
  # machine before it starts at all.
  for _ in $(seq 1 3600); do
    if [[ -f "$seed_file" ]]; then
      seed_domain="$(grep -m1 -o 'ROS_DOMAIN_ID=[0-9]*' "$seed_file" 2>/dev/null | cut -d= -f2 || true)"
    fi
    [[ -n "$seed_domain" ]] && break
    sleep 1
  done
  if [[ -z "$seed_domain" ]]; then
    echo "   could not determine the seeded run's domain; see $scratch/orphan_seed/" >&2
    kill -KILL "$seed_pid" 2>/dev/null || true
    return 1
  fi
  local tag
  tag="$(grep -m1 -o 'RESTOCKER_ISOLATION_TAG=[^ ]*' "$seed_file" | cut -d= -f2)"
  echo "   seeded run is on domain $seed_domain (tag $tag); waiting for its simulator"
  local nodes=0
  for _ in $(seq 1 180); do
    nodes="$(tagged_node_count "$tag")"
    (( nodes >= 4 )) && break
    sleep 1
  done
  echo "   the seeded run has $nodes simulator-side processes up"

  # kill -9 the launcher only. What it started is what the run leaves behind.
  orphan_the_run "$tag"
  kill -KILL "$seed_pid" 2>/dev/null || true
  wait "$seed_pid" 2>/dev/null || true
  sleep 3

  local orphans
  orphans="$(tagged_process_count "$tag")"
  echo "   after kill -9 the run left $orphans process(es) behind on domain $seed_domain"
  if (( orphans == 0 )); then
    echo "   nothing was orphaned, so this run of the case proves nothing; retry" >&2
    return 1
  fi

  if [[ "$pinned" == "--pinned" ]]; then
    # The pre-change second run: the same fixed domain the orphan is still publishing on, and no
    # teardown that could remove it. That is the state that cost twenty benchmark runs.
    legacy_domain="$seed_domain"
    run_mode="legacy"
    echo "   pre-change path: the next run takes the orphan's domain $seed_domain and reaps nothing"
  fi
  local status=0
  run_launch_test "$orphan_package" "$orphan_test" orphan_rerun slot || status=$?
  echo "   second run: $(report_domain orphan_rerun) exit=$status"

  local surviving
  surviving="$(tagged_process_count "$tag")"
  echo "   processes still alive under the seeded run's tag: $surviving"
  if [[ "$pinned" == "--pinned" ]]; then
    if (( surviving > 0 )); then
      echo "   RESULT: $surviving orphan(s) survived the second run and shared its domain"
      return 1
    fi
    echo "   RESULT: nothing survived; the case did not reproduce"
    return 1
  fi
  if (( status == 0 && surviving == 0 )); then
    echo "   RESULT: the second run passed and the orphans are gone"
    return 0
  fi
  echo "   RESULT: second run exit=$status, $surviving orphan(s) left"
  echo "   (see $scratch/orphan_rerun/orphan_rerun.console)"
  return 1
}

overall=0
case "$case_name" in
  concurrent) prove_concurrent || overall=1 ;;
  orphan) prove_orphan || overall=1 ;;
  both)
    prove_concurrent || overall=1
    prove_orphan || overall=1
    ;;
  *)
    echo "usage: prove_domain_isolation.bash [concurrent|orphan|both] [--pinned]" >&2
    exit 2
    ;;
esac
exit "$overall"
