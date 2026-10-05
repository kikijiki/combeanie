set dotenv-load := false
set shell := ["bash", "-euo", "pipefail", "-c"]

repo_root := justfile_directory()
workspace := repo_root / "ros_ws"

# Aggregate compiler-job limit: build one package at a time with up to this many Ninja jobs.
# Both CMake and MAKEFLAGS are required: colcon-cmake otherwise appends host-core native flags
# that override CMake's cap. Override with a positive integer, e.g. `just jobs=2 build`.
jobs := "8"

default: help

# List the supported project commands.
help:
  @just --list --list-heading $'Available commands:\n'

# Validate the active environment and common runtime prerequisites.
doctor:
  @{{repo_root}}/scripts/doctor.bash

# Explain first-time setup without modifying global or user state.
bootstrap:
  @echo "1. Enable direnv for this repository: direnv allow"
  @echo "2. Confirm the shell: just doctor"
  @echo "3. Build the workspace: just build"

# Optional kache build cache (https://github.com/kunobi-ninja/kache). When `kache` is on PATH,
# `scripts/kache_cmake_args.bash` appends `--cmake-args <defaults from config/colcon-defaults.yaml>
# -DCMAKE_C_COMPILER_LAUNCHER=$(command -v kache) -DCMAKE_CXX_COMPILER_LAUNCHER=$(command -v kache)`,
# so every compile runs through kache's content-addressed cache via CMake's compiler-launcher
# mechanism (PATH shims would not fire: the Nix dev shell gives CMake absolute compiler paths).
# The cache key includes GCC's `-frandom-seed`, which the Nix dev shell injects through
# NIX_CFLAGS_COMPILE from the dev-shell profile id — a value that rotated with every profile
# rebuild and made each fresh shell a cold cache. It is pinned once, at the source: `flake.nix`'s
# shared mkShell sets `NIX_OUTPATH_USED_AS_RANDOM_SEED = "combeanie"`, which stdenv's
# reproducible-builds.sh setup hook reads instead of `$out`, so every dev shell exports
# `-frandom-seed=combeanie` (see website/docs/run/index.md — an explicit -frandom-seed on the
# command line is NOT an option: kache classifies it as an unsupported flag and the compile
# bypasses the cache entirely).
# When kache (or yq) is absent the script prints nothing and the command below is exactly what it
# was before this integration existed, so CI and the Nix sandbox (no kache, no daemon;
# `flake.nix`'s checks reference neither) build unchanged.
# Beware: colcon reconfigures a build dir whenever `--cmake-args` differs from `cmake_args.last`,
# so switching kache on or off forces one full reconfigure/rebuild per build dir — the launcher
# only takes effect from that build on. Cache keys also normalize machine-local paths only above
# the workspace root: absolute paths below it (build/install dir names, rosidl-generated sources)
# differ between differently named build dirs, so cross-directory rebuilds miss on those TUs by
# design.

# Build every ROS package with Ninja.
build:
  @[[ {{quote(jobs)}} =~ ^[1-9][0-9]*$ ]] || { echo "jobs must be a positive integer" >&2; exit 2; }
  CMAKE_BUILD_PARALLEL_LEVEL={{jobs}} MAKEFLAGS="-j{{jobs}} -l{{jobs}}" colcon --log-base {{workspace}}/log build --parallel-workers 1 --base-paths {{workspace}}/src --build-base {{workspace}}/build --install-base {{workspace}}/install $(bash {{repo_root}}/scripts/kache_cmake_args.bash)

# Build one ROS package and its workspace dependencies.
build-package package:
  @[[ {{quote(jobs)}} =~ ^[1-9][0-9]*$ ]] || { echo "jobs must be a positive integer" >&2; exit 2; }
  CMAKE_BUILD_PARALLEL_LEVEL={{jobs}} MAKEFLAGS="-j{{jobs}} -l{{jobs}}" colcon --log-base {{workspace}}/log build --parallel-workers 1 --base-paths {{workspace}}/src --build-base {{workspace}}/build --install-base {{workspace}}/install --packages-up-to "{{package}}" $(bash {{repo_root}}/scripts/kache_cmake_args.bash)

# `colcon test-result` totals every result file under the build tree, including stale ones from
# earlier runs, so the old results are deleted first to make the summary describe this run.
# Run all package tests and print complete failures.
#
# The test phase uses one colcon worker on purpose: rendered-world tests share the ctest
# RESOURCE_LOCK "restocker-renderer-simulator", and the two packages' ctest pools are separate
# processes, so serial packages keep a single Gazebo world machine-wide (live Gazebo campaigns
# on this project serialize; two concurrent heavy worlds were measured unsafe — the
# wave that ran them showed selection failing on stale robot telemetry). An in-test
# machine-wide wait was rejected because the queue delay is charged to the test TIMEOUT
# (measured: two 120 s tests killed ***Timeout 120.11 while queued). Each package's build and
# ctest still run up to {{jobs}} jobs, so compilation and non-renderer tests stay parallel.
test: build
  find {{workspace}}/build \( -name '*.xunit.xml' -o -name '*.gtest.xml' \) -delete
  {{repo_root}}/scripts/with_workspace.bash colcon --log-base {{workspace}}/log test --parallel-workers 1 --base-paths {{workspace}}/src --build-base {{workspace}}/build --install-base {{workspace}}/install --ctest-args -j{{jobs}} --label-exclude dense-gate
  colcon --log-base {{workspace}}/log test-result --test-result-base {{workspace}}/build

# Run tests for one package and print complete failures.
# The `dense-gate` label (test_dense_sensor_driven_runtime) is deselected here too: the dense
# gate only runs through `just dense-gate`, so a green default suite never implies its coverage.
test-package package: (build-package package)
  # See the note on `test`.
  find "{{workspace}}/build/{{package}}" \( -name '*.xunit.xml' -o -name '*.gtest.xml' \) -delete
  {{repo_root}}/scripts/with_workspace.bash colcon --log-base {{workspace}}/log test --parallel-workers {{jobs}} --base-paths {{workspace}}/src --build-base {{workspace}}/build --install-base {{workspace}}/install --packages-select "{{package}}" --ctest-args -j{{jobs}} --label-exclude dense-gate
  colcon --log-base {{workspace}}/log test-result --test-result-base "{{workspace}}/build/{{package}}"

# Run the dense sensor validation gate: exactly one run of test_dense_sensor_driven_runtime
# (dense fixture, sensor-driven default) under one machine-wide simulator slot.
# Kept OUT of `test`/`test-package` by the `dense-gate` ctest label (CMB-SPEC-14), so a green
# default suite never implies this coverage. Predeclared contract: ONE run, no retries; quiet
# start checked once and never waited on — load1 < 16, no live gz, no ppid-1 gz/ruby/ros2
# strays, workspace built — else NOT RUN (exit 2) with the measurements printed. Exit 0 GREEN;
# the ctest exit code RED, failing case named in ctest's output; infrastructure failure to
# start (slot/workspace) is NOT RUN. The test's own assertions and 4500 s timeout apply
# unchanged. Record tip, load1 start/end and the xunit path this prints with the result.
dense-gate:
  #!/usr/bin/env bash
  set -euo pipefail
  cd "{{repo_root}}"
  not_run() { echo "dense-gate: NOT RUN — $1" >&2; exit 2; }
  [[ -f ros_ws/install/setup.bash ]] || not_run "workspace not built; run 'just build' first"
  load1_start=$(awk '{print $1}' /proc/loadavg)
  gz_count=$( (pgrep -x gz || true) | wc -l)
  stray_count=$(ps -eo ppid=,comm= | awk '$1 == 1 && $2 ~ /^(gz|ruby|ros2)$/' | wc -l)
  awk -v l="$load1_start" 'BEGIN { exit !(l < 16) }' || not_run "quiet start required: load1=$load1_start >= 16"
  [[ "$gz_count" -eq 0 ]] || not_run "quiet start required: $gz_count live gz process(es)"
  [[ "$stray_count" -eq 0 ]] || not_run "quiet start required: $stray_count stray ppid-1 gz/ruby/ros2 process(es)"
  echo "dense-gate: tip=$(git rev-parse --short HEAD) start=$(date -u +%FT%TZ) load1=$load1_start gz=$gz_count strays=$stray_count slot=1"
  rc=0
  RESTOCKER_SIM_SLOTS=1 scripts/with_simulator_slot.bash scripts/with_workspace.bash ctest --test-dir ros_ws/build/restocker_bringup -R '^test_dense_sensor_driven_runtime$' --no-tests=error -j1 --output-on-failure || rc=$?
  load1_end=$(awk '{print $1}' /proc/loadavg)
  xunit=$(find ros_ws/build/restocker_bringup -name 'test_dense_sensor_driven_runtime*.xunit.xml' -newer ros_ws/install/setup.bash 2>/dev/null | head -1 || true)
  echo "dense-gate: end=$(date -u +%FT%TZ) load1=$load1_end rc=$rc xunit=${xunit:-none}"
  if [[ "$rc" -eq 0 ]]; then
    echo "dense-gate: GREEN — test_dense_sensor_driven_runtime passed (1/1 run)"
  elif [[ "$rc" -eq 1 || "$rc" -eq 2 ]]; then
    echo "dense-gate: NOT RUN — the test did not execute (simulator slot, workspace, or ctest selection refused the start); see the error above"
    exit 2
  else
    echo "dense-gate: RED — ctest exit $rc; failing case named in the output above (xunit: ${xunit:-none})"
  fi
  exit "$rc"

# Run repository policy checks plus shell and Python lint.
lint:
  python3 {{repo_root}}/scripts/check_repository.py
  shellcheck {{repo_root}}/scripts/*.bash
  ruff check {{repo_root}}/ros_ws/src {{repo_root}}/scripts {{repo_root}}/tools
  # ament's Python linters are what `just test` enforces and are stricter about docstrings
  # (D205/D213/D400/D415). Scoped to ros_ws/src: scripts/ and tools/ are not ament packages, and
  # ament_flake8's import ordering disagrees with ruff's isort.
  ament_flake8 {{repo_root}}/ros_ws/src
  ament_pep257 {{repo_root}}/ros_ws/src

# Format project-owned Nix, C++, and Python sources.
format:
  nixfmt {{repo_root}}/flake.nix
  {{repo_root}}/scripts/format_cpp.bash apply
  ruff check --fix {{repo_root}}/ros_ws/src {{repo_root}}/scripts {{repo_root}}/tools
  ruff format {{repo_root}}/ros_ws/src {{repo_root}}/scripts {{repo_root}}/tools

# Verify formatting without modifying files.
format-check:
  nixfmt --check {{repo_root}}/flake.nix
  {{repo_root}}/scripts/format_cpp.bash check
  ruff format --check {{repo_root}}/ros_ws/src {{repo_root}}/scripts {{repo_root}}/tools

# Remove normal ROS build artifacts.
clean:
  rm -rf {{workspace}}/build {{workspace}}/install {{workspace}}/log

# Remove ROS artifacts plus repository-local generated diagnostics.
clean-all: clean
  rm -rf {{repo_root}}/artifacts {{repo_root}}/bags {{repo_root}}/benchmark-results

# Every `ros2 launch` writes a timestamped directory under ~/.ros/log and nothing removes them.
# Not part of `clean` or `clean-all`: it is the only target that deletes outside the repository,
# and the directory is shared with all of the user's ROS work. It empties only the log directory
# and honours ROS_LOG_DIR and ROS_HOME.
# Delete every ROS 2 launch log under ~/.ros/log. Outside the repository; ask for it by name.
clean-ros-logs:
  #!/usr/bin/env bash
  set -euo pipefail
  log_dir="${ROS_LOG_DIR:-${ROS_HOME:-$HOME/.ros}/log}"
  # Guard against a mis-set ROS_LOG_DIR.
  if [[ -z "$log_dir" || "$log_dir" == "/" || "$log_dir" == "$HOME" ]]; then
    echo "refusing to clear an implausible ROS log directory: '$log_dir'" >&2
    exit 1
  fi
  if [[ ! -d "$log_dir" ]]; then
    echo "no ROS log directory at $log_dir; nothing to remove"
    exit 0
  fi
  entries="$(find "$log_dir" -mindepth 1 -maxdepth 1 | wc -l)"
  echo "removing $(du -sh "$log_dir" | cut -f1) across ${entries} entries in $log_dir"
  find "$log_dir" -mindepth 1 -maxdepth 1 -exec rm -rf {} +

# Launch the robot model in RViz.
launch-rviz *args: build
  {{repo_root}}/scripts/with_workspace.bash ros2 launch restocker_description view_robot.launch.py {{args}}

# The flake is referenced as a plain path, not `path:{{repo_root}}`: the `path:` form copies the
# working directory into the Nix store ignoring .gitignore (ros_ws/build, agent worktrees), which
# is slow and fails if anything writes into the tree mid-copy. The plain form resolves through the
# git tree. It cannot see an uncommitted edit to flake.nix itself; commit it, or run
# `nix develop path:. -c ...` while iterating. Sources, launch files and configs are read from
# disk at runtime and are unaffected.
# Optional campaign cycle limit: `just demo 3` runs at most three survey/restock cycles; zero (the
# default) keeps looping until Ctrl-C. Each cycle drains all currently compatible transfers.
# Watch the robot autonomously sweep the shelf and tray, restock until idle, then repeat.
# The default is the shipped sensor-driven path (no evidence pins: wrist lane survey + wrist tray
# confirm own the world state's ingest topics, ground truth demoted to /perception/ground_truth/*,
# overhead camera obstacles-only) on the sensor acceptance fixture. `just demo gt:=true` is the
# labelled ground-truth opt-in: dense fixture, ground truth pinned back onto the ingest topics.
# Rebuilds first: demo only sources ros_ws/install.
demo max_cycles="0" gt="false": build
  nix develop "{{repo_root}}#baseline" --command {{repo_root}}/scripts/demo.bash "{{max_cycles}}" "{{gt}}"

# Launch the Gazebo and ros2_control simulation.
launch-sim *args: build
  nix develop "{{repo_root}}#simulation" --command {{repo_root}}/scripts/with_workspace.bash ros2 launch restocker_bringup simulation.launch.py {{args}}

# Launch the deterministic Gazebo, control, and MoveIt baseline.
launch-baseline *args: build
  nix develop "{{repo_root}}#baseline" --command {{repo_root}}/scripts/with_workspace.bash ros2 launch restocker_bringup baseline.launch.py {{args}}

# Run the finite headless MoveIt execution and collision acceptance workflow.
planning-smoke *args:
  just launch-baseline gui:=false rviz:=false planning_smoke:=true planning_scene_projection:=false {{args}}

# Launch the simulation with the RGB-D perception pipeline producing object observations.
launch-perception *args: build
  nix develop "{{repo_root}}#simulation" --command {{repo_root}}/scripts/with_workspace.bash ros2 launch restocker_bringup simulation.launch.py perception:=true {{args}}

# Launch the complete system including optional services (not implemented).
launch-full:
  @{{repo_root}}/scripts/not_implemented.bash "launch-full" "optional services are not implemented"

# List controller-manager state when the baseline is running.
controllers:
  nix develop "{{repo_root}}#simulation" --command {{repo_root}}/scripts/with_workspace.bash ros2 control list_controllers

# Validate controller ownership and execute the deterministic smoke trajectory.
control-smoke *args: build
  nix develop "{{repo_root}}#simulation" --command {{repo_root}}/scripts/with_workspace.bash ros2 run restocker_control control_smoke_test {{args}}

# List current ROS topics with their types.
topics:
  {{repo_root}}/scripts/with_workspace.bash ros2 topic list --show-types

# Generate artifacts/frames.pdf from the live TF graph.
tf-tree:
  @{{repo_root}}/scripts/generate_tf_tree.bash

# Record all ROS topics under bags/<name>.
record-bag name:
  mkdir -p {{repo_root}}/bags
  {{repo_root}}/scripts/with_workspace.bash ros2 bag record --all --output {{repo_root}}/bags/{{name}}

# Replay a ROS bag.
replay-bag path:
  {{repo_root}}/scripts/with_workspace.bash ros2 bag play "{{path}}"

# A campaign leases its ROS domain and Gazebo partition like a launch test does. The lease is
# exclusive machine-wide, reaps that campaign's own orphans on the way in, and is recorded in every
# run's metadata. It covers the whole invocation, so `--repeat N` runs share one domain and can
# inherit the previous run's orphans; separate invocations, campaigns and worktrees cannot.
# Run one headless benchmark scenario. Extra args go to benchmark_run (--seed, --repeat).
# Rebuilds first, as `demo` does.
benchmark scenario *args: build
  RESTOCKER_REPOSITORY_ROOT={{repo_root}} nix develop "{{repo_root}}#baseline" --command python3 {{repo_root}}/scripts/domain_isolation.py --seed "benchmark-{{scenario}}" -- {{repo_root}}/scripts/with_workspace.bash ros2 run restocker_benchmarks benchmark_run "{{scenario}}" {{args}}

# Run every benchmark scenario `repeat` times, then aggregate the campaign.
benchmark-all repeat="1":
  @{{repo_root}}/scripts/benchmark_all.bash "{{repeat}}"

# Aggregate benchmark-results into summary.json, report.md and plots.
benchmark-report campaign="benchmark-results":
  RESTOCKER_REPOSITORY_ROOT={{repo_root}} nix develop "{{repo_root}}#baseline" --command {{repo_root}}/scripts/with_workspace.bash ros2 run restocker_benchmarks benchmark_report "{{campaign}}"

# Drive the predeclared paired UR10e transfer-reliability campaign: baseline_transfers and
# dense_restock_transfers interleaved one pair at a time (AB on odd pairs, BA on even), every
# attempt retained and classified, UR10e-only population, load annotated. Measurement, not a
# gate: exits 0 when it records failures. Leases its domain like `just benchmark`.
# One invocation writes one fresh campaign directory under benchmark-results/ and refuses a
# directory that already holds records, so no attempt can replace another. Extra args go to
# benchmark_paired (--pairs, --results-dir, --summarise-only, --dense-rerun). Rebuilds first,
# as `benchmark`.
benchmark-paired *args: build
  RESTOCKER_REPOSITORY_ROOT={{repo_root}} nix develop "{{repo_root}}#baseline" --command python3 {{repo_root}}/scripts/domain_isolation.py --seed "benchmark-paired" -- {{repo_root}}/scripts/with_workspace.bash ros2 run restocker_benchmarks benchmark_paired {{args}}

# Build the public Docusaurus site (website/).
docs:
  cd {{repo_root}}/website && bun install --frozen-lockfile && bun run typecheck && bun run build && bun run check-links

# Serve the public Docusaurus site with live reload (default 127.0.0.1:3000).
# For Tailscale/LAN binding: just docs-serve -- --host 0.0.0.0 --port 3100
docs-serve *args:
  cd {{repo_root}}/website && bun install --frozen-lockfile && bun run start {{args}}

# Print the ROS package dependency graph.
graph:
  colcon --log-base {{workspace}}/log graph --base-paths {{workspace}}/src

# Evaluate and build all pinned Nix checks, including a clean ROS build.
flake-check:
  nix flake check "{{repo_root}}" --print-build-logs
