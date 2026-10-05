#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Build the offline planning probe, expand the descriptions it plans against, and run one sweep.
# All paths are derived from the repository root.
#
#   ./tools/diagnostics/planning_probe/run.bash --mode geometry
#   ./tools/diagnostics/planning_probe/run.bash --mode reach --from 0.30 --to 0.40 --step 0.005
#
# Everything after the script name is passed to the probe unchanged. See
# tools/diagnostics/README.md for what each mode measures and which shipped constant it backs.

set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
probe_src="$repo_root/tools/diagnostics/planning_probe"
# Under artifacts/, which is gitignored and removed by `just clean-all`.
work="${RESTOCKER_PROBE_WORKDIR:-$repo_root/artifacts/planning_probe}"
install="$work/install"
binary="$install/restocker_planning_probe/lib/restocker_planning_probe/planning_probe"

description="$repo_root/ros_ws/src/restocker_description"
moveit_config="$repo_root/ros_ws/src/restocker_moveit_config"
task_executor="$repo_root/ros_ws/src/restocker_task_executor"

if ! command -v colcon >/dev/null 2>&1; then
  echo "planning_probe: colcon is not on PATH; enter the development shell first" >&2
  exit 1
fi

mkdir -p "$work/desc"

# The probe has its own install space, separate from ros_ws/install, so building it does not
# disturb that workspace and `just clean` does not delete it mid-sweep. restocker_description is
# built there too because the xacro expansions below resolve `$(find restocker_description)`
# through the ament index. Rebuilds are unconditional; colcon skips unchanged packages.
colcon --log-base "$work/log" build \
  --base-paths "$repo_root/ros_ws/src" \
  --packages-select restocker_description \
  --build-base "$work/build" --install-base "$install" >"$work/build.log" 2>&1 ||
  { cat "$work/build.log" >&2; exit 1; }
colcon --log-base "$work/log" build \
  --base-paths "$probe_src" \
  --build-base "$work/build" --install-base "$install" >>"$work/build.log" 2>&1 ||
  { cat "$work/build.log" >&2; exit 1; }

# colcon's generated setup script reads COLCON_TRACE unguarded, so `set -u` aborts on it.
set +u
# shellcheck disable=SC1091
source "$install/setup.bash"
set -u

xacro "$description/urdf/restocker_planning.urdf.xacro" -o "$work/desc/robot.urdf"
xacro "$description/urdf/workcell.urdf.xacro" -o "$work/desc/workcell.urdf"

# The workcell pose (the launch's world-to-shelf transform) is read from the baseline scenario's
# `workcell_pose`.
workcell_pose="$(
  python3 - "$repo_root/ros_ws/src/restocker_gazebo/config/baseline_products.yaml" <<'PY'
import sys

import yaml

with open(sys.argv[1], encoding="utf-8") as handle:
    scenario = yaml.safe_load(handle)
print(",".join(str(float(value)) for value in scenario["workcell_pose"]))
PY
)"

exec "$binary" \
  --robot-urdf "$work/desc/robot.urdf" \
  --srdf "$moveit_config/config/restocker.srdf" \
  --workcell-urdf "$work/desc/workcell.urdf" \
  --gripper-geometry "$description/config/gripper_geometry.yaml" \
  --product-catalog "$description/config/product_collision_catalog.yaml" \
  --workcell-geometry "$description/config/workcell_geometry.yaml" \
  --coordinator-config "$task_executor/config/restock_action_coordinator.yaml" \
  --workcell-pose "$workcell_pose" \
  --padding "$(grep -o 'default_robot_padding": [0-9.]*' \
    "$moveit_config/launch/move_group.launch.py" | tail -1 | tr -d ' ' | cut -d: -f2)" \
  "$@"
