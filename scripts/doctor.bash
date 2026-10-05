#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="$repo_root/ros_ws"
errors=0
warnings=0

pass() {
  printf 'ok: %s\n' "$1"
}

warn() {
  printf 'warning: %s\n' "$1" >&2
  warnings=$((warnings + 1))
}

fail() {
  printf 'error: %s\n' "$1" >&2
  errors=$((errors + 1))
}

if [[ "${RESTOCKER_FLAKE_SHELL:-}" == "1" ]]; then
  pass "pinned flake shell is active"
else
  fail "flake shell is not active; run 'direnv allow' or 'nix develop $repo_root'"
fi

if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
  pass "ROS_DISTRO is jazzy"
else
  fail "ROS_DISTRO must be jazzy (found '${ROS_DISTRO:-unset}')"
fi

case "${RESTOCKER_SHELL_PROFILE:-}" in
  foundation | simulation | baseline)
    pass "shell profile is ${RESTOCKER_SHELL_PROFILE}"
    ;;
  *)
    fail "RESTOCKER_SHELL_PROFILE is missing or unknown"
    ;;
esac

required_commands=(cmake colcon git just ninja python3 ros2 xacro)
for command_name in "${required_commands[@]}"; do
  if command -v "$command_name" >/dev/null 2>&1; then
    pass "$command_name is available"
  else
    fail "$command_name is missing from PATH"
  fi
done

required_packages=(ament_cmake robot_state_publisher rviz2 xacro)
for package_name in "${required_packages[@]}"; do
  if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
    pass "ROS package '$package_name' resolves"
  else
    fail "ROS package '$package_name' does not resolve"
  fi
done

if [[ "${RESTOCKER_SHELL_PROFILE:-}" == "simulation" || "${RESTOCKER_SHELL_PROFILE:-}" == "baseline" ]]; then
  simulation_packages=(controller_manager gz_ros2_control ros_gz_sim)
  for package_name in "${simulation_packages[@]}"; do
    if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      pass "simulation package '$package_name' resolves"
    else
      fail "simulation package '$package_name' does not resolve"
    fi
  done
fi

if [[ "${RESTOCKER_SHELL_PROFILE:-}" == "baseline" ]]; then
  planning_packages=(moveit_ros_move_group moveit_ros_planning moveit_ros_planning_interface)
  for package_name in "${planning_packages[@]}"; do
    if ros2 pkg prefix "$package_name" >/dev/null 2>&1; then
      pass "planning package '$package_name' resolves"
    else
      fail "planning package '$package_name' does not resolve"
    fi
  done
fi

if [[ -n "${WAYLAND_DISPLAY:-}" || -n "${DISPLAY:-}" ]]; then
  pass "graphical display is available"
else
  warn "no display detected; builds, tests and benchmarks work, but the GUI launch recipes (demo, launch-rviz, launch-sim, launch-baseline) exit at once because their graphical client is a required process"
fi

if [[ -r /dev/dri/renderD128 ]]; then
  pass "render node /dev/dri/renderD128 is accessible"
else
  warn "render node is unavailable; RViz may use software rendering"
fi

if [[ -f "$workspace/install/setup.bash" ]]; then
  pass "workspace install exists"
  if [[ ":${AMENT_PREFIX_PATH:-}:" == *":$workspace/install:"* ]]; then
    pass "workspace overlay is sourced"
  else
    warn "workspace overlay is not sourced; launch recipes source it automatically"
  fi
else
  warn "workspace has not been built yet"
fi

if [[ -d "$workspace/build" ]]; then
  stale_package=""
  while IFS= read -r manifest; do
    package_dir="$(dirname -- "$manifest")"
    package_name="$(basename -- "$package_dir")"
    configure_stamp="$workspace/build/$package_name/build.ninja"
    if [[ -f "$configure_stamp" ]] &&
      { [[ "$manifest" -nt "$configure_stamp" ]] || [[ "$package_dir/CMakeLists.txt" -nt "$configure_stamp" ]]; }; then
      stale_package="$package_name"
      break
    fi
  done < <(find "$workspace/src" -mindepth 2 -maxdepth 2 -name package.xml -print | sort)

  if [[ -n "$stale_package" ]]; then
    warn "package metadata for '$stale_package' is newer than its configure result; run 'just build'"
  else
    pass "build artifacts are not detectably stale"
  fi
fi

printf 'doctor: %d error(s), %d warning(s)\n' "$errors" "$warnings"
((errors == 0))
