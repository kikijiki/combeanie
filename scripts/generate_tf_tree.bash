#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
artifacts="$repo_root/artifacts"

mkdir -p "$artifacts"
cd "$artifacts"
"$repo_root/scripts/with_workspace.bash" ros2 run tf2_tools view_frames
