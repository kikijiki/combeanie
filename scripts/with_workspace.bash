#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
setup_file="$repo_root/ros_ws/install/setup.bash"

if [[ ! -f "$setup_file" ]]; then
  echo "error: workspace is not built; run 'just build' first" >&2
  exit 2
fi

# The generated setup script contains references to variables that may be unset.
set +u
# The setup path is repository-relative and validated above, but is generated at build time.
# shellcheck disable=SC1090
source "$setup_file"
set -u

exec "$@"
