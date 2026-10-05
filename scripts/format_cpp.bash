#!/usr/bin/env bash
set -euo pipefail

mode="${1:-check}"
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source_root="$repo_root/ros_ws/src"
mapfile -d '' sources < <(
  find "$source_root" -type f \
    \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' -o -name '*.h' -o -name '*.hh' -o -name '*.hpp' \) \
    -print0 | sort -z
)

if ((${#sources[@]} == 0)); then
  echo "C++ formatter: no source files yet"
  exit 0
fi

case "$mode" in
  apply)
    ament_uncrustify --reformat "$source_root" || test "$?" -eq 1
    ament_uncrustify "$source_root"
    ;;
  check)
    ament_uncrustify "$source_root"
    ;;
  *)
    echo "usage: $0 {apply|check}" >&2
    exit 2
    ;;
esac
