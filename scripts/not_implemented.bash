#!/usr/bin/env bash
set -euo pipefail

command_name="${1:?command name is required}"
reason="${2:?reason is required}"

printf "error: '%s' is intentionally unavailable: %s\n" "$command_name" "$reason" >&2
exit 2
