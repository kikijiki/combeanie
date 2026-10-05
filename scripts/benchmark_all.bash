#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Run every benchmark scenario the stated number of times, then aggregate the campaign.
#
# Each run is a full headless launch of Gazebo, the controllers, MoveIt and the coordinator, so
# this is minutes per run and not seconds. Runs are strictly sequential: two simulators on one
# machine compete for cores and the timing metrics stop meaning anything.
set -euo pipefail

repeat="${1:-1}"
if ! [[ "$repeat" =~ ^[1-9][0-9]*$ ]]; then
  echo "usage: just benchmark-all [repeat]" >&2
  exit 2
fi

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"

# A failing scenario must not stop the campaign: the campaign exists to collect them. The
# runner already exits 0 for a recorded failure, so this only guards a harness-level crash.
status=0
for scenario in baseline_transfers lane_routing; do
  just benchmark "$scenario" --repeat "$repeat" || status=1
done
# The seeded scenario needs a seed to be reproducible. One is fixed here so `just benchmark-all`
# is repeatable; sweep seeds with `just benchmark seeded_autonomous --seed 1 --seed 2`.
just benchmark seeded_autonomous --seed 1 --repeat "$repeat" || status=1

just benchmark-report
exit "$status"
