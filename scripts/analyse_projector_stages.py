#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Report planning-scene projector stage costs and what ran inside each cadence gap.

Gaps between the projector's logged status lines are not reconcile cycles: `publish_status` logs
only when the state, error code or detail text changes, and never logs `STATE_SYNCHRONIZING`. A
gap is therefore either a stalled projector or one reconciling with nothing new to report.

Using the per-stage round trips logged at debug level, this reports for each gap how many
reconcile cycles completed inside it and the slowest stage, plus the stage durations as a
distribution (a stage waiting out `service_timeout_sec` shows as a spike at the deadline).

Since Card 041 a read-only stage is *held* at the deadline instead of being cancelled there, so a
timeout record can also land at `service_total_timeout_sec` when the request is finally abandoned:
a spike at the deadline means the hold reported, and a second spike beyond it means the hold gave
up. Apply stages are unchanged -- they still wait out their deadline and then wait for the
response.
"""

from __future__ import annotations

from collections import defaultdict
from pathlib import Path
import re
import statistics
import sys

STAGE_LINE = re.compile(
    r"\[planning_scene_projector_node-\d+\]\s+\[DEBUG\]\s+\[(\d+\.\d+)\]"
    r"\s+\[planning_scene_projector\]:\s+stage timing:\s+stage=(\w+)\s+outcome=(\w+)"
    r"\s+seconds=([\d.]+)"
)
STATUS_LINE = re.compile(
    r"\[planning_scene_projector_node-\d+\]\s+\[INFO\]\s+\[(\d+\.\d+)\]"
    r"\s+\[planning_scene_projector\]:\s+projection status:\s+(.*)"
)

RECONCILE_PERIOD_SEC = 0.5
SERVICE_TIMEOUT_SEC = 2.0
# A stage is pinned to its deadline when within a reconcile period of it (the deadline is only
# noticed on a timer tick).
PINNED_FLOOR_SEC = SERVICE_TIMEOUT_SEC - 1e-3


def parse(text: str) -> tuple[list[tuple[float, str, str, float]], list[tuple[float, str]]]:
    """Return this log's stage records and its logged status lines, both in time order."""
    stages = [
        (float(m.group(1)), m.group(2), m.group(3), float(m.group(4)))
        for m in STAGE_LINE.finditer(text)
    ]
    statuses = [(float(m.group(1)), m.group(2).strip()) for m in STATUS_LINE.finditer(text)]
    return stages, statuses


def summarise(values: list[float]) -> str:
    """Format a one-line median/p90/max summary of a duration sample."""
    ordered = sorted(values)
    p90 = ordered[min(len(ordered) - 1, int(0.9 * len(ordered)))]
    return (
        f"{len(values):6d} {statistics.median(values):9.4f} {p90:9.4f} {max(values):9.4f}"
        f" {min(values):9.4f}"
    )


def main(argv: list[str]) -> int:
    """Summarise stage cost and cadence-gap content across every run log under a directory."""
    directory = Path(argv[0] if argv else "/tmp/restocker-projector-stages")
    logs = sorted(directory.glob("*.log"))
    if not logs:
        print(f"no run logs under {directory}", file=sys.stderr)
        return 2

    by_stage: dict[tuple[str, str], list[float]] = defaultdict(list)
    gaps: list[tuple[float, int, float, str, str]] = []

    for log in logs:
        stages, statuses = parse(log.read_text(errors="replace"))
        for _, stage, outcome, seconds in stages:
            by_stage[(stage, outcome)].append(seconds)
        for (earlier, detail), (later, next_detail) in zip(statuses, statuses[1:], strict=False):
            inside = [record for record in stages if earlier < record[0] <= later]
            # A cycle always opens with a snapshot request; later stages depend on whether a diff
            # was needed, so count snapshots, not records.
            started = sum(1 for record in inside if record[1] == "snapshot")
            worst = max((record[3] for record in inside), default=0.0)
            gaps.append((later - earlier, started, worst, detail, next_detail))

    if not by_stage:
        print(
            "no stage records found: run with RESTOCKER_PROJECTOR_LOG_LEVEL=debug",
            file=sys.stderr,
        )
        return 2

    print(f"{len(logs)} run logs under {directory}")
    print()
    print(
        f"{'stage/outcome':30s} {'count':>6s} {'median':>9s} {'p90':>9s} {'max':>9s} {'min':>9s}"
    )
    for (stage, outcome), values in sorted(by_stage.items()):
        print(f"{stage + '/' + outcome:30s} {summarise(values)}")

    pinned = {
        key: [value for value in values if value >= PINNED_FLOOR_SEC]
        for key, values in by_stage.items()
    }
    print()
    print(f"round trips at or past service_timeout_sec ({SERVICE_TIMEOUT_SEC:.1f} s):")
    total_pinned = sum(len(values) for values in pinned.values())
    for key, values in sorted(pinned.items()):
        if values:
            print(f"  {key[0]}/{key[1]:14s} {len(values)}")
    if not total_pinned:
        print("  none")

    print()
    print("cadence gaps between logged status lines, and what ran inside them:")
    print(f"{'gap s':>8s} {'periods':>8s} {'cycles':>7s} {'worst s':>8s}  ended with")
    for gap, cycles, worst, _, next_detail in sorted(gaps, reverse=True)[:20]:
        print(
            f"{gap:8.3f} {gap / RECONCILE_PERIOD_SEC:8.2f} {cycles:7d}"
            f" {worst:8.4f}  {next_detail[:52]}"
        )
    print()
    print(f"  {len(gaps)} gaps; median {statistics.median(g[0] for g in gaps):.3f} s")
    quiet = [gap for gap in gaps if gap[1] == 0]
    print(f"  {len(quiet)} of them contained no reconcile cycle at all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
