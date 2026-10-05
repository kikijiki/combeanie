#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Measure the interval between the planning-scene projector's logged status lines.

WARNING: this is not the reconcile cycle, and reading it as one has already produced two wrong
conclusions. `publish_status` logs only when the state, error code or detail text changed, and
never logs a `STATE_SYNCHRONIZING` status at all, so a projector reconciling happily with nothing
new to say is silent for as long as nothing changes. A gap here means "the status text did not
change"; a projector that stalled and one that was working perfectly look identical in it. Gaps
land on multiples of `reconcile_period_sec`, so a peak at 4.00 s is eight quiet cycles -- which
was once read as two stages each waiting out a 2.0 s `service_timeout_sec`, because 8 * 0.5 and
2 * 2.0 are the same number. Use `analyse_projector_stages.py`, which reports how many reconcile
cycles actually ran inside each gap and what the slowest stage in it cost.

What this is still good for: seeing how often the projector's reported status *changes*.
`test_planning_scene_projection_runtime` ends by waiting five wall-clock seconds for a
verification whose `scene_content_generation` is unchanged -- that is, for the scene to have
stopped changing -- and a status stream that never goes quiet is a scene that never stops being
rewritten.

Since Card 041 there is one more way to be quiet while working: a read-only request held past its
deadline republishes a DEGRADED status on every tick *without* a console line, on purpose, so a
service that is merely slow does not write one warning per reconcile cycle. A hold therefore shows
in the log as a single throttled `stage ... is held (total budget ...)` warning (plus `slow stage`
INFO lines), not as status lines, and the gap that follows it is a hold, not a stall.
"""

from __future__ import annotations

from pathlib import Path
import re
import statistics
import sys

# Matches a captured line of the shape
# `[planning_scene_projector_node-17] [INFO] [1788980633.441] [planning_scene_projector]: ...`
STATUS_LINE = re.compile(
    r"\[planning_scene_projector_node-\d+\]\s+\[INFO\]\s+\[(\d+\.\d+)\]"
    r"\s+\[planning_scene_projector\]:\s+projection status:\s+(.*)"
)

NOMINAL_RECONCILE_PERIOD_SEC = 0.5
SERVICE_TIMEOUT_SEC = 2.0
TEST_WINDOW_SEC = 5.0


def cycle_intervals(text: str) -> tuple[list[float], list[str]]:
    """Return the gaps between consecutive projector status publications, and their details."""
    stamps: list[float] = []
    details: list[str] = []
    for match in STATUS_LINE.finditer(text):
        stamps.append(float(match.group(1)))
        details.append(match.group(2).strip())
    gaps = [later - earlier for earlier, later in zip(stamps, stamps[1:], strict=False)]
    return gaps, details


def main(argv: list[str]) -> int:
    """Summarise projector cadence across every run log under the given directory."""
    directory = Path(argv[0] if argv else "/tmp/restocker-lease-cost")
    logs = sorted(directory.glob("*.log"))
    if not logs:
        print(f"no run logs under {directory}", file=sys.stderr)
        return 2

    everything: list[float] = []
    print(f"{'run':22s} {'cycles':>7s} {'median':>8s} {'p90':>8s} {'max':>8s}")
    for log in logs:
        gaps, _ = cycle_intervals(log.read_text(errors="replace"))
        if not gaps:
            continue
        everything.extend(gaps)
        ordered = sorted(gaps)
        p90 = ordered[min(len(ordered) - 1, int(0.9 * len(ordered)))]
        print(
            f"{log.stem:22s} {len(gaps):7d} {statistics.median(gaps):8.2f} "
            f"{p90:8.2f} {max(gaps):8.2f}"
        )

    if not everything:
        print("no projector status lines found", file=sys.stderr)
        return 2

    ordered = sorted(everything)
    median = statistics.median(everything)
    p90 = ordered[min(len(ordered) - 1, int(0.9 * len(ordered)))]
    worst = max(everything)
    print()
    print(f"all runs: {len(everything)} cycles")
    print(f"  nominal reconcile_period_sec  {NOMINAL_RECONCILE_PERIOD_SEC:.2f}")
    print(f"  observed median               {median:.2f}  ({median / 0.5:.1f}x nominal)")
    print(f"  observed p90                  {p90:.2f}  ({p90 / 0.5:.1f}x nominal)")
    print(f"  observed max                  {worst:.2f}  ({worst / 0.5:.1f}x nominal)")

    # The assertion needs one cycle after its baseline, and a second whenever the scene content
    # advanced in between, which is what a still-settling product does, because the test
    # re-establishes its baseline and starts waiting again.
    needed = 2 * p90
    print()
    print(
        f"  test window is {TEST_WINDOW_SEC:.1f} s, which is "
        f"{TEST_WINDOW_SEC / max(p90, 1e-9):.1f} of the p90 cycle"
    )
    print(f"  two cycles at the p90 rate is {needed:.1f} s")
    ceiling = 2 * (NOMINAL_RECONCILE_PERIOD_SEC + 2 * SERVICE_TIMEOUT_SEC)
    print(
        f"  bounded from the projector's own parameters instead of from a machine: a cycle can "
        f"cost reconcile_period_sec plus its service round trips, so two cycles is at most "
        f"2 * (0.5 + 2 * {SERVICE_TIMEOUT_SEC:.1f}) = {ceiling:.1f} s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
