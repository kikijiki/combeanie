#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Measure how far the wrist camera's lane depletion reading is from the simulator's own."""
# One launch per arrangement, six lanes surveyed in each, one row per lane. The arrangements cover
# every catalogued product at every column length from empty to lane capacity, and every lane at
# least once per product.
#
# The run drives the acceptance test itself through two environment variables it honours:
# RESTOCKER_LANE_SURVEY_SCENARIO names the arrangement and RESTOCKER_LANE_SURVEY_SAMPLES names the
# row log. This keeps the campaign on the same code path as the test.
#
# One simulator, one slot, one launch at a time (see scripts/with_simulator_slot.bash).

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess  # noqa: S404 - fixed argv, no shell
import sys
import tempfile

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_gazebo"))

from restocker_gazebo.lane_columns import CATALOGUE, LaneColumn, lane_column_scenario  # noqa: E402

LANES = ("lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06")
TEST_FILE = (
    REPOSITORY_ROOT
    / "ros_ws"
    / "src"
    / "restocker_bringup"
    / "test"
    / "test_lane_survey_runtime.py"
)


def _arrangements() -> list[list[LaneColumn]]:
    """Return the scenarios to run, six lanes at a time."""
    # Every (product, count) pair the catalogue admits, dealt round-robin across the six lanes so
    # per-lane and per-fill biases are not confounded.
    cases: list[tuple[str, int]] = []
    for geometry_key, entry in CATALOGUE.items():
        for count in range(entry["lane_capacity"] + 1):
            cases.append((geometry_key, count))

    arrangements: list[list[LaneColumn]] = []
    for index, start in enumerate(range(0, len(cases), len(LANES))):
        window = cases[start : start + len(LANES)]
        arrangements.append(
            [
                LaneColumn(lane_id=LANES[(index + offset) % len(LANES)], geometry_key=key, count=n)
                for offset, (key, n) in enumerate(window)
            ]
        )
    return arrangements


def _run(scenario: Path, label: str, log: Path, timeout_s: float) -> int:
    """Run the acceptance test once against one arrangement."""
    environment = dict(os.environ)
    environment["RESTOCKER_LANE_SURVEY_SCENARIO"] = str(scenario)
    environment["RESTOCKER_LANE_SURVEY_SAMPLES"] = str(log)
    environment["RESTOCKER_LANE_SURVEY_LABEL"] = label
    command = [
        str(REPOSITORY_ROOT / "scripts" / "with_simulator_slot.bash"),
        "python3",
        str(REPOSITORY_ROOT / "scripts" / "domain_isolation.py"),
        "--seed",
        f"lane-depth-{label}",
        "--",
        # The workspace must be sourced so the launch description resolves its packages and the
        # test can import restocker_gazebo.
        str(REPOSITORY_ROOT / "scripts" / "with_workspace.bash"),
        "launch_test",
        str(TEST_FILE),
    ]
    print(f"--- {label}: {scenario}", flush=True)
    completed = subprocess.run(  # noqa: S603 - fixed argv, no shell
        command, env=environment, check=False, timeout=timeout_s
    )
    return completed.returncode


def _describe(values: list[float]) -> str:
    """Return the shape of a signed error population, in millimetres."""
    if not values:
        return "no samples"
    ordered = sorted(values)

    def quantile(fraction: float) -> float:
        index = min(len(ordered) - 1, round(fraction * (len(ordered) - 1)))
        return ordered[index] * 1000.0

    magnitudes = sorted(abs(value) for value in values)
    p95_magnitude = magnitudes[min(len(magnitudes) - 1, round(0.95 * (len(magnitudes) - 1)))]
    return (
        f"n={len(ordered)} signed min {quantile(0.0):+.3f} p50 {quantile(0.50):+.3f} "
        f"p95 {quantile(0.95):+.3f} max {quantile(1.0):+.3f} mm, "
        f"|error| p95 {p95_magnitude * 1000.0:.3f} max {magnitudes[-1] * 1000.0:.3f} mm"
    )


def summarise(log: Path) -> None:
    """Print the error distribution conditioned by product, lane, fill and column length."""
    rows = [json.loads(line) for line in log.read_text().splitlines() if line.strip()]
    if not rows:
        print("no rows recorded")
        return
    errors = [row["available_depth_error_m"] for row in rows]
    print()
    print(f"lane depth agreement, whole population: {_describe(errors)}")
    print(
        f"  coverage: min {min(row['coverage'] for row in rows):.4f}, "
        f"mean {statistics.fmean(row['coverage'] for row in rows):.4f}"
    )
    print(
        f"  refused observations: "
        f"{sum(1 for row in rows if row['observation_status'] != 0)} of {len(rows)}"
    )

    print("\nby product:")
    for geometry_key in sorted({row["geometry_key"] for row in rows}):
        subset = [
            row["available_depth_error_m"] for row in rows if row["geometry_key"] == geometry_key
        ]
        print(f"  {geometry_key or '(empty lane)':<24} {_describe(subset)}")

    print("\nby lane:")
    for lane in sorted({row["lane_id"] for row in rows}):
        subset = [row["available_depth_error_m"] for row in rows if row["lane_id"] == lane]
        print(f"  {lane:<24} {_describe(subset)}")

    print("\nby fill, as a fraction of the lane's 0.85 m usable depth:")
    for bucket in range(10):
        low = bucket / 10.0
        high = (bucket + 1) / 10.0
        subset = [
            row["available_depth_error_m"]
            for row in rows
            if low <= min(0.999, 1.0 - (row["ground_truth_available_depth_m"] / 0.85)) < high
        ]
        if subset:
            print(f"  {low:>4.0%}-{high:<4.0%}            {_describe(subset)}")

    print("\nby column length:")
    for count in sorted({row["count"] for row in rows}):
        subset = [row["available_depth_error_m"] for row in rows if row["count"] == count]
        print(f"  {count:>2d} products            {_describe(subset)}")


def main() -> int:
    """Generate the arrangements, run them one at a time, and summarise the rows."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True, help="JSONL row log to append to")
    parser.add_argument("--scenario-dir", type=Path, default=None)
    parser.add_argument("--timeout-s", type=float, default=2400.0)
    parser.add_argument("--limit", type=int, default=0, help="run only the first N arrangements")
    parser.add_argument(
        "--summarise-only", action="store_true", help="re-read an existing row log and report"
    )
    arguments = parser.parse_args()

    if arguments.summarise_only:
        summarise(arguments.out)
        return 0

    scenario_dir = arguments.scenario_dir or Path(tempfile.mkdtemp(prefix="restocker-lane-depth-"))
    scenario_dir.mkdir(parents=True, exist_ok=True)
    arrangements = _arrangements()
    if arguments.limit > 0:
        arrangements = arrangements[: arguments.limit]

    failures = 0
    for index, columns in enumerate(arrangements):
        label = f"{index:02d}"
        scenario = scenario_dir / f"lane_columns_{label}.yaml"
        scenario.write_text(yaml.safe_dump(lane_column_scenario(columns), sort_keys=False))
        status = _run(scenario, label, arguments.out, arguments.timeout_s)
        if status != 0:
            failures += 1
            print(f"arrangement {label} exited {status}", file=sys.stderr)

    summarise(arguments.out)
    if failures:
        print(f"\n{failures} of {len(arrangements)} arrangements failed", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
