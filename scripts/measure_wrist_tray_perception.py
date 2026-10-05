#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Predeclared wrist tray perception: print the matrix, build scenarios, run live, summarise."""
# Offline paths (matrix, scenarios, summarise) never start Gazebo. --live drives
# test_tray_survey_runtime through the same env hooks as the lane-depth campaign: one
# simulator slot at a time, one arrangement per launch, predeclared cell metadata on each
# row. The machine-wide slot must be free before --live is started.
#
# The matrix itself is fixed in restocker_benchmarks.tray_perception_campaign so the summary
# checks coverage against cells declared before any sample existed.

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess  # noqa: S404 - fixed argv, no shell
import sys
import tempfile
import time

import yaml

REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_benchmarks"))
sys.path.insert(0, str(REPOSITORY_ROOT / "ros_ws" / "src" / "restocker_gazebo"))

from restocker_benchmarks import tray_perception_campaign as campaign  # noqa: E402
from restocker_gazebo import tray_arrangements as tray  # noqa: E402

# isort wants the path-injected imports above; ament_flake8 wants yaml first among third-party.
# Keep yaml with stdlib imports above the sys.path block — both linters accept this order.

TEST_FILE = (
    REPOSITORY_ROOT
    / "ros_ws"
    / "src"
    / "restocker_bringup"
    / "test"
    / "test_tray_survey_runtime.py"
)
# Extrinsics cell: sequential confirm passes inside one launch at one optical pose.
EXTRINSICS_PASSES = 6


def _scenario_for_cell(cell: campaign.MatrixCell, seed: int) -> dict:
    """Return the scenario document a predeclared cell runs against."""
    if cell.cell_id == "A-isolated":
        return tray.isolated_catalogue_scenario(seed)
    if cell.cell_id.startswith("B-touching-"):
        keys = cell.geometry_keys
        if len(keys) == 1:
            return tray.touching_pair_scenario(keys[0], keys[0])
        return tray.touching_pair_scenario(keys[0], keys[1])
    if cell.cell_id.startswith("C-neighbour-"):
        return tray.close_neighbour_scenario(cell.geometry_keys[0])
    if cell.cell_id == "D-non-upright":
        return tray.non_upright_scenario(cell.geometry_keys[0])
    if cell.cell_id == "E-extrinsics":
        return tray.extrinsics_scenario(cell.geometry_keys[0])
    raise ValueError(f"no scenario builder for cell {cell.cell_id}")


def generate_scenarios(directory: Path) -> list[Path]:
    """Write one scenario YAML per predeclared (cell, seed) and return the paths."""
    directory.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    for cell in campaign.predeclared_matrix():
        for seed in cell.seeds:
            label = f"{cell.cell_id}-seed{seed}" if not cell.fixed_construction else cell.cell_id
            path = directory / f"{label}.yaml"
            path.write_text(yaml.safe_dump(_scenario_for_cell(cell, seed), sort_keys=False))
            written.append(path)
    return written


def summarise(log: Path) -> int:
    """Print the predeclared metric report for an existing row log."""
    rows = campaign.read_rows(log)
    summary = campaign.summarise(rows)
    text = campaign.format_summary(summary)
    print(text, end="")
    # Coverage is part of SC-001's aggregation: refuse success when predeclared cells are absent.
    return 1 if summary.get("missing_cells") else 0


def _cell_launches() -> list[tuple[campaign.MatrixCell, int, str]]:
    """Return (cell, seed, label) for each predeclared launch, in matrix order."""
    launches: list[tuple[campaign.MatrixCell, int, str]] = []
    for cell in campaign.predeclared_matrix():
        for seed in cell.seeds:
            label = cell.cell_id if cell.fixed_construction else f"{cell.cell_id}-seed{seed}"
            launches.append((cell, seed, label))
    return launches


def _run_launch(
    cell: campaign.MatrixCell,
    seed: int,
    label: str,
    scenario: Path,
    samples_log: Path,
    commit: str,
    timeout_s: float,
    log_dir: Path,
) -> int:
    """One arrangement through the tray survey acceptance test in campaign mode."""
    environment = dict(os.environ)
    environment["RESTOCKER_TRAY_SURVEY_SCENARIO"] = str(scenario)
    environment["RESTOCKER_TRAY_SURVEY_SAMPLES"] = str(samples_log)
    environment["RESTOCKER_TRAY_SURVEY_LABEL"] = label
    environment["RESTOCKER_TRAY_CELL_ID"] = cell.cell_id
    environment["RESTOCKER_TRAY_FEATURE"] = cell.feature
    environment["RESTOCKER_TRAY_SEED"] = str(seed)
    environment["RESTOCKER_TRAY_COMMIT"] = commit
    environment["RESTOCKER_TRAY_EXTRINSICS_PASSES"] = (
        str(EXTRINSICS_PASSES) if cell.cell_id == "E-extrinsics" else "1"
    )
    command = [
        str(REPOSITORY_ROOT / "scripts" / "with_simulator_slot.bash"),
        "python3",
        str(REPOSITORY_ROOT / "scripts" / "domain_isolation.py"),
        "--seed",
        f"wrist-tray-{label}",
        "--",
        str(REPOSITORY_ROOT / "scripts" / "with_workspace.bash"),
        "launch_test",
        str(TEST_FILE),
    ]
    print(f"--- {label}: {scenario}", flush=True)
    started = time.monotonic()
    log_path = log_dir / f"{label}.log"
    with log_path.open("w", encoding="utf-8") as log_handle:
        completed = subprocess.run(  # noqa: S603 - fixed argv, no shell
            command,
            env=environment,
            check=False,
            timeout=timeout_s,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )
    elapsed = time.monotonic() - started
    print(
        f"--- {label}: exit={completed.returncode} in {elapsed:.0f}s (log {log_path})",
        flush=True,
    )
    return completed.returncode


def _looks_like_infrastructure_failure(log_path: Path) -> bool:
    """Report whether a non-zero exit is a startup/lease fault, not a measurement outcome."""
    if not log_path.is_file():
        return True
    text = log_path.read_text(encoding="utf-8", errors="replace")
    markers = (
        "no simulator slot within",
        "the tray survey server, ground-truth products, or a certified planning scene never "
        "appeared",
        "ImportError",
        "ModuleNotFoundError",
        "Resource temporarily unavailable",
    )
    return any(marker in text for marker in markers)


def run_live(
    samples_log: Path,
    scenario_dir: Path | None,
    log_dir: Path,
    timeout_s: float,
    limit: int,
    retries: int,
    only: str = "",
) -> int:
    """Run every predeclared launch once (bounded retries on infrastructure faults only)."""
    commit = os.environ.get(
        "RESTOCKER_TRAY_COMMIT",
        subprocess.run(  # noqa: S603 - fixed argv
            ["git", "rev-parse", "HEAD"],
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
        ).stdout.strip(),
    )
    directory = scenario_dir or Path(tempfile.mkdtemp(prefix="wrist-tray-matrix-"))
    directory.mkdir(parents=True, exist_ok=True)
    log_dir.mkdir(parents=True, exist_ok=True)
    samples_log.parent.mkdir(parents=True, exist_ok=True)

    launches = _cell_launches()
    if only:
        launches = [item for item in launches if only in item[2]]
        if not launches:
            print(f"--only {only!r} matched no launches", file=sys.stderr)
            return 2
    if limit > 0:
        launches = launches[:limit]

    print(
        f"live campaign: {len(launches)} launches, commit={commit}, "
        f"samples={samples_log}, scenarios={directory}",
        flush=True,
    )
    failures = 0
    for cell, seed, label in launches:
        scenario = directory / f"{label}.yaml"
        scenario.write_text(yaml.safe_dump(_scenario_for_cell(cell, seed), sort_keys=False))
        status = _run_launch(cell, seed, label, scenario, samples_log, commit, timeout_s, log_dir)
        attempt = 0
        while status != 0 and attempt < retries:
            attempt += 1
            log_path = log_dir / f"{label}.log"
            if not _looks_like_infrastructure_failure(log_path):
                print(
                    f"--- {label}: non-zero exit is not an infrastructure fault; not retrying",
                    file=sys.stderr,
                    flush=True,
                )
                break
            print(
                f"--- {label}: infrastructure fault, retry {attempt}/{retries}",
                flush=True,
            )
            status = _run_launch(
                cell, seed, label, scenario, samples_log, commit, timeout_s, log_dir
            )
        if status != 0:
            failures += 1
            print(f"launch {label} exited {status}", file=sys.stderr, flush=True)

    summarise(samples_log)
    if failures:
        print(f"\n{failures} of {len(launches)} launches failed", file=sys.stderr, flush=True)
        return 1
    return 0


def main() -> int:
    """Dispatch to --print-matrix, --generate-scenarios, --live, or --summarise-only."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out", type=Path, default=None, help="JSONL row log for --summarise-only / --live"
    )
    parser.add_argument(
        "--summarise-only", action="store_true", help="re-read an existing row log"
    )
    parser.add_argument("--print-matrix", action="store_true", help="print the predeclared matrix")
    parser.add_argument(
        "--generate-scenarios",
        action="store_true",
        help="write one scenario YAML per predeclared cell/seed",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="run predeclared launches via the tray survey test (needs a free simulator slot)",
    )
    parser.add_argument(
        "--dir",
        type=Path,
        default=None,
        help="output directory for --generate-scenarios / scenario store for --live",
    )
    parser.add_argument(
        "--log-dir",
        type=Path,
        default=None,
        help="per-launch log directory for --live (default: alongside --out)",
    )
    parser.add_argument("--timeout-s", type=float, default=2400.0, help="per-launch timeout")
    parser.add_argument("--limit", type=int, default=0, help="run only the first N launches")
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        help="bounded retries per launch on infrastructure faults only (default 1)",
    )
    parser.add_argument(
        "--only",
        type=str,
        default="",
        help="run only launches whose label contains this substring (proportionate re-runs)",
    )
    arguments = parser.parse_args()

    if arguments.print_matrix:
        print(campaign.format_matrix(), end="")
        return 0

    if arguments.generate_scenarios:
        directory = arguments.dir or Path(tempfile.mkdtemp(prefix="wrist-tray-matrix-"))
        paths = generate_scenarios(directory)
        print(f"wrote {len(paths)} scenario documents under {directory}")
        return 0

    if arguments.live:
        if arguments.out is None:
            print("usage: --live requires --out for the JSONL row log", file=sys.stderr)
            return 2
        log_dir = arguments.log_dir or (arguments.out.parent / "wrist-tray-launch-logs")
        return run_live(
            arguments.out,
            arguments.dir,
            log_dir,
            arguments.timeout_s,
            arguments.limit,
            arguments.retries,
            only=arguments.only,
        )

    if arguments.summarise_only:
        if arguments.out is None or not arguments.out.is_file():
            print(
                "usage: --summarise-only requires --out pointing at an existing JSONL log",
                file=sys.stderr,
            )
            return 2
        return summarise(arguments.out)

    parser.error("choose one of --print-matrix, --generate-scenarios, --live, or --summarise-only")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
