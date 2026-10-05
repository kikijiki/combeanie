# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Drive the predeclared paired UR10e transfer-reliability campaign, then aggregate it."""
# One invocation is one campaign directory: the driver refuses a directory that already holds a
# record, so a fresh attempt can never overwrite or merge into an earlier population (the
# no-replacement policy lives in transfer_reliability; this is the write-side half of it).
#
# Pairing: configuration A on the first run of odd pairs, B first on even pairs, so neither
# fixture systematically inherits the other's teardown or a drifting load. Every attempt writes
# exactly one run.json — including a harness crash or an interrupt — classified at aggregation
# by transfer_reliability.classify_run rather than dropped.
#
# Not a pass/fail gate, like benchmark_run: it exits 0 when it records failures, because
# recording them is the job. `--summarise-only` re-aggregates a stored directory with no ROS
# environment beyond the import, so a campaign stays readable after its workspace is gone.

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import traceback

from ament_index_python.packages import get_package_share_directory
import rclpy

from restocker_benchmarks import (  # noqa: I100,I101 - ament and Ruff disagree
    RECORD_SCHEMA_VERSION,
    report,
    runner,
    transfer_reliability,
)
from restocker_benchmarks import scenarios as scenario_table  # noqa: I100


def _write_record(record: dict, run_directory: Path) -> None:
    """Persist one attempt's record beside its console log and spawned scenario document."""
    run_directory.mkdir(parents=True, exist_ok=True)
    (run_directory / "run.json").write_text(
        json.dumps(record, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8"
    )


def _harness_record(
    scenario,
    seed: str,
    run_directory: Path,
    *,
    detail: str,
    traceback_text: str,
) -> dict[str, object]:
    """Build the retained record for an attempt the runner itself could not complete."""
    hardware = {"load_average_at_start": list(os.getloadavg())}
    return {
        "schema_version": RECORD_SCHEMA_VERSION,
        "scenario": scenario.name,
        "scenario_description": scenario.description,
        "seed": seed,
        "run_directory": str(run_directory),
        "started_utc": dt.datetime.now(dt.UTC).isoformat(),
        "finished_utc": dt.datetime.now(dt.UTC).isoformat(),
        "outcome": "harness_exception",
        "outcome_detail": detail,
        "traceback": traceback_text,
        "provenance": {
            "arm": runner.arm_identity(),
            "hardware": hardware,
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
            "gz_partition": os.environ.get("GZ_PARTITION"),
            # Even a harness-terminated attempt names the pipeline the tree would have run
            # (Card 023): the value is a property of the launch file, not of this attempt.
            "obstacle_perception": runner.effective_obstacle_perception({}),
        },
        "goals": [],
    }


def _run_attempt(
    scenario,
    seed: str,
    run_directory: Path,
    repository: Path,
    *,
    campaign: str,
    configuration: str,
    pair_index: int,
    attempt_index: int,
) -> dict:
    """Execute one attempt, always write its record, and enrich it with campaign coordinates."""
    run_directory.mkdir(parents=True, exist_ok=True)
    try:
        record = runner.execute_run(scenario, seed, run_directory, repository)
    except KeyboardInterrupt:
        # An interrupted attempt still happened: retain it, then let the interrupt propagate
        # through main's finally so the partial campaign is aggregated on the way out.
        record = _harness_record(
            scenario,
            seed,
            run_directory,
            detail="interrupted by KeyboardInterrupt before the runner returned",
            traceback_text=traceback.format_exc(),
        )
        record.update(
            {
                "campaign": campaign,
                "configuration": configuration,
                "pair_index": pair_index,
                "attempt_index": attempt_index,
                "run_id": run_directory.name,
            }
        )
        _write_record(record, run_directory)
        raise
    except BaseException as error:  # noqa: BLE001 - any harness crash is a retained attempt
        # SystemExit included: a child that kills the driver between attempts must not leave a
        # silent hole in the population.
        record = _harness_record(
            scenario,
            seed,
            run_directory,
            detail=f"{type(error).__name__}: {error}",
            traceback_text=traceback.format_exc(),
        )
    record.update(
        {
            "campaign": campaign,
            "configuration": configuration,
            "pair_index": pair_index,
            "attempt_index": attempt_index,
            "run_id": run_directory.name,
        }
    )
    _write_record(record, run_directory)
    return record


def _declared_pairs_file(results_root: Path) -> Path:
    return results_root / "campaign.json"


def _read_declared_pairs(results_root: Path) -> int | None:
    """Return the pair count a stored campaign.json declares, or None when there is none."""
    path = _declared_pairs_file(results_root)
    if not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    pairs = document.get("declared_pairs")
    return pairs if isinstance(pairs, int) and not isinstance(pairs, bool) and pairs > 0 else None


def aggregate(
    results_root: Path,
    declared_pairs: int,
    *,
    configurations: tuple[str, ...] = transfer_reliability.CONFIGURATIONS,
    campaign: str = transfer_reliability.CAMPAIGN_NAME,
) -> dict:
    """Re-aggregate a campaign directory into summary.json, summary.txt and report.md."""
    runs = report.load_runs(results_root)
    summary = transfer_reliability.summarise(
        runs,
        declared_pairs=declared_pairs,
        configurations=configurations,
        campaign=campaign,
    )
    results_root.mkdir(parents=True, exist_ok=True)
    (results_root / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n", encoding="utf-8"
    )
    (results_root / "summary.txt").write_text(
        transfer_reliability.format_summary(summary), encoding="utf-8"
    )
    (results_root / "report.md").write_text(
        transfer_reliability.render_markdown(summary), encoding="utf-8"
    )
    print(
        f"[paired] {summary['retained']['total_records']} record(s), "
        f"population_complete={summary['population_complete']} "
        f"-> {results_root / 'report.md'}",
        flush=True,
    )
    return summary


def _refuse_existing_campaign(results_root: Path) -> None:
    """Refuse to drive into a directory that already holds campaign records."""
    existing = sorted(results_root.rglob("run.json")) if results_root.is_dir() else []
    if existing:
        raise SystemExit(
            f"{results_root} already holds {len(existing)} run record(s); a campaign is one "
            f"invocation into one fresh directory so no attempt can replace another. "
            f"Aggregate it with --summarise-only, or choose a new --results-dir."
        )


def _declaration_payload(
    *, campaign: str, declared_pairs: int, configurations: tuple[str, ...], started_utc: str
) -> dict:
    """Build the campaign.json a live invocation writes before attempt 1."""
    payload = {
        "campaign": campaign,
        "declared_pairs": declared_pairs,
        "configurations": list(configurations),
        "arm_label_required": transfer_reliability.ARM_LABEL,
        "seed_policy": {name: transfer_reliability.SEED_POLICY[name] for name in configurations},
        "started_utc": started_utc,
    }
    if campaign == transfer_reliability.ADDENDUM_CAMPAIGN_NAME:
        payload["addendum"] = transfer_reliability.ADDENDUM_ID
    return payload


def _read_declaration(results_root: Path) -> dict | None:
    """Return the stored campaign.json as a declaration dict, or None when absent/unreadable."""
    path = _declared_pairs_file(results_root)
    if not path.is_file():
        return None
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(document, dict):
        return None
    return document


def _declaration_scope(
    declaration: dict | None,
    arguments_pairs: int | None,
    *,
    default_configurations: tuple[str, ...] = transfer_reliability.CONFIGURATIONS,
    default_campaign: str = transfer_reliability.CAMPAIGN_NAME,
    default_pairs: int = transfer_reliability.PAIRS_PER_CONFIGURATION,
) -> dict:
    """
    Resolve (declared_pairs, configurations, campaign) from a stored declaration and flags.

    A stored campaign.json wins on every field — it is the frozen declaration the records were
    produced under. With no declaration, the defaults describe the invocation's population (the
    original paired campaign, or the dense addendum when ``--dense-rerun`` selected it).
    ``--pairs`` may only confirm the stored count, never rescale it.
    """
    stored_pairs = None
    configurations = default_configurations
    campaign = default_campaign
    if declaration is not None:
        pairs = declaration.get("declared_pairs")
        if isinstance(pairs, int) and not isinstance(pairs, bool) and pairs > 0:
            stored_pairs = pairs
        stored_configs = declaration.get("configurations")
        if isinstance(stored_configs, list) and stored_configs:
            configurations = tuple(stored_configs)
            try:
                transfer_reliability._check_configurations(configurations)
            except ValueError as error:
                raise SystemExit(
                    f"campaign.json declares an unusable population scope: {error}"
                ) from None
        stored_campaign = declaration.get("campaign")
        if isinstance(stored_campaign, str) and stored_campaign:
            campaign = stored_campaign
    declared = arguments_pairs or stored_pairs or default_pairs
    if (
        declaration is not None
        and arguments_pairs
        and stored_pairs
        and arguments_pairs != stored_pairs
    ):
        raise SystemExit(
            f"campaign.json declares {stored_pairs} pairs but --pairs {arguments_pairs} was "
            f"given; re-aggregate with the declared count so the population statement stays "
            f"frozen"
        )
    return {
        "declared_pairs": declared,
        "configurations": configurations,
        "campaign": campaign,
    }


def main(argv: list[str] | None = None) -> int:
    """Run the paired campaign (or just re-aggregate one) and print the summary."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--pairs",
        type=int,
        default=None,
        help=(
            f"pairs to drive (default: the predeclared "
            f"{transfer_reliability.PAIRS_PER_CONFIGURATION}, or "
            f"{transfer_reliability.ADDENDUM_ATTEMPTS} with --dense-rerun; stored campaign.json "
            f"wins on --summarise-only)"
        ),
    )
    parser.add_argument(
        "--results-dir",
        default=None,
        help=(
            "campaign directory "
            "(default: <repository root>/benchmark-results/"
            f"{transfer_reliability.CAMPAIGN_NAME}, or "
            f"{transfer_reliability.ADDENDUM_CAMPAIGN_NAME} with --dense-rerun)"
        ),
    )
    parser.add_argument(
        "--summarise-only",
        action="store_true",
        help="aggregate an existing campaign directory without launching anything",
    )
    parser.add_argument(
        "--dense-rerun",
        action="store_true",
        help=(
            "drive the frozen dense-only addendum population "
            f"({transfer_reliability.ADDENDUM_ID}): "
            f"{list(transfer_reliability.ADDENDUM_CONFIGURATIONS)} at "
            f"{transfer_reliability.ADDENDUM_ATTEMPTS} attempts into its own campaign directory; "
            "the original 2026-09-23 population is never re-run or replaced"
        ),
    )
    arguments = parser.parse_args(argv)

    repository = runner.repository_root()
    default_campaign = (
        transfer_reliability.ADDENDUM_CAMPAIGN_NAME
        if arguments.dense_rerun
        else transfer_reliability.CAMPAIGN_NAME
    )
    default_configurations = (
        transfer_reliability.ADDENDUM_CONFIGURATIONS
        if arguments.dense_rerun
        else transfer_reliability.CONFIGURATIONS
    )
    default_pairs = (
        transfer_reliability.ADDENDUM_ATTEMPTS
        if arguments.dense_rerun
        else transfer_reliability.PAIRS_PER_CONFIGURATION
    )
    scope_defaults = {
        "default_configurations": default_configurations,
        "default_campaign": default_campaign,
        "default_pairs": default_pairs,
    }
    results_root = (
        Path(arguments.results_dir)
        if arguments.results_dir
        else repository / "benchmark-results" / default_campaign
    )

    if arguments.summarise_only:
        scope = _declaration_scope(
            _read_declaration(results_root), arguments.pairs, **scope_defaults
        )
        if not results_root.is_dir():
            raise SystemExit(f"no such campaign directory: {results_root}")
        aggregate(
            results_root,
            scope["declared_pairs"],
            configurations=scope["configurations"],
            campaign=scope["campaign"],
        )
        return 0

    declared = arguments.pairs or default_pairs
    if declared < 1:
        raise SystemExit("--pairs must be at least 1")
    _refuse_existing_campaign(results_root)

    # Fail fast: without the planning profile the coordinator never becomes ready.
    try:
        get_package_share_directory("moveit_ros_move_group")
    except Exception:  # noqa: BLE001 - ament raises a package-specific error type
        raise SystemExit(
            "move_group is not on the search path; run the campaign through 'just "
            "benchmark-paired', which enters the baseline profile"
        ) from None

    resolved = {name: scenario_table.scenario(name) for name in default_configurations}
    results_root.mkdir(parents=True, exist_ok=True)
    _declared_pairs_file(results_root).write_text(
        json.dumps(
            _declaration_payload(
                campaign=default_campaign,
                declared_pairs=declared,
                configurations=default_configurations,
                started_utc=dt.datetime.now(dt.UTC).isoformat(),
            ),
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    attempts = {name: 0 for name in default_configurations}
    rclpy.init()
    try:
        for pair_index in range(declared):
            order = (
                default_configurations
                if pair_index % 2 == 0
                else tuple(reversed(default_configurations))
            )
            for configuration in order:
                attempts[configuration] += 1
                attempt_index = attempts[configuration]
                stamp = dt.datetime.now(dt.UTC).strftime("%Y%m%dT%H%M%SZ")
                run_id = (
                    f"{configuration}-pair-{pair_index + 1:02d}"
                    f"-attempt-{attempt_index:02d}-{stamp}"
                )
                run_directory = results_root / configuration / run_id
                seed = transfer_reliability.SEED_POLICY[configuration]
                print(
                    f"[paired] pair {pair_index + 1}/{declared} "
                    f"{configuration} attempt {attempt_index} -> {run_directory}",
                    flush=True,
                )
                record = _run_attempt(
                    resolved[configuration],
                    seed,
                    run_directory,
                    repository,
                    campaign=default_campaign,
                    configuration=configuration,
                    pair_index=pair_index,
                    attempt_index=attempt_index,
                )
                print(
                    f"[paired] {run_id}: {record['outcome']} "
                    f"class={transfer_reliability.classify_run(record)}",
                    flush=True,
                )
    finally:
        rclpy.shutdown()
        aggregate(
            results_root,
            declared,
            configurations=default_configurations,
            campaign=default_campaign,
        )

    # Measurement tool: a recorded failure is a successful campaign, same as benchmark_run.
    print(
        f"[paired] {declared} pair(s) driven into {results_root} "
        f"(exit 0: recording failures is the job)",
        flush=True,
    )
    return 0
