# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Predeclared paired UR10e transfer-reliability campaign and its offline aggregation."""
# The design below is frozen before any campaign sample exists, so the population, the
# denominators and the inference cannot be refit to results. The module is pure: no ROS imports,
# so a stored campaign re-aggregates after the workspace that produced it is gone. The frozen
# human-readable statement is Backstage's
# benchmark-campaigns/predeclared-2026-09-23-ur10e-transfer-reliability/README.md.
#
# ## Configurations and pairing (frozen)
#
# Two fixed-pose scenarios, the two the public claims already rest on one run each:
# ``baseline_transfers`` (the baseline fixture) and ``dense_restock_transfers`` (the same three
# addressed goals against the dense fixture). Both are non-seedable: the stock poses are pinned by
# the YAML and the runner records the stock file digest per run. A numeric seed on either
# configuration is a predeclaration violation.
#
# Runs interleave one pair at a time — A then B on odd pairs, B then A on even pairs — so neither
# configuration systematically inherits the other's teardown or a drifting machine load. Each run
# leases its ROS domain and Gazebo partition exactly as ``just benchmark`` does. Records carry
# ``configuration``, 0-based ``pair_index`` and 1-based ``attempt_index``.
#
# ## Population (frozen)
#
# ``PAIRS_PER_CONFIGURATION`` pairs: that many attempts per configuration. An attempt is one
# launched run that produced a record, including one that failed at startup. The population is
# attempts 1..N per configuration in launch order. A record with a larger ``attempt_index`` (an
# operator re-run) is retained, reported as ``beyond_population`` and never enters the rates.
#
# ## Denominators (frozen)
#
# - **Transfer (primary):** every goal record inside population runs; success is terminal status
#   ``SUCCEEDED``. Goals never dispatched because the run died at startup do not exist as
#   attempts — a startup failure is charged to the run denominator, not to three transfers.
# - **Run (secondary):** every population run; *clean* is completed with every recorded goal
#   succeeded. Every other run class is a failed run.
# - A claim requires a complete population. Coverage, missing slots, ineligible records and
#   ``beyond_population`` rows are reported explicitly; an incomplete population is never
#   silently narrowed.
#
# ## Terminal taxonomy (frozen)
#
# Goal level: the existing benchmark taxonomy (family and masked signature), recomputed from each
# record at aggregation time by ``report.classify_goal``. Run level: exactly one of ``clean``,
# ``completed_with_failure``, ``startup_failed``, ``launch_exited``, ``harness_exception``,
# ``unclassified`` — an outcome outside the declared set is ``unclassified`` and fails SC-002
# rather than being bucketed into a neighbour.
#
# ## Machine load (frozen)
#
# The runner records the 1-minute load average at start and at finish with the CPU count. At
# aggregation a run is *quiet* iff ``load1_start < 0.5 * cpu_count``, else *contended*
# (``unknown`` when either value is missing). Load annotates every rate and never excludes a
# run: the evidence audit's larger sample shows load moves the failure distribution, not the pass
# rate, so a loaded run is not discarded and a red run is not closed on load alone.
#
# ## Sample size and inference boundary (frozen)
#
# 72 planned transfers and 24 runs per configuration; 144 and 48 pooled. At the historical
# mid-range pass rate of 0.65 the design half-widths are ±11.0 pp (transfers, per configuration),
# ±7.8 pp (transfers, pooled) and ±19.1 pp (runs); zero run failures in 24 bounds the run-failure
# rate at 1 - 0.05^(1/24) = 11.7% one-sided 95%, and if the true run-failure rate were the
# historical 0.35 the chance of seeing zero failures in 24 runs is 0.65^24 ≈ 3.2e-5. That rejects
# the historical rate on a zero-failure population; it does not support sub-10 pp discrimination
# between the two configurations, any claim about the retired custom arm, or the autonomous
# dense-demo gate's population (Card 020's instrument).
#
# ## Arm gate (frozen, SC-003)
#
# Population eligibility requires ``provenance.arm.label == ARM_LABEL`` (``ur10e``, parsed by the
# runner from the description's arm.xacro). A record with a missing or different label — what any
# record produced before the UR10e swap looks like — is retained, reported under ``excluded_arm``
# and can never enter the UR10e rates.
#
# ## No-replacement policy (frozen, SC-002)
#
# No record is deleted, rewritten or superseded: a failed attempt stays in its denominator row.
# A re-run is a new record with the next ``attempt_index``; beyond the declared population it is
# retained as ``beyond_population`` and excluded from rates. It never replaces the failed row.
#
# ## Addendum (frozen 2026-09-24 before any re-run sample)
#
# The 2026-09-23 population above stands exactly as run: its dense half measured no transfer rate
# (every goal a ``product_unobserved`` harness terminal — the readiness gate dispatched while the
# 62-product fixture was still spawning) and those 24 rows are never replaced, patched or
# reinterpreted. The addendum below declares a *separate*, dense-only re-run population that
# closes the same SC-001 question after the readiness fix (dispatch only when observed ==
# stocked, bounded timeout classified as ``startup_failed``): same configuration
# (``dense_restock_transfers``), same fixed seed policy, same arm gate, same quiet load-band
# annotation (``load1 < 0.5 * cpu_count`` at start), same denominators, taxonomy and
# no-replacement policy, **24 attempts**, in its own campaign directory
# (``ADDENDUM_CAMPAIGN_NAME``). Nothing in the original declarations changes: the original
# population, its summary and its no-rate reading stay intact and unreplaced.

from __future__ import annotations

from collections import Counter
import math
import statistics
from typing import Any

from restocker_benchmarks import recovery, report, scenarios, taxonomy

#: Campaign directory name under ``benchmark-results/``.
CAMPAIGN_NAME = "ur10e-transfer-reliability"

#: The two paired configurations, in pair order (A, B).
CONFIGURATIONS: tuple[str, ...] = ("baseline_transfers", "dense_restock_transfers")

#: Frozen pair count: this many attempts per configuration enter the population.
PAIRS_PER_CONFIGURATION = 24

#: Addendum id: the dense-only re-run after the readiness fix, frozen before any sample.
ADDENDUM_ID = "dense-rerun-2026-09-24"

#: Campaign directory / summary name for the addendum population (its own directory, so the
#: original campaign's records are never mixed into or overwritten by the re-run).
ADDENDUM_CAMPAIGN_NAME = "ur10e-transfer-reliability-dense-rerun"

#: The addendum re-measures only the dense configuration; the baseline half is not re-run.
ADDENDUM_CONFIGURATIONS: tuple[str, ...] = ("dense_restock_transfers",)

#: Attempts in the addendum population — the same size the original reserved for this
#: configuration, so the re-run supports the same ±11.0 pp / ±19.1 pp / 11.7% statements.
ADDENDUM_ATTEMPTS = PAIRS_PER_CONFIGURATION

#: Only records on this arm are population-eligible (SC-003).
ARM_LABEL = "ur10e"

#: Seed policy per configuration; both stock fixtures are pinned, not seeded.
SEED_POLICY: dict[str, str] = {name: "fixed" for name in CONFIGURATIONS}

#: A run is *quiet* iff its start-of-run 1-minute load is below this fraction of the CPU count.
LOAD_QUIET_FRACTION = 0.5

#: Reference rate the design half-widths are quoted at (the historical mid-range pass rate).
DESIGN_REFERENCE_RATE = 0.65

#: Historical run-*failure* rate the zero-failure bound is meant to reject (~15/23 runs).
HISTORICAL_RUN_FAILURE_RATE = 0.35

#: Run-level terminal classes; anything else classifies as ``unclassified`` and fails SC-002.
RUN_CLASSES: tuple[str, ...] = (
    "clean",
    "completed_with_failure",
    "startup_failed",
    "launch_exited",
    "harness_exception",
    "unclassified",
)

# Goals each configuration's scenario drives, read from the scenario table so the planned
# transfer count cannot drift away from the scenarios the runner actually executes.
GOALS_PER_RUN: dict[str, int] = {
    name: len(scenarios.SCENARIOS_BY_NAME[name].goals or ()) for name in CONFIGURATIONS
}


def planned_transfers_per_configuration(declared_pairs: int = PAIRS_PER_CONFIGURATION) -> int:
    """Return the predeclared transfer count for one configuration at this pair count."""
    counts = {GOALS_PER_RUN[name] for name in CONFIGURATIONS}
    if len(counts) != 1:
        # The boundary statement quotes one figure for both configurations; a divergence means
        # the frozen statement has to be revisited rather than silently taking one side.
        raise ValueError(f"configurations disagree on goals per run: {GOALS_PER_RUN}")
    return declared_pairs * next(iter(counts))


def load_band(load1: Any, cpu_count: Any) -> str:
    """Classify a run's start-of-run load into ``quiet``, ``contended`` or ``unknown``."""
    try:
        load = float(load1)
        cpus = float(cpu_count)
    except (TypeError, ValueError):
        return "unknown"
    if not math.isfinite(load) or not math.isfinite(cpus) or cpus <= 0.0 or load < 0.0:
        return "unknown"
    return "quiet" if load < LOAD_QUIET_FRACTION * cpus else "contended"


def classify_run(record: dict[str, Any]) -> str:
    """Return the run-level terminal class for one campaign record."""
    outcome = record.get("outcome")
    goals = record.get("goals") or []
    if outcome == "startup_failed":
        return "startup_failed"
    if outcome == "launch_exited":
        return "launch_exited"
    if outcome == "harness_exception":
        return "harness_exception"
    if outcome == "completed":
        # A completed run that recorded no goal proved no transfer and is not clean.
        if goals and all(
            goal.get("outcome", {}).get("status") == taxonomy.STATUS_SUCCEEDED for goal in goals
        ):
            return "clean"
        return "completed_with_failure"
    return "unclassified"


def _normal_half_width(rate: float, total: int) -> float:
    """Return the normal-approximation 95% half-width used for the frozen design statement."""
    if total <= 0:
        return float("nan")
    return 1.96 * math.sqrt(rate * (1.0 - rate) / total)


def inference_boundary(declared_pairs: int = PAIRS_PER_CONFIGURATION) -> dict[str, Any]:
    """Return the frozen sample-size statement, computed from the declared population."""
    transfers = planned_transfers_per_configuration(declared_pairs)
    pooled_transfers = transfers * len(CONFIGURATIONS)
    zero_failure_upper = 1.0 - 0.05 ** (1.0 / declared_pairs) if declared_pairs > 0 else None
    return {
        "pairs": declared_pairs,
        "runs_per_configuration": declared_pairs,
        "runs_pooled": declared_pairs * len(CONFIGURATIONS),
        "transfers_per_configuration_planned": transfers,
        "transfers_pooled_planned": pooled_transfers,
        "design_reference_rate": DESIGN_REFERENCE_RATE,
        "historical_run_failure_rate": HISTORICAL_RUN_FAILURE_RATE,
        "half_width_transfers_per_configuration": _normal_half_width(
            DESIGN_REFERENCE_RATE, transfers
        ),
        "half_width_transfers_pooled": _normal_half_width(DESIGN_REFERENCE_RATE, pooled_transfers),
        "half_width_runs_per_configuration": _normal_half_width(
            DESIGN_REFERENCE_RATE, declared_pairs
        ),
        "zero_failure_run_upper_bound": zero_failure_upper,
        "probability_zero_failures_if_historical": (
            (1.0 - HISTORICAL_RUN_FAILURE_RATE) ** declared_pairs if declared_pairs > 0 else None
        ),
        "interval_reported": "Wilson 95% (recovery.wilson_interval); half-widths above are the "
        "frozen normal-approximation design statement",
        "supports": (
            "per-configuration transfer rate at the stated half-width",
            "per-configuration run rate at the stated half-width",
            "zero-failure upper bound on the run-failure rate",
            f"rejection of the historical {HISTORICAL_RUN_FAILURE_RATE:.0%} run-failure rate "
            "when zero or few failures recur",
        ),
        "excludes": (
            "sub-10 pp discrimination between the two configurations",
            "any claim about the retired custom arm",
            "the autonomous dense-demo gate population (a different instrument)",
            "certainty beyond the zero-failure upper bound",
        ),
    }


def _arm_label(record: dict[str, Any]) -> str | None:
    """Return ``provenance.arm.label`` or None when the record carries no usable arm identity."""
    provenance = record.get("provenance")
    if not isinstance(provenance, dict):
        return None
    arm = provenance.get("arm")
    if not isinstance(arm, dict):
        return None
    label = arm.get("label")
    return label if isinstance(label, str) and label else None


def _load1_start(record: dict[str, Any]) -> Any:
    """Return the start-of-run 1-minute load average from the hardware profile, or None."""
    hardware = (record.get("provenance") or {}).get("hardware")
    if not isinstance(hardware, dict):
        return None
    load = hardware.get("load_average_at_start")
    if isinstance(load, (list, tuple)) and load:
        return load[0]
    return None


def _cpu_count(record: dict[str, Any]) -> Any:
    hardware = (record.get("provenance") or {}).get("hardware")
    if not isinstance(hardware, dict):
        return None
    return hardware.get("cpu_count")


def _run_sort_key(record: dict[str, Any]) -> tuple[str, str]:
    return (str(record.get("started_utc", "")), str(record.get("run_id", "")))


def partition(
    runs: list[dict[str, Any]],
    declared_pairs: int = PAIRS_PER_CONFIGURATION,
    *,
    configurations: tuple[str, ...] = CONFIGURATIONS,
) -> dict[str, Any]:
    """
    Sort campaign records into the population and every retained out-of-population bucket.

    Precedence per record: unattributed (no usable configuration/attempt, or a configuration that
    does not match its scenario) beats everything; records past the declared attempt count are
    ``beyond_population``; arm and seed gates then hold a slot's record out without filling it; a
    second record on an already-claimed slot is a duplicate and fails completeness.

    ``configurations`` scopes which configurations have slots here: the original paired campaign
    uses both, the dense addendum only the dense one. A record whose configuration is outside the
    scope is unattributed for this population — it belongs to a different declaration, and mixing
    it in would silently widen the one being summarised.
    """
    _check_configurations(configurations)
    population: dict[str, dict[int, dict[str, Any]]] = {name: {} for name in configurations}
    beyond_population: list[dict[str, Any]] = []
    excluded_arm: list[tuple[dict[str, Any], str]] = []
    excluded_seed: list[tuple[dict[str, Any], str]] = []
    unattributed: list[tuple[dict[str, Any], str]] = []
    duplicates: list[tuple[str, int, str, str]] = []

    for record in sorted(runs, key=_run_sort_key):
        configuration = record.get("configuration")
        if configuration not in CONFIGURATIONS:
            unattributed.append((record, "missing or unknown configuration"))
            continue
        if configuration not in configurations:
            # A declared configuration from the *other* population (a baseline record inside a
            # dense-only addendum summarise): retained here, never counted in these slots.
            unattributed.append(
                (
                    record,
                    f"configuration {configuration!r} is outside this population's scope "
                    f"{list(configurations)}",
                )
            )
            continue
        attempt_raw = record.get("attempt_index")
        scenario = record.get("scenario")
        if scenario != configuration:
            unattributed.append(
                (record, f"scenario {scenario!r} does not match configuration {configuration!r}")
            )
            continue
        if not isinstance(attempt_raw, int) or isinstance(attempt_raw, bool):
            unattributed.append((record, "missing or non-integer attempt_index"))
            continue
        if attempt_raw > declared_pairs:
            beyond_population.append(record)
            continue
        if attempt_raw < 1:
            unattributed.append(
                (record, f"attempt_index {attempt_raw} outside 1..{declared_pairs}")
            )
            continue
        label = _arm_label(record)
        if label != ARM_LABEL:
            excluded_arm.append((record, f"arm label {label!r}, required {ARM_LABEL!r}"))
            continue
        seed = record.get("seed")
        expected_seed = SEED_POLICY[configuration]
        if seed != expected_seed:
            excluded_seed.append((record, f"seed {seed!r}, required {expected_seed!r}"))
            continue
        occupied = population[configuration].get(attempt_raw)
        if occupied is not None:
            duplicates.append(
                (
                    configuration,
                    attempt_raw,
                    str(occupied.get("run_id")),
                    str(record.get("run_id")),
                )
            )
            continue
        population[configuration][attempt_raw] = record

    missing_slots = {
        name: [index for index in range(1, declared_pairs + 1) if index not in population[name]]
        for name in configurations
    }
    # Pair p is the (p+1)-th attempt of each configuration under the interleaved driver, whatever
    # order the two runs executed in, so a pair is present only when both its slots are.
    incomplete_pairs = [
        pair_index
        for pair_index in range(declared_pairs)
        if any((pair_index + 1) not in population[name] for name in configurations)
    ]
    # Arm and seed exclusions leave their slot unfilled, so they surface as missing slots too;
    # duplicates do not (the first claim keeps the slot), so they gate completeness directly.
    population_complete = not any(missing_slots.values()) and not duplicates
    return {
        "population": population,
        "missing_slots": missing_slots,
        "incomplete_pairs": incomplete_pairs,
        "beyond_population": beyond_population,
        "excluded_arm": excluded_arm,
        "excluded_seed": excluded_seed,
        "unattributed": unattributed,
        "duplicates": duplicates,
        "population_complete": population_complete,
    }


def _goal_taxonomy(goals: list[dict[str, Any]]) -> dict[str, Any]:
    """Recompute family and failure-signature counts for a configuration's goals."""
    families: Counter[str] = Counter()
    signatures: dict[str, dict[str, Any]] = {}
    for goal in goals:
        family, signature = report.classify_goal(goal)
        families[family] += 1
        outcome = goal.get("outcome", {})
        if outcome.get("status") == taxonomy.STATUS_SUCCEEDED:
            continue
        entry = signatures.setdefault(
            signature,
            {
                "signature": signature,
                "family": family,
                "count": 0,
                "example_detail": outcome.get("detail_full") or outcome.get("detail", ""),
            },
        )
        entry["count"] += 1
    return {
        "families": dict(families.most_common()),
        "failure_signatures": sorted(
            signatures.values(), key=lambda entry: (-entry["count"], entry["signature"])
        ),
    }


def _rate_block(successes: int, total: int) -> dict[str, Any]:
    interval = recovery.wilson_interval(successes, total)
    return {
        "successes": successes,
        "total": total,
        "rate": (successes / total) if total else None,
        "wilson_95": interval,
        "wilson_95_text": recovery.format_interval(interval),
    }


def _configuration_block(records: list[dict[str, Any]], declared_pairs: int) -> dict[str, Any]:
    run_classes = Counter(classify_run(record) for record in records)
    goals = [goal for record in records for goal in (record.get("goals") or [])]
    succeeded = sum(
        1 for goal in goals if goal.get("outcome", {}).get("status") == taxonomy.STATUS_SUCCEEDED
    )
    bands = Counter(load_band(_load1_start(record), _cpu_count(record)) for record in records)
    loads = [
        float(value)
        for record in records
        if (value := _load1_start(record)) is not None
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
    ]
    taxonomy_block = _goal_taxonomy(goals)
    # A configuration whose every goal is a harness terminal (the goal never reached the
    # coordinator) measured no transfer pipeline at all; its numeric rate must not read as one.
    all_goals_harness_terminal = bool(goals) and all(
        int(goal.get("outcome", {}).get("status", taxonomy.HARNESS_STATUS))
        == taxonomy.HARNESS_STATUS
        for goal in goals
    )
    return {
        "declared_pairs": declared_pairs,
        "runs": len(records),
        "run_classes": dict(run_classes),
        "runs_unclassified": run_classes.get("unclassified", 0),
        "run_success": _rate_block(run_classes.get("clean", 0), len(records)),
        "transfer_success": _rate_block(succeeded, len(goals)),
        "all_goals_harness_terminal": all_goals_harness_terminal,
        "load": {
            "bands": dict(bands),
            "load1_start_min": min(loads) if loads else None,
            "load1_start_median": statistics.median(loads) if loads else None,
            "load1_start_max": max(loads) if loads else None,
        },
        "families": taxonomy_block["families"],
        "failure_signatures": taxonomy_block["failure_signatures"],
    }


def _run_id(record: dict[str, Any]) -> str:
    return str(record.get("run_id") or "<no run_id>")


def _pooled_label(summary: dict[str, Any], *, markdown: bool) -> str:
    """
    Label the pooled-secondary row for the configurations this summary actually scopes.

    With both paired configurations in scope the row mixes the two fixtures and says so; with a
    single-configuration scope (the dense addendum) there is nothing to mix, and claiming a mix
    would misdescribe the table.
    """
    single = len(summary.get("configurations") or ()) <= 1
    if markdown:
        return (
            "Pooled (secondary — single configuration in scope, not a separate population): "
            if single
            else "Pooled (secondary — mixes the two fixtures, never the headline claim): "
        )
    return (
        "pooled (secondary, single configuration in scope):"
        if single
        else "pooled (secondary, mixes fixtures):"
    )


def _check_configurations(configurations: tuple[str, ...]) -> None:
    """Refuse an empty or unknown population scope rather than silently summarising nothing."""
    if not configurations:
        raise ValueError("configurations must not be empty")
    unknown = [name for name in configurations if name not in CONFIGURATIONS]
    if unknown:
        raise ValueError(
            f"unknown configuration(s) {unknown}; the declared configurations are "
            f"{list(CONFIGURATIONS)}"
        )


def summarise(
    runs: list[dict[str, Any]],
    declared_pairs: int = PAIRS_PER_CONFIGURATION,
    *,
    configurations: tuple[str, ...] = CONFIGURATIONS,
    campaign: str = CAMPAIGN_NAME,
) -> dict[str, Any]:
    """
    Aggregate campaign records against a frozen predeclaration without mutating them.

    The defaults summarise the original paired population. The dense addendum calls this with
    ``configurations=ADDENDUM_CONFIGURATIONS, campaign=ADDENDUM_CAMPAIGN_NAME`` so its records
    aggregate against their own declaration and never against the paired one.
    """
    _check_configurations(configurations)
    parts = partition(runs, declared_pairs, configurations=configurations)
    population = parts["population"]
    blocks = {
        name: _configuration_block(
            [population[name][index] for index in sorted(population[name])], declared_pairs
        )
        for name in configurations
    }
    pooled_records = [record for name in configurations for record in population[name].values()]
    pooled = _configuration_block(pooled_records, declared_pairs)

    unclassified = [
        _run_id(record)
        for name in configurations
        for record in population[name].values()
        if classify_run(record) == "unclassified"
    ]
    summary: dict[str, Any] = {
        "campaign": campaign,
        "declared_pairs": declared_pairs,
        "configurations": list(configurations),
        "arm_label_required": ARM_LABEL,
        "seed_policy": {name: SEED_POLICY[name] for name in configurations},
        "goals_per_run": {name: GOALS_PER_RUN[name] for name in configurations},
        "population_complete": parts["population_complete"],
        "coverage": {
            name: {
                "slots": declared_pairs,
                "filled": len(population[name]),
                "missing": parts["missing_slots"][name],
            }
            for name in configurations
        },
        "incomplete_pairs": parts["incomplete_pairs"],
        "retained": {
            "total_records": len(runs),
            "beyond_population": [_run_id(record) for record in parts["beyond_population"]],
            "excluded_arm": [
                {"run_id": _run_id(record), "reason": reason}
                for record, reason in parts["excluded_arm"]
            ],
            "excluded_seed": [
                {"run_id": _run_id(record), "reason": reason}
                for record, reason in parts["excluded_seed"]
            ],
            "unattributed": [
                {"run_id": _run_id(record), "reason": reason}
                for record, reason in parts["unattributed"]
            ],
            "duplicates": [
                {
                    "configuration": name,
                    "attempt_index": index,
                    "kept_run_id": kept,
                    "duplicate_run_id": duplicate,
                }
                for name, index, kept, duplicate in parts["duplicates"]
            ],
        },
        "all_runs_classified": not unclassified,
        "unclassified_runs": unclassified,
        "by_configuration": blocks,
        # Secondary: mixes the configurations in scope, so it is never the headline claim.
        "pooled_secondary": pooled,
        "inference_boundary": inference_boundary(declared_pairs),
    }
    return summary


def format_summary(summary: dict[str, Any]) -> str:
    """Render a summarise() result as plain text for summary.txt."""
    boundary = summary["inference_boundary"]
    lines = [
        f"{summary['campaign']}: {summary['declared_pairs']} declared pairs, "
        f"required arm {summary['arm_label_required']!r}",
        "",
        "population coverage:",
    ]
    for name in summary["configurations"]:
        coverage = summary["coverage"][name]
        lines.append(
            f"  {name:<24} {coverage['filled']}/{coverage['slots']} filled"
            + (f" missing={coverage['missing']}" if coverage["missing"] else "")
        )
    lines.append(
        f"  population_complete={summary['population_complete']} "
        f"incomplete_pairs={summary['incomplete_pairs']}"
    )
    retained = summary["retained"]
    lines.append(
        f"  retained records={retained['total_records']} "
        f"beyond_population={len(retained['beyond_population'])} "
        f"excluded_arm={len(retained['excluded_arm'])} "
        f"excluded_seed={len(retained['excluded_seed'])} "
        f"unattributed={len(retained['unattributed'])} "
        f"duplicates={len(retained['duplicates'])}"
    )
    lines += ["", "per configuration:"]
    for name in summary["configurations"]:
        block = summary["by_configuration"][name]
        run = block["run_success"]
        transfer = block["transfer_success"]
        run_text = (
            f"{run['successes']}/{run['total']} ({run['wilson_95_text']})"
            if run["total"]
            else "n/a"
        )
        transfer_text = (
            f"{transfer['successes']}/{transfer['total']} ({transfer['wilson_95_text']})"
            if transfer["total"]
            else "n/a"
        )
        if block.get("all_goals_harness_terminal"):
            transfer_text += " [harness: no transfer rate]"
        lines.append(f"  {name}")
        lines.append(f"    runs clean   {run_text}  classes={block['run_classes']}")
        lines.append(f"    transfers    {transfer_text}")
        lines.append(f"    load bands   {block['load']['bands']} {block['load']}")
        if block["failure_signatures"]:
            lines.append("    failure signatures:")
            for entry in block["failure_signatures"]:
                lines.append(
                    f"      x{entry['count']} {entry['family']}: {entry['signature'][:120]}"
                )
        else:
            lines.append("    failure signatures: none recorded")
    lines += [
        "",
        _pooled_label(summary, markdown=False),
        f"  runs clean    "
        f"{summary['pooled_secondary']['run_success']['successes']}/"
        f"{summary['pooled_secondary']['run_success']['total']} "
        f"({summary['pooled_secondary']['run_success']['wilson_95_text']})",
        f"  transfers     "
        f"{summary['pooled_secondary']['transfer_success']['successes']}/"
        f"{summary['pooled_secondary']['transfer_success']['total']} "
        f"({summary['pooled_secondary']['transfer_success']['wilson_95_text']})",
        "",
        "inference boundary (frozen):",
        f"  planned transfers per configuration "
        f"{boundary['transfers_per_configuration_planned']}, "
        f"runs {boundary['runs_per_configuration']}",
        f"  design half-widths at p={boundary['design_reference_rate']}: "
        f"transfers/config ±{boundary['half_width_transfers_per_configuration'] * 100:.1f} pp, "
        f"transfers pooled ±{boundary['half_width_transfers_pooled'] * 100:.1f} pp, "
        f"runs/config ±{boundary['half_width_runs_per_configuration'] * 100:.1f} pp",
        f"  zero-failure run upper bound "
        f"{(boundary['zero_failure_run_upper_bound'] or float('nan')) * 100:.1f}%; "
        f"P(zero failures | historical "
        f"{boundary['historical_run_failure_rate']:.0%}) = "
        f"{boundary['probability_zero_failures_if_historical']:.2e}",
        f"  all_runs_classified={summary['all_runs_classified']} "
        f"unclassified={summary['unclassified_runs']}",
        "",
    ]
    return "\n".join(lines)


def render_markdown(summary: dict[str, Any]) -> str:
    """Render the campaign report, design statement and result tables as Markdown."""
    boundary = summary["inference_boundary"]
    lines = [
        f"# {summary['campaign']} campaign report",
        "",
        "Predeclared paired campaign: two fixed-pose configurations interleaved one pair at a "
        "time, every attempt retained and classified, UR10e records only, load annotated and "
        "never used to exclude a run. No retry replaces a failed denominator row.",
        "",
        "## Population coverage",
        "",
        "| Configuration | Slots filled | Missing | Run classes |",
        "| --- | ---: | --- | --- |",
    ]
    for name in summary["configurations"]:
        coverage = summary["coverage"][name]
        classes = summary["by_configuration"][name]["run_classes"]
        missing = ", ".join(str(index) for index in coverage["missing"]) or "none"
        lines.append(
            f"| `{name}` | {coverage['filled']}/{coverage['slots']} | {missing} | `{classes}` |"
        )
    retained = summary["retained"]
    lines += [
        "",
        f"- Population complete: **{summary['population_complete']}** "
        f"(incomplete pairs: {summary['incomplete_pairs'] or 'none'})",
        f"- Retained records: {retained['total_records']}; beyond population: "
        f"{len(retained['beyond_population'])}; excluded (arm): "
        f"{len(retained['excluded_arm'])}; excluded (seed): "
        f"{len(retained['excluded_seed'])}; unattributed: {len(retained['unattributed'])}; "
        f"duplicate slots: {len(retained['duplicates'])}",
        f"- Every population run classified: **{summary['all_runs_classified']}**"
        + (
            f" (unclassified: {summary['unclassified_runs']})"
            if summary["unclassified_runs"]
            else ""
        ),
        "",
        "## Results",
        "",
        "| Configuration | Runs clean (Wilson 95%) | Transfers succeeded (Wilson 95%) | "
        "Load bands (quiet/contended/unknown) | Load1 min/median/max |",
        "| --- | --- | --- | --- | --- |",
    ]
    for name in summary["configurations"]:
        block = summary["by_configuration"][name]
        run = block["run_success"]
        transfer = block["transfer_success"]
        load = block["load"]
        bands = load["bands"]
        band_text = "/".join(str(bands.get(key, 0)) for key in ("quiet", "contended", "unknown"))
        load_text = (
            f"{load['load1_start_min']:.1f}/{load['load1_start_median']:.1f}/"
            f"{load['load1_start_max']:.1f}"
            if load["load1_start_min"] is not None
            else "n/a"
        )
        run_text = (
            f"{run['successes']}/{run['total']} ({run['wilson_95_text']})"
            if run["total"]
            else "n/a"
        )
        transfer_text = (
            f"{transfer['successes']}/{transfer['total']} ({transfer['wilson_95_text']})"
            if transfer["total"]
            else "n/a"
        )
        if block.get("all_goals_harness_terminal"):
            transfer_text += " — harness: no transfer rate"
        lines.append(f"| `{name}` | {run_text} | {transfer_text} | {band_text} | {load_text} |")
    pooled = summary["pooled_secondary"]
    lines += [
        "",
        _pooled_label(summary, markdown=True)
        + f"runs {pooled['run_success']['successes']}/{pooled['run_success']['total']} "
        f"({pooled['run_success']['wilson_95_text']}), "
        f"transfers {pooled['transfer_success']['successes']}/"
        f"{pooled['transfer_success']['total']} "
        f"({pooled['transfer_success']['wilson_95_text']}).",
        "",
        "## Failure taxonomy (population goals, recomputed from the records)",
        "",
    ]
    any_signature = False
    for name in summary["configurations"]:
        block = summary["by_configuration"][name]
        lines += [f"### `{name}`", "", f"Families: `{block['families']}`", ""]
        if block["failure_signatures"]:
            any_signature = True
            lines += ["| Count | Family | Signature |", "| ---: | --- | --- |"]
            for entry in block["failure_signatures"]:
                signature = entry["signature"].replace("|", "/")
                if len(signature) > 170:
                    signature = f"{signature[:105]} [...] {signature[-55:]}"
                lines.append(f"| {entry['count']} | `{entry['family']}` | {signature} |")
            lines.append("")
        else:
            lines += ["No failed goals recorded.", ""]
    if not any_signature:
        lines += [
            "A campaign this size with no failures bounds the failure rate; it does not prove "
            "the rate is zero. See the inference boundary below.",
            "",
        ]
    lines += [
        "## Inference boundary (frozen before the campaign)",
        "",
        f"- Declared pairs: **{boundary['pairs']}** "
        f"({boundary['runs_per_configuration']} runs and "
        f"{boundary['transfers_per_configuration_planned']} planned transfers per configuration; "
        f"{boundary['runs_pooled']} runs and "
        f"{boundary['transfers_pooled_planned']} transfers pooled).",
        f"- Design half-widths at p = {boundary['design_reference_rate']}: "
        f"transfers per configuration **±"
        f"{boundary['half_width_transfers_per_configuration'] * 100:.1f} pp**, pooled **±"
        f"{boundary['half_width_transfers_pooled'] * 100:.1f} pp**, runs per configuration **±"
        f"{boundary['half_width_runs_per_configuration'] * 100:.1f} pp** "
        "(reported intervals are Wilson; these half-widths are the frozen design statement).",
        f"- Zero run failures in {boundary['runs_per_configuration']} bounds the run-failure rate "
        f"at **{(boundary['zero_failure_run_upper_bound'] or float('nan')) * 100:.1f}%** "
        "(one-sided 95%, exact). If the true run-failure rate were the historical "
        f"{boundary['historical_run_failure_rate']:.0%}, the chance of seeing zero failures in "
        f"that many runs is {boundary['probability_zero_failures_if_historical']:.2e}.",
        "",
        "Supports: " + "; ".join(boundary["supports"]) + ".",
        "",
        "Does not support: " + "; ".join(boundary["excludes"]) + ".",
        "",
        "## Reproducing this",
        "",
        "```bash",
        "just benchmark-paired                # full predeclared campaign (lease permitting)",
        "just benchmark-paired --summarise-only   # re-aggregate an existing campaign directory",
        "```",
        "",
        "Each run archives its console log and spawned scenario document beside its `run.json`, "
        "with commit, stock digest, seeds, arm identity and load at start and finish.",
        "",
    ]
    return "\n".join(lines) + "\n"
