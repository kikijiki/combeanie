# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The frozen predeclaration and its aggregation must hold without a simulator."""

from itertools import islice
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import recovery, taxonomy  # noqa: E402
from restocker_benchmarks import transfer_reliability as campaign  # noqa: E402


def _goal(status=taxonomy.STATUS_SUCCEEDED, detail="delivered", states=None, preceded=False):
    states = states if states is not None else [0, 1, 2, 31]
    return {
        "label": "goal",
        "addressed": True,
        "preceded_by_failure": preceded,
        "wall_s": 90.0,
        "elapsed_sim_s": 90.0,
        "retry_count": 0,
        "recovery_attempts": 0,
        "furthest_state": taxonomy.furthest_state(states),
        "furthest_state_name": taxonomy.state_name(taxonomy.furthest_state(states)),
        "trace": [
            {"state": state, "state_name": taxonomy.state_name(state), "elapsed_sim_s": 0.0}
            for state in states
        ],
        "outcome": {
            "status": status,
            "status_name": taxonomy.STATUS_NAMES.get(status, str(status)),
            "detail": detail,
            "detail_full": detail,
            "family": taxonomy.classify_family(status, detail),
            "signature": taxonomy.signature(status, detail),
        },
    }


def _goals(count=3, **overrides):
    return [_goal(**overrides) for _ in range(count)]


def _run(
    run_id,
    configuration,
    attempt_index,
    *,
    pair_index=None,
    outcome="completed",
    goals=None,
    arm="ur10e",
    seed="fixed",
    load1=4.0,
    cpu_count=32,
    scenario=None,
    started=None,
):
    if goals is None:
        goals = _goals()
    if pair_index is None:
        pair_index = attempt_index - 1
    if started is None:
        started = f"2026-09-23T00:{attempt_index:02d}:00+00:00"
    provenance = {
        "hardware": {"load_average_at_start": [load1, load1, load1], "cpu_count": cpu_count}
    }
    if arm is not None:
        provenance["arm"] = {"label": arm, "arm_xacro_sha256": "deadbeef"}
    return {
        "schema_version": 2,
        "run_id": run_id,
        "scenario": scenario if scenario is not None else configuration,
        "scenario_description": "test",
        "seed": seed,
        "configuration": configuration,
        "pair_index": pair_index,
        "attempt_index": attempt_index,
        "outcome": outcome,
        "started_utc": started,
        "provenance": provenance,
        "goals": goals,
    }


def _complete_campaign(**run_kwargs):
    """Build a full population at the declared pair count: every slot filled, all clean."""
    runs = []
    for pair in range(campaign.PAIRS_PER_CONFIGURATION):
        for configuration in campaign.CONFIGURATIONS:
            runs.append(
                _run(
                    f"{configuration}-{pair + 1:02d}",
                    configuration,
                    pair + 1,
                    pair_index=pair,
                    **run_kwargs,
                )
            )
    return runs


# ---------------------------------------------------------------------------
# The predeclaration itself
# ---------------------------------------------------------------------------


def test_the_predeclaration_is_frozen():
    assert campaign.CAMPAIGN_NAME == "ur10e-transfer-reliability"
    assert campaign.CONFIGURATIONS == ("baseline_transfers", "dense_restock_transfers")
    assert campaign.PAIRS_PER_CONFIGURATION == 24
    assert campaign.ARM_LABEL == "ur10e"
    assert campaign.SEED_POLICY == {
        "baseline_transfers": "fixed",
        "dense_restock_transfers": "fixed",
    }
    assert campaign.LOAD_QUIET_FRACTION == 0.5
    assert campaign.RUN_CLASSES == (
        "clean",
        "completed_with_failure",
        "startup_failed",
        "launch_exited",
        "harness_exception",
        "unclassified",
    )


def test_planned_transfers_derive_from_the_scenario_table():
    """The denominator's size is read from the scenarios the runner actually executes."""
    assert campaign.GOALS_PER_RUN == {"baseline_transfers": 3, "dense_restock_transfers": 3}
    assert campaign.planned_transfers_per_configuration() == 72
    assert campaign.planned_transfers_per_configuration(5) == 15


def test_inference_boundary_matches_the_frozen_statement():
    boundary = campaign.inference_boundary()
    assert boundary["pairs"] == 24
    assert boundary["runs_per_configuration"] == 24
    assert boundary["runs_pooled"] == 48
    assert boundary["transfers_per_configuration_planned"] == 72
    assert boundary["transfers_pooled_planned"] == 144
    # The published design numbers, to a tenth of a percentage point / a bound of 1e-4.
    assert boundary["half_width_transfers_per_configuration"] == pytest.approx(0.1102, abs=1e-3)
    assert boundary["half_width_transfers_pooled"] == pytest.approx(0.0779, abs=1e-3)
    assert boundary["half_width_runs_per_configuration"] == pytest.approx(0.1908, abs=1e-3)
    assert boundary["zero_failure_run_upper_bound"] == pytest.approx(0.1173, abs=1e-3)
    assert boundary["probability_zero_failures_if_historical"] == pytest.approx(3.24e-5, abs=1e-6)
    assert boundary["design_reference_rate"] == 0.65
    assert boundary["historical_run_failure_rate"] == 0.35
    # The boundary must say what its size cannot do, not only what it can.
    assert any("retired" in claim for claim in boundary["excludes"])
    assert any("dense-demo" in claim for claim in boundary["excludes"])


def test_planned_transfers_refuse_a_configuration_drift():
    """If the scenarios ever disagree on goal count the scalar boundary must not paper over it."""
    original = campaign.GOALS_PER_RUN["dense_restock_transfers"]
    campaign.GOALS_PER_RUN["dense_restock_transfers"] = 5
    try:
        with pytest.raises(ValueError):
            campaign.planned_transfers_per_configuration()
    finally:
        campaign.GOALS_PER_RUN["dense_restock_transfers"] = original


# ---------------------------------------------------------------------------
# Load bands and run classification
# ---------------------------------------------------------------------------


def test_load_band_splits_at_half_the_cpu_count():
    assert campaign.load_band(15.9, 32) == "quiet"
    assert campaign.load_band(16.0, 32) == "contended"
    assert campaign.load_band(44.0, 32) == "contended"
    assert campaign.load_band(0.0, 8) == "quiet"
    assert campaign.load_band(None, 32) == "unknown"
    assert campaign.load_band(4.0, None) == "unknown"
    assert campaign.load_band(4.0, 0) == "unknown"
    assert campaign.load_band(-1.0, 32) == "unknown"
    assert campaign.load_band(float("nan"), 32) == "unknown"


def test_classify_run_covers_every_declared_class():
    assert campaign.classify_run(_run("a", "baseline_transfers", 1)) == "clean"
    failed = _run("b", "baseline_transfers", 1, goals=_goals(2) + [_goal(5, "planning failed")])
    assert campaign.classify_run(failed) == "completed_with_failure"
    startup = _run("c", "baseline_transfers", 1, outcome="startup_failed", goals=[])
    assert campaign.classify_run(startup) == "startup_failed"
    assert (
        campaign.classify_run(_run("d", "baseline_transfers", 1, outcome="launch_exited"))
        == "launch_exited"
    )
    assert (
        campaign.classify_run(_run("e", "baseline_transfers", 1, outcome="harness_exception"))
        == "harness_exception"
    )
    assert campaign.classify_run(_run("f", "baseline_transfers", 1, outcome="mystery")) == (
        "unclassified"
    )
    # Completed with no goals proved nothing and must not count as clean.
    assert campaign.classify_run(_run("g", "baseline_transfers", 1, goals=[])) == (
        "completed_with_failure"
    )


# ---------------------------------------------------------------------------
# SC-002: retention, classification, no replacement
# ---------------------------------------------------------------------------


def test_a_complete_population_is_complete_and_fully_classified():
    summary = campaign.summarise(_complete_campaign())
    assert summary["population_complete"] is True
    assert summary["incomplete_pairs"] == []
    assert summary["all_runs_classified"] is True
    assert summary["unclassified_runs"] == []
    for name in campaign.CONFIGURATIONS:
        assert summary["coverage"][name] == {
            "slots": 24,
            "filled": 24,
            "missing": [],
        }
        assert summary["by_configuration"][name]["run_classes"] == {"clean": 24}
        assert summary["by_configuration"][name]["run_success"]["successes"] == 24
        assert summary["by_configuration"][name]["transfer_success"]["successes"] == 72
    assert summary["pooled_secondary"]["run_success"]["total"] == 48
    assert summary["pooled_secondary"]["transfer_success"]["total"] == 144


def test_missing_slots_report_incomplete_instead_of_narrowing_the_population():
    runs = _complete_campaign()
    removed = next(
        run
        for run in runs
        if run["configuration"] == "dense_restock_transfers" and run["attempt_index"] == 3
    )
    runs.remove(removed)
    summary = campaign.summarise(runs)
    assert summary["population_complete"] is False
    assert summary["coverage"]["dense_restock_transfers"]["missing"] == [3]
    assert summary["coverage"]["baseline_transfers"]["missing"] == []
    assert summary["incomplete_pairs"] == [2]
    # The denominator that exists is still reported; it is just not claimable as complete.
    assert summary["by_configuration"]["dense_restock_transfers"]["runs"] == 23


def test_a_failed_run_stays_in_and_a_rerun_does_not_replace_it():
    """No-replacement: attempt 1 fails, attempt 2 (beyond a 1-pair declaration) cannot swap in."""
    failed = _run(
        "baseline-01",
        "baseline_transfers",
        1,
        goals=_goals(2) + [_goal(6, "trajectory execution was aborted")],
    )
    rerun = _run("baseline-02", "baseline_transfers", 2, pair_index=0)
    summary = campaign.summarise(
        [failed, _run("dense-01", "dense_restock_transfers", 1)], declared_pairs=1
    )
    assert summary["population_complete"] is True
    block = summary["by_configuration"]["baseline_transfers"]
    assert block["run_classes"] == {"completed_with_failure": 1}
    assert block["run_success"]["successes"] == 0
    assert block["transfer_success"]["successes"] == 2
    assert block["transfer_success"]["total"] == 3

    with_rerun = campaign.summarise(
        [failed, rerun, _run("dense-01", "dense_restock_transfers", 1)], declared_pairs=1
    )
    # The failed row still holds its slot; the re-run is retained and excluded from rates.
    assert with_rerun["retained"]["beyond_population"] == ["baseline-02"]
    assert with_rerun["retained"]["total_records"] == 3
    assert with_rerun["population_complete"] is True
    rerun_block = with_rerun["by_configuration"]["baseline_transfers"]
    assert rerun_block["run_classes"] == {"completed_with_failure": 1}
    assert rerun_block["run_success"]["successes"] == 0


def test_an_unclassified_outcome_fails_the_classification_check():
    runs = _complete_campaign()
    runs[0]["outcome"] = "mystery_outcome"
    summary = campaign.summarise(runs)
    assert summary["all_runs_classified"] is False
    assert summary["unclassified_runs"] == [runs[0]["run_id"]]
    assert summary["by_configuration"]["baseline_transfers"]["runs_unclassified"] == 1


def test_duplicate_slots_fail_completeness_and_keep_the_first_claim():
    first = _run("a-1", "baseline_transfers", 1, started="2026-09-23T00:01:00+00:00")
    second = _run("a-1-again", "baseline_transfers", 1, started="2026-09-23T09:00:00+00:00")
    summary = campaign.summarise(
        [first, second, _run("d-1", "dense_restock_transfers", 1)], declared_pairs=1
    )
    assert summary["population_complete"] is False
    assert summary["retained"]["duplicates"] == [
        {
            "configuration": "baseline_transfers",
            "attempt_index": 1,
            "kept_run_id": "a-1",
            "duplicate_run_id": "a-1-again",
        }
    ]
    assert summary["by_configuration"]["baseline_transfers"]["runs"] == 1


def test_unattributed_records_are_retained_and_reported():
    no_configuration = _run("x", "baseline_transfers", 1)
    del no_configuration["configuration"]
    mismatched = _run("y", "baseline_transfers", 1, scenario="lane_routing")
    summary = campaign.summarise(
        [no_configuration, mismatched, _run("d-1", "dense_restock_transfers", 1)],
        declared_pairs=1,
    )
    reasons = {entry["run_id"]: entry["reason"] for entry in summary["retained"]["unattributed"]}
    assert reasons["x"] == "missing or unknown configuration"
    assert "does not match" in reasons["y"]
    assert summary["retained"]["total_records"] == 3
    assert summary["coverage"]["baseline_transfers"]["filled"] == 0
    assert summary["population_complete"] is False


# ---------------------------------------------------------------------------
# SC-003: the arm gate
# ---------------------------------------------------------------------------


def test_retired_arm_records_are_retained_but_never_counted():
    summary = campaign.summarise(
        [
            _run("ur-1", "baseline_transfers", 1),
            _run("old-1", "baseline_transfers", 1, arm=None, started="2026-09-01T00:00:00+00:00"),
            _run("other-1", "baseline_transfers", 1, arm="custom_arm"),
            _run("d-1", "dense_restock_transfers", 1),
        ],
        declared_pairs=1,
    )
    excluded = {entry["run_id"] for entry in summary["retained"]["excluded_arm"]}
    assert excluded == {"old-1", "other-1"}
    reasons = {entry["run_id"]: entry["reason"] for entry in summary["retained"]["excluded_arm"]}
    assert reasons["old-1"] == "arm label None, required 'ur10e'"
    assert reasons["other-1"] == "arm label 'custom_arm', required 'ur10e'"
    # The UR10e record still holds the slot: exclusions are reported, not slot-filling.
    assert summary["by_configuration"]["baseline_transfers"]["runs"] == 1
    assert summary["by_configuration"]["baseline_transfers"]["run_success"]["successes"] == 1
    assert summary["coverage"]["baseline_transfers"]["filled"] == 1
    assert summary["population_complete"] is True


def test_population_rates_never_include_a_non_ur10e_record():
    runs = [
        _run("old-1", "baseline_transfers", 1, arm=None),
        _run("d-1", "dense_restock_transfers", 1),
    ]
    summary = campaign.summarise(runs, declared_pairs=1)
    baseline = summary["by_configuration"]["baseline_transfers"]
    assert baseline["runs"] == 0
    assert baseline["run_success"]["total"] == 0
    assert baseline["run_success"]["rate"] is None
    assert summary["coverage"]["baseline_transfers"]["missing"] == [1]
    assert summary["population_complete"] is False


def test_a_numeric_seed_is_a_predeclaration_violation():
    summary = campaign.summarise(
        [
            _run("b-1", "baseline_transfers", 1, seed="42"),
            _run("d-1", "dense_restock_transfers", 1),
        ],
        declared_pairs=1,
    )
    assert [entry["run_id"] for entry in summary["retained"]["excluded_seed"]] == ["b-1"]
    assert "required 'fixed'" in summary["retained"]["excluded_seed"][0]["reason"]
    assert summary["coverage"]["baseline_transfers"]["filled"] == 0
    assert summary["population_complete"] is False


# ---------------------------------------------------------------------------
# Rates, taxonomy and load annotation
# ---------------------------------------------------------------------------


def test_wilson_intervals_are_the_reported_interval():
    block = campaign._rate_block(24, 24)
    low, high = block["wilson_95"]
    assert low == pytest.approx(0.8620, abs=1e-3)
    assert high == pytest.approx(1.0)
    assert block["wilson_95_text"] == "86.2-100.0%"
    assert campaign._rate_block(0, 0)["rate"] is None
    # A zero-success sample still yields a usable upper bound rather than nothing.
    zero = campaign._rate_block(0, 24)
    assert zero["wilson_95"][1] > 0.0


def test_failure_taxonomy_is_recomputed_from_the_records():
    truncated = (
        "linear path stopped 41.6667% of the way to the goal after 17 interpolated waypoints"
    )
    runs = [
        _run(
            "b-1",
            "baseline_transfers",
            1,
            goals=_goals(2) + [_goal(5, truncated, states=[0, 1, 2, 33, 7, 8])],
        ),
        _run("d-1", "dense_restock_transfers", 1),
    ]
    summary = campaign.summarise(runs, declared_pairs=1)
    failures = summary["by_configuration"]["baseline_transfers"]["failure_signatures"]
    assert len(failures) == 1
    assert failures[0]["family"] == "cartesian_truncated"
    assert failures[0]["count"] == 1
    families = summary["by_configuration"]["baseline_transfers"]["families"]
    assert families["succeeded"] == 2
    assert families["cartesian_truncated"] == 1
    # classify_goal is the same function the general report uses.
    from restocker_benchmarks import report  # noqa: PLC0415

    assert report.classify_goal(runs[0]["goals"][2])[0] == "cartesian_truncated"


def test_load_is_annotated_by_band_and_never_excludes():
    runs = [
        _run("b-quiet", "baseline_transfers", 1, load1=4.0, cpu_count=32),
        _run("b-loaded", "baseline_transfers", 2, load1=40.0, cpu_count=32),
        _run("d-quiet", "dense_restock_transfers", 1, load1=4.0, cpu_count=32),
    ]
    summary = campaign.summarise(runs, declared_pairs=2)
    baseline = summary["by_configuration"]["baseline_transfers"]
    assert baseline["runs"] == 2  # both counted: contended runs are not dropped
    assert baseline["load"]["bands"] == {"quiet": 1, "contended": 1}
    assert baseline["load"]["load1_start_min"] == 4.0
    assert baseline["load"]["load1_start_max"] == 40.0
    assert baseline["load"]["load1_start_median"] == 22.0


def test_an_empty_campaign_reports_incomplete_not_a_certainty():
    summary = campaign.summarise([])
    assert summary["population_complete"] is False
    assert summary["retained"]["total_records"] == 0
    for name in campaign.CONFIGURATIONS:
        assert summary["by_configuration"][name]["run_success"]["rate"] is None
        assert summary["coverage"][name]["filled"] == 0
        assert summary["coverage"][name]["missing"] == list(range(1, 25))


def test_format_summary_and_report_carry_coverage_boundary_and_taxonomy():
    truncated = (
        "linear path stopped 41.6667% of the way to the goal after 17 interpolated waypoints"
    )
    runs = [
        _run(
            f"{configuration}-{pair + 1:02d}",
            configuration,
            pair + 1,
            pair_index=pair,
            **(
                {"goals": _goals(2) + [_goal(5, truncated, states=[0, 1, 2, 33, 7, 8])]}
                if (configuration, pair) == ("baseline_transfers", 0)
                else {}
            ),
        )
        for pair in range(2)
        for configuration in campaign.CONFIGURATIONS
    ]
    summary = campaign.summarise(runs, declared_pairs=2)
    text = campaign.format_summary(summary)
    assert "population_complete=True" in text
    assert "inference boundary (frozen)" in text
    assert "zero-failure run upper bound" in text
    assert "cartesian_truncated" in text
    assert "transfers/config" in text

    markdown = campaign.render_markdown(summary)
    assert "## Population coverage" in markdown
    assert "## Inference boundary (frozen before the campaign)" in markdown
    assert "No retry replaces a failed denominator row" in markdown
    assert "cartesian_truncated" in markdown
    assert "Does not support" in markdown


def test_the_design_reference_rate_is_still_separated_by_this_sample_size():
    """The frozen half-widths, recomputed here, must stay the published numbers."""
    n_transfers = campaign.planned_transfers_per_configuration()
    half_width = 1.96 * math.sqrt(0.65 * 0.35 / n_transfers)
    assert half_width < 0.115  # the ±11.0 pp claim
    upper = 1.0 - 0.05 ** (1.0 / campaign.PAIRS_PER_CONFIGURATION)
    assert upper < 0.12  # the 11.7 % zero-failure bound
    assert recovery.wilson_interval(0, campaign.PAIRS_PER_CONFIGURATION) is not None


def _harness_goal(reason="scenario product was never observed"):
    """Build a goal record the harness wrote without the coordinator ever seeing it."""
    goal = _goal(status=taxonomy.HARNESS_STATUS, detail=reason, states=[])
    goal["outcome"]["reason"] = reason
    return goal


def test_all_harness_terminal_configuration_is_labelled_no_transfer_rate():
    """A configuration whose every goal is a harness terminal must not read as a rate."""
    runs = [
        _run("b-1", "baseline_transfers", 1, goals=_goals(3)),
        _run("d-1", "dense_restock_transfers", 1, goals=[_harness_goal() for _ in range(3)]),
    ]
    summary = campaign.summarise(runs, declared_pairs=1)
    dense = summary["by_configuration"]["dense_restock_transfers"]
    baseline = summary["by_configuration"]["baseline_transfers"]
    assert dense["all_goals_harness_terminal"] is True
    assert baseline["all_goals_harness_terminal"] is False
    # The pooled row mixes harness and real goals, so it keeps its numeric rate.
    assert summary["pooled_secondary"]["all_goals_harness_terminal"] is False

    text = campaign.format_summary(summary)
    assert text.count("[harness: no transfer rate]") == 1
    lines = text.splitlines()
    dense_index = next(
        i for i, line in enumerate(lines) if line.strip() == "dense_restock_transfers"
    )
    # islice, not lines[i:i+4]: ruff-format would re-space the slice and trip ament E203.
    assert any(
        "[harness: no transfer rate]" in line
        for line in islice(lines, dense_index, dense_index + 4)
    )
    baseline_index = next(
        i for i, line in enumerate(lines) if line.strip() == "baseline_transfers"
    )
    assert not any(
        "harness: no transfer rate" in line
        for line in islice(lines, baseline_index, baseline_index + 4)
    )

    markdown = campaign.render_markdown(summary)
    assert markdown.count("harness: no transfer rate") == 1
    # Two tables carry configuration rows (coverage and results); the annotation belongs to the
    # results row, so assert across every row for that configuration.
    dense_rows = [
        line for line in markdown.splitlines() if line.startswith("| `dense_restock_transfers`")
    ]
    assert any("harness: no transfer rate" in row for row in dense_rows)
    baseline_rows = [
        line for line in markdown.splitlines() if line.startswith("| `baseline_transfers`")
    ]
    assert baseline_rows
    assert all("harness: no transfer rate" not in row for row in baseline_rows)


def test_a_configuration_with_any_real_goal_keeps_its_numeric_rate():
    """One non-harness goal is enough that the row must stay a plain rate."""
    runs = [
        _run("b-1", "baseline_transfers", 1, goals=_goals(3)),
        _run("d-1", "dense_restock_transfers", 1, goals=[_harness_goal(), *_goals(2)]),
    ]
    summary = campaign.summarise(runs, declared_pairs=1)
    dense = summary["by_configuration"]["dense_restock_transfers"]
    assert dense["all_goals_harness_terminal"] is False
    assert "harness: no transfer rate" not in campaign.format_summary(summary)
    assert "harness: no transfer rate" not in campaign.render_markdown(summary)


# ---------------------------------------------------------------------------
# The dense re-run addendum (frozen 2026-09-24 before any sample)
# ---------------------------------------------------------------------------


def test_the_addendum_is_frozen_and_separate_from_the_original_population():
    assert campaign.ADDENDUM_ID == "dense-rerun-2026-09-24"
    assert campaign.ADDENDUM_CAMPAIGN_NAME == "ur10e-transfer-reliability-dense-rerun"
    assert campaign.ADDENDUM_CONFIGURATIONS == ("dense_restock_transfers",)
    assert campaign.ADDENDUM_ATTEMPTS == 24
    assert campaign.ADDENDUM_CAMPAIGN_NAME != campaign.CAMPAIGN_NAME
    assert campaign.ADDENDUM_CONFIGURATIONS != campaign.CONFIGURATIONS
    # The original declarations are untouched by the addendum.
    assert campaign.CAMPAIGN_NAME == "ur10e-transfer-reliability"
    assert campaign.CONFIGURATIONS == ("baseline_transfers", "dense_restock_transfers")
    assert campaign.PAIRS_PER_CONFIGURATION == 24
    # Same arm gate, same seed policy, same load band as the original.
    assert campaign.ARM_LABEL == "ur10e"
    assert campaign.LOAD_QUIET_FRACTION == 0.5
    for name in campaign.ADDENDUM_CONFIGURATIONS:
        assert campaign.SEED_POLICY[name] == "fixed"
        assert name in campaign.CONFIGURATIONS


def _dense_addendum_population(**run_kwargs):
    """Build a full addendum population: every dense slot filled, in launch order."""
    return [
        _run(f"dense-{attempt:02d}", "dense_restock_transfers", attempt, **run_kwargs)
        for attempt in range(1, campaign.ADDENDUM_ATTEMPTS + 1)
    ]


def test_a_complete_dense_addendum_population_is_complete_on_its_own():
    summary = campaign.summarise(
        _dense_addendum_population(),
        declared_pairs=campaign.ADDENDUM_ATTEMPTS,
        configurations=campaign.ADDENDUM_CONFIGURATIONS,
        campaign=campaign.ADDENDUM_CAMPAIGN_NAME,
    )
    assert summary["campaign"] == campaign.ADDENDUM_CAMPAIGN_NAME
    assert summary["configurations"] == ["dense_restock_transfers"]
    assert summary["population_complete"] is True
    assert summary["coverage"] == {
        "dense_restock_transfers": {"slots": 24, "filled": 24, "missing": []}
    }
    assert "baseline_transfers" not in summary["by_configuration"]
    assert summary["all_runs_classified"] is True
    block = summary["by_configuration"]["dense_restock_transfers"]
    assert block["run_classes"] == {"clean": 24}
    assert block["transfer_success"]["successes"] == 72
    assert block["transfer_success"]["total"] == 72
    # Single-configuration scope: the pooled row must not claim to mix fixtures.
    text = campaign.format_summary(summary)
    markdown = campaign.render_markdown(summary)
    assert "single configuration in scope" in text
    assert "mixes fixtures" not in text
    assert "single configuration in scope" in markdown
    assert "mixes the two fixtures" not in markdown
    # ...while the original paired scope keeps the mix wording.
    paired = campaign.summarise(_complete_campaign())
    assert "mixes fixtures" in campaign.format_summary(paired)
    assert "mixes the two fixtures" in campaign.render_markdown(paired)
    assert "single configuration in scope" not in campaign.format_summary(paired)
    # The addendum inherits the same inference statement the original reserved per configuration.
    boundary = summary["inference_boundary"]
    assert boundary["runs_per_configuration"] == campaign.ADDENDUM_ATTEMPTS
    assert boundary["transfers_per_configuration_planned"] == 72
    assert boundary["zero_failure_run_upper_bound"] == pytest.approx(0.1173, abs=1e-3)


def test_the_original_population_is_not_completed_by_dense_rows_alone():
    """24 dense rows never fill the paired declaration — the addendum is a separate scope."""
    summary = campaign.summarise(_dense_addendum_population())
    assert summary["campaign"] == campaign.CAMPAIGN_NAME
    assert summary["configurations"] == list(campaign.CONFIGURATIONS)
    assert summary["population_complete"] is False
    assert summary["coverage"]["baseline_transfers"]["missing"] == list(range(1, 25))
    assert summary["coverage"]["dense_restock_transfers"]["filled"] == 24


def test_a_baseline_record_inside_the_dense_addendum_is_retained_not_counted():
    summary = campaign.summarise(
        [
            _run("d-1", "dense_restock_transfers", 1),
            _run("b-1", "baseline_transfers", 1),
        ],
        declared_pairs=1,
        configurations=campaign.ADDENDUM_CONFIGURATIONS,
        campaign=campaign.ADDENDUM_CAMPAIGN_NAME,
    )
    assert summary["population_complete"] is True
    assert summary["retained"]["total_records"] == 2
    unattributed = {
        entry["run_id"]: entry["reason"] for entry in summary["retained"]["unattributed"]
    }
    assert "b-1" in unattributed
    assert "outside this population's scope" in unattributed["b-1"]
    assert summary["coverage"]["dense_restock_transfers"]["filled"] == 1
    assert "baseline_transfers" not in summary["coverage"]


def test_the_addendum_keeps_the_no_replacement_policy_beyond_attempt_24():
    beyond = _run("dense-25", "dense_restock_transfers", 25, started="2026-09-24T09:00:00+00:00")
    summary = campaign.summarise(
        [*_dense_addendum_population(), beyond],
        declared_pairs=campaign.ADDENDUM_ATTEMPTS,
        configurations=campaign.ADDENDUM_CONFIGURATIONS,
        campaign=campaign.ADDENDUM_CAMPAIGN_NAME,
    )
    assert summary["population_complete"] is True
    assert summary["retained"]["beyond_population"] == ["dense-25"]
    assert summary["by_configuration"]["dense_restock_transfers"]["runs"] == 24


def test_summarise_refuses_an_unusable_scope_rather_than_summarising_nothing():
    with pytest.raises(ValueError, match="must not be empty"):
        campaign.summarise([], configurations=())
    with pytest.raises(ValueError, match="unknown configuration"):
        campaign.summarise([], configurations=("lane_routing",))


def test_the_addendum_still_gates_on_arm_and_seed():
    runs = [
        _run("d-ok", "dense_restock_transfers", 1),
        _run(
            "d-old",
            "dense_restock_transfers",
            2,
            arm=None,
            started="2026-09-01T00:00:00+00:00",
        ),
        _run(
            "d-seed",
            "dense_restock_transfers",
            3,
            seed="7",
            started="2026-09-24T00:00:00+00:00",
        ),
    ]
    summary = campaign.summarise(
        runs,
        declared_pairs=3,
        configurations=campaign.ADDENDUM_CONFIGURATIONS,
        campaign=campaign.ADDENDUM_CAMPAIGN_NAME,
    )
    assert [entry["run_id"] for entry in summary["retained"]["excluded_arm"]] == ["d-old"]
    assert [entry["run_id"] for entry in summary["retained"]["excluded_seed"]] == ["d-seed"]
    # Exclusions leave their slots unfilled: the addendum is incomplete, never silently narrowed.
    assert summary["population_complete"] is False
    assert summary["coverage"]["dense_restock_transfers"]["missing"] == [2, 3]
