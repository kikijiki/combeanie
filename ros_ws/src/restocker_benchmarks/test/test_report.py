# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The report must count failures, group them by cause, and never invent a clearance."""

import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import report, taxonomy  # noqa: E402

TRUNCATED = (
    "linear path stopped {:.2f}% of the way to the goal after {} interpolated waypoints; "
    "MoveIt reported SUCCESS (code 1)"
)


def _goal(label, status, detail, states, preceded=False, wall=90.0):
    return {
        "label": label,
        "addressed": True,
        "preceded_by_failure": preceded,
        "wall_s": wall,
        "elapsed_sim_s": wall,
        "retry_count": 0,
        "recovery_attempts": 0,
        "phase_sim_s": {"motion_sim_s": 70.0, "other_sim_s": 20.0},
        "furthest_state": taxonomy.furthest_state(states),
        "furthest_state_name": taxonomy.state_name(taxonomy.furthest_state(states)),
        "minimum_clearance_m": None,
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


COMPLETE = [
    0,
    1,
    2,
    3,
    4,
    5,
    6,
    33,
    7,
    8,
    9,
    10,
    11,
    12,
    13,
    14,
    15,
    16,
    17,
    18,
    19,
    20,
    21,
    22,
    23,
    24,
    25,
    27,
    31,
]


def _run(run_id, scenario, goals, outcome="completed"):
    return {
        "schema_version": 1,
        "run_id": run_id,
        "scenario": scenario,
        "scenario_description": "test",
        "seed": "fixed",
        "outcome": outcome,
        "started_utc": "2026-09-08T00:00:00+00:00",
        "goals": goals,
        "ground_truth_observation": {
            "stocked": 3,
            "observed": 3,
            "max_translation_error_m": 3.0e-6,
            "per_product": [],
        },
        "provenance": {
            "git_describe": "abc1234",
            "flake_lock_sha256": "0" * 64,
            "planner_backend": {"planning_plugins": ["ompl_interface/OMPLPlanner"]},
            "model_backend": {"name": "none"},
            "perception_backend": "gazebo_ground_truth",
            "hardware": {"cpu_model": "test", "cpu_count": 4, "platform": "linux"},
        },
    }


def _campaign():
    return [
        _run(
            "run-a",
            "baseline_transfers",
            [
                _goal("can", taxonomy.STATUS_SUCCEEDED, "complete", COMPLETE),
                _goal(
                    "small bottle",
                    5,
                    TRUNCATED.format(31.25, 12),
                    [0, 1, 2, 3, 4, 5, 6, 33, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18],
                ),
                _goal("large bottle", 6, "the arm stopped", [0, 1, 2], preceded=True),
            ],
        ),
        _run(
            "run-b",
            "baseline_transfers",
            [
                # The same cause with different numbers must land on the same row as run-a's.
                _goal(
                    "can",
                    5,
                    TRUNCATED.format(74.50, 41),
                    [0, 1, 2, 3, 4, 5, 6, 33, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18],
                ),
            ],
        ),
    ]


def test_failures_are_grouped_by_cause_not_merely_counted():
    summary = report.summarize(_campaign())
    assert summary["goals"] == 4
    assert summary["goals_succeeded"] == 1
    signatures = {entry["signature"]: entry for entry in summary["failure_signatures"]}
    assert len(signatures) == 2, signatures
    truncated = next(
        entry
        for entry in summary["failure_signatures"]
        if entry["family"] == "cartesian_truncated"
    )
    assert truncated["count"] == 2
    assert truncated["independent"] == 2


def test_a_failure_that_inherited_a_broken_run_is_counted_separately():
    summary = report.summarize(_campaign())
    assert summary["goals_failed"] == 3
    assert summary["goals_failed_independent"] == 2
    cascaded = next(entry for entry in summary["failure_signatures"] if entry["cascaded"] == 1)
    assert cascaded["independent"] == 0


def test_stage_reach_drops_where_the_failures_stopped():
    summary = report.summarize(_campaign())
    reach = {entry["stage"]: entry["reached"] for entry in summary["stage_reach"]}
    # Three goals attached a product; only the one complete goal verified a placement.
    assert reach["attached the product"] == 3
    assert reach["verified the placement"] == 1


def test_clearance_is_reported_as_unavailable_and_never_as_zero():
    summary = report.summarize(_campaign())
    assert summary["minimum_clearance_m"] is None
    assert "no production path assigns" in summary["minimum_clearance_unavailable_reason"]
    markdown = report.render_markdown(_campaign(), summary, [])
    assert "NOT AVAILABLE" in markdown


def test_the_report_names_every_distinct_cause_with_its_full_detail():
    campaign = _campaign()
    markdown = report.render_markdown(campaign, report.summarize(campaign), [])
    assert "## Failure taxonomy" in markdown
    assert "cartesian_truncated" in markdown
    # The full, unmasked example is quoted so a reader can act on it.
    assert "interpolated waypoints" in markdown


def test_an_empty_campaign_reports_nothing_rather_than_a_pass():
    summary = report.summarize([])
    assert summary["goals"] == 0
    markdown = report.render_markdown([], summary, [])
    assert "No runs were found" in markdown


def test_a_campaign_round_trips_through_stored_json(tmp_path):
    directory = tmp_path / "baseline_transfers" / "run-a"
    directory.mkdir(parents=True)
    (directory / "run.json").write_text(json.dumps(_campaign()[0]), encoding="utf-8")
    assert report.main([str(tmp_path)]) == 0
    assert (tmp_path / "report.md").is_file()
    assert (tmp_path / "summary.json").is_file()
    for name in ("goal-outcomes.svg", "failure-depth.svg", "cycle-time.svg"):
        content = (tmp_path / "plots" / name).read_text(encoding="utf-8")
        assert content.startswith("<svg") and content.endswith("</svg>")
