# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Recovery attempts must be counted as attempts, and the shared budget attributed correctly."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import recovery  # noqa: E402

TRUNCATED = (
    "insert motion planning failed: insert: linear path stopped {}% of the way to the goal "
    "after 48 interpolated waypoints; MoveIt reported SUCCESS (code 1)"
)
ABORTED = (
    "pre-insert motion execution failed: trajectory execution was rejected or aborted "
    "(MoveIt reported CONTROL_FAILED, code -4)"
)


def _goal(slots):
    """Build a goal whose trace holds only the recovery ladder, as (slot, state, detail)."""
    return {
        "trace": [
            {"state_name": state, "recovery_attempt": slot, "detail": detail, "attempt": 1}
            for slot, state, detail in slots
        ]
    }


def _run(goals):
    return {"goals": goals}


def test_a_goal_that_never_recovered_contributes_nothing():
    quiet = _goal([])
    assert recovery.recovery_slots(quiet) == []
    summary = recovery.summarize([_run([quiet])])
    assert summary["independence"] == []
    assert summary["budget_sharing"]["goals_entering_recovery"] == 0


def test_each_recovery_slot_is_one_attempt_and_carries_its_family():
    goal = _goal(
        [
            (1, "RECOVER", TRUNCATED.format(56)),
            (2, "RECOVER", TRUNCATED.format(80)),
            (2, "FAULT", TRUNCATED.format("79.1667")),
        ]
    )
    slots = recovery.recovery_slots(goal)
    assert [entry["slot"] for entry in slots] == [1, 2, 2]
    assert {entry["family"] for entry in slots} == {"cartesian_truncated"}
    assert [entry["terminal"] for entry in slots] == [False, False, True]


def test_a_cause_that_comes_back_is_reported_as_recurrence():
    repeated = _goal([(1, "RECOVER", TRUNCATED.format(56)), (2, "FAULT", TRUNCATED.format(80))])
    cleared = _goal([(1, "RECOVER", TRUNCATED.format(56))])
    rows = {row["family"]: row for row in recovery.independence([_run([repeated, cleared])])}
    assert rows["cartesian_truncated"]["entered"] == 2
    assert rows["cartesian_truncated"]["recurred"] == 1
    assert rows["ALL"]["entered"] == 2


def test_a_fault_that_emptied_the_pool_itself_is_not_reported_as_starved():
    # Three truncations of the same cause: it spent both slots and then met the guard. The
    # first appearance had the full pool, so nothing "arrived with the pool empty".
    goal = _goal(
        [
            (1, "RECOVER", TRUNCATED.format(56)),
            (2, "RECOVER", TRUNCATED.format(80)),
            (2, "FAULT", TRUNCATED.format(79)),
        ]
    )
    sharing = recovery.budget_sharing([_run([goal])])
    entry = next(row for row in sharing["families"] if row["family"] == "cartesian_truncated")
    assert entry["first_seen_at_slot"] == {0: 1}
    assert entry["arrived_with_pool_empty"] == 0
    assert sharing["died_after_another_fault_spent_a_slot"] == 0
    assert sharing["died_with_the_pool_exhausted"] == 1


def test_a_fault_arriving_after_another_spent_the_pool_is_reported_as_starved():
    # The shared-budget case: two controller aborts take both slots, and the truncation that
    # follows meets the guard on its first appearance with no attempt of its own.
    goal = _goal(
        [
            (1, "RECOVER", ABORTED),
            (2, "RECOVER", ABORTED),
            (2, "FAULT", TRUNCATED.format(0)),
        ]
    )
    sharing = recovery.budget_sharing([_run([goal])])
    entry = next(row for row in sharing["families"] if row["family"] == "cartesian_truncated")
    assert entry["first_seen_at_slot"] == {2: 1}
    assert entry["arrived_with_pool_empty"] == 1
    assert sharing["died_after_another_fault_spent_a_slot"] == 1


def test_the_budget_bound_is_a_parameter_and_not_baked_in():
    goal = _goal([(1, "RECOVER", ABORTED), (1, "FAULT", TRUNCATED.format(0))])
    entry_at_two = next(
        row
        for row in recovery.budget_sharing([_run([goal])], budget=2)["families"]
        if row["family"] == "cartesian_truncated"
    )
    entry_at_one = next(
        row
        for row in recovery.budget_sharing([_run([goal])], budget=1)["families"]
        if row["family"] == "cartesian_truncated"
    )
    assert entry_at_two["arrived_with_pool_empty"] == 0
    assert entry_at_one["arrived_with_pool_empty"] == 1


def test_wilson_interval_brackets_the_point_estimate_and_survives_the_edges():
    low, high = recovery.wilson_interval(5, 11)
    assert low < 5 / 11 < high
    assert recovery.wilson_interval(0, 0) is None
    zero_low, zero_high = recovery.wilson_interval(0, 39)
    assert zero_low == 0.0
    # A rate of zero over 39 trials still admits rates far above 1-in-600; the report must not
    # present 0/39 as a demonstrated absence.
    assert zero_high > 1 / 600
    assert recovery.format_interval(None) == "n/a"


def test_summarize_reports_both_assumptions():
    goal = _goal([(1, "RECOVER", TRUNCATED.format(56)), (2, "FAULT", TRUNCATED.format(80))])
    summary = recovery.summarize([_run([goal])])
    assert summary["independence"]
    assert summary["budget_sharing"]["budget"] == 2
