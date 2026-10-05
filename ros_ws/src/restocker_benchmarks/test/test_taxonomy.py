# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The failure taxonomy must group equal causes and never silently absorb an unknown one."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import scenarios, taxonomy  # noqa: E402

# Verbatim from moveit_motion_port.cpp: the numbers differ between two occurrences of the same
# cause and the sentence does not, which normalization exploits.
TRUNCATED_A = (
    "linear path stopped 31.25% of the way to the goal after 12 interpolated waypoints; "
    "MoveIt reported SUCCESS (code 1)"
)
TRUNCATED_B = (
    "linear path stopped 74.5% of the way to the goal after 41 interpolated waypoints; "
    "MoveIt reported SUCCESS (code 1)"
)


def test_the_same_cause_with_different_numbers_shares_one_signature():
    assert taxonomy.signature(5, TRUNCATED_A) == taxonomy.signature(5, TRUNCATED_B)


def test_different_causes_do_not_share_a_signature():
    assert taxonomy.signature(5, TRUNCATED_A) != taxonomy.signature(
        7, "placement evidence was refused"
    )


def test_the_same_detail_under_different_statuses_stays_distinct():
    # Status is part of the key: the same sentence reported as a planning failure and as an
    # execution failure are different claims about what the robot did.
    assert taxonomy.signature(5, "the arm stopped") != taxonomy.signature(6, "the arm stopped")


def test_an_unknown_detail_is_labelled_rather_than_absorbed():
    family = taxonomy.classify_family(6, "something nobody has written a rule for yet")
    assert family == "unclassified"
    # It still carries a full signature, so it appears in a report as its own row.
    assert "something nobody has written a rule for yet" in taxonomy.signature(
        6, "something nobody has written a rule for yet"
    )


def test_success_is_not_a_failure_family():
    assert taxonomy.classify_family(taxonomy.STATUS_SUCCEEDED, "") == "succeeded"


def test_recoverable_skip_is_its_own_family():
    # Card 051 rung 5: the typed skip must never be absorbed by the recovery/timeout rules.
    assert (
        taxonomy.classify_family(
            13, "recoverable skip (rung 5): scripted test skip; skipped 1 of 2"
        )
        == "recoverable_skip"
    )
    assert taxonomy.STATUS_NAMES[13] == "SKIPPED_RECOVERABLE"


def test_families_pick_the_specific_subsystem():
    assert taxonomy.classify_family(5, TRUNCATED_A) == "cartesian_truncated"
    assert taxonomy.classify_family(6, "attachment submission was refused: no lease") == (
        "attachment"
    )
    assert taxonomy.classify_family(7, "placement evidence was not established") == (
        "placement_verification"
    )
    assert taxonomy.classify_family(3, "no compatible pair was admissible") == "selection"


def test_the_failure_modes_measured_on_this_system_land_in_distinct_families():
    # These four are the terminal details of a measured ten-run campaign: six passed, four failed.
    # They are pinned because separating them matters: an earlier campaign on the same scenario
    # failed mostly on attachment evidence, and the fix showed as a change in this distribution
    # while the aggregate pass rate barely moved.
    assert (
        taxonomy.classify_family(
            6, "pre-grasp motion execution failed (MoveIt reported CONTROL_FAILED, code -4)"
        )
        == "moveit_execution"
    )
    assert (
        taxonomy.classify_family(
            5, "pre-insert motion planning failed (MoveIt reported FAILURE, code -1)"
        )
        == "moveit_planning"
    )
    assert (
        taxonomy.classify_family(
            5,
            "approach motion planning failed: linear path stopped 41.6667% of the way to the "
            "goal after 17 interpolated waypoints; MoveIt reported SUCCESS (code 1)",
        )
        == "cartesian_truncated"
    )
    # And the family the earlier campaign was dominated by must not collide with any of them.
    assert taxonomy.classify_family(7, "attachment evidence was not established") == "attachment"


# Verbatim from moveit_motion_port.cpp, which appends the controller's own verdict to a
# CONTROL_FAILED. Before that, both arrived as the first string alone and the taxonomy had one row
# for them.
CONTROL_FAILED = (
    "pre-grasp motion execution failed: pre-grasp: trajectory execution was rejected or "
    "aborted (MoveIt reported CONTROL_FAILED, code -4)"
)
ABORT = (
    "controller 'arm_controller' aborted with PATH_TOLERANCE_VIOLATED, reporting "
    '"Aborted due to state tolerance violation"'
)
IMPULSE = (
    f"{CONTROL_FAILED}; {ABORT}; wrist_2_joint feedback velocity reached its URDF velocity "
    "limit of "
    "3.1000 rad/s during this segment, which planned that joint no faster than 0.0950 rad/s"
)
LAGGED = (
    f"{CONTROL_FAILED}; {ABORT}; no joint's feedback velocity was observed at its URDF velocity "
    "limit in the 412 joint-state samples covering this segment"
)


def test_a_controller_abort_is_labelled_by_what_the_controller_said():
    # A path-tolerance abort must not share a row with every other way a controller can stop,
    # because MoveIt reports them all as CONTROL_FAILED.
    assert taxonomy.classify_family(6, CONTROL_FAILED) == "moveit_execution"
    assert taxonomy.classify_family(6, LAGGED) == "controller_path_tolerance"
    assert (
        taxonomy.classify_family(
            6,
            f"{CONTROL_FAILED}; controller 'arm_controller' aborted with "
            'GOAL_TOLERANCE_VIOLATED, reporting "Aborted due to goal_time_tolerance exceeding '
            "by 1.006496 seconds\"; no joint's feedback velocity was observed at its URDF "
            "velocity limit in the 88 joint-state samples covering this segment",
        )
        == "controller_goal_tolerance"
    )


def test_the_velocity_limit_fingerprint_separates_the_impulse_from_tracking_lag():
    # Both are path-tolerance aborts. Only one has a joint at the bound the simulator clamps to
    # (the Signature B fingerprint); the report must count them apart by family and by signature.
    assert taxonomy.classify_family(6, IMPULSE) == "controller_velocity_limit_impulse"
    assert taxonomy.classify_family(6, LAGGED) == "controller_path_tolerance"
    assert taxonomy.signature(6, IMPULSE) != taxonomy.signature(6, LAGGED)
    # And two occurrences of the impulse still share one row: only the numbers differ.
    other = IMPULSE.replace("3.1000", "2.4000").replace("412", "377")
    assert taxonomy.signature(6, IMPULSE) == taxonomy.signature(6, other)


def test_a_control_failure_with_no_controller_line_is_not_promoted_to_a_named_abort():
    # Nothing was established, so it must not land in a row that claims something was.
    unreported = f"{CONTROL_FAILED}; no controller abort was reported on /rosout for this segment"
    assert taxonomy.classify_family(6, unreported) == "controller_abort_unreported"


def test_a_detail_naming_motion_and_nothing_more_specific_is_still_labelled():
    assert taxonomy.classify_family(6, "MoveIt raised during plan-and-execute") == (
        "moveit_execution"
    )
    assert taxonomy.classify_family(6, "the motion backend went away") == "motion_other"


def test_a_terse_result_detail_is_separated_by_the_fault_it_came_from():
    # Three runs of the baseline scenario ended with the result detail "motion inhibited;
    # operator required", which names no cause. The FAULT feedback a step earlier carried the
    # diagnosis. Two inhibitions from different causes must not share a row.
    detach = [
        {"state": 21, "detail": "open_gripper completed"},
        {
            "state": 29,
            "detail": (
                "attachment physically applied but not committed: simulated detach is applied "
                "but world state refused the commit until its deadline"
            ),
        },
    ]
    unrelated = [
        {"state": 8, "detail": "plan_approach completed"},
        {"state": 29, "detail": "the rail carriage left its commanded envelope"},
    ]
    inhibited = "motion inhibited; operator required"
    assert taxonomy.cause_detail(detach).startswith("attachment physically applied")
    assert taxonomy.signature(9, inhibited, taxonomy.cause_detail(detach)) != taxonomy.signature(
        9, inhibited, taxonomy.cause_detail(unrelated)
    )
    # And the family comes from the cause, not from the terse result.
    assert taxonomy.classify_family(9, taxonomy.cause_detail(detach)) == "attachment"


def test_motion_time_is_one_figure_because_the_feedback_cannot_split_it():
    # The PLAN_* state holds for the whole plan-and-execute; the paired EXECUTE_* state records
    # only that the trajectory ran. Attributing them separately gave "execution 0.0 s" on every
    # successful goal.
    trace = [
        {"state": 2, "state_name": "SELECT_PAIR", "elapsed_sim_s": 0.0},
        {"state": 5, "state_name": "PLAN_PRE_GRASP", "elapsed_sim_s": 2.0},
        {"state": 6, "state_name": "EXECUTE_PRE_GRASP", "elapsed_sim_s": 22.0},
        {"state": 9, "state_name": "CLOSE_GRIPPER", "elapsed_sim_s": 22.0},
    ]
    phases = taxonomy.phase_durations(trace, 25.0)
    assert phases["motion_sim_s"] == 20.0
    assert phases["other_sim_s"] == 5.0
    assert set(phases) == {"motion_sim_s", "other_sim_s"}


def test_a_goal_with_no_fault_state_has_no_cause_detail():
    assert taxonomy.cause_detail([{"state": 5, "detail": "plan_pre_grasp completed"}]) == ""
    assert taxonomy.signature(5, "a detail") == taxonomy.signature(5, "a detail", "")


def test_a_terminal_status_with_no_detail_is_still_a_finding():
    assert taxonomy.classify_family(255, "") == "undetailed:internal_error"


def test_furthest_state_uses_the_nominal_order_not_the_identifier():
    # OPEN_GRIPPER_FOR_APPROACH is identifier 33 but happens before CLOSE_GRIPPER, identifier 9.
    # Comparing identifiers would report the earlier state as the furthest reached.
    assert taxonomy.furthest_state([5, 6, 33]) == 33
    assert taxonomy.furthest_state([5, 6, 33, 9]) == 9
    assert taxonomy.furthest_state([]) is None
    # Off-path states are not a depth: entering FAULT says nothing about forward progress.
    assert taxonomy.furthest_state([5, 29]) == 5


def test_every_nominal_state_has_a_name():
    for state in taxonomy.NOMINAL_STATE_ORDER:
        assert not taxonomy.state_name(state).startswith("STATE_")


def test_a_half_addressed_goal_is_rejected_when_the_table_is_built():
    # select_task_pair requires an object and a lane together, so a scenario that names one is a
    # contract violation and must not reach a run.
    try:
        scenarios.Goal("bad", source_object_id="sim:stock_can_01")
    except ValueError:
        return
    raise AssertionError("a half-addressed goal must be rejected")


def test_every_scenario_names_a_stock_file_and_a_usable_goal_sequence():
    for scenario in scenarios.SCENARIOS:
        assert scenario.stock_config.endswith(".yaml")
        assert scenario.description
        if scenario.goals is not None:
            assert scenario.goals
