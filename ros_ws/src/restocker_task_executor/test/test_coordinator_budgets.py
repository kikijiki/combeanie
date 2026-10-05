# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The shipped coordinator budgets must keep the loaded-suite planning floor and retry room."""

from pathlib import Path
import re

import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config" / "restock_action_coordinator.yaml"
# Card 049 review note 4: the launch argument's parameter override wins at runtime, so the
# composition root carries the same budget this file reads from the yaml.
LAUNCH = (
    Path(__file__).resolve().parents[4]
    / "ros_ws"
    / "src"
    / "restocker_bringup"
    / "launch"
    / "baseline.launch.py"
)


def _parameters() -> dict:
    """Return the shipped `ros__parameters` map of the coordinator node."""
    document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    return document["restock_action_coordinator"]["ros__parameters"]


def test_planning_budget_gives_each_loaded_continuation_slice_room():
    """
    The pre-insert's continuation-endpoint slices must clear measured loaded solve times.

    At 20000 ms each slice got 5 s, every slice timed out under full-suite parallel load, and the
    segment reported MoveIt FAILURE 99999 for a pose a retry planned immediately afterwards;
    measured loaded solves reach 4.9 s. Solo plans return in well under a second, so the floor
    only bounds failures — it never sets green-path latency. The floor's division across the
    candidates is free (Milestone 10 §6, Card 074: first-pass share for all, half the floor to
    the endpoint that timed out); the floor itself is not.
    """
    parameters = _parameters()
    assert parameters["task.planning_timeout_ms"] >= 60000


def test_whole_task_budget_still_fits_a_full_retry_of_the_longest_segment():
    """
    Two worst-case plan+execute attempts of one segment stay inside the immutable task bound.

    `task.total_timeout_ms` is the fail-closed deadline for the whole goal; raising the planning
    budget must not let the per-command budget (planning + execution) exceed what a retry of the
    longest segment needs inside it.
    """
    parameters = _parameters()
    planning = parameters["task.planning_timeout_ms"]
    execution = parameters["task.execution_timeout_ms"]
    total = parameters["task.total_timeout_ms"]
    assert 2 * (planning + execution) <= total


def test_whole_task_budget_covers_the_shipped_acceptance_and_demo_compositions():
    """
    The bound must clear the floor of the compositions it is enforced in (Card 044, measured).

    `test_sensor_driven_acceptance_runtime`, `test_dense_sensor_driven_runtime`,
    `test_autonomous_restock_demo_runtime` and `scripts/demo.bash` pin
    `task_execution_timeout_ms:=180000` — the value the yaml itself ships since Card 049 — so
    inside those runs the documented `2 * (planning + execution) <= task.total_timeout_ms` floor
    is 2 * (60000 + 180000) = 480000 ms and the bound now sits exactly on its own composition
    floor. Card 044 first fixed the opposite failure: the old 240000 ms bound sat *below* the
    then-floor of its 120000-pinned compositions (2 * (60000 + 120000) = 360000 ms) and cut
    goals off at 240 s that were still mid-retreat (SC-003 run 7: retreat completed +247.9 s;
    Card 020 run 4: +266.1 s; Card 023: goals cut at 241-243 s with the pre-insert alone at
    146-158 s against 48 s planned). The shipped 480000 ms keeps clearing that floor and the
    measured natural-completion tail while staying a hard fail-closed bound on a stuck goal.
    """
    parameters = _parameters()
    planning = parameters["task.planning_timeout_ms"]
    total = parameters["task.total_timeout_ms"]
    assert 2 * (planning + 180000) <= total
    assert total >= 480000


def test_segment_budget_covers_the_measured_busy_form_receipts():
    """
    The per-segment budget must clear the measured busy-form class (Card 049, measured).

    Every `motion segment exceeded its command deadline` receipt expired *exactly* on the armed
    instant while the port was still completing controller goals — busy and progressing, so the
    budget (not the verdict, not a stall) was the defect:

    - SC-003 run 3, pre-insert op 59 (`evidence/cmbdef4-010/sc003-just-test-a3.log`): armed at
      the default 60000 + 60000 = 120000 ms, expired at controller goal 11/13 after 50.6 s of
      planning → needed ≈146 s;
    - SC-003 retry 2, op 158 (`…/sc003-just-test-ad0fad2-r2.log`): armed at the composition pin
      60000 + 120000 = 180000 ms, expired at goal 12/13 after 41.6 s → needed ≈199 s;
    - Card 066 dense dev 2, op 60
      (`evidence/cmbfeed-066/dev2-079708f-launch-ctest.log`): armed at 180000 ms, expired at
      goal 9/10 after 27.9 s (two OMPL 15 s slice timeouts) → needed ≈215 s.

    The largest measured need sets the floor; `test_whole_task_budget_still_fits_a_full_retry_of_
    the_longest_segment` keeps the ceiling. Shipped: 240000 ms, the invariant cap at
    `task.total_timeout_ms` = 480000 ms.
    """
    parameters = _parameters()
    planning = parameters["task.planning_timeout_ms"]
    execution = parameters["task.execution_timeout_ms"]
    assert planning + execution >= 215000


def test_launch_default_matches_the_shipped_execution_budget():
    """
    Card 049 review note 4: `baseline.launch.py`'s default must equal the yaml's budget.

    The launch argument is passed as a parameter override, so it — not the yaml — is the
    runtime budget for every composition that does not pin the argument itself. A drift back to
    the pre-Card-049 60000 ms would leave every assertion in this file green (they read the
    yaml) while the shipped compositions ran on 60000 ms and the retry invariant
    `2 * (planning + execution) <= task.total_timeout_ms` silently became 420000 ms of the
    480000 ms bound. `test_demo_wiring` pins `demo.bash`; this pins the launch default.
    """
    launch = LAUNCH.read_text(encoding="utf-8")
    declared = re.search(
        r'DeclareLaunchArgument\(\s*"task_execution_timeout_ms",\s*'
        r'default_value="(\d+)"',
        launch,
    )
    assert declared is not None, (
        "baseline.launch.py must declare task_execution_timeout_ms with a plain numeric default"
    )
    parameters = _parameters()
    assert int(declared.group(1)) == parameters["task.execution_timeout_ms"]


if __name__ == "__main__":
    test_planning_budget_gives_each_loaded_continuation_slice_room()
    test_whole_task_budget_still_fits_a_full_retry_of_the_longest_segment()
    test_whole_task_budget_covers_the_shipped_acceptance_and_demo_compositions()
    test_segment_budget_covers_the_measured_busy_form_receipts()
    test_launch_default_matches_the_shipped_execution_budget()
