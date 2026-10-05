# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Source pin for Card 086 stage 1b review B1 (no simulator, no MoveIt).

The MoveIt port cannot be driven without MoveIt, so the one behavior the driver's proof depends on
(a completion says whether a trajectory was ever handed to the backend) is pinned on the source:
the worker clears the bit before running a goal, execute() is preceded by setting it, and the
completion carries it out. Re-inlining or dropping any of these must fail here.
"""

import os
from pathlib import Path
import re


def verify_the_port_marks_every_execute_call_and_publishes_the_bit():
    root = Path(os.environ["RESTOCKER_PACKAGE_ROOT"])
    source = (root / "src" / "moveit_motion_port.cpp").read_text()

    executes = [m.start() for m in re.finditer(r"interface->execute\(", source)]
    assert len(executes) == 1, "exactly one execute() call site is expected"
    execute_at = executes[0]
    start = max(0, execute_at - 200)
    before_execute = source[start:execute_at]
    assert "execute_called_.store(true" in before_execute, "execute() must be preceded by the bit"

    first = source.index("execute_called_.store(false")
    worker = source[first:]
    assert "completion.submitted_to_backend = execute_called_.load" in worker[:300]
    assert worker.index("execute_goal(pending)") < worker.index("completion.submitted_to_backend")

    # No branch may sit between the store and the call, so the bit can never be set without the
    # call (a dead branch or lambda would make it lie) or be skipped by an early return.
    store_at = source.index("execute_called_.store(true")
    between = source[store_at:execute_at]
    for forbidden in ("return", "if (", "if(", "break", "continue", "[&]", "[this"):
        assert forbidden not in between, f"{forbidden!r} between the bit and execute()"

    # Every completion the port can produce is built inside execute_goal/plan_free_space, which
    # run only under run_worker's reset/copy; a new producer elsewhere would bypass the bit.
    allowed = ("execute_goal", "plan_free_space")
    functions = []
    for match in re.finditer(r"^[A-Za-z][^\n]*MoveItMotionPort::(\w+)\(", source, re.MULTILINE):
        end = source.index("\n}\n", match.start())
        functions.append((match.group(1), match.start(), end))
    for hit in re.finditer(r"MotionCompletion\{|MotionCompletion\(", source):
        owners = [name for name, begin, end in functions if begin <= hit.start() < end]
        assert owners and owners[0] in allowed, f"MotionCompletion built in {owners}"
    for call in re.finditer(r"plan_free_space\(", source):
        owners = [name for name, begin, end in functions if begin <= call.start() < end]
        assert owners[0] in ("execute_goal", "plan_free_space"), "plan_free_space called elsewhere"

    # The proof consumers must read the combined predicate, never the bare outcome class.
    driver = (root / "src" / "restock_coordinator_driver.cpp").read_text()
    assert "motion_completion_definitely_not_started(completion)" in driver
    assert "motion_definitely_not_started(completion.outcome)" not in driver
    survey = (root / "src" / "viewpoint_survey.cpp").read_text()
    assert "completion.submitted_to_backend && survey_definitely_not_started(outcome)" in survey


if __name__ == "__main__":
    verify_the_port_marks_every_execute_call_and_publishes_the_bit()
