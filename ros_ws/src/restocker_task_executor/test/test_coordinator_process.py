# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Black-box signal and exit-code tests for the coordinator executable."""

from __future__ import annotations

import os
from pathlib import Path
import selectors
import signal
import subprocess
import time

import pytest


# Read at use, not import: launch_testing's pytest_pycollect_makemodule imports every
# sibling module under this directory while collecting any one of them, and those
# imports must not require THIS test's ament ENV (Card 075).
def _env_path(name: str) -> Path:
    value = os.environ.get(name)
    if not value:
        raise AssertionError(f"{name} is not set; this test is only runnable under ctest")
    return Path(value)


def _start_coordinator(test_name: str) -> tuple[subprocess.Popen[str], str]:
    command = [
        str(_env_path("RESTOCKER_COORDINATOR_EXECUTABLE")),
        "--ros-args",
        "-r",
        f"__node:={test_name}",
        "-p",
        f"product_catalog_path:={_env_path('RESTOCKER_PRODUCT_CATALOG')}",
        "-p",
        f"workcell_geometry_path:={_env_path('RESTOCKER_WORKCELL_GEOMETRY')}",
        "-p",
        f"gripper_geometry_path:={_env_path('RESTOCKER_GRIPPER_GEOMETRY')}",
    ]
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    assert process.stdout is not None
    selector = selectors.DefaultSelector()
    selector.register(process.stdout, selectors.EVENT_READ)
    output = ""
    deadline = time.monotonic() + 5.0
    try:
        while "pre-motion restock action coordinator available" not in output:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise AssertionError(f"coordinator readiness timed out:\n{output}")
            events = selector.select(remaining)
            if not events:
                continue
            line = process.stdout.readline()
            if not line:
                raise AssertionError(
                    f"coordinator exited before readiness with {process.poll()}:\n{output}"
                )
            output += line
    except BaseException:
        process.kill()
        process.wait(timeout=5.0)
        raise
    finally:
        selector.close()
    return process, output


def _finish(process: subprocess.Popen[str], output: str) -> tuple[int, str]:
    try:
        remainder, _ = process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        remainder, _ = process.communicate(timeout=5.0)
        pytest.fail(f"coordinator did not exit after signal:\n{output}{remainder}")
    return process.returncode, output + remainder


@pytest.mark.parametrize("termination_signal", [signal.SIGINT, signal.SIGTERM])
def test_first_signal_drains_cleanly(termination_signal: signal.Signals) -> None:
    process, output = _start_coordinator(f"coordinator_clean_{termination_signal.name.lower()}")
    os.kill(process.pid, termination_signal)
    returncode, output = _finish(process, output)

    assert returncode == 0, output
    assert "coordinator drain completed cleanly" in output


def test_second_pending_signal_forces_stable_exit() -> None:
    process, output = _start_coordinator("coordinator_forced_exit")
    os.kill(process.pid, signal.SIGINT)
    os.kill(process.pid, signal.SIGTERM)
    returncode, output = _finish(process, output)

    assert returncode == 3, output
    assert "second termination signal forced fail-closed exit" in output
