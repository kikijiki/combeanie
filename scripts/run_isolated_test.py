#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Ament test runner that leases a ROS domain for the test it is about to run.

``ament_add_test`` accepts a ``RUNNER`` argument -- the script it invokes as
``python -u <runner> <result_file> --package-name P [--env ...] --command <cmd...>`` -- and
``add_launch_test`` forwards it. This is that script. It leases a domain, rewrites the ``--env``
group the runner would have applied anyway, and then hands the whole thing to the stock runner,
so nothing about result files, timeouts or output capture changes.

Rewriting ``--env`` rather than setting the process environment is deliberate: the stock runner
applies ``--env`` *after* anything this process could set, so an entry set here would be
overwritten by a stale one from CMake. It also means the leased domain is printed by the stock
runner's own "extra environment variables" block, into the test's output file, which is what
makes a failure traceable to the domain it ran on.
"""

from __future__ import annotations

from pathlib import Path
import signal
import sys

# The stock runner's own option names. Needed to find where the ``--env`` value run ends.
RUNNER_OPTIONS = frozenset(
    {
        "--package-name",
        "--command",
        "--env",
        "--append-env",
        "--output-file",
        "--generate-result-on-success",
        "--skip-test",
        "--skip-return-code",
    }
)


def _isolation():
    """Import the isolation module that sits beside this script in the same worktree."""
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    import domain_isolation

    return domain_isolation


def _env_span(argv: list[str]) -> tuple[int, int] | None:
    """Return the half-open index range of the values following ``--env``, if it is present."""
    try:
        start = argv.index("--env") + 1
    except ValueError:
        return None
    end = start
    while end < len(argv) and argv[end] not in RUNNER_OPTIONS:
        end += 1
    return start, end


def _apply_overrides(argv: list[str], overrides: dict[str, str]) -> list[str]:
    """Return argv with the isolation environment merged into its ``--env`` group."""
    added = [f"{name}={value}" for name, value in sorted(overrides.items())]
    span = _env_span(argv)
    if span is None:
        insert_at = argv.index("--command")
        return argv[:insert_at] + ["--env"] + added + argv[insert_at:]

    start, end = span
    # Replaced in place rather than removed and re-appended: the stock runner reattaches a value
    # CMake split on semicolons to whichever key preceded it, so entry order matters.
    remaining = dict(overrides)
    merged: list[str] = []
    for entry in argv[start:end]:
        name = entry.split("=", 1)[0]
        if "=" in entry and name in remaining:
            merged.append(f"{name}={remaining.pop(name)}")
        else:
            merged.append(entry)
    merged.extend(f"{name}={value}" for name, value in sorted(remaining.items()))
    return argv[:start] + merged + argv[end:]


def _existing_partition(argv: list[str]) -> str | None:
    """Return the ``GZ_PARTITION`` a CMakeLists asked for, so its name survives as a prefix."""
    span = _env_span(argv)
    if span is None:
        return None
    for entry in argv[span[0] : span[1]]:
        if entry.startswith("GZ_PARTITION="):
            return entry.split("=", 1)[1]
    return None


def main(argv: list[str]) -> int:
    """Lease a domain, then run the stock ament test runner under it."""
    isolation = _isolation()
    import ament_cmake_test

    # SIGTERM's default action would end this process without running the ``finally`` below,
    # releasing the lease while the test keeps running on the domain. Exiting through
    # SystemExit instead reaps the test first. SIGKILL cannot be handled; the next run in this
    # workspace reaps what it leaves behind.
    signal.signal(signal.SIGTERM, lambda _number, _frame: sys.exit(128 + signal.SIGTERM))

    # The result file is the first positional argument; its stem is the test name, which makes
    # the pool scan start in the same place for the same test in the same workspace.
    seed = f"{isolation.workspace_identity()}:{Path(argv[0]).name}"
    lease = isolation.acquire(seed)
    isolation.reap_stale(lease)
    overrides = lease.environment(_existing_partition(argv))
    print(
        f"run_isolated_test: {Path(argv[0]).stem} leased ROS_DOMAIN_ID={lease.domain} "
        f"(tag {lease.tag})",
        flush=True,
    )
    try:
        return ament_cmake_test.main(_apply_overrides(argv, overrides))
    finally:
        isolation.reap(lease.tag)
        lease.release()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
