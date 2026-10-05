# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Every ROS test must reach the domain-lease runner, and the runner must reap before it lets go.

Two ways a test could end up on an unleased domain without anyone noticing:

- the CMake shadow never applies, because a package includes the isolation module before the
  find_package() that defines the test command, so the upstream command runs unwrapped;
- the runner is terminated (a manager, a ``timeout`` wrapper), exits without its ``finally``,
  and releases the lease while the test process is still running on that domain.
"""

import contextlib
import fcntl
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time

REPO = Path(__file__).resolve().parents[1]
ISOLATION_CMAKE = REPO / "cmake" / "restocker_test_isolation.cmake"
RUNNER = REPO / "scripts" / "run_isolated_test.py"

# Stand-ins for the upstream commands: each prints its arguments so the test can see whether the
# lease runner was forwarded.
FAKE_COMMANDS = {
    "add_launch_test": (
        'function(add_launch_test)\n  message(STATUS "launch:${ARGN}")\nendfunction()\n'
    ),
    "ament_add_gtest": 'macro(ament_add_gtest)\n  message(STATUS "gtest:${ARGN}")\nendmacro()\n',
    "ament_add_pytest_test": (
        'function(ament_add_pytest_test)\n  message(STATUS "pytest:${ARGN}")\nendfunction()\n'
    ),
}


def _configure(defined_before: list[str], defined_after: list[str]) -> subprocess.CompletedProcess:
    """Configure a scratch project that defines some test commands before the include."""
    cmake = shutil.which("cmake")
    assert cmake, "cmake is required to check the isolation wiring"
    with tempfile.TemporaryDirectory() as scratch:
        source = Path(scratch, "src")
        source.mkdir()
        body = ["cmake_minimum_required(VERSION 3.22)", "project(isolation_probe NONE)"]
        body += [FAKE_COMMANDS[name] for name in defined_before]
        body.append(f'include("{ISOLATION_CMAKE}")')
        body += [FAKE_COMMANDS[name] for name in defined_after]
        for name in defined_before + defined_after:
            body.append(f"{name}(probe_{name} probe_path)")
        (source / "CMakeLists.txt").write_text("\n".join(body) + "\n", encoding="utf-8")
        return subprocess.run(
            [cmake, "-S", str(source), "-B", str(Path(scratch, "build"))],
            capture_output=True,
            text=True,
            check=False,
        )


def test_every_shadowed_command_forwards_the_lease_runner():
    """In the documented order, launch tests, gtests and pytests all get the runner."""
    result = _configure(list(FAKE_COMMANDS), [])
    assert result.returncode == 0, result.stdout + result.stderr
    for kind in ("launch", "gtest", "pytest"):
        lines = [line for line in result.stdout.splitlines() if f"-- {kind}:" in line]
        assert len(lines) == 1, (kind, result.stdout)
        arguments = lines[0].split(";")
        assert "RUNNER" in arguments, (kind, lines[0])
        forwarded = Path(arguments[arguments.index("RUNNER") + 1]).resolve()
        assert forwarded == RUNNER, (kind, lines[0])


def test_a_test_command_defined_after_the_include_fails_configure():
    """The wrong include order is a configure error, not a silently unleased test."""
    for late in ("ament_add_gtest", "ament_add_pytest_test"):
        result = _configure(["add_launch_test"], [late])
        assert result.returncode != 0, (late, result.stdout)
        message = result.stdout + result.stderr
        assert f"{late} is not routed through the ROS domain lease" in message, (late, message)


def _lease_is_free(lease_dir: Path, domain: int) -> bool:
    with open(lease_dir / f"domain-{domain}.lock", "a") as handle:
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            return False
        fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        return True


def _alive(pid: int) -> bool:
    try:
        state = Path("/proc", str(pid), "status").read_text(encoding="utf-8")
    except OSError:
        return False
    return "\nState:\tZ" not in state


def test_sigterm_reaps_the_test_before_the_lease_is_released():
    """A terminated runner must not leave its test running on a domain nobody holds."""
    domain = 231
    with tempfile.TemporaryDirectory() as scratch:
        # The runner derives its workspace identity from its own location and, on acquiring a
        # lease, reaps this workspace's processes on any domain whose lease is not held. Against
        # a scratch lease directory, the real checkout's running tests would all look unheld and
        # be killed. A copy of the runner in a scratch tree is its own workspace and can only
        # reap what it started.
        scripts = Path(scratch, "workspace", "scripts")
        scripts.mkdir(parents=True)
        for name in ("run_isolated_test.py", "domain_isolation.py"):
            shutil.copy2(REPO / "scripts" / name, scripts / name)
        lease_dir = Path(scratch, "leases")
        pid_file = Path(scratch, "child.pid")
        environment = dict(
            os.environ,
            RESTOCKER_DOMAIN_LEASE_DIR=str(lease_dir),
            RESTOCKER_DOMAIN_POOL=str(domain),
        )
        runner = subprocess.Popen(
            [
                sys.executable,
                "-u",
                str(scripts / "run_isolated_test.py"),
                str(Path(scratch, "probe.xunit.xml")),
                "--package-name",
                "isolation_probe",
                "--command",
                "sh",
                "-c",
                f'echo $$ > "{pid_file}"; exec sleep 60',
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        child = None
        try:
            deadline = time.monotonic() + 20.0
            while child is None and time.monotonic() < deadline:
                if pid_file.exists() and pid_file.read_text(encoding="utf-8").strip():
                    child = int(pid_file.read_text(encoding="utf-8"))
                else:
                    time.sleep(0.05)
            assert child is not None, "the runner never started its test command"
            assert not _lease_is_free(lease_dir, domain), "the runner did not hold the lease"

            runner.send_signal(signal.SIGTERM)
            runner.wait(timeout=20.0)
            # At the instant the lease is free, the test must already be gone.
            assert _lease_is_free(lease_dir, domain)
            assert not _alive(child), "the test outlived the runner on an unleased domain"
        finally:
            if runner.poll() is None:
                runner.kill()
                runner.wait()
            if child is not None and _alive(child):
                with contextlib.suppress(ProcessLookupError):
                    os.kill(child, signal.SIGKILL)


if __name__ == "__main__":
    test_every_shadowed_command_forwards_the_lease_runner()
    test_a_test_command_defined_after_the_include_fails_configure()
    test_sigterm_reaps_the_test_before_the_lease_is_released()
