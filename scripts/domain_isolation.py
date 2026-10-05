#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT

"""Lease a ROS domain for the life of one run, machine-wide, and reap that run's orphans.

Every launch test used to carry a fixed ``ROS_DOMAIN_ID`` baked into its ``CMakeLists.txt``.
That is distinct *within* a workspace and identical *across* workspaces, so two worktrees
running the same test see each other's nodes by construction. Two false diagnoses this project
paid for -- a live-shutdown test that "failed under load", and a measurement campaign that lost
twenty runs to superseded reservations -- were both this.

The fix is to stop deciding the domain at configure time and lease one at run time instead. The
lease is a ``flock`` on a file in a directory outside every worktree, so it is exclusive across
worktrees, across concurrent ``ctest`` jobs, and across successive runs. The descriptor is held
by the leasing process for the life of the run, so the kernel releases it however the run dies,
including ``kill -9``.

A lease also gives the orphan half a handle. Each run stamps its processes with
``RESTOCKER_ISOLATION_TAG=<workspace>-<domain>``. The tag is read back from
``/proc/<pid>/environ``, matched as a whole environment entry, and the matching pids are killed
by pid. Nothing here matches on a command line, so a simulator belonging to another worktree
cannot be caught by it -- ``pkill -f 'gz sim'`` has already voided three runs of someone else's
campaign on this machine, and that is exactly the mistake this avoids.

The tag is stamped from the workspace *and* the domain, and a domain is held by at most one live
run, so the only processes a tag can name are the current run's own and those of a previous run
in the same workspace that died without cleaning up. Both are ours to kill. Reaping happens
twice: once on acquiring the domain, which is what catches the orphans of a run that was killed
outright, and once when the run finishes.

The pre-run reap is a sweep of the whole workspace rather than a lookup of one domain, because a
run that dies on domain 51 whose successor leases 77 would otherwise leave its nodes running for
as long as nothing happened to lease 51 again. The sweep asks ``flock`` which of this workspace's
occupied domains nobody holds, and reaps only those; a domain whose lease is held belongs to a
live run and is left alone.

Holding the lease is what makes the pre-run reap safe, and it is a stronger test than ``ppid==1``:
a live run holds its domain, so anything wearing this tag at the moment the lease is granted is
by construction from a run that is already gone. ``ppid`` is recorded in the reap message as
evidence rather than used as a gate, because an orphaned launcher's own children keep pointing at
the orphan and would be missed by a ``ppid==1`` filter.
"""

from __future__ import annotations

import argparse
import contextlib
import fcntl
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

#: Environment variable every process in a leased run is stamped with.
TAG_VARIABLE = "RESTOCKER_ISOLATION_TAG"

#: Lock directory. Deliberately outside any worktree: the point is to coordinate between them.
DEFAULT_LEASE_DIR = "/tmp/restocker-domain-leases"

EPHEMERAL_RANGE_FILE = Path("/proc/sys/net/ipv4/ip_local_port_range")

# RTPS derives its ports from the domain: a domain's block starts at 7400 + 250 * domain. A
# domain is usable only when that block misses the ephemeral range the kernel hands out to
# everything else, or the two collide and discovery breaks in ways that look like anything but a
# port clash.
RTPS_PORT_BASE = 7400
RTPS_DOMAIN_GAIN = 250
LINUX_DEFAULT_EPHEMERAL_LOW = 32768

# Domain 0 is excluded on purpose: it is what an unconfigured shell gets, so leaving it unleased
# means an ad-hoc `ros2 topic list` never lands inside a test run.
LOW_BAND = (1, 101)

# Above the ephemeral range. 7400 + 250 * 215 = 61150, so this band is only safe when the
# kernel's ephemeral range ends below that; 232 is the highest domain whose block still fits
# under 65535. Checked at run time rather than assumed.
HIGH_BAND = (215, 232)
HIGH_BAND_FIRST_PORT = RTPS_PORT_BASE + RTPS_DOMAIN_GAIN * HIGH_BAND[0]

DEFAULT_WAIT_SECONDS = 1800.0
DEFAULT_REAP_GRACE_SECONDS = 5.0


def ephemeral_port_range() -> tuple[int, int] | None:
    """Return the kernel's ephemeral port range, or None when it cannot be read."""
    try:
        low_text, high_text = EPHEMERAL_RANGE_FILE.read_text(encoding="utf-8").split()
        return int(low_text), int(high_text)
    except (OSError, ValueError):
        return None


def _highest_safe_low_band_domain(ephemeral_low: int) -> int:
    """Return the highest domain whose port block stays below a given ephemeral floor."""
    if ephemeral_low >= LINUX_DEFAULT_EPHEMERAL_LOW:
        return LOW_BAND[1]
    highest = (ephemeral_low - 1 - RTPS_PORT_BASE) // RTPS_DOMAIN_GAIN
    return max(LOW_BAND[0] - 1, min(LOW_BAND[1], highest))


def domain_pool() -> list[int]:
    """Return every domain this machine can safely use, lowest first.

    The low band is the one the ROS 2 documentation calls always-safe on Linux. The high band is
    added only when the kernel's own ephemeral range leaves room for it, so a machine with a
    widened range silently gets the smaller pool rather than a broken domain.
    """
    override = os.environ.get("RESTOCKER_DOMAIN_POOL")
    if override:
        return _parse_pool_override(override)

    port_range = ephemeral_port_range()
    if port_range is None:
        return list(range(LOW_BAND[0], LOW_BAND[1] + 1))

    ephemeral_low, ephemeral_high = port_range
    domains = list(range(LOW_BAND[0], _highest_safe_low_band_domain(ephemeral_low) + 1))
    if ephemeral_high < HIGH_BAND_FIRST_PORT:
        domains.extend(range(HIGH_BAND[0], HIGH_BAND[1] + 1))
    return domains


def _parse_pool_override(text: str) -> list[int]:
    """Parse a ``RESTOCKER_DOMAIN_POOL`` value such as ``1-101,215-232``."""
    domains: list[int] = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            first, last = (int(value) for value in part.split("-", 1))
        else:
            first = last = int(part)
        domains.extend(range(first, last + 1))
    invalid = [domain for domain in domains if not 0 <= domain <= 232]
    if invalid:
        raise ValueError(f"RESTOCKER_DOMAIN_POOL contains invalid domain ids: {invalid}")
    if not domains:
        raise ValueError("RESTOCKER_DOMAIN_POOL is set but names no domains")
    return domains


def workspace_identity() -> str:
    """Return a short stable identifier for the workspace this script belongs to.

    Derived from the resolved path of this file, so each worktree gets its own value without
    anything having to be configured, and the same worktree gets the same value on every run.
    """
    root = Path(__file__).resolve().parents[1]
    return hashlib.sha256(str(root).encode("utf-8")).hexdigest()[:10]


def tagged_pids(tag: str) -> list[int]:
    """Return the pids whose environment carries exactly this isolation tag.

    Matched as a complete ``NAME=value`` entry of ``/proc/<pid>/environ``, never as a substring
    of a command line, so the result cannot include a process from another workspace or another
    domain.
    """
    needle = f"{TAG_VARIABLE}={tag}".encode()
    own_pid = os.getpid()
    matches: list[int] = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == own_pid:
            continue
        try:
            environ = Path("/proc", entry, "environ").read_bytes()
        except OSError:
            continue
        if needle in environ.split(b"\0"):
            matches.append(pid)
    return matches


def reap(tag: str, grace_seconds: float = DEFAULT_REAP_GRACE_SECONDS) -> list[int]:
    """Terminate every process carrying this isolation tag and return the pids killed.

    Signals go to individual pids. The tag is re-read immediately before the final ``SIGKILL``,
    so a pid recycled during the grace period is dropped rather than killed.
    """
    pids = tagged_pids(tag)
    for pid in pids:
        _signal_pid(pid, signal.SIGTERM)

    deadline = time.monotonic() + grace_seconds
    while time.monotonic() < deadline:
        if not tagged_pids(tag):
            return pids
        time.sleep(0.2)

    for pid in tagged_pids(tag):
        _signal_pid(pid, signal.SIGKILL)
    return pids


def parent_pid(pid: int) -> int | None:
    """Return a pid's parent, or None when it cannot be read."""
    try:
        status = Path("/proc", str(pid), "status").read_text(encoding="utf-8")
    except OSError:
        return None
    for line in status.splitlines():
        if line.startswith("PPid:"):
            return int(line.split()[1])
    return None


def describe_pids(pids: list[int]) -> str:
    """Render pids with their parents, so a reap message shows what was actually orphaned."""
    return ", ".join(f"{pid}(ppid={parent_pid(pid)})" for pid in pids)


def _signal_pid(pid: int, number: int) -> None:
    # Gone already, or not ours to signal. Either way there is nothing to do and nothing to
    # report: the caller's contract is "this pid is not running our run's processes afterwards".
    with contextlib.suppress(ProcessLookupError, PermissionError):
        os.kill(pid, number)


class DomainLease:
    """An exclusively held ROS domain, released when this process exits."""

    def __init__(self, domain: int, workspace: str, handle) -> None:
        self.domain = domain
        self.workspace = workspace
        self.tag = f"{workspace}-{domain}"
        self._handle = handle

    def environment(self, gz_partition_base: str | None = None) -> dict[str, str]:
        """Return the environment entries every process in this run must carry."""
        base = gz_partition_base or "restocker"
        # Fast DDS stays on its default transports here (shared memory for the megapixel camera
        # topics). Lease-wide FASTDDS_BUILTIN_TRANSPORTS=UDPv4 was tried and reverted: it removed
        # the measured cross-domain fastrtps_portNNNN init collisions (56 in one wave, on both
        # ends of a /clock hop) but forced every bridged camera frame onto UDP loopback, and the
        # first wave running that way produced jam-class controller path-tolerance aborts that no
        # earlier shared-memory wave had shown. LARGE_DATA was measured to create SHM segments
        # too, so it is not a collision-free alternative. UDP-only is scoped to the clock bridge
        # in restocker_gazebo's simulation.launch instead — the participant the collisions were
        # measured on — while cameras keep shared memory.
        return {
            "ROS_DOMAIN_ID": str(self.domain),
            "GZ_PARTITION": f"{base}_{self.tag}",
            TAG_VARIABLE: self.tag,
        }

    def release(self) -> None:
        """Release the lease. Optional, process exit releases it too."""
        if self._handle is not None:
            self._handle.close()
            self._handle = None


def acquire(seed: str, wait_seconds: float = DEFAULT_WAIT_SECONDS) -> DomainLease:
    """Lease a free domain, blocking until one is available.

    ``seed`` only chooses where in the pool the scan starts, so a given test in a given
    workspace prefers the same domain run after run while never being pinned to it. Set
    ``RESTOCKER_FORCE_DOMAIN_ID`` to reproduce a recorded run on the domain it used.
    """
    lease_dir = Path(os.environ.get("RESTOCKER_DOMAIN_LEASE_DIR", DEFAULT_LEASE_DIR))
    lease_dir.mkdir(parents=True, exist_ok=True)
    workspace = workspace_identity()

    forced = os.environ.get("RESTOCKER_FORCE_DOMAIN_ID")
    if forced:
        candidates = [int(forced)]
    else:
        pool = domain_pool()
        offset = int(hashlib.sha256(seed.encode("utf-8")).hexdigest(), 16) % len(pool)
        candidates = pool[offset:] + pool[:offset]

    deadline = time.monotonic() + wait_seconds
    while True:
        for domain in candidates:
            handle = open(lease_dir / f"domain-{domain}.lock", "a")  # noqa: SIM115
            try:
                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                handle.close()
                continue
            return DomainLease(domain, workspace, handle)
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"no free ROS domain within {wait_seconds:.0f}s "
                f"({len(candidates)} in the pool, leases in {lease_dir})"
            )
        time.sleep(1.0)


def workspace_tag_pids(workspace: str) -> dict[str, list[int]]:
    """Return every live pid of this workspace, grouped by the isolation tag it carries."""
    prefix = f"{TAG_VARIABLE}={workspace}-".encode()
    own_pid = os.getpid()
    groups: dict[str, list[int]] = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        pid = int(entry)
        if pid == own_pid:
            continue
        try:
            environ = Path("/proc", entry, "environ").read_bytes()
        except OSError:
            continue
        for item in environ.split(b"\0"):
            if item.startswith(prefix):
                tag = item.split(b"=", 1)[1].decode("utf-8", "replace")
                groups.setdefault(tag, []).append(pid)
                break
    return groups


def reap_stale(lease: DomainLease) -> dict[str, list[int]]:
    """Reap every orphan this workspace has left behind, announcing what was found.

    Two groups, and the second is why this is a sweep rather than a lookup. The first is this
    lease's own domain: we hold it, so anything wearing its tag is from a run that is gone. The
    second is every other domain this workspace has processes on whose lease nobody holds --
    which is the case a run that died on domain 51 and whose successor leased 77 would otherwise
    leave burning CPU indefinitely, and four `gz sim` processes did exactly that for half an hour
    before this existed.

    A domain whose lease is held is skipped untouched: that is a live run, and the fact that
    ``flock`` refuses is the whole proof.
    """
    lease_dir = Path(os.environ.get("RESTOCKER_DOMAIN_LEASE_DIR", DEFAULT_LEASE_DIR))
    reaped: dict[str, list[int]] = {}

    for tag, pids in sorted(workspace_tag_pids(lease.workspace).items()):
        try:
            domain = int(tag.rsplit("-", 1)[1])
        except (IndexError, ValueError):
            continue
        if domain != lease.domain and not _domain_is_free(lease_dir, domain):
            continue
        held = "held by this run" if domain == lease.domain else "leased by nobody"
        print(
            f"domain_isolation: domain {domain} ({held}) carries {len(pids)} orphan(s) of an "
            f"earlier run in this workspace; killing by pid: {describe_pids(pids)}",
            file=sys.stderr,
        )
        reap(tag)
        reaped[tag] = pids
    return reaped


def _domain_is_free(lease_dir: Path, domain: int) -> bool:
    """Report whether a domain's lease is currently unheld, without keeping it."""
    try:
        handle = open(lease_dir / f"domain-{domain}.lock", "a")  # noqa: SIM115
    except OSError:
        return False
    try:
        fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except OSError:
        return False
    finally:
        handle.close()


def run_isolated(command: list[str], seed: str) -> int:
    """Run a command holding a domain lease, reaping the run's own orphans either side."""
    lease = acquire(seed)
    overrides = lease.environment(os.environ.get("GZ_PARTITION"))
    reap_stale(lease)
    print(
        f"domain_isolation: ROS_DOMAIN_ID={lease.domain} "
        f"GZ_PARTITION={overrides['GZ_PARTITION']} {TAG_VARIABLE}={lease.tag}",
        file=sys.stderr,
    )

    environment = dict(os.environ)
    environment.update(overrides)
    # The lease descriptor is this process's alone; the child must not be able to hold the
    # domain open after this process is gone.
    process = subprocess.Popen(command, env=environment, close_fds=True)

    def forward(number, _frame):
        _signal_pid(process.pid, number)

    previous = {
        number: signal.signal(number, forward) for number in (signal.SIGTERM, signal.SIGINT)
    }
    try:
        while True:
            try:
                return process.wait()
            except KeyboardInterrupt:
                # The child is in the same process group and already has the signal; keep
                # waiting for it rather than abandoning it half-dead on a leased domain.
                continue
    finally:
        for number, handler in previous.items():
            signal.signal(number, handler)
        reap(lease.tag)
        lease.release()


def main(argv: list[str] | None = None) -> int:
    """Run a command under a leased ROS domain."""
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--seed",
        default="",
        help="chooses where in the domain pool the scan starts; any stable string will do",
    )
    parser.add_argument("--print-pool", action="store_true", help="print the pool and exit")
    parser.add_argument("command", nargs="*", help="the command to run")
    arguments = parser.parse_args(argv)

    if arguments.print_pool:
        pool = domain_pool()
        port_range = ephemeral_port_range()
        print(f"ephemeral port range: {port_range}")
        print(f"pool size: {len(pool)}")
        print(f"pool: {pool}")
        return 0

    if not arguments.command:
        parser.error("a command is required")
    return run_isolated(arguments.command, arguments.seed or " ".join(arguments.command))


if __name__ == "__main__":
    raise SystemExit(main())
