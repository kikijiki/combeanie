# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The lease environment must carry the isolation tag without moving cameras off shared memory."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))

from domain_isolation import TAG_VARIABLE, DomainLease  # noqa: E402


def test_lease_environment_pins_domain_partition_and_tag():
    """Every leased process must be tagged and domain-scoped."""
    environment = DomainLease(42, "ws-unit", None).environment("restocker_probe")
    assert environment["ROS_DOMAIN_ID"] == "42"
    assert environment["GZ_PARTITION"] == "restocker_probe_ws-unit-42"
    assert environment[TAG_VARIABLE] == "ws-unit-42"


def test_lease_environment_does_not_force_udp_only_transport():
    """Cameras keep shared memory; UDP-only is scoped to the clock bridge alone.

    Lease-wide FASTDDS_BUILTIN_TRANSPORTS=UDPv4 removed the measured fastrtps port-file
    collisions but pushed every bridged camera frame onto UDP loopback, and the first wave
    running that way produced jam-class controller aborts no shared-memory wave had shown.
    """
    environment = DomainLease(7, "ws-unit", None).environment()
    assert "FASTDDS_BUILTIN_TRANSPORTS" not in environment


def test_partition_base_defaults_to_restocker():
    """A lease without a CMake-provided base still gets a usable partition name."""
    environment = DomainLease(7, "ws-unit", None).environment()
    assert environment["GZ_PARTITION"] == "restocker_ws-unit-7"


if __name__ == "__main__":
    test_lease_environment_pins_domain_partition_and_tag()
    test_lease_environment_does_not_force_udp_only_transport()
    test_partition_base_defaults_to_restocker()
