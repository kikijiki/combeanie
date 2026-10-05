# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for observable coordinator startup authority."""

from restocker_interfaces.msg import RestockCoordinatorStatus


def test_startup_states_are_stable_and_closed() -> None:
    message = RestockCoordinatorStatus()

    assert {
        message.STARTUP_WAITING_FOR_AUTHORITY,
        message.STARTUP_PROBE_PENDING,
        message.STARTUP_READY,
        message.STARTUP_ORPHANED_RESERVATION,
        message.STARTUP_FAULTED,
    } == {0, 1, 2, 3, 4}
    assert message.startup_state == message.STARTUP_WAITING_FOR_AUTHORITY


def test_default_status_grants_no_authority_or_capability() -> None:
    message = RestockCoordinatorStatus()

    assert message.attempt_generation == 0
    assert message.admission_ready is False
    assert message.inhibited is False
    assert message.has_orphaned_reservation is False
    assert message.orphaned_reservation.reservation_id == 0
    assert "token" not in message.orphaned_reservation.get_fields_and_field_types()
