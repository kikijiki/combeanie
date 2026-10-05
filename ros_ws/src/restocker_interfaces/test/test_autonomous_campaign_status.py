# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Static contracts for the autonomous campaign's status message."""

from restocker_interfaces.msg import AutonomousRestockCampaignStatus


def test_phases_keep_their_shipped_values() -> None:
    status = AutonomousRestockCampaignStatus()
    assert status.PHASE_WAITING_FOR_COORDINATOR == 0
    assert status.PHASE_SURVEYING_FRONT == 1
    assert status.PHASE_SURVEYING_BACK == 2
    assert status.PHASE_MEASURED == 3
    assert status.PHASE_RESTOCKING == 4
    assert status.PHASE_FRONT_FULL == 5
    assert status.PHASE_STOCK_EXHAUSTED == 6
    assert status.PHASE_BLOCKED == 7
    assert status.PHASE_COMPLETE == 8
    assert status.PHASE_RECOVERING == 9


def test_mode_identifiers_are_the_five_spec_modes() -> None:
    status = AutonomousRestockCampaignStatus()
    assert status.MODE_IDLE == 0
    assert status.MODE_SURVEY_SHELF == 1
    assert status.MODE_SURVEY_TRAY == 2
    assert status.MODE_CONFIRM == 3
    assert status.MODE_TRANSFER == 4
    modes = [
        status.MODE_IDLE,
        status.MODE_SURVEY_SHELF,
        status.MODE_SURVEY_TRAY,
        status.MODE_CONFIRM,
        status.MODE_TRANSFER,
    ]
    assert len(set(modes)) == len(modes)


def test_defaults_carry_zero_mode_and_separated_arm_times() -> None:
    status = AutonomousRestockCampaignStatus()
    assert status.mode == status.MODE_IDLE
    assert status.survey_arm_time_sec == 0.0
    assert status.transfer_arm_time_sec == 0.0
    assert status.phase == status.PHASE_WAITING_FOR_COORDINATOR


def test_unresolved_motion_marker_defaults_clear() -> None:
    """Card 086 stage 1: a default status never claims unresolved motion or a pending restart."""
    status = AutonomousRestockCampaignStatus()
    assert status.motion_unresolved is False
    assert status.restart_recovery_pending is False
    assert list(status.unresolved_attempts) == []
