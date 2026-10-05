# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13 scenario 2): a result code UNKNOWN is no terminal.

The tray fake answers every result request the way rclcpp_action's peer does for a goal it has no
record of: STATUS_UNKNOWN. That says nothing about the goal, which may still be running. The
campaign must cancel by the exact goal id, retain the attempt as unresolved and send nothing more,
whatever evidence is refreshed.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

Status = AutonomousRestockCampaignStatus
PREFIX = "/test/campaign_unresolved_unknown"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign; the tray fake answers UNKNOWN to its result requests."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_unknown_subject")


class TestUnknownResultCode(support.UnresolvedCampaignFixture):
    """A result code UNKNOWN cancels by exact id and stays unresolved."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_unresolved_unknown"
    LANE_TARGET = 1

    def test_unknown_result_code_is_canceled_by_id_and_retained(self):
        # The goal stays active (the hang) so the exact-id cancel reaches the server.
        type(self).tray_script = ["hang"]
        type(self).tray_server.answer_unknown = True
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            40.0,
            "motion_unresolved after the UNKNOWN result code",
        )
        block_time = next(t for t, m in self.snapshot_statuses() if support.unresolved(m))
        marked = next(m for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.assertEqual(marked.phase, Status.PHASE_BLOCKED)
        self.spin_until(
            lambda: any(e[1] == "tray_cancel_request" for e in self.snapshot_events()),
            10.0,
            "a cancel by the exact goal id after the UNKNOWN code",
        )
        goal_hex = next(e[2] for e in self.snapshot_events() if e[1] == "tray_cancel_request")
        self.assertTrue(
            any(
                entry.startswith("tray#") and f"goal={goal_hex}" in entry
                for _, m in self.snapshot_statuses()
                for entry in support.attempts(m)
            ),
            "the retained record carries the exact canceled goal id",
        )
        with self.world.lock:
            self.world.lanes["lane_02"]["invalid"] = True
            self.world.add_back("sim:fresh_after_unknown", 1, "SIM-CAN-STD")
        before = sum(1 for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.spin_until(
            lambda: (
                sum(1 for _, m in self.snapshot_statuses() if support.unresolved(m)) >= before + 3
            ),
            15.0,
            "further blocked backoffs",
        )
        self.assertEqual(
            [e for e in self.sends() if e[0] > block_time],
            [],
            "no goal is sent after an UNKNOWN result code, whatever the world offers",
        )
        self.assertEqual(len([e for e in self.sends() if e[1] == "tray"]), 1)
