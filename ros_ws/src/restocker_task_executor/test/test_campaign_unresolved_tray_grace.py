# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 review B1 (CMB-SPEC-13 DECIDED 5): the send-grace-expiry tray terminal.

The tray survey's 5 s grace for a slow child admission expires; it returns UNAVAILABLE delivered
with code SUCCEEDED and the untouched default motion_definitely_not_started=true while the child
may still be admitted. A not-SUCCEEDED-code rule alone would miss this shape; the campaign must
keep the attempt unresolved and send nothing.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

Status = AutonomousRestockCampaignStatus

PREFIX = "/test/campaign_unresolved_tray_grace"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign; the tray fake delegates to a viewpoint child with a slow admission."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_tray_grace_subject")


class TestTrayGraceExpiry(support.UnresolvedCampaignFixture):
    """A tray terminal returned over a withheld child leg never settles on the default flag."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_unresolved_tray_grace"
    LANE_TARGET = 1

    def test_tray_terminal_with_a_withheld_child_stays_unresolved(self):
        type(self).tray_script = ["child_withheld_unavailable"]
        # The child's admission reply arrives long after the campaign's own timeout.
        type(self).retreat_admission_hang_sec = 3.0
        self.spin_until(
            lambda: any(e[1] == "tray_terminal" for e in self.snapshot_events()),
            40.0,
            "the tray parent to terminate after the campaign canceled it",
        )
        events = self.snapshot_events()
        terminal_time = next(e[0] for e in events if e[1] == "tray_terminal")
        # The tray terminal is delivered with the untouched pre-motion default, so the campaign
        # has no evidence about the child. It must hold, whatever the world offers.
        with self.world.lock:
            self.world.lanes["lane_02"]["invalid"] = True
            self.world.add_back("sim:fresh_after_parent_terminal", 1, "SIM-CAN-STD")
        self.spin_until(
            lambda: any(e[1] == "retreat_goal" for e in self.snapshot_events()),
            15.0,
            "the withheld child leg to be admitted after the parent already terminated",
        )
        child_landed = next(e[0] for e in self.snapshot_events() if e[1] == "retreat_goal")
        self.assertGreater(
            child_landed, terminal_time, "the child lands after the parent terminal (the race)"
        )
        blocked_after_landing = sum(
            1 for t, m in self.snapshot_statuses() if t > child_landed and support.unresolved(m)
        )
        self.spin_until(
            lambda: (
                sum(
                    1
                    for t, m in self.snapshot_statuses()
                    if t > child_landed and support.unresolved(m)
                )
                >= blocked_after_landing + 3
            ),
            15.0,
            "further blocked backoffs after the child landed",
        )
        marked = [m for _, m in self.snapshot_statuses() if support.unresolved(m)]
        self.assertTrue(marked, "the campaign reports unresolved motion")
        self.assertTrue(
            any(entry.startswith("tray#") for entry in support.attempts(marked[-1])),
            f"the tray attempt is retained: {support.attempts(marked[-1])}",
        )
        late_sends = [e for e in self.sends() if e[0] > terminal_time and e[1] != "retreat_goal"]
        self.assertEqual(
            late_sends,
            [],
            "no goal is sent after the tray terminal: its child leg may still be admitted",
        )
        self.assertEqual(len([e for e in self.sends() if e[1] == "tray"]), 1)
        self.assertTrue(
            all(
                m.phase not in {Status.PHASE_FRONT_FULL, Status.PHASE_COMPLETE}
                for t, m in self.snapshot_statuses()
                if t > terminal_time
            )
        )
