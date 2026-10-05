# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13 S3): a delivered terminal without evidence stays unresolved.

The tray survey answers UNAVAILABLE with neither motion_definitely_not_started nor
execution_reached_terminal_stop. The terminal is delivered, so nothing more can arrive for that
goal and nothing in the world can prove the arm stopped: the campaign sends no further goal,
whatever evidence is refreshed and however many backoffs elapse, and a quiet observation never
clears the record. Only a restart plus operator acknowledgment (a different process) does.
"""

import time

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

PREFIX = "/test/campaign_unresolved_delivered"
Status = AutonomousRestockCampaignStatus


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign; the fakes script one flagless tray terminal."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_delivered_subject")


class TestDeliveredWithoutEvidence(support.UnresolvedCampaignFixture):
    """A flagless delivered terminal is retained, not retried."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_unresolved_delivered"
    LANE_TARGET = 1

    def test_flagless_unavailable_terminal_blocks_every_later_send(self):
        type(self).tray_script = ["unavailable"]
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            40.0,
            "motion_unresolved after the flagless tray terminal",
        )
        block_time = next(t for t, m in self.snapshot_statuses() if support.unresolved(m))
        marked = next(m for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.assertEqual(marked.phase, Status.PHASE_BLOCKED)
        self.assertTrue(
            any(
                entry.startswith("tray#") and "goal=" in entry
                for entry in support.attempts(marked)
            ),
            f"the record carries the exact tray goal: {support.attempts(marked)}",
        )
        self.assertEqual(len([e for e in self.sends() if e[1] == "tray"]), 1)

        # Fresh observations that the old retry loop would have acted on.
        with self.world.lock:
            self.world.lanes["lane_02"]["invalid"] = True
            self.world.add_back("sim:fresh_after_block", 1, "SIM-CAN-STD")
        blocked_before = sum(1 for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.spin_until(
            lambda: (
                sum(1 for _, m in self.snapshot_statuses() if support.unresolved(m))
                >= blocked_before + 3
            ),
            20.0,
            "three further blocked backoffs",
        )
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self._publish_ready()
            time.sleep(0.05)
        self.assertEqual(
            [e for e in self.sends() if e[0] > block_time],
            [],
            "no goal is sent after a delivered terminal that proved neither non-start nor stop",
        )
        self.assertTrue(
            all(
                m.phase not in {Status.PHASE_FRONT_FULL, Status.PHASE_COMPLETE}
                for t, m in self.snapshot_statuses()
                if t > block_time
            )
        )
        self.assertTrue(
            all(support.unresolved(m) for t, m in self.snapshot_statuses() if t > block_time)
        )
