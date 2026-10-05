# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13): a result held past the deadline keeps the campaign blocked.

The tray survey is admitted but its result is withheld; the campaign's timeout cancels that exact
goal and the fake answers with a canceled terminal that carries neither non-start nor stop
evidence, which does not settle. Fresh tray evidence, a lane that needs a survey and many elapsed
backoffs must not authorize a single send, and when the configured cycle bound ends the run its
terminal status must disclose the unresolved motion instead of reporting plain completion.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

PREFIX = "/test/campaign_unresolved_bound"
Status = AutonomousRestockCampaignStatus
MAX_CYCLES = 60


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign with a small cycle bound so the run ends while unresolved."""
    return support.make_description(
        PREFIX, "autonomous_restock_campaign_bound_subject", max_cycles=MAX_CYCLES
    )


class TestUnresolvedResultBound(support.UnresolvedCampaignFixture):
    """A withheld tray result stays unresolved across backoffs, evidence and the cycle bound."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_unresolved_bound"
    # A front deficit with no valid tray candidate sends the loop to the tray survey.
    LANE_TARGET = 1

    def test_withheld_result_blocks_every_send_and_the_bound_discloses_it(self):
        world = self.world
        type(self).tray_script = ["hang"]
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            40.0,
            "motion_unresolved after the tray result was withheld past the deadline",
        )
        block_time = next(t for t, m in self.snapshot_statuses() if support.unresolved(m))
        sends_at_block = self.sends()
        self.assertEqual(
            len([e for e in sends_at_block if e[1] == "tray"]), 1, "exactly the one tray goal"
        )

        # Fresh observations: a valid tray candidate and a lane that needs a survey. Both would
        # authorize a send in the old retry loop.
        with world.lock:
            world.lanes["lane_02"]["invalid"] = True
            world.add_back("sim:fresh_after_block", 1, "SIM-CAN-STD")
        # The fake cancels by exact UUID; its canceled terminal carries no evidence.
        self.spin_until(
            lambda: any(e[1] == "tray_cancel" for e in self.snapshot_events()),
            15.0,
            "the withheld tray goal to be canceled by exact UUID",
        )
        goal_hex = next(e[2] for e in self.snapshot_events() if e[1] == "tray_cancel")

        # The bounded run ends: the terminal status must say motion is unresolved.
        self.spin_until(
            lambda: any("cycle bound" in m.detail for _, m in self.snapshot_statuses()),
            60.0,
            "the configured cycle bound to end the run",
        )
        statuses = self.snapshot_statuses()
        final = statuses[-1][1]
        self.assertTrue(
            support.unresolved(final), "the terminal status discloses unresolved motion"
        )
        self.assertEqual(final.phase, Status.PHASE_BLOCKED)
        self.assertNotEqual(final.phase, Status.PHASE_COMPLETE)
        self.assertIn("motion_unresolved", final.detail)
        self.assertTrue(
            any(f"goal={goal_hex}" in entry for entry in support.attempts(final)),
            f"the retained identity carries the exact tray goal: {support.attempts(final)}",
        )
        self.assertFalse(
            any(m.phase == Status.PHASE_COMPLETE for _, m in statuses),
            "no PHASE_COMPLETE while motion is unresolved",
        )
        self.assertEqual(
            [e for e in self.sends() if e[0] > block_time],
            [],
            "zero new goal sends across the backoffs, fresh observations and the cycle bound",
        )
        self.assertGreaterEqual(
            sum(1 for t, m in statuses if t > block_time and support.unresolved(m)),
            2,
            "the block is reported repeatedly across backoffs",
        )
