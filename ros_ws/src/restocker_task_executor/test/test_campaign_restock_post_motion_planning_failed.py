# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1b (CMB-SPEC-13): an unproven PLANNING_FAILED stays unresolved.

The whole-task deadline reports STATUS_PLANNING_FAILED after segments have executed. The result
carries neither proof field, so the campaign must treat it exactly like any unproven terminal:
retain the attempt and send nothing more, whatever evidence is refreshed.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

Status = AutonomousRestockCampaignStatus
PREFIX = "/test/campaign_restock_post_motion"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign against a coordinator fake that reports an unproven PLANNING_FAILED."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_post_motion_subject")


class TestPostMotionPlanningFailed(support.UnresolvedCampaignFixture):
    """An unproven PLANNING_FAILED wedges the campaign (fail-closed)."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_restock_post_motion"
    LANE_TARGET = 1

    def test_unproven_planning_failed_stays_unresolved(self):
        with self.world.lock:
            self.world.add_back("sim:post_motion_candidate", 1, "SIM-CAN-STD")
        type(self).restock_script = ["planning_failed_unproven"]
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            40.0,
            "motion_unresolved after the unproven PLANNING_FAILED",
        )
        block_time = next(t for t, m in self.snapshot_statuses() if support.unresolved(m))
        marked = next(m for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.assertEqual(marked.phase, Status.PHASE_BLOCKED)
        self.assertTrue(any(e.startswith("restock#") for e in support.attempts(marked)))
        with self.world.lock:
            self.world.lanes["lane_02"]["invalid"] = True
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
            "no goal is sent after a PLANNING_FAILED that proves nothing",
        )
