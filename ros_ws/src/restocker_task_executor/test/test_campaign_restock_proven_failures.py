# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1b (CMB-SPEC-13): restock failures the coordinator proves are settled and retried.

The coordinator's result now carries motion_definitely_not_started and
execution_reached_terminal_stop. A first-leg planning failure that proves nothing was commanded,
and an execution failure whose terminal stop was observed, settle their attempts, so the campaign
retries after its backoff exactly as it did before stage 1 (no record, no marker, no wedge).
"""

import pytest
import unresolved_campaign_support as support

PREFIX = "/test/campaign_restock_proven"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign against a coordinator fake that proves its failures."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_proven_subject")


class TestRestockProvenFailures(support.UnresolvedCampaignFixture):
    """Provable failures retry after the backoff instead of wedging."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_restock_proven"
    LANE_TARGET = 1

    def test_provable_failures_settle_and_retry_after_the_backoff(self):
        with self.world.lock:
            self.world.add_back("sim:proven_candidate", 1, "SIM-CAN-STD")
        type(self).restock_script = [
            "planning_failed_not_started",
            "execution_failed_stopped",
            "planning_failed_not_started",
        ]
        self.spin_until(
            lambda: len([e for e in self.snapshot_events() if e[1] == "restock"]) >= 4,
            40.0,
            "the restock goal retried after each proven failure",
        )
        self.assertFalse(
            any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            "proven failures never open a record or set the marker",
        )
