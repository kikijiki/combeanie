# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13 C1): an explicit admission rejection settles exactly that attempt.

An inhibited coordinator refuses the campaign's restock goal. An immediate refusal never opens a
record and the campaign retries after its backoff as before. A refusal that arrives after the
campaign's admission timeout settles the attempt that timed out, and only that one: the campaign
holds while the reply is pending and sends again once the rejection lands.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

Status = AutonomousRestockCampaignStatus
PREFIX = "/test/campaign_restock_rejection"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign against a coordinator fake that refuses restock goals."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_rejection_subject")


class TestRestockRejection(support.UnresolvedCampaignFixture):
    """A rejection is the one admission answer that proves nothing was commanded."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_restock_rejection"
    LANE_TARGET = 1

    def _restock_events(self):
        return [e for e in self.snapshot_events() if e[1] == "restock"]

    def test_rejection_settles_only_its_own_attempt(self):
        # A valid candidate skips the tray survey, so the restock goal is the first send.
        with self.world.lock:
            self.world.add_back("sim:rejection_candidate", 1, "SIM-CAN-STD")
        type(self).restock_mode = "reject"
        self.spin_until(
            lambda: len(self._restock_events()) >= 3,
            40.0,
            "three immediately refused restock goals retried after the backoff",
        )
        self.assertFalse(
            any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            "an immediate rejection proves nothing was commanded: no record is ever opened",
        )

        # Now the refusal arrives after the campaign's admission timeout.
        type(self).restock_mode = "late_reject"
        type(self).restock_admission_hang_sec = 3.0
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            20.0,
            "motion_unresolved while the restock admission reply is outstanding",
        )
        held = next(m for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.assertTrue(
            any(entry.startswith("restock#") for entry in support.attempts(held)),
            f"the pending admission is retained: {support.attempts(held)}",
        )
        pending_sends = len(self._restock_events())
        self.spin_until(
            lambda: any(e[1] == "restock_late_reject" for e in self.snapshot_events()),
            10.0,
            "the late rejection reply",
        )
        late = next(e for e in self.snapshot_events() if e[1] == "restock_late_reject")
        self.assertEqual(
            len([e for e in self._restock_events() if e[0] < late[0]]),
            pending_sends,
            "no further restock goal is sent while the late admission reply is pending",
        )
        self.assertEqual(
            [e for e in self.sends() if e[1] != "restock" and e[0] > held_time(self, held)],
            [],
            "nothing else is sent while the attempt is unresolved",
        )

        # The rejection settles that attempt (and only it): the campaign sends again.
        type(self).restock_mode = "reject"
        self.spin_until(
            lambda: (
                len(self._restock_events()) > pending_sends
                and not support.unresolved(self.snapshot_statuses()[-1][1])
            ),
            20.0,
            "a new restock goal after the late rejection settled its attempt",
        )


def held_time(test, held):
    """Return the receive time of the first status carrying ``held``'s identities."""
    return next(t for t, m in test.snapshot_statuses() if m is held)
