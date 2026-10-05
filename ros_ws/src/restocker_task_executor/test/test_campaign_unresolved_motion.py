# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 086 stage 1 (CMB-SPEC-13): the S21-class race, settled by exact identity.

A lane survey fails with a stop established, so the campaign commands the rung-1 retreat. The fake
retreat server holds the ADMISSION reply past action_timeout_sec, so the goal is in flight while
the campaign has no handle. Until the late goal is admitted, canceled by its exact UUID and its
delivered terminal proves a stop, the campaign must send nothing - not on elapsed backoff, not
on restored lane evidence, not on a fresh lane that needs a survey - and must never report
PHASE_FRONT_FULL or PHASE_COMPLETE. Sends are counted at the fake servers, not read from
reported states. Cross-endpoint exclusion is out of scope for stage 1 (the fakes are only
reachable through the campaign).
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import unresolved_campaign_support as support

PREFIX = "/test/campaign_unresolved_race"
ADMISSION_HANG_S = 3.0
Status = AutonomousRestockCampaignStatus


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign process; the test provides every action and the world snapshot."""
    return support.make_description(PREFIX, "autonomous_restock_campaign_unresolved_subject")


class TestUnresolvedRetreatRace(support.UnresolvedCampaignFixture):
    """The retreat admission race holds the campaign until exact-identity settlement."""

    PREFIX = PREFIX
    NODE_NAME = "campaign_unresolved_race"

    def test_retreat_race_holds_the_campaign_until_exact_identity_settlement(self):
        world = self.world
        self.spin_until(
            lambda: any(m.phase == Status.PHASE_FRONT_FULL for _, m in self.snapshot_statuses()),
            30.0,
            "the initial PHASE_FRONT_FULL (every lane at target)",
        )
        self.assertFalse(
            any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            "a settled campaign never claims unresolved motion",
        )

        # Trigger: lane_04 needs a survey and fails with a stop; the retreat admission hangs.
        type(self).retreat_admission_hang_sec = ADMISSION_HANG_S
        self.shelf_script["lane_04"] = ["fail_exec"]
        with world.lock:
            world.lanes["lane_04"]["invalid"] = True
        self.spin_until(
            lambda: any(support.unresolved(m) for _, m in self.snapshot_statuses()),
            20.0,
            "motion_unresolved after the retreat admission went unanswered",
        )
        block_time = next(t for t, m in self.snapshot_statuses() if support.unresolved(m))
        marked = next(m for _, m in self.snapshot_statuses() if support.unresolved(m))
        self.assertEqual(marked.phase, Status.PHASE_BLOCKED)
        self.assertTrue(
            any(entry.startswith("viewpoint#") for entry in support.attempts(marked)),
            f"the retained identity names the retreat attempt: {support.attempts(marked)}",
        )

        # Restore lane_04 AND make another lane need a survey: elapsed backoff, restored
        # evidence and fresh work must not authorize a single send while the retreat is unproven.
        with world.lock:
            world.lanes["lane_04"]["invalid"] = False
            world.lanes["lane_04"]["verified_age"] = 0.0
            world.lanes["lane_05"]["invalid"] = True
        blocked_before = sum(
            1
            for _, m in self.snapshot_statuses()
            if m.phase == Status.PHASE_BLOCKED and support.unresolved(m)
        )
        self.spin_until(
            lambda: (
                sum(
                    1
                    for _, m in self.snapshot_statuses()
                    if m.phase == Status.PHASE_BLOCKED and support.unresolved(m)
                )
                >= blocked_before + 2
            ),
            ADMISSION_HANG_S,
            "two further blocked backoffs while the late admission is still pending",
        )

        # The late goal lands, is canceled by its exact UUID, and its delivered terminal
        # (stop established) settles the attempt.
        self.spin_until(
            lambda: any(e[1] == "retreat_terminal" for e in self.snapshot_events()),
            ADMISSION_HANG_S + 15.0,
            "the late retreat goal to be admitted, canceled by exact UUID and terminate",
        )
        events = self.snapshot_events()
        goal_hex = next(e[2] for e in events if e[1] == "retreat_goal")
        self.assertIn(("retreat_cancel", goal_hex), [(e[1], e[2]) for e in events])
        terminal_time = next(e[0] for e in events if e[1] == "retreat_terminal")
        self.assertTrue(
            any(
                f"goal={goal_hex}" in entry
                for _, m in self.snapshot_statuses()
                for entry in support.attempts(m)
            ),
            "the record grew to carry the late goal's exact UUID",
        )

        # Nothing was sent between the block and the settling terminal (the late goal itself is
        # the one in-flight retreat that was already issued), and the campaign never parked or
        # completed in that window.
        late_sends = [
            e
            for e in events
            if e[1] in support.SENDS - {"retreat_goal"} and block_time < e[0] <= terminal_time
        ]
        self.assertEqual(late_sends, [], "no goal may be sent before the exact-identity terminal")
        parked = [
            m
            for t, m in self.snapshot_statuses()
            if block_time < t <= terminal_time
            and m.phase in {Status.PHASE_FRONT_FULL, Status.PHASE_COMPLETE}
        ]
        self.assertEqual(parked, [], "unresolved motion never reports front-full or complete")

        # After the settling terminal the campaign resumes: lane_05 is surveyed and the marker
        # clears.
        self.spin_until(
            lambda: any(
                e[1] == "shelf" and e[2] == "lane_05" and e[0] > terminal_time
                for e in self.snapshot_events()
            ),
            15.0,
            "the deferred lane_05 survey after settlement",
        )
        self.spin_until(
            lambda: any(
                (not support.unresolved(m))
                and t > terminal_time
                and m.phase == Status.PHASE_FRONT_FULL
                for t, m in self.snapshot_statuses()
            ),
            15.0,
            "the marker to clear and the campaign to park after settlement",
        )
