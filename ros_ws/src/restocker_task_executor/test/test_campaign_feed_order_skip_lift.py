# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 066 × Card 058 integration: the skip-mark lift and the feed-order rung together.

Milestone 10 §6 (Card 066's feed-front contract; Card 058's lift). Its own campaign process
with `max_skip_resurveys = 1`, so the lift can be spent inside one file, and the shipped
`max_feed_order_resurveys = 2`. The scripted survey reports, per overview candidate, whether it
is a feed-column front (`overview_candidate_feed_front`) and how much matching stock stood
behind a front (`feed_blocked_candidates`), as the real server does. The fixture places every
overview candidate at one tray pose, and a skip mark outlives its absent product, so test_a's live
mark (1 of 2 product skips) is what excludes the candidate in b and c. Cases run alphabetically:

- test_a: the only candidate carries a live skip mark but stands behind another product, so
  lifting it could not make it nominable. No lift is charged (the run-long lift budget is not
  burnt for nothing), and it is not exhaustion either (review cmbrev066b N1): the feed-order
  rung runs, then a PHASE_BLOCKED names the blocking front and the stock behind it.
- test_b: a live-skip-marked front with stock behind it: both rungs qualify, and the lift runs
  first (only it retries the skipped front). Its receipt is attempt 1 of 1; no feed-order
  receipt; the lifted survey confirms and the transfer closes at PHASE_FRONT_FULL.
- test_c: the same state with the lift spent: the feed-order rung takes over (attempts 1 and 2
  of 2), then a PHASE_BLOCKED naming feed order and max_feed_order_resurveys, never
  PHASE_STOCK_EXHAUSTED while stock stands behind a front.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import skip_lift_fixture as fixture

LIFT_RECEIPT = fixture.RECEIPT_PREFIX
FEED_RECEIPT = "feed-order NO_CANDIDATE recovery"


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero with a one-attempt lift budget."""
    return fixture.launch_description({"max_skip_resurveys": 1, "max_feed_order_resurveys": 2})


class TestFeedOrderSkipLift(fixture.SkipLiftFixture):
    """The lift counts only fronts, runs before the feed rung, and hands over when spent."""

    def _details(self, window, prefix):
        return [s.detail for s in window if prefix in s.detail]

    def _close_on(self, lanes, predicate):
        closed = False

        def check():
            nonlocal closed
            hit = predicate()
            if hit and not closed:
                with self.world.lock:
                    for lane in lanes:
                        self.world.lanes[lane]["target"] = 0
                closed = True
            return hit

        return check

    def test_a_a_skipped_rear_behind_a_front_is_a_named_block_not_exhaustion(self):
        """A marked rear behind a front: no lift, the feed rung, then a named PHASE_BLOCKED."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_01"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("skip")
        for _ in range(3):
            self.tray_script.append(
                {"outcome": "no_candidate_seen", "cls": product_class, "front": False}
            )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:rear_a", product_class, sku)
        self._spin_until(
            self._close_on(
                [lane],
                lambda: any(
                    s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                    and "max_feed_order_resurveys=2" in s.detail
                    for s in self.campaign_statuses[status_mark:]
                ),
            ),
            30.0,
            "the feed rung's named PHASE_BLOCKED for marked stock behind a front",
        )
        statuses = self.campaign_statuses[status_mark:]
        latch = next(
            index
            for index, s in enumerate(statuses)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
            and "max_feed_order_resurveys" in s.detail
        )
        window = statuses[: latch + 1]
        self.assertIn(fixture.FEED_BLOCK_EXAMPLE, window[-1].detail, "names front and stock")
        self.assertEqual(self._details(window, LIFT_RECEIPT), [], "no lift is charged")
        self.assertEqual(len(self._details(window, FEED_RECEIPT)), 2)
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED for s in window),
            f"marked stock behind a front is not exhausted: {[s.detail for s in window]}",
        )

    def test_b_a_skipped_front_with_stock_behind_lifts_before_the_feed_rung(self):
        """Both rungs qualify; the lift runs first and the skipped front is transferred."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        lane = "lane_02"
        product_class, sku = self.lane_identity(lane)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        self.restock_script.append({"lane": lane, "source": "sim:front_b"})
        self.tray_script.append(
            {
                "outcome": "no_candidate_seen",
                "cls": product_class,
                "front": True,
                "feed_blocked": 1,
            }
        )
        self.tray_script.append(
            {"outcome": "confirmed", "source": "sim:front_b", "cls": product_class, "sku": sku}
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:front_b", product_class, sku)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers == transfers + 1
                for s in self.campaign_statuses[status_mark:]
            ),
            30.0,
            "PHASE_FRONT_FULL after the lift retried the skipped front",
        )
        window = self.campaign_statuses[status_mark:]
        lifts = self._details(window, LIFT_RECEIPT)
        self.assertEqual(len(lifts), 1, lifts)
        self.assertIn("attempt 1 of 1", lifts[0])
        self.assertEqual(
            self._details(window, FEED_RECEIPT), [], "the feed rung never ran ahead of the lift"
        )
        goals = self.tray_goals[tray_mark:]
        self.assertEqual(
            [len(g.skip_mark_positions) for g in goals[:2]],
            [1, 0],
            "the marked survey, then the lifted one",
        )

    def test_c_a_spent_lift_hands_feed_blocked_stock_to_the_feed_rung(self):
        """Lift spent: feed-order attempts 1 and 2, then a named PHASE_BLOCKED, never exhausted."""
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        lane = "lane_03"
        product_class, sku = self.lane_identity(lane)
        for _ in range(3):
            self.tray_script.append(
                {
                    "outcome": "no_candidate_seen",
                    "cls": product_class,
                    "front": True,
                    "feed_blocked": 1,
                }
            )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:front_c", product_class, sku)
        self._spin_until(
            self._close_on(
                [lane],
                lambda: any(
                    s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                    and "max_feed_order_resurveys=2" in s.detail
                    for s in self.campaign_statuses[status_mark:]
                ),
            ),
            30.0,
            "the feed-order rung's named PHASE_BLOCKED after the spent lift",
        )
        statuses = self.campaign_statuses[status_mark:]
        latch = next(
            index
            for index, s in enumerate(statuses)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
            and "max_feed_order_resurveys" in s.detail
        )
        window = statuses[: latch + 1]
        self.assertIn("feed order", window[-1].detail)
        feed = self._details(window, FEED_RECEIPT)
        self.assertEqual(len(feed), 2, feed)
        self.assertTrue("attempt 1 of 2" in feed[0] and "attempt 2 of 2" in feed[1], feed)
        self.assertEqual(self._details(window, LIFT_RECEIPT), [], "the lift budget was spent")
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED for s in window),
            f"stock behind a front is never exhausted: {[(s.phase, s.detail) for s in window]}",
        )
        self.assertNotIn(("transfer",), self.events[event_mark:], "nothing was transferred")
