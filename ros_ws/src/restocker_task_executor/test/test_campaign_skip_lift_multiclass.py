# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 058 review notes 1 and 2: the lift rung across classes, and the spent budget.

Milestone 10 §6. Its own campaign process (the lift budget is monotone per run), shipped
`max_skip_resurveys = 2`, cases ordered alphabetically:

- test_a: two outstanding deficit classes. Class A (the largest deficit, asked first) has
  an empty tray; class B's only product carries a live skip mark. The class fallthrough must
  also ask B, the rung must remember B as the qualifying class, and the next cycle's lifted
  survey must be a B goal with the mark lifted, so B's product is retried, not parked.
- test_b: SC-002(d). A product that has spent `max_product_skips` (2 of 2, "gone for the
  run") is never lifted: NO_CANDIDATE with an overview candidate on its mark parks on the
  plain terminal, spends no lift charge, and the survey still carries the mark.
- test_c: Card 080. The plain park's detail carries the per-station acquisition attribution
  (§6 "The no-candidate park names what each overview station saw"): a zero-admitted station
  reads its stage label plus counts, an admitting station reads its admitted count; with no
  reports the detail stays byte-identical to the old terminal (test_b pins that side).
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import skip_lift_fixture as fixture

PLAIN_PARK = fixture.PLAIN_PARK
RECEIPT_PREFIX = fixture.RECEIPT_PREFIX


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped default lift budget (2)."""
    return fixture.launch_description()


class TestSkipLiftMulticlass(fixture.SkipLiftFixture):
    """The lift is tied to the class that qualified; a spent product is never lifted."""

    def test_a_two_class_lift_retries_the_marked_class(self):
        """A empty, B skip-marked: B is retried with its mark lifted instead of parking."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        lane_a, lane_b = "lane_01", "lane_02"
        class_a, _sku_a = self.lane_identity(lane_a)
        class_b, sku_b = self.lane_identity(lane_b)
        self.assertNotEqual(class_a, class_b)
        # Scripts land before the world opens the deficits (cadence zero wakes at once).
        self.restock_script.extend(["skip", {"lane": lane_b, "source": "sim:mc_b"}])
        self.tray_script.append({"outcome": "no_candidate_empty", "cls": class_a})
        self.tray_script.append({"outcome": "no_candidate_seen", "cls": class_b})
        self.tray_script.append(
            {"outcome": "confirmed", "source": "sim:mc_b", "cls": class_b, "sku": sku_b}
        )
        with self.world.lock:
            # A's deficit is the larger, so every forced survey starts at A.
            self.world.lanes[lane_a]["target"] = 2
            self.world.lanes[lane_b]["target"] = 1
            self.world.add_back("sim:mc_b", class_b, sku_b)
        done = False

        def b_restocked_then_parked():
            nonlocal done
            window = self.campaign_statuses[status_mark:]
            restocked = any(s.successful_transfers >= 1 for s in window)
            parked = restocked and any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED for s in window
            )
            if parked and not done:
                with self.world.lock:
                    self.world.lanes[lane_a]["target"] = 0
                    self.world.lanes[lane_b]["target"] = 0
                done = True
            return parked

        self._spin_until(
            b_restocked_then_parked,
            30.0,
            "B's transfer after the class-tied lift, then the honest park on empty A",
        )
        window = self.campaign_statuses[status_mark:]
        receipts = [s for s in window if RECEIPT_PREFIX in s.detail]
        self.assertEqual(
            [("attempt 1 of 2" in s.detail) for s in receipts],
            [True],
            f"exactly one charged lift for B: {[s.detail for s in window]}",
        )
        goals = [
            (g.product_class, len(g.skip_mark_positions)) for g in self.tray_goals[tray_mark:]
        ]
        self.assertEqual(
            goals[:3],
            [(class_a, 1), (class_b, 1), (class_b, 0)],
            "A asked first, the fallthrough asks marked B, and the lifted survey is a B goal",
        )
        self.assertTrue(
            all(marks > 0 for _cls, marks in goals[3:]),
            f"the lift is spent by B's survey and never outlives it: {goals}",
        )

    def test_b_spent_product_budget_is_never_lifted(self):
        """SC-002(d): a gone-for-the-run mark keeps excluding; plain park, zero charges."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        lane = "lane_03"
        product_class, sku = self.lane_identity(lane)
        # test_a's mark stands on its transferred product (review note 6), so every cycle
        # here takes the survey path: confirm -> skip 1 (1 of 2) -> NO_CANDIDATE on the mark
        # -> the lift rung charges and the lifted survey re-confirms -> skip 2 (2 of 2, gone
        # for the run) -> NO_CANDIDATE on the mark again, which must NOT be lifted.
        self.restock_script.extend(["skip", "skip"])
        confirm = {
            "outcome": "confirmed",
            "source": "sim:gone_b",
            "cls": product_class,
            "sku": sku,
        }
        seen = {"outcome": "no_candidate_seen", "cls": product_class}
        self.tray_script.extend([confirm, dict(seen), dict(confirm), dict(seen)])
        self.__class__.drain_on_park = "sim:gone_b"
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:gone_b", product_class, sku)
        parked = False

        def plainly_parked():
            nonlocal parked
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not parked:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                self._drops_back("sim:gone_b")
                parked = True
            return hit

        self._spin_until(plainly_parked, 30.0, "the plain park on a gone-for-the-run mark")
        window = self.campaign_statuses[status_mark:]
        details = [s.detail for s in window]
        gone = next(
            (i for i, s in enumerate(window) if "treated as gone for the run" in s.detail), None
        )
        self.assertIsNotNone(gone, f"the product spent max_product_skips: {details}")
        self.assertEqual(
            [("attempt 2 of 2" in s.detail) for s in window[:gone] if RECEIPT_PREFIX in s.detail],
            [True],
            f"the one lift before the budget was spent (the second skip's retry): {details}",
        )
        self.assertFalse(
            any(
                RECEIPT_PREFIX in s.detail or "max_skip_resurveys" in s.detail
                for s in window[gone:]
            ),
            f"a gone-for-the-run mark is never lifted and never charges: {details}",
        )
        park = next(
            s
            for s in window[gone:]
            if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
        )
        self.assertEqual(park.detail, PLAIN_PARK, "a spent product parks on the plain terminal")
        goals = self.tray_goals[tray_mark:]
        self.assertGreaterEqual(len(goals), 4, [len(g.skip_mark_positions) for g in goals])
        self.assertEqual(
            len(goals[3].skip_mark_positions),
            2,
            "the survey after the budget is spent carries both standing marks (none lifted)",
        )

    def test_c_plain_park_attributes_every_station_report(self):
        """Card 080: the park detail names each station's stage, not just the idle string."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_01"
        product_class, _sku = self.lane_identity(lane)
        # One scripted empty survey whose result carries Card 050's station reports:
        # tray_1 viewed empty (the product left the tray), tray_2 admitted frames.
        self.tray_script.append(
            {
                "outcome": "no_candidate_empty",
                "cls": product_class,
                "stations": [
                    ("tray_1", (6, 6, 0, 0, 0)),
                    ("tray_2", (5, 5, 5, 4, 4)),
                ],
            }
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            # The parked loop only wakes for evidence (§6: "idle until evidence changes") —
            # a lane that needs a survey re-enters the work cycle; a bare target change does
            # not, and no tray candidate exists here by construction.
            self.world.lanes[lane]["invalid"] = True
        parked = False
        found = []

        def attributed_park():
            nonlocal parked
            window = self.campaign_statuses[status_mark:]
            hit = next(
                (
                    s
                    for s in window
                    if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                    and "station tray_1" in s.detail
                ),
                None,
            )
            if hit is None:
                return False
            found.append(hit)
            if not parked:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                parked = True
            return True

        self._spin_until(
            attributed_park,
            30.0,
            "the no-candidate park carrying the per-station acquisition attribution",
        )
        attributed = found[0]
        split_at = PLAIN_PARK.index("; the loop is idle")
        expected = (
            PLAIN_PARK[:split_at]
            + "; station tray_1: empty view: the colour stage found nothing in view"
            " (no proposal in any in-window detection frame)"
            " (images=6, detection_frames=6, frames_with_detections=0, published=0)"
            + "; station tray_2: admitted=4"
            + PLAIN_PARK[split_at:]
        )
        self.assertEqual(
            attributed.detail,
            expected,
            "the park names every station's stage, in report order (Card 080)",
        )
