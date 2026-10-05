# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 058 re-review note B (Card 069): an armed lift that is not needed lapses.

Milestone 10 §6, skip-mark lift rung amendment (4) and its Card 069 guard. Its own campaign
process (the lift budget is monotone per run), shipped `max_skip_resurveys = 2`:

- test_a: the lift is armed for class B, then B's front deficit closes before the next cycle
  while class A's stays open. The armed lift lapses: the next survey starts at A and carries
  every mark — no goal is ever sent with B's marks lifted for a class with nothing to fill.
- test_b: the lift is armed, then the front fills and the next cycle parks at
  PHASE_FRONT_FULL. The lift lapses with that cycle: when the deficit reopens, the next survey
  carries every mark ("try others first" is not lifted where it was never exhausted).
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import skip_lift_fixture as fixture

RECEIPT_PREFIX = fixture.RECEIPT_PREFIX


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped default lift budget (2)."""
    return fixture.launch_description()


class TestSkipLiftLapse(fixture.SkipLiftFixture):
    """An armed lift lives for one cycle and only for a class with a front deficit."""

    def _park_then_clear(self, status_mark, lanes, source):
        parked = False

        def plain_park():
            nonlocal parked
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not parked:
                with self.world.lock:
                    for lane in lanes:
                        self.world.lanes[lane]["target"] = 0
                self._drops_back(source)
                parked = True
            return hit

        return plain_park

    def test_a_lift_for_a_filled_class_lapses(self):
        """B armed, B's deficit closes, A's stays open: the survey starts at A, marks kept."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        lane_a, lane_b = "lane_01", "lane_02"
        class_a, _sku_a = self.lane_identity(lane_a)
        class_b, sku_b = self.lane_identity(lane_b)
        self.restock_script.append("skip")
        self.tray_script.append({"outcome": "no_candidate_empty", "cls": class_a})
        self.tray_script.append(
            {"outcome": "no_candidate_seen", "cls": class_b, "then_targets": {lane_b: 0}}
        )
        with self.world.lock:
            self.world.lanes[lane_a]["target"] = 1
            self.world.lanes[lane_b]["target"] = 1
            self.world.add_back("sim:lapse_b", class_b, sku_b)
        self._spin_until(
            self._park_then_clear(status_mark, (lane_a, lane_b), "sim:lapse_b"),
            30.0,
            "the plain park on empty A after B's armed lift lapsed",
        )
        window = self.campaign_statuses[status_mark:]
        receipts = [s for s in window if RECEIPT_PREFIX in s.detail]
        self.assertEqual(
            [("attempt 1 of 2" in s.detail) for s in receipts],
            [True],
            f"B qualified once: {[s.detail for s in window]}",
        )
        goals = [
            (g.product_class, len(g.skip_mark_positions)) for g in self.tray_goals[tray_mark:]
        ]
        self.assertGreaterEqual(len(goals), 3, goals)
        self.assertEqual(
            goals[:3],
            [(class_a, 1), (class_b, 1), (class_a, 1)],
            "after B's deficit closed the next survey starts at A and carries the mark",
        )
        self.assertTrue(
            all(marks > 0 for _cls, marks in goals),
            f"no goal ever carries lifted marks for a class with nothing to fill: {goals}",
        )

    def test_b_front_full_cycle_lapses_the_lift(self):
        """Armed, then PHASE_FRONT_FULL: when the deficit reopens every mark is carried."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_03"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("skip")
        self.tray_script.append(
            {"outcome": "no_candidate_seen", "cls": product_class, "then_targets": {lane: 0}}
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:lapse_c", product_class, sku)
        reopened = {"tray_mark": None}

        def armed_then_full():
            window = self.campaign_statuses[status_mark:]
            charged = next((i for i, s in enumerate(window) if RECEIPT_PREFIX in s.detail), None)
            if charged is None:
                return False
            full = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                for s in window[charged:]
            )
            if full and reopened["tray_mark"] is None:
                reopened["tray_mark"] = len(self.tray_goals)
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 1
            return full

        self._spin_until(armed_then_full, 30.0, "the charge, then a PHASE_FRONT_FULL cycle")
        self._spin_until(
            self._park_then_clear(len(self.campaign_statuses), (lane,), "sim:lapse_c"),
            30.0,
            "the plain park after the reopened deficit's survey",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any("attempt 2 of 2" in s.detail for s in window if RECEIPT_PREFIX in s.detail),
            f"the lift was charged and armed: {[s.detail for s in window]}",
        )
        reopened_at = reopened["tray_mark"]
        goals = self.tray_goals[reopened_at:]
        self.assertGreaterEqual(len(goals), 1, "the reopened deficit forced a survey")
        self.assertEqual(goals[0].product_class, product_class)
        self.assertTrue(
            all(len(g.skip_mark_positions) > 0 for g in goals),
            "the lift lapsed with the FRONT_FULL cycle: every later survey carries the marks: "
            f"{[len(g.skip_mark_positions) for g in goals]}",
        )
