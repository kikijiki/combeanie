# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 058: a skip-mark-only OUTCOME_NO_CANDIDATE lifts the skip marks and retries.

Milestone 10 §6, spec committed before this code: a skip mark means "try others first",
not "never again". Four scripted shapes against one campaign process at the shipped
`max_skip_resurveys = 2`. The budget is monotone per run, so the cases are ordered to
spend it honestly — unittest runs them alphabetically:

- test_a: a recoverable transfer skip plants a mark; the forced survey then answers
  NO_CANDIDATE with an overview candidate sitting on that mark. The campaign does NOT park:
  it charges the lift rung (receipt on a MEASURED status, never PHASE_BLOCKED), the next
  survey goal carries the mark LIFTED (skip_mark_positions empty), the re-confirm selects
  the skipped product and the loop closes at PHASE_FRONT_FULL. Charge 1 of 2.
- test_b: the same marked state but an EMPTY overview spends nothing and parks at the
  unchanged honest PHASE_STOCK_EXHAUSTED — a genuinely absent product still reports no
  candidate.
- test_c: a second recoverable shape charges 2 of 2, and the third NO_CANDIDATE with a
  live candidate parks naming the rung and `max_skip_resurveys`, with the run's total
  receipts never exceeding the bound.
- test_d: a genuine confirm-refutation is never liftable: NO_CANDIDATE excluded by
  refuted_positions parks at the plain terminal and spends nothing.
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


class TestSkipLiftRecovery(fixture.SkipLiftFixture):
    """The Card 058 contract: lift the marks, retry the product, stay fail-closed."""

    # -- the four cases ------------------------------------------------------

    def test_a_forced_lift_recovers_the_skipped_product(self):
        """Skip-mark-only NO_CANDIDATE lifts the mark, re-surveys, closes at FRONT_FULL."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        event_mark = len(self.events)
        lane = "lane_01"
        product_class, sku = self.lane_identity(lane)
        # Scripts land BEFORE the world opens the deficit: at cycle_period 0 the wake is
        # immediate and a goal could otherwise beat the script append.
        self.restock_script.extend(["skip", {"lane": lane, "source": "sim:lift_a"}])
        self.tray_script.append({"outcome": "no_candidate_seen", "cls": product_class})
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:lift_a",
                "cls": product_class,
                "sku": sku,
            }
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:lift_a", product_class, sku)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers == 1
                for s in self.campaign_statuses[status_mark:]
            ),
            30.0,
            "PHASE_FRONT_FULL after the lift-recovered transfer",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
                and RECEIPT_PREFIX in s.detail
                and "attempt 1 of 2" in s.detail
                for s in window
            ),
            f"the lift rung must be receipted on a measured status: {[s.detail for s in window]}",
        )
        self.assertFalse(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                or s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in window
            ),
            "a recoverable skip-mark NO_CANDIDATE must never park or block on the first attempt",
        )
        goals = self.tray_goals[tray_mark:]
        self.assertGreaterEqual(len(goals), 2, [len(g.skip_mark_positions) for g in goals])
        self.assertEqual(
            [len(g.refuted_positions) for g in goals[:2]],
            [0, 0],
            "the campaign never merges skip marks into refuted_positions (Card 058)",
        )
        self.assertEqual(
            len(goals[0].skip_mark_positions),
            1,
            "the survey before the rung carries the skip mark",
        )
        self.assertEqual(
            len(goals[1].skip_mark_positions),
            0,
            "the forced re-survey carries the skip mark LIFTED",
        )
        events = [
            event[0] for event in self.events[event_mark:] if event[0] in ("transfer", "tray")
        ]
        self.assertEqual(
            events[-4:],
            ["transfer", "tray", "tray", "transfer"],
            "skip-path refusal, the marked survey, the lifted re-survey, the transfer",
        )

    def test_b_empty_overview_parks_without_spending(self):
        """An empty overview with marks present parks honestly: zero lift charges."""
        status_mark = len(self.campaign_statuses)
        charges_before = len(self._receipts())
        lane = "lane_02"
        product_class, sku = self.lane_identity(lane)
        self.tray_script.append({"outcome": "no_candidate_empty"})
        self.__class__.drain_on_park = "sim:lift_b"
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:lift_b", product_class, sku)
        parked = False

        def honestly_parked():
            nonlocal parked
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not parked:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                self._drops_back("sim:lift_b")
                parked = True
            return hit

        self._spin_until(
            honestly_parked,
            20.0,
            "PHASE_STOCK_EXHAUSTED when the overview found nothing at all",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertEqual(
            len(self._receipts()),
            charges_before,
            "an empty overview must not spend the lift budget",
        )
        for status in window:
            if status.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED:
                self.assertNotIn(
                    "max_skip_resurveys",
                    status.detail,
                    f"a genuinely empty overview parks on the plain terminal: {status.detail}",
                )

    def test_c_lift_budget_exhaustion_parks_naming_rung_and_bound(self):
        """A third live-candidate NO_CANDIDATE parks naming the rung and the budget."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        lane = "lane_03"
        product_class, sku = self.lane_identity(lane)
        self.tray_script.append({"outcome": "no_candidate_seen", "cls": product_class})
        self.tray_script.append({"outcome": "no_candidate_seen", "cls": product_class})
        self.__class__.drain_on_park = "sim:lift_c"
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:lift_c", product_class, sku)
        exhausted = False

        def budget_exhausted():
            nonlocal exhausted
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                and "max_skip_resurveys" in s.detail
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not exhausted:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                self._drops_back("sim:lift_c")
                exhausted = True
            return hit

        self._spin_until(
            budget_exhausted,
            20.0,
            "PHASE_STOCK_EXHAUSTED naming the skip-mark lift rung and max_skip_resurveys",
        )
        window = self.campaign_statuses[status_mark:]
        park = next(
            s
            for s in window
            if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
            and "max_skip_resurveys" in s.detail
        )
        self.assertIn("max_skip_resurveys=2", park.detail)
        self.assertIn("skip-mark lift rung", park.detail)
        self.assertTrue(
            any("attempt 2 of 2" in s.detail for s in window),
            f"the boundary charged its second attempt before the survey: "
            f"{[s.detail for s in window]}",
        )
        goals = self.tray_goals[tray_mark:]
        self.assertGreaterEqual(
            len(goals),
            2,
            "the charged survey and the lifted re-survey both reached the fixture",
        )
        self.assertGreater(
            len(goals[0].skip_mark_positions),
            0,
            "the survey before the second charge still carries the mark",
        )
        self.assertTrue(
            any(len(g.skip_mark_positions) == 0 for g in goals[1:]),
            "a forced re-survey in this window carries the skip mark LIFTED: "
            f"{[len(g.skip_mark_positions) for g in goals]}",
        )
        receipts = self._receipts()
        self.assertEqual(
            len(receipts),
            2,
            f"exactly max_skip_resurveys receipts for the whole run: "
            f"{[s.detail for s in receipts]}",
        )
        self.assertTrue(
            any("attempt 1 of 2" in s.detail for s in receipts)
            and any("attempt 2 of 2" in s.detail for s in receipts),
            f"each receipt names its attempt against the bound: {[s.detail for s in receipts]}",
        )
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED for s in window),
            "an exhausted lift budget parks at STOCK_EXHAUSTED, never blocked",
        )

    def test_d_confirm_refutation_is_never_lifted_and_spends_nothing(self):
        """A NO_CANDIDATE excluded by refuted_positions parks plainly: zero lift charges."""
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        charges_before = len(self._receipts())
        lane = "lane_04"
        product_class, sku = self.lane_identity(lane)
        self.tray_script.append({"outcome": "refuted", "cls": product_class})
        self.tray_script.append({"outcome": "no_candidate_seen", "cls": product_class})
        self.__class__.drain_on_park = "sim:lift_d"
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:lift_d", product_class, sku)
        parked = False

        def plainly_parked():
            nonlocal parked
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                and s.detail == PLAIN_PARK
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not parked:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                self._drops_back("sim:lift_d")
                parked = True
            return hit

        self._spin_until(
            plainly_parked,
            20.0,
            "the plain PHASE_STOCK_EXHAUSTED for a confirm-refuted exclusion",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertEqual(
            len(self._receipts()),
            charges_before,
            "a confirm-refutation must never spend the lift budget",
        )
        self.assertFalse(
            any("max_skip_resurveys" in s.detail or RECEIPT_PREFIX in s.detail for s in window),
            "the refuted exclusion must not be reported as a skip-mark boundary: "
            f"{[s.detail for s in window]}",
        )
        goals = self.tray_goals[tray_mark:]
        # A stale in-flight ask from the previous scenario's park window may prepend one
        # refuted-empty goal; otherwise the pair is exactly [not-yet-refuted, refuted].
        # Exactly one goal ever carries the refutation: it is never lifted or duplicated.
        self.assertIn(
            [len(g.refuted_positions) for g in goals[:3]],
            ([0, 1], [0, 0, 1]),
            "the refutation lands in refuted_positions and is never lifted",
        )
