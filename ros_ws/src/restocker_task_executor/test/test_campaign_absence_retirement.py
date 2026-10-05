# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 069: a completed tray survey that viewed a counted product's place empty retires it.

Milestone 10 §6 (committed before this code). Every product here is stale (last observed
300 s ago, pinned), so it still counts as back stock (Card 058 increment 2) but is never a
valid candidate: each cycle with an open deficit forces a tray survey, and the scripted
survey answers NO_CANDIDATE with one absence verdict per probe. A park is woken by expiring
the lane's evidence, as the 60 s horizon does in production. Shipped
`absence_confirmations = 2`:

- test_a (SC-002): one viewed-empty survey never retires; the second consecutive one does —
  receipt naming the bound, then `back:0` and no probe for the product.
- test_b (SC-003): SEEN resets the streak; OCCLUDED / NOT_COVERED / NO_EVIDENCE never count,
  so viewed-empty, seen, viewed-empty, occluded, not-covered, no-evidence never retires.
- test_c (SC-004): a retired product observed again (newer than the retirement) counts again
  and is restocked.
- test_d (review N3): counted empty views must be independent — two surveys in ONE cycle
  (the class fallthrough) count once, and a view from the next cycle but inside
  `absence_min_separation_sec` does not count; only a later cycle past the separation retires.
- test_e (review N5c/N5d): a retired product whose last observation still looks fresh forces
  the survey path, so the transfer that follows is pinned to the close-confirmed product
  behind it — never an unpinned skip-path pick that could land on the ghost.

- test_f (review cmbrev069b B1): the same, with the ghost last observed just past
  `tray_evidence_validity` but inside the coordinator's selection horizon — the gate window is
  max(tray_evidence_validity, coordinator_object_max_age_ms), so the transfer is still pinned.

This file shortens `absence_min_separation_sec` to SEPARATION_S (shipped: 30 s),
`tray_evidence_validity_ms` to VALIDITY_S and `coordinator_object_max_age_ms` to HORIZON_S
(shipped: 120 s / 180 s), and sleeps past the separation before every wake that must count.
"""

import time

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus, TrackedObject
import skip_lift_fixture as fixture

RETIRED = "retired from back stock"
STALE_S = 300.0
SEPARATION_S = 2.0
VALIDITY_S = 20.0
HORIZON_S = 60.0


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped absence_confirmations (2)."""
    return fixture.launch_description(
        {
            "absence_min_separation_sec": SEPARATION_S,
            "tray_evidence_validity_ms": int(VALIDITY_S * 1000),
            "coordinator_object_max_age_ms": int(HORIZON_S * 1000),
        }
    )


class TestAbsenceRetirement(fixture.SkipLiftFixture):
    """Observed absence retires a product only after a bounded, explicit confirmation."""

    def _parks(self, mark):
        return [
            s
            for s in self.campaign_statuses[mark:]
            if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
        ]

    def _survey_and_park(self, lane, product_class, probe, first):
        """Script one survey answer, wake the park if needed, wait for the next park."""
        mark = len(self.campaign_statuses)
        self.tray_script.append(
            {"outcome": "no_candidate_empty", "cls": product_class, "probe": probe}
        )
        if not first:
            time.sleep(SEPARATION_S + 0.3)
            self.world.expire_lane(lane)
        self._spin_until(lambda: bool(self._parks(mark)), 20.0, f"the park after a {probe} survey")
        return mark

    def _retirements(self, mark, source):
        return [
            s for s in self.campaign_statuses[mark:] if RETIRED in s.detail and source in s.detail
        ]

    def _open(self, lane, source):
        product_class, sku = self.lane_identity(lane)
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back(source, product_class, sku, observed_at=time.time() - STALE_S)
        # A stale product is never a fresh candidate, so the idle loop is woken the way the
        # 60 s lane-evidence horizon wakes it in production.
        self.world.expire_lane(lane)
        return product_class

    def _close(self, lane, source):
        with self.world.lock:
            self.world.lanes[lane]["target"] = 0
        self._drops_back(source)

    def test_a_two_consecutive_empty_views_retire(self):
        """One empty view is not enough; the second consecutive one retires the product."""
        lane, source = "lane_01", "sim:abs_a"
        status_mark = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        product_class = None
        self.tray_script.append(
            {
                "outcome": "no_candidate_empty",
                "cls": self.lane_identity(lane)[0],
                "probe": "viewed_empty",
            }
        )
        product_class = self._open(lane, source)
        self._spin_until(
            lambda: bool(self._parks(status_mark)), 20.0, "the park after the first empty view"
        )
        first_park = self._parks(status_mark)[0]
        slot = list(first_park.product_classes).index(product_class)
        self.assertEqual(first_park.back_stock[slot], 1, "stale, not gone: still back:1")
        self.assertEqual(
            self._retirements(status_mark, source), [], "one empty view never retires"
        )
        self.assertGreater(len(self.tray_goals[tray_mark].absence_probe_positions), 0)

        self._survey_and_park(lane, product_class, "viewed_empty", first=False)
        retired = self._retirements(status_mark, source)
        self.assertEqual(len(retired), 1, "the second consecutive empty view retires it")
        self.assertIn("absence_confirmations=2", retired[0].detail)
        self.assertEqual(retired[0].phase, AutonomousRestockCampaignStatus.PHASE_MEASURED)

        after = len(self.campaign_statuses)
        goals_before = len(self.tray_goals)
        self.world.expire_lane(lane)
        self._spin_until(
            lambda: bool(self._parks(after)), 20.0, "the park measured after the retirement"
        )
        park = self._parks(after)[0]
        self.assertEqual(park.back_stock[slot], 0, "the retired product no longer counts")
        self.assertTrue(
            all(len(g.absence_probe_positions) == 0 for g in self.tray_goals[goals_before:]),
            "a retired product is no longer probed",
        )
        self._close(lane, source)

    def test_b_seen_resets_and_non_evidence_never_counts(self):
        """viewed-empty, seen, viewed-empty, occluded, not-covered, no-evidence: never retired."""
        lane, source = "lane_02", "sim:abs_b"
        status_mark = len(self.campaign_statuses)
        product_class = self.lane_identity(lane)[0]
        sequence = [
            "viewed_empty",
            "seen",
            "viewed_empty",
            "occluded",
            "not_covered",
            "no_evidence",
        ]
        self.tray_script.append(
            {"outcome": "no_candidate_empty", "cls": product_class, "probe": sequence[0]}
        )
        self._open(lane, source)
        self._spin_until(
            lambda: bool(self._parks(status_mark)), 20.0, "the park after the first survey"
        )
        for probe in sequence[1:]:
            self._survey_and_park(lane, product_class, probe, first=False)
        self.assertEqual(
            self._retirements(status_mark, source),
            [],
            "SEEN resets the streak and non-evidence verdicts never extend it",
        )
        last = self._parks(status_mark)[-1]
        slot = list(last.product_classes).index(product_class)
        self.assertEqual(last.back_stock[slot], 1, "the product still counts")
        self._close(lane, source)

    def test_c_a_newer_observation_restores_a_retired_product(self):
        """Retired, then observed again: it counts again and is restocked."""
        lane, source = "lane_03", "sim:abs_c"
        status_mark = len(self.campaign_statuses)
        product_class = self.lane_identity(lane)[0]
        self.tray_script.append(
            {"outcome": "no_candidate_empty", "cls": product_class, "probe": "viewed_empty"}
        )
        self._open(lane, source)
        self._spin_until(
            lambda: bool(self._parks(status_mark)), 20.0, "the park after the first empty view"
        )
        self._survey_and_park(lane, product_class, "viewed_empty", first=False)
        self.assertEqual(len(self._retirements(status_mark, source)), 1)

        restored = len(self.campaign_statuses)
        self.restock_script.append({"lane": lane, "source": source})
        self.world.observe(source)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers >= 1
                for s in self.campaign_statuses[restored:]
            ),
            20.0,
            "the re-observed product restocked (PHASE_FRONT_FULL after a transfer)",
        )
        self._close(lane, source)

    def test_d_empty_views_count_only_when_independent(self):
        """Same cycle counts once; next cycle inside the separation does not count."""
        lane_p, lane_q = "lane_04", "lane_05"
        source = "sim:abs_d"
        class_p = self.lane_identity(lane_p)[0]
        class_q = self.lane_identity(lane_q)[0]
        status_mark = len(self.campaign_statuses)

        def both_classes():
            # P's deficit (2) is the larger, so every cycle asks P first, then falls through
            # to Q; both surveys probe P's place and both answer "viewed empty".
            self.tray_script.append(
                {"outcome": "no_candidate_empty", "cls": class_p, "probe": "viewed_empty"}
            )
            self.tray_script.append(
                {"outcome": "no_candidate_empty", "cls": class_q, "probe": "viewed_empty"}
            )

        both_classes()
        with self.world.lock:
            self.world.lanes[lane_q]["target"] = 1
        product_class, sku = self.lane_identity(lane_p)
        with self.world.lock:
            self.world.lanes[lane_p]["target"] = 2
            self.world.add_back(source, product_class, sku, observed_at=time.time() - STALE_S)
        self.world.expire_lane(lane_p)
        self._spin_until(
            lambda: bool(self._parks(status_mark)), 20.0, "the park after cycle 1 (two surveys)"
        )
        self.assertEqual(
            self._retirements(status_mark, source), [], "two surveys in one cycle count once"
        )

        # Next cycle at once: independent of the cycle, but inside the separation.
        mark = len(self.campaign_statuses)
        both_classes()
        self.world.expire_lane(lane_p)
        self._spin_until(lambda: bool(self._parks(mark)), 20.0, "the park after cycle 2")
        self.assertEqual(
            self._retirements(status_mark, source),
            [],
            "a view inside absence_min_separation_sec of the counted one does not count",
        )

        # Past the separation: the second independent view retires.
        mark = len(self.campaign_statuses)
        both_classes()
        time.sleep(SEPARATION_S + 0.3)
        self.world.expire_lane(lane_p)
        self._spin_until(lambda: bool(self._parks(mark)), 20.0, "the park after cycle 3")
        self.assertEqual(
            len(self._retirements(status_mark, source)),
            1,
            "the second independent empty view retires the product, exactly once",
        )
        with self.world.lock:
            self.world.lanes[lane_q]["target"] = 0
        self._close(lane_p, source)

    def test_e_a_fresh_looking_ghost_forces_a_pinned_transfer_to_the_next(self):
        """Retired but still fresh-looking: the next product is surveyed, confirmed, pinned."""
        self._ghost_forces_a_pinned_transfer("lane_06", "sim:ghost_e", "sim:behind_e", 5.0)

    def test_f_a_ghost_past_tray_validity_but_inside_the_coordinator_horizon_is_gated(self):
        """Observed VALIDITY_S + 0.5 s ago: past tray validity, inside the selection horizon."""
        self._ghost_forces_a_pinned_transfer(
            "lane_01", "sim:ghost_f", "sim:behind_f", VALIDITY_S + 0.5
        )

    def _ghost_forces_a_pinned_transfer(self, lane, ghost, behind, ghost_age_s):
        product_class, sku = self.lane_identity(lane)
        status_mark = len(self.campaign_statuses)
        self.tray_script.append(
            {"outcome": "no_candidate_empty", "cls": product_class, "probe": "viewed_empty"}
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            # Observed ghost_age_s ago with an unknown orientation: it counts as stock and is
            # probed, but is never a valid candidate, so every cycle surveys the tray.
            self.world.add_back(ghost, product_class, sku, observed_at=time.time() - ghost_age_s)
            self.world.back[-1]["orientation"] = TrackedObject.ORIENTATION_UNKNOWN
        self.world.expire_lane(lane)
        self._spin_until(
            lambda: bool(self._parks(status_mark)), 20.0, "the park after the first empty view"
        )
        self._survey_and_park(lane, product_class, "viewed_empty", first=False)
        self.assertEqual(len(self._retirements(status_mark, ghost)), 1, "the ghost is retired")

        after = len(self.campaign_statuses)
        tray_mark = len(self.tray_goals)
        restock_mark = len(self.restock_goals)
        self.tray_script.append(
            {"outcome": "confirmed", "source": behind, "cls": product_class, "sku": sku}
        )
        self.restock_script.append({"lane": lane, "source": behind})
        with self.world.lock:
            self.world.add_back(behind, product_class, sku)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers >= 1
                for s in self.campaign_statuses[after:]
            ),
            20.0,
            "the product behind restocked after the ghost's retirement",
        )
        self.assertGreaterEqual(
            len(self.tray_goals) - tray_mark,
            1,
            "a fresh-looking retired product forces the survey path (no skip-path pick)",
        )
        goals = self.restock_goals[restock_mark:]
        self.assertEqual(len(goals), 1, "exactly one transfer")
        self.assertTrue(goals[0].has_object_id, "the transfer is pinned to the confirmed product")
        with self.world.lock:
            self.world.lanes[lane]["target"] = 0
        self._drops_back(ghost)
