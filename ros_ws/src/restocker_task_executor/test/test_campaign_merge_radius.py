# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 066 (review cmbrev066b N2): the campaign matches skip marks with the server's merge radius.

Milestone 10 §6 "One merge radius". The server excludes candidates within its configurable
`candidate_merge_radius_m` of a mark and reports that radius in the result; the campaign's lift
qualification must use the reported radius, not its own fixed 0.04 m. Its own campaign process
at the shipped lift budget (2):

- test_a: the control, first because a reported radius persists. With no radius reported yet,
  the 0.04 m default applies: a candidate 0.045 m from a live skip mark is not near it, and no
  lift is charged.
- test_b: test_a's live mark (a skip mark outlives its drained product) stands where the
  fixture puts every candidate. A candidate 0.045 m from it, with a reported radius of 0.05 m, is
  excluded only by that mark as far as the server is concerned, so the lift is charged and the
  lifted survey confirms the product. With the fixed 0.04 m the campaign sees no marked
  candidate and parks instead.
"""

import pytest
from restocker_interfaces.msg import AutonomousRestockCampaignStatus
import skip_lift_fixture as fixture

LIFT_RECEIPT = fixture.RECEIPT_PREFIX


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped default lift budget (2)."""
    return fixture.launch_description()


class TestMergeRadius(fixture.SkipLiftFixture):
    """The lift qualification uses the radius the server reported."""

    def test_b_the_reported_radius_decides_the_lift(self):
        """0.045 m off the mark, reported radius 0.05 m: the lift is charged and recovers."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_02"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append({"lane": lane, "source": "sim:radius_a"})
        self.tray_script.append(
            {
                "outcome": "no_candidate_seen",
                "cls": product_class,
                "offset": 0.045,
                "radius": 0.05,
            }
        )
        self.tray_script.append(
            {"outcome": "confirmed", "source": "sim:radius_a", "cls": product_class, "sku": sku}
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:radius_a", product_class, sku)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers >= 1
                for s in self.campaign_statuses[status_mark:]
            ),
            30.0,
            "PHASE_FRONT_FULL after a lift qualified with the reported radius",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertEqual(
            len([s for s in window if LIFT_RECEIPT in s.detail]), 1, "one lift charged"
        )

    def test_a_without_a_report_the_default_radius_applies(self):
        """No radius reported: 0.045 m is beyond the 0.04 m default, so nothing is lifted."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_01"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("skip")
        self.tray_script.append(
            {"outcome": "no_candidate_seen", "cls": product_class, "offset": 0.045}
        )
        self.__class__.drain_on_park = "sim:radius_b"
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:radius_b", product_class, sku)
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
                parked = True
            return hit

        self._spin_until(plainly_parked, 30.0, "the plain park without a lift")
        window = self.campaign_statuses[status_mark:]
        self.assertEqual([s.detail for s in window if LIFT_RECEIPT in s.detail], [])
