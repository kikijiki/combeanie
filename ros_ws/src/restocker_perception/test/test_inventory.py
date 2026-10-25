# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Inventory aggregation and query on synthetic lane readings, tray and world state."""

from types import SimpleNamespace

import pytest
from restocker_perception.inventory import model as m
from restocker_perception.inventory.aggregate import build_inventory
from restocker_perception.inventory.model import (
    InventoryConfig,
    LaneReading,
    TrayItem,
    TrayReading,
    WorldLane,
    WorldTrayItem,
    WorldView,
)
from restocker_perception.inventory.query import query_inventory
from restocker_perception.inventory.world import world_view_from_snapshot

NOW = 1000.0
CONFIG = InventoryConfig(stale_after_s=300.0)


def reading(
    lane="lane_01",
    sku="A",
    count=3,
    lo=None,
    hi=None,
    age=10.0,
    names=(),
    unknown=0,
    misplaced=False,
    misplaced_skus=(),
    composition=None,
    answered=True,
    reason=0,
):
    lo = count if lo is None else lo
    hi = count if hi is None else hi
    statuses = (m.IDENTITY_NAMED,) * len(names) + (m.IDENTITY_UNKNOWN,) * unknown
    skus = tuple(names) + ("",) * unknown
    if composition is None:
        composition = m.COMPOSITION_MISPLACED if misplaced else (4 if count == 0 else 0)
    return LaneReading(
        lane_id=lane,
        stamp_s=NOW - age,
        answered=answered,
        reason=reason,
        has_expected_sku=sku is not None,
        expected_sku=sku or "",
        count_estimate=count if answered else 0,
        count_min=lo if answered else 0,
        count_max=hi if answered else 0,
        composition=composition,
        misplaced=misplaced,
        face_skus=skus,
        face_identity_status=statuses,
        misplaced_named_count=len(misplaced_skus),
        misplaced_skus=tuple(misplaced_skus),
    )


def world(lanes=None, tray=(), revision=7):
    return WorldView(
        revision=revision,
        lanes={w.lane_id: w for w in (lanes or [])},
        tray=tuple(WorldTrayItem(s) for s in tray),
    )


def wlane(lane="lane_01", sku="A", skus=("A", "A", "A"), unreliable=False):
    return WorldLane(lane, sku, tuple(skus), unreliable)


def inv(readings, w=None, tray=None, **kwargs):
    return build_inventory({r.lane_id: r for r in readings}, w, tray, NOW, CONFIG, **kwargs)


# --- counts and the SKU basis ----------------------------------


def test_policy_only_lane_is_expected_not_confirmed():
    view = inv([reading(count=4)], world([wlane(skus=("A",) * 4)]))
    line = view.lane("lane_01")
    assert line.sku_basis == m.SKU_BASIS_EXPECTED_ONLY
    assert line.confirmed_count == 0 and line.confirmed_skus == []
    sku = view.sku("A")
    assert (sku.confirmed_count, sku.expected_count_min, sku.expected_count_max) == (0, 4, 4)
    assert sku.total_estimate == 4


def test_named_products_are_confirmed_and_never_double_counted():
    view = inv([reading(count=4, names=("A",))], world([wlane(skus=("A",) * 4)]))
    line = view.lane("lane_01")
    assert line.sku_basis == m.SKU_BASIS_PARTLY_CONFIRMED
    sku = view.sku("A")
    assert sku.confirmed_count == 1
    assert (sku.expected_count_min, sku.expected_count_max) == (3, 3)
    assert (sku.total_min, sku.total_max, sku.total_estimate) == (4, 4, 4)


def test_every_product_named_as_expected_is_confirmed_lane():
    view = inv([reading(count=1, names=("A",))])
    assert view.lane("lane_01").sku_basis == m.SKU_BASIS_CONFIRMED
    assert view.sku("A").confirmed_count == 1


def test_count_range_carries_into_the_total():
    view = inv([reading(count=3, lo=2, hi=4)])
    sku = view.sku("A")
    assert (sku.total_min, sku.total_estimate, sku.total_max) == (2, 3, 4)


def test_empty_lane_is_a_zero_not_an_unknown():
    view = inv([reading(count=0)])
    line = view.lane("lane_01")
    assert line.empty and line.counted and line.count_max == 0
    assert view.sku("A").total_estimate == 0 and view.sku("A").complete is False  # no tray info


def test_unknown_product_is_never_attributed_to_the_policy_sku():
    view = inv([reading(count=3, unknown=1)])
    assert view.lane("lane_01").unknown_count == 1
    unknown = next(s for s in view.skus if s.is_unknown_bucket)
    assert unknown.expected_count_min == 1 and unknown.expected_count_max == 1
    assert view.sku("A").expected_count_max == 2


def test_lane_without_policy_counts_into_the_unknown_bucket():
    view = inv([reading(sku=None, count=2)])
    unknown = next(s for s in view.skus if s.is_unknown_bucket)
    assert (unknown.total_min, unknown.total_max) == (2, 2)
    assert view.lane("lane_01").sku_basis == m.SKU_BASIS_UNKNOWN


def test_misplaced_lane_is_contested_and_widens_the_policy_share():
    view = inv(
        [reading(count=4, names=("B",), misplaced=True, misplaced_skus=("B",))],
        world([wlane(skus=("A", "A", "A", "A"))]),
    )
    line = view.lane("lane_01")
    assert line.sku_basis == m.SKU_BASIS_CONTESTED and line.misplaced
    assert view.sku("B").confirmed_count == 1
    a = view.sku("A")
    assert (a.expected_count_min, a.expected_count_max) == (0, 3)
    assert a.contested_lanes == ["lane_01"]
    assert view.sku("B").contested_lanes == ["lane_01"]


def test_class_mismatch_without_a_name_puts_the_wrong_products_in_the_unknown_bucket():
    view = inv([reading(count=4, misplaced=True)])
    unknown = next(s for s in view.skus if s.is_unknown_bucket)
    assert (unknown.expected_count_min, unknown.expected_count_max) == (0, 4)
    assert view.sku("A").expected_count_max == 4 and view.sku("A").expected_count_min == 0


def test_more_names_than_counted_products_never_shrinks_the_count():
    view = inv([reading(count=1, names=("A", "A"))])
    assert view.lane("lane_01").count_max == 2


# --- unread, stale, never ----------------------------------


def test_cannot_tell_lane_contributes_nothing_and_makes_its_sku_incomplete():
    view = inv(
        [
            reading(lane="lane_01", sku="A", count=3),
            reading(lane="lane_02", sku="A", answered=False, reason=2),
        ],
        world(tray=()),
    )
    a = view.sku("A")
    assert a.total_estimate == 3
    assert a.unread_lanes == ["lane_02"] and a.complete is False
    assert view.lane("lane_02").cannot_tell and view.lane("lane_02").cannot_tell_reason == 2
    assert view.lane("lane_02").count_discrepancy == m.COUNT_DISCREPANCY_NOT_COMPARABLE


def test_lane_never_read_is_unread_not_zero():
    w = world([wlane("lane_01", "A", ("A",) * 2), wlane("lane_02", "B", ("B",) * 5)])
    view = inv([reading(lane="lane_01", sku="A", count=2)], w)
    assert view.lane("lane_02").observation == m.OBSERVATION_NEVER
    b = view.sku("B")
    assert b.total_estimate == 0 and b.unread_lanes == ["lane_02"] and not b.complete
    assert b.world_count == 5 and b.world_difference == 0  # difference only when complete


def test_stale_lane_is_flagged_and_excluded_from_totals():
    view = inv([reading(count=3, age=301.0)], world())
    line = view.lane("lane_01")
    assert line.observation == m.OBSERVATION_STALE and line.age_s == pytest.approx(301.0)
    assert line.count_estimate == 3  # last-known stays visible
    a = view.sku("A")
    assert a.total_estimate == 0 and a.stale_lanes == ["lane_01"] and not a.complete


def test_horizon_boundary_is_inclusive_fresh():
    assert inv([reading(age=300.0)]).lane("lane_01").observation == m.OBSERVATION_FRESH
    assert inv([reading(age=300.5)]).lane("lane_01").observation == m.OBSERVATION_STALE


def test_max_age_filter_overrides_the_declared_horizon():
    view = inv([reading(age=100.0)], max_age_s=50.0)
    assert view.lane("lane_01").observation == m.OBSERVATION_STALE
    assert view.max_age_s == 50.0 and view.stale_after_s == 300.0


def test_future_stamp_is_clamped_to_age_zero():
    r = reading(age=-5.0)
    assert inv([r]).lane("lane_01").age_s == 0.0


# --- the tray ----------------------------------


def test_tray_falls_back_to_the_world_state_as_reported_not_confirmed():
    w = world([wlane(skus=("A",))], tray=("A", "B", None))
    view = inv([reading(count=1)], w)
    assert view.tray_source == m.TRAY_SOURCE_WORLD_STATE and view.tray_count == 3
    a, b = view.sku("A"), view.sku("B")
    assert a.reported_count == 1 and a.tray_count == 1 and a.total_estimate == 2
    assert b.reported_count == 1 and b.confirmed_count == 0
    assert next(s for s in view.skus if s.is_unknown_bucket).reported_count == 1
    assert a.complete


def test_perception_tray_wins_over_the_world_state():
    tray = TrayReading(NOW - 5, (TrayItem("A", True), TrayItem("A"), TrayItem(None)))
    view = inv([reading(count=1)], world([wlane(skus=("A",))], tray=("A",)), tray)
    assert view.tray_source == m.TRAY_SOURCE_PERCEPTION and view.tray_count == 3
    a = view.sku("A")
    assert a.confirmed_count == 1 and a.expected_count_max == 2 and a.reported_count == 0
    assert a.tray_source == m.TRAY_SOURCE_PERCEPTION


def test_stale_perception_tray_falls_back_and_says_so():
    tray = TrayReading(NOW - 999, (TrayItem("A", True),))
    view = inv([reading(count=1)], world(tray=("B",)), tray)
    assert view.tray_stale and view.tray_source == m.TRAY_SOURCE_WORLD_STATE
    assert view.sku("A").confirmed_count == 0 and view.sku("B").reported_count == 1


def test_no_tray_source_means_no_sku_is_complete():
    view = inv([reading(count=2)])
    assert view.tray_source == m.TRAY_SOURCE_NONE
    assert view.sku("A").complete is False


# --- discrepancy against the world state ----------------------------------


@pytest.mark.parametrize(
    ("ledger", "lo", "hi", "kind", "delta"),
    [
        (3, 3, 3, m.COUNT_DISCREPANCY_NONE, 0),
        (2, 3, 3, m.COUNT_DISCREPANCY_OBSERVED_HIGHER, 1),
        (5, 3, 3, m.COUNT_DISCREPANCY_OBSERVED_LOWER, -2),
        (4, 3, 4, m.COUNT_DISCREPANCY_NONE, 0),  # a range that includes the ledger agrees
        (1, 3, 4, m.COUNT_DISCREPANCY_OBSERVED_HIGHER, 2),
    ],
)
def test_count_discrepancy_compares_the_range_to_the_ledger(ledger, lo, hi, kind, delta):
    view = inv([reading(count=lo, lo=lo, hi=hi)], world([wlane(skus=("A",) * ledger)]))
    line = view.lane("lane_01")
    assert (line.count_discrepancy, line.count_delta) == (kind, delta)
    assert line.has_discrepancy == (kind != m.COUNT_DISCREPANCY_NONE)


def test_unreadable_lane_is_not_comparable_rather_than_agreeing():
    view = inv([reading(answered=False, reason=2)], world([wlane()]))
    line = view.lane("lane_01")
    assert line.count_discrepancy == m.COUNT_DISCREPANCY_NOT_COMPARABLE
    assert not line.has_discrepancy
    assert view.discrepancy_count == 0


def test_named_sku_contradicting_the_ledgers_rear_is_a_sku_discrepancy():
    view = inv(
        [reading(count=2, names=("B",), misplaced=True, misplaced_skus=("B",))],
        world([wlane(skus=("A", "A"))]),
    )
    line = view.lane("lane_01")
    assert line.sku_discrepancy and "ledger rear A" in line.discrepancy_detail


def test_an_abstention_never_raises_a_sku_discrepancy():
    view = inv([reading(count=2, unknown=0)], world([wlane(skus=("B", "B"))]))
    assert not view.lane("lane_01").sku_discrepancy


def test_ledger_unreliable_is_carried_and_noted():
    view = inv([reading(count=2)], world([wlane(skus=("A",) * 3, unreliable=True)]))
    line = view.lane("lane_01")
    assert line.world_ledger_unreliable and "unreliable" in line.discrepancy_detail


def test_sku_world_difference_only_when_complete():
    w = world([wlane(skus=("A",) * 5)], tray=())
    view = inv([reading(count=3)], w)
    a = view.sku("A")
    assert a.complete and a.world_count == 5 and a.world_difference == -2


# --- the query ----------------------------------


def run(readings, w=None, tray=None, **kwargs):
    return query_inventory({r.lane_id: r for r in readings}, w, tray, NOW, CONFIG, **kwargs)


def test_sku_query_answers_with_range_and_explanation():
    readings = [reading("lane_01", "A", 3, names=("A",)), reading("lane_02", "A", 2, lo=1, hi=3)]
    answer = run(readings, world(tray=("A",)), sku="A")
    assert answer.ok and answer.count_known
    assert (answer.count_min, answer.count_estimate, answer.count_max) == (5, 6, 7)
    assert answer.confirmed_count == 1 and answer.complete
    assert "confirmed" in answer.explanation and "world state" in answer.explanation
    assert [x.lane_id for x in answer.view.lanes] == ["lane_01", "lane_02"]


def test_sku_query_says_when_a_lane_could_not_be_read():
    readings = [reading("lane_01", "A", 3), reading("lane_02", "A", answered=False, reason=1)]
    answer = run(readings, world(), sku="A")
    assert (
        answer.complete is False
        and "lane_02" in answer.explanation
        and "low" in answer.explanation
    )


def test_min_confidence_confirmed_counts_only_named_products():
    readings = [reading("lane_01", "A", 4, names=("A",))]
    answer = run(readings, world(tray=("A", "A")), sku="A", min_confidence=m.CONFIDENCE_CONFIRMED)
    assert answer.count_estimate == 1 and answer.confirmed_count == 1
    assert answer.excluded_count == 3 + 2
    assert "less evidence" in answer.explanation


def test_min_confidence_expected_drops_only_world_reports():
    readings = [reading("lane_01", "A", 4)]
    answer = run(readings, world(tray=("A", "A")), sku="A", min_confidence=m.CONFIDENCE_EXPECTED)
    assert answer.count_estimate == 4 and answer.excluded_count == 2


def test_max_age_filter_drops_older_lanes_from_the_answer():
    readings = [reading("lane_01", "A", 3, age=10.0), reading("lane_02", "A", 2, age=200.0)]
    answer = run(readings, world(), sku="A", max_age_s=100.0)
    assert answer.count_estimate == 3 and not answer.complete
    assert "lane_02" in answer.explanation


def test_lane_query_returns_only_that_lane_and_ignores_the_tray():
    readings = [reading("lane_01", "A", 3), reading("lane_02", "B", 2)]
    answer = run(readings, world(tray=("A",)), lane_id="lane_01")
    assert answer.ok and [x.lane_id for x in answer.view.lanes] == ["lane_01"]
    assert [s.sku for s in answer.view.skus] == ["A"]
    assert answer.view.skus[0].total_estimate == 3 and answer.complete


def test_lane_and_sku_query_gives_the_lanes_contribution():
    readings = [reading("lane_01", "A", 3), reading("lane_02", "A", 2)]
    answer = run(readings, world(), lane_id="lane_02", sku="A")
    assert answer.count_estimate == 2 and answer.complete


def test_unknown_lane_is_an_error_not_an_empty_answer():
    answer = run([reading()], world(), lane_id="lane_99")
    assert not answer.ok and "lane_99" in answer.message


def test_unknown_sku_is_zero_observed_with_the_caveat():
    answer = run(
        [reading("lane_01", "A", 3), reading("lane_02", "B", answered=False)], world(), sku="Z"
    )
    assert answer.ok and answer.count_estimate == 0 and not answer.complete
    assert "lane_02" in answer.explanation


def test_invalid_filters_are_refused():
    assert not run([reading()], min_confidence=9).ok
    assert not run([reading()], max_age_s=-1.0).ok


def test_whole_inventory_query_counts_lanes_by_state():
    readings = [
        reading("lane_01", "A", 3),
        reading("lane_02", "B", 2, age=900.0),
        reading("lane_03", "C", answered=False, reason=2),
    ]
    view = run(readings, world([wlane("lane_04", "D", ("D",))])).view
    assert (view.lanes_total, view.lanes_fresh, view.lanes_stale, view.lanes_unread) == (
        4,
        2,
        1,
        3,
    )


# --- the world adapter ----------------------------------


def snapshot(objects, lanes):
    return SimpleNamespace(revision=9, objects=objects, lanes=lanes)


def obj(i, sku=None, tracking=0, grasp=0, source=None):
    return SimpleNamespace(
        id=i,
        sku=sku or "",
        has_sku=sku is not None,
        tracking_state=tracking,
        grasp_state=grasp,
        source_object_id=source or f"sim:{i}",
    )


def shelf(lane_id, contents, sku="A", observed=()):
    return SimpleNamespace(
        id=lane_id,
        contents=list(contents),
        has_expected_sku=sku is not None,
        expected_sku=sku or "",
        observed_source_object_ids=list(observed),
        ledger_unreliable=False,
    )


def test_world_view_lane_ledger_and_tray_split():
    snap = snapshot(
        [
            obj(1, "A"),
            obj(2, "A"),
            obj(3, None),
            obj(4, "B"),
            obj(5, "B", tracking=3),
            obj(6, "B", grasp=1),
        ],
        [shelf("lane_01", [1, 2]), shelf("lane_02", [3], sku="B")],
    )
    view = world_view_from_snapshot(snap)
    assert view.revision == 9
    assert view.lanes["lane_01"].skus == ("A", "A") and view.lanes["lane_02"].skus == (None,)
    # 4 is free and unaccounted; 5 is removed; 6 is in the gripper.
    assert [t.sku for t in view.tray] == ["B"]


def test_world_view_does_not_put_observed_lane_objects_on_the_tray():
    snap = snapshot([obj(1, "A", source="sim:x")], [shelf("lane_01", [], observed=["sim:x"])])
    assert world_view_from_snapshot(snap).tray == ()
