# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The inventory replay: seeded, reproducible, and scored against labelled truth."""

from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_benchmarks"))

from restocker_benchmarks import perception_inventory_scorer as scorer  # noqa: E402
from restocker_perception.inventory import replay  # noqa: E402
from restocker_perception.inventory.model import InventoryConfig  # noqa: E402

LANES = {
    "lane_01": ("A", ["A", "A", "A"]),
    "lane_02": ("B", ["B", "B"]),
    "lane_03": ("C", ["C", "C", "B"]),  # the rear product is a wrong B
    "lane_04": ("D", []),
}
TRAY = ["A", "E"]


def label(seed=3, frame="seed000003-wrist-001"):
    return {
        "frame_id": frame,
        "seed": seed,
        "lanes": [
            {"lane_id": lid, "expected_sku": sku, "skus_in_order": list(skus), "count": len(skus)}
            for lid, (sku, skus) in LANES.items()
        ],
        "objects": [{"lane_id": None, "sku": s} for s in TRAY],
    }


def prediction(lane_id, frame, *, named=(), count=None, answered=True, misplaced=False):
    truth = LANES[lane_id][1]
    n = len(truth) if count is None else count
    return {
        "kind": "lane",
        "frame_id": frame,
        "lane_id": lane_id,
        "state": "estimated" if answered else "cannot_tell",
        "reason": 0 if answered else 2,
        "count": n if answered else None,
        "count_min": n if answered else None,
        "count_max": n if answered else None,
        "composition": (3 if misplaced else (4 if n == 0 else 0)) if answered else None,
        "misplaced": misplaced if answered else None,
        "face_skus": list(named),
        "face_identity_status": [1] * len(named),
        "misplaced_skus": [s for s in named if s != LANES[lane_id][0]],
    }


def labels_and_rows(**kwargs):
    labels = []
    rows = []
    for lane_id in LANES:
        frame = f"seed000003-{lane_id}"
        labels.append({**label(frame=frame), "viewpoint": lane_id})
        rows.append(prediction(lane_id, frame, **kwargs.get(lane_id, {})))
    return labels, rows


def test_scene_truth_reads_lanes_and_tray():
    truth = replay.scene_truth(label())
    assert truth.seed == 3 and truth.tray == ("A", "E")
    assert truth.lanes["lane_03"].skus == ("C", "C", "B")
    assert dict(truth.sku_totals()) == {"A": 4, "B": 3, "C": 2, "E": 1}


def test_draws_are_pure_and_distinct():
    assert replay.draw("x", 1) == replay.draw("x", 1)
    assert replay.draw("x", 1) != replay.draw("x", 2)
    assert 0.0 <= replay.draw("x", 1) < 1.0


def test_perturbation_is_seeded_and_changes_the_ledger_at_the_front_only():
    perturbed = 0
    for seed in range(60):
        truth = replay.scene_truth(label(seed=seed))
        assert replay.perturbation(truth).count_delta == replay.perturbation(truth).count_delta
        p = replay.perturbation(truth)
        world = replay.world_view(truth, p)
        for lane_id, lane in truth.lanes.items():
            ledger = world.lanes[lane_id].skus
            delta = p.count_delta.get(lane_id, 0)
            assert len(ledger) == len(lane.skus) + delta >= 0
            if lane_id not in p.sku_swapped and ledger and lane.skus:
                assert ledger[-1] == lane.skus[-1]  # the rear product is untouched
            perturbed += int(delta != 0)
        for lane_id, wrong in p.sku_swapped.items():
            assert world.lanes[lane_id].skus[-1] == wrong != truth.lanes[lane_id].skus[-1]
    assert 30 <= perturbed <= 110  # about a third of 240 lanes


def test_unperturbed_world_is_the_truth():
    truth = replay.scene_truth(label())
    world = replay.world_view(truth)
    assert world.lanes["lane_03"].skus == ("C", "C", "B")
    assert [t.sku for t in world.tray] == ["A", "E"]


def test_ages_stay_inside_the_horizon_unless_aged():
    truth = replay.scene_truth(label())
    assert max(replay.reading_ages(truth, 600.0, aged=False).values()) <= 600.0 * 0.9
    spread = [
        age
        for seed in range(40)
        for age in replay.reading_ages(
            replay.scene_truth(label(seed=seed)), 600.0, aged=True
        ).values()
    ]
    assert max(spread) > 600.0 and min(spread) < 600.0 and max(spread) <= 900.0


def test_replay_scene_builds_the_inventory_of_the_scene():
    labels, rows = labels_and_rows()
    truth = replay.scene_truth(labels[0])
    view, _ = replay.replay_scene(truth, rows, InventoryConfig(600.0))
    assert [x.lane_id for x in view.lanes] == list(LANES)
    assert view.sku("A").total_estimate == 4  # lane_01's three plus the tray's one
    assert view.sku("A").complete and view.tray_count == 2


def test_scorer_counts_hidden_misplacement_as_an_error_within_one():
    labels, rows = labels_and_rows()
    report = scorer.score_inventory(labels, rows, InventoryConfig(600.0))
    overall = report["overall"]
    # C: policy count 3 against truth 2 (the hidden B); B: 2 against truth 3.
    assert report["error_histogram"] == {-1: 1, 0: 3, 1: 1}
    assert overall["sku_total_within_one"]["k"] == overall["sku_total_within_one"]["n"] == 5
    assert overall["sku_total_contains_truth"]["k"] == 3
    assert overall["confirmed_claim_contradicted"]["n"] == 0
    assert overall["stale_flagged"]["value"] in (None, 1.0)
    assert overall["count_discrepancy_false_alarm"]["value"] == 0.0


def test_scorer_flags_a_refusal_as_incomplete_never_as_agreement():
    labels, rows = labels_and_rows(lane_02={"answered": False})
    report = scorer.score_inventory(labels, rows, InventoryConfig(600.0))
    overall = report["overall"]
    assert overall["refused_policy_lane_claimed_complete"] == pytest.approx(
        {
            "value": 0.0,
            "ci95": overall["refused_policy_lane_claimed_complete"]["ci95"],
            "k": 0,
            "n": 1,
        }
    )
    assert overall["refusal_not_comparable"]["k"] == overall["refusal_not_comparable"]["n"] == 1
    assert overall["complete_pair_share"]["k"] < overall["complete_pair_share"]["n"]


def test_scorer_catches_a_confirmed_claim_the_labels_contradict():
    labels, rows = labels_and_rows(lane_01={"named": ["B"], "misplaced": True})
    report = scorer.score_inventory(labels, rows, InventoryConfig(600.0))
    claim = report["overall"]["confirmed_claim_contradicted"]
    assert (claim["k"], claim["n"]) == (1, 1)


def test_evaluate_bounds_applies_the_pass_rule_and_lists_missing_arms():
    labels, rows = labels_and_rows()
    report = scorer.score_inventory(labels, rows, InventoryConfig(600.0))
    bounds = {
        "bounds": [
            {
                "id": "b",
                "arms": ["abstain", "oracle"],
                "metric": "sku_total_within_one",
                "direction": "min",
                "value": 0.5,
                "required_n": 3,
            }
        ]
    }
    rows_out = scorer.evaluate_bounds({"abstain": report}, bounds)
    assert {r["arm"]: r["verdict"] for r in rows_out}["oracle"] == "not_run"
    assert {r["arm"]: r["verdict"] for r in rows_out}["abstain"] in ("pass", "inconclusive")
