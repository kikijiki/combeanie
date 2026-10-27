# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""CMB-116 tables and verdict rule: denominators from the scene, pass/fail/inconclusive."""

import json
from pathlib import Path
import sys

import pytest
from restocker_perception.identity import ladder_benchmark as lb
from restocker_perception.identity import robustness as rb
from restocker_perception.identity.ladder import IDENTIFIED, NEEDS_HUMAN

sys.path.insert(0, str(Path(__file__).resolve().parent))


class FakeRegistry:
    def __init__(self, products):
        self.products = products

    def resolve(self, sku):
        return sku if sku in self.products else None


GROUPS = {"GEN-001": "can-0", "GEN-002": "can-0", "GEN-017": "box-0", "GEN-018": "box-0"}


def _entry(status, sku=None, look=0):
    decision = {"status": status, "sku": sku, "look": look, "rung": "text"}
    return {"decisions": {"off": [decision]}, "truth": {}}


def _run(entries):
    run = lb.LadderRun.__new__(lb.LadderRun)
    run.objects = entries
    return run


def _truth(*items):
    return {
        (seed, f"sim:stock_{n}"): {"sku": sku, "product_class": cls}
        for (seed, n, sku, cls) in items
    }


def test_unseen_objects_stay_in_the_denominator_and_are_never_named():
    registry = FakeRegistry({"GEN-001", "GEN-002"})
    truth = _truth((1, "can_01", "GEN-001", "can"), (1, "can_02", "GEN-002", "can"))
    run = _run({(1, "sim:stock_can_01"): _entry(IDENTIFIED, "GEN-001")})
    rows = rb.object_table(run, registry, truth, GROUPS, arm="off", budget=0)
    assert [r["status"] for r in rows] == [IDENTIFIED, rb.UNSEEN]
    assert rb.metric(rows, "coverage_known") == (1, 2)
    assert rb.metric(rows, "seen_known") == (1, 2)
    assert rb.metric(rows, "false_identity_named") == (0, 1)


def test_wrong_and_unknown_named_are_counted_and_twin_flag_follows_the_registry():
    registry = FakeRegistry({"GEN-001", "GEN-002", "GEN-017"})
    truth = _truth(
        (1, "can_01", "GEN-001", "can"),
        (1, "box_01", "GEN-018", "box"),
        (1, "box_02", "GEN-017", "box"),
    )
    run = _run(
        {
            (1, "sim:stock_can_01"): _entry(IDENTIFIED, "GEN-002"),
            (1, "sim:stock_box_01"): _entry(IDENTIFIED, "GEN-017"),
            (1, "sim:stock_box_02"): _entry(NEEDS_HUMAN),
        }
    )
    rows = {
        r["object"]: r for r in rb.object_table(run, registry, truth, GROUPS, arm="off", budget=0)
    }
    assert rows["sim:stock_can_01"]["wrong"] and not rows["sim:stock_can_01"]["correct"]
    assert (
        rows["sim:stock_box_01"]["unknown_named"] and rows["sim:stock_box_01"]["twin_registered"]
    )
    assert rows["sim:stock_box_02"]["status"] == NEEDS_HUMAN
    assert rb.metric(list(rows.values()), "unknown_named") == (1, 1)
    assert rb.metric(list(rows.values()), "false_identity_named") == (1, 1)


def test_judge_follows_the_pass_rule():
    assert rb.judge("max", 0, 300, 0.02, 200)["status"] == "pass"
    assert rb.judge("max", 0, 100, 0.02, 200)["status"] == "inconclusive"  # n short
    assert rb.judge("max", 1, 100, 0.02, 50)["status"] == "inconclusive"  # point under, upper over
    assert rb.judge("max", 5, 100, 0.02, 50)["status"] == "fail"  # point over
    assert rb.judge("min", 90, 100, 0.8, 50)["status"] == "pass"
    assert rb.judge("min", 82, 100, 0.8, 50)["status"] == "inconclusive"
    assert rb.judge("min", 70, 100, 0.8, 50)["status"] == "fail"
    assert rb.judge("max", 0, 0, 0.02, 1)["status"] == "inconclusive"


def test_evaluate_pools_stress_conditions_and_reports_missing_ones():
    registry = FakeRegistry({"GEN-001"})
    truth = _truth(*[(s, f"can_{s}", "GEN-001", "can") for s in range(1, 41)])
    run = _run({k: _entry(IDENTIFIED, "GEN-001") for k in truth})
    rows = rb.object_table(run, registry, truth, GROUPS, arm="off", budget=0)
    bounds = {
        "stress_conditions": ["a", "b"],
        "bounds": [
            {
                "id": "pooled_false_identity",
                "conditions": "all_stress",
                "budget": 0,
                "metric": "false_identity_named",
                "direction": "max",
                "value": 0.05,
                "n_required": 60,
            },
            {
                "id": "a_coverage",
                "conditions": ["a"],
                "budget": 0,
                "metric": "coverage_known",
                "direction": "min",
                "value": 0.5,
                "n_required": 30,
            },
            {
                "id": "c_coverage",
                "conditions": ["c"],
                "budget": 0,
                "metric": "coverage_known",
                "direction": "min",
                "value": 0.5,
                "n_required": 30,
            },
        ],
    }
    verdicts = {v["id"]: v for v in rb.evaluate(bounds, {"a": {0: rows}, "b": {0: rows}})}
    assert verdicts["pooled_false_identity"]["n"] == 80
    assert verdicts["pooled_false_identity"]["status"] == "pass"
    assert verdicts["a_coverage"]["status"] == "pass"
    assert verdicts["c_coverage"] == {
        "id": "c_coverage",
        "status": "inconclusive",
        "missing": ["c"],
    }
    with pytest.raises(ValueError, match="unknown metric"):
        rb.metric(rows, "sparkle")


def test_truth_objects_reads_tray_objects_from_labels(tmp_path):
    label = {
        "seed": 7,
        "objects": [
            {
                "object_id": "sim:stock_can_01",
                "lane_id": None,
                "sku": "GEN-001",
                "product_class": "can",
            },
            {
                "object_id": "sim:stock_can_02",
                "lane_id": "lane_01",
                "sku": "GEN-001",
                "product_class": "can",
            },
        ],
    }
    (tmp_path / "labels.jsonl").write_text(json.dumps(label) + "\n" + json.dumps(label) + "\n")
    truth = rb.truth_objects([tmp_path])
    assert list(truth) == [(7, "sim:stock_can_01")]
    assert rb.truth_objects([tmp_path], seeds={8}) == {}
