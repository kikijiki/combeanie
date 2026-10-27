# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The CMB-116 bounds files are well formed and name only what the splits and populations hold."""

import hashlib
import json
from pathlib import Path

import pytest
from restocker_perception.identity import robustness as rb

SRC = Path(__file__).resolve().parents[2]
IDENTITY = SRC / "restocker_perception" / "config" / "identity_robustness_bounds_cmb116.json"
STOCK = SRC / "restocker_perception" / "config" / "lane_stock_robustness_bounds_cmb116.json"
SPLITS = SRC / "restocker_benchmarks" / "config" / "perception_splits_v7.json"
METRICS = {"false_identity_named", "coverage_known", "seen_known", "unknown_named"}


@pytest.fixture(scope="module")
def identity():
    return json.loads(IDENTITY.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def splits():
    return json.loads(SPLITS.read_text(encoding="utf-8"))


def test_splits_reference_matches_the_file_and_seeds_are_fresh_held_out(identity, splits):
    ref = identity["splits"]
    assert ref["file_sha256"] == hashlib.sha256(SPLITS.read_bytes()).hexdigest()
    assert ref["embedded_sha256"] == splits["sha256"]
    fresh = set(splits["held_out_generations"]["fresh"])
    assert set(ref["fresh_held_out_seeds"]) == fresh and len(fresh) == 120
    pops = identity["populations"]
    assert pops["nominal"]["seeds"] == sorted(fresh)
    assert pops["stress"]["seeds"] == sorted(fresh)[:40] == pops["image"]["seeds"]
    assert ref["unregistered_products"] == splits["identity_round"]["unregistered_products"]


def test_every_bound_is_complete_and_names_a_known_condition(identity):
    states = set(identity["stress_conditions"]) | {"nominal"}
    states |= set(identity["populations"]["image"]["state_names"])
    ids = [b["id"] for b in identity["bounds"]]
    assert len(ids) == len(set(ids))
    for bound in identity["bounds"]:
        assert bound["metric"] in METRICS and bound["direction"] in ("min", "max")
        assert 0.0 <= bound["value"] <= 1.0 and bound["n_required"] > 0 and bound["budget"] >= 0
        names = (
            identity["stress_conditions"]
            if bound["conditions"] == "all_stress"
            else bound["conditions"]
        )
        assert set(names) <= states, bound["id"]
        if bound["metric"] == "unknown_named":
            assert "group" in bound or bound["conditions"] != ["nominal"]


def test_budgets_follow_the_populations(identity):
    pops = identity["populations"]
    for bound in identity["bounds"]:
        names = (
            identity["stress_conditions"]
            if bound["conditions"] == "all_stress"
            else bound["conditions"]
        )
        if names == ["nominal"]:
            assert bound["budget"] <= pops["nominal"]["budget_relooks"]
        elif all(n.startswith("img_") for n in names):
            assert bound["budget"] == 0
        else:
            assert bound["budget"] <= pops["stress"]["budget_relooks"]


def test_the_file_runs_through_evaluate_on_empty_tables_as_inconclusive(identity):
    verdicts = rb.evaluate(identity, {})
    assert {v["status"] for v in verdicts} == {"inconclusive"}
    assert len(verdicts) == len(identity["bounds"])


def test_lane_stock_bounds_name_declared_arms_and_known_metrics():
    stock = json.loads(STOCK.read_text(encoding="utf-8"))
    arms = set(stock["arms"])
    ids = [b["id"] for b in stock["bounds"]]
    assert len(ids) == len(set(ids))
    for bound in stock["bounds"]:
        assert set(bound["arms"]) <= arms, bound["id"]
        assert bound["direction"] in ("min", "max") and bound["required_n"] > 0
    assert set(stock["splits"]["fresh_held_out_seeds"]) <= set(
        json.loads(SPLITS.read_text(encoding="utf-8"))["held_out_generations"]["fresh"]
    )
