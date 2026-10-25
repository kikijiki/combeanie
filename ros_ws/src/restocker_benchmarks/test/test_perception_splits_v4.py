# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Append-only extension of the box splits with a block of seeds (version 4, CMB-120 round 2)."""
# Version 4 adds seeds only: no product, the catalogue view and ``box_seed_start`` of version 3
# stay, and the held-out seeds that earlier lane-stock measurements decided on are moved from
# ``fresh`` to ``burned``.

import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_splits as sp  # noqa: E402

CONFIG = Path(__file__).resolve().parents[1] / "config"
V3_SHA256 = "36c95b64fe07c7448f4e864cbe1f663da9306784bc6730cf7d4be516b277e7c5"
# The 13 v3 held-out seeds the CMB-120 lane stock held-out run used (box-perception record).
LANE_STOCK_RUN = (242, 243, 246, 247, 257, 258, 260, 266, 267, 279, 280, 282, 283)


@pytest.fixture(scope="module")
def v3():
    return sp.load_splits(CONFIG / "perception_splits_v3.json")


@pytest.fixture(scope="module")
def v4():
    return sp.load_splits(CONFIG / "perception_splits_v4.json")


def test_version_three_is_untouched(v3):
    assert v3["sha256"] == V3_SHA256 and v3["schema_version"] == 3


def test_version_four_adds_seeds_only_and_keeps_every_assignment(v3, v4):
    assert v4["schema_version"] == 4 and v4["extension"]["base_sha256"] == V3_SHA256
    assert v4["products"] == v3["products"]
    assert v4["catalogue_sha256"] == v3["catalogue_sha256"]
    assert v4["box_seed_start"] == v3["box_seed_start"] == 235
    for name in sp.SPLIT_NAMES:
        assert set(v3["seeds"][name]) <= set(v4["seeds"][name])
    added = set(range(285, 365))
    assert v4["extension"]["added_seed_range"] == [285, 364]
    assert {s for name in sp.SPLIT_NAMES for s in v4["seeds"][name]} - {
        s for name in sp.SPLIT_NAMES for s in v3["seeds"][name]
    } == added
    assert added <= set(v4["seeds"]["held_out"])
    assert sp.scene_includes_boxes(v4, 285)


def test_the_seeds_the_lane_stock_run_used_are_burned_and_the_new_ones_fresh(v3, v4):
    burned = set(v4["held_out_generations"]["burned"])
    fresh = set(v4["held_out_generations"]["fresh"])
    assert set(v3["held_out_generations"]["burned"]) <= burned
    assert set(LANE_STOCK_RUN) <= burned and not set(LANE_STOCK_RUN) & fresh
    assert set(range(285, 365)) <= fresh
    assert burned | fresh == set(v4["seeds"]["held_out"]) and not burned & fresh


def test_the_chain_reproduces_and_a_tampered_file_is_refused(v4):
    assert sp.reproduce_extension(v4) == v4
    assert sp.validate_splits(v4) == []
    broken = copy.deepcopy(v4)
    broken["held_out_generations"]["burned"].remove(LANE_STOCK_RUN[0])
    broken["held_out_generations"]["fresh"].append(LANE_STOCK_RUN[0])
    assert sp.validate_splits(broken)  # the hash no longer matches
    broken["sha256"] = sp.body_hash(broken)
    assert sp.validate_splits(broken) == []  # re-hashed, the chain check is what catches it
    assert sp.reproduce_extension(broken) != broken


def test_a_version_four_file_extends_again_and_only_fresh_seeds_can_be_burned(v4):
    again = sp.extend_box_seeds(v4, last_seed=370, seed_counts=(0, 0, 6), burn=[285])
    assert again["extension"]["generation"] == v4["extension"]["generation"] + 1
    assert 285 in again["held_out_generations"]["burned"]
    assert sp.reproduce_extension(again) == again
    with pytest.raises(ValueError):
        sp.extend_box_seeds(v4, last_seed=370, seed_counts=(0, 0, 6), burn=[242])
