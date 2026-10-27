# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Splits version 7 (CMB-116): fresh seeds, the r4 held-out run burned, boxes unregistered too."""
# Version 7 appends seeds to version 6, burns every held-out seed round 4 read (523..786) and moves
# two box products to the unregistered list, so the unknown-product test covers boxes as well as
# bottles: GEN-018 (the registered GEN-017's look-alike) and GEN-020 (a box with no sibling).

import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_splits as sp  # noqa: E402,I100,I101

CONFIG = Path(__file__).resolve().parent.parent / "config"
V6_SHA256 = "2aaee90575d037447fadbdd118aec063b4a385944279ea99fff92c22613e34c1"


@pytest.fixture(scope="module")
def v6():
    return sp.load_splits(CONFIG / "perception_splits_v6.json")


@pytest.fixture(scope="module")
def v7():
    return sp.load_splits(CONFIG / "perception_splits_v7.json")


def test_version_six_is_untouched(v6):
    assert v6["sha256"] == V6_SHA256


def test_version_seven_preserves_every_assignment_and_reproduces(v6, v7):
    assert v7["schema_version"] == 7 and v7["extension"]["base_sha256"] == V6_SHA256
    for kind in ("products", "seeds"):
        for name in sp.SPLIT_NAMES:
            assert set(v6[kind][name]) <= set(v7[kind][name])
    assert v7["products"] == v6["products"]
    assert sp.reproduce_extension(v7) == v7
    assert sp.validate_splits(v7) == []


def test_the_round_four_run_is_burned_and_the_new_held_out_seeds_are_fresh(v6, v7):
    assert v7["identity_round"]["burned_through_seed"] == 786
    assert v7["extension"]["added_seed_range"] == [787, 946]
    fresh = set(v7["held_out_generations"]["fresh"])
    assert fresh == {s for s in v7["seeds"]["held_out"] if s >= 787} and len(fresh) == 120
    assert set(v6["held_out_generations"]["fresh"]) <= set(v7["held_out_generations"]["burned"])
    assert sp.held_out_generation(v7, 523) == "burned"
    assert sp.held_out_generation(v7, 786) == "burned"
    assert sp.held_out_generation(v7, 787 if 787 in fresh else min(fresh)) == "fresh"


def test_two_box_products_join_the_unregistered_list(v6, v7):
    old = v6["identity_round"]
    new = v7["identity_round"]
    assert old["unregistered_products"] == ["GEN-008", "GEN-011"]
    assert new["unregistered_products"] == ["GEN-008", "GEN-011", "GEN-018", "GEN-020"]
    assert set(new["registered_products"]) == set(old["registered_products"]) - {
        "GEN-018",
        "GEN-020",
    }
    assert v7["lookalike_groups"]["GEN-017"] == v7["lookalike_groups"]["GEN-018"] == "box-0"


def test_unregistered_cannot_be_registered_again_or_invented(v6, v7):
    with pytest.raises(ValueError, match="subset of the registered"):
        sp.extend_identity_roles(
            v7,
            last_seed=1000,
            seed_counts=(1, 1, 1),
            burn_held_out_through=946,
            also_unregistered=["GEN-018"],
        )
    with pytest.raises(ValueError, match="subset of the registered"):
        sp.extend_identity_roles(
            v6,
            last_seed=800,
            seed_counts=(1, 1, 1),
            burn_held_out_through=786,
            also_unregistered=["GEN-999"],
        )
    tampered = copy.deepcopy(v7)
    tampered["identity_round"]["unregistered_products"] = ["GEN-008", "GEN-011"]
    tampered["identity_round"]["registered_products"] += ["GEN-018", "GEN-020"]
    tampered["identity_round"]["registered_products"].sort()
    tampered["sha256"] = sp.body_hash(tampered)
    assert any("registered again" in p or "declared" in p for p in sp.validate_splits(tampered))


def test_the_burn_line_cannot_move_down_and_old_bases_are_refused(v6, v7):
    with pytest.raises(ValueError, match="burn line"):
        sp.extend_identity_roles(
            v7,
            last_seed=1000,
            seed_counts=(18, 18, 18),
            burn_held_out_through=700,
            also_unregistered=["GEN-019"],
        )
    with pytest.raises(ValueError, match="version-6 or version-7"):
        sp.extend_identity_roles(
            sp.load_splits(CONFIG / "perception_splits_v4.json"),
            last_seed=800,
            seed_counts=(1, 1, 1),
            burn_held_out_through=786,
            also_unregistered=["GEN-019"],
        )


def test_train_and_validation_scenes_still_draw_only_train_and_validation_products(v7):
    allowed = sp.allowed_skus_for_seed(v7, v7["seeds"]["train"][-1])
    assert allowed == sp.registered_skus(v7)
    assert sp.allowed_skus_for_seed(v7, v7["held_out_generations"]["fresh"][0]) is None
