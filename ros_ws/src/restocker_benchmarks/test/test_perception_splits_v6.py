# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Splits version 6 (CMB-121 round 4): seeds appended, round 3's decision seeds burned."""
# Version 6 appends seeds to version 5, keeps the identity-round registry roles, and burns every
# held-out seed up to the burn line: the seeds the previous round decided a bound on can never
# decide another.

import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_splits as sp  # noqa: E402,I100,I101

CONFIG = Path(__file__).resolve().parent.parent / "config"
V5_SHA256 = "c7b36ad6158309a39ed3e653ac12eedac5756529b87298014569bb64b5273df8"


@pytest.fixture(scope="module")
def v5():
    return sp.load_splits(CONFIG / "perception_splits_v5.json")


@pytest.fixture(scope="module")
def v6():
    return sp.load_splits(CONFIG / "perception_splits_v6.json")


def test_version_five_is_untouched(v5):
    assert v5["sha256"] == V5_SHA256


def test_version_six_preserves_every_assignment_and_reproduces(v5, v6):
    assert v6["schema_version"] == 6 and v6["extension"]["base_sha256"] == V5_SHA256
    for kind in ("products", "seeds"):
        for name in sp.SPLIT_NAMES:
            assert set(v5[kind][name]) <= set(v6[kind][name])
    assert v6["products"] == v5["products"]
    assert sp.reproduce_extension(v6) == v6
    assert sp.validate_splits(v6) == []


def test_round_three_decision_seeds_are_burned_and_the_new_ones_fresh(v5, v6):
    assert v6["identity_round"]["burned_through_seed"] == 522
    assert v6["extension"]["added_seed_range"][0] == 523
    burned = set(v6["held_out_generations"]["burned"])
    fresh = set(v6["held_out_generations"]["fresh"])
    assert set(range(365, 523)) & set(v6["seeds"]["held_out"]) <= burned
    assert set(v5["held_out_generations"]["burned"]) <= burned
    assert fresh == {s for s in v6["seeds"]["held_out"] if s >= 523}
    assert len(fresh) == 240
    assert sp.held_out_generation(v6, 365) == "burned"
    assert sp.held_out_generation(v6, max(fresh)) == "fresh"


def test_registry_roles_are_those_of_the_base(v5, v6):
    assert (
        v6["identity_round"]["registered_products"] == v5["identity_round"]["registered_products"]
    )
    assert (
        v6["identity_round"]["unregistered_products"]
        == v5["identity_round"]["unregistered_products"]
    )


def test_the_burn_line_cannot_move_down_and_foreign_bases_are_refused(v5, v6):
    with pytest.raises(ValueError, match="burn line"):
        sp.extend_identity_seeds(
            v6, last_seed=800, seed_counts=(2, 1, 11), burn_held_out_through=500
        )
    with pytest.raises(ValueError, match="version-5 or version-6"):
        sp.extend_identity_seeds(
            sp.load_splits(CONFIG / "perception_splits_v4.json"),
            last_seed=800,
            seed_counts=(1, 1, 1),
            burn_held_out_through=522,
        )


def test_a_chained_extension_keeps_everything_burned(v6):
    chained = sp.extend_identity_seeds(
        v6, last_seed=800, seed_counts=(4, 3, 7), burn_held_out_through=786
    )
    assert sp.validate_splits(chained) == []
    assert sp.reproduce_extension(chained) == chained
    carried = set(chained["held_out_generations"]["burned"])
    assert set(v6["held_out_generations"]["fresh"]) <= carried
    assert len(chained["held_out_generations"]["fresh"]) == 7


def test_an_edited_file_or_an_unburned_seed_below_the_line_is_refused(v6):
    edited = copy.deepcopy(v6)
    edited["held_out_generations"]["fresh"].append(400)
    edited["held_out_generations"]["burned"].remove(400)
    edited["sha256"] = sp.body_hash(edited)
    assert any("burn line" in p for p in sp.validate_splits(edited))
