# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Append-only extension of the frozen perception splits (version 2), and what it must preserve."""
# The round-1 held-out run was made against the version-1 file. Version 2 adds seeds so that a
# fresh held-out set is large enough to demonstrate a bound; it must not move a single old
# assignment, and the old held-out seeds must be recorded as burned.

import copy
import json
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_cli as cli  # noqa: E402
from restocker_benchmarks import perception_dataset as ds  # noqa: E402
from restocker_benchmarks import perception_splits as sp  # noqa: E402

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config"
V1_SHA256 = "bcbda3f865b917933bea94fcd9aed119d9a648da49be132c79094ee3b9f58963"
CATALOGUE_PATH = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        HERE.parents[1] / "restocker_gazebo" / "config" / "generated_product_labels.yaml",
    )
)


@pytest.fixture(scope="module")
def v1():
    return sp.load_splits(CONFIG / "perception_splits_v1.json")


@pytest.fixture(scope="module")
def v2():
    return sp.load_splits(CONFIG / "perception_splits.json")


def test_version_one_file_is_the_round_one_file(v1):
    assert v1["sha256"] == V1_SHA256 and v1["schema_version"] == 1
    assert v1["seeds"]["held_out"] == [3, 6, 7, 9, 15, 16]


def test_every_old_assignment_is_preserved(v1, v2):
    assert v2["schema_version"] == 2
    for name in sp.SPLIT_NAMES:
        assert set(v1["seeds"][name]) <= set(v2["seeds"][name])
        added = set(v2["seeds"][name]) - set(v1["seeds"][name])
        assert all(seed > 24 for seed in added)
    assert v2["products"] == v1["products"]
    assert v2["lookalike_groups"] == v1["lookalike_groups"]
    assert v2["catalogue_sha256"] == v1["catalogue_sha256"]
    # the file is a chain: version 1 -> first extension (seeds 25..184) -> second (185..234)
    first = v2["extension"]["base_document"]
    assert v2["extension"]["generation"] == 2 and first["extension"]["generation"] == 1
    assert first["extension"]["base_sha256"] == V1_SHA256
    assert first["extension"]["base_document"] == v1
    for name in sp.SPLIT_NAMES:
        assert set(first["seeds"][name]) <= set(v2["seeds"][name])
    assert first["held_out_generations"]["burned"] == v2["held_out_generations"]["burned"]


def test_old_held_out_seeds_are_burned_and_new_ones_fresh(v1, v2):
    generations = v2["held_out_generations"]
    assert generations["burned"] == v1["seeds"]["held_out"]
    assert not set(generations["burned"]) & set(generations["fresh"])
    assert set(generations["burned"]) | set(generations["fresh"]) == set(v2["seeds"]["held_out"])
    assert sp.held_out_generation(v2, 3) == "burned"
    assert sp.held_out_generation(v2, generations["fresh"][0]) == "fresh"
    assert sp.held_out_generation(v2, 1) is None
    assert len(generations["fresh"]) == 150


def test_seeds_are_partitioned_and_contiguous(v2):
    flat = [seed for name in sp.SPLIT_NAMES for seed in v2["seeds"][name]]
    assert sorted(flat) == list(range(1, 235))
    assert v2["extension"]["added_seed_range"] == [185, 234]


def test_the_extension_chain_is_reproducible_from_the_file_alone(v2):
    assert sp.reproduce_extension(v2) == v2


def test_tampering_is_detected(v2):
    edited = copy.deepcopy(v2)
    edited["seeds"]["train"].remove(1)
    edited["seeds"]["held_out"].append(1)
    assert sp.validate_splits(edited)
    base_moved = copy.deepcopy(v2)
    base_moved["seeds"]["validation"] = [s for s in base_moved["seeds"]["validation"] if s != 2]
    base_moved["seeds"]["train"].append(2)
    base_moved["sha256"] = sp.body_hash(base_moved)  # re-hashed: only the extension rules catch it
    assert any("not preserved" in p for p in sp.validate_splits(base_moved))


def test_burned_seeds_are_refused_for_calibration(v2):
    problems = sp.check_run(v2, {"calibration_seeds": [3, 6, 20]})
    assert any("seed 3" in p for p in problems) and any("seed 6" in p for p in problems)
    assert not sp.check_run(v2, {"calibration_seeds": [1, 2]})


def test_extension_needs_a_version_one_base_and_matching_counts(v1, v2):
    with pytest.raises(ValueError, match="sum"):
        sp.extend_splits(v1, last_seed=30, counts=(1, 1, 1))
    with pytest.raises(ValueError, match="non-negative"):
        sp.extend_splits(v1, last_seed=30, counts=(-1, 1, 30))
    # a later extension adds held-out seeds only when asked to, and keeps every earlier seed
    more = sp.extend_splits(v2, last_seed=240, counts=(0, 0, 6))
    assert more["held_out_generations"]["burned"] == v2["held_out_generations"]["burned"]
    assert set(more["held_out_generations"]["fresh"]) == set(
        v2["held_out_generations"]["fresh"]
    ) | set(range(235, 241))
    assert sp.validate_splits(more) == []


def test_added_seeds_obey_the_product_policy(v2):
    products = ds.load_generated_catalogue(CATALOGUE_PATH)
    held = set(v2["products"]["held_out"])
    unknown_seen = False
    for seed in (25, 26, 27, 30, 40, 60, 90, 130, 184):
        scene = cli.scene_for_seed(seed, products, v2)
        skus = {o["sku"] for o in scene["truth"]["objects"]}
        if seed in v2["seeds"]["held_out"]:
            unknown_seen |= bool(skus & held)
        else:
            assert not skus & held
    assert unknown_seen


def test_old_seeds_draw_the_same_scenes_under_version_two(v1, v2):
    products = ds.load_generated_catalogue(CATALOGUE_PATH)
    for seed in (1, 3, 7, 12, 24):
        before = cli.scene_for_seed(seed, products, v1)
        after = cli.scene_for_seed(seed, products, v2)
        assert ds.canonical_json(before["scenario"]) == ds.canonical_json(after["scenario"])
        assert before["seed_split"] == after["seed_split"]


def test_cli_verifies_both_versions(tmp_path, capsys):
    for name in ("perception_splits_v1.json", "perception_splits.json"):
        args = cli.build_parser().parse_args(
            ["splits", "--catalogue", str(CATALOGUE_PATH), "--verify", str(CONFIG / name)]
        )
        assert args.func(args) == 0
    assert "splits ok" in capsys.readouterr().out
    first, second = tmp_path / "first.json", tmp_path / "second.json"
    for base, last, counts, out in (
        (CONFIG / "perception_splits_v1.json", "184", "40,20,100", first),
        (first, "234", "0,0,50", second),
    ):
        args = cli.build_parser().parse_args(
            [
                "splits",
                "--catalogue",
                str(CATALOGUE_PATH),
                "--extend",
                str(base),
                "--last-seed",
                last,
                "--counts",
                counts,
                "--out",
                str(out),
            ]
        )
        assert args.func(args) == 0
    assert json.loads(second.read_text()) == json.loads(
        (CONFIG / "perception_splits.json").read_text()
    )
