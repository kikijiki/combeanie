# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Append-only extension of the splits with the box SKUs (version 3, CMB-119)."""
# Adding box SKUs changes the catalogue the splits freeze. Versions 1 and 2 stay valid against the
# catalogue view they were frozen on (no contour bottle, no box); version 3 pins the view with
# boxes and appends the box SKUs and a block of seeds whose scenes may draw them. No older seed
# changes the scene it draws.

import copy
import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_cli as cli  # noqa: E402
from restocker_benchmarks import perception_dataset as ds  # noqa: E402
from restocker_benchmarks import perception_proposal_scorer as scorer  # noqa: E402
from restocker_benchmarks import perception_proposals as pp  # noqa: E402
from restocker_benchmarks import perception_splits as sp  # noqa: E402

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config"
V2_SHA256 = "72b4060006cbd8466c18028c735cdaced2438f5a11fd2fdec084497f9fd1376a"
CATALOGUE_PATH = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        HERE.parents[1] / "restocker_gazebo" / "config" / "generated_product_labels.yaml",
    )
)
BOX_SKUS = ("GEN-017", "GEN-018", "GEN-019", "GEN-020")


@pytest.fixture(scope="module")
def v2():
    return sp.load_splits(CONFIG / "perception_splits.json")


@pytest.fixture(scope="module")
def v3():
    return sp.load_splits(CONFIG / "perception_splits_v3.json")


def test_version_two_is_untouched_and_still_matches_the_catalogue_view_it_froze(v2):
    assert v2["sha256"] == V2_SHA256 and v2["schema_version"] == 2
    products = ds.load_generated_catalogue(CATALOGUE_PATH)
    assert not any(row["shape"] == "box" for row in products)
    assert v2["catalogue_sha256"] == ds.catalogue_sha256(products)


def test_version_three_preserves_every_assignment_and_pins_the_catalogue_with_boxes(v2, v3):
    assert v3["schema_version"] == 3 and v3["extension"]["base_sha256"] == V2_SHA256
    for kind in ("products", "seeds"):
        for name in sp.SPLIT_NAMES:
            assert set(v2[kind][name]) <= set(v3[kind][name])
    assert v3["held_out_generations"]["burned"] == v2["held_out_generations"]["burned"]
    assert set(v2["held_out_generations"]["fresh"]) <= set(v3["held_out_generations"]["fresh"])
    products = ds.load_generated_catalogue(CATALOGUE_PATH, include_boxes=True)
    assert v3["catalogue_sha256"] == ds.catalogue_sha256(products)
    assert v3["catalogue_sha256"] != v2["catalogue_sha256"]
    assert ds.load_catalogue_for_splits(CATALOGUE_PATH, v3) == products
    added = {sku for name in sp.SPLIT_NAMES for sku in v3["products"][name]} - {
        sku for name in sp.SPLIT_NAMES for sku in v2["products"][name]
    }
    assert added == set(BOX_SKUS)
    assert v3["box_seed_start"] == 235 and v3["extension"]["added_seed_range"] == [235, 284]
    assert max(v2["seeds"]["held_out"]) == 234  # the burned/fresh 185..234 block is not reused


def test_the_chain_reproduces_and_a_tampered_file_is_refused(v3):
    assert sp.reproduce_extension(v3) == v3
    assert sp.validate_splits(v3) == []
    broken = copy.deepcopy(v3)
    broken["seeds"]["train"].remove(broken["seeds"]["train"][-1])
    assert sp.validate_splits(broken)
    stolen = copy.deepcopy(v3)
    stolen["box_seed_start"] = 200
    stolen["sha256"] = sp.body_hash(stolen)
    assert any("box_seed_start" in problem for problem in sp.validate_splits(stolen))


def test_a_barcode_less_box_is_held_out_and_no_lookalike_pair_is_split_across_held_out(v3):
    held = set(v3["products"]["held_out"])
    assert "GEN-020" in held
    rows = {r["sku"]: r for r in ds.load_generated_catalogue(CATALOGUE_PATH, include_boxes=True)}
    assert rows["GEN-020"]["barcode"] is False and "gtin" not in rows["GEN-020"]
    assert len({v3["lookalike_groups"][s] for s in held if v3["lookalike_groups"][s]}) == len(
        [s for s in held if v3["lookalike_groups"][s]]
    )


def test_old_seeds_draw_the_scenes_they_always_drew(v2, v3):
    old = ds.load_generated_catalogue(CATALOGUE_PATH)
    new = ds.load_catalogue_for_splits(CATALOGUE_PATH, v3)
    for seed in (1, 2, 4, 26, 100, 184, 185, 234):
        a = cli.scene_for_seed(seed, old, v2)["scenario"]
        b = cli.scene_for_seed(seed, new, v3)["scenario"]
        assert json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True), seed
        assert not sp.scene_includes_boxes(v3, seed)


def test_new_seeds_draw_boxes_and_train_seeds_never_draw_a_held_out_product(v3):
    products = ds.load_catalogue_for_splits(CATALOGUE_PATH, v3)
    held = set(v3["products"]["held_out"])
    box_scenes = 0
    for seed in range(235, 285):
        scene = cli.scene_for_seed(seed, products, v3)
        skus = {p["sku"] for p in scene["scenario"]["products"]}
        boxes = [p for p in scene["scenario"]["products"] if p["product_class"] == "box"]
        box_scenes += bool(boxes)
        if scene["seed_split"] != "held_out":
            assert not skus & held, seed
        for product in boxes:
            assert product["visual_mesh"].startswith("box_") and product["label_texture"]
            assert product["geometry_key"].startswith("box.gen")
    assert box_scenes >= 40


def test_box_tray_products_stand_or_lie_on_the_tray_at_the_right_height(v3):
    products = ds.load_catalogue_for_splits(CATALOGUE_PATH, v3)
    standing = lying = 0
    for seed in range(235, 285):
        scene = cli.scene_for_seed(seed, products, v3)
        for product in scene["scenario"]["products"]:
            if product["product_class"] != "box" or not product["model_name"].startswith("stock_"):
                continue
            size = next(
                o["size_xyz_m"]
                for o in scene["truth"]["objects"]
                if o["object_id"] == product["source_object_id"]
            )
            pose = product["spawn_pose"]
            if abs(pose[3]) > 1.0:
                lying += 1
                assert pose[2] == pytest.approx(ds.TRAY_SURFACE_Z_M + size[1] / 2)
            else:
                standing += 1
                assert pose[2] == pytest.approx(ds.TRAY_SURFACE_Z_M + size[2] / 2)
    assert standing and lying


def test_proposal_box_scenes_are_valid_and_their_truth_carries_the_size(v3):
    products = ds.load_catalogue_for_splits(CATALOGUE_PATH, v3)
    assert pp.split_of_proposal_seed(1101) == "development"
    assert pp.scene_type_of_seed(1001) == pp.SCENE_TYPES[1001 % 6]  # old seeds keep their type
    kinds = set()
    for seed in pp.BOX_DEVELOPMENT_SEEDS:
        scene = pp.build_proposal_scene(seed, products)
        kinds.add(scene["tray_kind"])
        boxes = [o for o in scene["truth"]["objects"] if o["product_class"] == "box"]
        assert boxes and all(len(o["size_xyz_m"]) == 3 for o in boxes)
    assert kinds == {"box"}


def test_box_ray_cast_matches_the_analytic_box():
    # A box 0.1 x 0.06 x 0.15 at the origin, camera 1 m away on -y looking at +y: the first hit
    # along the optical axis is the front face at 1 - 0.03.
    obj = {
        "product_class": "box",
        "size_xyz_m": [0.1, 0.06, 0.15],
        "pose_world": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    }
    body = scorer.cylinder_of(obj)
    rays = np.zeros((1, 3, 3))
    rays[0, :, 1] = 1.0  # all along +y
    rays[0, 0, 0], rays[0, 2, 0] = -0.0, 0.2  # one ray misses to the side
    rays[0, 0, 0] = 0.0
    t = scorer.cylinder_depth(rays, np.array([0.0, -1.0, 0.0]), body, (slice(0, 1), slice(0, 3)))
    assert t[0, 0] == pytest.approx(0.97) and t[0, 1] == pytest.approx(0.97)
    assert math.isinf(t[0, 2])
    assert scorer.shape_of(obj)[1] == 0.15
