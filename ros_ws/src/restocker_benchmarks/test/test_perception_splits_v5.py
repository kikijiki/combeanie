# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Splits version 5 (CMB-112 round 3): identity-round roles, burned generations, re-look views."""
# Version 5 appends seeds to version 4, marks every held-out seed an earlier identity round has
# read as burned, and records which catalogue products the round leaves unregistered. In this round
# held-out products are registered (the twin of a registered product must be), so the run guard
# must stop refusing them and start refusing the unregistered ones.

import copy
import importlib.util
import math
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_cli as cli  # noqa: E402,I100,I101
from restocker_benchmarks import perception_dataset as ds  # noqa: E402
from restocker_benchmarks import perception_splits as sp  # noqa: E402
from restocker_benchmarks import perception_views as views  # noqa: E402

HERE = Path(__file__).resolve().parent
CONFIG = HERE.parent / "config"
V4_SHA256 = "e848c385eff329f897561c7f757a9b10d52fd155e57dd53912cf5e0f1aa3c546"
CATALOGUE_PATH = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        HERE.parents[1] / "restocker_gazebo" / "config" / "generated_product_labels.yaml",
    )
)
RECORDER = HERE.parents[3] / "tools" / "record_perception_dataset.py"


@pytest.fixture(scope="module")
def v5():
    return sp.load_splits(CONFIG / "perception_splits_v5.json")


@pytest.fixture(scope="module")
def v4():
    return sp.load_splits(CONFIG / "perception_splits_v4.json")


def test_version_four_is_untouched(v4):
    assert v4["sha256"] == V4_SHA256


def test_version_four_preserves_every_assignment_and_reproduces(v4, v5):
    assert v5["schema_version"] == 5 and v5["extension"]["base_sha256"] == V4_SHA256
    for kind in ("products", "seeds"):
        for name in sp.SPLIT_NAMES:
            assert set(v4[kind][name]) <= set(v5[kind][name])
    assert v5["products"] == v4["products"]
    assert sp.reproduce_extension(v5) == v5
    assert sp.validate_splits(v5) == []


def test_every_held_out_seed_an_earlier_identity_round_read_is_burned(v4, v5):
    burned = set(v5["held_out_generations"]["burned"])
    fresh = set(v5["held_out_generations"]["fresh"])
    old = set(v4["seeds"]["held_out"])
    assert {s for s in old if s <= 234} <= burned
    assert not burned & fresh and burned | fresh == set(v5["seeds"]["held_out"])
    added = range(
        v5["extension"]["added_seed_range"][0], v5["extension"]["added_seed_range"][1] + 1
    )
    assert set(added) & set(v5["seeds"]["held_out"]) <= fresh
    assert sp.held_out_generation(v5, 100) == "burned"


def test_the_roles_partition_the_catalogue_and_leave_twins_of_registered_products_out(v5):
    roles = v5["identity_round"]
    assert set(roles["registered_products"]) | set(roles["unregistered_products"]) == set(
        v5["lookalike_groups"]
    )
    assert not set(roles["registered_products"]) & set(roles["unregistered_products"])
    groups = v5["lookalike_groups"]
    for sku in roles["unregistered_products"]:
        twins = [s for s, g in groups.items() if g == groups[sku] and s != sku]
        assert twins and all(t in roles["registered_products"] for t in twins)
    # held-out products are registered in this round, except the listed ones
    assert set(v5["products"]["held_out"]) - set(roles["unregistered_products"]) <= set(
        roles["registered_products"]
    )


def test_the_run_guard_accepts_held_out_products_and_refuses_the_unregistered_ones(v5):
    roles = v5["identity_round"]
    ok = {"registry_skus": roles["registered_products"], "training_seeds": v5["seeds"]["train"]}
    assert sp.check_run(v5, ok) == []
    bad = {"registry_skus": roles["registered_products"] + roles["unregistered_products"][:1]}
    assert any("unregistered" in p for p in sp.check_run(v5, bad))
    leak = {"calibration_seeds": v5["held_out_generations"]["fresh"][:1]}
    assert any("held-out seed" in p for p in sp.check_run(v5, leak))


def test_scenes_of_old_seeds_are_unchanged_by_version_four(v4, v5):
    products = ds.load_catalogue_for_splits(CATALOGUE_PATH, v5)
    for seed in (3, 120, 236, 250, 284, 300, 364):
        a = cli.scene_for_seed(seed, products, v4)
        b = cli.scene_for_seed(seed, products, v5)
        assert a["scenario"] == b["scenario"]
    new = v5["extension"]["added_seed_range"][0]
    held = [s for s in v5["seeds"]["held_out"] if s >= new][0]
    skus = {p["sku"] for p in cli.scene_for_seed(held, products, v5)["scenario"]["products"]}
    assert skus  # held-out scenes draw from the whole catalogue
    train = [s for s in v5["seeds"]["train"] if s >= new][0]
    allowed = set(v5["products"]["train"]) | set(v5["products"]["validation"])
    assert {
        p["sku"] for p in cli.scene_for_seed(train, products, v5)["scenario"]["products"]
    } <= allowed


def test_a_version_four_extension_refuses_a_bad_request(v4):
    with pytest.raises(ValueError):
        sp.extend_identity_round(
            v4,
            last_seed=300,
            seed_counts=(1, 1, 1),
            unregistered=["GEN-008"],
            burn_held_out_through=0,
        )
    with pytest.raises(ValueError):
        sp.extend_identity_round(
            v4,
            last_seed=300,
            seed_counts=(1, 1, 1),
            unregistered=["NOPE"],
            burn_held_out_through=0,
        )


def test_the_edited_file_is_detected(v5):
    broken = copy.deepcopy(v5)
    broken["identity_round"]["unregistered_products"] = []
    assert sp.validate_splits(broken)


# --- re-look views and the rotation oracle ----------------------------------------------------


def truth_with(ids):
    return {
        "objects": [
            {
                "object_id": oid,
                "lane_id": None,
                "product_class": "can",
                "pose_world": [0.1 * i, -0.5, 0.85, 0, 0, 0, 1],
            }
            for i, oid in enumerate(ids)
        ]
    }


def test_relook_views_are_pure_target_the_named_objects_and_never_repeat_an_earlier_look():
    truth = truth_with(["sim:stock_can_01", "sim:stock_can_02", "sim:stock_can_03"])
    one = views.relook_views(7, truth, ["sim:stock_can_02"], look=1, per_object=2)
    assert one == views.relook_views(7, truth, ["sim:stock_can_02"], look=1, per_object=2)
    assert len(one) == 2 and {v["target_object_id"] for v in one} == {"sim:stock_can_02"}
    two = views.relook_views(7, truth, ["sim:stock_can_02"], look=2, per_object=2)
    assert [v["camera_optical_pose"] for v in one] != [v["camera_optical_pose"] for v in two]
    first = views.close_views(7, truth, 3)
    assert all(
        v["camera_optical_pose"] not in [f["camera_optical_pose"] for f in first] for v in one
    )


def test_close_views_keep_the_poses_they_always_drew():
    truth = truth_with(["sim:stock_can_01", "sim:stock_can_02"])
    out = views.close_views(5, truth, 3)
    assert [v["name"] for v in out] == ["close00", "close01", "close02"]
    assert all(math.isfinite(x) for v in out for x in v["camera_optical_pose"])


def _recorder():
    spec = importlib.util.spec_from_file_location("record_perception_dataset", RECORDER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_oracle_turns_a_tray_product_in_place_and_refuses_a_lane_product(v5):
    recorder = _recorder()
    products = ds.load_catalogue_for_splits(CATALOGUE_PATH, v5)
    seed = 236
    scene = cli.scene_for_seed(seed, products, v5)
    tray = [o for o in scene["truth"]["objects"] if o["lane_id"] is None]
    lane = [o for o in scene["truth"]["objects"] if o["lane_id"] is not None]
    target = tray[0]
    before = {p["source_object_id"]: list(p["spawn_pose"]) for p in scene["scenario"]["products"]}
    turned = recorder.apply_yaw_offsets(scene, {target["object_id"]: 90.0})
    after = {p["source_object_id"]: list(p["spawn_pose"]) for p in turned["scenario"]["products"]}
    yaw = (after[target["object_id"]][5] - before[target["object_id"]][5]) % (2 * math.pi)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-5)
    assert after[target["object_id"]][:5] == before[target["object_id"]][:5]
    assert {k: v for k, v in after.items() if k != target["object_id"]} == {
        k: v for k, v in before.items() if k != target["object_id"]
    }
    new_pose = next(o for o in turned["truth"]["objects"] if o["object_id"] == target["object_id"])
    assert new_pose["pose_world"][:3] == target["pose_world"][:3]
    assert new_pose["pose_world"][3:] != target["pose_world"][3:]
    with pytest.raises(SystemExit):
        recorder.apply_yaw_offsets(
            cli.scene_for_seed(seed, products, v5), {lane[0]["object_id"]: 10}
        )
    with pytest.raises(SystemExit):
        recorder.apply_yaw_offsets(cli.scene_for_seed(seed, products, v5), {"sim:nothing": 10})
