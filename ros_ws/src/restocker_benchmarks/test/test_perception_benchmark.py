# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Perception benchmark: scenes, labels, dataset hashing, frozen splits, run guard and scorer."""
# No simulator. The frozen split file and the generated catalogue are read from the paths the
# build passes in RESTOCKER_TEST_*, so the committed split file is checked against the committed
# catalogue and fails if either changes alone.

import copy
import json
import math
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_cli as cli  # noqa: E402
from restocker_benchmarks import perception_dataset as ds  # noqa: E402
from restocker_benchmarks import perception_lighting as lt  # noqa: E402
from restocker_benchmarks import perception_scorer as sc  # noqa: E402
from restocker_benchmarks import perception_splits as sp  # noqa: E402
from restocker_benchmarks import perception_views as pv  # noqa: E402

HERE = Path(__file__).resolve().parent
ALL_SEEDS = tuple(range(1, 25))
CATALOGUE_PATH = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        HERE.parents[1] / "restocker_gazebo" / "config" / "generated_product_labels.yaml",
    )
)
SPLITS_PATH = Path(
    os.environ.get(
        "RESTOCKER_TEST_PERCEPTION_SPLITS", HERE.parent / "config" / "perception_splits.json"
    )
)

# A camera straight above the lanes looking down (optical z = world -Z), 3 m up.
OVERHEAD_POSE = [0.0, 0.3, 3.0, 1.0, 0.0, 0.0, 0.0]
INTRINSICS = {"width": 1280, "height": 960, "fx": 700.0, "fy": 700.0, "cx": 640.0, "cy": 480.0}
LIGHTING = {"condition": "nominal", "source": "test"}


@pytest.fixture(scope="module")
def products():
    return ds.load_generated_catalogue(CATALOGUE_PATH)


@pytest.fixture(scope="module")
def splits():
    return sp.load_splits(SPLITS_PATH)


def _labels(products, splits, seeds):
    rows = []
    for seed in seeds:
        scene = cli.scene_for_seed(seed, products, splits)
        rows.append(
            ds.frame_label(
                frame_id=f"{scene['scenario_id']}-overhead-000",
                scenario=scene["scenario_id"],
                seed=seed,
                seed_split=scene["seed_split"],
                sensor="overhead",
                index=0,
                truth=scene["truth"],
                camera_pose_world=OVERHEAD_POSE,
                intrinsics=INTRINSICS,
                lighting=LIGHTING,
                files={},
            )
        )
    return rows


# ---- scenes and labels ---------------------------------------------------------------------


def test_scene_is_a_pure_function_of_the_seed(products, splits):
    first = cli.scene_for_seed(3, products, splits)
    again = cli.scene_for_seed(3, products, splits)
    other = cli.scene_for_seed(4, products, splits)
    assert ds.canonical_json(first["scenario"]) == ds.canonical_json(again["scenario"])
    assert ds.canonical_json(first["scenario"]) != ds.canonical_json(other["scenario"])


def test_truth_lane_order_count_and_misplaced_are_consistent(products, splits):
    found_misplaced = False
    for seed in range(1, 25):
        truth = cli.scene_for_seed(seed, products, splits)["truth"]
        for lane in truth["lanes"]:
            members = [o for o in truth["objects"] if o["lane_id"] == lane["lane_id"]]
            assert lane["count"] == len(members) == len(lane["skus_in_order"])
            assert [
                o["lane_order"] for o in sorted(members, key=lambda o: o["lane_order"])
            ] == list(range(len(members)))
            flagged = [o for o in members if o["misplaced"]]
            assert lane["misplaced_count"] == len(flagged)
            for member in flagged:
                assert member["sku"] != lane["expected_sku"]
            found_misplaced |= bool(flagged)
        for obj in truth["objects"]:
            if obj["lane_id"] is None:
                assert obj["misplaced"] is False and obj["lane_order"] is None
    assert found_misplaced, "no seed produced a misplaced product; the draw is dead"


def test_non_held_out_seeds_never_contain_held_out_products(products, splits):
    held = set(splits["products"]["held_out"])
    seen_held_out_in_held_seed = False
    for seed in range(1, 25):
        skus = {o["sku"] for o in cli.scene_for_seed(seed, products, splits)["truth"]["objects"]}
        if seed in splits["seeds"]["held_out"]:
            seen_held_out_in_held_seed |= bool(skus & held)
        else:
            assert not skus & held, f"seed {seed} leaks a held-out product"
    assert seen_held_out_in_held_seed


def test_shipped_scenarios_yield_truth():
    # The existing sensor-acceptance scenario works without any seeding.
    from restocker_gazebo.lane_columns import sensor_acceptance_scenario

    truth = ds.scene_truth(sensor_acceptance_scenario())
    assert len(truth["objects"]) == 3
    assert all(o["lane_id"] is None for o in truth["objects"])


def test_projection_of_a_point_below_the_camera():
    inside = ds.project_point([0.0, 0.3, 0.0], OVERHEAD_POSE, INTRINSICS)
    assert inside["in_view"] and inside["u"] == pytest.approx(640.0)
    assert inside["v"] == pytest.approx(480.0)
    assert inside["depth_m"] == pytest.approx(3.0, abs=1e-3)
    behind = ds.project_point([0.0, 0.3, 6.0], OVERHEAD_POSE, INTRINSICS)
    assert behind["in_view"] is False
    off = ds.project_point([5.0, 0.3, 0.0], OVERHEAD_POSE, INTRINSICS)
    assert off["in_view"] is False


# ---- dataset directory ---------------------------------------------------------------------


def _publish(tmp_path, products, splits, seeds, image_byte=0, name="out"):
    import numpy

    builder = ds.DatasetBuilder(tmp_path / f"stage-{name}")
    rows = _labels(products, splits, seeds)
    for seed, row in zip(seeds, rows, strict=True):
        scene = cli.scene_for_seed(seed, products, splits)
        builder.add_scenario(scene["scenario_id"], scene["scenario"], {"seed": seed})
        pixels = numpy.full((4, 5, 3), image_byte, dtype=numpy.uint8)
        row["files"] = {"rgb": builder.add_image(f"{row['frame_id']}.png", ds.encode_png(pixels))}
        builder.add_label(row)
    return builder.publish(
        tmp_path / name,
        seeds=seeds,
        world_sdf=HERE.parents[1] / "restocker_gazebo" / "worlds" / "restocking.sdf",
        repository=HERE,
        catalogue_hash=ds.catalogue_sha256(products),
        splits_hash=splits["sha256"],
    )


def test_dataset_is_content_hashed_and_reproducible(tmp_path, products, splits):
    first = _publish(tmp_path, products, splits, [1, 3], name="a")
    second = _publish(tmp_path, products, splits, [1, 3], name="b")
    changed = _publish(tmp_path, products, splits, [1, 3], image_byte=9, name="c")
    assert first.name == second.name and first.name != changed.name
    assert ds.verify_dataset(first) == []
    manifest = json.loads((first / "manifest.json").read_text())
    assert manifest["seeds"] == [1, 3]
    assert manifest["world_sha256"] and manifest["git"]["sha"] and manifest["splits_sha256"]
    # Renderer-independent label content hash does not move with the pixels.
    assert (
        manifest["labels_hash"]
        == json.loads((changed / "manifest.json").read_text())["labels_hash"]
    )
    assert (
        manifest["dataset_hash"]
        != json.loads((changed / "manifest.json").read_text())["dataset_hash"]
    )


def test_verify_detects_tampering(tmp_path, products, splits):
    path = _publish(tmp_path, products, splits, [2])
    labels = path / "labels.jsonl"
    labels.write_text(labels.read_text().replace("GEN-", "GEN_", 1))
    assert any("labels.jsonl" in p for p in ds.verify_dataset(path))


def test_png_round_trip():
    import numpy

    rgb = numpy.arange(4 * 5 * 3, dtype=numpy.uint8).reshape(4, 5, 3)
    depth = (numpy.arange(20, dtype=numpy.uint16).reshape(4, 5) * 301).astype(numpy.uint16)
    assert (ds.decode_png(ds.encode_png(rgb)) == rgb).all()
    assert (ds.decode_png(ds.encode_png(depth)) == depth).all()
    assert ds.encode_png(rgb) == ds.encode_png(rgb)


def test_world_lighting_is_read_from_the_world_file():
    lighting = ds.world_lighting(
        HERE.parents[1] / "restocker_gazebo" / "worlds" / "restocking.sdf"
    )
    assert lighting["lights"][0]["name"] == "workcell_sun"
    assert lighting["ambient"] == [0.35, 0.35, 0.35, 1.0]


WORLD_PATH = HERE.parents[1] / "restocker_gazebo" / "worlds" / "restocking.sdf"
WRIST_INTRINSICS = {
    "width": 1280,
    "height": 720,
    "fx": 700.0,
    "fy": 700.0,
    "cx": 640.0,
    "cy": 360.0,
}


# ---- seeded lighting -----------------------------------------------------------------------


def test_lighting_is_a_pure_function_of_the_seed_and_covers_every_condition():
    assert [lt.lighting_for_seed(s) for s in range(1, 9)] == [
        lt.lighting_for_seed(s) for s in range(1, 9)
    ]
    seen = {lt.lighting_for_seed(s)["condition"] for s in range(1, 200)}
    assert seen == set(lt.CONDITION_NAMES)
    assert len(lt.CONDITION_NAMES) >= 5 and "nominal" in seen


def test_nominal_condition_reproduces_the_shipped_world():
    shipped = ds.world_lighting(WORLD_PATH)
    nominal = lt.CONDITIONS["nominal"]
    assert shipped["ambient"][:3] == nominal["ambient"]
    sun = shipped["lights"][0]
    assert sun["diffuse"][:3] == nominal["diffuse"]
    assert sun["specular"][:3] == nominal["specular"]
    assert sun["direction"] == nominal["direction"]


def test_applied_lighting_really_changes_the_world_and_is_read_back(tmp_path):
    text = WORLD_PATH.read_text(encoding="utf-8")
    seen = set()
    for seed in range(1, 40):
        spec = lt.lighting_for_seed(seed)
        path = tmp_path / f"w{seed}.sdf"
        path.write_text(lt.apply_lighting(text, spec), encoding="utf-8")
        record = ds.world_lighting(path, spec["condition"], spec["intensity_scale"])
        assert record["ambient"][:3] == pytest.approx(spec["ambient"], abs=1e-5)
        assert record["lights"][0]["diffuse"][:3] == pytest.approx(spec["diffuse"], abs=1e-5)
        assert record["lights"][0]["direction"] == pytest.approx(spec["direction"], abs=1e-5)
        seen.add((tuple(record["ambient"]), tuple(record["lights"][0]["diffuse"])))
        if spec["condition"] == "directional":
            # A low sun: long shadows mean a shallow elevation.
            x, y, z = record["lights"][0]["direction"]
            assert abs(z) / math.hypot(x, y) < 0.35
    assert len(seen) > 20
    # Everything but the lighting survives: the physics comment and the other world content.
    assert "DART's default narrowphase" in lt.apply_lighting(text, lt.lighting_for_seed(1))
    assert "gz::sim::systems::Sensors" in lt.apply_lighting(text, lt.lighting_for_seed(1))


def test_lighting_does_not_decide_which_products_appear(products, splits):
    for seed in (1, 2, 3, 5, 8):
        before = cli.scene_for_seed(seed, products, splits)
        lt.lighting_for_seed(seed)
        again = cli.scene_for_seed(seed, products, splits)
        assert before["scenario"] == again["scenario"]


def test_lighting_is_in_the_label_hash_and_rows():
    base = {
        "scenario_id": "s",
        "seed": 1,
        "sensor": "wrist",
        "objects": [],
        "lanes": [],
        "lighting": {"condition": "dim", "intensity_scale": 1.0},
    }
    other = dict(base, lighting={"condition": "warm", "intensity_scale": 1.0})
    assert ds.label_content_hash_row(base) != ds.label_content_hash_row(other)


# ---- close-range wrist viewpoints ----------------------------------------------------------


def _first_scene_with_tray(products, splits):
    for seed in range(1, 25):
        scene = cli.scene_for_seed(seed, products, splits)
        if any(o["lane_id"] is None for o in scene["truth"]["objects"]):
            return seed, scene
    raise AssertionError("no seed has a tray product")


def test_close_views_are_deterministic_and_hold_a_product_within_reading_range(products, splits):
    checked = 0
    for seed in range(1, 25):
        scene = cli.scene_for_seed(seed, products, splits)
        views = pv.close_views(seed, scene["truth"])
        assert views == pv.close_views(seed, scene["truth"])
        tray = {o["object_id"] for o in scene["truth"]["objects"] if o["lane_id"] is None}
        assert bool(views) == bool(tray)
        for view in views:
            assert view["target_object_id"] in tray
            truth_row = next(
                o for o in scene["truth"]["objects"] if o["object_id"] == view["target_object_id"]
            )
            projection = ds.project_point(
                truth_row["pose_world"][:3], view["camera_optical_pose"], WRIST_INTRINSICS
            )
            assert projection["in_view"]
            assert pv.RANGE_LIMITS_M[0] <= projection["depth_m"] <= pv.RANGE_LIMITS_M[1] + 0.06
            assert view["camera_optical_pose"][2] <= pv.MAX_EYE_Z_M + 1e-6
            quaternion = view["camera_optical_pose"][3:]
            assert sum(c * c for c in quaternion) == pytest.approx(1.0, abs=1e-6)
            checked += 1
    assert checked >= 24


def test_close_view_rows_label_range_and_pixel_from_the_view_pose(products, splits):
    seed, scene = _first_scene_with_tray(products, splits)
    view = pv.close_views(seed, scene["truth"])[0]
    row = ds.frame_label(
        frame_id="f",
        scenario=scene["scenario_id"],
        seed=seed,
        seed_split=scene["seed_split"],
        sensor="wrist",
        index=2,
        truth=scene["truth"],
        camera_pose_world=view["camera_optical_pose"],
        intrinsics=WRIST_INTRINSICS,
        lighting=LIGHTING,
        files={},
        viewpoint=view["name"],
    )
    assert row["viewpoint"] == view["name"]
    target = next(o for o in row["objects"] if o["object_id"] == view["target_object_id"])
    assert target["in_view"] and 0.25 <= target["range_m"] <= 0.51
    assert 0 <= target["pixel"]["u"] < 1280 and 0 <= target["pixel"]["v"] < 720
    assert ds.label_content_hash_row(row)["objects"]


# ---- splits and the run guard --------------------------------------------------------------


def test_committed_splits_are_frozen_and_reproducible(products, splits):
    assert splits["catalogue_sha256"] == ds.catalogue_sha256(products)
    again = sp.freeze_splits(
        products,
        list(sp.seeds_range(24)),
        split_seed=splits["split_seed"],
        catalogue_sha256=ds.catalogue_sha256(products),
    )
    # The committed file is the version-2 extension; the version-1 file it extends (embedded in
    # it) is what the original seeds freeze to. test_perception_splits_v2 covers the extension.
    base = splits
    while base["schema_version"] == 2:
        base = base["extension"]["base_document"]
    assert again == base
    assert sp.validate_splits(splits) == []


def test_split_invariants(products, splits):
    train, validation, held = (set(splits["products"][n]) for n in sp.SPLIT_NAMES)
    assert train and validation and held
    assert not (train & validation or train & held or validation & held)
    assert train | validation | held == {row["sku"] for row in products}
    groups = splits["lookalike_groups"]
    twins_of_held = {s for s in train | validation if groups[s] in {groups[h] for h in held}}
    assert twins_of_held, "every held-out product should have a registered look-alike twin"
    assert len({groups[h] for h in held}) == len(held)


def test_editing_the_split_file_is_detected(splits):
    tampered = copy.deepcopy(splits)
    tampered["products"]["held_out"] = tampered["products"]["held_out"][:-1]
    assert any("sha256" in p for p in sp.validate_splits(tampered))


def test_guard_refuses_held_out_product_in_registry(splits):
    held = splits["products"]["held_out"][0]
    problems = sp.check_run(splits, {"registry_skus": ["GEN-001", held]})
    assert any(held in p for p in problems)
    with pytest.raises(sp.SplitViolation):
        sp.require_clean_run(splits, {"registry_skus": [held]})


@pytest.mark.parametrize("key", ["training_seeds", "calibration_seeds", "registry_source_seeds"])
def test_guard_refuses_held_out_seeds(splits, key):
    seed = splits["seeds"]["held_out"][0]
    assert any(str(seed) in p for p in sp.check_run(splits, {key: [seed, 1]}))


def test_guard_refuses_a_different_split_file(splits):
    assert sp.check_run(splits, {"splits_sha256": "0" * 64})


def test_guard_accepts_a_clean_run(splits):
    run = {
        "registry_skus": sp.registered_skus(splits),
        "training_seeds": splits["seeds"]["train"],
        "calibration_seeds": splits["seeds"]["validation"],
        "splits_sha256": splits["sha256"],
    }
    assert sp.check_run(splits, run) == []


def test_cli_check_exit_codes(tmp_path, splits):
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({"registry_skus": splits["products"]["held_out"]}))
    good = tmp_path / "good.json"
    good.write_text(json.dumps({"registry_skus": sp.registered_skus(splits)}))
    assert cli.main(["check", "--splits", str(SPLITS_PATH), "--run", str(bad)]) == 2
    assert cli.main(["check", "--splits", str(SPLITS_PATH), "--run", str(good)]) == 0


# ---- scorer --------------------------------------------------------------------------------


def _truth_report(products, splits, predictions_fn=None, seeds=ALL_SEEDS):
    labels = _labels(products, splits, seeds)
    registry = sp.registered_skus(splits)
    groups = cli.lookalike_map(products)
    rows = (predictions_fn or (lambda lab, r, g: sc.ground_truth_predictions(lab)))(
        labels, registry, groups
    )
    return labels, sc.score(labels, rows, registry, lookalike_groups=groups)


def test_ground_truth_passthrough_is_perfect(products, splits):
    _labels_, report = _truth_report(products, splits)
    objects = report["overall"]["objects"]
    assert objects["n_objects"] > 20 and objects["n_unknown"] > 0
    # Held-out products are named by the passthrough, so they are false identities by design:
    # the passthrough predicts truth, which the registry cannot contain.
    assert objects["identity_accuracy"]["value"] == 1.0
    assert objects["false_identity_rate"]["k"] == objects["n_unknown"]
    assert objects["pose"]["translation_error_m"]["mean"] == pytest.approx(0.0, abs=1e-9)
    assert objects["pose"]["axis_error_deg"]["median"] == pytest.approx(0.0, abs=1e-3)
    lanes = report["overall"]["lanes"]
    assert lanes["count_exact"]["value"] == 1.0
    assert lanes["count_abs_error"]["mean"] == 0.0


def test_passthrough_with_unknown_rejection_has_no_false_identity(products, splits):
    def reject_unknown(labels, registry, groups):
        rows = sc.ground_truth_predictions(labels)
        for row in rows:
            if row["kind"] == "object" and row["sku"] not in registry:
                row["sku"] = sc.UNKNOWN_SKU
        return rows

    _l, report = _truth_report(products, splits, reject_unknown)
    objects = report["overall"]["objects"]
    assert objects["false_identity_rate"]["k"] == 0
    assert objects["unknown_rejection_rate"]["value"] == 1.0


def test_noisy_baseline_is_measurably_worse_and_in_the_expected_band(products, splits):
    def noisy(labels, registry, groups):
        return sc.noisy_predictions(
            labels,
            registry,
            seed=7,
            lookalike_groups=groups,
            abstain_probability=0.1,
            wrong_probability=0.2,
            drop_probability=0.05,
        )

    _l, report = _truth_report(products, splits, noisy)
    objects = report["overall"]["objects"]
    assert 0.5 < objects["identity_accuracy"]["value"] < 0.9
    assert objects["false_identity_rate"]["value"] > 0.05
    assert objects["abstention_rate"]["value"] > 0.0
    assert objects["miss_rate"]["value"] > 0.0
    assert objects["false_identity_twin_share"]["value"] > 0.5
    pose = objects["pose"]["translation_error_m"]
    assert 0.005 < pose["mean"] < 0.03
    assert pose["mean_ci95"][0] < pose["mean"] < pose["mean_ci95"][1]
    assert report["overall"]["lanes"]["count_exact"]["value"] < 1.0
    # Per-condition blocks exist for every key.
    names = report["by_condition"]
    assert "sensor=overhead" in names and "known=unknown" in names and "location=lane" in names


def test_silence_is_a_miss_not_an_abstention_and_not_a_false_identity(products, splits):
    labels = _labels(products, splits, [1, 2])
    report = sc.score(labels, [], sp.registered_skus(splits))
    objects = report["overall"]["objects"]
    assert objects["miss_rate"]["value"] == 1.0
    assert objects["false_identity_rate"]["k"] == 0
    assert objects["abstention_rate"]["k"] == 0


def test_wilson_interval_known_values():
    low, high = sc.wilson_interval(0, 10)
    assert low == 0.0 and 0.25 < high < 0.31
    low, high = sc.wilson_interval(5, 10)
    assert low == pytest.approx(0.2366, abs=1e-3) and high == pytest.approx(0.7634, abs=1e-3)
    assert sc.wilson_interval(0, 0) is None


def test_pose_matching_without_object_ids(products, splits):
    labels = _labels(products, splits, [1])
    rows = [dict(r) for r in sc.ground_truth_predictions(labels) if r["kind"] == "object"]
    for row in rows:
        row.pop("object_id")
    report = sc.score(labels, rows, sp.registered_skus(splits))
    assert report["overall"]["objects"]["miss_rate"]["k"] == 0
    assert report["spurious_predictions"] == 0


def test_axis_error_is_acute_and_ignores_yaw():
    upright = [0, 0, 0, 1]
    yawed = [0, 0, 0.7071068, 0.7071068]
    tilted = [0.2588190, 0, 0, 0.9659258]  # 30 degrees about x
    assert sc._axis_error_deg(upright, yawed) == pytest.approx(0.0, abs=1e-3)
    assert sc._axis_error_deg(upright, tilted) == pytest.approx(30.0, abs=1e-3)


@pytest.mark.parametrize(
    "row",
    [
        {"kind": "nope", "frame_id": "f"},
        {"kind": "object", "frame_id": "f", "sku": "X"},
        {"kind": "object", "frame_id": "f", "object_id": "o", "sku": 3},
        {"kind": "lane", "frame_id": "f", "lane_id": "lane_01", "count": -1},
        {"kind": "object", "object_id": "o"},
    ],
)
def test_malformed_predictions_are_rejected(row):
    with pytest.raises(sc.ScoreError):
        sc.validate_prediction(row, 1)


def test_cli_end_to_end(tmp_path, products, splits, capsys):
    path = _publish(tmp_path, products, splits, [3, 6, 7])
    predictions = tmp_path / "pred.jsonl"
    assert (
        cli.main(
            [
                "baseline",
                "--labels",
                str(path),
                "--splits",
                str(SPLITS_PATH),
                "--kind",
                "noisy",
                "--out",
                str(predictions),
            ]
        )
        == 0
    )
    report_json = tmp_path / "report.json"
    assert (
        cli.main(
            [
                "score",
                "--labels",
                str(path),
                "--predictions",
                str(predictions),
                "--splits",
                str(SPLITS_PATH),
                "--json",
                str(report_json),
            ]
        )
        == 0
    )
    assert "FALSE IDENTITY" in capsys.readouterr().out
    assert json.loads(report_json.read_text())["n_frames"] == 3
    assert cli.main(["verify", str(path)]) == 0
