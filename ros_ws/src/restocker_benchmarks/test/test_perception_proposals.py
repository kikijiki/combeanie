# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Proposal benchmark: scenes, survey views, analytic ground truth, scoring and the bounds rule."""
# Pure Python. The proposer cases render a synthetic depth image from the same cylinder model the
# ground truth uses, so they test the proposer and the scorer against each other on a scene whose
# answer is known exactly; they need cv2 and skip without it.

import json
import math
import os
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_gazebo"))
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "restocker_perception"))

from restocker_benchmarks import perception_dataset as dataset  # noqa: E402,I100,I101
from restocker_benchmarks import perception_proposal_scorer as scorer  # noqa: E402
from restocker_benchmarks import perception_proposals as scenes  # noqa: E402
from restocker_benchmarks.perception_views import look_at_optical  # noqa: E402

CATALOGUE = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        str(
            Path(__file__).resolve().parents[2]
            / "restocker_gazebo"
            / "config"
            / "generated_product_labels.yaml"
        ),
    )
)
BOUNDS = (
    Path(__file__).resolve().parents[2]
    / "restocker_perception"
    / "config"
    / "proposal_acceptance_bounds.json"
)


def products():
    return yaml.safe_load(CATALOGUE.read_text(encoding="utf-8"))["products"]


def test_seed_ranges_are_disjoint_from_each_other_and_from_the_identity_benchmark():
    development, held_out = set(scenes.DEVELOPMENT_SEEDS), set(scenes.HELD_OUT_SEEDS)
    assert not development & held_out
    assert not (development | held_out) & set(range(1, 235))
    assert scenes.split_of_proposal_seed(1001) == "development"
    assert scenes.split_of_proposal_seed(2048) == "held_out"
    with pytest.raises(ValueError):
        scenes.split_of_proposal_seed(7)


def test_every_scene_type_is_balanced_in_each_seed_range():
    for seeds in (scenes.DEVELOPMENT_SEEDS, scenes.HELD_OUT_SEEDS):
        counts = {kind: 0 for kind in scenes.SCENE_TYPES}
        for seed in seeds:
            counts[scenes.scene_type_of_seed(seed)] += 1
        assert len(set(counts.values())) == 1


def test_scenes_are_pure_in_the_seed():
    first = scenes.build_proposal_scene(1003, products())
    second = scenes.build_proposal_scene(1003, products())
    assert first["scenario"] == second["scenario"]
    assert first["truth"] == second["truth"]
    assert first["scenario"] != scenes.build_proposal_scene(1009, products())["scenario"]


def test_tray_products_are_pinned_inside_the_tray_and_never_overlap():
    for seed in (*scenes.DEVELOPMENT_SEEDS, *scenes.HELD_OUT_SEEDS):
        scene = scenes.build_proposal_scene(seed, products())
        tray = [p for p in scene["scenario"]["products"] if p["model_name"].startswith("tray_")]
        assert tray, seed
        footprints = []
        for product in tray:
            assert product["static"] is True
            x, y, _, roll, _, yaw = product["spawn_pose"]
            lying = abs(roll) > 1.0
            axis_yaw = yaw + math.pi / 2.0 if lying else 0.0
            footprint = scenes.Footprint(product["product_class"], x, y, lying, axis_yaw)
            assert footprint.inside_tray(), (seed, product["model_name"])
            footprints.append(footprint)
        for index, a in enumerate(footprints):
            for b in footprints[index + 1 :]:
                assert a.gap_to(b) > -0.002, (seed, a.shape, b.shape)


def test_the_clustered_scene_types_really_touch():
    touching = 0
    for seed in scenes.DEVELOPMENT_SEEDS:
        if scenes.scene_type_of_seed(seed) not in ("dense", "rows"):
            continue
        scene = scenes.build_proposal_scene(seed, products())
        objects = [o for o in scene["truth"]["objects"] if o["lane_id"] is None]
        for obj in objects:
            gap = scorer.neighbour_gap(obj, objects)
            touching += gap < 0.003
    assert touching >= 8


def test_lying_and_contour_scenes_draw_what_they_name():
    lying = scenes.build_proposal_scene(1005, products())
    assert scenes.scene_type_of_seed(1005) == "lying"
    tray = [o for o in lying["truth"]["objects"] if o["lane_id"] is None]
    assert any(scorer.is_lying(o) for o in tray)
    contour = scenes.build_proposal_scene(1006, products())
    meshes = [p for p in contour["scenario"]["products"] if p.get("visual_mesh")]
    assert meshes and all(p["product_class"] == "contour_bottle" for p in meshes)


def test_scenarios_load_through_the_shipped_validator(tmp_path):
    from restocker_gazebo import scenario_config  # noqa: PLC0415

    for seed in (1003, 1005, 1006, 1007):
        path = tmp_path / f"{seed}.yaml"
        path.write_text(
            yaml.safe_dump(scenes.build_proposal_scene(seed, products())["scenario"]),
            encoding="utf-8",
        )
        assert scenario_config.load_scenario(path)["products"]


def test_survey_views_are_the_shipped_stations_and_target_no_product():
    scene = scenes.build_proposal_scene(1003, products())
    views = scenes.survey_views(1003, scene["truth"])
    names = [v["name"] for v in views]
    assert names[:2] == ["tray_a", "tray_b"]
    lanes = {row["lane_id"] for row in scene["truth"]["lanes"] if row["count"] > 0}
    assert 0 < len(names[2:]) <= 2 and set(names[2:]) <= lanes
    for view in views:
        assert view["target_object_id"] is None and view["range_check"] is False
        assert len(view["camera_optical_pose"]) == 7
    # The tray stations look from the robot side (+y) down at the tray.
    assert views[0]["camera_optical_pose"][1] > scenes.TRAY_CENTRE_WORLD[1]
    assert scenes.survey_views(1003, scene["truth"]) == views


def camera(eye, aim, width=1280, height=720, fx=700.917):
    from restocker_perception.proposals import CameraModel  # noqa: PLC0415

    pose = [*eye, *look_at_optical(eye, aim)]
    return CameraModel(fx, fx, width / 2, height / 2, width, height, tuple(pose))


def label_object(name, shape, x, y, lying=False, yaw=0.0):
    radius, height = scorer.SHAPES[shape]
    if lying:
        z, roll = scenes.TRAY_SURFACE_Z_M + radius, math.pi / 2
    else:
        z, roll = scenes.TRAY_SURFACE_Z_M + height / 2, 0.0
    quaternion = dataset.quaternion_from_rpy(roll, 0.0, yaw)
    return {
        "object_id": f"sim:{name}",
        "sku": "GEN-001",
        "product_class": shape,
        "pose_world": [x, y, z, *quaternion],
        "lane_id": None,
    }


def label_box(name, variant, x, y, lying=False, yaw=0.0):
    """A box label as the scene builder poses it: standing, or on its back (front face up)."""
    size = scenes.BOX_CATALOGUE[f"box.{variant}"]["size_xyz_m"]
    if lying:
        z, roll, yaw = scenes.TRAY_SURFACE_Z_M + size[1] / 2.0, -math.pi / 2.0, yaw - math.pi / 2.0
    else:
        z, roll = scenes.TRAY_SURFACE_Z_M + size[2] / 2.0, 0.0
    return {
        "object_id": f"sim:{name}",
        "sku": "GEN-017",
        "product_class": "box",
        "size_xyz_m": list(size),
        "pose_world": [x, y, z, *dataset.quaternion_from_rpy(roll, 0.0, yaw)],
        "lane_id": None,
    }


def scored(cam, objects, found, depth):
    row = {
        "frame_id": "f",
        "scenario_id": "s",
        "sensor": "wrist",
        "viewpoint": "tray_a",
        "lighting": {"condition": "nominal"},
    }
    return scorer.score_frame(
        row, scorer.ground_truth(objects, cam, depth), found.labels, found.proposals
    )


def render(cam, objects):
    """Synthetic z-depth (m) of the tray plane and the labelled cylinders."""
    rays, origin = cam.rays_world()
    with np.errstate(divide="ignore", invalid="ignore"):
        depth = (scenes.TRAY_SURFACE_Z_M - origin[2]) / rays[..., 2]
    depth = np.where(depth > 0, depth, 0.0)
    best = np.where(depth > 0, depth, np.inf)
    for obj in objects:
        cylinder = scorer.cylinder_of(obj)
        window = (slice(0, cam.height), slice(0, cam.width))
        best = np.minimum(best, scorer.cylinder_depth(rays, origin, cylinder, window))
    return np.where(np.isfinite(best), best, 0.0)


EYE, AIM = (0.0, -0.35, 1.30), (0.0, -0.80, 0.745)


def test_analytic_masks_agree_with_the_depth_they_were_cast_from():
    cam = camera(EYE, AIM)
    objects = [label_object("a", "can", -0.1, -0.8), label_object("b", "large_bottle", 0.12, -0.8)]
    depth = render(cam, objects)
    truth = scorer.ground_truth(objects, cam, depth)
    for entry in truth:
        assert entry["visible_fraction"] == pytest.approx(1.0, abs=0.01)
        assert entry["depth_residual_mm"] < 0.001
        assert scorer.recallable(entry)


def test_an_occluded_product_loses_visible_fraction_and_the_front_one_keeps_it():
    cam = camera(EYE, AIM)
    objects = [
        label_object("behind", "can", 0.0, -0.9),
        label_object("front", "large_bottle", 0.0, -0.84),
    ]
    truth = scorer.ground_truth(objects, cam, render(cam, objects))
    behind, front = truth
    assert front["visible_fraction"] > 0.99
    assert behind["visible_fraction"] < 0.9


def test_something_nearer_than_the_model_is_not_the_products_mask():
    cam = camera(EYE, AIM)
    objects = [label_object("a", "can", 0.0, -0.8)]
    depth = render(cam, objects)
    mask = scorer.ground_truth(objects, cam, depth)[0]["mask"]
    blocked = depth.copy()
    blocked[mask & (np.arange(cam.width)[None, :] < 640)] = 0.2  # an arm link in front, left half
    truth = scorer.ground_truth(objects, cam, blocked)[0]
    assert 0.2 < truth["visible_fraction"] < 0.8


def test_the_proposer_finds_separate_products_on_known_depth():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import ProposalConfig, propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [
        label_object("a", "can", -0.30, -0.80),
        label_object("b", "small_bottle", -0.10, -0.80),
        label_object("c", "large_bottle", 0.12, -0.85),
        label_object("d", "can", 0.28, -0.80),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam, config=ProposalConfig())
    truth = scorer.ground_truth(objects, cam, depth)
    record = scorer.score_frame(
        {
            "frame_id": "f",
            "scenario_id": "s",
            "sensor": "wrist",
            "viewpoint": "tray_a",
            "lighting": {"condition": "nominal"},
        },
        truth,
        found.labels,
        found.proposals,
    )
    assert record["objects"], "no recallable product in the synthetic frame"
    assert all(r["recalled"] for r in record["objects"]), record["objects"]
    assert not any(p["foreign"] for p in record["proposals"])
    kinds = {p.kind for p in found.proposals}
    assert kinds == {"disc"}
    classes = sorted(p.catalogue_class for p in found.proposals)
    assert classes == ["can", "can", "large_bottle", "small_bottle"]


def test_touching_products_of_equal_height_are_split_into_discs():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [
        label_object("a", "can", 0.0, -0.8),
        label_object("b", "can", 0.0662, -0.8),
        label_object("c", "can", 0.0, -0.8662),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam)
    assert [p.kind for p in found.proposals] == ["disc", "disc", "disc"]
    truth = scorer.ground_truth(objects, cam, depth)
    row = {
        "frame_id": "f",
        "scenario_id": "s",
        "sensor": "wrist",
        "viewpoint": "tray_a",
        "lighting": {"condition": "nominal"},
    }
    record = scorer.score_frame(row, truth, found.labels, found.proposals)
    assert all(r["recalled"] for r in record["objects"]), record["objects"]
    assert not any(p["merged"] for p in record["proposals"])


def test_a_lying_product_is_a_blob_not_a_disc():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [label_object("lying", "small_bottle", 0.0, -0.8, lying=True, yaw=0.3)]
    found = propose(render(cam, objects), cam)
    assert [p.kind for p in found.proposals] == ["blob"]
    assert found.proposals[0].catalogue_class is None


def test_a_lying_product_between_two_upright_ones_does_not_swallow_them():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [
        label_object("left", "can", -0.17, -0.8),
        label_object("lying", "small_bottle", 0.0, -0.8, lying=True, yaw=0.0),
        label_object("right", "can", 0.1366, -0.8),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam)
    truth = scorer.ground_truth(objects, cam, depth)
    row = {
        "frame_id": "f",
        "scenario_id": "s",
        "sensor": "wrist",
        "viewpoint": "tray_a",
        "lighting": {"condition": "nominal"},
    }
    record = scorer.score_frame(row, truth, found.labels, found.proposals)
    assert len(record["objects"]) == 3
    assert all(r["recalled"] for r in record["objects"]), (record["objects"], found.proposals)


@pytest.mark.parametrize("yaw", [0.0, 0.4, 1.2])
def test_a_standing_carton_is_a_rectangle_not_a_can(yaw):
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [label_box("carton", "carton_200", 0.05, -0.8, yaw=yaw)]
    depth = render(cam, objects)
    found = propose(depth, cam)
    assert [p.kind for p in found.proposals] == ["rect"]
    proposal = found.proposals[0]
    assert proposal.catalogue_class is None and proposal.shape_class == "box"
    long_side, short_side = proposal.size_m
    assert long_side == pytest.approx(0.065, abs=0.012) and short_side == pytest.approx(0.04, abs=0.012)
    assert all(r["recalled"] for r in scored(cam, objects, found, depth)["objects"])


def test_a_round_top_stays_a_disc_next_to_a_box():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [
        label_object("can", "can", -0.15, -0.8),
        label_box("carton", "carton_1000", 0.1, -0.8, yaw=0.5),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam)
    assert sorted(p.kind for p in found.proposals) == ["disc", "rect"]
    assert [p.catalogue_class for p in found.proposals if p.kind == "disc"] == ["can"]
    assert all(r["recalled"] for r in scored(cam, objects, found, depth)["objects"])


def test_a_low_box_lying_beside_a_tall_bottle_is_one_rectangle():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    # A slim box on its back is 25 mm high: one rectangle, not a disc and two blobs.
    objects = [
        label_object("tall", "large_bottle", -0.1, -0.8),
        label_box("slim", "slim_box", 0.0, -0.8, lying=True, yaw=0.6),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam)
    record = scored(cam, objects, found, depth)
    assert all(r["recalled"] for r in record["objects"]), (record["objects"], found.proposals)
    assert sorted(p.kind for p in found.proposals) == ["disc", "rect"]


def test_lying_boxes_end_to_end_are_two_rectangles():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    # Two slim boxes on their backs, end to end and slightly skewed, as the scene builder's
    # capsule packing lays them: one plateau of equal height with a waist where they meet.
    objects = [
        label_box("a", "slim_box", -0.07, -0.8, lying=True, yaw=0.0),
        label_box("b", "slim_box", 0.075, -0.79, lying=True, yaw=0.5),
    ]
    depth = render(cam, objects)
    found = propose(depth, cam)
    record = scored(cam, objects, found, depth)
    assert len(record["objects"]) == 2
    assert all(r["recalled"] for r in record["objects"]), (record["objects"], found.proposals)


def test_a_row_of_touching_round_tops_is_not_a_rectangle():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    objects = [label_object(f"c{i}", "can", -0.0662 + 0.0662 * i, -0.8) for i in range(3)]
    depth = render(cam, objects)
    found = propose(depth, cam)
    kinds = [p.kind for p in found.proposals]
    assert "rect" not in kinds and kinds.count("disc") >= 3
    assert all(r["recalled"] for r in scored(cam, objects, found, depth)["objects"])


def test_a_lying_cylinder_is_not_mistaken_for_a_box():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    for shape in ("can", "large_bottle"):
        objects = [label_object("lying", shape, 0.0, -0.8, lying=True, yaw=0.9)]
        found = propose(render(cam, objects), cam)
        assert [p.kind for p in found.proposals] == ["blob"]


def test_an_empty_tray_proposes_nothing():
    pytest.importorskip("cv2")
    from restocker_perception.proposals import propose  # noqa: PLC0415

    cam = camera(EYE, AIM)
    assert propose(render(cam, []), cam).proposals == ()


def record(recalled, scene, **extra):
    base = {
        "scene": scene,
        "recalled": recalled,
        "loose": recalled,
        "iou": 0.8 if recalled else None,
        "pose": "upright",
        "shape": "can",
        "gap": "separate",
        "occlusion": "clear",
        "surface": "tray",
        "tray_kind": "sparse",
        "view": "tray_a",
        "lighting": "nominal",
    }
    return {**base, **extra}


def test_cluster_rate_resamples_scenes_and_is_pinned():
    rows = [record(index % 10 != 0, f"s{index // 5}") for index in range(100)]
    first, second = scorer.cluster_rate(rows, "recalled"), scorer.cluster_rate(rows, "recalled")
    assert first == second
    assert first["value"] == pytest.approx(0.9)
    low, high = first["cluster_ci95"]
    assert low < 0.9 < high and first["scenes"] == 20


def bounds():
    return json.loads(BOUNDS.read_text(encoding="utf-8"))


def population(failing_stratum=None):
    rows, proposals = [], []
    kinds = ("dense", "rows", "tall_front", "lying", "contour", "sparse")
    for scene in range(60):
        kind = kinds[scene % len(kinds)]
        for index in range(10):
            extra = {"tray_kind": kind}
            if kind == "lying":
                extra["pose"] = "lying"
            if kind == "contour":
                extra["shape"] = "contour_bottle"
            if index == 0:
                extra["gap"] = "touching"
            if index == 1:
                extra["occlusion"] = "partial"
            if index == 2:
                extra["surface"] = "lane"
            ok = True
            if failing_stratum == "lying" and kind == "lying":
                ok = index < 5
            rows.append(record(ok, f"s{scene}", **extra))
        proposals.append({"foreign": False, "matched": True, "merged": False, "kind": "disc"})
    return rows, proposals


def test_bounds_rule_declares_geometry_sufficient_only_if_every_bound_passes():
    rows, proposals = population()
    decision = scorer.decide(rows, proposals, [40.0] * 10, bounds())
    assert decision["verdict"] == "SUFFICIENT", decision["checks"]
    rows, proposals = population("lying")
    decision = scorer.decide(rows, proposals, [40.0] * 10, bounds())
    assert decision["verdict"] == "INSUFFICIENT"
    assert decision["checks"]["hard: pose = lying"]["verdict"] == "fail"


def test_a_small_stratum_is_inconclusive_and_counts_as_not_sufficient():
    rows, proposals = population()
    rows = [r for r in rows if r["shape"] != "contour_bottle"] + [
        record(True, "x", shape="contour_bottle") for _ in range(5)
    ]
    decision = scorer.decide(rows, proposals, [40.0] * 10, bounds())
    assert decision["checks"]["hard: shape = contour_bottle"]["verdict"] == "inconclusive"
    assert decision["verdict"] == "INSUFFICIENT"


def test_foreign_proposals_and_latency_have_their_own_bounds():
    rows, proposals = population()
    proposals = proposals + [
        {"foreign": True, "matched": False, "merged": False, "kind": "blob"} for _ in range(20)
    ]
    decision = scorer.decide(rows, proposals, [900.0] * 10, bounds())
    assert decision["checks"]["foreign_proposal_rate"]["verdict"] == "fail"
    assert decision["checks"]["proposer_latency_p95_ms"]["verdict"] == "fail"
