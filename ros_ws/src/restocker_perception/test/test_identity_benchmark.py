# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity benchmark driver: dataset reading, calibration, the held-out guards."""
# A tiny synthetic dataset in the benchmark's public format: frames are 640x480 images with the
# registry's own views pasted at the labelled pixel, so the fake embedding can name them.

import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_perception.identity import benchmark as bm  # noqa: E402,I100,I101
from restocker_perception.registry import FakeEmbeddingBackend, Registry  # noqa: E402
from restocker_perception.registry import imageio as io  # noqa: E402
from restocker_perception.registry import render as rd  # noqa: E402
from restocker_perception.registry import simulated as sim  # noqa: E402

SRC = Path(__file__).resolve().parents[2]
CATALOGUE = Path(
    os.environ.get(
        "RESTOCKER_TEST_GENERATED_LABELS",
        SRC / "restocker_gazebo/config/generated_product_labels.yaml",
    )
)
GEOMETRY = Path(
    os.environ.get(
        "RESTOCKER_TEST_PRODUCT_CATALOG",
        SRC / "restocker_description/config/product_collision_catalog.yaml",
    )
)
SPLITS = Path(
    os.environ.get(
        "RESTOCKER_TEST_PERCEPTION_SPLITS",
        SRC / "restocker_benchmarks/config/perception_splits.json",
    )
)
BOUNDS = SRC / "restocker_perception/config/identity_acceptance_bounds_r2.json"
FX = 468.0
RANGE = 0.35


def texture(seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    image = np.tile(rng.uniform(0.1, 0.9, 3), (96, 192, 1)).astype(np.float32)
    for _ in range(10):
        x, y = int(rng.integers(0, 160)), int(rng.integers(0, 80))
        image[y : y + 12, x : x + 24] = rng.uniform(0, 1, 3)
    return image


@pytest.fixture(scope="module")
def world(tmp_path_factory):
    root = tmp_path_factory.mktemp("identity")
    splits = sim.load_split_file(SPLITS)
    rows = {r["sku"]: r for r in sim.load_catalogue_rows(CATALOGUE)}
    geometry = sim.load_geometry_catalogue(GEOMETRY)
    registry = Registry.create(root / "reg")
    views = {}
    for number, sku in enumerate(sorted(rows)[:12]):
        row = rows[sku]
        entry = geometry[row["geometry_key"]]
        views[sku] = rd.render_views(texture(number), entry["radius_m"], entry["height_m"])
        if sku in splits["products"]["held_out"]:
            continue
        registry.register(
            gtin=row["gtin"],
            display_name=f"{row['brand']} {row['flavour']} {row['kind']}",
            product_class=entry["product_class"],
            geometry_key=row["geometry_key"],
            radius_m=entry["radius_m"],
            height_m=entry["height_m"],
            images=views[sku],
            ocr_strings=[row["brand"], row["flavour"], row["kind"]],
            aliases=[sku],
        )

    def dataset(name: str, split: str, seeds: list[int], held: bool) -> Path:
        directory = root / name
        (directory / "frames").mkdir(parents=True)
        labels = []
        for seed in seeds:
            rng = np.random.default_rng(seed)
            for pose, sku in enumerate(sorted(rows)[:12]):
                if not held and sku in splits["products"]["held_out"]:
                    continue
                frame = np.full((480, 640, 3), 90, np.uint8)
                entry = geometry[rows[sku]["geometry_key"]]
                extent = max(2 * entry["radius_m"], entry["height_m"])
                side = int(FX * extent * 1.2 / RANGE)
                u, v = 320, 240
                patch = io.resize_area(views[sku][int(rng.integers(0, 8))], side, side)
                frame[
                    v - side // 2 : v - side // 2 + side, u - side // 2 : u - side // 2 + side
                ] = patch
                objects = [
                    {
                        "object_id": f"sim:{sku}",
                        "sku": sku,
                        "product_class": entry["product_class"],
                        "geometry_key": rows[sku]["geometry_key"],
                        "pixel": {"u": float(u), "v": float(v)},
                        "range_m": RANGE,
                        "in_view": True,
                    }
                ]
                frame_id = f"seed{seed:06d}-wrist-{pose:03d}"
                io.write_png(directory / "frames" / f"{frame_id}.png", frame)
                labels.append(
                    {
                        "frame_id": frame_id,
                        "scenario_id": f"seed{seed:06d}",
                        "seed": seed,
                        "seed_split": split,
                        "sensor": "wrist",
                        "camera": {
                            "intrinsics": {"fx": FX, "fy": FX, "cx": 320.0, "cy": 240.0},
                            "pose_world": [0.1 * pose, 0, 1, 0, 0, 0, 1],
                        },
                        "lighting": {"condition": "nominal"},
                        "files": {"rgb": f"frames/{frame_id}.png"},
                        "objects": objects,
                    }
                )
        (directory / "labels.jsonl").write_text("\n".join(json.dumps(r) for r in labels) + "\n")
        (directory / "manifest.json").write_text(json.dumps({"scenarios": []}))
        return directory

    return {
        "root": root,
        "registry": registry,
        "geometry": geometry,
        "train": dataset("train", "train", splits["seeds"]["train"][:3], False),
        "held_out": dataset(
            "held",
            "held_out",
            splits["held_out_generations"]["fresh"][:2]
            + splits["held_out_generations"]["burned"][:1],
            True,
        ),
    }


def test_groups_pair_views_of_one_object_and_refuse_other_splits(world):
    groups = list(bm.object_groups(world["train"], world["geometry"], allowed_splits=("train",)))
    assert groups and all(len(g.views) == 1 for g in groups)
    assert all(g.views[0].image.shape[0] >= 48 for g in groups)
    with pytest.raises(bm.ProtocolError, match="may read only"):
        list(
            bm.object_groups(
                world["held_out"], world["geometry"], allowed_splits=("train", "validation")
            )
        )


def test_calibrate_then_evaluate_once(world, tmp_path):
    registry, backend = world["registry"], FakeEmbeddingBackend()
    cal, whitener = tmp_path / "cal.json", tmp_path / "w.npz"
    result = bm.calibrate(
        [world["train"]],
        registry,
        backend,
        None,
        SPLITS,
        world["geometry"],
        out=cal,
        whitener_out=whitener,
        bounds_path=BOUNDS,
        whitening_components=8,
        progress=lambda _m: None,
    )
    assert result["fit"]["seed_splits_read"] == ["train", "validation"]
    assert result["fit"]["whitening"]["components"] == 8
    out = tmp_path / "held.json"
    report = bm.evaluate_held_out(
        [world["held_out"]],
        registry,
        backend,
        None,
        SPLITS,
        world["geometry"],
        calibration_path=cal,
        whitener_path=whitener,
        bounds_path=BOUNDS,
        out=out,
        progress=lambda _m: None,
    )
    fresh = report["fresh"]["multi_view"]
    assert fresh["n_unknown"] > 0 and fresh["n_known"] > 0
    assert report["n_burned_groups"] > 0 and "burned" in report  # reported, decides nothing
    assert set(fresh["channels"]) == {"embedding", "ocr", "barcode"}
    assert set(report["verdict"]) == {
        "false_identity_rate_known",
        "unknown_named_as_sku",
        "latency_p95_ms",
    }
    # a handful of objects can never demonstrate a bound: the sample-size rule says so
    assert report["verdict"]["unknown_named_as_sku"]["status"] in ("fail", "inconclusive")
    assert report["verdict"]["unknown_named_as_sku"]["sample_size_met"] is False
    # the held-out run happens once
    with pytest.raises(bm.ProtocolError, match="once"):
        bm.evaluate_held_out(
            [world["held_out"]],
            registry,
            backend,
            None,
            SPLITS,
            world["geometry"],
            calibration_path=cal,
            whitener_path=whitener,
            bounds_path=BOUNDS,
            out=out,
            progress=lambda _m: None,
        )
    # a calibration run is not allowed to read the held-out seeds
    with pytest.raises(bm.ProtocolError, match="may read only"):
        bm.calibrate(
            [world["held_out"]],
            registry,
            backend,
            None,
            SPLITS,
            world["geometry"],
            out=tmp_path / "c2.json",
            whitener_out=tmp_path / "w2.npz",
            bounds_path=BOUNDS,
            progress=lambda _m: None,
        )


def test_edited_bounds_after_calibration_are_refused(world, tmp_path):
    registry, backend = world["registry"], FakeEmbeddingBackend()
    cal, whitener = tmp_path / "cal.json", tmp_path / "w.npz"
    bm.calibrate(
        [world["train"]],
        registry,
        backend,
        None,
        SPLITS,
        world["geometry"],
        out=cal,
        whitener_out=whitener,
        bounds_path=BOUNDS,
        whitening_components=8,
        progress=lambda _m: None,
    )
    edited = tmp_path / "bounds.json"
    document = json.loads(BOUNDS.read_text())
    document["bounds"]["unknown_named_as_sku_max"] = 0.5
    edited.write_text(json.dumps(document))
    with pytest.raises(bm.ProtocolError, match="bounds"):
        bm.evaluate_held_out(
            [world["held_out"]],
            registry,
            backend,
            None,
            SPLITS,
            world["geometry"],
            calibration_path=cal,
            whitener_path=whitener,
            bounds_path=edited,
            out=tmp_path / "r.json",
            progress=lambda _m: None,
        )


def test_statistics_helpers():
    low, high = bm.wilson(0, 50)
    assert low < 1e-9 and 0.05 < high < 0.08
    assert bm.wilson(1, 0) is None
    assert bm.percentile([1, 2, 3, 4, 5], 50) == 3
