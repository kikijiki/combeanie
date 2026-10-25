# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Where does single-view embedding accuracy go? A per-factor breakdown on calibration data."""
# Reads train and validation seeds only (the same guard as calibration). For every close-view
# crop: the closed-set top-1 against the registry (which holds the true product), broken down by
# the factors that differ between the benchmark and CMB-101's renders: lighting condition,
# product class, range and crop size, tilt of the product, and the azimuth of the camera in the
# product's own frame (which side of the barrel faces the lens). It also tries crop-policy
# variants (margin) and a 180 degree query rotation, to separate pipeline defects from
# properties of the world.

from __future__ import annotations

from collections import defaultdict
from collections.abc import Sequence
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

from restocker_perception.identity import benchmark as bm
from restocker_perception.identity.crops import crop_square
from restocker_perception.registry.backends import EmbeddingBackend
from restocker_perception.registry.imageio import read_image
from restocker_perception.registry.model import Registry
from restocker_perception.registry.retrieval import RegistryIndex


def tilt_deg(quaternion: Sequence[float]) -> float:
    x, y, z, w = quaternion[3:7]
    z_axis = 1.0 - 2.0 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, z_axis))))


def yaw_rad(quaternion: Sequence[float]) -> float:
    x, y, z, w = quaternion[3:7]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def relative_azimuth_deg(object_pose: Sequence[float], camera_pose: Sequence[float]) -> float:
    """Direction of the camera seen from the product, in the product's own yaw frame, [0, 360)."""
    dx, dy = camera_pose[0] - object_pose[0], camera_pose[1] - object_pose[1]
    return math.degrees(math.atan2(dy, dx) - yaw_rad(object_pose)) % 360.0


def _accuracy(rows: list[dict[str, Any]], key) -> dict[str, dict[str, Any]]:
    bins: dict[str, list[bool]] = defaultdict(list)
    for row in rows:
        bins[str(key(row))].append(row["correct"])
    return {k: {"n": len(v), "top1": float(np.mean(v))} for k, v in sorted(bins.items())}


def diagnose(
    datasets: Sequence[str | Path],
    registry: Registry,
    backend: EmbeddingBackend,
    geometry: dict[str, dict[str, Any]],
    *,
    scoring: str = "max",
) -> dict[str, Any]:
    allowed = bm.CALIBRATION_SPLITS
    index = RegistryIndex.from_registry(registry, backend)
    rows: list[dict[str, Any]] = []
    cache = bm.CachedEmbedding(backend)
    for directory in datasets:
        root = Path(directory)
        groups = list(bm.object_groups(root, geometry, allowed_splits=allowed))
        for group in groups:
            truth = registry.resolve(group.sku)
            for view in group.views:
                vector = cache.embed([view.image])
                result = index.match(vector, scoring=scoring)
                meta = view.meta
                pose = meta["object_pose"]
                rows.append(
                    {
                        "sku": group.sku,
                        "class": group.product_class,
                        "lighting": view.lighting,
                        "range": view.range_m,
                        "side_px": meta["side_px"],
                        "tilt": tilt_deg(pose),
                        "azimuth": relative_azimuth_deg(pose, meta["camera_pose"]),
                        "correct": result.top1.product_id == truth,
                        "frame": view.frame_id,
                        "dataset": str(root),
                        "meta": meta,
                    }
                )
    report: dict[str, Any] = {
        "n_views": len(rows),
        "top1": float(np.mean([r["correct"] for r in rows])) if rows else None,
        "scoring": scoring,
        "by_lighting": _accuracy(rows, lambda r: r["lighting"]),
        "by_class": _accuracy(rows, lambda r: r["class"]),
        "by_range": _accuracy(rows, lambda r: f"{round(r['range'] * 10) / 10:.1f}m"),
        "by_crop_side": _accuracy(rows, lambda r: f"{int(r['side_px'] // 100) * 100}px"),
        "by_tilt": _accuracy(rows, lambda r: f"{int(r['tilt'] // 10) * 10}deg"),
        "by_camera_azimuth_in_product_frame": _accuracy(
            rows, lambda r: f"{int(r['azimuth'] // 45) * 45:03d}"
        ),
        "by_sku": _accuracy(rows, lambda r: r["sku"]),
    }
    report["variants"] = _variants(rows, registry, backend, index, geometry, scoring)
    return report


def _variants(rows, registry, backend, index, geometry, scoring) -> dict[str, Any]:
    """Re-cut every crop under other policies and measure top-1 again (same crops otherwise)."""
    frames: dict[str, np.ndarray] = {}
    out: dict[str, Any] = {}
    cache = bm.CachedEmbedding(backend)

    def accuracy(make) -> float:
        hits = []
        for row in rows:
            image = make(row)
            if image is None:
                continue
            result = index.match(cache.embed([image]), scoring=scoring)
            hits.append(result.top1.product_id == registry.resolve(row["sku"]))
        return float(np.mean(hits)) if hits else float("nan")

    def frame_of(row) -> np.ndarray:
        key = (row["dataset"], row["frame"])
        if key not in frames:
            _m, labels = bm.load_dataset(row["dataset"])
            record = next(r for r in labels if r["frame_id"] == row["frame"])
            frames[key] = read_image(Path(row["dataset"]) / record["files"]["rgb"])
        return frames[key]

    def recut(margin):
        def make(row):
            geo = geometry[row["meta"]["geometry_key"]]
            extent = max(2 * geo["radius_m"], geo["height_m"])
            record = _object_row(row)
            return crop_square(
                frame_of(row),
                record["u"],
                record["v"],
                row["range"],
                record["fx"],
                extent,
                margin=margin,
            )

        return make

    for margin in (1.0, 1.2, 1.5):
        out[f"margin_{margin}"] = accuracy(recut(margin))
    out["rotate_180"] = accuracy(
        lambda row: (
            np.ascontiguousarray(np.rot90(recut(1.2)(row), 2))
            if recut(1.2)(row) is not None
            else None
        )
    )
    out["flip_horizontal"] = accuracy(
        lambda row: (
            np.ascontiguousarray(recut(1.2)(row)[:, ::-1]) if recut(1.2)(row) is not None else None
        )
    )
    return out


_LABEL_CACHE: dict[str, list[dict[str, Any]]] = {}


def _object_row(row: dict[str, Any]) -> dict[str, Any]:
    """Pixel and focal length of the viewed object, re-read from the labels."""
    dataset = row["dataset"]
    if dataset not in _LABEL_CACHE:
        _LABEL_CACHE[dataset] = bm.load_dataset(dataset)[1]
    record = next(r for r in _LABEL_CACHE[dataset] if r["frame_id"] == row["frame"])
    for obj in record["objects"]:
        if obj["pose_world"] == row["meta"]["object_pose"]:
            return {
                "u": obj["pixel"]["u"],
                "v": obj["pixel"]["v"],
                "fx": record["camera"]["intrinsics"]["fx"],
            }
    raise KeyError(row["frame"])


def main(argv: Sequence[str] | None = None) -> int:
    import argparse  # noqa: PLC0415

    from restocker_perception.registry.backends import SigLip2Backend  # noqa: PLC0415
    from restocker_perception.registry.simulated import load_geometry_catalogue  # noqa: PLC0415

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", required=True, type=Path)
    parser.add_argument("--dataset", required=True, action="append", type=Path)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--revision", default="main")
    parser.add_argument("--scoring", default="max")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args(argv)
    report = diagnose(
        args.dataset,
        Registry.load(args.registry),
        SigLip2Backend(revision=args.revision),
        load_geometry_catalogue(args.geometry),
        scoring=args.scoring,
    )
    args.out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({k: v for k, v in report.items() if k != "variants"}, indent=1)[:6000])
    print(json.dumps(report["variants"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
