# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Run the lane stock estimator over recorded lane survey frames and write prediction rows."""
# The recorded frames (``--lane-views``) are wrist RGB-D frames taken at each lane's survey
# station, labelled with the scene's lane ground truth. The estimator sees only the pixels, the
# camera pose and intrinsics, and the lane policy (expected SKU and its class); never the count,
# the SKUs or the misplaced flags. Identity arms:
#
#   abstain  no identity channel at all (identity unavailable): what the system does with the
#            sidecar absent, and what the safe policy reduces to when nothing is confirmed.
#   channel  the real /identify_product logic on the framed rear-product crop, safe policy.
#   oracle   an ablation, never a result: a perfect channel for whatever face is framed. It shows
#            the ceiling that visibility alone puts on misplaced detection.
#
# A ``degrade`` option blinds a seeded band of the depth image before measuring, to test the
# refusal on low coverage: the evidence is made worse and the answer must follow it.

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np
import yaml

from restocker_perception.stock.config import CLASS_ENUM, StockConfig, load_stock_config
from restocker_perception.stock.depth_profile import Intrinsics, lane_from_optical
from restocker_perception.stock.pipeline import estimate_from_frame

ARMS = ("abstain", "channel", "oracle")
LANE_IDS = ("lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06")
# Share of lane frames degraded, and the band's share of the image height, when degrading.
DEGRADE_PROBABILITY = 0.5
DEGRADE_BAND = (0.30, 0.70)


def load_dataset(directory: str | Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    root = Path(directory)
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    rows = [
        json.loads(line)
        for line in (root / "labels.jsonl").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    return manifest, rows


def lane_survey_rows(rows: Iterable[Mapping[str, Any]]) -> list[Mapping[str, Any]]:
    """The wrist frames taken at a lane's survey station (their viewpoint is the lane's id)."""
    return [r for r in rows if r["sensor"] == "wrist" and r.get("viewpoint") in LANE_IDS]


def class_by_sku(catalogue_path: str | Path) -> dict[str, int]:
    """SKU -> ShelfLane product class enum, from the generated catalogue's shapes."""
    document = yaml.safe_load(Path(catalogue_path).read_text(encoding="utf-8"))
    return {
        row["sku"]: CLASS_ENUM[row["shape"]]
        for row in document["products"]
        if row.get("shape") in CLASS_ENUM
    }


def _read_png(path: Path, mode: str | None = None) -> np.ndarray:
    from PIL import Image  # noqa: PLC0415 - 16-bit PNG needs a real decoder

    with Image.open(path) as image:
        return np.asarray(image.convert(mode) if mode else image)


def degrade_mask(frame_id: str, height: int) -> tuple[int, int] | None:
    """Rows of the depth image to blind for this frame, or None (pure in the frame id)."""
    digest = hashlib.sha256(f"stock-degrade:{frame_id}".encode()).digest()
    draw = [b / 255.0 for b in digest[:3]]
    if draw[0] >= DEGRADE_PROBABILITY:
        return None
    share = DEGRADE_BAND[0] + draw[1] * (DEGRADE_BAND[1] - DEGRADE_BAND[0])
    top = int(draw[2] * (1.0 - share) * height)
    return top, top + int(share * height)


def perturb_depth(depth: np.ndarray, frame_id: str, noise: float, dropout: float) -> np.ndarray:
    """Seeded sensor-like noise: range-proportional Gaussian error and random pixel dropout.

    The simulator's depth camera is ideal, so this is a synthetic perturbation of the recorded
    frame, not a sensor model: ``noise`` is the standard deviation as a fraction of range (a
    stereo or time-of-flight camera at 1.2 m is of the order of 0.5-2 %), ``dropout`` the share of
    pixels that return nothing.
    """
    seed = int.from_bytes(
        hashlib.sha256(f"stock-noise:{frame_id}".encode()).digest()[:8], "little"
    )
    rng = np.random.default_rng(seed)
    out = depth * (1.0 + noise * rng.standard_normal(depth.shape)) if noise > 0 else depth.copy()
    if dropout > 0:
        out[rng.random(depth.shape) < dropout] = 0.0
    return out


def _oracle_reply(sku: str) -> dict[str, Any]:
    return {
        "status": 0,
        "sidecar_available": True,
        "has_sku": True,
        "sku": sku,
        "reason": "oracle",
        "evidence_kinds": ["barcode"],
        "evidence_product_ids": [sku],
    }


def predict_frame(
    dataset: Path,
    row: Mapping[str, Any],
    config: StockConfig,
    classes: Mapping[str, int],
    *,
    arm: str,
    degrade: bool,
    channel: Callable[[np.ndarray], Mapping[str, Any] | None] | None = None,
    identity_mode: str = "safe",
    noise: float = 0.0,
    dropout: float = 0.0,
) -> dict[str, Any]:
    """One lane survey frame -> one prediction row (the schema of perception_scorer plus stock)."""
    lane_id = row["viewpoint"]
    truth_lane = next(lane for lane in row["lanes"] if lane["lane_id"] == lane_id)
    expected_sku = truth_lane["expected_sku"]
    expected_class = classes.get(expected_sku, 0) if expected_sku else 0
    depth = (
        _read_png(dataset / "frames" / f"{row['frame_id']}.depth.png").astype(np.float64) / 1000.0
    )
    masked = degrade_mask(row["frame_id"], depth.shape[0]) if degrade else None
    if masked is not None:
        depth[masked[0] : masked[1], :] = 0.0
    if noise > 0 or dropout > 0:
        depth = perturb_depth(depth, row["frame_id"], noise, dropout)
    rgb = _read_png(dataset / "frames" / f"{row['frame_id']}.png", "RGB")
    camera = row["camera"]
    intr = camera["intrinsics"]
    matrix = lane_from_optical(
        camera["pose_world"], config.workcell.lane_center_x_m[lane_id], config.workcell.pose_world
    )
    identify = None
    if arm == "oracle":
        members = [o for o in row["objects"] if o["lane_id"] == lane_id]
        if members:
            rear = max(members, key=lambda o: o["lane_order"])
            identify = lambda _crop, sku=rear["sku"]: _oracle_reply(sku)  # noqa: E731
    elif arm == "channel":
        identify = channel
    replies: list[Mapping[str, Any] | None] = []
    if identify is not None:
        inner = identify

        def identify(crop: np.ndarray) -> Mapping[str, Any] | None:  # noqa: F811 - records replies
            reply = inner(crop)
            replies.append(reply)
            return reply

    estimate, profile = estimate_from_frame(
        depth,
        rgb,
        Intrinsics(intr["fx"], intr["fy"], intr["cx"], intr["cy"]),
        matrix,
        config,
        lane_id=lane_id,
        expected_class=expected_class,
        expected_sku=expected_sku,
        identify=identify,
        identity_mode=identity_mode,
    )
    answered = estimate.answered
    return {
        "kind": "lane",
        "frame_id": row["frame_id"],
        "lane_id": lane_id,
        "arm": arm,
        "degraded": masked is not None,
        "noise": noise,
        "dropout": dropout,
        "state": "estimated" if answered else "cannot_tell",
        "reason": estimate.reason,
        "reason_detail": estimate.reason_detail,
        "count": estimate.count_estimate if answered else None,
        "count_min": estimate.count_min if answered else None,
        "count_max": estimate.count_max if answered else None,
        "count_consistent": estimate.count_consistent if answered else None,
        "misplaced": estimate.misplaced if answered else None,
        "composition": estimate.composition if answered else None,
        "class_mismatch": estimate.class_mismatch,
        "segments": [list(item) for item in estimate.segments],
        "visible_faces": estimate.visible_faces,
        "pitch_residual": estimate.pitch_residual,
        "named_count": estimate.named_count,
        "unverified_count": estimate.unverified_count,
        "face_skus": list(estimate.face_skus),
        "face_identity_status": list(estimate.face_identity_status),
        "misplaced_skus": list(estimate.misplaced_skus),
        "coverage": estimate.coverage,
        "free_depth_m": estimate.free_depth_m,
        # What the channel itself answered (before the naming policy), for the ablations.
        "channel_replies": [
            None
            if r is None
            else {
                "status": r.get("status"),
                "sku": r.get("sku") if r.get("has_sku") else None,
                "evidence_kinds": list(r.get("evidence_kinds", [])),
                "sidecar_available": r.get("sidecar_available"),
            }
            for r in replies
        ],
        "measured_coverage": profile.coverage,
        "measured_free_depth_m": profile.available_depth_m,
    }


def predict_datasets(
    datasets: Iterable[str | Path],
    catalogue: str | Path,
    *,
    arm: str,
    degrade: bool = False,
    channel: Callable[[np.ndarray], Mapping[str, Any] | None] | None = None,
    identity_mode: str = "safe",
    config: StockConfig | None = None,
    seeds: Iterable[int] | None = None,
    noise: float = 0.0,
    dropout: float = 0.0,
) -> list[dict[str, Any]]:
    """Predictions for every lane survey frame of every dataset (optionally only some seeds)."""
    if arm not in ARMS:
        raise ValueError(f"arm must be one of {ARMS}")
    config = config or load_stock_config()
    classes = class_by_sku(catalogue)
    wanted = None if seeds is None else set(seeds)
    out: list[dict[str, Any]] = []
    for directory in datasets:
        root = Path(directory)
        _, rows = load_dataset(root)
        for row in lane_survey_rows(rows):
            if wanted is not None and row["seed"] not in wanted:
                continue
            out.append(
                predict_frame(
                    root,
                    row,
                    config,
                    classes,
                    arm=arm,
                    degrade=degrade,
                    channel=channel,
                    identity_mode=identity_mode,
                    noise=noise,
                    dropout=dropout,
                )
            )
    return out
