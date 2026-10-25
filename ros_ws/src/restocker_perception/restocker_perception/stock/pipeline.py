# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""From one lane survey frame to a stock estimate: depth profile, a face crop, identity, count."""
# Used by the ROS node (live) and by the benchmark adapter (recorded frames), so both run the same
# code. The identity call is injected: a callable taking an RGB crop and returning an
# IdentifyProduct reply dict, or None when no channel is reachable. A face is only framed when it
# is big enough to be worth asking about; below that the channel is not consulted and the face is
# unavailable, which the estimator treats as "not named", never as a name.

from __future__ import annotations

from collections.abc import Callable, Mapping
from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from restocker_perception.identity.crops import crop_square
from restocker_perception.stock import estimator as est
from restocker_perception.stock.config import ProductGeometry, StockConfig
from restocker_perception.stock.depth_profile import Intrinsics, LaneProfile, measure_lane
from restocker_perception.stock.identity_policy import face_from_reply

IdentifyFunction = Callable[[np.ndarray], Mapping[str, Any] | None]
# Smallest crop side, in pixels, the identity channel is asked about. The channel's own crops in
# CMB-112 were 48 px and up; a smaller face is not consulted.
MIN_FACE_SIDE_PX = 48
FACE_MARGIN = 1.2


@dataclass(frozen=True)
class FaceCrop:
    """A square crop around the rear product, with how it was framed."""

    image: np.ndarray
    side_px: float
    range_m: float
    u: float
    v: float


def top_profile(profile: LaneProfile) -> est.TopProfile:
    """The estimator's view of a measured lane profile."""
    return est.TopProfile(
        [None if math.isnan(d) else float(d) for d in profile.near_depth_m],
        [None if math.isnan(h) else float(h) for h in profile.top_height_m],
        float(profile.bin_start_m[1] - profile.bin_start_m[0])
        if len(profile.bin_start_m) > 1
        else 0.02,
        float(profile.bed_sigma_m),
    )


def rear_face_crop(
    rgb: np.ndarray,
    intrinsics: Intrinsics,
    lane_from_optical: np.ndarray,
    config: StockConfig,
    free_depth_m: float,
    product: ProductGeometry | int | None,
) -> FaceCrop | None:
    """Crop the rear-most product (the one nearest the camera) from the RGB frame.

    Its centre is placed from the measured free depth and the product's dimensions alone (the lane
    geometry), never from a label; a lane with no product, an unknown product or a face too small
    to read returns None. ``product`` is the shape, or a class enum with a single shape.
    """
    if isinstance(product, int):
        product = config.product_by_enum(product)
    cell = config.workcell
    if product is None or free_depth_m >= cell.usable_depth_m - est.EMPTY_MARGIN_M:
        return None
    centre_depth = cell.rear_clearance_m + free_depth_m + config.rear_extent_m(product)
    centre_height = float(cell.bed_height_at(centre_depth)) + product.height_m / 2.0
    optical_from_lane = np.linalg.inv(lane_from_optical)
    point = optical_from_lane @ np.array([0.0, centre_depth, centre_height, 1.0])
    if point[2] <= 0.0:
        return None
    u = intrinsics.fx * point[0] / point[2] + intrinsics.cx
    v = intrinsics.fy * point[1] / point[2] + intrinsics.cy
    range_m = float(np.linalg.norm(point[:3]))
    extent = max(product.height_m, 2.0 * product.radius_m)
    side = intrinsics.fx * extent * FACE_MARGIN / range_m
    if side < MIN_FACE_SIDE_PX:
        return None
    crop = crop_square(rgb, u, v, range_m, intrinsics.fx, extent, margin=FACE_MARGIN)
    if crop is None:
        return None
    return FaceCrop(crop, side, range_m, float(u), float(v))


def estimate_from_frame(
    depth_m: np.ndarray,
    rgb: np.ndarray | None,
    intrinsics: Intrinsics,
    lane_from_optical: np.ndarray,
    config: StockConfig,
    *,
    lane_id: str,
    expected_class: int,
    expected_sku: str | None,
    identify: IdentifyFunction | None = None,
    identity_mode: str = "safe",
    producer: Mapping[str, Any] | None = None,
) -> tuple[est.LaneStockEstimate, LaneProfile]:
    """Run the whole estimator on one survey frame.

    ``producer`` overrides the depth evidence with what a real LaneObservation said (keys
    ``ok``, ``coverage``, ``free_depth_m``, ``obstructed``); the top profile is always computed
    from the frame, since a LaneObservation does not carry it.
    """
    profile = measure_lane(depth_m, intrinsics, lane_from_optical, config, lane_id)
    producer = producer or {
        "ok": True,
        "coverage": profile.coverage,
        "free_depth_m": profile.available_depth_m,
        "obstructed": profile.obstructed,
    }
    base = dict(
        lane_id=lane_id,
        evidence_ok=bool(producer["ok"]),
        coverage=float(producer["coverage"]),
        free_depth_m=float(producer["free_depth_m"]),
        obstructed=bool(producer["obstructed"]),
        expected_class=expected_class,
        expected_sku=expected_sku,
        profile=top_profile(profile),
    )
    first = est.estimate_lane_stock(est.LaneEvidence(**base), config)
    if not first.answered or first.count_estimate == 0:
        return first, profile
    faces: list[est.FaceIdentity] = []
    reply: Mapping[str, Any] | None = None
    reachable = False
    crop = (
        rear_face_crop(
            rgb,
            intrinsics,
            lane_from_optical,
            config,
            first.free_depth_m,
            config.products.get(first.product_name),
        )
        if rgb is not None and identify is not None
        else None
    )
    if crop is not None and identify is not None:
        reply = identify(crop.image)
        reachable = bool(reply and reply.get("sidecar_available", False))
        faces.append(face_from_reply(reply, identity_mode))
    elif identify is not None:
        faces.append(est.FaceIdentity(est.IDENTITY_UNAVAILABLE, None, "no face framed"))
    final = est.estimate_lane_stock(
        est.LaneEvidence(
            **base,
            faces=faces,
            identity_reachable=reachable,
            identity_registry_sha256=str((reply or {}).get("registry_sha256", "")),
            identity_calibration_sha256=str((reply or {}).get("calibration_sha256", "")),
        ),
        config,
    )
    return final, profile
