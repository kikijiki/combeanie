# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Pinned stock-tray arrangements for the wrist perception measurement matrix."""
# The seeded generator's surface_separation_m keeps products apart for grasping reasons, so a
# truly touching pair has to be constructed on purpose (Milestone 10 Q11). These builders emit
# static products at computed poses, the same contract lane_columns.lane_column_scenario uses: the
# arrangement is the experiment's independent variable and nothing may settle or roll.
#
# Products are pinned (static: true) so physics cannot walk them off the declared pose; the live
# acceptance test must assert each spawned product against ground truth before any camera reading
# is believed, exactly as the lane-depth campaign does.
#
# Cans keep the shipped catalogue albedo (the same red the wrist `can.reference_rgb` signature was
# measured against). The same-shape label experiment's white base + label_texture makes a can
# invisible to chromaticity detection: the first live campaign measured 0 overview frames for a
# single upright can under that appearance. Identity is still tested honestly: one can of a class
# is identified (match or class_only against SIM-CAN-STD); two cans of a class are named by their
# declared stocking positions (Card 063; dropped before it). Label textures are not part of this
# backend's identity path.
#
# Bottles keep their colour albedo; the wrist signature catalogue has one SKU per class.

from __future__ import annotations

import math
from typing import Any

from restocker_gazebo.lane_columns import CATALOGUE, WORKCELL_POSE  # noqa: I100,I101
from restocker_gazebo.lane_columns import TRAY_SURFACE_Z_M as _LANE_TRAY_SURFACE_Z

# Tray surface height in world z (lane_columns.TRAY_SURFACE_Z_M).
TRAY_SURFACE_Z_M = _LANE_TRAY_SURFACE_Z
# Every geometry the product collision catalogue ships. lane_columns.CATALOGUE omits can.citrus
# (a lane-column concern never needed the second can SKU); the full catalogued set is what the
# measurement matrix runs, so this module restates it with the same albedo as can.standard —
# visually the same shape and colour, distinguished only by SKU, which is the wrist identity
# boundary (class fallback signature; two of a class are dropped).
TRAY_CATALOGUE: dict[str, dict[str, Any]] = {
    **CATALOGUE,
    "can.citrus": {
        "product_class": "can",
        "sku": "SIM-CAN-CITRUS",
        "radius_m": 0.033,
        "height_m": 0.122,
        "mass_kg": 0.355,
        "friction": 0.65,
        # Same albedo as can.standard: identical appearance, different SKU.
        "rgba": list(CATALOGUE["can.standard"]["rgba"]),
    },
}
# Tray usable volume in world x/y, from workcell_geometry.yaml stock_tray.usable_volume centred at
# (0, -1.35) under workcell_pose translation (0, 0.55): centre y = -0.80, half-width 0.68,
# half-depth 0.255. Products stand inset from these bounds.
TRAY_CENTRE_X_M = 0.0
TRAY_CENTRE_Y_M = -0.80
TRAY_HALF_WIDTH_M = 0.68
TRAY_HALF_DEPTH_M = 0.255
# Surface-to-surface gap for the "isolated" baseline: larger than the generator's 0.02 m seating
# bound so nothing touches.
ISOLATED_SURFACE_GAP_M = 0.04
# Lateral offset for a same-class neighbour that is close but not touching (Q12 shadow cell):
# one corridor radius plus a small gap, still inside the tray.
CLOSE_NEIGHBOUR_SURFACE_GAP_M = 0.03


def upright_pose(x_m: float, y_m: float, geometry_key: str) -> list[float]:
    """Return the world spawn pose for an upright cylinder of this geometry at (x, y)."""
    entry = TRAY_CATALOGUE[geometry_key]
    return [x_m, y_m, TRAY_SURFACE_Z_M + entry["height_m"] / 2.0, 0.0, 0.0, 0.0]


def horizontal_pose(x_m: float, y_m: float, geometry_key: str) -> list[float]:
    """Return the world spawn pose for the same cylinder lying on its side (roll 90°)."""
    entry = TRAY_CATALOGUE[geometry_key]
    # Axis along world +Y after a +90° roll about X; centre one radius above the tray surface.
    return [x_m, y_m, TRAY_SURFACE_Z_M + entry["radius_m"], math.pi / 2.0, 0.0, 0.0]


def tilted_pose(x_m: float, y_m: float, geometry_key: str, tilt_rad: float) -> list[float]:
    """Return the world pose for a cylinder tilted by tilt_rad about the tray's x axis."""
    entry = TRAY_CATALOGUE[geometry_key]
    # Centre height so the lowest point of the tilted cylinder still touches the tray plane:
    # half-height along the tilted axis plus the radius's vertical component.
    half_height = entry["height_m"] / 2.0
    radius = entry["radius_m"]
    z = TRAY_SURFACE_Z_M + half_height * math.cos(tilt_rad) + radius * abs(math.sin(tilt_rad))
    return [x_m, y_m, z, tilt_rad, 0.0, 0.0]


def _product(
    geometry_key: str, index: int, pose: list[float], static: bool = True
) -> dict[str, Any]:
    """Build one scenario product entry from the shipped catalogue."""
    if geometry_key not in TRAY_CATALOGUE:
        raise ValueError(f"unknown geometry {geometry_key}")
    entry = TRAY_CATALOGUE[geometry_key]
    model_name = f"tray_{entry['product_class']}_{index:02d}"
    product: dict[str, Any] = {
        "model_name": model_name,
        "source_object_id": f"sim:{model_name}",
        "geometry_key": geometry_key,
        "product_class": entry["product_class"],
        "sku": entry["sku"],
        "mass_kg": entry["mass_kg"],
        "rgba": list(entry["rgba"]),
        "friction": entry["friction"],
        "spawn_pose": list(pose),
    }
    if entry.get("label_texture"):
        product["label_texture"] = entry["label_texture"]
    if static:
        product["static"] = True
    return product


def tray_scenario(products: list[dict[str, Any]]) -> dict[str, Any]:
    """Wrap product entries in a validated scenario document with no shelf columns."""
    if not products:
        raise ValueError("a tray scenario must spawn at least one product")
    names = [product["model_name"] for product in products]
    if len(names) != len(set(names)):
        raise ValueError("scenario model names must be unique")
    return {
        "schema_version": 1,
        "pose_topic": "/world/restocking/pose/info",
        "frame_id": "world",
        "backend": {"name": "gazebo_ground_truth", "version": "harmonic_pose_v1"},
        "pose_covariance_diagonal": [1.0e-8] * 6,
        "workcell_pose": list(WORKCELL_POSE),
        "products": products,
    }


def _isolated_x_offsets(geometry_keys: tuple[str, ...], seed: int) -> list[float]:
    """Spread products along tray x with the isolated gap, jittered by seed within the tray."""
    widths = [2.0 * TRAY_CATALOGUE[key]["radius_m"] for key in geometry_keys]
    gaps = [ISOLATED_SURFACE_GAP_M] * (len(geometry_keys) - 1)
    total = sum(widths) + sum(gaps)
    if total > 2.0 * TRAY_HALF_WIDTH_M:
        raise ValueError("isolated arrangement does not fit the tray width")
    # Deterministic seed jitter: shift the whole row by a fraction of the leftover margin.
    leftover = 2.0 * TRAY_HALF_WIDTH_M - total
    # Use the seed as a small integer offset in tenths of the leftover margin (quantised).
    shift = (leftover * ((seed % 5) - 2)) / 10.0 if leftover > 0 else 0.0
    start = TRAY_CENTRE_X_M - total / 2.0 + shift
    offsets: list[float] = []
    cursor = start
    for width in widths:
        offsets.append(cursor + width / 2.0)
        cursor += width
        if gaps:
            cursor += gaps.pop(0)
    return offsets


def isolated_catalogue_scenario(seed: int) -> dict[str, Any]:
    """Cell A: every catalogued geometry upright and spaced on the tray for this seed."""
    if seed not in (0, 1, 2):
        raise ValueError(f"isolated catalogue sweep predeclares seeds 0,1,2; got {seed}")
    geometry_keys = (
        "can.standard",
        "can.citrus",
        "bottle.small.standard",
        "bottle.large.standard",
    )
    xs = _isolated_x_offsets(geometry_keys, seed)
    y = TRAY_CENTRE_Y_M
    products = [
        _product(key, index + 1, upright_pose(x, y, key))
        for index, (key, x) in enumerate(zip(geometry_keys, xs, strict=True))
    ]
    return tray_scenario(products)


def touching_pair_scenario(geometry_a: str, geometry_b: str) -> dict[str, Any]:
    """Cell B: a same-class pair with centres exactly r1+r2 apart (surface contact)."""
    if geometry_a not in TRAY_CATALOGUE or geometry_b not in TRAY_CATALOGUE:
        raise ValueError("touching pair requires known geometries")
    if TRAY_CATALOGUE[geometry_a]["product_class"] != TRAY_CATALOGUE[geometry_b]["product_class"]:
        raise ValueError("Q11 asks for a same-class touching pair")
    ra = TRAY_CATALOGUE[geometry_a]["radius_m"]
    rb = TRAY_CATALOGUE[geometry_b]["radius_m"]
    y = TRAY_CENTRE_Y_M
    x_a = TRAY_CENTRE_X_M - (ra + rb) / 2.0
    x_b = TRAY_CENTRE_X_M + (ra + rb) / 2.0
    # Stay inside the tray walls.
    if (
        x_a - ra < TRAY_CENTRE_X_M - TRAY_HALF_WIDTH_M
        or x_b + rb > TRAY_CENTRE_X_M + TRAY_HALF_WIDTH_M
    ):
        raise ValueError("touching pair does not fit the tray width")
    products = [
        _product(geometry_a, 1, upright_pose(x_a, y, geometry_a)),
        _product(geometry_b, 2, upright_pose(x_b, y, geometry_b)),
    ]
    return tray_scenario(products)


def close_neighbour_scenario(geometry_key: str) -> dict[str, Any]:
    """Cell C: upright target with a same-class neighbour at the close surface gap (shadow)."""
    if geometry_key not in TRAY_CATALOGUE:
        raise ValueError(f"unknown geometry {geometry_key}")
    radius = TRAY_CATALOGUE[geometry_key]["radius_m"]
    y = TRAY_CENTRE_Y_M
    # Identical geometry pair for the neighbour so identity confusion is possible (Q12).
    centres = (2.0 * radius + CLOSE_NEIGHBOUR_SURFACE_GAP_M) / 2.0
    x_target = TRAY_CENTRE_X_M - centres
    x_neighbour = TRAY_CENTRE_X_M + centres
    if x_target - radius < TRAY_CENTRE_X_M - TRAY_HALF_WIDTH_M:
        raise ValueError("close-neighbour pair does not fit the tray width")
    products = [
        _product(geometry_key, 1, upright_pose(x_target, y, geometry_key)),
        _product(geometry_key, 2, upright_pose(x_neighbour, y, geometry_key)),
    ]
    return tray_scenario(products)


def non_upright_scenario(
    geometry_key: str = "bottle.large.standard", tilt_rad: float = math.pi / 2.0
) -> dict[str, Any]:
    """Cell D: one non-upright product alone on the tray (horizontal when tilt is 90°)."""
    if geometry_key not in TRAY_CATALOGUE:
        raise ValueError(f"unknown geometry {geometry_key}")
    if not 0.0 < tilt_rad <= math.pi / 2.0:
        raise ValueError("tilt_rad must be in (0, pi/2]")
    pose = (
        horizontal_pose(TRAY_CENTRE_X_M, TRAY_CENTRE_Y_M, geometry_key)
        if math.isclose(tilt_rad, math.pi / 2.0)
        else tilted_pose(TRAY_CENTRE_X_M, TRAY_CENTRE_Y_M, geometry_key, tilt_rad)
    )
    return tray_scenario([_product(geometry_key, 1, pose)])


def extrinsics_scenario(geometry_key: str = "can.standard") -> dict[str, Any]:
    """Cell E: a single upright can; arm configurations vary, not the arrangement."""
    return tray_scenario(
        [_product(geometry_key, 1, upright_pose(TRAY_CENTRE_X_M, TRAY_CENTRE_Y_M, geometry_key))]
    )


def expected_product_count(scenario: dict[str, Any]) -> int:
    """Return how many products the arrangement stands, for refusal-rate denominators."""
    return len(scenario["products"])
