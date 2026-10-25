# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A stocked shelf and a packed back room of textured, labelled products for the showcase demo."""
# Every other manipulable fixture stands one or two plain coloured SKUs per shape. This one uses
# the generated catalogue (tools/label_factory.py): the straight-cylinder SKUs, which carry a
# printed label each and share the three base shapes' dimensions to the millimetre, so every
# column and tray formula in lane_columns.py applies unchanged.
#
# SHELF. Each of the six lanes stands one SKU's column, pinned at the computed pose like every
# other front column. A column's start count is at or below the lane's target, so some lanes are
# stocked to target and the rest are depleted by a different amount each.
#
# BACK ROOM. The stock tray holds one packed block per SKU: two columns of identical products,
# rows touching to within ROW_GAP_M. The block is a feed queue, as in the dense fixture: the robot
# takes the front (+Y) product of a column before the one behind it. Columns are as close as the
# approach corridor allows -- the tool sweeps in along -Y, so a product in a neighbouring column
# is in the way unless the centre distance exceeds the planner-padded wrist radius plus the larger
# of the two product radii (scenario_random.required_lateral_distance), here with a few
# millimetres to spare. Blocks sit side by side on that one pitch with no wider gap: neighbouring
# blocks are told apart by their labels, as on a real back-room pallet.
#
# The back room stocks every SKU the shelf sells. Only the SKUs with a deficit are picked; the
# rest stay put as realistic reserve, and a block with more stock than its deficit keeps the rest.
#
# LABELS. A strip is stored upright on disk and installed with its half-turn at launch
# (label_textures.py), so the fixture only has to choose which way each product faces. Products
# face the +Y side: the shelf front and the customer, and the viewpoint the demo cameras share.
# The shelf is faced; the back room is not. A stocked shelf is faced forward by whoever stocks it,
# but delivered stock sits in the tray however it was set down, so each tray product takes a yaw
# drawn from a stream seeded by its model name and TRAY_YAW_SEED (tray_yaw_rad). A cylinder is
# symmetric about its axis, so footprints, packing pitch and the approach corridor do not change.

from __future__ import annotations

import hashlib
import math
from typing import Any

from restocker_gazebo.lane_columns import (  # noqa: I100,I101
    CATALOGUE,
    TRAY_SURFACE_Z_M,
    WORKCELL_POSE,
    spawn_pose_in_world,
)
from restocker_gazebo.scenario_random import SplitMix64

# The base geometry a generated SKU's shape is laid out on (lane_columns.CATALOGUE key).
BASE_GEOMETRY = {
    "can": "can.standard",
    "small_bottle": "bottle.small.standard",
    "large_bottle": "bottle.large.standard",
}

# Lane targets, per base geometry. The dense fixture's measured regime: short of surveyed capacity
# (12/11/8) so every placement is a clean mouth insertion whose column growth the depth deficit
# can see (restocker_world_state/config/dense_restock_lanes.yaml carries the measurement).
TARGET_COUNT = {"can.standard": 10, "bottle.small.standard": 9, "bottle.large.standard": 6}

# (lane, SKU, products standing on the shelf at the start). The shelf alternates shape along the
# row; the large SKU holds two facings (lanes 3 and 6), as a popular product does.
SHELF: tuple[tuple[str, str, int], ...] = (
    ("lane_01", "GEN-001", 7),
    ("lane_02", "GEN-005", 9),
    ("lane_03", "GEN-009", 3),
    ("lane_04", "GEN-003", 8),
    ("lane_05", "GEN-007", 6),
    ("lane_06", "GEN-009", 6),
)

# The back room, left to right: (SKU, products in the block). Each block is two columns wide and
# as many rows deep as ROW_PITCH_M allows on the reachable part of the tray; a short count leaves
# the back-right slot(s) vacant, as a partly consumed delivery does.
BACK_ROOM: tuple[tuple[str, int], ...] = (
    ("GEN-007", 9),
    ("GEN-001", 10),
    ("GEN-009", 7),
    ("GEN-003", 9),
    ("GEN-005", 10),
)

COLUMNS_PER_BLOCK = 2
# Lateral centre pitch between adjacent columns: the larger of the pitch the pair's radii need
# (required_lateral_distance at the shipped gripper geometry, 0.0865 m + larger radius) plus a
# margin, so a column beside a large bottle is a little wider than one beside two cans.
LATERAL_MARGIN_M = 0.0095
WRIST_CORRIDOR_HALF_WIDTH_M = 0.0865
# Rows touch to within this surface gap. Products have no product-to-product contact, so the gap
# is the clearance the grasp needs for the finger pads, not a physics requirement.
ROW_GAP_M = 0.012
# The front row stands where the dense fixture's does, the nearest the arm reaches.
FRONT_ROW_Y_M = -0.68
# The far row stays inside the surveyed tray and inside the band the arm finishes the approach in.
FAR_ROW_LIMIT_Y_M = -1.0

# Which way a product faces, as a rotation about its own axis. The fixtures' zero faces the label
# toward -Y, the back room, so the shelf and the tray are turned half a turn to face +Y.
LANE_LABEL_SPIN_RAD = math.pi
# Scene seed of the tray's random yaws: change it to deal a different arrangement.
TRAY_YAW_SEED = 1


def tray_yaw_rad(model_name: str) -> float:
    """The yaw of a tray product, uniform over [0, 2 pi) and fixed by its name and the scene seed."""
    digest = hashlib.sha256(f"{TRAY_YAW_SEED}:{model_name}".encode()).digest()
    stream = SplitMix64(int.from_bytes(digest[:8], "big"))
    return round(stream.next_unit() * 2.0 * math.pi, 6)


def column_pitch_m(first_radius_m: float, second_radius_m: float) -> float:
    """The lateral centre distance two adjacent tray columns stand at."""
    return round(
        WRIST_CORRIDOR_HALF_WIDTH_M + max(first_radius_m, second_radius_m) + LATERAL_MARGIN_M, 3
    )


def row_pitch_m(radius_m: float) -> float:
    """The depth centre distance two rows of one SKU stand at."""
    return round((2.0 * radius_m) + ROW_GAP_M, 4)


def row_count(radius_m: float) -> int:
    """How many rows of this radius fit between the front row and the far limit."""
    return int(((FRONT_ROW_Y_M - FAR_ROW_LIMIT_Y_M) / row_pitch_m(radius_m)) + 1e-9) + 1


def _rotation_x_then_z(roll: float, spin: float) -> list[list[float]]:
    # Rx(roll) @ Rz(spin): turn about the product's own axis, then tilt the axis by the roll.
    cr, sr = math.cos(roll), math.sin(roll)
    cs, ss = math.cos(spin), math.sin(spin)
    return [
        [cs, -ss, 0.0],
        [cr * ss, cr * cs, -sr],
        [sr * ss, sr * cs, cr],
    ]


def _rpy(matrix: list[list[float]]) -> tuple[float, float, float]:
    """Roll, pitch, yaw of the SDF convention R = Rz(yaw) Ry(pitch) Rx(roll)."""
    pitch = -math.asin(max(-1.0, min(1.0, matrix[2][0])))
    roll = math.atan2(matrix[2][1], matrix[2][2])
    yaw = math.atan2(matrix[1][0], matrix[0][0])
    return roll, pitch, yaw


def lane_pose(lane_id: str, geometry_key: str, index: int) -> list[float]:
    """The pinned pose of the index-th product in a lane, turned to face the shelf front."""
    pose = spawn_pose_in_world(lane_id, geometry_key, index)
    roll, pitch, yaw = _rpy(_rotation_x_then_z(pose[3], LANE_LABEL_SPIN_RAD))
    return [round(value, 6) for value in (*pose[:3], roll, pitch, yaw)]


def tray_columns_x(block_radii_m: list[float]) -> list[float]:
    """World x of every back-room column, left to right, centred on the tray."""
    radii = [radius for radius in block_radii_m for _ in range(COLUMNS_PER_BLOCK)]
    steps = [column_pitch_m(a, b) for a, b in zip(radii, radii[1:], strict=False)]
    cursor = -sum(steps) / 2.0
    positions = [cursor]
    for step in steps:
        cursor += step
        positions.append(cursor)
    return [round(value, 6) for value in positions]


def _entry(row: dict[str, Any]) -> dict[str, Any]:
    return CATALOGUE[BASE_GEOMETRY[row["shape"]]]


def _product(
    model_name: str, row: dict[str, Any], pose: list[float], static: bool
) -> dict[str, Any]:
    entry = _entry(row)
    product: dict[str, Any] = {
        "model_name": model_name,
        "source_object_id": f"sim:{model_name}",
        "geometry_key": row["geometry_key"],
        "product_class": entry["product_class"],
        "sku": row["sku"],
        "mass_kg": entry["mass_kg"],
        # An albedo map is modulated by the material colour under it, so the textured product's
        # base is white and its colour is the label's.
        "rgba": [1.0, 1.0, 1.0, 1.0],
        "label_texture": row["texture"],
        "friction": entry["friction"],
        "spawn_pose": pose,
    }
    if static:
        product["static"] = True
    return product


def _labels_by_sku(labels: dict[str, Any]) -> dict[str, dict[str, Any]]:
    rows = {row["sku"]: row for row in labels["products"]}
    for sku in {sku for _, sku, _ in SHELF} | {sku for sku, _ in BACK_ROOM}:
        if sku not in rows:
            raise ValueError(f"unknown SKU {sku}")
        if rows[sku]["shape"] not in BASE_GEOMETRY:
            raise ValueError(f"{sku} is a {rows[sku]['shape']}, which the robot cannot manipulate")
    return rows


def shelf_products(labels: dict[str, Any]) -> list[dict[str, Any]]:
    """The six lane columns, pinned, frontmost product first."""
    rows = _labels_by_sku(labels)
    products: list[dict[str, Any]] = []
    for lane_id, sku, count in SHELF:
        row = rows[sku]
        base = BASE_GEOMETRY[row["shape"]]
        capacity = CATALOGUE[base]["lane_capacity"]
        if not 0 < count <= TARGET_COUNT[base] <= capacity:
            raise ValueError(f"{lane_id} cannot start with {count} {sku}")
        for index in range(count):
            name = f"{lane_id}_{row['shape']}_{index + 1:02d}"
            products.append(_product(name, row, lane_pose(lane_id, base, index), static=True))
    return products


def back_room_products(labels: dict[str, Any]) -> list[dict[str, Any]]:
    """The packed tray blocks: one SKU per block, filled row by row from the front."""
    rows = _labels_by_sku(labels)
    columns = tray_columns_x([_entry(rows[sku])["radius_m"] for sku, _ in BACK_ROOM])
    products: list[dict[str, Any]] = []
    for block, (sku, count) in enumerate(BACK_ROOM):
        row = rows[sku]
        entry = _entry(row)
        radius = entry["radius_m"]
        block_x = columns[block * COLUMNS_PER_BLOCK : (block + 1) * COLUMNS_PER_BLOCK]
        slots = row_count(radius) * COLUMNS_PER_BLOCK
        if not 0 < count < slots + 1:
            raise ValueError(f"block {sku} cannot hold {count} products in {slots} slots")
        for index in range(count):
            row_index, column_index = divmod(index, COLUMNS_PER_BLOCK)
            y_m = round(FRONT_ROW_Y_M - (row_index * row_pitch_m(radius)), 6)
            name = f"stock_{sku.lower().replace('-', '_')}_{index + 1:02d}"
            pose = [
                block_x[column_index],
                y_m,
                round(TRAY_SURFACE_Z_M + (entry["height_m"] / 2.0), 6),
                0.0,
                0.0,
                tray_yaw_rad(name),
            ]
            products.append(_product(name, row, pose, static=False))
    return products


def lane_policy(labels: dict[str, Any]) -> dict[str, Any]:
    """The lane semantics document: each lane's SKU, class and target."""
    rows = _labels_by_sku(labels)
    lanes: dict[str, Any] = {}
    for lane_id, sku, _ in SHELF:
        row = rows[sku]
        base = BASE_GEOMETRY[row["shape"]]
        lanes[lane_id] = {
            "expected_product_class": CATALOGUE[base]["product_class"],
            "expected_sku": sku,
            "target_count": TARGET_COUNT[base],
        }
    return {"schema_version": 1, "lanes": lanes}


def showcase_scenario(labels: dict[str, Any]) -> dict[str, Any]:
    """The showcase scenario document for the generated label manifest ``labels``."""
    products = shelf_products(labels) + back_room_products(labels)
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


def deficits(labels: dict[str, Any]) -> dict[str, int]:
    """Products each SKU's lanes are short of their target, summed over its lanes."""
    rows = _labels_by_sku(labels)
    short: dict[str, int] = {}
    for _, sku, count in SHELF:
        target = TARGET_COUNT[BASE_GEOMETRY[rows[sku]["shape"]]]
        short[sku] = short.get(sku, 0) + (target - count)
    return short
