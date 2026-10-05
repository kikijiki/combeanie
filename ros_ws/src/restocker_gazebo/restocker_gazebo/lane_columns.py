# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Scenarios that stand a known column of products in a lane."""
# Lane survey scenarios need lanes whose column length is known exactly, from empty to full, for
# every catalogued product.
#
# The gravity feed cannot build them: products have no product-to-product contact (dartsim on
# ODE's narrowphase has no cylinder/cylinder collider, and every catalogued product is a
# cylinder), so a released column arrives as a heap at one pose.
#
# Instead each product is stood at a computed pose and pinned. The runtime test verifies every
# spawned product against ground truth before treating the arrangement as known.
#
# The arithmetic is the gravity feed's, run forwards. A product rests on the inclined roller bed
# with its axis along the bed normal, the frontmost leaning on the retainer, and each one behind it
# one contact diameter further back along the bed: 2 * radius * cos(incline) along the lane's depth
# axis, the same pitch the placement proof in the world state uses.

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

# The product geometries the shipped catalogue carries, and the numbers a column needs from each.
# Not read from product_collision_catalog.yaml at import time, so a scenario document can be
# written without a workspace; test_lane_columns.py keeps the two in step.
#
# lane_capacity is how many the lane admits, not how many fit: room at the rear for one more is
# 2 * radius + insert_entry_clearance_m, so a lane takes products until the free depth drops below
# that. These are the counts the handover records.
CATALOGUE: dict[str, dict[str, Any]] = {
    "can.standard": {
        "product_class": "can",
        "sku": "SIM-CAN-STD",
        "radius_m": 0.033,
        "height_m": 0.122,
        "mass_kg": 0.355,
        "friction": 0.65,
        "rgba": [0.78, 0.12, 0.08, 1.0],
        "lane_capacity": 12,
    },
    "bottle.small.standard": {
        "product_class": "small_bottle",
        "sku": "SIM-BOTTLE-SMALL",
        "radius_m": 0.034,
        "height_m": 0.200,
        "mass_kg": 0.500,
        "friction": 0.72,
        "rgba": [0.10, 0.42, 0.82, 0.82],
        "lane_capacity": 11,
    },
    "bottle.large.standard": {
        "product_class": "large_bottle",
        "sku": "SIM-BOTTLE-LARGE",
        "radius_m": 0.045,
        "height_m": 0.290,
        "mass_kg": 1.100,
        "friction": 0.76,
        "rgba": [0.12, 0.68, 0.34, 0.82],
        "lane_capacity": 8,
    },
}

# The shipped workcell, restated. test_lane_columns.py checks each value against
# restocker_description/config/workcell_geometry.yaml.
WORKCELL_POSE = [0.0, 0.55, 0.75, 0.0, 0.0, 0.0]
BED_DATUM_DEPTH_M = 0.84
LANE_INCLINE_RAD = math.radians(4.0)
LANE_CENTER_X_M = {
    "lane_01": -1.0,
    "lane_02": -0.6,
    "lane_03": -0.2,
    "lane_04": 0.2,
    "lane_05": 0.6,
    "lane_06": 1.0,
}


@dataclass(frozen=True)
class LaneColumn:
    """One lane's contents: this many of this geometry, packed forward against the retainer."""

    lane_id: str
    geometry_key: str
    count: int


@dataclass(frozen=True)
class TrayGrid:
    """One SKU block on the stock tray, filled front-to-back with a spare slot."""

    geometry_key: str
    column_x_m: tuple[float, float]
    row_y_m: tuple[float, ...]
    count: int


def bed_height_at(lane_depth_m: float) -> float:
    """Height of the roller surface, in the lane frame, at a given lane depth."""
    return (BED_DATUM_DEPTH_M - lane_depth_m) * math.tan(LANE_INCLINE_RAD)


def column_pose_in_lane(geometry_key: str, index: int) -> tuple[float, float, float]:
    """Return the lane-frame centre depth, centre height and roll of the index-th product."""
    # Index zero is the frontmost product, leaning on the retainer; each one after it sits one
    # contact diameter further back along the inclined bed.
    entry = CATALOGUE[geometry_key]
    radius = entry["radius_m"]
    height = entry["height_m"]
    pitch = 2.0 * radius * math.cos(LANE_INCLINE_RAD)
    center_depth = BED_DATUM_DEPTH_M - radius - (index * pitch)
    # The base sits on the bed; the centre is half a height along the bed normal from it, and the
    # normal leans forward by the incline, so the centre is forward of the base by that much.
    base_depth = center_depth - ((height / 2.0) * math.sin(LANE_INCLINE_RAD))
    center_height = bed_height_at(base_depth) + ((height / 2.0) * math.cos(LANE_INCLINE_RAD))
    # Roll about the lane's X axis by minus the incline takes +Z to the bed normal, which leans
    # toward +Y. It is the same sign workcell.xacro rolls the roller slab by.
    return center_depth, center_height, -LANE_INCLINE_RAD


def spawn_pose_in_world(lane_id: str, geometry_key: str, index: int) -> list[float]:
    """Return the world-frame [x, y, z, roll, pitch, yaw] a scenario spawns this product at."""
    if lane_id not in LANE_CENTER_X_M:
        raise ValueError(f"unknown lane {lane_id}")
    center_depth, center_height, roll = column_pose_in_lane(geometry_key, index)
    return [
        LANE_CENTER_X_M[lane_id] + WORKCELL_POSE[0],
        center_depth + WORKCELL_POSE[1],
        center_height + WORKCELL_POSE[2],
        roll,
        0.0,
        0.0,
    ]


def lane_column_scenario(columns: list[LaneColumn]) -> dict[str, Any]:
    """Build a validated scenario document standing the requested columns in their lanes."""
    # Every product is static: the arrangement is the experiment's independent variable, so nothing
    # may settle, roll, or merge into a heap.
    products: list[dict[str, Any]] = []
    for column in columns:
        entry = CATALOGUE.get(column.geometry_key)
        if entry is None:
            raise ValueError(f"unknown geometry {column.geometry_key}")
        if column.count < 0 or column.count > entry["lane_capacity"]:
            raise ValueError(
                f"{column.lane_id} cannot hold {column.count} of {column.geometry_key}: "
                f"the lane takes {entry['lane_capacity']}"
            )
        for index in range(column.count):
            model_name = f"{column.lane_id}_{entry['product_class']}_{index + 1:02d}"
            products.append(
                {
                    "model_name": model_name,
                    "source_object_id": f"sim:{model_name}",
                    "geometry_key": column.geometry_key,
                    "product_class": entry["product_class"],
                    "sku": entry["sku"],
                    "mass_kg": entry["mass_kg"],
                    "rgba": list(entry["rgba"]),
                    "friction": entry["friction"],
                    "static": True,
                    "spawn_pose": spawn_pose_in_world(column.lane_id, column.geometry_key, index),
                }
            )
    if not products:
        raise ValueError("a scenario must spawn at least one product")
    return {
        "schema_version": 1,
        "pose_topic": "/world/restocking/pose/info",
        "frame_id": "world",
        "backend": {"name": "gazebo_ground_truth", "version": "harmonic_pose_v1"},
        "pose_covariance_diagonal": [1.0e-8] * 6,
        "workcell_pose": list(WORKCELL_POSE),
        "products": products,
    }


def ground_truth_available_depth_m(
    geometry_key: str,
    count: int,
    rear_clearance_m: float = 0.01,
    usable_depth_m: float = 0.85,
) -> float:
    """Return the free depth the ground-truth producer will report for this column."""
    # The same axis-aligned bound restocker_gazebo/src/lane_evidence.cpp takes of a tilted
    # cylinder, so expectation and producer agree and a disagreement in a run belongs to the
    # camera.
    if count == 0:
        return usable_depth_m
    entry = CATALOGUE[geometry_key]
    radius = entry["radius_m"]
    height = entry["height_m"]
    center_depth, _, _ = column_pose_in_lane(geometry_key, count - 1)
    axial = abs(math.sin(LANE_INCLINE_RAD))
    extent = (radius * math.sqrt(max(0.0, 1.0 - (axial * axial)))) + ((height / 2.0) * axial)
    return min(max(center_depth - extent - rear_clearance_m, 0.0), usable_depth_m)


# Baseline tray picks from baseline_products.yaml, kept here so a dense-shelf restock scenario can
# be rebuilt without that file; test_lane_columns.py checks the poses stay in step.
BASELINE_TRAY_PRODUCTS: list[dict[str, Any]] = [
    {
        "model_name": "stock_can_01",
        "source_object_id": "sim:stock_can_01",
        "geometry_key": "can.standard",
        "product_class": "can",
        "sku": "SIM-CAN-STD",
        "mass_kg": 0.355,
        "rgba": [0.78, 0.12, 0.08, 1.0],
        "friction": 0.65,
        "spawn_pose": [-0.42, -0.80, 0.631, 0.0, 0.0, 0.0],
    },
    {
        "model_name": "stock_small_bottle_01",
        "source_object_id": "sim:stock_small_bottle_01",
        "geometry_key": "bottle.small.standard",
        "product_class": "small_bottle",
        "sku": "SIM-BOTTLE-SMALL",
        "mass_kg": 0.500,
        "rgba": [0.10, 0.42, 0.82, 0.82],
        "friction": 0.72,
        "spawn_pose": [0.0, -0.80, 0.670, 0.0, 0.0, 0.0],
    },
    {
        "model_name": "stock_large_bottle_01",
        "source_object_id": "sim:stock_large_bottle_01",
        "geometry_key": "bottle.large.standard",
        "product_class": "large_bottle",
        "sku": "SIM-BOTTLE-LARGE",
        "mass_kg": 1.100,
        "rgba": [0.12, 0.68, 0.34, 0.82],
        "friction": 0.76,
        "spawn_pose": [0.42, -0.80, 0.715, 0.0, 0.0, 0.0],
    },
]


def dense_restock_columns() -> list[LaneColumn]:
    """Near-full columns for every lane, each leaving rear room for one tray transfer."""
    # Not capacity - 1: the last free slot from a pure capacity count can still be shorter than
    # 2 * radius + insert_entry_clearance_m (the placement predicate). Use the deepest count that
    # still reports available_depth_m >= that requirement so a tray grasp can finish.
    return [
        LaneColumn("lane_01", "can.standard", 10),
        LaneColumn("lane_02", "bottle.small.standard", 10),
        LaneColumn("lane_03", "bottle.large.standard", 7),
        LaneColumn("lane_04", "can.standard", 10),
        LaneColumn("lane_05", "bottle.small.standard", 10),
        LaneColumn("lane_06", "bottle.large.standard", 7),
    ]


# The visible front shelf starts partially stocked. Counts differ between sibling lanes so the
# fixture looks like customer depletion, and every lane is neither empty nor full. In aggregate the
# missing count per SKU equals the number in DENSE_RESTOCK_TRAY_GRIDS below, so one autonomous
# restock phase fills the shelf and terminates on NO_COMPATIBLE_PAIR with no surplus stock.
DENSE_RESTOCK_FRONT_COLUMNS: tuple[LaneColumn, ...] = (
    LaneColumn("lane_01", "can.standard", 8),
    LaneColumn("lane_02", "bottle.small.standard", 7),
    LaneColumn("lane_03", "bottle.large.standard", 4),
    LaneColumn("lane_04", "can.standard", 7),
    LaneColumn("lane_05", "bottle.small.standard", 6),
    LaneColumn("lane_06", "bottle.large.standard", 5),
)

# Back blocks do not repeat the shelf's first-occurrence order (can, small, large): left-to-right
# they are large, can, small. Each nominal 2-D grid has its back-right slot empty, so the delivery
# looks partially consumed while the layout stays regular and countable.
#
# The two columns are the simultaneously graspable dimension. Their 0.15/0.16 m centre pitches
# exceed the padded-wrist corridor plus the widest neighbouring product radius (0.0865 + 0.045 =
# 0.1315 m), leaving at least 18.5 mm of lateral planning clearance. Rows are feed queues: the
# robot takes the +Y/front item before the one behind it. The 0.085/0.110 m row pitches retain
# about 17/20 mm between product surfaces. Tighter packing and multiple layers are future work.
DENSE_RESTOCK_TRAY_GRIDS: tuple[TrayGrid, ...] = (
    TrayGrid(
        "bottle.large.standard",
        (-0.60, -0.44),
        (-0.68, -0.79, -0.90, -1.01),
        7,
    ),
    TrayGrid(
        "can.standard",
        (-0.15, 0.0),
        (-0.68, -0.765, -0.85, -0.935, -1.02),
        9,
    ),
    TrayGrid(
        "bottle.small.standard",
        (0.43, 0.58),
        (-0.68, -0.765, -0.85, -0.935, -1.02),
        9,
    ),
)

TRAY_SURFACE_Z_M = 0.570


def dense_restock_tray_products() -> list[dict[str, Any]]:
    """Build the partial per-SKU grids on the stock tray in their independent block order."""
    products: list[dict[str, Any]] = []
    for grid in DENSE_RESTOCK_TRAY_GRIDS:
        entry = CATALOGUE[grid.geometry_key]
        slots = [(x_m, y_m) for y_m in grid.row_y_m for x_m in grid.column_x_m]
        if grid.count <= 0 or grid.count >= len(slots):
            raise ValueError(
                f"dense tray grid {grid.geometry_key} must leave at least one of "
                f"its {len(slots)} slots empty"
            )
        for index, (x_m, y_m) in enumerate(slots[: grid.count], start=1):
            model_name = f"stock_{entry['product_class']}_{index:02d}"
            products.append(
                {
                    "model_name": model_name,
                    "source_object_id": f"sim:{model_name}",
                    "geometry_key": grid.geometry_key,
                    "product_class": entry["product_class"],
                    "sku": entry["sku"],
                    "mass_kg": entry["mass_kg"],
                    "rgba": list(entry["rgba"]),
                    "friction": entry["friction"],
                    "spawn_pose": [
                        x_m,
                        y_m,
                        TRAY_SURFACE_Z_M + (entry["height_m"] / 2.0),
                        0.0,
                        0.0,
                        0.0,
                    ],
                }
            )
    return products


def dense_restock_scenario(
    tray_products: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Partial front columns plus partial, dense stock-side grids for every SKU."""
    front = lane_column_scenario(list(DENSE_RESTOCK_FRONT_COLUMNS))["products"]
    tray = list(dense_restock_tray_products() if tray_products is None else tray_products)
    if not tray:
        raise ValueError("dense restock scenario requires at least one tray product")
    for product in tray:
        if product.get("static"):
            raise ValueError(f"tray product {product.get('model_name')} must not be static")
    return {
        "schema_version": 1,
        "pose_topic": "/world/restocking/pose/info",
        "frame_id": "world",
        "backend": {"name": "gazebo_ground_truth", "version": "harmonic_pose_v1"},
        "pose_covariance_diagonal": [1.0e-8] * 6,
        "workcell_pose": list(WORKCELL_POSE),
        "products": front + tray,
    }


# Card 010 SC-001: three depleted lanes (lane_01/02/03 start empty with targets 1/1/1 in
# sensor_acceptance_lanes.yaml; siblings want none). Front products are absent on purpose:
# ObservationIdentityAssigner only identifies a class the object manifest stocks exactly one
# of, and object_manifest_path is the whole scenario document — any front column of the same
# class as a tray product makes camera identity undecidable and every tray detection is
# dropped (acceptance attempt 1, 2026-09-24). One tray product per class is the whole cell's
# inventory for the camera path. Card 063 since names repeated classes by declared stocking
# position; this fixture keeps its one-per-class design as the proven acceptance table.
SENSOR_ACCEPTANCE_FRONT_COLUMNS: tuple[LaneColumn, ...] = ()

# Unordered mixed tray: exactly one product per class (identity-decidable), classes interleaved
# in x and y — not the baseline's ordered row and not the dense fixture's per-SKU blocks.
# Surface gaps stay above the isolated 0.04 m floor and every y stays inside the reachable band
# (pre-grasp bound MINIMUM_PREGRASP_REACH_Y_M = 0.34 from the rail at y = 0; the far tray edge
# sits near y = -0.955).
# (geometry_key, world x, world y); z is the upright pose on TRAY_SURFACE_Z_M.
SENSOR_ACCEPTANCE_TRAY_LAYOUT: tuple[tuple[str, float, float], ...] = (
    ("bottle.small.standard", -0.46, -0.74),
    ("can.standard", -0.04, -0.88),
    ("bottle.large.standard", 0.40, -0.66),
)


def sensor_acceptance_tray_products() -> list[dict[str, Any]]:
    """Build the unordered mixed tray: movable products at the interleaved layout poses."""
    products: list[dict[str, Any]] = []
    for index, (geometry_key, x_m, y_m) in enumerate(SENSOR_ACCEPTANCE_TRAY_LAYOUT, start=1):
        entry = CATALOGUE[geometry_key]
        model_name = f"stock_{entry['product_class']}_{index:02d}"
        products.append(
            {
                "model_name": model_name,
                "source_object_id": f"sim:{model_name}",
                "geometry_key": geometry_key,
                "product_class": entry["product_class"],
                "sku": entry["sku"],
                "mass_kg": entry["mass_kg"],
                "rgba": list(entry["rgba"]),
                "friction": entry["friction"],
                "spawn_pose": [
                    x_m,
                    y_m,
                    TRAY_SURFACE_Z_M + (entry["height_m"] / 2.0),
                    0.0,
                    0.0,
                    0.0,
                ],
            }
        )
    return products


def sensor_acceptance_scenario(
    tray_products: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Empty depleted lanes (targets in sensor_acceptance_lanes) plus the mixed tray."""
    front: list[dict[str, Any]] = []
    if SENSOR_ACCEPTANCE_FRONT_COLUMNS:
        front = lane_column_scenario(list(SENSOR_ACCEPTANCE_FRONT_COLUMNS))["products"]
    tray = list(sensor_acceptance_tray_products() if tray_products is None else tray_products)
    if not tray:
        raise ValueError("sensor acceptance scenario requires at least one tray product")
    for product in tray:
        if product.get("static"):
            raise ValueError(f"tray product {product.get('model_name')} must not be static")
    classes = [product["product_class"] for product in tray]
    if sorted(classes) != ["can", "large_bottle", "small_bottle"]:
        raise ValueError(
            "sensor acceptance tray must stock exactly one product per class for camera "
            f"identity (ObservationIdentityAssigner), got {classes}"
        )
    return {
        "schema_version": 1,
        "pose_topic": "/world/restocking/pose/info",
        "frame_id": "world",
        "backend": {"name": "gazebo_ground_truth", "version": "harmonic_pose_v1"},
        "pose_covariance_diagonal": [1.0e-8] * 6,
        "workcell_pose": list(WORKCELL_POSE),
        "products": front + tray,
    }
