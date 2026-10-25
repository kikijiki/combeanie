# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Proposal benchmark scenes (CMB-113): cluttered trays, lying and contour products, views."""
# The identity benchmark's scenes hold at most three tray products, all upright and far apart, and
# its wrist frames show one product at a time, so they cannot say whether geometry finds products
# in a cluttered tray. These scenes can: a seed draws one of six tray arrangements (sparse, dense
# with touching neighbours, tight rows, lying products, contoured bottles, tall products in front
# of short ones) on top of the identity benchmark's lane columns, and the recorder drives the wrist
# camera to the shipped survey stations (tray overview, lane) instead of only to close views.
#
# Pure Python, no ROS, pure in the seed. Every tray product is pinned (``static``) so physics
# cannot move a lying or touching product off its declared pose; the recorder still refuses a
# frame whose products are not within 3 cm of the declared poses.

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Any

from restocker_gazebo.lane_columns import (  # noqa: I100,I101 - ament and Ruff disagree
    BOX_CATALOGUE,
    LANE_CENTER_X_M,
    TRAY_SURFACE_Z_M,
    WORKCELL_POSE,
)
from restocker_gazebo.scenario_random import SplitMix64

from restocker_benchmarks import perception_dataset as dataset  # noqa: I100
from restocker_benchmarks.perception_views import look_at_optical  # noqa: I100

PROPOSAL_SCENES_VERSION = 1
TAG = "prop"

# Seeds are disjoint from the identity benchmark's (1..234). Development seeds are the only ones
# a tolerance may be tuned on; held-out seeds are read once, after the bounds file is committed.
DEVELOPMENT_SEEDS = tuple(range(1001, 1025))
HELD_OUT_SEEDS = tuple(range(2001, 2049))
# CMB-119: box scenes (cartons and a slim box on the tray, box columns in the lanes). Their own
# development seeds, so no existing seed changes its scene type. 1101..1112 are the CMB-119
# diagnostic's; 1113..1136 were appended by CMB-120 (append-only: nothing before them moved).
BOX_DEVELOPMENT_SEEDS = tuple(range(1101, 1137))
# CMB-120: held-out box scenes, declared before any of them was recorded. Read once, after the
# acceptance bounds are committed.
BOX_HELD_OUT_SEEDS = tuple(range(2101, 2137))

SCENE_TYPES = ("sparse", "dense", "rows", "lying", "contour", "tall_front")

# (radius m, height m) of every drawable shape, the collision catalogue's numbers.
SHAPES = {
    "can": (0.033, 0.122),
    "small_bottle": (0.034, 0.200),
    "large_bottle": (0.045, 0.290),
    "contour_bottle": (0.036, 0.230),
}
# Boxes: the footprint is the circumscribed circle of the width x depth rectangle (a capsule when
# lying), so two boxes are never closer than the circles allow: a box neighbour is never
# truly touching and a box's corners are not packed. Stated in the diagnostic note.
for _variant, _entry in BOX_CATALOGUE.items():
    _sx, _sy, _sz = _entry["size_xyz_m"]
    SHAPES["box_" + _variant.split(".", 1)[1]] = (0.5 * math.hypot(_sx, _sy), _sz)
TRAY_X_RANGE = (-0.68, 0.68)
TRAY_Y_RANGE = (-1.055, -0.545)
WALL_MARGIN_M = 0.01
# Surface-to-surface gaps a growing cluster draws from: touching, a finger, a hand.
CLUSTER_GAPS_M = (0.0005, 0.004, 0.012, 0.03)

_MASK64 = (1 << 64) - 1


def scene_type_of_seed(seed: int) -> str:
    """The tray arrangement a seed draws; cycles so every type is balanced in a seed range."""
    if seed in BOX_DEVELOPMENT_SEEDS or seed in BOX_HELD_OUT_SEEDS:
        return "box"
    return SCENE_TYPES[seed % len(SCENE_TYPES)]


def split_of_proposal_seed(seed: int) -> str:
    if seed in DEVELOPMENT_SEEDS:
        return "development"
    if seed in HELD_OUT_SEEDS:
        return "held_out"
    if seed in BOX_DEVELOPMENT_SEEDS:
        return "development"
    if seed in BOX_HELD_OUT_SEEDS:
        return "held_out"
    raise ValueError(f"seed {seed} is not a proposal-benchmark seed")


# --------------------------------------------------------------------------------------------
# Footprints
# --------------------------------------------------------------------------------------------


def _segment_distance(
    p0: tuple[float, float],
    p1: tuple[float, float],
    q0: tuple[float, float],
    q1: tuple[float, float],
) -> float:
    """Minimum distance between two 2-D segments (either may be a point)."""

    def point_to_segment(p, a, b):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length_sq = dx * dx + dy * dy
        t = (
            0.0
            if length_sq == 0
            else max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / length_sq))
        )
        return math.hypot(p[0] - (a[0] + t * dx), p[1] - (a[1] + t * dy))

    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    o1, o2 = orient(p0, p1, q0), orient(p0, p1, q1)
    o3, o4 = orient(q0, q1, p0), orient(q0, q1, p1)
    if o1 * o2 < 0 and o3 * o4 < 0:
        return 0.0
    return min(
        point_to_segment(p0, q0, q1),
        point_to_segment(p1, q0, q1),
        point_to_segment(q0, p0, p1),
        point_to_segment(q1, p0, p1),
    )


class Footprint:
    """A product's footprint on the tray: a capsule (a point with a radius when upright)."""

    def __init__(self, shape: str, x: float, y: float, lying: bool, yaw: float) -> None:
        radius, height = SHAPES[shape]
        self.shape, self.x, self.y, self.lying, self.yaw = shape, x, y, lying, yaw
        self.radius = radius
        half = max(0.0, height / 2.0 - radius) if lying else 0.0
        # Lying: the axis lies along (cos yaw, sin yaw) in the tray plane.
        self.a = (x - half * math.cos(yaw), y - half * math.sin(yaw))
        self.b = (x + half * math.cos(yaw), y + half * math.sin(yaw))

    def gap_to(self, other: Footprint) -> float:
        """Surface-to-surface distance in the tray plane (negative: overlapping)."""
        return _segment_distance(self.a, self.b, other.a, other.b) - self.radius - other.radius

    def inside_tray(self) -> bool:
        lo_x, hi_x = TRAY_X_RANGE[0] + WALL_MARGIN_M, TRAY_X_RANGE[1] - WALL_MARGIN_M
        lo_y, hi_y = TRAY_Y_RANGE[0] + WALL_MARGIN_M, TRAY_Y_RANGE[1] - WALL_MARGIN_M
        r = self.radius
        return all(
            lo_x + r <= px <= hi_x - r and lo_y + r <= py <= hi_y - r
            for px, py in (self.a, self.b)
        )


def _grow(
    stream: SplitMix64,
    footprints: list[Footprint],
    candidate_factory,
    gap: float,
) -> Footprint | None:
    """Place a footprint adjacent to an existing one at ``gap``; None after 60 tries."""
    for _ in range(60):
        anchor = footprints[stream.next_below(len(footprints))]
        angle = stream.next_unit() * 2.0 * math.pi
        candidate = candidate_factory()
        # Walk outward from the anchor along the angle until clear, then back off to the gap.
        direction = (math.cos(angle), math.sin(angle))
        low, high = 0.0, 0.5
        for _ in range(22):
            mid = (low + high) / 2.0
            probe = Footprint(
                candidate.shape,
                anchor.x + mid * direction[0],
                anchor.y + mid * direction[1],
                candidate.lying,
                candidate.yaw,
            )
            if probe.gap_to(anchor) < gap:
                low = mid
            else:
                high = mid
        placed = Footprint(
            candidate.shape,
            anchor.x + high * direction[0],
            anchor.y + high * direction[1],
            candidate.lying,
            candidate.yaw,
        )
        if placed.inside_tray() and all(placed.gap_to(o) >= 0.0002 for o in footprints):
            return placed
    return None


def _free_position(
    stream: SplitMix64, footprints: list[Footprint], candidate_factory, minimum_gap: float
) -> Footprint | None:
    for _ in range(200):
        candidate = candidate_factory()
        x = TRAY_X_RANGE[0] + 0.05 + stream.next_unit() * (TRAY_X_RANGE[1] - TRAY_X_RANGE[0] - 0.1)
        y = TRAY_Y_RANGE[0] + 0.05 + stream.next_unit() * (TRAY_Y_RANGE[1] - TRAY_Y_RANGE[0] - 0.1)
        placed = Footprint(candidate.shape, x, y, candidate.lying, candidate.yaw)
        if placed.inside_tray() and all(placed.gap_to(o) >= minimum_gap for o in footprints):
            return placed
    return None


# --------------------------------------------------------------------------------------------
# Scenes
# --------------------------------------------------------------------------------------------


def _tray_plan(seed: int, stream: SplitMix64, kind: str) -> list[Footprint]:
    """Footprints of one tray arrangement; the shape pool and clustering depend on the kind."""
    shapes = ("can", "small_bottle", "large_bottle")
    counts = {
        "sparse": (2, 3),
        "dense": (8, 12),
        "rows": (10, 14),
        "lying": (5, 8),
        "contour": (5, 9),
        "tall_front": (6, 10),
        "box": (5, 9),
    }[kind]
    count = counts[0] + stream.next_below(counts[1] - counts[0] + 1)
    plan: list[Footprint] = []

    box_shapes = tuple(sorted(name for name in SHAPES if name.startswith("box_")))

    def shape_for(index: int) -> str:
        if kind == "box":
            # Mostly boxes, a can or a bottle for company.
            if stream.next_unit() < 0.8:
                return box_shapes[stream.next_below(len(box_shapes))]
            return shapes[stream.next_below(len(shapes))]
        if kind == "contour":
            # Mostly contoured bottles, a can or two for company.
            return "contour_bottle" if stream.next_unit() < 0.75 else "can"
        if kind == "tall_front":
            # Short products first (they end up behind), tall ones are added in front below.
            return (
                "can"
                if index < count // 2
                else ("large_bottle" if stream.next_unit() < 0.6 else "small_bottle")
            )
        return shapes[stream.next_below(len(shapes))]

    def lying_for(shape: str) -> bool:
        if kind == "lying":
            return stream.next_unit() < 0.6
        if kind in ("contour", "box"):
            return stream.next_unit() < 0.3
        return False

    for index in range(count):
        shape = shape_for(index)
        lying = lying_for(shape)
        yaw = stream.next_unit() * math.pi
        if kind == "rows":
            yaw = 0.0
        gap_choice = CLUSTER_GAPS_M[stream.next_below(len(CLUSTER_GAPS_M))]

        def factory(shape=shape, lying=lying, yaw=yaw):
            return Footprint(shape, 0.0, 0.0, lying, yaw)

        if not plan:
            placed = _free_position(stream, [], factory, 0.0)
        elif kind == "sparse":
            placed = _free_position(stream, plan, factory, 0.04)
        elif kind == "rows":
            placed = _row_position(stream, plan, factory)
        elif kind == "tall_front" and index >= count // 2:
            placed = _front_position(stream, plan, factory)
        else:
            placed = _grow(stream, plan, factory, gap_choice)
        if placed is not None:
            plan.append(placed)
    return plan


def _row_position(stream: SplitMix64, plan: list[Footprint], factory) -> Footprint | None:
    """Tight rows: extend the cluster sideways along x at touching distance, then new rows."""
    for _ in range(80):
        anchor = plan[stream.next_below(len(plan))]
        candidate = factory()
        side = 1.0 if stream.next_unit() < 0.5 else -1.0
        direction = (side, 0.0) if stream.next_unit() < 0.75 else (0.0, side)
        low, high = 0.0, 0.5
        for _ in range(22):
            mid = (low + high) / 2.0
            probe = Footprint(
                candidate.shape,
                anchor.x + mid * direction[0],
                anchor.y + mid * direction[1],
                False,
                0.0,
            )
            low, high = (mid, high) if probe.gap_to(anchor) < 0.0005 else (low, mid)
        placed = Footprint(
            candidate.shape,
            anchor.x + high * direction[0],
            anchor.y + high * direction[1],
            False,
            0.0,
        )
        if placed.inside_tray() and all(placed.gap_to(o) >= 0.0002 for o in plan):
            return placed
    return None


def _front_position(stream: SplitMix64, plan: list[Footprint], factory) -> Footprint | None:
    """Put a tall product just in front of (toward the camera, +y) an existing one."""
    for _ in range(80):
        anchor = plan[stream.next_below(len(plan))]
        candidate = factory()
        offset_x = (stream.next_unit() - 0.5) * 0.04
        low, high = 0.0, 0.4
        for _ in range(22):
            mid = (low + high) / 2.0
            probe = Footprint(candidate.shape, anchor.x + offset_x, anchor.y + mid, False, 0.0)
            low, high = (mid, high) if probe.gap_to(anchor) < 0.003 else (low, mid)
        placed = Footprint(candidate.shape, anchor.x + offset_x, anchor.y + high, False, 0.0)
        if placed.inside_tray() and all(placed.gap_to(o) >= 0.0002 for o in plan):
            return placed
    return None


def build_proposal_scene(seed: int, products: Sequence[dict[str, Any]]) -> dict[str, Any]:
    """One proposal-benchmark scene: the seed's lane columns plus a drawn tray arrangement.

    Every catalogue product (contoured bottles included) may appear: this benchmark scores
    where things are, not which product they are. Returns the same keys as
    ``perception_dataset.build_scene`` plus ``tray_kind`` and ``truth``.
    """
    kind = scene_type_of_seed(seed)
    base = dataset.build_scene(
        seed,
        [p for p in products if p.get("shape") != "contour_bottle"],
        include_boxes=kind == "box",
    )
    scenario = base["scenario"]
    lane_products = [
        p for p in scenario["products"] if dataset.lane_of_pose(p["spawn_pose"][:3])[0] is not None
    ]
    scenario["products"] = lane_products

    stream = SplitMix64(
        (seed * 0x9E3779B97F4A7C15 + 0x50524F50 + PROPOSAL_SCENES_VERSION) & _MASK64
    )
    stream.next_u64()
    by_shape: dict[str, list[dict[str, Any]]] = {}
    for row in sorted(products, key=lambda item: item["sku"]):
        key = f"box_{row['box_variant']}" if row["shape"] == "box" else row["shape"]
        by_shape.setdefault(key, []).append(row)
    plan = _tray_plan(seed, stream, kind)
    for slot, footprint in enumerate(plan, start=1):
        row = by_shape[footprint.shape][stream.next_below(len(by_shape[footprint.shape]))]
        radius, height = SHAPES[footprint.shape]
        is_box = footprint.shape.startswith("box_")
        name = f"tray_{footprint.shape}_{slot:02d}"
        if is_box and footprint.lying:
            # On its back (front face up): the bounding capsule's axis is the box's height axis.
            sy = BOX_CATALOGUE["box." + footprint.shape[4:]]["size_xyz_m"][1]
            pose = [
                footprint.x,
                footprint.y,
                TRAY_SURFACE_Z_M + sy / 2.0,
                -math.pi / 2.0,
                0.0,
                footprint.yaw - math.pi / 2.0,
            ]
        elif footprint.lying:
            pose = [
                footprint.x,
                footprint.y,
                TRAY_SURFACE_Z_M + radius,
                math.pi / 2.0,
                0.0,
                footprint.yaw - math.pi / 2.0,
            ]
        else:
            yaw = stream.next_unit() * 2.0 * math.pi
            pose = [footprint.x, footprint.y, TRAY_SURFACE_Z_M + height / 2.0, 0.0, 0.0, yaw]
        product = {
            "model_name": name,
            "source_object_id": f"sim:{name}",
            "geometry_key": row["geometry_key"],
            "product_class": "box" if is_box else footprint.shape,
            "sku": row["sku"],
            "mass_kg": (
                BOX_CATALOGUE["box." + footprint.shape[4:]]["mass_kg"]
                if is_box
                else {
                    "can": 0.355,
                    "small_bottle": 0.5,
                    "large_bottle": 1.1,
                    "contour_bottle": 0.75,
                }[footprint.shape]
            ),
            "rgba": [1.0, 1.0, 1.0, 1.0],
            "friction": 0.55 if is_box else 0.7,
            "spawn_pose": [round(v, 6) for v in pose],
            "static": True,
            "label_texture": row["texture"],
        }
        if footprint.shape == "contour_bottle":
            product["visual_mesh"] = "contour_bottle.obj"
        if is_box:
            product["visual_mesh"] = row["visual_mesh"]
        scenario["products"].append(product)
    truth = dataset.scene_truth(scenario, base["lane_expected_sku"])
    return {
        "scenario": scenario,
        "lane_expected_sku": base["lane_expected_sku"],
        "truth": truth,
        "tray_kind": kind,
        "scenario_id": dataset.scenario_id(seed, TAG),
        "seed_split": split_of_proposal_seed(seed),
    }


# The simulator stack (ground-truth adapter, planning-scene projector) refuses the contour class,
# which has no message enum value and no executor support. Contoured bottles are drawn for the
# camera only: the scenario the simulator is given presents each as a small bottle of the same
# geometry (collision and class) while keeping its own label texture and visual mesh. The
# products are pinned, so the collision shape never acts; the labels and the ground-truth masks
# come from the unsubstituted scenario, with the contour's own radius and height.
SIM_STAND_IN = {
    "geometry_key": "small.bottle.gen005",
    "sku": "GEN-005",
    "product_class": "small_bottle",
}


def simulator_scenario(scenario: dict[str, Any]) -> dict[str, Any]:
    """The scenario document the simulator is launched with (contours stood in for)."""
    out = dict(scenario)
    out["products"] = []
    for product in scenario["products"]:
        product = dict(product)
        if product["product_class"] == "contour_bottle":
            product.update(SIM_STAND_IN)
        out["products"].append(product)
    return out


# --------------------------------------------------------------------------------------------
# Survey views
# --------------------------------------------------------------------------------------------

TRAY_STATION_X_M = (-0.34, 0.34)
TRAY_CENTRE_WORLD = (0.0, -0.80, WORKCELL_POSE[2] - 0.005)
TRAY_CAMERA_OFFSET = (0.0, 0.45, 0.555)  # survey_stations.hpp tray_camera_setback / height
LANE_CAMERA_SETBACK_M = 0.55
LANE_CAMERA_HEIGHT_M = 0.55
LANE_AIM_DEPTH_M = 0.435  # rear clearance 0.01 + half of the 0.85 m usable depth


def survey_views(seed: int, truth: dict[str, Any], lane_count: int = 2) -> list[dict[str, Any]]:
    """The shipped tray stations and lane stations for this scene, as camera poses.

    The two tray stations are always requested; ``lane_count`` seeded lane stations are chosen
    among the lanes that hold a product. Names: ``tray_a``, ``tray_b``, ``lane_NN``. No view has a
    target product, so the recorder does not check a target range for them.
    """
    views = []
    for name, x in zip(("tray_a", "tray_b"), TRAY_STATION_X_M, strict=True):
        aim = (x, TRAY_CENTRE_WORLD[1], TRAY_CENTRE_WORLD[2])
        eye = tuple(a + o for a, o in zip(aim, TRAY_CAMERA_OFFSET, strict=True))
        views.append(_view(name, eye, aim))
    lanes = sorted(row["lane_id"] for row in truth["lanes"] if row["count"] > 0)
    stream = SplitMix64((seed * 0x9E3779B97F4A7C15 + 0x4C414E45) & _MASK64)
    stream.next_u64()
    chosen: list[str] = []
    pool = list(lanes)
    while pool and len(chosen) < lane_count:
        chosen.append(pool.pop(stream.next_below(len(pool))))
    for lane_id in sorted(chosen):
        x = LANE_CENTER_X_M[lane_id] + WORKCELL_POSE[0]
        eye = (
            x,
            WORKCELL_POSE[1] - LANE_CAMERA_SETBACK_M,
            WORKCELL_POSE[2] + LANE_CAMERA_HEIGHT_M,
        )
        aim = (x, WORKCELL_POSE[1] + LANE_AIM_DEPTH_M, WORKCELL_POSE[2])
        views.append(_view(lane_id, eye, aim))
    return views


def _view(name: str, eye: Sequence[float], aim: Sequence[float]) -> dict[str, Any]:
    pose = [round(float(v), 6) for v in eye] + [round(v, 9) for v in look_at_optical(eye, aim)]
    return {
        "name": name,
        "target_object_id": None,
        "range_check": False,
        "standoff_m": round(math.dist(eye, aim), 4),
        "camera_optical_pose": pose,
    }
