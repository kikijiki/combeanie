# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Perception benchmark dataset: seeded scenes, ground-truth labels, content-hashed directory."""
# Everything here is a pure function of its inputs and imports nothing from ROS, so the label
# schema, the scene builder and the dataset writer are unit-tested without a simulator. The live
# recorder (restocker_bringup/test/test_perception_recorder_runtime.py) only supplies images,
# camera poses and intrinsics; the labels come from the scenario document, whose poses are what
# Gazebo is told to spawn.
#
# Determinism, stated exactly. For fixed seeds, catalogue and generator version the scenario
# documents and the label content (everything except camera pose and image bytes) are
# byte-identical on any machine: ``labels_hash``. The ``dataset_hash`` additionally covers the
# recorded image bytes and camera poses, which come from a renderer and can differ between GPUs
# and driver versions. Both are in the manifest so the difference is visible, not hidden.

from __future__ import annotations

from collections.abc import Iterable, Sequence
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess  # noqa: S404 - fixed argv, no shell
from typing import Any
import xml.etree.ElementTree as ElementTree
import zlib

from restocker_gazebo.lane_columns import (  # noqa: I100,I101 - ament and Ruff disagree
    BOX_CATALOGUE,
    CATALOGUE,
    LANE_CENTER_X_M,
    SENSOR_ACCEPTANCE_TRAY_LAYOUT,
    TRAY_SURFACE_Z_M,
    WORKCELL_POSE,
    LaneColumn,
    lane_column_scenario,
    product_entry,
)
from restocker_gazebo.scenario_random import SplitMix64
import yaml

SCHEMA_VERSION = 1
GENERATOR_VERSION = 2  # 2: tray products get a random yaw, so the label faces any direction
DATASET_KIND = "restocker_perception_benchmark"

LANE_IDS = tuple(LANE_CENTER_X_M)
LANE_HALF_WIDTH_M = 0.2
LANE_MAX_DEPTH_M = 0.84
SENSORS = ("wrist", "overhead")
# A scene holds at most this many products per lane, below the physical capacity, to keep the
# spawn count (and therefore the one simulator run) small.
MAX_COLUMN_COUNT = 6
# Probability that a non-empty lane carries one product of the wrong SKU, and that an empty lane
# is left empty. Fixed here so a seed means the same scene in every checkout.
MISPLACED_PROBABILITY = 0.3
EMPTY_LANE_PROBABILITY = 0.2
TRAY_SLOT_PROBABILITY = 0.7


# --------------------------------------------------------------------------------------------
# Hashing helpers
# --------------------------------------------------------------------------------------------


def sha256_bytes(data: bytes) -> str:
    """Return the hex SHA-256 of ``data``."""
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: str | Path) -> str:
    """Return the hex SHA-256 of a file's bytes."""
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_json(value: Any) -> str:
    """Serialise ``value`` identically on every machine (sorted keys, no spaces)."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def git_state(repository: str | Path) -> dict[str, Any]:
    """Return the checkout's commit and whether the tree had uncommitted changes."""

    def _run(*argv: str) -> str:
        completed = subprocess.run(  # noqa: S603 - fixed argv, no shell
            ["git", "-C", str(repository), *argv],
            check=False,
            capture_output=True,
            text=True,
        )
        return completed.stdout.strip() if completed.returncode == 0 else ""

    sha = _run("rev-parse", "HEAD")
    return {"sha": sha or "unknown", "dirty": bool(_run("status", "--porcelain")) if sha else None}


# --------------------------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------------------------


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> list[float]:
    """Return [x, y, z, w] for the extrinsic-XYZ (ROS rpy) rotation."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def rotation_matrix(quaternion: Sequence[float]) -> list[list[float]]:
    """Return the 3x3 rotation matrix of an [x, y, z, w] quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def project_point(
    point_world: Sequence[float],
    camera_pose_world: Sequence[float],
    intrinsics: dict[str, float],
) -> dict[str, Any]:
    """Project a world point through a pinhole camera; the pose is the optical frame in world."""
    # ``camera_pose_world`` is [x, y, z, qx, qy, qz, qw] of the optical frame (z forward, x right,
    # y down) expressed in world, so R maps optical coordinates to world coordinates.
    # Coerce to plain floats: rclpy hands back numpy scalars, whose bools are not JSON.
    camera_pose_world = [float(v) for v in camera_pose_world]
    intrinsics = {key: float(value) for key, value in intrinsics.items()}
    rotation = rotation_matrix(camera_pose_world[3:7])
    delta = [point_world[i] - camera_pose_world[i] for i in range(3)]
    camera = [sum(rotation[row][col] * delta[row] for row in range(3)) for col in range(3)]
    depth = camera[2]
    if depth <= 1e-6:
        return {"u": None, "v": None, "depth_m": depth, "in_view": False}
    u = intrinsics["fx"] * camera[0] / depth + intrinsics["cx"]
    v = intrinsics["fy"] * camera[1] / depth + intrinsics["cy"]
    inside = bool(0.0 <= u < intrinsics["width"] and 0.0 <= v < intrinsics["height"])
    return {"u": round(u, 3), "v": round(v, 3), "depth_m": round(depth, 4), "in_view": inside}


def lane_of_pose(position: Sequence[float]) -> tuple[str | None, float | None]:
    """Return (lane id, depth in metres from the lane's rear datum) or (None, None) for tray."""
    depth = position[1] - WORKCELL_POSE[1]
    if depth <= 0.0 or depth > LANE_MAX_DEPTH_M + 0.1:
        return None, None
    for lane_id, centre in LANE_CENTER_X_M.items():
        if abs(position[0] - (centre + WORKCELL_POSE[0])) <= LANE_HALF_WIDTH_M:
            return lane_id, round(depth, 6)
    return None, None


# --------------------------------------------------------------------------------------------
# Catalogue and scenes
# --------------------------------------------------------------------------------------------


def load_generated_catalogue(
    path: str | Path, *, include_boxes: bool = False
) -> list[dict[str, Any]]:
    """Load generated_product_labels.yaml into product rows (sku, geometry_key, lookalike).

    The default is the view splits v1 and v2 froze: no contour bottle (Card 110: visual only) and
    no box. ``include_boxes`` adds the box SKUs (CMB-119), which splits v3 pins.
    """
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    skipped = {"contour_bottle"} | (set() if include_boxes else {"box"})
    return [dict(row) for row in document["products"] if row.get("shape") not in skipped]


def load_catalogue_for_splits(
    path: str | Path, split_document: dict[str, Any]
) -> list[dict[str, Any]]:
    """The catalogue view a split file was frozen against (boxes only for splits v3)."""
    return load_generated_catalogue(
        path, include_boxes=bool(split_document.get("catalogue_includes_boxes"))
    )


def legacy_catalogue() -> list[dict[str, Any]]:
    """The three shipped colour-signature SKUs, in the same row shape."""
    return [
        {
            "sku": entry["sku"],
            "geometry_key": key,
            "shape": entry["product_class"],
            "lookalike_group": None,
        }
        for key, entry in CATALOGUE.items()
    ]


def catalogue_sha256(products: Iterable[dict[str, Any]]) -> str:
    """Fingerprint a product list by the fields a scene draw depends on."""
    rows = sorted(
        (row["sku"], row["geometry_key"], row.get("lookalike_group") or "") for row in products
    )
    return sha256_bytes(canonical_json(rows).encode())


def _pick(stream: SplitMix64, items: Sequence[Any]) -> Any:
    return items[stream.next_below(len(items))]


def _other_sku(stream: SplitMix64, wanted: dict[str, Any], pool: list[dict[str, Any]]) -> str:
    """A different SKU of the same geometry, preferring the look-alike twin (the hard case)."""
    others = [row for row in pool if row["sku"] != wanted["sku"]]
    twins = [
        row
        for row in others
        if wanted.get("lookalike_group")
        and (row.get("lookalike_group") == wanted["lookalike_group"])
    ]
    if twins:
        return _pick(stream, twins)["sku"]
    return _pick(stream, others)["sku"]


def build_scene(
    seed: int,
    products: Sequence[dict[str, Any]],
    *,
    allowed_skus: Iterable[str] | None = None,
    include_boxes: bool = False,
) -> dict[str, Any]:
    """Draw one scene: lane columns (some mixed), a few tray products, all pure in ``seed``.

    Returns ``{"scenario": <scenario document>, "lane_expected_sku": {lane: sku|None}}``. When
    ``allowed_skus`` is given only those SKUs appear, which is how train and validation seeds
    stay free of held-out products.

    Boxes (CMB-119) are drawn only with ``include_boxes``, so a seed drawn before they existed
    keeps its meaning: splits v3 declares the first seed that may include them
    (``perception_splits.scene_includes_boxes``). With it, box columns stand in lanes (yaw 0, the
    lane camera sees the front face) and boxes stand or lie on the tray at a random yaw.
    """
    allowed = None if allowed_skus is None else set(allowed_skus)
    pool = [
        row
        for row in products
        if (allowed is None or row["sku"] in allowed)
        and (include_boxes or row.get("shape") != "box")
    ]
    by_geometry: dict[str, list[dict[str, Any]]] = {}
    # Generated SKUs have their own collision geometry keys (can.gen001) with the base shape's
    # dimensions, so lanes are laid out on the base geometry and each product is then re-keyed.
    # A shape with no base geometry (the contour bottle) is not drawn.
    base_of_shape = {entry["product_class"]: key for key, entry in CATALOGUE.items()}
    texture_of_sku: dict[str, str] = {}
    mesh_of_sku: dict[str, str] = {}
    geometry_of_sku: dict[str, str] = {}
    for row in sorted(pool, key=lambda item: item["sku"]):
        base = base_of_shape.get(row.get("shape", ""))
        if row.get("shape") == "box":
            base = f"box.{row['box_variant']}"
        if base is None and row["geometry_key"] in CATALOGUE:
            base = row["geometry_key"]
        if base is None:
            continue
        by_geometry.setdefault(base, []).append(row)
        if row.get("texture"):
            texture_of_sku[row["sku"]] = row["texture"]
            geometry_of_sku[row["sku"]] = row["geometry_key"]
        if row.get("visual_mesh"):
            mesh_of_sku[row["sku"]] = row["visual_mesh"]
    if not by_geometry:
        raise ValueError("no product in the allowed set has a spawnable geometry")
    geometries = sorted(by_geometry)

    stream = SplitMix64((seed * 0x9E3779B97F4A7C15 + GENERATOR_VERSION) & ((1 << 64) - 1))
    stream.next_u64()
    columns: list[LaneColumn] = []
    expected: dict[str, str | None] = {}
    sku_of_model: dict[str, str] = {}
    lane_rows: dict[str, tuple[dict[str, Any], int]] = {}
    for lane_id in LANE_IDS:
        if stream.next_unit() < EMPTY_LANE_PROBABILITY:
            expected[lane_id] = None
            continue
        geometry = _pick(stream, geometries)
        row = _pick(stream, by_geometry[geometry])
        capacity = product_entry(geometry)["lane_capacity"]
        count = 1 + stream.next_below(min(capacity, MAX_COLUMN_COUNT))
        columns.append(LaneColumn(lane_id, geometry, count))
        expected[lane_id] = row["sku"]
        lane_rows[lane_id] = (row, count)
    if not columns:
        geometry = geometries[0]
        row = by_geometry[geometry][0]
        columns.append(LaneColumn(LANE_IDS[0], geometry, 1))
        expected[LANE_IDS[0]] = row["sku"]
        lane_rows[LANE_IDS[0]] = (row, 1)

    scenario = lane_column_scenario(columns)
    cursor = 0
    for column in columns:
        row, count = lane_rows[column.lane_id]
        names = [p["model_name"] for p in scenario["products"][cursor : cursor + count]]
        cursor += count
        for name in names:
            sku_of_model[name] = row["sku"]
        # Misplaced draw is always consumed so the stream stays aligned with or without a twin.
        draw = stream.next_unit()
        index = stream.next_below(count)
        same_geometry = by_geometry[column.geometry_key]
        if draw < MISPLACED_PROBABILITY and len(same_geometry) > 1:
            sku_of_model[names[index]] = _other_sku(stream, row, same_geometry)

    if include_boxes:
        _add_tray_boxes(stream, scenario, sku_of_model, by_geometry)
    for slot, (geometry, x_m, y_m) in enumerate(SENSOR_ACCEPTANCE_TRAY_LAYOUT, start=1):
        include = stream.next_unit() < TRAY_SLOT_PROBABILITY
        candidates = by_geometry.get(geometry)
        if not candidates:
            continue
        row = _pick(stream, candidates)
        yaw = stream.next_unit() * 2.0 * math.pi
        if not include:
            continue
        entry = CATALOGUE[geometry]
        name = f"stock_{entry['product_class']}_{slot:02d}"
        scenario["products"].append(
            {
                "model_name": name,
                "source_object_id": f"sim:{name}",
                "geometry_key": geometry,
                "product_class": entry["product_class"],
                "sku": row["sku"],
                "mass_kg": entry["mass_kg"],
                "rgba": list(entry["rgba"]),
                "friction": entry["friction"],
                "spawn_pose": [
                    x_m,
                    y_m,
                    TRAY_SURFACE_Z_M + entry["height_m"] / 2.0,
                    0,
                    0,
                    round(yaw, 6),
                ],
            }
        )
        sku_of_model[name] = row["sku"]
    for product in scenario["products"]:
        if product["model_name"] in sku_of_model:
            product["sku"] = sku_of_model[product["model_name"]]
        texture = texture_of_sku.get(product["sku"])
        if texture:
            # The scenario validator ties a SKU to its catalogue geometry; a generated SKU's own
            # key has the base geometry's dimensions to the millimetre.
            product["geometry_key"] = geometry_of_sku[product["sku"]]
            product["label_texture"] = texture
            product["rgba"] = [1.0, 1.0, 1.0, 1.0]
        if product["product_class"] == "box":
            product["visual_mesh"] = mesh_of_sku[product["sku"]]
    return {"scenario": scenario, "lane_expected_sku": expected}


# Tray slots for boxes (world x, y). Separate from the cylinders' slots: a box is wider, so these
# keep a 0.12 m surface gap to their neighbours at any yaw.
BOX_TRAY_LAYOUT = ((-0.30, -0.62), (0.18, -0.96), (0.52, -0.80))
BOX_LYING_PROBABILITY = 0.3


def _add_tray_boxes(
    stream: SplitMix64,
    scenario: dict[str, Any],
    sku_of_model: dict[str, str],
    by_geometry: dict[str, list[dict[str, Any]]],
) -> None:
    """Append up to three tray boxes: each slot draws presence, SKU, yaw and standing/lying."""
    boxes = sorted(key for key in by_geometry if key in BOX_CATALOGUE)
    if not boxes:
        return
    for slot, (x_m, y_m) in enumerate(BOX_TRAY_LAYOUT, start=1):
        include = stream.next_unit() < TRAY_SLOT_PROBABILITY
        geometry = _pick(stream, boxes)
        row = _pick(stream, by_geometry[geometry])
        yaw = stream.next_unit() * 2.0 * math.pi
        lying = stream.next_unit() < BOX_LYING_PROBABILITY
        if not include:
            continue
        entry = BOX_CATALOGUE[geometry]
        sx, sy, sz = entry["size_xyz_m"]
        name = f"stock_box_{slot:02d}"
        if lying:
            # Rolled a quarter turn about -x: the back face goes down, the front face looks up
            # and the box's height axis lies along the tray (yawed by `yaw`).
            pose = [x_m, y_m, TRAY_SURFACE_Z_M + sy / 2.0, -math.pi / 2.0, 0.0, round(yaw, 6)]
        else:
            pose = [x_m, y_m, TRAY_SURFACE_Z_M + sz / 2.0, 0, 0, round(yaw, 6)]
        scenario["products"].append(
            {
                "model_name": name,
                "source_object_id": f"sim:{name}",
                "geometry_key": geometry,
                "product_class": "box",
                "sku": row["sku"],
                "mass_kg": entry["mass_kg"],
                "rgba": list(entry["rgba"]),
                "friction": entry["friction"],
                "spawn_pose": pose,
            }
        )
        sku_of_model[name] = row["sku"]


def scene_truth(
    scenario: dict[str, Any],
    lane_expected_sku: dict[str, str | None] | None = None,
) -> dict[str, Any]:
    """Derive object and lane ground truth from a scenario document (any shipped scenario).

    Lane order is front first: index 0 is the product against the retainer. Where the scene did
    not declare an expected SKU for a lane (shipped scenarios), the expected SKU is the lane's
    most frequent SKU, ties broken by the frontmost product of that SKU.
    """
    lanes: dict[str, list[dict[str, Any]]] = {lane: [] for lane in LANE_IDS}
    objects: list[dict[str, Any]] = []
    for product in scenario["products"]:
        pose = [float(value) for value in product["spawn_pose"]]
        lane_id, depth = lane_of_pose(pose[:3])
        entry = {
            "object_id": product["source_object_id"],
            "sku": product["sku"],
            "product_class": product["product_class"],
            "geometry_key": product["geometry_key"],
            "pose_world": [round(v, 6) for v in pose[:3]]
            + [round(v, 9) for v in quaternion_from_rpy(*pose[3:6])],
            "lane_id": lane_id,
            "lane_order": None,
            "misplaced": False,
        }
        if product["product_class"] == "box":
            # Boxes carry their size: the class names no single geometry. The variant is the
            # visual mesh's name (box_<variant>.obj), which the dataset builder always sets.
            variant = product["visual_mesh"][len("box_") : -len(".obj")]
            entry["size_xyz_m"] = list(BOX_CATALOGUE[f"box.{variant}"]["size_xyz_m"])
        objects.append(entry)
        if lane_id is not None:
            entry["_depth"] = depth
            lanes[lane_id].append(entry)

    lane_rows: list[dict[str, Any]] = []
    for lane_id in LANE_IDS:
        members = sorted(lanes[lane_id], key=lambda item: -item["_depth"])
        for order, member in enumerate(members):
            member["lane_order"] = order
        wanted = (lane_expected_sku or {}).get(lane_id)
        if wanted is None and members and not (lane_expected_sku and lane_id in lane_expected_sku):
            tally: dict[str, int] = {}
            for member in members:
                tally[member["sku"]] = tally.get(member["sku"], 0) + 1
            best = max(tally.values())
            wanted = next(m["sku"] for m in members if tally[m["sku"]] == best)
        for member in members:
            member["misplaced"] = wanted is not None and member["sku"] != wanted
        lane_rows.append(
            {
                "lane_id": lane_id,
                "expected_sku": wanted,
                "count": len(members),
                "skus_in_order": [m["sku"] for m in members],
                "misplaced_count": sum(1 for m in members if m["misplaced"]),
                "centre_world": [
                    round(LANE_CENTER_X_M[lane_id] + WORKCELL_POSE[0], 6),
                    round(WORKCELL_POSE[1] + LANE_MAX_DEPTH_M / 2.0, 6),
                    round(WORKCELL_POSE[2] + 0.1, 6),
                ],
            }
        )
    for member in objects:
        member.pop("_depth", None)
    return {"objects": objects, "lanes": lane_rows}


def scenario_id(seed: int, tag: str = "seed") -> str:
    """The stable identifier of a seeded scene."""
    return f"{tag}{seed:06d}"


# --------------------------------------------------------------------------------------------
# Lighting
# --------------------------------------------------------------------------------------------


def world_lighting(
    world_sdf: str | Path, condition: str = "nominal", intensity_scale: float = 1.0
) -> dict[str, Any]:
    """Read the lighting actually present in a world file (ambient, background, lights).

    ``condition`` names the seeded condition the file was generated for; the numbers are always
    read back from the file, so a row cannot claim lighting the world does not have.
    """
    root = ElementTree.parse(world_sdf).getroot()  # noqa: S314 - repository-owned file

    def _floats(element: ElementTree.Element | None) -> list[float] | None:
        return (
            None
            if element is None or not element.text
            else [float(token) for token in element.text.split()]
        )

    world = root.find("world")
    scene = world.find("scene") if world is not None else None
    lights = []
    if world is not None:
        for light in world.findall("light"):
            lights.append(
                {
                    "name": light.get("name"),
                    "type": light.get("type"),
                    "diffuse": _floats(light.find("diffuse")),
                    "specular": _floats(light.find("specular")),
                    "direction": _floats(light.find("direction")),
                    "cast_shadows": (light.findtext("cast_shadows") or "").strip() == "true",
                }
            )
    return {
        "condition": condition,
        "intensity_scale": intensity_scale,
        "source": "world_sdf",
        "ambient": _floats(scene.find("ambient")) if scene is not None else None,
        "background": _floats(scene.find("background")) if scene is not None else None,
        "lights": lights,
    }


# --------------------------------------------------------------------------------------------
# Frame labels
# --------------------------------------------------------------------------------------------


def frame_label(
    *,
    frame_id: str,
    scenario: str,
    seed: int | None,
    seed_split: str | None,
    sensor: str,
    index: int,
    truth: dict[str, Any],
    camera_pose_world: Sequence[float],
    intrinsics: dict[str, float],
    lighting: dict[str, Any],
    files: dict[str, str],
    stamp_sim_s: float | None = None,
    viewpoint: str | None = None,
) -> dict[str, Any]:
    """Build one JSONL label row: per-object and per-lane truth seen from this camera."""
    if sensor not in SENSORS:
        raise ValueError(f"sensor must be one of {SENSORS}: {sensor}")
    objects = []
    for item in truth["objects"]:
        row = dict(item)
        projection = project_point(item["pose_world"][:3], camera_pose_world, intrinsics)
        row["pixel"] = {"u": projection["u"], "v": projection["v"]}
        row["range_m"] = projection["depth_m"]
        row["in_view"] = projection["in_view"]
        # Occlusion is not labelled: the renderer gives no per-instance mask here, so
        # ``in_view`` means "centre projects inside the image", not "unoccluded".
        objects.append(row)
    lanes = []
    for lane in truth["lanes"]:
        row = dict(lane)
        projection = project_point(lane["centre_world"], camera_pose_world, intrinsics)
        row["in_view"] = projection["in_view"]
        lanes.append(row)
    return {
        "schema_version": SCHEMA_VERSION,
        "frame_id": frame_id,
        "scenario_id": scenario,
        "seed": seed,
        "seed_split": seed_split,
        "sensor": sensor,
        "index": index,
        "stamp_sim_s": stamp_sim_s,
        "files": dict(sorted(files.items())),
        "camera": {
            "pose_world": [round(float(v), 6) for v in camera_pose_world],
            "pose_convention": "optical frame in world, [x, y, z, qx, qy, qz, qw]",
            "intrinsics": {key: float(value) for key, value in intrinsics.items()},
        },
        "lighting": lighting,
        "viewpoint": viewpoint,
        "objects": objects,
        "lanes": lanes,
    }


def label_content_hash_row(row: dict[str, Any]) -> dict[str, Any]:
    """The part of a label row that is renderer-independent (no camera, no files, no stamp)."""
    return {
        "scenario_id": row["scenario_id"],
        "seed": row["seed"],
        "sensor": row["sensor"],
        "lighting_condition": [
            row["lighting"].get("condition"),
            row["lighting"].get("intensity_scale"),
        ],
        "objects": [
            {k: v for k, v in obj.items() if k not in {"pixel", "range_m", "in_view"}}
            for obj in row["objects"]
        ],
        "lanes": [{k: v for k, v in ln.items() if k != "in_view"} for ln in row["lanes"]],
    }


# --------------------------------------------------------------------------------------------
# PNG (deterministic, no imaging library needed)
# --------------------------------------------------------------------------------------------


def _chunk(kind: bytes, payload: bytes) -> bytes:
    body = kind + payload
    return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body))


def encode_png(pixels: Any) -> bytes:
    """Encode an HxWx3 uint8 (RGB) or HxW uint16 (gray, e.g. depth in mm) array as a PNG."""
    import numpy  # noqa: PLC0415 - only the writer needs it

    array = numpy.asarray(pixels)
    if array.ndim == 3 and array.shape[2] == 3 and array.dtype == numpy.uint8:
        colour_type, depth, raw = 2, 8, array.astype(numpy.uint8)
    elif array.ndim == 2 and array.dtype == numpy.uint16:
        colour_type, depth, raw = 0, 16, array.astype(">u2")
    else:
        raise ValueError("encode_png takes HxWx3 uint8 or HxW uint16")
    height, width = array.shape[:2]
    rows = raw.reshape(height, -1).view(numpy.uint8) if raw.ndim == 2 else raw.reshape(height, -1)
    stream = b"".join(b"\x00" + rows[r].tobytes() for r in range(height))
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, depth, colour_type, 0, 0, 0))
        + _chunk(b"IDAT", zlib.compress(stream, 6))
        + _chunk(b"IEND", b"")
    )


def decode_png(data: bytes) -> Any:
    """Decode a PNG made by :func:`encode_png` (filter type 0 only) back to an array."""
    import numpy  # noqa: PLC0415

    if data[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    offset, idat, header = 8, b"", None
    while offset < len(data):
        (length,) = struct.unpack(">I", data[offset : offset + 4])
        kind = data[offset + 4 : offset + 8]
        payload = data[offset + 8 : offset + 8 + length]
        if kind == b"IHDR":
            header = struct.unpack(">IIBBBBB", payload)
        elif kind == b"IDAT":
            idat += payload
        offset += 12 + length
    width, height, depth, colour_type = header[:4]
    raw = zlib.decompress(idat)
    channels = 3 if colour_type == 2 else 1
    row_bytes = width * channels * (depth // 8)
    rows = []
    for r in range(height):
        start = r * (row_bytes + 1)
        if raw[start] != 0:
            raise ValueError("decode_png supports filter type 0 only")
        rows.append(numpy.frombuffer(raw[start + 1 : start + 1 + row_bytes], dtype=numpy.uint8))
    block = numpy.stack(rows)
    if depth == 16:
        return block.view(">u2").reshape(height, width).astype(numpy.uint16)
    return block.reshape(height, width, channels)


# --------------------------------------------------------------------------------------------
# Dataset directory
# --------------------------------------------------------------------------------------------

MANIFEST_NAME = "manifest.json"
LABELS_NAME = "labels.jsonl"


class DatasetBuilder:
    """Collect scenes and frames in memory-light staging, then publish one hashed directory."""

    def __init__(self, staging: str | Path) -> None:
        """Start an empty dataset staged under ``staging`` (created fresh)."""
        self.staging = Path(staging)
        if self.staging.exists():
            shutil.rmtree(self.staging)
        (self.staging / "frames").mkdir(parents=True)
        (self.staging / "scenarios").mkdir()
        self._rows: list[dict[str, Any]] = []
        self._scenarios: dict[str, dict[str, Any]] = {}

    def add_scenario(
        self, scenario_key: str, scenario: dict[str, Any], meta: dict[str, Any]
    ) -> None:
        """Store a scenario document next to the frames and remember its hash and metadata."""
        text = yaml.safe_dump(scenario, sort_keys=True, default_flow_style=False)
        (self.staging / "scenarios" / f"{scenario_key}.yaml").write_text(text, encoding="utf-8")
        self._scenarios[scenario_key] = {"scenario_sha256": sha256_bytes(text.encode()), **meta}

    def add_image(self, name: str, data: bytes) -> str:
        """Write one frame file under frames/ and return its dataset-relative path."""
        relative = f"frames/{name}"
        (self.staging / relative).write_bytes(data)
        return relative

    def add_label(self, row: dict[str, Any]) -> None:
        """Append one label row (call ``frame_label`` to build it)."""
        self._rows.append(row)

    def publish(
        self,
        out_dir: str | Path,
        *,
        seeds: Sequence[int],
        world_sdf: str | Path | None,
        repository: str | Path,
        catalogue_hash: str,
        splits_hash: str | None,
        extra: dict[str, Any] | None = None,
    ) -> Path:
        """Write labels and manifest, name the directory by content hash, and return its path."""
        rows = sorted(self._rows, key=lambda row: row["frame_id"])
        (self.staging / LABELS_NAME).write_text(
            "".join(canonical_json(row) + "\n" for row in rows), encoding="utf-8"
        )
        labels_hash = sha256_bytes(
            canonical_json(
                [label_content_hash_row(row) for row in rows if row["index"] == 0]
            ).encode()
        )
        files = file_hashes(self.staging)
        files.pop(MANIFEST_NAME, None)
        dataset_hash = hash_of_files(files)
        manifest = {
            "kind": DATASET_KIND,
            "schema_version": SCHEMA_VERSION,
            "generator_version": GENERATOR_VERSION,
            "dataset_hash": dataset_hash,
            "labels_hash": labels_hash,
            "seeds": sorted(int(seed) for seed in seeds),
            "frame_count": len(rows),
            "scenarios": dict(sorted(self._scenarios.items())),
            "world_sha256": sha256_file(world_sdf) if world_sdf else None,
            "catalogue_sha256": catalogue_hash,
            "splits_sha256": splits_hash,
            "git": git_state(repository),
            "files": files,
            **(extra or {}),
        }
        (self.staging / MANIFEST_NAME).write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
        target = Path(out_dir) / f"pbench-{dataset_hash[:16]}"
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            shutil.rmtree(target)
        os.replace(self.staging, target)
        return target


def file_hashes(root: str | Path) -> dict[str, str]:
    """Return {relative posix path: sha256} for every file under ``root``."""
    base = Path(root)
    return {
        path.relative_to(base).as_posix(): sha256_file(path)
        for path in sorted(base.rglob("*"))
        if path.is_file()
    }


def hash_of_files(files: dict[str, str]) -> str:
    """The dataset hash: SHA-256 over sorted ``<sha>  <path>`` lines (manifest excluded)."""
    lines = "".join(f"{sha}  {path}\n" for path, sha in sorted(files.items()))
    return sha256_bytes(lines.encode())


def verify_dataset(directory: str | Path) -> list[str]:
    """Recompute every hash in a published dataset; return a list of problems (empty = ok)."""
    base = Path(directory)
    problems: list[str] = []
    try:
        manifest = json.loads((base / MANIFEST_NAME).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        return [f"manifest unreadable: {error}"]
    actual = file_hashes(base)
    actual.pop(MANIFEST_NAME, None)
    for path, sha in manifest["files"].items():
        if path not in actual:
            problems.append(f"missing file {path}")
        elif actual[path] != sha:
            problems.append(f"hash mismatch {path}")
    for path in actual.keys() - manifest["files"].keys():
        problems.append(f"unlisted file {path}")
    if hash_of_files(actual) != manifest["dataset_hash"]:
        problems.append("dataset_hash does not match the files")
    if base.name != f"pbench-{manifest['dataset_hash'][:16]}":
        problems.append("directory name does not match dataset_hash")
    return problems


def read_labels(path: str | Path) -> list[dict[str, Any]]:
    """Read a labels.jsonl file (or the one inside a dataset directory)."""
    target = Path(path)
    if target.is_dir():
        target = target / LABELS_NAME
    return [json.loads(line) for line in target.read_text(encoding="utf-8").splitlines() if line]
