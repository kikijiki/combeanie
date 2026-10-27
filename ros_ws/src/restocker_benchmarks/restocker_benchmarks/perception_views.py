# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Seeded close-range wrist viewpoints for the perception benchmark."""
# Pure Python, no ROS. The wrist camera at its home pose sees no product, so the recorder also
# drives it to a few seeded viewpoints 0.28-0.45 m from a tray product, close enough to read the
# label. A viewpoint is a camera pose in the world, the same kind of request the survey server
# (`/survey_viewpoint`, `SurveyViewpoint.camera_optical_pose`) already accepts, so the arm is moved
# by the planner and the controllers, not by anything in the recorder. The approach direction is
# the one the confirm duty uses on the tray (survey_stations.hpp, ConfirmationViewpointConfig:
# 0.45 m behind the product toward the robot, 0.555 m above), which is known to be reachable;
# a seed only turns it by a bounded azimuth and picks the standoff. Whether the arm really
# gets there is decided by the planner at record time and written to the part, never assumed.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import asdict, dataclass
import math
from typing import Any

from restocker_gazebo.lane_columns import CATALOGUE
from restocker_gazebo.scenario_random import SplitMix64

VIEWS_VERSION = 1
VIEWS_PER_SCENE = 3
MIN_STANDOFF_M = 0.28
MAX_STANDOFF_M = 0.45
# The confirm duty's approach in the world frame, which the shelf frame is aligned with.
APPROACH = (0.0, 0.45, 0.555)
MAX_AZIMUTH_DEG = 30.0
MAX_ELEVATION_JITTER_DEG = 6.0
# The confirm duty biases the aim up the barrel so a tall bottle frames; the same here.
AIM_BIAS_Z_M = 0.05
RANGE_LIMITS_M = (0.25, 0.5)
# test_wrist_viewpoint_runtime measured that the arm holds a tray view only below world z 1.05.
MAX_EYE_Z_M = 1.0

_MASK64 = (1 << 64) - 1
_HEIGHT_OF_CLASS = {entry["product_class"]: entry["height_m"] for entry in CATALOGUE.values()}


@dataclass(frozen=True)
class ViewStress:
    """Perturbations of the seeded viewpoints (CMB-116). The default is the unperturbed view.

    ``max_azimuth_deg`` widens the turn about the vertical (the label is seen more obliquely),
    ``elevation_offset_deg`` adds to the approach elevation (positive looks more from above),
    ``standoff_scale`` multiplies the standoff (the label gets fewer pixels) and ``pan_deg``
    turns the camera about its own vertical image axis, alternately left and right per view,
    with the eye unchanged, so the product moves toward an image edge and may be cut off (the
    wrist camera's half field of view is 42 degrees: a pan of that size puts the product's centre
    on the edge). None of them changes a random draw: the stream of a stressed view is that of the
    unperturbed one.
    """

    max_azimuth_deg: float = MAX_AZIMUTH_DEG
    elevation_offset_deg: float = 0.0
    standoff_scale: float = 1.0
    pan_deg: float = 0.0

    def is_default(self) -> bool:
        return self == ViewStress()

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


def quaternion_from_matrix(matrix: Sequence[Sequence[float]]) -> list[float]:
    """Return [x, y, z, w] for a proper rotation matrix."""
    m = matrix
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        q = [
            (m[2][1] - m[1][2]) / scale,
            (m[0][2] - m[2][0]) / scale,
            (m[1][0] - m[0][1]) / scale,
            0.25 * scale,
        ]
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        scale = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        q = [
            0.25 * scale,
            (m[0][1] + m[1][0]) / scale,
            (m[0][2] + m[2][0]) / scale,
            (m[2][1] - m[1][2]) / scale,
        ]
    elif m[1][1] > m[2][2]:
        scale = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        q = [
            (m[0][1] + m[1][0]) / scale,
            0.25 * scale,
            (m[1][2] + m[2][1]) / scale,
            (m[0][2] - m[2][0]) / scale,
        ]
    else:
        scale = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        q = [
            (m[0][2] + m[2][0]) / scale,
            (m[1][2] + m[2][1]) / scale,
            0.25 * scale,
            (m[1][0] - m[0][1]) / scale,
        ]
    return q


def _cross(a: Sequence[float], b: Sequence[float]) -> list[float]:
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _unit(v: Sequence[float]) -> list[float]:
    norm = math.sqrt(sum(c * c for c in v))
    return [c / norm for c in v]


def look_at_optical(eye: Sequence[float], target: Sequence[float]) -> list[float]:
    """Optical-frame quaternion: +Z on the boresight, +Y image-down, world +Z up in the image."""
    boresight = _unit([t - e for t, e in zip(target, eye, strict=True)])
    down = [0.0, 0.0, -1.0]
    along = sum(d * b for d, b in zip(down, boresight, strict=True))
    perp = _unit([d - along * b for d, b in zip(down, boresight, strict=True)])
    axis_x = _unit(_cross(perp, boresight))
    axis_y = _cross(boresight, axis_x)
    matrix = [[axis_x[row], axis_y[row], boresight[row]] for row in range(3)]
    return quaternion_from_matrix(matrix)


def _matrix_from_quaternion(q: Sequence[float]) -> list[list[float]]:
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def _panned(quaternion: Sequence[float], angle: float) -> list[float]:
    """The optical orientation turned by ``angle`` about its own y axis (image vertical)."""
    m = _matrix_from_quaternion(quaternion)
    axis_x = [m[row][0] for row in range(3)]
    axis_y = [m[row][1] for row in range(3)]
    boresight = [m[row][2] for row in range(3)]
    c, s = math.cos(angle), math.sin(angle)
    new_z = [c * b + s * x for b, x in zip(boresight, axis_x, strict=True)]
    new_x = [c * x - s * b for b, x in zip(boresight, axis_x, strict=True)]
    return quaternion_from_matrix([[new_x[r], axis_y[r], new_z[r]] for r in range(3)])


def _view_at(
    stream: SplitMix64,
    target: dict[str, Any],
    index: int,
    name: str,
    *,
    close: bool = False,
    stress: ViewStress | None = None,
) -> dict[str, Any]:
    """One seeded camera pose at ``target``: the confirm duty's approach, turned and re-timed."""
    base_elevation = math.atan2(APPROACH[2], APPROACH[1])
    height = _HEIGHT_OF_CLASS.get(target["product_class"], 0.2)
    # Taller products need more room to frame the whole body.
    floor = max(MIN_STANDOFF_M, 0.9 * height + 0.05)
    draw = stream.next_unit()
    # A re-look is planned to read text: it approaches as close as the survey allows.
    standoff = floor + (0.15 if close else 1.0) * draw * (MAX_STANDOFF_M - floor)
    stress = stress or ViewStress()
    azimuth = math.radians((stream.next_unit() * 2.0 - 1.0) * stress.max_azimuth_deg)
    elevation = (
        base_elevation
        + math.radians((stream.next_unit() * 2.0 - 1.0) * MAX_ELEVATION_JITTER_DEG)
        + math.radians(stress.elevation_offset_deg)
    )
    position = target["pose_world"][:3]
    aim = [position[0], position[1], position[2] + AIM_BIAS_Z_M]
    reachable = (MAX_EYE_Z_M - aim[2]) / math.sin(elevation)
    standoff = max(RANGE_LIMITS_M[0] + 0.01, min(standoff * stress.standoff_scale, reachable))
    # Eye position: from the aim, back toward the robot (+y), turned about the vertical.
    horizontal = math.cos(elevation)
    offset = [
        standoff * horizontal * math.sin(azimuth),
        standoff * horizontal * math.cos(azimuth),
        standoff * math.sin(elevation),
    ]
    eye = [a + o for a, o in zip(aim, offset, strict=True)]
    quaternion = look_at_optical(eye, aim)
    if stress.pan_deg:
        side = 1.0 if index % 2 == 0 else -1.0
        quaternion = _panned(quaternion, math.radians(stress.pan_deg) * side)
    pose = [round(v, 6) for v in eye] + [round(v, 9) for v in quaternion]
    return {
        "name": name,
        "target_object_id": target["object_id"],
        "standoff_m": round(standoff, 4),
        "camera_optical_pose": pose,
    }


def close_views(
    seed: int,
    truth: dict[str, Any],
    count: int = VIEWS_PER_SCENE,
    stress: ViewStress | None = None,
) -> list[dict[str, Any]]:
    """Seeded close-range camera poses at tray products; empty if the scene has none.

    Each view is ``{"name", "target_object_id", "standoff_m", "camera_optical_pose"}``, the pose
    being ``[x, y, z, qx, qy, qz, qw]`` of the optical frame in the world. Pure in ``seed`` and
    the scene truth.
    """
    targets = sorted(
        (obj for obj in truth["objects"] if obj["lane_id"] is None),
        key=lambda obj: obj["object_id"],
    )
    if not targets:
        return []
    stream = SplitMix64((seed * 0x9E3779B97F4A7C15 + 0x56494557 + VIEWS_VERSION) & _MASK64)
    stream.next_u64()
    first = stream.next_below(len(targets))
    return [
        _view_at(
            stream,
            targets[(first + index) % len(targets)],
            index,
            f"close{index:02d}",
            stress=stress,
        )
        for index in range(count)
    ]


def relook_views(
    seed: int,
    truth: dict[str, Any],
    object_ids: Sequence[str],
    *,
    look: int,
    per_object: int = 2,
    stress: ViewStress | None = None,
) -> list[dict[str, Any]]:
    """Close views of the named tray products only, for look number ``look`` (1, 2, ...).

    The re-look of an object the identity ladder asked to see again: the same kind of pose as
    :func:`close_views`, drawn from a stream of its own (seed and look number), so a re-look never
    repeats the camera poses of an earlier look. Pure in its arguments.
    """
    by_id = {obj["object_id"]: obj for obj in truth["objects"] if obj["lane_id"] is None}
    stream = SplitMix64(
        (seed * 0x9E3779B97F4A7C15 + 0x52454C4B + look * 0x1000193 + VIEWS_VERSION) & _MASK64
    )
    stream.next_u64()
    views = []
    for object_id in sorted(object_ids):
        target = by_id[object_id]
        for index in range(per_object):
            views.append(
                _view_at(
                    stream,
                    target,
                    index,
                    f"look{look}-{object_id}-{index}",
                    close=True,
                    stress=stress,
                )
            )
    return views


def turn_error_deg(seed: int, object_id: str, look: int, sigma_deg: float) -> float:
    """A seeded Gaussian error of the rotation oracle's turn, in degrees (CMB-116).

    The oracle turns a product exactly as the ladder asked; a real regrasp does not. The error
    depends on the seed, the object and the look number only, so a re-recorded look repeats it.
    """
    if sigma_deg <= 0.0:
        return 0.0
    mixed = seed * 0x9E3779B97F4A7C15 + hash_text(object_id) + look * 0x1000193 + 0x5455524E
    stream = SplitMix64(mixed & _MASK64)
    stream.next_u64()
    # Box-Muller from two uniforms of the stream.
    u1, u2 = max(stream.next_unit(), 1e-12), stream.next_unit()
    return sigma_deg * math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def hash_text(text: str) -> int:
    """A stable 64-bit hash of a string (Python's hash() is salted per process)."""
    value = 0xCBF29CE484222325
    for byte in text.encode():
        value = ((value ^ byte) * 0x100000001B3) & _MASK64
    return value
