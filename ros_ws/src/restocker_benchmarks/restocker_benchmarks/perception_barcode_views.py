# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Aimed close-range wrist viewpoints at the barcode on a cylinder product's back panel."""
# Pure Python, no ROS. The close views of perception_views.py approach every product from the
# robot side, so the label side that faces the lens is whatever the product's random yaw gives.
# The EAN-13 sits on the back panel, and CMB-101 measured that a barcode decodes only at close
# range and near-frontal aim. So for each tray product this orbits the camera to the azimuth
# (in the product's own frame) where the generated label's barcode faces out, and takes a few
# views spread around it, closer than a read view. A view whose azimuth the arm cannot reach
# from the robot side is not requested (``max_reach_deg`` about the approach direction); the
# planner still decides at record time and the outcome is written to the part, never assumed.
#
# BARCODE_FACING_DEG is measured, not derived: the camera azimuth, in the product's yaw frame
# (atan2 of the camera offset minus the product yaw), at which the simulator renders the back
# panel's barcode facing the lens. See docs/measurements in the backstage for how it was found.

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Any

from restocker_benchmarks.perception_views import (
    APPROACH,
    MAX_EYE_Z_M,
    look_at_optical,
)

BARCODE_VIEWS_VERSION = 1
BARCODE_FACING_DEG = 10.0
STANDOFF_M = 0.22
ELEVATION_DEG = 6.0
SPREAD_DEG = 20.0
# The generated barcode sits low on the back panel: aim a little under the product's centre.
AIM_Z_OFFSET_M = -0.02
DEFAULT_MAX_REACH_DEG = 75.0


def yaw_of(pose_world: Sequence[float]) -> float:
    """Yaw of a pose [x, y, z, qx, qy, qz, qw], radians."""
    x, y, z, w = pose_world[3:7]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_up_z(pose_world: Sequence[float]) -> float:
    """z component of the body's +z axis in the world: 1 standing, 0 lying on a side."""
    x, y, z, w = pose_world[3:7]
    return 1.0 - 2.0 * (x * x + y * y)


def _wrap_deg(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def barcode_views(
    truth: dict[str, Any],
    *,
    facing_deg: float = BARCODE_FACING_DEG,
    standoff_m: float = STANDOFF_M,
    max_reach_deg: float = DEFAULT_MAX_REACH_DEG,
    offsets_deg: Sequence[float] = (-SPREAD_DEG, 0.0, SPREAD_DEG),
) -> list[dict[str, Any]]:
    """Camera poses that look at each tray product's barcode panel; pure in the scene truth."""
    approach_azimuth = math.degrees(math.atan2(APPROACH[1], APPROACH[0]))
    views = []
    targets = sorted(
        (
            o
            for o in truth["objects"]
            if o["lane_id"] is None and o["product_class"] != "contour_bottle"
        ),
        key=lambda o: o["object_id"],
    )
    elevation = math.radians(ELEVATION_DEG)
    for target in targets:
        yaw = yaw_of(target["pose_world"])
        position = target["pose_world"][:3]
        if target["product_class"] == "box":
            # A box has no continuous label angle: the barcode is on its back face, whose
            # outward normal is +y in the product frame, so the camera stands along yaw + 90
            # degrees. A box lying on its back or side shows no back face to the sky: skipped.
            if abs(quaternion_up_z(target["pose_world"])) < 0.9:
                continue
            face_deg, aim_z = 90.0, 0.0
        else:
            face_deg, aim_z = facing_deg, AIM_Z_OFFSET_M
        aim = [position[0], position[1], position[2] + aim_z]
        for offset in offsets_deg:
            azimuth_deg = math.degrees(yaw) + face_deg + offset
            if abs(_wrap_deg(azimuth_deg - approach_azimuth)) > max_reach_deg:
                continue
            azimuth = math.radians(azimuth_deg)
            horizontal = standoff_m * math.cos(elevation)
            eye = [
                aim[0] + horizontal * math.cos(azimuth),
                aim[1] + horizontal * math.sin(azimuth),
                aim[2] + standoff_m * math.sin(elevation),
            ]
            if eye[2] > MAX_EYE_Z_M:
                continue
            pose = [round(v, 6) for v in eye] + [round(v, 9) for v in look_at_optical(eye, aim)]
            views.append(
                {
                    "name": f"barcode{len(views):02d}",
                    "target_object_id": target["object_id"],
                    "standoff_m": round(standoff_m, 4),
                    "camera_optical_pose": pose,
                    "azimuth_world_deg": round(_wrap_deg(azimuth_deg), 2),
                    "kind": "barcode",
                }
            )
    return views
