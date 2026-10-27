# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The wrist survey's lane stations as benchmark viewpoints (one per lane, every scene)."""
# Pure Python, no ROS. The lane stock estimator (CMB-114) consumes what the wrist survey produces:
# a depth and RGB frame taken from a lane's survey station. This module states those stations
# in the world, from the same numbers survey_stations.cpp derives them from (the camera stands
# lane_camera_setback_m behind the shelf's rear edge, lane_camera_height_m above the lane floor,
# over the lane's centre line, aimed at mid-lane depth, image-right along +X), so the recorder
# can send the arm there through /survey_viewpoint exactly as it does for the close views. Every
# lane gets a view, empty ones included: an empty lane is an answer the estimator is scored on.

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any

from restocker_gazebo.lane_columns import LANE_CENTER_X_M, WORKCELL_POSE

from restocker_benchmarks.perception_views import look_at_optical

LANE_VIEWS_VERSION = 1
# survey_stations.hpp SurveyStationConfig defaults.
CAMERA_SETBACK_M = 0.55
CAMERA_HEIGHT_M = 0.55
# workcell_geometry.yaml lanes: rear_clearance_m + usable_depth_m / 2.
MID_LANE_DEPTH_M = 0.01 + 0.85 / 2.0


@dataclass(frozen=True)
class LaneStationStress:
    """Where the arm really stands compared with the nominal station (CMB-116), in metres.

    ``dx_m`` moves the camera along the shelf (off the lane's centre line), ``dy_m`` changes the
    setback (positive = farther from the shelf), ``dz_m`` the height. The camera keeps aiming at
    mid-lane, as a station reached with a pose error would; the recorded camera pose is the one
    the estimator is given, so the geometry stays exact.
    """

    dx_m: float = 0.0
    dy_m: float = 0.0
    dz_m: float = 0.0

    def is_default(self) -> bool:
        return self == LaneStationStress()

    def to_dict(self) -> dict[str, float]:
        return asdict(self)


def lane_views(
    truth: dict[str, Any] | None = None, stress: LaneStationStress | None = None
) -> list[dict[str, Any]]:
    """One survey-station camera pose per lane; ``truth`` only picks each lane's rear product.

    Each view is ``{"name": "lane_NN", "lane_id", "kind": "lane_survey", "target_object_id",
    "camera_optical_pose"}``; the pose is ``[x, y, z, qx, qy, qz, qw]`` of the optical frame in
    the world. ``target_object_id`` is the lane's rearmost product (the nearest to the camera) or
    None for an empty lane. It is for the recorder's bookkeeping and plays no part in the pose.
    """
    rear_of_lane: dict[str, str] = {}
    if truth is not None:
        members: dict[str, list[dict[str, Any]]] = {}
        for obj in truth["objects"]:
            if obj["lane_id"] is not None:
                members.setdefault(obj["lane_id"], []).append(obj)
        for lane_id, objects in members.items():
            rear_of_lane[lane_id] = max(objects, key=lambda o: o["lane_order"])["object_id"]
    stress = stress or LaneStationStress()
    views = []
    for lane_id, centre_x in LANE_CENTER_X_M.items():
        eye = [
            centre_x + WORKCELL_POSE[0] + stress.dx_m,
            WORKCELL_POSE[1] - CAMERA_SETBACK_M - stress.dy_m,
            WORKCELL_POSE[2] + CAMERA_HEIGHT_M + stress.dz_m,
        ]
        target = [
            centre_x + WORKCELL_POSE[0],
            WORKCELL_POSE[1] + MID_LANE_DEPTH_M,
            WORKCELL_POSE[2],
        ]
        pose = [round(v, 6) for v in eye] + [round(v, 9) for v in look_at_optical(eye, target)]
        views.append(
            {
                "name": lane_id,
                "lane_id": lane_id,
                "kind": "lane_survey",
                "range_check": False,
                "target_object_id": rear_of_lane.get(lane_id),
                "camera_optical_pose": pose,
            }
        )
    return views
