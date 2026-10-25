# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Aimed barcode viewpoints: geometry, reach filter, purity."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_barcode_views as bv  # noqa: E402,I100,I101


def truth(yaw_deg: float, product_class: str = "can") -> dict:
    half = math.radians(yaw_deg) / 2.0
    return {
        "objects": [
            {
                "object_id": "sim:stock_can_01",
                "lane_id": None,
                "product_class": product_class,
                "pose_world": [0.1, -0.5, 0.85, 0.0, 0.0, math.sin(half), math.cos(half)],
            },
            {
                "object_id": "sim:lane_01_can_01",
                "lane_id": "lane_01",
                "product_class": "can",
                "pose_world": [0.0, 1.0, 0.85, 0.0, 0.0, 0.0, 1.0],
            },
        ]
    }


def azimuth_of(view: dict, target=(0.1, -0.5)) -> float:
    eye = view["camera_optical_pose"]
    return math.degrees(math.atan2(eye[1] - target[1], eye[0] - target[0]))


def test_yaw_round_trip():
    for degrees in (-170, -45, 0, 30, 120):
        half = math.radians(degrees) / 2.0
        pose = [0, 0, 0, 0, 0, math.sin(half), math.cos(half)]
        assert math.isclose(math.degrees(bv.yaw_of(pose)), degrees, abs_tol=1e-6)


def test_views_orbit_to_the_barcode_azimuth_and_only_cover_tray_products():
    # Choose the yaw that puts the barcode azimuth on the robot-side approach direction (+y).
    yaw = 90.0 - bv.BARCODE_FACING_DEG
    views = bv.barcode_views(truth(yaw))
    assert len(views) == 3
    assert {v["target_object_id"] for v in views} == {"sim:stock_can_01"}
    azimuths = sorted(azimuth_of(v) for v in views)
    assert [round(a) for a in azimuths] == [
        round(90 - bv.SPREAD_DEG),
        90,
        round(90 + bv.SPREAD_DEG),
    ]
    assert all(v["kind"] == "barcode" and v["name"].startswith("barcode") for v in views)
    assert all(v["camera_optical_pose"][2] <= 1.0 for v in views)
    assert all(abs(v["standoff_m"] - bv.STANDOFF_M) < 1e-9 for v in views)


def test_unreachable_azimuths_are_not_requested():
    away = 90.0 - bv.BARCODE_FACING_DEG + 180.0  # barcode faces away from the robot
    assert bv.barcode_views(truth(away)) == []
    # widening the reach asks for them (the planner then decides)
    assert len(bv.barcode_views(truth(away), max_reach_deg=180.0)) == 3


def test_contour_bottle_is_skipped_and_the_function_is_pure():
    assert bv.barcode_views(truth(0.0, "contour_bottle"), max_reach_deg=180.0) == []
    first = bv.barcode_views(truth(10.0), max_reach_deg=180.0)
    assert first == bv.barcode_views(truth(10.0), max_reach_deg=180.0)


def test_a_box_is_viewed_from_its_back_face_and_a_lying_box_is_skipped():
    import math

    def box(yaw_deg, roll_deg=0.0, name="a"):
        half_yaw, half_roll = math.radians(yaw_deg) / 2, math.radians(roll_deg) / 2
        # quaternion of rotz(yaw) * rotx(roll)
        qx, qy = math.sin(half_roll) * math.cos(half_yaw), math.sin(half_roll) * math.sin(half_yaw)
        qz, qw = math.cos(half_roll) * math.sin(half_yaw), math.cos(half_roll) * math.cos(half_yaw)
        return {
            "object_id": f"sim:{name}",
            "product_class": "box",
            "lane_id": None,
            "pose_world": [0.0, -0.8, 0.62, qx, qy, qz, qw],
        }

    views = bv.barcode_views(
        {"objects": [box(0.0)]}, max_reach_deg=180.0, offsets_deg=(0.0,), standoff_m=0.2
    )
    assert len(views) == 1
    # Yaw 0: the back face looks along +y, so the camera stands at +y of the box.
    assert views[0]["azimuth_world_deg"] == pytest.approx(90.0)
    assert views[0]["camera_optical_pose"][1] > -0.8
    turned = bv.barcode_views(
        {"objects": [box(90.0)]}, max_reach_deg=180.0, offsets_deg=(0.0,), standoff_m=0.2
    )
    assert abs(turned[0]["azimuth_world_deg"]) == pytest.approx(180.0)
    assert bv.barcode_views({"objects": [box(0.0, 90.0)]}, max_reach_deg=180.0) == []
