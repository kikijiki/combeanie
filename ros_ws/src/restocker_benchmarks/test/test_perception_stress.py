# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""CMB-116 recorder options: named stress lighting and perturbed viewpoints (pure Python)."""
# The unperturbed paths must stay byte-identical: every recorded dataset and every frozen split
# depends on a seed meaning the same condition and the same camera poses.

import hashlib
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_benchmarks import perception_lighting as lighting  # noqa: E402,I100,I101
from restocker_benchmarks import perception_views as views  # noqa: E402,I100,I101

WORLD = Path(__file__).resolve().parents[2] / "restocker_gazebo" / "worlds" / "restocking.sdf"
# Pinned from the code before CMB-116 touched either module (see the commit message).
UNCHANGED_SHA256 = "afa80073ce7681396b87001d6bca5dcde9dc0b65d9275af74b747089ff3e487a"


def _truth(z: float) -> dict:
    return {
        "objects": [
            {
                "object_id": f"sim:o{i}",
                "lane_id": None,
                "product_class": "can",
                "pose_world": [0.1 * i - 0.2, -0.7, z],
            }
            for i in range(4)
        ]
    }


# The golden hash was taken on a tray product at 0.9 m; a can on the tray surface is at 0.63 m.
TRUTH = _truth(0.9)
TRAY_TRUTH = _truth(0.63)


def test_default_seeds_and_views_are_what_they_were():
    out = []
    for seed in (7, 235, 523, 786):
        out.append(views.close_views(seed, TRUTH, 3))
        out.append(views.relook_views(seed, TRUTH, ["sim:o1", "sim:o3"], look=2, per_object=2))
        out.append(lighting.lighting_for_seed(seed))
    assert hashlib.sha256(json.dumps(out, sort_keys=True).encode()).hexdigest() == UNCHANGED_SHA256


def test_a_default_view_stress_is_the_unperturbed_view():
    assert views.ViewStress().is_default()
    assert views.close_views(9, TRUTH, 3, stress=views.ViewStress()) == views.close_views(
        9, TRUTH, 3
    )


@pytest.mark.parametrize("name", lighting.STRESS_NAMES)
def test_stress_conditions_apply_to_the_shipped_world_and_are_not_drawn_by_seeds(name):
    assert name not in lighting.CONDITION_NAMES
    spec = lighting.lighting_named(name, 11)
    assert spec["condition"] == name
    text = lighting.apply_lighting(WORLD.read_text(encoding="utf-8"), spec)
    assert "workcell_sun" in text
    assert lighting.lighting_named(name, 11) == spec
    assert all(0.0 <= c <= 1.0 for key in ("ambient", "diffuse", "specular") for c in spec[key])
    drawn = {lighting.lighting_for_seed(seed)["condition"] for seed in range(300)}
    assert name not in drawn


def test_stress_conditions_are_ordered_by_what_they_say():
    nominal = lighting.lighting_named("nominal", 3)
    dim = lighting.lighting_named("very_dim", 3)
    over = lighting.lighting_named("max_light", 3)
    red = lighting.lighting_named("colour_red", 3)
    hard = lighting.lighting_named("hard_directional", 3)
    assert sum(dim["diffuse"]) < 0.3 * sum(nominal["diffuse"])
    assert min(over["ambient"]) > 2 * max(nominal["ambient"]) and min(over["diffuse"]) >= 0.9
    assert red["diffuse"][0] > 3 * red["diffuse"][2]
    assert hard["ambient"][0] < 0.05 and abs(hard["direction"][2]) < 0.2
    with pytest.raises(ValueError, match="unknown lighting"):
        lighting.lighting_named("sepia", 3)


def _eye(view):
    return view["camera_optical_pose"][:3]


def test_view_stress_moves_the_camera_the_way_it_says():
    base = views.close_views(5, TRAY_TRUTH, 3)
    wide = views.close_views(5, TRAY_TRUTH, 3, stress=views.ViewStress(max_azimuth_deg=70.0))
    assert [v["name"] for v in wide] == [v["name"] for v in base]
    for view in wide:
        target = next(
            o for o in TRAY_TRUTH["objects"] if o["object_id"] == view["target_object_id"]
        )
        dx = _eye(view)[0] - target["pose_world"][0]
        dy = _eye(view)[1] - target["pose_world"][1]
        assert abs(math.degrees(math.atan2(dx, dy))) <= 70.0 + 1e-6
    moved = max(
        abs(a - b)
        for w, c in zip(wide, base, strict=True)
        for a, b in zip(_eye(w), _eye(c), strict=True)
    )
    assert moved > 0.02


def test_standoff_is_capped_by_the_eye_height_unless_the_view_is_flatter():
    # The arm holds a tray view only below world z 1.0, so at the confirm duty's elevation a
    # longer standoff is clamped; a flatter approach is what buys distance.
    base = views.close_views(5, TRAY_TRUTH, 3)
    far_steep = views.close_views(5, TRAY_TRUTH, 3, stress=views.ViewStress(standoff_scale=1.4))
    far_flat = views.close_views(
        5, TRAY_TRUTH, 3, stress=views.ViewStress(standoff_scale=1.4, elevation_offset_deg=-20.0)
    )
    assert all(f["standoff_m"] >= b["standoff_m"] for f, b in zip(far_steep, base, strict=True))
    assert max(v["standoff_m"] for v in far_steep) < 0.43
    assert all(v["camera_optical_pose"][2] <= views.MAX_EYE_Z_M + 1e-6 for v in far_flat)
    assert max(v["standoff_m"] for v in far_flat) > max(v["standoff_m"] for v in base) + 0.05


def _rotation_angle_deg(qa, qb):
    ra, rb = views._matrix_from_quaternion(qa), views._matrix_from_quaternion(qb)
    trace = sum(sum(ra[k][i] * rb[k][i] for k in range(3)) for i in range(3))
    return math.degrees(math.acos(max(-1.0, min(1.0, (trace - 1.0) / 2.0))))


def test_pan_stress_keeps_the_eye_and_turns_the_camera_by_the_angle():
    base = views.close_views(5, TRAY_TRUTH, 2)
    side = views.close_views(5, TRAY_TRUTH, 2, stress=views.ViewStress(pan_deg=40.0))
    for b, s in zip(base, side, strict=True):
        assert _eye(b) == _eye(s)
        turned = _rotation_angle_deg(b["camera_optical_pose"][3:], s["camera_optical_pose"][3:])
        assert turned == pytest.approx(40.0, abs=0.01)
    # Alternating sides: the two views turn opposite ways about the camera's vertical axis.
    first = views._matrix_from_quaternion(side[0]["camera_optical_pose"][3:])
    base_first = views._matrix_from_quaternion(base[0]["camera_optical_pose"][3:])
    second = views._matrix_from_quaternion(side[1]["camera_optical_pose"][3:])
    base_second = views._matrix_from_quaternion(base[1]["camera_optical_pose"][3:])

    def pan_sign(new, old):
        # sign of (new boresight . old image-x axis)
        return sum(new[r][2] * old[r][0] for r in range(3))

    assert pan_sign(first, base_first) > 0 > pan_sign(second, base_second)


def test_view_stress_to_dict_round_trips():
    stress = views.ViewStress(max_azimuth_deg=60.0, elevation_offset_deg=10.0)
    assert views.ViewStress(**stress.to_dict()) == stress


def test_lane_station_stress_moves_the_eye_and_the_default_is_the_nominal_station():
    from restocker_benchmarks import perception_lane_views as lane_views  # noqa: PLC0415

    nominal = lane_views.lane_views()
    assert lane_views.lane_views(stress=lane_views.LaneStationStress()) == nominal
    moved = lane_views.lane_views(
        stress=lane_views.LaneStationStress(dx_m=0.1, dy_m=0.15, dz_m=-0.1)
    )
    for a, b in zip(nominal, moved, strict=True):
        assert b["camera_optical_pose"][0] == pytest.approx(a["camera_optical_pose"][0] + 0.1)
        assert b["camera_optical_pose"][1] == pytest.approx(a["camera_optical_pose"][1] - 0.15)
        assert b["camera_optical_pose"][2] == pytest.approx(a["camera_optical_pose"][2] - 0.1)
        assert a["camera_optical_pose"][3:] != b["camera_optical_pose"][3:]
    stress = lane_views.LaneStationStress(dy_m=0.2)
    assert lane_views.LaneStationStress(**stress.to_dict()) == stress


def test_turn_error_is_seeded_gaussian_and_zero_without_sigma():
    assert views.turn_error_deg(5, "sim:stock_can_01", 1, 0.0) == 0.0
    a = views.turn_error_deg(5, "sim:stock_can_01", 1, 25.0)
    assert a == views.turn_error_deg(5, "sim:stock_can_01", 1, 25.0)
    assert a != views.turn_error_deg(5, "sim:stock_can_01", 2, 25.0)
    assert a != views.turn_error_deg(5, "sim:stock_can_02", 1, 25.0)
    draws = [views.turn_error_deg(s, f"sim:stock_can_{s % 4}", 1, 25.0) for s in range(2000)]
    mean = sum(draws) / len(draws)
    sd = (sum((d - mean) ** 2 for d in draws) / len(draws)) ** 0.5
    assert abs(mean) < 2.0 and 22.0 < sd < 28.0
