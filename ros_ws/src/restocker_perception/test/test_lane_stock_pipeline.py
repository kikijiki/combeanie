# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The numpy lane depth port and the pipeline, on depth frames ray-cast from the survey station."""

import math

import numpy as np
import pytest
from restocker_perception.stock import estimator as est
from restocker_perception.stock.config import load_stock_config
from restocker_perception.stock.depth_profile import Intrinsics, measure_lane
from restocker_perception.stock.pipeline import (
    estimate_from_frame,
    rear_face_crop,
)

CONFIG = load_stock_config()
CELL = CONFIG.workcell
INTRINSICS = Intrinsics(fx=700.9, fy=700.9, cx=640.0, cy=360.0)
WIDTH, HEIGHT = 1280, 720


def station_pose():
    """lane <- optical of the shipped station: 0.55 m behind the rear edge, 0.55 m up, mid-lane."""
    eye = np.array([0.0, -0.55, 0.55])
    target = np.array([0.0, CELL.rear_clearance_m + CELL.usable_depth_m / 2.0, 0.0])
    boresight = (target - eye) / np.linalg.norm(target - eye)
    right = np.array([1.0, 0.0, 0.0])
    down = np.cross(boresight, right)
    down /= np.linalg.norm(down)
    matrix = np.eye(4)
    matrix[:3, 0], matrix[:3, 1], matrix[:3, 2], matrix[:3, 3] = right, down, boresight, eye
    return matrix


def render(columns, pose, blind_rows=None, tilt=True):
    """Ray-cast depth (z-depth, metres) of a lane: the bed plane plus packed upright cylinders."""
    rows, cols = np.mgrid[0:HEIGHT, 0:WIDTH]
    rays = np.stack(
        [
            (cols - INTRINSICS.cx) / INTRINSICS.fx,
            (rows - INTRINSICS.cy) / INTRINSICS.fy,
            np.ones_like(cols, dtype=float),
        ]
    ).reshape(3, -1)
    origin = pose[:3, 3]
    direction = pose[:3, :3] @ rays  # z-depth parametrisation: t is the optical z
    incline = CELL.incline_rad
    normal = np.array([0.0, math.sin(incline), math.cos(incline)])  # bed normal, leans to +y
    if not tilt:
        normal = np.array([0.0, 0.0, 1.0])
    depth = np.full(direction.shape[1], np.inf)
    # The bed: z = (datum - y) * tan(incline).
    denominator = direction[2] + math.tan(incline) * direction[1]
    with np.errstate(divide="ignore", invalid="ignore"):
        t_bed = (
            math.tan(incline) * (CELL.bed_datum_depth_m - origin[1]) - origin[2]
        ) / denominator
    on_bed = (t_bed > 0) & (np.abs(origin[0] + t_bed * direction[0]) < 0.19)
    depth[on_bed] = t_bed[on_bed]
    # The retainer is outside the window and ignored. Products: finite cylinders about `normal`.
    index = 0
    for geometry, count in columns:
        for k in range(count):
            centre_depth = (
                CELL.bed_datum_depth_m - geometry.radius_m - k * CONFIG.pitch_m(geometry)
            )
            base_y = centre_depth - (geometry.height_m / 2.0) * math.sin(incline)
            base = np.array([0.0, base_y, float(CELL.bed_height_at(base_y))])
            along = normal
            relative = origin - base
            # Side: |p - (p.n)n| = r with p = relative + t d.
            d_perp = direction - np.outer(along, along @ direction)
            r_perp = relative - along * (along @ relative)
            a = np.einsum("ij,ij->j", d_perp, d_perp)
            b = 2.0 * (r_perp @ d_perp)
            c = r_perp @ r_perp - geometry.radius_m**2
            disc = b * b - 4 * a * c
            with np.errstate(invalid="ignore", divide="ignore"):
                t = (-b - np.sqrt(np.where(disc >= 0, disc, np.nan))) / (2 * a)
            axial = (relative[:, None] + t * direction).T @ along
            hit = (disc >= 0) & (t > 0) & (axial >= 0) & (axial <= geometry.height_m)
            # Top disc.
            with np.errstate(invalid="ignore", divide="ignore"):
                t_top = (geometry.height_m - relative @ along) / (along @ direction)
            point = relative[:, None] + t_top * direction
            radial = np.linalg.norm(point - np.outer(along, along @ point), axis=0)
            hit_top = (t_top > 0) & (radial <= geometry.radius_m)
            best = np.where(hit, t, np.inf)
            best = np.minimum(best, np.where(hit_top, t_top, np.inf))
            depth = np.minimum(depth, best)
            index += 1
    depth = np.where(np.isfinite(depth), depth, 0.0).reshape(HEIGHT, WIDTH)
    if blind_rows is not None:
        depth[blind_rows[0] : blind_rows[1]] = 0.0
    return depth


@pytest.mark.parametrize("name", ["can", "small_bottle", "large_bottle"])
@pytest.mark.parametrize("count", [0, 1, 3, 6])
def test_port_matches_the_ground_truth_free_depth_and_the_count(name, count):
    geometry = CONFIG.products[name]
    pose = station_pose()
    depth = render([(geometry, count)] if count else [], pose)
    profile = measure_lane(depth, INTRINSICS, pose, CONFIG, "lane_01")
    expected = CONFIG.free_depth_of_column(geometry, count)
    assert profile.coverage == pytest.approx(1.0)
    assert profile.available_depth_m == pytest.approx(expected, abs=0.012)
    estimate, _ = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=geometry.product_class,
        expected_sku="GEN-001",
    )
    assert estimate.answered and estimate.count_estimate == count
    assert est.class_name(estimate.product_class_assumed) == name or count == 0


def test_wrong_class_in_a_can_lane_is_seen_in_the_depth_alone():
    pose = station_pose()
    depth = render([(CONFIG.products["large_bottle"], 3)], pose)
    estimate, _ = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=CONFIG.products["can"].product_class,
        expected_sku="GEN-001",
    )
    assert estimate.answered and estimate.misplaced and estimate.count_estimate == 3


def test_blinded_depth_is_refused_not_read_as_empty():
    pose = station_pose()
    depth = render([(CONFIG.products["can"], 0)], pose, blind_rows=(0, HEIGHT))
    estimate, profile = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=1,
        expected_sku="GEN-001",
    )
    assert profile.coverage == 0.0
    assert estimate.reason == est.REASON_LOW_COVERAGE and estimate.count_estimate == 0


def test_half_blinded_frame_that_loses_the_lane_is_refused():
    pose = station_pose()
    depth = render([(CONFIG.products["can"], 0)], pose, blind_rows=(0, HEIGHT // 2))
    estimate, profile = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=1,
        expected_sku="GEN-001",
    )
    assert profile.coverage < 0.9 and not estimate.answered


def test_face_crop_is_placed_on_the_rear_product_and_is_none_when_empty_or_tiny():
    pose = station_pose()
    geometry = CONFIG.products["large_bottle"]
    rgb = np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8)
    free = CONFIG.free_depth_of_column(geometry, 3)
    crop = rear_face_crop(rgb, INTRINSICS, pose, CONFIG, free, geometry.product_class)
    assert crop is not None and 0 < crop.u < WIDTH and 0 < crop.v < HEIGHT
    assert crop.image.shape[0] >= 48 or crop.side_px >= 48
    assert rear_face_crop(rgb, INTRINSICS, pose, CONFIG, CELL.usable_depth_m, 3) is None
    assert rear_face_crop(rgb, INTRINSICS, pose, CONFIG, free, 0) is None


def test_identity_channel_is_asked_once_and_applied_with_the_safe_policy():
    pose = station_pose()
    geometry = CONFIG.products["large_bottle"]
    depth = render([(geometry, 2)], pose)
    rgb = np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8)
    calls = []

    def channel(crop):
        calls.append(crop.shape)
        return {
            "sidecar_available": True,
            "status": 0,
            "has_sku": True,
            "sku": "GEN-002",
            "evidence_kinds": ["embedding"],
            "evidence_gtins": ["GEN-002"],
            "registry_sha256": "r",
            "calibration_sha256": "c",
        }

    safe, _ = estimate_from_frame(
        depth,
        rgb,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=geometry.product_class,
        expected_sku="GEN-001",
        identify=channel,
    )
    assert len(calls) == 1
    assert not safe.misplaced and safe.named_count == 0
    assert safe.identity_channel_reachable and safe.identity_registry_sha256 == "r"
    loose, _ = estimate_from_frame(
        depth,
        rgb,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=geometry.product_class,
        expected_sku="GEN-001",
        identify=channel,
        identity_mode="embedding",
    )
    assert loose.misplaced and loose.misplaced_skus == ["GEN-002"]


def test_absent_sidecar_still_counts():
    pose = station_pose()
    geometry = CONFIG.products["can"]
    depth = render([(geometry, 4)], pose)
    rgb = np.full((HEIGHT, WIDTH, 3), 128, dtype=np.uint8)
    estimate, _ = estimate_from_frame(
        depth,
        rgb,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=1,
        expected_sku="GEN-001",
        identify=lambda crop: {"sidecar_available": False},
    )
    assert estimate.answered and estimate.count_estimate == 4
    assert not estimate.identity_channel_reachable
    assert estimate.face_identity_status in ([est.IDENTITY_UNAVAILABLE], [])


@pytest.mark.parametrize("noise", [0.01, 0.02])
def test_range_noise_never_turns_an_empty_lane_into_a_full_one(noise):
    from restocker_perception.stock.benchmark import perturb_depth

    pose = station_pose()
    depth = perturb_depth(render([], pose), "noise-frame", noise, 0.0)
    estimate, profile = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=1,
        expected_sku="GEN-001",
    )
    # Whatever the producer made of the speckle (free depth near zero), the estimator must not
    # answer a count from a surface that no product has the height to be.
    assert not (estimate.answered and estimate.count_estimate > 1)


def test_a_surface_lower_than_every_product_is_refused():
    low = est.TopProfile([0.02 * i + 0.02 for i in range(10)], [0.03] * 10, 0.02)
    evidence = est.LaneEvidence(
        lane_id="lane_01",
        evidence_ok=True,
        coverage=1.0,
        free_depth_m=0.01,
        obstructed=False,
        expected_class=1,
        expected_sku="GEN-001",
        profile=low,
    )
    result = est.estimate_lane_stock(evidence, CONFIG)
    assert result.reason == est.REASON_INCONSISTENT_GEOMETRY and result.count_estimate == 0


@pytest.mark.parametrize("count", [0, 1, 4])
@pytest.mark.parametrize("noise", [0.005, 0.01])
def test_range_noise_is_counted_through_not_refused_or_phantom(count, noise):
    from restocker_perception.stock.benchmark import perturb_depth

    pose = station_pose()
    geometry = CONFIG.products["can"]
    depth = perturb_depth(
        render([(geometry, count)] if count else [], pose), "n-frame", noise, 0.02
    )
    estimate, profile = estimate_from_frame(
        depth,
        None,
        INTRINSICS,
        pose,
        CONFIG,
        lane_id="lane_01",
        expected_class=1,
        expected_sku="GEN-001",
    )
    if noise >= 0.01:
        assert profile.floor_margin_m > CONFIG.measurement.floor_margin_m  # it widened
    assert estimate.answered and estimate.count_estimate == count


def test_ideal_depth_keeps_the_fixed_floor_margin():
    pose = station_pose()
    profile = measure_lane(render([(CONFIG.products["can"], 3)], pose), INTRINSICS, pose, CONFIG)
    assert profile.floor_margin_m == pytest.approx(CONFIG.measurement.floor_margin_m, abs=0.003)
