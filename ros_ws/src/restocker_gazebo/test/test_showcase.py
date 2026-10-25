# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The showcase fixture is deterministic, collision-free and consistent with its lane policy."""

from collections import Counter
from itertools import combinations, pairwise
import math
import os
from pathlib import Path

import pytest
from restocker_gazebo import showcase  # noqa: I100,I101 - ament and Ruff disagree
from restocker_gazebo.lane_columns import (
    CATALOGUE,
    LANE_CENTER_X_M,
    TRAY_SURFACE_Z_M,
    WORKCELL_POSE,
    column_pose_in_lane,
)
from restocker_gazebo.scenario_config import (
    load_approach_corridor,
    load_product_catalog,
    load_scenario,
    resolve_product_geometry,
)
from restocker_gazebo.scenario_random import required_lateral_distance
import yaml


def _source(name: str) -> Path:
    value = os.environ.get(name)
    if not value:
        pytest.skip(f"{name} is not set")
    return Path(value)


@pytest.fixture(scope="module")
def labels() -> dict:
    return yaml.safe_load(
        (
            _source("RESTOCKER_GAZEBO_SOURCE_DIR") / "config" / "generated_product_labels.yaml"
        ).read_text()
    )


@pytest.fixture(scope="module")
def scenario(labels) -> dict:
    return showcase.showcase_scenario(labels)


def _shelf(scenario: dict) -> list[dict]:
    return [product for product in scenario["products"] if product.get("static")]


def _tray(scenario: dict) -> list[dict]:
    return [product for product in scenario["products"] if not product.get("static")]


def _radius(product: dict) -> float:
    return CATALOGUE[showcase.BASE_GEOMETRY[product["product_class"]]]["radius_m"]


def test_the_scenario_is_deterministic(labels):
    assert showcase.showcase_scenario(labels) == showcase.showcase_scenario(labels)
    assert showcase.lane_policy(labels) == showcase.lane_policy(labels)


def test_tray_yaw_is_seeded_per_product_and_the_shelf_is_unchanged(scenario, labels):
    tray = _tray(scenario)
    yaws = [product["spawn_pose"][5] for product in tray]
    for product in tray:
        assert product["spawn_pose"][3:5] == [0.0, 0.0]
        assert product["spawn_pose"][5] == showcase.tray_yaw_rad(product["model_name"])
        assert 0.0 <= product["spawn_pose"][5] < 2.0 * math.pi
    # Distinct, spread over the whole turn, and reproducible from the name alone.
    assert len(set(yaws)) == len(yaws)
    assert all(
        any(low <= yaw < low + math.pi / 2 for yaw in yaws)
        for low in (0.0, math.pi / 2, math.pi, 3 * math.pi / 2)
    )
    assert showcase.tray_yaw_rad("stock_gen_001_01") == showcase.tray_yaw_rad("stock_gen_001_01")
    # Lane products all keep the one half-turn that faces the shelf front.
    assert len({round(product["spawn_pose"][5], 6) for product in _shelf(scenario)}) == 1


def test_counts_cover_every_shape_with_several_skus(scenario):
    shelf = Counter(product["sku"] for product in _shelf(scenario))
    tray = Counter(product["sku"] for product in _tray(scenario))
    assert shelf == {"GEN-001": 7, "GEN-005": 9, "GEN-009": 9, "GEN-003": 8, "GEN-007": 6}
    assert tray == {"GEN-007": 9, "GEN-001": 10, "GEN-009": 7, "GEN-003": 9, "GEN-005": 10}
    shapes = {product["product_class"] for product in scenario["products"]}
    assert shapes == {"can", "small_bottle", "large_bottle"}
    skus_by_shape: dict[str, set[str]] = {}
    for product in scenario["products"]:
        skus_by_shape.setdefault(product["product_class"], set()).add(product["sku"])
    assert all(len(skus) >= 1 for skus in skus_by_shape.values())
    assert sum(len(skus) > 1 for skus in skus_by_shape.values()) >= 2
    assert len(scenario["products"]) == 84


def test_every_product_is_a_labelled_manipulable_catalogue_product(scenario, labels):
    catalogue = load_product_catalog(
        _source("RESTOCKER_DESCRIPTION_SOURCE_DIR") / "config" / "product_collision_catalog.yaml"
    )
    rows = {row["sku"]: row for row in labels["products"]}
    names = [product["model_name"] for product in scenario["products"]]
    assert len(names) == len(set(names))
    for product in scenario["products"]:
        row = rows[product["sku"]]
        assert product["geometry_key"] == row["geometry_key"]
        assert product["label_texture"] == row["texture"]
        assert product["product_class"] == row["shape"] in showcase.BASE_GEOMETRY
        geometry = resolve_product_geometry(product, catalogue)
        base = CATALOGUE[showcase.BASE_GEOMETRY[row["shape"]]]
        assert geometry["shape"]["radius_m"] == base["radius_m"]
        assert geometry["shape"]["height_m"] == base["height_m"]
        assert product["mass_kg"] == base["mass_kg"]
        assert product["rgba"] == [1.0, 1.0, 1.0, 1.0]


def test_the_scenario_loads_through_the_launch_validator(scenario, tmp_path):
    path = tmp_path / "scenario.yaml"
    path.write_text(yaml.safe_dump(scenario))
    assert load_scenario(path)["products"] == scenario["products"]


def test_shelf_columns_stand_on_the_computed_lane_poses(scenario, labels):
    rows = {row["sku"]: row for row in labels["products"]}
    for lane_id, sku, count in showcase.SHELF:
        base = showcase.BASE_GEOMETRY[rows[sku]["shape"]]
        column = [p for p in _shelf(scenario) if p["model_name"].startswith(f"{lane_id}_")]
        assert len(column) == count
        assert {p["sku"] for p in column} == {sku}
        for index, product in enumerate(column):
            x, y, z = product["spawn_pose"][:3]
            depth, height, _ = column_pose_in_lane(base, index)
            assert x == pytest.approx(LANE_CENTER_X_M[lane_id] + WORKCELL_POSE[0], abs=1e-6)
            assert y == pytest.approx(depth + WORKCELL_POSE[1], abs=1e-6)
            assert z == pytest.approx(height + WORKCELL_POSE[2], abs=1e-6)


def test_the_label_turn_keeps_the_product_axis_on_the_lane_bed(scenario):
    # Turning a product about its own axis must not change where the axis points.
    for product in _shelf(scenario):
        roll, pitch, yaw = product["spawn_pose"][3:]
        axis = (
            math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
            math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
            math.cos(pitch) * math.cos(roll),
        )
        incline = math.radians(4.0)
        assert axis[0] == pytest.approx(0.0, abs=1e-5)
        assert axis[1] == pytest.approx(math.sin(incline), abs=1e-5)
        assert axis[2] == pytest.approx(math.cos(incline), abs=1e-5)


def test_no_two_products_overlap(scenario):
    incline = math.radians(4.0)
    for first, second in combinations(scenario["products"], 2):
        distance = math.hypot(
            first["spawn_pose"][0] - second["spawn_pose"][0],
            first["spawn_pose"][1] - second["spawn_pose"][1],
        )
        minimum = _radius(first) + _radius(second)
        lane = first["model_name"][:7]
        if first.get("static") and second["model_name"].startswith(lane):
            # Neighbours in a lane lean on the inclined bed: they touch at the contact pitch.
            minimum *= math.cos(incline)
        assert distance >= minimum - 1e-5, (first["model_name"], second["model_name"])


def test_tray_blocks_are_packed_rows_on_the_corridor_pitch(scenario):
    corridor = load_approach_corridor(
        _source("RESTOCKER_DESCRIPTION_SOURCE_DIR") / "config" / "gripper_geometry.yaml"
    )
    tray = _tray(scenario)
    columns = sorted({product["spawn_pose"][0] for product in tray})
    assert len(columns) == len(showcase.BACK_ROOM) * showcase.COLUMNS_PER_BLOCK
    for left, right in pairwise(columns):
        radii = [_radius(p) for p in tray if p["spawn_pose"][0] in (left, right)]
        # The pair's own radii bound the pitch from below; the widest in the pair is the case.
        needed = required_lateral_distance(max(radii), max(radii), corridor.half_width_m)
        assert right - left >= needed + 0.003
    for sku, count in showcase.BACK_ROOM:
        block = [p for p in tray if p["sku"] == sku]
        assert len(block) == count
        radius = _radius(block[0])
        rows = sorted({p["spawn_pose"][1] for p in block}, reverse=True)
        assert len({p["spawn_pose"][0] for p in block}) == showcase.COLUMNS_PER_BLOCK
        for front, behind in pairwise(rows):
            assert front - behind == pytest.approx(showcase.row_pitch_m(radius), abs=1e-6)
            assert front - behind - 2.0 * radius == pytest.approx(showcase.ROW_GAP_M, abs=1e-6)
        # A partly consumed block loses its last slots only, never a middle one.
        assert Counter(p["spawn_pose"][1] for p in block[:-1]).most_common(1)[0][1] == (
            showcase.COLUMNS_PER_BLOCK
        )
        assert all(
            p["spawn_pose"][2]
            == pytest.approx(
                TRAY_SURFACE_Z_M
                + CATALOGUE[showcase.BASE_GEOMETRY[p["product_class"]]]["height_m"] / 2
            )
            for p in block
        )


def test_the_tray_stays_inside_the_surveyed_volume_and_reach(scenario):
    workcell = yaml.safe_load(
        (
            _source("RESTOCKER_DESCRIPTION_SOURCE_DIR") / "config" / "workcell_geometry.yaml"
        ).read_text()
    )
    volume = workcell["stock_tray"]["usable_volume"]
    centre_x = WORKCELL_POSE[0] + volume["center_xyz_m"][0]
    centre_y = WORKCELL_POSE[1] + volume["center_xyz_m"][1]
    half_x, half_y = volume["size_xyz_m"][0] / 2.0, volume["size_xyz_m"][1] / 2.0
    for product in _tray(scenario):
        x, y = product["spawn_pose"][:2]
        radius = _radius(product)
        assert centre_x - half_x <= x - radius
        assert x + radius <= centre_x + half_x
        assert centre_y - half_y <= y - radius
        assert y + radius <= centre_y + half_y
        assert showcase.FAR_ROW_LIMIT_Y_M <= y <= showcase.FRONT_ROW_Y_M
        # Clear of the tray walls by more than the open gripper's lateral reach beyond the wrist.
        assert abs(x) + radius <= 0.68 - 0.02


def test_lane_policy_matches_the_shelf_and_the_back_room_covers_its_deficits(scenario, labels):
    policy = showcase.lane_policy(labels)["lanes"]
    assert list(policy) == list(LANE_CENTER_X_M)
    rows = {row["sku"]: row for row in labels["products"]}
    shelf = Counter(p["model_name"][:7] for p in _shelf(scenario))
    tray = Counter(p["sku"] for p in _tray(scenario))
    short: Counter[str] = Counter()
    for lane_id, sku, count in showcase.SHELF:
        lane = policy[lane_id]
        base = showcase.BASE_GEOMETRY[rows[sku]["shape"]]
        assert lane["expected_sku"] == sku
        assert lane["expected_product_class"] == rows[sku]["shape"]
        assert lane["target_count"] == showcase.TARGET_COUNT[base]
        assert shelf[lane_id] == count <= lane["target_count"] <= CATALOGUE[base]["lane_capacity"]
        short[sku] += lane["target_count"] - count
    assert short == showcase.deficits(labels)
    # The robot has work in more than one lane per shape that is stocked twice, and every
    # deficit has back stock behind it.
    assert sum(1 for _, _, count in showcase.SHELF if count) == 6
    assert sum(short.values()) >= 8
    for sku, missing in short.items():
        assert tray[sku] >= missing
    assert set(tray) == {sku for _, sku, _ in showcase.SHELF}


def test_some_lanes_start_at_their_target_and_some_start_short(labels):
    short = [
        showcase.TARGET_COUNT[
            showcase.BASE_GEOMETRY[
                next(row["shape"] for row in labels["products"] if row["sku"] == sku)
            ]
        ]
        - count
        for _, sku, count in showcase.SHELF
    ]
    assert any(value == 0 for value in short)
    assert sum(value > 0 for value in short) >= 3


def test_shipped_documents_match_the_builder(scenario, labels):
    gazebo = _source("RESTOCKER_GAZEBO_SOURCE_DIR")
    world_state = _source("RESTOCKER_WORLD_STATE_SOURCE_DIR")
    loaded = load_scenario(gazebo / "config" / "showcase_products.yaml")
    assert loaded["products"] == scenario["products"]
    shipped = yaml.safe_load((world_state / "config" / "showcase_lanes.yaml").read_text())
    assert shipped == showcase.lane_policy(labels)
    for path in (
        gazebo / "config" / "showcase_products.yaml",
        world_state / "config" / "showcase_lanes.yaml",
    ):
        assert path.read_text().startswith("# Generated by tools/generate_showcase_scenario.py")
