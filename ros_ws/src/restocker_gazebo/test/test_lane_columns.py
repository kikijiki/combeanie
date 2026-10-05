# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The known-column scenarios agree with the workcell and the catalogue they restate."""

from collections import Counter
from itertools import pairwise
import math
import os
from pathlib import Path

import pytest
from restocker_gazebo.lane_columns import (  # noqa: I100,I101 - ament and Ruff disagree
    BED_DATUM_DEPTH_M,
    CATALOGUE,
    DENSE_RESTOCK_FRONT_COLUMNS,
    DENSE_RESTOCK_TRAY_GRIDS,
    LANE_CENTER_X_M,
    LANE_INCLINE_RAD,
    SENSOR_ACCEPTANCE_FRONT_COLUMNS,
    SENSOR_ACCEPTANCE_TRAY_LAYOUT,
    WORKCELL_POSE,
    LaneColumn,
    bed_height_at,
    column_pose_in_lane,
    dense_restock_scenario,
    ground_truth_available_depth_m,
    lane_column_scenario,
    sensor_acceptance_scenario,
)
from restocker_gazebo.scenario_config import load_approach_corridor, load_scenario
from restocker_gazebo.scenario_random import required_lateral_distance
import yaml


def _workcell() -> dict:
    path = os.environ.get("RESTOCKER_TEST_WORKCELL_GEOMETRY")
    if not path:
        pytest.skip("RESTOCKER_TEST_WORKCELL_GEOMETRY is not set")
    return yaml.safe_load(Path(path).read_text())


def _catalogue() -> dict:
    path = os.environ.get("RESTOCKER_TEST_PRODUCT_CATALOG")
    if not path:
        pytest.skip("RESTOCKER_TEST_PRODUCT_CATALOG is not set")
    return yaml.safe_load(Path(path).read_text())


def test_the_restated_workcell_matches_the_shipped_one():
    """This module carries the shelf's numbers so a scenario can be written without a workspace."""
    # Safe only while they are the same numbers: this goes red if the incline, the retainer
    # depth or a lane centre moves.
    workcell = _workcell()
    shelf = workcell["shelf"]
    assert pytest.approx(shelf["depth_m"] - shelf["front_retainer_depth_m"]) == BED_DATUM_DEPTH_M
    assert pytest.approx(math.radians(shelf["lane_incline_deg"])) == LANE_INCLINE_RAD
    assert {
        lane_id: lane["center_x_m"] for lane_id, lane in workcell["lanes"].items()
    } == LANE_CENTER_X_M


def test_the_restated_catalogue_matches_the_shipped_one():
    """The same, for the two product dimensions a column's geometry is made of."""
    geometries = {entry["geometry_key"]: entry for entry in _catalogue()["geometries"]}
    for geometry_key, entry in CATALOGUE.items():
        shipped = geometries[geometry_key]
        assert shipped["product_class"] == entry["product_class"]
        assert shipped["sku"] == entry["sku"]
        assert shipped["shape"]["radius_m"] == pytest.approx(entry["radius_m"])
        assert shipped["shape"]["height_m"] == pytest.approx(entry["height_m"])


def test_the_frontmost_product_leans_on_the_retainer_and_the_rest_pack_behind_it():
    """The column is the gravity feed run forwards: one contact diameter per product."""
    for geometry_key, entry in CATALOGUE.items():
        radius = entry["radius_m"]
        front_depth, _, roll = column_pose_in_lane(geometry_key, 0)
        assert front_depth == pytest.approx(BED_DATUM_DEPTH_M - radius)
        # Rolled by minus the incline, which takes +Z onto the bed normal.
        assert roll == pytest.approx(-LANE_INCLINE_RAD)
        second_depth, _, _ = column_pose_in_lane(geometry_key, 1)
        assert front_depth - second_depth == pytest.approx(
            2.0 * radius * math.cos(LANE_INCLINE_RAD)
        )


def test_every_product_stands_on_the_bed_rather_than_in_it():
    """The centre is half a height along the bed normal from a base that touches the surface."""
    for geometry_key, entry in CATALOGUE.items():
        height = entry["height_m"]
        for index in range(entry["lane_capacity"]):
            center_depth, center_height, _ = column_pose_in_lane(geometry_key, index)
            base_depth = center_depth - ((height / 2.0) * math.sin(LANE_INCLINE_RAD))
            base_height = center_height - ((height / 2.0) * math.cos(LANE_INCLINE_RAD))
            assert base_height == pytest.approx(bed_height_at(base_depth))


def test_a_full_lane_of_every_product_still_fits_behind_the_entrance():
    """The capacities this module carries have to leave the rearmost product inside the lane."""
    for geometry_key, entry in CATALOGUE.items():
        free = ground_truth_available_depth_m(geometry_key, entry["lane_capacity"])
        assert free > 0.0, f"{geometry_key} does not fit {entry['lane_capacity']} in one lane"
        # And one more would not be admitted: room at the rear is the product's own diameter plus
        # the lane's 0.045 m entry clearance.
        assert free < (2.0 * entry["radius_m"]) + 0.045


def test_the_generated_scenario_validates_and_pins_every_product(tmp_path):
    """It has to load through the same validator the simulator's launch runs it through."""
    scenario = lane_column_scenario(
        [
            LaneColumn("lane_01", "can.standard", 3),
            LaneColumn("lane_03", "bottle.large.standard", 1),
        ]
    )
    document = tmp_path / "scenario.yaml"
    document.write_text(yaml.safe_dump(scenario, sort_keys=False))
    loaded = load_scenario(document)
    assert len(loaded["products"]) == 4
    assert all(product["static"] for product in loaded["products"])
    assert loaded["workcell_pose"] == WORKCELL_POSE
    # Four distinct poses: a column sharing one would be the failure this arrangement avoids.
    poses = {tuple(product["spawn_pose"]) for product in loaded["products"]}
    assert len(poses) == 4


def test_it_refuses_a_column_longer_than_the_lane_takes():
    """A scenario that cannot exist is a bad experiment, not a small measurement error."""
    with pytest.raises(ValueError):
        lane_column_scenario([LaneColumn("lane_01", "can.standard", 13)])
    with pytest.raises(ValueError):
        lane_column_scenario([LaneColumn("lane_09", "can.standard", 1)])
    with pytest.raises(ValueError):
        lane_column_scenario([LaneColumn("lane_01", "bottle.tiny", 1)])


def test_an_empty_lane_reads_as_the_whole_lane():
    """The reference the camera has to reproduce, and the reading a blind camera also gives."""
    assert ground_truth_available_depth_m("can.standard", 0) == pytest.approx(0.85)


def test_dense_restock_scenario_matches_partial_front_deficits_with_partial_back_grids():
    """The demo has real deficits, exactly enough back stock to fill them, and no surplus."""
    scenario = dense_restock_scenario()
    front = [product for product in scenario["products"] if product.get("static")]
    tray = [product for product in scenario["products"] if not product.get("static")]

    front_counts = Counter(product["sku"] for product in front)
    tray_counts = Counter(product["sku"] for product in tray)
    assert front_counts == {
        "SIM-CAN-STD": 15,
        "SIM-BOTTLE-SMALL": 13,
        "SIM-BOTTLE-LARGE": 9,
    }
    assert tray_counts == {
        "SIM-CAN-STD": 9,
        "SIM-BOTTLE-SMALL": 9,
        "SIM-BOTTLE-LARGE": 7,
    }

    capacity_by_sku: Counter[str] = Counter()
    for column in DENSE_RESTOCK_FRONT_COLUMNS:
        entry = CATALOGUE[column.geometry_key]
        assert 0 < column.count < entry["lane_capacity"]
        capacity_by_sku[entry["sku"]] += entry["lane_capacity"]
    assert capacity_by_sku - front_counts == tray_counts

    # Shelf rear begins at world Y=workcell_pose.y. Every stocked product is on the negative-Y
    # robot/tray side, while the customer-facing front lies further along +Y.
    assert all(product["spawn_pose"][1] < WORKCELL_POSE[1] for product in tray)

    # Front first-occurrence order follows lane numbering. Back blocks are intentionally ordered
    # independently, and each is a proper two-dimensional grid with one realistic empty slot.
    front_order = list(dict.fromkeys(product["geometry_key"] for product in front))
    tray_order = list(dict.fromkeys(product["geometry_key"] for product in tray))
    assert front_order == ["can.standard", "bottle.small.standard", "bottle.large.standard"]
    assert tray_order == ["bottle.large.standard", "can.standard", "bottle.small.standard"]
    assert tray_order != front_order
    for grid in DENSE_RESTOCK_TRAY_GRIDS:
        products = [product for product in tray if product["geometry_key"] == grid.geometry_key]
        assert len(products) == grid.count
        assert len(grid.column_x_m) == 2
        assert len(set(product["spawn_pose"][0] for product in products)) == 2
        assert len(set(product["spawn_pose"][1] for product in products)) == len(grid.row_y_m)
        assert 0 < grid.count < len(grid.column_x_m) * len(grid.row_y_m)


def test_dense_restock_grid_spacing_clears_the_gripper_and_tray_walls():
    """Side pitch admits the stock-side wrist corridor; row pitch seats distinct products."""
    description_source = os.environ.get("RESTOCKER_DESCRIPTION_SOURCE_DIR")
    if not description_source:
        pytest.skip("RESTOCKER_DESCRIPTION_SOURCE_DIR is not set")
    corridor = load_approach_corridor(
        Path(description_source) / "config" / "gripper_geometry.yaml"
    )
    workcell = _workcell()
    volume = workcell["stock_tray"]["usable_volume"]
    world_center_x = WORKCELL_POSE[0] + volume["center_xyz_m"][0]
    world_center_y = WORKCELL_POSE[1] + volume["center_xyz_m"][1]
    x_min = world_center_x - (volume["size_xyz_m"][0] / 2.0)
    x_max = world_center_x + (volume["size_xyz_m"][0] / 2.0)
    y_min = world_center_y - (volume["size_xyz_m"][1] / 2.0)
    y_max = world_center_y + (volume["size_xyz_m"][1] / 2.0)

    for grid in DENSE_RESTOCK_TRAY_GRIDS:
        radius = CATALOGUE[grid.geometry_key]["radius_m"]
        assert grid.column_x_m[1] - grid.column_x_m[0] >= required_lateral_distance(
            radius, radius, corridor.half_width_m
        )
        for first, second in pairwise(grid.row_y_m):
            assert first - second >= (2.0 * radius) + 0.015
        assert x_min <= min(grid.column_x_m) - radius
        assert max(grid.column_x_m) + radius <= x_max
        assert y_min <= min(grid.row_y_m) - radius
        assert max(grid.row_y_m) + radius <= y_max


def test_shipped_dense_restock_products_matches_the_builder():
    """Shipped dense_restock_products.yaml must match dense_restock_scenario()."""
    gazebo_source = os.environ.get("RESTOCKER_GAZEBO_SOURCE_DIR")
    if not gazebo_source:
        pytest.skip("RESTOCKER_GAZEBO_SOURCE_DIR is not set")
    path = Path(gazebo_source) / "config" / "dense_restock_products.yaml"
    loaded = load_scenario(path)
    expected = dense_restock_scenario()
    assert loaded["products"] == expected["products"]
    assert sum(1 for product in loaded["products"] if product.get("static")) == 37
    assert sum(1 for product in loaded["products"] if not product.get("static")) == 25


def test_sensor_acceptance_empty_depleted_lanes_and_identity_decidable_tray():
    """Card 010 SC-001: empty depleted lanes, one tray product per class for camera identity."""
    scenario = sensor_acceptance_scenario()
    front = [product for product in scenario["products"] if product.get("static")]
    tray = [product for product in scenario["products"] if not product.get("static")]

    # Depleted lanes start empty: no front-column products, so the object manifest is the
    # tray alone and ObservationIdentityAssigner sees exactly one product per class.
    assert front == []
    assert SENSOR_ACCEPTANCE_FRONT_COLUMNS == ()
    tray_classes = Counter(product["product_class"] for product in tray)
    assert tray_classes == {"can": 1, "small_bottle": 1, "large_bottle": 1}
    # Unordered: the class sequence is not the baseline's can/small/large row order.
    class_order = [product["product_class"] for product in tray]
    assert class_order != ["can", "small_bottle", "large_bottle"]
    assert all(not product.get("static") for product in tray)


def test_sensor_acceptance_tray_clearance_and_reach():
    """Every mixed-tray pair keeps the isolated gap; poses stay on the tray and reachable."""
    scenario = sensor_acceptance_scenario()
    tray = [product for product in scenario["products"] if not product.get("static")]
    for first, second in pairwise(tray):
        first_radius = CATALOGUE[first["geometry_key"]]["radius_m"]
        second_radius = CATALOGUE[second["geometry_key"]]["radius_m"]
        distance = math.hypot(
            first["spawn_pose"][0] - second["spawn_pose"][0],
            first["spawn_pose"][1] - second["spawn_pose"][1],
        )
        assert distance - first_radius - second_radius >= 0.04
    for product in tray:
        x_m, y_m = product["spawn_pose"][0], product["spawn_pose"][1]
        radius = CATALOGUE[product["geometry_key"]]["radius_m"]
        assert abs(x_m) + radius <= 0.68 + 1e-9
        # Pre-grasp reach: the product must sit deeper than the measured near-edge bound and
        # not past the far tray edge (~y = -0.955) where some starts cannot finish.
        assert -0.95 <= y_m <= -0.40
    assert len(SENSOR_ACCEPTANCE_TRAY_LAYOUT) == 3


def test_shipped_sensor_acceptance_products_matches_the_builder():
    """Shipped sensor_acceptance_products.yaml must match sensor_acceptance_scenario()."""
    gazebo_source = os.environ.get("RESTOCKER_GAZEBO_SOURCE_DIR")
    if not gazebo_source:
        pytest.skip("RESTOCKER_GAZEBO_SOURCE_DIR is not set")
    path = Path(gazebo_source) / "config" / "sensor_acceptance_products.yaml"
    loaded = load_scenario(path)
    expected = sensor_acceptance_scenario()
    assert loaded["products"] == expected["products"]
    assert sum(1 for product in loaded["products"] if product.get("static")) == 0
    assert sum(1 for product in loaded["products"] if not product.get("static")) == 3
