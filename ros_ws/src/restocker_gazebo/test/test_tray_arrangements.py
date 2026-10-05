# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Stock-tray matrix arrangements stay inside the tray and pin the declared poses."""

from itertools import pairwise
import math
import os
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from restocker_gazebo import tray_arrangements as tray  # noqa: E402


def _product_catalog() -> dict:
    path = os.environ.get("RESTOCKER_TEST_PRODUCT_CATALOG")
    if not path:
        pytest.skip("RESTOCKER_TEST_PRODUCT_CATALOG is not set")
    import yaml

    return yaml.safe_load(Path(path).read_text())


def test_the_restated_catalogue_matches_the_shipped_one():
    """Every geometry this module can stand is in the shipped collision catalogue."""
    geometries = {entry["geometry_key"]: entry for entry in _product_catalog()["geometries"]}
    assert set(tray.TRAY_CATALOGUE) == set(geometries)
    for geometry_key, entry in tray.TRAY_CATALOGUE.items():
        shipped = geometries[geometry_key]
        assert shipped["sku"] == entry["sku"]
        assert shipped["product_class"] == entry["product_class"]
        assert shipped["shape"]["radius_m"] == pytest.approx(entry["radius_m"])
        assert shipped["shape"]["height_m"] == pytest.approx(entry["height_m"])


def test_cans_share_catalogue_albedo_so_chromaticity_detection_can_see_them():
    """White+label can appearance measured 0 overview frames; cans keep the shipped albedo."""
    scenario = tray.isolated_catalogue_scenario(0)
    by_key = {product["geometry_key"]: product for product in scenario["products"]}
    shipped_red = tray.CATALOGUE["can.standard"]["rgba"]
    assert by_key["can.standard"]["rgba"] == shipped_red
    assert by_key["can.citrus"]["rgba"] == shipped_red
    assert "label_texture" not in by_key["can.standard"]
    assert "label_texture" not in by_key["can.citrus"]
    assert by_key["can.standard"]["sku"] == "SIM-CAN-STD"
    assert by_key["can.citrus"]["sku"] == "SIM-CAN-CITRUS"
    # Bottles keep their distinct catalogue colours.
    assert (
        by_key["bottle.small.standard"]["rgba"]
        == tray.TRAY_CATALOGUE["bottle.small.standard"]["rgba"]
    )


def test_isolated_scenario_stands_every_geometry_upright_and_spaced():
    """Cell A: four upright products, surface gaps at least the isolated gap, inside the tray."""
    for seed in (0, 1, 2):
        scenario = tray.isolated_catalogue_scenario(seed)
        products = scenario["products"]
        assert len(products) == 4
        for product in products:
            assert product["static"] is True
            pose = product["spawn_pose"]
            entry = tray.TRAY_CATALOGUE[product["geometry_key"]]
            assert pose[0] - entry["radius_m"] >= tray.TRAY_CENTRE_X_M - tray.TRAY_HALF_WIDTH_M
            assert pose[0] + entry["radius_m"] <= tray.TRAY_CENTRE_X_M + tray.TRAY_HALF_WIDTH_M
            assert pose[1] - entry["radius_m"] >= tray.TRAY_CENTRE_Y_M - tray.TRAY_HALF_DEPTH_M
            assert pose[1] + entry["radius_m"] <= tray.TRAY_CENTRE_Y_M + tray.TRAY_HALF_DEPTH_M
            assert pose[3] == pose[4] == pose[5] == 0.0
            assert pose[2] == pytest.approx(tray.TRAY_SURFACE_Z_M + entry["height_m"] / 2.0)
        ordered = sorted(products, key=lambda product: product["spawn_pose"][0])
        for left, right in pairwise(ordered):
            gap = (
                right["spawn_pose"][0]
                - tray.TRAY_CATALOGUE[right["geometry_key"]]["radius_m"]
                - left["spawn_pose"][0]
                - tray.TRAY_CATALOGUE[left["geometry_key"]]["radius_m"]
            )
            assert gap >= tray.ISOLATED_SURFACE_GAP_M - 1e-9


def test_isolated_scenario_rejects_undeclared_seeds():
    """The matrix predeclares seeds 0,1,2 only; another seed is a programming error."""
    with pytest.raises(ValueError, match="seeds"):
        tray.isolated_catalogue_scenario(7)


def test_touching_pair_places_surfaces_in_contact_for_each_declared_class():
    """Q11 construction: centre distance is exactly r1+r2, same class, both upright."""
    pairs = (
        ("can.standard", "can.citrus"),
        ("bottle.small.standard", "bottle.small.standard"),
        ("bottle.large.standard", "bottle.large.standard"),
    )
    for geometry_a, geometry_b in pairs:
        scenario = tray.touching_pair_scenario(geometry_a, geometry_b)
        first, second = scenario["products"]
        assert first["product_class"] == second["product_class"]
        distance = abs(first["spawn_pose"][0] - second["spawn_pose"][0])
        expected = (
            tray.TRAY_CATALOGUE[geometry_a]["radius_m"]
            + tray.TRAY_CATALOGUE[geometry_b]["radius_m"]
        )
        assert distance == pytest.approx(expected)
        assert first["static"] and second["static"]


def test_touching_pair_rejects_cross_class_pairs():
    """Q11 is same-class separation; a cross-class pair is a different experiment."""
    with pytest.raises(ValueError, match="same-class"):
        tray.touching_pair_scenario("can.standard", "bottle.large.standard")


def test_close_neighbour_gap_is_positive_and_within_the_tray():
    """Q12 neighbour stands close but not touching, inside the usable width."""
    for geometry_key in tray.TRAY_CATALOGUE:
        scenario = tray.close_neighbour_scenario(geometry_key)
        first, second = scenario["products"]
        radius = tray.TRAY_CATALOGUE[geometry_key]["radius_m"]
        surface_gap = second["spawn_pose"][0] - radius - first["spawn_pose"][0] - radius
        assert surface_gap == pytest.approx(tray.CLOSE_NEIGHBOUR_SURFACE_GAP_M)
        assert surface_gap > 0.0
        for product in scenario["products"]:
            x = product["spawn_pose"][0]
            assert x - radius >= tray.TRAY_CENTRE_X_M - tray.TRAY_HALF_WIDTH_M
            assert x + radius <= tray.TRAY_CENTRE_X_M + tray.TRAY_HALF_WIDTH_M


def test_non_upright_scenario_lies_the_product_on_its_side():
    """Q13: roll of π/2, centre one radius above the tray, still inside the footprint."""
    scenario = tray.non_upright_scenario()
    (product,) = scenario["products"]
    pose = product["spawn_pose"]
    entry = tray.TRAY_CATALOGUE[product["geometry_key"]]
    assert pose[3] == pytest.approx(math.pi / 2.0)
    assert pose[2] == pytest.approx(tray.TRAY_SURFACE_Z_M + entry["radius_m"])
    assert product["static"] is True
    assert abs(pose[0]) + entry["radius_m"] <= tray.TRAY_HALF_WIDTH_M


def test_extrinsics_scenario_is_one_upright_can():
    """Q7 varies only the arm configuration; the arrangement stays one pinned can."""
    scenario = tray.extrinsics_scenario()
    (product,) = scenario["products"]
    assert product["geometry_key"] == "can.standard"
    assert product["spawn_pose"][3] == 0.0
    assert tray.expected_product_count(scenario) == 1


def test_tray_scenario_requires_unique_names_and_products():
    """Reject empty and duplicate-name documents rather than emitting an unloadable scenario."""
    with pytest.raises(ValueError, match="at least one"):
        tray.tray_scenario([])
    product = tray._product("can.standard", 1, tray.upright_pose(0.0, -0.80, "can.standard"))
    with pytest.raises(ValueError, match="unique"):
        tray.tray_scenario([product, dict(product)])
