# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Unit tests for scenario and collision-catalog validation."""

from pathlib import Path

import pytest
from restocker_gazebo.scenario_config import (  # noqa: I100,I101 - ament and Ruff disagree
    SUPPORTED_GENERATOR_VERSION,
    load_product_catalog,
    load_randomization,
    load_scenario,
    load_stock_region,
    resolve_product_geometry,
)
import yaml


def _write_yaml(path: Path, value: object) -> Path:
    path.write_text(yaml.safe_dump(value), encoding="utf-8")
    return path


def _geometry(**overrides) -> dict:
    geometry = {
        "geometry_key": "can.standard",
        "product_class": "can",
        "sku": "SIM-CAN-STD",
        "class_fallback": True,
        "shape": {"type": "cylinder", "radius_m": 0.033, "height_m": 0.122},
    }
    geometry.update(overrides)
    return geometry


def test_catalog_loads_valid_cylinder(tmp_path: Path) -> None:
    catalog = load_product_catalog(
        _write_yaml(tmp_path / "catalog.yaml", {"schema_version": 1, "geometries": [_geometry()]})
    )
    assert set(catalog) == {"can.standard"}
    assert catalog["can.standard"]["shape"]["height_m"] == pytest.approx(0.122)


def test_catalog_accepts_two_skus_of_one_class_with_one_fallback(tmp_path: Path) -> None:
    """One shape may carry more than one SKU; only the fallback has to be unique per class."""
    # Makes a same-shaped pair expressible at all. Complements the "multiple fallbacks" rejection
    # below, so tightening that rejection cannot make the shipped catalog unloadable.
    catalog = load_product_catalog(
        _write_yaml(
            tmp_path / "catalog.yaml",
            {
                "schema_version": 1,
                "geometries": [
                    _geometry(),
                    _geometry(
                        geometry_key="can.citrus",
                        sku="SIM-CAN-CITRUS",
                        class_fallback=False,
                    ),
                ],
            },
        )
    )
    assert set(catalog) == {"can.standard", "can.citrus"}
    assert catalog["can.standard"]["shape"] == catalog["can.citrus"]["shape"]


@pytest.mark.parametrize(
    "geometries, expected",
    [
        ([_geometry(), _geometry()], "empty or duplicated"),
        ([_geometry(product_class="crate")], "invalid product_class"),
        (
            [_geometry(shape={"type": "box", "radius_m": 0.1, "height_m": 0.2})],
            "supported cylinder",
        ),
        (
            [_geometry(shape={"type": "cylinder", "radius_m": 0.0, "height_m": 0.2})],
            "invalid radius_m",
        ),
        (
            [_geometry(), _geometry(geometry_key="can.alternate", sku="SIM-CAN-ALT")],
            "multiple fallbacks",
        ),
        (
            [
                _geometry(),
                _geometry(
                    geometry_key="can.alternate",
                    sku="SIM-CAN-STD",
                    class_fallback=False,
                ),
            ],
            "duplicate SKU",
        ),
    ],
)
def test_catalog_rejects_ambiguous_or_invalid_geometry(
    tmp_path: Path, geometries: list[dict], expected: str
) -> None:
    path = _write_yaml(tmp_path / "catalog.yaml", {"schema_version": 1, "geometries": geometries})
    with pytest.raises(RuntimeError, match=expected):
        load_product_catalog(path)


def test_product_resolution_requires_matching_semantics(tmp_path: Path) -> None:
    catalog = load_product_catalog(
        _write_yaml(tmp_path / "catalog.yaml", {"schema_version": 1, "geometries": [_geometry()]})
    )
    product = {
        "geometry_key": "can.standard",
        "product_class": "can",
        "sku": "SIM-CAN-STD",
    }
    assert resolve_product_geometry(product, catalog)["geometry_key"] == "can.standard"

    with pytest.raises(RuntimeError, match="unknown geometry_key"):
        resolve_product_geometry({**product, "geometry_key": "missing"}, catalog)
    with pytest.raises(RuntimeError, match="class disagrees"):
        resolve_product_geometry({**product, "product_class": "small_bottle"}, catalog)
    with pytest.raises(RuntimeError, match="SKU disagrees"):
        resolve_product_geometry({**product, "sku": "WRONG"}, catalog)


def test_scenario_rejects_duplicate_or_missing_model_names(tmp_path: Path) -> None:
    valid = {
        "schema_version": 1,
        "workcell_pose": [0.0] * 6,
        "products": [{"model_name": "stock_can_01"}],
    }
    assert load_scenario(_write_yaml(tmp_path / "valid.yaml", valid))["products"]

    duplicate = {**valid, "products": [{"model_name": "same"}, {"model_name": "same"}]}
    with pytest.raises(RuntimeError, match="non-empty and unique"):
        load_scenario(_write_yaml(tmp_path / "duplicate.yaml", duplicate))
    missing = {**valid, "products": [{"model_name": ""}]}
    with pytest.raises(RuntimeError, match="non-empty and unique"):
        load_scenario(_write_yaml(tmp_path / "missing.yaml", missing))


def test_scenario_label_texture_is_optional_but_must_be_a_bare_filename(tmp_path: Path) -> None:
    """The launch file joins the name onto one directory, so anything else resolves nowhere."""
    # An accepted name Gazebo cannot open gives a product rendered flat white with nothing said,
    # which would corrupt a vision measurement.
    base = {"schema_version": 1, "workcell_pose": [0.0] * 6}
    untextured = {**base, "products": [{"model_name": "stock_can_01"}]}
    assert (
        "label_texture"
        not in load_scenario(_write_yaml(tmp_path / "plain.yaml", untextured))["products"][0]
    )

    named = {
        **base,
        "products": [{"model_name": "stock_can_01", "label_texture": "can_citrus_label.png"}],
    }
    assert (
        load_scenario(_write_yaml(tmp_path / "named.yaml", named))["products"][0]["label_texture"]
        == "can_citrus_label.png"
    )

    for value, expected in (
        ("", "non-empty filename"),
        (7, "non-empty filename"),
        ("materials/textures/can_citrus_label.png", "bare filename"),
        ("..", "bare filename"),
    ):
        rejected = {**base, "products": [{"model_name": "x", "label_texture": value}]}
        with pytest.raises(RuntimeError, match=expected):
            load_scenario(_write_yaml(tmp_path / "rejected.yaml", rejected))


def _randomization(**overrides) -> dict:
    randomization = {
        "generator_version": SUPPORTED_GENERATOR_VERSION,
        "stock_region": {
            "source": "stock_tray.usable_volume",
            "wall_margin_m": 0.01,
            "surface_separation_m": 0.02,
            "max_placement_attempts": 32,
        },
        "product_count": {"minimum": 1, "maximum": 3},
        "variants": [
            {
                "geometry_key": "can.standard",
                "product_class": "can",
                "sku": "SIM-CAN-STD",
                "mass_kg": 0.355,
                "rgba": [0.78, 0.12, 0.08, 1.0],
                "friction": 0.65,
                "model_name_stem": "stock_can",
                # Three lanes because product_count.maximum is three: the loader refuses bounds
                # asking for more products than the declared lanes accept.
                "destination_lanes": ["lane_01", "lane_02", "lane_03"],
            }
        ],
    }
    randomization.update(overrides)
    return randomization


def _scenario_with(randomization: object) -> dict:
    return {
        "schema_version": 1,
        "workcell_pose": [0.0] * 6,
        "products": [{"model_name": "stock_can_01"}],
        "randomization": randomization,
    }


def test_randomization_bounds_load_when_completely_specified() -> None:
    bounds = load_randomization(_scenario_with(_randomization()))
    assert bounds["product_count"] == {"minimum": 1, "maximum": 3}
    assert [variant["model_name_stem"] for variant in bounds["variants"]] == ["stock_can"]


def _mutate(path: tuple[str, ...], value: object) -> dict:
    randomization = _randomization()
    target = randomization
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    return randomization


@pytest.mark.parametrize(
    "randomization, expected",
    [
        ("not a mapping", "randomization must be a mapping"),
        (
            _mutate(("generator_version",), SUPPORTED_GENERATOR_VERSION + 1),
            f"generator_version {SUPPORTED_GENERATOR_VERSION}",
        ),
        (
            _mutate(("generator_version",), SUPPORTED_GENERATOR_VERSION - 1),
            f"generator_version {SUPPORTED_GENERATOR_VERSION}",
        ),
        (_mutate(("stock_region", "source"), "shelf"), "stock_region source"),
        (_mutate(("stock_region", "wall_margin_m"), -0.01), "wall_margin_m"),
        (_mutate(("stock_region", "surface_separation_m"), float("nan")), "surface_separation_m"),
        (_mutate(("stock_region", "max_placement_attempts"), 0), "max_placement_attempts"),
        (_mutate(("stock_region", "max_placement_attempts"), True), "max_placement_attempts"),
        (_mutate(("product_count", "minimum"), 0), "product_count minimum"),
        (_mutate(("product_count", "maximum"), 0), "product_count maximum"),
        (_mutate(("product_count",), {"minimum": 4, "maximum": 2}), "must not be below minimum"),
        # A lane holds one product, so bounds asking for more products than lanes describe a
        # scenario no robot could complete. Rejected here rather than capped in the draw, since
        # it is a property of the bounds, not of one seed.
        (_mutate(("product_count",), {"minimum": 1, "maximum": 4}), "exceeds the 3 lanes"),
        (_mutate(("variants",), []), "at least one variant"),
        (_randomization(variants=[{"product_class": "can"}]), "requires a geometry_key"),
    ],
)
def test_randomization_rejects_unusable_bounds(randomization: object, expected: str) -> None:
    with pytest.raises(RuntimeError, match=expected):
        load_randomization(_scenario_with(randomization))


@pytest.mark.parametrize(
    "override, expected",
    [
        ({"product_class": "crate"}, "invalid product_class"),
        ({"sku": ""}, "requires a sku"),
        ({"mass_kg": 0.0}, "variant mass_kg"),
        ({"friction": -1.0}, "variant friction"),
        ({"rgba": [0.1, 0.2, 0.3]}, "four components"),
        ({"rgba": [0.1, 0.2, 0.3, 1.5]}, "must lie in .0, 1."),
        ({"model_name_stem": ""}, "model_name_stem is empty or duplicated"),
        ({"destination_lanes": []}, "at least one destination lane"),
        ({"destination_lanes": ["lane_01", "lane_01"]}, "non-empty and unique"),
        ({"destination_lanes": ["lane_01"]}, "exceeds the 1 lanes"),
    ],
)
def test_randomization_rejects_unusable_variant(override: dict, expected: str) -> None:
    variant = {**_randomization()["variants"][0], **override}
    with pytest.raises(RuntimeError, match=expected):
        load_randomization(_scenario_with(_randomization(variants=[variant])))


def test_randomization_rejects_duplicate_variant_stems() -> None:
    variant = _randomization()["variants"][0]
    with pytest.raises(RuntimeError, match="empty or duplicated"):
        load_randomization(_scenario_with(_randomization(variants=[variant, dict(variant)])))


def test_scenario_load_validates_bounds_eagerly_when_present(tmp_path: Path) -> None:
    """A typo in the bounds must fail on the default launch, not on the first seeded run."""
    scenario = _scenario_with(_mutate(("generator_version",), 7))
    with pytest.raises(RuntimeError, match=f"generator_version {SUPPORTED_GENERATOR_VERSION}"):
        load_scenario(_write_yaml(tmp_path / "scenario.yaml", scenario))
    # A scenario carrying no bounds at all stays loadable: randomization is opt-in.
    without = {key: value for key, value in scenario.items() if key != "randomization"}
    assert load_scenario(_write_yaml(tmp_path / "plain.yaml", without))["products"]


def _geometry_document(**volume_overrides) -> dict:
    volume = {
        "center_xyz_m": [0.0, -1.35, -0.005],
        "size_xyz_m": [1.36, 0.51, 0.35],
        "floor_settle_tolerance_m": 0.002,
    }
    volume.update(volume_overrides)
    return {"schema_version": 1, "stock_tray": {"usable_volume": volume}}


def test_stock_region_loads_the_surveyed_usable_volume(tmp_path: Path) -> None:
    region = load_stock_region(_write_yaml(tmp_path / "geometry.yaml", _geometry_document()))
    assert region.center_xyz_m == (0.0, -1.35, -0.005)
    assert region.size_xyz_m == (1.36, 0.51, 0.35)
    assert region.floor_settle_tolerance_m == pytest.approx(0.002)


@pytest.mark.parametrize(
    "document, expected",
    [
        ({"schema_version": 2}, "schema_version 1"),
        ({"schema_version": 1}, "stock_tray.usable_volume"),
        (_geometry_document(center_xyz_m=[0.0, -1.35]), "center_xyz_m must contain three"),
        (_geometry_document(size_xyz_m=[1.36, 0.51]), "size_xyz_m must contain three"),
        (_geometry_document(size_xyz_m=[1.36, 0.0, 0.35]), "size_xyz_m"),
        (_geometry_document(floor_settle_tolerance_m=-0.1), "floor_settle_tolerance_m"),
    ],
)
def test_stock_region_rejects_an_unusable_survey(
    tmp_path: Path, document: dict, expected: str
) -> None:
    with pytest.raises(RuntimeError, match=expected):
        load_stock_region(_write_yaml(tmp_path / "geometry.yaml", document))
