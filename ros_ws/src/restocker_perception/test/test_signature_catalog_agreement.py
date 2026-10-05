# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The perception signatures must describe the products the catalog actually defines."""

import os
from pathlib import Path

import yaml

# Resolved on first use, never at import: pytest's launch-testing plugin imports every sibling
# test module in this directory while collecting any one of them, and a module-level KeyError
# here would fail a collection that never intended to run these tests (the first sibling pytest
# in this package hit exactly that).


def _env_path(name: str) -> Path:
    return Path(os.environ[name])


def _perception_config() -> Path:
    return _env_path("RESTOCKER_TEST_PERCEPTION_CONFIG")


def _tray_perception_config() -> Path:
    return _env_path("RESTOCKER_TEST_TRAY_PERCEPTION_CONFIG")


def _product_catalog() -> Path:
    return _env_path("RESTOCKER_TEST_PRODUCT_CATALOG")


def _scenario() -> Path:
    return _env_path("RESTOCKER_TEST_BASELINE_SCENARIO")


def _load(path: Path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_every_signature_matches_the_catalogued_cylinder() -> None:
    """A resized product must not leave perception fitting the dimensions it used to have."""
    # The signature's nominal radius and height are the model the pose estimator fits: the height
    # turns an observed top face into a centre, and the radius is the gate that separates a product
    # from anything else its colour. Both are restated in the perception config because perception
    # owns its own thresholds, and both are asserted equal here so the restatement cannot drift.
    parameters = _load(_perception_config())["perception"]["ros__parameters"]
    entries = _load(_product_catalog())["geometries"]
    # One signature per product class is all this parameter namespace can express, the keys are
    # `<class>.sku`, `<class>.reference_rgb` and so on, so the entry a signature stands for is
    # the class's fallback, the one a product of that class resolves to when its SKU is unknown.
    # A class with a second SKU has a catalogue entry this backend cannot describe at all; that is
    # asserted separately below rather than papered over here.
    catalog = {entry["product_class"]: entry for entry in entries if entry.get("class_fallback")}

    configured = parameters["product_classes"]
    assert set(configured) == set(catalog), (
        f"perception describes {sorted(configured)} but the catalog defines {sorted(catalog)}"
    )
    for name in configured:
        shape = catalog[name]["shape"]
        assert shape["type"] == "cylinder", f"{name} is not a cylinder this backend can fit"
        assert parameters[f"{name}.nominal_radius_m"] == shape["radius_m"], name
        assert parameters[f"{name}.nominal_height_m"] == shape["height_m"], name
        # Lane policy admits a product by class and SKU together, so a signature naming a SKU the
        # catalog does not have puts every product it finds into a lane that will refuse it.
        assert parameters[f"{name}.sku"] == catalog[name]["sku"], name


def test_a_second_sku_of_one_class_is_the_same_cylinder_this_backend_already_fits() -> None:
    """A sibling SKU that is a different size would be silently fitted at the wrong dimensions."""
    # The signature namespace is per class, so every product of a class is measured against the
    # one signature that class has. That is survivable exactly while the siblings share a shape:
    # the radius gate and the height-to-centre conversion stay correct and only the reported SKU
    # is wrong, which is the current state of this backend and is recorded in the
    # package README. A sibling with a different cylinder would break the pose itself, silently,
    # and this is the assertion that stops one being added without the namespace being fixed
    # first.
    parameters = _load(_perception_config())["perception"]["ros__parameters"]
    entries = _load(_product_catalog())["geometries"]
    for entry in entries:
        name = entry["product_class"]
        if entry.get("class_fallback") or name not in parameters["product_classes"]:
            continue
        assert parameters[f"{name}.nominal_radius_m"] == entry["shape"]["radius_m"], entry
        assert parameters[f"{name}.nominal_height_m"] == entry["shape"]["height_m"], entry


def test_every_stocked_product_has_a_signature_to_be_found_by() -> None:
    """A product the scenario can spawn but perception has no signature for is invisible."""
    # With ground truth as the producer every spawned product was reported whatever it looked like.
    # A camera reports only what it was told to look for, so the scenario's variants and the
    # perception signatures have to cover each other.
    parameters = _load(_perception_config())["perception"]["ros__parameters"]
    scenario = _load(_scenario())
    stocked = {product["product_class"] for product in scenario["products"]}
    stocked |= {variant["product_class"] for variant in scenario["randomization"]["variants"]}
    assert stocked <= set(parameters["product_classes"]), (
        f"the scenario can stock {sorted(stocked)} but perception looks for "
        f"{sorted(parameters['product_classes'])}"
    )


def test_the_reference_colours_stay_ordered_the_way_the_products_are_coloured() -> None:
    """Each signature's dominant channel must be the one its product is actually painted in."""
    # The references are measured render colours rather than the scenario's declared albedo, so
    # they cannot be asserted equal to it. What can be asserted is that the measurement did not
    # drift onto a different product: the channel ordering of a signature and of the albedo it
    # stands for agree.
    parameters = _load(_perception_config())["perception"]["ros__parameters"]
    albedo = {
        product["product_class"]: product["rgba"][:3] for product in _load(_scenario())["products"]
    }
    for name, declared in albedo.items():
        measured = parameters[f"{name}.reference_rgb"]
        assert declared.index(max(declared)) == measured.index(max(measured)), (
            f"{name} is declared {declared} but perception looks for {measured}"
        )


def test_tray_duty_signatures_match_the_catalogued_cylinders() -> None:
    """Tray duty overview and confirm must describe the same products the catalog defines."""
    entries = _load(_product_catalog())["geometries"]
    catalog = {entry["product_class"]: entry for entry in entries if entry.get("class_fallback")}
    tray = _load(_tray_perception_config())
    for duty in ("tray_overview_perception", "tray_confirm_perception"):
        parameters = tray[duty]["ros__parameters"]
        assert set(parameters["product_classes"]) == set(catalog), duty
        for name in parameters["product_classes"]:
            shape = catalog[name]["shape"]
            assert parameters[f"{name}.nominal_radius_m"] == shape["radius_m"], (duty, name)
            assert parameters[f"{name}.nominal_height_m"] == shape["height_m"], (duty, name)
            assert parameters[f"{name}.sku"] == catalog[name]["sku"], (duty, name)
        # These files enforce that the overhead pixel and depth floors do
        # not travel unchanged into either wrist duty.
        assert parameters["minimum_component_pixels"] > 40, duty
        assert parameters["maximum_depth_m"] < 8.0, duty


def test_tray_duties_disagree_on_range_and_pixel_floor() -> None:
    """Confirm is closer and stricter on component size; overview is not a renamed copy of it."""
    tray = _load(_tray_perception_config())
    overview = tray["tray_overview_perception"]["ros__parameters"]
    confirm = tray["tray_confirm_perception"]["ros__parameters"]
    assert confirm["maximum_depth_m"] < overview["maximum_depth_m"]
    assert confirm["minimum_component_pixels"] > overview["minimum_component_pixels"]
    assert overview["backend_name"] == "wrist_rgbd_tray_overview"
    assert confirm["backend_name"] == "wrist_rgbd_tray_confirm"
