# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A bundled world holds every model from its first frame, and converts each description once."""

import xml.etree.ElementTree as ET

import pytest
from restocker_gazebo.world_bundle import (  # noqa: I100,I101
    BundledModel,
    bundle_models,
    load_materials,
)

WORLD = (
    '<?xml version="1.0"?>\n<sdf version="1.9">\n  <world name="restocking">\n'
    '    <light name="key" type="directional"/>\n  </world>\n</sdf>\n'
)
CONVERTED = (
    "<sdf version='1.9'><model name='restocker_product'><pose>9 9 9 0 0 0</pose>"
    "<link name='product_body'/></model></sdf>"
)


def _models(world: str) -> dict[str, ET.Element]:
    return {model.get("name"): model for model in ET.fromstring(world).iter("model")}


def test_every_model_is_in_the_world_with_its_name_and_pose():
    calls = []

    def convert(path, arguments):
        calls.append((path, dict(arguments)))
        return CONVERTED

    models = [
        BundledModel("product.xacro", {"static": "true"}, "lane_01_can_01", [1, 2, 3, 0, 0, 0.5]),
        BundledModel("product.xacro", {"static": "true"}, "lane_01_can_02", [1, 2.1, 3, 0, 0, 0]),
        BundledModel(
            "product.xacro", {"static": "false"}, "stock_can_01", [0, -0.68, 0.9, 0, 0, 3]
        ),
    ]
    bundled = bundle_models(WORLD, models, convert)
    found = _models(bundled)
    assert set(found) == {"lane_01_can_01", "lane_01_can_02", "stock_can_01"}
    assert found["lane_01_can_01"].find("pose").text.split() == [
        "1.0",
        "2.0",
        "3.0",
        "0.0",
        "0.0",
        "0.5",
    ]
    assert len(found["stock_can_01"].findall("pose")) == 1
    assert ET.fromstring(bundled).find("world/light") is not None


def test_equal_descriptions_are_converted_once():
    calls = []

    def convert(path, arguments):
        calls.append(dict(arguments))
        return CONVERTED

    models = [
        BundledModel("p.xacro", {"static": "true", "rgba": "1 1 1 1"}, f"m{i}", [0, 0, 0, 0, 0, 0])
        for i in range(5)
    ] + [BundledModel("p.xacro", {"static": "false"}, "other", [0, 0, 0, 0, 0, 0])]
    bundle_models(WORLD, models, convert)
    assert len(calls) == 2


def test_a_world_without_a_closing_tag_is_refused():
    with pytest.raises(ValueError):
        bundle_models("<sdf/>", [], lambda *_: CONVERTED)


def test_a_materials_file_restyles_matching_visuals_of_the_named_model_only():
    converted = (
        "<sdf><model name='m'><link name='l'>"
        "<visual name='a_divider_visual'><material><diffuse>1 1 1 1</diffuse></material></visual>"
        "<visual name='a_tray_visual'><material><diffuse>1 1 1 1</diffuse></material></visual>"
        "</link></model></sdf>"
    )
    rules = {"shelf": [{"match": "divider", "diffuse": [0.1, 0.2, 0.3, 0.4], "roughness": 0.5}]}
    models = [
        BundledModel("x", {}, "shelf", [0, 0, 0, 0, 0, 0]),
        BundledModel("x", {}, "other", [0, 0, 0, 0, 0, 0]),
    ]
    found = _models(bundle_models(WORLD, models, lambda *_: converted, rules))
    styled = {v.get("name"): v for v in found["shelf"].iter("visual")}
    assert styled["a_divider_visual"].find("material/diffuse").text == "0.1 0.2 0.3 0.4"
    assert styled["a_divider_visual"].find("material/pbr/metal/roughness").text == "0.5"
    assert styled["a_tray_visual"].find("material/diffuse").text == "1 1 1 1"
    untouched = {v.get("name"): v for v in found["other"].iter("visual")}
    assert untouched["a_divider_visual"].find("material/diffuse").text == "1 1 1 1"


def test_the_shipped_showcase_materials_cover_every_workcell_visual_class():
    import os
    from pathlib import Path

    root = os.environ.get("RESTOCKER_GAZEBO_SOURCE_DIR")
    if not root:
        pytest.skip("RESTOCKER_GAZEBO_SOURCE_DIR is not set")
    rules = load_materials(str(Path(root) / "config" / "showcase_materials.yaml"))["workcell"]
    for visual in (
        "shelf_fixed_joint_lump__lane_03_04_divider_visual_visual_3",
        "shelf_fixed_joint_lump__left_frame_visual_visual",
        "shelf_fixed_joint_lump__front_retainer_visual_visual_7",
        "shelf_fixed_joint_lump__stock_tray_visual_visual_8",
        "shelf_fixed_joint_lump__camera_visual_visual_9",
        "shelf_fixed_joint_lump__roller_bed_visual_visual_10",
        "shelf_fixed_joint_lump__roller_07_visual_visual_18",
    ):
        assert any(rule["match"] in visual for rule in rules), visual
