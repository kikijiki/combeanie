# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The generated label catalogue: valid barcodes, stable output, catalogue-matching shapes."""

import importlib.util
from pathlib import Path
import sys

import pytest
import yaml

_ROOT = Path(__file__).resolve().parents[4]
_spec = importlib.util.spec_from_file_location(
    "label_factory", _ROOT / "tools" / "label_factory.py"
)
factory = importlib.util.module_from_spec(_spec)
sys.modules["label_factory"] = factory
_spec.loader.exec_module(factory)


def test_known_real_ean13_validates():
    # 4006381333931 is the standard worked example for the EAN-13 check digit.
    assert factory.ean13_check_digit("400638133393") == 1
    assert len(factory.ean13_modules("4006381333931")) == 95


def test_bad_check_digit_is_refused():
    with pytest.raises(ValueError):
        factory.ean13_modules("4006381333932")


def test_modules_have_guards_and_decode_round_trip():
    code = "2000000000015"
    modules = factory.ean13_modules(code)
    assert modules.startswith("101") and modules.endswith("101") and modules[45:50] == "01010"
    # Decode the right half with the R table.
    right = [modules[50 + 7 * i : 57 + 7 * i] for i in range(6)]
    assert "".join(str(factory._R.index(group)) for group in right) == code[7:]


def test_catalogue_is_unique_and_gtins_are_in_restricted_range():
    specs = factory.build_catalogue()
    assert len({s.sku for s in specs}) == len(specs) >= 12
    # A SKU with no barcode panel has no GTIN at all (never an empty or made-up one).
    gtins = [g for g in (factory.gtin_for(s) for s in specs) if g is not None]
    assert len(gtins) == len(specs) - 1
    assert len(set(gtins)) == len(gtins)
    assert all(
        g.startswith("2") and factory.ean13_check_digit(g[:12]) == int(g[12]) for g in gtins
    )


def test_lookalike_pairs_share_layout_and_differ_in_flavour():
    specs = factory.build_catalogue()
    groups = {}
    for spec in specs:
        groups.setdefault(spec.pair, []).append(spec)
    # Boxes add one single-member family each for the 1 L carton and the slim box.
    assert all(len(group) == 2 or key in ("box-1", "box-2") for key, group in groups.items())
    assert len(groups["box-0"]) == 2
    for first, second in (group for group in groups.values() if len(group) == 2):
        assert (first.brand, first.kind, first.layout_seed) == (
            second.brand,
            second.kind,
            second.layout_seed,
        )
        assert first.flavour != second.flavour


def test_shapes_match_the_collision_catalogue():
    catalog = yaml.safe_load(
        (
            _ROOT / "ros_ws/src/restocker_description/config/product_collision_catalog.yaml"
        ).read_text()
    )
    by_key = {g["geometry_key"]: g["shape"] for g in catalog["geometries"]}
    for spec in factory.build_catalogue():
        shape = by_key[spec.geometry_key]
        if spec.is_box:
            assert shape == {
                "type": "box",
                "size_xyz_m": list(factory.BOX_VARIANTS[spec.variant]["size"]),
            }
            continue
        assert shape["radius_m"] == factory.SHAPES[spec.shape]["radius"]
        assert shape["height_m"] == factory.SHAPES[spec.shape]["height"]


def test_checked_in_output_is_current():
    assert factory.main(["--check"]) == 0


def test_manifest_lists_every_texture_on_disk():
    manifest = yaml.safe_load(factory.MANIFEST.read_text())
    for product in manifest["products"]:
        assert (factory.TEXTURE_DIR / Path(product["texture"]).name).is_file()


def test_catalogue_scenario_stocks_every_generated_product_once():
    spec = importlib.util.spec_from_file_location(
        "generate_catalogue_tray_scenario", _ROOT / "tools" / "generate_catalogue_tray_scenario.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["generate_catalogue_tray_scenario"] = module
    spec.loader.exec_module(module)
    assert module.main(["--check"]) == 0
    scenario = yaml.safe_load(module.OUT.read_text())
    skus = [product["sku"] for product in scenario["products"]]
    assert sorted(skus) == sorted(s.sku for s in factory.build_catalogue())
    for product in scenario["products"]:
        assert product["rgba"] == [1.0, 1.0, 1.0, 1.0]
        assert (factory.TEXTURE_DIR / product["label_texture"]).is_file()


def _box_specs():
    return [s for s in factory.build_catalogue() if s.is_box]


def test_there_are_look_alike_boxes_and_a_box_without_a_barcode_panel():
    boxes = _box_specs()
    assert len(boxes) == 4
    pair = [s for s in boxes if s.pair == "box-0"]
    assert len(pair) == 2 and pair[0].flavour != pair[1].flavour
    assert pair[0].layout_seed == pair[1].layout_seed and pair[0].variant == pair[1].variant
    no_code = [s for s in boxes if not s.barcode]
    assert len(no_code) == 1 and factory.gtin_for(no_code[0]) is None
    manifest = yaml.safe_load(factory.MANIFEST.read_text())["products"]
    row = next(r for r in manifest if r["sku"] == no_code[0].sku)
    assert "gtin" not in row and row["barcode"] is False
    assert all("gtin" in r for r in manifest if r["sku"] != no_code[0].sku)


def test_box_face_layout_tiles_the_atlas_without_overlap_and_every_face_is_right_handed():
    import numpy as np

    for variant, entry in factory.BOX_VARIANTS.items():
        layout = factory.box_layout(entry["size"])
        width, height = layout["atlas"]
        canvas = np.zeros((height, width), np.int32)
        for name in factory.BOX_FACES:
            x, y, w, h = layout["rects"][name]
            canvas[y : y + h, x : x + w] += 1
        assert canvas.max() == 1, variant  # no overlap
        for name, (normal, right, up) in factory.BOX_FACE_FRAMES.items():
            assert tuple(np.cross(right, up)) == normal, (variant, name)  # never mirrored
        # One pixel density: a face's pixel size over its metric size is the same everywhere.
        sx, sy, sz = entry["size"]
        ppm = factory.BOX_PIXELS_PER_METRE
        assert layout["rects"]["front"][2:] == (round(sx * ppm), round(sz * ppm))
        assert layout["rects"]["right"][2:] == (round(sy * ppm), round(sz * ppm))
        assert layout["rects"]["top"][2:] == (round(sx * ppm), round(sy * ppm))


def test_box_meshes_match_the_layout_and_are_current():
    spec = importlib.util.spec_from_file_location(
        "generate_box_meshes", _ROOT / "tools" / "generate_box_meshes.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules["generate_box_meshes"] = module
    spec.loader.exec_module(module)
    assert module.main(["--check"]) == 0
    text = (module.MESH_DIR / factory.box_mesh_name("carton_200")).read_text()
    lines = text.splitlines()
    assert sum(line.startswith("v ") for line in lines) == 24  # four per face, hard edges
    assert sum(line.startswith("f ") for line in lines) == 12
    uvs = [tuple(map(float, line.split()[1:])) for line in lines if line.startswith("vt ")]
    assert all(0.0 <= u <= 1.0 and 0.0 <= v <= 1.0 for u, v in uvs)


def test_box_front_face_is_drawn_with_the_brand_and_the_atlas_has_the_declared_size():
    from PIL import Image

    spec = next(s for s in _box_specs() if s.barcode)
    with Image.open(factory.TEXTURE_DIR / spec.texture) as image:
        assert list(image.size) == list(factory.box_faces(spec)["atlas"])
