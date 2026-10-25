# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Label textures are upright on disk; the cylinder half-turn is applied only at install."""

import io
from pathlib import Path

from PIL import Image, ImageChops
import pytest
from restocker_gazebo import label_textures
import yaml

_PACKAGE = Path(__file__).resolve().parents[1]
MANIFEST = _PACKAGE / "config" / "generated_product_labels.yaml"
TEXTURES = _PACKAGE / "materials" / "textures"


def test_manifest_rotation_follows_the_shape():
    rotations = label_textures.load_texture_rotations(MANIFEST)
    products = yaml.safe_load(MANIFEST.read_text())["products"]
    assert len(rotations) == len(products) == 20
    for product in products:
        expected = 0 if product["shape"] in ("contour_bottle", "box") else 180
        assert rotations[product["texture"]] == expected


def test_rotate_png_180_matches_pillow_for_every_filter_type():
    image = Image.effect_noise((37, 23), 60).convert("RGB")
    buffer = io.BytesIO()
    image.save(buffer, format="PNG")
    rotated = Image.open(io.BytesIO(label_textures.rotate_png_180(buffer.getvalue())))
    assert rotated.size == image.size
    assert ImageChops.difference(rotated, image.transpose(Image.ROTATE_180)).getbbox() is None


def test_installed_cylinder_texture_is_the_stored_one_turned(tmp_path):
    source = TEXTURES / "gen-001.png"
    installed = label_textures.install_texture(source, 180, tmp_path)
    assert installed != source and installed.parent == tmp_path
    assert (
        ImageChops.difference(
            Image.open(installed).convert("RGB"),
            Image.open(source).convert("RGB").transpose(Image.ROTATE_180),
        ).getbbox()
        is None
    )
    assert label_textures.install_texture(source, 180, tmp_path) == installed


def test_unrotated_texture_is_installed_in_place(tmp_path):
    source = TEXTURES / "gen-013.png"
    assert label_textures.install_texture(source, 0, tmp_path) == source
    assert not list(tmp_path.iterdir())


def test_unsupported_rotation_is_refused(tmp_path):
    with pytest.raises(RuntimeError):
        label_textures.install_texture(TEXTURES / "gen-001.png", 90, tmp_path)
