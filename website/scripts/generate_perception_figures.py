#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Generate the perception figures of the website from the catalogue textures.

Writes website/static/img/perception/:

  label-can.webp, label-box.webp   the stored label textures (upright, as shipped)
  registry-can.png, registry-box.png  the registry's eight reference views of one product
  catalogue.png                    the front-panel reference view of every catalogue product

The views come from ``restocker_perception.registry.render``, the same offline renderer that
builds the product registry, so they are what the registry holds for these products.

Usage (from the repository root, inside ``nix develop``):

  python3 website/scripts/generate_perception_figures.py
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

BACKGROUND = (244, 246, 248)
INK = (26, 31, 36)


def font(size: int) -> ImageFont.ImageFont:
    for name in ("DejaVuSans.ttf", "LiberationSans-Regular.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def fit(image: Image.Image, width: int, height: int) -> Image.Image:
    scale = min(width / image.width, height / image.height)
    return image.resize(
        (max(1, round(image.width * scale)), max(1, round(image.height * scale))),
        Image.LANCZOS,
    )


def strip(views: list[np.ndarray], labels: list[str], cell_h: int, cell_w: int) -> Image.Image:
    """A row of views, each on a cell with its caption underneath."""
    caption_h = 22
    sheet = Image.new("RGB", (cell_w * len(views), cell_h + caption_h), BACKGROUND)
    draw = ImageDraw.Draw(sheet)
    small = font(13)
    for index, (view, label) in enumerate(zip(views, labels)):
        tile = fit(Image.fromarray(view), cell_w - 8, cell_h - 8)
        x = index * cell_w + (cell_w - tile.width) // 2
        sheet.paste(tile, (x, (cell_h - tile.height) // 2))
        box = draw.textbbox((0, 0), label, font=small)
        draw.text(
            (index * cell_w + (cell_w - (box[2] - box[0])) // 2, cell_h + 2),
            label,
            fill=INK,
            font=small,
        )
    return sheet


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=None)
    args = parser.parse_args()
    repo = args.repo_root.resolve() if args.repo_root else Path(__file__).resolve().parents[2]
    perception = repo / "ros_ws/src/restocker_perception"
    gazebo = repo / "ros_ws/src/restocker_gazebo"
    sys.path.insert(0, str(perception))
    from restocker_perception.registry.render import (  # noqa: E402
        BOX_ATLAS_MAX_WIDTH,
        BoxLayout,
        load_texture,
        profile_from_obj,
        render_box_views,
        render_views,
    )
    from restocker_perception.registry.simulated import (  # noqa: E402
        load_catalogue_rows,
        load_geometry_catalogue,
    )

    rows = {row["sku"]: row for row in load_catalogue_rows(gazebo / "config/generated_product_labels.yaml")}
    geometry = load_geometry_catalogue(
        repo / "ros_ws/src/restocker_description/config/product_collision_catalog.yaml"
    )
    textures = gazebo / "materials/textures"
    meshes = gazebo / "meshes"
    out = repo / "website/static/img/perception"
    out.mkdir(parents=True, exist_ok=True)

    def views_of(sku: str, count: int = 8) -> list[np.ndarray]:
        row = rows[sku]
        entry = geometry[row["geometry_key"]]
        if row.get("shape") == "box":
            texture = load_texture(textures / row["texture"], max_width=BOX_ATLAS_MAX_WIDTH)
            return render_box_views(texture, BoxLayout.from_manifest_row(row), count=count)
        mesh = meshes / f"{row['shape']}.obj"
        profile = profile_from_obj(mesh) if mesh.exists() else None
        texture = load_texture(textures / row["texture"])
        return render_views(texture, entry["radius_m"], entry["height_m"], profile=profile, count=count)

    # Label textures, as stored (upright).
    for sku, name, width in (("GEN-001", "label-can", 900), ("GEN-019", "label-box", 1000)):
        image = Image.open(textures / rows[sku]["texture"]).convert("RGB")
        image.thumbnail((width, width))
        image.save(out / f"{name}.webp", quality=82)

    # The eight reference views of one round and one box product.
    azimuth = [f"{k * 45}°" for k in range(8)]
    strip(views_of("GEN-001"), azimuth, 190, 120).save(out / "registry-can.png", optimize=True)
    faces = ["front", "front-left", "left", "left-back", "back", "back-right", "right", "right-front"]
    strip(views_of("GEN-019"), faces, 190, 120).save(out / "registry-box.png", optimize=True)

    # One view of every catalogue product, rows by shape.
    order = sorted(rows)
    tile_w, tile_h, per_row = 96, 150, 10
    sheet = Image.new(
        "RGB", (tile_w * per_row, (tile_h + 20) * ((len(order) + per_row - 1) // per_row)), BACKGROUND
    )
    draw = ImageDraw.Draw(sheet)
    small = font(12)
    for index, sku in enumerate(order):
        # The front panel faces the camera at yaw 0 for a box and at yaw 90 for a body of
        # revolution (the strip's seam is at yaw 0).
        views = views_of(sku, count=8)
        view = views[0] if rows[sku].get("shape") == "box" else views[2]
        tile = fit(Image.fromarray(view), tile_w - 8, tile_h - 8)
        col, row_index = index % per_row, index // per_row
        x, y = col * tile_w, row_index * (tile_h + 20)
        sheet.paste(tile, (x + (tile_w - tile.width) // 2, y + (tile_h - tile.height) // 2))
        box = draw.textbbox((0, 0), sku, font=small)
        draw.text((x + (tile_w - (box[2] - box[0])) // 2, y + tile_h + 2), sku, fill=INK, font=small)
    sheet.save(out / "catalogue.png", optimize=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
