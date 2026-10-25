# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Registering simulated products from the generated catalogue and its textures."""
# The catalogue (generated_product_labels.yaml) names each product, its EAN-13 and its texture;
# the geometry catalogue (product_collision_catalog.yaml) gives radius and height. The views are
# rendered from the texture, so a registry built here can be rebuilt byte-for-byte from the
# repository. Held-out products are excluded by the benchmark's frozen split file, and the
# exclusion is recorded in the registry's notes.

from __future__ import annotations

from collections.abc import Collection
import hashlib
import json
from pathlib import Path
from typing import Any

import yaml

from restocker_perception.registry.model import (  # noqa: I100,I101 - ament and Ruff disagree
    Registry,
    RegistryError,
)
from restocker_perception.registry.render import (
    BOX_ATLAS_MAX_WIDTH,
    BoxLayout,
    load_texture,
    profile_from_obj,
    render_box_views,
    render_views,
)

# Mirrors restocker_benchmarks.perception_splits.body_hash/canonical_json, restated so this
# package does not import the benchmark package (which pulls in the Gazebo scenario code).
_SPLIT_NAMES = ("train", "validation", "held_out")


def _split_body_hash(document: dict[str, Any]) -> str:
    body = {k: v for k, v in document.items() if k != "sha256"}
    text = json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def load_split_file(path: str | Path) -> dict[str, Any]:
    """Read the frozen benchmark split file and refuse one whose embedded hash is wrong."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if document.get("sha256") != _split_body_hash(document):
        raise RegistryError(f"{path}: embedded sha256 does not match (file was edited)")
    products = document.get("products", {})
    if set(products) != set(_SPLIT_NAMES):
        raise RegistryError(f"{path}: products must have exactly {_SPLIT_NAMES}")
    return document


def held_out_products(split_document: dict[str, Any]) -> list[str]:
    """Product ids (``GEN-005``) a benchmark registry must never contain."""
    return sorted(split_document["products"]["held_out"])


def check_registry_against_splits(registry: Registry, split_document: dict[str, Any]) -> list[str]:
    """Return one problem per held-out product (by id, GTIN or alias) present in ``registry``."""
    held = set(held_out_products(split_document))
    problems = []
    for record in registry.records():
        names = {record.product_id, *({record.gtin} - {None}), *record.aliases}
        for name in sorted(names & held):
            problems.append(f"registry contains held-out product {name} ({record.product_id})")
    return problems


def load_catalogue_rows(path: str | Path) -> list[dict[str, Any]]:
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    return [dict(row) for row in document["products"]]


def load_geometry_catalogue(path: str | Path) -> dict[str, dict[str, Any]]:
    """geometry_key -> {product_class, radius_m, height_m} from product_collision_catalog.yaml."""
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    out = {}
    for entry in document["geometries"]:
        shape = entry["shape"]
        if shape["type"] == "box":
            # A box has no radius: the registry's radius_m is the footprint's bounding radius.
            size = tuple(float(v) for v in shape["size_xyz_m"])
            out[entry["geometry_key"]] = {
                "product_class": entry["product_class"],
                "radius_m": 0.5 * (size[0] ** 2 + size[1] ** 2) ** 0.5,
                "height_m": size[2],
                "size_xyz_m": size,
            }
            continue
        out[entry["geometry_key"]] = {
            "product_class": entry["product_class"],
            "radius_m": float(shape["radius_m"]),
            "height_m": float(shape["height_m"]),
        }
    return out


def register_simulated(
    registry: Registry,
    rows: list[dict[str, Any]],
    geometry: dict[str, dict[str, Any]],
    texture_dir: str | Path,
    *,
    mesh_dir: str | Path | None = None,
    only: Collection[str] | None = None,
    exclude: Collection[str] = (),
) -> list[str]:
    """Render and register ``rows``; returns the product ids added.

    ``exclude`` takes catalogue SKUs (``GEN-005``) to leave out; ``only`` restricts to some SKUs.
    A row whose ``shape`` has a ``<shape>.obj`` in ``mesh_dir`` is rendered from that mesh's
    profile (the contoured bottle), otherwise from its catalogue cylinder.
    """
    added = []
    for row in sorted(rows, key=lambda r: r["sku"]):
        sku = row["sku"]
        if sku in exclude or (only is not None and sku not in only):
            continue
        entry = geometry.get(row["geometry_key"])
        if entry is None:
            raise RegistryError(
                f"{sku}: geometry_key {row['geometry_key']} is not in the catalogue"
            )
        texture_path = Path(texture_dir) / row["texture"]
        texture_bytes = texture_path.read_bytes()
        profile = None
        mesh_sha = None
        if mesh_dir is not None:
            mesh_path = Path(mesh_dir) / f"{row['shape']}.obj"
            if mesh_path.exists():
                profile = profile_from_obj(mesh_path)
                mesh_sha = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
        box_layout = BoxLayout.from_manifest_row(row) if row.get("shape") == "box" else None
        if box_layout is not None:
            # A box atlas holds one upright panel per face; every view samples the face it shows.
            texture = load_texture(texture_path, max_width=BOX_ATLAS_MAX_WIDTH)
            images = render_box_views(texture, box_layout)
        else:
            texture = load_texture(texture_path)
            # The shipped strip is upright and render_views maps u around the barrel and v down
            # it, so the file is rendered as stored for every shape. Gazebo's half-turn for
            # cylinder primitives is applied when the texture is installed for the simulator
            # (restocker_gazebo.label_textures), not here.
            images = render_views(texture, entry["radius_m"], entry["height_m"], profile=profile)
        record = registry.register(
            gtin=row.get("gtin"),  # absent: a product with no barcode; "" is rejected
            display_name=f"{row['brand']} {row['flavour']} {row['kind']}",
            product_class=entry["product_class"],
            geometry_key=row["geometry_key"],
            radius_m=entry["radius_m"],
            height_m=entry["height_m"],
            size_xyz_m=box_layout.size_xyz_m if box_layout is not None else None,
            images=images,
            ocr_strings=[row["brand"], row["flavour"], row["kind"]],
            aliases=[sku],
            source={
                "kind": "simulated",
                "texture": row["texture"],
                "texture_sha256": hashlib.sha256(texture_bytes).hexdigest(),
                "mesh_sha256": mesh_sha,
            },
        )
        added.append(record.product_id)
    return added
