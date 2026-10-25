# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Registration tool: add real products from images, simulated ones from textures; inspect."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
from pathlib import Path
import re
import sys

from restocker_perception.registry.backends import (
    EmbeddingBackend,
    FakeEmbeddingBackend,
    SigLip2Backend,
)
from restocker_perception.registry.cache import DerivedCache
from restocker_perception.registry.imageio import read_image
from restocker_perception.registry.model import (  # noqa: I100,I101 - ament and Ruff disagree
    VIEW_COUNT,
    VIEW_STEP_DEG,
    Registry,
    RegistryError,
)
from restocker_perception.registry.retrieval import RegistryIndex
from restocker_perception.registry.simulated import (
    check_registry_against_splits,
    held_out_products,
    load_catalogue_rows,
    load_geometry_catalogue,
    load_split_file,
    register_simulated,
)

_AZIMUTH_NAME = re.compile(r"az(\d{3})")
_IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}
_SOURCE_TREE = Path(__file__).resolve().parents[3]


def _default(package: str, relative: str) -> Path:
    """Installed share path if ament knows the package, else the source tree."""
    try:
        from ament_index_python.packages import get_package_share_directory  # noqa: PLC0415

        return Path(get_package_share_directory(package)) / relative
    except Exception:  # noqa: BLE001 - no ament environment: use the checkout
        return _SOURCE_TREE / package / relative


def ordered_view_files(directory: Path) -> list[Path]:
    """The eight images of ``directory`` in azimuth order.

    Names ``az000`` .. ``az315`` fix the order explicitly; otherwise the sorted file names are
    taken to be the turntable order starting at azimuth 0.
    """
    files = sorted(p for p in directory.iterdir() if p.suffix.lower() in _IMAGE_SUFFIXES)
    if len(files) != VIEW_COUNT:
        raise RegistryError(f"{directory} holds {len(files)} images, need exactly {VIEW_COUNT}")
    tagged = [_AZIMUTH_NAME.search(p.stem) for p in files]
    if all(tagged):
        azimuths = [int(m.group(1)) for m in tagged if m]
        if sorted(azimuths) != [i * VIEW_STEP_DEG for i in range(VIEW_COUNT)]:
            raise RegistryError(f"{directory}: azNNN names must be 000, 045, ..., 315")
        return [p for _a, p in sorted(zip(azimuths, files, strict=True))]
    return files


def make_backend(name: str, revision: str) -> EmbeddingBackend:
    if name == "fake":
        return FakeEmbeddingBackend()
    if name == "siglip2":
        return SigLip2Backend(revision=revision)
    raise SystemExit(f"unknown backend {name!r}")


def _cmd_add_images(args: argparse.Namespace) -> int:
    registry = Registry.open_or_create(args.registry)
    images = [read_image(path) for path in ordered_view_files(Path(args.images))]
    record = registry.register(
        gtin=args.gtin,
        product_id=args.product_id,
        display_name=args.name,
        product_class=args.product_class,
        geometry_key=args.geometry_key,
        radius_m=args.radius,
        height_m=args.height,
        images=images,
        ocr_strings=args.ocr or (),
        aliases=args.alias or (),
        source={"kind": "images", "directory_name": Path(args.images).name},
    )
    print(
        f"registered {record.product_id} {record.display_name!r}; "
        f"registry {registry.content_hash()}"
    )
    return 0


def _cmd_add_sim(args: argparse.Namespace) -> int:
    registry = Registry.open_or_create(args.registry)
    exclude: set[str] = set()
    notes = {}
    if args.exclude_held_out:
        splits = load_split_file(args.exclude_held_out)
        exclude = set(held_out_products(splits))
        notes = {"excluded_held_out": sorted(exclude), "splits_sha256": splits["sha256"]}
    added = register_simulated(
        registry,
        load_catalogue_rows(args.catalogue),
        load_geometry_catalogue(args.geometry),
        args.textures,
        mesh_dir=args.mesh_dir if args.use_mesh else None,
        only=set(args.sku) if args.sku else None,
        exclude=exclude,
    )
    if notes:
        registry.set_notes(**notes)
        leaked = check_registry_against_splits(registry, load_split_file(args.exclude_held_out))
        if leaked:
            raise RegistryError("; ".join(leaked))
    print(f"registered {len(added)} products; registry {registry.content_hash()}")
    return 0


def _cmd_verify(args: argparse.Namespace) -> int:
    registry = Registry.load(args.registry, verify=False)
    manifest = json.loads((Path(args.registry) / "registry.json").read_text(encoding="utf-8"))
    manifest_hash = manifest["content_sha256"]
    problems = registry.verify(expected_hash=manifest_hash)
    if args.splits:
        problems += check_registry_against_splits(registry, load_split_file(args.splits))
    for problem in problems:
        print(f"PROBLEM: {problem}")
    print(f"{len(registry)} products; content {registry.content_hash()}")
    return 1 if problems else 0


def _cmd_info(args: argparse.Namespace) -> int:
    registry = Registry.load(args.registry)
    print(f"content_sha256 {registry.content_hash()}")
    for record in registry.records():
        gtin = record.gtin or "-"
        print(
            f"{record.product_id}  gtin {gtin:<13}  {record.product_class:<14} "
            f"{record.display_name}  {record.aliases}"
        )
    return 0


def _cmd_embed(args: argparse.Namespace) -> int:
    registry = Registry.load(args.registry)
    backend = make_backend(args.backend, args.revision)
    cache = DerivedCache(args.cache or Path(args.registry) / ".cache")
    for product_id in registry.product_ids:
        cache.embeddings(registry, backend, product_id)
    print(f"{backend.name} {backend.version}: {cache.misses} computed, {cache.hits} reused")
    return 0


def _cmd_query(args: argparse.Namespace) -> int:
    registry = Registry.load(args.registry)
    backend = make_backend(args.backend, args.revision)
    cache = DerivedCache(args.cache or Path(args.registry) / ".cache")
    index = RegistryIndex.from_registry(registry, backend, cache)
    result = index.identify([read_image(p) for p in args.image], backend, scoring=args.scoring)
    for match in result.top_k(args.top_k):
        print(f"{match.product_id}  {match.score:.4f}  best view {match.best_view_deg}")
    print(f"margin {result.margin}  vote agreement {result.vote_agreement:.2f}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    add = sub.add_parser("add-images", help="register a real product from eight photographs")
    add.add_argument("--registry", required=True)
    add.add_argument("--images", required=True, help="directory of eight views, 45 degrees apart")
    add.add_argument(
        "--gtin",
        help="EAN-13, check digit verified; it is then also the product id. Omit for a product "
        "with no barcode",
    )
    add.add_argument(
        "--product-id",
        help="id for a product with no GTIN (default: derived from name and views, prod-<hex>)",
    )
    add.add_argument("--name", required=True)
    add.add_argument("--product-class", required=True)
    add.add_argument("--geometry-key", required=True)
    add.add_argument("--radius", type=float, required=True, help="metres")
    add.add_argument("--height", type=float, required=True, help="metres")
    add.add_argument("--ocr", action="append", help="printed string (repeatable)")
    add.add_argument("--alias", action="append", help="other id for the product (repeatable)")
    add.set_defaults(func=_cmd_add_images)

    sim = sub.add_parser("add-sim", help="render and register simulated products")
    sim.add_argument("--registry", required=True)
    sim.add_argument(
        "--catalogue",
        default=_default("restocker_gazebo", "config/generated_product_labels.yaml"),
    )
    sim.add_argument(
        "--geometry",
        default=_default("restocker_description", "config/product_collision_catalog.yaml"),
    )
    sim.add_argument("--textures", default=_default("restocker_gazebo", "materials/textures"))
    sim.add_argument("--mesh-dir", default=_default("restocker_gazebo", "meshes"))
    sim.add_argument("--use-mesh", action="store_true", help="render bottles from their mesh")
    sim.add_argument("--sku", action="append", help="only this catalogue SKU (repeatable)")
    sim.add_argument(
        "--exclude-held-out",
        metavar="SPLITS_JSON",
        help="frozen benchmark split file; held-out products are not registered",
    )
    sim.set_defaults(func=_cmd_add_sim)

    verify = sub.add_parser("verify", help="recheck every hash; optionally against a split file")
    verify.add_argument("--registry", required=True)
    verify.add_argument("--splits", help="fail if a held-out product is present")
    verify.set_defaults(func=_cmd_verify)

    info = sub.add_parser("info", help="list products and the content hash")
    info.add_argument("--registry", required=True)
    info.set_defaults(func=_cmd_info)

    for name, func, helptext in (
        ("embed", _cmd_embed, "fill the embedding cache for a backend"),
        ("query", _cmd_query, "rank the registry against query images"),
    ):
        node = sub.add_parser(name, help=helptext)
        node.add_argument("--registry", required=True)
        node.add_argument("--backend", choices=("fake", "siglip2"), default="fake")
        node.add_argument("--revision", default="main", help="siglip2 model revision to pin")
        node.add_argument("--cache", help="cache directory (default: <registry>/.cache)")
        node.set_defaults(func=func)
        if name == "query":
            node.add_argument("image", nargs="+", help="crops of the same object (voted)")
            node.add_argument("--scoring", choices=("max", "mean", "top2"), default="max")
            node.add_argument("--top-k", type=int, default=3)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (RegistryError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
