# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""identity_benchmark: calibrate on train/validation seeds, evaluate once on the held-out seeds."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path
import sys

from restocker_perception.identity import benchmark
from restocker_perception.identity.barcode import ZxingBarcodeBackend
from restocker_perception.identity.config import IdentityConfig
from restocker_perception.registry.backends import (
    EmbeddingBackend,
    FakeEmbeddingBackend,
    OcrBackend,
    RapidOcrBackend,
    SigLip2Backend,
)
from restocker_perception.registry.model import Registry, RegistryError
from restocker_perception.registry.simulated import load_geometry_catalogue

_SOURCE_TREE = Path(__file__).resolve().parents[3]
_DEFAULT_SPLITS = _SOURCE_TREE / "restocker_benchmarks/config/perception_splits.json"
_DEFAULT_GEOMETRY = _SOURCE_TREE / "restocker_description/config/product_collision_catalog.yaml"
_DEFAULT_BOUNDS = _SOURCE_TREE / "restocker_perception/config/identity_acceptance_bounds_r2.json"


def make_backends(
    args: argparse.Namespace,
) -> tuple[EmbeddingBackend, OcrBackend | None, ZxingBarcodeBackend | None]:
    if args.backend == "fake":
        backend: EmbeddingBackend = FakeEmbeddingBackend()
    else:
        backend = SigLip2Backend(revision=args.revision, device=args.device)
    ocr = RapidOcrBackend(version=args.ocr_version) if args.ocr == "rapidocr" else None
    barcode = ZxingBarcodeBackend(version=args.zxing_version) if args.barcode == "zxing" else None
    return backend, ocr, barcode


def _common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--registry", required=True, type=Path)
    parser.add_argument("--dataset", required=True, action="append", type=Path)
    parser.add_argument("--splits", type=Path, default=_DEFAULT_SPLITS)
    parser.add_argument("--geometry", type=Path, default=_DEFAULT_GEOMETRY)
    parser.add_argument("--bounds", type=Path, default=_DEFAULT_BOUNDS)
    parser.add_argument("--backend", choices=("fake", "siglip2"), default="siglip2")
    parser.add_argument("--revision", default="main", help="siglip2 model revision (pin it)")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--ocr", choices=("rapidocr", "none"), default="rapidocr")
    parser.add_argument("--ocr-version", default="unpinned", help="installed rapidocr version")
    parser.add_argument("--barcode", choices=("zxing", "none"), default="zxing")
    parser.add_argument("--zxing-version", default="unpinned", help="installed zxing-cpp version")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    cal = sub.add_parser("calibrate", help="fit thresholds on train/validation seeds only")
    _common(cal)
    cal.add_argument("--out", required=True, type=Path, help="calibration JSON")
    cal.add_argument("--whitener-out", required=True, type=Path, help="whitening npz")
    cal.add_argument(
        "--ocr-rotations",
        default="0,45",
        help="comma-separated degrees tried per crop (fixed a priori, not searched)",
    )
    ev = sub.add_parser("evaluate", help="the one held-out run, with the calibration as written")
    _common(ev)
    ev.add_argument("--calibration", required=True, type=Path)
    ev.add_argument("--whitener", required=True, type=Path)
    ev.add_argument("--out", required=True, type=Path, help="result JSON (must not exist)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        registry = Registry.load(args.registry)
        backend, ocr, barcode = make_backends(args)
        geometry = load_geometry_catalogue(args.geometry)
        if args.command == "calibrate":
            benchmark.calibrate(
                args.dataset,
                registry,
                backend,
                ocr,
                args.splits,
                geometry,
                out=args.out,
                whitener_out=args.whitener_out,
                bounds_path=args.bounds,
                base=IdentityConfig(
                    ocr_rotations=tuple(int(d) for d in args.ocr_rotations.split(","))
                ),
                barcode_backend=barcode,
            )
        else:
            benchmark.evaluate_held_out(
                args.dataset,
                registry,
                backend,
                ocr,
                args.splits,
                geometry,
                calibration_path=args.calibration,
                whitener_path=args.whitener,
                bounds_path=args.bounds,
                out=args.out,
                barcode_backend=barcode,
            )
    except (RegistryError, benchmark.ProtocolError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
