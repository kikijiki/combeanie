# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""ladder_benchmark: whitener, step (one oracle round), report (numbers and verdict)."""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

from restocker_perception.identity import ladder_benchmark as lb
from restocker_perception.identity.barcode import ZxingBarcodeBackend
from restocker_perception.identity.benchmark import (
    ProtocolError,
    load_whitener,
    object_groups,
    save_whitener,
)
from restocker_perception.identity.ladder import LadderConfig, LadderEngine, build_index
from restocker_perception.registry.backends import RapidOcrBackend, SigLip2Backend
from restocker_perception.registry.model import Registry, RegistryError
from restocker_perception.registry.retrieval import Whitener
from restocker_perception.registry.simulated import load_geometry_catalogue

_SOURCE_TREE = Path(__file__).resolve().parents[3]
_DEFAULT_GEOMETRY = _SOURCE_TREE / "restocker_description/config/product_collision_catalog.yaml"
# The reader's thresholds are lowered from RapidOCR's defaults (text_score 0.5, box_thresh 0.5,
# unclip 1.6, thresh 0.3): the label text of a crop is a handful of pixels high.
OCR_OVERRIDES = {"text_score": 0.3, "box_thresh": 0.3, "unclip_ratio": 2.0, "thresh": 0.2}


def load_config(path: str | None) -> LadderConfig:
    if not path:
        return LadderConfig()
    return LadderConfig.from_dict(json.loads(Path(path).read_text(encoding="utf-8")))


def make_engine(args: argparse.Namespace, whitener: Whitener | None) -> LadderEngine:
    registry = Registry.load(args.registry)
    embedding = SigLip2Backend(revision=args.revision, device=args.device)
    ocr = RapidOcrBackend(args.ocr_version, overrides=OCR_OVERRIDES, threads=args.threads)
    barcode = ZxingBarcodeBackend(version=args.zxing_version)
    index = build_index(registry, embedding, whitener)
    return LadderEngine(
        registry, index, embedding, ocr, load_config(args.config), barcode_backend=barcode
    )


def _common(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--registry", required=True, type=Path)
    parser.add_argument("--geometry", type=Path, default=_DEFAULT_GEOMETRY)
    parser.add_argument("--revision", default="main", help="siglip2 revision (pin it)")
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--ocr-version", default="unpinned")
    parser.add_argument("--zxing-version", default="unpinned")
    parser.add_argument("--threads", type=int, default=3, help="ONNX threads of the OCR")
    parser.add_argument("--config", help="ladder config JSON (default: LadderConfig())")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    w = sub.add_parser("whitener", help="fit the whitening on train/validation crops, unlabeled")
    _common(w)
    w.add_argument("--dataset", required=True, action="append", type=Path)
    w.add_argument("--out", required=True, type=Path)
    s = sub.add_parser("step", help="one oracle round: look, decide, write the next turns")
    _common(s)
    s.add_argument("--whitener", required=True, type=Path)
    s.add_argument("--state", required=True, type=Path, help="pickle; created at round 0")
    s.add_argument("--round", required=True, type=int)
    s.add_argument("--dataset", required=True, action="append", type=Path)
    s.add_argument("--allow", default="train,validation", help="seed splits this run may read")
    s.add_argument("--splits", type=Path)
    s.add_argument("--offsets-out", required=True, type=Path)
    s.add_argument("--blind", action="store_true", help="ablation: fixed quarter turns")
    s.add_argument("--bounds", type=Path, help="required to read held_out")
    s.add_argument("--bounds-sha256", help="sha256 of the committed bounds file (held_out)")
    r = sub.add_parser("report", help="numbers, and the verdict against a bounds file")
    _common(r)
    r.add_argument("--state", required=True, action="append", type=Path, help="repeatable: batches")
    r.add_argument("--budget", type=int, default=3)
    r.add_argument("--out", required=True, type=Path)
    r.add_argument("--bounds", type=Path)
    r.add_argument("--splits", type=Path)
    r.add_argument("--fresh-only", action="store_true", help="count fresh held-out seeds only")
    return parser


def _whitener_command(args: argparse.Namespace) -> int:
    geometry = load_geometry_catalogue(args.geometry)
    registry = Registry.load(args.registry)
    embedding = SigLip2Backend(revision=args.revision, device=args.device)
    from restocker_perception.registry.cache import DerivedCache  # noqa: PLC0415

    cache = DerivedCache(registry.root / ".cache")
    exemplars = np.concatenate(
        [cache.embeddings(registry, embedding, p) for p in registry.product_ids]
    )
    crops = []
    for directory in args.dataset:
        for group in object_groups(directory, geometry, allowed_splits=("train", "validation")):
            crops += [v.image for v in group.views]
    vectors = embedding.embed(crops)
    pool = np.concatenate([exemplars, vectors])
    whitener = Whitener.fit(pool, min(64, len(pool) - 1))
    save_whitener(args.out, whitener)
    print(f"whitener {whitener.fingerprint}: {len(exemplars)} exemplars + {len(vectors)} crops")
    return 0


def _step_command(args: argparse.Namespace) -> int:
    allowed = tuple(args.allow.split(","))
    if "held_out" in allowed:
        if not (args.bounds and args.bounds_sha256 and args.splits):
            raise ProtocolError("held_out needs --bounds, --bounds-sha256 and --splits")
        if lb.file_sha256(args.bounds) != args.bounds_sha256:
            raise ProtocolError("bounds file differs from the committed one")
    whitener = load_whitener(args.whitener)
    engine = make_engine(args, whitener)
    geometry = load_geometry_catalogue(args.geometry)
    if args.round == 0:
        if args.state.exists():
            raise ProtocolError(f"{args.state} exists: round 0 creates the state")
        run = lb.LadderRun(engine.registry)
        run.meta = {
            "whitener_fingerprint": whitener.fingerprint,
            "config": engine.config.to_dict(),
            "allow": list(allowed),
            "bounds_sha256": args.bounds_sha256,
            "guided": not args.blind,
        }
    else:
        run = lb.LadderRun.load(args.state)
        if run.registry_sha256 != engine.registry.content_hash():
            raise ProtocolError("registry differs from the one the state was started with")
        if run.meta["allow"] != list(allowed) or run.meta["guided"] != (not args.blind):
            raise ProtocolError("splits or guidance mode differ from round 0")
    only = None
    if args.round > 0:
        only = {k for k, e in run.objects.items() if not e["closed"]}
    inputs = lb.round_inputs(args.dataset, geometry, allowed_splits=allowed, only=only)
    if args.splits:
        _check_seeds(args.splits, {k[0] for k in inputs}, allowed)
    offsets = lb.step(run, engine, inputs, round_index=args.round, guided=not args.blind)
    run.save(args.state)
    args.offsets_out.write_text(json.dumps(offsets, indent=1, sort_keys=True) + "\n")
    open_n = sum(len(v) for v in offsets.values())
    print(
        f"round {args.round}: {len(inputs)} objects looked at, "
        f"{open_n} still open in {len(offsets)} scenes"
    )
    return 0


def _check_seeds(splits_path: Path, seeds: set[int], allowed: Sequence[str]) -> None:
    from restocker_benchmarks import perception_splits as sp  # noqa: PLC0415

    document = sp.load_splits(splits_path)
    for seed in sorted(seeds):
        split = sp.split_of_seed(document, seed)
        if split not in allowed:
            raise ProtocolError(f"seed {seed} is {split!r}, this run may read {list(allowed)}")
        if split == "held_out" and sp.held_out_generation(document, seed) != "fresh":
            raise ProtocolError(f"held-out seed {seed} is burned: it cannot decide a bound")


def _report_command(args: argparse.Namespace) -> int:
    registry = Registry.load(args.registry)
    run = lb.LadderRun.load(args.state[0])
    for extra in args.state[1:]:
        more = lb.LadderRun.load(extra)
        if set(more.objects) & set(run.objects):
            raise ProtocolError("two batches hold the same object")
        run.objects.update(more.objects)
    if run.registry_sha256 != registry.content_hash():
        raise ProtocolError("registry differs from the one the state was started with")
    seeds = None
    if args.fresh_only:
        from restocker_benchmarks import perception_splits as sp  # noqa: PLC0415

        document = sp.load_splits(args.splits)
        seeds = {
            s
            for s in {k[0] for k in run.objects}
            if sp.held_out_generation(document, s) == "fresh"
        }
    report: dict = {
        "state": [p.name for p in args.state],
        "registry_sha256": run.registry_sha256,
        "meta": run.meta,
        "rounds_done": run.rounds_done,
        "arms": {
            arm: {
                f"budget_{b}": lb.summarise(run, registry, arm=arm, budget=b, seeds=seeds)
                for b in range(args.budget + 1)
            }
            for arm in lb.ARMS
        },
        "facing": lb.facing_report(run),
    }
    if args.bounds:
        bounds = json.loads(args.bounds.read_text(encoding="utf-8"))
        bounds_hash = hashlib.sha256(args.bounds.read_bytes()).hexdigest()
        if run.meta.get("bounds_sha256") not in (None, bounds_hash):
            raise ProtocolError("the state was produced under another bounds file")
        report["bounds_sha256"] = bounds_hash
        headline = bounds["headline_arm"]
        budget = f"budget_{bounds['budget_relooks']}"
        report["verdict"] = {
            "arm": headline,
            **lb.verdict(report["arms"][headline][budget], bounds),
        }
        # Every other arm the bounds give a coverage floor for is judged by the same rule.
        report["verdicts"] = {
            arm: lb.verdict(report["arms"][arm][budget], bounds)
            for arm in lb.ARMS
            if arm in bounds["bounds"]["coverage_known_min"]
        }
    if args.out.exists():
        raise ProtocolError(f"{args.out} exists; a report is written once")
    lb.dump_report(args.out, report)
    print(json.dumps(report.get("verdict", {"arms": "written"}), indent=2, default=float))
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.command == "whitener":
            return _whitener_command(args)
        if args.command == "step":
            return _step_command(args)
        return _report_command(args)
    except (RegistryError, ProtocolError, ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
