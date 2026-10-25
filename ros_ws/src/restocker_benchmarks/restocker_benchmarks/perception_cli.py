# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Command line for the perception benchmark: splits, run guard, scenes, scoring, baselines."""
# Everything here is offline. The simulator-backed recorder is a separate launch test
# (restocker_bringup/test/test_perception_recorder_runtime.py) driven by
# tools/record_perception_dataset.py; this tool prepares what it records and scores what it made.

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

import yaml

from restocker_benchmarks import perception_dataset as dataset
from restocker_benchmarks import perception_inventory_scorer as inventory_scorer
from restocker_benchmarks import perception_scorer as scorer
from restocker_benchmarks import perception_splits as splits_module
from restocker_benchmarks import perception_stock_scorer as stock_scorer

EXIT_REFUSED = 2


def lookalike_map(products: list[dict[str, Any]]) -> dict[str, str | None]:
    """{sku: look-alike group} for the scorer's twin-confusion statistic."""
    return {row["sku"]: row.get("lookalike_group") for row in products}


def scene_for_seed(
    seed: int, products: list[dict[str, Any]], split_document: dict[str, Any]
) -> dict[str, Any]:
    """Build the scene of a frozen seed, honouring its split's product restriction."""
    allowed = splits_module.allowed_skus_for_seed(split_document, seed)
    scene = dataset.build_scene(
        seed,
        products,
        allowed_skus=allowed,
        include_boxes=splits_module.scene_includes_boxes(split_document, seed),
    )
    scene["truth"] = dataset.scene_truth(scene["scenario"], scene["lane_expected_sku"])
    scene["seed_split"] = splits_module.split_of_seed(split_document, seed)
    scene["scenario_id"] = dataset.scenario_id(seed)
    return scene


def _load_products(
    path: str, split_document: dict[str, Any] | None = None
) -> list[dict[str, Any]]:
    if split_document is not None:
        return dataset.load_catalogue_for_splits(path, split_document)
    return dataset.load_generated_catalogue(path)


def _cmd_splits(args: argparse.Namespace) -> int:
    verified = splits_module.load_splits(args.verify) if args.verify else None
    products = _load_products(args.catalogue, verified)
    catalogue_hash = dataset.catalogue_sha256(products)
    if args.extend_box_seeds:
        base = splits_module.load_splits(args.extend_box_seeds)
        document = splits_module.extend_box_seeds(
            base,
            last_seed=args.last_seed,
            seed_counts=tuple(int(item) for item in args.counts.split(",")),
            burn=[int(item) for item in args.burn.split(",") if item],
        )
        splits_module.write_splits(document, args.out)
        print(f"wrote {args.out} sha256={document['sha256']} (base {base['sha256']})")
        return 0
    if args.extend_identity:
        base = splits_module.load_splits(args.extend_identity)
        document = splits_module.extend_identity_round(
            base,
            last_seed=args.last_seed,
            seed_counts=tuple(int(item) for item in args.counts.split(",")),
            unregistered=[item for item in args.unregistered.split(",") if item],
            burn_held_out_through=args.burn_through,
        )
        splits_module.write_splits(document, args.out)
        print(f"wrote {args.out} sha256={document['sha256']} (base {base['sha256']})")
        return 0
    if args.extend_boxes:
        base = splits_module.load_splits(args.extend_boxes)
        with_boxes = dataset.load_generated_catalogue(args.catalogue, include_boxes=True)
        new_rows = [row for row in with_boxes if row.get("shape") == "box"]
        document = splits_module.extend_with_products(
            base,
            new_rows,
            last_seed=args.last_seed,
            seed_counts=tuple(int(item) for item in args.counts.split(",")),
            product_counts=tuple(int(item) for item in args.product_counts.split(",")),
            catalogue_sha256=dataset.catalogue_sha256(with_boxes),
        )
        splits_module.write_splits(document, args.out)
        print(f"wrote {args.out} sha256={document['sha256']} (base {base['sha256']})")
        return 0
    if args.extend:
        base = splits_module.load_splits(args.extend)
        counts = tuple(int(item) for item in args.counts.split(","))
        document = splits_module.extend_splits(base, last_seed=args.last_seed, counts=counts)
        splits_module.write_splits(document, args.out)
        print(f"wrote {args.out} sha256={document['sha256']} (base {base['sha256']})")
        return 0
    if args.verify:
        document = splits_module.load_splits(args.verify)
        if document["catalogue_sha256"] != catalogue_hash:
            print("splits file was frozen against a different catalogue", file=sys.stderr)
            return EXIT_REFUSED
        if document["schema_version"] in (
            splits_module.SPLITS_SCHEMA_VERSION_EXTENDED,
            splits_module.SPLITS_SCHEMA_VERSION_BOXES,
            splits_module.SPLITS_SCHEMA_VERSION_BOX_SEEDS,
            splits_module.SPLITS_SCHEMA_VERSION_IDENTITY,
        ):
            fresh = splits_module.reproduce_extension(document)
        else:
            fresh = splits_module.freeze_splits(
                products,
                list(splits_module.seeds_range(args.seeds)),
                split_seed=document["split_seed"],
                catalogue_sha256=catalogue_hash,
            )
        if fresh != document:
            print(
                "regenerating from the recorded split_seed does not reproduce the file",
                file=sys.stderr,
            )
            return EXIT_REFUSED
        print(f"splits ok sha256={document['sha256']}")
        return 0
    document = splits_module.freeze_splits(
        products,
        list(splits_module.seeds_range(args.seeds)),
        split_seed=args.split_seed,
        catalogue_sha256=catalogue_hash,
    )
    splits_module.write_splits(document, args.out)
    print(f"wrote {args.out} sha256={document['sha256']}")
    return 0


def _cmd_check(args: argparse.Namespace) -> int:
    document = splits_module.load_splits(args.splits)
    run = json.loads(Path(args.run).read_text(encoding="utf-8"))
    problems = splits_module.check_run(document, run)
    if problems:
        print("REFUSED: this run crosses the held-out boundary", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return EXIT_REFUSED
    print("run ok: registry and training seeds are clear of the held-out split")
    return 0


def _cmd_scenes(args: argparse.Namespace) -> int:
    document = splits_module.load_splits(args.splits)
    products = _load_products(args.catalogue, document)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    seeds = args.seed or sorted(
        seed for name in splits_module.SPLIT_NAMES for seed in document["seeds"][name]
    )
    for seed in seeds:
        scene = scene_for_seed(seed, products, document)
        (out / f"{scene['scenario_id']}.yaml").write_text(
            yaml.safe_dump(scene["scenario"], sort_keys=True), encoding="utf-8"
        )
        (out / f"{scene['scenario_id']}.truth.json").write_text(
            json.dumps(
                {
                    "seed": seed,
                    "seed_split": scene["seed_split"],
                    "lane_expected_sku": scene["lane_expected_sku"],
                    **scene["truth"],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
    print(f"wrote {len(seeds)} scenes to {out}")
    return 0


def _cmd_score(args: argparse.Namespace) -> int:
    labels = dataset.read_labels(args.labels)
    predictions = scorer.read_predictions(args.predictions)
    document = splits_module.load_splits(args.splits)
    registry = args.registry or splits_module.registered_skus(document)
    report = scorer.score(
        labels,
        predictions,
        registry,
        lookalike_groups={k: v for k, v in document["lookalike_groups"].items()},
    )
    if args.json:
        Path(args.json).write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(scorer.format_report(report), end="")
    return 0


def _cmd_stock_score(args: argparse.Namespace) -> int:
    labels = [row for path in args.labels for row in dataset.read_labels(path)]
    predictions = []
    for path in args.predictions:
        predictions += scorer.read_predictions(path)
    report = stock_scorer.score_stock(labels, predictions)
    if args.json:
        Path(args.json).write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(stock_scorer.format_stock_report(report))
    return 0


def _cmd_stock_evaluate(args: argparse.Namespace) -> int:
    bounds = json.loads(Path(args.bounds).read_text(encoding="utf-8"))
    reports = {}
    for item in args.report:
        label, _, path = item.partition("=")
        reports[label] = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = stock_scorer.evaluate_bounds(reports, bounds)
    for row in rows:
        observed = (
            ""
            if row.get("observed") is None
            else f" observed {row['observed']:.4f} k={row['k']}/{row['n']}"
        )
        print(f"{row['bound']:48s} {row['arm']:16s} {row['verdict']:13s}{observed}")
    if args.json:
        Path(args.json).write_text(
            json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return 0


def _cmd_inventory_score(args: argparse.Namespace) -> int:
    from restocker_perception.inventory.model import InventoryConfig

    labels = [row for path in args.labels for row in dataset.read_labels(path)]
    predictions = []
    for path in args.predictions:
        predictions += scorer.read_predictions(path)
    report = inventory_scorer.score_inventory(
        labels, predictions, InventoryConfig(stale_after_s=args.horizon)
    )
    if args.json:
        Path(args.json).write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(inventory_scorer.format_inventory_report(report))
    return 0


def _cmd_inventory_evaluate(args: argparse.Namespace) -> int:
    bounds = json.loads(Path(args.bounds).read_text(encoding="utf-8"))
    reports = {}
    for item in args.report:
        label, _, path = item.partition("=")
        reports[label] = json.loads(Path(path).read_text(encoding="utf-8"))
    rows = inventory_scorer.evaluate_bounds(reports, bounds)
    for row in rows:
        observed = (
            ""
            if row.get("observed") is None
            else f" observed {row['observed']:.4f} k={row['k']}/{row['n']}"
        )
        print(f"{row['bound']:48s} {row['arm']:16s} {row['verdict']:13s}{observed}")
    if args.json:
        Path(args.json).write_text(
            json.dumps(rows, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    return 0


def _cmd_stock_seeds(args: argparse.Namespace) -> int:
    document = splits_module.load_splits(args.splits)
    print(" ".join(str(s) for s in stock_scorer.stock_held_out_population(document, args.round)))
    return 0


def _cmd_baseline(args: argparse.Namespace) -> int:
    labels = dataset.read_labels(args.labels)
    document = splits_module.load_splits(args.splits)
    registry = splits_module.registered_skus(document)
    if args.kind == "truth":
        rows = scorer.ground_truth_predictions(labels)
    else:
        rows = scorer.noisy_predictions(
            labels,
            registry,
            seed=args.seed,
            lookalike_groups=dict(document["lookalike_groups"]),
        )
    scorer.write_predictions(rows, args.out)
    print(f"wrote {len(rows)} prediction rows to {args.out}")
    return 0


def _cmd_verify(args: argparse.Namespace) -> int:
    problems = dataset.verify_dataset(args.dataset)
    for problem in problems:
        print(f"  - {problem}", file=sys.stderr)
    print("dataset ok" if not problems else "DATASET INVALID")
    return 1 if problems else 0


def build_parser() -> argparse.ArgumentParser:
    """The argument parser (exposed for tests)."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("splits", help="freeze, or verify, the split file")
    p.add_argument("--catalogue", required=True)
    p.add_argument("--seeds", type=int, default=24, help="seeds 1..N")
    p.add_argument("--split-seed", type=int, default=109)
    p.add_argument("--out")
    p.add_argument("--verify", help="split file to verify instead of writing")
    p.add_argument("--extend", help="version-1 split file to extend append-only into a version 2")
    p.add_argument("--last-seed", type=int, help="with --extend: last seed to add")
    p.add_argument(
        "--extend-boxes",
        help="version-2 split file to extend with the box SKUs and new seeds into a version 3",
    )
    p.add_argument(
        "--extend-box-seeds",
        help="version-3 or version-4 split file to extend with new seeds only into a version 4",
    )
    p.add_argument("--burn", default="", help="with --extend-box-seeds: fresh seeds to mark burned")
    p.add_argument("--product-counts", default="2,1,1", help="with --extend-boxes: train,val,held")
    p.add_argument(
        "--extend-identity",
        help="version-3 split file to extend with seeds and the identity round's registry roles "
        "into a version 4 (needs --last-seed, --counts, --unregistered, --burn-through)",
    )
    p.add_argument("--unregistered", default="", help="with --extend-identity: SKUs left out")
    p.add_argument(
        "--burn-through",
        type=int,
        default=0,
        help="with --extend-identity: held-out seeds up to this one are burned",
    )
    p.add_argument(
        "--counts", help="with --extend: train,validation,held_out counts of added seeds"
    )
    p.set_defaults(func=_cmd_splits)

    p = sub.add_parser("check", help="refuse a run whose registry or seeds overlap held-out")
    p.add_argument("--splits", required=True)
    p.add_argument("--run", required=True, help="JSON: registry_skus, training_seeds, ...")
    p.set_defaults(func=_cmd_check)

    p = sub.add_parser("scenes", help="write scenario YAML and truth for frozen seeds")
    p.add_argument("--catalogue", required=True)
    p.add_argument("--splits", required=True)
    p.add_argument("--out", required=True)
    p.add_argument("--seed", type=int, action="append")
    p.set_defaults(func=_cmd_scenes)

    p = sub.add_parser("score", help="score a predictions JSONL against a dataset")
    p.add_argument("--labels", required=True, help="dataset directory or labels.jsonl")
    p.add_argument("--predictions", required=True)
    p.add_argument("--splits", required=True)
    p.add_argument("--registry", nargs="*", help="override the registered SKUs")
    p.add_argument("--json")
    p.set_defaults(func=_cmd_score)

    p = sub.add_parser(
        "stock-score", help="score lane stock estimator predictions (CMB-114) against lane truth"
    )
    p.add_argument(
        "--labels", nargs="+", required=True, help="dataset directories or labels.jsonl"
    )
    p.add_argument("--predictions", nargs="+", required=True)
    p.add_argument("--json")
    p.set_defaults(func=_cmd_stock_score)

    p = sub.add_parser(
        "stock-evaluate", help="apply the pre-declared CMB-114 bounds to stock score reports"
    )
    p.add_argument("--bounds", required=True)
    p.add_argument("--report", nargs="+", required=True, help="ARM=score.json ...")
    p.add_argument("--json")
    p.set_defaults(func=_cmd_stock_evaluate)

    p = sub.add_parser(
        "inventory-score",
        help="replay lane stock predictions as inventories and score them (CMB-115)",
    )
    p.add_argument(
        "--labels", nargs="+", required=True, help="dataset directories or labels.jsonl"
    )
    p.add_argument("--predictions", nargs="+", required=True)
    p.add_argument("--horizon", type=float, default=600.0, help="stale-after, seconds")
    p.add_argument("--json")
    p.set_defaults(func=_cmd_inventory_score)

    p = sub.add_parser(
        "inventory-evaluate", help="apply the pre-declared CMB-115 bounds to inventory reports"
    )
    p.add_argument("--bounds", required=True)
    p.add_argument("--report", nargs="+", required=True, help="ARM=score.json ...")
    p.add_argument("--json")
    p.set_defaults(func=_cmd_inventory_evaluate)

    p = sub.add_parser("stock-seeds", help="print the pre-declared CMB-114 held-out seeds")
    p.add_argument("--splits", required=True)
    p.add_argument("--round", type=int, default=1, choices=(1, 2))
    p.set_defaults(func=_cmd_stock_seeds)

    p = sub.add_parser("baseline", help="write baseline predictions (truth or noisy)")
    p.add_argument("--labels", required=True)
    p.add_argument("--splits", required=True)
    p.add_argument("--kind", choices=("truth", "noisy"), default="truth")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--out", required=True)
    p.set_defaults(func=_cmd_baseline)

    p = sub.add_parser("verify", help="recompute every hash of a dataset directory")
    p.add_argument("dataset")
    p.set_defaults(func=_cmd_verify)
    return parser


def main(argv: list[str] | None = None) -> int:
    """Entry point."""
    arguments = build_parser().parse_args(argv)
    if arguments.command == "splits" and not (arguments.verify or arguments.out):
        print("splits needs --out (write) or --verify", file=sys.stderr)
        return 2
    try:
        return arguments.func(arguments)
    except (ValueError, scorer.ScoreError, splits_module.SplitViolation) as error:
        print(f"error: {error}", file=sys.stderr)
        return EXIT_REFUSED


if __name__ == "__main__":
    sys.exit(main())
