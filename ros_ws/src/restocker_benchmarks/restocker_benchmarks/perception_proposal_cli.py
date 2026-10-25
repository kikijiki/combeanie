# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""``benchmark_proposals``: score a proposer on proposal-benchmark datasets (CMB-113)."""
# Needs numpy and cv2 (the offline venv, not the flake) and the restocker_perception package on
# the path for the geometry proposer. A development dataset can be evaluated any number of times;
# a held-out dataset is read once: the bounds file must be given, the proposer configuration is
# hashed into the result, and an existing result file is never overwritten.

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from typing import Any

import numpy as np

from restocker_benchmarks import perception_dataset as dataset
from restocker_benchmarks import perception_proposal_scorer as scorer
from restocker_benchmarks import perception_proposals as scenes

EXIT_REFUSED = 2


def config_sha256(config: Any) -> str:
    """Fingerprint a proposer configuration dataclass."""
    return hashlib.sha256(
        json.dumps(config.__dict__, sort_keys=True, default=list).encode()
    ).hexdigest()


def classify_dataset(manifest: dict[str, Any]) -> str:
    """'development' or 'held_out' by the seeds a dataset holds; mixed or foreign is refused."""
    names = {scenes.split_of_proposal_seed(int(seed)) for seed in manifest["seeds"]}
    if len(names) != 1:
        raise ValueError(f"a dataset must hold one split, found {sorted(names)}")
    return names.pop()


def read_depth_m(directory: Path, row: dict[str, Any]) -> np.ndarray:
    import cv2  # noqa: PLC0415 - optional dependency

    image = cv2.imread(str(directory / row["files"]["depth_mm"]), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise FileNotFoundError(row["files"]["depth_mm"])
    return image.astype(np.float64) / 1000.0


def evaluate(
    directories: list[Path],
    *,
    proposer: Any,
    config: Any,
    bounds: dict[str, Any] | None,
    sensors: tuple[str, ...] = ("wrist",),
) -> dict[str, Any]:
    """Run a proposer over every frame of the datasets and score it; returns the result dict."""
    from restocker_perception.proposals import CameraModel  # noqa: PLC0415

    objects: list[dict[str, Any]] = []
    proposals: list[dict[str, Any]] = []
    latencies: list[float] = []
    residuals: list[float] = []
    frames = 0
    for directory in directories:
        for row in dataset.read_labels(directory):
            if row["sensor"] not in sensors:
                continue
            camera = CameraModel.from_label(row["camera"])
            depth = read_depth_m(directory, row)
            started = time.perf_counter()
            found = proposer(depth, camera, row)
            latencies.append((time.perf_counter() - started) * 1000.0)
            truth = scorer.ground_truth(row["objects"], camera, depth)
            residuals += [
                e["depth_residual_mm"] for e in truth if e["depth_residual_mm"] is not None
            ]
            record = scorer.score_frame(row, truth, found.labels, found.proposals)
            objects += record["objects"]
            proposals += record["proposals"]
            frames += 1
    report = scorer.aggregate(objects, proposals)
    report["frames"] = frames
    report["latency_ms"] = {
        "p50": float(np.quantile(latencies, 0.5)) if latencies else None,
        "p95": float(np.quantile(latencies, 0.95)) if latencies else None,
    }
    report["ground_truth_depth_residual_mm"] = {
        "median_of_medians": float(np.median(residuals)) if residuals else None,
        "p95": float(np.quantile(residuals, 0.95)) if residuals else None,
    }
    if bounds is not None:
        decide = scorer.decide_boxes if bounds.get("kind") == "box_proposals" else scorer.decide
        report["decision"] = decide(objects, proposals, latencies, bounds)
    report["config_sha256"] = config_sha256(config)
    report["records"] = {"objects": objects, "proposals": proposals}
    return report


def _cmd_evaluate(args: argparse.Namespace) -> int:
    from restocker_perception.proposals import ProposalConfig, propose  # noqa: PLC0415

    config = ProposalConfig()
    directories = [Path(path) for path in args.dataset]
    manifests = [
        json.loads((d / "manifest.json").read_text(encoding="utf-8")) for d in directories
    ]
    splits = {classify_dataset(m) for m in manifests}
    if len(splits) != 1:
        print(f"refused: datasets of different splits {sorted(splits)}", file=sys.stderr)
        return EXIT_REFUSED
    split = splits.pop()
    bounds = None
    out = Path(args.out) if args.out else None
    if split == "held_out":
        if not args.bounds or not out:
            print("refused: a held-out dataset needs --bounds and --out", file=sys.stderr)
            return EXIT_REFUSED
        if out.exists():
            print(
                f"refused: {out} exists; a held-out result is never overwritten", file=sys.stderr
            )
            return EXIT_REFUSED
    if args.bounds:
        bounds = json.loads(Path(args.bounds).read_text(encoding="utf-8"))

    def proposer(depth, camera, row):
        return propose(depth, camera, config=config)

    report = evaluate(directories, proposer=proposer, config=config, bounds=bounds)
    report["split"] = split
    report["datasets"] = [d.name for d in directories]
    if bounds is not None:
        report["bounds_sha256"] = hashlib.sha256(Path(args.bounds).read_bytes()).hexdigest()
    text = json.dumps(report, indent=1, sort_keys=True, default=float) + "\n"
    if out:
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(text, encoding="utf-8")
        report["result_sha256"] = hashlib.sha256(text.encode()).hexdigest()
    print(format_report(report))
    return 0


def format_report(report: dict[str, Any]) -> str:
    """A short human summary of an evaluation."""

    def line(name: str, block: dict[str, Any]) -> str:
        r = block["recall_strict"]
        loose = block["recall_loose"]["value"]
        ci = r["cluster_ci95"]
        interval = "n/a" if ci is None else f"[{ci[0]:.2f}, {ci[1]:.2f}]"
        value = "n/a" if r["value"] is None else f"{r['value']:.3f}"
        loose_text = "n/a" if loose is None else f"{loose:.3f}"
        return (
            f"  {name:<28} strict {value} {interval} k={r['k']}/{r['n']} "
            f"scenes={r['scenes']} loose {loose_text}"
        )

    lines = [f"frames {report['frames']}", line("overall", report["overall"])]
    for key, groups in report["strata"].items():
        lines.append(f" by {key}:")
        lines += [line(name, block) for name, block in groups.items()]
    p = report["proposals"]
    lines.append(
        f" proposals {p['n']} kinds {p['kinds']} foreign {p['foreign']['value']} "
        f"merged {p['merged']['value']} precision {p['precision_strict']['value']}"
    )
    lines.append(
        f" latency {report['latency_ms']}"
        f" depth residual {report['ground_truth_depth_residual_mm']}"
    )
    if "decision" in report:
        decision = report["decision"]
        lines.append(f" DECISION {decision['verdict']}")
        for name, check in decision["checks"].items():
            lines.append(f"   {name:<45} {check['verdict']} ({check.get('value')})")
    return "\n".join(lines)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="benchmark_proposals", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    evaluate_parser = sub.add_parser("evaluate", help="score the geometry proposer")
    evaluate_parser.add_argument("--dataset", nargs="+", required=True)
    evaluate_parser.add_argument("--bounds")
    evaluate_parser.add_argument("--out")
    evaluate_parser.set_defaults(handler=_cmd_evaluate)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    sys.exit(main())
