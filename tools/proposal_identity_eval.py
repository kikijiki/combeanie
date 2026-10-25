#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Run proposals through the identity channel and report the identity states (CMB-113).

Offline, venv only (torch, transformers, RapidOCR, cv2). For every proposal of every wrist frame
the geometry proposer makes, cut a square crop around its pixel box, ask the identity channel (one
view: a proposal is one look), resolve the proposal's identity state, and score it against the
product the proposal covers. The identity channel is consumed as calibrated in CMB-112 round 2;
nothing here fits or changes it.

    proposal_identity_eval.py --registry R --calibration C --whitener W --dataset D... --out O.json
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
for package in ("restocker_benchmarks", "restocker_gazebo", "restocker_perception"):
    sys.path.insert(0, str(ROOT / "ros_ws" / "src" / package))

from restocker_benchmarks import perception_dataset as dataset  # noqa: E402
from restocker_benchmarks import perception_proposal_scorer as scorer  # noqa: E402
from restocker_perception.identity.service import IdentifyService  # noqa: E402
from restocker_perception.proposals import (  # noqa: E402
    CameraModel,
    ProposalConfig,
    identity_fields,
    propose,
    resolve_identity,
)
from restocker_perception.registry.backends import RapidOcrBackend, SigLip2Backend  # noqa: E402
from restocker_perception.registry.model import Registry  # noqa: E402

MIN_PIXELS = 1500
MARGIN = 1.15


def crop_of(image: np.ndarray, bbox: tuple[int, int, int, int]) -> np.ndarray | None:
    u0, v0, u1, v1 = bbox
    side = max(u1 - u0, v1 - v0) * MARGIN
    cu, cv = (u0 + u1) / 2.0, (v0 + v1) / 2.0
    x0, y0 = int(max(cu - side / 2, 0)), int(max(cv - side / 2, 0))
    x1, y1 = int(min(cu + side / 2, image.shape[1])), int(min(cv + side / 2, image.shape[0]))
    if x1 - x0 < 24 or y1 - y0 < 24:
        return None
    return np.ascontiguousarray(image[y0:y1, x0:x1, :3])


def main() -> int:
    import cv2  # noqa: PLC0415

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--whitener", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, nargs="+", required=True)
    parser.add_argument("--revision", required=True)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    if args.out and args.out.exists():
        raise SystemExit(f"{args.out} exists; not overwritten")

    registry = Registry.load(args.registry)
    registered = {a for r in registry.records() for a in r.aliases}
    geometry = {r.product_id: (r.radius_m, r.height_m) for r in registry.records()}
    alias_of = {r.product_id: r.aliases[0] for r in registry.records() if r.aliases}
    service = IdentifyService.load(
        str(args.registry),
        str(args.calibration),
        str(args.whitener),
        SigLip2Backend(revision=args.revision, device="cpu"),
        RapidOcrBackend(version="unpinned"),
    )
    if not service.available:
        raise SystemExit(service.unavailable_reason)
    config = ProposalConfig()
    rows = []
    frames = 0
    for directory in args.dataset:
        for row in dataset.read_labels(directory):
            if row["sensor"] != "wrist":
                continue
            frames += 1
            if args.max_frames and frames > args.max_frames:
                break
            camera = CameraModel.from_label(row["camera"])
            depth = cv2.imread(str(directory / row["files"]["depth_mm"]), -1) / 1000.0
            rgb = cv2.cvtColor(cv2.imread(str(directory / row["files"]["rgb"])), cv2.COLOR_BGR2RGB)
            found = propose(depth, camera, config=config)
            truth = scorer.ground_truth(row["objects"], camera, depth)
            for proposal in found.proposals:
                if proposal.n_pixels < MIN_PIXELS:
                    continue
                mask = found.mask(proposal.proposal_id)
                best, share = None, 0.0
                for entry in truth:
                    s = float(np.logical_and(mask, entry["mask"]).sum()) / max(int(mask.sum()), 1)
                    if s > share:
                        best, share = entry, s
                crop = crop_of(rgb, proposal.bbox)
                if crop is None:
                    continue
                result = service.engine.identify([crop], [])
                identity = resolve_identity(
                    proposal,
                    result,
                    geometry_of_sku=geometry.get,
                    config=config,
                )
                fields = identity_fields(identity)
                assert not (fields["identity_state"] == 2 and fields["has_sku"])
                truth_sku = best["label"]["sku"] if best and share >= 0.5 else None
                truth_class = (
                    "foreign"
                    if truth_sku is None
                    else ("registered" if truth_sku in registered else "unregistered")
                )
                named = alias_of.get(identity.sku)
                rows.append(
                    {
                        "frame_id": row["frame_id"],
                        "view": row.get("viewpoint") or "home",
                        "kind": proposal.kind,
                        "truth_class": truth_class,
                        "truth_sku": truth_sku,
                        "state": identity.state_name,
                        "named": named,
                        "channel_status": result.status,
                        "reason": identity.reason,
                        "wrong_sku": identity.sku is not None and named != truth_sku,
                        "confirmed": identity.sku is not None
                        and bool({"ocr", "barcode"} & set(identity.evidence_kinds)),
                    }
                )
    summary: dict[str, dict] = defaultdict(dict)
    for truth_class in ("registered", "unregistered", "foreign"):
        group = [r for r in rows if r["truth_class"] == truth_class]
        states = Counter(r["state"] for r in group)
        summary[truth_class] = {
            "proposals": len(group),
            "states": dict(states),
            "forced_wrong_sku": sum(r["wrong_sku"] for r in group),
            "correct_sku": sum(r["state"] == "identified" and not r["wrong_sku"] for r in group),
            "channel_status": dict(Counter(r["channel_status"] for r in group)),
            # Identified with OCR or barcode evidence: the safe floor of CMB-112 round 2.
            "confirmed_identified": sum(r["confirmed"] for r in group),
            "confirmed_wrong_sku": sum(r["confirmed"] and r["wrong_sku"] for r in group),
        }
    summary["unknown_with_sku"] = sum(
        1 for r in rows if r["state"] == "unknown_product" and r["named"]
    )
    out = {"frames": frames, "summary": summary, "rows": rows}
    text = json.dumps(out, indent=1, sort_keys=True)
    if args.out:
        args.out.write_text(text + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=1, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
