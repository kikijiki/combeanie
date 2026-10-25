#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity channel on recorded scenes with boxes (CMB-119 diagnostic, train and validation seeds).

Runs the shipped identity engine exactly as calibrated (an existing calibration and whitener,
nothing is fitted here) over every object of a recorded dataset, once with the barcode channel on
and once off, and tallies the decision against the truth by product shape. Held-out seeds are
refused (the dataset reader raises). Offline venv only (torch, transformers, RapidOCR, zxing).

    box_identity_diag.py --registry R --calibration C --whitener W --dataset D... --out O.json
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
for package in ("restocker_benchmarks", "restocker_gazebo", "restocker_perception"):
    sys.path.insert(0, str(ROOT / "ros_ws" / "src" / package))

from restocker_perception.identity import benchmark as bm  # noqa: E402
from restocker_perception.identity.barcode import ZxingBarcodeBackend  # noqa: E402
from restocker_perception.identity.config import load_calibration  # noqa: E402
from restocker_perception.identity.engine import IdentityEngine  # noqa: E402
from restocker_perception.registry.backends import RapidOcrBackend, SigLip2Backend  # noqa: E402
from restocker_perception.registry.model import Registry  # noqa: E402
from restocker_perception.registry.retrieval import RegistryIndex  # noqa: E402
from restocker_perception.registry.simulated import load_geometry_catalogue  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", required=True, type=Path)
    parser.add_argument("--calibration", required=True, type=Path)
    parser.add_argument("--whitener", required=True, type=Path)
    parser.add_argument("--dataset", required=True, action="append", type=Path)
    parser.add_argument("--geometry", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    registry = Registry.load(args.registry)
    config, _fit, _sha = load_calibration(args.calibration)
    whitener = bm.load_whitener(args.whitener)
    backend = SigLip2Backend()
    ocr = RapidOcrBackend(version="unpinned")
    barcode = ZxingBarcodeBackend(version="unpinned")
    geometry = load_geometry_catalogue(args.geometry)
    index = RegistryIndex.from_registry(
        registry, backend, whitener=whitener if config.whiten else None
    )
    ocr_strings = bm.ocr_strings(registry)
    groups = []
    for directory in args.dataset:
        groups += list(
            bm.object_groups(directory, geometry, allowed_splits=("train", "validation"))
        )
    print(f"{len(groups)} object groups", flush=True)

    rows = []
    for barcode_on in (True, False):
        engine = IdentityEngine(
            index,
            backend,
            config,
            ocr_backend=ocr,
            ocr_strings=ocr_strings,
            barcode_backend=barcode,
            barcode_enabled=barcode_on,
        )
        for group in groups:
            for label, views in (("multi", group.views), *(("single", (v,)) for v in group.views)):
                result = engine.identify(
                    [v.image for v in views[: config.max_views]],
                    [
                        image
                        for v in views
                        if str(v.meta.get("viewpoint") or "").startswith("barcode")
                        for image in v.meta["barcode_variants"]
                    ],
                )
                truth = registry.resolve(group.sku)
                rows.append(
                    {
                        "barcode_enabled": barcode_on,
                        "unit": label,
                        "seed": group.seed,
                        "object_id": group.object_id,
                        "truth_sku": group.sku,
                        "known": truth is not None,
                        "product_class": group.product_class,
                        "has_barcode_view": any(
                            str(v.meta.get("viewpoint") or "").startswith("barcode") for v in views
                        ),
                        "status": result.status,
                        "named": result.sku,
                        "correct": truth is not None and result.sku == truth,
                        "reason": result.reason,
                        "evidence_kinds": sorted({e.kind for e in result.evidence}),
                    }
                )
    summary: dict = defaultdict(dict)
    for key in sorted(
        {(r["barcode_enabled"], r["unit"], r["product_class"], r["known"]) for r in rows}
    ):
        subset = [
            r
            for r in rows
            if (r["barcode_enabled"], r["unit"], r["product_class"], r["known"]) == key
        ]
        arm = "on" if key[0] else "off"
        registered = "registered" if key[3] else "unregistered"
        name = f"barcode={arm} {key[1]} {key[2]} {registered}"
        status = Counter(r["status"] for r in subset)
        summary[name] = {
            "n": len(subset),
            "status": dict(status),
            "identified_correct": sum(r["correct"] for r in subset),
            "identified_wrong": sum(
                1 for r in subset if r["status"] == "identified" and not r["correct"]
            ),
            "by_evidence": dict(Counter("+".join(r["evidence_kinds"]) or "none" for r in subset)),
        }
    args.out.write_text(json.dumps({"summary": summary, "rows": rows}, indent=1) + "\n")
    print(json.dumps(summary, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
