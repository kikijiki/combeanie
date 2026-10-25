# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""``stock_benchmark predict``: lane stock predictions over recorded lane survey frames."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from restocker_perception.stock import benchmark


def _load_channel(arguments):
    """The real identity logic in-process: what /identify_product answers for one crop."""
    from restocker_perception.identity.service import IdentifyService
    from restocker_perception.registry.backends import RapidOcrBackend, SigLip2Backend

    for name in ("registry", "calibration", "whitener"):
        if not getattr(arguments, name):
            raise SystemExit(f"--arm channel needs --{name}")
    service = IdentifyService.load(
        arguments.registry,
        arguments.calibration,
        arguments.whitener,
        SigLip2Backend(revision=arguments.revision, device="cpu"),
        RapidOcrBackend(version=arguments.ocr_version),
    )
    if not service.available:
        raise SystemExit(f"identity sidecar did not load: {service.unavailable_reason}")
    import yaml

    document = yaml.safe_load(Path(arguments.catalogue).read_text(encoding="utf-8"))
    to_sku = {row["gtin"]: row["sku"] for row in document["products"] if row.get("gtin")}

    def identify(crop):
        reply = dict(service.identify([crop], []))
        # The channel names registry product ids (the GTIN when a product has one); the
        # benchmark and the lane policy use SKUs.
        reply["sku"] = to_sku.get(reply.get("sku", ""), reply.get("sku", ""))
        reply["evidence_product_ids"] = [
            to_sku.get(g, g) for g in reply.get("evidence_product_ids", [])
        ]
        return reply

    return identify


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--datasets", nargs="+", required=True, help="pbench-* directories")
    parser.add_argument("--catalogue", required=True, help="generated_product_labels.yaml")
    parser.add_argument("--arm", choices=benchmark.ARMS, default="abstain")
    parser.add_argument("--degrade", action="store_true", help="blind a seeded depth band")
    parser.add_argument("--noise", type=float, default=0.0, help="depth noise, fraction of range")
    parser.add_argument("--dropout", type=float, default=0.0, help="share of depth pixels dropped")
    parser.add_argument("--seeds", type=int, nargs="*", help="only these seeds")
    parser.add_argument("--out", required=True)
    parser.add_argument("--config", help="lane_stock.yaml override")
    parser.add_argument(
        "--identity-mode",
        choices=("safe", "embedding"),
        default="safe",
        help="safe: name only what OCR or the barcode confirms (the default policy)",
    )
    parser.add_argument("--registry", help="arm channel: registry directory")
    parser.add_argument("--calibration", help="arm channel: identity calibration JSON")
    parser.add_argument("--whitener", help="arm channel: whitening npz")
    parser.add_argument("--ocr-version", default="unpinned", help="arm channel: rapidocr version")
    parser.add_argument("--revision", default="main", help="arm channel: SigLIP 2 revision")
    arguments = parser.parse_args(argv)
    config = None
    if arguments.config:
        from restocker_perception.stock.config import load_stock_config

        config = load_stock_config(arguments.config)
    channel = None
    if arguments.arm == "channel":
        channel = _load_channel(arguments)
    rows = benchmark.predict_datasets(
        arguments.datasets,
        arguments.catalogue,
        arm=arguments.arm,
        degrade=arguments.degrade,
        seeds=arguments.seeds,
        config=config,
        channel=channel,
        identity_mode=arguments.identity_mode,
        noise=arguments.noise,
        dropout=arguments.dropout,
    )
    Path(arguments.out).write_text(
        "".join(json.dumps(row, sort_keys=True) + "\n" for row in rows), encoding="utf-8"
    )
    print(f"wrote {len(rows)} lane predictions to {arguments.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
