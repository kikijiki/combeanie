# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""An /identify_product reply as what the stock estimator may believe: the safe policy."""
# CMB-112 round 2 failed its bounds: the embedding alone names an unregistered look-alike twin of
# a registered single as that twin 43.6 % of the time. The identity channel is consumed as it is,
# so the estimator reads a name only when an independent channel confirms it, which is the safe
# floor the measurement recorded (OCR or the barcode named the same GTIN). Everything else the
# channel answered is an abstention here, never a name. ``mode="embedding"`` accepts the channel's
# own IDENTIFIED verdict and exists only to measure what the policy costs.

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

from restocker_perception.stock.estimator import (
    IDENTITY_ABSTAIN,
    IDENTITY_NAMED,
    IDENTITY_UNAVAILABLE,
    IDENTITY_UNKNOWN,
    FaceIdentity,
)

STATUS_IDENTIFIED = 0
STATUS_ABSTAIN = 1
STATUS_UNKNOWN = 2
CONFIRMING_KINDS = frozenset({"ocr", "barcode"})
MODES = ("safe", "embedding")


def face_from_reply(reply: Mapping[str, Any] | None, mode: str = "safe") -> FaceIdentity:
    """Apply the naming policy to one reply dict (the keys of IdentifyProduct.Response)."""
    if mode not in MODES:
        raise ValueError(f"mode must be one of {MODES}")
    if reply is None or not reply.get("sidecar_available", False):
        return FaceIdentity(IDENTITY_UNAVAILABLE, None, "identity channel unavailable")
    status = int(reply.get("status", STATUS_ABSTAIN))
    sku = reply.get("sku") if reply.get("has_sku") else None
    if status == STATUS_UNKNOWN:
        return FaceIdentity(IDENTITY_UNKNOWN, None, str(reply.get("reason", "")))
    if status != STATUS_IDENTIFIED or not sku:
        return FaceIdentity(IDENTITY_ABSTAIN, None, str(reply.get("reason", "")))
    if mode == "embedding":
        return FaceIdentity(IDENTITY_NAMED, str(sku), "channel verdict")
    # Evidence points at product ids (evidence_gtins only for replies that predate CMB-118).
    pointed = reply.get("evidence_product_ids") or reply.get("evidence_gtins", ())
    confirmed = [
        kind
        for kind, product in zip(reply.get("evidence_kinds", ()), pointed, strict=False)
        if kind in CONFIRMING_KINDS and product == sku
    ]
    if confirmed:
        return FaceIdentity(
            IDENTITY_NAMED, str(sku), "confirmed by " + "+".join(sorted(set(confirmed)))
        )
    return FaceIdentity(
        IDENTITY_ABSTAIN, None, "named by the embedding alone: not confirmed by OCR or barcode"
    )
