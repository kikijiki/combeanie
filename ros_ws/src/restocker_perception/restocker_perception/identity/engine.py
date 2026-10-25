# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity decision: embedding, OCR and barcode channels fused with explicit abstention."""
# Three outcomes, never a forced guess:
#
#   identified  a SKU is named; the evidence list says which channels agreed.
#   abstain     the channels cannot name one product (weak, tied, or disagreeing evidence).
#   unknown     there is positive evidence that the product is not registered (a valid GTIN that
#               is not in the registry, or a best match weaker than the calibrated floor).
#
# The embedding alone does not reject unknown products (CMB-101), so by default it never names
# one on its own: naming needs an independent channel, or alone_* thresholds the calibration
# found safe. Rules, in order:
#
#   1. Barcode decoded to a valid GTIN not in the registry -> unknown.
#   2. Barcode decoded to a registered GTIN -> identified, unless OCR names another product or
#      the embedding ranks it worse than supported_max_rank (disagreement -> abstain).
#   3. OCR names a product -> identified under the same rank condition; OCR naming two products
#      (or two crops disagreeing) -> abstain.
#   4. Embedding alone, if enabled and all three thresholds are met -> identified.
#   5. Best match below unknown_below_score -> unknown; otherwise abstain.
#
# The SKU in a result is the registry product id: the GTIN for a product that has one, an internal
# id (prod-<hex>) for one that has none. Barcode-less products are named by embedding and OCR only;
# the barcode channel can be switched off (barcode_enabled=False) for a barcode-free benchmark arm.
# Write-once SKU semantics live in world state, not here.

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import time
from typing import Any

import numpy as np

from restocker_perception.identity.barcode import BarcodeBackend, BarcodeRead, read_gtin
from restocker_perception.identity.config import IdentityConfig
from restocker_perception.identity.crops import rotate_image
from restocker_perception.identity.ocr import OcrAnswer, OcrMatcher
from restocker_perception.registry.backends import EmbeddingBackend, OcrBackend
from restocker_perception.registry.gtin import InvalidGtin, validate_gtin13
from restocker_perception.registry.retrieval import MatchResult, RegistryIndex

IDENTIFIED = "identified"
ABSTAIN = "abstain"
UNKNOWN = "unknown"
STATUSES = (IDENTIFIED, ABSTAIN, UNKNOWN)


@dataclass(frozen=True)
class Evidence:
    """One channel's contribution. kind is embedding, ocr or barcode."""

    kind: str
    product_id: str | None
    score: float
    detail: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class IdentityResult:
    sku: str | None
    status: str
    reason: str
    evidence: tuple[Evidence, ...]
    provenance: dict[str, Any]
    latency_ms: dict[str, float]
    # Channels that ran for this decision (embedding, ocr, barcode), whether or not they named
    # anything; ``barcode`` is absent when it was disabled, had no images or nothing to look up.
    channels: tuple[str, ...] = ()

    @property
    def identified(self) -> bool:
        return self.status == IDENTIFIED

    def to_dict(self) -> dict[str, Any]:
        return {
            "sku": self.sku,
            "status": self.status,
            "reason": self.reason,
            "evidence": [
                {
                    "kind": e.kind,
                    "product_id": e.product_id,
                    "score": e.score,
                    "detail": e.detail,
                }
                for e in self.evidence
            ],
            "channels": list(self.channels),
            "provenance": self.provenance,
            "latency_ms": self.latency_ms,
        }


@dataclass(frozen=True)
class Observation:
    """Everything the channels measured for one object, before any decision (calibration input)."""

    match: MatchResult | None
    ocr: OcrAnswer | None
    barcode: BarcodeRead | None
    latency_ms: dict[str, float]

    @property
    def channels(self) -> tuple[str, ...]:
        """The channels that ran (produced an observation), in fixed order."""
        ran = (("embedding", self.match), ("ocr", self.ocr), ("barcode", self.barcode))
        return tuple(kind for kind, value in ran if value is not None)


class IdentityEngine:
    """Crops of one object in, :class:`IdentityResult` out."""

    def __init__(
        self,
        index: RegistryIndex,
        backend: EmbeddingBackend,
        config: IdentityConfig,
        *,
        ocr_backend: OcrBackend | None = None,
        ocr_strings: dict[str, Sequence[str]] | None = None,
        barcode_backend: BarcodeBackend | None = None,
        gtin_to_product: Mapping[str, str] | None = None,
        barcode_enabled: bool = True,
    ) -> None:
        self.index = index
        self.backend = backend
        self.config = config
        self.ocr_backend = ocr_backend
        self.ocr_matcher = OcrMatcher(ocr_strings) if ocr_backend and ocr_strings else None
        self.barcode_backend = barcode_backend
        # GTIN -> product id for the indexed products that have a GTIN. By default an indexed id
        # that is a valid GTIN-13 is one (a product with a GTIN uses it as its id; no other id
        # can be a GTIN), so a caller that has only the index need not pass it.
        self.gtin_to_product = (
            dict(gtin_to_product)
            if gtin_to_product is not None
            else {i: i for i in index.product_ids if _is_gtin(i)}
        )
        # Query-time switch (default on): False leaves the barcode channel out entirely, so the
        # engine names products by embedding and OCR as it does for barcode-less products.
        self.barcode_enabled = barcode_enabled

    def provenance(self) -> dict[str, Any]:
        out = self.index.provenance()
        out["ocr"] = (
            f"{self.ocr_backend.name} {self.ocr_backend.version}" if self.ocr_backend else None
        )
        out["barcode"] = (
            f"{self.barcode_backend.name} {self.barcode_backend.version}"
            if self.barcode_backend and self.barcode_enabled
            else None
        )
        out["barcode_enabled"] = self.barcode_enabled
        return out

    # --- measuring ---------------------------------------------------------------------------

    def observe(
        self,
        crops: Sequence[np.ndarray],
        barcode_images: Sequence[np.ndarray] = (),
    ) -> Observation:
        """Run every available channel; decide nothing."""
        crops = list(crops)[: self.config.max_views]
        latency: dict[str, float] = {}
        match = None
        if crops:
            start = time.perf_counter()
            match = self.index.identify(crops, self.backend, scoring=self.config.scoring)
            latency["embedding"] = (time.perf_counter() - start) * 1000.0
        ocr = None
        if crops and self.ocr_backend is not None and self.ocr_matcher is not None:
            start = time.perf_counter()
            ocr = self.ocr_matcher.combine(
                [self._ocr_of(crop) for crop in crops[: self.config.ocr_max_crops]]
            )
            latency["ocr"] = (time.perf_counter() - start) * 1000.0
        barcode = None
        if (
            self.barcode_enabled
            and self.barcode_backend is not None
            and self.gtin_to_product
            and len(barcode_images)
        ):
            start = time.perf_counter()
            barcode = read_gtin(barcode_images, self.barcode_backend, self.gtin_to_product)
            latency["barcode"] = (time.perf_counter() - start) * 1000.0
        return Observation(match, ocr, barcode, latency)

    def _ocr_of(self, crop: np.ndarray) -> OcrAnswer:
        """OCR one crop at each configured rotation; the first rotation that names a product wins.

        Two rotations naming different products is a conflict, which the fusion abstains on, so
        a rotation is only tried further when the previous ones named nothing.
        """
        assert self.ocr_backend is not None and self.ocr_matcher is not None
        answers = []
        for degrees in self.config.ocr_rotations:
            answer = self.ocr_matcher.names(self.ocr_backend.read(rotate_image(crop, degrees)))
            answers.append(answer)
            if answer.product_id is not None or answer.candidates:
                break
        return self.ocr_matcher.combine(answers)

    # --- deciding ----------------------------------------------------------------------------

    def identify(
        self, crops: Sequence[np.ndarray], barcode_images: Sequence[np.ndarray] = ()
    ) -> IdentityResult:
        return self.decide(self.observe(crops, barcode_images))

    def decide(self, observation: Observation) -> IdentityResult:
        return decide(observation, self.config, self.provenance())


def _is_gtin(value: str) -> bool:
    try:
        validate_gtin13(value)
    except InvalidGtin:
        return False
    return True


def _rank(match: MatchResult | None, product_id: str) -> int | None:
    if match is None:
        return None
    for position, candidate in enumerate(match.ranked, start=1):
        if candidate.product_id == product_id:
            return position
    return None


def evidence_of(observation: Observation) -> list[Evidence]:
    out: list[Evidence] = []
    match = observation.match
    if match is not None and match.top1 is not None:
        out.append(
            Evidence(
                "embedding",
                match.top1.product_id,
                match.top1.score,
                {
                    "margin": match.margin,
                    "vote_agreement": match.vote_agreement,
                    "n_views": match.n_queries,
                    "runner_up": match.top2.product_id if match.top2 else None,
                },
            )
        )
    ocr = observation.ocr
    if ocr is not None and (ocr.product_id or ocr.candidates):
        out.append(
            Evidence(
                "ocr",
                ocr.product_id,
                float(len(ocr.matched)),
                {"matched": list(ocr.matched), "candidates": list(ocr.candidates)},
            )
        )
    barcode = observation.barcode
    if barcode is not None and (barcode.valid or barcode.rejected):
        out.append(
            Evidence(
                "barcode",
                barcode.product_id,
                1.0 if barcode.gtin else 0.0,
                {
                    "gtin": barcode.gtin,
                    "registered": barcode.registered,
                    "valid": list(barcode.valid),
                    "rejected": barcode.rejected,
                },
            )
        )
    return out


def decide(
    observation: Observation, config: IdentityConfig, provenance: dict[str, Any]
) -> IdentityResult:
    """Apply the fusion rules to one observation (pure; the calibration reuses it)."""
    evidence = tuple(evidence_of(observation))

    def result(status: str, sku: str | None, reason: str) -> IdentityResult:
        return IdentityResult(
            sku, status, reason, evidence, provenance, observation.latency_ms, observation.channels
        )

    match, ocr, barcode = observation.match, observation.ocr, observation.barcode

    if barcode is not None and barcode.conflict:
        return result(ABSTAIN, None, "barcode images decode to different products")
    if barcode is not None and barcode.gtin is not None and not barcode.registered:
        return result(UNKNOWN, None, "barcode is a valid GTIN that is not registered")
    if ocr is not None and ocr.conflict:
        return result(ABSTAIN, None, "OCR names more than one product")

    named: tuple[str, str] | None = None  # (product id, channel)
    if barcode is not None and barcode.product_id is not None:
        if ocr is not None and ocr.product_id not in (None, barcode.product_id):
            return result(ABSTAIN, None, "barcode and OCR name different products")
        named = (barcode.product_id, "barcode")
    elif ocr is not None and ocr.product_id is not None:
        named = (ocr.product_id, "ocr")

    if named is not None:
        product_id, channel = named
        rank = _rank(match, product_id)
        if match is not None and (rank is None or rank > config.supported_max_rank):
            return result(
                ABSTAIN, None, f"{channel} names {product_id}, embedding ranks it {rank}"
            )
        return result(IDENTIFIED, product_id, f"{channel}" + (" and embedding" if match else ""))

    if match is not None and match.top1 is not None:
        margin = match.margin if match.margin is not None else 1.0
        if (
            config.alone_enabled
            and match.top1.score >= config.alone_min_score  # type: ignore[operator]
            and match.top1.score >= config.product_floors.get(match.top1.product_id, 0.0)
            and margin >= config.alone_min_margin  # type: ignore[operator]
            and match.vote_agreement >= config.alone_min_vote  # type: ignore[operator]
        ):
            return result(IDENTIFIED, match.top1.product_id, "embedding alone")
        floor = config.unknown_below_score
        if floor is not None and match.top1.score < floor:
            return result(UNKNOWN, None, "best match below the unknown floor")
    return result(ABSTAIN, None, "no channel names a product")
