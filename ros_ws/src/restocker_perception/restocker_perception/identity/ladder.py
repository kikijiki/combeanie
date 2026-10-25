# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The identity escalation ladder (CMB-112 round 3): cheap evidence first, then look again."""
# One object, one or more looks. Each look is a handful of crops (and, optionally, close-ups of the
# barcode panel). What the looks measured accumulates; after every look the ladder decides:
#
#   1. barcode     a valid GTIN of a registered product (if the product has one and the channel is
#                  on): exact, done.
#   2. label text  the reader's tokens scored against every registered product's label text; one
#                  product is named only if it clears a minimum score AND a clear margin over the
#                  runner-up (the tie-breaker for look-alikes), and the embedding shortlist
#                  contains it.
#   3. not yet     neither: say which side is visible (round body: the facing angle estimated from
#                  the visible strip; box: which face) and how far to turn the product, or where to
#                  look from, to show the side that carries text.
#   4. needs human the look budget is spent and nothing named the product. Never a guess.
#
# Round 4 (CMB-121) adds an escalation rung between 2 and 3, off by default: when the cheap reader
# left the object unresolved at this look, a heavier pretrained reader (a vision-language model,
# seconds per crop on a CPU) reads the look's largest crops once, and the label text is scored
# again over that look's heavy tokens (they replace the cheap reader's tokens of the look: the
# cheap tokens of an object it could not name are mostly garbage) and the earlier looks' tokens,
# under the same margin rule, contradiction rule and embedding shortlist. It has its own latency
# budget, counted apart from the per-look one.
#
# The embedding is a retrieval prior (the shortlist, and where on the turntable a crop sits); it
# never names a product by itself. Nothing is trained: every number is either fixed beforehand in
# :class:`LadderConfig` or read from the registry.

from __future__ import annotations

from collections.abc import Collection, Sequence
from dataclasses import asdict, dataclass, field
import math
import time
from typing import Any

import numpy as np

from restocker_perception.identity import facing as facing_module
from restocker_perception.identity.barcode import BarcodeBackend, BarcodeRead, read_gtin
from restocker_perception.identity.labeltext import (
    LabelTextIndex,
    TextMatch,
    TextPolicy,
    TextScore,
)
from restocker_perception.identity.textreader import (
    ObjectTextReader,
    TextReaderConfig,
    UpscaledOcr,
    read_positioned,
    registry_view_text_scores,
)
from restocker_perception.registry.backends import (
    EmbeddingBackend,
    Florence2OcrBackend,
    OcrBackend,
)
from restocker_perception.registry.cache import DerivedCache
from restocker_perception.registry.model import Registry
from restocker_perception.registry.retrieval import MatchResult, RegistryIndex, Whitener

IDENTIFIED = "identified"
NOT_YET = "not_identified_yet"
NEEDS_HUMAN = "needs_human"
UNKNOWN = "unknown"
LADDER_STATUSES = (IDENTIFIED, NOT_YET, NEEDS_HUMAN, UNKNOWN)

RUNG_BARCODE = "barcode"
RUNG_TEXT = "text"
RUNG_TEXT_ESCALATED = "text_escalated"


@dataclass(frozen=True)
class EscalationConfig:
    """The escalation rung: a heavier text reader for objects the cheap ladder left open."""

    # Off by default: the fallback is the round-3 ladder, bit for bit.
    enabled: bool = False
    # Which pretrained reader (see ``make_escalation_backend``), its ONNX/torch threads and, for a
    # generative reader, how many tokens it may write per crop.
    backend: str = "florence2"
    # The checkpoint revision to load ("main" = whatever the hub serves; pin a commit hash).
    revision: str = "main"
    threads: int = 4
    max_new_tokens: int = 96
    # Crops of a look read by the heavy reader (the largest ones).
    crops: int = 2
    # How the heavy reader's crops are cut and its lines filtered: the object's band, enlarged
    # twice (the form measured on dev), not unrolled (the unrolled strip did not help any reader).
    reader: TextReaderConfig = field(
        default_factory=lambda: TextReaderConfig(
            min_line_score=0.5, max_line_words=3, upscale=2, unrolled=False
        )
    )


@dataclass(frozen=True)
class LadderConfig:
    """Everything the ladder decides with. Fixed beforehand; none is fitted to held-out data."""

    text: TextPolicy = field(default_factory=lambda: TextPolicy(0.5, 0.25, 2, 0))
    escalation: EscalationConfig = field(default_factory=EscalationConfig)
    # How the reader cuts, enlarges and filters what it reads (textreader.TextReaderConfig).
    reader: TextReaderConfig = field(default_factory=TextReaderConfig)
    # The embedding shortlist: a product named by text or barcode must rank at or above this.
    shortlist_rank: int = 3
    scoring: str = "max"
    whiten: bool = True
    # Looks after the first one that may be asked for before the object is left to a human.
    max_relooks: int = 3
    # Crops embedded per look; crops read by the OCR per look (the largest ones).
    max_views: int = 5
    ocr_crops: int = 3
    # A text turn smaller than this means "already facing the text": try the neighbouring side.
    min_turn_deg: float = 22.5
    # A facing estimate is trusted to this z-score gap between its peak and the best other angle
    # (below it the suggestion is still made, flagged low confidence).
    facing_confidence: float = 0.5

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> LadderConfig:
        data = dict(data)
        text = TextPolicy(**data.pop("text")) if "text" in data else TextPolicy(0.5, 0.25, 2, 0)
        reader = TextReaderConfig(**data.pop("reader")) if "reader" in data else TextReaderConfig()
        escalation = EscalationConfig()
        if "escalation" in data:
            raw = dict(data.pop("escalation"))
            if "reader" in raw:
                raw["reader"] = TextReaderConfig(**raw["reader"])
            escalation = EscalationConfig(**raw)
        return cls(text=text, reader=reader, escalation=escalation, **data)


@dataclass(frozen=True)
class LookCrop:
    """One crop of the object, with what the caller knows about how it was taken."""

    image: np.ndarray
    # Half width of the object's body in the crop, pixels (from the detector's footprint).
    half_width_px: float | None = None
    # Direction of the camera seen from the object, degrees, in any fixed frame (world).
    camera_azimuth_deg: float | None = None


@dataclass
class LookRecord:
    """What one look measured. Everything a later decision needs; no image is kept."""

    embeddings: np.ndarray  # (n, dim) raw backend embeddings
    tokens: list[str]  # label-text tokens of the crops read
    barcode: BarcodeRead | None
    signatures: list[np.ndarray | None]  # barrel signatures, one per crop
    camera_azimuths: list[float | None]
    latency_ms: dict[str, float]
    # Registered label words the reader found on the barrel, where: (word weight, world azimuth of
    # the text = camera azimuth + its angle from the line of sight). Round bodies only.
    sightings: list[tuple[float, float]] = field(default_factory=list)
    # Tokens the escalation reader found on this look's crops (None = this look was not escalated)
    # and what reading them cost.
    escalation_tokens: list[str] | None = None
    escalation_ms: float = 0.0


@dataclass
class ObjectState:
    """The looks of one object so far (the ladder's only state; the caller keeps it)."""

    looks: list[LookRecord] = field(default_factory=list)

    @property
    def tokens(self) -> list[str]:
        return self.tokens_with(None)

    def tokens_with(self, escalated_looks: Collection[int] | None) -> list[str]:
        """Distinct tokens of every look; an escalated look contributes the heavy reader's own.

        The heavy reader's tokens *replace* the cheap reader's on the looks in ``escalated_looks``
        (``None`` = every look that was escalated): on the dev data the cheap reader's tokens of an
        object it could not name carry more garbage than information (a union named 7 of the 42
        open objects, the heavy tokens alone 18), and the contradiction rule reads every token.
        The benchmark passes the looks an arm escalated, so two arms that share a measurement
        still keep their own books.
        """
        seen: list[str] = []
        for index, look in enumerate(self.looks):
            heavy = look.escalation_tokens
            if heavy is not None and (escalated_looks is None or index in escalated_looks):
                own = heavy
            else:
                own = look.tokens
            for token in own:
                if token not in seen:
                    seen.append(token)
        return seen


@dataclass(frozen=True)
class Guidance:
    """What to do about an object the ladder could not name yet."""

    kind: str  # "turn" or "new_view"
    shape: str  # "round" or "box"
    estimated_facing_deg: float | None
    visible_face: str | None
    suggested_turn_deg: float | None  # counter-clockwise seen from above; box: a multiple of 90
    target_azimuth_deg: float | None
    target_face: str | None
    facing_confidence: float | None
    low_confidence: bool
    candidate: str | None  # the product the estimate assumed (the embedding's top-1)
    note: str

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass(frozen=True)
class LadderResult:
    status: str
    sku: str | None
    rung: str | None
    reason: str
    look: int  # index of the look this decision followed (0 = the first)
    text: dict[str, Any]
    shortlist: tuple[str, ...]
    guidance: Guidance | None = None
    barcode: str | None = None  # the GTIN read, if any
    latency_ms: dict[str, float] = field(default_factory=dict)
    # True when the escalation reader's tokens were part of this decision.
    escalated: bool = False

    @property
    def identified(self) -> bool:
        return self.status == IDENTIFIED

    def to_dict(self) -> dict[str, Any]:
        out = asdict(self)
        out["shortlist"] = list(self.shortlist)
        return out


def make_escalation_backend(config: EscalationConfig) -> OcrBackend | None:
    """The escalation reader the config names (None when the rung is off)."""
    if not config.enabled:
        return None
    if config.backend == "florence2":
        return Florence2OcrBackend(
            revision=config.revision,
            threads=config.threads,
            max_new_tokens=config.max_new_tokens,
        )
    raise ValueError(f"unknown escalation backend {config.backend!r}")


def _unit(matrix: np.ndarray) -> np.ndarray:
    norms = np.linalg.norm(matrix, axis=-1, keepdims=True)
    return (matrix / np.maximum(norms, 1e-12)).astype(np.float32)


class LadderEngine:
    """Registry, backends and policy; crops in, :class:`LadderResult` out."""

    def __init__(
        self,
        registry: Registry,
        index: RegistryIndex,
        embedding: EmbeddingBackend,
        ocr: OcrBackend,
        config: LadderConfig,
        *,
        barcode_backend: BarcodeBackend | None = None,
        barcode_enabled: bool = True,
        text_azimuths: facing_module.TextAzimuths | None = None,
        reader_config: TextReaderConfig | None = None,
        cache: DerivedCache | None = None,
        escalation_ocr: OcrBackend | None = None,
    ) -> None:
        self.registry = registry
        self.index = index
        self.embedding = embedding
        self.config = config
        self.barcode_backend = barcode_backend
        self.barcode_enabled = barcode_enabled
        self.reader = ObjectTextReader(ocr, reader_config or config.reader)
        # The escalation rung exists only if the config switches it on AND a reader was given.
        if config.escalation.enabled and escalation_ocr is None:
            escalation_ocr = make_escalation_backend(config.escalation)
        self.escalation_reader = (
            ObjectTextReader(escalation_ocr, config.escalation.reader)
            if config.escalation.enabled and escalation_ocr is not None
            else None
        )
        records = {p: registry.get(p) for p in index.product_ids}
        self.records = records
        self.text_index = LabelTextIndex({p: r.ocr_strings for p, r in records.items()})
        self.gtin_to_product = {g: p for g, p in registry.gtin_map().items() if p in records}
        self._cache = cache or DerivedCache(registry.root / ".cache")
        self._panoramas: dict[str, np.ndarray] = {}
        self.text_azimuths = text_azimuths or self._registry_text_azimuths(ocr)

    # --- registry-derived data -------------------------------------------------------------

    def _registry_text_azimuths(self, ocr: OcrBackend) -> facing_module.TextAzimuths:
        """How much label text each registered view carries, by the reader's own account."""
        enlarged = UpscaledOcr(ocr, 3)
        scores = {}
        for product_id in self.records:
            reads = self._cache.ocr_reads(self.registry, enlarged, product_id)
            scores[product_id] = registry_view_text_scores(
                reads,
                self.text_index.words_of(product_id),
                self.text_index.weight,
                self.text_index,
            )
        return facing_module.TextAzimuths(scores)

    def _panorama(self, product_id: str) -> np.ndarray:
        if product_id not in self._panoramas:
            self._panoramas[product_id] = facing_module.panorama(
                self.registry.view_images(product_id)
            )
        return self._panoramas[product_id]

    def _registered_weight(self, text: str) -> float:
        """The weight of the heaviest registered label word the line reads (0 if none)."""
        tokens = self.text_index.tokens_of([text])
        return max(
            (
                self.text_index.weight(w)
                for product in self.text_index.product_ids
                for w in self.text_index.words_of(product)
                if self.text_index.reads(w, tokens)
            ),
            default=0.0,
        )

    def is_box(self, product_id: str) -> bool:
        return self.records[product_id].size_xyz_m is not None

    # --- escalation ----------------------------------------------------------------------------

    @property
    def can_escalate(self) -> bool:
        return self.escalation_reader is not None

    def escalate(self, look: LookRecord, crops: Sequence[LookCrop]) -> None:
        """Read the look's largest crops with the escalation reader (once per look)."""
        if self.escalation_reader is None or look.escalation_tokens is not None:
            return
        crops = list(crops)[: self.config.max_views]
        biggest = sorted(
            range(len(crops)), key=lambda i: -(crops[i].half_width_px or crops[i].image.shape[1])
        )[: self.config.escalation.crops]
        start = time.perf_counter()
        lines: list[str] = []
        for i in sorted(biggest):
            crop = crops[i]
            lines += self.escalation_reader.read(crop.image, crop.half_width_px)
        look.escalation_tokens = self.text_index.tokens_of(lines)
        look.escalation_ms = (time.perf_counter() - start) * 1000.0

    # --- measuring ---------------------------------------------------------------------------

    def observe(
        self, crops: Sequence[LookCrop], barcode_images: Sequence[np.ndarray] = ()
    ) -> LookRecord:
        """Measure one look: embed the crops, read the largest ones, decode any barcode."""
        crops = list(crops)[: self.config.max_views]
        latency: dict[str, float] = {}
        start = time.perf_counter()
        embeddings = (
            self.embedding.embed([c.image for c in crops])
            if crops
            else np.zeros((0, self.embedding.dim), dtype=np.float32)
        )
        latency["embedding"] = (time.perf_counter() - start) * 1000.0
        start = time.perf_counter()
        biggest = sorted(
            range(len(crops)), key=lambda i: -(crops[i].half_width_px or crops[i].image.shape[1])
        )[: self.config.ocr_crops]
        lines: list[str] = []
        sightings: list[tuple[float, float]] = []
        for i in sorted(biggest):
            crop = crops[i]
            if crop.half_width_px:
                texts, seen = read_positioned(self.reader, crop.image, crop.half_width_px)
                lines += texts
                for sight in seen:
                    weight = self._registered_weight(sight.text)
                    if weight > 0:
                        sightings.append(
                            (weight, (crop.camera_azimuth_deg or 0.0) + sight.theta_deg)
                        )
            else:
                lines += self.reader.read(crop.image, None)
        tokens = self.text_index.tokens_of(lines)
        latency["ocr"] = (time.perf_counter() - start) * 1000.0
        start = time.perf_counter()
        signatures = [
            facing_module.barrel_signature(c.image, c.half_width_px) if c.half_width_px else None
            for c in crops
        ]
        latency["facing"] = (time.perf_counter() - start) * 1000.0
        barcode = None
        if self.barcode_backend is not None and self.gtin_to_product and len(barcode_images):
            start = time.perf_counter()
            barcode = read_gtin(barcode_images, self.barcode_backend, self.gtin_to_product)
            latency["barcode"] = (time.perf_counter() - start) * 1000.0
        return LookRecord(
            embeddings,
            tokens,
            barcode,
            signatures,
            [c.camera_azimuth_deg for c in crops],
            latency,
            sightings,
        )

    # --- deciding ----------------------------------------------------------------------------

    def _match(self, state: ObjectState) -> MatchResult | None:
        stack = [look.embeddings for look in state.looks if len(look.embeddings)]
        if not stack:
            return None
        return self.index.match(np.concatenate(stack), scoring=self.config.scoring)

    def decide(
        self,
        state: ObjectState,
        *,
        barcode: bool | None = None,
        guided: bool = True,
        relooks_left: int | None = None,
        escalated_looks: Collection[int] | None = None,
    ) -> LadderResult:
        """Decide after the looks in ``state``.

        ``barcode`` overrides the engine's switch for this call (the two benchmark arms share one
        measurement); ``guided=False`` replaces the estimated turn with a fixed quarter turn (the
        ablation that shows what the guidance is worth); ``relooks_left`` (default: what the
        budget leaves after ``len(state.looks) - 1`` looks) of zero turns "not yet" into "needs
        a human"; ``escalated_looks`` names the looks whose escalation tokens count (default:
        every look that was escalated).
        """
        use_barcode = self.barcode_enabled if barcode is None else barcode
        look = len(state.looks) - 1
        if relooks_left is None:
            relooks_left = self.config.max_relooks - look
        match = self._match(state)
        shortlist = (
            tuple(m.product_id for m in match.top_k(self.config.shortlist_rank)) if match else ()
        )
        latency = {
            k: sum(look_.latency_ms.get(k, 0.0) for look_ in state.looks)
            for k in ("embedding", "ocr", "facing", "barcode")
        }
        if not use_barcode:
            latency.pop("barcode", None)
        used = [
            i
            for i, look_ in enumerate(state.looks)
            if look_.escalation_tokens is not None
            and (escalated_looks is None or i in escalated_looks)
        ]
        if used:
            latency["escalation"] = sum(state.looks[i].escalation_ms for i in used)
        text_match = self.text_index.score(state.tokens_with(escalated_looks))
        text_summary = _text_summary(text_match)

        def result(status: str, sku: str | None, rung: str | None, reason: str, **extra: Any):
            return LadderResult(
                status,
                sku,
                rung,
                reason,
                look,
                text_summary,
                shortlist,
                latency_ms=latency,
                escalated=bool(used),
                **extra,
            )

        # Rung 1: the barcode.
        code = None
        if use_barcode:
            code = next((lk.barcode for lk in reversed(state.looks) if lk.barcode), None)
            if code is not None and code.gtin is not None:
                if not code.registered:
                    return result(
                        UNKNOWN,
                        None,
                        RUNG_BARCODE,
                        "a valid GTIN that no registered product has",
                        barcode=code.gtin,
                    )
                named = self.config.text.names(text_match)
                if named is not None and named.product_id != code.product_id:
                    return result(
                        NOT_YET if relooks_left > 0 else NEEDS_HUMAN,
                        None,
                        None,
                        "barcode and label text name different products",
                        barcode=code.gtin,
                        guidance=None,
                    )
                if code.product_id in shortlist:
                    return result(
                        IDENTIFIED, code.product_id, RUNG_BARCODE, "barcode", barcode=code.gtin
                    )
        # Rung 2: the label text, with a margin, inside the embedding shortlist.
        named = self.config.text.names(text_match)
        if named is not None and named.product_id in shortlist:
            cheap = self.config.text.names(self.text_index.score(state.tokens_with(())))
            rung = (
                RUNG_TEXT
                if cheap is not None and cheap.product_id == named.product_id
                else (RUNG_TEXT_ESCALATED if used else RUNG_TEXT)
            )
            return result(IDENTIFIED, named.product_id, rung, "label text with margin")
        # Rung 3 / 4.
        if relooks_left <= 0:
            return result(NEEDS_HUMAN, None, None, "look budget spent, nothing named the product")
        guidance = self.guide(state, text_match, match, guided=guided)
        return result(NOT_YET, None, None, "no rung named the product", guidance=guidance)

    def decide_escalating(
        self,
        state: ObjectState,
        crops: Sequence[LookCrop],
        *,
        escalated: set[int] | None = None,
        **kwargs: Any,
    ) -> LadderResult:
        """``decide``, and when the cheap rungs left the object open, escalate this look once.

        ``escalated`` is the set of looks this decision stream has escalated; it is updated in
        place (a caller with one stream per arm keeps one set per arm). Without it the looks that
        carry escalation tokens are the escalated ones.
        """
        track = escalated if escalated is not None else set()
        if escalated is None:
            track.update(i for i, lk in enumerate(state.looks) if lk.escalation_tokens is not None)
        result = self.decide(state, escalated_looks=track, **kwargs)
        look = len(state.looks) - 1
        if (
            self.can_escalate
            and result.status in (NOT_YET, NEEDS_HUMAN)
            and result.rung is None
            and look not in track
        ):
            self.escalate(state.looks[-1], crops)
            track.add(look)
            result = self.decide(state, escalated_looks=track, **kwargs)
        return result

    # --- guidance ----------------------------------------------------------------------------

    def guide(
        self,
        state: ObjectState,
        text_match: TextMatch,
        match: MatchResult | None,
        *,
        guided: bool = True,
    ) -> Guidance:
        """Which side the last look showed, and how to turn the product to show text."""
        candidate = self._candidate(text_match, match)
        if candidate is None or not state.looks or not len(state.looks[-1].embeddings):
            return Guidance(
                "new_view",
                "round",
                None,
                None,
                None,
                None,
                None,
                None,
                True,
                None,
                "nothing to estimate a facing from; look from another side",
            )
        box = self.is_box(candidate)
        facings = [self._facing(candidate, look, box) for look in state.looks]
        facing = facings[-1]
        if facing is None:
            return Guidance(
                "new_view",
                "box" if box else "round",
                None,
                None,
                None,
                None,
                None,
                None,
                True,
                candidate,
                "no crop to estimate from",
            )
        # The sides earlier looks showed (their estimated facing, in the registry's azimuths).
        seen = [f.azimuth_deg for f in facings[:-1] if f is not None]
        if not box and guided:
            servo = self._servo(state.looks[-1], facings[-1])
            if servo is not None:
                return Guidance(
                    "turn",
                    "round",
                    facing.azimuth_deg,
                    None,
                    servo,
                    None,
                    None,
                    facing.confidence,
                    facing.confidence < self.config.facing_confidence,
                    candidate,
                    "label text seen off-centre: turn to centre it",
                )
        target = self.text_azimuths.target(candidate, box=box, seen=seen)
        turn = facing_module.suggest_turn(facing, target)
        degrees = turn.degrees
        note = "turn to show the unseen side with the most registered label text"
        if abs(degrees) < self.config.min_turn_deg:
            # The text side already faces the camera and was not read: show the next best side.
            target = self.text_azimuths.target(
                candidate, box=box, seen=[*seen, facing.azimuth_deg]
            )
            turn = facing_module.suggest_turn(facing, target)
            degrees = turn.degrees
            note = "text side already toward the camera; show the next best side"
        if not guided:
            degrees = 90.0
            note = "blind quarter turn (no estimate used)"
        low = facing.confidence < self.config.facing_confidence
        return Guidance(
            "turn",
            facing.shape,
            facing.azimuth_deg,
            facing.visible_face,
            degrees,
            turn.target_azimuth_deg,
            turn.target_face,
            facing.confidence,
            low,
            candidate,
            note,
        )

    def _servo(self, look: LookRecord, facing: facing_module.Facing) -> float | None:
        """The turn that centres the registered text the reader saw off-centre, if it did.

        Text at world azimuth ``b`` is centred when the product is turned by ``cam - b``; the
        sightings are weighted by their word's rarity and averaged round the circle, each
        relative to the first crop's camera. Nothing is returned when the text already sits within
        ``min_turn_deg`` of the line of sight (it is centred and still unread: look elsewhere).
        """
        if not look.sightings:
            return None
        reference = (
            look.camera_azimuths[0]
            if look.camera_azimuths and look.camera_azimuths[0] is not None
            else 0.0
        )
        weights = np.array([w for w, _ in look.sightings])
        angles = np.radians([a - reference for _, a in look.sightings])
        mean = math.degrees(
            math.atan2((weights * np.sin(angles)).sum(), (weights * np.cos(angles)).sum())
        )
        turn = -mean
        return float(turn) if abs(turn) >= self.config.min_turn_deg else None

    def _facing(self, candidate: str, look: LookRecord, box: bool) -> facing_module.Facing | None:
        """The facing one look showed, assuming the object is ``candidate``."""
        if not len(look.embeddings):
            return None
        queries = _unit(
            self.index.whitener.apply(look.embeddings) if self.index.whitener else look.embeddings
        )
        row = self.index.product_ids.index(candidate)
        sims = queries @ self.index.exemplars[row].T  # (n crops, 8 registered views)
        views = [
            facing_module.ViewEvidence(
                tuple(float(x) for x in sims[i]), look.signatures[i], look.camera_azimuths[i]
            )
            for i in range(len(sims))
        ]
        return facing_module.estimate_facing(
            views, None if box else self._panorama(candidate), box=box
        )

    def _candidate(self, text_match: TextMatch, match: MatchResult | None) -> str | None:
        if match is not None and match.top1 is not None:
            return match.top1.product_id
        return text_match.best.product_id if text_match.best else None


def _text_summary(match: TextMatch) -> dict[str, Any]:
    best, runner = match.best, match.runner_up
    return {
        "best": best.product_id if best else None,
        "best_score": best.score if best else None,
        "best_matched": list(best.matched) if best else [],
        "runner_up": runner.product_id if runner else None,
        "runner_up_score": runner.score if runner else None,
        "margin": match.margin,
        "n_tokens": match.n_tokens,
    }


def build_index(
    registry: Registry,
    embedding: EmbeddingBackend,
    whitener: Whitener | None,
    *,
    exclude: Sequence[str] = (),
    cache: DerivedCache | None = None,
) -> RegistryIndex:
    return RegistryIndex.from_registry(
        registry, embedding, cache, whitener=whitener, exclude=exclude
    )


__all__ = [
    "IDENTIFIED",
    "RUNG_TEXT_ESCALATED",
    "EscalationConfig",
    "LADDER_STATUSES",
    "NEEDS_HUMAN",
    "NOT_YET",
    "UNKNOWN",
    "Guidance",
    "LadderConfig",
    "LadderEngine",
    "LadderResult",
    "LookCrop",
    "LookRecord",
    "ObjectState",
    "TextScore",
    "build_index",
    "make_escalation_backend",
]
