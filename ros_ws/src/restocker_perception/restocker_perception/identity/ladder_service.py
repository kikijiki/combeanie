# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The identify-object (ladder) service logic, with no ROS: looks in, a plain reply dict out."""
# Mirrors service.py: the node only converts messages, so the sidecar-absent behaviour, the
# per-object state and the reply contract are tested without a ROS graph. The state kept per
# object key is what each look measured (embeddings, label tokens, a barcode read, barrel
# signatures), never an image, and is bounded (the least recently used keys are forgotten).

from __future__ import annotations

from collections import OrderedDict
from collections.abc import Mapping, Sequence
import hashlib
import json
import logging
import math
from pathlib import Path
from typing import Any

import numpy as np

from restocker_perception.identity.barcode import BarcodeBackend
from restocker_perception.identity.benchmark import load_whitener
from restocker_perception.identity.ladder import (
    IDENTIFIED,
    NEEDS_HUMAN,
    NOT_YET,
    UNKNOWN,
    LadderConfig,
    LadderEngine,
    LadderResult,
    LookCrop,
    ObjectState,
    build_index,
)
from restocker_perception.identity.service import image_to_rgb  # noqa: F401 - re-exported
from restocker_perception.registry.backends import EmbeddingBackend, OcrBackend
from restocker_perception.registry.model import Registry

STATUS_CODES = {IDENTIFIED: 0, NOT_YET: 1, NEEDS_HUMAN: 2, UNKNOWN: 3}
_LOG = logging.getLogger(__name__)
MAX_OBJECTS = 256


def _reply(
    status: str,
    *,
    available: bool,
    reason: str,
    result: LadderResult | None = None,
    product_gtins: Mapping[str, str] | None = None,
    provenance: dict[str, Any] | None = None,
    config_sha256: str = "",
    barcode_enabled: bool = True,
    channels: Sequence[str] = (),
) -> dict[str, Any]:
    gtins = product_gtins or {}
    provenance = provenance or {}
    sku = result.sku if result else None
    guidance = result.guidance if result else None
    text = result.text if result else {}
    nan = math.nan
    return {
        "status": STATUS_CODES[status],
        "sidecar_available": available,
        "has_sku": sku is not None,
        "sku": sku or "",
        "has_gtin": bool(gtins.get(sku or "")),
        "gtin": gtins.get(sku or "", ""),
        "rung": (result.rung or "") if result else "",
        "reason": reason,
        "look_index": result.look if result else 0,
        "guidance_kind": guidance.kind if guidance else "",
        "guidance_shape": guidance.shape if guidance else "",
        "has_facing": bool(guidance and guidance.estimated_facing_deg is not None),
        "estimated_facing_deg": float(
            guidance.estimated_facing_deg
            if guidance and guidance.estimated_facing_deg is not None
            else nan
        ),
        "visible_face": (guidance.visible_face or "") if guidance else "",
        "has_turn": bool(guidance and guidance.suggested_turn_deg is not None),
        "suggested_turn_deg": float(
            guidance.suggested_turn_deg
            if guidance and guidance.suggested_turn_deg is not None
            else nan
        ),
        "target_azimuth_deg": float(
            guidance.target_azimuth_deg
            if guidance and guidance.target_azimuth_deg is not None
            else nan
        ),
        "target_face": (guidance.target_face or "") if guidance else "",
        "facing_confidence": float(
            guidance.facing_confidence
            if guidance and guidance.facing_confidence is not None
            else nan
        ),
        "facing_low_confidence": bool(guidance.low_confidence) if guidance else False,
        "guidance_candidate_sku": (guidance.candidate or "") if guidance else "",
        "guidance_note": guidance.note if guidance else "",
        "text_best_sku": text.get("best") or "",
        "text_best_score": float(text.get("best_score") or 0.0),
        "text_margin": float(text.get("margin") or 0.0),
        "text_matched": list(text.get("best_matched") or []),
        "shortlist": list(result.shortlist) if result else [],
        "channels_used": list(channels),
        "barcode_enabled": barcode_enabled,
        "backend_name": provenance.get("backend_name") or "",
        "backend_version": provenance.get("backend_version") or "",
        "registry_sha256": provenance.get("registry_sha256") or "",
        "config_sha256": config_sha256,
        "latency_ms": float(sum(result.latency_ms.values())) if result else 0.0,
    }


class IdentifyObjectService:
    """Wraps a :class:`LadderEngine`; or, with none, the sidecar-absent answer."""

    def __init__(
        self,
        engine: LadderEngine | None,
        *,
        config_sha256: str = "",
        unavailable_reason: str = "identity sidecar not configured",
    ) -> None:
        self.engine = engine
        self.config_sha256 = config_sha256
        self.unavailable_reason = unavailable_reason
        self.states: OrderedDict[str, ObjectState] = OrderedDict()
        self.product_gtins = (
            {p: g for g, p in engine.registry.gtin_map().items()} if engine is not None else {}
        )

    @classmethod
    def load(
        cls,
        registry_path: str,
        whitener_path: str,
        config_path: str,
        embedding: EmbeddingBackend,
        ocr: OcrBackend,
        *,
        barcode_backend: BarcodeBackend | None = None,
        barcode_enabled: bool = True,
    ) -> IdentifyObjectService:
        """Build the engine; any failure yields the unavailable service, never an exception."""
        try:
            registry = Registry.load(Path(registry_path))
            whitener = load_whitener(whitener_path)
            data = json.loads(Path(config_path).read_text(encoding="utf-8")) if config_path else {}
            digest = hashlib.sha256(json.dumps(data, sort_keys=True).encode()).hexdigest()
            config = LadderConfig.from_dict(data)
            index = build_index(registry, embedding, whitener if config.whiten else None)
            engine = LadderEngine(
                registry,
                index,
                embedding,
                ocr,
                config,
                barcode_backend=barcode_backend,
                barcode_enabled=barcode_enabled,
            )
        except Exception as error:  # noqa: BLE001 - every failure means "no sidecar"
            _LOG.warning("identity ladder sidecar unavailable: %s", error)
            return cls(None, unavailable_reason=f"identity sidecar unavailable: {error}")
        return cls(engine, config_sha256=digest)

    @property
    def available(self) -> bool:
        return self.engine is not None

    def reject(self, reason: str) -> dict[str, Any]:
        return _reply(
            NOT_YET, available=self.available, reason=reason, config_sha256=self.config_sha256
        )

    def identify(
        self,
        object_key: str,
        crops: Sequence[np.ndarray],
        half_widths: Sequence[float] = (),
        camera_azimuths: Sequence[float] = (),
        barcode_images: Sequence[np.ndarray] = (),
        *,
        reset: bool = False,
        max_relooks: int = -1,
    ) -> dict[str, Any]:
        if self.engine is None:
            return _reply(UNKNOWN, available=False, reason=self.unavailable_reason)
        engine = self.engine
        try:
            if reset or object_key not in self.states:
                self.states[object_key] = ObjectState()
            self.states.move_to_end(object_key)
            while len(self.states) > MAX_OBJECTS:
                self.states.popitem(last=False)
            state = self.states[object_key]
            look_crops = [
                LookCrop(
                    image,
                    float(half_widths[i]) if i < len(half_widths) and half_widths[i] > 0 else None,
                    float(camera_azimuths[i])
                    if i < len(camera_azimuths) and math.isfinite(camera_azimuths[i])
                    else None,
                )
                for i, image in enumerate(crops)
            ]
            state.looks.append(engine.observe(look_crops, barcode_images))
            budget = engine.config.max_relooks if max_relooks < 0 else max_relooks
            result = engine.decide_escalating(
                state, look_crops, relooks_left=budget - (len(state.looks) - 1)
            )
        except Exception as error:  # noqa: BLE001 - a failing model must not take the node down
            _LOG.exception("identity ladder failed")
            return _reply(
                UNKNOWN,
                available=False,
                reason=f"identity ladder failed: {error}",
                config_sha256=self.config_sha256,
            )
        if result.status in (IDENTIFIED, UNKNOWN, NEEDS_HUMAN):
            self.states.pop(object_key, None)  # the ladder is over for this object
        channels = (
            ["embedding", "text"]
            + (["text_escalated"] if result.escalated else [])
            + (["barcode"] if engine.barcode_enabled else [])
        )
        return _reply(
            result.status,
            available=True,
            reason=result.reason,
            result=result,
            product_gtins=self.product_gtins,
            provenance=engine.index.provenance(),
            config_sha256=self.config_sha256,
            barcode_enabled=engine.barcode_enabled,
            channels=channels,
        )
