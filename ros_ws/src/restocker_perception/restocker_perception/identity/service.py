# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""The identify-product service logic, with no ROS in it: images in, a plain reply dict out."""
# The ROS node only converts messages. Keeping the logic here means the sidecar-absent behaviour
# and the reply contract are tested without a ROS graph. A service that cannot load its registry,
# calibration, whitening or backend does not crash the node: it stays up and answers
# "unknown" with sidecar_available false and the reason, so a caller can tell "this product is
# not recognised" from "nothing is looking".

from __future__ import annotations

from collections.abc import Mapping, Sequence
import logging
from pathlib import Path
from typing import Any

import numpy as np

from restocker_perception.identity.barcode import BarcodeBackend
from restocker_perception.identity.benchmark import load_whitener
from restocker_perception.identity.config import load_calibration
from restocker_perception.identity.engine import ABSTAIN, IDENTIFIED, UNKNOWN, IdentityEngine
from restocker_perception.registry.backends import EmbeddingBackend, OcrBackend
from restocker_perception.registry.model import Registry
from restocker_perception.registry.retrieval import RegistryIndex

STATUS_CODES = {IDENTIFIED: 0, ABSTAIN: 1, UNKNOWN: 2}
_LOG = logging.getLogger(__name__)


def image_to_rgb(height: int, width: int, encoding: str, step: int, data: bytes) -> np.ndarray:
    """A sensor_msgs/Image payload as an HxWx3 uint8 RGB array (rgb8 and bgr8 only)."""
    if encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"unsupported image encoding {encoding!r}; use rgb8 or bgr8")
    rows = np.frombuffer(bytes(data), dtype=np.uint8).reshape(height, step)
    pixels = rows[:, : width * 3].reshape(height, width, 3)
    return pixels[:, :, ::-1].copy() if encoding == "bgr8" else pixels.copy()


def _reply(
    status: str,
    *,
    available: bool,
    reason: str,
    sku: str | None = None,
    evidence: Sequence[Any] = (),
    provenance: dict[str, Any] | None = None,
    calibration_sha256: str = "",
    latency_ms: float = 0.0,
    product_gtins: Mapping[str, str] | None = None,
    channels: Sequence[str] = (),
) -> dict[str, Any]:
    provenance = provenance or {}
    # GTIN of a product, "" for one that has none (has_gtin says which; "" is never a GTIN).
    gtin_of = lambda product_id: (product_gtins or {}).get(product_id or "", "")  # noqa: E731
    return {
        "status": STATUS_CODES[status],
        "sidecar_available": available,
        "has_sku": sku is not None,
        "sku": sku or "",
        "has_gtin": bool(gtin_of(sku)),
        "gtin": gtin_of(sku),
        "reason": reason,
        "evidence_kinds": [e.kind for e in evidence],
        "evidence_product_ids": [e.product_id or "" for e in evidence],
        "evidence_gtins": [gtin_of(e.product_id) for e in evidence],
        "evidence_scores": [float(e.score) for e in evidence],
        "channels_used": list(channels),
        "barcode_enabled": bool(provenance.get("barcode_enabled", True)),
        "backend_name": provenance.get("backend_name") or "",
        "backend_version": provenance.get("backend_version") or "",
        "registry_sha256": provenance.get("registry_sha256") or "",
        "calibration_sha256": calibration_sha256,
        "latency_ms": float(latency_ms),
    }


class IdentifyService:
    """Wraps an :class:`IdentityEngine`; or, with none, the sidecar-absent answer."""

    def __init__(
        self,
        engine: IdentityEngine | None,
        *,
        calibration_sha256: str = "",
        unavailable_reason: str = "identity sidecar not configured",
        product_gtins: Mapping[str, str] | None = None,
    ) -> None:
        self.engine = engine
        self.product_gtins = dict(product_gtins or {})
        self.calibration_sha256 = calibration_sha256
        self.unavailable_reason = unavailable_reason

    @classmethod
    def load(
        cls,
        registry_path: str,
        calibration_path: str,
        whitener_path: str,
        backend: EmbeddingBackend,
        ocr_backend: OcrBackend | None = None,
        *,
        barcode_backend: BarcodeBackend | None = None,
        barcode_enabled: bool = True,
    ) -> IdentifyService:
        """Build the engine; any failure yields the unavailable service, never an exception.

        ``barcode_enabled=False`` leaves the barcode channel out of every decision (barcode-off
        benchmark arm); the reply says so in ``barcode_enabled`` and ``channels_used``.
        """
        try:
            registry = Registry.load(Path(registry_path))
            config, fit, digest = load_calibration(calibration_path)
            whitener = load_whitener(whitener_path)
            if whitener.fingerprint != fit["whitening"]["fingerprint"]:
                raise ValueError("whitening file does not match the calibration file")
            if registry.content_hash() != fit["registry_sha256"]:
                raise ValueError("registry differs from the one the calibration was fitted on")
            if f"{backend.name} {backend.version}" != fit["backend"]:
                raise ValueError("embedding backend differs from the calibration")
            index = RegistryIndex.from_registry(
                registry, backend, whitener=whitener if config.whiten else None
            )
            engine = IdentityEngine(
                index,
                backend,
                config,
                ocr_backend=ocr_backend,
                ocr_strings={r.product_id: list(r.ocr_strings) for r in registry.records()},
                barcode_backend=barcode_backend,
                gtin_to_product=registry.gtin_map(),
                barcode_enabled=barcode_enabled,
            )
        except Exception as error:  # noqa: BLE001 - every failure means "no sidecar"
            _LOG.warning("identity sidecar unavailable: %s", error)
            return cls(None, unavailable_reason=f"identity sidecar unavailable: {error}")
        return cls(
            engine,
            calibration_sha256=digest,
            product_gtins={p: g for g, p in registry.gtin_map().items()},
        )

    @property
    def available(self) -> bool:
        return self.engine is not None

    def reject(self, reason: str) -> dict[str, Any]:
        """The reply for a request that could not be read: abstain, with the reason."""
        return _reply(
            ABSTAIN,
            available=self.available,
            reason=reason,
            calibration_sha256=self.calibration_sha256,
        )

    def identify(
        self, crops: Sequence[np.ndarray], barcode_images: Sequence[np.ndarray] = ()
    ) -> dict[str, Any]:
        if self.engine is None:
            return _reply(UNKNOWN, available=False, reason=self.unavailable_reason)
        try:
            result = self.engine.identify(crops, barcode_images)
        except Exception as error:  # noqa: BLE001 - a failing model must not take the node down
            _LOG.exception("identity channel failed")
            return _reply(
                UNKNOWN,
                available=False,
                reason=f"identity channel failed: {error}",
                calibration_sha256=self.calibration_sha256,
            )
        return _reply(
            result.status,
            available=True,
            reason=result.reason,
            sku=result.sku,
            evidence=result.evidence,
            provenance=result.provenance,
            calibration_sha256=self.calibration_sha256,
            latency_ms=sum(result.latency_ms.values()),
            product_gtins=self.product_gtins,
            channels=result.channels,
        )
