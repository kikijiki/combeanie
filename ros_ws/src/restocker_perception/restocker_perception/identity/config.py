# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity-channel thresholds and the calibration file that carries them."""
# The calibration file is small JSON with its own hash: the decision thresholds, plus what they
# were fitted on (products, seeds, backend, registry and whitening fingerprints, the pre-declared
# bounds file hash). The hash covers everything but the hash field, as the benchmark split file
# does, so an edited threshold is detected on load.

from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
import hashlib
import json
from pathlib import Path
from typing import Any

from restocker_perception.registry.retrieval import SCORINGS

SCHEMA_VERSION = 1


class CalibrationError(ValueError):
    """The calibration file is malformed, edited, or does not match its consumer."""


@dataclass(frozen=True)
class IdentityConfig:
    """Decision thresholds. Defaults are maximally conservative: nothing is ever named.

    Naming rules (see IdentityEngine): a barcode that decodes to a registered GTIN names it
    unless the embedding ranks it worse than supported_max_rank; an OCR answer names its
    product under the same rank condition; the embedding alone names its top-1 only if all three
    alone_* thresholds are set and met. unknown_below_score turns a weak best match into
    an explicit unknown.
    """

    scoring: str = "max"
    # Apply the fitted PCA whitening (CMB-101's recommendation); calibration chooses by data.
    whiten: bool = True
    # An independent channel (OCR, barcode) naming a product the embedding ranks at or above this
    # (1 = must be the embedding's top-1; 0 = the independent channels never name a product).
    supported_max_rank: int = 2
    alone_min_score: float | None = None
    alone_min_margin: float | None = None
    alone_min_vote: float | None = None
    unknown_below_score: float | None = None
    # Per-product floor on the embedding-alone score (product id -> minimum top-1 score): a product
    # that won wrongly on calibration data must score higher than it did then.
    product_floors: dict[str, float] = field(default_factory=dict)
    # OCR: rotations (degrees, counter-clockwise) tried per crop, first answer wins, on at most
    # ocr_max_crops crops of the object.
    ocr_rotations: tuple[int, ...] = (0, 45)
    ocr_max_crops: int = 2
    # Views of one object used per decision (the engine uses at most this many crops).
    max_views: int = 5

    def __post_init__(self) -> None:
        if self.scoring not in SCORINGS:
            raise CalibrationError(f"scoring must be one of {SCORINGS}, got {self.scoring!r}")
        if not self.ocr_rotations or self.ocr_max_crops < 1:
            raise CalibrationError("ocr_rotations must be non-empty and ocr_max_crops >= 1")
        if self.supported_max_rank < 0 or self.max_views < 1:
            raise CalibrationError("supported_max_rank must be >= 0 and max_views >= 1")
        alone = (self.alone_min_score, self.alone_min_margin, self.alone_min_vote)
        if any(a is None for a in alone) and any(a is not None for a in alone):
            raise CalibrationError("alone_* thresholds are set all together or not at all")

    @property
    def alone_enabled(self) -> bool:
        return self.alone_min_score is not None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> IdentityConfig:
        known = {f.name for f in fields(cls)}
        extra = set(data) - known
        if extra:
            raise CalibrationError(f"unknown config fields {sorted(extra)}")
        data = dict(data)
        if "ocr_rotations" in data:
            data["ocr_rotations"] = tuple(data["ocr_rotations"])
        return cls(**data)


def _body_hash(document: dict[str, Any]) -> str:
    body = {k: v for k, v in document.items() if k != "sha256"}
    text = json.dumps(body, sort_keys=True, separators=(",", ":"), allow_nan=False)
    return hashlib.sha256(text.encode()).hexdigest()


def write_calibration(path: str | Path, config: IdentityConfig, fit: dict[str, Any]) -> str:
    """Write the calibration file; returns its sha256."""
    document: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "config": config.to_dict(),
        "fit": fit,
    }
    document["sha256"] = _body_hash(document)
    Path(path).write_text(
        json.dumps(document, sort_keys=True, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return document["sha256"]


def load_calibration(path: str | Path) -> tuple[IdentityConfig, dict[str, Any], str]:
    """Read and verify a calibration file: (config, fit record, sha256)."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    if document.get("schema_version") != SCHEMA_VERSION:
        raise CalibrationError(
            f"{path}: unsupported schema_version {document.get('schema_version')}"
        )
    if document.get("sha256") != _body_hash(document):
        raise CalibrationError(f"{path}: embedded sha256 does not match (file was edited)")
    return IdentityConfig.from_dict(document["config"]), document["fit"], document["sha256"]
