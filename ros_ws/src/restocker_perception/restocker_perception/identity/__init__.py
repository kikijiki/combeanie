# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Identity channel: crop to embedding, OCR and barcode, fused into a calibrated decision."""

from restocker_perception.identity.barcode import (
    BarcodeBackend,
    BarcodeRead,
    ZxingBarcodeBackend,
    read_gtin,
)
from restocker_perception.identity.config import (
    CalibrationError,
    IdentityConfig,
    load_calibration,
    write_calibration,
)
from restocker_perception.identity.crops import crop_square
from restocker_perception.identity.engine import (
    ABSTAIN,
    IDENTIFIED,
    STATUSES,
    UNKNOWN,
    Evidence,
    IdentityEngine,
    IdentityResult,
    Observation,
    decide,
)
from restocker_perception.identity.ocr import OcrAnswer, OcrMatcher

__all__ = [
    "ABSTAIN",
    "IDENTIFIED",
    "STATUSES",
    "UNKNOWN",
    "BarcodeBackend",
    "BarcodeRead",
    "CalibrationError",
    "Evidence",
    "IdentityConfig",
    "IdentityEngine",
    "IdentityResult",
    "Observation",
    "OcrAnswer",
    "OcrMatcher",
    "ZxingBarcodeBackend",
    "crop_square",
    "decide",
    "load_calibration",
    "read_gtin",
    "write_calibration",
]
