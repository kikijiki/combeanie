# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Barcode channel: EAN-13 decode, check digit, exact GTIN lookup."""
# Only products that have a GTIN can be named by this channel; barcode-less products never are.
# CMB-101: the barcode decodes only at close range (<= 0.15 m, near-frontal aim), so this channel
# answers rarely. When it answers it is exact: a decoded string that is a well-formed GTIN-13 with
# a correct check digit is looked up in the registry. A well-formed GTIN that is not registered is
# positive evidence that the product is not one we know.

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Protocol, runtime_checkable

import numpy as np

from restocker_perception.registry.gtin import InvalidGtin, validate_gtin13


@runtime_checkable
class BarcodeBackend(Protocol):
    """Decodes the barcodes found in one image; returns the decoded strings."""

    name: str
    version: str

    def decode(self, image: np.ndarray) -> list[str]: ...


class ZxingBarcodeBackend:
    """zxing-cpp adapter, lazy and optional (not in the flake)."""

    name = "zxing-cpp"

    def __init__(self, version: str) -> None:
        self.version = version

    def decode(self, image: np.ndarray) -> list[str]:
        try:
            import zxingcpp  # noqa: PLC0415
        except ImportError as error:
            raise RuntimeError("ZxingBarcodeBackend needs zxing-cpp") from error
        return [
            result.text
            for result in zxingcpp.read_barcodes(
                np.asarray(image), formats=zxingcpp.BarcodeFormat.EAN13
            )
        ]


@dataclass(frozen=True)
class BarcodeRead:
    """A valid GTIN-13 decoded from the images (None when nothing valid was read)."""

    gtin: str | None
    registered: bool
    # Distinct valid GTINs seen; more than one means the images disagree.
    valid: tuple[str, ...]
    rejected: int  # decoded strings that were not a valid GTIN-13 (wrong length or check digit)
    # Registry product id of the product carrying ``gtin`` (None when unregistered or unread).
    product_id: str | None = None

    @property
    def conflict(self) -> bool:
        return len(self.valid) > 1


def read_gtin(
    images: Sequence[np.ndarray],
    backend: BarcodeBackend,
    registered: Mapping[str, str] | Sequence[str],
) -> BarcodeRead:
    """Decode ``images``. ``registered`` maps each registered GTIN to its product id (a plain
    sequence of GTINs means id == GTIN, the form every registry had before product ids)."""
    products = registered if isinstance(registered, Mapping) else {g: g for g in registered}
    valid: list[str] = []
    rejected = 0
    for image in images:
        for text in backend.decode(image):
            try:
                gtin = validate_gtin13(text)
            except InvalidGtin:
                rejected += 1
                continue
            if gtin not in valid:
                valid.append(gtin)
    if len(valid) == 1:
        return BarcodeRead(
            valid[0], valid[0] in products, tuple(valid), rejected, products.get(valid[0])
        )
    return BarcodeRead(None, False, tuple(valid), rejected)
