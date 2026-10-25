# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""EAN-13 / GTIN-13 validation."""
# A product with a GTIN uses it as its registry id; one without a barcode has none (CMB-118).

from __future__ import annotations


class InvalidGtin(ValueError):
    """The string is not a well-formed GTIN-13 with a correct check digit."""


def check_digit(first_twelve: str) -> int:
    """GS1 check digit for the first twelve digits of a GTIN-13."""
    if len(first_twelve) != 12 or not first_twelve.isdigit():
        raise InvalidGtin(f"need twelve digits, got {first_twelve!r}")
    total = sum(int(d) * (3 if i % 2 else 1) for i, d in enumerate(first_twelve))
    return (10 - total % 10) % 10


def validate_gtin13(value: str) -> str:
    """Return ``value`` unchanged if it is a valid GTIN-13, else raise :class:`InvalidGtin`."""
    if not isinstance(value, str) or len(value) != 13 or not value.isdigit():
        raise InvalidGtin(f"GTIN must be exactly 13 digits, got {value!r}")
    expected = check_digit(value[:12])
    if int(value[12]) != expected:
        raise InvalidGtin(f"bad check digit in {value}: expected {expected}, got {value[12]}")
    return value
