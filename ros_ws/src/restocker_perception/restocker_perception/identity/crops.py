# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Square product crops from an image, an object pixel and a range."""
# The detector (or the benchmark labels) supplies where the object centre projects and how far
# away it is; the product class supplies its real extent. The crop is the square that would just
# contain an object of that extent at that range, plus a margin, clipped to the image.

from __future__ import annotations

import numpy as np


def crop_square(
    image: np.ndarray,
    u: float,
    v: float,
    range_m: float,
    focal_px: float,
    extent_m: float,
    *,
    margin: float = 1.2,
    min_visible: float = 0.6,
) -> np.ndarray | None:
    """Crop around pixel (u, v); None if the object is mostly outside the image.

    extent_m is the object's largest dimension; the crop side in pixels is
    focal_px * extent_m * margin / range_m. min_visible is the fraction of the square
    that must lie inside the image.
    """
    if range_m <= 0 or focal_px <= 0 or extent_m <= 0:
        return None
    side = focal_px * extent_m * margin / range_m
    height, width = image.shape[:2]
    left, top = u - side / 2.0, v - side / 2.0
    x0, y0 = int(round(max(left, 0.0))), int(round(max(top, 0.0)))
    x1, y1 = int(round(min(left + side, width))), int(round(min(top + side, height)))
    if x1 - x0 < 2 or y1 - y0 < 2:
        return None
    if (x1 - x0) * (y1 - y0) < min_visible * side * side:
        return None
    return np.ascontiguousarray(image[y0:y1, x0:x1, :3])


def rotate_image(image: np.ndarray, degrees: float, fill: int = 128) -> np.ndarray:
    """Rotate counter-clockwise about the centre on an enlarged canvas (bilinear, numpy only).

    Quarter turns are exact (``rot90``). Used by the OCR channel: a tilted product shows its
    label text at an angle, and a reader that expects horizontal text needs the crop turned.
    """
    degrees = degrees % 360.0
    if degrees == 0.0:
        return image
    if degrees % 90.0 == 0.0:
        return np.ascontiguousarray(np.rot90(image, int(degrees // 90)))
    height, width = image.shape[:2]
    radians = np.deg2rad(degrees)
    cos, sin = float(np.cos(radians)), float(np.sin(radians))
    out_w = int(np.ceil(abs(width * cos) + abs(height * sin)))
    out_h = int(np.ceil(abs(width * sin) + abs(height * cos)))
    ys, xs = np.mgrid[0:out_h, 0:out_w].astype(np.float32)
    xs -= (out_w - 1) / 2.0
    ys -= (out_h - 1) / 2.0
    # Inverse map: output (x, y) came from the input rotated the other way (image y points down).
    src_x = cos * xs - sin * ys + (width - 1) / 2.0
    src_y = sin * xs + cos * ys + (height - 1) / 2.0
    inside = (src_x >= 0) & (src_x <= width - 1) & (src_y >= 0) & (src_y <= height - 1)
    x0 = np.clip(np.floor(src_x).astype(int), 0, width - 2)
    y0 = np.clip(np.floor(src_y).astype(int), 0, height - 2)
    fx = np.clip(src_x - x0, 0, 1)[..., None]
    fy = np.clip(src_y - y0, 0, 1)[..., None]
    pixels = image.astype(np.float32)
    top = pixels[y0, x0] * (1 - fx) + pixels[y0, x0 + 1] * fx
    bottom = pixels[y0 + 1, x0] * (1 - fx) + pixels[y0 + 1, x0 + 1] * fx
    mixed = top * (1 - fy) + bottom * fy
    mixed[~inside] = fill
    return np.clip(mixed + 0.5, 0, 255).astype(np.uint8)


def unwrap_cylinder(
    image: np.ndarray,
    radius_px: float,
    half_angle_deg: float = 80.0,
    centre_x: float | None = None,
) -> np.ndarray:
    """Unroll the visible barrel of an upright cylinder: columns become equal steps of angle.

    The barrel's axis is the image column ``centre_x`` (default the middle) and its silhouette
    half-width is ``radius_px``. A flat label wrapped around the cylinder is foreshortened
    towards the silhouette edges (x = r sin theta); sampling uniformly in theta undoes that, which
    is what a barcode reader needs for a code spanning a large share of the circumference.
    Orthographic approximation, nearest-better bilinear in x, rows untouched.
    """
    height, width = image.shape[:2]
    centre = (width - 1) / 2.0 if centre_x is None else centre_x
    half = np.deg2rad(half_angle_deg)
    out_width = max(8, int(round(2.0 * half * radius_px)))
    theta = np.linspace(-half, half, out_width, dtype=np.float32)
    source = centre + radius_px * np.sin(theta)
    x0 = np.clip(np.floor(source).astype(int), 0, width - 2)
    fx = np.clip(source - x0, 0.0, 1.0)[None, :, None]
    pixels = image.astype(np.float32)
    mixed = pixels[:, x0] * (1.0 - fx) + pixels[:, x0 + 1] * fx
    return np.clip(mixed + 0.5, 0, 255).astype(np.uint8)
