# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Which side of a product the camera sees, and how far to turn it to see the readable one."""
# The registry holds eight views per product, 45 degrees apart (a turntable). Two independent
# measurements say where on that turntable a query crop sits, and both use only registry data:
#
# * the embedding similarity of the crop to each of the product's eight exemplars, interpolated
#   between neighbouring azimuths;
# * for a body of revolution, a one-dimensional colour signature of the visible barrel (unrolled so
#   that columns are equal steps of angle) correlated against the same signature of the product's
#   panorama, rebuilt from the central slice of each registered view.
#
# Each is a score per azimuth; they are z-scored, summed, and pooled over the object's views, each
# shifted by the difference of the camera azimuths (the viewpoints are known to the caller). The
# peak is the estimated facing: the registry azimuth the camera sees. Turning the product by
# ``facing - target`` (degrees, counter-clockwise seen from above) shows the azimuth
# ``target``, so no absolute offset between the registry's and the world's azimuth is ever
# needed. The target is the azimuth whose registered view carries the most label text (the
# registry's OCR reads).
#
# A box has four faces, not a continuous barrel: its facing is the best of the views at 0, 90, 180
# and 270 degrees (the front, left, back and right face), and the suggested turn is a multiple
# of 90.
#
# Nothing here is trained.

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np

from restocker_perception.identity.crops import unwrap_cylinder

VIEW_STEP_DEG = 45
FACE_NAMES = {0: "front", 90: "left", 180: "back", 270: "right"}
# Vertical bands of the unrolled barrel whose mean colours form the signature.
BANDS = ((0.2, 0.4), (0.4, 0.6), (0.6, 0.8))
# Angular half-width of the visible barrel that is correlated (the rest is foreshortened away).
QUERY_HALF_ANGLE_DEG = 60
# Registry views are cut to the body with this margin on each side (registry.render.MARGIN).
DEFAULT_VIEW_MARGIN = 0.04


def circular_difference_deg(a: float, b: float) -> float:
    """a - b wrapped to (-180, 180]."""
    d = (a - b) % 360.0
    return d - 360.0 if d > 180.0 else d


def _signature(strip: np.ndarray) -> np.ndarray:
    height = strip.shape[0]
    return np.concatenate(
        [
            strip[int(height * a) : int(height * b)].astype(np.float32).mean(axis=0)
            for a, b in BANDS
        ],
        axis=1,
    )


def _resample(columns: np.ndarray, count: int) -> np.ndarray:
    x = np.linspace(0, len(columns) - 1, count)
    base = np.arange(len(columns))
    return np.stack([np.interp(x, base, columns[:, c]) for c in range(columns.shape[1])], axis=1)


def _zscore(values: np.ndarray) -> np.ndarray:
    return (values - values.mean()) / (values.std() + 1e-9)


def panorama(
    view_images: Sequence[np.ndarray], view_margin: float = DEFAULT_VIEW_MARGIN
) -> np.ndarray:
    """The 360-column colour signature of a body of revolution from its eight registered views."""
    tiles = []
    for image in view_images:
        half_width = image.shape[1] / (2.0 * (1.0 + view_margin))
        strip = unwrap_cylinder(image, half_width, half_angle_deg=VIEW_STEP_DEG / 2)
        tiles.append(_resample(_signature(strip), VIEW_STEP_DEG))
    # View k shows the texture angle 45 k at its centre: it covers [45 k - 22.5, 45 k + 22.5).
    # The concatenation starts at angle -22.5; roll it so that column i is the angle i degrees.
    return np.roll(np.concatenate(tiles, axis=0), -(VIEW_STEP_DEG // 2), axis=0)


def barrel_signature(crop: np.ndarray, half_width_px: float) -> np.ndarray:
    """The visible barrel of a crop as a (120, features) signature, one row per degree."""
    centre = (crop.shape[1] - 1) / 2.0
    strip = unwrap_cylinder(
        crop, half_width_px, half_angle_deg=QUERY_HALF_ANGLE_DEG, centre_x=centre
    )
    return _resample(_signature(strip), 2 * QUERY_HALF_ANGLE_DEG)


def barrel_curve(pano: np.ndarray, signature: np.ndarray) -> np.ndarray:
    """Correlation of a crop's barrel signature with the panorama centred at each angle (360,)."""
    query = signature - signature.mean(axis=0)
    norm_query = np.linalg.norm(query)
    out = np.zeros(360)
    window = np.arange(2 * QUERY_HALF_ANGLE_DEG) - QUERY_HALF_ANGLE_DEG
    for angle in range(360):
        piece = pano[(window + angle) % 360]
        piece = piece - piece.mean(axis=0)
        out[angle] = (query * piece).sum() / (norm_query * np.linalg.norm(piece) + 1e-9)
    return out


def exemplar_curve(similarities: Sequence[float]) -> np.ndarray:
    """Per-azimuth score (360,) interpolated between the eight exemplar similarities."""
    sims = np.asarray(similarities, dtype=np.float64)
    out = np.zeros(360)
    for angle in range(360):
        k, f = divmod(angle, VIEW_STEP_DEG)
        out[angle] = (1 - f / VIEW_STEP_DEG) * sims[k % 8] + f / VIEW_STEP_DEG * sims[(k + 1) % 8]
    return out


@dataclass(frozen=True)
class ViewEvidence:
    """What one crop contributes to a facing estimate."""

    exemplar_similarities: tuple[float, ...]
    # Round bodies only: the crop's barrel signature (:func:`barrel_signature`).
    signature: np.ndarray | None = None
    # Direction of the camera seen from the object, degrees, in any fixed (world) frame.
    camera_azimuth_deg: float | None = None


@dataclass(frozen=True)
class Facing:
    """The estimate for one object, in the frame of its first view."""

    azimuth_deg: float
    # Peak of the pooled score minus its runner-up outside a 60 degree neighbourhood, in z units.
    confidence: float
    shape: str  # "round" or "box"
    visible_face: str | None = None


@dataclass(frozen=True)
class Turn:
    """Where to turn the product: how far, and to show which azimuth of the registered views."""

    degrees: float  # counter-clockwise seen from above, in (-180, 180]; box: a multiple of 90
    target_azimuth_deg: float
    target_face: str | None = None


class TextAzimuths:
    """Per product: how much label text each registered view carries (the registry's OCR)."""

    # Azimuths closer than this to a side already looked at do not count as a new side.
    SEEN_RADIUS_DEG = 22.5

    def __init__(self, scores_by_product: dict[str, Sequence[float]]) -> None:
        self.scores = {p: np.asarray(s, dtype=np.float64) for p, s in scores_by_product.items()}

    def target(self, product_id: str, *, box: bool, seen: Sequence[float] = ()) -> float:
        """The azimuth to show next: the text-richest side that has not been looked at yet.

        ``seen`` are the azimuths (registry frame) earlier looks showed; when every side has been
        seen the richest side overall is returned. A product with no text at all gives 0.
        """
        scores = self.scores.get(product_id)
        if scores is None or not np.any(scores > 0):
            return 0.0
        if box:
            faces = [0, 90, 180, 270]
            new = [f for f in faces if all(abs(circular_difference_deg(f, s)) >= 45 for s in seen)]
            pool = new or faces
            return float(max(pool, key=lambda f: (scores[f // VIEW_STEP_DEG], -f)))
        smooth = np.array(
            [scores[(k - 1) % 8] + scores[k] + scores[(k + 1) % 8] for k in range(8)]
        )
        candidates = [
            k
            for k in range(8)
            if all(
                abs(circular_difference_deg(k * VIEW_STEP_DEG, s)) >= self.SEEN_RADIUS_DEG
                for s in seen
            )
        ]
        best = max(candidates or range(8), key=lambda k: (smooth[k], -k))
        # Centroid of the best view and its two neighbours, weighted by the square of their text
        # (a front panel spans several neighbouring views; the centroid lands between them, and a
        # second text panel on the far side of the barrel does not pull it).
        ks = [(best - 1) % 8, best, (best + 1) % 8]
        angles = np.radians(np.array(ks) * VIEW_STEP_DEG)
        weights = scores[ks] ** 2
        if weights.sum() == 0:
            return float(best * VIEW_STEP_DEG)
        return float(
            math.degrees(
                math.atan2((weights * np.sin(angles)).sum(), (weights * np.cos(angles)).sum())
            )
            % 360.0
        )


def estimate_facing(
    views: Sequence[ViewEvidence],
    panorama_of_product: np.ndarray | None,
    *,
    box: bool = False,
    barrel_weight: float = 0.5,
) -> Facing | None:
    """Pool the views into one facing estimate (registry azimuth of the first view's camera)."""
    if not views:
        return None
    reference = views[0].camera_azimuth_deg
    total = np.zeros(360)
    for view in views:
        curve = _zscore(exemplar_curve(view.exemplar_similarities))
        if not box and panorama_of_product is not None and view.signature is not None:
            barrel = _zscore(barrel_curve(panorama_of_product, view.signature))
            curve = barrel_weight * barrel + (1.0 - barrel_weight) * curve
        shift = 0.0
        if reference is not None and view.camera_azimuth_deg is not None:
            shift = view.camera_azimuth_deg - reference
        total += np.roll(curve, -int(round(shift)))
    if box:
        faces = np.array([total[a] for a in (0, 90, 180, 270)])
        face = int(faces.argmax())
        ordered = np.sort(faces)[::-1]
        return Facing(
            float(90 * face),
            float(ordered[0] - ordered[1]) / len(views),
            "box",
            FACE_NAMES[90 * face],
        )
    peak = int(total.argmax())
    outside = np.array(
        [total[a] for a in range(360) if abs(circular_difference_deg(a, peak)) > 30]
    )
    confidence = float((total[peak] - outside.max()) / len(views))
    return Facing(float(peak), confidence, "round")


def suggest_turn(facing: Facing, target_azimuth_deg: float) -> Turn:
    """The turn that moves the visible azimuth from ``facing`` to ``target_azimuth_deg``."""
    if facing.shape == "box":
        degrees = circular_difference_deg(facing.azimuth_deg, target_azimuth_deg)
        degrees = float(90 * round(degrees / 90))
        if degrees == -180:
            degrees = 180.0
        return Turn(degrees, target_azimuth_deg, FACE_NAMES.get(int(target_azimuth_deg) % 360))
    return Turn(
        circular_difference_deg(facing.azimuth_deg, target_azimuth_deg), target_azimuth_deg
    )


__all__ = [
    "Facing",
    "TextAzimuths",
    "Turn",
    "ViewEvidence",
    "barrel_signature",
    "estimate_facing",
    "panorama",
    "suggest_turn",
]
