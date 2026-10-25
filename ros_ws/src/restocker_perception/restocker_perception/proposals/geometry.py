# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Geometry-only product proposals from one depth image (CMB-113), no colour and no model."""
# What this is: the "geometry proposes" stage of docs/perception-plan.md, restated so it can be
# scored on the benchmark's textured products. The production colour-and-depth backend finds a
# product by chromaticity first and fits a cylinder second; a label-textured product has no
# chromaticity to find, so this proposer uses depth alone. It is the part of that backend that
# was never about colour:
#
#   1. back-project the depth image to world points and keep those above a known support
#      surface (the stock tray, or a lane's inclined roller bed) inside that surface's region of
#      interest;
#   2. rasterise them into a top-down height map (4 mm cells, tallest point per cell) and take
#      the connected components of the occupied cells: every component is a proposal ("blob");
#   3. split a component into upright cylinders the way the production backend does, by fitting
#      a disc to the highest flat plateau, giving the cylinder its footprint, removing that
#      footprint, and repeating on what is left (a partly hidden product behind it stays in the
#      loop). What no disc explains stays a blob. A flat top that is a rectangle rather than a
#      disc (a carton standing, a box lying on a face) is proposed as an oriented rectangle, chosen
#      by how well each model fills the plateau and by whether its edges are cliffs (a box) or
#      slopes (a lying cylinder's ridge); it is never named a catalogued cylinder (CMB-120).
#
# A disc proposal says "upright cylinder of this radius whose top is this high" and can be
# matched against the product catalogue; a blob says only "something stands here", which is
# what a lying product, a product with a narrow neck or a foreign object looks like. Nothing
# here reads a pixel colour, a product class or a SKU. numpy is required; cv2 is imported when a
# proposal is made (it is not in the flake, like torch for the identity channel).

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, replace
import math
from typing import Any

import numpy as np

CELL_M = 0.004
# Catalogued upright cylinders: (class, radius m, height m); a modelled test, not a gate.
CATALOGUE_CYLINDERS = (
    ("can", 0.033, 0.122),
    ("small_bottle", 0.034, 0.200),
    ("contour_bottle", 0.036, 0.230),
    ("large_bottle", 0.045, 0.290),
)
LANE_X_M = (-1.0, -0.6, -0.2, 0.2, 0.6, 1.0)
_LANE_SLOPE = math.tan(math.radians(4.0))


@dataclass(frozen=True)
class CameraModel:
    """Pinhole intrinsics and the optical frame's pose in the world ([x y z qx qy qz qw])."""

    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    pose_world: tuple[float, ...]

    @classmethod
    def from_label(cls, camera: dict[str, Any]) -> CameraModel:
        k = camera["intrinsics"]
        return cls(
            fx=float(k["fx"]),
            fy=float(k["fy"]),
            cx=float(k["cx"]),
            cy=float(k["cy"]),
            width=int(k["width"]),
            height=int(k["height"]),
            pose_world=tuple(float(v) for v in camera["pose_world"]),
        )

    def rotation(self) -> np.ndarray:
        x, y, z, w = self.pose_world[3:7]
        n = math.sqrt(x * x + y * y + z * z + w * w)
        x, y, z, w = x / n, y / n, z / n, w / n
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )

    def rays_world(self) -> tuple[np.ndarray, np.ndarray]:
        """Unit-depth ray directions (H, W, 3) in the world (z = 1 in the optical frame)."""
        u = (np.arange(self.width) - self.cx) / self.fx
        v = (np.arange(self.height) - self.cy) / self.fy
        grid = np.stack(
            [
                np.broadcast_to(u, (self.height, self.width)),
                np.broadcast_to(v[:, None], (self.height, self.width)),
                np.ones((self.height, self.width)),
            ],
            axis=-1,
        )
        return grid @ self.rotation().T, np.array(self.pose_world[:3])

    def points_world(self, depth_m: np.ndarray) -> np.ndarray:
        """World points (H, W, 3) of a z-depth image in metres; NaN where depth is not valid."""
        rays, origin = self.rays_world()
        points = rays * depth_m[..., None] + origin
        points[~(depth_m > 0)] = np.nan
        return points


@dataclass(frozen=True)
class SupportSurface:
    """A region of the world where products stand, with the floor they stand on.

    The floor is the plane ``z = floor_a + floor_b * y``; products are looked for in the box
    ``x_range`` x ``y_range`` between ``min_height_m`` and ``max_height_m`` above it.
    """

    name: str
    x_range: tuple[float, float]
    y_range: tuple[float, float]
    floor_a: float
    floor_b: float
    max_height_m: float
    # Known low structure that is not a product: points at y >= rail_from_y below rail_max_m
    # above the floor (a lane's front retainer rail, 0.09 m high, 0.84 m from the lane's rear).
    rail_from_y: float | None = None
    rail_max_m: float = 0.0

    def floor_z(self, y: np.ndarray) -> np.ndarray:
        return self.floor_a + self.floor_b * y


def tray_surface() -> SupportSurface:
    """The stock tray's usable volume (workcell_geometry.yaml, shelf frame shifted to world)."""
    return SupportSurface("tray", (-0.68, 0.68), (-1.055, -0.545), 0.570, 0.0, 0.35)


def lane_surface(lane_index: int) -> SupportSurface:
    """One lane's usable volume: the 4 degree roller bed, 0.365 m wide, 0.85 m deep."""
    centre = LANE_X_M[lane_index]
    return SupportSurface(
        f"lane_{lane_index + 1:02d}",
        (centre - 0.18, centre + 0.18),
        (0.55 + 0.01, 0.55 + 0.86),
        0.75 + 1.39 * _LANE_SLOPE,
        -_LANE_SLOPE,
        0.40,
        rail_from_y=0.55 + 0.835,
        rail_max_m=0.105,
    )


def all_surfaces() -> list[SupportSurface]:
    return [tray_surface(), *(lane_surface(i) for i in range(len(LANE_X_M)))]


@dataclass(frozen=True)
class ProposalConfig:
    """Tolerances, fixed from the catalogue and the sensor, not fitted on benchmark scenes."""

    min_height_m: float = 0.020
    min_area_m2: float = 0.0006
    plateau_band_m: float = 0.004
    disc_radius_m: tuple[float, float] = (0.024, 0.052)
    disc_rms_m: float = 0.006
    disc_min_cells: int = 12
    footprint_margin_m: float = 0.006
    catalogue_radius_tol_m: float = 0.009
    catalogue_height_tol_m: float = 0.030
    max_products_per_blob: int = 30
    # A peeled disc may be smaller than a whole top (part of it hidden), down to this fraction.
    peel_min_radius_ratio: float = 1.0
    # How far a blob reaches around its top plateau: about one product radius.
    blob_reach_m: float = 0.028
    # A flat top is a rectangle (CMB-120) when the oriented rectangle around the plateau is at
    # least this full (a disc fills pi/4 = 0.785 of its bounding rectangle, so a round top never
    # passes), its sides are at least rect_min_side_m / at most rect_max_side_m, its outline is
    # a cliff and not a slope (fraction of the not-taller, observed neighbours that are at least
    # rect_cliff_drop_m lower; a lying cylinder's ridge falls away by a few millimetres per
    # cell), and it explains the plateau better than the fitted disc by rect_over_disc_margin.
    rect_min_overlap: float = 0.86
    rect_min_side_m: float = 0.018
    rect_max_side_m: float = 0.30
    rect_cliff_drop_m: float = 0.010
    rect_min_cliff_share: float = 0.70
    rect_min_edge_cells: int = 8
    rect_max_concavity_m: float = 0.010
    rect_over_disc_margin: float = 0.06
    # Two flat tops of equal height that touch are cut apart at their waist: between concavities
    # at least neck_min_depth_m deep, a cut at most neck_max_m long, at most max_neck_splits deep.
    neck_min_depth_m: float = 0.006
    neck_max_m: float = 0.10
    max_neck_splits: int = 3
    # A blob does not take a flat surface (relief within a 3 x 3 cell window at most
    # flat_relief_m) lower than its top by more than flat_below_m: that is another product's top.
    flat_relief_m: float = 0.002
    flat_below_m: float = 0.010
    # Points this far below the support plane still count as the floor having been seen.
    floor_seen_tolerance_m: float = 0.015


@dataclass(frozen=True)
class Proposal:
    """One proposal: a pixel region of the image and what depth says stands there."""

    proposal_id: int
    surface: str
    # "disc": an upright cylinder top was fitted; "rect": a rectangular flat top (a box standing
    # or lying) was fitted; "blob": something stands here.
    kind: str
    centre_world: tuple[float, float]
    top_z_m: float
    height_m: float
    radius_m: float
    area_m2: float
    n_pixels: int
    bbox: tuple[int, int, int, int]  # u0, v0, u1, v1 (half open)
    disc_rms_m: float | None = None
    catalogue_class: str | None = None
    extra: dict[str, Any] = field(default_factory=dict)
    # kind "rect" only: the plateau's oriented rectangle, long side first, and the long side's
    # direction in the world xy plane (radians, in [0, pi)).
    size_m: tuple[float, float] | None = None
    yaw_rad: float | None = None

    @property
    def shape_class(self) -> str:
        """What the depth footprint says the product is: a cylinder (round top), a box
        (rectangular top) or unknown (a blob: a lying cylinder, a contoured bottle, a heap)."""
        return {"disc": "cylinder", "rect": "box"}.get(self.kind, "unknown")


@dataclass(frozen=True)
class ProposalSet:
    """Proposals and the per-pixel label image (0 = no proposal, i = proposal_id)."""

    proposals: tuple[Proposal, ...]
    labels: np.ndarray

    def mask(self, proposal_id: int) -> np.ndarray:
        return self.labels == proposal_id


def catalogue_class(radius_m: float, height_m: float, config: ProposalConfig) -> str | None:
    """The catalogued upright cylinder that this disc radius and height match, if any."""
    best, best_error = None, None
    for name, radius, height in CATALOGUE_CYLINDERS:
        if abs(radius_m - radius) > config.catalogue_radius_tol_m:
            continue
        if abs(height_m - height) > config.catalogue_height_tol_m:
            continue
        error = (
            abs(height_m - height) / config.catalogue_height_tol_m
            + abs(radius_m - radius) / config.catalogue_radius_tol_m
        )
        if best_error is None or error < best_error:
            best, best_error = name, error
    return best


def fit_circle(points: np.ndarray) -> tuple[float, float, float, float] | None:
    """Algebraic (Kasa) circle fit; returns centre x, y, radius and the rms radial residual."""
    if len(points) < 3:
        return None
    x, y = points[:, 0], points[:, 1]
    design = np.column_stack([x, y, np.ones(len(x))])
    target = x * x + y * y
    try:
        solution, *_ = np.linalg.lstsq(design, target, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = solution[0] / 2.0, solution[1] / 2.0
    radius_sq = solution[2] + cx * cx + cy * cy
    if radius_sq <= 0:
        return None
    radius = math.sqrt(radius_sq)
    residual = np.hypot(x - cx, y - cy) - radius
    return float(cx), float(cy), float(radius), float(math.sqrt(np.mean(residual**2)))


def _import_cv2() -> Any:
    import cv2  # noqa: PLC0415 - optional dependency, not in the flake

    return cv2


def _boundary(mask: np.ndarray, cv2: Any) -> np.ndarray:
    eroded = cv2.erode(mask.astype(np.uint8), np.ones((3, 3), np.uint8))
    return mask & (eroded == 0)


def _peel_plateau(
    cluster: np.ndarray,
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    config: ProposalConfig,
    cv2: Any,
) -> list[tuple[float, float, float, float]]:
    """Discs (centre x, y, radius, rms) peeled off a plateau by distance-transform peaks."""
    lo, hi = config.disc_radius_m
    work = cluster.copy()
    discs: list[tuple[float, float, float, float]] = []
    for _ in range(config.max_products_per_blob):
        if int(work.sum()) < config.disc_min_cells:
            break
        transform = cv2.distanceTransform(work.astype(np.uint8), cv2.DIST_L2, 5) * CELL_M
        peak = float(transform.max())
        radius = peak + 0.5 * CELL_M
        if radius < lo * config.peel_min_radius_ratio:
            break
        if radius > hi:
            # More than one disc wide: no product is that big, so this is no stack of discs.
            return []
        iy, ix = np.unravel_index(int(np.argmax(transform)), transform.shape)
        centre_x, centre_y = float(cell_x[iy, ix]), float(cell_y[iy, ix])
        discs.append((centre_x, centre_y, radius, 0.5 * CELL_M))
        work = work & (
            np.hypot(cell_x - centre_x, cell_y - centre_y) > radius + config.footprint_margin_m
        )
    return discs


@dataclass(frozen=True)
class _Rect:
    """An oriented rectangle fitted to a plateau (world xy, metres)."""

    centre: tuple[float, float]
    long_side: float
    short_side: float
    yaw: float
    fill: float
    cliff_share: float


def _rect_geometry(
    cluster: np.ndarray,
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    hidden: np.ndarray | None = None,
) -> tuple[tuple[float, float], float, float, float, float] | None:
    """Centre, long side, short side, long-side direction and overlap of the plateau's rectangle.

    The rectangle has the plateau's second moments (a uniform rectangle of sides L x W has
    variances L^2/12 and W^2/12 along its axes), which a staircase of cells does not bias the way
    the corner-to-corner enclosing rectangle is. ``overlap`` is the intersection over union of
    the plateau with that rectangle (cell centres inside it): about 0.93 for a rectangle, and
    below 0.85 for a disc, an L or two products joined at a corner. ``hidden`` marks cells a
    taller product stands on: a corner of the rectangle under one is not held against it.
    """
    xs, ys = cell_x[cluster], cell_y[cluster]
    if xs.size < 4:
        return None
    centre = (float(xs.mean()), float(ys.mean()))
    covariance = np.cov(np.vstack([xs - centre[0], ys - centre[1]]), bias=True)
    values, vectors = np.linalg.eigh(covariance)
    long_side = math.sqrt(12.0 * max(values[1], 0.0))
    short_side = math.sqrt(12.0 * max(values[0], 0.0))
    if short_side <= 0.0:
        return None
    direction = math.atan2(vectors[1, 1], vectors[0, 1]) % math.pi
    dx, dy = cell_x - centre[0], cell_y - centre[1]
    along = dx * math.cos(direction) + dy * math.sin(direction)
    across = -dx * math.sin(direction) + dy * math.cos(direction)
    inside = (np.abs(along) <= long_side / 2.0) & (np.abs(across) <= short_side / 2.0)
    explained = cluster if hidden is None else cluster | (hidden & inside)
    union = int((inside | explained).sum())
    overlap = float((inside & explained).sum()) / union if union else 0.0
    return centre, long_side, short_side, direction, overlap


def _deepest_concavity(cluster: np.ndarray, cv2: Any) -> float:
    """Depth in metres of the plateau outline's deepest concavity (0 for a convex plateau).

    A rectangle's staircase of cells dents its hull by a cell or two; a row of touching discs is
    pinched to a waist at every contact, as deep as a fraction of a disc's radius.
    """
    contours, _ = cv2.findContours(
        cluster.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        return 0.0
    outline = max(contours, key=len)
    if len(outline) < 8:
        return 0.0
    hull = cv2.convexHull(outline, returnPoints=False)
    if hull is None or len(hull) < 4:
        return 0.0
    try:
        defects = cv2.convexityDefects(outline, hull)
    except cv2.error:
        return 0.0
    if defects is None:
        return 0.0
    return float(defects[:, 0, 3].max()) / 256.0 * CELL_M


def _cliff_share(
    cluster: np.ndarray,
    heights: np.ndarray,
    seen: np.ndarray,
    top: float,
    config: ProposalConfig,
    cv2: Any,
) -> tuple[float, int]:
    """Share of the plateau's outline that is a cliff, and how many outline cells decided it.

    One cell outside the plateau, among the neighbours that are not taller (a taller neighbour
    hides the edge) and were observed at all (a shadow says nothing): a cliff drops at least
    ``rect_cliff_drop_m``, a slope (a lying cylinder's ridge) a few millimetres per cell.
    """
    kernel = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], np.uint8)
    ring = (cv2.dilate(cluster.astype(np.uint8), kernel) > 0) & ~cluster
    outside = np.where(heights > 0.0, heights, 0.0)
    usable = ring & seen & (outside <= top + config.plateau_band_m)
    count = int(usable.sum())
    if count == 0:
        return 0.0, 0
    return float((usable & (top - outside >= config.rect_cliff_drop_m)).sum()) / count, count


def _fit_rect(
    cluster: np.ndarray,
    heights: np.ndarray,
    seen: np.ndarray,
    top: float,
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    config: ProposalConfig,
    cv2: Any,
) -> _Rect | None:
    """The oriented rectangle of a flat top, if the plateau is a rectangle with cliff edges."""
    geometry = _rect_geometry(cluster, cell_x, cell_y, heights > top + config.plateau_band_m)
    if geometry is None:
        return None
    centre, long_side, short_side, direction, fill = geometry
    if (
        fill < config.rect_min_overlap
        or short_side < config.rect_min_side_m
        or long_side > config.rect_max_side_m
    ):
        return None
    if _deepest_concavity(cluster, cv2) > config.rect_max_concavity_m:
        return None
    share, count = _cliff_share(cluster, heights, seen, top, config, cv2)
    if count < config.rect_min_edge_cells or share < config.rect_min_cliff_share:
        return None
    return _Rect(centre, long_side, short_side, direction, fill, share)


def _disc_overlap(cluster: np.ndarray, fit: tuple[float, ...] | None, cell_x, cell_y) -> float:
    """Intersection over union of the plateau with the fitted circle (0 without a fit)."""
    if fit is None:
        return 0.0
    disc = np.hypot(cell_x - fit[0], cell_y - fit[1]) <= fit[2]
    union = int((disc | cluster).sum())
    return float((disc & cluster).sum()) / union if union else 0.0


def _inside_rect(rect: _Rect, cell_x: np.ndarray, cell_y: np.ndarray, margin: float) -> np.ndarray:
    """Cells within the rectangle grown by ``margin`` on every side."""
    dx, dy = cell_x - rect.centre[0], cell_y - rect.centre[1]
    along = dx * math.cos(rect.yaw) + dy * math.sin(rect.yaw)
    across = -dx * math.sin(rect.yaw) + dy * math.cos(rect.yaw)
    return (np.abs(along) <= rect.long_side / 2.0 + margin) & (
        np.abs(across) <= rect.short_side / 2.0 + margin
    )


def _flat_lower_tops(
    heights: np.ndarray, remaining: np.ndarray, top: float, config: ProposalConfig, cv2: Any
) -> np.ndarray:
    """Cells of flat surfaces clearly lower than ``top``, and the cell ring around each."""
    live = np.where(remaining, heights, 0.0).astype(np.float32)
    kernel = np.ones((3, 3), np.uint8)
    relief = cv2.dilate(live, kernel) - cv2.erode(live, kernel)
    core = remaining & (relief <= config.flat_relief_m) & (heights < top - config.flat_below_m)
    return cv2.dilate(core.astype(np.uint8), kernel) > 0


def _neck_split(cluster: np.ndarray, config: ProposalConfig, cv2: Any) -> list[np.ndarray] | None:
    """Cut a plateau in two at its narrowest neck between two concave corners, or None.

    Two flat tops of equal height that touch (a lying box next to another, a carton pair) form
    one plateau with a waist where they meet; the waist is bounded by two concavities of the
    outline. The cut is the shortest segment joining two concavities at least ``neck_min_depth_m``
    deep that leaves two pieces of at least ``disc_min_cells`` each.
    """
    contours, _ = cv2.findContours(
        cluster.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
    )
    if not contours:
        return None
    outline = max(contours, key=len)
    if len(outline) < 8:
        return None
    hull = cv2.convexHull(outline, returnPoints=False)
    if hull is None or len(hull) < 4:
        return None
    try:
        defects = cv2.convexityDefects(outline, hull)
    except cv2.error:
        return None
    if defects is None:
        return None
    floor = config.neck_min_depth_m / CELL_M
    points = [outline[int(far)][0] for _, _, far, depth in defects[:, 0] if depth / 256.0 >= floor]
    best: tuple[float, np.ndarray] | None = None
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            length = float(np.hypot(*(points[i] - points[j])))
            if length * CELL_M > config.neck_max_m or (best is not None and length >= best[0]):
                continue
            cut = np.zeros(cluster.shape, np.uint8)
            cv2.line(cut, tuple(int(v) for v in points[i]), tuple(int(v) for v in points[j]), 1, 1)
            rest = cluster & (cut == 0)
            count, labels = cv2.connectedComponents(rest.astype(np.uint8), connectivity=4)
            sizes = [int((labels == k).sum()) for k in range(1, count)]
            big = [k for k, size in zip(range(1, count), sizes, strict=True) if size >= config.disc_min_cells]
            if len(big) >= 2:
                best = (length, labels * np.isin(labels, big))
    if best is None:
        return None
    labels = best[1]
    # The cut's own cells go to the nearest piece.
    parts = [labels == k for k in np.unique(labels) if k > 0]
    claimed = np.zeros(cluster.shape, np.uint8)
    for k, part in enumerate(parts, start=1):
        claimed[part] = k
    near = cv2.dilate(claimed, np.ones((3, 3), np.uint8))
    out = []
    for k in range(1, len(parts) + 1):
        out.append(cluster & ((claimed == k) | ((claimed == 0) & (near == k))))
    return out


def _explain_plateau(
    cluster: np.ndarray,
    remaining: np.ndarray,
    heights: np.ndarray,
    seen: np.ndarray,
    top: float,
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    config: ProposalConfig,
    cv2: Any,
    splits_left: int,
) -> list[dict[str, Any]]:
    """The rectangles and discs one flat top is, or [] if it is neither (a ridge, a heap)."""
    if cluster.sum() < config.disc_min_cells:
        return []
    fit = fit_circle(np.column_stack([cell_x[_boundary(cluster, cv2)], cell_y[_boundary(cluster, cv2)]]))
    lo, hi = config.disc_radius_m
    disc_fits = fit is not None and lo <= fit[2] <= hi and fit[3] <= config.disc_rms_m
    # A rectangular top is not a disc, however well a circle's boundary fit may pass (a
    # 65 x 40 mm carton top fits a 26 mm circle within the rms), and it must not be peeled into
    # discs (a lying 55 x 150 mm box is three of them). The rectangle wins when it fills the
    # plateau clearly better than the fitted circle does.
    rect = _fit_rect(cluster, heights, seen, top, cell_x, cell_y, config, cv2)
    if rect is not None and (
        not disc_fits
        or rect.fill >= _disc_overlap(cluster, fit, cell_x, cell_y) + config.rect_over_disc_margin
    ):
        owned = remaining & _inside_rect(rect, cell_x, cell_y, config.footprint_margin_m)
        return [
            {
                "kind": "rect",
                "cells": owned,
                "centre": rect.centre,
                "radius": math.sqrt(rect.long_side * rect.short_side / math.pi),
                "size": (rect.long_side, rect.short_side),
                "yaw": rect.yaw,
                "top": top,
                "rms": None,
            }
        ]
    if disc_fits:
        return [_disc_piece(fit, remaining, top, cell_x, cell_y, config)]
    if splits_left <= 0:
        return []
    parts = _neck_split(cluster, config, cv2)
    if parts is None:
        return []
    found = [
        piece
        for part in parts
        for piece in _explain_plateau(
            part, remaining, heights, seen, top, cell_x, cell_y, config, cv2, splits_left - 1
        )
    ]
    # Only a split that finds a rectangle is trusted: touching discs have their own peeling.
    return found if any(piece["kind"] == "rect" for piece in found) else []


def _disc_piece(
    fit: tuple[float, ...],
    remaining: np.ndarray,
    top: float,
    cell_x: np.ndarray,
    cell_y: np.ndarray,
    config: ProposalConfig,
) -> dict[str, Any]:
    cx, cy, radius, rms = fit
    inside = np.hypot(cell_x - cx, cell_y - cy) <= radius + config.footprint_margin_m
    return {
        "kind": "disc",
        "cells": remaining & inside,
        "centre": (cx, cy),
        "radius": radius,
        "top": top,
        "rms": rms,
    }


def _split_component(
    heights: np.ndarray,
    component: np.ndarray,
    seen: np.ndarray,
    origin: np.ndarray,
    config: ProposalConfig,
    cv2: Any,
) -> list[dict[str, Any]]:
    """Peel upright cylinders and rectangular tops off one component, tallest plateau first.

    What neither a disc nor a rectangle explains is a blob.
    """
    pieces: list[dict[str, Any]] = []
    remaining = component.copy()
    ys, xs = np.mgrid[0 : heights.shape[0], 0 : heights.shape[1]]
    cell_x = origin[0] + (xs + 0.5) * CELL_M
    cell_y = origin[1] + (ys + 0.5) * CELL_M
    for _ in range(config.max_products_per_blob):
        if not remaining.any():
            break
        live = np.where(remaining, heights, -np.inf)
        top = float(live.max())
        plateau = remaining & (heights >= top - config.plateau_band_m)
        count, labels = cv2.connectedComponents(plateau.astype(np.uint8), connectivity=8)
        seed = np.unravel_index(int(np.argmax(live)), live.shape)
        cluster = labels == labels[seed]
        explained = _explain_plateau(
            cluster, remaining, heights, seen, top, cell_x, cell_y, config, cv2,
            config.max_neck_splits,
        )
        taken = False
        for piece in explained:
            owned = piece["cells"] & remaining
            if not owned.any():
                continue
            piece["cells"] = owned
            pieces.append(piece)
            remaining = remaining & ~owned
            taken = True
        if taken:
            continue
        # Touching tops of equal height merge into one plateau that is no circle: peel the
        # discs off it by distance-transform peaks before giving up on it.
        discs = _peel_plateau(cluster, cell_x, cell_y, config, cv2) if cluster.sum() >= config.disc_min_cells else []
        for fit in discs:
            piece = _disc_piece(fit, remaining, top, cell_x, cell_y, config)
            owned = piece["cells"] & remaining
            if not owned.any():
                continue
            pieces.append(piece)
            remaining = remaining & ~owned
            taken = True
        if taken:
            continue
        # No disc explains the tallest plateau: the connected leftover around it is one blob.
        # It takes the plateau and what hangs around it within a product's reach, not the whole
        # component: the shorter products it touches get their own turn in this loop.
        reach = max(1, int(round(config.blob_reach_m / CELL_M)))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * reach + 1, 2 * reach + 1))
        blob = remaining & (cv2.dilate(cluster.astype(np.uint8), kernel) > 0)
        # The flank of a lying cylinder falls away by millimetres per cell; a flat top that is
        # lower than the blob (a box's, a short product's) is another product's, and its turn in
        # this loop comes later, so the blob does not take it.
        blob &= ~_flat_lower_tops(heights, remaining, top, config, cv2)
        pieces.append({"kind": "blob", "cells": blob, "top": top, "rms": None})
        remaining = remaining & ~blob
    return pieces


def propose(
    depth_m: np.ndarray,
    camera: CameraModel,
    surfaces: Sequence[SupportSurface] | None = None,
    config: ProposalConfig | None = None,
) -> ProposalSet:
    """Geometry proposals for one z-depth image in metres (0 or NaN where invalid)."""
    config = config or ProposalConfig()
    surfaces = list(surfaces) if surfaces is not None else all_surfaces()
    cv2 = _import_cv2()
    points = camera.points_world(np.where(np.isfinite(depth_m), depth_m, 0.0))
    valid = np.isfinite(points[..., 0])
    labels = np.zeros(depth_m.shape, dtype=np.int32)
    proposals: list[Proposal] = []
    next_id = 1
    for surface in surfaces:
        x, y, z = points[..., 0], points[..., 1], points[..., 2]
        with np.errstate(invalid="ignore"):
            height = z - surface.floor_z(y)
            inside = (
                valid
                & (x >= surface.x_range[0])
                & (x < surface.x_range[1])
                & (y >= surface.y_range[0])
                & (y < surface.y_range[1])
                & (height > config.min_height_m)
                & (height < surface.max_height_m)
            )
            if surface.rail_from_y is not None:
                inside &= ~((y >= surface.rail_from_y) & (height < surface.rail_max_m))
        if not inside.any():
            continue
        origin = np.array([surface.x_range[0], surface.y_range[0]])
        shape = (
            int(math.ceil((surface.y_range[1] - surface.y_range[0]) / CELL_M)),
            int(math.ceil((surface.x_range[1] - surface.x_range[0]) / CELL_M)),
        )
        rows_px, cols_px = np.nonzero(inside)
        cell_col = np.clip(((x[inside] - origin[0]) / CELL_M).astype(int), 0, shape[1] - 1)
        cell_row = np.clip(((y[inside] - origin[1]) / CELL_M).astype(int), 0, shape[0] - 1)
        heights = np.full(shape, -np.inf)
        np.maximum.at(heights, (cell_row, cell_col), height[inside])
        occupied = np.isfinite(heights)
        # Cells where the camera saw anything at all inside the region, floor included: a cell
        # in a shadow has no return and says nothing about the edge of a neighbouring plateau.
        with np.errstate(invalid="ignore"):
            region = (
                valid
                & (x >= surface.x_range[0])
                & (x < surface.x_range[1])
                & (y >= surface.y_range[0])
                & (y < surface.y_range[1])
                & (height > -config.floor_seen_tolerance_m)
                & (height < surface.max_height_m)
            )
        seen = np.zeros(shape, dtype=bool)
        seen[
            np.clip(((y[region] - origin[1]) / CELL_M).astype(int), 0, shape[0] - 1),
            np.clip(((x[region] - origin[0]) / CELL_M).astype(int), 0, shape[1] - 1),
        ] = True
        seen = cv2.morphologyEx(
            seen.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
        ).astype(bool)
        occupied = cv2.morphologyEx(
            occupied.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
        ).astype(bool)
        heights = np.where(np.isfinite(heights), heights, 0.0)
        # A closed gap cell has no point of its own; give it its neighbours' height.
        filled = cv2.dilate(heights.astype(np.float32), np.ones((3, 3), np.uint8))
        heights = np.where((heights == 0.0) & occupied, filled, heights)
        count, component_labels = cv2.connectedComponents(
            occupied.astype(np.uint8), connectivity=8
        )
        cell_owner = np.zeros(shape, dtype=np.int32)
        local: dict[int, dict[str, Any]] = {}
        for component_index in range(1, count):
            component = component_labels == component_index
            if component.sum() * CELL_M * CELL_M < config.min_area_m2:
                continue
            for piece in _split_component(heights, component, seen, origin, config, cv2):
                area = float(piece["cells"].sum()) * CELL_M * CELL_M
                if area < config.min_area_m2:
                    continue
                cell_owner[piece["cells"]] = next_id
                local[next_id] = {**piece, "area": area, "surface_floor_top": piece["top"]}
                next_id += 1
        if not local:
            continue
        pixel_owner = cell_owner[cell_row, cell_col]
        labels[rows_px, cols_px] = pixel_owner
        for proposal_id, piece in local.items():
            member = labels == proposal_id
            n_pixels = int(member.sum())
            if n_pixels == 0:
                continue
            ys_px, xs_px = np.nonzero(member)
            if piece["kind"] in ("disc", "rect"):
                centre = piece["centre"]
                radius = piece["radius"]
            else:
                ys_c, xs_c = np.nonzero(piece["cells"])
                centre = (
                    float(origin[0] + (xs_c.mean() + 0.5) * CELL_M),
                    float(origin[1] + (ys_c.mean() + 0.5) * CELL_M),
                )
                radius = math.sqrt(piece["area"] / math.pi)
            height_m = float(piece["top"])
            proposals.append(
                Proposal(
                    proposal_id=proposal_id,
                    surface=surface.name,
                    kind=piece["kind"],
                    centre_world=(float(centre[0]), float(centre[1])),
                    top_z_m=float(surface.floor_z(np.array(centre[1])) + height_m),
                    height_m=height_m,
                    radius_m=float(radius),
                    area_m2=float(piece["area"]),
                    n_pixels=n_pixels,
                    bbox=(
                        int(xs_px.min()),
                        int(ys_px.min()),
                        int(xs_px.max()) + 1,
                        int(ys_px.max()) + 1,
                    ),
                    disc_rms_m=None if piece["rms"] is None else float(piece["rms"]),
                    size_m=piece.get("size"),
                    yaw_rad=piece.get("yaw"),
                    catalogue_class=(
                        catalogue_class(float(radius), height_m, config)
                        if piece["kind"] == "disc"
                        else None
                    ),
                )
            )
    kept = {p.proposal_id for p in proposals}
    labels[~np.isin(labels, list(kept))] = 0
    return ProposalSet(tuple(proposals), labels)


def with_extra(proposal: Proposal, **extra: Any) -> Proposal:
    """A copy of the proposal with extra evidence attached."""
    return replace(proposal, extra={**proposal.extra, **extra})
