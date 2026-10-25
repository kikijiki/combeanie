# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A numpy port of the lane depth histogram, plus the column's top-height profile."""
# lane_depth_measurement.cpp is the producer on the robot. This is the same arithmetic on the same
# window and thresholds, so a recorded depth frame can be turned into the evidence the wrist
# survey would have published (free depth, coverage, obstruction) without a ROS graph, and the
# estimator can be scored offline. It adds one thing the C++ does not report: for every depth bin,
# the greatest height above the bed among the bin's product returns. The top of a column of
# upright products is a staircase of heights, one step per run of equal products, and that is what
# the estimator reads the rear product's class and a mixed-class column from.

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from restocker_perception.stock.config import StockConfig

# Returns this close to the bed plane are taken to be the bed when estimating the frame's noise.
BED_BAND_M = 0.04


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole intrinsics of the depth camera, pixels."""

    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class LaneProfile:
    """What one depth frame says about one lane."""

    lane_id: str
    available_depth_m: float
    obstructed: bool
    coverage: float
    nearest_surface_found: bool
    nearest_surface_depth_m: float
    product_returns: int
    bed_returns: int
    obstruction_returns: int
    valid_depth_samples: int
    sampled_pixels: int
    bin_start_m: np.ndarray
    # Greatest height above the bed among a bin's product returns; NaN where the bin has fewer
    # than minimum_surface_bin_returns of them (no surface there).
    top_height_m: np.ndarray
    # Least lane depth of a bin's product returns; NaN where there is no surface.
    near_depth_m: np.ndarray
    # The floor margin the measurement used (the fixed one, or the noise-widened one).
    floor_margin_m: float = 0.0
    # Robust standard deviation of the bed's height residuals in the frame (0 if not estimated).
    bed_sigma_m: float = 0.0


def lane_from_optical(
    camera_pose_world: list[float] | np.ndarray,
    lane_center_x_m: float,
    pose_world: tuple[float, float, float],
) -> np.ndarray:
    """4x4 lane <- optical from an optical-frame pose [x y z qx qy qz qw] in the world.

    The lane frame is the shelf frame (axes aligned with the world) moved to the lane's centre
    line at the shelf's rear edge, at the retainer-face height datum.
    """
    x, y, z, qx, qy, qz, qw = (float(v) for v in camera_pose_world)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    rotation = np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ]
    )
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [
        x - (pose_world[0] + lane_center_x_m),
        y - pose_world[1],
        z - pose_world[2],
    ]
    return matrix


def measure_lane(
    depth_m: np.ndarray,
    intrinsics: Intrinsics,
    lane_from_optical_matrix: np.ndarray,
    config: StockConfig,
    lane_id: str = "",
) -> LaneProfile:
    """Measure one lane from a depth image in metres (``0``/NaN/inf are no return)."""
    cell, meas = config.workcell, config.measurement
    height, width = depth_m.shape
    rows = np.arange(0, height, meas.pixel_stride)
    columns = np.arange(0, width, meas.pixel_stride)
    sample = depth_m[np.ix_(rows, columns)].astype(np.float64)
    sampled_pixels = sample.size
    finite = np.isfinite(sample) & (sample > 0.0)
    in_range = finite & (sample >= meas.minimum_depth_m) & (sample <= meas.maximum_depth_m)
    valid_depth_samples = int(in_range.sum())
    column_grid, row_grid = np.meshgrid(columns, rows)
    rng = sample[in_range]
    optical = np.stack(
        [
            (column_grid[in_range] - intrinsics.cx) * rng / intrinsics.fx,
            (row_grid[in_range] - intrinsics.cy) * rng / intrinsics.fy,
            rng,
        ]
    )
    lane = lane_from_optical_matrix[:3, :3] @ optical + lane_from_optical_matrix[:3, 3:4]
    lx, ly, lz = lane
    inside = np.abs(lx) <= cell.half_width_m
    above_bed = lz - cell.bed_height_at(ly)
    margin = meas.floor_margin_m
    sigma = 0.0
    inset = meas.front_inset_m
    if meas.floor_noise_sigma_factor > 0.0:
        # Range noise lifts bed returns above a fixed margin and reads the bed as product. The
        # bed's own residuals say how noisy this frame is: widen the margin to that many robust
        # sigmas (plus the plane's offset), never below the fixed floor.
        near_bed = inside & (np.abs(above_bed) < BED_BAND_M)
        if int(near_bed.sum()) >= meas.minimum_accepted_returns:
            residual = above_bed[near_bed]
            centre = float(np.median(residual))
            sigma = 1.4826 * float(np.median(np.abs(residual - centre)))
            margin = max(margin, abs(centre) + meas.floor_noise_sigma_factor * sigma)
            # The same noise smears the retainer's face along the lane into the window: at the
            # station's grazing angle a depth error is about twice a height error, so the front
            # inset grows to twice the noise term of the margin.
            inset = max(inset, 2.0 * meas.floor_noise_sigma_factor * sigma)
    window_front = cell.bed_datum_depth_m - inset
    span = window_front - cell.rear_clearance_m
    bin_count = max(1, math.ceil(span / meas.bin_width_m))
    inside &= (above_bed <= cell.usable_height_m) & (above_bed >= -margin)
    above_floor = above_bed >= margin
    in_window = inside & (ly >= cell.rear_clearance_m) & (ly <= window_front)
    obstruction_rear = cell.rear_clearance_m - meas.obstruction_reach_m
    behind_entrance = inside & (ly >= obstruction_rear) & (ly < cell.rear_clearance_m)
    obstruction_returns = int((behind_entrance & above_floor).sum())

    product = in_window & above_floor
    bed_returns = int((in_window & ~above_floor).sum())
    bins = np.minimum(
        bin_count - 1, ((ly[in_window] - cell.rear_clearance_m) / meas.bin_width_m).astype(int)
    )
    any_per_bin = np.bincount(bins, minlength=bin_count)
    product_bins = np.minimum(
        bin_count - 1, ((ly[product] - cell.rear_clearance_m) / meas.bin_width_m).astype(int)
    )
    product_per_bin = np.bincount(product_bins, minlength=bin_count)
    near_depth = np.full(bin_count, window_front)
    np.minimum.at(near_depth, product_bins, ly[product])
    top = np.full(bin_count, -np.inf)
    np.maximum.at(top, product_bins, above_bed[product])

    surface_bins = np.flatnonzero(product_per_bin >= meas.minimum_surface_bin_returns)
    found = surface_bins.size > 0
    nearest = float(near_depth[surface_bins[0]]) if found else 0.0
    if found and sigma > 0.0:
        # The least depth of many noisy returns sits before the surface by a few depth sigmas
        # (measured on the development frames: 3.6 to 4.4 bed sigmas at 0.5-1 % range noise).
        nearest += meas.edge_bias_sigma_factor * sigma
    available = (
        min(max(nearest - cell.rear_clearance_m, 0.0), cell.usable_depth_m)
        if found
        else cell.usable_depth_m
    )
    bin_start = cell.rear_clearance_m + np.arange(bin_count) * meas.bin_width_m
    shadowed = (bin_start >= nearest) if found else np.zeros(bin_count, dtype=bool)
    covered = int(((any_per_bin > 0) | shadowed).sum())
    accepted = int(product.sum()) + bed_returns
    coverage = covered / bin_count if accepted >= meas.minimum_accepted_returns else 0.0
    has_surface = product_per_bin >= meas.minimum_surface_bin_returns
    return LaneProfile(
        lane_id=lane_id,
        available_depth_m=available,
        obstructed=obstruction_returns >= meas.minimum_obstruction_returns,
        coverage=coverage,
        nearest_surface_found=found,
        nearest_surface_depth_m=nearest,
        product_returns=int(product.sum()),
        bed_returns=bed_returns,
        obstruction_returns=obstruction_returns,
        valid_depth_samples=valid_depth_samples,
        sampled_pixels=sampled_pixels,
        bin_start_m=bin_start,
        top_height_m=np.where(has_surface, top, np.nan),
        near_depth_m=np.where(has_surface, near_depth, np.nan),
        floor_margin_m=margin,
        bed_sigma_m=sigma,
    )
