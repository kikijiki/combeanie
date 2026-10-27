# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Image-space stress on the crops a ladder look reads (CMB-116)."""
# Pure numpy, no model. The simulator can render a few lighting conditions and viewpoints, but it
# cannot dial an occluder, an exposure or a motion blur in steps. This module perturbs the crop the
# ladder is about to read, deterministically from (seed, object, look, crop), so one recorded
# dataset serves many stress levels at no simulator time. It is a proxy: a synthetic occluder is a
# flat block, a blur is a box kernel, a cast is a per-channel gain; the record says so. Nothing in
# the ladder, its thresholds or its decision logic knows it exists: it runs before ``observe``,
# only when ``ladder_benchmark step --stress SPEC`` is given.
#
# SPEC (all keys optional; an empty spec leaves every crop byte-identical):
#   gain, gamma        exposure: out = 255 * clip(in/255 * gain) ** gamma (gain > 1 clips)
#   cast               [r, g, b] per-channel gain (a coloured light)
#   noise_sigma        Gaussian sensor noise, grey levels
#   blur_px            box blur of this width (defocus)
#   motion_px          linear motion blur of this length, horizontal or vertical (arm moving)
#   downscale          shrink by this factor then enlarge back (fewer label pixels)
#   keep_views         vote over only the first K crops of a look (multi-view voting ablation)
#   every_nth          stress only every n-th crop (index % n == n - 1), the others stay clean:
#                      a sweep in which some frames are bad and some are not
#   occlusion          {"fraction": f, "kind": "bar" | "disc"}: a dark block covering about this
#                      share of the crop's width ("bar": a full-height strip entering from a seeded
#                      side, like a gripper finger or a neighbour; "disc": a disc of that diameter
#                      at a seeded position)

from __future__ import annotations

from collections.abc import Mapping
import hashlib
from typing import Any

import numpy as np

from restocker_perception.identity.ladder import LookCrop

STRESS_VERSION = 1
KEYS = frozenset(
    {
        "gain",
        "gamma",
        "cast",
        "noise_sigma",
        "blur_px",
        "motion_px",
        "downscale",
        "occlusion",
        "keep_views",
        "every_nth",
    }
)
# Keys that select crops instead of changing pixels.
SELECTORS = frozenset({"keep_views", "every_nth"})
OCCLUDER_GREY = 28


def validate_spec(spec: Mapping[str, Any]) -> None:
    unknown = set(spec) - KEYS - {"name", "note"}
    if unknown:
        raise ValueError(f"unknown stress keys: {sorted(unknown)}")
    occlusion = spec.get("occlusion")
    if occlusion is not None:
        if occlusion.get("kind") not in ("bar", "disc"):
            raise ValueError("occlusion.kind must be 'bar' or 'disc'")
        if not 0.0 <= float(occlusion.get("fraction", -1.0)) <= 1.0:
            raise ValueError("occlusion.fraction must be in [0, 1]")
    if float(spec.get("downscale", 1.0)) < 1.0:
        raise ValueError("downscale is a factor >= 1")
    if int(spec.get("keep_views", 1)) < 1 or int(spec.get("every_nth", 1)) < 1:
        raise ValueError("keep_views and every_nth are counts >= 1")


def rng_for(seed: int, object_id: str, look: int, crop: int) -> np.random.Generator:
    """A generator that depends on the crop's identity only (never on the order of processing)."""
    digest = hashlib.sha256(
        f"stress{STRESS_VERSION}|{seed}|{object_id}|{look}|{crop}".encode()
    ).digest()
    return np.random.default_rng(int.from_bytes(digest[:8], "big"))


def _box_blur(image: np.ndarray, width: int, axis: int | None) -> np.ndarray:
    """Mean over ``width`` pixels along ``axis`` (0 rows, 1 columns, None both)."""
    if width <= 1:
        return image
    out = image.astype(np.float32)
    for current in (0, 1) if axis is None else (axis,):
        padded = np.pad(
            out,
            [(width // 2, width - 1 - width // 2) if a == current else (0, 0) for a in range(3)],
            mode="edge",
        )
        cumulative = np.cumsum(padded, axis=current, dtype=np.float64)
        zero = np.zeros_like(np.take(cumulative, [0], axis=current))
        cumulative = np.concatenate([zero, cumulative], axis=current)
        upper = np.take(cumulative, range(width, width + out.shape[current]), axis=current)
        lower = np.take(cumulative, range(0, out.shape[current]), axis=current)
        out = ((upper - lower) / width).astype(np.float32)
    return out


def _resample(image: np.ndarray, factor: float) -> np.ndarray:
    """Shrink by ``factor`` with area-like averaging, then enlarge back by repetition."""
    height, width = image.shape[:2]
    small_h, small_w = max(1, round(height / factor)), max(1, round(width / factor))
    rows = np.linspace(0, height, small_h + 1).astype(int)
    cols = np.linspace(0, width, small_w + 1).astype(int)
    small = np.empty((small_h, small_w, image.shape[2]), dtype=np.float32)
    for i in range(small_h):
        for j in range(small_w):
            block = image[
                rows[i] : max(rows[i + 1], rows[i] + 1), cols[j] : max(cols[j + 1], cols[j] + 1)
            ]
            small[i, j] = block.reshape(-1, image.shape[2]).mean(axis=0)
    row_index = np.minimum((np.arange(height) * small_h) // height, small_h - 1)
    col_index = np.minimum((np.arange(width) * small_w) // width, small_w - 1)
    return small[row_index][:, col_index]


def stress_image(
    image: np.ndarray, spec: Mapping[str, Any], rng: np.random.Generator
) -> np.ndarray:
    """Return the crop with ``spec`` applied (uint8, same shape)."""
    validate_spec(spec)
    active = {key for key in KEYS - SELECTORS if key in spec}
    if not active:
        return image
    out = image.astype(np.float32)
    height, width = out.shape[:2]
    if "downscale" in spec and float(spec["downscale"]) > 1.0:
        out = _resample(out, float(spec["downscale"]))
    if spec.get("blur_px"):
        out = _box_blur(out, int(spec["blur_px"]), None)
    if spec.get("motion_px"):
        out = _box_blur(out, int(spec["motion_px"]), int(rng.integers(0, 2)))
    if "gain" in spec or "gamma" in spec or "cast" in spec:
        unit = out / 255.0
        if "cast" in spec:
            unit = unit * np.asarray(spec["cast"], dtype=np.float32).reshape(1, 1, 3)
        unit = np.clip(unit * float(spec.get("gain", 1.0)), 0.0, 1.0)
        out = 255.0 * unit ** float(spec.get("gamma", 1.0))
    if spec.get("noise_sigma"):
        out = out + rng.normal(0.0, float(spec["noise_sigma"]), size=out.shape)
    occlusion = spec.get("occlusion")
    if occlusion and float(occlusion["fraction"]) > 0.0:
        out = _occlude(out, float(occlusion["fraction"]), occlusion["kind"], rng)
    return np.clip(np.rint(out), 0, 255).astype(np.uint8)


def _occlude(
    image: np.ndarray, fraction: float, kind: str, rng: np.random.Generator
) -> np.ndarray:
    height, width = image.shape[:2]
    out = image.copy()
    if kind == "bar":
        span = max(1, round(fraction * width))
        side = int(rng.integers(0, 2))
        if side == 0:
            out[:, :span] = OCCLUDER_GREY
        else:
            out[:, width - span :] = OCCLUDER_GREY
        return out
    diameter = fraction * width
    centre_x = rng.uniform(0.25 * width, 0.75 * width)
    centre_y = rng.uniform(0.25 * height, 0.75 * height)
    rows, cols = np.ogrid[:height, :width]
    mask = (cols - centre_x) ** 2 + (rows - centre_y) ** 2 <= (diameter / 2.0) ** 2
    out[mask] = OCCLUDER_GREY
    return out


def apply_stress(
    inputs: dict[tuple[int, str], dict[str, Any]], spec: Mapping[str, Any], *, look: int
) -> int:
    """Stress every crop of a round's ``inputs`` in place; return how many crops changed."""
    validate_spec(spec)
    changed = 0
    for (seed, object_id), data in inputs.items():
        crops = []
        kept = data["crops"][: int(spec["keep_views"])] if "keep_views" in spec else data["crops"]
        if len(kept) != len(data["crops"]):
            changed += len(data["crops"]) - len(kept)
            for key in ("barcode_view_mask", "true_relative_azimuth_deg"):
                if data.get(key) is not None:
                    data[key] = data[key][: len(kept)]
        every = int(spec.get("every_nth", 1))
        for index, crop in enumerate(kept):
            if index % every != every - 1:
                crops.append(crop)
                continue
            image = stress_image(crop.image, spec, rng_for(seed, object_id, look, index))
            if image is crop.image:
                crops.append(crop)
                continue
            changed += 1
            crops.append(LookCrop(image, crop.half_width_px, crop.camera_azimuth_deg))
        data["crops"] = crops
    return changed
