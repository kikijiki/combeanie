#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Write the showcase floor texture: polished concrete slabs with saw-cut joints.

    tools/generate_showcase_floor.py [--output PATH] [--check]

The texture covers the floor visual of restocking_showcase.sdf once, edge to edge, so a joint
every SLAB_M metres is a joint of the world. Deterministic: the same seed gives the same bytes.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

import numpy as np
from PIL import Image

SIZE_PX = 1024
FLOOR_M = 7.2
SLAB_M = 1.8
SEED = 20261021
DEFAULT_OUTPUT = (
    Path(__file__).resolve().parent.parent
    / "ros_ws/src/restocker_gazebo/materials/textures/showcase_floor.png"
)


def _octave(rng: np.random.Generator, cells: int) -> np.ndarray:
    grid = rng.random((cells, cells)).astype(np.float32)
    image = Image.fromarray((grid * 255).astype(np.uint8)).resize(
        (SIZE_PX, SIZE_PX), Image.BICUBIC
    )
    return np.asarray(image, dtype=np.float32) / 255.0


def floor_texture() -> Image.Image:
    """The floor image: soft blotches, fine speckle, and a darker joint every SLAB_M."""
    rng = np.random.default_rng(SEED)
    blotch = sum(
        _octave(rng, cells) * weight for cells, weight in ((6, 0.5), (16, 0.3), (48, 0.2))
    )
    blotch = (blotch - blotch.min()) / (blotch.max() - blotch.min())
    speckle = rng.random((SIZE_PX, SIZE_PX)).astype(np.float32)
    shade = 0.80 + 0.14 * blotch + 0.035 * (speckle - 0.5)
    # Each slab is poured on its own day, so it carries its own overall tone.
    slabs = int(round(FLOOR_M / SLAB_M))
    tone = rng.uniform(0.965, 1.035, size=(slabs, slabs)).astype(np.float32)
    pixels_per_slab = SIZE_PX / slabs
    index = (np.arange(SIZE_PX) / pixels_per_slab).astype(int).clip(0, slabs - 1)
    shade *= tone[index[:, None], index[None, :]]
    # Joints: a dark line a few millimetres wide with a lighter shoulder.
    joint_px = max(2, int(round(0.006 * SIZE_PX / FLOOR_M)))
    along = np.arange(SIZE_PX)
    distance = np.abs(((along + pixels_per_slab / 2) % pixels_per_slab) - pixels_per_slab / 2)
    line = np.clip(1.0 - distance / joint_px, 0.0, 1.0)
    joints = np.maximum(line[:, None], line[None, :])
    shade *= 1.0 - 0.32 * joints
    shade = shade.clip(0.0, 1.0)
    base = np.array([0.80, 0.80, 0.78], dtype=np.float32)
    rgb = (shade[..., None] * base[None, None, :] * 255.0).clip(0, 255).astype(np.uint8)
    return Image.fromarray(rgb, "RGB")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--check", action="store_true", help="fail if the file on disk differs")
    arguments = parser.parse_args()
    image = floor_texture()
    if arguments.check:
        if not arguments.output.is_file():
            print(f"{arguments.output} is missing", file=sys.stderr)
            return 1
        if Image.open(arguments.output).convert("RGB").tobytes() != image.tobytes():
            print(f"{arguments.output} is stale; rerun without --check", file=sys.stderr)
            return 1
        return 0
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    image.save(arguments.output, optimize=True)
    print(f"wrote {arguments.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
