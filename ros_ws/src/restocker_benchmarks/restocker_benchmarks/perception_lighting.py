# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Seeded lighting conditions for the perception benchmark, applied to a copy of the world."""
# Pure Python, no ROS. A condition is a function of the seed only and is drawn from its own
# stream, so it can never change which products a scene holds (the frozen splits depend on that).
# The shipped world file is never edited: ``apply_lighting`` rewrites the ambient colour and the
# sun of a copy, which the recorder launches through the ``world`` launch argument.

from __future__ import annotations

from collections.abc import Sequence
import math
from typing import Any
import xml.etree.ElementTree as ElementTree

from restocker_gazebo.scenario_random import SplitMix64

LIGHTING_VERSION = 1
SUN_NAME = "workcell_sun"

# Each condition: scene ambient, the sun's diffuse and specular colour, and the direction the
# sun shines in (world frame, need not be unit). "nominal" is exactly the shipped world.
CONDITIONS: dict[str, dict[str, Any]] = {
    "nominal": {
        "ambient": [0.35, 0.35, 0.35],
        "diffuse": [0.9, 0.9, 0.9],
        "specular": [0.25, 0.25, 0.25],
        "direction": [-0.4, 0.2, -1.0],
    },
    "dim": {
        "ambient": [0.12, 0.12, 0.13],
        "diffuse": [0.3, 0.3, 0.32],
        "specular": [0.08, 0.08, 0.08],
        "direction": [-0.4, 0.2, -1.0],
    },
    "bright": {
        "ambient": [0.55, 0.55, 0.55],
        "diffuse": [1.0, 1.0, 1.0],
        "specular": [0.5, 0.5, 0.5],
        "direction": [-0.4, 0.2, -1.0],
    },
    "warm": {
        "ambient": [0.38, 0.32, 0.25],
        "diffuse": [1.0, 0.78, 0.52],
        "specular": [0.25, 0.2, 0.14],
        "direction": [-0.4, 0.2, -1.0],
    },
    "cool": {
        "ambient": [0.28, 0.33, 0.42],
        "diffuse": [0.68, 0.82, 1.0],
        "specular": [0.2, 0.24, 0.3],
        "direction": [-0.4, 0.2, -1.0],
    },
    # A low sun from the side: long shadows across the lanes and the tray.
    "directional": {
        "ambient": [0.14, 0.14, 0.15],
        "diffuse": [1.0, 0.97, 0.9],
        "specular": [0.3, 0.3, 0.3],
        "direction": [-0.9, 0.1, -0.22],
    },
}
CONDITION_NAMES = tuple(CONDITIONS)
# A seed scales light intensity by a factor in this range, so two scenes of one condition differ.
INTENSITY_JITTER = 0.1
# A seed turns the directional sun about the vertical axis by up to this many degrees.
AZIMUTH_JITTER_DEG = 25.0

_MASK64 = (1 << 64) - 1


def lighting_for_seed(seed: int) -> dict[str, Any]:
    """The lighting condition of a seed: a name plus the numbers applied to the world."""
    stream = SplitMix64((seed * 0x9E3779B97F4A7C15 + 0x4C494748 + LIGHTING_VERSION) & _MASK64)
    stream.next_u64()
    name = CONDITION_NAMES[stream.next_below(len(CONDITION_NAMES))]
    scale = 1.0 + (stream.next_unit() * 2.0 - 1.0) * INTENSITY_JITTER
    turn = math.radians((stream.next_unit() * 2.0 - 1.0) * AZIMUTH_JITTER_DEG)
    base = CONDITIONS[name]
    direction = list(base["direction"])
    if name == "directional":
        x, y = direction[0], direction[1]
        direction[0] = x * math.cos(turn) - y * math.sin(turn)
        direction[1] = x * math.sin(turn) + y * math.cos(turn)
    return {
        "condition": name,
        "version": LIGHTING_VERSION,
        "intensity_scale": round(scale, 6),
        "ambient": [round(min(1.0, value * scale), 6) for value in base["ambient"]],
        "diffuse": [round(min(1.0, value * scale), 6) for value in base["diffuse"]],
        "specular": [round(min(1.0, value * scale), 6) for value in base["specular"]],
        "direction": [round(value, 6) for value in direction],
    }


def _text(values: Sequence[float], alpha: bool) -> str:
    items = [f"{value:.6g}" for value in values] + (["1"] if alpha else [])
    return " ".join(items)


def apply_lighting(world_text: str, spec: dict[str, Any]) -> str:
    """Return ``world_text`` with its ambient colour and sun set from ``spec``."""
    parser = ElementTree.XMLParser(target=ElementTree.TreeBuilder(insert_comments=True))
    root = ElementTree.fromstring(world_text, parser=parser)  # noqa: S314 - repository file
    world = root.find("world")
    if world is None:
        raise ValueError("no <world> in the SDF")
    scene = world.find("scene")
    ambient = None if scene is None else scene.find("ambient")
    if ambient is None:
        raise ValueError("the world has no <scene><ambient>")
    ambient.text = _text(spec["ambient"], True)
    sun = next((light for light in world.findall("light") if light.get("name") == SUN_NAME), None)
    if sun is None:
        raise ValueError(f"the world has no light named {SUN_NAME}")
    for tag, key, alpha in (("diffuse", "diffuse", True), ("specular", "specular", True)):
        element = sun.find(tag)
        if element is None:
            raise ValueError(f"{SUN_NAME} has no <{tag}>")
        element.text = _text(spec[key], alpha)
    direction = sun.find("direction")
    if direction is None:
        raise ValueError(f"{SUN_NAME} has no <direction>")
    direction.text = _text(spec["direction"], False)
    ElementTree.indent(root, space="  ")
    return '<?xml version="1.0"?>\n' + ElementTree.tostring(root, encoding="unicode") + "\n"
