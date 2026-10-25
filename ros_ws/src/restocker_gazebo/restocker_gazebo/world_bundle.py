# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Write a world file that already holds the workcell and every scenario product."""
# The launch file normally starts one ``create`` process per product, paced a quarter of a second
# apart (see simulation.launch.py), so a scene of eighty products finishes appearing half a minute
# after the Gazebo window opens. A window that opens on an empty floor is wrong for a demo, so a
# bundled world carries the same models from its first frame: the URDF of each is converted to SDF
# exactly as ``create`` would convert it (``gz sdf -p``), given its scenario name and pose, and
# placed in the world next to the floor.
#
# Products differ only by name, pose and a handful of xacro arguments, so one conversion serves
# every product with the same arguments.
#
# A scene may also restyle a model's visuals (``materials``): the workcell description is shared
# with every test and benchmark, so its flat colours stay what they are there and a dressed scene
# overrides them here instead, in the world it is the only user of.

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import subprocess
import xml.etree.ElementTree as ET

import yaml

# xacro arguments -> SDF text of the converted model, as returned by ``gz sdf -p``.
Converter = Callable[[str, Mapping[str, str]], str]


@dataclass(frozen=True)
class BundledModel:
    """One model to place in the world: its xacro file and arguments, its name and its pose."""

    xacro_path: str
    arguments: Mapping[str, str]
    name: str
    pose: Sequence[float]


def _expand(xacro_path: str, arguments: Mapping[str, str]) -> str:
    import xacro  # noqa: PLC0415

    return xacro.process_file(xacro_path, mappings=dict(arguments)).toxml()


def _urdf_to_sdf(urdf: str) -> str:
    """Convert a URDF to SDF the way ros_gz_sim's create does (``gz sdf -p``)."""
    import tempfile  # noqa: PLC0415

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=True) as handle:
        handle.write(urdf)
        handle.flush()
        result = subprocess.run(
            ["gz", "sdf", "-p", handle.name], check=True, capture_output=True, text=True
        )
    return result.stdout


def convert_with_gz(xacro_path: str, arguments: Mapping[str, str]) -> str:
    """Expand a xacro file and convert the URDF to SDF."""
    return _urdf_to_sdf(_expand(xacro_path, arguments))


def convert_all_with_gz(keys: Sequence[tuple[str, Mapping[str, str]]]) -> list[str]:
    """Convert several descriptions: xacro one at a time, the ``gz`` processes side by side.

    Each ``gz sdf -p`` costs a second or more of start-up, and a launch test gives the launch
    fifteen seconds to start its processes, so the conversions must not queue up.
    """
    from concurrent.futures import ThreadPoolExecutor  # noqa: PLC0415

    urdfs = [_expand(path, arguments) for path, arguments in keys]
    with ThreadPoolExecutor(max_workers=8) as pool:
        return list(pool.map(_urdf_to_sdf, urdfs))


def _model_element(sdf_text: str, name: str, pose: Sequence[float]) -> ET.Element:
    root = ET.fromstring(sdf_text)
    model = root.find("model")
    if model is None:
        raise ValueError("converted description has no <model>")
    model.set("name", name)
    existing = model.find("pose")
    if existing is not None:
        model.remove(existing)
    pose_element = ET.Element("pose")
    pose_element.text = " ".join(repr(float(value)) for value in pose)
    model.insert(0, pose_element)
    return model


def load_materials(path: str) -> dict[str, list[dict]]:
    """Read a materials file: model name -> its rules (``match`` is a visual-name substring)."""
    with open(path, encoding="utf-8") as handle:
        document = yaml.safe_load(handle)
    return {model: list(rules) for model, rules in document.items()}


def _material_element(rule: dict) -> ET.Element:
    material = ET.Element("material")
    for tag in ("ambient", "diffuse", "specular", "emissive"):
        if tag in rule:
            ET.SubElement(material, tag).text = " ".join(repr(float(v)) for v in rule[tag])
    pbr = ET.SubElement(material, "pbr")
    metal = ET.SubElement(pbr, "metal")
    for tag in ("roughness", "metalness"):
        if tag in rule:
            ET.SubElement(metal, tag).text = repr(float(rule[tag]))
    if "albedo_map" in rule:
        ET.SubElement(metal, "albedo_map").text = str(rule["albedo_map"])
    return material


def restyle(model: ET.Element, rules: Sequence[dict]) -> int:
    """Give every visual whose name contains a rule's ``match`` that rule's material.

    The first matching rule wins; the number of visuals restyled is returned.
    """
    count = 0
    for visual in model.iter("visual"):
        name = visual.get("name", "")
        rule = next((rule for rule in rules if rule["match"] in name), None)
        if rule is None:
            continue
        for old in visual.findall("material"):
            visual.remove(old)
        visual.append(_material_element(rule))
        count += 1
    return count


def bundle_models(
    world_text: str,
    models: Sequence[BundledModel],
    convert: Converter | None = None,
    materials: Mapping[str, Sequence[dict]] | None = None,
) -> str:
    """Return ``world_text`` with every model placed before the world's closing tag."""
    close = world_text.rfind("</world>")
    if close < 0:
        raise ValueError("world has no closing </world>")
    keys = list(dict.fromkeys((m.xacro_path, tuple(sorted(m.arguments.items()))) for m in models))
    if convert is None:
        converted = dict(
            zip(keys, convert_all_with_gz([(p, dict(a)) for p, a in keys]), strict=True)
        )
    else:
        converted = {key: convert(key[0], dict(key[1])) for key in keys}
    pieces = []
    for model in models:
        key = (model.xacro_path, tuple(sorted(model.arguments.items())))
        element = _model_element(converted[key], model.name, model.pose)
        if materials and model.name in materials:
            restyle(element, materials[model.name])
        pieces.append(ET.tostring(element, encoding="unicode"))
    block = "\n    <!-- bundled scenario models -->\n    " + "\n    ".join(pieces) + "\n  "
    return world_text[:close] + block + world_text[close:]
