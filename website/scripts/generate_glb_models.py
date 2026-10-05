#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Generate website GLB models from the assembled restocker URDF.

The robot is expanded with Xacro and its visual geometry loaded, including the UR10e Collada
meshes from ``ur_description``. Run after ``just build`` from a ROS environment providing
``xacro`` and ``ur_description``. The workcell comes from surveyed YAML. The workcell and
combined exports also include the dense-demo products, using catalog geometry at the scenario
spawn poses. Writes:

  website/static/models/robot.glb
  website/static/models/workcell.glb
  website/static/models/restocking_cell.glb

Usage (from the repository root after ``just build``):

  scripts/with_workspace.bash python website/scripts/generate_glb_models.py
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

# ---------------------------------------------------------------------------
# Materials (materials.xacro + workcell.xacro inline colours)
# ---------------------------------------------------------------------------

MATERIALS = {
    "dark_gray": [0.13, 0.15, 0.18, 1.0],
    "light_gray": [0.55, 0.60, 0.65, 1.0],
    "orange": [0.95, 0.38, 0.08, 1.0],
    "blue": [0.08, 0.35, 0.72, 1.0],
    "black": [0.02, 0.02, 0.025, 1.0],
    "divider": [0.72, 0.78, 0.82, 0.72],
    "frame": [0.20, 0.23, 0.27, 1.0],
    "tray": [0.24, 0.28, 0.33, 1.0],
    "shelf": [0.58, 0.64, 0.70, 1.0],
    "roller": [0.42, 0.46, 0.52, 1.0],
    "camera": [0.04, 0.05, 0.07, 1.0],
}

# Gripper geometry from gripper_geometry.yaml defaults (overridden on load)
GRIPPER_DEFAULT = {
    "body_size": (0.16, 0.11, 0.07),
    "body_center": (0.0, 0.0, 0.035),
    "finger_size": (0.035, 0.018, 0.14),
    "finger_center": (0.0, 0.0, 0.07),
    "left_origin": (0.0, 0.03, 0.07),
    "right_origin": (0.0, -0.03, 0.07),
    "open_target_m": 0.032,
}


def _rgba(name: str) -> np.ndarray:
    return np.asarray(MATERIALS[name], dtype=np.float64)


def _paint(mesh: trimesh.Trimesh, name: str) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.visual.face_colors = (_rgba(name) * 255).astype(np.uint8)
    return mesh


def _paint_rgba(mesh: trimesh.Trimesh, rgba: list[float]) -> trimesh.Trimesh:
    mesh = mesh.copy()
    mesh.visual.face_colors = (np.asarray(rgba, dtype=np.float64) * 255).astype(np.uint8)
    return mesh


def _T(xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)) -> np.ndarray:
    """URDF origin transform: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    r = rz @ ry @ rx
    t = np.eye(4)
    t[:3, :3] = r
    t[:3, 3] = (x, y, z)
    return t


def _axis_angle(axis: tuple[float, float, float], angle: float) -> np.ndarray:
    ax = np.asarray(axis, dtype=np.float64)
    ax = ax / np.linalg.norm(ax)
    x, y, z = ax
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    r = np.array(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ]
    )
    t = np.eye(4)
    t[:3, :3] = r
    return t


def _box(size, xyz=(0, 0, 0), rpy=(0, 0, 0), material="light_gray") -> trimesh.Trimesh:
    mesh = trimesh.creation.box(extents=size)
    mesh.apply_transform(_T(xyz, rpy))
    return _paint(mesh, material)


def _cylinder(
    radius, length, xyz=(0, 0, 0), rpy=(0, 0, 0), material="light_gray"
) -> trimesh.Trimesh:
    # trimesh cylinder is Z-aligned, matching URDF
    mesh = trimesh.creation.cylinder(radius=radius, height=length, sections=24)
    mesh.apply_transform(_T(xyz, rpy))
    return _paint(mesh, material)


def _empty_scene() -> trimesh.Scene:
    return trimesh.Scene()


def _add(scene: trimesh.Scene, name: str, mesh: trimesh.Trimesh, matrix: np.ndarray) -> None:
    scene.add_geometry(mesh, geom_name=name, node_name=name, transform=matrix)


# ---------------------------------------------------------------------------
# Joint presets (metres / radians), matching URDF joint names without prefix
# ---------------------------------------------------------------------------

POSE_HOME = {
    "rail_joint": 0.0,
    "shoulder_pan_joint": 0.0,
    "shoulder_lift_joint": -math.pi / 2.0,
    "elbow_joint": 0.0,
    "wrist_1_joint": -math.pi / 2.0,
    "wrist_2_joint": 0.0,
    "wrist_3_joint": 0.0,
    "left_finger_joint": 0.032,
    "right_finger_joint": 0.032,
}

# Mild reach toward lane_03 / shelf (+Y): face shelf, fold elbow, lower wrist.
POSE_REACH_LANE = {
    "rail_joint": -0.2,
    "shoulder_pan_joint": math.pi / 2.0,
    "shoulder_lift_joint": -0.95,
    "elbow_joint": 1.35,
    "wrist_1_joint": 0.0,
    "wrist_2_joint": 0.55,
    "wrist_3_joint": 0.0,
    "left_finger_joint": 0.032,
    "right_finger_joint": 0.032,
}


def _numbers(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(float(v) for v in value.split()) if value else default


def _origin(element: ET.Element | None) -> np.ndarray:
    if element is None:
        return np.eye(4)
    return _T(
        _numbers(element.get("xyz"), (0.0, 0.0, 0.0)),
        _numbers(element.get("rpy"), (0.0, 0.0, 0.0)),
    )


def _mesh_path(uri: str) -> Path:
    parsed = urlparse(uri)
    if parsed.scheme not in ("", "file"):
        raise RuntimeError(f"website model generator requires an absolute mesh path, got {uri!r}")
    return Path(unquote(parsed.path if parsed.scheme else uri))


def _visual_mesh(visual: ET.Element, materials: dict[str, np.ndarray]) -> trimesh.Trimesh:
    geometry = visual.find("geometry")
    if geometry is None:
        raise RuntimeError("URDF visual has no geometry")
    mesh_node = geometry.find("mesh")
    if mesh_node is not None:
        loaded = trimesh.load(_mesh_path(mesh_node.attrib["filename"]), force="scene")
        mesh = loaded.to_geometry()
        scale = _numbers(mesh_node.get("scale"), (1.0, 1.0, 1.0))
        mesh.apply_scale(scale)
        return mesh
    box = geometry.find("box")
    cylinder = geometry.find("cylinder")
    sphere = geometry.find("sphere")
    if box is not None:
        mesh = trimesh.creation.box(extents=_numbers(box.get("size"), (1.0, 1.0, 1.0)))
    elif cylinder is not None:
        mesh = trimesh.creation.cylinder(
            radius=float(cylinder.attrib["radius"]),
            height=float(cylinder.attrib["length"]),
            sections=32,
        )
    elif sphere is not None:
        mesh = trimesh.creation.icosphere(radius=float(sphere.attrib["radius"]))
    else:
        raise RuntimeError("unsupported URDF visual geometry")
    material = visual.find("material")
    if material is not None:
        color = material.find("color")
        rgba = (
            _numbers(color.get("rgba"), (0.65, 0.68, 0.72, 1.0))
            if color is not None
            else materials.get(material.get("name", ""))
        )
        if rgba is not None:
            mesh.visual.face_colors = (np.asarray(rgba) * 255).astype(np.uint8)
    return mesh


def expand_robot_xacro(path: Path) -> ET.Element:
    try:
        xml = subprocess.run(
            ["xacro", str(path), "wrist_camera_sensor:=false"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        detail = exc.stderr.strip() if isinstance(exc, subprocess.CalledProcessError) else str(exc)
        raise RuntimeError(
            "could not expand the robot; run `just build` and invoke this script inside the "
            f"built ROS/Nix workspace: {detail}"
        ) from exc
    return ET.fromstring(xml)


def build_robot_scene(robot: ET.Element, joints: dict[str, float]) -> trimesh.Scene:
    """Bake the URDF visuals at one joint state into a flat world scene."""
    scene = _empty_scene()
    materials: dict[str, np.ndarray] = {}
    for material in robot.findall("material"):
        color = material.find("color")
        if color is not None and material.get("name"):
            materials[material.attrib["name"]] = np.asarray(
                _numbers(color.get("rgba"), (0.7, 0.7, 0.7, 1.0))
            )

    links = {link.attrib["name"]: link for link in robot.findall("link")}
    children: dict[str, list[ET.Element]] = {}
    child_names: set[str] = set()
    for joint in robot.findall("joint"):
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        children.setdefault(parent, []).append(joint)
        child_names.add(child)
    roots = [name for name in links if name not in child_names]
    if len(roots) != 1:
        raise RuntimeError(f"expected one URDF root link, got {roots}")

    transforms = {roots[0]: np.eye(4)}
    pending = [roots[0]]
    while pending:
        parent = pending.pop()
        for joint in children.get(parent, []):
            child = joint.find("child").attrib["link"]
            transform = transforms[parent] @ _origin(joint.find("origin"))
            position = joints.get(joint.attrib["name"], 0.0)
            axis = (
                _numbers(joint.find("axis").get("xyz"), (1.0, 0.0, 0.0))
                if joint.find("axis") is not None
                else (1.0, 0.0, 0.0)
            )
            if joint.get("type") in ("revolute", "continuous"):
                transform = transform @ _axis_angle(axis, position)
            elif joint.get("type") == "prismatic":
                transform = transform @ _T(tuple(position * component for component in axis))
            transforms[child] = transform
            pending.append(child)

    for link_name, link in links.items():
        for index, visual in enumerate(link.findall("visual")):
            mesh = _visual_mesh(visual, materials)
            _add(
                scene,
                f"{link_name}__visual_{index}",
                mesh,
                transforms[link_name] @ _origin(visual.find("origin")),
            )
    return scene


def build_workcell_scene(geometry: dict) -> trimesh.Scene:
    shelf = geometry["shelf"]
    tray = geometry["stock_tray"]
    lanes = geometry["lanes"]

    width = float(shelf["width_m"])
    depth = float(shelf["depth_m"])
    support_t = float(shelf["support_thickness_m"])
    div_t = float(shelf["divider_thickness_m"])
    div_h = float(shelf["divider_height_m"])
    ret_d = float(shelf["front_retainer_depth_m"])
    ret_h = float(shelf["front_retainer_height_m"])
    incline = math.radians(float(shelf["lane_incline_deg"]))
    roller_r = float(shelf["roller_radius_m"])
    roller_pitch = float(shelf["roller_pitch_m"])

    bed_datum = depth - ret_d
    bed_length = depth / math.cos(incline)
    bed_mid_h = (bed_datum - depth / 2.0) * math.tan(incline)
    bed_cy = depth / 2.0 - (support_t / 2.0) * math.sin(incline)
    bed_cz = bed_mid_h - (support_t / 2.0) * math.cos(incline)
    roller_count = int(depth / roller_pitch)

    scene = _empty_scene()
    tw = np.eye(4)  # shelf frame

    for name, x in shelf["divider_x_m"].items():
        _add(
            scene,
            f"divider_{name}",
            _box((div_t, depth, div_h), (float(x), depth / 2.0, div_h / 2.0), material="divider"),
            tw,
        )

    _add(
        scene,
        "front_retainer",
        _box((width, ret_d, ret_h), (0.0, depth - ret_d / 2.0, ret_h / 2.0), material="frame"),
        tw,
    )

    tray_c = tray["center_xyz_m"]
    _add(
        scene,
        "stock_tray",
        _box(
            (float(tray["width_m"]), float(tray["depth_m"]), float(tray["thickness_m"])),
            tuple(float(v) for v in tray_c),
            material="tray",
        ),
        tw,
    )

    _add(
        scene,
        "roller_bed",
        _box(
            (width, bed_length, support_t),
            (0.0, bed_cy, bed_cz),
            (-incline, 0, 0),
            material="shelf",
        ),
        tw,
    )

    for i in range(roller_count):
        roller_depth = (i + 0.5) * roller_pitch
        surface_z = (bed_datum - roller_depth) * math.tan(incline)
        _add(
            scene,
            f"roller_{i:02d}",
            _cylinder(
                roller_r,
                width,
                (
                    0.0,
                    roller_depth - roller_r * math.sin(incline),
                    surface_z - roller_r * math.cos(incline),
                ),
                (0.0, math.pi / 2.0, 0.0),
                material="roller",
            ),
            tw,
        )

    # Lane entrance markers (thin vertical posts at each lane center, front lip)
    for lane_name, lane in lanes.items():
        x = float(lane["center_x_m"])
        _add(
            scene,
            f"lane_marker_{lane_name}",
            _box((0.03, 0.03, 0.08), (x, 0.02, 0.04), material="orange"),
            tw,
        )

    # Overhead camera body at surveyed mount (visual only)
    cam_t = tw @ _T((0.0, -1.85, 2.05), (0.0, 55.0 * math.pi / 180.0, math.pi / 2.0))
    _add(scene, "overhead_camera", _box((0.10, 0.07, 0.05), material="camera"), cam_t)

    # Thin mast stub so the camera reads as mounted
    _add(
        scene,
        "camera_mast",
        _cylinder(0.025, 2.0, (0.0, -1.85, 1.0), material="frame"),
        tw,
    )

    return scene


def load_product_shapes(path: Path) -> dict[str, tuple[float, float]]:
    """Return catalog cylinder dimensions keyed by scenario geometry_key."""
    data = yaml.safe_load(path.read_text())
    shapes: dict[str, tuple[float, float]] = {}
    for entry in data["geometries"]:
        shape = entry["shape"]
        if shape["type"] != "cylinder":
            raise RuntimeError(
                f"website product rendering only supports cylinders, got {shape['type']!r} "
                f"for {entry['geometry_key']}"
            )
        shapes[entry["geometry_key"]] = (
            float(shape["radius_m"]),
            float(shape["height_m"]),
        )
    return shapes


def add_scenario_products(
    scene: trimesh.Scene,
    scenario: dict,
    shapes: dict[str, tuple[float, float]],
) -> None:
    """Add products in shelf-local coordinates to a workcell scene."""
    workcell_pose = tuple(float(value) for value in scenario["workcell_pose"])
    world_from_workcell = _T(workcell_pose[:3], workcell_pose[3:])
    workcell_from_world = np.linalg.inv(world_from_workcell)
    for product in scenario["products"]:
        radius, height = shapes[product["geometry_key"]]
        spawn_pose = tuple(float(value) for value in product["spawn_pose"])
        mesh = _paint_rgba(
            trimesh.creation.cylinder(radius=radius, height=height, sections=32),
            product["rgba"],
        )
        _add(
            scene,
            f"product_{product['model_name']}",
            mesh,
            workcell_from_world @ _T(spawn_pose[:3], spawn_pose[3:]),
        )


def _node_matrix(scene: trimesh.Scene, node_name: str) -> np.ndarray:
    try:
        return np.asarray(scene.graph.get(frame_to=node_name)[0], dtype=np.float64)
    except BaseException:
        return np.eye(4)


def merge_scenes(
    robot: trimesh.Scene,
    workcell: trimesh.Scene,
    workcell_pose: tuple[float, ...],
) -> trimesh.Scene:
    out = _empty_scene()
    tw_wc = _T(workcell_pose[:3], workcell_pose[3:])
    for name, geom in robot.geometry.items():
        _add(out, f"robot__{name}", geom.copy(), _node_matrix(robot, name))
    for name, geom in workcell.geometry.items():
        _add(out, f"workcell__{name}", geom.copy(), tw_wc @ _node_matrix(workcell, name))
    return out


def load_gripper(path: Path) -> dict:
    data = yaml.safe_load(path.read_text())
    finger = data["finger"]
    return {
        "body_size": tuple(float(v) for v in data["body"]["size_xyz_m"]),
        "body_center": tuple(float(v) for v in data["body"]["center_xyz_m"]),
        "finger_size": tuple(float(v) for v in finger["size_xyz_m"]),
        "finger_center": tuple(float(v) for v in finger["center_xyz_m"]),
        "left_origin": tuple(float(v) for v in finger["left"]["origin_xyz_m"]),
        "right_origin": tuple(float(v) for v in finger["right"]["origin_xyz_m"]),
        "open_target_m": float(data["attachment"]["open_target_m"]),
    }


def export_glb(scene: trimesh.Scene, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = scene.export(file_type="glb")
    path.write_bytes(data)
    print(f"wrote {path} ({path.stat().st_size} bytes, {len(scene.geometry)} meshes)")


def main() -> int:
    repo = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-xacro",
        type=Path,
        default=repo
        / "ros_ws"
        / "src"
        / "restocker_description"
        / "urdf"
        / "restocker.urdf.xacro",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=repo / "website" / "static" / "models",
        help="Directory for robot.glb, workcell.glb, restocking_cell.glb",
    )
    parser.add_argument(
        "--workcell-yaml",
        type=Path,
        default=repo
        / "ros_ws"
        / "src"
        / "restocker_description"
        / "config"
        / "workcell_geometry.yaml",
    )
    parser.add_argument(
        "--gripper-yaml",
        type=Path,
        default=repo
        / "ros_ws"
        / "src"
        / "restocker_description"
        / "config"
        / "gripper_geometry.yaml",
    )
    parser.add_argument(
        "--product-catalog",
        type=Path,
        default=repo
        / "ros_ws"
        / "src"
        / "restocker_description"
        / "config"
        / "product_collision_catalog.yaml",
    )
    parser.add_argument(
        "--scenario-yaml",
        type=Path,
        default=repo
        / "ros_ws"
        / "src"
        / "restocker_gazebo"
        / "config"
        / "dense_restock_products.yaml",
    )
    args = parser.parse_args()

    geometry = yaml.safe_load(args.workcell_yaml.read_text())
    scenario = yaml.safe_load(args.scenario_yaml.read_text())
    product_shapes = load_product_shapes(args.product_catalog)
    gripper = load_gripper(args.gripper_yaml) if args.gripper_yaml.is_file() else GRIPPER_DEFAULT
    open_m = gripper["open_target_m"]
    home_joints = {**POSE_HOME, "left_finger_joint": open_m, "right_finger_joint": open_m}
    reach_joints = {**POSE_REACH_LANE, "left_finger_joint": open_m, "right_finger_joint": open_m}

    robot_description = expand_robot_xacro(args.robot_xacro)
    robot_home = build_robot_scene(robot_description, home_joints)
    robot_reach = build_robot_scene(robot_description, reach_joints)

    # robot.glb carries both poses as name-prefixed meshes (home__*, reach__*).
    robot = _empty_scene()
    for name, geom in robot_home.geometry.items():
        _add(robot, f"home__{name}", geom.copy(), _node_matrix(robot_home, name))
    for name, geom in robot_reach.geometry.items():
        _add(robot, f"reach__{name}", geom.copy(), _node_matrix(robot_reach, name))

    workcell = build_workcell_scene(geometry)
    add_scenario_products(workcell, scenario, product_shapes)
    # Combined cell uses the reach pose so the arm reads as working in the cell.
    combined = merge_scenes(
        robot_reach,
        workcell,
        tuple(float(value) for value in scenario["workcell_pose"]),
    )

    out = args.out_dir
    export_glb(robot, out / "robot.glb")
    export_glb(workcell, out / "workcell.glb")
    export_glb(combined, out / "restocking_cell.glb")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
