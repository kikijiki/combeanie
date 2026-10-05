#!/usr/bin/env python3
"""Forward kinematics for the restocker, used to locate the arm at a recorded instant."""

import math
from pathlib import Path

import numpy as np


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1.0]])


def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1.0]])


def tr(x, y, z):
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


# link name -> (collision shape, half extents or (radius, half length), local centre)
SHAPES = {
    "carriage": ("box", (0.25, 0.23, 0.06), (0, 0, 0.06)),
    # Conservative boxes around the UR10e collision-mesh capsules used by depth_obstacle_node.
    "base_link_inertia": ("box", (0.099, 0.099, 0.1487), (0.0027, -0.0022, 0.0496)),
    "shoulder_link": ("box", (0.130, 0.130, 0.2178), (0.0009, 0.0108, 0.0061)),
    "upper_arm_link": ("box", (0.5226, 0.148, 0.148), (-0.2979, 0, 0.2220)),
    "forearm_link": ("box", (0.4333, 0.095, 0.095), (-0.2795, -0.0003, 0.0240)),
    "wrist_1_link": ("box", (0.085, 0.1471, 0.085), (-0.0008, -0.0045, 0.0025)),
    "wrist_2_link": ("box", (0.073, 0.1355, 0.073), (0.0009, 0.0055, 0.0028)),
    "wrist_3_link": ("box", (0.056, 0.1048, 0.056), (0.0004, -0.0034, -0.0158)),
    "gripper": ("box", (0.08, 0.055, 0.035), (0, 0, 0.035)),
    "left_finger": ("box", (0.0175, 0.009, 0.07), (0, 0, 0.07)),
    "right_finger": ("box", (0.0175, 0.009, 0.07), (0, 0, 0.07)),
    "wrist_camera_link": ("box", (0.025, 0.045, 0.02), (0, 0, 0)),
}


def link_frames(rail, q, left=0.0, right=0.0):
    """Return world transforms using ur_description's UR10e default kinematics."""
    f = {}
    t = tr(0, 0, 0)  # world -> rail_base
    f["rail_base"] = t
    t = t @ tr(rail, 0, 0.16)
    f["carriage"] = t
    t = t @ tr(0, 0, 0.12)
    f["base_link"] = t
    t = t @ rz(np.pi)
    f["base_link_inertia"] = t
    t = t @ tr(0, 0, 0.1807) @ rz(q[0])
    f["shoulder_link"] = t
    t = t @ rx(np.pi / 2.0) @ rz(q[1])
    f["upper_arm_link"] = t
    t = t @ tr(-0.6127, 0, 0) @ rz(q[2])
    f["forearm_link"] = t
    t = t @ tr(-0.57155, 0, 0.17415) @ rz(q[3])
    f["wrist_1_link"] = t
    t = t @ tr(0, -0.11985, 0) @ rx(np.pi / 2.0) @ rz(q[4])
    f["wrist_2_link"] = t
    t = t @ tr(0, 0.11655, 0) @ rx(np.pi / 2.0) @ ry(np.pi) @ rz(np.pi) @ rz(q[5])
    f["wrist_3_link"] = t
    t = t @ ry(-np.pi / 2.0) @ rz(-np.pi / 2.0)
    f["flange"] = t
    t0 = t @ rx(np.pi / 2.0) @ rz(np.pi / 2.0)
    f["tool0"] = t0
    f["gripper"] = t0
    f["left_finger"] = t0 @ tr(0.0, 0.03 + left, 0.07)
    f["right_finger"] = t0 @ tr(0.0, -0.03 - right, 0.07)
    f["wrist_camera_link"] = t0 @ tr(0.11, 0, 0.07) @ ry(-np.pi / 2.0)
    return f


def corners(shape, dims, centre):
    if shape == "box":
        hx, hy, hz = dims
    else:
        hx = hy = dims[0]
        hz = dims[1]
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                pts.append([centre[0] + sx * hx, centre[1] + sy * hy, centre[2] + sz * hz, 1.0])
    return np.array(pts).T


def world_points(rail, q, left=0.0, right=0.0):
    frames = link_frames(rail, q, left, right)
    out = {}
    for name, (shape, dims, centre) in SHAPES.items():
        if name not in frames:
            continue
        out[name] = (frames[name] @ corners(shape, dims, centre))[:3].T
    out["tool0"] = frames["tool0"][:3, 3].reshape(1, 3)
    return out


# Workcell collision boxes in world coordinates, derived from the installed geometry file.
import yaml  # noqa: E402

REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
INSTALL_ROOT = REPOSITORY_ROOT / "ros_ws" / "install"
GEOM_PATH = (
    INSTALL_ROOT
    / "restocker_description"
    / "share"
    / "restocker_description"
    / "config"
    / "workcell_geometry.yaml"
)
SCENARIO_PATH = (
    INSTALL_ROOT
    / "restocker_gazebo"
    / "share"
    / "restocker_gazebo"
    / "config"
    / "baseline_products.yaml"
)


def _box(cx, cy, cz, sx, sy, sz, name):
    return (
        name,
        np.array([cx - sx / 2, cy - sy / 2, cz - sz / 2]),
        np.array([cx + sx / 2, cy + sy / 2, cz + sz / 2]),
    )


def build_obstacles():
    with open(GEOM_PATH) as handle:
        g = yaml.safe_load(handle)
    with open(SCENARIO_PATH) as handle:
        scenario = yaml.safe_load(handle)
    wx, wy, wz = (float(v) for v in scenario["workcell_pose"][:3])
    shelf = g["shelf"]
    depth = float(shelf["depth_m"])
    width = float(shelf["width_m"])
    st = float(shelf["support_thickness_m"])
    # The lane floor is an inclined roller bed. It is bounded by the box containing the slab swept
    # through its pitch (conservative, exact at the two ends). The surface is level with the shelf
    # datum at the front rail's face and rises behind it.
    incline = math.radians(float(shelf["lane_incline_deg"]))
    datum = depth - float(shelf["front_retainer_depth_m"])
    bed_top = datum * math.tan(incline)
    bed_bottom = (datum - depth) * math.tan(incline) - st / math.cos(incline)
    out = [
        _box(
            wx,
            wy + depth / 2,
            wz + (bed_top + bed_bottom) / 2,
            width,
            depth,
            bed_top - bed_bottom,
            "roller_bed",
        ),
        _box(
            wx,
            wy + depth - shelf["front_retainer_depth_m"] / 2,
            wz + shelf["front_retainer_height_m"] / 2,
            width,
            shelf["front_retainer_depth_m"],
            shelf["front_retainer_height_m"],
            "front_retainer",
        ),
        _box(0.0, 0.0, -0.05, 8.0, 5.0, 0.1, "floor"),
        _box(wx, wy + 0.42, wz + 0.95, 0.10, 0.07, 0.05, "overhead_camera"),
    ]
    tray = g["stock_tray"]
    tc = [float(v) for v in tray["center_xyz_m"]]
    out.append(
        _box(
            wx + tc[0],
            wy + tc[1],
            wz + tc[2],
            tray["width_m"],
            tray["depth_m"],
            tray["thickness_m"],
            "stock_tray",
        )
    )
    for name, x in shelf["divider_x_m"].items():
        out.append(
            _box(
                wx + float(x),
                wy + depth / 2,
                wz + shelf["divider_height_m"] / 2,
                shelf["divider_thickness_m"],
                depth,
                shelf["divider_height_m"],
                name,
            )
        )
    return out


OBSTACLES = build_obstacles()
