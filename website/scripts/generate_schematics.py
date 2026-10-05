#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Generate educational 2D schematics from surveyed workcell / robot geometry.

Reads metres from:
  restocker_description/config/workcell_geometry.yaml
  restocker_description/config/gripper_geometry.yaml
  restocker_description/config/product_collision_catalog.yaml
  restocker_gazebo/config/baseline_products.yaml (workcell_pose)
  restocker_description/urdf/{rail,arm,sensors,workcell}.xacro (link sizes / mounts)

Writes SVG files under website/static/img/schematics/. Does not invent dimensions.
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable
import math
from pathlib import Path
import re
from xml.sax.saxutils import escape

import yaml

# Site primary teal from website/src/css/custom.css
TEAL = "#1f6b5c"
TEAL_SOFT = "#d7ebe6"
INK = "#1a1f24"
MUTED = "#5a6570"
LINE = "#2c3640"
FILL_RAIL = "#e8eaed"
FILL_ARM = "#f4f6f8"
FILL_SHELF = "#eef2f4"
FILL_TRAY = "#dfe5ea"
FILL_RETAINER = "#2d343c"
FILL_PRODUCT = "#c5ddd7"
ACCENT = TEAL


def repo_root_from_script(script: Path) -> Path:
    # website/scripts/this.py -> repo root
    return script.resolve().parents[2]


def load_yaml(path: Path) -> dict:
    with path.open(encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def parse_first_box(xacro: str, link_fragment: str) -> tuple[float, float, float]:
    """Return the first <box size="x y z"/> under a link whose name contains link_fragment."""
    link_re = re.compile(
        rf'<link name="[^"]*{re.escape(link_fragment)}[^"]*">(?P<body>.*?)</link>',
        re.DOTALL,
    )
    match = link_re.search(xacro)
    if not match:
        raise RuntimeError(f"link containing {link_fragment!r} not found")
    box = re.search(
        r'<box size="([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)"/>', match.group("body")
    )
    if not box:
        raise RuntimeError(f"box geometry missing for {link_fragment}")
    return tuple(float(v) for v in box.groups())  # type: ignore[return-value]


def parse_first_cylinder(xacro: str, link_fragment: str) -> tuple[float, float]:
    link_re = re.compile(
        rf'<link name="[^"]*{re.escape(link_fragment)}[^"]*">(?P<body>.*?)</link>',
        re.DOTALL,
    )
    match = link_re.search(xacro)
    if not match:
        raise RuntimeError(f"link containing {link_fragment!r} not found")
    cyl = re.search(
        r'<cylinder radius="([0-9.eE+-]+)" length="([0-9.eE+-]+)"/>', match.group("body")
    )
    if not cyl:
        raise RuntimeError(f"cylinder geometry missing for {link_fragment}")
    return float(cyl.group(1)), float(cyl.group(2))


def parse_joint_origin_z(xacro: str, joint_fragment: str) -> float:
    joint_re = re.compile(
        rf'<joint name="[^"]*{re.escape(joint_fragment)}[^"]*"[^>]*>\s*'
        rf'(?:.*?)<origin xyz="([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)"',
        re.DOTALL,
    )
    match = joint_re.search(xacro)
    if not match:
        raise RuntimeError(f"joint origin for {joint_fragment!r} not found")
    return float(match.group(3))


def parse_overhead_camera_xyz(workcell_xacro: str) -> tuple[float, float, float]:
    match = re.search(
        r'overhead_camera_mount_joint.*?<origin xyz="([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)"',
        workcell_xacro,
        re.DOTALL,
    )
    if not match:
        raise RuntimeError("overhead camera mount origin not found")
    return tuple(float(v) for v in match.groups())  # type: ignore[return-value]


def parse_wrist_camera_xyz(sensors_xacro: str) -> tuple[float, float, float]:
    match = re.search(
        r'wrist_camera_mount_joint.*?<origin xyz="([0-9.eE+-]+) ([0-9.eE+-]+) ([0-9.eE+-]+)"',
        sensors_xacro,
        re.DOTALL,
    )
    if not match:
        raise RuntimeError("wrist camera mount origin not found")
    return tuple(float(v) for v in match.groups())  # type: ignore[return-value]


class Svg:
    def __init__(self, width: float, height: float, title: str) -> None:
        self.width = width
        self.height = height
        self.title = title
        self.parts: list[str] = []

    def add(self, fragment: str) -> None:
        self.parts.append(fragment)

    def rect(
        self,
        x: float,
        y: float,
        w: float,
        h: float,
        *,
        fill: str = "none",
        stroke: str = LINE,
        sw: float = 1.6,
        rx: float = 0,
        opacity: float = 1.0,
        dash: str | None = None,
    ) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<rect x="{x:.2f}" y="{y:.2f}" width="{w:.2f}" height="{h:.2f}" '
            f'rx="{rx:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" '
            f'opacity="{opacity}"{dash_attr}/>'
        )

    def line(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        *,
        stroke: str = LINE,
        sw: float = 1.6,
        dash: str | None = None,
    ) -> None:
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(
            f'<line x1="{x1:.2f}" y1="{y1:.2f}" x2="{x2:.2f}" y2="{y2:.2f}" '
            f'stroke="{stroke}" stroke-width="{sw}"{dash_attr}/>'
        )

    def circle(
        self,
        cx: float,
        cy: float,
        r: float,
        *,
        fill: str = ACCENT,
        stroke: str = LINE,
        sw: float = 1.2,
    ) -> None:
        self.add(
            f'<circle cx="{cx:.2f}" cy="{cy:.2f}" r="{r:.2f}" fill="{fill}" '
            f'stroke="{stroke}" stroke-width="{sw}"/>'
        )

    def poly(
        self,
        points: Iterable[tuple[float, float]],
        *,
        fill: str = "none",
        stroke: str = LINE,
        sw: float = 1.6,
    ) -> None:
        pts = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
        self.add(f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>')

    def text(
        self,
        x: float,
        y: float,
        content: str,
        *,
        size: float = 13,
        fill: str = INK,
        weight: str = "600",
        anchor: str = "start",
        family: str = "IBM Plex Sans, Segoe UI, sans-serif",
    ) -> None:
        self.add(
            f'<text x="{x:.2f}" y="{y:.2f}" fill="{fill}" font-size="{size}" '
            f'font-weight="{weight}" text-anchor="{anchor}" font-family="{family}">'
            f"{escape(content)}</text>"
        )

    def label(
        self,
        x: float,
        y: float,
        name: str,
        *,
        dx: float = 10,
        dy: float = -8,
        size: float = 12,
    ) -> None:
        self.circle(x, y, 3.5, fill=ACCENT, stroke=TEAL, sw=1)
        self.line(x, y, x + dx, y + dy, stroke=ACCENT, sw=1.2)
        self.text(x + dx + 4, y + dy + 4, name, size=size, fill=TEAL, weight="700")

    def caption(self, text: str, y: float = 22) -> None:
        self.text(24, y, text, size=16, fill=INK, weight="700")

    def note(self, text: str, x: float, y: float, *, size: float = 11) -> None:
        self.text(x, y, text, size=size, fill=MUTED, weight="500")

    def render(self) -> str:
        body = "\n  ".join(self.parts)
        return (
            f'<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.width:.0f}" '
            f'height="{self.height:.0f}" viewBox="0 0 {self.width:.0f} {self.height:.0f}" '
            f'role="img" aria-labelledby="title">\n'
            f'  <title id="title">{escape(self.title)}</title>\n'
            f'  <rect width="100%" height="100%" fill="#fbfcfd"/>\n'
            f"  {body}\n"
            f"</svg>\n"
        )


def write_svg(path: Path, svg: Svg) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(svg.render(), encoding="utf-8")
    print(f"wrote {path}")


def robot_side_and_front(geom: dict, gripper: dict, urdf_dir: Path, out: Path) -> None:
    rail = (urdf_dir / "rail.xacro").read_text(encoding="utf-8")
    rail_box = parse_first_box(rail, "rail_base")
    carriage_box = parse_first_box(rail, "carriage")
    z_rail_joint = parse_joint_origin_z(rail, "rail_joint")
    # UR10e nominal geometry from ur_description 3.5.1 default_kinematics.yaml.
    mount_height = 0.12
    shoulder_height = 0.1807
    upper_length = 0.6127
    forearm_length = 0.57155
    z_carriage = z_rail_joint
    z_base = z_carriage + mount_height
    z_shoulder = z_base + shoulder_height
    grasp = gripper["grasp_center"]["xyz_m"]
    body = gripper["body"]["size_xyz_m"]
    finger = gripper["finger"]["size_xyz_m"]

    s = 170.0
    svg = Svg(980, 560, "Robot orthographic schematic")
    svg.caption("Rail-mounted Universal Robots UR10e")
    svg.note(
        "UR10e dimensions from ur_description 3.5.1; rail and gripper dimensions remain "
        "project-owned.",
        24,
        42,
    )

    # Side view: a bent pose keeps the two UR link spans legible.
    ox, oy = 70.0, 490.0

    def sy(y_m: float) -> float:
        return ox + y_m * s

    def sz(z_m: float) -> float:
        return oy - z_m * s

    svg.text(ox - 20, 72, "Side (YZ)", size=14, fill=MUTED)
    rail_y, rail_z = rail_box[1], rail_box[2]
    svg.rect(sy(-rail_y / 2), sz(rail_z), rail_y * s, rail_z * s, fill=FILL_RAIL)
    svg.rect(
        sy(-carriage_box[1] / 2),
        sz(z_carriage + carriage_box[2]),
        carriage_box[1] * s,
        carriage_box[2] * s,
        fill=TEAL_SOFT,
        stroke=TEAL,
    )
    svg.rect(sy(-0.095), sz(z_shoulder), 0.19 * s, shoulder_height * s, fill=FILL_ARM, rx=8)

    shoulder = (0.0, z_shoulder)
    elbow = (0.56, z_shoulder + 0.25)
    wrist = (0.76, z_shoulder + 0.785)
    tool = (0.88, wrist[1] + 0.03)
    grasp_point = (tool[0] + grasp[2], tool[1])
    svg.line(
        sy(*shoulder[:1]), sz(shoulder[1]), sy(elbow[0]), sz(elbow[1]), stroke=FILL_ARM, sw=34
    )
    svg.line(sy(elbow[0]), sz(elbow[1]), sy(wrist[0]), sz(wrist[1]), stroke=FILL_ARM, sw=29)
    svg.line(sy(wrist[0]), sz(wrist[1]), sy(tool[0]), sz(tool[1]), stroke=FILL_ARM, sw=22)
    for y_m, z_m in (shoulder, elbow, wrist):
        svg.circle(sy(y_m), sz(z_m), 12, fill=TEAL_SOFT, stroke=TEAL, sw=2)
    svg.rect(sy(tool[0]), sz(tool[1] + body[1] / 2), body[2] * s, body[1] * s, fill=FILL_RAIL)
    svg.line(
        sy(tool[0] + body[2]),
        sz(tool[1] + 0.035),
        sy(grasp_point[0]),
        sz(tool[1] + 0.035),
        stroke=TEAL,
        sw=5,
    )
    svg.line(
        sy(tool[0] + body[2]),
        sz(tool[1] - 0.035),
        sy(grasp_point[0]),
        sz(tool[1] - 0.035),
        stroke=TEAL,
        sw=5,
    )

    svg.label(sy(0), sz(0), "rail_base", dx=18, dy=14)
    svg.label(sy(tool[0]), sz(tool[1]), "tool0", dx=18, dy=-8)
    svg.label(sy(grasp_point[0]), sz(grasp_point[1]), "grasp_center", dx=10, dy=-18)
    svg.note(f"rail {rail_box[0]:.1f} m long (into page)", ox, oy + 28)
    svg.note(f"UR10e upper arm {upper_length:.4f} m · forearm {forearm_length:.5f} m", ox, oy + 44)

    svg.line(sy(-0.35), sz(0), sy(0.45), sz(0), stroke=MUTED, sw=1, dash="4 3")
    svg.line(sy(0), sz(0), sy(0), sz(1.45), stroke=MUTED, sw=1, dash="4 3")
    svg.text(sy(0.46), sz(0) + 4, "+Y", size=11, fill=MUTED)
    svg.text(sy(0) - 18, sz(1.48), "+Z", size=11, fill=MUTED)

    fx, fy = 540.0, 500.0

    def sx(x_m: float) -> float:
        return fx + x_m * s * 0.55

    def szf(z_m: float) -> float:
        return fy - z_m * s

    svg.text(fx - 40, 72, "Front (XZ)", size=14, fill=MUTED)
    svg.rect(
        sx(-rail_box[0] / 2),
        szf(rail_box[2]),
        rail_box[0] * s * 0.55,
        rail_box[2] * s,
        fill=FILL_RAIL,
    )
    svg.rect(
        sx(-carriage_box[0] / 2),
        szf(z_carriage + carriage_box[2]),
        carriage_box[0] * s * 0.55,
        carriage_box[2] * s,
        fill=TEAL_SOFT,
        stroke=TEAL,
    )
    svg.rect(
        sx(-0.095), szf(z_shoulder), 0.19 * s * 0.55, shoulder_height * s, fill=FILL_ARM, rx=6
    )
    shoulder_f = (0.0, z_shoulder)
    elbow_f = (0.30, z_shoulder + 0.53)
    wrist_f = (-0.10, z_shoulder + 0.93)
    tool_f = (-0.10, wrist_f[1] + 0.16)
    svg.line(
        sx(shoulder_f[0]),
        szf(shoulder_f[1]),
        sx(elbow_f[0]),
        szf(elbow_f[1]),
        stroke=FILL_ARM,
        sw=34,
    )
    svg.line(
        sx(elbow_f[0]), szf(elbow_f[1]), sx(wrist_f[0]), szf(wrist_f[1]), stroke=FILL_ARM, sw=29
    )
    for x_m, z_m in (shoulder_f, elbow_f, wrist_f):
        svg.circle(sx(x_m), szf(z_m), 12, fill=TEAL_SOFT, stroke=TEAL, sw=2)
    svg.rect(
        sx(tool_f[0] - body[0] / 2),
        szf(tool_f[1] + body[2]),
        body[0] * s * 0.55,
        body[2] * s,
        fill=FILL_RAIL,
    )
    open_m = gripper["finger"]["left"]["origin_xyz_m"][1]
    svg.rect(
        sx(tool_f[0] - open_m - finger[0] / 2),
        szf(tool_f[1] + 0.07 + finger[2]),
        finger[0] * s * 0.55,
        finger[2] * s,
        fill=TEAL_SOFT,
        stroke=TEAL,
    )
    svg.rect(
        sx(tool_f[0] + open_m - finger[0] / 2),
        szf(tool_f[1] + 0.07 + finger[2]),
        finger[0] * s * 0.55,
        finger[2] * s,
        fill=TEAL_SOFT,
        stroke=TEAL,
    )

    svg.label(sx(0), szf(0), "rail_base", dx=14, dy=16)
    svg.label(sx(tool_f[0]), szf(tool_f[1]), "tool0", dx=16, dy=-4)
    svg.label(sx(tool_f[0]), szf(tool_f[1] + grasp[2]), "grasp_center", dx=18, dy=-16)
    svg.note(f"rail span {rail_box[0]:.1f} m · carriage {carriage_box[0]:.2f} m", fx - 40, fy + 28)
    svg.note(
        f"grasp_center at tool0 + ({grasp[0]:.2f}, {grasp[1]:.2f}, {grasp[2]:.2f}) m",
        fx - 40,
        fy + 44,
    )

    _ = geom
    write_svg(out / "robot-orthographic.svg", svg)


def workcell_top_and_side(
    geom: dict,
    catalog: dict,
    workcell_xacro: str,
    sensors_xacro: str,
    workcell_pose: list[float],
    out: Path,
) -> None:
    shelf = geom["shelf"]
    tray = geom["stock_tray"]
    lanes = geom["lanes"]
    cam = parse_overhead_camera_xyz(workcell_xacro)
    wrist_off = parse_wrist_camera_xyz(sensors_xacro)

    width = shelf["width_m"]
    depth = shelf["depth_m"]
    retainer_d = shelf["front_retainer_depth_m"]
    retainer_h = shelf["front_retainer_height_m"]
    incline = math.radians(shelf["lane_incline_deg"])
    divider_h = shelf["divider_height_m"]
    divider_t = shelf["divider_thickness_m"]
    dividers = shelf["divider_x_m"]

    large = next(g for g in catalog["geometries"] if g["geometry_key"] == "bottle.large.standard")
    bottle_r = large["shape"]["radius_m"]
    bottle_h = large["shape"]["height_m"]

    svg = Svg(1000, 780, "Workcell top and side schematic")
    svg.caption("Workcell schematic (shelf frame)")
    svg.note(
        "From workcell_geometry.yaml and workcell.xacro. Shelf origin is the rear insertion "
        "datum.",
        24,
        42,
    )

    # --- Top view: X right, Y down-page (customer at bottom). Include tray/camera at -Y. ---
    s = 145.0
    # Place y=0 so tray (y≈-1.35) and camera (y≈-1.85) sit below the title band.
    ox, oy = 500.0, 360.0

    def tx(x_m: float) -> float:
        return ox + x_m * s

    def ty(y_m: float) -> float:
        return oy + y_m * s

    svg.text(40, 72, "Top view", size=14, fill=MUTED)

    # Overhead camera (behind tray)
    svg.circle(tx(cam[0]), ty(cam[1]), 7, fill=ACCENT)
    svg.text(tx(cam[0]) + 12, ty(cam[1]) + 4, "overhead_camera", size=11, fill=TEAL, weight="700")
    svg.line(tx(cam[0]), ty(cam[1]), tx(cam[0]), ty(cam[1] + 0.45), stroke=ACCENT, sw=1.4)

    # Stock tray
    tc = tray["center_xyz_m"]
    tw, td = tray["width_m"], tray["depth_m"]
    svg.rect(
        tx(tc[0] - tw / 2),
        ty(tc[1] - td / 2),
        tw * s,
        td * s,
        fill=FILL_TRAY,
        stroke=TEAL,
        sw=1.8,
    )
    svg.text(
        tx(tc[0]), ty(tc[1]) + 4, "stock_tray", size=12, fill=TEAL, weight="700", anchor="middle"
    )

    # Shelf footprint
    svg.rect(tx(-width / 2), ty(0), width * s, depth * s, fill=FILL_SHELF)
    for _name, x in dividers.items():
        svg.rect(
            tx(x - divider_t / 2),
            ty(0),
            divider_t * s,
            depth * s,
            fill="#c8d2d9",
            stroke=LINE,
            sw=1,
        )
    svg.rect(
        tx(-width / 2),
        ty(depth - retainer_d),
        width * s,
        retainer_d * s,
        fill=FILL_RETAINER,
        stroke=LINE,
    )
    for name, lane in lanes.items():
        svg.text(
            tx(lane["center_x_m"]),
            ty(depth / 2) + 4,
            name.replace("lane_", "L"),
            size=11,
            fill=TEAL,
            weight="700",
            anchor="middle",
        )

    svg.label(tx(0), ty(0), "shelf origin", dx=14, dy=-14)
    note_y = ty(depth) + 22
    svg.note(
        f"shelf {width:.1f} m × {depth:.1f} m · 6 lanes · retainer {retainer_d:.2f} m deep",
        40,
        note_y,
    )
    svg.note(
        f"tray {tw:.2f}×{td:.2f} m at shelf ({tc[0]:.2f}, {tc[1]:.2f}, {tc[2]:.2f}) m",
        40,
        note_y + 16,
    )
    svg.note(
        f"overhead_camera mount at shelf ({cam[0]:.2f}, {cam[1]:.2f}, {cam[2]:.2f}) m",
        40,
        note_y + 32,
    )
    svg.note(
        f"+Y toward customer / front retainer · wrist camera offset from tool0 {wrist_off} m",
        40,
        note_y + 48,
    )

    # --- Side view: Y right, Z up (compact inset under notes) ---
    ss = 180.0
    bx, by = 80.0, 740.0

    def sx(y_m: float) -> float:
        return bx + y_m * ss

    def sz(z_m: float) -> float:
        return by - z_m * ss

    svg.text(bx, note_y + 78, "Side view (lane incline)", size=14, fill=MUTED)
    bed_datum = depth - retainer_d
    rear_z = bed_datum * math.tan(incline)
    svg.line(sx(0), sz(rear_z), sx(bed_datum), sz(0), stroke=LINE, sw=2.2)
    svg.line(sx(0), sz(-0.05), sx(depth), sz(-0.05), stroke=MUTED, sw=1, dash="4 3")
    svg.rect(
        sx(depth - retainer_d),
        sz(retainer_h),
        retainer_d * ss,
        retainer_h * ss,
        fill=FILL_RETAINER,
    )
    svg.rect(
        sx(0),
        sz(divider_h),
        depth * ss,
        divider_h * ss,
        fill="none",
        stroke=MUTED,
        sw=1,
        dash="3 3",
    )
    lean = (bottle_h - retainer_h) * math.tan(incline)
    base_y = bed_datum - bottle_r
    svg.rect(
        sx(base_y - bottle_r),
        sz(bottle_h * math.cos(incline)),
        2 * bottle_r * ss,
        bottle_h * math.cos(incline) * ss,
        fill=FILL_PRODUCT,
        stroke=TEAL,
    )
    svg.note(
        f"incline {shelf['lane_incline_deg']:.1f}° · retainer {retainer_h:.2f} m · "
        f"large bottle {bottle_h:.3f} m leans {lean:.3f} m past rail face",
        bx,
        by + 22,
    )
    svg.note(
        f"rear bed rise {rear_z:.3f} m · workcell_pose world {workcell_pose[:3]}",
        bx,
        by + 38,
    )
    svg.label(sx(bed_datum), sz(0), "shelf z=0 at retainer face", dx=12, dy=16)
    write_svg(out / "workcell-top-side.svg", svg)


def combined_layout(
    geom: dict,
    workcell_xacro: str,
    workcell_pose: list[float],
    urdf_dir: Path,
    out: Path,
) -> None:
    shelf = geom["shelf"]
    tray = geom["stock_tray"]
    cam = parse_overhead_camera_xyz(workcell_xacro)
    rail = (urdf_dir / "rail.xacro").read_text(encoding="utf-8")
    rail_box = parse_first_box(rail, "rail_base")
    carriage_box = parse_first_box(rail, "carriage")

    # World frame: rail_base at origin; shelf at workcell_pose
    sx0, sy0, sz0 = workcell_pose[0], workcell_pose[1], workcell_pose[2]
    width = shelf["width_m"]
    depth = shelf["depth_m"]
    tc = tray["center_xyz_m"]

    svg = Svg(980, 620, "Workcell and robot plan layout")
    svg.caption("Combined plan: robot behind the shelf")
    svg.note(
        "World frame. rail_base at origin; shelf from baseline workcell_pose. Arm works from "
        "rear.",
        24,
        42,
    )

    s = 150.0
    ox, oy = 490.0, 300.0

    def wx(x_m: float) -> float:
        return ox + x_m * s

    def wy(y_m: float) -> float:
        # Page +Y down; world +Y up-page would flip; keep world +Y down-page toward customer
        return oy - y_m * s

    # Floor grid hint
    for g in range(-3, 4):
        svg.line(wx(g), wy(-2.4), wx(g), wy(1.8), stroke="#eef1f3", sw=1)
    for g in range(-3, 3):
        svg.line(wx(-2.4), wy(g * 0.5), wx(2.4), wy(g * 0.5), stroke="#eef1f3", sw=1)

    # Rail
    svg.rect(
        wx(-rail_box[0] / 2),
        wy(rail_box[1] / 2),
        rail_box[0] * s,
        rail_box[1] * s,
        fill=FILL_RAIL,
        stroke=LINE,
        sw=1.8,
    )
    svg.rect(
        wx(-carriage_box[0] / 2),
        wy(carriage_box[1] / 2),
        carriage_box[0] * s,
        carriage_box[1] * s,
        fill=TEAL_SOFT,
        stroke=TEAL,
        sw=1.8,
    )
    svg.text(wx(0), wy(0) - 14, "carriage", size=11, fill=TEAL, weight="700", anchor="middle")
    svg.text(wx(0), wy(0) + 18, "rail_base", size=11, fill=TEAL, weight="700", anchor="middle")

    # Arm reach sketch toward shelf (+Y): schematic links, not a joint solution
    svg.line(wx(0), wy(0), wx(0), wy(sy0 * 0.55), stroke=ACCENT, sw=3.0)
    svg.line(wx(0), wy(sy0 * 0.55), wx(0.15), wy(sy0), stroke=ACCENT, sw=2.2)
    svg.circle(wx(0), wy(0.28), 5, fill=ACCENT)

    # Shelf in world
    svg.rect(
        wx(sx0 - width / 2),
        wy(sy0 + depth),
        width * s,
        depth * s,
        fill=FILL_SHELF,
        stroke=LINE,
        sw=1.8,
    )
    # Dividers
    for x in shelf["divider_x_m"].values():
        svg.rect(
            wx(sx0 + x - shelf["divider_thickness_m"] / 2),
            wy(sy0 + depth),
            shelf["divider_thickness_m"] * s,
            depth * s,
            fill="#c8d2d9",
            stroke=LINE,
            sw=0.8,
        )
    # Front retainer
    svg.rect(
        wx(sx0 - width / 2),
        wy(sy0 + depth),
        width * s,
        shelf["front_retainer_depth_m"] * s,
        fill=FILL_RETAINER,
    )
    svg.text(
        wx(sx0),
        wy(sy0 + depth / 2) + 4,
        "shelf (6 lanes)",
        size=12,
        fill=INK,
        weight="700",
        anchor="middle",
    )

    # Stock tray world
    tray_wx = sx0 + tc[0]
    tray_wy = sy0 + tc[1]
    svg.rect(
        wx(tray_wx - tray["width_m"] / 2),
        wy(tray_wy + tray["depth_m"] / 2),
        tray["width_m"] * s,
        tray["depth_m"] * s,
        fill=FILL_TRAY,
        stroke=TEAL,
        sw=1.8,
    )
    svg.text(
        wx(tray_wx),
        wy(tray_wy) + 4,
        "stock_tray",
        size=12,
        fill=TEAL,
        weight="700",
        anchor="middle",
    )

    # Overhead camera world
    cam_wx = sx0 + cam[0]
    cam_wy = sy0 + cam[1]
    svg.circle(wx(cam_wx), wy(cam_wy), 7, fill=ACCENT)
    svg.text(wx(cam_wx) + 12, wy(cam_wy) + 4, "overhead_camera", size=11, fill=TEAL, weight="700")

    svg.label(wx(0), wy(0), "world / rail_base", dx=-90, dy=18)
    svg.label(wx(sx0), wy(sy0), "shelf", dx=14, dy=-10)

    svg.note(
        f"workcell_pose = [{sx0}, {sy0}, {sz0}, ...] m · robot between tray and shelf rear",
        24,
        580,
    )
    svg.note(
        "+Y toward customer (front retainer) · arm inserts from rear (−Y side of shelf depth)",
        24,
        596,
    )
    write_svg(out / "layout-plan.svg", svg)


def lane_cross_section(geom: dict, catalog: dict, out: Path) -> None:
    shelf = geom["shelf"]
    depth = shelf["depth_m"]
    retainer_d = shelf["front_retainer_depth_m"]
    retainer_h = shelf["front_retainer_height_m"]
    incline_deg = shelf["lane_incline_deg"]
    incline = math.radians(incline_deg)
    support = shelf["support_thickness_m"]
    roller_r = shelf["roller_radius_m"]
    roller_pitch = shelf["roller_pitch_m"]
    divider_h = shelf["divider_height_m"]

    large = next(g for g in catalog["geometries"] if g["geometry_key"] == "bottle.large.standard")
    bottle_r = large["shape"]["radius_m"]
    bottle_h = large["shape"]["height_m"]
    lean = (bottle_h - retainer_h) * math.tan(incline)
    bed_datum = depth - retainer_d
    rear_z = bed_datum * math.tan(incline)

    svg = Svg(900, 420, "Lane cross-section schematic")
    svg.caption("Lane cross-section: incline, retainer, settled product")
    svg.note(
        "Numbers from workcell_geometry.yaml and product_collision_catalog.yaml (large bottle).",
        24,
        42,
    )

    s = 320.0
    ox, oy = 60.0, 360.0

    def sx(y_m: float) -> float:
        return ox + y_m * s

    def sz(z_m: float) -> float:
        return oy - z_m * s

    # Support slab under surface
    # Surface points
    y0, z0 = 0.0, rear_z
    y1, z1 = bed_datum, 0.0
    # Thicken downward along approximate normal
    nx = math.sin(incline)
    nz = math.cos(incline)
    svg.poly(
        [
            (sx(y0), sz(z0)),
            (sx(y1), sz(z1)),
            (sx(y1 + support * nx), sz(z1 - support * nz)),
            (sx(y0 + support * nx), sz(z0 - support * nz)),
        ],
        fill=FILL_SHELF,
        stroke=LINE,
        sw=1.6,
    )
    # Rollers (visual)
    count = int(depth / roller_pitch)
    for i in range(count):
        yd = (i + 0.5) * roller_pitch
        if yd > bed_datum:
            break
        zs = (bed_datum - yd) * math.tan(incline)
        cx = sx(yd - roller_r * math.sin(incline))
        cy = sz(zs - roller_r * math.cos(incline))
        svg.circle(cx, cy, roller_r * s, fill="#9aa6b0", stroke=LINE, sw=1)

    # Front retainer
    svg.rect(
        sx(depth - retainer_d),
        sz(retainer_h),
        retainer_d * s,
        retainer_h * s,
        fill=FILL_RETAINER,
    )
    # Divider ghost height
    svg.line(sx(0), sz(divider_h), sx(depth), sz(divider_h), stroke=MUTED, sw=1, dash="4 3")
    svg.note(f"divider height {divider_h:.2f} m", sx(0.05), sz(divider_h) - 8)

    # Product resting against the retainer, tilted with the bed, drawn as a parallelogram.
    contact_y = bed_datum
    # Bottle center behind the retainer face by the projected radius.
    base_center_y = contact_y - bottle_r * math.cos(incline)
    base_center_z = bottle_r * math.sin(incline)
    hh = bottle_h
    # Bed rises toward -y: the unit along the surface toward the front has dy>0, dz<0.
    along_y = math.cos(incline)
    along_z = -math.sin(incline)
    # Normal up from the bed.
    n_y = math.sin(incline)
    n_z = math.cos(incline)
    # Base corners at ±r along the surface, height along the normal.
    corners = []
    for a in (-bottle_r, bottle_r):
        for b in (0.0, hh):
            yy = base_center_y + a * along_y + b * n_y
            zz = base_center_z + a * along_z + b * n_z
            corners.append((sx(yy), sz(zz)))
    ordered = [corners[0], corners[1], corners[3], corners[2]]
    svg.poly(ordered, fill=FILL_PRODUCT, stroke=TEAL, sw=1.8)

    svg.label(sx(contact_y), sz(0), "retainer face (z=0)", dx=-20, dy=22)
    svg.label(
        sx(base_center_y), sz(base_center_z + hh * n_z * 0.55), "large bottle", dx=-100, dy=-8
    )
    svg.note(
        f"incline {incline_deg:.1f}° · rear rise {rear_z:.3f} m · lean past rail {(lean):.3f} m "
        f"(height {bottle_h:.3f} m − retainer {retainer_h:.2f} m) × tan(incline)",
        24,
        400,
    )
    svg.note(
        f"rollers radius {roller_r:.3f} m pitch {roller_pitch:.2f} m (visual); collision is the "
        "inclined plane",
        24,
        416,
    )
    write_svg(out / "lane-cross-section.svg", svg)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--repo-root",
        type=Path,
        default=None,
        help="Repository root containing ros_ws/ and website/",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory for SVGs (default: website/static/img/schematics)",
    )
    args = parser.parse_args()
    script = Path(__file__).resolve()
    repo = args.repo_root.resolve() if args.repo_root else repo_root_from_script(script)
    desc = repo / "ros_ws/src/restocker_description"
    gazebo = repo / "ros_ws/src/restocker_gazebo"
    out = args.out.resolve() if args.out else repo / "website/static/img/schematics"

    workcell = load_yaml(desc / "config/workcell_geometry.yaml")
    gripper = load_yaml(desc / "config/gripper_geometry.yaml")
    catalog = load_yaml(desc / "config/product_collision_catalog.yaml")
    baseline = load_yaml(gazebo / "config/baseline_products.yaml")
    workcell_pose = [float(v) for v in baseline["workcell_pose"]]
    urdf_dir = desc / "urdf"
    workcell_xacro = (urdf_dir / "workcell.xacro").read_text(encoding="utf-8")
    sensors_xacro = (urdf_dir / "sensors.xacro").read_text(encoding="utf-8")

    out.mkdir(parents=True, exist_ok=True)
    robot_side_and_front(workcell, gripper, urdf_dir, out)
    workcell_top_and_side(workcell, catalog, workcell_xacro, sensors_xacro, workcell_pose, out)
    combined_layout(workcell, workcell_xacro, workcell_pose, urdf_dir, out)
    lane_cross_section(workcell, catalog, out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
