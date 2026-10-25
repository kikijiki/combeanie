#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Named camera views of the showcase scene, as Gazebo camera models and GUI poses.

The same table frames the Gazebo GUI that `just demo` opens (restocking_showcase.sdf carries the
`default` view as its GUI camera pose), the recorded video, the poster and the documentation
screenshots, so every picture of the scene is taken from a view that is written down here.

    showcase_views.py list
    showcase_views.py pose wide              # "x y z roll pitch yaw" in the world frame
    showcase_views.py sdf wide --width 1920 --height 1080 --rate 8
"""

from __future__ import annotations

import argparse
import math
import sys

# name -> (camera position, point looked at, horizontal field of view in radians, description).
# World frame: the shelf front is +Y, the stock tray -Y, the arm's rail runs along X at Y = 0.
VIEWS: dict[str, tuple[tuple[float, float, float], tuple[float, float, float], float, str]] = {
    "wide": ((3.2, 2.5, 2.3), (0.0, 0.0, 0.7), 0.95, "whole cell from the shelf-front corner"),
    "shelf": ((-2.4, 3.0, 1.7), (0.3, 1.0, 0.85), 0.95, "shelf front, six lanes, from the left"),
    "shelf_closeup": ((-0.55, 2.0, 1.15), (-0.55, 1.0, 0.85), 0.95, "two lanes, labels readable"),
    "tray": ((1.9, 0.6, 1.8), (-0.05, -0.85, 0.65), 0.95, "back room from the robot side"),
    "tray_closeup": (
        (0.7, -0.05, 1.1),
        (-0.1, -0.85, 0.7),
        0.95,
        "packed blocks, labels readable",
    ),
    "action": ((2.2, -1.9, 1.7), (0.0, -0.2, 0.9), 0.95, "arm between tray and shelf"),
    "side": ((3.4, 0.15, 1.5), (0.0, 0.15, 0.85), 0.95, "side-on, tray left, shelf right"),
}
DEFAULT_VIEW = "wide"


def look_at(
    position: tuple[float, float, float], target: tuple[float, float, float]
) -> tuple[float, float, float, float, float, float]:
    """Return x, y, z, roll, pitch, yaw of a camera at ``position`` looking at ``target``."""
    px, py, pz = position
    tx, ty, tz = target
    yaw = math.atan2(ty - py, tx - px)
    pitch = math.atan2(pz - tz, math.hypot(tx - px, ty - py))
    return (px, py, pz, 0.0, pitch, yaw)


def pose(name: str) -> tuple[float, float, float, float, float, float]:
    """Return x, y, z, roll, pitch, yaw of a camera looking from a view position at its target."""
    position, target, _, _ = VIEWS[name]
    return look_at(position, target)


def set_pose_request(model: str, camera_pose: tuple[float, ...]) -> str:
    """The gz.msgs.Pose text that /world/<world>/set_pose takes to move a camera model."""
    x, y, z, roll, pitch, yaw = camera_pose
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return (
        f'name: "{model}" position {{x: {x:.4f} y: {y:.4f} z: {z:.4f}}} '
        f"orientation {{x: {qx:.6f} y: {qy:.6f} z: {qz:.6f} w: {qw:.6f}}}"
    )


# Shots a director frames from where the gripper is: name -> function of the gripper's world x
# giving (camera position, point looked at). The rail slides the arm along x, so a shot follows
# the gripper's x and keeps the same relation to the shelf (+Y) and the tray (-Y).
#   pick   from the +X side of the tray at table height: the tool sweeps in along -Y, the arm's
#          column stands at y = 0 behind it, so nothing stands between this camera and the pads.
#   place  from in front of the shelf, above the customer's eye line, so the labels face the lens
#          and the arm, coming from behind the shelf, is behind the product it sets down.
SHOTS = {
    "pick": lambda gx: ((gx + 1.5, -0.62, 1.1), (gx, -0.62, 0.68)),
    "place": lambda gx: ((gx - 1.4, 0.15, 1.6), (gx, 0.65, 0.85)),
}


def shot_pose(name: str, gripper_x: float) -> tuple[float, float, float, float, float, float]:
    """The camera pose of a gripper-relative shot."""
    position, target = SHOTS[name](gripper_x)
    return look_at(position, target)


def camera_sdf(
    name: str,
    width: int,
    height: int,
    rate: float,
    fov: float | None = None,
    save_dir: str | None = None,
) -> str:
    """A static camera model named showcase_cam_<name>, publishing on the topic of that name.

    With ``save_dir`` the sensor also writes every frame there as a PNG, which keeps full-size
    frames off the ROS transport (a 6 MB message every 170 ms is dropped under load).
    """
    x, y, z, roll, pitch, yaw = pose(name)
    fov = VIEWS[name][2] if fov is None else fov
    save = f'\n          <save enabled="true"><path>{save_dir}</path></save>' if save_dir else ""
    return f"""<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="showcase_cam_{name}">
    <static>true</static>
    <pose>{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}</pose>
    <link name="link">
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <update_rate>{rate}</update_rate>
        <topic>showcase_cam_{name}</topic>
        <camera>
          <horizontal_fov>{fov}</horizontal_fov>
          <image><width>{width}</width><height>{height}</height></image>
          <clip><near>0.1</near><far>14</far></clip>
          <anti_aliasing>4</anti_aliasing>{save}
        </camera>
      </sensor>
    </link>
  </model>
</sdf>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list")
    quaternion_parser = sub.add_parser("set-pose-request")
    quaternion_parser.add_argument("view")
    quaternion_parser.add_argument("model")
    pose_parser = sub.add_parser("pose")
    pose_parser.add_argument("view")
    sdf_parser = sub.add_parser("sdf")
    sdf_parser.add_argument("view")
    sdf_parser.add_argument("--width", type=int, default=1920)
    sdf_parser.add_argument("--height", type=int, default=1080)
    sdf_parser.add_argument("--rate", type=float, default=8.0)
    sdf_parser.add_argument("--save-dir", default=None, help="write every frame here as a PNG")
    sdf_parser.add_argument(
        "--fov", type=float, default=None, help="override the view's field of view"
    )
    arguments = parser.parse_args()
    if arguments.command == "list":
        for name, (_, _, _, description) in VIEWS.items():
            print(f"{name:14s} {description}")
        return 0
    if arguments.view not in VIEWS:
        print(f"unknown view {arguments.view}; try: list", file=sys.stderr)
        return 2
    if arguments.command == "set-pose-request":
        print(set_pose_request(arguments.model, pose(arguments.view)))
        return 0
    if arguments.command == "pose":
        print(" ".join(f"{value:.4f}" for value in pose(arguments.view)))
    else:
        sys.stdout.write(
            camera_sdf(
                arguments.view,
                arguments.width,
                arguments.height,
                arguments.rate,
                arguments.fov,
                arguments.save_dir,
            )
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
