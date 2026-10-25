#!/usr/bin/env python3
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
"""Cut one Gazebo recording camera between shots as the campaign's events happen.

showcase_director.py FRAME_DIR MODEL SHOT_LOG

The camera model MODEL saves its frames to FRAME_DIR. This node watches the robot, not a clock:

* the campaign's mode (shelf survey, tray survey, transfer),
* the simulated attachment's phase (a product is gripped or it is not),
* where the gripper is in the world (from tf).

and cuts the camera accordingly. A transfer plays as: the tray close-up while the tool goes in and
takes the product, a mid shot while it is carried, the shelf close-up while it is set down, then
the wide shot again. Everything else, the surveys and the idle time, is the wide shot. Each cut is
appended to SHOT_LOG as one JSON line giving the number of frames saved so far, so showcase_edit.py
can tell which shot every frame belongs to.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import AutonomousRestockCampaignStatus, SimulationAttachmentState
import tf2_ros

_SPEC = importlib.util.spec_from_file_location(
    "showcase_views", Path(__file__).resolve().parent / "showcase_views.py"
)
views = importlib.util.module_from_spec(_SPEC)
sys.modules["showcase_views"] = views
_SPEC.loader.exec_module(views)

# Gripper y (world frame, metres) beyond which the tool is inside the tray's reach, and past which
# it is at the shelf's rear edge. Between them it is travelling.
TRAY_Y_M = -0.38
SHELF_Y_M = 0.25
# A shot stays at least this many frames, so a gripper hovering on a boundary does not flicker.
MIN_SHOT_FRAMES = 18
# The wide shot is held this many frames at the start so the viewer sees the whole scene first.
ESTABLISH_FRAMES = 36


class Director(Node):
    """Chooses the shot from the robot's state and moves the camera model to it."""

    def __init__(self, frame_dir: Path, model: str, log_path: Path) -> None:
        super().__init__("showcase_director")
        self.frame_dir = frame_dir
        self.model = model
        self.log = log_path.open("a", encoding="utf-8", buffering=1)
        self.shot = None
        self.shot_started_frame = 0
        self.mode = AutonomousRestockCampaignStatus.MODE_IDLE
        self.phase = AutonomousRestockCampaignStatus.PHASE_WAITING_FOR_COORDINATOR
        self.transfers = 0
        self.attached = False
        self.gripper_x = 0.0
        self.gripper_y = 0.0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            AutonomousRestockCampaignStatus,
            "/autonomous_restock_campaign/status",
            self.on_campaign,
            latched,
        )
        self.create_subscription(
            SimulationAttachmentState, "/simulation_attachment/state", self.on_attachment, latched
        )
        self.create_timer(0.2, self.tick)

    def on_campaign(self, message: AutonomousRestockCampaignStatus) -> None:
        self.mode = message.mode
        self.phase = message.phase
        self.transfers = int(message.successful_transfers)

    def on_attachment(self, message: SimulationAttachmentState) -> None:
        self.attached = message.phase == SimulationAttachmentState.PHASE_ATTACHED

    def frames(self) -> int:
        return sum(1 for _ in self.frame_dir.glob("*.png"))

    def choose(self) -> str:
        """The shot the robot's state calls for right now."""
        if self.phase in (
            AutonomousRestockCampaignStatus.PHASE_FRONT_FULL,
            AutonomousRestockCampaignStatus.PHASE_COMPLETE,
        ):
            return "wide"
        if self.mode != AutonomousRestockCampaignStatus.MODE_TRANSFER:
            return "wide"
        if self.attached:
            if self.gripper_y > SHELF_Y_M:
                return "place"
            # Still backing out of the tray with the product: the pick shot is not over.
            return "pick" if self.gripper_y < TRAY_Y_M + 0.3 else "carry"
        if self.gripper_y < TRAY_Y_M:
            return "pick"
        return "wide" if self.gripper_y > SHELF_Y_M else "approach"

    def read_gripper(self) -> None:
        try:
            transform = self.tf_buffer.lookup_transform("world", "gripper", rclpy.time.Time())
        except tf2_ros.TransformException:
            return
        self.gripper_x = transform.transform.translation.x
        self.gripper_y = transform.transform.translation.y

    def tick(self) -> None:
        self.read_gripper()
        frame = self.frames()
        wanted = self.choose()
        if self.shot is None:
            self.cut("wide", frame)
            return
        if wanted == self.shot or frame - self.shot_started_frame < MIN_SHOT_FRAMES:
            return
        if self.shot == "wide" and frame < ESTABLISH_FRAMES:
            return
        self.cut(wanted, frame)

    def camera_pose(self, shot: str):
        if shot in ("pick", "place"):
            return views.shot_pose(shot, self.gripper_x)
        if shot == "carry":
            return views.pose("action")
        if shot == "approach":
            return views.pose("action")
        return views.pose("wide")

    def cut(self, shot: str, frame: int) -> None:
        request = views.set_pose_request(self.model, self.camera_pose(shot))
        subprocess.run(
            [
                "gz",
                "service",
                "-s",
                "/world/restocking/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                request,
            ],
            check=False,
            capture_output=True,
        )
        self.shot = shot
        # The pose took effect a few frames after the call, and the frame count was read before it.
        self.shot_started_frame = frame
        record = {
            "frame": frame,
            "shot": shot,
            "transfers": self.transfers,
            "gripper": [round(self.gripper_x, 3), round(self.gripper_y, 3)],
            "wall": round(time.time(), 1),
        }
        self.log.write(json.dumps(record) + "\n")
        self.get_logger().info(f"frame {frame}: {shot}")


def main() -> int:
    frame_dir, model, log_path = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3])
    rclpy.init()
    node = Director(frame_dir, model, log_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
