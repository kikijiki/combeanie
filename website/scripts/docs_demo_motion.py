#!/usr/bin/env python3
"""Send a longer rail/arm/gripper motion sequence for docs video recording."""

from __future__ import annotations

import time

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def _point(positions: list[float], seconds: float) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = positions
    point.time_from_start.sec = int(seconds)
    point.time_from_start.nanosec = int((seconds - int(seconds)) * 1e9)
    return point


class DocsMotion(Node):
    def __init__(self) -> None:
        super().__init__("docs_demo_motion")
        self._rail = ActionClient(
            self, FollowJointTrajectory, "/rail_controller/follow_joint_trajectory"
        )
        self._arm = ActionClient(
            self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory"
        )
        self._grip = ActionClient(
            self, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory"
        )

    def _wait(self, client: ActionClient, name: str) -> bool:
        self.get_logger().info(f"waiting for {name}")
        return client.wait_for_server(timeout_sec=60.0)

    def _send(
        self,
        client: ActionClient,
        joint_names: list[str],
        positions: list[float],
        duration: float,
        label: str,
    ) -> None:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points = [_point(positions, duration)]
        self.get_logger().info(label)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"{label}: goal rejected")
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=duration + 15.0)
        if result.result() is None:
            raise RuntimeError(f"{label}: result timeout")

    def run(self) -> None:
        if not (
            self._wait(self._rail, "rail")
            and self._wait(self._arm, "arm")
            and self._wait(self._grip, "gripper")
        ):
            raise RuntimeError("controllers not available")

        # Home-ish open
        self._send(
            self._grip,
            ["left_finger_joint", "right_finger_joint"],
            [0.005, 0.005],
            1.5,
            "gripper open",
        )

        sequence = [
            ("rail", ["rail_joint"], [0.9], 4.0, "rail +X"),
            (
                "arm",
                ARM_JOINTS,
                [0.35, -0.55, 0.85, 0.15, 0.35, -0.2],
                5.0,
                "arm reach A",
            ),
            ("rail", ["rail_joint"], [-0.7], 5.0, "rail -X"),
            (
                "arm",
                ARM_JOINTS,
                [-0.25, -0.35, 0.55, -0.1, 0.25, 0.15],
                5.0,
                "arm reach B",
            ),
            (
                "grip",
                ["left_finger_joint", "right_finger_joint"],
                [0.03, 0.03],
                2.0,
                "gripper close",
            ),
            ("rail", ["rail_joint"], [0.5], 4.5, "rail mid"),
            (
                "arm",
                ARM_JOINTS,
                [0.15, -0.7, 1.0, 0.0, 0.4, 0.0],
                5.5,
                "arm fold toward shelf",
            ),
            (
                "grip",
                ["left_finger_joint", "right_finger_joint"],
                [0.005, 0.005],
                2.0,
                "gripper open",
            ),
            ("rail", ["rail_joint"], [0.0], 4.0, "rail home"),
            (
                "arm",
                ARM_JOINTS,
                [0.0, -0.2, 0.3, 0.0, 0.1, 0.0],
                4.5,
                "arm settle",
            ),
        ]

        for kind, names, pos, dur, label in sequence:
            client = {"rail": self._rail, "arm": self._arm, "grip": self._grip}[kind]
            self._send(client, names, pos, dur, label)
            time.sleep(0.3)

        self.get_logger().info("docs motion sequence finished")


def main() -> int:
    rclpy.init()
    node = DocsMotion()
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001 - surface to shell
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
