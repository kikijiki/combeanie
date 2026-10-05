# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Launch integration tests for robot_state_publisher and the live TF tree."""

from pathlib import Path
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener

JOINT_NAMES = [
    "rail_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
    "left_finger_joint",
    "right_finger_joint",
]


@pytest.mark.launch_test
def generate_test_description():
    """Start only the TF publisher; the test owns deterministic joint states."""
    description_share = Path(get_package_share_directory("restocker_description"))
    xacro_file = description_share / "urdf" / "restocker.urdf.xacro"
    robot_description = subprocess.run(
        ["xacro", str(xacro_file)], check=True, capture_output=True, text=True
    ).stdout
    state_publisher = launch_ros.actions.Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher_test",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    return (
        launch.LaunchDescription([state_publisher, launch_testing.actions.ReadyToTest()]),
        {"state_publisher": state_publisher},
    )


class TestRobotTf(unittest.TestCase):
    """Verify frame reachability and motion semantics through published TF."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("restocker_description_tf_test")
        self.publisher = self.node.create_publisher(JointState, "/joint_states", 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node, spin_thread=True)

    def tearDown(self):
        self.node.destroy_publisher(self.publisher)
        self.listener.unregister()
        self.node.destroy_node()

    def _publish(self, positions: list[float]) -> None:
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = positions
        self.publisher.publish(message)

    def _wait_for_transform(self, source: str, predicate=None):
        deadline = time.monotonic() + 10.0
        transform = None
        while time.monotonic() < deadline:
            self._publish(self.positions)
            time.sleep(0.05)
            try:
                transform = self.buffer.lookup_transform("world", source, Time())
            except Exception:  # noqa: BLE001 -- all TF lookup failures are retryable here
                continue
            if predicate is None or predicate(transform):
                return transform
        self.fail(f"timed out waiting for world -> {source}; last transform: {transform}")

    def test_complete_tree_and_joint_motion(self):
        self.positions = [0.0] * len(JOINT_NAMES)
        for frame in (
            "rail_base",
            "carriage",
            "tool0",
            "grasp_center",
            "left_finger",
            "right_finger",
            "wrist_camera_optical_frame",
        ):
            self._wait_for_transform(frame)

        gripper_from_grasp = self.buffer.lookup_transform("gripper", "grasp_center", Time())
        self.assertAlmostEqual(gripper_from_grasp.transform.translation.x, 0.0, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.translation.y, 0.0, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.translation.z, 0.14, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.rotation.x, 0.0, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.rotation.y, 0.0, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.rotation.z, 0.0, places=9)
        self.assertAlmostEqual(gripper_from_grasp.transform.rotation.w, 1.0, places=9)

        rail_base_before = self._wait_for_transform("rail_base")
        carriage_before = self._wait_for_transform("carriage")
        shoulder_before = self._wait_for_transform("shoulder_link")

        self.positions[0] = 0.4
        carriage_after_rail = self._wait_for_transform(
            "carriage", lambda transform: transform.transform.translation.x > 0.39
        )
        rail_base_after = self._wait_for_transform("rail_base")
        self.assertAlmostEqual(
            carriage_after_rail.transform.translation.x - carriage_before.transform.translation.x,
            0.4,
            places=6,
        )
        self.assertAlmostEqual(
            carriage_after_rail.transform.translation.y,
            carriage_before.transform.translation.y,
            places=6,
        )
        self.assertAlmostEqual(
            carriage_after_rail.transform.translation.z,
            carriage_before.transform.translation.z,
            places=6,
        )
        self.assertEqual(rail_base_before.transform, rail_base_after.transform)

        self.positions[1] = 0.5
        shoulder_after = self._wait_for_transform(
            "shoulder_link",
            lambda transform: abs(transform.transform.rotation.z) > 0.2,
        )
        carriage_after_arm = self._wait_for_transform("carriage")
        before_q = shoulder_before.transform.rotation
        after_q = shoulder_after.transform.rotation
        quaternion_dot = abs(
            before_q.x * after_q.x
            + before_q.y * after_q.y
            + before_q.z * after_q.z
            + before_q.w * after_q.w
        )
        self.assertLess(quaternion_dot, 0.99)
        self.assertEqual(carriage_after_rail.transform, carriage_after_arm.transform)


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):
    """Retain launch-process failures as test failures."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
