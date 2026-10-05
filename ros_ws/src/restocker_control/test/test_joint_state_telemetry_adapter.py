# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Installed-node contract test for controller telemetry normalization."""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import (  # noqa: I101 - ament and Ruff disagree on mixed-case ordering
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from restocker_interfaces.msg import RobotTelemetry
from sensor_msgs.msg import JointState

INPUT_TOPIC = "/test/control/joint_states"
OUTPUT_TOPIC = "/test/control/robot_telemetry"
SOURCE_ID = "test/control_adapter"


def generate_test_description():
    adapter = launch_ros.actions.Node(
        package="restocker_control",
        executable="joint_state_telemetry_adapter",
        name="test_joint_state_telemetry_adapter",
        parameters=[
            {
                "input_topic": INPUT_TOPIC,
                "output_topic": OUTPUT_TOPIC,
                "source_id": SOURCE_ID,
                "output_qos_depth": 4,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([adapter, launch_testing.actions.ReadyToTest()]),
        {"adapter": adapter},
    )


class TestJointStateTelemetryAdapter(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("joint_state_telemetry_adapter_test")
        self.received: list[RobotTelemetry] = []
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.node.create_subscription(
            RobotTelemetry, OUTPUT_TOPIC, self.received.append, output_qos
        )
        self.publisher = self.node.create_publisher(
            JointState, INPUT_TOPIC, qos_profile_sensor_data
        )

    def tearDown(self) -> None:
        self.node.destroy_publisher(self.publisher)
        self.node.destroy_subscription(self.subscription)
        self.node.destroy_node()

    def _wait_for_graph(self, timeout_sec: float = 10.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if (
                self.publisher.get_subscription_count() == 1
                and self.subscription.get_publisher_count() == 1
            ):
                return
        self.fail("adapter endpoints did not become discoverable")

    def _spin_for(self, duration_sec: float) -> None:
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _wait_for_messages(self, count: int, timeout_sec: float = 5.0) -> None:
        deadline = time.monotonic() + timeout_sec
        while len(self.received) < count and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(len(self.received), count)

    @staticmethod
    def _complete_sample(sec: int, nanosec: int) -> JointState:
        sample = JointState()
        sample.header.stamp.sec = sec
        sample.header.stamp.nanosec = nanosec
        sample.name = [
            "wrist_1_joint",
            "left_finger_joint",
            "rail_joint",
            "shoulder_pan_joint",
            "wrist_3_joint",
            "shoulder_lift_joint",
            "wrist_2_joint",
            "elbow_joint",
            "right_finger_joint",
        ]
        sample.position = [4.0, 0.01, 0.75, 1.0, 6.0, 2.0, 5.0, 3.0, 0.02]
        sample.velocity = [0.4, 0.001, 0.075, 0.1, 0.6, 0.2, 0.5, 0.3, 0.002]
        return sample

    def test_projection_timestamp_fence_qos_and_immutable_parameters(self) -> None:
        self._wait_for_graph()
        endpoints = self.node.get_publishers_info_by_topic(OUTPUT_TOPIC)
        self.assertEqual(len(endpoints), 1)
        self.assertEqual(endpoints[0].qos_profile.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(endpoints[0].qos_profile.durability, DurabilityPolicy.VOLATILE)

        incomplete = self._complete_sample(100, 1)
        incomplete.name.remove("wrist_2_joint")
        incomplete.position.pop(6)
        incomplete.velocity.pop(6)
        self.publisher.publish(incomplete)
        self._spin_for(0.3)
        self.assertEqual(self.received, [])

        first = self._complete_sample(100, 2)
        self.publisher.publish(first)
        self._wait_for_messages(1)
        projected = self.received[0]
        self.assertEqual(projected.stamp, first.header.stamp)
        self.assertEqual(projected.source_id, SOURCE_ID)
        self.assertEqual(
            list(projected.joint_names),
            [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint",
            ],
        )
        self.assertEqual(list(projected.joint_positions), [1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
        self.assertEqual(list(projected.joint_velocities), [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        self.assertAlmostEqual(projected.rail_position, 0.75)
        self.assertAlmostEqual(projected.rail_velocity, 0.075)
        self.assertEqual(
            list(projected.gripper_joint_names),
            ["left_finger_joint", "right_finger_joint"],
        )
        self.assertEqual(list(projected.gripper_joint_positions), [0.01, 0.02])
        self.assertEqual(list(projected.gripper_joint_velocities), [0.001, 0.002])

        self.publisher.publish(self._complete_sample(100, 2))
        self.publisher.publish(self._complete_sample(99, 999_999_999))
        self._spin_for(0.3)
        self.assertEqual(len(self.received), 1)

        invalid_newer = self._complete_sample(100, 4)
        invalid_newer.name.remove("elbow_joint")
        invalid_newer.position.pop()
        invalid_newer.velocity.pop()
        self.publisher.publish(invalid_newer)
        self._spin_for(0.3)
        self.assertEqual(len(self.received), 1)

        newer = self._complete_sample(100, 3)
        newer.position[3] = -0.5
        self.publisher.publish(newer)
        self._wait_for_messages(2)
        self.assertAlmostEqual(self.received[1].joint_positions[0], -0.5)

        parameters = AsyncParameterClient(self.node, "test_joint_state_telemetry_adapter")
        self.assertTrue(parameters.wait_for_services(timeout_sec=5.0))
        change = parameters.set_parameters([Parameter("source_id", value="changed/source")])
        rclpy.spin_until_future_complete(self.node, change, timeout_sec=5.0)
        self.assertTrue(change.done())
        self.assertFalse(change.result().results[0].successful)


@launch_testing.post_shutdown_test()
class TestJointStateTelemetryAdapterShutdown(unittest.TestCase):
    def test_process_exits_cleanly(self, proc_info, adapter) -> None:
        launch_testing.asserts.assertExitCodes(proc_info, process=adapter)
