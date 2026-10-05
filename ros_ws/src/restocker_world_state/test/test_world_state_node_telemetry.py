# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Live contract checks for the authoritative robot-telemetry ingress."""

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
from restocker_interfaces.msg import RobotTelemetry
from restocker_interfaces.srv import GetWorldState, ValidateExecutionWorldAuthority


def generate_test_description():
    world_state = launch_ros.actions.Node(
        package="restocker_world_state",
        executable="world_state_node",
        name="world_state",
        parameters=[{"use_sim_time": False}],
        output="screen",
    )
    return (
        launch.LaunchDescription([world_state, launch_testing.actions.ReadyToTest()]),
        {"world_state": world_state},
    )


class TestWorldStateNodeTelemetry(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("world_state_telemetry_test")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def _snapshot(self):
        client = self.node.create_client(GetWorldState, "/world_state/get_snapshot")
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(GetWorldState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        self.assertIsNotNone(future.result())
        self.node.destroy_client(client)
        return future.result().snapshot

    def _telemetry(self, source_id: str, offset_ns: int = 0) -> RobotTelemetry:
        message = RobotTelemetry()
        now_ns = self.node.get_clock().now().nanoseconds + offset_ns
        message.stamp.sec = now_ns // 1_000_000_000
        message.stamp.nanosec = now_ns % 1_000_000_000
        message.source_id = source_id
        message.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        message.joint_positions = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
        message.joint_velocities = [0.01, -0.02, 0.03, -0.04, 0.05, -0.06]
        message.rail_position = 0.25
        message.rail_velocity = 0.025
        message.gripper_joint_names = ["left_finger_joint", "right_finger_joint"]
        message.gripper_joint_positions = [0.01, 0.02]
        message.gripper_joint_velocities = [0.001, 0.002]
        return message

    def test_accepts_one_writer_and_fails_closed_with_competitor(self) -> None:
        primary = self.node.create_publisher(RobotTelemetry, "/robot_telemetry", 10)
        deadline = time.monotonic() + 10.0
        while primary.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(primary.get_subscription_count(), 1)

        sent = self._telemetry("primary_adapter")
        primary.publish(sent)
        deadline = time.monotonic() + 5.0
        accepted = self._snapshot()
        while accepted.robot.revision == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            accepted = self._snapshot()
        self.assertGreater(accepted.robot.revision, 0)
        self.assertEqual(list(accepted.robot.joint_positions), [0.1, -0.2, 0.3, -0.4, 0.5, -0.6])
        self.assertEqual(
            list(accepted.robot.joint_velocities),
            [0.01, -0.02, 0.03, -0.04, 0.05, -0.06],
        )
        self.assertAlmostEqual(accepted.robot.rail_position, 0.25)
        self.assertAlmostEqual(accepted.robot.rail_velocity, 0.025)
        self.assertEqual(list(accepted.robot.gripper_joint_positions), [0.01, 0.02])
        self.assertEqual(list(accepted.robot.gripper_joint_velocities), [0.001, 0.002])
        self.assertEqual(accepted.robot.telemetry_time, sent.stamp)
        self.assertEqual(accepted.robot.telemetry_source_id, "primary_adapter")
        self.assertGreater(accepted.robot.telemetry_revision, 0)

        parameters = AsyncParameterClient(self.node, "world_state")
        self.assertTrue(parameters.wait_for_services(timeout_sec=5.0))
        changed = parameters.set_parameters([Parameter("robot_telemetry_qos_depth", value=20)])
        rclpy.spin_until_future_complete(self.node, changed, timeout_sec=5.0)
        self.assertTrue(changed.done())
        self.assertFalse(changed.result().results[0].successful)

        competitor = self.node.create_publisher(RobotTelemetry, "/robot_telemetry", 10)
        deadline = time.monotonic() + 5.0
        while competitor.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(competitor.get_subscription_count(), 1)
        competitor.publish(self._telemetry("competing_adapter", 1_000_000))
        time.sleep(0.25)
        rejected = self._snapshot()
        self.assertEqual(rejected.revision, accepted.revision)
        self.assertEqual(rejected.robot.revision, accepted.robot.revision)

        self.node.destroy_publisher(competitor)
        self.node.destroy_publisher(primary)

    def test_runtime_clock_override_switch_is_rejected(self) -> None:
        parameters = AsyncParameterClient(self.node, "world_state")
        self.assertTrue(parameters.wait_for_services(timeout_sec=5.0))
        future = parameters.set_parameters([Parameter("use_sim_time", value=True)])
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        self.assertFalse(future.result().results[0].successful)
        self.assertIn("immutable", future.result().results[0].reason)
        current = parameters.get_parameters(["use_sim_time"])
        rclpy.spin_until_future_complete(self.node, current, timeout_sec=5.0)
        self.assertTrue(current.done())
        self.assertFalse(current.result().values[0].bool_value)

    def test_execution_authority_service_rejects_unknown_capability(self) -> None:
        client = self.node.create_client(
            ValidateExecutionWorldAuthority,
            "/world_state/validate_execution_authority",
        )
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = ValidateExecutionWorldAuthority.Request()
        request.token = "not-an-active-capability"
        request.expected_reservation_id = 1
        request.expected_reservation_revision = 1
        request.expected_object_id = 1
        request.expected_destination_lane_id = "lane_01"
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        response = future.result()
        self.assertIsNotNone(response)
        self.assertEqual(response.status.code, response.status.TOKEN_MISMATCH)
        self.assertNotIn(request.token, response.status.detail)
        self.assertFalse(response.has_proof)
        self.assertFalse(response.has_source_lane)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestWorldStateNodeTelemetryShutdown(unittest.TestCase):
    def test_process_exits_cleanly(self, proc_info, world_state) -> None:
        launch_testing.asserts.assertExitCodes(proc_info, process=world_state)
