# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Live startup and authorization checks for the simulator attachment adapter."""

import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from restocker_interfaces.msg import SimulationAttachmentOperationStatus
from restocker_interfaces.srv import (
    GetSimulationAttachmentState,
    SetSimulationAttachment,
)


@pytest.mark.launch_test
def generate_test_description():
    """Start the complete headless baseline with the attachment boundary enabled."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            # Ground-truth path pins (Milestone 10 Stage 7 default switch):
            "object_observation_topic": "/perception/object_observations",
            "lane_observation_topic": "/perception/ground_truth/lane_observations",
            "tray_overview_perception": "false",
            "tray_confirm_perception": "false",
            "gui": "false",
            "rviz": "false",
            "planning_smoke": "false",
            "planning_scene_projection": "true",
            "attachment_adapter": "true",
            "controller_timeout": "30.0",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


class TestAttachmentAdapterRuntime(unittest.TestCase):
    """Require clean startup and fail-closed capability authorization."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("attachment_adapter_runtime_test")
        cls.query_client = cls.node.create_client(
            GetSimulationAttachmentState, "/simulation_attachment/get_state"
        )
        cls.set_client = cls.node.create_client(
            SetSimulationAttachment, "/simulation_attachment/set"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout_sec=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call did not complete")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def test_clean_startup_and_invalid_capability_rejection(self):
        """Expose mutation only after reconciliation and reject secrets before transport."""
        self.assertTrue(self.set_client.wait_for_service(timeout_sec=45.0))
        current = self._call(self.query_client, GetSimulationAttachmentState.Request())
        self.assertEqual(current.status.code, SimulationAttachmentOperationStatus.DETACHED)
        self.assertTrue(current.has_state)
        self.assertTrue(current.simulator_epoch)
        self.assertEqual(current.state.phase, current.state.PHASE_DETACHED)
        self.assertEqual(current.state.motion_gate, current.state.MOTION_GATE_VALID)

        command = SetSimulationAttachment.Request()
        command.command = command.COMMAND_ATTACH
        command.operation_id = "runtime-invalid-capability"
        command.object_id = 7
        command.reservation_token = "invalid-reservation-token"
        command.planning_scene_lease_token = "invalid-lease-token"
        command.expected_grasp_center_to_child.orientation.w = 1.0
        accepted = self._call(self.set_client, command)
        self.assertEqual(accepted.status.code, SimulationAttachmentOperationStatus.PENDING)

        deadline = time.monotonic() + 10.0
        rejected = None
        while time.monotonic() < deadline:
            query = GetSimulationAttachmentState.Request()
            query.operation_id = command.operation_id
            candidate = self._call(self.query_client, query)
            if candidate.status.code != SimulationAttachmentOperationStatus.PENDING:
                rejected = candidate
                break
            time.sleep(0.05)
        self.assertIsNotNone(rejected, "authorization result remained pending")
        self.assertEqual(rejected.status.code, SimulationAttachmentOperationStatus.TOKEN_MISMATCH)

        unchanged = self._call(self.query_client, GetSimulationAttachmentState.Request())
        self.assertEqual(unchanged.status.code, SimulationAttachmentOperationStatus.DETACHED)
        self.assertEqual(unchanged.simulator_epoch, current.simulator_epoch)
        self.assertEqual(unchanged.state.sequence, current.state.sequence)
