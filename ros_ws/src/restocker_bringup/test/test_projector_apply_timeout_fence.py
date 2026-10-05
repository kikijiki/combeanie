# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A timed-out remote apply is drained, not locally replaced."""

import json
from pathlib import Path
import sys
import time
import unittest

import launch
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import (
    PlanningSceneLeaseOperationStatus,
    PlanningSceneProjectionStatus,
)
from restocker_interfaces.srv import (
    AcquirePlanningSceneLease,
    GetPlanningSceneProjectionStatus,
    ReleasePlanningSceneLease,
)
from std_srvs.srv import Trigger

WORKCELL_DESCRIPTION = """<?xml version="1.0"?>
<robot name="lease_test_workcell">
  <link name="shelf">
    <collision name="fixture">
      <geometry><box size="0.10 0.10 0.10"/></geometry>
    </collision>
  </link>
</robot>
"""


@pytest.mark.launch_test
def generate_test_description():
    """Start the projector against a backend that delays the first apply."""
    backend = ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name("fake_delayed_scene_backend.py"))],
        output="screen",
    )
    projector = Node(
        package="restocker_task_executor",
        executable="planning_scene_projector_node",
        name="planning_scene_projector_timeout_fence_test",
        output="screen",
        parameters=[
            {
                "planning_frame": "world",
                "workcell_root_frame": "world",
                "workcell_description": WORKCELL_DESCRIPTION,
                "product_catalog_path": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "product_collision_catalog.yaml",
                    ]
                ),
                "workcell_geometry_path": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "workcell_geometry.yaml",
                    ]
                ),
                "reconcile_period_sec": 0.05,
                "service_timeout_sec": 0.25,
            }
        ],
    )
    return launch.LaunchDescription([backend, projector, launch_testing.actions.ReadyToTest()])


class TestProjectorApplyTimeoutFence(unittest.TestCase):
    """Require causal verification after the original apply response."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("projector_apply_timeout_fence_test")
        cls.query_client = cls.node.create_client(Trigger, "/fake_scene_backend/query")
        cls.release_backend_client = cls.node.create_client(
            Trigger, "/fake_scene_backend/release_apply"
        )
        cls.acquire_client = cls.node.create_client(
            AcquirePlanningSceneLease,
            "/planning_scene_projection/acquire_lease",
        )
        cls.release_lease_client = cls.node.create_client(
            ReleasePlanningSceneLease,
            "/planning_scene_projection/release_lease",
        )
        cls.status_client = cls.node.create_client(
            GetPlanningSceneProjectionStatus, "/planning_scene_projection/get_status"
        )
        cls.ignore_status_delivery = False
        cls.latest_status = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            PlanningSceneProjectionStatus,
            "/planning_scene_projection/status",
            cls._on_status,
            status_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, status):
        if not cls.ignore_status_delivery:
            cls.latest_status = status

    def _call(self, client, request, timeout_sec=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call did not complete")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def _wait_until(self, predicate, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = predicate()
            if result is not None and result is not False:
                return result
        self.fail(failure)

    def _pending_backend_state(self):
        response = self._call(self.query_client, Trigger.Request())
        return json.loads(response.message) if response.success else None

    def _acquire(self):
        request = AcquirePlanningSceneLease.Request()
        request.operation_id = "delayed-apply-acquire"
        request.minimum_applied_revision = 10
        return self._call(self.acquire_client, request)

    def _granted_acquisition(self):
        response = self._acquire()
        if response.status.code == PlanningSceneLeaseOperationStatus.GRANTED:
            return response
        return None

    def test_original_apply_response_fences_reconciliation(self):
        initial_counts = self._wait_until(
            self._pending_backend_state,
            timeout_sec=10.0,
            failure="projector never issued the delayed apply",
        )
        self.assertEqual(initial_counts["apply_requests"], 1)
        self.assertEqual(initial_counts["get_scene_requests"], 1)
        in_flight = self._call(
            self.status_client, GetPlanningSceneProjectionStatus.Request()
        ).status
        self.assertNotEqual(in_flight.state, PlanningSceneProjectionStatus.STATE_APPLIED)

        unknown = self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.error_code
                == PlanningSceneProjectionStatus.ERROR_APPLY_OUTCOME_UNKNOWN
                else None
            ),
            timeout_sec=3.0,
            failure="projector did not report the unresolved apply outcome",
        )
        self.assertEqual(unknown.state, PlanningSceneProjectionStatus.STATE_DEGRADED)
        self.assertTrue(unknown.projector_epoch)
        # Intentionally retain the old topic delivery while RPCs observe current serialized state.
        type(self).ignore_status_delivery = True
        retained_topic = self.latest_status

        draining = self._acquire()
        self.assertEqual(draining.status.code, PlanningSceneLeaseOperationStatus.DRAINING)
        self.assertFalse(draining.token)

        time.sleep(0.6)
        fenced_counts = self._pending_backend_state()
        self.assertIsNotNone(fenced_counts)
        self.assertEqual(fenced_counts["apply_requests"], 1)
        self.assertEqual(fenced_counts["get_scene_requests"], 1)

        released = self._call(self.release_backend_client, Trigger.Request())
        self.assertTrue(released.success)

        granted = self._wait_until(
            self._granted_acquisition,
            timeout_sec=5.0,
            failure="lease was not granted after causally ordered verification",
        )
        self.assertTrue(granted.token)
        self.assertGreater(granted.lease.verification_epoch, unknown.verification_epoch)
        applied = self._call(self.status_client, GetPlanningSceneProjectionStatus.Request()).status
        self.assertEqual(applied.state, PlanningSceneProjectionStatus.STATE_TRANSACTION_HELD)
        self.assertIs(self.latest_status, retained_topic)
        # A read neither publishes nor refreshes the held proof's evidence timestamp.
        again = self._call(self.status_client, GetPlanningSceneProjectionStatus.Request()).status
        self.assertEqual(again.header.stamp, applied.header.stamp)
        self.assertEqual(again.verification_epoch, applied.verification_epoch)
        self.assertEqual(applied.projector_epoch, unknown.projector_epoch)
        self.assertGreater(
            applied.scene_content_generation,
            unknown.scene_content_generation,
        )

        release_request = ReleasePlanningSceneLease.Request()
        release_request.operation_id = "delayed-apply-release"
        release_request.token = granted.token
        release_request.required_semantic_revision = 10
        release = self._call(self.release_lease_client, release_request)
        self.assertEqual(
            release.status.code,
            PlanningSceneLeaseOperationStatus.RELEASE_ACCEPTED,
        )
