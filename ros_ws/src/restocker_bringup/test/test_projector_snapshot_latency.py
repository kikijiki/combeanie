# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A snapshot service that is slow but alive still gets served; a dead one still refuses."""

# Latency injection for Card 041's first path. The world-state snapshot service answers every
# request, just later than the projector's nominal `service_timeout_sec` — which is what that
# service looks like when its single-threaded executor is starved by machine load. Two cases,
# both judged on the status topic the authority gate itself reads:
#
# 1. A round trip longer than `service_timeout_sec` still certifies the scene. Re-issuing the
#    request on every deadline makes a service that answers consistently in T > deadline
#    unserviceable for ever (every attempt is cancelled before it can complete), so the projector
#    must hold the request it already has and let it land.
# 2. A service that answers later than the total wait budget never certifies: the projector
#    degrades with ERROR_WORLD_STATE_UNAVAILABLE and stays degraded, the retry rate stays bounded
#    by that budget instead of one request per reconcile cycle, and the moment the service answers
#    in time again the scene is certified. Fail-closed is unchanged; only the livelock is removed.
#
# No simulator: the backend is fake_delayed_scene_backend.py, the same harness
# test_projector_apply_timeout_fence.py uses.

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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import PlanningSceneProjectionStatus
from std_srvs.srv import Trigger

WORKCELL_DESCRIPTION = """<?xml version="1.0"?>
<robot name="snapshot_latency_workcell">
  <link name="shelf">
    <collision name="fixture">
      <geometry><box size="0.10 0.10 0.10"/></geometry>
    </collision>
  </link>
</robot>
"""

# The projector's deadline in this test, and the round trips that sit either side of it.
SERVICE_TIMEOUT_SEC = 0.15
SLOW_ROUND_TRIP_SEC = 0.40
# Longer than the projector's total wait budget below, so no reply can land inside one bounded
# retry; short enough that the backend's sleeping callbacks still finish during teardown.
STALLED_ROUND_TRIP_SEC = 2.0
# Total wait budget the projector is allowed before it gives up on one request and retries.
SERVICE_TOTAL_TIMEOUT_SEC = 1.5


@pytest.mark.launch_test
def generate_test_description():
    """Start the delayable backend and a projector with a short deadline."""
    backend = ExecuteProcess(
        cmd=[
            sys.executable,
            str(Path(__file__).with_name("fake_delayed_scene_backend.py")),
            "--ros-args",
            "-p",
            "hold_first_apply:=false",
        ],
        output="screen",
    )
    projector = Node(
        package="restocker_task_executor",
        executable="planning_scene_projector_node",
        name="planning_scene_projector_snapshot_latency_test",
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
                "service_timeout_sec": SERVICE_TIMEOUT_SEC,
                "service_total_timeout_sec": SERVICE_TOTAL_TIMEOUT_SEC,
            }
        ],
    )
    return launch.LaunchDescription([backend, projector, launch_testing.actions.ReadyToTest()])


class TestProjectorSnapshotLatency(unittest.TestCase):
    """Hold a slow snapshot request; never certify without an answer."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("projector_snapshot_latency_test")
        cls.query_client = cls.node.create_client(Trigger, "/fake_scene_backend/query")
        cls.set_parameters_client = cls.node.create_client(
            SetParameters, "/fake_scene_backend/set_parameters"
        )
        cls.latest_status = None
        status_qos = QoSProfile(
            depth=10,
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
        cls.latest_status = status

    def _call(self, client, request, timeout_sec=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call did not complete")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def _set_snapshot_delay(self, seconds):
        request = SetParameters.Request()
        request.parameters = [
            Parameter(
                name="snapshot_delay_sec",
                type=ParameterType.PARAMETER_DOUBLE,
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE, double_value=float(seconds)
                ),
            )
        ]
        response = self._call(self.set_parameters_client, request)
        self.assertTrue(
            all(result.successful for result in response.results), str(response.results)
        )

    def _backend_counts(self):
        # `success` on this trigger reports whether an apply is pending (the fence test's
        # question); the counts are in the message either way.
        response = self._call(self.query_client, Trigger.Request())
        return json.loads(response.message) if response.message else None

    def _wait_until(self, predicate, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = predicate()
            if result is not None and result is not False:
                return result
        self.fail(failure)

    @staticmethod
    def _stamp_seconds(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _applied_after(self, wall_seconds):
        status = self.latest_status
        if status is None or status.state != PlanningSceneProjectionStatus.STATE_APPLIED:
            return None
        if self._stamp_seconds(status.header.stamp) <= wall_seconds:
            return None
        return status

    def test_a_service_slower_than_the_deadline_still_certifies(self):
        self._set_snapshot_delay(0.0)
        self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                else None
            ),
            timeout_sec=15.0,
            failure="projector never certified against the instantaneous backend",
        )

        # Every reconcile cycle from here on pays a round trip well past the deadline.
        changed_at = time.time()
        self._set_snapshot_delay(SLOW_ROUND_TRIP_SEC)
        applied = self._wait_until(
            lambda: self._applied_after(changed_at),
            timeout_sec=10.0,
            failure=(
                f"projector never certified a round trip of {SLOW_ROUND_TRIP_SEC} s against a "
                f"{SERVICE_TIMEOUT_SEC} s deadline: the snapshot request is abandoned at the "
                "deadline and re-issued, so a service that is consistently slower than the "
                "deadline can never be served"
            ),
        )
        self.assertEqual(
            applied.error_code, PlanningSceneProjectionStatus.ERROR_NONE, applied.detail
        )
        self._set_snapshot_delay(0.0)

    def test_c_the_status_stamp_advances_while_a_request_is_held(self):
        # The authority gate ages the status itself: a projector that held a request but stopped
        # republishing would read as a stale status (a different, equally fatal refusal) instead
        # of settling. Sample the topic through one held window and require the stamps to keep
        # moving, with the hold actually observed in the sample.
        self._set_snapshot_delay(SLOW_ROUND_TRIP_SEC)
        seen = []
        held = 0
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and len(seen) < 6:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            status = self.latest_status
            if status is None:
                continue
            stamp = self._stamp_seconds(status.header.stamp)
            if not seen or stamp != seen[-1]:
                seen.append(stamp)
            if (
                status.state == PlanningSceneProjectionStatus.STATE_DEGRADED
                and status.error_code
                == PlanningSceneProjectionStatus.ERROR_WORLD_STATE_UNAVAILABLE
            ):
                held += 1
        self._set_snapshot_delay(0.0)
        self.assertGreaterEqual(held, 1, "the hold never published a degraded status to sample")
        self.assertGreaterEqual(
            len(set(seen)),
            3,
            "the projector did not republish its status while a request was held, so the "
            "authority gate would age the status instead of reading it as settling",
        )

    def test_b_a_service_slower_than_the_budget_never_certifies(self):
        self._set_snapshot_delay(STALLED_ROUND_TRIP_SEC)
        degraded = self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_DEGRADED
                and self.latest_status.error_code
                == PlanningSceneProjectionStatus.ERROR_WORLD_STATE_UNAVAILABLE
                else None
            ),
            timeout_sec=10.0,
            failure="projector did not degrade while the snapshot service stopped answering",
        )
        self.assertTrue(degraded.projector_epoch)

        counts_before = self._backend_counts()
        self.assertIsNotNone(counts_before)
        observe_until = time.monotonic() + 1.5
        certified_while_silent = None
        while time.monotonic() < observe_until:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            status = self.latest_status
            if status is not None and status.state == PlanningSceneProjectionStatus.STATE_APPLIED:
                certified_while_silent = status
                break
        self.assertIsNone(
            certified_while_silent,
            "the projector certified a scene while the snapshot service was not answering",
        )

        counts_after = self._backend_counts()
        self.assertIsNotNone(counts_after)
        # Bounded retry: the request is held to the total wait budget, not re-issued on every
        # 0.05 s reconcile cycle.
        self.assertLessEqual(
            counts_after["snapshot_requests"] - counts_before["snapshot_requests"],
            3,
            "the projector re-issued the snapshot request once per reconcile cycle while waiting",
        )

        self._set_snapshot_delay(0.0)
        self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                else None
            ),
            timeout_sec=15.0,
            failure="projector did not certify after the snapshot service recovered",
        )
