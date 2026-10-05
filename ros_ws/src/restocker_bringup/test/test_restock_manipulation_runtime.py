# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Acceptance: every product category is moved from the stock tray into its lane."""

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
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import RestockCoordinatorStatus, ShelfLane
from restocker_interfaces.srv import GetWorldState

# The scenario stocks one product of each category and two lanes accept each category. This is
# one admissible pairing, requested explicitly so the transfer is deterministic and a change in
# lane policy fails the test. Autonomous lane choice is covered by the selection unit tests.
EXPECTED_PAIRS = (
    ("sim:stock_can_01", ShelfLane.PRODUCT_CLASS_CAN, "lane_01"),
    ("sim:stock_small_bottle_01", ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE, "lane_02"),
    ("sim:stock_large_bottle_01", ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE, "lane_03"),
)

# The terminal status assertion catches failed goals; these details also catch the stock-side
# pre-grasp refusal while its evidence is still in action feedback.
FORBIDDEN_ABORT_DETAILS = (
    "pre-grasp motion planning failed",
    "MoveIt reported FAILURE",
    "CONTROL_FAILED",
    "PATH_TOLERANCE_VIOLATED",
    "EXTERNAL_INCONSISTENCY",
)


@pytest.mark.launch_test
def generate_test_description():
    """Start the baseline with the manipulation backends composed."""
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
            "motion_enabled": "true",
            "controller_timeout": "30.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestRestockManipulationRuntime(unittest.TestCase):
    """Require a complete, verified transfer of every stocked product category."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("restock_manipulation_runtime_test")
        cls.latest_status = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            RestockCoordinatorStatus,
            "/restock_action_coordinator/status",
            cls._on_status,
            status_qos,
        )
        cls.action_client = ActionClient(cls.node, RestockProduct, "/restock_product")
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, status):
        cls.latest_status = status

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            result = predicate()
            if result:
                return result
        self.fail(f"timed out waiting for {description}")

    def _snapshot(self):
        self.assertTrue(
            self.world_state.wait_for_service(timeout_sec=30.0),
            "world state never offered its snapshot service",
        )
        future = self.world_state.call_async(GetWorldState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15.0)
        self.assertIsNotNone(future.result(), "world state snapshot request did not complete")
        return future.result().snapshot

    def _admitted_snapshot(self):
        """Return a snapshot only once it carries admitted robot telemetry."""
        snapshot = self._snapshot()
        if snapshot.robot.telemetry_source_id and snapshot.robot.telemetry_revision > 0:
            return snapshot
        return None

    def _restock(self, object_id, lane_id):
        """Drive one explicitly addressed transfer and require it to succeed."""
        goal = RestockProduct.Goal()
        goal.has_object_id = True
        goal.object_id = object_id
        goal.has_lane_id = True
        goal.lane_id = lane_id
        feedback_details = []

        def on_feedback(message):
            detail = message.feedback.detail
            if detail and (not feedback_details or feedback_details[-1] != detail):
                feedback_details.append(detail)

        goal_future = self.action_client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, f"the restock goal for {lane_id} was never answered")
        self.assertTrue(
            goal_handle.accepted, f"the coordinator rejected the restock goal for {lane_id}"
        )

        result_future = goal_handle.get_result_async()
        # A full transfer plans and executes six trajectory segments, two gripper moves and two
        # attachment transactions. The budget stays above the coordinator's total timeout so an
        # overrun is reported by the coordinator with its reason.
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=450.0)
        self.assertIsNotNone(
            result_future.result(), f"the restock goal for {lane_id} produced no result"
        )
        result = result_future.result().result
        abort_evidence = [
            detail
            for detail in feedback_details
            if any(marker in detail for marker in FORBIDDEN_ABORT_DETAILS)
        ]
        self.assertFalse(
            abort_evidence,
            f"restock of object {object_id} into {lane_id} emitted a forbidden manipulation "
            f"abort: {abort_evidence}",
        )
        self.assertNotEqual(
            result.status,
            RestockProduct.Result.STATUS_EXTERNAL_INCONSISTENCY,
            f"restock of object {object_id} into {lane_id} ended EXTERNAL_INCONSISTENCY: "
            f"{result.detail}",
        )
        self.assertEqual(
            result.status,
            RestockProduct.Result.STATUS_SUCCEEDED,
            f"restock of object {object_id} into {lane_id} failed: "
            f"status={result.status} detail={result.detail}",
        )

    def test_every_product_category_is_moved_into_its_lane(self):
        self._spin_until(
            lambda: self.latest_status is not None and self.latest_status.admission_ready,
            120.0,
            "the coordinator to publish readiness",
        )
        # Readiness does not imply the world state has admitted a robot telemetry sample, and
        # selection rejects a snapshot with unset robot telemetry. Wait for that evidence.
        before = self._spin_until(
            self._admitted_snapshot,
            120.0,
            "the world state to admit robot telemetry",
        )
        occupied_before = {lane.id: list(lane.contents) for lane in before.lanes}
        self.assertTrue(
            all(not contents for contents in occupied_before.values()),
            f"the scenario must start with empty lanes, got {occupied_before}",
        )

        # Object IDs are assigned by the world state on admission, so resolve them from the
        # source identity fixed by the scenario file.
        observed = {tracked.source_object_id: tracked for tracked in before.objects}
        lanes_before = {lane.id: lane for lane in before.lanes}
        transfers = []
        for source_object_id, product_class, lane_id in EXPECTED_PAIRS:
            self.assertIn(
                source_object_id, observed, f"the scenario must stock {source_object_id}"
            )
            self.assertIn(lane_id, lanes_before, f"the shelf must offer {lane_id}")
            self.assertEqual(
                lanes_before[lane_id].expected_product_class,
                product_class,
                f"{lane_id} must accept the category this transfer sends it",
            )
            self.assertEqual(
                observed[source_object_id].product_class,
                product_class,
                f"{source_object_id} must be observed as the category it was stocked as",
            )
            transfers.append((source_object_id, observed[source_object_id].id, lane_id))

        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=30.0),
            "the restock action server never became available",
        )

        placed_so_far = {}
        for source_object_id, object_id, lane_id in transfers:
            self._restock(object_id, lane_id)

            # The world state, not the action result, proves the transfer. Read it after every
            # goal so a later transfer cannot mask an earlier one that landed in the wrong lane.
            after = self._snapshot()
            placed_so_far[lane_id] = [object_id]
            placed = {lane.id: list(lane.contents) for lane in after.lanes if lane.contents}
            self.assertEqual(
                placed,
                placed_so_far,
                f"after moving {source_object_id} the occupied lanes should be "
                f"{placed_so_far}, got {placed}",
            )
            self.assertFalse(
                after.robot.has_held_object,
                f"the robot must not still hold {source_object_id}",
            )
            self.assertGreater(after.revision, before.revision)
            before = after
