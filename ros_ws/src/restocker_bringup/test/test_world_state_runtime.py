# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Gazebo ground-truth to persistent world-state integration test."""

import math
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
from restocker_interfaces.msg import (
    ObjectObservation,
    RobotExecutionState,
    ShelfLane,
    WorldStateOperationStatus,
)
from restocker_interfaces.srv import (
    CheckpointTaskState,
    CommitReservedAttachment,
    CommitReservedDetachment,
    GetWorldState,
    ReleaseTaskReservation,
    ReserveTask,
    ValidateTaskReservation,
)

EXPECTED_PRODUCTS = {
    "sim:stock_can_01": ObjectObservation.PRODUCT_CLASS_CAN,
    "sim:stock_small_bottle_01": ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE,
    "sim:stock_large_bottle_01": ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE,
}
# Two lanes per product class, interleaved so a class's alternatives sit 1.2 m apart and
# selection has a cost difference to weigh.
EXPECTED_LANES = {
    "lane_01": (ShelfLane.PRODUCT_CLASS_CAN, "SIM-CAN-STD"),
    "lane_02": (ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE, "SIM-BOTTLE-SMALL"),
    "lane_03": (ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE, "SIM-BOTTLE-LARGE"),
    "lane_04": (ShelfLane.PRODUCT_CLASS_CAN, "SIM-CAN-STD"),
    "lane_05": (ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE, "SIM-BOTTLE-SMALL"),
    "lane_06": (ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE, "SIM-BOTTLE-LARGE"),
}


@pytest.mark.launch_test
def generate_test_description():
    """Start the public simulation composition including its world-state node."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "controller_timeout": "30.0",
            "spawn_scenario": "true",
            "ground_truth": "true",
        }.items(),
    )
    return launch.LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestWorldStateRuntime(unittest.TestCase):
    """Verify semantic identities survive the live Gazebo-to-ROS boundary."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("world_state_runtime_test")
        cls.client = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.reserve_client = cls.node.create_client(ReserveTask, "/world_state/reserve_task")
        cls.validate_client = cls.node.create_client(
            ValidateTaskReservation, "/world_state/validate_reservation"
        )
        cls.checkpoint_client = cls.node.create_client(
            CheckpointTaskState, "/world_state/checkpoint_task"
        )
        cls.attachment_client = cls.node.create_client(
            CommitReservedAttachment, "/world_state/commit_attachment"
        )
        cls.detachment_client = cls.node.create_client(
            CommitReservedDetachment, "/world_state/commit_detachment"
        )
        cls.release_client = cls.node.create_client(
            ReleaseTaskReservation, "/world_state/release_reservation"
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _request_nonempty_snapshot(self, timeout_sec=45.0):
        deadline = time.monotonic() + timeout_sec
        self.assertTrue(self.client.wait_for_service(timeout_sec=timeout_sec))
        while time.monotonic() < deadline:
            request = GetWorldState.Request()
            request.include_removed = False
            request.include_events = False
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.done() and future.result() is not None:
                snapshot = future.result().snapshot
                lanes_are_fresh = len(snapshot.lanes) == len(EXPECTED_LANES) and all(
                    lane.last_verified.sec > 0 or lane.last_verified.nanosec > 0
                    for lane in snapshot.lanes
                )
                telemetry_is_live = (
                    snapshot.robot.telemetry_revision > 0
                    and (
                        snapshot.robot.telemetry_time.sec > 0
                        or snapshot.robot.telemetry_time.nanosec > 0
                    )
                    and len(snapshot.robot.joint_positions) == 6
                    and all(math.isfinite(value) for value in snapshot.robot.joint_positions)
                    and len(snapshot.robot.joint_velocities) == 6
                    and all(math.isfinite(value) for value in snapshot.robot.joint_velocities)
                    and math.isfinite(snapshot.robot.rail_position)
                    and math.isfinite(snapshot.robot.rail_velocity)
                )
                if (
                    len(snapshot.objects) == len(EXPECTED_PRODUCTS)
                    and lanes_are_fresh
                    and telemetry_is_live
                ):
                    return snapshot
            time.sleep(0.1)
        self.fail("world state did not expose the complete configured scenario")

    def _call(self, client, request, timeout_sec=5.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done())
        self.assertIsNotNone(future.result())
        return future.result()

    def test_products_retain_stable_semantic_identity(self):
        first = self._request_nonempty_snapshot()
        time.sleep(0.2)
        second = self._request_nonempty_snapshot(timeout_sec=10.0)

        self.assertEqual(first.header.frame_id, "world")
        self.assertEqual(second.header.frame_id, "world")
        self.assertGreater(second.revision, first.revision)

        first_ids = {item.source_object_id: item.id for item in first.objects}
        second_ids = {item.source_object_id: item.id for item in second.objects}
        self.assertEqual(first_ids, second_ids)
        self.assertEqual(set(first_ids), set(EXPECTED_PRODUCTS))
        self.assertEqual(len(set(first_ids.values())), len(EXPECTED_PRODUCTS))

        for item in second.objects:
            self.assertEqual(item.product_class, EXPECTED_PRODUCTS[item.source_object_id])
            self.assertEqual(item.tracking_state, item.TRACKING_TRACKED)
            self.assertEqual(item.grasp_state, item.GRASP_FREE)
            self.assertTrue(item.has_sku)
            quaternion = item.pose.pose.orientation
            norm = math.sqrt(quaternion.x**2 + quaternion.y**2 + quaternion.z**2 + quaternion.w**2)
            self.assertAlmostEqual(norm, 1.0, places=6)

        first_lanes = {lane.id: lane for lane in first.lanes}
        second_lanes = {lane.id: lane for lane in second.lanes}
        self.assertEqual(set(first_lanes), set(EXPECTED_LANES))
        self.assertEqual(set(second_lanes), set(EXPECTED_LANES))
        for lane_id, lane in second_lanes.items():
            expected_class, expected_sku = EXPECTED_LANES[lane_id]
            self.assertEqual(lane.expected_product_class, expected_class)
            self.assertTrue(lane.has_expected_sku)
            self.assertEqual(lane.expected_sku, expected_sku)
            self.assertAlmostEqual(lane.depth_m, 0.85)
            self.assertAlmostEqual(lane.available_depth_m, 0.85)
            self.assertFalse(lane.obstructed)
            self.assertEqual(lane.observed_source_object_ids, [])
            self.assertEqual(len(lane.contents), 0)
            self.assertGreater(lane.revision, first_lanes[lane_id].revision)

    def test_controller_telemetry_has_one_producer_and_advances_authority(self):
        first = self._request_nonempty_snapshot()
        self.assertEqual(self.node.count_publishers("/robot_telemetry"), 1)
        deadline = time.monotonic() + 5.0
        second = first
        while second.robot.telemetry_revision <= first.robot.telemetry_revision:
            self.assertLess(time.monotonic(), deadline, "controller telemetry stopped advancing")
            rclpy.spin_once(self.node, timeout_sec=0.05)
            second = self._request_nonempty_snapshot(timeout_sec=2.0)
        self.assertGreater(second.robot.telemetry_revision, first.robot.telemetry_revision)
        self.assertEqual(len(second.robot.joint_positions), 6)
        self.assertTrue(all(math.isfinite(value) for value in second.robot.joint_positions))
        self.assertEqual(len(second.robot.joint_velocities), 6)
        self.assertTrue(all(math.isfinite(value) for value in second.robot.joint_velocities))
        self.assertTrue(math.isfinite(second.robot.rail_position))
        self.assertTrue(math.isfinite(second.robot.rail_velocity))

    def test_reservation_services_enforce_live_transaction_contract(self):
        deadline = time.monotonic() + 15.0
        reserved = None
        while time.monotonic() < deadline and reserved is None:
            snapshot = self._request_nonempty_snapshot(timeout_sec=5.0)
            target = next(
                item for item in snapshot.objects if item.source_object_id == "sim:stock_can_01"
            )
            destination = next(item for item in snapshot.lanes if item.id == "lane_01")
            request = ReserveTask.Request()
            request.request_id = "runtime-reservation-001"
            request.selected_snapshot_revision = snapshot.revision
            request.object_id = target.id
            request.object_revision = target.revision
            request.has_source_lane = False
            request.destination_lane_id = destination.id
            request.destination_lane_revision = destination.revision
            response = self._call(self.reserve_client, request)
            if response.status.code == WorldStateOperationStatus.OK:
                reserved = response
            else:
                self.assertEqual(response.status.code, WorldStateOperationStatus.REVISION_CONFLICT)
        self.assertIsNotNone(reserved, "fresh reservation request never won its revision race")
        self.assertEqual(len(reserved.token), 32)
        self.assertEqual(reserved.reservation.request_id, "runtime-reservation-001")

        validation = ValidateTaskReservation.Request()
        validation.token = reserved.token
        validated = self._call(self.validate_client, validation)
        self.assertEqual(validated.status.code, WorldStateOperationStatus.OK)
        self.assertTrue(validated.has_reservation)

        checkpoint = CheckpointTaskState.Request()
        checkpoint.token = reserved.token
        checkpoint.operation_id = "runtime-checkpoint-001"
        checkpoint.expected_reservation_id = reserved.reservation.reservation_id
        checkpoint.expected_reservation_stage = reserved.reservation.stage
        checkpoint.expected_reservation_revision = reserved.reservation.revision
        checkpoint.task_phase = RobotExecutionState.TASK_EXECUTING
        checkpoint.fault_state = RobotExecutionState.FAULT_NONE
        checkpointed = self._call(self.checkpoint_client, checkpoint)
        self.assertEqual(checkpointed.status.code, WorldStateOperationStatus.OK)
        replayed = self._call(self.checkpoint_client, checkpoint)
        self.assertEqual(replayed.status.code, WorldStateOperationStatus.OK)
        self.assertEqual(replayed.world_revision, checkpointed.world_revision)

        bad_attachment = CommitReservedAttachment.Request()
        bad_attachment.token = "wrong-token"
        bad_attachment.operation_id = "runtime-bad-attachment"
        rejected_attachment = self._call(self.attachment_client, bad_attachment)
        self.assertEqual(rejected_attachment.status.code, WorldStateOperationStatus.TOKEN_MISMATCH)

        bad_detachment = CommitReservedDetachment.Request()
        bad_detachment.token = reserved.token
        bad_detachment.operation_id = "runtime-bad-detachment"
        bad_detachment.disposition = 200
        rejected_detachment = self._call(self.detachment_client, bad_detachment)
        self.assertEqual(
            rejected_detachment.status.code, WorldStateOperationStatus.INVALID_ARGUMENT
        )

        release = ReleaseTaskReservation.Request()
        release.token = reserved.token
        release.operation_id = "runtime-release-001"
        release.expected_reservation_id = checkpointed.reservation.reservation_id
        release.expected_reservation_stage = checkpointed.reservation.stage
        release.expected_reservation_revision = checkpointed.reservation.revision
        release.outcome = ReleaseTaskReservation.Request.OUTCOME_CANCELED
        release.terminal_task_phase = RobotExecutionState.TASK_IDLE
        release.terminal_fault_state = RobotExecutionState.FAULT_NONE
        released = self._call(self.release_client, release)
        self.assertEqual(released.status.code, WorldStateOperationStatus.OK)

        invalidated = self._call(self.validate_client, validation)
        self.assertEqual(invalidated.status.code, WorldStateOperationStatus.TOKEN_MISMATCH)
