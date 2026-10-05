# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT
# SPDX-License-Identifier: MIT

"""Managed /clock admission and replay through the real node, without a simulator."""

from pathlib import Path
import tempfile
import time
import unittest

from builtin_interfaces.msg import Time
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from restocker_interfaces.msg import (
    LaneObservation,
    ObjectObservation,
    RobotTelemetry,
    TaskReservation,
)
from restocker_interfaces.msg import (
    WorldStateOperationStatus as Status,
)
from restocker_interfaces.srv import (
    CommitReservedAttachment,
    GetWorldState,
    ReserveTask,
    ValidateExecutionWorldAuthority,
)
from rosgraph_msgs.msg import Clock


def generate_test_description():
    fixture = tempfile.TemporaryDirectory(prefix="world-clock-contract-")
    folder = Path(fixture.name)
    semantics = folder / "lanes.yaml"
    semantics.write_text(
        "schema_version: 1\nlanes:\n  lane:\n"
        "    expected_product_class: can\n    target_count: 1\n",
        encoding="utf-8",
    )
    geometry = folder / "geometry.yaml"
    geometry.write_text(
        "schema_version: 1\nshelf:\n  lane_incline_deg: 0.0\nlanes:\n  lane:\n"
        "    usable_depth_m: 0.85\n    insert_entry_clearance_m: 0.045\n",
        encoding="utf-8",
    )
    catalog = folder / "catalog.yaml"
    catalog.write_text(
        "schema_version: 1\ngeometries:\n  - product_class: can\n"
        "    shape:\n      type: cylinder\n      radius_m: 0.033\n",
        encoding="utf-8",
    )
    world_state = launch_ros.actions.Node(
        package="restocker_world_state",
        executable="world_state_node",
        name="world_state",
        parameters=[
            {
                "use_sim_time": True,
                "lane_semantics_config": str(semantics),
                "workcell_geometry": str(geometry),
                "product_catalog": str(catalog),
                "operation_journal_capacity": 4,
            }
        ],
        output="screen",
    )
    return (
        launch.LaunchDescription([world_state, launch_testing.actions.ReadyToTest()]),
        {"world_state": world_state, "fixture": fixture},
    )


def stamp(ns):
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


class TestWorldStateManagedClock(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("world_clock_contract_test")
        self.snapshot_client = self.node.create_client(GetWorldState, "/world_state/get_snapshot")

    def tearDown(self):
        self.node.destroy_node()

    def call(self, client, request):
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done())
        self.assertIsNotNone(future.result())
        return future.result()

    def snapshot(self):
        return self.call(self.snapshot_client, GetWorldState.Request()).snapshot

    def wait_snapshot(self, predicate):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            value = self.snapshot()
            if predicate(value):
                return value
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.fail("world-state snapshot did not reach the expected condition")

    def publisher(self, message_type, topic):
        publisher = self.node.create_publisher(message_type, topic, 10)
        deadline = time.monotonic() + 5.0
        while publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.01)
        self.assertEqual(publisher.get_subscription_count(), 1)
        return publisher

    def set_clock(self, publisher, ns):
        publisher.publish(Clock(clock=stamp(ns)))
        self.wait_snapshot(lambda value: value.header.stamp == stamp(ns))

    def test_paused_readiness_maturation_and_historical_reset(self):
        clock = self.publisher(Clock, "/clock")
        objects = self.publisher(ObjectObservation, "/perception/object_observations")
        lanes = self.publisher(LaneObservation, "/perception/lane_observations")
        robots = self.publisher(RobotTelemetry, "/robot_telemetry")
        parameters = AsyncParameterClient(self.node, "world_state")
        self.assertTrue(parameters.wait_for_services(timeout_sec=5.0))
        switch = parameters.set_parameters([Parameter("use_sim_time", value=False)])
        rclpy.spin_until_future_complete(self.node, switch, timeout_sec=5.0)
        self.assertTrue(switch.done())
        self.assertFalse(switch.result().results[0].successful)
        self.assertIn("immutable", switch.result().results[0].reason)
        self.set_clock(clock, 9_400_000_000)
        lane = LaneObservation()
        lane.header.stamp = stamp(9_400_000_000)
        lane.header.frame_id = "lane"
        lane.lane_id = "lane"
        lane.available_depth_m = 0.85
        lane.confidence = 1.0
        lane.backend_name = "clock-test"
        lane.backend_version = "1"
        lanes.publish(lane)
        self.wait_snapshot(lambda value: value.lanes[0].last_verified == lane.header.stamp)
        self.set_clock(clock, 9_500_000_000)
        product = ObjectObservation()
        product.header.frame_id = "world"
        product.header.stamp = stamp(9_500_000_000)
        product.source_object_id = "clock-test-product"
        product.product_class = ObjectObservation.PRODUCT_CLASS_CAN
        product.orientation = ObjectObservation.ORIENTATION_UPRIGHT
        product.pose.pose.orientation.w = 1.0
        for index in range(6):
            product.pose.covariance[index * 6 + index] = 1.0e-8
        product.confidence = 1.0
        product.backend_name = "clock-test"
        product.backend_version = "1"
        objects.publish(product)
        self.wait_snapshot(lambda value: len(value.objects) == 1)
        self.set_clock(clock, 9_550_000_000)
        robot = RobotTelemetry()
        robot.stamp = stamp(9_550_000_000)
        robot.source_id = "clock-test"
        robot.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        robot.joint_positions = [0.0] * 6
        robot.joint_velocities = [0.0] * 6
        robot.gripper_joint_names = ["left_finger_joint", "right_finger_joint"]
        robot.gripper_joint_positions = [0.0, 0.0]
        robot.gripper_joint_velocities = [0.0, 0.0]
        robots.publish(robot)
        self.wait_snapshot(lambda value: value.robot.telemetry_time == robot.stamp)
        self.set_clock(clock, 9_600_000_000)
        selected = self.snapshot()
        reserve = ReserveTask.Request()
        reserve.request_id = "clock-test-reserve"
        reserve.selected_snapshot_revision = selected.revision
        reserve.object_id = selected.objects[0].id
        reserve.object_revision = selected.objects[0].revision
        reserve.destination_lane_id = "lane"
        reserve.destination_lane_revision = selected.lanes[0].revision
        reserved = self.call(
            self.node.create_client(ReserveTask, "/world_state/reserve_task"), reserve
        )
        self.assertEqual(reserved.status.code, Status.OK, reserved.status.detail)
        self.set_clock(clock, 9_650_000_000)
        product.header.stamp = stamp(9_700_000_000)
        objects.publish(product)
        before = self.wait_snapshot(
            lambda value: value.objects[0].observation_time == product.header.stamp
        )
        attach = CommitReservedAttachment.Request()
        attach.token = reserved.token
        attach.operation_id = "clock-test-attach"
        attach.grasp_center_from_held_object.orientation.w = 1.0
        client = self.node.create_client(
            CommitReservedAttachment, "/world_state/commit_attachment"
        )
        for _ in range(3):
            pending = self.call(client, attach)
            self.assertEqual(pending.status.code, Status.ATTACHMENT_CLOCK_NOT_READY)
            self.assertFalse(pending.has_reservation)
            self.assertEqual(self.snapshot().revision, before.revision)
        self.set_clock(clock, 9_700_000_000)
        applied = self.call(client, attach)
        self.assertEqual(applied.status.code, Status.OK, applied.status.detail)
        self.assertEqual(applied.reservation.stage, TaskReservation.STAGE_ATTACHED)
        # Treat the successful receipt as lost to the caller, then reconcile after a real
        # TimeSource backward update. The historical result cannot become ordinary OK.
        self.set_clock(clock, 9_000_000_000)
        replay = self.call(client, attach)
        self.assertEqual(replay.status.code, Status.CLOCK_AUTHORITY_INHIBITED)
        self.assertTrue(replay.has_reservation)
        self.assertEqual(replay.world_revision, applied.world_revision)
        self.assertEqual(replay.reservation, applied.reservation)
        proof = ValidateExecutionWorldAuthority.Request()
        proof.token = reserved.token
        proof.expected_reservation_id = applied.reservation.reservation_id
        proof.expected_reservation_revision = applied.reservation.revision
        proof.expected_object_id = selected.objects[0].id
        proof.expected_destination_lane_id = "lane"
        refused = self.call(
            self.node.create_client(
                ValidateExecutionWorldAuthority, "/world_state/validate_execution_authority"
            ),
            proof,
        )
        self.assertEqual(refused.status.code, Status.CLOCK_AUTHORITY_INHIBITED)
        self.assertFalse(refused.has_proof)
        self.assertEqual(self.snapshot().revision, applied.world_revision)


@launch_testing.post_shutdown_test()
class TestWorldStateManagedClockShutdown(unittest.TestCase):
    def test_process_exits_cleanly(self, proc_info, world_state, fixture):
        launch_testing.asserts.assertExitCodes(proc_info, process=world_state)
        fixture.cleanup()
