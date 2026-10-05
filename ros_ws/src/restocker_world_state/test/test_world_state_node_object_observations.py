# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Live contract checks for authoritative object-observation publisher auth."""

from __future__ import annotations

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from restocker_interfaces.msg import ObjectObservation
from restocker_interfaces.srv import GetWorldState


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


class TestWorldStateNodeObjectObservations(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("world_state_object_observation_test")

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

    def _observation(
        self,
        *,
        source_object_id: str,
        backend_name: str,
        offset_ns: int = 0,
    ) -> ObjectObservation:
        message = ObjectObservation()
        now_ns = self.node.get_clock().now().nanoseconds + offset_ns
        message.header.stamp.sec = now_ns // 1_000_000_000
        message.header.stamp.nanosec = now_ns % 1_000_000_000
        message.header.frame_id = "world"
        message.source_object_id = source_object_id
        message.product_class = ObjectObservation.PRODUCT_CLASS_CAN
        message.has_sku = True
        message.sku = "SIM-CAN"
        message.pose.pose.position.x = 0.25
        message.pose.pose.position.y = -0.5
        message.pose.pose.position.z = 0.6
        message.pose.pose.orientation.w = 1.0
        for index in range(6):
            message.pose.covariance[index * 6 + index] = 1.0e-8
        message.orientation = ObjectObservation.ORIENTATION_UPRIGHT
        message.confidence = 1.0
        message.backend_name = backend_name
        message.backend_version = "test"
        message.status = ObjectObservation.STATUS_OK
        return message

    def _object_by_source(self, snapshot, source_object_id: str):
        matches = [
            object_msg
            for object_msg in snapshot.objects
            if object_msg.source_object_id == source_object_id
        ]
        self.assertEqual(len(matches), 1, f"expected one object for {source_object_id}")
        return matches[0]

    def test_accepts_one_writer_and_fails_closed_on_identity_change(self) -> None:
        primary = self.node.create_publisher(
            ObjectObservation, "/perception/object_observations", 10
        )
        deadline = time.monotonic() + 10.0
        while primary.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(primary.get_subscription_count(), 1)

        sent = self._observation(source_object_id="sim:can_01", backend_name="primary_perception")
        primary.publish(sent)
        deadline = time.monotonic() + 5.0
        accepted = self._snapshot()
        while len(accepted.objects) == 0 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            accepted = self._snapshot()
        tracked = self._object_by_source(accepted, "sim:can_01")
        self.assertGreater(tracked.revision, 0)
        self.assertEqual(tracked.observation_time, sent.header.stamp)
        self.assertAlmostEqual(tracked.pose.pose.position.x, 0.25)
        latched_revision = accepted.revision

        primary.publish(
            self._observation(
                source_object_id="sim:can_02",
                backend_name="substituted_backend",
                offset_ns=1_000_000,
            )
        )
        time.sleep(0.25)
        backend_rejected = self._snapshot()
        self.assertEqual(backend_rejected.revision, latched_revision)
        self.assertEqual(len(backend_rejected.objects), 1)
        self.assertEqual(backend_rejected.objects[0].source_object_id, "sim:can_01")

        competitor = self.node.create_publisher(
            ObjectObservation, "/perception/object_observations", 10
        )
        deadline = time.monotonic() + 5.0
        while competitor.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(competitor.get_subscription_count(), 1)
        competitor.publish(
            self._observation(
                source_object_id="sim:can_03",
                backend_name="primary_perception",
                offset_ns=2_000_000,
            )
        )
        time.sleep(0.25)
        competitor_rejected = self._snapshot()
        self.assertEqual(competitor_rejected.revision, latched_revision)
        self.assertEqual(len(competitor_rejected.objects), 1)
        self.assertEqual(competitor_rejected.objects[0].source_object_id, "sim:can_01")

        self.node.destroy_publisher(competitor)
        self.node.destroy_publisher(primary)


@launch_testing.post_shutdown_test()
class TestWorldStateNodeObjectObservationsShutdown(unittest.TestCase):
    def test_process_exits_cleanly(self, proc_info, world_state) -> None:
        launch_testing.asserts.assertExitCodes(proc_info, process=world_state)
