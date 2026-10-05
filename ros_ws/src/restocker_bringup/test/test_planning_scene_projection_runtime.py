# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Live authoritative-world-state to MoveIt projection acceptance test."""

import math
import time
import unittest

from geometry_msgs.msg import Pose
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import (
    ObjectObservation,
    PlanningSceneLease,
    PlanningSceneLeaseOperationStatus,
    PlanningSceneProjectionStatus,
)
from restocker_interfaces.srv import (
    AcquirePlanningSceneLease,
    GetWorldState,
    ReleasePlanningSceneLease,
    ValidatePlanningSceneLease,
)
from shape_msgs.msg import SolidPrimitive

EXPECTED_WORKCELL_IDS = {
    "restocker/workcell/overhead_camera_link/camera_collision",
    "restocker/workcell/shelf/front_retainer_collision",
    "restocker/workcell/shelf/lane_01_02_divider_collision",
    "restocker/workcell/shelf/lane_02_03_divider_collision",
    "restocker/workcell/shelf/left_frame_collision",
    "restocker/workcell/shelf/right_frame_collision",
    "restocker/workcell/roller_bed/roller_bed_collision",
    "restocker/workcell/shelf/stock_tray_collision",
}
EXPECTED_PRODUCT_DIMENSIONS = {
    ObjectObservation.PRODUCT_CLASS_CAN: [0.122, 0.033],
    ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE: [0.200, 0.034],
    ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE: [0.290, 0.045],
}
FOREIGN_ID = "foreign/projection_test_probe"
STALE_MANAGED_ID = "restocker/object/999999"


@pytest.mark.launch_test
def generate_test_description():
    """Start the complete headless baseline with persistent projection enabled."""
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
            "controller_timeout": "30.0",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


class TestPlanningSceneProjectionRuntime(unittest.TestCase):
    """Require verified geometry and restart-safe namespace reconciliation."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("planning_scene_projection_runtime_test")
        cls.snapshot_client = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.get_scene_client = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.apply_scene_client = cls.node.create_client(
            ApplyPlanningScene, "/apply_planning_scene"
        )
        cls.acquire_lease_client = cls.node.create_client(
            AcquirePlanningSceneLease,
            "/planning_scene_projection/acquire_lease",
        )
        cls.validate_lease_client = cls.node.create_client(
            ValidatePlanningSceneLease,
            "/planning_scene_projection/validate_lease",
        )
        cls.release_lease_client = cls.node.create_client(
            ReleasePlanningSceneLease,
            "/planning_scene_projection/release_lease",
        )
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
        cls.latest_status = status

    def _wait_until(self, predicate, timeout_sec, failure):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            value = predicate()
            if value is not None and value is not False:
                return value
        self.fail(failure)

    def _call(self, client, request, timeout_sec=10.0):
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertTrue(future.done(), "service call did not complete")
        self.assertIsNotNone(future.result(), "service call returned no response")
        return future.result()

    def _snapshot(self):
        request = GetWorldState.Request()
        request.include_removed = False
        request.include_events = False
        return self._call(self.snapshot_client, request).snapshot

    def _scene_objects(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        response = self._call(self.get_scene_client, request)
        return response.scene.world.collision_objects

    def _acquire_lease(self, minimum_revision):
        request = AcquirePlanningSceneLease.Request()
        request.operation_id = "projection-runtime-acquire"
        request.minimum_applied_revision = minimum_revision
        response = self._call(self.acquire_lease_client, request)
        self.assertIn(
            response.status.code,
            {
                PlanningSceneLeaseOperationStatus.DRAINING,
                PlanningSceneLeaseOperationStatus.GRANTED,
            },
        )
        if response.status.code == PlanningSceneLeaseOperationStatus.GRANTED:
            return response
        return None

    def _complete_snapshot(self):
        snapshot = self._snapshot()
        if len(snapshot.objects) == len(EXPECTED_PRODUCT_DIMENSIONS):
            return snapshot
        return None

    def _reconciled_scene(self):
        objects = self._scene_objects()
        objects_by_id = {item.id: item for item in objects}
        if (
            len(objects_by_id) == len(objects)
            and FOREIGN_ID in objects_by_id
            and STALE_MANAGED_ID not in objects_by_id
        ):
            return objects_by_id
        return None

    @staticmethod
    def _probe(object_id, x):
        collision = CollisionObject()
        collision.header.frame_id = "world"
        collision.id = object_id
        collision.pose.position.x = x
        collision.pose.orientation.w = 1.0
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.02, 0.02, 0.02]
        collision.primitives.append(primitive)
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        collision.primitive_poses.append(primitive_pose)
        collision.operation = CollisionObject.ADD
        return collision

    def test_projection_and_namespace_reconciliation(self):
        snapshot = self._wait_until(
            self._complete_snapshot,
            timeout_sec=45.0,
            failure="world state did not expose the complete configured scenario",
        )
        status = self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                and self.latest_status.applied_revision >= snapshot.revision
                else None
            ),
            timeout_sec=30.0,
            failure="projector did not verify the sampled world revision",
        )
        self.assertEqual(status.error_code, PlanningSceneProjectionStatus.ERROR_NONE)
        self.assertGreaterEqual(status.applied_revision, snapshot.revision)
        self.assertTrue(status.projector_epoch)
        self.assertGreater(status.scene_content_generation, 0)

        stable_status = status

        def periodic_no_op_proof():
            nonlocal stable_status
            latest = self.latest_status
            if (
                latest is None
                or latest.state != PlanningSceneProjectionStatus.STATE_APPLIED
                or latest.projector_epoch != stable_status.projector_epoch
                or latest.verification_epoch <= stable_status.verification_epoch
            ):
                return None
            if latest.scene_content_generation != stable_status.scene_content_generation:
                # Gazebo products may still be physically settling after the first proof.
                # Establish a new baseline, then require a later no-op verification of it.
                stable_status = latest
                return None
            return latest

        no_op_status = self._wait_until(
            periodic_no_op_proof,
            timeout_sec=5.0,
            failure="projector did not publish a periodic exact verification",
        )
        self.assertEqual(no_op_status.projector_epoch, stable_status.projector_epoch)
        self.assertEqual(
            no_op_status.scene_content_generation,
            stable_status.scene_content_generation,
        )

        objects = self._scene_objects()
        objects_by_id = {item.id: item for item in objects}
        self.assertEqual(len(objects_by_id), len(objects), "scene contains duplicate IDs")
        expected_product_ids = {f"restocker/object/{item.id}" for item in snapshot.objects}
        self.assertTrue(EXPECTED_WORKCELL_IDS.issubset(objects_by_id))
        self.assertTrue(expected_product_ids.issubset(objects_by_id))

        for tracked in snapshot.objects:
            collision = objects_by_id[f"restocker/object/{tracked.id}"]
            self.assertEqual(collision.header.frame_id, "world")
            self.assertEqual(len(collision.primitives), 1)
            self.assertEqual(collision.primitives[0].type, SolidPrimitive.CYLINDER)
            expected_dimensions = EXPECTED_PRODUCT_DIMENSIONS[tracked.product_class]
            self.assertEqual(len(collision.primitives[0].dimensions), 2)
            for actual, expected in zip(
                collision.primitives[0].dimensions, expected_dimensions, strict=True
            ):
                self.assertAlmostEqual(actual, expected, places=9)
            scene_position = collision.pose.position
            world_position = tracked.pose.pose.position
            position_error = math.sqrt(
                (scene_position.x - world_position.x) ** 2
                + (scene_position.y - world_position.y) ** 2
                + (scene_position.z - world_position.z) ** 2
            )
            self.assertLess(position_error, 0.002)

        acquisition = self._wait_until(
            lambda: self._acquire_lease(status.applied_revision),
            timeout_sec=15.0,
            failure="projector did not grant a lease over the verified revision",
        )
        self.assertTrue(acquisition.has_lease)
        self.assertTrue(acquisition.token)
        self.assertEqual(acquisition.lease.phase, PlanningSceneLease.PHASE_HELD)
        self.assertGreaterEqual(
            acquisition.lease.granted_applied_revision, status.applied_revision
        )

        held_status = self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state
                == PlanningSceneProjectionStatus.STATE_TRANSACTION_HELD
                and self.latest_status.lease_id == acquisition.lease.lease_id
                else None
            ),
            timeout_sec=5.0,
            failure="projector did not publish its held transaction state",
        )
        self.assertEqual(held_status.lease_phase, PlanningSceneLease.PHASE_HELD)

        validate_request = ValidatePlanningSceneLease.Request()
        validate_request.token = acquisition.token
        validation = self._call(self.validate_lease_client, validate_request)
        self.assertEqual(validation.status.code, PlanningSceneLeaseOperationStatus.VALID)
        self.assertTrue(validation.has_lease)

        mutation = PlanningScene()
        mutation.is_diff = True
        mutation.robot_state.is_diff = True
        mutation.world.collision_objects.extend(
            [self._probe(FOREIGN_ID, 2.0), self._probe(STALE_MANAGED_ID, 2.1)]
        )
        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = mutation
        self.assertTrue(self._call(self.apply_scene_client, apply_request).success)

        held_objects = {item.id: item for item in self._scene_objects()}
        self.assertIn(STALE_MANAGED_ID, held_objects)
        self.assertEqual(
            self.latest_status.state,
            PlanningSceneProjectionStatus.STATE_TRANSACTION_HELD,
        )

        release_request = ReleasePlanningSceneLease.Request()
        release_request.operation_id = "projection-runtime-release"
        release_request.token = acquisition.token
        release_request.required_semantic_revision = snapshot.revision
        release = self._call(self.release_lease_client, release_request)
        self.assertEqual(
            release.status.code,
            PlanningSceneLeaseOperationStatus.RELEASE_ACCEPTED,
        )
        self.assertTrue(release.has_lease)
        self.assertEqual(release.lease.phase, PlanningSceneLease.PHASE_RELEASING)

        invalidated = self._call(self.validate_lease_client, validate_request)
        self.assertEqual(
            invalidated.status.code,
            PlanningSceneLeaseOperationStatus.TOKEN_MISMATCH,
        )

        reconciled = self._wait_until(
            self._reconciled_scene,
            timeout_sec=15.0,
            failure=(
                "projector did not remove stale managed geometry while preserving foreign geometry"
            ),
        )
        self.assertTrue(EXPECTED_WORKCELL_IDS.issubset(reconciled))
        self.assertTrue(expected_product_ids.issubset(reconciled))

        released_status = self._wait_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                and self.latest_status.lease_phase == PlanningSceneLease.PHASE_NONE
                else None
            ),
            timeout_sec=5.0,
            failure="projector did not publish release reconciliation completion",
        )
        self.assertGreater(released_status.verification_epoch, held_status.verification_epoch)
        self.assertEqual(released_status.projector_epoch, status.projector_epoch)
        self.assertGreater(
            released_status.scene_content_generation,
            held_status.scene_content_generation,
        )
