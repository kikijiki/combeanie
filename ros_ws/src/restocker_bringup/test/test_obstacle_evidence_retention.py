# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Prove stale obstacle evidence degrades the projection and keeps the geometry."""
# Fail-closed obstacle evidence, without the simulator. A stale product observation may be
# dropped (removing a product only opens space). A stale obstacle observation means the sensor can
# no longer see, so dropping the box would hand the planner a corridor with no evidence it is
# clear. Also checks that an arriving obstacle advances `scene_content_generation`, the counter
# every in-flight plan is judged against.

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
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import (
    ObstacleBox,
    ObstacleObservation,
    PlanningSceneProjectionStatus,
)
from std_srvs.srv import Trigger

WORKCELL_DESCRIPTION = """<?xml version="1.0"?>
<robot name="obstacle_test_workcell">
  <link name="shelf">
    <collision name="fixture">
      <geometry><box size="0.10 0.10 0.10"/></geometry>
    </collision>
  </link>
</robot>
"""

OBSTACLE_TOPIC = "/perception/obstacle_observations"
OBSTACLE_ID_PREFIX = "restocker/obstacle/"
# Far from the modelled fixture at the origin, so the known-geometry subtraction has no say here.
OBSTACLE_CENTRE = (0.90, -0.35, 0.30)
OBSTACLE_SIZE = (0.15, 0.15, 0.50)
OBSTACLE_MAX_AGE_SEC = 1.0


@pytest.mark.launch_test
def generate_test_description():
    """Start the projector against the controllable scene backend, with no simulator."""
    backend = ExecuteProcess(
        cmd=[sys.executable, str(Path(__file__).with_name("fake_delayed_scene_backend.py"))],
        output="screen",
    )
    projector = Node(
        package="restocker_task_executor",
        executable="planning_scene_projector_node",
        name="planning_scene_projector_obstacle_retention_test",
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
                "reconcile_period_sec": 0.2,
                "service_timeout_sec": 2.0,
                "obstacle_max_age_sec": OBSTACLE_MAX_AGE_SEC,
            }
        ],
    )
    return launch.LaunchDescription([backend, projector, launch_testing.actions.ReadyToTest()])


class TestObstacleEvidenceRetention(unittest.TestCase):
    """Require retention, degradation, and content-generation movement."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("obstacle_evidence_retention_test")
        cls.release_backend = cls.node.create_client(Trigger, "/fake_scene_backend/release_apply")
        cls.planning_scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.obstacle_publisher = cls.node.create_publisher(ObstacleObservation, OBSTACLE_TOPIC, 10)
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
        cls.publishing = False
        cls.sequence = 0
        cls.node.create_timer(0.1, cls._publish_obstacle)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_status(cls, status):
        cls.latest_status = status

    @classmethod
    def _publish_obstacle(cls):
        if not cls.publishing:
            return
        message = ObstacleObservation()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.sensor_frame = "overhead_camera_optical_frame"
        cls.sequence += 1
        message.sequence = cls.sequence
        message.robot_static = True
        box = ObstacleBox()
        box.center.x, box.center.y, box.center.z = OBSTACLE_CENTRE
        box.size.x, box.size.y, box.size.z = OBSTACLE_SIZE
        box.point_count = 640
        message.boxes.append(box)
        cls.obstacle_publisher.publish(message)

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            result = predicate()
            if result:
                return result
        self.fail(f"timed out waiting for {description}")

    def _applied(self):
        status = self.latest_status
        if (
            status is not None
            and status.state == PlanningSceneProjectionStatus.STATE_APPLIED
            and status.error_code == PlanningSceneProjectionStatus.ERROR_NONE
        ):
            return status
        return None

    def _release_first_apply(self):
        """Let the backend's held first apply through."""

        def released():
            if not self.release_backend.wait_for_service(timeout_sec=0.2):
                return False
            future = self.release_backend.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            return future.result() is not None and future.result().success

        self._spin_until(released, 30.0, "the backend to release its first apply")

    def _obstacle_objects(self):
        self.assertTrue(self.planning_scene.wait_for_service(timeout_sec=10.0))
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future = self.planning_scene.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertIsNotNone(future.result(), "the scene backend did not answer")
        return {
            obj.id: obj
            for obj in future.result().scene.world.collision_objects
            if obj.id.startswith(OBSTACLE_ID_PREFIX)
        }

    def test_stale_obstacle_evidence_degrades_the_projection_without_dropping_geometry(self):
        """Walk the obstacle from absent to applied to stale to applied again."""
        self._release_first_apply()
        baseline = self._spin_until(self._applied, 60.0, "the projector to certify the scene")
        self.assertEqual(self._obstacle_objects(), {}, "the scene carried an unobserved obstacle")

        # 1. Evidence arrives and the obstacle becomes managed geometry.
        type(self).publishing = True
        observed = self._spin_until(
            lambda: self._obstacle_objects() or None,
            30.0,
            "the obstacle to reach the planning scene",
        )
        self.assertEqual(len(observed), 1, f"expected one obstacle, got {sorted(observed)}")
        obstacle = next(iter(observed.values()))
        self.assertAlmostEqual(obstacle.pose.position.x, OBSTACLE_CENTRE[0], places=6)
        self.assertAlmostEqual(obstacle.pose.position.y, OBSTACLE_CENTRE[1], places=6)
        self.assertAlmostEqual(obstacle.pose.position.z, OBSTACLE_CENTRE[2], places=6)
        self.assertEqual(len(obstacle.primitives), 1)
        # Padded on every face: a single depth view cannot bound the far side.
        for observed_extent, reported in zip(
            OBSTACLE_SIZE, obstacle.primitives[0].dimensions, strict=True
        ):
            self.assertGreater(reported, observed_extent)

        applied = self._spin_until(
            self._applied, 30.0, "the projector to certify the scene with the obstacle in it"
        )
        # Must move, or an arriving obstacle would not invalidate an earlier plan.
        self.assertGreater(
            applied.scene_content_generation,
            baseline.scene_content_generation,
            "an obstacle reached the scene without advancing the collision-content generation",
        )

        # 2. Evidence stops. The projection must stop being certified and the geometry must stay.
        type(self).publishing = False
        degraded = self._spin_until(
            lambda: (
                self.latest_status
                if self.latest_status is not None
                and self.latest_status.state == PlanningSceneProjectionStatus.STATE_DEGRADED
                and self.latest_status.error_code
                == PlanningSceneProjectionStatus.ERROR_OBSTACLE_EVIDENCE_STALE
                else None
            ),
            30.0,
            "the projector to degrade on stale obstacle evidence",
        )
        self.assertGreater(degraded.applied_revision, 0)
        retained = self._obstacle_objects()
        self.assertEqual(
            len(retained),
            1,
            "the obstacle was dropped when its evidence went stale; a sensor that cannot see is "
            "not a corridor that is clear",
        )

        # 3. Evidence returns. The projection is certified again, unchanged.
        type(self).publishing = True
        recovered = self._spin_until(
            self._applied, 30.0, "the projector to recover once evidence returned"
        )
        self.assertEqual(
            recovered.scene_content_generation,
            applied.scene_content_generation,
            "unchanged obstacle geometry announced itself as new collision content",
        )
