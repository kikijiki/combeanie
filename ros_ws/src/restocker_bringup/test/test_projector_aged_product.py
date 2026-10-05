# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""An aged product stays an obstacle in a certified scene; it does not make the scene stale."""

# Card 071 (Milestone 10 §6, "An aged product is an obstacle, not a scene fault"). Card 066's
# dense dev run 1 ended when two tray products passed the projector's 600 s evidence age and the
# projector degraded the whole projection with ERROR_OBSERVATION_EVIDENCE_STALE, so every planning
# leg settled until the campaign's skip budget ran out. Here the world snapshot carries one free
# product last observed 700 s ago against the shipped 600 s horizon. The projector must certify
# the scene (STATE_APPLIED, ERROR_NONE), name the product as aged in the detail, and the applied
# MoveIt scene must hold its collision object: age never frees space.
#
# No simulator: the backend is fake_delayed_scene_backend.py, the same harness
# test_projector_snapshot_latency.py uses.

from pathlib import Path
import sys
import time
import unittest

from geometry_msgs.msg import Pose
import launch
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import PlanningSceneProjectionStatus

WORKCELL_DESCRIPTION = """<?xml version="1.0"?>
<robot name="aged_product_workcell">
  <link name="shelf">
    <collision name="fixture">
      <geometry><box size="0.10 0.10 0.10"/></geometry>
    </collision>
  </link>
</robot>
"""

MAX_OBSERVATION_AGE_SEC = 600.0
PRODUCT_AGE_SEC = 700.0
PRODUCT_ID = "restocker/object/1"


@pytest.mark.launch_test
def generate_test_description():
    """Start a backend whose snapshot carries an aged product, and a projector against it."""
    backend = ExecuteProcess(
        cmd=[
            sys.executable,
            str(Path(__file__).with_name("fake_delayed_scene_backend.py")),
            "--ros-args",
            "-p",
            "hold_first_apply:=false",
            "-p",
            f"aged_product_age_sec:={PRODUCT_AGE_SEC}",
        ],
        output="screen",
    )
    projector = Node(
        package="restocker_task_executor",
        executable="planning_scene_projector_node",
        name="planning_scene_projector_aged_product_test",
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
                "max_observation_age_sec": MAX_OBSERVATION_AGE_SEC,
            }
        ],
    )
    return launch.LaunchDescription([backend, projector, launch_testing.actions.ReadyToTest()])


class TestProjectorAgedProduct(unittest.TestCase):
    """Certify a scene whose only product is aged, with the product in it."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("projector_aged_product_test")
        cls.scene_client = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.statuses = []
        status_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            PlanningSceneProjectionStatus,
            "/planning_scene_projection/status",
            cls.statuses.append,
            status_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _applied(self):
        for status in reversed(self.statuses):
            if status.state == PlanningSceneProjectionStatus.STATE_APPLIED:
                return status
        return None

    def test_an_aged_product_is_certified_as_an_obstacle(self):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and self._applied() is None:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        applied = self._applied()
        stale = [
            status
            for status in self.statuses
            if status.error_code == PlanningSceneProjectionStatus.ERROR_OBSERVATION_EVIDENCE_STALE
        ]
        self.assertEqual(
            len(stale),
            0,
            "an aged product degraded the projection: "
            + "; ".join(status.detail for status in stale[:3]),
        )
        self.assertIsNotNone(
            applied,
            "projector never certified a scene whose product is older than its evidence age: "
            + (self.statuses[-1].detail if self.statuses else "no status published"),
        )
        self.assertEqual(applied.error_code, PlanningSceneProjectionStatus.ERROR_NONE)
        self.assertIn("held at last known pose", applied.detail)
        self.assertIn(PRODUCT_ID, applied.detail)

        # The scene MoveIt would plan against (the backend's applied scene) holds the product at
        # its last observed pose.
        self.assertTrue(self.scene_client.wait_for_service(timeout_sec=5.0))
        future = self.scene_client.call_async(GetPlanningScene.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertTrue(future.done() and future.result() is not None)
        products = [
            item for item in future.result().scene.world.collision_objects if item.id == PRODUCT_ID
        ]
        self.assertEqual(len(products), 1, "the aged product is not in the applied scene")
        pose: Pose = products[0].pose
        self.assertAlmostEqual(pose.position.x, 0.5, places=6)
        self.assertAlmostEqual(pose.position.y, -0.5, places=6)
