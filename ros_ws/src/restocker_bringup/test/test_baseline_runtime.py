# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Headless Gazebo, controller, and MoveIt planning acceptance test."""

import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import RestockCoordinatorStatus
from restocker_moveit_config import build_moveit_config


@pytest.mark.launch_test
def generate_test_description():
    """Start the public baseline and retain the finite client action for exit assertions."""
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
            "planning_scene_projection": "false",
            # The smoke test owns MoveIt's scene, and the manipulation backends need the projector
            # (the attachment boundary takes its planning-scene lease from it). The manipulation
            # runtime test covers the motion-enabled case.
            "motion_enabled": "false",
            "controller_timeout": "30.0",
        }.items(),
    )
    moveit_config = build_moveit_config()
    smoke_test = Node(
        package="restocker_task_executor",
        executable="planning_smoke_test",
        name="planning_smoke_test",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {
                "use_sim_time": True,
                "startup_timeout_sec": 30.0,
                "planning_time_sec": 5.0,
            },
        ],
    )
    return (
        launch.LaunchDescription([baseline, smoke_test, launch_testing.actions.ReadyToTest()]),
        {"smoke_test": smoke_test},
    )


class TestBaselineRuntime(unittest.TestCase):
    """Require every bounded execution and collision assertion to pass."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("baseline_runtime_test")
        cls.latest_coordinator_status = None
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_subscription = cls.node.create_subscription(
            RestockCoordinatorStatus,
            "/restock_action_coordinator/status",
            cls._on_coordinator_status,
            status_qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_coordinator_status(cls, status):
        cls.latest_coordinator_status = status

    def test_coordinator_reaches_effective_readiness(self):
        deadline = time.monotonic() + 45.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            status = self.latest_coordinator_status
            if status is not None and status.admission_ready:
                self.assertEqual(status.startup_state, status.STARTUP_READY)
                self.assertFalse(status.inhibited)
                self.assertFalse(status.has_orphaned_reservation)
                return
        self.fail("baseline coordinator did not publish effective readiness")

    def test_planning_smoke_test_succeeds(self, proc_info, smoke_test):
        proc_info.assertWaitForShutdown(process=smoke_test, timeout=90)
        launch_testing.asserts.assertExitCodes(proc_info, process=smoke_test)
