# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Prove SIGINT drains a real reservation before coordinator exit."""

import os
import signal
import unittest

from action_msgs.msg import GoalStatus
from coordinator_live_test_support import LiveShutdownClient
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from restocker_interfaces.action import RestockProduct

ACTION_NAME = "/test/clean/restock_product"
COORDINATOR_NODE_NAME = "live_shutdown_clean_coordinator"
STATUS_TOPIC = f"/{COORDINATOR_NODE_NAME}/status"


def _coordinator_parameters():
    return {
        "use_sim_time": False,
        "action_name": ACTION_NAME,
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
        "gripper_geometry_path": PathJoinSubstitution(
            [
                FindPackageShare("restocker_description"),
                "config",
                "gripper_geometry.yaml",
            ]
        ),
        "pump_period_ms": 10,
        "task.validation_timeout_ms": 500,
        "reconciliation.window_ms": 500,
        "reconciliation.attempt_timeout_ms": 200,
        "reconciliation.max_attempts": 2,
        "shutdown_timeout_ms": 2500,
        # Every age bound must outlast coordinator startup, since the evidence is stamped and
        # published once before a goal can be sent. The robot bound was left at its 500 ms default
        # and is the one startup races: latch-to-selection took 241-3642 ms (median 455) under
        # load, giving "robot telemetry is stale" failures. test_task_selection.cpp covers the
        # freshness rule itself; this test only needs the evidence to outlive goal admission.
        "selection.maximum_object_age_ms": 30000,
        "selection.lane_evidence_validity_ms": 30000,
        "selection.maximum_robot_age_ms": 30000,
    }


@pytest.mark.launch_test
def generate_test_description():
    """Start the real authority, static workcell transform, and coordinator."""
    world_state = Node(
        package="restocker_world_state",
        executable="world_state_node",
        name="live_shutdown_clean_world_state",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "maximum_observation_age_ms": 30000,
                # Ground-truth path pin (Milestone 10 Stage 7 default switch): this fixture
                # publishes its lane evidence on the ground-truth topic, not the camera one.
                "lane_observation_topic": "/perception/ground_truth/lane_observations",
                "lane_semantics_config": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_world_state"),
                        "config",
                        "baseline_lanes.yaml",
                    ]
                ),
                "workcell_geometry": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "workcell_geometry.yaml",
                    ]
                ),
                # The lane depth one more of each catalogued product costs. Every destination
                # predicate is a question about depth now, so a world state without this admits
                # no reservation at all.
                "product_catalog": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "product_collision_catalog.yaml",
                    ]
                ),
            }
        ],
    )
    transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="live_shutdown_clean_world_to_shelf",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.55",
            "--z",
            "0.75",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "shelf",
        ],
    )
    tool_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="live_shutdown_clean_tool_to_grasp_center",
        output="screen",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.14",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "tool0",
            "--child-frame-id",
            "grasp_center",
        ],
    )
    coordinator = Node(
        package="restocker_task_executor",
        executable="restock_action_coordinator",
        name=COORDINATOR_NODE_NAME,
        output="screen",
        parameters=[_coordinator_parameters()],
    )
    return (
        launch.LaunchDescription(
            [
                world_state,
                transform,
                tool_transform,
                coordinator,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"coordinator": coordinator, "world_state": world_state},
    )


class TestLiveCleanShutdown(unittest.TestCase):
    """Validate action, process, and surviving-authority outcomes together."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fixture = LiveShutdownClient(
            "live_shutdown_clean_test_client",
            ACTION_NAME,
            STATUS_TOPIC,
        )

    @classmethod
    def tearDownClass(cls):
        cls.fixture.destroy()
        rclpy.shutdown()

    def test_sigint_releases_before_clean_exit(
        self,
        coordinator,
        proc_info,
        proc_output,
    ):
        seeded = self.fixture.seed_authority()
        self.fixture.send_until_accepted(seeded)
        captured = self.fixture.wait_for_reservation()
        self.assertEqual(captured.object_source_id, "test:shutdown_can")

        os.kill(coordinator.process_details["pid"], signal.SIGINT)
        action_result = self.fixture.wait_for_result(timeout_sec=5.0)
        self.assertEqual(action_result.status, GoalStatus.STATUS_ABORTED)
        self.assertEqual(action_result.result.status, RestockProduct.Result.STATUS_SHUTDOWN)

        proc_info.assertWaitForShutdown(process=coordinator, timeout=5.0)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0],
            process=coordinator,
        )
        # assertWaitForShutdown already proved the process exited; this waits for launch_testing's
        # capture thread to hand over output already written. One second let scheduling delay
        # decide the result under load; five seconds matches the wait above and still fails a
        # coordinator that exits cleanly without saying so.
        proc_output.assertWaitFor(
            "coordinator drain completed cleanly",
            process=coordinator,
            timeout=5.0,
        )
        self.assertFalse(self.fixture.snapshot().has_active_reservation)


@launch_testing.post_shutdown_test()
class TestCleanShutdownProcess(unittest.TestCase):
    """Keep the world-state teardown result independent from coordinator exit."""

    def test_world_state_exited_normally(self, proc_info, world_state):
        launch_testing.asserts.assertExitCodes(proc_info, process=world_state)
