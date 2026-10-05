# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Prove unavailable release authority causes fail-closed coordinator exit."""

import os
import signal
import time
import unittest

from coordinator_live_test_support import LiveShutdownClient
import launch
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
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

ACTION_NAME = "/test/timeout/restock_product"
COORDINATOR_NODE_NAME = "live_shutdown_timeout_coordinator"
STATUS_TOPIC = f"/{COORDINATOR_NODE_NAME}/status"
FAKE_RELEASE_SERVICE = "/test/timeout/release_reservation"
REPLACEMENT_ACTION_NAME = "/test/timeout/replacement/restock_product"
REPLACEMENT_STATUS_TOPIC = "/test/timeout/replacement/status"
REPLACEMENT_RESERVE_SERVICE = "/test/timeout/replacement/reserve_task"
REPLACEMENT_VALIDATE_SERVICE = "/test/timeout/replacement/validate_reservation"
REPLACEMENT_RELEASE_SERVICE = "/test/timeout/replacement/release_reservation"


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
        "world_state.release_reservation_service": FAKE_RELEASE_SERVICE,
        "pump_period_ms": 10,
        "task.validation_timeout_ms": 500,
        "reconciliation.window_ms": 500,
        "reconciliation.attempt_timeout_ms": 200,
        "reconciliation.max_attempts": 2,
        "shutdown_timeout_ms": 2500,
        # Every age bound must outlast coordinator startup, since the evidence is published once
        # before a goal can be admitted. The robot bound was left at its 500 ms default and is the
        # one startup races (latch-to-selection took 241-3642 ms under load).
        # test_task_selection.cpp covers the freshness rule itself; this test is about fail-closed
        # shutdown.
        "selection.maximum_object_age_ms": 30000,
        "selection.lane_evidence_validity_ms": 30000,
        "selection.maximum_robot_age_ms": 30000,
    }


@pytest.mark.launch_test
def generate_test_description():
    """Start the real authority and a coordinator with only release remapped."""
    world_state = Node(
        package="restocker_world_state",
        executable="world_state_node",
        name="live_shutdown_timeout_world_state",
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
        name="live_shutdown_timeout_world_to_shelf",
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
        name="live_shutdown_timeout_tool_to_grasp_center",
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
    replacement_parameters = _coordinator_parameters()
    replacement_parameters.update(
        {
            "action_name": REPLACEMENT_ACTION_NAME,
            "status_topic": REPLACEMENT_STATUS_TOPIC,
            "world_state.reserve_task_service": REPLACEMENT_RESERVE_SERVICE,
            "world_state.validate_reservation_service": REPLACEMENT_VALIDATE_SERVICE,
            "world_state.release_reservation_service": REPLACEMENT_RELEASE_SERVICE,
        }
    )
    replacement = Node(
        package="restocker_task_executor",
        executable="restock_action_coordinator",
        name="live_shutdown_timeout_replacement",
        output="screen",
        parameters=[replacement_parameters],
    )
    start_replacement = RegisterEventHandler(
        OnProcessExit(target_action=coordinator, on_exit=[replacement])
    )
    return (
        launch.LaunchDescription(
            [
                world_state,
                transform,
                tool_transform,
                coordinator,
                start_replacement,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            "coordinator": coordinator,
            "replacement": replacement,
            "world_state": world_state,
        },
    )


class TestLiveTimedOutShutdown(unittest.TestCase):
    """Require exact authority retention when release transport disappears."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fixture = LiveShutdownClient(
            "live_shutdown_timeout_test_client",
            ACTION_NAME,
            STATUS_TOPIC,
        )
        cls.fixture.advertise_fake_release(FAKE_RELEASE_SERVICE)
        cls.fixture.advertise_authority_guards(
            REPLACEMENT_RESERVE_SERVICE,
            REPLACEMENT_VALIDATE_SERVICE,
            REPLACEMENT_RELEASE_SERVICE,
        )
        cls.replacement_status = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        cls.status_subscription = cls.fixture.node.create_subscription(
            RestockCoordinatorStatus,
            REPLACEMENT_STATUS_TOPIC,
            cls._record_replacement_status,
            qos,
        )

    @classmethod
    def _record_replacement_status(cls, message):
        cls.replacement_status = message

    @classmethod
    def tearDownClass(cls):
        cls.fixture.node.destroy_subscription(cls.status_subscription)
        cls.fixture.destroy()
        rclpy.shutdown()

    def test_missing_release_times_out_with_exact_reservation(
        self,
        coordinator,
        replacement,
        proc_info,
        proc_output,
    ):
        seeded = self.fixture.seed_authority()
        self.fixture.send_until_accepted(seeded)
        captured = self.fixture.wait_for_reservation()
        self.fixture.assert_no_valid_result()

        self.fixture.withdraw_fake_release(FAKE_RELEASE_SERVICE)
        self.assertEqual(self.fixture.fake_release_calls, 0)
        os.kill(coordinator.process_details["pid"], signal.SIGINT)

        proc_info.assertWaitForShutdown(process=coordinator, timeout=6.0)
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[2],
            process=coordinator,
        )
        self.fixture.assert_no_valid_result()
        # The exit code above already proved the drain timed out; this waits only for
        # launch_testing's capture thread to deliver text already written.
        proc_output.assertWaitFor(
            "coordinator drain timed out: goal_generation=1 "
            "reservation_capability_may_remain=true",
            process=coordinator,
            timeout=5.0,
            stream="stderr",
        )

        retained = self.fixture.snapshot()
        self.assertTrue(self.fixture.reservation_matches(retained, captured))
        self.assertEqual(self.fixture.fake_release_calls, 0)

        replacement_deadline = time.monotonic() + 5.0
        while time.monotonic() < replacement_deadline:
            status = self.replacement_status
            if status is not None and status.startup_state == status.STARTUP_ORPHANED_RESERVATION:
                break
            rclpy.spin_once(self.fixture.node, timeout_sec=0.05)
        else:
            self.fail("replacement did not publish orphaned-reservation startup status")

        status = self.replacement_status
        self.assertTrue(status.inhibited)
        self.assertFalse(status.admission_ready)
        self.assertTrue(status.has_orphaned_reservation)
        self.assertEqual(
            type(captured).from_message(status.orphaned_reservation),
            captured,
        )
        self.fixture.assert_goal_rejected(
            REPLACEMENT_ACTION_NAME,
            captured.object_id,
            captured.destination_lane_id,
        )
        self.assertEqual(
            self.fixture.authority_guard_calls,
            {"reserve": 0, "validate": 0, "release": 0},
        )
        replacement_retained = self.fixture.snapshot()
        self.assertTrue(self.fixture.reservation_matches(replacement_retained, captured))
        # The status-topic wait above already established this state; this only confirms the
        # replacement also says so on stdout. A one-second budget let capture-thread scheduling
        # delay decide the result; five seconds bounds capture lag without weakening the assertion.
        proc_output.assertWaitFor(
            "coordinator startup state=orphaned_reservation",
            process=replacement,
            timeout=5.0,
        )


@launch_testing.post_shutdown_test()
class TestTimedOutShutdownProcess(unittest.TestCase):
    """Keep the authority process teardown independent from timeout code two."""

    def test_world_state_and_replacement_exited_normally(
        self, proc_info, world_state, replacement
    ):
        launch_testing.asserts.assertExitCodes(proc_info, process=world_state)
        launch_testing.asserts.assertExitCodes(proc_info, process=replacement)
