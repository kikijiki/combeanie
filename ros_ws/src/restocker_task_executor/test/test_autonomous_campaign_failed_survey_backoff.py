# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""A survey that keeps failing must not spin one campaign cycle per round trip."""

import time
import unittest

from campaign_loop_support import FakeWorld, pitch_by_sku
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import SurveyLane, SurveyViewpoint
from restocker_interfaces.msg import RestockCoordinatorStatus
from restocker_interfaces.srv import GetWorldState

SHELF_ACTION = "/test/campaign_backoff/survey_lane"
TRAY_ACTION = "/test/campaign_backoff/survey_viewpoint"
RESTOCK_ACTION = "/test/campaign_backoff/restock_product"
STATUS_TOPIC = "/test/campaign_backoff/coordinator_status"
CAMPAIGN_STATUS_TOPIC = "/test/campaign_backoff/status"
WORLD_STATE_SERVICE = "/test/campaign_backoff/get_snapshot"

# Mirrors failed_cycle_backoff_sec's shipped default; the gap assertion allows jitter below it.
BACKOFF_FLOOR_S = 1.0


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero against a shelf survey that refuses every goal."""
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign_backoff_subject",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "workcell_geometry_path": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "workcell_geometry.yaml",
                    ]
                ),
                "product_catalog_path": PathJoinSubstitution(
                    [
                        FindPackageShare("restocker_description"),
                        "config",
                        "product_collision_catalog.yaml",
                    ]
                ),
                "restock_action_name": RESTOCK_ACTION,
                "shelf_survey_action_name": SHELF_ACTION,
                "tray_survey_action_name": TRAY_ACTION,
                "coordinator_status_topic": STATUS_TOPIC,
                "campaign_status_topic": CAMPAIGN_STATUS_TOPIC,
                "world_state_service_name": WORLD_STATE_SERVICE,
                "max_cycles": 3,
                "cycle_period_sec": 0.0,
                "server_wait_timeout_sec": 10.0,
                "action_timeout_sec": 5.0,
                # Card 086: a fresh fake world is settled; acknowledge the restart gate.
                "restart_acknowledged": True,
            }
        ],
    )
    return launch.LaunchDescription([campaign, launch_testing.actions.ReadyToTest()])


class TestFailedSurveyBackoff(unittest.TestCase):
    """Every shelf attempt must be separated by the failed-cycle floor."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("campaign_backoff_fixture")
        cls.world = FakeWorld(pitch_by_sku())
        cls.shelf_arrivals = []
        cls.status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_publisher = cls.node.create_publisher(
            RestockCoordinatorStatus, STATUS_TOPIC, cls.status_qos
        )
        cls.world_state_service = cls.node.create_service(
            GetWorldState, WORLD_STATE_SERVICE, cls._world_state
        )

        def on_shelf_goal(_goal_request):
            # The campaign's failure path starts at goal admission; rejection is the fastest
            # terminal answer, so only this callback can stamp the arrivals under test.
            cls.shelf_arrivals.append(time.monotonic())
            return GoalResponse.REJECT

        def never_execute(_goal_handle):
            raise AssertionError("a rejected shelf goal must never be executed")

        cls.shelf_server = ActionServer(
            cls.node,
            SurveyLane,
            SHELF_ACTION,
            execute_callback=never_execute,
            goal_callback=on_shelf_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        # Unreached while the shelf survey never completes; only the name must resolve. The
        # coordinated tray action is what the mode loop actually names, but this fixture only
        # needs wait_for_server to eventually time out the same way on either type.
        cls.tray_server = ActionServer(
            cls.node,
            SurveyViewpoint,
            TRAY_ACTION,
            execute_callback=never_execute,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )

    @classmethod
    def tearDownClass(cls):
        cls.shelf_server.destroy()
        cls.tray_server.destroy()
        cls.node.destroy_service(cls.world_state_service)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _world_state(cls, _request, response):
        response.snapshot = cls.world.snapshot()
        return response

    @classmethod
    def _publish_ready(cls):
        status = RestockCoordinatorStatus()
        status.startup_state = RestockCoordinatorStatus.STARTUP_READY
        status.admission_ready = True
        cls.status_publisher.publish(status)

    def test_failed_cycles_respect_the_backoff_floor(self):
        """Three refused cycles at cadence zero must still be >= floor apart."""
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and len(self.shelf_arrivals) < 3:
            # Republishing also proves readiness; the snapshot service runs on the same node
            # and must be spun for the campaign's validity measurement to answer.
            self._publish_ready()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertEqual(
            len(self.shelf_arrivals),
            3,
            f"expected exactly the three configured cycles, saw {len(self.shelf_arrivals)}",
        )
        gaps = [
            later - earlier
            for earlier, later in zip(self.shelf_arrivals, self.shelf_arrivals[1:], strict=False)
        ]
        # Without the floor each gap is one action round trip (~5 ms); with it each gap carries
        # at least failed_cycle_backoff_sec of steady-clock sleep.
        for gap in gaps:
            self.assertGreaterEqual(
                gap,
                BACKOFF_FLOOR_S * 0.9,
                f"cycle retry gap {gap:.3f}s is below the {BACKOFF_FLOOR_S}s floor: "
                "failed survey cycles are spinning",
            )
