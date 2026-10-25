# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Headless acceptance for the showcase scene `just demo` opens: labelled products, five SKUs."""

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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import AutonomousRestockCampaignStatus

CYCLES = 2


@pytest.mark.launch_test
def generate_test_description():
    """Run the public baseline composition with the demo's scenario and campaign switches."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            # Ground-truth path pins (Milestone 10 Stage 7 default switch):
            "object_observation_topic": "/perception/object_observations",
            "tray_overview_perception": "false",
            "tray_confirm_perception": "false",
            "gui": "false",
            "rviz": "false",
            "planning_smoke": "false",
            "motion_enabled": "true",
            "controller_timeout": "30.0",
            "scenario_config": PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "config", "showcase_products.yaml"]
            ),
            "world": PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "worlds", "restocking_showcase.sdf"]
            ),
            "bundle_scenario": "true",
            "scene_materials": PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "config", "showcase_materials.yaml"]
            ),
            "lane_observation_topic": "/perception/ground_truth/lane_observations",
            "product_observation_max_age": "2.0",
            "selection_object_max_age_ms": "2000",
            "task_execution_timeout_ms": "180000",
            "placement_require_column_growth": "false",
            "autonomous_campaign": "true",
            # Card 086: a fresh simulated world is settled; acknowledge the restart gate.
            "campaign_restart_acknowledged": "true",
            # Two progress cycles, each at most one transfer: the run ends in PHASE_COMPLETE at
            # the bound, and the test rejects either terminal failure phase immediately. The whole
            # eleven-transfer run is the demo's own story, not this gate's.
            "campaign_max_cycles": str(CYCLES),
            "campaign_cycle_period": "0.0",
            # The mode loop idles at a zero section 4 deficit (target_count minus held at the
            # catalogued pitch). showcase_lanes.yaml declares can 10, small bottle 9 and large
            # bottle 6 per lane, short of surveyed capacity where the adaptive release inserts
            # into the column and the rear-gap growth under-counts against the pitch. Persistence
            # is disabled so a stale SetLanePolicy document cannot overlay these targets.
            "lane_semantics": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_world_state"),
                    "config",
                    "showcase_lanes.yaml",
                ]
            ),
            "lane_policy_state": "",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


class TestShowcaseRestockRuntime(unittest.TestCase):
    """Require the measured showcase shelf to receive stock from the packed tray."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("showcase_restock_runtime_test")
        cls.statuses = []
        qos = QoSProfile(
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscription = cls.node.create_subscription(
            AutonomousRestockCampaignStatus,
            "/autonomous_restock_campaign/status",
            cls.statuses.append,
            qos,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _wait_for_phase(self, phase, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            matching = [status for status in self.statuses if status.phase == phase]
            if matching:
                return matching[-1]
            blocked = [
                status
                for status in self.statuses
                if status.phase
                in (
                    AutonomousRestockCampaignStatus.PHASE_BLOCKED,
                    AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                )
            ]
            if blocked:
                self.fail(f"campaign terminated before front full: {blocked[-1].detail}")
        phases = [(status.sequence, status.phase, status.detail) for status in self.statuses]
        self.fail(f"timed out waiting for campaign phase {phase}; observed {phases}")

    @staticmethod
    def _by_class(status, values):
        return dict(zip(status.product_classes, values, strict=True))

    def test_showcase_shelf_is_surveyed_and_two_cycles_move_stock_from_the_packed_tray(self):
        measured = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_MEASURED, 900.0)
        self.assertTrue(measured.front_survey_complete)
        self.assertTrue(measured.back_survey_complete)
        phases_before_measurement = [
            status.phase for status in self.statuses if status.sequence <= measured.sequence
        ]
        self.assertIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT, phases_before_measurement
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_RESTOCKING,
            phases_before_measurement,
            "no transfer may begin before the initial shelf survey and its measurement",
        )

        # The showcase's measured inventory: 15 cans, 15 small bottles and 9 large bottles stand
        # on the shelf against targets of 10/9/6 per lane, and the packed tray behind holds
        # 19 cans, 19 small bottles and 7 large bottles. Counts are per class: the campaign keys
        # its accounting by class and the SKU rules are the task selection's.
        front = self._by_class(measured, measured.front_stock)
        deficits = self._by_class(measured, measured.front_deficits)
        back = self._by_class(measured, measured.back_stock)
        self.assertEqual(sorted(front.values()), [9, 15, 15])
        self.assertEqual(sorted(deficits.values()), [3, 3, 5])
        for product_class in measured.product_classes:
            self.assertGreater(front[product_class], 0)
            self.assertGreater(deficits[product_class], 0)
            self.assertGreaterEqual(back[product_class], deficits[product_class])
        self.assertEqual(measured.total_back_stock, 45)

        complete = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_COMPLETE, 2400.0)
        self.assertEqual(complete.successful_transfers, CYCLES)
        # The campaign measures after each transfer and not after the last one it is allowed, so
        # the back stock on the completion status lags by one: the last measurement is the one
        # whose consumption must match the transfers it had counted.
        last = [
            status
            for status in self.statuses
            if status.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
        ][-1]
        measured_back = self._by_class(measured, measured.back_stock)
        last_back = self._by_class(last, last.back_stock)
        consumed = sum(
            measured_back[product_class] - last_back[product_class]
            for product_class in last.product_classes
        )
        self.assertEqual(consumed, last.successful_transfers)
        self.assertGreaterEqual(last.successful_transfers, CYCLES - 1)
        self.assertFalse(
            any(
                status.phase
                in (
                    AutonomousRestockCampaignStatus.PHASE_BLOCKED,
                    AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                )
                for status in self.statuses
            )
        )
