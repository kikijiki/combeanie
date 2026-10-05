# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Headless acceptance for the complete autonomous dense-restock demo story."""

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
                [FindPackageShare("restocker_gazebo"), "config", "dense_restock_products.yaml"]
            ),
            "lane_observation_topic": "/perception/ground_truth/lane_observations",
            "product_observation_max_age": "2.0",
            "selection_object_max_age_ms": "2000",
            "task_execution_timeout_ms": "180000",
            "placement_require_column_growth": "false",
            "autonomous_campaign": "true",
            # Card 086: a fresh simulated world is settled; acknowledge the restart gate.
            "campaign_restart_acknowledged": "true",
            # Match `just demo`: a transient incomplete survey retries a fresh cycle, while the
            # test still rejects either terminal failure phase immediately.
            "campaign_max_cycles": "0",
            "campaign_cycle_period": "0.0",
            # The mode loop idles at a zero section 4 deficit (target_count minus held at the
            # catalogued pitch). The demo declares targets the dense front can reach from its
            # measured starting columns (can 10, small 9, large 6 per lane: thirteen
            # transfers, back stock covers them with surplus) and stops short of packed
            # capacity, where the adaptive release inserts into the column and the rear-gap
            # growth under-counts against the pitch. Persistence is disabled so a stale
            # SetLanePolicy document cannot overlay these targets.
            "lane_semantics": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_world_state"),
                    "config",
                    "dense_restock_lanes.yaml",
                ]
            ),
            "lane_policy_state": "",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


class TestAutonomousRestockDemoRuntime(unittest.TestCase):
    """Require measured partial stock to become full while compatible tray stock remains."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("autonomous_restock_demo_runtime_test")
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

    def test_surveyed_partial_front_is_filled_without_exhausting_dense_back_stock(self):
        measured = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_MEASURED, 900.0)
        self.assertTrue(measured.front_survey_complete)
        self.assertTrue(measured.back_survey_complete)

        phases_before_measurement = [
            status.phase for status in self.statuses if status.sequence <= measured.sequence
        ]
        self.assertIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT,
            phases_before_measurement,
        )
        # The mode loop surveys the tray only when a deficit lacks a valid admitted candidate.
        # Ground-truth back stock is always fresh, so no SURVEYING_BACK may precede the first
        # transfer: overview cadence is not a thing the loop still runs on.
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_BACK,
            phases_before_measurement,
        )
        self.assertLess(
            phases_before_measurement.index(AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT),
            phases_before_measurement.index(AutonomousRestockCampaignStatus.PHASE_MEASURED),
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_RESTOCKING,
            phases_before_measurement,
            "no transfer may begin before the initial shelf survey and its measurement",
        )

        front = self._by_class(measured, measured.front_stock)
        deficits = self._by_class(measured, measured.front_deficits)
        back = self._by_class(measured, measured.back_stock)
        for product_class in measured.product_classes:
            self.assertGreater(front[product_class], 0, "every front type must start non-empty")
            self.assertGreater(
                deficits[product_class], 0, "every front type must start below capacity"
            )
            self.assertGreaterEqual(
                back[product_class],
                deficits[product_class],
                "back stock must cover every measured front deficit",
            )
        self.assertGreater(
            measured.total_back_stock,
            2,
            "the gate requires a dense tray, not a one- or two-product smoke fixture",
        )

        full = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_FRONT_FULL, 4200.0)
        self.assertEqual(full.total_front_deficit, 0)
        # Every measured deficit unit is filled; the run may spend extra product when a
        # released bottle stops short of the existing column and the rear-gap growth the
        # section 4 deficit reads does not register (observed: one placement moved the rear
        # gap 2 mm against a 67.8 mm pitch, so the held count lagged and the loop placed
        # again). placement_require_column_growth stays false in this demo for Card 019's
        # contact measurement, so packing spend is allowed; the conservation check below is
        # what pins it. Card 018/020 own the transfer-rate consequences.
        self.assertGreaterEqual(full.successful_transfers, measured.total_front_deficit)
        measured_back = self._by_class(measured, measured.back_stock)
        full_back = self._by_class(full, full.back_stock)
        consumed = {
            product_class: measured_back[product_class] - full_back[product_class]
            for product_class in full.product_classes
        }
        self.assertEqual(
            sum(consumed.values()),
            full.successful_transfers,
            "every successful transfer must consume exactly one back product per class",
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_BACK,
            [status.phase for status in self.statuses if status.sequence <= full.sequence],
            "valid admitted back stock must keep the tray survey out of the whole run",
        )
        self.assertFalse(
            any(
                status.phase
                in (
                    AutonomousRestockCampaignStatus.PHASE_BLOCKED,
                    AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                )
                for status in self.statuses
                if status.sequence <= full.sequence
            )
        )
