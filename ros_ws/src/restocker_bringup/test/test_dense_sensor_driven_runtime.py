# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 063: the dense restock scenario, sensor-driven.

The dense fixture stocks 9 cans, 9 small bottles and 7 large bottles on the tray. Camera
identity used to refuse every class stocked more than once, so on the shipped camera default
this scenario identified nothing. The test runs baseline.launch.py's own defaults (no evidence
pins) on the dense fixture and lanes and requires two things:

- the mode loop fills the measured front to PHASE_FRONT_FULL with no terminal failure phase;
- every object observation admitted on the ingest topic names the product ground truth has at
  that place (the identity receipt), and more than one instance of each class gets a name.

Ground truth is read here, by the test, and by the evaluators; no execution-path node reads it
(Card 010 SC-002 is asserted by test_sensor_driven_acceptance_runtime).
"""

import math
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import AutonomousRestockCampaignStatus, ObjectObservation

INGEST_OBJECT_TOPIC = "/perception/object_observations"
GT_OBJECT_TOPIC = "/perception/ground_truth/object_observations"
CONFIRM_BACKEND = "wrist_rgbd_tray_confirm"

# An admitted observation names the wrong product when the ground truth it names is further
# away than this. Distinct tray products stand at least 0.085 m apart and the confirm duty's
# measured error is millimetres, so 0.05 m separates "right product" from "a neighbour".
IDENTITY_MISMATCH_M = 0.05


@pytest.mark.launch_test
def generate_test_description():
    """Run the shipped sensor-driven baseline on the dense fixture."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "planning_smoke": "false",
            "motion_enabled": "true",
            "controller_timeout": "60.0",
            "scenario_config": PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "config", "dense_restock_products.yaml"]
            ),
            "lane_semantics": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_world_state"),
                    "config",
                    "dense_restock_lanes.yaml",
                ]
            ),
            # No persisted SetLanePolicy document may overlay the dense targets.
            "lane_policy_state": "",
            # No evidence pins: the sensor-driven configuration is baseline.launch.py's default,
            # exactly what `just demo` runs.
            "task_execution_timeout_ms": "180000",
            "autonomous_campaign": "true",
            # Card 086: a fresh simulated world is settled; acknowledge the restart gate.
            "campaign_restart_acknowledged": "true",
            "campaign_max_cycles": "0",
            # Card 048 teardown ordering, as in test_sensor_driven_acceptance_runtime: a parked
            # window between cycles keeps the launch SIGINT out of a MoveGroup plan.
            "campaign_cycle_period": "10.0",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


# See test_sensor_driven_acceptance_runtime: startup slop under contention is not acceptance.
generate_test_description.__ready_to_test_action_timeout__ = 60.0


class TestDenseSensorDriven(unittest.TestCase):
    """Dense tray, camera identity by pose, front filled."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dense_sensor_driven_test")
        cls.statuses = []
        cls.gt_latest: dict[str, ObjectObservation] = {}
        # (source_object_id, distance to that id's latest ground truth, or None when absent)
        cls.identity_samples: list[tuple[str, float | None]] = []
        status_qos = QoSProfile(
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscriptions = [
            cls.node.create_subscription(
                AutonomousRestockCampaignStatus,
                "/autonomous_restock_campaign/status",
                cls.statuses.append,
                status_qos,
            ),
            cls.node.create_subscription(
                ObjectObservation,
                GT_OBJECT_TOPIC,
                cls._on_ground_truth,
                QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE),
            ),
            cls.node.create_subscription(
                ObjectObservation,
                INGEST_OBJECT_TOPIC,
                cls._on_ingest,
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_ground_truth(cls, message: ObjectObservation) -> None:
        if message.source_object_id:
            cls.gt_latest[message.source_object_id] = message

    @classmethod
    def _on_ingest(cls, message: ObjectObservation) -> None:
        if message.backend_name != CONFIRM_BACKEND:
            return
        truth = cls.gt_latest.get(message.source_object_id)
        if truth is None:
            cls.identity_samples.append((message.source_object_id, None))
            return
        estimate = message.pose.pose.position
        actual = truth.pose.pose.position
        distance = math.dist((estimate.x, estimate.y, estimate.z), (actual.x, actual.y, actual.z))
        cls.identity_samples.append((message.source_object_id, distance))
        if distance > IDENTITY_MISMATCH_M:
            print(
                f"identity receipt: '{message.source_object_id}' published at "
                f"({estimate.x:.4f}, {estimate.y:.4f}, {estimate.z:.4f}) but its ground truth "
                f"is {distance:.4f} m away at ({actual.x:.4f}, {actual.y:.4f}, {actual.z:.4f})",
                flush=True,
            )

    def _identity_summary(self) -> str:
        named = {source_id for source_id, _ in self.identity_samples}
        distances = [d for _, d in self.identity_samples if d is not None]
        worst = max(distances) if distances else float("nan")
        return (
            f"identity receipt: {len(self.identity_samples)} admitted confirm observations, "
            f"{len(named)} distinct names, worst distance to the named ground truth "
            f"{worst:.4f} m"
        )

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
                self.fail(
                    f"campaign terminated before front full: {blocked[-1].detail}\n"
                    f"{self._identity_summary()}"
                )
        phases = [(status.sequence, status.phase, status.detail) for status in self.statuses]
        self.fail(
            f"timed out waiting for campaign phase {phase}; observed {phases}\n"
            f"{self._identity_summary()}"
        )

    def test_dense_front_is_filled_from_camera_named_tray_products(self):
        measured = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_MEASURED, 900.0)
        self.assertTrue(measured.front_survey_complete, "cold-start shelf survey incomplete")
        self.assertGreater(measured.total_front_deficit, 0, "the dense front must start short")
        print(
            f"measured: deficit {measured.total_front_deficit}, back {measured.total_back_stock}",
            flush=True,
        )

        full = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_FRONT_FULL, 4200.0)
        print(
            f"front full: transfers {full.successful_transfers}, "
            f"survey arm {full.survey_arm_time_sec:.1f} s, "
            f"transfer arm {full.transfer_arm_time_sec:.1f} s",
            flush=True,
        )
        print(self._identity_summary(), flush=True)
        self.assertEqual(full.total_front_deficit, 0)
        self.assertGreater(full.successful_transfers, 0)
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

        # The identity receipt: no admitted observation named a product that is not there.
        mismatches = [
            (source_id, distance)
            for source_id, distance in self.identity_samples
            if distance is None or distance > IDENTITY_MISMATCH_M
        ]
        self.assertEqual(mismatches, [], self._identity_summary())
        # Multi-instance naming actually happened: more than one product of a class was named.
        by_class: dict[int, set[str]] = {}
        for source_id, _ in self.identity_samples:
            truth = self.gt_latest.get(source_id)
            if truth is not None:
                by_class.setdefault(truth.product_class, set()).add(source_id)
        self.assertTrue(
            any(len(names) > 1 for names in by_class.values()),
            f"no class had more than one named instance: {by_class}",
        )


@launch_testing.post_shutdown_test()
class TestDenseSensorDrivenShutdown(unittest.TestCase):
    """No node may die of a fatal signal during the run."""

    def test_no_fatal_signals(self, proc_info):
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        # SIGKILL (-9) is launch's teardown escalation under load, not a crash (see
        # test_sensor_driven_acceptance_runtime).
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"process crashed: {crashed}")
