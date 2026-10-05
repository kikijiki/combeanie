# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Headless Gazebo-to-controller trajectory integration test."""

import subprocess
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
from restocker_interfaces.msg import LaneObservation


@pytest.mark.launch_test
def generate_test_description():
    """Start the simulation and run the finite-time smoke client."""
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "launch", "simulation.launch.py"]
            )
        ),
        launch_arguments={"gui": "false", "controller_timeout": "30.0"}.items(),
    )
    smoke_test = Node(
        package="restocker_control",
        executable="control_smoke_test",
        name="control_smoke_test",
        output="screen",
        parameters=[{"startup_timeout_sec": 30.0, "action_timeout_sec": 8.0}],
    )
    return (
        launch.LaunchDescription([simulation, smoke_test, launch_testing.actions.ReadyToTest()]),
        {"smoke_test": smoke_test},
    )


class TestSimulationRuntime(unittest.TestCase):
    """Assert the smoke client reaches its successful terminal state."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("simulation_lane_evidence_test")
        cls.lanes = {}
        cls.lane_subscription = cls.node.create_subscription(
            LaneObservation,
            "/perception/ground_truth/lane_observations",
            lambda message: cls.lanes.__setitem__(message.lane_id, message),
            20,
        )

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.lane_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_control_smoke_test_succeeds(self, proc_info, smoke_test):
        proc_info.assertWaitForShutdown(process=smoke_test, timeout=60)
        launch_testing.asserts.assertExitCodes(proc_info, process=smoke_test)

    def test_attachment_transport_boundary_is_advertised(self):
        expected = {
            "/world/restocking/restocker/attachment/set",
            "/world/restocking/restocker/attachment/query",
        }
        deadline = time.monotonic() + 15.0
        observed = set()
        while time.monotonic() < deadline:
            result = subprocess.run(
                ["gz", "service", "-l"],
                check=True,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            observed = set(result.stdout.splitlines())
            if expected <= observed:
                break
            time.sleep(0.1)
        self.assertTrue(expected <= observed, f"missing services: {expected - observed}")

    def test_ground_truth_publishes_empty_lane_evidence(self):
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if len(self.lanes) == 6:
                break
        self.assertEqual(
            set(self.lanes),
            {"lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06"},
        )
        for lane_id, observation in self.lanes.items():
            self.assertEqual(observation.header.frame_id, lane_id)
            stamp_ns = observation.header.stamp.sec * 1_000_000_000
            stamp_ns += observation.header.stamp.nanosec
            self.assertGreater(stamp_ns, 0)
            self.assertEqual(observation.status, LaneObservation.STATUS_OK)
            self.assertAlmostEqual(observation.available_depth_m, 0.85, places=9)
            self.assertFalse(observation.obstructed)
            self.assertEqual(observation.observed_source_object_ids, [])


@launch_testing.post_shutdown_test()
class TestGroundTruthAdapterDidNotCrash(unittest.TestCase):
    """Fail the run when the ground-truth adapter died on a fault."""

    def test_ground_truth_adapter_exited_cleanly(self, proc_info):
        """
        Assert `ground_truth_adapter` was not killed by SIGSEGV, SIGABRT, SIGBUS or SIGILL.

        Only fatal-signal exit codes are checked, so slow or signalled shutdown does not fail
        the test. Scoped to the adapter because the externally provided bridge processes crash
        independently of this package.
        """
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if "ground_truth_adapter" in info.process_name and info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"ground-truth adapter crashed: {crashed}")
