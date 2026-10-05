# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Lane survey: look down one lane and compare the measured free depth with ground truth."""
# The arm is driven to each lane's survey station, one RGB-D acquisition is taken there, and the
# free depth the camera measures is compared with the free depth the simulator computes from the
# poses it holds. Both producers run on separate topics for the whole test, so the comparison is
# made on the same instant.
#
# The scenario stands known columns in four lanes and leaves two empty, spanning full (twelve
# cans, 0.036 m free), a middling column, one product, and nothing. Products are pinned in place
# and checked against ground truth first: the simulator has no cylinder/cylinder collider (dartsim
# on ODE's narrowphase) and every catalogued product is a cylinder, so a column left to settle
# would arrive as a heap at one pose. Pinning avoids product-to-product contact.

import json
import os
from pathlib import Path
import statistics
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_gazebo.lane_columns import ground_truth_available_depth_m
from restocker_interfaces.action import SurveyLane
from restocker_interfaces.msg import (
    LaneDepthErrorSample,
    LaneObservation,
    PlanningSceneProjectionStatus,
)
import yaml

SCENARIO = "lane_column_products.yaml"

# A campaign runs this file against many generated scenarios, so the arrangement and the row log
# are overridable. With nothing set this is the shipped scenario and nothing is written.
SCENARIO_OVERRIDE = os.environ.get("RESTOCKER_LANE_SURVEY_SCENARIO", "")
SAMPLE_LOG = os.environ.get("RESTOCKER_LANE_SURVEY_SAMPLES", "")
RUN_LABEL = os.environ.get("RESTOCKER_LANE_SURVEY_LABEL", "shipped")
LANES = ("lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06")
GROUND_TRUTH_TOPIC = "/perception/ground_truth/lane_observations"
MEASUREMENT_TOPIC = "/perception/lane_observations"
ERROR_TOPIC = "/perception/lane_depth_error"

# How far a spawned product may sit from where the scenario put it. The bodies are static, so this
# checks that the spawn took and nothing merged, not that physics settled.
SPAWN_TOLERANCE_M = 0.001

# Bound on the camera's agreement with ground truth, from three terms:
#
#  * Estimator geometry: over thirty-four columns from empty to full for all three products, the
#    worst disagreement with ground truth is 0.014 mm, always toward emptier. See
#    test_lane_depth_measurement.cpp.
#  * The bed's floor band: a product leans 4 degrees with the bed, so its rearmost point is the
#    bottom rim, inside the 10 mm band excluded as floor. The measurement finds the barrel rear
#    10 mm up instead, 10 mm * tan(4 degrees) = 0.7 mm forward of the rim.
#  * The arm's pose, which enters through forward kinematics at the acquisition stamp (not the
#    commanded pose), so it costs only the disagreement between the published chain and the
#    simulator's. The run prints the distribution.
#
# 5 mm covers these and is an order of magnitude below the smallest product pitch (65.8 mm for a
# can), which is what a lane count is made of.
AGREEMENT_BOUND_M = 0.005


@pytest.mark.launch_test
def generate_test_description():
    """Start the planning composition with the wrist camera and the lane survey server."""
    scenario = (
        SCENARIO_OVERRIDE
        if SCENARIO_OVERRIDE
        else PathJoinSubstitution([FindPackageShare("restocker_gazebo"), "config", SCENARIO])
    )
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
            "cameras": "true",
            "wrist_camera": "true",
            "ground_truth": "true",
            "scenario_config": scenario,
            # Object perception is not under test here.
            "perception": "false",
            # The survey's motion port and the coordinator's are two clients of one move_group;
            # only one may drive the arm.
            "task_coordinator": "false",
            "attachment_adapter": "false",
            "planning_smoke": "false",
            "survey_viewpoint": "false",
            "planning_scene_projection": "true",
            "lane_survey": "true",
            "controller_timeout": "60.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


def _scenario_columns() -> dict[str, list[str]]:
    """Return the geometry of every product the scenario stands in each lane."""
    per_lane: dict[str, list[str]] = {lane: [] for lane in LANES}
    for lane, products in _scenario_products().items():
        per_lane[lane] = [product["geometry_key"] for product in products]
    return per_lane


def _scenario_products() -> dict[str, list[dict]]:
    """Return every product the scenario stands, keyed by the lane it stands it in."""
    scenario_path = (
        Path(SCENARIO_OVERRIDE)
        if SCENARIO_OVERRIDE
        else Path(get_package_share_directory("restocker_gazebo")) / "config" / SCENARIO
    )
    document = yaml.safe_load(scenario_path.read_text())
    per_lane: dict[str, list[dict]] = {lane: [] for lane in LANES}
    for product in document["products"]:
        # Model names are "<lane>_<product class>_<index>" and a product class contains an
        # underscore, so the lane is the first two fields.
        lane = "_".join(product["model_name"].split("_")[:2])
        per_lane.setdefault(lane, []).append(product)
    return per_lane


def _expected_free_depth(columns: dict[str, list[str]]) -> dict[str, float]:
    """Free depth the scenario's own arithmetic says each lane holds."""
    return {
        lane: ground_truth_available_depth_m(
            geometries[0] if geometries else "can.standard", len(geometries)
        )
        for lane, geometries in columns.items()
    }


def _record(row: dict) -> None:
    """Append one measured row to the campaign log, when a campaign asked for one."""
    if not SAMPLE_LOG:
        return
    path = Path(SAMPLE_LOG)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")


class TestLaneSurveyMeasuresDepletion(unittest.TestCase):
    """Survey every lane and report how far the camera and the simulator disagree."""

    @classmethod
    def setUpClass(cls):
        """Bring up a client node and subscribe to both lane streams and the evaluator."""
        rclpy.init()
        cls.node = rclpy.create_node("lane_survey_runtime_test")
        cls.ground_truth: dict[str, LaneObservation] = {}
        cls.measurements: dict[str, LaneObservation] = {}
        cls.samples: list[LaneDepthErrorSample] = []
        cls.node.create_subscription(
            LaneObservation,
            GROUND_TRUTH_TOPIC,
            lambda message: cls.ground_truth.__setitem__(message.lane_id, message),
            20,
        )
        cls.node.create_subscription(
            LaneObservation,
            MEASUREMENT_TOPIC,
            lambda message: cls.measurements.__setitem__(message.lane_id, message),
            20,
        )
        cls.node.create_subscription(LaneDepthErrorSample, ERROR_TOPIC, cls.samples.append, 20)
        cls.projection_status = None
        cls.node.create_subscription(
            PlanningSceneProjectionStatus,
            "/planning_scene_projection/status",
            cls._on_projection_status,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.client = ActionClient(cls.node, SurveyLane, "survey_lane")
        cls.products = _scenario_products()
        cls.columns = _scenario_columns()
        cls.expected = _expected_free_depth(cls.columns)

    @classmethod
    def _on_projection_status(cls, message):
        cls.projection_status = message

    @classmethod
    def tearDownClass(cls):
        """Tear the client node down."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _expected_ids(self) -> dict[str, list[str]]:
        return {
            lane: sorted(product["source_object_id"] for product in self.products.get(lane, []))
            for lane in LANES
        }

    def _arrangement_is_complete(self) -> bool:
        expected = self._expected_ids()
        return all(
            lane in self.ground_truth
            and sorted(self.ground_truth[lane].observed_source_object_ids) == expected[lane]
            for lane in LANES
        )

    def _await_ground_truth(self, timeout_s: float = 180.0) -> None:
        # Naming every lane is not enough: products spawn one process at a time, so a column can
        # still be filling (a twelve-can lane read eleven products with the depth already right,
        # since free depth is set by the rearmost product). Wait for exactly the scenario's
        # products in each lane.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._arrangement_is_complete():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        seen = {
            lane: sorted(observation.observed_source_object_ids)
            for lane, observation in self.ground_truth.items()
        }
        self.assertTrue(
            self._arrangement_is_complete(),
            "the ground-truth lane producer never named the arrangement the scenario stands: "
            f"expected {self._expected_ids()}, saw {seen}",
        )

    def _await_scene_authority(self, timeout_s: float = 300.0) -> None:
        # A survey is a free-space segment under the planning-scene authority gate and is refused
        # until the projector has certified a scene. The stamp is part of the condition: the gate
        # rejects a non-positive scene.header.stamp as kInvalidTimestamp, and the projector's first
        # APPLIED status carries a zero stamp because it is published before the simulated clock
        # advances.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline and not self._scene_is_certified():
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertIsNotNone(self.projection_status, "the planning-scene projector never reported")
        self.assertTrue(
            self._scene_is_certified(),
            "the planning scene was never certified with a usable stamp, so no survey could have "
            f"been commanded: state {self.projection_status.state}, error "
            f"{self.projection_status.error_code}, stamp {self.projection_status.header.stamp}",
        )

    def _scene_is_certified(self) -> bool:
        status = self.projection_status
        if status is None:
            return False
        stamped = (status.header.stamp.sec > 0) or (status.header.stamp.nanosec > 0)
        return (
            stamped
            and status.state == PlanningSceneProjectionStatus.STATE_APPLIED
            and status.error_code == PlanningSceneProjectionStatus.ERROR_NONE
        )

    def test_a_the_scenario_stood_the_columns_it_says_it_did(self):
        """Before any camera is believed, check the arrangement is the one that was asked for."""
        # Agreement with the scenario's arithmetic shows every product spawned and no two merged
        # (the failure the missing cylinder/cylinder collider produces).
        self._await_ground_truth()
        for lane in LANES:
            observation = self.ground_truth[lane]
            self.assertEqual(observation.status, LaneObservation.STATUS_OK)
            self.assertFalse(observation.obstructed, f"{lane} reports an obstruction")
            self.assertAlmostEqual(
                observation.available_depth_m,
                self.expected[lane],
                delta=SPAWN_TOLERANCE_M,
                msg=(
                    f"{lane} holds {observation.available_depth_m:.4f} m of free depth against "
                    f"the {self.expected[lane]:.4f} m the scenario computed. Either a product "
                    "failed to spawn, or two of them are at the same pose -- check whether this "
                    "tree still has cylinder/cylinder contact disabled."
                ),
            )
            # Catches a product missing mid-column, which leaves the depth number right.
            self.assertEqual(
                sorted(observation.observed_source_object_ids),
                sorted(product["source_object_id"] for product in self.products.get(lane, [])),
                f"{lane} does not contain the products the scenario stands in it",
            )
            print(
                f"{lane}: ground truth {observation.available_depth_m:.4f} m free, "
                f"{len(observation.observed_source_object_ids)} products named"
            )

    def test_b_every_lane_is_surveyed_and_agrees_with_ground_truth(self):
        """Every lane's measured depth agrees with ground truth; reported as a distribution."""
        self._await_ground_truth()
        self.assertTrue(
            self.client.wait_for_server(timeout_sec=120.0), "the survey_lane server never appeared"
        )
        self._await_scene_authority()

        errors: dict[str, float] = {}
        for lane in LANES:
            goal = SurveyLane.Goal()
            goal.lane_id = lane
            handle_future = self.client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self.node, handle_future, timeout_sec=180.0)
            handle = handle_future.result()
            self.assertIsNotNone(handle, f"the survey of {lane} was never accepted")
            self.assertTrue(handle.accepted, f"the survey of {lane} was rejected")
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=240.0)
            self.assertIsNotNone(result_future.result(), f"the survey of {lane} never returned")
            result = result_future.result().result
            self.assertEqual(
                result.outcome,
                SurveyLane.Result.OUTCOME_OBSERVED,
                f"{lane}: outcome {result.outcome}, {result.detail}",
            )
            observation = result.observation
            self.assertEqual(
                observation.status,
                LaneObservation.STATUS_OK,
                f"{lane} was refused: {observation.status_detail}",
            )
            # A false obstruction on an ordinary column would refuse work in that lane for the run.
            self.assertEqual(
                observation.obstructed,
                self.ground_truth[lane].obstructed,
                f"{lane}: the camera and the simulator disagree about obstruction",
            )
            truth = self.ground_truth[lane].available_depth_m
            error = observation.available_depth_m - truth
            errors[lane] = error
            geometries = self.columns.get(lane, [])
            _record(
                {
                    "label": RUN_LABEL,
                    "lane_id": lane,
                    "geometry_key": geometries[0] if geometries else "",
                    "count": len(geometries),
                    "ground_truth_available_depth_m": truth,
                    "measured_available_depth_m": observation.available_depth_m,
                    "available_depth_error_m": error,
                    "coverage": float(observation.confidence),
                    "observation_status": int(observation.status),
                    "aim_translation_error_m": result.achieved_translation_error_m,
                    "aim_rotation_error_rad": result.achieved_rotation_error_rad,
                }
            )
            print(
                f"{lane}: measured {observation.available_depth_m:.4f} m, "
                f"truth {truth:.4f} m, error {error * 1000.0:+.2f} mm, "
                f"coverage {observation.confidence:.3f}, "
                f"aim {result.achieved_translation_error_m * 1000.0:.2f} mm / "
                f"{result.achieved_rotation_error_rad * 1000.0:.2f} mrad"
            )

        ordered = sorted(errors.values())
        print(
            "lane depth agreement over "
            f"{len(ordered)} lanes: signed min {ordered[0] * 1000.0:+.2f} mm, median "
            f"{statistics.median(ordered) * 1000.0:+.2f} mm, max {ordered[-1] * 1000.0:+.2f} mm; "
            f"worst |error| {max(abs(value) for value in ordered) * 1000.0:.2f} mm against a "
            f"bound of {AGREEMENT_BOUND_M * 1000.0:.1f} mm"
        )
        for lane, error in errors.items():
            self.assertLess(
                abs(error),
                AGREEMENT_BOUND_M,
                f"{lane} disagreed by {error * 1000.0:+.2f} mm, beyond the "
                f"{AGREEMENT_BOUND_M * 1000.0:.1f} mm this stage asserts",
            )

    def test_d_a_lane_measured_from_the_wrong_place_is_refused_in_the_live_cell(self):
        """A camera aimed elsewhere, against the live renderer."""
        # test_b leaves the arm at lane_06's station, so measuring lane_01 without moving looks
        # across five dividers from a metre away. It must be refused, not read as empty (the
        # synthetic blindness test proves the same at message level).
        goal = SurveyLane.Goal()
        goal.lane_id = "lane_01"
        goal.measure_only = True
        handle_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, handle_future, timeout_sec=60.0)
        handle = handle_future.result()
        self.assertIsNotNone(handle, "the measure-only survey was never accepted")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=120.0)
        self.assertIsNotNone(result_future.result(), "the measure-only survey never returned")
        result = result_future.result().result
        self.assertEqual(result.outcome, SurveyLane.Result.OUTCOME_OBSERVED, result.detail)
        # Nothing was commanded, so no viewpoint is claimed.
        self.assertFalse(result.viewpoint_measured)
        observation = result.observation
        self.assertEqual(observation.status, LaneObservation.STATUS_INSUFFICIENT_COVERAGE)
        self.assertLess(observation.confidence, 0.90)
        print(
            f"lane_01 measured from lane_06's station: {observation.available_depth_m:.4f} m "
            f"free, coverage {observation.confidence:.3f}, refused"
        )

    def test_c_the_evaluator_paired_every_measurement_with_ground_truth(self):
        """The evaluator produced a usable sample for every lane."""
        self._spin(2.0)
        usable = [
            sample for sample in self.samples if sample.status == LaneDepthErrorSample.STATUS_OK
        ]
        self.assertEqual(
            sorted({sample.lane_id for sample in usable}),
            sorted(LANES),
            "the evaluator did not produce a usable sample for every lane; statuses seen: "
            f"{sorted({sample.status for sample in self.samples})}",
        )
        for sample in usable:
            # Backend name and signed error, so the population can be used to choose a floor.
            self.assertEqual(sample.backend_name, "wrist_depth_lane_survey")
            self.assertAlmostEqual(
                sample.available_depth_error_m,
                sample.measured_available_depth_m - sample.ground_truth_available_depth_m,
                places=9,
            )


@launch_testing.post_shutdown_test()
class TestLaneSurveyNodeDidNotCrash(unittest.TestCase):
    """Fail the run when `lane_survey_node` died on a fault rather than exiting."""

    def test_lane_survey_node_exited_cleanly(self, proc_info):
        """
        Assert `lane_survey_node` was not killed by SIGSEGV, SIGABRT, SIGBUS or SIGILL.

        Covers an abort during teardown with a survey goal in flight: `~ServerGoalHandle`
        publishes a cancellation for a goal that never reached a terminal state, and after the
        context is down that publish throws out of a destructor. The assertions above have
        already passed by then. Scoped to this one process.
        """
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if "lane_survey_node" in info.process_name and info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"lane_survey_node crashed: {crashed}")
