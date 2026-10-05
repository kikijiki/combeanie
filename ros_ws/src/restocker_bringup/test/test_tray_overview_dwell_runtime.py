# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 050: the focused overview-dwell loop against the acceptance tray fixture.

SC-001's repro vehicle for the zero-frame family, kept small: the acceptance scenario's three
products (the fixture Card 010's `test_d` fails on), a loop of overview-only tray surveys that
re-positions the arm between dwells, and the per-station stage receipt read off every result.

What it pins:

* every dwell of every loop admits at least one ``wrist_rgbd_tray_overview`` frame at each
  station — both slices of the acceptance tray are stocked for the whole run, so a zero-admitted
  station here is exactly the Card 050 signature (the acceptance campaign's legitimate
  empty-slice zeros cannot occur in this loop);
* the stage taps were live on the real sensor graph (images and duty detection frames counted
  inside the window), so a future zero names its stage instead of the log line staying silent;
* the survey's own motions never tip a tray product — ground truth's up-axis stays upright for
  all three products through every loop, which is the receipt that separates "the camera went
  blind" from "the product the camera must see is no longer standing" (Card 050's hypothesis
  H1). A tipped product here is a red, with the pose printed.

The full campaign repro — survey, transfer, re-survey — stays with
``test_sensor_driven_acceptance_runtime``; this test is the focused, bounded loop the card
asks for.

Ground truth is moved off the ingest topic for the whole run (the confirm duty must be the sole
publisher the world state's single-publisher pin may latch); the test itself subscribing to it
is allowed by that test's SC-002 contract, and this file only reads it.
"""

import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import SurveyTray
from restocker_interfaces.msg import (
    ObjectObservation,
    PlanningSceneProjectionStatus,
)
from restocker_interfaces.srv import GetWorldState

GT_OBJECT_TOPIC = "/perception/ground_truth/object_observations"
OVERVIEW_BACKEND = "wrist_rgbd_tray_overview"
# The acceptance fixture stands exactly one product per class, all three in the stock tray.
EXPECTED_GT_IDS = 3
# Milestone 10 section 6, Card 050: the projector seeds these from spawn until world state
# tracks them, so no transfer segment can be planned through an unconfirmed tray product.
DECLARED_SEED_IDS = {
    "restocker/declared/sim:stock_small_bottle_01",
    "restocker/declared/sim:stock_can_02",
    "restocker/declared/sim:stock_large_bottle_03",
}
# Every station of every loop must admit this many frames or more.
MINIMUM_ADMITTED_FRAMES = 1
LOOP_COUNT = 4


@pytest.mark.launch_test
def generate_test_description():
    """Survey-only composition on the Card 010 acceptance tray fixture."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "cameras": "true",
            "wrist_camera": "true",
            "ground_truth": "true",
            "scenario_config": PathJoinSubstitution(
                [FindPackageShare("restocker_gazebo"), "config", "sensor_acceptance_products.yaml"]
            ),
            # The confirm duty owns the ingest topic; ground truth keeps publishing, elsewhere.
            "object_observation_topic": GT_OBJECT_TOPIC,
            "perception": "false",
            # The survey's client role excludes the coordinator: only one motion user.
            "task_coordinator": "false",
            "attachment_adapter": "false",
            "planning_smoke": "false",
            "planning_scene_projection": "true",
            # Same reason as test_tray_survey_runtime: the wrist stations sit outside the
            # confirm duty's depth band, so a short freshness horizon would degrade the scene
            # between legs. Raised so the loop under test depends only on the survey contracts.
            "product_observation_max_age": "600.0",
            "lane_survey": "false",
            "survey_viewpoint": "true",
            "tray_survey": "true",
            "tray_overview_perception": "true",
            "tray_confirm_perception": "true",
            "controller_timeout": "60.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestOverviewDwellLoop(unittest.TestCase):
    """Drive repeated overview-only surveys and read every station's stage receipt."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node(
            "tray_overview_dwell_loop_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.gt_latest: dict[str, ObjectObservation] = {}
        cls.projection_status = None
        cls.node.create_subscription(ObjectObservation, GT_OBJECT_TOPIC, cls._on_ground_truth, 20)
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
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.get_scene_client = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.client = ActionClient(cls.node, SurveyTray, "survey_tray")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_ground_truth(cls, message: ObjectObservation) -> None:
        if message.status == ObjectObservation.STATUS_OK and message.source_object_id:
            cls.gt_latest[message.source_object_id] = message

    @classmethod
    def _on_projection_status(cls, message: PlanningSceneProjectionStatus) -> None:
        cls.projection_status = message

    @staticmethod
    def _up_z(message: ObjectObservation) -> float:
        orientation = message.pose.pose.orientation
        return 1.0 - 2.0 * (orientation.x**2 + orientation.y**2)

    def _gt_receipt(self) -> str:
        if not self.gt_latest:
            return "ground-truth tray receipt: nothing received"
        lines = ["ground-truth tray receipt (latest):"]
        for source_id in sorted(self.gt_latest):
            message = self.gt_latest[source_id]
            position = message.pose.pose.position
            lines.append(
                f"  {source_id}: xyz=({position.x:.4f}, {position.y:.4f}, {position.z:.4f}) "
                f"up_z={self._up_z(message):.3f}"
            )
        return "\n".join(lines)

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

    def _await_backends(self, timeout_s: float = 300.0) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if (
                self.client.wait_for_server(timeout_sec=0.0)
                and len(self.gt_latest) >= EXPECTED_GT_IDS
                and self._scene_is_certified()
            ):
                return
        self.fail(
            "the tray survey server, ground-truth products, or a certified planning scene never "
            f"appeared: ground truth saw {sorted(self.gt_latest)} "
            f"(expected {EXPECTED_GT_IDS} ids), projection {self.projection_status}"
        )

    def _send_overview(self, timeout_s: float = 600.0) -> SurveyTray.Result:
        goal = SurveyTray.Goal()
        goal.overview_only = True
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=60.0)
        handle = send_future.result()
        self.assertIsNotNone(handle, "the tray survey goal was never answered")
        self.assertTrue(handle.accepted, "the tray survey goal was rejected")
        result_future = handle.get_result_async()
        deadline = time.monotonic() + timeout_s
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(result_future.done(), "the tray survey never returned")
        return result_future.result().result

    def test_declared_tray_products_are_seeded_before_any_confirmation(self):
        """
        Milestone 10 section 6 (Card 050): untracked tray products are scene obstacles.

        The whole acceptance fixture is in the stock tray and this run never confirms anything,
        so every declared product must be carried by the projector's seeds: this is the scene
        state whose absence let a transfer's free-space path be planned through an unconfirmed
        product and tip it (the zero-frame family's cause).
        """
        self._await_backends()
        scene_ids = self._scene_object_ids()
        missing = DECLARED_SEED_IDS - scene_ids
        self.assertFalse(
            missing,
            "declared tray products missing from the planning scene: "
            f"{sorted(missing)}\nscene ids: {sorted(scene_ids)}",
        )
        print(
            f"declared scene seeds present: {len(DECLARED_SEED_IDS & scene_ids)} of "
            f"{len(DECLARED_SEED_IDS)}",
            flush=True,
        )

    def _scene_object_ids(self) -> set[str]:
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        deadline = time.monotonic() + 60.0
        while not self.get_scene_client.wait_for_service(timeout_sec=1.0):
            if time.monotonic() > deadline:
                self.fail("the /get_planning_scene service never appeared")
        future = self.get_scene_client.call_async(request)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
        self.assertTrue(future.done(), "/get_planning_scene did not answer")
        return {item.id for item in future.result().scene.world.collision_objects}

    def test_every_dwell_loop_admits_frames_and_names_its_stages(self):
        """Loop overview dwells; every stocked station admits and reports every stage live."""
        self._await_backends()

        for loop_index in range(LOOP_COUNT):
            result = self._send_overview()
            self.assertEqual(
                result.outcome,
                SurveyTray.Result.OUTCOME_OVERVIEW_ONLY,
                f"loop {loop_index}: outcome {result.outcome}: {result.detail}",
            )
            self.assertEqual(
                list(result.overview_stations_visited),
                ["tray_1", "tray_2"],
                f"loop {loop_index}: stations out of order: {result.overview_stations_visited}",
            )
            self.assertEqual(
                len(result.overview_station_reports),
                len(result.overview_stations_visited),
                f"loop {loop_index}: {len(result.overview_stations_visited)} station(s) "
                f"visited but {len(result.overview_station_reports)} stage report(s)",
            )
            for report in result.overview_station_reports:
                self.assertTrue(
                    report.image_tap_configured and report.detection_tap_configured,
                    f"loop {loop_index} {report.station}: stage taps not configured "
                    f"(image={report.image_tap_configured}, "
                    f"detection={report.detection_tap_configured})",
                )
                stage = (
                    f"loop {loop_index} {report.station}: images={report.images} "
                    f"detection_frames={report.detection_frames} "
                    f"frames_with_detections={report.frames_with_detections} "
                    f"published={report.published} admitted={report.admitted}"
                )
                self.assertGreater(report.images, 0, f"no images in the dwell window: {stage}")
                self.assertGreater(
                    report.detection_frames,
                    0,
                    f"the overview duty processed nothing in the dwell window: {stage}",
                )
                self.assertGreater(
                    report.frames_with_detections,
                    0,
                    f"the colour stage proposed nothing in the dwell window: {stage}",
                )
                self.assertGreater(
                    report.published,
                    0,
                    f"the duty published nothing into the dwell window: {stage}",
                )
                self.assertGreaterEqual(
                    report.admitted,
                    MINIMUM_ADMITTED_FRAMES,
                    f"zero-frame dwell (Card 050 signature): {stage}",
                )
                self.assertGreaterEqual(
                    report.published,
                    report.admitted,
                    f"admitted exceeds published — impossible: {stage}",
                )

            # Card 050's hypothesis H1 receipt: the survey's own motions must never leave a
            # tray product in a state the overview backend cannot see. A tipped product here
            # is the mechanism, with the pose printed.
            rclpy.spin_once(self.node, timeout_sec=0.0)
            for source_id, message in sorted(self.gt_latest.items()):
                up_z = self._up_z(message)
                self.assertGreater(
                    up_z,
                    0.9,
                    f"loop {loop_index}: {source_id} is no longer upright after a read-only "
                    f"survey (up_z={up_z:.3f})\n{self._gt_receipt()}",
                )
            print(
                f"dwell loop {loop_index}: every station admitted its frames; "
                f"ground truth upright on {len(self.gt_latest)} product(s)",
                flush=True,
            )


@launch_testing.post_shutdown_test()
class TestOverviewDwellLoopShutdown(unittest.TestCase):
    """Neither survey server died on a fatal signal during teardown."""

    def test_survey_nodes_exited_cleanly(self, proc_info):
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if (
                "tray_survey_node" in info.process_name
                or "survey_viewpoint_node" in info.process_name
            )
            and info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"survey node crashed: {crashed}")
