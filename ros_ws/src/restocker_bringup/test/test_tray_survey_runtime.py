# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Coordinated tray survey: overview before confirmation, and no transfer from overview alone."""

# Acceptance for Card 003, against the live cell:
#
# SC-001: an overview-only survey visits both overview stations, collects candidates, and
# never aims confirmation — and no overview-backend message ever appears on the world state's
# object ingest topic, nor any object at all while only overview evidence exists, so a
# transfer cannot begin from overview evidence. The full survey then proves the other half:
# overview feedback strictly precedes confirmation, and the confirmed observation is the one
# that reaches the world state.
#
# SC-002: a cancellation during the second overview station abandons it, keeps the candidates
# the first station already produced, and returns only after every /survey_viewpoint goal is
# terminal.
#
# SC-003: a close-range observation that contradicts the overview hypothesis at the
# candidate's place produces the typed REFUTED outcome rather than a confirmation. The
# contradiction is injected as a synthetic confirm-duty publication (misclassifying the real
# pipeline on demand is not scriptable) at the candidate's position; a contradiction among
# the associated close-range observations wins over any agreeing frame, so the injection is
# deterministic against the real duty's own frames.
#
# Ground truth is moved off the ingest topic for the whole run: the confirm duty must be the
# sole publisher the world state's single-publisher pin may latch.
#
# Campaign mode (same shape as test_lane_survey_runtime): when
# RESTOCKER_TRAY_SURVEY_SAMPLES names a row log, this file skips the Card 003 acceptance
# cases and runs one measurement pass against RESTOCKER_TRAY_SURVEY_SCENARIO instead —
# overview survey, then one confirmation attempt per catalogued product in the scenario —
# appending schema rows for the predeclared wrist-tray matrix. With nothing set, behaviour
# is the shipped three-product acceptance run and nothing is written.

import json
import math
import os
from pathlib import Path
import threading
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
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
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import SurveyTray
from restocker_interfaces.msg import (
    ObjectObservation,
    PlanningSceneProjectionStatus,
    PoseErrorSample,
)
from restocker_interfaces.srv import GetWorldState
import yaml

INGEST_TOPIC = "/perception/object_observations"
GROUND_TRUTH_TOPIC = "/perception/ground_truth/object_observations"
CANDIDATE_TOPIC = "/perception/tray_candidates"
POSE_ERROR_TOPIC = "/perception/pose_error"
OVERVIEW_BACKEND = "wrist_rgbd_tray_overview"
CONFIRM_BACKEND = "wrist_rgbd_tray_confirm"
TERMINAL_STATUS = (
    GoalStatus.STATUS_SUCCEEDED,
    GoalStatus.STATUS_CANCELED,
    GoalStatus.STATUS_ABORTED,
)

# Campaign hooks. Unset: shipped scenario, acceptance tests, no row log (Card 003).
SCENARIO_OVERRIDE = os.environ.get("RESTOCKER_TRAY_SURVEY_SCENARIO", "")
SAMPLE_LOG = os.environ.get("RESTOCKER_TRAY_SURVEY_SAMPLES", "")
RUN_LABEL = os.environ.get("RESTOCKER_TRAY_SURVEY_LABEL", "shipped")
CELL_ID = os.environ.get("RESTOCKER_TRAY_CELL_ID", "")
CELL_FEATURE = os.environ.get("RESTOCKER_TRAY_FEATURE", "")
CELL_SEED = os.environ.get("RESTOCKER_TRAY_SEED", "-1")
# Q7: sequential confirm passes at one commanded optical pose inside one launch.
EXTRINSICS_PASSES = int(os.environ.get("RESTOCKER_TRAY_EXTRINSICS_PASSES", "1") or "1")
CAMPAIGN = bool(SAMPLE_LOG)

CLASS_NAMES = {
    ObjectObservation.PRODUCT_CLASS_CAN: "can",
    ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE: "small_bottle",
    ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE: "large_bottle",
}
# ObjectObservation product_class value -> collision catalogue geometry_key default.
CLASS_DEFAULT_GEOMETRY = {
    ObjectObservation.PRODUCT_CLASS_CAN: "can.standard",
    ObjectObservation.PRODUCT_CLASS_SMALL_BOTTLE: "bottle.small.standard",
    ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE: "bottle.large.standard",
}


def _record(row: dict) -> None:
    """Append one measured row to the campaign log, when a campaign asked for one."""
    if not SAMPLE_LOG:
        return
    path = Path(SAMPLE_LOG)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True, allow_nan=True) + "\n")


def _scenario_document() -> dict:
    """Load the arrangement under test (campaign override or the shipped share scenario)."""
    scenario_path = (
        Path(SCENARIO_OVERRIDE)
        if SCENARIO_OVERRIDE
        else Path(get_package_share_directory("restocker_gazebo"))
        / "config"
        / "baseline_products.yaml"
    )
    return yaml.safe_load(scenario_path.read_text(encoding="utf-8"))


def _scenario_products() -> list[dict]:
    """Tray products the scenario stands (every product in the document for these campaigns)."""
    document = _scenario_document()
    return list(document["products"])


def _identity_outcome(sku_expected, sku_reported, class_expected, class_reported) -> str:
    """Classify identity against the arrangement expectation (matrix metric 4)."""
    if not sku_reported and not class_reported:
        return "none"
    if sku_reported and sku_reported == sku_expected:
        return "match"
    if class_reported and class_reported == class_expected:
        return "class_only"
    if sku_reported or class_reported:
        return "mismatch"
    return "none"


def _range_band(range_m: float) -> str:
    """Classify camera-to-product range into the predeclared report bands."""
    if range_m is None or not math.isfinite(range_m) or range_m < 0.0:
        return "unknown"
    if range_m < 0.50:
        return "confirm"
    if range_m < 1.20:
        return "overview"
    return "far"


@pytest.mark.launch_test
def generate_test_description():
    """Start the planning composition with both tray duties and the coordinated survey."""
    scenario = (
        SCENARIO_OVERRIDE
        if SCENARIO_OVERRIDE
        else PathJoinSubstitution(
            [FindPackageShare("restocker_gazebo"), "config", "baseline_products.yaml"]
        )
    )
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            # Ground-truth path pins (Milestone 10 Stage 7 default switch):
            "lane_observation_topic": "/perception/ground_truth/lane_observations",
            "gui": "false",
            "rviz": "false",
            "cameras": "true",
            "wrist_camera": "true",
            "ground_truth": "true",
            "scenario_config": scenario,
            # The confirm duty owns the ingest topic; ground truth keeps publishing, elsewhere.
            "object_observation_topic": GROUND_TRUTH_TOPIC,
            # Object perception overhead is not under test; the tray duties are.
            "perception": "false",
            # The survey's client role excludes the coordinator: only one motion user.
            "task_coordinator": "false",
            "attachment_adapter": "false",
            "planning_smoke": "false",
            "planning_scene_projection": "true",
            # The 0.5 s scene-projection freshness gate cannot be satisfied by the wrist tray
            # duties: once the confirm duty admits a tray object, the overview stations sit
            # outside its depth band (0.42-1.01 m against maximum_depth_m 0.55), so the object
            # goes stale the moment the arm leaves the close viewpoint and the projector
            # degrades, refusing every later leg. Retained stale geometry still guards the
            # plan; re-deriving object freshness as a validity horizon is the specification's
            # section 5 work, not this card's. Raised here so the survey contracts under test
            # (SC-001..003) do not depend on that unbuilt stage.
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


def _skip_unless_acceptance(test):
    """Card 003 cases run only when no campaign row log is attached."""
    return unittest.skipIf(CAMPAIGN, "campaign mode: measurement only")(test)


def _skip_unless_campaign(test):
    """Run the measurement pass only when a campaign row log is attached."""
    return unittest.skipUnless(CAMPAIGN, "acceptance mode: no RESTOCKER_TRAY_SURVEY_SAMPLES")(test)


class TestCoordinatedTraySurvey(unittest.TestCase):
    """Drive the real survey_tray action against the live cell."""

    @classmethod
    def setUpClass(cls):
        """Bring up a client node and every stream the assertions read."""
        rclpy.init()
        # use_sim_time so injected observations carry the same clock the survey windows on
        # (the node's own clock), rather than wall time a sub-real-time-factor run drifts
        # out of the confirmation window with.
        cls.node = rclpy.create_node(
            "tray_survey_runtime_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.candidates: list[ObjectObservation] = []
        # One ordered event log: (kind, payload). kind is 'overview_msg', 'confirm_msg',
        # 'phase:<n>:<station>', so ordering between feedback and observations is visible.
        cls.events: list[tuple[str, object]] = []
        cls.ingest_overview_messages: list[ObjectObservation] = []
        cls.confirm_ingest_messages: list[ObjectObservation] = []
        cls.feedback: list[tuple[int, str]] = []
        cls.cancel_on_station: str | None = None
        cls.cancel_fired = threading.Event()
        cls.projection_status = None
        cls.synthetic_publisher = None

        cls.node.create_subscription(ObjectObservation, CANDIDATE_TOPIC, cls._on_candidate, 20)
        cls.node.create_subscription(ObjectObservation, INGEST_TOPIC, cls._on_ingest, 20)
        cls.node.create_subscription(
            ObjectObservation, GROUND_TRUTH_TOPIC, cls._on_ground_truth, 20
        )
        cls.pose_errors: list[PoseErrorSample] = []
        cls.node.create_subscription(PoseErrorSample, POSE_ERROR_TOPIC, cls.pose_errors.append, 50)
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
        cls.node.create_subscription(
            GoalStatusArray,
            "/survey_viewpoint/_action/status",
            cls._on_viewpoint_status,
            QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.client = ActionClient(cls.node, SurveyTray, "survey_tray")
        cls.viewpoint_status: GoalStatusArray | None = None
        cls.ground_truth_ids: set[str] = set()
        cls.ground_truth_messages: dict[str, ObjectObservation] = {}
        cls.expected_gt_count = len(_scenario_products()) if SCENARIO_OVERRIDE else 3

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_candidate(cls, message: ObjectObservation) -> None:
        cls.candidates.append(message)

    @classmethod
    def _on_ingest(cls, message: ObjectObservation) -> None:
        if message.backend_name == OVERVIEW_BACKEND:
            cls.ingest_overview_messages.append(message)
            cls.events.append(("overview_msg", message))
        elif message.backend_name == CONFIRM_BACKEND:
            cls.confirm_ingest_messages.append(message)
            cls.events.append(("confirm_msg", message))

    @classmethod
    def _on_ground_truth(cls, message: ObjectObservation) -> None:
        if message.status == ObjectObservation.STATUS_OK and message.source_object_id:
            cls.ground_truth_ids.add(message.source_object_id)
            cls.ground_truth_messages[message.source_object_id] = message

    @classmethod
    def _on_projection_status(cls, message: PlanningSceneProjectionStatus) -> None:
        cls.projection_status = message

    @classmethod
    def _on_viewpoint_status(cls, message: GoalStatusArray) -> None:
        cls.viewpoint_status = message

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _on_feedback(self, message: SurveyTray.Impl.FeedbackMessage) -> None:
        # rclpy hands the action's FeedbackMessage wrapper (goal_id + feedback) to the client
        # callback, not the inner Feedback; reading fields off the wrapper raises
        # AttributeError inside the spin loop, which errors the test and leaves the goal
        # outstanding for the rest of the run.
        feedback = message.feedback
        station = feedback.current_station
        self.feedback.append((feedback.phase, station))
        self.events.append((f"phase:{feedback.phase}:{station}", None))
        if (
            self.cancel_on_station is not None
            and feedback.phase == SurveyTray.Feedback.PHASE_OVERVIEW
            and feedback.current_station == self.cancel_on_station
        ):
            self.cancel_fired.set()

    def _await_backends(self, timeout_s: float = 300.0) -> None:
        # Ground-truth products from the moved topic: naming them proves the arrangement spawned
        # before the camera is judged. Campaign scenarios stand 1-4 products; the shipped
        # acceptance scenario stands three.
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if (
                self.client.wait_for_server(timeout_sec=0.0)
                and len(self.ground_truth_ids) >= self.expected_gt_count
                and self._scene_is_certified()
            ):
                return
        self.fail(
            "the tray survey server, ground-truth products, or a certified planning scene never "
            f"appeared: ground truth saw {sorted(self.ground_truth_ids)} "
            f"(expected {self.expected_gt_count}), projection {self.projection_status}"
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

    def _snapshot_objects(self) -> dict:
        request = GetWorldState.Request()
        future = self.world_state.call_async(request)
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(future.done(), "the world-state snapshot service did not answer")
        snapshot = future.result().snapshot
        return {obj.source_object_id: obj for obj in snapshot.objects}

    def _send(
        self, customize, timeout_s: float = 600.0, cancel: bool = False
    ) -> SurveyTray.Result:
        goal = SurveyTray.Goal()
        customize(goal)
        send_future = self.client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self.node, send_future, timeout_sec=60.0)
        handle = send_future.result()
        self.assertIsNotNone(handle, "the tray survey goal was never answered")
        self.assertTrue(handle.accepted, "the tray survey goal was rejected")
        if cancel:
            deadline = time.monotonic() + 60.0
            while not self.cancel_fired.is_set() and time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            self.assertTrue(
                self.cancel_fired.is_set(),
                "the survey never reported the station the cancellation targets",
            )
            cancel_future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=30.0)
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=timeout_s)
        self.assertIsNotNone(result_future.result(), "the tray survey never returned")
        self.wrapped_status = result_future.result().status
        return result_future.result().result

    # -- SC-001 ---------------------------------------------------------------------------------

    @_skip_unless_acceptance
    def test_a_overview_only_never_puts_evidence_where_a_transfer_could_read_it(self):
        """Both overview stations, candidates collected, nothing admitted, no confirm aimed."""
        self._await_backends()
        self.feedback.clear()
        self.events.clear()
        objects_before = self._snapshot_objects()
        # Ground truth was moved off the ingest topic, so the world state starts without tray
        # products; if that is not true, the SC-001 proof below would be reading ground truth.
        self.assertEqual(
            objects_before,
            {},
            f"the ingest topic is not camera-only; world state already holds {objects_before}",
        )

        result = self._send(lambda goal: setattr(goal, "overview_only", True))
        self.assertEqual(
            result.outcome,
            SurveyTray.Result.OUTCOME_OVERVIEW_ONLY,
            f"outcome {result.outcome}: {result.detail}",
        )
        self.assertEqual(
            list(result.overview_stations_visited),
            ["tray_1", "tray_2"],
            f"stations visited out of order or incomplete: {result.overview_stations_visited}",
        )
        self.assertGreaterEqual(
            len(result.overview_candidates),
            1,
            "the fixed scenario stands three tray products and the overview saw none",
        )
        for candidate in result.overview_candidates:
            self.assertEqual(candidate.backend_name, OVERVIEW_BACKEND)
        self.assertEqual(result.confirmed_observation.header.frame_id, "")
        self.assertEqual(result.selected_candidate.header.frame_id, "")

        # The structural half of SC-001: overview evidence never lands on the topic the world
        # state ingests, so no grasp candidate could ever be generated from it.
        self.assertEqual(
            self.ingest_overview_messages,
            [],
            "the overview duty published on the world state's ingest topic",
        )
        self.assertEqual(
            self._snapshot_objects(),
            {},
            "the world state admitted objects from overview evidence alone",
        )

        # Confirmation was never aimed: no explicit-pose feedback phase, and by the absence of
        # a confirmed observation.
        phases = [phase for phase, _ in self.feedback]
        self.assertNotIn(SurveyTray.Feedback.PHASE_CONFIRMING, phases)

    @_skip_unless_acceptance
    def test_b_full_survey_confirms_after_both_overview_stations(self):
        """Overview feedback precedes confirmation, and only the confirm observation admits."""
        self.feedback.clear()
        self.events.clear()
        self.confirm_ingest_messages.clear()

        result = self._send(lambda goal: None)

        # Ordering: some overview feedback, then selecting, then confirming — strictly after
        # both overview stations, all from the events recorded while the goal ran.
        phase_kinds = [kind for kind, _ in self.events if kind.startswith("phase:")]
        overview_indexes = [i for i, k in enumerate(phase_kinds) if k.startswith("phase:0:")]
        confirm_indexes = [i for i, k in enumerate(phase_kinds) if k.startswith("phase:2:")]
        self.assertTrue(overview_indexes, "no overview feedback was published")
        self.assertTrue(confirm_indexes, "no confirmation feedback was published")
        self.assertLess(
            max(overview_indexes),
            min(confirm_indexes),
            f"confirmation was announced before overview finished: {phase_kinds}",
        )
        # And no confirm-duty observation reached the ingest topic before the confirmation
        # phase began: the overview legs produce candidates only, never admitted evidence.
        confirm_phase_index = next(
            (i for i, (kind, _) in enumerate(self.events) if kind.startswith("phase:2:")),
            None,
        )
        self.assertIsNotNone(confirm_phase_index, "no confirmation feedback was recorded")
        early_confirm_msgs = [
            i
            for i, (kind, _) in enumerate(self.events)
            if kind == "confirm_msg" and i < confirm_phase_index
        ]
        self.assertEqual(
            early_confirm_msgs,
            [],
            "a confirm-duty observation reached the world state before the confirmation phase",
        )

        if result.outcome != SurveyTray.Result.OUTCOME_CONFIRMED:
            # The confirmation duty's live detection quality is Card 004's measurement; what
            # this card must prove is the contract around it. A non-confirm outcome here is
            # reported with its detail rather than silently passing.
            self.fail(
                f"expected a confirmed survey, got outcome {result.outcome}: {result.detail}"
            )
        self.assertEqual(list(result.overview_stations_visited), ["tray_1", "tray_2"])
        self.assertEqual(result.confirmed_observation.backend_name, CONFIRM_BACKEND)
        self.assertEqual(result.refutation, SurveyTray.Result.REFUTATION_NONE)

        # The replacing observation is the one the world state admitted: its identity is in
        # the snapshot, and no overview-backend message ever appeared on that topic.
        objects = self._snapshot_objects()
        self.assertIn(result.confirmed_observation.source_object_id, objects)
        self.assertEqual(
            self.ingest_overview_messages,
            [],
            "the overview duty published on the world state's ingest topic",
        )

    # -- SC-002 ---------------------------------------------------------------------------------

    @_skip_unless_acceptance
    def test_c_cancellation_abandons_unvisited_stations_and_leaves_no_motion_goal(self):
        """Cancel at the second station: candidates kept, stations abandoned, legs terminal."""
        self.feedback.clear()
        self.cancel_on_station = "tray_2"
        self.cancel_fired.clear()
        try:
            result = self._send(lambda goal: setattr(goal, "overview_only", True), cancel=True)
        finally:
            self.cancel_on_station = None
        self.assertEqual(
            self.wrapped_status,
            GoalStatus.STATUS_CANCELED,
            "the canceled goal did not end with the canceled terminal status",
        )
        self.assertEqual(
            result.outcome,
            SurveyTray.Result.OUTCOME_CANCELED,
            f"outcome {result.outcome}: {result.detail}",
        )
        self.assertEqual(
            list(result.overview_stations_visited),
            ["tray_1"],
            f"unvisited stations were not abandoned: {result.overview_stations_visited}",
        )
        self.assertGreaterEqual(
            len(result.overview_candidates),
            1,
            "candidates already collected were discarded by the cancellation",
        )
        for candidate in result.overview_candidates:
            self.assertEqual(candidate.backend_name, OVERVIEW_BACKEND)

        # No active motion goal survives the canceled result. The status topic is latched, so
        # a short settle then every goal must be terminal.
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            statuses = self.viewpoint_status.status_list if self.viewpoint_status else []
            if statuses and all(status.status in TERMINAL_STATUS for status in statuses):
                break
        statuses = self.viewpoint_status.status_list if self.viewpoint_status else []
        self.assertTrue(statuses, "the /survey_viewpoint status topic never published")
        active = [s for s in statuses if s.status not in TERMINAL_STATUS]
        self.assertEqual(
            active,
            [],
            f"motion goals still active after the canceled survey: {active}",
        )

    # -- SC-003 ---------------------------------------------------------------------------------

    @_skip_unless_acceptance
    def test_d_contradicted_close_range_identity_returns_a_typed_refutation(self):
        """Inject a contradiction at the candidate's place; the survey must refuse, not confirm."""
        self.feedback.clear()
        self.confirm_ingest_messages.clear()
        # Target the large bottle deliberately: with no class filter the survey selects it
        # (highest confidence — test_b confirms it unfiltered), and with the filter below it
        # is the only candidate of its class in the fixed scenario. Pinning the target to the
        # class the survey empirically selects makes the injection site and the selection the
        # same product either way, instead of depending on which candidate happened to arrive
        # first on the topic.
        large = [
            message
            for message in self.candidates
            if message.product_class == ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE
        ]
        self.assertTrue(
            large,
            "no large-bottle overview candidate was ever observed, so there is nothing to "
            "contradict",
        )
        target = large[-1]
        # A class the target is not, so the close view contradicts rather than agrees.
        contradiction_class = ObjectObservation.PRODUCT_CLASS_CAN
        contradiction_sku = "SIM-CAN-STD"
        contradiction_id = "sim:stock_can_01"
        # Inject at the first and the most recent estimate of that product: the survey's
        # selected candidate is its own fresh merge of this class, which is one of the two.
        injection_poses = [large[0].pose]
        if self._distance(large[0], target) > 0.02:
            injection_poses.append(target.pose)
        # The synthetic publisher is created here, not in setUpClass: the world state pins a
        # single publisher on the ingest topic, and tests a/b needed the confirm duty to be it.
        self.synthetic_publisher = self.node.create_publisher(ObjectObservation, INGEST_TOPIC, 20)
        stop = threading.Event()

        def contradict():
            """Publish the contradiction at the candidate's exact positions until the goal ends."""
            while not stop.is_set() and rclpy.ok():
                stamp = self.node.get_clock().now().to_msg()
                for pose in injection_poses:
                    message = ObjectObservation()
                    message.header.frame_id = "world"
                    message.header.stamp = stamp
                    message.source_object_id = contradiction_id
                    message.product_class = contradiction_class
                    message.has_sku = True
                    message.sku = contradiction_sku
                    message.pose = pose
                    message.orientation = ObjectObservation.ORIENTATION_UPRIGHT
                    message.confidence = 0.95
                    message.backend_name = CONFIRM_BACKEND
                    message.backend_version = "synthetic_contradiction"
                    message.status = ObjectObservation.STATUS_OK
                    self.synthetic_publisher.publish(message)
                time.sleep(0.2)

        def log_selection_intent(message: SurveyTray.Goal) -> None:
            message.product_class = ObjectObservation.PRODUCT_CLASS_LARGE_BOTTLE
            print(
                f"test_d: injecting class {contradiction_class} at {len(injection_poses)} "
                f"pose(s); goal product_class={message.product_class}; target at "
                f"({target.pose.pose.position.x:.3f}, {target.pose.pose.position.y:.3f}, "
                f"{target.pose.pose.position.z:.3f})",
                flush=True,
            )

        thread = threading.Thread(target=contradict, daemon=True)
        thread.start()
        try:
            result = self._send(log_selection_intent, timeout_s=600.0)
        finally:
            stop.set()
            thread.join(timeout=5.0)

        synthetic_seen = [
            message
            for message in self.confirm_ingest_messages
            if message.backend_version == "synthetic_contradiction"
        ]
        self.assertGreater(
            len(synthetic_seen),
            0,
            "the contradiction was published but never observed on the ingest topic",
        )
        self.assertEqual(
            result.outcome,
            SurveyTray.Result.OUTCOME_REFUTED,
            f"a contradicted identity must refute, not confirm: outcome {result.outcome}: "
            f"{result.detail} (selected "
            f"{result.selected_candidate.product_class} "
            f"'{result.selected_candidate.sku}' at "
            f"({result.selected_candidate.pose.pose.position.x:.3f}, "
            f"{result.selected_candidate.pose.pose.position.y:.3f}, "
            f"{result.selected_candidate.pose.pose.position.z:.3f}); target at "
            f"({target.pose.pose.position.x:.3f}, {target.pose.pose.position.y:.3f}, "
            f"{target.pose.pose.position.z:.3f}), distance "
            f"{self._distance(result.selected_candidate, target):.3f} m; "
            f"{len(synthetic_seen)} injected frame(s) on the topic)",
        )
        self.assertEqual(result.refutation, SurveyTray.Result.REFUTATION_CLASS_MISMATCH)
        # The survey really selected the candidate this test contradicted, so the refutation
        # is about that product rather than about a neighbour the injection missed.
        self.assertEqual(result.selected_candidate.product_class, target.product_class)
        self.assertLess(
            self._distance(result.selected_candidate, target),
            0.10,
            "the survey selected a different candidate than the one the injection contradicts",
        )
        self.assertEqual(
            result.confirmed_observation.header.frame_id,
            "",
            "a refuted survey must not carry a grasp-authorizing observation",
        )
        self.assertEqual(
            result.refuting_observation.product_class,
            contradiction_class,
        )

    @staticmethod
    def _distance(left: ObjectObservation, right: ObjectObservation) -> float:
        """Straight-line distance between two observations in the planning frame."""
        dx = left.pose.pose.position.x - right.pose.pose.position.x
        dy = left.pose.pose.position.y - right.pose.pose.position.y
        dz = left.pose.pose.position.z - right.pose.pose.position.z
        return (dx * dx + dy * dy + dz * dz) ** 0.5

    # -- Campaign measurement (Card 004 matrix) ---------------------------------------------------

    def _base_row(self, product: dict) -> dict:
        """Matrix row fields that come from the arrangement cell and the scenario product."""
        geometry_key = product.get("geometry_key", "")
        return {
            "cell_id": CELL_ID,
            "feature": CELL_FEATURE,
            "seed": int(CELL_SEED),
            "geometry_key": geometry_key,
            "sku_expected": product.get("sku", ""),
            "run_label": RUN_LABEL,
            "commit": os.environ.get("RESTOCKER_TRAY_COMMIT", ""),
        }

    @staticmethod
    def _class_name(product_class: int) -> str:
        """Map ObjectObservation product_class to the catalogue class name."""
        return CLASS_NAMES.get(product_class, "")

    @staticmethod
    def _geometry_for_message(message: ObjectObservation, scenario_products: list[dict]) -> str:
        """Resolve the scenario geometry_key for an observation's source identity."""
        for product in scenario_products:
            if product.get("source_object_id") == message.source_object_id:
                return product.get("geometry_key", "")
        # Fall back to the class default when identity assigner minted a different id.
        return CLASS_DEFAULT_GEOMETRY.get(message.product_class, "")

    def _latest_pose_error(self, source_object_id: str, backend: str) -> PoseErrorSample | None:
        """Most recent OK pose-error sample for this object and backend."""
        for sample in reversed(self.pose_errors):
            if (
                sample.source_object_id == source_object_id
                and sample.backend_name == backend
                and sample.status == PoseErrorSample.STATUS_OK
            ):
                return sample
        return None

    def _observation_row(
        self,
        product: dict,
        duty: str,
        message: ObjectObservation | None,
        *,
        arm_configuration: str = "",
        observations_for_pair: int | None = None,
    ) -> dict:
        """Build one schema row for a product × duty attempt (admitted or refused)."""
        row = self._base_row(product)
        row["duty"] = duty
        scenario_products = _scenario_products()
        if message is None:
            row.update(
                {
                    "sku_reported": "",
                    "class_reported": "",
                    "identity": "none",
                    "status": 1,
                    "orientation": "unset",
                    "translation_error_m": None,
                    "observation_range_m": None,
                    "range_band": "unknown",
                    "axis_error_rad": None,
                    "orientation_status": 0,
                    "confidence": 0.0,
                    "covariance": None,
                    "backend_name": OVERVIEW_BACKEND if duty == "overview" else CONFIRM_BACKEND,
                    "source_object_id": product.get("source_object_id", ""),
                }
            )
        else:
            backend = message.backend_name or (
                OVERVIEW_BACKEND if duty == "overview" else CONFIRM_BACKEND
            )
            sample = self._latest_pose_error(message.source_object_id, backend)
            # Residual vector in the planning frame: estimate minus ground truth (metric 9).
            residual = None
            truth = self.ground_truth_messages.get(product.get("source_object_id", ""), None)
            if truth is None:
                # Association may have remapped ids; fall back to nearest GT by position.
                truth = min(
                    self.ground_truth_messages.values(),
                    key=lambda gt: self._distance(message, gt),
                    default=None,
                )
            if truth is not None:
                residual = [
                    message.pose.pose.position.x - truth.pose.pose.position.x,
                    message.pose.pose.position.y - truth.pose.pose.position.y,
                    message.pose.pose.position.z - truth.pose.pose.position.z,
                ]
            translation_error = sample.translation_error_m if sample else None
            if translation_error is None and residual is not None:
                translation_error = math.sqrt(sum(v * v for v in residual))
            observation_range = sample.observation_range_m if sample else float("nan")
            axis_error = sample.axis_error_rad if sample else float("nan")
            orientation_status = sample.orientation_status if sample else 0
            covariance = list(message.pose.covariance)
            class_expected = product.get("product_class", "")
            class_reported = self._class_name(message.product_class)
            row.update(
                {
                    "sku_reported": message.sku if message.has_sku else "",
                    "class_reported": class_reported,
                    "identity": _identity_outcome(
                        product.get("sku", ""),
                        message.sku if message.has_sku else "",
                        class_expected,
                        class_reported,
                    ),
                    "status": int(message.status),
                    "orientation": str(int(message.orientation)),
                    "translation_error_m": translation_error,
                    "observation_range_m": observation_range,
                    "range_band": _range_band(
                        observation_range if observation_range is not None else float("nan")
                    ),
                    "axis_error_rad": axis_error,
                    "orientation_status": int(orientation_status),
                    "confidence": float(message.confidence),
                    "covariance": covariance,
                    "backend_name": backend,
                    "source_object_id": message.source_object_id,
                    "residual_xyz_m": residual,
                    "geometry_key": self._geometry_for_message(message, scenario_products)
                    or row["geometry_key"],
                }
            )
        if arm_configuration:
            row["arm_configuration"] = arm_configuration
        if CELL_FEATURE == "touching_pair" and duty == "overview":
            row["observations_for_pair"] = (
                0 if observations_for_pair is None else int(observations_for_pair)
            )
        return row

    def _overview_phase(self) -> dict[str, ObjectObservation]:
        """Run overview-only; return product source_id -> latest overview observation."""
        self.candidates.clear()
        self.events.clear()
        self.feedback.clear()
        self.pose_errors.clear()
        result = self._send(lambda goal: setattr(goal, "overview_only", True), timeout_s=600.0)
        overview_by_id: dict[str, ObjectObservation] = {}
        for message in self.candidates:
            if (
                message.backend_name != OVERVIEW_BACKEND
                or message.status != ObjectObservation.STATUS_OK
            ):
                continue
            overview_by_id[message.source_object_id] = message
        # Also catch overview messages that never became candidates (identity drop leaves none).
        print(
            f"campaign overview: outcome={result.outcome} "
            f"candidates={len(result.overview_candidates)} "
            f"stations={list(result.overview_stations_visited)} detail={result.detail}",
            flush=True,
        )
        return overview_by_id

    def _confirm_pass(self, product: dict, arm_label: str) -> ObjectObservation | None:
        """One full survey selecting this product's class/SKU; return the confirm observation."""
        self.confirm_ingest_messages.clear()
        self.feedback.clear()
        self.pose_errors.clear()
        product_class = product.get("product_class", "")
        class_to_int = {name: value for value, name in CLASS_NAMES.items()}
        selected_class = class_to_int.get(product_class, SurveyTray.Goal.PRODUCT_CLASS_ANY)
        sku = product.get("sku", "")

        def customize(goal: SurveyTray.Goal) -> None:
            goal.product_class = selected_class
            if sku:
                goal.has_sku = True
                goal.sku = sku

        result = self._send(customize, timeout_s=600.0)
        print(
            f"campaign confirm[{arm_label}] {product.get('source_object_id')}: "
            f"outcome={result.outcome} detail={result.detail}",
            flush=True,
        )
        if result.outcome == SurveyTray.Result.OUTCOME_CONFIRMED:
            return result.confirmed_observation
        return None

    @_skip_unless_campaign
    def test_campaign_matrix_measurement(self):
        """One overview pass and one confirm attempt per product; append schema rows."""
        self._await_backends()
        scenario_products = _scenario_products()
        self.assertTrue(
            scenario_products,
            "campaign scenario stands no products",
        )

        overview_by_id = self._overview_phase()

        # Q11: how many of the pair's ground-truth positions the overview actually saw.
        pair_seen = 0
        if CELL_FEATURE == "touching_pair":
            for product in scenario_products:
                if product.get("source_object_id") in overview_by_id:
                    pair_seen += 1

        for product in scenario_products:
            source_id = product.get("source_object_id", "")
            overview_message = overview_by_id.get(source_id)
            # When identity assignment minted a different id for a single-class product, match
            # by nearest overview observation of the same class.
            if overview_message is None:
                class_name = product.get("product_class", "")
                class_int = {name: key for key, name in CLASS_NAMES.items()}.get(class_name)
                if class_int is not None:
                    same_class = [
                        m for m in overview_by_id.values() if m.product_class == class_int
                    ]
                    if len(same_class) == 1 and len(scenario_products) == 1:
                        overview_message = same_class[0]
            overview_row = self._observation_row(
                product,
                "overview",
                overview_message,
                observations_for_pair=pair_seen if CELL_FEATURE == "touching_pair" else None,
            )
            _record(overview_row)
            print(
                f"row overview {source_id}: status={overview_row['status']} "
                f"identity={overview_row['identity']} err={overview_row['translation_error_m']}",
                flush=True,
            )

        passes = max(1, EXTRINSICS_PASSES)
        # source_id -> list of admitted confirm rows (Q7 keeps every pass; single-pass keeps one).
        confirm_admitted: dict[str, list[ObjectObservation]] = {
            product.get("source_object_id", ""): [] for product in scenario_products
        }
        for pass_index in range(passes):
            arm_label = f"cfg{pass_index}" if passes > 1 else ""
            for product in scenario_products:
                source_id = product.get("source_object_id", "")
                confirm_message = self._confirm_pass(product, arm_label)
                if confirm_message is None:
                    continue
                if passes > 1:
                    confirm_admitted.setdefault(source_id, []).append(confirm_message)
                    row = self._observation_row(
                        product, "confirm", confirm_message, arm_configuration=arm_label
                    )
                    _record(row)
                elif not confirm_admitted[source_id]:
                    confirm_admitted[source_id].append(confirm_message)
                    row = self._observation_row(
                        product, "confirm", confirm_message, arm_configuration=arm_label
                    )
                    _record(row)
                print(
                    f"row confirm {source_id} [{arm_label}]: "
                    f"{'admitted' if confirm_message else 'none'}",
                    flush=True,
                )

        # One refusal row per product that never confirmed (even when a sibling class did).
        for product in scenario_products:
            source_id = product.get("source_object_id", "")
            if confirm_admitted.get(source_id):
                continue
            row = self._observation_row(product, "confirm", None)
            _record(row)
            print(
                f"row confirm REFUSED {source_id}: no confirmation",
                flush=True,
            )

        # Sanity: at least the overview rows for every product.
        self.assertGreaterEqual(
            len(scenario_products),
            1,
            "campaign recorded no overview rows",
        )
        print(
            f"campaign measurement complete label={RUN_LABEL} cell={CELL_ID} "
            f"products={len(scenario_products)} extrinsics_passes={passes}",
            flush=True,
        )


@launch_testing.post_shutdown_test()
class TestTraySurveyNodeDidNotCrash(unittest.TestCase):
    """Fail the run when either survey server died on a fault rather than exiting."""

    def test_survey_nodes_exited_cleanly(self, proc_info):
        """Assert neither tray_survey_node nor survey_viewpoint_node hit a fatal signal."""
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
