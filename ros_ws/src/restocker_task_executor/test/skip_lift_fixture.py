# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 058 skip-lift fixture shared by the skip-lift launch tests.

One campaign process per test file at the shipped `max_skip_resurveys = 2` (the budget is
monotone per run, so each file spends its own), scripted SurveyTray / RestockProduct fakes,
and a FakeWorld snapshot. The test files hold only their cases.
"""

from collections import deque
import threading
import time
import unittest

from campaign_loop_support import FakeWorld, pitch_by_sku
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct, SurveyLane, SurveyTray, SurveyViewpoint
from restocker_interfaces.msg import (
    AutonomousRestockCampaignStatus,
    ObjectObservation,
    RestockCoordinatorStatus,
    TrayStationAcquisition,
)
from restocker_interfaces.srv import GetWorldState

RESTOCK_ACTION = "/test/skip_lift/restock_product"
SHELF_ACTION = "/test/skip_lift/survey_lane"
TRAY_ACTION = "/test/skip_lift/survey_tray"
RETREAT_ACTION = "/test/skip_lift/survey_viewpoint"
STATUS_TOPIC = "/test/skip_lift/coordinator_status"
CAMPAIGN_STATUS_TOPIC = "/test/skip_lift/status"
WORLD_STATE_SERVICE = "/test/skip_lift/get_snapshot"

# The fixture renders every back-stock product at the tray centre, which is also where the
# campaign plants its skip marks and where this file scripts its overview candidates.
TRAY_POSE = (-0.4, -0.8, 0.7)
# What the server names when a matching candidate stands behind a front (Card 066, cmbrev066b N1).
FEED_BLOCK_EXAMPLE = "can at (-0.400, -0.800) stands behind can at (-0.400, -0.715)"
PLAIN_PARK = (
    "the tray survey found no candidate for any outstanding deficit class; the loop is "
    "idle until evidence changes"
)
RECEIPT_PREFIX = "skip-mark NO_CANDIDATE recovery: forced tray re-survey with skip marks"


def launch_description(overrides=None):
    """
    Run the campaign at cadence zero at the shipped default lift budget (2).

    overrides replaces or adds campaign parameters (the Card 069 file shortens
    absence_min_separation_sec so its streaks fit a test).
    """
    parameters = {
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
        "recovery_viewpoint_action_name": RETREAT_ACTION,
        "recovery_safe_station": "tray_1",
        "coordinator_status_topic": STATUS_TOPIC,
        "campaign_status_topic": CAMPAIGN_STATUS_TOPIC,
        "world_state_service_name": WORLD_STATE_SERVICE,
        # Shipped default on purpose: the file asserts "attempt n of 2" and the
        # exhausted terminal naming max_skip_resurveys=2.
        "max_skip_resurveys": 2,
        # Unbounded: PHASE_COMPLETE is not part of this file's assertions, and a
        # cycle bound mid-walk would starve the later cases of the campaign.
        "max_cycles": 0,
        "cycle_period_sec": 0.0,
        "failed_cycle_backoff_sec": 0.05,
        "server_wait_timeout_sec": 10.0,
        # Admission latency is not what this file tests; keep it generous so a
        # loaded -j2 window cannot turn a local goal handshake into a timeout.
        "action_timeout_sec": 10.0,
        # Card 086: a fresh fake world is settled; acknowledge the restart gate.
        "restart_acknowledged": True,
    }
    parameters.update(overrides or {})
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign_skip_lift_subject",
        output="screen",
        parameters=[parameters],
    )
    return launch.LaunchDescription([campaign, launch_testing.actions.ReadyToTest()])


class SkipLiftFixture(unittest.TestCase):
    """Scripted fakes around one campaign process; the test files hold only the cases."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("campaign_skip_lift_fixture")
        cls.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        cls.events = []
        cls.restock_goals = []
        cls.tray_goals = []
        cls.campaign_statuses = []
        cls.restock_script = deque()
        cls.tray_script = []
        # Whose still-counted product the next scripted parking NO_CANDIDATE drops, so a
        # cadence-zero cycle with an open deficit cannot hot-loop on a counted candidate
        # (the Card 055 fixture's drain, named per scenario).
        cls.drain_on_park = None
        cls.world = FakeWorld(pitch_by_sku())
        # Cold start with no deficit: every lane surveys once and the loop parks at
        # PHASE_FRONT_FULL, so each case below opens exactly its own deficit.
        with cls.world.lock:
            for lane in cls.world.lanes.values():
                lane["target"] = 0

        cls.shelf_server = ActionServer(
            cls.node,
            SurveyLane,
            SHELF_ACTION,
            execute_callback=cls._survey_lane,
            goal_callback=cls._shelf_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        cls.tray_server = ActionServer(
            cls.node,
            SurveyTray,
            TRAY_ACTION,
            execute_callback=cls._survey_tray,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        cls.retreat_server = ActionServer(
            cls.node,
            SurveyViewpoint,
            RETREAT_ACTION,
            execute_callback=cls._retreat,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        cls.restock_server = ActionServer(
            cls.node,
            RestockProduct,
            RESTOCK_ACTION,
            execute_callback=cls._restock,
            goal_callback=cls._restock_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        cls.world_state_service = cls.node.create_service(
            GetWorldState, WORLD_STATE_SERVICE, cls._world_state
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.status_publisher = cls.node.create_publisher(
            RestockCoordinatorStatus, STATUS_TOPIC, status_qos
        )
        cls.campaign_status_subscription = cls.node.create_subscription(
            AutonomousRestockCampaignStatus,
            CAMPAIGN_STATUS_TOPIC,
            cls.campaign_statuses.append,
            QoSProfile(
                depth=256,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls._publish_ready()
        cls._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                for s in cls.campaign_statuses
            ),
            30.0,
            "the cold-start park at PHASE_FRONT_FULL before any case opens a deficit",
        )

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=5.0)
        cls.shelf_server.destroy()
        cls.tray_server.destroy()
        cls.retreat_server.destroy()
        cls.restock_server.destroy()
        cls.node.destroy_service(cls.world_state_service)
        cls.node.destroy_subscription(cls.campaign_status_subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _publish_ready(cls):
        status = RestockCoordinatorStatus()
        status.startup_state = RestockCoordinatorStatus.STARTUP_READY
        status.admission_ready = True
        cls.status_publisher.publish(status)

    @classmethod
    def _world_state(cls, _request, response):
        response.snapshot = cls.world.snapshot()
        return response

    @classmethod
    def _shelf_goal(cls, _goal_request):
        return GoalResponse.ACCEPT

    @classmethod
    def _survey_lane(cls, goal_handle):
        cls.world.survey_lane(goal_handle.request.lane_id)
        result = SurveyLane.Result()
        result.outcome = SurveyLane.Result.OUTCOME_OBSERVED
        result.lane_id = goal_handle.request.lane_id
        goal_handle.succeed()
        return result

    @classmethod
    def _retreat(cls, goal_handle):
        result = SurveyViewpoint.Result()
        result.outcome = SurveyViewpoint.Result.OUTCOME_ARRIVED
        result.station = goal_handle.request.station
        result.execution_reached_terminal_stop = True
        goal_handle.succeed()
        return result

    @classmethod
    def _overview_candidate(cls, product_class):
        """Return a class-matching overview hypothesis at the tray centre (fixture pose)."""
        candidate = ObjectObservation()
        candidate.header.frame_id = "world"
        candidate.header.stamp = cls.node.get_clock().now().to_msg()
        candidate.product_class = product_class
        candidate.backend_name = "wrist_rgbd_tray_overview"
        candidate.status = ObjectObservation.STATUS_OK
        candidate.pose.pose.position.x = TRAY_POSE[0]
        candidate.pose.pose.position.y = TRAY_POSE[1]
        candidate.pose.pose.position.z = TRAY_POSE[2]
        candidate.pose.pose.orientation.w = 1.0
        return candidate

    @classmethod
    def _drop_drained_back(cls):
        """Remove the scenario's still-counted product, closing the cadence-zero hot loop."""
        if cls.drain_on_park:
            with cls.world.lock:
                cls.world.back = [e for e in cls.world.back if e["source"] != cls.drain_on_park]

    @classmethod
    def _survey_tray(cls, goal_handle):
        cls.tray_goals.append(goal_handle.request)
        cls.events.append(("tray",))
        # Class-aware consumption: a stale in-flight ask from the previous scenario's
        # park window carries that scenario's class and must not eat this scenario's
        # scripts — it gets the unscripted empty answer instead.
        scripted = None
        if cls.tray_script and (
            "cls" not in cls.tray_script[0]
            or cls.tray_script[0]["cls"] == goal_handle.request.product_class
        ):
            scripted = cls.tray_script.pop(0)
        if scripted is None:
            # An unscripted forced ask answers the way production does once the only
            # candidate is gone: no candidate, no overview, never liftable. Recorded under
            # a DISTINCT tag so a race cycle cannot inflate a scenario's tray counts.
            cls.events.append(("tray_exhausted",))
            result = SurveyTray.Result()
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "script exhausted: no candidate"
            goal_handle.succeed()
            return result
        result = SurveyTray.Result()
        if scripted["outcome"] == "no_candidate_seen":
            # The Card 058 R1 shape: the overview found the class-matching product, but
            # every such candidate sits on an active skip mark.
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = (
                "every overview candidate matching the request was excluded by an active skip mark"
            )
            candidate = cls._overview_candidate(scripted["cls"])
            # Card 066 (cmbrev066b N2): a candidate standing off its mark, and the merge radius
            # the server reports having applied (zero = unreported, as before).
            candidate.pose.pose.position.x += scripted.get("offset", 0.0)
            result.overview_candidates.append(candidate)
            result.candidate_merge_radius_m = scripted.get("radius", 0.0)
            # Card 066 × 058: the server reports whether each candidate is a feed-column front;
            # the lift counts only fronts. Default: the lone candidate is a front.
            front = scripted.get("front", True)
            result.overview_candidate_feed_front = [front]
            # A marked candidate that is not a front is marked stock behind a front (N1).
            result.marked_feed_blocked_candidates = scripted.get(
                "marked_feed_blocked", 0 if front else 1
            )
            if not front or scripted.get("feed_blocked", 0):
                result.feed_block_example = FEED_BLOCK_EXAMPLE
            # Matching stock seen standing behind a front (Card 066); zero by default.
            result.feed_blocked_candidates = scripted.get("feed_blocked", 0)
            if result.feed_blocked_candidates:
                result.detail = (
                    "every remaining matching candidate stands behind another tray product in "
                    "its feed column"
                )
            cls._drop_drained_back()
        elif scripted["outcome"] == "no_candidate_empty":
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "no overview candidate matched the requested selection"
            cls._drop_drained_back()
        elif scripted["outcome"] == "refuted":
            candidate = cls._overview_candidate(scripted["cls"])
            result.outcome = SurveyTray.Result.OUTCOME_REFUTED
            result.detail = "confirmation refuted the selected candidate"
            result.refutation = SurveyTray.Result.REFUTATION_ABSENT
            result.selected_candidate = candidate
        elif scripted["outcome"] == "confirmed":
            observation = ObjectObservation()
            observation.header.frame_id = "world"
            observation.header.stamp = cls.node.get_clock().now().to_msg()
            observation.source_object_id = scripted["source"]
            observation.product_class = scripted["cls"]
            observation.has_sku = True
            observation.sku = scripted["sku"]
            observation.backend_name = "wrist_rgbd_tray_confirm"
            observation.status = ObjectObservation.STATUS_OK
            observation.pose.pose.orientation.w = 1.0
            result.outcome = SurveyTray.Result.OUTCOME_CONFIRMED
            result.confirmed_observation = observation
            result.selected_candidate = observation
            cls.world.admit(scripted["source"], scripted["cls"], scripted["sku"])
        else:
            raise AssertionError(f"unknown scripted tray outcome {scripted}")
        if "stations" in scripted and result.outcome == SurveyTray.Result.OUTCOME_NO_CANDIDATE:
            # Card 080: the survey's per-station acquisition reports (Card 050's stage
            # accounting), which the park's detail must attribute. Each entry is
            # (station, (images, detection_frames, frames_with_detections, published, admitted)).
            for name, counts in scripted["stations"]:
                report = TrayStationAcquisition()
                report.station = name
                report.image_tap_configured = True
                report.detection_tap_configured = True
                (
                    report.images,
                    report.detection_frames,
                    report.frames_with_detections,
                    report.published,
                    report.admitted,
                ) = counts
                report.drained = report.admitted == 0
                result.overview_station_reports.append(report)
        if "probe" in scripted:
            # Card 069: one scripted absence verdict for every probe the goal carried.
            verdict = getattr(SurveyTray.Result, "PROBE_" + scripted["probe"].upper())
            result.absence_probe_verdicts = [verdict] * len(
                goal_handle.request.absence_probe_positions
            )
        if "then_targets" in scripted:
            # Lane targets change atomically with this answer, so the campaign's next
            # measurement already sees them (Card 069: a deficit closing between the rung's
            # charge and the lifted survey).
            with cls.world.lock:
                for lane_id, target in scripted["then_targets"].items():
                    cls.world.lanes[lane_id]["target"] = target
        goal_handle.succeed()
        return result

    @classmethod
    def _restock_goal(cls, _goal_request):
        if not cls.restock_script:
            # Never raise inside a callback: an escaped exception kills the fixture's spin
            # thread and turns every later goal into an admission timeout. Reject loudly
            # instead — the scenario's own assertions then report the misalignment.
            cls.events.append(("transfer-unscripted",))
            return GoalResponse.REJECT
        cls.events.append(("transfer",))
        return GoalResponse.ACCEPT

    @classmethod
    def _restock(cls, goal_handle):
        cls.restock_goals.append(goal_handle.request)
        if not cls.restock_script:
            cls.events.append(("transfer-empty-script",))
            result = RestockProduct.Result()
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            result.detail = "fixture: transfer arrived with an empty script"
            goal_handle.abort()
            return result
        scripted = cls.restock_script.popleft()
        if scripted == "skip":
            # Card 051 rung 5: the typed recoverable skip, naming the product it was
            # transferring so the campaign charges max_product_skips against the right one.
            result = RestockProduct.Result()
            result.status = RestockProduct.Result.STATUS_SKIPPED_RECOVERABLE
            result.detail = "recoverable skip (rung 5): scripted test skip"
            if goal_handle.request.has_object_id:
                result.selected_object_id = goal_handle.request.object_id
            elif cls.world.back:
                result.selected_object_id = cls.world.back[0]["id"]
            else:
                result.selected_object_id = 7
            goal_handle.abort()
            return result
        if not isinstance(scripted, dict):
            raise AssertionError(f"unscripted restock behavior {scripted!r}")
        cls.world.apply_transfer(scripted["lane"], scripted["source"])
        result = RestockProduct.Result()
        result.status = RestockProduct.Result.STATUS_SUCCEEDED
        goal_handle.succeed()
        return result

    # -- helpers -----------------------------------------------------------

    @classmethod
    def _spin_until(cls, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            cls._publish_ready()
            time.sleep(0.05)
            if predicate():
                return
        phases = [(s.phase, s.detail) for s in cls.campaign_statuses]
        raise AssertionError(
            f"timed out waiting for {description}; events={cls.events} phases={phases}"
        )

    @classmethod
    def _receipts(cls):
        return [s for s in cls.campaign_statuses if RECEIPT_PREFIX in s.detail]

    def lane_identity(self, lane_id):
        """Return the (class, sku) the fixture declares for one lane."""
        lane = self.world.lanes[lane_id]
        return lane["cls"], lane["sku"]

    def _drops_back(self, source):
        with self.world.lock:
            self.world.back = [e for e in self.world.back if e["source"] != source]
