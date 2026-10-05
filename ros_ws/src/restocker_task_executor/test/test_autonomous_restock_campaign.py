# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Deterministic mode-loop acceptance for the autonomous sensor-driven campaign (Card 009).

Every action boundary is a scripted fake; no simulator is involved. One campaign process walks
the spec's transition graph in order: a cold-start shelf survey over admitted back stock, a
partial validity-driven re-survey, a deficit with no valid tray candidate (SURVEY_TRAY and
CONFIRM), a refutation that reselects with the mark, an admitted-candidate transfer that never
touches the tray after a blocked retry, the NO_CANDIDATE idle, a cancelled shelf survey that
recovers, and the terminal cycle bound. SC-003 is the ordering itself: no transfer goal exists
before the surveys and confirmations that authorize it.
"""

from collections import deque
import threading
import time
import unittest

from campaign_loop_support import FakeWorld, fixture_io_callback_group, pitch_by_sku
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct, SurveyLane, SurveyTray, SurveyViewpoint
from restocker_interfaces.msg import (
    AutonomousRestockCampaignStatus,
    ObjectObservation,
    RestockCoordinatorStatus,
    ShelfLane,
)
from restocker_interfaces.srv import GetWorldState

RESTOCK_ACTION = "/test/autonomous_campaign/restock_product"
SHELF_ACTION = "/test/autonomous_campaign/survey_lane"
TRAY_ACTION = "/test/autonomous_campaign/survey_tray"
RETREAT_ACTION = "/test/autonomous_campaign/survey_viewpoint"
STATUS_TOPIC = "/test/autonomous_campaign/coordinator_status"
CAMPAIGN_STATUS_TOPIC = "/test/autonomous_campaign/status"
WORLD_STATE_SERVICE = "/test/autonomous_campaign/get_snapshot"


@pytest.mark.launch_test
def generate_test_description():
    """Start the campaign process; the test provides every action and the world snapshot."""
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign_test_subject",
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
                "recovery_viewpoint_action_name": RETREAT_ACTION,
                "recovery_safe_station": "tray_1",
                "max_lane_survey_attempts_per_cycle": 2,
                # One shared per-run survey-skip budget (Milestone 10 §6, Card 052). The
                # shipped default is 2; the walk raises it to 4 so one run can exercise
                # lane charging (S16), station skips plus the all-skipped non-block
                # backoff (S19 cycle 1), and the rung-5 exhaustion latch (S19 cycle 2) —
                # the latch mechanism is the same at any bound.
                "max_survey_skips": 4,
                "coordinator_status_topic": STATUS_TOPIC,
                "campaign_status_topic": CAMPAIGN_STATUS_TOPIC,
                "world_state_service_name": WORLD_STATE_SERVICE,
                # Card 052's S15-S16, S19b and S21 add bounded failing cycles; 170 keeps
                # the closing PHASE_COMPLETE wait reachable without racing the bound
                # mid-scenario.
                "max_cycles": 170,
                "cycle_period_sec": 0.0,
                "failed_cycle_backoff_sec": 0.05,
                "server_wait_timeout_sec": 10.0,
                "action_timeout_sec": 3.0,
                # Card 086: a fresh fake world is settled; acknowledge the restart gate.
                "restart_acknowledged": True,
            }
        ],
    )
    return launch.LaunchDescription([campaign, launch_testing.actions.ReadyToTest()])


class TestSensorDrivenModeLoop(unittest.TestCase):
    """Walk the whole transition graph against scripted action boundaries."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("autonomous_campaign_mode_loop_fixture")
        cls.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        cls.events = []
        cls.restock_goals = []
        cls.tray_goals = []
        cls.retreat_goals = []
        cls.retreat_goal_times = []
        cls.campaign_status_times = []
        cls.retreat_admission_hang_sec = 0.0
        cls.admitted_ids = {}
        cls.campaign_statuses = []
        cls.shelf_script = {}
        cls.tray_script = []
        cls.restock_script = deque()
        cls.world = FakeWorld(pitch_by_sku())

        cls.shelf_server = ActionServer(
            cls.node,
            SurveyLane,
            SHELF_ACTION,
            execute_callback=cls._survey_lane,
            goal_callback=cls._shelf_goal,
            # rclpy's default cancel callback rejects every cancel; the campaign's timeout
            # path cancels, and a fixture that refused would hang until its own deadline.
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        cls.tray_server = ActionServer(
            cls.node,
            SurveyTray,
            TRAY_ACTION,
            execute_callback=cls._survey_tray,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        # S21 deliberately stalls retreat admission. This endpoint represents a separate
        # production server, so its delay must not hold unrelated action admissions or
        # the status observer behind the same executor lock. Keep retreat callbacks
        # serialized with each other, and retain the group for the server's lifetime.
        cls.retreat_admission_group = MutuallyExclusiveCallbackGroup()
        cls.retreat_server = ActionServer(
            cls.node,
            SurveyViewpoint,
            RETREAT_ACTION,
            execute_callback=cls._retreat,
            goal_callback=cls._retreat_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
            callback_group=cls.retreat_admission_group,
        )
        cls.restock_server = ActionServer(
            cls.node,
            RestockProduct,
            RESTOCK_ACTION,
            execute_callback=cls._restock,
            goal_callback=cls._restock_goal,
            cancel_callback=lambda _goal_handle: CancelResponse.ACCEPT,
        )
        # Card 075: snapshots remain independently responsive on the reentrant I/O group.
        # Status stays on the node default group (Card 077, 075 review note 2 — if reentrant,
        # two queued statuses can be appended by two executor threads out of arrival order, which
        # every window assertion below assumes cannot happen), and TRANSIENT_LOCAL depth 64
        # preserves the receipt history. Call-site tests pin all three callback groups below.
        cls.fixture_io_group = fixture_io_callback_group()
        cls.world_state_service = cls.node.create_service(
            GetWorldState,
            WORLD_STATE_SERVICE,
            cls._world_state,
            callback_group=cls.fixture_io_group,
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
            cls._on_campaign_status,
            QoSProfile(
                depth=64,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls._publish_ready()

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
    def _on_campaign_status(cls, message):
        cls.campaign_status_times.append(time.monotonic())
        cls.campaign_statuses.append(message)

    @classmethod
    def _publish_ready(cls):
        status = RestockCoordinatorStatus()
        status.startup_state = RestockCoordinatorStatus.STARTUP_READY
        status.admission_ready = True
        cls.status_publisher.publish(status)

    @classmethod
    def _shelf_goal(cls, goal_request):
        cls.events.append(("shelf", goal_request.lane_id))
        script = cls.shelf_script.get(goal_request.lane_id)
        if script and script[0] == "reject":
            script.popleft()
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @classmethod
    def _survey_lane(cls, goal_handle):
        lane_id = goal_handle.request.lane_id
        script = cls.shelf_script.get(lane_id)
        behavior = script.popleft() if script else "ok"
        if behavior == "fail_plan":
            # Milestone 10 §6, Card 052: a recoverable lane-survey failure whose own
            # machinery proves no joint command was issued — the ladder must climb instead
            # of blocking (the classification's arm half is established by the flag).
            result = SurveyLane.Result()
            result.outcome = SurveyLane.Result.OUTCOME_VIEWPOINT_UNREACHED
            result.lane_id = lane_id
            result.detail = "scripted: planning failed, nothing was commanded"
            result.motion_definitely_not_started = True
            result.execution_reached_terminal_stop = False
            goal_handle.succeed()
            return result
        if behavior == "fail_exec":
            # A recoverable failure whose execution reached a terminal stop: the arm may
            # have moved, so rung 1 must retreat to the safe survey pose first.
            result = SurveyLane.Result()
            result.outcome = SurveyLane.Result.OUTCOME_VIEWPOINT_UNREACHED
            result.lane_id = lane_id
            result.detail = "scripted: execution failed at a terminal stop"
            result.motion_definitely_not_started = False
            result.execution_reached_terminal_stop = True
            goal_handle.succeed()
            return result
        if behavior == "hang":
            # The test node spins on a multi-threaded executor, so the cancel that answers
            # the campaign's timeout is observed while this one waits on the goal's own flag.
            deadline = time.monotonic() + 15.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.02)
            if goal_handle.is_cancel_requested:
                cls.events.append(("shelf_cancel", lane_id))
                result = SurveyLane.Result()
                result.outcome = SurveyLane.Result.OUTCOME_CANCELED
                result.lane_id = lane_id
                # Card 086: the production lane server reports the stop its cancel established.
                # A canceled terminal without it would (correctly) leave the attempt unresolved.
                result.execution_reached_terminal_stop = True
                goal_handle.canceled()
                return result
            goal_handle.abort()
            return SurveyLane.Result()
        cls.world.survey_lane(lane_id)
        result = SurveyLane.Result()
        result.outcome = SurveyLane.Result.OUTCOME_OBSERVED
        result.lane_id = lane_id
        goal_handle.succeed()
        return result

    @classmethod
    def _retreat_goal(cls, _goal_request):
        # Review F1: delay the admission response past action_timeout_sec when scripted,
        # so the campaign's admission wait fails while the goal IS in flight — the race
        # the fail-closed fix must not paper over with "nothing was commanded".
        hang = cls.retreat_admission_hang_sec
        if hang > 0:
            time.sleep(hang)
        return GoalResponse.ACCEPT

    @classmethod
    def _retreat(cls, goal_handle):
        # Milestone 10 §6 rung 1 (Card 052): the campaign's cleanup retreat to the safe
        # survey station. Arrives with the controllers' terminal-stop evidence.
        cls.retreat_goals.append({"station": goal_handle.request.station})
        cls.retreat_goal_times.append(time.monotonic())
        result = SurveyViewpoint.Result()
        result.outcome = SurveyViewpoint.Result.OUTCOME_ARRIVED
        result.station = goal_handle.request.station
        result.execution_reached_terminal_stop = True
        goal_handle.succeed()
        return result

    @classmethod
    def _survey_tray(cls, goal_handle):
        goal = goal_handle.request
        refuted = tuple((point.x, point.y, point.z) for point in goal.refuted_positions)
        skip = tuple((point.x, point.y, point.z) for point in goal.skip_mark_positions)
        if not cls.tray_script:
            # A forced tray ask whose script is exhausted answers the way production does
            # once the only candidate is marked gone: no candidate. The loop then falls
            # through the classes and parks stock-exhausted. Raising here instead made a
            # settle-cycle race (Card 051's persistent skip mark forces the tray path while
            # a deficit is still open) cost a blocked cycle and desynchronise every later
            # scenario. Recorded under a DISTINCT event tag and never in tray_goals: the
            # per-scenario exact "tray" counts and tray_goals windows assert on scripted
            # asks only, so a post-latch race cycle (backoff 0.05 s vs the fixture's
            # ~0.25 s poll granularity) cannot inflate them (review-F1 follow-up, S20's
            # 9th goal). A genuinely unexpected scripted-slot ask still lands as "tray".
            cls.events.append(("tray_exhausted", goal.product_class, refuted, skip))
            result = SurveyTray.Result()
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "script exhausted: no candidate"
            goal_handle.succeed()
            return result
        cls.tray_goals.append(
            {
                "product_class": goal.product_class,
                "refuted": refuted,
                "skip": skip,
                "overview_stations": list(goal.overview_stations),
            }
        )
        cls.events.append(("tray", goal.product_class, refuted, skip))
        scripted = cls.tray_script.pop(0)
        if scripted.get("expire_lane"):
            # The world moved while the arm was away at the tray: that lane's evidence is
            # now older than its validity horizon when the transfer boundary re-checks.
            cls.world.expire_lane(scripted["expire_lane"])
        feedback = SurveyTray.Feedback()
        feedback.phase = SurveyTray.Feedback.PHASE_OVERVIEW
        feedback.current_station = "tray_1"
        goal_handle.publish_feedback(feedback)
        if not scripted.get("skip_confirm_feedback"):
            # By default feedback and result race on separate DDS entities. Withholding
            # feedback gives deterministic coverage of the result fallback. Request lifetime
            # and callback/worker ordering are covered by test_campaign_confirm_report.
            feedback = SurveyTray.Feedback()
            feedback.phase = SurveyTray.Feedback.PHASE_CONFIRMING
            goal_handle.publish_feedback(feedback)

        result = SurveyTray.Result()
        outcome = scripted["outcome"]
        if outcome == "confirmed":
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
            if scripted.get("drop_admission"):
                # Card 037: the confirming observation never reaches the world state, so the
                # campaign cannot resolve a pin at the transfer boundary and must block
                # instead of falling back to an unpinned transfer goal.
                pass
            else:
                cls.admitted_ids[scripted["source"]] = cls.world.admit(
                    scripted["source"], scripted["cls"], scripted["sku"]
                )
        elif outcome == "refuted":
            candidate = ObjectObservation()
            candidate.header.frame_id = "world"
            candidate.pose.pose.position.x = scripted["x"]
            candidate.pose.pose.position.y = scripted["y"]
            candidate.pose.pose.position.z = scripted["z"]
            candidate.product_class = scripted.get("cls", ShelfLane.PRODUCT_CLASS_CAN)
            result.outcome = SurveyTray.Result.OUTCOME_REFUTED
            result.refutation = SurveyTray.Result.REFUTATION_CLASS_MISMATCH
            result.selected_candidate = candidate
            result.detail = "scripted refutation"
        elif outcome == "no_candidate":
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "scripted: no candidate"
        elif outcome == "unavailable":
            result.outcome = SurveyTray.Result.OUTCOME_UNAVAILABLE
            result.detail = "scripted: tray survey unavailable"
            # Card 086: an UNAVAILABLE tray terminal never settles (a child leg may be unadmitted,
            # review B1), so the walk no longer scripts it; test_campaign_unresolved_* cover it.
        elif outcome == "unreachable_recoverable":
            # Milestone 10 §6, Card 052: a recoverable completed failure — the evidence
            # flags (and the failed station the ladder rotates or skips) come from the
            # script, so a fixture can pin the RECOVERABLE branch exactly; without the
            # flags the same outcome classifies UNSAFE and blocks, as S6 proves.
            result.outcome = SurveyTray.Result.OUTCOME_VIEWPOINT_UNREACHED
            result.detail = scripted.get(
                "detail", "scripted: overview leg planning failed before any command"
            )
            result.motion_definitely_not_started = scripted.get("not_started", True)
            result.execution_reached_terminal_stop = scripted.get("terminal_stop", False)
            result.failed_station = scripted.get("failed_station", "")
        else:
            raise AssertionError(f"unknown scripted tray outcome {outcome}")
        goal_handle.succeed()
        return result

    @classmethod
    def _restock_goal(cls, _goal_request):
        if not cls.restock_script:
            raise AssertionError("the campaign sent an unscripted transfer")
        if cls.restock_script[0] == "reject":
            cls.restock_script.popleft()
            cls.events.append(("transfer", "reject"))
            cls.restock_goals.append(None)
            return GoalResponse.REJECT
        cls.events.append(("transfer", "accepted"))
        return GoalResponse.ACCEPT

    @classmethod
    def _restock(cls, goal_handle):
        goal = goal_handle.request
        cls.restock_goals.append(goal)
        if not cls.restock_script:
            raise AssertionError("a transfer goal arrived with an empty script")
        scripted = cls.restock_script.popleft()
        if scripted == "no_pair":
            result = RestockProduct.Result()
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            goal_handle.abort()
            return result
        if scripted == "skip":
            # Card 051 rung 5: the typed recoverable skip, naming the product it was
            # transferring so the campaign charges max_product_skips against the right one.
            result = RestockProduct.Result()
            result.status = RestockProduct.Result.STATUS_SKIPPED_RECOVERABLE
            result.detail = "recoverable skip (rung 5): scripted test skip"
            if goal.has_object_id:
                result.selected_object_id = goal.object_id
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

    @classmethod
    def _world_state(cls, _request, response):
        response.snapshot = cls.world.snapshot()
        return response

    # -- helpers -----------------------------------------------------------

    def _spin_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._publish_ready()
            time.sleep(0.05)
            if predicate():
                return
        phases = [(s.phase, s.detail) for s in self.campaign_statuses]
        self.fail(f"timed out waiting for {description}; events={self.events} phases={phases}")

    def _wait_for_front_full(self, transfers, timeout=30.0):
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers == transfers
                for s in self.campaign_statuses
            ),
            timeout,
            f"PHASE_FRONT_FULL after {transfers} transfers",
        )

    @staticmethod
    def _count_events(events, kind):
        return sum(1 for event in events if event[0] == kind)

    def lane_identity(self, lane_id):
        """Return the (class, sku) the fixture declares for one lane."""
        lane = self.world.lanes[lane_id]
        return lane["cls"], lane["sku"]

    def _statuses_from(self, mark):
        return self.campaign_statuses[mark:]

    # -- the fixture's own callback-group wiring (Card 077) -----------------

    def test_world_state_service_is_on_the_reentrant_fixture_group(self):
        """
        Card 075 review note 1: pin the CALL SITE, not just the fixture helper.

        `test_fixture_world_state_callback_group` proves `fixture_io_callback_group()` answers
        while a `goal_callback` sleeps, but it wires its own service. This asserts the campaign
        fixture actually passes that group to the snapshot service, so dropping
        `callback_group=` above fails here instead of only under S21's load-sensitive window.
        """
        service_group = self.world_state_service.callback_group
        self.assertIsInstance(service_group, ReentrantCallbackGroup)
        self.assertIsNot(
            service_group,
            self.node.default_callback_group,
            "snapshot progress must remain independent of the serialized scenario callbacks",
        )

    def test_campaign_status_subscription_stays_on_the_node_default_group(self):
        """
        Card 075 review note 2: the status subscription must stay mutually exclusive.

        Reentrancy would be a hazard here: the subscription calls `cls.campaign_statuses.append`,
        so on a Reentrant group two statuses queued at once can be appended by two
        executor threads out of arrival order — rclpy's handler takes a message, re-arms the
        entity and only then runs the callback, and a Reentrant group's
        `beginning_execution` always grants the next dispatch — while every window assertion
        in this file — `modes.index(...)`, `campaign_statuses[-1]`, the sequence marks —
        assumes arrival order. The default group serializes them with each other and with the
        other action servers; retreat admission has its own exclusive group, so its deliberate
        delay cannot stall this observer. TRANSIENT_LOCAL depth 64 retains the receipt history.
        """
        subscription_group = self.campaign_status_subscription.callback_group
        self.assertIs(subscription_group, self.node.default_callback_group)
        self.assertIsInstance(subscription_group, MutuallyExclusiveCallbackGroup)

    def test_retreat_admission_has_its_own_mutually_exclusive_group(self):
        """Keep S21's admission delay local to the retreat endpoint."""
        retreat_group = self.retreat_server.callback_group
        self.assertIs(retreat_group, self.retreat_admission_group)
        self.assertIsInstance(retreat_group, MutuallyExclusiveCallbackGroup)
        for entity in (
            self.campaign_status_subscription,
            self.world_state_service,
            self.shelf_server,
            self.tray_server,
            self.restock_server,
        ):
            self.assertIsNot(retreat_group, entity.callback_group)
        self.assertIsNot(retreat_group, self.node.default_callback_group)

    # -- the walk ----------------------------------------------------------

    def test_mode_loop_walks_every_transition(self):
        """Walk the scripted scenarios covering every transition, refutation, and block."""
        world = self.world
        # S1: cold start surveys every lane before the first transfer, drains admitted back
        # stock over the deficit, and idles at front full without ever touching the tray.
        with world.lock:
            for index, lane in enumerate(world.lanes.values(), start=1):
                world.add_back(f"sim:stock_{index}", lane["cls"], lane["sku"])
        for index in range(1, 7):
            self.restock_script.append(
                {"lane": f"lane_{index:02d}", "source": f"sim:stock_{index}"}
            )
        self._wait_for_front_full(6)
        self.assertEqual(
            self.events,
            [("shelf", f"lane_{lane:02d}") for lane in range(1, 7)]
            + [("transfer", "accepted")] * 6,
            "cold start: six shelf surveys, six transfers, and no tray survey",
        )
        measured = next(
            s
            for s in self.campaign_statuses
            if s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
        )
        self.assertTrue(measured.front_survey_complete)
        self.assertTrue(
            measured.back_survey_complete,
            "admitted back stock means no tray survey was required",
        )
        self.assertEqual(measured.total_front_deficit, 6)
        self.assertEqual(measured.total_back_stock, 6)
        for goal in self.restock_goals:
            self.assertFalse(goal.has_object_id)
            self.assertFalse(goal.has_lane_id)
        modes = [s.mode for s in self.campaign_statuses]
        self.assertIn(AutonomousRestockCampaignStatus.MODE_SURVEY_SHELF, modes)
        self.assertIn(AutonomousRestockCampaignStatus.MODE_TRANSFER, modes)
        self.assertNotIn(AutonomousRestockCampaignStatus.MODE_SURVEY_TRAY, modes)
        self.assertLess(
            modes.index(AutonomousRestockCampaignStatus.MODE_SURVEY_SHELF),
            modes.index(AutonomousRestockCampaignStatus.MODE_TRANSFER),
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_BACK,
            [s.phase for s in self.campaign_statuses],
            "a deficit with valid admitted candidates must not survey the tray",
        )
        self.assertGreater(measured.survey_arm_time_sec, 0.0)
        last = self.campaign_statuses[-1]
        self.assertGreaterEqual(last.transfer_arm_time_sec, 0.0)

        # S2: only the invalidated and the expired lane are re-surveyed; the other four
        # stand on their still-valid evidence, and no tray survey happens.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        product_class, sku = self.lane_identity("lane_03")
        other_class, other_sku = self.lane_identity("lane_05")
        with world.lock:
            world.lanes["lane_03"]["target"] = 2
            world.lanes["lane_03"]["invalid"] = True
            world.lanes["lane_05"]["target"] = 2
            world.lanes["lane_05"]["verified_age"] = 120.0
            world.add_back("sim:large_s2", product_class, sku)
            world.add_back("sim:small_s2", other_class, other_sku)
        self.restock_script.extend(
            [
                {"lane": "lane_03", "source": "sim:large_s2"},
                {"lane": "lane_05", "source": "sim:small_s2"},
            ]
        )
        self._wait_for_front_full(8)
        self.assertEqual(
            self.events[mark:],
            [
                ("shelf", "lane_03"),
                ("shelf", "lane_05"),
                ("transfer", "accepted"),
                ("transfer", "accepted"),
            ],
            "only invalid or expired lanes are surveyed before the next transfers",
        )
        phases = [s.phase for s in self._statuses_from(status_mark)]
        self.assertNotIn(AutonomousRestockCampaignStatus.PHASE_SURVEYING_BACK, phases)

        # S3: a stale tray observation is not a valid candidate; the deficit forces
        # SURVEY_TRAY, CONFIRM must confirm, and only then may the transfer go out.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        product_class, sku = self.lane_identity("lane_01")
        with world.lock:
            world.lanes["lane_01"]["target"] = 2
            world.lanes["lane_01"]["invalid"] = True
            world.add_back("sim:stale_can", product_class, sku, observed_age=300.0)
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:tray_can_s3",
                "cls": product_class,
                "sku": sku,
                # Card 079: withhold the confirm feedback so this goal deterministically
                # takes the result path — the ordering that made the flake.
                "skip_confirm_feedback": True,
            }
        )
        self.restock_script.append({"lane": "lane_01", "source": "sim:tray_can_s3"})
        self._wait_for_front_full(9)
        events = self.events[mark:]
        self.assertEqual(
            events,
            [
                ("shelf", "lane_01"),
                ("tray", ShelfLane.PRODUCT_CLASS_CAN, (), ()),
                ("transfer", "accepted"),
            ],
            "stale tray evidence never replaces a survey, and the transfer follows the "
            "confirmation",
        )
        with world.lock:
            self.assertIn(
                "sim:stale_can",
                [entry["source"] for entry in world.back],
                "the stale candidate was never eligible to transfer",
            )
        # Card 079: this goal withholds its confirm feedback, so the confirm status is
        # emitted from the result instead — wait for it by condition (never sleep on it),
        # then make exactly the assertions the feedback path has always had to satisfy.
        self._spin_until(
            lambda: any(
                s.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
                for s in self._statuses_from(status_mark)
            ),
            10.0,
            "the MODE_CONFIRM status of the tray goal that withheld its confirm feedback",
        )
        modes = [s.mode for s in self._statuses_from(status_mark)]
        self.assertIn(AutonomousRestockCampaignStatus.MODE_SURVEY_TRAY, modes)
        self.assertIn(AutonomousRestockCampaignStatus.MODE_CONFIRM, modes)
        self.assertEqual(
            modes.count(AutonomousRestockCampaignStatus.MODE_CONFIRM),
            1,
            "Card 079: exactly one confirm status per goal — the withheld feedback's "
            "result path must not double with anything else",
        )
        self.assertLess(
            modes.index(AutonomousRestockCampaignStatus.MODE_SURVEY_TRAY),
            modes.index(AutonomousRestockCampaignStatus.MODE_CONFIRM),
        )
        self.assertLess(
            modes.index(AutonomousRestockCampaignStatus.MODE_CONFIRM),
            modes.index(AutonomousRestockCampaignStatus.MODE_TRANSFER),
        )
        # The CONFIRM status is published from an executor thread (the action feedback) or
        # the worker thread (Card 079's result path); either way it must carry a locked
        # snapshot of the worker's measurement and the atomic front-survey flag, not a racy
        # half-updated read.
        confirm_status = next(
            s
            for s in self._statuses_from(status_mark)
            if s.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
        )
        self.assertTrue(
            confirm_status.front_survey_complete,
            "the CONFIRM status must see the worker's completed shelf survey",
        )
        self.assertGreater(
            confirm_status.total_front_deficit,
            0,
            "the CONFIRM status must carry the measurement snapshot the worker stored",
        )
        self.assertEqual(self.tray_goals[-1]["product_class"], product_class)

        # Card 037 / Milestone 10 §1 confirmed-identity contract: the confirm-path transfer
        # goal names exactly the product the close view confirmed — object selector set,
        # lane selector unset (the destination stays the coordinator's) — and the transfer
        # status says so. The skip-path goals asserted above stay empty-selector.
        confirm_goal = self.restock_goals[-1]
        self.assertTrue(confirm_goal.has_object_id)
        self.assertFalse(confirm_goal.has_lane_id)
        self.assertEqual(confirm_goal.object_id, self.admitted_ids["sim:tray_can_s3"])
        transfer_statuses = [
            s
            for s in self._statuses_from(status_mark)
            if s.mode == AutonomousRestockCampaignStatus.MODE_TRANSFER
        ]
        self.assertTrue(
            any("close-confirmed candidate" in s.detail for s in transfer_statuses),
            "the confirm-path transfer status must name the pinned candidate",
        )

        # S4: a refuted candidate returns to SURVEY_TRAY marked, and the re-issue carries
        # the mark so the next candidate can be confirmed instead.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        product_class, sku = self.lane_identity("lane_04")
        refuted_at = (-0.42, -0.8, 0.7)
        with world.lock:
            world.lanes["lane_04"]["target"] = 2
            world.lanes["lane_04"]["invalid"] = True
        self.tray_script.extend(
            [
                {
                    "outcome": "refuted",
                    "x": refuted_at[0],
                    "y": refuted_at[1],
                    "z": refuted_at[2],
                    "cls": product_class,
                },
                {
                    "outcome": "confirmed",
                    "source": "sim:tray_can_s4",
                    "cls": product_class,
                    "sku": sku,
                },
            ]
        )
        self.restock_script.append({"lane": "lane_04", "source": "sim:tray_can_s4"})
        self._wait_for_front_full(10)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "tray", "tray", "transfer"],
            "refutation re-enters SURVEY_TRAY before any transfer",
        )
        self.assertEqual(self.tray_goals[-1]["refuted"], (refuted_at,))
        # Both goals send feedback and a confirming result. Either DDS order must report
        # once per goal. The focused unit test separately forces both possible orders.
        confirms = [
            status
            for status in self._statuses_from(status_mark)
            if status.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
        ]
        self.assertEqual(
            len(confirms),
            2,
            "exactly one confirm status per goal that sent feedback and a result "
            f"({[status.detail for status in confirms]})",
        )

        # S5: a rejected transfer blocks, and the retry rides the admitted candidate
        # without surveying the tray again. A STATUS_NO_COMPATIBLE_PAIR on this same skip
        # path is Card 055's recoverable boundary (it now forces a tray re-survey instead
        # of blocking) and is covered by test_campaign_ncp_recovery; here only the
        # reject → blocked → unpinned-retry shape is asserted.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        product_class, sku = self.lane_identity("lane_02")
        with world.lock:
            world.lanes["lane_02"]["target"] = 2
            world.lanes["lane_02"]["invalid"] = True
            world.add_back("sim:admitted_small", product_class, sku)
        self.restock_script.extend(
            [
                "reject",
                {"lane": "lane_02", "source": "sim:admitted_small"},
            ]
        )
        self._wait_for_front_full(11)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "transfer", "transfer"],
            "the retry after a blocked transfer never re-surveys a valid tray candidate",
        )
        phases = [s.phase for s in self._statuses_from(status_mark)]
        self.assertIn(AutonomousRestockCampaignStatus.PHASE_BLOCKED, phases)
        self.assertEqual(
            self._count_events(events, "tray"),
            0,
            "SC-003: admitted candidates transfer without a tray survey",
        )
        retry_goal = [goal for goal in self.restock_goals if goal is not None][-1]
        self.assertFalse(retry_goal.has_object_id)
        self.assertFalse(retry_goal.has_lane_id)

        # S6: a deficit with no valid candidate surveys the tray; a recoverable failure retries, a
        # typed NO_CANDIDATE idles the loop, and the idle does not spin the tray.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        with world.lock:
            world.lanes["lane_06"]["target"] = 2
            world.lanes["lane_06"]["invalid"] = True
        self.tray_script.extend(
            [
                # Card 079: both goals withhold their confirm feedback (the deterministic
                # shape of the client's drop); neither reached the confirm phase, so
                # neither may report one afterwards.
                {
                    "outcome": "unreachable_recoverable",
                    "detail": "scripted: overview leg planning failed before any command",
                    "skip_confirm_feedback": True,
                },
                {"outcome": "no_candidate", "skip_confirm_feedback": True},
            ]
        )
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self._statuses_from(status_mark)
            ),
            20.0,
            "PHASE_STOCK_EXHAUSTED after the tray reported no candidate",
        )
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "tray", "tray"],
            "a recoverable pre-motion tray failure is retried once, then NO_CANDIDATE idles the "
            "loop",
        )
        # Card 086: the evidence-bearing failure settles, so the ladder retries inside the cycle
        # without blocking and nothing is marked unresolved. The old flagless UNAVAILABLE
        # (which blocked once and then self-healed) is unresolved and covered elsewhere.
        self.assertFalse(
            any(s.motion_unresolved for s in self._statuses_from(status_mark)),
            "a settled recoverable failure never claims unresolved motion",
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.MODE_CONFIRM,
            [s.mode for s in self._statuses_from(status_mark)],
            "Card 079: neither goal reached the confirm phase, so the fail-safe must not "
            "invent a MODE_CONFIRM status for them",
        )
        idle_modes = [
            s.mode
            for s in self._statuses_from(status_mark)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
        ]
        self.assertTrue(idle_modes)
        self.assertEqual(idle_modes[-1], AutonomousRestockCampaignStatus.MODE_IDLE)
        quiet_deadline = time.monotonic() + 0.4
        settled = len(self.events)
        while time.monotonic() < quiet_deadline:
            self._publish_ready()
            time.sleep(0.05)
        self.assertEqual(
            len(self.events),
            settled,
            "an idle loop must not keep surveying the tray while lanes stay valid",
        )

        # S7: the expired lane is surveyed again; the first attempt hangs and is
        # cancelled at the action timeout, the retry recovers, and the transfer closes
        # the loop back through the admitted candidate. The cancel path must bill the
        # in-flight hang to survey_arm_time_sec — action_timeout_sec is 3.0, so a
        # charged cancel moves the counter by seconds while the fake's success path
        # costs microseconds.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        survey_arm_before_cancel = self.campaign_statuses[-1].survey_arm_time_sec
        product_class, sku = self.lane_identity("lane_06")
        with world.lock:
            world.lanes["lane_06"]["verified_age"] = 120.0
            world.add_back("sim:large_s7", product_class, sku)
        self.shelf_script["lane_06"] = deque(["hang"])
        self.restock_script.append({"lane": "lane_06", "source": "sim:large_s7"})
        self._spin_until(
            lambda: any(event[0] == "shelf_cancel" for event in self.events[mark:]),
            15.0,
            "the hung shelf survey to be cancelled at the action timeout",
        )
        # Card 086: the canceled terminal carries the stop the cancel established, so the
        # attempt settles and the ladder retries inside the cycle; a canceled terminal without
        # that evidence would stay unresolved (test_campaign_unresolved_*).
        self._wait_for_front_full(12, timeout=30.0)
        billed_after_cancel = max(s.survey_arm_time_sec for s in self._statuses_from(status_mark))
        self.assertGreaterEqual(
            billed_after_cancel - survey_arm_before_cancel,
            2.0,
            "a goal cancelled mid-motion must charge its in-flight time to the survey "
            "arm-time budget",
        )
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "shelf_cancel", "shelf", "transfer"],
            "a cancelled survey whose stop is established is retried by the ladder",
        )
        self.assertFalse(
            any(s.motion_unresolved for s in self._statuses_from(status_mark)),
            "a settled cancel never claims unresolved motion",
        )
        with world.lock:
            self.assertEqual(world.lanes["lane_06"]["held"], 2)

        # S9: lane evidence that passes its validity horizon while the arm is away at the
        # tray forces a SURVEY_SHELF re-entry at the transfer boundary, before the goal is
        # emitted — the coordinator refuses stale destinations, so the loop must re-survey.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        product_class, sku = self.lane_identity("lane_02")
        with world.lock:
            world.lanes["lane_02"]["target"] = 3
            world.lanes["lane_02"]["invalid"] = True
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:tray_small_s9",
                "cls": product_class,
                "sku": sku,
                "expire_lane": "lane_04",
                # Card 079: this is the step that actually flaked (the confirm status was
                # never published because the feedback lost the race to the result).
                "skip_confirm_feedback": True,
            }
        )
        self.restock_script.append({"lane": "lane_02", "source": "sim:tray_small_s9"})
        self._wait_for_front_full(13, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "tray", "shelf", "transfer"],
            "the transfer-boundary re-check must re-enter SURVEY_SHELF for the lane the "
            "tray step aged out, and only then emit the transfer",
        )
        self.assertEqual(events[0][1], "lane_02", "the evidence event wakes the deficit lane")
        self.assertEqual(events[2][1], "lane_04", "only the aged-out lane is re-surveyed")
        # Card 079: this goal withheld its confirm feedback, so its confirm status has to
        # come from the result path — wait for it by condition (never sleep on it) before
        # reading the window; every assertion below is unchanged.
        self._spin_until(
            lambda: any(
                s.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
                for s in self._statuses_from(status_mark)
            ),
            10.0,
            "the MODE_CONFIRM status of the tray goal that withheld its confirm feedback",
        )
        slice_statuses = self._statuses_from(status_mark)
        phases = [s.phase for s in slice_statuses]
        self.assertNotIn(AutonomousRestockCampaignStatus.PHASE_BLOCKED, phases)
        self.assertEqual(
            sum(
                1
                for status in slice_statuses
                if status.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
            ),
            1,
            "Card 079: exactly one confirm status for the tray goal that withheld its "
            "confirm feedback",
        )
        confirm_index = next(
            index
            for index, status in enumerate(slice_statuses)
            if status.mode == AutonomousRestockCampaignStatus.MODE_CONFIRM
        )
        resweep_index = next(
            index
            for index, status in enumerate(slice_statuses)
            if index > confirm_index
            and status.phase == AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT
        )
        transfer_index = next(
            index
            for index, status in enumerate(slice_statuses)
            if status.mode == AutonomousRestockCampaignStatus.MODE_TRANSFER
        )
        self.assertLess(confirm_index, resweep_index)
        self.assertLess(resweep_index, transfer_index)

        # S10: a NO_CANDIDATE for the largest deficit class must not park a multi-class
        # campaign that can still fill a smaller deficit. The loop asks the next outstanding
        # class, confirms it, transfers, and only then returns for the class that had no
        # candidate — never PHASE_STOCK_EXHAUSTED between the two tray surveys.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        can_class, can_sku = self.lane_identity("lane_01")
        small_class, small_sku = self.lane_identity("lane_02")
        self.assertEqual(can_class, ShelfLane.PRODUCT_CLASS_CAN)
        self.assertEqual(small_class, ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE)
        with world.lock:
            world.lanes["lane_01"]["target"] += 1
            world.lanes["lane_01"]["invalid"] = True
            world.lanes["lane_02"]["target"] += 1
            world.lanes["lane_02"]["invalid"] = True
        self.tray_script.extend(
            [
                {"outcome": "no_candidate"},
                {
                    "outcome": "confirmed",
                    "source": "sim:tray_small_s10",
                    "cls": small_class,
                    "sku": small_sku,
                },
                {
                    "outcome": "confirmed",
                    "source": "sim:tray_can_s10",
                    "cls": can_class,
                    "sku": can_sku,
                },
            ]
        )
        self.restock_script.extend(
            [
                {"lane": "lane_02", "source": "sim:tray_small_s10"},
                {"lane": "lane_01", "source": "sim:tray_can_s10"},
            ]
        )
        self._wait_for_front_full(15, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "shelf", "tray", "tray", "transfer", "tray", "transfer"],
            "NO_CANDIDATE on the largest class must try the next deficit class before parking",
        )
        tray_classes = [event[1] for event in events if event[0] == "tray"]
        self.assertEqual(
            tray_classes,
            [
                ShelfLane.PRODUCT_CLASS_CAN,
                ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE,
                ShelfLane.PRODUCT_CLASS_CAN,
            ],
            "the first tray ask is the largest deficit class, the second is the smaller "
            "class NO_CANDIDATE freed the loop to try",
        )
        slice_statuses = self._statuses_from(status_mark)
        first_tray_index = next(
            index
            for index, status in enumerate(slice_statuses)
            if status.mode == AutonomousRestockCampaignStatus.MODE_SURVEY_TRAY
        )
        second_tray_index = next(
            index
            for index, status in enumerate(slice_statuses)
            if index > first_tray_index
            and status.mode == AutonomousRestockCampaignStatus.MODE_SURVEY_TRAY
        )
        phases_between = [
            status.phase for status in slice_statuses[first_tray_index:second_tray_index]
        ]
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
            phases_between,
            "the cross-class retry must not pass through the idle park",
        )

        # Card 037: each confirming cycle's goal names its own confirmation — the pin follows
        # the candidate just confirmed, never a stale or different one from an earlier cycle.
        first_pin, second_pin = self.restock_goals[-2:]
        self.assertTrue(first_pin.has_object_id)
        self.assertFalse(first_pin.has_lane_id)
        self.assertEqual(first_pin.object_id, self.admitted_ids["sim:tray_small_s10"])
        self.assertTrue(second_pin.has_object_id)
        self.assertFalse(second_pin.has_lane_id)
        self.assertEqual(second_pin.object_id, self.admitted_ids["sim:tray_can_s10"])

        # S11: refutation marks are for one cycle. A refuted-then-confirmed candidate whose
        # transfer is rejected blocks with the mark still held; the next cycle must not
        # re-enter SURVEY_TRAY carrying that stale mark — the admitted candidate is valid
        # again once the mark set resets — so the retry transfers without a third tray ask.
        mark = len(self.events)
        large_class, large_sku = self.lane_identity("lane_03")
        refuted_at_s11 = (-0.42, -0.8, 0.7)
        with world.lock:
            world.lanes["lane_03"]["target"] += 1
            world.lanes["lane_03"]["invalid"] = True
        self.tray_script.extend(
            [
                {
                    "outcome": "refuted",
                    "x": refuted_at_s11[0],
                    "y": refuted_at_s11[1],
                    "z": refuted_at_s11[2],
                    "cls": large_class,
                },
                {
                    "outcome": "confirmed",
                    "source": "sim:tray_large_s11",
                    "cls": large_class,
                    "sku": large_sku,
                },
            ]
        )
        self.restock_script.extend(
            [
                "reject",
                {"lane": "lane_03", "source": "sim:tray_large_s11"},
            ]
        )
        self._wait_for_front_full(16, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "tray", "tray", "transfer", "transfer"],
            "marks from the blocked cycle must not force a third tray survey on the retry",
        )
        tray_slice = [event for event in events if event[0] == "tray"]
        self.assertEqual(
            tray_slice[1][2],
            (refuted_at_s11,),
            "the in-cycle reselection still carries the mark",
        )

        # S12: IDLE wakes on a tray candidate that becomes valid while parked. After front
        # full the loop sleeps with valid lanes; raising a target and admitting back stock
        # without invalidating any lane must still re-enter the work cycle and transfer —
        # a lane-evidence-only wake would leave the new candidate stranded until expiry.
        mark = len(self.events)
        wake_class, wake_sku = self.lane_identity("lane_05")
        with world.lock:
            world.lanes["lane_05"]["target"] = world.lanes["lane_05"]["held"] + 1
            world.add_back("sim:wake_s12", wake_class, wake_sku)
        self.restock_script.append({"lane": "lane_05", "source": "sim:wake_s12"})
        self._wait_for_front_full(17, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["transfer"],
            "a fresh tray candidate must wake IDLE without a lane-evidence event",
        )

        # S13 (Card 051): the typed recoverable skip — a transfer goal that ends in the skip
        # charges max_product_skips, forces the next tray survey with the skip mark carried in
        # skip_mark_positions (the Card 058 split from refuted_positions), reports the product
        # gone, and NEVER publishes PHASE_BLOCKED. The
        # lane stays deficient until the restore below, so the budget is what bounds the loop.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        skip_class, skip_sku = self.lane_identity("lane_04")
        with world.lock:
            # Exactly one candidate: the skip budget is keyed by source, and the scripted
            # coordinator must be able to name the same product the campaign picked.
            world.back.clear()
            world.lanes["lane_04"]["target"] = world.lanes["lane_04"]["held"] + 1
            world.lanes["lane_04"]["invalid"] = True
            world.add_back("sim:skip_s13", skip_class, skip_sku)
        self.restock_script.extend(["skip", "skip"])
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:skip_s13",
                "cls": skip_class,
                "sku": skip_sku,
            }
        )
        self._spin_until(
            lambda: any(
                "treated as gone for the run" in s.detail for s in self._statuses_from(status_mark)
            ),
            20.0,
            "the skip budget reports the product gone after max_product_skips",
        )
        # Restore the deficit the instant the gone report is seen: with the deficit open,
        # Card 051's persistent skip marks force one more tray ask every cycle, and the
        # next poll boundary (up to 50 ms) is exactly where that ask can race the restore.
        # Restoring first makes the stray impossible; the settle wait below then parks the
        # loop at a fresh FRONT_FULL before S14 takes its mark.
        with world.lock:
            world.lanes["lane_04"]["target"] = world.lanes["lane_04"]["held"]
        settle_mark = len(self.campaign_statuses)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                for s in self.campaign_statuses[settle_mark:]
            ),
            15.0,
            "the loop to park at PHASE_FRONT_FULL after the skip budget restored",
        )
        window = self._statuses_from(status_mark)
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED for s in window),
            "a recoverable skip must never publish PHASE_BLOCKED",
        )
        self.assertTrue(
            any("skipped 1 of 2" in s.detail for s in window),
            f"the first skip must be receipted against the budget: {[s.detail for s in window]}",
        )
        self.assertTrue(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
                and "recoverable skip" in s.detail
                for s in window
            ),
            "the skip must be reported on a measured status, not a blocked one",
        )
        events = self.events[mark:]
        # Prefix match: a settle-cycle tray ask that raced the restore (see above) may add
        # a trailing event after the four this scenario owns; it must never reorder them.
        self.assertEqual(
            [event[0] for event in events[:4]],
            ["shelf", "transfer", "tray", "transfer"],
            "skip 1 on the skip path, then the forced tray survey with the mark, skip 2",
        )
        tray_events = [event for event in events[:4] if event[0] == "tray"]
        self.assertEqual(len(tray_events), 1, events)
        self.assertTrue(
            tray_events[0][3],
            "the forced tray survey must carry the skip mark in skip_mark_positions",
        )
        self.assertFalse(
            tray_events[0][2],
            "a skip mark must not be merged into refuted_positions (Card 058 split)",
        )
        # The deficit restore moved above, next to the gone report it belongs to.

        # S14 (Card 037, was S13): a confirmation whose observation never reaches the world
        # state must NOT fall back to an unpinned transfer goal — and under Card 051's
        # product-fell rule it does not block either: the absent product is marked gone and
        # the cycle continues, so the next survey reports no candidate and the walk parks
        # typed at PHASE_STOCK_EXHAUSTED with the transfer count unchanged.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        goals_before_s14 = len([goal for goal in self.restock_goals if goal is not None])
        drop_class, drop_sku = self.lane_identity("lane_06")
        with world.lock:
            world.lanes["lane_06"]["target"] = world.lanes["lane_06"]["held"] + 1
            world.lanes["lane_06"]["invalid"] = True
        self.tray_script.extend(
            [
                {
                    "outcome": "confirmed",
                    "source": "sim:tray_ghost_s14",
                    "cls": drop_class,
                    "sku": drop_sku,
                    "drop_admission": True,
                },
                {"outcome": "no_candidate"},
            ]
        )
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
                and "gone from the tray" in s.detail
                for s in self._statuses_from(status_mark)
            ),
            20.0,
            "the ghost confirmation is reported gone, not blocked (Card 051 product-fell)",
        )
        self.assertFalse(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "absent from the world-state snapshot" in s.detail
                for s in self._statuses_from(status_mark)
            ),
            "the product-fell case must not publish the old blocking detail",
        )
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self._statuses_from(status_mark)
            ),
            20.0,
            "PHASE_STOCK_EXHAUSTED once the ghost candidate never arrives",
        )
        events = self.events[mark:]
        # Prefix match, same settle rationale as S13: a trailing tray ask may follow the
        # three events this scenario owns; it must never precede the shelf survey.
        self.assertEqual(
            [event[0] for event in events[:3]],
            ["shelf", "tray", "tray"],
            "an unresolvable confirmation continues before any transfer goal (Card 037/051)",
        )
        self.assertEqual(
            len([goal for goal in self.restock_goals if goal is not None]),
            goals_before_s14,
            "no transfer goal may be emitted after an unresolvable confirmation",
        )
        with world.lock:
            # Restore the target so the closing park cannot wake into another tray ask when
            # lane evidence later passes its horizon; the gone report itself is the assertion.
            world.lanes["lane_06"]["target"] = world.lanes["lane_06"]["held"]

        # S15 (Card 052): a recoverable lane-survey failure climbs the campaign ladder
        # instead of publishing PHASE_BLOCKED — rung 1 retreats to the safe survey pose
        # (the failure reports a terminal stop, so the arm may have moved), rung 3 retries
        # with fresh evidence, and the cycle closes with confirm + transfer. The
        # classification is RECOVERABLE only because the fake's evidence flags say so; the
        # same failure without them classifies UNSAFE and blocks exactly as before (S7).
        # Note: S13's product skip mark persists for the rest of the walk (the fixture
        # renders every back-stock product at one fixed pose, so the mark's clear-on-move
        # rule can never fire), which forces the tray path — script it.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        retreat_mark = len(self.retreat_goals)
        s15_class, s15_sku = self.lane_identity("lane_05")
        with world.lock:
            world.lanes["lane_05"]["target"] += 1
            world.lanes["lane_05"]["invalid"] = True
        self.shelf_script["lane_05"] = deque(["fail_exec"])
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:recover_s15",
                "cls": s15_class,
                "sku": s15_sku,
            }
        )
        self.restock_script.append({"lane": "lane_05", "source": "sim:recover_s15"})
        self._wait_for_front_full(18, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "shelf", "tray", "transfer"],
            "the recoverable failure is retried after one retreat, never blocked",
        )
        window = self._statuses_from(status_mark)
        blocked_in_window = [
            s.detail for s in window if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
        ]
        self.assertFalse(
            blocked_in_window,
            f"a recoverable lane-survey failure must not publish PHASE_BLOCKED: "
            f"{blocked_in_window}",
        )
        self.assertEqual(
            len(self.retreat_goals) - retreat_mark,
            1,
            "rung 1 commands exactly one retreat for the failed attempt",
        )
        self.assertEqual(
            self.retreat_goals[retreat_mark]["station"],
            "tray_1",
            "the retreat aims at the configured safe survey station",
        )

        # S16 (Card 052): the lane path's bounded charging — the lane keeps failing
        # recoverably (plan refused, nothing moved, so no retreat), every cycle burns
        # exactly max_lane_survey_attempts_per_cycle attempts before rung 5 skips it for
        # the cycle, and the rung-5 charge succeeds without ever publishing PHASE_BLOCKED
        # (the shared budget's exhaustion latch is S19's, on the station path). The lane
        # has no deficit here, so nothing else may fire while it fails. The deficit
        # restore rides the same poll that sees the fourth attempt, so no third cycle can
        # slip extra shelf events past the exact-count assertion.
        #
        # Re-scope note (review F5): commit ce328ce's message describes S16 as
        # "fail_plan x6 → PHASE_BLOCKED naming rung 5" — true of that commit alone. Commit
        # 5d44554 re-scoped S16 to the per-cycle bound and moved the shared rung-5
        # exhaustion latch to S19 (station path) while raising the fixture budget 2 → 4;
        # S19b below then closes the gap that left behind: the lane-labelled
        # survey_skip_exhausted_detail("lane …") branch, asserted end-to-end once the
        # budget is already spent. The card and evidence README describe the final state.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        retreat_mark = len(self.retreat_goals)
        with world.lock:
            world.lanes["lane_05"]["invalid"] = True
        self.shelf_script["lane_05"] = deque(["fail_plan"] * 8)
        s16_restored = False

        def s16_two_cycles_bounded():
            nonlocal s16_restored
            count = sum(
                1 for event in self.events[mark:] if event[0] == "shelf" and event[1] == "lane_05"
            )
            if count >= 4 and not s16_restored:
                with world.lock:
                    world.lanes["lane_05"]["invalid"] = False
                    world.lanes["lane_05"]["verified_age"] = 0.0
                s16_restored = True
            return count >= 4

        self._spin_until(
            s16_two_cycles_bounded,
            30.0,
            "at least two charging cycles of max_lane_survey_attempts_per_cycle attempts each",
        )
        # Let any cycle that was mid-flight when the restore landed finish its round, so
        # the SURVEYING_FRONT count and the shelf-goal count describe the same cycles.
        quiet_deadline = time.monotonic() + 0.5
        while time.monotonic() < quiet_deadline:
            time.sleep(0.05)
        shelf_attempts = [
            event for event in self.events[mark:] if event[0] == "shelf" and event[1] == "lane_05"
        ]
        surveying_cycles = [
            s
            for s in self._statuses_from(status_mark)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT
        ]
        self.assertGreaterEqual(
            len(surveying_cycles),
            2,
            "at least two charging cycles observed",
        )
        self.assertEqual(
            len(shelf_attempts),
            2 * len(surveying_cycles),
            "every charging cycle issues exactly max_lane_survey_attempts_per_cycle "
            "(2) attempts before rung 5 skips the lane for that cycle — the idle wake "
            "re-enters immediately, so count cycles, not wall time",
        )
        self.assertEqual(
            len(self.retreat_goals),
            retreat_mark,
            "not-started failures are rung-1 no-ops; no retreat may be commanded",
        )
        window = self._statuses_from(status_mark)
        blocking = [s for s in window if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED]
        self.assertFalse(
            blocking,
            f"charging rung 5 must never publish PHASE_BLOCKED: {[s.detail for s in blocking]}",
        )

        # S18 (Card 052): a recoverable tray-survey failure climbs the ladder — rung 1 is a
        # no-op (the script's not-started evidence says nothing moved), rung 2 rotates the
        # failed station to the end of the overview list so the retry re-surveys ANOTHER
        # station first, rung 3 retries inside max_tray_attempts_per_cycle, and the
        # confirm + transfer close the cycle. No PHASE_BLOCKED may appear in this window.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        goals_before = len(self.tray_goals)
        s18_class, s18_sku = self.lane_identity("lane_01")
        with world.lock:
            world.lanes["lane_01"]["target"] += 1
            world.lanes["lane_01"]["invalid"] = True
        self.tray_script.extend(
            [
                {"outcome": "unreachable_recoverable", "failed_station": "tray_1"},
                {
                    "outcome": "confirmed",
                    "source": "sim:recover_s18",
                    "cls": s18_class,
                    "sku": s18_sku,
                },
            ]
        )
        self.restock_script.append({"lane": "lane_01", "source": "sim:recover_s18"})
        self._wait_for_front_full(19, timeout=30.0)
        events = self.events[mark:]
        self.assertEqual(
            [event[0] for event in events],
            ["shelf", "tray", "tray", "transfer"],
            "the recoverable tray failure is retried after a station rotation, never blocked",
        )
        window = self._statuses_from(status_mark)
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED for s in window),
            "a recoverable tray-survey failure must not publish PHASE_BLOCKED",
        )
        window_goals = self.tray_goals[goals_before:]
        self.assertEqual(len(window_goals), 2, window_goals)
        self.assertEqual(
            window_goals[0]["overview_stations"],
            [],
            "the first attempt keeps the server's nominal station derivation",
        )
        self.assertEqual(
            window_goals[1]["overview_stations"],
            ["tray_2", "tray_1"],
            "the retry is explicit with the failed station rotated to the end (rung 2)",
        )

        # S19 (Card 052): tray-station rung 5, the all-skipped backoff, and the shared
        # budget's exhaustion latch — a station that fails
        # max_lane_survey_attempts_per_cycle times within the cycle is skipped for the
        # cycle and charged to max_survey_skips (S16 already charged twice of four); when
        # every station is skipped the cycle backs off WITHOUT PHASE_BLOCKED and revisits
        # next cycle (the skips are per cycle, the charge is per run); on the fifth charge
        # the budget is exhausted and PHASE_BLOCKED names rung 5, the station and the
        # budget.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        with world.lock:
            world.lanes["lane_02"]["target"] += 1
            world.lanes["lane_02"]["invalid"] = True
        self.tray_script.extend(
            [
                {"outcome": "unreachable_recoverable", "failed_station": "tray_1"},
                {"outcome": "unreachable_recoverable", "failed_station": "tray_1"},
                {"outcome": "unreachable_recoverable", "failed_station": "tray_2"},
                {"outcome": "unreachable_recoverable", "failed_station": "tray_2"},
                # Cycle 2: the per-cycle skips reset, the charge does not.
                {"outcome": "unreachable_recoverable", "failed_station": "tray_1"},
                {"outcome": "unreachable_recoverable", "failed_station": "tray_1"},
            ]
        )
        # Restore the deficit on the SAME poll that first sees the latch — and leave the
        # lane needing a survey: the latch's 50 ms backoff can otherwise run one stray
        # cycle against the exhausted tray script (NO_CANDIDATE → STOCK_EXHAUSTED idle
        # with the deficit still open) before the restore lands, and a later restore then
        # parks a loop whose wake conditions are all false. The invalid flag is the wake
        # handle either way: the next cycle surveys the lane (clearing it) and re-measures
        # to FRONT_FULL. Same hardening as S20, plus a deterministic wake. (Pre-existing
        # fixture race; surfaced by -j2 package runs 2026-09-29.)
        s19_restored = False

        def s19_latched():
            nonlocal s19_restored
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "rung 5" in s.detail
                and "max_survey_skips" in s.detail
                for s in self._statuses_from(status_mark)
            )
            if hit and not s19_restored:
                with world.lock:
                    world.lanes["lane_02"]["target"] = world.lanes["lane_02"]["held"]
                    world.lanes["lane_02"]["invalid"] = True
                    world.lanes["lane_02"]["verified_age"] = 120.0
                s19_restored = True
            return hit

        self._spin_until(
            s19_latched,
            30.0,
            "PHASE_BLOCKED naming rung 5 once the tray-station skip budget is exhausted",
        )
        window = self._statuses_from(status_mark)
        tray_events = [event for event in self.events[mark:] if event[0] == "tray"]
        self.assertGreaterEqual(
            len(tray_events),
            6,
            "four station attempts in the charging cycle (two skips, all-skipped "
            "backoff) plus two before the rung-5 latch in the next cycle",
        )
        blocked_in_window = [
            s for s in window if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
        ]
        self.assertTrue(
            all(
                "rung 5" in s.detail and "max_survey_skips" in s.detail for s in blocked_in_window
            ),
            f"every block in this window is the rung-5 latch: "
            f"{[s.detail for s in blocked_in_window]}",
        )
        self.assertIn("station tray_1", blocked_in_window[0].detail)
        settle_mark = len(self.campaign_statuses)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                for s in self.campaign_statuses[settle_mark:]
            ),
            15.0,
            "the loop to park after S19 restored its deficit",
        )

        # S20 (Card 052): the tray loop's own rung-3 budget — a pre-motion failure with no
        # failed station (nothing to rotate or skip) retries inside
        # max_tray_attempts_per_cycle (8); the ninth boundary latches PHASE_BLOCKED naming
        # rung 3 and the budget, after exactly eight attempts.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        with world.lock:
            world.lanes["lane_03"]["target"] += 1
            world.lanes["lane_03"]["invalid"] = True
        self.tray_script.extend(
            [
                {
                    "outcome": "unreachable_recoverable",
                    "failed_station": "",
                    "detail": "scripted: the tray survey backend was unavailable before "
                    "any leg ran",
                }
            ]
            * 8
        )
        # Restore the deficit on the SAME poll that first sees the latch: the next cycle
        # (50 ms backoff) would otherwise re-enter the tray with an exhausted script, and
        # its event could land between the poll and the exact-count assertion below.
        s20_restored = False

        def s20_latched():
            nonlocal s20_restored
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "rung 3" in s.detail
                and "max_tray_attempts_per_cycle" in s.detail
                for s in self._statuses_from(status_mark)
            )
            if hit and not s20_restored:
                with world.lock:
                    world.lanes["lane_03"]["target"] = world.lanes["lane_03"]["held"]
                    world.lanes["lane_03"]["invalid"] = False
                    world.lanes["lane_03"]["verified_age"] = 0.0
                s20_restored = True
            return hit

        self._spin_until(
            s20_latched,
            30.0,
            "PHASE_BLOCKED naming rung 3 once max_tray_attempts_per_cycle is exhausted",
        )
        tray_events = [event for event in self.events[mark:] if event[0] == "tray"]
        self.assertEqual(
            len(tray_events),
            8,
            "exactly max_tray_attempts_per_cycle attempts before the rung-3 latch "
            f"(got {len(tray_events)})",
        )

        # S19b (Card 052, review F5): the lane-labelled rung-5 exhaustion branch, asserted
        # end-to-end. S19 already spent the shared max_survey_skips budget (4 of 4), so a
        # lane that burns its per-cycle attempts and needs its rung-5 charge now latches
        # PHASE_BLOCKED naming the LANE — survey_skip_exhausted_detail("lane …"), the same
        # helper S19 pinned under the station label. Two scripted failures, no deficit, no
        # tray; the restore rides the latch predicate so no second blocked cycle can slip
        # extra shelf events past the count.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        with world.lock:
            world.lanes["lane_04"]["invalid"] = True
        self.shelf_script["lane_04"] = deque(["fail_plan"] * 2)
        s19b_restored = False

        def s19b_lane_latched():
            nonlocal s19b_restored
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "lane lane_04" in s.detail
                and "rung 5" in s.detail
                and "max_survey_skips" in s.detail
                for s in self._statuses_from(status_mark)
            )
            if hit and not s19b_restored:
                with world.lock:
                    world.lanes["lane_04"]["invalid"] = False
                    world.lanes["lane_04"]["verified_age"] = 0.0
                s19b_restored = True
            return hit

        self._spin_until(
            s19b_lane_latched,
            20.0,
            "PHASE_BLOCKED naming the lane-labelled rung-5 exhaustion (budget spent by S16+S19)",
        )
        shelf_events_04 = [
            event for event in self.events[mark:] if event[0] == "shelf" and event[1] == "lane_04"
        ]
        self.assertGreaterEqual(
            len(shelf_events_04),
            2,
            "the lane burns max_lane_survey_attempts_per_cycle attempts before the latch",
        )

        # S21 (Card 052, review F1): the retreat admission race, fail-closed. A recoverable
        # failure with motion evidence commands the rung-1 retreat; the fake server then
        # holds the ADMISSION response past action_timeout_sec (3.0), so the campaign's
        # admission wait fails while the goal is genuinely in flight. The fail-closed fix
        # cancels everything its dedicated client owns, reports the arm's state as no
        # longer established and latches — PHASE_BLOCKED naming rung 1 — instead of the
        # old benign "nothing was commanded" no-op. The late-accepted goal arriving at the
        # fake afterwards proves the race was real: something COULD have been commanded.
        mark = len(self.events)
        status_mark = len(self.campaign_statuses)
        retreat_mark = len(self.retreat_goals)
        with world.lock:
            world.lanes["lane_04"]["invalid"] = True
        self.shelf_script["lane_04"] = deque(["fail_exec"])
        type(self).retreat_admission_hang_sec = 6.0
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "survey recovery rung 1" in s.detail
                for s in self._statuses_from(status_mark)
            ),
            20.0,
            "PHASE_BLOCKED naming rung 1 when retreat admission never answered "
            "(fail-closed, review F1)",
        )
        window = self._statuses_from(status_mark)
        rung1_blocks = [
            s.detail for s in window if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
        ]
        self.assertTrue(
            all(
                "survey recovery rung 1" in detail
                or "no goal is sent while motion is unresolved" in detail
                for detail in rung1_blocks
            ),
            "every block in this window is the rung-1 latch or the unresolved-motion hold "
            f"that follows it (Card 086): {rung1_blocks}",
        )
        block_seq = max(
            s.sequence for s in window if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
        )
        # The goal the old code would have leaked IS admitted after the fact — the race
        # existed; only the campaign's verdict on it changed.
        self._spin_until(
            lambda: len(self.retreat_goals) >= retreat_mark + 1,
            15.0,
            "the unconfirmed retreat goal to be admitted by the server after the wait failed",
        )
        type(self).retreat_admission_hang_sec = 0.0
        with world.lock:
            world.lanes["lane_04"]["invalid"] = False
            world.lanes["lane_04"]["verified_age"] = 0.0
        # Card 086 stage 1 (CMB-SPEC-13), replacing the old "self-heals and may park BEFORE the
        # late retreat lands" assertion: the campaign holds the unresolved retreat attempt, so
        # nothing is sent and it cannot park until the late goal is admitted, canceled by its
        # exact UUID, and its delivered terminal settles the attempt.
        blocked_marks = [
            s
            for s in window
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED and s.motion_unresolved
        ]
        self.assertTrue(blocked_marks, "the rung-1 latch carries the explicit motion_unresolved")
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.sequence > block_seq
                for s in self.campaign_statuses
            ),
            15.0,
            "the loop to park after S21's late retreat settled",
        )
        parked_index = next(
            index
            for index, s in enumerate(self.campaign_statuses)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
            and s.sequence > block_seq
        )
        self.assertFalse(
            self.campaign_statuses[parked_index].motion_unresolved,
            "the park after S21 is a settled state, not an unresolved one",
        )
        self.assertGreater(len(self.retreat_goals), retreat_mark, "the late retreat goal landed")
        self.assertGreater(
            self.campaign_status_times[parked_index],
            self.retreat_goal_times[retreat_mark],
            "the campaign must not park before the late retreat goal lands (the old self-heal)",
        )
        self.assertEqual(
            [event[0] for event in self.events[mark:]],
            ["shelf"],
            "the only goal sent in S21 is the lane survey that failed; the unresolved retreat "
            "inhibits every later send, whatever backoff elapsed or lane evidence was restored",
        )

        # The run ends at the configured cycle bound, reporting both arm times.
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_COMPLETE
                for s in self.campaign_statuses
            ),
            30.0,
            "PHASE_COMPLETE at max_cycles",
        )
        final = self.campaign_statuses[-1]
        self.assertEqual(final.phase, AutonomousRestockCampaignStatus.PHASE_COMPLETE)
        self.assertGreater(final.survey_arm_time_sec, 0.0)
        self.assertGreater(final.transfer_arm_time_sec, 0.0)
        self.assertEqual(final.successful_transfers, 19)
