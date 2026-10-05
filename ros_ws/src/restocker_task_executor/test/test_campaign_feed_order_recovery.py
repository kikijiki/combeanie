# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 066: a feed-blocked tray NO_CANDIDATE is a bounded rung, never stock exhaustion.

Milestone 10 §6, spec committed before this code ("The tray survey nominates only the front
of a feed column"). The survey nominates only feed-column fronts and reports how many
matching candidates stood behind something (`feed_blocked_candidates`). The campaign reads a
NO_CANDIDATE with that count as recoverable: it re-surveys so the front is observed again,
charging `max_feed_order_resurveys` (reset by a successful transfer), and latches
PHASE_BLOCKED naming feed order when the budget is spent. Cases run alphabetically against one
campaign process:

- test_a: a plain NO_CANDIDATE (nothing feed-blocked) still parks at PHASE_STOCK_EXHAUSTED
  and spends nothing: the rung needs the typed count, not just an empty answer.
- test_b: a feed-blocked NO_CANDIDATE is receipted on PHASE_MEASURED ("attempt 1 of 2"), the
  forced re-survey confirms the front and the transfer closes at PHASE_FRONT_FULL.
- test_c: the success reset the count, so two further consecutive feed-blocked answers are
  attempts 1 and 2 again before the confirm closes the loop.
- test_d: three consecutive feed-blocked answers latch PHASE_BLOCKED naming feed order and
  max_feed_order_resurveys=2, never PHASE_STOCK_EXHAUSTED.
- test_e: Card 055's rung is unchanged, but its receipt now carries the coordinator's own
  refusal detail, so a feed-queue refusal of a pinned product reads as one.
- test_f: the skip path is feed-safe (Card 065's front1 shape). A fresh world-state candidate
  that the latest overview reports as standing behind a front is not handed to the coordinator
  on the skip path: the campaign surveys, and the transfer is the confirmed, pinned one.
- test_g: the control. When the latest overview vouches for the candidate as a front, the skip
  path is taken as before, with no survey.
- test_h: the coordinator's horizon (after Card 069's cmbrev069b B1). A vouched-for fresh front
  opens the skip path, but a product 150 s old stands behind it: older than the tray window
  (120 s), younger than the coordinator's selection horizon (180 s), so the coordinator could
  still take it. The skip path is refused and the cycle surveys.
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
import pytest
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct, SurveyLane, SurveyTray, SurveyViewpoint
from restocker_interfaces.msg import (
    AutonomousRestockCampaignStatus,
    ObjectObservation,
    RestockCoordinatorStatus,
)
from restocker_interfaces.srv import GetWorldState

RESTOCK_ACTION = "/test/feed_order/restock_product"
SHELF_ACTION = "/test/feed_order/survey_lane"
TRAY_ACTION = "/test/feed_order/survey_tray"
RETREAT_ACTION = "/test/feed_order/survey_viewpoint"
STATUS_TOPIC = "/test/feed_order/coordinator_status"
CAMPAIGN_STATUS_TOPIC = "/test/feed_order/status"
WORLD_STATE_SERVICE = "/test/feed_order/get_snapshot"
FEED_QUEUE_REFUSAL = (
    "task selection failed: requested object 15 has no eligible destination lane; pair "
    "refusals: object 15 object_unavailable x6 (object 15 and lane lane_01: another free tray "
    "product blocks the front of this feed queue)"
)
FEED_RECEIPT = "feed-order NO_CANDIDATE recovery"


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped default recovery budget (2)."""
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign_feed_order_subject",
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
                "coordinator_status_topic": STATUS_TOPIC,
                "campaign_status_topic": CAMPAIGN_STATUS_TOPIC,
                "world_state_service_name": WORLD_STATE_SERVICE,
                # Shipped defaults on purpose: the file asserts "attempt n of 2" and the
                # latch naming max_feed_order_resurveys=2.
                "max_ncp_resurveys": 2,
                "max_feed_order_resurveys": 2,
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
        ],
    )
    return launch.LaunchDescription([campaign, launch_testing.actions.ReadyToTest()])


class TestFeedOrderRecovery(unittest.TestCase):
    """The Card 066 contract: feed-blocked NO_CANDIDATE recovers, resets, and terminates."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("campaign_feed_order_fixture")
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
                depth=64,
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
    def _survey_tray(cls, goal_handle):
        cls.tray_goals.append(goal_handle.request)
        cls.events.append(("tray",))
        if not cls.tray_script:
            result = SurveyTray.Result()
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "script exhausted: no candidate"
            goal_handle.succeed()
            return result
        scripted = cls.tray_script.pop(0)
        result = SurveyTray.Result()
        if scripted["outcome"] == "confirmed":
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
        elif scripted["outcome"] == "feed_blocked":
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = (
                "scripted: every remaining matching candidate stands behind another tray "
                "product in its feed column"
            )
            result.feed_blocked_candidates = scripted["count"]
        elif scripted["outcome"] == "no_candidate":
            result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
            result.detail = "scripted: no candidate (products unseen)"
            # The slot-10 fallen end state: nothing observes the products again, so the
            # next snapshot no longer carries them as stock either. Draining here (the
            # instant the campaign will publish STOCK_EXHAUSTED) also closes the fixture's
            # hot loop — at cycle_period 0 a still-counted candidate behind an open deficit
            # would re-wake the loop and fire a stray transfer into the test's script gap
            # before the scenario can restore its deficit. The recovery charge already
            # proved stock_remains at refusal time; this is the state afterwards.
            with cls.world.lock:
                cls.world.back.clear()
        else:
            raise AssertionError(f"unknown scripted tray outcome {scripted}")
        # Card 066 skip-path clause: an overview whose candidates stand where FakeWorld puts
        # every back product, each with the server's feed-column front flag.
        for entry in scripted.get("overview", []):
            # A bare flag stands at the shared tray pose; a (position, flag) pair is placed.
            position, front = entry if isinstance(entry, tuple) else ((-0.4, -0.8, 0.7), entry)
            candidate = ObjectObservation()
            candidate.header.frame_id = "world"
            candidate.header.stamp = cls.node.get_clock().now().to_msg()
            candidate.product_class = scripted.get("cls", 0)
            candidate.backend_name = "wrist_rgbd_tray_overview"
            candidate.status = ObjectObservation.STATUS_OK
            candidate.pose.pose.position.x = position[0]
            candidate.pose.pose.position.y = position[1]
            candidate.pose.pose.position.z = position[2]
            candidate.pose.pose.orientation.w = 1.0
            result.overview_candidates.append(candidate)
            result.overview_candidate_feed_front.append(front)
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
        result = RestockProduct.Result()
        if scripted == "no_pair":
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            goal_handle.abort()
            return result
        if scripted == "feed_queue_refusal":
            # The coordinator's own words for the dense-run shape (Card 063 dev run 2).
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            result.detail = FEED_QUEUE_REFUSAL
            goal_handle.abort()
            return result
        if scripted == "no_pair_drained":
            # The coordinator refuses the pair AND the class's stock is gone by the time
            # the campaign re-measures: the genuine-exhaustion classification must win and
            # the recovery budget must stay untouched.
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            with cls.world.lock:
                cls.world.back.clear()
            goal_handle.abort()
            return result
        cls.world.apply_transfer(scripted["lane"], scripted["source"])
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
        return [s for s in cls.campaign_statuses if FEED_RECEIPT in s.detail]

    def lane_identity(self, lane_id):
        """Return the (class, sku) the fixture declares for one lane."""
        lane = self.world.lanes[lane_id]
        return lane["cls"], lane["sku"]

    def confirmed(self, source, lane):
        """Script one close-confirmed candidate of the lane's product."""
        product_class, sku = self.lane_identity(lane)
        return {"outcome": "confirmed", "source": source, "cls": product_class, "sku": sku}

    def open_deficit(self, lane, source):
        """
        Open a one-product deficit with one back product the tray still holds.

        Its evidence is older than the tray validity window, so no valid candidate lets the
        campaign take the skip path: every case goes through SURVEY_TRAY. A stale candidate
        does not wake a parked loop either, so the lane's evidence is expired as well: the
        lane survey is the wake-up.
        """
        product_class, sku = self.lane_identity(lane)
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back(source, product_class, sku, observed_age=600.0)
            self.world.expire_lane(lane)

    def close_deficit_when(self, lane, predicate):
        """Wrap `predicate` so the deficit closes on the same poll that first sees it."""
        closed = False

        def check():
            nonlocal closed
            hit = predicate()
            if hit and not closed:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                closed = True
            return hit

        return check

    def front_full_after(self, status_mark, transfers):
        """Whether the window reached PHASE_FRONT_FULL with `transfers` successful transfers."""
        return any(
            s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
            and s.successful_transfers == transfers
            for s in self.campaign_statuses[status_mark:]
        )

    def assert_never_exhausted_or_blocked(self, window):
        """Stock stood behind a front the whole time: neither parking verdict may appear."""
        wrong = [
            (s.phase, s.detail)
            for s in window
            if s.phase
            in (
                AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                AutonomousRestockCampaignStatus.PHASE_BLOCKED,
            )
        ]
        self.assertFalse(wrong, f"a feed-blocked tray is neither exhausted nor blocked: {wrong}")

    # -- the cases -----------------------------------------------------------

    def test_a_plain_no_candidate_still_parks_without_a_charge(self):
        """An empty answer with nothing feed-blocked parks at STOCK_EXHAUSTED as before."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_01"
        self.tray_script.append({"outcome": "no_candidate"})
        self.open_deficit(lane, "sim:feed_a")
        self._spin_until(
            self.close_deficit_when(
                lane,
                lambda: any(
                    s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                    for s in self.campaign_statuses[status_mark:]
                ),
            ),
            20.0,
            "PHASE_STOCK_EXHAUSTED for a NO_CANDIDATE that reports nothing feed-blocked",
        )
        self.assertEqual(self._receipts(), [], "a plain NO_CANDIDATE charges no feed-order rung")

    def test_b_feed_blocked_no_candidate_recovers(self):
        """One feed-blocked answer: a MEASURED receipt, a forced re-survey, then FRONT_FULL."""
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        lane = "lane_02"
        self.restock_script.append({"lane": lane, "source": "sim:feed_b"})
        self.tray_script.extend(
            [{"outcome": "feed_blocked", "count": 3}, self.confirmed("sim:feed_b", lane)]
        )
        self.open_deficit(lane, "sim:feed_b")
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "PHASE_FRONT_FULL after the feed-order rung re-surveyed and the front transferred",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
                and f"{FEED_RECEIPT}" in s.detail
                and "attempt 1 of 2" in s.detail
                for s in window
            ),
            f"the rung is receipted on a measured status: {[s.detail for s in window]}",
        )
        self.assert_never_exhausted_or_blocked(window)
        events = [event[0] for event in self.events[event_mark:]]
        self.assertEqual(events, ["tray", "tray", "transfer"])

    def test_c_a_successful_transfer_resets_the_count(self):
        """After a transfer the rung starts again at attempt 1, and can spend both attempts."""
        status_mark = len(self.campaign_statuses)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        lane = "lane_03"
        self.restock_script.append({"lane": lane, "source": "sim:feed_c"})
        self.tray_script.extend(
            [
                {"outcome": "feed_blocked", "count": 1},
                {"outcome": "feed_blocked", "count": 1},
                self.confirmed("sim:feed_c", lane),
            ]
        )
        self.open_deficit(lane, "sim:feed_c")
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "PHASE_FRONT_FULL after two feed-order rungs within a reset budget",
        )
        window = self.campaign_statuses[status_mark:]
        details = [s.detail for s in window if FEED_RECEIPT in s.detail]
        self.assertTrue(
            any("attempt 1 of 2" in d for d in details)
            and any("attempt 2 of 2" in d for d in details),
            f"the success in test_b reset the count: {details}",
        )
        self.assert_never_exhausted_or_blocked(window)

    def test_d_an_exhausted_budget_blocks_naming_feed_order(self):
        """Three consecutive feed-blocked answers: two receipts, then a named PHASE_BLOCKED."""
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        lane = "lane_04"
        self.tray_script.extend([{"outcome": "feed_blocked", "count": 2}] * 3)
        self.open_deficit(lane, "sim:feed_d")
        self._spin_until(
            self.close_deficit_when(
                lane,
                lambda: any(
                    s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                    and "max_feed_order_resurveys=2" in s.detail
                    for s in self.campaign_statuses[status_mark:]
                ),
            ),
            30.0,
            "PHASE_BLOCKED naming feed order and max_feed_order_resurveys",
        )
        # Up to the latch: afterwards the fixture closes the deficit, and a cycle racing that
        # close may legitimately see an unscripted, plain NO_CANDIDATE.
        statuses = self.campaign_statuses[status_mark:]
        latch = next(
            index
            for index, s in enumerate(statuses)
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
            and "max_feed_order_resurveys" in s.detail
        )
        window = statuses[: latch + 1]
        blocked = window[-1]
        self.assertIn("feed order", blocked.detail)
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED for s in window),
            "stock behind a front is never reported as exhausted",
        )
        details = [s.detail for s in window if FEED_RECEIPT in s.detail]
        self.assertEqual(len(details), 2, f"exactly the budget's receipts: {details}")
        self.assertNotIn(
            ("transfer",),
            self.events[event_mark:],
            "nothing is transferred from a feed-blocked tray",
        )

    def test_e_a_feed_queue_refusal_is_named_in_the_ncp_receipt(self):
        """Card 055's rung, unchanged, carries the coordinator's refusal in its receipt."""
        status_mark = len(self.campaign_statuses)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        lane = "lane_05"
        self.restock_script.extend(["feed_queue_refusal", {"lane": lane, "source": "sim:feed_e"}])
        self.tray_script.extend(
            [self.confirmed("sim:feed_e", lane), self.confirmed("sim:feed_e", lane)]
        )
        self.open_deficit(lane, "sim:feed_e")
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "PHASE_FRONT_FULL after Card 055's forced re-survey",
        )
        window = self.campaign_statuses[status_mark:]
        receipts = [s.detail for s in window if "forced tray re-survey attempt 1 of 2" in s.detail]
        self.assertTrue(receipts, f"Card 055's rung fired: {[s.detail for s in window]}")
        self.assertIn("blocks the front of this feed queue", receipts[0])

    def test_f_the_skip_path_refuses_a_candidate_the_overview_saw_behind_a_front(self):
        """A fresh candidate reported behind a front forces a survey instead of the skip path."""
        lane = "lane_06"
        product_class, sku = self.lane_identity(lane)
        # Phase 1: a stale product forces a survey, whose overview reports the product's place
        # as behind a front; the confirmed product is transferred (pinned).
        status_mark = len(self.campaign_statuses)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        self.restock_script.append({"lane": lane, "source": "sim:skip_f1"})
        confirmed = self.confirmed("sim:skip_f1", lane)
        confirmed["overview"] = [False]
        self.tray_script.append(confirmed)
        self.open_deficit(lane, "sim:skip_f1")
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "the phase-1 transfer that leaves a 'behind a front' reference overview",
        )
        # Phase 2: a fresh candidate at that place. Without the clause the coordinator would take
        # it on the skip path; with it, the campaign surveys first.
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        self.restock_script.append({"lane": lane, "source": "sim:skip_f2"})
        confirmed = self.confirmed("sim:skip_f2", lane)
        confirmed["overview"] = [True]
        self.tray_script.append(confirmed)
        with self.world.lock:
            self.world.lanes[lane]["target"] = 2
            self.world.add_back("sim:skip_f2", product_class, sku)
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 2),
            30.0,
            "the phase-2 transfer",
        )
        events = [event[0] for event in self.events[event_mark:]]
        self.assertEqual(events, ["tray", "transfer"], "surveyed before the transfer")
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any("skip path refused" in s.detail and "behind a front" in s.detail for s in window),
            f"the refusal is receipted: {[s.detail for s in window]}",
        )
        self.assertTrue(self.restock_goals[-1].has_object_id, "the transfer is pinned")

    def test_g_a_candidate_vouched_for_as_a_front_keeps_the_skip_path(self):
        """The control: the latest overview calls it a front, so no survey is added."""
        lane = "lane_06"
        product_class, sku = self.lane_identity(lane)
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        self.restock_script.append({"lane": lane, "source": "sim:skip_g"})
        with self.world.lock:
            self.world.lanes[lane]["target"] = 3
            self.world.add_back("sim:skip_g", product_class, sku)
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "the skip-path transfer",
        )
        events = [event[0] for event in self.events[event_mark:]]
        self.assertEqual(events, ["transfer"], "the skip path, no survey")
        self.assertFalse(self.restock_goals[-1].has_object_id, "the skip path is unpinned")

    def test_h_the_clause_covers_the_coordinators_selection_horizon(self):
        """A 150 s old rear behind a fresh vouched front still refuses the skip path."""
        # lane_01 holds nothing yet (test_a parked without a transfer), so target 1 is a deficit.
        lane = "lane_01"
        product_class, sku = self.lane_identity(lane)
        front_at = (-0.4, -0.715, 0.7)
        rear_at = (-0.4, -0.8, 0.7)
        # Phase 1: a survey leaves a reference overview with a front and a rear in one column.
        status_mark = len(self.campaign_statuses)
        transfers = max((s.successful_transfers for s in self.campaign_statuses), default=0)
        self.restock_script.append({"lane": lane, "source": "sim:h_c"})
        confirmed = self.confirmed("sim:h_c", lane)
        confirmed["overview"] = [(front_at, True), (rear_at, False)]
        self.tray_script.append(confirmed)
        self.open_deficit(lane, "sim:h_c")
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 1),
            30.0,
            "the phase-1 transfer that leaves the two-candidate reference overview",
        )
        # Phase 2: a fresh product at the vouched front, a 150 s old one behind it.
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        self.restock_script.append({"lane": lane, "source": "sim:h_front"})
        confirmed = self.confirmed("sim:h_front", lane)
        confirmed["overview"] = [(front_at, True), (rear_at, False)]
        self.tray_script.append(confirmed)
        with self.world.lock:
            self.world.lanes[lane]["target"] = 2
            self.world.add_back("sim:h_rear", product_class, sku, 150.0, position=rear_at)
            self.world.add_back("sim:h_front", product_class, sku, position=front_at)
        self._spin_until(
            lambda: self.front_full_after(status_mark, transfers + 2),
            30.0,
            "the phase-2 transfer",
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 0
        events = [event[0] for event in self.events[event_mark:]]
        self.assertEqual(events[:2], ["tray", "transfer"], "surveyed before the transfer")
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any("skip path refused" in s.detail and "sim:h_rear" in s.detail for s in window),
            f"the refusal names the old rear: {[s.detail for s in window]}",
        )
