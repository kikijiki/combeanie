# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 055: the premature-NO_COMPATIBLE_PAIR boundary recovers unless its budget is spent.

Milestone 10 §6, spec committed before this code: a genuinely exhausted state still
reports NO_COMPATIBLE_PAIR (then PHASE_STOCK_EXHAUSTED) and spends nothing. Four scripted
shapes against one campaign process at the shipped `max_ncp_resurveys = 2`. The budget is
monotone per run (like max_survey_skips), so the cases are ordered to spend it honestly —
unittest runs them alphabetically:

- test_a: STATUS_NO_COMPATIBLE_PAIR with stock remaining forces ONE tray re-survey
  (receipt on a MEASURED status, never PHASE_BLOCKED), the re-confirm refreshes the
  candidate, and the loop closes at PHASE_FRONT_FULL — the cycle-2 skip-path shape that
  ended Card 053's population red four times. Charge 1 of 2.
- test_b: stock gone at the re-measure (the coordinator refused and the class's back stock
  is empty) reports PHASE_STOCK_EXHAUSTED — the unchanged genuine-exhaustion
  classification — and spends nothing.
- test_c: the forced re-survey finds no candidate (the slot-10 fallen shape: the products
  were still counted as back stock when the sweep refused, and nothing observes them
  again) ends honestly at PHASE_STOCK_EXHAUSTED through the existing NO_CANDIDATE path,
  never a premature block. Charge 2 of 2.
- test_d: the next stock-remaining refusal finds the budget spent and latches
  PHASE_BLOCKED naming the rung and `max_ncp_resurveys`, with the run's total receipts
  never exceeding the bound.
- test_e: the refusal is answered while the only back product's evidence has aged past
  tray_evidence_validity (Card 058 increment 2, amended by review note 3: staleness is not
  evidence of absence). The product still counts (`back:1`), so the refusal is NOT
  reported as exhaustion: it takes Card 055's rung, whose budget test_d spent, so here it
  latches naming `max_ncp_resurveys` (test_a pins the re-survey itself).
- test_f: the refusal is answered after the only back product was observed fallen
  (ORIENTATION_HORIZONTAL, fresh evidence). Positive fall evidence removes it from stock:
  PHASE_STOCK_EXHAUSTED "back stock was exhausted" with `back:0`, the product still
  tracked, and the recovery budget untouched.
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
    TrackedObject,
)
from restocker_interfaces.srv import GetWorldState

RESTOCK_ACTION = "/test/ncp_recovery/restock_product"
SHELF_ACTION = "/test/ncp_recovery/survey_lane"
TRAY_ACTION = "/test/ncp_recovery/survey_tray"
RETREAT_ACTION = "/test/ncp_recovery/survey_viewpoint"
STATUS_TOPIC = "/test/ncp_recovery/coordinator_status"
CAMPAIGN_STATUS_TOPIC = "/test/ncp_recovery/status"
WORLD_STATE_SERVICE = "/test/ncp_recovery/get_snapshot"


@pytest.mark.launch_test
def generate_test_description():
    """Run the campaign at cadence zero at the shipped default recovery budget (2)."""
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name="autonomous_restock_campaign_ncp_subject",
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
                # Shipped default on purpose: the file asserts "attempt n of 2" and the
                # latch naming max_ncp_resurveys=2.
                "max_ncp_resurveys": 2,
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


class TestPrematureNcpRecovery(unittest.TestCase):
    """The Card 055 contract: forced re-survey, honest exhaustion, budget termination."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("campaign_ncp_fixture")
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
        if scripted == "no_pair_drained":
            # The coordinator refuses the pair AND the class's stock is gone by the time
            # the campaign re-measures: the genuine-exhaustion classification must win and
            # the recovery budget must stay untouched.
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            with cls.world.lock:
                cls.world.back.clear()
            goal_handle.abort()
            return result
        if scripted == "no_pair_fallen":
            # Card 058 increment 2: the refusal is answered after the back product was
            # observed fallen (fresh evidence, horizontal) — positive fall evidence, so it
            # stops counting as stock and stock_remains is false.
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            with cls.world.lock:
                for entry in cls.world.back:
                    entry["orientation"] = TrackedObject.ORIENTATION_HORIZONTAL
            goal_handle.abort()
            return result
        if scripted == "no_pair_aged":
            # Card 058 increment 2 (review note 3): the refusal is answered while the only
            # back product's evidence has aged past tray_evidence_validity — stale, not
            # gone, so it still counts as stock and stock_remains stays true.
            result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
            with cls.world.lock:
                for entry in cls.world.back:
                    entry["observed_age"] = 300.0
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
        return [s for s in cls.campaign_statuses if "forced tray re-survey attempt" in s.detail]

    def lane_identity(self, lane_id):
        """Return the (class, sku) the fixture declares for one lane."""
        lane = self.world.lanes[lane_id]
        return lane["cls"], lane["sku"]

    def _drops_back(self, source):
        with self.world.lock:
            self.world.back = [e for e in self.world.back if e["source"] != source]

    # -- the four cases ------------------------------------------------------

    def test_a_forced_resurvey_recovers_the_skip_path(self):
        """NCP with stock remaining forces one re-survey and closes at FRONT_FULL."""
        status_mark = len(self.campaign_statuses)
        event_mark = len(self.events)
        lane = "lane_01"
        product_class, sku = self.lane_identity(lane)
        # Scripts land BEFORE the world opens the deficit: the loop wakes on the world
        # change alone, so a goal could otherwise beat the script append (and at
        # cycle_period 0 the wake is immediate).
        self.restock_script.extend(["no_pair", {"lane": lane, "source": "sim:recovery_a"}])
        self.tray_script.append(
            {
                "outcome": "confirmed",
                "source": "sim:recovery_a",
                "cls": product_class,
                "sku": sku,
            }
        )
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_a", product_class, sku)
        self._spin_until(
            lambda: any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_FRONT_FULL
                and s.successful_transfers == 1
                for s in self.campaign_statuses[status_mark:]
            ),
            30.0,
            "PHASE_FRONT_FULL after the recovered transfer",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_MEASURED
                and "forced tray re-survey attempt 1 of 2" in s.detail
                for s in window
            ),
            f"the recovery must be receipted on a measured status: {[s.detail for s in window]}",
        )
        self.assertFalse(
            any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "premature NO_COMPATIBLE_PAIR" in s.detail
                for s in window
            ),
            "a recoverable premature NCP must never publish PHASE_BLOCKED",
        )
        events = [event[0] for event in self.events[event_mark:]]
        self.assertEqual(
            events,
            ["transfer", "tray", "transfer"],
            "skip-path refusal, then the forced re-survey, then the re-confirmed transfer",
        )

    def test_b_genuine_exhaustion_spends_no_budget(self):
        """Stock gone at the measurement: STOCK_EXHAUSTED as before, zero new charges."""
        status_mark = len(self.campaign_statuses)
        charges_before = len(self._receipts())
        lane = "lane_02"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("no_pair_drained")
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_b", product_class, sku)
        # Restore the deficit on the SAME poll that first sees the terminal: the next
        # cycle (50 ms backoff) would otherwise idle forever on a deficit with no stock.
        exhausted = False

        def genuinely_exhausted():
            nonlocal exhausted
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                and "back stock was exhausted" in s.detail
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not exhausted:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                exhausted = True
            return hit

        self._spin_until(
            genuinely_exhausted,
            20.0,
            "PHASE_STOCK_EXHAUSTED for the genuinely exhausted classification",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertEqual(
            len(self._receipts()),
            charges_before,
            "genuine exhaustion must not spend the recovery budget",
        )
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED for s in window),
            "genuine exhaustion idles at STOCK_EXHAUSTED, never blocked",
        )

    def test_c_unseen_stock_ends_honest_not_premature(self):
        """NCP whose forced re-survey finds no candidate → STOCK_EXHAUSTED, one charge."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_03"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("no_pair")
        self.tray_script.append({"outcome": "no_candidate"})
        with self.world.lock:
            # Still counted as back stock (Tracked+Free), but the tray survey — the
            # camera's stand-in — sees nothing: the slot-10 fallen shape.
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_c", product_class, sku)
        unseen = False

        def stock_unseen():
            nonlocal unseen
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not unseen:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                unseen = True
            return hit

        self._spin_until(
            stock_unseen,
            20.0,
            "PHASE_STOCK_EXHAUSTED once the forced re-survey finds no candidate",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertTrue(
            any("forced tray re-survey attempt 2 of 2" in s.detail for s in window),
            f"the boundary charged its second attempt before the survey: "
            f"{[s.detail for s in window]}",
        )
        blocked = [
            (s.phase, s.detail)
            for s in window
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
        ]
        self.assertFalse(
            blocked,
            "unseen stock ends at STOCK_EXHAUSTED through NO_CANDIDATE, never blocked: "
            f"{blocked} window={[(s.phase, s.detail) for s in window]} "
            f"events={self.events}",
        )

    def test_d_budget_latches_naming_rung_and_bound(self):
        """The next stock-remaining refusal with the budget spent latches, naming both."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_04"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("no_pair")
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_d", product_class, sku)
        latched = False

        def budget_latched():
            nonlocal latched
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "max_ncp_resurveys" in s.detail
                and "forced tray re-survey rung" in s.detail
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not latched:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                latched = True
            return hit

        self._spin_until(
            budget_latched,
            30.0,
            "PHASE_BLOCKED naming the forced tray re-survey rung and max_ncp_resurveys",
        )
        blocked = next(
            s
            for s in self.campaign_statuses[status_mark:]
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
            and "max_ncp_resurveys" in s.detail
        )
        self.assertIn("max_ncp_resurveys=2", blocked.detail)
        self.assertIn("premature NO_COMPATIBLE_PAIR", blocked.detail)
        # The run's total spend, across all four cases: the bound is per run, not per
        # occurrence, and never exceeds max_ncp_resurveys.
        receipts = self._receipts()
        self.assertEqual(
            len(receipts),
            2,
            f"exactly max_ncp_resurveys receipts for the whole run: "
            f"{[s.detail for s in receipts]}",
        )
        self.assertTrue(
            any("attempt 1 of 2" in s.detail for s in receipts)
            and any("attempt 2 of 2" in s.detail for s in receipts),
            f"each receipt names its attempt against the bound: {[s.detail for s in receipts]}",
        )
        self.assertFalse(
            self.restock_script,
            f"the latch consumed exactly one further refusal: {list(self.restock_script)}",
        )

    def test_e_aged_back_product_still_counts_as_stock(self):
        """Review note 3: stale-but-unrefuted stock is not exhaustion; Card 055's rung decides."""
        status_mark = len(self.campaign_statuses)
        lane = "lane_05"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("no_pair_aged")
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_e", product_class, sku)
        latched = False

        def rung_decided():
            nonlocal latched
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
                and "max_ncp_resurveys" in s.detail
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not latched:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                self._drops_back("sim:recovery_e")
                latched = True
            return hit

        self._spin_until(
            rung_decided,
            20.0,
            "Card 055's rung (budget spent by test_d) deciding the stale-stock refusal",
        )
        window = self.campaign_statuses[status_mark:]
        blocked = next(
            s
            for s in window
            if s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED
            and "max_ncp_resurveys" in s.detail
        )
        slot = list(blocked.product_classes).index(product_class)
        self.assertEqual(
            blocked.back_stock[slot],
            1,
            "the stale product is stale, not gone: it still receipts as back:1",
        )
        self.assertFalse(
            any("back stock was exhausted" in s.detail for s in window),
            "stale-but-unrefuted stock must never be reported as exhausted: "
            f"{[s.detail for s in window]}",
        )

    def test_f_fallen_back_product_stops_counting_as_stock(self):
        """Card 058 increment 2: an observed fall removes the product from stock at no charge."""
        status_mark = len(self.campaign_statuses)
        charges_before = len(self._receipts())
        lane = "lane_06"
        product_class, sku = self.lane_identity(lane)
        self.restock_script.append("no_pair_fallen")
        with self.world.lock:
            self.world.lanes[lane]["target"] = 1
            self.world.add_back("sim:recovery_f", product_class, sku)
        exhausted = False

        def fallen_stock_exhausted():
            nonlocal exhausted
            hit = any(
                s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
                and "back stock was exhausted" in s.detail
                for s in self.campaign_statuses[status_mark:]
            )
            if hit and not exhausted:
                with self.world.lock:
                    self.world.lanes[lane]["target"] = 0
                exhausted = True
            return hit

        self._spin_until(
            fallen_stock_exhausted,
            20.0,
            "PHASE_STOCK_EXHAUSTED once the fallen product stops counting as stock",
        )
        window = self.campaign_statuses[status_mark:]
        self.assertEqual(
            len(self._receipts()),
            charges_before,
            "fallen-only stock must not spend the recovery budget",
        )
        park = next(
            s
            for s in window
            if s.phase == AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED
            and "back stock was exhausted" in s.detail
        )
        slot = list(park.product_classes).index(product_class)
        self.assertEqual(
            park.back_stock[slot],
            0,
            "the fallen product is still tracked but must receipt as back:0",
        )
        self.assertTrue(
            any(entry["source"] == "sim:recovery_f" for entry in self.world.back),
            "the product must remain tracked in the snapshot; only its stock count changes",
        )
        self.assertFalse(
            any(s.phase == AutonomousRestockCampaignStatus.PHASE_BLOCKED for s in window),
            "genuine exhaustion idles at STOCK_EXHAUSTED, never blocked",
        )
