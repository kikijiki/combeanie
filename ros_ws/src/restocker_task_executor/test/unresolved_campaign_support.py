# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Shared fakes for the Card 086 stage 1 unresolved-motion launch tests (CMB-SPEC-13).

Every action boundary is a scripted fake and every send is counted at the fake server, never read
from a reported state. Stage 1 fences the campaign only: the fakes are reachable through the
campaign alone, so none of these tests claims cross-endpoint exclusion.
"""

import threading
import time
import unittest

from action_msgs.msg import GoalStatus
from campaign_loop_support import FakeWorld, fixture_io_callback_group, pitch_by_sku
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct, SurveyLane, SurveyTray, SurveyViewpoint
from restocker_interfaces.msg import AutonomousRestockCampaignStatus, RestockCoordinatorStatus
from restocker_interfaces.srv import GetWorldState

Status = AutonomousRestockCampaignStatus
SENDS = {"shelf", "tray", "restock", "retreat_goal"}


class ScriptableResultServer(ActionServer):
    """
    An action server that can answer every result request with STATUS_UNKNOWN.

    That is what rclcpp_action's peer reports when a result request finds no registered goal
    (an overtaken admission, an expired or restarted server): it says nothing about the goal,
    which may still be running. The default is the stock behavior.
    """

    answer_unknown = False

    async def _execute_get_result_request(self, request_header_and_message):
        if not self.answer_unknown:
            return await super()._execute_get_result_request(request_header_and_message)
        request_header, _request = request_header_and_message
        response = self._action_type.Impl.GetResultService.Response()
        response.status = GoalStatus.STATUS_UNKNOWN
        with self._lock:
            self._handle.send_result_response(request_header, response)
        return None


def unresolved(message):
    """Return the status's explicit motion_unresolved marker."""
    return message.motion_unresolved


def attempts(message):
    """Return the retained unresolved-attempt identities of one status."""
    return list(message.unresolved_attempts)


def make_description(prefix, node_name, **overrides):
    """Describe one campaign process wired to the fakes under ``prefix``."""
    parameters = {
        "use_sim_time": False,
        "workcell_geometry_path": PathJoinSubstitution(
            [FindPackageShare("restocker_description"), "config", "workcell_geometry.yaml"]
        ),
        "product_catalog_path": PathJoinSubstitution(
            [FindPackageShare("restocker_description"), "config", "product_collision_catalog.yaml"]
        ),
        "restock_action_name": f"{prefix}/restock_product",
        "shelf_survey_action_name": f"{prefix}/survey_lane",
        "tray_survey_action_name": f"{prefix}/survey_tray",
        "recovery_viewpoint_action_name": f"{prefix}/survey_viewpoint",
        "recovery_safe_station": "tray_1",
        "coordinator_status_topic": f"{prefix}/coordinator_status",
        "campaign_status_topic": f"{prefix}/status",
        "world_state_service_name": f"{prefix}/get_snapshot",
        "max_cycles": 0,
        "cycle_period_sec": 0.0,
        "failed_cycle_backoff_sec": 0.05,
        "server_wait_timeout_sec": 10.0,
        "action_timeout_sec": 1.0,
        "restart_acknowledged": True,
    }
    parameters.update(overrides)
    campaign = Node(
        package="restocker_task_executor",
        executable="autonomous_restock_campaign",
        name=node_name,
        output="screen",
        parameters=[parameters],
    )
    return launch.LaunchDescription([campaign, launch_testing.actions.ReadyToTest()])


class UnresolvedCampaignFixture(unittest.TestCase):
    """Scriptable fakes for all four campaign endpoints, the world snapshot and the status."""

    PREFIX = ""
    NODE_NAME = ""
    # Lane targets: 0 parks the campaign at FRONT_FULL once every lane is surveyed.
    LANE_TARGET = 0

    @classmethod
    def setUpClass(cls):
        prefix = cls.PREFIX
        rclpy.init()
        cls.node = rclpy.create_node(cls.NODE_NAME + "_fixture")
        cls.executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()
        cls.lock = threading.Lock()
        cls.events = []
        cls.statuses = []
        # Per-endpoint behaviors, consumed in order; an empty script means the default.
        cls.shelf_script = {}
        cls.tray_script = []
        cls.restock_script = []
        # "reject" refuses every restock goal; "late_reject" holds the admission reply
        # restock_admission_hang_sec and then refuses; anything else uses restock_script.
        cls.restock_mode = "script"
        cls.restock_admission_hang_sec = 0.0
        cls.retreat_admission_hang_sec = 0.0
        cls.retreat_cancel_stop = True
        cls.world = FakeWorld(pitch_by_sku())
        with cls.world.lock:
            for lane in cls.world.lanes.values():
                lane["target"] = cls.LANE_TARGET

        def cancel_ok(_handle):
            return CancelResponse.ACCEPT

        cls.shelf_server = ActionServer(
            cls.node,
            SurveyLane,
            f"{prefix}/survey_lane",
            execute_callback=cls._survey_lane,
            goal_callback=cls._shelf_goal,
            cancel_callback=cancel_ok,
        )
        cls.tray_cancel_requests = []

        def tray_cancel(handle):
            cls._record("tray_cancel_request", bytes(handle.goal_id.uuid).hex())
            return CancelResponse.ACCEPT

        cls.tray_server = ScriptableResultServer(
            cls.node,
            SurveyTray,
            f"{prefix}/survey_tray",
            execute_callback=cls._survey_tray,
            goal_callback=cls._tray_goal,
            cancel_callback=tray_cancel,
        )
        # The restock endpoint has its own group so a held admission reply stalls nothing else.
        cls.restock_group = MutuallyExclusiveCallbackGroup()
        cls.restock_server = ActionServer(
            cls.node,
            RestockProduct,
            f"{prefix}/restock_product",
            execute_callback=cls._restock,
            goal_callback=cls._restock_goal,
            cancel_callback=cancel_ok,
            callback_group=cls.restock_group,
        )
        # The viewpoint child the tray fake delegates to (the retreat endpoint's server).
        cls.child_client = ActionClient(cls.node, SurveyViewpoint, f"{prefix}/survey_viewpoint")
        # The retreat endpoint has its own group so a held admission reply stalls nothing else.
        cls.retreat_group = MutuallyExclusiveCallbackGroup()
        cls.retreat_server = ActionServer(
            cls.node,
            SurveyViewpoint,
            f"{prefix}/survey_viewpoint",
            execute_callback=cls._retreat,
            goal_callback=cls._retreat_goal,
            cancel_callback=cancel_ok,
            callback_group=cls.retreat_group,
        )
        cls.world_state_service = cls.node.create_service(
            GetWorldState,
            f"{prefix}/get_snapshot",
            cls._world_state,
            callback_group=fixture_io_callback_group(),
        )
        cls.status_publisher = cls.node.create_publisher(
            RestockCoordinatorStatus,
            f"{prefix}/coordinator_status",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        cls.subscription = cls.node.create_subscription(
            Status,
            f"{prefix}/status",
            cls._on_status,
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
        for server in (cls.shelf_server, cls.tray_server, cls.restock_server, cls.retreat_server):
            server.destroy()
        cls.child_client.destroy()
        cls.node.destroy_service(cls.world_state_service)
        cls.node.destroy_subscription(cls.subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    # -- recording -----------------------------------------------------------------------------

    @classmethod
    def _record(cls, *event):
        with cls.lock:
            cls.events.append((time.monotonic(), *event))

    @classmethod
    def _on_status(cls, message):
        with cls.lock:
            cls.statuses.append((time.monotonic(), message))

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

    # -- fakes ---------------------------------------------------------------------------------

    @classmethod
    def _shelf_goal(cls, goal_request):
        cls._record("shelf", goal_request.lane_id)
        return GoalResponse.ACCEPT

    @classmethod
    def _survey_lane(cls, goal_handle):
        lane_id = goal_handle.request.lane_id
        script = cls.shelf_script.get(lane_id)
        behavior = script.pop(0) if script else "ok"
        result = SurveyLane.Result()
        result.lane_id = lane_id
        if behavior == "fail_exec":
            # A recoverable failure that reached a terminal stop: the rung-1 retreat follows.
            result.outcome = SurveyLane.Result.OUTCOME_VIEWPOINT_UNREACHED
            result.detail = "scripted: execution failed at a terminal stop"
            result.execution_reached_terminal_stop = True
        else:
            cls.world.survey_lane(lane_id)
            result.outcome = SurveyLane.Result.OUTCOME_OBSERVED
        goal_handle.succeed()
        return result

    @classmethod
    def _tray_goal(cls, _request):
        cls._record("tray")
        return GoalResponse.ACCEPT

    @classmethod
    def _survey_tray(cls, goal_handle):
        goal_hex = bytes(goal_handle.goal_id.uuid).hex()
        behavior = cls.tray_script.pop(0) if cls.tray_script else "no_candidate"
        result = SurveyTray.Result()
        if behavior in {"child_withheld", "child_withheld_unavailable"}:
            # The real tray survey delegates each leg to a viewpoint child. Its admission is slow
            # (the retreat endpoint's admission hang), so when the campaign cancels the parent
            # the child may still be admitted later while the parent returns with its
            # untouched pre-motion default motion_definitely_not_started=true.
            cls.child_client.wait_for_server(timeout_sec=5.0)
            child = SurveyViewpoint.Goal()
            child.station = "tray_1"
            cls.child_client.send_goal_async(child)
            deadline = time.monotonic() + 20.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.02)
            cls._record("tray_cancel", goal_hex)
            result.motion_definitely_not_started = True
            result.execution_reached_terminal_stop = False
            if behavior == "child_withheld":
                result.outcome = SurveyTray.Result.OUTCOME_CANCELED
                result.detail = "scripted: canceled while the child leg was unadmitted"
                goal_handle.canceled()
            else:
                # The 5 s send-grace expiry shape: UNAVAILABLE delivered with code SUCCEEDED.
                result.outcome = SurveyTray.Result.OUTCOME_UNAVAILABLE
                result.detail = "scripted: the child server did not accept the goal in time"
                goal_handle.succeed()
            cls._record("tray_terminal", goal_hex)
            return result
        if behavior == "hang":
            # Hold the result until the campaign's timeout cancels this exact goal, then answer
            # with a canceled terminal that carries neither non-start nor stop evidence.
            deadline = time.monotonic() + 20.0
            while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
                time.sleep(0.02)
            if goal_handle.is_cancel_requested:
                cls._record("tray_cancel", goal_hex)
                result.outcome = SurveyTray.Result.OUTCOME_CANCELED
                result.detail = "scripted: canceled without stop evidence"
                goal_handle.canceled()
                return result
            goal_handle.abort()
            return result
        if behavior == "unavailable":
            # A delivered terminal with neither non-start nor stop evidence (SPEC-13 S3).
            result.outcome = SurveyTray.Result.OUTCOME_UNAVAILABLE
            result.detail = "scripted: unavailable, no evidence flags"
            goal_handle.succeed()
            return result
        result.outcome = SurveyTray.Result.OUTCOME_NO_CANDIDATE
        result.detail = "scripted: no candidate"
        goal_handle.succeed()
        return result

    @classmethod
    def _restock_goal(cls, _request):
        cls._record("restock")
        if cls.restock_mode == "late_reject":
            time.sleep(cls.restock_admission_hang_sec)
            cls._record("restock_late_reject")
            return GoalResponse.REJECT
        if cls.restock_mode == "reject":
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @classmethod
    def _restock(cls, goal_handle):
        behavior = cls.restock_script.pop(0) if cls.restock_script else "no_pair"
        result = RestockProduct.Result()
        if behavior == "execution_failed":
            # A delivered terminal failure that carries no non-start or stop evidence.
            result.status = RestockProduct.Result.STATUS_EXECUTION_FAILED
            result.detail = "scripted: execution failed, no stop evidence"
            goal_handle.abort()
            return result
        if behavior == "planning_failed_not_started":
            # The coordinator proves nothing was commanded (a first-leg plan failure).
            result.status = RestockProduct.Result.STATUS_PLANNING_FAILED
            result.detail = "scripted: first-leg planning failed before any command"
            result.motion_definitely_not_started = True
            goal_handle.abort()
            return result
        if behavior == "execution_failed_stopped":
            result.status = RestockProduct.Result.STATUS_EXECUTION_FAILED
            result.detail = "scripted: controller aborted, terminal stop observed"
            result.execution_reached_terminal_stop = True
            goal_handle.abort()
            return result
        if behavior == "planning_failed_unproven":
            # The whole-task deadline after segments executed: PLANNING_FAILED with no proof.
            result.status = RestockProduct.Result.STATUS_PLANNING_FAILED
            result.detail = "scripted: whole-task steady deadline exceeded"
            goal_handle.abort()
            return result
        result.status = RestockProduct.Result.STATUS_NO_COMPATIBLE_PAIR
        goal_handle.abort()
        return result

    @classmethod
    def _retreat_goal(cls, _request):
        # Hold the admission reply past action_timeout_sec: the goal is in flight while the
        # campaign has no handle for it (Card 052 review F1, the S21 race).
        hang = cls.retreat_admission_hang_sec
        if hang > 0:
            time.sleep(hang)
        return GoalResponse.ACCEPT

    @classmethod
    def _retreat(cls, goal_handle):
        goal_hex = bytes(goal_handle.goal_id.uuid).hex()
        cls._record("retreat_goal", goal_hex)
        deadline = time.monotonic() + 15.0
        while not goal_handle.is_cancel_requested and time.monotonic() < deadline:
            time.sleep(0.02)
        result = SurveyViewpoint.Result()
        if goal_handle.is_cancel_requested:
            cls._record("retreat_cancel", goal_hex)
            result.outcome = SurveyViewpoint.Result.OUTCOME_CANCELED
            result.execution_reached_terminal_stop = cls.retreat_cancel_stop
            cls._record("retreat_terminal", goal_hex)
            goal_handle.canceled()
            return result
        result.outcome = SurveyViewpoint.Result.OUTCOME_ARRIVED
        result.execution_reached_terminal_stop = True
        cls._record("retreat_terminal", goal_hex)
        goal_handle.succeed()
        return result

    # -- helpers -------------------------------------------------------------------------------

    def snapshot_events(self):
        with self.lock:
            return list(self.events)

    def snapshot_statuses(self):
        with self.lock:
            return list(self.statuses)

    def sends(self):
        """Every goal any fake endpoint admitted or was asked to admit, in order."""
        return [e for e in self.snapshot_events() if e[1] in SENDS]

    def spin_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self._publish_ready()
            time.sleep(0.05)
            if predicate():
                return
        phases = [(m.phase, unresolved(m), m.detail) for _, m in self.snapshot_statuses()]
        self.fail(
            f"timed out waiting for {description}; events={self.snapshot_events()} "
            f"phases={phases[-12:]}"
        )
