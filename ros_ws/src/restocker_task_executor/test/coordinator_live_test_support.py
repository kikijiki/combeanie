# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Shared client fixture for live coordinator shutdown launch tests."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import time
from typing import TypeVar

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import (
    LaneObservation,
    ObjectObservation,
    RestockCoordinatorStatus,
    RobotTelemetry,
    TaskReservation,
)
from restocker_interfaces.srv import (
    GetWorldState,
    ReleaseTaskReservation,
    ReserveTask,
    ValidateTaskReservation,
)

_Result = TypeVar("_Result")
SOURCE_OBJECT_ID = "test:shutdown_can"
LANE_ID = "lane_01"
# Every lane the world state is configured with, because it only reports a lane once that
# lane has evidence: publishing for a subset leaves the remaining lanes absent and the
# readiness wait below never completes.
LANE_IDS = ("lane_01", "lane_02", "lane_03", "lane_04", "lane_05", "lane_06")
RELEASE_SERVICE_TYPE = "restocker_interfaces/srv/ReleaseTaskReservation"
RESERVE_SERVICE_TYPE = "restocker_interfaces/srv/ReserveTask"
VALIDATE_SERVICE_TYPE = "restocker_interfaces/srv/ValidateTaskReservation"


@dataclass(frozen=True)
class CapturedReservation:
    """Public reservation identity retained independently of mutable ROS messages."""

    reservation_id: int
    request_id: str
    object_id: int
    object_source_id: str
    product_class: int
    has_sku: bool
    sku: str
    has_source_lane: bool
    source_lane_id: str
    destination_lane_id: str
    stage: int
    placed_in_destination: bool
    created_at_sec: int
    created_at_nanosec: int
    created_revision: int
    admitted_robot_telemetry_revision: int
    revision: int

    @classmethod
    def from_message(cls, message: TaskReservation) -> CapturedReservation:
        """Copy the fields required for the post-exit authority proof."""
        return cls(
            reservation_id=message.reservation_id,
            request_id=message.request_id,
            object_id=message.object_id,
            object_source_id=message.object_source_id,
            product_class=message.product_class,
            has_sku=message.has_sku,
            sku=message.sku,
            has_source_lane=message.has_source_lane,
            source_lane_id=message.source_lane_id,
            destination_lane_id=message.destination_lane_id,
            stage=message.stage,
            placed_in_destination=message.placed_in_destination,
            created_at_sec=message.created_at.sec,
            created_at_nanosec=message.created_at.nanosec,
            created_revision=message.created_revision,
            admitted_robot_telemetry_revision=message.admitted_robot_telemetry_revision,
            revision=message.revision,
        )


class LiveShutdownClient:
    """Drive one coordinator while observing the independent world-state process."""

    def __init__(self, node_name: str, action_name: str, status_topic: str) -> None:
        self.node: Node = rclpy.create_node(node_name)
        self._coordinator_status = None
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_subscription = self.node.create_subscription(
            RestockCoordinatorStatus,
            status_topic,
            self._record_coordinator_status,
            status_qos,
        )
        self._object_publisher = self.node.create_publisher(
            ObjectObservation,
            "/perception/object_observations",
            10,
        )
        self._lane_publisher = self.node.create_publisher(
            LaneObservation,
            "/perception/ground_truth/lane_observations",
            10,
        )
        self._telemetry_publisher = self.node.create_publisher(
            RobotTelemetry,
            "/robot_telemetry",
            10,
        )
        self._snapshot_client = self.node.create_client(
            GetWorldState,
            "/world_state/get_snapshot",
        )
        self._action_client = ActionClient(self.node, RestockProduct, action_name)
        self._feedback_trace: list[tuple[int, str]] = []
        self._result_future = None
        self._fake_release_service = None
        self._authority_guard_services = []
        self.authority_guard_calls = {"reserve": 0, "validate": 0, "release": 0}
        self.fake_release_calls = 0

    def destroy(self) -> None:
        """Destroy test-owned graph entities without touching launched processes."""
        if self._fake_release_service is not None:
            self.node.destroy_service(self._fake_release_service)
            self._fake_release_service = None
        for service in self._authority_guard_services:
            self.node.destroy_service(service)
        self._authority_guard_services.clear()
        self.node.destroy_subscription(self._status_subscription)
        self._action_client.destroy()
        self.node.destroy_node()

    def spin_until(
        self,
        predicate: Callable[[], _Result | None | bool],
        timeout_sec: float,
        failure: str,
    ) -> _Result:
        """Evaluate a predicate while servicing ROS work under a steady deadline."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            result = predicate()
            if result is not None and result is not False:
                return result
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise AssertionError(failure)

    def seed_authority(self):
        """Publish one exact evidence pair and return its authoritative snapshot."""
        self.spin_until(
            lambda: (
                self._object_publisher.get_subscription_count() > 0
                and self._lane_publisher.get_subscription_count() > 0
                and self._telemetry_publisher.get_subscription_count() > 0
            ),
            5.0,
            "world-state evidence and telemetry subscriptions were not discovered",
        )

        stamp = self.node.get_clock().now().to_msg()
        telemetry = RobotTelemetry()
        telemetry.stamp = stamp
        telemetry.source_id = "shutdown_integration_fixture"
        telemetry.joint_names = [
            RobotTelemetry.JOINT_1_NAME,
            RobotTelemetry.JOINT_2_NAME,
            RobotTelemetry.JOINT_3_NAME,
            RobotTelemetry.JOINT_4_NAME,
            RobotTelemetry.JOINT_5_NAME,
            RobotTelemetry.JOINT_6_NAME,
        ]
        telemetry.joint_positions = [0.0] * 6
        telemetry.joint_velocities = [0.0] * 6
        telemetry.rail_position = 0.0
        telemetry.rail_velocity = 0.0
        telemetry.gripper_joint_names = [
            RobotTelemetry.LEFT_FINGER_JOINT_NAME,
            RobotTelemetry.RIGHT_FINGER_JOINT_NAME,
        ]
        telemetry.gripper_joint_positions = [0.0, 0.0]
        telemetry.gripper_joint_velocities = [0.0, 0.0]

        object_observation = ObjectObservation()
        object_observation.header.stamp = stamp
        object_observation.header.frame_id = "world"
        object_observation.source_object_id = SOURCE_OBJECT_ID
        object_observation.product_class = ObjectObservation.PRODUCT_CLASS_CAN
        object_observation.has_sku = True
        object_observation.sku = "SIM-CAN-STD"
        object_observation.pose.pose.position.x = -0.25
        object_observation.pose.pose.position.y = -0.80
        object_observation.pose.pose.position.z = 0.65
        object_observation.pose.pose.orientation.w = 1.0
        for index in range(6):
            object_observation.pose.covariance[index * 6 + index] = 1.0e-8
        object_observation.orientation = ObjectObservation.ORIENTATION_UPRIGHT
        object_observation.confidence = 1.0
        object_observation.backend_name = "shutdown_integration_fixture"
        object_observation.backend_version = "1"
        object_observation.status = ObjectObservation.STATUS_OK

        self._telemetry_publisher.publish(telemetry)
        self._object_publisher.publish(object_observation)
        for lane_id in LANE_IDS:
            lane_observation = LaneObservation()
            lane_observation.header.stamp = stamp
            lane_observation.header.frame_id = lane_id
            lane_observation.lane_id = lane_id
            lane_observation.available_depth_m = 0.85
            lane_observation.obstructed = False
            lane_observation.confidence = 1.0
            lane_observation.backend_name = "shutdown_integration_fixture"
            lane_observation.backend_version = "1"
            lane_observation.status = LaneObservation.STATUS_OK
            self._lane_publisher.publish(lane_observation)

        def seeded_snapshot():
            snapshot = self.snapshot()
            objects = [
                item for item in snapshot.objects if item.source_object_id == SOURCE_OBJECT_ID
            ]
            observed_lanes = {item.id for item in snapshot.lanes if item.evidence_revision > 0}
            robot_time_matches = (
                snapshot.robot.telemetry_time.sec == stamp.sec
                and snapshot.robot.telemetry_time.nanosec == stamp.nanosec
                and snapshot.robot.telemetry_revision > 0
            )
            robot_state_matches = (
                snapshot.robot.revision > 0
                and list(snapshot.robot.joint_positions) == [0.0] * 6
                and snapshot.robot.rail_position == 0.0
                and list(snapshot.robot.gripper_joint_positions) == [0.0, 0.0]
                and robot_time_matches
            )
            return (
                snapshot
                if len(objects) == 1 and observed_lanes == set(LANE_IDS) and robot_state_matches
                else None
            )

        return self.spin_until(
            seeded_snapshot,
            5.0,
            "world state did not retain the expected object, lane, and robot evidence",
        )

    def snapshot(self):
        """Obtain one response from the real world-state snapshot service."""
        # Callers poll this inside spin_until, which tolerates a world that is not ready yet, but
        # a 0.2 s service probe or 1 s call budget raised instead of returning, so one transient
        # blip during early discovery aborted the test. A healthy snapshot answers in milliseconds;
        # a genuinely absent world state still fails, just later.
        if not self._snapshot_client.wait_for_service(timeout_sec=5.0):
            raise AssertionError("world-state snapshot service is unavailable")
        future = self._snapshot_client.call_async(GetWorldState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise AssertionError("world-state snapshot request did not complete")
        return future.result().snapshot

    def wait_for_admission_ready(self, timeout_sec: float = 10.0) -> None:
        """Wait for the coordinator to publish that it can admit work."""
        # wait_for_server returns before the coordinator has probed the authority and the
        # world-state services. A goal sent in that window is answered while discovery is still
        # one-sided and rclcpp_action drops the reply ("Failed to send goal response (timeout)"),
        # so the client waits forever. The status topic is TRANSIENT_LOCAL, so a late subscriber
        # still gets the latest sample and this wait cannot miss the transition.
        self.spin_until(
            lambda: (
                self._coordinator_status is not None and self._coordinator_status.admission_ready
            ),
            timeout_sec,
            "coordinator never published an admission-ready status",
        )

    def send_until_accepted(self, seeded_snapshot, timeout_sec: float = 5.0):
        """Retry side-effect-free admission until one goal owns generation one."""
        self.wait_for_admission_ready()
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            raise AssertionError("coordinator action server is unavailable")
        selected = next(
            item for item in seeded_snapshot.objects if item.source_object_id == SOURCE_OBJECT_ID
        )
        goal = RestockProduct.Goal()
        goal.has_object_id = True
        goal.object_id = selected.id
        goal.has_lane_id = True
        goal.lane_id = LANE_ID

        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            sent = self._action_client.send_goal_async(
                goal,
                feedback_callback=self._record_feedback,
            )
            rclpy.spin_until_future_complete(self.node, sent, timeout_sec=1.0)
            if not sent.done() or sent.result() is None:
                raise AssertionError("goal-admission request did not complete")
            goal_handle = sent.result()
            if goal_handle.accepted:
                self._result_future = goal_handle.get_result_async()
                return goal_handle
            if self.snapshot().has_active_reservation:
                raise AssertionError("rejected goal produced an authoritative side effect")
            rclpy.spin_once(self.node, timeout_sec=0.05)
        raise AssertionError("coordinator never admitted the valid goal")

    def wait_for_reservation(self, timeout_sec: float = 5.0) -> CapturedReservation:
        """Wait for feedback and the matching real authoritative reservation."""

        def staged_reservation():
            if self._result_future is not None and self._result_future.done():
                response = self._result_future.result()
                if response is not None:
                    raise AssertionError(
                        "goal terminated before reservation staging: "
                        f"protocol_status={response.status} "
                        f"payload_status={response.result.status} "
                        f"detail={response.result.detail!r}"
                    )
            snapshot = self.snapshot()
            if (
                RestockProduct.Feedback.STATE_GENERATE_GRASPS
                not in [state for state, _ in self._feedback_trace]
                or not snapshot.has_active_reservation
            ):
                return None
            reservation = snapshot.active_reservation
            if (
                reservation.object_source_id != SOURCE_OBJECT_ID
                or reservation.destination_lane_id != LANE_ID
                or reservation.stage != TaskReservation.STAGE_RESERVED
            ):
                raise AssertionError("world state retained an unexpected reservation")
            return CapturedReservation.from_message(reservation)

        return self.spin_until(
            staged_reservation,
            timeout_sec,
            "coordinator did not reach the exact reservation staging boundary; "
            f"feedback={self._feedback_trace}",
        )

    def wait_for_result(self, timeout_sec: float = 5.0):
        """Wait for the accepted goal's terminal action response."""
        if self._result_future is None:
            raise AssertionError("no accepted goal owns a result future")
        rclpy.spin_until_future_complete(
            self.node,
            self._result_future,
            timeout_sec=timeout_sec,
        )
        if not self._result_future.done() or self._result_future.result() is None:
            raise AssertionError("action result did not complete")
        return self._result_future.result()

    def assert_no_valid_result(self) -> None:
        """Reject any coordinator-published terminal result for the timeout case."""
        if self._result_future is None or not self._result_future.done():
            return
        try:
            result = self._result_future.result()
        except Exception:  # noqa: BLE001 - transport teardown is not an action result.
            return
        if result is not None:
            raise AssertionError("coordinator published a terminal result without release proof")

    def advertise_fake_release(self, service_name: str) -> None:
        """Advertise a release endpoint that must be removed before it is used."""
        if self._fake_release_service is not None:
            raise AssertionError("fake release service is already advertised")

        def unexpected_release(request, response):
            del request
            self.fake_release_calls += 1
            response.status.code = response.status.INTERNAL_ERROR
            response.status.detail = "fixture release service must be removed before shutdown"
            return response

        self._fake_release_service = self.node.create_service(
            ReleaseTaskReservation,
            service_name,
            unexpected_release,
        )
        self.spin_until(
            lambda: self._service_has_type(service_name, RELEASE_SERVICE_TYPE),
            5.0,
            "fake release service was not visible in the ROS graph",
        )

    def withdraw_fake_release(self, service_name: str) -> None:
        """Destroy the fake endpoint before the coordinator enters shutdown."""
        if self._fake_release_service is None:
            raise AssertionError("fake release service is not advertised")
        if not self.node.destroy_service(self._fake_release_service):
            raise AssertionError("fake release service could not be destroyed")
        self._fake_release_service = None
        del service_name

    def advertise_authority_guards(
        self,
        reserve_service: str,
        validate_service: str,
        release_service: str,
    ) -> None:
        """Expose counted endpoints that a fail-closed replacement must never call."""
        if self._authority_guard_services:
            raise AssertionError("authority guards are already advertised")

        def unexpected_reserve(request, response):
            del request
            self.authority_guard_calls["reserve"] += 1
            response.status.code = response.status.INTERNAL_ERROR
            response.status.detail = "replacement must not reserve during startup"
            return response

        def unexpected_validate(request, response):
            del request
            self.authority_guard_calls["validate"] += 1
            response.status.code = response.status.INTERNAL_ERROR
            response.status.detail = "replacement must not validate guessed authority"
            return response

        def unexpected_release(request, response):
            del request
            self.authority_guard_calls["release"] += 1
            response.status.code = response.status.INTERNAL_ERROR
            response.status.detail = "replacement must not release guessed authority"
            return response

        self._authority_guard_services = [
            self.node.create_service(ReserveTask, reserve_service, unexpected_reserve),
            self.node.create_service(
                ValidateTaskReservation,
                validate_service,
                unexpected_validate,
            ),
            self.node.create_service(
                ReleaseTaskReservation,
                release_service,
                unexpected_release,
            ),
        ]
        expected_types = {
            reserve_service: RESERVE_SERVICE_TYPE,
            validate_service: VALIDATE_SERVICE_TYPE,
            release_service: RELEASE_SERVICE_TYPE,
        }
        self.spin_until(
            lambda: all(
                self._service_has_type(name, service_type)
                for name, service_type in expected_types.items()
            ),
            5.0,
            "replacement authority guard services were not visible in the ROS graph",
        )

    def assert_goal_rejected(
        self,
        action_name: str,
        object_id: int,
        lane_id: str,
    ) -> None:
        """Send one structurally valid goal and require side-effect-free rejection."""
        client = ActionClient(self.node, RestockProduct, action_name)
        try:
            if not client.wait_for_server(timeout_sec=5.0):
                raise AssertionError("replacement coordinator action server is unavailable")
            goal = RestockProduct.Goal()
            goal.has_object_id = True
            goal.object_id = object_id
            goal.has_lane_id = True
            goal.lane_id = lane_id
            sent = client.send_goal_async(goal)
            # Created here, so the first round trip pays for discovery on top of the rejection.
            # The test checks that the replacement refuses the goal, not discovery speed.
            rclpy.spin_until_future_complete(self.node, sent, timeout_sec=5.0)
            if not sent.done() or sent.result() is None:
                raise AssertionError("replacement goal request did not complete")
            if sent.result().accepted:
                raise AssertionError("replacement accepted work over an orphaned reservation")
        finally:
            client.destroy()

    def reservation_matches(
        self,
        snapshot,
        expected: CapturedReservation,
    ) -> bool:
        """Compare every public reservation identity and lineage field."""
        return (
            snapshot.has_active_reservation
            and CapturedReservation.from_message(snapshot.active_reservation) == expected
        )

    def _record_coordinator_status(self, message) -> None:
        self._coordinator_status = message

    def _record_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        self._feedback_trace.append((feedback.state, feedback.detail))

    def _service_has_type(self, name: str, service_type: str) -> bool:
        services = dict(self.node.get_service_names_and_types())
        return service_type in services.get(name, [])
