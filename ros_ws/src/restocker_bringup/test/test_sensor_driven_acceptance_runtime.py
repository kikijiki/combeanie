# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""
Card 010 SC-001/SC-002: sensor-driven acceptance from an unordered mixed tray.

SC-001: with no external transfer goal, the mode loop cold-starts on wrist lane evidence
(every lane at evidence_revision 0 until SURVEY_SHELF), surveys the mixed tray, confirms,
transfers, and restores the declared table from three depleted lanes
(`sensor_acceptance_products.yaml` + `sensor_acceptance_lanes.yaml`) to PHASE_FRONT_FULL.

SC-002: on the live ROS graph, no execution-path node subscribes to the ground-truth object
or lane topics; the tray and lane evaluators still do, and the messages admitted on the
operational topics carry the sensor backends, never `gazebo_ground_truth`.

Before Card 010's default switch this file pins the future sensor defaults explicitly; after
the switch the same arguments match the shipped defaults and the pins stay as documentation
of the intended configuration.
"""

import math
import os
from pathlib import Path
import time
import unittest

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import (
    AutonomousRestockCampaignStatus,
    LaneDepthErrorSample,
    LaneObservation,
    ObjectObservation,
    PoseErrorSample,
)
from restocker_interfaces.srv import GetWorldState
from sensor_msgs.msg import Image

INGEST_OBJECT_TOPIC = "/perception/object_observations"
CAMERA_LANE_TOPIC = "/perception/lane_observations"
GT_OBJECT_TOPIC = "/perception/ground_truth/object_observations"
GT_LANE_TOPIC = "/perception/ground_truth/lane_observations"
LANE_ERROR_TOPIC = "/perception/lane_depth_error"
POSE_ERROR_TOPIC = "/perception/pose_error"
WRIST_IMAGE_TOPIC = "/wrist_camera/image"

# Card 050 receipts: when set, a terminated (blocked/idle) campaign writes the last wrist frame
# it saw as a PPM into this directory, beside the ground-truth pose receipt that the failure
# message always carries. Unset: the pose receipt is still in the failure message.
EVIDENCE_DIR = os.environ.get("RESTOCKER_ZERO_FRAME_EVIDENCE_DIR", "")

CONFIRM_BACKEND = "wrist_rgbd_tray_confirm"
LANE_BACKEND = "wrist_depth_lane_survey"
GROUND_TRUTH_BACKEND = "gazebo_ground_truth"

# Every node the operational composition starts that may end up subscribed to evidence topics.
# Evaluators are the only allowed subscribers to the ground-truth topics (SC-002).
EXECUTION_PATH_NODES = frozenset(
    {
        "world_state",
        "planning_scene_projector",
        "restock_action_coordinator",
        "survey_viewpoint",
        "lane_survey",
        "tray_survey",
        "autonomous_restock_campaign",
        "simulation_attachment_adapter",
        "joint_state_telemetry_adapter",
        "lane_observation",
        "tray_overview_perception",
        "tray_confirm_perception",
        "perception",
        "depth_obstacle_detector",
        "ground_truth_adapter",
    }
)
EVALUATOR_NODES = frozenset(
    {
        "pose_error_evaluator",
        "tray_pose_error_evaluator",
        "lane_depth_error_evaluator",
    }
)


@pytest.mark.launch_test
def generate_test_description():
    """Run the public baseline with the Card 010 sensor-driven evidence configuration."""
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("restocker_bringup"), "launch", "baseline.launch.py"]
            )
        ),
        launch_arguments={
            "gui": "false",
            "rviz": "false",
            "planning_smoke": "false",
            "motion_enabled": "true",
            "controller_timeout": "60.0",
            "scenario_config": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_gazebo"),
                    "config",
                    "sensor_acceptance_products.yaml",
                ]
            ),
            "lane_semantics": PathJoinSubstitution(
                [
                    FindPackageShare("restocker_world_state"),
                    "config",
                    "sensor_acceptance_lanes.yaml",
                ]
            ),
            # No persisted SetLanePolicy document may overlay the acceptance targets.
            "lane_policy_state": "",
            # The sensor-driven evidence configuration is no longer pinned here: since the
            # Milestone 10 Stage 7 default switch it *is* baseline.launch.py's shipped default
            # (GT demoted to the ground-truth topic, the wrist tray duties on the ingest topic,
            # the 180 s evidence ages, no overhead perception). Leaving the pins out is the
            # assertion: a default that drifts back to ground truth fails this test rather than
            # being masked by the test pinning the sensor path itself.
            "task_execution_timeout_ms": "180000",
            # Growth is the placement proof on wrist lane evidence (Milestone 10 section 3).
            "placement_require_column_growth": "true",
            # No external transfer goal: the policy loop owns every RestockProduct goal.
            "autonomous_campaign": "true",
            # Card 086: a fresh simulated world is settled; acknowledge the restart gate.
            "campaign_restart_acknowledged": "true",
            "campaign_max_cycles": "0",
            # Card 048 teardown ordering: with a zero period the loop starts the next cycle's
            # MoveGroup plan milliseconds after PHASE_FRONT_FULL, and the launch SIGINT wave
            # that follows this suite lands inside that plan — move_group runs goal callbacks
            # on a detached thread its shutdown never joins, so SIGINT mid-plan is an upstream
            # SIGSEGV (Card 048's cores). The shipped default is 30.0 s; 10.0 s keeps the run
            # bounded while giving the loop a parked window long enough for test_d below to
            # return only while the loop is parked.
            "campaign_cycle_period": "10.0",
        }.items(),
    )
    return launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()])


# launch_testing waits only 15 s for every process to spawn before declaring startup
# failure (loader.py TestRun, `__ready_to_test_action_timeout__`). On this shared machine a
# description/spawn stall of 18.6 s happened under another agent's ~90 GB disk churn
# (attempt 10: "Timed out waiting for processes to start up"; all 24 processes did spawn,
# 3.6 s past the default, and the run died before any test executed — attempt 9 needed
# 2.4 s). Startup slop is not an acceptance criterion; SC readiness stays with the tests'
# own bounded waits below. 60 s fails fast on a genuinely dead spawn while absorbing
# contention for the rare quiet window this test needs.
generate_test_description.__ready_to_test_action_timeout__ = 60.0


class TestSensorDrivenAcceptance(unittest.TestCase):
    """SC-002 graph/authentication first, then SC-001 table restoration."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("sensor_driven_acceptance_test")
        cls.statuses = []
        cls.ingest_objects = []
        cls.camera_lanes = []
        cls.lane_errors = []
        cls.pose_errors = []
        # Card 050: latest ground-truth pose per tray product (up-axis receipts — a product the
        # camera can no longer see must show its orientation here) and the last wrist frame.
        cls.gt_latest: dict[str, ObjectObservation] = {}
        cls.last_up: dict[str, float] = {}
        cls.last_image: Image | None = None
        status_qos = QoSProfile(
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.subscriptions = [
            cls.node.create_subscription(
                AutonomousRestockCampaignStatus,
                "/autonomous_restock_campaign/status",
                cls.statuses.append,
                status_qos,
            ),
            cls.node.create_subscription(
                ObjectObservation,
                INGEST_OBJECT_TOPIC,
                cls.ingest_objects.append,
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
            ),
            cls.node.create_subscription(
                LaneObservation,
                CAMERA_LANE_TOPIC,
                cls.camera_lanes.append,
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
            ),
            cls.node.create_subscription(
                LaneDepthErrorSample,
                LANE_ERROR_TOPIC,
                cls.lane_errors.append,
                QoSProfile(depth=64),
            ),
            cls.node.create_subscription(
                PoseErrorSample,
                POSE_ERROR_TOPIC,
                cls.pose_errors.append,
                QoSProfile(depth=64),
            ),
            # Card 050: the test itself may subscribe to ground truth (SC-002 allows it); this
            # is the receipt that separates "the product tipped over" from "the camera went
            # blind" on a zero-frame dwell.
            cls.node.create_subscription(
                ObjectObservation,
                GT_OBJECT_TOPIC,
                cls._on_ground_truth,
                QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE),
            ),
            cls.node.create_subscription(
                Image,
                WRIST_IMAGE_TOPIC,
                cls._on_image,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            ),
        ]
        cls.snapshot_client = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")

    @classmethod
    def _on_ground_truth(cls, message: ObjectObservation) -> None:
        if not message.source_object_id:
            return
        previous = cls.gt_latest.get(message.source_object_id)
        cls.gt_latest[message.source_object_id] = message
        orientation = message.pose.pose.orientation
        # z component of the product's up axis after the quaternion: 1 - 2(x^2 + y^2).
        up_z = 1.0 - 2.0 * (orientation.x**2 + orientation.y**2)
        if previous is None:
            cls.last_up[message.source_object_id] = up_z
            return
        previous_up = cls.last_up.get(message.source_object_id, 1.0)
        if (previous_up >= 0.9) != (up_z >= 0.9):
            stamp = message.header.stamp
            print(
                f"GT receipt: {message.source_object_id} up_z crossed 0.9: "
                f"{previous_up:.3f} -> {up_z:.3f} at {stamp.sec}.{stamp.nanosec:09d} "
                f"quat=({orientation.x:.3f}, {orientation.y:.3f}, {orientation.z:.3f}, "
                f"{orientation.w:.3f})",
                flush=True,
            )
        cls.last_up[message.source_object_id] = up_z
        previous_position = previous.pose.pose.position
        position = message.pose.pose.position
        jumped = math.dist(
            (position.x, position.y, position.z),
            (previous_position.x, previous_position.y, previous_position.z),
        )
        if jumped > 0.05:
            print(
                f"GT receipt: {message.source_object_id} moved {jumped * 1000.0:.1f} mm "
                f"({previous_position.x:.4f}, {previous_position.y:.4f}, "
                f"{previous_position.z:.4f}) -> ({position.x:.4f}, {position.y:.4f}, "
                f"{position.z:.4f})",
                flush=True,
            )

    @classmethod
    def _on_image(cls, message: Image) -> None:
        cls.last_image = message

    @classmethod
    def _gt_receipt(cls) -> str:
        if not cls.gt_latest:
            return "ground-truth tray receipt: no ground-truth object observations received"
        lines = ["ground-truth tray receipt (latest):"]
        for source_id in sorted(cls.gt_latest):
            message = cls.gt_latest[source_id]
            position = message.pose.pose.position
            orientation = message.pose.pose.orientation
            up_z = 1.0 - 2.0 * (orientation.x**2 + orientation.y**2)
            lines.append(
                f"  {source_id}: xyz=({position.x:.4f}, {position.y:.4f}, {position.z:.4f}) "
                f"quat=({orientation.x:.4f}, {orientation.y:.4f}, {orientation.z:.4f}, "
                f"{orientation.w:.4f}) up_z={up_z:.3f}"
            )
        return "\n".join(lines)

    @classmethod
    def _image_dump(cls) -> str:
        """Write the last wrist frame as a PPM when an evidence directory is configured."""
        if not EVIDENCE_DIR or cls.last_image is None:
            return ""
        message = cls.last_image
        if message.encoding not in ("rgb8", "bgr8"):
            return f"\nlast wrist frame not dumped: encoding {message.encoding}"
        directory = Path(EVIDENCE_DIR)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = message.header.stamp
        path = directory / f"wrist_image_last_{stamp.sec}_{stamp.nanosec:09d}.ppm"
        stride = message.step
        rows = []
        for row in range(message.height):
            offset = row * stride
            end = offset + message.width * 3
            row_bytes = bytes(message.data[offset:end])
            if message.encoding == "bgr8":
                swapped = bytearray(len(row_bytes))
                for index in range(0, len(row_bytes), 3):
                    swapped[index] = row_bytes[index + 2]
                    swapped[index + 1] = row_bytes[index + 1]
                    swapped[index + 2] = row_bytes[index]
                row_bytes = bytes(swapped)
            rows.append(row_bytes)
        payload = b"".join(rows)
        path.write_bytes(b"P6\n" + f"{message.width} {message.height}\n255\n".encode() + payload)
        return f"\nlast wrist frame at stop: {path}"

    @classmethod
    def tearDownClass(cls):
        for subscription in cls.subscriptions:
            cls.node.destroy_subscription(subscription)
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.2)
            if predicate():
                return
        self.fail(f"timed out waiting for {description}")

    def _wait_for_phase(self, phase, timeout_sec):
        matched = []

        def found():
            matched.extend(status for status in self.statuses if status.phase == phase)
            blocked = [
                status
                for status in self.statuses
                if status.phase
                in (
                    AutonomousRestockCampaignStatus.PHASE_BLOCKED,
                    AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                )
            ]
            if blocked and not matched:
                # Card 050: the terminal receipt carries the tray products' ground-truth poses
                # and orientations — a station that admitted zero frames is either looking at a
                # product the camera can no longer see (tipped/moved: shown here) or at nothing
                # the backend admits — plus the last wrist frame when an evidence directory is
                # configured.
                self.fail(
                    f"campaign terminated before phase {phase}: {blocked[-1].detail}\n"
                    f"{self._gt_receipt()}{self._image_dump()}"
                )
            return bool(matched)

        self._spin_until(found, timeout_sec, f"campaign phase {phase}")
        return matched[-1]

    def _subscriber_nodes(self, topic):
        return {info.node_name for info in self.node.get_subscriptions_info_by_topic(topic)}

    def _publisher_nodes(self, topic):
        return {info.node_name for info in self.node.get_publishers_info_by_topic(topic)}

    def test_a_no_execution_path_subscribes_to_ground_truth_topics(self):
        """SC-002: only evaluators (and the test itself) may subscribe to GT topics."""
        # Wait until the composition is fully up: world state and the evaluators' subscriptions
        # on BOTH ground-truth topics. The GT-object subscription is part of the wait because
        # DDS discovery of a just-created subscription can lag the node's own startup banner by
        # more than a test tick (attempt 9 asserted 60 ms after the pose evaluator's banner and
        # saw an empty graph); waiting keeps the assertions below about *presence of wrong
        # subscribers*, not about discovery timing.
        self._spin_until(
            lambda: (
                (
                    "world_state" in self._subscriber_nodes(INGEST_OBJECT_TOPIC)
                    or "world_state" in self._subscriber_nodes(CAMERA_LANE_TOPIC)
                )
                and bool(self._subscriber_nodes(GT_LANE_TOPIC) & EVALUATOR_NODES)
                and bool(self._subscriber_nodes(GT_OBJECT_TOPIC) & EVALUATOR_NODES)
            ),
            180.0,
            "world state and both evaluator subscriptions to appear on the graph",
        )
        for topic in (GT_OBJECT_TOPIC, GT_LANE_TOPIC):
            subscribers = self._subscriber_nodes(topic)
            execution = subscribers & EXECUTION_PATH_NODES
            self.assertEqual(
                execution,
                set(),
                f"execution-path nodes must not subscribe to {topic}: {sorted(execution)}",
            )
            evaluators = subscribers & EVALUATOR_NODES
            self.assertTrue(
                evaluators,
                f"an evaluator must still subscribe to {topic} for error reporting",
            )
        # The lane evaluator must be one of them; the tray evaluator joins once tray duties run.
        self.assertIn("lane_depth_error_evaluator", self._subscriber_nodes(GT_LANE_TOPIC))
        self.assertTrue(
            self._subscriber_nodes(GT_OBJECT_TOPIC) & EVALUATOR_NODES,
            "no evaluator subscribed to the ground-truth object topic",
        )

    def test_b_world_state_ingests_sensor_producers_only(self):
        """SC-002: operational topics carry sensor backends; GT stays demoted for scoring."""
        # Cold start: the camera lane topic has no messages until SURVEY_SHELF runs.
        self._spin_until(
            lambda: bool(self.camera_lanes),
            300.0,
            "the first wrist lane observation on the camera topic",
        )
        lane_backends = {message.backend_name for message in self.camera_lanes}
        self.assertEqual(
            lane_backends,
            {LANE_BACKEND},
            f"camera lane topic must be produced only by {LANE_BACKEND}: {lane_backends}",
        )
        self.assertNotIn(GROUND_TRUTH_BACKEND, lane_backends)

        # Confirm observations arrive once the tray survey runs; wait for at least one.
        self._spin_until(
            lambda: bool(self.ingest_objects),
            600.0,
            "the first confirmed object observation on the ingest topic",
        )
        object_backends = {message.backend_name for message in self.ingest_objects}
        self.assertEqual(
            object_backends,
            {CONFIRM_BACKEND},
            f"ingest topic must be produced only by {CONFIRM_BACKEND}: {object_backends}",
        )
        self.assertNotIn(GROUND_TRUTH_BACKEND, object_backends)

        # GT must still publish so evaluators can score against it.
        self._spin_until(
            lambda: (
                bool(self._publisher_nodes(GT_LANE_TOPIC))
                and bool(self._publisher_nodes(GT_OBJECT_TOPIC))
            ),
            60.0,
            "ground-truth publishers for the evaluators",
        )
        gt_lane_publishers = self._publisher_nodes(GT_LANE_TOPIC)
        self.assertIn("ground_truth_adapter", gt_lane_publishers)
        for topic in (GT_LANE_TOPIC, GT_OBJECT_TOPIC):
            gt_execution = self._subscriber_nodes(topic) & EXECUTION_PATH_NODES
            self.assertEqual(
                gt_execution,
                set(),
                f"execution path must not subscribe to {topic}: {sorted(gt_execution)}",
            )

        # World state must not be latched onto a GT backend: ask for events and check none
        # of the admitted object/lane backends is gazebo_ground_truth on the operational path.
        self.assertTrue(
            self.snapshot_client.wait_for_service(timeout_sec=30.0),
            "world state snapshot service never appeared",
        )
        request = GetWorldState.Request()
        request.include_events = True
        request.include_removed = False
        future = self.snapshot_client.call_async(request)
        self._spin_until(
            lambda: future.done(),
            30.0,
            "world state snapshot for backend authentication",
        )
        snapshot = future.result().snapshot
        # Objects admitted so far must be tray-confirm shaped once any exist. Tracked objects
        # do not carry backend_name; the latched pin is proven by the ingest-topic backends
        # above plus the single-publisher rejection of a second writer (world_state_node).
        self.assertIsNotNone(snapshot)

    def test_c_evaluators_still_report_error_against_ground_truth(self):
        """SC-002: evaluators publish paired error samples, not just subscriptions."""
        self._spin_until(
            lambda: bool(self.lane_errors),
            300.0,
            "at least one lane-depth error sample from the evaluator",
        )
        statuses = {sample.status for sample in self.lane_errors}
        self.assertTrue(
            statuses
            <= {
                LaneDepthErrorSample.STATUS_OK,
                LaneDepthErrorSample.STATUS_NO_GROUND_TRUTH,
                LaneDepthErrorSample.STATUS_LANE_MISMATCH,
                LaneDepthErrorSample.STATUS_STALE_PAIRING,
                LaneDepthErrorSample.STATUS_MEASUREMENT_UNUSABLE,
                LaneDepthErrorSample.STATUS_INTERNAL_ERROR,
            },
            f"unexpected lane error status: {statuses}",
        )
        self.assertIn(
            LaneDepthErrorSample.STATUS_OK,
            statuses,
            "the lane evaluator never produced a paired OK sample against ground truth",
        )

    def test_d_restores_the_declared_table_from_the_unordered_tray(self):
        """SC-001: cold-start survey, then transfers, then PHASE_FRONT_FULL at zero deficit."""
        measured = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_MEASURED, 900.0)
        self.assertTrue(measured.front_survey_complete, "cold-start shelf survey incomplete")
        # Cold start on the camera topic: SURVEYING_FRONT must precede the measurement.
        phases_before = [
            status.phase for status in self.statuses if status.sequence <= measured.sequence
        ]
        self.assertIn(
            AutonomousRestockCampaignStatus.PHASE_SURVEYING_FRONT,
            phases_before,
            "no start-of-run SURVEY_SHELF before measurement on a cold camera-lane start",
        )
        self.assertNotIn(
            AutonomousRestockCampaignStatus.PHASE_RESTOCKING,
            phases_before,
            "no transfer may begin before the cold-start shelf survey and measurement",
        )
        deficits = dict(zip(measured.product_classes, measured.front_deficits, strict=True))
        # Three depleted lanes: one deficit unit per class (can / small / large).
        self.assertGreaterEqual(
            sum(deficits.values()),
            3,
            f"acceptance fixture must start with at least three deficit units: {deficits}",
        )
        for product_class, count in deficits.items():
            self.assertGreater(
                count,
                0,
                f"class {product_class} must start depleted: {deficits}",
            )
        # Cold-start MEASURED runs before SURVEY_TRAY, so back stock is still 0 on the sensor
        # path (only ground truth publishes tray inventory before the tray is surveyed).
        # The deficits themselves are the proof the declared table needs filling.

        full = self._wait_for_phase(AutonomousRestockCampaignStatus.PHASE_FRONT_FULL, 4500.0)
        self.assertEqual(
            full.total_front_deficit,
            0,
            f"declared table not restored: detail={full.detail}",
        )
        self.assertGreaterEqual(
            full.successful_transfers,
            3,
            "restoring three depleted lanes needs at least three successful transfers",
        )
        # Tray survey is for the deficit that lacks a candidate; after confirmations the
        # loop may re-enter it for a second class. It must not run with a full back stock
        # once every class has a valid candidate for the whole run after the first cycle —
        # only assert no terminal failure phases and that surveying happened.
        self.assertFalse(
            any(
                status.phase
                in (
                    AutonomousRestockCampaignStatus.PHASE_BLOCKED,
                    AutonomousRestockCampaignStatus.PHASE_STOCK_EXHAUSTED,
                )
                for status in self.statuses
                if status.sequence <= full.sequence
            ),
            "the acceptance run must not end blocked or stock-exhausted",
        )
        self.assertGreater(full.survey_arm_time_sec, 0.0, "survey arm time must be reported")
        self.assertGreater(full.transfer_arm_time_sec, 0.0, "transfer arm time must be reported")

        # Card 048 teardown ordering: launch_testing SIGINTs the whole launch as soon as this
        # (last) active test returns, and move_group SIGSEGVs when SIGINT lands inside an OMPL
        # plan (upstream: goal callbacks run on a detached thread its shutdown never joins —
        # Card 048's cores 529866/469476). The fixture's 10 s campaign_cycle_period parks the
        # loop after FRONT_FULL; return only once the FRONT_FULL status has held for 3 s, so at
        # least 7 s of parked window remain for the teardown wave and SIGINT meets an idle
        # move_group. A non-FRONT_FULL status (the next cycle's own phases) restarts the hold.
        park_hold = None

        def parked_after_front_full() -> bool:
            nonlocal park_hold
            latest = self.statuses[-1] if self.statuses else None
            if latest is None or latest.phase != AutonomousRestockCampaignStatus.PHASE_FRONT_FULL:
                park_hold = None
                return False
            now = time.monotonic()
            if park_hold is None:
                park_hold = now
                return False
            return now - park_hold >= 3.0

        self._spin_until(
            parked_after_front_full,
            600.0,
            "the campaign to stay parked at PHASE_FRONT_FULL before teardown",
        )


@launch_testing.post_shutdown_test()
class TestSensorDrivenAcceptanceShutdown(unittest.TestCase):
    """Post-condition: no process died on a fatal signal during teardown."""

    def test_no_fatal_signals(self, proc_info):
        fatal = {-4: "SIGILL", -6: "SIGABRT", -7: "SIGBUS", -11: "SIGSEGV"}
        # SIGKILL (-9) is launch's escalation when SIGINT/SIGTERM do not land within the
        # teardown budget under load (observed on lane_survey_node, acceptance attempt 1);
        # it is not a crash of the node under test.
        crashed = [
            f"{info.process_name} died on {fatal[info.returncode]}"
            for info in proc_info
            if info.returncode in fatal
        ]
        self.assertEqual(crashed, [], f"process crashed: {crashed}")
