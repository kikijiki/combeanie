# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Acceptance for the composed depth obstacle path, from the camera to the motion gate."""
# test_dynamic_obstacle_runtime.py publishes obstacle observations by hand; this test covers the
# half in front of the message, composing the pipeline as the launch file does with the overhead
# camera and following one obstacle through:
#
#   camera -> depth_obstacle_node -> ObstacleObservation -> projector -> planning scene -> gate
#
# `obstacle_perception` also turns on `require_obstacle_evidence`, so the scene is certified only
# while depth evidence keeps arriving. Step 3 certifies because the pipeline feeds the projector,
# and step 7 refuses because step 6 stopped it.
#
# No transfer is driven to completion: depth sim-time gaps on this composition run p50 0.166 s,
# p95 0.498 s, max 2.157 s, and obstacle_max_age_sec is 2.0, so one long gap latches
# `motion inhibited; operator required`. Step 3 is the positive control instead: the same gate
# was open on the same evidence before step 6.
#
# Known limit: an obstacle very close to the arm can be partly removed by the self-filter (its
# capsules are tight but not zero-radius). The obstacle stands at a standoff where that does not
# bite. Shrinking the filter further would delete real obstacles.

import os
from pathlib import Path
import signal
import subprocess
import tempfile
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import (
    ObstacleObservation,
    PlanningSceneProjectionStatus,
    RestockCoordinatorStatus,
    ShelfLane,
)
from restocker_interfaces.srv import GetWorldState
from sensor_msgs.msg import JointState

# The obstacle configuration the capsule self-filter was measured at (see the measurement table
# in restocker_bringup/launch/baseline.launch.py): recovered as one full-height box with every
# face within 0.015 m.
OBSTACLE_SIZE = (0.15, 0.15, 0.50)
OBSTACLE_CENTER = (0.60, -0.20, 0.60)
OBSTACLE_MODEL = "restocker_dynamic_obstacle"
OBSTACLE_ID_PREFIX = "restocker/obstacle/"
OBSTACLE_TOPIC = "/perception/obstacle_observations"
# Recovery tolerances. Recorded face error is 0.03 m at the base and 0.015 m elsewhere.
CENTER_TOLERANCE_M = 0.05
SIZE_TOLERANCE_M = 0.05
# Depth returns the recovered box must rest on; a 0.15 x 0.50 m face at 2.5 m fills far more.
MIN_OBSTACLE_POINTS = 100
# `max_boxes` is 8. One obstacle must cost one box, or bands would fill the cap.
MAX_BOXES = 8
# Observations the empty corridor must stay empty across (one per depth frame, 0.166 s apart in
# simulator time, so ten simulator seconds).
EMPTY_CORRIDOR_OBSERVATIONS = 60
# Deep enough that no burst is lost during a service call or action; the default of 10 is two
# seconds of this topic.
OBSERVATION_QUEUE_DEPTH = 500
# Joints MoveIt's "manipulator" group owns, in the order the SRDF home state lists them.
MANIPULATOR_JOINTS = (
    "rail_joint",
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
# The transfer that must be refused. lane_01 is centred at -1.0, 1.6 m along the rail from the
# obstacle, so the refusal comes from stale evidence, not from the obstacle blocking the way.
REFUSED_TRANSFER = ("sim:stock_can_01", ShelfLane.PRODUCT_CLASS_CAN, "lane_01")


def _descendant_pids(name_fragment):
    """Return pids of this process's descendants whose command line names the fragment."""
    # Descendants only: pkill -f would also reach another checkout's simulation.
    parents = {}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
        except OSError:
            continue
        # The executable name (second field) may contain spaces and parentheses, so parse after
        # the last ")".
        parents[int(entry.name)] = int(stat.rpartition(")")[2].split()[1])
    matched = []
    for pid in parents:
        walker = parents.get(pid, 0)
        while walker > 1 and walker != os.getpid():
            walker = parents.get(walker, 0)
        if walker != os.getpid():
            continue
        try:
            cmdline = Path(f"/proc/{pid}/cmdline").read_text(encoding="utf-8")
        except OSError:
            continue
        if name_fragment in cmdline:
            matched.append(pid)
    return matched


@pytest.mark.launch_test
def generate_test_description():
    """Start the baseline with the depth obstacle pipeline composed into it."""
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
            "planning_smoke": "false",
            "motion_enabled": "true",
            "planning_scene_projection": "true",
            # Pins the production default: starts depth_obstacle_node as this topic's sole
            # publisher and puts the projector in require_obstacle_evidence mode.
            "obstacle_perception": "true",
            "controller_timeout": "30.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestObstaclePerceptionRuntime(unittest.TestCase):
    """Follow one real obstacle from the depth stream to a refused transfer."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Wall time, not simulator time: this test only reads and never compares stamps, and
        # /clock would add 1000 messages a second to the executor that receives the 3.5 Hz topic.
        cls.node = rclpy.create_node("obstacle_perception_runtime_test")
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.planning_scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.action_client = ActionClient(cls.node, RestockProduct, "/restock_product")
        # Keep every observation: the assertions below are about consecutive frames.
        cls.observations = []
        cls.node.create_subscription(
            ObstacleObservation, OBSTACLE_TOPIC, cls._on_observation, OBSERVATION_QUEUE_DEPTH
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        cls.projection_status = None
        cls.node.create_subscription(
            PlanningSceneProjectionStatus,
            "/planning_scene_projection/status",
            cls._on_projection_status,
            status_qos,
        )
        cls.coordinator_status = None
        cls.node.create_subscription(
            RestockCoordinatorStatus,
            "/restock_action_coordinator/status",
            cls._on_coordinator_status,
            status_qos,
        )
        cls.joint_state = None
        cls.node.create_subscription(JointState, "/joint_states", cls._on_joint_state, 10)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_observation(cls, message):
        cls.observations.append(message)

    @classmethod
    def _on_projection_status(cls, message):
        cls.projection_status = message

    @classmethod
    def _on_coordinator_status(cls, message):
        cls.coordinator_status = message

    @classmethod
    def _on_joint_state(cls, message):
        cls.joint_state = message

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            # Drain ready callbacks before each check: /joint_states arrives at 100 Hz, and one
            # callback per poll would drop the low-rate topic this test counts.
            drain_deadline = time.monotonic() + 0.05
            while time.monotonic() < drain_deadline:
                rclpy.spin_once(self.node, timeout_sec=0.001)
            result = predicate()
            if result:
                return result
        self.fail(f"timed out waiting for {description}")

    def _call(self, client, request, timeout_sec=15.0):
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            f"{client.srv_name} never became available",
        )
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), f"{client.srv_name} did not answer")
        return future.result()

    def _snapshot(self):
        return self._call(self.world_state, GetWorldState.Request()).snapshot

    def _admitted_snapshot(self):
        snapshot = self._snapshot()
        if snapshot.robot.telemetry_source_id and snapshot.robot.telemetry_revision > 0:
            return snapshot
        return None

    def _obstacle_objects(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self._call(self.planning_scene, request).scene
        return {
            obj.id: obj
            for obj in scene.world.collision_objects
            if obj.id.startswith(OBSTACLE_ID_PREFIX)
        }

    def _collect_observations(self, count, timeout_sec, description):
        """Wait until `count` further observations have arrived and return them."""
        start = len(self.observations)
        self._spin_until(lambda: len(self.observations) >= start + count, timeout_sec, description)
        return self.observations[start:]

    def _resolve(self, snapshot, transfer):
        source_object_id, product_class, lane_id = transfer
        observed = {tracked.source_object_id: tracked for tracked in snapshot.objects}
        self.assertIn(source_object_id, observed, f"{source_object_id} is not in the world state")
        lanes = {lane.id: lane for lane in snapshot.lanes}
        self.assertIn(lane_id, lanes, f"{lane_id} is not in the world state")
        self.assertEqual(lanes[lane_id].expected_product_class, product_class)
        return observed[source_object_id].id, lane_id

    def _restock(self, object_id, lane_id, result_timeout_sec=450.0):
        """Send one addressed transfer and return its result, or None if none arrived."""
        goal = RestockProduct.Goal()
        goal.has_object_id = True
        goal.object_id = object_id
        goal.has_lane_id = True
        goal.lane_id = lane_id
        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, f"the restock goal for {lane_id} was never answered")
        if not goal_handle.accepted:
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=result_timeout_sec)
        if result_future.result() is None:
            return None
        return result_future.result().result

    def _spawn_obstacle(self):
        """Spawn the obstacle column into the running world through ros_gz_sim's create node."""
        size = " ".join(str(value) for value in OBSTACLE_SIZE)
        # Static, so the box stays where the camera sees it.
        model = (
            f'<sdf version="1.10"><model name="{OBSTACLE_MODEL}"><static>true</static>'
            f'<link name="obstacle_link"><collision name="obstacle_collision">'
            f"<geometry><box><size>{size}</size></box></geometry></collision>"
            f'<visual name="obstacle_visual">'
            f"<geometry><box><size>{size}</size></box></geometry>"
            f"<material><ambient>0.6 0.2 0.1 1</ambient><diffuse>0.8 0.3 0.1 1</diffuse>"
            f"</material></visual></link></model></sdf>"
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "obstacle.sdf"
            path.write_text(model, encoding="utf-8")
            completed = subprocess.run(
                [
                    "ros2",
                    "run",
                    "ros_gz_sim",
                    "create",
                    "--ros-args",
                    "-p",
                    "world:=restocking",
                    "-p",
                    f"file:={path}",
                    "-p",
                    f"name:={OBSTACLE_MODEL}",
                    "-p",
                    "allow_renaming:=false",
                    "-p",
                    f"x:={OBSTACLE_CENTER[0]}",
                    "-p",
                    f"y:={OBSTACLE_CENTER[1]}",
                    "-p",
                    f"z:={OBSTACLE_CENTER[2]}",
                ],
                capture_output=True,
                text=True,
                timeout=120,
                check=False,
            )
        self.assertEqual(
            completed.returncode,
            0,
            f"spawning the obstacle failed: {completed.stdout}\n{completed.stderr}",
        )

    def _stop_depth_pipeline(self):
        """Kill this run's depth_obstacle_node so real obstacle evidence stops arriving."""
        pids = _descendant_pids("depth_obstacle_node")
        self.assertTrue(
            pids,
            "no depth_obstacle_node process was found under this test, so obstacle_perception "
            "did not actually start the depth pipeline",
        )
        for pid in pids:
            os.kill(pid, signal.SIGKILL)

    def _manipulator_positions(self):
        self.assertIsNotNone(self.joint_state, "no joint state has been observed")
        lookup = dict(zip(self.joint_state.name, self.joint_state.position, strict=False))
        return [lookup[name] for name in MANIPULATOR_JOINTS if name in lookup]

    def test_real_depth_recovers_an_obstacle_and_losing_it_refuses_execution(self):
        """Prove the composed depth pipeline from the camera through to the motion gate."""
        self._spin_until(
            lambda: (
                self.coordinator_status is not None and self.coordinator_status.admission_ready
            ),
            180.0,
            "the coordinator to publish readiness",
        )
        self._spin_until(self._admitted_snapshot, 180.0, "admitted robot telemetry")

        # 1. The depth pipeline is alive: the sequence must advance (freshness needs new frames).
        first = self._collect_observations(
            2, 180.0, "the depth pipeline to publish obstacle observations"
        )
        self.assertGreater(
            first[1].sequence,
            first[0].sequence,
            "the obstacle observation sequence did not advance, so the depth stream is stalled",
        )
        self.assertEqual(first[0].header.frame_id, "world")
        self.assertEqual(first[0].sensor_frame, "overhead_camera_optical_frame")
        # Single-publisher admission: the production depth node is the only writer, so the
        # topic is never contested by a second producer (ADR 0013's single-publisher rule).
        self.assertEqual(
            self.node.count_publishers(OBSTACLE_TOPIC),
            1,
            "depth_obstacle_node must be the sole publisher on the obstacle observation topic",
        )

        # 2. The empty corridor is reported empty (the arm must be filtered out of its own view).
        empty = self._collect_observations(
            EMPTY_CORRIDOR_OBSERVATIONS,
            120.0,
            f"{EMPTY_CORRIDOR_OBSERVATIONS} observations of the empty corridor",
        )
        still = [observation for observation in empty if observation.robot_static]
        self.assertGreater(
            len(still),
            EMPTY_CORRIDOR_OBSERVATIONS // 2,
            "the arm was reported moving while it should have been parked",
        )
        for observation in still:
            self.assertEqual(
                [],
                list(observation.boxes),
                f"the empty corridor produced a false obstacle in observation "
                f"{observation.sequence}",
            )
        self.assertEqual(
            self._obstacle_objects(), {}, "the scene carried obstacles before any were observed"
        )

        # 3. The projector certifies the scene while require_obstacle_evidence is on, which needs
        #    the pipeline to be feeding it.
        self._spin_until(
            lambda: (
                self.projection_status is not None
                and self.projection_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                and self.projection_status.error_code == PlanningSceneProjectionStatus.ERROR_NONE
            ),
            180.0,
            "the projector to certify the scene from real depth evidence",
        )
        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=60.0),
            "the restock action server never appeared",
        )

        # 4. Spawn an obstacle; the depth stream must recover it.
        self._spawn_obstacle()
        recovered = self._spin_until(
            lambda: next(
                (
                    observation
                    for observation in reversed(self.observations)
                    if observation.robot_static and observation.boxes
                ),
                None,
            ),
            120.0,
            "the depth pipeline to recover the spawned obstacle",
        )
        self.assertEqual(
            len(recovered.boxes),
            1,
            f"one obstacle must cost one box, got {len(recovered.boxes)} of a {MAX_BOXES} cap",
        )
        box = recovered.boxes[0]
        for axis, observed, truth in (
            ("x", box.center.x, OBSTACLE_CENTER[0]),
            ("y", box.center.y, OBSTACLE_CENTER[1]),
            ("z", box.center.z, OBSTACLE_CENTER[2]),
        ):
            self.assertAlmostEqual(
                observed,
                truth,
                delta=CENTER_TOLERANCE_M,
                msg=f"the recovered obstacle centre is wrong in {axis}",
            )
        for axis, observed, truth in (
            ("x", box.size.x, OBSTACLE_SIZE[0]),
            ("y", box.size.y, OBSTACLE_SIZE[1]),
            ("z", box.size.z, OBSTACLE_SIZE[2]),
        ):
            self.assertAlmostEqual(
                observed,
                truth,
                delta=SIZE_TOLERANCE_M,
                msg=f"the recovered obstacle extent is wrong in {axis}",
            )
        self.assertGreater(
            box.point_count,
            MIN_OBSTACLE_POINTS,
            "the recovered obstacle rests on too few depth returns to be a solid obstacle",
        )

        # 5. The projector adds it to the planning scene, still certified. Only the centre is
        #    checked: the projector inflates boxes by obstacle_padding_m on each face.
        observed = self._spin_until(
            lambda: self._obstacle_objects() or None,
            90.0,
            "the obstacle to reach the planning scene",
        )
        self.assertEqual(len(observed), 1, f"expected exactly one obstacle, got {list(observed)}")
        obstacle = next(iter(observed.values()))
        for axis, observed_position, truth in (
            ("x", obstacle.pose.position.x, OBSTACLE_CENTER[0]),
            ("y", obstacle.pose.position.y, OBSTACLE_CENTER[1]),
            ("z", obstacle.pose.position.z, OBSTACLE_CENTER[2]),
        ):
            self.assertAlmostEqual(
                observed_position,
                truth,
                delta=CENTER_TOLERANCE_M,
                msg=f"the projected obstacle centre is wrong in {axis}",
            )
        self._spin_until(
            lambda: (
                self.projection_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                and self.projection_status.error_code == PlanningSceneProjectionStatus.ERROR_NONE
            ),
            90.0,
            "the projector to certify the scene with the depth-derived obstacle in it",
        )

        # 6. Stop the depth pipeline. The projector must degrade and keep the geometry: stale
        #    evidence means the sensor cannot see, not that the corridor is clear.
        self._stop_depth_pipeline()
        self._spin_until(
            lambda: (
                self.projection_status.state == PlanningSceneProjectionStatus.STATE_DEGRADED
                and self.projection_status.error_code
                == PlanningSceneProjectionStatus.ERROR_OBSTACLE_EVIDENCE_STALE
            ),
            120.0,
            "the projector to degrade on stale obstacle evidence",
        )
        retained = self._obstacle_objects()
        self.assertEqual(
            len(retained),
            1,
            "the obstacle was dropped from the scene when its evidence went stale",
        )

        # 7. The transfer must be refused and the arm must not move (step 3 showed the gate open).
        object_id, lane_id = self._resolve(self._snapshot(), REFUSED_TRANSFER)
        before = self._manipulator_positions()
        refused = self._restock(object_id, lane_id, result_timeout_sec=180.0)
        after = self._manipulator_positions()
        if refused is not None:
            self.assertNotEqual(
                refused.status,
                RestockProduct.Result.STATUS_SUCCEEDED,
                "a transfer succeeded while the scene geometry could not be certified",
            )
        self.assertEqual(len(before), len(after))
        for name, start, end in zip(MANIPULATOR_JOINTS, before, after, strict=False):
            self.assertAlmostEqual(
                start,
                end,
                delta=0.01,
                msg=f"{name} moved while the planning scene could not be certified",
            )
