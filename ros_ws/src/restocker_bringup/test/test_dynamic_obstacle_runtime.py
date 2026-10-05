# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
#
# SPDX-License-Identifier: MIT

"""Acceptance: an unmodeled obstacle changes what may be executed."""
# Two criteria, in one run because they share an expensive startup.
#
# 1. An unexpected obstacle changes the planned path. The test records the trajectory of a real
#    transfer, spawns a box into the corridor it crossed, and checks that MoveIt now refuses a
#    configuration the robot occupied, naming the obstacle. It does not compare two sampled paths:
#    RRTConnect is stochastic, so plans differ with or without an obstacle.
# 2. Stale geometry refuses execution. With the obstacle evidence stopped, the projector degrades,
#    the geometry stays in the scene, and the next transfer is refused with the joints unmoved. A
#    stale observation means the sensor cannot see, not that the corridor is clear.
#
# The test publishes the obstacle observations itself and pins `obstacle_perception` off, so it
# is /perception/obstacle_observations' only publisher: `depth_obstacle_node` produces the same
# message from a depth image and has its own tests, and a second publisher here would keep the
# evidence this test stops from ever going stale (ADR 0013's single-publisher discipline).

from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

from geometry_msgs.msg import Pose
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import launch_testing
import launch_testing.actions
from moveit_msgs.msg import CollisionObject, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetStateValidity
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import rclpy.time
from restocker_interfaces.action import RestockProduct
from restocker_interfaces.msg import (
    ObstacleBox,
    ObstacleObservation,
    PlanningSceneProjectionStatus,
    RestockCoordinatorStatus,
    ShelfLane,
)
from restocker_interfaces.srv import GetWorldState
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
import tf2_ros

# A slim column: tall enough to matter, narrow enough for the arm to route around.
OBSTACLE_SIZE = (0.15, 0.15, 0.50)
OBSTACLE_MODEL = "restocker_dynamic_obstacle"
OBSTACLE_ID_PREFIX = "restocker/obstacle/"
OBSTACLE_TOPIC = "/perception/obstacle_observations"
# Traverse corridor in the world frame (workcell_geometry.yaml, workcell at y = 0.55): the shelf
# front face is at y = 0.10 and the stock tray reaches y = -0.525, so an obstacle between them
# obstructs the traverse without standing on modelled geometry. The z bound keeps it off the rail.
CORRIDOR_Y_RANGE = (-0.45, 0.05)
CORRIDOR_MIN_Z = 0.35
# Lane centre of the transfer that must still complete (workcell_geometry.yaml), and the minimum
# rail distance of the obstacle from it. Approach and insert are straight lines with no room to
# route around, so an obstacle in the corridor near that lane's rail position blocks the next
# transfer; far enough along the rail it is in the corridor without being in the way.
OBSTRUCTED_LANE_CENTER_X_M = -0.6
OBSTACLE_RAIL_CLEARANCE_M = 0.30
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
# Transfers, matching the fixed baseline scenario: the first records the baseline trajectory, the
# second must still complete with the obstacle present, the third must be refused.
BASELINE_TRANSFER = ("sim:stock_can_01", ShelfLane.PRODUCT_CLASS_CAN, "lane_01")
OBSTRUCTED_TRANSFER = (
    "sim:stock_small_bottle_01",
    ShelfLane.PRODUCT_CLASS_SMALL_BOTTLE,
    "lane_02",
)
REFUSED_TRANSFER = ("sim:stock_large_bottle_01", ShelfLane.PRODUCT_CLASS_LARGE_BOTTLE, "lane_03")
# Widest joint-state-to-transform pairing this test accepts when TF publishes one frame behind
# the joint states. Beyond it a recorded tool position is not the one those joints produce, and
# the box would be published against a configuration that was never occupied.
PAIR_SKEW_LIMIT_NS = 50_000_000
# The projector inflates every obstacle box by obstacle_padding_m on each face (0.03 m; the
# value test_obstacle_perception_runtime checks against the projector's configuration).
PROJECTOR_PADDING_M = 0.03
# How far outside the padded box the arm's parked tool must sit for the placement to pass the
# cheap tool0 prefilter. The padded box half-extents already cover the projection; this margin
# covers the wrist assembly, which reaches roughly 0.15 m from tool0, and it is what makes the
# rule fail safe rather than exact. It is a necessary condition only — a box containing tool0
# can never be valid — and it is NOT the placement decision: every candidate that passes it is
# then judged against the parked configuration's full state validity (Card 059; the three red
# receipts cleared this margin by 0.32–0.48 m while colliding with upper_arm_link).
START_STATE_MARGIN_M = 0.15
# Bounds on the pre-spawn full-state check. Distinct candidate boxes are judged at most once
# (centres within this distance share a verdict), and a recording that would need more than
# this many distinct rejections fails closed with ValueError rather than spending minutes of
# service calls.
MAX_STATE_VALIDITY_CHECKS = 512
STATE_VALIDITY_DEDUP_M = 0.05
# Trial box id for the pre-spawn full-state check. Deliberately outside every
# is_projector_managed_id namespace (restocker/workcell|obstacle|lane|declared|product): the
# projector's diff neither adopts nor removes it, so it cannot race the check, and the test
# removes it itself before the real obstacle is spawned.
TRIAL_BOX_ID = "test/dynamic_obstacle/trial_parked_clearance"


def _stamp_ns(stamp):
    """Convert a builtin_interfaces Time to integer nanoseconds for skew arithmetic."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def start_state_clearance(sample_tool, parked_tool):
    """
    Return the distance from the parked tool to the padded box on this sample.

    Negative means the box (the sample's tool position, inflated to the size the projector
    publishes) contains the parked tool; zero means it touches it. The rule compares this to
    START_STATE_MARGIN_M.
    """
    half = [0.5 * size + PROJECTOR_PADDING_M for size in OBSTACLE_SIZE]
    return max(abs(sample_tool[axis] - parked_tool[axis]) - half[axis] for axis in range(3))


def corridor_sample(samples, parked_state_valid):
    """
    Pick the recorded configuration the obstacle box is placed on.

    Restricting to the corridor keeps the box off modelled geometry (a blocked lane or tray is
    already owned by the projector). Two rules then apply, in order:

    1. The box must leave the arm's own start state for the next transfer (the last recorded
       configuration) **fully valid** — `parked_state_valid(centre)` answers, for a padded box
       centred on the sample's tool position, whether that configuration still passes state
       validity for every link, the same `/check_state_validity` question assertion 4 asks. A
       box on it makes MoveIt refuse every grasp candidate before the planner runs, one generic
       FAILURE 99999 per candidate (the refusal SC-003 runs 6 and 7 recorded, Card 043). The
       tool0 START_STATE_MARGIN_M clearance is only the cheap necessary-condition prefilter
       ahead of that check — the tool0 metric alone is not enough (Card 059: the three red
       receipts cleared it by 0.32–0.48 m while colliding with upper_arm_link).
    2. Among the samples that pass both, take the one furthest along the rail from the next
       transfer's lane, at least OBSTACLE_RAIL_CLEARANCE_M away, so the obstacle lies on the
       baseline trajectory but clear of the next transfer's approach.

    When no sample reaches the full rail clearance, the furthest full-state-clear sample is
    used and the shortfall is printed: in that case every rail-clear sample sits on the arm's
    own start state, and refusing to place anything would fail the acceptance for a reason the
    criteria do not name.
    Raises ValueError when the recording offers no placement at all, which says so instead of
    spawning something else.
    """
    in_corridor = [
        sample
        for sample in samples
        if CORRIDOR_Y_RANGE[0] <= sample[2][1] <= CORRIDOR_Y_RANGE[1]
        and sample[2][2] >= CORRIDOR_MIN_Z
    ]
    if not in_corridor:
        raise ValueError(
            "the recorded transfer never put the tool in the traverse corridor, so there is "
            "nowhere to put an obstacle that obstructs it"
        )
    parked_tool = samples[-1][2]
    clears_start_state = [
        sample
        for sample in in_corridor
        if start_state_clearance(sample[2], parked_tool) >= START_STATE_MARGIN_M
    ]
    if not clears_start_state:
        raise ValueError(
            "every recorded configuration in the corridor would place the obstacle on the arm's "
            f"own start state for the next transfer (parked tool {parked_tool}); the scenario "
            "cannot be built from this recording"
        )

    def rail_key(sample):
        return abs(sample[2][0] - OBSTRUCTED_LANE_CENTER_X_M)

    ordered = sorted(clears_start_state, key=rail_key, reverse=True)
    judged_centres = []
    checks = 0
    rejected = 0
    chosen = None
    capped = False
    for sample in ordered:
        centre = sample[2]
        dedup_m2 = STATE_VALIDITY_DEDUP_M * STATE_VALIDITY_DEDUP_M
        if any(
            sum((a - b) ** 2 for a, b in zip(centre, seen, strict=True)) < dedup_m2
            for seen in judged_centres
        ):
            continue
        if checks >= MAX_STATE_VALIDITY_CHECKS:
            capped = True
            break
        judged_centres.append(centre)
        checks += 1
        if parked_state_valid(centre):
            chosen = sample
            break
        rejected += 1
    if chosen is None and capped:
        raise ValueError(
            f"the first {checks} distinct candidate box(es) that clear the "
            f"{START_STATE_MARGIN_M} m tool0 margin were all refused by full state validity of "
            f"the parked configuration and the {MAX_STATE_VALIDITY_CHECKS}-check cap was "
            "reached with unjudged candidates left; failing closed rather than placing"
        )
    if chosen is None:
        raise ValueError(
            f"every corridor configuration that clears the {START_STATE_MARGIN_M} m tool0 "
            f"margin was refused by full state validity of the parked configuration "
            f"({checks} distinct candidate box(es) judged); the scenario cannot be built "
            "from this recording"
        )
    print(
        f"obstacle placement full-state: {checks} distinct candidate box(es) judged against "
        f"the parked configuration ({rejected} refused) after the {START_STATE_MARGIN_M} m "
        "tool0 prefilter",
        flush=True,
    )
    if rail_key(chosen) < OBSTACLE_RAIL_CLEARANCE_M:
        print(
            "obstacle placement note: no recorded configuration reached the full "
            f"{OBSTACLE_RAIL_CLEARANCE_M} m rail clearance while clearing the arm's start state; "
            f"using the furthest that does clear it ({rail_key(chosen):.4f} m from lane_02)",
            flush=True,
        )
    return chosen


def placement_receipt(samples, chosen, parked):
    """
    Print the placement's geometry as one receipt of what this run recorded.

    It shows what was chosen and how far the box will sit from the arm's own parked start state
    for the next transfer.
    """
    in_corridor = [
        sample
        for sample in samples
        if CORRIDOR_Y_RANGE[0] <= sample[2][1] <= CORRIDOR_Y_RANGE[1]
        and sample[2][2] >= CORRIDOR_MIN_Z
    ]
    parked_tool = parked[2]
    lines = [
        f"obstacle placement: recorded {len(samples)} configurations, "
        f"{len(in_corridor)} in the corridor"
    ]
    ranked = sorted(
        in_corridor,
        key=lambda sample: abs(sample[2][0] - OBSTRUCTED_LANE_CENTER_X_M),
        reverse=True,
    )
    unique = []
    for sample in ranked:
        if any(
            sum((a - b) ** 2 for a, b in zip(sample[2], other[2], strict=True)) < 1.0e-4
            for other in unique
        ):
            continue
        unique.append(sample)
        if len(unique) == 10:
            break
    for rank, sample in enumerate(unique, start=1):
        key = abs(sample[2][0] - OBSTRUCTED_LANE_CENTER_X_M)
        distance = sum((a - b) ** 2 for a, b in zip(sample[2], parked_tool, strict=True)) ** 0.5
        lines.append(
            f"  corridor candidate {rank}: tool=({sample[2][0]:.4f}, {sample[2][1]:.4f}, "
            f"{sample[2][2]:.4f}) rail-clearance={key:.4f} m "
            f"distance-to-parked={distance:.4f} m"
        )
    near_parked = sum(
        1
        for sample in in_corridor
        if sum((a - b) ** 2 for a, b in zip(sample[2], parked_tool, strict=True)) < 0.25
    )
    lines.append(f"  corridor candidates within 0.5 m of the parked tool: {near_parked}")
    lines.append(
        f"  first recorded sample: tool=({samples[0][2][0]:.4f}, {samples[0][2][1]:.4f}, "
        f"{samples[0][2][2]:.4f})"
    )
    chosen_distance = sum((a - b) ** 2 for a, b in zip(chosen[2], parked_tool, strict=True)) ** 0.5
    lines.append(
        f"  chosen: tool=({chosen[2][0]:.4f}, {chosen[2][1]:.4f}, {chosen[2][2]:.4f}) "
        f"distance-to-parked={chosen_distance:.4f} m"
    )
    lines.append(
        f"  parked start state for the next transfer: tool=({parked_tool[0]:.4f}, "
        f"{parked_tool[1]:.4f}, {parked_tool[2]:.4f})"
    )
    return "\n".join(lines)


def dump_placement_samples(samples, parked):
    """
    Write the corridor samples and the parked tool into artifacts/ for the unit test.

    The placement rule is unit-tested against what one run actually recorded; only tool
    positions are needed, because `corridor_sample` reads only sample[2]. Returns the path,
    or None when the directory is unwritable.
    """
    import json

    corridor = [
        sample[2]
        for sample in samples
        if CORRIDOR_Y_RANGE[0] <= sample[2][1] <= CORRIDOR_Y_RANGE[1]
        and sample[2][2] >= CORRIDOR_MIN_Z
    ]
    payload = {
        "parked_tool": parked[2],
        "corridor_samples": [list(tool) for tool in corridor],
        "recorded": len(samples),
        "obstructed_lane_center_x_m": OBSTRUCTED_LANE_CENTER_X_M,
        "obstacle_rail_clearance_m": OBSTACLE_RAIL_CLEARANCE_M,
        "corridor_y_range": list(CORRIDOR_Y_RANGE),
        "corridor_min_z": CORRIDOR_MIN_Z,
    }
    try:
        directory = Path(__file__).resolve().parents[4] / "artifacts"
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"placement-samples-{int(time.time())}.json"
        path.write_text(json.dumps(payload), encoding="utf-8")
        return path
    except OSError:
        return None


@pytest.mark.launch_test
def generate_test_description():
    """Start the baseline with manipulation composed and the scene projector certifying it."""
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
            # This test owns /perception/obstacle_observations. With the production default on,
            # depth_obstacle_node would be a second publisher whose frames stay fresh, so the
            # evidence stopped at step 6 could never go stale. It also turns off
            # require_obstacle_evidence, which is what the scene-certified-before-any-observation
            # check needs: once observations have arrived, staleness is judged on their age
            # either way.
            "obstacle_perception": "false",
            "controller_timeout": "30.0",
        }.items(),
    )
    return (
        launch.LaunchDescription([baseline, launch_testing.actions.ReadyToTest()]),
        {},
    )


class TestDynamicObstacleRuntime(unittest.TestCase):
    """Drive transfers around a spawned obstacle, then take the evidence away."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # The projector judges observation age against simulator time, so use the same clock.
        cls.node = rclpy.create_node(
            "dynamic_obstacle_runtime_test",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        cls.world_state = cls.node.create_client(GetWorldState, "/world_state/get_snapshot")
        cls.planning_scene = cls.node.create_client(GetPlanningScene, "/get_planning_scene")
        cls.state_validity = cls.node.create_client(GetStateValidity, "/check_state_validity")
        cls.apply_scene = cls.node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        cls.action_client = ActionClient(cls.node, RestockProduct, "/restock_product")
        cls.obstacle_publisher = cls.node.create_publisher(ObstacleObservation, OBSTACLE_TOPIC, 10)
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
        cls.tf_buffer = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf_buffer, cls.node)
        # Set once the obstacle exists; the timer republishes it. Clearing it stops the evidence.
        cls.published_obstacle = None
        cls.obstacle_sequence = 0
        cls.node.create_timer(0.2, cls._publish_obstacle)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    @classmethod
    def _on_projection_status(cls, message):
        cls.projection_status = message

    @classmethod
    def _on_coordinator_status(cls, message):
        cls.coordinator_status = message

    @classmethod
    def _on_joint_state(cls, message):
        cls.joint_state = message

    @classmethod
    def _publish_obstacle(cls):
        if cls.published_obstacle is None:
            return
        centre, size = cls.published_obstacle
        message = ObstacleObservation()
        message.header.stamp = cls.node.get_clock().now().to_msg()
        message.header.frame_id = "world"
        message.sensor_frame = "overhead_camera_optical_frame"
        cls.obstacle_sequence += 1
        message.sequence = cls.obstacle_sequence
        message.robot_static = True
        box = ObstacleBox()
        box.center.x, box.center.y, box.center.z = centre
        box.size.x, box.size.y, box.size.z = size
        box.point_count = 900
        message.boxes.append(box)
        cls.obstacle_publisher.publish(message)

    def _spin_until(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
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

    def _world_objects(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self._call(self.planning_scene, request).scene
        return {obj.id: obj for obj in scene.world.collision_objects}

    def _obstacle_objects(self):
        return {
            name: obj
            for name, obj in self._world_objects().items()
            if name.startswith(OBSTACLE_ID_PREFIX)
        }

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

    def _resolve(self, snapshot, transfer):
        source_object_id, product_class, lane_id = transfer
        observed = {tracked.source_object_id: tracked for tracked in snapshot.objects}
        self.assertIn(source_object_id, observed, f"{source_object_id} is not in the world state")
        lanes = {lane.id: lane for lane in snapshot.lanes}
        self.assertIn(lane_id, lanes, f"{lane_id} is not in the world state")
        self.assertEqual(lanes[lane_id].expected_product_class, product_class)
        return observed[source_object_id].id, lane_id

    def _record_traversed_configurations(self, samples):
        """
        Append the current joint state paired with the tool pose those joints put it at.

        The transform is looked up at the joint state's own stamp. A "latest" lookup pairs joints
        from one instant with a tool pose from another, and under load that skew moves the
        recorded tool position further than the obstacle's own half-width — which is how the box
        came to be published against a configuration the arm did not actually occupy (Card 023's
        open item, measured as the step-5 placement flake). A newest transform within
        PAIR_SKEW_LIMIT_NS of the joint stamp is accepted when the exact stamp is not in the
        buffer yet (TF may publish one frame behind the joint states); anything further apart is
        dropped rather than paired.
        """
        if self.joint_state is None:
            return
        stamp = self.joint_state.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            return
        try:
            transform = self.tf_buffer.lookup_transform("world", "tool0", stamp)
        except tf2_ros.TransformException:
            try:
                latest = self.tf_buffer.lookup_transform("world", "tool0", rclpy.time.Time())
            except tf2_ros.TransformException:
                return
            skew = abs(_stamp_ns(latest.header.stamp) - _stamp_ns(stamp))
            if skew > PAIR_SKEW_LIMIT_NS:
                return
            transform = latest
        translation = transform.transform.translation
        samples.append(
            (
                list(self.joint_state.name),
                list(self.joint_state.position),
                (translation.x, translation.y, translation.z),
            )
        )

    def _drive_and_record(self, object_id, lane_id, samples):
        """Run one transfer while sampling the configurations the arm actually occupies."""
        goal = RestockProduct.Goal()
        goal.has_object_id = True
        goal.object_id = object_id
        goal.has_lane_id = True
        goal.lane_id = lane_id
        goal_future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=30.0)
        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle, "the baseline restock goal was never answered")
        self.assertTrue(goal_handle.accepted, "the coordinator rejected the baseline restock goal")
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + 450.0
        while time.monotonic() < deadline and not result_future.done():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self._record_traversed_configurations(samples)
        self.assertTrue(result_future.done(), "the baseline transfer never produced a result")
        result = result_future.result().result
        self.assertEqual(
            result.status,
            RestockProduct.Result.STATUS_SUCCEEDED,
            f"the baseline transfer failed: status={result.status} detail={result.detail}",
        )

    def _corridor_sample(self, samples, parked):
        """
        Pick a recorded configuration whose tool sits in the traverse corridor.

        Delegates to the module-level `corridor_sample` so the placement rule is unit-testable
        against recorded samples without a simulator; the full-state predicate is this test's
        own `/check_state_validity` answer for a trial padded box (Card 059).
        """
        chosen = corridor_sample(samples, lambda centre: self._parked_state_valid(parked, centre))
        # The trial id is outside the projector-managed namespaces, so _obstacle_objects()
        # would never notice a lingering one; check the whole world before anything spawns.
        self.assertNotIn(
            TRIAL_BOX_ID,
            self._world_objects(),
            "the pre-spawn full-state trial box is still in the planning scene",
        )
        return chosen

    def _apply_collision_object(self, box, check=True):
        """
        Add or remove one world collision object through /apply_planning_scene.

        check=False sends it without waiting or asserting, for cleanup while another error is
        already propagating (an assertion there would mask the original failure).
        """
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.robot_state.is_diff = True
        request.scene.world.collision_objects.append(box)
        if not check:
            self.apply_scene.call_async(request)
            return
        response = self._call(self.apply_scene, request)
        self.assertTrue(response.success, f"applying trial collision object {box.id} was refused")

    def _parked_state_valid(self, parked, centre):
        """
        Full-state answer for a padded obstacle box centred at `centre`.

        The box is the size and place the projector will publish (obstacle size + padding on
        each face), applied under the unmanaged trial id so the projector neither adopts nor
        deletes it mid-check, then asked against the parked joint state with the same
        `/check_state_validity` call assertion 4 uses — every link, not a tool0 proxy.
        """
        box = CollisionObject()
        box.header.frame_id = "world"
        box.id = TRIAL_BOX_ID
        box.pose.orientation.w = 1.0
        box.pose.position.x, box.pose.position.y, box.pose.position.z = centre
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size + 2.0 * PROJECTOR_PADDING_M for size in OBSTACLE_SIZE]
        box.primitives.append(primitive)
        primitive_pose = Pose()
        primitive_pose.orientation.w = 1.0
        box.primitive_poses.append(primitive_pose)
        box.operation = CollisionObject.ADD
        try:
            self._apply_collision_object(box)
            valid = self._state_validity(parked[0], parked[1]).valid
        finally:
            removal = CollisionObject()
            removal.header.frame_id = box.header.frame_id
            removal.id = TRIAL_BOX_ID
            removal.operation = CollisionObject.REMOVE
            # Assert on the removal only when nothing is already propagating; either way
            # _corridor_sample confirms the trial box is gone before anything spawns.
            self._apply_collision_object(removal, check=sys.exc_info()[0] is None)
        return valid

    def _spawn_obstacle(self, centre):
        """Spawn the obstacle into the running world through ros_gz_sim's create node."""
        half_height = 0.5 * OBSTACLE_SIZE[2]
        size = " ".join(str(value) for value in OBSTACLE_SIZE)
        # Static, so the box stays where the planning scene puts it.
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
                    f"x:={centre[0]}",
                    "-p",
                    f"y:={centre[1]}",
                    "-p",
                    f"z:={centre[2]}",
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
        # The model origin is the box centre, matching the projector's collision object.
        return (centre[0], centre[1], centre[2]), half_height

    def _state_validity(self, names, positions):
        request = GetStateValidity.Request()
        request.group_name = "manipulator"
        request.robot_state.is_diff = True
        request.robot_state.joint_state.name = names
        request.robot_state.joint_state.position = positions
        return self._call(self.state_validity, request)

    def _manipulator_positions(self):
        self.assertIsNotNone(self.joint_state, "no joint state has been observed")
        lookup = dict(zip(self.joint_state.name, self.joint_state.position, strict=False))
        return [lookup[name] for name in MANIPULATOR_JOINTS if name in lookup]

    def test_unmodeled_obstacle_changes_planning_and_stale_evidence_refuses_execution(self):
        """Check both acceptance criteria against the running system."""
        self._spin_until(
            lambda: self.node.get_clock().now().nanoseconds > 0, 120.0, "simulator clock"
        )
        # The action server answers before the coordinator accepts goals (it is still discovering
        # the backends); wait for published readiness.
        self._spin_until(
            lambda: (
                self.coordinator_status is not None and self.coordinator_status.admission_ready
            ),
            180.0,
            "the coordinator to publish readiness",
        )
        # Readiness does not imply admitted robot telemetry, which selection requires.
        self._spin_until(self._admitted_snapshot, 180.0, "admitted robot telemetry")
        self._spin_until(
            lambda: (
                self.projection_status is not None
                and self.projection_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
            ),
            180.0,
            "the planning-scene projector to certify the scene",
        )
        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=60.0),
            "the restock action server never appeared",
        )
        # Single-publisher admission: this test is the only writer on the observation topic. A
        # second publisher (the production depth node) would both contest it and keep the
        # evidence fresh, so step 6 could never go stale.
        self.assertEqual(
            self.node.count_publishers(OBSTACLE_TOPIC),
            1,
            "the dynamic-obstacle test must own /perception/obstacle_observations alone",
        )
        self.assertEqual(
            self._obstacle_objects(), {}, "the scene carried obstacles before any were observed"
        )

        # 1. Baseline trajectory.
        snapshot = self._snapshot()
        object_id, lane_id = self._resolve(snapshot, BASELINE_TRANSFER)
        samples = []
        self._drive_and_record(object_id, lane_id, samples)
        self.assertGreater(len(samples), 20, "the baseline transfer produced too few samples")

        # 2. Spawn an obstacle on that trajectory — only after the parked configuration's full
        # state validity has accepted the chosen centre (Card 059); assertion 4 below re-asks
        # the same question against the real projected box.
        parked = samples[-1]
        chosen = self._corridor_sample(samples, parked)
        print(placement_receipt(samples, chosen, parked), flush=True)
        dumped = dump_placement_samples(samples, parked)
        if dumped is not None:
            print(f"obstacle placement samples dumped to {dumped}", flush=True)
        names, positions, tool_position = chosen
        centre, _ = self._spawn_obstacle(tool_position)
        type(self).published_obstacle = (centre, OBSTACLE_SIZE)

        # 3. The projector adopts it and certifies the scene again.
        observed = self._spin_until(
            lambda: self._obstacle_objects() or None,
            90.0,
            "the obstacle to reach the planning scene",
        )
        self.assertEqual(len(observed), 1, f"expected exactly one obstacle, got {list(observed)}")
        obstacle = next(iter(observed.values()))
        self.assertAlmostEqual(obstacle.pose.position.x, centre[0], delta=0.05)
        self.assertAlmostEqual(obstacle.pose.position.y, centre[1], delta=0.05)
        self.assertAlmostEqual(obstacle.pose.position.z, centre[2], delta=0.05)
        self._spin_until(
            lambda: (
                self.projection_status.state == PlanningSceneProjectionStatus.STATE_APPLIED
                and self.projection_status.error_code == PlanningSceneProjectionStatus.ERROR_NONE
            ),
            90.0,
            "the projector to certify the scene with the obstacle in it",
        )
        # The projected box is padded past the observation on every face, so the receipt records
        # the padded size and where it sits against the arm's parked start state.
        parked_tool = parked[2]
        scene_size = (
            obstacle.primitives[0].dimensions[0],
            obstacle.primitives[0].dimensions[1],
            obstacle.primitives[0].dimensions[2],
        )
        box_centre = (
            obstacle.pose.position.x,
            obstacle.pose.position.y,
            obstacle.pose.position.z,
        )
        clearance = max(
            abs(parked_tool[axis] - box_centre[axis]) - 0.5 * scene_size[axis] for axis in range(3)
        )
        print(
            f"obstacle in scene: centre=({box_centre[0]:.4f}, {box_centre[1]:.4f}, "
            f"{box_centre[2]:.4f}) padded size=({scene_size[0]:.4f}, {scene_size[1]:.4f}, "
            f"{scene_size[2]:.4f}) from observed {OBSTACLE_SIZE}; parked tool "
            f"({parked_tool[0]:.4f}, {parked_tool[1]:.4f}, {parked_tool[2]:.4f}) sits "
            f"{clearance:.4f} m outside the padded box (negative = inside)",
            flush=True,
        )

        # 4. The configuration the next transfer must *start* from is still valid. The obstacle
        # goes on a configuration the arm occupied during the baseline transfer; on the state it
        # still holds, every candidate of the next transfer fails on the same start-state
        # collision — MoveIt's CheckStartStateCollision adapter aborts the pipeline as generic
        # FAILURE 99999, the candidates are consumed one by one, and recovery has nothing left.
        # That is a scenario the planner must refuse, not one it can route around, so the
        # placement rules must never build it.
        parked_validity = self._state_validity(parked[0], parked[1])
        parked_blamed = {
            body
            for contact in parked_validity.contacts
            for body in (contact.contact_body_1, contact.contact_body_2)
        }
        self.assertTrue(
            parked_validity.valid,
            "the obstacle was placed on the arm's own start state for the next transfer: parked "
            f"tool {parked[2]} against obstacle centre {centre} (padded clearance "
            f"{clearance:.4f} m); contacts: {sorted(parked_blamed)}",
        )

        # 5. A configuration the robot occupied is now refused, with the obstacle named.
        validity = self._state_validity(names, positions)
        self.assertFalse(
            validity.valid,
            "a configuration the arm previously occupied is still valid with the obstacle in it",
        )
        blamed = {
            body
            for contact in validity.contacts
            for body in (contact.contact_body_1, contact.contact_body_2)
        }
        self.assertTrue(
            any(body.startswith(OBSTACLE_ID_PREFIX) for body in blamed),
            f"the obstacle was not named among the reported contacts: {sorted(blamed)}",
        )

        # 6. Transfers still complete around it.
        object_id, lane_id = self._resolve(self._snapshot(), OBSTRUCTED_TRANSFER)
        result = self._restock(object_id, lane_id)
        self.assertIsNotNone(result, "the obstructed transfer produced no result")
        self.assertEqual(
            result.status,
            RestockProduct.Result.STATUS_SUCCEEDED,
            f"the transfer did not complete with the obstacle present: detail={result.detail}",
        )

        # 7. Stop the evidence. The projector must degrade and keep the geometry.
        type(self).published_obstacle = None
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

        # 8. The next transfer is refused and the arm does not move.
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
